#!/usr/bin/env python3
"""Gapless CRSF capture: stream an fx2lafw analyzer, decode live, log every frame.

Usage: ./crsf_capture.py --duration 120 --outdir run1
"""

import argparse
import csv
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np

from crsf_decode import (TICK_MAX, TICK_MIN, TYPE_NAMES, T_RC_CHANNELS,
                         CrsfParser, UartDecoder, unpack_channels)

BLOCK = 1 << 20
QUEUE_BLOCKS = 64

stop_flag = threading.Event()


class Stream:
    """Decode + anomaly state for one wire direction."""

    def __init__(self, name, args, rc_w, fr_w, ev):
        self.name = name
        self.args = args
        self.uart = UartDecoder(args.samplerate, args.baud, args.rx if name == "rx" else args.tx)
        self.parser = CrsfParser()
        self.rc_w, self.fr_w, self.ev = rc_w, fr_w, ev
        self.frames = 0
        self.rc_frames = 0
        self.last_rc_sample = None
        self.last_ticks = None
        self.prev_dropped = 0
        self.anomalies = 0

    def _event(self, sample, kind, **detail):
        self.anomalies += 1
        self.ev.write(json.dumps({
            "t_s": round(sample / self.args.samplerate, 6),
            "sample": int(sample), "dir": self.name, "kind": kind, **detail,
        }) + "\n")

    def process(self, raw):
        events = self.uart.feed(raw)
        bad_framing = sum(1 for _, _, ok in events if not ok)
        frames = self.parser.feed(events)

        if self.parser.dropped > self.prev_dropped:
            n = self.parser.dropped - self.prev_dropped
            self.prev_dropped = self.parser.dropped
            ref = frames[0]["sample"] if frames else (events[-1][0] if events else 0)
            self._event(ref, "unparsed_bytes", count=int(n), framing_errors=int(bad_framing))

        sr = self.args.samplerate
        for f in frames:
            self.frames += 1
            t = f["sample"] / sr
            if not f["framing_ok"]:
                self._event(f["sample"], "framing_error", type=hex(f["type"]))

            if f["type"] != T_RC_CHANNELS:
                self.fr_w.writerow([f"{t:.6f}", f["sample"], self.name, hex(f["type"]),
                                    TYPE_NAMES.get(f["type"], "?"), f["length"],
                                    f["payload"].hex()])
                continue

            if len(f["payload"]) < 22:
                self._event(f["sample"], "short_rc_frame", length=f["length"])
                continue

            ticks = unpack_channels(f["payload"])
            self.rc_frames += 1

            if self.last_rc_sample is not None:
                gap_ms = (f["sample"] - self.last_rc_sample) / sr * 1000.0
                if gap_ms > self.args.max_gap_ms:
                    self._event(f["sample"], "frame_gap", gap_ms=round(gap_ms, 3))
            self.last_rc_sample = f["sample"]

            oor = [(i + 1, v) for i, v in enumerate(ticks) if not TICK_MIN <= v <= TICK_MAX]
            if oor:
                self._event(f["sample"], "out_of_range", channels=oor)

            if self.last_ticks is not None:
                jumps = [(i + 1, self.last_ticks[i], v) for i, v in enumerate(ticks)
                         if abs(v - self.last_ticks[i]) > self.args.max_jump]
                if jumps:
                    self._event(f["sample"], "jump", channels=jumps)
            self.last_ticks = ticks

            self.rc_w.writerow([f"{t:.6f}", f["sample"], self.name] + ticks)


def reader_thread(proc, q, raw_file, state):
    while not stop_flag.is_set():
        b = proc.stdout.read(BLOCK)
        if not b:
            break
        state["bytes"] += len(b)
        if raw_file is not None:
            raw_file.write(b)
        try:
            q.put_nowait(b)
        except queue.Full:
            state["lagged"] += 1
        if state["max_bytes"] and state["bytes"] >= state["max_bytes"]:
            state["hit_cap"] = True
            break
    stop_flag.set()
    q.put(None)


def decoder_thread(q, streams):
    while True:
        b = q.get()
        if b is None:
            break
        raw = np.frombuffer(b, dtype=np.uint8)
        for s in streams:
            s.process(raw)


def stderr_thread(proc, log):
    for line in iter(proc.stderr.readline, b""):
        txt = line.decode(errors="replace").rstrip()
        log.write(f"[sigrok] {txt}\n")
        log.flush()
        print(f"\n[sigrok] {txt}", file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, default=60, help="seconds (default 60)")
    p.add_argument("--outdir", default=None)
    p.add_argument("--samplerate", type=float, default=8e6)
    p.add_argument("--baud", type=int, default=420000)
    p.add_argument("--rx", type=int, default=0, help="channel for RX->FC (default D0)")
    p.add_argument("--tx", type=int, default=None, help="channel for FC->RX (default off)")
    p.add_argument("--driver", default="fx2lafw")
    p.add_argument("--conn", default=None)
    p.add_argument("--no-raw", action="store_true", help="skip raw.bin (no PulseView slices)")
    p.add_argument("--max-gb", type=float, default=20.0)
    p.add_argument("--max-gap-ms", type=float, default=25.0)
    p.add_argument("--max-jump", type=int, default=400, help="ticks/frame before flagging")
    args = p.parse_args()

    outdir = args.outdir or datetime.now().strftime("run-%Y%m%d-%H%M%S")
    os.makedirs(outdir, exist_ok=True)

    cmd = ["sigrok-cli", "--driver", args.driver + (f":conn={args.conn}" if args.conn else ""),
           "--config", f"samplerate={int(args.samplerate)}",
           "--time", str(int(args.duration * 1000)), "-O", "binary"]

    meta = {"cmd": cmd, "samplerate": args.samplerate, "baud": args.baud,
            "rx_channel": args.rx, "tx_channel": args.tx,
            "started": datetime.now().isoformat(), "duration_s": args.duration}
    with open(f"{outdir}/meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)

    rc_f = open(f"{outdir}/rc.csv", "w", newline="")
    fr_f = open(f"{outdir}/frames.csv", "w", newline="")
    ev_f = open(f"{outdir}/events.jsonl", "w")
    log_f = open(f"{outdir}/capture.log", "w")
    rc_w, fr_w = csv.writer(rc_f), csv.writer(fr_f)
    rc_w.writerow(["t_s", "sample", "dir"] + [f"ch{i}" for i in range(1, 17)])
    fr_w.writerow(["t_s", "sample", "dir", "type", "type_name", "len", "payload_hex"])

    raw_file = None if args.no_raw else open(f"{outdir}/raw.bin", "wb")

    print(f"-> {outdir}  ({args.samplerate/1e6:g} MHz, {args.baud} baud, "
          f"{args.duration:g}s, ~{args.samplerate*args.duration/1e9:.2f} GB raw)", file=sys.stderr)

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

    streams = [Stream("rx", args, rc_w, fr_w, ev_f)]
    if args.tx is not None:
        streams.append(Stream("tx", args, rc_w, fr_w, ev_f))

    q = queue.Queue(maxsize=QUEUE_BLOCKS)
    state = {"bytes": 0, "lagged": 0, "hit_cap": False,
             "max_bytes": int(args.max_gb * 1e9) if not args.no_raw else 0}

    threads = [threading.Thread(target=reader_thread, args=(proc, q, raw_file, state), daemon=True),
               threading.Thread(target=decoder_thread, args=(q, streams), daemon=True),
               threading.Thread(target=stderr_thread, args=(proc, log_f), daemon=True)]
    for t in threads:
        t.start()

    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())

    t0 = time.monotonic()
    try:
        while not stop_flag.is_set() and threads[0].is_alive():
            time.sleep(0.5)
            el = time.monotonic() - t0
            s = streams[0]
            sys.stderr.write(
                f"\r{el:7.1f}s  {state['bytes']/1e6:8.1f} MB  "
                f"rc={s.rc_frames:6d} ({s.rc_frames/max(el,1e-9):5.1f}/s)  "
                f"anom={s.anomalies:4d}  q={q.qsize():2d}  lag={state['lagged']}   ")
            sys.stderr.flush()
    finally:
        stop_flag.set()
        if proc.poll() is None:
            proc.terminate()
        threads[0].join(timeout=5)
        q.put(None)
        threads[1].join(timeout=30)
        proc.wait(timeout=5)

        for f in (rc_f, fr_f, ev_f, log_f):
            f.close()
        if raw_file:
            raw_file.close()

    el = time.monotonic() - t0
    expected = int(args.samplerate * min(el, args.duration))
    short = expected - state["bytes"]
    print("\n" + "-" * 62, file=sys.stderr)
    for s in streams:
        print(f"{s.name}: {s.frames} frames ({s.rc_frames} RC), "
              f"{s.anomalies} anomalies, {s.parser.dropped} unparsed bytes", file=sys.stderr)
    print(f"raw: {state['bytes']/1e6:.1f} MB in {el:.1f}s "
          f"({state['bytes']/el/1e6:.2f} MB/s), shortfall ~{short/args.samplerate*1000:.0f} ms",
          file=sys.stderr)
    if state["lagged"]:
        print(f"WARNING: decoder lagged on {state['lagged']} block(s); "
              f"raw.bin is complete, re-decode offline", file=sys.stderr)
    if state["hit_cap"]:
        print(f"WARNING: stopped at --max-gb {args.max_gb}", file=sys.stderr)
    print(f"-> {outdir}/rc.csv  events.jsonl  frames.csv", file=sys.stderr)


if __name__ == "__main__":
    main()
