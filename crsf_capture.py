#!/usr/bin/env python3
"""Gapless CRSF capture: stream an fx2lafw analyzer, decode live, log every frame.

Usage: ./crsf_capture.py --duration 120 --outdir run1
"""

import argparse
import csv
import json
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np

from crsf_decode import (TICK_MAX, TICK_MIN, TYPE_NAMES, T_RC_CHANNELS,
                         CrsfParser, UartDecoder, ticks_to_us, unpack_channels)

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
        self.synced = False
        self.last_change = [0] * 16
        self.moved = [False] * 16
        self.stuck_flagged = [False] * 16

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
            # bytes before the first good frame are just the frame in flight when
            # capture started, not a fault
            if self.synced:
                ref = frames[0]["sample"] if frames else (events[-1][0] if events else 0)
                self._event(ref, "unparsed_bytes", count=int(n),
                            framing_errors=int(bad_framing),
                            hex=bytes(self.parser.dropped_bytes).hex())
            self.parser.dropped_bytes.clear()
        if frames:
            self.synced = True

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

            # a channel that stops moving while the others carry on is the
            # hypothesis itself; a channel that never moves is deliberate (ARM)
            if self.last_ticks is not None:
                changed = [i for i in range(16) if ticks[i] != self.last_ticks[i]]
                for i in changed:
                    self.moved[i] = True
                    self.last_change[i] = f["sample"]
                    self.stuck_flagged[i] = False
                if changed:
                    for i in range(16):
                        if self.moved[i] and not self.stuck_flagged[i]:
                            held = (f["sample"] - self.last_change[i]) / sr
                            if held >= self.args.stuck_s:
                                self._event(f["sample"], "stuck", channel=i + 1,
                                            seconds=round(held, 3),
                                            value=int(ticks[i]))
                                self.stuck_flagged[i] = True
            self.last_ticks = ticks

            self.rc_w.writerow([f"{t:.6f}", f["sample"], self.name] + ticks)


def status_line(el, state, s, show, width):
    ticks = s.last_ticks
    if ticks is None:
        chans = " ".join(["----"] * 16)
    elif show == "us":
        chans = " ".join(f"{round(ticks_to_us(v)):4d}" for v in ticks)
    else:
        chans = " ".join(f"{v:4d}" for v in ticks)

    line = (f"{el:6.1f}s rc={s.rc_frames:6d} {s.rc_frames/max(el, 1e-9):5.1f}/s "
            f"a={s.anomalies:3d} lag={state['lagged']} | {chans}")
    # channels are the point of this line; drop the stats before truncating them
    if len(line) >= width:
        line = chans
    return line[:width - 1]


def reader_thread(src, q, raw_file, state):
    while not stop_flag.is_set():
        b = src.read(BLOCK)
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
    p.add_argument("--raw", action="store_true",
                   help="also write raw.bin (0.48 GB/min; needed for crsf_slice.py)")
    p.add_argument("--show", choices=("us", "ticks"), default="us",
                   help="units for the live channel display (default us)")
    p.add_argument("--from-raw", default=None,
                   help="re-decode an existing raw.bin instead of capturing")
    p.add_argument("--max-gb", type=float, default=20.0)
    p.add_argument("--max-gap-ms", type=float, default=25.0)
    p.add_argument("--stuck-s", type=float, default=1.0,
                   help="flag a channel held this long while others move (default 1.0)")
    args = p.parse_args()

    outdir = args.outdir or datetime.now().strftime("run-%Y%m%d-%H%M%S")
    os.makedirs(outdir, exist_ok=True)

    offline = args.from_raw is not None
    cmd = None if offline else [
        "sigrok-cli", "--driver", args.driver + (f":conn={args.conn}" if args.conn else ""),
        "--config", f"samplerate={int(args.samplerate)}",
        "--time", str(int(args.duration * 1000)), "-O", "binary"]

    meta = {"cmd": cmd, "source": args.from_raw, "samplerate": args.samplerate, "baud": args.baud,
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

    keep_raw = args.raw and not offline
    raw_file = open(f"{outdir}/raw.bin", "wb") if keep_raw else None

    if offline:
        n = os.path.getsize(args.from_raw)
        print(f"-> {outdir}  re-decoding {args.from_raw} "
              f"({n/1e6:.1f} MB = {n/args.samplerate:.2f}s)", file=sys.stderr)
        proc = None
        src = open(args.from_raw, "rb")
    else:
        raw_note = (f"~{args.samplerate*args.duration/1e9:.2f} GB raw"
                    if keep_raw else "no raw (--raw to keep)")
        print(f"-> {outdir}  ({args.samplerate/1e6:g} MHz, {args.baud} baud, "
              f"{args.duration:g}s, {raw_note})", file=sys.stderr)
        assert cmd is not None
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        src = proc.stdout

    streams = [Stream("rx", args, rc_w, fr_w, ev_f)]
    if args.tx is not None:
        streams.append(Stream("tx", args, rc_w, fr_w, ev_f))

    q = queue.Queue(maxsize=QUEUE_BLOCKS)
    state = {"bytes": 0, "lagged": 0, "hit_cap": False,
             "max_bytes": int(args.max_gb * 1e9) if keep_raw else 0}

    threads = [threading.Thread(target=reader_thread, args=(src, q, raw_file, state), daemon=True),
               threading.Thread(target=decoder_thread, args=(q, streams), daemon=True)]
    if proc is not None:
        threads.append(threading.Thread(target=stderr_thread, args=(proc, log_f), daemon=True))
    for t in threads:
        t.start()

    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())

    t0 = time.monotonic()
    try:
        prev = 0
        while not stop_flag.is_set() and threads[0].is_alive():
            time.sleep(0.2)
            width = shutil.get_terminal_size((160, 24)).columns
            line = status_line(time.monotonic() - t0, state, streams[0], args.show, width)
            sys.stderr.write("\r" + line.ljust(prev))
            sys.stderr.flush()
            prev = len(line)
    finally:
        stop_flag.set()
        if proc is not None:
            if proc.poll() is None:
                proc.terminate()
            threads[0].join(timeout=5)
            proc.wait(timeout=5)
        else:
            threads[0].join(timeout=30)
        q.put(None)
        threads[1].join(timeout=60)

        for f in (rc_f, fr_f, ev_f, log_f):
            f.close()
        if raw_file:
            raw_file.close()

    el = time.monotonic() - t0
    print("\n" + "-" * 62, file=sys.stderr)
    for s in streams:
        print(f"{s.name}: {s.frames} frames ({s.rc_frames} RC), "
              f"{s.anomalies} anomalies, {s.parser.dropped} unparsed bytes", file=sys.stderr)
    if offline:
        print(f"decoded {state['bytes']/1e6:.1f} MB "
              f"({state['bytes']/args.samplerate:.2f}s) in {el:.1f}s", file=sys.stderr)
    else:
        short = int(args.samplerate * min(el, args.duration)) - state["bytes"]
        print(f"stream: {state['bytes']/1e6:.1f} MB in {el:.1f}s "
              f"({state['bytes']/el/1e6:.2f} MB/s), "
              f"shortfall ~{short/args.samplerate*1000:.0f} ms", file=sys.stderr)
    if state["lagged"]:
        tail = ("raw.bin is complete, re-decode with --from-raw" if keep_raw
                else "those samples are GONE — rerun with --raw to make lag recoverable")
        print(f"WARNING: decoder lagged on {state['lagged']} block(s); {tail}",
              file=sys.stderr)
    if state["hit_cap"]:
        print(f"WARNING: stopped at --max-gb {args.max_gb}", file=sys.stderr)
    print(f"-> {outdir}/rc.csv  events.jsonl  frames.csv", file=sys.stderr)


if __name__ == "__main__":
    main()
