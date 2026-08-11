#!/usr/bin/env python3
"""Summarise a capture run: frame health, per-channel behaviour, anomaly windows.

Usage: ./crsf_analyze.py run1
"""

import argparse
import csv
import json
import os
from collections import Counter

import numpy as np

from crsf_decode import TICK_MAX, TICK_MIN, parse_link_stats, ticks_to_us


def load_link_stats(path):
    t, lq, rssi, snr = [], [], [], []
    if not os.path.exists(path):
        return (np.array([]),) * 4
    with open(path) as fh:
        for row in csv.DictReader(fh):
            if row["type"] != "0x14":
                continue
            s = parse_link_stats(bytes.fromhex(row["payload_hex"]))
            if s:
                t.append(float(row["t_s"]))
                lq.append(s["up_lq"])
                rssi.append(s["up_rssi1"])
                snr.append(s["up_snr"])
    return np.array(t), np.array(lq), np.array(rssi), np.array(snr)


def report_link(rundir, t_rc, med):
    """Are the frame gaps RF-caused or receiver-caused?"""
    lt, lq, rssi, snr = load_link_stats(os.path.join(rundir, "frames.csv"))
    if lt.size == 0:
        return
    print(f"\nlink statistics ({lt.size} frames, {lt.size/(lt[-1]-lt[0]):.1f} Hz):")
    print(f"  uplink LQ   min {lq.min():3d}  p1 {np.percentile(lq,1):5.1f}  "
          f"median {np.median(lq):5.1f}")
    print(f"  uplink RSSI min {rssi.min():4d}  median {np.median(rssi):6.1f} dBm"
          f"   SNR min {snr.min():3d}  median {np.median(snr):5.1f} dB")

    # LQ is a windowed delivery rate, so missing frames are expected at LQ<100.
    # The question is whether more frames are missing than LQ accounts for.
    observed = t_rc.size
    expected = (t_rc[-1] - t_rc[0]) / (med / 1000.0)
    loss = 1.0 - observed / expected
    rf_loss = 1.0 - lq.mean() / 100.0
    print(f"  frames present {observed} of ~{expected:.0f} expected at median "
          f"cadence = {100*loss:.2f}% missing")
    print(f"  uplink LQ accounts for {100*rf_loss:.2f}% loss "
          f"(mean LQ {lq.mean():.1f})")
    excess = loss - rf_loss
    if excess > 0.02:
        print(f"  -> {100*excess:.2f}% MORE missing than the link explains — "
              f"the receiver dropped frames it had the data for")
    else:
        print(f"  -> consistent with the reported link quality; no unexplained loss")


def load_rc(path, direction):
    t, ch = [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            if row["dir"] != direction:
                continue
            t.append(float(row["t_s"]))
            ch.append([int(row[f"ch{i}"]) for i in range(1, 17)])
    return np.array(t), np.array(ch, dtype=np.int32)


def find_stuck(t, ch, moving, world, stuck_s):
    """Channels held constant for >= stuck_s while other channels kept moving."""
    out = []
    d = np.diff(ch, axis=0)
    for i in moving:
        pts = np.concatenate(([0], np.flatnonzero(d[:, i] != 0) + 1, [len(t) - 1]))
        for a, b in zip(pts[:-1], pts[1:]):
            if b <= a:
                continue
            dur = t[b] - t[a]
            if dur >= stuck_s and world[a:b].any():
                out.append({"kind": "stuck", "ch": i + 1, "start": t[a], "end": t[b],
                            "detail": f"held {ch[a, i]} for {dur:.2f}s"})
    return out


def find_frozen_all(t, ch, active, stuck_s):
    """Every channel stopped at once.

    find_stuck needs some other channel still moving as its reference, so it is
    blind to a receiver that freezes all outputs together -- which is exactly
    the failure being hunted. This looks for stretches where nothing moved.
    """
    if not len(active):
        return []
    moved = (np.diff(ch[:, active], axis=0) != 0).any(axis=1)
    pts = np.concatenate(([0], np.flatnonzero(moved) + 1, [len(t) - 1]))
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        if b <= a:
            continue
        dur = t[b] - t[a]
        if dur >= stuck_s:
            vals = ",".join(str(int(v)) for v in ch[a, active][:8])
            out.append({"kind": "ALL-FROZEN", "ch": 0, "start": t[a], "end": t[b],
                        "detail": f"no channel changed for {dur:.2f}s (held {vals})"})
    return out


def channel_groups(d, moving):
    """Cluster channels by which frames they update on.

    ELRS half-rate switch modes send alternate halves of the payload on
    successive packets, so channels do not all move together and a single
    cross-channel consensus would be meaningless. Each group gets its own.
    """
    masks = {i: d[:, i] != 0 for i in moving}
    groups = []
    for i in moving:
        for g in groups:
            inter = int((masks[i] & masks[g[0]]).sum())
            union = int((masks[i] | masks[g[0]]).sum())
            if union and inter / union > 0.5:
                g.append(i)
                break
        else:
            groups.append([i])
    return groups


def find_diverged(t, ch, groups, tol, min_frames):
    """Channels disagreeing with the consensus of their own update group.

    Within a group the sweep moves every channel together, so the median is what
    each should read. Marker slams move the whole group and cancel out; a single
    channel going its own way does not.
    """
    out = []
    for g in groups:
        if len(g) < 3:
            continue
        med = np.median(ch[:, g], axis=1)
        for i in g:
            bad = np.abs(ch[:, i] - med) > tol
            if not bad.any():
                continue
            edges = np.diff(np.concatenate(([0], bad.view(np.int8), [0])))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                if b - a < min_frames:
                    continue
                worst = int(np.abs(ch[a:b, i] - med[a:b]).max())
                out.append({"kind": "diverged", "ch": i + 1, "start": t[a],
                            "end": t[b - 1],
                            "detail": f"up to {worst} ticks off group consensus"})
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("rundir")
    p.add_argument("--dir", default="rx", choices=("rx", "tx"))
    p.add_argument("--stuck-s", type=float, default=1.0)
    # healthy lag between channels tops out at one staircase step (~34 ticks);
    # marker slams show up as 1-frame transients, hence the 3-frame minimum
    p.add_argument("--diverge-ticks", type=int, default=40)
    p.add_argument("--min-diverge-frames", type=int, default=3)
    p.add_argument("--cluster-s", type=float, default=0.5)
    p.add_argument("--ignore-ch", "--ignore", dest="ignore_ch", default="",
                   help="channels to exclude entirely, e.g. --ignore-ch 9,10,11")
    p.add_argument("--ignore-arm", action="store_true",
                   help="shorthand for --ignore-ch 5")
    args = p.parse_args()

    ignore = {int(x) for x in args.ignore_ch.replace(",", " ").split()}
    if args.ignore_arm:
        ignore.add(5)

    t, ch = load_rc(os.path.join(args.rundir, "rc.csv"), args.dir)
    if len(t) == 0:
        print(f"no RC frames for dir={args.dir}")
        return

    span = t[-1] - t[0]
    dt = np.diff(t) * 1000.0
    print(f"== {args.rundir} [{args.dir}] ==")
    print(f"{len(t)} RC frames over {span:.3f}s = {len(t)/span:.1f} Hz")
    print(f"frame interval ms: median {np.median(dt):.3f}  p99 {np.percentile(dt, 99):.3f}  "
          f"max {dt.max():.3f}")

    gaps = np.flatnonzero(dt > 3 * np.median(dt))
    if gaps.size:
        print(f"\n{gaps.size} interval(s) >3x median:")
        for i in gaps[:15]:
            print(f"   t={t[i+1]:9.4f}s  gap {dt[i]:8.2f} ms "
                  f"(~{dt[i]/np.median(dt):.1f} frames)")
        if gaps.size > 15:
            print(f"   ... and {gaps.size - 15} more")

    report_link(args.rundir, t, np.median(dt))

    rng = ch.max(0) - ch.min(0)
    moving = np.array([i for i in np.flatnonzero(rng >= 10) if i + 1 not in ignore],
                      dtype=int)
    d = np.diff(ch, axis=0)
    world = (d[:, moving] != 0).any(axis=1) if moving.size else np.zeros(len(t) - 1, bool)

    print(f"\nper-channel (ticks {TICK_MIN}..{TICK_MAX} = {ticks_to_us(TICK_MIN):.0f}.."
          f"{ticks_to_us(TICK_MAX):.0f} us):")
    print(f"{'ch':>3} {'min':>6} {'max':>6} {'min_us':>7} {'max_us':>7} "
          f"{'levels':>7} {'chg/s':>7}  note")
    for i in range(16):
        c = ch[:, i]
        note = []
        if c.min() < TICK_MIN or c.max() > TICK_MAX:
            note.append("OUT-OF-RANGE")
        if i + 1 in ignore:
            note.append("ignored")
        elif rng[i] < 10:
            note.append("held constant")
        print(f"{i+1:>3} {c.min():>6} {c.max():>6} {ticks_to_us(c.min()):>7.0f} "
              f"{ticks_to_us(c.max()):>7.0f} {len(np.unique(c)):>7} "
              f"{int(np.count_nonzero(d[:, i]))/span:>7.1f}  {' '.join(note)}")

    active = [i for i in range(16) if i + 1 not in ignore]
    findings = find_frozen_all(t, ch, active, args.stuck_s)
    groups = []
    if moving.size:
        groups = channel_groups(d, moving)
        if len(groups) > 1:
            print(f"\nupdate groups (channels never move together across groups):")
            for g in groups:
                print(f"  {[int(i) + 1 for i in g]}")
        findings += find_stuck(t, ch, moving, world, args.stuck_s)
        findings += find_diverged(t, ch, groups, args.diverge_ticks,
                                  args.min_diverge_frames)

    print(f"\nchannel behaviour (stuck >={args.stuck_s}s, diverged >{args.diverge_ticks} "
          f"ticks from consensus):")
    if not findings:
        print("  clean — every moving channel tracked the others throughout")
    else:
        for f in sorted(findings, key=lambda f: f["start"])[:30]:
            who = "ALL" if f["ch"] == 0 else f"ch{f['ch']}"
            print(f"  {f['kind']:11s} {who:<5s} t={f['start']:9.4f}s.."
                  f"{f['end']:9.4f}s  {f['detail']}")
        if len(findings) > 30:
            print(f"  ... and {len(findings) - 30} more")

        windows = []
        for f in sorted(findings, key=lambda f: f["start"]):
            if windows and f["start"] - windows[-1]["end"] <= args.cluster_s:
                windows[-1]["end"] = max(windows[-1]["end"], f["end"])
                windows[-1]["chs"].add(f["ch"])
            else:
                windows.append({"start": f["start"], "end": f["end"], "chs": {f["ch"]}})
        print(f"\n{len(windows)} window(s) to look at:")
        for w in windows[:10]:
            print(f"  t={w['start']:9.4f}s..{w['end']:9.4f}s  "
                  f"ch {sorted(int(c) for c in w['chs'])}")
            print(f"     ./crsf_slice.py {args.rundir}/raw.bin "
                  f"--start {max(0, w['start']-0.2):.3f} --end {w['end']+0.2:.3f} "
                  f"-o {args.rundir}/bug-{w['start']:.3f}.sr")

    ev_path = os.path.join(args.rundir, "events.jsonl")
    if os.path.exists(ev_path):
        with open(ev_path) as fh:
            events = [json.loads(l) for l in fh if l.strip()]
        events = [e for e in events if e.get("dir") == args.dir
                  and e["kind"] not in ("stuck",)]
        print(f"\nframe-level events from capture: {len(events)}")
        for kind, n in Counter(e["kind"] for e in events).most_common():
            print(f"  {kind:<18} {n}")


if __name__ == "__main__":
    main()
