#!/usr/bin/env python3
"""Summarise a capture run: frame health, per-channel behaviour, anomaly clusters.

Usage: ./crsf_analyze.py run1
"""

import argparse
import csv
import json
import os
from collections import Counter

import numpy as np

from crsf_decode import TICK_MAX, TICK_MIN, ticks_to_us


def load_rc(path, direction):
    t, ch = [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            if row["dir"] != direction:
                continue
            t.append(float(row["t_s"]))
            ch.append([int(row[f"ch{i}"]) for i in range(1, 17)])
    return np.array(t), np.array(ch, dtype=np.int32)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("rundir")
    p.add_argument("--dir", default="rx", choices=("rx", "tx"))
    p.add_argument("--cluster-s", type=float, default=0.5,
                   help="merge anomalies closer than this into one window")
    args = p.parse_args()

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

    print(f"\nper-channel (ticks {TICK_MIN}..{TICK_MAX} = {ticks_to_us(TICK_MIN):.0f}.."
          f"{ticks_to_us(TICK_MAX):.0f} us):")
    print(f"{'ch':>3} {'min':>6} {'max':>6} {'min_us':>7} {'max_us':>7} "
          f"{'levels':>7} {'chg/s':>7}  note")
    for i in range(16):
        c = ch[:, i]
        changes = int(np.count_nonzero(np.diff(c)))
        levels = len(np.unique(c))
        note = []
        if c.min() < TICK_MIN or c.max() > TICK_MAX:
            note.append("OUT-OF-RANGE")
        if changes == 0:
            note.append("static")
        elif levels <= 8 and len(t) >= 200:
            note.append("quantised (ELRS switch ch?)")
        print(f"{i+1:>3} {c.min():>6} {c.max():>6} {ticks_to_us(c.min()):>7.0f} "
              f"{ticks_to_us(c.max()):>7.0f} {levels:>7} {changes/span:>7.1f}  "
              f"{' '.join(note)}")

    ev_path = os.path.join(args.rundir, "events.jsonl")
    events = []
    if os.path.exists(ev_path):
        with open(ev_path) as fh:
            events = [json.loads(l) for l in fh if l.strip()]
    events = [e for e in events if e.get("dir") == args.dir]

    print(f"\n{len(events)} anomaly event(s)")
    if not events:
        print("  clean")
        return
    for kind, n in Counter(e["kind"] for e in events).most_common():
        print(f"  {kind:<18} {n}")

    windows = []
    for e in sorted(events, key=lambda e: e["t_s"]):
        if windows and e["t_s"] - windows[-1]["end"] <= args.cluster_s:
            windows[-1]["end"] = e["t_s"]
            windows[-1]["n"] += 1
            windows[-1]["kinds"][e["kind"]] += 1
        else:
            windows.append({"start": e["t_s"], "end": e["t_s"], "n": 1,
                            "kinds": Counter([e["kind"]])})

    print(f"\n{len(windows)} anomaly window(s), worst first:")
    for w in sorted(windows, key=lambda w: -w["n"])[:20]:
        dur = w["end"] - w["start"]
        kinds = " ".join(f"{k}={v}" for k, v in w["kinds"].most_common())
        print(f"  t={w['start']:9.4f}s..{w['end']:9.4f}s ({dur:6.3f}s) "
              f"{w['n']:5d} events  {kinds}")
        print(f"     ./crsf_slice.py {args.rundir}/raw.bin --start {max(0, w['start']-0.2):.3f} "
              f"--end {w['end']+0.2:.3f} -o {args.rundir}/bug-{w['start']:.3f}.sr")


if __name__ == "__main__":
    main()
