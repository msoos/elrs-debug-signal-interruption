#!/usr/bin/env python3
"""Cut a window out of raw.bin into a .sr file for PulseView.

Usage: ./crsf_slice.py run1/raw.bin --at 47.3 --window 2 -o bug.sr
"""

import argparse
import json
import os
import zipfile

METADATA = """[global]
sigrok version=0.5.2

[device 1]
capturefile=logic-1
total probes=8
samplerate={rate} Hz
total analog=0
{probes}unitsize=1
"""


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("raw")
    p.add_argument("-o", "--output", required=True)
    p.add_argument("--at", type=float, help="centre of window, seconds")
    p.add_argument("--window", type=float, default=1.0, help="width in seconds (default 1)")
    p.add_argument("--start", type=float, help="start, seconds (alternative to --at)")
    p.add_argument("--end", type=float, help="end, seconds")
    p.add_argument("--samplerate", type=float, default=None,
                   help="default: read from meta.json beside raw")
    p.add_argument("--channels", type=int, default=8)
    args = p.parse_args()

    rate = args.samplerate
    if rate is None:
        meta_path = os.path.join(os.path.dirname(os.path.abspath(args.raw)), "meta.json")
        if os.path.exists(meta_path):
            with open(meta_path) as fh:
                rate = json.load(fh)["samplerate"]
        else:
            p.error("no meta.json beside raw; pass --samplerate")

    if args.at is not None:
        start, end = args.at - args.window / 2, args.at + args.window / 2
    elif args.start is not None:
        start = args.start
        end = args.end if args.end is not None else start + args.window
    else:
        p.error("need --at or --start")

    total = os.path.getsize(args.raw)
    s0 = max(0, int(start * rate))
    s1 = min(total, int(end * rate))
    if s1 <= s0:
        p.error(f"empty window ({s0}..{s1}); capture holds {total/rate:.3f}s")

    with open(args.raw, "rb") as fh:
        fh.seek(s0)
        data = fh.read(s1 - s0)

    probes = "".join(f"probe{i+1}=D{i}\n" for i in range(args.channels))
    with zipfile.ZipFile(args.output, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("version", "2")
        z.writestr("metadata", METADATA.format(rate=int(rate), probes=probes))
        z.writestr("logic-1-1", data)

    print(f"{args.output}: {s0/rate:.4f}s..{s1/rate:.4f}s "
          f"({len(data)} samples, {len(data)/1e6:.1f} MB raw)")
    print(f"sample offset {s0} — add it back to match rc.csv sample numbers")


if __name__ == "__main__":
    main()
