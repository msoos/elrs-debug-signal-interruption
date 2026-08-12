#!/usr/bin/env python3
"""Test the frozen-outputs hypothesis using ANT as a receiver-side attitude witness.

Usage: ./ant_analyze.py [flights.db]
"""

import sqlite3
import sys
from pathlib import Path

DB = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "flights.db")
WIN = 10.0
STEP = 0.5
EPISODES = {1: ("12:50:26.690", "12:51:05.190"), 2: ("13:32:47.200", "13:33:50.200")}


def load(db, f):
    return list(db.execute(
        "SELECT row, t, Time, Alt, ANT, c_1RSS, c_2RSS, RQly, Rud, Ail, Ele "
        "FROM telemetry WHERE flight=? ORDER BY row", (f,)))


def windows(rows):
    """Sliding windows: commanded control effort vs observed antenna activity."""
    out = []
    n = len(rows)
    j = 0
    for i in range(n):
        while rows[j]["t"] < rows[i]["t"] - WIN:
            j += 1
        w = rows[j:i + 1]
        if len(w) < 15:
            continue
        sw = sum(1 for k in range(1, len(w)) if w[k]["ANT"] != w[k - 1]["ANT"])
        eff = sum(max(abs(r["Rud"]), abs(r["Ail"])) for r in w) / len(w) / 1024
        out.append(dict(t=rows[i]["t"], time=rows[i]["Time"][:12], sw=sw, eff=eff,
                        alt=rows[i]["Alt"], rq=min(r["RQly"] for r in w)))
    return out


def dwells(rows):
    out, s = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["ANT"] != rows[s]["ANT"]:
            out.append((rows[s]["t"], rows[i - 1]["t"] - rows[s]["t"] + 0.5,
                        rows[s]["ANT"], rows[s]["Time"][:12]))
            s = i
    return out


def geom_p(dw, length):
    """P(some dwell >= length) under a geometric model fitted to this flight's dwells."""
    samples = sum(d / 0.5 for _, d, _, _ in dw)
    p_switch = len(dw) / samples  # per-sample switch probability
    k = length / 0.5
    p_one = (1 - p_switch) ** (k - 1)
    return 1 - (1 - p_one) ** len(dw), p_switch


def longest_mute(hi, sw, w, lo=None, hi_i=None):
    """Longest span (samples) of windows that are high-effort with zero ANT switch.
    hi[i]: window ending at i is high-effort. sw: per-sample switch indicator.
    If lo/hi_i given, only runs overlapping that index range count."""
    pre = [0] * (len(sw) + 1)
    for i, s in enumerate(sw):
        pre[i + 1] = pre[i] + s
    best = cur = 0
    for i in range(w, len(sw)):
        if hi[i] and pre[i + 1] - pre[i + 1 - w] == 0:
            cur += 1
            if lo is None or (i - cur - w < hi_i and i >= lo):
                best = max(best, cur)
        else:
            cur = 0
    return (best + w) * 0.5 if best else 0.0


def rotation_test(rows, thr, ep=None, trials=2000):
    """Circularly rotate the ANT switch train against the (fixed) stick series.
    Each series keeps its own autocorrelation; only their alignment is destroyed.
    With ep=(lo,hi) the statistic is the longest mute run overlapping the episode."""
    n = len(rows)
    w = int(WIN / 0.5)
    eff = [max(abs(r["Rud"]), abs(r["Ail"])) / 1024 for r in rows]
    pe = [0.0] * (n + 1)
    for i, e in enumerate(eff):
        pe[i + 1] = pe[i] + e
    hi = [i >= w and (pe[i + 1] - pe[i + 1 - w]) / w >= thr for i in range(n)]
    sw = [0] + [1 if rows[i]["ANT"] != rows[i - 1]["ANT"] else 0 for i in range(1, n)]
    lo_i, hi_i = ep if ep else (None, None)

    obs = longest_mute(hi, sw, w, lo_i, hi_i)
    ge = sum(1 for k in range(1, trials + 1)
             if longest_mute(hi, sw[(k * n) // (trials + 1):] + sw[:(k * n) // (trials + 1)],
                             w, lo_i, hi_i) >= obs)
    return obs, (ge + 1) / (trials + 1)


def main():
    db = sqlite3.connect(DB)
    db.row_factory = sqlite3.Row

    for f in (1, 2):
        rows = load(db, f)
        by_time = {r["Time"][:12]: r["t"] for r in rows}
        e0, e1 = (by_time[x] for x in EPISODES[f])
        ws = windows(rows)
        dw = dwells(rows)

        print(f"\n{'=' * 74}\nFLIGHT {f}   {rows[0]['Time'][:8]}-{rows[-1]['Time'][:8]}   "
              f"episode {EPISODES[f][0][:8]}-{EPISODES[f][1][:8]}")
        print("=" * 74)

        # 0. is ANT a usable witness at all? does it track commanded turning?
        act = [w for w in ws if not (e0 <= w["t"] <= e1)]
        act.sort(key=lambda w: w["eff"])
        q = len(act) // 4
        mx = sum(w["eff"] for w in act) / len(act)
        my = sum(w["sw"] for w in act) / len(act)
        num = sum((w["eff"] - mx) * (w["sw"] - my) for w in act)
        den = (sum((w["eff"] - mx) ** 2 for w in act)
               * sum((w["sw"] - my) ** 2 for w in act)) ** 0.5
        print("\n[0] Is ANT a witness?  non-episode windows by control-effort quartile")
        for j, nm in enumerate(["Q1 low ", "Q2     ", "Q3     ", "Q4 high"]):
            g = act[j * q:(j + 1) * q]
            print(f"    {nm}  eff {g[0]['eff']:.2f}-{g[-1]['eff']:.2f}  "
                  f"mean {sum(x['sw'] for x in g)/len(g):.2f} switches  "
                  f"zero-switch {100*sum(1 for x in g if x['sw']==0)/len(g):3.0f}%")
        print(f"    Pearson r(effort, switches) = {num/den:+.3f}")

        # 1. antenna dwell: is the episode freeze exceptional for this flight?
        srt = sorted(dw, key=lambda x: -x[1])
        p, ps = geom_p(dw, srt[0][1])
        print(f"\n[1] Antenna dwell   {len(dw)} dwells, mean {sum(d[1] for d in dw)/len(dw):.1f}s, "
              f"per-sample switch prob {ps:.3f}")
        for t0, d, a, ts in srt[:5]:
            inep = "  <-- IN EPISODE" if t0 < e1 and t0 + d > e0 else ""
            print(f"    {d:5.1f}s  ANT={a}  at {ts}{inep}")
        print(f"    P(longest >= {srt[0][1]:.1f}s | geometric fit) = {p:.4f}")

        # 2. the discriminator: commanded turn with no antenna response
        thr = act[3 * len(act) // 4]["eff"]
        base = [w for w in act if w["eff"] >= thr]
        dead = [w for w in base if w["sw"] == 0]
        print(f"\n[2] Command vs response   (10s windows, effort = mean |stick| / full)")
        print(f"    outside episode, top-quartile effort (>= {thr:.2f}): {len(base)} windows, "
              f"mean {sum(w['sw'] for w in base)/len(base):.2f} ANT switches; "
              f"{len(dead)} ({100*len(dead)/len(base):.0f}%) had zero")
        ep = [w for w in ws if e0 <= w["t"] <= e1]
        eph = [w for w in ep if w["eff"] >= thr]
        if eph:
            print(f"    inside episode,  same effort band: {len(eph)} windows, "
                  f"mean {sum(w['sw'] for w in eph)/len(eph):.2f} ANT switches; "
                  f"{sum(1 for w in eph if w['sw']==0)} had zero")

        # 3. worst windows overall: high command, zero observed attitude change
        mute = [w for w in ws if w["sw"] == 0 and w["eff"] >= thr]
        print(f"\n[3] Mute windows (effort >= {thr:.2f}, zero ANT switch), whole flight:")
        if not mute:
            print("    none")
        merged, cur = [], None
        for w in mute:
            if cur and w["t"] - cur[-1]["t"] <= STEP * 1.5:
                cur.append(w)
            else:
                cur = [w]
                merged.append(cur)
        for grp in sorted(merged, key=lambda g: -(g[-1]["t"] - g[0]["t"]))[:6]:
            span = grp[-1]["t"] - grp[0]["t"] + WIN
            inep = "  <-- EPISODE" if grp[0]["t"] - WIN < e1 and grp[-1]["t"] > e0 else ""
            print(f"    {grp[0]['time']} .. {grp[-1]['time']}  span {span:4.1f}s  "
                  f"peak effort {max(w['eff'] for w in grp):.2f}  "
                  f"min RQly {min(w['rq'] for w in grp)}{inep}")

        idx = {r["t"]: i for i, r in enumerate(rows)}
        obs, p = rotation_test(rows, thr)
        obs_e, p_e = rotation_test(rows, thr, ep=(idx[e0], idx[e1]))
        print(f"\n[4] Rotation test (2000 rotations of ANT against the sticks)")
        print(f"    longest commanded-but-mute run anywhere : {obs:5.1f}s   p = {p:.4f}")
        print(f"    longest one overlapping the episode     : {obs_e:5.1f}s   p = {p_e:.4f}")


if __name__ == "__main__":
    main()
