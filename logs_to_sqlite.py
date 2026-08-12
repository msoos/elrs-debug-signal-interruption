#!/usr/bin/env python3
"""Load the EdgeTX flight CSVs into flights.db. Usage: ./logs_to_sqlite.py [out.db]"""

import csv
import re
import sqlite3
import sys
from datetime import datetime
from pathlib import Path

LOGS = {
    1: "f5j-new-2026-08-08-124224.csv",
    2: "f5j-new-2026-08-08-132714.csv",
}


def colname(h):
    h = re.sub(r"\(.*?\)", "", h).strip()
    h = re.sub(r"[^A-Za-z0-9]+", "_", h).strip("_")
    return ("c_" + h) if h[0].isdigit() else h


def typed(v):
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v


def main():
    root = Path(__file__).parent
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "flights.db"
    if out.exists():
        out.unlink()

    with open(root / LOGS[1], newline="") as f:
        header = next(csv.reader(f))
    cols = [colname(h) for h in header]

    db = sqlite3.connect(out)
    db.execute(
        "CREATE TABLE telemetry (flight INTEGER, row INTEGER, t REAL, "
        + ", ".join(f"{c} {'TEXT' if c in ('Date', 'Time', 'LSW') else 'NUMERIC'}" for c in cols)
        + ", PRIMARY KEY (flight, row))"
    )
    db.execute(
        "CREATE TABLE meta (flight INTEGER PRIMARY KEY, source TEXT, rows INTEGER, "
        "start TEXT, end TEXT, duration REAL)"
    )

    ins = f"INSERT INTO telemetry VALUES ({','.join('?' * (len(cols) + 3))})"
    for flight, name in LOGS.items():
        with open(root / name, newline="") as f:
            rdr = csv.reader(f)
            hdr = next(rdr)
            assert [colname(h) for h in hdr] == cols, name
            rows = list(rdr)
        t0 = datetime.strptime(rows[0][1], "%H:%M:%S.%f")
        recs = []
        for i, r in enumerate(rows):
            t = (datetime.strptime(r[1], "%H:%M:%S.%f") - t0).total_seconds()
            recs.append([flight, i, t] + [typed(v) for v in r])
        db.executemany(ins, recs)
        db.execute(
            "INSERT INTO meta VALUES (?,?,?,?,?,?)",
            (flight, name, len(rows), rows[0][1], rows[-1][1], recs[-1][2]),
        )

    db.execute("CREATE INDEX tel_t ON telemetry(flight, t)")
    db.commit()
    for f, n, s, e in db.execute("SELECT flight, rows, start, end FROM meta"):
        print(f"flight {f}: {n} rows, {s} -> {e}")
    db.close()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
