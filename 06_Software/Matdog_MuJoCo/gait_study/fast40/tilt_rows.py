#!/usr/bin/env python3
"""FAST 40 study: run the tilt/STOP check (grid.tilt) on chosen screen survivors of a grid file.
Usage: tilt_rows.py <bin dir> <grid.jsonl> <out.jsonl> <python filter expression on r> [--jobs 8] [--limit N]
(survivors in ascending law peak speed: the STOP arrest grows with it)"""
import concurrent.futures as cf
import json
import sys

import grid


def main():
    bindir, src, out, expr = sys.argv[1:5]
    jobs = int(sys.argv[sys.argv.index("--jobs") + 1]) if "--jobs" in sys.argv else 8
    rows = [json.loads(l) for l in open(src)]
    pick = sorted((r for r in rows if r.get("screen") and "tilt_pass" not in r and eval(expr, {}, {"r": r})), key=lambda r: r["vmax"])
    if "--limit" in sys.argv:
        pick = pick[:int(sys.argv[sys.argv.index("--limit") + 1])]
    print(f"{len(pick)} rows", file=sys.stderr)
    with cf.ThreadPoolExecutor(jobs) as ex, open(out, "w") as f:
        for t in ex.map(lambda r: grid.tilt(bindir, r), pick):
            f.write(json.dumps(t) + "\n")
            f.flush()


if __name__ == "__main__":
    main()
