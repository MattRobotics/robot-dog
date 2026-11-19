#!/usr/bin/env python3
"""Memory threshold of the 16/11 evening mandate (read-only, run directories of this session).

Reads http.jsonl of each run directory (/status heap_free and heap_min_free; /api/motion/status
transport.heap_free, heap_min_free, heap_largest) and scans serial.log for allocation-failure lines.
Verdict MEMORY_STOP when any heap_min_free sample is below 24 KB (24 576 B) or an allocation-failure
line is present: the validation stops and the rollback of the plan follows, with no automatic Motion resume.
heap_largest is reported (largest free block), it is not part of the threshold.

Usage: heap_check.py <run-dir> [<run-dir> ...]"""
import json, re, sys
from pathlib import Path

LIMIT = 24 * 1024
ALLOC = re.compile(r'(?i:alloc\w*\s*fail|fail\w*\s*(?:to\s*)?alloc|out of memory|malloc|heap_caps)|NO_MEM|\bOOM\b')


def scan(run):
    run = Path(run)
    mins, largest, free, stamps = [], [], [], []
    hp = run / 'http.jsonl'
    if hp.exists():
        for line in open(hp, errors='replace'):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            s = r.get('s')
            if not isinstance(s, dict):
                continue
            src = s.get('transport') if r.get('path') == '/api/motion/status' else s
            if not isinstance(src, dict) or 'heap_min_free' not in src:
                continue
            v = int(src['heap_min_free'])
            mins.append(v)
            stamps.append((v, r.get('host_utc'), r.get('path')))
            if 'heap_free' in src:
                free.append(int(src['heap_free']))
            if 'heap_largest' in src:
                largest.append(int(src['heap_largest']))
    alloc = []
    sp = run / 'serial.log'
    if sp.exists():
        for line in open(sp, errors='replace'):
            if ALLOC.search(line):
                alloc.append(line.rstrip('\n')[:200])
    return mins, largest, free, stamps, alloc


if __name__ == '__main__':
    stop = False
    for run in sys.argv[1:]:
        mins, largest, free, stamps, alloc = scan(run)
        if not mins:
            print('%s: no heap samples' % run)
            continue
        low = min(stamps)
        below = sum(1 for v in mins if v < LIMIT)
        print('%s: samples %d, heap_min_free min %d B (%s %s), heap_largest min %s / max %s B, heap_free min %s B, below 24 KB %d, allocation-failure lines %d'
              % (run, len(mins), low[0], low[1], low[2], min(largest) if largest else '-', max(largest) if largest else '-',
                 min(free) if free else '-', below, len(alloc)))
        for a in alloc[:5]:
            print('  ALLOC', a)
        stop |= below > 0 or bool(alloc)
    print('MEMORY_STOP' if stop else 'MEMORY_OK (heap_min_free >= 24 KB, no allocation failure)')
    sys.exit(1 if stop else 0)
