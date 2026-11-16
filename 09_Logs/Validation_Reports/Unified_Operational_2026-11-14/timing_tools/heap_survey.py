#!/usr/bin/env python3
"""Read-only survey of every http.jsonl under verification-artifacts: per run, build, internal-heap steady free
(median of heap_free samples), lowest heap_min_free, the transient (steady - min), max ap_clients, sample rate.
Usage: heap_survey.py <root>"""
import json, statistics, sys
from pathlib import Path
rows = []
for f in sorted(Path(sys.argv[1]).glob('**/http.jsonl')):
    free, mins, build, apc, t = [], [], set(), [], []
    try:
        for line in open(f, errors='replace'):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            s = r.get('s')
            if not isinstance(s, dict):
                continue
            if r.get('path') == '/status':
                if 'build_id' in s: build.add(s['build_id'])
                if 'ap_clients' in s:
                    try: apc.append(int(s['ap_clients']))
                    except ValueError: pass
                src = s
            else:
                src = s.get('transport') or {}
            if 'heap_min_free' in src:
                try:
                    mins.append(int(src['heap_min_free'])); free.append(int(src['heap_free'])); t.append(r['host_utc'])
                except (KeyError, ValueError):
                    pass
    except OSError:
        continue
    if mins:
        st = int(statistics.median(free))
        rows.append((str(f.parent.relative_to(sys.argv[1])), ','.join(sorted(build)) or '?', st, min(mins), st - min(mins),
                     max(apc) if apc else '-', len(mins), t[0][:16]))
print('%-78s %-14s %7s %7s %7s %4s %5s %s' % ('run', 'build', 'steady', 'min', 'trans', 'apc', 'n', 'first'))
for r in sorted(rows, key=lambda r: r[7]):
    print('%-78s %-14s %7d %7d %7d %4s %5d %s' % r)
