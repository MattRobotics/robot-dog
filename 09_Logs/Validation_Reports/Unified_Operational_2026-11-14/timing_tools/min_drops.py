#!/usr/bin/env python3
"""Read-only: for one session folder, merge the heap samples of every http.jsonl (one file per watcher) and print each
decrease of heap_min_free (since-boot watermark; a reset shows as an increase) with the watchers that have samples within
+-1.5 s and the fixture tools whose run was active (events.jsonl between session_start and the last event).
Usage: min_drops.py <session-folder>"""
import json, sys, datetime
from pathlib import Path

def ts(s):
    return datetime.datetime.fromisoformat(s.replace('Z', '+00:00')).timestamp()

root = Path(sys.argv[1])
samples, runs = [], []
for f in sorted(root.glob('**/http.jsonl')):
    for line in open(f, errors='replace'):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        s = r.get('s')
        if not isinstance(s, dict):
            continue
        src = s if r.get('path') == '/status' else (s.get('transport') or {})
        if 'heap_min_free' in src:
            samples.append((ts(r['host_utc']), int(src['heap_min_free']), int(src['heap_free']), f.parent.name, r.get('path')))
for e in sorted(root.glob('**/events.jsonl')):
    t = [ts(json.loads(l)['host_utc']) for l in open(e, errors='replace') if l.strip()]
    if t:
        runs.append((min(t), max(t), e.parent.name))
samples.sort()
prev = None
for t, mn, fr, who, path in samples:
    if prev is not None and mn < prev:
        near = sorted({w for (u, _, _, w, _) in samples if abs(u - t) < 1.5})
        active = [n for a, b, n in runs if a - 1 <= t <= b + 1]
        print('%s min %6d -> %6d (-%5d)  free %6d  by %-28s watchers %d %s  tool %s' % (
            datetime.datetime.utcfromtimestamp(t).strftime('%m-%d %H:%M:%S.%f')[:-3], prev, mn, prev - mn, fr, who + path[:12],
            len(near), near, active))
    prev = mn
