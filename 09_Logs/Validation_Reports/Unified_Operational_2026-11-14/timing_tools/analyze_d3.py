#!/usr/bin/env python3
"""D3 analysis (build 82c1af9), read-only: does a release lost with Motion idle follow a @MOTION TIMING dump?

From serial.log:
- each dump: `MOTION_TIMING record=LAST_IDLE_SKIP dump_start_us=<t0> count=<n> stamp_us=<s> ...` (first line of the dump)
  and `MOTION_TIMING dump_us=<d>` (last line);
- each activation: `MOTION_ACC_CEILING verified=.. target=.. us=<a> next_release_in_us=<r>`.

An idle loss is the increase of `count` between two dumps; its time is the `stamp_us` read at the later dump.
It is ATTRIBUTED to the earlier dump when t0 <= stamp <= t0 + d + 2 periods (40 ms), else UNEXPLAINED.
Criterion fixed before the session (candidate_82c1af9/NEXT_SESSION_PLAN.md §4): the hypothesis "the activation
losses of 15-16/11 are the diagnostic dump" holds only if every idle loss is ATTRIBUTED and no loss is counted
inside an activation (command_timing).
Usage: analyze_d3.py <serial.log>"""
import re, sys

NUM = re.compile(r'(\w+)=(-?\d+)')
dumps, acc = [], []
for line in open(sys.argv[1], errors='replace'):
    stamp, _, text = line.rstrip('\n').partition(' ')
    if text.startswith('MOTION_TIMING record=LAST_IDLE_SKIP'):
        f = {k: int(v) for k, v in NUM.findall(text)}
        f['host'] = stamp
        f['phase'] = text.split('phase=')[1].split()[0] if 'phase=' in text else '?'
        dumps.append(f)
    elif text.startswith('MOTION_TIMING dump_us=') and dumps:
        dumps[-1]['dump_us'] = int(text.split('=')[1])
    elif text.startswith('MOTION_ACC_CEILING'):
        f = {k: int(v) for k, v in NUM.findall(text)}
        f['host'] = stamp
        acc.append(f)
print('dumps %d, activations %d' % (len(dumps), len(acc)))
for d in dumps:
    print('  dump %s start_us %d dump_us %s idle_skip_count %d' % (d['host'][11:23], d['dump_start_us'], d.get('dump_us', '?'), d['count']))
for a in acc:
    print('  activation %s register-85 pass %s us, next release in %s us' % (a['host'][11:23], a.get('us', '?'), a.get('next_release_in_us', '?')))
attributed = unexplained = 0
for prev, cur in zip(dumps, dumps[1:]):
    new = cur['count'] - prev['count']
    if new <= 0:
        continue
    lo, hi = prev['dump_start_us'], prev['dump_start_us'] + prev.get('dump_us', 0) + 40000
    ok = lo <= cur['stamp_us'] <= hi
    attributed += ok
    unexplained += not ok
    print('  idle loss x%d (last at stamp %d us, lateness %d us, phase %s) after dump %s: %s'
          % (new, cur['stamp_us'], cur['lateness_us'], cur['phase'], prev['host'][11:23],
             'ATTRIBUTED to that dump' if ok else 'UNEXPLAINED (outside the dump window)'))
    if new > 1:
        print('    note: %d losses between the two dumps, only the last one is time-stamped' % new)
print('idle-loss intervals: attributed %d, unexplained %d' % (attributed, unexplained))
