#!/usr/bin/env python3
"""Per-overrun attribution from the HTTP 1 Hz motion status (P0 closure, 16/11/2026). Read-only.
The firmware keeps no per-overrun record: the status gives the since-boot overrun counter, the phase, the reason
and the armed command's maximum release (work, phase, COMPENSATE, solves). Every increment of the counter between
two consecutive samples is attributed to that ~1 s interval:
  - phases: the phases of the two samples (one phase if equal, else the transition);
  - exact: when the command maximum rose inside the interval (same command id), the release that raised it is one
    of the overruns of that interval: its work, phase, COMPENSATE and solves are exact.
Usage: overrun_timeline.py <run-dir> [...]   (prints one line per interval with overruns, then a summary per class)"""
import collections, json, sys
from pathlib import Path

ATT = {'ALIGN', 'STAND_DEPLOY', 'STAND_LIFT', 'HEIGHT', 'PRONE_HEIGHT', 'PRONE_LIFT', 'PRONE_DEPLOY', 'PRONE_LAND', 'DISENGAGE'}


def samples(run):
    for line in open(Path(run) / 'http.jsonl'):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        s = d.get('s')
        if d.get('path') == '/api/motion/status' and isinstance(s, dict) and 'overrun_releases' in s:
            yield d['host_utc'][11:23], s


def cls(phase, reason, prev_phase):
    if reason == 'STAND_FROM_STOPPED_WALK':
        return 'RECOVERY'
    if phase == 'WALK_SETTLE' or prev_phase == 'WALK_SETTLE':
        return 'WALK_SETTLE'
    if phase == 'WALK_STANDARD':
        return {'WALK_JOYSTICK': 'WALK_STEERED', 'WALK_STOPPING': 'WALK_STOP', 'JOYSTICK_REVERSING': 'WALK_STEERED'}.get(reason, 'WALK_FRONTAL')
    if phase in ATT:
        return 'ATTENUATED'
    return phase or '?'


def run(path):
    out, prev = [], None
    for t, s in samples(path):
        if prev:
            pt, ps = prev
            n = s['overrun_releases'] - ps['overrun_releases']
            if n > 0:
                c0, c1 = ps.get('command_timing') or {}, s.get('command_timing') or {}
                same = (ps.get('active_command') or {}).get('id') == (s.get('active_command') or {}).get('id')
                exact = None
                if c1.get('max_work_us', 0) > 20000 and (not same or c1.get('max_work_us', 0) > c0.get('max_work_us', 0)) and c1.get('overruns', 0) > 0:
                    exact = (c1['max_work_us'], c1['max_work_phase'], c1['max_compensate_us'], c1['max_work_solves'])
                k = cls(exact[1] if exact else s['phase'], s.get('reason'), ps['phase'])
                out.append((pt, t, n, ps['phase'], s['phase'], s.get('reason'), exact, k))
        prev = (t, s)
    return out


if __name__ == '__main__':
    for p in sys.argv[1:]:
        rows = run(p)
        print('== %s: %d overruns in %d intervals' % (p, sum(r[2] for r in rows), len(rows)))
        tot, ex = collections.Counter(), []
        for pt, t, n, p0, p1, reason, exact, k in rows:
            tot[k] += n
            if exact:
                ex.append((k,) + exact)
            print('  %s..%s +%d %-13s -> %-13s %-24s %-12s %s' % (pt, t, n, p0, p1, reason, k,
                  'exact: %d us %s comp %d us solves %d' % exact if exact else ''))
        print('  per class:', dict(tot))
        for k in sorted(set(e[0] for e in ex)):
            v = [e for e in ex if e[0] == k]
            print('  exact releases %-12s n %d work %d..%d us, COMPENSATE %d..%d us, solves %s' % (
                k, len(v), min(e[1] for e in v), max(e[1] for e in v), min(e[3] for e in v), max(e[3] for e in v), sorted(set(e[4] for e in v))))
