#!/usr/bin/env python3
"""Read-only comparison of two fixture sessions at equal blocks (82c1af9 session, 16/11/2026).
Usage: session_compare.py <before-suite-dir> <after-suite-dir> [<before-H-dir> <after-H-dir>]
Each suite dir holds serial.log, http.jsonl, results.json, analyze_diag.txt (analyze_diag.py, unchanged).
Prints, before -> after:
 1. per block and class (analyze_diag trials): ms/release, N+C ms, k instructions, IPC, istall_mem;
 2. heaviest release per class since boot (wall us, nominal / compensate us);
 3. release work distribution per step class from the firmware histogram (command_timing work bins
    <5,<10,<15,<20,<25,<30,<40,>=40 ms, last sample of each command): count, share >= 20 ms (overrun bins),
    upper bound of the p99 bin;
 4. missed / overrun totals and every since-boot missed increment with the state around it;
 5. IMU wanted - applied (mrad, 1 Hz samples) in frontal walk, steered walk and recovery;
 6. STOP verification time and resume completion per STOP step.
The same script on 15/11 (e321e86) vs 16/11 (46b3175) reproduces the 16/11 report's tables."""
import json, math, re, statistics as st, sys
from pathlib import Path

BINS_MS = [5, 10, 15, 20, 25, 30, 40, float('inf')]


def trials(path, align=False):
    # align (suite block table only, 16/11 P4 session): when a targeted run precedes the suite on the same
    # boot, analyze_diag reports the first dump as already holding PMU sums and trial 1 = boot -> A0, not a
    # suite block; the blocks are then trials 2.. and are renumbered so that block A is trial 1.
    out, trial, cur, shift = {}, None, None, 0
    for l in open(path):
        if align and l.startswith('NOTE: the first dump already holds PMU sums'):
            shift = 1; continue
        m = re.match(r'== trial (\d+)', l)
        if m:
            trial = int(m.group(1)) - shift; continue
        m = re.match(r'\s+(WALK|RECOVERY|OTHER)\s+releases (\d+), wall ([\d.]+) ms/release \(N\+C ([\d.]+) ms\)', l)
        if m and trial:
            cur = (trial, m.group(1)); out[cur] = {'n': int(m.group(2)), 'wall': float(m.group(3)), 'nc': float(m.group(4))}; continue
        m = re.match(r'\s+N\+C: (\d+) k instr/release.*IPC ([\d.]+).*istall [\d.]+ \(mem ([\d.]+)', l)
        if m and cur:
            out[cur].update(ki=int(m.group(1)), ipc=float(m.group(2)), im=float(m.group(3)))
    return out


def heaviest(path):
    h = {}
    for l in open(path):
        m = re.match(r'\s+heaviest (\w+)\s+since boot: release (\d+) work (\d+) us.*nominal (\d+) us compensate (\d+) us', l)
        if m:
            h[m.group(1)] = tuple(int(x) for x in m.groups()[1:])
    return h


def motion_samples(run):
    for l in open(Path(run) / 'http.jsonl'):
        d = json.loads(l)
        if d.get('path') == '/api/motion/status' and 's' in d:
            yield d['host_utc'], d['s']


def step_class(name):
    k = name.split()[0]
    if 'resume' in name or k in ('H7', 'H9', 'H14', 'H16'):
        return 'RESUME/RECOVERY'
    if 'joystick' in name or name.startswith(('D3 ', 'D5 ', 'D7 ', 'D9 ', 'D11 ')):
        return 'STEERED WALK'
    if 'walk' in name.lower() or 'WALK' in name:
        return 'FRONTAL WALK'
    if 'STAND' in name or 'PRONE' in name:
        return 'STAND/PRONE'
    return 'OTHER'


def work_hist_by_class(run):
    """command_timing.work hist is not in the results; use the http 'work_hist' of the last sample of each command id."""
    last = {}
    for t, s in motion_samples(run):
        cid = s['active_command'].get('id')
        if cid and s.get('work_hist'):
            last[cid] = (s['active_command']['reason'], s['work_hist'])
    return last


def p99_bin(hist):
    n = sum(hist)
    if not n:
        return None
    acc = 0
    for edge, c in zip(BINS_MS, hist):
        acc += c
        if acc >= 0.99 * n:
            return edge
    return BINS_MS[-1]


def misses(run):
    prev, out = None, []
    for t, s in motion_samples(run):
        if prev is not None and s['missed_releases'] > prev[1]['missed_releases']:
            out.append((t[11:23], s['missed_releases'] - prev[1]['missed_releases'], prev[1]['state'] + '/' + prev[1]['reason'], s['state'] + '/' + s['phase']))
        prev = (t, s)
    return out


def imu(run):
    res = {}
    for t, s in motion_samples(run):
        ic = s.get('imu_correction') or {}
        if 'wanted_mrad' not in ic or (s['state'] != 'WALKING' and s['reason'] != 'STAND_FROM_STOPPED_WALK'):
            continue
        c = 'RECOVERY' if s['reason'] == 'STAND_FROM_STOPPED_WALK' else ('STEER' if s['reason'] == 'WALK_JOYSTICK' else 'FRONTAL')
        w, a = ic['wanted_mrad'], ic['applied_mrad']
        res.setdefault(c, []).append(math.hypot(w[0] - a[0], w[1] - a[1]))
    return res


def results(run):
    return {r['step'].split()[0]: r for r in json.loads((Path(run) / 'results.json').read_text())}


def main():
    a, b = sys.argv[1], sys.argv[2]
    A, B = trials(Path(a) / 'analyze_diag.txt', True), trials(Path(b) / 'analyze_diag.txt', True)
    names = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E-F'}
    print('1. per block and class: ms/release  N+C ms  k instr  IPC  istall_mem   (before -> after)')
    for t in range(1, 6):
        for c in ('WALK', 'RECOVERY', 'OTHER'):
            x, y = A.get((t, c)), B.get((t, c))
            if not x and not y:
                continue
            f = lambda v: '%6.2f %5.2f %5s %.2f %.2f' % (v['wall'], v['nc'], v.get('ki', '-'), v.get('ipc', 0), v.get('im', 0)) if v else '   -'
            print('  %-4s %-9s %s  ->  %s' % (names[t], c, f(x), f(y)))
    if len(sys.argv) > 4:
        for lab, p in (('H before', sys.argv[3]), ('H after', sys.argv[4])):
            for (t, c), v in sorted(trials(Path(p) / 'analyze_diag.txt').items()):
                print('  %-9s %-9s %6.2f %5.2f %5s %.2f %.2f' % (lab, c, v['wall'], v['nc'], v.get('ki'), v.get('ipc', 0), v.get('im', 0)))
    print('2. heaviest release since boot (release, wall us, nominal us, compensate us)')
    ha, hb = heaviest(Path(a) / 'analyze_diag.txt'), heaviest(Path(b) / 'analyze_diag.txt')
    for c in ('WALK', 'RECOVERY', 'OTHER'):
        print('  %-9s %s  ->  %s' % (c, ha.get(c), hb.get(c)))
    print('3. release work per step class (firmware histogram of each command, last sample)')
    for lab, run in (('before', a), ('after', b)):
        res = results(run)
        hist = work_hist_by_class(run)
        agg = {}
        for cid, (reason, h) in hist.items():
            agg.setdefault(reason, [0] * 8)
            agg[reason] = [x + y for x, y in zip(agg[reason], h)]
        for reason in sorted(agg):
            h = agg[reason]
            n = sum(h)
            if n < 50:
                continue
            print('  %-6s %-42s n %6d  >=20 ms %5.2f %%  p99 bin < %s ms' % (lab, reason[:42], n, 100.0 * sum(h[4:]) / n, p99_bin(h)))
    print('4. missed / overruns')
    for lab, run in (('before', a), ('after', b)):
        r = json.loads((Path(run) / 'results.json').read_text())
        print('  %-6s steps %d PASS %d missed %d overruns %d' % (lab, len(r), sum(x['result'] == 'PASS' for x in r),
                                                              sum(x.get('missed_delta', 0) for x in r), sum(x.get('overrun_delta', 0) for x in r)))
        for m in misses(run):
            print('     missed +%d at %s  %s -> %s' % (m[1], m[0], m[2], m[3]))
    print('5. IMU |wanted - applied| (mrad, 1 Hz)')
    ia, ib = imu(a), imu(b)
    for c in ('FRONTAL', 'STEER', 'RECOVERY'):
        f = lambda v: 'n %3d mean %.2f p90 %.2f max %.2f' % (len(v), st.mean(v), sorted(v)[int(.9 * (len(v) - 1))], max(v)) if v else '-'
        print('  %-9s %s  ->  %s' % (c, f(ia.get(c, [])), f(ib.get(c, []))))
    print('6. STOP verification and resume')
    ra, rb = results(a), results(b)
    for k, r in (('B4', 'B6'), ('D14', 'D15'), ('D18', 'D19'), ('F4', 'F5')):
        if k in ra and k in rb:
            print('  %-4s stop %.1f -> %.1f s (%s -> %s); %s %s -> %s s' % (k, ra[k]['stop_verified_s'], rb[k]['stop_verified_s'], ra[k]['resume_reason'], rb[k]['resume_reason'],
                                                                     r, ra[r].get('complete_s', ra[r].get('walking_after_s')), rb[r].get('complete_s', rb[r].get('walking_after_s'))))


if __name__ == '__main__':
    main()
