#!/usr/bin/env python3
"""Read-only per-step check of a fixture run (19/11, block D / S9 completion): for every step, from the run's own
status samples (poller.jsonl + http.jsonl, /api/motion/status only), the executed walk velocity while WALKING, the
IMU correction reasons, faults and fault-like reasons, and per joint |raw - commanded_raw| (ticks) with torque ON.
raw is the joint's last FAST read, taken age_ms before the snapshot, commanded_raw the frame written at the snapshot:
the difference is tracking error plus the motion during age_ms, a feedback indicator, not a pure tracking error.
Usage: steer_check.py <run-dir> [...]"""
import collections, json, sys
from pathlib import Path

BAD = ('WORKSPACE_FAULT', 'WALK_IK_OR_WORKSPACE', 'OUTSIDE_BOX', 'PATH_NOT_CLEAR')


def samples(run):
    out = []
    for name in ('poller.jsonl', 'http.jsonl'):
        p = run / name
        if not p.exists():
            continue
        for line in open(p):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            s = r.get('s')
            if r.get('path') == '/api/motion/status' and isinstance(s, dict) and 'state' in s:
                out.append((r['host_utc'], s))
    return sorted(out, key=lambda x: x[0])


def steps(run):
    ev = []
    for line in open(run / 'events.jsonl'):
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get('kind') in ('step_start', 'session_end'):
            ev.append((e['host_utc'], e.get('step', 'END')))
    return [(name, t0, t1) for (t0, name), (t1, _) in zip(ev, ev[1:]) if name != 'END']


def main(run):
    run = Path(run)
    ss, res = samples(run), {r['step']: r for r in json.loads((run / 'results.json').read_text())}
    print('== %s: %d status samples' % (run.name, len(ss)))
    track = collections.defaultdict(list)
    faults = [(u, s['state'], s['reason']) for u, s in ss if s.get('fault') or s['state'] == 'FAULT' or any(b in str(s.get('reason')) for b in BAD)]
    for name, t0, t1 in steps(run):
        win = [s for u, s in ss if t0 <= u < t1]
        walk = [s for s in win if s['state'] == 'WALKING']
        reasons = collections.Counter((s.get('imu_correction') or {}).get('reason') for s in win if s.get('torque_active'))
        line = '%-62s %3d smp' % (name[:62], len(win))
        if walk:
            v = [(s['walk_velocity']['vx_m_s'], s['walk_velocity']['vy_m_s'], s['walk_velocity']['yaw_rate_rad_s']) for s in walk]
            rng = ['%+.4f..%+.4f' % (min(x[i] for x in v), max(x[i] for x in v)) for i in range(3)]
            line += ' | WALKING %2d vx %s vy %s yaw %s joy %s' % (len(walk), rng[0], rng[1], rng[2], walk[-1]['walk_velocity'].get('joystick'))
        r = res.get(name, {})
        if 'velocity' in r:
            line += ' | end-of-step velocity %s' % json.dumps(r['velocity'], separators=(',', ':'))
        if reasons:
            line += ' | imu %s' % dict(reasons)
        print(line)
        for s in win:
            if not s.get('torque_active'):
                continue
            for j in s['joints']:
                if j.get('valid') and j.get('commanded_raw') is not None:
                    track[j['joint']].append((abs(j['raw'] - j['commanded_raw']), j['commanded_raw'], j['raw'], name))
    print('faults or fault-like reasons: %s' % (faults or 'none'))
    for jn, xs in track.items():
        d = sorted(x[0] for x in xs)
        worst = max(xs, key=lambda x: x[0])
        print('  %-9s n %4d |raw-cmd| p50 %3d p95 %3d max %3d ticks (max in %s) | commanded %d..%d raw %d..%d' % (
            jn, len(d), d[len(d) // 2], d[int(.95 * (len(d) - 1))], d[-1], worst[3][:30],
            min(x[1] for x in xs), max(x[1] for x in xs), min(x[2] for x in xs), max(x[2] for x in xs)))


if __name__ == '__main__':
    for a in sys.argv[1:]:
        main(a)
