#!/usr/bin/env python3
"""MATDOG fixture block H, 16/11/2026: STOP inside the stopped-walk RECOVERY (build 46b31757eb81).

A separate tool, run after the qualified A-G suite has ended in PRONE_REST. fixture_suite.py is
imported unchanged: its USB owner, HTTP poller, health checks, preconditions, commands, step
recording and abort handling are reused as they are; only the sequence below and one step
(stop_in_recovery) are new. The A-G suite file is not modified.

What it adds: the firmware path a STOP takes while the STAND recovery from a stopped walk runs
(state MOVING, reason STAND_FROM_STOPPED_WALK; walkRecover_ in SimpleMotion, on the T2 bracket
with T2R1M). Offline qualification of that firmware path: host scenarios R2_IN_RECOVERY_d1..d80
on the production SimpleMotion (P0 structural rig, base f3b0d48 and T2R1M 0fb22b0): every STOP
verified, the following STAND accepted, STAND_REACHED. The two instants used here, about 15 and
about 50 releases after the STAND, lie inside that covered range (d12..d25 and d40..d55).

The STOP is sent only if a status read taken after the STAND shows MOVING/STAND_FROM_STOPPED_WALK
for this STAND; otherwise the step ends as RECOVERY_WINDOW_MISSED (the shared abort handling then
sends one STOP if the robot is engaged and not faulted, as in A-G). No retry, no resume after a
FAULT or a BMS loss, no limit or box changed; joystick values are not used.

Usage: fixture_block_h.py <run-dir> [--plan]
Offline dry run only: MATDOG_SIM_PTY=<pty> MATDOG_SIM_BASE=http://127.0.0.1:<port> (sim_device.py)."""
import importlib.util, json, os, signal, sys, time, urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('fixture_suite', ROOT / 'fixture_suite.py')
fs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fs)
if os.environ.get('MATDOG_SIM_PTY'):          # offline dry run against sim_device.py only
    fs.PORT = os.environ['MATDOG_SIM_PTY']
    fs.BASE = os.environ['MATDOG_SIM_BASE']
Abort = fs.Abort
RELEASE_S = 0.020                              # period_us 20000


def stop_in_recovery(self, at_s, label):
    """STAND 120 from the held walk STOP, then @STOP at_s after the STAND line left the host,
    only while that STAND's recovery is running (checked on a fresh status read)."""
    self.pre(('STOPPED',), label)
    m0, *_ = self.snap()
    if not (m0['stop']['active'] and m0['stop']['motion_verified'] and m0['stop']['stand_resume_allowed']):
        raise Abort('%s: no verified STOP with STAND recovery available (%s)' % (label, m0['stop']))
    with self.lock:
        n0 = len(self.acks)
    t_cmd = time.time()
    self.tx('@MOTION STAND 120')
    a = self.ack_for(n0)
    self.event('ack', label=label + ' STAND', command='@MOTION STAND 120', ack=a)
    if not a or a.get('outcome') != 'ACCEPTED':
        raise Abort('%s: STAND ACK %s' % (label, a))
    cid = a['id']
    seen = None
    while True:
        if self.term:
            raise Abort('%s: SIGTERM (external stop request)' % label)
        m, mt, st, stt = self.snap()
        why = self.health(m, st, mt, stt)
        if why:
            raise Abort('%s: %s' % (label, why))
        if m['state'] not in ('STOPPED', 'MOVING'):
            raise Abort('%s: unexpected state %s/%s (%s)' % (label, m['state'], m['phase'], m['reason']))
        if time.time() - t_cmd >= at_s - 0.15:
            break
        time.sleep(.02)
    # A fresh read of the Controller state, taken after the STAND, decides whether the STOP is sent.
    body, g0, g1 = self.get('/api/motion/status')
    m = json.loads(body)
    in_recovery = (m['state'] == 'MOVING' and m['reason'] == 'STAND_FROM_STOPPED_WALK' and
                   m['active_command']['id'] == cid and not m['stop']['active'] and not m.get('fault'))
    self.event('recovery_check', label=label, ok=in_recovery, state=m['state'], phase=m['phase'], reason=m['reason'],
               releases=m['releases'], read_s=round(g1 - t_cmd, 3))
    if not in_recovery:
        raise Abort('%s: RECOVERY_WINDOW_MISSED %s/%s (%s) at %.2f s' % (label, m['state'], m['phase'], m['reason'], g1 - t_cmd))
    while time.time() - t_cmd < at_s:
        time.sleep(.005)
    t_stop = time.time()
    s = self.command('@STOP', label + ' STOP')
    stop_after = t_stop - t_cmd
    r0 = m['releases']
    # Only a status sampled on the device after this STOP counts: the poller's cached sample can still be
    # the previous held STOP (dry run 16/11: at 0.3 s the old verified STOP satisfied the check at once).
    stop_dev_ms = t_stop * 1000 + self.offset_ms
    m, dt = self.wait(label, lambda m: m['now_ms'] > stop_dev_ms and m['stop']['active'] and m['stop']['motion_verified']
                      and not m['stop']['in_progress'], 20, allowed=('MOVING', 'STOPPED'))
    return {'stand_ack': a['reason'], 'stop_ack': s['reason'], 'stop_after_stand_s': round(stop_after, 3),
            'recovery_releases_at_stop_est': round(stop_after / RELEASE_S), 'releases_at_check': r0,
            'stop_verified_s': round(dt, 2), 'torque_active': m['torque_active'], 'state': m['state'],
            'resume_allowed': m['stop']['stand_resume_allowed'], 'resume_reason': m['stop']['stand_resume_reason']}


fs.Suite.stop_in_recovery = stop_in_recovery


def sequence_h(s):
    S = s
    return [
        ('H1 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        ('H2 MODE STANDARD', lambda: S.gait('STANDARD', ('WALK_MODE_SELECTED',))),
        ('H3 WALK STANDARD preset 1', lambda: S.walk_preset(1, 'H3 WALK')),
        ('H4 walk 4.0 s', lambda: S.walk_for(4.0, 'H4 walk')),
        ('H5 STOP during the walk', lambda: S.stop_walk('H5 STOP')),
        ('H6 wait 3 s held', lambda: S.held(3, 'H6 held')),
        ('H7 STAND 120 resume, STOP 1.0 s into the RECOVERY (~50 releases)', lambda: S.stop_in_recovery(1.0, 'H7')),
        ('H8 wait 3 s held', lambda: S.held(3, 'H8 held')),
        ('H9 STAND 120 resume to STAND_REACHED', lambda: S.stand(120, resume=True)),
        ('H10 WALK STANDARD preset 1', lambda: S.walk_preset(1, 'H10 WALK')),
        ('H11 walk 5.3 s', lambda: S.walk_for(5.3, 'H11 walk')),
        ('H12 STOP during the walk', lambda: S.stop_walk('H12 STOP')),
        ('H13 wait 3 s held', lambda: S.held(3, 'H13 held')),
        ('H14 STAND 120 resume, STOP 0.3 s into the RECOVERY (~15 releases)', lambda: S.stop_in_recovery(0.3, 'H14')),
        ('H15 wait 3 s held', lambda: S.held(3, 'H15 held')),
        ('H16 STAND 120 resume to STAND_REACHED', lambda: S.stand(120, resume=True)),
        ('H17 MODE SLOW (default for the next session)', lambda: S.gait('SLOW', ('WALK_MODE_SELECTED',))),
        ('H18 PRONE to PRONE_REST', S.prone),
        ('H19 TIMING dump (block H)', lambda: S.timing_dump('H')),
    ]


def main():
    run = ROOT / sys.argv[1]
    s = fs.Suite(run)
    seq = sequence_h(s)
    if '--plan' in sys.argv:
        for name, _ in seq:
            print(name)
        return 0
    s.caps = json.loads(urllib.request.urlopen(fs.BASE + '/api/motion/capabilities', timeout=3).read())
    st = dict(l.split('=', 1) for l in urllib.request.urlopen(fs.BASE + '/status', timeout=3).read().decode().splitlines() if '=' in l)
    assert st.get('build_id') == fs.EXPECT_BUILD, 'unexpected build ' + str(st.get('build_id'))
    s.open()
    (run / 'capabilities.json').write_text(json.dumps(s.caps, indent=1))
    signal.signal(signal.SIGTERM, lambda *_: setattr(s, 'term', True))
    t0 = time.time()
    while s.motion is None or s.status is None or s.offset_ms is None:
        if time.time() - t0 > 10:
            print('no HTTP status')
            s.close()
            return 2
        time.sleep(.2)
    code = 0
    try:
        for name, fn in seq:
            s.step(name, fn)
        print('BLOCK H COMPLETE: %d/%d PASS' % (sum(r['result'] == 'PASS' for r in s.results), len(seq)), flush=True)
    except Abort as e:
        s.safe_end(str(e))
        code = 1
    except Exception as e:
        s.safe_end('EXCEPTION %r' % e)
        code = 3
    finally:
        s.close()
    return code


if __name__ == '__main__':
    sys.exit(main())
