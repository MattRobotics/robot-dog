#!/usr/bin/env python3
"""MATDOG fixture validation suite, 15/11/2026, installed build 8530d3a11ac9.

One start, then every command of the sequence below is issued automatically
through the firmware's existing USB commands (CommandRouter: @STOP,
@MOTION STAND/PRONE/WALK STANDARD/WALK STOP/WALK MODE, @MOTION JSON with owner
"local" for the joystick and the 32 mm/s preset), each one checked on its ACK,
on the real Controller state over HTTP and on its completion before the next.

Recording: this process is the single USB owner (same no-reset opener as the
dev.8u recorder: os.open, flock+TIOCEXCL, no DTR/RTS) and writes serial.log and
events.jsonl; the unchanged http_watch.py, started on the same run directory,
writes http.jsonl at 1 Hz and queues the read-only @MOTION TIMING (ctl.d) each
time Motion is no longer engaged. Only @MOTION TIMING and QUIT are accepted
from ctl.d. analyze_diag.py reads the result.

Safety (the firmware's own protections stay authoritative):
 - before every movement: BMS comm OK, fresh, alarms 0, health not FAULT, IMU
   valid, Motion not faulted, no STOP in progress, 12 joints fresh with status 0,
   temperature < 65 C and voltage >= 10.0 V, no command in progress, expected state;
 - during every step: a FAULT, a BMS loss, an unexpected state or a step
   timeout ends the suite; if the robot is still engaged and not faulted, one
   @STOP is sent. No automatic resume, no reset, no retry.
 - commands come only from the fixed sequence below; joystick and preset values
   are checked against the limits the device publishes (capabilities).
Usage: fixture_suite.py <run-dir> [--plan]   (--plan prints the sequence only)"""
import datetime, importlib.util, json, os, re, sys, threading, time, urllib.request, fcntl, termios, signal
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OPENER = '/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_TIMING_HTTP_RESOLUTION_20261027/cr5_stand50_session.py'
PORT = '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00'
BASE = 'http://192.168.4.1'
EXPECT_BUILD = '8530d3a11ac9'
PRESET2_VX = 0.1 / 5.0          # kStrideM[2] / kWalkPeriodS: MotionService maps it to preset 2 (STANDARD 90 mm, FAST 80 mm)
CTL_ALLOWED = {'@MOTION TIMING', 'QUIT'}


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


class Abort(Exception):
    pass


class Suite:
    def __init__(self, run):
        self.run = run
        self.lock = threading.Lock()
        self.acks = []            # (host_time, dict)
        self.lines = []           # (host_time, text) of interesting lines
        self.timing_hist_t = 0.0  # host time of the last 'MOTION_TIMING hist' line
        self.motion = None
        self.motion_t = 0.0
        self.status = None
        self.status_t = 0.0
        self.offset_ms = None     # device_ms - host_ms
        self.seq = 0
        self.stop_flag = False
        self.results = []
        self.caps = None
        self.term = False

    # ---------------- recording ----------------
    def event(self, kind, **k):
        with self.lock:
            self.ev.write(json.dumps({'host_utc': utc(), 'host_monotonic': time.monotonic(), 'kind': kind, **k}) + '\n')

    def open(self):
        spec = importlib.util.spec_from_file_location('qualified', OPENER)
        self.q = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.q)
        self.run.mkdir()
        (self.run / 'ctl.d').mkdir()
        assert not self.q.serial_owners(PORT), 'SERIAL OCCUPIED'
        self.fd = self.q.open_serial(PORT)
        self.log = (self.run / 'serial.log').open('x', buffering=1)
        self.ev = (self.run / 'events.jsonl').open('x', buffering=1)
        self.event('session_start', port=os.path.realpath(PORT))
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.poller, daemon=True).start()
        threading.Thread(target=self.ctl_loop, daemon=True).start()
        self.tx('')  # the first line after opening can be lost

    def tx(self, c):
        p = (c + '\n').encode()
        with self.lock:
            if os.write(self.fd, p) != len(p):
                raise RuntimeError('PARTIAL USB WRITE')
            self.log.write(utc() + ' >>> ' + c + '\n')

    def reader(self):
        buf = b''
        while not self.stop_flag:
            try:
                d = os.read(self.fd, 16384)
            except BlockingIOError:
                d = b''
            if not d:
                time.sleep(.005)
                continue
            buf += d
            while b'\n' in buf:
                x, buf = buf.split(b'\n', 1)
                s = x.rstrip(b'\r').decode(errors='replace')
                now = time.time()
                with self.lock:
                    self.log.write(utc() + ' ' + s + '\n')
                if s.startswith('{"version":1,"ack":true'):
                    try:
                        a = json.loads(s)
                        with self.lock:
                            self.acks.append((now, a))
                    except ValueError:
                        pass
                elif s.startswith('MOTION_TIMING hist'):
                    self.timing_hist_t = now
                elif s.startswith('MOTION=REJECTED') or s.startswith('MOTION_TIMING=REFUSED') or 'panic' in s.lower() or 'brownout' in s.lower():
                    with self.lock:
                        self.lines.append((now, s))
                    print('  USB:', s[:120], flush=True)

    def ctl_loop(self):
        ctl = self.run / 'ctl.d'
        while not self.stop_flag:
            for f in sorted(ctl.glob('*.cmd')):
                c = f.read_text().strip()
                f.rename(f.with_suffix('.sent'))
                if c not in CTL_ALLOWED:
                    self.event('ctl_refused', cmd=c)
                    continue
                self.event('ctl_cmd', cmd=c)
                if c == 'QUIT':
                    self.stop_flag = True
                    break
                self.tx(c)
            time.sleep(.05)

    def get(self, path, timeout=1.5):
        t0 = time.time()
        with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
            body = r.read()
        return body, t0, time.time()

    def poller(self):
        k = 0
        while not self.stop_flag:
            try:
                body, t0, t1 = self.get('/api/motion/status')
                m = json.loads(body)
                with self.lock:
                    self.motion, self.motion_t = m, t1
                    self.offset_ms = m['now_ms'] - (t0 + t1) / 2 * 1000
            except Exception as e:  # recorded; a stale status fails every check
                self.event('http_error', path='/api/motion/status', error=str(e)[:120])
            if k % 4 == 0:
                try:
                    body, t0, t1 = self.get('/status')
                    st = dict(l.split('=', 1) for l in body.decode(errors='replace').splitlines() if '=' in l)
                    with self.lock:
                        self.status, self.status_t = st, t1
                except Exception as e:
                    self.event('http_error', path='/status', error=str(e)[:120])
            k += 1
            time.sleep(.5)

    # ---------------- checks ----------------
    def snap(self):
        with self.lock:
            return self.motion, self.motion_t, self.status, self.status_t

    def health(self, m, st, mt, stt):
        """Conditions that end the suite whatever the step (None = fine)."""
        now = time.time()
        if m is None or now - mt > 3.0:
            return 'MOTION_STATUS_STALE'
        if st is None or now - stt > 8.0:
            return 'SYSTEM_STATUS_STALE'
        if m.get('fault') or m.get('state') == 'FAULT':
            return 'FAULT:' + str(m.get('reason'))
        if st.get('bms_comm') != 'OK' or st.get('bms_fresh') != 'YES':
            return 'BMS_LOST:%s fail_streak=%s rx=%s' % (st.get('bms_comm'), st.get('bms_fail_streak'), st.get('bms_rx_bytes'))
        if st.get('bms_alarms', '').replace(' ', '').strip('0'):
            return 'BMS_ALARM:' + st.get('bms_alarms', '')
        if st.get('system_health') == 'FAULT':
            return 'SYSTEM_HEALTH_FAULT'
        if not m.get('imu_valid'):
            return 'IMU_INVALID'
        return None

    def pre(self, states, label):
        """Before every movement: health plus a resting, coherent robot."""
        deadline = time.time() + 6
        while True:
            m, mt, st, stt = self.snap()
            why = self.health(m, st, mt, stt)
            if not why:
                j = m['joints']
                bad = [x['joint'] for x in j if not x['valid'] or x['age_ms'] > 500 or x['status'] or x['temperature'] >= 65 or x['voltage_dv'] < 100]
                if m['state'] not in states:
                    why = 'STATE %s not in %s' % (m['state'], states)
                elif bad:
                    why = 'JOINTS ' + ','.join(bad)
                elif m['stop']['in_progress']:
                    why = 'STOP_IN_PROGRESS'
                elif m['active_command']['outcome'] == 'ACCEPTED':
                    why = 'COMMAND_IN_PROGRESS ' + m['active_command']['reason']
                elif m.get('power_reason') != 'READY':
                    why = 'POWER ' + str(m.get('power_reason'))
            if not why:
                self.event('pre_ok', label=label, state=m['state'], reason=m['reason'], bms_rx=st.get('bms_rx_bytes'), bms_first=st.get('bms_first_byte_ms'))
                return m
            if time.time() > deadline:
                raise Abort('PRECONDITION %s: %s' % (label, why))
            time.sleep(.5)

    def wait(self, label, pred, timeout, allowed=None):
        t0 = time.time()
        while True:
            if self.term:
                raise Abort('%s: SIGTERM (external stop request)' % label)
            m, mt, st, stt = self.snap()
            why = self.health(m, st, mt, stt)
            if why:
                raise Abort('%s: %s' % (label, why))
            if allowed and m['state'] not in allowed:
                raise Abort('%s: unexpected state %s/%s (%s)' % (label, m['state'], m['phase'], m['reason']))
            if pred(m):
                return m, time.time() - t0
            if time.time() - t0 > timeout:
                raise Abort('%s: timeout %.0f s in %s/%s (%s)' % (label, timeout, m['state'], m['phase'], m['reason']))
            time.sleep(.2)

    # ---------------- commands ----------------
    def ack_for(self, n0, want_id=None, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                new = self.acks[n0:]
            for _, a in new:
                if want_id is None or a.get('id') == want_id:
                    return a
            with self.lock:
                rej = [s for t, s in self.lines if t >= t0 - .05 and s.startswith('MOTION=REJECTED')]
            if rej:
                return {'outcome': 'REJECTED', 'reason': rej[-1]}
            time.sleep(.02)
        return None

    def command(self, line, label, ok=('ACCEPTED',)):
        with self.lock:
            n0 = len(self.acks)
        self.tx(line)
        a = self.ack_for(n0)
        self.event('ack', label=label, command=line, ack=a)
        if not a or a.get('outcome') not in ok:
            raise Abort('%s: ACK %s' % (label, a))
        return a

    def joy_json(self, vx, vy, yaw, gait=True):
        # The range is the one of SimpleMotion::joystickMode(), published as
        # status walk_mode.joystick when it differs from walk_mode.selected
        # (MotionService.cpp:325); the dashboard scales the same way (joyMode).
        # walk_mode.active is the law of the last walk (SLOW at boot), not this.
        wm = self.motion['walk_mode']
        mode = wm.get('joystick') or wm['selected']
        spec = next(w for w in self.caps['walk_modes'] if w['name'] == mode)
        lim = spec['joystick']
        if gait:
            top = spec['top_speed_mm_s'] / 1000
            assert -lim['vx_back_max_m_s'] - 1e-12 <= vx <= top + 1e-12, 'joystick vx outside the published range'
            norm = (vx / top) ** 2 + sum((v / r) ** 2 if r > 0 else (0 if v == 0 else 9) for v, r in
                                         ((vy, lim['vy_max_m_s']), (yaw, lim['yaw_rate_max_rad_s'])))
            assert norm <= 1 + 1e-9, 'joystick outside the published ellipsoid (%s %.6f)' % (mode, norm)
        else:
            assert vx == PRESET2_VX and vy == 0 and yaw == 0, 'only the 32 mm/s (FAST) / 30 mm/s (STANDARD) preset is sent this way'
        self.seq += 1
        issued = int(time.time() * 1000 + self.offset_ms - 150)
        params = {'vx': vx, 'vy': vy, 'yaw_rate': yaw}
        if gait:
            params['gait'] = 'JOYSTICK'
        cid = 'suite-%d' % self.seq
        body = json.dumps({'version': 1, 'id': cid, 'sequence': self.seq, 'issued_at_ms': issued, 'ttl_ms': 1000,
                           'owner': 'local', 'type': 'BASE_VELOCITY', 'params': params}, separators=(',', ':'))
        return cid, '@MOTION JSON ' + body

    def json_command(self, vx, vy, yaw, label, gait=True, ok=('ACCEPTED',)):
        cid, line = self.joy_json(vx, vy, yaw, gait)
        with self.lock:
            n0 = len(self.acks)
        self.tx(line)
        a = self.ack_for(n0, want_id=cid)
        self.event('ack', label=label, command=line, ack=a)
        if not a or a.get('outcome') not in ok:
            raise Abort('%s: ACK %s' % (label, a))
        return a

    # ---------------- steps ----------------
    def step(self, name, fn):
        m0, _, st0, _ = self.snap()
        t0 = time.time()
        self.event('step_start', step=name)
        print('%s STEP %s' % (utc()[11:19], name), flush=True)
        try:
            info = fn() or {}
        except Abort as e:
            self.results.append({'step': name, 'result': 'FAIL', 'why': str(e), 'seconds': round(time.time() - t0, 1)})
            self.event('step_fail', step=name, why=str(e))
            raise
        m1, _, st1, _ = self.snap()
        r = {'step': name, 'result': 'PASS', 'seconds': round(time.time() - t0, 1), **info,
             'missed_delta': m1['missed_releases'] - m0['missed_releases'], 'overrun_delta': m1['overrun_releases'] - m0['overrun_releases'],
             'command_timing': m1['command_timing'], 'max_release_work_us': m1['max_release_work_us']}
        self.results.append(r)
        self.event('step_pass', **r)
        print('   PASS %.1f s missed +%d overruns +%d' % (r['seconds'], r['missed_delta'], r['overrun_delta']), flush=True)
        return r

    def stand(self, mm, from_states=('READY', 'STOPPED', 'STANDING'), resume=False):
        if not resume:
            self.pre(from_states, 'STAND %d' % mm)
        a = self.command('@MOTION STAND %d' % mm, 'STAND %d' % mm)
        cid = a['id']
        m, dt = self.wait('STAND %d' % mm, lambda m: m['active_command']['id'] == cid and m['active_command']['outcome'] in ('COMPLETED', 'ABORTED'),
                          150, allowed=('MOVING', 'STANDING', 'WALKING', 'STOPPED', 'READY'))
        if not (m['active_command']['outcome'] == 'COMPLETED' and m['active_command']['reason'] == 'STAND_REACHED' and m['state'] == 'STANDING'):
            raise Abort('STAND %d ended %s/%s state %s' % (mm, m['active_command']['outcome'], m['active_command']['reason'], m['state']))
        return {'ack_reason': a['reason'], 'complete_s': round(dt, 1), 'height_estimate_mm': m['height_estimate_mm'], 'requested_mm': m['requested_height_mm'],
                'imu_correction': m['imu_correction']}

    def prone(self):
        self.pre(('STANDING',), 'PRONE')
        a = self.command('@MOTION PRONE', 'PRONE')
        cid = a['id']
        m, dt = self.wait('PRONE', lambda m: m['active_command']['id'] == cid and m['active_command']['outcome'] in ('COMPLETED', 'ABORTED'), 180,
                          allowed=('MOVING', 'STANDING', 'READY', 'STOPPED'))
        if not (m['active_command']['reason'] == 'PRONE_REST' and m['safe_off_verified'] and not m['torque_active']):
            raise Abort('PRONE ended %s/%s safe_off=%s torque=%s' % (m['active_command']['outcome'], m['active_command']['reason'], m['safe_off_verified'], m['torque_active']))
        return {'complete_s': round(dt, 1), 'safe_off_mask': m['safe_off_mask']}

    def timing_dump(self, label):
        t0 = time.time()
        while time.time() - t0 < 12:          # http_watch queues it after two not-engaged samples
            if self.timing_hist_t > t0 - 1:
                break
            time.sleep(.2)
        if self.timing_hist_t <= t0 - 1:
            self.tx('@MOTION TIMING')         # read-only, refused by the firmware while engaged
            t1 = time.time()
            while time.time() - t1 < 5 and self.timing_hist_t <= t1:
                time.sleep(.1)
        ok = self.timing_hist_t > t0 - 1
        self.event('timing_dump', label=label, ok=ok)
        if not ok:
            raise Abort('TIMING dump %s missing' % label)
        return {'dump': 'OK'}

    def stop_rest(self):
        self.pre(('READY',), 'STOP at rest')
        self.command('@STOP', 'STOP at rest')
        m, dt = self.wait('STOP at rest', lambda m: m['state'] == 'STOPPED' and m['safe_off_verified'] and not m['torque_active']
                          and m['active_command']['outcome'] == 'COMPLETED', 15, allowed=('READY', 'STOPPED'))
        return {'complete_s': round(dt, 1), 'reason': m['reason'], 'ack_final': m['active_command']['reason']}

    def stop_walk(self, label):
        a = self.command('@STOP', label)
        t0 = time.time()
        m, dt = self.wait(label, lambda m: m['stop']['active'] and m['stop']['motion_verified'] and not m['stop']['in_progress'], 20,
                          allowed=('WALKING', 'MOVING', 'STOPPED'))
        return {'stop_verified_s': round(dt, 2), 'torque_active': m['torque_active'], 'state': m['state'], 'resume_allowed': m['stop']['stand_resume_allowed'],
                'resume_reason': m['stop']['stand_resume_reason'], 'walk_cycles': m['walk_cycles']}

    def walk_preset(self, preset, label):
        self.pre(('STANDING',), label)
        if preset == 1:
            self.command('@MOTION WALK STANDARD', label)
        else:
            self.json_command(PRESET2_VX, 0, 0, label, gait=False)
        m, dt = self.wait(label, lambda m: m['state'] == 'WALKING', 60, allowed=('STANDING', 'MOVING', 'WALKING'))
        return {'walking_after_s': round(dt, 1), 'mode': m['walk_mode'], 'vx': m['walk_velocity']['vx_m_s']}

    def walk_for(self, seconds, label):
        until = time.time() + seconds
        m, _ = self.wait(label, lambda m: time.time() >= until, seconds + 5, allowed=('WALKING',))
        return {'walk_cycles': m['walk_cycles'], 'imu_correction': m['imu_correction'], 'velocity': m['walk_velocity']}

    def held(self, seconds, label):
        until = time.time() + seconds
        m, _ = self.wait(label, lambda m: time.time() >= until, seconds + 5, allowed=('STOPPED',))
        if not (m['stop']['active'] and m['torque_active']):
            raise Abort('%s: STOP not held (active=%s torque=%s)' % (label, m['stop']['active'], m['torque_active']))
        return {'resume_allowed': m['stop']['stand_resume_allowed'], 'resume_reason': m['stop']['stand_resume_reason']}

    def walk_stop(self, label):
        a = self.command('@MOTION WALK STOP', label)
        cid = a['id']
        m, dt = self.wait(label, lambda m: m['state'] == 'STANDING' and m['active_command']['outcome'] == 'COMPLETED', 60,
                          allowed=('WALKING', 'MOVING', 'STANDING'))
        return {'standing_after_s': round(dt, 1), 'reason': m['active_command']['reason']}

    def joystick(self, vx, vy, yaw, seconds, label):
        """Hold the stick: refresh every 0.3 s; count the hold from WALKING; release = zero velocity (WALK STOP)."""
        self.pre(('STANDING',), label)
        self.json_command(vx, vy, yaw, label, ok=('ACCEPTED',))
        walking_at = None
        t0 = time.time()
        while True:
            if self.term:
                raise Abort('%s: SIGTERM (external stop request)' % label)
            m, mt, st, stt = self.snap()
            why = self.health(m, st, mt, stt)
            if why:
                raise Abort('%s: %s' % (label, why))
            if m['state'] not in ('STANDING', 'MOVING', 'WALKING'):
                raise Abort('%s: unexpected state %s (%s)' % (label, m['state'], m['reason']))
            if m['state'] == 'WALKING' and walking_at is None:
                walking_at = time.time()
                self.event('joystick_walking', label=label, after_s=round(walking_at - t0, 1))
            if walking_at and time.time() - walking_at >= seconds:
                break
            if walking_at is None and time.time() - t0 > 60:
                raise Abort('%s: not WALKING after 60 s (%s)' % (label, m['reason']))
            time.sleep(.3)
            self.json_command(vx, vy, yaw, label, ok=('ACCEPTED',))
        info = {'walking_after_s': round(walking_at - t0, 1), 'walk_cycles': m['walk_cycles'], 'velocity': m['walk_velocity'], 'imu_correction': m['imu_correction']}
        return info

    def joystick_release(self, label):
        self.json_command(0.0, 0.0, 0.0, label, ok=('ACCEPTED',))
        m, dt = self.wait(label, lambda m: m['state'] == 'STANDING' and m['active_command']['outcome'] == 'COMPLETED', 60,
                          allowed=('WALKING', 'MOVING', 'STANDING'))
        return {'standing_after_s': round(dt, 1), 'reason': m['active_command']['reason']}

    def gait(self, mode, ok_reasons):
        a = self.command('@MOTION WALK MODE %s' % mode, 'MODE %s' % mode)
        if a['reason'] not in ok_reasons:
            raise Abort('MODE %s: reason %s' % (mode, a['reason']))
        return {'reason': a['reason']}

    # ---------------- end ----------------
    def safe_end(self, why):
        """After an abort: one STOP if still engaged and not faulted; never resume."""
        m, mt, st, stt = self.snap()
        self.event('abort', why=why, state=m and m['state'], torque=m and m['torque_active'])
        print('ABORT:', why, flush=True)
        held = m and m['stop']['active'] and m['stop']['motion_verified'] and not m['stop']['in_progress']
        if held:
            self.event('abort_no_stop', why='STOP already verified (held)', state=m['state'], torque=m['torque_active'])
        elif m and not m.get('fault') and (m['torque_active'] or m['state'] in ('MOVING', 'WALKING', 'STANDING')):
            self.tx('@STOP')
            self.event('abort_stop_sent')
            t0 = time.time()
            while time.time() - t0 < 10:
                m, *_ = self.snap()
                if m and (m['stop']['motion_verified'] or m['safe_off_verified'] or m.get('fault')):
                    break
                time.sleep(.2)
            self.event('abort_stop_state', state=m['state'], stop=m['stop'], safe_off=m['safe_off_verified'], torque=m['torque_active'])

    def close(self):
        time.sleep(1.0)
        self.stop_flag = True
        time.sleep(.3)
        (self.run / 'results.json').write_text(json.dumps(self.results, indent=1))
        self.event('session_end')
        try:
            fcntl.ioctl(self.fd, termios.TIOCNXCL)
        except OSError:
            pass
        os.close(self.fd)
        self.log.close()
        self.ev.close()


def sequence(s):
    """The fixture validation sequence (each entry: step name, callable)."""
    S = s
    std = next(w for w in S.caps['walk_modes'] if w['name'] == 'STANDARD')['joystick']
    yaw, lat, back = std['yaw_rate_max_rad_s'], std['vy_max_m_s'], std['vx_back_max_m_s']
    seq = [
        # Block A: STOP at rest with torque OFF, explicit STAND re-arm (7878be0), heights, PRONE and back.
        ('A0 TIMING dump (baseline, Motion inactive)', lambda: S.timing_dump('baseline')),
        ('A1 STOP at rest (READY, torque OFF)', S.stop_rest),
        ('A2 STAND 120 after the STOP (ACQUIRE/REFERENCE/ENABLE)', lambda: S.stand(120, from_states=('STOPPED',))),
        ('A3 STAND 150', lambda: S.stand(150, from_states=('STANDING',))),
        ('A4 STAND 80', lambda: S.stand(80, from_states=('STANDING',))),
        ('A5 STAND 120', lambda: S.stand(120, from_states=('STANDING',))),
        ('A6 PRONE to PRONE_REST', S.prone),
        ('A7 TIMING dump (block A)', lambda: S.timing_dump('A')),
        ('A8 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        # Block B: the 14/11 WALK_SETTLE fault scenario (Patch A 4d4b797): STANDARD yaw clockwise 22 s, STOP, 4 s, STAND 120.
        ('B1 MODE STANDARD', lambda: S.gait('STANDARD', ('WALK_MODE_SELECTED',))),
        ('B2 STAND 110', lambda: S.stand(110, from_states=('STANDING',))),
        ('B3 joystick yaw clockwise full scale 22 s', lambda: S.joystick(0.0, 0.0, -yaw, 22, 'B3 yaw cw')),
        ('B4 STOP during the yaw walk', lambda: S.stop_walk('B4 STOP')),
        ('B5 wait 4 s held', lambda: S.held(4, 'B5 held')),
        ('B6 STAND 120 resume (WALK recovery, WALK_SETTLE, HEIGHT)', lambda: S.stand(120, resume=True)),
        ('B7 PRONE to PRONE_REST', S.prone),
        ('B8 TIMING dump (block B)', lambda: S.timing_dump('B')),
        # Block C: Timing window, WALK STANDARD preset 1 (75 mm, 25 mm/s) for 30 s, WALK STOP.
        ('C1 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        ('C2 WALK STANDARD preset 1', lambda: S.walk_preset(1, 'C2 WALK STANDARD')),
        ('C3 walk 30 s', lambda: S.walk_for(30, 'C3 walk 30 s')),
        ('C4 WALK STOP', lambda: S.walk_stop('C4 WALK STOP')),
        ('C5 PRONE to PRONE_REST', S.prone),
        ('C6 TIMING dump (block C)', lambda: S.timing_dump('C')),
        # Block D: backward, lateral and yaw both ways, STOP in different gait phases, STAND and WALK resumes.
        ('D1 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        ('D2 joystick backward full scale 12 s', lambda: S.joystick(-back, 0.0, 0.0, 12, 'D2 back')),
        ('D3 release (WALK STOP)', lambda: S.joystick_release('D3 release')),
        ('D4 joystick lateral left 12 s', lambda: S.joystick(0.0, lat, 0.0, 12, 'D4 left')),
        ('D5 release', lambda: S.joystick_release('D5 release')),
        ('D6 joystick lateral right 12 s', lambda: S.joystick(0.0, -lat, 0.0, 12, 'D6 right')),
        ('D7 release', lambda: S.joystick_release('D7 release')),
        ('D8 joystick yaw counter-clockwise 12 s', lambda: S.joystick(0.0, 0.0, yaw, 12, 'D8 yaw ccw')),
        ('D9 release', lambda: S.joystick_release('D9 release')),
        ('D10 joystick yaw clockwise 12 s', lambda: S.joystick(0.0, 0.0, -yaw, 12, 'D10 yaw cw')),
        ('D11 release', lambda: S.joystick_release('D11 release')),
        ('D12 WALK STANDARD preset 1', lambda: S.walk_preset(1, 'D12 WALK')),
        ('D13 walk 4.0 s', lambda: S.walk_for(4.0, 'D13 walk')),
        ('D14 STOP (phase 1)', lambda: S.stop_walk('D14 STOP')),
        ('D15 STAND 120 resume', lambda: S.stand(120, resume=True)),
        ('D16 WALK STANDARD preset 1', lambda: S.walk_preset(1, 'D16 WALK')),
        ('D17 walk 5.3 s', lambda: S.walk_for(5.3, 'D17 walk')),
        ('D18 STOP (phase 2)', lambda: S.stop_walk('D18 STOP')),
        ('D19 WALK resume after the STOP (preset 1)', lambda: S.walk_preset_resume()),
        ('D20 walk 6 s', lambda: S.walk_for(6, 'D20 walk')),
        ('D21 WALK STOP', lambda: S.walk_stop('D21 WALK STOP')),
        ('D22 PRONE to PRONE_REST', S.prone),
        ('D23 TIMING dump (block D)', lambda: S.timing_dump('D')),
        # Block E: walk-mode change during a forward walk (STANDARD -> SLOW -> STANDARD), qualified offline.
        ('E1 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        ('E2 WALK STANDARD preset 1', lambda: S.walk_preset(1, 'E2 WALK')),
        ('E3 walk 6 s', lambda: S.walk_for(6, 'E3 walk')),
        ('E4 MODE SLOW during the walk', lambda: S.gait('SLOW', ('WALK_MODE_CHANGING', 'WALK_MODE_NEXT_WALK'))),
        ('E5 walk 12 s', lambda: S.walk_for(12, 'E5 walk')),
        ('E6 MODE STANDARD during the walk', lambda: S.gait('STANDARD', ('WALK_MODE_CHANGING', 'WALK_MODE_NEXT_WALK'))),
        ('E7 walk 12 s', lambda: S.walk_for(12, 'E7 walk')),
        ('E8 WALK STOP', lambda: S.walk_stop('E8 WALK STOP')),
        # Block F: FAST 32 mm/s, preset 2 (80 mm, 2.5 s): walk, STOP, STAND resume, walk, WALK STOP.
        ('F1 MODE FAST', lambda: S.gait('FAST', ('WALK_MODE_SELECTED',))),
        ('F2 WALK FAST preset 2 (32 mm/s)', lambda: S.walk_preset(2, 'F2 WALK FAST')),
        ('F3 walk 15 s', lambda: S.walk_for(15, 'F3 walk')),
        ('F4 STOP during the FAST walk', lambda: S.stop_walk('F4 STOP')),
        ('F5 STAND 120 resume', lambda: S.stand(120, resume=True)),
        ('F6 WALK FAST preset 2', lambda: S.walk_preset(2, 'F6 WALK FAST')),
        ('F7 walk 10 s', lambda: S.walk_for(10, 'F7 walk')),
        ('F8 WALK STOP', lambda: S.walk_stop('F8 WALK STOP')),
        ('F9 MODE SLOW (default for the next session)', lambda: S.gait('SLOW', ('WALK_MODE_SELECTED',))),
        # Block G: final PRONE_REST, SAFE_OFF 13/13, torque OFF.
        ('G1 PRONE to PRONE_REST', S.prone),
        ('G2 TIMING dump (blocks E-F, final)', lambda: S.timing_dump('EF')),
    ]
    return seq


def walk_preset_resume(self):
    """dev.17: after a verified STOP, WALK is admitted exactly as STAND after STOP."""
    a = self.command('@MOTION WALK STANDARD', 'D19 WALK resume')

    # The resume passes through the walk's own HOLD (startWalk): STAND_REACHED,
    # then WALK_IMU_SETTLING while the correction slews into the walk envelope
    # (seen on the hardware 15/11 08:16:08). Any other STANDING reason ends it.
    def walking(m):
        if m['state'] == 'STANDING' and m['reason'] not in ('STAND_REACHED', 'WALK_IMU_SETTLING'):
            raise Abort('D19 WALK resume: STANDING/%s (%s)' % (m['phase'], m['reason']))
        return m['state'] == 'WALKING'
    m, dt = self.wait('D19 WALK resume', walking, 90, allowed=('STOPPED', 'MOVING', 'STANDING', 'WALKING'))
    return {'ack_reason': a['reason'], 'walking_after_s': round(dt, 1)}


Suite.walk_preset_resume = walk_preset_resume


def main():
    run = ROOT / sys.argv[1]
    s = Suite(run)
    caps = json.loads(urllib.request.urlopen(BASE + '/api/motion/capabilities', timeout=3).read())
    s.caps = caps
    seq = sequence(s)
    # --start <step>: continue a suite that ended on an abort, from that step;
    # the robot must be in the held STOP the abort left, resumed by STAND first.
    if '--start' in sys.argv:
        first = sys.argv[sys.argv.index('--start') + 1]
        k = next(i for i, (name, _) in enumerate(seq) if name.split()[0] == first)
        height = int(sys.argv[sys.argv.index('--resume-stand') + 1])
        skip = set(sys.argv[sys.argv.index('--skip') + 1].split(',')) if '--skip' in sys.argv else set()
        seq = [('R0 STAND %d resume from the held STOP left by the abort' % height, lambda: s.stand(height, from_states=('STOPPED',)))] + \
              [x for x in seq[k:] if x[0].split()[0] not in skip]
    if '--plan' in sys.argv:
        for name, _ in seq:
            print(name)
        return 0
    st = dict(l.split('=', 1) for l in urllib.request.urlopen(BASE + '/status', timeout=3).read().decode().splitlines() if '=' in l)
    assert st.get('build_id') == EXPECT_BUILD, 'unexpected build ' + str(st.get('build_id'))
    s.open()
    (run / 'capabilities.json').write_text(json.dumps(caps, indent=1))
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
        print('SUITE COMPLETE: %d/%d PASS' % (sum(r['result'] == 'PASS' for r in s.results), len(seq)), flush=True)
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
