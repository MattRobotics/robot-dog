#!/usr/bin/env python3
"""OFFLINE ONLY: a behavioural stand-in for the MATDOG Controller's USB and HTTP surface, used to dry-run
fixture_block_h.py (and the shared fixture_suite.py primitives it calls) before the hardware session.

It models only the published state sequence the 15/11 hardware recordings show (http.jsonl of the e321e86
suite: STOP -> MOVING/STOP/STOP_IN_PROGRESS -> STOPPED/STOPPED_SUPPORT_UNVERIFIED; STAND from the held walk
STOP -> MOVING/WALK_STANDARD/STAND_FROM_STOPPED_WALK ~4 s -> MOVING/HEIGHT/WALK_SETTLING -> HEIGHT_CHANGE ->
STANDING/STAND_REACHED) and the ACK line format of serial.log. It is not the firmware: Motion law, IK, STOP
proof and protections are not modelled; the firmware path itself is qualified by the host R2 scenarios.

Fault injection (SIM_MODE): nominal | short_recovery | fault_on_recovery_stop | bms_loss_in_hold | resume_refused
Usage: sim_device.py <port> <pty-name-file>   (writes the PTY slave path, then serves until killed)"""
import json, os, sys, threading, time, select, copy
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path

HERE = Path(__file__).resolve().parent
TEMPLATE = json.loads((HERE / 'template_motion_status.json').read_text())
CAPS = (HERE / 'capabilities.json').read_text()
MODE = os.environ.get('SIM_MODE', 'nominal')
T0 = time.time()
# 16/11 MEMSTOP tests: internal-heap model. SIM_HEAP_MIN0 = watermark at start; after the first
# /api/motion/capabilities request (a fixture tool's start), the watermark falls to SIM_DROP_TO at SIM_DROP_AT s.
BUILD = os.environ.get('SIM_BUILD', 'cb9ed85565f6')
HEAP = {'min': int(os.environ.get('SIM_HEAP_MIN0', '39424')), 'free': 60476, 't_caps': None}
DROP_AT = float(os.environ.get('SIM_DROP_AT', '-1'))
DROP_TO = int(os.environ.get('SIM_DROP_TO', '11144'))
SEEN = []          # (time, path) of every HTTP request, for the tests


def heap_now():
    if HEAP['t_caps'] is not None and DROP_AT >= 0 and time.time() - HEAP['t_caps'] >= DROP_AT:
        HEAP['min'] = min(HEAP['min'], DROP_TO)
    return HEAP['free'], HEAP['min']
L = threading.RLock()


class Dev:
    def __init__(self):
        self.m = copy.deepcopy(TEMPLATE)
        m = self.m
        m.update(state='READY', phase='IDLE', reason='PRONE_REST', fault=False, safe_off_verified=True, safe_off_mask=8191,
                 torque_active=False, missed_releases=0, overrun_releases=0, max_release_work_us=0, power_reason='READY')
        m['active_command'] = {'outcome': 'COMPLETED', 'reason': 'PRONE_REST', 'id': 'boot', 'sequence': 0, 'accepted_ms': 0, 'finished_ms': 0}
        m['stop'] = self.stop_dict(False, False, False, False, 'STOP_NOT_ACTIVE')
        m['walk_mode'] = {'selected': 'SLOW', 'active': 'SLOW', 'target': 'SLOW'}
        self.seq = 0
        self.timers = []
        self.origin = None          # phase the held STOP came from
        self.in_recovery = False
        self.recovery_stop = False
        self.bms_ok = True
        self.out = []

    @staticmethod
    def stop_dict(active, prog, verified, allowed, why):
        return {'active': active, 'in_progress': prog, 'motion_verified': verified, 'support_verified': False,
                'stand_resume_allowed': allowed, 'stand_resume_reason': why}

    def ack(self, outcome, reason):
        self.seq += 1
        cid = 'usb-%d' % self.seq
        self.out.append(json.dumps({'version': 1, 'ack': True, 'id': cid, 'sequence': self.seq, 'owner': 'local', 'outcome': outcome,
                                    'reason': reason, 'executor_state': self.m['state']}, separators=(',', ':')))
        return cid

    def at(self, dt, fn):
        self.timers.append((time.time() + dt, fn))

    def cancel(self):
        self.timers = []

    def set(self, **k):
        self.m.update(k)

    def cmd_done(self, cid, outcome, reason):
        self.m['active_command'] = {'outcome': outcome, 'reason': reason, 'id': cid, 'sequence': self.seq, 'accepted_ms': 0, 'finished_ms': 0}

    def standing(self, cid, h):
        self.set(state='STANDING', phase='HOLD', reason='STAND_REACHED', requested_height_mm=float(h), height_estimate_mm=float(h) - 0.1)
        self.cmd_done(cid, 'COMPLETED', 'STAND_REACHED')
        self.in_recovery = False

    def handle(self, c):
        m = self.m
        if c == '':
            return
        print('%.3f CMD %s' % (time.time(), c), flush=True)
        if c == '@MOTION TIMING':
            if m['torque_active']:
                self.out.append('MOTION_TIMING=REFUSED engaged')
            else:
                self.out.append('MOTION_TIMING hist work=1,2,3,0,0,0,0,0 read=1,0,0,0 lateness=1,0,0,0,0,0')
            return
        if c == '@STOP':
            cid = self.ack('ACCEPTED', 'STOP_REQUESTED')
            if not m['torque_active']:
                self.set(state='STOPPED', reason='SAFE_OFF_VERIFIED')
                self.cmd_done(cid, 'COMPLETED', 'SAFE_OFF_VERIFIED')
                return
            self.cancel()
            self.recovery_stop = self.in_recovery
            self.origin = 'WALK' if (m['state'] == 'WALKING' or self.in_recovery) else 'HOLD'
            self.in_recovery = False
            self.set(state='MOVING', phase='STOP', reason='STOP_IN_PROGRESS')
            m['stop'] = self.stop_dict(True, True, False, False, 'STOP_IN_PROGRESS')
            self.cmd_done(cid, 'ACCEPTED', 'STOP_IN_PROGRESS')
            if MODE == 'fault_on_recovery_stop' and self.recovery_stop:
                def fault():
                    self.set(state='FAULT', phase='FAULT', reason='STOP_HOLD_TRACKING_UNSAFE', fault=True, torque_active=False, safe_off_verified=True)
                self.at(0.6, fault)
                return

            def verified():
                self.set(state='STOPPED', reason='STOPPED_SUPPORT_UNVERIFIED')
                allowed = not (MODE == 'resume_refused' and self.recovery_stop)
                m['stop'] = self.stop_dict(True, False, True, allowed, 'STAND_RECOVERY_AVAILABLE' if allowed else 'STOP_RECOVERY_PATH_NOT_CLEAR')
                self.cmd_done(cid, 'COMPLETED', 'STOPPED_SUPPORT_UNVERIFIED')
                if MODE == 'bms_loss_in_hold' and self.recovery_stop:
                    self.at(1.0, lambda: setattr(self, 'bms_ok', False))
            self.at(2.0, verified)
            return
        if c.startswith('@MOTION STAND '):
            h = int(c.split()[-1])
            if m['state'] in ('MOVING', 'WALKING'):
                self.ack('REJECTED', 'MOTION_BUSY')
                return
            if m['state'] == 'STOPPED' and m['stop']['active'] and m['torque_active']:
                if not m['stop']['stand_resume_allowed']:
                    self.ack('REJECTED', m['stop']['stand_resume_reason'])
                    return
                cid = self.ack('ACCEPTED', 'COMMAND_ACCEPTED')
                m['stop'] = self.stop_dict(False, False, False, False, 'STOP_NOT_ACTIVE')
                self.cmd_done(cid, 'ACCEPTED', 'COMMAND_ACCEPTED')
                if self.origin == 'WALK':
                    self.in_recovery = True
                    self.set(state='MOVING', phase='WALK_STANDARD', reason='STAND_FROM_STOPPED_WALK')
                    rec = 0.2 if MODE == 'short_recovery' else 4.0

                    def settle():
                        self.in_recovery = False
                        self.set(phase='HEIGHT', reason='WALK_SETTLING')
                    self.at(rec, settle)
                    self.at(rec + 1.0, lambda: self.set(reason='HEIGHT_CHANGE'))
                    self.at(rec + 2.0, lambda: self.standing(cid, h))
                else:
                    self.set(state='MOVING', phase='HEIGHT', reason='HEIGHT_CHANGE')
                    self.at(2.0, lambda: self.standing(cid, h))
                return
            cid = self.ack('ACCEPTED', 'COMMAND_ACCEPTED')
            self.cmd_done(cid, 'ACCEPTED', 'COMMAND_ACCEPTED')
            if m['state'] in ('READY', 'STOPPED'):
                m['stop'] = self.stop_dict(False, False, False, False, 'STOP_NOT_ACTIVE')
                self.set(state='MOVING', phase='ENABLE', reason='ACQUIRING_REAL_POSE', torque_active=True, safe_off_verified=False, safe_off_mask=0)
                self.at(3.0, lambda: self.standing(cid, h))
            else:
                self.set(state='MOVING', phase='HEIGHT', reason='HEIGHT_CHANGE')
                self.at(1.5, lambda: self.standing(cid, h))
            return
        if c == '@MOTION PRONE':
            if m['state'] != 'STANDING':
                self.ack('REJECTED', 'PRONE_REQUIRES_STAND_HOLD')
                return
            cid = self.ack('ACCEPTED', 'COMMAND_ACCEPTED')
            self.cmd_done(cid, 'ACCEPTED', 'COMMAND_ACCEPTED')
            self.set(state='MOVING', phase='PRONE', reason='PRONE_DESCENT_NOT_VALIDATED')

            def rest():
                self.set(state='READY', phase='IDLE', reason='PRONE_REST', torque_active=False, safe_off_verified=True, safe_off_mask=8191)
                self.cmd_done(cid, 'COMPLETED', 'PRONE_REST')
            self.at(3.0, rest)
            return
        if c.startswith('@MOTION WALK MODE '):
            mode = c.split()[-1]
            if m['state'] == 'STANDING':
                m['walk_mode']['selected'] = mode
                self.ack('ACCEPTED', 'WALK_MODE_SELECTED')
            else:
                self.ack('ACCEPTED', 'WALK_MODE_NEXT_WALK')
            return
        if c == '@MOTION WALK STANDARD':
            if m['state'] != 'STANDING':
                self.ack('REJECTED', 'MOTION_BUSY')
                return
            cid = self.ack('ACCEPTED', 'COMMAND_ACCEPTED')
            self.cmd_done(cid, 'ACCEPTED', 'COMMAND_ACCEPTED')
            self.set(state='MOVING', phase='WALK_STANDARD', reason='WALK_IMU_SETTLING')
            m['walk_mode']['active'] = m['walk_mode']['selected']
            self.at(1.2, lambda: self.set(state='WALKING', reason='WALK_STANDARD'))
            return
        if c == '@MOTION WALK STOP':
            if m['state'] != 'WALKING':
                self.ack('REJECTED', 'NOT_WALKING')
                return
            cid = self.ack('ACCEPTED', 'COMMAND_ACCEPTED')
            self.set(state='MOVING', reason='WALK_STOPPING')
            self.at(2.0, lambda: self.standing(cid, 120))
            return
        self.out.append('MOTION=REJECTED UNKNOWN_SIM_COMMAND ' + c)

    def tick(self):
        now = time.time()
        due = [t for t in self.timers if t[0] <= now]
        self.timers = [t for t in self.timers if t[0] > now]
        for _, fn in sorted(due, key=lambda x: x[0]):
            fn()
        m = self.m
        m['now_ms'] = int((now - T0) * 1000)
        m['releases'] = int((now - T0) * 50)
        if m['state'] == 'WALKING':
            m['walk_cycles'] = round(m.get('walk_cycles', 0) + 0.002, 3)
        for j in m['joints']:
            j['torque'] = 1 if m['torque_active'] else 0


D = Dev()


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        with L:
            D.tick()
            SEEN.append((time.time(), self.path))
            if self.path == '/api/motion/capabilities' and HEAP['t_caps'] is None:
                HEAP['t_caps'] = time.time()
            hf, hm = heap_now()
            if self.path == '/api/motion/status':
                D.m.setdefault('transport', {}).update(heap_free=hf, heap_min_free=hm, heap_largest=31732)
                body = json.dumps(D.m).encode()
            elif self.path == '/api/motion/capabilities':
                body = CAPS.encode()
            elif self.path == '/status':
                ok = D.bms_ok
                body = ('build_id=%s\nheap_free=%d\nheap_min_free=%d\n' % (BUILD, hf, hm) + 'firmware_version=1.0.2-dev.8\nsystem_health=READY\npower_state=RUN\n'
                        'bms_comm=%s\nbms_fresh=%s\nbms_alarms=0000 0000 0000 0000\nbms_rx_bytes=%s\nbms_first_byte_ms=15\n'
                        'bms_fail_streak=%s\nbms_d2_samples=900\nbms_d2_rx_low=40\n' % ('OK' if ok else 'TIMEOUT', 'YES' if ok else 'NO',
                                                                                     129 if ok else 0, 0 if ok else 3)).encode()
            else:
                self.send_response(404)
                self.end_headers()
                return
        self.send_response(200)
        self.end_headers()
        self.wfile.write(body)


def pty_loop(master):
    buf = b''
    while True:
        r, _, _ = select.select([master], [], [], 0.02)
        if r:
            try:
                d = os.read(master, 4096)
            except OSError:          # no slave open yet (EIO)
                d = b''
                time.sleep(.02)
            buf += d
            while b'\n' in buf:
                x, buf = buf.split(b'\n', 1)
                with L:
                    D.tick()
                    D.handle(x.decode(errors='replace').strip())
        with L:
            D.tick()
            out, D.out = D.out, []
        for s in out:
            try:
                os.write(master, (s + '\r\n').encode())
            except OSError:
                pass


def main():
    port = int(sys.argv[1])
    master, slave = os.openpty()
    name = os.ttyname(slave)
    os.close(slave)              # the tool under test must be the only owner of the slave
    Path(sys.argv[2]).write_text(name)
    threading.Thread(target=pty_loop, args=(master,), daemon=True).start()
    ThreadingHTTPServer(('127.0.0.1', port), H).serve_forever()


if __name__ == '__main__':
    main()
