#!/usr/bin/env python3
"""MATDOG four-leg Full Calibration - one-shot hardware session runner.

    python3 scripts/calibration_hw_session.py \\
        --evidence-dir ~/MATDOG/evidence/<session> \\
        --backup ~/MATDOG/backups/esp32/<fullflash>.bin --backup-sha256 <hex> \\
        --confirm-q0-pose [--legs LF,RF,RH,LH] [--no-flash]

What it does, in order, stopping at the FIRST failed check:
  1. verifies the committed tree + build manifest (CLEAN, ROBOT_POWERED,
     OTA ingest 0, SOURCE_COMMIT == HEAD, binary SHA256 == manifest);
  2. application-only flash through the canonical scripts/flash_app_only.sh;
  3. waits for USB re-enumeration, opens the port WITHOUT resetting it;
  4. SOURCE_SIGNATURE == HEAD, MODE=MAINTENANCE, health READY;
  5. SAFE_OFF all 13 installed servos (13/13 VERIFIED_OFF);
  6. fresh Q0 CAPTURE (the operator asserts the q=0 pose with
     --confirm-q0-pose), 12 values vs CR2-C, hard stop at |delta| >= 82;
  7. Q0 PROMOTE, transforms_admitted=12;
  8. for each leg (default LF -> RF -> RH -> LH): SESSION START, PERMIT
     GRANT, FULL LEG, monitor until the exact terminal record
     ^CALIBRATION_FULL_LEG_RESULT leg=<L> verdict=... failure=...$,
     then verify cleanup (SAFE_OFF, authority NONE, session COMPLETED,
     HARDWARE_CONTACT_CALIBRATED record);
  9. SAFE_OFF 13/13, EVIDENCE EXPORT, exact END summary for all legs run.

Any failure: FULL LEG ABORT + SESSION ABORT (if the link is alive), SAFE_OFF
all 13, a read-only evidence export, then stop - no further leg.

Hard rules (2026-09-29 session):
  - every decision is an ANCHORED regex against one whole record line. The
    informational "CALIBRATION_FULL_LEG_NOTE HARDWARE_CONTACT_CALIBRATED = ..."
    line is never a result (the script that read it as one caused an
    EXECUTOR_FAILED on 2026-09-29);
  - nothing but "@CALIBRATION FULL LEG STATUS" is sent while a run is in
    flight, except the always-allowed ABORTs on a watchdog/link fault;
  - no command here can bypass a firmware gate: it only issues the operator
    commands the firmware already exposes, and the firmware re-checks them.
"""
import argparse
import datetime
import errno
import hashlib
import json
import os
import re
import select
import subprocess
import sys
import termios
import threading
import time

DEFAULT_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00"
INSTALLED = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51]
CR2C = {11: 2087, 12: 2100, 13: 1996, 21: 1985, 22: 2092, 23: 2030,
        31: 2034, 32: 2042, 33: 2081, 41: 2073, 42: 2089, 43: 2035}
HALF_TOOTH_TICKS = 82
# hip, upper, lower bus; rear park (Geometry V5's UPPER MAX auxiliary) and its bus.
LEG_MATRIX = {
    "LF": (13, 12, 11, "LH_UPPER", 42),
    "RF": (23, 22, 21, "RH_UPPER", 32),
    "RH": (33, 32, 31, "NONE", 0),
    "LH": (43, 42, 41, "NONE", 0),
}
# The six contacts of a leg, in the LF V25 measurement order.
CONTACT_ORDER = [("UPPER", "MIN"), ("UPPER", "MAX"), ("LOWER", "MIN"), ("LOWER", "MAX"),
                 ("HIP", "MIN"), ("HIP", "MAX")]
# LF_UPPER's true MIN stop was found by hand ~23 ticks past the canonical
# contact (2026-09-29). A LF UPPER MIN contact far from that is stopped for
# review before any other leg runs.
LF_MIN_EXPECTED_BEYOND = (7, 39)
# One whole 24-contact leg: recovery of 12 joints, 6 two-pass searches, the
# prerequisite and return moves, cleanup. Generous; the firmware bounds every
# move itself - this only catches a hung run.
LEG_WATCHDOG_S = 1800.0
RECOVERY_WATCHDOG_S = 300.0
POLL_S = 2.0
SILENCE_S = 10.0


class SessionFailure(Exception):
    pass


def now():
    return datetime.datetime.now().isoformat(timespec="milliseconds")


# --------------------------------------------------------------------------
# Transport: a no-reset native USB CDC link (never pyserial, never DTR/RTS,
# never a speed change; see memory esp32s3-readonly-probe-without-reset).
# --------------------------------------------------------------------------

class SerialLink:
    def __init__(self, port, log):
        self.port = port
        self.log = log
        self.lines = []          # (monotonic, text)
        self.cond = threading.Condition()
        self.lost = False
        self.fd = None
        self._stop = False
        self._thread = None

    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
                   termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB)
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
        self.log.write("SYS", f"PORT_OPEN {self.port} (raw, speed untouched, no DTR/RTS ioctl)")
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        buf = b""
        while not self._stop:
            try:
                r, _, _ = select.select([self.fd], [], [], 0.05)
                if not r:
                    continue
                data = os.read(self.fd, 4096)
                if data == b"":
                    raise OSError(errno.EIO, "EOF")
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                self.log.write("SYS", f"DEVICE_LOST {e}")
                with self.cond:
                    self.lost = True
                    self.cond.notify_all()
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.rstrip(b"\r").decode("utf-8", "replace")
                self.log.write("RX", text)
                with self.cond:
                    self.lines.append((time.monotonic(), text))
                    self.cond.notify_all()

    def send(self, text):
        if self.lost:
            raise SessionFailure(f"link lost; not sending {text!r}")
        os.write(self.fd, (text + "\n").encode())
        self.log.write("TX", text)

    def close(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.log.write("SYS", "PORT_CLOSED")


class Log:
    def __init__(self, path, echo=True):
        self.f = open(path, "a", buffering=1)
        self.echo = echo
        self.lock = threading.Lock()

    def write(self, tag, text):
        with self.lock:
            if not self.f.closed:
                self.f.write(f"{now()} {tag} {text}\n")

    def say(self, text):
        self.write("RUN", text)
        if self.echo:
            print(text, flush=True)

    def close(self):
        with self.lock:
            self.f.close()


# --------------------------------------------------------------------------
# Session: every exchange waits for ANCHORED record lines only.
# --------------------------------------------------------------------------

class Session:
    def __init__(self, link, log, clock=time.monotonic, sleep=time.sleep):
        self.link = link
        self.log = log
        self.clock = clock
        self.sleep = sleep

    def mark(self):
        with self.link.cond:
            return len(self.link.lines)

    def lines_since(self, mark):
        with self.link.cond:
            return [t for _, t in self.link.lines[mark:]]

    def wait_for(self, pattern, mark, timeout, fail_patterns=()):
        """First line after `mark` fully matching `pattern` (re.fullmatch)."""
        rx = re.compile(pattern)
        frx = [re.compile(p) for p in fail_patterns]
        deadline = self.clock() + timeout
        seen = mark
        while True:
            with self.link.cond:
                lines = [t for _, t in self.link.lines[seen:]]
                seen = len(self.link.lines)
                lost = self.link.lost
            for text in lines:
                m = rx.fullmatch(text)
                if m:
                    return m
                for f in frx:
                    if f.fullmatch(text):
                        raise SessionFailure(f"refused: {text}")
            if lost:
                raise SessionFailure("link lost while waiting for " + pattern)
            if self.clock() >= deadline:
                raise SessionFailure(f"timeout ({timeout}s) waiting for {pattern}")
            with self.link.cond:
                if len(self.link.lines) == seen and not self.link.lost:
                    self.link.cond.wait(timeout=0.05)

    def request(self, command, pattern, timeout=5.0, fail_patterns=(r"REASON=.*",)):
        mark = self.mark()
        self.link.send(command)
        return self.wait_for(pattern, mark, timeout, fail_patterns)

    # -- steps ---------------------------------------------------------------

    def verify_signature(self, expected_build_id):
        m = self.request("@SYSTEM SOURCE_SIGNATURE",
                         r"SOURCE_SIGNATURE build_id=(\S+) firmware=MATDOG Controller "
                         r"version=\S+ profile=(\S+) board=.*")
        if m.group(1) != expected_build_id or m.group(2) != "ROBOT_POWERED":
            raise SessionFailure(f"SOURCE_SIGNATURE build_id={m.group(1)} profile={m.group(2)}, "
                                 f"expected {expected_build_id} ROBOT_POWERED")
        self.log.say(f"PASS  source signature {m.group(1)} ROBOT_POWERED")

    def verify_maintenance(self):
        self.request("@MODE STATUS", r"MODE=MAINTENANCE", fail_patterns=(r"MODE=RUN",))
        self.log.say("PASS  MODE=MAINTENANCE")

    def wait_health_ready(self, timeout=20.0):
        deadline = self.clock() + timeout
        while True:
            m = self.request("@STATUS", r"SYSTEM health=(\S+) .*mode=MAINTENANCE authority=(\S+) .*")
            if m.group(1) == "READY":
                self.log.say(f"PASS  health READY, authority {m.group(2)}")
                return
            if self.clock() >= deadline:
                raise SessionFailure(f"system health {m.group(1)}, not READY")
            self.sleep(1.0)

    def safe_off(self, ids):
        ok = 0
        for bus in ids:
            m = self.request(f"@SERVO SAFE_OFF {bus}", rf"SERVO_SAFE_OFF id={bus} result=(\S+)")
            if m.group(1) != "VERIFIED_OFF":
                raise SessionFailure(f"SAFE_OFF {bus} -> {m.group(1)}")
            ok += 1
        return ok

    def safe_off_all(self):
        n = self.safe_off(INSTALLED)
        self.log.say(f"PASS  SAFE_OFF {n}/{len(INSTALLED)} VERIFIED_OFF")

    def authority_none(self):
        m = self.request("@AUTHORITY STATUS", r"AUTHORITY owner=(\S+) generation=\d+ last_result=\S+")
        if m.group(1) != "NONE":
            raise SessionFailure(f"authority owner={m.group(1)}, expected NONE")

    def q0_capture(self):
        mark = self.mark()
        self.link.send("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE")
        self.wait_for(r"CALIBRATION_Q0=STARTED session=\d+ samples_per_joint=9 stability_ticks=16",
                      mark, 5.0, (r"CALIBRATION_Q0=(REFUSED|BLOCKED|BUSY)",))
        self.wait_for(r"CALIBRATION_Q0 state=COMPLETE failure=NONE session=\d+ sample_passes=9/9 "
                      r"next_joint=\d+ candidates=12/12", mark, 90.0,
                      (r"CALIBRATION_Q0 state=FAILED.*",))
        self.wait_for(r"CALIBRATION_Q0_POPULATION status=PASS verdict=PASS observed=12/12", mark, 5.0)
        self.wait_for(r"CALIBRATION_Q0_RESULT=12_CANDIDATES_ONLY", mark, 5.0)
        q0 = {}
        rx = re.compile(r"  Q0 bus=(\d+) leg=\w+ joint=\w+ unit=\S+ tick=(\d+) spread=(\d+) "
                        r"samples=(\d+) state=CANDIDATE estimator=MANUAL_ZERO_POSE")
        for text in self.lines_since(mark):
            m = rx.fullmatch(text)
            if m:
                q0[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))
        if sorted(q0) != sorted(CR2C):
            raise SessionFailure(f"Q0 capture reported buses {sorted(q0)}, expected 12 leg joints")
        self.log.say("      bus  q0    CR2-C  delta  spread")
        worst = 0
        for bus in sorted(q0):
            tick, spread = q0[bus]
            delta = tick - CR2C[bus]
            worst = max(worst, abs(delta))
            self.log.say(f"      {bus:3d}  {tick:4d}  {CR2C[bus]:4d}  {delta:+5d}  {spread}")
        if worst >= HALF_TOOTH_TICKS:
            raise SessionFailure(f"q0 differs from CR2-C by {worst} >= {HALF_TOOTH_TICKS} ticks "
                                 f"(half a spline tooth): re-check the q=0 pose / assembly")
        self.log.say(f"PASS  Q0 12/12, max |delta vs CR2-C| = {worst}")
        return {b: t for b, (t, _) in q0.items()}

    def q0_promote(self):
        self.request("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION",
                     r"CALIBRATION_Q0_PROMOTE=OK admitted=12/12 source=CURRENT_BOOT_CAPTURE "
                     r"capture_session=\d+",
                     fail_patterns=(r"CALIBRATION_Q0_PROMOTE=(PARTIAL|REFUSED|BLOCKED|BUSY).*",))
        self.request("@ACTUATOR STATUS",
                     r"ACTUATOR_PROVENANCE limits_admitted=0 transforms_admitted=12 geometry_bound=YES")
        self.log.say("PASS  Q0 promoted 12/12, transforms_admitted=12")

    # -- servo positions (read-only, evidence only) --------------------------

    def read_positions(self, label):
        """@SERVO READ of every leg joint - read-only, refused by the firmware
        itself while an executor is active. Evidence, never a decision."""
        out = {}
        for bus in sorted(CR2C):
            m = self.request(f"@SERVO READ {bus}",
                             rf"SERVO_READ id={bus} position=(-?\d+) speed=(-?\d+) load=(-?\d+) "
                             r"voltage=(-?\d+) temp=(-?\d+) torque=(-?\d+) current=(-?\d+)",
                             fail_patterns=(r"SERVO_READ=BLOCKED", rf"SERVO_READ id={bus} result=\S+"))
            out[bus] = (int(m.group(1)), int(m.group(6)))
        self.log.say(f"      {label}: " + " ".join(f"{b}:{p}{'*' if t else ''}"
                                                  for b, (p, t) in sorted(out.items())))
        return out

    # -- one leg's session + permit ---------------------------------------------

    def open_leg_session(self, leg):
        self.authority_none()
        self.request(f"@CALIBRATION SESSION START {leg} CONFIRM_CURRENT_Q0",
                     rf"CALIBRATION_SESSION=ACTIVE leg={leg} session=\d+ authority_generation=\d+",
                     fail_patterns=(r"CALIBRATION_SESSION=REFUSED",))
        mark = self.mark()
        self.link.send("@CALIBRATION STATUS")
        self.wait_for(r"CALIBRATION_POPULATION verdict=PASS observed=12/12 mask=0xfff", mark, 5.0)
        self.request("@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION",
                     r"CALIBRATION_MOTION_PERMIT=ACTIVE generation=\d+ session=\d+ "
                     r"authority_generation=\d+ direction_verify_budget_ticks=16",
                     fail_patterns=(r"CALIBRATION_MOTION_PERMIT=REFUSED",))
        self.log.say(f"PASS  {leg} session ACTIVE, permit ACTIVE")

    def leg_session_ready(self, leg):
        """True when a live <leg> session with an active permit already exists
        (the state `--phase recover` leaves for the operator's GO)."""
        mark = self.mark()
        self.link.send("@CALIBRATION STATUS")
        m = self.wait_for(r"CALIBRATION_SESSION state=(\S+) origin=(\S+) leg=(\S+) .*", mark, 5.0)
        p = self.wait_for(r"CALIBRATION_MOTION_PERMIT_STATE active=(YES|NO) "
                          r"operator_authorized=(YES|NO) token_valid=(YES|NO)", mark, 5.0)
        return (m.group(1) == "ACTIVE" and m.group(2) == "LIVE_SESSION" and m.group(3) == leg and
                p.group(1, 2, 3) == ("YES", "YES", "YES"))

    # -- INITIAL RECOVERY: all twelve actively to q0, verified -------------------

    def initial_recovery(self, leg, q0=None):
        mark = self.mark()
        self.link.send(f"@CALIBRATION INITIAL RECOVERY {leg} CONFIRM_Q0_RECOVERY")
        self.wait_for(rf"CALIBRATION_INITIAL_RECOVERY=ARMED session_leg={leg} joints=12 "
                      r"torque_limit=500 phase=PREFLIGHT", mark, 5.0,
                      (r"CALIBRATION_INITIAL_RECOVERY=REFUSED",))
        rx = re.compile(r"CALIBRATION_INITIAL_RECOVERY_TARGET bus=(\d+) leg=\w+ joint=\w+ unit=\S+ q0=(\d+)")
        targets = {}
        deadline = self.clock() + 3.0
        while len(targets) < 12 and self.clock() < deadline:
            for text in self.lines_since(mark):
                m = rx.fullmatch(text)
                if m:
                    targets[int(m.group(1))] = int(m.group(2))
            self.sleep(0.05)
        if sorted(targets) != sorted(CR2C):
            self.link.send("@CALIBRATION FULL LEG ABORT")
            raise SessionFailure(f"INITIAL RECOVERY targets for buses {sorted(targets)}, expected 12")
        for bus, tick in targets.items():
            if abs(tick - CR2C[bus]) >= HALF_TOOTH_TICKS or (q0 is not None and q0.get(bus) != tick):
                self.link.send("@CALIBRATION FULL LEG ABORT")
                raise SessionFailure(f"INITIAL RECOVERY target bus {bus} q0={tick} is not the promoted q0")
        self.log.say(f"RUN   INITIAL RECOVERY: 12 joints -> promoted q0, one at a time")
        result = self._monitor(
            re.compile(r"CALIBRATION_INITIAL_RECOVERY_RESULT verdict=(\S+) recovered=(\d+)/(\d+) "
                       r"failure=(\S+) failed_phase=(\S+) last_decision=(\S+)"),
            mark, f"{leg} INITIAL RECOVERY", RECOVERY_WATCHDOG_S)
        verdict, recovered, total, failure = result.group(1, 2, 3, 4)
        self.log.say(f"RESULT INITIAL RECOVERY verdict={verdict} recovered={recovered}/{total} "
                     f"failure={failure}")
        if verdict != "PASS" or recovered != "12" or total != "12" or failure != "NONE":
            raise SessionFailure(f"INITIAL RECOVERY {result.group(0)}")
        self.log.say("PASS  INITIAL RECOVERY 12/12: every leg joint actively at its promoted q0, "
                     "settled, SAFE_OFF verified")

    # -- one TRUE Full-Leg run: six contacts -------------------------------------

    def run_full_leg(self, leg, lf_crosscheck=True):
        hip, upper, lower, park, park_bus = LEG_MATRIX[leg]
        mark = self.mark()
        self.link.send(f"@CALIBRATION FULL LEG {leg} CONFIRM_FULL_CALIBRATION")
        armed = self.wait_for(r"CALIBRATION_FULL_LEG=ARMED leg=(\S+) contacts_expected=(\d+) "
                              r"hip_bus=(\d+) upper_bus=(\d+) lower_bus=(\d+) park=(\S+) "
                              r"park_bus=(\d+) phase=PREFLIGHT", mark, 5.0,
                              (r"CALIBRATION_FULL_LEG=REFUSED",))
        got = (armed.group(1), int(armed.group(2)), int(armed.group(3)), int(armed.group(4)),
               int(armed.group(5)), armed.group(6), int(armed.group(7)))
        if got != (leg, 6, hip, upper, lower, park, park_bus):
            self.link.send("@CALIBRATION FULL LEG ABORT")
            raise SessionFailure(f"{leg} ARMED {armed.group(0)} does not match the 24-contact plan")
        crx = re.compile(r"CALIBRATION_FULL_LEG_SEARCH_CORRIDOR joint=(UPPER|LOWER|HIP) side=(MIN|MAX) "
                         r"probe_sign=(-?1) q0=(\d+) contact=(\d+) urdf_limit=(\d+) entry=(\d+) "
                         r"guard=(\d+) opposite_limit=(\d+) guard_beyond_contact=(-?\d+)")
        prx = re.compile(r"CALIBRATION_FULL_LEG_PREREQUISITE pose=(\S+) urad=(-?\d+) tick=(\d+)")
        corridors, poses = {}, {}
        deadline = self.clock() + 3.0
        want_poses = 5 if park != "NONE" else 4
        while (len(corridors) < 6 or len(poses) < want_poses) and self.clock() < deadline:
            for text in self.lines_since(mark):
                m = crx.fullmatch(text)
                if m:
                    corridors[(m.group(1), m.group(2))] = m
                m = prx.fullmatch(text)
                if m:
                    poses[m.group(1)] = (int(m.group(2)), int(m.group(3)))
            self.sleep(0.05)
        if len(corridors) != 6 or len(poses) != want_poses:
            self.link.send("@CALIBRATION FULL LEG ABORT")
            raise SessionFailure(f"{leg}: {len(corridors)}/6 corridors, {len(poses)}/{want_poses} "
                                 f"prerequisite poses reported on ARMED")
        for key in CONTACT_ORDER:
            c = corridors[key]
            self.log.say(f"      {leg} {key[0]:5s} {key[1]}: q0={c.group(4)} contact={c.group(5)} "
                         f"urdf_limit={c.group(6)} entry={c.group(7)} guard={c.group(8)}")
        for name, (urad, tick) in sorted(poses.items()):
            self.log.say(f"      {leg} prerequisite {name}: {urad} urad = tick {tick}")
        self.log.say(f"RUN   {leg} TRUE Full Calibration ARMED: 6 contacts, park {park}")

        self._monitor(re.compile(rf"CALIBRATION_FULL_LEG_CONTACTS leg={leg} expected=6 measured=(\d+) "
                                 r"accepted=(\d+) diagnostics_accepted=(YES|NO)"),
                      mark, f"{leg} FULL LEG", LEG_WATCHDOG_S, stop_on_match=False)
        result = self._monitor(re.compile(rf"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict=(\S+) "
                                          r"failure=(\S+)"),
                               mark, f"{leg} FULL LEG", LEG_WATCHDOG_S)
        contacts = self.wait_for(rf"CALIBRATION_FULL_LEG_CONTACTS leg={leg} expected=6 measured=(\d+) "
                                 r"accepted=(\d+) diagnostics_accepted=(YES|NO)", mark, 1.0)
        verdict, failure = result.group(1), result.group(2)
        self.log.say(f"RESULT {leg} verdict={verdict} failure={failure} contacts "
                     f"{contacts.group(2)}/6 (measured {contacts.group(1)}, diagnostics "
                     f"{contacts.group(3)})")
        if verdict != "HARDWARE_CONTACT_CALIBRATED" or failure != "NONE" or \
                contacts.group(1, 2, 3) != ("6", "6", "YES"):
            raise SessionFailure(f"{leg} verdict={verdict} failure={failure} "
                                 f"contacts={contacts.group(2)}/6")

        # Cleanup, exactly as the runbook requires before the next leg.
        self.safe_off_all()
        self.authority_none()
        mark = self.mark()
        self.link.send("@CALIBRATION STATUS")
        self.wait_for(rf"CALIBRATION_SESSION state=COMPLETED origin=LIVE_SESSION leg={leg} .*", mark, 5.0)
        mark = self.mark()
        self.link.send("@CALIBRATION FULL LEG STATUS")
        self.wait_for(rf"CALIBRATION_FULL_LEG_RECORD leg={leg} present=YES attempts=\d+ "
                      r"verdict=HARDWARE_CONTACT_CALIBRATED contacts_accepted=6/6 "
                      r"contact_calibrated=YES envelope_accepted=NO failure=NONE", mark, 5.0)
        rx = re.compile(r"CALIBRATION_FULL_LEG_CONTACT joint=(UPPER|LOWER|HIP) side=(MIN|MAX) "
                        r"measured=YES scout=(\d+) fine1=(\d+) fine2=(\d+) witness_accepted=YES")
        seen = {}
        deadline = self.clock() + 3.0
        while len(seen) < 6 and self.clock() < deadline:
            for text in self.lines_since(mark):
                m = rx.fullmatch(text)
                if m:
                    seen[(m.group(1), m.group(2))] = (int(m.group(3)), int(m.group(4)),
                                                      int(m.group(5)))
            self.sleep(0.05)
        if sorted(seen) != sorted(CONTACT_ORDER):
            raise SessionFailure(f"{leg}: status shows {len(seen)}/6 accepted contacts")
        for key in CONTACT_ORDER:
            c = corridors[key]
            sign, contact = int(c.group(3)), int(c.group(5))
            scout, f1, f2 = seen[key]
            beyond = (f2 - contact) * sign
            self.log.say(f"      {leg} {key[0]:5s} {key[1]} contact scout={scout} (reference) "
                         f"fine1={f1} fine2={f2} = canonical {beyond:+d}")
            if leg == "LF" and key == ("UPPER", "MIN") and lf_crosscheck and \
                    not (LF_MIN_EXPECTED_BEYOND[0] <= beyond <= LF_MIN_EXPECTED_BEYOND[1]):
                raise SessionFailure(
                    f"LF UPPER MIN contact is canonical {beyond:+d}, but the stop was found by hand "
                    f"at about +23: stopping for operator review before any other leg "
                    f"(--no-lf-min-crosscheck to accept)")
        self.log.say(f"PASS  {leg} 6/6 HARDWARE_CONTACT_CALIBRATED; cleanup verified")

    def _monitor(self, terminal, mark, label, watchdog_s, stop_on_match=True):
        """Polls FULL LEG STATUS (the only command sent while a run is in
        flight); returns the first line after `mark` that fully matches
        `terminal`. With stop_on_match=False it only waits for it."""
        event = re.compile(r"CALIBRATION_SEARCH exec=(\S+) joint=(\S+) side=(MIN|MAX) pass=(\d) "
                           r"stage=(\S+) probe=(\S+) target=(\d+) pos=(-?\d+) beyond_contact=(-?\d+) .*")
        seq = re.compile(r"CALIBRATION_SEQUENCE leg=\S+ phase=(\S+) step=(\S+) joint=\S+ bus=(\d+) "
                         r"target=(\d+) held=(\d+) contacts=(\d+)/6 recovered=(\d+) prerequisites=\S+ "
                         r"failure=(\S+)")
        final = re.compile(r"CALIBRATION_FULL_LEG_PROBE_FINAL .*")
        start = self.clock()
        last_poll = -POLL_S
        last_rx = self.clock()
        seen = mark
        aborted_at = None
        session_aborted = False
        last_state = None
        last_phase = None
        while True:
            with self.link.cond:
                new = [t for _, t in self.link.lines[seen:]]
                seen = len(self.link.lines)
                lost = self.link.lost
            if new:
                last_rx = self.clock()
            for text in new:
                m = terminal.fullmatch(text)
                if m:
                    return m
                q = seq.fullmatch(text)
                if q and q.group(1) != last_phase:
                    last_phase = q.group(1)
                    self.log.say(f"      {self.clock() - start:6.1f}s phase {q.group(1)} "
                                 f"contacts={q.group(6)}/6 recovered={q.group(7)} "
                                 f"failure={q.group(8)}")
                e = event.fullmatch(text)
                if e:
                    state = e.group(1, 2, 3, 4, 5, 6)
                    if state != last_state:
                        last_state = state
                        self.log.say(f"      {self.clock() - start:6.1f}s {e.group(1)} {e.group(2)} "
                                     f"{e.group(3)} pass {e.group(4)} {e.group(5)} {e.group(6)} "
                                     f"target={e.group(7)} pos={e.group(8)} "
                                     f"beyond_contact={e.group(9)}")
                if final.fullmatch(text):
                    self.log.say("      " + text)
            if lost:
                raise SessionFailure(f"{label}: USB link lost during an energized run - "
                                     f"use the physical disconnect; nothing can be sent")
            t = self.clock()
            if t - last_poll >= POLL_S:
                self.link.send("@CALIBRATION FULL LEG STATUS")
                last_poll = t
            if aborted_at is None and (t - start > watchdog_s or t - last_rx > SILENCE_S):
                self.log.say(f"!!    {label}: watchdog (elapsed {t - start:.0f}s, silence "
                             f"{t - last_rx:.0f}s) -> FULL LEG ABORT")
                self.link.send("@CALIBRATION FULL LEG ABORT")
                aborted_at = t
            if aborted_at is not None and not session_aborted and t - aborted_at > 30.0:
                self.link.send("@CALIBRATION SESSION ABORT")
                session_aborted = True
            if aborted_at is not None and t - aborted_at > 60.0:
                raise SessionFailure(f"{label}: no terminal record 60 s after ABORT")
            with self.link.cond:
                if len(self.link.lines) == seen and not self.link.lost:
                    self.link.cond.wait(timeout=0.05)

    def export(self):
        mark = self.mark()
        self.link.send("@CALIBRATION EVIDENCE EXPORT")
        end = self.wait_for(r"CALIBRATION_EVIDENCE_EXPORT=END legs_present=(\d+) "
                            r"legs_contact_calibrated=(\d+) legs_envelope_accepted=(\d+) "
                            r"total_contacts_expected=24 total_contacts_accepted=(\d+) "
                            r"all_contact_calibrated=(\d)", mark, 30.0,
                            (r"CALIBRATION_EVIDENCE_EXPORT=REFUSED",))
        lines = self.lines_since(mark)
        return end, lines

    def verify_export(self, legs, end, lines):
        """Every leg run must be a complete 6/6 record; the four-leg result
        (all_contact_calibrated=1) is claimed only at 4 legs x 6 = 24/24."""
        n = len(legs)
        want = (str(n), str(n), "0", str(6 * n), "1" if n == 4 else "0")
        if end.group(1, 2, 3, 4, 5) != want:
            raise SessionFailure(f"export summary {end.group(0)} != legs_present={n} "
                                 f"legs_contact_calibrated={n} legs_envelope_accepted=0 "
                                 f"total_contacts_accepted={6 * n} all_contact_calibrated={want[4]}")
        leg_rx = re.compile(r"CALIBRATION_EVIDENCE_LEG leg=(\S+) present=1 attempts=\d+ session=\d+ "
                            r"geometry=\S+ verdict=HARDWARE_CONTACT_CALIBRATED contacts_expected=6 "
                            r"contacts_measured=6 contacts_accepted=6 diagnostics_accepted=1 "
                            r"contact_calibrated=1 envelope_accepted=0")
        close_rx = re.compile(r"CALIBRATION_EVIDENCE_LEG_CLOSE leg=(\S+) failure=NONE "
                              r"executor_failure=NONE failed_phase=- session_completed=1 "
                              r"permit_revoked=1 authority_released=1 parameters_approved=0")
        contact_rx = re.compile(r"CALIBRATION_EVIDENCE_CONTACT leg=(\S+) joint=(UPPER|LOWER|HIP) "
                                r"side=(MIN|MAX) recorded=1 measured=1 .* witness_accepted=1")
        legs_ok = {m.group(1) for m in (leg_rx.fullmatch(t) for t in lines) if m}
        close_ok = {m.group(1) for m in (close_rx.fullmatch(t) for t in lines) if m}
        contacts = {}
        for t in lines:
            m = contact_rx.fullmatch(t)
            if m:
                contacts.setdefault(m.group(1), set()).add((m.group(2), m.group(3)))
        missing = [leg for leg in legs if leg not in legs_ok or leg not in close_ok or
                   contacts.get(leg, set()) != set(CONTACT_ORDER)]
        if missing:
            raise SessionFailure(f"export has no complete 6/6 HARDWARE_CONTACT_CALIBRATED record "
                                 f"for {missing}")
        self.log.say(f"PASS  evidence export: {end.group(0)}")

    def emergency_stop(self):
        """Always-allowed de-escalation; best effort, never raises."""
        for cmd in ("@CALIBRATION FULL LEG ABORT", "@CALIBRATION SESSION ABORT"):
            try:
                self.link.send(cmd)
            except Exception as e:  # link gone: nothing more can be done electronically
                self.log.say(f"!!    could not send {cmd}: {e}")
                return
        self.sleep(1.0)
        for bus in INSTALLED:
            try:
                m = self.request(f"@SERVO SAFE_OFF {bus}", rf"SERVO_SAFE_OFF id={bus} result=(\S+)",
                                 timeout=3.0, fail_patterns=())
                self.log.say(f"      SAFE_OFF {bus}: {m.group(1)}")
            except Exception as e:
                self.log.say(f"!!    SAFE_OFF {bus} not confirmed: {e}")


# --------------------------------------------------------------------------
# Build / flash
# --------------------------------------------------------------------------

def read_manifest(path):
    out = {}
    with open(path) as f:
        for line in f:
            if "=" in line:
                k, v = line.rstrip("\n").split("=", 1)
                out[k] = v
    return out


def verify_build(sketch_dir, log):
    repo = os.path.abspath(os.path.join(sketch_dir, "..", ".."))
    status = subprocess.run(["git", "-C", repo, "status", "--porcelain"], capture_output=True,
                            text=True, check=True).stdout
    if status.strip():
        raise SessionFailure("working tree is not clean")
    head = subprocess.run(["git", "-C", repo, "rev-parse", "HEAD"], capture_output=True,
                          text=True, check=True).stdout.strip()
    build_dir = os.path.join(sketch_dir, "build", "esp32.esp32.esp32s3")
    manifest = read_manifest(os.path.join(build_dir, "matdog_build_manifest.txt"))
    binary = os.path.join(build_dir, "MATDOG_Controller.ino.bin")
    digest = hashlib.sha256(open(binary, "rb").read()).hexdigest()
    checks = {"SOURCE_COMMIT": head, "SOURCE_STATE": "CLEAN", "HARDWARE_PROFILE": "ROBOT_POWERED",
              "OTA_INGEST_ENABLED": "0", "APPLICATION_SHA256": digest}
    for key, want in checks.items():
        if manifest.get(key) != want:
            raise SessionFailure(f"manifest {key}={manifest.get(key)!r}, expected {want!r}")
    log.say(f"PASS  build manifest: {head[:12]} CLEAN ROBOT_POWERED ingest=0 sha256={digest[:16]}...")
    return head[:12], manifest


def flash(sketch_dir, backup, backup_sha256, log):
    # flash_app_only.sh accepts a non-default (fresh) backup only with an
    # explicitly authorized SHA256 - passed through, never computed here.
    env = dict(os.environ, MATDOG_FLASH_PROFILE="ROBOT_POWERED", MATDOG_FLASH_OTA_INGEST="0",
               MATDOG_FLASH_BACKUP=backup, MATDOG_FLASH_BACKUP_SHA256=backup_sha256)
    proc = subprocess.run([os.path.join(sketch_dir, "scripts", "flash_app_only.sh")], env=env,
                          capture_output=True, text=True)
    for line in (proc.stdout + proc.stderr).splitlines():
        log.write("FLASH", line)
    lines = set((proc.stdout + proc.stderr).splitlines())
    for required in ("APPLICATION_ONLY_FLASH = PASS", "FLASHED_HARDWARE_PROFILE = ROBOT_POWERED",
                     "FLASHED_OTA_INGEST_ENABLED = 0"):
        if proc.returncode != 0 or required not in lines:
            raise SessionFailure(f"application-only flash did not report {required!r} "
                                 f"(exit {proc.returncode})")
    log.say("PASS  application-only flash")


def wait_for_port(port, log, timeout=30.0, flashed=False):
    # flash_app_only.sh ends with esptool's RTS hard reset, so the USB node
    # drops and re-enumerates just AFTER it returns. Opening the old node in
    # that window gives a dead descriptor: wait for it to go (bounded), come
    # back, and settle before the no-reset open.
    if flashed:
        gone_by = time.monotonic() + 5.0
        while os.path.exists(port) and time.monotonic() < gone_by:
            time.sleep(0.05)
    deadline = time.monotonic() + timeout
    while not os.path.exists(port):
        if time.monotonic() > deadline:
            raise SessionFailure(f"{port} did not re-enumerate within {timeout:.0f}s")
        time.sleep(0.2)
    if flashed:
        time.sleep(1.5)
    log.say("PASS  USB port present" + (" (re-enumerated after flash)" if flashed else ""))


# --------------------------------------------------------------------------

PHASES = ("prepare", "q0", "recover", "legs", "all")


def run(args, link_factory=SerialLink):
    """Phases, each opening the board WITHOUT a reset (state survives between
    invocations), each stopping at its first failed check:

      prepare  build manifest (CLEAN, ROBOT_POWERED, ingest 0, HEAD), optional
               application-only flash, signature, MAINTENANCE, SAFE_OFF 13/13,
               health READY, authority NONE. No motion.
      q0       --confirm-q0-pose: the operator placed all four legs at q=0.
               Fresh Q0 CAPTURE (12/12, |delta vs CR2-C| < 82), Q0 PROMOTE.
      recover  <first leg> session + permit, then @CALIBRATION INITIAL RECOVERY:
               all 12 leg joints actively to their promoted q0, verified.
               Leaves the session and permit live: the ready state for GO.
      legs     --confirm-operator-go: for each leg, a TRUE 6-contact Full
               Calibration (the first leg reuses the ready session), each later
               leg preceded by its own session, permit and verified INITIAL
               RECOVERY; then SAFE_OFF 13/13 and the evidence export
               (all_contact_calibrated=1 only at 24/24).
      all      prepare + q0 + recover + legs (both confirmations required).

    Any failure: FULL LEG ABORT + SESSION ABORT, SAFE_OFF all 13, a read-only
    evidence export, then stop - no further leg, no retry."""
    sketch_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    legs = [leg.strip().upper() for leg in args.legs.split(",") if leg.strip()]
    for leg in legs:
        if leg not in LEG_MATRIX:
            raise SystemExit(f"unknown leg {leg}")
    if not legs or len(set(legs)) != len(legs):
        raise SystemExit("--legs must name each leg at most once")
    phase = args.phase
    if phase not in PHASES:
        raise SystemExit(f"--phase must be one of {PHASES}")
    flashing = phase in ("prepare", "all") and not args.no_flash
    if flashing and not re.fullmatch(r"[0-9a-f]{64}", args.backup_sha256 or ""):
        raise SystemExit("--backup-sha256 <64 hex> is required to flash: the operator-authorized "
                         "SHA256 of the --backup full-flash image (flash_app_only.sh checks it)")
    if phase in ("q0", "all") and not args.confirm_q0_pose:
        raise SystemExit("--confirm-q0-pose is required: Q0 CAPTURE asserts all four legs are "
                         "physically at the manual q=0 calibration pose")
    if phase in ("legs", "all") and not args.confirm_operator_go:
        raise SystemExit("--confirm-operator-go is required: the operator is physically present, "
                         "the robot is supported, the charger disconnected, the envelope clear "
                         "and the physical disconnect reachable")
    os.makedirs(args.evidence_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = Log(os.path.join(args.evidence_dir, f"hw_session_{phase}_{stamp}.log"), echo=not args.quiet)
    q0_path = os.path.join(args.evidence_dir, "q0_promoted.json")

    # Before the link exists nothing is energized: a failure here only stops.
    try:
        build_id, _ = verify_build(sketch_dir, log) if not args.skip_build_check \
            else (args.build_id, {})
        if flashing:
            flash(sketch_dir, args.backup, args.backup_sha256, log)
        wait_for_port(args.port, log, flashed=flashing)
        link = link_factory(args.port, log)
        link.open()
    except (SessionFailure, OSError, subprocess.CalledProcessError) as e:
        log.say(f"FAIL  before any servo command: {e}")
        log.close()
        return 1
    session = Session(link, log)
    try:
        time.sleep(3.0) if link_factory is SerialLink else None  # passive: boot banner
        link.send("")  # a lone newline flushes a half-received first line
        session.request("@IMU STREAM OFF", r"IMU_STREAM=OFF", timeout=3.0, fail_patterns=()) \
            if args.imu_stream_off else None
        session.verify_signature(build_id)
        session.verify_maintenance()
        q0 = None
        if phase in ("prepare", "q0", "all"):
            session.safe_off_all()
            session.wait_health_ready()
            session.authority_none()
        if phase in ("q0", "all"):
            session.read_positions("raw positions before Q0 CAPTURE")
            q0 = session.q0_capture()
            session.q0_promote()
            with open(q0_path, "w") as f:
                json.dump({str(b): t for b, t in sorted(q0.items())}, f, indent=1)
        if q0 is None and os.path.exists(q0_path):
            with open(q0_path) as f:
                q0 = {int(b): t for b, t in json.load(f).items()}
        if phase in ("recover", "all"):
            session.open_leg_session(legs[0])
            session.read_positions("raw positions before INITIAL RECOVERY")
            session.initial_recovery(legs[0], q0)
            session.read_positions("raw positions after INITIAL RECOVERY")
            session.log.say(f"READY {legs[0]} session + permit live; every leg joint verified at q0. "
                            f"Next: --phase legs --confirm-operator-go (operator GO).")
        if phase in ("legs", "all"):
            for i, leg in enumerate(legs):
                if i == 0 and session.leg_session_ready(leg):
                    session.log.say(f"      {leg}: using the ready session + permit")
                else:
                    session.open_leg_session(leg)
                    session.initial_recovery(leg, q0)
                session.run_full_leg(leg, lf_crosscheck=not args.no_lf_min_crosscheck)
                session.read_positions(f"raw positions after {leg}")
            session.safe_off_all()
            session.authority_none()
            end, lines = session.export()
            with open(os.path.join(args.evidence_dir, f"evidence_export_{stamp}.txt"), "w") as f:
                f.write("\n".join(lines) + "\n")
            session.verify_export(legs, end, lines)
            total = 6 * len(legs)
            log.say(f"DONE  {len(legs)} leg(s) x 6 = {total} contacts HARDWARE_CONTACT_CALIBRATED"
                    + (" - TRUE FULL CALIBRATION 24/24" if total == 24 else "")
                    + f" (RAM only - export saved in {args.evidence_dir})")
        return 0
    except SessionFailure as e:
        log.say(f"FAIL  {e}")
        if not link.lost:
            log.say("      de-escalating: FULL LEG ABORT, SESSION ABORT, SAFE_OFF all 13")
            session.emergency_stop()
            try:
                end, lines = session.export()
                with open(os.path.join(args.evidence_dir, f"evidence_export_{stamp}_after_failure.txt"),
                          "w") as f:
                    f.write("\n".join(lines) + "\n")
                log.say(f"      evidence export after failure: {end.group(0)}")
            except Exception as ex:
                log.say(f"      evidence export unavailable: {ex}")
        return 1
    finally:
        link.close()
        log.close()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--evidence-dir", required=True)
    p.add_argument("--backup", default=os.path.expanduser(
        "~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-29_185351_nostub.bin"))
    p.add_argument("--backup-sha256", default="",
                   help="authorized SHA256 of --backup (required unless --no-flash)")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--legs", default="LF,RF,RH,LH")
    p.add_argument("--phase", default="all", choices=PHASES)
    p.add_argument("--confirm-q0-pose", action="store_true")
    p.add_argument("--confirm-operator-go", action="store_true")
    p.add_argument("--no-flash", action="store_true", help="the board already runs this build")
    p.add_argument("--no-lf-min-crosscheck", action="store_true")
    p.add_argument("--imu-stream-off", action="store_true", default=True)
    p.add_argument("--skip-build-check", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--build-id", default="", help=argparse.SUPPRESS)
    p.add_argument("--quiet", action="store_true")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
