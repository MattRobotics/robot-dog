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
LEG_MATRIX = {  # primary bus, auxiliary, aux bus - Geometry V5 parking matrix
    "LF": (12, "LH_UPPER", 42),
    "RF": (22, "RH_UPPER", 32),
    "RH": (32, "NONE", 0),
    "LH": (42, "NONE", 0),
}
# LF_UPPER's true MIN stop was found by hand ~23 ticks past the canonical
# contact (2026-09-29). A LF MIN contact far from that is stopped for review
# before any other leg runs.
LF_MIN_EXPECTED_BEYOND = (7, 39)
LEG_WATCHDOG_S = 420.0
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

    def run_leg(self, leg, lf_crosscheck=True):
        bus, aux, aux_bus = LEG_MATRIX[leg]
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

        mark = self.mark()
        self.link.send(f"@CALIBRATION FULL LEG {leg} CONFIRM_FULL_CALIBRATION")
        armed = self.wait_for(r"CALIBRATION_FULL_LEG=ARMED leg=(\S+) joint=UPPER bus=(\d+) "
                              r"phase=UPPER_MIN_PROBE auxiliary=(\S+) aux_bus=(\d+)", mark, 5.0,
                              (r"CALIBRATION_FULL_LEG=REFUSED",))
        if (armed.group(1), int(armed.group(2)), armed.group(3), int(armed.group(4))) != \
                (leg, bus, aux, aux_bus):
            self.link.send("@CALIBRATION FULL LEG ABORT")
            raise SessionFailure(f"{leg} ARMED {armed.group(0)} does not match the parking matrix")
        corridors = {}
        crx = re.compile(r"CALIBRATION_FULL_LEG_SEARCH_CORRIDOR side=(MIN|MAX) probe_sign=(-?1) "
                         r"q0=(\d+) contact=(\d+) urdf_limit=(\d+) entry=(\d+) guard=(\d+) "
                         r"opposite_limit=(\d+) guard_beyond_contact=(-?\d+)")
        deadline = self.clock() + 3.0
        while len(corridors) < 2 and self.clock() < deadline:
            for text in self.lines_since(mark):
                m = crx.fullmatch(text)
                if m:
                    corridors[m.group(1)] = m
            self.sleep(0.05)
        if len(corridors) != 2:
            self.link.send("@CALIBRATION FULL LEG ABORT")
            raise SessionFailure(f"{leg}: search corridors not reported on ARMED")
        for side in ("MIN", "MAX"):
            c = corridors[side]
            self.log.say(f"      {leg} {side}: q0={c.group(3)} contact={c.group(4)} "
                         f"urdf_limit={c.group(5)} entry={c.group(6)} guard={c.group(7)} "
                         f"(guard {c.group(9)} ticks past contact)")
        self.log.say(f"RUN   {leg} full leg ARMED (bus {bus}, auxiliary {aux})")

        verdict, failure = self._monitor(leg, mark)
        self.log.say(f"RESULT {leg} verdict={verdict} failure={failure}")
        if verdict != "HARDWARE_CONTACT_CALIBRATED" or failure != "NONE":
            raise SessionFailure(f"{leg} verdict={verdict} failure={failure}")

        # Cleanup, exactly as the runbook requires before the next leg.
        self.safe_off([bus] + ([aux_bus] if aux_bus else []))
        self.authority_none()
        mark = self.mark()
        self.link.send("@CALIBRATION STATUS")
        self.wait_for(rf"CALIBRATION_SESSION state=COMPLETED origin=LIVE_SESSION leg={leg} .*", mark, 5.0)
        mark = self.mark()
        self.link.send("@CALIBRATION FULL LEG STATUS")
        self.wait_for(rf"CALIBRATION_FULL_LEG_RECORD leg={leg} present=YES attempts=\d+ "
                      r"verdict=HARDWARE_CONTACT_CALIBRATED contact_calibrated=YES "
                      r"envelope_accepted=NO failure=NONE", mark, 5.0)
        mins = self.wait_for(r"CALIBRATION_FULL_LEG_UPPER_MIN measured=YES fine_tick=(\d+) "
                             r"witness_accepted=YES", mark, 5.0)
        maxs = self.wait_for(r"CALIBRATION_FULL_LEG_UPPER_MAX measured=YES fine_tick=(\d+) "
                             r"witness_accepted=YES", mark, 5.0)
        for side, m in (("MIN", mins), ("MAX", maxs)):
            c = corridors[side]
            sign, contact = int(c.group(2)), int(c.group(4))
            beyond = (int(m.group(1)) - contact) * sign
            self.log.say(f"      {leg} {side} contact tick {m.group(1)} = canonical {beyond:+d}")
            if leg == "LF" and side == "MIN" and lf_crosscheck and \
                    not (LF_MIN_EXPECTED_BEYOND[0] <= beyond <= LF_MIN_EXPECTED_BEYOND[1]):
                raise SessionFailure(
                    f"LF MIN contact is canonical {beyond:+d}, but the stop was found by hand at "
                    f"about +23: stopping for operator review before any other leg "
                    f"(--no-lf-min-crosscheck to accept)")
        self.log.say(f"PASS  {leg} HARDWARE_CONTACT_CALIBRATED; cleanup verified")

    def _monitor(self, leg, mark):
        """Polls FULL LEG STATUS; returns only on the exact terminal record."""
        terminal = re.compile(rf"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict=(\S+) failure=(\S+)")
        event = re.compile(r"CALIBRATION_SEARCH exec=(\S+) side=(MIN|MAX) pass=(\d) stage=(\S+) "
                           r"probe=(\S+) target=(\d+) pos=(-?\d+) beyond_contact=(-?\d+) .*")
        final = re.compile(r"CALIBRATION_FULL_LEG_PROBE_FINAL .*")
        start = self.clock()
        last_poll = -POLL_S
        last_rx = self.clock()
        seen = mark
        aborted_at = None
        session_aborted = False
        last_state = None
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
                    return m.group(1), m.group(2)
                e = event.fullmatch(text)
                if e:
                    state = (e.group(1), e.group(2), e.group(3), e.group(4), e.group(5))
                    if state != last_state:
                        last_state = state
                        self.log.say(f"      {self.clock() - start:6.1f}s {e.group(1)} {e.group(2)} "
                                     f"pass {e.group(3)} {e.group(4)} {e.group(5)} target={e.group(6)} "
                                     f"pos={e.group(7)} beyond_contact={e.group(8)}")
                if final.fullmatch(text):
                    self.log.say("      " + text)
            if lost:
                raise SessionFailure(f"{leg}: USB link lost during an energized run - "
                                     f"use the physical disconnect; nothing can be sent")
            t = self.clock()
            if t - last_poll >= POLL_S:
                self.link.send("@CALIBRATION FULL LEG STATUS")
                last_poll = t
            if aborted_at is None and (t - start > LEG_WATCHDOG_S or t - last_rx > SILENCE_S):
                self.log.say(f"!!    {leg}: watchdog (elapsed {t - start:.0f}s, silence "
                             f"{t - last_rx:.0f}s) -> FULL LEG ABORT")
                self.link.send("@CALIBRATION FULL LEG ABORT")
                aborted_at = t
            if aborted_at is not None and not session_aborted and t - aborted_at > 30.0:
                self.link.send("@CALIBRATION SESSION ABORT")
                session_aborted = True
            if aborted_at is not None and t - aborted_at > 60.0:
                raise SessionFailure(f"{leg}: no terminal record 60 s after ABORT")
            with self.link.cond:
                if len(self.link.lines) == seen and not self.link.lost:
                    self.link.cond.wait(timeout=0.05)

    def export(self, expected_legs):
        mark = self.mark()
        self.link.send("@CALIBRATION EVIDENCE EXPORT")
        end = self.wait_for(r"CALIBRATION_EVIDENCE_EXPORT=END legs_present=(\d+) "
                            r"legs_contact_calibrated=(\d+) legs_envelope_accepted=(\d+) "
                            r"all_contact_calibrated=(\d)", mark, 20.0,
                            (r"CALIBRATION_EVIDENCE_EXPORT=REFUSED",))
        lines = self.lines_since(mark)
        return end, lines

    def verify_export(self, legs, end, lines):
        n = len(legs)
        want = (str(n), str(n), "0", "1" if n == 4 else end.group(4))
        if (end.group(1), end.group(2), end.group(3), end.group(4)) != want:
            raise SessionFailure(f"export summary {end.group(0)} != legs_present={n} "
                                 f"legs_contact_calibrated={n} legs_envelope_accepted=0"
                                 + (" all_contact_calibrated=1" if n == 4 else ""))
        rx = re.compile(r"CALIBRATION_EVIDENCE_LEG leg=(\S+) present=1 attempts=\d+ session=\d+ "
                        r"geometry=\S+ verdict=HARDWARE_CONTACT_CALIBRATED contact_calibrated=1 "
                        r"envelope_accepted=0 failure=NONE executor_failure=NONE "
                        r"session_completed=1 permit_revoked=1 authority_released=1 "
                        r"parameters_approved=0")
        ok = {m.group(1) for m in (rx.fullmatch(t) for t in lines) if m}
        missing = [leg for leg in legs if leg not in ok]
        if missing:
            raise SessionFailure(f"export has no complete HARDWARE_CONTACT_CALIBRATED record for {missing}")
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

def run(args, link_factory=SerialLink):
    sketch_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    legs = [leg.strip().upper() for leg in args.legs.split(",") if leg.strip()]
    for leg in legs:
        if leg not in LEG_MATRIX:
            raise SystemExit(f"unknown leg {leg}")
    if not args.no_flash and not re.fullmatch(r"[0-9a-f]{64}", args.backup_sha256 or ""):
        raise SystemExit("--backup-sha256 <64 hex> is required to flash: the operator-authorized "
                         "SHA256 of the --backup full-flash image (flash_app_only.sh checks it)")
    if not args.confirm_q0_pose:
        raise SystemExit("--confirm-q0-pose is required: Q0 CAPTURE asserts all four legs are "
                         "physically at the manual q=0 calibration pose")
    os.makedirs(args.evidence_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = Log(os.path.join(args.evidence_dir, f"hw_session_{stamp}.log"), echo=not args.quiet)

    # Before the link exists nothing is energized: a failure here only stops.
    try:
        build_id, manifest = verify_build(sketch_dir, log) if not args.skip_build_check \
            else (args.build_id, {})
        if not args.no_flash:
            flash(sketch_dir, args.backup, args.backup_sha256, log)
        wait_for_port(args.port, log, flashed=not args.no_flash)
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
        session.safe_off_all()
        session.wait_health_ready()
        session.authority_none()
        session.q0_capture()
        session.q0_promote()
        for leg in legs:
            session.run_leg(leg, lf_crosscheck=not args.no_lf_min_crosscheck)
        session.safe_off_all()
        session.authority_none()
        end, lines = session.export(legs)
        session.verify_export(legs, end, lines)
        log.say(f"DONE  {len(legs)}/{len(legs)} legs HARDWARE_CONTACT_CALIBRATED "
                f"(RAM only - export saved in {args.evidence_dir})")
        return 0
    except SessionFailure as e:
        log.say(f"FAIL  {e}")
        if not link.lost:
            log.say("      de-escalating: FULL LEG ABORT, SESSION ABORT, SAFE_OFF all 13")
            session.emergency_stop()
            try:
                end, lines = session.export(legs)
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
    p.add_argument("--confirm-q0-pose", action="store_true")
    p.add_argument("--no-flash", action="store_true", help="the board already runs this build")
    p.add_argument("--no-lf-min-crosscheck", action="store_true")
    p.add_argument("--imu-stream-off", action="store_true", default=True)
    p.add_argument("--skip-build-check", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--build-id", default="", help=argparse.SUPPRESS)
    p.add_argument("--quiet", action="store_true")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
