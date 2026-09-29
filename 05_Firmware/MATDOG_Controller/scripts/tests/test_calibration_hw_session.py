#!/usr/bin/env python3
"""Offline tests for scripts/calibration_hw_session.py against a fake
Controller that speaks the firmware's real record formats (CommandRouter /
Controller / FullLegCalibrationFinalizer). No hardware, no serial port.

Regression coverage from the 2026-09-29 session:
  - the informational NOTE containing "HARDWARE_CONTACT_CALIBRATED" (printed
    by every FULL LEG STATUS) is never a terminal result; only the exact
    CALIBRATION_FULL_LEG_RESULT record is, for the right leg, whole-line;
  - nothing but FULL LEG STATUS is sent while a run is in flight (no mid-run
    SAFE_OFF - the cause of the 2026-09-29 EXECUTOR_FAILED);
  - a failed leg stops the session: no further leg, ABORTs, SAFE_OFF 13;
  - q0 |delta| >= 82 vs CR2-C stops before PROMOTE;
  - SESSION START is refused without a current-boot promoted q0;
  - a lost link sends nothing more; a silent run is ABORTed by the watchdog.
"""
import os
import re
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import calibration_hw_session as hw  # noqa: E402

BUILD_ID = "0123456789ab"
Q0 = {11: 2088, 12: 2086, 13: 1981, 21: 1995, 22: 2108, 23: 2030,
      31: 2034, 32: 2042, 33: 2081, 41: 2073, 42: 2072, 43: 2025}
UNITS = {11: ("LF", "LOWER", "M33"), 12: ("LF", "UPPER", "ELR01"), 13: ("LF", "HIP", "M22"),
         21: ("RF", "LOWER", "NEW03"), 22: ("RF", "UPPER", "ELR03"), 23: ("RF", "HIP", "NEW01"),
         31: ("RH", "LOWER", "NEW05"), 32: ("RH", "UPPER", "ELR02"), 33: ("RH", "HIP", "NEW06"),
         41: ("LH", "LOWER", "M41"), 42: ("LH", "UPPER", "M42"), 43: ("LH", "HIP", "M43")}
# (probe_sign, q0, contact, urdf_limit) per leg/side for the fake corridors
CORRIDOR = {"LF": {"MIN": (-1, 2086, 1493, 1489), "MAX": (1, 2086, 3473, 3480)},
            "RF": {"MIN": (1, 2108, 2701, 2705), "MAX": (-1, 2108, 721, 714)},
            "RH": {"MIN": (1, 2042, 2635, 2639), "MAX": (-1, 2042, 655, 648)},
            "LH": {"MIN": (-1, 2072, 1479, 1475), "MAX": (1, 2072, 3459, 3466)}}


class FakeController:
    def __init__(self, link):
        self.link = link
        self.promoted = False
        self.authority = "NONE"
        self.session = None          # (leg, state)
        self.permit = False
        self.records = {}            # leg -> verdict
        self.run = None              # dict for the armed run
        self.polls_before_result = 4
        self.fail_leg = None
        self.lf_min_beyond = 23
        self.q0 = dict(Q0)
        self.build_id = BUILD_ID
        self.lose_link_during = None
        self.never_finish = False
        self.decorated_result_first = False
        self.wrong_leg_result_first = False
        self.sent = []

    def emit(self, *lines):
        self.link.inject(lines)

    def later(self, delay, *lines):
        threading.Timer(delay, lambda: self.emit(*lines)).start()

    def contact_tick(self, leg, side):
        sign, q0, contact, limit = CORRIDOR[leg][side]
        beyond = self.lf_min_beyond if (leg == "LF" and side == "MIN") else 20
        return contact + sign * beyond

    def handle(self, cmd):
        self.sent.append(cmd)
        u = cmd.upper()
        if u == "":
            return
        if u == "@IMU STREAM OFF":
            return self.emit("IMU_STREAM=OFF")
        if u == "@SYSTEM SOURCE_SIGNATURE":
            return self.emit(f"SOURCE_SIGNATURE build_id={self.build_id} firmware=MATDOG Controller "
                             f"version=0.1.0 profile=ROBOT_POWERED board=YD-ESP32-S3 N16R8",
                             f"  ota_running_build_id={self.build_id} ota_running_image_state=UNDEFINED "
                             f"reset_reason=OTHER", "  partition=app0 address=0x010000 size=0x300000")
        if u == "@MODE STATUS":
            return self.emit("MODE=MAINTENANCE")
        m = re.fullmatch(r"@SERVO SAFE_OFF (\d+)", u)
        if m:
            return self.emit(f"SERVO_SAFE_OFF id={m.group(1)} result=VERIFIED_OFF")
        if u == "@STATUS":
            return self.emit(f"SYSTEM health=READY power_state=RUN mode=MAINTENANCE "
                             f"authority={self.authority} uptime_ms=1234 profile=ROBOT_POWERED",
                             "SERVO_POP canonical=17 expected_now=13 absent_by_design=4 last_census=NOT_RUN")
        if u == "@AUTHORITY STATUS":
            return self.emit(f"AUTHORITY owner={self.authority} generation=1 last_result=RELEASED",
                             "AUTHORITY_INHIBIT active=NO reason=NONE")
        if u == "@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE":
            self.emit("CALIBRATION_Q0=STARTED session=1 samples_per_joint=9 stability_ticks=16",
                      "CALIBRATION_Q0_NOTE read-only; torque must already be OFF; no motion/authority/EEPROM write")
            lines = ["CALIBRATION_Q0 state=COMPLETE failure=NONE session=1 sample_passes=9/9 next_joint=0 "
                     "candidates=12/12", "CALIBRATION_Q0_POPULATION status=PASS verdict=PASS observed=12/12"]
            for bus in sorted(self.q0):
                leg, joint, unit = UNITS[bus]
                lines.append(f"  Q0 bus={bus} leg={leg} joint={joint} unit={unit} tick={self.q0[bus]} "
                             f"spread=0 samples=9 state=CANDIDATE estimator=MANUAL_ZERO_POSE")
            lines += ["CALIBRATION_Q0_RESULT=12_CANDIDATES_ONLY",
                      "CALIBRATION_Q0_NOTE accepted=NO promoted=NO transform_admitted=NO motion_authorized=NO"]
            return self.later(0.05, *lines)
        if u == "@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION":
            self.promoted = True
            return self.emit("CALIBRATION_Q0_PROMOTE=OK admitted=12/12 source=CURRENT_BOOT_CAPTURE capture_session=1",
                             "CALIBRATION_Q0_PROMOTE_NOTE RAM-only; no EEPROM write")
        if u == "@ACTUATOR STATUS":
            return self.emit("ACTUATOR_POLICY epoch=1 outstanding_transaction=NO last_decision=ACCEPT",
                             f"ACTUATOR_PROVENANCE limits_admitted=0 transforms_admitted="
                             f"{12 if self.promoted else 0} geometry_bound=YES")
        m = re.fullmatch(r"@CALIBRATION SESSION START (LF|RF|RH|LH) CONFIRM_CURRENT_Q0", u)
        if m:
            if not self.promoted:
                return self.emit("CALIBRATION_SESSION=REFUSED",
                                 "REASON=CURRENT_BOOT_Q0_NOT_PROMOTED hint=@CALIBRATION_Q0_PROMOTE")
            self.session = (m.group(1), "ACTIVE")
            self.authority = "CALIBRATION"
            return self.emit(f"CALIBRATION_SESSION=ACTIVE leg={m.group(1)} session=1 authority_generation=1",
                             "CALIBRATION_SESSION_NOTE motion_permit=NOT_GRANTED hardware_motion_authorized=FALSE")
        if u == "@CALIBRATION STATUS":
            leg, state = self.session if self.session else ("LF", "NO_SESSION")
            return self.emit("CALIBRATION_CURRENT state=STALE_PENDING_FULL_RECALIBRATION hardware_motion=BLOCKED",
                             f"CALIBRATION_SESSION state={state} origin=LIVE_SESSION leg={leg} last_result=OK",
                             "CALIBRATION_POPULATION verdict=PASS observed=12/12 mask=0xfff")
        if u == "@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION":
            self.permit = True
            return self.emit("CALIBRATION_MOTION_PERMIT=ACTIVE generation=2 session=1 authority_generation=1 "
                             "direction_verify_budget_ticks=16", "CALIBRATION_MOTION_PERMIT_NOTE RAM_ONLY")
        m = re.fullmatch(r"@CALIBRATION FULL LEG (LF|RF|RH|LH) CONFIRM_FULL_CALIBRATION", u)
        if m:
            leg = m.group(1)
            bus, aux, aux_bus = hw.LEG_MATRIX[leg]
            self.run = {"leg": leg, "polls": 0}
            lines = [f"CALIBRATION_FULL_LEG=ARMED leg={leg} joint=UPPER bus={bus} phase=UPPER_MIN_PROBE "
                     f"auxiliary={aux} aux_bus={aux_bus}"]
            for side in ("MIN", "MAX"):
                sign, q0, contact, limit = CORRIDOR[leg][side]
                lines.append(f"CALIBRATION_FULL_LEG_SEARCH_CORRIDOR side={side} probe_sign={sign} q0={q0} "
                             f"contact={contact} urdf_limit={limit} entry={limit - sign * 64} "
                             f"guard={limit + sign * 64} opposite_limit=0 guard_beyond_contact=68")
            lines.append("CALIBRATION_FULL_LEG_NOTE no_write_in_command_handler; poll with @CALIBRATION FULL LEG STATUS")
            return self.emit(*lines)
        if u == "@CALIBRATION FULL LEG STATUS":
            return self.status_poll()
        if u == "@CALIBRATION FULL LEG ABORT":
            if self.run:
                leg = self.run["leg"]
                self.run = None
                self.records[leg] = "FAILED"
                self.session = (leg, "FAILED")
                self.authority = "NONE"
                self.later(0.05, "CALIBRATION_FULL_LEG_ABORT=OK",
                           f"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict=FAILED failure=EXECUTOR_FAILED")
            return self.emit("CALIBRATION_FULL_LEG_ABORT=NO_ACTIVE_SEQUENCE")
        if u == "@CALIBRATION SESSION ABORT":
            self.authority = "NONE"
            return self.emit("CALIBRATION_SESSION_ABORT=OK")
        if u == "@CALIBRATION EVIDENCE EXPORT":
            return self.export()
        self.emit(f"UNKNOWN_COMMAND={cmd}")

    def status_poll(self):
        lines = []
        run = self.run
        if run:
            run["polls"] += 1
            leg = run["leg"]
            if self.lose_link_during == leg and run["polls"] == 2:
                self.link.drop()
                return
            lines.append(f"CALIBRATION_SEARCH exec=UPPER_MIN_PROBE side=MIN pass=1 stage=FINE_SEARCH "
                         f"probe=STEP_MONITORING target=1480 pos=1485 beyond_contact=13 contact=1493 "
                         f"entry=1553 guard=1425 speed=0 current=40 baseline=30/35 steps=12 bypass=0 "
                         f"p1=0 p2=0 failure=NONE")
            lines.append("CALIBRATION_FULL_LEG phase=UPPER_MIN_PROBE failure=NONE last_decision=ACCEPT")
        for leg in ("LF", "RF", "RH", "LH"):
            if leg in self.records:
                v = self.records[leg]
                ok = v == "HARDWARE_CONTACT_CALIBRATED"
                lines.append(f"CALIBRATION_FULL_LEG_RECORD leg={leg} present=YES attempts=1 verdict={v} "
                             f"contact_calibrated={'YES' if ok else 'NO'} envelope_accepted=NO "
                             f"failure={'NONE' if ok else 'EXECUTOR_FAILED'}")
            else:
                lines.append(f"CALIBRATION_FULL_LEG_RECORD leg={leg} present=NO verdict=NOT_RUN")
        last = getattr(self, "last_leg", None)
        if last and not run:
            lines.append(f"CALIBRATION_FULL_LEG_UPPER_MIN measured=YES fine_tick={self.contact_tick(last, 'MIN')} "
                         f"witness_accepted=YES")
            lines.append(f"CALIBRATION_FULL_LEG_UPPER_MAX measured=YES fine_tick={self.contact_tick(last, 'MAX')} "
                         f"witness_accepted=YES")
        # Printed by EVERY status poll in the real firmware - never a result.
        lines.append("CALIBRATION_FULL_LEG_NOTE HARDWARE_CONTACT_CALIBRATED = both UPPER contacts recorded + "
                     "SAFE_OFF + session completed; FINAL_OPERATIONAL_ENVELOPE_ACCEPTED additionally needs "
                     "APPROVED envelope parameters (none exist in this build)")
        self.emit(*lines)
        if run and not self.never_finish and run["polls"] >= self.polls_before_result:
            leg = run["leg"]
            self.run = None
            verdict = "FAILED" if leg == self.fail_leg else "HARDWARE_CONTACT_CALIBRATED"
            failure = "EXECUTOR_FAILED" if leg == self.fail_leg else "NONE"
            self.records[leg] = verdict
            self.last_leg = leg
            self.session = (leg, "COMPLETED" if failure == "NONE" else "FAILED")
            self.authority = "NONE"
            final = []
            if self.wrong_leg_result_first:
                other = "RH" if leg != "RH" else "LH"
                final.append(f"CALIBRATION_FULL_LEG_RESULT leg={other} verdict=FAILED failure=EXECUTOR_FAILED")
            if self.decorated_result_first:
                # Not whole-line records: an unanchored matcher would take
                # these FAILED verdicts and end the session.
                final.append(f"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict=FAILED failure=X trailing_text")
                final.append(f"xCALIBRATION_FULL_LEG_RESULT leg={leg} verdict=FAILED failure=X")
            final.append(f"CALIBRATION_FULL_LEG_PROBE_FINAL leg={leg} side=MAX executor_failure=NONE "
                         f"probe_phase=COMPLETE probe_failure=NONE pass=2 stage=FINE_SEARCH target=1 pos=1 "
                         f"contact=1 guard=1 p1=1 p2=1 bypass=0 steps=40")
            final.append(f"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict={verdict} failure={failure}")
            self.later(0.05, *final)

    def export(self):
        lines = ["CALIBRATION_EVIDENCE_EXPORT=BEGIN format=1 geometry=663f4d82f5817fb9 parameters_approved=0 "
                 "upper_margin_ticks=8 hip_lower_margin_urad=50000"]
        n = cal = 0
        for leg in ("LF", "RF", "RH", "LH"):
            v = self.records.get(leg)
            if v is None:
                lines.append(f"CALIBRATION_EVIDENCE_LEG leg={leg} present=0 attempts=0 verdict=NOT_RUN")
                continue
            n += 1
            ok = v == "HARDWARE_CONTACT_CALIBRATED"
            cal += 1 if ok else 0
            lines.append(f"CALIBRATION_EVIDENCE_LEG leg={leg} present=1 attempts=1 session=1 geometry=663f4d82f5817fb9 "
                         f"verdict={v} contact_calibrated={1 if ok else 0} envelope_accepted=0 "
                         f"failure={'NONE' if ok else 'EXECUTOR_FAILED'} "
                         f"executor_failure={'NONE' if ok else 'UPPER_MIN_PROBE_FAILED'} "
                         f"session_completed={1 if ok else 0} permit_revoked=1 authority_released=1 "
                         f"parameters_approved=0")
        lines.append(f"CALIBRATION_EVIDENCE_EXPORT=END legs_present={n} legs_contact_calibrated={cal} "
                     f"legs_envelope_accepted=0 all_contact_calibrated={1 if cal == 4 else 0}")
        self.later(0.02, *lines)


class FakeLink:
    def __init__(self, port, log):
        self.log = log
        self.lines = []
        self.cond = threading.Condition()
        self.lost = False
        self.controller = FakeController(self)
        self.sent_after_loss = []

    def open(self):
        pass

    def inject(self, lines):
        with self.cond:
            if self.lost:
                return
            for text in lines:
                self.log.write("RX", text)
                self.lines.append((time.monotonic(), text))
            self.cond.notify_all()

    def drop(self):
        with self.cond:
            self.lost = True
            self.cond.notify_all()

    def send(self, text):
        if self.lost:
            self.sent_after_loss.append(text)
            raise hw.SessionFailure("link lost")
        self.log.write("TX", text)
        self.controller.handle(text)

    def close(self):
        pass


class RunnerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.link = None
        self.configure = lambda c: None
        hw.LEG_WATCHDOG_S = 420.0
        hw.POLL_S = 0.02

    def run_session(self, *extra):
        holder = {}

        def factory(port, log):
            link = FakeLink(port, log)
            self.configure(link.controller)
            holder["link"] = link
            return link

        port = os.path.join(self.tmp, "fake_port")
        open(port, "w").close()
        argv = ["--evidence-dir", self.tmp, "--port", port, "--confirm-q0-pose", "--no-flash",
                "--skip-build-check", "--build-id", BUILD_ID, "--quiet", *extra]
        rc = hw.run(self._parse(argv), link_factory=factory)
        self.link = holder["link"]
        return rc

    def _parse(self, argv):
        # Reuse main()'s parser exactly.
        captured = {}
        real_run = hw.run
        hw.run = lambda args, link_factory=None: captured.setdefault("args", args)
        try:
            hw.main(argv)
        finally:
            hw.run = real_run
        return captured["args"]

    def sent(self):
        return self.link.controller.sent

    def test_happy_path_all_four_legs(self):
        rc = self.run_session()
        self.assertEqual(rc, 0)
        cmds = self.sent()
        order = [c for c in cmds if c.startswith("@CALIBRATION FULL LEG ") and "CONFIRM" in c]
        self.assertEqual(order, [f"@CALIBRATION FULL LEG {l} CONFIRM_FULL_CALIBRATION"
                                 for l in ("LF", "RF", "RH", "LH")])
        self.assertIn("@CALIBRATION EVIDENCE EXPORT", cmds)
        # q0 capture precedes promotion precedes the first session.
        self.assertLess(cmds.index("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE"),
                        cmds.index("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION"))
        self.assertLess(cmds.index("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION"),
                        cmds.index("@CALIBRATION SESSION START LF CONFIRM_CURRENT_Q0"))
        self.assertEqual(sum(1 for c in cmds if c.startswith("@SERVO SAFE_OFF")),
                         13 + 13 + 2 + 2 + 1 + 1)  # pre, post, LF(12,42), RF(22,32), RH, LH

    def test_note_line_is_never_a_result_and_nothing_else_is_sent_mid_run(self):
        self.configure = lambda c: setattr(c, "polls_before_result", 6)
        rc = self.run_session("--legs", "LF")
        self.assertEqual(rc, 0)
        cmds = self.sent()
        i = cmds.index("@CALIBRATION FULL LEG LF CONFIRM_FULL_CALIBRATION")
        in_flight = []
        for c in cmds[i + 1:]:
            if c != "@CALIBRATION FULL LEG STATUS":
                break
            in_flight.append(c)
        # The runner kept polling through several NOTE-bearing replies...
        self.assertGreaterEqual(len(in_flight), 6)
        # ...and the first non-poll command came only after the RESULT: SAFE_OFF 12.
        self.assertEqual(cmds[i + 1 + len(in_flight)], "@SERVO SAFE_OFF 12")

    def test_decorated_or_other_leg_result_is_not_terminal(self):
        def cfg(c):
            c.decorated_result_first = True
            c.wrong_leg_result_first = True
        self.configure = cfg
        rc = self.run_session("--legs", "LF")
        self.assertEqual(rc, 0)  # it finished on the exact LF record only

    def test_failed_leg_stops_the_session(self):
        self.configure = lambda c: setattr(c, "fail_leg", "RF")
        rc = self.run_session()
        self.assertEqual(rc, 1)
        cmds = self.sent()
        self.assertIn("@CALIBRATION FULL LEG RF CONFIRM_FULL_CALIBRATION", cmds)
        self.assertNotIn("@CALIBRATION SESSION START RH CONFIRM_CURRENT_Q0", cmds)
        self.assertNotIn("@CALIBRATION SESSION START LH CONFIRM_CURRENT_Q0", cmds)
        tail = cmds[cmds.index("@CALIBRATION FULL LEG RF CONFIRM_FULL_CALIBRATION"):]
        self.assertIn("@CALIBRATION FULL LEG ABORT", tail)
        self.assertIn("@CALIBRATION SESSION ABORT", tail)
        for bus in hw.INSTALLED:
            self.assertIn(f"@SERVO SAFE_OFF {bus}", tail)
        self.assertEqual(tail[-1], "@CALIBRATION EVIDENCE EXPORT")

    def test_q0_half_tooth_delta_stops_before_promotion(self):
        def cfg(c):
            c.q0[13] = hw.CR2C[13] + 82
        self.configure = cfg
        rc = self.run_session()
        self.assertEqual(rc, 1)
        self.assertNotIn("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION", self.sent())
        self.assertFalse(any("SESSION START" in c for c in self.sent()))

    def test_wrong_build_stops_before_any_motion(self):
        self.configure = lambda c: setattr(c, "build_id", "ffffffffffff")
        rc = self.run_session()
        self.assertEqual(rc, 1)
        self.assertFalse(any("Q0 CAPTURE" in c or "SESSION START" in c for c in self.sent()))

    def test_session_start_requires_promoted_current_boot_q0(self):
        def cfg(c):
            real = c.handle

            def handle(cmd):
                if cmd == "@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION":
                    c.sent.append(cmd)
                    # a promotion that the firmware refused
                    return c.emit("CALIBRATION_Q0_PROMOTE=REFUSED", "REASON=REJECT_FRESH_CAPTURE_NOT_COMPLETE")
                return real(cmd)
            c.handle = handle
        self.configure = cfg
        rc = self.run_session()
        self.assertEqual(rc, 1)
        self.assertFalse(any("SESSION START" in c for c in self.sent()))

    def test_link_loss_mid_run_sends_nothing_more(self):
        self.configure = lambda c: setattr(c, "lose_link_during", "LF")
        rc = self.run_session()
        self.assertEqual(rc, 1)
        self.assertEqual(self.link.sent_after_loss, [])  # never even attempted
        self.assertNotIn("@CALIBRATION SESSION START RF CONFIRM_CURRENT_Q0", self.sent())

    def test_silent_run_is_aborted_by_the_watchdog(self):
        hw.LEG_WATCHDOG_S = 0.5
        self.configure = lambda c: setattr(c, "never_finish", True)
        rc = self.run_session("--legs", "LF")
        self.assertEqual(rc, 1)
        self.assertIn("@CALIBRATION FULL LEG ABORT", self.sent())

    def test_lf_min_contact_far_from_the_hand_found_stop_stops_before_rf(self):
        self.configure = lambda c: setattr(c, "lf_min_beyond", 60)
        rc = self.run_session()
        self.assertEqual(rc, 1)
        self.assertNotIn("@CALIBRATION SESSION START RF CONFIRM_CURRENT_Q0", self.sent())

    def test_q0_pose_confirmation_is_mandatory(self):
        with self.assertRaises(SystemExit):
            hw.main(["--evidence-dir", self.tmp, "--no-flash", "--skip-build-check"])

    def test_missing_port_fails_cleanly_before_any_servo_command(self):
        opened = []
        real_timeout = hw.wait_for_port.__defaults__
        hw.wait_for_port.__defaults__ = (0.2, False)  # timeout, flashed
        try:
            args = self._parse(["--evidence-dir", self.tmp, "--port", "/nonexistent/port",
                                "--confirm-q0-pose", "--no-flash", "--skip-build-check",
                                "--build-id", BUILD_ID, "--quiet"])
            rc = hw.run(args, link_factory=lambda port, log: opened.append(port))
        finally:
            hw.wait_for_port.__defaults__ = real_timeout
        self.assertEqual(rc, 1)
        self.assertEqual(opened, [])

    def test_flashing_requires_the_authorized_backup_sha256(self):
        # No --no-flash and no --backup-sha256: refused before any build
        # check, flash or port access (the flash script would refuse too).
        flashed = []
        real_flash = hw.flash
        hw.flash = lambda *a, **k: flashed.append(a)
        try:
            for bad in ([], ["--backup-sha256", "7290a327"], ["--backup-sha256", "X" * 64]):
                with self.assertRaises(SystemExit):
                    hw.main(["--evidence-dir", self.tmp, "--confirm-q0-pose",
                             "--skip-build-check", "--port", "/nonexistent", *bad])
        finally:
            hw.flash = real_flash
        self.assertEqual(flashed, [])


if __name__ == "__main__":
    unittest.main(verbosity=1)
