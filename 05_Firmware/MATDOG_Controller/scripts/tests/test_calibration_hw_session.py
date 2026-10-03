#!/usr/bin/env python3
"""Offline tests for scripts/calibration_hw_session.py against a fake
Controller that speaks the firmware's real record formats (CommandRouter /
Controller / FullLegCalibrationFinalizer) for the TRUE 24-contact Full
Calibration. No hardware, no serial port.

Covered:
  - the phased flow: prepare (no motion) / q0 / recover (verified 12/12
    INITIAL RECOVERY, session + permit left ready) / legs (only with the
    operator's GO), state carried across invocations like the real board;
  - a leg is a success only with CONTACTS expected=6 measured=6 accepted=6
    AND the exact RESULT record; 5/6 or a lying RESULT stops everything;
  - the four-leg export must say 24/24 and all_contact_calibrated=1;
  - the informational NOTE containing "HARDWARE_CONTACT_CALIBRATED" is never
    a result; decorated / other-leg RESULT lines are never terminal;
  - nothing but FULL LEG STATUS is sent while a run is in flight;
  - a failed recovery or leg stops the session: ABORTs, SAFE_OFF 13, export;
  - q0 |delta| >= 82 vs CR2-C stops before PROMOTE; recovery targets must be
    the promoted q0; an ARMED plan that disagrees with the matrix aborts;
  - a lost link sends nothing more; a silent run is ABORTed by the watchdog.
"""
import os
import json
import re
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import calibration_hw_session as hw  # noqa: E402

BUILD_ID = "0123456789ab"
Q0 = {11: 2088, 12: 2086, 13: 1981, 21: 1995, 22: 2108, 23: 2030,
      31: 2034, 32: 2042, 33: 2081, 41: 2073, 42: 2072, 43: 2025}
UNITS = {11: ("LF", "LOWER", "M33"), 12: ("LF", "UPPER", "ELR01"), 13: ("LF", "HIP", "M22"),
         21: ("RF", "LOWER", "NEW03"), 22: ("RF", "UPPER", "ELR03"), 23: ("RF", "HIP", "NEW01"),
         31: ("RH", "LOWER", "NEW05"), 32: ("RH", "UPPER", "ELR02"), 33: ("RH", "HIP", "NEW06"),
         41: ("LH", "LOWER", "M41"), 42: ("LH", "UPPER", "M42"), 43: ("LH", "HIP", "M43")}
DIRECTION = {"LF": {"HIP": 1, "UPPER": 1, "LOWER": 1}, "RF": {"HIP": 1, "UPPER": -1, "LOWER": -1},
             "RH": {"HIP": -1, "UPPER": -1, "LOWER": -1}, "LH": {"HIP": -1, "UPPER": 1, "LOWER": 1}}
# canonical contact / URDF limit depth from q0 per joint and side (ticks)
DEPTH = {"UPPER": {"MIN": (593, 597), "MAX": (1386, 1394)},
         "LOWER": {"MIN": (1048, 1047), "MAX": (434, 427)},
         "HIP": {"MIN": (523, 512), "MAX": (514, 512)}}


def bus_of(leg, joint):
    return next(b for b, (l, j, _) in UNITS.items() if l == leg and j == joint)


class FakeController:
    def __init__(self):
        self.link = None
        self.promoted = False
        self.authority = "NONE"
        self.session = None          # (leg, state)
        self.permit = False
        self.records = {}            # leg -> (verdict, accepted)
        self.run = None
        self.polls_before_result = 4
        self.fail_leg = None         # leg ends FAILED
        self.fail_leg_contacts = 5
        self.lying_leg = None        # leg says HARDWARE_CONTACT_CALIBRATED but 5 contacts
        self.fail_recovery = False
        self.recovery_q0_offset = 0  # TARGET lines report a q0 this far off the promotion
        self.wrong_armed_hip = False
        self.export_short = False    # the export claims 23/24
        self.incompatible_geometry = False
        self.lf_min_fine = (1463, 1459)
        self.q0 = dict(Q0)
        self.build_id = BUILD_ID
        self.uptime_ms = 100000
        self.capture_session = 1
        self.generation = 0
        self.acknowledged = 0
        self.save_fails = False
        self.post_abort_fails = False
        self.lose_link_during = None
        self.never_finish = False
        self.decorated_result_first = False
        self.wrong_leg_result_first = False
        self.last_leg = None
        self.sent = []
        self.bms_stream = False
        self.bms_pack = 12.0
        self.bms_cell = 4000
        self.bms_alarm = "0000"
        self.bms_comm = "OK"
        self.bms_age = 100
        self.bms_fault_leg = None

    def bms_status(self):
        self.emit("DALY   init=OK detected=ONLINE expected=REQUIRED result=PASS",
                  f"  comm={self.bms_comm} age_ms={self.bms_age}",
                  f"  pack_v={self.bms_pack:.1f} current_a=0.0 soc=99.0% cells=3",
                  f"  cell_max_mv={self.bms_cell} cell_min_mv={self.bms_cell} delta_mv=0",
                  f"  charge_mos=ON discharge_mos=ON state=IDLE alarms={self.bms_alarm} 0000 0000 0000")

    def emit(self, *lines):
        self.link.inject(lines)

    def later(self, delay, *lines):
        link = self.link
        threading.Timer(delay, lambda: link.inject(lines)).start()

    def corridor(self, leg, joint, side):
        q0 = self.q0[bus_of(leg, joint)]
        sign = DIRECTION[leg][joint] * (-1 if side == "MIN" else 1)
        contact_d, limit_d = DEPTH[joint][side]
        return sign, q0, q0 + sign * contact_d, q0 + sign * limit_d

    def contact_tick(self, leg, joint, side):
        if leg == "LF" and (joint, side) == ("UPPER", "MIN"):
            return self.lf_min_fine[1]
        sign, _, contact, _ = self.corridor(leg, joint, side)
        return contact + sign * 4

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
        m = re.fullmatch(r"@SERVO READ (\d+)", u)
        if m:
            if self.run:
                return self.emit("SERVO_READ=BLOCKED", "REASON=MOTION_EXECUTOR_ACTIVE")
            b = int(m.group(1))
            return self.emit(f"SERVO_READ id={b} position={self.q0.get(b, 2048)} speed=0 load=0 "
                             f"voltage=120 temp=31 torque=0 current=0")
        if u == "@STATUS":
            return self.emit(f"SYSTEM health=READY power_state=RUN mode=MAINTENANCE "
                             f"authority={self.authority} uptime_ms={self.uptime_ms} profile=ROBOT_POWERED",
                             "SERVO_POP canonical=17 expected_now=13 absent_by_design=4 last_census=NOT_RUN")
        if u in ("@BMS STREAM ON", "@BMS STREAM OFF"):
            self.bms_stream = u.endswith("ON")
            return self.emit("BMS_STREAM=" + ("ON" if self.bms_stream else "OFF"))
        if u == "@BMS STATUS":
            return self.bms_status()
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
        if u == "@CALIBRATION Q0 STATUS":
            lines = [f"CALIBRATION_Q0 state=COMPLETE failure=NONE session={self.capture_session} "
                     "sample_passes=9/9 next_joint=0 candidates=12/12",
                     f"CALIBRATION_Q0_PROMOTION promoted_capture={self.capture_session if self.promoted else 0} geometry=663f4d82f5817fb9"]
            for bus in sorted(self.q0):
                leg,joint,unit=UNITS[bus]
                lines.append(f"  Q0 bus={bus} leg={leg} joint={joint} unit={unit} tick={self.q0[bus]} "
                             "spread=0 samples=9 state=CANDIDATE estimator=MANUAL_ZERO_POSE")
            return self.emit(*lines)
        if u == "@CALIBRATION PERSIST SAVE CHECK":
            return self.emit("CALIBRATION_PERSIST_SAVE=CHECK_OK" if not self.save_fails else "CALIBRATION_PERSIST_SAVE=CHECK_REFUSED", "PERSISTED=0")
        if u == "@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION":
            self.generation+=1
            return self.emit(f"CALIBRATION_PERSIST_SAVE=WRITTEN_AWAITING_ACK generation={self.generation} slot=A")
        ack = re.fullmatch(r"@CALIBRATION PERSIST ACK (\d+)",u)
        if ack:
            self.acknowledged=int(ack.group(1))
            return self.emit(f"CALIBRATION_PERSIST_ACK=OK generation={self.acknowledged}")
        if u == "@CALIBRATION PERSIST STATUS":
            return self.emit("NVS=READY ESP_ERROR=0 PARTITION=matdog_nvs",
                             f"CLASS=CONSISTENT ACKNOWLEDGED_GENERATION={self.acknowledged} PENDING_GENERATION=0 "
                             "AWAITING_ACK_GENERATION=0 ACKNOWLEDGED_RECORD_INTACT=1",
                             "WRITE_STATE=OPEN ACK_UNCERTAIN=0 RECONCILE_UNCERTAIN=0 WRITES_BLOCKED=0",
                             "CALIBRATION_AVAILABLE=1 MOTION_AUTHORIZED=0 RESTORE=NOT_IMPLEMENTED")
        post = re.fullmatch(r"@CALIBRATION POST_ABORT RECOVERY (LF|RF|RH|LH) CONFIRM_Q0_RECOVERY", u)
        if post:
            if self.post_abort_fails:
                return self.emit("CALIBRATION_INITIAL_RECOVERY=REFUSED", "REASON=POST_ABORT_NO_WITNESS")
            real_emit=self.emit
            self.emit=lambda *lines: real_emit(*(t.replace("CALIBRATION_INITIAL_RECOVERY=ARMED", "CALIBRATION_POST_ABORT_RECOVERY=ARMED") for t in lines))
            self.handle(f"@CALIBRATION INITIAL RECOVERY {post.group(1)} CONFIRM_Q0_RECOVERY")
            self.emit=real_emit
            self.run['post_abort']=True
            return
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
            self.permit = False
            return self.emit(f"CALIBRATION_SESSION=ACTIVE leg={m.group(1)} session=1 authority_generation=1",
                             "CALIBRATION_SESSION_NOTE motion_permit=NOT_GRANTED hardware_motion_authorized=FALSE")
        if u == "@CALIBRATION STATUS":
            leg, state = self.session if self.session else ("LF", "NO_SESSION")
            p = "YES" if self.permit else "NO"
            return self.emit("CALIBRATION_CURRENT state=STALE_PENDING_FULL_RECALIBRATION hardware_motion=BLOCKED",
                             f"CALIBRATION_SESSION state={state} origin=LIVE_SESSION leg={leg} last_result=OK",
                             f"CALIBRATION_AUTHORITY held={'YES' if self.authority != 'NONE' else 'NO'} generation=1",
                             f"CALIBRATION_MOTION_PERMIT_STATE active={p} operator_authorized={p} token_valid={p}",
                             "CALIBRATION_POPULATION verdict=PASS observed=12/12 mask=0xfff")
        if u == "@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION":
            self.permit = True
            return self.emit("CALIBRATION_MOTION_PERMIT=ACTIVE generation=2 session=1 authority_generation=1 "
                             "direction_verify_budget_ticks=16", "CALIBRATION_MOTION_PERMIT_NOTE RAM_ONLY")
        m = re.fullmatch(r"@CALIBRATION INITIAL RECOVERY (LF|RF|RH|LH) CONFIRM_Q0_RECOVERY", u)
        if m:
            if not (self.session == (m.group(1), "ACTIVE") and self.permit):
                return self.emit("CALIBRATION_INITIAL_RECOVERY=REFUSED", "REASON=NO_CURRENT_MOTION_PERMIT")
            self.run = {"kind": "recovery", "leg": m.group(1), "polls": 0}
            lines = [f"CALIBRATION_INITIAL_RECOVERY=ARMED session_leg={m.group(1)} joints=12 "
                     f"torque_limit=500 phase=PREFLIGHT"]
            for bus in sorted(self.q0):
                leg, joint, unit = UNITS[bus]
                lines.append(f"CALIBRATION_INITIAL_RECOVERY_TARGET bus={bus} leg={leg} joint={joint} "
                             f"unit={unit} q0={self.q0[bus] + self.recovery_q0_offset}")
            lines.append("CALIBRATION_INITIAL_RECOVERY_NOTE no_write_in_command_handler")
            return self.emit(*lines)
        m = re.fullmatch(r"@CALIBRATION FULL LEG (LF|RF|RH|LH) CONFIRM_FULL_CALIBRATION", u)
        if m:
            leg = m.group(1)
            if not (self.session == (leg, "ACTIVE") and self.permit):
                return self.emit("CALIBRATION_FULL_LEG=REFUSED", "REASON=NO_CURRENT_MOTION_PERMIT")
            hip, upper, lower, park, park_bus = hw.LEG_MATRIX[leg]
            if self.wrong_armed_hip:
                hip += 1
            self.run = {"kind": "leg", "leg": leg, "polls": 0}
            lines = [f"CALIBRATION_FULL_LEG=ARMED leg={leg} contacts_expected=6 hip_bus={hip} "
                     f"upper_bus={upper} lower_bus={lower} park={park} park_bus={park_bus} phase=PREFLIGHT"]
            for joint in ("UPPER", "LOWER", "HIP"):
                for side in ("MIN", "MAX"):
                    sign, q0, contact, limit = self.corridor(leg, joint, side)
                    lines.append(f"CALIBRATION_FULL_LEG_SEARCH_CORRIDOR joint={joint} side={side} "
                                 f"probe_sign={sign} q0={q0} contact={contact} urdf_limit={limit} "
                                 f"entry={limit - sign * 64} guard={limit + sign * 64} opposite_limit=0 "
                                 f"guard_beyond_contact=60")
            for pose in ("UPPER_FOR_LOWER", "UPPER_FOR_HIP_MIN", "UPPER_FOR_HIP_MAX", "LOWER_FOLDED"):
                lines.append(f"CALIBRATION_FULL_LEG_PREREQUISITE pose={pose} urad=1570796 tick=3000")
            if park != "NONE":
                lines.append("CALIBRATION_FULL_LEG_PREREQUISITE pose=REAR_PARK urad=610865 tick=2470")
            lines.append("CALIBRATION_FULL_LEG_NOTE no_write_in_command_handler; poll with @CALIBRATION FULL LEG STATUS")
            return self.emit(*lines)
        if u == "@CALIBRATION FULL LEG STATUS":
            return self.status_poll()
        if u == "@CALIBRATION FULL LEG ABORT":
            if self.run:
                run, self.run = self.run, None
                leg = run["leg"]
                self.authority = "CALIBRATION" if run["kind"] == "recovery" else "NONE"
                if run["kind"] == "recovery":
                    return self.later(0.05, "CALIBRATION_FULL_LEG_ABORT=OK",
                                      "CALIBRATION_INITIAL_RECOVERY_RESULT verdict=FAILED recovered=3/12 "
                                      "failure=OPERATOR_ABORT failed_phase=INITIAL_RECOVERY last_decision=ACCEPT")
                self.records[leg] = ("FAILED", 0)
                self.session = (leg, "ABORTED")
                return self.later(0.05, "CALIBRATION_FULL_LEG_ABORT=OK",
                                  f"CALIBRATION_FULL_LEG_CONTACTS leg={leg} expected=6 measured=2 accepted=0 "
                                  f"diagnostics_accepted=NO",
                                  f"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict=FAILED failure=EXECUTOR_FAILED")
            return self.emit("CALIBRATION_FULL_LEG_ABORT=NO_ACTIVE_SEQUENCE")
        if u == "@CALIBRATION SESSION ABORT":
            self.authority = "NONE"
            self.permit = False
            if self.session:
                self.session = (self.session[0], "ABORTED")
            return self.emit("CALIBRATION_SESSION_ABORT=OK")
        if u == "@CALIBRATION EVIDENCE EXPORT":
            return self.export()
        self.emit(f"UNKNOWN_COMMAND={cmd}")

    def status_poll(self):
        lines = []
        run = self.run
        if run:
            if self.bms_stream:
                if run["leg"] == self.bms_fault_leg: self.bms_pack = 10.7
                self.bms_status()
            run["polls"] += 1
            leg = run["leg"]
            if self.lose_link_during == leg and run["kind"] == "leg" and run["polls"] == 2:
                self.link.drop()
                return
            if run["kind"] == "leg":
                lines.append(f"CALIBRATION_SEQUENCE leg={leg} phase=LOWER_MIN step=PROBE joint=LOWER bus=11 "
                             f"target=1040 held=3 contacts=2/6 recovered=12 prerequisites=VERIFIED failure=NONE")
                lines.append("CALIBRATION_SEARCH exec=LOWER_MIN joint=LOWER side=MIN pass=1 stage=FINE_SEARCH "
                             "probe=STEP_MONITORING target=1040 pos=1045 beyond_contact=3 contact=1040 "
                             "entry=1100 guard=972 speed=0 current=40 baseline=30/35 steps=12 bypass=0 "
                             "kplateau=0 scout=1034 p1=0 p2=0 failure=NONE")
            else:
                lines.append(f"CALIBRATION_SEQUENCE leg={leg} phase=INITIAL_RECOVERY step=MOVE_MONITOR joint=HIP "
                             f"bus=13 target=1981 held=0 contacts=0/6 recovered={run['polls']} "
                             f"prerequisites=NO failure=NONE")
            lines.append("CALIBRATION_FULL_LEG phase=LOWER_MIN step=PROBE failure=NONE failed_phase=- "
                         "last_decision=ACCEPT")
        for leg in ("LF", "RF", "RH", "LH"):
            if leg in self.records:
                v, n = self.records[leg]
                ok = v == "HARDWARE_CONTACT_CALIBRATED"
                lines.append(f"CALIBRATION_FULL_LEG_RECORD leg={leg} present=YES attempts=1 verdict={v} "
                             f"contacts_accepted={n}/6 contact_calibrated={'YES' if ok else 'NO'} "
                             f"envelope_accepted=NO failure={'NONE' if ok else 'EXECUTOR_FAILED'}")
            else:
                lines.append(f"CALIBRATION_FULL_LEG_RECORD leg={leg} present=NO verdict=NOT_RUN contacts_accepted=0/6")
        if self.last_leg and not run:
            for joint in ("UPPER", "LOWER", "HIP"):
                for side in ("MIN", "MAX"):
                    t = self.contact_tick(self.last_leg, joint, side)
                    f1, f2 = self.lf_min_fine if self.last_leg == "LF" and (joint, side) == ("UPPER", "MIN") else (t, t)
                    lines.append(f"CALIBRATION_FULL_LEG_CONTACT joint={joint} side={side} measured=YES "
                                 f"scout={t} fine1={f1} fine2={f2} witness_accepted=YES")
        # Printed by EVERY status poll in the real firmware - never a result.
        lines.append("CALIBRATION_FULL_LEG_NOTE FULL CALIBRATION = 4 legs x 3 joints x MIN/MAX = 24 contacts; "
                     "HARDWARE_CONTACT_CALIBRATED = all 6 of a leg's contacts recorded")
        self.emit(*lines)
        if run and not self.never_finish and run["polls"] >= self.polls_before_result:
            self.run = None
            leg = run["leg"]
            if run["kind"] == "recovery":
                if self.fail_recovery:
                    return self.later(0.05, "CALIBRATION_INITIAL_RECOVERY_RESULT verdict=FAILED recovered=4/12 "
                                            "failure=MOVE_TIMEOUT failed_phase=INITIAL_RECOVERY "
                                            "last_decision=ACCEPT")
                return self.later(0.05, f"CALIBRATION_{'POST_ABORT' if run.get('post_abort') else 'INITIAL'}_RECOVERY_RESULT verdict=PASS recovered=12/12 "
                                        "failure=NONE failed_phase=- last_decision=ACCEPT")
            failed = leg == self.fail_leg
            lying = leg == self.lying_leg
            verdict = "FAILED" if failed else "HARDWARE_CONTACT_CALIBRATED"
            failure = "EXECUTOR_FAILED" if failed else "NONE"
            measured = self.fail_leg_contacts if failed else (5 if lying else 6)
            accepted = 0 if failed else (5 if lying else 6)
            self.records[leg] = (verdict, accepted)
            self.last_leg = leg
            self.session = (leg, "COMPLETED" if not failed else "FAILED")
            self.authority = "NONE"
            self.permit = False
            final = []
            if self.wrong_leg_result_first:
                other = "RH" if leg != "RH" else "LH"
                final.append(f"CALIBRATION_FULL_LEG_RESULT leg={other} verdict=FAILED failure=EXECUTOR_FAILED")
            if self.decorated_result_first:
                final.append(f"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict=FAILED failure=X trailing_text")
                final.append(f"xCALIBRATION_FULL_LEG_RESULT leg={leg} verdict=FAILED failure=X")
            final.append(f"CALIBRATION_FULL_LEG_PROBE_FINAL leg={leg} joint=HIP side=MAX executor_failure="
                         f"{'HIP_MAX_PROBE_FAILED' if failed else 'NONE'} failed_phase=- probe_phase=COMPLETE "
                         f"probe_failure=NONE pass=2 stage=RELEASE target=1 pos=1 contact=1 guard=1 scout=1 "
                         f"p1=1 p2=1 bypass=0 steps=40")
            final.append(f"CALIBRATION_FULL_LEG_CONTACTS leg={leg} expected=6 measured={measured} "
                         f"accepted={accepted} diagnostics_accepted={'NO' if failed else 'YES'}")
            final.append(f"CALIBRATION_FULL_LEG_RESULT leg={leg} verdict={verdict} failure={failure}")
            self.later(0.05, *final)

    def export(self):
        lines = ["CALIBRATION_EVIDENCE_EXPORT=BEGIN format=2 geometry=663f4d82f5817fb9 parameters_approved=0 "
                 "contact_margin_ticks=8 contacts_per_leg=6 total_contacts_expected=24"]
        n = cal = total = 0
        for leg in ("LF", "RF", "RH", "LH"):
            rec = self.records.get(leg)
            if rec is None:
                lines.append(f"CALIBRATION_EVIDENCE_LEG leg={leg} present=0 attempts=0 verdict=NOT_RUN "
                             f"contacts_expected=6 contacts_accepted=0")
                continue
            v, accepted = rec
            n += 1
            ok = v == "HARDWARE_CONTACT_CALIBRATED"
            cal += 1 if ok and accepted == 6 else 0
            total += accepted if ok else 0
            lines.append(f"CALIBRATION_EVIDENCE_LEG leg={leg} present=1 attempts=1 session=1 "
                         f"geometry=663f4d82f5817fb9 verdict={v} contacts_expected=6 "
                         f"contacts_measured={accepted if ok else 2} contacts_accepted={accepted} "
                         f"diagnostics_accepted={1 if ok else 0} contact_calibrated={1 if ok else 0} "
                         f"envelope_accepted=0")
            lines.append(f"CALIBRATION_EVIDENCE_LEG_CLOSE leg={leg} failure={'NONE' if ok else 'EXECUTOR_FAILED'} "
                         f"executor_failure={'NONE' if ok else 'HIP_MAX_PROBE_FAILED'} "
                         f"failed_phase={'-' if ok else 'HIP_MAX'} session_completed={1 if ok else 0} "
                         f"permit_revoked=1 authority_released=1 parameters_approved=0")
            for joint in ("UPPER", "LOWER", "HIP"):
                bus=bus_of(leg,joint)
                geometry="OTHER" if self.incompatible_geometry and leg=="LF" else "663f4d82f5817fb9"
                lines.append(f"CALIBRATION_EVIDENCE_Q0 leg={leg} joint={joint} unit={UNITS[bus][2]} bus={bus} "
                             f"present=1 q0_tick={self.q0[bus]} state=PROMOTED origin=LIVE_SESSION geometry={geometry}")
                for side in ("MIN", "MAX"):
                    rec_ok = 1 if ok else 0
                    lines.append(f"CALIBRATION_EVIDENCE_CONTACT leg={leg} joint={joint} side={side} "
                                 f"recorded={rec_ok} measured={rec_ok} detection=CONTACT_CONFIRMED "
                                 f"state=PROMOTED origin=LIVE_SESSION scout_tick=1 fine1_tick=1 fine2_tick=1 "
                                 f"repeatability_ticks=0 witness_accepted={rec_ok}")
        if self.export_short:
            total -= 1
        lines.append(f"CALIBRATION_EVIDENCE_EXPORT=END legs_present={n} legs_contact_calibrated={cal} "
                     f"legs_envelope_accepted=0 total_contacts_expected=24 total_contacts_accepted={total} "
                     f"all_contact_calibrated={1 if cal == 4 and total == 24 else 0}")
        self.later(0.02, *lines)


class FakeLink:
    def __init__(self, port, log, controller):
        self.log = log
        self.lines = []
        self.cond = threading.Condition()
        self.lost = False
        self.controller = controller
        controller.link = self
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
        self.controller = FakeController()
        self.link = None
        hw.LEG_WATCHDOG_S = 420.0
        hw.RECOVERY_WATCHDOG_S = 120.0
        hw.POLL_S = 0.02

    def run_phase(self, phase, *extra, go=True):
        def factory(port, log):
            self.link = FakeLink(port, log, self.controller)
            return self.link

        port = os.path.join(self.tmp, "fake_port")
        open(port, "w").close()
        argv = ["--evidence-dir", self.tmp, "--port", port, "--phase", phase, "--no-flash",
                "--skip-build-check", "--build-id", BUILD_ID, "--quiet", *extra]
        if phase in ("q0", "all"):
            argv.append("--confirm-q0-pose")
        if phase in ("legs", "resume", "post-abort", "all") and go:
            argv.append("--confirm-operator-go")
        return hw.run(self._parse(argv), link_factory=factory)

    def _parse(self, argv):
        captured = {}
        real_run = hw.run
        hw.run = lambda args, link_factory=None: captured.setdefault("args", args)
        try:
            hw.main(argv)
        finally:
            hw.run = real_run
        return captured["args"]

    def sent(self):
        return self.controller.sent

    def ready(self, leg="LF"):
        self.assertEqual(self.run_phase("prepare"), 0)
        self.assertEqual(self.run_phase("q0"), 0)
        self.assertEqual(self.run_phase("recover", "--legs", leg), 0)

    # --- the phased flow ---------------------------------------------------------

    def test_prepare_moves_nothing(self):
        self.assertEqual(self.run_phase("prepare"), 0)
        cmds = self.sent()
        self.assertEqual(sum(1 for c in cmds if c.startswith("@SERVO SAFE_OFF")), 13)
        self.assertFalse(any("Q0 CAPTURE" in c or "SESSION START" in c or "RECOVERY" in c or
                             "FULL LEG" in c for c in cmds))

    def test_admitted_boot_anchor_is_checked_before_q0(self):
        anchor = time.time() - self.controller.uptime_ms / 1000
        self.assertEqual(self.run_phase("prepare", "--expected-boot-anchor", str(anchor)), 0)
        self.assertEqual(self.run_phase("all", "--expected-boot-anchor", str(anchor - 30)), 1)
        self.assertFalse(any("Q0 CAPTURE" in c for c in self.sent()))

    def test_nonfinite_boot_anchor_cannot_bypass_admission(self):
        self.assertEqual(self.run_phase("all", "--expected-boot-anchor", "nan"), 1)
        self.assertFalse(any("Q0 CAPTURE" in c for c in self.sent()))

    def test_interruption_aborts_and_safe_off(self):
        with patch.object(hw.Session, 'run_full_leg', side_effect=KeyboardInterrupt('test interruption')):
            self.assertEqual(self.run_phase("all"), 1)
        self.assertIn("@CALIBRATION FULL LEG ABORT", self.sent())
        self.assertTrue(all(f"@SERVO SAFE_OFF {bus}" in self.sent() for bus in hw.INSTALLED))

    def test_simulation_success_receipt_cannot_claim_actual_hardware(self):
        result = os.path.join(self.tmp, 'result.json')
        self.assertEqual(self.run_phase("all", "--result-json", result), 0)
        with open(result) as source: receipt = json.load(source)
        self.assertFalse(receipt['hardware_observed'])
        self.assertEqual(receipt['contacts_accepted'], 24)

    def test_q0_then_recover_stops_ready_for_go(self):
        self.ready()
        cmds = self.sent()
        self.assertLess(cmds.index("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE"),
                        cmds.index("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION"))
        self.assertLess(cmds.index("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION"),
                        cmds.index("@CALIBRATION SESSION START LF CONFIRM_CURRENT_Q0"))
        self.assertLess(cmds.index("@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION"),
                        cmds.index("@CALIBRATION INITIAL RECOVERY LF CONFIRM_Q0_RECOVERY"))
        # Ready, not started: no Full-Leg motion command at all.
        self.assertFalse(any(c.startswith("@CALIBRATION FULL LEG ") and "CONFIRM" in c for c in cmds))
        self.assertEqual(self.controller.session, ("LF", "ACTIVE"))
        self.assertTrue(self.controller.permit)
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "q0_promoted.json")))

    def test_go_runs_24_contacts_in_order(self):
        self.ready()
        mark = len(self.sent())
        self.assertEqual(self.run_phase("legs"), 0)
        cmds = self.sent()[mark:]
        full = [c for c in cmds if c.startswith("@CALIBRATION FULL LEG ") and "CONFIRM" in c]
        self.assertEqual(full, [f"@CALIBRATION FULL LEG {l} CONFIRM_FULL_CALIBRATION"
                                for l in ("LF", "RF", "RH", "LH")])
        # LF reuses the ready session; every later leg gets its own session,
        # permit and verified INITIAL RECOVERY before its FULL LEG.
        self.assertNotIn("@CALIBRATION SESSION START LF CONFIRM_CURRENT_Q0", cmds)
        for leg in ("RF", "RH", "LH"):
            s = cmds.index(f"@CALIBRATION SESSION START {leg} CONFIRM_CURRENT_Q0")
            r = cmds.index(f"@CALIBRATION INITIAL RECOVERY {leg} CONFIRM_Q0_RECOVERY")
            f = cmds.index(f"@CALIBRATION FULL LEG {leg} CONFIRM_FULL_CALIBRATION")
            self.assertLess(s, r)
            self.assertLess(r, f)
        self.assertEqual(cmds[-1], "@CALIBRATION EVIDENCE EXPORT")
        self.assertTrue(any(f.startswith("evidence_export_") for f in os.listdir(self.tmp)))

    def test_resume_preserves_lf_then_recovers_rf_and_finishes_24(self):
        self.ready();self.controller.fail_leg="RF"
        self.assertEqual(self.run_phase("legs"),1)
        self.controller.fail_leg=None
        mark=len(self.sent())
        self.assertEqual(self.run_phase("resume"),0)
        commands=self.sent()[mark:]
        self.assertNotIn("@CALIBRATION FULL LEG LF CONFIRM_FULL_CALIBRATION",commands)
        self.assertIn("@CALIBRATION POST_ABORT RECOVERY RF CONFIRM_Q0_RECOVERY",commands)
        self.assertEqual(self.controller.records['LF'],("HARDWARE_CONTACT_CALIBRATED",6))

    def test_resume_rejects_a_different_acquisition(self):
        self.ready();self.controller.capture_session=2
        mark=len(self.sent());self.assertEqual(self.run_phase("resume"),1)
        self.assertFalse(any("CONFIRM_FULL_CALIBRATION" in c for c in self.sent()[mark:]))

    def test_daly_same_link_healthy_full_sequence(self):
        self.assertEqual(self.run_phase("all", "--require-daly"), 0)
        self.assertIn("@BMS STREAM ON", self.sent())
        self.assertNotIn("@BMS KEY SET DISCHARGE CONFIRM", self.sent())
        self.assertEqual(self.sent()[-1], "@BMS STREAM OFF")

    def test_daly_fault_before_motion(self):
        self.controller.bms_cell = 3599
        self.assertEqual(self.run_phase("all", "--require-daly"), 1)
        self.assertFalse(any("CONFIRM_FULL_CALIBRATION" in c for c in self.sent()))

    def test_daly_fault_during_rf_stops_remaining_legs(self):
        self.controller.bms_fault_leg = "RF"
        self.assertEqual(self.run_phase("all", "--require-daly"), 1)
        self.assertIn("@CALIBRATION FULL LEG ABORT", self.sent())
        self.assertTrue(any(c.startswith("@SERVO SAFE_OFF") for c in self.sent()))
        self.assertNotIn("@CALIBRATION FULL LEG RH CONFIRM_FULL_CALIBRATION", self.sent())

    def test_verify_persistence_is_read_only(self):
        self.assertEqual(self.run_phase("all"),0); self.assertEqual(self.run_phase("persist"),0)
        self.controller.uptime_ms=500
        mark=len(self.sent()); self.assertEqual(self.run_phase("verify-persistence"),0)
        self.assertTrue(all(c=="" or c in ("@SYSTEM SOURCE_SIGNATURE", "@MODE STATUS", "@STATUS", "@AUTHORITY STATUS", "@CALIBRATION PERSIST STATUS") for c in self.sent()[mark:]))

    def test_resume_rejects_incompatible_retained_geometry(self):
        self.ready();self.controller.fail_leg="RF"
        self.assertEqual(self.run_phase("legs"),1)
        self.controller.incompatible_geometry=True
        mark=len(self.sent());self.assertEqual(self.run_phase("resume"),1)
        self.assertFalse(any("CONFIRM_FULL_CALIBRATION" in c for c in self.sent()[mark:]))

    def test_resume_rejects_changed_current_q0(self):
        self.ready();self.controller.q0[21]+=1
        mark=len(self.sent());self.assertEqual(self.run_phase("resume"),1)
        self.assertFalse(any("CONFIRM_FULL_CALIBRATION" in c for c in self.sent()[mark:]))

    def test_save_ack_and_read_after_simulated_reboot(self):
        self.assertEqual(self.run_phase("all"),0)
        self.assertEqual(self.run_phase("persist"),0)
        self.assertEqual(self.controller.acknowledged,1)
        self.controller.uptime_ms=500
        self.controller.promoted=False;self.controller.records={}
        self.assertEqual(self.run_phase("verify-persistence"),0)
        self.assertEqual(self.controller.acknowledged,1)

    def test_verify_persistence_requires_reboot_evidence(self):
        self.assertEqual(self.run_phase("all"),0);self.assertEqual(self.run_phase("persist"),0)
        mark=len(self.sent())
        self.assertEqual(self.run_phase("verify-persistence"),1)
        self.assertFalse(any(c.endswith("ABORT") or c.startswith("@SERVO") for c in self.sent()[mark:]))

    def test_save_check_failure_never_writes(self):
        self.assertEqual(self.run_phase("all"),0);self.controller.save_fails=True
        hw.POLL_S=.001
        # Refusal is immediate through a REASON line, matching the native surface.
        self.controller.emit_original=self.controller.emit
        self.controller.emit=lambda *lines: self.controller.emit_original(*lines, *(["REASON=LEG_RUN_NOT_CLOSED"] if "CALIBRATION_PERSIST_SAVE=CHECK_REFUSED" in lines else []))
        mark=len(self.sent());self.assertEqual(self.run_phase("persist"),1)
        self.assertNotIn("@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION",self.sent()[mark:])

    def test_all_in_one_invocation(self):
        self.assertEqual(self.run_phase("all"), 0)

    def test_legs_need_the_operator_go(self):
        self.ready()
        with self.assertRaises(SystemExit):
            self.run_phase("legs", go=False)
        with self.assertRaises(SystemExit):
            self.run_phase("all", go=False)

    def test_q0_pose_confirmation_is_mandatory(self):
        with self.assertRaises(SystemExit):
            hw.main(["--evidence-dir", self.tmp, "--phase", "q0", "--no-flash", "--skip-build-check"])

    # --- the 24-contact definition at the runner ---------------------------------

    def test_a_5_of_6_leg_stops_everything(self):
        self.controller.fail_leg = "RF"
        self.ready()
        mark = len(self.sent())
        self.assertEqual(self.run_phase("legs"), 1)
        cmds = self.sent()[mark:]
        self.assertIn("@CALIBRATION FULL LEG RF CONFIRM_FULL_CALIBRATION", cmds)
        self.assertNotIn("@CALIBRATION SESSION START RH CONFIRM_CURRENT_Q0", cmds)
        tail = cmds[cmds.index("@CALIBRATION FULL LEG RF CONFIRM_FULL_CALIBRATION"):]
        self.assertIn("@CALIBRATION FULL LEG ABORT", tail)
        self.assertIn("@CALIBRATION SESSION ABORT", tail)
        for bus in hw.INSTALLED:
            self.assertIn(f"@SERVO SAFE_OFF {bus}", tail)
        self.assertEqual(tail[-1], "@CALIBRATION EVIDENCE EXPORT")

    def test_a_calibrated_verdict_without_6_contacts_is_refused(self):
        self.controller.lying_leg = "LF"
        self.ready()
        self.assertEqual(self.run_phase("legs"), 1)
        self.assertNotIn("@CALIBRATION SESSION START RF CONFIRM_CURRENT_Q0", self.sent())

    def test_the_export_must_be_24_of_24(self):
        self.controller.export_short = True
        self.ready()
        self.assertEqual(self.run_phase("legs"), 1)

    def test_a_single_leg_is_never_claimed_as_full_calibration(self):
        self.ready()
        self.assertEqual(self.run_phase("legs", "--legs", "LF"), 0)
        with open(next(os.path.join(self.tmp, f) for f in sorted(os.listdir(self.tmp))
                       if f.startswith("hw_session_legs_"))) as f:
            log = f.read()
        self.assertIn("1 leg(s) x 6 = 6 contacts", log)
        self.assertNotIn("TRUE FULL CALIBRATION 24/24", log)

    # --- recovery -----------------------------------------------------------------

    def test_failed_recovery_stops_before_any_leg(self):
        self.controller.fail_recovery = True
        self.assertEqual(self.run_phase("prepare"), 0)
        self.assertEqual(self.run_phase("q0"), 0)
        self.assertEqual(self.run_phase("recover"), 1)
        self.assertFalse(any(c.startswith("@CALIBRATION FULL LEG ") and "CONFIRM" in c for c in self.sent()))
        self.assertIn("@CALIBRATION SESSION ABORT", self.sent())

    def test_recovery_targets_must_be_the_promoted_q0(self):
        self.controller.recovery_q0_offset = 3
        self.assertEqual(self.run_phase("prepare"), 0)
        self.assertEqual(self.run_phase("q0"), 0)
        self.assertEqual(self.run_phase("recover"), 1)
        self.assertIn("@CALIBRATION FULL LEG ABORT", self.sent())

    def test_only_status_polls_while_recovery_runs(self):
        self.controller.polls_before_result = 6
        self.assertEqual(self.run_phase("prepare"), 0)
        self.assertEqual(self.run_phase("q0"), 0)
        self.assertEqual(self.run_phase("recover"), 0)
        cmds = self.sent()
        i = cmds.index("@CALIBRATION INITIAL RECOVERY LF CONFIRM_Q0_RECOVERY")
        polls = 0
        for c in cmds[i + 1:]:
            if c != "@CALIBRATION FULL LEG STATUS":
                break
            polls += 1
        self.assertGreaterEqual(polls, 6)

    # --- exact terminal parsing, nothing mid-run ------------------------------------

    def test_note_line_is_never_a_result_and_nothing_else_is_sent_mid_run(self):
        self.controller.polls_before_result = 6
        self.ready()
        mark = len(self.sent())
        self.assertEqual(self.run_phase("legs", "--legs", "LF"), 0)
        cmds = self.sent()[mark:]
        i = cmds.index("@CALIBRATION FULL LEG LF CONFIRM_FULL_CALIBRATION")
        in_flight = []
        for c in cmds[i + 1:]:
            if c != "@CALIBRATION FULL LEG STATUS":
                break
            in_flight.append(c)
        self.assertGreaterEqual(len(in_flight), 6)
        self.assertEqual(cmds[i + 1 + len(in_flight)], "@SERVO SAFE_OFF 11")

    def test_decorated_or_other_leg_result_is_not_terminal(self):
        self.controller.decorated_result_first = True
        self.controller.wrong_leg_result_first = True
        self.ready()
        self.assertEqual(self.run_phase("legs", "--legs", "LF"), 0)

    def test_armed_plan_mismatch_aborts(self):
        self.controller.wrong_armed_hip = True
        self.ready()
        self.assertEqual(self.run_phase("legs", "--legs", "LF"), 1)
        self.assertIn("@CALIBRATION FULL LEG ABORT", self.sent())

    def test_lf_upper_min_absolute_drift_stops_before_rf(self):
        # This contact is +23 past canonical (the old check would pass),
        # but its absolute midpoint is 32 ticks from the hardware reference.
        self.controller.q0[12] = 2109
        self.controller.lf_min_fine = (1491, 1495)
        self.ready()
        self.assertEqual(self.run_phase("legs"), 1)
        self.assertNotIn("@CALIBRATION SESSION START RF CONFIRM_CURRENT_Q0", self.sent())

    def test_lf_upper_min_current_contact_is_independent_of_manual_q0(self):
        self.controller.q0[12] = 2105
        self.controller.lf_min_fine = (1461, 1465)
        self.ready()
        self.assertEqual(self.run_phase("legs"), 0)

    def test_lf_upper_min_absolute_comparison_includes_budget_boundary(self):
        self.controller.lf_min_fine = (1475, 1479)  # midpoint 1477 = 1461+16
        self.ready()
        self.assertEqual(self.run_phase("legs", "--legs", "LF"), 0)

    def test_lf_upper_min_absolute_comparison_rejects_half_tick_over_budget(self):
        self.controller.lf_min_fine = (1476, 1479)  # midpoint 1477.5
        self.ready()
        self.assertEqual(self.run_phase("legs"), 1)
        self.assertNotIn("@CALIBRATION SESSION START RF CONFIRM_CURRENT_Q0", self.sent())

    # --- the pre-motion stops ---------------------------------------------------------

    def test_q0_half_tooth_delta_stops_before_promotion(self):
        self.controller.q0[13] = hw.CR2C[13] + 82
        self.assertEqual(self.run_phase("prepare"), 0)
        self.assertEqual(self.run_phase("q0"), 1)
        self.assertNotIn("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION", self.sent())
        self.assertFalse(any("SESSION START" in c for c in self.sent()))

    def test_wrong_build_stops_before_any_motion(self):
        self.controller.build_id = "ffffffffffff"
        self.assertEqual(self.run_phase("prepare"), 1)
        # Only the always-allowed de-escalation (ABORTs, SAFE_OFF) may follow.
        self.assertFalse(any("Q0 CAPTURE" in c or "SESSION START" in c or "RECOVERY" in c or
                             ("FULL LEG" in c and "CONFIRM" in c) or "PERMIT GRANT" in c
                             for c in self.sent()))

    def test_session_start_requires_promoted_current_boot_q0(self):
        real = self.controller.handle

        def handle(cmd):
            if cmd == "@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION":
                self.controller.sent.append(cmd)
                return self.controller.emit("CALIBRATION_Q0_PROMOTE=REFUSED",
                                            "REASON=REJECT_FRESH_CAPTURE_NOT_COMPLETE")
            return real(cmd)
        self.controller.handle = handle
        self.assertEqual(self.run_phase("all"), 1)
        self.assertFalse(any("SESSION START" in c for c in self.sent()))

    def test_link_loss_mid_run_sends_nothing_more(self):
        self.controller.lose_link_during = "LF"
        self.ready()
        self.assertEqual(self.run_phase("legs"), 1)
        self.assertEqual(self.link.sent_after_loss, [])
        self.assertNotIn("@CALIBRATION SESSION START RF CONFIRM_CURRENT_Q0", self.sent())

    def test_silent_run_is_aborted_by_the_watchdog(self):
        self.ready()
        hw.LEG_WATCHDOG_S = 0.5
        self.controller.never_finish = True
        self.assertEqual(self.run_phase("legs", "--legs", "LF"), 1)
        self.assertIn("@CALIBRATION FULL LEG ABORT", self.sent())

    def test_missing_port_fails_cleanly_before_any_servo_command(self):
        opened = []
        real_timeout = hw.wait_for_port.__defaults__
        hw.wait_for_port.__defaults__ = (0.2, False)
        try:
            args = self._parse(["--evidence-dir", self.tmp, "--port", "/nonexistent/port",
                                "--phase", "prepare", "--no-flash", "--skip-build-check",
                                "--build-id", BUILD_ID, "--quiet"])
            rc = hw.run(args, link_factory=lambda port, log: opened.append(port))
        finally:
            hw.wait_for_port.__defaults__ = real_timeout
        self.assertEqual(rc, 1)
        self.assertEqual(opened, [])

    def test_flashing_requires_the_authorized_backup_sha256(self):
        flashed = []
        real_flash = hw.flash
        hw.flash = lambda *a, **k: flashed.append(a)
        try:
            for bad in ([], ["--backup-sha256", "7290a327"], ["--backup-sha256", "X" * 64]):
                with self.assertRaises(SystemExit):
                    hw.main(["--evidence-dir", self.tmp, "--phase", "prepare",
                             "--skip-build-check", "--port", "/nonexistent", *bad])
        finally:
            hw.flash = real_flash
        self.assertEqual(flashed, [])


if __name__ == "__main__":
    unittest.main(verbosity=1)
