"""Offline protocol/session tests for the Full Leg Calibrator V1 host runner."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
RUNNER_PATH = CALIBRATION_DIR / "matdog_full_leg_calibrator_runner.py"
SPEC = importlib.util.spec_from_file_location("flc_runner_under_test", RUNNER_PATH)
assert SPEC is not None and SPEC.loader is not None
runner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)


class FakeFirmware:
    """Small stateful implementation of the firmware's framed host protocol."""

    def __init__(self, *, stage: int = 6) -> None:
        self.stage = stage
        self.open_count = 0
        self.boot_counter = 0
        self.boot_id = "00000001"
        self.generation = 0
        self.active = "00000000"
        self.census_epoch = 0
        self.census_fresh = False
        self.session_fault = False
        self.manual_q0 = False
        self.bootstrap = False
        self.witnesses: set[int] = set()
        self.characterized: set[int] = set()
        self.calibrated: set[int] = set()
        self.commands: list[str] = []
        self.connections: list[FakeSerial] = []
        self.reset_during: str | None = None
        self.fail_safe_off = False

    def _reset_for_open(self) -> None:
        self.boot_counter += 1
        self.boot_id = f"{self.boot_counter:08X}"
        self.generation = 0
        self.active = "00000000"
        self.census_epoch = 0
        self.census_fresh = False
        self.session_fault = False
        self.manual_q0 = False
        self.bootstrap = False
        self.witnesses.clear()
        self.characterized.clear()
        self.calibrated.clear()

    def serial_factory(self, port: str, baud: int, *, timeout: float) -> "FakeSerial":
        del port, baud, timeout
        self.open_count += 1
        self._reset_for_open()
        connection = FakeSerial(self)
        self.connections.append(connection)
        return connection

    def status_lines(self) -> list[str]:
        state = 2 if self.session_fault else (1 if self.census_fresh else 0)
        return [
            "STATUS_BEGIN",
            "FIRMWARE_NAME=matdog_full_leg_calibrator_v1",
            "FIRMWARE_VERSION=1.0.0-test",
            f"BUILD_GIT_SHA={runner.git_commit()}",
            "BUILD_WORKTREE_DIRTY=NO",
            "BUILD_DATE=Aug 29 2026",
            "BUILD_TIME=12:00:00",
            f"PROTOCOL_ID={runner.PROTOCOL_ID}",
            f"PROTOCOL_SCOPE={runner.PROTOCOL_SCOPE}",
            f"AUTHORIZED_HARDWARE_STAGE=H{self.stage}",
            f"BOOT_SESSION_ID={self.boot_id}",
            f"ACTIVE_HOST_SESSION_ID={self.active.upper()}",
            f"SESSION_GENERATION={self.generation}",
            f"SESSION_STATE={state}",
            f"CENSUS_FRESH={'YES' if self.census_fresh else 'NO'}",
            f"CENSUS_EPOCH={self.census_epoch}",
            f"LAST_FAULT={'TEST_FAULT' if self.session_fault else 'NONE'}",
            "H3_BOOTSTRAP_BUILD_APPROVED=YES",
            f"H3_BOOTSTRAP_SESSION_APPROVED={'YES' if self.bootstrap else 'NO'}",
            f"H3_BOOTSTRAP_ACTIVE={'YES' if self.bootstrap else 'NO'}",
            f"MOTION_UNLOCKED={'YES' if self.bootstrap and self.census_fresh else 'NO'}",
            "PRE_MOTION_OUTSTANDING=0",
            f"JOINTS_CHARACTERIZED={len(self.characterized)}/12",
            f"MANUAL_Q0_CANDIDATES={12 if self.manual_q0 else 0}/12",
            f"DIRECTION_WITNESSES={len(self.witnesses)}/12",
            f"CALIBRATION_CANDIDATES={len(self.calibrated)}/12",
            "STATION_IN_CONTROL_PATH=NO",
            "EEPROM_WRITE_SURFACE=NONE",
            "BROADCAST_WRITE=NEVER",
            "GOAL_POSITION_AUTHORITY=flcWritePosEx",
            "GOAL_POSITION_DOMAIN=0..4095",
            "HEAD_SERVOS_EXPECTED_PRESENT=NO",
            "STATUS_RESULT PASS",
            "STATUS_END",
        ]

    @staticmethod
    def _frame(name: str, result: str = "PASS", *body: str) -> list[str]:
        return [f"{name}_BEGIN", *body, f"{name}_RESULT {result}", f"{name}_END"]

    def respond(self, command: str) -> list[str]:
        self.commands.append(command)
        if self.reset_during == command:
            self.reset_during = None
            return ["FULL_LEG_CALIBRATOR_READY"]
        if command == "@STATUS":
            return self.status_lines()
        if command.startswith("@SESSION_BEGIN "):
            host_id = command.rsplit(" ", 1)[1]
            self.census_fresh = False
            self.manual_q0 = False
            self.bootstrap = False
            self.witnesses.clear()
            self.characterized.clear()
            self.calibrated.clear()
            self.active = host_id
            self.generation = (self.generation + 1) & 0xFFFFFFFF
            return self._frame(
                "SESSION_BEGIN",
                "PASS",
                f"SESSION_BEGIN_BOOT_SESSION_ID={self.boot_id}",
                f"SESSION_BEGIN_ACTIVE_HOST_SESSION_ID={host_id.upper()}",
                f"SESSION_BEGIN_GENERATION={self.generation}",
            )
        if command == "@SESSION_END":
            self.census_fresh = False
            self.manual_q0 = False
            self.bootstrap = False
            self.witnesses.clear()
            self.characterized.clear()
            self.calibrated.clear()
            self.active = "00000000"
            return self._frame(
                "SESSION_END", "PASS", "SESSION_END_SAFE_OFF_VERIFIED=YES"
            )
        if command == "@SAFE_OFF":
            if self.fail_safe_off:
                return self._frame(
                    "SAFE_OFF", "FAIL", "SAFE_OFF_RESPONDERS_SEEN=YES",
                    "CUT_SERVO_POWER_NOW",
                )
            return self._frame(
                "SAFE_OFF", "PASS", "SAFE_OFF_RESPONDERS_SEEN=YES"
            )
        if command == "@CENSUS":
            self.census_epoch = (self.census_epoch + 1) & 0xFFFFFFFF
            self.census_fresh = True
            self.manual_q0 = False
            self.bootstrap = False
            self.witnesses.clear()
            self.characterized.clear()
            self.calibrated.clear()
            return self._frame("CENSUS", "PASS", "CENSUS_PRESENT=12/12")
        if command.startswith("@CAPTURE_Q0 "):
            if not self.census_fresh:
                return self._frame("CAPTURE_Q0", "REFUSED")
            self.manual_q0 = True
            return self._frame(
                "CAPTURE_Q0", "PASS",
                "CAPTURE_Q0_JOINT ID=13 CENTRE=2048 RESULT=CANDIDATE",
            )
        if command.startswith("@WITNESS_DIRECTION "):
            joint = int(command.split()[1])
            if not self.manual_q0:
                return self._frame("WITNESS_DIRECTION", "REFUSED")
            self.witnesses.add(joint)
            return self._frame(
                "WITNESS_DIRECTION", "PASS",
                f"WITNESS_DIRECTION ID={joint} SEMANTIC={command.split()[2]}",
            )
        if command == "@APPROVE_BOOTSTRAP CONFIRM":
            if not self.manual_q0:
                return self._frame("APPROVE_BOOTSTRAP", "REFUSED")
            self.bootstrap = True
            return self._frame("APPROVE_BOOTSTRAP")
        if command.startswith("@CHARACTERIZE_JOINT "):
            joint = int(command.split()[1])
            if not self.bootstrap or joint not in self.witnesses:
                return self._frame("CHARACTERIZE_JOINT", "REFUSED")
            self.characterized.add(joint)
            return self._frame(
                "CHARACTERIZE_JOINT", "PASS",
                f"MOTION ID={joint} SEGMENT=CHARACTERIZE",
                "CHARACTERIZE_JOINT_OUTPUT ORIGIN=CHARACTERIZED_CURRENT_HARDWARE",
            )
        if command.startswith("@CALIBRATE_JOINT "):
            joint = int(command.split()[1])
            if joint not in self.characterized:
                return self._frame("CALIBRATE_JOINT", "REFUSED")
            self.calibrated.add(joint)
            return self._frame(
                "CALIBRATE_JOINT", "PASS",
                f"MOTION ID={joint} SEGMENT=ENDPOINT_SEARCH",
                "CALIBRATE_JOINT_JOINT_MIN_ENDPOINT TICK=1000 ACCEPTED=0",
                "CALIBRATE_JOINT_JOINT_MAX_ENDPOINT TICK=3000 ACCEPTED=0",
                "CALIBRATE_JOINT_JOINT_DERIVED_Q0 TICK=2048 DIRECTION=1",
                "CALIBRATE_JOINT_JOINT_STATUS=OK TIER=CANDIDATE",
            )
        if command.startswith("@CALIBRATE_LEG "):
            return self._frame("CALIBRATE_LEG", "PASS", "MOTION SEGMENT=LEG")
        if command == "@CALIBRATE_ALL":
            return self._frame("CALIBRATE_ALL", "PASS", "MOTION SEGMENT=ALL")
        raise AssertionError(f"fake received unexpected command {command!r}")


class FakeSerial:
    def __init__(self, firmware: FakeFirmware) -> None:
        self.firmware = firmware
        self.closed = False
        self.rx: deque[bytes] = deque()
        self._queue(
            "MATDOG FULL LEG CALIBRATOR V1",
            "FULL_LEG_CALIBRATOR_READY",
            "",
            "Commands:",
            "  @SESSION_BEGIN <8-hex-host-id>",
            "  @HELP",
            "",
        )

    def _queue(self, *lines: str) -> None:
        self.rx.extend((line + "\n").encode("utf-8") for line in lines)

    def readline(self) -> bytes:
        return self.rx.popleft() if self.rx else b""

    def write(self, data: bytes) -> int:
        command = data.decode("ascii")
        if not command.endswith("\n") or command.count("\n") != 1:
            raise AssertionError(f"bad command framing {data!r}")
        self._queue(*self.firmware.respond(command[:-1]))
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class ScriptedSerial:
    """Minimal serial endpoint used for transport framing fault tests."""

    def __init__(self, responses: list[str]) -> None:
        self.rx: deque[bytes] = deque(
            (line + "\n").encode("utf-8")
            for line in [
                "FULL_LEG_CALIBRATOR_READY",
                "  @HELP",
                *responses,
            ]
        )
        self.writes: list[bytes] = []
        self.closed = False

    def readline(self) -> bytes:
        return self.rx.popleft() if self.rx else b""

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


def make_session(stage=runner.HardwareStage.H6_FOUR_LEGS) -> runner.Session:
    return runner.Session(
        mode="session",
        port="FAKE",
        requested_stage=runner.HardwareStage.H0_ESP32_ONLY,
        host_authorized_stage=stage,
    )


class TestSemanticSurfaceAndFraming(unittest.TestCase):
    def test_direction_witness_uses_exact_firmware_semantics_and_h2_stage(self) -> None:
        increase = runner.operation_spec(
            "witness-direction", joint=13, q_plus_raw="increases"
        )
        decrease = runner.operation_spec(
            "witness-direction", joint=13, q_plus_raw="decreases"
        )
        self.assertEqual(
            increase.command,
            "@WITNESS_DIRECTION 13 Q_PLUS_RAW_INCREASES CONFIRM",
        )
        self.assertEqual(
            decrease.command,
            "@WITNESS_DIRECTION 13 Q_PLUS_RAW_DECREASES CONFIRM",
        )
        self.assertIs(
            runner.MODE_STAGE["witness-direction"][0],
            runner.HardwareStage.H2_MANUAL_Q0,
        )

    def test_link_writes_one_ascii_line_and_requires_exact_frame(self) -> None:
        serial = ScriptedSerial(
            ["STATUS_BEGIN", "STATUS_RESULT PASS", "STATUS_END"]
        )
        link = runner.CalibratorLink("FAKE", serial_factory=lambda *a, **k: serial)
        with link:
            lines = link.send(
                "@STATUS", expected_begin="STATUS_BEGIN", expected_end="STATUS_END"
            )
        self.assertEqual(serial.writes, [b"@STATUS\n"])
        self.assertEqual(lines[0], "STATUS_BEGIN")
        self.assertEqual(lines[-1], "STATUS_END")

    def test_link_rejects_wrong_begin_cross_frame_and_reset_marker(self) -> None:
        cases = (
            (["SAFE_OFF_BEGIN"], runner.CalibratorProtocolError),
            (["STATUS_BEGIN", "SAFE_OFF_END"], runner.CalibratorProtocolError),
            (["FULL_LEG_CALIBRATOR_READY"], runner.CalibratorResetDetected),
        )
        for response, expected in cases:
            with self.subTest(response=response):
                serial = ScriptedSerial(response)
                link = runner.CalibratorLink(
                    "FAKE", serial_factory=lambda *a, _serial=serial, **k: _serial
                )
                with link:
                    with self.assertRaises(expected):
                        link.send(
                            "@STATUS",
                            expected_begin="STATUS_BEGIN",
                            expected_end="STATUS_END",
                        )

    def test_raw_unknown_and_injected_commands_are_never_writable(self) -> None:
        for command in (
            "@WRITE 13 0x2A 1",
            "@STATUS\n@CALIBRATE_ALL",
            "@WITNESS_DIRECTION 13 1 CONFIRM",
            "@CAPTURE_Q0 9999",
        ):
            self.assertFalse(runner.semantic_command_allowed(command), command)
        self.assertTrue(runner.semantic_command_allowed("@SAFE_OFF"))

    def test_workflow_parser_has_no_raw_command_escape_hatch(self) -> None:
        with self.assertRaises(ValueError):
            runner.parse_workflow_line("@STATUS")
        self.assertEqual(
            runner.parse_workflow_line("witness-direction 13 decreases"),
            runner.WorkflowAction(
                "witness-direction", joint=13, q_plus_raw="decreases"
            ),
        )
        self.assertEqual(
            runner.parse_workflow_line("pause inspect H2 evidence").label,
            "inspect H2 evidence",
        )


class TestPersistentPhysicalSession(unittest.TestCase):
    def test_h1_through_h4_use_one_open_link_with_pauses_and_checkpoints(self) -> None:
        firmware = FakeFirmware(stage=6)
        session = make_session()
        pauses: list[str] = []
        script = [
            "census",
            "pause review-h1",
            "capture-q0 64",
            "pause review-h2",
            "witness-direction 13 increases",
            "approve-bootstrap",
            "characterize-joint 13",
            "pause review-h3",
            "calibrate-joint 13",
            "end",
        ]
        with tempfile.TemporaryDirectory() as tmp:
            evidence_root = Path(tmp)
            link = runner.CalibratorLink(
                "FAKE", serial_factory=firmware.serial_factory
            )
            controller = runner.CalibrationSessionController(
                session,
                link,
                host_stage=runner.HardwareStage.H6_FOUR_LEGS,
                host_session_id="1a2b3c4d",
                evidence_root=evidence_root,
            )
            with controller:
                outcomes = runner.run_session_workflow(
                    controller, script, pause_callback=pauses.append
                )
                report_path = (
                    evidence_root
                    / "sessions"
                    / f"{session.started_utc.replace(':', '').replace('-', '')}_session_1a2b3c4d"
                    / "session_report.json"
                )
                self.assertTrue(report_path.is_file())
                checkpoint = json.loads(report_path.read_text(encoding="utf-8"))
                self.assertEqual(checkpoint["session"]["connection_open_count"], 1)
                self.assertEqual(controller.link.open_count, 1)

            report = json.loads(report_path.read_text(encoding="utf-8"))

        self.assertEqual([outcome.status for outcome in outcomes], ["PASS"] * 6)
        self.assertEqual(pauses, ["review-h1", "review-h2", "review-h3"])
        self.assertEqual(firmware.open_count, 1)
        self.assertEqual(session.connection_open_count, 1)
        self.assertTrue(firmware.connections[0].closed)
        self.assertEqual(session.continuity_state, "CLOSED_SAFE_OFF")
        self.assertEqual(session.firmware_boot_session_id, "00000001")
        self.assertEqual(session.firmware_session_generation, "1")
        self.assertEqual(session.requested_stage, runner.HardwareStage.H4_JOINT_CALIBRATE)
        self.assertEqual(report["session"]["result"], "PASS")
        self.assertEqual(report["session"]["connection_open_count"], 1)
        self.assertEqual(
            report["provenance"]["host_policy_authorized_stage"],
            "H6_FOUR_LEGS",
        )
        self.assertTrue(report["provenance"]["firmware_build_matches_local_git"])
        self.assertIn("geometry_parking_source_sha256", report["provenance"])
        self.assertIn("firmware_binary_sha256", report["provenance"])
        self.assertEqual(report["observed"]["eeprom_writes"], 0)
        self.assertEqual(report["observed"]["broadcast_writes"], 0)
        self.assertEqual(report["observed"]["final_torque_state"], "OFF_VERIFIED")

        expected_operations = [
            "@CENSUS",
            "@CAPTURE_Q0 64",
            "@WITNESS_DIRECTION 13 Q_PLUS_RAW_INCREASES CONFIRM",
            "@APPROVE_BOOTSTRAP CONFIRM",
            "@CHARACTERIZE_JOINT 13",
            "@CALIBRATE_JOINT 13",
        ]
        cursor = -1
        for command in expected_operations:
            next_cursor = firmware.commands.index(command, cursor + 1)
            self.assertGreater(next_cursor, cursor)
            if command != "@CENSUS":
                self.assertIn("@SAFE_OFF", firmware.commands[cursor + 1:next_cursor])
            cursor = next_cursor
        self.assertEqual(firmware.commands.count("@SESSION_BEGIN 1a2b3c4d"), 1)
        self.assertEqual(firmware.commands.count("@SESSION_END"), 1)
        self.assertEqual(firmware.commands[-1], "@SESSION_END")

    def test_pause_checkpoint_marks_link_held_open(self) -> None:
        firmware = FakeFirmware()
        session = make_session()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            controller = runner.CalibrationSessionController(
                session,
                runner.CalibratorLink("FAKE", serial_factory=firmware.serial_factory),
                host_stage=runner.HardwareStage.H6_FOUR_LEGS,
                host_session_id="10203040",
                evidence_root=root,
            )
            controller.start()
            controller.pause("physical-review")
            report_path = next((root / "sessions").glob("*/session_report.json"))
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(
                report["session"]["continuity_state"], "PAUSED_LINK_HELD_OPEN"
            )
            self.assertIsNotNone(controller.link.serial)
            self.assertEqual(controller.link.open_count, 1)
            controller.finish()

    def test_boot_id_generation_and_active_id_changes_each_fail_closed(self) -> None:
        mutations = (
            lambda fw: setattr(fw, "boot_id", "ABCDEF12"),
            lambda fw: setattr(fw, "generation", fw.generation + 1),
            lambda fw: setattr(fw, "active", "decafbad"),
        )
        expected_text = ("BOOT_SESSION_ID changed", "SESSION_GENERATION changed", "active host session")
        for mutate, message in zip(mutations, expected_text):
            with self.subTest(message=message):
                firmware = FakeFirmware()
                session = make_session()
                controller = runner.CalibrationSessionController(
                    session,
                    runner.CalibratorLink(
                        "FAKE", serial_factory=firmware.serial_factory
                    ),
                    host_stage=runner.HardwareStage.H6_FOUR_LEGS,
                    host_session_id="11223344",
                    evidence_root=None,
                )
                controller.start()
                mutate(firmware)
                with self.assertRaisesRegex(runner.CalibratorResetDetected, message):
                    controller.execute("status")
                self.assertTrue(controller.faulted)
                self.assertFalse(controller.active)
                self.assertEqual(session.continuity_state, "FAULT")
                self.assertEqual(firmware.open_count, 1)
                controller.finish(aborted=True)
                self.assertNotIn("@SESSION_END", firmware.commands)

    def test_ready_during_command_is_reset_evidence_and_stops_sequence(self) -> None:
        firmware = FakeFirmware()
        session = make_session()
        controller = runner.CalibrationSessionController(
            session,
            runner.CalibratorLink("FAKE", serial_factory=firmware.serial_factory),
            host_stage=runner.HardwareStage.H6_FOUR_LEGS,
            host_session_id="55667788",
            evidence_root=None,
        )
        controller.start()
        self.assertTrue(controller.execute("census").passed)
        firmware.reset_during = "@CAPTURE_Q0 64"
        with self.assertRaises(runner.CalibratorResetDetected):
            controller.execute("capture-q0")
        self.assertTrue(controller.faulted)
        self.assertEqual(session.continuity_state, "FAULT")
        controller.finish(aborted=True)

    def test_reconnect_starts_empty_and_cannot_restore_prior_authorization(self) -> None:
        firmware = FakeFirmware()
        first_session = make_session()
        first = runner.CalibrationSessionController(
            first_session,
            runner.CalibratorLink("FAKE", serial_factory=firmware.serial_factory),
            host_stage=runner.HardwareStage.H6_FOUR_LEGS,
            host_session_id="a1b2c3d4",
            evidence_root=None,
        )
        with first:
            self.assertTrue(first.execute("census").passed)
            self.assertTrue(first.execute("capture-q0").passed)
        first_boot = first.boot_session_id

        second_session = make_session()
        second = runner.CalibrationSessionController(
            second_session,
            runner.CalibratorLink("FAKE", serial_factory=firmware.serial_factory),
            host_stage=runner.HardwareStage.H6_FOUR_LEGS,
            host_session_id="a1b2c3d4",
            evidence_root=None,
        )
        with second:
            commands_before = list(firmware.commands)
            outcome = second.execute("capture-q0")
            self.assertEqual(outcome.status, "REFUSED")
            self.assertEqual(outcome.reason, "HOST_EVIDENCE_MISSING")
            self.assertNotIn("@CAPTURE_Q0 64", firmware.commands[len(commands_before):])
            self.assertIsNone(second.manual_q0_epoch)
            self.assertFalse(second.census_valid)
        self.assertNotEqual(first_boot, second.boot_session_id)
        self.assertEqual(firmware.open_count, 2)

    def test_safe_off_failure_faults_and_never_advances(self) -> None:
        firmware = FakeFirmware()
        session = make_session()
        controller = runner.CalibrationSessionController(
            session,
            runner.CalibratorLink("FAKE", serial_factory=firmware.serial_factory),
            host_stage=runner.HardwareStage.H6_FOUR_LEGS,
            host_session_id="1234abcd",
            evidence_root=None,
        )
        controller.start()
        firmware.fail_safe_off = True
        with self.assertRaisesRegex(runner.CalibratorProtocolError, "SAFE_OFF failed"):
            controller.execute("census")
        self.assertTrue(controller.faulted)
        self.assertEqual(session.continuity_state, "FAULT")
        self.assertIn("CUT_SERVO_POWER_NOW", session.all_lines)
        controller.finish(aborted=True)
        self.assertNotIn("@SESSION_END", firmware.commands)


class TestFirmwareOutputContract(unittest.TestCase):
    """The runner must read the tokens the shared orchestrator actually emits.

    H4, H5 and H6 no longer print a per-joint TORQUE_OFF_VERIFIED=YES summary;
    the one production orchestrator prints a single run-level release line. A
    runner that only knew the old token would silently downgrade a verified
    release to UNKNOWN, which is exactly the kind of quiet evidence loss the
    session report exists to prevent.
    """

    def _session(self, lines: list[str]) -> runner.Session:
        session = runner.Session(
            mode="calibrate-all",
            port="/dev/null",
            requested_stage=runner.HardwareStage.H6_FOUR_LEGS,
        )
        session.commands.append({"command": "@CALIBRATE_ALL", "response": lines})
        return session

    def test_run_level_safe_off_counts_as_verified_release(self) -> None:
        session = self._session([
            "MOTION ID=13 GOAL=2048",
            "CALIBRATE_ALL_SAFE_OFF_VERIFIED=YES",
            "CALIBRATE_ALL_RESULT PASS",
        ])
        self.assertEqual(session.final_torque_state(), "OFF_VERIFIED")

    def test_h3_single_joint_release_is_still_recognised(self) -> None:
        session = self._session([
            "MOTION ID=13 GOAL=2048",
            "CHARACTERIZE_JOINT_TORQUE_OFF_VERIFIED=YES",
        ])
        self.assertEqual(session.final_torque_state(), "OFF_VERIFIED")

    def test_cut_power_outranks_any_release_claim(self) -> None:
        session = self._session([
            "MOTION ID=13 GOAL=2048",
            "CALIBRATE_ALL_SAFE_OFF_VERIFIED=NO",
            "CUT_SERVO_POWER_NOW",
        ])
        self.assertEqual(session.final_torque_state(), "UNVERIFIED_HARD_FAULT")

    def test_motion_without_any_release_evidence_is_unknown(self) -> None:
        session = self._session([
            "MOTION ID=13 GOAL=2048",
            "CALIBRATE_ALL_RESULT FAIL",
        ])
        self.assertEqual(session.final_torque_state(), "UNKNOWN")

    def test_no_motion_at_all_reports_never_enabled(self) -> None:
        session = self._session(["CALIBRATE_ALL_RESULT REFUSED"])
        self.assertEqual(session.final_torque_state(), "NEVER_ENABLED")


class TestHostAuthorization(unittest.TestCase):
    @staticmethod
    def _spec(mode: str):
        kwargs = {}
        if mode in ("witness-direction", "characterize-joint", "calibrate-joint"):
            kwargs["joint"] = 13
        if mode == "witness-direction":
            kwargs["q_plus_raw"] = "increases"
        if mode == "calibrate-leg":
            kwargs["leg"] = "LF"
        return runner.operation_spec(mode, **kwargs)

    def test_h0_blocks_every_higher_one_shot_before_any_transmit(self) -> None:
        class NoSendLink:
            def send(self, *args, **kwargs):
                raise AssertionError("unauthorized one-shot reached the transport")

        for mode, (needed, _) in runner.MODE_STAGE.items():
            if needed.value == 0 or mode == "session":
                continue
            with self.subTest(mode=mode):
                session = runner.Session(mode, "FAKE", needed)
                runner.run_named(
                    session,
                    NoSendLink(),
                    self._spec(mode),
                    runner.HardwareStage.H0_ESP32_ONLY,
                )
                self.assertTrue(session.refused)

    def test_h0_blocks_every_higher_persistent_action_even_on_h6_firmware(self) -> None:
        firmware = FakeFirmware(stage=6)
        session = make_session(runner.HardwareStage.H0_ESP32_ONLY)
        controller = runner.CalibrationSessionController(
            session,
            runner.CalibratorLink("FAKE", serial_factory=firmware.serial_factory),
            host_stage=runner.HardwareStage.H0_ESP32_ONLY,
            host_session_id="cafef00d",
            evidence_root=None,
        )
        controller.start()
        lifecycle_commands = list(firmware.commands)
        for mode, (needed, _) in runner.MODE_STAGE.items():
            if needed.value == 0 or mode == "session":
                continue
            spec = self._spec(mode)
            outcome = controller.execute(
                mode,
                joint=spec.servo_ids[0] if mode in (
                    "witness-direction", "characterize-joint", "calibrate-joint"
                ) else None,
                leg="LF" if mode == "calibrate-leg" else None,
                q_plus_raw="increases" if mode == "witness-direction" else None,
            )
            self.assertEqual(outcome.status, "REFUSED", mode)
            self.assertEqual(outcome.reason, "HOST_STAGE_LOCKED", mode)
        self.assertEqual(firmware.commands, lifecycle_commands)
        controller.finish()

    def test_main_refuses_h2_one_shot_without_opening_serial(self) -> None:
        calls = 0

        def forbidden_factory(*args, **kwargs):
            nonlocal calls
            calls += 1
            raise AssertionError("serial must not open")

        result = runner.main(
            ["capture-q0", "--no-write"], serial_factory=forbidden_factory
        )
        self.assertEqual(result, 0)
        self.assertEqual(calls, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
