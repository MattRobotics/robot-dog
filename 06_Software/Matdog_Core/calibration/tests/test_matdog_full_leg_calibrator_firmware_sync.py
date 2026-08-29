"""Static safety audit of the Full Leg Calibrator V1 firmware source.

Some of the V1 safety contract cannot be proven by running the firmware — the
strongest evidence for "there is no EEPROM write path" is that the calling code
does not exist. These tests read the sketch and assert those structural
properties, and pin the constants the firmware and the host policy must agree on
so the two cannot drift apart silently.

They also confirm the frozen bench tooling was not edited.
"""

from __future__ import annotations

import hashlib
import re
import subprocess
import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

REPO_ROOT = CALDIR.parents[2]
SKETCH = (
    REPO_ROOT
    / "05_Firmware"
    / "Full_Leg_Calibrator_V1"
    / "matdog_full_leg_calibrator_v1"
    / "matdog_full_leg_calibrator_v1.ino"
)
DETECTOR = SKETCH.parent / "flc_contact_detector.h"
ENGINE = SKETCH.parent / "flc_calibration_engine.h"
GEOMETRY_PLAN = SKETCH.parent / "flc_leg_plan.h"
FIRMWARE_TOOLS = SKETCH.parent.parent / "tools"
FROZEN_DIR = REPO_ROOT / "05_Firmware" / "ST3215_Bench_Tools"

import matdog_full_leg_calibrator_policy as policy  # noqa: E402


def strip_comments(source: str) -> str:
    """Remove block and line comments so prose cannot satisfy a code assertion."""
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def strip_string_literals(source: str) -> str:
    """Also remove string literals, for checks about real dependencies.

    The STATUS banner legitimately prints the string "Station in path  : NO",
    which is an assertion that Station is absent, not a use of it.
    """
    return re.sub(r'"(?:[^"\\]|\\.)*"', '""', source)


class TestFirmwareStructure(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.raw = SKETCH.read_text(encoding="utf-8")
        cls.code = strip_comments(cls.raw)
        cls.executable = strip_string_literals(cls.code)
        cls.detector_code = strip_comments(DETECTOR.read_text(encoding="utf-8"))
        cls.engine_code = strip_comments(ENGINE.read_text(encoding="utf-8"))

    def test_sketch_and_detector_exist(self):
        self.assertTrue(SKETCH.is_file())
        self.assertTrue(DETECTOR.is_file())

    def test_no_eeprom_unlock_or_lock_call(self):
        for forbidden in ("unLockEprom", "LockEprom"):
            with self.subTest(call=forbidden):
                self.assertIsNone(
                    re.search(rf"\b{forbidden}\s*\(", self.executable)
                )

    def test_calibration_ofs_is_never_called(self):
        self.assertIsNone(re.search(r"\bCalibrationOfs\s*\(", self.executable))

    def test_no_factory_reset_call(self):
        for forbidden in ("factoryReset", "FactoryReset", "resetServo", "Reset"):
            with self.subTest(call=forbidden):
                self.assertIsNone(
                    re.search(rf"(?:\.|\b){forbidden}\s*\(", self.executable)
                )

    def test_no_write_to_any_eeprom_register(self):
        """PositionOffset, ID, Lock and the 20 profile registers are read-only."""
        for register in ("REG_POSITION_OFFSET", "REG_ID", "REG_LOCK"):
            for writer in ("writeByte", "writeWord", "flcWrite"):
                pattern = rf"{writer}\s*\([^)]*{register}"
                with self.subTest(register=register, writer=writer):
                    self.assertIsNone(
                        re.search(pattern, self.code),
                        f"{writer} appears to target {register}",
                    )

    def test_write_allowlist_default_branch_refuses(self):
        """writeAllowed() must fall through to `return false`, not to permission."""
        match = re.search(r"bool writeAllowed\(.*?\n\}", self.code, re.DOTALL)
        self.assertIsNotNone(match, "writeAllowed() not found")
        body = match.group(0)
        self.assertIn("default:", body)
        self.assertIn("return false;", body)
        # Only two addresses may ever be written.
        self.assertIn("case REG_TORQUE_ENABLE:", body)
        self.assertIn("case REG_TORQUE_LIMIT:", body)
        self.assertNotIn("case REG_LOCK:", body)
        self.assertNotIn("case REG_POSITION_OFFSET:", body)
        self.assertNotIn("case REG_ID:", body)

    def test_exactly_one_goal_position_authority(self):
        """WritePosEx may be called from exactly one function in the firmware."""
        occurrences = re.findall(r"st\.WritePosEx\s*\(", self.executable)
        self.assertEqual(len(occurrences), 1, "more than one GoalPosition writer")

    def test_no_alternative_motion_primitives(self):
        for forbidden in ("RegWritePosEx", "SyncWritePosEx", "WheelMode", "WriteSpe"):
            with self.subTest(call=forbidden):
                self.assertIsNone(
                    re.search(rf"(?:\.|\b){forbidden}\s*\(", self.executable)
                )

    def test_no_broadcast_id_anywhere(self):
        # Word-bounded so digits inside geometry float literals (0.666361254f)
        # are not mistaken for the broadcast id.
        self.assertIsNone(
            re.search(r"\b254\b", self.code), "broadcast id 254 appears in firmware code"
        )
        # Every write is guarded by a leg-id check.
        self.assertIn("validLegId", self.code)

    def test_station_is_absent_from_the_control_path(self):
        """No Station include, symbol or call — only the banner asserting absence."""
        executable = strip_string_literals(self.code)
        for token in ("Station", "station", "norma", "Norma", "NormaCore"):
            with self.subTest(token=token):
                self.assertNotIn(token, executable)
        # The STATUS banner must still positively declare the absence.
        self.assertIn("STATION_IN_CONTROL_PATH=NO", self.raw)

    def test_only_expected_headers_are_included(self):
        includes = set(re.findall(r'#include\s+[<"]([^>"]+)[>"]', self.code))
        self.assertEqual(includes, {
            "Arduino.h", "SCServo.h", "esp_system.h", "flc_stage_config.h",
            "flc_contact_detector.h", "flc_calibration_engine.h", "flc_leg_plan.h",
        })

    def test_session_authorization_is_ram_only_and_reset_detectable(self):
        """A USB reset must invalidate authorization; no NVS/EEPROM restoration."""
        for required in (
            "bootSessionId", "activeHostSessionId", "sessionGeneration",
            "censusEpoch", "@SESSION_BEGIN", "@SESSION_END",
        ):
            with self.subTest(required=required):
                self.assertIn(required, self.raw)
        for forbidden in ("Preferences.h", "nvs_flash", "EEPROM.h", "RTC_DATA_ATTR"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.executable)

    def test_geometry_plan_is_on_the_production_path(self):
        self.assertTrue(GEOMETRY_PLAN.is_file())
        self.assertIn('#include "flc_leg_plan.h"', self.raw)
        plan = strip_comments(GEOMETRY_PLAN.read_text(encoding="utf-8"))
        for provenance in (
            "FLC_GEOMETRY_PARKING_FILE_SHA256",
            "FLC_GEOMETRY_PARKING_SEMANTIC_SHA256",
            "FLC_GEOMETRY_ARTIFACT_GRANTS_MOTION_AUTHORIZATION",
        ):
            with self.subTest(provenance=provenance):
                self.assertIn(provenance, plan)

    def test_engine_is_actually_used_by_the_firmware(self):
        """The detector/engine must be on the real firmware path, not just built."""
        for symbol in ("flcCharacterizeJoint", "flcRunCalibrationPlan",
                       "flcEndMotion", "flcBootstrapEnvelope"):
            with self.subTest(symbol=symbol):
                self.assertIn(symbol + "(", self.code)

    def test_h4_h5_and_h6_share_one_orchestration_entry_point(self):
        """The tested orchestration must be the executed orchestration.

        H4, H5 and H6 may differ only in their selection mask. A firmware-local
        leg loop would mean the offline suite proves nothing about H6.
        """
        self.assertEqual(self.code.count("flcRunCalibrationPlan("), 1)
        for legacy in ("flcCalibrateJoint(", "flcCalibrateLeg(",
                       "flcCalibrateAllLegs("):
            with self.subTest(symbol=legacy):
                self.assertNotIn(legacy, self.code)
        for mode in ("runCalibrateJoint", "runCalibrateLeg", "runCalibrateAll"):
            with self.subTest(mode=mode):
                self.assertIn("runCalibrationSelection(", self.code)

    def test_the_engine_port_validates_session_context(self):
        """Every read and motion-capable write is gated on session validity."""
        self.assertIn("port.validateContext = portValidateContext", self.code)

    def test_firmware_reports_no_wall_clock_build_stamp(self):
        """__DATE__/__TIME__ would make the image differ on every rebuild.

        A binary hash that changes without the source changing cannot be used as
        a pre-flash integrity check, and a hash published for a commit becomes
        unattainable minutes later. Build metadata must come from the commit.
        """
        for macro in ("__DATE__", "__TIME__", "__TIMESTAMP__"):
            with self.subTest(macro=macro):
                self.assertNotIn(macro, self.executable)
        self.assertIn("BUILD_SOURCE_EPOCH=", self.raw)
        self.assertIn("FLC_BUILD_SOURCE_EPOCH", self.code)

    def test_build_script_pins_source_date_epoch_and_commit_stamp(self):
        """Reproducibility needs BOTH: our stamp and GCC's __DATE__ override.

        The Arduino ESP32 core embeds its own "Compile Date" string, so removing
        the macros from this sketch alone is not enough — two clean builds still
        differed until SOURCE_DATE_EPOCH pinned the core's expansion too.
        """
        script = (FIRMWARE_TOOLS / "build_stage.sh").read_text(encoding="utf-8")
        self.assertIn("GIT_COMMIT_EPOCH=", script)
        self.assertIn("export SOURCE_DATE_EPOCH=", script)
        self.assertIn("-DFLC_BUILD_SOURCE_EPOCH=", script)
        self.assertIn("-DFLC_BUILD_GIT_SHA_TOKEN=", script)
        self.assertIn("-DFLC_BUILD_WORKTREE_DIRTY=", script)

    def test_a_reproducible_build_checker_exists_and_forces_a_cold_cache(self):
        checker = FIRMWARE_TOOLS / "check_reproducible_build.sh"
        self.assertTrue(checker.is_file())
        text = checker.read_text(encoding="utf-8")
        # A cached object would hide the very nondeterminism this proves absent.
        self.assertIn("--clean", text)
        self.assertIn("export SOURCE_DATE_EPOCH=", text)
        self.assertIn("REPRODUCIBLE_BUILD=PASS", text)

    def test_acceptance_gates_are_value_initialised(self):
        """An unassigned gate must read UNKNOWN, never uninitialised stack.

        FlcAcceptanceGates carries `...ToleranceKnown` booleans. If the struct
        were default-initialised, an indeterminate byte could make the engine
        treat an uncharacterised tolerance as validated and promote a result to
        ACCEPTED — the one tier this firmware may never fabricate.
        """
        body = self._function_body("makeAcceptanceGates")
        self.assertIn("FlcAcceptanceGates gates = FlcAcceptanceGates()", body)
        for field in ("repeatabilityToleranceKnown", "endpointVsUrdfToleranceKnown",
                      "manualVsDerivedQ0ToleranceKnown"):
            with self.subTest(field=field):
                self.assertIn(f"gates.{field} = false", body)

    def test_the_manual_q0_agreement_tolerance_is_declared_uncharacterised(self):
        """No build has characterised it, so H4 must stay BLOCKED, not ACCEPTED."""
        body = self._function_body("makeAcceptanceGates")
        self.assertIn("gates.manualVsDerivedQ0ToleranceKnown = false", body)
        self.assertNotIn("gates.manualVsDerivedQ0ToleranceKnown = true", body)

    def test_guard_config_assigns_every_detector_field(self):
        """A guard left uninitialised is a guard that may silently not fire."""
        struct = re.search(r"struct FlcContactConfig\s*\{(.*?)\n\};",
                           DETECTOR.read_text(encoding="utf-8"), flags=re.DOTALL)
        self.assertIsNotNone(struct)
        fields = re.findall(r"\b(?:u?int(?:8|16|32)_t|int|bool|float)\s+(\w+)\s*;",
                            strip_comments(struct.group(1)))
        self.assertGreaterEqual(len(fields), 10)
        body = self._function_body("makeGuards")
        for field in fields:
            with self.subTest(field=field):
                self.assertIn(f"guards.{field} =", body)

    def _function_body(self, name: str) -> str:
        """Source of one firmware function, comments stripped."""
        start = self.code.index(f"{name}(")
        open_brace = self.code.index("{", start)
        depth, i = 0, open_brace
        while i < len(self.code):
            if self.code[i] == "{":
                depth += 1
            elif self.code[i] == "}":
                depth -= 1
                if depth == 0:
                    return self.code[open_brace:i + 1]
            i += 1
        raise AssertionError(f"unterminated function {name}")

    def test_engine_reaches_the_detector(self):
        for symbol in ("flcDetectorInit", "flcDetectorObserve", "flcEvaluateRepeatability"):
            with self.subTest(symbol=symbol):
                self.assertIn(symbol + "(", self.engine_code)

    def test_engine_has_no_arduino_dependency(self):
        for token in ("Arduino.h", "SCServo", "Serial.", "delay("):
            with self.subTest(token=token):
                self.assertNotIn(token, self.engine_code)

    def test_no_per_leg_state_machines(self):
        """One generic engine + data, not four copied leg programs."""
        for forbidden in ("LfStateMachine", "RfStateMachine", "RhStateMachine",
                          "LhStateMachine", "calibrateLF", "calibrateRF"):
            with self.subTest(symbol=forbidden):
                self.assertNotIn(forbidden, self.code)

    def test_motion_primitive_enforces_unsigned_domain(self):
        match = re.search(r"bool flcWritePosEx\(.*?\n\}", self.code, re.DOTALL)
        self.assertIsNotNone(match)
        body = match.group(0)
        self.assertIn("position < 0", body)
        self.assertIn("ENCODER_MAX", body)
        self.assertIn("STAGE_MOTION", body)
        self.assertIn("AUTHORIZED_STAGE", body)
        # Motion requires either resolved pre-motion parameters or an explicitly
        # approved bootstrap envelope — never a historical value.
        self.assertIn("preMotionOutstanding", body)
        self.assertIn("bootstrapUsable", body)

    def test_authorized_stage_defaults_to_h0(self):
        """The single stage switch must be fail-closed when no flag is given."""
        config = strip_comments(
            (SKETCH.parent / "flc_stage_config.h").read_text(encoding="utf-8")
        )
        match = re.search(
            r"#ifndef FLC_AUTHORIZED_STAGE\s*#define FLC_AUTHORIZED_STAGE\s+(\S+)",
            config,
        )
        self.assertIsNotNone(match, "default stage not found")
        self.assertEqual(match.group(1), "FLC_STAGE_H0_ESP32_ONLY")
        # The sketch must take the stage from that one place, not redefine it.
        self.assertIn("AUTHORIZED_STAGE = FLC_AUTHORIZED_STAGE", self.code)

    def test_bootstrap_defaults_to_denied(self):
        config = strip_comments(
            (SKETCH.parent / "flc_stage_config.h").read_text(encoding="utf-8")
        )
        match = re.search(
            r"#ifndef FLC_H3_BOOTSTRAP_APPROVED\s*#define FLC_H3_BOOTSTRAP_APPROVED\s+(\S+)",
            config,
        )
        self.assertIsNotNone(match)
        self.assertEqual(match.group(1), "0")

    def test_bootstrap_envelope_is_gentler_than_every_historical_value(self):
        """The first motion on the rebuilt robot must be the gentlest ever run."""
        config = strip_comments(
            (SKETCH.parent / "flc_stage_config.h").read_text(encoding="utf-8")
        )

        def value(name: str) -> int:
            m = re.search(rf"#define {name}\s+(\d+)", config)
            self.assertIsNotNone(m, f"{name} not found")
            return int(m.group(1))

        self.assertLess(value("FLC_BOOTSTRAP_TORQUE_LIMIT"), 300)   # provisioner
        self.assertLess(value("FLC_BOOTSTRAP_TORQUE_LIMIT"), 500)   # LF V25
        self.assertLess(value("FLC_BOOTSTRAP_GOAL_SPEED"), 160)     # LF V25

    def test_detector_uses_euclidean_modulo(self):
        """Regression guard: C++ '%' truncates, which breaks wrap-boundary math."""
        self.assertIn("flcMod", self.detector_code)
        match = re.search(r"int flcSignedTickDelta\(.*?\n\}", self.detector_code, re.DOTALL)
        self.assertIsNotNone(match)
        self.assertIn("flcMod", match.group(0))

    def test_detector_has_no_arduino_dependency(self):
        """It must stay host-compilable so the offline tests exercise real code."""
        for token in ("Arduino.h", "SCServo", "Serial.", "delay("):
            with self.subTest(token=token):
                self.assertNotIn(token, self.detector_code)


class TestFirmwareHostPolicySync(unittest.TestCase):
    """Constants that exist in both places must hold the same value."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.code = strip_comments(SKETCH.read_text(encoding="utf-8"))

    def _constant(self, name: str) -> int:
        match = re.search(rf"{name}\s*=\s*(-?\d+)", self.code)
        self.assertIsNotNone(match, f"{name} not found in firmware")
        return int(match.group(1))

    def test_servo_identity_constants_match_policy(self):
        pairs = {
            "EXPECTED_MODEL": "EXPECTED_MODEL",
            "EXPECTED_RESPONSE_STATUS": "EXPECTED_RESPONSE_STATUS",
            "EXPECTED_BAUD": "EXPECTED_BAUD_REGISTER",
            "EXPECTED_POSITION_OFFSET": "EXPECTED_POSITION_OFFSET",
        }
        for firmware_name, policy_name in pairs.items():
            with self.subTest(constant=firmware_name):
                self.assertEqual(
                    self._constant(firmware_name), policy.policy_value(policy_name).value
                )

    def test_guard_constants_match_policy(self):
        pairs = {
            "MON_THERMAL_LIMIT_C": "THERMAL_LIMIT_C",
            "MON_VOLTAGE_MIN": "VOLTAGE_MIN",
            "MON_VOLTAGE_MAX": "VOLTAGE_MAX",
            "MON_PERIOD_US": "MON_PERIOD_US",
            "WRITE_SETTLE_MS": "WRITE_SETTLE_MS",
        }
        for firmware_name, policy_name in pairs.items():
            with self.subTest(constant=firmware_name):
                self.assertEqual(
                    self._constant(firmware_name), policy.policy_value(policy_name).value
                )

    def test_encoder_domain_matches(self):
        self.assertEqual(self._constant("RAW_ELECTRICAL_CENTER"), 2048)

    def test_historical_constants_are_present_but_prefixed(self):
        """LF V25 numbers must be visibly quarantined behind a HIST_ prefix."""
        for name, expected in (
            ("HIST_TORQUE_LIMIT", 500),
            ("HIST_GOAL_SPEED", 160),
            ("HIST_HARD_CURRENT_ABORT_RAW", 200),
        ):
            with self.subTest(constant=name):
                self.assertEqual(self._constant(name), expected)

    def test_characterization_list_matches_policy(self):
        """The firmware and host must agree on the parameters AND their classes."""
        firmware = dict(re.findall(
            r'\{"([A-Z0-9_]+)",\s*(?:UNRESOLVED_U16|\d+),\s*(CLASS_[A-Z0-9_]+)', self.code
        ))
        expected = {
            v.name: {
                policy.ParameterClass.A_PRE_MOTION: "CLASS_A_PRE_MOTION",
                policy.ParameterClass.B_MEASURED_H3: "CLASS_B_MEASURED_H3",
                policy.ParameterClass.C_DERIVED: "CLASS_C_DERIVED",
                policy.ParameterClass.D_ACCEPTANCE: "CLASS_D_ACCEPTANCE",
            }[v.parameter_class]
            for v in policy.CHARACTERIZATION_REQUIRED
        }
        self.assertEqual(firmware, expected)

    def test_only_class_a_can_block_first_motion(self):
        """A tolerance that judges a measurement must not gate taking it."""
        match = re.search(r"static size_t preMotionOutstanding\(\).*?\n\}",
                          self.code, re.DOTALL)
        self.assertIsNotNone(match)
        self.assertIn("CLASS_A_PRE_MOTION", match.group(0))
        self.assertNotIn("CLASS_D_ACCEPTANCE", match.group(0))

    def test_bootstrap_constants_match_policy(self):
        config = strip_comments(
            (SKETCH.parent / "flc_stage_config.h").read_text(encoding="utf-8")
        )

        def value(name: str) -> int:
            m = re.search(rf"#define {name}\s+(\d+)", config)
            self.assertIsNotNone(m, f"{name} not found")
            return int(m.group(1))

        self.assertEqual(value("FLC_BOOTSTRAP_TORQUE_LIMIT"),
                         policy.BOOTSTRAP_ENVELOPE["torque_limit"])
        self.assertEqual(value("FLC_BOOTSTRAP_GOAL_SPEED"),
                         policy.BOOTSTRAP_ENVELOPE["goal_speed"])
        self.assertEqual(value("FLC_BOOTSTRAP_ACCELERATION"),
                         policy.BOOTSTRAP_ENVELOPE["acceleration"])
        self.assertEqual(value("FLC_ABSOLUTE_MAX_TORQUE_LIMIT"),
                         policy.ABSOLUTE_CEILINGS["torque_limit"])
        self.assertEqual(value("FLC_ABSOLUTE_MAX_TRAVEL_BUDGET_TICKS"),
                         policy.ABSOLUTE_CEILINGS["travel_budget_ticks"])

    def test_expected_leg_ids_match_policy(self):
        firmware_ids = set(
            int(m) for m in re.findall(r"\{\s*(\d\d),\s*\"[lr][fh]_", self.code)
        )
        self.assertEqual(firmware_ids, set(policy.EXPECTED_LEG_IDS))

    def test_firmware_joint_table_matches_allocation_units(self):
        rows = re.findall(r'\{\s*(\d\d),\s*"([a-z_]+)",\s*"([A-Z0-9]+)"', self.code)
        firmware_map = {int(bus_id): (joint, unit) for bus_id, joint, unit in rows}
        policy_map = {
            spec.bus_id: (spec.joint_name, spec.unit_label)
            for spec in policy.build_joint_specs()
        }
        self.assertEqual(firmware_map, policy_map)


class TestFrozenToolsUnchanged(unittest.TestCase):
    """The frozen QC / provisioning / survey files are evidence, not source."""

    def test_frozen_sha256sums_still_verify(self):
        manifest = FROZEN_DIR / "SHA256SUMS"
        self.assertTrue(manifest.is_file(), "frozen SHA256SUMS is missing")

        failures: list[str] = []
        for line in manifest.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            expected, _, name = line.partition("  ")
            target = (FROZEN_DIR / name.strip()).resolve()
            if not target.is_file():
                failures.append(f"missing {name}")
                continue
            actual = hashlib.sha256(target.read_bytes()).hexdigest()
            if actual != expected.strip():
                failures.append(f"changed {name}")
        self.assertEqual(failures, [], f"frozen evidence modified: {failures}")

    def test_frozen_tools_not_modified_on_this_branch(self):
        result = subprocess.run(
            ["git", "status", "--porcelain", "--", "05_Firmware/ST3215_Bench_Tools"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.strip(), "", "frozen bench tools were modified")


if __name__ == "__main__":
    unittest.main(verbosity=2)
