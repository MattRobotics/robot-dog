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
        cls.detector_code = strip_comments(DETECTOR.read_text(encoding="utf-8"))

    def test_sketch_and_detector_exist(self):
        self.assertTrue(SKETCH.is_file())
        self.assertTrue(DETECTOR.is_file())

    def test_no_eeprom_unlock_or_lock_call(self):
        for forbidden in ("unLockEprom", "LockEprom"):
            with self.subTest(call=forbidden):
                self.assertNotIn(forbidden, self.code)

    def test_calibration_ofs_is_never_called(self):
        self.assertNotIn("CalibrationOfs(", self.code)

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
        occurrences = re.findall(r"st\.WritePosEx\s*\(", self.code)
        self.assertEqual(len(occurrences), 1, "more than one GoalPosition writer")

    def test_no_alternative_motion_primitives(self):
        for forbidden in ("RegWritePosEx", "SyncWritePosEx", "WheelMode", "WriteSpe"):
            with self.subTest(call=forbidden):
                self.assertNotIn(forbidden, self.code)

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

    def test_only_the_scservo_and_arduino_headers_are_included(self):
        includes = set(re.findall(r'#include\s+[<"]([^>"]+)[>"]', self.code))
        self.assertEqual(includes, {"Arduino.h", "SCServo.h", "flc_contact_detector.h"})

    def test_motion_primitive_enforces_unsigned_domain(self):
        match = re.search(r"bool flcWritePosEx\(.*?\n\}", self.code, re.DOTALL)
        self.assertIsNotNone(match)
        body = match.group(0)
        self.assertIn("position < 0", body)
        self.assertIn("ENCODER_MAX", body)
        self.assertIn("STAGE_MOTION", body)
        self.assertIn("AUTHORIZED_STAGE", body)
        self.assertIn("characterizationOutstanding", body)

    def test_authorized_stage_is_h0(self):
        match = re.search(
            r"AUTHORIZED_STAGE\s*=\s*(H\d_[A-Z0-9_]+)", self.code
        )
        self.assertIsNotNone(match)
        self.assertEqual(match.group(1), "H0_ESP32_ONLY")

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
        """The firmware and host must agree on what is still unvalidated."""
        firmware_names = set(re.findall(r'\{"([A-Z0-9_]+)", UNRESOLVED_U16', self.code))
        policy_names = {v.name for v in policy.CHARACTERIZATION_REQUIRED}
        self.assertEqual(firmware_names, policy_names)

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
