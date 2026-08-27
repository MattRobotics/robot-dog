"""Tests for the MATDOG calibration fail-closed gate.

Two things must hold:

1. The **current** stale configuration cannot authorize hardware, and every live entry
   point refuses before acquiring any hardware handle.
2. A **synthetic future calibrated** configuration still exercises the normal path, so
   the gate is not simply a permanent block.
"""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

import yaml

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_calibration_gate import (  # noqa: E402
    STATE_CALIBRATED,
    STATE_RESET_PENDING,
    CalibrationGateError,
    CalibrationResetError,
    calibration_state,
    hardware_motion_authorized,
    load_calibration,
    load_for_historical_inspection,
    refusal_reason,
    require_hardware_authorized,
    stale_banner,
)

REAL_CONFIG = CALIBRATION_DIR / "MATDOG_JOINT_CALIBRATION.yaml"


def _calibrated_config() -> dict:
    """Synthetic post-recalibration config: what a valid future state looks like."""
    data = copy.deepcopy(yaml.safe_load(REAL_CONFIG.read_text(encoding="utf-8")))
    data["calibration_reset"]["state"] = STATE_CALIBRATED
    data["calibration_reset"]["hardware_motion_authorized"] = True
    return data


class TestCurrentStateIsFailClosed(unittest.TestCase):
    """The config as committed today must never authorize hardware."""

    def setUp(self) -> None:
        self.data = load_calibration(REAL_CONFIG)

    def test_current_state_is_reset_pending(self) -> None:
        self.assertEqual(calibration_state(self.data), STATE_RESET_PENDING)

    def test_current_state_does_not_authorize_hardware(self) -> None:
        self.assertFalse(hardware_motion_authorized(self.data))

    def test_require_hardware_authorized_raises(self) -> None:
        with self.assertRaises(CalibrationResetError) as ctx:
            require_hardware_authorized("unit-test", REAL_CONFIG)
        msg = str(ctx.exception)
        self.assertIn("HARDWARE BLOCKED", msg)
        self.assertIn(STATE_RESET_PENDING, msg)

    def test_legacy_status_does_not_override_reset(self) -> None:
        """The stale legacy enum must not be able to re-authorize hardware."""
        legacy = self.data.get("robot", {}).get("calibration_status")
        self.assertEqual(legacy, "DIGITAL_ZERO_CALIBRATED_AND_VERIFIED")
        # Preserved for provenance...
        self.assertIsNotNone(legacy)
        # ...but powerless.
        self.assertFalse(hardware_motion_authorized(self.data))

    def test_legacy_digital_zero_status_does_not_override_reset(self) -> None:
        dz = self.data.get("digital_zero_calibration", {}).get("status")
        self.assertEqual(dz, "PASS_FINAL_EEPROM_READBACK")
        self.assertFalse(hardware_motion_authorized(self.data))

    def test_refusal_reason_explains_itself(self) -> None:
        reason = refusal_reason(self.data)
        self.assertTrue(reason)
        self.assertIn("PositionOffset=0", reason)

    def test_historical_values_preserved_for_provenance(self) -> None:
        """The reset must not have destroyed the historical record."""
        joints = self.data.get("joints", {})
        self.assertEqual(len(joints), 12)
        self.assertIn("zero_encoder_visual", joints["lf_hip_joint"])


class TestTamperResistance(unittest.TestCase):
    """Partial or malformed authorization must not open the gate."""

    def test_missing_calibration_reset_block_is_refused(self) -> None:
        data = load_calibration(REAL_CONFIG)
        data.pop("calibration_reset")
        self.assertFalse(hardware_motion_authorized(data))
        self.assertIn("predates", refusal_reason(data))

    def test_authorized_flag_alone_is_not_enough(self) -> None:
        """hardware_motion_authorized=true with a reset state must still refuse."""
        data = load_calibration(REAL_CONFIG)
        data["calibration_reset"]["hardware_motion_authorized"] = True
        self.assertEqual(calibration_state(data), STATE_RESET_PENDING)
        self.assertFalse(hardware_motion_authorized(data))

    def test_calibrated_state_alone_is_not_enough(self) -> None:
        """A calibrated state without the explicit flag must still refuse."""
        data = load_calibration(REAL_CONFIG)
        data["calibration_reset"]["state"] = STATE_CALIBRATED
        data["calibration_reset"]["hardware_motion_authorized"] = False
        self.assertFalse(hardware_motion_authorized(data))

    def test_truthy_non_true_flag_is_refused(self) -> None:
        for value in ("true", "yes", 1, [1]):
            data = _calibrated_config()
            data["calibration_reset"]["hardware_motion_authorized"] = value
            with self.subTest(value=value):
                self.assertFalse(hardware_motion_authorized(data))

    def test_missing_config_file_raises(self) -> None:
        with self.assertRaises(CalibrationGateError):
            load_calibration(CALIBRATION_DIR / "does_not_exist.yaml")


class TestHistoricalInspection(unittest.TestCase):
    """Read-only inspection is allowed, but must be labelled stale."""

    def test_inspection_reports_stale(self) -> None:
        data, is_stale = load_for_historical_inspection(REAL_CONFIG)
        self.assertTrue(is_stale)
        self.assertEqual(len(data.get("joints", {})), 12)

    def test_stale_banner_present_when_stale(self) -> None:
        data = load_calibration(REAL_CONFIG)
        self.assertIn("STALE CALIBRATION", stale_banner(data))

    def test_no_banner_when_authorized(self) -> None:
        self.assertEqual(stale_banner(_calibrated_config()), "")


class TestFutureCalibratedStatePasses(unittest.TestCase):
    """The gate must not be a permanent block — the valid path must still work."""

    def setUp(self) -> None:
        self.data = _calibrated_config()

    def test_synthetic_calibrated_state_authorizes(self) -> None:
        self.assertEqual(calibration_state(self.data), STATE_CALIBRATED)
        self.assertTrue(hardware_motion_authorized(self.data))

    def test_require_hardware_authorized_returns_config(self) -> None:
        returned = require_hardware_authorized("unit-test", data=self.data)
        self.assertEqual(calibration_state(returned), STATE_CALIBRATED)

    def test_no_refusal_reason_when_authorized(self) -> None:
        self.assertEqual(refusal_reason(self.data), "")


class TestLiveConsumersAreGated(unittest.TestCase):
    """Every live/hardware entry point must call the gate before acquiring hardware."""

    CORE = CALIBRATION_DIR.parent
    GATED = {
        "hardware/matdog_c5_supervised_first_stand_executor.py": "async def main_async(args):",
        "calibration/matdog_live_joint_monitor.py": "async def main_async(args):",
        "calibration/matdog_visual_zero_pose_probe.py": "async def main_async(args):",
        "kinematics/matdog_leg_fk_live.py": "async def main_async(args) -> None:",
        "calibration/matdog_endstop_station_readonly_watch.py": "async def main_async(args: argparse.Namespace) -> int:",
    }

    def test_every_live_consumer_imports_the_gate(self) -> None:
        for rel in self.GATED:
            with self.subTest(module=rel):
                src = (self.CORE / rel).read_text(encoding="utf-8")
                self.assertIn("require_hardware_authorized", src)

    def test_gate_is_first_statement_of_entry_point(self) -> None:
        """The guard must run before anything else in the async entry point."""
        for rel, anchor in self.GATED.items():
            with self.subTest(module=rel):
                lines = (self.CORE / rel).read_text(encoding="utf-8").splitlines()
                idx = next(i for i, l in enumerate(lines) if l.startswith(anchor))
                body = [l for l in lines[idx + 1 :] if l.strip() and not l.strip().startswith("#")]
                self.assertIn("require_hardware_authorized(", body[0])

    def test_gate_precedes_station_client_acquisition(self) -> None:
        """No entry point may acquire a Station client before calling the gate."""
        for rel, anchor in self.GATED.items():
            with self.subTest(module=rel):
                lines = (self.CORE / rel).read_text(encoding="utf-8").splitlines()
                start = next(i for i, l in enumerate(lines) if l.startswith(anchor))
                guard = next(
                    i for i, l in enumerate(lines[start:], start)
                    if "require_hardware_authorized(" in l and '"' in l
                )
                after = [
                    i for i, l in enumerate(lines[start:], start)
                    if "await new_station_client(" in l
                ]
                for acq in after:
                    self.assertLess(guard, acq)


if __name__ == "__main__":
    unittest.main()
