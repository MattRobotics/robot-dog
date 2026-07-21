import ast
from pathlib import Path
import sys
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALIBRATION_DIR))

import matdog_endstop_contact_detector as detector
import matdog_mechanical_endstop_calibration as planner


class ContactDetectorTests(unittest.TestCase):
    def setUp(self):
        self.policy = detector.ContactPolicy(
            baseline_window=5,
            minimum_baseline_samples=4,
            moving_velocity_floor_tick_s=15.0,
            stall_velocity_ceiling_tick_s=3.0,
            current_rise_raw=8,
            minimum_travel_before_contact_tick=10,
            suspicion_consecutive_samples=2,
            confirmation_consecutive_samples=3,
            max_sample_gap_s=0.25,
            timeout_s=5.0,
            reverse_tolerance_tick=2,
        )

    def _sample(
        self,
        t_ms,
        tick,
        current,
        status=0,
    ):
        return detector.TelemetrySample(
            monotonic_stamp_ns=t_ms * 1_000_000,
            present_tick=tick,
            current_raw=current,
            error_status=status,
        )

    def test_state_contract_matches_offline_planner(self):
        self.assertEqual(
            [state.value for state in detector.ContactState],
            [state.value for state in planner.ContactState],
        )

    def test_free_motion_builds_adaptive_baseline(self):
        machine = detector.ContactDetector(
            start_tick=1000,
            approach_direction=1,
            max_travel_tick=100,
            policy=self.policy,
        )

        observations = [
            machine.ingest(self._sample(0, 1000, 2)),
            machine.ingest(self._sample(100, 1005, 2)),
            machine.ingest(self._sample(200, 1010, 3)),
            machine.ingest(self._sample(300, 1015, 2)),
            machine.ingest(self._sample(400, 1020, 3)),
        ]

        self.assertTrue(all(
            item.state == detector.ContactState.FREE_MOTION
            for item in observations
        ))
        self.assertEqual(
            observations[-1].baseline_current_raw,
            2.5,
        )

    def test_persistent_stall_and_current_rise_confirms_contact(self):
        machine = detector.ContactDetector(
            start_tick=1000,
            approach_direction=1,
            max_travel_tick=100,
            policy=self.policy,
        )

        samples = [
            self._sample(0, 1000, 2),
            self._sample(100, 1005, 2),
            self._sample(200, 1010, 2),
            self._sample(300, 1015, 3),
            self._sample(400, 1020, 2),
            self._sample(500, 1020, 14),
            self._sample(600, 1020, 15),
            self._sample(700, 1020, 16),
        ]

        observations = [machine.ingest(sample) for sample in samples]

        self.assertEqual(
            observations[-2].state,
            detector.ContactState.CONTACT_SUSPECTED,
        )
        self.assertEqual(
            observations[-1].state,
            detector.ContactState.CONTACT_CONFIRMED,
        )
        self.assertEqual(
            observations[-1].reason,
            "STALL_AND_CURRENT_RISE_PERSISTENT",
        )

    def test_servo_status_error_is_hard_abort(self):
        machine = detector.ContactDetector(
            start_tick=1000,
            approach_direction=1,
            max_travel_tick=100,
            policy=self.policy,
        )

        observation = machine.ingest(
            self._sample(0, 1000, 2, status=4)
        )

        self.assertEqual(
            observation.state,
            detector.ContactState.HARD_ABORT,
        )
        self.assertEqual(
            observation.reason,
            "SERVO_STATUS_ERROR_0x04",
        )

    def test_telemetry_gap_is_hard_abort(self):
        machine = detector.ContactDetector(
            start_tick=1000,
            approach_direction=1,
            max_travel_tick=100,
            policy=self.policy,
        )

        machine.ingest(self._sample(0, 1000, 2))
        observation = machine.ingest(
            self._sample(400, 1005, 2)
        )

        self.assertEqual(
            observation.state,
            detector.ContactState.HARD_ABORT,
        )
        self.assertEqual(
            observation.reason,
            "TELEMETRY_GAP_EXCEEDED",
        )

    def test_model_travel_guard_is_hard_abort(self):
        machine = detector.ContactDetector(
            start_tick=1000,
            approach_direction=1,
            max_travel_tick=20,
            policy=self.policy,
        )

        observation = machine.ingest(
            self._sample(0, 1021, 2)
        )

        self.assertEqual(
            observation.state,
            detector.ContactState.HARD_ABORT,
        )
        self.assertEqual(
            observation.reason,
            "MODEL_TRAVEL_GUARD_EXCEEDED",
        )

    def test_recovery_and_repeatability_helpers(self):
        self.assertTrue(detector.recovery_verified(
            contact_tick=1200,
            recovered_tick=1185,
            approach_direction=1,
            minimum_backoff_tick=12,
        ))
        self.assertFalse(detector.recovery_verified(
            contact_tick=1200,
            recovered_tick=1195,
            approach_direction=1,
            minimum_backoff_tick=12,
        ))
        self.assertTrue(detector.contact_is_repeatable(
            first_contact_tick=4092,
            second_contact_tick=3,
            tolerance_tick=8,
        ))
        self.assertEqual(
            detector.contact_repeatability_spread_tick(4092, 3),
            7,
        )

    def test_module_has_no_hardware_command_capability(self):
        module_path = (
            CALIBRATION_DIR / "matdog_endstop_contact_detector.py"
        )
        tree = ast.parse(module_path.read_text(encoding="utf-8"))

        forbidden_import_roots = {
            "serial",
            "pyserial",
            "commands",
            "mirror",
            "state",
        }
        forbidden_calls = {
            "send_commands",
            "send_motor_commands",
            "send_goal",
            "set_torque",
            "new_station_client",
        }

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotIn(
                        alias.name.split(".")[0],
                        forbidden_import_roots,
                    )

            if isinstance(node, ast.ImportFrom):
                root = (node.module or "").split(".")[0]
                self.assertNotIn(root, forbidden_import_roots)
                self.assertFalse(
                    (node.module or "").startswith("software.station")
                )

            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Name):
                    self.assertNotIn(node.func.id, forbidden_calls)

        source = module_path.read_text(encoding="utf-8")
        self.assertNotIn("--execute", source)


if __name__ == "__main__":
    unittest.main()
