"""Policy, provenance and joint-spec tests for Full Leg Calibrator V1.

The safety property under test is narrow and important: **no number measured on
the previous installation may authorize motion on the current one**, and no
safety-critical parameter may be silently defaulted.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

import matdog_full_leg_calibrator_policy as policy  # noqa: E402


class TestProvenanceGating(unittest.TestCase):
    def test_no_historical_value_may_authorize_motion(self):
        """LF V25 and bench-centering numbers are recorded but never authorizing."""
        for value in policy.HISTORICAL_CANDIDATES:
            with self.subTest(name=value.name):
                self.assertFalse(value.may_authorize_motion)
                self.assertIs(value.provenance, policy.Provenance.HISTORICAL_CANDIDATE_ONLY)

    def test_characterization_required_values_are_unresolved(self):
        for value in policy.CHARACTERIZATION_REQUIRED:
            with self.subTest(name=value.name):
                self.assertIsNone(value.value)
                self.assertFalse(value.resolved)
                self.assertFalse(value.may_authorize_motion)

    def test_every_policy_value_carries_evidence(self):
        for value in policy.ALL_POLICY_VALUES:
            with self.subTest(name=value.name):
                self.assertTrue(value.evidence.strip(), "constant without provenance")

    def test_policy_value_names_are_unique(self):
        names = [v.name for v in policy.ALL_POLICY_VALUES]
        self.assertEqual(len(names), len(set(names)))

    def test_lf_v25_torque_limit_records_the_heterogeneous_fleet_evidence(self):
        """The 500/1000 split is the concrete reason old numbers are not an oracle."""
        value = policy.policy_value("LF_V25_TORQUE_LIMIT")
        self.assertEqual(value.value, 500)
        self.assertIn("13 and 23", value.evidence)
        self.assertIn("1000", value.evidence)

    def test_provisioner_centering_torque_is_not_a_contact_constant(self):
        value = policy.policy_value("PROVISIONER_CENTER_TORQUE_LIMIT")
        self.assertIs(value.provenance, policy.Provenance.HISTORICAL_CANDIDATE_ONLY)
        self.assertIn("free-shaft", value.evidence.lower())


class TestMotionGate(unittest.TestCase):
    def test_motion_is_blocked_at_the_shipped_stage(self):
        self.assertFalse(policy.motion_authorized())
        self.assertTrue(policy.motion_blockers())

    def test_require_motion_authorized_raises_with_reasons(self):
        with self.assertRaises(policy.CalibrationPolicyError) as ctx:
            policy.require_motion_authorized("CALIBRATE_JOINT")
        message = str(ctx.exception)
        self.assertIn("MOTION BLOCKED", message)
        self.assertIn("CONTACT_TORQUE_LIMIT", message)
        self.assertIn("No LF V25 numeric result", message)

    def test_stage_alone_does_not_unlock_motion(self):
        """Even at H7, unvalidated contact parameters must still block."""
        self.assertFalse(policy.motion_authorized(policy.HardwareStage.H7_FREEZE))
        with self.assertRaises(policy.CalibrationPolicyError):
            policy.require_motion_authorized("CALIBRATE_ALL", policy.HardwareStage.H7_FREEZE)

    def test_shipped_stage_is_h0(self):
        self.assertIs(policy.AUTHORIZED_STAGE, policy.HardwareStage.H0_ESP32_ONLY)

    def test_read_only_stages_are_ordered_below_motion(self):
        self.assertLess(
            policy.HardwareStage.H2_MANUAL_Q0.value,
            policy.HardwareStage.H3_JOINT_CHARACTERIZE.value,
        )
        self.assertLess(
            policy.HardwareStage.H3_JOINT_CHARACTERIZE.value,
            policy.HardwareStage.H4_JOINT_CALIBRATE.value,
        )


class TestJointSpecs(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.specs = policy.build_joint_specs()

    def test_twelve_leg_joints_with_expected_ids(self):
        self.assertEqual(len(self.specs), 12)
        self.assertEqual(
            sorted(s.bus_id for s in self.specs), sorted(policy.EXPECTED_LEG_IDS)
        )

    def test_allocation_matches_the_documented_physical_units(self):
        """Bus id -> physical unit must come from the allocation, not from memory."""
        expected = {
            13: ("lf_hip_joint", "M22"), 12: ("lf_upper_leg_joint", "ELR01"),
            11: ("lf_lower_leg_joint", "M33"), 23: ("rf_hip_joint", "NEW01"),
            22: ("rf_upper_leg_joint", "ELR03"), 21: ("rf_lower_leg_joint", "NEW03"),
            33: ("rh_hip_joint", "NEW06"), 32: ("rh_upper_leg_joint", "ELR02"),
            31: ("rh_lower_leg_joint", "NEW05"), 43: ("lh_hip_joint", "M43"),
            42: ("lh_upper_leg_joint", "M42"), 41: ("lh_lower_leg_joint", "M41"),
        }
        actual = {s.bus_id: (s.joint_name, s.unit_label) for s in self.specs}
        self.assertEqual(actual, expected)

    def test_direction_is_unmeasured_on_this_installation(self):
        """Stale pre-reassembly directions must not leak in as current truth."""
        for spec in self.specs:
            with self.subTest(joint=spec.joint_name):
                self.assertIsNone(spec.direction)
                self.assertFalse(spec.direction_known)

    def test_hip_endpoints_are_not_assumed_symmetric(self):
        """Derived from the endpoint profile: hips genuinely differ per leg."""
        hips = {s.leg: s for s in self.specs if s.kind == "HIP"}
        self.assertAlmostEqual(hips["LF"].geometric_contact_min_rad, -0.803055986689, places=9)
        self.assertAlmostEqual(hips["LF"].geometric_contact_max_rad, 0.789284248, places=9)
        self.assertAlmostEqual(hips["RF"].geometric_contact_min_rad, -0.789284248, places=9)
        self.assertAlmostEqual(hips["RF"].geometric_contact_max_rad, 0.803055986689, places=9)
        self.assertAlmostEqual(hips["RH"].geometric_contact_min_rad, -0.788125240, places=9)
        self.assertAlmostEqual(hips["LH"].geometric_contact_max_rad, 0.788125240, places=9)
        # No leg's hip is sign-symmetric about zero.
        for leg, spec in hips.items():
            with self.subTest(leg=leg):
                self.assertNotAlmostEqual(
                    spec.geometric_contact_min_rad, -spec.geometric_contact_max_rad, places=6
                )

    def test_hip_endpoints_differ_between_front_and_hind_on_the_same_side(self):
        """Guards against a FRONT/HIND symmetry shortcut."""
        hips = {s.leg: s for s in self.specs if s.kind == "HIP"}
        self.assertNotAlmostEqual(
            hips["RF"].geometric_contact_min_rad,
            hips["RH"].geometric_contact_min_rad,
            places=6,
        )

    def test_geometric_contacts_bracket_or_track_declared_limits(self):
        for spec in self.specs:
            with self.subTest(joint=spec.joint_name):
                self.assertLess(spec.geometric_contact_min_rad, spec.geometric_contact_max_rad)
                self.assertLess(spec.declared_min_rad, spec.declared_max_rad)

    def test_head_ids_are_not_part_of_a_leg_session(self):
        for head_id in policy.HEAD_IDS:
            self.assertNotIn(head_id, policy.EXPECTED_LEG_IDS)


class TestCensusExpectation(unittest.TestCase):
    def test_head_absence_is_expected_not_a_failure(self):
        expectation = policy.CensusExpectation()
        self.assertTrue(expectation.head_absence_expected)
        self.assertEqual(expectation.expected_leg_ids, policy.EXPECTED_LEG_IDS)

    def test_forbidden_operations_cover_the_permanent_prohibitions(self):
        forbidden = policy.CensusExpectation().forbidden_operations
        for operation in (
            "CalibrationOfs", "one_key_middle", "factory_reset", "broadcast_write",
            "PositionOffset_rewrite", "servo_id_rewrite", "persistent_profile_rewrite",
            "any_EEPROM_write",
        ):
            self.assertIn(operation, forbidden)


class TestAllocationValidation(unittest.TestCase):
    def test_allocation_loads_all_twelve_leg_units(self):
        allocation = policy.load_leg_allocation()
        self.assertEqual(sorted(allocation), sorted(policy.EXPECTED_LEG_IDS))

    def test_allocation_rejects_nonzero_position_offset(self):
        import tempfile
        import yaml

        data = {
            "profile": "MATDOG_C018_V1",
            "units": [
                {"unit": "X", "joint": "LF_HIP", "bus_id": 13, "position_offset": 7}
            ],
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            yaml.safe_dump(data, handle)
            path = Path(handle.name)
        self.addCleanup(path.unlink)
        with self.assertRaises(policy.CalibrationPolicyError) as ctx:
            policy.load_leg_allocation(path)
        self.assertIn("position_offset", str(ctx.exception))

    def test_allocation_rejects_missing_leg_ids(self):
        import tempfile
        import yaml

        data = {
            "profile": "MATDOG_C018_V1",
            "units": [
                {"unit": "X", "joint": "LF_HIP", "bus_id": 13, "position_offset": 0}
            ],
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            yaml.safe_dump(data, handle)
            path = Path(handle.name)
        self.addCleanup(path.unlink)
        with self.assertRaises(policy.CalibrationPolicyError) as ctx:
            policy.load_leg_allocation(path)
        self.assertIn("missing leg bus ids", str(ctx.exception))


class TestProtocolLabelling(unittest.TestCase):
    def test_protocol_is_labelled_calibrator_local(self):
        """The USB CDC command set here is not the final MATDOG runtime protocol."""
        self.assertEqual(policy.PROTOCOL_ID, "FLC1")
        self.assertIn("NOT_FINAL_RUNTIME_PROTOCOL", policy.PROTOCOL_SCOPE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
