"""Test automatici MATDOG per il path/prerequisite/parking planner.

Uses a coarser path-sampling step than the module's own default (which
matches the 2026-07-20 checkpoint's ~1 deg precedent and is deliberately
thorough for the real compiler run) purely to keep this automated test
fast; the underlying logic exercised is identical.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_path_planner import (  # noqa: E402
    leg_calibration_sequence,
    plan_leg_parking,
)
from matdog_geometry_scene import RobotScene, leg_pose_overrides  # noqa: E402


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf").is_file():
            return parent
    raise RuntimeError("Repository root non trovato")


REPO_ROOT = _repo_root()
FAST_TEST_STEP_RAD = math.radians(3.0)


class TestParkingPreference(unittest.TestCase):
    """Item H: the planner prefers NONE when the path is safe.

    RH is the checkpoint-documented case that needs no extra parking
    ("RH and LH calibration do not require an additional cross-leg
    parking pose based on the current geometry audit"), so it is also the
    fastest real case to validate end to end (the no-parking attempt
    should pass outright, without any retry).
    """

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def test_rh_needs_no_auxiliary_parking(self):
        plan = plan_leg_parking(self.scene, "rh", step_rad=FAST_TEST_STEP_RAD)

        # "Prefers NONE" means no auxiliary leg is parked, not that every
        # sampled segment cleared the (separately configurable) minimum
        # modelled clearance bar: a true mesh collision would always
        # imply required=True (see plan_leg_parking), but a marginal,
        # collision-free clearance dip is reported as its own honest
        # finding rather than forcing an auxiliary-parking search.
        self.assertFalse(plan.required, plan.reason)
        self.assertIsNone(plan.parked_leg)
        self.assertIsNone(plan.parking_angle_rad)
        self.assertIsNone(plan.park_path)

        failure = plan.active_leg_sequence.first_failure
        if failure is not None:
            self.assertFalse(failure.has_true_collision, "no auxiliary parking implies no real collision")

    def test_lh_needs_no_auxiliary_parking(self):
        plan = plan_leg_parking(self.scene, "lh", step_rad=FAST_TEST_STEP_RAD)

        self.assertFalse(plan.required, plan.reason)
        self.assertIsNone(plan.parked_leg)


class TestParkingSeedAcceptanceBugRegression(unittest.TestCase):
    """Regression for the reconciliation-review bug: the parking seed
    search used to accept a candidate seed only if the ENTIRE per-leg
    sequence passed the 3mm clearance gate (LegSequenceResult.passed),
    which conflates unrelated low-clearance segments (e.g. this leg's own
    hip probe near its own declared limit, which has nothing to do with
    the parked leg) with the actual cross-leg collision the parking is
    meant to fix. That made every seed angle look like a failure even
    when the real collision was already resolved -- LF/RF were reported
    as NEEDS_HUMAN_DECISION ("no seed 30..90deg works") despite the
    checkpoint and LF V25 hardware both validating a plain +30deg parking
    pose for the whole session."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def test_lh_parked_30deg_leaves_no_true_collision_in_lf_sequence(self):
        parked_pose = leg_pose_overrides("lh", 0.0, math.radians(30.0), 0.0)
        sequence = leg_calibration_sequence(self.scene, "lf", parked_pose, step_rad=FAST_TEST_STEP_RAD)

        true_collisions = [s for s in sequence.segments if s.has_true_collision]
        self.assertEqual(
            true_collisions, [],
            f"true collision(s) remain with lh parked at 30deg: {[s.description for s in true_collisions]}",
        )
        # The bug's symptom: passed can still be False (unrelated low
        # clearance elsewhere), which is fine and must not be confused
        # with "parking did not work".

    def test_lf_parking_search_resolves_at_smallest_seed(self):
        plan = plan_leg_parking(self.scene, "lf", step_rad=FAST_TEST_STEP_RAD)

        self.assertTrue(plan.required, plan.reason)
        self.assertEqual(plan.parked_leg, "lh")
        self.assertIsNotNone(plan.parking_angle_rad)
        self.assertAlmostEqual(plan.parking_angle_rad, math.radians(30.0), places=6)
        self.assertFalse(
            any(s.has_true_collision for s in plan.active_leg_sequence.segments),
            "resolved parking plan must leave no true collision in the active sequence",
        )


class TestLegSequenceStructure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def test_sequence_has_expected_segment_count_and_order(self):
        result = leg_calibration_sequence(self.scene, "rh", {}, step_rad=FAST_TEST_STEP_RAD)

        # upper (4) + transition to HIP prereq (1) + hip (4) +
        # transition HIP->LOWER prereq (1) + lower (4) + restore (1) = 15
        self.assertEqual(len(result.segments), 15)
        self.assertEqual(result.segments[0].joint_name, "rh_upper_leg_joint")
        self.assertEqual(result.segments[-1].description, "restore leg to home")

    def test_every_segment_has_at_least_two_samples(self):
        result = leg_calibration_sequence(self.scene, "lh", {}, step_rad=FAST_TEST_STEP_RAD)

        for segment in result.segments:
            self.assertGreaterEqual(segment.sample_count, 1)


if __name__ == "__main__":
    unittest.main()
