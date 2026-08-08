"""MATDOG Geometry Compiler — regression oracles from the 2026-07-20 checkpoint.

06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
documents six offline geometric findings. These are historical evidence,
not the new plan (see MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
section 5: "search seeds", not permanent constants) -- but the Geometry
Compiler's collision engine must still reproduce them against the current
canonical URDF/meshes, since they were the only previously-validated
ground truth available before this compiler existed.

Every test here calls the real scene/collision-kernel logic (RobotScene,
matdog_geometry_mesh_kernel), not string checks against source files.

These are the slowest tests in the calibration suite (full continuous-path
sweeps over a real 105k-triangle body mesh); expect low tens of seconds
total, not milliseconds.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_scene import RobotScene, full_pose, leg_pose_overrides  # noqa: E402


HIP_PREREQUISITE_UPPER_RAD = math.radians(50.0)
LOWER_PREREQUISITE_UPPER_RAD = math.radians(90.0)
REAR_PARKING_UPPER_RAD = math.radians(30.0)

HIP_MIN_RAD = math.radians(-45.0)
HIP_MAX_RAD = math.radians(45.0)
LOWER_MIN_RAD = math.radians(-92.0)
LOWER_MAX_RAD = math.radians(37.5)


def _sweep_collision_free(scene: RobotScene, poses: list[dict[str, float]]) -> tuple[bool, dict | None]:
    for pose in poses:
        collide, pair = scene.is_colliding_at_pose(pose)

        if collide:
            return False, pose

    return True, None


def _hip_path_poses(leg: str, upper_rad: float, step_deg: float = 3.0) -> list[dict[str, float]]:
    """0 -> -45 -> 0 -> +45 -> 0 at the given upper prerequisite, matching
    the checkpoint's HIP-path structure. Sampled coarser than the
    checkpoint's original ~1 deg for automated-test runtime; the compiler
    run itself (not this unit test) is the exhaustive 1 deg validation."""
    degrees = (
        list(_frange(0.0, -45.0, -step_deg)) + [-45.0]
        + list(_frange(-45.0, 45.0, step_deg)) + [45.0]
        + list(_frange(45.0, 0.0, -step_deg)) + [0.0]
    )
    return [
        full_pose(leg_pose_overrides(leg, math.radians(deg), upper_rad, 0.0))
        for deg in degrees
    ]


def _lower_path_poses(leg: str, upper_rad: float, step_deg: float = 4.0) -> list[dict[str, float]]:
    degrees = (
        list(_frange(0.0, -92.0, -step_deg)) + [-92.0]
        + list(_frange(-92.0, 37.5, step_deg)) + [37.5]
        + list(_frange(37.5, 0.0, -step_deg)) + [0.0]
    )
    return [
        full_pose(leg_pose_overrides(leg, 0.0, upper_rad, math.radians(deg)))
        for deg in degrees
    ]


def _frange(start: float, stop: float, step: float):
    value = start
    if step > 0:
        while value < stop:
            yield value
            value += step
    else:
        while value > stop:
            yield value
            value += step


class TestCheckpointRegression(unittest.TestCase):
    """Regression oracles A from the Phase 1 test contract."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene()

    def test_hip_prerequisite_upper_50_collision_free_all_legs(self):
        """Checkpoint: 'HIP prerequisite upper=+50deg' path 0->-45->0->+45->0
        passed for LF, RF, RH, LH."""
        for leg in ("lf", "rf", "rh", "lh"):
            with self.subTest(leg=leg):
                ok, failing_pose = _sweep_collision_free(
                    self.scene, _hip_path_poses(leg, HIP_PREREQUISITE_UPPER_RAD)
                )
                self.assertTrue(ok, f"{leg}: collision found at {failing_pose}")

    def test_anterior_hip_extreme_with_upper_90_collides(self):
        """Checkpoint: at the exact URDF hip extremes with upper=+90deg,
        LF hip=+45deg collides base_link<->lf_lower_leg_link and RF
        hip=-45deg collides base_link<->rf_lower_leg_link (real triangle
        surface contact, not a convex-hull false positive)."""
        pose_lf = full_pose(leg_pose_overrides("lf", HIP_MAX_RAD, LOWER_PREREQUISITE_UPPER_RAD, 0.0))
        collide_lf, pair_lf = self.scene.is_colliding_at_pose(pose_lf)
        self.assertTrue(collide_lf, "LF hip=+45deg, upper=+90deg expected to collide")
        self.assertEqual(set(pair_lf), {"base_link", "lf_lower_leg_link"})

        pose_rf = full_pose(leg_pose_overrides("rf", HIP_MIN_RAD, LOWER_PREREQUISITE_UPPER_RAD, 0.0))
        collide_rf, pair_rf = self.scene.is_colliding_at_pose(pose_rf)
        self.assertTrue(collide_rf, "RF hip=-45deg, upper=+90deg expected to collide")
        self.assertEqual(set(pair_rf), {"base_link", "rf_lower_leg_link"})

    def test_anterior_hip_opposite_extreme_with_upper_90_is_clear(self):
        """Checkpoint: 'The opposite anterior extremes were clear: LF
        hip -45deg: clear; RF hip +45deg: clear.'"""
        pose_lf = full_pose(leg_pose_overrides("lf", HIP_MIN_RAD, LOWER_PREREQUISITE_UPPER_RAD, 0.0))
        collide_lf, pair_lf = self.scene.is_colliding_at_pose(pose_lf)
        self.assertFalse(collide_lf, f"LF hip=-45deg, upper=+90deg expected clear, found {pair_lf}")

        pose_rf = full_pose(leg_pose_overrides("rf", HIP_MAX_RAD, LOWER_PREREQUISITE_UPPER_RAD, 0.0))
        collide_rf, pair_rf = self.scene.is_colliding_at_pose(pose_rf)
        self.assertFalse(collide_rf, f"RF hip=+45deg, upper=+90deg expected clear, found {pair_rf}")

    def test_rear_hip_extremes_with_upper_90_are_clear(self):
        """Checkpoint: 'RH and LH were clear at both hip extremes with
        upper +90deg.'"""
        for leg in ("rh", "lh"):
            for hip_rad in (HIP_MIN_RAD, HIP_MAX_RAD):
                with self.subTest(leg=leg, hip_rad=hip_rad):
                    pose = full_pose(leg_pose_overrides(leg, hip_rad, LOWER_PREREQUISITE_UPPER_RAD, 0.0))
                    collide, pair = self.scene.is_colliding_at_pose(pose)
                    self.assertFalse(collide, f"{leg} hip={hip_rad:.3f} expected clear, found {pair}")

    def test_lower_prerequisite_upper_90_collision_free_all_legs(self):
        """Checkpoint: LOWER path 0->-92->0->+37.5->0 at upper=+90deg
        passed for LF, RF, RH, LH (the hip_link<->lower_leg_link overlap
        near -92deg was a convex-hull false positive; real STL surfaces
        were separated)."""
        for leg in ("lf", "rf", "rh", "lh"):
            with self.subTest(leg=leg):
                ok, failing_pose = _sweep_collision_free(
                    self.scene, _lower_path_poses(leg, LOWER_PREREQUISITE_UPPER_RAD)
                )
                self.assertTrue(ok, f"{leg}: collision found at {failing_pose}")

    def test_front_cross_leg_transition_collides_with_ipsilateral_rear_at_home(self):
        """Checkpoint: the direct LF/RF upper +50->+90deg transition (hip=0,
        lower=0) collides against the ipsilateral rear foot/lower with the
        rear leg at home, roughly upper in [74, 87] deg."""
        for leg, note in (("lf", "lf vs lh"), ("rf", "rf vs rh")):
            with self.subTest(leg=leg, note=note):
                found_collision = False

                for upper_deg in range(50, 91, 2):
                    pose = full_pose(leg_pose_overrides(leg, 0.0, math.radians(upper_deg), 0.0))
                    collide, pair = self.scene.is_colliding_at_pose(pose)

                    if collide:
                        found_collision = True
                        break

                self.assertTrue(
                    found_collision,
                    f"{leg}: expected a cross-leg collision somewhere in upper [50,90]deg "
                    "with ipsilateral rear leg at home",
                )

    def test_front_transition_clear_with_ipsilateral_rear_parked(self):
        """Checkpoint: with the ipsilateral rear leg parked at upper=+30deg,
        the complete anterior upper +50->+90deg transition passes."""
        rear_by_front = {"lf": "lh", "rf": "rh"}

        for front_leg, rear_leg in rear_by_front.items():
            with self.subTest(front_leg=front_leg, rear_leg=rear_leg):
                for upper_deg in range(50, 91, 5):
                    overrides = leg_pose_overrides(front_leg, 0.0, math.radians(upper_deg), 0.0)
                    overrides.update(leg_pose_overrides(rear_leg, 0.0, REAR_PARKING_UPPER_RAD, 0.0))
                    pose = full_pose(overrides)
                    collide, pair = self.scene.is_colliding_at_pose(pose)
                    self.assertFalse(
                        collide,
                        f"{front_leg} upper={upper_deg}deg with {rear_leg} parked "
                        f"expected clear, found {pair}",
                    )

    def test_rh_lh_transition_needs_no_extra_parking(self):
        """Checkpoint: 'RH and LH calibration do not require an additional
        cross-leg parking pose based on the current geometry audit.'"""
        for leg in ("rh", "lh"):
            with self.subTest(leg=leg):
                for upper_deg in range(50, 91, 5):
                    pose = full_pose(leg_pose_overrides(leg, 0.0, math.radians(upper_deg), 0.0))
                    collide, pair = self.scene.is_colliding_at_pose(pose)
                    self.assertFalse(collide, f"{leg} upper={upper_deg}deg expected clear, found {pair}")


if __name__ == "__main__":
    unittest.main()
