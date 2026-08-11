#!/usr/bin/env python3
"""Focused pure-unit tests for canonical V5 direct path domains."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_contact_search_v5 import (  # noqa: E402
    GEOMETRIC_CONTACT_FOUND,
    PATH_DOMAIN_DIRECT_TO_GEOMETRIC_TARGET,
    PATH_OBSTRUCTION,
    EndpointSpecV5,
    analyze_endpoint,
)


class _Model:
    def path_obstruction_pairs(self, *, exclude_pair):
        assert exclude_pair == ("parent", "child")
        return (("blocker", "body"),)

    def pair_relation(self, _link_a, _link_b):
        return "body_vs_branch"


class _Scene:
    model = _Model()

    def __init__(self):
        self.path_probes = []

    def full_pose(self, overrides):
        pose = {"joint": 0.0}
        pose.update(overrides)
        return pose

    def first_collision_at_pose(self, pose, *, link_pairs):
        angle = pose["joint"]
        if link_pairs == (("parent", "child"),):
            return (angle >= 0.83, ("parent", "child") if angle >= 0.83 else None)
        self.path_probes.append(angle)
        return (angle >= 0.44, ("blocker", "body") if angle >= 0.44 else None)


class TestCanonicalDirectPathDomain(unittest.TestCase):
    def test_path_uses_contact_target_and_parking_equal_subdivision_grid(self):
        endpoint = EndpointSpecV5(
            endpoint_id="joint:max",
            joint_name="joint",
            side="max",
            branch_root_joint_name="joint",
            structural_depth=0,
            motor_id=1,
            motor_direction=1,
            urdf_lower_rad=-0.8,
            urdf_upper_rad=0.8,
            active_link_pair=("parent", "child"),
        )
        scene = _Scene()
        result = analyze_endpoint(
            scene,
            endpoint,
            coarse_step_rad=0.1,
            envelope_margin_rad=0.2,
            bisection_resolution_rad=1e-5,
            path_domain_mode=PATH_DOMAIN_DIRECT_TO_GEOMETRIC_TARGET,
        )
        self.assertEqual(result.geometry.status, GEOMETRIC_CONTACT_FOUND)
        self.assertEqual(result.path.status, PATH_OBSTRUCTION)
        self.assertEqual(
            result.path.search_domain_rad,
            (0.0, result.geometry.contact_angle_rad),
        )
        # target≈0.83 gives nine equal baseline intervals, not fixed 0.1
        # increments.  The first coarse collision is therefore 5/9 target.
        expected_first_sample = result.geometry.contact_angle_rad * 5.0 / 9.0
        expected_coarse_probes = [
            result.geometry.contact_angle_rad * index / 9.0
            for index in range(6)
        ]
        self.assertEqual(len(scene.path_probes[:6]), 6)
        for observed, expected in zip(
            scene.path_probes[:6], expected_coarse_probes, strict=True
        ):
            self.assertAlmostEqual(observed, expected, places=14)
        self.assertAlmostEqual(
            result.path.bracket_contact_rad,
            0.44,
            delta=1e-5,
        )
        self.assertGreaterEqual(expected_first_sample, result.path.bracket_contact_rad)


if __name__ == "__main__":
    unittest.main()
