"""Test automatici MATDOG per il livello scena/FK del Geometry Compiler."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_scene import (  # noqa: E402
    ALL_LINKS,
    LEG_IDS,
    SERVO_ID_BY_JOINT,
    GeometrySceneError,
    RobotScene,
    all_candidate_collision_pairs,
    full_pose,
    home_pose,
    is_adjacent_pair,
    joint_name,
    leg_of_joint,
    leg_of_link,
    leg_pose_overrides,
    link_name,
    pair_is_cross_leg,
)


class TestNamingAndAdjacency(unittest.TestCase):
    def test_all_links_count(self):
        # base_link + 4 legs x 4 links (hip, upper, lower, foot)
        self.assertEqual(len(ALL_LINKS), 1 + 4 * 4)
        self.assertEqual(len(set(ALL_LINKS)), len(ALL_LINKS))

    def test_leg_of_link(self):
        self.assertIsNone(leg_of_link("base_link"))
        self.assertEqual(leg_of_link("lf_hip_link"), "lf")
        self.assertEqual(leg_of_link("rh_lower_leg_link"), "rh")

        with self.assertRaises(GeometrySceneError):
            leg_of_link("not_a_real_link")

    def test_leg_of_joint(self):
        self.assertEqual(leg_of_joint("lh_upper_leg_joint"), "lh")

        with self.assertRaises(GeometrySceneError):
            leg_of_joint("bogus_joint")

    def test_adjacent_pairs_excluded_from_candidates(self):
        candidates = all_candidate_collision_pairs()

        self.assertTrue(is_adjacent_pair("base_link", "lf_hip_link"))
        self.assertTrue(is_adjacent_pair("lf_hip_link", "lf_upper_leg_link"))
        self.assertTrue(is_adjacent_pair("lf_upper_leg_link", "lf_lower_leg_link"))
        self.assertTrue(is_adjacent_pair("lf_lower_leg_link", "lf_foot_link"))

        for pair in candidates:
            self.assertFalse(is_adjacent_pair(*pair), f"{pair} should not be a candidate")

        # every non-adjacent combination should appear exactly once
        expected_count = 0
        for i, a in enumerate(ALL_LINKS):
            for b in ALL_LINKS[i + 1 :]:
                if not is_adjacent_pair(a, b):
                    expected_count += 1

        self.assertEqual(len(candidates), expected_count)
        self.assertEqual(len(set(candidates)), len(candidates))

    def test_pair_is_cross_leg(self):
        self.assertFalse(pair_is_cross_leg("base_link", "lf_upper_leg_link"))
        self.assertFalse(pair_is_cross_leg("lf_hip_link", "lf_lower_leg_link"))
        self.assertTrue(pair_is_cross_leg("lf_lower_leg_link", "rh_lower_leg_link"))
        self.assertTrue(pair_is_cross_leg("lf_foot_link", "lh_foot_link"))

    def test_servo_mapping_matches_checkpoint(self):
        # Canonical mapping from MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
        expected = {
            "lf_hip_joint": 13, "lf_upper_leg_joint": 12, "lf_lower_leg_joint": 11,
            "rf_hip_joint": 23, "rf_upper_leg_joint": 22, "rf_lower_leg_joint": 21,
            "rh_hip_joint": 33, "rh_upper_leg_joint": 32, "rh_lower_leg_joint": 31,
            "lh_hip_joint": 43, "lh_upper_leg_joint": 42, "lh_lower_leg_joint": 41,
        }
        self.assertEqual(SERVO_ID_BY_JOINT, expected)

    def test_home_pose_is_all_zero(self):
        pose = home_pose()
        self.assertEqual(len(pose), 12)
        self.assertTrue(all(value == 0.0 for value in pose.values()))

    def test_full_pose_rejects_unknown_joint(self):
        with self.assertRaises(GeometrySceneError):
            full_pose({"not_a_joint": 0.1})

    def test_link_name_joint_name_roundtrip(self):
        for leg in LEG_IDS:
            for group in ("hip", "upper_leg", "lower_leg"):
                self.assertTrue(joint_name(leg, group).startswith(leg))
                self.assertTrue(link_name(leg, group).startswith(leg))


class TestRobotSceneLoad(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene()

    def test_all_meshes_loaded(self):
        for link in ALL_LINKS:
            mesh = self.scene.mesh(link)
            self.assertGreater(mesh.triangle_count, 0)

    def test_home_pose_is_self_collision_free(self):
        collide, pair = self.scene.is_colliding_at_pose(home_pose())
        self.assertFalse(collide, f"home pose expected collision-free, found {pair}")

    def test_link_transform_base_link_is_identity(self):
        transform = self.scene.link_transform("base_link", home_pose())
        self.assertTrue((transform.rotation == __import__("numpy").eye(3)).all())
        self.assertTrue((transform.translation == 0.0).all())


class TestFrontHindGeometry(unittest.TestCase):
    """Section 5 of the canonical handoff: front hip Z ~= 0.0465 m, hind hip
    Z ~= 0.0265 m -- a real, numerically-derivable 20 mm difference, not an
    assumption."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene()

    def test_front_hind_hip_height_differs_by_20mm(self):
        pose = home_pose()
        lf_hip_z = self.scene.link_transform("lf_hip_link", pose).translation[2]
        rh_hip_z = self.scene.link_transform("rh_hip_link", pose).translation[2]

        self.assertAlmostEqual(lf_hip_z, 0.0465, places=6)
        self.assertAlmostEqual(rh_hip_z, 0.0265, places=6)
        self.assertAlmostEqual(lf_hip_z - rh_hip_z, 0.020, places=6)

    def test_lf_rf_hip_height_equal(self):
        pose = home_pose()
        lf_z = self.scene.link_transform("lf_hip_link", pose).translation[2]
        rf_z = self.scene.link_transform("rf_hip_link", pose).translation[2]
        self.assertAlmostEqual(lf_z, rf_z, places=9)

    def test_rh_lh_hip_height_equal(self):
        pose = home_pose()
        rh_z = self.scene.link_transform("rh_hip_link", pose).translation[2]
        lh_z = self.scene.link_transform("lh_hip_link", pose).translation[2]
        self.assertAlmostEqual(rh_z, lh_z, places=9)


class TestMirrorGeometry(unittest.TestCase):
    """LF/RF and RH/LH should be Y-mirror-equivalent: same hip origin X/Z,
    mirrored Y, and mirrored joint motorDirection (already asserted by
    validate_servo_mapping / URDF); this test checks the FK geometry
    itself rather than assuming mirroring from naming convention alone."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene()

    def _hip_origin(self, link: str):
        return self.scene.link_transform(link, home_pose()).translation

    def test_lf_rf_hip_origin_mirrors_in_y(self):
        lf = self._hip_origin("lf_hip_link")
        rf = self._hip_origin("rf_hip_link")
        self.assertAlmostEqual(lf[0], rf[0], places=9)
        self.assertAlmostEqual(lf[1], -rf[1], places=9)
        self.assertAlmostEqual(lf[2], rf[2], places=9)

    def test_rh_lh_hip_origin_mirrors_in_y(self):
        rh = self._hip_origin("rh_hip_link")
        lh = self._hip_origin("lh_hip_link")
        self.assertAlmostEqual(rh[0], lh[0], places=9)
        self.assertAlmostEqual(rh[1], -lh[1], places=9)
        self.assertAlmostEqual(rh[2], lh[2], places=9)

    def test_upper_lower_joint_motion_is_side_independent(self):
        # The upper/lower joint axis (local Y) and every leg's joint-origin
        # rpy are both zero in the canonical URDF, so a given upper/lower q
        # produces an identical local XZ foot displacement on every leg
        # (verified numerically here rather than assumed): only the hip
        # joint's response differs by side, because the hip axis (local X)
        # couples into the leg's own Y offset from the hip, which differs
        # in sign between left and right legs.
        eps = math.radians(5.0)
        pose0 = home_pose()

        for group, index in (("upper_leg", 1), ("lower_leg", 2)):
            deltas = {}

            for leg in LEG_IDS:
                q = [0.0, 0.0, 0.0]
                q[index] = eps
                pose = full_pose(leg_pose_overrides(leg, *q))
                p1 = self.scene.link_transform(f"{leg}_foot_link", pose).translation
                p0 = self.scene.link_transform(f"{leg}_foot_link", pose0).translation
                deltas[leg] = p1 - p0

            reference = deltas["lf"]

            for leg in LEG_IDS[1:]:
                for axis in range(3):
                    self.assertAlmostEqual(
                        deltas[leg][axis], reference[axis], places=6,
                        msg=f"{group} axis={axis}: {leg} vs lf",
                    )


if __name__ == "__main__":
    unittest.main()
