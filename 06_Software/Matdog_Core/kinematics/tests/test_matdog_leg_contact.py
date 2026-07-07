from __future__ import annotations

import math
from pathlib import Path
import unittest

from matdog_leg_contact import (
    LegContactError,
    lf_foot_contact_from_joint_angles,
    validate_canonical_foot_joint_bindings,
)
from matdog_urdf_fk import (
    canonical_urdf_path,
    forward_kinematics,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (
            parent
            / "03_CAD/URDF/matt_robodog_rev00/"
            "matt_robodog_rev00.urdf"
        ).is_file():
            return parent

    raise RuntimeError("Repository root MATDOG non trovato")


IDENTITY = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class TestMatdogLegContact(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()
        cls.urdf = canonical_urdf_path(cls.repo)

    def test_canonical_foot_joint_bindings_pass(self):
        validate_canonical_foot_joint_bindings(self.repo)

    def test_visual_zero_contact_matches_fk_foot_origin(self):
        joints = {
            "lf_hip_joint": 0.0,
            "lf_upper_leg_joint": 0.0,
            "lf_lower_leg_joint": 0.0,
        }

        result = lf_foot_contact_from_joint_angles(
            joint_positions_rad=joints,
            repo_root=self.repo,
        )

        fk = forward_kinematics(
            urdf_path=self.urdf,
            root_link="base_link",
            tip_link="lf_foot_link",
            joint_positions_rad=joints,
            enforce_limits=True,
        )

        self.assertEqual(
            result.contact.support_mode,
            "NOMINAL_STRIP_CONTACT",
        )

        for actual, expected in zip(
            result.world_from_foot_translation_m,
            fk.tip_position_m,
        ):
            self.assertAlmostEqual(actual, expected, places=12)

        for actual, expected in zip(
            result.contact.cross_section_contact_center_world_m,
            fk.tip_position_m,
        ):
            self.assertAlmostEqual(actual, expected, places=12)

        self.assertAlmostEqual(
            result.contact.support_strip_end_a_world_m[1],
            fk.tip_position_m[1] - 0.00495,
            places=12,
        )
        self.assertAlmostEqual(
            result.contact.support_strip_end_b_world_m[1],
            fk.tip_position_m[1] + 0.00495,
            places=12,
        )

    def test_lf_hip_tilt_changes_contact_to_edge_biased(self):
        joints = {
            "lf_hip_joint": math.radians(10.0),
            "lf_upper_leg_joint": 0.0,
            "lf_lower_leg_joint": 0.0,
        }

        result = lf_foot_contact_from_joint_angles(
            joint_positions_rad=joints,
            repo_root=self.repo,
        )

        self.assertEqual(
            result.contact.support_mode,
            "EDGE_BIASED_CONTACT",
        )
        self.assertAlmostEqual(
            result.contact.axis_tilt_from_ground_rad,
            math.radians(10.0),
            places=12,
        )

        self.assertLess(
            result.contact.lowest_core_point_world_m[2],
            result.contact.cross_section_contact_center_world_m[2],
        )

    def test_world_from_base_translation_is_applied_to_contact(self):
        joints = {
            "lf_hip_joint": 0.0,
            "lf_upper_leg_joint": 0.0,
            "lf_lower_leg_joint": 0.0,
        }

        base_result = lf_foot_contact_from_joint_angles(
            joint_positions_rad=joints,
            repo_root=self.repo,
        )

        world_shift = (0.100, -0.050, 0.300)

        shifted_result = lf_foot_contact_from_joint_angles(
            joint_positions_rad=joints,
            repo_root=self.repo,
            world_from_base_translation_m=world_shift,
        )

        for base, shifted, delta in zip(
            base_result.contact.cross_section_contact_center_world_m,
            shifted_result.contact.cross_section_contact_center_world_m,
            world_shift,
        ):
            self.assertAlmostEqual(
                shifted - base,
                delta,
                places=12,
            )

    def test_rejects_incomplete_or_unknown_joint_target(self):
        with self.assertRaises(LegContactError):
            lf_foot_contact_from_joint_angles(
                joint_positions_rad={
                    "lf_hip_joint": 0.0,
                    "lf_upper_leg_joint": 0.0,
                },
                repo_root=self.repo,
            )

        with self.assertRaises(LegContactError):
            lf_foot_contact_from_joint_angles(
                joint_positions_rad={
                    "lf_hip_joint": 0.0,
                    "lf_upper_leg_joint": 0.0,
                    "lf_lower_leg_joint": 0.0,
                    "rf_hip_joint": 0.0,
                },
                repo_root=self.repo,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
