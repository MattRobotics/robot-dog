from __future__ import annotations

import math
from pathlib import Path
import unittest

from matdog_leg_contact import lf_foot_contact_from_joint_angles
from matdog_leg_contact_ik import (
    GroundContactIkUnreachableError,
    solve_lf_ground_contact_ik,
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


class TestMatdogLegContactIk(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()

    def test_visual_zero_target_returns_zero_solution(self):
        target = (0.2195, 0.0940, -0.0934)

        result = solve_lf_ground_contact_ik(
            repo_root=self.repo,
            target_contact_world_m=target,
            tolerance_m=1e-9,
        )

        self.assertEqual(result.iterations, 0)
        self.assertLessEqual(result.residual_m, 1e-9)
        self.assertEqual(
            result.leg_contact.contact.support_mode,
            "NOMINAL_STRIP_CONTACT",
        )

        for value in result.joint_positions_rad.values():
            self.assertAlmostEqual(value, 0.0, places=12)

    def test_nominal_contact_fk_to_ik_round_trip(self):
        cases = (
            (0.0, 0.20, -0.25),
            (0.0, -0.15, 0.35),
            (0.0, 0.45, -0.40),
        )

        for q_reference in cases:
            with self.subTest(q_reference=q_reference):
                reference = lf_foot_contact_from_joint_angles(
                    joint_positions_rad={
                        "lf_hip_joint": q_reference[0],
                        "lf_upper_leg_joint": q_reference[1],
                        "lf_lower_leg_joint": q_reference[2],
                    },
                    repo_root=self.repo,
                )

                target = (
                    reference.contact.cross_section_contact_center_world_m
                )

                self.assertEqual(
                    reference.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

                result = solve_lf_ground_contact_ik(
                    repo_root=self.repo,
                    target_contact_world_m=target,
                    initial_guess_rad=(0.0, 0.0, 0.0),
                    tolerance_m=1e-6,
                )

                self.assertLessEqual(result.residual_m, 1e-6)
                self.assertEqual(
                    result.leg_contact.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

    def test_edge_contact_target_is_solved_when_explicitly_allowed(self):
        q_reference = (math.radians(10.0), 0.0, 0.0)

        reference = lf_foot_contact_from_joint_angles(
            joint_positions_rad={
                "lf_hip_joint": q_reference[0],
                "lf_upper_leg_joint": q_reference[1],
                "lf_lower_leg_joint": q_reference[2],
            },
            repo_root=self.repo,
        )

        self.assertEqual(
            reference.contact.support_mode,
            "EDGE_BIASED_CONTACT",
        )

        result = solve_lf_ground_contact_ik(
            repo_root=self.repo,
            target_contact_world_m=(
                reference.contact.cross_section_contact_center_world_m
            ),
            initial_guess_rad=(0.0, 0.0, 0.0),
            require_nominal_strip_contact=False,
            tolerance_m=1e-6,
        )

        self.assertLessEqual(result.residual_m, 1e-6)
        self.assertEqual(
            result.leg_contact.contact.support_mode,
            "EDGE_BIASED_CONTACT",
        )

    def test_world_from_base_translation_is_respected(self):
        world_shift = (0.100, -0.050, 0.300)

        target = (
            0.2195 + world_shift[0],
            0.0940 + world_shift[1],
            -0.0934 + world_shift[2],
        )

        result = solve_lf_ground_contact_ik(
            repo_root=self.repo,
            target_contact_world_m=target,
            world_from_base_translation_m=world_shift,
            tolerance_m=1e-9,
        )

        self.assertEqual(result.iterations, 0)
        self.assertLessEqual(result.residual_m, 1e-9)

    def test_far_target_is_rejected(self):
        with self.assertRaises(GroundContactIkUnreachableError):
            solve_lf_ground_contact_ik(
                repo_root=self.repo,
                target_contact_world_m=(1.0, 1.0, 1.0),
                max_iterations=120,
                tolerance_m=1e-6,
            )

    def test_incomplete_target_joint_seed_is_rejected(self):
        with self.assertRaises(ValueError):
            solve_lf_ground_contact_ik(
                repo_root=self.repo,
                target_contact_world_m=(
                    0.2195,
                    0.0940,
                    -0.0934,
                ),
                initial_guess_rad=(0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
