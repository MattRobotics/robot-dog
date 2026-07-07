from __future__ import annotations

import math
from pathlib import Path
import unittest

from matdog_quadruped_leg_contact import (
    LEG_IDS,
    leg_foot_contact_from_joint_angles,
    leg_joint_names,
)
from matdog_quadruped_leg_contact_ik import (
    QuadrupedContactIkConstraintError,
    QuadrupedContactIkUnreachableError,
    solve_leg_contact_reference_ik,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent
            / "03_CAD/URDF/matt_robodog_rev00/"
            "matt_robodog_rev00.urdf"
        )

        if candidate.is_file():
            return parent

    raise RuntimeError("Repository root MATDOG non trovato")


def zero_joint_map(leg: str) -> dict[str, float]:
    names = leg_joint_names(leg)

    return {
        names[0]: 0.0,
        names[1]: 0.0,
        names[2]: 0.0,
    }


class TestMatdogQuadrupedLegContactIk(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()

    def test_visual_zero_returns_zero_solution_for_every_leg(self):
        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                zero_joints = zero_joint_map(leg)

                reference = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=zero_joints,
                    repo_root=self.repo,
                )

                result = solve_leg_contact_reference_ik(
                    leg_id=leg,
                    target_contact_reference_world_m=(
                        reference.contact
                        .cross_section_contact_center_world_m
                    ),
                    repo_root=self.repo,
                    tolerance_m=1e-9,
                )

                self.assertEqual(result.iterations, 0)
                self.assertLessEqual(result.residual_m, 1e-9)
                self.assertEqual(
                    result.leg_contact.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

                for joint_name in leg_joint_names(leg):
                    self.assertAlmostEqual(
                        result.joint_positions_rad[joint_name],
                        0.0,
                        places=12,
                    )

    def test_nominal_contact_fk_to_ik_round_trip_for_every_leg(self):
        reference_q = (
            0.0,
            0.20,
            -0.25,
        )

        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                names = leg_joint_names(leg)

                reference = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad={
                        names[0]: reference_q[0],
                        names[1]: reference_q[1],
                        names[2]: reference_q[2],
                    },
                    repo_root=self.repo,
                )

                self.assertEqual(
                    reference.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

                result = solve_leg_contact_reference_ik(
                    leg_id=leg,
                    target_contact_reference_world_m=(
                        reference.contact
                        .cross_section_contact_center_world_m
                    ),
                    repo_root=self.repo,
                    initial_guess_rad=(0.0, 0.0, 0.0),
                    tolerance_m=1e-6,
                )

                self.assertLessEqual(result.residual_m, 1e-6)
                self.assertEqual(
                    result.leg_contact.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

    def test_world_translation_is_respected_for_every_leg(self):
        world_shift = (0.100, -0.050, 0.300)

        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                zero_joints = zero_joint_map(leg)

                base_reference = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=zero_joints,
                    repo_root=self.repo,
                )

                base_target = (
                    base_reference.contact
                    .cross_section_contact_center_world_m
                )

                shifted_target = tuple(
                    coordinate + delta
                    for coordinate, delta in zip(
                        base_target,
                        world_shift,
                    )
                )

                result = solve_leg_contact_reference_ik(
                    leg_id=leg,
                    target_contact_reference_world_m=shifted_target,
                    repo_root=self.repo,
                    world_from_base_translation_m=world_shift,
                    tolerance_m=1e-9,
                )

                self.assertEqual(result.iterations, 0)
                self.assertLessEqual(result.residual_m, 1e-9)

    def test_edge_contact_target_is_solved_when_explicitly_allowed(self):
        leg = "rf"
        names = leg_joint_names(leg)

        reference = leg_foot_contact_from_joint_angles(
            leg_id=leg,
            joint_positions_rad={
                names[0]: math.radians(10.0),
                names[1]: 0.0,
                names[2]: 0.0,
            },
            repo_root=self.repo,
        )

        self.assertEqual(
            reference.contact.support_mode,
            "EDGE_BIASED_CONTACT",
        )

        result = solve_leg_contact_reference_ik(
            leg_id=leg,
            target_contact_reference_world_m=(
                reference.contact
                .cross_section_contact_center_world_m
            ),
            repo_root=self.repo,
            initial_guess_rad=(0.0, 0.0, 0.0),
            require_nominal_strip_contact=False,
            tolerance_m=1e-6,
        )

        self.assertLessEqual(result.residual_m, 1e-6)
        self.assertEqual(
            result.leg_contact.contact.support_mode,
            "EDGE_BIASED_CONTACT",
        )

    def test_edge_contact_is_rejected_when_nominal_policy_is_required(self):
        leg = "rf"
        names = leg_joint_names(leg)
        edge_q = (
            math.radians(10.0),
            0.0,
            0.0,
        )

        reference = leg_foot_contact_from_joint_angles(
            leg_id=leg,
            joint_positions_rad={
                names[0]: edge_q[0],
                names[1]: edge_q[1],
                names[2]: edge_q[2],
            },
            repo_root=self.repo,
        )

        with self.assertRaises(QuadrupedContactIkConstraintError):
            solve_leg_contact_reference_ik(
                leg_id=leg,
                target_contact_reference_world_m=(
                    reference.contact
                    .cross_section_contact_center_world_m
                ),
                repo_root=self.repo,
                initial_guess_rad=edge_q,
                require_nominal_strip_contact=True,
                tolerance_m=1e-9,
            )

    def test_far_target_is_rejected(self):
        with self.assertRaises(QuadrupedContactIkUnreachableError):
            solve_leg_contact_reference_ik(
                leg_id="lh",
                target_contact_reference_world_m=(
                    1.0,
                    1.0,
                    1.0,
                ),
                repo_root=self.repo,
                max_iterations=120,
                tolerance_m=1e-6,
            )

    def test_incomplete_initial_guess_is_rejected(self):
        with self.assertRaises(ValueError):
            solve_leg_contact_reference_ik(
                leg_id="rh",
                target_contact_reference_world_m=(
                    -0.0055,
                    -0.0940,
                    -0.1134,
                ),
                repo_root=self.repo,
                initial_guess_rad=(0.0, 0.0),  # type: ignore[arg-type]
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
