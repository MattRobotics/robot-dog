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


def visual_zero_world_transform(repo: Path):
    """Derive the C2-A world_from_base transform from visual-zero contacts."""
    contacts_base = {}

    for leg in LEG_IDS:
        reference = leg_foot_contact_from_joint_angles(
            leg_id=leg,
            joint_positions_rad=zero_joint_map(leg),
            repo_root=repo,
        )

        if reference.contact.support_mode != "NOMINAL_STRIP_CONTACT":
            raise AssertionError(
                f"{leg}: visual-zero support mode inatteso: "
                f"{reference.contact.support_mode}"
            )

        contacts_base[leg] = (
            reference.contact.cross_section_contact_center_world_m
        )

    front = contacts_base["lf"]
    rear = contacts_base["lh"]

    pitch_y_rad = math.atan2(
        front[2] - rear[2],
        front[0] - rear[0],
    )

    cosine = math.cos(pitch_y_rad)
    sine = math.sin(pitch_y_rad)

    rotation = (
        (cosine, 0.0, sine),
        (0.0, 1.0, 0.0),
        (-sine, 0.0, cosine),
    )

    # The contact reference is ground-normal dependent.  Derive the
    # translation after pitch through the canonical contact model itself,
    # rather than rigidly rotating a point calculated in base coordinates.
    tilted_zero_z_values = []

    for leg in LEG_IDS:
        reference = leg_foot_contact_from_joint_angles(
            leg_id=leg,
            joint_positions_rad=zero_joint_map(leg),
            repo_root=repo,
            world_from_base_rotation=rotation,
            world_from_base_translation_m=(0.0, 0.0, 0.0),
        )

        if reference.contact.support_mode != "NOMINAL_STRIP_CONTACT":
            raise AssertionError(
                f"{leg}: visual-zero tilted support mode inatteso: "
                f"{reference.contact.support_mode}"
            )

        tilted_zero_z_values.append(
            reference.contact.cross_section_contact_center_world_m[2]
        )

    if (
        max(tilted_zero_z_values)
        - min(tilted_zero_z_values)
        > 1e-12
    ):
        raise AssertionError(
            "visual-zero tilted contact plane non complanare"
        )

    base_z_world_m = -(
        sum(tilted_zero_z_values) / len(tilted_zero_z_values)
    )

    translation = (
        0.0,
        0.0,
        base_z_world_m,
    )

    return rotation, translation, pitch_y_rad


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


    def test_visual_zero_world_closure_for_every_leg(self):
        rotation, translation, pitch_y_rad = (
            visual_zero_world_transform(self.repo)
        )

        self.assertAlmostEqual(
            pitch_y_rad,
            0.08865588186743747,
            places=12,
        )
        self.assertAlmostEqual(
            translation[2],
            0.112526186261669,
            places=12,
        )

        world_z_values = []

        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                reference = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=zero_joint_map(leg),
                    repo_root=self.repo,
                    world_from_base_rotation=rotation,
                    world_from_base_translation_m=translation,
                )

                self.assertEqual(
                    reference.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

                world_z_values.append(
                    reference.contact
                    .cross_section_contact_center_world_m[2]
                )

        self.assertLessEqual(
            max(world_z_values) - min(world_z_values),
            1e-12,
        )

        for world_z_m in world_z_values:
            self.assertAlmostEqual(
                world_z_m,
                0.0,
                places=12,
            )

    def test_nonzero_world_closure_and_ik_round_trip_for_every_leg(self):
        q_probe = (
            0.0,
            0.20,
            -0.25,
        )
        tolerance_m = 1e-6

        rotation, c2a_translation, _pitch_y_rad = (
            visual_zero_world_transform(self.repo)
        )

        contacts_under_c2a = {}

        for leg in LEG_IDS:
            names = leg_joint_names(leg)

            reference = leg_foot_contact_from_joint_angles(
                leg_id=leg,
                joint_positions_rad=dict(zip(names, q_probe)),
                repo_root=self.repo,
                world_from_base_rotation=rotation,
                world_from_base_translation_m=c2a_translation,
            )

            self.assertEqual(
                reference.contact.support_mode,
                "NOMINAL_STRIP_CONTACT",
            )

            contacts_under_c2a[leg] = (
                reference.contact
                .cross_section_contact_center_world_m
            )

        c2a_z_values = [
            contact[2]
            for contact in contacts_under_c2a.values()
        ]
        c2a_plane_z_m = sum(c2a_z_values) / len(c2a_z_values)

        self.assertLessEqual(
            max(c2a_z_values) - min(c2a_z_values),
            1e-12,
        )
        self.assertAlmostEqual(
            c2a_plane_z_m,
            0.008597372202434,
            places=12,
        )

        closed_translation = (
            0.0,
            0.0,
            c2a_translation[2] - c2a_plane_z_m,
        )

        self.assertAlmostEqual(
            closed_translation[2],
            0.103928814059235,
            places=12,
        )

        target_z_values = []
        achieved_z_values = []

        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                names = leg_joint_names(leg)

                reference = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=dict(zip(names, q_probe)),
                    repo_root=self.repo,
                    world_from_base_rotation=rotation,
                    world_from_base_translation_m=closed_translation,
                )

                target = (
                    reference.contact
                    .cross_section_contact_center_world_m
                )
                target_z_values.append(target[2])

                self.assertEqual(
                    reference.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )
                self.assertAlmostEqual(
                    target[2],
                    0.0,
                    places=12,
                )

                result = solve_leg_contact_reference_ik(
                    leg_id=leg,
                    target_contact_reference_world_m=target,
                    repo_root=self.repo,
                    initial_guess_rad=(0.0, 0.0, 0.0),
                    world_from_base_rotation=rotation,
                    world_from_base_translation_m=closed_translation,
                    require_nominal_strip_contact=True,
                    tolerance_m=tolerance_m,
                )

                self.assertLessEqual(
                    result.residual_m,
                    tolerance_m,
                )
                self.assertEqual(
                    result.leg_contact.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

                solved_q = tuple(
                    result.joint_positions_rad[name]
                    for name in names
                )

                for solved_value, expected_value in zip(
                    solved_q,
                    q_probe,
                ):
                    self.assertAlmostEqual(
                        solved_value,
                        expected_value,
                        places=5,
                    )

                achieved_z_values.append(
                    result.achieved_contact_reference_world_m[2]
                )

        self.assertLessEqual(
            max(target_z_values) - min(target_z_values),
            1e-12,
        )
        self.assertLessEqual(
            max(abs(value) for value in target_z_values),
            1e-12,
        )
        self.assertLessEqual(
            max(achieved_z_values) - min(achieved_z_values),
            tolerance_m,
        )
        self.assertLessEqual(
            max(abs(value) for value in achieved_z_values),
            tolerance_m,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
