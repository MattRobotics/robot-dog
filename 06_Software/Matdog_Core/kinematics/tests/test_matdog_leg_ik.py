from __future__ import annotations

from pathlib import Path
import unittest

from matdog_leg_ik import (
    IkUnreachableError,
    LF_JOINT_NAMES,
    solve_lf_position_ik,
)
from matdog_urdf_fk import (
    CANONICAL_URDF_RELATIVE_PATH,
    canonical_urdf_path,
    forward_kinematics,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / CANONICAL_URDF_RELATIVE_PATH).is_file():
            return parent

    raise RuntimeError(
        "Repository root non trovato: URDF canonico assente dai parent"
    )


def lf_foot_from_joint_angles(
    urdf_path: Path,
    q_rad: tuple[float, float, float],
) -> tuple[float, float, float]:
    result = forward_kinematics(
        urdf_path=urdf_path,
        root_link="base_link",
        tip_link="lf_foot_link",
        joint_positions_rad=dict(zip(LF_JOINT_NAMES, q_rad)),
    )
    return result.tip_position_m


class TestMatdogLegIk(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()
        cls.urdf = canonical_urdf_path(cls.repo)

    def test_visual_zero_target_solves_to_zero(self):
        target = (0.2195, 0.0940, -0.0934)

        result = solve_lf_position_ik(
            repo_root=self.repo,
            target_position_m=target,
            tolerance_m=1e-9,
        )

        self.assertLessEqual(result.residual_m, 1e-9)
        self.assertEqual(result.iterations, 0)

        for joint_name in LF_JOINT_NAMES:
            self.assertAlmostEqual(
                result.joint_positions_rad[joint_name],
                0.0,
                places=12,
            )

    def test_fk_to_ik_to_fk_round_trip_for_reachable_targets(self):
        cases = (
            (0.15, 0.50, -0.50),
            (-0.20, 1.00, -0.60),
            (0.30, 1.50, -1.00),
        )

        for q_reference in cases:
            with self.subTest(q_reference=q_reference):
                target = lf_foot_from_joint_angles(self.urdf, q_reference)

                result = solve_lf_position_ik(
                    repo_root=self.repo,
                    target_position_m=target,
                    initial_guess_rad=(0.0, 0.0, 0.0),
                    tolerance_m=1e-7,
                )

                self.assertLessEqual(result.residual_m, 1e-7)

                reconstructed = lf_foot_from_joint_angles(
                    self.urdf,
                    tuple(
                        result.joint_positions_rad[name]
                        for name in LF_JOINT_NAMES
                    ),
                )

                for actual, expected in zip(reconstructed, target):
                    self.assertAlmostEqual(actual, expected, places=6)

    def test_far_target_is_rejected(self):
        with self.assertRaises(IkUnreachableError):
            solve_lf_position_ik(
                repo_root=self.repo,
                target_position_m=(1.0, 1.0, 1.0),
                tolerance_m=1e-6,
                max_iterations=120,
            )

    def test_initial_guess_outside_urdf_limits_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "fuori dai limiti"):
            solve_lf_position_ik(
                repo_root=self.repo,
                target_position_m=(0.2195, 0.0940, -0.0934),
                initial_guess_rad=(2.0, 0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
