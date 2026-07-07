from __future__ import annotations

from pathlib import Path
import unittest

from matdog_urdf_fk import (
    CANONICAL_URDF_RELATIVE_PATH,
    CANONICAL_URDF_SHA256,
    LF_CHAIN_JOINT_NAMES,
    canonical_urdf_path,
    forward_kinematics,
    load_urdf_joints,
    sha256_file,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / CANONICAL_URDF_RELATIVE_PATH).is_file():
            return parent
    raise RuntimeError(
        "Repository root non trovato: URDF canonico assente dai parent"
    )


class TestMatdogUrdfFk(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()
        cls.urdf = canonical_urdf_path(cls.repo)

    def test_canonical_urdf_integrity(self):
        self.assertTrue(self.urdf.is_file())
        self.assertEqual(sha256_file(self.urdf), CANONICAL_URDF_SHA256)

    def test_lf_chain_matches_canonical_joint_order(self):
        result = forward_kinematics(
            self.urdf,
            root_link="base_link",
            tip_link="lf_foot_link",
            joint_positions_rad={},
        )
        self.assertEqual(result.chain_joint_names, LF_CHAIN_JOINT_NAMES)

    def test_lf_visual_zero_foot_position(self):
        result = forward_kinematics(
            self.urdf,
            root_link="base_link",
            tip_link="lf_foot_link",
            joint_positions_rad={
                "lf_hip_joint": 0.0,
                "lf_upper_leg_joint": 0.0,
                "lf_lower_leg_joint": 0.0,
            },
        )

        x, y, z = result.tip_position_m

        self.assertAlmostEqual(x, 0.2195, places=12)
        self.assertAlmostEqual(y, 0.0940, places=12)
        self.assertAlmostEqual(z, -0.0934, places=12)

    def test_lf_joint_axes_are_canonical(self):
        joints = load_urdf_joints(self.urdf)

        self.assertEqual(joints["lf_hip_joint"].axis_xyz, (1.0, 0.0, 0.0))
        self.assertEqual(
            joints["lf_upper_leg_joint"].axis_xyz,
            (0.0, 1.0, 0.0),
        )
        self.assertEqual(
            joints["lf_lower_leg_joint"].axis_xyz,
            (0.0, 1.0, 0.0),
        )

    def test_urdf_limit_violation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "fuori dai limiti URDF"):
            forward_kinematics(
                self.urdf,
                root_link="base_link",
                tip_link="lf_foot_link",
                joint_positions_rad={
                    "lf_hip_joint": 1.0,
                    "lf_upper_leg_joint": 0.0,
                    "lf_lower_leg_joint": 0.0,
                },
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
