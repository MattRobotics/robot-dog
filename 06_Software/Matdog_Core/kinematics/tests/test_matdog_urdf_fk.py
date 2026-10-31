from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
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

    def test_everything_except_inertial_is_the_validated_rev01_fix_model(self):
        # The 2026-10-30 physical-property update rewrote only the <inertial> blocks.
        # With them removed, the canonical (comment-free) URDF must still be the exact
        # REV01-FIX model validated by Geometry Compiler V5: joints, axes, limits, motor
        # metadata, hip origins (+2.2 mm), mesh paths, scales. The relation of this model to
        # the REV00 calibration basis is checked by matdog_calibration_basis.py.
        root = ET.parse(self.urdf).getroot()
        for link in root.findall("link"):
            link.remove(link.find("inertial"))
        canonical = ET.canonicalize(ET.tostring(root, encoding="unicode"), strip_text=True)
        self.assertEqual(
            hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
            "d66fa1cc84a39da41a5e3a20e2dd16a98658d017a539c7f615843ded2509c2e7",
        )

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
        self.assertAlmostEqual(z, -0.0912, places=12)

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
