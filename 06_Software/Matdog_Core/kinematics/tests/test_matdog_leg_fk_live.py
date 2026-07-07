from __future__ import annotations

from pathlib import Path
import unittest

from matdog_leg_fk_live import (
    LF_JOINT_NAMES,
    fk_from_encoder_ticks,
    joint_radians_from_encoder_ticks,
    load_lf_calibration,
    visual_zero_errors_ticks,
)
from matdog_urdf_fk import CANONICAL_URDF_RELATIVE_PATH


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / CANONICAL_URDF_RELATIVE_PATH).is_file():
            return parent

    raise RuntimeError(
        "Repository root non trovato: URDF canonico assente dai parent"
    )


class TestMatdogLegFkLivePureLayer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()
        cls.config_path = (
            cls.repo
            / "06_Software/Matdog_Core/calibration/"
            "MATDOG_JOINT_CALIBRATION.yaml"
        )
        cls.calibration = load_lf_calibration(cls.config_path)

    def test_lf_joint_order_servo_mapping_and_direction(self):
        self.assertEqual(
            tuple(item.joint_name for item in self.calibration),
            LF_JOINT_NAMES,
        )
        self.assertEqual(
            tuple(item.servo_id for item in self.calibration),
            (13, 12, 11),
        )
        self.assertEqual(
            tuple(item.direction for item in self.calibration),
            (-1, 1, -1),
        )

    def test_visual_zero_encoder_map_becomes_zero_radians(self):
        ticks = {
            item.servo_id: item.zero_encoder_visual
            for item in self.calibration
        }

        q_rad = joint_radians_from_encoder_ticks(
            self.calibration,
            ticks,
        )

        self.assertEqual(
            q_rad,
            {
                "lf_hip_joint": 0.0,
                "lf_upper_leg_joint": 0.0,
                "lf_lower_leg_joint": 0.0,
            },
        )

    def test_visual_zero_encoder_map_reproduces_urdf_foot_frame(self):
        ticks = {
            item.servo_id: item.zero_encoder_visual
            for item in self.calibration
        }

        q_rad, fk_result = fk_from_encoder_ticks(
            self.repo,
            self.calibration,
            ticks,
        )

        self.assertEqual(
            q_rad,
            {
                "lf_hip_joint": 0.0,
                "lf_upper_leg_joint": 0.0,
                "lf_lower_leg_joint": 0.0,
            },
        )

        x, y, z = fk_result.tip_position_m
        self.assertAlmostEqual(x, 0.2195, places=12)
        self.assertAlmostEqual(y, 0.0940, places=12)
        self.assertAlmostEqual(z, -0.0934, places=12)

    def test_visual_zero_error_is_circular_and_unsigned(self):
        ticks = {
            item.servo_id: item.zero_encoder_visual
            for item in self.calibration
        }
        ticks[13] = (ticks[13] + 10) % 4096
        ticks[12] = (ticks[12] - 4) % 4096
        ticks[11] = (ticks[11] + 2) % 4096

        errors = visual_zero_errors_ticks(
            self.calibration,
            ticks,
        )

        self.assertEqual(
            errors,
            {
                "lf_hip_joint": 10,
                "lf_upper_leg_joint": 4,
                "lf_lower_leg_joint": 2,
            },
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
