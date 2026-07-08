from __future__ import annotations

from pathlib import Path
import unittest

from matdog_leg_fk_live import (
    LEG_JOINT_NAMES,
    LEG_TIP_LINKS,
    fk_from_encoder_ticks,
    joint_radians_from_encoder_ticks,
    load_leg_calibration,
    visual_zero_errors_ticks,
)
from matdog_urdf_fk import CANONICAL_URDF_RELATIVE_PATH


EXPECTED_LEG_CONFIG = {
    "lf": {
        "servos": (13, 12, 11),
        "directions": (-1, 1, -1),
        "tip_position_m": (0.2195, 0.0940, -0.0934),
    },
    "rf": {
        "servos": (23, 22, 21),
        "directions": (-1, -1, 1),
        "tip_position_m": (0.2195, -0.0940, -0.0934),
    },
    "rh": {
        "servos": (33, 32, 31),
        "directions": (1, -1, 1),
        "tip_position_m": (-0.0055, -0.0940, -0.1134),
    },
    "lh": {
        "servos": (43, 42, 41),
        "directions": (1, 1, -1),
        "tip_position_m": (-0.0055, 0.0940, -0.1134),
    },
}


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

    def load_calibration(self, leg: str):
        return load_leg_calibration(self.config_path, leg)

    def visual_zero_ticks(self, calibration):
        return {
            item.servo_id: item.zero_encoder_visual
            for item in calibration
        }

    def test_joint_order_servo_mapping_and_direction_for_every_leg(self):
        for leg, expected in EXPECTED_LEG_CONFIG.items():
            with self.subTest(leg=leg):
                calibration = self.load_calibration(leg)

                self.assertEqual(
                    tuple(item.joint_name for item in calibration),
                    LEG_JOINT_NAMES[leg],
                )
                self.assertEqual(
                    tuple(item.servo_id for item in calibration),
                    expected["servos"],
                )
                self.assertEqual(
                    tuple(item.direction for item in calibration),
                    expected["directions"],
                )

    def test_visual_zero_encoder_map_becomes_zero_radians_for_every_leg(self):
        for leg in EXPECTED_LEG_CONFIG:
            with self.subTest(leg=leg):
                calibration = self.load_calibration(leg)
                ticks = self.visual_zero_ticks(calibration)

                q_rad = joint_radians_from_encoder_ticks(
                    calibration,
                    ticks,
                )

                self.assertEqual(
                    q_rad,
                    {
                        joint_name: 0.0
                        for joint_name in LEG_JOINT_NAMES[leg]
                    },
                )

    def test_visual_zero_encoder_map_reproduces_urdf_foot_frame_for_every_leg(
        self,
    ):
        for leg, expected in EXPECTED_LEG_CONFIG.items():
            with self.subTest(leg=leg):
                calibration = self.load_calibration(leg)
                ticks = self.visual_zero_ticks(calibration)
                tip_link = LEG_TIP_LINKS[leg]

                q_rad, fk_result = fk_from_encoder_ticks(
                    self.repo,
                    calibration,
                    ticks,
                    tip_link,
                )

                self.assertEqual(
                    q_rad,
                    {
                        joint_name: 0.0
                        for joint_name in LEG_JOINT_NAMES[leg]
                    },
                )

                expected_x, expected_y, expected_z = expected[
                    "tip_position_m"
                ]
                x, y, z = fk_result.tip_position_m

                self.assertAlmostEqual(x, expected_x, places=12)
                self.assertAlmostEqual(y, expected_y, places=12)
                self.assertAlmostEqual(z, expected_z, places=12)

    def test_visual_zero_error_is_circular_and_unsigned_for_every_leg(self):
        offsets_by_joint_index = (10, -4, 2)
        expected_errors_by_joint_index = (10, 4, 2)

        for leg in EXPECTED_LEG_CONFIG:
            with self.subTest(leg=leg):
                calibration = self.load_calibration(leg)
                ticks = self.visual_zero_ticks(calibration)

                for item, offset in zip(
                    calibration,
                    offsets_by_joint_index,
                ):
                    ticks[item.servo_id] = (
                        ticks[item.servo_id] + offset
                    ) % 4096

                errors = visual_zero_errors_ticks(
                    calibration,
                    ticks,
                )

                self.assertEqual(
                    errors,
                    {
                        item.joint_name: expected_error
                        for item, expected_error in zip(
                            calibration,
                            expected_errors_by_joint_index,
                        )
                    },
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)
