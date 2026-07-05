"""Test automatici MATDOG per conversione encoder <-> radianti URDF."""

import math
import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_joint_math import (
    ENCODER_MODULUS,
    RAD_PER_TICK,
    encoder_round_trip_error,
    encoder_to_joint_rad,
    joint_rad_to_encoder,
    normalize_tick,
    signed_tick_delta,
)


class TestMatdogJointMath(unittest.TestCase):
    def test_normalize_tick(self):
        self.assertEqual(normalize_tick(0), 0)
        self.assertEqual(normalize_tick(4095), 4095)
        self.assertEqual(normalize_tick(4096), 0)
        self.assertEqual(normalize_tick(4097), 1)
        self.assertEqual(normalize_tick(-1), 4095)

    def test_signed_delta_without_wrap(self):
        self.assertEqual(signed_tick_delta(1200, 1000), 200)
        self.assertEqual(signed_tick_delta(1000, 1200), -200)

    def test_signed_delta_with_wrap(self):
        self.assertEqual(signed_tick_delta(0, 4095), 1)
        self.assertEqual(signed_tick_delta(1, 4095), 2)
        self.assertEqual(signed_tick_delta(4095, 0), -1)
        self.assertEqual(signed_tick_delta(4094, 1), -3)

    def test_zero_is_zero_rad_for_both_directions(self):
        for zero_tick in (0, 155, 1268, 2048, 2936, 3979, 4095):
            for direction in (-1, 1):
                self.assertAlmostEqual(
                    encoder_to_joint_rad(zero_tick, zero_tick, direction),
                    0.0,
                    places=12,
                )

    def test_encoder_to_rad_positive_direction(self):
        angle = encoder_to_joint_rad(20, 0, 1)
        self.assertAlmostEqual(angle, 20 * RAD_PER_TICK, places=12)

    def test_encoder_to_rad_negative_direction(self):
        angle = encoder_to_joint_rad(20, 0, -1)
        self.assertAlmostEqual(angle, -20 * RAD_PER_TICK, places=12)

    def test_rad_to_encoder_positive_direction(self):
        target = joint_rad_to_encoder(20 * RAD_PER_TICK, 0, 1)
        self.assertEqual(target, 20)

    def test_rad_to_encoder_negative_direction(self):
        target = joint_rad_to_encoder(20 * RAD_PER_TICK, 0, -1)
        self.assertEqual(target, ENCODER_MODULUS - 20)

    def test_round_trip_representative_encoder_values(self):
        zero_ticks = (0, 15, 155, 1009, 1268, 1483, 2048, 2936, 3979, 4095)
        present_ticks = (0, 1, 15, 155, 1009, 1268, 1483, 2048, 2936, 3979, 4094, 4095)

        for direction in (-1, 1):
            for zero_tick in zero_ticks:
                for present_tick in present_ticks:
                    with self.subTest(
                        direction=direction,
                        zero_tick=zero_tick,
                        present_tick=present_tick,
                    ):
                        error = encoder_round_trip_error(
                            present_tick,
                            zero_tick,
                            direction,
                        )
                        self.assertLessEqual(abs(error), 1)

    def test_invalid_direction_is_rejected(self):
        with self.assertRaises(ValueError):
            encoder_to_joint_rad(100, 100, 0)

        with self.assertRaises(ValueError):
            joint_rad_to_encoder(0.1, 100, 2)

    def test_non_finite_angle_is_rejected(self):
        with self.assertRaises(ValueError):
            joint_rad_to_encoder(math.nan, 100, 1)

        with self.assertRaises(ValueError):
            joint_rad_to_encoder(math.inf, 100, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
