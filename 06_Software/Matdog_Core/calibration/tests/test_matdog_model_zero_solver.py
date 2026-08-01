import unittest

from matdog_model_zero_solver import (
    DIGITAL_HOME_TICK,
    LF_MODELS,
    circular_distance,
    solve_model_zero,
)


class ModelZeroSolverTests(unittest.TestCase):
    def test_exact_urdf_endpoints_recover_2048(self):
        cases = {
            "upper": (1451, 3442),
            "lower": (3095, 1621),
            "hip": (2560, 1536),
        }
        for name, (minimum, maximum) in cases.items():
            with self.subTest(name=name):
                result = solve_model_zero(
                    LF_MODELS[name], [minimum, minimum], [maximum, maximum]
                )
                self.assertTrue(result.accepted)
                self.assertEqual(result.estimated_zero_tick, DIGITAL_HOME_TICK)
                self.assertEqual(result.endpoint_disagreement_ticks, 0)

    def test_2026_08_01_upper_evidence_is_consistent(self):
        result = solve_model_zero(LF_MODELS["upper"], [1443, 1443], [3443, 3442])
        self.assertTrue(result.accepted)
        self.assertEqual(result.zero_from_minimum_tick, 2040)
        self.assertEqual(result.zero_from_maximum_tick, 2048)
        self.assertEqual(result.endpoint_disagreement_ticks, 8)
        self.assertEqual(result.estimated_zero_tick, 2044)

    def test_2026_08_01_lower_evidence_requires_recheck(self):
        result = solve_model_zero(LF_MODELS["lower"], [3094, 3092], [1664, 1666])
        self.assertFalse(result.accepted)
        self.assertEqual(result.zero_from_minimum_tick, 2046)
        self.assertEqual(result.zero_from_maximum_tick, 2092)
        self.assertEqual(result.endpoint_disagreement_ticks, 46)

    def test_2026_08_01_hip_evidence_requires_stronger_max_recheck(self):
        result = solve_model_zero(LF_MODELS["hip"], [2530, 2530], [1595, 1595])
        self.assertFalse(result.accepted)
        self.assertEqual(result.zero_from_minimum_tick, 2018)
        self.assertEqual(result.zero_from_maximum_tick, 2107)
        self.assertEqual(result.endpoint_disagreement_ticks, 89)
        self.assertLessEqual(result.shift_from_digital_home_ticks, 96)

    def test_wrap_distance(self):
        self.assertEqual(circular_distance(4092, 8), 12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
