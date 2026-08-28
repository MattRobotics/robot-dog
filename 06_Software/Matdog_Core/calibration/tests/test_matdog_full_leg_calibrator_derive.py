"""q0 derivation and cross-check tests for Full Leg Calibrator V1.

The properties under test:

* ``manual_pose_q0_candidate`` and ``derived_q0_final`` stay separate objects;
* q0 is never forced, rounded or defaulted to RAW 2048;
* encoder direction is derived from measurement, not assumed from the leg;
* disagreement produces a structured FAIL, never an average and never an
  EEPROM write.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_joint_math import TICKS_PER_RAD, normalize_tick  # noqa: E402
from matdog_full_leg_calibrator_derive import (  # noqa: E402
    RAW_ELECTRICAL_CENTER,
    CrossCheckStatus,
    DerivationStatus,
    EndpointMeasurement,
    Q0Status,
    capture_manual_q0,
    cross_check_q0,
    derive_q0,
)


def make_samples(centre: int, spread: int = 2, count: int = 64) -> list[int]:
    """Deterministic sample cloud centred on `centre` with a bounded spread."""
    return [normalize_tick(centre + (i % (spread + 1)) - spread // 2) for i in range(count)]


class TestManualQ0Capture(unittest.TestCase):
    def test_stable_capture_is_accepted(self):
        candidate = capture_manual_q0("lf_hip_joint", 13, "M22", make_samples(2051))
        self.assertIs(candidate.status, Q0Status.OK)
        self.assertTrue(candidate.ok)
        self.assertEqual(candidate.sample_count, 64)

    def test_q0_is_never_rounded_to_the_electrical_centre(self):
        """A joint sitting 40 ticks off 2048 must report 40 ticks, not zero."""
        candidate = capture_manual_q0("lf_hip_joint", 13, "M22", make_samples(2088, spread=0))
        self.assertEqual(candidate.median_tick, 2088)
        self.assertNotEqual(candidate.median_tick, RAW_ELECTRICAL_CENTER)
        self.assertEqual(candidate.residual_from_center_ticks, 40)
        self.assertAlmostEqual(candidate.residual_from_center_deg, 40 * 360 / 4096, places=6)

    def test_spline_scale_residual_is_plausible_not_rejected(self):
        """~+/-5 deg (~57 ticks) of spline-indexed residual is a legitimate outcome."""
        for offset in (-57, -30, 0, 30, 57):
            with self.subTest(offset=offset):
                candidate = capture_manual_q0(
                    "j", 13, "U", make_samples(RAW_ELECTRICAL_CENTER + offset, spread=0)
                )
                self.assertIs(candidate.status, Q0Status.OK)

    def test_unstable_samples_are_rejected(self):
        candidate = capture_manual_q0("j", 13, "U", make_samples(2048, spread=40))
        self.assertIs(candidate.status, Q0Status.UNSTABLE_SAMPLES)
        self.assertFalse(candidate.ok)

    def test_insufficient_samples_are_rejected(self):
        candidate = capture_manual_q0("j", 13, "U", [2048, 2049, 2050])
        self.assertIs(candidate.status, Q0Status.INSUFFICIENT_SAMPLES)

    def test_implausible_residual_is_rejected(self):
        """A joint 90 deg from the pose means the robot is not in the CAD pose."""
        candidate = capture_manual_q0("j", 13, "U", make_samples(3072, spread=0))
        self.assertIs(candidate.status, Q0Status.IMPLAUSIBLE_RESIDUAL)

    def test_torque_on_invalidates_the_capture(self):
        candidate = capture_manual_q0(
            "j", 13, "U", make_samples(2050), torque_off_verified=False
        )
        self.assertFalse(candidate.ok)

    def test_capture_across_the_wrap_boundary_reports_a_real_spread(self):
        """A pose straddling 4095/0 must not report a ~4095 spread."""
        samples = [4094, 4095, 0, 1, 2] * 13
        candidate = capture_manual_q0("j", 13, "U", samples)
        self.assertEqual(candidate.median_tick, 0)
        self.assertEqual(candidate.spread_ticks, 4)

    def test_candidate_is_tagged_as_not_final_q0(self):
        payload = capture_manual_q0("j", 13, "U", make_samples(2050)).as_dict()
        self.assertEqual(payload["kind"], "manual_pose_q0_candidate")
        self.assertEqual(payload["promotion"], "NOT_FINAL_Q0")


class TestDerivedQ0(unittest.TestCase):
    """Synthetic joint: q0 at 2060, direction +1, unity scale."""

    def _endpoints(self, q0=2060, direction=1, scale=1.0,
                   q_min=-0.785398163397, q_max=0.785398163397):
        min_tick = normalize_tick(round(q0 + direction * q_min * TICKS_PER_RAD * scale))
        max_tick = normalize_tick(round(q0 + direction * q_max * TICKS_PER_RAD * scale))
        return (
            EndpointMeasurement("min", min_tick, min_tick, min_tick, 0, q_min),
            EndpointMeasurement("max", max_tick, max_tick, max_tick, 0, q_max),
        )

    def test_recovers_q0_and_direction(self):
        low, high = self._endpoints()
        result = derive_q0("lf_hip_joint", 13, low, high)
        self.assertIs(result.status, DerivationStatus.OK)
        self.assertEqual(result.direction, 1)
        self.assertAlmostEqual(result.q0_tick, 2060, delta=1)
        self.assertAlmostEqual(result.scale, 1.0, places=3)

    def test_recovers_negative_direction_without_special_casing(self):
        low, high = self._endpoints(direction=-1)
        result = derive_q0("rf_hip_joint", 23, low, high)
        self.assertIs(result.status, DerivationStatus.OK)
        self.assertEqual(result.direction, -1)
        self.assertAlmostEqual(result.q0_tick, 2060, delta=1)

    def test_direction_is_derived_not_inherited_from_the_leg(self):
        """The same leg label yields either sign depending on the measurement."""
        for direction in (1, -1):
            with self.subTest(direction=direction):
                low, high = self._endpoints(direction=direction)
                result = derive_q0("lf_hip_joint", 13, low, high)
                self.assertEqual(result.direction, direction)

    def test_endpoint_residuals_are_reported(self):
        low, high = self._endpoints()
        result = derive_q0("lf_hip_joint", 13, low, high)
        self.assertLess(abs(result.residual_min_deg), 0.2)
        self.assertLess(abs(result.residual_max_deg), 0.2)

    def test_endpoint_order_contradiction_fails_closed(self):
        low, high = self._endpoints()
        swapped_min = EndpointMeasurement("min", low.contact_tick, 0, 0, 0, 0.5)
        swapped_max = EndpointMeasurement("max", high.contact_tick, 0, 0, 0, -0.5)
        result = derive_q0("j", 13, swapped_min, swapped_max)
        self.assertIs(result.status, DerivationStatus.ENDPOINT_ORDER_CONTRADICTION)
        self.assertIsNone(result.q0_tick)

    def test_degenerate_span_fails_closed(self):
        both = EndpointMeasurement("min", 2048, 2048, 2048, 0, -0.7)
        other = EndpointMeasurement("max", 2048, 2048, 2048, 0, 0.7)
        result = derive_q0("j", 13, both, other)
        self.assertIs(result.status, DerivationStatus.DEGENERATE_SPAN)

    def test_affine_solution_inconsistent_with_urdf_fails_closed(self):
        """A measured span far from the URDF span is a reported conflict."""
        low, high = self._endpoints(scale=1.6)
        result = derive_q0("j", 13, low, high)
        self.assertIs(result.status, DerivationStatus.SCALE_OUT_OF_RANGE)
        self.assertIsNone(result.q0_tick)
        self.assertIn("disagree", " ".join(result.notes))

    def test_scale_slightly_off_unity_is_still_accepted(self):
        low, high = self._endpoints(scale=1.05)
        result = derive_q0("j", 13, low, high)
        self.assertIs(result.status, DerivationStatus.OK)

    def test_derived_q0_stays_in_the_unsigned_domain(self):
        low, high = self._endpoints(q0=2060)
        result = derive_q0("j", 13, low, high)
        self.assertGreaterEqual(result.q0_tick, 0)
        self.assertLessEqual(result.q0_tick, 4095)

    def test_wide_span_near_half_revolution_is_refused(self):
        """A span approaching 2048 ticks cannot be signed unambiguously."""
        low = EndpointMeasurement("min", 1000, 1000, 1000, 0, -1.57)
        high = EndpointMeasurement("max", 3047, 3047, 3047, 0, 1.57)
        result = derive_q0("j", 13, low, high)
        self.assertIn(
            result.status,
            (DerivationStatus.WRAP_DOMAIN_VIOLATION, DerivationStatus.SCALE_OUT_OF_RANGE),
        )

    def test_derived_payload_requires_explicit_promotion(self):
        low, high = self._endpoints()
        payload = derive_q0("j", 13, low, high).as_dict()
        self.assertEqual(payload["kind"], "derived_q0_final_candidate")
        self.assertEqual(payload["promotion"], "REQUIRES_EXPLICIT_PROMOTION_GATE")

    def test_notes_confirm_position_offset_is_untouched(self):
        low, high = self._endpoints()
        result = derive_q0("j", 13, low, high)
        self.assertIn("PositionOffset remains 0", " ".join(result.notes))


class TestQ0CrossCheck(unittest.TestCase):
    def _pair(self, manual_centre: int, derived_q0: int = 2060):
        manual = capture_manual_q0("j", 13, "U", make_samples(manual_centre, spread=0))
        low = EndpointMeasurement(
            "min",
            normalize_tick(round(derived_q0 - 0.785398163397 * TICKS_PER_RAD)),
            0, 0, 0, -0.785398163397,
        )
        high = EndpointMeasurement(
            "max",
            normalize_tick(round(derived_q0 + 0.785398163397 * TICKS_PER_RAD)),
            0, 0, 0, 0.785398163397,
        )
        return manual, derive_q0("j", 13, low, high)

    def test_unvalidated_tolerance_blocks_certification(self):
        """With the tolerance still CHARACTERIZATION_REQUIRED, nothing may pass."""
        manual, derived = self._pair(2060)
        result = cross_check_q0(manual, derived, tolerance_ticks=None)
        self.assertIs(result.status, CrossCheckStatus.BLOCKED_TOLERANCE_UNVALIDATED)
        self.assertFalse(result.ok)
        self.assertIn("CHARACTERIZATION_REQUIRED", result.diagnosis)

    def test_agreement_within_a_supplied_tolerance(self):
        manual, derived = self._pair(2062)
        result = cross_check_q0(manual, derived, tolerance_ticks=16)
        self.assertIs(result.status, CrossCheckStatus.AGREE)
        self.assertEqual(result.disagreement_ticks, 2)

    def test_disagreement_fails_closed_with_a_diagnosis(self):
        manual, derived = self._pair(2160)
        result = cross_check_q0(manual, derived, tolerance_ticks=16)
        self.assertIs(result.status, CrossCheckStatus.DISAGREE)
        self.assertEqual(result.disagreement_ticks, 100)
        for phrase in ("Do NOT average", "PositionOffset", "URDF"):
            self.assertIn(phrase, result.diagnosis)

    def test_disagreement_never_produces_a_blended_value(self):
        manual, derived = self._pair(2160)
        result = cross_check_q0(manual, derived, tolerance_ticks=16)
        self.assertFalse(hasattr(result, "blended_q0"))
        self.assertFalse(result.ok)

    def test_unusable_manual_candidate_blocks_the_check(self):
        manual = capture_manual_q0("j", 13, "U", make_samples(2048, spread=40))
        _, derived = self._pair(2060)
        result = cross_check_q0(manual, derived, tolerance_ticks=16)
        self.assertIs(result.status, CrossCheckStatus.MISSING_INPUT)

    def test_missing_inputs_block_the_check(self):
        self.assertIs(
            cross_check_q0(None, None, 16).status, CrossCheckStatus.MISSING_INPUT
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
