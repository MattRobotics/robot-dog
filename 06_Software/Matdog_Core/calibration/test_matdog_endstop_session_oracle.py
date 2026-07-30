#!/usr/bin/env python3

from __future__ import annotations

import unittest

from matdog_endstop_session_oracle import (
    ContactClass,
    ContactObservation,
    Joint,
    OrderedLegSession,
    Side,
    classify_contact,
    probe_return_state,
)


class ContactOracleTests(unittest.TestCase):
    def test_v19_m13_is_early_stall_not_contact(self) -> None:
        observation = ContactObservation(
            present_tick=2405,
            target_tick=2416,
            inner_tick=2496,
            outer_guard_tick=2624,
            probe_sign=1,
            travel_ticks=357,
            progress_ticks=0,
            velocity_raw=0,
            current_raw=1,
            hard_current_abort_raw=200,
            persistence_samples=3,
        )
        self.assertEqual(classify_contact(observation), ContactClass.EARLY_STALL)

    def test_validated_m12_max_is_not_rejected_by_neutral_current(self) -> None:
        observation = ContactObservation(
            present_tick=3442,
            target_tick=3454,
            inner_tick=3378,
            outer_guard_tick=3506,
            probe_sign=1,
            travel_ticks=1394,
            progress_ticks=0,
            velocity_raw=0,
            current_raw=4,
            hard_current_abort_raw=200,
            persistence_samples=3,
        )
        self.assertEqual(
            classify_contact(observation),
            ContactClass.EXPECTED_CORRIDOR_STALL,
        )

    def test_current_rise_without_stall_is_free_motion(self) -> None:
        observation = ContactObservation(
            present_tick=3400,
            target_tick=3432,
            inner_tick=3378,
            outer_guard_tick=3506,
            probe_sign=1,
            travel_ticks=1352,
            progress_ticks=8,
            velocity_raw=20,
            current_raw=100,
            hard_current_abort_raw=200,
            persistence_samples=3,
        )
        self.assertEqual(classify_contact(observation), ContactClass.FREE_MOTION)

    def test_hard_current_abort_wins(self) -> None:
        observation = ContactObservation(
            present_tick=3442,
            target_tick=3454,
            inner_tick=3378,
            outer_guard_tick=3506,
            probe_sign=1,
            travel_ticks=1394,
            progress_ticks=0,
            velocity_raw=0,
            current_raw=200,
            hard_current_abort_raw=200,
            persistence_samples=3,
        )
        self.assertEqual(classify_contact(observation), ContactClass.HARD_ABORT)


class OrderedSessionTests(unittest.TestCase):
    def test_hip_cannot_be_armed_before_lower_pair(self) -> None:
        session = OrderedLegSession(imported_upper_checkpoint=True)
        session.upper_horizontal_validated = True
        session.lower_compact_validated = True
        session.fixture_validated = True
        with self.assertRaisesRegex(RuntimeError, "LOWER MIN\+MAX"):
            session.record_contact(Joint.HIP, Side.MIN, 2500)

    def test_lower_requires_upper_horizontal(self) -> None:
        session = OrderedLegSession(imported_upper_checkpoint=True)
        with self.assertRaisesRegex(RuntimeError, "UPPER horizontal"):
            session.record_contact(Joint.LOWER, Side.MIN, 1001)

    def test_complete_lf_resume_order_from_frozen_upper_checkpoint(self) -> None:
        session = OrderedLegSession(imported_upper_checkpoint=True)
        self.assertEqual(session.next_joint(), Joint.LOWER)
        session.upper_horizontal_validated = True
        session.record_contact(Joint.LOWER, Side.MIN, 1001)
        session.record_contact(Joint.LOWER, Side.MAX, 2475)
        self.assertEqual(session.next_joint(), Joint.HIP)
        session.lower_compact_validated = True
        session.fixture_validated = True
        session.record_contact(Joint.HIP, Side.MIN, 2500)
        session.record_contact(Joint.HIP, Side.MAX, 1600)
        self.assertIsNone(session.next_joint())

    def test_probe_return_handoff_is_strict(self) -> None:
        self.assertEqual(probe_return_state(2058), "STATIC_HANDOFF_ALLOWED")
        self.assertEqual(probe_return_state(2059), "ACTIVE_NUDGE_REQUIRED")
        self.assertEqual(
            probe_return_state(2065),
            "RECOVERY_FAILED_KEEP_PREREQUISITES",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
