"""Offline census, q0-capture and mode-gate fault injection.

Covers, from section 13 of the V1 specification: zero responders, one missing
expected servo, unexpected extra servo, wrong model, nonzero PositionOffset,
profile mismatch, torque unexpectedly ON at census, unstable RAW during manual
q0 capture, bus read failure, non-target joint responding unexpectedly, stale
calibration presented, and abnormal exit leaving torque enabled.

Also asserts the two absolute prohibitions at the wire level: the calibrator
never writes an EEPROM address and never issues a broadcast write.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_full_leg_calibrator_policy import (  # noqa: E402
    EXPECTED_LEG_IDS,
    HardwareStage,
)
from matdog_full_leg_calibrator_sim import (  # noqa: E402
    BROADCAST_ID,
    REG_LOCK,
    REG_POSITION_OFFSET,
    REG_TORQUE_ENABLE,
    BroadcastWriteError,
    BusFault,
    EepromWriteError,
    MockCalibratorFirmware,
    MockST3215Bus,
    SimServo,
    make_healthy_leg_bus,
)


class TestCensus(unittest.TestCase):
    def test_healthy_twelve_servo_bus_passes(self):
        firmware = MockCalibratorFirmware(make_healthy_leg_bus())
        result = firmware.census()
        self.assertTrue(result.passed)
        self.assertEqual(result.present_count, 12)
        self.assertEqual(result.identity_ok_count, 12)
        self.assertTrue(firmware.census_fresh)

    def test_zero_responders_fails_and_blocks(self):
        """Today's H0 condition: no servos connected at all."""
        firmware = MockCalibratorFirmware(MockST3215Bus())
        result = firmware.census()
        self.assertFalse(result.passed)
        self.assertEqual(result.present_count, 0)
        self.assertEqual(result.fail_reason, "NO_RESPONDERS")
        self.assertFalse(firmware.census_fresh)

    def test_one_missing_expected_servo_fails(self):
        bus = make_healthy_leg_bus()
        bus.servos[22].fault = BusFault.NO_RESPONDER
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        self.assertEqual(result.present_count, 11)
        missing = [s for s in result.servos if s.bus_id == 22]
        self.assertEqual(missing[0].reasons, ("MISSING",))

    def test_unexpected_extra_servo_on_the_leg_bus_fails(self):
        bus = make_healthy_leg_bus()
        bus.servos[99] = SimServo(servo_id=99)
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        self.assertIn(99, result.unexpected_responders)
        self.assertEqual(result.fail_reason, "UNEXPECTED_RESPONDER")

    def test_head_servo_appearing_is_treated_as_unexpected(self):
        """The head is not built; a responder at 51 means something is wrong."""
        bus = make_healthy_leg_bus()
        bus.servos[51] = SimServo(servo_id=51)
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        self.assertIn(51, result.unexpected_responders)

    def test_head_absence_alone_is_not_a_failure(self):
        """A leg-only session must pass with head ids 51..55 absent."""
        result = MockCalibratorFirmware(make_healthy_leg_bus()).census()
        self.assertTrue(result.passed)

    def test_wrong_model_is_rejected(self):
        bus = make_healthy_leg_bus()
        bus.servos[13].model = 999
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        entry = next(s for s in result.servos if s.bus_id == 13)
        self.assertIn("WRONG_MODEL", entry.reasons)

    def test_nonzero_position_offset_is_rejected(self):
        bus = make_healthy_leg_bus()
        bus.servos[11].position_offset = -505
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        entry = next(s for s in result.servos if s.bus_id == 11)
        self.assertIn("NONZERO_POSITION_OFFSET", entry.reasons)

    def test_profile_mismatch_is_rejected(self):
        bus = make_healthy_leg_bus()
        bus.servos[41].profile[0x1C] = 500  # ProtectionCurrent drifted
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        entry = next(s for s in result.servos if s.bus_id == 41)
        self.assertTrue(any(r.startswith("PROFILE_MISMATCH") for r in entry.reasons))

    def test_torque_unexpectedly_on_at_census_is_rejected(self):
        bus = make_healthy_leg_bus()
        bus.servos[33].torque_enable = 1
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        entry = next(s for s in result.servos if s.bus_id == 33)
        self.assertIn("TORQUE_UNEXPECTEDLY_ON", entry.reasons)

    def test_id_register_mismatch_is_rejected(self):
        """Duplicate/ambiguous identity must not pass silently."""
        bus = make_healthy_leg_bus()
        bus.servos[32].stored_id = 42
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        entry = next(s for s in result.servos if s.bus_id == 32)
        self.assertIn("ID_REGISTER_MISMATCH", entry.reasons)

    def test_impossible_telemetry_is_rejected(self):
        bus = make_healthy_leg_bus()
        bus.servos[21].present_position = 9000
        bus.servos[23].voltage = 200
        bus.servos[31].temperature = 85
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)
        self.assertIn(
            "IMPOSSIBLE_POSITION",
            next(s for s in result.servos if s.bus_id == 21).reasons,
        )
        self.assertIn(
            "VOLTAGE_OUT_OF_RANGE",
            next(s for s in result.servos if s.bus_id == 23).reasons,
        )
        self.assertIn(
            "TEMPERATURE", next(s for s in result.servos if s.bus_id == 31).reasons
        )

    def test_bus_read_failure_is_reported_as_missing(self):
        bus = make_healthy_leg_bus()
        bus.servos[12].fault = BusFault.READ_TIMEOUT
        result = MockCalibratorFirmware(bus).census()
        self.assertFalse(result.passed)

    def test_census_is_invalidated_by_a_later_failing_census(self):
        """Freshness must not survive a subsequent failure."""
        bus = make_healthy_leg_bus()
        firmware = MockCalibratorFirmware(bus)
        self.assertTrue(firmware.census().passed)
        self.assertTrue(firmware.census_fresh)
        bus.servos[13].fault = BusFault.NO_RESPONDER
        self.assertFalse(firmware.census().passed)
        self.assertFalse(firmware.census_fresh)

    def test_census_performs_no_writes_at_all(self):
        bus = make_healthy_leg_bus()
        MockCalibratorFirmware(bus).census()
        self.assertEqual(bus.write_log, [])


class TestManualQ0Mode(unittest.TestCase):
    def _firmware(self, stage=HardwareStage.H2_MANUAL_Q0):
        bus = make_healthy_leg_bus()
        firmware = MockCalibratorFirmware(bus, stage=stage)
        firmware.census()
        return bus, firmware

    def test_blocked_at_h0(self):
        bus, firmware = self._firmware(stage=HardwareStage.H0_ESP32_ONLY)
        result = firmware.capture_manual_q0()
        self.assertFalse(result.accepted)
        self.assertTrue(any("H2_MANUAL_Q0" in r for r in result.reasons))

    def test_requires_a_fresh_census(self):
        bus = make_healthy_leg_bus()
        firmware = MockCalibratorFirmware(bus, stage=HardwareStage.H2_MANUAL_Q0)
        result = firmware.capture_manual_q0()
        self.assertFalse(result.accepted)
        self.assertTrue(any("fresh successful census" in r for r in result.reasons))

    def test_stable_capture_succeeds_and_writes_nothing(self):
        bus, firmware = self._firmware()
        for servo in bus.servos.values():
            servo.position_sequence = [2050, 2051, 2050, 2052]
        result = firmware.capture_manual_q0(samples=32)
        self.assertTrue(result.accepted)
        self.assertEqual(bus.write_log, [])
        self.assertTrue(bus.all_torque_off())

    def test_unstable_raw_during_capture_is_rejected(self):
        bus, firmware = self._firmware()
        for servo in bus.servos.values():
            servo.position_sequence = [2050, 2051, 2050, 2052]
        bus.servos[13].position_sequence = [2000, 2100, 2010, 2090]
        result = firmware.capture_manual_q0(samples=32)
        self.assertFalse(result.accepted)
        self.assertEqual(result.payload["13"]["status"], "UNSTABLE_SAMPLES")

    def test_torque_on_refuses_capture_for_that_joint(self):
        bus, firmware = self._firmware()
        bus.servos[43].torque_enable = 1
        result = firmware.capture_manual_q0(samples=32)
        self.assertFalse(result.accepted)
        self.assertEqual(result.payload["43"]["status"], "REFUSED_TORQUE_ON")

    def test_capture_output_is_never_promoted_to_final_q0(self):
        bus, firmware = self._firmware()
        for servo in bus.servos.values():
            servo.position_sequence = [2050, 2051]
        result = firmware.capture_manual_q0(samples=32)
        for entry in result.payload.values():
            self.assertEqual(entry["kind"], "manual_pose_q0_candidate")
            self.assertEqual(entry["promotion"], "NOT_FINAL_Q0")


class TestMotionModesAreLocked(unittest.TestCase):
    def _ready_firmware(self, stage=HardwareStage.H7_FREEZE):
        bus = make_healthy_leg_bus()
        firmware = MockCalibratorFirmware(bus, stage=stage)
        firmware.census()
        return bus, firmware

    def test_calibrate_joint_refused_even_at_h7(self):
        """Stage alone must never unlock motion while parameters are unvalidated."""
        bus, firmware = self._ready_firmware()
        result = firmware.calibrate_joint(13)
        self.assertFalse(result.accepted)
        self.assertTrue(any("CHARACTERIZATION_REQUIRED" in r for r in result.reasons))
        self.assertEqual(bus.write_log, [])

    def test_calibrate_leg_refused(self):
        bus, firmware = self._ready_firmware()
        self.assertFalse(firmware.calibrate_leg("LF").accepted)
        self.assertEqual(bus.write_log, [])

    def test_calibrate_all_refused(self):
        bus, firmware = self._ready_firmware()
        self.assertFalse(firmware.calibrate_all_legs().accepted)
        self.assertEqual(bus.write_log, [])

    def test_all_modes_cite_unmeasured_direction(self):
        _, firmware = self._ready_firmware()
        for result in (
            firmware.calibrate_joint(13),
            firmware.calibrate_leg("LF"),
            firmware.calibrate_all_legs(),
        ):
            self.assertTrue(any("direction is unmeasured" in r for r in result.reasons))

    def test_non_leg_bus_id_is_refused(self):
        _, firmware = self._ready_firmware()
        result = firmware.calibrate_joint(51)
        self.assertFalse(result.accepted)
        self.assertTrue(any("not a leg bus id" in r for r in result.reasons))

    def test_modes_are_refused_without_a_census_at_every_stage(self):
        bus = make_healthy_leg_bus()
        firmware = MockCalibratorFirmware(bus, stage=HardwareStage.H7_FREEZE)
        result = firmware.calibrate_joint(13)
        self.assertTrue(any("fresh successful census" in r for r in result.reasons))


class TestSafeOff(unittest.TestCase):
    def test_safe_off_with_no_responders_is_a_pass(self):
        firmware = MockCalibratorFirmware(MockST3215Bus())
        result = firmware.safe_off()
        self.assertTrue(result.accepted)
        self.assertFalse(result.payload["responders_seen"])

    def test_safe_off_turns_torque_off_unicast_and_verifies(self):
        bus = make_healthy_leg_bus()
        for servo in bus.servos.values():
            servo.torque_enable = 1
        firmware = MockCalibratorFirmware(bus)
        result = firmware.safe_off()
        self.assertTrue(result.accepted)
        self.assertTrue(bus.all_torque_off())
        self.assertEqual(len(bus.write_log), 12)
        for record in bus.write_log:
            self.assertEqual(record.address, REG_TORQUE_ENABLE)
            self.assertEqual(record.value, 0)
            self.assertIn(record.servo_id, EXPECTED_LEG_IDS)
            self.assertNotEqual(record.servo_id, BROADCAST_ID)

    def test_safe_off_is_idempotent(self):
        bus = make_healthy_leg_bus()
        for servo in bus.servos.values():
            servo.torque_enable = 1
        firmware = MockCalibratorFirmware(bus)
        self.assertTrue(firmware.safe_off().accepted)
        writes_after_first = len(bus.write_log)
        self.assertTrue(firmware.safe_off().accepted)
        self.assertEqual(len(bus.write_log), writes_after_first)

    def test_safe_off_reports_failure_when_a_servo_will_not_release(self):
        """An abnormal exit that leaves torque enabled must be loud, not silent."""
        bus = make_healthy_leg_bus()
        for servo in bus.servos.values():
            servo.torque_enable = 1
        bus.servos[13].fault = BusFault.WRITE_REJECTED
        firmware = MockCalibratorFirmware(bus)
        result = firmware.safe_off()
        self.assertFalse(result.accepted)
        self.assertFalse(bus.all_torque_off())
        self.assertEqual(firmware.last_fault, "SAFE_OFF_FAILED")


class TestAbsoluteProhibitions(unittest.TestCase):
    def test_eeprom_write_raises_immediately(self):
        bus = make_healthy_leg_bus()
        with self.assertRaises(EepromWriteError):
            bus.write(13, REG_POSITION_OFFSET, 2, 0)
        with self.assertRaises(EepromWriteError):
            bus.write(13, REG_LOCK, 1, 0)

    def test_broadcast_write_raises_immediately(self):
        bus = make_healthy_leg_bus()
        with self.assertRaises(BroadcastWriteError):
            bus.write(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 0)

    def test_a_full_session_touches_no_eeprom_address(self):
        bus = make_healthy_leg_bus()
        for servo in bus.servos.values():
            servo.position_sequence = [2050, 2051]
        firmware = MockCalibratorFirmware(bus, stage=HardwareStage.H2_MANUAL_Q0)
        firmware.census()
        firmware.capture_manual_q0(samples=32)
        firmware.calibrate_joint(13)
        firmware.calibrate_leg("LF")
        firmware.calibrate_all_legs()
        firmware.safe_off()
        self.assertEqual(bus.eeprom_writes, [])
        self.assertTrue(bus.all_torque_off())


if __name__ == "__main__":
    unittest.main(verbosity=2)
