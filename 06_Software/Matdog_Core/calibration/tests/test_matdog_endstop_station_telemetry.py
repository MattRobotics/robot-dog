import ast
from pathlib import Path
import struct
import sys
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALIBRATION_DIR))

import matdog_endstop_station_telemetry as telemetry


class FakeCommand:
    def __init__(self, command_id: bytes):
        self.command_id = command_id

    def get_command_id(self):
        return memoryview(self.command_id)


class FakeLastCommand:
    def __init__(self, command_id: bytes, result):
        self.command = FakeCommand(command_id)
        self.result = result

    def get_command(self):
        return self.command

    def get_result(self):
        return self.result


class FakeMotor:
    def __init__(
        self,
        *,
        motor_id=13,
        stamp_ns=100,
        system_stamp_ns=200,
        app_start_id=7,
        present=2048,
        speed=12,
        current=3,
        status=0,
        torque=0,
        goal=2048,
        command_id=b"",
        result=None,
    ):
        if result is None:
            result = telemetry.st3215.CommandResult.CR_PROCESSING

        state = bytearray(0x47)
        state[telemetry.RAM_TORQUE_ENABLE] = torque
        state[telemetry.RAM_STATUS] = status

        struct.pack_into(
            "<H",
            state,
            telemetry.RAM_GOAL_POSITION,
            goal,
        )
        struct.pack_into(
            "<H",
            state,
            telemetry.RAM_PRESENT_POSITION,
            present,
        )
        struct.pack_into(
            "<H",
            state,
            telemetry.RAM_PRESENT_SPEED,
            speed,
        )
        struct.pack_into(
            "<H",
            state,
            telemetry.RAM_PRESENT_CURRENT,
            current,
        )

        self.motor_id = motor_id
        self.stamp_ns = stamp_ns
        self.system_stamp_ns = system_stamp_ns
        self.app_start_id = app_start_id
        self.state = bytes(state)
        self.last_command = FakeLastCommand(
            command_id,
            result,
        )

    def get_id(self):
        return self.motor_id

    def get_monotonic_stamp_ns(self):
        return self.stamp_ns

    def get_system_stamp_ns(self):
        return self.system_stamp_ns

    def get_app_start_id(self):
        return self.app_start_id

    def get_state(self):
        return memoryview(self.state)

    def get_last_command(self):
        return self.last_command


class FakeBusInfo:
    def __init__(self, serial):
        self.serial = serial

    def get_serial_number(self):
        return self.serial


class FakeBus:
    def __init__(self, serial, motors):
        self.info = FakeBusInfo(serial)
        self.motors = motors

    def get_bus(self):
        return self.info

    def get_motors(self):
        return self.motors


class FakeInference:
    def __init__(self, buses):
        self.buses = buses

    def get_buses(self):
        return self.buses


class StationTelemetryTests(unittest.TestCase):
    def test_find_and_parse_motor_snapshot(self):
        motor = FakeMotor(
            motor_id=13,
            stamp_ns=123456,
            system_stamp_ns=456789,
            app_start_id=11,
            present=2050,
            speed=19,
            current=7,
            status=0,
            torque=1,
            goal=2060,
            command_id=b"abcdefgh",
            result=telemetry.st3215.CommandResult.CR_SUCCESS,
        )
        state = FakeInference([
            FakeBus("wrong", [FakeMotor(motor_id=13)]),
            FakeBus("MATDOG-BUS", [motor]),
        ])

        found = telemetry.find_motor_reader(
            state,
            "MATDOG-BUS",
            13,
        )
        self.assertIs(found, motor)

        snapshot = telemetry.parse_motor_snapshot(found)

        self.assertEqual(snapshot.motor_id, 13)
        self.assertEqual(snapshot.monotonic_stamp_ns, 123456)
        self.assertEqual(snapshot.system_stamp_ns, 456789)
        self.assertEqual(snapshot.app_start_id, 11)
        self.assertEqual(snapshot.present_tick, 2050)
        self.assertEqual(snapshot.present_speed_raw, 19)
        self.assertEqual(snapshot.current_raw, 7)
        self.assertEqual(snapshot.error_status, 0)
        self.assertTrue(snapshot.torque_enabled)
        self.assertEqual(snapshot.goal_tick, 2060)
        self.assertEqual(
            snapshot.last_command_id,
            b"abcdefgh",
        )
        self.assertEqual(
            snapshot.last_command_result,
            telemetry.st3215.CommandResult.CR_SUCCESS,
        )

    def test_missing_last_command_is_valid_initial_state(self):
        class FakeMotorWithoutLastCommand(FakeMotor):
            def get_last_command(self):
                return None

        snapshot = telemetry.parse_motor_snapshot(
            FakeMotorWithoutLastCommand(
                stamp_ns=500,
                present=2048,
            )
        )

        self.assertEqual(snapshot.last_command_id, b"")
        self.assertIsNone(snapshot.last_command_result)

    def test_missing_command_body_is_valid_initial_state(self):
        class FakeLastCommandWithoutCommand:
            def get_command(self):
                return None

            def get_result(self):
                return telemetry.st3215.CommandResult.CR_PROCESSING

        class FakeMotorWithoutCommandBody(FakeMotor):
            def get_last_command(self):
                return FakeLastCommandWithoutCommand()

        snapshot = telemetry.parse_motor_snapshot(
            FakeMotorWithoutCommandBody(
                stamp_ns=600,
                present=2048,
            )
        )

        self.assertEqual(snapshot.last_command_id, b"")
        self.assertIsNone(snapshot.last_command_result)

    def test_detector_sample_preserves_motor_timestamp(self):
        snapshot = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=900,
                present=2049,
                current=5,
                status=2,
            )
        )

        sample = telemetry.detector_sample(snapshot)

        self.assertEqual(sample.monotonic_stamp_ns, 900)
        self.assertEqual(sample.present_tick, 2049)
        self.assertEqual(sample.current_raw, 5)
        self.assertEqual(sample.error_status, 2)

    def test_barrier_requires_success_then_newer_sample(self):
        command_id = b"\x01\x00\x00\x00\x00\x00\x00\x00"
        barrier = telemetry.CommandBarrier(
            command_id=command_id,
            motor_id=13,
        )

        processing = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=100,
                command_id=command_id,
                result=telemetry.st3215.CommandResult.CR_PROCESSING,
            )
        )
        success = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=200,
                command_id=command_id,
                result=telemetry.st3215.CommandResult.CR_SUCCESS,
            )
        )
        same_stamp = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=200,
                command_id=command_id,
                result=telemetry.st3215.CommandResult.CR_SUCCESS,
            )
        )
        fresh = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=201,
                command_id=command_id,
                result=telemetry.st3215.CommandResult.CR_SUCCESS,
            )
        )

        self.assertEqual(
            barrier.observe(processing).state,
            telemetry.CommandBarrierState.WAIT_COMMAND_RESULT,
        )
        self.assertEqual(
            barrier.observe(success).state,
            telemetry.CommandBarrierState.WAIT_FRESH_MOTOR_SAMPLE,
        )
        self.assertEqual(
            barrier.observe(same_stamp).state,
            telemetry.CommandBarrierState.WAIT_FRESH_MOTOR_SAMPLE,
        )
        self.assertEqual(
            barrier.observe(fresh).state,
            telemetry.CommandBarrierState.READY,
        )

    def test_barrier_rejected_and_failed(self):
        command_id = b"12345678"

        rejected = telemetry.CommandBarrier(
            command_id=command_id,
            motor_id=13,
        )
        rejected_snapshot = telemetry.parse_motor_snapshot(
            FakeMotor(
                command_id=command_id,
                result=telemetry.st3215.CommandResult.CR_REJECTED,
            )
        )
        self.assertEqual(
            rejected.observe(rejected_snapshot).state,
            telemetry.CommandBarrierState.REJECTED,
        )

        failed = telemetry.CommandBarrier(
            command_id=command_id,
            motor_id=13,
        )
        failed_snapshot = telemetry.parse_motor_snapshot(
            FakeMotor(
                command_id=command_id,
                result=telemetry.st3215.CommandResult.CR_FAILED,
            )
        )
        self.assertEqual(
            failed.observe(failed_snapshot).state,
            telemetry.CommandBarrierState.FAILED,
        )

    def test_station_restart_is_hard_abort(self):
        command_id = b"ABCDEFGH"
        barrier = telemetry.CommandBarrier(
            command_id=command_id,
            motor_id=13,
        )

        first = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=100,
                app_start_id=7,
                command_id=command_id,
            )
        )
        restarted = telemetry.parse_motor_snapshot(
            FakeMotor(
                stamp_ns=101,
                app_start_id=8,
                command_id=command_id,
            )
        )

        barrier.observe(first)

        self.assertEqual(
            barrier.observe(restarted).state,
            telemetry.CommandBarrierState.HARD_ABORT,
        )

    def test_module_has_no_command_or_connection_capability(self):
        module_path = (
            CALIBRATION_DIR
            / "matdog_endstop_station_telemetry.py"
        )
        source = module_path.read_text(encoding="utf-8")
        tree = ast.parse(source)

        forbidden_calls = {
            "new_station_client",
            "send_commands",
            "send_motor_commands",
            "send_goal",
            "set_torque",
            "enqueue",
            "follow",
        }
        forbidden_attributes = {
            "ST3215WriteCommand",
            "ST3215SyncWriteCommand",
            "ST3215RegWriteCommand",
            "ST3215ActionCommand",
            "DriverCommand",
        }

        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Name):
                    self.assertNotIn(
                        node.func.id,
                        forbidden_calls,
                    )
                elif isinstance(node.func, ast.Attribute):
                    self.assertNotIn(
                        node.func.attr,
                        forbidden_calls,
                    )
                    self.assertNotIn(
                        node.func.attr,
                        forbidden_attributes,
                    )

        self.assertNotIn("--execute", source)
        self.assertNotIn("import serial", source)
        self.assertNotIn("from commands import", source)


if __name__ == "__main__":
    unittest.main()
