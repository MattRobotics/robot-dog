import ast
from pathlib import Path
import struct
import sys
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALIBRATION_DIR))

import matdog_endstop_station_readonly_watch as watcher


class FakeMotor:
    def __init__(
        self,
        motor_id: int,
        stamp_ns: int,
        *,
        app_start_id: int = 9,
        torque: int = 0,
        status: int = 0,
    ):
        state = bytearray(0x47)
        state[
            watcher.parse_motor_snapshot.__globals__[
                "RAM_TORQUE_ENABLE"
            ]
        ] = torque
        state[
            watcher.parse_motor_snapshot.__globals__[
                "RAM_STATUS"
            ]
        ] = status

        struct.pack_into(
            "<H",
            state,
            watcher.parse_motor_snapshot.__globals__[
                "RAM_GOAL_POSITION"
            ],
            2048,
        )
        struct.pack_into(
            "<H",
            state,
            watcher.parse_motor_snapshot.__globals__[
                "RAM_PRESENT_POSITION"
            ],
            2048 + motor_id % 3,
        )
        struct.pack_into(
            "<H",
            state,
            watcher.parse_motor_snapshot.__globals__[
                "RAM_PRESENT_SPEED"
            ],
            0,
        )
        struct.pack_into(
            "<H",
            state,
            watcher.parse_motor_snapshot.__globals__[
                "RAM_PRESENT_CURRENT"
            ],
            motor_id % 4,
        )

        self.motor_id = motor_id
        self.stamp_ns = stamp_ns
        self.app_start_id = app_start_id
        self.state = bytes(state)

    def get_id(self):
        return self.motor_id

    def get_monotonic_stamp_ns(self):
        return self.stamp_ns

    def get_system_stamp_ns(self):
        return self.stamp_ns + 1000

    def get_app_start_id(self):
        return self.app_start_id

    def get_state(self):
        return memoryview(self.state)

    def get_last_command(self):
        return None


class FakeBusInfo:
    def __init__(self, serial: str):
        self.serial = serial

    def get_serial_number(self):
        return self.serial


class FakeBus:
    def __init__(self, serial: str, motors):
        self.info = FakeBusInfo(serial)
        self.motors = motors

    def get_bus(self):
        return self.info

    def get_motors(self):
        return self.motors


class FakeInference:
    def __init__(self, serial: str, motors):
        self.bus = FakeBus(serial, motors)

    def get_buses(self):
        return [self.bus]


def make_frame(
    stamp_ns: int,
    *,
    torque_motor: int | None = None,
    status_motor: int | None = None,
    app_start_id: int = 9,
):
    motors = []

    for motor_id in watcher.EXPECTED_MOTOR_IDS:
        motors.append(FakeMotor(
            motor_id,
            stamp_ns + motor_id,
            app_start_id=app_start_id,
            torque=int(motor_id == torque_motor),
            status=4 if motor_id == status_motor else 0,
        ))

    return FakeInference("MATDOG-BUS", motors)


def make_frame_from_stamps(
    stamps: dict[int, int],
    *,
    app_start_id: int = 9,
):
    motors = [
        FakeMotor(
            motor_id,
            stamps[motor_id],
            app_start_id=app_start_id,
        )
        for motor_id in watcher.EXPECTED_MOTOR_IDS
    ]

    return FakeInference("MATDOG-BUS", motors)


class ReadonlyWatcherTests(unittest.TestCase):
    def test_accumulator_builds_safe_report(self):
        accumulator = watcher.SnapshotAccumulator()

        accumulator.add_frame(
            make_frame(1000),
            "MATDOG-BUS",
        )
        accumulator.add_frame(
            make_frame(2000),
            "MATDOG-BUS",
        )

        report = accumulator.report(
            station_server="localhost",
            bus_serial="MATDOG-BUS",
            requested_frames=2,
            received_frames=2,
        )

        self.assertEqual(report.received_frames, 2)
        self.assertTrue(report.all_status_zero)
        self.assertTrue(report.all_torque_disabled)
        self.assertTrue(
            report.timestamps_strictly_increasing
        )
        self.assertFalse(report.command_api_available)
        self.assertFalse(report.motor_commands_sent)
        self.assertFalse(report.eeprom_writes_sent)
        self.assertEqual(len(report.motors), 12)

    def test_report_surfaces_torque_and_status(self):
        accumulator = watcher.SnapshotAccumulator()

        accumulator.add_frame(
            make_frame(
                1000,
                torque_motor=13,
                status_motor=21,
            ),
            "MATDOG-BUS",
        )

        report = accumulator.report(
            station_server="localhost",
            bus_serial="MATDOG-BUS",
            requested_frames=1,
            received_frames=1,
        )

        self.assertFalse(report.all_status_zero)
        self.assertFalse(report.all_torque_disabled)

    def test_missing_motor_is_rejected(self):
        motors = [
            FakeMotor(motor_id, 1000 + motor_id)
            for motor_id in watcher.EXPECTED_MOTOR_IDS
            if motor_id != 43
        ]

        accumulator = watcher.SnapshotAccumulator()

        with self.assertRaisesRegex(
            watcher.ReadonlyWatchError,
            "Set motori inatteso",
        ):
            accumulator.add_frame(
                FakeInference("MATDOG-BUS", motors),
                "MATDOG-BUS",
            )

    def test_aggregate_frames_accept_duplicate_motor_stamps(self):
        accumulator = watcher.SnapshotAccumulator()
        stamps = {
            motor_id: 1000 + motor_id
            for motor_id in watcher.EXPECTED_MOTOR_IDS
        }

        accumulator.add_frame(
            make_frame_from_stamps(stamps),
            "MATDOG-BUS",
        )

        for motor_id in watcher.EXPECTED_MOTOR_IDS:
            stamps = dict(stamps)
            stamps[motor_id] += 1000
            accumulator.add_frame(
                make_frame_from_stamps(stamps),
                "MATDOG-BUS",
            )

        received_frames = 1 + len(
            watcher.EXPECTED_MOTOR_IDS
        )
        report = accumulator.report(
            station_server="localhost",
            bus_serial="MATDOG-BUS",
            requested_frames=received_frames,
            received_frames=received_frames,
        )

        self.assertTrue(report.all_motors_advanced)
        self.assertTrue(
            report.timestamps_strictly_increasing
        )

        for summary in report.motors:
            self.assertEqual(summary.sample_count, 2)
            self.assertEqual(
                summary.frame_observation_count,
                received_frames,
            )
            self.assertEqual(
                summary.duplicate_frame_count,
                received_frames - 2,
            )

    def test_timestamp_regression_is_rejected(self):
        accumulator = watcher.SnapshotAccumulator()

        accumulator.add_frame(
            make_frame(1000),
            "MATDOG-BUS",
        )

        with self.assertRaisesRegex(
            watcher.ReadonlyWatchError,
            "timestamp monotonic regressivo",
        ):
            accumulator.add_frame(
                make_frame(999),
                "MATDOG-BUS",
            )

    def test_report_rejects_motor_without_advancement(self):
        accumulator = watcher.SnapshotAccumulator()

        accumulator.add_frame(
            make_frame(1000),
            "MATDOG-BUS",
        )
        accumulator.add_frame(
            make_frame(1000),
            "MATDOG-BUS",
        )

        with self.assertRaisesRegex(
            watcher.ReadonlyWatchError,
            "nessun avanzamento timestamp",
        ):
            accumulator.report(
                station_server="localhost",
                bus_serial="MATDOG-BUS",
                requested_frames=2,
                received_frames=2,
            )

    def test_station_restart_is_rejected(self):
        accumulator = watcher.SnapshotAccumulator()

        accumulator.add_frame(
            make_frame(
                1000,
                app_start_id=9,
            ),
            "MATDOG-BUS",
        )

        with self.assertRaisesRegex(
            watcher.ReadonlyWatchError,
            "Restart Station",
        ):
            accumulator.add_frame(
                make_frame(
                    2000,
                    app_start_id=10,
                ),
                "MATDOG-BUS",
            )

    def test_module_has_no_command_capability(self):
        module_path = (
            CALIBRATION_DIR
            / "matdog_endstop_station_readonly_watch.py"
        )
        source = module_path.read_text(encoding="utf-8")
        tree = ast.parse(source)

        forbidden_imports = {
            "send_commands",
            "commands",
            "drivers",
        }
        forbidden_calls = {
            "send_commands",
            "enqueue",
            "enqueue_pack",
            "set_torque",
            "send_goal",
            "send_write",
        }
        forbidden_attributes = {
            "DriverCommand",
            "ST3215WriteCommand",
            "ST3215SyncWriteCommand",
            "ST3215RegWriteCommand",
            "ST3215ActionCommand",
        }

        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    self.assertNotIn(
                        alias.name,
                        forbidden_imports,
                    )

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

        self.assertNotIn('"commands"', source)
        self.assertNotIn("'commands'", source)
        self.assertNotIn("--execute", source)


if __name__ == "__main__":
    unittest.main()
