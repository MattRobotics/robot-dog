#!/usr/bin/env python3
"""Fail-closed headless launcher/observer for the MATDOG LF calibrator.

This process does not open the serial device and does not launch Station.  It
subscribes to the one already-running headless Station, proves a passive
source-latest preflight using a short consecutive-snapshot gate, enqueues exactly one
native auto-calibration request, and
then observes the persistent Rust state machine through a terminal state and
fresh global torque-OFF readback.

The only ST3215 command bodies this file can construct are:

* AutoCalibrateCommand(calibrate=True)
* StopAutoCalibrateCommand(stop=True)

There are deliberately no register-write, reset, freeze, save, lock, RegWrite,
or Action builders in this runner.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import Counter
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import logging
import os
from pathlib import Path
import signal
import struct
import sys
import time
from typing import Any
import uuid


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215
from target.gen_python.protobuf.station import commands, drivers


INFERENCE_QUEUE = "st3215/inference"
COMMAND_QUEUE = "commands"
EXPECTED_BUS_SERIAL = "5B14114953"
EXPECTED_MOTOR_IDS = (
    11,
    12,
    13,
    21,
    22,
    23,
    31,
    32,
    33,
    41,
    42,
    43,
)
CONTROLLED_MOTOR_IDS = frozenset((11, 12, 13, 42))
NONPARTICIPATING_MOTOR_IDS = frozenset(EXPECTED_MOTOR_IDS) - CONTROLLED_MOTOR_IDS

# ST3215 memory image offsets.  Reading the image published by Station is
# passive; this runner never asks the bus to write any of these registers.
MAX_TEMPERATURE_LIMIT = 0x0D
TORQUE_ENABLE = 0x28
GOAL_POSITION = 0x2A
TORQUE_LIMIT = 0x30
PRESENT_POSITION = 0x38
PRESENT_SPEED = 0x3A
PRESENT_VOLTAGE = 0x3E
PRESENT_TEMPERATURE = 0x3F
STATUS = 0x41
PRESENT_CURRENT = 0x45
MIN_STATE_LENGTH = PRESENT_CURRENT + 2

MAX_SAFE_CURRENT_RAW = 199
MAX_IDLE_SPEED_RAW = 4
MAX_IDLE_POSITION_SPREAD = 8
MAX_NONPARTICIPANT_DRIFT = 16
MAX_TELEMETRY_AGE_NS = 3_000_000_000
PREFLIGHT_TEMPERATURE_CEILING = 60
EXPECTED_TEMPERATURE_LIMIT_C = 70
MAX_CONSECUTIVE_TEMPERATURE_JUMP_C = 5
LATEST_HEAD_SETTLE_YIELDS = 3
LATEST_DRAIN_YIELD_INTERVAL = 1024
EXPECTED_STATION_SHA256 = (
    "29aba7a67bd0d3a80bdf77d293576704a8f61d006c3ba858699f23bb20cc26df"
)
EXPECTED_ACTIVE_TORQUE_LIMIT = 500
CONTROLLED_POSITION_CORRIDORS = {
    11: (1557, 3159),
    12: (1387, 3506),
    13: (1472, 2624),
    42: (2016, 2421),
}
CONTROLLED_GOAL_CORRIDORS = {
    11: (1557, 3159),
    12: (1387, 3506),
    13: (1472, 2624),
    42: (2048, 2389),
}

EXPECTED_START_BODY_HEX = "0a0a354231343131343935338a01020801"
EXPECTED_STOP_BODY_HEX = "0a0a354231343131343935339201020801"
EXPECTED_FULL_TOTAL_STEPS = 58
FULL_PROFILE_PREFIX = "LF_LEG_STATE_MACHINE:"
FULL_COMPLETED_PHASE = "LF_LEG_STATE_MACHINE: completed"

TERMINAL_STATUSES = frozenset(
    (
        st3215.AutoCalibrationState_Status.DONE,
        st3215.AutoCalibrationState_Status.FAILED,
        st3215.AutoCalibrationState_Status.STOPPED,
    )
)



class LatestOnlyQueue(asyncio.Queue[Any]):
    """A queue-of-one: newer telemetry atomically supersedes older telemetry."""

    def __init__(self) -> None:
        super().__init__(maxsize=1)

    def put_nowait(self, item: Any) -> None:
        if self.full():
            try:
                self.get_nowait()
            except asyncio.QueueEmpty:
                pass
        super().put_nowait(item)

    async def put(self, item: Any) -> None:
        self.put_nowait(item)


class RunnerError(RuntimeError):
    """A fail-closed runner contract violation."""


class InferenceStreamError(RunnerError):
    """The current inference subscription ended or failed."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def station_monotonic_stamp_ns() -> int:
    """Use the same Linux clock as `systime::get_monotonic_stamp_ns()`."""

    clock_boottime = getattr(time, "CLOCK_BOOTTIME", None)
    if clock_boottime is None:
        raise RunnerError("CLOCK_BOOTTIME is unavailable on this host")
    return int(time.clock_gettime_ns(clock_boottime))


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _u8(state: bytes, address: int) -> int:
    if len(state) <= address:
        raise RunnerError(
            f"ST3215 state too short for 0x{address:02X}: {len(state)} bytes"
        )
    return int(state[address])


def _u16(state: bytes, address: int) -> int:
    if len(state) < address + 2:
        raise RunnerError(
            f"ST3215 state too short for 0x{address:02X}: {len(state)} bytes"
        )
    return int(struct.unpack_from("<H", state, address)[0])


def feedback_magnitude(raw: int) -> int:
    """Remove the ST3215 direction bit from speed/current feedback."""

    return int(raw) & 0x7FFF


def normalize_position(raw: int) -> int:
    """Normalize only a received position; goals remain unsigned on the wire."""

    raw = int(raw)
    if raw & 0x8000:
        return (4096 - (raw & 0x0FFF)) & 0x0FFF
    return raw & 0x0FFF


def circular_distance(first: int, second: int) -> int:
    direct = abs(int(first) - int(second))
    return min(direct, 4096 - direct)


@dataclass(frozen=True)
class MotorSample:
    motor_id: int
    monotonic_stamp_ns: int
    system_stamp_ns: int
    app_start_id: int
    state_length: int
    rx_pointer_hex: str
    rx_pointer_decimal: int
    raw_ram_0x28_0x46_hex: str
    position: int
    speed_raw: int
    current_raw: int
    voltage_raw: int
    voltage_v: float
    temperature_c: int
    temperature_limit_c: int
    status: int
    torque_raw: int
    torque_enabled: bool
    goal_position: int
    torque_limit: int
    driver_error_present: bool


@dataclass(frozen=True)
class CalibrationSample:
    status: int
    status_name: str
    current_step: int
    total_steps: int
    phase: str
    error_message: str


@dataclass(frozen=True)
class FrameSample:
    bus_serial: str
    bus_monotonic_stamp_ns: int
    bus_system_stamp_ns: int
    bus_app_start_id: int
    calibration: CalibrationSample
    motors: dict[int, MotorSample]

    def compact(self) -> dict[str, Any]:
        return {
            "bus_serial": self.bus_serial,
            "bus_monotonic_stamp_ns": self.bus_monotonic_stamp_ns,
            "bus_system_stamp_ns": self.bus_system_stamp_ns,
            "bus_app_start_id": self.bus_app_start_id,
            "calibration": asdict(self.calibration),
            "motors": {
                str(motor_id): asdict(sample)
                for motor_id, sample in sorted(self.motors.items())
            },
        }


def parse_frame(entry_data: memoryview, bus_serial: str) -> FrameSample:
    inference = st3215.InferenceStateReader(entry_data)
    buses = inference.get_buses() or []
    observed_serials = [bus.get_bus().get_serial_number() for bus in buses]
    if len(buses) != 1 or observed_serials != [bus_serial]:
        raise RunnerError(
            "expected exactly one ST3215 bus "
            f"{bus_serial!r}, observed {observed_serials!r}"
        )

    bus = buses[0]
    readers = bus.get_motors() or []
    observed_ids = [int(reader.get_id()) for reader in readers]
    if len(observed_ids) != len(set(observed_ids)):
        raise RunnerError(f"duplicate motor IDs in inference frame: {observed_ids}")
    if tuple(sorted(observed_ids)) != EXPECTED_MOTOR_IDS:
        raise RunnerError(
            f"unexpected motor set: expected={EXPECTED_MOTOR_IDS}, "
            f"observed={tuple(sorted(observed_ids))}"
        )

    motors: dict[int, MotorSample] = {}
    for reader in readers:
        motor_id = int(reader.get_id())
        state = bytes(reader.get_state())
        if len(state) < MIN_STATE_LENGTH:
            raise RunnerError(
                f"M{motor_id} state length {len(state)} < {MIN_STATE_LENGTH}"
            )
        torque_raw = _u8(state, TORQUE_ENABLE)
        if torque_raw not in (0, 1):
            raise RunnerError(f"M{motor_id} invalid TorqueEnable value {torque_raw}")
        goal_position = _u16(state, GOAL_POSITION)
        if not 0 <= goal_position <= 4095:
            raise RunnerError(
                f"M{motor_id} GoalPosition is not unsigned 12-bit: {goal_position}"
            )

        # The generated reader has no public presence bit for this optional
        # message.  `_error_buf` is the generated schema's exact presence bit.
        driver_error_present = getattr(reader, "_error_buf", None) is not None
        rx_pointer = bytes(reader.get_rx_pointer())
        motors[motor_id] = MotorSample(
            motor_id=motor_id,
            monotonic_stamp_ns=int(reader.get_monotonic_stamp_ns()),
            system_stamp_ns=int(reader.get_system_stamp_ns()),
            app_start_id=int(reader.get_app_start_id()),
            state_length=len(state),
            rx_pointer_hex=rx_pointer.hex(),
            rx_pointer_decimal=(
                int.from_bytes(rx_pointer, byteorder="little") if rx_pointer else 0
            ),
            raw_ram_0x28_0x46_hex=state[TORQUE_ENABLE:MIN_STATE_LENGTH].hex(),
            position=normalize_position(_u16(state, PRESENT_POSITION)),
            speed_raw=feedback_magnitude(_u16(state, PRESENT_SPEED)),
            current_raw=feedback_magnitude(_u16(state, PRESENT_CURRENT)),
            voltage_raw=_u8(state, PRESENT_VOLTAGE),
            voltage_v=_u8(state, PRESENT_VOLTAGE) / 10.0,
            temperature_c=_u8(state, PRESENT_TEMPERATURE),
            temperature_limit_c=_u8(state, MAX_TEMPERATURE_LIMIT),
            status=_u8(state, STATUS),
            torque_raw=torque_raw,
            torque_enabled=torque_raw == 1,
            goal_position=goal_position,
            torque_limit=_u16(state, TORQUE_LIMIT),
            driver_error_present=driver_error_present,
        )

    calibration_reader = bus.get_auto_calibration()
    calibration_status = calibration_reader.get_status()
    calibration = CalibrationSample(
        status=int(calibration_status),
        status_name=calibration_status.name,
        current_step=int(calibration_reader.get_current_step()),
        total_steps=int(calibration_reader.get_total_steps()),
        phase=calibration_reader.get_current_phase(),
        error_message=calibration_reader.get_error_message(),
    )
    return FrameSample(
        bus_serial=bus_serial,
        bus_monotonic_stamp_ns=int(bus.get_monotonic_stamp_ns()),
        bus_system_stamp_ns=int(bus.get_system_stamp_ns()),
        bus_app_start_id=int(bus.get_app_start_id()),
        calibration=calibration,
        motors=motors,
    )


def system_stamp_utc(system_stamp_ns: int) -> str:
    return datetime.fromtimestamp(
        int(system_stamp_ns) / 1_000_000_000,
        tz=timezone.utc,
    ).isoformat()


def motor_evidence_record(
    sample: MotorSample,
    *,
    phase: str,
    inference_queue_entry_id_hex: str,
    collector_received_boottime_ns: int,
    collector_received_utc: str,
) -> dict[str, Any]:
    return {
        "phase": phase,
        "collector_received_utc": collector_received_utc,
        "collector_received_boottime_ns": collector_received_boottime_ns,
        "inference_queue_entry_id_hex": inference_queue_entry_id_hex,
        "system_timestamp_utc": system_stamp_utc(sample.system_stamp_ns),
        **asdict(sample),
    }


class ThermalSeriesRecorder:
    """In-memory safety series; persistence is deliberately outside ingestion."""

    def __init__(self) -> None:
        self.last_stamp = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.last_sample: dict[int, MotorSample] = {}
        self.last_temperature: dict[int, int] = {}
        self.counts = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.minimum_temperature: dict[int, int] = {}
        self.maximum_temperature: dict[int, int] = {}
        self.histograms = {
            motor_id: Counter() for motor_id in EXPECTED_MOTOR_IDS
        }
        self.max_jump = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.max_jump_records: dict[int, dict[str, Any]] = {}
        self.m11_records: list[dict[str, Any]] = []
        self.m11_last_recorded_stamp_ns = 0
        self.m11_last_phase = ""
        self.anomalies: list[dict[str, Any]] = []

    def observe_frame(
        self,
        frame: FrameSample,
        *,
        phase: str,
        inference_queue_entry_id_hex: str,
        collector_received_boottime_ns: int,
        collector_received_utc: str,
    ) -> None:
        for motor_id in EXPECTED_MOTOR_IDS:
            sample = frame.motors[motor_id]
            previous_stamp = self.last_stamp[motor_id]
            if sample.monotonic_stamp_ns < previous_stamp:
                raise RunnerError(
                    f"M{motor_id} timestamp regressed in thermal series: "
                    f"{previous_stamp} -> {sample.monotonic_stamp_ns}"
                )
            if sample.monotonic_stamp_ns == previous_stamp:
                if self.last_sample.get(motor_id) != sample:
                    raise RunnerError(
                        f"M{motor_id} thermal payload changed without source "
                        "timestamp advance"
                    )
                continue

            record = motor_evidence_record(
                sample,
                phase=phase,
                inference_queue_entry_id_hex=inference_queue_entry_id_hex,
                collector_received_boottime_ns=collector_received_boottime_ns,
                collector_received_utc=collector_received_utc,
            )
            previous_temperature = self.last_temperature.get(motor_id)
            if previous_temperature is not None:
                jump = abs(sample.temperature_c - previous_temperature)
                if jump > self.max_jump[motor_id]:
                    self.max_jump[motor_id] = jump
                    self.max_jump_records[motor_id] = {
                        "previous_temperature_c": previous_temperature,
                        "temperature_jump_c": jump,
                        **record,
                    }
                if jump > MAX_CONSECUTIVE_TEMPERATURE_JUMP_C:
                    self.anomalies.append(
                        {
                            "kind": "consecutive_temperature_jump",
                            "previous_temperature_c": previous_temperature,
                            "temperature_jump_c": jump,
                            **record,
                        }
                    )
            above_phase_ceiling = (
                phase == "preflight"
                and sample.temperature_c >= PREFLIGHT_TEMPERATURE_CEILING
            ) or (
                phase != "preflight"
                and sample.temperature_c > sample.temperature_limit_c
            )
            if above_phase_ceiling:
                self.anomalies.append(
                    {
                        "kind": "temperature_above_phase_ceiling",
                        **record,
                    }
                )

            self.last_stamp[motor_id] = sample.monotonic_stamp_ns
            self.last_sample[motor_id] = sample
            self.last_temperature[motor_id] = sample.temperature_c
            self.counts[motor_id] += 1
            self.minimum_temperature[motor_id] = min(
                self.minimum_temperature.get(motor_id, sample.temperature_c),
                sample.temperature_c,
            )
            self.maximum_temperature[motor_id] = max(
                self.maximum_temperature.get(motor_id, sample.temperature_c),
                sample.temperature_c,
            )
            self.histograms[motor_id][sample.temperature_c] += 1
            if motor_id == 11:
                phase_changed = phase != self.m11_last_phase
                period_elapsed = (
                    sample.monotonic_stamp_ns - self.m11_last_recorded_stamp_ns
                    >= 250_000_000
                )
                if phase_changed or period_elapsed:
                    self.m11_records.append(record)
                    self.m11_last_recorded_stamp_ns = sample.monotonic_stamp_ns
                    self.m11_last_phase = phase

    def summary(self) -> dict[str, Any]:
        return {
            "per_motor": {
                str(motor_id): {
                    "samples": self.counts[motor_id],
                    "temperature_min_c": self.minimum_temperature.get(motor_id),
                    "temperature_max_c": self.maximum_temperature.get(motor_id),
                    "temperature_histogram_c": {
                        str(temperature): count
                        for temperature, count in sorted(
                            self.histograms[motor_id].items()
                        )
                    },
                    "max_consecutive_jump_c": self.max_jump[motor_id],
                    "max_jump_record": self.max_jump_records.get(motor_id),
                }
                for motor_id in EXPECTED_MOTOR_IDS
            },
            "m11_complete_samples": len(self.m11_records),
            "thermal_anomalies": self.anomalies,
        }


class LatestSamplePreflight:
    """Fail-closed passive gate evaluated only on the source-latest sample."""

    def __init__(self, telemetry: ThermalSeriesRecorder) -> None:
        self.telemetry = telemetry
        self.app_start_id: int | None = None
        self.latest_by_motor: dict[int, MotorSample] = {}
        self.previous_window_stamps = {
            motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS
        }
        self.first_source_stamps: dict[int, int] = {}
        self.initial_goals: dict[int, int] = {}
        self.initial_positions: dict[int, int] = {}
        self.position_min: dict[int, int] = {}
        self.position_max: dict[int, int] = {}
        self.position_spread = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.unique_samples = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.max_observed_backlog_ns = {
            motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS
        }
        self.max_latest_source_age_ns = {
            motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS
        }
        self.frames_observed = 0
        self.superseded_motor_samples = 0
        self.windows_validated = 0
        self.first_window_latest: dict[int, MotorSample] | None = None

    def _validate_frame_identity(self, frame: FrameSample) -> None:
        if frame.bus_serial != EXPECTED_BUS_SERIAL:
            raise RunnerError(
                f"unexpected bus serial {frame.bus_serial!r} during preflight"
            )
        observed_ids = tuple(sorted(frame.motors))
        if observed_ids != EXPECTED_MOTOR_IDS:
            raise RunnerError(
                f"unexpected motor set: expected={EXPECTED_MOTOR_IDS}, "
                f"observed={observed_ids}"
            )
        if frame.bus_app_start_id <= 0:
            raise RunnerError("Station bus app_start_id is zero")
        if self.app_start_id is None:
            self.app_start_id = frame.bus_app_start_id
        elif frame.bus_app_start_id != self.app_start_id:
            raise RunnerError(
                "Station restart detected during preflight: "
                f"{self.app_start_id} -> {frame.bus_app_start_id}"
            )
        if frame.calibration.status != int(st3215.AutoCalibrationState_Status.IDLE):
            raise RunnerError(
                "calibrator is not IDLE during preflight: "
                f"{frame.calibration.status_name}"
            )

    def _validate_new_sample(self, sample: MotorSample) -> None:
        motor_id = sample.motor_id
        if sample.app_start_id != self.app_start_id:
            raise RunnerError(
                f"M{motor_id} app_start_id {sample.app_start_id} != "
                f"bus {self.app_start_id}"
            )
        if sample.monotonic_stamp_ns <= 0:
            raise RunnerError(f"M{motor_id} has a non-positive timestamp")
        if sample.driver_error_present:
            raise RunnerError(f"M{motor_id} driver error is present")
        if sample.status != 0:
            raise RunnerError(f"M{motor_id} servo status is 0x{sample.status:02X}")
        if sample.torque_enabled:
            raise RunnerError(f"M{motor_id} torque is ON during preflight")
        if sample.speed_raw > MAX_IDLE_SPEED_RAW:
            raise RunnerError(f"M{motor_id} is not idle: speed={sample.speed_raw}")
        if sample.current_raw > MAX_SAFE_CURRENT_RAW:
            raise RunnerError(
                f"M{motor_id} current {sample.current_raw} > {MAX_SAFE_CURRENT_RAW}"
            )
        if sample.temperature_limit_c != 70:
            raise RunnerError(
                f"M{motor_id} configured temperature limit changed: "
                f"{sample.temperature_limit_c} C != 70 C"
            )
        if sample.temperature_c >= PREFLIGHT_TEMPERATURE_CEILING:
            raise RunnerError(
                f"M{motor_id} preflight temperature {sample.temperature_c} C "
                f">= {PREFLIGHT_TEMPERATURE_CEILING} C"
            )

        previous = self.latest_by_motor.get(motor_id)
        if previous is not None:
            jump = abs(sample.temperature_c - previous.temperature_c)
            if jump > MAX_CONSECUTIVE_TEMPERATURE_JUMP_C:
                raise RunnerError(
                    f"M{motor_id} consecutive temperature jump {jump} C > "
                    f"{MAX_CONSECUTIVE_TEMPERATURE_JUMP_C} C"
                )

        self.initial_goals.setdefault(motor_id, sample.goal_position)
        self.initial_positions.setdefault(motor_id, sample.position)
        if sample.goal_position != self.initial_goals[motor_id]:
            raise RunnerError(
                f"M{motor_id} goal changed during passive preflight: "
                f"{self.initial_goals[motor_id]} -> {sample.goal_position}"
            )
        spread = circular_distance(
            self.initial_positions[motor_id], sample.position
        )
        self.position_spread[motor_id] = max(
            self.position_spread[motor_id], spread
        )
        if self.position_spread[motor_id] > MAX_IDLE_POSITION_SPREAD:
            raise RunnerError(
                f"M{motor_id} preflight position spread "
                f"{self.position_spread[motor_id]} > {MAX_IDLE_POSITION_SPREAD}"
            )
        self.position_min[motor_id] = min(
            self.position_min.get(motor_id, sample.position), sample.position
        )
        self.position_max[motor_id] = max(
            self.position_max.get(motor_id, sample.position), sample.position
        )

    def observe_frame(
        self,
        frame: FrameSample,
        *,
        inference_queue_entry_id_hex: str,
        received_boottime_ns: int,
        received_utc: str,
    ) -> None:
        self._validate_frame_identity(frame)
        self.frames_observed += 1

        # Record every source-new thermal sample before replacing samples for
        # the latest-only freshness gate. No filtering or averaging occurs.
        self.telemetry.observe_frame(
            frame,
            phase="preflight",
            inference_queue_entry_id_hex=inference_queue_entry_id_hex,
            collector_received_boottime_ns=received_boottime_ns,
            collector_received_utc=received_utc,
        )

        for motor_id in EXPECTED_MOTOR_IDS:
            sample = frame.motors[motor_id]
            observed_age_ns = received_boottime_ns - sample.monotonic_stamp_ns
            if observed_age_ns < 0:
                raise RunnerError(
                    f"M{motor_id} source timestamp is in the CLOCK_BOOTTIME future: "
                    f"age_ns={observed_age_ns}"
                )
            self.max_observed_backlog_ns[motor_id] = max(
                self.max_observed_backlog_ns[motor_id], observed_age_ns
            )

            previous = self.latest_by_motor.get(motor_id)
            if previous is not None:
                if sample.monotonic_stamp_ns < previous.monotonic_stamp_ns:
                    raise RunnerError(
                        f"M{motor_id} timestamp regressed: "
                        f"{previous.monotonic_stamp_ns} -> "
                        f"{sample.monotonic_stamp_ns}"
                    )
                if sample.monotonic_stamp_ns == previous.monotonic_stamp_ns:
                    if sample != previous:
                        raise RunnerError(
                            f"M{motor_id} payload changed without source timestamp advance"
                        )
                    self.superseded_motor_samples += 1
                    continue

            self._validate_new_sample(sample)
            self.latest_by_motor[motor_id] = sample
            self.first_source_stamps.setdefault(
                motor_id, sample.monotonic_stamp_ns
            )
            self.unique_samples[motor_id] += 1

    def window_ready(self) -> bool:
        return all(
            motor_id in self.latest_by_motor
            and self.latest_by_motor[motor_id].monotonic_stamp_ns
            > self.previous_window_stamps[motor_id]
            for motor_id in EXPECTED_MOTOR_IDS
        )

    def validate_window(self, now_boottime_ns: int) -> dict[str, Any]:
        missing = [
            motor_id
            for motor_id in EXPECTED_MOTOR_IDS
            if motor_id not in self.latest_by_motor
        ]
        if missing:
            raise RunnerError(f"latest preflight sample missing IDs: {missing}")

        ages: dict[str, int] = {}
        for motor_id in EXPECTED_MOTOR_IDS:
            sample = self.latest_by_motor[motor_id]
            age_ns = now_boottime_ns - sample.monotonic_stamp_ns
            if age_ns < 0 or age_ns > MAX_TELEMETRY_AGE_NS:
                raise RunnerError(
                    f"M{motor_id} latest telemetry is not fresh: age_ns={age_ns}"
                )
            previous_window = self.previous_window_stamps[motor_id]
            if sample.monotonic_stamp_ns <= previous_window:
                raise RunnerError(
                    f"M{motor_id} has no new source sample for latest window: "
                    f"latest={sample.monotonic_stamp_ns}, previous={previous_window}"
                )
            self.max_latest_source_age_ns[motor_id] = max(
                self.max_latest_source_age_ns[motor_id], age_ns
            )
            ages[str(motor_id)] = age_ns

        if self.first_window_latest is None:
            self.first_window_latest = dict(self.latest_by_motor)
        self.previous_window_stamps = {
            motor_id: self.latest_by_motor[motor_id].monotonic_stamp_ns
            for motor_id in EXPECTED_MOTOR_IDS
        }
        self.windows_validated += 1
        return {
            "window": self.windows_validated,
            "evaluated_at_boottime_ns": now_boottime_ns,
            "latest_source_age_ns": ages,
            "latest_source_stamp_ns": {
                str(motor_id): self.latest_by_motor[motor_id].monotonic_stamp_ns
                for motor_id in EXPECTED_MOTOR_IDS
            },
        }

    def minimum_source_span_ns(self) -> int:
        if self.first_window_latest is None:
            return 0
        return min(
            self.latest_by_motor[motor_id].monotonic_stamp_ns
            - self.first_window_latest[motor_id].monotonic_stamp_ns
            for motor_id in EXPECTED_MOTOR_IDS
        )

    def summary(self) -> dict[str, Any]:
        return {
            "app_start_id": self.app_start_id,
            "frames_observed": self.frames_observed,
            "windows_validated": self.windows_validated,
            "superseded_motor_samples_for_gate": self.superseded_motor_samples,
            "unique_samples": {
                str(motor_id): self.unique_samples[motor_id]
                for motor_id in EXPECTED_MOTOR_IDS
            },
            "max_observed_backlog_ns": {
                str(motor_id): self.max_observed_backlog_ns[motor_id]
                for motor_id in EXPECTED_MOTOR_IDS
            },
            "max_latest_source_age_ns": {
                str(motor_id): self.max_latest_source_age_ns[motor_id]
                for motor_id in EXPECTED_MOTOR_IDS
            },
            "minimum_source_span_ns": self.minimum_source_span_ns(),
            "position_ranges": {
                str(motor_id): [
                    self.position_min[motor_id],
                    self.position_max[motor_id],
                ]
                for motor_id in EXPECTED_MOTOR_IDS
            },
            "position_spread_from_initial": {
                str(motor_id): self.position_spread[motor_id]
                for motor_id in EXPECTED_MOTOR_IDS
            },
            "first_window_latest": {
                str(motor_id): asdict(sample)
                for motor_id, sample in sorted(
                    (self.first_window_latest or {}).items()
                )
            },
            "last_window_latest": {
                str(motor_id): asdict(self.latest_by_motor[motor_id])
                for motor_id in EXPECTED_MOTOR_IDS
            },
        }


class FrameContract:
    def __init__(self) -> None:
        self.app_start_id: int | None = None
        self.last_motor_stamps = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.unique_motor_samples = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        self.initial_goals: dict[int, int] = {}
        self.initial_positions: dict[int, int] = {}

    def validate_payload(self, frame: FrameSample) -> None:
        """Validate every dequeued payload without applying wall-clock freshness."""

        if frame.bus_app_start_id <= 0:
            raise RunnerError("Station bus app_start_id is zero")
        if self.app_start_id is None:
            self.app_start_id = frame.bus_app_start_id
        elif frame.bus_app_start_id != self.app_start_id:
            raise RunnerError(
                "Station restart detected: "
                f"{self.app_start_id} -> {frame.bus_app_start_id}"
            )

        for motor_id in EXPECTED_MOTOR_IDS:
            sample = frame.motors[motor_id]
            if sample.app_start_id != self.app_start_id:
                raise RunnerError(
                    f"M{motor_id} app_start_id {sample.app_start_id} != "
                    f"bus {self.app_start_id}"
                )
            if sample.monotonic_stamp_ns <= 0:
                raise RunnerError(f"M{motor_id} has a non-positive timestamp")
            previous = self.last_motor_stamps[motor_id]
            if previous and sample.monotonic_stamp_ns < previous:
                raise RunnerError(f"M{motor_id} timestamp regressed")
            if sample.monotonic_stamp_ns > previous:
                self.unique_motor_samples[motor_id] += 1
                self.last_motor_stamps[motor_id] = sample.monotonic_stamp_ns
            if sample.driver_error_present:
                raise RunnerError(f"M{motor_id} driver error is present")
            if sample.status != 0:
                raise RunnerError(f"M{motor_id} servo status is 0x{sample.status:02X}")
            if sample.current_raw > MAX_SAFE_CURRENT_RAW:
                raise RunnerError(
                    f"M{motor_id} current {sample.current_raw} > {MAX_SAFE_CURRENT_RAW}"
                )
            if sample.temperature_limit_c != EXPECTED_TEMPERATURE_LIMIT_C:
                raise RunnerError(
                    f"M{motor_id} configured temperature limit changed: "
                    f"{sample.temperature_limit_c} C != "
                    f"{EXPECTED_TEMPERATURE_LIMIT_C} C"
                )
            if sample.temperature_c > sample.temperature_limit_c:
                raise RunnerError(
                    f"M{motor_id} temperature {sample.temperature_c} C > "
                    f"limit {sample.temperature_limit_c} C"
                )

    def validate_common(self, frame: FrameSample) -> None:
        self.validate_payload(frame)
        now_boottime_ns = station_monotonic_stamp_ns()
        for motor_id in EXPECTED_MOTOR_IDS:
            sample = frame.motors[motor_id]
            telemetry_age_ns = now_boottime_ns - sample.monotonic_stamp_ns
            if telemetry_age_ns < 0 or telemetry_age_ns > MAX_TELEMETRY_AGE_NS:
                raise RunnerError(
                    f"M{motor_id} telemetry is not fresh: age_ns={telemetry_age_ns}"
                )

    def validate_preflight(self, frame: FrameSample) -> None:
        self.validate_common(frame)
        if frame.calibration.status != int(st3215.AutoCalibrationState_Status.IDLE):
            raise RunnerError(
                "calibrator is not IDLE during preflight: "
                f"{frame.calibration.status_name}"
            )
        for motor_id, sample in frame.motors.items():
            if sample.torque_enabled:
                raise RunnerError(f"M{motor_id} torque is ON during preflight")
            if sample.speed_raw > MAX_IDLE_SPEED_RAW:
                raise RunnerError(
                    f"M{motor_id} is not idle: speed={sample.speed_raw}"
                )
            self.initial_goals.setdefault(motor_id, sample.goal_position)
            self.initial_positions.setdefault(motor_id, sample.position)
            if sample.goal_position != self.initial_goals[motor_id]:
                raise RunnerError(
                    f"M{motor_id} goal changed during passive preflight: "
                    f"{self.initial_goals[motor_id]} -> {sample.goal_position}"
                )

    def validate_running(self, frame: FrameSample) -> None:
        self.validate_common(frame)
        for motor_id in CONTROLLED_MOTOR_IDS:
            sample = frame.motors[motor_id]
            if sample.torque_enabled:
                if sample.torque_limit != EXPECTED_ACTIVE_TORQUE_LIMIT:
                    raise RunnerError(
                        f"controlled M{motor_id} torque limit changed: "
                        f"{sample.torque_limit} != {EXPECTED_ACTIVE_TORQUE_LIMIT}"
                    )
                position_low, position_high = CONTROLLED_POSITION_CORRIDORS[motor_id]
                if not position_low <= sample.position <= position_high:
                    raise RunnerError(
                        f"controlled M{motor_id} position {sample.position} outside "
                        f"{position_low}..={position_high}"
                    )
                goal_low, goal_high = CONTROLLED_GOAL_CORRIDORS[motor_id]
                if not goal_low <= sample.goal_position <= goal_high:
                    raise RunnerError(
                        f"controlled M{motor_id} goal {sample.goal_position} outside "
                        f"{goal_low}..={goal_high}"
                    )
        for motor_id in NONPARTICIPATING_MOTOR_IDS:
            sample = frame.motors[motor_id]
            if sample.torque_enabled:
                raise RunnerError(f"nonparticipant M{motor_id} torque became ON")
            if sample.goal_position != self.initial_goals[motor_id]:
                raise RunnerError(
                    f"nonparticipant M{motor_id} goal changed: "
                    f"{self.initial_goals[motor_id]} -> {sample.goal_position}"
                )
            drift = circular_distance(sample.position, self.initial_positions[motor_id])
            if drift > MAX_NONPARTICIPANT_DRIFT:
                raise RunnerError(
                    f"nonparticipant M{motor_id} drifted {drift} ticks"
                )
            if sample.speed_raw > MAX_IDLE_SPEED_RAW:
                raise RunnerError(
                    f"nonparticipant M{motor_id} speed={sample.speed_raw}"
                )

    def validate_torque_off(self, frame: FrameSample) -> None:
        self.validate_common(frame)
        enabled = [
            motor_id
            for motor_id, sample in frame.motors.items()
            if sample.torque_enabled
        ]
        if enabled:
            raise RunnerError(f"global torque-OFF not verified; enabled={enabled}")


class EvidenceWriter:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir.resolve()
        if self.output_dir.exists() and any(self.output_dir.iterdir()):
            raise RunnerError(f"refusing to overwrite non-empty {self.output_dir}")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output_dir / "headless-events.jsonl"
        self.report_path = self.output_dir / "headless-report.json"
        self._events = self.events_path.open("x", encoding="utf-8")
        self.io_errors: list[str] = []

    def emit(self, event: str, **payload: Any) -> None:
        record = {"timestamp_utc": utc_now(), "event": event, **payload}
        try:
            self._events.write(json.dumps(record, sort_keys=True) + "\n")
        except OSError as exc:
            message = f"{type(exc).__name__}: {exc}"
            if message not in self.io_errors:
                self.io_errors.append(message)

    def sync(self) -> None:
        try:
            self._events.flush()
            os.fsync(self._events.fileno())
        except OSError as exc:
            message = f"{type(exc).__name__}: {exc}"
            if message not in self.io_errors:
                self.io_errors.append(message)
            raise RunnerError(f"evidence event sync failed: {message}") from exc

    def _write_records(
        self,
        basename: str,
        records: list[dict[str, Any]],
    ) -> dict[str, Any]:
        jsonl_path = self.output_dir / f"{basename}.jsonl"
        csv_path = self.output_dir / f"{basename}.csv"
        jsonl_temporary = jsonl_path.with_suffix(".jsonl.tmp")
        csv_temporary = csv_path.with_suffix(".csv.tmp")
        try:
            with jsonl_temporary.open("x", encoding="utf-8") as stream:
                for record in records:
                    stream.write(json.dumps(record, sort_keys=True) + "\n")
                stream.flush()
                os.fsync(stream.fileno())

            fieldnames = list(records[0]) if records else ["phase"]
            with csv_temporary.open("x", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(records)
                stream.flush()
                os.fsync(stream.fileno())

            os.replace(jsonl_temporary, jsonl_path)
            os.replace(csv_temporary, csv_path)
        except OSError as exc:
            message = f"{type(exc).__name__}: {exc}"
            if message not in self.io_errors:
                self.io_errors.append(message)
            raise RunnerError(
                f"buffered telemetry persistence failed: {message}"
            ) from exc

        return {
            "records": len(records),
            "jsonl": str(jsonl_path),
            "jsonl_sha256": sha256_bytes(jsonl_path.read_bytes()),
            "csv": str(csv_path),
            "csv_sha256": sha256_bytes(csv_path.read_bytes()),
        }

    def write_preflight_series(
        self,
        records: list[dict[str, Any]],
    ) -> dict[str, Any]:
        return self._write_records("m11-preflight-complete-series", records)

    def write_complete_telemetry(
        self,
        telemetry: ThermalSeriesRecorder,
    ) -> dict[str, Any]:
        series = self._write_records(
            "m11-before-during-after-complete-series",
            telemetry.m11_records,
        )
        summary_path = self.output_dir / "thermal-series-summary.json"
        temporary = summary_path.with_suffix(".json.tmp")
        try:
            with temporary.open("x", encoding="utf-8") as stream:
                json.dump(telemetry.summary(), stream, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, summary_path)
        except OSError as exc:
            message = f"{type(exc).__name__}: {exc}"
            if message not in self.io_errors:
                self.io_errors.append(message)
            raise RunnerError(f"thermal summary persistence failed: {message}") from exc
        return {
            **series,
            "summary": str(summary_path),
            "summary_sha256": sha256_bytes(summary_path.read_bytes()),
        }

    def finalize(self, report: dict[str, Any]) -> None:
        self.sync()
        self._events.close()
        temporary = self.report_path.with_suffix(".json.tmp")
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)


def build_command(
    *,
    bus_serial: str,
    stop: bool,
    command_id: bytes,
) -> tuple[bytes, bytes]:
    if bus_serial != EXPECTED_BUS_SERIAL:
        raise RunnerError(
            f"this runner is pinned to bus {EXPECTED_BUS_SERIAL}, got {bus_serial}"
        )
    if not command_id:
        raise RunnerError("command_id must not be empty")

    if stop:
        body = st3215.Command(
            target_bus_serial=bus_serial,
            stop_auto_calibrate=st3215.StopAutoCalibrateCommand(stop=True),
        ).encode()
        expected_hex = EXPECTED_STOP_BODY_HEX
    else:
        body = st3215.Command(
            target_bus_serial=bus_serial,
            auto_calibrate=st3215.AutoCalibrateCommand(calibrate=True),
        ).encode()
        expected_hex = EXPECTED_START_BODY_HEX

    if body.hex() != expected_hex:
        raise RunnerError(
            f"exact ST3215 wire assertion failed: {body.hex()} != {expected_hex}"
        )

    driver_command = commands.DriverCommand(
        command_id=bytes(command_id),
        type=drivers.StationCommandType.STC_ST3215_COMMAND,
        body=body,
    )
    pack = commands.StationCommandsPack(commands=[driver_command]).encode()

    # Decode the wrapper back before allowing it near the live command queue.
    pack_reader = commands.StationCommandsPackReader(memoryview(pack))
    decoded = pack_reader.get_commands()
    if len(decoded) != 1:
        raise RunnerError("command pack does not contain exactly one command")
    decoded_command = decoded[0]
    if bytes(decoded_command.get_command_id()) != bytes(command_id):
        raise RunnerError("command_id changed during command-pack encoding")
    if decoded_command.get_type() != drivers.StationCommandType.STC_ST3215_COMMAND:
        raise RunnerError("command pack has a non-ST3215 command type")
    if bytes(decoded_command.get_body()) != body:
        raise RunnerError("ST3215 command body changed in command-pack encoding")
    return body, pack


@dataclass(frozen=True)
class ProcessIdentity:
    pid: int
    executable: str
    start_time_ticks: int
    state: str


def read_process_identity(pid: int) -> ProcessIdentity | None:
    proc = Path("/proc") / str(int(pid))
    try:
        stat = (proc / "stat").read_text(encoding="utf-8")
        closing_parenthesis = stat.rfind(")")
        if closing_parenthesis < 0:
            raise RunnerError(f"cannot parse /proc/{pid}/stat")
        fields_after_command = stat[closing_parenthesis + 2 :].split()
        state = fields_after_command[0]
        start_time_ticks = int(fields_after_command[19])
        executable = os.readlink(proc / "exe")
    except FileNotFoundError:
        return None
    except (IndexError, OSError, ValueError) as exc:
        raise RunnerError(f"cannot inspect Station PID {pid}: {exc}") from exc
    return ProcessIdentity(
        pid=int(pid),
        executable=executable,
        start_time_ticks=start_time_ticks,
        state=state,
    )


def verify_station_process(
    pid: int,
    expected_sha256: str,
) -> tuple[ProcessIdentity, dict[str, Any]]:
    identity = read_process_identity(pid)
    if identity is None or identity.state == "Z":
        raise RunnerError(f"Station PID {pid} is not a live process")
    executable_path = Path(f"/proc/{pid}/exe")
    try:
        actual_sha256 = sha256_bytes(executable_path.read_bytes())
    except OSError as exc:
        raise RunnerError(f"cannot hash Station PID {pid} executable: {exc}") from exc
    if actual_sha256 != expected_sha256:
        raise RunnerError(
            "running Station executable SHA-256 mismatch: "
            f"{actual_sha256} != {expected_sha256}"
        )
    return identity, {
        "pid": pid,
        "executable": identity.executable,
        "start_time_ticks": identity.start_time_ticks,
        "sha256": actual_sha256,
    }


def process_identity_is_live(identity: ProcessIdentity) -> bool:
    current = read_process_identity(identity.pid)
    return bool(
        current
        and current.state != "Z"
        and current.start_time_ticks == identity.start_time_ticks
    )


async def controlled_station_shutdown(
    identity: ProcessIdentity,
    *,
    timeout_s: float,
    poll_interval_s: float = 0.1,
    signal_sender: Any = os.kill,
    identity_is_live: Any = process_identity_is_live,
) -> dict[str, Any]:
    """SIGINT + bounded wait; SIGKILL only if the exact process survives."""

    started = time.monotonic()
    result: dict[str, Any] = {
        "pid": identity.pid,
        "sigint_sent": False,
        "sigkill_sent": False,
        "graceful": False,
        "stopped": False,
        "timeout_seconds": timeout_s,
    }
    if not identity_is_live(identity):
        result.update(
            stopped=True,
            already_stopped=True,
            elapsed_seconds=time.monotonic() - started,
        )
        return result

    try:
        signal_sender(identity.pid, signal.SIGINT)
    except ProcessLookupError:
        result.update(
            graceful=True,
            stopped=True,
            elapsed_seconds=time.monotonic() - started,
        )
        return result
    result["sigint_sent"] = True
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not identity_is_live(identity):
            result.update(
                graceful=True,
                stopped=True,
                elapsed_seconds=time.monotonic() - started,
            )
            return result
        await asyncio.sleep(poll_interval_s)

    if identity_is_live(identity):
        try:
            signal_sender(identity.pid, signal.SIGKILL)
        except ProcessLookupError:
            result.update(
                stopped=True,
                elapsed_seconds=time.monotonic() - started,
            )
            return result
        result["sigkill_sent"] = True
        kill_deadline = time.monotonic() + min(5.0, timeout_s)
        while time.monotonic() < kill_deadline:
            if not identity_is_live(identity):
                result.update(
                    stopped=True,
                    elapsed_seconds=time.monotonic() - started,
                )
                return result
            await asyncio.sleep(poll_interval_s)

    result["elapsed_seconds"] = time.monotonic() - started
    return result


async def finish_deferred_write_while_observing(
    persistence_task: asyncio.Task[Any],
    observe_next_window: Any,
) -> tuple[Any, int]:
    """Keep the safety observer live while a buffered writer catches up."""

    windows_observed = 0
    try:
        while not persistence_task.done():
            await observe_next_window()
            windows_observed += 1
    except BaseException:
        # A to_thread write cannot be force-cancelled safely. Do not let it
        # outlive evidence finalization if the safety observer fails.
        try:
            await asyncio.shield(persistence_task)
        except Exception:
            pass
        raise
    return await persistence_task, windows_observed


class HeadlessRun:
    def __init__(self, args: argparse.Namespace, evidence: EvidenceWriter) -> None:
        self.args = args
        self.evidence = evidence
        self.telemetry = ThermalSeriesRecorder()
        self.preflight_collector = LatestSamplePreflight(self.telemetry)
        self.contract = FrameContract()
        self.client: Any = None
        self.entries: asyncio.Queue[Any] = LatestOnlyQueue()
        self.stream_errors: asyncio.Queue[Any] | None = None
        self.stop_event = asyncio.Event()
        self.start_attempted = False
        self.start_entry_id: bytes | None = None
        self.stop_attempted = False
        self.stop_entry_id: bytes | None = None
        self.stop_required = False
        self.faults: list[str] = []
        self.progress: list[dict[str, Any]] = []
        self.last_frame: FrameSample | None = None
        self.terminal_frame: FrameSample | None = None
        self.in_progress_seen = False
        self.expected_total_steps: int | None = None
        self.last_step = 0
        self.max_queue_depth_observed = 0
        self.max_frames_per_head_drain = 0
        self.head_drain_passes = 0
        self.telemetry_evidence: dict[str, Any] | None = None
        self.station_identity: ProcessIdentity | None = None
        self.station_provenance: dict[str, Any] | None = None

    def request_stop(self) -> None:
        self.stop_event.set()

    def fault(self, message: str) -> None:
        if message not in self.faults:
            self.faults.append(message)
            self.evidence.emit("fault", message=message)

    def _raise_stream_error_if_present(self) -> None:
        if self.stream_errors is None:
            raise RunnerError("inference subscription was not initialized")
        if not self.stream_errors.empty():
            raise InferenceStreamError(
                f"inference stream error: {self.stream_errors.get_nowait()}"
            )

    @staticmethod
    def _entry_id_hex(entry: Any) -> str:
        try:
            return bytes(entry.ID.ID).hex()
        except (AttributeError, TypeError, ValueError):
            return ""

    def _parse_stream_entry(self, entry: Any, *, phase: str) -> FrameSample:
        if entry is None:
            raise InferenceStreamError("st3215/inference stream ended")
        received_boottime_ns = station_monotonic_stamp_ns()
        received_utc = utc_now()
        frame = parse_frame(entry.Data, self.args.bus_serial)
        self.last_frame = frame
        self.telemetry.observe_frame(
            frame,
            phase=phase,
            inference_queue_entry_id_hex=self._entry_id_hex(entry),
            collector_received_boottime_ns=received_boottime_ns,
            collector_received_utc=received_utc,
        )
        return frame

    async def next_frame(self, timeout_s: float, *, phase: str) -> FrameSample:
        """Drain to the current inference head and validate only the source-latest frame.

        Every dequeued payload is still parsed, recorded and checked for
        non-freshness-independent safety faults, so a superseded thermal/status
        anomaly cannot be hidden by a newer normal sample.
        """

        deadline = time.monotonic() + timeout_s
        latest: FrameSample | None = None
        drained = 0
        empty_yields = 0

        while empty_yields < LATEST_HEAD_SETTLE_YIELDS:
            self._raise_stream_error_if_present()
            try:
                entry = self.entries.get_nowait()
            except asyncio.QueueEmpty:
                if latest is None and empty_yields == 0:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise RunnerError("timeout waiting for st3215/inference")
                    try:
                        entry = await asyncio.wait_for(
                            self.entries.get(), timeout=remaining
                        )
                    except asyncio.TimeoutError as exc:
                        raise RunnerError(
                            "timeout waiting for st3215/inference"
                        ) from exc
                else:
                    empty_yields += 1
                    await asyncio.sleep(0)
                    continue

            empty_yields = 0
            frame = self._parse_stream_entry(entry, phase=phase)
            self.contract.validate_payload(frame)
            latest = frame
            drained += 1
            if drained % LATEST_DRAIN_YIELD_INTERVAL == 0:
                await asyncio.sleep(0)

        if latest is None:
            raise RunnerError("inference head drain produced no frame")

        self.max_queue_depth_observed = max(
            self.max_queue_depth_observed, drained
        )
        self.max_frames_per_head_drain = max(
            self.max_frames_per_head_drain, drained
        )
        self.head_drain_passes += 1
        return latest

    def _observe_preflight_entry(self, entry: Any) -> None:
        if entry is None:
            raise InferenceStreamError("st3215/inference stream ended")
        received_boottime_ns = station_monotonic_stamp_ns()
        received_utc = utc_now()
        frame = parse_frame(entry.Data, self.args.bus_serial)
        self.last_frame = frame
        self.preflight_collector.observe_frame(
            frame,
            inference_queue_entry_id_hex=self._entry_id_hex(entry),
            received_boottime_ns=received_boottime_ns,
            received_utc=received_utc,
        )

    async def drain_preflight_to_current_head(self, deadline: float) -> int:
        """Coalesce the inference backlog, yielding until its current head."""

        drained = 0
        empty_yields = 0
        self.max_queue_depth_observed = max(
            self.max_queue_depth_observed, self.entries.qsize()
        )
        while empty_yields < LATEST_HEAD_SETTLE_YIELDS:
            if self.stop_event.is_set():
                raise RunnerError("interrupted before START; no command was sent")
            self._raise_stream_error_if_present()
            try:
                entry = self.entries.get_nowait()
            except asyncio.QueueEmpty:
                if drained == 0 and empty_yields == 0:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise RunnerError(
                            "timeout waiting for a latest preflight sample"
                        )
                    try:
                        entry = await asyncio.wait_for(
                            self.entries.get(), timeout=remaining
                        )
                    except asyncio.TimeoutError as exc:
                        raise RunnerError(
                            "timeout waiting for st3215/inference during preflight"
                        ) from exc
                else:
                    empty_yields += 1
                    await asyncio.sleep(0)
                    continue

            empty_yields = 0
            self._observe_preflight_entry(entry)
            drained += 1
            if drained % LATEST_DRAIN_YIELD_INTERVAL == 0:
                await asyncio.sleep(0)
            self.max_queue_depth_observed = max(
                self.max_queue_depth_observed, self.entries.qsize()
            )

        self.head_drain_passes += 1
        self.max_frames_per_head_drain = max(
            self.max_frames_per_head_drain, drained
        )
        return drained

    async def next_preflight_window(self) -> dict[str, Any]:
        deadline = time.monotonic() + self.args.frame_timeout
        while True:
            await self.drain_preflight_to_current_head(deadline)
            if self.preflight_collector.window_ready():
                return self.preflight_collector.validate_window(
                    station_monotonic_stamp_ns()
                )
            if time.monotonic() >= deadline:
                stale = {
                    motor_id: (
                        self.preflight_collector.latest_by_motor[motor_id]
                        .monotonic_stamp_ns
                        if motor_id in self.preflight_collector.latest_by_motor
                        else None
                    )
                    for motor_id in EXPECTED_MOTOR_IDS
                    if motor_id not in self.preflight_collector.latest_by_motor
                    or self.preflight_collector.latest_by_motor[
                        motor_id
                    ].monotonic_stamp_ns
                    <= self.preflight_collector.previous_window_stamps[motor_id]
                }
                raise RunnerError(
                    "latest preflight window did not advance for every ID: "
                    f"{stale}"
                )

    async def enqueue(self, *, stop: bool) -> None:
        command_id = uuid.uuid4().bytes
        body, pack = build_command(
            bus_serial=self.args.bus_serial,
            stop=stop,
            command_id=command_id,
        )
        event_name = "stop_command" if stop else "start_command"
        self.evidence.emit(
            f"{event_name}_prepared",
            command_id_hex=command_id.hex(),
            body_hex=body.hex(),
            body_sha256=sha256_bytes(body),
            pack_hex=pack.hex(),
            pack_sha256=sha256_bytes(pack),
        )
        entry_id = await self.client.enqueue(COMMAND_QUEUE, pack)
        if stop:
            self.stop_entry_id = bytes(entry_id)
        else:
            self.start_entry_id = bytes(entry_id)
        self.evidence.emit(
            f"{event_name}_enqueued",
            command_id_hex=command_id.hex(),
            queue_entry_id_hex=bytes(entry_id).hex(),
        )

    async def send_stop_once(self, reason: str) -> None:
        self.stop_required = True
        if self.stop_attempted:
            return
        try:
            await self.client.wait_ready(timeout=min(15.0, self.args.cleanup_timeout))
        except Exception as exc:
            # No enqueue was attempted, so a later connection recovery may
            # still make the one allowed STOP possible.
            self.fault(f"STOP waiting for connected Station failed: {exc}")
            return
        self.stop_attempted = True
        self.evidence.emit("stop_requested", reason=reason)
        try:
            await self.enqueue(stop=True)
        except Exception as exc:  # A retry could duplicate an accepted STOP.
            self.fault(f"STOP enqueue outcome ambiguous: {exc}")

    async def enqueue_start_with_interrupt_guard(self) -> None:
        """Race the sole START acknowledgement with post-commit interrupts."""

        start_task = asyncio.create_task(self.enqueue(stop=False))
        interrupt_task = asyncio.create_task(self.stop_event.wait())
        stop_task: asyncio.Task[None] | None = None
        try:
            done, _ = await asyncio.wait(
                (start_task, interrupt_task),
                return_when=asyncio.FIRST_COMPLETED,
            )
            if interrupt_task in done and interrupt_task.result():
                self.fault("operator interrupt received during START enqueue")
                # Do not cancel or retry the possibly committed START. Queue
                # the sole STOP concurrently so the server observes START ->
                # STOP even if the START acknowledgement is delayed or lost.
                stop_task = asyncio.create_task(
                    self.send_stop_once("operator interrupt during START enqueue")
                )

            try:
                await start_task
            except Exception as exc:  # Never retry an ambiguously committed START.
                self.fault(f"START enqueue outcome ambiguous: {exc}")
                if stop_task is None:
                    stop_task = asyncio.create_task(
                        self.send_stop_once("ambiguous START enqueue")
                    )

            if stop_task is not None:
                await stop_task
        finally:
            if not interrupt_task.done():
                interrupt_task.cancel()
                try:
                    await interrupt_task
                except asyncio.CancelledError:
                    pass

    async def resubscribe_after_stream_error(self) -> None:
        await self.client.wait_ready(timeout=min(30.0, self.args.cleanup_timeout))
        self.entries = LatestOnlyQueue()
        self.stream_errors = self.client.follow(INFERENCE_QUEUE, self.entries)
        self.evidence.emit("inference_resubscribed")

    def record_progress(self, frame: FrameSample) -> None:
        calibration = frame.calibration
        current = asdict(calibration)
        if not self.progress or self.progress[-1] != current:
            self.progress.append(current)
            self.evidence.emit("calibration_progress", **current)
            print(
                f"[{calibration.current_step:02d}/{calibration.total_steps:02d}] "
                f"{calibration.status_name}: {calibration.phase}"
                + (f" — {calibration.error_message}" if calibration.error_message else ""),
                flush=True,
            )

    def validate_progress(self, frame: FrameSample) -> None:
        calibration = frame.calibration
        status = st3215.AutoCalibrationState_Status(calibration.status)
        if status == st3215.AutoCalibrationState_Status.IN_PROGRESS:
            first_in_progress = not self.in_progress_seen
            self.in_progress_seen = True
            if calibration.total_steps != EXPECTED_FULL_TOTAL_STEPS:
                raise RunnerError(
                    "wrong calibrator profile total_steps: "
                    f"{calibration.total_steps} != {EXPECTED_FULL_TOTAL_STEPS}"
                )
            if first_in_progress and not calibration.phase.startswith(FULL_PROFILE_PREFIX):
                raise RunnerError(
                    "first IN_PROGRESS phase is not the full LF profile: "
                    f"{calibration.phase!r}"
                )
            if self.expected_total_steps is None:
                self.expected_total_steps = calibration.total_steps
            elif calibration.total_steps != self.expected_total_steps:
                raise RunnerError(
                    f"total_steps changed: {self.expected_total_steps} -> "
                    f"{calibration.total_steps}"
                )
            if not self.last_step <= calibration.current_step <= calibration.total_steps:
                raise RunnerError(
                    f"invalid progress step {calibration.current_step}/"
                    f"{calibration.total_steps}, previous={self.last_step}"
                )
            self.last_step = calibration.current_step
        elif status == st3215.AutoCalibrationState_Status.IDLE:
            if self.in_progress_seen:
                raise RunnerError("calibrator returned to IDLE after IN_PROGRESS")
        elif status in TERMINAL_STATUSES:
            # Capture even an immediate failure/STOP so postflight can prove
            # global torque OFF. It can never satisfy DONE semantics without
            # the observed full-profile IN_PROGRESS transition.
            return

    async def preflight(self) -> dict[str, Any]:
        required_duration_ns = int(self.args.preflight_seconds * 1_000_000_000)
        first_window = await self.next_preflight_window()
        started_boottime_ns = int(first_window["evaluated_at_boottime_ns"])
        last_window = first_window

        while True:
            now_boottime_ns = station_monotonic_stamp_ns()
            elapsed_ns = now_boottime_ns - started_boottime_ns
            enough_duration = elapsed_ns >= required_duration_ns
            enough_source_span = (
                self.preflight_collector.minimum_source_span_ns()
                >= required_duration_ns
            )
            enough_frames = (
                self.preflight_collector.frames_observed
                >= self.args.preflight_frames
            )
            if enough_duration and enough_source_span and enough_frames:
                break
            last_window = await self.next_preflight_window()

        preflight_gate_completed_boottime_ns = station_monotonic_stamp_ns()
        preflight_records = list(self.telemetry.m11_records)
        persistence_task = asyncio.create_task(
            asyncio.to_thread(
                self.evidence.write_preflight_series,
                preflight_records,
            )
        )

        # A slow disk writer cannot pause safety ingestion. Continue draining
        # and validating latest windows until the deferred writer is complete.
        persisted_series, persistence_windows = (
            await finish_deferred_write_while_observing(
                persistence_task,
                self.next_preflight_window,
            )
        )

        # Re-establish a source-new, current-head boundary after persistence.
        final_guard = await self.next_preflight_window()
        self.evidence.sync()
        if self.evidence.io_errors:
            raise RunnerError(
                "evidence writer failed before START: "
                + "; ".join(self.evidence.io_errors)
            )

        collector = self.preflight_collector
        self.contract.app_start_id = collector.app_start_id
        self.contract.initial_goals = dict(collector.initial_goals)
        self.contract.initial_positions = dict(collector.initial_positions)
        self.contract.last_motor_stamps = dict(collector.previous_window_stamps)
        self.contract.unique_motor_samples = dict(collector.unique_samples)

        summary = {
            **collector.summary(),
            "required_duration_seconds": self.args.preflight_seconds,
            "duration_ns_before_persistence": (
                preflight_gate_completed_boottime_ns - started_boottime_ns
            ),
            "duration_seconds_before_persistence": (
                preflight_gate_completed_boottime_ns - started_boottime_ns
            )
            / 1_000_000_000,
            "first_window": first_window,
            "last_duration_window": last_window,
            "final_post_persistence_guard": final_guard,
            "max_queue_depth_observed": self.max_queue_depth_observed,
            "head_drain_passes": self.head_drain_passes,
            "max_frames_per_head_drain": self.max_frames_per_head_drain,
            "m11_preflight_series": persisted_series,
            "windows_validated_while_persisting": persistence_windows,
            "thermal_summary_at_gate": self.telemetry.summary(),
        }
        self.evidence.emit("preflight_passed", summary=summary)
        self.evidence.sync()
        return summary

    async def monitor_until_terminal(self) -> None:
        run_deadline = time.monotonic() + self.args.timeout
        cleanup_deadline: float | None = (
            time.monotonic() + self.args.cleanup_timeout
            if self.stop_required
            else None
        )
        while self.terminal_frame is None:
            if self.evidence.io_errors and not self.stop_required:
                self.fault(
                    "evidence writer failed during active run: "
                    + "; ".join(self.evidence.io_errors)
                )
                await self.send_stop_once("evidence writer failure")
                cleanup_deadline = time.monotonic() + self.args.cleanup_timeout
            if self.stop_event.is_set() and not self.stop_required:
                self.fault("operator interrupt received after START")
                await self.send_stop_once("operator interrupt")
                cleanup_deadline = time.monotonic() + self.args.cleanup_timeout
            if time.monotonic() >= run_deadline and not self.stop_required:
                self.fault(f"calibration timeout after {self.args.timeout:.1f}s")
                await self.send_stop_once("calibration timeout")
                cleanup_deadline = time.monotonic() + self.args.cleanup_timeout
            if self.stop_required and not self.stop_attempted:
                await self.send_stop_once("pending fail-closed stop")
            if cleanup_deadline is not None and time.monotonic() >= cleanup_deadline:
                self.fault("no terminal state before cleanup timeout")
                return

            try:
                frame = await self.next_frame(
                    self.args.frame_timeout, phase="during_calibration"
                )
                self.contract.validate_running(frame)
                self.record_progress(frame)
                self.validate_progress(frame)
            except Exception as exc:
                self.fault(str(exc))
                await self.send_stop_once("runner contract violation")
                cleanup_deadline = cleanup_deadline or (
                    time.monotonic() + self.args.cleanup_timeout
                )
                if isinstance(exc, InferenceStreamError):
                    try:
                        await self.resubscribe_after_stream_error()
                    except Exception as reconnect_exc:
                        self.fault(
                            f"inference resubscribe failed: {reconnect_exc}"
                        )
                continue

            status = st3215.AutoCalibrationState_Status(frame.calibration.status)
            if status in TERMINAL_STATUSES:
                if not self.in_progress_seen:
                    self.fault(f"terminal {status.name} observed before IN_PROGRESS")
                self.terminal_frame = frame
                self.evidence.emit("terminal_state", frame=frame.compact())

    async def postflight(self) -> dict[str, Any] | None:
        terminal_observed = self.terminal_frame is not None
        anchor = self.terminal_frame or self.last_frame
        if anchor is None:
            return None

        if not terminal_observed:
            self.fault(
                "postflight cannot certify cleanup without an observed terminal state"
            )

        try:
            self.contract.validate_torque_off(anchor)
        except Exception as exc:
            self.fault(f"post-run anchor safety check failed: {exc}")

        terminal_stamps = {
            motor_id: sample.monotonic_stamp_ns
            for motor_id, sample in anchor.motors.items()
        }
        newer_counts = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
        last_counted = dict(terminal_stamps)
        deadline = time.monotonic() + self.args.cleanup_timeout
        last_postflight: FrameSample | None = None

        while min(newer_counts.values()) < 2 and time.monotonic() < deadline:
            frame: FrameSample | None = None
            try:
                frame = await self.next_frame(
                    self.args.frame_timeout, phase="postflight_torque_off"
                )
                self.contract.validate_torque_off(frame)
                terminal_status = (
                    self.terminal_frame.calibration.status
                    if self.terminal_frame
                    else None
                )
                if terminal_status is not None and frame.calibration.status != terminal_status:
                    raise RunnerError(
                        "terminal calibration status changed during postflight: "
                        f"{terminal_status} -> {frame.calibration.status}"
                    )
            except Exception as exc:
                self.fault(f"postflight safety check failed: {exc}")
                newer_counts = {motor_id: 0 for motor_id in EXPECTED_MOTOR_IDS}
                if frame is not None:
                    last_counted = {
                        motor_id: sample.monotonic_stamp_ns
                        for motor_id, sample in frame.motors.items()
                    }
                continue

            last_postflight = frame
            for motor_id, sample in frame.motors.items():
                if sample.monotonic_stamp_ns > last_counted[motor_id]:
                    newer_counts[motor_id] += 1
                    last_counted[motor_id] = sample.monotonic_stamp_ns

        verified = terminal_observed and min(newer_counts.values()) >= 2
        if not verified:
            if terminal_observed:
                self.fault(
                    "did not obtain two fresh global torque-OFF samples per motor: "
                    f"{newer_counts}"
                )
        summary = {
            "verified": verified,
            "terminal_observed": terminal_observed,
            "newer_samples_per_motor": newer_counts,
            "terminal_stamps": terminal_stamps,
            "last": last_postflight.compact() if last_postflight else None,
        }
        self.evidence.emit("postflight_complete", summary=summary)
        return summary

    async def execute(self) -> dict[str, Any]:
        self.station_identity, self.station_provenance = verify_station_process(
            self.args.station_pid,
            self.args.expected_station_sha256,
        )
        self.evidence.emit(
            "runner_started",
            server=self.args.server,
            bus_serial=self.args.bus_serial,
            expected_motor_ids=EXPECTED_MOTOR_IDS,
            source_sha256=sha256_bytes(Path(__file__).read_bytes()),
            station_process=self.station_provenance,
        )
        self.client = await new_station_client(self.args.server, logging.getLogger("matdog"))
        self.stream_errors = self.client.follow(INFERENCE_QUEUE, self.entries)
        preflight = await self.preflight()
        if self.evidence.io_errors:
            raise RunnerError(
                "evidence writer failed before START: "
                + "; ".join(self.evidence.io_errors)
            )
        # Let queued SIGINT/SIGTERM callbacks run before the final START
        # commit boundary. Signals after this check are handled as post-START
        # and trigger the one permitted STOP/cleanup path.
        await asyncio.sleep(0)
        if self.stop_event.is_set():
            raise RunnerError("interrupted after preflight; START was not sent")

        self.start_attempted = True
        await self.enqueue_start_with_interrupt_guard()

        await self.monitor_until_terminal()
        postflight = await self.postflight()

        terminal = self.terminal_frame.calibration if self.terminal_frame else None
        done_semantics = bool(
            terminal
            and terminal.status == int(st3215.AutoCalibrationState_Status.DONE)
            and self.in_progress_seen
            and terminal.total_steps == EXPECTED_FULL_TOTAL_STEPS
            and terminal.current_step == terminal.total_steps
            and terminal.phase == FULL_COMPLETED_PHASE
            and not terminal.error_message
        )
        cleanup_verified = bool(postflight and postflight["verified"])
        passed = (
            done_semantics
            and cleanup_verified
            and not self.faults
            and not self.evidence.io_errors
        )
        return {
            "schema": "matdog.lf.headless_auto_calibrate.v1",
            "generated_at_utc": utc_now(),
            "result": "PASS" if passed else "FAIL",
            "server": self.args.server,
            "bus_serial": self.args.bus_serial,
            "expected_motor_ids": EXPECTED_MOTOR_IDS,
            "preflight": preflight,
            "start_attempted": self.start_attempted,
            "start_queue_entry_id_hex": (
                self.start_entry_id.hex() if self.start_entry_id else None
            ),
            "stop_attempted": self.stop_attempted,
            "stop_queue_entry_id_hex": (
                self.stop_entry_id.hex() if self.stop_entry_id else None
            ),
            "in_progress_seen": self.in_progress_seen,
            "progress": self.progress,
            "terminal": asdict(terminal) if terminal else None,
            "done_semantics_verified": done_semantics,
            "postflight": postflight,
            "global_torque_off_verified": cleanup_verified,
            "faults": self.faults,
            "evidence_io_errors": self.evidence.io_errors,
            "station_process": self.station_provenance,
            "thermal_series_summary": self.telemetry.summary(),
            "eeprom_writes_sent": False,
            "register_writes_sent_by_runner": False,
            "start_commands_attempted": 1,
            "stop_commands_attempted": int(self.stop_attempted),
        }


async def self_test_start_interrupt_guard() -> None:
    class SelfTestEvidence:
        io_errors: list[str] = []

        def emit(self, _event: str, **_payload: Any) -> None:
            return

    run = HeadlessRun(
        argparse.Namespace(cleanup_timeout=1.0),
        SelfTestEvidence(),  # type: ignore[arg-type]
    )
    start_release = asyncio.Event()
    events: list[str] = []

    async def delayed_start(*, stop: bool) -> None:
        assert not stop
        events.append("start_dispatched")
        await start_release.wait()
        events.append("start_acknowledged")

    async def immediate_stop(reason: str) -> None:
        assert reason == "operator interrupt during START enqueue"
        run.stop_required = True
        run.stop_attempted = True
        events.append("stop_dispatched")
        start_release.set()

    run.enqueue = delayed_start  # type: ignore[method-assign]
    run.send_stop_once = immediate_stop  # type: ignore[method-assign]
    guarded_start = asyncio.create_task(run.enqueue_start_with_interrupt_guard())
    for _ in range(10):
        if events:
            break
        await asyncio.sleep(0)
    assert events == ["start_dispatched"]
    run.request_stop()
    await asyncio.wait_for(guarded_start, timeout=1.0)
    assert events == [
        "start_dispatched",
        "stop_dispatched",
        "start_acknowledged",
    ]
    assert run.faults == ["operator interrupt received during START enqueue"]


def self_test() -> None:
    for stop, expected in (
        (False, EXPECTED_START_BODY_HEX),
        (True, EXPECTED_STOP_BODY_HEX),
    ):
        body, pack = build_command(
            bus_serial=EXPECTED_BUS_SERIAL,
            stop=stop,
            command_id=b"self-test-command",
        )
        assert body.hex() == expected
        assert pack
    assert feedback_magnitude(0x8007) == 7
    assert normalize_position(0x8001) == 4095
    assert circular_distance(4095, 1) == 2
    assert CONTROLLED_POSITION_CORRIDORS[42] == (2016, 2421)
    assert CONTROLLED_GOAL_CORRIDORS[42] == (2048, 2389)

    now_ns = station_monotonic_stamp_ns()
    synthetic_motors = []
    for motor_id in EXPECTED_MOTOR_IDS:
        state = bytearray(MIN_STATE_LENGTH)
        state[MAX_TEMPERATURE_LIMIT] = 70
        state[PRESENT_TEMPERATURE] = 25
        struct.pack_into("<H", state, GOAL_POSITION, 2048)
        struct.pack_into("<H", state, PRESENT_POSITION, 2048)
        synthetic_motors.append(
            st3215.InferenceState_MotorState(
                id=motor_id,
                monotonic_stamp_ns=now_ns - 1,
                system_stamp_ns=now_ns - 1,
                app_start_id=42,
                state=bytes(state),
            )
        )
    synthetic = st3215.InferenceState(
        buses=[
            st3215.InferenceState_BusState(
                bus=st3215.ST3215Bus(serial_number=EXPECTED_BUS_SERIAL),
                monotonic_stamp_ns=now_ns - 1,
                system_stamp_ns=now_ns - 1,
                app_start_id=42,
                motors=synthetic_motors,
                auto_calibration=st3215.AutoCalibrationState(
                    status=st3215.AutoCalibrationState_Status.IDLE,
                ),
            )
        ]
    )
    parsed = parse_frame(memoryview(synthetic.encode()), EXPECTED_BUS_SERIAL)
    contract = FrameContract()
    contract.validate_preflight(parsed)
    telemetry = ThermalSeriesRecorder()
    latest = LatestSamplePreflight(telemetry)
    latest.observe_frame(
        parsed,
        inference_queue_entry_id_hex="0100",
        received_boottime_ns=now_ns,
        received_utc=utc_now(),
    )
    latest_window = latest.validate_window(now_ns)
    assert tuple(sorted(parsed.motors)) == EXPECTED_MOTOR_IDS
    assert all(not sample.torque_enabled for sample in parsed.motors.values())
    assert latest_window["latest_source_age_ns"]["43"] == 1
    assert len(telemetry.m11_records) == 1
    assert len(parsed.motors[11].raw_ram_0x28_0x46_hex) == (
        MIN_STATE_LENGTH - TORQUE_ENABLE
    ) * 2
    asyncio.run(self_test_start_interrupt_guard())
    print("matdog_headless_auto_calibrate self-test: PASS")


async def main_async(args: argparse.Namespace) -> int:
    evidence = EvidenceWriter(args.output_dir)
    run = HeadlessRun(args, evidence)
    loop = asyncio.get_running_loop()
    installed_signals: list[signal.Signals] = []
    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(signum, run.request_stop)
            installed_signals.append(signum)
        except NotImplementedError:
            pass

    report: dict[str, Any]
    try:
        report = await run.execute()
    except Exception as exc:
        run.fault(f"fatal runner error: {exc}")
        recovered_postflight: dict[str, Any] | None = None
        if run.start_attempted:
            try:
                await run.send_stop_once("top-level fail-closed recovery")
                if run.terminal_frame is None:
                    await run.monitor_until_terminal()
                recovered_postflight = await run.postflight()
            except Exception as recovery_exc:
                recovery_message = f"top-level safety recovery failed: {recovery_exc}"
                if recovery_message not in run.faults:
                    run.faults.append(recovery_message)
        report = {
            "schema": "matdog.lf.headless_auto_calibrate.v1",
            "generated_at_utc": utc_now(),
            "result": "FAIL",
            "server": args.server,
            "bus_serial": args.bus_serial,
            "expected_motor_ids": EXPECTED_MOTOR_IDS,
            "start_attempted": run.start_attempted,
            "stop_attempted": run.stop_attempted,
            "terminal": (
                asdict(run.terminal_frame.calibration) if run.terminal_frame else None
            ),
            "postflight": recovered_postflight,
            "global_torque_off_verified": bool(
                recovered_postflight and recovered_postflight.get("verified")
            ),
            "faults": run.faults,
            "evidence_io_errors": evidence.io_errors,
            "station_process": run.station_provenance,
            "thermal_series_summary": run.telemetry.summary(),
            "eeprom_writes_sent": False,
            "register_writes_sent_by_runner": False,
            "start_commands_attempted": int(run.start_attempted),
            "stop_commands_attempted": int(run.stop_attempted),
        }
    try:
        run.telemetry_evidence = await asyncio.to_thread(
            evidence.write_complete_telemetry,
            run.telemetry,
        )
        report["m11_complete_telemetry_evidence"] = run.telemetry_evidence
    except Exception as telemetry_exc:
        run.fault(f"complete telemetry persistence failed: {telemetry_exc}")
        report["result"] = "FAIL"
        report["m11_complete_telemetry_evidence"] = None

    shutdown_result: dict[str, Any]
    if run.station_identity is None:
        shutdown_result = {
            "pid": args.station_pid,
            "stopped": False,
            "graceful": False,
            "sigint_sent": False,
            "sigkill_sent": False,
            "error": "Station identity was not verified; runner refused to signal it",
        }
        run.fault(shutdown_result["error"])
    else:
        try:
            shutdown_result = await controlled_station_shutdown(
                run.station_identity,
                timeout_s=args.station_shutdown_timeout,
            )
            evidence.emit("station_shutdown", **shutdown_result)
            if not shutdown_result.get("stopped"):
                run.fault("Station did not stop after controlled shutdown")
            elif not shutdown_result.get("graceful"):
                run.fault("Station required the documented SIGKILL fallback")
        except Exception as shutdown_exc:
            shutdown_result = {
                "pid": args.station_pid,
                "stopped": False,
                "graceful": False,
                "sigint_sent": False,
                "sigkill_sent": False,
                "error": f"controlled Station shutdown failed: {shutdown_exc}",
            }
            run.fault(shutdown_result["error"])

    report["station_shutdown"] = shutdown_result
    report["faults"] = run.faults
    report["evidence_io_errors"] = evidence.io_errors
    report["thermal_series_summary"] = run.telemetry.summary()
    if run.faults or evidence.io_errors:
        report["result"] = "FAIL"

    for signum in installed_signals:
        loop.remove_signal_handler(signum)

    evidence.finalize(report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["result"] == "PASS" else 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run and prove the one-shot MATDOG LF native calibrator"
    )
    parser.add_argument("--server", default="127.0.0.1:8888")
    parser.add_argument("--bus-serial", default=EXPECTED_BUS_SERIAL)
    parser.add_argument("--preflight-frames", type=int, default=10)
    parser.add_argument("--preflight-seconds", type=float, default=1.0)
    parser.add_argument("--frame-timeout", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--cleanup-timeout", type=float, default=90.0)
    parser.add_argument("--station-pid", type=int)
    parser.add_argument(
        "--expected-station-sha256",
        default=EXPECTED_STATION_SHA256,
    )
    parser.add_argument("--station-shutdown-timeout", type=float, default=30.0)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return args
    if args.output_dir is None:
        parser.error("--output-dir is required unless --self-test is used")
    if args.station_pid is None or args.station_pid <= 0:
        parser.error("--station-pid must identify the single live Station process")
    if args.bus_serial != EXPECTED_BUS_SERIAL:
        parser.error(f"--bus-serial must be exactly {EXPECTED_BUS_SERIAL}")
    if args.expected_station_sha256 != EXPECTED_STATION_SHA256:
        parser.error(
            "--expected-station-sha256 must be exactly "
            f"{EXPECTED_STATION_SHA256}"
        )
    if args.preflight_frames < 10:
        parser.error("--preflight-frames must be at least 10")
    if args.preflight_seconds < 1.0:
        parser.error("--preflight-seconds must be at least 1.0")
    for name in (
        "frame_timeout",
        "timeout",
        "cleanup_timeout",
        "station_shutdown_timeout",
    ):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    return args


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    return asyncio.run(main_async(args))


if __name__ == "__main__":
    raise SystemExit(main())
