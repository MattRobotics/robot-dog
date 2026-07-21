#!/usr/bin/env python3
"""
MATDOG — bridge telemetrico read-only Station -> end-stop detector.

Responsabilità:
- individuare un motore nel frame st3215/inference;
- estrarre timestamp del singolo motore;
- leggere posizione, velocità raw, corrente raw, stato e torque;
- leggere command_id e risultato di last_command;
- applicare la barriera:
    CR_SUCCESS osservato
    + successivo campione motore con timestamp strettamente più recente.

Questo modulo NON:
- crea un client Station;
- apre seriali;
- accoda comandi;
- costruisce write/sync_write;
- abilita torque;
- invia goal;
- scrive EEPROM.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import struct
import sys


NORMACORE = Path.home() / "norma-core"
sys.path.insert(0, str(NORMACORE))

from target.gen_python.protobuf.drivers.st3215 import st3215

from matdog_endstop_contact_detector import TelemetrySample


RAM_TORQUE_ENABLE = 0x28
RAM_GOAL_POSITION = 0x2A
RAM_PRESENT_POSITION = 0x38
RAM_PRESENT_SPEED = 0x3A
RAM_STATUS = 0x40
RAM_PRESENT_CURRENT = 0x45

MAX_ANGLE_STEP = 4095
SIGN_BIT_MASK = 0x8000


class TelemetryContractError(RuntimeError):
    pass


class CommandBarrierState(str, Enum):
    WAIT_COMMAND_RESULT = "WAIT_COMMAND_RESULT"
    WAIT_FRESH_MOTOR_SAMPLE = "WAIT_FRESH_MOTOR_SAMPLE"
    READY = "READY"
    REJECTED = "REJECTED"
    FAILED = "FAILED"
    HARD_ABORT = "HARD_ABORT"


@dataclass(frozen=True)
class StationMotorSnapshot:
    motor_id: int
    monotonic_stamp_ns: int
    system_stamp_ns: int
    app_start_id: int
    present_tick: int
    present_speed_raw: int
    current_raw: int
    error_status: int
    torque_enabled: bool
    goal_tick: int
    last_command_id: bytes
    last_command_result: st3215.CommandResult | None


@dataclass(frozen=True)
class CommandBarrierObservation:
    state: CommandBarrierState
    reason: str
    command_id: bytes
    motor_stamp_ns: int
    success_observed_stamp_ns: int | None
    app_start_id: int


def _u8(state: bytes, address: int) -> int:
    if len(state) <= address:
        raise TelemetryContractError(
            f"Stato ST3215 troppo corto per 0x{address:02X}: "
            f"{len(state)} byte"
        )

    return int(state[address])


def _u16(state: bytes, address: int) -> int:
    if len(state) < address + 2:
        raise TelemetryContractError(
            f"Stato ST3215 troppo corto per 0x{address:02X}: "
            f"{len(state)} byte"
        )

    return int(struct.unpack_from("<H", state, address)[0])


def normalize_present_position(raw: int) -> int:
    """Normalizza il readback posizione secondo il parser Station esistente.

    Questa funzione riguarda esclusivamente il PRESENT_POSITION ricevuto.
    Non implementa né autorizza GOAL_POSITION signed-wrap.
    """
    if raw & SIGN_BIT_MASK:
        magnitude = raw & MAX_ANGLE_STEP
        return (
            MAX_ANGLE_STEP + 1 - magnitude
        ) & MAX_ANGLE_STEP

    return raw & MAX_ANGLE_STEP


def find_motor_reader(
    inference_state,
    bus_serial: str,
    motor_id: int,
):
    if inference_state is None:
        return None

    for bus in inference_state.get_buses() or []:
        bus_info = bus.get_bus()

        if (
            bus_info is None
            or bus_info.get_serial_number() != bus_serial
        ):
            continue

        for motor in bus.get_motors() or []:
            if int(motor.get_id()) == motor_id:
                return motor

    return None


def parse_motor_snapshot(motor_reader) -> StationMotorSnapshot:
    motor_id = int(motor_reader.get_id())

    if motor_id <= 0:
        raise TelemetryContractError(
            f"Motor ID non valido: {motor_id}"
        )

    monotonic_stamp_ns = int(
        motor_reader.get_monotonic_stamp_ns()
    )
    system_stamp_ns = int(
        motor_reader.get_system_stamp_ns()
    )
    app_start_id = int(
        motor_reader.get_app_start_id()
    )

    if monotonic_stamp_ns < 0:
        raise TelemetryContractError(
            "monotonic_stamp_ns negativo"
        )

    if system_stamp_ns < 0:
        raise TelemetryContractError(
            "system_stamp_ns negativo"
        )

    state = bytes(motor_reader.get_state())

    present_tick = normalize_present_position(
        _u16(state, RAM_PRESENT_POSITION)
    )
    goal_tick = normalize_present_position(
        _u16(state, RAM_GOAL_POSITION)
    )

    last_command_id = b""
    last_command_result = None

    last_command = motor_reader.get_last_command()

    if last_command is not None:
        command = last_command.get_command()

        if command is not None:
            last_command_id = bytes(
                command.get_command_id()
            )
            last_command_result = last_command.get_result()

    return StationMotorSnapshot(
        motor_id=motor_id,
        monotonic_stamp_ns=monotonic_stamp_ns,
        system_stamp_ns=system_stamp_ns,
        app_start_id=app_start_id,
        present_tick=present_tick,
        present_speed_raw=_u16(
            state,
            RAM_PRESENT_SPEED,
        ),
        current_raw=_u16(
            state,
            RAM_PRESENT_CURRENT,
        ),
        error_status=_u8(
            state,
            RAM_STATUS,
        ),
        torque_enabled=(
            _u8(state, RAM_TORQUE_ENABLE) != 0
        ),
        goal_tick=goal_tick,
        last_command_id=last_command_id,
        last_command_result=last_command_result,
    )


def detector_sample(
    snapshot: StationMotorSnapshot,
) -> TelemetrySample:
    return TelemetrySample(
        monotonic_stamp_ns=snapshot.monotonic_stamp_ns,
        present_tick=snapshot.present_tick,
        current_raw=snapshot.current_raw,
        error_status=snapshot.error_status,
    )


class CommandBarrier:
    """Barriera deterministica command-result -> fresh motor sample."""

    def __init__(
        self,
        *,
        command_id: bytes,
        motor_id: int,
    ):
        command_id = bytes(command_id)

        if not command_id:
            raise ValueError("command_id non può essere vuoto")

        if motor_id <= 0:
            raise ValueError("motor_id deve essere > 0")

        self.command_id = command_id
        self.motor_id = motor_id
        self.success_observed_stamp_ns: int | None = None
        self.app_start_id: int | None = None
        self.state = CommandBarrierState.WAIT_COMMAND_RESULT

    def _observation(
        self,
        snapshot: StationMotorSnapshot,
        state: CommandBarrierState,
        reason: str,
    ) -> CommandBarrierObservation:
        self.state = state

        return CommandBarrierObservation(
            state=state,
            reason=reason,
            command_id=self.command_id,
            motor_stamp_ns=snapshot.monotonic_stamp_ns,
            success_observed_stamp_ns=(
                self.success_observed_stamp_ns
            ),
            app_start_id=snapshot.app_start_id,
        )

    def observe(
        self,
        snapshot: StationMotorSnapshot,
    ) -> CommandBarrierObservation:
        if snapshot.motor_id != self.motor_id:
            return self._observation(
                snapshot,
                CommandBarrierState.HARD_ABORT,
                "UNEXPECTED_MOTOR_ID",
            )

        if snapshot.monotonic_stamp_ns <= 0:
            return self._observation(
                snapshot,
                CommandBarrierState.HARD_ABORT,
                "INVALID_MOTOR_TIMESTAMP",
            )

        if self.app_start_id is None:
            self.app_start_id = snapshot.app_start_id
        elif snapshot.app_start_id != self.app_start_id:
            return self._observation(
                snapshot,
                CommandBarrierState.HARD_ABORT,
                "STATION_APP_RESTART_DETECTED",
            )

        if self.state in {
            CommandBarrierState.READY,
            CommandBarrierState.REJECTED,
            CommandBarrierState.FAILED,
            CommandBarrierState.HARD_ABORT,
        }:
            return self._observation(
                snapshot,
                self.state,
                "TERMINAL_STATE_LATCHED",
            )

        if self.success_observed_stamp_ns is not None:
            if snapshot.last_command_id != self.command_id:
                return self._observation(
                    snapshot,
                    CommandBarrierState.HARD_ABORT,
                    "COMMAND_OVERWRITTEN_BEFORE_FRESH_SAMPLE",
                )

            if (
                snapshot.monotonic_stamp_ns
                > self.success_observed_stamp_ns
            ):
                return self._observation(
                    snapshot,
                    CommandBarrierState.READY,
                    "COMMAND_SUCCESS_AND_FRESH_SAMPLE",
                )

            return self._observation(
                snapshot,
                CommandBarrierState.WAIT_FRESH_MOTOR_SAMPLE,
                "WAITING_STRICTLY_NEWER_MOTOR_TIMESTAMP",
            )

        if snapshot.last_command_id != self.command_id:
            return self._observation(
                snapshot,
                CommandBarrierState.WAIT_COMMAND_RESULT,
                "COMMAND_ID_NOT_OBSERVED",
            )

        result = snapshot.last_command_result

        if result == st3215.CommandResult.CR_SUCCESS:
            self.success_observed_stamp_ns = (
                snapshot.monotonic_stamp_ns
            )
            return self._observation(
                snapshot,
                CommandBarrierState.WAIT_FRESH_MOTOR_SAMPLE,
                "COMMAND_SUCCESS_OBSERVED",
            )

        if result == st3215.CommandResult.CR_REJECTED:
            return self._observation(
                snapshot,
                CommandBarrierState.REJECTED,
                "COMMAND_REJECTED",
            )

        if result == st3215.CommandResult.CR_FAILED:
            return self._observation(
                snapshot,
                CommandBarrierState.FAILED,
                "COMMAND_FAILED",
            )

        return self._observation(
            snapshot,
            CommandBarrierState.WAIT_COMMAND_RESULT,
            "COMMAND_PROCESSING",
        )
