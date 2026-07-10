#!/usr/bin/env python3
"""Unified MATDOG ST3215 digital-zero calibration.

Workflow:
  1. Manually place all 12 joints in the mechanical q=0 pose, torque OFF.
  2. Run ``preflight`` to capture a stable pose and create an immutable plan.
  3. Run ``execute`` once to apply the 12 EEPROM offsets serially.
  4. Restart Station, then run ``verify`` for authoritative EEPROM read-back.

Station remains the sole serial owner. This tool never sends TorqueEnable or
GoalPosition commands.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import struct
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[3]
NORMACORE = Path.home() / "norma-core"
EXAMPLE_DIR = NORMACORE / "software/station/examples/st3215-remote-teleop-py"

sys.path.insert(0, str(NORMACORE))
sys.path.insert(0, str(EXAMPLE_DIR))

from software.station.shared.station_py import (  # noqa: E402
    new_station_client,
    send_commands,
)
from target.gen_python.protobuf.station import (  # noqa: E402
    commands as station_commands,
    drivers,
)
from target.gen_python.protobuf.drivers.st3215 import st3215  # noqa: E402

SERVER = "localhost:8888"
BUS_SERIAL = "5B14114953"
TARGET_DISPLAYED = 2048

MOTOR_ORDER = [
    13, 12, 11,      # LF
    33, 32, 31,      # RH
    43, 42, 41,      # LH
    23, 22, 21,      # RF last
]

JOINTS = {
    13: "LF_HIP",
    12: "LF_UPPER",
    11: "LF_LOWER",
    23: "RF_HIP",
    22: "RF_UPPER",
    21: "RF_LOWER",
    33: "RH_HIP",
    32: "RH_UPPER",
    31: "RH_LOWER",
    43: "LH_HIP",
    42: "LH_UPPER",
    41: "LH_LOWER",
}

EEPROM_OFFSET = 0x1F
RAM_TORQUE_ENABLE = 0x28
RAM_LOCK = 0x37
RAM_PRESENT_POSITION = 0x38
LOCKED = 1
UNLOCKED = 0
STATE_MIN_BYTES = 71

COMMAND_TIMEOUT_S = 5.0
REGISTER_TIMEOUT_S = 5.0
POSITION_TIMEOUT_S = 6.0
DEFAULT_Q0_TOLERANCE = 10
DEFAULT_STABILITY_SPREAD = 2
DEFAULT_SAMPLES = 20
CONFIRM_TEXT = "WRITE ALL 12 DIGITAL ZERO OFFSETS"


class CalibrationError(RuntimeError):
    """Safety or validation failure."""


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def timestamp_utc() -> str:
    return utc_now().strftime("%Y-%m-%d_%H%M%SZ")


def circular_delta(current: int, reference: int) -> int:
    return ((current - reference + 2048) % 4096) - 2048


def circular_center(values: list[int]) -> int:
    if not values:
        raise CalibrationError("Campioni encoder mancanti")
    reference = values[0]
    unwrapped = [reference + circular_delta(value, reference) for value in values]
    ordered = sorted(unwrapped)
    center = ordered[len(ordered) // 2]
    return center % 4096


def circular_spread(values: list[int], center: int) -> int:
    return max(abs(circular_delta(value, center)) for value in values)


def signed_offset_for_target(raw_q0: int, target: int = TARGET_DISPLAYED) -> int:
    return circular_delta(raw_q0, target)


def normal_position(raw: int) -> int:
    if raw & 0x8000:
        magnitude = raw & 0x0FFF
        return (4096 - magnitude) & 0x0FFF
    return raw & 0x0FFF


def resolve_repo_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = REPO / path
    return path.resolve()


def relative_or_absolute(path: Path) -> str:
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def sha256_hex(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_sha256(path: Path) -> Path:
    digest = sha256_hex(path)
    sha_path = Path(f"{path}.sha256")
    sha_path.write_text(f"{digest}  {path.name}\n", encoding="utf-8")
    return sha_path


def verify_sha256(path: Path) -> None:
    sha_path = Path(f"{path}.sha256")
    if not sha_path.is_file():
        raise CalibrationError(f"SHA256 mancante: {sha_path}")
    expected = sha_path.read_text(encoding="utf-8").split()[0].lower()
    actual = sha256_hex(path)
    if actual != expected:
        raise CalibrationError(
            f"SHA256 non valido per {path}: expected={expected}, actual={actual}"
        )


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_sha256(path)


def load_json_verified(path: Path) -> dict[str, Any]:
    verify_sha256(path)
    return json.loads(path.read_text(encoding="utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="MATDOG unified 12-servo digital-zero calibration"
    )
    parser.add_argument("--server", default=SERVER)
    parser.add_argument("--bus-serial", default=BUS_SERIAL)
    sub = parser.add_subparsers(dest="mode", required=True)

    preflight = sub.add_parser(
        "preflight",
        help="Read-only capture and immutable 12-servo calibration plan",
    )
    preflight.add_argument(
        "--output-dir",
        default="09_Logs/Calibration/Digital_Zero",
    )
    preflight.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    preflight.add_argument(
        "--stability-spread",
        type=int,
        default=DEFAULT_STABILITY_SPREAD,
    )
    preflight.add_argument(
        "--q0-tolerance",
        type=int,
        default=DEFAULT_Q0_TOLERANCE,
    )

    execute = sub.add_parser(
        "execute",
        help="Apply the plan serially to all 12 servos",
    )
    execute.add_argument("--plan", required=True)
    execute.add_argument("--confirm", required=True)
    execute.add_argument(
        "--q0-tolerance",
        type=int,
        default=DEFAULT_Q0_TOLERANCE,
    )

    verify = sub.add_parser(
        "verify",
        help="Final read-only EEPROM verification after Station restart",
    )
    verify.add_argument("--plan", required=True)
    verify.add_argument("--samples", type=int, default=21)
    verify.add_argument(
        "--q0-tolerance",
        type=int,
        default=DEFAULT_Q0_TOLERANCE,
    )

    return parser.parse_args()


def selected_bus(state: Any, bus_serial: str) -> Any:
    for bus in state.get_buses() or []:
        info = bus.get_bus()
        if info and info.get_serial_number() == bus_serial:
            return bus
    raise CalibrationError(f"Bus ST3215 non trovato: {bus_serial}")


def motor_map(state: Any, bus_serial: str) -> dict[int, Any]:
    bus = selected_bus(state, bus_serial)
    return {int(motor.get_id()): motor for motor in bus.get_motors() or []}


async def next_state(queue: asyncio.Queue, timeout: float = 10.0) -> Any:
    entry = await asyncio.wait_for(queue.get(), timeout=timeout)
    if entry is None:
        raise CalibrationError("Stream st3215/inference chiuso")
    return st3215.InferenceStateReader(entry.Data)


async def fresh_complete_state(
    queue: asyncio.Queue,
    bus_serial: str,
    timeout: float = 12.0,
) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = await next_state(
            queue,
            timeout=max(0.1, deadline - time.monotonic()),
        )
        try:
            motors = motor_map(state, bus_serial)
        except CalibrationError:
            continue
        if all(
            motor_id in motors
            and len(bytes(motors[motor_id].get_state())) >= STATE_MIN_BYTES
            for motor_id in MOTOR_ORDER
        ):
            return state
    raise CalibrationError("Inference completa dei 12 motori non disponibile")


def state_values(state: Any, motor_id: int, bus_serial: str) -> dict[str, Any]:
    motors = motor_map(state, bus_serial)
    if motor_id not in motors:
        raise CalibrationError(f"M{motor_id} non presente nell'inference")
    data = bytes(motors[motor_id].get_state())
    if len(data) < STATE_MIN_BYTES:
        raise CalibrationError(f"M{motor_id}: stato incompleto, {len(data)} byte")
    present_raw = struct.unpack_from("<H", data, RAM_PRESENT_POSITION)[0]
    present = normal_position(present_raw)
    offset = struct.unpack_from("<h", data, EEPROM_OFFSET)[0]
    return {
        "present": present,
        "offset": offset,
        "raw_unoffset": (present + offset) % 4096,
        "torque": int(data[RAM_TORQUE_ENABLE]),
        "lock": int(data[RAM_LOCK]),
        "state_hex": data.hex(),
    }


async def capture_samples(
    queue: asyncio.Queue,
    bus_serial: str,
    count: int,
) -> dict[int, dict[str, list[int] | str]]:
    if count < 3:
        raise CalibrationError("Servono almeno 3 campioni")
    captured: dict[int, dict[str, list[int] | str]] = {
        motor_id: {
            "present": [],
            "raw": [],
            "offset": [],
            "lock": [],
            "torque": [],
            "state_hex": "",
        }
        for motor_id in MOTOR_ORDER
    }
    for _ in range(count):
        state = await fresh_complete_state(queue, bus_serial)
        for motor_id in MOTOR_ORDER:
            values = state_values(state, motor_id, bus_serial)
            captured[motor_id]["present"].append(values["present"])
            captured[motor_id]["raw"].append(values["raw_unoffset"])
            captured[motor_id]["offset"].append(values["offset"])
            captured[motor_id]["lock"].append(values["lock"])
            captured[motor_id]["torque"].append(values["torque"])
            captured[motor_id]["state_hex"] = values["state_hex"]
    return captured


def make_driver_command(
    *,
    command_id: bytes,
    body: st3215.Command,
) -> station_commands.DriverCommand:
    return station_commands.DriverCommand(
        command_id=command_id,
        type=drivers.StationCommandType.STC_ST3215_COMMAND,
        body=body.encode(),
    )


async def wait_command_result(
    queue: asyncio.Queue,
    motor_id: int,
    command_id: bytes,
    label: str,
    bus_serial: str,
) -> Any:
    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    while time.monotonic() < deadline:
        state = await next_state(
            queue,
            timeout=max(0.1, deadline - time.monotonic()),
        )
        try:
            motors = motor_map(state, bus_serial)
        except CalibrationError:
            continue
        motor = motors.get(motor_id)
        if motor is None:
            continue
        last = motor.get_last_command()
        command = last.get_command()
        if bytes(command.get_command_id()) != command_id:
            continue
        result = last.get_result()
        if result == st3215.CommandResult.CR_SUCCESS:
            return state
        if result == st3215.CommandResult.CR_REJECTED:
            raise CalibrationError(f"{label}: CR_REJECTED")
        if result == st3215.CommandResult.CR_FAILED:
            raise CalibrationError(f"{label}: CR_FAILED")
    raise CalibrationError(f"{label}: timeout comando")


async def send_and_wait(
    client: Any,
    queue: asyncio.Queue,
    motor_id: int,
    label: str,
    body: st3215.Command,
    bus_serial: str,
) -> Any:
    command_id = uuid.uuid4().bytes
    wrapped = make_driver_command(command_id=command_id, body=body)
    await send_commands(client, [wrapped])
    return await wait_command_result(
        queue,
        motor_id,
        command_id,
        label,
        bus_serial,
    )


async def wait_register(
    queue: asyncio.Queue,
    motor_id: int,
    address: int,
    expected: bytes,
    label: str,
    bus_serial: str,
) -> Any:
    deadline = time.monotonic() + REGISTER_TIMEOUT_S
    while time.monotonic() < deadline:
        state = await next_state(
            queue,
            timeout=max(0.1, deadline - time.monotonic()),
        )
        try:
            motors = motor_map(state, bus_serial)
        except CalibrationError:
            continue
        motor = motors.get(motor_id)
        if motor is None:
            continue
        data = bytes(motor.get_state())
        end = address + len(expected)
        if len(data) >= end and data[address:end] == expected:
            return state
    raise CalibrationError(
        f"{label}: read-back timeout, expected={expected.hex()}"
    )


async def send_direct_write_verified(
    client: Any,
    queue: asyncio.Queue,
    motor_id: int,
    address: int,
    value: bytes,
    label: str,
    bus_serial: str,
    attempts: int = 3,
) -> Any:
    last_error: Exception | None = None
    for attempt in range(1, attempts + 1):
        body = st3215.Command(
            target_bus_serial=bus_serial,
            write=st3215.ST3215WriteCommand(
                motor_id=motor_id,
                address=address,
                value=value,
            ),
        )
        try:
            await send_and_wait(
                client,
                queue,
                motor_id,
                f"{label} attempt {attempt}",
                body,
                bus_serial,
            )
            return await wait_register(
                queue,
                motor_id,
                address,
                value,
                label,
                bus_serial,
            )
        except Exception as exc:  # retry only direct lock/unlock writes
            last_error = exc
            await asyncio.sleep(0.15)
    raise CalibrationError(f"{label}: {last_error}")


async def send_reg_write_offset(
    client: Any,
    queue: asyncio.Queue,
    motor_id: int,
    offset: int,
    bus_serial: str,
) -> None:
    body = st3215.Command(
        target_bus_serial=bus_serial,
        reg_write=st3215.ST3215RegWriteCommand(
            motor_id=motor_id,
            address=EEPROM_OFFSET,
            value=struct.pack("<h", offset),
        ),
    )
    await send_and_wait(
        client,
        queue,
        motor_id,
        f"M{motor_id} REG_WRITE OFFSET",
        body,
        bus_serial,
    )


async def send_action(
    client: Any,
    queue: asyncio.Queue,
    motor_id: int,
    bus_serial: str,
) -> str:
    body = st3215.Command(
        target_bus_serial=bus_serial,
        action=st3215.ST3215ActionCommand(motor_id=motor_id),
    )
    try:
        await send_and_wait(
            client,
            queue,
            motor_id,
            f"M{motor_id} ACTION OFFSET",
            body,
            bus_serial,
        )
        return "CR_SUCCESS"
    except CalibrationError as exc:
        text = str(exc)
        if "CR_FAILED" in text or "timeout comando" in text:
            return "RESPONSE_UNCERTAIN_POSITION_VERIFICATION_REQUIRED"
        raise


async def wait_displayed_position(
    queue: asyncio.Queue,
    motor_id: int,
    expected: int,
    tolerance: int,
    bus_serial: str,
) -> Any:
    deadline = time.monotonic() + POSITION_TIMEOUT_S
    last_present: int | None = None
    while time.monotonic() < deadline:
        state = await next_state(
            queue,
            timeout=max(0.1, deadline - time.monotonic()),
        )
        try:
            values = state_values(state, motor_id, bus_serial)
        except CalibrationError as exc:
            if "stato incompleto" in str(exc) or "non presente" in str(exc):
                continue
            raise
        last_present = int(values["present"])
        if abs(circular_delta(last_present, expected)) <= tolerance:
            return state
    raise CalibrationError(
        f"M{motor_id}: posizione non ricentrata; "
        f"expected={expected}, last={last_present}"
    )


async def shutdown_client(client: Any | None) -> None:
    if client is not None:
        try:
            client.close_connection()
        except Exception:
            pass
    current = asyncio.current_task()
    tasks = [task for task in asyncio.all_tasks() if task is not current]
    for task in tasks:
        task.cancel()
    if tasks:
        await asyncio.gather(*tasks, return_exceptions=True)


def assert_no_stream_errors(errors: asyncio.Queue) -> None:
    if not errors.empty():
        raise CalibrationError(errors.get_nowait())


def plan_motor(plan: dict[str, Any], motor_id: int) -> dict[str, Any]:
    try:
        return plan["motors"][str(motor_id)]
    except KeyError as exc:
        raise CalibrationError(f"M{motor_id} assente dal piano") from exc


async def run_preflight(args: argparse.Namespace, client: Any) -> None:
    queue: asyncio.Queue = asyncio.Queue()
    errors = client.follow("st3215/inference", queue)
    captured = await capture_samples(queue, args.bus_serial, args.samples)
    assert_no_stream_errors(errors)

    stamp = timestamp_utc()
    output_dir = resolve_repo_path(args.output_dir)
    backup_path = output_dir / f"{stamp}_st3215_pre_recenter_backup.json"
    q0_path = output_dir / f"{stamp}_mechanical_q0_snapshot.json"
    plan_path = output_dir / f"{stamp}_digital_zero_plan.json"

    backup_motors: dict[str, Any] = {}
    q0_motors: dict[str, Any] = {}
    plan_motors: dict[str, Any] = {}
    failures: list[str] = []

    print("=== MATDOG 12-SERVO DIGITAL ZERO PREFLIGHT ===")
    print(
        "ID  JOINT       POS_Q0  RAW_Q0  SPREAD  OLD_OFFSET  "
        "NEW_OFFSET  PREDICTED  LOCK  TORQUE  RESULT"
    )

    for motor_id in MOTOR_ORDER:
        sample = captured[motor_id]
        present_values = [int(v) for v in sample["present"]]
        raw_values = [int(v) for v in sample["raw"]]
        offset_values = sorted(set(int(v) for v in sample["offset"]))
        lock_values = sorted(set(int(v) for v in sample["lock"]))
        torque_values = sorted(set(int(v) for v in sample["torque"]))

        present_q0 = circular_center(present_values)
        raw_q0 = circular_center(raw_values)
        spread = circular_spread(raw_values, raw_q0)
        old_offset = offset_values[0] if len(offset_values) == 1 else None
        new_offset = signed_offset_for_target(raw_q0)
        predicted = (raw_q0 - new_offset) % 4096

        passed = (
            old_offset is not None
            and lock_values == [LOCKED]
            and torque_values == [0]
            and spread <= args.stability_spread
            and abs(circular_delta(predicted, TARGET_DISPLAYED))
            <= args.q0_tolerance
        )
        if not passed:
            failures.append(f"M{motor_id}")

        backup_motors[str(motor_id)] = {
            "joint": JOINTS[motor_id],
            "state_hex": sample["state_hex"],
            "offset_values": offset_values,
            "lock_values": lock_values,
            "torque_values": torque_values,
        }
        q0_motors[str(motor_id)] = {
            "joint": JOINTS[motor_id],
            "present_samples": present_values,
            "raw_samples": raw_values,
            "present_median": present_q0,
            "raw_encoder_unoffset_from_median": raw_q0,
            "raw_spread_ticks": spread,
        }
        plan_motors[str(motor_id)] = {
            "joint": JOINTS[motor_id],
            "old_offset_signed_i16": old_offset,
            "new_offset_signed_i16": new_offset,
            "raw_encoder_q0": raw_q0,
            "present_q0": present_q0,
            "predicted_displayed_after_write": predicted,
        }

        old_text = f"{old_offset:+d}" if old_offset is not None else str(offset_values)
        print(
            f"{motor_id:02d}  {JOINTS[motor_id]:<10} "
            f"{present_q0:6d}  {raw_q0:6d}  {spread:6d}  "
            f"{old_text:>10}  {new_offset:+10d}  {predicted:9d}  "
            f"{str(lock_values):>4}  {str(torque_values):>6}  "
            f"{'PASS' if passed else 'FAIL'}"
        )

    if failures:
        print(f"\nRESULT: FAIL — preflight bloccato: {failures}")
        raise CalibrationError("Preflight globale non superato")

    backup = {
        "schema": "matdog.st3215.pre_recenter_backup.v2",
        "created_at_utc": utc_now().isoformat(),
        "hardware_operation": "read_only",
        "bus_serial": args.bus_serial,
        "motors": backup_motors,
    }
    write_json(backup_path, backup)

    q0 = {
        "schema": "matdog.st3215.mechanical_q0_snapshot.v2",
        "created_at_utc": utc_now().isoformat(),
        "hardware_operation": "read_only",
        "bus_serial": args.bus_serial,
        "samples_per_motor": args.samples,
        "stability_spread_limit_ticks": args.stability_spread,
        "motors": q0_motors,
    }
    write_json(q0_path, q0)

    plan = {
        "schema": "matdog.st3215.digital_zero_plan.v2",
        "created_at_utc": utc_now().isoformat(),
        "write_performed": False,
        "bus_serial": args.bus_serial,
        "target_displayed_position": TARGET_DISPLAYED,
        "motor_order": MOTOR_ORDER,
        "confirmation_required": CONFIRM_TEXT,
        "q0_tolerance_ticks": args.q0_tolerance,
        "source_q0_snapshot": relative_or_absolute(q0_path),
        "source_pre_recenter_backup": relative_or_absolute(backup_path),
        "motors": plan_motors,
    }
    write_json(plan_path, plan)

    print("\nRESULT: PASS — preflight globale read-only completato.")
    print(f"Backup: {relative_or_absolute(backup_path)}")
    print(f"Q0 snapshot: {relative_or_absolute(q0_path)}")
    print(f"Plan: {relative_or_absolute(plan_path)}")
    print(f"Execution confirmation: {CONFIRM_TEXT}")
    print("MOTOR COMMANDS SENT: NO")


def classify_execute_state(
    current: dict[str, Any],
    item: dict[str, Any],
    tolerance: int,
) -> tuple[str, int, int]:
    old_offset = int(item["old_offset_signed_i16"])
    new_offset = int(item["new_offset_signed_i16"])
    q0_raw = int(item["raw_encoder_q0"])
    raw = int(current["raw_unoffset"])
    raw_delta = circular_delta(raw, q0_raw)
    display_delta = circular_delta(int(current["present"]), TARGET_DISPLAYED)

    if current["torque"] != 0:
        raise CalibrationError("torque attivo")
    if current["lock"] != LOCKED:
        raise CalibrationError("EEPROM non bloccata")
    if abs(raw_delta) > tolerance:
        raise CalibrationError(f"raw drift {raw_delta:+d} tick")
    if current["offset"] == old_offset:
        predicted = (raw - new_offset) % 4096
        if abs(circular_delta(predicted, TARGET_DISPLAYED)) > tolerance:
            raise CalibrationError(f"predicted display {predicted}")
        return "PENDING", raw_delta, predicted
    if current["offset"] == new_offset and abs(display_delta) <= tolerance:
        return "ALREADY_APPLIED", raw_delta, int(current["present"])
    raise CalibrationError(
        f"offset inatteso {current['offset']:+d}; "
        f"old={old_offset:+d}, new={new_offset:+d}"
    )


async def run_execute(args: argparse.Namespace, client: Any) -> None:
    if args.confirm != CONFIRM_TEXT:
        raise CalibrationError(
            f"Conferma errata. Richiesta esatta: {CONFIRM_TEXT!r}"
        )

    plan_path = resolve_repo_path(args.plan)
    plan = load_json_verified(plan_path)
    if plan.get("schema") != "matdog.st3215.digital_zero_plan.v2":
        raise CalibrationError("Schema piano non supportato")
    if plan.get("bus_serial") != args.bus_serial:
        raise CalibrationError("Bus serial diverso dal piano")

    q0_path = resolve_repo_path(plan["source_q0_snapshot"])
    backup_path = resolve_repo_path(plan["source_pre_recenter_backup"])
    load_json_verified(q0_path)
    load_json_verified(backup_path)

    queue: asyncio.Queue = asyncio.Queue()
    errors = client.follow("st3215/inference", queue)
    state = await fresh_complete_state(queue, args.bus_serial)
    assert_no_stream_errors(errors)

    print("=== MATDOG 12-SERVO DIGITAL ZERO EXECUTION PREFLIGHT ===")
    print(
        "ID  JOINT       POS   RAW   RAW_DELTA  OFFSET_NOW  "
        "NEW_OFFSET  PREDICTED  STATE"
    )

    initial_states: dict[int, str] = {}
    for motor_id in MOTOR_ORDER:
        current = state_values(state, motor_id, args.bus_serial)
        item = plan_motor(plan, motor_id)
        try:
            status, raw_delta, predicted = classify_execute_state(
                current,
                item,
                args.q0_tolerance,
            )
        except CalibrationError as exc:
            raise CalibrationError(f"M{motor_id}: {exc}") from exc
        initial_states[motor_id] = status
        print(
            f"{motor_id:02d}  {JOINTS[motor_id]:<10} "
            f"{current['present']:4d}  {current['raw_unoffset']:4d}  "
            f"{raw_delta:+9d}  {current['offset']:+10d}  "
            f"{int(item['new_offset_signed_i16']):+10d}  "
            f"{predicted:9d}  {status}"
        )

    audit_path = plan_path.parent / f"{timestamp_utc()}_digital_zero_execution.json"
    audit: dict[str, Any] = {
        "schema": "matdog.st3215.digital_zero_execution.v2",
        "created_at_utc": utc_now().isoformat(),
        "status": "in_progress",
        "bus_serial": args.bus_serial,
        "source_plan": relative_or_absolute(plan_path),
        "order": MOTOR_ORDER,
        "completed": [],
        "failed": None,
        "final_eeprom_readback_requires_station_restart": True,
        "goal_position_commands_sent": False,
        "torque_commands_sent": False,
    }
    write_json(audit_path, audit)

    try:
        for index, motor_id in enumerate(MOTOR_ORDER, start=1):
            item = plan_motor(plan, motor_id)
            state = await fresh_complete_state(queue, args.bus_serial)
            all_values = {
                mid: state_values(state, mid, args.bus_serial)
                for mid in MOTOR_ORDER
            }
            active = [
                mid for mid, values in all_values.items() if values["torque"] != 0
            ]
            if active:
                raise CalibrationError(f"Torque attivo su {active}")

            current = all_values[motor_id]
            try:
                status, raw_delta, predicted = classify_execute_state(
                    current,
                    item,
                    args.q0_tolerance,
                )
            except CalibrationError as exc:
                raise CalibrationError(f"M{motor_id}: {exc}") from exc

            print(
                f"\n[{index}/{len(MOTOR_ORDER)}] M{motor_id} "
                f"{JOINTS[motor_id]}: {status}"
            )

            if status == "ALREADY_APPLIED":
                record = {
                    "motor_id": motor_id,
                    "joint": JOINTS[motor_id],
                    "result": "ALREADY_APPLIED_VERIFIED",
                    "displayed": current["present"],
                    "offset": current["offset"],
                    "lock": current["lock"],
                    "torque": current["torque"],
                    "raw_delta": raw_delta,
                }
                audit["completed"].append(record)
                write_json(audit_path, audit)
                print(
                    f"PASS M{motor_id}: already applied, "
                    f"POS={current['present']}, LOCK=1, torque=0"
                )
                continue

            new_offset = int(item["new_offset_signed_i16"])
            unlocked = False
            after_action: dict[str, Any] | None = None
            action_result = "NOT_SENT"
            after_lock: dict[str, Any] | None = None

            try:
                await send_direct_write_verified(
                    client,
                    queue,
                    motor_id,
                    RAM_LOCK,
                    bytes([UNLOCKED]),
                    f"M{motor_id} UNLOCK EEPROM",
                    args.bus_serial,
                )
                unlocked = True
                await asyncio.sleep(0.10)

                await send_reg_write_offset(
                    client,
                    queue,
                    motor_id,
                    new_offset,
                    args.bus_serial,
                )
                await asyncio.sleep(0.10)

                action_result = await send_action(
                    client,
                    queue,
                    motor_id,
                    args.bus_serial,
                )
                if action_result != "CR_SUCCESS":
                    print(
                        f"M{motor_id}: risposta Action non conclusiva; "
                        "verifica posizione obbligatoria..."
                    )

                action_state = await wait_displayed_position(
                    queue,
                    motor_id,
                    predicted,
                    args.q0_tolerance,
                    args.bus_serial,
                )
                after_action = state_values(
                    action_state,
                    motor_id,
                    args.bus_serial,
                )
            finally:
                if unlocked:
                    locked_state = await send_direct_write_verified(
                        client,
                        queue,
                        motor_id,
                        RAM_LOCK,
                        bytes([LOCKED]),
                        f"M{motor_id} LOCK EEPROM",
                        args.bus_serial,
                    )
                    after_lock = state_values(
                        locked_state,
                        motor_id,
                        args.bus_serial,
                    )
                    if after_lock["lock"] != LOCKED:
                        raise CalibrationError(f"M{motor_id}: EEPROM non ribloccata")
                    if after_lock["torque"] != 0:
                        raise CalibrationError(f"M{motor_id}: torque non zero")

            if after_action is None or after_lock is None:
                raise CalibrationError(f"M{motor_id}: verifica finale incompleta")

            record = {
                "motor_id": motor_id,
                "joint": JOINTS[motor_id],
                "old_offset": int(item["old_offset_signed_i16"]),
                "new_offset": new_offset,
                "raw_before": current["raw_unoffset"],
                "raw_delta_before": raw_delta,
                "predicted_displayed": predicted,
                "displayed_after_action": after_action["present"],
                "action_result": action_result,
                "position_verification_passed": True,
                "lock_after": after_lock["lock"],
                "torque_after": after_lock["torque"],
                "result": "PASS_PENDING_STATION_RESTART_READBACK",
            }
            audit["completed"].append(record)
            write_json(audit_path, audit)
            print(
                f"PASS M{motor_id}: POS={after_action['present']}, "
                "LOCK=1, torque=0"
            )
            await asyncio.sleep(0.10)

    except Exception as exc:
        audit["status"] = "failed"
        audit["failed"] = {
            "error": str(exc),
            "completed_count": len(audit["completed"]),
        }
        write_json(audit_path, audit)
        print(f"\nBATCH ABORT: {exc}")
        print(f"Audit: {relative_or_absolute(audit_path)}")
        raise

    audit["status"] = "completed_pending_station_restart_readback"
    write_json(audit_path, audit)
    print("\nRESULT: PASS — batch digitale dei 12 servo completato.")
    print("NEXT: riavviare Station ed eseguire il comando verify.")
    print(f"Audit: {relative_or_absolute(audit_path)}")
    print("GOAL POSITION COMMANDS SENT: NO")
    print("TORQUE COMMANDS SENT: NO")


async def run_verify(args: argparse.Namespace, client: Any) -> None:
    plan_path = resolve_repo_path(args.plan)
    plan = load_json_verified(plan_path)
    if plan.get("schema") != "matdog.st3215.digital_zero_plan.v2":
        raise CalibrationError("Schema piano non supportato")
    if plan.get("bus_serial") != args.bus_serial:
        raise CalibrationError("Bus serial diverso dal piano")

    queue: asyncio.Queue = asyncio.Queue()
    errors = client.follow("st3215/inference", queue)
    captured = await capture_samples(queue, args.bus_serial, args.samples)
    assert_no_stream_errors(errors)

    print("=== MATDOG FINAL 12-SERVO EEPROM READ-BACK ===")
    print(
        "ID  JOINT       POS   RAW   Q0_RAW  RAW_DELTA  "
        "OFFSET  EXPECTED  LOCK  TORQUE  RESULT"
    )

    rows: list[dict[str, Any]] = []
    failures: list[int] = []
    for motor_id in MOTOR_ORDER:
        item = plan_motor(plan, motor_id)
        sample = captured[motor_id]
        present_values = [int(v) for v in sample["present"]]
        raw_values = [int(v) for v in sample["raw"]]
        offsets = sorted(set(int(v) for v in sample["offset"]))
        locks = sorted(set(int(v) for v in sample["lock"]))
        torques = sorted(set(int(v) for v in sample["torque"]))
        present = circular_center(present_values)
        raw = circular_center(raw_values)
        expected_offset = int(item["new_offset_signed_i16"])
        q0_raw = int(item["raw_encoder_q0"])
        raw_delta = circular_delta(raw, q0_raw)
        display_delta = circular_delta(present, TARGET_DISPLAYED)
        passed = (
            offsets == [expected_offset]
            and locks == [LOCKED]
            and torques == [0]
            and abs(raw_delta) <= args.q0_tolerance
            and abs(display_delta) <= args.q0_tolerance
        )
        if not passed:
            failures.append(motor_id)
        row = {
            "motor_id": motor_id,
            "joint": JOINTS[motor_id],
            "present_median": present,
            "display_delta_from_2048": display_delta,
            "raw_median": raw,
            "q0_raw": q0_raw,
            "raw_delta": raw_delta,
            "offset_values": offsets,
            "expected_offset": expected_offset,
            "lock_values": locks,
            "torque_values": torques,
            "result": "PASS" if passed else "FAIL",
        }
        rows.append(row)
        offset_text = f"{offsets[0]:+d}" if len(offsets) == 1 else str(offsets)
        lock_text = str(locks[0]) if len(locks) == 1 else str(locks)
        torque_text = str(torques[0]) if len(torques) == 1 else str(torques)
        print(
            f"{motor_id:02d}  {JOINTS[motor_id]:<10} "
            f"{present:4d}  {raw:4d}  {q0_raw:6d}  {raw_delta:+9d}  "
            f"{offset_text:>7}  {expected_offset:+8d}  "
            f"{lock_text:>4}  {torque_text:>6}  "
            f"{'PASS' if passed else 'FAIL'}"
        )

    audit_path = plan_path.parent / f"{timestamp_utc()}_final_12_offset_readback.json"
    audit = {
        "schema": "matdog.st3215.final_12_offset_readback.v2",
        "created_at_utc": utc_now().isoformat(),
        "status": "PASS" if not failures else "FAIL",
        "hardware_operation": "read_only",
        "samples": args.samples,
        "tolerance_ticks": args.q0_tolerance,
        "source_plan": relative_or_absolute(plan_path),
        "motors": rows,
        "failures": failures,
        "goal_position_commands_sent": False,
        "torque_commands_sent": False,
        "eeprom_writes_sent": False,
    }
    write_json(audit_path, audit)

    if failures:
        print(f"\nRESULT: FAIL — controllare motori {failures}")
        print(f"Audit: {relative_or_absolute(audit_path)}")
        raise CalibrationError("Read-back EEPROM finale non superato")

    print("\nRESULT: PASS — tutti i 12 offset EEPROM sono confermati.")
    print(f"Audit: {relative_or_absolute(audit_path)}")
    print("Nessun comando motore o scrittura EEPROM inviati.")


async def async_main(args: argparse.Namespace) -> None:
    client: Any | None = None
    try:
        logger = logging.getLogger("matdog_digital_zero_calibration")
        client = await new_station_client(args.server, logger)
        if args.mode == "preflight":
            await run_preflight(args, client)
        elif args.mode == "execute":
            await run_execute(args, client)
        elif args.mode == "verify":
            await run_verify(args, client)
        else:
            raise CalibrationError(f"Modalità non supportata: {args.mode}")
    finally:
        await shutdown_client(client)


def main() -> int:
    args = parse_args()
    try:
        asyncio.run(async_main(args))
        return 0
    except KeyboardInterrupt:
        print("\nABORT operatore: nessun nuovo comando verrà inviato.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
