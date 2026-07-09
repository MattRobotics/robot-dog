#!/usr/bin/env python3
"""
MATDOG C5 — supervised first stand executor.

Default mode is DRY-RUN.
No serial access.
Commands go only through NormaCore Station queue "commands".

SAFETY STATUS:
Physical execution is currently BLOCKED after the 2026-07-09 C5 RF failure.
RF_HIP/RF_LOWER must be mechanically realigned away from encoder wrap 0/4095
and revalidated before this executor can be re-enabled for real motion.

Execution phases:
1. read C4-C offline trajectory report;
2. convert q_rad targets to ST3215 encoder ticks using MATDOG calibration;
3. build bridge from current visual-zero vicinity to C4-C frame 0;
4. continue C4-C frame 0 -> frame 50;
5. only execute when --execute-confirm exact phrase is provided.

Operator safety:
- robot must be suspended/supported;
- physical servo power cut must be reachable;
- torque stays ON at end unless --torque-off-at-end is passed.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
MATDOG_CORE = REPO_ROOT / "06_Software" / "Matdog_Core"
CALIBRATION_DIR = MATDOG_CORE / "calibration"
KINEMATICS_DIR = MATDOG_CORE / "kinematics"

sys.path.insert(0, str(CALIBRATION_DIR))
sys.path.insert(0, str(KINEMATICS_DIR))

from matdog_joint_math import joint_rad_to_encoder, signed_tick_delta  # noqa: E402

# NormaCore paths
NORMA_ROOT = Path.home() / "norma-core"
sys.path.insert(0, str(NORMA_ROOT))
sys.path.insert(0, str(NORMA_ROOT / "target" / "gen_python"))

from software.station.shared.station_py import new_station_client, send_commands  # noqa: E402
from target.gen_python.protobuf.station import commands as station_commands, drivers  # noqa: E402
from target.gen_python.protobuf.drivers.st3215 import st3215  # noqa: E402


RAM_TORQUE_ENABLE = 0x28
RAM_ACC = 0x29
RAM_GOAL_POSITION = 0x2A
RAM_GOAL_SPEED = 0x2E

EXECUTE_CONFIRM = "APPROVO C5 PRIMO STAND FISICO SUPERVISIONATO"

DEFAULT_C4C_REPORT = (
    REPO_ROOT
    / "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory"
    / "2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json"
)

DEFAULT_CALIBRATION = (
    REPO_ROOT
    / "06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml"
)

JOINT_ORDER = [
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
]


def load_joint_calibration(path: Path) -> dict[str, dict[str, Any]]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))

    if isinstance(raw, dict) and "joints" in raw:
        joints = raw["joints"]
    elif isinstance(raw, dict):
        joints = raw
    else:
        raise RuntimeError(f"Formato calibrazione non valido: {path}")

    result: dict[str, dict[str, Any]] = {}

    for joint_name, item in joints.items():
        if not isinstance(item, dict):
            continue

        servo_id = item.get("servo_id")
        zero = item.get("zero_encoder_visual")
        direction = item.get("direction")

        if servo_id is None or zero is None or direction is None:
            continue

        result[joint_name] = {
            "servo_id": int(servo_id),
            "zero_encoder_visual": int(zero),
            "direction": int(direction),
        }

    missing = [name for name in JOINT_ORDER if name not in result]
    if missing:
        raise RuntimeError(f"Joint mancanti nella calibrazione: {missing}")

    return result


def _sync_write(bus_serial: str, address: int, motors: list[tuple[int, bytes]]):
    if not motors:
        return None

    cmd = st3215.Command(
        target_bus_serial=bus_serial,
        sync_write=st3215.ST3215SyncWriteCommand(
            address=address,
            motors=[
                st3215.ST3215SyncWriteCommand_MotorWrite(
                    motor_id=mid,
                    value=val,
                )
                for mid, val in motors
            ],
        ),
    )

    return station_commands.DriverCommand(
        type=drivers.StationCommandType.STC_ST3215_COMMAND,
        body=cmd.encode(),
    )


async def send_sync_write(client, bus_serial: str, address: int, motors: list[tuple[int, bytes]]):
    cmd = _sync_write(bus_serial, address, motors)
    if cmd is not None:
        await send_commands(client, [cmd])


async def set_torque(client, bus_serial: str, motor_ids: list[int], enable: bool):
    value = b"\x01" if enable else b"\x00"
    await send_sync_write(
        client,
        bus_serial,
        RAM_TORQUE_ENABLE,
        [(mid, value) for mid in motor_ids],
    )


async def send_pose(
    client,
    bus_serial: str,
    pose_ticks: dict[str, int],
    calibration: dict[str, dict[str, Any]],
    speed: int,
    accel: int,
):
    speed_writes = []
    accel_writes = []
    goal_writes = []

    for joint_name in JOINT_ORDER:
        c = calibration[joint_name]
        mid = int(c["servo_id"])
        tick = int(pose_ticks[joint_name]) % 4096

        speed_writes.append((mid, int(speed).to_bytes(2, "little")))
        accel_writes.append((mid, bytes([int(accel)])))
        goal_writes.append((mid, tick.to_bytes(2, "little")))

    pack = []
    for cmd in (
        _sync_write(bus_serial, RAM_GOAL_SPEED, speed_writes),
        _sync_write(bus_serial, RAM_ACC, accel_writes),
        _sync_write(bus_serial, RAM_GOAL_POSITION, goal_writes),
    ):
        if cmd is not None:
            pack.append(cmd)

    if pack:
        await send_commands(client, pack)


def frame_to_pose_ticks(
    frame: dict[str, Any],
    calibration: dict[str, dict[str, Any]],
) -> dict[str, int]:
    pose: dict[str, int] = {}

    for leg_id in ["lf", "rf", "rh", "lh"]:
        joint_rad = frame["legs"][leg_id]["joint_positions_rad"]
        for joint_name, q_rad in joint_rad.items():
            c = calibration[joint_name]
            pose[joint_name] = joint_rad_to_encoder(
                float(q_rad),
                int(c["zero_encoder_visual"]),
                int(c["direction"]),
            )

    missing = [name for name in JOINT_ORDER if name not in pose]
    if missing:
        raise RuntimeError(f"Target mancanti nel frame: {missing}")

    return pose


def visual_zero_pose_ticks(calibration: dict[str, dict[str, Any]]) -> dict[str, int]:
    return {
        joint_name: int(calibration[joint_name]["zero_encoder_visual"])
        for joint_name in JOINT_ORDER
    }


def interpolate_circular_ticks(
    start: dict[str, int],
    end: dict[str, int],
    steps: int,
) -> list[dict[str, int]]:
    if steps < 2:
        raise RuntimeError("steps deve essere >= 2")

    out = []

    for i in range(steps):
        alpha = i / (steps - 1)
        pose = {}

        for joint_name in JOINT_ORDER:
            s = int(start[joint_name])
            e = int(end[joint_name])
            delta = signed_tick_delta(e, s)
            pose[joint_name] = (s + round(delta * alpha)) % 4096

        out.append(pose)

    return out


def max_step_ticks(frames: list[dict[str, int]]) -> int:
    worst = 0
    for a, b in zip(frames, frames[1:]):
        for joint_name in JOINT_ORDER:
            worst = max(worst, abs(signed_tick_delta(b[joint_name], a[joint_name])))
    return worst


def max_range_ticks(start: dict[str, int], end: dict[str, int]) -> dict[str, int]:
    return {
        joint_name: abs(signed_tick_delta(end[joint_name], start[joint_name]))
        for joint_name in JOINT_ORDER
    }


def print_pose(label: str, pose: dict[str, int], calibration: dict[str, dict[str, Any]]):
    print(label)
    for joint_name in JOINT_ORDER:
        mid = calibration[joint_name]["servo_id"]
        print(f"  {joint_name:24s} M{mid:02d} tick={pose[joint_name]:4d}")


def build_sequence(
    c4c_report: Path,
    calibration: dict[str, dict[str, Any]],
    bridge_seconds: float,
    c4c_stride: int,
    dt_s: float,
) -> tuple[list[dict[str, int]], dict[str, Any]]:
    data = json.loads(c4c_report.read_text(encoding="utf-8"))
    frames = data["frames"]

    c4c_poses = [
        frame_to_pose_ticks(frame, calibration)
        for index, frame in enumerate(frames)
        if index % c4c_stride == 0 or index == len(frames) - 1
    ]

    start_pose = visual_zero_pose_ticks(calibration)
    frame0_pose = c4c_poses[0]

    bridge_steps = max(2, int(round(bridge_seconds / dt_s)) + 1)
    bridge = interpolate_circular_ticks(start_pose, frame0_pose, bridge_steps)

    # Avoid duplicate frame0.
    sequence = bridge + c4c_poses[1:]

    summary = {
        "c4c_frames_original": len(frames),
        "c4c_frames_used": len(c4c_poses),
        "bridge_steps": len(bridge),
        "sequence_steps": len(sequence),
        "dt_s": dt_s,
        "duration_estimated_s": dt_s * max(0, len(sequence) - 1),
        "bridge_max_total_range_ticks": max_range_ticks(start_pose, frame0_pose),
        "sequence_max_step_ticks": max_step_ticks(sequence),
    }

    return sequence, summary


async def main_async(args):
    calibration = load_joint_calibration(args.config)
    sequence, summary = build_sequence(
        c4c_report=args.c4c_report,
        calibration=calibration,
        bridge_seconds=args.bridge_seconds,
        c4c_stride=args.c4c_stride,
        dt_s=args.dt,
    )

    motor_ids = [calibration[name]["servo_id"] for name in JOINT_ORDER]

    print("=== MATDOG C5 SUPERVISED FIRST STAND EXECUTOR ===")
    print(f"mode: {'EXECUTE' if args.execute_confirm == EXECUTE_CONFIRM else 'DRY_RUN'}")
    print(f"server: {args.server}")
    print(f"bus: {args.bus}")
    print(f"config: {args.config}")
    print(f"c4c_report: {args.c4c_report}")
    print(f"speed_raw: {args.speed}")
    print(f"accel_raw: {args.accel}")
    print(f"dt_s: {args.dt}")
    print(f"bridge_seconds: {args.bridge_seconds}")
    print(f"c4c_stride: {args.c4c_stride}")
    print("")
    print("SUMMARY:")
    for k, v in summary.items():
        print(f"  {k}: {v}")

    print("")
    print_pose("START ASSUMED = visual-zero ticks", sequence[0], calibration)
    print("")
    print_pose("FRAME0 TARGET = C4-C frame 0", sequence[summary["bridge_steps"] - 1], calibration)
    print("")
    print_pose("FINAL TARGET = C4-C final frame", sequence[-1], calibration)

    if summary["sequence_max_step_ticks"] > args.max_step_ticks:
        raise RuntimeError(
            f"ABORT: max step {summary['sequence_max_step_ticks']} tick "
            f"> limit {args.max_step_ticks}"
        )

    print("")
    print("SAFETY GATES:")
    print("  serial_direct_access: false")
    print("  station_command_queue_only: true")
    print("  command_source: STC_ST3215_COMMAND sync_write")
    print("  torque_default: off until execute")
    print("  execute_phrase_required:", EXECUTE_CONFIRM)

    if args.execute_confirm != EXECUTE_CONFIRM:
        print("")
        print("DRY_RUN_ONLY: nessun comando Station inviato.")
        print("Per eseguire serve --execute-confirm con frase esatta.")
        return

    raise RuntimeError(
        "EXECUTION BLOCKED: C5 first-stand physical execution is disabled "
        "after RF_HIP/RF_LOWER wrap/collision failure. Mechanically realign "
        "RF_HIP and RF_LOWER away from encoder wrap 0/4095, then revalidate "
        "read-only zero, FK, dry-run and hardware gates before re-enabling."
    )

    if not args.operator_abort_confirmed:
        raise RuntimeError("ABORT: manca --operator-abort-confirmed")

    logger = logging.getLogger("matdog_c5_first_stand")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")

    client = await new_station_client(args.server, logger)
    bus_serial = args.bus

    if bus_serial == "auto":
        # In C5 user already verified one bus via read-only tools.
        # This executor keeps the concrete bus serial explicit after that gate.
        bus_serial = args.resolved_bus_serial

    if not bus_serial:
        raise RuntimeError("ABORT: bus serial non risolto")

    print("")
    print("EXECUTE: invio priming goal visual-zero, torque ON, traiettoria lenta.")
    print("ABORT PRIMARIO: tagliare fisicamente alimentazione servo.")

    # Prime goal registers to visual-zero before enabling torque.
    await send_pose(
        client,
        bus_serial,
        sequence[0],
        calibration,
        speed=args.speed,
        accel=args.accel,
    )
    await asyncio.sleep(0.25)

    await set_torque(client, bus_serial, motor_ids, True)
    await asyncio.sleep(0.50)

    t0 = time.monotonic()
    for index, pose in enumerate(sequence):
        await send_pose(
            client,
            bus_serial,
            pose,
            calibration,
            speed=args.speed,
            accel=args.accel,
        )
        print(f"sent step {index+1}/{len(sequence)}  t={time.monotonic()-t0:.2f}s")
        await asyncio.sleep(args.dt)

    print("HOLD: traiettoria completata, torque resta ON.")

    if args.torque_off_at_end:
        print("torque_off_at_end: disabilito torque.")
        await set_torque(client, bus_serial, motor_ids, False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--resolved-bus-serial", default="5B14114953")
    parser.add_argument("--config", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--c4c-report", type=Path, default=DEFAULT_C4C_REPORT)
    parser.add_argument("--dt", type=float, default=0.20)
    parser.add_argument("--bridge-seconds", type=float, default=14.0)
    parser.add_argument("--c4c-stride", type=int, default=1)
    parser.add_argument("--speed", type=int, default=60)
    parser.add_argument("--accel", type=int, default=4)
    parser.add_argument("--max-step-ticks", type=int, default=24)
    parser.add_argument("--execute-confirm", default="")
    parser.add_argument("--operator-abort-confirmed", action="store_true")
    parser.add_argument("--torque-off-at-end", action="store_true")
    args = parser.parse_args()

    if args.dt <= 0:
        raise RuntimeError("--dt deve essere > 0")
    if args.bridge_seconds <= 0:
        raise RuntimeError("--bridge-seconds deve essere > 0")
    if args.c4c_stride <= 0:
        raise RuntimeError("--c4c-stride deve essere > 0")
    if not args.config.exists():
        raise RuntimeError(f"Config non trovato: {args.config}")
    if not args.c4c_report.exists():
        raise RuntimeError(f"Report C4-C non trovato: {args.c4c_report}")

    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
