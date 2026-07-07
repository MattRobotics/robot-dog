#!/usr/bin/env python3
"""
MATDOG — primo comando coordinato verso la posa zero visuale.

Regole:
- Station resta proprietaria unica della seriale;
- tutti i goal sono letti da MATDOG_JOINT_CALIBRATION.yaml;
- preflight obbligatorio: ogni servo deve essere entro max_delta dal suo zero;
- priming con goal uguale alla posizione corrente prima del torque;
- torque OFF automatico in cleanup;
- dry-run di default.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
NORMACORE = Path.home() / "norma-core"
EXAMPLE_DIR = NORMACORE / "software/station/examples/st3215-remote-teleop-py"
CALDIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = CALDIR / "MATDOG_JOINT_CALIBRATION.yaml"

sys.path.insert(0, str(NORMACORE))
sys.path.insert(0, str(EXAMPLE_DIR))
sys.path.insert(0, str(CALDIR))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215
from commands import send_motor_commands, set_torque
from mirror import MotorCommand
from state import find_bus, parse_motor_state, resolve_bus_serial
from matdog_joint_math import signed_tick_delta

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger("matdog_visual_zero_pose_probe")

SERVO_ORDER = [13, 12, 11, 23, 22, 21, 33, 32, 31, 43, 42, 41]
EXPECTED_STATUS = "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"


class BusReader:
    def __init__(self, client):
        self.latest = None
        self._last_entry_id = b""
        self._queue = asyncio.Queue()
        self._error_queue = client.follow("st3215/inference", self._queue)

    async def run(self):
        while True:
            if not self._error_queue.empty():
                raise RuntimeError(
                    f"inference stream error: {self._error_queue.get_nowait()}"
                )

            entry = await self._queue.get()
            if entry is None:
                raise RuntimeError("inference stream chiuso")

            entry_id = bytes(entry.ID.ID)
            if entry_id == self._last_entry_id:
                continue

            self._last_entry_id = entry_id
            self.latest = st3215.InferenceStateReader(entry.Data)


async def wait_for_frame(reader, timeout_s):
    deadline = time.monotonic() + timeout_s

    while reader.latest is None:
        if time.monotonic() > deadline:
            raise RuntimeError(f"nessun frame ST3215 entro {timeout_s:.1f}s")
        await asyncio.sleep(0.05)


def position_for_motor(inference, bus_serial, motor_id):
    bus = find_bus(inference, bus_serial)

    if bus is None:
        raise RuntimeError(f"bus {bus_serial!r} non trovato")

    for motor in bus.get_motors() or []:
        if motor.get_id() == motor_id:
            return int(parse_motor_state(motor).present_position)

    raise RuntimeError(f"M{motor_id} non trovato sul bus {bus_serial}")


def circular_error(actual, target):
    return abs(signed_tick_delta(actual, target))


async def wait_targets(reader, bus_serial, targets, tolerance, timeout_s):
    deadline = time.monotonic() + timeout_s
    last_positions = {}

    while time.monotonic() < deadline:
        if reader.latest is not None:
            last_positions = {
                motor_id: position_for_motor(reader.latest, bus_serial, motor_id)
                for motor_id in targets
            }

            errors = {
                motor_id: circular_error(last_positions[motor_id], targets[motor_id])
                for motor_id in targets
            }

            if max(errors.values()) <= tolerance:
                return True, last_positions, errors

        await asyncio.sleep(0.05)

    errors = {
        motor_id: circular_error(last_positions[motor_id], targets[motor_id])
        for motor_id in last_positions
    }
    return False, last_positions, errors


def load_targets(config_path):
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    status = data.get("robot", {}).get("calibration_status")
    if status != EXPECTED_STATUS:
        raise RuntimeError(
            f"calibration_status inatteso: {status!r}; atteso {EXPECTED_STATUS!r}"
        )

    targets = {}

    for joint_name, joint in data.get("joints", {}).items():
        servo_id = joint.get("servo_id")
        zero = joint.get("zero_encoder_visual")

        if servo_id not in SERVO_ORDER:
            raise RuntimeError(f"{joint_name}: servo_id non previsto: {servo_id!r}")

        if not isinstance(zero, int) or not 0 <= zero <= 4095:
            raise RuntimeError(
                f"{joint_name}: zero_encoder_visual invalido: {zero!r}"
            )

        targets[servo_id] = zero

    if set(targets) != set(SERVO_ORDER):
        raise RuntimeError(
            f"mappa servo incompleta: trovati {sorted(targets)}, "
            f"attesi {SERVO_ORDER}"
        )

    return targets


async def main_async(args):
    config_path = Path(args.config).expanduser().resolve()
    targets = load_targets(config_path)

    client = None
    reader_task = None
    bus_serial = None
    torque_enabled = False

    try:
        client = await new_station_client(args.server, logger)
        reader = BusReader(client)
        reader_task = asyncio.create_task(reader.run())

        await wait_for_frame(reader, args.timeout)
        bus_serial = resolve_bus_serial(reader.latest, args.bus)

        current = {
            motor_id: position_for_motor(reader.latest, bus_serial, motor_id)
            for motor_id in SERVO_ORDER
        }

        deltas = {
            motor_id: signed_tick_delta(targets[motor_id], current[motor_id])
            for motor_id in SERVO_ORDER
        }

        logger.info("=== MATDOG VISUAL ZERO POSE PROBE ===")
        logger.info("Bus: %s", bus_serial)
        logger.info("Config: %s", config_path)
        logger.info(
            "Vincolo preflight: |target - current| <= %s tick",
            args.max_delta,
        )
        logger.info("")
        logger.info("servo  current  zero_target  delta_to_target")

        for motor_id in SERVO_ORDER:
            logger.info(
                "M%02d    %4d     %4d        %+5d",
                motor_id,
                current[motor_id],
                targets[motor_id],
                deltas[motor_id],
            )

        exceeded = {
            motor_id: delta
            for motor_id, delta in deltas.items()
            if abs(delta) > args.max_delta
        }

        if exceeded:
            raise RuntimeError(
                "preflight bloccato: servo troppo lontani dallo zero: "
                f"{exceeded}"
            )

        if not args.execute:
            logger.info("")
            logger.info("PREFLIGHT PASS: nessun torque e nessun goal inviato.")
            logger.info(
                "Per eseguire: --execute --confirm ZERO_VISUAL_ALL"
            )
            return

        if args.confirm != "ZERO_VISUAL_ALL":
            raise RuntimeError(
                "conferma errata. Usa esattamente: "
                "--confirm ZERO_VISUAL_ALL"
            )

        prime_commands = [
            MotorCommand(
                motor_id=motor_id,
                speed=args.speed,
                accel=args.accel,
                goal=current[motor_id],
            )
            for motor_id in SERVO_ORDER
        ]

        target_commands = [
            MotorCommand(
                motor_id=motor_id,
                speed=args.speed,
                accel=args.accel,
                goal=targets[motor_id],
            )
            for motor_id in SERVO_ORDER
        ]

        logger.info("Priming: goal uguali alle posizioni attuali, torque OFF.")
        await send_motor_commands(client, bus_serial, prime_commands)
        await asyncio.sleep(0.25)

        logger.info("Torque ON su tutti i 12 servo.")
        await set_torque(client, bus_serial, SERVO_ORDER, enable=True)
        torque_enabled = True
        await asyncio.sleep(0.30)

        logger.info("Comando coordinato verso lo zero visuale.")
        await send_motor_commands(client, bus_serial, target_commands)

        reached, final_positions, errors = await wait_targets(
            reader,
            bus_serial,
            targets,
            args.tolerance,
            args.timeout,
        )

        logger.info("")
        logger.info("servo  final  target  error")

        for motor_id in SERVO_ORDER:
            logger.info(
                "M%02d    %4d   %4d    %3d",
                motor_id,
                final_positions[motor_id],
                targets[motor_id],
                errors[motor_id],
            )

        if not reached:
            raise RuntimeError(
                f"target zero visuale non raggiunto entro {args.timeout:.1f}s; "
                f"max errore={max(errors.values())} tick"
            )

        logger.info(
            "ZERO VISUAL POSE PASS: tutti i 12 servo entro %s tick.",
            args.tolerance,
        )

    finally:
        if client is not None and bus_serial is not None and torque_enabled:
            try:
                logger.info("Safety cleanup: torque OFF su tutti i 12 servo.")
                await asyncio.shield(
                    set_torque(client, bus_serial, SERVO_ORDER, enable=False)
                )
            except Exception:
                logger.exception("Cleanup torque OFF fallito")

        if reader_task is not None:
            reader_task.cancel()


def main():
    parser = argparse.ArgumentParser(
        description="MATDOG — posa zero visuale coordinata, limitata e reversibile"
    )
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--speed", type=int, default=15)
    parser.add_argument("--accel", type=int, default=2)
    parser.add_argument("--max-delta", type=int, default=30)
    parser.add_argument("--tolerance", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
