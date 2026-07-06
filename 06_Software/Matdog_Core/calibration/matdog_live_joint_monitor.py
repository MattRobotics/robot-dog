#!/usr/bin/env python3
"""
MATDOG — monitor live read-only di un joint calibrato.

Legge st3215/inference da Station e converte:
present_position -> q_urdf_rad.

NON:
- abilita torque;
- invia target;
- modifica YAML;
- apre direttamente la seriale.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import math
import sys
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
NORMACORE = Path.home() / "norma-core"
EXAMPLE_DIR = NORMACORE / "software/station/examples/st3215-remote-teleop-py"
DEFAULT_CONFIG = REPO_ROOT / "06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml"

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger("matdog_live_joint_monitor")

sys.path.insert(0, str(NORMACORE))
sys.path.insert(0, str(EXAMPLE_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215
from state import find_bus, parse_motor_state, resolve_bus_serial
from matdog_joint_math import encoder_to_joint_rad, signed_tick_delta


class BusReader:
    def __init__(self, client):
        self.latest = None
        self.frame_count = 0
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
            self.frame_count += 1


async def wait_first_frame(reader, timeout_s):
    deadline = time.monotonic() + timeout_s
    while reader.latest is None:
        if time.monotonic() > deadline:
            raise RuntimeError(f"nessun frame ST3215 entro {timeout_s:.1f}s")
        await asyncio.sleep(0.05)


async def wait_next_frame(reader, previous_count, timeout_s):
    deadline = time.monotonic() + timeout_s
    while reader.frame_count <= previous_count:
        if time.monotonic() > deadline:
            raise RuntimeError("timeout in attesa del frame ST3215 successivo")
        await asyncio.sleep(0.02)


def read_motor_state(inference_state, bus_serial, motor_id):
    bus = find_bus(inference_state, bus_serial)
    if bus is None:
        raise RuntimeError(f"bus {bus_serial!r} non trovato")

    for motor in bus.get_motors() or []:
        if motor.get_id() == motor_id:
            return parse_motor_state(motor)

    raise RuntimeError(f"motore M{motor_id} non trovato sul bus {bus_serial}")


async def main_async(args):
    config_path = Path(args.config).expanduser().resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    if config.get("robot", {}).get("calibration_status") != (
        "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"
    ):
        raise RuntimeError(
            "stato calibrazione inatteso: serve "
            "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"
        )

    joint = config.get("joints", {}).get(args.joint)
    if joint is None:
        available = ", ".join(config.get("joints", {}).keys())
        raise RuntimeError(f"joint sconosciuto: {args.joint}. Disponibili: {available}")

    servo_id = int(joint["servo_id"])
    direction = int(joint["direction"])
    zero_tick = joint["zero_encoder_visual"]

    if not isinstance(zero_tick, int):
        raise RuntimeError(f"{args.joint}: zero_encoder_visual non valido")

    client = await new_station_client(args.server, logger)
    reader = BusReader(client)
    reader_task = asyncio.create_task(reader.run())

    try:
        print("=== MATDOG LIVE JOINT MONITOR — READ ONLY ===")
        print("Nessun torque, target o comando motore verrà inviato.")
        print(f"Joint: {args.joint} | servo: M{servo_id} | direction: {direction:+d}")
        print(f"Zero visuale: {zero_tick} tick")
        print("")

        await wait_first_frame(reader, args.timeout)
        bus_serial = resolve_bus_serial(reader.latest, args.bus)
        print(f"Bus: {bus_serial}")
        print("Muovi manualmente il joint solo con torque disabilitato.")
        print("Premi Ctrl+C per terminare.\n")

        previous_count = reader.frame_count
        last_tick = None
        deadline = time.monotonic() + args.duration

        while time.monotonic() < deadline:
            await wait_next_frame(reader, previous_count, args.timeout)
            previous_count = reader.frame_count

            state = read_motor_state(reader.latest, bus_serial, servo_id)
            present_tick = int(state.present_position)

            if present_tick == last_tick:
                continue

            q_rad = encoder_to_joint_rad(present_tick, zero_tick, direction)
            delta_tick = signed_tick_delta(present_tick, zero_tick)

            print(
                f"M{servo_id:02d}  "
                f"encoder={present_tick:4d}  "
                f"delta={delta_tick:+5d} tick  "
                f"q={q_rad:+.6f} rad  "
                f"({math.degrees(q_rad):+.2f} deg)"
            )

            last_tick = present_tick

        print("\nTIMEOUT: monitor concluso senza inviare comandi.")

    finally:
        reader_task.cancel()
        try:
            await reader_task
        except asyncio.CancelledError:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Monitor read-only encoder -> radianti URDF per un joint MATDOG."
    )
    parser.add_argument("--joint", required=True)
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente.")
    except Exception as exc:
        print(f"\nERRORE: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
