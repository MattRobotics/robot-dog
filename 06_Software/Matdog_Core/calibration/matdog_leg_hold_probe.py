#!/usr/bin/env python3
"""
MATDOG — prova coordinata di HOLD su una zampa.

Legge le posizioni presenti dei tre servo, imposta i goal uguali alle
posizioni attuali, abilita torque solo su quei tre servo, mantiene la posa
per pochi secondi e poi disabilita torque.

Non genera movimenti intenzionali.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path

NORMACORE = Path.home() / "norma-core"
EXAMPLE_DIR = NORMACORE / "software/station/examples/st3215-remote-teleop-py"

sys.path.insert(0, str(NORMACORE))
sys.path.insert(0, str(EXAMPLE_DIR))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215
from commands import send_motor_commands, set_torque
from mirror import MotorCommand
from state import find_bus, parse_motor_state, resolve_bus_serial

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger("matdog_leg_hold_probe")

LEGS = {
    "LF": [13, 12, 11],
    "RF": [23, 22, 21],
    "RH": [33, 32, 31],
    "LH": [43, 42, 41],
    "ALL": [13, 12, 11, 23, 22, 21, 33, 32, 31, 43, 42, 41],
}


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


async def wait_for_frame(reader, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while reader.latest is None:
        if time.monotonic() > deadline:
            raise RuntimeError("nessun frame ST3215 ricevuto")
        await asyncio.sleep(0.05)


def motor_position(inference, bus_serial, motor_id):
    bus = find_bus(inference, bus_serial)
    if bus is None:
        raise RuntimeError(f"bus {bus_serial!r} non trovato")

    for motor in bus.get_motors() or []:
        if motor.get_id() == motor_id:
            return int(parse_motor_state(motor).present_position)

    raise RuntimeError(f"M{motor_id} non trovato sul bus {bus_serial}")


async def main_async(args):
    servo_ids = LEGS[args.leg]
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

        start = {
            motor_id: motor_position(reader.latest, bus_serial, motor_id)
            for motor_id in servo_ids
        }

        logger.info("=== MATDOG LEG HOLD PROBE ===")
        logger.info("Leg: %s | bus: %s", args.leg, bus_serial)
        logger.info("Goal priming, nessun movimento previsto:")
        for motor_id in servo_ids:
            logger.info("M%s: %s", motor_id, start[motor_id])

        if not args.execute:
            logger.info("DRY RUN: nessun torque e nessun goal inviato.")
            logger.info(
                "Per eseguire: --execute --confirm HOLD_%s",
                args.leg,
            )
            return

        expected = f"HOLD_{args.leg}"
        if args.confirm != expected:
            raise RuntimeError(f"conferma errata: usa --confirm {expected}")

        commands = [
            MotorCommand(
                motor_id=motor_id,
                speed=args.speed,
                accel=args.accel,
                goal=start[motor_id],
            )
            for motor_id in servo_ids
        ]

        logger.info("Priming goal uguali alle posizioni attuali, torque OFF.")
        await send_motor_commands(client, bus_serial, commands)
        await asyncio.sleep(0.25)

        logger.info("Torque ON solo su %s", servo_ids)
        await set_torque(client, bus_serial, servo_ids, enable=True)
        torque_enabled = True

        logger.info("HOLD per %.1f s.", args.hold)
        await asyncio.sleep(args.hold)

        logger.info("HOLD completato senza movimento intenzionale.")

    finally:
        if client is not None and bus_serial is not None and torque_enabled:
            try:
                logger.info("Safety cleanup: torque OFF su %s", servo_ids)
                await asyncio.shield(
                    set_torque(client, bus_serial, servo_ids, enable=False)
                )
            except Exception:
                logger.exception("Cleanup torque OFF fallito")

        if reader_task is not None:
            reader_task.cancel()


def main():
    parser = argparse.ArgumentParser(
        description="MATDOG — hold coordinato di una sola zampa"
    )
    parser.add_argument("--leg", choices=LEGS, required=True)
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--hold", type=float, default=1.5)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--accel", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
