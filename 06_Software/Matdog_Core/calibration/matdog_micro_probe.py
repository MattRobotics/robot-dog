#!/usr/bin/env python3
"""
MATDOG — micro-probe sicuro per testare il verso di un singolo ST3215.

Regole:
- Station resta l'unico proprietario della seriale.
- Un solo servo alla volta.
- Goal iniziale primed alla posizione presente PRIMA del torque on.
- Movimento relativo piccolo, ritorno garantito, torque off in cleanup.
- Dry run di default: per muovere davvero richiede --execute e --confirm.
"""

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
logger = logging.getLogger("matdog_micro_probe")

MAX_STEPS = 4096
VALID_MATDOG_IDS = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43}


class BusReader:
    def __init__(self, client, label: str):
        self.client = client
        self.label = label
        self.latest = None
        self._last_entry_id = b""
        self._queue = asyncio.Queue()
        self._error_queue = client.follow("st3215/inference", self._queue)

    async def run(self):
        while True:
            if not self._error_queue.empty():
                err = self._error_queue.get_nowait()
                raise RuntimeError(f"[{self.label}] inference stream error: {err}")

            entry = await self._queue.get()
            if entry is None:
                raise RuntimeError(f"[{self.label}] inference stream closed")

            entry_id = bytes(entry.ID.ID)
            if entry_id == self._last_entry_id:
                continue

            self._last_entry_id = entry_id
            self.latest = st3215.InferenceStateReader(entry.Data)


async def wait_for_first_frame(reader: BusReader, timeout_s: float = 10.0):
    deadline = time.monotonic() + timeout_s
    while reader.latest is None:
        if time.monotonic() > deadline:
            raise RuntimeError(
                f"[{reader.label}] nessun frame ST3215 entro {timeout_s:.1f}s"
            )
        await asyncio.sleep(0.05)


def find_motor_state(inference_state, bus_serial: str, motor_id: int):
    bus = find_bus(inference_state, bus_serial)
    if bus is None:
        raise RuntimeError(f"Bus '{bus_serial}' non trovato nello stato Station")

    for motor in bus.get_motors() or []:
        if motor.get_id() == motor_id:
            return parse_motor_state(motor)

    raise RuntimeError(f"Motore {motor_id} non trovato sul bus '{bus_serial}'")


def circular_error(actual: int, target: int) -> int:
    return abs(((actual - target + MAX_STEPS // 2) % MAX_STEPS) - MAX_STEPS // 2)


async def wait_for_position(
    reader: BusReader,
    bus_serial: str,
    motor_id: int,
    target: int,
    tolerance: int,
    timeout_s: float,
    phase: str,
):
    deadline = time.monotonic() + timeout_s
    last_state = None

    while time.monotonic() < deadline:
        if reader.latest is not None:
            last_state = find_motor_state(reader.latest, bus_serial, motor_id)
            err = circular_error(last_state.present_position, target)
            if err <= tolerance:
                logger.info(
                    "%s raggiunto: present=%s target=%s err=%s tick",
                    phase,
                    last_state.present_position,
                    target,
                    err,
                )
                return True, last_state

        await asyncio.sleep(0.05)

    if last_state is None:
        raise RuntimeError(f"{phase}: nessuna telemetria disponibile")

    logger.warning(
        "%s timeout: present=%s target=%s err=%s tick",
        phase,
        last_state.present_position,
        target,
        circular_error(last_state.present_position, target),
    )
    return False, last_state


async def send_goal(client, bus_serial, motor_id, goal, speed, accel):
    await send_motor_commands(
        client,
        bus_serial,
        [
            MotorCommand(
                motor_id=motor_id,
                speed=speed,
                accel=accel,
                goal=goal,
            )
        ],
    )


async def main_async(args):
    if args.motor_id not in VALID_MATDOG_IDS:
        raise RuntimeError(
            f"ID {args.motor_id} non è nella mappa MATDOG: "
            f"{sorted(VALID_MATDOG_IDS)}"
        )

    if not -30 <= args.delta <= 30 or args.delta == 0:
        raise RuntimeError("--delta deve essere diverso da 0 e compreso fra -30 e +30")

    if abs(args.delta) <= args.tolerance:
        raise RuntimeError(
            "delta insufficiente: |--delta| deve essere maggiore di "
            "--tolerance, altrimenti il target può risultare raggiunto "
            "senza movimento reale."
        )

    client = None
    reader_task = None
    reader = None
    bus_serial = None
    start_pos = None
    torque_enabled = False

    try:
        client = await new_station_client(args.server, logger)
        reader = BusReader(client, label=f"station@{args.server}")
        reader_task = asyncio.create_task(reader.run())

        logger.info("Aspetto il primo frame ST3215...")
        await wait_for_first_frame(reader)

        bus_serial = resolve_bus_serial(reader.latest, args.bus)
        state = find_motor_state(reader.latest, bus_serial, args.motor_id)
        start_pos = state.present_position
        target_pos = start_pos + args.delta

        if not 0 <= target_pos < MAX_STEPS:
            raise RuntimeError(
                f"Target {target_pos} fuori dall'intervallo 0..4095. "
                "Per questo primo tool non usiamo wrap-around."
            )

        logger.info("=== MATDOG MICRO-PROBE ===")
        logger.info("Bus: %s", bus_serial)
        logger.info("Servo: M%s", args.motor_id)
        logger.info("Present iniziale: %s", start_pos)
        logger.info("Target micro-probe: %s", target_pos)
        logger.info("Delta: %+d tick", args.delta)
        logger.info("Speed=%s accel=%s hold=%.2fs", args.speed, args.accel, args.hold)

        if not args.execute:
            logger.info("DRY RUN: nessun torque e nessun comando di movimento inviato.")
            logger.info(
                "Per eseguire davvero: --execute --confirm MOVE_M%s",
                args.motor_id,
            )
            return

        expected_confirmation = f"MOVE_M{args.motor_id}"
        if args.confirm != expected_confirmation:
            raise RuntimeError(
                f"Conferma errata. Usa esattamente: --confirm {expected_confirmation}"
            )

        logger.info("Priming: imposto goal uguale alla posizione attuale, torque ancora OFF.")
        await send_goal(
            client, bus_serial, args.motor_id, start_pos, args.speed, args.accel
        )
        await asyncio.sleep(0.15)

        logger.info("Abilito torque SOLO su M%s", args.motor_id)
        await set_torque(client, bus_serial, [args.motor_id], enable=True)
        torque_enabled = True
        await asyncio.sleep(0.25)

        await wait_for_position(
            reader,
            bus_serial,
            args.motor_id,
            start_pos,
            args.tolerance,
            args.timeout,
            "HOLD iniziale",
        )

        logger.info("Micro-movimento: %s -> %s", start_pos, target_pos)
        await send_goal(
            client, bus_serial, args.motor_id, target_pos, args.speed, args.accel
        )

        await asyncio.sleep(args.hold)

        await wait_for_position(
            reader,
            bus_serial,
            args.motor_id,
            target_pos,
            args.tolerance,
            args.timeout,
            "TARGET micro-movimento",
        )

        logger.info("Ritorno automatico: %s -> %s", target_pos, start_pos)
        await send_goal(
            client, bus_serial, args.motor_id, start_pos, args.speed, args.accel
        )

        await asyncio.sleep(args.hold)

        returned, final_state = await wait_for_position(
            reader,
            bus_serial,
            args.motor_id,
            start_pos,
            args.tolerance,
            args.timeout,
            "RITORNO iniziale",
        )

        if not returned:
            logger.warning(
                "Ritorno non entro tolleranza: M%s present=%s expected=%s. "
                "Il risultato del test verso resta valido; posizione raw registrata.",
                args.motor_id,
                final_state.present_position,
                start_pos,
            )
        else:
            logger.info("Ritorno iniziale verificato.")

        logger.info("Micro-probe completato.")

    finally:
        if (
            torque_enabled
            and client is not None
            and bus_serial is not None
            and start_pos is not None
        ):
            try:
                logger.info("Safety cleanup: ritorno a %s", start_pos)
                await asyncio.shield(
                    send_goal(
                        client,
                        bus_serial,
                        args.motor_id,
                        start_pos,
                        args.speed,
                        args.accel,
                    )
                )
                await asyncio.sleep(0.25)
            except Exception:
                logger.exception("Cleanup: ritorno alla posizione iniziale fallito")

        if client is not None and bus_serial is not None and torque_enabled:
            try:
                logger.info("Safety cleanup: torque OFF solo su M%s", args.motor_id)
                await asyncio.shield(
                    set_torque(client, bus_serial, [args.motor_id], enable=False)
                )
            except Exception:
                logger.exception("Cleanup: torque off fallito")

        if reader_task is not None:
            reader_task.cancel()


def main():
    parser = argparse.ArgumentParser(
        description="MATDOG — micro-probe reversibile di un solo ST3215"
    )
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--delta", type=int, default=20)
    parser.add_argument("--speed", type=int, default=40)
    parser.add_argument("--accel", type=int, default=5)
    parser.add_argument("--hold", type=float, default=0.8)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--tolerance", type=int, default=8)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")

    args = parser.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
