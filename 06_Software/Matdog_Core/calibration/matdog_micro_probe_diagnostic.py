#!/usr/bin/env python3
"""
MATDOG — micro-probe diagnostico per un singolo ST3215.

Deriva dalla sequenza sicura di matdog_micro_probe.py, ma resta separato
per non modificare lo strumento già validato.

Registra per ogni campione:
- tempo relativo;
- posizione presente;
- target attivo;
- errore circolare;
- current raw;
- goal speed;
- goal accel;
- error status.

Station resta l'unico proprietario della seriale.
"""

import argparse
import asyncio
import json
import logging
import time
from datetime import datetime
from pathlib import Path

from matdog_micro_probe import (
    MAX_STEPS,
    VALID_MATDOG_IDS,
    BusReader,
    circular_error,
    find_motor_state,
    send_goal,
    wait_for_first_frame,
)
from commands import set_torque
from software.station.shared.station_py import new_station_client
from state import resolve_bus_serial

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger("matdog_micro_probe_diagnostic")

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_DIR = REPO_ROOT / "09_Logs" / "Calibration_Sessions"


class DiagnosticRecorder:
    def __init__(
        self,
        reader,
        bus_serial: str,
        motor_id: int,
        console_interval_s: float,
    ):
        self.reader = reader
        self.bus_serial = bus_serial
        self.motor_id = motor_id
        self.console_interval_s = console_interval_s
        self.started_at = time.monotonic()
        self.samples: list[dict] = []
        self._last_console_time = float("-inf")

    def capture(self, phase: str, expected_target: int, force_console: bool = False):
        if self.reader.latest is None:
            return None

        state = find_motor_state(
            self.reader.latest,
            self.bus_serial,
            self.motor_id,
        )
        now = time.monotonic()
        err = circular_error(state.present_position, expected_target)

        sample = {
            "t_s": round(now - self.started_at, 4),
            "phase": phase,
            "present_position": int(state.present_position),
            "expected_target": int(expected_target),
            "error_tick": int(err),
            "current_raw": int(state.current),
            "goal_speed": int(state.goal_speed),
            "goal_accel": int(state.goal_accel),
            "error_status": int(state.error_status),
        }
        self.samples.append(sample)

        if force_console or (now - self._last_console_time) >= self.console_interval_s:
            logger.info(
                "TEL phase=%s t=%.2fs present=%s target=%s err=%s "
                "current_raw=%s goal_speed=%s goal_accel=%s status=0x%02X",
                phase,
                sample["t_s"],
                sample["present_position"],
                sample["expected_target"],
                sample["error_tick"],
                sample["current_raw"],
                sample["goal_speed"],
                sample["goal_accel"],
                sample["error_status"],
            )
            self._last_console_time = now

        return state


async def sample_window(
    recorder: DiagnosticRecorder,
    phase: str,
    expected_target: int,
    duration_s: float,
    sample_interval_s: float,
):
    deadline = time.monotonic() + duration_s
    last_state = None

    while time.monotonic() < deadline:
        state = recorder.capture(phase, expected_target)
        if state is not None:
            last_state = state
        await asyncio.sleep(sample_interval_s)

    if last_state is not None:
        recorder.capture(phase, expected_target, force_console=True)

    return last_state


async def wait_for_position_diagnostic(
    recorder: DiagnosticRecorder,
    phase: str,
    expected_target: int,
    tolerance: int,
    timeout_s: float,
    sample_interval_s: float,
):
    deadline = time.monotonic() + timeout_s
    last_state = None

    while time.monotonic() < deadline:
        state = recorder.capture(phase, expected_target)
        if state is not None:
            last_state = state
            err = circular_error(state.present_position, expected_target)

            if err <= tolerance:
                recorder.capture(phase, expected_target, force_console=True)
                logger.info(
                    "%s raggiunto: present=%s target=%s err=%s tick "
                    "current_raw=%s status=0x%02X",
                    phase,
                    state.present_position,
                    expected_target,
                    err,
                    state.current,
                    state.error_status,
                )
                return True, state

        await asyncio.sleep(sample_interval_s)

    if last_state is None:
        raise RuntimeError(f"{phase}: nessuna telemetria disponibile")

    err = circular_error(last_state.present_position, expected_target)
    recorder.capture(phase, expected_target, force_console=True)
    logger.warning(
        "%s timeout: present=%s target=%s err=%s tick "
        "current_raw=%s status=0x%02X",
        phase,
        last_state.present_position,
        expected_target,
        err,
        last_state.current,
        last_state.error_status,
    )
    return False, last_state


def resolve_output_path(requested_path: str) -> Path:
    if requested_path:
        return Path(requested_path).expanduser().resolve()

    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    return DEFAULT_LOG_DIR / f"{stamp}_M13_micro_probe_diagnostic.json"


def write_artifact(
    output_path: Path,
    metadata: dict,
    result: dict,
    samples: list[dict],
):
    output_path.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "schema": 1,
        "kind": "MATDOG_ST3215_MICRO_PROBE_DIAGNOSTIC",
        "metadata": metadata,
        "result": result,
        "samples": samples,
    }

    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")

    logger.info("Artefatto diagnostico salvato: %s", output_path)


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
            "--tolerance."
        )

    if args.sample_interval <= 0:
        raise RuntimeError("--sample-interval deve essere > 0")

    if args.console_interval <= 0:
        raise RuntimeError("--console-interval deve essere > 0")

    client = None
    reader_task = None
    reader = None
    recorder = None
    bus_serial = None
    start_pos = None
    torque_enabled = False
    output_path = resolve_output_path(args.output)

    result = {
        "hold_initial_reached": None,
        "outbound_reached": None,
        "return_reached": None,
        "exception": None,
    }

    try:
        client = await new_station_client(args.server, logger)
        reader = BusReader(client, label=f"station@{args.server}")
        reader_task = asyncio.create_task(reader.run())

        logger.info("Aspetto il primo frame ST3215...")
        await wait_for_first_frame(reader)

        bus_serial = resolve_bus_serial(reader.latest, args.bus)
        initial_state = find_motor_state(reader.latest, bus_serial, args.motor_id)
        start_pos = initial_state.present_position
        target_pos = start_pos + args.delta

        if not 0 <= target_pos < MAX_STEPS:
            raise RuntimeError(
                f"Target {target_pos} fuori dall'intervallo 0..4095. "
                "Questo tool diagnostico non usa wrap-around."
            )

        logger.info("=== MATDOG MICRO-PROBE DIAGNOSTICO ===")
        logger.info("Bus: %s", bus_serial)
        logger.info("Servo: M%s", args.motor_id)
        logger.info("Present iniziale: %s", start_pos)
        logger.info("Target micro-probe: %s", target_pos)
        logger.info("Delta: %+d tick", args.delta)
        logger.info(
            "Speed=%s accel=%s hold=%.2fs timeout=%.2fs",
            args.speed,
            args.accel,
            args.hold,
            args.timeout,
        )

        if not args.execute:
            logger.info("DRY RUN: nessun torque e nessun comando inviato.")
            logger.info(
                "Per eseguire davvero: --execute --confirm DIAG_M%s",
                args.motor_id,
            )
            return

        expected_confirmation = f"DIAG_M{args.motor_id}"
        if args.confirm != expected_confirmation:
            raise RuntimeError(
                f"Conferma errata. Usa esattamente: --confirm {expected_confirmation}"
            )

        recorder = DiagnosticRecorder(
            reader=reader,
            bus_serial=bus_serial,
            motor_id=args.motor_id,
            console_interval_s=args.console_interval,
        )
        recorder.capture("START_TORQUE_OFF", start_pos, force_console=True)

        logger.info("Priming: goal uguale alla posizione attuale, torque OFF.")
        await send_goal(
            client,
            bus_serial,
            args.motor_id,
            start_pos,
            args.speed,
            args.accel,
        )
        await sample_window(
            recorder,
            "PRIMING_TORQUE_OFF",
            start_pos,
            0.15,
            args.sample_interval,
        )

        logger.info("Abilito torque SOLO su M%s", args.motor_id)
        await set_torque(client, bus_serial, [args.motor_id], enable=True)
        torque_enabled = True
        await sample_window(
            recorder,
            "TORQUE_ON_SETTLE",
            start_pos,
            0.25,
            args.sample_interval,
        )

        hold_ok, _ = await wait_for_position_diagnostic(
            recorder,
            "HOLD_INITIAL",
            start_pos,
            args.tolerance,
            args.timeout,
            args.sample_interval,
        )
        result["hold_initial_reached"] = hold_ok

        logger.info("Micro-movimento: %s -> %s", start_pos, target_pos)
        await send_goal(
            client,
            bus_serial,
            args.motor_id,
            target_pos,
            args.speed,
            args.accel,
        )

        await sample_window(
            recorder,
            "OUTBOUND_SETTLE",
            target_pos,
            args.hold,
            args.sample_interval,
        )

        outbound_ok, _ = await wait_for_position_diagnostic(
            recorder,
            "OUTBOUND_WAIT",
            target_pos,
            args.tolerance,
            args.timeout,
            args.sample_interval,
        )
        result["outbound_reached"] = outbound_ok

        logger.info("Ritorno automatico: %s -> %s", target_pos, start_pos)
        await send_goal(
            client,
            bus_serial,
            args.motor_id,
            start_pos,
            args.speed,
            args.accel,
        )

        await sample_window(
            recorder,
            "RETURN_SETTLE",
            start_pos,
            args.hold,
            args.sample_interval,
        )

        returned, _ = await wait_for_position_diagnostic(
            recorder,
            "RETURN_WAIT",
            start_pos,
            args.tolerance,
            args.timeout,
            args.sample_interval,
        )
        result["return_reached"] = returned

        if returned:
            logger.info("Ritorno iniziale verificato.")
        else:
            logger.warning(
                "Ritorno non entro tolleranza. "
                "L'artefatto contiene la telemetria completa."
            )

        logger.info("Micro-probe diagnostico completato.")

    except Exception as exc:
        result["exception"] = f"{type(exc).__name__}: {exc}"
        raise

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

                if recorder is not None:
                    recorder.capture(
                        "CLEANUP_RETURN_BEFORE_TORQUE_OFF",
                        start_pos,
                        force_console=True,
                    )
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

        if recorder is not None and args.execute:
            metadata = {
                "created_at_local": datetime.now().isoformat(timespec="seconds"),
                "server": args.server,
                "bus": bus_serial,
                "motor_id": args.motor_id,
                "start_position": start_pos,
                "target_position": (
                    start_pos + args.delta if start_pos is not None else None
                ),
                "delta_tick": args.delta,
                "speed": args.speed,
                "accel": args.accel,
                "hold_s": args.hold,
                "timeout_s": args.timeout,
                "tolerance_tick": args.tolerance,
                "sample_interval_s": args.sample_interval,
            }
            write_artifact(output_path, metadata, result, recorder.samples)

        if reader_task is not None:
            reader_task.cancel()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG — micro-probe diagnostico reversibile "
            "con telemetria di corrente"
        )
    )
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--delta", type=int, default=20)
    parser.add_argument("--speed", type=int, default=40)
    parser.add_argument("--accel", type=int, default=5)
    parser.add_argument("--hold", type=float, default=0.8)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--tolerance", type=int, default=10)
    parser.add_argument("--sample-interval", type=float, default=0.05)
    parser.add_argument("--console-interval", type=float, default=0.25)
    parser.add_argument(
        "--output",
        default="",
        help=(
            "Path opzionale per il JSON. "
            "Default: 09_Logs/Calibration_Sessions/<timestamp>_M13_micro_probe_diagnostic.json"
        ),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")

    args = parser.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
