#!/usr/bin/env python3
"""
MATDOG — watcher Station strettamente read-only per end-stop calibration.

Il watcher:
- apre un client NormaCore Station;
- segue esclusivamente `st3215/inference`;
- individua un bus seriale esplicito;
- verifica i 12 servo MATDOG attesi;
- raccoglie campioni con timestamp per singolo motore;
- controlla restart Station, timestamp monotoni, status e torque;
- produce esclusivamente un report JSON su stdout.

Non:
- importa `send_commands`;
- accoda dati nella queue `commands`;
- costruisce DriverCommand;
- abilita o disabilita torque;
- invia target, speed o acceleration;
- scrive RAM o EEPROM.

Nota lifecycle:
il client Station corrente avvia task interni non esposti. Questo programma è
quindi deliberatamente finito e process-scoped sotto asyncio.run(); alla fine
il loop cancella i task ancora pendenti.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import sys
from typing import Any

NORMACORE = Path.home() / "norma-core"
sys.path.insert(0, str(NORMACORE))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215

from matdog_endstop_station_telemetry import (
    StationMotorSnapshot,
    find_motor_reader,
    parse_motor_snapshot,
)


INFERENCE_QUEUE = "st3215/inference"
EXPECTED_MOTOR_IDS = (
    11, 12, 13,
    21, 22, 23,
    31, 32, 33,
    41, 42, 43,
)


class ReadonlyWatchError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotorWatchSummary:
    motor_id: int
    sample_count: int
    first_monotonic_stamp_ns: int
    last_monotonic_stamp_ns: int
    first_system_stamp_ns: int
    last_system_stamp_ns: int
    app_start_id: int
    present_tick_min: int
    present_tick_max: int
    current_raw_min: int
    current_raw_max: int
    status_values: tuple[int, ...]
    torque_values: tuple[bool, ...]
    goal_tick_last: int


@dataclass(frozen=True)
class ReadonlyWatchReport:
    schema: str
    generated_at_utc: str
    station_server: str
    bus_serial: str
    inference_queue: str
    requested_frames: int
    received_frames: int
    expected_motor_ids: tuple[int, ...]
    observed_motor_ids: tuple[int, ...]
    station_app_start_id: int
    all_status_zero: bool
    all_torque_disabled: bool
    timestamps_strictly_increasing: bool
    command_api_available: bool
    motor_commands_sent: bool
    eeprom_writes_sent: bool
    motors: tuple[MotorWatchSummary, ...]


class SnapshotAccumulator:
    def __init__(
        self,
        expected_motor_ids: tuple[int, ...] = EXPECTED_MOTOR_IDS,
    ):
        self.expected_motor_ids = tuple(expected_motor_ids)
        self.samples: dict[int, list[StationMotorSnapshot]] = {
            motor_id: []
            for motor_id in self.expected_motor_ids
        }
        self.app_start_id: int | None = None

    def add_frame(
        self,
        inference_state,
        bus_serial: str,
    ) -> None:
        observed: list[int] = []

        target_bus = None

        for bus in inference_state.get_buses() or []:
            bus_info = bus.get_bus()

            if (
                bus_info is not None
                and bus_info.get_serial_number() == bus_serial
            ):
                target_bus = bus
                break

        if target_bus is None:
            raise ReadonlyWatchError(
                f"Bus {bus_serial!r} non presente nel frame"
            )

        for motor in target_bus.get_motors() or []:
            observed.append(int(motor.get_id()))

        observed_ids = tuple(sorted(observed))

        if observed_ids != tuple(sorted(self.expected_motor_ids)):
            raise ReadonlyWatchError(
                "Set motori inatteso: "
                f"attesi={sorted(self.expected_motor_ids)}, "
                f"osservati={list(observed_ids)}"
            )

        for motor_id in self.expected_motor_ids:
            reader = find_motor_reader(
                inference_state,
                bus_serial,
                motor_id,
            )

            if reader is None:
                raise ReadonlyWatchError(
                    f"Motore {motor_id} assente dal frame"
                )

            snapshot = parse_motor_snapshot(reader)

            if self.app_start_id is None:
                self.app_start_id = snapshot.app_start_id
            elif snapshot.app_start_id != self.app_start_id:
                raise ReadonlyWatchError(
                    "Restart Station rilevato durante il watcher"
                )

            previous = self.samples[motor_id][-1:] or []

            if (
                previous
                and snapshot.monotonic_stamp_ns
                <= previous[0].monotonic_stamp_ns
            ):
                raise ReadonlyWatchError(
                    f"M{motor_id}: timestamp monotonic non crescente"
                )

            self.samples[motor_id].append(snapshot)

    def report(
        self,
        *,
        station_server: str,
        bus_serial: str,
        requested_frames: int,
        received_frames: int,
    ) -> ReadonlyWatchReport:
        if self.app_start_id is None:
            raise ReadonlyWatchError("Nessun campione acquisito")

        summaries: list[MotorWatchSummary] = []
        all_status_zero = True
        all_torque_disabled = True
        timestamps_strictly_increasing = True

        for motor_id in self.expected_motor_ids:
            motor_samples = self.samples[motor_id]

            if len(motor_samples) != received_frames:
                raise ReadonlyWatchError(
                    f"M{motor_id}: campioni {len(motor_samples)} "
                    f"diversi dai frame {received_frames}"
                )

            stamps = [
                sample.monotonic_stamp_ns
                for sample in motor_samples
            ]
            status_values = tuple(sorted({
                sample.error_status
                for sample in motor_samples
            }))
            torque_values = tuple(sorted({
                sample.torque_enabled
                for sample in motor_samples
            }))

            if any(value != 0 for value in status_values):
                all_status_zero = False

            if any(torque_values):
                all_torque_disabled = False

            if any(
                following <= current
                for current, following in zip(
                    stamps,
                    stamps[1:],
                )
            ):
                timestamps_strictly_increasing = False

            summaries.append(
                MotorWatchSummary(
                    motor_id=motor_id,
                    sample_count=len(motor_samples),
                    first_monotonic_stamp_ns=stamps[0],
                    last_monotonic_stamp_ns=stamps[-1],
                    first_system_stamp_ns=(
                        motor_samples[0].system_stamp_ns
                    ),
                    last_system_stamp_ns=(
                        motor_samples[-1].system_stamp_ns
                    ),
                    app_start_id=motor_samples[0].app_start_id,
                    present_tick_min=min(
                        sample.present_tick
                        for sample in motor_samples
                    ),
                    present_tick_max=max(
                        sample.present_tick
                        for sample in motor_samples
                    ),
                    current_raw_min=min(
                        sample.current_raw
                        for sample in motor_samples
                    ),
                    current_raw_max=max(
                        sample.current_raw
                        for sample in motor_samples
                    ),
                    status_values=status_values,
                    torque_values=torque_values,
                    goal_tick_last=motor_samples[-1].goal_tick,
                )
            )

        return ReadonlyWatchReport(
            schema="matdog.endstop.station_readonly_watch.v1",
            generated_at_utc=datetime.now(
                timezone.utc
            ).isoformat(),
            station_server=station_server,
            bus_serial=bus_serial,
            inference_queue=INFERENCE_QUEUE,
            requested_frames=requested_frames,
            received_frames=received_frames,
            expected_motor_ids=self.expected_motor_ids,
            observed_motor_ids=self.expected_motor_ids,
            station_app_start_id=self.app_start_id,
            all_status_zero=all_status_zero,
            all_torque_disabled=all_torque_disabled,
            timestamps_strictly_increasing=(
                timestamps_strictly_increasing
            ),
            command_api_available=False,
            motor_commands_sent=False,
            eeprom_writes_sent=False,
            motors=tuple(summaries),
        )


async def collect_readonly_report(
    *,
    server: str,
    bus_serial: str,
    frames: int,
    frame_timeout_s: float,
    logger: logging.Logger,
) -> ReadonlyWatchReport:
    if not bus_serial:
        raise ValueError(
            "bus_serial esplicito obbligatorio"
        )

    if frames <= 0:
        raise ValueError("frames deve essere > 0")

    if frame_timeout_s <= 0.0:
        raise ValueError(
            "frame_timeout_s deve essere > 0"
        )

    client = await new_station_client(server, logger)

    inference_queue: asyncio.Queue[Any] = asyncio.Queue(
        maxsize=max(4, frames)
    )
    error_queue = client.follow(
        INFERENCE_QUEUE,
        inference_queue,
    )

    accumulator = SnapshotAccumulator()
    received_frames = 0

    while received_frames < frames:
        if not error_queue.empty():
            error = error_queue.get_nowait()
            raise ReadonlyWatchError(
                f"Errore stream inference: {error}"
            )

        try:
            entry = await asyncio.wait_for(
                inference_queue.get(),
                timeout=frame_timeout_s,
            )
        except asyncio.TimeoutError as exc:
            raise ReadonlyWatchError(
                "Timeout in attesa di st3215/inference"
            ) from exc

        if entry is None:
            raise ReadonlyWatchError(
                "Stream st3215/inference terminato"
            )

        state = st3215.InferenceStateReader(entry.Data)
        accumulator.add_frame(state, bus_serial)
        received_frames += 1

    return accumulator.report(
        station_server=server,
        bus_serial=bus_serial,
        requested_frames=frames,
        received_frames=received_frames,
    )


async def main_async(args: argparse.Namespace) -> int:
    logger = logging.getLogger(
        "matdog_endstop_station_readonly_watch"
    )

    report = await collect_readonly_report(
        server=args.server,
        bus_serial=args.bus_serial,
        frames=args.frames,
        frame_timeout_s=args.frame_timeout,
        logger=logger,
    )

    print(json.dumps(
        asdict(report),
        indent=2,
        sort_keys=True,
    ))

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Watcher finito e strettamente read-only per "
            "st3215/inference MATDOG"
        )
    )
    parser.add_argument(
        "--server",
        default="localhost",
    )
    parser.add_argument(
        "--bus-serial",
        required=True,
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=10,
    )
    parser.add_argument(
        "--frame-timeout",
        type=float,
        default=5.0,
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(message)s",
    )

    return asyncio.run(main_async(args))


if __name__ == "__main__":
    raise SystemExit(main())
