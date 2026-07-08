#!/usr/bin/env python3
"""
MATDOG — FK live read-only di una zampa MATDOG.

Questo tool:
- si collega esclusivamente alla telemetria pubblicata da NormaCore Station;
- non apre direttamente la seriale;
- non abilita/disabilita torque;
- non invia goal, speed o accel;
- converte encoder -> radianti con la calibrazione MATDOG;
- calcola la FK con il vero URDF canonico.

Uso previsto:
1. Robot alimentato e Station già avviata.
2. Torque disabilitato prima di qualunque manipolazione manuale.
3. Esecuzione read-only per osservare encoder, q URDF e foot frame.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import logging
import math
from pathlib import Path
import sys
import time

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
CALIBRATION_DIR = REPO_ROOT / "06_Software/Matdog_Core/calibration"
DEFAULT_CONFIG = CALIBRATION_DIR / "MATDOG_JOINT_CALIBRATION.yaml"

NORMACORE = Path.home() / "norma-core"
EXAMPLE_DIR = (
    NORMACORE
    / "software/station/examples/st3215-remote-teleop-py"
)

sys.path.insert(0, str(CALIBRATION_DIR))
sys.path.insert(0, str(NORMACORE))
sys.path.insert(0, str(EXAMPLE_DIR))

from matdog_joint_math import encoder_to_joint_rad, signed_tick_delta
from matdog_urdf_fk import (
    CANONICAL_URDF_SHA256,
    canonical_urdf_path,
    forward_kinematics,
    sha256_file,
)


EXPECTED_CALIBRATION_STATUS = "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"

LEG_ORDER = ("lf", "rf", "rh", "lh")

LEG_JOINT_NAMES = {
    leg: (
        f"{leg}_hip_joint",
        f"{leg}_upper_leg_joint",
        f"{leg}_lower_leg_joint",
    )
    for leg in LEG_ORDER
}

LEG_TIP_LINKS = {
    leg: f"{leg}_foot_link"
    for leg in LEG_ORDER
}


def normalize_leg_id(value: str) -> str:
    leg_id = value.lower()

    if leg_id not in LEG_JOINT_NAMES:
        expected = ", ".join(LEG_ORDER)
        raise ValueError(
            f"leg non valida: {value!r}; valori ammessi: {expected}"
        )

    return leg_id


@dataclass(frozen=True)
class LegJointCalibration:
    joint_name: str
    servo_id: int
    direction: int
    zero_encoder_visual: int


def load_leg_calibration(
    config_path: Path,
    leg_id: str,
) -> tuple[LegJointCalibration, ...]:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    status = data.get("robot", {}).get("calibration_status")
    if status != EXPECTED_CALIBRATION_STATUS:
        raise RuntimeError(
            "calibration_status inatteso: "
            f"{status!r}; atteso {EXPECTED_CALIBRATION_STATUS!r}"
        )

    joints = data.get("joints", {})
    result: list[LegJointCalibration] = []

    for joint_name in LEG_JOINT_NAMES[leg_id]:
        joint = joints.get(joint_name)

        if not isinstance(joint, dict):
            raise RuntimeError(
                f"Configurazione mancante o invalida per {joint_name}"
            )

        servo_id = joint.get("servo_id")
        direction = joint.get("direction")
        zero_tick = joint.get("zero_encoder_visual")

        if not isinstance(servo_id, int) or not 1 <= servo_id <= 253:
            raise RuntimeError(
                f"{joint_name}: servo_id non valido: {servo_id!r}"
            )

        if direction not in (-1, 1):
            raise RuntimeError(
                f"{joint_name}: direction non valida: {direction!r}"
            )

        if not isinstance(zero_tick, int) or not 0 <= zero_tick <= 4095:
            raise RuntimeError(
                f"{joint_name}: zero_encoder_visual non valido: {zero_tick!r}"
            )

        result.append(
            LegJointCalibration(
                joint_name=joint_name,
                servo_id=servo_id,
                direction=direction,
                zero_encoder_visual=zero_tick,
            )
        )

    servo_ids = [item.servo_id for item in result]

    if len(set(servo_ids)) != len(servo_ids):
        raise RuntimeError(
            f"Servo {leg_id.upper()} duplicati nella calibrazione: {servo_ids}"
        )

    return tuple(result)


def verify_canonical_urdf(repo_root: Path) -> Path:
    urdf_path = canonical_urdf_path(repo_root)

    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF canonico non trovato: {urdf_path}")

    actual_sha256 = sha256_file(urdf_path)

    if actual_sha256 != CANONICAL_URDF_SHA256:
        raise RuntimeError(
            "Integrità URDF fallita: "
            f"sha256={actual_sha256}, atteso={CANONICAL_URDF_SHA256}"
        )

    return urdf_path


def joint_radians_from_encoder_ticks(
    calibration: tuple[LegJointCalibration, ...],
    encoder_ticks_by_servo: dict[int, int],
) -> dict[str, float]:
    expected_servo_ids = {item.servo_id for item in calibration}
    present_servo_ids = set(encoder_ticks_by_servo)

    missing = expected_servo_ids - present_servo_ids
    unexpected = present_servo_ids - expected_servo_ids

    if missing or unexpected:
        raise ValueError(
            "Mappa encoder zampa non valida: "
            f"missing={sorted(missing)}, unexpected={sorted(unexpected)}"
        )

    result: dict[str, float] = {}

    for item in calibration:
        present_tick = encoder_ticks_by_servo[item.servo_id]

        if not isinstance(present_tick, int):
            raise TypeError(
                f"M{item.servo_id}: encoder non intero: {present_tick!r}"
            )

        result[item.joint_name] = encoder_to_joint_rad(
            present_tick=present_tick,
            zero_tick=item.zero_encoder_visual,
            direction=item.direction,
        )

    return result


def visual_zero_errors_ticks(
    calibration: tuple[LegJointCalibration, ...],
    encoder_ticks_by_servo: dict[int, int],
) -> dict[str, int]:
    errors: dict[str, int] = {}

    for item in calibration:
        present_tick = encoder_ticks_by_servo[item.servo_id]

        if not isinstance(present_tick, int):
            raise TypeError(
                f"M{item.servo_id}: encoder non intero: {present_tick!r}"
            )

        errors[item.joint_name] = abs(
            signed_tick_delta(
                present_tick,
                item.zero_encoder_visual,
            )
        )

    return errors


def fk_from_encoder_ticks(
    repo_root: Path,
    calibration: tuple[LegJointCalibration, ...],
    encoder_ticks_by_servo: dict[int, int],
    tip_link: str,
):
    urdf_path = verify_canonical_urdf(repo_root)

    joint_positions_rad = joint_radians_from_encoder_ticks(
        calibration,
        encoder_ticks_by_servo,
    )

    result = forward_kinematics(
        urdf_path=urdf_path,
        root_link="base_link",
        tip_link=tip_link,
        joint_positions_rad=joint_positions_rad,
        enforce_limits=True,
    )

    return joint_positions_rad, result


STATION_IMPORT_ERROR: Exception | None = None

try:
    from software.station.shared.station_py import new_station_client
    from target.gen_python.protobuf.drivers.st3215 import st3215
    from state import find_bus, parse_motor_state, resolve_bus_serial
except Exception as exc:
    STATION_IMPORT_ERROR = exc


def require_station_dependencies() -> None:
    if STATION_IMPORT_ERROR is not None:
        raise RuntimeError(
            "Dipendenze Station non disponibili. "
            "Esegui questo tool sull'host MATDOG con NormaCore installato."
        ) from STATION_IMPORT_ERROR


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
                    "inference stream error: "
                    f"{self._error_queue.get_nowait()}"
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


async def wait_first_frame(reader: BusReader, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s

    while reader.latest is None:
        if time.monotonic() > deadline:
            raise RuntimeError(
                f"Nessun frame ST3215 entro {timeout_s:.1f}s"
            )
        await asyncio.sleep(0.05)


async def wait_next_frame(
    reader: BusReader,
    previous_count: int,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s

    while reader.frame_count <= previous_count:
        if time.monotonic() > deadline:
            raise RuntimeError(
                "Timeout in attesa del frame ST3215 successivo"
            )
        await asyncio.sleep(0.02)


def read_encoder_ticks(
    inference_state,
    bus_serial: str,
    calibration: tuple[LegJointCalibration, ...],
) -> dict[int, int]:
    bus = find_bus(inference_state, bus_serial)

    if bus is None:
        raise RuntimeError(f"Bus {bus_serial!r} non trovato")

    expected_servo_ids = {item.servo_id for item in calibration}
    result: dict[int, int] = {}

    for motor in bus.get_motors() or []:
        motor_id = motor.get_id()

        if motor_id in expected_servo_ids:
            result[motor_id] = int(
                parse_motor_state(motor).present_position
            )

    missing = expected_servo_ids - set(result)

    if missing:
        raise RuntimeError(
            f"Motori attesi mancanti sul bus {bus_serial}: {sorted(missing)}"
        )

    return result


def print_sample(
    elapsed_s: float,
    calibration: tuple[LegJointCalibration, ...],
    encoder_ticks: dict[int, int],
    joint_positions_rad: dict[str, float],
    foot_position_m: tuple[float, float, float],
    tip_link: str,
) -> None:
    fields = []

    for item in calibration:
        tick = encoder_ticks[item.servo_id]
        q_rad = joint_positions_rad[item.joint_name]
        fields.append(
            f"M{item.servo_id:02d}={tick:4d} "
            f"q={q_rad:+.5f} rad ({math.degrees(q_rad):+.2f}°)"
        )

    x, y, z = foot_position_m

    print(f"[{elapsed_s:6.2f}s] " + " | ".join(fields))
    print(
        f"  {tip_link} in base_link: "
        f"X={x:+.6f} m  Y={y:+.6f} m  Z={z:+.6f} m"
    )


async def main_async(args) -> None:
    require_station_dependencies()

    if args.duration <= 0.0:
        raise ValueError("--duration deve essere > 0")

    if args.print_period <= 0.0:
        raise ValueError("--print-period deve essere > 0")

    if (
        args.require_visual_zero_tolerance is not None
        and args.require_visual_zero_tolerance < 0
    ):
        raise ValueError(
            "--require-visual-zero-tolerance deve essere >= 0"
        )

    leg_id = normalize_leg_id(args.leg)
    tip_link = LEG_TIP_LINKS[leg_id]

    config_path = Path(args.config).expanduser().resolve()
    calibration = load_leg_calibration(config_path, leg_id)
    urdf_path = verify_canonical_urdf(REPO_ROOT)

    logger = logging.getLogger("matdog_leg_fk_live")
    client = None
    reader_task = None

    try:
        client = await new_station_client(args.server, logger)
        reader = BusReader(client)
        reader_task = asyncio.create_task(reader.run())

        await wait_first_frame(reader, args.timeout)
        bus_serial = resolve_bus_serial(reader.latest, args.bus)

        print(f"=== MATDOG {leg_id.upper()} LIVE FK — READ ONLY ===")
        print("Questo tool non invia torque, target, speed o accel.")
        print(
            "Prima di muovere manualmente il robot, verifica "
            "fisicamente torque OFF."
        )
        print(f"Bus: {bus_serial}")
        print(f"Config: {config_path}")
        print(f"URDF: {urdf_path}")
        print(
            f"{leg_id.upper()} servo order: "
            + ", ".join(
                f"{item.joint_name}=M{item.servo_id}"
                for item in calibration
            )
        )
        print("")

        start = time.monotonic()
        deadline = start + args.duration
        previous_frame_count = reader.frame_count
        last_print_time = -math.inf
        last_ticks: tuple[int, int, int] | None = None
        visual_zero_checked = False

        while time.monotonic() < deadline:
            await wait_next_frame(
                reader,
                previous_frame_count,
                args.timeout,
            )
            previous_frame_count = reader.frame_count

            encoder_ticks = read_encoder_ticks(
                reader.latest,
                bus_serial,
                calibration,
            )

            tick_tuple = tuple(
                encoder_ticks[item.servo_id]
                for item in calibration
            )

            q_rad, fk_result = fk_from_encoder_ticks(
                REPO_ROOT,
                calibration,
                encoder_ticks,
                tip_link,
            )

            if (
                args.require_visual_zero_tolerance is not None
                and not visual_zero_checked
            ):
                errors = visual_zero_errors_ticks(
                    calibration,
                    encoder_ticks,
                )

                exceeded = {
                    name: error
                    for name, error in errors.items()
                    if error > args.require_visual_zero_tolerance
                }

                if exceeded:
                    raise RuntimeError(
                        "Preflight visual-zero fallito: "
                        f"tolleranza={args.require_visual_zero_tolerance} tick, "
                        f"errori={errors}"
                    )

                print(
                    "VISUAL-ZERO PREFLIGHT PASS: "
                    f"tutti i joint {leg_id.upper()} entro "
                    f"{args.require_visual_zero_tolerance} tick. "
                    f"errori={errors}"
                )
                print("")
                visual_zero_checked = True

            now = time.monotonic()

            if (
                tick_tuple != last_ticks
                or now - last_print_time >= args.print_period
            ):
                print_sample(
                    elapsed_s=now - start,
                    calibration=calibration,
                    encoder_ticks=encoder_ticks,
                    joint_positions_rad=q_rad,
                    foot_position_m=fk_result.tip_position_m,
                    tip_link=tip_link,
                )
                last_ticks = tick_tuple
                last_print_time = now

        print("\nTIMEOUT: FK live conclusa senza inviare alcun comando.")

    finally:
        if reader_task is not None:
            reader_task.cancel()

            try:
                await reader_task
            except asyncio.CancelledError:
                pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG FK live read-only: "
            "encoder Station -> radianti URDF -> foot frame."
        )
    )
    parser.add_argument("--leg", default="lf", help="Zampa: lf, rf, rh oppure lh")
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--print-period", type=float, default=0.50)
    parser.add_argument(
        "--require-visual-zero-tolerance",
        type=int,
        default=None,
        metavar="TICKS",
        help=(
            "Preflight read-only: blocca l'esecuzione se la zampa scelta "
            "non è entro questa tolleranza dallo zero visuale."
        ),
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(message)s",
    )

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente: nessun comando inviato.")
    except Exception as exc:
        print(f"\nERRORE: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
