#!/usr/bin/env python3
"""
MATDOG — acquisizione read-only dello zero visuale.

Questo tool:
- legge soltanto st3215/inference da Station;
- cattura più frame dei 12 encoder;
- verifica la stabilità della posa;
- salva un file candidato di sessione.

Questo tool NON:
- abilita torque;
- invia goal;
- modifica MATDOG_JOINT_CALIBRATION.yaml;
- apre direttamente la seriale.
"""

import argparse
import asyncio
import hashlib
import logging
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

NORMACORE = Path.home() / "norma-core"
EXAMPLE_DIR = NORMACORE / "software/station/examples/st3215-remote-teleop-py"
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = REPO_ROOT / "06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml"
DEFAULT_LOG_DIR = REPO_ROOT / "09_Logs/Calibration_Sessions"

sys.path.insert(0, str(NORMACORE))
sys.path.insert(0, str(EXAMPLE_DIR))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215
from state import find_bus, parse_motor_state, resolve_bus_serial
from matdog_joint_math import circular_tick_summary

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger("matdog_capture_visual_zero")

JOINT_PARTS = ("hip", "upper_leg", "lower_leg")


class BusReader:
    def __init__(self, client, label: str):
        self.label = label
        self.latest = None
        self.frame_count = 0
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
            self.frame_count += 1


async def wait_for_first_frame(reader: BusReader, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while reader.latest is None:
        if time.monotonic() > deadline:
            raise RuntimeError(
                f"[{reader.label}] nessun frame ST3215 entro {timeout_s:.1f}s"
            )
        await asyncio.sleep(0.05)


async def wait_for_next_frame(reader: BusReader, previous_count: int, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while reader.frame_count <= previous_count:
        if time.monotonic() > deadline:
            raise RuntimeError(
                f"[{reader.label}] timeout in attesa del frame ST3215 successivo"
            )
        await asyncio.sleep(0.02)


def find_motor_state(inference_state, bus_serial: str, motor_id: int):
    bus = find_bus(inference_state, bus_serial)
    if bus is None:
        raise RuntimeError(f"Bus '{bus_serial}' non trovato nello stato Station")

    for motor in bus.get_motors() or []:
        if motor.get_id() == motor_id:
            return parse_motor_state(motor)

    raise RuntimeError(f"Motore {motor_id} non trovato sul bus '{bus_serial}'")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def ensure_safe_output_path(output_path: Path, config_path: Path) -> Path:
    """Accetta soltanto nuovi file YAML dentro il log directory MATDOG."""
    output_path = output_path.resolve()
    config_path = config_path.resolve()
    log_root = DEFAULT_LOG_DIR.resolve()

    if output_path == config_path:
        raise RuntimeError(
            "--output non può coincidere con MATDOG_JOINT_CALIBRATION.yaml"
        )

    try:
        output_path.relative_to(log_root)
    except ValueError as exc:
        raise RuntimeError(
            "--output deve stare sotto "
            f"{log_root}, ricevuto: {output_path}"
        ) from exc

    if output_path.suffix.lower() not in {".yaml", ".yml"}:
        raise RuntimeError(
            "--output deve avere estensione .yaml oppure .yml"
        )

    if output_path.exists():
        raise RuntimeError(
            "Il file candidato esiste già e non verrà sovrascritto: "
            f"{output_path}"
        )

    temporary_path = output_path.with_name(f".{output_path.name}.tmp")
    if temporary_path.exists():
        raise RuntimeError(
            "Esiste un file temporaneo di una cattura precedente: "
            f"{temporary_path}. Verificalo o rimuovilo manualmente."
        )

    return output_path


def write_yaml_atomically(output_path: Path, content: str) -> None:
    """Pubblica un candidato YAML senza sovrascrivere file esistenti.

    Il file temporaneo viene eliminato solo quando è stato creato da
    questa stessa esecuzione. Un temporaneo preesistente resta intatto.
    """
    temporary_path = output_path.with_name(f".{output_path.name}.tmp")
    temporary_created = False

    try:
        try:
            with temporary_path.open("x", encoding="utf-8") as handle:
                temporary_created = True
                handle.write(content)
                handle.flush()
                os.fsync(handle.fileno())
        except FileExistsError as exc:
            raise RuntimeError(
                "File temporaneo già esistente: "
                f"{temporary_path}. La cattura non viene proseguita."
            ) from exc

        try:
            # link() crea il nome finale solo se non esiste già.
            # Fallisce atomicamente con FileExistsError se qualcuno ha
            # creato il candidato fra il controllo iniziale e questo punto.
            os.link(temporary_path, output_path)
        except FileExistsError as exc:
            raise RuntimeError(
                "Il file candidato esiste già e non verrà sovrascritto: "
                f"{output_path}"
            ) from exc
    finally:
        if temporary_created and temporary_path.exists():
            temporary_path.unlink()


def load_joint_map(config_path: Path):
    data = yaml.safe_load(config_path.read_text())

    if data.get("schema_version") != 2:
        raise RuntimeError(
            f"schema_version atteso 2, trovato {data.get('schema_version')!r}"
        )

    if data.get("robot", {}).get("calibration_status") != (
        "DIRECTION_MAPPING_COMPLETE_ZERO_PENDING"
    ):
        raise RuntimeError(
            "calibration_status inatteso: il tool richiede "
            "DIRECTION_MAPPING_COMPLETE_ZERO_PENDING"
        )

    joints = data.get("joints", {})
    joint_map = []

    for leg in data.get("robot", {}).get("leg_order", []):
        prefix = leg.lower()

        for part in JOINT_PARTS:
            joint_name = f"{prefix}_{part}_joint"
            joint = joints.get(joint_name)

            if joint is None:
                raise RuntimeError(f"Joint mancante nel YAML: {joint_name}")

            if joint.get("servo_id") is None:
                raise RuntimeError(f"{joint_name}: servo_id mancante")

            if joint.get("direction") not in (-1, 1):
                raise RuntimeError(f"{joint_name}: direction invalida")

            if joint.get("zero_encoder_visual") is not None:
                raise RuntimeError(
                    f"{joint_name}: zero_encoder_visual già impostato. "
                    "Questo tool non sovrascrive calibrazioni esistenti."
                )

            joint_map.append(
                {
                    "joint_name": joint_name,
                    "servo_id": int(joint["servo_id"]),
                    "direction": int(joint["direction"]),
                    "joint_group": joint["joint_group"],
                }
            )

    if len(joint_map) != 12:
        raise RuntimeError(f"Attesi 12 joint, trovati {len(joint_map)}")

    return joint_map


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    return DEFAULT_LOG_DIR / f"{timestamp}_visual_zero_capture.result.yaml"


async def main_async(args):
    if args.samples < 3:
        raise RuntimeError("--samples deve essere almeno 3")

    if args.max_spread < 0:
        raise RuntimeError("--max-spread non può essere negativo")

    config_path = Path(args.config).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else default_output_path()
    )

    output_path = ensure_safe_output_path(output_path, config_path)
    joint_map = load_joint_map(config_path)

    client = None
    reader_task = None

    try:
        client = await new_station_client(args.server, logger)
        reader = BusReader(client, label=f"station@{args.server}")
        reader_task = asyncio.create_task(reader.run())

        logger.info("=== MATDOG VISUAL ZERO CAPTURE — READ ONLY ===")
        logger.info("Nessun torque, goal o comando motore verrà inviato.")
        logger.info("Aspetto il primo frame ST3215...")
        await wait_for_first_frame(reader, args.timeout)

        bus_serial = resolve_bus_serial(reader.latest, args.bus)
        logger.info("Bus: %s", bus_serial)
        logger.info("Campioni per joint: %s", args.samples)
        logger.info("Spread massimo ammesso: %s tick", args.max_spread)

        samples = {item["joint_name"]: [] for item in joint_map}
        previous_frame_count = reader.frame_count

        for sample_index in range(1, args.samples + 1):
            await wait_for_next_frame(reader, previous_frame_count, args.timeout)
            previous_frame_count = reader.frame_count

            for item in joint_map:
                state = find_motor_state(
                    reader.latest,
                    bus_serial,
                    item["servo_id"],
                )
                samples[item["joint_name"]].append(int(state.present_position))

            logger.info("Campione %s/%s acquisito", sample_index, args.samples)

        result_joints = {}
        unstable = []

        for item in joint_map:
            joint_name = item["joint_name"]
            values = samples[joint_name]
            raw_low = min(values)
            raw_high = max(values)
            candidate, spread = circular_tick_summary(values)

            result_joints[joint_name] = {
                "servo_id": item["servo_id"],
                "direction": item["direction"],
                "joint_group": item["joint_group"],
                "samples_ticks": values,
                "raw_min_ticks": raw_low,
                "raw_max_ticks": raw_high,
                "circular_spread_ticks": spread,
                "zero_encoder_visual_candidate": candidate,
            }

            if spread > args.max_spread:
                unstable.append((joint_name, spread, values))

        print("\n=== RISULTATO ZERO VISUALE — CANDIDATO ===")
        print("joint                     servo  median  raw_min raw_max circ_spread")
        print("----------------------------------------------------------")

        for item in joint_map:
            row = result_joints[item["joint_name"]]
            print(
                f"{item['joint_name']:25} "
                f"M{row['servo_id']:02d}   "
                f"{row['zero_encoder_visual_candidate']:4d}   "
                f"{row['raw_min_ticks']:4d} "
                f"{row['raw_max_ticks']:4d} "
                f"{row['circular_spread_ticks']:3d}"
            )

        if unstable:
            print("\nERRORE: posa non sufficientemente stabile.")
            for joint_name, spread, values in unstable:
                print(
                    f"- {joint_name}: spread={spread} tick, samples={values}"
                )
            print(
                "\nNessun file candidato è stato scritto. "
                "Riposiziona il robot, attendi che smetta di assestarsi "
                "e ripeti la cattura."
            )
            raise RuntimeError(
                f"Stabilità fallita: {len(unstable)} joint oltre "
                f"{args.max_spread} tick"
            )

        output_path.parent.mkdir(parents=True, exist_ok=True)

        config_hash = sha256_file(config_path)
        config_data = yaml.safe_load(config_path.read_text())
        canonical_urdf = (
            config_data.get("kinematic_model", {})
            .get("canonical_urdf", {})
        )

        result = {
            "session_id": output_path.stem.replace(".result", ""),
            "robot": "MATDOG",
            "operation": "visual_zero_capture_read_only",
            "status": "PASS_STABLE_CAPTURE_CANDIDATE_NOT_APPLIED",
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "station": {
                "server": args.server,
                "bus_serial": bus_serial,
            },
            "capture": {
                "samples_per_joint": args.samples,
                "max_spread_ticks": args.max_spread,
                "config_source": str(config_path),
                "config_sha256": config_hash,
                "canonical_urdf_contract": canonical_urdf,
                "note": (
                    "Read-only capture. "
                    "MATDOG_JOINT_CALIBRATION.yaml non è stato modificato."
                ),
            },
            "joints": result_joints,
        }

        yaml_content = yaml.safe_dump(
            result,
            sort_keys=False,
            allow_unicode=True,
        )
        write_yaml_atomically(output_path, yaml_content)

        print("\nPASS: posa stabile.")
        print(f"File candidato salvato: {output_path}")
        print(
            "Il YAML di calibrazione non è stato modificato. "
            "La candidata dovrà essere revisionata prima dell'applicazione."
        )

    finally:
        if reader_task is not None:
            reader_task.cancel()
            try:
                await reader_task
            except asyncio.CancelledError:
                pass

        if client is not None:
            await client.close()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG read-only visual zero capture da telemetria Station. "
            "Non invia comandi ai servo."
        )
    )
    parser.add_argument("--server", default="localhost:8888")
    parser.add_argument("--bus", default="auto")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--output", default=None)
    parser.add_argument("--samples", type=int, default=18)
    parser.add_argument("--max-spread", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=10.0)

    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.error("Interrotto dall'utente.")
        raise SystemExit(130)
    except Exception as exc:
        logger.error("ERRORE: %s", exc)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
