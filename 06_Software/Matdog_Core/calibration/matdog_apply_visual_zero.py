#!/usr/bin/env python3
"""
MATDOG — promozione controllata di una cattura visual-zero candidata.

Non apre Station, non invia comandi e non modifica URDF.
Senza --apply esegue solo la preflight validation.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = (
    REPO_ROOT
    / "06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml"
)
EXPECTED_CAPTURE_OPERATION = "visual_zero_capture_read_only"
EXPECTED_CAPTURE_STATUS = "PASS_STABLE_CAPTURE_CANDIDATE_NOT_APPLIED"
SOURCE_STATUS = "DIRECTION_MAPPING_COMPLETE_ZERO_PENDING"
TARGET_STATUS = "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"


def die(message: str) -> None:
    raise SystemExit(f"ERRORE: {message}")


def load_yaml(path: Path) -> dict:
    if not path.is_file():
        die(f"file non trovato: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        die(f"YAML non valido: {path}")
    return data


def candidate_value(candidate_joint: dict, joint_name: str) -> int:
    value = candidate_joint.get("zero_encoder_visual_candidate")
    if not isinstance(value, int) or not 0 <= value <= 4095:
        die(f"{joint_name}: candidato encoder non valido: {value!r}")

    spread = candidate_joint.get("circular_spread_ticks")
    if not isinstance(spread, int) or spread < 0:
        die(f"{joint_name}: circular_spread_ticks non valido: {spread!r}")

    return value


def replace_exactly_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        die(f"{label}: attesa una sola occorrenza, trovate {count}")
    return text.replace(old, new, 1)


def replace_joint_zero(
    text: str,
    joint_name: str,
    zero_value: int,
) -> str:
    pattern = (
        rf"(?ms)"
        rf"(^  {re.escape(joint_name)}:\n.*?"
        rf"^    zero_encoder_visual: )null$"
    )

    updated, count = re.subn(
        pattern,
        rf"\g<1>{zero_value}",
        text,
        count=1,
    )

    if count != 1:
        die(
            f"{joint_name}: impossibile sostituire zero_encoder_visual "
            "in modo univoco"
        )

    return updated


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Valida e applica una candidata visual-zero MATDOG. "
            "Senza --apply non modifica nulla."
        )
    )
    parser.add_argument(
        "--candidate",
        required=True,
        help="File .result.yaml prodotto da matdog_capture_visual_zero.py",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="MATDOG_JOINT_CALIBRATION.yaml da aggiornare",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Applica davvero i 12 zero_encoder_visual",
    )
    args = parser.parse_args()

    candidate_path = Path(args.candidate).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()

    candidate = load_yaml(candidate_path)
    config = load_yaml(config_path)

    if candidate.get("operation") != EXPECTED_CAPTURE_OPERATION:
        die(
            "operation candidata inattesa: "
            f"{candidate.get('operation')!r}"
        )

    if candidate.get("status") != EXPECTED_CAPTURE_STATUS:
        die(
            "status candidata inatteso: "
            f"{candidate.get('status')!r}"
        )

    if config.get("schema_version") != 2:
        die(f"schema_version config inattesa: {config.get('schema_version')!r}")

    robot = config.get("robot", {})
    if robot.get("calibration_status") != SOURCE_STATUS:
        die(
            "calibration_status non applicabile: "
            f"{robot.get('calibration_status')!r}"
        )

    config_joints = config.get("joints", {})
    candidate_joints = candidate.get("joints", {})

    if set(config_joints) != set(candidate_joints):
        missing = sorted(set(config_joints) - set(candidate_joints))
        extra = sorted(set(candidate_joints) - set(config_joints))
        die(f"joint non coerenti. missing={missing}, extra={extra}")

    values: dict[str, int] = {}

    for joint_name, joint in config_joints.items():
        capture_joint = candidate_joints[joint_name]

        if capture_joint.get("servo_id") != joint.get("servo_id"):
            die(
                f"{joint_name}: servo_id candidato="
                f"{capture_joint.get('servo_id')!r}, config="
                f"{joint.get('servo_id')!r}"
            )

        if capture_joint.get("direction") != joint.get("direction"):
            die(
                f"{joint_name}: direction candidata="
                f"{capture_joint.get('direction')!r}, config="
                f"{joint.get('direction')!r}"
            )

        if joint.get("zero_encoder_visual") is not None:
            die(
                f"{joint_name}: zero_encoder_visual già presente. "
                "Questo tool non sovrascrive una calibrazione."
            )

        values[joint_name] = candidate_value(capture_joint, joint_name)

    print("=== MATDOG APPLY VISUAL ZERO — PREFLIGHT PASS ===")
    print(f"Config:    {config_path}")
    print(f"Candidate: {candidate_path}")
    print()
    print("joint                     servo  zero_candidate")
    print("------------------------------------------------")
    for joint_name, joint in config_joints.items():
        print(
            f"{joint_name:25} "
            f"M{joint['servo_id']:02d}   "
            f"{values[joint_name]:4d}"
        )

    if not args.apply:
        print(
            "\nDRY RUN: nessun file è stato modificato. "
            "Riesegui con --apply solo dopo revisione."
        )
        return 0

    original = config_path.read_text(encoding="utf-8")
    updated = replace_exactly_once(
        original,
        f"calibration_status: {SOURCE_STATUS}",
        f"calibration_status: {TARGET_STATUS}",
        "calibration_status",
    )

    for joint_name, zero_value in values.items():
        updated = replace_joint_zero(updated, joint_name, zero_value)

    backup_path = config_path.with_suffix(
        config_path.suffix + ".before_visual_zero_backup"
    )
    if backup_path.exists():
        die(f"backup già esistente: {backup_path}")

    backup_path.write_text(original, encoding="utf-8")
    config_path.write_text(updated, encoding="utf-8")

    print("\nAPPLY PASS")
    print(f"Backup config: {backup_path}")
    print(f"Config aggiornata: {config_path}")
    print(f"Nuovo stato: {TARGET_STATUS}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
