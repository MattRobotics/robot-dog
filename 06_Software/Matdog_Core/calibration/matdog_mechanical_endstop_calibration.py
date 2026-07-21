#!/usr/bin/env python3
"""
MATDOG — mechanical end-stop calibration offline planner.

Questa prima implementazione costruisce e valida esclusivamente il piano
deterministico della calibrazione meccanica.

Non:
- apre NormaCore Station;
- apre porte seriali;
- importa API di comando motore;
- abilita torque;
- invia goal position;
- scrive EEPROM;
- espone una modalità --execute.

L'esecuzione hardware resta intenzionalmente bloccata dal contratto YAML.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from enum import Enum
import json
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = (
    REPO_ROOT
    / "06_Software/Matdog_Core/calibration/"
    / "MATDOG_JOINT_CALIBRATION.yaml"
)

EXPECTED_SCHEMA_VERSION = 4
EXPECTED_STATUS = "GEOMETRY_VALIDATED_OFFLINE_HARDWARE_BLOCKED"
EXPECTED_LEG_ORDER = ("LF", "RF", "RH", "LH")

JOINT_SUFFIXES = {
    "hip": "hip_joint",
    "upper_leg": "upper_leg_joint",
    "lower_leg": "lower_leg_joint",
}


class ContactState(str, Enum):
    FREE_MOTION = "FREE_MOTION"
    CONTACT_SUSPECTED = "CONTACT_SUSPECTED"
    CONTACT_CONFIRMED = "CONTACT_CONFIRMED"
    CONTACT_REPEATABLE = "CONTACT_REPEATABLE"
    AMBIGUOUS_CONTACT = "AMBIGUOUS_CONTACT"
    HARD_ABORT = "HARD_ABORT"


@dataclass(frozen=True)
class PlanStep:
    index: int
    leg: str
    phase: str
    action: str
    joint: str | None = None
    side: str | None = None
    attempt: int | None = None
    pose_ref: str | None = None
    dependent_leg: str | None = None
    expected_terminal_state: str | None = None
    note: str = ""


class ContractError(RuntimeError):
    pass


def _joint_name(leg: str, group: str) -> str:
    return f"{leg.lower()}_{JOINT_SUFFIXES[group]}"


def load_contract(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ContractError("Configurazione YAML non valida")

    if data.get("schema_version") != EXPECTED_SCHEMA_VERSION:
        raise ContractError(
            "schema_version inatteso: "
            f"{data.get('schema_version')!r}"
        )

    robot = data.get("robot", {})
    mechanical = data.get("mechanical_endstop_calibration", {})
    joints = data.get("joints", {})

    if tuple(robot.get("leg_order", [])) != EXPECTED_LEG_ORDER:
        raise ContractError("robot.leg_order inatteso")

    if tuple(mechanical.get("leg_order", [])) != EXPECTED_LEG_ORDER:
        raise ContractError(
            "mechanical_endstop_calibration.leg_order inatteso"
        )

    if mechanical.get("status") != EXPECTED_STATUS:
        raise ContractError(
            "stato mechanical end-stop inatteso: "
            f"{mechanical.get('status')!r}"
        )

    if mechanical.get("command_eligible") is not False:
        raise ContractError(
            "Il contratto deve restare command_eligible=false"
        )

    prerequisites = mechanical.get("prerequisites", {})
    for pose_name in (
        "hip_probe_pose_rad",
        "lower_probe_pose_rad",
        "rear_parking_pose_rad",
    ):
        if pose_name not in prerequisites:
            raise ContractError(f"Posa prerequisite mancante: {pose_name}")

    for leg in EXPECTED_LEG_ORDER:
        for group in ("hip", "upper_leg", "lower_leg"):
            name = _joint_name(leg, group)

            if name not in joints:
                raise ContractError(f"Joint mancante: {name}")

            joint = joints[name]

            for field in (
                "first_stand_limit_rad",
                "measured_contact_rad",
                "safe_limit_rad",
            ):
                if joint.get(field) != {"min": None, "max": None}:
                    raise ContractError(
                        f"{name}: {field} deve restare null "
                        "prima delle misure hardware"
                    )

    expected_dependencies = {
        "LF": {
            "park_leg": "LH",
            "parking_pose": "rear_parking_pose_rad",
        },
        "RF": {
            "park_leg": "RH",
            "parking_pose": "rear_parking_pose_rad",
        },
        "RH": None,
        "LH": None,
    }

    if mechanical.get("front_leg_dependencies") != expected_dependencies:
        raise ContractError("Dipendenze cross-leg inattese")

    return data


def build_plan(data: dict[str, Any]) -> list[PlanStep]:
    mechanical = data["mechanical_endstop_calibration"]
    dependencies = mechanical["front_leg_dependencies"]

    steps: list[PlanStep] = []

    def add(
        *,
        leg: str,
        phase: str,
        action: str,
        joint: str | None = None,
        side: str | None = None,
        attempt: int | None = None,
        pose_ref: str | None = None,
        dependent_leg: str | None = None,
        expected_terminal_state: str | None = None,
        note: str = "",
    ) -> None:
        steps.append(
            PlanStep(
                index=len(steps) + 1,
                leg=leg,
                phase=phase,
                action=action,
                joint=joint,
                side=side,
                attempt=attempt,
                pose_ref=pose_ref,
                dependent_leg=dependent_leg,
                expected_terminal_state=expected_terminal_state,
                note=note,
            )
        )

    def add_contact_pair(
        leg: str,
        phase: str,
        joint: str,
        side: str,
    ) -> None:
        add(
            leg=leg,
            phase=phase,
            action="APPROACH_UNTIL_CONTACT",
            joint=joint,
            side=side,
            attempt=1,
            expected_terminal_state=ContactState.CONTACT_CONFIRMED.value,
            note="Stop immediato al contatto confermato.",
        )
        add(
            leg=leg,
            phase=phase,
            action="BACKOFF_AND_VERIFY_RECOVERY",
            joint=joint,
            side=side,
            attempt=1,
            expected_terminal_state=ContactState.FREE_MOTION.value,
            note="Il recupero deve essere verificato prima della ripetizione.",
        )
        add(
            leg=leg,
            phase=phase,
            action="APPROACH_UNTIL_CONTACT",
            joint=joint,
            side=side,
            attempt=2,
            expected_terminal_state=ContactState.CONTACT_REPEATABLE.value,
            note="Confrontare la posizione con il primo contatto.",
        )

    for leg in EXPECTED_LEG_ORDER:
        dependency = dependencies[leg]

        if dependency is not None:
            add(
                leg=leg,
                phase="DEPENDENCY_PARK",
                action="MOVE_DEPENDENT_LEG_TO_POSE",
                pose_ref=dependency["parking_pose"],
                dependent_leg=dependency["park_leg"],
                note="Prerequisito geometrico per evitare collisioni.",
            )

        upper_joint = _joint_name(leg, "upper_leg")
        hip_joint = _joint_name(leg, "hip")
        lower_joint = _joint_name(leg, "lower_leg")

        for side in ("min", "max"):
            add_contact_pair(
                leg=leg,
                phase="UPPER_ENDSTOP",
                joint=upper_joint,
                side=side,
            )

        add(
            leg=leg,
            phase="UPPER_ENDSTOP",
            action="RETURN_JOINT_HOME",
            joint=upper_joint,
            expected_terminal_state=ContactState.FREE_MOTION.value,
        )

        add(
            leg=leg,
            phase="HIP_PREREQUISITE",
            action="MOVE_ACTIVE_LEG_TO_POSE",
            pose_ref="hip_probe_pose_rad",
        )

        for side in ("min", "max"):
            add_contact_pair(
                leg=leg,
                phase="HIP_ENDSTOP",
                joint=hip_joint,
                side=side,
            )

        add(
            leg=leg,
            phase="HIP_ENDSTOP",
            action="RETURN_JOINT_HOME",
            joint=hip_joint,
            expected_terminal_state=ContactState.FREE_MOTION.value,
        )

        add(
            leg=leg,
            phase="LOWER_PREREQUISITE",
            action="MOVE_ACTIVE_LEG_TO_POSE",
            pose_ref="lower_probe_pose_rad",
        )

        for side in ("min", "max"):
            add_contact_pair(
                leg=leg,
                phase="LOWER_ENDSTOP",
                joint=lower_joint,
                side=side,
            )

        add(
            leg=leg,
            phase="LEG_RESTORE",
            action="RETURN_ACTIVE_LEG_HOME",
            expected_terminal_state=ContactState.FREE_MOTION.value,
        )

        if dependency is not None:
            add(
                leg=leg,
                phase="DEPENDENCY_RESTORE",
                action="RETURN_DEPENDENT_LEG_HOME",
                dependent_leg=dependency["park_leg"],
                expected_terminal_state=ContactState.FREE_MOTION.value,
            )

    return steps


def make_document(
    config_path: Path,
    data: dict[str, Any],
    steps: list[PlanStep],
) -> dict[str, Any]:
    return {
        "schema": "matdog.mechanical_endstop.offline_plan.v1",
        "robot": data["robot"]["name"],
        "source_config": str(config_path),
        "hardware_execution_supported": False,
        "command_eligible": False,
        "station_connection_opened": False,
        "serial_connection_opened": False,
        "motor_commands_sent": False,
        "eeprom_writes_sent": False,
        "contact_state_model": [state.value for state in ContactState],
        "step_count": len(steps),
        "steps": [asdict(step) for step in steps],
    }


def print_summary(document: dict[str, Any]) -> None:
    steps = document["steps"]

    print("=== MATDOG MECHANICAL END-STOP — OFFLINE PLAN ===")
    print(f"Schema: {document['schema']}")
    print(f"Steps:  {document['step_count']}")
    print("Hardware execution supported: false")
    print("Motor commands sent: false")
    print("EEPROM writes sent: false")
    print("")

    for step in steps[:14]:
        target = step["joint"] or step["dependent_leg"] or step["pose_ref"]
        print(
            f"{step['index']:02d} "
            f"{step['leg']} "
            f"{step['phase']:<20} "
            f"{step['action']:<32} "
            f"{target or '-'}"
        )

    if len(steps) > 14:
        print(f"... altri {len(steps) - 14} step")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compila il piano offline della calibrazione meccanica MATDOG. "
            "Non supporta esecuzione hardware."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
    )
    parser.add_argument(
        "--format",
        choices=("summary", "json"),
        default="summary",
    )
    args = parser.parse_args()

    config_path = args.config.expanduser().resolve()
    data = load_contract(config_path)
    steps = build_plan(data)
    document = make_document(config_path, data, steps)

    if args.format == "json":
        print(json.dumps(document, indent=2, sort_keys=True))
    else:
        print_summary(document)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
