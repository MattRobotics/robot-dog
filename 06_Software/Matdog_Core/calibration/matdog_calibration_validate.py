#!/usr/bin/env python3
"""MATDOG — validatore del contratto calibrazione <-> URDF.

Read-only:
- non apre Station;
- non invia comandi;
- non modifica YAML;
- non modifica URDF.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = (
    REPO_ROOT
    / "06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml"
)
CANONICAL_URDF_RELATIVE_PATH = (
    "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
)

EXPECTED_JOINTS = {
    "lf_hip_joint":       (13, -1, "hip"),
    "lf_upper_leg_joint": (12,  1, "upper_leg"),
    "lf_lower_leg_joint": (11, -1, "lower_leg"),
    "rf_hip_joint":       (23, -1, "hip"),
    "rf_upper_leg_joint": (22, -1, "upper_leg"),
    "rf_lower_leg_joint": (21,  1, "lower_leg"),
    "rh_hip_joint":       (33,  1, "hip"),
    "rh_upper_leg_joint": (32, -1, "upper_leg"),
    "rh_lower_leg_joint": (31,  1, "lower_leg"),
    "lh_hip_joint":       (43,  1, "hip"),
    "lh_upper_leg_joint": (42,  1, "upper_leg"),
    "lh_lower_leg_joint": (41, -1, "lower_leg"),
}

EXPECTED_AXIS = {
    "hip": (1.0, 0.0, 0.0),
    "upper_leg": (0.0, 1.0, 0.0),
    "lower_leg": (0.0, 1.0, 0.0),
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_axis(node: ET.Element) -> tuple[float, float, float]:
    axis = node.find("axis")
    if axis is None:
        raise ValueError("axis URDF mancante")

    raw = axis.attrib.get("xyz")
    if raw is None:
        raise ValueError("axis URDF senza attributo xyz")

    values = tuple(float(value) for value in raw.split())
    if len(values) != 3:
        raise ValueError(f"axis URDF invalido: {raw!r}")

    return values


def same_axis(actual: tuple[float, float, float],
              expected: tuple[float, float, float]) -> bool:
    return all(abs(a - e) <= 1e-12 for a, e in zip(actual, expected))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Valida il contratto MATDOG YAML <-> URDF."
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    if not config_path.is_file():
        raise SystemExit(f"ERRORE: config non trovato: {config_path}")

    data = yaml.safe_load(config_path.read_text())
    errors: list[str] = []

    if data.get("schema_version") != 2:
        errors.append(
            f"schema_version atteso 2, trovato {data.get('schema_version')!r}"
        )

    robot = data.get("robot", {})
    if robot.get("leg_order") != ["LF", "RF", "RH", "LH"]:
        errors.append(f"leg_order inatteso: {robot.get('leg_order')!r}")

    pre_zero_status = "DIRECTION_MAPPING_COMPLETE_ZERO_PENDING"
    visual_zero_status = "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"
    calibration_status = robot.get("calibration_status")

    if calibration_status not in {pre_zero_status, visual_zero_status}:
        errors.append(
            "calibration_status inatteso: "
            f"{calibration_status!r}"
        )

    model = data.get("kinematic_model", {}).get("canonical_urdf", {})
    declared_path = model.get("path")
    declared_hash = model.get("sha256")

    if declared_path != CANONICAL_URDF_RELATIVE_PATH:
        errors.append(
            "path URDF inatteso: "
            f"atteso {CANONICAL_URDF_RELATIVE_PATH!r}, trovato {declared_path!r}"
        )

    if not isinstance(declared_hash, str) or len(declared_hash) != 64:
        errors.append(f"sha256 URDF invalido: {declared_hash!r}")

    urdf_path = (REPO_ROOT / CANONICAL_URDF_RELATIVE_PATH).resolve()
    if not urdf_path.is_file():
        errors.append(f"URDF canonico non trovato: {urdf_path}")

    actual_hash = sha256_file(urdf_path) if urdf_path.is_file() else None
    if actual_hash is not None and declared_hash != actual_hash:
        errors.append(
            "SHA256 URDF non coerente: "
            f"YAML={declared_hash!r}, file={actual_hash!r}"
        )

    joints = data.get("joints", {})
    if set(joints) != set(EXPECTED_JOINTS):
        errors.append(
            "insieme joint YAML non coerente: "
            f"trovati={sorted(joints)}"
        )

    servo_ids: list[int] = []

    for joint_name, (servo_id, direction, group) in EXPECTED_JOINTS.items():
        joint = joints.get(joint_name, {})

        if joint.get("servo_id") != servo_id:
            errors.append(
                f"{joint_name}: servo_id atteso {servo_id}, "
                f"trovato {joint.get('servo_id')!r}"
            )

        if joint.get("direction") != direction:
            errors.append(
                f"{joint_name}: direction attesa {direction:+d}, "
                f"trovata {joint.get('direction')!r}"
            )

        if joint.get("joint_group") != group:
            errors.append(
                f"{joint_name}: joint_group atteso {group!r}, "
                f"trovato {joint.get('joint_group')!r}"
            )

        reference = joint.get("provisional_reference_encoder")
        if not isinstance(reference, int) or not 0 <= reference <= 4095:
            errors.append(
                f"{joint_name}: provisional_reference_encoder invalido: "
                f"{reference!r}"
            )

        zero_visual = joint.get("zero_encoder_visual")

        if calibration_status == pre_zero_status:
            if zero_visual is not None:
                errors.append(
                    f"{joint_name}: zero_encoder_visual deve essere null "
                    "nella fase pre-zero"
                )

        elif calibration_status == visual_zero_status:
            if not isinstance(zero_visual, int) or not 0 <= zero_visual <= 4095:
                errors.append(
                    f"{joint_name}: zero_encoder_visual invalido: "
                    f"{zero_visual!r}"
                )

        if joint.get("zero_encoder_final") is not None:
            errors.append(
                f"{joint_name}: zero_encoder_final deve essere null in questa fase"
            )

        if joint.get("first_stand_limit_rad") != {"min": None, "max": None}:
            errors.append(
                f"{joint_name}: first_stand_limit_rad non deve essere impostato"
            )

        if joint.get("safe_limit_rad") != {"min": None, "max": None}:
            errors.append(
                f"{joint_name}: safe_limit_rad non deve essere impostato"
            )

        if joint.get("validation_status") != "PASS_DIRECTION_TEST":
            errors.append(
                f"{joint_name}: validation_status inatteso: "
                f"{joint.get('validation_status')!r}"
            )

        servo_ids.append(joint.get("servo_id"))

    if len(set(servo_ids)) != 12:
        errors.append(f"servo_id non univoci: {servo_ids}")

    urdf_joints: dict[str, ET.Element] = {}

    if urdf_path.is_file():
        root = ET.parse(urdf_path).getroot()

        for node in root.findall("joint"):
            urdf_joints[node.attrib.get("name", "")] = node

        for joint_name, (_, _, group) in EXPECTED_JOINTS.items():
            node = urdf_joints.get(joint_name)

            if node is None:
                errors.append(f"{joint_name}: joint assente nell'URDF")
                continue

            if node.attrib.get("type") != "revolute":
                errors.append(
                    f"{joint_name}: tipo URDF atteso revolute, "
                    f"trovato {node.attrib.get('type')!r}"
                )

            limit = node.find("limit")
            if limit is None:
                errors.append(f"{joint_name}: limite URDF mancante")
            else:
                try:
                    lower = float(limit.attrib["lower"])
                    upper = float(limit.attrib["upper"])
                    if lower >= upper:
                        errors.append(
                            f"{joint_name}: limiti URDF invalidi "
                            f"[{lower}, {upper}]"
                        )
                except (KeyError, ValueError) as exc:
                    errors.append(f"{joint_name}: limite URDF invalido: {exc}")

            try:
                axis = parse_axis(node)
                if not same_axis(axis, EXPECTED_AXIS[group]):
                    errors.append(
                        f"{joint_name}: axis URDF atteso "
                        f"{EXPECTED_AXIS[group]}, trovato {axis}"
                    )
            except ValueError as exc:
                errors.append(f"{joint_name}: {exc}")

    print("=== MATDOG CALIBRATION CONTRACT ===")
    print(f"Status: {calibration_status}")
    print(f"Config: {config_path}")
    print(f"URDF:   {urdf_path}")
    print(f"SHA256: {actual_hash}")

    if urdf_path.is_file():
        print("\nJoint verificati:")
        for joint_name, (servo_id, direction, group) in EXPECTED_JOINTS.items():
            node = urdf_joints[joint_name]
            limit = node.find("limit")
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
            print(
                f"- {joint_name:25} M{servo_id:02d} dir={direction:+d} "
                f"axis={EXPECTED_AXIS[group]} "
                f"limit=[{lower:+.9f}, {upper:+.9f}]"
            )

    if errors:
        print("\nFAIL: contratto non valido.")
        for error in errors:
            print(f"- {error}")
        return 1

    print("\nPASS: YAML, mapping servo, direzioni, assi e URDF sono bloccati")
    print("      sullo stesso riferimento cinematico REV00.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
