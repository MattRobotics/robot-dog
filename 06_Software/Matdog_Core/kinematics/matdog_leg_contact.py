#!/usr/bin/env python3
"""
MATDOG — collegamento offline LF FK → geometria di contatto del piede.

Pipeline:
    q LF in radianti URDF
    → FK canonica base_link → lf_foot_link
    → posa world → foot_link
    → cilindro rigido del gommino
    → geometria di contatto con il terreno.

Questo modulo:
- non apre Station;
- non apre porte seriali;
- non invia comandi ai servo;
- usa esclusivamente URDF canonico, configurazione contatto verificata e FK.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import yaml

from matdog_foot_contact import (
    FootGroundContact,
    Matrix3,
    Vector3,
    contact_from_foot_pose,
    load_foot_contact_model,
)
from matdog_urdf_fk import (
    CANONICAL_URDF_RELATIVE_PATH,
    CANONICAL_URDF_SHA256,
    canonical_urdf_path,
    forward_kinematics,
    sha256_file,
)


REPO_ROOT = Path(__file__).resolve().parents[3]

CONTACT_CONFIG_RELATIVE_PATH = Path(
    "06_Software/Matdog_Core/kinematics/"
    "MATDOG_FOOT_CONTACT_GEOMETRY.yaml"
)

LF_JOINT_NAMES = (
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
)

IDENTITY_MATRIX3: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class LegContactError(RuntimeError):
    """Errore del bridge MATDOG FK → contatto."""


@dataclass(frozen=True)
class LfLegContactResult:
    joint_positions_rad: dict[str, float]
    base_from_foot_transform: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ]
    world_from_foot_rotation: Matrix3
    world_from_foot_translation_m: Vector3
    contact: FootGroundContact


def _vector3(value: Any, field_name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise LegContactError(
            f"{field_name}: attesi esattamente tre valori"
        )

    result = tuple(float(component) for component in value)

    if not all(math.isfinite(component) for component in result):
        raise LegContactError(f"{field_name}: valori non finiti")

    return result  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2],
    )


def _mat3_vec(rotation: Matrix3, vector: Vector3) -> Vector3:
    return (
        _dot(rotation[0], vector),
        _dot(rotation[1], vector),
        _dot(rotation[2], vector),
    )


def _mat3_mul(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def _validate_rotation(rotation: Matrix3, field_name: str) -> None:
    for index, row in enumerate(rotation):
        if abs(_norm(row) - 1.0) > 1e-6:
            raise LegContactError(
                f"{field_name}: riga {index} non normalizzata"
            )

    for left_index in range(3):
        for right_index in range(left_index + 1, 3):
            if abs(
                _dot(
                    rotation[left_index],
                    rotation[right_index],
                )
            ) > 1e-6:
                raise LegContactError(
                    f"{field_name}: righe non ortogonali"
                )

    determinant = (
        rotation[0][0] * (
            rotation[1][1] * rotation[2][2]
            - rotation[1][2] * rotation[2][1]
        )
        - rotation[0][1] * (
            rotation[1][0] * rotation[2][2]
            - rotation[1][2] * rotation[2][0]
        )
        + rotation[0][2] * (
            rotation[1][0] * rotation[2][1]
            - rotation[1][1] * rotation[2][0]
        )
    )

    if abs(determinant - 1.0) > 1e-6:
        raise LegContactError(
            f"{field_name}: determinante atteso +1, trovato {determinant:.9f}"
        )


def _rotation_translation_from_matrix4(
    transform: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ],
) -> tuple[Matrix3, Vector3]:
    if len(transform) != 4 or any(len(row) != 4 for row in transform):
        raise LegContactError(
            "tip_transform: attesa matrice omogenea 4x4"
        )

    last_row = transform[3]

    if any(
        abs(actual - expected) > 1e-9
        for actual, expected in zip(last_row, (0.0, 0.0, 0.0, 1.0))
    ):
        raise LegContactError(
            f"tip_transform: ultima riga non omogenea: {last_row!r}"
        )

    rotation: Matrix3 = (
        (
            float(transform[0][0]),
            float(transform[0][1]),
            float(transform[0][2]),
        ),
        (
            float(transform[1][0]),
            float(transform[1][1]),
            float(transform[1][2]),
        ),
        (
            float(transform[2][0]),
            float(transform[2][1]),
            float(transform[2][2]),
        ),
    )

    translation: Vector3 = (
        float(transform[0][3]),
        float(transform[1][3]),
        float(transform[2][3]),
    )

    _validate_rotation(rotation, "tip_transform rotation")

    return rotation, translation


def _validate_lf_joint_positions(
    joint_positions_rad: dict[str, float],
) -> dict[str, float]:
    values: dict[str, float] = {}

    for joint_name in LF_JOINT_NAMES:
        if joint_name not in joint_positions_rad:
            raise LegContactError(
                f"Target LF incompleto: manca {joint_name}"
            )

        value = float(joint_positions_rad[joint_name])

        if not math.isfinite(value):
            raise LegContactError(
                f"{joint_name}: valore non finito"
            )

        values[joint_name] = value

    unknown = sorted(
        set(joint_positions_rad) - set(LF_JOINT_NAMES)
    )

    if unknown:
        raise LegContactError(
            "Target LF contiene joint non previsti: "
            + ", ".join(unknown)
        )

    return values


def _parse_xyz(value: str, field_name: str) -> Vector3:
    tokens = value.split()

    if len(tokens) != 3:
        raise LegContactError(
            f"{field_name}: attesi tre valori, trovato {value!r}"
        )

    return _vector3(
        tuple(float(token) for token in tokens),
        field_name,
    )


def validate_canonical_foot_joint_bindings(
    repo_root: Path = REPO_ROOT,
) -> None:
    """
    Verifica che YAML di contatto e URDF canonico descrivano lo stesso
    binding fixed lower_leg_link → foot_link per tutte le quattro zampe.
    """
    config_path = repo_root / CONTACT_CONFIG_RELATIVE_PATH

    if not config_path.is_file():
        raise FileNotFoundError(
            f"Configurazione contatto assente: {config_path}"
        )

    config = yaml.safe_load(
        config_path.read_text(encoding="utf-8")
    )

    if not isinstance(config, dict):
        raise LegContactError("Configurazione contatto YAML non valida")

    canonical = config.get("canonical_urdf")

    if not isinstance(canonical, dict):
        raise LegContactError(
            "canonical_urdf mancante nella configurazione contatto"
        )

    expected_path = str(CANONICAL_URDF_RELATIVE_PATH)

    if canonical.get("path") != expected_path:
        raise LegContactError(
            "canonical_urdf.path incoerente: "
            f"atteso {expected_path!r}, trovato {canonical.get('path')!r}"
        )

    if canonical.get("sha256") != CANONICAL_URDF_SHA256:
        raise LegContactError(
            "canonical_urdf.sha256 incoerente con il contratto URDF"
        )

    urdf_path = canonical_urdf_path(repo_root)

    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"URDF canonico assente: {urdf_path}"
        )

    actual_sha256 = sha256_file(urdf_path)

    if actual_sha256 != CANONICAL_URDF_SHA256:
        raise LegContactError(
            "Integrità URDF fallita: "
            f"sha256={actual_sha256}, atteso={CANONICAL_URDF_SHA256}"
        )

    bindings = config.get("foot_joint_bindings")

    if not isinstance(bindings, dict):
        raise LegContactError(
            "foot_joint_bindings mancante nella configurazione contatto"
        )

    expected_legs = {"lf", "rf", "rh", "lh"}

    if set(bindings) != expected_legs:
        raise LegContactError(
            "foot_joint_bindings deve contenere esattamente: "
            + ", ".join(sorted(expected_legs))
        )

    root = ET.parse(urdf_path).getroot()

    for leg in sorted(expected_legs):
        binding = bindings[leg]

        if not isinstance(binding, dict):
            raise LegContactError(
                f"foot_joint_bindings.{leg}: sezione non valida"
            )

        joint_name = f"{leg}_foot_joint"
        joint = root.find(f"./joint[@name='{joint_name}']")

        if joint is None:
            raise LegContactError(
                f"URDF: joint assente {joint_name}"
            )

        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")

        if parent is None or child is None or origin is None:
            raise LegContactError(
                f"URDF: parent/child/origin mancanti in {joint_name}"
            )

        expected_parent = binding.get("parent_link")
        expected_child = binding.get("child_link")

        if parent.attrib.get("link") != expected_parent:
            raise LegContactError(
                f"{joint_name}: parent incoerente con YAML"
            )

        if child.attrib.get("link") != expected_child:
            raise LegContactError(
                f"{joint_name}: child incoerente con YAML"
            )

        actual_xyz = _parse_xyz(
            origin.attrib.get("xyz", "0 0 0"),
            f"{joint_name}.origin_xyz",
        )
        actual_rpy = _parse_xyz(
            origin.attrib.get("rpy", "0 0 0"),
            f"{joint_name}.origin_rpy",
        )

        expected_xyz = _vector3(
            binding.get("origin_xyz_m"),
            f"foot_joint_bindings.{leg}.origin_xyz_m",
        )
        expected_rpy = _vector3(
            binding.get("origin_rpy_rad"),
            f"foot_joint_bindings.{leg}.origin_rpy_rad",
        )

        for axis, actual, expected in zip(
            ("x", "y", "z"),
            actual_xyz,
            expected_xyz,
        ):
            if abs(actual - expected) > 1e-12:
                raise LegContactError(
                    f"{joint_name}: origin {axis} incoerente "
                    f"(URDF={actual}, YAML={expected})"
                )

        for axis, actual, expected in zip(
            ("r", "p", "y"),
            actual_rpy,
            expected_rpy,
        ):
            if abs(actual - expected) > 1e-12:
                raise LegContactError(
                    f"{joint_name}: origin {axis} rpy incoerente "
                    f"(URDF={actual}, YAML={expected})"
                )


def lf_foot_contact_from_joint_angles(
    joint_positions_rad: dict[str, float],
    repo_root: Path = REPO_ROOT,
    world_from_base_rotation: Matrix3 = IDENTITY_MATRIX3,
    world_from_base_translation_m: Vector3 = (0.0, 0.0, 0.0),
    ground_normal_world_unit: Vector3 = (0.0, 0.0, 1.0),
) -> LfLegContactResult:
    """
    Calcola la geometria di contatto LF da tre angoli URDF.

    Per default world coincide con base_link. Nei planner futuri il chiamante
    fornirà la posa reale world ← base_link.
    """
    _validate_rotation(
        world_from_base_rotation,
        "world_from_base_rotation",
    )

    world_from_base_translation = _vector3(
        world_from_base_translation_m,
        "world_from_base_translation_m",
    )

    joints = _validate_lf_joint_positions(joint_positions_rad)

    validate_canonical_foot_joint_bindings(repo_root)

    urdf_path = canonical_urdf_path(repo_root)

    fk_result = forward_kinematics(
        urdf_path=urdf_path,
        root_link="base_link",
        tip_link="lf_foot_link",
        joint_positions_rad=joints,
        enforce_limits=True,
    )

    base_from_foot_rotation, base_from_foot_translation = (
        _rotation_translation_from_matrix4(
            fk_result.tip_transform
        )
    )

    world_from_foot_rotation = _mat3_mul(
        world_from_base_rotation,
        base_from_foot_rotation,
    )

    world_from_foot_translation = _add(
        _mat3_vec(
            world_from_base_rotation,
            base_from_foot_translation,
        ),
        world_from_base_translation,
    )

    contact_model = load_foot_contact_model(repo_root)

    contact = contact_from_foot_pose(
        model=contact_model,
        rotation_world_from_foot=world_from_foot_rotation,
        translation_world_from_foot_m=world_from_foot_translation,
        ground_normal_world_unit=ground_normal_world_unit,
    )

    return LfLegContactResult(
        joint_positions_rad=joints,
        base_from_foot_transform=fk_result.tip_transform,
        world_from_foot_rotation=world_from_foot_rotation,
        world_from_foot_translation_m=world_from_foot_translation,
        contact=contact,
    )
