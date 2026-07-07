#!/usr/bin/env python3
"""
MATDOG — generic offline FK → foot-contact bridge for all four legs.

Pipeline:
    leg ID + three URDF joint angles
    → canonical URDF FK
    → world_from_foot pose
    → local foot-cylinder contact geometry.

No Station, serial port or motor command is used.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

from matdog_foot_contact import (
    FootGroundContact,
    Matrix3,
    Vector3,
    contact_from_foot_pose,
    load_foot_contact_model,
)
from matdog_leg_contact import validate_canonical_foot_joint_bindings
from matdog_urdf_fk import (
    canonical_urdf_path,
    forward_kinematics,
    load_urdf_joints,
)


REPO_ROOT = Path(__file__).resolve().parents[3]

LEG_IDS = ("lf", "rf", "rh", "lh")

IDENTITY_MATRIX3: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class QuadrupedLegContactError(RuntimeError):
    """Errore del bridge quadrupede FK → contatto."""


@dataclass(frozen=True)
class LegKinematicContract:
    leg_id: str
    joint_names: tuple[str, str, str]
    foot_joint_name: str
    foot_link_name: str
    expected_chain_joint_names: tuple[str, str, str, str]
    joint_limits_rad: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]


@dataclass(frozen=True)
class QuadrupedLegContactResult:
    leg_id: str
    joint_positions_rad: dict[str, float]
    contract: LegKinematicContract
    base_from_foot_transform: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ]
    world_from_foot_rotation: Matrix3
    world_from_foot_translation_m: Vector3
    contact: FootGroundContact


def _normalize_leg_id(leg_id: str) -> str:
    normalized = str(leg_id).strip().lower()

    if normalized not in LEG_IDS:
        raise QuadrupedLegContactError(
            "leg_id non valido: "
            f"{leg_id!r}; attesi {', '.join(LEG_IDS)}"
        )

    return normalized


def leg_joint_names(leg_id: str) -> tuple[str, str, str]:
    leg = _normalize_leg_id(leg_id)

    return (
        f"{leg}_hip_joint",
        f"{leg}_upper_leg_joint",
        f"{leg}_lower_leg_joint",
    )


def _finite_vector3(value: Any, field_name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise QuadrupedLegContactError(
            f"{field_name}: attesi esattamente tre valori"
        )

    result = tuple(float(component) for component in value)

    if not all(math.isfinite(component) for component in result):
        raise QuadrupedLegContactError(
            f"{field_name}: valori non finiti"
        )

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
            sum(
                left[row][index] * right[index][column]
                for index in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def _validate_rotation(rotation: Matrix3, field_name: str) -> None:
    for index, row in enumerate(rotation):
        if abs(_norm(row) - 1.0) > 1e-6:
            raise QuadrupedLegContactError(
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
                raise QuadrupedLegContactError(
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
        raise QuadrupedLegContactError(
            f"{field_name}: determinante atteso +1, "
            f"trovato {determinant:.9f}"
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
        raise QuadrupedLegContactError(
            "tip_transform: attesa matrice omogenea 4x4"
        )

    if any(
        abs(actual - expected) > 1e-9
        for actual, expected in zip(
            transform[3],
            (0.0, 0.0, 0.0, 1.0),
        )
    ):
        raise QuadrupedLegContactError(
            "tip_transform: ultima riga non omogenea"
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


def load_leg_kinematic_contract(
    leg_id: str,
    repo_root: Path = REPO_ROOT,
) -> LegKinematicContract:
    """
    Read selected-leg topology and limits from canonical URDF.

    The audited fixed foot-joint binding contract is validated before use.
    """
    leg = _normalize_leg_id(leg_id)

    validate_canonical_foot_joint_bindings(repo_root)

    urdf_path = canonical_urdf_path(repo_root)
    joint_map = load_urdf_joints(urdf_path)

    joint_names = leg_joint_names(leg)
    foot_joint_name = f"{leg}_foot_joint"
    foot_link_name = f"{leg}_foot_link"

    expected_links = (
        ("base_link", f"{leg}_hip_link"),
        (f"{leg}_hip_link", f"{leg}_upper_leg_link"),
        (f"{leg}_upper_leg_link", f"{leg}_lower_leg_link"),
    )

    limits: list[tuple[float, float]] = []

    for joint_name, (expected_parent, expected_child) in zip(
        joint_names,
        expected_links,
    ):
        joint = joint_map.get(joint_name)

        if joint is None:
            raise QuadrupedLegContactError(
                f"URDF: joint assente {joint_name}"
            )

        if joint.joint_type != "revolute":
            raise QuadrupedLegContactError(
                f"{joint_name}: atteso revolute, "
                f"trovato {joint.joint_type!r}"
            )

        if joint.parent_link != expected_parent:
            raise QuadrupedLegContactError(
                f"{joint_name}: parent inatteso "
                f"{joint.parent_link!r}"
            )

        if joint.child_link != expected_child:
            raise QuadrupedLegContactError(
                f"{joint_name}: child inatteso "
                f"{joint.child_link!r}"
            )

        lower = joint.lower_limit_rad
        upper = joint.upper_limit_rad

        if lower is None or upper is None:
            raise QuadrupedLegContactError(
                f"{joint_name}: limiti URDF mancanti"
            )

        lower_value = float(lower)
        upper_value = float(upper)

        if not (
            math.isfinite(lower_value)
            and math.isfinite(upper_value)
            and lower_value < upper_value
        ):
            raise QuadrupedLegContactError(
                f"{joint_name}: limiti URDF non validi"
            )

        limits.append((lower_value, upper_value))

    foot_joint = joint_map.get(foot_joint_name)

    if foot_joint is None:
        raise QuadrupedLegContactError(
            f"URDF: joint assente {foot_joint_name}"
        )

    if foot_joint.joint_type != "fixed":
        raise QuadrupedLegContactError(
            f"{foot_joint_name}: atteso fixed, "
            f"trovato {foot_joint.joint_type!r}"
        )

    if foot_joint.parent_link != f"{leg}_lower_leg_link":
        raise QuadrupedLegContactError(
            f"{foot_joint_name}: parent inatteso "
            f"{foot_joint.parent_link!r}"
        )

    if foot_joint.child_link != foot_link_name:
        raise QuadrupedLegContactError(
            f"{foot_joint_name}: child inatteso "
            f"{foot_joint.child_link!r}"
        )

    return LegKinematicContract(
        leg_id=leg,
        joint_names=joint_names,
        foot_joint_name=foot_joint_name,
        foot_link_name=foot_link_name,
        expected_chain_joint_names=(
            joint_names[0],
            joint_names[1],
            joint_names[2],
            foot_joint_name,
        ),
        joint_limits_rad=tuple(limits),  # type: ignore[arg-type]
    )


def _validate_joint_positions(
    contract: LegKinematicContract,
    joint_positions_rad: dict[str, float],
) -> dict[str, float]:
    if not isinstance(joint_positions_rad, dict):
        raise QuadrupedLegContactError(
            "joint_positions_rad deve essere un dict"
        )

    expected = set(contract.joint_names)
    actual = set(joint_positions_rad)

    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)

    if missing:
        raise QuadrupedLegContactError(
            "target joint incompleto: manca "
            + ", ".join(missing)
        )

    if unknown:
        raise QuadrupedLegContactError(
            "target joint contiene nomi non previsti: "
            + ", ".join(unknown)
        )

    validated: dict[str, float] = {}

    for joint_name, (lower, upper) in zip(
        contract.joint_names,
        contract.joint_limits_rad,
    ):
        value = float(joint_positions_rad[joint_name])

        if not math.isfinite(value):
            raise QuadrupedLegContactError(
                f"{joint_name}: valore non finito"
            )

        if not lower <= value <= upper:
            raise QuadrupedLegContactError(
                f"{joint_name}: {value:.9f} rad fuori limiti "
                f"[{lower:.9f}, {upper:.9f}]"
            )

        validated[joint_name] = value

    return validated


def leg_foot_contact_from_joint_angles(
    leg_id: str,
    joint_positions_rad: dict[str, float],
    repo_root: Path = REPO_ROOT,
    world_from_base_rotation: Matrix3 = IDENTITY_MATRIX3,
    world_from_base_translation_m: Vector3 = (0.0, 0.0, 0.0),
    ground_normal_world_unit: Vector3 = (0.0, 0.0, 1.0),
) -> QuadrupedLegContactResult:
    """
    Evaluate one leg through canonical URDF FK and common contact geometry.
    """
    contract = load_leg_kinematic_contract(leg_id, repo_root)

    _validate_rotation(
        world_from_base_rotation,
        "world_from_base_rotation",
    )

    world_translation = _finite_vector3(
        world_from_base_translation_m,
        "world_from_base_translation_m",
    )

    joints = _validate_joint_positions(
        contract,
        joint_positions_rad,
    )

    urdf_path = canonical_urdf_path(repo_root)

    fk_result = forward_kinematics(
        urdf_path=urdf_path,
        root_link="base_link",
        tip_link=contract.foot_link_name,
        joint_positions_rad=joints,
        enforce_limits=True,
    )

    if fk_result.chain_joint_names != contract.expected_chain_joint_names:
        raise QuadrupedLegContactError(
            f"{contract.leg_id}: chain FK inattesa "
            f"{fk_result.chain_joint_names!r}"
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
        world_translation,
    )

    contact_model = load_foot_contact_model(repo_root)

    contact = contact_from_foot_pose(
        model=contact_model,
        rotation_world_from_foot=world_from_foot_rotation,
        translation_world_from_foot_m=world_from_foot_translation,
        ground_normal_world_unit=ground_normal_world_unit,
    )

    return QuadrupedLegContactResult(
        leg_id=contract.leg_id,
        joint_positions_rad=joints,
        contract=contract,
        base_from_foot_transform=fk_result.tip_transform,
        world_from_foot_rotation=world_from_foot_rotation,
        world_from_foot_translation_m=world_from_foot_translation,
        contact=contact,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG generic leg FK → contact geometry, offline only."
        )
    )
    parser.add_argument(
        "--leg",
        required=True,
        choices=LEG_IDS,
    )
    parser.add_argument(
        "--q",
        nargs=3,
        type=float,
        metavar=("HIP", "UPPER", "LOWER"),
        default=(0.0, 0.0, 0.0),
    )
    args = parser.parse_args()

    names = leg_joint_names(args.leg)

    result = leg_foot_contact_from_joint_angles(
        leg_id=args.leg,
        joint_positions_rad=dict(zip(names, args.q)),
        repo_root=REPO_ROOT,
    )

    point = result.contact.cross_section_contact_center_world_m
    lowest = result.contact.lowest_core_point_world_m

    print("=== MATDOG GENERIC LEG FK → CONTACT — OFFLINE ===")
    print(f"leg: {result.leg_id.upper()}")
    print(f"chain: {result.contract.expected_chain_joint_names}")
    print(
        "foot_origin_base_m: "
        f"({result.world_from_foot_translation_m[0]:+.9f}, "
        f"{result.world_from_foot_translation_m[1]:+.9f}, "
        f"{result.world_from_foot_translation_m[2]:+.9f})"
    )
    print(
        "contact_reference_base_m: "
        f"({point[0]:+.9f}, {point[1]:+.9f}, {point[2]:+.9f})"
    )
    print(
        "lowest_core_base_m: "
        f"({lowest[0]:+.9f}, {lowest[1]:+.9f}, {lowest[2]:+.9f})"
    )
    print(f"support_mode: {result.contact.support_mode}")
    print("Offline only: no Station, serial or motor command was used.")


if __name__ == "__main__":
    main()
