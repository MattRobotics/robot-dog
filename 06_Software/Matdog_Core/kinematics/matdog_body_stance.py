#!/usr/bin/env python3
"""
MATDOG — audit offline della geometria body e della futura reference stance.

Questo modulo formalizza un vincolo progettuale intenzionale:
gli assi hip anteriori sono +20 mm più alti degli assi hip posteriori.

Non apre Station, non usa seriale e non invia comandi ai servo.
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

CONFIG_RELATIVE_PATH = Path(
    "06_Software/Matdog_Core/kinematics/"
    "MATDOG_BODY_STANCE_GEOMETRY.yaml"
)

IDENTITY_MATRIX3: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class BodyStanceGeometryError(RuntimeError):
    """Errore del contratto geometrico body / stance MATDOG."""


@dataclass(frozen=True)
class BodyStanceGeometry:
    front_legs: tuple[str, str]
    rear_legs: tuple[str, str]
    front_hip_axis_elevation_relative_rear_m: float
    visual_zero_front_contact_z_minus_rear_contact_z_m: float
    expected_visual_zero_contact_spread_m: float


@dataclass(frozen=True)
class VisualZeroFootContact:
    leg: str
    foot_origin_base_m: tuple[float, float, float]
    contact: FootGroundContact


@dataclass(frozen=True)
class BodyStanceGeometryAudit:
    front_hip_origin_z_m: tuple[float, float]
    rear_hip_origin_z_m: tuple[float, float]
    front_minus_rear_hip_elevation_m: float
    visual_zero_contacts: tuple[
        VisualZeroFootContact,
        VisualZeroFootContact,
        VisualZeroFootContact,
        VisualZeroFootContact,
    ]
    front_mean_contact_z_m: float
    rear_mean_contact_z_m: float
    front_minus_rear_contact_z_m: float
    contact_z_spread_m: float


def _vector2_strings(value: Any, field_name: str) -> tuple[str, str]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise BodyStanceGeometryError(
            f"{field_name}: attesi esattamente due elementi"
        )

    result = tuple(str(item) for item in value)

    if len(set(result)) != 2:
        raise BodyStanceGeometryError(
            f"{field_name}: gli elementi devono essere distinti"
        )

    return result  # type: ignore[return-value]


def _finite_float(value: Any, field_name: str) -> float:
    result = float(value)

    if not math.isfinite(result):
        raise BodyStanceGeometryError(
            f"{field_name}: valore non finito"
        )

    return result


def _parse_xyz(value: str, field_name: str) -> tuple[float, float, float]:
    tokens = value.split()

    if len(tokens) != 3:
        raise BodyStanceGeometryError(
            f"{field_name}: attesi tre valori, trovato {value!r}"
        )

    result = tuple(float(token) for token in tokens)

    if not all(math.isfinite(component) for component in result):
        raise BodyStanceGeometryError(
            f"{field_name}: valori non finiti"
        )

    return result  # type: ignore[return-value]


def _rotation_translation_from_transform(
    transform: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ],
) -> tuple[Matrix3, tuple[float, float, float]]:
    rotation: Matrix3 = (
        (transform[0][0], transform[0][1], transform[0][2]),
        (transform[1][0], transform[1][1], transform[1][2]),
        (transform[2][0], transform[2][1], transform[2][2]),
    )

    translation = (
        transform[0][3],
        transform[1][3],
        transform[2][3],
    )

    return rotation, translation


def load_body_stance_geometry(
    repo_root: Path = REPO_ROOT,
) -> BodyStanceGeometry:
    config_path = repo_root / CONFIG_RELATIVE_PATH

    if not config_path.is_file():
        raise FileNotFoundError(
            f"Configurazione body stance assente: {config_path}"
        )

    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise BodyStanceGeometryError(
            "Configurazione YAML body stance non valida"
        )

    if data.get("kind") != "MATDOG_BODY_STANCE_GEOMETRY":
        raise BodyStanceGeometryError(
            "kind body stance non riconosciuto"
        )

    if data.get("status") != "DESIGN_INTENT_CONFIRMED":
        raise BodyStanceGeometryError(
            "status body stance non riconosciuto"
        )

    canonical = data.get("canonical_urdf")

    if not isinstance(canonical, dict):
        raise BodyStanceGeometryError(
            "canonical_urdf mancante"
        )

    if canonical.get("path") != str(CANONICAL_URDF_RELATIVE_PATH):
        raise BodyStanceGeometryError(
            "canonical_urdf.path incoerente con il contratto URDF"
        )

    if canonical.get("sha256") != CANONICAL_URDF_SHA256:
        raise BodyStanceGeometryError(
            "canonical_urdf.sha256 incoerente con il contratto URDF"
        )

    geometry = data.get("front_rear_geometry")

    if not isinstance(geometry, dict):
        raise BodyStanceGeometryError(
            "front_rear_geometry mancante"
        )

    result = BodyStanceGeometry(
        front_legs=_vector2_strings(
            geometry.get("front_legs"),
            "front_legs",
        ),
        rear_legs=_vector2_strings(
            geometry.get("rear_legs"),
            "rear_legs",
        ),
        front_hip_axis_elevation_relative_rear_m=_finite_float(
            geometry.get(
                "front_hip_axis_elevation_relative_rear_m"
            ),
            "front_hip_axis_elevation_relative_rear_m",
        ),
        visual_zero_front_contact_z_minus_rear_contact_z_m=(
            _finite_float(
                geometry.get(
                    "visual_zero_front_contact_z_minus_rear_contact_z_m"
                ),
                "visual_zero_front_contact_z_minus_rear_contact_z_m",
            )
        ),
        expected_visual_zero_contact_spread_m=_finite_float(
            geometry.get(
                "expected_visual_zero_contact_spread_m"
            ),
            "expected_visual_zero_contact_spread_m",
        ),
    )

    expected_front = {"lf", "rf"}
    expected_rear = {"rh", "lh"}

    if set(result.front_legs) != expected_front:
        raise BodyStanceGeometryError(
            "front_legs deve contenere esattamente lf e rf"
        )

    if set(result.rear_legs) != expected_rear:
        raise BodyStanceGeometryError(
            "rear_legs deve contenere esattamente rh e lh"
        )

    if set(result.front_legs) & set(result.rear_legs):
        raise BodyStanceGeometryError(
            "front_legs e rear_legs non devono sovrapporsi"
        )

    for field_name, value in (
        (
            "front_hip_axis_elevation_relative_rear_m",
            result.front_hip_axis_elevation_relative_rear_m,
        ),
        (
            "visual_zero_front_contact_z_minus_rear_contact_z_m",
            result.visual_zero_front_contact_z_minus_rear_contact_z_m,
        ),
        (
            "expected_visual_zero_contact_spread_m",
            result.expected_visual_zero_contact_spread_m,
        ),
    ):
        if value <= 0.0:
            raise BodyStanceGeometryError(
                f"{field_name}: deve essere > 0"
            )

    return result


def _hip_origin_z(
    urdf_root: ET.Element,
    leg: str,
) -> float:
    joint_name = f"{leg}_hip_joint"
    joint = urdf_root.find(f"./joint[@name='{joint_name}']")

    if joint is None:
        raise BodyStanceGeometryError(
            f"URDF: joint assente {joint_name}"
        )

    origin = joint.find("origin")

    if origin is None:
        raise BodyStanceGeometryError(
            f"URDF: origin assente {joint_name}"
        )

    xyz = _parse_xyz(
        origin.attrib.get("xyz", "0 0 0"),
        f"{joint_name}.origin.xyz",
    )

    return xyz[2]


def audit_body_stance_geometry(
    repo_root: Path = REPO_ROOT,
    tolerance_m: float = 1e-12,
) -> BodyStanceGeometryAudit:
    if tolerance_m <= 0.0:
        raise ValueError("tolerance_m deve essere > 0")

    geometry = load_body_stance_geometry(repo_root)
    urdf_path = canonical_urdf_path(repo_root)

    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"URDF canonico assente: {urdf_path}"
        )

    actual_sha256 = sha256_file(urdf_path)

    if actual_sha256 != CANONICAL_URDF_SHA256:
        raise BodyStanceGeometryError(
            "Integrità URDF fallita: "
            f"sha256={actual_sha256}, "
            f"atteso={CANONICAL_URDF_SHA256}"
        )

    root = ET.parse(urdf_path).getroot()

    front_hip_z = tuple(
        _hip_origin_z(root, leg)
        for leg in geometry.front_legs
    )
    rear_hip_z = tuple(
        _hip_origin_z(root, leg)
        for leg in geometry.rear_legs
    )

    front_hip_mean = sum(front_hip_z) / len(front_hip_z)
    rear_hip_mean = sum(rear_hip_z) / len(rear_hip_z)

    front_minus_rear_hip = front_hip_mean - rear_hip_mean

    if abs(
        front_minus_rear_hip
        - geometry.front_hip_axis_elevation_relative_rear_m
    ) > tolerance_m:
        raise BodyStanceGeometryError(
            "Elevazione hip front/rear incoerente con configurazione: "
            f"URDF={front_minus_rear_hip:.12f} m, "
            "config="
            f"{geometry.front_hip_axis_elevation_relative_rear_m:.12f} m"
        )

    contact_model = load_foot_contact_model(repo_root)
    contacts: list[VisualZeroFootContact] = []

    for leg in (
        geometry.front_legs[0],
        geometry.front_legs[1],
        geometry.rear_legs[0],
        geometry.rear_legs[1],
    ):
        result = forward_kinematics(
            urdf_path=urdf_path,
            root_link="base_link",
            tip_link=f"{leg}_foot_link",
            joint_positions_rad={
                f"{leg}_hip_joint": 0.0,
                f"{leg}_upper_leg_joint": 0.0,
                f"{leg}_lower_leg_joint": 0.0,
            },
            enforce_limits=True,
        )

        rotation, translation = _rotation_translation_from_transform(
            result.tip_transform
        )

        contact = contact_from_foot_pose(
            model=contact_model,
            rotation_world_from_foot=rotation,
            translation_world_from_foot_m=translation,
        )

        if contact.support_mode != "NOMINAL_STRIP_CONTACT":
            raise BodyStanceGeometryError(
                f"{leg}: visual-zero support mode inatteso: "
                f"{contact.support_mode}"
            )

        contacts.append(
            VisualZeroFootContact(
                leg=leg,
                foot_origin_base_m=translation,
                contact=contact,
            )
        )

    front_contact_z = tuple(
        item.contact.cross_section_contact_center_world_m[2]
        for item in contacts[:2]
    )
    rear_contact_z = tuple(
        item.contact.cross_section_contact_center_world_m[2]
        for item in contacts[2:]
    )

    front_mean_z = sum(front_contact_z) / len(front_contact_z)
    rear_mean_z = sum(rear_contact_z) / len(rear_contact_z)

    front_minus_rear_contact = front_mean_z - rear_mean_z

    all_contact_z = front_contact_z + rear_contact_z
    contact_spread = max(all_contact_z) - min(all_contact_z)

    if abs(
        front_minus_rear_contact
        - geometry.visual_zero_front_contact_z_minus_rear_contact_z_m
    ) > tolerance_m:
        raise BodyStanceGeometryError(
            "Delta Z visual-zero front/rear incoerente: "
            f"URDF/FK={front_minus_rear_contact:.12f} m, "
            "config="
            f"{geometry.visual_zero_front_contact_z_minus_rear_contact_z_m:.12f} m"
        )

    if abs(
        contact_spread
        - geometry.expected_visual_zero_contact_spread_m
    ) > tolerance_m:
        raise BodyStanceGeometryError(
            "Spread Z visual-zero incoerente: "
            f"URDF/FK={contact_spread:.12f} m, "
            "config="
            f"{geometry.expected_visual_zero_contact_spread_m:.12f} m"
        )

    return BodyStanceGeometryAudit(
        front_hip_origin_z_m=front_hip_z,
        rear_hip_origin_z_m=rear_hip_z,
        front_minus_rear_hip_elevation_m=front_minus_rear_hip,
        visual_zero_contacts=tuple(contacts),  # type: ignore[arg-type]
        front_mean_contact_z_m=front_mean_z,
        rear_mean_contact_z_m=rear_mean_z,
        front_minus_rear_contact_z_m=front_minus_rear_contact,
        contact_z_spread_m=contact_spread,
    )


def main() -> None:
    audit = audit_body_stance_geometry(REPO_ROOT)

    print("=== MATDOG BODY STANCE GEOMETRY AUDIT — OFFLINE ===")
    print(
        "front_hip_origin_z_m: "
        + ", ".join(
            f"{value:+.9f}"
            for value in audit.front_hip_origin_z_m
        )
    )
    print(
        "rear_hip_origin_z_m: "
        + ", ".join(
            f"{value:+.9f}"
            for value in audit.rear_hip_origin_z_m
        )
    )
    print(
        "front_minus_rear_hip_elevation_mm: "
        f"{audit.front_minus_rear_hip_elevation_m * 1000.0:+.6f}"
    )
    print("")

    for item in audit.visual_zero_contacts:
        point = item.contact.cross_section_contact_center_world_m
        print(
            f"{item.leg.upper()} contact_reference_base_m: "
            f"({point[0]:+.9f}, {point[1]:+.9f}, {point[2]:+.9f})"
        )

    print("")
    print(
        "front_minus_rear_contact_z_mm: "
        f"{audit.front_minus_rear_contact_z_m * 1000.0:+.6f}"
    )
    print(
        "visual_zero_contact_spread_mm: "
        f"{audit.contact_z_spread_m * 1000.0:.6f}"
    )
    print("VISUAL-ZERO NON-COPLANARITY: EXPECTED_INTENTIONAL")
    print("Offline only: no Station, serial or motor command was used.")


if __name__ == "__main__":
    main()
