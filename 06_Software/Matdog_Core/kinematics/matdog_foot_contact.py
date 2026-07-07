#!/usr/bin/env python3
"""
MATDOG — modello analitico offline del contatto rigido del piede.

Il frame foot_link resta rigidamente collegato al lower_leg_link tramite il
foot_joint URDF. Il punto di contatto può invece muoversi sul profilo
cilindrico del gommino quando il piede cambia orientamento rispetto al terreno.

Questo modulo:
- non apre Station;
- non apre porte seriali;
- non invia comandi;
- lavora soltanto con geometria CAD/URDF e trasformazioni rigide.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]

CONFIG_RELATIVE_PATH = Path(
    "06_Software/Matdog_Core/kinematics/"
    "MATDOG_FOOT_CONTACT_GEOMETRY.yaml"
)

Vector3 = tuple[float, float, float]
Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


class FootContactError(RuntimeError):
    """Errore base del modello di contatto MATDOG."""


class FootContactDegeneracyError(FootContactError):
    """Posa cilindrica non idonea a un contatto utile con il terreno."""


@dataclass(frozen=True)
class FootContactModel:
    frame_name: str
    nominal_contact_point_m: Vector3
    cylinder_center_in_foot_link_m: Vector3
    cylinder_axis_in_foot_link_unit: Vector3
    cylinder_radius_m: float
    total_tread_width_m: float
    end_fillet_radius_m: float
    central_rigid_support_width_m: float
    nominal_strip_max_axis_tilt_from_ground_rad: float


@dataclass(frozen=True)
class FootGroundContact:
    support_mode: str
    cylinder_center_world_m: Vector3
    cylinder_axis_world_unit: Vector3
    radial_down_world_unit: Vector3
    cross_section_contact_center_world_m: Vector3
    support_strip_end_a_world_m: Vector3
    support_strip_end_b_world_m: Vector3
    lowest_core_point_world_m: Vector3
    axis_tilt_from_ground_rad: float


def _vector3(value: Any, field_name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise FootContactError(
            f"{field_name}: attesi esattamente tre valori"
        )

    result = tuple(float(component) for component in value)

    if not all(math.isfinite(component) for component in result):
        raise FootContactError(f"{field_name}: valori non finiti")

    return result  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector: Vector3, field_name: str) -> Vector3:
    length = _norm(vector)

    if length <= 1e-12:
        raise FootContactError(f"{field_name}: vettore nullo")

    return (
        vector[0] / length,
        vector[1] / length,
        vector[2] / length,
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2],
    )


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def _scale(vector: Vector3, factor: float) -> Vector3:
    return (
        vector[0] * factor,
        vector[1] * factor,
        vector[2] * factor,
    )


def _mat_vec(rotation: Matrix3, vector: Vector3) -> Vector3:
    return (
        _dot(rotation[0], vector),
        _dot(rotation[1], vector),
        _dot(rotation[2], vector),
    )


def _validate_rotation(rotation: Matrix3) -> None:
    rows = rotation

    for index, row in enumerate(rows):
        if abs(_norm(row) - 1.0) > 1e-6:
            raise FootContactError(
                f"rotation row {index}: matrice non ortonormale"
            )

    for left_index in range(3):
        for right_index in range(left_index + 1, 3):
            if abs(_dot(rows[left_index], rows[right_index])) > 1e-6:
                raise FootContactError(
                    "rotation: righe non ortogonali"
                )

    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )

    if abs(determinant - 1.0) > 1e-6:
        raise FootContactError(
            f"rotation: determinante atteso +1, trovato {determinant:.9f}"
        )


def load_foot_contact_model(
    repo_root: Path = REPO_ROOT,
) -> FootContactModel:
    path = repo_root / CONFIG_RELATIVE_PATH

    if not path.is_file():
        raise FileNotFoundError(f"Configurazione piede assente: {path}")

    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise FootContactError("Configurazione YAML non valida")

    if data.get("kind") != "MATDOG_FOOT_CONTACT_GEOMETRY":
        raise FootContactError("kind configurazione non riconosciuto")

    if data.get("status") != "CAD_AND_STL_AUDITED":
        raise FootContactError(
            "La geometria piede non è marcata CAD_AND_STL_AUDITED"
        )

    frame = data["frame"]
    cylinder = data["rigid_cylinder"]
    support = data["support_surface"]

    model = FootContactModel(
        frame_name=str(frame["name"]),
        nominal_contact_point_m=_vector3(
            frame["nominal_contact_point_m"],
            "nominal_contact_point_m",
        ),
        cylinder_center_in_foot_link_m=_vector3(
            cylinder["center_in_foot_link_m"],
            "center_in_foot_link_m",
        ),
        cylinder_axis_in_foot_link_unit=_unit(
            _vector3(
                cylinder["axis_in_foot_link_unit"],
                "axis_in_foot_link_unit",
            ),
            "axis_in_foot_link_unit",
        ),
        cylinder_radius_m=float(cylinder["radius_m"]),
        total_tread_width_m=float(support["total_tread_width_m"]),
        end_fillet_radius_m=float(support["end_fillet_radius_m"]),
        central_rigid_support_width_m=float(
            support["central_rigid_support_width_m"]
        ),
        nominal_strip_max_axis_tilt_from_ground_rad=float(
            support["nominal_strip_max_axis_tilt_from_ground_rad"]
        ),
    )

    if model.frame_name != "foot_link":
        raise FootContactError(
            f"frame atteso foot_link, trovato {model.frame_name}"
        )

    if model.cylinder_radius_m <= 0.0:
        raise FootContactError("cylinder_radius_m deve essere > 0")

    if model.total_tread_width_m <= 0.0:
        raise FootContactError("total_tread_width_m deve essere > 0")

    if model.end_fillet_radius_m < 0.0:
        raise FootContactError("end_fillet_radius_m deve essere >= 0")

    expected_core_width = (
        model.total_tread_width_m
        - 2.0 * model.end_fillet_radius_m
    )

    if expected_core_width <= 0.0:
        raise FootContactError(
            "La larghezza centrale del battistrada non può essere <= 0"
        )

    if (
        abs(
            model.central_rigid_support_width_m
            - expected_core_width
        )
        > 1e-9
    ):
        raise FootContactError(
            "central_rigid_support_width_m incoerente con "
            "total_tread_width_m - 2 * end_fillet_radius_m"
        )

    if not (
        0.0
        < model.nominal_strip_max_axis_tilt_from_ground_rad
        < math.pi / 2.0
    ):
        raise FootContactError(
            "nominal_strip_max_axis_tilt_from_ground_rad non valido"
        )

    return model


def contact_from_foot_pose(
    model: FootContactModel,
    rotation_world_from_foot: Matrix3,
    translation_world_from_foot_m: Vector3,
    ground_normal_world_unit: Vector3 = (0.0, 0.0, 1.0),
) -> FootGroundContact:
    """
    Restituisce la geometria di contatto del cilindro rigido.

    Il punto cross_section_contact_center_world_m è il punto più basso
    della sezione circolare infinita. Per un cilindro finito inclinato,
    lowest_core_point_world_m identifica quale lato della fascia centrale
    risulta più basso rispetto alla normale del terreno.
    """
    _validate_rotation(rotation_world_from_foot)

    translation = _vector3(
        translation_world_from_foot_m,
        "translation_world_from_foot_m",
    )

    ground_normal = _unit(
        _vector3(
            ground_normal_world_unit,
            "ground_normal_world_unit",
        ),
        "ground_normal_world_unit",
    )

    cylinder_axis_world = _unit(
        _mat_vec(
            rotation_world_from_foot,
            model.cylinder_axis_in_foot_link_unit,
        ),
        "cylinder_axis_world",
    )

    cylinder_center_world = _add(
        _mat_vec(
            rotation_world_from_foot,
            model.cylinder_center_in_foot_link_m,
        ),
        translation,
    )

    axis_ground_component = _dot(
        cylinder_axis_world,
        ground_normal,
    )

    normal_perpendicular_to_axis = _subtract(
        ground_normal,
        _scale(cylinder_axis_world, axis_ground_component),
    )

    perpendicular_norm = _norm(normal_perpendicular_to_axis)

    if perpendicular_norm <= 1e-9:
        raise FootContactDegeneracyError(
            "Asse cilindro quasi parallelo alla normale del terreno: "
            "contatto geometrico degenere."
        )

    radial_down_world = _scale(
        _unit(
            normal_perpendicular_to_axis,
            "normal_perpendicular_to_axis",
        ),
        -1.0,
    )

    cross_section_contact_center_world = _add(
        cylinder_center_world,
        _scale(radial_down_world, model.cylinder_radius_m),
    )

    support_half_width = (
        model.central_rigid_support_width_m / 2.0
    )

    support_strip_end_a_world = _subtract(
        cross_section_contact_center_world,
        _scale(cylinder_axis_world, support_half_width),
    )

    support_strip_end_b_world = _add(
        cross_section_contact_center_world,
        _scale(cylinder_axis_world, support_half_width),
    )

    if _dot(
        ground_normal,
        support_strip_end_a_world,
    ) <= _dot(
        ground_normal,
        support_strip_end_b_world,
    ):
        lowest_core_point_world = support_strip_end_a_world
    else:
        lowest_core_point_world = support_strip_end_b_world

    axis_tilt_from_ground_rad = math.asin(
        min(1.0, abs(axis_ground_component))
    )

    if (
        axis_tilt_from_ground_rad
        <= model.nominal_strip_max_axis_tilt_from_ground_rad
    ):
        support_mode = "NOMINAL_STRIP_CONTACT"
    else:
        support_mode = "EDGE_BIASED_CONTACT"

    return FootGroundContact(
        support_mode=support_mode,
        cylinder_center_world_m=cylinder_center_world,
        cylinder_axis_world_unit=cylinder_axis_world,
        radial_down_world_unit=radial_down_world,
        cross_section_contact_center_world_m=cross_section_contact_center_world,
        support_strip_end_a_world_m=support_strip_end_a_world,
        support_strip_end_b_world_m=support_strip_end_b_world,
        lowest_core_point_world_m=lowest_core_point_world,
        axis_tilt_from_ground_rad=axis_tilt_from_ground_rad,
    )
