#!/usr/bin/env python3
"""
MATDOG — forward kinematics pura dal URDF canonico.

Questo modulo:
- non apre Station;
- non apre porte seriali;
- non invia comandi ai servo;
- usa il vero albero URDF come sorgente delle trasformazioni cinematiche.

La conversione encoder -> radianti resta esterna e deve usare
MATDOG_JOINT_CALIBRATION.yaml + matdog_joint_math.py.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET


CANONICAL_URDF_RELATIVE_PATH = Path(
    "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
)

CANONICAL_URDF_SHA256 = (
    "5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef"
)

LF_CHAIN_JOINT_NAMES = (
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "lf_foot_joint",
)

Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis_xyz: tuple[float, float, float]
    lower_limit_rad: float | None
    upper_limit_rad: float | None


@dataclass(frozen=True)
class FkResult:
    root_link: str
    tip_link: str
    chain_joint_names: tuple[str, ...]
    tip_transform: Matrix4

    @property
    def tip_position_m(self) -> tuple[float, float, float]:
        return (
            self.tip_transform[0][3],
            self.tip_transform[1][3],
            self.tip_transform[2][3],
        )


def canonical_urdf_path(repo_root: Path) -> Path:
    return repo_root / CANONICAL_URDF_RELATIVE_PATH


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_vector(
    raw_value: str | None,
    default: tuple[float, float, float],
    field_name: str,
) -> tuple[float, float, float]:
    if raw_value is None:
        return default

    tokens = raw_value.split()
    if len(tokens) != 3:
        raise ValueError(
            f"{field_name}: attesi 3 valori, ricevuti {raw_value!r}"
        )

    values = tuple(float(token) for token in tokens)

    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{field_name}: valori non finiti {raw_value!r}")

    return values


def load_urdf_joints(urdf_path: Path) -> dict[str, UrdfJoint]:
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF non trovato: {urdf_path}")

    root = ET.parse(urdf_path).getroot()
    joints: dict[str, UrdfJoint] = {}

    for joint_node in root.findall("joint"):
        name = joint_node.get("name")
        joint_type = joint_node.get("type")

        if not name or not joint_type:
            raise ValueError("Joint URDF senza name o type")

        parent_node = joint_node.find("parent")
        child_node = joint_node.find("child")

        if parent_node is None or child_node is None:
            raise ValueError(f"{name}: parent o child mancanti")

        parent_link = parent_node.get("link")
        child_link = child_node.get("link")

        if not parent_link or not child_link:
            raise ValueError(f"{name}: parent link o child link mancanti")

        origin_node = joint_node.find("origin")
        if origin_node is None:
            origin_xyz = (0.0, 0.0, 0.0)
            origin_rpy = (0.0, 0.0, 0.0)
        else:
            origin_xyz = _parse_vector(
                origin_node.get("xyz"),
                (0.0, 0.0, 0.0),
                f"{name}.origin.xyz",
            )
            origin_rpy = _parse_vector(
                origin_node.get("rpy"),
                (0.0, 0.0, 0.0),
                f"{name}.origin.rpy",
            )

        axis_node = joint_node.find("axis")
        axis_xyz = _parse_vector(
            axis_node.get("xyz") if axis_node is not None else None,
            (1.0, 0.0, 0.0),
            f"{name}.axis.xyz",
        )

        limit_node = joint_node.find("limit")
        lower_limit_rad = None
        upper_limit_rad = None

        if joint_type in {"revolute", "prismatic"}:
            if limit_node is None:
                raise ValueError(f"{name}: limit richiesto per joint {joint_type}")

            lower_raw = limit_node.get("lower")
            upper_raw = limit_node.get("upper")

            if lower_raw is None or upper_raw is None:
                raise ValueError(f"{name}: lower/upper limit mancanti")

            lower_limit_rad = float(lower_raw)
            upper_limit_rad = float(upper_raw)

        if name in joints:
            raise ValueError(f"Nome joint duplicato nel URDF: {name}")

        joints[name] = UrdfJoint(
            name=name,
            joint_type=joint_type,
            parent_link=parent_link,
            child_link=child_link,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis_xyz=axis_xyz,
            lower_limit_rad=lower_limit_rad,
            upper_limit_rad=upper_limit_rad,
        )

    if not joints:
        raise ValueError(f"Nessun joint trovato nel URDF: {urdf_path}")

    return joints


def joint_chain(
    joints: dict[str, UrdfJoint],
    root_link: str,
    tip_link: str,
) -> tuple[UrdfJoint, ...]:
    by_child: dict[str, UrdfJoint] = {}

    for joint in joints.values():
        if joint.child_link in by_child:
            raise ValueError(
                f"Più joint con child link {joint.child_link!r}"
            )
        by_child[joint.child_link] = joint

    chain_reversed: list[UrdfJoint] = []
    current_link = tip_link

    while current_link != root_link:
        joint = by_child.get(current_link)

        if joint is None:
            raise ValueError(
                f"Nessuna catena URDF da {root_link!r} a {tip_link!r}; "
                f"link interrotto: {current_link!r}"
            )

        chain_reversed.append(joint)
        current_link = joint.parent_link

    return tuple(reversed(chain_reversed))


def _identity() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matmul(left: Matrix4, right: Matrix4) -> Matrix4:
    return tuple(
        tuple(
            sum(left[row][k] * right[k][column] for k in range(4))
            for column in range(4)
        )
        for row in range(4)
    )  # type: ignore[return-value]


def _transform_from_rotation_translation(
    rotation: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    translation: tuple[float, float, float],
) -> Matrix4:
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], translation[0]),
        (rotation[1][0], rotation[1][1], rotation[1][2], translation[1]),
        (rotation[2][0], rotation[2][1], rotation[2][2], translation[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotation_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # URDF origin rpy: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_angle_rotation(
    axis_xyz: tuple[float, float, float],
    angle_rad: float,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    x, y, z = axis_xyz
    norm = math.sqrt(x * x + y * y + z * z)

    if norm <= 0.0:
        raise ValueError(f"Asse joint nullo: {axis_xyz!r}")

    x /= norm
    y /= norm
    z /= norm

    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    one_minus_c = 1.0 - c

    return (
        (
            c + x * x * one_minus_c,
            x * y * one_minus_c - z * s,
            x * z * one_minus_c + y * s,
        ),
        (
            y * x * one_minus_c + z * s,
            c + y * y * one_minus_c,
            y * z * one_minus_c - x * s,
        ),
        (
            z * x * one_minus_c - y * s,
            z * y * one_minus_c + x * s,
            c + z * z * one_minus_c,
        ),
    )


def _joint_position(
    joint: UrdfJoint,
    joint_positions_rad: dict[str, float],
    enforce_limits: bool,
) -> float:
    if joint.joint_type == "fixed":
        return 0.0

    if joint.joint_type not in {"revolute", "continuous"}:
        raise ValueError(
            f"{joint.name}: joint type non supportato per FK: "
            f"{joint.joint_type!r}"
        )

    value = float(joint_positions_rad.get(joint.name, 0.0))

    if not math.isfinite(value):
        raise ValueError(f"{joint.name}: angolo non finito {value!r}")

    if (
        enforce_limits
        and joint.lower_limit_rad is not None
        and joint.upper_limit_rad is not None
        and not (
            joint.lower_limit_rad - 1e-12
            <= value
            <= joint.upper_limit_rad + 1e-12
        )
    ):
        raise ValueError(
            f"{joint.name}: q={value:.12f} rad fuori dai limiti URDF "
            f"[{joint.lower_limit_rad:.12f}, {joint.upper_limit_rad:.12f}]"
        )

    return value


def _joint_transform(
    joint: UrdfJoint,
    joint_position_rad: float,
) -> Matrix4:
    origin_rotation = _rotation_from_rpy(*joint.origin_rpy)
    origin_transform = _transform_from_rotation_translation(
        origin_rotation,
        joint.origin_xyz,
    )

    if joint.joint_type == "fixed":
        return origin_transform

    motion_rotation = _axis_angle_rotation(
        joint.axis_xyz,
        joint_position_rad,
    )

    motion_transform = _transform_from_rotation_translation(
        motion_rotation,
        (0.0, 0.0, 0.0),
    )

    # URDF convention: parent -> joint origin -> joint-axis motion -> child.
    return _matmul(origin_transform, motion_transform)


def forward_kinematics(
    urdf_path: Path,
    root_link: str,
    tip_link: str,
    joint_positions_rad: dict[str, float],
    enforce_limits: bool = True,
) -> FkResult:
    joints = load_urdf_joints(urdf_path)
    chain = joint_chain(joints, root_link, tip_link)

    transform = _identity()

    for joint in chain:
        position = _joint_position(
            joint,
            joint_positions_rad,
            enforce_limits,
        )
        transform = _matmul(
            transform,
            _joint_transform(joint, position),
        )

    return FkResult(
        root_link=root_link,
        tip_link=tip_link,
        chain_joint_names=tuple(joint.name for joint in chain),
        tip_transform=transform,
    )
