#!/usr/bin/env python3
"""
MATDOG — C4-B offline collision/contact policy.

This script evaluates the C4-A offline safe stand candidate with MATDOG-specific
ground/contact rules.

Important MATDOG-specific distinction:

- the TPU foot cylinder is Shore D 90, solid, and is treated as practically
  rigid for this phase;
- the distal fork of each lower_leg_link intentionally surrounds the foot
  cylinder and can be close to the ground near the contact strip;
- that expected distal fork low clearance is not automatically a failure;
- the critical safety check is the knee / lower_leg_joint clearance relative
  to the foot contact reference, especially for future rest-to-stand motion.

Offline only:
- no Station;
- no serial;
- no motor command;
- no torque, target, speed, accel, stand or gait command.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from matdog_quadruped_leg_contact import LEG_IDS
from matdog_urdf_fk import canonical_urdf_path, forward_kinematics


REPO_ROOT = Path(__file__).resolve().parents[3]
REPORTS_RELATIVE_DIR = Path("09_Logs/Validation_Reports/C4_collision_contact_policy")

NON_FOOT_GROUND_TOLERANCE_M = 1e-6
EXPECTED_FOOT_FORK_LOW_CLEARANCE_M = 0.010
KNEE_CONTACT_REVIEW_CLEARANCE_M = 0.030


Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class LinkGroundClearance:
    link_name: str
    policy: str
    min_z_m: float
    max_z_m: float
    is_foot_link: bool
    is_lower_leg_link: bool


@dataclass(frozen=True)
class KneeContactClearance:
    leg_id: str
    lower_leg_joint_world_m: Vector3
    foot_contact_reference_world_m: Vector3
    knee_contact_clearance_m: float
    policy: str


def _parse_xyz(
    raw: str | None,
    default: Vector3 = (0.0, 0.0, 0.0),
) -> Vector3:
    if raw is None:
        return default

    values = tuple(float(token) for token in raw.split())

    if len(values) != 3:
        raise ValueError(f"xyz/rpy non valido: {raw!r}")

    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"xyz/rpy contiene valori non finiti: {raw!r}")

    return values  # type: ignore[return-value]


def _matmul(left: Matrix4, right: Matrix4) -> Matrix4:
    return tuple(
        tuple(
            sum(left[row][k] * right[k][column] for k in range(4))
            for column in range(4)
        )
        for row in range(4)
    )  # type: ignore[return-value]


def _transform_point(transform: Matrix4, point: Vector3) -> Vector3:
    x, y, z = point

    return (
        transform[0][0] * x
        + transform[0][1] * y
        + transform[0][2] * z
        + transform[0][3],
        transform[1][0] * x
        + transform[1][1] * y
        + transform[1][2] * z
        + transform[1][3],
        transform[2][0] * x
        + transform[2][1] * y
        + transform[2][2] * z
        + transform[2][3],
    )


def _transform_from_xyz_rpy(xyz: Vector3, rpy: Vector3) -> Matrix4:
    roll, pitch, yaw = rpy

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # URDF origin rpy convention: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )

    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]),
        (rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]),
        (rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _stl_vertices(path: Path) -> list[Vector3]:
    data = path.read_bytes()

    if len(data) >= 84:
        triangle_count = struct.unpack("<I", data[80:84])[0]
        expected_size = 84 + triangle_count * 50

        if expected_size == len(data):
            vertices: list[Vector3] = []
            offset = 84

            for _ in range(triangle_count):
                offset += 12

                for _ in range(3):
                    vertices.append(
                        struct.unpack("<fff", data[offset : offset + 12])
                    )
                    offset += 12

                offset += 2

            return vertices

    vertices = []
    text = data.decode("utf-8", errors="ignore")

    for line in text.splitlines():
        parts = line.strip().split()

        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append(tuple(float(value) for value in parts[1:]))

    if not vertices:
        raise ValueError(f"STL senza vertici leggibili: {path}")

    return vertices  # type: ignore[return-value]


def _identity4() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _latest_c4a_report(repo_root: Path) -> Path:
    reports = sorted(
        (repo_root / "09_Logs/Validation_Reports").glob(
            "*_C4A_offline_safe_stand_candidate.json"
        )
    )

    if not reports:
        raise FileNotFoundError("Nessun report C4-A offline safe stand trovato")

    return reports[-1]


def _candidate_joint_positions(candidate: dict) -> dict[str, float]:
    joint_positions: dict[str, float] = {}

    for leg_data in candidate["legs"].values():
        joint_positions.update(leg_data["joint_positions_rad"])

    return joint_positions


def _world_from_base(candidate: dict) -> Matrix4:
    translation = candidate["body_pose"]["translation_world_m"]

    return (
        (1.0, 0.0, 0.0, float(translation[0])),
        (0.0, 1.0, 0.0, float(translation[1])),
        (0.0, 0.0, 1.0, float(translation[2])),
        (0.0, 0.0, 0.0, 1.0),
    )


def _link_world_transform(
    urdf_path: Path,
    link_name: str,
    joint_positions_rad: dict[str, float],
    world_from_base: Matrix4,
) -> Matrix4:
    if link_name == "base_link":
        base_from_link = _identity4()
    else:
        fk = forward_kinematics(
            urdf_path=urdf_path,
            root_link="base_link",
            tip_link=link_name,
            joint_positions_rad=joint_positions_rad,
            enforce_limits=True,
        )
        base_from_link = fk.tip_transform

    return _matmul(world_from_base, base_from_link)


def _evaluate_link_ground_clearance(
    repo_root: Path,
    urdf_path: Path,
    candidate: dict,
) -> list[LinkGroundClearance]:
    root = ET.parse(urdf_path).getroot()
    urdf_dir = urdf_path.parent

    joint_positions = _candidate_joint_positions(candidate)
    world_from_base = _world_from_base(candidate)

    results: list[LinkGroundClearance] = []

    for link in root.findall("link"):
        link_name = str(link.attrib["name"])
        collision = link.find("collision")

        if collision is None:
            continue

        mesh = collision.find("./geometry/mesh")

        if mesh is None:
            continue

        filename = mesh.attrib["filename"]
        scale = _parse_xyz(mesh.attrib.get("scale"), (1.0, 1.0, 1.0))

        origin = collision.find("origin")
        origin_xyz = _parse_xyz(
            origin.attrib.get("xyz") if origin is not None else None
        )
        origin_rpy = _parse_xyz(
            origin.attrib.get("rpy") if origin is not None else None
        )

        vertices = _stl_vertices(urdf_dir / filename)
        scaled_vertices = [
            (
                vertex[0] * scale[0],
                vertex[1] * scale[1],
                vertex[2] * scale[2],
            )
            for vertex in vertices
        ]

        world_from_link = _link_world_transform(
            urdf_path=urdf_path,
            link_name=link_name,
            joint_positions_rad=joint_positions,
            world_from_base=world_from_base,
        )
        link_from_collision = _transform_from_xyz_rpy(origin_xyz, origin_rpy)
        world_from_collision = _matmul(world_from_link, link_from_collision)

        world_points = [
            _transform_point(world_from_collision, vertex)
            for vertex in scaled_vertices
        ]

        min_z = min(point[2] for point in world_points)
        max_z = max(point[2] for point in world_points)

        is_foot_link = link_name.endswith("_foot_link")
        is_lower_leg_link = link_name.endswith("_lower_leg_link")

        if is_foot_link:
            policy = "FOOT_CONTACT_ALLOWED_RIGID_TPU"
        elif min_z < -NON_FOOT_GROUND_TOLERANCE_M:
            policy = "NON_FOOT_GROUND_PENETRATION_FAIL"
        elif is_lower_leg_link and min_z < EXPECTED_FOOT_FORK_LOW_CLEARANCE_M:
            policy = "EXPECTED_FOOT_FORK_LOW_CLEARANCE_REVIEW"
        elif min_z < EXPECTED_FOOT_FORK_LOW_CLEARANCE_M:
            policy = "LOW_CLEARANCE_REVIEW"
        else:
            policy = "GROUND_CLEARANCE_OK"

        results.append(
            LinkGroundClearance(
                link_name=link_name,
                policy=policy,
                min_z_m=min_z,
                max_z_m=max_z,
                is_foot_link=is_foot_link,
                is_lower_leg_link=is_lower_leg_link,
            )
        )

    return results


def _evaluate_knee_contact_clearance(
    urdf_path: Path,
    candidate: dict,
) -> list[KneeContactClearance]:
    joint_positions = _candidate_joint_positions(candidate)
    world_from_base = _world_from_base(candidate)

    results: list[KneeContactClearance] = []

    for leg_id in LEG_IDS:
        lower_leg_link = f"{leg_id}_lower_leg_link"

        world_from_lower_leg = _link_world_transform(
            urdf_path=urdf_path,
            link_name=lower_leg_link,
            joint_positions_rad=joint_positions,
            world_from_base=world_from_base,
        )

        lower_leg_joint_world = (
            world_from_lower_leg[0][3],
            world_from_lower_leg[1][3],
            world_from_lower_leg[2][3],
        )

        leg_report = candidate["legs"][leg_id]

        foot_contact_world = tuple(
            float(value)
            for value in leg_report["achieved_contact_reference_world_m"]
        )

        clearance = lower_leg_joint_world[2] - foot_contact_world[2]

        if clearance <= 0.0:
            policy = "KNEE_BELOW_OR_AT_CONTACT_FAIL"
        elif clearance < KNEE_CONTACT_REVIEW_CLEARANCE_M:
            policy = "KNEE_CONTACT_CLEARANCE_REVIEW"
        else:
            policy = "KNEE_CONTACT_CLEARANCE_OK"

        results.append(
            KneeContactClearance(
                leg_id=leg_id,
                lower_leg_joint_world_m=lower_leg_joint_world,
                foot_contact_reference_world_m=foot_contact_world,  # type: ignore[arg-type]
                knee_contact_clearance_m=clearance,
                policy=policy,
            )
        )

    return results


def _overall_status(
    link_clearances: list[LinkGroundClearance],
    knee_clearances: list[KneeContactClearance],
) -> str:
    fail_policies = {
        "NON_FOOT_GROUND_PENETRATION_FAIL",
        "KNEE_BELOW_OR_AT_CONTACT_FAIL",
    }

    review_prefixes = (
        "EXPECTED_FOOT_FORK_LOW_CLEARANCE_REVIEW",
        "LOW_CLEARANCE_REVIEW",
        "KNEE_CONTACT_CLEARANCE_REVIEW",
    )

    policies = [item.policy for item in link_clearances] + [
        item.policy for item in knee_clearances
    ]

    if any(policy in fail_policies for policy in policies):
        return "FAIL"

    if any(policy in review_prefixes for policy in policies):
        return "PASS_WITH_EXPECTED_FOOT_FORK_REVIEW"

    return "PASS"


def _default_report_path(repo_root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        repo_root
        / REPORTS_RELATIVE_DIR
        / f"{stamp}_C4B_collision_contact_policy.json"
    )


def _build_report(
    repo_root: Path,
    candidate_report_path: Path,
    link_clearances: list[LinkGroundClearance],
    knee_clearances: list[KneeContactClearance],
    status: str,
) -> dict:
    return {
        "schema": 1,
        "kind": "MATDOG_C4B_OFFLINE_COLLISION_CONTACT_POLICY",
        "status": status,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "candidate_report": str(candidate_report_path),
        "offline_only": True,
        "station_used": False,
        "serial_used": False,
        "motor_command_used": False,
        "material_assumption": {
            "foot_tpu_shore": "90D",
            "treated_as": "practically_rigid_for_C4B",
            "compression_model_used": False,
        },
        "matdog_specific_policy": {
            "foot_link_contact": "allowed",
            "distal_lower_leg_fork_low_clearance": (
                "expected near the rigid TPU foot cylinder if above world Z=0"
            ),
            "critical_kinematic_risk": (
                "lower_leg_joint/knee descending toward or below the foot "
                "contact reference during rest-to-stand"
            ),
            "command_eligible": False,
        },
        "thresholds": {
            "non_foot_ground_tolerance_m": NON_FOOT_GROUND_TOLERANCE_M,
            "expected_foot_fork_low_clearance_m": (
                EXPECTED_FOOT_FORK_LOW_CLEARANCE_M
            ),
            "knee_contact_review_clearance_m": (
                KNEE_CONTACT_REVIEW_CLEARANCE_M
            ),
        },
        "link_ground_clearance": [
            asdict(item) for item in link_clearances
        ],
        "knee_contact_clearance": [
            asdict(item) for item in knee_clearances
        ],
        "command_eligibility": {
            "command_eligible": False,
            "reason": (
                "C4-B validates only the static C4-A candidate. "
                "Trajectory sampling, stability and supervised hardware gates "
                "are still required before any automatic stand command."
            ),
        },
    }


def _print_summary(
    candidate_report_path: Path,
    link_clearances: list[LinkGroundClearance],
    knee_clearances: list[KneeContactClearance],
    status: str,
    report_path: Path,
) -> None:
    print("=== MATDOG C4-B OFFLINE COLLISION/CONTACT POLICY ===")
    print(f"candidate_report: {candidate_report_path}")
    print("material_assumption: TPU_90_SHORE_D_TREATED_AS_PRACTICALLY_RIGID")
    print("command_eligible: false")
    print("")

    print("GROUND / MESH CLEARANCE:")
    for item in link_clearances:
        print(
            f"{item.policy:42} {item.link_name:20} "
            f"min_z={item.min_z_m:+.6f} m "
            f"max_z={item.max_z_m:+.6f} m"
        )

    print("")
    print("KNEE / FOOT CONTACT CLEARANCE:")
    for item in knee_clearances:
        print(
            f"{item.policy:32} {item.leg_id.upper()} "
            f"knee_z={item.lower_leg_joint_world_m[2]:+.6f} m "
            f"contact_z={item.foot_contact_reference_world_m[2]:+.6f} m "
            f"clearance={item.knee_contact_clearance_m * 1000.0:.3f} mm"
        )

    print("")
    print(f"C4_B_COLLISION_CONTACT_STATUS: {status}")
    print(f"report: {report_path}")
    print("Offline only: no Station, serial or motor command was used.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MATDOG C4-B offline collision/contact policy."
    )
    parser.add_argument(
        "--candidate-report",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    candidate_report_path = (
        args.candidate_report
        if args.candidate_report is not None
        else _latest_c4a_report(REPO_ROOT)
    )

    candidate = json.loads(
        candidate_report_path.read_text(encoding="utf-8")
    )

    urdf_path = canonical_urdf_path(REPO_ROOT)

    link_clearances = _evaluate_link_ground_clearance(
        repo_root=REPO_ROOT,
        urdf_path=urdf_path,
        candidate=candidate,
    )
    knee_clearances = _evaluate_knee_contact_clearance(
        urdf_path=urdf_path,
        candidate=candidate,
    )

    status = _overall_status(link_clearances, knee_clearances)

    report_path = (
        args.report_path
        if args.report_path is not None
        else _default_report_path(REPO_ROOT)
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)

    report = _build_report(
        repo_root=REPO_ROOT,
        candidate_report_path=candidate_report_path,
        link_clearances=link_clearances,
        knee_clearances=knee_clearances,
        status=status,
    )

    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    _print_summary(
        candidate_report_path=candidate_report_path,
        link_clearances=link_clearances,
        knee_clearances=knee_clearances,
        status=status,
        report_path=report_path,
    )

    if status == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
