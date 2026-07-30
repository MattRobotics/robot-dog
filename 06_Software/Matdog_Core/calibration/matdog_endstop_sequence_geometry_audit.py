#!/usr/bin/env python3
"""Offline collision audit for the ordered MATDOG end-stop sequence.

This program never opens Station or a serial device. It loads the canonical
URDF and its exact collision STL meshes, derives the horizontal UPPER and
compact/folded LOWER poses, and samples the complete UPPER -> LOWER -> HIP
sequence for all four legs.

Hardware certification additionally requires a fixture keep-out YAML. The
external support is not part of the URDF and therefore cannot be silently
ignored.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import Iterable

import numpy as np
import trimesh
import yaml


URDF_RELATIVE = Path("03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf")
LEGS = ("lf", "rf", "rh", "lh")
FRONT_REAR_PARK = {"lf": "lh", "rf": "rh"}
JOINT_KIND = ("hip", "upper_leg", "lower_leg")
DEG = math.pi / 180.0


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


@dataclass(frozen=True)
class CollisionGeometry:
    link: str
    mesh: trimesh.Trimesh
    local_transform: np.ndarray


@dataclass(frozen=True)
class FixtureBox:
    name: str
    minimum: np.ndarray
    maximum: np.ndarray
    ignore_links: frozenset[str]


class AuditFailure(RuntimeError):
    pass


def _vector(raw: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if raw is None:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in raw.split()]
    if len(values) != 3 or not np.isfinite(values).all():
        raise AuditFailure(f"invalid URDF vector: {raw!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def _translation(xyz: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = xyz
    return result


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm <= 0.0:
        raise AuditFailure("zero joint axis")
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    rotation = np.asarray(
        [
            [c + x * x * k, x * y * k - z * s, x * z * k + y * s],
            [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
            [z * x * k - y * s, z * y * k + x * s, c + z * z * k],
        ],
        dtype=float,
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def _origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    return _translation(xyz) @ _rpy_matrix(rpy)


def _mesh_path(urdf_path: Path, filename: str) -> Path:
    if filename.startswith("package://"):
        filename = filename.removeprefix("package://")
    candidate = Path(filename)
    if candidate.is_absolute():
        return candidate
    return (urdf_path.parent / candidate).resolve()


def load_model(urdf_path: Path) -> tuple[str, dict[str, Joint], list[CollisionGeometry]]:
    root = ET.parse(urdf_path).getroot()
    links = {node.attrib["name"] for node in root.findall("link")}
    joints: dict[str, Joint] = {}
    child_links: set[str] = set()

    for node in root.findall("joint"):
        name = node.attrib["name"]
        joint_type = node.attrib["type"]
        parent = node.find("parent").attrib["link"]  # type: ignore[union-attr]
        child = node.find("child").attrib["link"]  # type: ignore[union-attr]
        origin = node.find("origin")
        axis_node = node.find("axis")
        limit = node.find("limit")
        xyz = _vector(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0))
        rpy = _vector(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0))
        axis = _vector(axis_node.attrib.get("xyz") if axis_node is not None else None, (1, 0, 0))
        lower = float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None
        upper = float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None
        joints[name] = Joint(name, joint_type, parent, child, xyz, rpy, axis, lower, upper)
        child_links.add(child)

    roots = links - child_links
    if len(roots) != 1:
        raise AuditFailure(f"expected one URDF root, found {sorted(roots)}")
    root_link = next(iter(roots))

    geometries: list[CollisionGeometry] = []
    for link_node in root.findall("link"):
        link_name = link_node.attrib["name"]
        for collision in link_node.findall("collision"):
            origin = collision.find("origin")
            local = _origin_transform(
                _vector(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)),
                _vector(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)),
            )
            mesh_node = collision.find("geometry/mesh")
            if mesh_node is None:
                raise AuditFailure(f"{link_name}: collision geometry is not a mesh")
            path = _mesh_path(urdf_path, mesh_node.attrib["filename"])
            loaded = trimesh.load_mesh(path, process=False)
            if isinstance(loaded, trimesh.Scene):
                loaded = loaded.dump(concatenate=True)
            if not isinstance(loaded, trimesh.Trimesh):
                raise AuditFailure(f"{link_name}: unsupported mesh type {type(loaded)}")
            scale = _vector(mesh_node.attrib.get("scale"), (1, 1, 1))
            mesh = loaded.copy()
            mesh.vertices = np.asarray(mesh.vertices) * scale
            geometries.append(CollisionGeometry(link_name, mesh, local))

    return root_link, joints, geometries


def link_transforms(root_link: str, joints: dict[str, Joint], q: dict[str, float]) -> dict[str, np.ndarray]:
    transforms = {root_link: np.eye(4)}
    unresolved = set(joints)
    while unresolved:
        progressed = False
        for name in tuple(unresolved):
            joint = joints[name]
            parent_tf = transforms.get(joint.parent)
            if parent_tf is None:
                continue
            value = float(q.get(name, 0.0))
            if joint.joint_type == "revolute":
                if joint.lower is None or joint.upper is None:
                    raise AuditFailure(f"{name}: missing limits")
                if not joint.lower - 1e-10 <= value <= joint.upper + 1e-10:
                    raise AuditFailure(
                        f"{name}: q={value / DEG:.3f} deg outside "
                        f"[{joint.lower / DEG:.3f}, {joint.upper / DEG:.3f}]"
                    )
                motion = _axis_angle(joint.axis, value)
            elif joint.joint_type == "fixed":
                motion = np.eye(4)
            else:
                raise AuditFailure(f"unsupported joint type {joint.joint_type!r}")
            transforms[joint.child] = parent_tf @ _origin_transform(joint.xyz, joint.rpy) @ motion
            unresolved.remove(name)
            progressed = True
        if not progressed:
            raise AuditFailure(f"cannot resolve joint tree: {sorted(unresolved)}")
    return transforms


def adjacent_link_pairs(joints: dict[str, Joint]) -> set[frozenset[str]]:
    return {frozenset((joint.parent, joint.child)) for joint in joints.values()}


def collision_pairs(
    geometries: list[CollisionGeometry],
    transforms: dict[str, np.ndarray],
    ignored_pairs: set[frozenset[str]],
) -> set[tuple[str, str]]:
    manager = trimesh.collision.CollisionManager()
    for index, geometry in enumerate(geometries):
        manager.add_object(
            f"{geometry.link}#{index}",
            geometry.mesh,
            transform=transforms[geometry.link] @ geometry.local_transform,
        )
    collided, names = manager.in_collision_internal(return_names=True)
    if not collided:
        return set()
    result: set[tuple[str, str]] = set()
    for left_raw, right_raw in names:
        left = left_raw.split("#", 1)[0]
        right = right_raw.split("#", 1)[0]
        if left == right or frozenset((left, right)) in ignored_pairs:
            continue
        result.add(tuple(sorted((left, right))))
    return result


def fixture_collisions(
    geometries: list[CollisionGeometry],
    transforms: dict[str, np.ndarray],
    boxes: list[FixtureBox],
    safety_margin: float,
) -> set[tuple[str, str]]:
    collisions: set[tuple[str, str]] = set()
    for box in boxes:
        extent = box.maximum - box.minimum + 2.0 * safety_margin
        if np.any(extent <= 0):
            raise AuditFailure(f"fixture {box.name}: non-positive extent")
        center = 0.5 * (box.minimum + box.maximum)
        fixture_mesh = trimesh.creation.box(extents=extent)
        fixture_tf = _translation(center)
        fixture_manager = trimesh.collision.CollisionManager()
        fixture_manager.add_object(box.name, fixture_mesh, transform=fixture_tf)
        for index, geometry in enumerate(geometries):
            if geometry.link in box.ignore_links:
                continue
            robot_manager = trimesh.collision.CollisionManager()
            robot_manager.add_object(
                f"{geometry.link}#{index}",
                geometry.mesh,
                transform=transforms[geometry.link] @ geometry.local_transform,
            )
            if robot_manager.in_collision_other(fixture_manager):
                collisions.add((geometry.link, box.name))
    return collisions


def load_fixture(path: Path | None) -> tuple[list[FixtureBox], float]:
    if path is None:
        return [], 0.0
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or data.get("frame") != "base_link":
        raise AuditFailure("fixture schema_version=1 and frame=base_link are required")
    margin = float(data.get("safety_margin_m", 0.0))
    if not math.isfinite(margin) or margin < 0.0:
        raise AuditFailure("invalid fixture safety margin")
    boxes: list[FixtureBox] = []
    for raw in data.get("keepout_boxes", []):
        minimum = np.asarray(raw["min_xyz_m"], dtype=float)
        maximum = np.asarray(raw["max_xyz_m"], dtype=float)
        if minimum.shape != (3,) or maximum.shape != (3,) or not np.isfinite(minimum).all() or not np.isfinite(maximum).all():
            raise AuditFailure(f"invalid fixture box {raw!r}")
        if np.any(maximum <= minimum):
            raise AuditFailure(f"fixture box has invalid bounds: {raw['name']}")
        if raw.get("placeholder", False):
            raise AuditFailure(f"fixture box is still a placeholder: {raw['name']}")
        boxes.append(
            FixtureBox(
                str(raw["name"]),
                minimum,
                maximum,
                frozenset(str(value) for value in raw.get("ignore_links", [])),
            )
        )
    if not boxes:
        raise AuditFailure("fixture file contains no keep-out boxes")
    return boxes, margin


def joint_name(leg: str, kind: str) -> str:
    return f"{leg}_{kind}_joint"


def _segment_vector(
    transforms: dict[str, np.ndarray],
    joints: dict[str, Joint],
    link: str,
    downstream_joint: str,
) -> np.ndarray:
    direction_local = joints[downstream_joint].xyz
    return transforms[link][:3, :3] @ direction_local


def derive_horizontal_upper(
    leg: str,
    root_link: str,
    joints: dict[str, Joint],
) -> tuple[float, float]:
    upper_joint = joints[joint_name(leg, "upper_leg")]
    lower_joint_name = joint_name(leg, "lower_leg")
    best: tuple[float, float] | None = None
    samples = int(round((upper_joint.upper - upper_joint.lower) / (0.25 * DEG))) + 1  # type: ignore[operator]
    for value in np.linspace(upper_joint.lower, upper_joint.upper, samples):  # type: ignore[arg-type]
        transforms = link_transforms(root_link, joints, {upper_joint.name: float(value)})
        vector = _segment_vector(transforms, joints, upper_joint.child, lower_joint_name)
        score = abs(float(vector[2])) / float(np.linalg.norm(vector))
        if best is None or score < best[1]:
            best = (float(value), score)
    assert best is not None
    return best


def derive_compact_lower(
    leg: str,
    upper_value: float,
    root_link: str,
    joints: dict[str, Joint],
) -> tuple[float, dict[str, float]]:
    upper_joint = joints[joint_name(leg, "upper_leg")]
    lower_joint = joints[joint_name(leg, "lower_leg")]
    foot_joint_name = f"{leg}_foot_joint"
    samples = int(round((lower_joint.upper - lower_joint.lower) / (0.25 * DEG))) + 1  # type: ignore[operator]
    best_value: float | None = None
    best_metrics: dict[str, float] | None = None
    for value in np.linspace(lower_joint.lower, lower_joint.upper, samples):  # type: ignore[arg-type]
        q = {upper_joint.name: upper_value, lower_joint.name: float(value)}
        transforms = link_transforms(root_link, joints, q)
        upper_vector = _segment_vector(
            transforms,
            joints,
            upper_joint.child,
            lower_joint.name,
        )
        lower_vector = _segment_vector(
            transforms,
            joints,
            lower_joint.child,
            foot_joint_name,
        )
        upper_unit = upper_vector / np.linalg.norm(upper_vector)
        lower_unit = lower_vector / np.linalg.norm(lower_vector)
        parallel_opposite_error = 1.0 + float(np.dot(upper_unit, lower_unit))
        lower_horizontal_error = abs(float(lower_unit[2]))
        score = 4.0 * lower_horizontal_error + parallel_opposite_error
        metrics = {
            "score": score,
            "lower_horizontal_error": lower_horizontal_error,
            "parallel_opposite_error": parallel_opposite_error,
            "upper_lower_angle_deg": math.degrees(
                math.acos(float(np.clip(np.dot(upper_unit, lower_unit), -1.0, 1.0)))
            ),
        }
        if best_metrics is None or score < best_metrics["score"]:
            best_value = float(value)
            best_metrics = metrics
    assert best_value is not None and best_metrics is not None
    return best_value, best_metrics


def interpolated(start: float, stop: float, step_deg: float) -> Iterable[float]:
    distance_deg = abs(stop - start) / DEG
    count = max(1, int(math.ceil(distance_deg / step_deg)))
    for index in range(count + 1):
        yield start + (stop - start) * index / count


def base_pose() -> dict[str, float]:
    return {joint_name(leg, kind): 0.0 for leg in LEGS for kind in JOINT_KIND}


def sample_segment(
    *,
    label: str,
    q_start: dict[str, float],
    q_stop: dict[str, float],
    step_deg: float,
    root_link: str,
    joints: dict[str, Joint],
    geometries: list[CollisionGeometry],
    ignored_pairs: set[frozenset[str]],
    fixture_boxes: list[FixtureBox],
    fixture_margin: float,
) -> int:
    changed = [name for name in q_start if abs(q_start[name] - q_stop[name]) > 1e-12]
    if len(changed) > 1:
        raise AuditFailure(f"{label}: only one joint may move, changed={changed}")
    if not changed:
        values = [0.0]
        moving = None
    else:
        moving = changed[0]
        values = list(interpolated(q_start[moving], q_stop[moving], step_deg))
    for index, value in enumerate(values):
        q = dict(q_start)
        if moving is not None:
            q[moving] = value
        transforms = link_transforms(root_link, joints, q)
        self_pairs = collision_pairs(geometries, transforms, ignored_pairs)
        if self_pairs:
            raise AuditFailure(
                f"{label}: self/cross collision at sample {index}/{len(values)-1}: "
                f"{sorted(self_pairs)}"
            )
        fixture_pairs = fixture_collisions(
            geometries,
            transforms,
            fixture_boxes,
            fixture_margin,
        )
        if fixture_pairs:
            raise AuditFailure(
                f"{label}: fixture collision at sample {index}/{len(values)-1}: "
                f"{sorted(fixture_pairs)}"
            )
    return len(values)


def audit_leg(
    leg: str,
    *,
    root_link: str,
    joints: dict[str, Joint],
    geometries: list[CollisionGeometry],
    ignored_pairs: set[frozenset[str]],
    fixture_boxes: list[FixtureBox],
    fixture_margin: float,
) -> dict[str, object]:
    upper_name = joint_name(leg, "upper_leg")
    lower_name = joint_name(leg, "lower_leg")
    hip_name = joint_name(leg, "hip")
    upper = joints[upper_name]
    lower = joints[lower_name]
    hip = joints[hip_name]
    assert upper.lower is not None and upper.upper is not None
    assert lower.lower is not None and lower.upper is not None
    assert hip.lower is not None and hip.upper is not None

    upper_horizontal, upper_error = derive_horizontal_upper(leg, root_link, joints)
    lower_compact, lower_metrics = derive_compact_lower(
        leg, upper_horizontal, root_link, joints
    )
    if upper_error > math.sin(0.5 * DEG):
        raise AuditFailure(
            f"{leg}: no upper-horizontal pose within 0.5 deg; error={upper_error}"
        )
    if lower_metrics["lower_horizontal_error"] > math.sin(0.75 * DEG):
        raise AuditFailure(
            f"{leg}: compact lower is not horizontal enough: {lower_metrics}"
        )
    if lower_metrics["upper_lower_angle_deg"] < 175.0:
        raise AuditFailure(f"{leg}: lower is not folded close to upper: {lower_metrics}")

    q = base_pose()
    samples = 0

    parked_leg = FRONT_REAR_PARK.get(leg)
    if parked_leg is not None:
        target = dict(q)
        target[joint_name(parked_leg, "upper_leg")] = 30.0 * DEG
        samples += sample_segment(
            label=f"{leg}: rear parking",
            q_start=q,
            q_stop=target,
            step_deg=0.5,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes,
            fixture_margin=fixture_margin,
        )
        q = target

    for side_name, limit in (("MIN", upper.lower), ("MAX", upper.upper)):
        target = dict(q)
        target[upper_name] = limit
        samples += sample_segment(
            label=f"{leg}: UPPER {side_name}", q_start=q, q_stop=target,
            step_deg=1.0, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )
        samples += sample_segment(
            label=f"{leg}: UPPER {side_name} return", q_start=target, q_stop=q,
            step_deg=1.0, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )

    upper_pose = dict(q)
    upper_pose[upper_name] = upper_horizontal
    samples += sample_segment(
        label=f"{leg}: upper to horizontal", q_start=q, q_stop=upper_pose,
        step_deg=0.5, root_link=root_link, joints=joints,
        geometries=geometries, ignored_pairs=ignored_pairs,
        fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
    )
    q = upper_pose

    for side_name, limit in (("MIN", lower.lower), ("MAX", lower.upper)):
        target = dict(q)
        target[lower_name] = limit
        samples += sample_segment(
            label=f"{leg}: LOWER {side_name}", q_start=q, q_stop=target,
            step_deg=1.0, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )
        samples += sample_segment(
            label=f"{leg}: LOWER {side_name} return", q_start=target, q_stop=q,
            step_deg=1.0, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )

    compact_pose = dict(q)
    compact_pose[lower_name] = lower_compact
    samples += sample_segment(
        label=f"{leg}: lower to compact", q_start=q, q_stop=compact_pose,
        step_deg=0.5, root_link=root_link, joints=joints,
        geometries=geometries, ignored_pairs=ignored_pairs,
        fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
    )
    q = compact_pose

    for side_name, limit in (("MIN", hip.lower), ("MAX", hip.upper)):
        target = dict(q)
        target[hip_name] = limit
        samples += sample_segment(
            label=f"{leg}: HIP {side_name}", q_start=q, q_stop=target,
            step_deg=1.0, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )
        samples += sample_segment(
            label=f"{leg}: HIP {side_name} return", q_start=target, q_stop=q,
            step_deg=1.0, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )

    lower_home = dict(q)
    lower_home[lower_name] = 0.0
    samples += sample_segment(
        label=f"{leg}: lower compact to home", q_start=q, q_stop=lower_home,
        step_deg=0.5, root_link=root_link, joints=joints,
        geometries=geometries, ignored_pairs=ignored_pairs,
        fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
    )
    q = lower_home

    upper_home = dict(q)
    upper_home[upper_name] = 0.0
    samples += sample_segment(
        label=f"{leg}: upper horizontal to home", q_start=q, q_stop=upper_home,
        step_deg=0.5, root_link=root_link, joints=joints,
        geometries=geometries, ignored_pairs=ignored_pairs,
        fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
    )
    q = upper_home

    if parked_leg is not None:
        rear_home = dict(q)
        rear_home[joint_name(parked_leg, "upper_leg")] = 0.0
        samples += sample_segment(
            label=f"{leg}: rear parking return", q_start=q, q_stop=rear_home,
            step_deg=0.5, root_link=root_link, joints=joints,
            geometries=geometries, ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes, fixture_margin=fixture_margin,
        )
        q = rear_home

    return {
        "leg": leg.upper(),
        "upper_horizontal_rad": upper_horizontal,
        "upper_horizontal_deg": upper_horizontal / DEG,
        "upper_horizontal_error": upper_error,
        "lower_compact_rad": lower_compact,
        "lower_compact_deg": lower_compact / DEG,
        "lower_metrics": lower_metrics,
        "sample_count": samples,
        "final_home": all(abs(value) < 1e-12 for value in q.values()),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--fixture", type=Path)
    parser.add_argument("--certify-hardware", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    urdf_path = repo_root / URDF_RELATIVE
    fixture_boxes, fixture_margin = load_fixture(args.fixture)
    if args.certify_hardware and not fixture_boxes:
        raise AuditFailure(
            "hardware certification requires --fixture with measured non-placeholder keep-outs"
        )

    root_link, joints, geometries = load_model(urdf_path)
    ignored_pairs = adjacent_link_pairs(joints)
    results = [
        audit_leg(
            leg,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes,
            fixture_margin=fixture_margin,
        )
        for leg in LEGS
    ]
    payload = {
        "result": "PASS",
        "hardware_certified": bool(args.certify_hardware),
        "fixture_boxes": [box.name for box in fixture_boxes],
        "fixture_margin_m": fixture_margin,
        "order": ["UPPER_MIN_MAX", "LOWER_MIN_MAX", "HIP_MIN_MAX"],
        "legs": results,
    }
    rendered = json.dumps(payload, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AuditFailure as error:
        print(f"HARD BLOCK: {error}", file=sys.stderr)
        raise SystemExit(2)
