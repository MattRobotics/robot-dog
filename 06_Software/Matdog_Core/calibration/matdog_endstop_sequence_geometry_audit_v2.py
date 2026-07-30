#!/usr/bin/env python3
"""V2 ordered end-stop audit: select compact LOWER from real mesh envelope.

The base module provides URDF parsing, exact STL collision checks and fixture
handling. This V2 replaces the impossible requirement that the knee-to-foot
vector be exactly horizontal/antiparallel. Instead it searches LOWER poses at
least five degrees inside the URDF limits, ranks them by the maximum radius of
the real lower+foot collision meshes around the active upper joint, and accepts
the first candidate whose complete transition and HIP MIN/MAX paths are free
of non-adjacent, cross-leg and fixture collisions.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


HERE = Path(__file__).resolve().parent
BASE_PATH = HERE / "matdog_endstop_sequence_geometry_audit.py"
SPEC = importlib.util.spec_from_file_location("matdog_sequence_audit_base", BASE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import base audit: {BASE_PATH}")
base = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = base
SPEC.loader.exec_module(base)

MODEL_LIMIT_MARGIN_DEG = 5.0
CANDIDATE_STEP_DEG = 0.5


def transformed_vertices(geometry: Any, transforms: dict[str, np.ndarray]) -> np.ndarray:
    matrix = transforms[geometry.link] @ geometry.local_transform
    vertices = np.column_stack(
        (np.asarray(geometry.mesh.vertices, dtype=float), np.ones(len(geometry.mesh.vertices)))
    )
    return (matrix @ vertices.T).T[:, :3]


def compact_candidates(
    leg: str,
    upper_value: float,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    ignored_pairs: set[frozenset[str]],
) -> list[dict[str, float]]:
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    foot_joint_name = f"{leg}_foot_joint"
    upper_joint = joints[upper_name]
    lower_joint = joints[lower_name]
    assert lower_joint.lower is not None and lower_joint.upper is not None

    minimum = lower_joint.lower + MODEL_LIMIT_MARGIN_DEG * base.DEG
    maximum = lower_joint.upper - MODEL_LIMIT_MARGIN_DEG * base.DEG
    if minimum >= maximum:
        raise base.AuditFailure(f"{leg}: no compact search range after model margin")

    count = int(round((maximum - minimum) / (CANDIDATE_STEP_DEG * base.DEG))) + 1
    lower_links = {lower_joint.child, joints[foot_joint_name].child}
    candidates: list[dict[str, float]] = []

    for value in np.linspace(minimum, maximum, count):
        q = {upper_name: upper_value, lower_name: float(value)}
        transforms = base.link_transforms(root_link, joints, q)
        if base.collision_pairs(geometries, transforms, ignored_pairs):
            continue

        upper_origin = transforms[upper_joint.child][:3, 3]
        maximum_radius = 0.0
        for geometry in geometries:
            if geometry.link not in lower_links:
                continue
            vertices = transformed_vertices(geometry, transforms)
            radius = float(np.linalg.norm(vertices - upper_origin, axis=1).max())
            maximum_radius = max(maximum_radius, radius)

        upper_vector = base._segment_vector(
            transforms, joints, upper_joint.child, lower_name
        )
        lower_vector = base._segment_vector(
            transforms, joints, lower_joint.child, foot_joint_name
        )
        upper_unit = upper_vector / np.linalg.norm(upper_vector)
        lower_unit = lower_vector / np.linalg.norm(lower_vector)
        dot = float(np.clip(np.dot(upper_unit, lower_unit), -1.0, 1.0))

        candidates.append(
            {
                "value_rad": float(value),
                "value_deg": float(value / base.DEG),
                "mesh_radius_m": maximum_radius,
                "knee_to_foot_horizontal_error": abs(float(lower_unit[2])),
                "upper_lower_angle_deg": math.degrees(math.acos(dot)),
                "distance_from_min_deg": float((value - lower_joint.lower) / base.DEG),
                "distance_from_max_deg": float((lower_joint.upper - value) / base.DEG),
            }
        )

    if not candidates:
        raise base.AuditFailure(f"{leg}: no collision-free compact LOWER candidates")
    return sorted(
        candidates,
        key=lambda item: (
            item["mesh_radius_m"],
            -item["upper_lower_angle_deg"],
            item["knee_to_foot_horizontal_error"],
        ),
    )


def audit_leg_v2(
    leg: str,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    ignored_pairs: set[frozenset[str]],
    fixture_boxes: list[Any],
    fixture_margin: float,
) -> dict[str, object]:
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    hip_name = base.joint_name(leg, "hip")
    upper = joints[upper_name]
    lower = joints[lower_name]
    hip = joints[hip_name]
    assert upper.lower is not None and upper.upper is not None
    assert lower.lower is not None and lower.upper is not None
    assert hip.lower is not None and hip.upper is not None

    upper_horizontal, upper_error = base.derive_horizontal_upper(leg, root_link, joints)
    if upper_error > math.sin(0.5 * base.DEG):
        raise base.AuditFailure(
            f"{leg}: no UPPER horizontal pose within 0.5 deg, error={upper_error}"
        )

    q = base.base_pose()
    samples = 0
    parked_leg = base.FRONT_REAR_PARK.get(leg)

    def segment(label: str, start: dict[str, float], stop: dict[str, float], step: float) -> int:
        return base.sample_segment(
            label=label,
            q_start=start,
            q_stop=stop,
            step_deg=step,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes,
            fixture_margin=fixture_margin,
        )

    if parked_leg is not None:
        target = dict(q)
        target[base.joint_name(parked_leg, "upper_leg")] = 30.0 * base.DEG
        samples += segment(f"{leg}: rear parking", q, target, 0.5)
        q = target

    for side_name, limit in (("MIN", upper.lower), ("MAX", upper.upper)):
        target = dict(q)
        target[upper_name] = limit
        samples += segment(f"{leg}: UPPER {side_name}", q, target, 1.0)
        samples += segment(f"{leg}: UPPER {side_name} return", target, q, 1.0)

    upper_pose = dict(q)
    upper_pose[upper_name] = upper_horizontal
    samples += segment(f"{leg}: UPPER to horizontal", q, upper_pose, 0.5)
    q = upper_pose

    for side_name, limit in (("MIN", lower.lower), ("MAX", lower.upper)):
        target = dict(q)
        target[lower_name] = limit
        samples += segment(f"{leg}: LOWER {side_name}", q, target, 1.0)
        samples += segment(f"{leg}: LOWER {side_name} return", target, q, 1.0)

    rejection_reasons: list[str] = []
    selected: dict[str, float] | None = None
    selected_samples = 0
    selected_end_pose: dict[str, float] | None = None

    for candidate in compact_candidates(
        leg,
        upper_horizontal,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        ignored_pairs=ignored_pairs,
    ):
        compact_pose = dict(q)
        compact_pose[lower_name] = candidate["value_rad"]
        candidate_samples = 0
        try:
            candidate_samples += segment(
                f"{leg}: LOWER to compact {candidate['value_deg']:.2f} deg",
                q,
                compact_pose,
                0.5,
            )
            for side_name, limit in (("MIN", hip.lower), ("MAX", hip.upper)):
                target = dict(compact_pose)
                target[hip_name] = limit
                candidate_samples += segment(
                    f"{leg}: HIP {side_name} at compact LOWER",
                    compact_pose,
                    target,
                    1.0,
                )
                candidate_samples += segment(
                    f"{leg}: HIP {side_name} return",
                    target,
                    compact_pose,
                    1.0,
                )
        except base.AuditFailure as error:
            rejection_reasons.append(
                f"lower={candidate['value_deg']:.2f} deg: {error}"
            )
            continue
        selected = candidate
        selected_samples = candidate_samples
        selected_end_pose = compact_pose
        break

    if selected is None or selected_end_pose is None:
        preview = rejection_reasons[:10]
        raise base.AuditFailure(
            f"{leg}: no compact LOWER candidate supports complete HIP sweep; "
            f"first_rejections={preview}"
        )

    samples += selected_samples
    q = selected_end_pose

    lower_home = dict(q)
    lower_home[lower_name] = 0.0
    samples += segment(f"{leg}: LOWER compact to home", q, lower_home, 0.5)
    q = lower_home

    upper_home = dict(q)
    upper_home[upper_name] = 0.0
    samples += segment(f"{leg}: UPPER horizontal to home", q, upper_home, 0.5)
    q = upper_home

    if parked_leg is not None:
        rear_home = dict(q)
        rear_home[base.joint_name(parked_leg, "upper_leg")] = 0.0
        samples += segment(f"{leg}: rear parking return", q, rear_home, 0.5)
        q = rear_home

    return {
        "leg": leg.upper(),
        "upper_horizontal_rad": upper_horizontal,
        "upper_horizontal_deg": upper_horizontal / base.DEG,
        "upper_horizontal_error": upper_error,
        "lower_compact": selected,
        "rejected_compact_candidates_before_selection": len(rejection_reasons),
        "sample_count": samples,
        "final_home": all(abs(value) < 1e-12 for value in q.values()),
    }


def main() -> int:
    args = base.parse_args()
    repo_root = args.repo_root.resolve()
    fixture_boxes, fixture_margin = base.load_fixture(args.fixture)
    if args.certify_hardware and not fixture_boxes:
        raise base.AuditFailure(
            "hardware certification requires --fixture with measured non-placeholder keep-outs"
        )

    root_link, joints, geometries = base.load_model(repo_root / base.URDF_RELATIVE)
    ignored_pairs = base.adjacent_link_pairs(joints)
    results = [
        audit_leg_v2(
            leg,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            ignored_pairs=ignored_pairs,
            fixture_boxes=fixture_boxes,
            fixture_margin=fixture_margin,
        )
        for leg in base.LEGS
    ]
    payload = {
        "result": "PASS",
        "algorithm": "ordered-upper-lower-hip-mesh-envelope-v2",
        "hardware_certified": bool(args.certify_hardware),
        "fixture_boxes": [box.name for box in fixture_boxes],
        "fixture_margin_m": fixture_margin,
        "model_limit_margin_deg": MODEL_LIMIT_MARGIN_DEG,
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
    except base.AuditFailure as error:
        print(f"HARD BLOCK: {error}", file=sys.stderr)
        raise SystemExit(2)
