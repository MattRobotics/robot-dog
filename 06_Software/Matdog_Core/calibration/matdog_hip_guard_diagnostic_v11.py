#!/usr/bin/env python3
"""Two-stage exact HIP guard budget diagnostic.

Stage 1 scans every coordinated UPPER+LOWER candidate at 2.5 degrees.
Stage 2 re-evaluates the best common/MIN/MAX candidates at 0.25 degrees using
V10's exact-mesh evaluator. The fixture remains excluded by the signed support
contract and the table remains at Z=-0.180 m.
"""

from __future__ import annotations

import argparse
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
V10_PATH = HERE / "matdog_hip_guard_diagnostic_v10.py"
SPEC = importlib.util.spec_from_file_location("matdog_guard_diagnostic_v10_fast", V10_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V10 diagnostic: {V10_PATH}")
v10 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v10
SPEC.loader.exec_module(v10)

v9 = v10.v9
v8 = v10.v8
v5 = v10.v5
base = v10.base

COARSE_HIP_STEP_DEG = 2.5
REFINE_CANDIDATE_COUNT = 18


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def coarse_direction_interval(
    q_clearance: dict[str, float],
    hip_name: str,
    sign: int,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
) -> tuple[float, int]:
    guard_limit = v10.URDF_HIP_LIMIT_DEG + v8.GUARD_DEG
    safe_deg = 0.0
    for magnitude in v10.inclusive_values(
        0.0,
        guard_limit,
        COARSE_HIP_STEP_DEG,
    )[1:]:
        hip_deg = sign * magnitude
        q = dict(q_clearance)
        q[hip_name] = hip_deg * base.DEG
        clear, _, _, _ = v10.pose_state(
            q,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        if not clear:
            break
        safe_deg = hip_deg
    extra_deg = max(0.0, abs(safe_deg) - v10.URDF_HIP_LIMIT_DEG)
    extra_ticks = int(math.floor(extra_deg * v10.TICKS_PER_DEGREE + 1e-9))
    return safe_deg, extra_ticks


def coarse_candidate(
    leg: str,
    upper_deg: float,
    lower_deg: float,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
    q_start: dict[str, float],
) -> dict[str, float] | None:
    hip_name = base.joint_name(leg, "hip")
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    clear, q_clearance, _, _, _ = v10.transition_state(
        q_start,
        [
            (upper_name, upper_deg * base.DEG),
            (lower_name, lower_deg * base.DEG),
        ],
        step_deg=v10.SEARCH_STEP_DEG,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )
    if not clear:
        return None
    safe_min_deg, min_extra_ticks = coarse_direction_interval(
        q_clearance,
        hip_name,
        -1,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )
    safe_max_deg, max_extra_ticks = coarse_direction_interval(
        q_clearance,
        hip_name,
        1,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )
    return {
        "upper_deg": upper_deg,
        "lower_deg": lower_deg,
        "safe_min_deg": safe_min_deg,
        "safe_max_deg": safe_max_deg,
        "min_extra_ticks": float(min_extra_ticks),
        "max_extra_ticks": float(max_extra_ticks),
        "common_extra_ticks": float(min(min_extra_ticks, max_extra_ticks)),
        "total_extra_ticks": float(min_extra_ticks + max_extra_ticks),
    }


def select_refine_keys(
    coarse: list[dict[str, float]],
    horizontal_deg: float,
) -> list[tuple[float, float]]:
    common = sorted(
        coarse,
        key=lambda item: (
            -item["common_extra_ticks"],
            -item["total_extra_ticks"],
            abs(item["upper_deg"] - horizontal_deg),
            abs(item["lower_deg"] + 87.0),
        ),
    )[:REFINE_CANDIDATE_COUNT]
    minimum = sorted(
        coarse,
        key=lambda item: (
            -item["min_extra_ticks"],
            abs(item["upper_deg"] - horizontal_deg),
            abs(item["lower_deg"] + 87.0),
            -item["max_extra_ticks"],
        ),
    )[:REFINE_CANDIDATE_COUNT]
    maximum = sorted(
        coarse,
        key=lambda item: (
            -item["max_extra_ticks"],
            abs(item["upper_deg"] - horizontal_deg),
            abs(item["lower_deg"] + 87.0),
            -item["min_extra_ticks"],
        ),
    )[:REFINE_CANDIDATE_COUNT]
    keys = {
        (item["upper_deg"], item["lower_deg"])
        for item in common + minimum + maximum
    }
    return sorted(keys)


def audit_leg(
    leg: str,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    ignored_pairs: set[frozenset[str]],
    home_transforms: dict[str, Any],
    table_z: float,
) -> dict[str, Any]:
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    upper = joints[upper_name]
    lower = joints[lower_name]
    assert upper.lower is not None and upper.upper is not None
    assert lower.lower is not None and lower.upper is not None

    checker = v5.PairFilteredExactChecker(
        geometries,
        v5.moving_links_for_leg(leg, geometries),
        ignored_pairs,
        home_transforms,
    )
    q_start = base.base_pose()
    parked = base.FRONT_REAR_PARK.get(leg)
    if parked is not None:
        q_start[base.joint_name(parked, "upper_leg")] = 30.0 * base.DEG

    horizontal_rad, horizontal_error = base.derive_horizontal_upper(
        leg,
        root_link,
        joints,
    )
    horizontal_deg = horizontal_rad / base.DEG
    upper_min = max(
        upper.lower / base.DEG + v10.MODEL_MARGIN_DEG,
        v10.UPPER_SEARCH_MIN_DEG,
    )
    upper_max = min(
        upper.upper / base.DEG - v10.MODEL_MARGIN_DEG,
        v10.UPPER_SEARCH_MAX_DEG,
    )
    lower_min = lower.lower / base.DEG + v10.MODEL_MARGIN_DEG
    lower_max = lower.upper / base.DEG - v10.MODEL_MARGIN_DEG

    upper_values = v10.inclusive_values(
        upper_min,
        upper_max,
        v10.SEARCH_STEP_DEG,
    )
    upper_values.sort(key=lambda value: (abs(value - horizontal_deg), value))
    lower_values = v10.inclusive_values(
        lower_min,
        lower_max,
        v10.SEARCH_STEP_DEG,
    )

    coarse: list[dict[str, float]] = []
    rejected_transitions = 0
    for upper_deg, lower_deg in itertools.product(upper_values, lower_values):
        item = coarse_candidate(
            leg,
            upper_deg,
            lower_deg,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
            q_start=q_start,
        )
        if item is None:
            rejected_transitions += 1
        else:
            coarse.append(item)
    if not coarse:
        raise base.AuditFailure(f"{leg}: no coarse candidate survives transition")

    refine_keys = select_refine_keys(coarse, horizontal_deg)
    reference_key = (87.5 if leg in ("lf", "rf") else 90.0, -87.0)
    refine_keys.append(reference_key)
    refined: list[v10.PoseEvaluation] = []
    for upper_deg, lower_deg in sorted(set(refine_keys)):
        item = v10.evaluate_candidate(
            leg,
            upper_deg,
            lower_deg,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
            q_start=q_start,
        )
        if item is not None:
            refined.append(item)
    if not refined:
        raise base.AuditFailure(f"{leg}: no refined candidate survives")

    common_sorted = sorted(
        refined,
        key=lambda item: (
            -item.common_extra_ticks,
            -item.total_extra_ticks,
            abs(item.upper_deg - horizontal_deg),
            abs(item.lower_deg + 87.0),
        ),
    )
    min_sorted = sorted(
        refined,
        key=lambda item: (
            -item.min_extra_ticks,
            abs(item.upper_deg - horizontal_deg),
            abs(item.lower_deg + 87.0),
            -item.max_extra_ticks,
        ),
    )
    max_sorted = sorted(
        refined,
        key=lambda item: (
            -item.max_extra_ticks,
            abs(item.upper_deg - horizontal_deg),
            abs(item.lower_deg + 87.0),
            -item.min_extra_ticks,
        ),
    )
    reference = next(
        (
            item
            for item in refined
            if abs(item.upper_deg - reference_key[0]) < 1e-9
            and abs(item.lower_deg - reference_key[1]) < 1e-9
        ),
        None,
    )
    best_common = common_sorted[0]
    return {
        "leg": leg.upper(),
        "upper_horizontal_deg": horizontal_deg,
        "upper_horizontal_error": horizontal_error,
        "coarse_candidate_count": len(coarse),
        "rejected_transition_count": rejected_transitions,
        "refined_candidate_count": len(refined),
        "full_uniform_64_tick_guard_possible": best_common.common_extra_ticks >= 64,
        "best_single_pose_both_directions": v10.serialize_evaluation(best_common),
        "best_pose_for_min": v10.serialize_evaluation(min_sorted[0]),
        "best_pose_for_max": v10.serialize_evaluation(max_sorted[0]),
        "reference_pose": (
            v10.serialize_evaluation(reference) if reference is not None else None
        ),
        "top_refined_common_candidates": [
            v10.serialize_evaluation(item)
            for item in common_sorted[:v10.TOP_RESULT_COUNT]
        ],
    }


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    support = v8.load_support_contract(repo_root)
    table_z = float(support["table_plane_z_m"])
    root_link, joints, geometries = base.load_model(repo_root / base.URDF_RELATIVE)
    ignored_pairs = base.adjacent_link_pairs(joints)
    home_transforms = v9.guarded_link_transforms(
        root_link,
        joints,
        base.base_pose(),
    )
    home_collisions = v5.full_home_check(
        geometries,
        ignored_pairs,
        home_transforms,
    )
    if home_collisions:
        raise base.AuditFailure(
            f"canonical home has non-adjacent collisions: {sorted(home_collisions)}"
        )

    legs = [
        audit_leg(
            leg,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            ignored_pairs=ignored_pairs,
            home_transforms=home_transforms,
            table_z=table_z,
        )
        for leg in base.LEGS
    ]
    payload = {
        "result": "PASS",
        "algorithm": "hip-guard-budget-diagnostic-v11-two-stage",
        "hardware_authorized": False,
        "fixture_geometry_modeled": False,
        "fixture_exclusion_basis": "operator_validated_impossible",
        "table_plane_z_m": table_z,
        "nominal_hip_limit_deg": v10.URDF_HIP_LIMIT_DEG,
        "requested_guard_ticks": v8.GUARD_TICKS,
        "requested_guard_deg": v8.GUARD_DEG,
        "coarse_hip_step_deg": COARSE_HIP_STEP_DEG,
        "refine_hip_step_deg": v10.REFINE_HIP_STEP_DEG,
        "collision_margin_ticks": v10.COLLISION_MARGIN_TICKS,
        "legs": legs,
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
