#!/usr/bin/env python3
"""Diagnose collision-safe HIP guard budgets for the ordered MATDOG sequence.

The external calibration support is excluded by the operator-validated support
contract. The table remains a plane at Z=-0.180 m. This diagnostic searches
coordinated UPPER+LOWER prerequisites and reports:

- the best single prerequisite usable for both HIP directions;
- the best direction-specific prerequisite for HIP MIN and HIP MAX;
- the maximum safe HIP angle in each direction;
- safe ticks beyond the nominal +/-45 degree URDF limit;
- the first internal collision or table violation.

It is intentionally read-only and never opens Station or a serial device.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


HERE = Path(__file__).resolve().parent
V9_PATH = HERE / "matdog_guarded_support_audit_v9.py"
SPEC = importlib.util.spec_from_file_location("matdog_guard_diagnostic_v9", V9_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V9 audit: {V9_PATH}")
v9 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v9
SPEC.loader.exec_module(v9)
v8 = v9.v8
v7 = v8.v7
v5 = v8.v5
base = v8.base

TICKS_PER_DEGREE = v8.TICKS_PER_REVOLUTION / 360.0
URDF_HIP_LIMIT_DEG = 45.0
SEARCH_STEP_DEG = 2.5
REFINE_HIP_STEP_DEG = 0.25
MODEL_MARGIN_DEG = 5.0
UPPER_SEARCH_MIN_DEG = 55.0
UPPER_SEARCH_MAX_DEG = 120.0
TOP_RESULT_COUNT = 12
COLLISION_MARGIN_TICKS = 8


@dataclass(frozen=True)
class PoseEvaluation:
    upper_deg: float
    lower_deg: float
    safe_min_deg: float
    safe_max_deg: float
    min_extra_ticks: int
    max_extra_ticks: int
    min_first_failure: dict[str, Any] | None
    max_first_failure: dict[str, Any] | None
    minimum_table_clearance_m: float
    minimum_table_clearance_link: str

    @property
    def common_extra_ticks(self) -> int:
        return min(self.min_extra_ticks, self.max_extra_ticks)

    @property
    def total_extra_ticks(self) -> int:
        return self.min_extra_ticks + self.max_extra_ticks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def inclusive_values(start: float, stop: float, step: float) -> list[float]:
    if step <= 0:
        raise ValueError("step must be positive")
    count = int(math.floor((stop - start) / step + 1e-9))
    values = [start + index * step for index in range(count + 1)]
    if not values or values[-1] < stop - 1e-9:
        values.append(stop)
    return values


def transformed_bounds(
    geometry: Any,
    transforms: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    matrix = transforms[geometry.link] @ geometry.local_transform
    minimum, maximum = np.asarray(geometry.mesh.bounds, dtype=float)
    corners = np.asarray(list(itertools.product(*zip(minimum, maximum))), dtype=float)
    homogeneous = np.column_stack((corners, np.ones(len(corners))))
    world = (matrix @ homogeneous.T).T[:, :3]
    return world.min(axis=0), world.max(axis=0)


def table_state(
    transforms: dict[str, np.ndarray],
    geometries: list[Any],
    table_z: float,
) -> tuple[float, str, dict[str, Any] | None]:
    minimum_clearance = math.inf
    minimum_link = ""
    for geometry in geometries:
        world_minimum, _ = transformed_bounds(geometry, transforms)
        clearance = float(world_minimum[2]) - table_z
        if clearance < minimum_clearance:
            minimum_clearance = clearance
            minimum_link = geometry.link
    if minimum_clearance < -1e-9:
        return minimum_clearance, minimum_link, {
            "kind": "TABLE_COLLISION",
            "link": minimum_link,
            "clearance_m": minimum_clearance,
            "table_plane_z_m": table_z,
        }
    return minimum_clearance, minimum_link, None


def pose_state(
    q: dict[str, float],
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
) -> tuple[bool, float, str, dict[str, Any] | None]:
    transforms = v9.guarded_link_transforms(root_link, joints, q)
    collisions = checker.check(transforms)
    clearance, clearance_link, table_failure = table_state(
        transforms,
        geometries,
        table_z,
    )
    if collisions:
        return False, clearance, clearance_link, {
            "kind": "SELF_OR_CROSS_COLLISION",
            "pairs": [list(pair) for pair in sorted(collisions)],
        }
    if table_failure is not None:
        return False, clearance, clearance_link, table_failure
    return True, clearance, clearance_link, None


def sampled_joint_values(start: float, stop: float, step_deg: float) -> list[float]:
    count = max(1, int(math.ceil(abs(stop - start) / step_deg)))
    return [start + (stop - start) * index / count for index in range(1, count + 1)]


def transition_state(
    q_start: dict[str, float],
    moves: list[tuple[str, float]],
    *,
    step_deg: float,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
) -> tuple[bool, dict[str, float], float, str, dict[str, Any] | None]:
    q = dict(q_start)
    minimum_clearance = math.inf
    minimum_link = ""
    for joint_name, target_rad in moves:
        start_deg = q[joint_name] / base.DEG
        target_deg = target_rad / base.DEG
        for value_deg in sampled_joint_values(start_deg, target_deg, step_deg):
            q[joint_name] = value_deg * base.DEG
            clear, clearance, link, failure = pose_state(
                q,
                root_link=root_link,
                joints=joints,
                geometries=geometries,
                checker=checker,
                table_z=table_z,
            )
            if clearance < minimum_clearance:
                minimum_clearance = clearance
                minimum_link = link
            if not clear:
                detail = {
                    "stage": "PREREQUISITE_TRANSITION",
                    "joint": joint_name,
                    "joint_deg": value_deg,
                    "failure": failure,
                }
                return False, q, minimum_clearance, minimum_link, detail
    return True, q, minimum_clearance, minimum_link, None


def direction_interval(
    q_clearance: dict[str, float],
    hip_name: str,
    sign: int,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
) -> tuple[float, int, dict[str, Any] | None, float, str]:
    assert sign in (-1, 1)
    guard_limit_deg = sign * (URDF_HIP_LIMIT_DEG + v8.GUARD_DEG)
    values = inclusive_values(
        0.0,
        abs(guard_limit_deg),
        REFINE_HIP_STEP_DEG,
    )[1:]
    safe_deg = 0.0
    first_failure: dict[str, Any] | None = None
    minimum_clearance = math.inf
    minimum_link = ""
    for magnitude in values:
        hip_deg = sign * magnitude
        q = dict(q_clearance)
        q[hip_name] = hip_deg * base.DEG
        clear, clearance, link, failure = pose_state(
            q,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        if clearance < minimum_clearance:
            minimum_clearance = clearance
            minimum_link = link
        if not clear:
            first_failure = {
                "hip_deg": hip_deg,
                "failure": failure,
            }
            break
        safe_deg = hip_deg

    if sign < 0:
        extra_deg = max(0.0, abs(safe_deg) - URDF_HIP_LIMIT_DEG)
    else:
        extra_deg = max(0.0, safe_deg - URDF_HIP_LIMIT_DEG)
    extra_ticks = int(math.floor(extra_deg * TICKS_PER_DEGREE + 1e-9))
    return safe_deg, extra_ticks, first_failure, minimum_clearance, minimum_link


def evaluate_candidate(
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
) -> PoseEvaluation | None:
    hip_name = base.joint_name(leg, "hip")
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    clear, q_clearance, transition_clearance, transition_link, _ = transition_state(
        q_start,
        [
            (upper_name, upper_deg * base.DEG),
            (lower_name, lower_deg * base.DEG),
        ],
        step_deg=SEARCH_STEP_DEG,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )
    if not clear:
        return None

    (
        safe_min_deg,
        min_extra_ticks,
        min_failure,
        min_clearance,
        min_link,
    ) = direction_interval(
        q_clearance,
        hip_name,
        -1,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )
    (
        safe_max_deg,
        max_extra_ticks,
        max_failure,
        max_clearance,
        max_link,
    ) = direction_interval(
        q_clearance,
        hip_name,
        1,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )

    clearances = [
        (transition_clearance, transition_link),
        (min_clearance, min_link),
        (max_clearance, max_link),
    ]
    minimum_clearance, minimum_link = min(clearances, key=lambda item: item[0])
    return PoseEvaluation(
        upper_deg=upper_deg,
        lower_deg=lower_deg,
        safe_min_deg=safe_min_deg,
        safe_max_deg=safe_max_deg,
        min_extra_ticks=min_extra_ticks,
        max_extra_ticks=max_extra_ticks,
        min_first_failure=min_failure,
        max_first_failure=max_failure,
        minimum_table_clearance_m=minimum_clearance,
        minimum_table_clearance_link=minimum_link,
    )


def serialize_evaluation(item: PoseEvaluation) -> dict[str, Any]:
    return {
        "upper_deg": item.upper_deg,
        "lower_deg": item.lower_deg,
        "safe_hip_min_deg": item.safe_min_deg,
        "safe_hip_max_deg": item.safe_max_deg,
        "safe_extra_ticks_beyond_urdf_min": item.min_extra_ticks,
        "safe_extra_ticks_beyond_urdf_max": item.max_extra_ticks,
        "safe_common_extra_ticks": item.common_extra_ticks,
        "safe_total_extra_ticks": item.total_extra_ticks,
        "recommended_collision_margin_ticks": COLLISION_MARGIN_TICKS,
        "usable_extra_ticks_after_margin_min": max(
            0,
            item.min_extra_ticks - COLLISION_MARGIN_TICKS,
        ),
        "usable_extra_ticks_after_margin_max": max(
            0,
            item.max_extra_ticks - COLLISION_MARGIN_TICKS,
        ),
        "first_failure_min": item.min_first_failure,
        "first_failure_max": item.max_first_failure,
        "minimum_conservative_table_clearance_m": item.minimum_table_clearance_m,
        "minimum_table_clearance_link": item.minimum_table_clearance_link,
    }


def audit_leg(
    leg: str,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    ignored_pairs: set[frozenset[str]],
    home_transforms: dict[str, np.ndarray],
    table_z: float,
) -> dict[str, Any]:
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    upper = joints[upper_name]
    lower = joints[lower_name]
    assert upper.lower is not None and upper.upper is not None
    assert lower.lower is not None and lower.upper is not None

    active_links = v5.moving_links_for_leg(leg, geometries)
    checker = v5.PairFilteredExactChecker(
        geometries,
        active_links,
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

    upper_search_min = max(
        upper.lower / base.DEG + MODEL_MARGIN_DEG,
        UPPER_SEARCH_MIN_DEG,
    )
    upper_search_max = min(
        upper.upper / base.DEG - MODEL_MARGIN_DEG,
        UPPER_SEARCH_MAX_DEG,
    )
    lower_search_min = lower.lower / base.DEG + MODEL_MARGIN_DEG
    lower_search_max = lower.upper / base.DEG - MODEL_MARGIN_DEG

    upper_values = inclusive_values(
        upper_search_min,
        upper_search_max,
        SEARCH_STEP_DEG,
    )
    upper_values.sort(key=lambda value: (abs(value - horizontal_deg), value))
    lower_values = inclusive_values(
        lower_search_min,
        lower_search_max,
        SEARCH_STEP_DEG,
    )
    lower_values.sort()

    evaluations: list[PoseEvaluation] = []
    rejected_transition_count = 0
    for upper_deg, lower_deg in itertools.product(upper_values, lower_values):
        result = evaluate_candidate(
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
        if result is None:
            rejected_transition_count += 1
            continue
        evaluations.append(result)

    if not evaluations:
        raise base.AuditFailure(f"{leg}: no valid coordinated prerequisites")

    common_sorted = sorted(
        evaluations,
        key=lambda item: (
            -item.common_extra_ticks,
            -item.total_extra_ticks,
            abs(item.upper_deg - horizontal_deg),
            abs(item.lower_deg + 87.0),
        ),
    )
    min_sorted = sorted(
        evaluations,
        key=lambda item: (
            -item.min_extra_ticks,
            abs(item.upper_deg - horizontal_deg),
            abs(item.lower_deg + 87.0),
            -item.max_extra_ticks,
        ),
    )
    max_sorted = sorted(
        evaluations,
        key=lambda item: (
            -item.max_extra_ticks,
            abs(item.upper_deg - horizontal_deg),
            abs(item.lower_deg + 87.0),
            -item.min_extra_ticks,
        ),
    )

    reference = evaluate_candidate(
        leg,
        87.5 if leg in ("lf", "rf") else 90.0,
        -87.0,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
        q_start=q_start,
    )

    best_common = common_sorted[0]
    return {
        "leg": leg.upper(),
        "upper_horizontal_deg": horizontal_deg,
        "upper_horizontal_error": horizontal_error,
        "candidate_count": len(evaluations),
        "rejected_transition_count": rejected_transition_count,
        "search_upper_deg": [upper_search_min, upper_search_max],
        "search_lower_deg": [lower_search_min, lower_search_max],
        "full_uniform_64_tick_guard_possible": best_common.common_extra_ticks >= 64,
        "best_single_pose_both_directions": serialize_evaluation(best_common),
        "best_pose_for_min": serialize_evaluation(min_sorted[0]),
        "best_pose_for_max": serialize_evaluation(max_sorted[0]),
        "reference_pose": (
            serialize_evaluation(reference) if reference is not None else None
        ),
        "top_common_pose_candidates": [
            serialize_evaluation(item)
            for item in common_sorted[:TOP_RESULT_COUNT]
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
        "algorithm": "hip-guard-budget-diagnostic-v10",
        "hardware_authorized": False,
        "fixture_geometry_modeled": False,
        "fixture_exclusion_basis": "operator_validated_impossible",
        "table_plane_z_m": table_z,
        "nominal_hip_limit_deg": URDF_HIP_LIMIT_DEG,
        "requested_guard_ticks": v8.GUARD_TICKS,
        "requested_guard_deg": v8.GUARD_DEG,
        "collision_margin_ticks": COLLISION_MARGIN_TICKS,
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
