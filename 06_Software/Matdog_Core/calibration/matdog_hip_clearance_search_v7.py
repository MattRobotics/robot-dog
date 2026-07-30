#!/usr/bin/env python3
"""Search coordinated UPPER+LOWER prerequisites for the full HIP sweep.

This is an offline exact-mesh search. It does not change the ordered contract:
UPPER contacts are measured first, LOWER contacts second, HIP last. It answers
whether the HIP phase can use a compact coordinated prerequisite near the
horizontal pose or whether one/both HIP sides are collision-limited and must
not be contact-probed.
"""

from __future__ import annotations

import argparse
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


HERE = Path(__file__).resolve().parent
V6_PATH = HERE / "matdog_endstop_sequence_geometry_audit_v6.py"
SPEC = importlib.util.spec_from_file_location("matdog_sequence_audit_v6_search", V6_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V6 audit: {V6_PATH}")
v6 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v6
SPEC.loader.exec_module(v6)
v5 = v6.v5
v2 = v5.v2
base = v2.base

COARSE_STEP_DEG = 2.5
HIP_COARSE_STEP_DEG = 2.5
REFINE_STEP_DEG = 0.5
UPPER_SEARCH_RADIUS_DEG = 35.0
MODEL_MARGIN_DEG = 5.0


def ordered_offsets(radius: float, step: float) -> list[float]:
    values = [0.0]
    count = int(math.floor(radius / step))
    for index in range(1, count + 1):
        values.extend((index * step, -index * step))
    return values


def inclusive_values(start: float, stop: float, step: float) -> list[float]:
    count = int(math.floor((stop - start) / step + 1e-9))
    values = [start + index * step for index in range(count + 1)]
    if not values or values[-1] < stop - 1e-9:
        values.append(stop)
    return values


def pose_collisions(
    q: dict[str, float],
    *,
    root_link: str,
    joints: dict[str, Any],
    checker: Any,
) -> set[tuple[str, str]]:
    transforms = base.link_transforms(root_link, joints, q)
    return checker.check(transforms)


def sampled_joint_path(
    q_start: dict[str, float],
    joint_name: str,
    target: float,
    step_deg: float,
) -> Iterable[dict[str, float]]:
    start = q_start[joint_name]
    distance_deg = abs(target - start) / base.DEG
    count = max(1, int(math.ceil(distance_deg / step_deg)))
    for index in range(1, count + 1):
        q = dict(q_start)
        q[joint_name] = start + (target - start) * index / count
        yield q


def path_is_clear(
    q_start: dict[str, float],
    moves: list[tuple[str, float, float]],
    *,
    root_link: str,
    joints: dict[str, Any],
    checker: Any,
) -> tuple[bool, dict[str, float], tuple[str, set[tuple[str, str]]] | None]:
    q = dict(q_start)
    for joint_name, target, step_deg in moves:
        for sample in sampled_joint_path(q, joint_name, target, step_deg):
            collisions = pose_collisions(
                sample,
                root_link=root_link,
                joints=joints,
                checker=checker,
            )
            if collisions:
                return False, q, (joint_name, collisions)
            q = sample
    return True, q, None


def hip_sweep_is_clear(
    q_clearance: dict[str, float],
    hip_name: str,
    hip_min: float,
    hip_max: float,
    step_deg: float,
    *,
    root_link: str,
    joints: dict[str, Any],
    checker: Any,
) -> tuple[bool, dict[str, Any] | None]:
    values = inclusive_values(hip_min / base.DEG, hip_max / base.DEG, step_deg)
    for hip_deg in values:
        q = dict(q_clearance)
        q[hip_name] = hip_deg * base.DEG
        collisions = pose_collisions(
            q,
            root_link=root_link,
            joints=joints,
            checker=checker,
        )
        if collisions:
            return False, {
                "hip_deg": hip_deg,
                "collisions": [list(pair) for pair in sorted(collisions)],
            }
    return True, None


def safe_hip_interval(
    q_clearance: dict[str, float],
    hip_name: str,
    hip_min: float,
    hip_max: float,
    *,
    root_link: str,
    joints: dict[str, Any],
    checker: Any,
) -> dict[str, Any]:
    negative_safe = 0.0
    positive_safe = 0.0
    negative_collision = None
    positive_collision = None

    for sign, limit, key in ((-1, hip_min, "negative"), (1, hip_max, "positive")):
        maximum_deg = abs(limit / base.DEG)
        safe = 0.0
        first_collision = None
        for magnitude in inclusive_values(0.0, maximum_deg, REFINE_STEP_DEG)[1:]:
            hip_deg = sign * magnitude
            q = dict(q_clearance)
            q[hip_name] = hip_deg * base.DEG
            collisions = pose_collisions(
                q,
                root_link=root_link,
                joints=joints,
                checker=checker,
            )
            if collisions:
                first_collision = {
                    "hip_deg": hip_deg,
                    "collisions": [list(pair) for pair in sorted(collisions)],
                }
                break
            safe = magnitude
        if key == "negative":
            negative_safe = -safe
            negative_collision = first_collision
        else:
            positive_safe = safe
            positive_collision = first_collision

    return {
        "safe_min_deg": negative_safe,
        "safe_max_deg": positive_safe,
        "first_negative_collision": negative_collision,
        "first_positive_collision": positive_collision,
    }


def search_leg(
    leg: str,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    ignored_pairs: set[frozenset[str]],
    home_transforms: dict[str, np.ndarray],
) -> dict[str, Any]:
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    hip_name = base.joint_name(leg, "hip")
    upper = joints[upper_name]
    lower = joints[lower_name]
    hip = joints[hip_name]
    assert upper.lower is not None and upper.upper is not None
    assert lower.lower is not None and lower.upper is not None
    assert hip.lower is not None and hip.upper is not None

    active_links = v5.moving_links_for_leg(leg, geometries)
    checker = v5.PairFilteredExactChecker(
        geometries,
        active_links,
        ignored_pairs,
        home_transforms,
    )
    upper_horizontal, upper_horizontal_error = base.derive_horizontal_upper(
        leg,
        root_link,
        joints,
    )

    upper_min = max(
        upper.lower + MODEL_MARGIN_DEG * base.DEG,
        upper_horizontal - UPPER_SEARCH_RADIUS_DEG * base.DEG,
    )
    upper_max = min(
        upper.upper - MODEL_MARGIN_DEG * base.DEG,
        upper_horizontal + UPPER_SEARCH_RADIUS_DEG * base.DEG,
    )
    lower_min = lower.lower + MODEL_MARGIN_DEG * base.DEG
    lower_max = lower.upper - MODEL_MARGIN_DEG * base.DEG

    upper_candidates: list[float] = []
    for offset_deg in ordered_offsets(UPPER_SEARCH_RADIUS_DEG, COARSE_STEP_DEG):
        candidate = upper_horizontal + offset_deg * base.DEG
        if upper_min <= candidate <= upper_max:
            upper_candidates.append(candidate)
    lower_candidates = [
        value * base.DEG
        for value in inclusive_values(
            lower_min / base.DEG,
            lower_max / base.DEG,
            COARSE_STEP_DEG,
        )
    ]
    lower_candidates.sort()

    q_home = base.base_pose()
    parked = base.FRONT_REAR_PARK.get(leg)
    if parked is not None:
        q_home[base.joint_name(parked, "upper_leg")] = 30.0 * base.DEG

    candidate_failures: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None

    for upper_value, lower_value in itertools.product(
        upper_candidates,
        lower_candidates,
    ):
        moves = [
            (upper_name, upper_value, COARSE_STEP_DEG),
            (lower_name, lower_value, COARSE_STEP_DEG),
        ]
        clear_transition, q_clearance, transition_failure = path_is_clear(
            q_home,
            moves,
            root_link=root_link,
            joints=joints,
            checker=checker,
        )
        if not clear_transition:
            if len(candidate_failures) < 20:
                candidate_failures.append(
                    {
                        "upper_deg": upper_value / base.DEG,
                        "lower_deg": lower_value / base.DEG,
                        "stage": "transition",
                        "failure": {
                            "joint": transition_failure[0],
                            "collisions": [
                                list(pair) for pair in sorted(transition_failure[1])
                            ],
                        },
                    }
                )
            continue

        clear_sweep, sweep_failure = hip_sweep_is_clear(
            q_clearance,
            hip_name,
            hip.lower,
            hip.upper,
            HIP_COARSE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            checker=checker,
        )
        if not clear_sweep:
            if len(candidate_failures) < 20:
                candidate_failures.append(
                    {
                        "upper_deg": upper_value / base.DEG,
                        "lower_deg": lower_value / base.DEG,
                        "stage": "hip_sweep",
                        "failure": sweep_failure,
                    }
                )
            continue

        # Refine the complete transition and HIP sweep before accepting.
        clear_transition, q_clearance, transition_failure = path_is_clear(
            q_home,
            [
                (upper_name, upper_value, REFINE_STEP_DEG),
                (lower_name, lower_value, REFINE_STEP_DEG),
            ],
            root_link=root_link,
            joints=joints,
            checker=checker,
        )
        if not clear_transition:
            continue
        clear_sweep, sweep_failure = hip_sweep_is_clear(
            q_clearance,
            hip_name,
            hip.lower,
            hip.upper,
            REFINE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            checker=checker,
        )
        if not clear_sweep:
            continue

        selected = {
            "upper_rad": upper_value,
            "upper_deg": upper_value / base.DEG,
            "lower_rad": lower_value,
            "lower_deg": lower_value / base.DEG,
            "upper_deviation_from_horizontal_deg": abs(
                upper_value - upper_horizontal
            )
            / base.DEG,
            "full_hip_min_deg": hip.lower / base.DEG,
            "full_hip_max_deg": hip.upper / base.DEG,
        }
        break

    fallback_upper = upper_horizontal
    fallback_lower = lower_min
    fallback_q = dict(q_home)
    fallback_q[upper_name] = fallback_upper
    fallback_q[lower_name] = fallback_lower
    fallback_interval = safe_hip_interval(
        fallback_q,
        hip_name,
        hip.lower,
        hip.upper,
        root_link=root_link,
        joints=joints,
        checker=checker,
    )

    return {
        "leg": leg.upper(),
        "upper_horizontal_deg": upper_horizontal / base.DEG,
        "upper_horizontal_error": upper_horizontal_error,
        "coarse_upper_candidate_count": len(upper_candidates),
        "coarse_lower_candidate_count": len(lower_candidates),
        "full_range_clearance": selected,
        "fallback_horizontal_compact": {
            "upper_deg": fallback_upper / base.DEG,
            "lower_deg": fallback_lower / base.DEG,
            "safe_hip_interval": fallback_interval,
        },
        "first_candidate_failures": candidate_failures,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    root_link, joints, geometries = base.load_model(
        repo_root / base.URDF_RELATIVE
    )
    ignored_pairs = base.adjacent_link_pairs(joints)
    home_transforms = base.link_transforms(
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

    results = [
        search_leg(
            leg,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            ignored_pairs=ignored_pairs,
            home_transforms=home_transforms,
        )
        for leg in base.LEGS
    ]
    payload = {
        "result": "PASS",
        "algorithm": "coordinated-upper-lower-hip-clearance-v7",
        "order_unchanged": ["UPPER_MIN_MAX", "LOWER_MIN_MAX", "HIP_MIN_MAX"],
        "legs": results,
        "all_legs_have_full_hip_clearance": all(
            result["full_range_clearance"] is not None for result in results
        ),
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
