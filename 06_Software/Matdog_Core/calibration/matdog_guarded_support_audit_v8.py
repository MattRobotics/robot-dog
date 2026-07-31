#!/usr/bin/env python3
"""Guard-aware ordered MATDOG audit with a 180 mm table plane.

The external support fixture is deliberately excluded from geometry because the
operator has validated it experimentally. This audit still proves:

- mandatory UPPER -> LOWER -> HIP order;
- exact-mesh MATDOG self/cross collision freedom;
- complete software guard corridors, not only URDF limits;
- clearance from the table plane at Z=-0.180 m;
- coordinated UPPER+LOWER prerequisites for HIP.

No hardware is opened or commanded.
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
import yaml


HERE = Path(__file__).resolve().parent
V7_PATH = HERE / "matdog_hip_clearance_search_v7.py"
SPEC = importlib.util.spec_from_file_location("matdog_guarded_v7", V7_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V7 search: {V7_PATH}")
v7 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v7
SPEC.loader.exec_module(v7)
v6 = v7.v6
v5 = v7.v5
v2 = v7.v2
base = v7.base

SUPPORT_RELATIVE = Path(
    "06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_SUPPORT_CONTRACT.yaml"
)
GUARD_TICKS = 64
TICKS_PER_REVOLUTION = 4096
GUARD_DEG = GUARD_TICKS * 360.0 / TICKS_PER_REVOLUTION
GUARD_RAD = GUARD_DEG * base.DEG
COARSE_STEP_DEG = 2.5
REFINE_STEP_DEG = 0.5
MODEL_MARGIN_DEG = 5.0
UPPER_SEARCH_RADIUS_DEG = 35.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def load_support_contract(repo_root: Path) -> dict[str, Any]:
    path = repo_root / SUPPORT_RELATIVE
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise base.AuditFailure("support contract schema_version must be 1")
    if payload.get("frame") != "base_link":
        raise base.AuditFailure("support contract frame must be base_link")
    if payload.get("table_plane_z_m") != -0.180:
        raise base.AuditFailure("table plane must be exactly Z=-0.180 m")
    fixture = payload.get("support_fixture") or {}
    if fixture.get("geometry_modeled") is not False:
        raise base.AuditFailure("fixture geometry must remain explicitly unmodelled")
    if fixture.get("interference_status") != "operator_validated_impossible":
        raise base.AuditFailure("fixture exclusion lacks operator validation")
    if payload.get("fixture_exclusion_authorized") is not True:
        raise base.AuditFailure("fixture exclusion is not authorized")
    expected_order = [
        "UPPER_MIN_MAX",
        "UPPER_HORIZONTAL",
        "LOWER_MIN_MAX",
        "COORDINATED_UPPER_LOWER_COMPACT",
        "HIP_MIN_MAX",
    ]
    if fixture.get("required_sequence") != expected_order:
        raise base.AuditFailure("support contract sequence differs from canonical order")
    return payload


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


def table_clearance(
    transforms: dict[str, np.ndarray],
    geometries: list[Any],
    table_z: float,
) -> tuple[float, str]:
    minimum_z = math.inf
    minimum_link = ""
    for geometry in geometries:
        world_minimum, _ = transformed_bounds(geometry, transforms)
        if float(world_minimum[2]) < minimum_z:
            minimum_z = float(world_minimum[2])
            minimum_link = geometry.link
    if minimum_z < table_z - 1e-9:
        raise base.AuditFailure(
            f"table collision: {minimum_link} conservative_z={minimum_z:.6f} "
            f"below table_z={table_z:.6f}"
        )
    return minimum_z - table_z, minimum_link


def pose_check(
    q: dict[str, float],
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
) -> tuple[float, str]:
    transforms = base.link_transforms(root_link, joints, q)
    collisions = checker.check(transforms)
    if collisions:
        raise base.AuditFailure(f"self/cross collision: {sorted(collisions)}")
    return table_clearance(transforms, geometries, table_z)


def sampled_joint_path(
    q_start: dict[str, float],
    joint_name: str,
    target: float,
    step_deg: float,
) -> Iterable[dict[str, float]]:
    start = q_start[joint_name]
    count = max(1, int(math.ceil(abs(target - start) / base.DEG / step_deg)))
    for index in range(1, count + 1):
        q = dict(q_start)
        q[joint_name] = start + (target - start) * index / count
        yield q


def move_and_check(
    q_start: dict[str, float],
    joint_name: str,
    target: float,
    step_deg: float,
    *,
    root_link: str,
    joints: dict[str, Any],
    geometries: list[Any],
    checker: Any,
    table_z: float,
) -> tuple[dict[str, float], float, str, int]:
    q = dict(q_start)
    minimum_clearance = math.inf
    minimum_link = ""
    samples = 0
    for sample in sampled_joint_path(q, joint_name, target, step_deg):
        clearance, link = pose_check(
            sample,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        samples += 1
        if clearance < minimum_clearance:
            minimum_clearance = clearance
            minimum_link = link
        q = sample
    return q, minimum_clearance, minimum_link, samples


def guarded_limits(joint: Any) -> tuple[float, float]:
    assert joint.lower is not None and joint.upper is not None
    return joint.lower - GUARD_RAD, joint.upper + GUARD_RAD


def candidate_values(start: float, stop: float, step_deg: float) -> list[float]:
    count = int(math.floor((stop - start) / (step_deg * base.DEG) + 1e-9))
    values = [start + index * step_deg * base.DEG for index in range(count + 1)]
    if not values or values[-1] < stop - 1e-9:
        values.append(stop)
    return values


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
    hip_name = base.joint_name(leg, "hip")
    upper_name = base.joint_name(leg, "upper_leg")
    lower_name = base.joint_name(leg, "lower_leg")
    hip = joints[hip_name]
    upper = joints[upper_name]
    lower = joints[lower_name]
    assert hip.lower is not None and hip.upper is not None
    assert upper.lower is not None and upper.upper is not None
    assert lower.lower is not None and lower.upper is not None

    active_links = v5.moving_links_for_leg(leg, geometries)
    checker = v5.PairFilteredExactChecker(
        geometries,
        active_links,
        ignored_pairs,
        home_transforms,
    )
    q = base.base_pose()
    parked = base.FRONT_REAR_PARK.get(leg)
    if parked is not None:
        q, _, _, _ = move_and_check(
            q,
            base.joint_name(parked, "upper_leg"),
            30.0 * base.DEG,
            REFINE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )

    minimum_clearance = math.inf
    minimum_link = ""
    total_samples = 0

    # UPPER calibration corridors include the full 64-tick software guard.
    for target in guarded_limits(upper):
        q_target, clearance, link, samples = move_and_check(
            q,
            upper_name,
            target,
            REFINE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        total_samples += samples
        if clearance < minimum_clearance:
            minimum_clearance, minimum_link = clearance, link
        _, clearance, link, samples = move_and_check(
            q_target,
            upper_name,
            q[upper_name],
            REFINE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        total_samples += samples
        if clearance < minimum_clearance:
            minimum_clearance, minimum_link = clearance, link

    upper_horizontal, horizontal_error = base.derive_horizontal_upper(
        leg, root_link, joints
    )
    q_horizontal, clearance, link, samples = move_and_check(
        q,
        upper_name,
        upper_horizontal,
        REFINE_STEP_DEG,
        root_link=root_link,
        joints=joints,
        geometries=geometries,
        checker=checker,
        table_z=table_z,
    )
    total_samples += samples
    if clearance < minimum_clearance:
        minimum_clearance, minimum_link = clearance, link

    # LOWER is calibrated only with UPPER horizontal, including both guards.
    for target in guarded_limits(lower):
        q_target, clearance, link, samples = move_and_check(
            q_horizontal,
            lower_name,
            target,
            REFINE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        total_samples += samples
        if clearance < minimum_clearance:
            minimum_clearance, minimum_link = clearance, link
        _, clearance, link, samples = move_and_check(
            q_target,
            lower_name,
            q_horizontal[lower_name],
            REFINE_STEP_DEG,
            root_link=root_link,
            joints=joints,
            geometries=geometries,
            checker=checker,
            table_z=table_z,
        )
        total_samples += samples
        if clearance < minimum_clearance:
            minimum_clearance, minimum_link = clearance, link

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
    upper_candidates = [
        upper_horizontal + offset * base.DEG
        for offset in v7.ordered_offsets(UPPER_SEARCH_RADIUS_DEG, COARSE_STEP_DEG)
        if upper_min
        <= upper_horizontal + offset * base.DEG
        <= upper_max
    ]
    lower_candidates = candidate_values(lower_min, lower_max, COARSE_STEP_DEG)

    hip_guard_min, hip_guard_max = guarded_limits(hip)
    selected: dict[str, float] | None = None
    rejected = 0

    for upper_value, lower_value in itertools.product(upper_candidates, lower_candidates):
        try:
            q_candidate, _, _, _ = move_and_check(
                q_horizontal,
                upper_name,
                upper_value,
                COARSE_STEP_DEG,
                root_link=root_link,
                joints=joints,
                geometries=geometries,
                checker=checker,
                table_z=table_z,
            )
            q_candidate, _, _, _ = move_and_check(
                q_candidate,
                lower_name,
                lower_value,
                COARSE_STEP_DEG,
                root_link=root_link,
                joints=joints,
                geometries=geometries,
                checker=checker,
                table_z=table_z,
            )
            for hip_value in candidate_values(
                hip_guard_min,
                hip_guard_max,
                COARSE_STEP_DEG,
            ):
                probe = dict(q_candidate)
                probe[hip_name] = hip_value
                pose_check(
                    probe,
                    root_link=root_link,
                    joints=joints,
                    geometries=geometries,
                    checker=checker,
                    table_z=table_z,
                )

            # Refine the selected transition and entire guarded HIP sweep.
            q_refined, clearance, link, samples = move_and_check(
                q_horizontal,
                upper_name,
                upper_value,
                REFINE_STEP_DEG,
                root_link=root_link,
                joints=joints,
                geometries=geometries,
                checker=checker,
                table_z=table_z,
            )
            total_samples += samples
            if clearance < minimum_clearance:
                minimum_clearance, minimum_link = clearance, link
            q_refined, clearance, link, samples = move_and_check(
                q_refined,
                lower_name,
                lower_value,
                REFINE_STEP_DEG,
                root_link=root_link,
                joints=joints,
                geometries=geometries,
                checker=checker,
                table_z=table_z,
            )
            total_samples += samples
            if clearance < minimum_clearance:
                minimum_clearance, minimum_link = clearance, link
            for hip_value in candidate_values(
                hip_guard_min,
                hip_guard_max,
                REFINE_STEP_DEG,
            ):
                probe = dict(q_refined)
                probe[hip_name] = hip_value
                clearance, link = pose_check(
                    probe,
                    root_link=root_link,
                    joints=joints,
                    geometries=geometries,
                    checker=checker,
                    table_z=table_z,
                )
                total_samples += 1
                if clearance < minimum_clearance:
                    minimum_clearance, minimum_link = clearance, link

            selected = {
                "upper_deg": upper_value / base.DEG,
                "lower_deg": lower_value / base.DEG,
                "upper_deviation_from_horizontal_deg": abs(
                    upper_value - upper_horizontal
                )
                / base.DEG,
            }
            break
        except base.AuditFailure:
            rejected += 1

    if selected is None:
        raise base.AuditFailure(
            f"{leg}: no coordinated UPPER+LOWER pose clears the full HIP "
            f"guard range {hip_guard_min/base.DEG:.3f}..{hip_guard_max/base.DEG:.3f} deg; "
            f"rejected={rejected}"
        )

    return {
        "leg": leg.upper(),
        "upper_horizontal_deg": upper_horizontal / base.DEG,
        "upper_horizontal_error": horizontal_error,
        "upper_guard_deg": [
            (upper.lower - GUARD_RAD) / base.DEG,
            (upper.upper + GUARD_RAD) / base.DEG,
        ],
        "lower_guard_deg": [
            (lower.lower - GUARD_RAD) / base.DEG,
            (lower.upper + GUARD_RAD) / base.DEG,
        ],
        "hip_guard_deg": [hip_guard_min / base.DEG, hip_guard_max / base.DEG],
        "coordinated_hip_prerequisite": selected,
        "minimum_conservative_table_clearance_m": minimum_clearance,
        "minimum_clearance_link": minimum_link,
        "sample_count": total_samples,
        "fixture_geometry_checked": False,
        "fixture_exclusion_basis": "operator_validated_impossible",
    }


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    support = load_support_contract(repo_root)
    table_z = float(support["table_plane_z_m"])
    root_link, joints, geometries = base.load_model(repo_root / base.URDF_RELATIVE)
    ignored_pairs = base.adjacent_link_pairs(joints)
    home_transforms = base.link_transforms(root_link, joints, base.base_pose())
    home_collisions = v5.full_home_check(
        geometries,
        ignored_pairs,
        home_transforms,
    )
    if home_collisions:
        raise base.AuditFailure(
            f"canonical home has non-adjacent collisions: {sorted(home_collisions)}"
        )
    table_clearance(home_transforms, geometries, table_z)

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
        "algorithm": "ordered-guarded-support-contract-v8",
        "order": ["UPPER_MIN_MAX", "LOWER_MIN_MAX", "HIP_MIN_MAX"],
        "guard_ticks": GUARD_TICKS,
        "guard_deg": GUARD_DEG,
        "table_plane_z_m": table_z,
        "support_fixture_geometry_modeled": False,
        "support_fixture_exclusion": "operator_validated_impossible",
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
