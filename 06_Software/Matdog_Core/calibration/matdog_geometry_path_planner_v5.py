#!/usr/bin/env python3
"""Deterministic geometry-driven path and parking planner for V5.

For each endpoint the planner validates home, identifies the first path
obstruction, derives movable joints from URDF ancestry, searches 1-DOF within
URDF limits and only then a controlled 2-DOF grid. Candidate ordering is:

1. zero mesh intersection (mandatory),
2. maximum sampled minimum geometric clearance,
3. minimum displacement from home,
4. deterministic topology/value tie-break.

No LF hardware evidence, 3 mm threshold, leg-name rule, +50/+90 prerequisite
or fixed 30..90 degree seed list exists in this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable

from matdog_geometry_contact_search_v5 import (
    GEOMETRIC_CONTACT_FOUND,
    NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
    NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
    PATH_OBSTRUCTION,
    EndpointSpecV5,
    load_endpoint_specs,
)
from matdog_geometry_profile_v5 import (
    SCHEMA_VERSION as GEOMETRY_PROFILE_SCHEMA_VERSION,
    semantic_content_sha256 as geometry_profile_semantic_sha256,
    validate_pure_geometry_profile,
)
from matdog_geometry_scene_v5 import RobotSceneV5


DEFAULT_PATH_STEP_RAD = math.radians(1.0)
DEFAULT_1DOF_GRID_DIVISIONS = 6
DEFAULT_2DOF_GRID_DIVISIONS = 4
DEFAULT_CLEARANCE_SAMPLE_STRIDE = 10
DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD = 0.0001
DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS = 40

PATH_COLLISION_FREE = "COLLISION_FREE"
PATH_OBSTRUCTED = "PATH_OBSTRUCTION"

PARKING_NOT_NEEDED = "NOT_NEEDED"
PARKING_FEASIBLE_1DOF = "FEASIBLE_1DOF_PLAN_FOUND"
PARKING_FEASIBLE_2DOF = "FEASIBLE_2DOF_PLAN_FOUND"
PARKING_NO_FEASIBLE = "NO_FEASIBLE_PARKING_FOUND_IN_DECLARED_SEARCH_DOMAIN"
PARKING_NO_MOVABLE_JOINT = "NO_RELEVANT_MOVABLE_JOINT"

PARKING_SCHEMA_V1 = "matdog.geometry_path_parking.v1"
PARKING_SCHEMA_V2 = "matdog.geometry_path_parking.v2"
PARKING_V2_SEMANTIC_KEYS = (
    "schema_version",
    "input_geometry_profile",
    "provenance",
    "parameters",
    "summary",
    "plans",
)
PARKING_V2_TOP_LEVEL_KEYS = frozenset(
    (*PARKING_V2_SEMANTIC_KEYS, "generation_metadata", "semantic_content_sha256")
)


class GeometryPathPlannerV5Error(RuntimeError):
    """Invalid path domain or unstable collision result."""


@dataclass(frozen=True)
class EndpointParkingTaskV5:
    """Minimal G9 input derived from a validated saved V5 geometry profile.

    G7 search context and G7 path-obstruction results are intentionally not
    carried into G9.  They are oracle-replay provenance; G9 independently
    starts at home and derives the blocker and any parking configuration.
    """

    endpoint: EndpointSpecV5
    canonical_endpoint_index: int
    target_angle_rad: float
    target_source: str


@dataclass(frozen=True)
class FirstObstructionV5:
    sample_index: int
    progress: float
    joint_positions_rad: dict[str, float]
    link_pair: tuple[str, str]
    relation: str


@dataclass(frozen=True)
class RefinedObstructionV5:
    clear_progress: float
    contact_progress: float
    clear_joint_positions_rad: dict[str, float]
    contact_joint_positions_rad: dict[str, float]
    link_pair: tuple[str, str]
    relation: str
    bisection_resolution_rad: float
    bisection_iterations: int


@dataclass(frozen=True)
class PathValidationV5:
    path_id: str
    start_joint_positions_rad: dict[str, float]
    end_joint_positions_rad: dict[str, float]
    moving_joint_names: tuple[str, ...]
    active_pair_excluded: tuple[str, str] | None
    status: str
    planned_sample_count: int
    evaluated_sample_count: int
    first_obstruction: FirstObstructionV5 | None
    min_clearance_m: float | None
    min_clearance_kind: str | None
    clearance_samples_evaluated: int
    sampled_configuration_sha256: str
    reversed_sampled_configuration_sha256: str
    reverse_validation_of: str | None = None
    refined_first_obstruction: RefinedObstructionV5 | None = None

    @property
    def collision_free(self) -> bool:
        return self.status == PATH_COLLISION_FREE


@dataclass(frozen=True)
class ParkingObjectiveV5:
    zero_unintended_intersection: bool
    min_clearance_m: float | None
    min_clearance_kind: str | None
    displacement_l2_rad: float
    deterministic_tie_break: tuple[tuple[str, float], ...]


@dataclass(frozen=True)
class EndpointParkingPlanV5:
    canonical_endpoint_index: int
    endpoint_id: str
    joint_name: str
    limit_side: str
    active_branch_id: str
    declared_limit_rad: float
    target_angle_rad: float
    target_delta_from_declared_rad: float
    target_within_urdf_limits: bool
    target_domain: str
    target_source: str
    allowed_endpoint_contact_pair: tuple[str, str]
    start_configuration_valid: bool
    baseline_task_path: PathValidationV5
    first_blocking_pair: tuple[str, str] | None
    first_blocking_relation: str | None
    relevant_movable_joint_names: tuple[str, ...]
    outcome: str
    parking_degrees_of_freedom: int
    parking_configuration_rad: dict[str, float]
    declared_joint_limit_bounds_rad: dict[str, tuple[float, float]]
    one_dof_search_grid_rad: dict[str, tuple[float, ...]]
    two_dof_search_grid_rad: dict[str, tuple[float, ...]]
    path_in: PathValidationV5 | None
    task_path: PathValidationV5
    task_return: PathValidationV5 | None
    path_out: PathValidationV5 | None
    objective: ParkingObjectiveV5 | None
    evaluated_1d_candidates: int
    evaluated_2d_candidates: int


@dataclass(frozen=True)
class ParkingPlannerParametersV5:
    path_step_rad: float = DEFAULT_PATH_STEP_RAD
    one_dof_grid_divisions: int = DEFAULT_1DOF_GRID_DIVISIONS
    two_dof_grid_divisions: int = DEFAULT_2DOF_GRID_DIVISIONS
    clearance_sample_stride: int = DEFAULT_CLEARANCE_SAMPLE_STRIDE

    def validate(self) -> None:
        if not (math.isfinite(self.path_step_rad) and self.path_step_rad > 0.0):
            raise GeometryPathPlannerV5Error(f"invalid path step {self.path_step_rad}")
        if self.one_dof_grid_divisions < 2 or self.two_dof_grid_divisions < 2:
            raise GeometryPathPlannerV5Error("parking grid divisions must be >= 2")
        if self.clearance_sample_stride <= 0:
            raise GeometryPathPlannerV5Error("clearance sample stride must be positive")


def endpoint_parking_tasks_from_profile(
    scene: RobotSceneV5,
    geometry_profile: dict[str, Any],
) -> tuple[EndpointParkingTaskV5, ...]:
    """Join saved endpoint geometry to the live model without name parsing.

    Full URDF/mesh/source provenance matching is a runner-level hard gate.
    This function additionally proves that every serialized endpoint identity
    and model-derived structural field still agrees with the live scene.
    """

    validate_pure_geometry_profile(geometry_profile)
    stored_semantic = geometry_profile.get("semantic_content_sha256")
    recomputed_semantic = geometry_profile_semantic_sha256(geometry_profile)
    if stored_semantic != recomputed_semantic:
        raise GeometryPathPlannerV5Error(
            "geometry profile semantic SHA changed while constructing G9 tasks"
        )

    records_by_key: dict[tuple[str, str], dict[str, Any]] = {}
    for record in geometry_profile.get("endpoint_searches", []):
        identity = record.get("identity", {})
        key = (identity.get("joint_name"), identity.get("limit_side"))
        if not all(isinstance(value, str) for value in key):
            raise GeometryPathPlannerV5Error(
                f"invalid saved endpoint identity: {identity!r}"
            )
        typed_key = (str(key[0]), str(key[1]))
        if typed_key in records_by_key:
            raise GeometryPathPlannerV5Error(
                f"duplicate saved endpoint identity: {typed_key}"
            )
        records_by_key[typed_key] = record

    specs = load_endpoint_specs(scene.model)
    expected_keys = {(spec.joint_name, spec.side) for spec in specs}
    actual_keys = set(records_by_key)
    if actual_keys != expected_keys or len(records_by_key) != len(specs):
        raise GeometryPathPlannerV5Error(
            "saved/live endpoint coverage mismatch: "
            f"missing={sorted(expected_keys - actual_keys)}, "
            f"extra={sorted(actual_keys - expected_keys)}"
        )

    tasks: list[EndpointParkingTaskV5] = []
    for canonical_endpoint_index, spec in enumerate(specs):
        record = records_by_key[(spec.joint_name, spec.side)]
        identity = record["identity"]
        expected_presentation_id = spec.endpoint_id
        if identity.get("presentation_id") != expected_presentation_id:
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: presentation identity mismatch"
            )
        if record.get("articulated_branch_id") != spec.branch_root_joint_name:
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: articulated branch mismatch"
            )
        if record.get("structural_depth") != spec.structural_depth:
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: structural depth mismatch"
            )
        declared = record.get("declared_limit_rad")
        if not isinstance(declared, (int, float)) or not math.isclose(
            float(declared),
            spec.urdf_declared_limit_rad,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: declared limit mismatch"
            )
        if tuple(record.get("active_revolute_pair", ())) != spec.active_link_pair:
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: active revolute pair mismatch"
            )

        geometric = record.get("geometric_contact", {})
        if tuple(geometric.get("link_pair", ())) != spec.active_link_pair:
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: saved contact pair mismatch"
            )
        status = geometric.get("status")
        angle = geometric.get("angle_rad")
        if status == GEOMETRIC_CONTACT_FOUND:
            if not isinstance(angle, (int, float)) or not math.isfinite(float(angle)):
                raise GeometryPathPlannerV5Error(
                    f"{expected_presentation_id}: contact status lacks a finite angle"
                )
            target = float(angle)
            source = "GEOMETRIC_CONTACT"
        elif status == NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN:
            if angle is not None:
                raise GeometryPathPlannerV5Error(
                    f"{expected_presentation_id}: no-contact status has an angle"
                )
            target = spec.urdf_declared_limit_rad
            source = "DECLARED_LIMIT_NO_GEOMETRIC_CONTACT"
        else:
            raise GeometryPathPlannerV5Error(
                f"{expected_presentation_id}: unsupported geometric status {status!r}"
            )
        tasks.append(
            EndpointParkingTaskV5(
                endpoint=spec,
                canonical_endpoint_index=canonical_endpoint_index,
                target_angle_rad=target,
                target_source=source,
            )
        )
    return tuple(tasks)


def _canonical_configuration(
    scene: RobotSceneV5,
    overrides: dict[str, float],
) -> dict[str, float]:
    pose = scene.full_pose(overrides)
    return {
        name: round(float(pose[name]), 15)
        for name in scene.model.actuated_joint_names
    }


def _interpolated_configurations(
    scene: RobotSceneV5,
    start: dict[str, float],
    end: dict[str, float],
    step_rad: float,
) -> tuple[tuple[float, dict[str, float]], ...]:
    start_full = _canonical_configuration(scene, start)
    end_full = _canonical_configuration(scene, end)
    maximum_delta = max(
        abs(end_full[name] - start_full[name])
        for name in scene.model.actuated_joint_names
    )
    intervals = max(1, int(math.ceil(maximum_delta / step_rad)))
    samples = []
    for index in range(intervals + 1):
        progress = index / intervals
        pose = {
            name: round(
                start_full[name] + (end_full[name] - start_full[name]) * progress,
                15,
            )
            for name in scene.model.actuated_joint_names
        }
        samples.append((progress, pose))
    return tuple(samples)


def _configuration_sequence_sha256(
    scene: RobotSceneV5,
    samples: tuple[tuple[float, dict[str, float]], ...],
    *,
    reverse: bool = False,
) -> str:
    ordered_samples = reversed(samples) if reverse else iter(samples)
    values = [
        [pose[name] for name in scene.model.actuated_joint_names]
        for _progress, pose in ordered_samples
    ]
    canonical = json.dumps(values, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _refine_first_obstruction(
    scene: RobotSceneV5,
    samples: tuple[tuple[float, dict[str, float]], ...],
    sample_index: int,
    pairs: tuple[tuple[str, str], ...],
) -> RefinedObstructionV5 | None:
    """Bisect one planner clear/contact sample interval in joint space."""

    if sample_index <= 0:
        return None
    clear_progress, clear_pose = samples[sample_index - 1]
    contact_progress, contact_pose = samples[sample_index]
    clear_pose = dict(clear_pose)
    contact_pose = dict(contact_pose)
    iterations = 0
    while (
        max(
            abs(contact_pose[name] - clear_pose[name])
            for name in scene.model.actuated_joint_names
        )
        > DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD
        and iterations < DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
    ):
        midpoint_progress = (clear_progress + contact_progress) / 2.0
        midpoint_pose = {
            name: round((clear_pose[name] + contact_pose[name]) / 2.0, 15)
            for name in scene.model.actuated_joint_names
        }
        collides, _pair = scene.first_collision_at_pose(
            midpoint_pose,
            link_pairs=pairs,
        )
        if collides:
            contact_progress = midpoint_progress
            contact_pose = midpoint_pose
        else:
            clear_progress = midpoint_progress
            clear_pose = midpoint_pose
        iterations += 1
    collides, pair = scene.first_collision_at_pose(contact_pose, link_pairs=pairs)
    if not collides or pair is None:
        raise GeometryPathPlannerV5Error(
            "refined planner obstruction lost its contact-side collision"
        )
    return RefinedObstructionV5(
        clear_progress=clear_progress,
        contact_progress=contact_progress,
        clear_joint_positions_rad=clear_pose,
        contact_joint_positions_rad=contact_pose,
        link_pair=pair,
        relation=scene.model.pair_relation(*pair),
        bisection_resolution_rad=DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD,
        bisection_iterations=iterations,
    )


def validate_configuration_path(
    scene: RobotSceneV5,
    *,
    path_id: str,
    start_joint_positions_rad: dict[str, float],
    end_joint_positions_rad: dict[str, float],
    active_pair_excluded: tuple[str, str] | None,
    step_rad: float,
    clearance_sample_stride: int,
    measure_clearance: bool = True,
) -> PathValidationV5:
    samples = _interpolated_configurations(
        scene,
        start_joint_positions_rad,
        end_joint_positions_rad,
        step_rad,
    )
    pairs = scene.model.path_obstruction_pairs(exclude_pair=active_pair_excluded)
    clearance_pairs = scene.model.clearance_pairs()
    sequence_sha256 = _configuration_sequence_sha256(scene, samples)
    reversed_sequence_sha256 = _configuration_sequence_sha256(
        scene,
        samples,
        reverse=True,
    )
    start = samples[0][1]
    end = samples[-1][1]
    moving = tuple(
        name
        for name in scene.model.actuated_joint_names
        if start[name] != end[name]
    )
    min_clearance: float | None = None
    min_clearance_kind: str | None = None
    clearance_count = 0
    for index, (progress, pose) in enumerate(samples):
        collides, pair = scene.first_collision_at_pose(pose, link_pairs=pairs)
        if collides:
            if pair is None:
                raise GeometryPathPlannerV5Error(
                    f"{path_id}: collision reported without a pair"
                )
            refined = _refine_first_obstruction(scene, samples, index, pairs)
            return PathValidationV5(
                path_id=path_id,
                start_joint_positions_rad=start,
                end_joint_positions_rad=end,
                moving_joint_names=moving,
                active_pair_excluded=active_pair_excluded,
                status=PATH_OBSTRUCTED,
                planned_sample_count=len(samples),
                evaluated_sample_count=index + 1,
                first_obstruction=FirstObstructionV5(
                    sample_index=index,
                    progress=progress,
                    joint_positions_rad=pose,
                    link_pair=pair,
                    relation=scene.model.pair_relation(*pair),
                ),
                min_clearance_m=min_clearance,
                min_clearance_kind=min_clearance_kind,
                clearance_samples_evaluated=clearance_count,
                sampled_configuration_sha256=sequence_sha256,
                reversed_sampled_configuration_sha256=reversed_sequence_sha256,
                refined_first_obstruction=refined,
            )

        should_measure = (
            measure_clearance
            and (
                index % clearance_sample_stride == 0
                or (len(samples) - 1 - index) % clearance_sample_stride == 0
            )
        )
        if should_measure:
            # Clearance is reported over non-adjacent geometry. Revolute
            # interfaces remain collision-validated but are not clearance
            # policy surfaces; the active contact cannot leak back here.
            _link_a, _link_b, result = scene.worst_pair_at_pose(
                pose,
                link_pairs=clearance_pairs,
            )
            clearance_count += 1
            if result.status == "INTERSECTING" or result.clearance_m is None:
                raise GeometryPathPlannerV5Error(
                    f"{path_id}: clearance found an intersection missed by feasibility"
                )
            if min_clearance is None or result.clearance_m < min_clearance:
                min_clearance = result.clearance_m
                min_clearance_kind = result.clearance_kind

    return PathValidationV5(
        path_id=path_id,
        start_joint_positions_rad=start,
        end_joint_positions_rad=end,
        moving_joint_names=moving,
        active_pair_excluded=active_pair_excluded,
        status=PATH_COLLISION_FREE,
        planned_sample_count=len(samples),
        evaluated_sample_count=len(samples),
        first_obstruction=None,
        min_clearance_m=min_clearance,
        min_clearance_kind=min_clearance_kind,
        clearance_samples_evaluated=clearance_count,
        sampled_configuration_sha256=sequence_sha256,
        reversed_sampled_configuration_sha256=reversed_sequence_sha256,
    )


def _validated_reverse(path: PathValidationV5, path_id: str) -> PathValidationV5:
    if not path.collision_free:
        raise GeometryPathPlannerV5Error(
            f"cannot infer reverse validation from obstructed path {path.path_id}"
        )
    return PathValidationV5(
        path_id=path_id,
        start_joint_positions_rad=path.end_joint_positions_rad,
        end_joint_positions_rad=path.start_joint_positions_rad,
        moving_joint_names=path.moving_joint_names,
        active_pair_excluded=path.active_pair_excluded,
        status=PATH_COLLISION_FREE,
        planned_sample_count=path.planned_sample_count,
        evaluated_sample_count=path.evaluated_sample_count,
        first_obstruction=None,
        min_clearance_m=path.min_clearance_m,
        min_clearance_kind=path.min_clearance_kind,
        clearance_samples_evaluated=path.clearance_samples_evaluated,
        sampled_configuration_sha256=path.reversed_sampled_configuration_sha256,
        reversed_sampled_configuration_sha256=path.sampled_configuration_sha256,
        reverse_validation_of=path.path_id,
    )


def _aggregate_clearance(
    paths: Iterable[PathValidationV5],
) -> tuple[float | None, str | None]:
    selected: PathValidationV5 | None = None
    for path in paths:
        if path.min_clearance_m is None:
            continue
        if selected is None:
            selected = path
            continue
        assert selected.min_clearance_m is not None
        if path.min_clearance_m < selected.min_clearance_m:
            selected = path
    if selected is None:
        return None, None
    return selected.min_clearance_m, selected.min_clearance_kind


def _grid_values(lower: float, upper: float, divisions: int) -> tuple[float, ...]:
    values = {
        round(lower + (upper - lower) * index / divisions, 15)
        for index in range(divisions + 1)
    }
    if lower <= 0.0 <= upper:
        values.add(0.0)
    values.discard(0.0)
    return tuple(sorted(values, key=lambda value: (abs(value), value)))


def _relevant_movable_joints(
    scene: RobotSceneV5,
    task: EndpointParkingTaskV5,
    obstruction_pair: tuple[str, str],
) -> tuple[str, ...]:
    model = scene.model
    endpoint = task.endpoint
    selected = set(
        model.relevant_actuated_joints_for_pair(*obstruction_pair)
    )
    # Common rigid ancestors have already been removed by the pair-relative
    # topology query.  For cross-branch blockers both branches remain eligible;
    # preferring "the other leg" would be an external motion policy, not
    # geometric truth.  Only the joint currently swept to its endpoint is held.
    selected.discard(endpoint.joint_name)
    return tuple(
        name for name in model.actuated_joint_names if name in selected
    )


def _objective(
    scene: RobotSceneV5,
    parking_configuration: dict[str, float],
    paths: tuple[PathValidationV5, ...],
) -> ParkingObjectiveV5:
    clearance, kind = _aggregate_clearance(paths)
    displacement = math.sqrt(sum(value * value for value in parking_configuration.values()))
    tie = tuple(
        (name, round(parking_configuration[name], 15))
        for name in scene.model.actuated_joint_names
        if name in parking_configuration
    )
    return ParkingObjectiveV5(
        zero_unintended_intersection=True,
        min_clearance_m=clearance,
        min_clearance_kind=kind,
        displacement_l2_rad=displacement,
        deterministic_tie_break=tie,
    )


def _candidate_sort_key(objective: ParkingObjectiveV5) -> tuple[Any, ...]:
    clearance = objective.min_clearance_m
    return (
        -(clearance if clearance is not None else -math.inf),
        objective.displacement_l2_rad,
        objective.deterministic_tie_break,
    )


@dataclass(frozen=True)
class _FeasibleCandidate:
    parking_configuration_rad: dict[str, float]
    path_in: PathValidationV5
    task_path: PathValidationV5
    task_return: PathValidationV5
    path_out: PathValidationV5
    objective: ParkingObjectiveV5


def _evaluate_candidate(
    scene: RobotSceneV5,
    task: EndpointParkingTaskV5,
    parking_configuration: dict[str, float],
    parameters: ParkingPlannerParametersV5,
) -> _FeasibleCandidate | None:
    endpoint = task.endpoint
    target = task.target_angle_rad
    home = scene.home_pose()
    parked = scene.full_pose(parking_configuration)
    path_in_feasibility = validate_configuration_path(
        scene,
        path_id=f"{endpoint.endpoint_id}:path_in",
        start_joint_positions_rad=home,
        end_joint_positions_rad=parked,
        active_pair_excluded=None,
        step_rad=parameters.path_step_rad,
        clearance_sample_stride=parameters.clearance_sample_stride,
        measure_clearance=False,
    )
    if not path_in_feasibility.collision_free:
        return None

    target_configuration = dict(parked)
    target_configuration[endpoint.joint_name] = target
    task_feasibility = validate_configuration_path(
        scene,
        path_id=f"{endpoint.endpoint_id}:task_path",
        start_joint_positions_rad=parked,
        end_joint_positions_rad=target_configuration,
        active_pair_excluded=endpoint.active_link_pair,
        step_rad=parameters.path_step_rad,
        clearance_sample_stride=parameters.clearance_sample_stride,
        measure_clearance=False,
    )
    if not task_feasibility.collision_free:
        return None

    # Clearance is an optimization quantity, not the feasibility rule.  Run
    # it only for candidates already proven intersection-free, over the same
    # deterministic configuration sequences and pair domains.
    path_in = validate_configuration_path(
        scene,
        path_id=path_in_feasibility.path_id,
        start_joint_positions_rad=home,
        end_joint_positions_rad=parked,
        active_pair_excluded=None,
        step_rad=parameters.path_step_rad,
        clearance_sample_stride=parameters.clearance_sample_stride,
        measure_clearance=True,
    )
    task_path = validate_configuration_path(
        scene,
        path_id=task_feasibility.path_id,
        start_joint_positions_rad=parked,
        end_joint_positions_rad=target_configuration,
        active_pair_excluded=endpoint.active_link_pair,
        step_rad=parameters.path_step_rad,
        clearance_sample_stride=parameters.clearance_sample_stride,
        measure_clearance=True,
    )
    if not path_in.collision_free or not task_path.collision_free:
        raise GeometryPathPlannerV5Error(
            f"{endpoint.endpoint_id}: candidate feasibility changed during clearance evaluation"
        )

    task_return = _validated_reverse(
        task_path,
        f"{endpoint.endpoint_id}:task_return",
    )
    path_out = _validated_reverse(path_in, f"{endpoint.endpoint_id}:path_out")
    objective = _objective(
        scene,
        parking_configuration,
        (path_in, task_path, task_return, path_out),
    )
    return _FeasibleCandidate(
        parking_configuration_rad=dict(sorted(parking_configuration.items())),
        path_in=path_in,
        task_path=task_path,
        task_return=task_return,
        path_out=path_out,
        objective=objective,
    )


def _validated_home_configuration(scene: RobotSceneV5) -> dict[str, float]:
    home = scene.home_pose()
    outside_limits = []
    for joint_name in scene.model.actuated_joint_names:
        joint = scene.model.joints[joint_name]
        assert joint.lower_limit_rad is not None
        assert joint.upper_limit_rad is not None
        value = home[joint_name]
        if not joint.lower_limit_rad <= value <= joint.upper_limit_rad:
            outside_limits.append(
                (joint_name, value, joint.lower_limit_rad, joint.upper_limit_rad)
            )
    if outside_limits:
        raise GeometryPathPlannerV5Error(
            f"STOP: home configuration lies outside URDF limits: {outside_limits}"
        )
    home_collision, home_pair = scene.first_collision_at_pose(home)
    if home_collision:
        raise GeometryPathPlannerV5Error(
            f"STOP: home configuration intersects at {home_pair}"
        )
    return home


def plan_endpoint_parking(
    scene: RobotSceneV5,
    task: EndpointParkingTaskV5,
    *,
    parameters: ParkingPlannerParametersV5 = ParkingPlannerParametersV5(),
) -> EndpointParkingPlanV5:
    parameters.validate()
    endpoint = task.endpoint
    target = task.target_angle_rad
    target_source = task.target_source
    target_delta = target - endpoint.urdf_declared_limit_rad
    target_within_limits = endpoint.urdf_lower_rad <= target <= endpoint.urdf_upper_rad
    target_domain = (
        "EXECUTABLE_URDF_DOMAIN"
        if target_within_limits
        else "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS"
    )
    home = _validated_home_configuration(scene)
    target_configuration = dict(home)
    target_configuration[endpoint.joint_name] = target
    baseline_feasibility = validate_configuration_path(
        scene,
        path_id=f"{endpoint.endpoint_id}:baseline_task",
        start_joint_positions_rad=home,
        end_joint_positions_rad=target_configuration,
        active_pair_excluded=endpoint.active_link_pair,
        step_rad=parameters.path_step_rad,
        clearance_sample_stride=parameters.clearance_sample_stride,
        measure_clearance=False,
    )
    if baseline_feasibility.collision_free:
        baseline = validate_configuration_path(
            scene,
            path_id=baseline_feasibility.path_id,
            start_joint_positions_rad=home,
            end_joint_positions_rad=target_configuration,
            active_pair_excluded=endpoint.active_link_pair,
            step_rad=parameters.path_step_rad,
            clearance_sample_stride=parameters.clearance_sample_stride,
            measure_clearance=True,
        )
        if not baseline.collision_free:
            raise GeometryPathPlannerV5Error(
                f"{endpoint.endpoint_id}: baseline feasibility changed during clearance evaluation"
            )
        reverse = _validated_reverse(baseline, f"{endpoint.endpoint_id}:baseline_return")
        objective = _objective(scene, {}, (baseline, reverse))
        return EndpointParkingPlanV5(
            canonical_endpoint_index=task.canonical_endpoint_index,
            endpoint_id=endpoint.endpoint_id,
            joint_name=endpoint.joint_name,
            limit_side=endpoint.side,
            active_branch_id=endpoint.branch_root_joint_name,
            declared_limit_rad=endpoint.urdf_declared_limit_rad,
            target_angle_rad=target,
            target_delta_from_declared_rad=target_delta,
            target_within_urdf_limits=target_within_limits,
            target_domain=target_domain,
            target_source=target_source,
            allowed_endpoint_contact_pair=endpoint.active_link_pair,
            start_configuration_valid=True,
            baseline_task_path=baseline,
            first_blocking_pair=None,
            first_blocking_relation=None,
            relevant_movable_joint_names=(),
            outcome=PARKING_NOT_NEEDED,
            parking_degrees_of_freedom=0,
            parking_configuration_rad={},
            declared_joint_limit_bounds_rad={},
            one_dof_search_grid_rad={},
            two_dof_search_grid_rad={},
            path_in=None,
            task_path=baseline,
            task_return=reverse,
            path_out=None,
            objective=objective,
            evaluated_1d_candidates=0,
            evaluated_2d_candidates=0,
        )

    baseline = baseline_feasibility
    obstruction = baseline.first_obstruction
    assert obstruction is not None
    refined_obstruction = baseline.refined_first_obstruction
    blocking_pair = (
        refined_obstruction.link_pair
        if refined_obstruction is not None
        else obstruction.link_pair
    )
    blocking_relation = (
        refined_obstruction.relation
        if refined_obstruction is not None
        else obstruction.relation
    )
    relevant = _relevant_movable_joints(scene, task, blocking_pair)
    search_domains = {
        name: (
            float(scene.model.joints[name].lower_limit_rad),
            float(scene.model.joints[name].upper_limit_rad),
        )
        for name in relevant
    }
    one_dof_grids = {
        name: _grid_values(
            *search_domains[name],
            parameters.one_dof_grid_divisions,
        )
        for name in relevant
    }
    if not relevant:
        return EndpointParkingPlanV5(
            canonical_endpoint_index=task.canonical_endpoint_index,
            endpoint_id=endpoint.endpoint_id,
            joint_name=endpoint.joint_name,
            limit_side=endpoint.side,
            active_branch_id=endpoint.branch_root_joint_name,
            declared_limit_rad=endpoint.urdf_declared_limit_rad,
            target_angle_rad=target,
            target_delta_from_declared_rad=target_delta,
            target_within_urdf_limits=target_within_limits,
            target_domain=target_domain,
            target_source=target_source,
            allowed_endpoint_contact_pair=endpoint.active_link_pair,
            start_configuration_valid=True,
            baseline_task_path=baseline,
            first_blocking_pair=blocking_pair,
            first_blocking_relation=blocking_relation,
            relevant_movable_joint_names=(),
            outcome=PARKING_NO_MOVABLE_JOINT,
            parking_degrees_of_freedom=0,
            parking_configuration_rad={},
            declared_joint_limit_bounds_rad={},
            one_dof_search_grid_rad={},
            two_dof_search_grid_rad={},
            path_in=None,
            task_path=baseline,
            task_return=None,
            path_out=None,
            objective=None,
            evaluated_1d_candidates=0,
            evaluated_2d_candidates=0,
        )

    feasible_1d: list[_FeasibleCandidate] = []
    evaluated_1d = 0
    for joint_name in relevant:
        for value in one_dof_grids[joint_name]:
            evaluated_1d += 1
            candidate = _evaluate_candidate(
                scene,
                task,
                {joint_name: value},
                parameters,
            )
            if candidate is not None:
                feasible_1d.append(candidate)

    if feasible_1d:
        best = min(feasible_1d, key=lambda item: _candidate_sort_key(item.objective))
        return EndpointParkingPlanV5(
            canonical_endpoint_index=task.canonical_endpoint_index,
            endpoint_id=endpoint.endpoint_id,
            joint_name=endpoint.joint_name,
            limit_side=endpoint.side,
            active_branch_id=endpoint.branch_root_joint_name,
            declared_limit_rad=endpoint.urdf_declared_limit_rad,
            target_angle_rad=target,
            target_delta_from_declared_rad=target_delta,
            target_within_urdf_limits=target_within_limits,
            target_domain=target_domain,
            target_source=target_source,
            allowed_endpoint_contact_pair=endpoint.active_link_pair,
            start_configuration_valid=True,
            baseline_task_path=baseline,
            first_blocking_pair=blocking_pair,
            first_blocking_relation=blocking_relation,
            relevant_movable_joint_names=relevant,
            outcome=PARKING_FEASIBLE_1DOF,
            parking_degrees_of_freedom=1,
            parking_configuration_rad=best.parking_configuration_rad,
            declared_joint_limit_bounds_rad=search_domains,
            one_dof_search_grid_rad=one_dof_grids,
            two_dof_search_grid_rad={},
            path_in=best.path_in,
            task_path=best.task_path,
            task_return=best.task_return,
            path_out=best.path_out,
            objective=best.objective,
            evaluated_1d_candidates=evaluated_1d,
            evaluated_2d_candidates=0,
        )

    feasible_2d: list[_FeasibleCandidate] = []
    evaluated_2d = 0
    two_dof_grids = {
        name: _grid_values(
            *search_domains[name],
            parameters.two_dof_grid_divisions,
        )
        for name in relevant
    }
    for joint_a, joint_b in itertools.combinations(relevant, 2):
        values_a = two_dof_grids[joint_a]
        values_b = two_dof_grids[joint_b]
        for value_a, value_b in itertools.product(values_a, values_b):
            evaluated_2d += 1
            candidate = _evaluate_candidate(
                scene,
                task,
                {joint_a: value_a, joint_b: value_b},
                parameters,
            )
            if candidate is not None:
                feasible_2d.append(candidate)

    if feasible_2d:
        best = min(feasible_2d, key=lambda item: _candidate_sort_key(item.objective))
        outcome = PARKING_FEASIBLE_2DOF
        degrees = 2
        parking = best.parking_configuration_rad
        path_in = best.path_in
        task_path = best.task_path
        task_return = best.task_return
        path_out = best.path_out
        objective = best.objective
    else:
        outcome = PARKING_NO_FEASIBLE
        degrees = 0
        parking = {}
        path_in = None
        task_path = baseline
        task_return = None
        path_out = None
        objective = None

    return EndpointParkingPlanV5(
        canonical_endpoint_index=task.canonical_endpoint_index,
        endpoint_id=endpoint.endpoint_id,
        joint_name=endpoint.joint_name,
        limit_side=endpoint.side,
        active_branch_id=endpoint.branch_root_joint_name,
        declared_limit_rad=endpoint.urdf_declared_limit_rad,
        target_angle_rad=target,
        target_delta_from_declared_rad=target_delta,
        target_within_urdf_limits=target_within_limits,
        target_domain=target_domain,
        target_source=target_source,
        allowed_endpoint_contact_pair=endpoint.active_link_pair,
        start_configuration_valid=True,
        baseline_task_path=baseline,
        first_blocking_pair=blocking_pair,
        first_blocking_relation=blocking_relation,
        relevant_movable_joint_names=relevant,
        outcome=outcome,
        parking_degrees_of_freedom=degrees,
        parking_configuration_rad=parking,
        declared_joint_limit_bounds_rad=search_domains,
        one_dof_search_grid_rad=one_dof_grids,
        two_dof_search_grid_rad=two_dof_grids,
        path_in=path_in,
        task_path=task_path,
        task_return=task_return,
        path_out=path_out,
        objective=objective,
        evaluated_1d_candidates=evaluated_1d,
        evaluated_2d_candidates=evaluated_2d,
    )


def plan_all_endpoint_parking(
    scene: RobotSceneV5,
    tasks: Iterable[EndpointParkingTaskV5],
    *,
    parameters: ParkingPlannerParametersV5 = ParkingPlannerParametersV5(),
) -> tuple[EndpointParkingPlanV5, ...]:
    parameters.validate()
    _validated_home_configuration(scene)
    tasks = tuple(tasks)
    expected_ids = {
        spec.endpoint_id for spec in load_endpoint_specs(scene.model)
    }
    actual_ids = [task.endpoint.endpoint_id for task in tasks]
    if len(actual_ids) != len(set(actual_ids)) or set(actual_ids) != expected_ids:
        raise GeometryPathPlannerV5Error(
            "parking task coverage mismatch: "
            f"expected={len(expected_ids)}, actual={len(actual_ids)}, "
            f"missing={sorted(expected_ids - set(actual_ids))}, "
            f"extra={sorted(set(actual_ids) - expected_ids)}"
        )
    actual_indices = [task.canonical_endpoint_index for task in tasks]
    expected_indices = set(range(len(expected_ids)))
    if len(actual_indices) != len(set(actual_indices)) or set(actual_indices) != expected_indices:
        raise GeometryPathPlannerV5Error(
            f"canonical endpoint indices must be exactly 0..{len(expected_ids) - 1}: "
            f"{actual_indices}"
        )
    sorted_tasks = sorted(tasks, key=lambda item: item.canonical_endpoint_index)
    return tuple(
        plan_endpoint_parking(scene, task, parameters=parameters)
        for task in sorted_tasks
    )


def validate_endpoint_path_plan_consistency(
    geometry_profile: dict[str, Any],
    plans: Iterable[EndpointParkingPlanV5],
    *,
    path_step_rad: float,
) -> dict[str, Any]:
    """Hard-gate the two representations of the canonical direct sweep.

    Endpoint search and the planner independently bisect the first transition
    on the same equal coarse grid.  Their refined contact-side boundaries must
    agree within the larger of their declared bisection resolutions.
    """

    validate_pure_geometry_profile(geometry_profile)
    if not (math.isfinite(path_step_rad) and path_step_rad > 0.0):
        raise GeometryPathPlannerV5Error(
            "STOP: path-consistency gate received an invalid sample step"
        )
    records = geometry_profile.get("endpoint_searches")
    if not isinstance(records, list) or len(records) != 24:
        raise GeometryPathPlannerV5Error(
            "STOP: path-consistency gate requires 24 canonical endpoint records"
        )
    typed_plans = tuple(sorted(plans, key=lambda plan: plan.canonical_endpoint_index))
    if (
        len(typed_plans) != 24
        or [plan.canonical_endpoint_index for plan in typed_plans] != list(range(24))
    ):
        raise GeometryPathPlannerV5Error(
            "STOP: path-consistency gate requires canonical parking indices 0..23"
        )

    rows: list[dict[str, Any]] = []
    max_delta = 0.0
    for index, (record, plan) in enumerate(zip(records, typed_plans, strict=True)):
        identity = record.get("identity", {})
        endpoint_id = identity.get("presentation_id")
        joint_name = identity.get("joint_name")
        limit_side = identity.get("limit_side")
        if (
            endpoint_id != plan.endpoint_id
            or joint_name != plan.joint_name
            or limit_side != plan.limit_side
        ):
            raise GeometryPathPlannerV5Error(
                f"STOP: path-consistency identity mismatch at canonical index {index}"
            )
        if record.get("search_context", {}).get("joint_positions_rad") != {}:
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: canonical endpoint path has non-empty context"
            )

        geometric = record.get("geometric_contact", {})
        if geometric.get("status") == GEOMETRIC_CONTACT_FOUND:
            target = geometric.get("angle_rad")
        elif geometric.get("status") == NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN:
            target = record.get("declared_limit_rad")
        else:
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: unsupported geometric target status"
            )
        if not isinstance(target, (int, float)) or not math.isfinite(float(target)):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: canonical direct target is not finite"
            )
        target = float(target)
        if not math.isclose(
            plan.target_angle_rad, target, rel_tol=0.0, abs_tol=1e-12
        ):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: endpoint and parking targets differ"
            )

        path = record.get("path_obstruction")
        if not isinstance(path, dict):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: canonical endpoint path layer is missing"
            )
        search = path.get("search", {})
        coarse_step = search.get("coarse_step_rad")
        resolution = search.get("bisection_resolution_rad")
        domain = search.get("domain_rad")
        expected_domain = [min(0.0, target), max(0.0, target)]
        if (
            not isinstance(coarse_step, (int, float))
            or not math.isclose(
                float(coarse_step), path_step_rad, rel_tol=0.0, abs_tol=1e-12
            )
            or not isinstance(resolution, (int, float))
            or not math.isfinite(float(resolution))
            or float(resolution) <= 0.0
            or not isinstance(domain, list)
            or len(domain) != 2
            or any(
                not math.isclose(
                    float(observed), expected, rel_tol=0.0, abs_tol=1e-12
                )
                for observed, expected in zip(domain, expected_domain, strict=True)
            )
        ):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: endpoint path is not the declared direct sweep"
            )

        baseline = plan.baseline_task_path
        if (
            baseline.active_pair_excluded != plan.allowed_endpoint_contact_pair
            or baseline.moving_joint_names != (plan.joint_name,)
            or not math.isclose(
                baseline.end_joint_positions_rad[plan.joint_name],
                target,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            or any(
                not math.isclose(float(value), 0.0, rel_tol=0.0, abs_tol=1e-12)
                for value in baseline.start_joint_positions_rad.values()
            )
            or any(
                name != plan.joint_name
                and not math.isclose(
                    float(value), 0.0, rel_tol=0.0, abs_tol=1e-12
                )
                for name, value in baseline.end_joint_positions_rad.items()
            )
        ):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: planner baseline is not the same q=0 direct sweep"
            )

        endpoint_obstructed = path.get("status") == PATH_OBSTRUCTION
        endpoint_clear = path.get("status") == NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN
        if not (endpoint_obstructed or endpoint_clear):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: unsupported endpoint path status"
            )
        baseline_obstructed = baseline.status == PATH_OBSTRUCTED
        if endpoint_obstructed != baseline_obstructed:
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: endpoint/planner obstruction status mismatch"
            )

        row: dict[str, Any] = {
            "canonical_endpoint_index": index,
            "endpoint_id": endpoint_id,
            "status": "PASS",
            "obstructed": endpoint_obstructed,
            "endpoint_path_status": path.get("status"),
            "baseline_path_status": baseline.status,
        }
        if endpoint_obstructed:
            obstruction = baseline.first_obstruction
            refined = baseline.refined_first_obstruction
            precise_angle = path.get("angle_rad")
            if (
                obstruction is None
                or refined is None
                or not isinstance(precise_angle, (int, float))
                or tuple(path.get("link_pair", ())) != refined.link_pair
                or path.get("relation") != refined.relation
                or obstruction.sample_index <= 0
                or obstruction.sample_index >= baseline.planned_sample_count
                or baseline.planned_sample_count <= 1
                or baseline.evaluated_sample_count != obstruction.sample_index + 1
            ):
                raise GeometryPathPlannerV5Error(
                    f"STOP: {endpoint_id}: endpoint/planner first blocker mismatch"
                )
            sample_angle = obstruction.joint_positions_rad[plan.joint_name]
            intervals = baseline.planned_sample_count - 1
            expected_sample_progress = obstruction.sample_index / intervals
            previous_progress = (obstruction.sample_index - 1) / intervals
            previous_angle = target * (obstruction.sample_index - 1) / intervals
            precise_angle = float(precise_angle)
            refined_angle = refined.contact_joint_positions_rad[plan.joint_name]
            pose_names = set(baseline.start_joint_positions_rad)
            if (
                not pose_names
                or set(baseline.end_joint_positions_rad) != pose_names
                or set(obstruction.joint_positions_rad) != pose_names
                or set(refined.clear_joint_positions_rad) != pose_names
                or set(refined.contact_joint_positions_rad) != pose_names
                or not math.isfinite(refined.clear_progress)
                or not math.isfinite(refined.contact_progress)
                or not math.isfinite(refined.bisection_resolution_rad)
                or refined.bisection_resolution_rad <= 0.0
                or not math.isclose(
                    refined.bisection_resolution_rad,
                    DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD,
                    rel_tol=0.0,
                    abs_tol=1e-15,
                )
                or not isinstance(refined.bisection_iterations, int)
                or isinstance(refined.bisection_iterations, bool)
                or not 0
                <= refined.bisection_iterations
                <= DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
            ):
                raise GeometryPathPlannerV5Error(
                    f"STOP: {endpoint_id}: invalid planner refinement evidence"
                )
            refined_width = max(
                abs(
                    refined.contact_joint_positions_rad[name]
                    - refined.clear_joint_positions_rad[name]
                )
                for name in pose_names
            )
            tolerance = max(
                float(resolution),
                refined.bisection_resolution_rad,
            ) + 1e-12
            lower = min(previous_angle, sample_angle) - tolerance
            upper = max(previous_angle, sample_angle) + tolerance
            delta = abs(refined_angle - precise_angle)
            effective_interval = abs(target) / intervals
            if (
                not lower <= precise_angle <= upper
                or not lower <= refined_angle <= upper
                or delta > tolerance
                or not math.isclose(
                    obstruction.progress,
                    expected_sample_progress,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                or not (
                    previous_progress
                    <= refined.clear_progress
                    < refined.contact_progress
                    <= expected_sample_progress
                )
                or refined_width > refined.bisection_resolution_rad + 1e-12
                or any(
                    not math.isclose(
                        refined_pose[name],
                        baseline.start_joint_positions_rad[name]
                        + (
                            baseline.end_joint_positions_rad[name]
                            - baseline.start_joint_positions_rad[name]
                        )
                        * progress,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                    for progress, refined_pose in (
                        (
                            refined.clear_progress,
                            refined.clear_joint_positions_rad,
                        ),
                        (
                            refined.contact_progress,
                            refined.contact_joint_positions_rad,
                        ),
                    )
                    for name in pose_names
                )
                or any(
                    not math.isclose(
                        obstruction.joint_positions_rad[name],
                        baseline.start_joint_positions_rad[name]
                        + (
                            baseline.end_joint_positions_rad[name]
                            - baseline.start_joint_positions_rad[name]
                        )
                        * expected_sample_progress,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                    for name in pose_names
                )
            ):
                raise GeometryPathPlannerV5Error(
                    f"STOP: {endpoint_id}: refined endpoint/planner obstruction "
                    "differs beyond the declared bisection resolution"
                )
            max_delta = max(max_delta, delta)
            row.update(
                {
                    "link_pair": list(refined.link_pair),
                    "relation": refined.relation,
                    "precise_obstruction_angle_rad": precise_angle,
                    "first_sampled_obstruction_angle_rad": sample_angle,
                    "planner_refined_obstruction_angle_rad": refined_angle,
                    "precise_to_refined_delta_rad": delta,
                    "effective_sample_interval_rad": effective_interval,
                    "endpoint_bisection_resolution_rad": float(resolution),
                    "planner_bisection_resolution_rad": (
                        refined.bisection_resolution_rad
                    ),
                    "planner_refined_bracket_width_rad": refined_width,
                }
            )
        elif (
            baseline.first_obstruction is not None
            or baseline.refined_first_obstruction is not None
        ):
            raise GeometryPathPlannerV5Error(
                f"STOP: {endpoint_id}: collision-free baseline retains an obstruction"
            )
        rows.append(row)

    return {
        "status": "PASS",
        "endpoint_count": 24,
        "consistent_endpoint_count": 24,
        "obstructed_count": sum(row["obstructed"] for row in rows),
        "collision_free_count": sum(not row["obstructed"] for row in rows),
        "max_precise_to_refined_delta_rad": max_delta,
        "coarse_step_rad": path_step_rad,
        "rows": rows,
    }


def _path_record(path: PathValidationV5 | None) -> dict[str, Any] | None:
    if path is None:
        return None
    obstruction = path.first_obstruction
    refined = path.refined_first_obstruction
    return {
        "path_id": path.path_id,
        "start_joint_positions_rad": path.start_joint_positions_rad,
        "end_joint_positions_rad": path.end_joint_positions_rad,
        "moving_joint_names": list(path.moving_joint_names),
        "active_pair_excluded": list(path.active_pair_excluded) if path.active_pair_excluded else None,
        "status": path.status,
        "collision_free": path.collision_free,
        "planned_sample_count": path.planned_sample_count,
        "evaluated_sample_count": path.evaluated_sample_count,
        "first_sampled_obstruction": (
            {
                "sample_index": obstruction.sample_index,
                "progress": obstruction.progress,
                "joint_positions_rad": obstruction.joint_positions_rad,
                "link_pair": list(obstruction.link_pair),
                "relation": obstruction.relation,
            }
            if obstruction is not None
            else None
        ),
        "refined_first_obstruction": (
            {
                "clear_progress": refined.clear_progress,
                "contact_progress": refined.contact_progress,
                "clear_joint_positions_rad": refined.clear_joint_positions_rad,
                "contact_joint_positions_rad": refined.contact_joint_positions_rad,
                "link_pair": list(refined.link_pair),
                "relation": refined.relation,
                "bisection_resolution_rad": refined.bisection_resolution_rad,
                "bisection_iterations": refined.bisection_iterations,
            }
            if refined is not None
            else None
        ),
        "min_clearance_m": path.min_clearance_m,
        "min_clearance_kind": path.min_clearance_kind,
        "clearance_samples_evaluated": path.clearance_samples_evaluated,
        "sampled_configuration_sha256": path.sampled_configuration_sha256,
        "reversed_sampled_configuration_sha256": (
            path.reversed_sampled_configuration_sha256
        ),
        "reverse_validation_of": path.reverse_validation_of,
    }


def _objective_record(objective: ParkingObjectiveV5 | None) -> dict[str, Any] | None:
    if objective is None:
        return None
    return {
        "zero_unintended_intersection": objective.zero_unintended_intersection,
        "min_clearance_m": objective.min_clearance_m,
        "min_clearance_kind": objective.min_clearance_kind,
        "displacement_l2_rad": objective.displacement_l2_rad,
        "deterministic_tie_break": [list(item) for item in objective.deterministic_tie_break],
    }


def parking_plan_record(plan: EndpointParkingPlanV5) -> dict[str, Any]:
    return {
        "canonical_endpoint_index": plan.canonical_endpoint_index,
        "endpoint_id": plan.endpoint_id,
        "joint_name": plan.joint_name,
        "limit_side": plan.limit_side,
        "active_branch_id": plan.active_branch_id,
        "declared_limit_rad": plan.declared_limit_rad,
        "target_angle_rad": plan.target_angle_rad,
        "target_delta_from_declared_rad": plan.target_delta_from_declared_rad,
        "target_within_urdf_limits": plan.target_within_urdf_limits,
        "target_domain": plan.target_domain,
        "target_source": plan.target_source,
        "allowed_endpoint_contact_pair": list(plan.allowed_endpoint_contact_pair),
        "start_configuration_valid": plan.start_configuration_valid,
        "baseline_task_path": _path_record(plan.baseline_task_path),
        "first_sampled_blocking_pair": (
            list(plan.baseline_task_path.first_obstruction.link_pair)
            if plan.baseline_task_path.first_obstruction is not None
            else None
        ),
        "first_sampled_blocking_relation": (
            plan.baseline_task_path.first_obstruction.relation
            if plan.baseline_task_path.first_obstruction is not None
            else None
        ),
        "first_refined_blocking_pair": (
            list(plan.first_blocking_pair) if plan.first_blocking_pair else None
        ),
        "first_refined_blocking_relation": plan.first_blocking_relation,
        "relevant_movable_joint_names": list(plan.relevant_movable_joint_names),
        "outcome": plan.outcome,
        "parking_degrees_of_freedom": plan.parking_degrees_of_freedom,
        "parking_configuration_rad": plan.parking_configuration_rad,
        "declared_search_domain": {
            "kind": "DETERMINISTIC_SAMPLED_GRID_WITHIN_URDF_LIMITS",
            "joint_limit_bounds_rad": {
                name: list(domain)
                for name, domain in plan.declared_joint_limit_bounds_rad.items()
            },
            "one_dof_grid_rad": {
                name: list(values)
                for name, values in plan.one_dof_search_grid_rad.items()
            },
            "two_dof_grid_rad": {
                name: list(values)
                for name, values in plan.two_dof_search_grid_rad.items()
            },
        },
        "path_in": _path_record(plan.path_in),
        "task_path": _path_record(plan.task_path),
        "task_return": _path_record(plan.task_return),
        "path_out": _path_record(plan.path_out),
        "objective": _objective_record(plan.objective),
        "evaluated_1d_candidates": plan.evaluated_1d_candidates,
        "evaluated_2d_candidates": plan.evaluated_2d_candidates,
    }


def parking_content_sha256(artifact: dict[str, Any]) -> str:
    schema_version = artifact.get("schema_version")
    if schema_version == PARKING_SCHEMA_V1:
        # Frozen G9 compatibility: preserve the original v1 algorithm exactly.
        content = {
            key: value
            for key, value in artifact.items()
            if key not in {"generation_metadata", "semantic_content_sha256"}
        }
    elif schema_version == PARKING_SCHEMA_V2:
        # V2 is deliberately closed: materialization identity and execution
        # telemetry cannot become geometry merely by adding a new top-level key.
        content = {key: artifact.get(key) for key in PARKING_V2_SEMANTIC_KEYS}
    else:
        raise GeometryPathPlannerV5Error(
            f"cannot hash unknown parking schema: {schema_version!r}"
        )
    canonical = json.dumps(content, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _validate_serialized_refined_obstruction(
    segment: dict[str, Any],
    *,
    endpoint_id: Any,
    declared_resolution_rad: float,
    max_iterations: int,
) -> None:
    """Validate that serialized refinement evidence proves its own contract."""

    sampled = segment.get("first_sampled_obstruction")
    refined = segment.get("refined_first_obstruction")
    planned = segment.get("planned_sample_count")
    evaluated = segment.get("evaluated_sample_count")
    if not isinstance(sampled, dict) or not isinstance(refined, dict):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: obstructed path lacks sampled/refined evidence"
        )
    sample_index = sampled.get("sample_index")
    if (
        not isinstance(planned, int)
        or isinstance(planned, bool)
        or planned < 2
        or not isinstance(evaluated, int)
        or isinstance(evaluated, bool)
        or not isinstance(sample_index, int)
        or isinstance(sample_index, bool)
        or not 1 <= sample_index < planned
        or evaluated != sample_index + 1
    ):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: invalid sampled obstruction index"
        )
    intervals = planned - 1
    expected_sample_progress = sample_index / intervals
    previous_progress = (sample_index - 1) / intervals

    def finite_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )

    sample_progress = sampled.get("progress")
    clear_progress = refined.get("clear_progress")
    contact_progress = refined.get("contact_progress")
    if (
        not finite_number(sample_progress)
        or not math.isclose(
            float(sample_progress),
            expected_sample_progress,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        or not finite_number(clear_progress)
        or not finite_number(contact_progress)
        or not (
            previous_progress
            <= float(clear_progress)
            < float(contact_progress)
            <= expected_sample_progress
        )
    ):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: refined obstruction progress is outside its sampled bracket"
        )

    start = segment.get("start_joint_positions_rad")
    end = segment.get("end_joint_positions_rad")
    sampled_pose = sampled.get("joint_positions_rad")
    clear_pose = refined.get("clear_joint_positions_rad")
    contact_pose = refined.get("contact_joint_positions_rad")
    pose_records = (start, end, sampled_pose, clear_pose, contact_pose)
    if not all(isinstance(pose, dict) and pose for pose in pose_records):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: refined obstruction pose evidence is incomplete"
        )
    pose_names = set(start)
    if any(set(pose) != pose_names for pose in pose_records[1:]):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: refined obstruction pose joints are inconsistent"
        )
    if any(
        not finite_number(value)
        for pose in pose_records
        for value in pose.values()
    ):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: refined obstruction pose is not finite"
        )

    def interpolation_matches(pose: dict[str, Any], progress: float) -> bool:
        return all(
            math.isclose(
                float(pose[name]),
                float(start[name])
                + (float(end[name]) - float(start[name])) * progress,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            for name in pose_names
        )

    if (
        not interpolation_matches(sampled_pose, expected_sample_progress)
        or not interpolation_matches(clear_pose, float(clear_progress))
        or not interpolation_matches(contact_pose, float(contact_progress))
    ):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: refined obstruction poses do not match path interpolation"
        )
    bracket_width = max(
        abs(float(contact_pose[name]) - float(clear_pose[name]))
        for name in pose_names
    )
    if bracket_width > declared_resolution_rad + 1e-12:
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: refined obstruction bracket exceeds declared resolution"
        )

    link_pair = refined.get("link_pair")
    relation = refined.get("relation")
    sampled_link_pair = sampled.get("link_pair")
    sampled_relation = sampled.get("relation")
    iterations = refined.get("bisection_iterations")
    resolution = refined.get("bisection_resolution_rad")
    if (
        not isinstance(link_pair, list)
        or len(link_pair) != 2
        or any(not isinstance(link, str) or not link for link in link_pair)
        or not isinstance(relation, str)
        or not relation
        or not isinstance(sampled_link_pair, list)
        or len(sampled_link_pair) != 2
        or any(
            not isinstance(link, str) or not link for link in sampled_link_pair
        )
        or not isinstance(sampled_relation, str)
        or not sampled_relation
        or not finite_number(resolution)
        or not math.isclose(
            float(resolution),
            declared_resolution_rad,
            rel_tol=0.0,
            abs_tol=1e-15,
        )
        or not isinstance(iterations, int)
        or isinstance(iterations, bool)
        or not 0 <= iterations <= max_iterations
    ):
        raise GeometryPathPlannerV5Error(
            f"{endpoint_id}: invalid refined obstruction contract"
        )


def validate_parking_artifact(artifact: dict[str, Any]) -> None:
    schema_version = artifact.get("schema_version")
    if schema_version not in {PARKING_SCHEMA_V1, PARKING_SCHEMA_V2}:
        raise GeometryPathPlannerV5Error(
            f"unexpected parking schema: {schema_version!r}"
        )
    if schema_version == PARKING_SCHEMA_V2 and set(artifact) != PARKING_V2_TOP_LEVEL_KEYS:
        raise GeometryPathPlannerV5Error(
            "parking v2 top-level envelope mismatch: "
            f"expected={sorted(PARKING_V2_TOP_LEVEL_KEYS)}, actual={sorted(artifact)}"
        )
    stored = artifact.get("semantic_content_sha256")
    recomputed = parking_content_sha256(artifact)
    if stored != recomputed:
        raise GeometryPathPlannerV5Error(
            f"parking semantic SHA mismatch: stored={stored!r}, recomputed={recomputed!r}"
        )
    forbidden_policy_keys = {
        "accepted",
        "safety_passed",
        "safety_threshold_m",
        "min_clearance_pass_m",
        "clearance_gate_result",
        "parking_seed_angles_deg",
    }

    def walk_keys(value: Any) -> Iterable[str]:
        if isinstance(value, dict):
            for key, child in value.items():
                yield str(key)
                yield from walk_keys(child)
        elif isinstance(value, list):
            for child in value:
                yield from walk_keys(child)

    forbidden_found = sorted(forbidden_policy_keys.intersection(walk_keys(artifact)))
    if forbidden_found:
        raise GeometryPathPlannerV5Error(
            f"safety/policy fields leaked into raw parking artifact: {forbidden_found}"
        )
    plans = artifact.get("plans")
    if not isinstance(plans, list) or len(plans) != 24:
        raise GeometryPathPlannerV5Error("parking artifact must contain exactly 24 plans")
    endpoint_ids = [plan.get("endpoint_id") for plan in plans]
    if len(set(endpoint_ids)) != 24:
        raise GeometryPathPlannerV5Error("parking endpoint IDs are not unique")
    if [plan.get("canonical_endpoint_index") for plan in plans] != list(range(24)):
        raise GeometryPathPlannerV5Error(
            "parking plans are not in canonical model-derived endpoint order"
        )

    input_reference = artifact.get("input_geometry_profile")
    if not isinstance(input_reference, dict):
        raise GeometryPathPlannerV5Error("parking input geometry reference is missing")
    if schema_version == PARKING_SCHEMA_V1:
        required_reference = {
            "relative_path",
            "file_sha256",
            "semantic_content_sha256",
        }
        if not required_reference.issubset(input_reference):
            raise GeometryPathPlannerV5Error("parking v1 input reference is incomplete")
    else:
        expected_keys = {"schema_version", "semantic_content_sha256"}
        if set(input_reference) != expected_keys:
            raise GeometryPathPlannerV5Error(
                "parking v2 semantic input reference contains audit file metadata"
            )
        if input_reference.get("schema_version") != GEOMETRY_PROFILE_SCHEMA_VERSION:
            raise GeometryPathPlannerV5Error("parking v2 geometry schema reference mismatch")
        audit_reference = artifact.get("generation_metadata", {}).get(
            "audit_input_geometry_profile"
        )
        if not isinstance(audit_reference, dict) or set(audit_reference) != {
            "relative_path",
            "file_sha256",
        }:
            raise GeometryPathPlannerV5Error(
                "parking v2 non-semantic audit input reference is incomplete"
            )
        audit_path = audit_reference.get("relative_path")
        if (
            not isinstance(audit_path, str)
            or not audit_path
            or Path(audit_path).is_absolute()
            or ".." in Path(audit_path).parts
        ):
            raise GeometryPathPlannerV5Error(
                "parking v2 audit input path is not repository-relative"
            )
        generation_metadata = artifact.get("generation_metadata")
        if not isinstance(generation_metadata, dict):
            raise GeometryPathPlannerV5Error("parking v2 generation metadata is missing")
        allowed_generation_keys = {
            "generated_at_utc",
            "worker_count",
            "runtime_seconds",
            "audit_input_geometry_profile",
            "execution_audit",
        }
        required_generation_keys = allowed_generation_keys - {"execution_audit"}
        if not required_generation_keys.issubset(generation_metadata) or not set(
            generation_metadata
        ).issubset(allowed_generation_keys):
            raise GeometryPathPlannerV5Error(
                "parking v2 generation metadata envelope mismatch"
            )
        try:
            datetime.fromisoformat(str(generation_metadata["generated_at_utc"]))
        except (TypeError, ValueError) as exc:
            raise GeometryPathPlannerV5Error(
                "parking v2 generation timestamp is not ISO-8601"
            ) from exc
        if generation_metadata.get("worker_count") not in {1, 4}:
            raise GeometryPathPlannerV5Error(
                "parking v2 worker_count must be exactly 1 or 4"
            )
        runtime = generation_metadata.get("runtime_seconds")
        if runtime is not None and (
            not isinstance(runtime, (int, float))
            or not math.isfinite(float(runtime))
            or float(runtime) < 0.0
        ):
            raise GeometryPathPlannerV5Error(
                "parking v2 runtime must be null or finite and non-negative"
            )
    for reference, digest_keys in (
        (input_reference, ("semantic_content_sha256",)),
        (
            artifact.get("generation_metadata", {}).get(
                "audit_input_geometry_profile", {}
            ),
            ("file_sha256",) if schema_version == PARKING_SCHEMA_V2 else (),
        ),
    ):
        for digest_key in digest_keys:
            digest = reference.get(digest_key)
            if (
                not isinstance(digest, str)
                or len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise GeometryPathPlannerV5Error(
                    f"invalid parking geometry reference {digest_key}"
                )

    provenance = artifact.get("provenance")
    if not isinstance(provenance, dict) or set(provenance) != {
        "source_file_sha256",
        "source_combined_sha256",
    }:
        raise GeometryPathPlannerV5Error("parking source provenance is incomplete")
    source_manifest = provenance.get("source_file_sha256")
    if not isinstance(source_manifest, dict) or not source_manifest:
        raise GeometryPathPlannerV5Error("parking source manifest is empty")
    for source_path, digest in source_manifest.items():
        if (
            not isinstance(source_path, str)
            or not source_path
            or Path(source_path).is_absolute()
            or ".." in Path(source_path).parts
        ):
            raise GeometryPathPlannerV5Error(
                f"invalid parking source path: {source_path!r}"
            )
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise GeometryPathPlannerV5Error(
                f"invalid parking source SHA for {source_path!r}"
            )
    if provenance.get("source_combined_sha256") != _combined_source_sha256(
        source_manifest
    ):
        raise GeometryPathPlannerV5Error("parking combined source SHA mismatch")

    parameters_record = artifact.get("parameters", {})
    if not isinstance(parameters_record, dict):
        raise GeometryPathPlannerV5Error("parking parameter record is invalid")
    refinement_keys = {
        "obstruction_bisection_resolution_rad",
        "max_obstruction_bisection_iterations",
    }
    has_refinement_contract = refinement_keys.issubset(parameters_record)
    if refinement_keys.intersection(parameters_record) and not has_refinement_contract:
        raise GeometryPathPlannerV5Error(
            "parking obstruction refinement parameter contract is incomplete"
        )
    if has_refinement_contract:
        declared_refinement_resolution = parameters_record.get(
            "obstruction_bisection_resolution_rad"
        )
        declared_refinement_iterations = parameters_record.get(
            "max_obstruction_bisection_iterations"
        )
        if (
            not isinstance(declared_refinement_resolution, (int, float))
            or isinstance(declared_refinement_resolution, bool)
            or not math.isfinite(float(declared_refinement_resolution))
            or float(declared_refinement_resolution) <= 0.0
            or not math.isclose(
                float(declared_refinement_resolution),
                DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD,
                rel_tol=0.0,
                abs_tol=1e-15,
            )
            or not isinstance(declared_refinement_iterations, int)
            or isinstance(declared_refinement_iterations, bool)
            or declared_refinement_iterations
            != DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
        ):
            raise GeometryPathPlannerV5Error(
                "parking obstruction refinement parameter contract is invalid"
            )
        declared_refinement_resolution = float(declared_refinement_resolution)
    for plan in plans:
        outcome = plan.get("outcome")
        task_path = plan.get("task_path")
        task_return = plan.get("task_return")
        path_in = plan.get("path_in")
        path_out = plan.get("path_out")
        expected_target_domain = (
            "EXECUTABLE_URDF_DOMAIN"
            if plan.get("target_within_urdf_limits")
            else "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS"
        )
        if plan.get("target_domain") != expected_target_domain:
            raise GeometryPathPlannerV5Error(
                f"{plan.get('endpoint_id')}: target-domain classification is inconsistent"
            )
        if not math.isclose(
            float(plan.get("target_angle_rad")),
            float(plan.get("declared_limit_rad"))
            + float(plan.get("target_delta_from_declared_rad")),
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise GeometryPathPlannerV5Error(
                f"{plan.get('endpoint_id')}: target delta is inconsistent"
            )
        if task_path.get("active_pair_excluded") != plan.get(
            "allowed_endpoint_contact_pair"
        ):
            raise GeometryPathPlannerV5Error(
                f"{plan.get('endpoint_id')}: allowed endpoint contact pair is inconsistent"
            )
        if has_refinement_contract:
            baseline = plan.get("baseline_task_path", {})
            sampled = baseline.get("first_sampled_obstruction")
            refined = baseline.get("refined_first_obstruction")
            if plan.get("first_sampled_blocking_pair") != (
                sampled.get("link_pair") if isinstance(sampled, dict) else None
            ) or plan.get("first_sampled_blocking_relation") != (
                sampled.get("relation") if isinstance(sampled, dict) else None
            ):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: sampled blocker summary is inconsistent"
                )
            if plan.get("first_refined_blocking_pair") != (
                refined.get("link_pair") if isinstance(refined, dict) else None
            ) or plan.get("first_refined_blocking_relation") != (
                refined.get("relation") if isinstance(refined, dict) else None
            ):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: refined blocker summary is inconsistent"
                )
        for segment in (plan.get("baseline_task_path"), path_in, task_path, task_return, path_out):
            if segment is None:
                continue
            expected_collision_free = segment.get("status") == PATH_COLLISION_FREE
            if segment.get("collision_free") is not expected_collision_free:
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: path status/collision-free mismatch"
                )
            planned = segment.get("planned_sample_count")
            evaluated = segment.get("evaluated_sample_count")
            if not (
                isinstance(planned, int)
                and isinstance(evaluated, int)
                and 1 <= evaluated <= planned
            ):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: invalid planned/evaluated sample counts"
                )
            if has_refinement_contract:
                sampled = segment.get("first_sampled_obstruction")
                refined = segment.get("refined_first_obstruction")
                if expected_collision_free and (sampled is not None or refined is not None):
                    raise GeometryPathPlannerV5Error(
                        f"{plan.get('endpoint_id')}: collision-free path retains obstruction evidence"
                    )
                if not expected_collision_free:
                    _validate_serialized_refined_obstruction(
                        segment,
                        endpoint_id=plan.get("endpoint_id"),
                        declared_resolution_rad=declared_refinement_resolution,
                        max_iterations=declared_refinement_iterations,
                    )
        if task_return is not None:
            if task_return.get("reverse_validation_of") != task_path.get("path_id"):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: task return does not reference task path"
                )
            if (
                task_return.get("sampled_configuration_sha256")
                != task_path.get("reversed_sampled_configuration_sha256")
                or task_return.get("reversed_sampled_configuration_sha256")
                != task_path.get("sampled_configuration_sha256")
            ):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: task return is not the exact sampled reverse"
                )
        if path_out is not None:
            if path_in is None or path_out.get("reverse_validation_of") != path_in.get("path_id"):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: path-out does not reference path-in"
                )
            if (
                path_out.get("sampled_configuration_sha256")
                != path_in.get("reversed_sampled_configuration_sha256")
                or path_out.get("reversed_sampled_configuration_sha256")
                != path_in.get("sampled_configuration_sha256")
            ):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: path-out is not the exact sampled reverse"
                )
        feasible = outcome in {
            PARKING_NOT_NEEDED,
            PARKING_FEASIBLE_1DOF,
            PARKING_FEASIBLE_2DOF,
        }
        if feasible and (task_return is None or not task_return.get("collision_free")):
            raise GeometryPathPlannerV5Error(
                f"{plan.get('endpoint_id')}: feasible plan lacks validated task return"
            )
        if outcome in {PARKING_FEASIBLE_1DOF, PARKING_FEASIBLE_2DOF}:
            if not all(
                segment is not None and segment.get("collision_free")
                for segment in (path_in, task_path, task_return, path_out)
            ):
                raise GeometryPathPlannerV5Error(
                    f"{plan.get('endpoint_id')}: parking plan lacks a complete collision-free sequence"
                )

    if schema_version == PARKING_SCHEMA_V2:
        outcome_counts: dict[str, int] = {}
        for plan in plans:
            outcome = str(plan.get("outcome"))
            outcome_counts[outcome] = outcome_counts.get(outcome, 0) + 1

        def has_complete_sequence(plan: dict[str, Any]) -> bool:
            if plan.get("outcome") == PARKING_NOT_NEEDED:
                return bool(
                    plan.get("task_path", {}).get("collision_free")
                    and plan.get("task_return")
                    and plan["task_return"].get("collision_free")
                )
            if plan.get("outcome") in {
                PARKING_FEASIBLE_1DOF,
                PARKING_FEASIBLE_2DOF,
            }:
                return all(
                    isinstance(plan.get(name), dict)
                    and plan[name].get("collision_free")
                    for name in ("path_in", "task_path", "task_return", "path_out")
                )
            return False

        complete_count = sum(has_complete_sequence(plan) for plan in plans)
        expected_summary = {
            "endpoint_plan_count": 24,
            "outcome_counts": outcome_counts,
            "all_start_configurations_valid": all(
                plan.get("start_configuration_valid") is True for plan in plans
            ),
            "baseline_obstructed_count": sum(
                plan.get("baseline_task_path", {}).get("collision_free") is False
                for plan in plans
            ),
            "geometry_feasible_complete_sequence_count": complete_count,
            "no_complete_sequence_count": 24 - complete_count,
            "executable_urdf_target_count": sum(
                plan.get("target_within_urdf_limits") is True for plan in plans
            ),
            "diagnostic_target_outside_urdf_limits_count": sum(
                plan.get("target_within_urdf_limits") is False for plan in plans
            ),
            "external_safety_policy_applied": False,
        }
        if artifact.get("summary") != expected_summary:
            raise GeometryPathPlannerV5Error("parking v2 summary does not match its plans")


def _combined_source_sha256(source_manifest: dict[str, str]) -> str:
    canonical = "".join(
        f"{name}:{digest}\n" for name, digest in sorted(source_manifest.items())
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _plan_has_complete_sequence(plan: EndpointParkingPlanV5) -> bool:
    if plan.outcome == PARKING_NOT_NEEDED:
        return plan.task_path.collision_free and bool(
            plan.task_return and plan.task_return.collision_free
        )
    if plan.outcome in {PARKING_FEASIBLE_1DOF, PARKING_FEASIBLE_2DOF}:
        return all(
            path is not None and path.collision_free
            for path in (plan.path_in, plan.task_path, plan.task_return, plan.path_out)
        )
    return False


def build_parking_artifact(
    plans: Iterable[EndpointParkingPlanV5],
    *,
    geometry_profile_reference: dict[str, str],
    source_file_sha256: dict[str, str],
    parameters: ParkingPlannerParametersV5,
    worker_count: int = 1,
    runtime_seconds: float | None = None,
) -> dict[str, Any]:
    plans = tuple(sorted(plans, key=lambda plan: plan.canonical_endpoint_index))
    if len(plans) != 24 or len({plan.endpoint_id for plan in plans}) != 24:
        raise GeometryPathPlannerV5Error(
            f"parking artifact requires exact 24/24 endpoint coverage, got {len(plans)}"
        )
    required_profile_reference = {
        "relative_path",
        "file_sha256",
        "semantic_content_sha256",
    }
    missing_reference = required_profile_reference - set(geometry_profile_reference)
    if missing_reference:
        raise GeometryPathPlannerV5Error(
            f"geometry profile reference missing fields: {sorted(missing_reference)}"
        )
    if not source_file_sha256:
        raise GeometryPathPlannerV5Error("path planner source provenance is empty")
    if worker_count <= 0:
        raise GeometryPathPlannerV5Error(f"invalid worker count: {worker_count}")
    records = [parking_plan_record(plan) for plan in plans]
    outcomes: dict[str, int] = {}
    for plan in plans:
        outcomes[plan.outcome] = outcomes.get(plan.outcome, 0) + 1
    complete_count = sum(_plan_has_complete_sequence(plan) for plan in plans)
    artifact: dict[str, Any] = {
        "schema_version": PARKING_SCHEMA_V1,
        "generation_metadata": {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "worker_count": worker_count,
            "runtime_seconds": (
                round(float(runtime_seconds), 6)
                if runtime_seconds is not None
                else None
            ),
        },
        "input_geometry_profile": dict(sorted(geometry_profile_reference.items())),
        "provenance": {
            "source_file_sha256": dict(sorted(source_file_sha256.items())),
            "source_combined_sha256": _combined_source_sha256(source_file_sha256),
        },
        "parameters": {
            "path_step_rad": parameters.path_step_rad,
            "obstruction_bisection_resolution_rad": (
                DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD
            ),
            "max_obstruction_bisection_iterations": (
                DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
            ),
            "one_dof_grid_divisions": parameters.one_dof_grid_divisions,
            "two_dof_grid_divisions": parameters.two_dof_grid_divisions,
            "clearance_sample_stride": parameters.clearance_sample_stride,
            "path_feasibility_semantics": (
                "intersection/no-intersection at every listed linear configuration sample, "
                "with the first clear/contact sample interval locally bisected to the "
                "declared obstruction resolution; not a continuous swept-volume proof"
            ),
            "search_domain_rule": (
                "finite uniform normalized grids listed per plan, bounded by each relevant "
                "joint's URDF limits; 1-DOF grid first and 2-DOF grid only if needed"
            ),
            "clearance_semantics": (
                "minimum exact-or-conservative-lower-bound clearance over non-adjacent pairs "
                "and the symmetric sampled subset declared by clearance_sample_stride; "
                "revolute and fixed joint interfaces are not clearance-policy surfaces"
            ),
            "intentional_endpoint_contact_rule": (
                "the active revolute parent-child pair is excluded only from its task sweep; "
                "the target geometric contact is not a path obstruction"
            ),
            "two_dof_path_rule": (
                "path-in and path-out use simultaneous linear interpolation of both parked "
                "joint coordinates over the declared sampled configuration set"
            ),
            "optimization_order": [
                "zero unintended mesh intersection (declared endpoint contact pair allowed)",
                "maximize minimum sampled geometric clearance evidence",
                "minimize L2 displacement from home",
                "deterministic topology/value tie-break",
            ],
        },
        "summary": {
            "endpoint_plan_count": len(plans),
            "outcome_counts": outcomes,
            "all_start_configurations_valid": all(plan.start_configuration_valid for plan in plans),
            "baseline_obstructed_count": sum(
                not plan.baseline_task_path.collision_free for plan in plans
            ),
            "geometry_feasible_complete_sequence_count": complete_count,
            "no_complete_sequence_count": len(plans) - complete_count,
            "executable_urdf_target_count": sum(
                plan.target_within_urdf_limits for plan in plans
            ),
            "diagnostic_target_outside_urdf_limits_count": sum(
                not plan.target_within_urdf_limits for plan in plans
            ),
            "external_safety_policy_applied": False,
        },
        "plans": records,
    }
    artifact["semantic_content_sha256"] = parking_content_sha256(artifact)
    validate_parking_artifact(artifact)
    return artifact


def build_parking_artifact_v2(
    plans: Iterable[EndpointParkingPlanV5],
    *,
    geometry_profile_semantic_sha256: str,
    audit_geometry_profile_reference: dict[str, str],
    source_file_sha256: dict[str, str],
    parameters: ParkingPlannerParametersV5,
    worker_count: int,
    runtime_seconds: float | None = None,
) -> dict[str, Any]:
    """Build the G11 deterministic artifact without semantic file identity.

    The exact intermediate profile path and file SHA differ legitimately
    between workers=1 and workers=4 because generation metadata differs. They
    remain available for audit under the excluded generation metadata, while
    schema plus semantic SHA are the only semantic input identity.
    """

    required_audit = {"relative_path", "file_sha256"}
    missing = required_audit - set(audit_geometry_profile_reference)
    if missing:
        raise GeometryPathPlannerV5Error(
            f"parking v2 audit profile reference missing: {sorted(missing)}"
        )
    v1 = build_parking_artifact(
        plans,
        geometry_profile_reference={
            "relative_path": audit_geometry_profile_reference["relative_path"],
            "file_sha256": audit_geometry_profile_reference["file_sha256"],
            "semantic_content_sha256": geometry_profile_semantic_sha256,
        },
        source_file_sha256=source_file_sha256,
        parameters=parameters,
        worker_count=worker_count,
        runtime_seconds=runtime_seconds,
    )
    artifact = json.loads(json.dumps(v1))
    artifact["schema_version"] = PARKING_SCHEMA_V2
    artifact["input_geometry_profile"] = {
        "schema_version": GEOMETRY_PROFILE_SCHEMA_VERSION,
        "semantic_content_sha256": geometry_profile_semantic_sha256,
    }
    artifact["generation_metadata"]["audit_input_geometry_profile"] = {
        "relative_path": audit_geometry_profile_reference["relative_path"],
        "file_sha256": audit_geometry_profile_reference["file_sha256"],
    }
    artifact.pop("semantic_content_sha256", None)
    artifact["semantic_content_sha256"] = parking_content_sha256(artifact)
    validate_parking_artifact(artifact)
    return artifact


def derive_geometry_profile_with_path_plans(
    geometry_profile: dict[str, Any],
    parking_artifact: dict[str, Any],
) -> dict[str, Any]:
    """Create a new canonical V5 profile; never mutate the frozen G7 input."""

    validate_pure_geometry_profile(geometry_profile)
    validate_parking_artifact(parking_artifact)
    input_semantic = geometry_profile["semantic_content_sha256"]
    if (
        parking_artifact["input_geometry_profile"]["semantic_content_sha256"]
        != input_semantic
    ):
        raise GeometryPathPlannerV5Error(
            "parking artifact does not reference the supplied geometry profile"
        )
    derived = json.loads(json.dumps(geometry_profile))
    derived["path_plans"] = parking_artifact["plans"]
    if parking_artifact["schema_version"] == PARKING_SCHEMA_V1:
        path_planner_provenance = {
            "input_geometry_profile": parking_artifact["input_geometry_profile"],
            **parking_artifact["provenance"],
        }
    else:
        path_planner_provenance = {
            "parking_schema_version": PARKING_SCHEMA_V2,
            "input_geometry_profile_semantic_sha256": input_semantic,
            "parking_semantic_content_sha256": parking_artifact[
                "semantic_content_sha256"
            ],
            **parking_artifact["provenance"],
        }
    derived.setdefault("provenance", {})["path_planner"] = path_planner_provenance
    derived.setdefault("analysis_parameters", {})["path_planning"] = (
        parking_artifact["parameters"]
    )
    derived.setdefault("generation_metadata", {})["path_planning"] = (
        parking_artifact["generation_metadata"]
    )
    derived.pop("semantic_content_sha256", None)
    derived["semantic_content_sha256"] = geometry_profile_semantic_sha256(derived)
    validate_pure_geometry_profile(derived)
    return derived


def render_parking_report(artifact: dict[str, Any]) -> str:
    validate_parking_artifact(artifact)
    if artifact["schema_version"] == PARKING_SCHEMA_V1:
        audit_reference = artifact["input_geometry_profile"]
    else:
        audit_reference = artifact["generation_metadata"][
            "audit_input_geometry_profile"
        ]
    lines = [
        "# MATDOG Geometry Compiler V5 — geometry-driven path/parking",
        "",
        f"input geometry profile: `{audit_reference['relative_path']}`",
        f"geometry file SHA256: `{audit_reference['file_sha256']}`",
        f"geometry semantic SHA256: `{artifact['input_geometry_profile']['semantic_content_sha256']}`",
        f"parking semantic SHA256: `{artifact['semantic_content_sha256']}`",
        f"endpoint plans: {artifact['summary']['endpoint_plan_count']}",
        f"baseline obstructions: {artifact['summary']['baseline_obstructed_count']}",
        f"complete geometric sequences: {artifact['summary']['geometry_feasible_complete_sequence_count']}",
        f"targets inside URDF limits: {artifact['summary']['executable_urdf_target_count']}",
        f"diagnostic targets outside URDF limits: {artifact['summary']['diagnostic_target_outside_urdf_limits_count']}",
        "",
        "Search is normalized to URDF limits: 1-DOF first, 2-DOF only if no 1-DOF plan is feasible.",
        "The declared search domain is the finite grid serialized per plan; it is not a continuous-domain proof.",
        "Path feasibility is evaluated at every serialized-step configuration, not as continuous swept volume.",
        "The first baseline obstruction is additionally bisected to the serialized refinement resolution; sampled and refined evidence remain distinct.",
        "A DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS target is not an executable robot motion claim.",
        "No fixed historical parking-angle seed list and no safety threshold is used.",
        "",
        "| Endpoint | Target domain | Baseline | First blocker (refined when available) | Relation | Refined contact / bracket / resolution rad | Relevant joints | Outcome | In/task/return/out | Candidates 1D/2D | Parking rad | Min clearance |",
        "|---|---|---|---|---|---|---|---|---|---:|---|---:|",
    ]
    for plan in artifact["plans"]:
        blocker_pair = plan.get(
            "first_refined_blocking_pair",
            plan.get("first_sampled_blocking_pair"),
        )
        blocker = (
            " ↔ ".join(blocker_pair)
            if blocker_pair
            else "-"
        )
        blocker_relation = plan.get(
            "first_refined_blocking_relation",
            plan.get("first_sampled_blocking_relation"),
        )
        refined = plan["baseline_task_path"].get("refined_first_obstruction")
        joint_name = plan["joint_name"]
        if (
            refined is None
            or joint_name not in refined.get("clear_joint_positions_rad", {})
            or joint_name not in refined.get("contact_joint_positions_rad", {})
        ):
            refinement = "-"
        else:
            clear_angle = refined["clear_joint_positions_rad"][joint_name]
            contact_angle = refined["contact_joint_positions_rad"][joint_name]
            refinement = (
                f"`{contact_angle:.9f}` / "
                f"`{abs(contact_angle - clear_angle):.9g}` / "
                f"`{refined['bisection_resolution_rad']:.9g}`"
            )
        objective = plan["objective"]
        clearance = objective["min_clearance_m"] if objective is not None else None
        segments = "/".join(
            segment["status"] if segment is not None else "-"
            for segment in (
                plan["path_in"],
                plan["task_path"],
                plan["task_return"],
                plan["path_out"],
            )
        )
        lines.append(
            f"| {plan['endpoint_id']} | {plan['target_domain']} | "
            f"{plan['baseline_task_path']['status']} | {blocker} | "
            f"{blocker_relation or '-'} | "
            f"{refinement} | "
            f"{', '.join(plan['relevant_movable_joint_names']) or '-'} | {plan['outcome']} | "
            f"{segments} | {plan['evaluated_1d_candidates']}/{plan['evaluated_2d_candidates']} | "
            f"`{plan['parking_configuration_rad']}` | {clearance if clearance is not None else '-'} |"
        )
    lines.extend(
        [
            "",
            "Geometric feasibility is intersection/no intersection. Clearance is a measured quantity.",
            "Safety acceptance is intentionally absent and belongs to the separate policy artifact.",
            "",
        ]
    )
    return "\n".join(lines)


def write_parking_artifact(artifact: dict[str, Any], json_path: Path) -> tuple[Path, Path]:
    validate_parking_artifact(artifact)
    target = Path(json_path)
    target.parent.mkdir(parents=True, exist_ok=True)

    def atomic(text: str, path: Path) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                handle.write(text)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_name, path)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    atomic(json.dumps(artifact, indent=2, sort_keys=True) + "\n", target)
    report_path = target.with_suffix(".md")
    atomic(render_parking_report(artifact), report_path)
    return target, report_path
