#!/usr/bin/env python3
"""Pure geometric endpoint and path-obstruction search for V5.

The two questions are deliberately independent:

* geometric endpoint contact: when the active revolute parent/child pair
  first intersects in the declared search domain;
* path obstruction: when any other relevant pair first intersects along the
  same sampled motion.

Neither result imports hardware evidence, applies a safety threshold, or
changes the other result's status.  Context poses are explicit geometric
inputs; the core contains no mandatory +50/+90 degree prerequisites.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from matdog_geometry_model_v5 import RobotGeometryModel
from matdog_geometry_scene_v5 import RobotSceneV5


DEFAULT_COARSE_STEP_RAD = math.radians(1.0)
DEFAULT_ENVELOPE_MARGIN_RAD = math.radians(10.0)
DEFAULT_BISECTION_RESOLUTION_RAD = 0.0001
DEFAULT_MAX_BISECTION_ITERATIONS = 40

GEOMETRIC_CONTACT_FOUND = "GEOMETRIC_CONTACT_FOUND"
NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN = "NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN"
PATH_OBSTRUCTION = "PATH_OBSTRUCTION"
NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN = "NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN"


class GeometryContactSearchV5Error(RuntimeError):
    """The pure geometric search could not produce a stable result."""


@dataclass(frozen=True)
class EndpointSpecV5:
    endpoint_id: str
    joint_name: str
    side: str
    branch_root_joint_name: str
    structural_depth: int
    motor_id: int
    motor_direction: int
    urdf_lower_rad: float
    urdf_upper_rad: float
    active_link_pair: tuple[str, str]

    @property
    def urdf_declared_limit_rad(self) -> float:
        return self.urdf_lower_rad if self.side == "min" else self.urdf_upper_rad


@dataclass(frozen=True)
class GeometricEndpointResultV5:
    endpoint: EndpointSpecV5
    status: str
    contact_angle_rad: float | None
    contact_link_pair: tuple[str, str]
    declared_limit_delta_rad: float | None
    search_start_rad: float
    search_domain_rad: tuple[float, float]
    bracket_clear_rad: float | None
    bracket_contact_rad: float | None
    coarse_step_rad: float
    bisection_resolution_rad: float
    max_bisection_iterations: int
    bisection_iterations: int
    context_pose_rad: dict[str, float]


@dataclass(frozen=True)
class PathObstructionResultV5:
    endpoint_id: str
    status: str
    obstruction_angle_rad: float | None
    obstruction_link_pair: tuple[str, str] | None
    relation: str | None
    search_domain_rad: tuple[float, float]
    bracket_clear_rad: float | None
    bracket_contact_rad: float | None
    coarse_step_rad: float
    bisection_resolution_rad: float
    max_bisection_iterations: int
    bisection_iterations: int
    context_pose_rad: dict[str, float]


@dataclass(frozen=True)
class EndpointAnalysisV5:
    geometry: GeometricEndpointResultV5
    path: PathObstructionResultV5 | None

    @property
    def path_obstruction_precedes_contact(self) -> bool | None:
        if self.path is None or self.path.obstruction_angle_rad is None:
            return False
        if self.geometry.contact_angle_rad is None:
            return True
        return abs(self.path.obstruction_angle_rad) < abs(self.geometry.contact_angle_rad)


def load_endpoint_specs(model: RobotGeometryModel) -> tuple[EndpointSpecV5, ...]:
    specs: list[EndpointSpecV5] = []
    for branch in model.articulated_branches:
        for depth, joint_name in enumerate(branch.joint_names):
            joint = model.joints[joint_name]
            if not joint.is_actuated_revolute:
                raise GeometryContactSearchV5Error(
                    f"{joint_name}: selected model joint is not an actuated bounded revolute"
                )
            assert joint.motor_id is not None
            assert joint.motor_direction is not None
            assert joint.lower_limit_rad is not None
            assert joint.upper_limit_rad is not None
            for side in ("min", "max"):
                specs.append(
                    EndpointSpecV5(
                        endpoint_id=f"{joint_name}:{side}",
                        joint_name=joint_name,
                        side=side,
                        branch_root_joint_name=branch.root_joint_name,
                        structural_depth=depth,
                        motor_id=joint.motor_id,
                        motor_direction=joint.motor_direction,
                        urdf_lower_rad=joint.lower_limit_rad,
                        urdf_upper_rad=joint.upper_limit_rad,
                        active_link_pair=model.active_revolute_pair(joint_name),
                    )
                )
    if len(specs) != 2 * len(model.actuated_joint_names):
        raise GeometryContactSearchV5Error(
            "endpoint construction did not produce exactly two sides per actuated joint"
        )
    if len({spec.endpoint_id for spec in specs}) != len(specs):
        raise GeometryContactSearchV5Error("endpoint IDs are not unique")
    return tuple(specs)


def bracket_collision_boundary(
    collide_fn: Callable[[float], bool],
    start_value: float,
    sign: float,
    domain_end: float,
    coarse_step: float,
) -> tuple[float, float] | None:
    if sign not in (-1.0, 1.0):
        raise GeometryContactSearchV5Error(f"invalid search sign: {sign}")
    if not (math.isfinite(coarse_step) and coarse_step > 0.0):
        raise GeometryContactSearchV5Error(f"invalid coarse step: {coarse_step}")

    last_clear = start_value
    value = start_value
    while True:
        candidate = value + sign * coarse_step
        candidate = min(candidate, domain_end) if sign > 0.0 else max(candidate, domain_end)
        if collide_fn(candidate):
            return last_clear, candidate
        last_clear = candidate
        value = candidate
        if value == domain_end:
            return None


def bisect_collision_boundary(
    collide_fn: Callable[[float], bool],
    clear_value: float,
    contact_value: float,
    resolution: float,
    max_iterations: int,
) -> tuple[float, float, int]:
    if not (math.isfinite(resolution) and resolution > 0.0):
        raise GeometryContactSearchV5Error(f"invalid bisection resolution: {resolution}")
    if max_iterations <= 0:
        raise GeometryContactSearchV5Error(
            f"max_iterations must be positive, got {max_iterations}"
        )
    iterations = 0
    while abs(contact_value - clear_value) > resolution and iterations < max_iterations:
        midpoint = (clear_value + contact_value) / 2.0
        if collide_fn(midpoint):
            contact_value = midpoint
        else:
            clear_value = midpoint
        iterations += 1
    return clear_value, contact_value, iterations


def _search_domain(endpoint: EndpointSpecV5, envelope_margin_rad: float) -> tuple[float, float, float]:
    if not (math.isfinite(envelope_margin_rad) and envelope_margin_rad >= 0.0):
        raise GeometryContactSearchV5Error(
            f"invalid envelope margin: {envelope_margin_rad}"
        )
    sign = -1.0 if endpoint.side == "min" else 1.0
    domain_end = endpoint.urdf_declared_limit_rad + sign * envelope_margin_rad
    return sign, min(0.0, domain_end), max(0.0, domain_end)


def _pose_for_angle(
    scene: RobotSceneV5,
    endpoint: EndpointSpecV5,
    angle_rad: float,
    context_pose_rad: dict[str, float],
) -> dict[str, float]:
    pose = scene.full_pose(context_pose_rad)
    pose[endpoint.joint_name] = angle_rad
    return pose


def _search_first_transition(
    scene: RobotSceneV5,
    endpoint: EndpointSpecV5,
    context_pose_rad: dict[str, float],
    link_pairs: tuple[tuple[str, str], ...],
    *,
    coarse_step_rad: float,
    envelope_margin_rad: float,
    bisection_resolution_rad: float,
    max_bisection_iterations: int,
) -> tuple[tuple[float, float] | None, int, tuple[float, float]]:
    sign, domain_min, domain_max = _search_domain(endpoint, envelope_margin_rad)
    domain_end = domain_max if sign > 0.0 else domain_min

    def collides(angle_rad: float) -> bool:
        pose = _pose_for_angle(scene, endpoint, angle_rad, context_pose_rad)
        collision, _pair = scene.first_collision_at_pose(pose, link_pairs=link_pairs)
        return collision

    start_pose = _pose_for_angle(scene, endpoint, 0.0, context_pose_rad)
    start_collision, start_pair = scene.first_collision_at_pose(
        start_pose,
        link_pairs=link_pairs,
    )
    if start_collision:
        raise GeometryContactSearchV5Error(
            f"{endpoint.endpoint_id}: search starts in collision for {start_pair}; "
            "q=0/start validity is a hard gate, not an endpoint result"
        )

    bracket = bracket_collision_boundary(
        collides,
        0.0,
        sign,
        domain_end,
        coarse_step_rad,
    )
    if bracket is None:
        return None, 0, (domain_min, domain_max)
    clear_angle, contact_angle = bracket
    clear_angle, contact_angle, iterations = bisect_collision_boundary(
        collides,
        clear_angle,
        contact_angle,
        bisection_resolution_rad,
        max_bisection_iterations,
    )
    return (clear_angle, contact_angle), iterations, (domain_min, domain_max)


def search_geometric_endpoint(
    scene: RobotSceneV5,
    endpoint: EndpointSpecV5,
    *,
    context_pose_rad: dict[str, float] | None = None,
    coarse_step_rad: float = DEFAULT_COARSE_STEP_RAD,
    envelope_margin_rad: float = DEFAULT_ENVELOPE_MARGIN_RAD,
    bisection_resolution_rad: float = DEFAULT_BISECTION_RESOLUTION_RAD,
    max_bisection_iterations: int = DEFAULT_MAX_BISECTION_ITERATIONS,
) -> GeometricEndpointResultV5:
    context = dict(context_pose_rad or {})
    context.pop(endpoint.joint_name, None)
    active_pair = endpoint.active_link_pair
    bracket, iterations, domain = _search_first_transition(
        scene,
        endpoint,
        context,
        (active_pair,),
        coarse_step_rad=coarse_step_rad,
        envelope_margin_rad=envelope_margin_rad,
        bisection_resolution_rad=bisection_resolution_rad,
        max_bisection_iterations=max_bisection_iterations,
    )
    if bracket is None:
        return GeometricEndpointResultV5(
            endpoint=endpoint,
            status=NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
            contact_angle_rad=None,
            contact_link_pair=active_pair,
            declared_limit_delta_rad=None,
            search_start_rad=0.0,
            search_domain_rad=domain,
            bracket_clear_rad=None,
            bracket_contact_rad=None,
            coarse_step_rad=coarse_step_rad,
            bisection_resolution_rad=bisection_resolution_rad,
            max_bisection_iterations=max_bisection_iterations,
            bisection_iterations=0,
            context_pose_rad=dict(sorted(context.items())),
        )

    clear_angle, contact_angle = bracket
    contact_pose = _pose_for_angle(scene, endpoint, contact_angle, context)
    collides, pair = scene.first_collision_at_pose(
        contact_pose,
        link_pairs=(active_pair,),
    )
    if not collides or pair is None:
        raise GeometryContactSearchV5Error(
            f"{endpoint.endpoint_id}: converged active-pair contact is unstable"
        )
    return GeometricEndpointResultV5(
        endpoint=endpoint,
        status=GEOMETRIC_CONTACT_FOUND,
        contact_angle_rad=contact_angle,
        contact_link_pair=pair,
        declared_limit_delta_rad=contact_angle - endpoint.urdf_declared_limit_rad,
        search_start_rad=0.0,
        search_domain_rad=domain,
        bracket_clear_rad=clear_angle,
        bracket_contact_rad=contact_angle,
        coarse_step_rad=coarse_step_rad,
        bisection_resolution_rad=bisection_resolution_rad,
        max_bisection_iterations=max_bisection_iterations,
        bisection_iterations=iterations,
        context_pose_rad=dict(sorted(context.items())),
    )


def search_path_obstruction(
    scene: RobotSceneV5,
    endpoint: EndpointSpecV5,
    *,
    context_pose_rad: dict[str, float] | None = None,
    coarse_step_rad: float = DEFAULT_COARSE_STEP_RAD,
    envelope_margin_rad: float = DEFAULT_ENVELOPE_MARGIN_RAD,
    bisection_resolution_rad: float = DEFAULT_BISECTION_RESOLUTION_RAD,
    max_bisection_iterations: int = DEFAULT_MAX_BISECTION_ITERATIONS,
) -> PathObstructionResultV5:
    context = dict(context_pose_rad or {})
    context.pop(endpoint.joint_name, None)
    pairs = scene.model.path_obstruction_pairs(exclude_pair=endpoint.active_link_pair)
    bracket, iterations, domain = _search_first_transition(
        scene,
        endpoint,
        context,
        pairs,
        coarse_step_rad=coarse_step_rad,
        envelope_margin_rad=envelope_margin_rad,
        bisection_resolution_rad=bisection_resolution_rad,
        max_bisection_iterations=max_bisection_iterations,
    )
    if bracket is None:
        return PathObstructionResultV5(
            endpoint_id=endpoint.endpoint_id,
            status=NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
            obstruction_angle_rad=None,
            obstruction_link_pair=None,
            relation=None,
            search_domain_rad=domain,
            bracket_clear_rad=None,
            bracket_contact_rad=None,
            coarse_step_rad=coarse_step_rad,
            bisection_resolution_rad=bisection_resolution_rad,
            max_bisection_iterations=max_bisection_iterations,
            bisection_iterations=0,
            context_pose_rad=dict(sorted(context.items())),
        )

    clear_angle, contact_angle = bracket
    contact_pose = _pose_for_angle(scene, endpoint, contact_angle, context)
    collides, pair = scene.first_collision_at_pose(contact_pose, link_pairs=pairs)
    if not collides or pair is None:
        raise GeometryContactSearchV5Error(
            f"{endpoint.endpoint_id}: converged path obstruction is unstable"
        )
    return PathObstructionResultV5(
        endpoint_id=endpoint.endpoint_id,
        status=PATH_OBSTRUCTION,
        obstruction_angle_rad=contact_angle,
        obstruction_link_pair=pair,
        relation=scene.model.pair_relation(*pair),
        search_domain_rad=domain,
        bracket_clear_rad=clear_angle,
        bracket_contact_rad=contact_angle,
        coarse_step_rad=coarse_step_rad,
        bisection_resolution_rad=bisection_resolution_rad,
        max_bisection_iterations=max_bisection_iterations,
        bisection_iterations=iterations,
        context_pose_rad=dict(sorted(context.items())),
    )


def analyze_endpoint(
    scene: RobotSceneV5,
    endpoint: EndpointSpecV5,
    *,
    context_pose_rad: dict[str, float] | None = None,
    include_path_obstruction: bool = True,
    coarse_step_rad: float = DEFAULT_COARSE_STEP_RAD,
    envelope_margin_rad: float = DEFAULT_ENVELOPE_MARGIN_RAD,
    bisection_resolution_rad: float = DEFAULT_BISECTION_RESOLUTION_RAD,
    max_bisection_iterations: int = DEFAULT_MAX_BISECTION_ITERATIONS,
) -> EndpointAnalysisV5:
    kwargs = {
        "context_pose_rad": context_pose_rad,
        "coarse_step_rad": coarse_step_rad,
        "envelope_margin_rad": envelope_margin_rad,
        "bisection_resolution_rad": bisection_resolution_rad,
        "max_bisection_iterations": max_bisection_iterations,
    }
    geometry = search_geometric_endpoint(scene, endpoint, **kwargs)
    path = search_path_obstruction(scene, endpoint, **kwargs) if include_path_obstruction else None
    return EndpointAnalysisV5(geometry=geometry, path=path)
