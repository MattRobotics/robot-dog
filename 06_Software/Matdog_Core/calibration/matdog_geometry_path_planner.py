#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler path/prerequisite/parking planner.

New rule (canonical handoff section 5/6): default = NO AUXILIARY PARKING.
For every leg, this module:

1. tries the active leg's full calibration sequence (prerequisites +
   endpoint probes + restores) with every other leg at its nominal home
   pose, validating the *continuous* path (not just endpoints);
2. if that passes with adequate clearance, records auxiliary_moves = [];
3. if it fails, searches the smallest justified auxiliary parking angle
   (from the historical +30..+90 deg seeds) for whichever other leg was
   actually implicated in the failure, re-validating the complete
   sequence with that leg parked;
4. validates the parking path itself (home -> parking, parking -> home).

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
from pathlib import Path

CALIBRATION_DIR = Path(__file__).resolve().parent

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_scene import (  # noqa: E402
    RobotScene,
    full_pose,
    joint_name,
    leg_of_link,
    leg_pose_overrides,
)
from matdog_geometry_contact_search import (  # noqa: E402
    HIP_PREREQUISITE_UPPER_RAD,
    LOWER_PREREQUISITE_UPPER_RAD,
    load_all_endpoints,
)
from matdog_geometry_mesh_kernel import clearance_gate  # noqa: E402


DEFAULT_PATH_STEP_RAD = 0.017453292519943295  # 1 deg, matches checkpoint precedent
DEFAULT_PARKING_PATH_STEP_RAD = 0.008726646259971648  # 0.5 deg, matches checkpoint precedent
DEFAULT_MIN_CLEARANCE_PASS_M = 0.003
"""3 mm: a documented, configurable default for "adequate clearance" in
path planning (canonical handoff section 6.3). Not derived from a
project-mandated value (none exists yet); flagged as an assumption in the
compiler report, not silently authoritative."""
DEFAULT_PARKING_SEED_ANGLES_DEG: tuple[float, ...] = (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0)
"""Historical +30..+90 deg seeds from the 2026-07-20 checkpoint, used only
as a search order (smallest first), not as mandatory constants."""
DEFAULT_CLEARANCE_SAMPLE_STRIDE = 25
"""Compute an exact worst-pair clearance figure only every Nth path
sample (booleans are checked at every sample). `worst_pair_at_pose`
evaluates all ~120 candidate link pairs to an exact clearance and does
not short-circuit the way the boolean collision check does, so it
dominates path-validation runtime; empirically, a stride of 4 made a
full per-leg sequence (~1100 samples at the 1 deg default step) take
several minutes. A stride of 25 keeps a representative minimum-clearance
figure while keeping full 4-leg planning tractable in one offline run."""


class PathPlannerError(RuntimeError):
    """Errore nel planner di percorsi/parcheggio del Geometry Compiler."""


def _mm(value: float | None) -> str:
    if value is None:
        return "unknown"
    return f"{value * 1000.0:.4f} mm"


@dataclass(frozen=True)
class PathSample:
    angle_rad: float
    collides: bool
    contact_pair: tuple[str, str] | None
    clearance_m: float | None
    """Only populated at the sampling stride; None elsewhere (still
    collision-free, just not the exact-clearance evaluation)."""


@dataclass(frozen=True)
class PathSegmentResult:
    description: str
    joint_name: str
    start_rad: float
    end_rad: float
    other_legs_pose: dict[str, float]
    passed: bool
    min_clearance_m: float | None
    min_clearance_kind: str | None
    """EXACT, LOWER_BOUND, or None (see matdog_geometry_mesh_kernel
    PairCollisionResult.clearance_kind) -- the kind attached to the
    specific sample that produced `min_clearance_m`."""
    clearance_gate_result: str
    """PASS, FAIL, UNRESOLVED_FOR_THRESHOLD, or NOT_EVALUATED (see
    matdog_geometry_mesh_kernel.clearance_gate). A LOWER_BOUND figure
    below the pass threshold is UNRESOLVED_FOR_THRESHOLD, not FAIL: the
    search margin does not prove the true clearance is actually below the
    threshold, only that it was not proven to be above it."""
    first_collision_angle_rad: float | None
    first_collision_pair: tuple[str, str] | None
    sample_count: int

    @property
    def has_true_collision(self) -> bool:
        """True mesh intersection found, distinct from `passed=False`
        caused only by the minimum modelled clearance dipping below
        `DEFAULT_MIN_CLEARANCE_PASS_M` (whether that low reading is a
        confirmed EXACT figure or a merely UNRESOLVED_FOR_THRESHOLD lower
        bound) with no actual intersection -- these are different findings
        and must not be reported the same way (a low-but-nonzero or
        unresolved clearance is not evidence that a *different* leg's
        parking would help, since there is no colliding pair to act on)."""
        return self.first_collision_pair is not None


@dataclass(frozen=True)
class LegSequenceResult:
    leg: str
    other_legs_pose: dict[str, float]
    segments: tuple[PathSegmentResult, ...]

    @property
    def passed(self) -> bool:
        return all(segment.passed for segment in self.segments)

    @property
    def min_clearance_m(self) -> float | None:
        values = [s.min_clearance_m for s in self.segments if s.min_clearance_m is not None]
        return min(values) if values else None

    @property
    def first_failure(self) -> PathSegmentResult | None:
        for segment in self.segments:
            if not segment.passed:
                return segment
        return None


@dataclass(frozen=True)
class ParkingPlan:
    leg: str
    required: bool
    reason: str
    parked_leg: str | None
    parking_angle_rad: float | None
    park_path: LegSequenceResult | None
    """home -> parking -> home validation for the parked leg, only present
    when required=True."""
    active_leg_sequence: LegSequenceResult
    """The active leg's full calibration sequence with the chosen
    other_legs_pose (home if not required, home+parked leg if required)."""


def _leg_joint_names(leg: str) -> tuple[str, str, str]:
    return (joint_name(leg, "hip"), joint_name(leg, "upper_leg"), joint_name(leg, "lower_leg"))


def _sweep_and_validate(
    scene: RobotScene,
    description: str,
    active_joint: str,
    start_rad: float,
    end_rad: float,
    fixed_active_leg_overrides: dict[str, float],
    other_legs_pose: dict[str, float],
    step_rad: float,
    clearance_stride: int,
) -> PathSegmentResult:
    if start_rad == end_rad:
        samples_rad = [start_rad]
    else:
        direction = 1.0 if end_rad > start_rad else -1.0
        samples_rad = []
        value = start_rad
        while (value - end_rad) * direction < 0:
            samples_rad.append(value)
            value += direction * step_rad
        samples_rad.append(end_rad)

    min_clearance: float | None = None
    min_clearance_kind: str | None = None
    first_collision_angle: float | None = None
    first_collision_pair: tuple[str, str] | None = None

    for index, angle in enumerate(samples_rad):
        overrides = dict(other_legs_pose)
        overrides.update(fixed_active_leg_overrides)
        overrides[active_joint] = angle
        pose = full_pose(overrides)

        if index % clearance_stride == 0:
            link_a, link_b, result = scene.worst_pair_at_pose(pose)

            if result.status == "INTERSECTING":
                first_collision_angle = angle
                first_collision_pair = (link_a, link_b)
                break

            if result.clearance_m is not None and (min_clearance is None or result.clearance_m < min_clearance):
                min_clearance = result.clearance_m
                min_clearance_kind = result.clearance_kind
        else:
            collide, pair = scene.is_colliding_at_pose(pose)

            if collide:
                first_collision_angle = angle
                first_collision_pair = pair
                break

    if first_collision_angle is not None:
        gate = "FAIL"
    elif min_clearance is None:
        gate = "NOT_EVALUATED"
    else:
        gate = clearance_gate("SEPARATED_NARROW", min_clearance, min_clearance_kind, DEFAULT_MIN_CLEARANCE_PASS_M)

    passed = first_collision_angle is None and gate == "PASS"

    return PathSegmentResult(
        description=description,
        joint_name=active_joint,
        start_rad=start_rad,
        end_rad=end_rad,
        other_legs_pose=dict(other_legs_pose),
        passed=passed,
        min_clearance_m=min_clearance,
        min_clearance_kind=min_clearance_kind,
        clearance_gate_result=gate,
        first_collision_angle_rad=first_collision_angle,
        first_collision_pair=first_collision_pair,
        sample_count=len(samples_rad),
    )


def leg_calibration_sequence(
    scene: RobotScene,
    leg: str,
    other_legs_pose: dict[str, float],
    *,
    step_rad: float = DEFAULT_PATH_STEP_RAD,
    clearance_stride: int = DEFAULT_CLEARANCE_SAMPLE_STRIDE,
) -> LegSequenceResult:
    """Validate the complete per-leg sequence structure documented in the
    2026-07-20 checkpoint's 'Intended per-leg calibration sequence':
    upper probe at home, transition to HIP prerequisite, hip probe,
    transition to LOWER prerequisite, lower probe, restore home."""
    hip_j, upper_j, lower_j = _leg_joint_names(leg)
    urdf_endpoints = {e.joint_name: e for e in load_all_endpoints(scene.repo_root) if e.leg == leg}

    upper_min = urdf_endpoints[upper_j].urdf_lower_rad
    upper_max = urdf_endpoints[upper_j].urdf_upper_rad
    hip_min = urdf_endpoints[hip_j].urdf_lower_rad
    hip_max = urdf_endpoints[hip_j].urdf_upper_rad
    lower_min = urdf_endpoints[lower_j].urdf_lower_rad
    lower_max = urdf_endpoints[lower_j].urdf_upper_rad

    segments: list[PathSegmentResult] = []

    def sweep(description: str, joint: str, start: float, end: float, fixed: dict[str, float]) -> None:
        segments.append(
            _sweep_and_validate(
                scene, description, joint, start, end, fixed, other_legs_pose, step_rad, clearance_stride
            )
        )

    fixed_home = {hip_j: 0.0, lower_j: 0.0}
    sweep("upper probe negative (home prerequisite)", upper_j, 0.0, upper_min, fixed_home)
    sweep("upper probe return to home", upper_j, upper_min, 0.0, fixed_home)
    sweep("upper probe positive (home prerequisite)", upper_j, 0.0, upper_max, fixed_home)
    sweep("upper probe return to home", upper_j, upper_max, 0.0, fixed_home)

    sweep("transition to HIP prerequisite", upper_j, 0.0, HIP_PREREQUISITE_UPPER_RAD, fixed_home)
    fixed_hip_prereq = {upper_j: HIP_PREREQUISITE_UPPER_RAD, lower_j: 0.0}
    sweep("hip probe negative", hip_j, 0.0, hip_min, fixed_hip_prereq)
    sweep("hip probe return", hip_j, hip_min, 0.0, fixed_hip_prereq)
    sweep("hip probe positive", hip_j, 0.0, hip_max, fixed_hip_prereq)
    sweep("hip probe return", hip_j, hip_max, 0.0, fixed_hip_prereq)

    sweep(
        "transition HIP prerequisite -> LOWER prerequisite",
        upper_j, HIP_PREREQUISITE_UPPER_RAD, LOWER_PREREQUISITE_UPPER_RAD, fixed_home,
    )
    fixed_lower_prereq = {hip_j: 0.0, upper_j: LOWER_PREREQUISITE_UPPER_RAD}
    sweep("lower probe negative", lower_j, 0.0, lower_min, fixed_lower_prereq)
    sweep("lower probe return", lower_j, lower_min, 0.0, fixed_lower_prereq)
    sweep("lower probe positive", lower_j, 0.0, lower_max, fixed_lower_prereq)
    sweep("lower probe return", lower_j, lower_max, 0.0, fixed_lower_prereq)

    sweep("restore leg to home", upper_j, LOWER_PREREQUISITE_UPPER_RAD, 0.0, fixed_home)

    return LegSequenceResult(leg=leg, other_legs_pose=dict(other_legs_pose), segments=tuple(segments))


def _parking_sequence(scene: RobotScene, parked_leg: str, parking_angle_rad: float, step_rad: float) -> LegSequenceResult:
    hip_j, upper_j, lower_j = _leg_joint_names(parked_leg)
    fixed = {hip_j: 0.0, lower_j: 0.0}

    to_park = _sweep_and_validate(
        scene, "park leg: home -> parking", upper_j, 0.0, parking_angle_rad, fixed, {}, step_rad,
        DEFAULT_CLEARANCE_SAMPLE_STRIDE,
    )
    from_park = _sweep_and_validate(
        scene, "park leg: parking -> home", upper_j, parking_angle_rad, 0.0, fixed, {}, step_rad,
        DEFAULT_CLEARANCE_SAMPLE_STRIDE,
    )

    return LegSequenceResult(leg=parked_leg, other_legs_pose={}, segments=(to_park, from_park))


def _low_clearance_note(sequence: LegSequenceResult) -> str | None:
    """Describe the worst non-colliding clearance finding in a sequence
    that has no true collision at all, distinguishing a confirmed EXACT
    low reading from a merely UNRESOLVED_FOR_THRESHOLD lower bound (see
    matdog_geometry_mesh_kernel.clearance_gate) -- both are informational,
    neither is evidence that a different leg's parking would help."""
    worst_segment = None

    for segment in sequence.segments:
        if segment.clearance_gate_result in ("FAIL", "UNRESOLVED_FOR_THRESHOLD"):
            if worst_segment is None or (
                segment.min_clearance_m is not None
                and (worst_segment.min_clearance_m is None or segment.min_clearance_m < worst_segment.min_clearance_m)
            ):
                worst_segment = segment

    if worst_segment is None:
        return None

    kind_note = (
        "a confirmed exact figure" if worst_segment.min_clearance_kind == "EXACT"
        else "an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap)"
    )
    return (
        f"minimum modelled clearance {_mm(worst_segment.min_clearance_m)} at '{worst_segment.description}' "
        f"is below the configured PASS bar ({_mm(DEFAULT_MIN_CLEARANCE_PASS_M)}); this is {kind_note}"
    )


def plan_leg_parking(
    scene: RobotScene,
    leg: str,
    *,
    step_rad: float = DEFAULT_PATH_STEP_RAD,
    parking_step_rad: float = DEFAULT_PARKING_PATH_STEP_RAD,
    parking_seed_angles_deg: tuple[float, ...] = DEFAULT_PARKING_SEED_ANGLES_DEG,
) -> ParkingPlan:
    """Default-no-parking search: try the leg's full sequence with every
    other leg at home. Only a TRUE mesh intersection in some segment
    triggers a parking search (a low or unresolved-for-threshold modelled
    clearance is a separate, non-collision finding that no amount of
    auxiliary parking can be shown to fix, per canonical handoff section
    6). The seed-angle acceptance criterion below is scoped to "no true
    collision remains in any segment" -- NOT the stronger, unrelated
    "every segment individually clears the 3mm bar" -- because segments
    with nothing to do with the implicated leg (e.g. this leg's own hip
    probe near its own declared limit) can carry an unrelated low-clearance
    reading that no parking angle for a different leg could ever resolve;
    using the stronger criterion previously made every seed angle look
    like a failure even when the actual cross-leg collision was already
    resolved (see MATDOG_CALIBRATION_GEOMETRY_PROFILE reconciliation
    against LF V25 hardware, which validated exactly this: a single +30deg
    parking pose held for the whole session, matching the smallest
    checkpoint seed)."""
    import math

    no_parking_sequence = leg_calibration_sequence(scene, leg, {}, step_rad=step_rad)
    colliding_segments = [s for s in no_parking_sequence.segments if s.has_true_collision]

    if not colliding_segments:
        low_clearance_note = _low_clearance_note(no_parking_sequence)
        reason = "no true mesh collision in any segment with every other leg at home"

        if low_clearance_note is not None:
            reason += f"; NEEDS_HUMAN_DECISION margin finding (not a collision, parking not applicable): {low_clearance_note}"

        return ParkingPlan(
            leg=leg,
            required=False,
            reason=reason,
            parked_leg=None,
            parking_angle_rad=None,
            park_path=None,
            active_leg_sequence=no_parking_sequence,
        )

    implicated_legs: set[str] = set()

    for segment in colliding_segments:
        assert segment.first_collision_pair is not None
        for link in segment.first_collision_pair:
            other = leg_of_link(link)

            if other is not None and other != leg:
                implicated_legs.add(other)

    colliding_descriptions = ", ".join(f"'{s.description}'" for s in colliding_segments)

    if len(implicated_legs) != 1:
        reason = (
            f"segment(s) [{colliding_descriptions}] have a true mesh collision, but the implicated "
            f"leg set is not exactly one other leg (found {sorted(implicated_legs) or ['none -- self-collision within ' + leg]}); "
            "auxiliary parking of a single other leg cannot be searched unambiguously, NEEDS_HUMAN_DECISION"
        )
        return ParkingPlan(
            leg=leg,
            required=False,
            reason=reason,
            parked_leg=None,
            parking_angle_rad=None,
            park_path=None,
            active_leg_sequence=no_parking_sequence,
        )

    implicated_leg = next(iter(implicated_legs))

    for angle_deg in parking_seed_angles_deg:
        angle_rad = math.radians(angle_deg)
        parked_pose = leg_pose_overrides(implicated_leg, 0.0, angle_rad, 0.0)
        candidate_sequence = leg_calibration_sequence(scene, leg, parked_pose, step_rad=step_rad)
        remaining_collisions = [s for s in candidate_sequence.segments if s.has_true_collision]

        if not remaining_collisions:
            park_path = _parking_sequence(scene, implicated_leg, angle_rad, parking_step_rad)
            low_clearance_note = _low_clearance_note(candidate_sequence)
            reason = (
                f"segment(s) [{colliding_descriptions}] collide against {implicated_leg} at home "
                f"(true mesh intersection); minimal seed parking {angle_deg:.0f} deg for "
                f"{implicated_leg}, held for the whole {leg} sequence (single park-before/restore-after, "
                "matching the validated 2026-07-20 checkpoint and LF V25 hardware practice), resolves "
                "every true collision"
            )

            if low_clearance_note is not None:
                reason += f"; residual NEEDS_HUMAN_DECISION margin finding (not a collision): {low_clearance_note}"

            return ParkingPlan(
                leg=leg,
                required=True,
                reason=reason,
                parked_leg=implicated_leg,
                parking_angle_rad=angle_rad,
                park_path=park_path,
                active_leg_sequence=candidate_sequence,
            )

    return ParkingPlan(
        leg=leg,
        required=True,
        reason=(
            f"NEEDS_HUMAN_DECISION: no seed parking angle in {parking_seed_angles_deg} for "
            f"{implicated_leg} resolved every true collision from [{colliding_descriptions}]; a search "
            "over a wider/finer angle range or a different auxiliary leg may be required"
        ),
        parked_leg=implicated_leg,
        parking_angle_rad=None,
        park_path=None,
        active_leg_sequence=no_parking_sequence,
    )
