#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler per-endpoint contact search.

For one of the 24 (leg x joint x side) endpoints, finds the first real
mesh-to-mesh contact along that joint's sweep, inside an explicit
configurable analysis envelope around the URDF-declared limit -- never
assuming the declared limit itself is the mesh contact (canonical handoff
section 3/6).

Method (matches the 2026-07-20 checkpoint's own stated method, generalized
into reusable code): coarse angular scout to bracket the clear/contact
transition, then adaptive bisection refinement down to a configurable
angular resolution.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
from pathlib import Path

CALIBRATION_DIR = Path(__file__).resolve().parent
KINEMATICS_DIR = CALIBRATION_DIR.parents[0] / "kinematics"

for _extra_path in (KINEMATICS_DIR, CALIBRATION_DIR):
    if str(_extra_path) not in sys.path:
        sys.path.insert(0, str(_extra_path))

from matdog_urdf_fk import canonical_urdf_path, load_urdf_joints  # noqa: E402

import math

from matdog_geometry_scene import (  # noqa: E402
    JOINT_GROUPS,
    LEG_IDS,
    RobotScene,
    full_pose,
    joint_name,
    pair_is_cross_leg,
    same_leg_non_adjacent_pairs,
)


DEFAULT_COARSE_STEP_RAD = 0.017453292519943295  # 1 deg, matches checkpoint precedent
DEFAULT_ENVELOPE_MARGIN_RAD = 0.17453292519943295  # 10 deg beyond the declared limit
DEFAULT_BISECTION_RESOLUTION_RAD = 0.0001
"""~0.0057 deg, well under one ST3215 encoder tick (~0.0879 deg /
RAD_PER_TICK) -- a numerical search resolution, not a claim of
hardware-matching precision."""
DEFAULT_MAX_BISECTION_ITERATIONS = 40
DEFAULT_MODEL_LIMIT_MISMATCH_THRESHOLD_RAD = 0.03490658503988659  # 2 deg

HIP_PREREQUISITE_UPPER_RAD = 0.8726646259971648  # +50 deg, checkpoint search seed
LOWER_PREREQUISITE_UPPER_RAD = 1.5707963267948966  # +90 deg, checkpoint search seed

MESH_HARDWARE_AGREEMENT_THRESHOLD_DEG = 2.0
"""When a direct hardware contact angle is known for an endpoint, a
same-leg mesh contact within this many degrees of the REAL hardware
contact (not the declared URDF limit) is accepted as representing that
same physical event (MODELED_ENDSTOP_CONTACT). Matches
DEFAULT_MODEL_LIMIT_MISMATCH_THRESHOLD_RAD's 2deg for consistency; no LF
endpoint in the 2026-08-04 V25 data actually falls inside this band (the
closest is ~4.7deg away), so the exact value is not decisive for the
current dataset, only documented as a deliberate, physically-motivated
choice for future data."""

LF_V25_HARDWARE_EVIDENCE: dict[str, dict[str, object]] = {
    # Source: 06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
    # (2026-08-04, EEPROM-frozen, hardware-validated) cross-checked against
    # the archived run's per-motor profile
    # (_archive/MATDOG_LF_MEASURE_FREEZE_V25_execute_20260804T041729Z/final-lf-profile.json).
    #
    # `hardware_contact_rad` is the REAL joint angle (URDF convention) at
    # which V25 hardware detected contact, computed from the raw contact
    # tick via q = (tick - q0) * direction / ticks_per_deg, using the
    # NOMINAL ST3215 scale (ticks_per_deg = 4096/360 = 11.377778, q0=2048)
    # -- NOT the per-motor calibrated affine scale in that profile, which
    # is itself fit so that contact_min/contact_max map EXACTLY onto
    # urdf_lower_rad/urdf_upper_rad (residuals ~0.02-0.07deg by
    # construction); using that affine scale to ask "does the real contact
    # match the declared/mesh limit" would be circular.
    #
    # This is the authoritative oracle value for reconciliation: the
    # primary comparison it supports is mesh_contact_rad vs
    # hardware_contact_rad (does the collision-mesh event correspond to
    # the same real physical contact hardware found), not
    # hardware_contact_rad vs urdf_declared_limit_rad (a secondary,
    # informational comparison -- see hardware_vs_urdf_status).
    "lf_hip_min": {
        "hardware_contact_rad": -0.7470486437003072,  # -42.8027 deg
        "note": (
            "M13 contact_min=2535 tick -> q=(2535-2048)*(-1)/11.377778=-42.8027deg "
            "(nominal ST3215 scale, direction=-1, q0=2048)"
        ),
    },
    "lf_hip_max": {
        "hardware_contact_rad": 0.6872233929727672,  # +39.3750 deg
        "note": (
            "M13 contact_max=1600 tick -> q=(1600-2048)*(-1)/11.377778=+39.3750deg "
            "(nominal ST3215 scale, direction=-1, q0=2048)"
        ),
    },
    "lf_upper_leg_min": {
        "hardware_contact_rad": -0.9341942998223555,  # -53.5254 deg
        "note": (
            "M12 contact_min=1439 tick -> q=(1439-2048)*(1)/11.377778=-53.5254deg "
            "(nominal ST3215 scale, direction=+1, q0=2048)"
        ),
    },
    "lf_upper_leg_max": {
        "hardware_contact_rad": 2.1399031991004693,  # +122.6074 deg
        "note": (
            "M12 contact_max=3443 tick -> q=(3443-2048)*(1)/11.377778=+122.6074deg "
            "(nominal ST3215 scale, direction=+1, q0=2048)"
        ),
    },
    "lf_lower_leg_min": {
        "hardware_contact_rad": -1.603009923340495,  # -91.8457 deg
        "note": (
            "M11 contact_min=3093 tick -> q=(3093-2048)*(-1)/11.377778=-91.8457deg "
            "(nominal ST3215 scale, direction=-1, q0=2048)"
        ),
    },
    "lf_lower_leg_max": {
        "hardware_contact_rad": 0.5982525072754,  # +34.2773 deg
        "note": (
            "M11 contact_max=1658 tick -> q=(1658-2048)*(-1)/11.377778=+34.2773deg "
            "(nominal ST3215 scale, direction=-1, q0=2048)"
        ),
    },
}
"""Direct hardware oracle, LF leg only (2026-08-04 V25 freeze). Never
mirrored onto RF/RH/LH: those legs are physically distinct assemblies and
have not been independently hardware-calibrated, so applying LF's numbers
to them would present an inference-by-symmetry as if it were their own
hardware fact. RF's mesh geometry has been separately verified as
symmetric to LF's (see checkpoint regression / mirror tests); that is a
geometry-model finding, not hardware evidence, and is reported as such.

NOTE (reconciliation review, corrects the first-pass version of this
table): the first-pass table stored a "hardware_delta_from_declared_deg"
figure directly and had the SIGN wrong on 3 of 6 entries (hip_min,
upper_leg_min, lower_leg_min) due to inconsistent delta-direction
conventions across manual derivations. Storing the actual
hardware_contact_rad angle instead of a pre-computed delta removes that
whole class of sign error: every downstream comparison (vs declared, vs
mesh) is now computed once, in one place, from this single value."""


class ContactSearchError(RuntimeError):
    """Errore nella ricerca del contatto mesh per endpoint."""


@dataclass(frozen=True)
class EndpointSpec:
    leg: str
    joint_group: str  # "hip" | "upper_leg" | "lower_leg"
    side: str  # "min" | "max"
    joint_name: str
    servo_id: int
    urdf_lower_rad: float
    urdf_upper_rad: float
    prerequisite_overrides: dict[str, float]
    """Search-seed pose for the OTHER two joints of the same leg while
    probing this one (see canonical handoff section 5: seeds, not
    permanent constants)."""

    @property
    def urdf_declared_limit_rad(self) -> float:
        return self.urdf_upper_rad if self.side == "max" else self.urdf_lower_rad

    @property
    def endpoint_id(self) -> str:
        return f"{self.leg}_{self.joint_group}_{self.side}"


@dataclass(frozen=True)
class EndpointContactResult:
    endpoint: EndpointSpec
    result_kind: str
    """One of: MESH_CONTACT_FOUND, NO_MESH_CONTACT_IN_ENVELOPE -- outcome
    of the ENDSTOP_CONTACT_POLICY (same-leg-only) search alone. A cross-leg
    obstruction never appears here any more (it cannot: the same-leg
    search's candidate pairs exclude every other leg by construction) --
    see `path_collision_link_a/b` and `contact_model_status` for that."""
    mesh_predicted_contact_rad: float | None
    contact_link_a: str | None
    contact_link_b: str | None
    clearance_before_contact_m: float | None
    bracket_clear_rad: float | None
    bracket_contact_rad: float | None
    coarse_step_rad: float
    bisection_resolution_rad: float
    bisection_iterations: int
    analysis_envelope_rad: tuple[float, float]
    model_limit_mismatch: bool
    delta_from_declared_rad: float | None
    other_legs_pose: dict[str, float]
    contact_model_status: str
    """The canonical-handoff-reconciliation classification, one of:
    MODELED_ENDSTOP_CONTACT, NO_MODELED_ENDSTOP, MODEL_INCOMPLETE,
    PATH_COLLISION_BEFORE_ENDPOINT, UNINTENDED_SELF_COLLISION,
    MODEL_LIMIT_MISMATCH. This is the field a human reviewer / future
    hardware runner should read; `result_kind` alone is not sufficient
    (see `_classify_contact_model_status`)."""
    contact_model_status_reason: str
    path_collision_angle_rad: float | None
    """Angle (rad) of the first PATH_SELF_COLLISION_POLICY (full pair set,
    cross-leg included) obstruction found in the sweep, independent of
    the same-leg search above; None if none was found in the envelope."""
    path_collision_link_a: str | None
    path_collision_link_b: str | None
    hardware_evidence_note: str | None
    """Direct V25 hardware evidence for this endpoint if available (LF
    only, see LF_V25_HARDWARE_EVIDENCE); None otherwise."""
    hardware_vs_urdf_status: str
    """COMPATIBLE | INCOMPATIBLE | NOT_AVAILABLE -- does the real hardware
    contact angle agree with the URDF-declared limit. Informational only:
    reconciliation review corrected a bug where this comparison was used
    to drive contact_model_status; the comparison that actually matters
    for that decision is mesh_vs_hardware_status below."""
    mesh_vs_hardware_status: str
    """AGREES | DISAGREES | NO_MESH_CONTACT | NOT_AVAILABLE -- does the
    same-leg mesh contact (if any) correspond to the same real physical
    event the hardware oracle found. This is the primary comparison
    driving contact_model_status when a hardware oracle exists (canonical
    handoff reconciliation review): a mesh contact several degrees past
    where hardware actually stopped is not "the endpoint" just because it
    is close to the declared URDF limit."""

    @property
    def is_cross_leg(self) -> bool:
        if self.contact_link_a is None or self.contact_link_b is None:
            return False
        return pair_is_cross_leg(self.contact_link_a, self.contact_link_b)


def _prerequisite_for(joint_group: str) -> tuple[float, float, float]:
    """(hip_rad, upper_rad, lower_rad) prerequisite pose for probing the
    given joint group, as checkpoint-derived search seeds."""
    if joint_group == "hip":
        return (0.0, HIP_PREREQUISITE_UPPER_RAD, 0.0)
    if joint_group == "lower_leg":
        return (0.0, LOWER_PREREQUISITE_UPPER_RAD, 0.0)
    if joint_group == "upper_leg":
        return (0.0, 0.0, 0.0)
    raise ContactSearchError(f"joint_group sconosciuto: {joint_group!r}")


def load_all_endpoints(repo_root: Path) -> tuple[EndpointSpec, ...]:
    from matdog_geometry_scene import SERVO_ID_BY_JOINT  # noqa: E402

    urdf_path = canonical_urdf_path(repo_root)
    joints = load_urdf_joints(urdf_path)

    endpoints: list[EndpointSpec] = []

    for leg in LEG_IDS:
        for group in JOINT_GROUPS:
            name = joint_name(leg, group)
            joint = joints.get(name)

            if joint is None or joint.lower_limit_rad is None or joint.upper_limit_rad is None:
                raise ContactSearchError(f"URDF: limiti mancanti per {name}")

            hip_seed, upper_seed, lower_seed = _prerequisite_for(group)
            overrides: dict[str, float] = {}

            for other_group, seed_value in zip(JOINT_GROUPS, (hip_seed, upper_seed, lower_seed)):
                if other_group == group:
                    continue
                overrides[joint_name(leg, other_group)] = seed_value

            for side in ("min", "max"):
                endpoints.append(
                    EndpointSpec(
                        leg=leg,
                        joint_group=group,
                        side=side,
                        joint_name=name,
                        servo_id=SERVO_ID_BY_JOINT[name],
                        urdf_lower_rad=joint.lower_limit_rad,
                        urdf_upper_rad=joint.upper_limit_rad,
                        prerequisite_overrides=overrides,
                    )
                )

    return tuple(endpoints)


def _pose_for_probe_angle(endpoint: EndpointSpec, probe_angle_rad: float, other_legs_pose: dict[str, float]) -> dict[str, float]:
    overrides = dict(other_legs_pose)
    overrides.update(endpoint.prerequisite_overrides)
    overrides[endpoint.joint_name] = probe_angle_rad
    return full_pose(overrides)


def bracket_collision_boundary(
    collide_fn,
    start_value: float,
    sign: float,
    envelope_end: float,
    coarse_step: float,
) -> tuple[float, float] | None:
    """Generic coarse scout: sweep `collide_fn(value) -> bool` from
    `start_value` toward `envelope_end` (in the direction of `sign`) at
    `coarse_step` resolution, returning (last_clear, first_contact) or
    None if the whole envelope is collision-free. `collide_fn` is the only
    coupling to any specific geometry/kinematics -- this function and
    `bisect_collision_boundary` below are pure numerical-method code,
    independently testable against a synthetic collide_fn with a known
    analytical crossing point."""
    last_clear = start_value
    value = start_value

    while True:
        next_value = value + sign * coarse_step

        if sign > 0:
            next_value = min(next_value, envelope_end)
        else:
            next_value = max(next_value, envelope_end)

        if collide_fn(next_value):
            return last_clear, next_value

        last_clear = next_value
        value = next_value

        if value == envelope_end:
            return None


def bisect_collision_boundary(
    collide_fn,
    clear_value: float,
    contact_value: float,
    resolution: float,
    max_iterations: int,
) -> tuple[float, float, int]:
    """Generic bisection refinement of a [clear, contact) bracket down to
    `resolution`, given `collide_fn(value) -> bool`."""
    iterations = 0

    while abs(contact_value - clear_value) > resolution and iterations < max_iterations:
        mid = (clear_value + contact_value) / 2.0

        if collide_fn(mid):
            contact_value = mid
        else:
            clear_value = mid

        iterations += 1

    return clear_value, contact_value, iterations


def _coarse_scout(
    scene: RobotScene,
    endpoint: EndpointSpec,
    other_legs_pose: dict[str, float],
    coarse_step_rad: float,
    envelope_margin_rad: float,
    link_pairs: tuple[tuple[str, str], ...] | None,
) -> tuple[float, float] | None:
    """Sweep from 0 toward declared_limit +/- envelope_margin_rad at
    coarse_step_rad resolution; thin endpoint-specific adapter over
    `bracket_collision_boundary`. `link_pairs=None` means the default
    PATH_SELF_COLLISION_POLICY (all non-adjacent pairs, cross-leg
    included); pass `same_leg_non_adjacent_pairs(endpoint.leg)` for the
    ENDSTOP_CONTACT_POLICY (this leg's own geometry only)."""
    declared = endpoint.urdf_declared_limit_rad
    sign = 1.0 if endpoint.side == "max" else -1.0
    envelope_end = declared + sign * envelope_margin_rad

    def collide_fn(angle: float) -> bool:
        pose = _pose_for_probe_angle(endpoint, angle, other_legs_pose)
        collide, _pair = scene.is_colliding_at_pose(pose, link_pairs=link_pairs)
        return collide

    return bracket_collision_boundary(collide_fn, 0.0, sign, envelope_end, coarse_step_rad)


def _bisect_to_resolution(
    scene: RobotScene,
    endpoint: EndpointSpec,
    other_legs_pose: dict[str, float],
    clear_angle: float,
    contact_angle: float,
    resolution_rad: float,
    max_iterations: int,
    link_pairs: tuple[tuple[str, str], ...] | None,
) -> tuple[float, float, int]:
    """Thin endpoint-specific adapter over `bisect_collision_boundary`."""

    def collide_fn(angle: float) -> bool:
        pose = _pose_for_probe_angle(endpoint, angle, other_legs_pose)
        collide, _pair = scene.is_colliding_at_pose(pose, link_pairs=link_pairs)
        return collide

    return bisect_collision_boundary(collide_fn, clear_angle, contact_angle, resolution_rad, max_iterations)


def _classify_contact_model_status(
    endpoint: EndpointSpec,
    *,
    same_leg_found: bool,
    mesh_contact_rad: float | None,
    delta_rad: float | None,
    mismatch: bool,
    coarse_step_rad: float,
    path_collision_angle_rad: float | None,
    model_limit_mismatch_threshold_deg: float,
) -> tuple[str, str, str | None, str, str]:
    """Canonical-handoff-reconciliation classification. Returns
    (contact_model_status, reason, hardware_evidence_note,
    hardware_vs_urdf_status, mesh_vs_hardware_status).

    Two collision policies feed this: the ENDSTOP_CONTACT_POLICY
    (same-leg-only) search decides whether *this joint's own geometry*
    shows a contact; the PATH_SELF_COLLISION_POLICY (full pair set)
    search decides whether some *other* obstruction (almost always
    cross-leg, by construction) is encountered first along the sweep.

    When a hardware oracle is available (LF only, LF_V25_HARDWARE_EVIDENCE),
    the decision is driven by MESH vs HARDWARE agreement, not hardware vs
    declared URDF (reconciliation review correction: an earlier version of
    this function used the hardware-vs-declared gap to decide
    MODEL_INCOMPLETE, which is the wrong comparison -- a mesh contact can
    sit close to the declared limit yet be several degrees away from
    where hardware actually stopped, which means it is NOT the real
    endstop regardless of how close it is to the URDF number). The
    hardware-vs-URDF comparison is kept only as an informational
    `hardware_vs_urdf_status` field.

    For legs without a hardware oracle (RF/RH/LH as of this writing), the
    only available signal is mesh-vs-declared-URDF, so the classification
    falls back to MODELED_ENDSTOP_CONTACT / MODEL_LIMIT_MISMATCH /
    NO_MODELED_ENDSTOP as before -- these legs are never auto-promoted to
    MODEL_INCOMPLETE, since there is no hardware proof that the model is
    actually missing something (canonical handoff reconciliation section
    "NO_MODELED_ENDSTOP must be used mainly when no hardware oracle
    proves the model is missing a real endstop").
    """
    hw = LF_V25_HARDWARE_EVIDENCE.get(endpoint.endpoint_id)
    hw_note = str(hw["note"]) if hw is not None else None
    hw_contact_rad = float(hw["hardware_contact_rad"]) if hw is not None else None
    declared_rad = endpoint.urdf_declared_limit_rad

    if hw_contact_rad is None:
        hardware_vs_urdf_status = "NOT_AVAILABLE"
    else:
        hw_urdf_delta_deg = math.degrees(hw_contact_rad - declared_rad)
        hardware_vs_urdf_status = (
            "COMPATIBLE" if abs(hw_urdf_delta_deg) <= MESH_HARDWARE_AGREEMENT_THRESHOLD_DEG else "INCOMPATIBLE"
        )

    if hw_contact_rad is None:
        mesh_vs_hardware_status = "NOT_AVAILABLE"
    elif not same_leg_found:
        mesh_vs_hardware_status = "NO_MESH_CONTACT"
    else:
        assert mesh_contact_rad is not None
        mesh_hw_delta_deg = math.degrees(mesh_contact_rad - hw_contact_rad)
        mesh_vs_hardware_status = (
            "AGREES" if abs(mesh_hw_delta_deg) <= MESH_HARDWARE_AGREEMENT_THRESHOLD_DEG else "DISAGREES"
        )

    has_path_collision = path_collision_angle_rad is not None
    path_precedes_same_leg = has_path_collision and (
        not same_leg_found or abs(path_collision_angle_rad) < abs(mesh_contact_rad)  # type: ignore[arg-type]
    )

    if path_precedes_same_leg:
        reason = (
            f"cross-leg path obstruction found at {math.degrees(path_collision_angle_rad):.2f} deg, "  # type: ignore[arg-type]
            "before any same-leg mesh contact in the envelope (or none exists at all); this is a "
            "path/parking prerequisite, not this joint's own designed limit"
        )
        return "PATH_COLLISION_BEFORE_ENDPOINT", reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status

    if same_leg_found:
        assert mesh_contact_rad is not None and delta_rad is not None
        swept_from_home_rad = abs(mesh_contact_rad)

        if swept_from_home_rad <= 2.0 * coarse_step_rad:
            reason = (
                f"same-leg mesh contact found within {math.degrees(swept_from_home_rad):.2f} deg of "
                "home (2 coarse steps or less) -- too close to home to plausibly be the intended "
                "joint limit; flagged as an incidental/unintended same-leg collision rather than a "
                "designed endstop"
            )
            return (
                "UNINTENDED_SELF_COLLISION", reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status,
            )

        if hw_contact_rad is not None:
            mesh_hw_delta_deg = math.degrees(mesh_contact_rad - hw_contact_rad)

            if mesh_vs_hardware_status == "AGREES":
                reason = (
                    f"same-leg mesh contact at {math.degrees(mesh_contact_rad):.2f} deg agrees with the "
                    f"LF V25 hardware contact at {math.degrees(hw_contact_rad):.2f} deg (delta "
                    f"{mesh_hw_delta_deg:+.2f} deg, within the {MESH_HARDWARE_AGREEMENT_THRESHOLD_DEG:.1f} "
                    "deg agreement band) -- represents the same real physical event"
                )
                status = "MODELED_ENDSTOP_CONTACT"
            else:
                reason = (
                    f"same-leg mesh contact at {math.degrees(mesh_contact_rad):.2f} deg does NOT match "
                    f"the LF V25 hardware contact at {math.degrees(hw_contact_rad):.2f} deg (delta "
                    f"{mesh_hw_delta_deg:+.2f} deg) -- this mesh event is not the real mechanical "
                    "endstop; recorded as a diagnostic collision only, not the endpoint"
                )
                if mismatch:
                    reason += (
                        f". Separately, model_limit_mismatch=True vs the declared URDF limit "
                        f"(independent diagnostic flag, delta {math.degrees(delta_rad):+.2f} deg)"
                    )
                status = "MODEL_INCOMPLETE"

            return status, reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status

        # No hardware oracle for this leg: fall back to mesh-vs-declared-URDF.
        if not mismatch:
            reason = (
                f"same-leg mesh contact at {math.degrees(mesh_contact_rad):.2f} deg is within "
                f"{model_limit_mismatch_threshold_deg:.1f} deg of the declared limit "
                "(no hardware oracle for this leg to cross-check against)"
            )
            return (
                "MODELED_ENDSTOP_CONTACT", reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status,
            )

        reason = (
            f"same-leg mesh contact at {math.degrees(mesh_contact_rad):.2f} deg is "
            f"{math.degrees(delta_rad):+.2f} deg from the declared limit; no direct hardware evidence "
            "available for this leg, so whether this is a real endstop or a modelling artifact cannot "
            "be determined (MODEL_INCOMPLETE is reserved for legs with a hardware oracle proving a "
            "disagreement)"
        )
        return "MODEL_LIMIT_MISMATCH", reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status

    # No same-leg mesh contact found at all.
    if hw_contact_rad is not None:
        reason = (
            "no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a "
            f"real contact exists at {math.degrees(hw_contact_rad):.2f} deg -- the real stopping "
            "mechanism (servo/bracket internal limit, not the collision STL) is not represented in "
            "this model"
        )
        return "MODEL_INCOMPLETE", reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status

    reason = "no same-leg mesh contact found in the analysis envelope; no hardware oracle for this leg"
    return "NO_MODELED_ENDSTOP", reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status


def search_endpoint_contact(
    scene: RobotScene,
    endpoint: EndpointSpec,
    *,
    other_legs_pose: dict[str, float] | None = None,
    coarse_step_rad: float = DEFAULT_COARSE_STEP_RAD,
    envelope_margin_rad: float = DEFAULT_ENVELOPE_MARGIN_RAD,
    bisection_resolution_rad: float = DEFAULT_BISECTION_RESOLUTION_RAD,
    max_bisection_iterations: int = DEFAULT_MAX_BISECTION_ITERATIONS,
    model_limit_mismatch_threshold_rad: float = DEFAULT_MODEL_LIMIT_MISMATCH_THRESHOLD_RAD,
) -> EndpointContactResult:
    """Coarse-bracket + bisection search for one endpoint's real mesh
    contact, run as two independent policies (canonical handoff
    reconciliation section 2/3):

    1. ENDSTOP_CONTACT_POLICY -- this leg's own non-adjacent pairs only,
       answering "does this joint's own geometry show a contact".
    2. PATH_SELF_COLLISION_POLICY -- the full candidate pair set (cross-leg
       included), answering "is anything else in the way first".

    A cross-leg obstruction can no longer silently terminate the same-leg
    search and be misreported as if it were this joint's own limit; the
    two are combined by `_classify_contact_model_status`."""
    context_pose = dict(other_legs_pose) if other_legs_pose is not None else {}
    declared = endpoint.urdf_declared_limit_rad
    envelope = (
        min(declared - envelope_margin_rad, declared + envelope_margin_rad),
        max(declared - envelope_margin_rad, declared + envelope_margin_rad),
    )
    leg_pairs = same_leg_non_adjacent_pairs(endpoint.leg)

    # 1. ENDSTOP_CONTACT_POLICY: same-leg-only search.
    same_leg_bracket = _coarse_scout(
        scene, endpoint, context_pose, coarse_step_rad, envelope_margin_rad, link_pairs=leg_pairs
    )

    result_kind = "NO_MESH_CONTACT_IN_ENVELOPE"
    mesh_contact_rad: float | None = None
    contact_link_a: str | None = None
    contact_link_b: str | None = None
    clearance_before: float | None = None
    bracket_clear: float | None = None
    bracket_contact: float | None = None
    iterations = 0
    delta: float | None = None
    mismatch = False

    if same_leg_bracket is not None:
        clear_angle, contact_angle = same_leg_bracket
        clear_angle, contact_angle, iterations = _bisect_to_resolution(
            scene, endpoint, context_pose, clear_angle, contact_angle,
            bisection_resolution_rad, max_bisection_iterations, link_pairs=leg_pairs,
        )

        contact_pose = _pose_for_probe_angle(endpoint, contact_angle, context_pose)
        _collide, pair = scene.is_colliding_at_pose(contact_pose, link_pairs=leg_pairs)

        if pair is None:
            raise ContactSearchError(
                f"{endpoint.endpoint_id}: il bracket di bisezione (same-leg) converge ma il "
                "campione al contatto non risulta in collisione; ricerca instabile"
            )

        contact_link_a, contact_link_b = pair
        # Pinned to the SAME pair that converged (not worst_pair_at_pose's
        # possibly-different worst pair overall), for the same reason the
        # sensitivity analysis is pinned (canonical handoff section 8).
        clear_pose = _pose_for_probe_angle(endpoint, clear_angle, context_pose)
        clearance_result = scene.check_link_pair(
            contact_link_a, contact_link_b, clear_pose, require_distance=True
        )
        result_kind = "MESH_CONTACT_FOUND"
        mesh_contact_rad = contact_angle
        bracket_clear, bracket_contact = clear_angle, contact_angle
        clearance_before = clearance_result.clearance_m
        delta = contact_angle - declared
        mismatch = abs(delta) > model_limit_mismatch_threshold_rad

    # 2. PATH_SELF_COLLISION_POLICY: full pair set, independent of the
    # same-leg result above -- may find nothing, may find the same
    # same-leg pair, or may find a cross-leg obstruction earlier/only.
    path_bracket = _coarse_scout(
        scene, endpoint, context_pose, coarse_step_rad, envelope_margin_rad, link_pairs=None
    )
    path_angle: float | None = None
    path_link_a: str | None = None
    path_link_b: str | None = None

    if path_bracket is not None:
        p_clear, p_contact = path_bracket
        p_clear, p_contact, _p_iter = _bisect_to_resolution(
            scene, endpoint, context_pose, p_clear, p_contact,
            bisection_resolution_rad, max_bisection_iterations, link_pairs=None,
        )
        path_pose = _pose_for_probe_angle(endpoint, p_contact, context_pose)
        _collide, pair = scene.is_colliding_at_pose(path_pose, link_pairs=None)

        if pair is not None and pair_is_cross_leg(*pair):
            path_angle = p_contact
            path_link_a, path_link_b = pair

    status, status_reason, hw_note, hardware_vs_urdf_status, mesh_vs_hardware_status = _classify_contact_model_status(
        endpoint,
        same_leg_found=(result_kind == "MESH_CONTACT_FOUND"),
        mesh_contact_rad=mesh_contact_rad,
        delta_rad=delta,
        mismatch=mismatch,
        coarse_step_rad=coarse_step_rad,
        path_collision_angle_rad=path_angle,
        model_limit_mismatch_threshold_deg=math.degrees(model_limit_mismatch_threshold_rad),
    )

    return EndpointContactResult(
        endpoint=endpoint,
        result_kind=result_kind,
        mesh_predicted_contact_rad=mesh_contact_rad,
        contact_link_a=contact_link_a,
        contact_link_b=contact_link_b,
        clearance_before_contact_m=clearance_before,
        bracket_clear_rad=bracket_clear,
        bracket_contact_rad=bracket_contact,
        coarse_step_rad=coarse_step_rad,
        bisection_resolution_rad=bisection_resolution_rad,
        bisection_iterations=iterations,
        analysis_envelope_rad=envelope,
        model_limit_mismatch=mismatch,
        delta_from_declared_rad=delta,
        other_legs_pose=context_pose,
        contact_model_status=status,
        contact_model_status_reason=status_reason,
        path_collision_angle_rad=path_angle,
        path_collision_link_a=path_link_a,
        path_collision_link_b=path_link_b,
        hardware_evidence_note=hw_note,
        hardware_vs_urdf_status=hardware_vs_urdf_status,
        mesh_vs_hardware_status=mesh_vs_hardware_status,
    )
