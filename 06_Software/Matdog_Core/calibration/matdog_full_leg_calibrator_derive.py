"""MATDOG Full Leg Calibrator V1 — q0 and joint-calibration candidate derivation.

Pure functions. No serial port, no servo, no file writes.

The central rule of this module is that ``manual_pose_q0_candidate`` and
``derived_q0_final`` are **two different things** and are never conflated:

* :class:`ManualQ0Candidate` is what the encoder read while the user held the
  robot in the nominal CAD pose with a square. Torque OFF, multi-sample, with
  dispersion. It is evidence with finite uncertainty, not a metrology fixture.

* :class:`DerivedQ0` is solved from the *measured* mechanical endpoint contacts
  and the *current* URDF/Geometry Compiler angles for those endpoints.

They are compared by :func:`cross_check_q0`. If they disagree beyond an
evidence-based tolerance the result is a structured FAIL with a diagnosis — never
an average of the two, and never a servo ``PositionOffset`` write.

Encoder/angle conversion is delegated to :mod:`matdog_joint_math`, the canonical
MATDOG source for encoder <-> radian math, rather than re-deriving the formula.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

from matdog_joint_math import (
    ENCODER_MODULUS,
    TICKS_PER_RAD,
    circular_tick_summary,
    normalize_tick,
    signed_tick_delta,
)

#: Servo electrical centre. Present ONLY as a reporting reference. q0 is never
#: asserted to equal it: the horn/pulley spline is finite-toothed, so a joint can
#: sit up to roughly +/-5 deg (~+/-57 ticks) from the ideal CAD pose even though
#: the servo was held at RAW 2048 during assembly.
RAW_ELECTRICAL_CENTER = 2048


class Q0Status(Enum):
    OK = "OK"
    UNSTABLE_SAMPLES = "UNSTABLE_SAMPLES"
    INSUFFICIENT_SAMPLES = "INSUFFICIENT_SAMPLES"
    IMPLAUSIBLE_RESIDUAL = "IMPLAUSIBLE_RESIDUAL"


class DerivationStatus(Enum):
    OK = "OK"
    ENDPOINT_ORDER_CONTRADICTION = "ENDPOINT_ORDER_CONTRADICTION"
    DEGENERATE_SPAN = "DEGENERATE_SPAN"
    SCALE_OUT_OF_RANGE = "SCALE_OUT_OF_RANGE"
    WRAP_DOMAIN_VIOLATION = "WRAP_DOMAIN_VIOLATION"
    DIRECTION_AMBIGUOUS = "DIRECTION_AMBIGUOUS"


class CrossCheckStatus(Enum):
    AGREE = "AGREE"
    DISAGREE = "DISAGREE"
    BLOCKED_TOLERANCE_UNVALIDATED = "BLOCKED_TOLERANCE_UNVALIDATED"
    MISSING_INPUT = "MISSING_INPUT"


@dataclass(frozen=True)
class ManualQ0Candidate:
    """Read-only capture of the manually-set CAD calibration pose, torque OFF."""

    joint_name: str
    bus_id: int
    unit_label: str
    sample_count: int
    median_tick: int
    min_tick: int
    max_tick: int
    spread_ticks: int
    stdev_ticks: float
    residual_from_center_ticks: int
    residual_from_center_deg: float
    status: Q0Status
    pose_id: str = "urdf_mechanical_zero_pose"
    torque_off_verified: bool = True

    @property
    def ok(self) -> bool:
        return self.status is Q0Status.OK

    def as_dict(self) -> dict:
        return {
            "kind": "manual_pose_q0_candidate",
            "promotion": "NOT_FINAL_Q0",
            "joint_name": self.joint_name,
            "bus_id": self.bus_id,
            "unit_label": self.unit_label,
            "pose_id": self.pose_id,
            "torque_off_verified": self.torque_off_verified,
            "sample_count": self.sample_count,
            "median_tick": self.median_tick,
            "min_tick": self.min_tick,
            "max_tick": self.max_tick,
            "spread_ticks": self.spread_ticks,
            "stdev_ticks": round(self.stdev_ticks, 4),
            "residual_from_electrical_center_ticks": self.residual_from_center_ticks,
            "residual_from_electrical_center_deg": round(self.residual_from_center_deg, 4),
            "status": self.status.value,
        }


def capture_manual_q0(
    joint_name: str,
    bus_id: int,
    unit_label: str,
    samples: Sequence[int],
    *,
    max_spread_ticks: int = 12,
    min_samples: int = 16,
    plausible_residual_ticks: int = 120,
    torque_off_verified: bool = True,
) -> ManualQ0Candidate:
    """Summarize a manual-pose capture without ever rounding toward 2048.

    Uses the canonical :func:`circular_tick_summary`, so a pose that happens to
    straddle the 4095/0 boundary reports a real spread rather than ~4095.
    """
    usable = [normalize_tick(int(s)) for s in samples]

    if len(usable) < min_samples:
        return ManualQ0Candidate(
            joint_name, bus_id, unit_label, len(usable), 0, 0, 0, 0, 0.0, 0, 0.0,
            Q0Status.INSUFFICIENT_SAMPLES, torque_off_verified=torque_off_verified,
        )

    median_tick, spread = circular_tick_summary(usable)

    # Report min/max on the same unwrapped arc the summary used, so a
    # wrap-straddling capture does not report a nonsense 0..4095 range.
    unwrapped = [median_tick + signed_tick_delta(t, median_tick) for t in usable]
    stdev = statistics.pstdev(unwrapped) if len(unwrapped) > 1 else 0.0

    residual = signed_tick_delta(median_tick, RAW_ELECTRICAL_CENTER)
    residual_deg = residual * 360.0 / ENCODER_MODULUS

    if spread > max_spread_ticks:
        status = Q0Status.UNSTABLE_SAMPLES
    elif abs(residual) > plausible_residual_ticks:
        status = Q0Status.IMPLAUSIBLE_RESIDUAL
    elif not torque_off_verified:
        status = Q0Status.UNSTABLE_SAMPLES
    else:
        status = Q0Status.OK

    return ManualQ0Candidate(
        joint_name=joint_name,
        bus_id=bus_id,
        unit_label=unit_label,
        sample_count=len(usable),
        median_tick=median_tick,
        min_tick=normalize_tick(min(unwrapped)),
        max_tick=normalize_tick(max(unwrapped)),
        spread_ticks=spread,
        stdev_ticks=stdev,
        residual_from_center_ticks=residual,
        residual_from_center_deg=residual_deg,
        status=status,
        torque_off_verified=torque_off_verified,
    )


@dataclass(frozen=True)
class EndpointMeasurement:
    """One repeatability-confirmed mechanical contact, in raw encoder ticks."""

    side: str  # "min" or "max"
    contact_tick: int
    first_tick: int
    second_tick: int
    spread_ticks: int
    expected_angle_rad: float  # URDF / Geometry Compiler V5 angle at this contact


@dataclass(frozen=True)
class DerivedQ0:
    """q0 and encoder direction solved from measured contacts + current geometry."""

    joint_name: str
    bus_id: int
    status: DerivationStatus
    q0_tick: int | None = None
    direction: int | None = None
    scale: float | None = None
    measured_span_ticks: int | None = None
    expected_span_ticks: float | None = None
    residual_min_deg: float | None = None
    residual_max_deg: float | None = None
    notes: tuple[str, ...] = field(default_factory=tuple)

    @property
    def ok(self) -> bool:
        return self.status is DerivationStatus.OK

    def as_dict(self) -> dict:
        return {
            "kind": "derived_q0_final_candidate",
            "promotion": "REQUIRES_EXPLICIT_PROMOTION_GATE",
            "joint_name": self.joint_name,
            "bus_id": self.bus_id,
            "status": self.status.value,
            "q0_tick": self.q0_tick,
            "direction": self.direction,
            "scale": None if self.scale is None else round(self.scale, 6),
            "measured_span_ticks": self.measured_span_ticks,
            "expected_span_ticks": (
                None if self.expected_span_ticks is None
                else round(self.expected_span_ticks, 3)
            ),
            "residual_min_deg": (
                None if self.residual_min_deg is None else round(self.residual_min_deg, 4)
            ),
            "residual_max_deg": (
                None if self.residual_max_deg is None else round(self.residual_max_deg, 4)
            ),
            "notes": list(self.notes),
        }


def derive_q0(
    joint_name: str,
    bus_id: int,
    minimum: EndpointMeasurement,
    maximum: EndpointMeasurement,
    *,
    scale_min: float = 0.85,
    scale_max: float = 1.15,
) -> DerivedQ0:
    """Solve the affine encoder<->joint mapping from two measured endpoints.

    The model is the canonical MATDOG convention::

        q = direction * signed_tick_delta(raw, q0_tick) * RAD_PER_TICK

    generalized with a measured ``scale`` so a span mismatch is *reported*
    rather than silently absorbed into q0.

    Direction is **derived**, never assumed: it is the sign of the raw travel
    between the min-angle contact and the max-angle contact. No FRONT/HIND or
    LEFT/RIGHT symmetry is inferred.
    """
    notes: list[str] = []

    if maximum.expected_angle_rad <= minimum.expected_angle_rad:
        return DerivedQ0(
            joint_name, bus_id, DerivationStatus.ENDPOINT_ORDER_CONTRADICTION,
            notes=("expected max angle is not greater than expected min angle",),
        )

    expected_span_rad = maximum.expected_angle_rad - minimum.expected_angle_rad
    expected_span_ticks = expected_span_rad * TICKS_PER_RAD

    raw_delta = signed_tick_delta(maximum.contact_tick, minimum.contact_tick)
    if raw_delta == 0:
        return DerivedQ0(
            joint_name, bus_id, DerivationStatus.DEGENERATE_SPAN,
            expected_span_ticks=expected_span_ticks,
            notes=("both endpoint contacts landed on the same encoder tick",),
        )

    direction = 1 if raw_delta > 0 else -1
    measured_span_ticks = abs(raw_delta)

    # A joint span close to half a revolution cannot be resolved unambiguously in
    # a 4096-tick circular domain; refuse rather than pick a branch.
    if measured_span_ticks >= ENCODER_MODULUS // 2 - 1:
        return DerivedQ0(
            joint_name, bus_id, DerivationStatus.WRAP_DOMAIN_VIOLATION,
            expected_span_ticks=expected_span_ticks,
            measured_span_ticks=measured_span_ticks,
            notes=("measured span approaches half a revolution; sign is ambiguous",),
        )

    scale = measured_span_ticks / expected_span_ticks
    if not (scale_min <= scale <= scale_max):
        return DerivedQ0(
            joint_name, bus_id, DerivationStatus.SCALE_OUT_OF_RANGE,
            direction=direction, scale=scale,
            measured_span_ticks=measured_span_ticks,
            expected_span_ticks=expected_span_ticks,
            notes=(
                f"measured/expected span ratio {scale:.4f} outside "
                f"[{scale_min}, {scale_max}]; hardware and URDF disagree",
            ),
        )

    # q0 sits where q == 0, walking back from the min contact by its angle.
    ticks_from_min_to_zero = (-minimum.expected_angle_rad) * TICKS_PER_RAD * scale
    q0_float = minimum.contact_tick + direction * ticks_from_min_to_zero
    q0_tick = normalize_tick(int(math.floor(q0_float + 0.5)))

    def residual_deg(measurement: EndpointMeasurement) -> float:
        predicted_ticks = measurement.expected_angle_rad * TICKS_PER_RAD * scale
        predicted_tick = q0_tick + direction * predicted_ticks
        error_ticks = signed_tick_delta(
            measurement.contact_tick, int(round(predicted_tick))
        )
        return error_ticks * 360.0 / ENCODER_MODULUS

    if q0_tick < 0 or q0_tick > ENCODER_MODULUS - 1:
        return DerivedQ0(
            joint_name, bus_id, DerivationStatus.WRAP_DOMAIN_VIOLATION,
            notes=("derived q0 left the unsigned 0..4095 domain",),
        )

    notes.append(
        "q0 derived from measured contacts and current URDF geometry; "
        "servo PositionOffset remains 0 and is never written"
    )

    return DerivedQ0(
        joint_name=joint_name,
        bus_id=bus_id,
        status=DerivationStatus.OK,
        q0_tick=q0_tick,
        direction=direction,
        scale=scale,
        measured_span_ticks=measured_span_ticks,
        expected_span_ticks=expected_span_ticks,
        residual_min_deg=residual_deg(minimum),
        residual_max_deg=residual_deg(maximum),
        notes=tuple(notes),
    )


@dataclass(frozen=True)
class Q0CrossCheck:
    """Comparison of the manual candidate against the geometry-derived q0."""

    joint_name: str
    status: CrossCheckStatus
    disagreement_ticks: int | None = None
    disagreement_deg: float | None = None
    tolerance_ticks: int | None = None
    diagnosis: str = ""

    @property
    def ok(self) -> bool:
        return self.status is CrossCheckStatus.AGREE


def cross_check_q0(
    manual: ManualQ0Candidate | None,
    derived: DerivedQ0 | None,
    tolerance_ticks: int | None,
) -> Q0CrossCheck:
    """Compare the two q0 estimates and fail closed on disagreement.

    ``tolerance_ticks`` is ``None`` until the manual-pose uncertainty has been
    characterized on this build. While it is ``None`` this returns
    ``BLOCKED_TOLERANCE_UNVALIDATED``: an uncharacterized tolerance cannot
    certify agreement, and inventing one would be exactly the "adjust the
    threshold until it passes" failure the policy forbids.

    The two values are NEVER averaged and no EEPROM is touched on disagreement.
    """
    name = (manual.joint_name if manual else None) or (
        derived.joint_name if derived else "<unknown>"
    )

    if manual is None or derived is None:
        return Q0CrossCheck(
            name, CrossCheckStatus.MISSING_INPUT,
            diagnosis="both a manual candidate and a derived q0 are required",
        )
    if not manual.ok:
        return Q0CrossCheck(
            name, CrossCheckStatus.MISSING_INPUT,
            diagnosis=f"manual candidate is not usable: {manual.status.value}",
        )
    if not derived.ok or derived.q0_tick is None:
        return Q0CrossCheck(
            name, CrossCheckStatus.MISSING_INPUT,
            diagnosis=f"derived q0 is not usable: {derived.status.value}",
        )

    disagreement = signed_tick_delta(manual.median_tick, derived.q0_tick)
    disagreement_deg = disagreement * 360.0 / ENCODER_MODULUS

    if tolerance_ticks is None:
        return Q0CrossCheck(
            name, CrossCheckStatus.BLOCKED_TOLERANCE_UNVALIDATED,
            disagreement_ticks=disagreement,
            disagreement_deg=disagreement_deg,
            diagnosis=(
                "MANUAL_Q0_VS_DERIVED_Q0_TOLERANCE_TICKS is CHARACTERIZATION_REQUIRED; "
                "agreement cannot be certified and q0 must not be promoted"
            ),
        )

    if abs(disagreement) <= tolerance_ticks:
        return Q0CrossCheck(
            name, CrossCheckStatus.AGREE,
            disagreement_ticks=disagreement,
            disagreement_deg=disagreement_deg,
            tolerance_ticks=tolerance_ticks,
            diagnosis="manual pose and geometry-derived q0 agree within tolerance",
        )

    return Q0CrossCheck(
        name, CrossCheckStatus.DISAGREE,
        disagreement_ticks=disagreement,
        disagreement_deg=disagreement_deg,
        tolerance_ticks=tolerance_ticks,
        diagnosis=(
            f"manual and derived q0 differ by {disagreement} ticks "
            f"({disagreement_deg:.2f} deg), beyond {tolerance_ticks} ticks. "
            "Candidate causes, in order of likelihood: manual pose error against "
            "the square; contact detection error; mechanical assembly error; "
            "genuine geometry mismatch. Do NOT average, do NOT write "
            "PositionOffset, do NOT edit the URDF to force agreement."
        ),
    )
