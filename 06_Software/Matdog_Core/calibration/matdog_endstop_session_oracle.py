#!/usr/bin/env python3
"""Pure offline oracle for the ordered MATDOG end-stop session.

No Station, serial, filesystem persistence or hardware commands are used.
The oracle specifies phase dependencies and contact/return classification that
future Rust code must reproduce.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


STATIC_TOLERANCE_TICKS = 10
INTERMEDIATE_PROBE_SETTLE_TICKS = 16


class ContactClass(Enum):
    FREE_MOTION = auto()
    CONTACT_SUSPECTED = auto()
    EXPECTED_CORRIDOR_STALL = auto()
    EARLY_STALL = auto()
    HARD_ABORT = auto()


class Joint(Enum):
    UPPER = auto()
    LOWER = auto()
    HIP = auto()


class Side(Enum):
    MIN = auto()
    MAX = auto()


@dataclass(frozen=True)
class ContactObservation:
    present_tick: int
    target_tick: int
    inner_tick: int
    outer_guard_tick: int
    probe_sign: int
    travel_ticks: int
    progress_ticks: int
    velocity_raw: int
    current_raw: int
    hard_current_abort_raw: int
    healthy: bool = True
    telemetry_fresh: bool = True
    persistence_samples: int = 3


@dataclass(frozen=True)
class ContactPolicy:
    minimum_travel_ticks: int = 24
    maximum_progress_ticks: int = 2
    maximum_velocity_raw: int = 10
    required_persistence_samples: int = 3


def directional_progress(target: int, origin: int, sign: int) -> int:
    return max(0, (target - origin) * sign)


def inside_directed_corridor(value: int, inner: int, outer: int, sign: int) -> bool:
    if sign > 0:
        return inner <= value <= outer
    return outer <= value <= inner


def before_inner_corridor(value: int, inner: int, sign: int) -> bool:
    return value < inner if sign > 0 else value > inner


def classify_contact(
    observation: ContactObservation,
    policy: ContactPolicy = ContactPolicy(),
) -> ContactClass:
    if (
        not observation.healthy
        or not observation.telemetry_fresh
        or observation.current_raw >= observation.hard_current_abort_raw
        or observation.probe_sign not in (-1, 1)
    ):
        return ContactClass.HARD_ABORT

    target_ahead = directional_progress(
        observation.target_tick,
        observation.present_tick,
        observation.probe_sign,
    ) > 0
    stalled = (
        observation.travel_ticks >= policy.minimum_travel_ticks
        and observation.progress_ticks <= policy.maximum_progress_ticks
        and observation.velocity_raw <= policy.maximum_velocity_raw
        and target_ahead
    )
    if not stalled:
        return ContactClass.FREE_MOTION

    if before_inner_corridor(
        observation.present_tick,
        observation.inner_tick,
        observation.probe_sign,
    ):
        return ContactClass.EARLY_STALL

    if not inside_directed_corridor(
        observation.present_tick,
        observation.inner_tick,
        observation.outer_guard_tick,
        observation.probe_sign,
    ):
        return ContactClass.HARD_ABORT

    if observation.persistence_samples < policy.required_persistence_samples:
        return ContactClass.CONTACT_SUSPECTED
    return ContactClass.EXPECTED_CORRIDOR_STALL


def probe_return_state(present_tick: int, home_tick: int = 2048) -> str:
    error = abs(present_tick - home_tick)
    if error <= STATIC_TOLERANCE_TICKS:
        return "STATIC_HANDOFF_ALLOWED"
    if error <= INTERMEDIATE_PROBE_SETTLE_TICKS:
        return "ACTIVE_NUDGE_REQUIRED"
    return "RECOVERY_FAILED_KEEP_PREREQUISITES"


@dataclass
class OrderedLegSession:
    imported_upper_checkpoint: bool = False
    contacts: dict[tuple[Joint, Side], int] = field(default_factory=dict)
    upper_horizontal_validated: bool = False
    lower_compact_validated: bool = False
    fixture_validated: bool = False

    def record_contact(self, joint: Joint, side: Side, tick: int) -> None:
        if not 0 <= tick <= 4095:
            raise ValueError("contact tick outside unsigned ST3215 range")
        if joint is Joint.LOWER and not self.upper_pair_complete:
            raise RuntimeError("LOWER requires UPPER MIN+MAX")
        if joint is Joint.LOWER and not self.upper_horizontal_validated:
            raise RuntimeError("LOWER requires validated UPPER horizontal pose")
        if joint is Joint.HIP and not self.lower_pair_complete:
            raise RuntimeError("HIP requires LOWER MIN+MAX")
        if joint is Joint.HIP and not self.lower_compact_validated:
            raise RuntimeError("HIP requires validated compact LOWER pose")
        if joint is Joint.HIP and not self.fixture_validated:
            raise RuntimeError("HIP requires measured fixture keep-out PASS")
        self.contacts[(joint, side)] = tick

    @property
    def upper_pair_complete(self) -> bool:
        return self.imported_upper_checkpoint or all(
            (Joint.UPPER, side) in self.contacts for side in Side
        )

    @property
    def lower_pair_complete(self) -> bool:
        return all((Joint.LOWER, side) in self.contacts for side in Side)

    @property
    def hip_pair_complete(self) -> bool:
        return all((Joint.HIP, side) in self.contacts for side in Side)

    def next_joint(self) -> Joint | None:
        if not self.upper_pair_complete:
            return Joint.UPPER
        if not self.lower_pair_complete:
            return Joint.LOWER
        if not self.hip_pair_complete:
            return Joint.HIP
        return None
