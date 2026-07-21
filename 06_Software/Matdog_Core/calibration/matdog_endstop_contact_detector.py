#!/usr/bin/env python3
"""
MATDOG — detector puro di contatto per la calibrazione degli end-stop.

Il modulo elabora campioni telemetrici già acquisiti da un livello esterno.
Non apre NormaCore Station, non apre porte seriali e non invia comandi.

La conferma del contatto richiede contemporaneamente:
- telemetria monotona e senza errori;
- avanzamento già iniziato;
- perdita di velocità/progresso;
- aumento di corrente rispetto alla baseline mobile;
- persistenza dell'evidenza su più campioni;
- rispetto dei limiti di tempo e corsa.

La corrente resta espressa in unità raw ST3215 non caratterizzate.
Le soglie devono quindi essere esplicite e validate prima dell'hardware.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import statistics

from matdog_joint_math import normalize_tick, signed_tick_delta


class ContactState(str, Enum):
    FREE_MOTION = "FREE_MOTION"
    CONTACT_SUSPECTED = "CONTACT_SUSPECTED"
    CONTACT_CONFIRMED = "CONTACT_CONFIRMED"
    CONTACT_REPEATABLE = "CONTACT_REPEATABLE"
    AMBIGUOUS_CONTACT = "AMBIGUOUS_CONTACT"
    HARD_ABORT = "HARD_ABORT"


@dataclass(frozen=True)
class TelemetrySample:
    monotonic_stamp_ns: int
    present_tick: int
    current_raw: int
    error_status: int = 0


@dataclass(frozen=True)
class ContactPolicy:
    baseline_window: int = 7
    minimum_baseline_samples: int = 4
    moving_velocity_floor_tick_s: float = 15.0
    stall_velocity_ceiling_tick_s: float = 3.0
    current_rise_raw: int = 8
    minimum_travel_before_contact_tick: int = 10
    suspicion_consecutive_samples: int = 2
    confirmation_consecutive_samples: int = 3
    max_sample_gap_s: float = 0.25
    timeout_s: float = 8.0
    reverse_tolerance_tick: int = 3

    def validate(self) -> None:
        integer_positive = {
            "baseline_window": self.baseline_window,
            "minimum_baseline_samples": self.minimum_baseline_samples,
            "current_rise_raw": self.current_rise_raw,
            "minimum_travel_before_contact_tick":
                self.minimum_travel_before_contact_tick,
            "suspicion_consecutive_samples":
                self.suspicion_consecutive_samples,
            "confirmation_consecutive_samples":
                self.confirmation_consecutive_samples,
        }

        for name, value in integer_positive.items():
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} deve essere un intero > 0")

        if self.minimum_baseline_samples > self.baseline_window:
            raise ValueError(
                "minimum_baseline_samples non può superare baseline_window"
            )

        if (
            self.confirmation_consecutive_samples
            < self.suspicion_consecutive_samples
        ):
            raise ValueError(
                "confirmation_consecutive_samples deve essere >= "
                "suspicion_consecutive_samples"
            )

        for name, value in {
            "moving_velocity_floor_tick_s":
                self.moving_velocity_floor_tick_s,
            "stall_velocity_ceiling_tick_s":
                self.stall_velocity_ceiling_tick_s,
            "max_sample_gap_s": self.max_sample_gap_s,
            "timeout_s": self.timeout_s,
        }.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} deve essere finito e > 0")

        if (
            self.moving_velocity_floor_tick_s
            <= self.stall_velocity_ceiling_tick_s
        ):
            raise ValueError(
                "moving_velocity_floor_tick_s deve essere maggiore di "
                "stall_velocity_ceiling_tick_s"
            )

        if self.reverse_tolerance_tick < 0:
            raise ValueError("reverse_tolerance_tick deve essere >= 0")


@dataclass(frozen=True)
class ContactObservation:
    state: ContactState
    reason: str
    present_tick: int
    travel_tick: int
    directed_step_tick: int | None
    velocity_tick_s: float | None
    current_raw: int
    baseline_current_raw: float | None
    current_rise_raw: float | None
    evidence_streak: int


class ContactDetector:
    def __init__(
        self,
        *,
        start_tick: int,
        approach_direction: int,
        max_travel_tick: int,
        policy: ContactPolicy,
    ):
        if approach_direction not in (-1, 1):
            raise ValueError("approach_direction deve essere -1 oppure +1")

        if not isinstance(max_travel_tick, int) or max_travel_tick <= 0:
            raise ValueError("max_travel_tick deve essere un intero > 0")

        policy.validate()

        self.start_tick = normalize_tick(start_tick)
        self.approach_direction = approach_direction
        self.max_travel_tick = max_travel_tick
        self.policy = policy

        self.state = ContactState.FREE_MOTION
        self.previous: TelemetrySample | None = None
        self.started_stamp_ns: int | None = None
        self.evidence_streak = 0
        self.baseline_currents: deque[int] = deque(
            maxlen=policy.baseline_window
        )

    def _abort(
        self,
        sample: TelemetrySample,
        reason: str,
        travel_tick: int,
        directed_step_tick: int | None = None,
        velocity_tick_s: float | None = None,
    ) -> ContactObservation:
        self.state = ContactState.HARD_ABORT

        baseline = self._baseline_current()
        rise = (
            float(sample.current_raw) - baseline
            if baseline is not None
            else None
        )

        return ContactObservation(
            state=self.state,
            reason=reason,
            present_tick=normalize_tick(sample.present_tick),
            travel_tick=travel_tick,
            directed_step_tick=directed_step_tick,
            velocity_tick_s=velocity_tick_s,
            current_raw=int(sample.current_raw),
            baseline_current_raw=baseline,
            current_rise_raw=rise,
            evidence_streak=self.evidence_streak,
        )

    def _baseline_current(self) -> float | None:
        if (
            len(self.baseline_currents)
            < self.policy.minimum_baseline_samples
        ):
            return None

        return float(statistics.median(self.baseline_currents))

    def ingest(self, sample: TelemetrySample) -> ContactObservation:
        if self.state in {
            ContactState.CONTACT_CONFIRMED,
            ContactState.HARD_ABORT,
        }:
            travel = self.approach_direction * signed_tick_delta(
                sample.present_tick,
                self.start_tick,
            )
            return ContactObservation(
                state=self.state,
                reason="TERMINAL_STATE_LATCHED",
                present_tick=normalize_tick(sample.present_tick),
                travel_tick=travel,
                directed_step_tick=None,
                velocity_tick_s=None,
                current_raw=int(sample.current_raw),
                baseline_current_raw=self._baseline_current(),
                current_rise_raw=None,
                evidence_streak=self.evidence_streak,
            )

        if sample.monotonic_stamp_ns < 0:
            return self._abort(
                sample,
                "INVALID_NEGATIVE_TIMESTAMP",
                travel_tick=0,
            )

        if sample.current_raw < 0:
            return self._abort(
                sample,
                "INVALID_NEGATIVE_CURRENT",
                travel_tick=0,
            )

        present = normalize_tick(sample.present_tick)
        travel = self.approach_direction * signed_tick_delta(
            present,
            self.start_tick,
        )

        if sample.error_status != 0:
            return self._abort(
                sample,
                f"SERVO_STATUS_ERROR_0x{sample.error_status:02X}",
                travel_tick=travel,
            )

        if travel < -self.policy.reverse_tolerance_tick:
            return self._abort(
                sample,
                "UNEXPECTED_REVERSE_TRAVEL",
                travel_tick=travel,
            )

        if travel > self.max_travel_tick:
            return self._abort(
                sample,
                "MODEL_TRAVEL_GUARD_EXCEEDED",
                travel_tick=travel,
            )

        if self.started_stamp_ns is None:
            self.started_stamp_ns = sample.monotonic_stamp_ns

        elapsed_s = (
            sample.monotonic_stamp_ns - self.started_stamp_ns
        ) / 1_000_000_000.0

        if elapsed_s > self.policy.timeout_s:
            return self._abort(
                sample,
                "APPROACH_TIMEOUT",
                travel_tick=travel,
            )

        directed_step: int | None = None
        velocity_tick_s: float | None = None

        if self.previous is not None:
            delta_ns = (
                sample.monotonic_stamp_ns
                - self.previous.monotonic_stamp_ns
            )

            if delta_ns <= 0:
                return self._abort(
                    sample,
                    "NON_MONOTONIC_MOTOR_TIMESTAMP",
                    travel_tick=travel,
                )

            gap_s = delta_ns / 1_000_000_000.0

            if gap_s > self.policy.max_sample_gap_s:
                return self._abort(
                    sample,
                    "TELEMETRY_GAP_EXCEEDED",
                    travel_tick=travel,
                )

            directed_step = (
                self.approach_direction
                * signed_tick_delta(
                    present,
                    self.previous.present_tick,
                )
            )
            velocity_tick_s = directed_step / gap_s

            if (
                velocity_tick_s
                >= self.policy.moving_velocity_floor_tick_s
                and self.state == ContactState.FREE_MOTION
            ):
                self.baseline_currents.append(int(sample.current_raw))

        baseline = self._baseline_current()
        current_rise = (
            float(sample.current_raw) - baseline
            if baseline is not None
            else None
        )

        travel_ready = (
            travel
            >= self.policy.minimum_travel_before_contact_tick
        )
        stalled = (
            velocity_tick_s is not None
            and abs(velocity_tick_s)
            <= self.policy.stall_velocity_ceiling_tick_s
        )
        current_evidence = (
            current_rise is not None
            and current_rise >= self.policy.current_rise_raw
        )

        combined_evidence = (
            travel_ready
            and stalled
            and current_evidence
        )

        if combined_evidence:
            self.evidence_streak += 1
        else:
            self.evidence_streak = 0

        if (
            self.evidence_streak
            >= self.policy.confirmation_consecutive_samples
        ):
            self.state = ContactState.CONTACT_CONFIRMED
            reason = "STALL_AND_CURRENT_RISE_PERSISTENT"
        elif (
            self.evidence_streak
            >= self.policy.suspicion_consecutive_samples
        ):
            self.state = ContactState.CONTACT_SUSPECTED
            reason = "STALL_AND_CURRENT_RISE_SUSPECTED"
        else:
            self.state = ContactState.FREE_MOTION

            if baseline is None:
                reason = "COLLECTING_MOVING_CURRENT_BASELINE"
            elif not travel_ready:
                reason = "MINIMUM_TRAVEL_NOT_REACHED"
            else:
                reason = "NO_COMBINED_CONTACT_EVIDENCE"

        self.previous = TelemetrySample(
            monotonic_stamp_ns=int(sample.monotonic_stamp_ns),
            present_tick=present,
            current_raw=int(sample.current_raw),
            error_status=int(sample.error_status),
        )

        return ContactObservation(
            state=self.state,
            reason=reason,
            present_tick=present,
            travel_tick=travel,
            directed_step_tick=directed_step,
            velocity_tick_s=velocity_tick_s,
            current_raw=int(sample.current_raw),
            baseline_current_raw=baseline,
            current_rise_raw=current_rise,
            evidence_streak=self.evidence_streak,
        )


def backoff_distance_tick(
    *,
    contact_tick: int,
    recovered_tick: int,
    approach_direction: int,
) -> int:
    if approach_direction not in (-1, 1):
        raise ValueError("approach_direction deve essere -1 oppure +1")

    return (
        -approach_direction
        * signed_tick_delta(recovered_tick, contact_tick)
    )


def recovery_verified(
    *,
    contact_tick: int,
    recovered_tick: int,
    approach_direction: int,
    minimum_backoff_tick: int,
) -> bool:
    if minimum_backoff_tick <= 0:
        raise ValueError("minimum_backoff_tick deve essere > 0")

    return backoff_distance_tick(
        contact_tick=contact_tick,
        recovered_tick=recovered_tick,
        approach_direction=approach_direction,
    ) >= minimum_backoff_tick


def contact_repeatability_spread_tick(
    first_contact_tick: int,
    second_contact_tick: int,
) -> int:
    return abs(signed_tick_delta(
        second_contact_tick,
        first_contact_tick,
    ))


def contact_is_repeatable(
    *,
    first_contact_tick: int,
    second_contact_tick: int,
    tolerance_tick: int,
) -> bool:
    if tolerance_tick < 0:
        raise ValueError("tolerance_tick deve essere >= 0")

    return contact_repeatability_spread_tick(
        first_contact_tick,
        second_contact_tick,
    ) <= tolerance_tick
