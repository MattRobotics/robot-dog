#!/usr/bin/env python3
"""Offline MATDOG model-zero solver from repeated MIN/MAX contacts.

This module performs no I/O to Station or servos. It keeps the ST3215 encoder
scale and the URDF joint range fixed, derives one q=0 candidate from each
measured physical endpoint, and accepts a calibrated software HOME only when
both candidates agree within the configured tolerance.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from statistics import median
from typing import Iterable

ENCODER_MODULUS = 4096
DIGITAL_HOME_TICK = 2048
ENDPOINT_CONSISTENCY_TICKS = 24
MAX_SHIFT_FROM_DIGITAL_HOME_TICKS = 96


@dataclass(frozen=True)
class JointModel:
    name: str
    servo_id: int
    direction: int
    urdf_min_delta_ticks: int
    urdf_max_delta_ticks: int


@dataclass(frozen=True)
class ModelZeroResult:
    joint: str
    servo_id: int
    minimum_contact_tick: int
    maximum_contact_tick: int
    zero_from_minimum_tick: int
    zero_from_maximum_tick: int
    endpoint_disagreement_ticks: int
    estimated_zero_tick: int
    shift_from_digital_home_ticks: int
    accepted: bool
    decision: str


LF_MODELS = {
    "upper": JointModel("lf_upper_leg_joint", 12, +1, -597, +1394),
    "lower": JointModel("lf_lower_leg_joint", 11, -1, -1047, +427),
    "hip": JointModel("lf_hip_joint", 13, -1, -512, +512),
}


def normalize_tick(value: int) -> int:
    return int(value) % ENCODER_MODULUS


def signed_tick_delta(value: int, reference: int) -> int:
    return (
        (normalize_tick(value) - normalize_tick(reference) + ENCODER_MODULUS // 2)
        % ENCODER_MODULUS
    ) - ENCODER_MODULUS // 2


def circular_distance(first: int, second: int) -> int:
    return abs(signed_tick_delta(first, second))


def circular_midpoint(first: int, second: int) -> int:
    return normalize_tick(first + int(signed_tick_delta(second, first) / 2))


def repeated_contact_tick(values: Iterable[int]) -> int:
    ticks = [normalize_tick(value) for value in values]
    if not ticks:
        raise ValueError("at least one contact tick is required")
    reference = ticks[0]
    unwrapped = [reference + signed_tick_delta(value, reference) for value in ticks]
    return normalize_tick(round(median(unwrapped)))


def zero_candidate(model: JointModel, contact_tick: int, q_delta_ticks: int) -> int:
    return normalize_tick(contact_tick - model.direction * q_delta_ticks)


def solve_model_zero(
    model: JointModel,
    minimum_contacts: Iterable[int],
    maximum_contacts: Iterable[int],
    *,
    endpoint_consistency_ticks: int = ENDPOINT_CONSISTENCY_TICKS,
    max_shift_from_digital_home_ticks: int = MAX_SHIFT_FROM_DIGITAL_HOME_TICKS,
) -> ModelZeroResult:
    minimum = repeated_contact_tick(minimum_contacts)
    maximum = repeated_contact_tick(maximum_contacts)
    zero_min = zero_candidate(model, minimum, model.urdf_min_delta_ticks)
    zero_max = zero_candidate(model, maximum, model.urdf_max_delta_ticks)
    disagreement = circular_distance(zero_min, zero_max)
    estimated = circular_midpoint(zero_min, zero_max)
    shift = circular_distance(estimated, DIGITAL_HOME_TICK)
    accepted = (
        disagreement <= endpoint_consistency_ticks
        and shift <= max_shift_from_digital_home_ticks
    )
    return ModelZeroResult(
        joint=model.name,
        servo_id=model.servo_id,
        minimum_contact_tick=minimum,
        maximum_contact_tick=maximum,
        zero_from_minimum_tick=zero_min,
        zero_from_maximum_tick=zero_max,
        endpoint_disagreement_ticks=disagreement,
        estimated_zero_tick=estimated,
        shift_from_digital_home_ticks=shift,
        accepted=accepted,
        decision="MODEL_ZERO_ACCEPTED" if accepted else "MODEL_ZERO_INCONSISTENT",
    )


def parse_ticks(value: str) -> list[int]:
    try:
        ticks = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    if not ticks:
        raise argparse.ArgumentTypeError("provide one or more comma-separated ticks")
    if any(not 0 <= tick < ENCODER_MODULUS for tick in ticks):
        raise argparse.ArgumentTypeError("ticks must be in 0..4095")
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=sorted(LF_MODELS), required=True)
    parser.add_argument("--min", dest="minimum", type=parse_ticks, required=True)
    parser.add_argument("--max", dest="maximum", type=parse_ticks, required=True)
    args = parser.parse_args()
    result = solve_model_zero(LF_MODELS[args.joint], args.minimum, args.maximum)
    print(json.dumps(asdict(result), indent=2, sort_keys=True))
    return 0 if result.accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
