"""MATDOG — conversioni pure encoder ST3215 <-> radianti URDF.

Questo modulo non apre porte seriali e non invia comandi ai servo.
È la sorgente matematica unica per calibrazione, viewer, FK, IK,
pose e gait.
"""

from __future__ import annotations

import math
from numbers import Integral, Real

ENCODER_MODULUS = 4096
HALF_ENCODER_RANGE = ENCODER_MODULUS // 2
RAD_PER_TICK = math.tau / ENCODER_MODULUS
TICKS_PER_RAD = ENCODER_MODULUS / math.tau


def _require_tick(value: int, field_name: str) -> int:
    if not isinstance(value, Integral):
        raise TypeError(f"{field_name} deve essere un intero, ricevuto {value!r}")
    return int(value)


def _require_direction(direction: int) -> int:
    if direction not in (-1, 1):
        raise ValueError(
            f"direction deve essere +1 oppure -1, ricevuto {direction!r}"
        )
    return direction


def _require_finite_angle(angle_rad: float) -> float:
    if not isinstance(angle_rad, Real):
        raise TypeError(f"joint_rad deve essere numerico, ricevuto {angle_rad!r}")

    angle = float(angle_rad)
    if not math.isfinite(angle):
        raise ValueError(f"joint_rad deve essere finito, ricevuto {angle!r}")

    return angle


def normalize_tick(tick: int) -> int:
    """Riporta un tick encoder nel dominio [0, 4095]."""
    return _require_tick(tick, "tick") % ENCODER_MODULUS


def signed_tick_delta(present_tick: int, reference_tick: int) -> int:
    """Differenza circolare present-reference nel range [-2048, 2047].

    Nel MATDOG gli intervalli articolari sono inferiori a mezzo giro,
    quindi questo rappresenta in modo non ambiguo la variazione locale
    rispetto allo zero del giunto.
    """
    present = normalize_tick(present_tick)
    reference = normalize_tick(reference_tick)

    return (
        (present - reference + HALF_ENCODER_RANGE) % ENCODER_MODULUS
    ) - HALF_ENCODER_RANGE


def circular_tick_summary(ticks: list[int]) -> tuple[int, int]:
    """Restituisce mediana e spread sul cerchio encoder 0..4095.

    La funzione trova l'arco circolare minimo che contiene tutti i
    campioni. Questo evita falsi spread enormi vicino alla transizione
    4095 -> 0 ed è indipendente dall'ordine di arrivo dei campioni.
    """
    if not ticks:
        raise ValueError("ticks non può essere vuoto")

    normalized = sorted(normalize_tick(tick) for tick in ticks)

    if len(normalized) == 1:
        return normalized[0], 0

    gaps = []

    for index, current in enumerate(normalized):
        following = normalized[(index + 1) % len(normalized)]

        if index == len(normalized) - 1:
            following += ENCODER_MODULUS

        gaps.append(following - current)

    largest_gap_index = gaps.index(max(gaps))
    arc_start = normalized[(largest_gap_index + 1) % len(normalized)]

    unwrapped = sorted(
        arc_start + ((tick - arc_start) % ENCODER_MODULUS)
        for tick in normalized
    )

    midpoint = len(unwrapped) // 2

    if len(unwrapped) % 2 == 1:
        median_unwrapped = unwrapped[midpoint]
    else:
        median_unwrapped = (
            unwrapped[midpoint - 1] + unwrapped[midpoint] + 1
        ) // 2

    spread = unwrapped[-1] - unwrapped[0]
    return normalize_tick(median_unwrapped), spread


def encoder_to_joint_rad(
    present_tick: int,
    zero_tick: int,
    direction: int,
) -> float:
    """Converte posizione encoder assoluta in angolo URDF del joint."""
    delta_tick = signed_tick_delta(present_tick, zero_tick)
    return _require_direction(direction) * delta_tick * RAD_PER_TICK


def _round_nearest_tick(value: float) -> int:
    """Arrotondamento simmetrico, evitando il banker rounding di Python."""
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def joint_rad_to_encoder(
    joint_rad: float,
    zero_tick: int,
    direction: int,
) -> int:
    """Converte un angolo URDF in target encoder assoluto [0, 4095]."""
    angle = _require_finite_angle(joint_rad)
    sign = _require_direction(direction)

    delta_tick = _round_nearest_tick(sign * angle * TICKS_PER_RAD)
    return normalize_tick(normalize_tick(zero_tick) + delta_tick)


def encoder_round_trip_error(
    present_tick: int,
    zero_tick: int,
    direction: int,
) -> int:
    """Errore in tick dopo encoder -> rad -> encoder."""
    angle = encoder_to_joint_rad(present_tick, zero_tick, direction)
    reconstructed = joint_rad_to_encoder(angle, zero_tick, direction)

    return signed_tick_delta(reconstructed, present_tick)
