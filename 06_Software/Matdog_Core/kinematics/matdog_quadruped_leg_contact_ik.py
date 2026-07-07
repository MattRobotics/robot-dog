#!/usr/bin/env python3
"""
MATDOG — generic contact-reference IK for LF, RF, RH and LH.

Pipeline:
    leg ID + target contact reference in world
    → numerical IK
    → canonical URDF FK
    → common foot contact model
    → final contact-mode validation.

The IK target is the continuous cross-section contact reference of the
cylindrical foot. The finite tread lowest-core point remains available for
future ground-clearance and support validation.

No Station, serial port or motor command is used.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys

from matdog_foot_contact import (
    FootContactDegeneracyError,
    Matrix3,
    Vector3,
)
from matdog_quadruped_leg_contact import (
    LEG_IDS,
    QuadrupedLegContactError,
    QuadrupedLegContactResult,
    leg_foot_contact_from_joint_angles,
    leg_joint_names,
    load_leg_kinematic_contract,
)


REPO_ROOT = Path(__file__).resolve().parents[3]

IDENTITY_MATRIX3: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class QuadrupedContactIkError(RuntimeError):
    """Errore base della contact-reference IK quadrupede."""


class QuadrupedContactIkConstraintError(QuadrupedContactIkError):
    """La soluzione viola una policy di contatto richiesta."""


class QuadrupedContactIkUnreachableError(QuadrupedContactIkError):
    """Il target non converge entro limiti URDF e tolleranza."""


@dataclass(frozen=True)
class QuadrupedLegContactIkResult:
    leg_id: str
    target_contact_reference_world_m: Vector3
    achieved_contact_reference_world_m: Vector3
    joint_positions_rad: dict[str, float]
    residual_m: float
    iterations: int
    leg_contact: QuadrupedLegContactResult


def _vector3(
    value: tuple[float, float, float] | list[float],
    field_name: str,
) -> Vector3:
    if len(value) != 3:
        raise ValueError(
            f"{field_name}: attesi esattamente tre valori"
        )

    result = tuple(float(component) for component in value)

    if not all(math.isfinite(component) for component in result):
        raise ValueError(
            f"{field_name}: contiene valori non finiti"
        )

    return result  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def _clamp_joint_vector(
    values: tuple[float, float, float],
    limits: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
) -> tuple[float, float, float]:
    return tuple(
        min(max(value, lower), upper)
        for value, (lower, upper) in zip(values, limits)
    )  # type: ignore[return-value]


def _validate_initial_guess(
    leg_id: str,
    initial_guess_rad: tuple[float, float, float] | None,
    joint_names: tuple[str, str, str],
    limits: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
) -> tuple[float, float, float]:
    if initial_guess_rad is None:
        return _clamp_joint_vector(
            (0.0, 0.0, 0.0),
            limits,
        )

    values = _vector3(
        initial_guess_rad,
        "initial_guess_rad",
    )

    for joint_name, value, (lower, upper) in zip(
        joint_names,
        values,
        limits,
    ):
        if not lower <= value <= upper:
            raise ValueError(
                f"{leg_id}.{joint_name}: initial guess "
                f"{value:.9f} rad fuori limiti "
                f"[{lower:.9f}, {upper:.9f}]"
            )

    return values


def _solve_3x3(
    matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    augmented = [
        [
            matrix[row][0],
            matrix[row][1],
            matrix[row][2],
            vector[row],
        ]
        for row in range(3)
    ]

    for column in range(3):
        pivot_row = max(
            range(column, 3),
            key=lambda row: abs(augmented[row][column]),
        )

        if abs(augmented[pivot_row][column]) < 1e-14:
            raise QuadrupedContactIkError(
                "Jacobiano contact-reference singolare o degenerato"
            )

        augmented[column], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[column],
        )

        pivot = augmented[column][column]

        for item in range(column, 4):
            augmented[column][item] /= pivot

        for row in range(3):
            if row == column:
                continue

            factor = augmented[row][column]

            for item in range(column, 4):
                augmented[row][item] -= (
                    factor * augmented[column][item]
                )

    return (
        augmented[0][3],
        augmented[1][3],
        augmented[2][3],
    )


def _damped_least_squares_step(
    jacobian: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    cartesian_error: Vector3,
    damping: float,
) -> tuple[float, float, float]:
    if damping <= 0.0:
        raise ValueError("damping deve essere > 0")

    normal_rows: list[tuple[float, float, float]] = []

    for row in range(3):
        values: list[float] = []

        for column in range(3):
            value = sum(
                jacobian[index][row] * jacobian[index][column]
                for index in range(3)
            )

            if row == column:
                value += damping * damping

            values.append(value)

        normal_rows.append(
            (values[0], values[1], values[2])
        )

    right_hand_side = tuple(
        sum(
            jacobian[index][column]
            * cartesian_error[index]
            for index in range(3)
        )
        for column in range(3)
    )

    return _solve_3x3(
        (
            normal_rows[0],
            normal_rows[1],
            normal_rows[2],
        ),
        right_hand_side,
    )


def _contact_reference_from_q(
    leg_id: str,
    q_rad: tuple[float, float, float],
    repo_root: Path,
    world_from_base_rotation: Matrix3,
    world_from_base_translation_m: Vector3,
    ground_normal_world_unit: Vector3,
) -> tuple[Vector3, QuadrupedLegContactResult]:
    joint_names = leg_joint_names(leg_id)

    result = leg_foot_contact_from_joint_angles(
        leg_id=leg_id,
        joint_positions_rad=dict(zip(joint_names, q_rad)),
        repo_root=repo_root,
        world_from_base_rotation=world_from_base_rotation,
        world_from_base_translation_m=world_from_base_translation_m,
        ground_normal_world_unit=ground_normal_world_unit,
    )

    return (
        result.contact.cross_section_contact_center_world_m,
        result,
    )


def _numerical_contact_reference_jacobian(
    leg_id: str,
    q_rad: tuple[float, float, float],
    limits: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
    step_rad: float,
    repo_root: Path,
    world_from_base_rotation: Matrix3,
    world_from_base_translation_m: Vector3,
    ground_normal_world_unit: Vector3,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    columns: list[Vector3] = []

    for index in range(3):
        lower, upper = limits[index]

        plus_value = min(
            q_rad[index] + step_rad,
            upper,
        )
        minus_value = max(
            q_rad[index] - step_rad,
            lower,
        )
        denominator = plus_value - minus_value

        if denominator <= 1e-12:
            raise QuadrupedContactIkError(
                f"{leg_id}.{leg_joint_names(leg_id)[index]}: "
                "Jacobiano non calcolabile al limite articolare"
            )

        plus_q = list(q_rad)
        minus_q = list(q_rad)
        plus_q[index] = plus_value
        minus_q[index] = minus_value

        plus_contact, _ = _contact_reference_from_q(
            leg_id=leg_id,
            q_rad=tuple(plus_q),  # type: ignore[arg-type]
            repo_root=repo_root,
            world_from_base_rotation=world_from_base_rotation,
            world_from_base_translation_m=world_from_base_translation_m,
            ground_normal_world_unit=ground_normal_world_unit,
        )

        minus_contact, _ = _contact_reference_from_q(
            leg_id=leg_id,
            q_rad=tuple(minus_q),  # type: ignore[arg-type]
            repo_root=repo_root,
            world_from_base_rotation=world_from_base_rotation,
            world_from_base_translation_m=world_from_base_translation_m,
            ground_normal_world_unit=ground_normal_world_unit,
        )

        columns.append(
            (
                (plus_contact[0] - minus_contact[0])
                / denominator,
                (plus_contact[1] - minus_contact[1])
                / denominator,
                (plus_contact[2] - minus_contact[2])
                / denominator,
            )
        )

    return (
        (columns[0][0], columns[1][0], columns[2][0]),
        (columns[0][1], columns[1][1], columns[2][1]),
        (columns[0][2], columns[1][2], columns[2][2]),
    )


def solve_leg_contact_reference_ik(
    leg_id: str,
    target_contact_reference_world_m: tuple[float, float, float],
    repo_root: Path = REPO_ROOT,
    initial_guess_rad: tuple[float, float, float] | None = None,
    world_from_base_rotation: Matrix3 = IDENTITY_MATRIX3,
    world_from_base_translation_m: Vector3 = (0.0, 0.0, 0.0),
    ground_normal_world_unit: Vector3 = (0.0, 0.0, 1.0),
    require_nominal_strip_contact: bool = True,
    tolerance_m: float = 1e-6,
    max_iterations: int = 250,
    damping: float = 1e-3,
    numerical_step_rad: float = 1e-5,
    maximum_step_rad: float = 0.12,
) -> QuadrupedLegContactIkResult:
    """
    Solve selected-leg IK on the continuous contact-reference point.

    The final solution must satisfy:
    - Cartesian residual within tolerance;
    - canonical URDF joint limits;
    - NOMINAL_STRIP_CONTACT when that policy is requested.

    EDGE_BIASED_CONTACT can be evaluated explicitly by passing
    require_nominal_strip_contact=False. That is offline geometry only and
    does not authorize physical stance or actuation.
    """
    contract = load_leg_kinematic_contract(
        leg_id,
        repo_root,
    )

    target = _vector3(
        target_contact_reference_world_m,
        "target_contact_reference_world_m",
    )

    if tolerance_m <= 0.0:
        raise ValueError("tolerance_m deve essere > 0")

    if max_iterations <= 0:
        raise ValueError("max_iterations deve essere > 0")

    if damping <= 0.0:
        raise ValueError("damping deve essere > 0")

    if numerical_step_rad <= 0.0:
        raise ValueError("numerical_step_rad deve essere > 0")

    if maximum_step_rad <= 0.0:
        raise ValueError("maximum_step_rad deve essere > 0")

    current_q = _validate_initial_guess(
        leg_id=contract.leg_id,
        initial_guess_rad=initial_guess_rad,
        joint_names=contract.joint_names,
        limits=contract.joint_limits_rad,
    )

    for iteration in range(max_iterations + 1):
        current_contact, current_leg_contact = (
            _contact_reference_from_q(
                leg_id=contract.leg_id,
                q_rad=current_q,
                repo_root=repo_root,
                world_from_base_rotation=world_from_base_rotation,
                world_from_base_translation_m=(
                    world_from_base_translation_m
                ),
                ground_normal_world_unit=ground_normal_world_unit,
            )
        )

        cartesian_error = _subtract(
            target,
            current_contact,
        )
        residual = _norm(cartesian_error)

        if residual <= tolerance_m:
            if (
                require_nominal_strip_contact
                and current_leg_contact.contact.support_mode
                != "NOMINAL_STRIP_CONTACT"
            ):
                raise QuadrupedContactIkConstraintError(
                    "La soluzione finale richiede "
                    f"{current_leg_contact.contact.support_mode}; "
                    "policy richiesta: NOMINAL_STRIP_CONTACT"
                )

            return QuadrupedLegContactIkResult(
                leg_id=contract.leg_id,
                target_contact_reference_world_m=target,
                achieved_contact_reference_world_m=current_contact,
                joint_positions_rad=dict(
                    zip(
                        contract.joint_names,
                        current_q,
                    )
                ),
                residual_m=residual,
                iterations=iteration,
                leg_contact=current_leg_contact,
            )

        if iteration == max_iterations:
            break

        jacobian = _numerical_contact_reference_jacobian(
            leg_id=contract.leg_id,
            q_rad=current_q,
            limits=contract.joint_limits_rad,
            step_rad=numerical_step_rad,
            repo_root=repo_root,
            world_from_base_rotation=world_from_base_rotation,
            world_from_base_translation_m=(
                world_from_base_translation_m
            ),
            ground_normal_world_unit=ground_normal_world_unit,
        )

        delta_q = _damped_least_squares_step(
            jacobian=jacobian,
            cartesian_error=cartesian_error,
            damping=damping,
        )

        delta_norm = _norm(delta_q)

        if delta_norm <= 1e-12:
            break

        if delta_norm > maximum_step_rad:
            scale = maximum_step_rad / delta_norm
            delta_q = tuple(
                component * scale
                for component in delta_q
            )

        accepted = False
        line_scale = 1.0

        for _ in range(12):
            candidate_q = _clamp_joint_vector(
                (
                    current_q[0] + line_scale * delta_q[0],
                    current_q[1] + line_scale * delta_q[1],
                    current_q[2] + line_scale * delta_q[2],
                ),
                contract.joint_limits_rad,
            )

            try:
                candidate_contact, _ = _contact_reference_from_q(
                    leg_id=contract.leg_id,
                    q_rad=candidate_q,
                    repo_root=repo_root,
                    world_from_base_rotation=(
                        world_from_base_rotation
                    ),
                    world_from_base_translation_m=(
                        world_from_base_translation_m
                    ),
                    ground_normal_world_unit=ground_normal_world_unit,
                )
            except FootContactDegeneracyError:
                line_scale *= 0.5
                continue

            candidate_residual = _norm(
                _subtract(
                    target,
                    candidate_contact,
                )
            )

            if candidate_residual < residual:
                current_q = candidate_q
                accepted = True
                break

            line_scale *= 0.5

        if not accepted:
            break

    try:
        final_contact, final_leg_contact = _contact_reference_from_q(
            leg_id=contract.leg_id,
            q_rad=current_q,
            repo_root=repo_root,
            world_from_base_rotation=world_from_base_rotation,
            world_from_base_translation_m=(
                world_from_base_translation_m
            ),
            ground_normal_world_unit=ground_normal_world_unit,
        )
        final_residual = _norm(
            _subtract(
                target,
                final_contact,
            )
        )
        final_mode = final_leg_contact.contact.support_mode
    except FootContactDegeneracyError as exc:
        raise QuadrupedContactIkUnreachableError(
            "IK terminata fuori dal dominio geometrico valido: "
            f"{exc}"
        ) from exc

    raise QuadrupedContactIkUnreachableError(
        "IK generic contact-reference non convergente entro "
        "limiti URDF: "
        f"leg={contract.leg_id}, "
        f"target={target}, "
        f"residual={final_residual:.9f} m, "
        f"support_mode={final_mode}, "
        f"iterations={max_iterations}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG generic contact-reference IK, offline only."
        )
    )
    parser.add_argument(
        "--leg",
        required=True,
        choices=LEG_IDS,
    )
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument(
        "--initial-q",
        type=float,
        nargs=3,
        metavar=("HIP", "UPPER", "LOWER"),
        default=None,
    )
    parser.add_argument(
        "--base-translation",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
    )
    parser.add_argument(
        "--allow-edge-contact",
        action="store_true",
        help=(
            "Permette EDGE_BIASED_CONTACT per verifica offline."
        ),
    )
    parser.add_argument(
        "--tolerance-mm",
        type=float,
        default=0.001,
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=250,
    )
    args = parser.parse_args()

    result = solve_leg_contact_reference_ik(
        leg_id=args.leg,
        target_contact_reference_world_m=(
            args.x,
            args.y,
            args.z,
        ),
        repo_root=REPO_ROOT,
        initial_guess_rad=(
            tuple(args.initial_q)
            if args.initial_q is not None
            else None
        ),
        world_from_base_translation_m=(
            tuple(args.base_translation)
        ),
        require_nominal_strip_contact=(
            not args.allow_edge_contact
        ),
        tolerance_m=args.tolerance_mm / 1000.0,
        max_iterations=args.max_iterations,
    )

    print("=== MATDOG GENERIC CONTACT-REFERENCE IK — OFFLINE ===")
    print(f"leg: {result.leg_id.upper()}")
    print(
        "target_contact_reference_world_m: "
        f"({result.target_contact_reference_world_m[0]:+.9f}, "
        f"{result.target_contact_reference_world_m[1]:+.9f}, "
        f"{result.target_contact_reference_world_m[2]:+.9f})"
    )
    print("")

    print("joint_solution:")

    for joint_name in result.leg_contact.contract.joint_names:
        q_rad = result.joint_positions_rad[joint_name]
        print(
            f"  {joint_name}: "
            f"{q_rad:+.9f} rad "
            f"({math.degrees(q_rad):+.6f} deg)"
        )

    print("")
    print(
        "achieved_contact_reference_world_m: "
        f"({result.achieved_contact_reference_world_m[0]:+.9f}, "
        f"{result.achieved_contact_reference_world_m[1]:+.9f}, "
        f"{result.achieved_contact_reference_world_m[2]:+.9f})"
    )
    print(
        "lowest_core_world_m: "
        f"{result.leg_contact.contact.lowest_core_point_world_m}"
    )
    print(
        "support_mode: "
        f"{result.leg_contact.contact.support_mode}"
    )
    print(
        "residual_mm: "
        f"{result.residual_m * 1000.0:.6f}"
    )
    print(f"iterations: {result.iterations}")
    print("Offline only: no Station, serial or motor command was used.")


if __name__ == "__main__":
    try:
        main()
    except (
        QuadrupedContactIkError,
        QuadrupedLegContactError,
        ValueError,
    ) as exc:
        print(
            f"ERRORE GENERIC CONTACT IK: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(1)
