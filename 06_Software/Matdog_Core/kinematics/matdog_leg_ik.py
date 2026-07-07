#!/usr/bin/env python3
"""
MATDOG — inverse kinematics offline della zampa LF.

Questo modulo:
- non apre Station;
- non apre porte seriali;
- non invia torque, target, speed o accel;
- usa il URDF canonico per calcolare FK e Jacobiano numerico;
- rispetta i limiti URDF dei tre joint LF.

Input:
    target posizione del piede LF [x, y, z] nel frame base_link.

Output:
    q URDF per lf_hip_joint, lf_upper_leg_joint, lf_lower_leg_joint,
    con verifica FK finale e residuo cartesiano.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys

from matdog_urdf_fk import (
    CANONICAL_URDF_SHA256,
    canonical_urdf_path,
    forward_kinematics,
    load_urdf_joints,
    sha256_file,
)


REPO_ROOT = Path(__file__).resolve().parents[3]

LF_ROOT_LINK = "base_link"
LF_TIP_LINK = "lf_foot_link"
LF_JOINT_NAMES = (
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
)


class IkError(RuntimeError):
    """Errore base per IK MATDOG."""


class IkUnreachableError(IkError):
    """Il target non converge entro limiti, iterazioni e tolleranza."""


@dataclass(frozen=True)
class IkResult:
    target_position_m: tuple[float, float, float]
    achieved_position_m: tuple[float, float, float]
    joint_positions_rad: dict[str, float]
    residual_m: float
    iterations: int


def _require_finite_vector(
    value: tuple[float, float, float],
    field_name: str,
) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{field_name}: attesi esattamente 3 valori")

    converted = tuple(float(component) for component in value)

    if not all(math.isfinite(component) for component in converted):
        raise ValueError(f"{field_name}: contiene valori non finiti")

    return converted


def _vector_subtract(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def _vector_norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _verify_canonical_urdf(repo_root: Path) -> Path:
    urdf_path = canonical_urdf_path(repo_root)

    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF canonico non trovato: {urdf_path}")

    actual_sha256 = sha256_file(urdf_path)

    if actual_sha256 != CANONICAL_URDF_SHA256:
        raise IkError(
            "Integrità URDF fallita: "
            f"sha256={actual_sha256}, atteso={CANONICAL_URDF_SHA256}"
        )

    return urdf_path


def lf_joint_limits_rad(
    urdf_path: Path,
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]:
    joints = load_urdf_joints(urdf_path)
    limits: list[tuple[float, float]] = []

    for joint_name in LF_JOINT_NAMES:
        joint = joints.get(joint_name)

        if joint is None:
            raise IkError(f"Joint LF assente nel URDF: {joint_name}")

        if joint.lower_limit_rad is None or joint.upper_limit_rad is None:
            raise IkError(f"{joint_name}: limiti URDF mancanti")

        limits.append((joint.lower_limit_rad, joint.upper_limit_rad))

    return tuple(limits)  # type: ignore[return-value]


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
    initial_guess_rad: tuple[float, float, float] | None,
    limits: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
) -> tuple[float, float, float]:
    if initial_guess_rad is None:
        return _clamp_joint_vector((0.0, 0.0, 0.0), limits)

    values = _require_finite_vector(initial_guess_rad, "initial_guess_rad")

    for joint_name, value, (lower, upper) in zip(
        LF_JOINT_NAMES,
        values,
        limits,
    ):
        if not lower <= value <= upper:
            raise ValueError(
                f"{joint_name}: initial guess {value:.9f} rad fuori dai "
                f"limiti [{lower:.9f}, {upper:.9f}]"
            )

    return values


def _lf_foot_position(
    urdf_path: Path,
    q_rad: tuple[float, float, float],
) -> tuple[float, float, float]:
    result = forward_kinematics(
        urdf_path=urdf_path,
        root_link=LF_ROOT_LINK,
        tip_link=LF_TIP_LINK,
        joint_positions_rad=dict(zip(LF_JOINT_NAMES, q_rad)),
        enforce_limits=True,
    )
    return result.tip_position_m


def _solve_3x3(
    matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    augmented = [
        [matrix[row][0], matrix[row][1], matrix[row][2], vector[row]]
        for row in range(3)
    ]

    for column in range(3):
        pivot_row = max(
            range(column, 3),
            key=lambda row: abs(augmented[row][column]),
        )

        if abs(augmented[pivot_row][column]) < 1e-14:
            raise IkError("Jacobiano IK singolare o numericamente degenerato")

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
                augmented[row][item] -= factor * augmented[column][item]

    return (
        augmented[0][3],
        augmented[1][3],
        augmented[2][3],
    )


def _numerical_jacobian(
    urdf_path: Path,
    q_rad: tuple[float, float, float],
    limits: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
    step_rad: float,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    columns: list[tuple[float, float, float]] = []

    for index in range(3):
        lower, upper = limits[index]

        plus_value = min(q_rad[index] + step_rad, upper)
        minus_value = max(q_rad[index] - step_rad, lower)
        denominator = plus_value - minus_value

        if denominator <= 1e-12:
            raise IkError(
                f"{LF_JOINT_NAMES[index]}: impossibile calcolare il Jacobiano "
                "al limite articolare"
            )

        plus_q = list(q_rad)
        minus_q = list(q_rad)
        plus_q[index] = plus_value
        minus_q[index] = minus_value

        plus_position = _lf_foot_position(urdf_path, tuple(plus_q))
        minus_position = _lf_foot_position(urdf_path, tuple(minus_q))

        columns.append(
            (
                (plus_position[0] - minus_position[0]) / denominator,
                (plus_position[1] - minus_position[1]) / denominator,
                (plus_position[2] - minus_position[2]) / denominator,
            )
        )

    return (
        (columns[0][0], columns[1][0], columns[2][0]),
        (columns[0][1], columns[1][1], columns[2][1]),
        (columns[0][2], columns[1][2], columns[2][2]),
    )


def _damped_least_squares_step(
    jacobian: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    cartesian_error: tuple[float, float, float],
    damping: float,
) -> tuple[float, float, float]:
    if damping <= 0.0:
        raise ValueError("damping deve essere > 0")

    # A = J^T J + lambda^2 I
    normal = []

    for row in range(3):
        normal_row = []

        for column in range(3):
            value = sum(
                jacobian[k][row] * jacobian[k][column]
                for k in range(3)
            )

            if row == column:
                value += damping * damping

            normal_row.append(value)

        normal.append(tuple(normal_row))

    # b = J^T e
    right_hand_side = tuple(
        sum(
            jacobian[k][column] * cartesian_error[k]
            for k in range(3)
        )
        for column in range(3)
    )

    return _solve_3x3(
        tuple(normal),  # type: ignore[arg-type]
        right_hand_side,
    )


def solve_lf_position_ik(
    repo_root: Path,
    target_position_m: tuple[float, float, float],
    initial_guess_rad: tuple[float, float, float] | None = None,
    tolerance_m: float = 1e-6,
    max_iterations: int = 250,
    damping: float = 1e-3,
    numerical_step_rad: float = 1e-5,
    maximum_step_rad: float = 0.12,
) -> IkResult:
    """
    Risolve IK posizionale LF usando DLS e il URDF canonico.

    Il risultato è valido soltanto se il residuo cartesiano è entro
    tolerance_m e tutti i joint restano nei limiti URDF.
    """
    target = _require_finite_vector(target_position_m, "target_position_m")

    if tolerance_m <= 0.0:
        raise ValueError("tolerance_m deve essere > 0")

    if max_iterations <= 0:
        raise ValueError("max_iterations deve essere > 0")

    if numerical_step_rad <= 0.0:
        raise ValueError("numerical_step_rad deve essere > 0")

    if maximum_step_rad <= 0.0:
        raise ValueError("maximum_step_rad deve essere > 0")

    urdf_path = _verify_canonical_urdf(repo_root)
    limits = lf_joint_limits_rad(urdf_path)
    current_q = _validate_initial_guess(initial_guess_rad, limits)

    for iteration in range(max_iterations + 1):
        current_position = _lf_foot_position(urdf_path, current_q)
        cartesian_error = _vector_subtract(target, current_position)
        residual = _vector_norm(cartesian_error)

        if residual <= tolerance_m:
            return IkResult(
                target_position_m=target,
                achieved_position_m=current_position,
                joint_positions_rad=dict(zip(LF_JOINT_NAMES, current_q)),
                residual_m=residual,
                iterations=iteration,
            )

        if iteration == max_iterations:
            break

        jacobian = _numerical_jacobian(
            urdf_path=urdf_path,
            q_rad=current_q,
            limits=limits,
            step_rad=numerical_step_rad,
        )

        delta_q = _damped_least_squares_step(
            jacobian=jacobian,
            cartesian_error=cartesian_error,
            damping=damping,
        )

        delta_norm = _vector_norm(delta_q)

        if delta_norm <= 1e-12:
            break

        if delta_norm > maximum_step_rad:
            scale = maximum_step_rad / delta_norm
            delta_q = tuple(component * scale for component in delta_q)

        accepted = False
        scale = 1.0

        for _ in range(12):
            candidate_q = _clamp_joint_vector(
                (
                    current_q[0] + scale * delta_q[0],
                    current_q[1] + scale * delta_q[1],
                    current_q[2] + scale * delta_q[2],
                ),
                limits,
            )

            candidate_position = _lf_foot_position(
                urdf_path,
                candidate_q,
            )

            candidate_residual = _vector_norm(
                _vector_subtract(target, candidate_position)
            )

            if candidate_residual < residual:
                current_q = candidate_q
                accepted = True
                break

            scale *= 0.5

        if not accepted:
            break

    final_position = _lf_foot_position(urdf_path, current_q)
    final_residual = _vector_norm(
        _vector_subtract(target, final_position)
    )

    raise IkUnreachableError(
        "IK LF non convergente entro i limiti URDF: "
        f"target={target}, residual={final_residual:.9f} m, "
        f"iterations={max_iterations}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG LF IK offline: target piede in base_link "
            "-> joint URDF -> FK di verifica."
        )
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
        help="Seed IK in radianti: hip upper lower.",
    )
    parser.add_argument("--tolerance-mm", type=float, default=0.001)
    parser.add_argument("--max-iterations", type=int, default=250)
    args = parser.parse_args()

    result = solve_lf_position_ik(
        repo_root=REPO_ROOT,
        target_position_m=(args.x, args.y, args.z),
        initial_guess_rad=(
            tuple(args.initial_q)
            if args.initial_q is not None
            else None
        ),
        tolerance_m=args.tolerance_mm / 1000.0,
        max_iterations=args.max_iterations,
    )

    print("=== MATDOG LF IK — OFFLINE ===")
    print(
        "Target foot position [base_link]: "
        f"X={result.target_position_m[0]:+.6f} m "
        f"Y={result.target_position_m[1]:+.6f} m "
        f"Z={result.target_position_m[2]:+.6f} m"
    )

    print("")
    print("Joint solution:")

    for joint_name in LF_JOINT_NAMES:
        q_rad = result.joint_positions_rad[joint_name]
        print(
            f"{joint_name}: "
            f"{q_rad:+.9f} rad "
            f"({math.degrees(q_rad):+.4f} deg)"
        )

    print("")
    print(
        "Achieved foot position [base_link]: "
        f"X={result.achieved_position_m[0]:+.6f} m "
        f"Y={result.achieved_position_m[1]:+.6f} m "
        f"Z={result.achieved_position_m[2]:+.6f} m"
    )
    print(f"Residual: {result.residual_m * 1000.0:.6f} mm")
    print(f"Iterations: {result.iterations}")
    print("Offline only: no Station, serial or motor command was used.")


if __name__ == "__main__":
    try:
        main()
    except IkError as exc:
        print(f"ERRORE IK: {exc}", file=sys.stderr)
        raise SystemExit(1)
