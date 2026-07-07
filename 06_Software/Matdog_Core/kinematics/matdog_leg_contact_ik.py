#!/usr/bin/env python3
"""
MATDOG — inverse kinematics offline sul punto reale di contatto LF.

Pipeline:
    target di contatto in world
    → IK numerica LF
    → FK canonica base_link → lf_foot_link
    → modello cilindrico del piede
    → riferimento continuo del contatto di rotolamento
    → lowest_core_point_world per la validazione del terreno
    → residuo cartesiano.

Questo modulo:
- non apre Station;
- non apre porte seriali;
- non invia comandi;
- usa URDF, FK e modello contatto già verificati.
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
from matdog_leg_contact import (
    LegContactError,
    LfLegContactResult,
    lf_foot_contact_from_joint_angles,
)
from matdog_leg_ik import (
    LF_JOINT_NAMES,
    lf_joint_limits_rad,
)
from matdog_urdf_fk import canonical_urdf_path


REPO_ROOT = Path(__file__).resolve().parents[3]

IDENTITY_MATRIX3: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class GroundContactIkError(RuntimeError):
    """Errore base della IK LF sul contatto reale."""


class GroundContactConstraintError(GroundContactIkError):
    """La soluzione non rispetta il vincolo di supporto richiesto."""


class GroundContactIkUnreachableError(GroundContactIkError):
    """Il target di contatto non converge entro limiti e tolleranza."""


@dataclass(frozen=True)
class LfGroundContactIkResult:
    target_contact_world_m: Vector3
    achieved_contact_world_m: Vector3
    joint_positions_rad: dict[str, float]
    residual_m: float
    iterations: int
    leg_contact: LfLegContactResult


def _vector3(value: tuple[float, float, float], field_name: str) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"{field_name}: attesi esattamente tre valori")

    result = tuple(float(component) for component in value)

    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{field_name}: contiene valori non finiti")

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
    initial_guess_rad: tuple[float, float, float] | None,
    limits: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
) -> tuple[float, float, float]:
    if initial_guess_rad is None:
        return _clamp_joint_vector((0.0, 0.0, 0.0), limits)

    values = _vector3(initial_guess_rad, "initial_guess_rad")

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
            raise GroundContactIkError(
                "Jacobiano contact-aware singolare o degenerato"
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
                augmented[row][item] -= factor * augmented[column][item]

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


def _contact_from_q(
    q_rad: tuple[float, float, float],
    repo_root: Path,
    world_from_base_rotation: Matrix3,
    world_from_base_translation_m: Vector3,
    ground_normal_world_unit: Vector3,
    require_nominal_strip_contact: bool,
) -> tuple[Vector3, LfLegContactResult]:
    joints = dict(zip(LF_JOINT_NAMES, q_rad))

    result = lf_foot_contact_from_joint_angles(
        joint_positions_rad=joints,
        repo_root=repo_root,
        world_from_base_rotation=world_from_base_rotation,
        world_from_base_translation_m=world_from_base_translation_m,
        ground_normal_world_unit=ground_normal_world_unit,
    )

    # Il target IK usa il centro della sezione cilindrica: è continuo
    # quando il piede attraversa la posa nominale e incorpora il raggio
    # reale del gommino. Il punto lowest_core_point resta disponibile nel
    # risultato per validazione del terreno e clearance conservativa.
    #
    # Il vincolo NOMINAL_STRIP_CONTACT viene verificato soltanto sulla
    # soluzione finale, non nei campioni ±epsilon del Jacobiano numerico.
    return result.contact.cross_section_contact_center_world_m, result


def _numerical_contact_jacobian(
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
    require_nominal_strip_contact: bool,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    columns: list[Vector3] = []

    for index in range(3):
        lower, upper = limits[index]

        plus_value = min(q_rad[index] + step_rad, upper)
        minus_value = max(q_rad[index] - step_rad, lower)
        denominator = plus_value - minus_value

        if denominator <= 1e-12:
            raise GroundContactIkError(
                f"{LF_JOINT_NAMES[index]}: Jacobiano non calcolabile "
                "al limite articolare"
            )

        plus_q = list(q_rad)
        minus_q = list(q_rad)
        plus_q[index] = plus_value
        minus_q[index] = minus_value

        plus_contact, _ = _contact_from_q(
            tuple(plus_q),
            repo_root,
            world_from_base_rotation,
            world_from_base_translation_m,
            ground_normal_world_unit,
            require_nominal_strip_contact,
        )

        minus_contact, _ = _contact_from_q(
            tuple(minus_q),
            repo_root,
            world_from_base_rotation,
            world_from_base_translation_m,
            ground_normal_world_unit,
            require_nominal_strip_contact,
        )

        columns.append(
            (
                (plus_contact[0] - minus_contact[0]) / denominator,
                (plus_contact[1] - minus_contact[1]) / denominator,
                (plus_contact[2] - minus_contact[2]) / denominator,
            )
        )

    return (
        (columns[0][0], columns[1][0], columns[2][0]),
        (columns[0][1], columns[1][1], columns[2][1]),
        (columns[0][2], columns[1][2], columns[2][2]),
    )


def solve_lf_ground_contact_ik(
    repo_root: Path,
    target_contact_world_m: tuple[float, float, float],
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
) -> LfGroundContactIkResult:
    """
    Risolve IK LF sul riferimento continuo del contatto cilindrico.

    Il riferimento IK è il centro della sezione circolare più bassa del
    gommino. Il punto lowest_core_point_world_m resta nel risultato per
    la futura validazione conservativa contro il piano terreno.

    Il risultato è valido soltanto se:
    - il residuo cartesiano è entro tolerance_m;
    - i joint rispettano i limiti URDF;
    - il support_mode rispetta il vincolo richiesto.
    """
    target = _vector3(
        target_contact_world_m,
        "target_contact_world_m",
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

    urdf_path = canonical_urdf_path(repo_root)
    limits = lf_joint_limits_rad(urdf_path)

    current_q = _validate_initial_guess(
        initial_guess_rad,
        limits,
    )

    for iteration in range(max_iterations + 1):
        current_contact, current_leg_contact = _contact_from_q(
            current_q,
            repo_root,
            world_from_base_rotation,
            world_from_base_translation_m,
            ground_normal_world_unit,
            require_nominal_strip_contact,
        )

        cartesian_error = _subtract(target, current_contact)
        residual = _norm(cartesian_error)

        if residual <= tolerance_m:
            if (
                require_nominal_strip_contact
                and current_leg_contact.contact.support_mode
                != "NOMINAL_STRIP_CONTACT"
            ):
                raise GroundContactConstraintError(
                    "La soluzione finale richiede contatto di bordo: "
                    f"{current_leg_contact.contact.support_mode}"
                )

            return LfGroundContactIkResult(
                target_contact_world_m=target,
                achieved_contact_world_m=current_contact,
                joint_positions_rad=dict(
                    zip(LF_JOINT_NAMES, current_q)
                ),
                residual_m=residual,
                iterations=iteration,
                leg_contact=current_leg_contact,
            )

        if iteration == max_iterations:
            break

        jacobian = _numerical_contact_jacobian(
            q_rad=current_q,
            limits=limits,
            step_rad=numerical_step_rad,
            repo_root=repo_root,
            world_from_base_rotation=world_from_base_rotation,
            world_from_base_translation_m=world_from_base_translation_m,
            ground_normal_world_unit=ground_normal_world_unit,
            require_nominal_strip_contact=require_nominal_strip_contact,
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

            try:
                candidate_contact, _ = _contact_from_q(
                    candidate_q,
                    repo_root,
                    world_from_base_rotation,
                    world_from_base_translation_m,
                    ground_normal_world_unit,
                    require_nominal_strip_contact,
                )
            except (
                FootContactDegeneracyError,
                GroundContactConstraintError,
            ):
                scale *= 0.5
                continue

            candidate_residual = _norm(
                _subtract(target, candidate_contact)
            )

            if candidate_residual < residual:
                current_q = candidate_q
                accepted = True
                break

            scale *= 0.5

        if not accepted:
            break

    try:
        final_contact, final_leg_contact = _contact_from_q(
            current_q,
            repo_root,
            world_from_base_rotation,
            world_from_base_translation_m,
            ground_normal_world_unit,
            require_nominal_strip_contact,
        )
        final_residual = _norm(
            _subtract(target, final_contact)
        )
        final_mode = final_leg_contact.contact.support_mode
    except (
        FootContactDegeneracyError,
        GroundContactConstraintError,
    ) as exc:
        raise GroundContactIkUnreachableError(
            "IK LF contact-aware terminata fuori dal dominio valido: "
            f"{exc}"
        ) from exc

    raise GroundContactIkUnreachableError(
        "IK LF contact-aware non convergente entro limiti URDF: "
        f"target={target}, residual={final_residual:.9f} m, "
        f"support_mode={final_mode}, iterations={max_iterations}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MATDOG LF contact-aware IK offline: target contact world "
            "→ q LF → FK + contact verification."
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
    )
    parser.add_argument(
        "--base-translation",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help="Traslazione world ← base_link in metri.",
    )
    parser.add_argument(
        "--allow-edge-contact",
        action="store_true",
        help=(
            "Permette EDGE_BIASED_CONTACT. "
            "Il default richiede NOMINAL_STRIP_CONTACT."
        ),
    )
    parser.add_argument("--tolerance-mm", type=float, default=0.001)
    parser.add_argument("--max-iterations", type=int, default=250)
    args = parser.parse_args()

    result = solve_lf_ground_contact_ik(
        repo_root=REPO_ROOT,
        target_contact_world_m=(args.x, args.y, args.z),
        initial_guess_rad=(
            tuple(args.initial_q)
            if args.initial_q is not None
            else None
        ),
        world_from_base_translation_m=tuple(args.base_translation),
        require_nominal_strip_contact=not args.allow_edge_contact,
        tolerance_m=args.tolerance_mm / 1000.0,
        max_iterations=args.max_iterations,
    )

    print("=== MATDOG LF CONTACT-AWARE IK — OFFLINE ===")
    print(
        "Target contact [world]: "
        f"X={result.target_contact_world_m[0]:+.6f} m "
        f"Y={result.target_contact_world_m[1]:+.6f} m "
        f"Z={result.target_contact_world_m[2]:+.6f} m"
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
        "Achieved contact [world]: "
        f"X={result.achieved_contact_world_m[0]:+.6f} m "
        f"Y={result.achieved_contact_world_m[1]:+.6f} m "
        f"Z={result.achieved_contact_world_m[2]:+.6f} m"
    )
    print(
        "Support mode: "
        f"{result.leg_contact.contact.support_mode}"
    )
    print(f"Residual: {result.residual_m * 1000.0:.6f} mm")
    print(f"Iterations: {result.iterations}")
    print("Offline only: no Station, serial or motor command was used.")


if __name__ == "__main__":
    try:
        main()
    except (
        GroundContactIkError,
        LegContactError,
        ValueError,
    ) as exc:
        print(f"ERRORE CONTACT IK: {exc}", file=sys.stderr)
        raise SystemExit(1)
