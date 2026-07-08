#!/usr/bin/env python3
"""
MATDOG — C4-A offline safe stand candidate.

This tool proposes a first offline four-leg stand candidate:

    body pose in world
    + reachable FK-derived foot contact references on world Z = 0
    + contact-reference IK per leg
    + URDF limit verification
    + NOMINAL_STRIP_CONTACT policy verification
    + offline report

It does not open Station, serial ports, or motor drivers.
It never sends torque, speed, accel, target, stand or gait commands.

C4-A result is an OFFLINE POSE CANDIDATE only.
It is intentionally NOT command-eligible until later gates exist:
self-collision, ground-collision, static stability, trajectory sampling,
safe hardware mode and explicit operator approval.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from typing import Any

import matdog_quadruped_leg_contact as qlc
import matdog_quadruped_leg_contact_ik as qik
from matdog_body_stance import audit_body_stance_geometry
from matdog_foot_contact import Matrix3, Vector3
from matdog_quadruped_leg_contact import (
    LEG_IDS,
    QuadrupedLegContactError,
    leg_foot_contact_from_joint_angles,
    leg_joint_names,
)
from matdog_quadruped_leg_contact_ik import (
    QuadrupedContactIkError,
    solve_leg_contact_reference_ik,
)


REPO_ROOT = Path(__file__).resolve().parents[3]

REPORTS_RELATIVE_DIR = Path("09_Logs/Validation_Reports")

IDENTITY_MATRIX3: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

GROUND_NORMAL_WORLD_UNIT: Vector3 = (0.0, 0.0, 1.0)

DEFAULT_BODY_HEIGHT_M = 0.150
DEFAULT_TARGET_CONTACT_Z_M = 0.0
DEFAULT_GRID_STEP_DEG = 4.0
DEFAULT_IK_TOLERANCE_M = 1e-5


class OfflineSafeStandCandidateError(RuntimeError):
    """Errore della generazione offline C4-A safe stand candidate."""


@dataclass(frozen=True)
class FkSeedCandidate:
    leg_id: str
    target_contact_reference_world_m: Vector3
    initial_guess_rad: tuple[float, float, float]
    initial_contact_base_m: Vector3
    initial_world_z_error_m: float
    support_mode: str


@dataclass(frozen=True)
class LegStandCandidate:
    leg_id: str
    target_contact_reference_world_m: Vector3
    achieved_contact_reference_world_m: Vector3
    joint_positions_rad: dict[str, float]
    joint_positions_deg: dict[str, float]
    urdf_limit_margin_rad: dict[str, dict[str, float]]
    support_mode: str
    residual_m: float
    iterations: int
    seed_contact_base_m: Vector3
    seed_world_z_error_m: float


@dataclass(frozen=True)
class OfflineSafeStandCandidate:
    body_translation_world_m: Vector3
    body_rotation_world_from_base: Matrix3
    target_contact_z_world_m: float
    leg_candidates: tuple[
        LegStandCandidate,
        LegStandCandidate,
        LegStandCandidate,
        LegStandCandidate,
    ]
    command_eligible: bool
    command_eligibility_reason: str


_CONTRACT_CACHE: dict[tuple[str, str], qlc.QuadrupedLegKinematicContract] = {}


def _enable_leg_contract_cache() -> None:
    original_loader = qlc.load_leg_kinematic_contract

    def cached_loader(
        leg_id: str,
        repo_root: Path = REPO_ROOT,
    ) -> qlc.QuadrupedLegKinematicContract:
        key = (leg_id, str(Path(repo_root).resolve()))

        if key not in _CONTRACT_CACHE:
            _CONTRACT_CACHE[key] = original_loader(leg_id, repo_root)

        return _CONTRACT_CACHE[key]

    qlc.load_leg_kinematic_contract = cached_loader
    qik.load_leg_kinematic_contract = cached_loader


def _finite_positive(value: float, field_name: str) -> float:
    result = float(value)

    if not math.isfinite(result) or result <= 0.0:
        raise OfflineSafeStandCandidateError(
            f"{field_name}: atteso valore finito > 0"
        )

    return result


def _frange_inclusive(
    lower: float,
    upper: float,
    step: float,
) -> list[float]:
    values: list[float] = []
    current = lower

    while current <= upper + 1e-12:
        values.append(current)
        current += step

    return values


def _find_fk_seed_for_leg(
    leg_id: str,
    repo_root: Path,
    body_height_m: float,
    target_contact_z_world_m: float,
    grid_step_rad: float,
) -> FkSeedCandidate:
    """
    Find a reachable nominal FK seed whose contact lies close to world Z = 0
    once the body is placed at body_height_m.

    This deliberately avoids using visual-zero XY as the commanded footprint.
    """
    contract = qlc.load_leg_kinematic_contract(leg_id, repo_root)
    joint_names = contract.joint_names
    limits = contract.joint_limits_rad

    hip_q = 0.0

    if not limits[0][0] <= hip_q <= limits[0][1]:
        raise OfflineSafeStandCandidateError(
            f"{leg_id}: hip=0 fuori dai limiti URDF"
        )

    upper_values = _frange_inclusive(
        limits[1][0],
        limits[1][1],
        grid_step_rad,
    )
    lower_values = _frange_inclusive(
        limits[2][0],
        limits[2][1],
        grid_step_rad,
    )

    best: tuple[
        float,
        float,
        Vector3,
        tuple[float, float, float],
        str,
    ] | None = None

    for upper_q in upper_values:
        for lower_q in lower_values:
            q = {
                joint_names[0]: hip_q,
                joint_names[1]: upper_q,
                joint_names[2]: lower_q,
            }

            result = leg_foot_contact_from_joint_angles(
                leg_id=leg_id,
                joint_positions_rad=q,
                repo_root=repo_root,
                world_from_base_rotation=IDENTITY_MATRIX3,
                world_from_base_translation_m=(0.0, 0.0, 0.0),
                ground_normal_world_unit=GROUND_NORMAL_WORLD_UNIT,
            )

            support_mode = result.contact.support_mode

            if support_mode != "NOMINAL_STRIP_CONTACT":
                continue

            contact_base = result.contact.cross_section_contact_center_world_m
            world_z = contact_base[2] + body_height_m
            z_error = abs(world_z - target_contact_z_world_m)

            # Secondary score keeps the first reference stance compact,
            # preferring a contact close to the hip vertical projection.
            compactness = abs(contact_base[0])

            score = z_error + compactness * 1e-4

            if best is None or score < best[0]:
                best = (
                    score,
                    z_error,
                    contact_base,
                    (hip_q, upper_q, lower_q),
                    support_mode,
                )

    if best is None:
        raise OfflineSafeStandCandidateError(
            f"{leg_id}: nessun seed FK NOMINAL_STRIP_CONTACT trovato"
        )

    _, z_error, contact_base, initial_guess, support_mode = best

    return FkSeedCandidate(
        leg_id=leg_id,
        target_contact_reference_world_m=(
            contact_base[0],
            contact_base[1],
            target_contact_z_world_m,
        ),
        initial_guess_rad=initial_guess,
        initial_contact_base_m=contact_base,
        initial_world_z_error_m=z_error,
        support_mode=support_mode,
    )


def _joint_limit_margins_rad(
    joint_positions_rad: dict[str, float],
    joint_limits_rad: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ],
    joint_names: tuple[str, str, str],
) -> dict[str, dict[str, float]]:
    margins: dict[str, dict[str, float]] = {}

    for joint_name, (lower, upper) in zip(joint_names, joint_limits_rad):
        value = joint_positions_rad[joint_name]

        if not lower <= value <= upper:
            raise OfflineSafeStandCandidateError(
                f"{joint_name}: soluzione fuori limiti URDF: "
                f"{value:.9f} rad not in [{lower:.9f}, {upper:.9f}]"
            )

        margins[joint_name] = {
            "lower_rad": lower,
            "value_rad": value,
            "upper_rad": upper,
            "margin_to_lower_rad": value - lower,
            "margin_to_upper_rad": upper - value,
        }

    return margins


def build_offline_safe_stand_candidate(
    repo_root: Path = REPO_ROOT,
    body_height_m: float = DEFAULT_BODY_HEIGHT_M,
    target_contact_z_world_m: float = DEFAULT_TARGET_CONTACT_Z_M,
    grid_step_deg: float = DEFAULT_GRID_STEP_DEG,
    tolerance_m: float = DEFAULT_IK_TOLERANCE_M,
    max_iterations: int = 160,
) -> OfflineSafeStandCandidate:
    body_height_m = _finite_positive(body_height_m, "body_height_m")
    grid_step_deg = _finite_positive(grid_step_deg, "grid_step_deg")
    tolerance_m = _finite_positive(tolerance_m, "tolerance_m")

    if max_iterations <= 0:
        raise OfflineSafeStandCandidateError(
            "max_iterations deve essere > 0"
        )

    _enable_leg_contract_cache()

    # Validate the existing body/stance geometry contract before using it.
    audit_body_stance_geometry(repo_root)

    grid_step_rad = math.radians(grid_step_deg)

    body_translation_world_m: Vector3 = (
        0.0,
        0.0,
        body_height_m,
    )

    leg_candidates: list[LegStandCandidate] = []

    for leg_id in LEG_IDS:
        seed = _find_fk_seed_for_leg(
            leg_id=leg_id,
            repo_root=repo_root,
            body_height_m=body_height_m,
            target_contact_z_world_m=target_contact_z_world_m,
            grid_step_rad=grid_step_rad,
        )

        result = solve_leg_contact_reference_ik(
            leg_id=leg_id,
            target_contact_reference_world_m=(
                seed.target_contact_reference_world_m
            ),
            repo_root=repo_root,
            initial_guess_rad=seed.initial_guess_rad,
            world_from_base_rotation=IDENTITY_MATRIX3,
            world_from_base_translation_m=body_translation_world_m,
            ground_normal_world_unit=GROUND_NORMAL_WORLD_UNIT,
            require_nominal_strip_contact=True,
            tolerance_m=tolerance_m,
            max_iterations=max_iterations,
        )

        support_mode = result.leg_contact.contact.support_mode

        if support_mode != "NOMINAL_STRIP_CONTACT":
            raise OfflineSafeStandCandidateError(
                f"{leg_id}: support_mode inatteso: {support_mode}"
            )

        contract = result.leg_contact.contract

        margins = _joint_limit_margins_rad(
            joint_positions_rad=result.joint_positions_rad,
            joint_limits_rad=contract.joint_limits_rad,
            joint_names=contract.joint_names,
        )

        leg_candidates.append(
            LegStandCandidate(
                leg_id=leg_id,
                target_contact_reference_world_m=(
                    seed.target_contact_reference_world_m
                ),
                achieved_contact_reference_world_m=(
                    result.achieved_contact_reference_world_m
                ),
                joint_positions_rad=result.joint_positions_rad,
                joint_positions_deg={
                    name: math.degrees(value)
                    for name, value in result.joint_positions_rad.items()
                },
                urdf_limit_margin_rad=margins,
                support_mode=support_mode,
                residual_m=result.residual_m,
                iterations=result.iterations,
                seed_contact_base_m=seed.initial_contact_base_m,
                seed_world_z_error_m=seed.initial_world_z_error_m,
            )
        )

    return OfflineSafeStandCandidate(
        body_translation_world_m=body_translation_world_m,
        body_rotation_world_from_base=IDENTITY_MATRIX3,
        target_contact_z_world_m=target_contact_z_world_m,
        leg_candidates=tuple(leg_candidates),  # type: ignore[arg-type]
        command_eligible=False,
        command_eligibility_reason=(
            "C4-A is offline only. Self-collision, ground-collision, "
            "static stability, trajectory sampling, hardware safe mode and "
            "operator approval gates are not implemented yet."
        ),
    )


def _candidate_to_report_dict(
    candidate: OfflineSafeStandCandidate,
    repo_root: Path,
    tolerance_m: float,
    grid_step_deg: float,
) -> dict[str, Any]:
    legs: dict[str, Any] = {}

    for leg in candidate.leg_candidates:
        legs[leg.leg_id] = {
            "target_contact_reference_world_m": (
                leg.target_contact_reference_world_m
            ),
            "achieved_contact_reference_world_m": (
                leg.achieved_contact_reference_world_m
            ),
            "joint_positions_rad": leg.joint_positions_rad,
            "joint_positions_deg": leg.joint_positions_deg,
            "urdf_limit_margin_rad": leg.urdf_limit_margin_rad,
            "support_mode": leg.support_mode,
            "residual_m": leg.residual_m,
            "residual_mm": leg.residual_m * 1000.0,
            "iterations": leg.iterations,
            "fk_seed": {
                "contact_base_m": leg.seed_contact_base_m,
                "world_z_error_m": leg.seed_world_z_error_m,
                "world_z_error_mm": leg.seed_world_z_error_m * 1000.0,
            },
        }

    return {
        "schema": 1,
        "kind": "MATDOG_C4A_OFFLINE_SAFE_STAND_CANDIDATE",
        "status": "OFFLINE_CANDIDATE_VALID",
        "repo_root": str(repo_root),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "offline_only": True,
        "station_used": False,
        "serial_used": False,
        "motor_command_used": False,
        "body_pose": {
            "translation_world_m": candidate.body_translation_world_m,
            "rotation_world_from_base": (
                candidate.body_rotation_world_from_base
            ),
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "base_link_parallel_to_ground": True,
        },
        "ground_plane": {
            "world_z_m": candidate.target_contact_z_world_m,
            "normal_world_unit": GROUND_NORMAL_WORLD_UNIT,
        },
        "footprint_policy": {
            "source": "reachable FK seed per leg, not visual-zero XY shortcut",
            "grid_step_deg": grid_step_deg,
        },
        "ik_policy": {
            "solver": "solve_leg_contact_reference_ik",
            "required_support_mode": "NOMINAL_STRIP_CONTACT",
            "tolerance_m": tolerance_m,
        },
        "legs": legs,
        "command_eligibility": {
            "command_eligible": candidate.command_eligible,
            "reason": candidate.command_eligibility_reason,
        },
    }


def _default_report_path(repo_root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        repo_root
        / REPORTS_RELATIVE_DIR
        / f"{stamp}_C4A_offline_safe_stand_candidate.json"
    )


def write_report(
    report: dict[str, Any],
    report_path: Path,
) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _print_candidate(
    candidate: OfflineSafeStandCandidate,
    report_path: Path,
) -> None:
    print("=== MATDOG C4-A OFFLINE SAFE STAND CANDIDATE ===")
    print(
        "body_translation_world_m: "
        f"({candidate.body_translation_world_m[0]:+.9f}, "
        f"{candidate.body_translation_world_m[1]:+.9f}, "
        f"{candidate.body_translation_world_m[2]:+.9f})"
    )
    print("body_roll_pitch_yaw_rad: (+0.000000000, +0.000000000, +0.000000000)")
    print(f"target_contact_world_z_m: {candidate.target_contact_z_world_m:+.9f}")
    print("footprint_policy: reachable FK seed per leg, not visual-zero XY shortcut")
    print("required_support_mode: NOMINAL_STRIP_CONTACT")
    print("")

    for leg in candidate.leg_candidates:
        print(f"{leg.leg_id.upper()}:")

        print(
            "  fk_seed_contact_base_m: "
            f"({leg.seed_contact_base_m[0]:+.9f}, "
            f"{leg.seed_contact_base_m[1]:+.9f}, "
            f"{leg.seed_contact_base_m[2]:+.9f})"
        )
        print(
            "  fk_seed_world_z_error_mm: "
            f"{leg.seed_world_z_error_m * 1000.0:.6f}"
        )
        print(
            "  target_contact_reference_world_m: "
            f"({leg.target_contact_reference_world_m[0]:+.9f}, "
            f"{leg.target_contact_reference_world_m[1]:+.9f}, "
            f"{leg.target_contact_reference_world_m[2]:+.9f})"
        )
        print(
            "  achieved_contact_reference_world_m: "
            f"({leg.achieved_contact_reference_world_m[0]:+.9f}, "
            f"{leg.achieved_contact_reference_world_m[1]:+.9f}, "
            f"{leg.achieved_contact_reference_world_m[2]:+.9f})"
        )

        for joint_name in leg_joint_names(leg.leg_id):
            q_rad = leg.joint_positions_rad[joint_name]
            q_deg = leg.joint_positions_deg[joint_name]
            margins = leg.urdf_limit_margin_rad[joint_name]

            print(
                f"  {joint_name}: "
                f"{q_rad:+.9f} rad "
                f"({q_deg:+.6f} deg), "
                "URDF margins deg: "
                f"lower={math.degrees(margins['margin_to_lower_rad']):.3f}, "
                f"upper={math.degrees(margins['margin_to_upper_rad']):.3f}"
            )

        print(f"  support_mode: {leg.support_mode}")
        print(f"  residual_mm: {leg.residual_m * 1000.0:.6f}")
        print(f"  iterations: {leg.iterations}")
        print("")

    print("COMMAND_ELIGIBLE: false")
    print(f"reason: {candidate.command_eligibility_reason}")
    print(f"report: {report_path}")
    print("Offline only: no Station, serial or motor command was used.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MATDOG C4-A offline safe stand candidate."
    )
    parser.add_argument(
        "--body-height-m",
        type=float,
        default=DEFAULT_BODY_HEIGHT_M,
    )
    parser.add_argument(
        "--target-contact-z-m",
        type=float,
        default=DEFAULT_TARGET_CONTACT_Z_M,
    )
    parser.add_argument(
        "--grid-step-deg",
        type=float,
        default=DEFAULT_GRID_STEP_DEG,
    )
    parser.add_argument(
        "--tolerance-mm",
        type=float,
        default=0.010,
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=160,
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    tolerance_m = args.tolerance_mm / 1000.0

    candidate = build_offline_safe_stand_candidate(
        repo_root=REPO_ROOT,
        body_height_m=args.body_height_m,
        target_contact_z_world_m=args.target_contact_z_m,
        grid_step_deg=args.grid_step_deg,
        tolerance_m=tolerance_m,
        max_iterations=args.max_iterations,
    )

    report = _candidate_to_report_dict(
        candidate=candidate,
        repo_root=REPO_ROOT,
        tolerance_m=tolerance_m,
        grid_step_deg=args.grid_step_deg,
    )

    report_path = (
        args.report_path
        if args.report_path is not None
        else _default_report_path(REPO_ROOT)
    )

    write_report(report, report_path)
    _print_candidate(candidate, report_path)


if __name__ == "__main__":
    try:
        main()
    except (
        OfflineSafeStandCandidateError,
        QuadrupedContactIkError,
        QuadrupedLegContactError,
        ValueError,
    ) as exc:
        print(
            f"ERRORE C4-A OFFLINE SAFE STAND CANDIDATE: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(1)
