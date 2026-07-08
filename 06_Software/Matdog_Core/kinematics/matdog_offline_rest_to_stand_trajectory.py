#!/usr/bin/env python3
"""
MATDOG — C4-C offline rest-to-stand trajectory candidate.

This tool generates an offline contact-locked IK trajectory from a low
contact-compatible stance to the C4-A safe stand candidate.

Important:
- this is not joint-space interpolation from q=0;
- q=0 / visual-zero is not a valid four-foot contact stance;
- feet remain locked to the C4-A contact references on world Z = 0;
- body stays parallel to the ground;
- IK is solved at every sampled body height;
- C4-B collision/contact policy is evaluated at every sample.

Offline only:
- no Station;
- no serial;
- no motor command;
- no torque, target, speed, accel, stand or gait command.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from matdog_offline_safe_stand_candidate import (
    GROUND_NORMAL_WORLD_UNIT,
    IDENTITY_MATRIX3,
)
from matdog_quadruped_leg_contact import LEG_IDS, leg_joint_names
from matdog_quadruped_leg_contact_ik import solve_leg_contact_reference_ik
from matdog_urdf_fk import canonical_urdf_path
from matdog_offline_collision_contact_policy import (
    _evaluate_knee_contact_clearance,
    _evaluate_link_ground_clearance,
    _overall_status,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
REPORTS_RELATIVE_DIR = Path(
    "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory"
)

DEFAULT_START_BODY_Z_M = 0.100
DEFAULT_SAMPLES = 51
IK_TOLERANCE_M = 1e-5
MAX_IK_ITERATIONS = 180
FOOT_CONTACT_Z_TOLERANCE_M = 1e-5


class OfflineRestToStandTrajectoryError(RuntimeError):
    """Errore nella generazione offline C4-C rest-to-stand."""


def _latest_c4a_report(repo_root: Path) -> Path:
    reports = sorted(
        (repo_root / "09_Logs/Validation_Reports").glob(
            "*_C4A_offline_safe_stand_candidate.json"
        )
    )

    if not reports:
        raise OfflineRestToStandTrajectoryError(
            "Nessun report C4-A offline safe stand trovato"
        )

    return reports[-1]


def _latest_c4b_report(repo_root: Path) -> Path:
    reports = sorted(
        (
            repo_root
            / "09_Logs/Validation_Reports/C4_collision_contact_policy"
        ).glob("*_C4B_collision_contact_policy.json")
    )

    if not reports:
        raise OfflineRestToStandTrajectoryError(
            "Nessun report C4-B collision/contact policy trovato"
        )

    return reports[-1]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _default_report_path(repo_root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        repo_root
        / REPORTS_RELATIVE_DIR
        / f"{stamp}_C4C_contact_locked_rest_to_stand_trajectory.json"
    )


def _linspace(start: float, stop: float, samples: int) -> list[float]:
    if samples < 2:
        raise OfflineRestToStandTrajectoryError("samples deve essere >= 2")

    return [
        start + (stop - start) * index / (samples - 1)
        for index in range(samples)
    ]


def _final_joint_guess_from_c4a(
    c4a: dict[str, Any],
) -> dict[str, tuple[float, float, float]]:
    guesses: dict[str, tuple[float, float, float]] = {}

    for leg_id in LEG_IDS:
        names = leg_joint_names(leg_id)
        guesses[leg_id] = tuple(
            float(c4a["legs"][leg_id]["joint_positions_rad"][name])
            for name in names
        )

    return guesses


def _solve_frame(
    *,
    repo_root: Path,
    c4a: dict[str, Any],
    body_z_m: float,
    previous_guess: dict[str, tuple[float, float, float]],
) -> dict[str, Any]:
    frame = {
        "body_pose": {
            "translation_world_m": [0.0, 0.0, body_z_m],
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "base_link_parallel_to_ground": True,
        },
        "legs": {},
    }

    residual_max_m = 0.0

    for leg_id in LEG_IDS:
        names = leg_joint_names(leg_id)
        target = tuple(
            float(value)
            for value in c4a["legs"][leg_id][
                "target_contact_reference_world_m"
            ]
        )

        result = solve_leg_contact_reference_ik(
            leg_id=leg_id,
            target_contact_reference_world_m=target,
            repo_root=repo_root,
            initial_guess_rad=previous_guess[leg_id],
            world_from_base_rotation=IDENTITY_MATRIX3,
            world_from_base_translation_m=(0.0, 0.0, body_z_m),
            ground_normal_world_unit=GROUND_NORMAL_WORLD_UNIT,
            require_nominal_strip_contact=True,
            tolerance_m=IK_TOLERANCE_M,
            max_iterations=MAX_IK_ITERATIONS,
        )

        q_tuple = tuple(
            float(result.joint_positions_rad[name])
            for name in names
        )
        previous_guess[leg_id] = q_tuple
        residual_max_m = max(residual_max_m, float(result.residual_m))

        frame["legs"][leg_id] = {
            "target_contact_reference_world_m": target,
            "achieved_contact_reference_world_m": (
                result.achieved_contact_reference_world_m
            ),
            "joint_positions_rad": result.joint_positions_rad,
            "joint_positions_deg": {
                name: result.joint_positions_rad[name] * 180.0 / 3.141592653589793
                for name in result.joint_positions_rad
            },
            "support_mode": result.leg_contact.contact.support_mode,
            "residual_m": float(result.residual_m),
            "iterations": int(result.iterations),
        }

    frame["ik_residual_max_m"] = residual_max_m
    return frame


def _evaluate_frame(
    *,
    repo_root: Path,
    urdf_path: Path,
    frame: dict[str, Any],
) -> dict[str, Any]:
    link_clearances = _evaluate_link_ground_clearance(
        repo_root=repo_root,
        urdf_path=urdf_path,
        candidate=frame,
    )
    knee_clearances = _evaluate_knee_contact_clearance(
        urdf_path=urdf_path,
        candidate=frame,
    )

    policy_status = _overall_status(link_clearances, knee_clearances)

    support_bad = [
        f"{leg_id}:{frame['legs'][leg_id]['support_mode']}"
        for leg_id in LEG_IDS
        if frame["legs"][leg_id]["support_mode"] != "NOMINAL_STRIP_CONTACT"
    ]

    if support_bad and policy_status != "FAIL":
        policy_status = "SUPPORT_MODE_FAIL"

    non_foot = [
        item for item in link_clearances
        if not item.is_foot_link
    ]
    lower_links = [
        item for item in link_clearances
        if item.is_lower_leg_link
    ]

    non_foot_min_z_m = min(item.min_z_m for item in non_foot)
    lower_leg_min_z_m = min(item.min_z_m for item in lower_links)
    knee_clearance_min_m = min(
        item.knee_contact_clearance_m for item in knee_clearances
    )

    foot_contact_abs_z_max_m = max(
        abs(frame["legs"][leg_id]["achieved_contact_reference_world_m"][2])
        for leg_id in LEG_IDS
    )

    safe = (
        policy_status in {"PASS", "PASS_WITH_EXPECTED_FOOT_FORK_REVIEW"}
        and not support_bad
        and foot_contact_abs_z_max_m <= FOOT_CONTACT_Z_TOLERANCE_M
    )

    return {
        "policy_status": policy_status,
        "safe": safe,
        "support_bad": support_bad,
        "non_foot_min_z_m": non_foot_min_z_m,
        "lower_leg_min_z_m": lower_leg_min_z_m,
        "knee_clearance_min_m": knee_clearance_min_m,
        "foot_contact_abs_z_max_m": foot_contact_abs_z_max_m,
        "link_ground_clearance": [
            {
                "link_name": item.link_name,
                "policy": item.policy,
                "min_z_m": item.min_z_m,
                "max_z_m": item.max_z_m,
                "is_foot_link": item.is_foot_link,
                "is_lower_leg_link": item.is_lower_leg_link,
            }
            for item in link_clearances
        ],
        "knee_contact_clearance": [
            {
                "leg_id": item.leg_id,
                "policy": item.policy,
                "lower_leg_joint_world_m": item.lower_leg_joint_world_m,
                "foot_contact_reference_world_m": (
                    item.foot_contact_reference_world_m
                ),
                "knee_contact_clearance_m": (
                    item.knee_contact_clearance_m
                ),
            }
            for item in knee_clearances
        ],
    }


def build_contact_locked_trajectory(
    *,
    repo_root: Path,
    c4a: dict[str, Any],
    start_body_z_m: float,
    samples: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    final_body_z_m = float(c4a["body_pose"]["translation_world_m"][2])

    if start_body_z_m <= 0.0:
        raise OfflineRestToStandTrajectoryError(
            "start_body_z_m deve essere > 0"
        )

    if start_body_z_m >= final_body_z_m:
        raise OfflineRestToStandTrajectoryError(
            "start_body_z_m deve essere minore della quota C4-A finale"
        )

    urdf_path = canonical_urdf_path(repo_root)

    # For solver robustness, solve from known C4-A final pose downward,
    # then reverse the solved frames to obtain the actual stand-up trajectory.
    descending_heights = _linspace(final_body_z_m, start_body_z_m, samples)
    previous_guess = _final_joint_guess_from_c4a(c4a)

    descending_frames: list[dict[str, Any]] = []

    for solver_index, body_z_m in enumerate(descending_heights):
        frame = _solve_frame(
            repo_root=repo_root,
            c4a=c4a,
            body_z_m=body_z_m,
            previous_guess=previous_guess,
        )
        evaluation = _evaluate_frame(
            repo_root=repo_root,
            urdf_path=urdf_path,
            frame=frame,
        )

        frame["solver_index_descending"] = solver_index
        frame["evaluation"] = evaluation
        descending_frames.append(frame)

    trajectory_frames = list(reversed(descending_frames))

    for trajectory_index, frame in enumerate(trajectory_frames):
        frame["trajectory_index"] = trajectory_index
        frame["phase"] = "contact_locked_body_height_ramp"

    metrics = _trajectory_metrics(trajectory_frames)

    return trajectory_frames, metrics


def _trajectory_metrics(
    frames: list[dict[str, Any]],
) -> dict[str, Any]:
    safe_samples = sum(
        1 for frame in frames if frame["evaluation"]["safe"]
    )

    statuses = [
        frame["evaluation"]["policy_status"]
        for frame in frames
    ]

    non_foot_min_z_m = min(
        frame["evaluation"]["non_foot_min_z_m"]
        for frame in frames
    )
    lower_leg_min_z_m = min(
        frame["evaluation"]["lower_leg_min_z_m"]
        for frame in frames
    )
    knee_clearance_min_m = min(
        frame["evaluation"]["knee_clearance_min_m"]
        for frame in frames
    )
    foot_contact_abs_z_max_m = max(
        frame["evaluation"]["foot_contact_abs_z_max_m"]
        for frame in frames
    )
    ik_residual_max_m = max(
        frame["ik_residual_max_m"]
        for frame in frames
    )

    all_safe = safe_samples == len(frames)

    return {
        "samples": len(frames),
        "safe_samples": safe_samples,
        "all_samples_safe": all_safe,
        "policy_statuses": sorted(set(statuses)),
        "non_foot_min_z_m": non_foot_min_z_m,
        "lower_leg_min_z_m": lower_leg_min_z_m,
        "knee_clearance_min_m": knee_clearance_min_m,
        "foot_contact_abs_z_max_m": foot_contact_abs_z_max_m,
        "ik_residual_max_m": ik_residual_max_m,
    }


def _overall_trajectory_status(metrics: dict[str, Any]) -> str:
    if not metrics["all_samples_safe"]:
        return "FAIL"

    if "PASS_WITH_EXPECTED_FOOT_FORK_REVIEW" in metrics["policy_statuses"]:
        return "OFFLINE_TRAJECTORY_VALID_WITH_EXPECTED_FOOT_FORK_REVIEW"

    return "OFFLINE_TRAJECTORY_VALID"


def _build_report(
    *,
    repo_root: Path,
    c4a_report_path: Path,
    c4b_report_path: Path,
    c4a: dict[str, Any],
    start_body_z_m: float,
    frames: list[dict[str, Any]],
    metrics: dict[str, Any],
    status: str,
) -> dict[str, Any]:
    final_body_z_m = float(c4a["body_pose"]["translation_world_m"][2])

    return {
        "schema": 1,
        "kind": "MATDOG_C4C_OFFLINE_CONTACT_LOCKED_REST_TO_STAND_TRAJECTORY",
        "status": status,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "offline_only": True,
        "station_used": False,
        "serial_used": False,
        "motor_command_used": False,
        "source_reports": {
            "c4a_safe_stand_candidate": str(c4a_report_path),
            "c4b_collision_contact_policy": str(c4b_report_path),
        },
        "trajectory_policy": {
            "type": "contact_locked_ik_body_height_ramp",
            "rejected_policy": "direct_joint_space_interpolation_from_q0",
            "reason": (
                "q=0 visual-zero is not a valid four-foot contact stance and "
                "direct interpolation was shown to create lower-leg ground "
                "penetration in C4-C1."
            ),
            "body_parallel_to_ground": True,
            "footprint_source": "C4-A target_contact_reference_world_m",
            "foot_contact_world_z_m": 0.0,
            "start_body_z_m": start_body_z_m,
            "final_body_z_m": final_body_z_m,
            "solver_order": (
                "solved descending from C4-A for IK robustness, then reversed "
                "as ascending rest-to-stand trajectory"
            ),
        },
        "ik_policy": {
            "solver": "solve_leg_contact_reference_ik",
            "required_support_mode": "NOMINAL_STRIP_CONTACT",
            "tolerance_m": IK_TOLERANCE_M,
            "max_iterations": MAX_IK_ITERATIONS,
        },
        "safety_policy": {
            "collision_contact_policy": "C4-B",
            "expected_low_clearance": (
                "distal lower-leg fork around rigid TPU 90D foot cylinder"
            ),
            "command_eligible": False,
        },
        "metrics": metrics,
        "frames": frames,
        "command_eligibility": {
            "command_eligible": False,
            "reason": (
                "C4-C is offline trajectory validation only. Dynamic stability, "
                "servo speed/accel limits, supervised hardware safe mode and "
                "operator approval are still required before any stand command."
            ),
        },
    }


def _print_summary(
    *,
    report_path: Path,
    status: str,
    metrics: dict[str, Any],
    start_body_z_m: float,
    final_body_z_m: float,
) -> None:
    print("=== MATDOG C4-C OFFLINE REST-TO-STAND TRAJECTORY ===")
    print("trajectory_type: contact_locked_ik_body_height_ramp")
    print("rejected: direct_joint_space_interpolation_from_q0")
    print(f"start_body_z_m: {start_body_z_m:+.6f}")
    print(f"final_body_z_m: {final_body_z_m:+.6f}")
    print(f"samples: {metrics['samples']}")
    print(f"safe_samples: {metrics['safe_samples']} / {metrics['samples']}")
    print(f"status: {status}")
    print("")
    print("WORST CASE:")
    print(
        "  non_foot_min_z_mm: "
        f"{metrics['non_foot_min_z_m'] * 1000.0:+.3f}"
    )
    print(
        "  lower_leg_min_z_mm: "
        f"{metrics['lower_leg_min_z_m'] * 1000.0:+.3f}"
    )
    print(
        "  knee_clearance_min_mm: "
        f"{metrics['knee_clearance_min_m'] * 1000.0:+.3f}"
    )
    print(
        "  foot_contact_abs_z_max_mm: "
        f"{metrics['foot_contact_abs_z_max_m'] * 1000.0:+.6f}"
    )
    print(
        "  ik_residual_max_mm: "
        f"{metrics['ik_residual_max_m'] * 1000.0:+.6f}"
    )
    print("")
    print(f"report: {report_path}")
    print("COMMAND_ELIGIBLE: false")
    print("Offline only: no Station, serial or motor command was used.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MATDOG C4-C offline rest-to-stand trajectory."
    )
    parser.add_argument(
        "--start-body-z-m",
        type=float,
        default=DEFAULT_START_BODY_Z_M,
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=DEFAULT_SAMPLES,
    )
    parser.add_argument(
        "--candidate-report",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--collision-contact-report",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    c4a_report_path = (
        args.candidate_report
        if args.candidate_report is not None
        else _latest_c4a_report(REPO_ROOT)
    )
    c4b_report_path = (
        args.collision_contact_report
        if args.collision_contact_report is not None
        else _latest_c4b_report(REPO_ROOT)
    )

    c4a = _load_json(c4a_report_path)
    c4b = _load_json(c4b_report_path)

    if c4a.get("status") != "OFFLINE_CANDIDATE_VALID":
        raise OfflineRestToStandTrajectoryError(
            f"C4-A status inatteso: {c4a.get('status')!r}"
        )

    if c4b.get("status") not in {
        "PASS",
        "PASS_WITH_EXPECTED_FOOT_FORK_REVIEW",
    }:
        raise OfflineRestToStandTrajectoryError(
            f"C4-B status non valido per C4-C: {c4b.get('status')!r}"
        )

    frames, metrics = build_contact_locked_trajectory(
        repo_root=REPO_ROOT,
        c4a=c4a,
        start_body_z_m=float(args.start_body_z_m),
        samples=int(args.samples),
    )

    status = _overall_trajectory_status(metrics)

    report_path = (
        args.report_path
        if args.report_path is not None
        else _default_report_path(REPO_ROOT)
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)

    report = _build_report(
        repo_root=REPO_ROOT,
        c4a_report_path=c4a_report_path,
        c4b_report_path=c4b_report_path,
        c4a=c4a,
        start_body_z_m=float(args.start_body_z_m),
        frames=frames,
        metrics=metrics,
        status=status,
    )

    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    final_body_z_m = float(c4a["body_pose"]["translation_world_m"][2])

    _print_summary(
        report_path=report_path,
        status=status,
        metrics=metrics,
        start_body_z_m=float(args.start_body_z_m),
        final_body_z_m=final_body_z_m,
    )

    if status == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
