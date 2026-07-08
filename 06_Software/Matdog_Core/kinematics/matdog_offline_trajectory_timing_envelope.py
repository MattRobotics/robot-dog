#!/usr/bin/env python3
"""
MATDOG — C4-D offline trajectory timing and servo envelope validation.

This tool evaluates the C4-C contact-locked rest-to-stand trajectory against a
conservative timing envelope.

It computes, for every joint:
- total range;
- per-sample position step;
- segment speed;
- discrete acceleration.

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
import math
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
REPORTS_RELATIVE_DIR = Path(
    "09_Logs/Validation_Reports/C4_trajectory_timing_envelope"
)

DEFAULT_DURATION_S = 10.0
DEFAULT_MAX_SPEED_DEG_S = 10.0
DEFAULT_MAX_ACCEL_DEG_S2 = 25.0


class OfflineTrajectoryTimingEnvelopeError(RuntimeError):
    """Errore nella validazione offline C4-D timing/envelope."""


def _latest_c4c_report(repo_root: Path) -> Path:
    reports = sorted(
        (
            repo_root
            / "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory"
        ).glob("*_C4C_contact_locked_rest_to_stand_trajectory.json")
    )

    if not reports:
        raise OfflineTrajectoryTimingEnvelopeError(
            "Nessun report C4-C rest-to-stand trajectory trovato"
        )

    return reports[-1]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _default_report_path(repo_root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        repo_root
        / REPORTS_RELATIVE_DIR
        / f"{stamp}_C4D_trajectory_timing_envelope.json"
    )


def _joint_keys(frames: list[dict[str, Any]]) -> list[tuple[str, str]]:
    keys: list[tuple[str, str]] = []

    for leg_id in ("lf", "rf", "rh", "lh"):
        for joint_name in sorted(
            frames[0]["legs"][leg_id]["joint_positions_rad"]
        ):
            keys.append((leg_id, joint_name))

    return keys


def _rad_to_deg(value_rad: float) -> float:
    return math.degrees(value_rad)


def _analyze_joint(
    *,
    frames: list[dict[str, Any]],
    leg_id: str,
    joint_name: str,
    duration_s: float,
) -> dict[str, Any]:
    samples = len(frames)
    dt_s = duration_s / (samples - 1)

    values_rad = [
        float(frame["legs"][leg_id]["joint_positions_rad"][joint_name])
        for frame in frames
    ]
    values_deg = [_rad_to_deg(value) for value in values_rad]

    position_steps_deg = [
        values_deg[index + 1] - values_deg[index]
        for index in range(samples - 1)
    ]

    speeds_deg_s = [
        step / dt_s
        for step in position_steps_deg
    ]

    accelerations_deg_s2 = [
        (speeds_deg_s[index + 1] - speeds_deg_s[index]) / dt_s
        for index in range(len(speeds_deg_s) - 1)
    ]

    total_range_deg = max(values_deg) - min(values_deg)
    max_abs_step_deg = max(
        (abs(value) for value in position_steps_deg),
        default=0.0,
    )
    max_abs_speed_deg_s = max(
        (abs(value) for value in speeds_deg_s),
        default=0.0,
    )
    max_abs_accel_deg_s2 = max(
        (abs(value) for value in accelerations_deg_s2),
        default=0.0,
    )

    return {
        "leg_id": leg_id,
        "joint_name": joint_name,
        "start_deg": values_deg[0],
        "end_deg": values_deg[-1],
        "min_deg": min(values_deg),
        "max_deg": max(values_deg),
        "total_range_deg": total_range_deg,
        "max_abs_step_deg": max_abs_step_deg,
        "max_abs_speed_deg_s": max_abs_speed_deg_s,
        "max_abs_accel_deg_s2": max_abs_accel_deg_s2,
        "position_steps_deg": position_steps_deg,
        "speeds_deg_s": speeds_deg_s,
        "accelerations_deg_s2": accelerations_deg_s2,
    }


def _build_timing_report(
    *,
    repo_root: Path,
    c4c_report_path: Path,
    c4c: dict[str, Any],
    duration_s: float,
    max_speed_deg_s: float,
    max_accel_deg_s2: float,
) -> dict[str, Any]:
    if duration_s <= 0.0:
        raise OfflineTrajectoryTimingEnvelopeError(
            "duration_s deve essere > 0"
        )

    if max_speed_deg_s <= 0.0:
        raise OfflineTrajectoryTimingEnvelopeError(
            "max_speed_deg_s deve essere > 0"
        )

    if max_accel_deg_s2 <= 0.0:
        raise OfflineTrajectoryTimingEnvelopeError(
            "max_accel_deg_s2 deve essere > 0"
        )

    if c4c.get("status") not in {
        "OFFLINE_TRAJECTORY_VALID",
        "OFFLINE_TRAJECTORY_VALID_WITH_EXPECTED_FOOT_FORK_REVIEW",
    }:
        raise OfflineTrajectoryTimingEnvelopeError(
            f"C4-C status inatteso: {c4c.get('status')!r}"
        )

    frames = c4c["frames"]

    if len(frames) < 3:
        raise OfflineTrajectoryTimingEnvelopeError(
            "servono almeno 3 frame per stimare accelerazione discreta"
        )

    dt_s = duration_s / (len(frames) - 1)

    per_joint = [
        _analyze_joint(
            frames=frames,
            leg_id=leg_id,
            joint_name=joint_name,
            duration_s=duration_s,
        )
        for leg_id, joint_name in _joint_keys(frames)
    ]

    max_abs_speed_deg_s = max(
        item["max_abs_speed_deg_s"] for item in per_joint
    )
    max_abs_accel_deg_s2 = max(
        item["max_abs_accel_deg_s2"] for item in per_joint
    )
    max_abs_step_deg = max(
        item["max_abs_step_deg"] for item in per_joint
    )
    max_total_range_deg = max(
        item["total_range_deg"] for item in per_joint
    )

    speed_ok = max_abs_speed_deg_s <= max_speed_deg_s
    accel_ok = max_abs_accel_deg_s2 <= max_accel_deg_s2
    all_samples_safe = bool(c4c["metrics"]["all_samples_safe"])

    status = (
        "OFFLINE_TIMING_ENVELOPE_VALID"
        if speed_ok and accel_ok and all_samples_safe
        else "FAIL"
    )

    return {
        "schema": 1,
        "kind": "MATDOG_C4D_OFFLINE_TRAJECTORY_TIMING_ENVELOPE",
        "status": status,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "offline_only": True,
        "station_used": False,
        "serial_used": False,
        "motor_command_used": False,
        "source_reports": {
            "c4c_rest_to_stand_trajectory": str(c4c_report_path),
        },
        "timing_policy": {
            "duration_s": duration_s,
            "samples": len(frames),
            "dt_s": dt_s,
            "max_speed_deg_s": max_speed_deg_s,
            "max_accel_deg_s2": max_accel_deg_s2,
            "timing_source": (
                "conservative first-stand offline envelope; no hardware "
                "command generated"
            ),
        },
        "trajectory_geometry_status": {
            "c4c_status": c4c["status"],
            "all_c4c_samples_safe": all_samples_safe,
            "c4c_metrics": c4c["metrics"],
        },
        "metrics": {
            "max_total_range_deg": max_total_range_deg,
            "max_abs_step_deg": max_abs_step_deg,
            "max_abs_speed_deg_s": max_abs_speed_deg_s,
            "max_abs_accel_deg_s2": max_abs_accel_deg_s2,
            "speed_ok": speed_ok,
            "accel_ok": accel_ok,
        },
        "per_joint": per_joint,
        "command_eligibility": {
            "command_eligible": False,
            "reason": (
                "C4-D validates only the offline timing envelope. Hardware "
                "safe mode, operator approval and supervised execution checks "
                "are still required before any stand command."
            ),
        },
    }


def _print_summary(report: dict[str, Any], report_path: Path) -> None:
    metrics = report["metrics"]
    timing = report["timing_policy"]

    print("=== MATDOG C4-D OFFLINE TRAJECTORY TIMING ENVELOPE ===")
    print(f"duration_s: {timing['duration_s']:.3f}")
    print(f"samples: {timing['samples']}")
    print(f"dt_s: {timing['dt_s']:.6f}")
    print(f"status: {report['status']}")
    print("")
    print("ENVELOPE:")
    print(f"  max_allowed_speed_deg_s: {timing['max_speed_deg_s']:.3f}")
    print(f"  max_allowed_accel_deg_s2: {timing['max_accel_deg_s2']:.3f}")
    print("")
    print("MEASURED:")
    print(f"  max_total_range_deg: {metrics['max_total_range_deg']:.3f}")
    print(f"  max_abs_step_deg: {metrics['max_abs_step_deg']:.3f}")
    print(f"  max_abs_speed_deg_s: {metrics['max_abs_speed_deg_s']:.3f}")
    print(f"  max_abs_accel_deg_s2: {metrics['max_abs_accel_deg_s2']:.3f}")
    print(f"  speed_ok: {metrics['speed_ok']}")
    print(f"  accel_ok: {metrics['accel_ok']}")
    print("")

    print("PER JOINT:")
    for item in report["per_joint"]:
        print(
            f"  {item['joint_name']:20} "
            f"range={item['total_range_deg']:8.3f} deg "
            f"speed={item['max_abs_speed_deg_s']:8.3f} deg/s "
            f"accel={item['max_abs_accel_deg_s2']:8.3f} deg/s^2"
        )

    print("")
    print(f"report: {report_path}")
    print("COMMAND_ELIGIBLE: false")
    print("Offline only: no Station, serial or motor command was used.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MATDOG C4-D offline trajectory timing envelope."
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=DEFAULT_DURATION_S,
    )
    parser.add_argument(
        "--max-speed-deg-s",
        type=float,
        default=DEFAULT_MAX_SPEED_DEG_S,
    )
    parser.add_argument(
        "--max-accel-deg-s2",
        type=float,
        default=DEFAULT_MAX_ACCEL_DEG_S2,
    )
    parser.add_argument(
        "--trajectory-report",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    c4c_report_path = (
        args.trajectory_report
        if args.trajectory_report is not None
        else _latest_c4c_report(REPO_ROOT)
    )
    c4c = _load_json(c4c_report_path)

    report = _build_timing_report(
        repo_root=REPO_ROOT,
        c4c_report_path=c4c_report_path,
        c4c=c4c,
        duration_s=float(args.duration_s),
        max_speed_deg_s=float(args.max_speed_deg_s),
        max_accel_deg_s2=float(args.max_accel_deg_s2),
    )

    report_path = (
        args.report_path
        if args.report_path is not None
        else _default_report_path(REPO_ROOT)
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    _print_summary(report, report_path)

    if report["status"] == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
