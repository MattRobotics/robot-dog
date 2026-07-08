#!/usr/bin/env python3
"""
MATDOG — C4-E offline static stability / support-polygon validation.

This tool validates the C4-C contact-locked rest-to-stand trajectory against a
conservative static support-polygon check.

Assumption for C4-E:
- the projection of base_link origin is used as a first COM proxy;
- an uncertainty box ±20 mm in X/Y around that proxy is also tested;
- all tested points must remain inside the four-foot support polygon.

This does not replace a future CAD-derived center-of-mass model. It is an
offline gate before supervised hardware testing.

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
    "09_Logs/Validation_Reports/C4_static_stability_support_polygon"
)

DEFAULT_COM_PROXY_X_M = 0.0
DEFAULT_COM_PROXY_Y_M = 0.0
DEFAULT_COM_UNCERTAINTY_XY_M = 0.020
DEFAULT_MIN_SUPPORT_MARGIN_M = 0.0


class OfflineStaticStabilityError(RuntimeError):
    """Errore nella validazione offline C4-E static stability."""


Point2 = tuple[float, float]


def _latest_c4c_report(repo_root: Path) -> Path:
    reports = sorted(
        (
            repo_root
            / "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory"
        ).glob("*_C4C_contact_locked_rest_to_stand_trajectory.json")
    )

    if not reports:
        raise OfflineStaticStabilityError(
            "Nessun report C4-C rest-to-stand trajectory trovato"
        )

    return reports[-1]


def _latest_c4d_report(repo_root: Path) -> Path:
    reports = sorted(
        (
            repo_root
            / "09_Logs/Validation_Reports/C4_trajectory_timing_envelope"
        ).glob("*_C4D_trajectory_timing_envelope.json")
    )

    if not reports:
        raise OfflineStaticStabilityError(
            "Nessun report C4-D trajectory timing envelope trovato"
        )

    return reports[-1]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _default_report_path(repo_root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        repo_root
        / REPORTS_RELATIVE_DIR
        / f"{stamp}_C4E_static_stability_support_polygon.json"
    )


def _cross(origin: Point2, a: Point2, b: Point2) -> float:
    return (
        (a[0] - origin[0]) * (b[1] - origin[1])
        - (a[1] - origin[1]) * (b[0] - origin[0])
    )


def _convex_hull(points: list[Point2]) -> list[Point2]:
    unique_points = sorted(set(points))

    if len(unique_points) <= 1:
        return unique_points

    lower: list[Point2] = []

    for point in unique_points:
        while (
            len(lower) >= 2
            and _cross(lower[-2], lower[-1], point) <= 0.0
        ):
            lower.pop()

        lower.append(point)

    upper: list[Point2] = []

    for point in reversed(unique_points):
        while (
            len(upper) >= 2
            and _cross(upper[-2], upper[-1], point) <= 0.0
        ):
            upper.pop()

        upper.append(point)

    return lower[:-1] + upper[:-1]


def _point_in_convex_polygon(
    point: Point2,
    polygon: list[Point2],
    tolerance: float = 1e-12,
) -> bool:
    if len(polygon) < 3:
        return False

    signs: list[int] = []

    for index in range(len(polygon)):
        a = polygon[index]
        b = polygon[(index + 1) % len(polygon)]
        value = _cross(a, b, point)

        if abs(value) <= tolerance:
            signs.append(0)
        else:
            signs.append(1 if value > 0.0 else -1)

    nonzero = [sign for sign in signs if sign != 0]

    if not nonzero:
        return True

    return all(sign == nonzero[0] for sign in nonzero)


def _distance_point_to_segment(
    point: Point2,
    a: Point2,
    b: Point2,
) -> float:
    px, py = point
    ax, ay = a
    bx, by = b

    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay

    length_squared = vx * vx + vy * vy

    if length_squared == 0.0:
        return math.hypot(px - ax, py - ay)

    t = max(
        0.0,
        min(1.0, (wx * vx + wy * vy) / length_squared),
    )

    closest_x = ax + t * vx
    closest_y = ay + t * vy

    return math.hypot(px - closest_x, py - closest_y)


def _polygon_signed_margin(point: Point2, polygon: list[Point2]) -> float:
    if len(polygon) < 3:
        return -math.inf

    distances = [
        _distance_point_to_segment(
            point,
            polygon[index],
            polygon[(index + 1) % len(polygon)],
        )
        for index in range(len(polygon))
    ]

    margin = min(distances)

    if _point_in_convex_polygon(point, polygon):
        return margin

    return -margin


def _test_points(
    com_proxy_world_xy_m: Point2,
    com_uncertainty_xy_m: float,
) -> list[dict[str, Any]]:
    x, y = com_proxy_world_xy_m
    u = com_uncertainty_xy_m

    return [
        {
            "name": "com_proxy_center",
            "world_xy_m": (x, y),
        },
        {
            "name": "com_proxy_plus_x_plus_y",
            "world_xy_m": (x + u, y + u),
        },
        {
            "name": "com_proxy_plus_x_minus_y",
            "world_xy_m": (x + u, y - u),
        },
        {
            "name": "com_proxy_minus_x_plus_y",
            "world_xy_m": (x - u, y + u),
        },
        {
            "name": "com_proxy_minus_x_minus_y",
            "world_xy_m": (x - u, y - u),
        },
    ]


def _frame_contacts_xy(frame: dict[str, Any]) -> dict[str, Point2]:
    contacts: dict[str, Point2] = {}

    for leg_id in ("lf", "rf", "rh", "lh"):
        point = frame["legs"][leg_id][
            "achieved_contact_reference_world_m"
        ]
        contacts[leg_id] = (float(point[0]), float(point[1]))

    return contacts


def _evaluate_frame(
    *,
    frame: dict[str, Any],
    com_proxy_world_xy_m: Point2,
    com_uncertainty_xy_m: float,
    min_support_margin_m: float,
) -> dict[str, Any]:
    contacts_by_leg = _frame_contacts_xy(frame)
    support_hull = _convex_hull(list(contacts_by_leg.values()))

    tests = []

    for item in _test_points(
        com_proxy_world_xy_m=com_proxy_world_xy_m,
        com_uncertainty_xy_m=com_uncertainty_xy_m,
    ):
        margin = _polygon_signed_margin(
            point=item["world_xy_m"],
            polygon=support_hull,
        )
        tests.append(
            {
                "name": item["name"],
                "world_xy_m": item["world_xy_m"],
                "support_margin_m": margin,
                "inside_support_polygon": margin >= min_support_margin_m,
            }
        )

    min_margin_m = min(test["support_margin_m"] for test in tests)
    safe = min_margin_m >= min_support_margin_m

    return {
        "contacts_world_xy_m": contacts_by_leg,
        "support_hull_world_xy_m": support_hull,
        "test_points": tests,
        "min_support_margin_m": min_margin_m,
        "safe": safe,
    }


def _build_report(
    *,
    repo_root: Path,
    c4c_report_path: Path,
    c4d_report_path: Path,
    c4c: dict[str, Any],
    c4d: dict[str, Any],
    com_proxy_world_xy_m: Point2,
    com_uncertainty_xy_m: float,
    min_support_margin_m: float,
) -> dict[str, Any]:
    if c4c.get("status") not in {
        "OFFLINE_TRAJECTORY_VALID",
        "OFFLINE_TRAJECTORY_VALID_WITH_EXPECTED_FOOT_FORK_REVIEW",
    }:
        raise OfflineStaticStabilityError(
            f"C4-C status inatteso: {c4c.get('status')!r}"
        )

    if c4d.get("status") != "OFFLINE_TIMING_ENVELOPE_VALID":
        raise OfflineStaticStabilityError(
            f"C4-D status inatteso: {c4d.get('status')!r}"
        )

    frames = c4c["frames"]

    frame_reports = [
        {
            "trajectory_index": int(frame["trajectory_index"]),
            "body_z_m": float(
                frame["body_pose"]["translation_world_m"][2]
            ),
            "evaluation": _evaluate_frame(
                frame=frame,
                com_proxy_world_xy_m=com_proxy_world_xy_m,
                com_uncertainty_xy_m=com_uncertainty_xy_m,
                min_support_margin_m=min_support_margin_m,
            ),
        }
        for frame in frames
    ]

    worst_frame = min(
        frame_reports,
        key=lambda item: item["evaluation"]["min_support_margin_m"],
    )
    worst_margin_m = float(
        worst_frame["evaluation"]["min_support_margin_m"]
    )

    safe_samples = sum(
        1 for frame in frame_reports if frame["evaluation"]["safe"]
    )
    all_samples_safe = safe_samples == len(frame_reports)

    status = (
        "OFFLINE_STATIC_STABILITY_VALID_WITH_COM_PROXY_UNCERTAINTY"
        if all_samples_safe
        else "FAIL"
    )

    return {
        "schema": 1,
        "kind": "MATDOG_C4E_OFFLINE_STATIC_STABILITY_SUPPORT_POLYGON",
        "status": status,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "offline_only": True,
        "station_used": False,
        "serial_used": False,
        "motor_command_used": False,
        "source_reports": {
            "c4c_rest_to_stand_trajectory": str(c4c_report_path),
            "c4d_trajectory_timing_envelope": str(c4d_report_path),
        },
        "stability_policy": {
            "support_polygon_source": (
                "convex hull of four achieved foot contact references"
            ),
            "com_model": "base_link_origin_projection_proxy",
            "com_proxy_world_xy_m": com_proxy_world_xy_m,
            "com_uncertainty_xy_m": com_uncertainty_xy_m,
            "tested_points": (
                "center plus four corners of the COM uncertainty box"
            ),
            "min_required_support_margin_m": min_support_margin_m,
            "cad_com_model_used": False,
        },
        "metrics": {
            "samples": len(frame_reports),
            "safe_samples": safe_samples,
            "all_samples_safe": all_samples_safe,
            "worst_sample": worst_frame["trajectory_index"],
            "worst_support_margin_m": worst_margin_m,
            "worst_support_margin_mm": worst_margin_m * 1000.0,
        },
        "frames": frame_reports,
        "command_eligibility": {
            "command_eligible": False,
            "reason": (
                "C4-E validates offline static support polygon only using a "
                "COM proxy. Hardware safe mode, operator approval and "
                "supervised execution checks are still required before any "
                "stand command."
            ),
        },
    }


def _print_summary(report: dict[str, Any], report_path: Path) -> None:
    metrics = report["metrics"]
    policy = report["stability_policy"]

    print("=== MATDOG C4-E OFFLINE STATIC STABILITY SUPPORT POLYGON ===")
    print(f"status: {report['status']}")
    print(f"samples: {metrics['samples']}")
    print(f"safe_samples: {metrics['safe_samples']} / {metrics['samples']}")
    print(
        "com_proxy_world_xy_m: "
        f"({policy['com_proxy_world_xy_m'][0]:+.6f}, "
        f"{policy['com_proxy_world_xy_m'][1]:+.6f})"
    )
    print(
        "com_uncertainty_xy_m: "
        f"±{policy['com_uncertainty_xy_m']:.6f}"
    )
    print("")
    print("WORST CASE:")
    print(f"  worst_sample: {metrics['worst_sample']}")
    print(
        "  worst_support_margin_mm: "
        f"{metrics['worst_support_margin_mm']:.3f}"
    )
    print("")
    print(f"report: {report_path}")
    print("COMMAND_ELIGIBLE: false")
    print("Offline only: no Station, serial or motor command was used.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MATDOG C4-E offline static stability support polygon."
    )
    parser.add_argument(
        "--com-proxy-x-m",
        type=float,
        default=DEFAULT_COM_PROXY_X_M,
    )
    parser.add_argument(
        "--com-proxy-y-m",
        type=float,
        default=DEFAULT_COM_PROXY_Y_M,
    )
    parser.add_argument(
        "--com-uncertainty-xy-m",
        type=float,
        default=DEFAULT_COM_UNCERTAINTY_XY_M,
    )
    parser.add_argument(
        "--min-support-margin-m",
        type=float,
        default=DEFAULT_MIN_SUPPORT_MARGIN_M,
    )
    parser.add_argument(
        "--trajectory-report",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--timing-report",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    if args.com_uncertainty_xy_m < 0.0:
        raise OfflineStaticStabilityError(
            "com_uncertainty_xy_m deve essere >= 0"
        )

    c4c_report_path = (
        args.trajectory_report
        if args.trajectory_report is not None
        else _latest_c4c_report(REPO_ROOT)
    )
    c4d_report_path = (
        args.timing_report
        if args.timing_report is not None
        else _latest_c4d_report(REPO_ROOT)
    )

    c4c = _load_json(c4c_report_path)
    c4d = _load_json(c4d_report_path)

    report = _build_report(
        repo_root=REPO_ROOT,
        c4c_report_path=c4c_report_path,
        c4d_report_path=c4d_report_path,
        c4c=c4c,
        c4d=c4d,
        com_proxy_world_xy_m=(
            float(args.com_proxy_x_m),
            float(args.com_proxy_y_m),
        ),
        com_uncertainty_xy_m=float(args.com_uncertainty_xy_m),
        min_support_margin_m=float(args.min_support_margin_m),
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
