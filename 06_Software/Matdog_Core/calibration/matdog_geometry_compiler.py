#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler / 24-endpoint offline geometry audit.

Phase 1 orchestrator: URDF + collision meshes -> per-leg parking plans ->
24 endpoint contact searches (using each leg's determined safe context
pose) -> local sensitivity analysis -> machine-readable geometry profile
+ human-readable report.

NON esegue hardware. NON avvia Station. NON apre la seriale. NON comanda
servo. NON modifica EEPROM. NON modifica norma-core. NON fa commit, push,
PR o merge. NON cancella branch, worktree, log o artefatti.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import sys
from pathlib import Path

CALIBRATION_DIR = Path(__file__).resolve().parent

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_scene import RobotScene, LEG_IDS  # noqa: E402
from matdog_geometry_contact_search import (  # noqa: E402
    DEFAULT_BISECTION_RESOLUTION_RAD,
    DEFAULT_COARSE_STEP_RAD,
    DEFAULT_ENVELOPE_MARGIN_RAD,
    DEFAULT_MAX_BISECTION_ITERATIONS,
    DEFAULT_MODEL_LIMIT_MISMATCH_THRESHOLD_RAD,
    EndpointContactResult,
    load_all_endpoints,
    search_endpoint_contact,
)
from matdog_geometry_path_planner import (  # noqa: E402
    DEFAULT_MIN_CLEARANCE_PASS_M,
    DEFAULT_PARKING_PATH_STEP_RAD,
    DEFAULT_PARKING_SEED_ANGLES_DEG,
    DEFAULT_PATH_STEP_RAD,
    ParkingPlan,
    plan_leg_parking,
)
from matdog_geometry_uncertainty import (  # noqa: E402
    DEFAULT_MIN_GRADIENT_M_PER_RAD,
    DEFAULT_PRINT_TOLERANCE_M,
    DEFAULT_SENSITIVITY_STEP_RAD,
    ContactSensitivityResult,
    ManufacturingToleranceInputs,
    compute_contact_sensitivity,
)
from matdog_geometry_mesh_kernel import (  # noqa: E402
    DEFAULT_GRID_CELL_SIZE_M,
    DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
    DEFAULT_NARROW_PHASE_MARGIN_M,
)
from matdog_geometry_profile import (  # noqa: E402
    build_geometry_profile,
    write_geometry_profile,
)
from matdog_geometry_report import render_report, write_report  # noqa: E402


REPORT_RELATIVE_DIR = Path("09_Logs/Validation_Reports/Geometry_Compiler")


class GeometryCompilerError(RuntimeError):
    """Errore nel Geometry Compiler MATDOG."""


def _numerical_parameters() -> dict:
    return {
        "coarse_step_rad": DEFAULT_COARSE_STEP_RAD,
        "envelope_margin_rad": DEFAULT_ENVELOPE_MARGIN_RAD,
        "bisection_resolution_rad": DEFAULT_BISECTION_RESOLUTION_RAD,
        "max_bisection_iterations": DEFAULT_MAX_BISECTION_ITERATIONS,
        "model_limit_mismatch_threshold_rad": DEFAULT_MODEL_LIMIT_MISMATCH_THRESHOLD_RAD,
        "path_step_rad": DEFAULT_PATH_STEP_RAD,
        "parking_path_step_rad": DEFAULT_PARKING_PATH_STEP_RAD,
        "parking_seed_angles_deg": list(DEFAULT_PARKING_SEED_ANGLES_DEG),
        "min_clearance_pass_m": DEFAULT_MIN_CLEARANCE_PASS_M,
        "narrow_phase_margin_m": DEFAULT_NARROW_PHASE_MARGIN_M,
        "grid_cell_size_m": DEFAULT_GRID_CELL_SIZE_M,
        "max_narrow_phase_candidate_pairs": DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
        "sensitivity_step_rad": DEFAULT_SENSITIVITY_STEP_RAD,
        "min_gradient_m_per_rad": DEFAULT_MIN_GRADIENT_M_PER_RAD,
    }


def run_geometry_compiler(
    repo_root: Path,
    *,
    legs: tuple[str, ...] = LEG_IDS,
) -> tuple[dict, list[EndpointContactResult], dict[str, ParkingPlan], dict[str, ContactSensitivityResult]]:
    scene = RobotScene(repo_root)

    parking_plans: dict[str, ParkingPlan] = {}
    for leg in legs:
        parking_plans[leg] = plan_leg_parking(scene, leg)

    all_endpoints = load_all_endpoints(repo_root)
    endpoints_for_legs = [e for e in all_endpoints if e.leg in legs]

    endpoint_results: list[EndpointContactResult] = []
    sensitivity_by_endpoint: dict[str, ContactSensitivityResult] = {}
    tolerance_inputs = ManufacturingToleranceInputs(print_tolerance_m=DEFAULT_PRINT_TOLERANCE_M)

    for endpoint in endpoints_for_legs:
        plan = parking_plans[endpoint.leg]
        other_legs_pose = plan.active_leg_sequence.other_legs_pose

        result = search_endpoint_contact(scene, endpoint, other_legs_pose=other_legs_pose)
        endpoint_results.append(result)

        if (
            result.result_kind == "MESH_CONTACT_FOUND"
            and result.bracket_clear_rad is not None
            and result.contact_link_a is not None
            and result.contact_link_b is not None
        ):
            sensitivity = compute_contact_sensitivity(
                scene, endpoint, result.bracket_clear_rad, other_legs_pose,
                result.contact_link_a, result.contact_link_b,
            )
            sensitivity_by_endpoint[endpoint.endpoint_id] = sensitivity

    unresolved_assumptions = _collect_unresolved_assumptions(endpoint_results, parking_plans)

    profile = build_geometry_profile(
        scene,
        endpoint_results=endpoint_results,
        sensitivity_by_endpoint=sensitivity_by_endpoint,
        parking_plans=parking_plans,
        tolerance_inputs=tolerance_inputs,
        numerical_parameters=_numerical_parameters(),
        unresolved_assumptions=unresolved_assumptions,
    )

    return profile, endpoint_results, parking_plans, sensitivity_by_endpoint


def _collect_unresolved_assumptions(
    endpoint_results: list[EndpointContactResult],
    parking_plans: dict[str, ParkingPlan],
) -> list[str]:
    notes = [
        (
            "min_clearance_pass_m=3mm ('adequate clearance' threshold for path PASS) is a "
            "documented conservative default chosen for this compiler, not a project-mandated "
            "value; NEEDS_HUMAN_DECISION if a different bar is intended."
        ),
        (
            "manufacturing tolerance UNKNOWN: only the +/-0.15mm per-part print tolerance is "
            "modelled; assembly-level stack-up (bushings, screws, servo horn backlash, fit "
            "clearances) is not known and is not included in the sensitivity estimate."
        ),
        (
            "narrow_phase_margin_m=1mm / grid_cell_size_m=5mm are compiler performance/precision "
            "parameters tuned for MATDOG's actual mm-scale collision meshes; they are configurable "
            "and reported here rather than hardcoded assumptions, per canonical handoff section 4."
        ),
    ]

    for result in endpoint_results:
        eid = result.endpoint.endpoint_id

        if result.contact_model_status == "PATH_COLLISION_BEFORE_ENDPOINT":
            notes.append(
                f"{eid}: PATH_COLLISION_BEFORE_ENDPOINT against "
                f"{result.path_collision_link_a}/{result.path_collision_link_b} even with the leg's "
                "determined parking context; this joint's own designed limit could not be searched "
                "past this obstruction. NEEDS_HUMAN_DECISION on whether a different auxiliary pose "
                "or a mechanical redesign is required."
            )
        elif result.contact_model_status == "MODEL_INCOMPLETE":
            notes.append(
                f"{eid}: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- {result.contact_model_status_reason}. "
                "NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current "
                "collision STL geometry."
            )
        elif result.contact_model_status == "UNINTENDED_SELF_COLLISION":
            notes.append(
                f"{eid}: UNINTENDED_SELF_COLLISION -- {result.contact_model_status_reason}. "
                "NEEDS_HUMAN_DECISION on whether this is a genuine design feature or a modelling "
                "artifact."
            )
        elif result.contact_model_status == "NO_MODELED_ENDSTOP":
            notes.append(
                f"{eid}: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope "
                "around the declared URDF limit; that limit likely originates from a constraint "
                "this compiler does not model (e.g. internal servo/bracket range), not from these "
                "collision meshes."
            )
        elif result.contact_model_status == "MODEL_LIMIT_MISMATCH":
            notes.append(
                f"{eid}: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared "
                f"URDF limit by {result.delta_from_declared_rad:.6f} rad, with no direct hardware "
                "evidence available to explain it; NEEDS_HUMAN_DECISION on whether to revise the "
                "URDF limit (the compiler never does this automatically)."
            )

    for leg, plan in parking_plans.items():
        if plan.required and plan.parking_angle_rad is None:
            notes.append(f"{leg}: {plan.reason}")

    return notes


def _default_report_path(repo_root: Path, suffix: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return repo_root / REPORT_RELATIVE_DIR / f"{stamp}_MATDOG_CALIBRATION_GEOMETRY_{suffix}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MATDOG Geometry Compiler: offline 24-endpoint geometric audit. No hardware."
    )
    parser.add_argument("--repo-root", type=Path, default=CALIBRATION_DIR.parents[2])
    parser.add_argument("--legs", nargs="+", choices=LEG_IDS, default=list(LEG_IDS))
    parser.add_argument("--profile-path", type=Path, default=None)
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    profile, endpoint_results, parking_plans, sensitivity = run_geometry_compiler(
        repo_root, legs=tuple(args.legs)
    )

    profile_path = args.profile_path or _default_report_path(repo_root, "PROFILE.json")
    write_geometry_profile(profile, profile_path)

    report_path = profile_path.with_name(profile_path.name.replace("PROFILE.json", "REPORT.md"))
    write_report(render_report(profile), report_path)

    print("=== MATDOG GEOMETRY COMPILER ===")
    print(f"endpoints processed: {len(endpoint_results)}")
    print(f"profile written: {profile_path}")
    print(f"report written: {report_path}")
    print(f"content_sha256: {profile['content_sha256']}")
    print("HARDWARE NOT USED. NORMA-CORE NOT MODIFIED. NO COMMIT/PUSH/PR/MERGE.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
