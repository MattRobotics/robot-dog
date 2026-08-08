#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler machine-readable profile serialization.

Builds and (de)serializes the MATDOG_CALIBRATION_GEOMETRY_PROFILE JSON
artifact described in the canonical handoff section 6 "Machine-readable
artifact", pinned by URDF/mesh/source hashes so a future hardware runner
can reject a mismatched geometry profile instead of silently calibrating
against stale geometry (canonical handoff section 12).

Deterministically regenerable from the same input: everything except
`generation_timestamp_utc` must be identical across two runs against the
same repository state (see `content_sha256`, which excludes the
timestamp field precisely so it can be used as a determinism check).

Offline only: no Station, serial, motor command or EEPROM access. Never
writes to the canonical URDF.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

CALIBRATION_DIR = Path(__file__).resolve().parent
KINEMATICS_DIR = CALIBRATION_DIR.parents[0] / "kinematics"

for _extra_path in (KINEMATICS_DIR, CALIBRATION_DIR):
    if str(_extra_path) not in sys.path:
        sys.path.insert(0, str(_extra_path))

from matdog_urdf_fk import CANONICAL_URDF_RELATIVE_PATH  # noqa: E402

from matdog_geometry_scene import RobotScene  # noqa: E402
from matdog_geometry_contact_search import (  # noqa: E402
    LF_V25_HARDWARE_EVIDENCE,
    EndpointContactResult,
)
from matdog_geometry_path_planner import ParkingPlan  # noqa: E402
from matdog_geometry_uncertainty import ContactSensitivityResult, ManufacturingToleranceInputs  # noqa: E402


SCHEMA_VERSION = "matdog.calibration_geometry_profile.v3"
"""v2 (2026-08-07 reconciliation, first pass): adds contact_model_status
(replacing the old implicit "same-leg found => endpoint" rule), explicit
clearance_kind (EXACT/LOWER_BOUND) + clearance_gate_result on path
segments, segment-scoped parking reasons, and per-pair tolerance budget
notes on sensitivity.

v3 (2026-08-07 reconciliation, second pass -- corrects v2): the LF V25
hardware oracle now stores the actual hardware_contact_rad angle instead
of a pre-computed "delta from declared" figure (v2 had the sign wrong on
3 of 6 LF endpoints from inconsistent manual derivations). More
importantly, contact_model_status for hardware-oracle endpoints is now
driven by mesh_vs_hardware agreement, not hardware_vs_urdf agreement --
v2 could call an endpoint MODEL_LIMIT_MISMATCH (implying "maybe fine")
when hardware and URDF happened to agree closely, even though the mesh
contact itself was several degrees away from where hardware actually
stopped, which is the comparison that actually matters. Endpoint records
now carry hardware_vs_urdf_status and mesh_vs_hardware_status as
separate, non-conflated fields alongside contact_model_status."""

GEOMETRY_COMPILER_SOURCE_FILES: tuple[str, ...] = (
    "matdog_geometry_mesh_kernel.py",
    "matdog_geometry_scene.py",
    "matdog_geometry_contact_search.py",
    "matdog_geometry_path_planner.py",
    "matdog_geometry_uncertainty.py",
    "matdog_geometry_profile.py",
    "matdog_geometry_compiler.py",
)


class GeometryProfileError(RuntimeError):
    """Errore nella serializzazione del profilo geometrico MATDOG."""


def _round(value: float | None, digits: int = 12) -> float | None:
    if value is None:
        return None
    if isinstance(value, float) and math.isnan(value):
        return None
    return round(value, digits)


def git_commit_sha(repo_root: Path) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    if completed.returncode != 0:
        return None

    return completed.stdout.strip() or None


def git_dirty(repo_root: Path) -> bool | None:
    try:
        completed = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    if completed.returncode != 0:
        return None

    return bool(completed.stdout.strip())


def geometry_compiler_source_manifest(repo_root: Path) -> dict[str, str]:
    manifest: dict[str, str] = {}

    for filename in GEOMETRY_COMPILER_SOURCE_FILES:
        path = CALIBRATION_DIR / filename

        if not path.is_file():
            raise GeometryProfileError(f"file sorgente del compiler mancante: {path}")

        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest[filename] = digest

    return manifest


def geometry_compiler_source_hash(repo_root: Path) -> str:
    manifest = geometry_compiler_source_manifest(repo_root)
    concatenated = "".join(f"{name}:{digest}\n" for name, digest in sorted(manifest.items()))
    return hashlib.sha256(concatenated.encode("utf-8")).hexdigest()


def _endpoint_record(result: EndpointContactResult, sensitivity: ContactSensitivityResult | None) -> dict[str, Any]:
    endpoint = result.endpoint

    record: dict[str, Any] = {
        "endpoint_id": endpoint.endpoint_id,
        "leg": endpoint.leg,
        "joint_group": endpoint.joint_group,
        "side": endpoint.side,
        "joint_name": endpoint.joint_name,
        "servo_id": endpoint.servo_id,
        "urdf_declared_limit_rad": _round(endpoint.urdf_declared_limit_rad),
        "urdf_lower_rad": _round(endpoint.urdf_lower_rad),
        "urdf_upper_rad": _round(endpoint.urdf_upper_rad),
        "prerequisite_pose_rad": {k: _round(v) for k, v in sorted(endpoint.prerequisite_overrides.items())},
        "other_legs_pose_rad": {k: _round(v) for k, v in sorted(result.other_legs_pose.items())},
        "result_kind": result.result_kind,
        "mesh_predicted_contact_rad": _round(result.mesh_predicted_contact_rad),
        "delta_from_declared_rad": _round(result.delta_from_declared_rad),
        "model_limit_mismatch": result.model_limit_mismatch,
        "contact_link_pair": (
            sorted([result.contact_link_a, result.contact_link_b])
            if result.contact_link_a is not None
            else None
        ),
        "is_cross_leg_contact": result.is_cross_leg,
        "clearance_before_contact_m": _round(result.clearance_before_contact_m, 9),
        "bracket_clear_rad": _round(result.bracket_clear_rad),
        "bracket_contact_rad": _round(result.bracket_contact_rad),
        "contact_model_status": result.contact_model_status,
        "contact_model_status_reason": result.contact_model_status_reason,
        "path_collision_angle_rad": _round(result.path_collision_angle_rad),
        "path_collision_link_pair": (
            sorted([result.path_collision_link_a, result.path_collision_link_b])
            if result.path_collision_link_a is not None
            else None
        ),
        "hardware_evidence_note": result.hardware_evidence_note,
        "hardware_vs_urdf_status": result.hardware_vs_urdf_status,
        "mesh_vs_hardware_status": result.mesh_vs_hardware_status,
        "numerical_search": {
            "coarse_step_rad": _round(result.coarse_step_rad),
            "bisection_resolution_rad": _round(result.bisection_resolution_rad),
            "bisection_iterations": result.bisection_iterations,
            "analysis_envelope_rad": [_round(v) for v in result.analysis_envelope_rad],
        },
        "sensitivity": None,
    }

    if sensitivity is not None:
        record["sensitivity"] = {
            "contact_link_pair": sorted([sensitivity.contact_link_a, sensitivity.contact_link_b]),
            "finite_difference_step_rad": _round(sensitivity.finite_difference_step_rad),
            "clearance_near_m": _round(sensitivity.clearance_near_m, 9),
            "clearance_near_kind": sensitivity.clearance_near_kind,
            "clearance_far_m": _round(sensitivity.clearance_far_m, 9),
            "clearance_far_kind": sensitivity.clearance_far_kind,
            "gradient_m_per_rad": _round(sensitivity.gradient_m_per_rad, 9),
            "gradient_stable": sensitivity.gradient_stable,
            "tolerance_used_m": _round(sensitivity.tolerance_used_m, 9),
            "tolerance_budget_note": sensitivity.tolerance_budget_note,
            "estimated_uncertainty_rad": _round(sensitivity.estimated_uncertainty_rad),
            "unstable_reason": sensitivity.unstable_reason,
        }

    return record


def _parking_record(plan: ParkingPlan) -> dict[str, Any]:
    return {
        "leg": plan.leg,
        "auxiliary_parking_required": plan.required,
        "reason": plan.reason,
        "parked_leg": plan.parked_leg,
        "parking_angle_rad": _round(plan.parking_angle_rad),
        "active_leg_sequence_passed": plan.active_leg_sequence.passed,
        "active_leg_sequence_min_clearance_m": _round(plan.active_leg_sequence.min_clearance_m, 9),
        "park_path_passed": plan.park_path.passed if plan.park_path is not None else None,
        "park_path_min_clearance_m": (
            _round(plan.park_path.min_clearance_m, 9) if plan.park_path is not None else None
        ),
        "segments": [
            {
                "description": segment.description,
                "joint_name": segment.joint_name,
                "start_rad": _round(segment.start_rad),
                "end_rad": _round(segment.end_rad),
                "passed": segment.passed,
                "has_true_collision": segment.has_true_collision,
                "min_clearance_m": _round(segment.min_clearance_m, 9),
                "min_clearance_kind": segment.min_clearance_kind,
                "clearance_gate_result": segment.clearance_gate_result,
                "first_collision_angle_rad": _round(segment.first_collision_angle_rad),
                "first_collision_pair": (
                    sorted(segment.first_collision_pair) if segment.first_collision_pair is not None else None
                ),
                "sample_count": segment.sample_count,
            }
            for segment in plan.active_leg_sequence.segments
        ],
    }


def _lf_v25_reconciliation_table(endpoint_results: list[EndpointContactResult]) -> list[dict[str, Any]]:
    """The mandatory LF V25 hardware-vs-mesh reconciliation table
    (canonical handoff reconciliation section 1): one row per LF
    endpoint with direct V25 hardware evidence.

    Three DIFFERENT comparisons are reported as separate fields, never
    conflated into one "compatible" flag (reconciliation review
    correction): hardware_vs_urdf_status (informational only -- does the
    real hardware contact agree with the declared URDF limit) and
    mesh_vs_hardware_status (the comparison that actually decides
    contact_model_status -- does the collision-mesh event correspond to
    where hardware actually stopped). A close hardware-vs-URDF agreement
    does NOT imply the mesh model is correct if the mesh contact itself
    is several degrees away from the real hardware contact.

    Empty (not an error) if no LF endpoints were processed in this run."""
    rows = []

    for result in sorted(endpoint_results, key=lambda r: r.endpoint.endpoint_id):
        hw = LF_V25_HARDWARE_EVIDENCE.get(result.endpoint.endpoint_id)

        if hw is None:
            continue

        hw_contact_rad = float(hw["hardware_contact_rad"])
        status = result.contact_model_status

        if status == "MODEL_INCOMPLETE":
            action = (
                "none automatic: real stopping mechanism is not represented in the collision "
                "STL (servo/bracket internal limit, or a mesh contact that does not correspond "
                "to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that "
                "feature should be added"
            )
        elif status == "PATH_COLLISION_BEFORE_ENDPOINT":
            action = (
                "none for this endpoint's own limit: resolve/park the path obstruction "
                "separately (see parking_plans), then re-evaluate"
            )
        elif status == "MODELED_ENDSTOP_CONTACT":
            action = "none: mesh contact corresponds to the real hardware contact within tolerance"
        else:
            action = "NEEDS_HUMAN_DECISION: reconcile compiler classification against hardware evidence"

        rows.append(
            {
                "endpoint_id": result.endpoint.endpoint_id,
                "urdf_declared_limit_rad": _round(result.endpoint.urdf_declared_limit_rad),
                "hardware_contact_rad": _round(hw_contact_rad),
                "compiler_contact_model_status": status,
                "compiler_mesh_predicted_contact_rad": _round(result.mesh_predicted_contact_rad),
                "compiler_contact_link_pair": (
                    sorted([result.contact_link_a, result.contact_link_b])
                    if result.contact_link_a is not None
                    else None
                ),
                "hardware_vs_urdf_status": result.hardware_vs_urdf_status,
                "mesh_vs_hardware_status": result.mesh_vs_hardware_status,
                "hardware_evidence_note": hw["note"],
                "required_corrective_action": action,
            }
        )

    return rows


def build_geometry_profile(
    scene: RobotScene,
    *,
    endpoint_results: list[EndpointContactResult],
    sensitivity_by_endpoint: dict[str, ContactSensitivityResult],
    parking_plans: dict[str, ParkingPlan],
    tolerance_inputs: ManufacturingToleranceInputs,
    numerical_parameters: dict[str, Any],
    unresolved_assumptions: list[str],
) -> dict[str, Any]:
    repo_root = scene.repo_root
    urdf_path = scene.urdf_path

    mesh_manifest = {
        link: {
            "stl_relative_path": entry.stl_relative_path,
            "sha256": scene.mesh(link).sha256,
            "triangle_count": scene.mesh(link).triangle_count,
            "degenerate_triangles_dropped": scene.mesh(link).degenerate_triangle_count,
        }
        for link, entry in sorted(scene.mesh_manifest.items())
    }

    profile: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "generation_timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "robot_dog_commit_sha": git_commit_sha(repo_root),
        "robot_dog_working_tree_dirty": git_dirty(repo_root),
        "urdf": {
            "relative_path": str(CANONICAL_URDF_RELATIVE_PATH),
            "sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
        },
        "collision_mesh_manifest": mesh_manifest,
        "geometry_compiler": {
            "source_file_sha256": geometry_compiler_source_manifest(repo_root),
            "source_combined_sha256": geometry_compiler_source_hash(repo_root),
        },
        "numerical_parameters": numerical_parameters,
        "manufacturing_tolerance": {
            "print_tolerance_m": tolerance_inputs.print_tolerance_m,
            "assembly_tolerance_note": tolerance_inputs.assembly_tolerance_note,
        },
        "endpoints": [
            _endpoint_record(result, sensitivity_by_endpoint.get(result.endpoint.endpoint_id))
            for result in sorted(endpoint_results, key=lambda r: r.endpoint.endpoint_id)
        ],
        "parking_plans": {leg: _parking_record(plan) for leg, plan in sorted(parking_plans.items())},
        "lf_v25_hardware_reconciliation": _lf_v25_reconciliation_table(endpoint_results),
        "unresolved_assumptions": sorted(unresolved_assumptions),
    }

    profile["content_sha256"] = content_sha256(profile)
    return profile


def content_sha256(profile: dict[str, Any]) -> str:
    """Hash of the profile excluding the timestamp and any pre-existing
    content hash field, so it can be used as a same-input -> same-output
    determinism check across two separate compiler runs."""
    content = {k: v for k, v in profile.items() if k not in ("generation_timestamp_utc", "content_sha256")}
    canonical = json.dumps(content, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def write_geometry_profile(profile: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_geometry_profile(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def find_geometry_mismatches(profile: dict[str, Any], scene: RobotScene) -> list[str]:
    """Compare a loaded profile's pinned hashes against the current URDF
    and collision meshes, returning a human-readable mismatch per
    difference found (empty list = profile matches current geometry).

    A hardware calibration runner is expected to call this (or the
    equivalent check in its own language) and refuse to proceed on any
    mismatch, per canonical handoff section 12: "The hardware runner must
    reject a mismatched geometry profile rather than silently calibrating
    against stale geometry."
    """
    mismatches: list[str] = []

    current_urdf_sha256 = hashlib.sha256(scene.urdf_path.read_bytes()).hexdigest()
    profile_urdf_sha256 = profile.get("urdf", {}).get("sha256")

    if profile_urdf_sha256 != current_urdf_sha256:
        mismatches.append(
            f"URDF sha256 mismatch: profile has {profile_urdf_sha256!r}, "
            f"current file is {current_urdf_sha256!r}"
        )

    profile_meshes = profile.get("collision_mesh_manifest", {})

    for link in sorted(scene.mesh_manifest):
        current_sha256 = scene.mesh(link).sha256
        profile_entry = profile_meshes.get(link)

        if profile_entry is None:
            mismatches.append(f"{link}: present in current geometry but missing from profile mesh manifest")
            continue

        if profile_entry.get("sha256") != current_sha256:
            mismatches.append(
                f"{link}: mesh sha256 mismatch: profile has {profile_entry.get('sha256')!r}, "
                f"current file is {current_sha256!r}"
            )

    for link in sorted(profile_meshes):
        if link not in scene.mesh_manifest:
            mismatches.append(f"{link}: present in profile mesh manifest but not in current geometry")

    current_source_hash = geometry_compiler_source_hash(scene.repo_root)
    profile_source_hash = profile.get("geometry_compiler", {}).get("source_combined_sha256")

    if profile_source_hash != current_source_hash:
        mismatches.append(
            f"geometry compiler source hash mismatch: profile has {profile_source_hash!r}, "
            f"current source is {current_source_hash!r}"
        )

    return mismatches
