#!/usr/bin/env python3
"""Pure-geometry schema-v5 serialization and provenance.

The canonical profile contains URDF/STL/model facts and geometric results. It
contains no LF hardware evidence, hardware verdict, manufacturing tolerance,
3 mm safety gate or legacy parking seed table.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from typing import Any, Iterable

from matdog_geometry_contact_search_v5 import EndpointAnalysisV5
from matdog_geometry_mesh_kernel import (
    DEFAULT_GRID_CELL_SIZE_M,
    DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
    DEFAULT_NARROW_PHASE_MARGIN_M,
)
from matdog_geometry_model_v5 import (
    PAIR_CLASS_FIXED_ADJACENT,
    PAIR_CLASS_REVOLUTE_ADJACENT,
)
from matdog_geometry_scene_v5 import RobotSceneV5


SCHEMA_VERSION = "matdog.calibration_geometry_profile.v5"

CALIBRATION_DIR = Path(__file__).resolve().parent
PURE_GEOMETRY_SOURCE_FILES: tuple[str, ...] = (
    "matdog_geometry_mesh_kernel.py",
    "matdog_geometry_model_v5.py",
    "matdog_geometry_scene_v5.py",
    "matdog_geometry_contact_search_v5.py",
    "matdog_geometry_g4_oracle_v5.py",
    "matdog_geometry_profile_v5.py",
    "matdog_geometry_process_workers_v5.py",
    "matdog_geometry_compiler_v5.py",
)

FORBIDDEN_PURE_GEOMETRY_KEYS = frozenset(
    {
        "hardware_evidence_note",
        "hardware_vs_urdf_status",
        "mesh_vs_hardware_status",
        "lf_v25_hardware_reconciliation",
        "contact_model_status",
        "endpoint_evidence_class",
        "min_clearance_pass_m",
        "clearance_gate_result",
        "parking_seed_angles_deg",
        "manufacturing_tolerance",
        "model_limit_mismatch",
    }
)


class GeometryProfileV5Error(RuntimeError):
    """Invalid V5 profile or provenance mismatch."""


def _round(value: float | None, digits: int = 12) -> float | None:
    if value is None:
        return None
    if not math.isfinite(value):
        return None
    return round(value, digits)


def _vector(values: Iterable[float]) -> list[float]:
    return [_round(float(value)) for value in values]  # type: ignore[list-item]


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_value(repo_root: Path, args: list[str]) -> str | None:
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=repo_root,
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


def _git_dirty(repo_root: Path) -> bool | None:
    value = _git_value(repo_root, ["status", "--porcelain"])
    return None if value is None else bool(value)


def _portable_path(path: Path, repo_root: Path) -> str:
    try:
        return str(path.resolve().relative_to(repo_root.resolve()))
    except ValueError:
        return path.name


def geometry_source_manifest(
    source_files: tuple[str, ...] = PURE_GEOMETRY_SOURCE_FILES,
) -> dict[str, str]:
    manifest: dict[str, str] = {}
    for filename in source_files:
        path = CALIBRATION_DIR / filename
        if not path.is_file():
            raise GeometryProfileV5Error(f"pure-geometry source missing: {path}")
        manifest[filename] = _sha256_file(path)
    return manifest


def combined_source_sha256(manifest: dict[str, str]) -> str:
    canonical = "".join(
        f"{name}:{digest}\n" for name, digest in sorted(manifest.items())
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _model_record(scene: RobotSceneV5) -> dict[str, Any]:
    model = scene.model
    links = []
    for link_name in model.link_names_topological:
        link = model.links[link_name]
        links.append(
            {
                "name": link_name,
                "articulated_branch_id": model.branch_root_by_link[link_name],
                "has_collision_mesh": link.collision is not None,
            }
        )

    joints = []
    for joint_name in model.joint_names_topological:
        joint = model.joints[joint_name]
        joints.append(
            {
                "name": joint.name,
                "type": joint.joint_type,
                "parent_link": joint.parent_link,
                "child_link": joint.child_link,
                "origin_xyz": _vector(joint.origin_xyz),
                "origin_rpy": _vector(joint.origin_rpy),
                "axis_xyz": _vector(joint.axis_xyz),
                "limit": (
                    {
                        "lower_rad": _round(joint.lower_limit_rad),
                        "upper_rad": _round(joint.upper_limit_rad),
                    }
                    if joint.lower_limit_rad is not None
                    else None
                ),
                "motor_id": joint.motor_id,
                "motor_direction": joint.motor_direction,
                "selected_actuated": joint_name in model.actuated_joint_names,
                "articulated_branch_id": model.branch_root_by_joint.get(joint_name),
            }
        )

    branches = [
        {
            "branch_id": branch.root_joint_name,
            "root_joint_name": branch.root_joint_name,
            "joint_names": list(branch.joint_names),
            "link_names": list(branch.link_names),
            "deterministic_sort_motor_id": branch.sort_motor_id,
        }
        for branch in model.articulated_branches
    ]
    return {
        "root_link": model.root_link,
        "links": links,
        "joints": joints,
        "selected_actuated_joint_names": list(model.actuated_joint_names),
        "selected_actuated_joint_count": len(model.actuated_joint_names),
        "articulated_branches": branches,
    }


def _collision_manifest(scene: RobotSceneV5) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for link_name in scene.model.collision_link_names:
        entry = scene.model.collision_geometry(link_name)
        mesh = scene.mesh(link_name)
        result[link_name] = {
            "stl_relative_path": entry.filename,
            "sha256": mesh.sha256,
            "triangle_count": mesh.triangle_count,
            "degenerate_triangles_dropped": mesh.degenerate_triangle_count,
            "scale_xyz": _vector(entry.scale),
            "origin_xyz": _vector(entry.origin_xyz),
            "origin_rpy": _vector(entry.origin_rpy),
        }
    return result


def _pair_policy(scene: RobotSceneV5) -> dict[str, Any]:
    model = scene.model
    revolute_pairs = [
        list(pair)
        for pair in model.collision_pairs()
        if model.classify_pair(*pair) == PAIR_CLASS_REVOLUTE_ADJACENT
    ]
    fixed_pairs = [
        list(pair)
        for pair in model.collision_pairs()
        if model.classify_pair(*pair) == PAIR_CLASS_FIXED_ADJACENT
    ]
    return {
        "version": "v5_pure_geometry_topology_driven",
        "active_pair_contact_search_rule": (
            "search exactly the selected revolute joint parent-child pair"
        ),
        "path_obstruction_pair_rule": (
            "all collision pairs except fixed adjacency and the active pair"
        ),
        "revolute_adjacent_pairs": revolute_pairs,
        "fixed_adjacent_pairs": fixed_pairs,
        "revolute_adjacent_pair_count": len(revolute_pairs),
        "fixed_adjacent_pair_count": len(fixed_pairs),
        "active_pair_by_joint": {
            joint_name: list(model.active_revolute_pair(joint_name))
            for joint_name in model.actuated_joint_names
        },
        "path_relation_values": [
            "same_branch",
            "body_vs_branch",
            "cross_branch",
            "body_internal",
        ],
    }


def _endpoint_record(scene: RobotSceneV5, analysis: EndpointAnalysisV5) -> dict[str, Any]:
    geometry = analysis.geometry
    endpoint = geometry.endpoint
    geometric_contact = {
        "status": geometry.status,
        "angle_rad": _round(geometry.contact_angle_rad),
        "minus_declared_limit_rad": _round(geometry.declared_limit_delta_rad),
        "link_pair": list(geometry.contact_link_pair),
        "relation": scene.model.pair_relation(*geometry.contact_link_pair),
        "bracket": {
            "clear_rad": _round(geometry.bracket_clear_rad),
            "contact_rad": _round(geometry.bracket_contact_rad),
        },
        "search": {
            "start_rad": _round(geometry.search_start_rad),
            "domain_rad": _vector(geometry.search_domain_rad),
            "coarse_step_rad": _round(geometry.coarse_step_rad),
            "bisection_resolution_rad": _round(geometry.bisection_resolution_rad),
            "max_bisection_iterations": geometry.max_bisection_iterations,
            "bisection_iterations": geometry.bisection_iterations,
        },
    }

    path_record: dict[str, Any] | None = None
    if analysis.path is not None:
        path = analysis.path
        path_record = {
            "status": path.status,
            "angle_rad": _round(path.obstruction_angle_rad),
            "link_pair": (
                list(path.obstruction_link_pair)
                if path.obstruction_link_pair is not None
                else None
            ),
            "relation": path.relation,
            "precedes_geometric_contact": analysis.path_obstruction_precedes_contact,
            "bracket": {
                "clear_rad": _round(path.bracket_clear_rad),
                "contact_rad": _round(path.bracket_contact_rad),
            },
            "search": {
                "domain_rad": _vector(path.search_domain_rad),
                "coarse_step_rad": _round(path.coarse_step_rad),
                "bisection_resolution_rad": _round(path.bisection_resolution_rad),
                "max_bisection_iterations": path.max_bisection_iterations,
                "bisection_iterations": path.bisection_iterations,
            },
        }

    return {
        "identity": {
            "joint_name": endpoint.joint_name,
            "limit_side": endpoint.side,
            "presentation_id": endpoint.endpoint_id,
        },
        "articulated_branch_id": endpoint.branch_root_joint_name,
        "structural_depth": endpoint.structural_depth,
        "declared_limit_rad": _round(endpoint.urdf_declared_limit_rad),
        "active_revolute_pair": list(endpoint.active_link_pair),
        "search_context": {
            "joint_positions_rad": {
                key: _round(value)
                for key, value in sorted(geometry.context_pose_rad.items())
            }
        },
        "geometric_contact": geometric_contact,
        "path_obstruction": path_record,
    }


def semantic_content_sha256(profile: dict[str, Any]) -> str:
    content = {
        key: value
        for key, value in profile.items()
        if key not in {"generation_metadata", "semantic_content_sha256"}
    }
    canonical = json.dumps(content, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _walk_keys(value: Any) -> Iterable[str]:
    if isinstance(value, dict):
        for key, child in value.items():
            yield str(key)
            yield from _walk_keys(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk_keys(child)


def validate_pure_geometry_profile(profile: dict[str, Any]) -> None:
    if profile.get("schema_version") != SCHEMA_VERSION:
        raise GeometryProfileV5Error(
            f"unexpected schema_version: {profile.get('schema_version')!r}"
        )
    forbidden = sorted(FORBIDDEN_PURE_GEOMETRY_KEYS.intersection(_walk_keys(profile)))
    if forbidden:
        raise GeometryProfileV5Error(
            f"hardware/safety/policy fields found in pure geometry profile: {forbidden}"
        )
    stored = profile.get("semantic_content_sha256")
    recomputed = semantic_content_sha256(profile)
    if stored != recomputed:
        raise GeometryProfileV5Error(
            f"semantic content hash mismatch: stored={stored!r}, recomputed={recomputed!r}"
        )


def build_geometry_profile_v5(
    scene: RobotSceneV5,
    analyses: Iterable[EndpointAnalysisV5],
    *,
    repo_root: Path,
    worker_count: int,
    runtime_seconds: float | None = None,
    geometry_unknowns: Iterable[dict[str, Any] | str] = (),
    source_files: tuple[str, ...] = PURE_GEOMETRY_SOURCE_FILES,
) -> dict[str, Any]:
    analyses = tuple(analyses)
    expected_ids = {
        (joint_name, side)
        for joint_name in scene.model.actuated_joint_names
        for side in ("min", "max")
    }
    actual_ids = {
        (analysis.geometry.endpoint.joint_name, analysis.geometry.endpoint.side)
        for analysis in analyses
    }
    if actual_ids != expected_ids or len(analyses) != len(expected_ids):
        raise GeometryProfileV5Error(
            f"endpoint coverage mismatch: expected {len(expected_ids)}, got {len(analyses)}"
        )
    if worker_count <= 0:
        raise GeometryProfileV5Error(f"worker_count must be positive, got {worker_count}")

    repo_root = Path(repo_root).resolve()
    source_manifest = geometry_source_manifest(source_files)
    analysis_parameters = {
        "coarse_step_rad": _round(analyses[0].geometry.coarse_step_rad),
        "envelope_margin_rad": _round(
            abs(
                max(analyses[0].geometry.search_domain_rad, key=abs)
                - analyses[0].geometry.endpoint.urdf_declared_limit_rad
            )
        ),
        "bisection_resolution_rad": _round(
            analyses[0].geometry.bisection_resolution_rad
        ),
        "max_bisection_iterations": analyses[0].geometry.max_bisection_iterations,
        "narrow_phase_margin_m": DEFAULT_NARROW_PHASE_MARGIN_M,
        "grid_cell_size_m": DEFAULT_GRID_CELL_SIZE_M,
        "max_narrow_phase_candidate_pairs": DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
    }

    profile: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "generation_metadata": {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "working_tree_dirty": _git_dirty(repo_root),
            "worker_count": worker_count,
            "runtime_seconds": _round(runtime_seconds, 6),
        },
        "provenance": {
            "repository": {
                "commit_sha": _git_value(repo_root, ["rev-parse", "HEAD"]),
            },
            "urdf": {
                "relative_path": _portable_path(scene.urdf_path, repo_root),
                "sha256": _sha256_file(scene.urdf_path),
            },
            "collision_meshes": _collision_manifest(scene),
            "geometry_compiler": {
                "source_file_sha256": source_manifest,
                "source_combined_sha256": combined_source_sha256(source_manifest),
            },
        },
        "model": _model_record(scene),
        "geometry_pair_policy": _pair_policy(scene),
        "analysis_parameters": analysis_parameters,
        "endpoint_searches": [
            _endpoint_record(scene, analysis)
            for analysis in sorted(
                analyses,
                key=lambda item: (
                    scene.model.actuated_joint_names.index(item.geometry.endpoint.joint_name),
                    0 if item.geometry.endpoint.side == "min" else 1,
                ),
            )
        ],
        "path_plans": [],
        "geometry_unknowns": sorted(
            [
                dict(item) if isinstance(item, dict) else {"description": str(item)}
                for item in geometry_unknowns
            ],
            key=lambda item: json.dumps(item, sort_keys=True),
        ),
    }
    profile["semantic_content_sha256"] = semantic_content_sha256(profile)
    validate_pure_geometry_profile(profile)
    return profile


def write_geometry_profile_v5(profile: dict[str, Any], path: Path) -> None:
    """Atomically serialize one already-aggregated profile.

    Process workers never call this function; the parent owns canonical output.
    """
    validate_pure_geometry_profile(profile)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(profile, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        dir=target.parent,
        prefix=f".{target.name}.",
        suffix=".tmp",
        text=True,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, target)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def load_geometry_profile_v5(path: Path) -> dict[str, Any]:
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_pure_geometry_profile(profile)
    return profile


def find_geometry_mismatches_v5(
    profile: dict[str, Any],
    scene: RobotSceneV5,
    *,
    repo_root: Path,
    source_files: tuple[str, ...] = PURE_GEOMETRY_SOURCE_FILES,
) -> list[str]:
    mismatches: list[str] = []
    try:
        validate_pure_geometry_profile(profile)
    except GeometryProfileV5Error as exc:
        mismatches.append(str(exc))

    current_urdf_sha = _sha256_file(scene.urdf_path)
    stored_urdf_sha = profile.get("provenance", {}).get("urdf", {}).get("sha256")
    if stored_urdf_sha != current_urdf_sha:
        mismatches.append(
            f"URDF sha256 mismatch: profile={stored_urdf_sha!r}, current={current_urdf_sha!r}"
        )

    stored_meshes = profile.get("provenance", {}).get("collision_meshes", {})
    current_meshes = _collision_manifest(scene)
    if stored_meshes != current_meshes:
        mismatches.append("collision mesh path/hash/count/scale/origin manifest mismatch")

    stored_model = profile.get("model")
    current_model = _model_record(scene)
    if stored_model != current_model:
        mismatches.append("URDF model topology/origin/axis/limits/motor metadata mismatch")

    current_sources = geometry_source_manifest(source_files)
    stored_sources = (
        profile.get("provenance", {})
        .get("geometry_compiler", {})
        .get("source_file_sha256")
    )
    if stored_sources != current_sources:
        mismatches.append("pure geometry source manifest mismatch")
    return mismatches
