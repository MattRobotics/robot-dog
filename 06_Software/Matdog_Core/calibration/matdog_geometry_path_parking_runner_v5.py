#!/usr/bin/env python3
"""Offline G9 runner over one explicit, validated V5 geometry profile.

The frozen G7 input is read-only.  This parent process validates live
URDF/mesh/model/source provenance before and after planning, writes a separate
raw path/parking artifact, and optionally writes a new derived V5 profile with
``path_plans`` populated.  No hardware or safety policy is imported.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import time


for _thread_environment_name in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
):
    os.environ[_thread_environment_name] = "1"


from matdog_geometry_path_planner_v5 import (  # noqa: E402
    ParkingPlannerParametersV5,
    build_parking_artifact_v2,
    derive_geometry_profile_with_path_plans,
    endpoint_parking_tasks_from_profile,
    write_parking_artifact,
)
from matdog_geometry_mesh_kernel import clear_mesh_cache  # noqa: E402
from matdog_geometry_process_workers_v5 import (  # noqa: E402
    execute_parking_tasks,
    scene_input_fingerprint,
)
from matdog_geometry_profile_v5 import (  # noqa: E402
    find_geometry_mismatches_v5,
    load_geometry_profile_v5,
    write_geometry_profile_v5,
)
from matdog_geometry_report_v5 import (  # noqa: E402
    render_geometry_report_v5,
    write_geometry_report_v5,
)
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402


class GeometryPathParkingRunnerV5Error(RuntimeError):
    """A G9 input/provenance hard gate failed."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _inside_repo(path: Path, repo_root: Path, *, label: str) -> Path:
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(repo_root)
    except ValueError as exc:
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: {label} escapes explicit repository root: {resolved}"
        ) from exc
    return resolved


def _relative_path(path: Path, repo_root: Path) -> str:
    return str(_inside_repo(path, repo_root, label="artifact").relative_to(repo_root))


def _planner_source_manifest(repo_root: Path) -> dict[str, str]:
    paths = (
        Path(__file__).resolve().with_name("matdog_geometry_path_planner_v5.py"),
        Path(__file__).resolve().with_name("matdog_geometry_process_workers_v5.py"),
        Path(__file__).resolve(),
    )
    return {
        _relative_path(path, repo_root): _sha256_file(path)
        for path in paths
    }


def _validated_output_paths(
    *,
    repo_root: Path,
    input_profile_path: Path,
    parking_json_path: Path,
    derived_profile_path: Path,
) -> tuple[Path, Path, Path, Path]:
    parking_json = _inside_repo(
        parking_json_path,
        repo_root,
        label="parking JSON output",
    )
    derived_profile = _inside_repo(
        derived_profile_path,
        repo_root,
        label="derived profile output",
    )
    if parking_json.suffix != ".json" or derived_profile.suffix != ".json":
        raise GeometryPathParkingRunnerV5Error(
            "STOP: parking and derived-profile outputs must both use .json"
        )
    parking_report = parking_json.with_suffix(".md")
    derived_report = derived_profile.with_suffix(".md")
    paths = (parking_json, parking_report, derived_profile, derived_report)
    resolved_input = Path(input_profile_path).resolve()
    if len(set(paths)) != len(paths) or resolved_input in paths:
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: input/output path alias detected: input={resolved_input}, outputs={paths}"
        )
    existing = [str(path) for path in paths if path.exists()]
    if existing:
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: refusing to overwrite existing G9 output(s): {existing}"
        )
    return paths


def _resolve_recorded_urdf(
    geometry_profile: dict,
    repo_root: Path,
) -> Path:
    recorded = geometry_profile.get("provenance", {}).get("urdf", {}).get("relative_path")
    if not isinstance(recorded, str) or not recorded:
        raise GeometryPathParkingRunnerV5Error(
            "STOP: geometry profile has no usable recorded URDF path"
        )
    relative = Path(recorded)
    if relative.is_absolute():
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: geometry profile URDF path is not repository-relative: {recorded}"
        )
    urdf_path = _inside_repo(repo_root / relative, repo_root, label="recorded URDF")
    if not urdf_path.is_file():
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: recorded URDF does not exist: {urdf_path}"
        )
    return urdf_path


def _assert_live_provenance(
    geometry_profile: dict,
    scene: RobotSceneV5,
    repo_root: Path,
    *,
    phase: str,
) -> None:
    mismatches = find_geometry_mismatches_v5(
        geometry_profile,
        scene,
        repo_root=repo_root,
    )
    if mismatches:
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: {phase} geometry provenance mismatch: {mismatches}"
        )
    q0 = scene.q0_active_pair_results()
    intersecting = {
        joint_name: result.status
        for joint_name, result in q0.items()
        if result.status == "INTERSECTING"
    }
    if len(q0) != 12 or intersecting:
        raise GeometryPathParkingRunnerV5Error(
            f"STOP: {phase} q=0 gate failed: count={len(q0)}, intersecting={intersecting}"
        )


def run_path_parking_from_saved_profile(
    *,
    repo_root: Path,
    geometry_profile_path: Path,
    parameters: ParkingPlannerParametersV5 = ParkingPlannerParametersV5(),
    workers: int = 1,
) -> tuple[dict, dict, float]:
    if workers not in (1, 4):
        raise GeometryPathParkingRunnerV5Error("STOP: workers must be exactly 1 or 4")
    repo_root = Path(repo_root).resolve()
    profile_path = _inside_repo(
        geometry_profile_path,
        repo_root,
        label="input geometry profile",
    )
    input_file_sha256 = _sha256_file(profile_path)
    geometry_profile = load_geometry_profile_v5(profile_path)
    urdf_path = _resolve_recorded_urdf(geometry_profile, repo_root)
    scene = RobotSceneV5.from_urdf(urdf_path)
    _assert_live_provenance(
        geometry_profile,
        scene,
        repo_root,
        phase="pre-G9",
    )
    initial_source_manifest = _planner_source_manifest(repo_root)
    tasks = endpoint_parking_tasks_from_profile(scene, geometry_profile)
    input_fingerprint = scene_input_fingerprint(scene)
    actuated_joint_names = scene.model.actuated_joint_names
    del scene
    clear_mesh_cache()

    started = time.perf_counter()
    plans = execute_parking_tasks(
        urdf_path,
        actuated_joint_names,
        input_fingerprint,
        tasks,
        workers=workers,
        parameters=parameters,
    )
    runtime_seconds = time.perf_counter() - started

    if _sha256_file(profile_path) != input_file_sha256:
        raise GeometryPathParkingRunnerV5Error(
            "STOP: input geometry profile changed during G9"
        )
    final_source_manifest = _planner_source_manifest(repo_root)
    if final_source_manifest != initial_source_manifest:
        raise GeometryPathParkingRunnerV5Error(
            "STOP: G9 planner/runner source changed during execution"
        )
    scene = RobotSceneV5.from_urdf(urdf_path)
    if scene_input_fingerprint(scene) != input_fingerprint:
        raise GeometryPathParkingRunnerV5Error(
            "STOP: URDF/mesh/q0 fingerprint changed during G9"
        )
    _assert_live_provenance(
        geometry_profile,
        scene,
        repo_root,
        phase="post-G9",
    )
    del scene
    clear_mesh_cache()

    audit_profile_reference = {
        "relative_path": _relative_path(profile_path, repo_root),
        "file_sha256": input_file_sha256,
    }
    artifact = build_parking_artifact_v2(
        plans,
        geometry_profile_semantic_sha256=geometry_profile[
            "semantic_content_sha256"
        ],
        audit_geometry_profile_reference=audit_profile_reference,
        source_file_sha256=initial_source_manifest,
        parameters=parameters,
        worker_count=workers,
        runtime_seconds=runtime_seconds,
    )
    derived_profile = derive_geometry_profile_with_path_plans(
        geometry_profile,
        artifact,
    )
    return artifact, derived_profile, runtime_seconds


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MATDOG Geometry Compiler V5 G9 raw path/parking. Offline only."
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--geometry-profile", type=Path, required=True)
    parser.add_argument("--parking-json-path", type=Path, required=True)
    parser.add_argument("--derived-profile-path", type=Path, required=True)
    parser.add_argument("--path-step-rad", type=float, default=None)
    parser.add_argument("--one-dof-grid-divisions", type=int, default=None)
    parser.add_argument("--two-dof-grid-divisions", type=int, default=None)
    parser.add_argument("--clearance-sample-stride", type=int, default=None)
    parser.add_argument("--workers", type=int, choices=(1, 4), default=1)
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    input_profile = _inside_repo(
        args.geometry_profile,
        repo_root,
        label="input geometry profile",
    )
    (
        parking_json_path,
        expected_parking_report_path,
        derived_profile_path,
        derived_report_path,
    ) = _validated_output_paths(
        repo_root=repo_root,
        input_profile_path=input_profile,
        parking_json_path=args.parking_json_path,
        derived_profile_path=args.derived_profile_path,
    )

    defaults = ParkingPlannerParametersV5()
    parameters = ParkingPlannerParametersV5(
        path_step_rad=(args.path_step_rad if args.path_step_rad is not None else defaults.path_step_rad),
        one_dof_grid_divisions=(
            args.one_dof_grid_divisions
            if args.one_dof_grid_divisions is not None
            else defaults.one_dof_grid_divisions
        ),
        two_dof_grid_divisions=(
            args.two_dof_grid_divisions
            if args.two_dof_grid_divisions is not None
            else defaults.two_dof_grid_divisions
        ),
        clearance_sample_stride=(
            args.clearance_sample_stride
            if args.clearance_sample_stride is not None
            else defaults.clearance_sample_stride
        ),
    )
    artifact, derived_profile, runtime_seconds = run_path_parking_from_saved_profile(
        repo_root=repo_root,
        geometry_profile_path=input_profile,
        parameters=parameters,
        workers=args.workers,
    )
    _validated_output_paths(
        repo_root=repo_root,
        input_profile_path=input_profile,
        parking_json_path=parking_json_path,
        derived_profile_path=derived_profile_path,
    )
    parking_json, parking_report = write_parking_artifact(
        artifact,
        parking_json_path,
    )
    if parking_report != expected_parking_report_path:
        raise GeometryPathParkingRunnerV5Error(
            "STOP: parking writer returned an unexpected report path"
        )
    write_geometry_profile_v5(derived_profile, derived_profile_path)
    write_geometry_report_v5(
        render_geometry_report_v5(derived_profile),
        derived_report_path,
    )

    print("=== MATDOG GEOMETRY COMPILER V5 — G9 PATH/PARKING ===")
    print(f"runtime_seconds: {runtime_seconds:.6f}")
    print(f"workers: {args.workers}")
    print(f"parking_semantic_sha256: {artifact['semantic_content_sha256']}")
    print(f"derived_profile_semantic_sha256: {derived_profile['semantic_content_sha256']}")
    print(f"parking JSON: {parking_json}")
    print(f"parking report: {parking_report}")
    print(f"derived profile: {derived_profile_path}")
    print(f"derived geometry report: {derived_report_path}")
    print("NO HARDWARE USED. NO NORMA-CORE MODIFIED. NO MERGE PERFORMED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
