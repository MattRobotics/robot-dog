#!/usr/bin/env python3
"""MATDOG Geometry Compiler V5 pure-geometry composition root.

G11 executes the same endpoint task/result boundary with either one parent
process or four spawned worker processes. Canonical artifacts are always
aggregated and written by the parent.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
import time
from typing import Any


for _thread_environment_name in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
):
    os.environ[_thread_environment_name] = "1"


from matdog_geometry_contact_search_v5 import (
    EndpointAnalysisV5,
    load_endpoint_specs,
)
from matdog_geometry_mesh_kernel import clear_mesh_cache
from matdog_geometry_process_workers_v5 import (
    execute_contact_tasks,
    scene_input_fingerprint,
)
from matdog_geometry_g4_oracle_v5 import (
    G4ReplayTask,
    build_g4_replay_tasks,
    compare_g4_g7,
    load_frozen_g4_profile,
    render_g4_g7_report,
)
from matdog_geometry_profile_v5 import (
    PURE_GEOMETRY_SOURCE_FILES,
    build_geometry_profile_v5,
    geometry_source_manifest,
    write_geometry_profile_v5,
)
from matdog_geometry_report_v5 import (
    render_geometry_report_v5,
    write_geometry_report_v5,
)
from matdog_geometry_scene_v5 import RobotSceneV5


DEFAULT_URDF_RELATIVE_PATH = Path(
    "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
)
DEFAULT_ARTIFACT_RELATIVE_DIR = Path(
    "09_Logs/Validation_Reports/Geometry_Compiler"
)
V5_PROFILE_SOURCE_FILES = PURE_GEOMETRY_SOURCE_FILES


class GeometryCompilerV5Error(RuntimeError):
    """A V5 hard gate failed."""


@dataclass(frozen=True)
class GeometryCompilerV5Run:
    profile: dict[str, Any]
    analyses: tuple[EndpointAnalysisV5, ...]
    q0_status_by_joint: dict[str, str]
    runtime_seconds: float
    g4_g7_comparison: dict[str, Any] | None


def _default_tasks(scene: RobotSceneV5) -> tuple[G4ReplayTask, ...]:
    """Canonical geometry-only endpoint tasks with all other joints at home.

    G4 replay tasks are built separately from the frozen profile. Keeping the
    default context empty proves that +50/+90 degree legacy prerequisites are
    not mandatory core truth.
    """
    from matdog_geometry_contact_search_v5 import (
        DEFAULT_BISECTION_RESOLUTION_RAD,
        DEFAULT_COARSE_STEP_RAD,
        DEFAULT_ENVELOPE_MARGIN_RAD,
        DEFAULT_MAX_BISECTION_ITERATIONS,
    )

    return tuple(
        G4ReplayTask(
            endpoint=endpoint,
            context_pose_rad={},
            coarse_step_rad=DEFAULT_COARSE_STEP_RAD,
            envelope_margin_rad=DEFAULT_ENVELOPE_MARGIN_RAD,
            bisection_resolution_rad=DEFAULT_BISECTION_RESOLUTION_RAD,
            max_bisection_iterations=DEFAULT_MAX_BISECTION_ITERATIONS,
            g4_record={},
        )
        for endpoint in load_endpoint_specs(scene.model)
    )


def run_geometry_compiler_v5(
    *,
    repo_root: Path,
    urdf_path: Path,
    workers: int = 1,
    include_path_obstruction: bool = True,
    g4_reference_profile_path: Path | None = None,
    source_files: tuple[str, ...] = V5_PROFILE_SOURCE_FILES,
) -> GeometryCompilerV5Run:
    if workers not in (1, 4):
        raise GeometryCompilerV5Error("workers must be exactly 1 or 4")
    repo_root = Path(repo_root).resolve()
    initial_source_manifest = geometry_source_manifest(source_files)
    scene = RobotSceneV5.from_urdf(Path(urdf_path))

    q0_results = scene.q0_active_pair_results()
    q0_status = {joint: result.status for joint, result in q0_results.items()}
    bad_q0 = {
        joint: status for joint, status in q0_status.items() if status == "INTERSECTING"
    }
    if len(q0_status) != 12 or bad_q0:
        raise GeometryCompilerV5Error(
            f"STOP: q=0 active-pair hard gate failed; count={len(q0_status)}, intersecting={bad_q0}"
        )

    g4_profile = None
    if g4_reference_profile_path is not None:
        g4_profile = load_frozen_g4_profile(g4_reference_profile_path)
        tasks = build_g4_replay_tasks(
            scene,
            load_endpoint_specs(scene.model),
            g4_profile,
        )
    else:
        tasks = _default_tasks(scene)

    input_fingerprint = scene_input_fingerprint(scene)
    actuated_joint_names = scene.model.actuated_joint_names
    del scene
    clear_mesh_cache()

    started = time.perf_counter()
    analyses = execute_contact_tasks(
        Path(urdf_path),
        actuated_joint_names,
        input_fingerprint,
        tasks,
        workers=workers,
        include_path_obstruction=include_path_obstruction,
    )
    runtime = time.perf_counter() - started

    scene = RobotSceneV5.from_urdf(Path(urdf_path))
    if scene_input_fingerprint(scene) != input_fingerprint:
        raise GeometryCompilerV5Error(
            "STOP: URDF/mesh/q0 fingerprint changed during endpoint execution"
        )
    if geometry_source_manifest(source_files) != initial_source_manifest:
        raise GeometryCompilerV5Error(
            "STOP: geometry source provenance changed during endpoint execution"
        )

    comparison = None
    if g4_profile is not None:
        comparison = compare_g4_g7(tasks, analyses)

    profile = build_geometry_profile_v5(
        scene,
        analyses,
        repo_root=repo_root,
        worker_count=workers,
        runtime_seconds=runtime,
        geometry_unknowns=(
            {
                "code": "NOMINAL_COLLISION_GEOMETRY_ONLY",
                "description": (
                    "assembly stack-up, bushings, screws, backlash and fit clearances are not represented"
                ),
            },
            {
                "code": "ADJACENT_HARDSTOP_LOCAL_SENSITIVITY_NOT_COMPUTED",
                "description": (
                    "minimum adjacent-pair clearance is dominated by joint-core fit and is not a hardstop-feature derivative"
                ),
            },
        ),
        source_files=source_files,
    )
    stored_source_manifest = profile["provenance"]["geometry_compiler"][
        "source_file_sha256"
    ]
    if (
        stored_source_manifest != initial_source_manifest
        or geometry_source_manifest(source_files) != initial_source_manifest
    ):
        raise GeometryCompilerV5Error(
            "STOP: geometry source provenance changed while building the profile"
        )
    result = GeometryCompilerV5Run(
        profile=profile,
        analyses=analyses,
        q0_status_by_joint=q0_status,
        runtime_seconds=runtime,
        g4_g7_comparison=comparison,
    )
    del scene
    clear_mesh_cache()
    return result


def _atomic_json(value: dict[str, Any], path: Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
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


def _default_profile_path(repo_root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        repo_root
        / DEFAULT_ARTIFACT_RELATIVE_DIR
        / f"{stamp}_MATDOG_GEOMETRY_V5_PURE_PROFILE.json"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MATDOG Geometry Compiler V5 pure geometry. Offline only."
    )
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--without-path-obstruction", action="store_true")
    parser.add_argument("--g4-reference-profile", type=Path, default=None)
    parser.add_argument("--profile-path", type=Path, default=None)
    parser.add_argument("--oracle-json-path", type=Path, default=None)
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    urdf_path = (args.urdf or (repo_root / DEFAULT_URDF_RELATIVE_PATH)).resolve()
    run = run_geometry_compiler_v5(
        repo_root=repo_root,
        urdf_path=urdf_path,
        workers=args.workers,
        include_path_obstruction=not args.without_path_obstruction,
        g4_reference_profile_path=args.g4_reference_profile,
    )

    profile_path = args.profile_path or _default_profile_path(repo_root)
    write_geometry_profile_v5(run.profile, profile_path)
    report_path = profile_path.with_name(
        profile_path.name.replace("PROFILE.json", "REPORT.md")
    )
    write_geometry_report_v5(render_geometry_report_v5(run.profile), report_path)

    oracle_json_path = None
    oracle_report_path = None
    if run.g4_g7_comparison is not None:
        oracle_json_path = args.oracle_json_path or profile_path.with_name(
            profile_path.name.replace("PURE_PROFILE.json", "G4_G7_ORACLE.json")
        )
        _atomic_json(run.g4_g7_comparison, oracle_json_path)
        oracle_report_path = oracle_json_path.with_suffix(".md")
        write_geometry_report_v5(
            render_g4_g7_report(run.g4_g7_comparison),
            oracle_report_path,
        )

    print("=== MATDOG GEOMETRY COMPILER V5 — PURE GEOMETRY ===")
    print(f"q=0 active pairs separated: {len(run.q0_status_by_joint)}/12")
    print(f"endpoints processed: {len(run.analyses)}")
    print(f"workers: {args.workers}")
    print(f"runtime_seconds: {run.runtime_seconds:.6f}")
    print(f"semantic_content_sha256: {run.profile['semantic_content_sha256']}")
    print(f"profile: {profile_path}")
    print(f"report: {report_path}")
    if oracle_json_path is not None:
        print(f"G4/G7 oracle JSON: {oracle_json_path}")
        print(f"G4/G7 oracle report: {oracle_report_path}")
    print("NO HARDWARE USED. NO NORMA-CORE MODIFIED. NO MERGE PERFORMED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
