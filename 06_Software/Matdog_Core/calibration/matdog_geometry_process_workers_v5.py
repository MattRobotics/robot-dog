#!/usr/bin/env python3
"""Deterministic process-only compute boundary for Geometry Compiler V5.

Workers load one private read-only scene, receive topology-derived endpoint
batches and return indexed dataclasses.  This module has no artifact writer and
never receives an output path.  The parent owns validation, canonical sorting
and serialization.
"""

from __future__ import annotations

import os


THREAD_ENVIRONMENT_NAMES = (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)
for _thread_environment_name in THREAD_ENVIRONMENT_NAMES:
    os.environ[_thread_environment_name] = "1"


from concurrent.futures import ProcessPoolExecutor, as_completed  # noqa: E402
from dataclasses import dataclass  # noqa: E402
import hashlib  # noqa: E402
import multiprocessing  # noqa: E402
from pathlib import Path  # noqa: E402
from typing import Iterable  # noqa: E402

from matdog_geometry_contact_search_v5 import (  # noqa: E402
    EndpointAnalysisV5,
    analyze_endpoint,
)
from matdog_geometry_mesh_kernel import clear_mesh_cache  # noqa: E402
from matdog_geometry_g4_oracle_v5 import G4ReplayTask  # noqa: E402
from matdog_geometry_path_planner_v5 import (  # noqa: E402
    EndpointParkingPlanV5,
    EndpointParkingTaskV5,
    ParkingPlannerParametersV5,
    plan_endpoint_parking,
)
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402


ALLOWED_WORKER_COUNTS = (1, 4)


class GeometryProcessWorkerV5Error(RuntimeError):
    """The deterministic worker boundary or one isolated task failed."""


@dataclass(frozen=True)
class ContactWorkItemV5:
    canonical_endpoint_index: int
    task: G4ReplayTask


@dataclass(frozen=True)
class ContactWorkBatchV5:
    canonical_batch_index: int
    joint_name: str
    items: tuple[ContactWorkItemV5, ...]


@dataclass(frozen=True)
class ContactWorkResultV5:
    canonical_endpoint_index: int
    analysis: EndpointAnalysisV5


@dataclass(frozen=True)
class ParkingWorkBatchV5:
    canonical_batch_index: int
    joint_name: str
    tasks: tuple[EndpointParkingTaskV5, ...]


@dataclass(frozen=True)
class GeometryInputFingerprintV5:
    urdf_sha256: str
    collision_mesh_sha256: tuple[tuple[str, str], ...]
    q0_status_by_joint: tuple[tuple[str, str], ...]


_WORKER_SCENE: RobotSceneV5 | None = None


def _validate_worker_count(workers: int) -> None:
    if workers not in ALLOWED_WORKER_COUNTS:
        raise GeometryProcessWorkerV5Error(
            f"workers must be one of {ALLOWED_WORKER_COUNTS}, got {workers}"
        )


def _side_order(side: str) -> int:
    if side == "min":
        return 0
    if side == "max":
        return 1
    raise GeometryProcessWorkerV5Error(f"unsupported endpoint side: {side!r}")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scene_input_fingerprint(scene: RobotSceneV5) -> GeometryInputFingerprintV5:
    q0 = scene.q0_active_pair_results()
    if len(q0) != 12 or any(result.status == "INTERSECTING" for result in q0.values()):
        raise GeometryProcessWorkerV5Error("q=0 fingerprint gate is not 12/12 separated")
    return GeometryInputFingerprintV5(
        urdf_sha256=_sha256_file(scene.urdf_path),
        collision_mesh_sha256=tuple(
            (link_name, scene.mesh(link_name).sha256)
            for link_name in scene.model.collision_link_names
        ),
        q0_status_by_joint=tuple(
            (joint_name, q0[joint_name].status)
            for joint_name in scene.model.actuated_joint_names
        ),
    )


def _validate_scene_fingerprint(
    scene: RobotSceneV5,
    expected: GeometryInputFingerprintV5,
) -> None:
    observed = scene_input_fingerprint(scene)
    if observed != expected:
        raise GeometryProcessWorkerV5Error(
            f"worker geometry input fingerprint mismatch: expected={expected}, observed={observed}"
        )


def build_contact_work_batches(
    actuated_joint_names: Iterable[str],
    tasks: Iterable[G4ReplayTask],
) -> tuple[ContactWorkBatchV5, ...]:
    tasks = tuple(tasks)
    joint_names = tuple(actuated_joint_names)
    if len(joint_names) != 12 or len(set(joint_names)) != 12:
        raise GeometryProcessWorkerV5Error("contact batching requires 12 unique actuated joints")
    expected_keys = [
        (joint_name, side)
        for joint_name in joint_names
        for side in ("min", "max")
    ]
    actual_by_key: dict[tuple[str, str], G4ReplayTask] = {}
    for task in tasks:
        key = (task.endpoint.joint_name, task.endpoint.side)
        if key in actual_by_key:
            raise GeometryProcessWorkerV5Error(f"duplicate contact task: {key}")
        actual_by_key[key] = task
    if set(actual_by_key) != set(expected_keys) or len(tasks) != 24:
        raise GeometryProcessWorkerV5Error(
            "contact task coverage must match the canonical 24 endpoint specs"
        )
    canonical_index = {key: index for index, key in enumerate(expected_keys)}
    batches = []
    for batch_index, joint_name in enumerate(joint_names):
        items = tuple(
            ContactWorkItemV5(
                canonical_endpoint_index=canonical_index[(joint_name, side)],
                task=actual_by_key[(joint_name, side)],
            )
            for side in ("min", "max")
        )
        batches.append(
            ContactWorkBatchV5(
                canonical_batch_index=batch_index,
                joint_name=joint_name,
                items=items,
            )
        )
    if len(batches) != 12:
        raise GeometryProcessWorkerV5Error(
            f"expected 12 topology-derived contact batches, got {len(batches)}"
        )
    return tuple(batches)


def build_parking_work_batches(
    actuated_joint_names: Iterable[str],
    tasks: Iterable[EndpointParkingTaskV5],
) -> tuple[ParkingWorkBatchV5, ...]:
    tasks = tuple(tasks)
    joint_names = tuple(actuated_joint_names)
    if len(joint_names) != 12 or len(set(joint_names)) != 12:
        raise GeometryProcessWorkerV5Error("parking batching requires 12 unique actuated joints")
    expected_keys = {
        (joint_name, side)
        for joint_name in joint_names
        for side in ("min", "max")
    }
    actual_by_key: dict[tuple[str, str], EndpointParkingTaskV5] = {}
    for task in tasks:
        key = (task.endpoint.joint_name, task.endpoint.side)
        if key in actual_by_key:
            raise GeometryProcessWorkerV5Error(f"duplicate parking task: {key}")
        actual_by_key[key] = task
    if set(actual_by_key) != expected_keys or len(tasks) != 24:
        raise GeometryProcessWorkerV5Error(
            "parking task coverage must match the canonical 24 endpoint specs"
        )
    indices = sorted(task.canonical_endpoint_index for task in tasks)
    if indices != list(range(24)):
        raise GeometryProcessWorkerV5Error(
            f"parking task indices must be exactly 0..23, got {indices}"
        )
    batches = []
    for batch_index, joint_name in enumerate(joint_names):
        joint_tasks = tuple(
            sorted(
                (
                    actual_by_key[(joint_name, "min")],
                    actual_by_key[(joint_name, "max")],
                ),
                key=lambda task: _side_order(task.endpoint.side),
            )
        )
        batches.append(
            ParkingWorkBatchV5(
                canonical_batch_index=batch_index,
                joint_name=joint_name,
                tasks=joint_tasks,
            )
        )
    if len(batches) != 12:
        raise GeometryProcessWorkerV5Error(
            f"expected 12 topology-derived parking batches, got {len(batches)}"
        )
    return tuple(batches)


def _initialize_worker_scene(
    urdf_path: str,
    expected_fingerprint: GeometryInputFingerprintV5,
) -> None:
    global _WORKER_SCENE
    for name in THREAD_ENVIRONMENT_NAMES:
        os.environ[name] = "1"
    _WORKER_SCENE = RobotSceneV5.from_urdf(Path(urdf_path))
    _validate_scene_fingerprint(_WORKER_SCENE, expected_fingerprint)


def _require_worker_scene() -> RobotSceneV5:
    if _WORKER_SCENE is None:
        raise GeometryProcessWorkerV5Error("worker scene was not initialized")
    return _WORKER_SCENE


def _analyze_contact_batch_with_scene(
    scene: RobotSceneV5,
    batch: ContactWorkBatchV5,
    include_path_obstruction: bool,
) -> tuple[ContactWorkResultV5, ...]:
    results = []
    for item in batch.items:
        task = item.task
        analysis = analyze_endpoint(
            scene,
            task.endpoint,
            context_pose_rad=task.context_pose_rad,
            include_path_obstruction=include_path_obstruction,
            coarse_step_rad=task.coarse_step_rad,
            envelope_margin_rad=task.envelope_margin_rad,
            bisection_resolution_rad=task.bisection_resolution_rad,
            max_bisection_iterations=task.max_bisection_iterations,
        )
        results.append(
            ContactWorkResultV5(
                canonical_endpoint_index=item.canonical_endpoint_index,
                analysis=analysis,
            )
        )
    return tuple(results)


def _worker_analyze_contact_batch(
    batch: ContactWorkBatchV5,
    include_path_obstruction: bool,
) -> tuple[ContactWorkResultV5, ...]:
    return _analyze_contact_batch_with_scene(
        _require_worker_scene(),
        batch,
        include_path_obstruction,
    )


def _plan_parking_batch_with_scene(
    scene: RobotSceneV5,
    batch: ParkingWorkBatchV5,
    parameters: ParkingPlannerParametersV5,
) -> tuple[EndpointParkingPlanV5, ...]:
    return tuple(
        plan_endpoint_parking(scene, task, parameters=parameters)
        for task in batch.tasks
    )


def _worker_plan_parking_batch(
    batch: ParkingWorkBatchV5,
    parameters: ParkingPlannerParametersV5,
) -> tuple[EndpointParkingPlanV5, ...]:
    return _plan_parking_batch_with_scene(
        _require_worker_scene(),
        batch,
        parameters,
    )


def _collect_process_results(
    *,
    urdf_path: Path,
    expected_fingerprint: GeometryInputFingerprintV5,
    workers: int,
    submissions: Iterable[tuple[object, tuple]],
) -> list[object]:
    context = multiprocessing.get_context("spawn")
    futures = []
    results: list[object] = []
    try:
        with ProcessPoolExecutor(
            max_workers=workers,
            mp_context=context,
            initializer=_initialize_worker_scene,
            initargs=(str(Path(urdf_path).resolve()), expected_fingerprint),
        ) as executor:
            for function, arguments in submissions:
                futures.append(executor.submit(function, *arguments))
            for future in as_completed(futures):
                results.extend(future.result())
    except BaseException as exc:
        for future in futures:
            future.cancel()
        raise GeometryProcessWorkerV5Error(
            f"process worker failed; no serial fallback is permitted: {exc}"
        ) from exc
    return results


def execute_contact_tasks(
    urdf_path: Path,
    actuated_joint_names: Iterable[str],
    expected_fingerprint: GeometryInputFingerprintV5,
    tasks: Iterable[G4ReplayTask],
    *,
    workers: int,
    include_path_obstruction: bool,
) -> tuple[EndpointAnalysisV5, ...]:
    _validate_worker_count(workers)
    tasks = tuple(tasks)
    batches = build_contact_work_batches(actuated_joint_names, tasks)
    expected_by_index = {
        item.canonical_endpoint_index: (
            item.task.endpoint.joint_name,
            item.task.endpoint.side,
            item.task.endpoint.endpoint_id,
        )
        for batch in batches
        for item in batch.items
    }
    if workers == 1:
        local_scene = RobotSceneV5.from_urdf(Path(urdf_path))
        _validate_scene_fingerprint(local_scene, expected_fingerprint)
        try:
            raw_results = [
                result
                for batch in batches
                for result in _analyze_contact_batch_with_scene(
                    local_scene,
                    batch,
                    include_path_obstruction,
                )
            ]
        finally:
            del local_scene
            clear_mesh_cache()
    else:
        raw_results = _collect_process_results(
            urdf_path=urdf_path,
            expected_fingerprint=expected_fingerprint,
            workers=workers,
            submissions=(
                (_worker_analyze_contact_batch, (batch, include_path_obstruction))
                for batch in batches
            ),
        )
    typed_results = [
        result for result in raw_results if isinstance(result, ContactWorkResultV5)
    ]
    if len(typed_results) != 24 or len(typed_results) != len(raw_results):
        raise GeometryProcessWorkerV5Error(
            f"contact workers returned invalid coverage: {len(typed_results)}/24"
        )
    typed_results.sort(key=lambda result: result.canonical_endpoint_index)
    if [result.canonical_endpoint_index for result in typed_results] != list(range(24)):
        raise GeometryProcessWorkerV5Error(
            "contact worker results do not cover canonical endpoint indices 0..23"
        )
    for result in typed_results:
        endpoint = result.analysis.geometry.endpoint
        observed = (endpoint.joint_name, endpoint.side, endpoint.endpoint_id)
        expected = expected_by_index[result.canonical_endpoint_index]
        if observed != expected:
            raise GeometryProcessWorkerV5Error(
                "contact worker result identity mismatch: "
                f"index={result.canonical_endpoint_index}, expected={expected}, observed={observed}"
            )
    return tuple(result.analysis for result in typed_results)


def execute_parking_tasks(
    urdf_path: Path,
    actuated_joint_names: Iterable[str],
    expected_fingerprint: GeometryInputFingerprintV5,
    tasks: Iterable[EndpointParkingTaskV5],
    *,
    workers: int,
    parameters: ParkingPlannerParametersV5,
) -> tuple[EndpointParkingPlanV5, ...]:
    _validate_worker_count(workers)
    parameters.validate()
    tasks = tuple(tasks)
    batches = build_parking_work_batches(actuated_joint_names, tasks)
    expected_by_index = {
        task.canonical_endpoint_index: (
            task.endpoint.joint_name,
            task.endpoint.side,
            task.endpoint.endpoint_id,
        )
        for task in tasks
    }
    if workers == 1:
        local_scene = RobotSceneV5.from_urdf(Path(urdf_path))
        _validate_scene_fingerprint(local_scene, expected_fingerprint)
        try:
            raw_plans = [
                plan
                for batch in batches
                for plan in _plan_parking_batch_with_scene(
                    local_scene,
                    batch,
                    parameters,
                )
            ]
        finally:
            del local_scene
            clear_mesh_cache()
    else:
        raw_plans = _collect_process_results(
            urdf_path=urdf_path,
            expected_fingerprint=expected_fingerprint,
            workers=workers,
            submissions=(
                (_worker_plan_parking_batch, (batch, parameters))
                for batch in batches
            ),
        )
    typed_plans = [
        plan for plan in raw_plans if isinstance(plan, EndpointParkingPlanV5)
    ]
    if len(typed_plans) != 24 or len(typed_plans) != len(raw_plans):
        raise GeometryProcessWorkerV5Error(
            f"parking workers returned invalid coverage: {len(typed_plans)}/24"
        )
    typed_plans.sort(key=lambda plan: plan.canonical_endpoint_index)
    if [plan.canonical_endpoint_index for plan in typed_plans] != list(range(24)):
        raise GeometryProcessWorkerV5Error(
            "parking worker results do not cover canonical endpoint indices 0..23"
        )
    for plan in typed_plans:
        observed = (plan.joint_name, plan.limit_side, plan.endpoint_id)
        expected = expected_by_index[plan.canonical_endpoint_index]
        if observed != expected:
            raise GeometryProcessWorkerV5Error(
                "parking worker result identity mismatch: "
                f"index={plan.canonical_endpoint_index}, expected={expected}, observed={observed}"
            )
    return tuple(typed_plans)


def worker_environment_snapshot() -> dict[str, str]:
    """Small read-only probe used by tests; it performs no geometry or I/O."""

    return {name: os.environ.get(name, "") for name in THREAD_ENVIRONMENT_NAMES}
