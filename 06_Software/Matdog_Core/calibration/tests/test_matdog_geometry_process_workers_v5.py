#!/usr/bin/env python3
"""Fast offline tests for the G11 spawned-process compute boundary.

The tests exercise batching, pickle transport, deterministic parent-side
aggregation and the numerical-thread environment.  They never load the robot
scene in a child process and never create canonical artifacts.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import multiprocessing
import os
import pickle
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_contact_search_v5 import (  # noqa: E402
    GEOMETRIC_CONTACT_FOUND,
    EndpointAnalysisV5,
    EndpointSpecV5,
    GeometricEndpointResultV5,
)
from matdog_geometry_g4_oracle_v5 import G4ReplayTask  # noqa: E402
from matdog_geometry_path_planner_v5 import (  # noqa: E402
    PARKING_NOT_NEEDED,
    PATH_COLLISION_FREE,
    EndpointParkingPlanV5,
    EndpointParkingTaskV5,
    ParkingObjectiveV5,
    ParkingPlannerParametersV5,
    PathValidationV5,
)
import matdog_geometry_process_workers_v5 as process_workers  # noqa: E402
from matdog_geometry_process_workers_v5 import (  # noqa: E402
    ContactWorkResultV5,
    GeometryInputFingerprintV5,
    GeometryProcessWorkerV5Error,
    THREAD_ENVIRONMENT_NAMES,
    build_contact_work_batches,
    build_parking_work_batches,
    execute_contact_tasks,
    execute_parking_tasks,
    worker_environment_snapshot,
)


JOINT_NAMES = tuple(f"joint_{index:02d}" for index in range(12))


def spawned_worker_environment_probe() -> tuple[int, dict[str, str]]:
    """Importable spawn target with no scene construction or geometry work."""

    from matdog_geometry_process_workers_v5 import worker_environment_snapshot

    return os.getpid(), worker_environment_snapshot()


def _endpoint(joint_index: int, side: str, *, endpoint_id: str | None = None) -> EndpointSpecV5:
    joint_name = JOINT_NAMES[joint_index]
    return EndpointSpecV5(
        endpoint_id=endpoint_id or f"{joint_name}:{side}",
        joint_name=joint_name,
        side=side,
        branch_root_joint_name=f"branch_{joint_index // 3}",
        structural_depth=joint_index % 3,
        motor_id=100 + joint_index,
        motor_direction=1 if joint_index % 2 == 0 else -1,
        urdf_lower_rad=-1.0,
        urdf_upper_rad=1.0,
        active_link_pair=(f"parent_{joint_index:02d}", f"child_{joint_index:02d}"),
    )


def _contact_tasks() -> tuple[G4ReplayTask, ...]:
    return tuple(
        G4ReplayTask(
            endpoint=_endpoint(joint_index, side),
            context_pose_rad={},
            coarse_step_rad=0.1,
            envelope_margin_rad=0.2,
            bisection_resolution_rad=0.0001,
            max_bisection_iterations=40,
            g4_record={},
        )
        for joint_index in range(12)
        for side in ("min", "max")
    )


def _parking_tasks() -> tuple[EndpointParkingTaskV5, ...]:
    return tuple(
        EndpointParkingTaskV5(
            endpoint=_endpoint(joint_index, side),
            canonical_endpoint_index=2 * joint_index + (side == "max"),
            target_angle_rad=-0.8 if side == "min" else 0.8,
            target_source="GEOMETRIC_CONTACT",
        )
        for joint_index in range(12)
        for side in ("min", "max")
    )


def _analysis(task: G4ReplayTask) -> EndpointAnalysisV5:
    endpoint = task.endpoint
    contact_angle = -0.8 if endpoint.side == "min" else 0.8
    geometry = GeometricEndpointResultV5(
        endpoint=endpoint,
        status=GEOMETRIC_CONTACT_FOUND,
        contact_angle_rad=contact_angle,
        contact_link_pair=endpoint.active_link_pair,
        declared_limit_delta_rad=contact_angle - endpoint.urdf_declared_limit_rad,
        search_start_rad=0.0,
        search_domain_rad=(-1.2, 1.2),
        bracket_clear_rad=contact_angle * 0.99,
        bracket_contact_rad=contact_angle,
        coarse_step_rad=task.coarse_step_rad,
        bisection_resolution_rad=task.bisection_resolution_rad,
        max_bisection_iterations=task.max_bisection_iterations,
        bisection_iterations=4,
        context_pose_rad={},
    )
    return EndpointAnalysisV5(geometry=geometry, path=None)


def _path(task: EndpointParkingTaskV5, *, reverse: bool = False) -> PathValidationV5:
    endpoint = task.endpoint
    start = {endpoint.joint_name: 0.0}
    end = {endpoint.joint_name: task.target_angle_rad}
    forward_hash = f"{task.canonical_endpoint_index:064x}"
    reverse_hash = f"{task.canonical_endpoint_index + 24:064x}"
    return PathValidationV5(
        path_id=f"{endpoint.endpoint_id}:{'return' if reverse else 'task'}",
        start_joint_positions_rad=end if reverse else start,
        end_joint_positions_rad=start if reverse else end,
        moving_joint_names=(endpoint.joint_name,),
        active_pair_excluded=endpoint.active_link_pair,
        status=PATH_COLLISION_FREE,
        planned_sample_count=3,
        evaluated_sample_count=3,
        first_obstruction=None,
        min_clearance_m=0.01,
        min_clearance_kind="LOWER_BOUND",
        clearance_samples_evaluated=2,
        sampled_configuration_sha256=reverse_hash if reverse else forward_hash,
        reversed_sampled_configuration_sha256=forward_hash if reverse else reverse_hash,
        reverse_validation_of=(
            f"{endpoint.endpoint_id}:task" if reverse else None
        ),
    )


def _parking_plan(task: EndpointParkingTaskV5) -> EndpointParkingPlanV5:
    forward = _path(task)
    reverse_path = _path(task, reverse=True)
    endpoint = task.endpoint
    return EndpointParkingPlanV5(
        canonical_endpoint_index=task.canonical_endpoint_index,
        endpoint_id=endpoint.endpoint_id,
        joint_name=endpoint.joint_name,
        limit_side=endpoint.side,
        active_branch_id=endpoint.branch_root_joint_name,
        declared_limit_rad=endpoint.urdf_declared_limit_rad,
        target_angle_rad=task.target_angle_rad,
        target_delta_from_declared_rad=(
            task.target_angle_rad - endpoint.urdf_declared_limit_rad
        ),
        target_within_urdf_limits=True,
        target_domain="EXECUTABLE_URDF_DOMAIN",
        target_source=task.target_source,
        allowed_endpoint_contact_pair=endpoint.active_link_pair,
        start_configuration_valid=True,
        baseline_task_path=forward,
        first_blocking_pair=None,
        first_blocking_relation=None,
        relevant_movable_joint_names=(),
        outcome=PARKING_NOT_NEEDED,
        parking_degrees_of_freedom=0,
        parking_configuration_rad={},
        declared_joint_limit_bounds_rad={},
        one_dof_search_grid_rad={},
        two_dof_search_grid_rad={},
        path_in=None,
        task_path=forward,
        task_return=reverse_path,
        path_out=None,
        objective=ParkingObjectiveV5(
            zero_unintended_intersection=True,
            min_clearance_m=0.01,
            min_clearance_kind="LOWER_BOUND",
            displacement_l2_rad=0.0,
            deterministic_tie_break=(),
        ),
        evaluated_1d_candidates=0,
        evaluated_2d_candidates=0,
    )


def _fingerprint() -> GeometryInputFingerprintV5:
    return GeometryInputFingerprintV5(
        urdf_sha256="0" * 64,
        collision_mesh_sha256=(("link", "1" * 64),),
        q0_status_by_joint=tuple(
            (joint_name, "SEPARATED_AABB") for joint_name in JOINT_NAMES
        ),
    )


class TestTopologyDerivedBatching(unittest.TestCase):
    def test_contact_and_parking_form_twelve_min_max_batches(self):
        contact_batches = build_contact_work_batches(JOINT_NAMES, _contact_tasks())
        parking_batches = build_parking_work_batches(JOINT_NAMES, _parking_tasks())

        self.assertEqual(len(contact_batches), 12)
        self.assertEqual(len(parking_batches), 12)
        self.assertEqual(sum(len(batch.items) for batch in contact_batches), 24)
        self.assertEqual(sum(len(batch.tasks) for batch in parking_batches), 24)
        for index, (contact_batch, parking_batch) in enumerate(
            zip(contact_batches, parking_batches)
        ):
            self.assertEqual(contact_batch.canonical_batch_index, index)
            self.assertEqual(parking_batch.canonical_batch_index, index)
            self.assertEqual(contact_batch.joint_name, JOINT_NAMES[index])
            self.assertEqual(parking_batch.joint_name, JOINT_NAMES[index])
            self.assertEqual(
                [item.task.endpoint.side for item in contact_batch.items],
                ["min", "max"],
            )
            self.assertEqual(
                [task.endpoint.side for task in parking_batch.tasks],
                ["min", "max"],
            )
            self.assertEqual(
                [item.canonical_endpoint_index for item in contact_batch.items],
                [2 * index, 2 * index + 1],
            )
            self.assertEqual(
                [task.canonical_endpoint_index for task in parking_batch.tasks],
                [2 * index, 2 * index + 1],
            )

    def test_duplicate_missing_and_bad_parking_index_are_rejected(self):
        contact = _contact_tasks()
        parking = _parking_tasks()
        cases = (
            (
                build_contact_work_batches,
                contact[:-1] + (contact[0],),
                "duplicate contact task",
            ),
            (build_contact_work_batches, contact[:-1], "coverage"),
            (
                build_parking_work_batches,
                parking[:-1] + (parking[0],),
                "duplicate parking task",
            ),
            (build_parking_work_batches, parking[:-1], "coverage"),
            (
                build_parking_work_batches,
                parking[:-1]
                + (replace(parking[-1], canonical_endpoint_index=22),),
                "indices must be exactly 0..23",
            ),
        )
        for builder, tasks, message in cases:
            with self.subTest(builder=builder.__name__, message=message):
                with self.assertRaisesRegex(GeometryProcessWorkerV5Error, message):
                    builder(JOINT_NAMES, tasks)

    def test_joint_side_identity_mismatch_is_rejected_as_bad_coverage(self):
        tasks = list(_parking_tasks())
        tasks[0] = replace(
            tasks[0],
            endpoint=replace(tasks[0].endpoint, side="unexpected"),
        )
        with self.assertRaisesRegex(GeometryProcessWorkerV5Error, "coverage"):
            build_parking_work_batches(JOINT_NAMES, tasks)


class TestSpawnPayload(unittest.TestCase):
    def test_batches_submissions_and_fingerprint_pickle_roundtrip(self):
        contact_batch = build_contact_work_batches(JOINT_NAMES, _contact_tasks())[0]
        parking_batch = build_parking_work_batches(JOINT_NAMES, _parking_tasks())[0]
        values = (
            contact_batch,
            parking_batch,
            _fingerprint(),
            (
                process_workers._worker_analyze_contact_batch,
                (contact_batch, True),
            ),
            (
                process_workers._worker_plan_parking_batch,
                (parking_batch, ParkingPlannerParametersV5()),
            ),
        )
        for value in values:
            with self.subTest(value_type=type(value).__name__):
                self.assertEqual(pickle.loads(pickle.dumps(value)), value)

    def test_parent_and_true_spawn_worker_force_one_numerical_thread(self):
        self.assertEqual(
            worker_environment_snapshot(),
            {name: "1" for name in THREAD_ENVIRONMENT_NAMES},
        )
        inherited_noncanonical = {name: "7" for name in THREAD_ENVIRONMENT_NAMES}
        with patch.dict(os.environ, inherited_noncanonical):
            context = multiprocessing.get_context("spawn")
            with ProcessPoolExecutor(max_workers=1, mp_context=context) as executor:
                child_pid, snapshot = executor.submit(
                    spawned_worker_environment_probe
                ).result(timeout=30)
        self.assertNotEqual(child_pid, os.getpid())
        self.assertEqual(
            snapshot,
            {name: "1" for name in THREAD_ENVIRONMENT_NAMES},
        )


class TestCanonicalParentAggregation(unittest.TestCase):
    def test_contact_results_are_sorted_after_shuffled_collector_completion(self):
        tasks = _contact_tasks()
        raw = [
            ContactWorkResultV5(index, _analysis(task))
            for index, task in enumerate(tasks)
        ]
        with patch.object(
            process_workers,
            "_collect_process_results",
            return_value=list(reversed(raw)),
        ) as collector:
            analyses = execute_contact_tasks(
                Path("unused.urdf"),
                JOINT_NAMES,
                _fingerprint(),
                tasks,
                workers=4,
                include_path_obstruction=True,
            )

        self.assertEqual(
            [analysis.geometry.endpoint.endpoint_id for analysis in analyses],
            [task.endpoint.endpoint_id for task in tasks],
        )
        self.assertEqual(collector.call_args.kwargs["workers"], 4)
        self.assertEqual(
            len(list(collector.call_args.kwargs["submissions"])),
            12,
        )

    def test_parking_results_are_sorted_after_shuffled_collector_completion(self):
        tasks = _parking_tasks()
        raw = [_parking_plan(task) for task in reversed(tasks)]
        with patch.object(
            process_workers,
            "_collect_process_results",
            return_value=raw,
        ) as collector:
            plans = execute_parking_tasks(
                Path("unused.urdf"),
                JOINT_NAMES,
                _fingerprint(),
                tasks,
                workers=4,
                parameters=ParkingPlannerParametersV5(),
            )

        self.assertEqual(
            [plan.canonical_endpoint_index for plan in plans],
            list(range(24)),
        )
        self.assertEqual(collector.call_args.kwargs["workers"], 4)
        self.assertEqual(
            len(list(collector.call_args.kwargs["submissions"])),
            12,
        )

    def test_duplicate_result_index_is_rejected_after_collection(self):
        tasks = _contact_tasks()
        raw = [
            ContactWorkResultV5(index, _analysis(task))
            for index, task in enumerate(tasks)
        ]
        raw[-1] = replace(raw[-1], canonical_endpoint_index=22)
        with patch.object(
            process_workers,
            "_collect_process_results",
            return_value=raw,
        ):
            with self.assertRaisesRegex(
                GeometryProcessWorkerV5Error,
                "canonical endpoint indices 0..23",
            ):
                execute_contact_tasks(
                    Path("unused.urdf"),
                    JOINT_NAMES,
                    _fingerprint(),
                    tasks,
                    workers=4,
                    include_path_obstruction=True,
                )


if __name__ == "__main__":
    unittest.main()
