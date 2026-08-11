#!/usr/bin/env python3
"""G9 tests for topology-driven raw path/parking geometry."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import math
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
URDF_PATH = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
G7_PROFILE_PATH = (
    REPO_ROOT
    / "09_Logs/Validation_Reports/Geometry_Compiler"
    / "2026-08-10_185433_MATDOG_GEOMETRY_V5_G7_PURE_PROFILE.json"
)

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

import matdog_geometry_path_planner_v5 as planner  # noqa: E402
from matdog_geometry_contact_search_v5 import EndpointSpecV5  # noqa: E402
from matdog_geometry_path_planner_v5 import (  # noqa: E402
    PARKING_FEASIBLE_1DOF,
    PARKING_FEASIBLE_2DOF,
    PARKING_NO_FEASIBLE,
    PARKING_NOT_NEEDED,
    EndpointParkingPlanV5,
    EndpointParkingTaskV5,
    FirstObstructionV5,
    ParkingObjectiveV5,
    ParkingPlannerParametersV5,
    PathValidationV5,
    build_parking_artifact,
    build_parking_artifact_v2,
    derive_geometry_profile_with_path_plans,
    endpoint_parking_tasks_from_profile,
    parking_content_sha256,
    plan_endpoint_parking,
    validate_configuration_path,
    validate_parking_artifact,
)
from matdog_geometry_path_parking_runner_v5 import (  # noqa: E402
    GeometryPathParkingRunnerV5Error,
    _validated_output_paths,
)
from matdog_geometry_profile_v5 import load_geometry_profile_v5  # noqa: E402
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402


def _path(
    path_id: str,
    *,
    collision_free: bool = True,
    reverse_of: str | None = None,
) -> PathValidationV5:
    forward_hash = hashlib.sha256(f"{path_id}:forward".encode()).hexdigest()
    reverse_hash = hashlib.sha256(f"{path_id}:reverse".encode()).hexdigest()
    obstruction = None
    if not collision_free:
        obstruction = FirstObstructionV5(
            sample_index=1,
            progress=0.5,
            joint_positions_rad={"active": 0.5, "escape_a": 0.0, "escape_b": 0.0},
            link_pair=("opaque_a", "opaque_b"),
            relation="same_branch",
        )
    return PathValidationV5(
        path_id=path_id,
        start_joint_positions_rad={"active": 0.0, "escape_a": 0.0, "escape_b": 0.0},
        end_joint_positions_rad={"active": 1.0, "escape_a": 0.0, "escape_b": 0.0},
        moving_joint_names=("active",),
        active_pair_excluded=("parent", "child"),
        status=planner.PATH_COLLISION_FREE if collision_free else planner.PATH_OBSTRUCTED,
        planned_sample_count=3,
        evaluated_sample_count=3 if collision_free else 2,
        first_obstruction=obstruction,
        min_clearance_m=0.001 if collision_free else None,
        min_clearance_kind="LOWER_BOUND" if collision_free else None,
        clearance_samples_evaluated=2 if collision_free else 0,
        sampled_configuration_sha256=forward_hash,
        reversed_sampled_configuration_sha256=reverse_hash,
        reverse_validation_of=reverse_of,
    )


def _reverse(path: PathValidationV5, path_id: str) -> PathValidationV5:
    value = planner._validated_reverse(path, path_id)
    assert value.reverse_validation_of == path.path_id
    return value


class _TinyModel:
    actuated_joint_names = ("active", "escape_a", "escape_b")
    joints = {
        name: SimpleNamespace(lower_limit_rad=-1.0, upper_limit_rad=1.0)
        for name in actuated_joint_names
    }

    def path_obstruction_pairs(self, *, exclude_pair=None):
        del exclude_pair
        return (("opaque_a", "opaque_b"),)

    def clearance_pairs(self):
        return (("opaque_a", "opaque_b"),)

    def pair_relation(self, link_a, link_b):
        del link_a, link_b
        return "same_branch"


class _TinyScene:
    model = _TinyModel()

    def home_pose(self):
        return {name: 0.0 for name in self.model.actuated_joint_names}

    def full_pose(self, overrides):
        pose = self.home_pose()
        pose.update(overrides)
        return pose

    def first_collision_at_pose(self, pose, *, link_pairs=None):
        del pose, link_pairs
        return False, None

    def collision_transforms(self, pose, links):
        del pose
        return {link: object() for link in links}

    def world_broad_phases(self, pose, links):
        del pose
        return {link: object() for link in links}

    def check_link_pair(self, link_a, link_b, pose, **kwargs):
        del link_a, link_b, pose, kwargs
        return SimpleNamespace(
            status="SEPARATED_AABB",
            clearance_m=0.01,
            clearance_kind="LOWER_BOUND",
        )

    def worst_pair_at_pose(self, pose, *, link_pairs=None):
        del pose, link_pairs
        return (
            "opaque_a",
            "opaque_b",
            SimpleNamespace(
                status="SEPARATED_AABB",
                clearance_m=0.01,
                clearance_kind="LOWER_BOUND",
            ),
        )


def _tiny_task() -> EndpointParkingTaskV5:
    endpoint = EndpointSpecV5(
        endpoint_id="active:min",
        joint_name="active",
        side="min",
        branch_root_joint_name="active",
        structural_depth=0,
        motor_id=901,
        motor_direction=1,
        urdf_lower_rad=-1.0,
        urdf_upper_rad=1.0,
        active_link_pair=("parent", "child"),
    )
    return EndpointParkingTaskV5(
        endpoint=endpoint,
        canonical_endpoint_index=0,
        target_angle_rad=-0.8,
        target_source="GEOMETRIC_CONTACT",
    )


def _candidate(configuration: dict[str, float]) -> planner._FeasibleCandidate:
    path_in = _path("path_in")
    task_path = _path("task_path")
    task_return = _reverse(task_path, "task_return")
    path_out = _reverse(path_in, "path_out")
    objective = ParkingObjectiveV5(
        zero_unintended_intersection=True,
        min_clearance_m=0.002,
        min_clearance_kind="LOWER_BOUND",
        displacement_l2_rad=math.sqrt(sum(value * value for value in configuration.values())),
        deterministic_tie_break=tuple(sorted(configuration.items())),
    )
    return planner._FeasibleCandidate(
        parking_configuration_rad=configuration,
        path_in=path_in,
        task_path=task_path,
        task_return=task_return,
        path_out=path_out,
        objective=objective,
    )


class TestSavedProfileTaskLoader(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = load_geometry_profile_v5(G7_PROFILE_PATH)
        cls.scene = RobotSceneV5.from_urdf(URDF_PATH)
        cls.tasks = endpoint_parking_tasks_from_profile(cls.scene, cls.profile)

    def test_exact_24_task_join_does_not_carry_legacy_context(self):
        self.assertEqual(len(self.tasks), 24)
        self.assertEqual(self.tasks[0].endpoint.endpoint_id, "lf_hip_joint:min")
        self.assertFalse(hasattr(self.tasks[0], "context_pose_rad"))
        self.assertFalse(hasattr(self.tasks[0], "path_obstruction"))

    def test_common_rigid_ancestors_are_not_escape_joints(self):
        task = next(
            item for item in self.tasks if item.endpoint.endpoint_id == "rh_lower_leg_joint:min"
        )
        relevant = planner._relevant_movable_joints(
            self.scene,
            task,
            ("rh_hip_link", "rh_lower_leg_link"),
        )
        self.assertEqual(relevant, ("rh_upper_leg_joint",))

    def test_saved_targets_explicitly_include_sixteen_diagnostic_outside_limit_contacts(self):
        outside = sum(
            not (
                task.endpoint.urdf_lower_rad
                <= task.target_angle_rad
                <= task.endpoint.urdf_upper_rad
            )
            for task in self.tasks
        )
        self.assertEqual(outside, 16)

    def test_cross_branch_escape_uses_other_branch_topology(self):
        task = next(
            item for item in self.tasks if item.endpoint.endpoint_id == "lf_upper_leg_joint:max"
        )
        relevant = planner._relevant_movable_joints(
            self.scene,
            task,
            ("lf_foot_link", "lh_foot_link"),
        )
        self.assertEqual(
            relevant,
            (
                "lf_hip_joint",
                "lf_lower_leg_joint",
                "lh_hip_joint",
                "lh_upper_leg_joint",
                "lh_lower_leg_joint",
            ),
        )


class TestPathSemantics(unittest.TestCase):
    def test_return_is_exact_reverse_of_same_sampled_configuration_set(self):
        scene = _TinyScene()
        forward = validate_configuration_path(
            scene,
            path_id="forward",
            start_joint_positions_rad=scene.home_pose(),
            end_joint_positions_rad={"active": 0.2},
            active_pair_excluded=("parent", "child"),
            step_rad=0.05,
            clearance_sample_stride=2,
        )
        reverse = planner._validated_reverse(forward, "reverse")
        self.assertEqual(
            reverse.sampled_configuration_sha256,
            forward.reversed_sampled_configuration_sha256,
        )
        self.assertEqual(
            reverse.reversed_sampled_configuration_sha256,
            forward.sampled_configuration_sha256,
        )
        self.assertEqual(reverse.reverse_validation_of, "forward")
        self.assertEqual(reverse.min_clearance_kind, "LOWER_BOUND")

    def test_obstructed_path_distinguishes_planned_and_evaluated_sample_counts(self):
        class ObstructedScene(_TinyScene):
            def first_collision_at_pose(self, pose, *, link_pairs=None):
                del link_pairs
                return (pose["active"] >= 0.1, ("opaque_a", "opaque_b"))

        result = validate_configuration_path(
            ObstructedScene(),
            path_id="sampled-obstruction",
            start_joint_positions_rad={"active": 0.0},
            end_joint_positions_rad={"active": 0.3},
            active_pair_excluded=None,
            step_rad=0.05,
            clearance_sample_stride=2,
            measure_clearance=False,
        )
        self.assertEqual(result.status, planner.PATH_OBSTRUCTED)
        self.assertLess(result.evaluated_sample_count, result.planned_sample_count)

    def test_normalized_grid_is_bounded_and_not_historical_seed_table(self):
        values = planner._grid_values(-0.7, 1.3, 6)
        self.assertTrue(all(-0.7 <= value <= 1.3 for value in values))
        self.assertEqual(values, tuple(sorted(values, key=lambda value: (abs(value), value))))
        historical = tuple(math.radians(value) for value in (30, 40, 50, 60, 70, 80, 90))
        self.assertNotEqual(values, historical)

    def test_objective_order_is_clearance_then_displacement_then_tie(self):
        high_clearance = ParkingObjectiveV5(True, 0.004, "LOWER_BOUND", 2.0, (("b", 1.0),))
        low_clearance = ParkingObjectiveV5(True, 0.003, "EXACT", 0.1, (("a", 0.1),))
        self.assertLess(
            planner._candidate_sort_key(high_clearance),
            planner._candidate_sort_key(low_clearance),
        )


class TestSearchControl(unittest.TestCase):
    def _run(self, evaluator):
        baseline = _path("baseline", collision_free=False)
        parameters = ParkingPlannerParametersV5(
            path_step_rad=0.1,
            one_dof_grid_divisions=2,
            two_dof_grid_divisions=2,
            clearance_sample_stride=1,
        )
        with (
            patch.object(planner, "validate_configuration_path", return_value=baseline),
            patch.object(
                planner,
                "_relevant_movable_joints",
                return_value=("escape_a", "escape_b"),
            ),
            patch.object(planner, "_evaluate_candidate", side_effect=evaluator),
        ):
            return plan_endpoint_parking(_TinyScene(), _tiny_task(), parameters=parameters)

    def test_one_dof_search_finishes_before_any_two_dof_search(self):
        calls = []

        def evaluate(scene, task, configuration, parameters):
            del scene, task, parameters
            calls.append(tuple(sorted(configuration)))
            if configuration == {"escape_a": -1.0}:
                return _candidate(configuration)
            return None

        plan = self._run(evaluate)
        self.assertEqual(plan.outcome, PARKING_FEASIBLE_1DOF)
        self.assertEqual(plan.evaluated_1d_candidates, 4)
        self.assertEqual(plan.evaluated_2d_candidates, 0)
        self.assertTrue(all(len(call) == 1 for call in calls))

    def test_two_dof_is_evaluated_only_after_all_one_dof_candidates_fail(self):
        calls = []

        def evaluate(scene, task, configuration, parameters):
            del scene, task, parameters
            calls.append(tuple(sorted(configuration)))
            if len(configuration) == 2 and configuration == {
                "escape_a": -1.0,
                "escape_b": -1.0,
            }:
                return _candidate(configuration)
            return None

        plan = self._run(evaluate)
        self.assertEqual(plan.outcome, PARKING_FEASIBLE_2DOF)
        self.assertEqual(plan.evaluated_1d_candidates, 4)
        self.assertEqual(plan.evaluated_2d_candidates, 4)
        first_two_dof = next(index for index, call in enumerate(calls) if len(call) == 2)
        self.assertEqual(first_two_dof, 4)

    def test_no_feasible_plan_is_valid_geometry_outcome(self):
        plan = self._run(lambda *_args, **_kwargs: None)
        self.assertEqual(plan.outcome, PARKING_NO_FEASIBLE)
        self.assertIsNone(plan.task_return)
        self.assertEqual(plan.evaluated_1d_candidates, 4)
        self.assertEqual(plan.evaluated_2d_candidates, 4)


class TestParkingArtifact(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = load_geometry_profile_v5(G7_PROFILE_PATH)
        cls.scene = RobotSceneV5.from_urdf(URDF_PATH)
        cls.tasks = endpoint_parking_tasks_from_profile(cls.scene, cls.profile)

    def _plans(self):
        result = []
        for task in self.tasks:
            endpoint = task.endpoint
            target_within_limits = (
                endpoint.urdf_lower_rad
                <= task.target_angle_rad
                <= endpoint.urdf_upper_rad
            )
            forward = _path(f"{endpoint.endpoint_id}:task")
            reverse = _reverse(forward, f"{endpoint.endpoint_id}:return")
            result.append(
                EndpointParkingPlanV5(
                    canonical_endpoint_index=len(result),
                    endpoint_id=endpoint.endpoint_id,
                    joint_name=endpoint.joint_name,
                    limit_side=endpoint.side,
                    active_branch_id=endpoint.branch_root_joint_name,
                    declared_limit_rad=endpoint.urdf_declared_limit_rad,
                    target_angle_rad=task.target_angle_rad,
                    target_delta_from_declared_rad=(
                        task.target_angle_rad - endpoint.urdf_declared_limit_rad
                    ),
                    target_within_urdf_limits=target_within_limits,
                    target_domain=(
                        "EXECUTABLE_URDF_DOMAIN"
                        if target_within_limits
                        else "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS"
                    ),
                    target_source=task.target_source,
                    allowed_endpoint_contact_pair=("parent", "child"),
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
                    task_return=reverse,
                    path_out=None,
                    objective=ParkingObjectiveV5(True, 0.001, "LOWER_BOUND", 0.0, ()),
                    evaluated_1d_candidates=0,
                    evaluated_2d_candidates=0,
                )
            )
        return tuple(result)

    def _artifact(self):
        return build_parking_artifact(
            self._plans(),
            geometry_profile_reference={
                "relative_path": str(G7_PROFILE_PATH.relative_to(REPO_ROOT)),
                "file_sha256": hashlib.sha256(G7_PROFILE_PATH.read_bytes()).hexdigest(),
                "semantic_content_sha256": self.profile["semantic_content_sha256"],
            },
            source_file_sha256={"planner.py": "0" * 64, "runner.py": "1" * 64},
            parameters=ParkingPlannerParametersV5(),
            worker_count=1,
            runtime_seconds=12.0,
        )

    def _artifact_v2(
        self,
        *,
        worker_count: int,
        relative_path: str,
        file_sha256: str,
    ):
        return build_parking_artifact_v2(
            self._plans(),
            geometry_profile_semantic_sha256=self.profile["semantic_content_sha256"],
            audit_geometry_profile_reference={
                "relative_path": relative_path,
                "file_sha256": file_sha256,
            },
            source_file_sha256={"planner.py": "0" * 64, "worker.py": "1" * 64},
            parameters=ParkingPlannerParametersV5(),
            worker_count=worker_count,
            runtime_seconds=12.0 / worker_count,
        )

    def test_artifact_contains_complete_return_and_no_policy(self):
        artifact = self._artifact()
        validate_parking_artifact(artifact)
        self.assertEqual(len(artifact["plans"]), 24)
        self.assertTrue(all(plan["task_return"] is not None for plan in artifact["plans"]))
        self.assertFalse(artifact["summary"]["external_safety_policy_applied"])
        self.assertNotIn("safety_threshold_m", repr(artifact))
        self.assertNotIn("min_clearance_pass_m", repr(artifact))
        self.assertNotIn("parking_seed_angles_deg", repr(artifact))

    def test_generation_metadata_does_not_change_parking_semantic_hash(self):
        artifact = self._artifact()
        changed = deepcopy(artifact)
        changed["generation_metadata"] = {
            "generated_at_utc": "different",
            "worker_count": 4,
            "runtime_seconds": 1.0,
        }
        self.assertEqual(parking_content_sha256(artifact), parking_content_sha256(changed))

    def test_builder_canonically_sorts_shuffled_worker_results(self):
        plans = list(self._plans())
        artifact_forward = build_parking_artifact(
            plans,
            geometry_profile_reference={
                "relative_path": str(G7_PROFILE_PATH.relative_to(REPO_ROOT)),
                "file_sha256": hashlib.sha256(G7_PROFILE_PATH.read_bytes()).hexdigest(),
                "semantic_content_sha256": self.profile["semantic_content_sha256"],
            },
            source_file_sha256={"planner.py": "0" * 64},
            parameters=ParkingPlannerParametersV5(),
        )
        artifact_reverse = build_parking_artifact(
            reversed(plans),
            geometry_profile_reference=artifact_forward["input_geometry_profile"],
            source_file_sha256={"planner.py": "0" * 64},
            parameters=ParkingPlannerParametersV5(),
        )
        self.assertEqual(
            artifact_forward["semantic_content_sha256"],
            artifact_reverse["semantic_content_sha256"],
        )

    def test_derived_profile_populates_path_plans_without_mutating_g7(self):
        artifact = self._artifact()
        original = deepcopy(self.profile)
        derived = derive_geometry_profile_with_path_plans(self.profile, artifact)
        self.assertEqual(self.profile, original)
        self.assertEqual(self.profile["path_plans"], [])
        self.assertEqual(len(derived["path_plans"]), 24)
        self.assertNotEqual(
            derived["semantic_content_sha256"],
            self.profile["semantic_content_sha256"],
        )

    def test_v2_file_materialization_identity_is_nonsemantic(self):
        workers_1 = self._artifact_v2(
            worker_count=1,
            relative_path="reports/benchmark-c-profile.json",
            file_sha256="2" * 64,
        )
        workers_4 = self._artifact_v2(
            worker_count=4,
            relative_path="reports/benchmark-d-profile.json",
            file_sha256="3" * 64,
        )
        self.assertEqual(
            workers_1["semantic_content_sha256"],
            workers_4["semantic_content_sha256"],
        )
        self.assertEqual(
            set(workers_1["input_geometry_profile"]),
            {"schema_version", "semantic_content_sha256"},
        )
        self.assertNotEqual(
            workers_1["generation_metadata"]["audit_input_geometry_profile"],
            workers_4["generation_metadata"]["audit_input_geometry_profile"],
        )

    def test_v2_combined_profile_workers_1_and_4_are_semantically_identical(self):
        artifact_1 = self._artifact_v2(
            worker_count=1,
            relative_path="reports/c.json",
            file_sha256="4" * 64,
        )
        artifact_4 = self._artifact_v2(
            worker_count=4,
            relative_path="reports/d.json",
            file_sha256="5" * 64,
        )
        derived_1 = derive_geometry_profile_with_path_plans(self.profile, artifact_1)
        derived_4 = derive_geometry_profile_with_path_plans(self.profile, artifact_4)
        self.assertEqual(
            derived_1["semantic_content_sha256"],
            derived_4["semantic_content_sha256"],
        )
        self.assertNotEqual(
            derived_1["generation_metadata"]["path_planning"],
            derived_4["generation_metadata"]["path_planning"],
        )

    def test_frozen_v1_artifact_remains_readable_and_unchanged(self):
        frozen_path = G7_PROFILE_PATH.with_name(
            "2026-08-10_185433_MATDOG_GEOMETRY_V5_G9_PATH_PARKING.json"
        )
        frozen = __import__("json").loads(frozen_path.read_text(encoding="utf-8"))
        validate_parking_artifact(frozen)
        self.assertEqual(
            frozen["semantic_content_sha256"],
            "3e09aec7ca355f79f43abe803b672958b9d992b1aab9c27b5cc7cdff571b9757",
        )


class TestRunnerOutputSafety(unittest.TestCase):
    def test_runner_forces_one_numerical_thread(self):
        for name in (
            "OMP_NUM_THREADS",
            "OPENBLAS_NUM_THREADS",
            "MKL_NUM_THREADS",
            "NUMEXPR_NUM_THREADS",
        ):
            self.assertEqual(os.environ[name], "1")

    def test_rejects_alias_suffix_escape_and_existing_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            input_profile = root / "input.json"
            input_profile.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(GeometryPathParkingRunnerV5Error, "alias"):
                _validated_output_paths(
                    repo_root=root,
                    input_profile_path=input_profile,
                    parking_json_path=root / "parking.json",
                    derived_profile_path=input_profile,
                )
            with self.assertRaisesRegex(GeometryPathParkingRunnerV5Error, "must both use .json"):
                _validated_output_paths(
                    repo_root=root,
                    input_profile_path=input_profile,
                    parking_json_path=root / "parking.md",
                    derived_profile_path=root / "derived.json",
                )
            existing = root / "parking.json"
            existing.write_text("existing", encoding="utf-8")
            with self.assertRaisesRegex(GeometryPathParkingRunnerV5Error, "overwrite"):
                _validated_output_paths(
                    repo_root=root,
                    input_profile_path=input_profile,
                    parking_json_path=existing,
                    derived_profile_path=root / "derived.json",
                )
            with self.assertRaisesRegex(GeometryPathParkingRunnerV5Error, "escapes"):
                _validated_output_paths(
                    repo_root=root,
                    input_profile_path=input_profile,
                    parking_json_path=root.parent / "outside.json",
                    derived_profile_path=root / "derived.json",
                )


if __name__ == "__main__":
    unittest.main()
