#!/usr/bin/env python3
"""G9B tests for the external, non-geometric safety policy."""

from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
G4_PROFILE_PATH = (
    REPO_ROOT
    / "09_Logs/Validation_Reports/Geometry_Compiler"
    / "2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json"
)
G9_PARKING_PATH = (
    REPO_ROOT
    / "09_Logs/Validation_Reports/Geometry_Compiler"
    / "2026-08-10_185433_MATDOG_GEOMETRY_V5_G9_PATH_PARKING.json"
)

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_path_planner_v5 import (  # noqa: E402
    PARKING_NO_FEASIBLE,
    PARKING_NOT_NEEDED,
    parking_content_sha256,
    validate_parking_artifact,
)
from matdog_geometry_safety_policy import (  # noqa: E402
    GeometrySafetyPolicyError,
    POLICY_FAIL_EXACT,
    POLICY_PASS,
    POLICY_REJECT_GEOMETRY,
    POLICY_UNRESOLVED_BOUND,
    REFERENCE_CLEARANCE_THRESHOLD_M,
    build_safety_policy_artifact,
    _resolve_within_repo,
    safety_policy_content_sha256,
    validate_safety_policy_artifact,
    write_safety_policy_artifact,
)


def _path(path_id: str, *, reverse_of: str | None = None) -> dict:
    forward = hashlib.sha256(f"{path_id}:forward".encode()).hexdigest()
    reverse = hashlib.sha256(f"{path_id}:reverse".encode()).hexdigest()
    return {
        "path_id": path_id,
        "start_joint_positions_rad": {},
        "end_joint_positions_rad": {},
        "moving_joint_names": [],
        "active_pair_excluded": None,
        "status": "COLLISION_FREE",
        "collision_free": True,
        "planned_sample_count": 2,
        "evaluated_sample_count": 2,
        "first_obstruction": None,
        "min_clearance_m": 0.004,
        "min_clearance_kind": "LOWER_BOUND",
        "clearance_samples_evaluated": 2,
        "sampled_configuration_sha256": forward,
        "reversed_sampled_configuration_sha256": reverse,
        "reverse_validation_of": reverse_of,
    }


def _reverse(forward: dict, path_id: str) -> dict:
    return {
        **forward,
        "path_id": path_id,
        "sampled_configuration_sha256": forward["reversed_sampled_configuration_sha256"],
        "reversed_sampled_configuration_sha256": forward["sampled_configuration_sha256"],
        "reverse_validation_of": forward["path_id"],
    }


def _parking_artifact() -> dict:
    plans = []
    for index in range(24):
        task = _path(f"endpoint-{index}:task")
        task_return = _reverse(task, f"endpoint-{index}:return")
        outcome = PARKING_NOT_NEEDED
        clearance = 0.004
        kind = "LOWER_BOUND"
        objective = {
            "zero_unintended_intersection": True,
            "min_clearance_m": clearance,
            "min_clearance_kind": kind,
            "displacement_l2_rad": 0.0,
            "deterministic_tie_break": [],
        }
        if index == 1:
            objective["min_clearance_m"] = 0.002
            objective["min_clearance_kind"] = "EXACT"
        elif index == 2:
            objective["min_clearance_m"] = 0.002
            objective["min_clearance_kind"] = "LOWER_BOUND"
        elif index == 3:
            outcome = PARKING_NO_FEASIBLE
            objective = None
            task_return = None
        plans.append(
            {
                "canonical_endpoint_index": index,
                "endpoint_id": f"endpoint-{index}",
                "joint_name": f"opaque-joint-{index // 2}",
                "limit_side": "min" if index % 2 == 0 else "max",
                "active_branch_id": "opaque-branch",
                "declared_limit_rad": 0.4,
                "target_angle_rad": 0.5,
                "target_delta_from_declared_rad": 0.1,
                "target_within_urdf_limits": False,
                "target_domain": "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS",
                "target_source": "GEOMETRIC_CONTACT",
                "allowed_endpoint_contact_pair": None,
                "start_configuration_valid": True,
                "baseline_task_path": task,
                "first_sampled_blocking_pair": None,
                "first_sampled_blocking_relation": None,
                "relevant_movable_joint_names": [],
                "outcome": outcome,
                "parking_degrees_of_freedom": 0,
                "parking_configuration_rad": {},
                "declared_search_domain": {
                    "kind": "DETERMINISTIC_SAMPLED_GRID_WITHIN_URDF_LIMITS",
                    "joint_limit_bounds_rad": {},
                    "one_dof_grid_rad": {},
                    "two_dof_grid_rad": {},
                },
                "path_in": None,
                "task_path": task,
                "task_return": task_return,
                "path_out": None,
                "objective": objective,
                "evaluated_1d_candidates": 0,
                "evaluated_2d_candidates": 0,
            }
        )
    source_manifest = {"planner.py": "2" * 64}
    source_combined_sha256 = hashlib.sha256(
        "".join(
            f"{name}:{digest}\n"
            for name, digest in sorted(source_manifest.items())
        ).encode("utf-8")
    ).hexdigest()
    artifact = {
        "schema_version": "matdog.geometry_path_parking.v1",
        "generation_metadata": {
            "generated_at_utc": "test",
            "worker_count": 1,
            "runtime_seconds": 1.0,
        },
        "input_geometry_profile": {
            "relative_path": "profile.json",
            "file_sha256": "0" * 64,
            "semantic_content_sha256": "1" * 64,
        },
        "provenance": {
            "source_file_sha256": source_manifest,
            "source_combined_sha256": source_combined_sha256,
        },
        "parameters": {},
        "summary": {
            "endpoint_plan_count": 24,
            "outcome_counts": {},
            "all_start_configurations_valid": True,
            "baseline_obstructed_count": 1,
            "geometry_feasible_complete_sequence_count": 23,
            "no_complete_sequence_count": 1,
            "external_safety_policy_applied": False,
        },
        "plans": plans,
    }
    artifact["semantic_content_sha256"] = parking_content_sha256(artifact)
    validate_parking_artifact(artifact)
    return artifact


class TestExternalSafetyPolicy(unittest.TestCase):
    def setUp(self) -> None:
        self.parking = _parking_artifact()
        self.reference = {
            "path": "parking.json",
            "file_sha256": "4" * 64,
            "semantic_content_sha256": self.parking["semantic_content_sha256"],
        }

    def _build(self, threshold=REFERENCE_CLEARANCE_THRESHOLD_M, *, g4=False):
        profile = None
        reference = None
        if g4:
            profile = json.loads(G4_PROFILE_PATH.read_text(encoding="utf-8"))
            reference = {
                "path": str(G4_PROFILE_PATH.relative_to(REPO_ROOT)),
                "file_sha256": hashlib.sha256(G4_PROFILE_PATH.read_bytes()).hexdigest(),
            }
        return build_safety_policy_artifact(
            self.parking,
            parking_reference=self.reference,
            threshold_m=threshold,
            frozen_g4_profile=profile,
            frozen_g4_reference=reference,
        )

    def test_policy_preserves_exact_lower_bound_and_geometry_distinctions(self):
        artifact = self._build()
        by_id = {row["endpoint_id"]: row for row in artifact["endpoint_policy_results"]}
        self.assertEqual(by_id["endpoint-0"]["policy_result"], POLICY_PASS)
        self.assertEqual(by_id["endpoint-1"]["policy_result"], POLICY_FAIL_EXACT)
        self.assertEqual(by_id["endpoint-2"]["policy_result"], POLICY_UNRESOLVED_BOUND)
        self.assertEqual(by_id["endpoint-3"]["policy_result"], POLICY_REJECT_GEOMETRY)
        self.assertTrue(by_id["endpoint-0"]["raw_geometry_feasible"])
        self.assertFalse(by_id["endpoint-3"]["raw_geometry_feasible"])
        self.assertTrue(by_id["endpoint-0"]["clearance_policy_accepted"])
        self.assertFalse(by_id["endpoint-0"]["raw_target_within_urdf_limits"])
        self.assertEqual(
            by_id["endpoint-0"]["motion_authorization"],
            "NOT_GRANTED_OFFLINE_EVIDENCE_ONLY",
        )
        self.assertEqual(artifact["summary"]["motion_authorization_granted_count"], 0)

    def test_clearance_pass_for_diagnostic_target_is_not_motion_authorization(self):
        artifact = self._build()
        result = artifact["endpoint_policy_results"][0]
        self.assertEqual(result["policy_result"], POLICY_PASS)
        self.assertTrue(result["clearance_policy_accepted"])
        self.assertEqual(result["raw_target_domain"], "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS")
        self.assertFalse(result["raw_target_within_urdf_limits"])
        self.assertEqual(result["motion_authorization"], "NOT_GRANTED_OFFLINE_EVIDENCE_ONLY")

    def test_threshold_change_changes_only_policy_output(self):
        before_parking = deepcopy(self.parking)
        policy_3mm = self._build(0.003)
        policy_1mm = self._build(0.001)
        self.assertEqual(self.parking, before_parking)
        self.assertEqual(
            [row["raw_geometry_outcome"] for row in policy_3mm["endpoint_policy_results"]],
            [row["raw_geometry_outcome"] for row in policy_1mm["endpoint_policy_results"]],
        )
        self.assertNotEqual(
            policy_3mm["semantic_content_sha256"],
            policy_1mm["semantic_content_sha256"],
        )
        self.assertEqual(self.parking["semantic_content_sha256"], before_parking["semantic_content_sha256"])

    def test_frozen_g4_3mm_policy_replays_exactly_and_all_four_sequences_remain_false(self):
        artifact = self._build(g4=True)
        replay = artifact["frozen_g4_legacy_3mm_replay"]
        self.assertTrue(replay["exact_replay"])
        self.assertEqual(replay["active_sequence_passed_count"], 0)
        self.assertEqual(replay["active_sequence_failed_or_unresolved_count"], 4)
        self.assertEqual(replay["content_sha256"], "4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61")

    def test_generation_metadata_is_nonsemantic(self):
        artifact = self._build()
        changed = deepcopy(artifact)
        changed["generation_metadata"] = {"generated_at_utc": "different"}
        self.assertEqual(
            safety_policy_content_sha256(artifact),
            safety_policy_content_sha256(changed),
        )

    def test_validator_rejects_unknown_clearance_kind_and_bad_summary(self):
        artifact = self._build()
        changed = deepcopy(artifact)
        changed["endpoint_policy_results"][0]["raw_clearance_kind"] = "UNKNOWN_KIND"
        changed["semantic_content_sha256"] = safety_policy_content_sha256(changed)
        with self.assertRaisesRegex(GeometrySafetyPolicyError, "unknown clearance kind"):
            validate_safety_policy_artifact(changed)

        changed = deepcopy(artifact)
        changed["summary"]["pass_count"] += 1
        changed["semantic_content_sha256"] = safety_policy_content_sha256(changed)
        with self.assertRaisesRegex(GeometrySafetyPolicyError, "summary is inconsistent"):
            validate_safety_policy_artifact(changed)

    def test_validator_enforces_domain_scope_and_deep_g4_replay(self):
        artifact = self._build(g4=True)
        mutations = (
            (
                "target domain contradicts",
                lambda changed: changed["endpoint_policy_results"][0].__setitem__(
                    "raw_target_domain", "EXECUTABLE_URDF_DOMAIN"
                ),
            ),
            (
                "acceptance scope is not clearance-only",
                lambda changed: changed["policy"].__setitem__(
                    "acceptance_scope", "overall executable motion safety"
                ),
            ),
            (
                "segment replay is not exact",
                lambda changed: changed["frozen_g4_legacy_3mm_replay"]["plans"][0][
                    "segments"
                ][0].__setitem__("exact_match", False),
            ),
        )
        for expected_message, mutate in mutations:
            with self.subTest(expected_message=expected_message):
                changed = deepcopy(artifact)
                mutate(changed)
                changed["semantic_content_sha256"] = safety_policy_content_sha256(changed)
                with self.assertRaisesRegex(GeometrySafetyPolicyError, expected_message):
                    validate_safety_policy_artifact(changed)

    def test_source_provenance_and_no_clobber(self):
        artifact = self._build()
        source_manifest = artifact["provenance"]["source_file_sha256"]
        self.assertEqual(len(source_manifest), 1)
        self.assertIn(
            "06_Software/Matdog_Core/calibration/matdog_geometry_safety_policy.py",
            source_manifest,
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "policy.json"
            write_safety_policy_artifact(artifact, output)
            with self.assertRaisesRegex(GeometrySafetyPolicyError, "refusing to overwrite"):
                write_safety_policy_artifact(artifact, output)

    def test_repository_boundary_rejects_path_escape(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory).resolve()
            with self.assertRaisesRegex(GeometrySafetyPolicyError, "inside repository root"):
                _resolve_within_repo(
                    repo_root,
                    repo_root.parent / "escaped-policy.json",
                    label="safety output",
                    must_exist=False,
                )

    def test_real_g9_policy_preserves_diagnostic_target_boundary(self):
        parking = json.loads(G9_PARKING_PATH.read_text(encoding="utf-8"))
        g4 = json.loads(G4_PROFILE_PATH.read_text(encoding="utf-8"))
        artifact = build_safety_policy_artifact(
            parking,
            parking_reference={
                "path": str(G9_PARKING_PATH.relative_to(REPO_ROOT)),
                "file_sha256": hashlib.sha256(G9_PARKING_PATH.read_bytes()).hexdigest(),
                "semantic_content_sha256": parking["semantic_content_sha256"],
            },
            frozen_g4_profile=g4,
            frozen_g4_reference={
                "path": str(G4_PROFILE_PATH.relative_to(REPO_ROOT)),
                "file_sha256": hashlib.sha256(G4_PROFILE_PATH.read_bytes()).hexdigest(),
            },
        )
        validate_safety_policy_artifact(artifact)
        self.assertEqual(artifact["summary"]["pass_count"], 16)
        self.assertEqual(artifact["summary"]["fail_count"], 0)
        self.assertEqual(artifact["summary"]["unresolved_count"], 8)
        self.assertEqual(
            artifact["summary"]["diagnostic_target_outside_urdf_limits_count"],
            16,
        )
        self.assertEqual(
            artifact["summary"]["clearance_pass_but_target_outside_urdf_limits_count"],
            8,
        )
        self.assertEqual(artifact["summary"]["motion_authorization_granted_count"], 0)

    def test_geometry_core_never_imports_external_safety_policy(self):
        for filename in (
            "matdog_geometry_model_v5.py",
            "matdog_geometry_scene_v5.py",
            "matdog_geometry_contact_search_v5.py",
            "matdog_geometry_profile_v5.py",
            "matdog_geometry_path_planner_v5.py",
        ):
            tree = ast.parse((CALIBRATION_DIR / filename).read_text(encoding="utf-8"))
            imports = {
                node.module
                for node in ast.walk(tree)
                if isinstance(node, ast.ImportFrom) and node.module is not None
            }
            self.assertNotIn("matdog_geometry_safety_policy", imports, filename)


if __name__ == "__main__":
    unittest.main()
