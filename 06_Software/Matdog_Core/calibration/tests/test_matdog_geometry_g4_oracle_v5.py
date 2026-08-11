#!/usr/bin/env python3
"""Frozen G4 provenance hard-gate tests for the V5 oracle adapter."""

from __future__ import annotations

import copy
from dataclasses import fields, replace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_contact_search_v5 import (  # noqa: E402
    GEOMETRIC_CONTACT_FOUND,
    NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
    NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
    PATH_DOMAIN_DIRECT_TO_GEOMETRIC_TARGET,
    PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE,
    PATH_OBSTRUCTION,
    EndpointAnalysisV5,
    EndpointSpecV5,
    GeometricEndpointResultV5,
    PathObstructionResultV5,
)
from matdog_geometry_g4_oracle_v5 import (  # noqa: E402
    EXPECTED_G4_CONTENT_SHA256,
    G4OracleV5Error,
    G4ReplayTask,
    G4ReplayRunV5,
    _validate_replay_fingerprint,
    compare_g4_g7,
    load_frozen_g4_profile,
    render_g4_g7_report,
    run_g4_replay_v5,
)
from matdog_geometry_process_workers_v5 import (  # noqa: E402
    GeometryInputFingerprintV5,
)


FROZEN_G4 = REPO_ROOT / (
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json"
)


class TestFrozenG4ContentGate(unittest.TestCase):
    def test_stored_and_recomputed_content_hash_both_match(self) -> None:
        profile = load_frozen_g4_profile(FROZEN_G4)
        self.assertEqual(profile["content_sha256"], EXPECTED_G4_CONTENT_SHA256)

    def test_content_tamper_is_rejected_even_when_stored_hash_is_unchanged(self) -> None:
        profile = json.loads(FROZEN_G4.read_text(encoding="utf-8"))
        profile["endpoints"][0]["mesh_predicted_contact_rad"] += 0.001
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "tampered.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            with self.assertRaises(G4OracleV5Error):
                load_frozen_g4_profile(path)

    def test_all_24_frozen_replay_records_explicitly_carry_noncanonical_context(self) -> None:
        profile = load_frozen_g4_profile(FROZEN_G4)
        contexts = [
            {
                **record["other_legs_pose_rad"],
                **record["prerequisite_pose_rad"],
            }
            for record in profile["endpoints"]
        ]
        self.assertEqual(len(contexts), 24)
        self.assertTrue(all(context for context in contexts))

    def test_q0_accepts_closed_separated_taxonomy_and_rejects_intersection(self) -> None:
        profile = load_frozen_g4_profile(FROZEN_G4)
        statuses = ("SEPARATED_AABB", "SEPARATED_HULL", "SEPARATED_NARROW")
        fingerprint = GeometryInputFingerprintV5(
            urdf_sha256=profile["urdf"]["sha256"],
            collision_mesh_sha256=tuple(
                (name, record["sha256"])
                for name, record in profile["collision_mesh_manifest"].items()
            ),
            q0_status_by_joint=tuple(
                (f"joint_{index}", statuses[index % len(statuses)])
                for index in range(12)
            ),
        )
        _validate_replay_fingerprint(fingerprint, profile)
        bad = replace(
            fingerprint,
            q0_status_by_joint=(
                ("joint_0", "INTERSECTING"),
                *fingerprint.q0_status_by_joint[1:],
            ),
        )
        with self.assertRaisesRegex(G4OracleV5Error, "12/12 separated"):
            _validate_replay_fingerprint(bad, profile)


class TestG4G7Comparator(unittest.TestCase):
    def setUp(self) -> None:
        self.endpoint = EndpointSpecV5(
            endpoint_id="lf_hip_joint:min",
            joint_name="lf_hip_joint",
            side="min",
            branch_root_joint_name="lf_hip_joint",
            structural_depth=0,
            motor_id=13,
            motor_direction=-1,
            urdf_lower_rad=-0.8,
            urdf_upper_rad=0.8,
            active_link_pair=("base_link", "lf_hip_link"),
        )
        self.context = {
            "lf_upper_leg_joint": 0.5,
            "lh_upper_leg_joint": 0.3,
        }
        self.g4_record = {
            "endpoint_id": "lf_hip_min",
            "joint_name": "lf_hip_joint",
            "side": "min",
            "urdf_declared_limit_rad": -0.8,
            "active_revolute_pair": ["base_link", "lf_hip_link"],
            "result_kind": "MESH_CONTACT_FOUND",
            "mesh_predicted_contact_rad": -0.85,
            "contact_link_pair": ["base_link", "lf_hip_link"],
            "bracket_clear_rad": -0.849375,
            "bracket_contact_rad": -0.85,
            "delta_from_declared_rad": -0.05,
            "contact_model_status": "PATH_COLLISION_BEFORE_ENDPOINT",
            "prerequisite_pose_rad": {"lf_upper_leg_joint": 0.5},
            "other_legs_pose_rad": {"lh_upper_leg_joint": 0.3},
            "path_collision_angle_rad": -0.84,
            "path_collision_link_pair": ["base_link", "lf_lower_leg_link"],
            "numerical_search": {
                "analysis_envelope_rad": [-0.9, -0.7],
                "coarse_step_rad": 0.01,
                "bisection_resolution_rad": 0.001,
                "bisection_iterations": 4,
            },
        }
        self.task = G4ReplayTask(
            endpoint=self.endpoint,
            context_pose_rad=dict(self.context),
            coarse_step_rad=0.01,
            envelope_margin_rad=0.1,
            bisection_resolution_rad=0.001,
            max_bisection_iterations=40,
            path_domain_mode=PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE,
            g4_record=self.g4_record,
        )
        self.geometry = GeometricEndpointResultV5(
            endpoint=self.endpoint,
            status=GEOMETRIC_CONTACT_FOUND,
            contact_angle_rad=-0.85,
            contact_link_pair=("lf_hip_link", "base_link"),
            declared_limit_delta_rad=-0.05,
            search_start_rad=0.0,
            search_domain_rad=(-0.9, 0.0),
            bracket_clear_rad=-0.849375,
            bracket_contact_rad=-0.85,
            coarse_step_rad=0.01,
            bisection_resolution_rad=0.001,
            max_bisection_iterations=40,
            bisection_iterations=4,
            context_pose_rad=dict(self.context),
        )
        self.path = PathObstructionResultV5(
            endpoint_id=self.endpoint.endpoint_id,
            status=PATH_OBSTRUCTION,
            obstruction_angle_rad=-0.84,
            obstruction_link_pair=("lf_lower_leg_link", "base_link"),
            relation="body_vs_branch",
            search_domain_rad=(-0.9, 0.0),
            bracket_clear_rad=-0.839375,
            bracket_contact_rad=-0.84,
            coarse_step_rad=0.01,
            bisection_resolution_rad=0.001,
            max_bisection_iterations=40,
            bisection_iterations=4,
            context_pose_rad=dict(self.context),
        )
        self.analysis = EndpointAnalysisV5(geometry=self.geometry, path=self.path)

    def _assert_rejected(
        self,
        analysis: EndpointAnalysisV5,
        failed_check: str,
    ) -> None:
        with self.assertRaisesRegex(G4OracleV5Error, failed_check):
            compare_g4_g7((self.task,), (analysis,))

    def test_complete_same_context_replay_comparison_passes(self) -> None:
        comparison = compare_g4_g7((self.task,), (self.analysis,))
        self.assertEqual(comparison["status"], "PASS")
        self.assertEqual(
            comparison["canonical_use"],
            "NON_CANONICAL_REPLAY_EVIDENCE_ONLY",
        )
        self.assertTrue(all(comparison["comparisons"][0]["checks"].values()))
        self.assertEqual(
            comparison["tight_replay_diagnostic"]["max_contact_abs_delta_rad"],
            0.0,
        )

    def test_replay_task_rejects_canonical_direct_path_domain(self) -> None:
        with self.assertRaisesRegex(G4OracleV5Error, "FULL_ENDPOINT_ENVELOPE"):
            replace(
                self.task,
                path_domain_mode=PATH_DOMAIN_DIRECT_TO_GEOMETRIC_TARGET,
            )

    def test_contact_angle_beyond_declared_resolution_is_rejected(self) -> None:
        geometry = replace(self.geometry, contact_angle_rad=-0.848)
        self._assert_rejected(
            replace(self.analysis, geometry=geometry),
            "contact_angle",
        )

    def test_outcome_within_declared_resolution_remains_accepted(self) -> None:
        offset = 0.0005
        geometry = replace(
            self.geometry,
            contact_angle_rad=self.geometry.contact_angle_rad + offset,
            declared_limit_delta_rad=(
                self.geometry.declared_limit_delta_rad + offset
            ),
            bracket_clear_rad=self.geometry.bracket_clear_rad + offset,
            bracket_contact_rad=self.geometry.bracket_contact_rad + offset,
        )
        comparison = compare_g4_g7(
            (self.task,),
            (replace(self.analysis, geometry=geometry),),
        )
        self.assertEqual(comparison["status"], "PASS")
        self.assertAlmostEqual(
            comparison["tight_replay_diagnostic"]["max_contact_abs_delta_rad"],
            offset,
        )

    def test_endpoint_identity_change_is_rejected(self) -> None:
        endpoint = replace(self.endpoint, joint_name="wrong_joint")
        geometry = replace(self.geometry, endpoint=endpoint)
        self._assert_rejected(
            replace(self.analysis, geometry=geometry),
            "endpoint_identity",
        )

    def test_contact_status_change_is_rejected(self) -> None:
        geometry = replace(
            self.geometry,
            status=NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
        )
        self._assert_rejected(
            replace(self.analysis, geometry=geometry),
            "contact_status",
        )

    def test_declared_limit_change_is_rejected(self) -> None:
        endpoint = replace(self.endpoint, urdf_lower_rad=-0.81)
        geometry = replace(self.geometry, endpoint=endpoint)
        self._assert_rejected(
            replace(self.analysis, geometry=geometry),
            "declared_limit",
        )

    def test_active_pair_change_is_rejected(self) -> None:
        endpoint = replace(
            self.endpoint,
            active_link_pair=("base_link", "wrong_link"),
        )
        geometry = replace(self.geometry, endpoint=endpoint)
        self._assert_rejected(
            replace(self.analysis, geometry=geometry),
            "active_pair",
        )

    def test_contact_pair_change_is_rejected(self) -> None:
        geometry = replace(
            self.geometry,
            contact_link_pair=("base_link", "wrong_link"),
        )
        self._assert_rejected(
            replace(self.analysis, geometry=geometry),
            "contact_pair",
        )

    def test_missing_expected_path_analysis_is_rejected(self) -> None:
        self._assert_rejected(
            replace(self.analysis, path=None),
            "path_analysis_present",
        )

    def test_path_pair_change_is_rejected(self) -> None:
        path = replace(
            self.path,
            obstruction_link_pair=("base_link", "wrong_link"),
        )
        self._assert_rejected(replace(self.analysis, path=path), "path_pair")

    def test_contact_bracket_domain_and_bisection_mutations_are_rejected(self) -> None:
        mutations = {
            "contact_bracket_clear": replace(
                self.geometry,
                bracket_clear_rad=-0.847,
            ),
            "contact_bracket_contact": replace(
                self.geometry,
                bracket_contact_rad=-0.847,
            ),
            "contact_search_domain": replace(
                self.geometry,
                search_domain_rad=(-0.91, 0.0),
            ),
            "contact_coarse_step": replace(self.geometry, coarse_step_rad=0.02),
            "contact_bisection_resolution": replace(
                self.geometry,
                bisection_resolution_rad=0.002,
            ),
            "contact_max_bisection_iterations": replace(
                self.geometry,
                max_bisection_iterations=41,
            ),
            "contact_bisection_iterations": replace(
                self.geometry,
                bisection_iterations=3,
            ),
            "declared_limit_delta": replace(
                self.geometry,
                declared_limit_delta_rad=-0.047,
            ),
            "replay_context": replace(self.geometry, context_pose_rad={}),
        }
        for failed_check, geometry in mutations.items():
            with self.subTest(failed_check=failed_check):
                self._assert_rejected(
                    replace(self.analysis, geometry=geometry),
                    failed_check,
                )

    def test_path_bracket_domain_and_bisection_mutations_are_rejected(self) -> None:
        mutations = {
            "path_angle": replace(self.path, obstruction_angle_rad=-0.837),
            "path_bracket_clear": replace(self.path, bracket_clear_rad=-0.837),
            "path_bracket_contact": replace(self.path, bracket_contact_rad=-0.837),
            "path_search_domain": replace(
                self.path,
                search_domain_rad=(-0.91, 0.0),
            ),
            "path_coarse_step": replace(self.path, coarse_step_rad=0.02),
            "path_bisection_resolution": replace(
                self.path,
                bisection_resolution_rad=0.002,
            ),
            "path_max_bisection_iterations": replace(
                self.path,
                max_bisection_iterations=41,
            ),
            "path_bisection_iterations": replace(
                self.path,
                bisection_iterations=3,
            ),
            "path_context": replace(self.path, context_pose_rad={}),
        }
        for failed_check, path in mutations.items():
            with self.subTest(failed_check=failed_check):
                self._assert_rejected(
                    replace(self.analysis, path=path),
                    failed_check,
                )

    def test_renderer_handles_legitimate_no_contact_record(self) -> None:
        g4_record = copy.deepcopy(self.g4_record)
        g4_record.update(
            {
                "result_kind": "NO_MESH_CONTACT_IN_ENVELOPE",
                "mesh_predicted_contact_rad": None,
                "contact_link_pair": None,
                "bracket_clear_rad": None,
                "bracket_contact_rad": None,
                "delta_from_declared_rad": None,
                "contact_model_status": "NO_MODELED_ENDSTOP",
                "path_collision_angle_rad": None,
                "path_collision_link_pair": None,
            }
        )
        g4_record["numerical_search"]["bisection_iterations"] = 0
        task = replace(self.task, g4_record=g4_record)
        geometry = replace(
            self.geometry,
            status=NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
            contact_angle_rad=None,
            contact_link_pair=self.endpoint.active_link_pair,
            declared_limit_delta_rad=None,
            bracket_clear_rad=None,
            bracket_contact_rad=None,
            bisection_iterations=0,
        )
        path = replace(
            self.path,
            status=NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
            obstruction_angle_rad=None,
            obstruction_link_pair=None,
            relation=None,
            bracket_clear_rad=None,
            bracket_contact_rad=None,
            bisection_iterations=0,
        )
        comparison = compare_g4_g7(
            (task,),
            (EndpointAnalysisV5(geometry=geometry, path=path),),
        )
        report = render_g4_g7_report(comparison)
        self.assertIn("| lf_hip_joint:min | - | - | - | - | PASS |", report)
        self.assertIn("geometric contacts: 0", report)
        self.assertIn("NON_CANONICAL_REPLAY_EVIDENCE_ONLY", report)

    def test_dedicated_runner_returns_evidence_only_with_toctou_provenance(self) -> None:
        frozen = json.loads(FROZEN_G4.read_text(encoding="utf-8"))
        fingerprint = GeometryInputFingerprintV5(
            urdf_sha256=frozen["urdf"]["sha256"],
            collision_mesh_sha256=tuple(
                (link_name, record["sha256"])
                for link_name, record in frozen["collision_mesh_manifest"].items()
            ),
            q0_status_by_joint=tuple(
                (f"joint_{index:02d}", "SEPARATED_NARROW") for index in range(12)
            ),
        )
        fake_scene = MagicMock()
        fake_scene.model.actuated_joint_names = tuple(
            name for name, _status in fingerprint.q0_status_by_joint
        )
        tasks = tuple(self.task for _index in range(24))
        analyses = tuple(self.analysis for _index in range(24))
        source_manifest = {"matdog_geometry_g4_oracle_v5.py": "a" * 64}
        comparison = {
            "status": "PASS",
            "artifact_role": "NONCANONICAL_G4_REPLAY_ORACLE",
            "canonical_profile_eligible": False,
        }

        with (
            patch(
                "matdog_geometry_g4_oracle_v5.geometry_source_manifest",
                return_value=source_manifest,
            ) as source_manifest_mock,
            patch(
                "matdog_geometry_g4_oracle_v5.RobotSceneV5.from_urdf",
                return_value=fake_scene,
            ),
            patch(
                "matdog_geometry_g4_oracle_v5.scene_input_fingerprint",
                return_value=fingerprint,
            ),
            patch(
                "matdog_geometry_g4_oracle_v5.load_endpoint_specs",
                return_value=tuple(self.endpoint for _index in range(24)),
            ),
            patch(
                "matdog_geometry_g4_oracle_v5.build_g4_replay_tasks",
                return_value=tasks,
            ),
            patch(
                "matdog_geometry_g4_oracle_v5.execute_contact_tasks",
                return_value=analyses,
            ) as execute_mock,
            patch(
                "matdog_geometry_g4_oracle_v5.compare_g4_g7",
                return_value=comparison,
            ),
            patch("matdog_geometry_g4_oracle_v5.clear_mesh_cache"),
        ):
            result = run_g4_replay_v5(
                repo_root=REPO_ROOT,
                urdf_path=REPO_ROOT
                / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf",
                g4_profile_path=FROZEN_G4,
                workers=1,
                source_files=("matdog_geometry_g4_oracle_v5.py",),
            )

        self.assertIsInstance(result, G4ReplayRunV5)
        self.assertEqual(
            {field.name for field in fields(G4ReplayRunV5)},
            {"analyses", "comparison", "runtime_seconds"},
        )
        self.assertNotIn("profile", result.__dict__)
        self.assertEqual(
            result.comparison["artifact_role"],
            "NONCANONICAL_G4_REPLAY_ORACLE",
        )
        self.assertIs(result.comparison["canonical_profile_eligible"], False)
        self.assertEqual(
            result.comparison["replay_execution_provenance"]["q0_status_by_joint"],
            dict(fingerprint.q0_status_by_joint),
        )
        self.assertEqual(source_manifest_mock.call_count, 2)
        execute_mock.assert_called_once()
        self.assertTrue(
            all(
                task.path_domain_mode == PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE
                for task in execute_mock.call_args.args[3]
            )
        )


if __name__ == "__main__":
    unittest.main()
