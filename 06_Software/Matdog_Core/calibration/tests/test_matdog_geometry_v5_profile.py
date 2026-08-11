#!/usr/bin/env python3
"""G6 tests for the pure schema-v5 profile and semantic hash."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import hashlib
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
URDF_PATH = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
G4_PROFILE = (
    REPO_ROOT
    / "09_Logs/Validation_Reports/Geometry_Compiler"
    / "2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json"
)

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_contact_search_v5 import (  # noqa: E402
    GEOMETRIC_CONTACT_FOUND,
    NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
    PATH_OBSTRUCTION,
    EndpointAnalysisV5,
    GeometricEndpointResultV5,
    PathObstructionResultV5,
    load_endpoint_specs,
)
import matdog_geometry_profile_v5 as profile_module  # noqa: E402
from matdog_geometry_profile_v5 import (  # noqa: E402
    GeometryProfileV5Error,
    SCHEMA_VERSION,
    build_geometry_profile_v5,
    combined_source_sha256,
    find_geometry_mismatches_v5,
    semantic_content_sha256,
    validate_pure_geometry_profile,
)
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ProfileFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = RobotSceneV5.from_urdf(URDF_PATH)
        cls.specs = load_endpoint_specs(cls.scene.model)

    def fake_analyses(self) -> tuple[EndpointAnalysisV5, ...]:
        analyses = []
        for index, endpoint in enumerate(self.specs):
            sign = -1.0 if endpoint.side == "min" else 1.0
            angle = endpoint.urdf_declared_limit_rad + sign * math.radians(0.5)
            domain_end = endpoint.urdf_declared_limit_rad + sign * math.radians(10.0)
            domain = (min(0.0, domain_end), max(0.0, domain_end))
            geometry = GeometricEndpointResultV5(
                endpoint=endpoint,
                status=GEOMETRIC_CONTACT_FOUND,
                contact_angle_rad=angle,
                contact_link_pair=endpoint.active_link_pair,
                declared_limit_delta_rad=angle - endpoint.urdf_declared_limit_rad,
                search_start_rad=0.0,
                search_domain_rad=domain,
                bracket_clear_rad=angle - sign * 0.00005,
                bracket_contact_rad=angle,
                coarse_step_rad=math.radians(1.0),
                bisection_resolution_rad=0.0001,
                max_bisection_iterations=40,
                bisection_iterations=8,
                context_pose_rad={},
            )
            if index == 0:
                branch = self.scene.model.articulated_branches[0]
                pair = (self.scene.model.root_link, branch.link_names[-1])
                path = PathObstructionResultV5(
                    endpoint_id=endpoint.endpoint_id,
                    status=PATH_OBSTRUCTION,
                    obstruction_angle_rad=angle * 0.9,
                    obstruction_link_pair=pair,
                    relation=self.scene.model.pair_relation(*pair),
                    search_domain_rad=domain,
                    bracket_clear_rad=angle * 0.9 - sign * 0.00005,
                    bracket_contact_rad=angle * 0.9,
                    coarse_step_rad=math.radians(1.0),
                    bisection_resolution_rad=0.0001,
                    max_bisection_iterations=40,
                    bisection_iterations=8,
                    context_pose_rad={},
                )
            else:
                path = PathObstructionResultV5(
                    endpoint_id=endpoint.endpoint_id,
                    status=NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
                    obstruction_angle_rad=None,
                    obstruction_link_pair=None,
                    relation=None,
                    search_domain_rad=domain,
                    bracket_clear_rad=None,
                    bracket_contact_rad=None,
                    coarse_step_rad=math.radians(1.0),
                    bisection_resolution_rad=0.0001,
                    max_bisection_iterations=40,
                    bisection_iterations=0,
                    context_pose_rad={},
                )
            analyses.append(EndpointAnalysisV5(geometry=geometry, path=path))
        return tuple(analyses)

    def build(
        self,
        *,
        workers: int = 1,
        runtime: float = 1.0,
        analyses: tuple[EndpointAnalysisV5, ...] | None = None,
    ):
        return build_geometry_profile_v5(
            self.scene,
            self.fake_analyses() if analyses is None else analyses,
            repo_root=REPO_ROOT,
            worker_count=workers,
            runtime_seconds=runtime,
        )


class TestPureProfile(ProfileFixture):
    def test_schema_and_complete_model_provenance(self):
        profile = self.build()
        self.assertEqual(profile["schema_version"], SCHEMA_VERSION)
        self.assertEqual(profile["model"]["selected_actuated_joint_count"], 12)
        self.assertEqual(len(profile["endpoint_searches"]), 24)
        base_mesh = profile["provenance"]["collision_meshes"]["base_link"]
        self.assertEqual(base_mesh["scale_xyz"], [0.001, 0.001, 0.001])
        self.assertEqual(base_mesh["origin_xyz"], [0.0, 0.0, 0.0])
        self.assertEqual(base_mesh["origin_rpy"], [0.0, 0.0, 0.0])
        joints = profile["model"]["joints"]
        self.assertEqual(sum(joint["selected_actuated"] for joint in joints), 12)
        self.assertTrue(all("motor_direction" in joint for joint in joints))
        self.assertNotIn("repository", profile["provenance"])
        repository = profile["generation_metadata"]["repository_materialization"]
        self.assertEqual(set(repository), {"commit_sha", "working_tree_dirty"})

    def test_all_24_canonical_endpoint_contexts_are_explicitly_empty(self):
        profile = self.build()
        contexts = [
            record["search_context"]["joint_positions_rad"]
            for record in profile["endpoint_searches"]
        ]
        self.assertEqual(len(contexts), 24)
        self.assertTrue(all(context == {} for context in contexts))

    def test_hardware_and_safety_fields_are_absent(self):
        profile = self.build()
        text = repr(profile)
        for forbidden in (
            "LF_V25_HARDWARE_EVIDENCE",
            "HARDWARE_CONFIRMED_CONTACT",
            "HARDWARE_CONTRADICTED",
            "mesh_vs_hardware_status",
            "min_clearance_pass_m",
            "parking_seed_angles_deg",
        ):
            self.assertNotIn(forbidden, text)

    def test_path_obstruction_is_not_automatically_cross_branch(self):
        profile = self.build()
        obstruction = profile["endpoint_searches"][0]["path_obstruction"]
        self.assertEqual(obstruction["status"], PATH_OBSTRUCTION)
        self.assertEqual(obstruction["relation"], "body_vs_branch")
        self.assertNotEqual(obstruction["relation"], "cross_branch")

    def test_forbidden_v4_field_is_rejected(self):
        profile = self.build()
        profile["endpoint_searches"][0]["hardware_vs_urdf_status"] = "COMPATIBLE"
        profile["semantic_content_sha256"] = semantic_content_sha256(profile)
        with self.assertRaisesRegex(GeometryProfileV5Error, "hardware/safety/policy"):
            validate_pure_geometry_profile(profile)


class TestSemanticDeterminism(ProfileFixture):
    def test_repository_dirty_audit_distinguishes_clean_dirty_and_git_failure(self):
        cases = (
            (SimpleNamespace(returncode=0, stdout="", stderr=""), False),
            (SimpleNamespace(returncode=0, stdout=" M source.py\n", stderr=""), True),
            (SimpleNamespace(returncode=1, stdout="", stderr="failure"), None),
        )
        for completed, expected in cases:
            with self.subTest(expected=expected):
                with patch.object(
                    profile_module.subprocess,
                    "run",
                    return_value=completed,
                ):
                    self.assertIs(profile_module._git_dirty(REPO_ROOT), expected)

    def test_worker_runtime_and_timestamp_metadata_do_not_change_semantic_hash(self):
        profile_1 = self.build(workers=1, runtime=12.5)
        profile_4 = self.build(workers=4, runtime=3.25)
        profile_4["generation_metadata"]["generated_at_utc"] = "different"
        self.assertEqual(
            profile_1["semantic_content_sha256"],
            profile_4["semantic_content_sha256"],
        )
        self.assertEqual(
            semantic_content_sha256(profile_1),
            semantic_content_sha256(profile_4),
        )

    def test_repository_materialization_metadata_is_nonsemantic(self):
        profile = self.build()
        changed = deepcopy(profile)
        changed["generation_metadata"]["repository_materialization"] = {
            "commit_sha": "f" * 40,
            "working_tree_dirty": not profile["generation_metadata"][
                "repository_materialization"
            ]["working_tree_dirty"],
        }
        self.assertEqual(
            profile["semantic_content_sha256"],
            semantic_content_sha256(changed),
        )
        validate_pure_geometry_profile(changed)

    def test_semantic_source_hash_change_changes_hash_and_fails_live_provenance(self):
        profile = self.build()
        changed = deepcopy(profile)
        source_manifest = changed["provenance"]["geometry_compiler"][
            "source_file_sha256"
        ]
        first_source = sorted(source_manifest)[0]
        source_manifest[first_source] = "0" * 64
        changed["provenance"]["geometry_compiler"][
            "source_combined_sha256"
        ] = combined_source_sha256(source_manifest)
        self.assertNotEqual(
            profile["semantic_content_sha256"],
            semantic_content_sha256(changed),
        )
        changed["semantic_content_sha256"] = semantic_content_sha256(changed)
        self.assertIn(
            "pure geometry source manifest mismatch",
            find_geometry_mismatches_v5(changed, self.scene, repo_root=REPO_ROOT),
        )

    def test_semantic_geometry_change_changes_hash(self):
        profile = self.build()
        changed = deepcopy(profile)
        changed["endpoint_searches"][0]["geometric_contact"]["angle_rad"] += 0.001
        self.assertNotEqual(
            semantic_content_sha256(profile),
            semantic_content_sha256(changed),
        )


class TestProfileWideAnalysisParameters(ProfileFixture):
    def _replace_result_parameter(
        self,
        analyses: tuple[EndpointAnalysisV5, ...],
        *,
        layer: str,
        field: str,
    ) -> tuple[EndpointAnalysisV5, ...]:
        changed = list(analyses)
        analysis = changed[1]
        result = analysis.geometry if layer == "geometric_contact" else analysis.path
        assert result is not None
        if field == "envelope_margin_rad":
            domain = list(result.search_domain_rad)
            domain[0 if analysis.geometry.endpoint.side == "min" else 1] += 0.01
            updated_result = replace(result, search_domain_rad=tuple(domain))
        else:
            delta = 1 if field == "max_bisection_iterations" else 0.01
            updated_result = replace(result, **{field: getattr(result, field) + delta})
        changed[1] = (
            replace(analysis, geometry=updated_result)
            if layer == "geometric_contact"
            else replace(analysis, path=updated_result)
        )
        return tuple(changed)

    def test_every_profile_wide_search_parameter_must_be_uniform(self):
        analyses = self.fake_analyses()
        fields_by_layer = {
            "geometric_contact": (
                "coarse_step_rad",
                "envelope_margin_rad",
                "bisection_resolution_rad",
                "max_bisection_iterations",
            ),
            "path_obstruction": (
                "coarse_step_rad",
                "bisection_resolution_rad",
                "max_bisection_iterations",
            ),
        }
        for layer, fields in fields_by_layer.items():
            for field in fields:
                with self.subTest(layer=layer, field=field):
                    changed = self._replace_result_parameter(
                        analyses,
                        layer=layer,
                        field=field,
                    )
                    with self.assertRaisesRegex(
                        GeometryProfileV5Error,
                        rf"STOP: profile-wide analysis parameter mismatch.*{field}",
                    ):
                        self.build(analyses=changed)

    def test_per_endpoint_direct_path_domain_is_not_misreported_as_envelope_margin(self):
        analyses = self.fake_analyses()
        changed = self._replace_result_parameter(
            analyses,
            layer="path_obstruction",
            field="envelope_margin_rad",
        )
        profile = self.build(analyses=changed)
        self.assertEqual(
            profile["analysis_parameters"]["canonical_path_domain"],
            "Q0_TO_GEOMETRIC_CONTACT_OR_DECLARED_LIMIT",
        )


class TestV5StaleDetection(ProfileFixture):
    def test_fresh_profile_matches_current_model_geometry_and_sources(self):
        profile = self.build()
        self.assertEqual(
            find_geometry_mismatches_v5(profile, self.scene, repo_root=REPO_ROOT),
            [],
        )

    def test_scale_or_origin_tamper_is_detected(self):
        profile = self.build()
        profile["provenance"]["collision_meshes"]["base_link"]["scale_xyz"][0] = 0.002
        profile["semantic_content_sha256"] = semantic_content_sha256(profile)
        mismatches = find_geometry_mismatches_v5(profile, self.scene, repo_root=REPO_ROOT)
        self.assertIn("collision mesh path/hash/count/scale/origin manifest mismatch", mismatches)

    def test_frozen_g4_profile_file_is_unchanged(self):
        self.assertEqual(
            _sha(G4_PROFILE),
            "f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa",
        )


if __name__ == "__main__":
    unittest.main()
