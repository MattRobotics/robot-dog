"""Test automatici MATDOG per la serializzazione del profilo geometrico."""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_contact_search import EndpointSpec, EndpointContactResult  # noqa: E402
from matdog_geometry_path_planner import LegSequenceResult, ParkingPlan, PathSegmentResult  # noqa: E402
from matdog_geometry_profile import (  # noqa: E402
    build_geometry_profile,
    content_sha256,
    find_geometry_mismatches,
    geometry_compiler_source_hash,
)
from matdog_geometry_scene import RobotScene  # noqa: E402
from matdog_geometry_uncertainty import ManufacturingToleranceInputs  # noqa: E402


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf").is_file():
            return parent
    raise RuntimeError("Repository root non trovato")


REPO_ROOT = _repo_root()


def _fake_endpoint(leg: str, joint_group: str, side: str) -> EndpointSpec:
    return EndpointSpec(
        leg=leg,
        joint_group=joint_group,
        side=side,
        joint_name=f"{leg}_{joint_group}_joint",
        servo_id=13,
        urdf_lower_rad=-0.5,
        urdf_upper_rad=0.5,
        prerequisite_overrides={f"{leg}_hip_joint": 0.0, f"{leg}_lower_leg_joint": 0.0},
    )


def _fake_result(endpoint: EndpointSpec) -> EndpointContactResult:
    return EndpointContactResult(
        endpoint=endpoint,
        result_kind="MESH_CONTACT_FOUND",
        mesh_predicted_contact_rad=0.4,
        contact_link_a=f"{endpoint.leg}_lower_leg_link",
        contact_link_b="base_link",
        clearance_before_contact_m=1e-5,
        bracket_clear_rad=0.399,
        bracket_contact_rad=0.4,
        coarse_step_rad=0.017,
        bisection_resolution_rad=0.0001,
        bisection_iterations=12,
        analysis_envelope_rad=(0.3, 0.7),
        model_limit_mismatch=False,
        delta_from_declared_rad=-0.1,
        other_legs_pose={},
        contact_model_status="MODELED_ENDSTOP_CONTACT",
        contact_model_status_reason="fake: within mismatch threshold",
        path_collision_angle_rad=None,
        path_collision_link_a=None,
        path_collision_link_b=None,
        hardware_evidence_note=None,
        hardware_vs_urdf_status="NOT_AVAILABLE",
        mesh_vs_hardware_status="NOT_AVAILABLE",
    )


def _fake_parking_plan(leg: str, endpoint: EndpointSpec) -> ParkingPlan:
    segment = PathSegmentResult(
        description="fake segment",
        joint_name=endpoint.joint_name,
        start_rad=0.0,
        end_rad=0.4,
        other_legs_pose={},
        passed=True,
        min_clearance_m=0.01,
        min_clearance_kind="EXACT",
        clearance_gate_result="PASS",
        first_collision_angle_rad=None,
        first_collision_pair=None,
        sample_count=5,
    )
    sequence = LegSequenceResult(leg=leg, other_legs_pose={}, segments=(segment,))
    return ParkingPlan(
        leg=leg,
        required=False,
        reason="fake: path collision-free",
        parked_leg=None,
        parking_angle_rad=None,
        park_path=None,
        active_leg_sequence=sequence,
    )


class TestContactModelStatusRepresentable(unittest.TestCase):
    """Item G: every contact_model_status taxonomy value (canonical
    handoff reconciliation section 10) must round-trip through the
    serialized profile without being coerced/lost."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def test_every_taxonomy_value_round_trips(self):
        for status in (
            "MODELED_ENDSTOP_CONTACT",
            "NO_MODELED_ENDSTOP",
            "MODEL_INCOMPLETE",
            "PATH_COLLISION_BEFORE_ENDPOINT",
            "UNINTENDED_SELF_COLLISION",
            "MODEL_LIMIT_MISMATCH",
        ):
            with self.subTest(status=status):
                endpoint = _fake_endpoint("lf", "hip", "max")
                result = _fake_result(endpoint)
                result = EndpointContactResult(
                    **{**result.__dict__, "contact_model_status": status}
                )
                plan = _fake_parking_plan("lf", endpoint)

                profile = build_geometry_profile(
                    self.scene,
                    endpoint_results=[result],
                    sensitivity_by_endpoint={},
                    parking_plans={"lf": plan},
                    tolerance_inputs=ManufacturingToleranceInputs(),
                    numerical_parameters={},
                    unresolved_assumptions=[],
                )

                self.assertEqual(profile["endpoints"][0]["contact_model_status"], status)


class TestProfileDeterminism(unittest.TestCase):
    """Item D: same input -> same technical geometry profile, excluding
    non-semantic timestamps."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def _build(self) -> dict:
        endpoint = _fake_endpoint("lf", "hip", "max")
        result = _fake_result(endpoint)
        plan = _fake_parking_plan("lf", endpoint)

        return build_geometry_profile(
            self.scene,
            endpoint_results=[result],
            sensitivity_by_endpoint={},
            parking_plans={"lf": plan},
            tolerance_inputs=ManufacturingToleranceInputs(),
            numerical_parameters={"coarse_step_rad": 0.017, "example": 1},
            unresolved_assumptions=["note a", "note b"],
        )

    def test_content_sha256_identical_across_two_builds(self):
        profile_1 = self._build()
        profile_2 = self._build()

        self.assertNotEqual(profile_1["generation_timestamp_utc"], "")
        self.assertEqual(profile_1["content_sha256"], profile_2["content_sha256"])

        # And the timestamp field is legitimately allowed to differ / be
        # the only difference between two otherwise-identical builds.
        stripped_1 = {k: v for k, v in profile_1.items() if k != "generation_timestamp_utc"}
        stripped_2 = {k: v for k, v in profile_2.items() if k != "generation_timestamp_utc"}
        self.assertEqual(stripped_1, stripped_2)

    def test_content_sha256_changes_when_content_changes(self):
        profile_1 = self._build()
        profile_2 = copy.deepcopy(profile_1)
        profile_2["endpoints"][0]["mesh_predicted_contact_rad"] = 0.999
        profile_2["content_sha256"] = content_sha256(profile_2)

        self.assertNotEqual(profile_1["content_sha256"], profile_2["content_sha256"])

    def test_source_hash_is_stable_for_unchanged_source(self):
        self.assertEqual(
            geometry_compiler_source_hash(REPO_ROOT),
            geometry_compiler_source_hash(REPO_ROOT),
        )


class TestHashPinningAndStaleDetection(unittest.TestCase):
    """Items J and K: URDF/mesh manifest hash pinning and stale-profile
    mismatch detection against real current geometry."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def _build(self) -> dict:
        endpoint = _fake_endpoint("rh", "lower_leg", "min")
        result = _fake_result(endpoint)
        plan = _fake_parking_plan("rh", endpoint)

        return build_geometry_profile(
            self.scene,
            endpoint_results=[result],
            sensitivity_by_endpoint={},
            parking_plans={"rh": plan},
            tolerance_inputs=ManufacturingToleranceInputs(),
            numerical_parameters={},
            unresolved_assumptions=[],
        )

    def test_urdf_hash_matches_real_file(self):
        import hashlib

        profile = self._build()
        expected = hashlib.sha256(self.scene.urdf_path.read_bytes()).hexdigest()
        self.assertEqual(profile["urdf"]["sha256"], expected)

    def test_mesh_manifest_hash_matches_real_mesh(self):
        profile = self._build()
        entry = profile["collision_mesh_manifest"]["lf_foot_link"]
        self.assertEqual(entry["sha256"], self.scene.mesh("lf_foot_link").sha256)

    def test_fresh_profile_has_no_mismatches_against_current_geometry(self):
        profile = self._build()
        mismatches = find_geometry_mismatches(profile, self.scene)
        self.assertEqual(mismatches, [])

    def test_tampered_urdf_hash_is_detected_as_mismatch(self):
        profile = self._build()
        profile["urdf"]["sha256"] = "0" * 64

        mismatches = find_geometry_mismatches(profile, self.scene)
        self.assertTrue(any("URDF sha256 mismatch" in m for m in mismatches))

    def test_tampered_mesh_hash_is_detected_as_mismatch(self):
        profile = self._build()
        profile["collision_mesh_manifest"]["lf_foot_link"]["sha256"] = "0" * 64

        mismatches = find_geometry_mismatches(profile, self.scene)
        self.assertTrue(any("lf_foot_link" in m and "sha256 mismatch" in m for m in mismatches))

    def test_missing_mesh_manifest_entry_is_detected(self):
        profile = self._build()
        del profile["collision_mesh_manifest"]["lf_foot_link"]

        mismatches = find_geometry_mismatches(profile, self.scene)
        self.assertTrue(any("lf_foot_link" in m and "missing from profile" in m for m in mismatches))


if __name__ == "__main__":
    unittest.main()
