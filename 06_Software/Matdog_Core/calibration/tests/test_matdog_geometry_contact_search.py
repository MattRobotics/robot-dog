"""Test automatici MATDOG per la ricerca del contatto per-endpoint."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_contact_search import (  # noqa: E402
    EndpointSpec,
    LF_V25_HARDWARE_EVIDENCE,
    _classify_contact_model_status,
    bisect_collision_boundary,
    bracket_collision_boundary,
    load_all_endpoints,
    search_endpoint_contact,
)
from matdog_geometry_mesh_kernel import (  # noqa: E402
    check_pair,
    identity_transform,
    make_transform,
)
from matdog_geometry_scene import ALL_JOINT_NAMES, RobotScene  # noqa: E402


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf").is_file():
            return parent
    raise RuntimeError("Repository root non trovato")


REPO_ROOT = _repo_root()


class TestSyntheticBracketAndBisect(unittest.TestCase):
    """Item C of the Phase 1 test contract: contact bracketing/refinement
    converges to a known synthetic clear -> contact boundary.

    A small box translates along +X toward a fixed wall box; the exact
    crossing point (where the boxes first touch) is computable in closed
    form, so bisection convergence can be checked against ground truth
    rather than another part of this same codebase.
    """

    def test_bisection_converges_to_analytical_contact_point(self):
        from tests.test_matdog_geometry_mesh_kernel import _box_triangles, _mesh_from_triangles  # noqa: E402

        moving = _mesh_from_triangles("moving", _box_triangles((-0.01, -0.01, -0.01), (0.01, 0.01, 0.01)))
        wall = _mesh_from_triangles("wall", _box_triangles((0.05, -0.05, -0.05), (0.15, 0.05, 0.05)))
        wall_transform = identity_transform()

        # Exact analytical touching point: moving box's +X face (at
        # translation + 0.01) meets the wall's -X face (0.05) when
        # translation == 0.04.
        true_contact_translation = 0.04

        def collide_fn(translation: float) -> bool:
            transform = make_transform(np.eye(3), (translation, 0.0, 0.0))
            result = check_pair(moving, transform, wall, wall_transform)
            return result.status == "INTERSECTING"

        self.assertFalse(collide_fn(0.0))
        self.assertTrue(collide_fn(0.06))

        bracket = bracket_collision_boundary(collide_fn, 0.0, 1.0, 0.06, 0.005)
        self.assertIsNotNone(bracket)
        clear_value, contact_value = bracket
        self.assertLessEqual(clear_value, true_contact_translation)
        self.assertGreaterEqual(contact_value, true_contact_translation)

        resolution = 1e-6
        clear_value, contact_value, iterations = bisect_collision_boundary(
            collide_fn, clear_value, contact_value, resolution, max_iterations=40
        )

        self.assertLessEqual(contact_value - clear_value, resolution)
        self.assertAlmostEqual(clear_value, true_contact_translation, delta=resolution)
        self.assertAlmostEqual(contact_value, true_contact_translation, delta=resolution)
        self.assertGreater(iterations, 0)
        self.assertFalse(collide_fn(clear_value))
        self.assertTrue(collide_fn(contact_value))

    def test_bracket_returns_none_when_fully_collision_free(self):
        def never_collides(_value: float) -> bool:
            return False

        result = bracket_collision_boundary(never_collides, 0.0, 1.0, 1.0, 0.1)
        self.assertIsNone(result)


class TestEndpointCoverage(unittest.TestCase):
    """Item E: 24 endpoint coverage, no endpoint missing or duplicated."""

    def test_exactly_24_unique_endpoints(self):
        endpoints = load_all_endpoints(REPO_ROOT)
        self.assertEqual(len(endpoints), 24)

        ids = [e.endpoint_id for e in endpoints]
        self.assertEqual(len(set(ids)), 24)

        legs = {"lf", "rf", "rh", "lh"}
        groups = {"hip", "upper_leg", "lower_leg"}
        sides = {"min", "max"}

        expected_ids = {f"{leg}_{group}_{side}" for leg in legs for group in groups for side in sides}
        self.assertEqual(set(ids), expected_ids)

    def test_every_joint_appears_in_exactly_two_endpoints(self):
        endpoints = load_all_endpoints(REPO_ROOT)
        joint_counts: dict[str, int] = {}

        for endpoint in endpoints:
            joint_counts[endpoint.joint_name] = joint_counts.get(endpoint.joint_name, 0) + 1

        self.assertEqual(set(joint_counts), set(ALL_JOINT_NAMES))
        self.assertTrue(all(count == 2 for count in joint_counts.values()))

    def test_prerequisite_overrides_cover_the_other_two_joints_only(self):
        endpoints = load_all_endpoints(REPO_ROOT)

        for endpoint in endpoints:
            self.assertEqual(len(endpoint.prerequisite_overrides), 2)
            self.assertNotIn(endpoint.joint_name, endpoint.prerequisite_overrides)


class TestModelLimitMismatch(unittest.TestCase):
    """Item L: MODEL_LIMIT_MISMATCH classification, exercised against the
    real RF hip endpoint using a deliberately wrong declared limit (the
    real contact behind base_link<->rf_lower_leg_link near hip=-45deg with
    the LOWER prerequisite is a known, checkpoint-documented, real mesh
    contact -- see MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md).

    Deliberately uses RF, not LF: RF has no LF_V25_HARDWARE_EVIDENCE
    entry, so this exercises the "no hardware oracle" classification path
    (mesh-vs-declared-URDF only) that MODEL_LIMIT_MISMATCH is reserved
    for. Reusing an LF endpoint_id here would instead exercise the
    mesh-vs-hardware path (LF has a real oracle for every endpoint,
    keyed only by leg/joint_group/side, independent of whatever
    urdf_lower/upper_rad this test supplies) and could legitimately
    resolve to MODELED_ENDSTOP_CONTACT or MODEL_INCOMPLETE instead,
    which is a different test than the one this class is named for."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)

    def test_deliberately_wrong_declared_limit_is_flagged_as_mismatch(self):
        real_endpoints = {e.endpoint_id: e for e in load_all_endpoints(REPO_ROOT)}
        real_hip_min = real_endpoints["rf_hip_min"]

        wrong_endpoint = EndpointSpec(
            leg="rf",
            joint_group="hip",
            side="min",
            joint_name=real_hip_min.joint_name,
            servo_id=real_hip_min.servo_id,
            urdf_lower_rad=math.radians(-30.0),  # deliberately far from the real ~-45deg contact
            urdf_upper_rad=real_hip_min.urdf_upper_rad,
            prerequisite_overrides={
                "rf_upper_leg_joint": math.radians(90.0),  # LOWER prerequisite, matches checkpoint contact
                "rf_lower_leg_joint": 0.0,
            },
        )

        result = search_endpoint_contact(
            self.scene, wrong_endpoint, envelope_margin_rad=math.radians(20.0)
        )

        self.assertEqual(result.result_kind, "MESH_CONTACT_FOUND")
        self.assertTrue(result.model_limit_mismatch)
        self.assertIsNotNone(result.delta_from_declared_rad)
        self.assertGreater(abs(result.delta_from_declared_rad), math.radians(2.0))
        # Item F: a mesh contact far beyond the declared limit must never
        # be auto-promoted to MODELED_ENDSTOP_CONTACT just because it
        # exists -- and with no hardware oracle for RF, the only
        # reachable non-endpoint status is MODEL_LIMIT_MISMATCH (never
        # MODEL_INCOMPLETE, which is reserved for legs with a hardware
        # oracle proving disagreement).
        self.assertEqual(result.contact_model_status, "MODEL_LIMIT_MISMATCH")
        self.assertEqual(result.hardware_vs_urdf_status, "NOT_AVAILABLE")
        self.assertEqual(result.mesh_vs_hardware_status, "NOT_AVAILABLE")


class TestClassifyContactModelStatusUnintendedSelfCollision(unittest.TestCase):
    """Item E: a same-leg contact found suspiciously close to home (within
    2 coarse steps) must not be auto-promoted to a designed endpoint --
    pure unit test of the classifier, no scene/geometry needed."""

    def test_contact_immediately_at_home_is_flagged_unintended_not_endpoint(self):

        endpoint = EndpointSpec(
            leg="lf",
            joint_group="hip",
            side="max",
            joint_name="lf_hip_joint",
            servo_id=13,
            urdf_lower_rad=-0.785398163397,
            urdf_upper_rad=0.785398163397,
            prerequisite_overrides={},
        )
        coarse_step = math.radians(1.0)

        status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
            endpoint,
            same_leg_found=True,
            mesh_contact_rad=coarse_step * 1.5,  # well within 2 coarse steps of home
            delta_rad=coarse_step * 1.5 - endpoint.urdf_upper_rad,
            mismatch=True,
            coarse_step_rad=coarse_step,
            path_collision_angle_rad=None,
            model_limit_mismatch_threshold_deg=2.0,
        )

        self.assertEqual(status, "UNINTENDED_SELF_COLLISION")

    def test_contact_well_past_home_is_not_flagged_unintended(self):

        endpoint = EndpointSpec(
            leg="rh",
            joint_group="lower_leg",
            side="min",
            joint_name="rh_lower_leg_joint",
            servo_id=31,
            urdf_lower_rad=-1.605702911835,
            urdf_upper_rad=0.654498469498,
            prerequisite_overrides={},
        )
        coarse_step = math.radians(1.0)

        status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
            endpoint,
            same_leg_found=True,
            mesh_contact_rad=endpoint.urdf_lower_rad - 0.001,  # near declared, far from home
            delta_rad=-0.001,
            mismatch=False,
            coarse_step_rad=coarse_step,
            path_collision_angle_rad=None,
            model_limit_mismatch_threshold_deg=2.0,
        )

        self.assertEqual(status, "MODELED_ENDSTOP_CONTACT")
        self.assertIsNone(hw_note)  # rh has no V25 hardware evidence
        self.assertEqual(hw_vs_urdf, "NOT_AVAILABLE")
        self.assertEqual(mesh_vs_hw, "NOT_AVAILABLE")


class TestPathCollisionVsEndpointClassification(unittest.TestCase):
    """Item B: a cross-leg path obstruction (rear leg at home) must be
    classified as PATH_COLLISION_BEFORE_ENDPOINT, never conflated with
    this joint's own designed endpoint -- reproduces the reconciliation
    finding for LF/RF upper MAX against the ipsilateral rear leg's foot
    at home (2026-07-20 checkpoint's documented ~74-87deg collision
    zone), which the pre-reconciliation code mis-tagged as if it were
    this joint's own mesh limit."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)
        cls.endpoints = {e.endpoint_id: e for e in load_all_endpoints(REPO_ROOT)}

    def test_lf_upper_max_with_lh_at_home_is_path_collision_not_endpoint(self):
        endpoint = self.endpoints["lf_upper_leg_max"]
        result = search_endpoint_contact(self.scene, endpoint, other_legs_pose={})

        self.assertEqual(result.contact_model_status, "PATH_COLLISION_BEFORE_ENDPOINT")
        self.assertIsNotNone(result.path_collision_link_a)
        self.assertIsNotNone(result.path_collision_link_b)
        pair = {result.path_collision_link_a, result.path_collision_link_b}
        self.assertTrue(any(link.startswith("lh_") for link in pair))

    def test_rf_upper_max_with_rh_at_home_is_path_collision_not_endpoint(self):
        endpoint = self.endpoints["rf_upper_leg_max"]
        result = search_endpoint_contact(self.scene, endpoint, other_legs_pose={})

        self.assertEqual(result.contact_model_status, "PATH_COLLISION_BEFORE_ENDPOINT")
        pair = {result.path_collision_link_a, result.path_collision_link_b}
        self.assertTrue(any(link.startswith("rh_") for link in pair))


class TestLFV25MeshVsHardwareReconciliation(unittest.TestCase):
    """Second-pass reconciliation correction: contact_model_status for a
    hardware-oracle endpoint must be driven by MESH vs HARDWARE agreement,
    not by HARDWARE vs URDF agreement (the first-pass classifier used the
    latter, which let lf_lower_leg_min be called MODEL_LIMIT_MISMATCH
    even though its mesh contact is ~5.8deg past where hardware actually
    stopped -- the hardware-vs-URDF gap being small at that endpoint is
    irrelevant to whether the MESH finding is correct).

    Deliberately synthetic/direct calls to `_classify_contact_model_status`
    with the real, already-known LF V25 hardware contact angles and the
    real, already-confirmed mesh contact angles from this session's
    compiler run -- no new expensive geometric search is performed here
    (the full compiler re-run separately exercises the real end-to-end
    wiring across all 24 endpoints)."""

    @staticmethod
    def _endpoint(endpoint_id: str) -> EndpointSpec:
        parts = endpoint_id.split("_")
        leg, side, group = parts[0], parts[-1], "_".join(parts[1:-1])
        limits = {
            "hip": (-0.785398163397, 0.785398163397),
            "upper_leg": (-0.916297857297, 2.138028333693),
            "lower_leg": (-1.605702911835, 0.654498469498),
        }
        lower, upper = limits[group]
        return EndpointSpec(
            leg=leg, joint_group=group, side=side, joint_name=f"{leg}_{group}_joint", servo_id=0,
            urdf_lower_rad=lower, urdf_upper_rad=upper, prerequisite_overrides={},
        )

    def test_lf_lower_leg_min_is_model_incomplete_not_mismatch(self):
        """Point 1: mesh ~5.8deg past the real hardware contact must not
        stay MODEL_LIMIT_MISMATCH just because hardware happens to be
        close to the declared URDF value."""
        endpoint = self._endpoint("lf_lower_leg_min")
        mesh_contact_rad = -1.709725  # -97.957 deg, this session's confirmed compiler search result
        status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
            endpoint,
            same_leg_found=True,
            mesh_contact_rad=mesh_contact_rad,
            delta_rad=mesh_contact_rad - endpoint.urdf_lower_rad,
            mismatch=True,
            coarse_step_rad=math.radians(1.0),
            path_collision_angle_rad=None,
            model_limit_mismatch_threshold_deg=2.0,
        )
        self.assertNotEqual(status, "MODEL_LIMIT_MISMATCH")
        self.assertEqual(status, "MODEL_INCOMPLETE")
        self.assertEqual(mesh_vs_hw, "DISAGREES")
        self.assertEqual(hw_vs_urdf, "COMPATIBLE")  # hw is close to declared here -- irrelevant to the mesh finding
        self.assertIsNotNone(hw_note)

    def test_lf_upper_leg_min_and_max_are_model_incomplete(self):
        """Point 2: V25 proves a real contact exists for both upper
        endpoints, but the collision mesh shows nothing there at all --
        must be MODEL_INCOMPLETE (missing model geometry), never
        NO_MODELED_ENDSTOP (which implies "no reason to think anything is
        missing")."""
        for endpoint_id in ("lf_upper_leg_min", "lf_upper_leg_max"):
            with self.subTest(endpoint_id=endpoint_id):
                endpoint = self._endpoint(endpoint_id)
                status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
                    endpoint,
                    same_leg_found=False,
                    mesh_contact_rad=None,
                    delta_rad=None,
                    mismatch=False,
                    coarse_step_rad=math.radians(1.0),
                    path_collision_angle_rad=None,
                    model_limit_mismatch_threshold_deg=2.0,
                )
                self.assertEqual(status, "MODEL_INCOMPLETE")
                self.assertEqual(mesh_vs_hw, "NO_MESH_CONTACT")
                self.assertIsNotNone(hw_note)

    def test_all_six_lf_hardware_validated_endpoints_are_model_incomplete(self):
        """Point 3: with the real mesh-search results from this session
        (hip_min/lower_min have a mesh contact several degrees past
        hardware; hip_max/upper_min/upper_max/lower_max have no mesh
        contact at all), every one of the 6 hardware-validated LF
        endpoints must resolve to MODEL_INCOMPLETE -- none of the six
        mesh contacts (or absences) actually correspond to where V25
        hardware proved the real endstop is."""

        # (endpoint_id, mesh_contact_rad or None) -- mesh values are this
        # session's confirmed compiler search results (geometry search is
        # unchanged by this classification fix).
        cases = {
            "lf_hip_min": -0.829031394697,  # -47.500 deg
            "lf_hip_max": None,
            "lf_upper_leg_min": None,
            "lf_upper_leg_max": None,
            "lf_lower_leg_min": -1.709725,  # -97.957 deg
            "lf_lower_leg_max": None,
        }
        self.assertEqual(set(cases), set(LF_V25_HARDWARE_EVIDENCE))

        for endpoint_id, mesh_contact_rad in cases.items():
            with self.subTest(endpoint_id=endpoint_id):
                endpoint = self._endpoint(endpoint_id)
                declared = endpoint.urdf_upper_rad if endpoint_id.endswith("max") else endpoint.urdf_lower_rad
                status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
                    endpoint,
                    same_leg_found=mesh_contact_rad is not None,
                    mesh_contact_rad=mesh_contact_rad,
                    delta_rad=(mesh_contact_rad - declared) if mesh_contact_rad is not None else None,
                    mismatch=(mesh_contact_rad is not None),
                    coarse_step_rad=math.radians(1.0),
                    path_collision_angle_rad=None,
                    model_limit_mismatch_threshold_deg=2.0,
                )
                self.assertEqual(status, "MODEL_INCOMPLETE", f"{endpoint_id}: got {status} ({reason})")
                self.assertIn(mesh_vs_hw, ("DISAGREES", "NO_MESH_CONTACT"))

    def test_model_limit_mismatch_remains_available_as_independent_diagnostic(self):
        """Point 4: MODEL_LIMIT_MISMATCH must still be reachable as the
        primary status for legs WITHOUT a hardware oracle, and the
        mesh-vs-declared-URDF mismatch must still be mentioned as an
        independent diagnostic detail even when MODEL_INCOMPLETE (driven
        by mesh-vs-hardware) is the primary status for LF."""
        # (a) no hardware oracle -> MODEL_LIMIT_MISMATCH remains reachable.
        rf_endpoint = self._endpoint("rf_hip_max")
        status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
            rf_endpoint,
            same_leg_found=True,
            mesh_contact_rad=rf_endpoint.urdf_upper_rad + math.radians(2.5),
            delta_rad=math.radians(2.5),
            mismatch=True,
            coarse_step_rad=math.radians(1.0),
            path_collision_angle_rad=None,
            model_limit_mismatch_threshold_deg=2.0,
        )
        self.assertEqual(status, "MODEL_LIMIT_MISMATCH")
        self.assertEqual(hw_vs_urdf, "NOT_AVAILABLE")
        self.assertEqual(mesh_vs_hw, "NOT_AVAILABLE")

        # (b) LF with hardware oracle, mesh disagrees with hardware -> the
        # mismatch-vs-URDF detail is still surfaced in the reason text
        # even though the primary status is MODEL_INCOMPLETE.
        lf_endpoint = self._endpoint("lf_hip_min")
        status, reason, hw_note, hw_vs_urdf, mesh_vs_hw = _classify_contact_model_status(
            lf_endpoint,
            same_leg_found=True,
            mesh_contact_rad=-0.829031394697,
            delta_rad=-0.829031394697 - lf_endpoint.urdf_lower_rad,
            mismatch=True,
            coarse_step_rad=math.radians(1.0),
            path_collision_angle_rad=None,
            model_limit_mismatch_threshold_deg=2.0,
        )
        self.assertEqual(status, "MODEL_INCOMPLETE")
        self.assertIn("model_limit_mismatch=True", reason)

    def test_rf_rh_lh_without_hardware_oracle_never_auto_promoted_to_model_incomplete(self):
        """Point 5: legs without a V25 hardware oracle must never be
        promoted to MODEL_INCOMPLETE automatically -- there is no proof
        the model is missing anything for them, only a mesh-vs-declared
        mismatch or absence, which stays MODEL_LIMIT_MISMATCH /
        NO_MODELED_ENDSTOP."""

        for leg in ("rf", "rh", "lh"):
            for group, side in (("hip", "max"), ("lower_leg", "min")):
                endpoint_id = f"{leg}_{group}_{side}"
                self.assertNotIn(endpoint_id, LF_V25_HARDWARE_EVIDENCE)
                endpoint = self._endpoint(endpoint_id)
                declared = endpoint.urdf_upper_rad if side == "max" else endpoint.urdf_lower_rad

                with self.subTest(endpoint_id=endpoint_id, case="mesh_found_mismatched"):
                    status, *_ = _classify_contact_model_status(
                        endpoint, same_leg_found=True,
                        mesh_contact_rad=declared + math.radians(6.0),
                        delta_rad=math.radians(6.0), mismatch=True,
                        coarse_step_rad=math.radians(1.0), path_collision_angle_rad=None,
                        model_limit_mismatch_threshold_deg=2.0,
                    )
                    self.assertNotEqual(status, "MODEL_INCOMPLETE")
                    self.assertEqual(status, "MODEL_LIMIT_MISMATCH")

                with self.subTest(endpoint_id=endpoint_id, case="no_mesh_found"):
                    status, *_ = _classify_contact_model_status(
                        endpoint, same_leg_found=False,
                        mesh_contact_rad=None, delta_rad=None, mismatch=False,
                        coarse_step_rad=math.radians(1.0), path_collision_angle_rad=None,
                        model_limit_mismatch_threshold_deg=2.0,
                    )
                    self.assertNotEqual(status, "MODEL_INCOMPLETE")
                    self.assertEqual(status, "NO_MODELED_ENDSTOP")


if __name__ == "__main__":
    unittest.main()
