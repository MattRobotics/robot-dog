#!/usr/bin/env python3
"""MATDOG Phase 1B — joint-aware adjacency, endstop metrology vs path safety.

These tests exercise the real production logic against the real URDF and the
real (corrected) collision meshes. They deliberately avoid mocks and avoid
asserting on constants alone: the point is to pin the behaviour that GATE A and
GATE B established, so the blanket-adjacent-exclusion bug cannot come back.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import sys
import unittest

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
KINEMATICS_DIR = CALIBRATION_DIR.parents[0] / "kinematics"
REPO_ROOT = CALIBRATION_DIR.parents[2]

for _p in (KINEMATICS_DIR, CALIBRATION_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from matdog_geometry_contact_search import (  # noqa: E402
    LF_V25_HARDWARE_EVIDENCE,
    load_all_endpoints,
    search_endpoint_contact,
)
from matdog_geometry_mesh_kernel import (  # noqa: E402
    BOOLEAN_COLLISION_MARGIN_M,
    DEFAULT_NARROW_PHASE_MARGIN_M,
    clearance_gate,
)
from matdog_geometry_scene import (  # noqa: E402
    ALL_JOINT_NAMES,
    LEG_IDS,
    PAIR_CLASS_FIXED_ADJACENT,
    PAIR_CLASS_NON_ADJACENT,
    PAIR_CLASS_REVOLUTE_ADJACENT,
    RobotScene,
    active_revolute_contact_pair,
    classify_link_pair,
    clearance_policy_pairs,
    full_pose,
    home_pose,
    load_link_adjacency,
    path_safety_pairs,
)

VISUAL_MESH_DIR = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/meshes"
COLLISION_MESH_DIR = VISUAL_MESH_DIR / "collision"

# These five historical detailed meshes remain unchanged visual assets after G1.
EXPECTED_VISUAL_MESH_SHA256 = {
    "base_link.stl": "644a83e98fd116f3fc8e5d8792ca2b60b0bdb09bcafa4fc49140d641079e4b6b",
    "lf_upper_leg_link.stl": "3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169",
    "lh_upper_leg_link.stl": "3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169",
    "rf_upper_leg_link.stl": "08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16",
    "rh_upper_leg_link.stl": "08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16",
}

# Their approved G1 collision representations are pinned independently.
EXPECTED_COLLISION_MESH_SHA256 = {
    "base_link.stl": "0a485e7a1101d457f317b664e52a7a4ef061382e6ecc2c7c87a333c81d5b466f",
    "lf_upper_leg_link.stl": "0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c",
    "lh_upper_leg_link.stl": "0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c",
    "rf_upper_leg_link.stl": "fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91",
    "rh_upper_leg_link.stl": "fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91",
}

_SCENE: RobotScene | None = None


def shared_scene() -> RobotScene:
    """One RobotScene for the whole module: loading 17 STL meshes and their
    convex hulls is the dominant cost, and the scene is read-only here."""
    global _SCENE
    if _SCENE is None:
        _SCENE = RobotScene()
    return _SCENE


class TestJointAwareAdjacency(unittest.TestCase):
    """1-2: the 12 revolute and 4 fixed adjacencies, derived from URDF."""

    def setUp(self):
        self.adjacency = load_link_adjacency()

    def test_twelve_revolute_adjacent_pairs_identified(self):
        revolute = self.adjacency.revolute_pairs
        self.assertEqual(len(revolute), 12, "MATDOG has 12 revolute parent-child pairs")

        for leg in LEG_IDS:
            for pair in (
                ("base_link", f"{leg}_hip_link"),
                (f"{leg}_hip_link", f"{leg}_upper_leg_link"),
                (f"{leg}_upper_leg_link", f"{leg}_lower_leg_link"),
            ):
                self.assertIn(frozenset(pair), revolute, f"{pair} must be REVOLUTE adjacent")
                self.assertEqual(classify_link_pair(*pair), PAIR_CLASS_REVOLUTE_ADJACENT)

    def test_four_fixed_adjacent_pairs_identified(self):
        fixed = self.adjacency.fixed_pairs
        self.assertEqual(len(fixed), 4, "MATDOG has 4 fixed (foot) attachments")

        for leg in LEG_IDS:
            pair = (f"{leg}_lower_leg_link", f"{leg}_foot_link")
            self.assertIn(frozenset(pair), fixed, f"{pair} must be FIXED adjacent")
            self.assertEqual(classify_link_pair(*pair), PAIR_CLASS_FIXED_ADJACENT)

    def test_adjacency_is_derived_from_urdf_not_hardcoded(self):
        """Every adjacency must be traceable to a real URDF joint."""
        from matdog_urdf_fk import canonical_urdf_path, load_urdf_joints

        joints = load_urdf_joints(canonical_urdf_path(REPO_ROOT))
        urdf_pairs = {frozenset((j.parent_link, j.child_link)) for j in joints.values()}
        self.assertEqual(self.adjacency.all_adjacent_pairs, urdf_pairs)


class TestActivePairPolicy(unittest.TestCase):
    """3-6: the primary endpoint pair per joint, and its exclusion from path safety."""

    def test_primary_active_pair_hip(self):
        for leg in LEG_IDS:
            self.assertEqual(
                active_revolute_contact_pair(f"{leg}_hip_joint"), ("base_link", f"{leg}_hip_link")
            )

    def test_primary_active_pair_upper(self):
        for leg in LEG_IDS:
            self.assertEqual(
                active_revolute_contact_pair(f"{leg}_upper_leg_joint"),
                (f"{leg}_hip_link", f"{leg}_upper_leg_link"),
            )

    def test_primary_active_pair_lower(self):
        for leg in LEG_IDS:
            self.assertEqual(
                active_revolute_contact_pair(f"{leg}_lower_leg_joint"),
                (f"{leg}_upper_leg_link", f"{leg}_lower_leg_link"),
            )

    def test_fixed_joint_has_no_active_contact_pair(self):
        from matdog_geometry_scene import GeometrySceneError

        with self.assertRaises(GeometrySceneError):
            active_revolute_contact_pair("lf_foot_joint")

    def test_active_pair_excluded_from_its_own_path_obstruction_set(self):
        for joint in ALL_JOINT_NAMES:
            active = active_revolute_contact_pair(joint)
            safety = path_safety_pairs(exclude_pair=active)
            self.assertNotIn(active, safety, f"{joint}: active pair must not obstruct itself")
            self.assertNotIn(tuple(reversed(active)), safety)

    def test_non_active_revolute_adjacent_is_in_path_safety_set(self):
        """7: a revolute adjacency that is NOT the one being measured stays a
        real obstruction candidate."""
        active = active_revolute_contact_pair("lf_lower_leg_joint")
        safety = frozenset(frozenset(p) for p in path_safety_pairs(exclude_pair=active))

        self.assertIn(frozenset(("base_link", "lf_hip_link")), safety)
        self.assertIn(frozenset(("lf_hip_link", "lf_upper_leg_link")), safety)

    def test_fixed_adjacent_never_in_path_safety_set(self):
        safety = frozenset(frozenset(p) for p in path_safety_pairs())
        for leg in LEG_IDS:
            self.assertNotIn(frozenset((f"{leg}_lower_leg_link", f"{leg}_foot_link")), safety)

    def test_historical_regression_hip_foot_is_path_not_lower_endstop(self):
        """The regression that must never come back: during a LOWER probe,
        hip<->foot is a PATH obstruction, not the LOWER endstop."""
        active = active_revolute_contact_pair("lf_lower_leg_joint")
        safety = frozenset(frozenset(p) for p in path_safety_pairs(exclude_pair=active))

        self.assertIn(frozenset(("lf_hip_link", "lf_foot_link")), safety)
        self.assertEqual(active, ("lf_upper_leg_link", "lf_lower_leg_link"))
        self.assertNotEqual(frozenset(active), frozenset(("lf_hip_link", "lf_foot_link")))


class TestClearancePolicy(unittest.TestCase):
    """8-9: the generic clearance gate applies to NON_ADJACENT only."""

    def test_revolute_adjacent_excluded_from_clearance_gate_set(self):
        pairs = frozenset(frozenset(p) for p in clearance_policy_pairs())
        for leg in LEG_IDS:
            self.assertNotIn(frozenset(("base_link", f"{leg}_hip_link")), pairs)
            self.assertNotIn(frozenset((f"{leg}_hip_link", f"{leg}_upper_leg_link")), pairs)
            self.assertNotIn(frozenset((f"{leg}_upper_leg_link", f"{leg}_lower_leg_link")), pairs)

    def test_separated_revolute_adjacent_under_3mm_does_not_fail_generic_gate(self):
        """A hinge resting a fraction of a millimetre from its counterpart is
        healthy and must not be dragged through the generic 3 mm gate.

        The requirement asserted here is SEMANTIC -- the pair is SEPARATED, and
        the generic gate is not applied to it -- not a clearance value. The
        GATE B figures (0.0026-0.0372 mm at q=0) are evidence about the current
        tessellation, not a physical assembly clearance to be preserved, so no
        exact clearance is computed or asserted.
        """
        scene = shared_scene()
        pose = home_pose()

        for joint in ("lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"):
            link_a, link_b = active_revolute_contact_pair(joint)
            result = scene.check_link_pair(link_a, link_b, pose, require_distance=False)

            self.assertNotEqual(result.status, "INTERSECTING", f"{joint} must be separated at q=0")

            # The pair is outside the set the generic gate is ever applied to --
            # which is what prevents a healthy sub-3mm hinge from being failed.
            self.assertNotIn(
                (link_a, link_b),
                clearance_policy_pairs(),
                f"{joint}: revolute adjacency must be outside the clearance gate set",
            )

        # Demonstrate the consequence explicitly: were such a separation fed to
        # the generic gate, it WOULD fail. Exclusion from the set is the whole
        # mechanism, so this must stay true for the exclusion to matter.
        self.assertEqual(clearance_gate("SEPARATED_NARROW", 0.0026e-3, "EXACT", 0.003), "FAIL")

    def test_no_policy_depends_on_the_measured_micro_clearance(self):
        """Guard against the GATE B micro-clearances hardening into a spec.

        Asserted as SOFTWARE SEMANTICS only. The physical scales that make such
        a figure insignificant -- per-part print tolerance, observed
        CAD/tessellation/model surface differences of O(0.1 mm), unmodelled
        assembly stack-up -- are documentation rationale and are deliberately
        NOT encoded as numeric requirements here. What the code must guarantee
        is narrower and checkable: no pass/fail anywhere keys off the value.
        """
        import ast

        production_sources = (
            "matdog_geometry_scene.py",
            "matdog_geometry_contact_search.py",
            "matdog_geometry_mesh_kernel.py",
            "matdog_geometry_path_planner.py",
            "matdog_geometry_uncertainty.py",
            "matdog_geometry_profile.py",
            "matdog_geometry_compiler.py",
        )
        # The GATE B q=0 figures, in metres. Quoting them in prose is fine and
        # intended -- they are the rationale. What must not exist is a numeric
        # LITERAL equal to one of them, which is what a threshold would look
        # like. So the check is on parsed constants, not on raw text.
        forbidden_m = (0.0026e-3, 0.0101e-3, 0.0372e-3)

        for name in production_sources:
            tree = ast.parse((CALIBRATION_DIR / name).read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Constant) and isinstance(node.value, float):
                    for value in forbidden_m:
                        self.assertFalse(
                            math.isclose(node.value, value, rel_tol=1e-6),
                            f"{name}:{node.lineno} hard-codes a GATE B micro-clearance "
                            f"({node.value}) as a numeric constant; it is evidence, not a threshold",
                        )

        # No clearance gate can ever see a revolute adjacent pair, so no
        # clearance figure of theirs can drive a verdict.
        gate_set = frozenset(frozenset(p) for p in clearance_policy_pairs())
        for joint in ALL_JOINT_NAMES:
            self.assertNotIn(frozenset(active_revolute_contact_pair(joint)), gate_set)

    def test_non_adjacent_clearance_semantics_unchanged(self):
        """9: EXACT / LOWER_BOUND / UNRESOLVED_FOR_THRESHOLD are untouched."""
        self.assertEqual(clearance_gate("SEPARATED_NARROW", 0.001, "LOWER_BOUND", 0.003),
                         "UNRESOLVED_FOR_THRESHOLD")
        self.assertEqual(clearance_gate("SEPARATED_NARROW", 0.001, "EXACT", 0.003), "FAIL")
        self.assertEqual(clearance_gate("SEPARATED_AABB", 0.005, "LOWER_BOUND", 0.003), "PASS")
        self.assertEqual(clearance_gate("INTERSECTING", None, None, 0.003), "FAIL")

        pairs = frozenset(frozenset(p) for p in clearance_policy_pairs())
        self.assertIn(frozenset(("base_link", "lf_upper_leg_link")), pairs)
        self.assertIn(frozenset(("lf_upper_leg_link", "lf_foot_link")), pairs)
        self.assertEqual(len(pairs), 120, "non-adjacent set size must not silently change")


class TestBooleanVersusClearancePath(unittest.TestCase):
    """7 (spec): boolean intersection and clearance must not share a margin."""

    def test_boolean_and_clearance_margins_are_distinct(self):
        self.assertEqual(BOOLEAN_COLLISION_MARGIN_M, 0.0)
        self.assertGreater(DEFAULT_NARROW_PHASE_MARGIN_M, 0.0)

    def test_boolean_path_agrees_with_clearance_path_on_intersection(self):
        """Margin 0 must not change any verdict -- intersecting triangles always
        share a grid cell regardless of near-miss inflation.

        Exercised on NON_ADJACENT pairs, both clear and colliding. The clearance
        path is not run against revolute adjacent pairs here on purpose: at
        micron separation its near-miss search is enormous, and it is not the
        path those pairs are ever evaluated through in production.
        """
        scene = shared_scene()

        cases = (
            ("base_link", "lf_upper_leg_link", home_pose()),
            ("base_link", "lf_upper_leg_link", full_pose({"lf_hip_joint": math.radians(-50.0)})),
            ("base_link", "lf_lower_leg_link", home_pose()),
        )

        for link_a, link_b, pose in cases:
            boolean = scene.check_link_pair(link_a, link_b, pose, require_distance=False)
            exact = scene.check_link_pair(link_a, link_b, pose, require_distance=True)
            self.assertEqual(
                boolean.status == "INTERSECTING",
                exact.status == "INTERSECTING",
                f"{link_a} <-> {link_b}: boolean and clearance paths disagree",
            )

    def test_clearance_path_still_reports_exact_figures(self):
        """Non-adjacent clearance semantics must survive Phase 1B untouched."""
        scene = shared_scene()
        result = scene.check_link_pair(
            "base_link", "lf_lower_leg_link", home_pose(), require_distance=True
        )
        self.assertNotEqual(result.status, "INTERSECTING")
        self.assertIsNotNone(result.clearance_m)
        self.assertIn(result.clearance_kind, ("EXACT", "LOWER_BOUND"))

    def test_boolean_path_does_not_report_a_clearance_figure(self):
        scene = shared_scene()
        result = scene.check_link_pair("base_link", "lf_hip_link", home_pose(), require_distance=False)
        self.assertIsNone(result.clearance_m)


class TestCorrectedGeometryAtHome(unittest.TestCase):
    """10: the motor-pin fix removes the permanent overlap on all 12 revolute pairs."""

    def test_all_twelve_revolute_pairs_separated_at_q0(self):
        scene = shared_scene()
        pose = home_pose()
        offenders = []

        for joint in ALL_JOINT_NAMES:
            link_a, link_b = active_revolute_contact_pair(joint)
            result = scene.check_link_pair(link_a, link_b, pose, require_distance=False)
            if result.status == "INTERSECTING":
                offenders.append(f"{joint} ({link_a} <-> {link_b})")

        self.assertEqual(offenders, [], f"revolute pairs still intersecting at q=0: {offenders}")


class TestMeshIntegrity(unittest.TestCase):
    """16: visual assets stay fixed and collision assets follow approved G1."""

    def test_five_historical_visual_mesh_hashes(self):
        for name, expected in EXPECTED_VISUAL_MESH_SHA256.items():
            path = VISUAL_MESH_DIR / name
            self.assertTrue(path.is_file(), f"missing {name}")
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(digest, expected, f"{name} visual geometry changed")

    def test_meshes_are_real_binary_stl_not_lfs_pointers(self):
        for directory, names in (
            (VISUAL_MESH_DIR, EXPECTED_VISUAL_MESH_SHA256),
            (COLLISION_MESH_DIR, EXPECTED_COLLISION_MESH_SHA256),
        ):
            for name in names:
                with (directory / name).open("rb") as handle:
                    head = handle.read(64)
                self.assertFalse(head.startswith(b"version https://git-lfs"),
                                 f"{directory / name} is an unsmudged LFS pointer")

    def test_scene_mesh_hashes_match_files_on_disk(self):
        """Staleness guard: the scene follows the URDF collision manifest.

        G1 explicitly separated detailed visual meshes from the approved
        collision meshes, so the old root-level visual-file constants are no
        longer valid expectations for ``RobotScene.mesh``.
        """
        scene = shared_scene()
        for name, approved_collision_sha in EXPECTED_COLLISION_MESH_SHA256.items():
            link = name[:-4]
            entry = scene.mesh_manifest[link]
            collision_path = scene.urdf_path.parent / entry.stl_relative_path
            expected = hashlib.sha256(collision_path.read_bytes()).hexdigest()
            self.assertEqual(collision_path, COLLISION_MESH_DIR / name)
            self.assertEqual(expected, approved_collision_sha, f"{link} collision mesh changed")
            self.assertEqual(
                scene.mesh(link).sha256,
                expected,
                f"{link} scene mesh does not match URDF collision filename",
            )


class TestNoHardwareAccess(unittest.TestCase):
    """20: the geometry stack must not reach hardware."""

    def test_geometry_modules_import_no_hardware_or_serial(self):
        forbidden = ("serial", "pyserial", "norma_core", "station")
        sources = [
            "matdog_geometry_scene.py",
            "matdog_geometry_contact_search.py",
            "matdog_geometry_mesh_kernel.py",
            "matdog_geometry_path_planner.py",
            "matdog_geometry_profile.py",
            "matdog_geometry_report.py",
            "matdog_geometry_compiler.py",
        ]
        for name in sources:
            text = (CALIBRATION_DIR / name).read_text().lower()
            for token in forbidden:
                self.assertNotIn(f"import {token}", text, f"{name} imports {token}")
                self.assertNotIn(f"from {token}", text, f"{name} imports from {token}")

    def test_no_hardware_modules_loaded_after_import(self):
        self.assertNotIn("serial", sys.modules)


class TestDeterminism(unittest.TestCase):
    """17: repeated evaluation must give identical results."""

    def test_pair_sets_are_deterministic(self):
        self.assertEqual(path_safety_pairs(), path_safety_pairs())
        self.assertEqual(clearance_policy_pairs(), clearance_policy_pairs())
        self.assertEqual(
            active_revolute_contact_pair("lf_lower_leg_joint"),
            active_revolute_contact_pair("lf_lower_leg_joint"),
        )

    def test_collision_evaluation_is_deterministic(self):
        """Two independent scenes must agree exactly -- a shared scene would
        only prove the pair cache returns what it stored."""
        pose = full_pose({"lf_hip_joint": math.radians(-30.0)})

        first = shared_scene().check_link_pair(
            "base_link", "lf_hip_link", pose, require_distance=False
        )
        second = RobotScene().check_link_pair(
            "base_link", "lf_hip_link", pose, require_distance=False
        )

        self.assertEqual(first.status, second.status)
        self.assertEqual(first.witness_triangle_a, second.witness_triangle_a)
        self.assertEqual(first.witness_triangle_b, second.witness_triangle_b)

    def test_non_adjacent_clearance_is_deterministic(self):
        pose = home_pose()
        scene = shared_scene()
        first = scene.check_link_pair("base_link", "lf_lower_leg_link", pose, require_distance=True)
        second = RobotScene().check_link_pair(
            "base_link", "lf_lower_leg_link", pose, require_distance=True
        )
        self.assertEqual(first.status, second.status)
        self.assertEqual(first.clearance_m, second.clearance_m)
        self.assertEqual(first.clearance_kind, second.clearance_kind)


class TestEndpointCoverage(unittest.TestCase):
    """18: 24 endpoints, each bound to its own active revolute pair."""

    def test_twenty_four_endpoints_each_with_an_active_pair(self):
        endpoints = load_all_endpoints(REPO_ROOT)
        self.assertEqual(len(endpoints), 24)

        for endpoint in endpoints:
            pair = active_revolute_contact_pair(endpoint.joint_name)
            self.assertEqual(classify_link_pair(*pair), PAIR_CLASS_REVOLUTE_ADJACENT)
            self.assertIn(endpoint.side, ("min", "max"))

    def test_endpoint_ids_are_unique(self):
        ids = [e.endpoint_id for e in load_all_endpoints(REPO_ROOT)]
        self.assertEqual(len(ids), len(set(ids)))


class TestMemoryBoundFailurePaths(unittest.TestCase):
    """19: the safety caps still fail loudly instead of growing unbounded."""

    def test_candidate_cap_raises_instead_of_growing(self):
        from matdog_geometry_mesh_kernel import MeshKernelError, check_pair

        scene = shared_scene()
        pose = home_pose()
        mesh = scene.mesh("base_link")
        transform = scene.link_transform("base_link", pose)

        with self.assertRaises(MeshKernelError):
            check_pair(mesh, transform, mesh, transform,
                       max_candidate_pairs=8, require_distance=False)

    def test_grid_rejects_wrong_unit_scale(self):
        from matdog_geometry_mesh_kernel import MeshKernelError, _triangle_grid_index
        import numpy as np

        # A triangle in millimetres fed to a metre-scale grid.
        millimetre_triangle = np.array([[[0.0, 0.0, 0.0], [300.0, 0.0, 0.0], [0.0, 300.0, 0.0]]])
        with self.assertRaises(MeshKernelError):
            _triangle_grid_index(millimetre_triangle, 0.005, 0.0)


class TestLfEndpointsAgainstHardware(unittest.TestCase):
    """11-14: the six LF endpoints, the hardware oracle, and the two historical
    non-adjacent findings. This is the slow, decisive block."""

    @classmethod
    def setUpClass(cls):
        cls.scene = shared_scene()
        cls.endpoints = {e.endpoint_id: e for e in load_all_endpoints(REPO_ROOT) if e.leg == "lf"}
        cls.results = {}

    def _result(self, endpoint_id):
        if endpoint_id not in self.results:
            self.results[endpoint_id] = search_endpoint_contact(
                self.scene, self.endpoints[endpoint_id]
            )
        return self.results[endpoint_id]

    def test_lf_six_endpoints_find_adjacent_contact(self):
        missing = []
        for endpoint_id in sorted(self.endpoints):
            result = self._result(endpoint_id)
            if result.result_kind != "MESH_CONTACT_FOUND":
                missing.append(f"{endpoint_id}: {result.result_kind}")
        self.assertEqual(missing, [], f"LF endpoints without an adjacent contact: {missing}")

    def test_lf_contacts_are_on_the_active_revolute_pair(self):
        for endpoint_id in sorted(self.endpoints):
            result = self._result(endpoint_id)
            if result.result_kind != "MESH_CONTACT_FOUND":
                continue
            expected = active_revolute_contact_pair(self.endpoints[endpoint_id].joint_name)
            self.assertEqual(
                frozenset((result.contact_link_a, result.contact_link_b)),
                frozenset(expected),
                f"{endpoint_id}: contact must be on the active revolute pair",
            )

    def test_adjacent_endpoint_emits_no_clearance_figure(self):
        """Nothing downstream can depend on a micro-clearance that is never
        produced: the search reports no clearance for an adjacent contact."""
        for endpoint_id in sorted(self.endpoints):
            result = self._result(endpoint_id)
            if result.result_kind == "MESH_CONTACT_FOUND":
                self.assertIsNone(
                    result.clearance_before_contact_m,
                    f"{endpoint_id}: adjacent endpoints must not report a clearance value",
                )

    def test_lf_hardware_reconciliation_is_reported(self):
        """12: every LF endpoint carries its V25 oracle and a computed delta.
        No PASS threshold is asserted -- the deltas are evidence, not a target."""
        for endpoint_id in sorted(self.endpoints):
            result = self._result(endpoint_id)
            self.assertIsNotNone(result.hardware_evidence_note, endpoint_id)
            self.assertIn(result.mesh_vs_hardware_status,
                          ("AGREES", "DISAGREES", "NO_MESH_CONTACT"))

            if result.result_kind == "MESH_CONTACT_FOUND":
                hw = float(LF_V25_HARDWARE_EVIDENCE[endpoint_id]["hardware_contact_rad"])
                delta_deg = math.degrees(result.mesh_predicted_contact_rad - hw)
                self.assertLess(abs(delta_deg), 15.0,
                                f"{endpoint_id}: delta {delta_deg:+.3f} deg is implausibly large")

    def test_historical_non_adjacent_base_upper_still_reproducible(self):
        """13: base<->upper at ~-47.500 deg must remain measurable."""
        angle = self._first_contact_angle_deg(
            "base_link", "lf_upper_leg_link", "lf_hip_joint", -1.0,
            {"lf_upper_leg_joint": math.radians(50.0), "lf_lower_leg_joint": 0.0},
        )
        self.assertIsNotNone(angle, "historical base<->upper finding disappeared")
        self.assertAlmostEqual(angle, -47.500, delta=1.0)

    def test_historical_non_adjacent_upper_foot_still_reproducible(self):
        """14: upper<->foot at ~-97.957 deg must remain measurable."""
        angle = self._first_contact_angle_deg(
            "lf_upper_leg_link", "lf_foot_link", "lf_lower_leg_joint", -1.0,
            {"lf_hip_joint": 0.0, "lf_upper_leg_joint": math.radians(90.0)},
        )
        self.assertIsNotNone(angle, "historical upper<->foot finding disappeared")
        self.assertAlmostEqual(angle, -97.957, delta=1.0)

    def _first_contact_angle_deg(self, link_a, link_b, joint, sign, seeds):
        from matdog_geometry_contact_search import (
            DEFAULT_BISECTION_RESOLUTION_RAD,
            DEFAULT_COARSE_STEP_RAD,
            bisect_collision_boundary,
            bracket_collision_boundary,
        )

        def collide(angle):
            overrides = dict(seeds)
            overrides[joint] = angle
            pose = full_pose(overrides)
            result = self.scene.check_link_pair(link_a, link_b, pose, require_distance=False)
            return result.status == "INTERSECTING"

        bracket = bracket_collision_boundary(
            collide, 0.0, sign, sign * math.radians(110.0), DEFAULT_COARSE_STEP_RAD
        )
        if bracket is None:
            return None

        clear, contact = bracket
        _clear, contact, _iters = bisect_collision_boundary(
            collide, clear, contact, DEFAULT_BISECTION_RESOLUTION_RAD, 40
        )
        return math.degrees(contact)


class TestPathAndParking(unittest.TestCase):
    """15: the path/prerequisite/parking layer still runs under the new policy."""

    def test_path_segment_uses_both_boolean_and_clearance_sets(self):
        from matdog_geometry_path_planner import (
            DEFAULT_CLEARANCE_SAMPLE_STRIDE,
            DEFAULT_MIN_CLEARANCE_PASS_M,
            DEFAULT_PATH_STEP_RAD,
            _sweep_and_validate,
        )

        scene = shared_scene()
        result = _sweep_and_validate(
            scene,
            description="home -> small hip sweep",
            active_joint="lf_hip_joint",
            start_rad=0.0,
            end_rad=math.radians(-10.0),
            other_legs_pose={},
            fixed_active_leg_overrides={},
            step_rad=DEFAULT_PATH_STEP_RAD,
            clearance_stride=DEFAULT_CLEARANCE_SAMPLE_STRIDE,
        )
        self.assertGreater(result.sample_count, 0)
        self.assertIn(result.clearance_gate_result,
                      ("PASS", "FAIL", "UNRESOLVED_FOR_THRESHOLD", "NOT_EVALUATED"))
        self.assertGreater(DEFAULT_MIN_CLEARANCE_PASS_M, 0.0)
        # A healthy small sweep must not be failed by a revolute hinge's normal
        # sub-millimetre resting separation.
        self.assertIsNone(result.first_collision_angle_rad,
                          f"unexpected obstruction on {result.first_collision_pair}")

    def test_home_pose_has_no_path_obstruction(self):
        """With the corrected meshes, the robot at home must be collision-free
        under the full PATH SAFETY set -- revolute adjacencies included."""
        scene = shared_scene()
        collide, pair = scene.is_colliding_at_pose(home_pose())
        self.assertFalse(collide, f"home pose reports a collision on {pair}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
