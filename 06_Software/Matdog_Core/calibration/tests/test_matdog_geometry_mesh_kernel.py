"""Test automatici MATDOG per il kernel di collisione mesh del Geometry Compiler.

Le auto-verifiche del kernel triangolo/triangolo (separato / a contatto /
compenetrato) richieste dal contratto Fase 1 vivono qui: chiamano la logica
reale (SAT 11 assi + distanza esatta triangolo/triangolo), non stringhe nel
sorgente.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_mesh_kernel import (  # noqa: E402
    Mesh,
    MeshKernelError,
    aabb_separation,
    check_pair,
    clearance_gate,
    hull_separation_margin,
    identity_transform,
    load_mesh,
    make_transform,
    sha256_file,
    transformed_aabb,
    triangle_triangle_distance,
    triangle_triangle_overlap,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
URDF_MESH_DIR = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/meshes"


def _box_triangles(min_xyz: tuple[float, float, float], max_xyz: tuple[float, float, float]) -> np.ndarray:
    """Axis-aligned box as 12 triangles (two per face)."""
    x0, y0, z0 = min_xyz
    x1, y1, z1 = max_xyz

    corners = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ]
    )

    faces = [
        (0, 1, 2), (0, 2, 3),  # bottom
        (4, 6, 5), (4, 7, 6),  # top
        (0, 5, 1), (0, 4, 5),  # front
        (1, 6, 2), (1, 5, 6),  # right
        (2, 7, 3), (2, 6, 7),  # back
        (3, 4, 0), (3, 7, 4),  # left
    ]

    return np.array([corners[list(face)] for face in faces], dtype=np.float64)


def _mesh_from_triangles(name: str, triangles: np.ndarray) -> Mesh:
    from scipy.spatial import ConvexHull

    from matdog_geometry_mesh_kernel import sha256_bytes  # noqa: E402

    centroids = triangles.mean(axis=1)
    radii = np.max(np.linalg.norm(triangles - centroids[:, None, :], axis=2), axis=1)
    unique_vertices = np.unique(triangles.reshape(-1, 3), axis=0)
    hull = ConvexHull(unique_vertices)

    return Mesh(
        name=name,
        stl_path=Path(f"<synthetic:{name}>"),
        sha256=sha256_bytes(triangles.tobytes()),
        triangles_local=triangles,
        centroids_local=centroids,
        radii_local=radii,
        hull_vertices_local=unique_vertices[hull.vertices],
        hull_equations_local=np.array(hull.equations),
        degenerate_triangle_count=0,
    )


class TestTriangleTriangleSelfTest(unittest.TestCase):
    """Item B of the Phase 1 test contract: separated / touching / intersecting."""

    def test_clearly_separated_triangles(self):
        # Both triangles point away from each other from a single pair of
        # facing vertices exactly 100 apart, so the true minimum distance
        # is unambiguous.
        tri_a = np.array([[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        tri_b = np.array([[100.0, 0.0, 0.0], [101.0, 0.0, 0.0], [100.0, 1.0, 0.0]])

        self.assertFalse(triangle_triangle_overlap(tri_a, tri_b))
        distance = triangle_triangle_distance(tri_a, tri_b)
        self.assertAlmostEqual(distance, 100.0, places=9)

    def test_shared_edge_touching_triangles(self):
        tri_a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        tri_b = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]])

        self.assertTrue(triangle_triangle_overlap(tri_a, tri_b))
        self.assertAlmostEqual(triangle_triangle_distance(tri_a, tri_b), 0.0, places=9)

    def test_vertex_touching_coplanar_triangles(self):
        # Shares exactly one vertex (1, 0, 0); the rest of each triangle
        # points away from the other, so this is a single-point touch, not
        # an edge overlap.
        tri_a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        tri_b = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, -1.0, 0.0]])

        self.assertTrue(triangle_triangle_overlap(tri_a, tri_b))
        self.assertAlmostEqual(triangle_triangle_distance(tri_a, tri_b), 0.0, places=9)

    def test_vertex_near_miss_coplanar_triangles_are_separated(self):
        tri_a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        tri_b = np.array([[1.0, 1.0, 0.0], [2.0, 1.0, 0.0], [1.0, 2.0, 0.0]])

        self.assertFalse(triangle_triangle_overlap(tri_a, tri_b))
        expected = 1.0 / np.sqrt(2.0)
        self.assertAlmostEqual(triangle_triangle_distance(tri_a, tri_b), expected, places=9)

    def test_interpenetrating_triangles(self):
        tri_a = np.array([[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]])
        tri_b = np.array([[-1.0, 0.0, -1.0], [1.0, 0.0, -1.0], [0.0, 0.0, 1.0]])

        self.assertTrue(triangle_triangle_overlap(tri_a, tri_b))

    def test_coplanar_non_overlapping_triangles_are_separated(self):
        tri_a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        tri_b = np.array([[5.0, 0.0, 0.0], [6.0, 0.0, 0.0], [5.0, 1.0, 0.0]])

        self.assertFalse(triangle_triangle_overlap(tri_a, tri_b))
        self.assertAlmostEqual(triangle_triangle_distance(tri_a, tri_b), 4.0, places=9)

    def test_edge_crossing_requires_edge_axis(self):
        # These two triangles' face normals alone do not separate them; the
        # edge x edge cross-product axes are required to find the overlap.
        tri_a = np.array([[-1.0, 0.0, -1.0], [1.0, 0.0, -1.0], [0.0, 0.0, 1.0]])
        tri_b = np.array([[0.0, -1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0]])

        self.assertTrue(triangle_triangle_overlap(tri_a, tri_b))


class TestMeshCascade(unittest.TestCase):
    """AABB / hull / narrow-phase cascade against synthetic boxes."""

    def test_far_apart_boxes_separated_at_aabb(self):
        mesh_a = _mesh_from_triangles("box_a", _box_triangles((0, 0, 0), (1, 1, 1)))
        mesh_b = _mesh_from_triangles("box_b", _box_triangles((0, 0, 0), (1, 1, 1)))

        transform_a = identity_transform()
        transform_b = make_transform(np.eye(3), (10.0, 0.0, 0.0))

        result = check_pair(mesh_a, transform_a, mesh_b, transform_b)
        self.assertEqual(result.status, "SEPARATED_AABB")
        self.assertAlmostEqual(result.clearance_m, 9.0, places=9)

    def test_overlapping_aabb_but_separated_hulls(self):
        # Box A = [0,1]^3. Box B is a unit cube (half-extent 0.5) rotated
        # 45 deg about Z and placed diagonally at (1.6, 1.6, 0.5): its
        # axis-aligned bounding square (diagonal reach 0.7071 from center)
        # overlaps box A's AABB on both X and Y, but the true diamond
        # cross-section's nearest vertex to box A is 0.6 m away (verified
        # numerically), so the real hulls do not touch.
        mesh_a = _mesh_from_triangles("box_a", _box_triangles((0, 0, 0), (1, 1, 1)))
        mesh_b = _mesh_from_triangles("box_b", _box_triangles((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)))

        transform_a = identity_transform()
        rotation_45z = np.array(
            [
                [np.cos(np.pi / 4), -np.sin(np.pi / 4), 0.0],
                [np.sin(np.pi / 4), np.cos(np.pi / 4), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        transform_b = make_transform(rotation_45z, (1.6, 1.6, 0.5))

        aabb_a = transformed_aabb(mesh_a, transform_a)
        aabb_b = transformed_aabb(mesh_b, transform_b)
        self.assertLess(aabb_separation(aabb_a, aabb_b), 0.0, "fixture must have overlapping AABBs")

        result = check_pair(mesh_a, transform_a, mesh_b, transform_b)
        self.assertIn(result.status, ("SEPARATED_HULL", "SEPARATED_NARROW"))
        # A face-normal-only margin is a conservative (safe) lower bound,
        # not the exact minimum distance (0.6 m, verified numerically by
        # clamping every box-B vertex into box A): it must be positive but
        # is not required to equal the true minimum.
        self.assertGreater(result.clearance_m, 0.0)
        self.assertLessEqual(result.clearance_m, 0.6 + 1e-9)

    def test_intersecting_boxes(self):
        # Robot-part-scale boxes (2 cm): the narrow-phase grid's defensive
        # per-triangle cell-count cap is tuned for MATDOG's actual
        # centimetre-scale geometry against its fixed 1 cm cell size, so a
        # full 1 m test cube would (correctly) trip that cap here.
        mesh_a = _mesh_from_triangles("box_a", _box_triangles((0, 0, 0), (0.02, 0.02, 0.02)))
        mesh_b = _mesh_from_triangles("box_b", _box_triangles((0, 0, 0), (0.02, 0.02, 0.02)))

        transform_a = identity_transform()
        transform_b = make_transform(np.eye(3), (0.01, 0.01, 0.01))

        result = check_pair(mesh_a, transform_a, mesh_b, transform_b)
        self.assertEqual(result.status, "INTERSECTING")
        self.assertIsNone(result.clearance_m)

    def test_hull_separation_margin_none_when_hulls_overlap(self):
        mesh_a = _mesh_from_triangles("box_a", _box_triangles((0, 0, 0), (1, 1, 1)))
        mesh_b = _mesh_from_triangles("box_b", _box_triangles((0, 0, 0), (1, 1, 1)))

        transform_a = identity_transform()
        transform_b = make_transform(np.eye(3), (0.5, 0.5, 0.5))

        margin = hull_separation_margin(mesh_a, transform_a, mesh_b, transform_b)
        self.assertIsNone(margin)


class TestRealMeshLoading(unittest.TestCase):
    """Loads a real canonical collision mesh to validate the STL loader end to end."""

    def test_load_lf_foot_mesh(self):
        if not URDF_MESH_DIR.is_dir():
            self.skipTest("URDF mesh directory non trovata in questo checkout")

        mesh = load_mesh(URDF_MESH_DIR / "lf_foot_link.stl", use_cache=False)

        self.assertEqual(mesh.name, "lf_foot_link")
        self.assertGreater(mesh.triangle_count, 0)
        self.assertEqual(mesh.hull_equations_local.shape[1], 4)
        self.assertTrue(np.all(np.isfinite(mesh.triangles_local)))
        self.assertEqual(
            mesh.sha256,
            sha256_file(URDF_MESH_DIR / "lf_foot_link.stl"),
        )

    def test_self_collision_of_identical_mesh_at_same_pose(self):
        if not URDF_MESH_DIR.is_dir():
            self.skipTest("URDF mesh directory non trovata in questo checkout")

        mesh = load_mesh(URDF_MESH_DIR / "lf_foot_link.stl", scale=(0.001, 0.001, 0.001))
        transform = identity_transform()

        result = check_pair(mesh, transform, mesh, transform)
        self.assertEqual(result.status, "INTERSECTING")

    def test_wrong_scale_mesh_is_rejected_defensively_not_silently_slow(self):
        # Loading without the URDF's 0.001 scale (millimetres treated as
        # metres) must fail fast with a clear error, not attempt an
        # unbounded grid binning loop -- this is the exact mistake that
        # previously caused this test file to run out of memory.
        mesh_mm = load_mesh(URDF_MESH_DIR / "lf_foot_link.stl", use_cache=False)
        transform = identity_transform()

        with self.assertRaises(MeshKernelError):
            check_pair(mesh_mm, transform, mesh_mm, transform)


class TestClearanceGateSemantics(unittest.TestCase):
    """Item D of the reconciliation test contract: a LOWER_BOUND clearance
    figure below a threshold must resolve as UNRESOLVED_FOR_THRESHOLD, not
    silently as FAIL -- only an EXACT figure below threshold, or an
    actual INTERSECTING status, may resolve as FAIL. Reproduces the
    reconciliation bug where a 1mm narrow-phase search-margin fallback
    value was read as if it were a measured clearance."""

    def test_lower_bound_below_threshold_is_unresolved_not_fail(self):
        self.assertEqual(
            clearance_gate("SEPARATED_NARROW", 0.001, "LOWER_BOUND", 0.003),
            "UNRESOLVED_FOR_THRESHOLD",
        )

    def test_exact_below_threshold_is_fail(self):
        self.assertEqual(clearance_gate("SEPARATED_NARROW", 0.001, "EXACT", 0.003), "FAIL")

    def test_lower_bound_at_or_above_threshold_is_pass(self):
        self.assertEqual(clearance_gate("SEPARATED_AABB", 0.005, "LOWER_BOUND", 0.003), "PASS")
        self.assertEqual(clearance_gate("SEPARATED_HULL", 0.003, "LOWER_BOUND", 0.003), "PASS")

    def test_exact_at_or_above_threshold_is_pass(self):
        self.assertEqual(clearance_gate("SEPARATED_NARROW", 0.0031, "EXACT", 0.003), "PASS")

    def test_intersecting_is_always_fail_regardless_of_clearance_value(self):
        self.assertEqual(clearance_gate("INTERSECTING", None, None, 0.003), "FAIL")

    def test_none_clearance_is_fail(self):
        self.assertEqual(clearance_gate("SEPARATED_NARROW", None, None, 0.003), "FAIL")

    def test_real_narrow_phase_no_candidates_reports_lower_bound_not_exact(self):
        """Direct kernel-level check that the 1mm search-margin fallback
        (no candidate triangle pair found within the grid search) is
        tagged LOWER_BOUND, not EXACT -- the specific representation bug
        raised in the reconciliation review."""
        far_a = _mesh_from_triangles("far_a", _box_triangles((0.0, 0.0, 0.0), (0.01, 0.01, 0.01)))
        far_b = _mesh_from_triangles("far_b", _box_triangles((0.05, 0.0, 0.0), (0.06, 0.01, 0.01)))
        transform = identity_transform()

        result = check_pair(far_a, transform, far_b, transform, narrow_phase_margin_m=0.001)

        # AABB/hull will likely already separate this trivial case; force
        # the narrow-phase path directly to exercise the fallback branch.
        from matdog_geometry_mesh_kernel import narrow_phase_check

        narrow_result = narrow_phase_check(far_a, transform, far_b, transform, margin_m=0.001)
        self.assertEqual(narrow_result.status, "SEPARATED_NARROW")
        self.assertEqual(narrow_result.clearance_kind, "LOWER_BOUND")
        self.assertEqual(narrow_result.clearance_m, 0.001)
        self.assertEqual(clearance_gate(narrow_result.status, narrow_result.clearance_m, narrow_result.clearance_kind, 0.003), "UNRESOLVED_FOR_THRESHOLD")
        del result  # AABB/hull path result not the point of this test


if __name__ == "__main__":
    unittest.main()
