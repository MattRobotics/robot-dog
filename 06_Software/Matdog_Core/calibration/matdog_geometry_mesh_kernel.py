#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler mesh collision kernel.

Offline-only exact collision engine over the canonical URDF collision STL
meshes. Reuses the AABB / convex-hull / triangle-triangle narrow-phase
methodology described in
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
(that investigation's actual analysis code was not committed at the time;
this module is the first committed, reusable implementation of the same
method so future geometry revisions do not repeat the exploration).

Pipeline per link pair, cheapest test first, never trusting a cheap test as
proof of collision (only as proof of separation):

    1. exact transformed AABB broad phase        -> can prove SEPARATED
    2. convex-hull face-normal mid phase          -> can prove SEPARATED
    3. k-d tree pruned triangle/triangle narrow
       phase (Moeller-style 11-axis SAT + exact
       triangle/triangle closest distance)        -> authoritative

AABB overlap alone is never accepted as proof of collision (matches the
2026-07-20 checkpoint rule). Convex-hull separation is accepted as proof
that the enclosed real meshes are also separated, because the mesh points
are always a subset of the mesh's own convex hull, so if the hulls are
provably separated along some axis the underlying triangle surfaces are
separated along that same axis too.

No Station, serial port, motor command or EEPROM access is performed by
this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import struct
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.spatial import ConvexHull


DEFAULT_MARGIN_M = 0.0

DEFAULT_NARROW_PHASE_MARGIN_M = 0.001
"""
Narrow-phase candidate search margin: the largest true mesh-to-mesh gap we
still want to resolve exactly via the spatial hash grid. Not a collision
threshold; generous relative to the +/-0.15 mm print tolerance discussed in
the canonical handoff (~7x), but deliberately NOT a large "safety margin"
value: narrow phase only runs once AABB and hull broad/mid phase have
already failed to prove separation, meaning by that point the true gap (if
any) is expected to already be small -- a hull separating margin bigger
than this would normally have resolved the pair one phase earlier. Every
triangle's AABB is inflated by (roughly) half of this value before
grid binning, so an oversized margin directly inflates candidate-pair
counts; a real deeply interpenetrating pair (e.g. base_link vs a lower leg
link at a colliding pose, checked while fixing repeated OOM kills during
Phase 1 development) produced tens of millions of raw candidate slots
before deduplication at the previous 20 mm default, entirely from this
inflation, not from the genuine contact geometry. If a true gap turns out
to exceed this margin, the result is a safe (conservative, non-exact)
SEPARATED_NARROW lower bound rather than a wrong verdict. Configurable per
call.
"""

DEFAULT_GRID_CELL_SIZE_M = 0.005
"""
Fixed uniform-grid cell size for narrow-phase candidate pruning (5 mm),
matched to MATDOG's actual mm-scale triangle sizes. This is a pure
performance tuning constant, not derived from any per-mesh statistic:
correctness of the candidate search does not depend on this value (see
`_triangle_grid_index`), only its speed/memory profile does.
"""

DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS = 500_000
"""
Hard safety cap on narrow-phase candidate pairs per link-pair check. This
exists so a pathological input fails fast with a clear error instead of
growing an unbounded in-memory structure (the failure mode that produced
repeated OOM kills before this cap and the fixed-cell grid replaced the
mesh-derived k-d tree radius approach).
"""

_MAX_GRID_CELLS_PER_TRIANGLE = 10_000
"""Defence-in-depth cap: a single triangle should never legitimately span
this many grid cells at the default cell size for MATDOG-scale (metre)
geometry. Exceeding it (e.g. because a mesh was loaded in millimetres
without applying the URDF mesh scale) is treated as a configuration error,
not silently absorbed by a very long binning loop."""

_MAX_GRID_TOTAL_ENTRIES = 5_000_000
"""Defence-in-depth cap on total (triangle, cell) binning insertions for
one mesh, checked before the Python binning loop runs."""

_MIN_TRIANGLE_CROSS_NORM = 1e-15
"""Triangles whose edge cross-product norm (2x their area) falls below
this are treated as degenerate CAD/tessellation-export slivers (colinear
or coincident vertices) and dropped at load time. Deliberately tight: the
smallest confirmed-degenerate triangles found in the canonical mesh set
have an exactly-zero cross product (bit-exact, not just small), while the
smallest legitimate real triangles measured across those same meshes sit
around 1e-12; this threshold stays several orders of magnitude below the
latter so no real (if tiny) surface triangle is ever dropped."""

_STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attr", "<u2"),
    ]
)

_MESH_CACHE: dict[str, "Mesh"] = {}


class MeshKernelError(RuntimeError):
    """Errore del kernel di collisione mesh MATDOG."""


@dataclass(frozen=True)
class Mesh:
    name: str
    stl_path: Path
    sha256: str
    triangles_local: np.ndarray
    """(T, 3, 3) float64 — T triangles, 3 vertices, 3 coords, local link frame."""
    centroids_local: np.ndarray
    """(T, 3) float64 triangle centroids in local link frame."""
    radii_local: np.ndarray
    """(T,) float64 triangle bounding-sphere radius about its own centroid."""
    hull_vertices_local: np.ndarray
    """(H, 3) float64 — vertices of the mesh convex hull in local frame."""
    hull_equations_local: np.ndarray
    """(F, 4) float64 — scipy ConvexHull facet equations [nx, ny, nz, offset]."""
    degenerate_triangle_count: int
    """Zero/near-zero-area triangles dropped from the raw STL at load time
    (a known artifact of CAD-exported tessellation; such triangles carry no
    physical surface, so they are excluded rather than crashing the SAT
    test, which is undefined for a degenerate triangle)."""

    @property
    def triangle_count(self) -> int:
        return int(self.triangles_local.shape[0])


@dataclass(frozen=True)
class RigidTransform:
    rotation: np.ndarray
    """(3, 3) float64 rotation matrix, world_from_local."""
    translation: np.ndarray
    """(3,) float64 translation, world_from_local."""

    def apply_points(self, points_local: np.ndarray) -> np.ndarray:
        return points_local @ self.rotation.T + self.translation


@dataclass(frozen=True)
class Aabb:
    min_xyz: np.ndarray
    max_xyz: np.ndarray


@dataclass(frozen=True)
class PairCollisionResult:
    link_a: str
    link_b: str
    status: str
    """One of: SEPARATED_AABB, SEPARATED_HULL, SEPARATED_NARROW, INTERSECTING."""
    clearance_m: float | None
    """
    Meaning depends on `clearance_kind` -- SEPARATED_NARROW alone does NOT
    imply an exact figure (see that field). None when INTERSECTING
    (penetration, not a signed depth estimate) or when clearance was not
    requested (require_distance=False).
    """
    clearance_kind: str | None = None
    """One of:

    EXACT -- `clearance_m` is the true minimum surface-to-surface distance
    (narrow phase found the actual closest candidate triangle pair within
    the search margin).

    LOWER_BOUND -- `clearance_m` is only a proven-safe lower bound on the
    true distance: either an AABB gap, a single-face-normal hull
    separating margin (edge/edge closest approach can be closer than any
    tested face normal), or the narrow-phase search margin itself when no
    candidate triangle pair was found inside it (the true distance is
    *at least* this value, never claim it *equals* this value -- see
    canonical handoff section 6 on clearance semantics).

    None -- INTERSECTING (no clearance value applies) or clearance was not
    requested at all.
    """
    witness_triangle_a: int | None = None
    witness_triangle_b: int | None = None

    @property
    def collision(self) -> bool:
        return self.status == "INTERSECTING"

    def clearance_at_least(self, threshold_m: float) -> str:
        """Tri-state gate check against a clearance threshold; see
        `clearance_gate` for the semantics (this is a thin instance-method
        wrapper over it)."""
        return clearance_gate(self.status, self.clearance_m, self.clearance_kind, threshold_m)


def clearance_gate(
    status: str, clearance_m: float | None, clearance_kind: str | None, threshold_m: float
) -> str:
    """Tri-state gate check of a clearance figure against a threshold,
    honest about what LOWER_BOUND values can and cannot prove (canonical
    handoff section 6):

    PASS -- proven clearance_m >= threshold_m (either an EXACT figure at
    or above threshold, or a LOWER_BOUND already at or above it, which is
    enough to prove the true distance clears the threshold too).

    FAIL -- INTERSECTING, or an EXACT clearance_m below threshold_m.

    UNRESOLVED_FOR_THRESHOLD -- a LOWER_BOUND below threshold_m: the true
    distance could be on either side of the threshold, this result does
    not resolve the question and must not be silently read as FAIL.
    """
    if status == "INTERSECTING" or clearance_m is None:
        return "FAIL"

    if clearance_m >= threshold_m:
        return "PASS"

    if clearance_kind == "EXACT":
        return "FAIL"

    return "UNRESOLVED_FOR_THRESHOLD"


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_binary_stl(data: bytes, stl_path: Path) -> np.ndarray:
    if len(data) < 84:
        raise MeshKernelError(f"STL troppo corto: {stl_path}")

    triangle_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + triangle_count * 50

    if expected_size != len(data):
        raise MeshKernelError(
            f"STL binario malformato o non binario: {stl_path} "
            f"(attesi {expected_size} byte, trovati {len(data)})"
        )

    records = np.frombuffer(data, dtype=_STL_DTYPE, offset=84, count=triangle_count)
    triangles = np.array(records["vertices"], dtype=np.float64)

    if triangles.shape[0] == 0:
        raise MeshKernelError(f"STL senza triangoli: {stl_path}")

    if not np.all(np.isfinite(triangles)):
        raise MeshKernelError(f"STL con vertici non finiti: {stl_path}")

    return triangles


def load_mesh(
    stl_path: Path,
    *,
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
    use_cache: bool = True,
) -> Mesh:
    """Load an STL collision mesh and precompute local-frame collision data.

    `scale` applies the URDF `<mesh scale="...">` factor (MATDOG STL files
    are authored in millimetres; the canonical URDF applies scale
    "0.001 0.001 0.001" on every collision mesh to stay SI-consistent) so
    the returned mesh is directly in the link's local metre frame.

    Results are cached by resolved path + scale so repeated pose
    evaluations reuse the same immutable local-frame geometry (only the
    rigid transform changes per pose).
    """
    resolved = f"{Path(stl_path).resolve()}::scale={scale}"

    if use_cache and resolved in _MESH_CACHE:
        return _MESH_CACHE[resolved]

    data = Path(stl_path).read_bytes()
    triangles = _parse_binary_stl(data, Path(stl_path)) * np.array(scale, dtype=np.float64)

    edge1 = triangles[:, 1] - triangles[:, 0]
    edge2 = triangles[:, 2] - triangles[:, 0]
    cross_norm = np.linalg.norm(np.cross(edge1, edge2), axis=1)
    degenerate_mask = cross_norm < _MIN_TRIANGLE_CROSS_NORM
    degenerate_count = int(degenerate_mask.sum())

    if degenerate_count:
        # Known CAD/tessellation-export artifact (zero/near-zero-area
        # slivers). Such a triangle has no physical surface and is
        # undefined input for the SAT test, so it is dropped rather than
        # raising or silently mis-handling it deep in narrow phase.
        triangles = triangles[~degenerate_mask]

    if triangles.shape[0] == 0:
        raise MeshKernelError(f"STL senza triangoli non degeneri: {stl_path}")

    centroids = triangles.mean(axis=1)
    radii = np.max(
        np.linalg.norm(triangles - centroids[:, None, :], axis=2),
        axis=1,
    )

    flat_vertices = triangles.reshape(-1, 3)
    unique_vertices = np.unique(flat_vertices, axis=0)

    if unique_vertices.shape[0] < 4:
        raise MeshKernelError(
            f"STL con troppo pochi vertici distinti per un hull 3D: {stl_path}"
        )

    hull = ConvexHull(unique_vertices)
    hull_vertices = unique_vertices[hull.vertices]
    hull_equations = np.array(hull.equations, dtype=np.float64)

    mesh = Mesh(
        name=Path(stl_path).stem,
        stl_path=Path(stl_path),
        sha256=sha256_bytes(data),
        triangles_local=triangles,
        centroids_local=centroids,
        radii_local=radii,
        hull_vertices_local=hull_vertices,
        hull_equations_local=hull_equations,
        degenerate_triangle_count=degenerate_count,
    )

    if use_cache:
        _MESH_CACHE[resolved] = mesh

    return mesh


def clear_mesh_cache() -> None:
    _MESH_CACHE.clear()


def make_transform(rotation: Sequence[Sequence[float]], translation: Sequence[float]) -> RigidTransform:
    rotation_array = np.array(rotation, dtype=np.float64)
    translation_array = np.array(translation, dtype=np.float64)

    if rotation_array.shape != (3, 3):
        raise MeshKernelError(f"rotation deve essere 3x3, trovato {rotation_array.shape}")

    if translation_array.shape != (3,):
        raise MeshKernelError(f"translation deve avere 3 componenti, trovato {translation_array.shape}")

    if not (np.all(np.isfinite(rotation_array)) and np.all(np.isfinite(translation_array))):
        raise MeshKernelError("rotation/translation contengono valori non finiti")

    return RigidTransform(rotation=rotation_array, translation=translation_array)


def identity_transform() -> RigidTransform:
    return RigidTransform(rotation=np.eye(3, dtype=np.float64), translation=np.zeros(3, dtype=np.float64))


# ---------------------------------------------------------------------------
# Broad phase: exact transformed AABB
# ---------------------------------------------------------------------------


def transformed_aabb(mesh: Mesh, transform: RigidTransform) -> Aabb:
    world_vertices = transform.apply_points(mesh.triangles_local.reshape(-1, 3))
    return Aabb(
        min_xyz=world_vertices.min(axis=0),
        max_xyz=world_vertices.max(axis=0),
    )


def aabb_separation(aabb_a: Aabb, aabb_b: Aabb) -> float:
    """Signed per-axis gap; positive means separated by that much, negative
    means overlapping on every axis (does not imply mesh collision)."""
    gap_low = aabb_b.min_xyz - aabb_a.max_xyz
    gap_high = aabb_a.min_xyz - aabb_b.max_xyz
    per_axis_gap = np.maximum(gap_low, gap_high)
    return float(np.max(per_axis_gap))


# ---------------------------------------------------------------------------
# Mid phase: convex-hull face-normal separating test
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class WorldBroadPhase:
    """Per-link broad/mid-phase geometry at one pose, computed once and
    reusable across every pair that link participates in. Checking one
    pose against `all_candidate_collision_pairs()` (120 pairs) would
    otherwise re-transform the same handful of large meshes (e.g.
    base_link) up to 15 times each; computing this once per link per pose
    was the main throughput fix once the OOM/candidate-explosion issues
    were resolved."""

    aabb: Aabb
    hull_vertices_world: np.ndarray
    hull_equations_world: np.ndarray


def world_broad_phase(mesh: Mesh, transform: RigidTransform) -> WorldBroadPhase:
    world_vertices = transform.apply_points(mesh.triangles_local.reshape(-1, 3))
    aabb = Aabb(min_xyz=world_vertices.min(axis=0), max_xyz=world_vertices.max(axis=0))

    hull_vertices_world = transform.apply_points(mesh.hull_vertices_local)

    normals_local = mesh.hull_equations_local[:, :3]
    offsets_local = mesh.hull_equations_local[:, 3]
    normals_world = normals_local @ transform.rotation.T
    offsets_world = offsets_local - np.einsum("ij,j->i", normals_world, transform.translation)
    hull_equations_world = np.concatenate([normals_world, offsets_world[:, None]], axis=1)

    return WorldBroadPhase(
        aabb=aabb,
        hull_vertices_world=hull_vertices_world,
        hull_equations_world=hull_equations_world,
    )


def hull_separation_margin_from_world(world_a: WorldBroadPhase, world_b: WorldBroadPhase) -> float | None:
    """Pure comparison over already-transformed hull geometry; see
    `hull_separation_margin` for the semantics and the convenience
    single-pair wrapper that also does the transform step."""
    best_margin: float | None = None

    for equations, other_points in (
        (world_a.hull_equations_world, world_b.hull_vertices_world),
        (world_b.hull_equations_world, world_a.hull_vertices_world),
    ):
        normals = equations[:, :3]
        offsets = equations[:, 3]
        signed = other_points @ normals.T + offsets
        margin_per_facet = signed.min(axis=0)
        candidate = float(margin_per_facet.max())

        if candidate > 0.0 and (best_margin is None or candidate > best_margin):
            best_margin = candidate

    return best_margin


def hull_separation_margin(
    mesh_a: Mesh,
    transform_a: RigidTransform,
    mesh_b: Mesh,
    transform_b: RigidTransform,
) -> float | None:
    """Return a positive separating margin in metres if some face normal of
    either hull proves separation; None if this face-normal-only test does
    not find a separating axis (edge/edge axes are not tested here, so a
    None result must fall back to the narrow phase rather than being read
    as proof of collision). Single-pair convenience wrapper: callers
    checking many pairs at the same pose should use `world_broad_phase` +
    `hull_separation_margin_from_world` instead to avoid re-transforming
    the same mesh once per pair it participates in."""
    return hull_separation_margin_from_world(
        world_broad_phase(mesh_a, transform_a),
        world_broad_phase(mesh_b, transform_b),
    )


# ---------------------------------------------------------------------------
# Narrow phase: k-d tree candidate pruning + exact triangle/triangle tests
# ---------------------------------------------------------------------------


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.array(
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    )


def _axis_separates(triangle_a: np.ndarray, triangle_b: np.ndarray, axis: np.ndarray, eps: float) -> bool:
    norm = np.linalg.norm(axis)

    if norm < eps:
        return False

    axis_unit = axis / norm
    proj_a = triangle_a @ axis_unit
    proj_b = triangle_b @ axis_unit

    return bool(proj_a.max() < proj_b.min() - eps or proj_b.max() < proj_a.min() - eps)


def _coplanar_triangle_overlap(
    triangle_a: np.ndarray,
    triangle_b: np.ndarray,
    plane_normal: np.ndarray,
    eps: float,
) -> bool:
    """2D separating-axis fallback for two triangles known to share a plane.

    The general 3D 11-axis SAT (2 face normals + 9 edge x edge cross
    products) is degenerate for coplanar triangles: every edge x edge cross
    product collapses onto the shared plane normal, so it cannot detect
    in-plane separation (Moeller 1997 notes the same limitation and uses an
    equivalent 2D projection fallback). For two coplanar convex polygons,
    testing only each polygon's own edge normals (2D SAT) is complete.
    """
    normal_unit = plane_normal / np.linalg.norm(plane_normal)
    reference = np.array([1.0, 0.0, 0.0])

    if abs(float(normal_unit @ reference)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])

    basis_u = np.cross(normal_unit, reference)
    basis_u /= np.linalg.norm(basis_u)
    basis_v = np.cross(normal_unit, basis_u)

    points_a_2d = np.stack([triangle_a @ basis_u, triangle_a @ basis_v], axis=1)
    points_b_2d = np.stack([triangle_b @ basis_u, triangle_b @ basis_v], axis=1)

    for points in (points_a_2d, points_b_2d):
        for i in range(3):
            edge = points[(i + 1) % 3] - points[i]
            axis_2d = np.array([-edge[1], edge[0]])
            norm = np.linalg.norm(axis_2d)

            if norm < eps:
                continue

            axis_unit = axis_2d / norm
            proj_a = points_a_2d @ axis_unit
            proj_b = points_b_2d @ axis_unit

            if proj_a.max() < proj_b.min() - eps or proj_b.max() < proj_a.min() - eps:
                return False

    return True


def triangle_triangle_overlap(triangle_a: np.ndarray, triangle_b: np.ndarray, eps: float = 1e-9) -> bool:
    """Exact separating-axis test between two triangles in 3D.

    Tests both face normals plus all nine edge x edge cross products (the
    standard 11-axis SAT for two non-coplanar triangles), with a 2D
    projected SAT fallback for the coplanar case where that 11-axis test is
    known to be incomplete (see `_coplanar_triangle_overlap`). Touching
    (shared boundary, zero gap) triangles are classified as overlapping,
    matching the ContactConfirmed-at-zero-clearance semantics used
    elsewhere in MATDOG.
    """
    edges_a = (
        triangle_a[1] - triangle_a[0],
        triangle_a[2] - triangle_a[1],
        triangle_a[0] - triangle_a[2],
    )
    edges_b = (
        triangle_b[1] - triangle_b[0],
        triangle_b[2] - triangle_b[1],
        triangle_b[0] - triangle_b[2],
    )

    normal_a = _cross(edges_a[0], edges_a[1])
    normal_b = _cross(edges_b[0], edges_b[1])
    norm_a = np.linalg.norm(normal_a)
    norm_b = np.linalg.norm(normal_b)

    # Degeneracy (near-zero triangle area) and the SAT axis/projection
    # tolerance are different physical quantities -- a cross-product norm
    # (area-like, units of length^2) versus a coordinate-scale tolerance
    # (units of length) -- and must not share `eps`. MATDOG's real
    # collision meshes legitimately contain small triangles with cross
    # norms around 1e-12; using `eps` (~1e-9, tuned for coordinate-scale
    # SAT comparisons) here would misclassify those as degenerate.
    if norm_a < _MIN_TRIANGLE_CROSS_NORM or norm_b < _MIN_TRIANGLE_CROSS_NORM:
        raise MeshKernelError("triangolo degenere (area nulla) passato al test SAT")

    normals_parallel = np.linalg.norm(_cross(normal_a / norm_a, normal_b / norm_b)) < eps
    coplanar_offset = abs(float((normal_a / norm_a) @ (triangle_b[0] - triangle_a[0])))

    if normals_parallel and coplanar_offset < eps:
        return _coplanar_triangle_overlap(triangle_a, triangle_b, normal_a, eps)

    axes = [normal_a, normal_b]

    for edge_a in edges_a:
        for edge_b in edges_b:
            axes.append(_cross(edge_a, edge_b))

    for axis in axes:
        if _axis_separates(triangle_a, triangle_b, axis, eps):
            return False

    return True


def _closest_point_segment_segment(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> float:
    """Minimum distance between segment p1-q1 and segment p2-q2."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2

    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ r)

    eps = 1e-15

    if a <= eps and e <= eps:
        return float(np.linalg.norm(p1 - p2))

    if a <= eps:
        t = np.clip(f / e, 0.0, 1.0)
        s = 0.0
    else:
        c = float(d1 @ r)

        if e <= eps:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(d1 @ d2)
            denominator = a * e - b * b

            if denominator > eps:
                s = np.clip((b * f - c * e) / denominator, 0.0, 1.0)
            else:
                s = 0.0

            t = (b * s + f) / e

            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    closest_1 = p1 + s * d1
    closest_2 = p2 + t * d2
    return float(np.linalg.norm(closest_1 - closest_2))


def _closest_point_point_triangle(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    """Ericson, 'Real-Time Collision Detection', ClosestPtPointTriangle."""
    a, b, c = triangle[0], triangle[1], triangle[2]

    ab = b - a
    ac = c - a
    ap = point - a

    d1 = float(ab @ ap)
    d2 = float(ac @ ap)

    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = point - b
    d3 = float(ab @ bp)
    d4 = float(ac @ bp)

    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2

    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return a + v * ab

    cp = point - c
    d5 = float(ab @ cp)
    d6 = float(ac @ cp)

    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6

    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return a + w * ac

    va = d3 * d6 - d5 * d4

    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + w * (c - b)

    denominator = 1.0 / (va + vb + vc)
    v = vb * denominator
    w = vc * denominator

    return a + ab * v + ac * w


def triangle_triangle_distance(triangle_a: np.ndarray, triangle_b: np.ndarray) -> float:
    """Exact minimum distance between two (non-degenerate) triangles.

    Only meaningful when the triangles do not overlap; callers should check
    triangle_triangle_overlap first.
    """
    best = float("inf")

    for i in range(3):
        edge_a = (triangle_a[i], triangle_a[(i + 1) % 3])

        for j in range(3):
            edge_b = (triangle_b[j], triangle_b[(j + 1) % 3])
            distance = _closest_point_segment_segment(edge_a[0], edge_a[1], edge_b[0], edge_b[1])
            best = min(best, distance)

    for vertex in triangle_a:
        closest = _closest_point_point_triangle(vertex, triangle_b)
        best = min(best, float(np.linalg.norm(vertex - closest)))

    for vertex in triangle_b:
        closest = _closest_point_point_triangle(vertex, triangle_a)
        best = min(best, float(np.linalg.norm(vertex - closest)))

    return best


def _triangle_grid_index(
    triangles_world: np.ndarray,
    cell_size_m: float,
    inflate_m: float,
) -> dict[tuple[int, int, int], list[int]]:
    """Bin triangles into a fixed-size uniform spatial hash grid by AABB.

    Each triangle is inserted into every grid cell its (margin-inflated)
    AABB overlaps. `cell_size_m` is a fixed, mesh-content-independent
    constant chosen purely for query efficiency: correctness (never missing
    a candidate pair whose true gap is <= the combined inflation) does not
    depend on its value, only performance does. This is deliberately not
    derived from any per-mesh statistic (e.g. a max triangle radius), which
    is what let a single oversized triangle blow up the previous k-d tree
    radius query to a near-quadratic candidate count.
    """
    aabb_min = triangles_world.min(axis=1) - inflate_m
    aabb_max = triangles_world.max(axis=1) + inflate_m

    cell_min = np.floor(aabb_min / cell_size_m).astype(np.int64)
    cell_max = np.floor(aabb_max / cell_size_m).astype(np.int64)

    # Fail fast, before running the (potentially large) Python binning
    # loop below, if the geometry/cell-size combination would produce a
    # pathological number of grid entries -- e.g. a mesh accidentally
    # loaded in the wrong units (millimetres instead of the expected
    # metres) relative to this fixed cell size. This is defence in depth
    # on top of correct unit handling elsewhere, not a substitute for it.
    span = (cell_max - cell_min + 1).astype(np.int64)
    cells_per_triangle = span[:, 0] * span[:, 1] * span[:, 2]

    if cells_per_triangle.size and int(cells_per_triangle.max()) > _MAX_GRID_CELLS_PER_TRIANGLE:
        raise MeshKernelError(
            f"un triangolo occupa {int(cells_per_triangle.max())} celle griglia "
            f"(limite {_MAX_GRID_CELLS_PER_TRIANGLE}); verificare scala/unita della "
            f"mesh rispetto a cell_size_m={cell_size_m}"
        )

    total_entries = int(cells_per_triangle.sum())

    if total_entries > _MAX_GRID_TOTAL_ENTRIES:
        raise MeshKernelError(
            f"il binning griglia richiederebbe {total_entries} inserimenti "
            f"(limite {_MAX_GRID_TOTAL_ENTRIES}); verificare scala/unita della "
            f"mesh rispetto a cell_size_m={cell_size_m}"
        )

    grid: dict[tuple[int, int, int], list[int]] = {}

    for triangle_index in range(triangles_world.shape[0]):
        x0, y0, z0 = cell_min[triangle_index]
        x1, y1, z1 = cell_max[triangle_index]

        for x in range(int(x0), int(x1) + 1):
            for y in range(int(y0), int(y1) + 1):
                for z in range(int(z0), int(z1) + 1):
                    grid.setdefault((x, y, z), []).append(triangle_index)

    return grid


def _grid_candidate_pairs(
    grid_a: dict[tuple[int, int, int], list[int]],
    grid_b: dict[tuple[int, int, int], list[int]],
    max_candidate_pairs: int,
    pair_description: str,
):
    """Yield each distinct (index_a, index_b) candidate pair lazily.

    Never materialises the full candidate set: a caller that stops
    iterating early (e.g. on the first confirmed intersection) never pays
    for pairs beyond that point, and `seen` -- the only structure that
    grows -- is capped incrementally rather than after the fact. This is
    what keeps a densely populated single cell (e.g. checking a mesh
    against an exact copy of itself, where one cell can hold thousands of
    mutually touching triangles) from ever allocating an all-pairs
    structure: the very first pair tried in such a cell is expected to
    already be a hit, so the caller returns before `seen` grows further.
    """
    seen: set[tuple[int, int]] = set()
    shared_cells = grid_a.keys() & grid_b.keys()

    for cell in shared_cells:
        for index_a in grid_a[cell]:
            for index_b in grid_b[cell]:
                pair = (index_a, index_b)

                if pair in seen:
                    continue

                if len(seen) >= max_candidate_pairs:
                    raise MeshKernelError(
                        f"{pair_description}: oltre {max_candidate_pairs} coppie "
                        "candidate narrow-phase senza una risoluzione; aumentare "
                        "grid_cell_size_m o ridurre margin_m invece di procedere "
                        "con una struttura non limitata in memoria."
                    )

                seen.add(pair)
                yield pair


_CANDIDATE_BATCH_SIZE = 20_000
"""Bounded batch size for vectorised SAT overlap testing in narrow phase.
Candidates are still consumed lazily from `_grid_candidate_pairs` (so the
`max_candidate_pairs` cap and early-exit-on-intersection behaviour are
unchanged); only up to this many pairs are ever materialised into arrays
at once, keeping peak memory bounded regardless of how many candidates a
link pair ultimately has."""


def _batched_triangle_overlap_mask(
    triangles_a: np.ndarray,
    triangles_b: np.ndarray,
    eps: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised 11-axis SAT over a batch of (triangle_a, triangle_b) pairs.

    `triangles_a`/`triangles_b` are (N, 3, 3) arrays: N pairs, 3 vertices,
    3 coordinates. Mirrors `triangle_triangle_overlap`'s per-pair logic
    exactly, but as batched numpy operations, so the per-call Python/numpy
    dispatch overhead (the dominant cost for the scalar version, ~130
    microseconds/pair even for the overlap test alone) is paid once per
    batch instead of once per pair.

    Returns (not_separated, is_coplanar):
    - `not_separated[i]` is True iff no non-degenerate axis among the 11
      candidates separates pair i (i.e. the 3D SAT alone reports overlap).
    - `is_coplanar[i]` flags pairs where that 3D SAT is known-incomplete
      (see `_coplanar_triangle_overlap`); callers must resolve those pairs
      exactly instead of trusting `not_separated` for them.
    """
    edges_a = np.stack(
        [triangles_a[:, 1] - triangles_a[:, 0], triangles_a[:, 2] - triangles_a[:, 1], triangles_a[:, 0] - triangles_a[:, 2]],
        axis=1,
    )  # (N, 3, 3): edge index, xyz
    edges_b = np.stack(
        [triangles_b[:, 1] - triangles_b[:, 0], triangles_b[:, 2] - triangles_b[:, 1], triangles_b[:, 0] - triangles_b[:, 2]],
        axis=1,
    )

    normal_a = np.cross(edges_a[:, 0], edges_a[:, 1])  # (N, 3)
    normal_b = np.cross(edges_b[:, 0], edges_b[:, 1])  # (N, 3)

    edge_cross = np.cross(edges_a[:, :, None, :], edges_b[:, None, :, :]).reshape(-1, 9, 3)  # (N, 9, 3)
    axes = np.concatenate([normal_a[:, None, :], normal_b[:, None, :], edge_cross], axis=1)  # (N, 11, 3)

    axis_norms = np.linalg.norm(axes, axis=2)  # (N, 11)
    degenerate_axis = axis_norms < eps
    safe_norms = np.where(degenerate_axis, 1.0, axis_norms)
    axes_unit = axes / safe_norms[:, :, None]

    proj_a = np.einsum("nvj,naj->nva", triangles_a, axes_unit)  # (N, 3 vertices, 11 axes)
    proj_b = np.einsum("nvj,naj->nva", triangles_b, axes_unit)

    max_a, min_a = proj_a.max(axis=1), proj_a.min(axis=1)  # (N, 11)
    max_b, min_b = proj_b.max(axis=1), proj_b.min(axis=1)

    separated_by_axis = ((max_a < min_b - eps) | (max_b < min_a - eps)) & ~degenerate_axis
    not_separated = ~separated_by_axis.any(axis=1)

    normal_a_norm = np.linalg.norm(normal_a, axis=1)
    normal_b_norm = np.linalg.norm(normal_b, axis=1)
    unit_normal_a = normal_a / normal_a_norm[:, None]
    unit_normal_b = normal_b / normal_b_norm[:, None]
    normals_parallel = np.linalg.norm(np.cross(unit_normal_a, unit_normal_b), axis=1) < eps
    coplanar_offset = np.abs(np.einsum("nj,nj->n", unit_normal_a, triangles_b[:, 0] - triangles_a[:, 0]))
    is_coplanar = normals_parallel & (coplanar_offset < eps)

    return not_separated, is_coplanar


def _resolve_batch_overlap(
    triangles_a: np.ndarray,
    triangles_b: np.ndarray,
    eps: float,
) -> np.ndarray:
    """Exact per-pair overlap boolean for a batch, matching
    `triangle_triangle_overlap` precisely: the vectorised 11-axis SAT
    resolves the (expected to be large majority of) non-coplanar pairs,
    and only the (expected rare) coplanar-flagged subset falls back to the
    exact scalar 2D-projected test."""
    not_separated, is_coplanar = _batched_triangle_overlap_mask(triangles_a, triangles_b, eps)
    overlap = not_separated.copy()

    coplanar_indices = np.flatnonzero(is_coplanar)

    for index in coplanar_indices:
        overlap[index] = triangle_triangle_overlap(triangles_a[index], triangles_b[index], eps)

    return overlap


def _iter_candidate_batches(
    grid_a: dict[tuple[int, int, int], list[int]],
    grid_b: dict[tuple[int, int, int], list[int]],
    max_candidate_pairs: int,
    pair_description: str,
    batch_size: int,
):
    """Group `_grid_candidate_pairs` into bounded-size (index_a, index_b)
    batches for vectorised processing, without ever materialising more
    than `batch_size` pairs at once."""
    batch_a: list[int] = []
    batch_b: list[int] = []

    for index_a, index_b in _grid_candidate_pairs(grid_a, grid_b, max_candidate_pairs, pair_description):
        batch_a.append(index_a)
        batch_b.append(index_b)

        if len(batch_a) >= batch_size:
            yield np.array(batch_a), np.array(batch_b)
            batch_a, batch_b = [], []

    if batch_a:
        yield np.array(batch_a), np.array(batch_b)


def narrow_phase_check(
    mesh_a: Mesh,
    transform_a: RigidTransform,
    mesh_b: Mesh,
    transform_b: RigidTransform,
    *,
    margin_m: float = DEFAULT_NARROW_PHASE_MARGIN_M,
    grid_cell_size_m: float = DEFAULT_GRID_CELL_SIZE_M,
    max_candidate_pairs: int = DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
    require_distance: bool = True,
    sat_eps: float = 1e-9,
) -> PairCollisionResult:
    """Authoritative exact narrow-phase check between two full triangle
    meshes, pruned with a fixed-cell-size uniform spatial hash grid (a flat
    single-level BVH broad phase) so a handful of oversized triangles in
    one mesh cannot inflate the candidate set for the rest of the mesh.

    Candidate pairs are generated lazily and processed in bounded-size
    batches (`_CANDIDATE_BATCH_SIZE`) through a vectorised SAT test, with
    an immediate return on the first confirmed intersection.

    `require_distance=False` (the fast path used by bisection-style
    contact search, which only needs a collision boolean, not an exact
    clearance figure) skips the per-pair exact minimum-distance
    computation entirely -- that computation dominates narrow-phase cost
    an order of magnitude more than the overlap test alone, and is not
    needed just to confirm SEPARATED. In that mode a SEPARATED_NARROW
    result reports `clearance_m=None` rather than a real distance.
    """
    triangles_a_world = transform_a.apply_points(mesh_a.triangles_local.reshape(-1, 3)).reshape(-1, 3, 3)
    triangles_b_world = transform_b.apply_points(mesh_b.triangles_local.reshape(-1, 3)).reshape(-1, 3, 3)

    half_margin = margin_m / 2.0
    grid_a = _triangle_grid_index(triangles_a_world, grid_cell_size_m, half_margin)
    grid_b = _triangle_grid_index(triangles_b_world, grid_cell_size_m, half_margin)

    pair_description = f"{mesh_a.name} vs {mesh_b.name}"
    best_distance = float("inf")
    best_pair: tuple[int, int] | None = None

    for indices_a, indices_b in _iter_candidate_batches(
        grid_a, grid_b, max_candidate_pairs, pair_description, _CANDIDATE_BATCH_SIZE
    ):
        batch_triangles_a = triangles_a_world[indices_a]
        batch_triangles_b = triangles_b_world[indices_b]

        overlap_mask = _resolve_batch_overlap(batch_triangles_a, batch_triangles_b, sat_eps)
        hit_positions = np.flatnonzero(overlap_mask)

        if hit_positions.size:
            hit = int(hit_positions[0])
            return PairCollisionResult(
                link_a=mesh_a.name,
                link_b=mesh_b.name,
                status="INTERSECTING",
                clearance_m=None,
                clearance_kind=None,
                witness_triangle_a=int(indices_a[hit]),
                witness_triangle_b=int(indices_b[hit]),
            )

        if not require_distance:
            continue

        for index_a, index_b in zip(indices_a.tolist(), indices_b.tolist()):
            distance = triangle_triangle_distance(triangles_a_world[index_a], triangles_b_world[index_b])

            if distance < best_distance:
                best_distance = distance
                best_pair = (index_a, index_b)

    if not require_distance:
        return PairCollisionResult(
            link_a=mesh_a.name,
            link_b=mesh_b.name,
            status="SEPARATED_NARROW",
            clearance_m=None,
            clearance_kind=None,
        )

    if best_pair is None:
        # No candidate pairs at all within the inflated grid search: the
        # true minimum distance is at least the grid search reach, but
        # report that bound explicitly (LOWER_BOUND) rather than an
        # unfounded exact figure -- this margin_m value must never be read
        # as "the clearance is margin_m", only as "the clearance is at
        # least margin_m" (canonical handoff section 6).
        return PairCollisionResult(
            link_a=mesh_a.name,
            link_b=mesh_b.name,
            status="SEPARATED_NARROW",
            clearance_m=margin_m,
            clearance_kind="LOWER_BOUND",
        )

    return PairCollisionResult(
        link_a=mesh_a.name,
        link_b=mesh_b.name,
        status="SEPARATED_NARROW",
        clearance_m=best_distance,
        clearance_kind="EXACT",
        witness_triangle_a=best_pair[0],
        witness_triangle_b=best_pair[1],
    )


# ---------------------------------------------------------------------------
# Cascade entry point
# ---------------------------------------------------------------------------


def check_pair_from_world(
    mesh_a: Mesh,
    transform_a: RigidTransform,
    world_a: WorldBroadPhase,
    mesh_b: Mesh,
    transform_b: RigidTransform,
    world_b: WorldBroadPhase,
    *,
    narrow_phase_margin_m: float = DEFAULT_NARROW_PHASE_MARGIN_M,
    grid_cell_size_m: float = DEFAULT_GRID_CELL_SIZE_M,
    max_candidate_pairs: int = DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
    require_distance: bool = True,
) -> PairCollisionResult:
    """Run the full broad -> mid -> narrow cascade for one link pair, reusing
    already-computed per-link world broad/mid-phase geometry (see
    `world_broad_phase`). Preferred over `check_pair` whenever the same
    link is checked against several others at the same pose (e.g. scanning
    all candidate pairs at one sampled pose), since it avoids re-computing
    that link's world AABB/hull once per pair.

    `require_distance=False` propagates to `narrow_phase_check`'s fast
    collision-boolean-only path (see there); use it for bulk collision
    sweeps (coarse angular scans, bisection brackets) where only a
    collision/no-collision verdict is needed, and keep the default True
    for the final, small number of calls where an actual clearance figure
    must be reported.
    """
    aabb_gap = aabb_separation(world_a.aabb, world_b.aabb)

    if aabb_gap > 0.0:
        return PairCollisionResult(
            link_a=mesh_a.name,
            link_b=mesh_b.name,
            status="SEPARATED_AABB",
            clearance_m=aabb_gap,
            # Exact gap between the two boxes, but the enclosed mesh
            # surfaces can be recessed further inside -- a safe lower
            # bound on true surface-to-surface distance, not the distance
            # itself.
            clearance_kind="LOWER_BOUND",
        )

    hull_margin = hull_separation_margin_from_world(world_a, world_b)

    if hull_margin is not None:
        return PairCollisionResult(
            link_a=mesh_a.name,
            link_b=mesh_b.name,
            status="SEPARATED_HULL",
            clearance_m=hull_margin,
            # Face-normal-only separating axis: a valid lower bound on
            # true hull distance, but edge/edge closest-approach axes are
            # not tested, so this can under-estimate the true separation.
            clearance_kind="LOWER_BOUND",
        )

    return narrow_phase_check(
        mesh_a,
        transform_a,
        mesh_b,
        transform_b,
        margin_m=narrow_phase_margin_m,
        grid_cell_size_m=grid_cell_size_m,
        max_candidate_pairs=max_candidate_pairs,
        require_distance=require_distance,
    )


def check_pair(
    mesh_a: Mesh,
    transform_a: RigidTransform,
    mesh_b: Mesh,
    transform_b: RigidTransform,
    *,
    narrow_phase_margin_m: float = DEFAULT_NARROW_PHASE_MARGIN_M,
    grid_cell_size_m: float = DEFAULT_GRID_CELL_SIZE_M,
    max_candidate_pairs: int = DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
    require_distance: bool = True,
) -> PairCollisionResult:
    """Run the full broad -> mid -> narrow cascade for one link pair.
    Single-pair convenience wrapper around `check_pair_from_world`; callers
    checking many pairs at the same pose should precompute `world_broad_phase`
    per link once and call `check_pair_from_world` directly instead."""
    return check_pair_from_world(
        mesh_a,
        transform_a,
        world_broad_phase(mesh_a, transform_a),
        mesh_b,
        transform_b,
        world_broad_phase(mesh_b, transform_b),
        narrow_phase_margin_m=narrow_phase_margin_m,
        grid_cell_size_m=grid_cell_size_m,
        max_candidate_pairs=max_candidate_pairs,
        require_distance=require_distance,
    )
