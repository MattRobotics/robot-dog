#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler full-body scene: URDF FK + collision-mesh manifest.

Bridges the canonical URDF (topology, joint limits, per-link collision mesh
references) and the reusable per-leg forward-kinematics module in
06_Software/Matdog_Core/kinematics/matdog_urdf_fk.py with the collision
kernel in matdog_geometry_mesh_kernel.py, so the rest of the Geometry
Compiler can ask "where is every link, and does this pair of links touch"
without repeating URDF or STL parsing.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from functools import lru_cache
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

CALIBRATION_DIR = Path(__file__).resolve().parent
KINEMATICS_DIR = CALIBRATION_DIR.parents[0] / "kinematics"

for _extra_path in (KINEMATICS_DIR, CALIBRATION_DIR):
    if str(_extra_path) not in sys.path:
        sys.path.insert(0, str(_extra_path))

from matdog_urdf_fk import (  # noqa: E402
    CANONICAL_URDF_RELATIVE_PATH,
    canonical_urdf_path,
    forward_kinematics,
    load_urdf_joints,
)

from matdog_geometry_mesh_kernel import (  # noqa: E402
    BOOLEAN_COLLISION_MARGIN_M,
    DEFAULT_NARROW_PHASE_MARGIN_M,
    Mesh,
    PairCollisionResult,
    RigidTransform,
    WorldBroadPhase,
    check_pair_from_world,
    identity_transform,
    load_mesh,
    make_transform,
    world_broad_phase,
)


REPO_ROOT = CALIBRATION_DIR.parents[2]

URDF_MESH_SCALE = (0.001, 0.001, 0.001)
"""Every collision mesh in the canonical URDF applies this scale (STL files
are authored in millimetres); verified against the URDF text at module
import indirectly through `load_link_mesh_manifest`."""

LEG_IDS: tuple[str, ...] = ("lf", "rf", "rh", "lh")
JOINT_GROUPS: tuple[str, ...] = ("hip", "upper_leg", "lower_leg")

FRONT_LEGS: tuple[str, ...] = ("lf", "rf")
HIND_LEGS: tuple[str, ...] = ("rh", "lh")

MIRROR_PAIRS: tuple[tuple[str, str], ...] = (("lf", "rf"), ("rh", "lh"))

SERVO_ID_BY_JOINT: dict[str, int] = {
    "lf_hip_joint": 13, "lf_upper_leg_joint": 12, "lf_lower_leg_joint": 11,
    "rf_hip_joint": 23, "rf_upper_leg_joint": 22, "rf_lower_leg_joint": 21,
    "rh_hip_joint": 33, "rh_upper_leg_joint": 32, "rh_lower_leg_joint": 31,
    "lh_hip_joint": 43, "lh_upper_leg_joint": 42, "lh_lower_leg_joint": 41,
}
"""Canonical servo mapping from
MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md, cross-checked
against the URDF <hardware><motorId> tags in `validate_servo_mapping`."""


class GeometrySceneError(RuntimeError):
    """Errore nello strato scena/FK del Geometry Compiler."""


def leg_of_link(link_name: str) -> str | None:
    if link_name == "base_link":
        return None

    prefix = link_name.split("_", 1)[0]

    if prefix in LEG_IDS:
        return prefix

    raise GeometrySceneError(f"link non riconosciuto: {link_name!r}")


def leg_of_joint(joint_name: str) -> str:
    prefix = joint_name.split("_", 1)[0]

    if prefix not in LEG_IDS:
        raise GeometrySceneError(f"joint non riconosciuto: {joint_name!r}")

    return prefix


def joint_name(leg_id: str, joint_group: str) -> str:
    if leg_id not in LEG_IDS:
        raise GeometrySceneError(f"leg_id non valido: {leg_id!r}")

    if joint_group not in JOINT_GROUPS:
        raise GeometrySceneError(f"joint_group non valido: {joint_group!r}")

    return f"{leg_id}_{joint_group}_joint"


def link_name(leg_id: str, joint_group: str) -> str:
    return f"{leg_id}_{joint_group}_link"


ALL_JOINT_NAMES: tuple[str, ...] = tuple(
    joint_name(leg, group) for leg in LEG_IDS for group in JOINT_GROUPS
)

ALL_LINKS: tuple[str, ...] = ("base_link",) + tuple(
    f"{leg}_{group}_link" for leg in LEG_IDS for group in ("hip", "upper_leg", "lower_leg", "foot")
)


# ---------------------------------------------------------------------------
# Joint-aware link adjacency (Phase 1B)
# ---------------------------------------------------------------------------
#
# The Phase 1 v3 policy was `adjacent pair -> EXCLUDE`, applied uniformly. That
# is superseded: it treated a REVOLUTE hinge (where the designed mechanical
# hardstop physically lives) and a FIXED structural attachment as the same kind
# of thing, which made the real endstop event unobservable by construction
# (GATE A, 2026-08-08). Adjacency and joint type are now derived from the URDF
# topology rather than from a hard-coded chain, so a future URDF revision
# cannot silently invalidate the classification.

PAIR_CLASS_REVOLUTE_ADJACENT = "REVOLUTE_ADJACENT"
"""Parent/child joined by a revolute joint: a real articulation, whose own
geometry carries the designed MIN/MAX hardstop. Included in collision analysis."""

PAIR_CLASS_FIXED_ADJACENT = "FIXED_ADJACENT"
"""Parent/child joined by a fixed joint (MATDOG: lower_leg <-> foot). A
structural attachment, not two independently moving bodies; excluded from both
endstop metrology and path collision."""

PAIR_CLASS_NON_ADJACENT = "NON_ADJACENT"
"""Any other pair. Retains the pre-existing clearance/path-safety policy
unchanged."""


@dataclass(frozen=True)
class LinkAdjacency:
    """URDF-derived adjacency, keyed by unordered link pair."""

    joint_type_by_pair: dict[frozenset[str], str]
    joint_name_by_pair: dict[frozenset[str], str]
    ordered_pair_by_joint: dict[str, tuple[str, str]]
    """joint name -> (parent_link, child_link), in URDF order."""

    def classify(self, link_a: str, link_b: str) -> str:
        joint_type = self.joint_type_by_pair.get(frozenset((link_a, link_b)))

        if joint_type is None:
            return PAIR_CLASS_NON_ADJACENT
        if joint_type == "fixed":
            return PAIR_CLASS_FIXED_ADJACENT
        return PAIR_CLASS_REVOLUTE_ADJACENT

    @property
    def revolute_pairs(self) -> frozenset[frozenset[str]]:
        return frozenset(p for p, t in self.joint_type_by_pair.items() if t != "fixed")

    @property
    def fixed_pairs(self) -> frozenset[frozenset[str]]:
        return frozenset(p for p, t in self.joint_type_by_pair.items() if t == "fixed")

    @property
    def all_adjacent_pairs(self) -> frozenset[frozenset[str]]:
        return frozenset(self.joint_type_by_pair)


@lru_cache(maxsize=4)
def load_link_adjacency(urdf_path: Path | None = None) -> LinkAdjacency:
    """Build the adjacency map straight from the URDF joint topology."""
    path = Path(urdf_path) if urdf_path is not None else canonical_urdf_path(REPO_ROOT)
    joints = load_urdf_joints(path)

    joint_type_by_pair: dict[frozenset[str], str] = {}
    joint_name_by_pair: dict[frozenset[str], str] = {}
    ordered_pair_by_joint: dict[str, tuple[str, str]] = {}

    for name, joint in joints.items():
        pair = frozenset((joint.parent_link, joint.child_link))
        joint_type_by_pair[pair] = joint.joint_type
        joint_name_by_pair[pair] = name
        ordered_pair_by_joint[name] = (joint.parent_link, joint.child_link)

    return LinkAdjacency(joint_type_by_pair, joint_name_by_pair, ordered_pair_by_joint)


def is_adjacent_pair(link_a: str, link_b: str, adjacency: LinkAdjacency | None = None) -> bool:
    """True for any parent/child pair, revolute or fixed. Kept because
    'are these two links directly connected' is still a meaningful question;
    it is no longer the exclusion criterion on its own."""
    adjacency = adjacency or load_link_adjacency()
    return frozenset((link_a, link_b)) in adjacency.joint_type_by_pair


def classify_link_pair(link_a: str, link_b: str, adjacency: LinkAdjacency | None = None) -> str:
    adjacency = adjacency or load_link_adjacency()
    return adjacency.classify(link_a, link_b)


def active_revolute_contact_pair(joint_name: str, adjacency: LinkAdjacency | None = None) -> tuple[str, str]:
    """The single primary endstop-contact pair for `joint_name`.

    ENDSTOP METROLOGY is defined on exactly one pair per joint -- the revolute
    parent/child the joint itself articulates:

        HIP   -> base_link            <-> <leg>_hip_link
        UPPER -> <leg>_hip_link       <-> <leg>_upper_leg_link
        LOWER -> <leg>_upper_leg_link <-> <leg>_lower_leg_link

    Derived from URDF `<parent>`/`<child>`, never from a hard-coded table.
    """
    adjacency = adjacency or load_link_adjacency()
    pair = adjacency.ordered_pair_by_joint.get(joint_name)

    if pair is None:
        raise GeometrySceneError(f"joint sconosciuto nell'URDF: {joint_name!r}")

    if adjacency.joint_type_by_pair[frozenset(pair)] == "fixed":
        raise GeometrySceneError(
            f"{joint_name} e' un joint fixed: non ha una coppia di contatto di finecorsa"
        )

    return pair


def path_safety_pairs(
    exclude_pair: tuple[str, str] | None = None,
    adjacency: LinkAdjacency | None = None,
) -> tuple[tuple[str, str], ...]:
    """PATH SAFETY candidate set: everything whose intersection means "something
    is in the way", as opposed to "this joint reached its limit".

    Includes every NON_ADJACENT pair *and* every REVOLUTE_ADJACENT pair that is
    not the one currently being measured. Excludes FIXED_ADJACENT pairs
    (structural attachments) always, and excludes `exclude_pair` -- during a
    joint's own endpoint sweep its active revolute pair's contact is the desired
    result, not an obstruction, so counting it here would make every endpoint
    report a path collision at its own endstop.

    The historical regression this protects: while probing LOWER, the primary
    pair is upper<->lower; a hip<->foot intersection appearing earlier is a
    PATH_COLLISION_BEFORE_ENDPOINT, never the LOWER endstop.
    """
    adjacency = adjacency or load_link_adjacency()
    excluded = frozenset(exclude_pair) if exclude_pair is not None else None
    pairs = []

    for i, link_a in enumerate(ALL_LINKS):
        for link_b in ALL_LINKS[i + 1 :]:
            key = frozenset((link_a, link_b))

            if excluded is not None and key == excluded:
                continue
            if adjacency.classify(link_a, link_b) == PAIR_CLASS_FIXED_ADJACENT:
                continue

            pairs.append((link_a, link_b))

    return tuple(pairs)


def clearance_policy_pairs(adjacency: LinkAdjacency | None = None) -> tuple[tuple[str, str], ...]:
    """Pairs the generic minimum-clearance gate may be applied to: NON_ADJACENT
    only.

    Revolute adjacent pairs are deliberately absent. GATE B measured their real
    mating clearances at q=0 as 0.0372 mm (HIP), 0.0026 mm (UPPER) and
    0.0101 mm (LOWER): sub-millimetre separation is the *correct* state for a
    hinge, so subjecting them to the generic 3 mm gate would flag normal,
    healthy geometry as unsafe. Their contribution to path safety is the boolean
    intersection test only (see `path_safety_pairs`); the non-adjacent clearance
    policy itself is unchanged.
    """
    adjacency = adjacency or load_link_adjacency()
    return tuple(
        (a, b)
        for i, a in enumerate(ALL_LINKS)
        for b in ALL_LINKS[i + 1 :]
        if adjacency.classify(a, b) == PAIR_CLASS_NON_ADJACENT
    )


def all_candidate_collision_pairs() -> tuple[tuple[str, str], ...]:
    """Default collision candidate set = `path_safety_pairs()` with nothing
    excluded (all non-adjacent plus all revolute adjacent pairs)."""
    return path_safety_pairs()


def same_leg_non_adjacent_pairs(leg_id: str) -> tuple[tuple[str, str], ...]:
    """One leg's own NON-ADJACENT pairs (base_link plus this leg's links).

    HISTORICAL / DIAGNOSTIC ONLY as of Phase 1B. This was the v3
    ENDSTOP_CONTACT_POLICY candidate set; it is no longer used to identify a
    joint limit, because the pairs it excludes -- the revolute parent/child
    ones -- are exactly where the real hardstop lives (GATE A/GATE B). Endstop
    metrology now uses `active_revolute_contact_pair`. Retained so historical
    v3 findings (base<->upper at -47.500 deg, upper<->foot at -97.957 deg)
    remain reproducible and comparable.
    """
    adjacency = load_link_adjacency()
    leg_links = (
        "base_link",
        link_name(leg_id, "hip"),
        link_name(leg_id, "upper_leg"),
        link_name(leg_id, "lower_leg"),
        link_name(leg_id, "foot"),
    )
    pairs = []

    for i, link_a in enumerate(leg_links):
        for link_b in leg_links[i + 1 :]:
            if adjacency.classify(link_a, link_b) == PAIR_CLASS_NON_ADJACENT:
                pairs.append((link_a, link_b))

    return tuple(pairs)


def __getattr__(name: str):
    # Back-compat for the pre-Phase-1B module constant, now URDF-derived.
    if name == "ADJACENT_LINK_PAIRS":
        return load_link_adjacency().all_adjacent_pairs
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def pair_is_cross_leg(link_a: str, link_b: str) -> bool:
    leg_a = leg_of_link(link_a)
    leg_b = leg_of_link(link_b)
    return leg_a is not None and leg_b is not None and leg_a != leg_b


def home_pose() -> dict[str, float]:
    return {name: 0.0 for name in ALL_JOINT_NAMES}


def full_pose(overrides: dict[str, float]) -> dict[str, float]:
    unknown = sorted(set(overrides) - set(ALL_JOINT_NAMES))

    if unknown:
        raise GeometrySceneError(f"joint sconosciuti in overrides: {unknown}")

    pose = home_pose()
    pose.update(overrides)
    return pose


def leg_pose_overrides(leg_id: str, hip_rad: float, upper_rad: float, lower_rad: float) -> dict[str, float]:
    return {
        joint_name(leg_id, "hip"): hip_rad,
        joint_name(leg_id, "upper_leg"): upper_rad,
        joint_name(leg_id, "lower_leg"): lower_rad,
    }


@dataclass(frozen=True)
class LinkMeshEntry:
    link_name: str
    stl_relative_path: str
    scale: tuple[float, float, float]
    collision_origin_xyz: tuple[float, float, float]
    collision_origin_rpy: tuple[float, float, float]


def load_link_mesh_manifest(urdf_path: Path) -> dict[str, LinkMeshEntry]:
    """Parse the <collision> mesh reference of every link from the URDF.

    Fails loudly if any collision origin is non-identity or any two links
    use a different scale, since the rest of the Geometry Compiler assumes
    that (true for the canonical REV00 URDF as of this writing; if a future
    URDF revision changes this, the compiler must not silently mis-place a
    mesh).
    """
    root = ET.parse(urdf_path).getroot()
    manifest: dict[str, LinkMeshEntry] = {}

    for link_node in root.findall("link"):
        name = str(link_node.attrib["name"])
        collision = link_node.find("collision")

        if collision is None:
            continue

        mesh_node = collision.find("./geometry/mesh")

        if mesh_node is None:
            continue

        filename = str(mesh_node.attrib["filename"])
        scale_raw = mesh_node.attrib.get("scale", "1 1 1")
        scale = tuple(float(token) for token in scale_raw.split())

        if scale != URDF_MESH_SCALE:
            raise GeometrySceneError(
                f"{name}: scale mesh inatteso {scale}; atteso {URDF_MESH_SCALE}. "
                "Il Geometry Compiler assume scala uniforme su tutti i link."
            )

        origin_node = collision.find("origin")
        origin_xyz = (0.0, 0.0, 0.0)
        origin_rpy = (0.0, 0.0, 0.0)

        if origin_node is not None:
            origin_xyz = tuple(float(v) for v in origin_node.attrib.get("xyz", "0 0 0").split())
            origin_rpy = tuple(float(v) for v in origin_node.attrib.get("rpy", "0 0 0").split())

        if origin_xyz != (0.0, 0.0, 0.0) or origin_rpy != (0.0, 0.0, 0.0):
            raise GeometrySceneError(
                f"{name}: origin collisione non identità {origin_xyz}/{origin_rpy}; "
                "il Geometry Compiler assume origin collisione = identità su tutti i link."
            )

        manifest[name] = LinkMeshEntry(
            link_name=name,
            stl_relative_path=filename,
            scale=scale,  # type: ignore[arg-type]
            collision_origin_xyz=origin_xyz,  # type: ignore[arg-type]
            collision_origin_rpy=origin_rpy,  # type: ignore[arg-type]
        )

    missing = sorted(set(ALL_LINKS) - set(manifest))

    if missing:
        raise GeometrySceneError(f"URDF: mesh di collisione mancante per {missing}")

    return manifest


def validate_servo_mapping(urdf_path: Path) -> None:
    root = ET.parse(urdf_path).getroot()

    for joint_node in root.findall("joint"):
        name = joint_node.get("name")

        if name not in SERVO_ID_BY_JOINT:
            continue

        hardware = joint_node.find("hardware")

        if hardware is None:
            raise GeometrySceneError(f"{name}: tag <hardware> mancante nel URDF")

        motor_id_node = hardware.find("motorId")

        if motor_id_node is None or motor_id_node.text is None:
            raise GeometrySceneError(f"{name}: motorId mancante nel URDF")

        urdf_motor_id = int(motor_id_node.text.strip())
        expected_motor_id = SERVO_ID_BY_JOINT[name]

        if urdf_motor_id != expected_motor_id:
            raise GeometrySceneError(
                f"{name}: motorId URDF {urdf_motor_id} diverso dalla mappa "
                f"canonica del checkpoint 2026-07-20 ({expected_motor_id})"
            )


def _relevant_joints_for_link(link: str) -> tuple[str, ...]:
    """Joint names a link's world transform actually depends on (its own
    kinematic chain from base_link). Used to key the FK cache narrowly so
    unrelated joints (e.g. a different leg's parking pose) do not multiply
    the number of distinct cache entries for links that do not move."""
    if link == "base_link":
        return ()

    leg = leg_of_link(link)
    assert leg is not None
    group = link.removeprefix(f"{leg}_").removesuffix("_link")
    order = ("hip", "upper_leg", "lower_leg")
    depth = order.index(group) + 1 if group in order else 3  # foot -> full chain
    return tuple(joint_name(leg, g) for g in order[:depth])


def _pair_relevant_joints(link_a: str, link_b: str) -> tuple[str, ...]:
    """Joints a link PAIR's collision verdict actually depends on.

    Collision is a function of the two links' RELATIVE transform, so joints
    shared by both chains cancel out exactly: they move both bodies together.
    When one link's chain is a prefix of the other's (the same-leg case,
    including every parent/child pair), only the joints BETWEEN them matter --
    `lf_hip_link <-> lf_upper_leg_link` depends on `lf_upper_leg_joint` alone,
    whatever the hip is doing. Cross-chain pairs share no prefix beyond the
    base, so both chains are relevant.

    Cancelling the shared prefix analytically rather than numerically also
    avoids the floating-point trap: the common transform is not bit-identical
    across poses, so a key built from a computed relative matrix would miss.
    """
    joints_a = _relevant_joints_for_link(link_a)
    joints_b = _relevant_joints_for_link(link_b)

    if len(joints_a) <= len(joints_b) and joints_b[: len(joints_a)] == joints_a:
        return joints_b[len(joints_a) :]
    if len(joints_b) < len(joints_a) and joints_a[: len(joints_b)] == joints_b:
        return joints_a[len(joints_b) :]

    return joints_a + joints_b


_FK_CACHE_MAX_ENTRIES = 4096
"""Bounded LRU size for the per-scene FK cache. Small and fixed on purpose:
this cache only exists to avoid re-parsing the URDF for a pose already seen
in the current sweep, not to retain history across an entire compiler run."""


class RobotScene:
    """Loads the canonical URDF + collision meshes once and evaluates poses."""

    def __init__(self, repo_root: Path = REPO_ROOT):
        self.repo_root = Path(repo_root)
        self.urdf_path = canonical_urdf_path(self.repo_root)

        if not self.urdf_path.is_file():
            raise GeometrySceneError(f"URDF canonico non trovato: {self.urdf_path}")

        validate_servo_mapping(self.urdf_path)
        self.mesh_manifest = load_link_mesh_manifest(self.urdf_path)
        self.urdf_relative_path = CANONICAL_URDF_RELATIVE_PATH

        self._meshes: dict[str, Mesh] = {
            link: load_mesh(
                self.urdf_path.parent / entry.stl_relative_path,
                scale=entry.scale,
            )
            for link, entry in self.mesh_manifest.items()
        }

        self._fk_cache: OrderedDict[tuple[str, tuple[float, ...]], RigidTransform] = OrderedDict()
        self._broad_phase_cache: OrderedDict[tuple[str, tuple[float, ...]], WorldBroadPhase] = OrderedDict()
        self._pair_cache: OrderedDict[tuple, PairCollisionResult] = OrderedDict()

    def mesh(self, link: str) -> Mesh:
        return self._meshes[link]

    def link_transform(self, link: str, pose: dict[str, float]) -> RigidTransform:
        if link == "base_link":
            return identity_transform()

        relevant = _relevant_joints_for_link(link)
        cache_key = (link, tuple(pose[name] for name in relevant))
        cached = self._fk_cache.get(cache_key)

        if cached is not None:
            self._fk_cache.move_to_end(cache_key)
            return cached

        fk = forward_kinematics(
            urdf_path=self.urdf_path,
            root_link="base_link",
            tip_link=link,
            joint_positions_rad=pose,
            enforce_limits=False,
        )

        transform = make_transform(
            [row[:3] for row in fk.tip_transform[:3]],
            [row[3] for row in fk.tip_transform[:3]],
        )

        self._fk_cache[cache_key] = transform

        if len(self._fk_cache) > _FK_CACHE_MAX_ENTRIES:
            self._fk_cache.popitem(last=False)

        return transform

    def link_transforms(self, pose: dict[str, float], links: tuple[str, ...] = ALL_LINKS) -> dict[str, RigidTransform]:
        return {link: self.link_transform(link, pose) for link in links}

    def world_broad_phase(self, link: str, pose: dict[str, float]) -> WorldBroadPhase:
        relevant = _relevant_joints_for_link(link)
        cache_key = (link, tuple(pose[name] for name in relevant))
        cached = self._broad_phase_cache.get(cache_key)

        if cached is not None:
            self._broad_phase_cache.move_to_end(cache_key)
            return cached

        result = world_broad_phase(self._meshes[link], self.link_transform(link, pose))
        self._broad_phase_cache[cache_key] = result

        if len(self._broad_phase_cache) > _FK_CACHE_MAX_ENTRIES:
            self._broad_phase_cache.popitem(last=False)

        return result

    def world_broad_phases(
        self,
        pose: dict[str, float],
        links: tuple[str, ...] = ALL_LINKS,
    ) -> dict[str, WorldBroadPhase]:
        return {link: self.world_broad_phase(link, pose) for link in links}

    def check_link_pair(
        self,
        link_a: str,
        link_b: str,
        pose: dict[str, float],
        *,
        transforms: dict[str, RigidTransform] | None = None,
        broad_phases: dict[str, WorldBroadPhase] | None = None,
        require_distance: bool = True,
        narrow_phase_margin_m: float | None = None,
    ) -> PairCollisionResult:
        """`narrow_phase_margin_m=None` selects the margin from the question
        being asked: `BOOLEAN_COLLISION_MARGIN_M` (0) when only a collision
        boolean is wanted, the production near-miss margin when a clearance
        figure is. See `BOOLEAN_COLLISION_MARGIN_M` for why the two must not
        share one value."""
        transform_a = transforms[link_a] if transforms is not None else self.link_transform(link_a, pose)
        transform_b = transforms[link_b] if transforms is not None else self.link_transform(link_b, pose)
        world_a = broad_phases[link_a] if broad_phases is not None else self.world_broad_phase(link_a, pose)
        world_b = broad_phases[link_b] if broad_phases is not None else self.world_broad_phase(link_b, pose)

        if narrow_phase_margin_m is None:
            narrow_phase_margin_m = (
                DEFAULT_NARROW_PHASE_MARGIN_M if require_distance else BOOLEAN_COLLISION_MARGIN_M
            )

        # Keyed on the joints the pair's RELATIVE pose actually depends on (see
        # `_pair_relevant_joints`), not on each link's full chain. That
        # distinction is what makes the cache effective: while sweeping the hip,
        # lf_hip_link <-> lf_upper_leg_link is rigidly frozen, but both links'
        # chains contain the moving hip joint, so a naive per-link key would
        # miss on every sample and re-run a full narrow-phase pass on a pair
        # sitting microns apart. That was the dominant cost of the whole sweep.
        cache_key = (
            link_a,
            link_b,
            tuple(pose[name] for name in _pair_relevant_joints(link_a, link_b)),
            require_distance,
            narrow_phase_margin_m,
        )
        cached = self._pair_cache.get(cache_key)

        if cached is not None:
            self._pair_cache.move_to_end(cache_key)
            return cached

        result = check_pair_from_world(
            self._meshes[link_a],
            transform_a,
            world_a,
            self._meshes[link_b],
            transform_b,
            world_b,
            narrow_phase_margin_m=narrow_phase_margin_m,
            require_distance=require_distance,
        )

        self._pair_cache[cache_key] = result

        if len(self._pair_cache) > _FK_CACHE_MAX_ENTRIES:
            self._pair_cache.popitem(last=False)

        return result

    def is_colliding_at_pose(
        self,
        pose: dict[str, float],
        link_pairs: tuple[tuple[str, str], ...] = None,  # type: ignore[assignment]
    ) -> tuple[bool, tuple[str, str] | None]:
        """Fast collision-boolean-only sweep over candidate pairs at one
        pose: returns as soon as any pair is confirmed INTERSECTING,
        without computing exact clearance for that pair or any other. This
        is the method bisection-style contact search should call in its
        inner loop; use `worst_pair_at_pose` only for the small number of
        calls that need an actual clearance figure (e.g. the final
        converged bracket boundary, or path minimum-clearance reporting).

        Defaults to the PATH SAFETY set: non-adjacent pairs PLUS revolute
        adjacent pairs (a real articulation can genuinely obstruct a path),
        excluding fixed structural attachments."""
        pairs = link_pairs if link_pairs is not None else path_safety_pairs()
        links_needed = tuple(sorted({link for pair in pairs for link in pair}))
        transforms = self.link_transforms(pose, links_needed)
        broad_phases = self.world_broad_phases(pose, links_needed)

        for link_a, link_b in pairs:
            result = self.check_link_pair(
                link_a, link_b, pose, transforms=transforms, broad_phases=broad_phases, require_distance=False
            )

            if result.status == "INTERSECTING":
                return True, (link_a, link_b)

        return False, None

    def worst_pair_at_pose(
        self,
        pose: dict[str, float],
        link_pairs: tuple[tuple[str, str], ...] = None,  # type: ignore[assignment]
    ) -> tuple[str, str, PairCollisionResult]:
        """Evaluate every candidate pair at one pose and return the pair
        with the smallest clearance (or the first found INTERSECTING pair,
        returned immediately without scanning the rest). Computes an exact
        clearance figure for every non-intersecting pair; prefer
        `is_colliding_at_pose` for bulk collision-only sweeps.

        Defaults to `clearance_policy_pairs()` -- NON_ADJACENT only. Revolute
        adjacent pairs are excluded from *clearance* reporting because their
        healthy resting state is sub-millimetre contact-fit separation, which
        would dominate any minimum-clearance figure and fail a generic gate for
        no physical reason. They remain fully covered by the boolean path-safety
        test in `is_colliding_at_pose`."""
        pairs = link_pairs if link_pairs is not None else clearance_policy_pairs()
        links_needed = tuple(sorted({link for pair in pairs for link in pair}))
        transforms = self.link_transforms(pose, links_needed)
        broad_phases = self.world_broad_phases(pose, links_needed)

        worst_pair: tuple[str, str] | None = None
        worst_result: PairCollisionResult | None = None
        worst_clearance = float("inf")

        for link_a, link_b in pairs:
            result = self.check_link_pair(link_a, link_b, pose, transforms=transforms, broad_phases=broad_phases)

            if result.status == "INTERSECTING":
                return link_a, link_b, result

            if result.clearance_m is not None and result.clearance_m < worst_clearance:
                worst_clearance = result.clearance_m
                worst_pair = (link_a, link_b)
                worst_result = result

        assert worst_pair is not None and worst_result is not None
        return worst_pair[0], worst_pair[1], worst_result
