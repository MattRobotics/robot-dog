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
)

from matdog_geometry_mesh_kernel import (  # noqa: E402
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


def _adjacent_link_pairs() -> frozenset[frozenset[str]]:
    pairs: set[frozenset[str]] = set()

    for leg in LEG_IDS:
        chain = ("base_link", f"{leg}_hip_link", f"{leg}_upper_leg_link", f"{leg}_lower_leg_link", f"{leg}_foot_link")

        for parent, child in zip(chain, chain[1:]):
            pairs.add(frozenset((parent, child)))

    return frozenset(pairs)


ADJACENT_LINK_PAIRS = _adjacent_link_pairs()


def is_adjacent_pair(link_a: str, link_b: str) -> bool:
    return frozenset((link_a, link_b)) in ADJACENT_LINK_PAIRS


def all_candidate_collision_pairs() -> tuple[tuple[str, str], ...]:
    """PATH_SELF_COLLISION_POLICY: every non-adjacent link pair across the
    whole robot (cross-leg included). Adjacent pairs are excluded because
    they are permanently INTERSECTING at every joint angle including home
    (verified: every leg's base_link<->hip_link, hip_link<->upper_leg_link,
    upper_leg_link<->lower_leg_link and lower_leg_link<->foot_link mesh
    pair overlaps at q=0 -- the collision STL bodies share/interpenetrate
    the hinge/bracket area by mechanical design), so without this
    exclusion every single pose would report a collision and the check
    would be useless for path/self-collision purposes."""
    pairs = []

    for i, link_a in enumerate(ALL_LINKS):
        for link_b in ALL_LINKS[i + 1 :]:
            if not is_adjacent_pair(link_a, link_b):
                pairs.append((link_a, link_b))

    return tuple(pairs)


def same_leg_non_adjacent_pairs(leg_id: str) -> tuple[tuple[str, str], ...]:
    """ENDSTOP_CONTACT_POLICY candidate pairs for one leg: only that leg's
    own non-adjacent link pairs (base_link and this leg's hip/upper/lower/
    foot links), never another leg's geometry.

    This is a *different* policy from `all_candidate_collision_pairs`, not
    a subset used for the same purpose: it exists so a per-joint endstop
    contact search can be run without a cross-leg path obstruction (e.g. a
    different leg's foot in the way) silently terminating the search and
    being misreported as if it were this joint's own mechanical limit
    (canonical handoff reconciliation section 2/3 -- same-leg collision
    must not be conflated with a cross-leg PATH_COLLISION_BEFORE_ENDPOINT
    finding). Adjacent pairs are excluded for the same reason as in
    `all_candidate_collision_pairs`: they are permanently intersecting at
    every angle (hinge/bracket overlap), so a boolean intersection test on
    them can never localize a true joint-limit *event* -- it is always
    true, at literally every angle, and offers no discriminating
    information about where in the sweep a real endstop occurs."""
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
            if not is_adjacent_pair(link_a, link_b):
                pairs.append((link_a, link_b))

    return tuple(pairs)


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
    ) -> PairCollisionResult:
        transform_a = transforms[link_a] if transforms is not None else self.link_transform(link_a, pose)
        transform_b = transforms[link_b] if transforms is not None else self.link_transform(link_b, pose)
        world_a = broad_phases[link_a] if broad_phases is not None else self.world_broad_phase(link_a, pose)
        world_b = broad_phases[link_b] if broad_phases is not None else self.world_broad_phase(link_b, pose)

        return check_pair_from_world(
            self._meshes[link_a],
            transform_a,
            world_a,
            self._meshes[link_b],
            transform_b,
            world_b,
            require_distance=require_distance,
        )

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
        converged bracket boundary, or path minimum-clearance reporting)."""
        pairs = link_pairs if link_pairs is not None else all_candidate_collision_pairs()
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
        `is_colliding_at_pose` for bulk collision-only sweeps."""
        pairs = link_pairs if link_pairs is not None else all_candidate_collision_pairs()
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
