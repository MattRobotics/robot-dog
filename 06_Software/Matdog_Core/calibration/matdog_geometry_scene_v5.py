#!/usr/bin/env python3
"""Pure, model-driven collision scene for Geometry Compiler V5.

All structure comes from :mod:`matdog_geometry_model_v5`.  In particular,
this scene honors each URDF collision filename, scale and origin and derives
kinematic dependencies from topology.  It deliberately knows nothing about
MATDOG leg prefixes, LF hardware measurements, parking seeds or safety gates.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import replace
import math
from pathlib import Path

import numpy as np

from matdog_geometry_mesh_kernel import (
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
from matdog_geometry_model_v5 import (
    GeometryModelError,
    RobotGeometryModel,
    load_robot_geometry_model,
)


class GeometrySceneV5Error(RuntimeError):
    """Invalid pose or scene operation."""


def _rotation_from_rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def _axis_angle_rotation(axis_xyz: tuple[float, float, float], angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis_xyz, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm <= 0.0:
        raise GeometrySceneV5Error(f"zero joint axis: {axis_xyz}")
    x, y, z = axis / norm
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    v = 1.0 - c
    return np.array(
        (
            (c + x * x * v, x * y * v - z * s, x * z * v + y * s),
            (y * x * v + z * s, c + y * y * v, y * z * v - x * s),
            (z * x * v - y * s, z * y * v + x * s, c + z * z * v),
        ),
        dtype=np.float64,
    )


def _compose(world_from_parent: RigidTransform, parent_from_child: RigidTransform) -> RigidTransform:
    rotation = world_from_parent.rotation @ parent_from_child.rotation
    translation = (
        world_from_parent.rotation @ parent_from_child.translation
        + world_from_parent.translation
    )
    return make_transform(rotation, translation)


def _static_transform(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> RigidTransform:
    return make_transform(_rotation_from_rpy(rpy), xyz)


def _joint_transform(scene_model: RobotGeometryModel, joint_name: str, pose: dict[str, float]) -> RigidTransform:
    joint = scene_model.joints[joint_name]
    transform = _static_transform(joint.origin_xyz, joint.origin_rpy)
    if joint.joint_type == "fixed":
        return transform
    try:
        angle = pose[joint_name]
    except KeyError as exc:
        raise GeometrySceneV5Error(f"pose missing actuated joint {joint_name!r}") from exc
    motion = make_transform(_axis_angle_rotation(joint.axis_xyz, angle), (0.0, 0.0, 0.0))
    return _compose(transform, motion)


_CACHE_MAX_ENTRIES = 4096


class RobotSceneV5:
    """Loaded URDF model and immutable collision meshes evaluated at poses."""

    def __init__(self, model: RobotGeometryModel):
        self.model = model
        self.urdf_path = model.urdf_path
        self.repo_root: Path | None = None
        self._meshes: dict[str, Mesh] = {}
        for link_name in model.collision_link_names:
            entry = model.collision_geometry(link_name)
            mesh_path = model.collision_mesh_path(link_name)
            if not mesh_path.is_file():
                raise GeometrySceneV5Error(
                    f"{link_name}: collision mesh not found: {mesh_path}"
                )
            self._meshes[link_name] = load_mesh(mesh_path, scale=entry.scale)

        self._transform_cache: OrderedDict[
            tuple[str, tuple[float, ...]], RigidTransform
        ] = OrderedDict()
        self._broad_phase_cache: OrderedDict[
            tuple[str, tuple[float, ...]], WorldBroadPhase
        ] = OrderedDict()
        self._pair_cache: OrderedDict[tuple[object, ...], PairCollisionResult] = OrderedDict()

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path,
        *,
        expected_actuated_joint_count: int | None = 12,
    ) -> "RobotSceneV5":
        return cls(
            load_robot_geometry_model(
                urdf_path,
                expected_actuated_joint_count=expected_actuated_joint_count,
            )
        )

    def mesh(self, link_name: str) -> Mesh:
        try:
            return self._meshes[link_name]
        except KeyError as exc:
            raise GeometrySceneV5Error(f"{link_name}: no loaded collision mesh") from exc

    def full_pose(self, overrides: dict[str, float]) -> dict[str, float]:
        try:
            return self.model.full_pose(overrides)
        except GeometryModelError as exc:
            raise GeometrySceneV5Error(str(exc)) from exc

    def home_pose(self) -> dict[str, float]:
        return self.model.home_pose()

    def _cache_values(self, link_name: str, pose: dict[str, float]) -> tuple[float, ...]:
        return tuple(
            pose[joint_name]
            for joint_name in self.model.relevant_actuated_joints_for_link(link_name)
        )

    def collision_transform(self, link_name: str, pose: dict[str, float]) -> RigidTransform:
        """Return world-from-collision-geometry, including URDF collision origin."""
        cache_key = (link_name, self._cache_values(link_name, pose))
        cached = self._transform_cache.get(cache_key)
        if cached is not None:
            self._transform_cache.move_to_end(cache_key)
            return cached

        world_from_link = identity_transform()
        for joint_name in self.model.joint_chain_by_link[link_name]:
            world_from_link = _compose(
                world_from_link,
                _joint_transform(self.model, joint_name, pose),
            )
        collision = self.model.collision_geometry(link_name)
        world_from_collision = _compose(
            world_from_link,
            _static_transform(collision.origin_xyz, collision.origin_rpy),
        )
        self._transform_cache[cache_key] = world_from_collision
        if len(self._transform_cache) > _CACHE_MAX_ENTRIES:
            self._transform_cache.popitem(last=False)
        return world_from_collision

    def collision_transforms(
        self,
        pose: dict[str, float],
        links: tuple[str, ...] | None = None,
    ) -> dict[str, RigidTransform]:
        selected = links if links is not None else self.model.collision_link_names
        return {link: self.collision_transform(link, pose) for link in selected}

    def world_broad_phase(self, link_name: str, pose: dict[str, float]) -> WorldBroadPhase:
        cache_key = (link_name, self._cache_values(link_name, pose))
        cached = self._broad_phase_cache.get(cache_key)
        if cached is not None:
            self._broad_phase_cache.move_to_end(cache_key)
            return cached
        result = world_broad_phase(
            self.mesh(link_name),
            self.collision_transform(link_name, pose),
        )
        self._broad_phase_cache[cache_key] = result
        if len(self._broad_phase_cache) > _CACHE_MAX_ENTRIES:
            self._broad_phase_cache.popitem(last=False)
        return result

    def world_broad_phases(
        self,
        pose: dict[str, float],
        links: tuple[str, ...] | None = None,
    ) -> dict[str, WorldBroadPhase]:
        selected = links if links is not None else self.model.collision_link_names
        return {link: self.world_broad_phase(link, pose) for link in selected}

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
        if link_a == link_b:
            raise GeometrySceneV5Error("a link cannot be checked against itself")
        relevant = self.model.relevant_actuated_joints_for_pair(link_a, link_b)
        if narrow_phase_margin_m is None:
            narrow_phase_margin_m = (
                DEFAULT_NARROW_PHASE_MARGIN_M
                if require_distance
                else BOOLEAN_COLLISION_MARGIN_M
            )
        cache_key: tuple[object, ...] = (
            link_a,
            link_b,
            tuple(pose[name] for name in relevant),
            require_distance,
            narrow_phase_margin_m,
        )
        cached = self._pair_cache.get(cache_key)
        if cached is not None:
            self._pair_cache.move_to_end(cache_key)
            return cached

        transform_a = (
            transforms[link_a]
            if transforms is not None
            else self.collision_transform(link_a, pose)
        )
        transform_b = (
            transforms[link_b]
            if transforms is not None
            else self.collision_transform(link_b, pose)
        )
        world_a = (
            broad_phases[link_a]
            if broad_phases is not None
            else self.world_broad_phase(link_a, pose)
        )
        world_b = (
            broad_phases[link_b]
            if broad_phases is not None
            else self.world_broad_phase(link_b, pose)
        )
        raw = check_pair_from_world(
            self.mesh(link_a),
            transform_a,
            world_a,
            self.mesh(link_b),
            transform_b,
            world_b,
            narrow_phase_margin_m=narrow_phase_margin_m,
            require_distance=require_distance,
        )
        # The validated kernel labels results by STL stem.  V5 collision
        # filenames are data and may differ from link names, so scene results
        # explicitly retain the URDF link identity instead.
        result = replace(raw, link_a=link_a, link_b=link_b)
        self._pair_cache[cache_key] = result
        if len(self._pair_cache) > _CACHE_MAX_ENTRIES:
            self._pair_cache.popitem(last=False)
        return result

    def first_collision_at_pose(
        self,
        pose: dict[str, float],
        *,
        link_pairs: tuple[tuple[str, str], ...] | None = None,
    ) -> tuple[bool, tuple[str, str] | None]:
        pairs = (
            link_pairs
            if link_pairs is not None
            else self.model.path_obstruction_pairs()
        )
        links = tuple(sorted({link for pair in pairs for link in pair}))
        transforms = self.collision_transforms(pose, links)
        broad_phases = self.world_broad_phases(pose, links)
        for link_a, link_b in pairs:
            result = self.check_link_pair(
                link_a,
                link_b,
                pose,
                transforms=transforms,
                broad_phases=broad_phases,
                require_distance=False,
            )
            if result.status == "INTERSECTING":
                return True, (link_a, link_b)
        return False, None

    def worst_pair_at_pose(
        self,
        pose: dict[str, float],
        *,
        link_pairs: tuple[tuple[str, str], ...] | None = None,
    ) -> tuple[str, str, PairCollisionResult]:
        pairs = link_pairs if link_pairs is not None else self.model.clearance_pairs()
        if not pairs:
            raise GeometrySceneV5Error("no collision pairs available for clearance evaluation")
        links = tuple(sorted({link for pair in pairs for link in pair}))
        transforms = self.collision_transforms(pose, links)
        broad_phases = self.world_broad_phases(pose, links)
        worst: tuple[str, str, PairCollisionResult] | None = None
        worst_clearance = float("inf")
        for link_a, link_b in pairs:
            result = self.check_link_pair(
                link_a,
                link_b,
                pose,
                transforms=transforms,
                broad_phases=broad_phases,
            )
            if result.status == "INTERSECTING":
                return link_a, link_b, result
            if result.clearance_m is not None and result.clearance_m < worst_clearance:
                worst = (link_a, link_b, result)
                worst_clearance = result.clearance_m
        if worst is None:
            raise GeometrySceneV5Error("no pair produced a clearance result")
        return worst

    def q0_active_pair_results(self) -> dict[str, PairCollisionResult]:
        pose = self.home_pose()
        return {
            joint_name: self.check_link_pair(
                *self.model.active_revolute_pair(joint_name),
                pose,
                require_distance=False,
            )
            for joint_name in self.model.actuated_joint_names
        }
