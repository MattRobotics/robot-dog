#!/usr/bin/env python3
"""V5 pair-filtered exact FCL audit for the ordered MATDOG sequence.

Dense STL meshes make generic all-pairs CollisionManager checks unnecessarily
expensive because adjacent pairs are evaluated and discarded afterwards. V5:

1. builds each exact triangle BVH once;
2. excludes adjacent/same-link pairs before collision queries;
3. keeps only pairs involving at least one link that can move in this leg audit;
4. uses transformed AABBs as a cheap broad phase;
5. calls exact FCL triangle collision only for overlapping AABBs.

The motion paths, candidate poses and acceptance rules remain those of V2.
"""

from __future__ import annotations

import importlib.util
import itertools
from pathlib import Path
import sys
from typing import Any

import fcl
import numpy as np


HERE = Path(__file__).resolve().parent
V2_PATH = HERE / "matdog_endstop_sequence_geometry_audit_v2.py"
SPEC = importlib.util.spec_from_file_location("matdog_sequence_audit_v2_pairwise", V2_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V2 audit: {V2_PATH}")
v2 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v2
SPEC.loader.exec_module(v2)


class ExactObject:
    def __init__(self, name: str, geometry: Any) -> None:
        self.name = name
        self.link = geometry.link
        self.local_transform = geometry.local_transform
        vertices = np.asarray(geometry.mesh.vertices, dtype=np.float64)
        faces = np.asarray(geometry.mesh.faces, dtype=np.int32)
        if len(vertices) == 0 or len(faces) == 0:
            raise v2.base.AuditFailure(f"empty exact mesh: {name}")
        model = fcl.BVHModel()
        model.beginModel(len(vertices), len(faces))
        model.addSubModel(vertices, faces)
        model.endModel()
        self.object = fcl.CollisionObject(model)

        bounds = np.asarray(geometry.mesh.bounds, dtype=float)
        minimum, maximum = bounds
        self.local_corners = np.asarray(
            list(itertools.product(*zip(minimum, maximum))),
            dtype=float,
        )
        self.world_minimum = np.zeros(3)
        self.world_maximum = np.zeros(3)

    def update(self, link_transform: np.ndarray) -> None:
        transform = link_transform @ self.local_transform
        self.object.setTransform(
            fcl.Transform(transform[:3, :3], transform[:3, 3])
        )
        self.object.computeAABB()
        homogeneous = np.column_stack(
            (self.local_corners, np.ones(len(self.local_corners)))
        )
        world = (transform @ homogeneous.T).T[:, :3]
        self.world_minimum = world.min(axis=0)
        self.world_maximum = world.max(axis=0)


class PairFilteredExactChecker:
    def __init__(
        self,
        geometries: list[Any],
        active_links: set[str],
        ignored_pairs: set[frozenset[str]],
        home_transforms: dict[str, np.ndarray],
    ) -> None:
        self.objects = [
            ExactObject(f"{geometry.link}#{index}", geometry)
            for index, geometry in enumerate(geometries)
        ]
        self.active_links = active_links
        self.pairs: list[tuple[ExactObject, ExactObject]] = []

        for left_index, left in enumerate(self.objects):
            for right in self.objects[left_index + 1 :]:
                link_pair = frozenset((left.link, right.link))
                if left.link == right.link or link_pair in ignored_pairs:
                    continue
                if left.link not in active_links and right.link not in active_links:
                    continue
                self.pairs.append((left, right))

        if not self.pairs:
            raise v2.base.AuditFailure(
                f"no non-adjacent exact pairs for active links {sorted(active_links)}"
            )
        for item in self.objects:
            item.update(home_transforms[item.link])

    @staticmethod
    def _aabb_overlap(left: ExactObject, right: ExactObject) -> bool:
        return bool(
            np.all(left.world_maximum >= right.world_minimum)
            and np.all(right.world_maximum >= left.world_minimum)
        )

    def check(self, transforms: dict[str, np.ndarray]) -> set[tuple[str, str]]:
        for item in self.objects:
            if item.link in self.active_links:
                item.update(transforms[item.link])

        collisions: set[tuple[str, str]] = set()
        request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=False)
        for left, right in self.pairs:
            if not self._aabb_overlap(left, right):
                continue
            result = fcl.CollisionResult()
            if fcl.collide(left.object, right.object, request, result) > 0:
                collisions.add(tuple(sorted((left.link, right.link))))
        return collisions


def moving_links_for_leg(leg: str, geometries: list[Any]) -> set[str]:
    prefixes = {f"{leg}_"}
    parked = v2.base.FRONT_REAR_PARK.get(leg)
    if parked is not None:
        prefixes.add(f"{parked}_")
    return {
        geometry.link
        for geometry in geometries
        if any(geometry.link.startswith(prefix) for prefix in prefixes)
    }


def full_home_check(
    geometries: list[Any],
    ignored_pairs: set[frozenset[str]],
    transforms: dict[str, np.ndarray],
) -> set[tuple[str, str]]:
    all_non_base_links = {
        geometry.link for geometry in geometries if geometry.link != "base_link"
    }
    checker = PairFilteredExactChecker(
        geometries,
        all_non_base_links,
        ignored_pairs,
        transforms,
    )
    return checker.check(transforms)


def main() -> int:
    args = v2.base.parse_args()
    repo_root = args.repo_root.resolve()
    fixture_boxes, fixture_margin = v2.base.load_fixture(args.fixture)
    if args.certify_hardware and not fixture_boxes:
        raise v2.base.AuditFailure(
            "hardware certification requires --fixture with measured non-placeholder keep-outs"
        )

    root_link, joints, geometries = v2.base.load_model(
        repo_root / v2.base.URDF_RELATIVE
    )
    ignored_pairs = v2.base.adjacent_link_pairs(joints)
    home_transforms = v2.base.link_transforms(
        root_link,
        joints,
        v2.base.base_pose(),
    )
    home_collisions = full_home_check(
        geometries,
        ignored_pairs,
        home_transforms,
    )
    if home_collisions:
        raise v2.base.AuditFailure(
            f"canonical home has non-adjacent collisions: {sorted(home_collisions)}"
        )

    results: list[dict[str, object]] = []
    original_collision_pairs = v2.base.collision_pairs
    try:
        for leg in v2.base.LEGS:
            active_links = moving_links_for_leg(leg, geometries)
            checker = PairFilteredExactChecker(
                geometries,
                active_links,
                ignored_pairs,
                home_transforms,
            )

            def current_checker(
                _geometries: list[Any],
                transforms: dict[str, np.ndarray],
                _ignored_pairs: set[frozenset[str]],
                *,
                _checker: PairFilteredExactChecker = checker,
            ) -> set[tuple[str, str]]:
                return _checker.check(transforms)

            v2.base.collision_pairs = current_checker
            result = v2.audit_leg_v2(
                leg,
                root_link=root_link,
                joints=joints,
                geometries=geometries,
                ignored_pairs=ignored_pairs,
                fixture_boxes=fixture_boxes,
                fixture_margin=fixture_margin,
            )
            result["active_collision_links"] = sorted(active_links)
            result["exact_non_adjacent_pair_count"] = len(checker.pairs)
            results.append(result)
    finally:
        v2.base.collision_pairs = original_collision_pairs

    payload = {
        "result": "PASS",
        "algorithm": "ordered-upper-lower-hip-pair-filtered-fcl-v5",
        "hardware_certified": bool(args.certify_hardware),
        "fixture_boxes": [box.name for box in fixture_boxes],
        "fixture_margin_m": fixture_margin,
        "model_limit_margin_deg": v2.MODEL_LIMIT_MARGIN_DEG,
        "order": ["UPPER_MIN_MAX", "LOWER_MIN_MAX", "HIP_MIN_MAX"],
        "legs": results,
    }
    rendered = v2.json.dumps(payload, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except v2.base.AuditFailure as error:
        print(f"HARD BLOCK: {error}", file=sys.stderr)
        raise SystemExit(2)
