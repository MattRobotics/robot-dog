#!/usr/bin/env python3
"""V4 exact-mesh audit with active-vs-static collision managers.

For each audited leg, only that leg and its optional rear parking leg can move.
Static-static pairs are checked once at home. Every sampled waypoint then checks:
- non-adjacent collisions inside the active set;
- collisions between active and static sets;
- fixture keep-outs through the base audit.

No collision pair involving a moving link is omitted.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import Any

import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
V2_PATH = HERE / "matdog_endstop_sequence_geometry_audit_v2.py"
SPEC = importlib.util.spec_from_file_location("matdog_sequence_audit_v2_fast", V2_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V2 audit: {V2_PATH}")
v2 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v2
SPEC.loader.exec_module(v2)


def _plain_link(object_name: str) -> str:
    return object_name.split("#", 1)[0]


def _filtered_pairs(
    names: set[tuple[str, str]],
    ignored_pairs: set[frozenset[str]],
) -> set[tuple[str, str]]:
    result: set[tuple[str, str]] = set()
    for left_raw, right_raw in names:
        left = _plain_link(left_raw)
        right = _plain_link(right_raw)
        if left == right or frozenset((left, right)) in ignored_pairs:
            continue
        result.add(tuple(sorted((left, right))))
    return result


class ActiveStaticCollisionChecker:
    def __init__(
        self,
        geometries: list[Any],
        active_links: set[str],
        ignored_pairs: set[frozenset[str]],
    ) -> None:
        self.ignored_pairs = ignored_pairs
        self.active_manager = trimesh.collision.CollisionManager()
        self.static_manager = trimesh.collision.CollisionManager()
        self.active_entries: list[tuple[str, str, np.ndarray]] = []
        self.static_entries: list[tuple[str, str, np.ndarray]] = []

        for index, geometry in enumerate(geometries):
            name = f"{geometry.link}#{index}"
            entry = (name, geometry.link, geometry.local_transform)
            if geometry.link in active_links:
                self.active_manager.add_object(name, geometry.mesh)
                self.active_entries.append(entry)
            else:
                self.static_manager.add_object(name, geometry.mesh)
                self.static_entries.append(entry)

        if not self.active_entries or not self.static_entries:
            raise v2.base.AuditFailure(
                f"invalid active/static partition: active={sorted(active_links)}"
            )

    def check(self, transforms: dict[str, np.ndarray]) -> set[tuple[str, str]]:
        for name, link, local in self.active_entries:
            self.active_manager.set_transform(name, transforms[link] @ local)
        for name, link, local in self.static_entries:
            self.static_manager.set_transform(name, transforms[link] @ local)

        result: set[tuple[str, str]] = set()
        active_collided, active_names = self.active_manager.in_collision_internal(
            return_names=True
        )
        if active_collided:
            result.update(_filtered_pairs(active_names, self.ignored_pairs))

        cross_collided, cross_names = self.active_manager.in_collision_other(
            self.static_manager,
            return_names=True,
        )
        if cross_collided:
            result.update(_filtered_pairs(cross_names, self.ignored_pairs))
        return result


def full_home_collision_check(
    geometries: list[Any],
    transforms: dict[str, np.ndarray],
    ignored_pairs: set[frozenset[str]],
) -> set[tuple[str, str]]:
    manager = trimesh.collision.CollisionManager()
    for index, geometry in enumerate(geometries):
        manager.add_object(
            f"{geometry.link}#{index}",
            geometry.mesh,
            transform=transforms[geometry.link] @ geometry.local_transform,
        )
    collided, names = manager.in_collision_internal(return_names=True)
    if not collided:
        return set()
    return _filtered_pairs(names, ignored_pairs)


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
    home_collisions = full_home_collision_check(
        geometries,
        home_transforms,
        ignored_pairs,
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
            checker = ActiveStaticCollisionChecker(
                geometries,
                active_links,
                ignored_pairs,
            )

            def current_checker(
                _geometries: list[Any],
                transforms: dict[str, np.ndarray],
                _ignored_pairs: set[frozenset[str]],
                *,
                _checker: ActiveStaticCollisionChecker = checker,
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
            results.append(result)
    finally:
        v2.base.collision_pairs = original_collision_pairs

    payload = {
        "result": "PASS",
        "algorithm": "ordered-upper-lower-hip-active-static-bvh-v4",
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
