#!/usr/bin/env python3
"""V3 performance wrapper for the ordered MATDOG geometry audit.

It reuses the V2 algorithm and replaces only collision-manager construction:
BVH models are built once, while each sampled configuration updates transforms.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import Any

import trimesh


HERE = Path(__file__).resolve().parent
V2_PATH = HERE / "matdog_endstop_sequence_geometry_audit_v2.py"
SPEC = importlib.util.spec_from_file_location("matdog_sequence_audit_v2", V2_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V2 audit: {V2_PATH}")
v2 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v2
SPEC.loader.exec_module(v2)

_CACHE: dict[int, tuple[Any, list[tuple[str, str, Any]]]] = {}


def cached_collision_pairs(
    geometries: list[Any],
    transforms: dict[str, Any],
    ignored_pairs: set[frozenset[str]],
) -> set[tuple[str, str]]:
    key = id(geometries)
    cached = _CACHE.get(key)
    if cached is None:
        manager = trimesh.collision.CollisionManager()
        entries: list[tuple[str, str, Any]] = []
        for index, geometry in enumerate(geometries):
            name = f"{geometry.link}#{index}"
            manager.add_object(name, geometry.mesh)
            entries.append((name, geometry.link, geometry.local_transform))
        cached = (manager, entries)
        _CACHE[key] = cached

    manager, entries = cached
    for name, link, local_transform in entries:
        manager.set_transform(name, transforms[link] @ local_transform)

    collided, names = manager.in_collision_internal(return_names=True)
    if not collided:
        return set()

    result: set[tuple[str, str]] = set()
    for left_raw, right_raw in names:
        left = left_raw.split("#", 1)[0]
        right = right_raw.split("#", 1)[0]
        if left == right or frozenset((left, right)) in ignored_pairs:
            continue
        result.add(tuple(sorted((left, right))))
    return result


v2.base.collision_pairs = cached_collision_pairs


if __name__ == "__main__":
    try:
        raise SystemExit(v2.main())
    except v2.base.AuditFailure as error:
        print(f"HARD BLOCK: {error}", file=sys.stderr)
        raise SystemExit(2)
