#!/usr/bin/env python3
"""Compatibility entry point for Python-FCL 0.7.0.8.

The binding exposes CollisionObject.setTransform but not computeAABB. The
ordered audit already computes transformed AABBs independently, so no FCL AABB
method is required.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import fcl
import numpy as np


HERE = Path(__file__).resolve().parent
V5_PATH = HERE / "matdog_endstop_sequence_geometry_audit_v5.py"
SPEC = importlib.util.spec_from_file_location("matdog_sequence_audit_v5_compat", V5_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V5 audit: {V5_PATH}")
v5 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v5
SPEC.loader.exec_module(v5)


def compatible_update(self: object, link_transform: np.ndarray) -> None:
    transform = link_transform @ self.local_transform
    self.object.setTransform(
        fcl.Transform(transform[:3, :3], transform[:3, 3])
    )
    homogeneous = np.column_stack(
        (self.local_corners, np.ones(len(self.local_corners)))
    )
    world = (transform @ homogeneous.T).T[:, :3]
    self.world_minimum = world.min(axis=0)
    self.world_maximum = world.max(axis=0)


v5.ExactObject.update = compatible_update


if __name__ == "__main__":
    try:
        raise SystemExit(v5.main())
    except v5.v2.base.AuditFailure as error:
        print(f"HARD BLOCK: {error}", file=sys.stderr)
        raise SystemExit(2)
