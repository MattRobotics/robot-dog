#!/usr/bin/env python3
"""V9 guard-aware kinematics wrapper for the V8 support audit.

The canonical URDF parser intentionally rejects q outside nominal joint limits.
Calibration, however, commands a bounded 64-tick outer guard. V9 permits only
that declared corridor for offline collision/table evaluation and rejects any
larger excursion.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
V8_PATH = HERE / "matdog_guarded_support_audit_v8.py"
SPEC = importlib.util.spec_from_file_location("matdog_guarded_support_v8_wrapped", V8_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import V8 audit: {V8_PATH}")
v8 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v8
SPEC.loader.exec_module(v8)
base = v8.base


def guarded_link_transforms(
    root_link: str,
    joints: dict[str, object],
    q: dict[str, float],
) -> dict[str, np.ndarray]:
    transforms = {root_link: np.eye(4)}
    unresolved = set(joints)
    while unresolved:
        progressed = False
        for name in tuple(unresolved):
            joint = joints[name]
            parent_tf = transforms.get(joint.parent)
            if parent_tf is None:
                continue
            value = float(q.get(name, 0.0))
            if joint.joint_type == "revolute":
                if joint.lower is None or joint.upper is None:
                    raise base.AuditFailure(f"{name}: missing limits")
                lower = joint.lower - v8.GUARD_RAD
                upper = joint.upper + v8.GUARD_RAD
                if not lower - 1e-10 <= value <= upper + 1e-10:
                    raise base.AuditFailure(
                        f"{name}: q={value / base.DEG:.3f} deg outside "
                        f"guarded [{lower / base.DEG:.3f}, {upper / base.DEG:.3f}]"
                    )
                motion = base._axis_angle(joint.axis, value)
            elif joint.joint_type == "fixed":
                motion = np.eye(4)
            else:
                raise base.AuditFailure(
                    f"unsupported joint type {joint.joint_type!r}"
                )
            transforms[joint.child] = (
                parent_tf
                @ base._origin_transform(joint.xyz, joint.rpy)
                @ motion
            )
            unresolved.remove(name)
            progressed = True
        if not progressed:
            raise base.AuditFailure(
                f"cannot resolve joint tree: {sorted(unresolved)}"
            )
    return transforms


def main() -> int:
    base.link_transforms = guarded_link_transforms
    return v8.main()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except base.AuditFailure as error:
        print(f"HARD BLOCK: {error}", file=sys.stderr)
        raise SystemExit(2)
