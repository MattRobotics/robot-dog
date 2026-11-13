#!/usr/bin/env python3
"""Locomotion vNext follow-up (2026-11-13), DIAGNOSTIC ONLY: the joystick gate's geometric checks on the pose the
commanded joints would really take on a rigid floor.

The gate (foot_edge_contact.py frames, unchanged and still the verdict) keeps the body at the law's height and level,
with a rigid robot on a flat floor at -bz: a stance foot commanded below that plane counts as penetration and every
link moves down with it. A robot cannot push its feet through the floor; its body rests on the stance contacts.
Here, per checked frame (every 3rd release, as the gate), the same URDF geometry (foot_edge_contact.frames/contact,
offline_prone geometry) is placed on its stance contacts: the floor is the least-squares plane of the stance
contact points (exact for three), gravity its normal. Reported, as the gate: the static support margin of the URDF
COM inside the stance polygon (projected along the normal) and the lowest non-foot mesh point above that floor.
Still a rigid robot: with the compliance feedforward the commanded pre-deflection stays uncancelled (C_true = 0),
the worst case of an over-estimated C. No compliance, friction or dynamics.

Usage: joy_realized.py <export MATDOG_Controller dir> <joystick output dir> [<joystick output dir> ...]
"""
import math
import pathlib
import sys

import numpy as np
from scipy.spatial import ConvexHull


def main():
    fw = pathlib.Path(sys.argv[1])
    sys.path.insert(0, str(fw / "scripts/offline_motion"))
    sys.path.insert(0, str(fw / "scripts/offline_prone"))
    import foot_edge_contact as fec
    import geometry as G
    for d in map(pathlib.Path, sys.argv[2:]):
        worst = dict(support=(1e9, ""), clearance=(1e9, ""), tilt=(0.0, ""))
        for f in sorted(d.glob("*.txt")):
            if f.name in ("envelope.log",) or f.stat().st_size == 0:
                continue
            try:
                a = np.loadtxt(f, ndmin=2)
            except ValueError:
                continue
            if a.shape[1] != 32:
                continue
            for k in range(0, len(a), 3):
                row = a[k]
                q, ft = row[4:16], row[16:].reshape(4, 4)
                fr = fec.frames(q)
                V, com = G.geometry(q)
                pts = [fec.contact(fr[leg + "_foot_link"])[1] for l, leg in enumerate(fec.LEGS) if ft[l, 3] > .5]
                if len(pts) < 3:
                    continue
                P = np.array(pts)
                M = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
                ca, cb, cc = np.linalg.lstsq(M, P[:, 2], rcond=None)[0]
                n = np.array([-ca, -cb, 1.0]) / math.sqrt(1 + ca * ca + cb * cb)  # floor normal (up), body frame
                o = np.array([0, 0, cc])
                e1 = np.cross([0, 1.0, 0], n); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
                flat = lambda x: np.c_[(x - o) @ e1, (x - o) @ e2]
                h = ConvexHull(flat(P))
                c2 = flat(np.asarray(com)[None, :3])[0]
                s = float(np.min(-(h.equations[:, :2] @ c2 + h.equations[:, 2]))) * 1e3
                if s < worst["support"][0]:
                    worst["support"] = (s, f"{f.stem} c{row[0]:.3f}")
                for name, v in V.items():
                    if name.endswith("foot_link"):
                        continue
                    z = float(np.min((v - o) @ n)) * 1e3
                    if z < worst["clearance"][0]:
                        worst["clearance"] = (z, f"{f.stem} {name} c{row[0]:.3f}")
                t = math.degrees(math.acos(min(1.0, n[2])))
                if t > worst["tilt"][0]:
                    worst["tilt"] = (t, f"{f.stem} c{row[0]:.3f}")
        print(f"{d}: realized support min {worst['support'][0]:.2f} mm ({worst['support'][1]}) | non-foot clearance min "
              f"{worst['clearance'][0]:.2f} mm ({worst['clearance'][1]}) | body tilt on its stance feet max {worst['tilt'][0]:.2f} deg ({worst['tilt'][1]})", flush=True)


if __name__ == "__main__":
    main()
