#!/usr/bin/env python3
"""
Mostra, dal solo URDF, come si sposta il foot frame quando un joint aumenta.
Non invia comandi e non apre alcuna porta seriale.
"""

import math
import sys
import xml.etree.ElementTree as ET

EPS_RAD = math.radians(5.0)

def matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]

def eye():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def origin_matrix(xyz, rpy):
    t = rpy_matrix(*rpy)
    t[0][3], t[1][3], t[2][3] = xyz
    return t

def axis_rotation(axis, angle):
    x, y, z = axis
    n = math.sqrt(x*x + y*y + z*z)
    x, y, z = x/n, y/n, z/n

    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)

    return [
        [c + x*x*v,     x*y*v - z*s, x*z*v + y*s, 0.0],
        [y*x*v + z*s, c + y*y*v,     y*z*v - x*s, 0.0],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v,     0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def parse_vec(node, attr, default):
    if node is None:
        return default
    return [float(v) for v in node.attrib.get(attr, default).split()]

def build_link_transforms(joints, q_by_joint):
    transforms = {"base_link": eye()}
    unresolved = list(joints)

    while unresolved:
        remaining = []
        progressed = False

        for joint in unresolved:
            parent = joint["parent"]
            child = joint["child"]

            if parent not in transforms:
                remaining.append(joint)
                continue

            t = matmul(transforms[parent], joint["origin"])
            if joint["type"] in {"revolute", "continuous"}:
                t = matmul(t, axis_rotation(joint["axis"], q_by_joint.get(joint["name"], 0.0)))

            transforms[child] = t
            progressed = True

        if not progressed:
            missing = ", ".join(j["name"] for j in remaining)
            raise RuntimeError(f"Catena URDF non risolvibile: {missing}")

        unresolved = remaining

    return transforms

def position(t):
    return t[0][3], t[1][3], t[2][3]

def main(urdf_path):
    root = ET.parse(urdf_path).getroot()
    joints = []

    for el in root.findall("joint"):
        parent = el.find("parent")
        child = el.find("child")
        if parent is None or child is None:
            continue

        origin = el.find("origin")
        axis = el.find("axis")

        joints.append({
            "name": el.attrib["name"],
            "type": el.attrib.get("type", "fixed"),
            "parent": parent.attrib["link"],
            "child": child.attrib["link"],
            "origin": origin_matrix(
                parse_vec(origin, "xyz", "0 0 0"),
                parse_vec(origin, "rpy", "0 0 0"),
            ),
            "axis": parse_vec(axis, "xyz", "1 0 0"),
        })

    joint_by_name = {j["name"]: j for j in joints}
    neutral = build_link_transforms(joints, {})

    print("=== URDF: SPOSTAMENTO FOOT CON +5° SUL JOINT ===")
    print("Convenzione: X avanti, Y sinistra, Z alto\n")

    for name, joint in joint_by_name.items():
        if joint["type"] not in {"revolute", "continuous"}:
            continue
        if not name.endswith(("_hip_joint", "_upper_leg_joint", "_lower_leg_joint")):
            continue

        prefix = name.split("_")[0]
        foot_joint_name = f"{prefix}_foot_joint"

        if foot_joint_name not in joint_by_name:
            print(f"{name}: foot_joint non trovato")
            continue

        foot_link = joint_by_name[foot_joint_name]["child"]
        p0 = position(neutral[foot_link])

        moved = build_link_transforms(joints, {name: EPS_RAD})
        p1 = position(moved[foot_link])

        dx = (p1[0] - p0[0]) * 1000.0
        dy = (p1[1] - p0[1]) * 1000.0
        dz = (p1[2] - p0[2]) * 1000.0

        print(
            f"{name:24}  "
            f"foot ΔX={dx:+7.2f} mm  "
            f"ΔY={dy:+7.2f} mm  "
            f"ΔZ={dz:+7.2f} mm"
        )

if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"Uso: {sys.argv[0]} /percorso/robot.urdf")
    main(sys.argv[1])
