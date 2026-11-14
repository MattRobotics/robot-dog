#!/usr/bin/env python3
"""Milestone 1: kinematic replay of SimpleMotion's STAND poses and joint
trajectories in the MuJoCo model, checked against the firmware's own FK.

Usage: m1_kinematics.py <fk_reference> <sequences dir> <delivered dir> <out dir>

Checks, each with an explicit tolerance (results in <out>/m1_kinematics.json):
  A  the compiled model equals the URDF: masses, CoM, inertia tensors, joint
     origins, axes, limits;
  B  the delivered CSVs are the tail of the recorded sequences (same RAW);
  C  MuJoCo FK == firmware FK (LegKinematics.cpp, FootContact.cpp) for every
     replayed row: foot_link origin, cylinder centre, contact reference;
  D  the sent joint references reproduce the law's foot references (body pose
     of the law headed by the applied level correction) within the RAW
     rounding bound of each row;
  E  STAND poses: with the body level at the nominal base_link height the
     four contact references lie on the ground at the canonical stand contacts.
No physics is involved: q is imposed, gravity and contacts play no role.
"""
import json
import math
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import twin

TOL_IDENTITY_M = 1e-9  # C: same geometry, double precision
TOL_MODEL_REL = 1e-9   # A: MuJoCo stores principal inertia + frame; reconstruction round-off only


def check_model(model, data):
    root = ET.parse(twin.HERE.parents[1] / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf").getroot()
    worst = {"mass_rel": 0.0, "com_m": 0.0, "inertia_rel": 0.0, "joint_pos_m": 0.0, "axis": 0.0, "range_rad": 0.0}
    for link in root.findall("link"):
        b = model.body(link.get("name"))
        i = link.find("inertial")
        mass = float(i.find("mass").get("value"))
        com = np.array([float(v) for v in i.find("origin").get("xyz").split()])
        a = i.find("inertia").attrib
        full = np.array([[float(a["ixx"]), float(a["ixy"]), float(a["ixz"])],
                         [float(a["ixy"]), float(a["iyy"]), float(a["iyz"])],
                         [float(a["ixz"]), float(a["iyz"]), float(a["izz"])]])
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, b.iquat)
        rot = rot.reshape(3, 3)
        rebuilt = rot @ np.diag(b.inertia) @ rot.T
        worst["mass_rel"] = max(worst["mass_rel"], abs(b.mass[0] - mass) / mass)
        worst["com_m"] = max(worst["com_m"], float(np.max(np.abs(b.ipos - com))))
        worst["inertia_rel"] = max(worst["inertia_rel"], float(np.max(np.abs(rebuilt - full)) / np.max(np.abs(full))))
    for joint in root.findall("joint"):
        child = model.body(joint.find("child").get("link"))
        pos = np.array([float(v) for v in joint.find("origin").get("xyz").split()])
        worst["joint_pos_m"] = max(worst["joint_pos_m"], float(np.max(np.abs(child.pos - pos))))
        if joint.get("type") == "revolute":
            j = model.joint(joint.get("name"))
            axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            lim = joint.find("limit")
            worst["axis"] = max(worst["axis"], float(np.max(np.abs(j.axis - axis))))
            worst["range_rad"] = max(worst["range_rad"], float(np.max(np.abs(j.range - [float(lim.get("lower")), float(lim.get("upper"))]))))
    total = float(np.sum(model.body_mass))
    ok = (worst["mass_rel"] <= TOL_MODEL_REL and worst["inertia_rel"] <= TOL_MODEL_REL and
          max(worst["com_m"], worst["joint_pos_m"], worst["axis"], worst["range_rad"]) <= 1e-12)
    return {"pass": ok, "tolerance": {"relative": TOL_MODEL_REL, "absolute": 1e-12}, "worst": worst,
            "total_mass_kg": total, "actuated_joints": model.nu, "links": model.nbody - 1}


class Fk:
    def __init__(self, model, data):
        self.m, self.d = model, data
        self.origin = twin.site_ids(model, "foot_origin")
        self.center = twin.site_ids(model, "foot_cylinder_center")
        self.cyl = np.array([model.geom(f"{l}_foot_cylinder").id for l in twin.LEGS])

    def __call__(self, q, pos=(0, 0, 0), yaw=0.0, rotation=None):
        twin.set_base(self.d, pos, yaw, rotation)
        self.d.qpos[7:19] = q
        mujoco.mj_kinematics(self.m, self.d)
        return (self.d.site_xpos[self.origin].copy(), self.d.site_xpos[self.center].copy(),
                twin.contact_reference(self.d, self.center, self.cyl))


def firmware_fk(binary, qs):
    text = "\n".join(",".join(repr(float(v)) for v in row) for row in qs) + "\n"
    out = subprocess.run([binary], input=text, capture_output=True, text=True, check=True).stdout.splitlines()
    values = np.array([[float(v) for v in line.split(",")[1:]] for line in out[1:]]).reshape(len(qs), 4, 10)
    return values[:, :, 0], values[:, :, 1:4], values[:, :, 7:10], values[:, :, 4:7]  # status, origin, center, contact


def rounding_bound(fk, q, pos, yaw, rotation=None):
    """First-order bound of the contact-point error from rounding each joint to the tick grid."""
    _, _, base = fk(q, pos, yaw, rotation)
    bound = np.zeros(4)
    for i in range(12):
        dq = np.zeros(12)
        dq[i] = 1e-6
        _, _, moved = fk(q + dq, pos, yaw, rotation)
        bound += np.linalg.norm(moved - base, axis=1) / 1e-6 * (twin.TICK / 2)
    return bound


def main():
    fk_binary, seq_dir, delivered_dir, out_dir = map(pathlib.Path, sys.argv[1:5])
    out_dir.mkdir(parents=True, exist_ok=True)
    model, data = twin.load_model()
    fk = Fk(model, data)
    report = {"A_model_vs_urdf": check_model(model, data)}

    # B: delivered CSVs are the tails of the recorded sequences.
    tails = {}
    for path in sorted(delivered_dir.glob("*.csv")):
        dv, sq = twin.delivered(path), twin.sequence(seq_dir / path.name)
        walk = sq["t"] >= -1e-9
        same = int(np.sum(walk)) == len(dv["raw"]) and np.array_equal(sq["raw"][walk], dv["raw"])
        tails[path.name] = {"rows": len(dv["raw"]), "prefix_rows_from_standing": int(np.sum(~walk)), "identical_raw": bool(same)}
    report["B_delivered_are_tails"] = {"pass": all(v["identical_raw"] for v in tails.values()), "files": tails}

    # C and D over every recorded sequence; E over the STAND poses.
    sequences = sorted(p for p in seq_dir.glob("*.csv") if p.name != "stand_poses.csv")
    worst_c = {"origin_m": 0.0, "center_m": 0.0, "contact_m": 0.0}
    rows_c = 0
    law = {"stance": [], "swing": [], "excess": 0}
    per_file = {}
    for path in sequences:
        sq = twin.sequence(path)
        status, fw_origin, fw_center, fw_contact = firmware_fk(fk_binary, sq["q"])
        if np.any(status != 0):
            sys.exit(f"{path.name}: firmware FK status != OK")
        stance_err, swing_err, excess = [], [], 0
        for k, q in enumerate(sq["q"]):
            o, c, r = fk(q)
            worst_c["origin_m"] = max(worst_c["origin_m"], float(np.max(np.abs(o - fw_origin[k]))))
            worst_c["center_m"] = max(worst_c["center_m"], float(np.max(np.abs(c - fw_center[k]))))
            worst_c["contact_m"] = max(worst_c["contact_m"], float(np.max(np.abs(r - fw_contact[k]))))
            if sq["law_valid"][k] != "1":
                continue
            pos, rotation = twin.law_body_pose(sq, k)
            _, _, world = fk(q, pos, 0.0, rotation)
            target = np.array([[float(sq[f"foot{l}_{a}"][k]) for a in "xyz"] for l in range(4)])
            err = np.linalg.norm(world - target, axis=1)
            bound = rounding_bound(fk, q, pos, 0.0, rotation)
            excess += int(np.sum(err > bound + 1e-9))
            for l in range(4):
                (stance_err if sq[f"stance{l}"][k] == "1" else swing_err).append(err[l])
        rows_c += len(sq["q"])
        law["stance"] += stance_err
        law["swing"] += swing_err
        law["excess"] += excess
        per_file[path.name] = {"rows": len(sq["q"]), "law_rows": int(np.sum(sq["law_valid"] == "1")),
                               "max_stance_mm": 1e3 * max(stance_err, default=0.0), "max_swing_mm": 1e3 * max(swing_err, default=0.0),
                               "rows_beyond_rounding_bound": excess}
    report["C_mujoco_fk_equals_firmware_fk"] = {"pass": max(worst_c.values()) <= TOL_IDENTITY_M, "tolerance_m": TOL_IDENTITY_M,
                                                "rows": rows_c, "worst": worst_c}
    st, sw = np.array(law["stance"]), np.array(law["swing"])
    report["D_law_feet_from_sent_joints"] = {
        "pass": law["excess"] == 0,
        "tolerance": "per row and foot: first-order RAW rounding bound sum_j |dp/dq_j| * 0.5 tick (max reported below)",
        "foot_samples": {"stance": len(st), "swing": len(sw)},
        "stance_mm": {"max": 1e3 * st.max(), "p99": 1e3 * np.percentile(st, 99), "median": 1e3 * np.median(st)},
        "swing_mm": {"max": 1e3 * sw.max(), "p99": 1e3 * np.percentile(sw, 99), "median": 1e3 * np.median(sw)},
        "foot_errors_beyond_bound": law["excess"], "files": per_file}

    stand = twin.read_csv(seq_dir / "stand_poses.csv")
    poses, bounds_max = [], 0.0
    for k in range(len(stand["h_nominal_m"])):
        raw = np.array([int(stand[f"raw{i}"][k]) for i in range(12)])
        q = (raw - 2048) * twin.TICK
        base = float(stand["base_link_m"][k])
        _, _, contact = fk(q, (0, 0, base))
        canonical = np.array([[float(stand[f"contact{l}_{a}"][k]) for a in "xyz"] for l in range(4)])
        err = np.linalg.norm(contact - canonical, axis=1)
        bound = rounding_bound(fk, q, (0, 0, base), 0.0)
        bounds_max = max(bounds_max, float(bound.max()))
        status, *_ = firmware_fk(fk_binary, [q])
        poses.append({"h_nominal_mm": round(1e3 * float(stand["h_nominal_m"][k])), "base_link_mm": 1e3 * base,
                      "max_contact_error_mm": 1e3 * float(err.max()), "contact_z_mm": [1e3 * float(v) for v in contact[:, 2]],
                      "within_rounding_bound": bool(np.all(err <= bound + 1e-9)), "q_rad": [float(v) for v in q]})
    report["E_stand_poses_on_ground"] = {"pass": all(p["within_rounding_bound"] for p in poses),
                                         "tolerance": f"RAW rounding bound per foot (max {1e3 * bounds_max:.3f} mm)", "poses": poses}
    report["pass"] = all(v["pass"] for v in report.values() if isinstance(v, dict) and "pass" in v)
    (out_dir / "m1_kinematics.json").write_text(json.dumps(report, indent=2) + "\n")

    a, c, d, e = (report[k] for k in ("A_model_vs_urdf", "C_mujoco_fk_equals_firmware_fk", "D_law_feet_from_sent_joints", "E_stand_poses_on_ground"))
    print(f"A model vs URDF        {'PASS' if a['pass'] else 'FAIL'}  mass {a['total_mass_kg']:.4f} kg, worst {a['worst']}")
    print(f"B delivered == tails   {'PASS' if report['B_delivered_are_tails']['pass'] else 'FAIL'}  {len(tails)} files")
    print(f"C MuJoCo FK == fw FK   {'PASS' if c['pass'] else 'FAIL'}  {c['rows']} rows, worst {max(c['worst'].values()):.2e} m (tol {TOL_IDENTITY_M:.0e})")
    print(f"D law feet from q      {'PASS' if d['pass'] else 'FAIL'}  stance max {d['stance_mm']['max']:.3f} mm, swing max {d['swing_mm']['max']:.3f} mm, beyond bound {d['foot_errors_beyond_bound']}")
    print(f"E STAND on ground      {'PASS' if e['pass'] else 'FAIL'}  worst {max(p['max_contact_error_mm'] for p in e['poses']):.3f} mm ({e['tolerance']})")
    print("M1", "PASS" if report["pass"] else "FAIL")


if __name__ == "__main__":
    main()
