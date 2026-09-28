#!/usr/bin/env python3
"""Differential validation against existing MATDOG FK/IK/contact oracles.

No new Python kinematics implementation. C4 data are replayed as foot-origin
IK targets derived by canonical FK; the recovered angles are then checked by
the existing contact oracle at the archived world body pose.
"""
from __future__ import annotations
import hashlib
import itertools
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
KIN = ROOT / "06_Software/Matdog_Core/kinematics"
sys.path.insert(0, str(KIN))
from matdog_urdf_fk import (canonical_urdf_path, forward_kinematics,
                            load_urdf_joints, sha256_file, CANONICAL_URDF_SHA256)
from matdog_leg_ik import solve_lf_position_ik
from matdog_quadruped_leg_contact import leg_foot_contact_from_joint_angles, leg_joint_names
from matdog_quadruped_leg_contact_ik import solve_leg_contact_reference_ik
from matdog_motion_geometry_export import render, OUTPUT

LEGS = ("lf", "rf", "rh", "lh")
C4A = ROOT / "09_Logs/Validation_Reports/2026-07-08_175245_C4A_offline_safe_stand_candidate.json"
C4C = ROOT / "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json"
URDF = canonical_urdf_path(ROOT)
FK_TOLERANCE = 1e-12  # metres; rotation entries use the same absolute threshold
IK_TOLERANCE = 1e-9   # metres
CONTACT_TOLERANCE = 1e-5  # existing C4 contract, metres


def require(value, message):
    if not value:
        raise AssertionError(message)


def fk(leg, q):
    return forward_kinematics(URDF, "base_link", f"{leg}_foot_link",
                              dict(zip(leg_joint_names(leg), q)))


def check_exporter():
    require(sha256_file(URDF) == CANONICAL_URDF_SHA256, "oracle geometry hash mismatch")
    require(OUTPUT.read_text() == render(URDF), "stale generated model")
    # Fail closed on changes that would invalidate the analytic reduction.
    mutations = [
        ("lf_hip_joint", "axis", "xyz", "0 1 0"),
        ("rf_upper_leg_joint", "origin", "xyz", "0.001 -0.048 0"),
        ("rh_lower_leg_joint", "origin", "rpy", "0 0.1 0"),
        ("lh_foot_joint", "origin", "xyz", "0 -0.0015 0"),
        ("lf_hip_joint", "limit", "upper", "4"),
        ("lf_lower_leg_joint", "origin", "xyz", "0 0 0"),
        ("lf_foot_joint", "origin", "xyz", "0.107 -0.048 -0.0499"),
    ]
    with tempfile.TemporaryDirectory(prefix="matdog-motion-model-") as directory:
        for joint, tag, attribute, value in mutations:
            root = ET.parse(URDF)
            node = root.find(f"./joint[@name='{joint}']/{tag}")
            node.set(attribute, value)
            path = Path(directory) / "mutated.urdf"
            root.write(path)
            try:
                render(path)
            except ValueError:
                continue
            raise AssertionError(f"exporter accepted unsupported {joint}/{tag}/{attribute}")
    return len(mutations)


def main():
    require(len(sys.argv) == 2, "usage: test_motion_oracle.py COMPILED_DRIVER")
    mutation_count = check_exporter()
    rng = random.Random(20260928)
    joints = load_urdf_joints(URDF)
    cases = []

    def add(leg, q, seed, label, reference=None):
        oracle = fk(leg, q)
        cases.append((leg, q, seed, label, oracle, reference))

    for leg in LEGS:
        limits = [(joints[n].lower_limit_rad, joints[n].upper_limit_rad)
                  for n in leg_joint_names(leg)]
        add(leg, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), "q0")
        for q in itertools.product(*[(lo, (lo + hi) / 2, hi) for lo, hi in limits]):
            add(leg, q, q, "limit_grid")
        for index in range(512):
            q = tuple(rng.uniform(lo, hi) for lo, hi in limits)
            # Cover both nearest-branch recovery and reachability from zero.
            seed = q if index % 2 == 0 else (0.0, 0.0, 0.0)
            add(leg, q, seed, "random")

    # The LF DLS solver targets foot_link, exactly the C++ API's semantics.
    python_dls_max = 0.0
    for _ in range(12):
        q = (rng.uniform(-0.2, 0.2), rng.uniform(0.1, 0.7), rng.uniform(-0.4, 0.3))
        target = fk("lf", q).tip_position_m
        solved = solve_lf_position_ik(ROOT, target, tolerance_m=1e-8)
        python_dls_max = max(python_dls_max, solved.residual_m)
        add("lf", q, (0.0, 0.0, 0.0), "python_dls")

    c4a, c4c = (json.loads(p.read_text()) for p in (C4A, C4C))
    require(len(c4c["frames"]) == 51, "expected archived 51-frame C4-C")
    require(c4a["offline_only"] and c4c["offline_only"], "reference scope changed")
    require(not c4a["command_eligibility"]["command_eligible"] and
            not c4c["command_eligibility"]["command_eligible"], "eligibility changed")
    contact_ik_count = 0
    contact_ik_max = 0.0
    frames = [("C4A", c4a)] + [(f"C4C_{i}", frame) for i, frame in enumerate(c4c["frames"])]
    previous = {}
    for label, frame in frames:
        require(frame["body_pose"]["base_link_parallel_to_ground"], "unsupported body rotation")
        if label != "C4A":
            require(frame["evaluation"]["safe"], "archived C4-C frame unsafe")
        translation = tuple(frame["body_pose"]["translation_world_m"])
        for leg in LEGS:
            record = frame["legs"][leg]
            q = tuple(record["joint_positions_rad"][n] for n in leg_joint_names(leg))
            reference = (record, translation)
            add(leg, q, previous.get(leg, (0.0, 0.0, 0.0)), label, reference)
            previous[leg] = q
            # Independently run existing contact DLS on stand + start/middle/end.
            if label in ("C4A", "C4C_0", "C4C_25", "C4C_50"):
                result = solve_leg_contact_reference_ik(
                    leg, tuple(record["target_contact_reference_world_m"]),
                    repo_root=ROOT, initial_guess_rad=(q[0], q[1] + 0.001, q[2]),
                    world_from_base_translation_m=translation, tolerance_m=CONTACT_TOLERANCE)
                contact_ik_count += 1
                contact_ik_max = max(contact_ik_max, result.residual_m)

    lines = []
    for leg, q, seed, _, oracle, _ in cases:
        values = (LEGS.index(leg), *q, *oracle.tip_position_m, *seed)
        lines.append(" ".join(str(v) for v in values))
    payload = "\n".join(lines) + "\n"
    output = subprocess.run([sys.argv[1]], input=payload, text=True, capture_output=True, check=True).stdout
    repeated = subprocess.run([sys.argv[1]], input=payload, text=True, capture_output=True, check=True).stdout
    require(output == repeated, "C++ repeated-run output is not deterministic")
    rows = output.splitlines()
    require(len(rows) == len(cases), "driver result count mismatch")
    max_fk = max_rotation = max_ik = max_contact = max_archived_contact = 0.0
    contact_count = 0
    for case, row in zip(cases, rows):
        leg, q, seed, label, oracle, reference = case
        values = [float(v) for v in row.split()]
        require(len(values) == 19 and all(math.isfinite(v) for v in values), f"bad output {label}")
        require(values[0] == 0 and values[13] == 0, f"FK/IK failed {leg}/{label}: {row}")
        fk_error = math.dist(values[1:4], oracle.tip_position_m)
        rotation_error = max(abs(values[4 + 3*r+c] - oracle.tip_transform[r][c])
                             for r in range(3) for c in range(3))
        solved_q = values[14:17]
        solved_fk = fk(leg, solved_q)  # also enforces every URDF joint limit
        ik_error = math.dist(solved_fk.tip_position_m, oracle.tip_position_m)
        require(fk_error <= FK_TOLERANCE, f"FK mismatch {leg}/{label}: {fk_error}")
        require(rotation_error <= FK_TOLERANCE, f"rotation mismatch {leg}/{label}")
        require(ik_error <= IK_TOLERANCE, f"IK mismatch {leg}/{label}: {ik_error}")
        require(abs(ik_error - values[17]) <= FK_TOLERANCE, "incorrect reported residual")
        require(1 <= values[18] <= 4, "invalid solution count")
        if seed == q:
            require(max(abs(a-b) for a, b in zip(solved_q, q)) <= 1e-8,
                    f"nearest-seed branch mismatch {leg}/{label}")
        max_fk, max_rotation, max_ik = max(max_fk, fk_error), max(max_rotation, rotation_error), max(max_ik, ik_error)
        if reference:
            record, translation = reference
            contact = leg_foot_contact_from_joint_angles(
                leg, dict(zip(leg_joint_names(leg), solved_q)), repo_root=ROOT,
                world_from_base_translation_m=translation).contact
            point = contact.cross_section_contact_center_world_m
            contact_error = math.dist(point, record["target_contact_reference_world_m"])
            archived_error = math.dist(point, record["achieved_contact_reference_world_m"])
            require(contact.support_mode == record["support_mode"] == "NOMINAL_STRIP_CONTACT",
                    f"contact mode mismatch {leg}/{label}")
            require(contact_error <= CONTACT_TOLERANCE and abs(point[2]) <= CONTACT_TOLERANCE,
                    f"contact target mismatch {leg}/{label}: {contact_error}")
            require(archived_error <= 1e-10, f"archived contact drift {leg}/{label}: {archived_error}")
            max_contact, max_archived_contact = max(max_contact, contact_error), max(max_archived_contact, archived_error)
            contact_count += 1

    # Check the pure module's actual include closure and absence of allocation/I/O.
    source_dir = ROOT / "05_Firmware/MATDOG_Controller/src/motion"
    allowed = {'"LegKinematics.h"', '"LegGeometryData.h"', '<stdint.h>', '<cmath>', '<limits>'}
    for path in source_dir.iterdir():
        for line in path.read_text().splitlines():
            if line.startswith("#include "):
                require(line.removeprefix("#include ") in allowed, f"dependency boundary: {path}: {line}")
    print(f"MOTION_ORACLE = PASS: {len(cases)} vectors, {contact_count} C4 contacts, "
          f"12 Python LF DLS + {contact_ik_count} Python contact DLS, "
          f"{mutation_count} rejected model mutations; repeated output identical")
    print(f"max FK={max_fk:.3e} m; rotation={max_rotation:.3e}; IK={max_ik:.3e} m")
    print(f"max Python LF DLS={python_dls_max:.3e} m; contact DLS={contact_ik_max:.3e} m")
    print(f"max C4 target contact={max_contact:.3e} m; archived contact drift={max_archived_contact:.3e} m")
    for path in (URDF, KIN / "MATDOG_FOOT_CONTACT_GEOMETRY.yaml", C4A, C4C):
        print(f"sha256 {path.relative_to(ROOT)} {hashlib.sha256(path.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    main()
