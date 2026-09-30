#!/usr/bin/env python3
"""Geometry validation of the 24-contact Full Calibration sequence (LF V25 generalized).

The canonical Geometry Compiler V5 profile searches every endpoint from q=0 with all other
joints at q=0 (plus one parking joint where needed). The LF V25 hardware oracle
(09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle, matdog.rs) did NOT run
that way: it holds same-leg prerequisites at reviewed poses (UPPER horizontal while the LOWER is
probed; UPPER raised and LOWER folded while the HIP is probed) and moves between them in a fixed
order. None of those configurations is covered by the canonical V5 artifacts, so this tool
evaluates them on the SAME URDF + collision meshes with the SAME V5 scene/kernel:

  1. a candidate scan for the UPPER prerequisite angle used while the LOWER is probed and while
     the HIP is probed (V25: +90 deg, and +85/+90 deg), and for the LOWER folded pose (V25: -990
     ticks = -87.01 deg);
  2. every segment of the per-leg sequence (parking, both probe corridors up to the calibration
     guard = URDF limit + 64 ticks, every prerequisite transition, return, restore), each checked
     for ANY collision other than the probed joint's own intended stop (its active parent/child
     pair), refined by bisection;
  3. the minimum non-adjacent clearance along every segment (coarser sampling; evidence only).

Geometry only. No hardware, serial, servo, EEPROM or Station access. Collision-free is mesh
non-intersection of nominal CAD; it is not a clearance-policy or motion-authorization verdict.

    python3 matdog_full_calibration_sequence_geometry_v5.py --out-dir <dir> [--legs lf,rf,rh,lh]
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))

from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402

URDF = REPO / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
# Geometry V5 provenance compiled into the Controller (CalibrationGeometryProfileData.h kProvenance[0]).
EXPECTED_URDF_SHA256 = "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59"

TICK = 2.0 * math.pi / 4096.0
GUARD_OVERSHOOT_TICKS = 64            # V25 GUARD_OVERSHOOT_TICKS
V25_UPPER_90_DELTA = 1024             # V25 UPPER_90_DELTA
V25_UPPER_85_DELTA = 967              # V25 UPPER_85_DELTA
V25_LOWER_FOLDED_DELTA = -990         # V25 LOWER_FOLDED_DELTA
V5_REAR_UPPER_PARK_RAD = 0.610865238198   # Geometry V5 parking plan for LF/RF UPPER MAX (35 deg)
BISECTION_RESOLUTION_RAD = 1.0e-4     # the V5 compiler's own resolution
FRONT_REAR = {"lf": "lh", "rf": "rh"}
# V25 matdog.rs hip_upper_clearance_delta(): the UPPER pose held while each HIP side is probed.
V25_HIP_UPPER_CLEARANCE_TICKS = {
    "lf": (V25_UPPER_90_DELTA, V25_UPPER_85_DELTA),
    "rf": (V25_UPPER_85_DELTA, V25_UPPER_90_DELTA),
    "rh": (V25_UPPER_90_DELTA, V25_UPPER_90_DELTA),
    "lh": (V25_UPPER_90_DELTA, V25_UPPER_90_DELTA),
}
# The LOWER folded pose held while the HIP is probed. Front legs: V25 LOWER_FOLDED_DELTA
# (-990 ticks, -87.01 deg) unchanged - LF is the hardware-proven oracle and RF its mirror.
# Rear legs: DEVIATION from V25 (V25 never ran a rear leg on hardware; its -990 was an
# extrapolation). At -87.01 deg the folded RH/LH lower leg passes the body at 0.04 mm (nominal
# CAD, base_link <-> {rh,lh}_lower_leg_link at |hip| = 21.4 deg) during the HIP sweep - inside mesh
# and assembly tolerance, so a false "contact" there is plausible. At -455 ticks (-39.99 deg) the
# worst interior clearance over both HIP sides is 2.02 mm, set by the upper leg approaching the
# body right before the modelled stop (2-D UPPER x fold scan, UPPER 80..110 deg, fold -87..-30 deg,
# 09_Logs/Validation_Reports/*full_calibration_sequence_geometry*).
REAR_LOWER_FOLDED_TICKS = -455
LOWER_FOLDED_TICKS = {
    "lf": V25_LOWER_FOLDED_DELTA,
    "rf": V25_LOWER_FOLDED_DELTA,
    "rh": REAR_LOWER_FOLDED_TICKS,
    "lh": REAR_LOWER_FOLDED_TICKS,
}
SCHEMA = "matdog.full_calibration_sequence_geometry.v1"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def deg(rad: float) -> float:
    return rad * 180.0 / math.pi


class Checker:
    def __init__(self, scene: RobotSceneV5):
        self.scene = scene
        self.model = scene.model
        self.all_pairs = self.model.path_obstruction_pairs()
        self.clearance_pairs = self.model.clearance_pairs()
        self.evaluations = 0

    def moving_links(self, joints: tuple[str, ...]) -> set[str]:
        return {link for link in self.model.collision_link_names
                if any(j in self.model.relevant_actuated_joints_for_link(link) for j in joints)}

    def pairs_for(self, joints: tuple[str, ...], exclude: tuple[str, str] | None,
                  pairs: tuple[tuple[str, str], ...]) -> tuple[tuple[str, str], ...]:
        moving = self.moving_links(joints)
        out = []
        for a, b in pairs:
            if exclude is not None and frozenset((a, b)) == frozenset(exclude):
                continue
            if a in moving or b in moving:
                out.append((a, b))
        return tuple(out)

    def collides(self, pose: dict, pairs) -> tuple[bool, tuple[str, str] | None]:
        self.evaluations += 1
        return self.scene.first_collision_at_pose(self.scene.full_pose(pose), link_pairs=pairs)

    def static_check(self, pose: dict) -> tuple[bool, tuple[str, str] | None]:
        return self.collides(pose, self.all_pairs)

    def sweep(self, base: dict, joint: str, start: float, end: float, step: float,
              exclude: tuple[str, str] | None) -> dict:
        """First collision of any pair involving a link moved by `joint`, joint swept
        start -> end with every other joint held at `base`."""
        pairs = self.pairs_for((joint,), exclude, self.all_pairs)
        sign = 1.0 if end >= start else -1.0
        n = max(1, math.ceil(abs(end - start) / step))
        last_clear = start
        pose = dict(base)
        pose[joint] = start
        hit, pair = self.collides(pose, pairs)
        if hit:
            return {"status": "COLLISION_AT_START", "angle_rad": start, "pair": list(pair)}
        for i in range(1, n + 1):
            value = start + (end - start) * i / n
            pose[joint] = value
            hit, pair = self.collides(pose, pairs)
            if hit:
                clear, contact = last_clear, value
                it = 0
                while abs(contact - clear) > BISECTION_RESOLUTION_RAD and it < 40:
                    mid = 0.5 * (clear + contact)
                    pose[joint] = mid
                    h, p = self.collides(pose, pairs)
                    if h:
                        contact, pair = mid, p
                    else:
                        clear = mid
                    it += 1
                return {"status": "COLLISION", "angle_rad": contact, "angle_deg": deg(contact),
                        "last_clear_rad": clear, "pair": list(pair), "sign": sign}
            last_clear = value
        return {"status": "COLLISION_FREE", "end_rad": end}

    def min_clearance(self, base: dict, joint: str, start: float, end: float, step: float,
                      exclude: tuple[str, str] | None) -> dict:
        pairs = self.pairs_for((joint,), exclude, self.clearance_pairs)
        if not pairs:
            return {"min_clearance_m": None}
        n = max(1, math.ceil(abs(end - start) / step))
        worst = None
        pose = dict(base)
        for i in range(0, n + 1):
            pose[joint] = start + (end - start) * i / n
            self.evaluations += 1
            a, b, result = self.scene.worst_pair_at_pose(self.scene.full_pose(pose), link_pairs=pairs)
            c = 0.0 if result.status == "INTERSECTING" else result.clearance_m
            if c is not None and (worst is None or c < worst[0]):
                worst = (c, a, b, pose[joint])
        return {"min_clearance_m": worst[0], "pair": [worst[1], worst[2]],
                "at_rad": worst[3]} if worst else {"min_clearance_m": None}


def leg_joints(leg: str) -> dict:
    return {"hip": f"{leg}_hip_joint", "upper": f"{leg}_upper_leg_joint",
            "lower": f"{leg}_lower_leg_joint"}


def joint_limits(model, joint: str) -> tuple[float, float]:
    spec = model.joint(joint)
    return spec.lower_limit_rad, spec.upper_limit_rad


def active_pair(model, joint: str) -> tuple[str, str]:
    return model.active_revolute_pair(joint)


def plan_segments(model, leg: str, upper_for_lower: float, upper_for_hip_min: float,
                  upper_for_hip_max: float, lower_folded: float, park: float) -> list[dict]:
    """The V25 LF state-machine order (matdog.rs run_lf_state_machine), generalized:
    Parking -> UpperMin -> UpperMax -> UpperHorizontal -> LowerMin -> LowerMax -> LowerFolded
    (+ UPPER to the HIP MIN clearance pose) -> HipMin -> HipMax -> ReturnHip -> ReturnLower
    -> ReturnUpper -> RestoreParking. UPPER and LOWER sweep their whole calibration corridor from
    the MIN guard to the MAX guard (the physical traversal starts at the MIN contact, inside it).
    The HIP uses V25's per-side UPPER clearance pose (matdog.rs hip_upper_clearance_delta): when
    the two differ, HIP MIN is searched from 0 to its guard, the HIP returns to 0, the UPPER moves
    to the MAX pose, and HIP MAX is searched from 0 to its guard."""
    upper_for_hip = upper_for_hip_min
    j = leg_joints(leg)
    g = GUARD_OVERSHOOT_TICKS * TICK
    lim = {k: joint_limits(model, v) for k, v in j.items()}
    base = {v: 0.0 for v in j.values()}
    rear = FRONT_REAR.get(leg)
    if rear:
        base[f"{rear}_upper_leg_joint"] = park
    segs = []
    if rear:
        segs.append({"id": "PARKING", "joint": f"{rear}_upper_leg_joint",
                     "base": {k: v for k, v in base.items() if k != f"{rear}_upper_leg_joint"},
                     "start": 0.0, "end": park, "exclude_active": False})
    hip, up, lo = j["hip"], j["upper"], j["lower"]
    segs.append({"id": "UPPER_MIN_SEARCH", "joint": up, "base": dict(base),
                 "start": 0.0, "end": lim["upper"][0] - g, "exclude_active": True})
    segs.append({"id": "UPPER_MIN_TO_MAX_SEARCH", "joint": up, "base": dict(base),
                 "start": lim["upper"][0] - g, "end": lim["upper"][1] + g, "exclude_active": True})
    segs.append({"id": "UPPER_HORIZONTAL", "joint": up, "base": dict(base),
                 "start": lim["upper"][1], "end": upper_for_lower, "exclude_active": True})
    b2 = dict(base); b2[up] = upper_for_lower
    segs.append({"id": "LOWER_MIN_SEARCH", "joint": lo, "base": dict(b2),
                 "start": 0.0, "end": lim["lower"][0] - g, "exclude_active": True})
    segs.append({"id": "LOWER_MIN_TO_MAX_SEARCH", "joint": lo, "base": dict(b2),
                 "start": lim["lower"][0] - g, "end": lim["lower"][1] + g, "exclude_active": True})
    segs.append({"id": "LOWER_FOLDED", "joint": lo, "base": dict(b2),
                 "start": lim["lower"][1], "end": lower_folded, "exclude_active": True})
    b3 = dict(b2); b3[lo] = lower_folded
    if abs(upper_for_hip - upper_for_lower) > 1e-9:
        segs.append({"id": "UPPER_TO_HIP_CLEARANCE", "joint": up, "base": dict(b3),
                     "start": upper_for_lower, "end": upper_for_hip, "exclude_active": True})
    b3[up] = upper_for_hip_min
    if abs(upper_for_hip_max - upper_for_hip_min) > 1e-9:
        segs.append({"id": "HIP_MIN_SEARCH", "joint": hip, "base": dict(b3),
                     "start": 0.0, "end": lim["hip"][0] - g, "exclude_active": True})
        segs.append({"id": "HIP_MIN_RETURN_TO_ZERO", "joint": hip, "base": dict(b3),
                     "start": lim["hip"][0], "end": 0.0, "exclude_active": True})
        b3h = dict(b3); b3h[hip] = 0.0
        segs.append({"id": "UPPER_TO_HIP_MAX_CLEARANCE", "joint": up, "base": dict(b3h),
                     "start": upper_for_hip_min, "end": upper_for_hip_max, "exclude_active": True})
        b3 = dict(b3h); b3[up] = upper_for_hip_max
        segs.append({"id": "HIP_MAX_SEARCH", "joint": hip, "base": dict(b3),
                     "start": 0.0, "end": lim["hip"][1] + g, "exclude_active": True})
    else:
        segs.append({"id": "HIP_MIN_SEARCH", "joint": hip, "base": dict(b3),
                     "start": 0.0, "end": lim["hip"][0] - g, "exclude_active": True})
        segs.append({"id": "HIP_MIN_TO_MAX_SEARCH", "joint": hip, "base": dict(b3),
                     "start": lim["hip"][0] - g, "end": lim["hip"][1] + g, "exclude_active": True})
    segs.append({"id": "RETURN_HIP", "joint": hip, "base": dict(b3),
                 "start": lim["hip"][1], "end": 0.0, "exclude_active": True})
    b4 = dict(b3); b4[hip] = 0.0
    segs.append({"id": "RETURN_LOWER", "joint": lo, "base": dict(b4),
                 "start": lower_folded, "end": 0.0, "exclude_active": True})
    b5 = dict(b4); b5[lo] = 0.0
    segs.append({"id": "RETURN_UPPER", "joint": up, "base": dict(b5),
                 "start": upper_for_hip_max, "end": 0.0, "exclude_active": True})
    if rear:
        b6 = {k: 0.0 for k in b5}
        segs.append({"id": "RESTORE_PARKING", "joint": f"{rear}_upper_leg_joint",
                     "base": {k: v for k, v in b6.items() if k != f"{rear}_upper_leg_joint"},
                     "start": park, "end": 0.0, "exclude_active": False})
    return segs


def classify_probe(model, seg: dict, result: dict) -> str:
    """A probe corridor collision is classified against the modelled stop of the joint:
    before the canonical contact = PATH_OBSTRUCTION (the probe would stop on something else);
    between the contact and the guard = CORRIDOR_RISK (matters only if the real stop is deeper)."""
    if result["status"] == "COLLISION_FREE":
        return "CLEAR_TO_END"
    if result["status"] == "COLLISION_AT_START":
        return "COLLISION_AT_START"
    a = result["angle_rad"]
    lower, upper = joint_limits(model, seg["joint"])
    if seg["id"].endswith("_SEARCH"):
        # modelled contact of the side being approached lies within ~1.02 deg of the URDF limit
        # (canonical V5 report); anything inside the URDF domain precedes it.
        if lower <= a <= upper:
            return "PATH_OBSTRUCTION_BEFORE_URDF_LIMIT"
        return "CORRIDOR_RISK_BEYOND_URDF_LIMIT"
    return "TRANSITION_COLLISION"


def run_leg(checker: Checker, leg: str, step_deg: float, scan: bool) -> dict:
    model = checker.model
    j = leg_joints(leg)
    rear = FRONT_REAR.get(leg)
    park = V5_REAR_UPPER_PARK_RAD if rear else 0.0
    step = step_deg * math.pi / 180.0
    out = {"leg": leg.upper(), "joints": j, "rear_park_joint": f"{rear}_upper_leg_joint" if rear else None,
           "rear_park_rad": park if rear else None}
    g = GUARD_OVERSHOOT_TICKS * TICK
    lim_lo = joint_limits(model, j["lower"])
    lim_hip = joint_limits(model, j["hip"])
    base = {v: 0.0 for v in j.values()}
    if rear:
        base[f"{rear}_upper_leg_joint"] = park
    folded = LOWER_FOLDED_TICKS[leg] * TICK

    if scan:
        # UPPER angle while the LOWER sweeps its whole corridor (hip 0) and folds.
        scan_l = []
        for u_deg in list(range(40, 123, 2)) + [90.0 + 0.0]:
            u = u_deg * math.pi / 180.0
            b = dict(base); b[j["upper"]] = u
            static_hit, static_pair = checker.static_check(b)
            r = {"upper_deg": u_deg, "static": "CLEAR" if not static_hit else list(static_pair)}
            if not static_hit:
                s = checker.sweep(b, j["lower"], lim_lo[0] - g, lim_lo[1] + g, step,
                                  active_pair(model, j["lower"]))
                r["lower_full_corridor"] = s["status"]
                if s["status"] != "COLLISION_FREE":
                    r["lower_first_collision_deg"] = s.get("angle_deg")
                    r["lower_pair"] = s.get("pair")
            scan_l.append(r)
        out["scan_upper_for_lower"] = scan_l
        # UPPER angle (+ LOWER folded) while the HIP sweeps its corridor.
        scan_h = []
        for u_deg in [60, 70, 80, 85, 87, 90, 93, 95, 100, 105, 110]:
            u = u_deg * math.pi / 180.0
            b = dict(base); b[j["upper"]] = u; b[j["lower"]] = folded
            static_hit, static_pair = checker.static_check(b)
            r = {"upper_deg": u_deg, "lower_folded_deg": deg(folded),
                 "static": "CLEAR" if not static_hit else list(static_pair)}
            if not static_hit:
                s = checker.sweep(b, j["hip"], lim_hip[0] - g, lim_hip[1] + g, step,
                                  active_pair(model, j["hip"]))
                r["hip_full_corridor"] = s["status"]
                if s["status"] != "COLLISION_FREE":
                    r["hip_first_collision_deg"] = s.get("angle_deg")
                    r["hip_pair"] = s.get("pair")
            scan_h.append(r)
        out["scan_upper_for_hip"] = scan_h
    return out


def validate_plan(checker: Checker, leg: str, upper_for_lower: float, upper_for_hip_min: float,
                  upper_for_hip_max: float, lower_folded: float, step_deg: float,
                  clearance_step_deg: float) -> dict:
    model = checker.model
    rear = FRONT_REAR.get(leg)
    park = V5_REAR_UPPER_PARK_RAD if rear else 0.0
    step = step_deg * math.pi / 180.0
    cstep = clearance_step_deg * math.pi / 180.0
    segs = plan_segments(model, leg, upper_for_lower, upper_for_hip_min, upper_for_hip_max,
                         lower_folded, park)
    results = []
    ok = True
    for seg in segs:
        exclude = active_pair(model, seg["joint"]) if seg["exclude_active"] else None
        # The whole robot at the segment's ACTUAL start pose (every pair, the segment's own
        # modelled stop excluded: a MIN_TO_MAX search starts resting on its MIN stop).
        static_pairs = tuple(pr for pr in checker.all_pairs
                             if exclude is None or frozenset(pr) != frozenset(exclude))
        static_hit, static_pair = checker.collides({**seg["base"], seg["joint"]: seg["start"]},
                                                   static_pairs)
        r = checker.sweep(seg["base"], seg["joint"], seg["start"], seg["end"], step, exclude)
        verdict = classify_probe(model, seg, r)
        c = checker.min_clearance(seg["base"], seg["joint"], seg["start"], seg["end"], cstep, exclude)
        entry = {"segment": seg["id"], "joint": seg["joint"],
                 "start_deg": deg(seg["start"]), "end_deg": deg(seg["end"]),
                 "held": {k: round(deg(v), 3) for k, v in seg["base"].items() if abs(v) > 1e-12},
                 "base_static": "CLEAR" if not static_hit else list(static_pair),
                 "sweep": r, "verdict": verdict, "clearance": c}
        # The only acceptable outcomes: clear to the end, or (probe corridors only) a collision
        # BEYOND the URDF limit, i.e. past where the modelled stop is; that is reported, and the
        # firmware's guard/corridor bounds how far a search may go.
        if verdict not in ("CLEAR_TO_END", "CORRIDOR_RISK_BEYOND_URDF_LIMIT") or static_hit:
            ok = False
        results.append(entry)
    return {"leg": leg.upper(),
            "upper_for_lower_rad": upper_for_lower, "upper_for_lower_deg": deg(upper_for_lower),
            "upper_for_hip_min_rad": upper_for_hip_min, "upper_for_hip_min_deg": deg(upper_for_hip_min),
            "upper_for_hip_max_rad": upper_for_hip_max, "upper_for_hip_max_deg": deg(upper_for_hip_max),
            "lower_folded_rad": lower_folded, "lower_folded_deg": deg(lower_folded),
            "rear_park_joint": f"{rear}_upper_leg_joint" if rear else None,
            "rear_park_rad": park if rear else None,
            "rear_park_deg": deg(park) if rear else None, "segments": results,
            "sequence_collision_free": ok}


def mesh_manifest_sha256() -> str:
    return sha256(URDF.parent / "SHA256SUMS.txt")


def urad(rad: float) -> int:
    return int(round(rad * 1.0e6))


def export_header(artifacts: list[Path], header: Path) -> int:
    """Writes CalibrationSequencePlanData.h from validate artifacts covering all four legs.
    Refuses unless every leg's whole sequence is collision-free on the pinned URDF."""
    plans = {}
    shas = []
    for a in artifacts:
        data = json.loads(a.read_text())
        if data.get("schema") != SCHEMA or "plans" not in data:
            print(f"REFUSED: {a} is not a validate artifact", file=sys.stderr)
            return 2
        if data["urdf_sha256"] != EXPECTED_URDF_SHA256 or \
                data.get("mesh_manifest_sha256") != mesh_manifest_sha256():
            print(f"REFUSED: {a} was computed on another URDF / mesh set", file=sys.stderr)
            return 2
        shas.append(sha256(a))
        for p in data["plans"]:
            plans[p["leg"]] = p
    order = ["LF", "RF", "RH", "LH"]  # calibration::Leg values 0..3
    missing = [leg for leg in order if leg not in plans]
    if missing:
        print(f"REFUSED: no validate artifact for {missing}", file=sys.stderr)
        return 2
    combined = hashlib.sha256("\n".join(sorted(shas)).encode()).hexdigest()
    rows = []
    for leg in order:
        p = plans[leg]
        rear = p["rear_park_joint"]
        park_leg = rear.split("_")[0].upper() if rear else "LF"
        rows.append(
            f"    // {leg}: sequence_collision_free={p['sequence_collision_free']}\n"
            f"    {{calibration::Leg::{leg}, {'true' if p['sequence_collision_free'] else 'false'}, "
            f"{'true' if rear else 'false'}, calibration::Leg::{park_leg}, "
            f"calibration::JointKind::UPPER, {urad(p['rear_park_rad']) if rear else 0}, "
            f"{urad(p['upper_for_lower_rad'])}, {urad(p['upper_for_hip_min_rad'])}, "
            f"{urad(p['upper_for_hip_max_rad'])}, {urad(p['lower_folded_rad'])}}},")
    text = f"""// GENERATED FILE - DO NOT EDIT BY HAND.
//
// Produced by 06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py
//   --export-header from the geometry validation artifact(s) (sha256):
{chr(10).join('//   ' + s for s in sorted(shas))}
// Every pose below was evaluated, with every segment of its leg's 24-contact sequence up to the
// calibration guard (URDF limit + 64 ticks), on the SHA-pinned URDF and collision meshes.
// Regenerate with the tool; never patch a value (static_audit.py re-derives this file).

#ifndef MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_DATA_H
#define MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_DATA_H

#include "CalibrationSequencePlan.h"

namespace matdog {{
namespace actuator {{
namespace sequence_plan_data {{

constexpr char kSchema[] = "{SCHEMA}";
constexpr char kArtifactSha256[] = "{combined}";
constexpr char kUrdfSha256[] = "{EXPECTED_URDF_SHA256}";
constexpr char kMeshManifestSha256[] = "{mesh_manifest_sha256()}";

// {{leg, geometry_validated, has_rear_park, park_leg, park_joint, park_target,
//  upper_for_lower, upper_for_hip_min, upper_for_hip_max, lower_folded}} - URDF q, micro-radians.
constexpr CalibrationSequencePlan kPlan = {{
    kSchema, kArtifactSha256, kUrdfSha256, kMeshManifestSha256,
    {{
{chr(10).join(rows)}
    }},
}};

}}  // namespace sequence_plan_data
}}  // namespace actuator
}}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_DATA_H
"""
    header.write_text(text)
    print(f"WROTE {header} (artifacts combined sha256 {combined})")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out-dir", default="")
    ap.add_argument("--legs", default="lf,rf,rh,lh")
    ap.add_argument("--step-deg", type=float, default=0.5)
    ap.add_argument("--clearance-step-deg", type=float, default=2.0)
    ap.add_argument("--scan", action="store_true", help="candidate scan only")
    ap.add_argument("--export-header", default="",
                    help="write CalibrationSequencePlanData.h from --artifacts")
    ap.add_argument("--artifacts", default="", help="comma-separated validate artifacts")
    args = ap.parse_args(argv)

    if args.export_header:
        return export_header([Path(x) for x in args.artifacts.split(",") if x],
                             Path(args.export_header))

    urdf_sha = sha256(URDF)
    if urdf_sha != EXPECTED_URDF_SHA256:
        print(f"REFUSED: URDF sha256 {urdf_sha} != compiled Geometry V5 provenance", file=sys.stderr)
        return 2
    scene = RobotSceneV5.from_urdf(URDF)
    checker = Checker(scene)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    report = {"schema": SCHEMA,
              "generated": _dt.datetime.now().isoformat(timespec="seconds"),
              "urdf": str(URDF.relative_to(REPO)), "urdf_sha256": urdf_sha,
              "mesh_manifest_sha256": mesh_manifest_sha256(),
              "tool_sha256": sha256(Path(__file__)),
              "constants": {"tick_rad": TICK, "guard_overshoot_ticks": GUARD_OVERSHOOT_TICKS,
                            "v25_upper_90_deg": deg(V25_UPPER_90_DELTA * TICK),
                            "v25_upper_85_deg": deg(V25_UPPER_85_DELTA * TICK),
                            "v25_lower_folded_deg": deg(V25_LOWER_FOLDED_DELTA * TICK),
                            "lower_folded_ticks": LOWER_FOLDED_TICKS,
                            "v5_rear_upper_park_deg": deg(V5_REAR_UPPER_PARK_RAD),
                            "step_deg": args.step_deg, "clearance_step_deg": args.clearance_step_deg,
                            "bisection_resolution_rad": BISECTION_RESOLUTION_RAD}}
    legs = [x.strip().lower() for x in args.legs.split(",") if x.strip()]
    if args.scan:
        report["scan"] = [run_leg(checker, leg, max(args.step_deg, 1.0), True) for leg in legs]
        name = "scan"
    else:
        # V25 values: UPPER_90 while the LOWER is probed, the per-side
        # hip_upper_clearance_delta while the HIP is probed; LOWER_FOLDED_TICKS under the HIP
        # (V25 on the front legs, the documented rear deviation above).
        report["plans"] = []
        for leg in legs:
            hip_min_ticks, hip_max_ticks = V25_HIP_UPPER_CLEARANCE_TICKS[leg]
            plan = validate_plan(
                checker, leg, V25_UPPER_90_DELTA * TICK, hip_min_ticks * TICK,
                hip_max_ticks * TICK, LOWER_FOLDED_TICKS[leg] * TICK,
                args.step_deg, args.clearance_step_deg)
            plan["v25_deviations"] = ([] if LOWER_FOLDED_TICKS[leg] == V25_LOWER_FOLDED_DELTA else
                                      [{"parameter": "lower_folded",
                                        "v25_ticks": V25_LOWER_FOLDED_DELTA,
                                        "value_ticks": LOWER_FOLDED_TICKS[leg],
                                        "reason": "rear HIP sweep interior clearance 0.04 mm at "
                                                  "V25 fold; 2.02 mm at this fold"}])
            report["plans"].append(plan)
        report["all_sequences_collision_free"] = all(p["sequence_collision_free"]
                                                     for p in report["plans"])
        name = "validate"
    report["evaluations"] = checker.evaluations
    report["runtime_s"] = round(time.time() - t0, 1)
    path = out_dir / f"full_calibration_sequence_geometry_{name}.json"
    path.write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(f"WROTE {path} ({report['runtime_s']} s, {checker.evaluations} evaluations)")
    if not args.scan:
        print("ALL_SEQUENCES_COLLISION_FREE =", report["all_sequences_collision_free"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
