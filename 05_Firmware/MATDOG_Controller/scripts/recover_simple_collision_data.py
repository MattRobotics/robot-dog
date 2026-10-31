#!/usr/bin/env python3
"""Recover KISS runtime geometry bounds from existing continuous CAD/FCL proofs.
With --trajectory-root the WALK box is the V2 proof and the STAND route uses the
whole-group DEPLOY/LIFT proofs plus the decimated route vias (no per-cell stops).
Offline file conversion only. Does not operate a robot or create a certificate.
"""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--evidence-root", type=Path,
                    help="existing TOLERANCE_V1_20261018 evidence directory")
parser.add_argument("--optimization-root", type=Path, help="SIMPLE_OPT_20261022 continuous box proofs")
parser.add_argument("--trajectory-root", type=Path, help="SIMPLE_TRAJ_V2_20261022 WALK/DEPLOY/LIFT box proofs and route vias")
parser.add_argument("--certificate-root", type=Path, help="one directory holding the ENTRY/WALK/HEIGHT/DEPLOY/LIFT box certificates and stand_vias.json of a recertified CAD; replaces the three roots above")
parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "src/simple/CollisionData.h")
args = parser.parse_args()
keys = [leg + "_" + joint + "_joint" for leg in ("lf", "rf", "rh", "lh")
        for joint in ("hip", "upper_leg", "lower_leg")]
def recertified(root):
    names = ("ENTRY", "WALK", "HEIGHT", "DEPLOY", "LIFT")
    raw = [(root / (name + "_certificate.json")).read_bytes() for name in names]
    proofs = [json.loads(data) for data in raw]
    if not all(proof["all_pass"] for proof in proofs):
        raise SystemExit("cannot recover a failed geometry proof")
    vias = json.loads((root / "stand_vias.json").read_bytes())
    return raw, proofs, vias
files = ["entry_region_40mrad_bounded.json", "walk_region_40mrad.json",
         "height_region_25mrad.json", "trajectory-optimization/phase_chords-certificate.json"]
trajectory = None
if args.certificate_root:
    raw, proofs, vias = recertified(args.certificate_root)
    trajectory = (proofs[3], raw[3], proofs[4], raw[4], vias, 160)
    raw, proofs = raw[:3], proofs[:3]
else:
    raw = [(args.evidence_root / name).read_bytes() for name in files]
    proofs = [json.loads(data) for data in raw]
    if not all(proof["all_pass"] for proof in proofs):
        raise SystemExit("cannot recover a failed geometry proof")

if args.optimization_root:
    height_raw = (args.optimization_root / "height50-certificate.json").read_bytes()
    route_raw = (args.optimization_root / "stand-segments-certificate.json").read_bytes()
    height, route = json.loads(height_raw), json.loads(route_raw)
    if not height["all_pass"] or not route["all_pass"]:
        raise SystemExit("cannot recover a failed SIMPLE optimization proof")
    proofs[2], proofs[3] = height, route
    raw[2], raw[3] = height_raw, route_raw

if args.trajectory_root:
    walk_raw = (args.trajectory_root / "walk_v2_certificate.json").read_bytes()
    deploy_raw = (args.trajectory_root / "DEPLOY_0_107_certificate.json").read_bytes()
    lift_name = "LIFT_107_251_certificate.json"
    if not (args.trajectory_root / lift_name).exists() or not json.loads((args.trajectory_root / lift_name).read_bytes())["all_pass"]:
        lift_name = "LIFT_107_160_certificate.json"  # whole rise not proved: lift to 50 mm only
    lift_raw = (args.trajectory_root / lift_name).read_bytes()
    vias = json.loads((args.trajectory_root / "stand_vias.json").read_bytes())
    walk, deploy, lift = (json.loads(x) for x in (walk_raw, deploy_raw, lift_raw))
    if not (walk["all_pass"] and deploy["all_pass"] and lift["all_pass"]):
        raise SystemExit("cannot recover a failed SIMPLE trajectory proof")
    proofs[1], raw[1] = walk, walk_raw
    trajectory = (deploy, deploy_raw, lift, lift_raw, vias, 251 if lift_name.startswith("LIFT_107_251") else 160)

def array(values):
    return "{" + ",".join(format(value, ".17g") for value in values) + "}"

def box(bounds):
    return "{" + array([bounds[key][0] for key in keys]) + "," + array([bounds[key][1] for key in keys]) + "}"

text = """#pragma once
// Recovered continuous CAD/FCL clearances; no measured tracking constraint.
// Canonical MATDOG geometry only. Reproduce with scripts/recover_simple_collision_data.py.
namespace matdog { namespace simple { namespace collision {
struct Box { double lo[12], hi[12]; };
"""
for name, proof in zip(("kEntry", "kWalk", "kHeight"), proofs):
    text += "constexpr Box " + name + "=" + box(proof["box_rad"]) + ";\n"
if trajectory:
    deploy, deploy_raw, lift, lift_raw, vias, top = trajectory
    text += "constexpr Box kDeploy=" + box(deploy["box_rad"]) + ";\n"
    text += "constexpr Box kLift=" + box(lift["box_rad"]) + ";\n"
    text += "constexpr unsigned kDeployKnots[]={" + ",".join(map(str, vias["deploy"])) + "};\n"
    text += "constexpr unsigned kLiftKnots[]={" + ",".join(str(k) for k in vias["lift"] if k <= top) + "};\n"
    text += "constexpr unsigned kLiftTopKnot=" + str(top) + ";\n"
    for name, data in zip(("kEntryProof", "kWalkProof", "kHeightProof", "kDeployProof", "kLiftProof"),
                          (raw[0], raw[1], raw[2], deploy_raw, lift_raw)):
        text += 'constexpr char ' + name + '[]="' + hashlib.sha256(data).hexdigest() + '";\n'
    args.output.write_text(text + "} } }\n")
    raise SystemExit(0)
cells = proofs[3]["cells"]
text += "constexpr Box kStandCells[]={\n" + ",\n".join(box(cell["box_rad"]) for cell in cells) + "\n};\n"
text += "constexpr unsigned kStandCellCount=sizeof(kStandCells)/sizeof(kStandCells[0]);\n"
for name, data in zip(("kEntryProof", "kWalkProof", "kHeightProof", "kStandProof"), raw):
    text += 'constexpr char ' + name + '[]="' + hashlib.sha256(data).hexdigest() + '";\n'
text += "constexpr unsigned kPhaseKnots[]={0,30,80,100,107,251};\n"
text += "constexpr unsigned kCellEdge[]={" + ",".join(str(cell["edge"]) for cell in cells) + "};\n"
text += "constexpr double kCellEnd[]={" + ",".join(format(cell["alpha"][1], ".17g") for cell in cells) + "};\n"
args.output.write_text(text + "} } }\n")
