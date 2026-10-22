#!/usr/bin/env python3
"""Recover KISS runtime geometry bounds from existing continuous CAD/FCL proofs.
Offline file conversion only. Does not operate a robot or create a certificate.
"""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--evidence-root", type=Path, required=True,
                    help="existing TOLERANCE_V1_20261018 evidence directory")
parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "src/simple/CollisionData.h")
args = parser.parse_args()
keys = [leg + "_" + joint + "_joint" for leg in ("lf", "rf", "rh", "lh")
        for joint in ("hip", "upper_leg", "lower_leg")]
files = ["entry_region_40mrad_bounded.json", "walk_region_40mrad.json",
         "height_region_25mrad.json", "trajectory-optimization/phase_chords-certificate.json"]
raw = [(args.evidence_root / name).read_bytes() for name in files]
proofs = [json.loads(data) for data in raw]
if not all(proof["all_pass"] for proof in proofs):
    raise SystemExit("cannot recover a failed geometry proof")

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
cells = proofs[3]["cells"]
text += "constexpr Box kStandCells[]={\n" + ",\n".join(box(cell["box_rad"]) for cell in cells) + "\n};\n"
text += "constexpr unsigned kStandCellCount=sizeof(kStandCells)/sizeof(kStandCells[0]);\n"
for name, data in zip(("kEntryProof", "kWalkProof", "kHeightProof", "kStandProof"), raw):
    text += 'constexpr char ' + name + '[]="' + hashlib.sha256(data).hexdigest() + '";\n'
text += "constexpr unsigned kPhaseKnots[]={0,30,80,100,107,251};\n"
text += "constexpr unsigned kCellEdge[]={" + ",".join(str(cell["edge"]) for cell in cells) + "};\n"
text += "constexpr double kCellEnd[]={" + ",".join(format(cell["alpha"][1], ".17g") for cell in cells) + "};\n"
args.output.write_text(text + "} } }\n")
