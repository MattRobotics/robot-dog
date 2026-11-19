#!/usr/bin/env bash
# Bench-only probes on a COPY of src (never the firmware): MotionTiming.h reads the bench clock (the 17/11 CCOUNT hunk)
# and one inclusive scope probe at the entry of each function of the WALK path. Usage: bench_probes.sh <copied src dir>
set -euo pipefail
R="$(cd "$(dirname "$0")" && pwd)"; S=$1
patch -s "$S/simple/MotionTiming.h" < "$R/MotionTiming_ccount.diff"
python3 -I - "$S" <<'PY'
import sys, pathlib
S = pathlib.Path(sys.argv[1])
def probe(p, head, pid):
    t = p.read_text(); n = t.count(head)
    if n != 1: raise SystemExit(f"{p}: {n} x {head[:70]}")
    p.write_text(t.replace(head, head + f"\n  QPROBE({pid});", 1))
sm = S / "simple/SimpleMotion.cpp"
t = sm.read_text(); sm.write_text('#include "qprobe.h"\n' + t)
for head, pid in [
    ("void SimpleMotion::generateWalk(uint64_t now){", "GENWALK"),
    ("void SimpleMotion::startWalk(uint64_t now){", "STARTWALK"),
    ("void SimpleMotion::steerWalk(uint64_t now){", "STEER"),
    ("void SimpleMotion::stepWalkMode(double eased){", "STEPMODE"),
    ("void SimpleMotion::nominalRelease(LocomotionFrame& f,const GaitParameters& g,double terminal,bool clockDone){", "NOMREL"),
    ("bool SimpleMotion::regimeHandover(bool toSteered) const {", "HANDOVER"),
    ("bool SimpleMotion::loadedArrestInside(const JointTargetFrame& q,const collision::Box& box) const {", "ARREST"),
    ("void SimpleMotion::lerpFrame(LocomotionFrame& f,double u,const GaitParameters& g,double terminal) const {", "LERP"),
    ("void SimpleMotion::compensate(LocomotionFrame& f,uint64_t now){", "COMP"),
    ("MATDOG_MOTION_HOT JointTargetFrame SimpleMotion::compensationTarget(const LocomotionFrame& f,const BodyPose& body,const JointTargetFrame& seed,bool nominalStrip,double edgeTilt){", "COMPTARGET"),
    ("bool SimpleMotion::compensationSafe(const JointTargetFrame& q,const collision::Box& box,bool stopProof) const {", "COMPSAFE"),
    ("void SimpleMotion::beginNative(Phase next,const JointTargetFrame& to,const collision::Box& box,uint64_t now,double pitch,double x,double z){", "BEGINNATIVE"),
    ("MATDOG_MOTION_HOT bool SimpleMotion::pointSafe(const JointTargetFrame& q) const {", "POINTSAFE")]:
    probe(sm, head, pid)
g = S / "motion/Gait.cpp"
t = g.read_text(); g.write_text('#include "qprobe.h"\n' + t)
probe(g, "void solveGaitInto(LocomotionFrame& out,const GaitParameters& p,const CartesianSample& s,const LocomotionFrame* prev,bool derivatives){", "SOLVEGAIT")
fc = S / "motion/FootContact.cpp"
t = fc.read_text(); fc.write_text('#include "qprobe.h"\n' + t)
# FP32 trees (18/11): the hot contact IK is the core detail::contactInverse, the public wrapper is off the WALK path.
fp32 = "detail::ContactIkF detail::contactInverse(" in fc.read_text()
probe(fc, "float toleranceM, bool requireNominalStrip) {" if fp32 else "const ContactIkOptions& options) {", "CIK")
probe(g, "bool gaitRetarget(GaitTrack& track,const GaitParameters& from,const GaitParameters& to,double cycles,double terminal){", "RETARGET")
PY
