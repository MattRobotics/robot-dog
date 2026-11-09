#!/usr/bin/env bash
# Perturbation matrix (dev.6 consolidation), host only: production SimpleMotion
# on the rig of test_simple_motion.cpp with IMU bias, per-joint offsets with
# sag/load, feedback lag, release jitter, missed releases, SYNC_WRITE time,
# low feedback voltage and all combined, crossed with WALK100/WALK75 (160 s),
# the held joystick sequence and seeded random joystick runs (one with STOP at
# a random instant and STAND recovery). Software only, not a hardware result.
# Usage: SEEDS=4 run_perturbation_matrix.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)";out="$(mktemp -d /tmp/matdog-perturb.XXXXXX)";trap 'rm -rf "$out"' EXIT
python3 -I - "$sketch/scripts/tests/test_simple_motion.cpp" "$here/perturbation_matrix.inc" "$out/probe.cpp" <<'PY'
import sys;src,inc,dst=sys.argv[1:];t=open(src).read();i=t.index("int main(){")
t='#include <deque>\n#include <random>\n#include <string>\n'+t[:i]+open(inc).read()+"int main(){perturb::matrix();return 0;}\nstatic int unused_main(){"+t[i+len("int main(){"):]
open(dst,'w').write(t.replace('#include "../../src/','#include "SKETCH_SRC/'))
PY
sed -i "s#SKETCH_SRC/#$sketch/src/#g" "$out/probe.cpp"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -w -I "$sketch/scripts/tests/simple_host_stub" -I "$sketch/scripts/tests" -std=c++17 -O2 -DMATDOG_T2_SHADOW "$out/probe.cpp" "${src[@]}" -o "$out/perturb"
"$out/perturb"
