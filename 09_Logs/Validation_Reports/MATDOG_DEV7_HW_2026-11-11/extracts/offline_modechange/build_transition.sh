#!/usr/bin/env bash
# build_transition.sh SKETCH OUT MAIN_INC : test_simple_motion prefix + servo_plant.inc + MAIN_INC
set -euo pipefail
sketch=$1;out=$2;main=$3;mkdir -p "$out"
python3 -I - "$sketch/scripts/tests/test_simple_motion.cpp" "$sketch/scripts/offline_motion/servo_plant.inc" "$main" "$out/probe.cpp" "$sketch" <<'PY'
import sys;src,inc,main,dst,sk=sys.argv[1:];t=open(src).read();i=t.index("int main(){")
t='#include <deque>\n#include <random>\n#include <string>\n'+t[:i]+open(inc).read()+open(main).read()
open(dst,'w').write(t.replace('#include "../../src/','#include "'+sk+'/src/'))
PY
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
g++ -w -I "$sketch/scripts/tests/simple_host_stub" -I "$sketch/scripts/tests" -std=c++17 -O2 -DMATDOG_T2_SHADOW "$out/probe.cpp" "${src[@]}" -o "$out/bin"
