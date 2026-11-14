#!/usr/bin/env bash
# FAST 40 study: build the firmware's own servo_plant matrix (scripts/offline_motion/run_servo_plant.sh,
# same probe, same sources, same flags) into <out>/plant so that single modes/scenarios/seeds can be rerun
# (MODE, SCENARIO, SEEDS, PLANT_TRACE env of servo_plant.inc). SWEEP_INC=<file>: append it and call
# plant::sweep() instead (mode_stop_sweep.inc: STOP swept through the WALK mode changes).
# Usage: build_plant.sh <firmware tree MATDOG_Controller> <out dir>
set -euo pipefail
sketch="$(cd "$1" && pwd)"; out="$2"; mkdir -p "$out"; here="$sketch/scripts/offline_motion"
python3 -I - "$sketch/scripts/tests/test_simple_motion.cpp" "$here/servo_plant.inc" "$out/probe.cpp" <<'PY'
import sys;src,inc,dst=sys.argv[1:];t=open(src).read();i=t.index("int main(){")
import os;extra=os.environ.get("SWEEP_INC");body=open(inc).read()+(open(extra).read() if extra else "");call="plant::sweep()" if extra else "plant::matrix()"
t='#include <deque>\n#include <random>\n#include <string>\n'+t[:i]+body+"int main(){"+call+";return 0;}\nstatic int unused_main(){"+t[i+len("int main(){"):]
open(dst,'w').write(t.replace('#include "../../src/','#include "SKETCH_SRC/'))
PY
sed -i "s#SKETCH_SRC/#$sketch/src/#g" "$out/probe.cpp"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -w -I "$sketch/scripts/tests/simple_host_stub" -I "$sketch/scripts/tests" -std=c++17 -O2 -DMATDOG_T2_SHADOW ${EXTRA_CXXFLAGS:-} "$out/probe.cpp" "${src[@]}" -o "$out/plant"
echo "built $out/plant"
