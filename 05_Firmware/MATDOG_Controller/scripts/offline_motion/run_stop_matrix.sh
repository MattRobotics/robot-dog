#!/usr/bin/env bash
# STOP matrices under the servo_plant response model (vNext audit, 2026-11-11),
# host only, production SimpleMotion on the rig of test_simple_motion.cpp
# (stop_matrix.inc): a STOP at every release of the sway-transfer windows, or
# right at a joystick regime handover. Software only, not a hardware result.
# Usage: run_stop_matrix.sh window MODE SEEDS [BACK]
#        run_stop_matrix.sh handover MODE DIR SEEDS
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)";out="$(mktemp -d /tmp/matdog-stopmatrix.XXXXXX)";trap 'rm -rf "$out"' EXIT
python3 -I - "$sketch/scripts/tests/test_simple_motion.cpp" "$here/servo_plant.inc" "$here/stop_matrix.inc" "$out/probe.cpp" <<'PY'
import sys;src,inc,mx,dst=sys.argv[1:];t=open(src).read();i=t.index("int main(){")
main='''int main(int argc,char** argv){
  if(argc>=4&&!std::strcmp(argv[1],"window"))return stopmatrix::window(std::atoi(argv[2]),unsigned(std::atoi(argv[3])),argc>4&&std::atoi(argv[4]));
  if(argc>=5&&!std::strcmp(argv[1],"handover"))return stopmatrix::handover(std::atoi(argv[2]),std::atoi(argv[3]),unsigned(std::atoi(argv[4])));
  std::fprintf(stderr,"usage: window MODE SEEDS [BACK] | handover MODE DIR SEEDS\\n");return 2;}
'''
t='#include <deque>\n#include <random>\n#include <string>\n'+t[:i]+open(inc).read()+open(mx).read()+main+"static int unused_main(){"+t[i+len("int main(){"):]
open(dst,'w').write(t.replace('#include "../../src/','#include "SKETCH_SRC/'))
PY
sed -i "s#SKETCH_SRC/#$sketch/src/#g" "$out/probe.cpp"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -w -I "$sketch/scripts/tests/simple_host_stub" -I "$sketch/scripts/tests" -std=c++17 -O2 -DMATDOG_T2_SHADOW "$out/probe.cpp" "${src[@]}" -o "$out/matrix"
"$out/matrix" "$@"
