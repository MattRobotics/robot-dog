#!/usr/bin/env bash
# Servo plant matrix (walk modes 2026-11-09), host only: production SimpleMotion
# on the rig of test_simple_motion.cpp, each servo replaced by the ST3215 response
# model of servo_plant.inc (profile, capability, latency, deadband, sag; ranges
# drawn per seed), for SLOW, STANDARD and FAST: long walk, 8 STOP + resume,
# IMU at the pitch-down vertex, mode changes during the walk with STOPs inside
# them (2026-11-10); vNext joystick (2026-11-11, scenarios 4-7): steered and
# backward sequences with STOPs, the steered IMU vertex, selections during backward walks;
# 11.1 and 9.6 V. Controller outcome against a
# servo that lags; software only, not a hardware result.
# Usage: SEEDS=6 run_servo_plant.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)";out="$(mktemp -d /tmp/matdog-plant.XXXXXX)";trap 'rm -rf "$out"' EXIT
python3 -I - "$sketch/scripts/tests/test_simple_motion.cpp" "$here/servo_plant.inc" "$out/probe.cpp" <<'PY'
import sys;src,inc,dst=sys.argv[1:];t=open(src).read();i=t.index("int main(){")
t='#include <deque>\n#include <random>\n#include <string>\n'+t[:i]+open(inc).read()+"int main(){plant::matrix();return 0;}\nstatic int unused_main(){"+t[i+len("int main(){"):]
open(dst,'w').write(t.replace('#include "../../src/','#include "SKETCH_SRC/'))
PY
sed -i "s#SKETCH_SRC/#$sketch/src/#g" "$out/probe.cpp"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -w -I "$sketch/scripts/tests/simple_host_stub" -I "$sketch/scripts/tests" -std=c++17 -O2 -DMATDOG_T2_SHADOW "$out/probe.cpp" "${src[@]}" -o "$out/plant"
"$out/plant"
