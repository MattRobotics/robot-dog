#!/usr/bin/env bash
# Motion release cost probe (dev.15), host only. Counts libm transcendental and
# sqrt calls per Motion release with link-time wrappers (-fno-builtin keeps the
# calls). A soft-double cost proxy for the ESP32-S3, not a WCET measurement.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)";out="$(mktemp -d /tmp/matdog-cost.XXXXXX)";trap 'rm -rf "$out"' EXIT
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
w=();for f in sin cos sincos tan asin acos atan atan2 sqrt; do w+=("-Wl,--wrap=$f"); done
"${CC:-gcc}" -O2 -c "$here/opcount.c" -o "$out/opcount.o"
"${CXX:-g++}" -std=c++17 -O2 -fno-builtin -w -I "$sketch/scripts/tests/simple_host_stub" "$here/motion_cost_probe.cpp" "${src[@]}" "$out/opcount.o" "${w[@]}" -lm -o "$out/cost"
"$out/cost"
