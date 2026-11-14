#!/usr/bin/env bash
# Builds swing_probe.cpp against a firmware tree (default: MATDOG_FW_TREE, else
# this checkout's 05_Firmware/MATDOG_Controller). Usage: build_probe.sh <out binary>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sk="${MATDOG_FW_TREE:-$here/../../../../05_Firmware/MATDOG_Controller}"
src=("$sk/src/simple/SimpleMotion.cpp" "$sk/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sk/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sk" -I "$sk/scripts/tests/simple_host_stub" "$here/swing_probe.cpp" "${src[@]}" -o "$1"
