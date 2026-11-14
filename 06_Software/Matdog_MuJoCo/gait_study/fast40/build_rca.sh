#!/usr/bin/env bash
# Builds stop_rca.cpp against a firmware tree. Usage: build_rca.sh <firmware tree> <out binary>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; sk="$1"
src=("$sk/src/simple/SimpleMotion.cpp" "$sk/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sk/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sk" -I "$sk/scripts/tests/simple_host_stub" "$here/stop_rca.cpp" "${src[@]}" -o "$2"
