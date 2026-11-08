#!/usr/bin/env bash
# Builds walk_support_check.cpp on the production sources and runs it.
# Usage: run_walk_support_check.sh [--stride 0|1|2] [what-if flags, see the .cpp]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)";out="$(mktemp -d /tmp/matdog-support.XXXXXX)";trap 'rm -rf "$out"' EXIT
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sketch/scripts/tests/simple_host_stub" "$here/walk_support_check.cpp" "${src[@]}" -o "$out/check"
"$out/check" "$@"
