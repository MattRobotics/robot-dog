#!/usr/bin/env bash
# Builds walk_tilt_envelope_check.cpp on the production sources and runs it.
# Usage: run_walk_tilt_envelope_check.sh [--stride 45|60] [--start c] [--stop c] [--period s]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)";out="$(mktemp -d /tmp/matdog-tilt.XXXXXX)";trap 'rm -rf "$out"' EXIT
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sketch/scripts/tests/simple_host_stub" "$here/walk_tilt_envelope_check.cpp" "${src[@]}" -o "$out/check"
"$out/check" "$@"
