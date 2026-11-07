#!/usr/bin/env bash
# Builds motion_frame_digest.cpp on the production sources of this tree (or of
# MATDOG_SKETCH) and runs it. Usage: run_motion_frame_digest.sh [--dump file]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="${MATDOG_SKETCH:-$(cd "$here/../.." && pwd)}";out="$(mktemp -d /tmp/matdog-digest.XXXXXX)";trap 'rm -rf "$out"' EXIT
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -w ${MATDOG_DIGEST_FLAGS:-} -I "$sketch/scripts/tests/simple_host_stub" -I "$sketch/scripts/tests" "$here/motion_frame_digest.cpp" "${src[@]}" -o "$out/digest"
"$out/digest" "$@"
