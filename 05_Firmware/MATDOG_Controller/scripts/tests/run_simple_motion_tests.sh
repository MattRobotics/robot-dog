#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
out="$(mktemp -d /tmp/matdog-simple.XXXXXX)"
trap 'rm -rf "$out"' EXIT
sources=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for name in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do sources+=("$sketch/src/motion/$name.cpp"); done
test_source=test_simple_motion.cpp
if [[ "${1:-}" == "--ground-contact" ]]; then test_source=test_ground_contact.cpp; fi
"${CXX:-g++}" -I "$here/simple_host_stub" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -DMATDOG_T2_SHADOW -fsanitize=address,undefined -fno-omit-frame-pointer "$here/$test_source" "${sources[@]}" -o "$out/simple"
"$out/simple"
