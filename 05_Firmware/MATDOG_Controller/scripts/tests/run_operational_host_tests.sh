#!/usr/bin/env bash
# Offline-only production Motion compilation; arguments are host test basenames.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
out="${MATDOG_TEST_OUTPUT:?set an isolated artifact directory}"
mkdir -p "$out"
sources=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for name in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do sources+=("$sketch/src/motion/$name.cpp"); done
for test in "$@"; do
 "${CXX:-g++}" -I "$here/simple_host_stub" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined -fno-omit-frame-pointer "$here/$test.cpp" "${sources[@]}" -o "$out/$test"
 "$out/$test" > "$out/$test.log"
 tail -3 "$out/$test.log"
done
