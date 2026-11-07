#!/usr/bin/env bash
# PRONE host suite (offline only): dev.14 hardware HOLD replay (69 cases) and
# the dev.15 hardware handover replay (2 timeouts, 2 rests).
# Not built: test_prone_descent (State::PRONE), test_prone_admission,
# test_prone_height_residuals and test_prone_temperature (PRONE_CONTACT_UNVERIFIED)
# predate dev.15 and fail identically on the installed dev.15 source 7997ad4;
# dev.15+ PRONE behaviour is covered by test_simple_motion.cpp and these two.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
out="$(mktemp -d /tmp/matdog-prone.XXXXXX)"
trap 'rm -rf "$out"' EXIT
sources=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for name in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do sources+=("$sketch/src/motion/$name.cpp"); done
for t in "${@:-test_prone_hardware_replay test_prone_dev15_replay}"; do
  for name in $t; do
    "${CXX:-g++}" -I "$here/simple_host_stub" -I "$here" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined -fno-omit-frame-pointer "$here/$name.cpp" "${sources[@]}" -o "$out/$name"
    echo "== $name"; "$out/$name" | grep -v '^REAL_\|^REPLAY_CASE' | tail -12
  done
done
