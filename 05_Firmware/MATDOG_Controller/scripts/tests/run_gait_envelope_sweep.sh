#!/usr/bin/env bash
# Issue #52: production-IK gait envelope sweep (host only). Usage: run_gait_envelope_sweep.sh [stride_mm,T,lift_mm,duty]
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)";S="$D/../../src/motion";OUT="$(mktemp -d)";trap 'rm -rf "$OUT"' EXIT
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -DDISABLED=0x00 -o "$OUT/sweep" "$D/gait_envelope_sweep.cpp" \
  "$S/Gait.cpp" "$S/Locomotion.cpp" "$S/BodyPose.cpp" "$S/PoseSupport.cpp" "$S/LegKinematics.cpp" "$S/LegInverseKinematics.cpp" \
  "$S/MotionState.cpp" "$S/FootContact.cpp" "$S/ContactMode.cpp" "$S/TiltedContactIk.cpp" "$S/StandTrajectory.cpp" "$S/StandTransition.cpp" "$S/TimedStand.cpp" "$S/StartupAcquisition.cpp" "$S/MotionContract.cpp"
"$OUT/sweep" "$@"
