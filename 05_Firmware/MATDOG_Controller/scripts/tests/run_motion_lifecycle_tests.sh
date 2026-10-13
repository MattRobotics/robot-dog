#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT="$(mktemp -d /tmp/matdog-motion-lifecycle.XXXXXX)"
trap 'rm -rf "$OUT"' EXIT
CXX="${CXX:-g++}"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -o "$OUT/test_motion_lifecycle" "$SCRIPT_DIR/test_motion_lifecycle.cpp" \
  "$SKETCH_DIR/src/core/MotionService.cpp" "$SKETCH_DIR/src/motion/MotionContract.cpp" \
  "$SKETCH_DIR/src/execution/MotionExecutor.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" "$SKETCH_DIR/src/core/OperatingMode.cpp"
"$OUT/test_motion_lifecycle"
