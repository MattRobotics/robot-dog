#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
lib="${SCSERVO_DIR:-$HOME/Arduino/libraries/SCServo/src}"
out="$(mktemp -d /tmp/matdog-reference-guard.XXXXXX)"
trap 'rm -rf "$out"' EXIT
sources=("${MOTION_SOURCE:-$sketch/src/simple/SimpleMotion.cpp}" "$sketch/src/core/MotionService.cpp" "$sketch/src/motion/MotionContract.cpp"
 "$sketch/src/calibration/CalibrationDomain.cpp" "$sketch/src/servo/ServoBus.cpp" "$sketch/src/servo/ServoProfile.cpp" "$sketch/src/servo/ServoPopulation.cpp"
 "$sketch/src/core/Availability.cpp" "$sketch/src/diagnostics/PrimeDiagnostic.cpp" "$sketch/src/actuator/CalibrationTargetResolver.cpp" "$sketch/src/actuator/CalibrationGeometryProfile.cpp" "$sketch/src/actuator/ActuatorRuntime.cpp")
for name in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do sources+=("$sketch/src/motion/$name.cpp"); done
"${CXX:-g++}" -I "$here/servo_timed_stub" -I "$lib" -I "$sketch/src/simple" -std=c++17 -O1 -g -Wall -Wextra -Werror -Wno-misleading-indentation -Wno-unused-parameter -Wno-array-bounds -DARDUINO=100 -ffunction-sections -fdata-sections -Wl,--gc-sections -fsanitize=address,undefined -fno-omit-frame-pointer \
 "$here/test_reference_guard.cpp" "${sources[@]}" "$lib/SCS.cpp" "$lib/SCSerial.cpp" "$lib/SMS_STS.cpp" -o "$out/test"
"$out/test" "${REFERENCE_GUARD_TRACE:-$out/wire.txt}"
