#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
lib="${SCSERVO_DIR:-$HOME/Arduino/libraries/SCServo/src}"
out="$(mktemp -d /tmp/matdog-preenable.XXXXXX)";trap 'rm -rf "$out"' EXIT
sources=("$sketch/src/calibration/CalibrationDomain.cpp" "$sketch/src/power/DalyBms.cpp" "$sketch/src/power/DalyProtocol.cpp" "${MOTION_SOURCE:-$sketch/src/simple/SimpleMotion.cpp}" "$sketch/src/servo/ServoBus.cpp" "$sketch/src/servo/ServoProfile.cpp" "$sketch/src/servo/ServoPopulation.cpp" "$sketch/src/core/Availability.cpp" "$sketch/src/diagnostics/PrimeDiagnostic.cpp" "$sketch/src/actuator/CalibrationTargetResolver.cpp" "$sketch/src/actuator/CalibrationGeometryProfile.cpp" "$sketch/src/actuator/ActuatorRuntime.cpp")
for name in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do sources+=("$sketch/src/motion/$name.cpp"); done
# The production Controller::SimpleServoPort::frame, verbatim (the harness binds it to the real ServoBus).
python3 -I - "$sketch/src/core/SimpleMotionIntegration.cpp" "$out/production_simple_servo_port_frame.inc" <<'PY'
import sys;src,dst=sys.argv[1:];t=open(src).read();s="bool Controller::SimpleServoPort::frame(";e="bool Controller::SimpleServoPort::enable("
assert t.count(s)==1 and t.count(e)==1;open(dst,"w").write(s+t.split(s,1)[1].split(e,1)[0])
PY
"${CXX:-g++}" -I "$out" -I "$here/servo_timed_stub" -I "$lib" -I "$sketch/src/simple" -std=c++17 -O1 -g -Wall -Wextra -Werror -Wno-misleading-indentation -Wno-unused-parameter -Wno-array-bounds -DARDUINO=100 -ffunction-sections -fdata-sections -Wl,--gc-sections -fsanitize=address,undefined -fno-omit-frame-pointer "${PREENABLE_TEST_SOURCE:-$here/test_preenable_bus.cpp}" "${sources[@]}" "$lib/SCS.cpp" "$lib/SCSerial.cpp" "$lib/SMS_STS.cpp" -o "$out/test"
"$out/test"
