#!/usr/bin/env bash
# Core Motion V5: bounded servo UART transactions (production ServoBus + real SCServo, virtual-time UART).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LIB="${SCSERVO_DIR:-$HOME/Arduino/libraries/SCServo/src}"
OUT="$(mktemp -d /tmp/matdog-bus-deadline.XXXXXX)";trap 'rm -rf "$OUT"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  -DARDUINO=100 -Wno-unused-parameter -Wno-array-bounds -I "$SCRIPT_DIR/servo_timed_stub" -I "$LIB" \
  "$SCRIPT_DIR/test_servo_bus_deadline.cpp" "$SKETCH_DIR/src/servo/ServoBus.cpp" "$SKETCH_DIR/src/servo/ServoProfile.cpp" \
  "$SKETCH_DIR/src/servo/ServoBusActuatorBackend.cpp" \
  "$SKETCH_DIR/src/core/Availability.cpp" "$SKETCH_DIR/src/diagnostics/PrimeDiagnostic.cpp" \
  "$LIB/SCS.cpp" "$LIB/SCSerial.cpp" "$LIB/SMS_STS.cpp" -o "$OUT/test_servo_bus_deadline"
"$OUT/test_servo_bus_deadline"
