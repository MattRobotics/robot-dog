#!/usr/bin/env bash
# Issue #52: packet builder vs the installed SCServo library (host, no device).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB="${SCSERVO_DIR:-$HOME/Arduino/libraries/SCServo/src}"
OUT="$(mktemp -d)";trap 'rm -rf "$OUT"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -Wall -Wextra -Werror -Wno-unused-parameter -DARDUINO=100 -I "$SCRIPT_DIR/scservo_host_stub" -I "$LIB" \
  "$SCRIPT_DIR/test_servo_packet.cpp" "$LIB/SCS.cpp" "$LIB/SCSerial.cpp" "$LIB/SMS_STS.cpp" -o "$OUT/test_servo_packet"
"$OUT/test_servo_packet"
