#!/usr/bin/env bash
# Compiles and runs the offline C++ host test suite for the G2 servo
# population / hardware profile logic. No hardware, no Arduino toolchain,
# no device I/O — it links the real firmware translation units, which is
# why they were kept Arduino-free.
#
# Invoked by scripts/static_audit.py so there is one gate command, matching
# how the OTA partition parser's Python suite is already run.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "HOST_TESTS = FAIL (no C++ compiler found: CXX=$CXX)" >&2
  exit 1
fi

OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_servo_population" \
  "$SCRIPT_DIR/test_servo_population.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/Availability.cpp"

"$OUT/test_servo_population"
