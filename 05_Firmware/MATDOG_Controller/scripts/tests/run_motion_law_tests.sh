#!/usr/bin/env bash
# Core Motion V5: common time law (S-curve) and dynamic-profile analysis. Host only.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT="$(mktemp -d /tmp/matdog-motion-law.XXXXXX)";trap 'rm -rf "$OUT"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
  -o "$OUT/test_motion_law" "$SCRIPT_DIR/test_motion_law.cpp" "$SKETCH_DIR/src/motion/MotionLaw.cpp"
"$OUT/test_motion_law"
