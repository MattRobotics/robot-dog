#!/usr/bin/env bash
# Core Motion V5: IMU acquisition core (ASan/UBSan) and single-writer snapshot (TSan). Host only.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$(mktemp -d /tmp/matdog-imu-feed.XXXXXX)";trap 'rm -rf "$OUT"' EXIT
CXX="${CXX:-g++}"
"$CXX" -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -pthread -o "$OUT/imu_asan" "$SCRIPT_DIR/test_imu_feed.cpp"
"$OUT/imu_asan"
"$CXX" -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=thread -pthread -DIMU_FEED_TSAN -o "$OUT/imu_tsan" "$SCRIPT_DIR/test_imu_feed.cpp"
# TSan needs a non-randomized address space on recent kernels.
setarch "$(uname -m)" -R "$OUT/imu_tsan" | grep -E "IMU_SEQLOCK|IMU_FEED"
