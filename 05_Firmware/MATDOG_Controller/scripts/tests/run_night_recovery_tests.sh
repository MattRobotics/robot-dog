#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
out="$(mktemp -d /tmp/matdog-night.XXXXXX)"
trap 'rm -rf "$out"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
 -I "$here/daly_async_stub" "$here/test_daly_async.cpp" "$sketch/src/power/DalyBms.cpp" \
 "$sketch/src/power/DalyProtocol.cpp" "$sketch/src/core/Availability.cpp" -o "$out/daly"
"$out/daly"
bash "$here/run_simple_motion_tests.sh"
bash "$here/run_servo_bus_deadline_tests.sh"
bash "$here/run_preenable_bus_tests.sh"
PREENABLE_TEST_SOURCE="$here/test_night_recovery.cpp" bash "$here/run_preenable_bus_tests.sh"
bash "$here/run_servo_packet_tests.sh"
bash "$here/run_imu_feed_tests.sh"
