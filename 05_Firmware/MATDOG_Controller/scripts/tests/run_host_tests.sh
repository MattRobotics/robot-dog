#!/usr/bin/env bash
# Compiles and runs the offline C++ host test suites: the G2 servo
# population / hardware profile logic, the DALY wire protocol / KEY
# probe (frames, CRC, decoders, bus scheduling), and the W1 Wi-Fi runtime
# policy (credential gate, two-phase radio start, connect deadline, backoff
# ladder, link loss, wraparound). No hardware, no Arduino toolchain, no
# device I/O — they link the real firmware translation units, which is why
# those were kept Arduino-free.
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
  "$SKETCH_DIR/src/core/Availability.cpp" \
  "$SKETCH_DIR/src/core/SystemState.cpp"

# -DDISABLED=0x00 reproduces the Arduino-ESP32 core macro (esp32-hal-gpio.h)
# so an identifier clash with it fails here, not only in the device build.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_daly_protocol" \
  "$SCRIPT_DIR/test_daly_protocol.cpp" \
  "$SKETCH_DIR/src/power/DalyProtocol.cpp"

# -DDISABLED=0x00 here too: WifiPolicy.h documents why its first state is
# INACTIVE and not DISABLED, and this flag makes that reasoning enforceable
# on the host instead of only discoverable on the device build.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_wifi_policy" \
  "$SCRIPT_DIR/test_wifi_policy.cpp" \
  "$SKETCH_DIR/src/network/WifiPolicy.cpp"

"$OUT/test_servo_population"
"$OUT/test_daly_protocol"
"$OUT/test_wifi_policy"
