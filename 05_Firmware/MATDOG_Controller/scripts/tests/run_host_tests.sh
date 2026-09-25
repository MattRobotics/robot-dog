#!/usr/bin/env bash
# Compiles and runs the offline C++ host test suites: the G2 servo
# population / hardware profile logic, the DALY wire protocol / KEY
# probe (frames, CRC, decoders, bus scheduling), and the W1 Wi-Fi runtime
# policy (credential gate, two-phase radio start, connect deadline, backoff
# ladder, link loss, wraparound). No hardware, no Arduino toolchain, no
# device I/O — they link the real firmware translation units, which is why
# those were kept Arduino-free. OTA-A adds a fourth: the update state
# machine, the first-boot rollback guard and the streaming SHA-256, driven
# against a fake OtaBackend so every flash-failure path is reachable offline.
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

# The MATDOG_C018_V1 contract links the REAL generated register table, so the
# twenty values under test are the ones the firmware carries. The preflight
# SERVICE is device-only (it holds a ServoBus); what is host-testable is the
# contract, the comparison semantics and the leg selection.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_servo_profile" \
  "$SCRIPT_DIR/test_servo_profile.cpp" \
  "$SKETCH_DIR/src/servo/ServoProfile.cpp" \
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

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_domain" \
  "$SCRIPT_DIR/test_calibration_domain.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"

# The calibration manager links the REAL arbiter, so the authority
# integration it exercises is the shipped one rather than a mock.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_manager" \
  "$SCRIPT_DIR/test_calibration_manager.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationManager.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_actuator_authority" \
  "$SCRIPT_DIR/test_actuator_authority.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The Safe Actuator Layer policy links the REAL arbiter and the REAL
# calibration domain model: the authority binding and the limit-provenance
# rules it enforces are the shipped ones, not a mock's idea of them.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_actuator_write_policy" \
  "$SCRIPT_DIR/test_actuator_write_policy.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The calibration bootstrap geometry suite links the REAL generated profile
# table, so the 24 endpoints, 6 parking plans and per-joint envelopes it
# checks are the exact ones the firmware would carry - reduced from the
# canonical Geometry Compiler V5 bundle, never retyped.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_geometry" \
  "$SCRIPT_DIR/test_calibration_geometry.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The OTA suite links the REAL OTA-B gate and the REAL arbiter, so the
# authorization path it exercises is the shipped one, not a stub.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_ota_policy" \
  "$SCRIPT_DIR/test_ota_policy.cpp" \
  "$SKETCH_DIR/src/update/OtaPolicy.cpp" \
  "$SKETCH_DIR/src/update/OtaBootGuard.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp" \
  "$SKETCH_DIR/src/update/OtaAuthorityGate.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The LED status suite links the REAL decision core; LedStatusPolicy.* has
# no <Arduino.h> and no LedRing dependency, so this drives the whole
# priority/effect table from a synthetic clock, the same contract as the
# Wi-Fi and OTA policy suites above.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_led_status_policy" \
  "$SCRIPT_DIR/test_led_status_policy.cpp" \
  "$SKETCH_DIR/src/status/LedStatusPolicy.cpp"

"$OUT/test_servo_population"
"$OUT/test_servo_profile"
"$OUT/test_daly_protocol"
"$OUT/test_wifi_policy"
"$OUT/test_actuator_authority"
"$OUT/test_actuator_write_policy"
"$OUT/test_calibration_geometry"
"$OUT/test_ota_policy"
"$OUT/test_calibration_domain"
"$OUT/test_calibration_manager"
"$OUT/test_led_status_policy"
