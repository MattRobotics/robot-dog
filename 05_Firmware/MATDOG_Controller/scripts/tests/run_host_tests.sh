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

# The Safe Actuator runtime adapter suite links the REAL adapter, the REAL
# policy and the REAL arbiter, so the "no ACCEPT -> no backend call" and
# "ACCEPT -> exactly one backend call" properties under test are the shipped
# ones. Only the backend is fake - the one thing this adapter is meant to be
# tested against, the same contract as the OTA suite's fake OtaBackend.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_actuator_runtime" \
  "$SCRIPT_DIR/test_actuator_runtime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The Calibration Execution boundary suite links the REAL engine, the REAL
# policy, the REAL runtime adapter and the REAL arbiter against a fake
# backend, the same contract as test_actuator_runtime.cpp. LF V25's 18-phase
# sequence is never linked here (V3 handoff Sec 15.11) - only the current
# generated geometry profile data.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_execution_engine" \
  "$SCRIPT_DIR/test_calibration_execution_engine.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationExecutionEngine.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The HostLink readiness classifier suite links the REAL pure classifier -
# no module pointer, no hardware call - I6 (2026-09-25 objective change).
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_service_readiness" \
  "$SCRIPT_DIR/test_service_readiness.cpp" \
  "$SKETCH_DIR/src/core/ServiceReadiness.cpp"

# The LED status suite links the REAL decision core; LedStatusPolicy.* has
# no <Arduino.h> and no LedRing dependency, so this drives the whole
# priority/effect table from a synthetic clock, the same contract as the
# Wi-Fi and OTA policy suites above.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_led_status_policy" \
  "$SCRIPT_DIR/test_led_status_policy.cpp" \
  "$SKETCH_DIR/src/status/LedStatusPolicy.cpp"

# Link the actual driver, manager and policy under both hardware profiles.
# Only the Arduino/NeoPixel transport is replaced with host observations.
for LED_PROFILE in USB_ONLY ROBOT_POWERED; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
    "-DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::$LED_PROFILE" \
    -I"$SCRIPT_DIR/led_stubs" \
    -o "$OUT/test_led_ring_manager_$LED_PROFILE" \
    "$SCRIPT_DIR/test_led_ring_manager.cpp" \
    "$SKETCH_DIR/src/status/LedRing.cpp" \
    "$SKETCH_DIR/src/status/LedStatusManager.cpp" \
    "$SKETCH_DIR/src/status/LedStatusPolicy.cpp" \
    "$SKETCH_DIR/src/core/Availability.cpp"
done

# HMAC-SHA256 (I7, 2026-09-25 correction), verified against the RFC 4231
# vectors, not self-consistency only. Links the REAL Sha256 it is built on.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_hmac256" \
  "$SCRIPT_DIR/test_hmac256.cpp" \
  "$SKETCH_DIR/src/update/Hmac256.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp"

# The OTA transport's authentication/session layer (I7). Links the REAL
# Hmac256/Sha256 it is built on. OtaPolicy.h is included only for the
# OtaImageMetadata type, so OtaPolicy.cpp is deliberately not linked here.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_ota_session" \
  "$SCRIPT_DIR/test_ota_session.cpp" \
  "$SKETCH_DIR/src/update/OtaSession.cpp" \
  "$SKETCH_DIR/src/update/Hmac256.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp"

# HttpTransport's single-slot mailbox correlation logic (I7/I8 hardening,
# 2026-09-25). Header-only and pure: no .cpp to link.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_http_mailbox" \
  "$SCRIPT_DIR/test_http_mailbox.cpp"

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
"$OUT/test_actuator_runtime"
"$OUT/test_calibration_execution_engine"
"$OUT/test_service_readiness"
"$OUT/test_led_status_policy"
"$OUT/test_led_ring_manager_USB_ONLY"
"$OUT/test_led_ring_manager_ROBOT_POWERED"
"$OUT/test_hmac256"
"$OUT/test_ota_session"
"$OUT/test_http_mailbox"
