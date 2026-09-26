#ifndef MATDOG_CORE_SERVICE_READINESS_H
#define MATDOG_CORE_SERVICE_READINESS_H

#include <stdint.h>

// HostLink's readiness vocabulary — I6, implemented per the 2026-09-25
// objective-change instruction: a semantic layer may expose state/services
// for currently unavailable functions, but those functions must report
// BLOCKED / TO_TEST rather than silently disappear or become reachable.
//
// Deliberately <stdint.h> only: no <Arduino.h>, no module pointer, no
// hardware call — pure and total, so scripts/tests/test_service_readiness.cpp
// links the REAL classifier, the same contract as every other pure decision
// core in this codebase (WifiPolicy, OtaPolicy, SafeActuatorPolicy, ...).
//
// TO_TEST, not a separate "NOT_READY" label, deliberately: the root
// README's own status vocabulary already defines TO_TEST ("implemented or
// assembled enough for a defined validation step that has not yet
// passed") — introducing a second word for the same meaning would just be
// a second vocabulary to keep in sync with the first.

namespace matdog {
namespace core {

enum class ServiceReadiness : uint8_t {
  READY   = 0,  // reachable and expected to work now
  TO_TEST = 1,  // implemented, offline-tested; hardware validation pending
  BLOCKED = 2,  // structurally unreachable — a fail-closed gate is engaged
};

const char* toString(ServiceReadiness readiness);

// Named capabilities this layer can classify. Deliberately a small,
// reviewed enum — not a free-form string key — so a caller cannot invent a
// capability and receive a silently-wrong answer for one nobody reviewed.
enum class ServiceCapability : uint8_t {
  ACTUATOR_TORQUE_ENABLE       = 0,
  ACTUATOR_POSITION_COMMAND    = 1,
  CALIBRATION_CONTACT_PROBE    = 2,
  CALIBRATION_AUXILIARY_MOVE   = 3,
  CALIBRATION_DIRECTION_VERIFY = 4,
  WIFI_HARDWARE_ASSOCIATION    = 5,
  OTA_END_TO_END               = 6,
  WEB_READ_ONLY_DASHBOARD      = 7,
};

const char* toString(ServiceCapability capability);

// Every input this classification may depend on — a snapshot, never a
// pointer to anything live. Defaults are the CURRENT repository truth:
// motion blocked, nothing hardware-validated yet.
struct ServiceReadinessInputs {
  bool hardware_motion_authorized = false;  // MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED
  bool wifi_hardware_validated = false;     // no MATDOG build has ever associated with an AP
  bool ota_hardware_validated = false;      // no OTA image has ever been received on device
};

// Pure, total, deterministic. Never queries a module, never reads a global.
ServiceReadiness classifyCapability(ServiceCapability capability,
                                    const ServiceReadinessInputs& inputs);

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_SERVICE_READINESS_H
