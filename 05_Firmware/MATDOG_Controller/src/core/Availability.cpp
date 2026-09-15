#include "Availability.h"

namespace matdog {
namespace core {

const char* toString(InitializationState s) {
  switch (s) {
    case InitializationState::NOT_INITIALIZED: return "NOT_INIT";
    case InitializationState::INITIALIZED:     return "OK";
    case InitializationState::INIT_FAILED:     return "FAILED";
    case InitializationState::DEFERRED:        return "DEFERRED";
  }
  return "UNKNOWN";
}

const char* toString(DetectedState s) {
  switch (s) {
    case DetectedState::UNKNOWN:     return "UNKNOWN";
    case DetectedState::ONLINE:      return "ONLINE";
    case DetectedState::NO_RESPONSE: return "NO_RESPONSE";
    case DetectedState::UNPOWERED:   return "UNPOWERED";
  }
  return "UNKNOWN";
}

const char* toString(ExpectedState s) {
  switch (s) {
    case ExpectedState::REQUIRED:           return "REQUIRED";
    case ExpectedState::OPTIONAL:           return "OPTIONAL";
    case ExpectedState::EXPECTED_OFFLINE:   return "OFFLINE";
    case ExpectedState::EXPECTED_UNPOWERED: return "UNPOWERED";
  }
  return "UNKNOWN";
}

const char* toString(Classification c) {
  switch (c) {
    case Classification::PASS:     return "PASS";
    case Classification::DEGRADED: return "DEGRADED";
    case Classification::FAULT:    return "FAULT";
    case Classification::UNKNOWN:  return "UNKNOWN";
  }
  return "UNKNOWN";
}

Classification classify(const AvailabilityStatus& s) {
  // A driver-level failure is always worth surfacing strongly — it means
  // the software/transport itself broke, independent of whether the
  // hardware behind it was expected to be reachable.
  if (s.init == InitializationState::INIT_FAILED) return Classification::FAULT;

  // Nothing has been observed yet (e.g. before first begin()/probe).
  if (s.init == InitializationState::NOT_INITIALIZED) return Classification::UNKNOWN;

  // init is INITIALIZED or DEFERRED from here.
  if (s.detected == DetectedState::ONLINE) return Classification::PASS;

  // Not online (NO_RESPONSE / UNPOWERED / UNKNOWN): whether that is fine
  // depends entirely on what was expected under the current profile. This
  // is the one branch a future ROBOT_POWERED profile changes by flipping
  // ExpectedState values in the modules — not by touching this function.
  switch (s.expected) {
    case ExpectedState::REQUIRED:           return Classification::FAULT;
    case ExpectedState::OPTIONAL:           return Classification::DEGRADED;
    case ExpectedState::EXPECTED_OFFLINE:   return Classification::PASS;
    case ExpectedState::EXPECTED_UNPOWERED: return Classification::PASS;
  }
  return Classification::UNKNOWN;
}

ModuleHealth toModuleHealth(Classification c) {
  switch (c) {
    case Classification::PASS:     return ModuleHealth::OK;
    case Classification::DEGRADED: return ModuleHealth::DEGRADED;
    case Classification::FAULT:    return ModuleHealth::FAULT;
    case Classification::UNKNOWN:  return ModuleHealth::NOT_INITIALIZED;
  }
  return ModuleHealth::NOT_INITIALIZED;
}

}  // namespace core
}  // namespace matdog
