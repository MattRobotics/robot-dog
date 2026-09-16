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

  // G2 pre-G3 hardening (review Finding 2, and a second case found while
  // evaluating it across both profiles).
  //
  // DetectedState::UNKNOWN means "no probe has established anything" — it
  // is the ABSENCE OF EVIDENCE, not evidence of absence. V0.1 collapsed it
  // together with NO_RESPONSE/UNPOWERED and manufactured a verdict from a
  // probe that never happened. Under USB_ONLY that was harmless (both
  // non-REQUIRED cases return PASS either way), so it never showed up; under
  // ROBOT_POWERED it produced two wrong answers:
  //
  //   LED, OPTIONAL + UNKNOWN -> DEGRADED
  //     A WS2812 chain has no readback path at all, so its DetectedState is
  //     permanently UNKNOWN when powered. Calling that DEGRADED made
  //     SystemHealth::READY unreachable on a perfectly healthy robot.
  //
  //   ServoBus, REQUIRED + UNKNOWN -> FAULT
  //     Nothing probes the bus at boot (no automatic scan/census is allowed),
  //     so a freshly booted powered robot reported SystemHealth::FAULT before
  //     anyone had asked it a single question.
  //
  // Both are fixed by the same rule rather than by two special cases, and
  // neither fix invents a physical observation: the module still reports
  // detected=UNKNOWN honestly, and @STATUS still shows it.
  if (s.detected == DetectedState::UNKNOWN) {
    switch (s.expected) {
      // "Must be reachable, but nothing has looked yet." Not a fault (no
      // failure was observed) and not a pass (nothing was proven). UNKNOWN
      // maps to ModuleHealth::NOT_INITIALIZED, so the system reports
      // BOOTING rather than READY until a real probe happens — the gap
      // stays visible and cannot be mistaken for health.
      case ExpectedState::REQUIRED:           return Classification::UNKNOWN;
      // An OPTIONAL module's absence would not even be a fault, so "not yet
      // observed" cannot be a problem. This is what a non-probeable status
      // device (the LED ring) reports for its entire powered lifetime.
      case ExpectedState::OPTIONAL:           return Classification::PASS;
      case ExpectedState::EXPECTED_OFFLINE:   return Classification::PASS;
      case ExpectedState::EXPECTED_UNPOWERED: return Classification::PASS;
    }
    return Classification::UNKNOWN;
  }

  // NO_RESPONSE or UNPOWERED: absence was actually observed or is asserted
  // by the profile. Unchanged from V0.1 — a real failure still escalates,
  // and ExpectedState::OPTIONAL still means "absence degrades but never
  // faults".
  switch (s.expected) {
    case ExpectedState::REQUIRED:           return Classification::FAULT;
    case ExpectedState::OPTIONAL:           return Classification::DEGRADED;
    case ExpectedState::EXPECTED_OFFLINE:   return Classification::PASS;
    case ExpectedState::EXPECTED_UNPOWERED: return Classification::PASS;
  }
  return Classification::UNKNOWN;
}

ExpectedState expectedStateForServoBus(const config::ProfileExpectations& profile) {
  return profile.servo_power_available ? ExpectedState::REQUIRED
                                       : ExpectedState::EXPECTED_OFFLINE;
}

ExpectedState expectedStateForBattery(const config::ProfileExpectations& profile) {
  return profile.battery_available ? ExpectedState::REQUIRED
                                   : ExpectedState::EXPECTED_OFFLINE;
}

ExpectedState expectedStateForLedRail(const config::ProfileExpectations& profile) {
  // OPTIONAL, never REQUIRED — see the header comment: LED state must not
  // be able to fault an otherwise healthy robot.
  return profile.led_rail_powered ? ExpectedState::OPTIONAL
                                  : ExpectedState::EXPECTED_UNPOWERED;
}

DetectedState detectedStateForLedRail(const config::ProfileExpectations& profile) {
  // UNKNOWN when powered: a WS2812 chain cannot be probed, so the honest
  // answer is "not observed", not "ONLINE".
  return profile.led_rail_powered ? DetectedState::UNKNOWN : DetectedState::UNPOWERED;
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
