#ifndef MATDOG_CORE_AVAILABILITY_H
#define MATDOG_CORE_AVAILABILITY_H

#include <Arduino.h>

#include "SystemState.h"

namespace matdog {
namespace core {

// Separates four questions that Session 1 conflated into one ModuleHealth
// value (a module reporting `OK` merely because its driver had been
// initialized, regardless of whether the physical hardware behind it was
// ever actually detected):
//
//   1. Did the software driver initialize?             -> InitializationState
//   2. What did the last probe/comm attempt observe?    -> DetectedState
//   3. Is this hardware supposed to be reachable right
//      now, given the current bench/power profile?      -> ExpectedState
//   4. Given all three, is that a PASS or a problem?     -> Classification (derived)
//
// classify() is the one place profile-dependent judgement happens. A future
// ROBOT_POWERED profile changes module ExpectedState values (e.g. DALY
// becomes REQUIRED instead of EXPECTED_OFFLINE) and the SAME classify()
// turns an unchanged NO_RESPONSE into FAULT/DEGRADED automatically — no
// module or aggregation logic needs to be rewritten.

enum class InitializationState : uint8_t {
  NOT_INITIALIZED = 0,  // begin() not called yet
  INITIALIZED     = 1,  // driver/transport set up successfully
  INIT_FAILED     = 2,  // begin() ran and failed
  DEFERRED        = 3,  // begin() deliberately did not touch hardware
                         // (e.g. LED transport withheld under USB_ONLY to
                         // avoid driving an unpowered rail)
};

enum class DetectedState : uint8_t {
  UNKNOWN     = 0,  // no probe attempted yet
  ONLINE      = 1,  // last probe got a valid response
  NO_RESPONSE = 2,  // last probe timed out / got no or invalid response
  UNPOWERED   = 3,  // known-absent by profile, not by a failed probe
                     // (used only where probing would itself be unsafe,
                     // e.g. LED under USB_ONLY — see LedRing)
};

enum class ExpectedState : uint8_t {
  REQUIRED          = 0,  // must be ONLINE for the system to be nominal
  OPTIONAL          = 1,  // absence degrades but never faults
  EXPECTED_OFFLINE  = 2,  // absence under the current profile is normal
  EXPECTED_UNPOWERED = 3, // rail is known absent under the current profile
};

enum class Classification : uint8_t {
  PASS      = 0,
  DEGRADED  = 1,
  FAULT     = 2,
  UNKNOWN   = 3,
};

struct AvailabilityStatus {
  InitializationState init = InitializationState::NOT_INITIALIZED;
  DetectedState detected = DetectedState::UNKNOWN;
  ExpectedState expected = ExpectedState::REQUIRED;
};

const char* toString(InitializationState s);
const char* toString(DetectedState s);
const char* toString(ExpectedState s);
const char* toString(Classification c);

// Pure function: the only place "what does this combination mean" is
// decided. See the header comment above for why this is deliberately
// separate from the modules themselves.
Classification classify(const AvailabilityStatus& status);

// Bridges the new model to the existing SystemState aggregation so
// Controller/SystemState do not need to change shape for this hardening
// pass.
ModuleHealth toModuleHealth(Classification c);

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_AVAILABILITY_H
