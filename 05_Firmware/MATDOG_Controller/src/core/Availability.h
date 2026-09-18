#ifndef MATDOG_CORE_AVAILABILITY_H
#define MATDOG_CORE_AVAILABILITY_H

#include <stdint.h>

#include "../config/HardwareProfile.h"
#include "SystemState.h"

// Deliberately <stdint.h>, not <Arduino.h> (G2): nothing in this header or
// its .cpp needs the Arduino runtime, and keeping it free of it lets the
// offline host test suite link the REAL classify()/expectedStateFor*()
// implementations instead of a second copy of the rules that could drift.
// See scripts/tests/test_servo_population.cpp.

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

// NOTE (G2): the distinction between UNKNOWN and NO_RESPONSE is
// load-bearing in classify(), not cosmetic. UNKNOWN means "nothing has
// established anything"; NO_RESPONSE means "we asked and it did not
// answer". A module that cannot be probed at all (WS2812) stays UNKNOWN
// forever and must never be reported as NO_RESPONSE to force a verdict,
// nor as ONLINE to make a status green.
enum class DetectedState : uint8_t {
  UNKNOWN     = 0,  // no probe attempted, or hardware inherently non-probeable
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

// G2: the profile -> ExpectedState rules, in ONE place.
//
// V0.1 inlined these as a `build::kXxxAvailable ? A : B` ternary inside
// each module's availability(). That worked, but it meant the rule "what
// does an unpowered rail imply for this module" was stated three times in
// three files, reachable only by reading each module, and testable only by
// compiling for the ESP32. These free functions take the rail facts as a
// parameter (rather than reading build::kProfileExpectations directly) so
// they stay pure and can be exercised offline against BOTH profiles in the
// same test run — which is what proves the USB_ONLY semantics did not
// regress while ROBOT_POWERED was added.
//
// The asymmetry between the three is intentional and load-bearing:
//   - servo bus / battery: powered means REQUIRED (a silent servo or BMS on
//     a live robot is a genuine FAULT), unpowered means EXPECTED_OFFLINE
//     (the V0.1 USB_ONLY semantics, unchanged).
//   - LED ring: powered means OPTIONAL, never REQUIRED. The ring is a
//     status indicator; the handoff is explicit that LED behaviour must
//     never be safety-critical, so a dead ring degrades and must not fault
//     a robot that is otherwise healthy.
ExpectedState expectedStateForServoBus(const config::ProfileExpectations& profile);
ExpectedState expectedStateForBattery(const config::ProfileExpectations& profile);
ExpectedState expectedStateForLedRail(const config::ProfileExpectations& profile);

// The LED ring is the one module whose DetectedState is profile-derived
// rather than probe-derived: there is no read-back path on a WS2812 chain,
// so "unpowered rail" is asserted from the profile, not observed.
DetectedState detectedStateForLedRail(const config::ProfileExpectations& profile);

// Bridges the new model to the existing SystemState aggregation so
// Controller/SystemState do not need to change shape for this hardening
// pass.
ModuleHealth toModuleHealth(Classification c);

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_AVAILABILITY_H
