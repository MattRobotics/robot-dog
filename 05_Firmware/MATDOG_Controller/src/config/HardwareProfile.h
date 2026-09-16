#ifndef MATDOG_CONFIG_HARDWARE_PROFILE_H
#define MATDOG_CONFIG_HARDWARE_PROFILE_H

#include <stdint.h>

// THE single authority for "which physical power/hardware configuration is
// this firmware built for". G2 (ROBOT_POWERED configuration support).
//
// Why this file exists
// --------------------
// V0.1 expressed the bench configuration as three independent `constexpr
// bool` literals in BuildConfig.h (kServoPowerAvailable / kBatteryAvailable
// / kLedRailPowered) plus an unrelated kTestProfile *string*. Nothing tied
// the four together: a future ROBOT_POWERED change had to remember to flip
// all four consistently, and any partial edit produced a firmware whose
// printed profile name contradicted its own expectations. That is exactly
// the "several unrelated `true` values" the G2 handoff forbids.
//
// Here there is one enum, one mapping table, and everything else is
// DERIVED. BuildConfig.h picks a profile; it no longer stores rail facts.
//
// Deliberately NOT a generic configuration framework (handoff section 14:
// "Do not overengineer"). Two profiles, three rail facts, one constexpr
// function. It is intentionally Arduino-free (<stdint.h> only) so the
// offline host test suite can link it directly — see
// scripts/tests/test_servo_population.cpp.
//
// Note on scope: a profile describes the POWER/RAIL configuration. It does
// NOT describe which servos are physically installed — that is a separate,
// orthogonal fact owned by src/servo/ServoPopulation.h (canonical 17 vs
// expected-now 13). A ROBOT_POWERED robot with a different installed servo
// set is a valid combination, so the two must not be merged.

namespace matdog {
namespace config {

enum class HardwareProfile : uint8_t {
  // ESP32-S3 powered solely from the host USB link. The DALY-protected
  // robot domain, the servo power branch and the LED 5 V rail are all
  // unpowered BY DESIGN. Hardware-validated configuration for V0.1.
  USB_ONLY = 0,

  // Full robot protected domain live: battery -> DALY -> P- -> step-down.
  // DALY reachable, ST3215 bus powered, LED 5 V rail powered. Prepared in
  // G2; NOT hardware-validated until G3 authorizes powered validation.
  ROBOT_POWERED = 1,
};

// The rail facts a profile asserts. Every module ExpectedState in the
// firmware derives from exactly these three booleans (see
// core/Availability.h expectedStateFor*()).
//
// BNO085 is deliberately absent: it is fed from the ESP32's own 3V3 rail,
// so it is REQUIRED under every profile and has no profile-dependent
// expectation to express.
struct ProfileExpectations {
  bool servo_power_available;  // ST3215 bus powered -> servos must answer
  bool battery_available;      // DALY reachable -> BMS telemetry must arrive
  bool led_rail_powered;       // WS2812 5 V present -> LED may be driven
};

// The one mapping table. A profile that does not power a rail must never
// be written as powering it, and vice versa — scripts/static_audit.py
// re-derives both rows from this text and fails on any drift.
constexpr ProfileExpectations expectationsFor(HardwareProfile profile) {
  return (profile == HardwareProfile::ROBOT_POWERED)
             ? ProfileExpectations{true, true, true}
             : ProfileExpectations{false, false, false};
}

constexpr const char* toString(HardwareProfile profile) {
  return (profile == HardwareProfile::ROBOT_POWERED) ? "ROBOT_POWERED" : "USB_ONLY";
}

}  // namespace config
}  // namespace matdog

#endif  // MATDOG_CONFIG_HARDWARE_PROFILE_H
