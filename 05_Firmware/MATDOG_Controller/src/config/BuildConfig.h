#ifndef MATDOG_CONFIG_BUILD_CONFIG_H
#define MATDOG_CONFIG_BUILD_CONFIG_H

#include <stdint.h>

#include "HardwareProfile.h"

// Injected by scripts/build.sh via --build-property compiler.cpp.extra_flags
// as the short git SHA (+ "-dirty" if the tree had uncommitted changes at
// build time). Falls back to "unknown" for an IDE build that skips the script.
#ifndef MATDOG_BUILD_ID
#define MATDOG_BUILD_ID "unknown"
#endif

// ---------------------------------------------------------------------------
// ACTIVE HARDWARE PROFILE — the single G3 authorization gate.
// ---------------------------------------------------------------------------
// The source default is USB_ONLY and must stay USB_ONLY until the powered
// no-motion hardware validation gate (G3) is explicitly authorized.
// scripts/static_audit.py FAILS the build if this default is anything else,
// so a ROBOT_POWERED image can never be produced by an unreviewed edit or
// by accident.
//
// Switching profiles is deliberately a ONE-symbol change — either this
// #define, or an explicit -DMATDOG_ACTIVE_HARDWARE_PROFILE=... build
// override for a G3 session. Nothing else in the firmware needs editing:
// every rail expectation below, and every module ExpectedState derived from
// them, follows automatically (config/HardwareProfile.h).
#ifndef MATDOG_ACTIVE_HARDWARE_PROFILE
#define MATDOG_ACTIVE_HARDWARE_PROFILE ::matdog::config::HardwareProfile::USB_ONLY
#endif

namespace matdog {
namespace build {

constexpr const char* kFirmwareName    = "MATDOG Controller";

// Release identity deliberately unchanged during the v02 development
// branch (G2 handoff section 34): the 0.2.x release number is decided at
// the release gate, not by a development branch name. Development builds
// are distinguished by kBuildId (git SHA), not by this string.
constexpr const char* kFirmwareVersion = "0.1.0";
constexpr const char* kBoardName       = "YD-ESP32-S3 N16R8";
constexpr const char* kBuildId         = MATDOG_BUILD_ID;

// The active profile and everything derived from it. kProfileExpectations
// is the value every module consults (via core::expectedStateFor*()) to
// build its ExpectedState — see core/Availability.h.
constexpr config::HardwareProfile kHardwareProfile = MATDOG_ACTIVE_HARDWARE_PROFILE;
constexpr config::ProfileExpectations kProfileExpectations =
    config::expectationsFor(kHardwareProfile);

// Profile name printed by the boot banner / @STATUS. Derived, so it can
// never contradict the rail facts below.
constexpr const char* kTestProfile = config::toString(kHardwareProfile);

// Per-rail availability under the active profile. These names are retained
// from V0.1 so no module changed shape, but they are now DERIVED values,
// not independently editable literals — flipping one in isolation (and
// thereby contradicting the other two and the profile name) is no longer
// expressible.
//
// kLedRailPowered additionally gates whether LedRing is allowed to drive
// GPIO47 at all (Session 2 hardening: anti-back-power). false means the
// NeoPixel transport is never initialized and no WS2812 frame is ever
// transmitted — see src/status/LedRing.cpp.
constexpr bool kServoPowerAvailable = kProfileExpectations.servo_power_available;
constexpr bool kBatteryAvailable    = kProfileExpectations.battery_available;
constexpr bool kLedRailPowered      = kProfileExpectations.led_rail_powered;

constexpr uint32_t kUsbSerialBaud = 115200;   // USB CDC ignores baud; kept for host-side tooling parity.
constexpr uint32_t kServoBusBaud  = 1000000;  // ST3215 / Seeed driver, hardware-proven.
constexpr uint32_t kDalyBusBaud   = 9600;     // DALY XY-017 RS485, hardware-proven.

// G3.1 USB CDC transmit policy, applied in Controller::begin(). Timeout 0
// makes HWCDC drop, instead of wait for, output no host is draining. The
// ring holds the largest single Controller::update() burst (worst census
// reply + DALY KEY write ACK and COMPLETE lines + IMU block + BMS block =
// 2674 B; 2395 B before the KEY probe), so replies are expected complete
// while a host drains the port - best effort, see VALIDATION.md § G3.1 and
// § DALY KEY.
// The core default is 256 B.
constexpr uint32_t kUsbTxTimeoutMs = 0;
constexpr uint32_t kUsbTxRingBytes = 3072;

}  // namespace build
}  // namespace matdog

#endif  // MATDOG_CONFIG_BUILD_CONFIG_H
