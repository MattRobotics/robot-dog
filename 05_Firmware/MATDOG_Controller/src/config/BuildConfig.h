#ifndef MATDOG_CONFIG_BUILD_CONFIG_H
#define MATDOG_CONFIG_BUILD_CONFIG_H

// Injected by scripts/build.sh via --build-property compiler.cpp.extra_flags
// as the short git SHA (+ "-dirty" if the tree had uncommitted changes at
// build time). Falls back to "unknown" for an IDE build that skips the script.
#ifndef MATDOG_BUILD_ID
#define MATDOG_BUILD_ID "unknown"
#endif

namespace matdog {
namespace build {

constexpr const char* kFirmwareName    = "MATDOG Controller";
constexpr const char* kFirmwareVersion = "0.1.0";
constexpr const char* kBoardName       = "YD-ESP32-S3 N16R8";
constexpr const char* kBuildId         = MATDOG_BUILD_ID;

// Bench test profile currently active. USB_ONLY means the ESP32-S3 is
// powered solely from the ASUS USB link: DALY, LED 5V and ST3215 servo
// power are all absent by design. This is a documented bench profile, not
// a permanent classification shortcut — a future ROBOT_POWERED profile
// will classify the same absent-response conditions differently.
constexpr const char* kTestProfile = "USB_ONLY";

// Per-rail power availability under the current profile. These are the
// single source of truth modules consult to build their ExpectedState
// (core::Availability.h) — flipping these three flags is the entire change
// needed to move a module's expectations from USB_ONLY to a future
// ROBOT_POWERED profile; no module logic changes shape.
//
// kLedRailPowered additionally gates whether LedRing is allowed to drive
// GPIO47 at all (Session 2 hardening: anti-back-power). false means the
// NeoPixel transport is never initialized and no WS2812 frame is ever
// transmitted — see src/status/LedRing.cpp.
constexpr bool kServoPowerAvailable = false;
constexpr bool kBatteryAvailable    = false;
constexpr bool kLedRailPowered      = false;

constexpr uint32_t kUsbSerialBaud = 115200;   // USB CDC ignores baud; kept for host-side tooling parity.
constexpr uint32_t kServoBusBaud  = 1000000;  // ST3215 / Seeed driver, hardware-proven.
constexpr uint32_t kDalyBusBaud   = 9600;     // DALY XY-017 RS485, hardware-proven.

}  // namespace build
}  // namespace matdog

#endif  // MATDOG_CONFIG_BUILD_CONFIG_H
