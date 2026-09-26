#ifndef MATDOG_STATUS_LED_RING_H
#define MATDOG_STATUS_LED_RING_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "../config/BuildConfig.h"
#include "../config/Pins.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"
#include "LedStatusPolicy.h"

namespace matdog {
namespace status {

enum class LedDiagnostic : uint8_t { NONE, CHASE, SOC_TEST };
const char* toString(LedDiagnostic diagnostic);

// WS2812B x12 ring driver, GPIO47.
//
// No prior MATDOG LED source was found under ~/MATDOG/runtime/esp32 (audit
// grep for *led*/*ring*/*ws2812*/*neopixel* found none) — this is the one
// genuinely new low-level module in V0.1, as anticipated by the handoff.
//
// Built on Adafruit_NeoPixel (mature, installed for this task; no local
// custom WS2812 timing/RMT code).
//
// SESSION 2 HARDENING — anti-back-power: under the current USB_ONLY profile
// (build::kLedRailPowered == false) the 5V rail feeding the ring is
// physically absent. Session 1 still called pixels_.begin()/show(), which
// configures GPIO47 as an output and transmits WS2812 bit-banged frames
// toward an unpowered peripheral. This module now withholds the NeoPixel
// transport entirely when the rail is not powered: begin() only sets
// GPIO47 to INPUT (a defined, conservative, high-impedance state — not
// left floating/undefined), pixels_.begin()/show() are never called, and
// both LED diagnostics are refused with an explicit reason rather than silently
// no-op'd. GPIO47 itself is never read to "detect" whether the rail is
// powered (that would still involve driving/sensing the pin without a
// defined safe protocol) — power state is a profile fact, not something
// this module tries to infer electrically.
//
// When build::kLedRailPowered is true (ROBOT_POWERED profile), this
// module initializes and drives the ring exactly as Session 1 did.
class LedRing {
 public:
  static constexpr uint16_t kNumPixels = 12;
  // Conservative ceiling well under 255/max, for whenever the rail is
  // actually powered — kept even though USB_ONLY never reaches a show().
  static constexpr uint8_t kMaxBrightness = 60;

  bool begin();
  void update(uint32_t now_ms);
  core::ModuleHealth health() const { return core::toModuleHealth(core::classify(availability())); }
  core::AvailabilityStatus availability() const;

  void off();
  // r/g/b in 0..255; brightness in 0..kMaxBrightness (clamped). No-op if
  // the rail is not powered.
  void setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
  // Independent per-pixel brightness; the same ceiling/transport guard as
  // setSolid(). LedStatusManager is the sole periodic caller.
  void setFrame(const LedFrame& frame);

  // Starts (or restarts) a short non-blocking low-brightness diagnostic
  // chase. Refuses (returns false) if the rail is not powered — see
  // blockedReason() for the diagnostic text to report.
  bool startTest();
  // 600 ms per level, 0..12, a 1200 ms pause, then 12..0. Absolute elapsed
  // time bounds the diagnostic to 16.8 s even if update() misses ticks.
  bool startSocTest();
  bool testRunning() const { return diagnostic_ != LedDiagnostic::NONE; }
  LedDiagnostic diagnostic() const { return diagnostic_; }
  static const char* blockedReason() { return "LED_RAIL_UNPOWERED"; }

  // True once any WS2812 frame has actually been transmitted this boot.
  // Must stay false for the entire session under USB_ONLY.
  bool dataPinDriven() const { return data_pin_driven_; }

 private:
  Adafruit_NeoPixel pixels_{kNumPixels, pins::kLedRingDin, NEO_GRB + NEO_KHZ800};
  core::InitializationState init_ = core::InitializationState::NOT_INITIALIZED;
  LedDiagnostic diagnostic_ = LedDiagnostic::NONE;
  uint16_t test_step_ = 0;
  uint32_t test_last_step_ms_ = 0;
  uint32_t soc_test_started_ms_ = 0;
  uint8_t soc_test_segments_ = 0;
  bool data_pin_driven_ = false;
  static constexpr uint32_t kTestStepMs = 120;
  static constexpr uint32_t kSocTestStepMs = 600;
  static constexpr uint32_t kSocTestPauseMs = 1200;
  static constexpr uint32_t kSocTestLegMs = (kSocPixelCount + 1) * kSocTestStepMs;
  static constexpr uint32_t kSocTestDurationMs = 2 * kSocTestLegMs + kSocTestPauseMs;

  // Also used by the bounded diagnostic without cancelling its ownership.
  void renderFrame(const LedFrame& frame);
};

static_assert(LedRing::kNumPixels == kSocPixelCount, "SOC mapping must cover the ring");

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_RING_H
