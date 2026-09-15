#ifndef MATDOG_STATUS_LED_RING_H
#define MATDOG_STATUS_LED_RING_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "../config/Pins.h"
#include "../core/SystemState.h"

namespace matdog {
namespace status {

// WS2812B x12 ring driver, GPIO47.
//
// No prior MATDOG LED source was found under ~/MATDOG/runtime/esp32 (audit
// grep for *led*/*ring*/*ws2812*/*neopixel* found none) — this is the one
// genuinely new low-level module in V0.1, as anticipated by the handoff.
//
// Built on Adafruit_NeoPixel (mature, installed for this task; no local
// custom WS2812 timing/RMT code). Requirements preserved: boots OFF, never
// drives all pixels at hardware-maximum brightness, non-blocking effects,
// a failure here must never stop the rest of the controller (this module
// therefore has no path that can loop or block).
class LedRing {
 public:
  static constexpr uint16_t kNumPixels = 12;
  // Conservative ceiling well under 255/max. Chosen so that even the
  // explicit @LED TEST diagnostic stays well short of the ring's rated
  // current — there is no optical validation available in USB-only bench
  // state (5V rail absent) to justify anything brighter.
  static constexpr uint8_t kMaxBrightness = 60;

  bool begin();
  void update(uint32_t now_ms);
  core::ModuleHealth health() const { return health_; }

  void off();
  // r/g/b in 0..255; brightness in 0..kMaxBrightness (clamped).
  void setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

  // Starts (or restarts) a short non-blocking low-brightness diagnostic
  // chase. Safe to call repeatedly; stops automatically after one lap.
  void startTest();
  bool testRunning() const { return test_active_; }

 private:
  Adafruit_NeoPixel pixels_{kNumPixels, pins::kLedRingDin, NEO_GRB + NEO_KHZ800};
  core::ModuleHealth health_ = core::ModuleHealth::NOT_INITIALIZED;

  bool test_active_ = false;
  uint16_t test_step_ = 0;
  uint32_t test_last_step_ms_ = 0;
  static constexpr uint32_t kTestStepMs = 120;
};

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_RING_H
