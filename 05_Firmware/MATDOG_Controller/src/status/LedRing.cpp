#include "LedRing.h"

namespace matdog {
namespace status {

bool LedRing::begin() {
  if (!build::kLedRailPowered) {
    // Anti-back-power: do not configure GPIO47 as a NeoPixel output and do
    // not transmit any frame. Put it in a defined, conservative,
    // high-impedance state instead.
    pinMode(pins::kLedRingDin, INPUT);
    init_ = core::InitializationState::DEFERRED;
    return true;
  }

  pixels_.begin();
  pixels_.setBrightness(kMaxBrightness);
  pixels_.clear();
  pixels_.show();  // Boot state is always OFF.
  data_pin_driven_ = true;

  init_ = core::InitializationState::INITIALIZED;
  return true;
}

core::AvailabilityStatus LedRing::availability() const {
  core::AvailabilityStatus a;
  a.init = init_;
  // G2: derived from the active hardware profile (core/Availability.h).
  // The ring stays OPTIONAL even when powered — LED state must never be
  // able to fault an otherwise healthy robot.
  a.detected = core::detectedStateForLedRail(build::kProfileExpectations);
  a.expected = core::expectedStateForLedRail(build::kProfileExpectations);
  return a;
}

void LedRing::off() {
  test_active_ = false;
  if (!build::kLedRailPowered) return;
  pixels_.clear();
  pixels_.show();
}

void LedRing::setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  test_active_ = false;
  if (!build::kLedRailPowered) return;
  brightness = brightness > kMaxBrightness ? kMaxBrightness : brightness;
  pixels_.setBrightness(brightness);
  for (uint16_t i = 0; i < kNumPixels; ++i) {
    pixels_.setPixelColor(i, pixels_.Color(r, g, b));
  }
  pixels_.show();
  data_pin_driven_ = true;
}

bool LedRing::startTest() {
  if (!build::kLedRailPowered) return false;  // caller reports blockedReason()

  test_active_ = true;
  test_step_ = 0;
  test_last_step_ms_ = millis();
  pixels_.setBrightness(kMaxBrightness);
  pixels_.clear();
  pixels_.show();
  data_pin_driven_ = true;
  return true;
}

void LedRing::update(uint32_t now_ms) {
  if (!test_active_) return;
  if (!build::kLedRailPowered) {
    // Defensive: should be unreachable since startTest() refuses to set
    // test_active_ in this profile, but never touch the pin regardless.
    test_active_ = false;
    return;
  }

  if (now_ms - test_last_step_ms_ < kTestStepMs) return;
  test_last_step_ms_ = now_ms;

  if (test_step_ >= kNumPixels) {
    // One full lap complete; return to OFF.
    test_active_ = false;
    pixels_.clear();
    pixels_.show();
    return;
  }

  pixels_.clear();
  pixels_.setPixelColor(test_step_, pixels_.Color(0, 40, 0));  // dim green marker
  pixels_.show();
  test_step_++;
}

}  // namespace status
}  // namespace matdog
