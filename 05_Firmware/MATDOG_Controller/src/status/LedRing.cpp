#include "LedRing.h"

namespace matdog {
namespace status {

bool LedRing::begin() {
  pixels_.begin();
  pixels_.setBrightness(kMaxBrightness);
  pixels_.clear();
  pixels_.show();  // Boot state is always OFF.

  health_ = core::ModuleHealth::OK;
  return true;
}

void LedRing::off() {
  test_active_ = false;
  pixels_.clear();
  pixels_.show();
}

void LedRing::setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  test_active_ = false;
  brightness = brightness > kMaxBrightness ? kMaxBrightness : brightness;
  pixels_.setBrightness(brightness);
  for (uint16_t i = 0; i < kNumPixels; ++i) {
    pixels_.setPixelColor(i, pixels_.Color(r, g, b));
  }
  pixels_.show();
}

void LedRing::startTest() {
  test_active_ = true;
  test_step_ = 0;
  test_last_step_ms_ = millis();
  pixels_.setBrightness(kMaxBrightness);
  pixels_.clear();
  pixels_.show();
}

void LedRing::update(uint32_t now_ms) {
  if (!test_active_) return;

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
