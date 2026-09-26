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
  diagnostic_ = LedDiagnostic::NONE;
  if (!build::kLedRailPowered) return;
  pixels_.clear();
  pixels_.show();
}

void LedRing::setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  diagnostic_ = LedDiagnostic::NONE;
  if (!build::kLedRailPowered) return;
  brightness = brightness > kMaxBrightness ? kMaxBrightness : brightness;
  pixels_.setBrightness(brightness);
  for (uint16_t i = 0; i < kNumPixels; ++i) {
    pixels_.setPixelColor(i, pixels_.Color(r, g, b));
  }
  pixels_.show();
  data_pin_driven_ = true;
}

void LedRing::setFrame(const LedFrame& frame) {
  diagnostic_ = LedDiagnostic::NONE;
  renderFrame(frame);
}

void LedRing::renderFrame(const LedFrame& frame) {
  if (!build::kLedRailPowered) return;
  // Each RGB value carries its own brightness. At 255 the NeoPixel global
  // scale is disabled, so independent pixels are never scaled twice.
  pixels_.setBrightness(255);
  for (uint16_t i = 0; i < kNumPixels; ++i) {
    const LedEffect& effect = frame.pixels[i];
    const uint8_t brightness = effect.brightness > kMaxBrightness
                                  ? kMaxBrightness : effect.brightness;
    // Match NeoPixel's (brightness + 1) / 256 quantization used by the
    // legacy setSolid() path; preserves every existing RGB/effect value.
    const uint16_t scale = static_cast<uint16_t>(brightness) + 1;
    pixels_.setPixelColor(i, pixels_.Color(
        static_cast<uint8_t>((effect.r * scale) >> 8),
        static_cast<uint8_t>((effect.g * scale) >> 8),
        static_cast<uint8_t>((effect.b * scale) >> 8)));
  }
  pixels_.show();
  data_pin_driven_ = true;
}

bool LedRing::startTest() {
  if (!build::kLedRailPowered) return false;  // caller reports blockedReason()

  diagnostic_ = LedDiagnostic::CHASE;
  test_step_ = 0;
  test_last_step_ms_ = millis();
  pixels_.setBrightness(kMaxBrightness);
  pixels_.clear();
  pixels_.show();
  data_pin_driven_ = true;
  return true;
}

bool LedRing::startSocTest() {
  if (!build::kLedRailPowered) return false;
  diagnostic_ = LedDiagnostic::SOC_TEST;
  soc_test_started_ms_ = millis();
  soc_test_segments_ = 0;
  renderFrame(socBarFrame(0, kMaxBrightness));
  return true;
}

void LedRing::update(uint32_t now_ms) {
  if (!testRunning()) return;
  if (!build::kLedRailPowered) {
    // Both starts refuse this profile; the transport remains guarded here.
    diagnostic_ = LedDiagnostic::NONE;
    return;
  }

  if (diagnostic_ == LedDiagnostic::SOC_TEST) {
    const uint32_t elapsed = now_ms - soc_test_started_ms_;
    // A command can start after Controller captured this tick's now_ms.
    // Treat that earlier tick as pre-start, using the usual millis() half
    // range ordering; otherwise unsigned underflow would end it at once.
    if (elapsed > UINT32_MAX / 2) return;
    if (elapsed >= kSocTestDurationMs) {
      off();
      return;
    }
    uint8_t segments = kSocPixelCount;
    if (elapsed < kSocTestLegMs) {
      segments = static_cast<uint8_t>(elapsed / kSocTestStepMs);
    } else if (elapsed >= kSocTestLegMs + kSocTestPauseMs) {
      segments = static_cast<uint8_t>(kSocPixelCount -
          (elapsed - kSocTestLegMs - kSocTestPauseMs) / kSocTestStepMs);
    }
    if (segments != soc_test_segments_) {
      soc_test_segments_ = segments;
      renderFrame(socBarFrame(segments, kMaxBrightness));
    }
    return;
  }

  if (now_ms - test_last_step_ms_ < kTestStepMs) return;
  test_last_step_ms_ = now_ms;

  if (test_step_ >= kNumPixels) {
    // One full lap complete; return to OFF.
    diagnostic_ = LedDiagnostic::NONE;
    pixels_.clear();
    pixels_.show();
    return;
  }

  pixels_.clear();
  pixels_.setPixelColor(test_step_, pixels_.Color(0, 40, 0));  // dim green marker
  pixels_.show();
  test_step_++;
}

const char* toString(LedDiagnostic diagnostic) {
  switch (diagnostic) {
    case LedDiagnostic::NONE: return "NONE";
    case LedDiagnostic::CHASE: return "CHASE";
    case LedDiagnostic::SOC_TEST: return "SOC_TEST";
  }
  return "UNKNOWN";
}

}  // namespace status
}  // namespace matdog
