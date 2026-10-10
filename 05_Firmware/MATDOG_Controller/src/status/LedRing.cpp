#include "LedRing.h"

#include <string.h>

namespace matdog {
namespace status {

#if MATDOG_LED_RMT_DMA
void Ws2812DmaStrip::begin() {
  // Once the channel owns the pin, a second begin() must not hand it back
  // to plain GPIO.
  if (channel_ != nullptr) return;
  // The library still puts the pin in its usual idle state (output, low)
  // before the channel is attached to it.
  Adafruit_NeoPixel::begin();

  rmt_tx_channel_config_t channel_config = {};
  channel_config.gpio_num = static_cast<gpio_num_t>(pin);
  channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
  channel_config.resolution_hz = kResolutionHz;
  channel_config.mem_block_symbols = kDmaBufferSymbols;
  channel_config.trans_queue_depth = 1;
  channel_config.flags.with_dma = 1;

  rmt_bytes_encoder_config_t encoder_config = {};
  encoder_config.bit0.level0 = 1;
  encoder_config.bit0.duration0 = kBit0HighTicks;
  encoder_config.bit0.level1 = 0;
  encoder_config.bit0.duration1 = kBit0LowTicks;
  encoder_config.bit1.level0 = 1;
  encoder_config.bit1.duration0 = kBit1HighTicks;
  encoder_config.bit1.level1 = 0;
  encoder_config.bit1.duration1 = kBit1LowTicks;
  encoder_config.flags.msb_first = 1;

  const char* failed = nullptr;
  if (rmt_new_tx_channel(&channel_config, &channel_) != ESP_OK) {
    channel_ = nullptr;
    failed = "CHANNEL";
  } else if (rmt_new_bytes_encoder(&encoder_config, &encoder_) != ESP_OK) {
    encoder_ = nullptr;
    failed = "ENCODER";
  } else if (rmt_enable(channel_) != ESP_OK) {
    failed = "ENABLE";
  }
  if (failed != nullptr) {
    releaseDma();
    Serial.printf("LED_TRANSPORT=RMT_LEGACY reason=DMA_%s_FAILED\n", failed);
    return;
  }
  Serial.println("LED_TRANSPORT=RMT_DMA");
}

void Ws2812DmaStrip::show() {
  if (channel_ == nullptr) {
    Adafruit_NeoPixel::show();
    return;
  }
  if (pixels == nullptr) return;
  // The WS2812 latch: at least 300 us of low line since the previous frame,
  // the rule (and the bounded spin) the library's own show() applies.
  while (!canShow()) {
  }
  rmt_transmit_config_t transmit_config = {};
  transmit_config.flags.queue_nonblocking = 1;
  const bool sent =
      rmt_transmit(channel_, encoder_, pixels, numBytes, &transmit_config) == ESP_OK &&
      rmt_tx_wait_all_done(channel_, kTransmitTimeoutMs) == ESP_OK;
  endTime = micros();
  if (!sent) {
    // Give the pin back and let the library drive it from now on, starting
    // with the frame that did not make it.
    releaseDma();
    Serial.println("LED_TRANSPORT=RMT_LEGACY reason=DMA_TRANSMIT_FAILED");
    Adafruit_NeoPixel::show();
  }
}

void Ws2812DmaStrip::releaseDma() {
  if (channel_ != nullptr) {
    rmt_disable(channel_);
    rmt_del_channel(channel_);
    channel_ = nullptr;
  }
  if (encoder_ != nullptr) {
    rmt_del_encoder(encoder_);
    encoder_ = nullptr;
  }
}
#endif  // MATDOG_LED_RMT_DMA

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
  sent_valid_ = false;
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
  sent_valid_ = false;
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
  sent_valid_ = false;
  data_pin_driven_ = true;
}

void LedRing::setFrame(const LedFrame& frame) {
  diagnostic_ = LedDiagnostic::NONE;
  renderFrame(frame);
}

void LedRing::renderFrame(const LedFrame& frame) {
  if (!build::kLedRailPowered) return;
  uint8_t rgb[kNumPixels][3];
  for (uint16_t i = 0; i < kNumPixels; ++i) {
    const LedEffect& effect = frame.pixels[i];
    const uint8_t brightness = effect.brightness > kMaxBrightness
                                  ? kMaxBrightness : effect.brightness;
    // Match NeoPixel's (brightness + 1) / 256 quantization used by the
    // legacy setSolid() path; preserves every existing RGB/effect value.
    const uint16_t scale = static_cast<uint16_t>(brightness) + 1;
    rgb[i][0] = static_cast<uint8_t>((effect.r * scale) >> 8);
    rgb[i][1] = static_cast<uint8_t>((effect.g * scale) >> 8);
    rgb[i][2] = static_cast<uint8_t>((effect.b * scale) >> 8);
  }
  // Unsigned difference: correct across the millis() wrap.
  const uint32_t now_ms = millis();
  if (sent_valid_ && memcmp(rgb, sent_rgb_, sizeof(rgb)) == 0 &&
      now_ms - sent_at_ms_ < kUnchangedRefreshMs) {
    return;  // the ring already shows these exact bytes
  }
  // Each RGB value carries its own brightness. At 255 the NeoPixel global
  // scale is disabled, so independent pixels are never scaled twice.
  pixels_.setBrightness(255);
  for (uint16_t i = 0; i < kNumPixels; ++i) {
    pixels_.setPixelColor(i, pixels_.Color(rgb[i][0], rgb[i][1], rgb[i][2]));
  }
  pixels_.show();
  memcpy(sent_rgb_, rgb, sizeof(rgb));
  sent_valid_ = true;
  sent_at_ms_ = now_ms;
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
  sent_valid_ = false;
  data_pin_driven_ = true;
  return true;
}

bool LedRing::startSocTest() {
  if (!build::kLedRailPowered) return false;
  diagnostic_ = LedDiagnostic::SOC_TEST;
  soc_test_started_ms_ = millis();
  soc_test_segments_ = 0;
  // A diagnostic always opens with a real transmission, whatever the
  // presentation path sent last.
  sent_valid_ = false;
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
    sent_valid_ = false;
    return;
  }

  pixels_.clear();
  pixels_.setPixelColor(test_step_, pixels_.Color(0, 40, 0));  // dim green marker
  pixels_.show();
  sent_valid_ = false;
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
