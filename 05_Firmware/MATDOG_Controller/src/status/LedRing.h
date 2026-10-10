#ifndef MATDOG_STATUS_LED_RING_H
#define MATDOG_STATUS_LED_RING_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "../config/BuildConfig.h"
#include "../config/Pins.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"
#include "LedStatusPolicy.h"

// The native ESP-IDF RMT driver, where the target has an RMT channel with
// DMA (ESP32-S3: TX channel 3). Absent on the host test stubs, which keep
// driving the recorded Adafruit_NeoPixel transport directly.
#if defined(__has_include)
#if __has_include("driver/rmt_tx.h") && __has_include("soc/soc_caps.h")
#include "soc/soc_caps.h"
#if SOC_RMT_SUPPORT_DMA
#include "driver/rmt_tx.h"
#define MATDOG_LED_RMT_DMA 1
#endif
#endif
#endif
#ifndef MATDOG_LED_RMT_DMA
#define MATDOG_LED_RMT_DMA 0
#endif

namespace matdog {
namespace status {

enum class LedDiagnostic : uint8_t { NONE, CHASE, SOC_TEST };
const char* toString(LedDiagnostic diagnostic);

#if MATDOG_LED_RMT_DMA
// WS2812 transport over RMT with DMA.
//
// Adafruit_NeoPixel still owns the pixel buffer, the GRB packing and the
// brightness arithmetic, so every byte handed to the wire is the byte it
// always was. Only show() changes. The library sends through the Arduino RMT
// HAL, which gives the channel one 48-symbol memory block: a 288-symbol frame
// is then refilled by the RMT threshold interrupt eleven times while it is
// being transmitted, each refill with a 28.8 us deadline. Here the frame is
// encoded once, in the calling task, into a DMA buffer large enough to hold
// it whole; GDMA feeds the RMT transmitter and no interrupt takes part until
// the transmission is over.
//
// If the DMA channel cannot be created, or ever fails at run time, the strip
// goes back to the library's own show() - the transport dev.3 shipped with.
class Ws2812DmaStrip : public Adafruit_NeoPixel {
 public:
  using Adafruit_NeoPixel::Adafruit_NeoPixel;
  ~Ws2812DmaStrip() { releaseDma(); }

  // The waveform Adafruit_NeoPixel's esp.c produces, unchanged: 100 ns
  // ticks, "1" = 800 ns high + 400 ns low, "0" = 400 ns high + 800 ns low,
  // most significant bit first, line low when idle.
  static constexpr uint32_t kResolutionHz = 10000000;
  static constexpr uint16_t kBit1HighTicks = 8;
  static constexpr uint16_t kBit1LowTicks = 4;
  static constexpr uint16_t kBit0HighTicks = 4;
  static constexpr uint16_t kBit0LowTicks = 8;
  // The driver splits this buffer across two DMA descriptors and only the
  // first is free of any refill callback: half of it must hold one whole
  // frame plus the end marker (see the static_assert below LedRing).
  static constexpr size_t kDmaBufferSymbols = 1024;
  // A frame lasts 0.35 ms. A transmission that has not finished by then is a
  // fault of the transport, never a reason to stall the Controller.
  static constexpr int kTransmitTimeoutMs = 20;

  void begin();  // hides Adafruit_NeoPixel::begin()
  void show();   // hides Adafruit_NeoPixel::show()
  bool dmaActive() const { return channel_ != nullptr; }

 private:
  void releaseDma();
  rmt_channel_handle_t channel_ = nullptr;
  rmt_encoder_handle_t encoder_ = nullptr;
};
using LedRingStrip = Ws2812DmaStrip;
#else
using LedRingStrip = Adafruit_NeoPixel;
#endif

// WS2812B x12 ring driver, GPIO47.
//
// No prior MATDOG LED source was found under ~/MATDOG/runtime/esp32 (audit
// grep for *led*/*ring*/*ws2812*/*neopixel* found none) — this is the one
// genuinely new low-level module in V0.1, as anticipated by the handoff.
//
// Built on Adafruit_NeoPixel (mature, installed for this task) for the pixel
// buffer, colour order and brightness. On the ESP32-S3 the frame is
// transmitted by Ws2812DmaStrip above instead of the library's own show().
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
  LedRingStrip pixels_{kNumPixels, pins::kLedRingDin, NEO_GRB + NEO_KHZ800};
  core::InitializationState init_ = core::InitializationState::NOT_INITIALIZED;
  LedDiagnostic diagnostic_ = LedDiagnostic::NONE;
  uint16_t test_step_ = 0;
  uint32_t test_last_step_ms_ = 0;
  uint32_t soc_test_started_ms_ = 0;
  uint8_t soc_test_segments_ = 0;
  bool data_pin_driven_ = false;
  // Transmit gating for renderFrame(). It used to re-send an identical
  // WS2812 frame on every Controller tick; it now transmits at once when the
  // frame's bytes differ from the last ones it transmitted, and otherwise
  // re-sends the unchanged frame once per kUnchangedRefreshMs so a ring that
  // lost its state (re-plugged, rail dip, one corrupted frame) still
  // recovers on its own. Every other transmit path - begin(), off(),
  // setSolid(), both diagnostics - invalidates the cache, so the next
  // renderFrame() always transmits. Nothing here runs under USB_ONLY.
  static constexpr uint32_t kUnchangedRefreshMs = 250;
  uint8_t sent_rgb_[kNumPixels][3] = {};
  bool sent_valid_ = false;
  uint32_t sent_at_ms_ = 0;
  static constexpr uint32_t kTestStepMs = 120;
  static constexpr uint32_t kSocTestStepMs = 600;
  static constexpr uint32_t kSocTestPauseMs = 1200;
  static constexpr uint32_t kSocTestLegMs = (kSocPixelCount + 1) * kSocTestStepMs;
  static constexpr uint32_t kSocTestDurationMs = 2 * kSocTestLegMs + kSocTestPauseMs;

  // Also used by the bounded diagnostic without cancelling its ownership.
  void renderFrame(const LedFrame& frame);
};

static_assert(LedRing::kNumPixels == kSocPixelCount, "SOC mapping must cover the ring");
#if MATDOG_LED_RMT_DMA
static_assert(LedRing::kNumPixels * 24 + 1 <= Ws2812DmaStrip::kDmaBufferSymbols / 2,
              "one WS2812 frame and its end marker must fit the first DMA descriptor");
#endif

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_RING_H
