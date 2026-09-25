#include "LedStatusPolicy.h"

namespace matdog {
namespace status {

namespace {

constexpr uint32_t kBreathePeriodMs = 2000;   // one full dim -> bright -> dim cycle
constexpr uint8_t kBreatheMinBrightness = 6;  // never fully dark - stays a visible "alive" pulse

// Triangle wave, 0..half rising then half..period falling, scaled into
// [min_b, max_b]. Bounded, integer-only, no floating point, no state -
// driven entirely by the now_ms the caller already has.
uint8_t triangleBrightness(uint32_t now_ms, uint32_t period_ms, uint8_t min_b, uint8_t max_b) {
  if (max_b <= min_b) return max_b;
  const uint32_t phase = now_ms % period_ms;
  const uint32_t half = period_ms / 2;
  const uint32_t level = (phase <= half) ? phase : (period_ms - phase);  // 0..half
  const uint32_t range = static_cast<uint32_t>(max_b - min_b);
  return static_cast<uint8_t>(min_b + (level * range) / half);
}

}  // namespace

LedPresentationState selectLedState(const LedStatusInputs& in) {
  if (in.system_health == core::SystemHealth::FAULT) {
    return LedPresentationState::FAULT;
  }
  if (in.firmware_update_in_progress) {
    return LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS;
  }
  if (in.calibration_in_progress) {
    return LedPresentationState::CALIBRATION_IN_PROGRESS;
  }
  if (in.system_health == core::SystemHealth::DEGRADED) {
    return LedPresentationState::DEGRADED;
  }
  if (in.wifi_connecting) {
    return LedPresentationState::WIFI_CONNECTING;
  }
  if (in.system_health == core::SystemHealth::BOOTING) {
    return LedPresentationState::BOOTING;
  }
  // READY, and the fallback for SystemHealth::MAINTENANCE, which
  // SystemState::update() never actually produces today (see SystemState.h)
  // - no state above claims it, so a future producer of it is not silently
  // misreported as a fault or masked by this table.
  return LedPresentationState::READY;
}

LedEffect ledEffectFor(LedPresentationState state, uint32_t now_ms, uint8_t max_brightness) {
  switch (state) {
    case LedPresentationState::FAULT:
      return LedEffect{255, 0, 0, max_brightness};
    case LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS:
      return LedEffect{0, 80, 255,
                        triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                            max_brightness)};
    case LedPresentationState::CALIBRATION_IN_PROGRESS:
      return LedEffect{160, 0, 220,
                        triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                            max_brightness)};
    case LedPresentationState::DEGRADED:
      return LedEffect{255, 140, 0, static_cast<uint8_t>(max_brightness / 2)};
    case LedPresentationState::WIFI_CONNECTING:
      return LedEffect{0, 200, 200,
                        triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                            max_brightness)};
    case LedPresentationState::BOOTING:
      return LedEffect{255, 255, 255, static_cast<uint8_t>(max_brightness / 3)};
    case LedPresentationState::READY:
      return LedEffect{0, 255, 0, static_cast<uint8_t>(max_brightness / 3)};
  }
  return LedEffect{0, 0, 0, 0};
}

const char* toString(LedPresentationState state) {
  switch (state) {
    case LedPresentationState::READY:                     return "READY";
    case LedPresentationState::BOOTING:                   return "BOOTING";
    case LedPresentationState::WIFI_CONNECTING:            return "WIFI_CONNECTING";
    case LedPresentationState::DEGRADED:                   return "DEGRADED";
    case LedPresentationState::CALIBRATION_IN_PROGRESS:    return "CALIBRATION_IN_PROGRESS";
    case LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS: return "FIRMWARE_UPDATE_IN_PROGRESS";
    case LedPresentationState::FAULT:                      return "FAULT";
  }
  return "UNKNOWN";
}

LedEffect LedStatusPolicy::update(const LedStatusInputs& inputs, uint32_t now_ms,
                                  uint8_t max_brightness) {
  state_ = selectLedState(inputs);
  return ledEffectFor(state_, now_ms, max_brightness);
}

}  // namespace status
}  // namespace matdog
