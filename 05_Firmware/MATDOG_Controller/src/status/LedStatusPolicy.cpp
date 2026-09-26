#include "LedStatusPolicy.h"

#include <cmath>

#include "../power/DalyProtocol.h"

namespace matdog {
namespace status {
namespace {

constexpr uint32_t kBreathePeriodMs = 2000;
constexpr uint32_t kSubtleBreathePeriodMs = 3000;
constexpr uint8_t kBreatheMinBrightness = 6;

uint8_t triangleBrightness(uint32_t now_ms, uint32_t period_ms, uint8_t min_b, uint8_t max_b) {
  if (max_b <= min_b) return max_b;
  const uint32_t phase = now_ms % period_ms;
  const uint32_t half = period_ms / 2;
  const uint32_t level = (phase <= half) ? phase : (period_ms - phase);
  return static_cast<uint8_t>(min_b + (level * (max_b - min_b)) / half);
}

uint8_t subtleBrightness(uint32_t now_ms, uint8_t max_brightness) {
  return triangleBrightness(now_ms, kSubtleBreathePeriodMs, kBreatheMinBrightness,
                            max_brightness / 3);
}

bool telemetryFresh(const LedStatusInputs& in) {
  return in.sample_valid && in.daly_comm_ok &&
         in.telemetry_age_ms <= power::kDalyTelemetryFreshnessMs;
}

LedPresentationState selectFromFacts(const LedStatusInputs& in, const LedStatusSnapshot& facts) {
  if (in.system_health == core::SystemHealth::FAULT) return LedPresentationState::FAULT;
  if (in.firmware_update_in_progress) return LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS;
  if (in.calibration_in_progress) return LedPresentationState::CALIBRATION_IN_PROGRESS;
  if (facts.charging_fault) return LedPresentationState::CHARGING_FAULT;
  if (in.system_health == core::SystemHealth::DEGRADED) return LedPresentationState::DEGRADED;
  if (in.wifi_connecting) return LedPresentationState::WIFI_CONNECTING;
  if (in.system_health == core::SystemHealth::BOOTING) return LedPresentationState::BOOTING;
  if (facts.charge_complete_verified) return LedPresentationState::CHARGE_COMPLETE_VERIFIED;
  if (facts.charging) return LedPresentationState::CHARGING;
  // Also preserves the legacy MAINTENANCE fallback.
  return LedPresentationState::READY;
}

LedStatusSnapshot factsFor(const LedStatusInputs& in) {
  LedStatusSnapshot facts;
  const bool fresh = telemetryFresh(in);
  facts.soc_valid = fresh && std::isfinite(in.soc_percent);
  if (facts.soc_valid) {
    facts.soc_percent = in.soc_percent;
    facts.soc_segments = socCompletedSegments(in.soc_percent);
  }
  facts.charging = fresh && in.battery_charging;
  facts.charging_fault = facts.charging && in.battery_alarm;
  facts.charge_complete_verified = fresh && !in.battery_alarm && in.charge_complete_verified;
  facts.presentation = selectFromFacts(in, facts);
  return facts;
}

LedFrame uniformFrame(const LedEffect& effect) {
  LedFrame frame;
  for (auto& pixel : frame.pixels) pixel = effect;
  return frame;
}

}  // namespace

uint8_t socCompletedSegments(float reported_soc) {
  if (!std::isfinite(reported_soc) || reported_soc <= 0) return 0;
  if (reported_soc >= 100) return kSocPixelCount;
  // Promote BEFORE multiplication: binary32 multiplication could round a
  // value just below n*100/12 upward. Binary64 represents float*12 exactly;
  // truncation of the nonnegative result implements floor with no epsilon.
  return static_cast<uint8_t>(static_cast<double>(reported_soc) * kSocPixelCount / 100.0);
}

LedFrame socBarFrame(uint8_t completed, uint8_t max_brightness) {
  LedFrame frame{};
  for (uint8_t logical = 0; logical < completed && logical < kSocPixelCount; ++logical) {
    frame.pixels[kSocPixelOrder[logical]] = {0, 255, 0,
                                           static_cast<uint8_t>(max_brightness / 3)};
  }
  return frame;
}

LedPresentationState selectLedState(const LedStatusInputs& in) {
  return factsFor(in).presentation;
}

LedEffect ledEffectFor(LedPresentationState state, uint32_t now_ms, uint8_t max_brightness) {
  switch (state) {
    case LedPresentationState::FAULT:
      return {255, 0, 0, max_brightness};
    case LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS:
      return {0, 80, 255, triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                           max_brightness)};
    case LedPresentationState::CALIBRATION_IN_PROGRESS:
      return {160, 0, 220, triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                            max_brightness)};
    case LedPresentationState::CHARGING_FAULT:
      return {255, 0, 0, triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                           max_brightness)};
    case LedPresentationState::DEGRADED:
      return {255, 140, 0, static_cast<uint8_t>(max_brightness / 2)};
    case LedPresentationState::WIFI_CONNECTING:
      return {0, 200, 200, triangleBrightness(now_ms, kBreathePeriodMs, kBreatheMinBrightness,
                                            max_brightness)};
    case LedPresentationState::BOOTING:
      return {255, 255, 255, subtleBrightness(now_ms, max_brightness)};
    case LedPresentationState::CHARGE_COMPLETE_VERIFIED:
    case LedPresentationState::CHARGING:
      return {0, 255, 0, subtleBrightness(now_ms, max_brightness)};
    case LedPresentationState::READY:
      return {0, 255, 0, static_cast<uint8_t>(max_brightness / 3)};
  }
  return {};
}

const char* toString(LedPresentationState state) {
  switch (state) {
    case LedPresentationState::READY: return "READY";
    case LedPresentationState::CHARGING: return "CHARGING";
    case LedPresentationState::CHARGE_COMPLETE_VERIFIED: return "CHARGE_COMPLETE_VERIFIED";
    case LedPresentationState::BOOTING: return "BOOTING";
    case LedPresentationState::WIFI_CONNECTING: return "WIFI_CONNECTING";
    case LedPresentationState::DEGRADED: return "DEGRADED";
    case LedPresentationState::CHARGING_FAULT: return "CHARGING_FAULT";
    case LedPresentationState::CALIBRATION_IN_PROGRESS: return "CALIBRATION_IN_PROGRESS";
    case LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS: return "FIRMWARE_UPDATE_IN_PROGRESS";
    case LedPresentationState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

LedFrame LedStatusPolicy::update(const LedStatusInputs& inputs, uint32_t now_ms,
                                 uint8_t max_brightness) {
  snapshot_ = factsFor(inputs);
  if (state() == LedPresentationState::READY || state() == LedPresentationState::CHARGING) {
    if (!snapshot_.soc_valid) {
      return uniformFrame({255, 140, 0, subtleBrightness(now_ms, max_brightness)});
    }
    LedFrame frame = socBarFrame(snapshot_.soc_segments, max_brightness);
    if (state() == LedPresentationState::CHARGING) {
      // At reported 100%, keep the final logical segment pulsing. Never FULL.
      const uint8_t next = snapshot_.soc_segments < kSocPixelCount
                               ? snapshot_.soc_segments : kSocPixelCount - 1;
      frame.pixels[kSocPixelOrder[next]] = ledEffectFor(state(), now_ms, max_brightness);
    }
    return frame;
  }
  return uniformFrame(ledEffectFor(state(), now_ms, max_brightness));
}

}  // namespace status
}  // namespace matdog
