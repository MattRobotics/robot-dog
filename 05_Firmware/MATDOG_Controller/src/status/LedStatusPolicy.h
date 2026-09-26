#ifndef MATDOG_STATUS_LED_STATUS_POLICY_H
#define MATDOG_STATUS_LED_STATUS_POLICY_H

#include <stdint.h>

#include "../core/SystemState.h"

// Pure, host-linked presentation over cached facts. No hardware reads,
// subsystem writes, motion authority or charge-completion policy lives here.
namespace matdog {
namespace status {

constexpr uint8_t kSocPixelCount = 12;
constexpr uint8_t kSocStartPixel = 1;
enum class SocDirection : uint8_t { CLOCKWISE };
constexpr SocDirection kSocDirection = SocDirection::CLOCKWISE;
// Physical 0 = 11 o'clock, 1 = 12 o'clock, 2 = 1 o'clock, ... 11 = 10 o'clock.
// Native physical index progression is clockwise. The SOC bar starts at noon.
constexpr uint8_t kSocPixelOrder[kSocPixelCount] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0};

// Increasing priority; READY includes the indeterminate battery renderer.
enum class LedPresentationState : uint8_t {
  READY = 0,
  CHARGING,
  CHARGE_COMPLETE_VERIFIED,
  BOOTING,
  WIFI_CONNECTING,
  DEGRADED,
  CHARGING_FAULT,
  CALIBRATION_IN_PROGRESS,
  FIRMWARE_UPDATE_IN_PROGRESS,
  FAULT,
};
constexpr uint8_t kLedPresentationStateCount = 10;

struct LedStatusInputs {
  core::SystemHealth system_health = core::SystemHealth::BOOTING;
  bool firmware_update_in_progress = false;
  bool calibration_in_progress = false;
  bool wifi_connecting = false;
  bool sample_valid = false;
  bool daly_comm_ok = false;
  uint32_t telemetry_age_ms = UINT32_MAX;
  float soc_percent = 0;
  bool battery_charging = false;
  bool battery_alarm = false;
  // Reserved for a future reviewed power policy. No production producer.
  // SOC (even 100%) can never set this fact.
  bool charge_complete_verified = false;
};

// Independent brightness per physical pixel, capped at the caller's ceiling.
struct LedEffect {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint8_t brightness = 0;
};
struct LedFrame {
  LedEffect pixels[kSocPixelCount];
};

// The manager's read-only status is the same derived model used to render.
// soc_percent/segments are meaningful only when soc_valid is true.
struct LedStatusSnapshot {
  LedPresentationState presentation = LedPresentationState::BOOTING;
  bool soc_valid = false;
  float soc_percent = 0;
  uint8_t soc_segments = 0;
  bool charging = false;
  bool charging_fault = false;
  bool charge_complete_verified = false;
};

LedPresentationState selectLedState(const LedStatusInputs& inputs);
LedEffect ledEffectFor(LedPresentationState state, uint32_t now_ms, uint8_t max_brightness);
uint8_t socCompletedSegments(float reported_soc);
LedFrame socBarFrame(uint8_t completed, uint8_t max_brightness);
const char* toString(LedPresentationState state);

class LedStatusPolicy {
 public:
  LedFrame update(const LedStatusInputs& inputs, uint32_t now_ms, uint8_t max_brightness);
  LedPresentationState state() const { return snapshot_.presentation; }
  const LedStatusSnapshot& snapshot() const { return snapshot_; }

 private:
  LedStatusSnapshot snapshot_;
};

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_STATUS_POLICY_H
