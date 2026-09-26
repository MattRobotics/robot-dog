#ifndef MATDOG_STATUS_LED_STATUS_POLICY_H
#define MATDOG_STATUS_LED_STATUS_POLICY_H

#include <stdint.h>

#include "../core/SystemState.h"

// Pure LED presentation decision core. Deliberately <stdint.h> plus
// core/SystemState.h only — no <Arduino.h>, no LedRing, no Adafruit_NeoPixel
// — the same host-linkable contract as network/WifiPolicy.* and
// update/OtaPolicy.*: scripts/tests/test_led_status_policy.cpp links this
// translation unit directly, so the priority order and effects under test
// are the ones that ship. status/LedStatusManager.* is the only unit that
// touches LedRing; it turns the LedEffect this policy computes into exactly
// one setSolid() call per tick.
//
// LED PRESENTATION IS NEVER SOURCE OF TRUTH. Every input here is a snapshot
// already computed by its owning subsystem (SystemState, ActuatorAuthority,
// CalibrationManager, WifiPolicy) — this file duplicates no hardware read
// and holds no telemetry of its own beyond the derived on-screen state.
//
// Battery/charging LED states (I2 architecture table, 09_Logs/Development_
// Log/2026-09-25_I2_LED_STATUS_MANAGER.md) are deliberately NOT part of this
// enum yet: MATDOG_POWER_STATES_AND_CHARGING.md is explicit that FULL must
// never be inferred from SOC alone and that no reviewed empirical threshold
// exists for it. Adding those states is a separate, reviewed change once a
// threshold policy exists — not a placeholder guessed here.

namespace matdog {
namespace status {

// Deterministic priority, FAULT highest, READY lowest/default. Reordering
// this list is a reviewed safety/legibility decision, not a rendering
// tweak — see the I2 architecture table referenced above.
enum class LedPresentationState : uint8_t {
  READY                       = 0,
  BOOTING                     = 1,
  WIFI_CONNECTING             = 2,
  DEGRADED                    = 3,
  CALIBRATION_IN_PROGRESS     = 4,
  FIRMWARE_UPDATE_IN_PROGRESS = 5,
  FAULT                       = 6,
};

constexpr uint8_t kLedPresentationStateCount = 7;

// Snapshot of everything the policy is allowed to look at. Each field is
// already-computed state from its owning subsystem — no field here may ever
// become a second read of hardware.
struct LedStatusInputs {
  core::SystemHealth system_health = core::SystemHealth::BOOTING;
  // ActuatorAuthorityArbiter::inhibited() && inhibitReason() == FIRMWARE_UPDATE.
  bool firmware_update_in_progress = false;
  // CalibrationManager::sessionLive() — PREFLIGHT or ACTIVE, live or replay.
  bool calibration_in_progress = false;
  // WifiManager::status().state in {RADIO_STARTING, CONNECTING}.
  bool wifi_connecting = false;
};

// r/g/b in 0..255; brightness already time-modulated and capped by the
// caller's max_brightness — passed straight to LedRing::setSolid().
struct LedEffect {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint8_t brightness = 0;
};

// Highest-priority true condition wins; deterministic, single evaluation,
// no history beyond what LedStatusInputs carries this tick.
LedPresentationState selectLedState(const LedStatusInputs& inputs);

// Pure function of (state, now_ms, max_brightness). Solid states ignore
// now_ms; breathing states derive a triangle-wave brightness from it —
// never a delay, never a blocking wait.
LedEffect ledEffectFor(LedPresentationState state, uint32_t now_ms, uint8_t max_brightness);

const char* toString(LedPresentationState state);

// Thin stateful wrapper so the manager has one call site per tick.
class LedStatusPolicy {
 public:
  LedEffect update(const LedStatusInputs& inputs, uint32_t now_ms, uint8_t max_brightness);
  LedPresentationState state() const { return state_; }

 private:
  LedPresentationState state_ = LedPresentationState::BOOTING;
};

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_STATUS_POLICY_H
