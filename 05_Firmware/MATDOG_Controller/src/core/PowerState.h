#ifndef MATDOG_CORE_POWER_STATE_H
#define MATDOG_CORE_POWER_STATE_H

#include <Arduino.h>

namespace matdog {
namespace core {

// Power-state baseline per handoff section 21A / 8A.7.
//
// OFF is a physical state (the ESP32 has no execution power) and therefore
// has no in-firmware representation: firmware only ever observes itself
// already past KEY ON / BOOT. Internal representation starts at BOOTING.
//
// SHUTDOWN is intentionally self-power-cutting at the final step (Discharge
// MOS OFF). Because that write is NOT YET VERIFIED (handoff 8A.8), this
// state machine can reach SHUTTING_DOWN and attempt the cut, but the cut
// itself always fails closed into POWER_CUT_FAILED in V0.1 — the firmware
// must never report OFF, since it cannot know or cause that transition.
//
// PowerState::RUN is NOT the same concept as core::OperatingMode::RUN
// (OperatingMode.h, Session 2.1) despite the shared name — see that
// header's comment for the full distinction. In short: PowerState is
// about the power-on lifecycle (has the controller finished booting?);
// OperatingMode is about whether blocking servo diagnostics are safe to
// run right now (is a motion loop active?). They vary independently.
enum class PowerState : uint8_t {
  BOOTING              = 0,
  POWER_CHECK          = 1,
  RUN                  = 2,
  SHUTDOWN_REQUESTED   = 3,
  SHUTTING_DOWN        = 4,
  POWER_CUT_REQUESTED  = 5,
  POWER_CUT_FAILED      = 6,
};

const char* toString(PowerState state);

class PowerStateMachine {
 public:
  PowerState state() const { return state_; }

  // Called once boot-time module init has completed.
  void enterPowerCheck() { state_ = PowerState::POWER_CHECK; }

  // Called once HW/BMS check has been evaluated (V0.1: always proceeds —
  // no motion-authorization gate exists yet to block RUN on).
  void enterRun() { state_ = PowerState::RUN; }

  // Operator-initiated software shutdown request (e.g. @SYSTEM SHUTDOWN).
  // Safe to call at any time; only has effect from RUN.
  void requestShutdown();

  // Advances the shutdown sequence. Returns true while there is more work
  // to do (caller should keep calling), false once the sequence has
  // resolved into a terminal state (POWER_CUT_FAILED in V0.1, since the
  // verified Discharge MOS OFF write does not exist yet).
  bool update(uint32_t now_ms);

 private:
  PowerState state_ = PowerState::BOOTING;
  uint32_t shutdown_started_ms_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_POWER_STATE_H
