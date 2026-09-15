#include "PowerState.h"

namespace matdog {
namespace core {

const char* toString(PowerState state) {
  switch (state) {
    case PowerState::BOOTING:             return "BOOTING";
    case PowerState::POWER_CHECK:         return "POWER_CHECK";
    case PowerState::RUN:                 return "RUN";
    case PowerState::SHUTDOWN_REQUESTED:  return "SHUTDOWN_REQUESTED";
    case PowerState::SHUTTING_DOWN:       return "SHUTTING_DOWN";
    case PowerState::POWER_CUT_REQUESTED: return "POWER_CUT_REQUESTED";
    case PowerState::POWER_CUT_FAILED:    return "POWER_CUT_FAILED";
  }
  return "UNKNOWN";
}

void PowerStateMachine::requestShutdown() {
  if (state_ == PowerState::RUN) {
    state_ = PowerState::SHUTDOWN_REQUESTED;
  }
}

bool PowerStateMachine::update(uint32_t now_ms) {
  switch (state_) {
    case PowerState::SHUTDOWN_REQUESTED:
      // Step order per handoff 21A: reject new motion (V0.1 has none to
      // reject — no motion path exists), servo-safe state is already the
      // permanent boot state, nonessential peripherals need no quiescing
      // beyond what modules already do. Proceed straight to the BMS-link
      // confirmation step.
      shutdown_started_ms_ = now_ms;
      state_ = PowerState::SHUTTING_DOWN;
      return true;

    case PowerState::SHUTTING_DOWN:
      // Would confirm DALY comms health here before requesting the cut.
      state_ = PowerState::POWER_CUT_REQUESTED;
      return true;

    case PowerState::POWER_CUT_REQUESTED:
      // The verified K-Series "Discharge MOS OFF" write does not exist in
      // V0.1 (handoff 8A.8). No frame is transmitted. Fail closed rather
      // than claim OFF, which this firmware cannot observe or cause.
      state_ = PowerState::POWER_CUT_FAILED;
      return false;

    default:
      return false;
  }
}

}  // namespace core
}  // namespace matdog
