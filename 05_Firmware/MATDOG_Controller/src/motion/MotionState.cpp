#include "MotionState.h"
namespace matdog { namespace motion {
bool MotionStateMachine::apply(MotionEvent event) {
  if (event == MotionEvent::DISABLE || event == MotionEvent::FAULT) {
    state_ = MotionState::OFF;
    return true;
  }
  switch (state_) {
    case MotionState::OFF:
      if (event == MotionEvent::ENABLE) { state_=MotionState::IDLE; return true; }
      break;
    case MotionState::IDLE:

      break;
    case MotionState::STAND_TRANSITION:

      if (event == MotionEvent::STOP) { state_=MotionState::STOPPING; return true; }
      break;
    case MotionState::STAND:
      if (event == MotionEvent::STOP) { state_=MotionState::STOPPING; return true; }
      break;
    case MotionState::GAIT_START:
    case MotionState::WALK:
    case MotionState::TROT:
      break;  // Owned only by Locomotion, not injectable into the G3 machine.
    case MotionState::STOPPING:
      if (event == MotionEvent::STOP_COMPLETE) { state_=MotionState::IDLE; return true; }
      break;
  }
  return false;  // invalid event/state pair leaves state unchanged
}
bool MotionStateMachine::beginVerifiedStand() {
  if(state_!=MotionState::IDLE)return false;
  state_=MotionState::STAND_TRANSITION;return true;
}
bool MotionStateMachine::completeVerifiedStand() {
  if(state_!=MotionState::STAND_TRANSITION)return false;
  state_=MotionState::STAND;return true;
}
} }
