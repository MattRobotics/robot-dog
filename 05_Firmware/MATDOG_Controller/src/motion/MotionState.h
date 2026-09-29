#ifndef MATDOG_MOTION_MOTION_STATE_H
#define MATDOG_MOTION_MOTION_STATE_H
#include <stdint.h>
namespace matdog { namespace motion {
enum class MotionState : uint8_t { OFF, IDLE, STAND_TRANSITION, STAND, STOPPING };
enum class MotionEvent : uint8_t { ENABLE, DISABLE, BEGIN_STAND, STAND_COMPLETE, STOP, STOP_COMPLETE, FAULT };
// G3: BEGIN_STAND/STAND_COMPLETE cannot be injected as public events.
// StandTransition alone commits verified entry and successful full completion.
// STOP/DISABLE/FAULT cancel generation through that coordinator. STOP_COMPLETE
// acknowledges cancellation/hold setup, not physical braking or deceleration.
class MotionStateMachine {
 public:
  MotionState state() const { return state_; }
  bool apply(MotionEvent event);
 private:
  friend class StandTransition;
  bool beginVerifiedStand();
  bool completeVerifiedStand();
  MotionState state_ = MotionState::OFF;
};
} }
#endif
