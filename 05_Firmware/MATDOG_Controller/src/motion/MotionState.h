#ifndef MATDOG_MOTION_MOTION_STATE_H
#define MATDOG_MOTION_MOTION_STATE_H
#include <stdint.h>
namespace matdog { namespace motion {
enum class MotionState : uint8_t { OFF, IDLE, STAND_TRANSITION, STAND, STOPPING };
enum class MotionEvent : uint8_t { ENABLE, DISABLE, BEGIN_STAND, STAND_COMPLETE, STOP, STOP_COMPLETE, FAULT };
// Pure lifecycle only. Completion events are evidence from the caller, never
// timeouts. BEGIN_STAND requires a prepared valid contact-compatible trajectory.
// STOP cancels generation; STOP_COMPLETE acknowledges cancellation/hold setup.
// No deceleration profile, hardware command or authority is implied here.
class MotionStateMachine {
 public:
  MotionState state() const { return state_; }
  bool apply(MotionEvent event);
 private:
  MotionState state_ = MotionState::OFF;
};
} }
#endif
