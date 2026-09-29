#ifndef MATDOG_SERVO_SERVO_BUS_ACTUATOR_BACKEND_H
#define MATDOG_SERVO_SERVO_BUS_ACTUATOR_BACKEND_H

#include "../actuator/ActuatorRuntime.h"
#include "ServoBus.h"

namespace matdog {
namespace servo {

// CR3-M3 minimal production ActuatorBackend.
//
// It owns no UART and creates no second transport: every call delegates to
// the one Controller-owned ServoBus. It deliberately exposes only the two
// methods already fixed by ActuatorBackend. SAFE_OFF is not routed through
// this class and remains directly reachable on ServoBus.
//
// Merely constructing/binding this adapter must not authorize motion.
// SafeActuatorPolicy + session-scoped calibration permit + command surface
// remain separate gates. Controller still uses nullptr at CR3-M3.
class ServoBusActuatorBackend final : public actuator::ActuatorBackend {
 public:
  void begin(ServoBus* bus) { bus_ = bus; }
  bool bound() const { return bus_ != nullptr; }

  actuator::BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    if (bus_ == nullptr) return actuator::BackendWriteOutcome::VERIFIED_NOT_APPLIED;
    return translate(bus_->enableTorqueOn(bus_id));
  }

  actuator::BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick,
                                                  actuator::MotionProfile profile) override {
    if (bus_ == nullptr) return actuator::BackendWriteOutcome::VERIFIED_NOT_APPLIED;
    GoalMotionProfile goal_profile = GoalMotionProfile::BOUNDED;
    switch (profile) {
      case actuator::MotionProfile::BOUNDED_DEFAULT:
        goal_profile = GoalMotionProfile::BOUNDED;
        break;
      case actuator::MotionProfile::CALIBRATION_SEARCH:
        goal_profile = GoalMotionProfile::SEARCH_ENVELOPE;
        break;
      default:
        return actuator::BackendWriteOutcome::VERIFIED_NOT_APPLIED;  // write nothing
    }
    return translate(bus_->writeGoalPosition(bus_id, target_tick, goal_profile));
  }

 private:
  // ServoBus's transport-level verdict and the actuator layer's
  // transport-independent one are deliberately the same three states under
  // different names (this class is the one place that is allowed to know
  // both vocabularies) - see ServoWriteVerifyResult's file comment for why
  // UNVERIFIED_NO_RESPONSE must never be collapsed into either verified case.
  static actuator::BackendWriteOutcome translate(ServoWriteVerifyResult result) {
    switch (result) {
      case ServoWriteVerifyResult::VERIFIED_APPLIED:
        return actuator::BackendWriteOutcome::VERIFIED_APPLIED;
      case ServoWriteVerifyResult::VERIFIED_NOT_APPLIED:
        return actuator::BackendWriteOutcome::VERIFIED_NOT_APPLIED;
      case ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE:
        return actuator::BackendWriteOutcome::UNCERTAIN;
    }
    return actuator::BackendWriteOutcome::UNCERTAIN;
  }

  ServoBus* bus_ = nullptr;
};

}  // namespace servo
}  // namespace matdog

#endif
