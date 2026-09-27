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

  bool enableTorque(uint8_t bus_id) override {
    return bus_ != nullptr && bus_->enableTorqueOn(bus_id);
  }

  bool writeGoalPosition(uint8_t bus_id, uint16_t target_tick) override {
    return bus_ != nullptr && bus_->writeGoalPosition(bus_id, target_tick);
  }

 private:
  ServoBus* bus_ = nullptr;
};

}  // namespace servo
}  // namespace matdog

#endif
