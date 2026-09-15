#ifndef MATDOG_CORE_CONTROLLER_H
#define MATDOG_CORE_CONTROLLER_H

#include <Arduino.h>

#include "../imu/Bno085Imu.h"
#include "../power/DalyBms.h"
#include "../servo/ServoBus.h"
#include "../status/LedRing.h"
#include "CommandRouter.h"
#include "PowerState.h"
#include "SystemState.h"

namespace matdog {
namespace core {

// Top-level owner of every MATDOG Controller module. Owns initialization
// order, the non-blocking update loop, and health/power-state aggregation.
// Hardware logic lives in the modules below, not here or in the .ino.
class Controller {
 public:
  void begin();
  void update(uint32_t now_ms);

 private:
  void printBootBanner();

  servo::ServoBus servo_bus_;
  imu::Bno085Imu imu_;
  power::DalyBms daly_;
  status::LedRing led_;

  SystemState system_state_;
  PowerStateMachine power_state_;
  CommandRouter command_router_;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_CONTROLLER_H
