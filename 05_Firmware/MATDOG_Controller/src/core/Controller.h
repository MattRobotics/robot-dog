#ifndef MATDOG_CORE_CONTROLLER_H
#define MATDOG_CORE_CONTROLLER_H

#include <Arduino.h>

#include "../calibration/CalibrationManager.h"
#include "../imu/Bno085Imu.h"
#include "../network/WifiManager.h"
#include "../power/DalyBms.h"
#include "../servo/ServoBus.h"
#include "../servo/ServoCensus.h"
#include "../servo/ServoPreflight.h"
#include "../status/LedRing.h"
#include "../status/LedStatusManager.h"
#include "../update/OtaManager.h"
#include "ActuatorAuthority.h"
#include "CommandRouter.h"
#include "OperatingMode.h"
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
  servo::ServoCensus servo_census_;  // semantic census over servo_bus_; never auto-start
  servo::ServoPreflight servo_preflight_;  // H0 leg verification; read-only, never auto-started
  imu::Bno085Imu imu_;
  power::DalyBms daly_;
  status::LedRing led_;
  // The single periodic owner of LED presentation, above led_. See
  // status/LedStatusManager.h.
  status::LedStatusManager led_status_;
  // Owns the radio; owns nothing else. It has no path to ServoBus, to
  // OperatingMode or to any actuator — see network/WifiManager.h.
  network::WifiManager wifi_;
  // Owns the OTA subsystem. In OTA-A it runs the first-boot rollback
  // lifecycle and reports state; it has no transport, so nothing can feed it
  // an image.
  update::OtaManager ota_;

  SystemState system_state_;
  PowerStateMachine power_state_;
  OperatingModeManager operating_mode_;
  // THE central arbiter of actuator write authority. Exactly one instance
  // exists, here. No other component keeps a copy of its state - they hold a
  // pointer and ask. Orthogonal to operating_mode_: that says what the
  // Controller is doing, this says who, if anyone, may write actuators.
  ActuatorAuthorityArbiter authority_;
  // The ONE calibration session manager. It holds no transport and cannot
  // command a joint; it arbitrates a session through authority_ and records
  // evidence. See calibration/CalibrationManager.h.
  calibration::CalibrationManager calibration_;
  CommandRouter command_router_;

  // Set only at the very end of begin(). The OTA self-check must not treat a
  // half-initialized Controller as evidence that the image works.
  bool initialized_ = false;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_CONTROLLER_H
