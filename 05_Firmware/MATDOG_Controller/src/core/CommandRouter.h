#ifndef MATDOG_CORE_COMMAND_ROUTER_H
#define MATDOG_CORE_COMMAND_ROUTER_H

#include <Arduino.h>

#include "../imu/Bno085Imu.h"
#include "../power/DalyBms.h"
#include "../servo/ServoBus.h"
#include "../status/LedRing.h"
#include "Availability.h"
#include "PowerState.h"
#include "SystemState.h"

namespace matdog {
namespace core {

// Small, explicit, safe USB CDC diagnostic command surface.
// See handoff section 22. Deliberately does NOT expose: arbitrary EEPROM
// write, ID recode, factory reset, CalibrationOfs, broadcast write,
// unrestricted GoalPosition, or any DALY write.
class CommandRouter {
 public:
  struct Modules {
    servo::ServoBus* servo_bus;
    imu::Bno085Imu* imu;
    power::DalyBms* daly;
    status::LedRing* led;
    SystemState* system_state;
    PowerStateMachine* power_state;
  };

  void begin(const Modules& modules);
  void update(uint32_t now_ms);

 private:
  void handleLine(String line);
  void printHelp();
  void printStatus();
  void printImuStatus();
  void printBmsStatus();
  void printLedStatus();
  void printServoScanResult();
  void printServoRead(int id);
  void printServoSafeOff(int id);
  static void printAvailabilityLine(const char* label, const AvailabilityStatus& a);

  Modules modules_{};
  bool bms_stream_enabled_ = false;
  uint32_t last_bms_stream_ms_ = 0;
  bool servo_scan_result_pending_ = false;

  static constexpr size_t kLineBufSize = 96;
  char line_buf_[kLineBufSize] = {0};
  size_t line_len_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_COMMAND_ROUTER_H
