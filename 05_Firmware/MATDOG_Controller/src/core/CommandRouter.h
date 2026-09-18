#ifndef MATDOG_CORE_COMMAND_ROUTER_H
#define MATDOG_CORE_COMMAND_ROUTER_H

#include <Arduino.h>

#include "../imu/Bno085Imu.h"
#include "../power/DalyBms.h"
#include "../servo/ServoBus.h"
#include "../servo/ServoCensus.h"
#include "../status/LedRing.h"
#include "Availability.h"
#include "OperatingMode.h"
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
    servo::ServoCensus* servo_census;
    imu::Bno085Imu* imu;
    power::DalyBms* daly;
    status::LedRing* led;
    SystemState* system_state;
    PowerStateMachine* power_state;
    OperatingModeManager* operating_mode;
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
  // Pure presentation of servo_census->result(). Computes nothing: the
  // classification lives in servo/ServoPopulation.h so a future Web UI /
  // HostLink adapter can render the same structured result without
  // reimplementing it or re-scanning the bus (handoff sections 7/8/9).
  void printServoCensusResult();
  void printServoRead(int id);
  void printServoSafeOff(int id);
  void printModeStatus();
  static void printAvailabilityLine(const char* label, const AvailabilityStatus& a);

  Modules modules_{};
  bool bms_stream_enabled_ = false;
  uint32_t last_bms_stream_ms_ = 0;
  bool servo_scan_result_pending_ = false;
  bool servo_census_result_pending_ = false;

  static constexpr size_t kLineBufSize = 96;
  char line_buf_[kLineBufSize] = {0};
  size_t line_len_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_COMMAND_ROUTER_H
