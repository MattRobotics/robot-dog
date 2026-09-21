#ifndef MATDOG_CORE_COMMAND_ROUTER_H
#define MATDOG_CORE_COMMAND_ROUTER_H

#include <Arduino.h>

#include "../imu/Bno085Imu.h"
#include "../network/WifiManager.h"
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
// unrestricted GoalPosition, or any DALY write other than the ONE semantic
// KEY configuration (@BMS KEY SET DISCHARGE CONFIRM: 0x0120 := 0x005A). The
// DALY KEY commands are fixed strings with no arguments: no register,
// address or value input.
class CommandRouter {
 public:
  struct Modules {
    servo::ServoBus* servo_bus;
    servo::ServoCensus* servo_census;
    imu::Bno085Imu* imu;
    power::DalyBms* daly;
    status::LedRing* led;
    network::WifiManager* wifi;
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
  void printBmsKeyReadResult();
  void printBmsKeyStatus();
  static void printBmsKeySnapshot(const power::DalyKeyConfigSnapshot& k);
  void printBmsKeyWriteResult();
  void printBmsKeyWriteStatus();
  void printLedStatus();
  // Pure presentation of wifi->status(). Formats a snapshot the Wi-Fi layer
  // already computed; it never queries the radio, so a future Web adapter
  // renders the same struct without a second hardware path (see
  // ARCHITECTURE.md, telemetry snapshot model).
  void printWifiStatus();
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
  bool bms_key_read_result_pending_ = false;
  bool bms_key_write_pending_ = false;
  bool bms_key_write_ack_reported_ = false;
  bool servo_scan_result_pending_ = false;
  bool servo_census_result_pending_ = false;

  static constexpr size_t kLineBufSize = 96;
  char line_buf_[kLineBufSize] = {0};
  size_t line_len_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_COMMAND_ROUTER_H
