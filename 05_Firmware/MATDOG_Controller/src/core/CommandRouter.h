#ifndef MATDOG_CORE_COMMAND_ROUTER_H
#define MATDOG_CORE_COMMAND_ROUTER_H

#include <Arduino.h>

#include "../calibration/CalibrationManager.h"
#include "../imu/Bno085Imu.h"
#include "../network/WifiManager.h"
#include "../power/DalyBms.h"
#include "../update/OtaManager.h"
#include "../servo/ServoBus.h"
#include "../servo/ServoCensus.h"
#include "../servo/ServoPreflight.h"
#include "../status/LedRing.h"
#include "../status/LedStatusManager.h"
#include "ActuatorAuthority.h"
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
    servo::ServoPreflight* servo_preflight;
    imu::Bno085Imu* imu;
    power::DalyBms* daly;
    status::LedRing* led;
    status::LedStatusManager* led_status;
    network::WifiManager* wifi;
    update::OtaManager* ota;
    SystemState* system_state;
    PowerStateMachine* power_state;
    OperatingModeManager* operating_mode;
    // Pointer to the ONE arbiter owned by Controller. Never a copy of its
    // state: the router asks, it does not remember.
    ActuatorAuthorityArbiter* authority;
    calibration::CalibrationManager* calibration;
  };

  void begin(const Modules& modules);
  void update(uint32_t now_ms);

  // Used by the OTA first-boot self-check: "is the command surface usable?"
  // is one of the software-only conditions that must hold before a freshly
  // booted OTA image is allowed to confirm itself.
  bool bound() const { return modules_.system_state != nullptr; }

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
  // Pure presentation of ota->status(). Read-only: OTA-A ships no transport
  // and no command that can start an update.
  void printOtaStatus();
  // Read-only build/source identity (G4 SOURCE_SIGNATURE). Presents facts
  // already computed elsewhere (build::kBuildId, the OTA manager's running-
  // image bookkeeping, the ESP-IDF running partition) — no new hardware
  // read, no new bus traffic. Deliberately does NOT surface esp_app_desc_t:
  // in an Arduino-ESP32 build it describes arduino-lib-builder, not MATDOG
  // (see update/OtaPolicy.h), so it is not authoritative identity here
  // either.
  void printSourceSignature();
  void printServoScanResult();
  // Pure presentation of servo_census->result(). Computes nothing: the
  // classification lives in servo/ServoPopulation.h so a future Web UI /
  // HostLink adapter can render the same structured result without
  // reimplementing it or re-scanning the bus (handoff sections 7/8/9).
  void printServoCensusResult();

  // Pure presentation of servo_preflight->result(). Computes nothing and
  // issues no bus transaction: the service already did the reading.
  void printServoPreflightResult();
  void printServoRead(int id);
  void printServoSafeOff(int id);
  void printModeStatus();
  // Read-only presentation of the central arbiter. There is deliberately no
  // command that acquires or releases authority: no write-capable owner
  // exists yet, and an operator-driven acquire would be a write path this
  // phase explicitly does not add.
  void printAuthorityStatus();
  // Read-only presentation of the calibration manager. There is deliberately
  // no command that starts, runs or moves anything: no write path exists, and
  // the repository declares hardware motion unauthorized.
  void printCalibrationStatus();
  static void printAvailabilityLine(const char* label, const AvailabilityStatus& a);

  Modules modules_{};
  bool bms_stream_enabled_ = false;
  uint32_t last_bms_stream_ms_ = 0;
  bool bms_key_read_result_pending_ = false;
  bool bms_key_write_pending_ = false;
  bool bms_key_write_ack_reported_ = false;
  bool servo_scan_result_pending_ = false;
  bool servo_census_result_pending_ = false;
  bool servo_preflight_result_pending_ = false;

  static constexpr size_t kLineBufSize = 96;
  char line_buf_[kLineBufSize] = {0};
  size_t line_len_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_COMMAND_ROUTER_H
