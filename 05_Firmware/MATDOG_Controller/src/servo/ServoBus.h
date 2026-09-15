#ifndef MATDOG_SERVO_SERVO_BUS_H
#define MATDOG_SERVO_SERVO_BUS_H

#include <Arduino.h>
#include <SCServo.h>

#include "../config/BuildConfig.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"

namespace matdog {
namespace servo {

// Result of a bounded ID-range scan.
struct ScanResult {
  int lo = 0;
  int hi = 0;
  int found_count = 0;
  int found_ids[64] = {0};  // capped; see kMaxScanIds
};

// Non-blocking scan progress. SESSION 2 HARDENING: Session 1's scan()
// pinged an entire ID range synchronously inside one call — each Ping()
// carries the SCServo library's own ~100ms IOTimeOut, so a 45-ID range
// with nothing responding (exactly the USB_ONLY case) could monopolize
// loop() for ~4.5s. That starves BNO085 acquisition and the USB command
// router for the whole scan. startScan()/update() replace it with a state
// machine that probes exactly one ID per update() tick, bounding any single
// loop() iteration's servo-bus blocking to one Ping() call (~100ms worst
// case) instead of the whole range.
enum class ScanState : uint8_t { IDLE, RUNNING, COMPLETE };

// Lean operational transport around the hardware-proven ST3215 / SCServo
// path (05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/matdog_servo_commissioning.ino,
// SHA256 74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd).
//
// This is NOT the gait engine and NOT the bench QC/provisioning tool.
// Preserved: GPIO17/18, 1 Mbps, SCServo transport, unsigned GoalPosition
// contract (never exercised here — no motion primitive exists in V0.1).
// Deliberately absent: EEPROM normalization, ID recoding, CalibrationOfs,
// factory reset, broadcast write, torque-on, any GoalPosition write.
class ServoBus {
 public:
  static constexpr int kMaxScanIds = 64;
  static constexpr int kMaxScanRange = 64;  // bounded: SCServo IOTimeOut=100ms/id

  bool begin();
  void update(uint32_t now_ms);
  core::ModuleHealth health() const { return core::toModuleHealth(core::classify(availability())); }
  core::AvailabilityStatus availability() const;

  // Pings a single ID. Returns true and fills model/present if it answers.
  // Bounded to one SCServo IOTimeOut (~100ms) — fine for an on-demand
  // single-ID diagnostic, unlike a range scan.
  bool ping(int id);

  // Reads the C018 model word (register 0x03). Expected value is 777 but
  // this module does not enforce that — it is a read-only diagnostic.
  bool readModel(int id, int* model_out);

  // Starts a non-blocking scan across [lo, hi] (clamped to 0..253 and to
  // kMaxScanRange). Returns false without effect if a scan is already
  // RUNNING or the range is invalid. Call update() every loop tick to
  // advance it; poll scanState()/lastScanResult() for progress/outcome.
  bool startScan(int lo, int hi);
  ScanState scanState() const { return scan_state_; }
  const ScanResult& lastScanResult() const { return scan_result_; }

  // TorqueEnable = 0. The only servo write exposed in V0.1: it can only
  // remove torque, never add it, and it never touches EEPROM.
  bool safeOff(int id);

  // Read-only runtime snapshot (present position/speed/load/voltage/temp).
  // Returns false if the servo does not answer within the bounded timeout.
  struct RuntimeState {
    int present_position = -1;
    int present_speed = -1;
    int present_load = -1;
    int present_voltage = -1;
    int present_temperature = -1;
    int torque_enable = -1;
  };
  bool readRuntimeState(int id, RuntimeState* out);

 private:
  HardwareSerial servo_uart_{1};  // matches frozen bench source's UART index.
  SMS_STS st_;
  core::InitializationState init_ = core::InitializationState::NOT_INITIALIZED;
  core::DetectedState last_detected_ = core::DetectedState::UNKNOWN;

  ScanState scan_state_ = ScanState::IDLE;
  int scan_next_id_ = 0;
  ScanResult scan_result_;
};

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_BUS_H
