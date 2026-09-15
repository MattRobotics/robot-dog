#ifndef MATDOG_SERVO_SERVO_BUS_H
#define MATDOG_SERVO_SERVO_BUS_H

#include <Arduino.h>
#include <SCServo.h>

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
  core::ModuleHealth health() const { return health_; }

  // Pings a single ID. Returns true and fills model/present if it answers.
  bool ping(int id);

  // Reads the C018 model word (register 0x03). Expected value is 777 but
  // this module does not enforce that — it is a read-only diagnostic.
  bool readModel(int id, int* model_out);

  // Bounded scan across [lo, hi]. Range is clamped to kMaxScanRange and IDs
  // to 0..253. This is an explicit, operator-triggered diagnostic call, not
  // an automatic/periodic one.
  ScanResult scan(int lo, int hi);

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
  core::ModuleHealth health_ = core::ModuleHealth::NOT_INITIALIZED;
};

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_BUS_H
