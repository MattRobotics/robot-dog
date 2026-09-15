#ifndef MATDOG_SERVO_SERVO_BUS_H
#define MATDOG_SERVO_SERVO_BUS_H

#include <Arduino.h>
#include <SCServo.h>

#include "../config/BuildConfig.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"

namespace matdog {
namespace servo {

// Result of a bounded ID-range scan, including the measured latency
// evidence Session 2.1 requires: how long the scan actually took, and the
// single slowest per-ID probe observed (the number that matters for "how
// long could one loop() iteration have blocked").
struct ScanResult {
  int lo = 0;
  int hi = 0;
  int found_count = 0;
  int found_ids[64] = {0};  // capped; see kMaxScanIds
  uint32_t elapsed_ms = 0;
  uint32_t max_ping_us = 0;
};

// Incremental scan progress. SESSION 2 HARDENING, CORRECTED IN SESSION 2.1:
// Session 1's scan() pinged an entire ID range synchronously inside one
// call — each Ping() carries the SCServo library's own IOTimeOut, so a
// 45-ID range with nothing responding (exactly the USB_ONLY case) could
// monopolize loop() for the full range's worth of timeouts. startScan()/
// update() replace it with a state machine that probes exactly one ID per
// update() tick, bounding any single loop() iteration's servo-bus blocking
// to one Ping() call instead of the whole range.
//
// Session 2 called this "non-blocking". That was imprecise and has been
// corrected: SCServo::Ping() is still a synchronous call with its own
// bounded internal timeout (see kPingTimeoutMs below) — one update() tick
// CAN still block the caller for up to that long. The accurate description
// is "incremental scan with bounded per-ID blocking", not non-blocking.
// Because of that, @SERVO SCAN/@SERVO READ are gated to
// core::OperatingMode::MAINTENANCE (see CommandRouter) and must never be
// reachable from a future deterministic motion RUN loop.
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
  static constexpr int kMaxScanRange = 64;  // bounded regardless of per-ID timeout

  // SCServo's own default IOTimeOut is 100ms (SCSerial.cpp), a generic
  // library-wide safety margin, not a MATDOG/ST3215-specific figure. Real
  // hardware measurement exists for this exact servo/bus combination: the
  // NEW01 characterization campaign (09_Logs/Validation_Reports/
  // ST3215_Provisioning_2026-08-27/characterization_sessions/), timing a
  // live powered ST3215 at 1 Mbaud with micros(), recorded Ping+register-read
  // round trips clustering at 593-620us (hundreds of samples) and a
  // dedicated register-read timing of 334-358us. kPingTimeoutMs below is set
  // to roughly 32x that measured worst case (20ms vs ~0.62ms) — enough
  // margin for a different bench topology (this session's bus runs through
  // the Seeed driver, not the characterization rig's point-to-point wiring)
  // while still bounding the USB_ONLY no-response worst case far tighter
  // than the 100ms library default. SCSerial::IOTimeOut (SCSerial.h) is
  // compared against millis()-measured elapsed time inside the library, so
  // this constant is in milliseconds, matching that unit exactly. It is a
  // public field — setting it in begin() does not modify the vendored
  // library file.
  static constexpr unsigned long kPingTimeoutMs = 20;

  bool begin();
  void update(uint32_t now_ms);
  core::ModuleHealth health() const { return core::toModuleHealth(core::classify(availability())); }
  core::AvailabilityStatus availability() const;

  // Pings a single ID. Returns true and fills model/present if it answers.
  // Bounded to one SCServo IOTimeOut (kPingTimeoutMs) — fine for an
  // on-demand single-ID diagnostic, unlike a range scan. Like all servo
  // diagnostics that can block for that long, only meaningful/reachable
  // during core::OperatingMode::MAINTENANCE.
  bool ping(int id);

  // Reads the C018 model word (register 0x03). Expected value is 777 but
  // this module does not enforce that — it is a read-only diagnostic.
  bool readModel(int id, int* model_out);

  // Starts an incremental scan across [lo, hi] (clamped to 0..253 and to
  // kMaxScanRange): bounded per-ID blocking (see kPingTimeoutMs), not
  // non-blocking — see the ScanState comment above. Returns false without
  // effect if a scan is already RUNNING or the range is invalid. Call
  // update() every loop tick to advance it; poll scanState()/
  // lastScanResult() for progress/outcome.
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
  uint32_t scan_started_ms_ = 0;
};

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_BUS_H
