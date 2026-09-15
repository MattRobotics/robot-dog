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

// SESSION 2.2, FINDING D: `SCServo::EnableTorque()`/`writeByte()` return
// `SCS::Ack()`'s result, which is 1 on a validated ACK packet and **0**
// on any failure/timeout/no-response — NOT -1 like Ping()/readByte()/
// readWord() (see SCS.cpp: Ack() has no negative return path at all).
// Session 2.1's `safeOff()` checked `result >= 0`, which is true for
// BOTH outcomes (0 and 1) — it could never observe a failure, regardless
// of whether a servo was even present. Confirmed live: with the servo bus
// completely unpowered, `@SERVO SAFE_OFF 11` still reported "result=OK".
//
// Fixed by never trusting the write's own return value for the safety
// claim at all: safeOff() now performs the write, then always attempts a
// read-only TorqueEnable readback, and classifies purely on whether that
// readback responds and what it reports. A write ACK/NACK is informational
// only, not authoritative — the readback is the actual proof.
enum class SafeOffResult : uint8_t {
  VERIFIED_OFF            = 0,  // readback responded and confirms TorqueEnable == 0
  UNVERIFIED_NO_RESPONSE  = 1,  // no readback response at all — cannot verify anything
  VERIFY_FAILED           = 2,  // readback responded but TorqueEnable != 0
};

const char* toString(SafeOffResult result);

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
  // than the 100ms library default.
  //
  // SESSION 2.2, FINDING C: this is a DIAGNOSTIC absence-detection timeout,
  // NOT a validated operational ServoBus timeout for a future powered
  // 17-servo bus. Session 2.1 set it globally, once, in begin() — every
  // subsequent SCServo call for the rest of the session (including any
  // future register-read/telemetry call unrelated to MAINTENANCE-mode
  // scanning) would silently inherit 20ms whether that was appropriate for
  // it or not. begin() no longer touches IOTimeOut at all: it stays at the
  // library's own 100ms default (still a public SCSerial field — the
  // vendored library file is never edited) except for the exact duration of
  // a diagnostic transaction, via ScopedPingTimeout below. Every public
  // method that talks to the bus applies this guard around its own
  // transaction and lets it restore the previous value on every exit path,
  // including early returns.
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
  // remove torque, never add it, and it never touches EEPROM. Returns a
  // verified outcome (see SafeOffResult above) — never a bare bool that
  // could be mistaken for "confirmed off".
  SafeOffResult safeOff(int id);

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
  // RAII guard: saves SMS_STS::IOTimeOut (a plain public field, not a
  // vendored-library edit), sets it to the given diagnostic timeout for the
  // guard's scope, and restores the saved value on every exit path
  // (destructor runs on early `return` too, not just fall-through) — see
  // kPingTimeoutMs above for why this must never become a standing global
  // change. Deliberately not copyable/movable: exactly one guard per
  // transaction, matching "no overengineering" from the handoff.
  class ScopedPingTimeout {
   public:
    ScopedPingTimeout(SMS_STS& st, unsigned long diagnostic_timeout_ms)
        : st_(st), previous_ms_(st.IOTimeOut) {
      st_.IOTimeOut = diagnostic_timeout_ms;
    }
    ~ScopedPingTimeout() { st_.IOTimeOut = previous_ms_; }
    ScopedPingTimeout(const ScopedPingTimeout&) = delete;
    ScopedPingTimeout& operator=(const ScopedPingTimeout&) = delete;

   private:
    SMS_STS& st_;
    unsigned long previous_ms_;
  };

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
