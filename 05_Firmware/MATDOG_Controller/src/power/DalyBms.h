#ifndef MATDOG_POWER_DALY_BMS_H
#define MATDOG_POWER_DALY_BMS_H

#include <Arduino.h>

#include "../config/BuildConfig.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"
#include "DalyProtocol.h"

namespace matdog {
namespace power {

// Read-only DALY Smart K-Series telemetry over the XY-017 RS485 bridge.
//
// Source: ~/MATDOG/runtime/esp32/matdog_daly_rs485_probe_v2/matdog_daly_rs485_probe_v2.ino
// (SHA256 4fd7fd3982ab57376ad9a5ade7ed497fb2515234465612a59aac22f756acb81c),
// selected over v1 because v2 is the CRC-validated decoder that actually
// produced the sane battery values recorded in the handoff (cell ~3.78 V,
// pack ~11.3 V, SOC ~49.8%, MOS ON) — v1 only dumped raw hex.
//
// Deliberately DIFFERENT from both probes: this module never busy-waits.
// The probes block for up to 750 ms per poll inside loop(); doing that here
// would stall BNO085 acquisition and the USB command router for the same
// window (handoff section 20 forbids exactly this). update() instead runs a
// small non-blocking state machine: send, then accumulate available bytes
// across however many update() calls it takes, bounded by the same 750 ms
// deadline measured from send time.
//
// Deliberately DIFFERENT from both probes: HardwareSerial index. Both
// standalone probes used HardwareSerial(1), the same peripheral ServoBus
// uses. Running both simultaneously would collide on one UART controller,
// so this module binds to HardwareSerial(2) instead — same GPIO15/16 pins,
// same 9600 8N1 framing, same Modbus request/response bytes.
//
// V0.1 is READ-ONLY. See requestDischargeOff() below.
//
// DALY KEY probe (2026-09-19): the same bus also carries an operator-
// triggered one-shot read of the 0x81 parameter personality (KEY logic,
// charge/discharge MOS control, sleep time). Both transactions are FC03
// reads of constant frames (power/DalyProtocol.h); DalyBusScheduler is the
// single owner that starts them, so they can never overlap on the bus. The
// probe's outcome is kept apart from telemetry health: a silent 0x81
// address never marks the BMS absent.
class DalyBms {
 public:
  bool begin();
  void update(uint32_t now_ms);
  core::ModuleHealth health() const { return core::toModuleHealth(core::classify(availability())); }
  core::AvailabilityStatus availability() const;

  DalyCommResult lastCommResult() const { return last_result_; }
  uint32_t lastResultAgeMs(uint32_t now_ms) const { return now_ms - last_result_ms_; }
  bool hasValidSample() const { return sample_.valid; }
  const DalySample& sample() const { return sample_; }

  // BLOCKED per handoff 8A.8 / CLAUDE_CODE_PROMPT "DALY write is BLOCKED
  // until verified". The specific K-Series write protocol (opcode, framing,
  // ACK semantics, KEY-vs-MOS precedence) has not been identified or bench
  // verified. This method transmits NOTHING and always returns false. It
  // exists only so the power-state machine has a call site to wire up once
  // the protocol is positively identified — do not implement the body
  // without a verified command evidence chain.
  bool requestDischargeOff() { return false; }

  // One-shot read-only KEY/parameter probe (0x81 FC03 0x0100 x 0x0078).
  // Queues the read for the next idle bus boundary; false while one is
  // already queued or in flight. Nothing here can write the BMS.
  bool requestKeyConfigRead();
  DalyKeyReadResult keyConfigReadResult() const { return key_read_result_; }
  uint32_t keyConfigReadAgeMs(uint32_t now_ms) const { return now_ms - key_read_result_ms_; }
  // Bytes received by the last completed KEY read (0 = silence).
  size_t keyConfigReadRxBytes() const { return key_read_rx_bytes_; }
  // Last VALID snapshot; a failed read never replaces it.
  const DalyKeyConfigSnapshot& keyConfigSnapshot() const { return key_snapshot_; }

 private:
  void startTransaction(DalyRequest request);
  void finishTransaction();
  void handleTelemetryResponse();
  void handleKeyConfigResponse();

  HardwareSerial bms_uart_{2};
  core::InitializationState init_ = core::InitializationState::NOT_INITIALIZED;
  core::DetectedState detected_ = core::DetectedState::UNKNOWN;

  DalyBusScheduler bus_;

  uint8_t rx_buf_[kDalyMaxResponseLen] = {0};
  size_t rx_len_ = 0;

  DalyCommResult last_result_ = DalyCommResult::NEVER_POLLED;
  uint32_t last_result_ms_ = 0;

  DalyKeyReadResult key_read_result_ = DalyKeyReadResult::NOT_REQUESTED;
  uint32_t key_read_result_ms_ = 0;
  size_t key_read_rx_bytes_ = 0;
  DalyKeyConfigSnapshot key_snapshot_;

  DalySample sample_;
};

}  // namespace power
}  // namespace matdog

#endif  // MATDOG_POWER_DALY_BMS_H
