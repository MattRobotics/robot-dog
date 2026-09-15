#ifndef MATDOG_POWER_DALY_BMS_H
#define MATDOG_POWER_DALY_BMS_H

#include <Arduino.h>

#include "../config/BuildConfig.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"

namespace matdog {
namespace power {

enum class DalyCommResult : uint8_t {
  NEVER_POLLED = 0,
  OK           = 1,
  TIMEOUT      = 2,
  CRC_FAIL     = 3,
  BAD_HEADER   = 4,
};

const char* toString(DalyCommResult result);

struct DalySample {
  bool valid = false;
  uint32_t sampled_at_ms = 0;

  float pack_voltage_v = 0;
  float pack_current_a = 0;
  float soc_percent = 0;
  float remaining_ah = 0;

  uint16_t cell_count = 0;
  uint16_t cell_mv[32] = {0};
  uint16_t cell_max_mv = 0;
  uint16_t cell_min_mv = 0;
  uint16_t cell_avg_mv = 0;
  uint16_t cell_delta_mv = 0;

  uint16_t temp_count = 0;
  int16_t temp_c[8] = {0};
  bool temp_valid[8] = {false};
  int16_t temp_max_c = 0;
  int16_t temp_min_c = 0;

  const char* state_name = "UNKNOWN";
  uint16_t cycles = 0;
  bool charge_mos_on = false;
  bool discharge_mos_on = false;
  uint16_t alarms[4] = {0, 0, 0, 0};
};

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

 private:
  enum class PollState : uint8_t { IDLE, AWAITING_RESPONSE };

  void sendQuery();
  void handleResponse();

  HardwareSerial bms_uart_{2};
  core::InitializationState init_ = core::InitializationState::NOT_INITIALIZED;
  core::DetectedState detected_ = core::DetectedState::UNKNOWN;

  PollState poll_state_ = PollState::IDLE;
  uint32_t request_sent_ms_ = 0;
  uint32_t last_poll_start_ms_ = 0;

  static constexpr uint32_t kPollIntervalMs = 2000;
  static constexpr uint32_t kResponseTimeoutMs = 750;
  static constexpr size_t kExpectedResponseLen = 129;

  uint8_t rx_buf_[kExpectedResponseLen] = {0};
  size_t rx_len_ = 0;

  DalyCommResult last_result_ = DalyCommResult::NEVER_POLLED;
  uint32_t last_result_ms_ = 0;

  DalySample sample_;
};

}  // namespace power
}  // namespace matdog

#endif  // MATDOG_POWER_DALY_BMS_H
