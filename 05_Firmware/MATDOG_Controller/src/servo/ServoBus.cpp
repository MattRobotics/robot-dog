#include "ServoBus.h"

#include "../config/Pins.h"

namespace matdog {
namespace servo {

bool ServoBus::begin() {
  servo_uart_.begin(
      build::kServoBusBaud,
      SERIAL_8N1,
      pins::kServoRx,
      pins::kServoTx);

  st_.pSerial = &servo_uart_;

  // Public SCSerial field, not a vendored-library edit — see kPingTimeoutMs
  // in ServoBus.h for the hardware-measured justification.
  st_.IOTimeOut = ServoBus::kPingTimeoutMs;

  // No automatic ping/scan/torque on boot — matches the frozen bench
  // source's own stated invariant ("Automatic ping : DISABLED") and the
  // handoff's "no automatic motion at boot" rule. detected_ therefore
  // stays UNKNOWN until an explicit @SERVO SCAN/READ runs.
  init_ = core::InitializationState::INITIALIZED;
  return true;
}

core::AvailabilityStatus ServoBus::availability() const {
  core::AvailabilityStatus a;
  a.init = init_;
  a.detected = last_detected_;
  a.expected = build::kServoPowerAvailable ? core::ExpectedState::REQUIRED
                                             : core::ExpectedState::EXPECTED_OFFLINE;
  return a;
}

bool ServoBus::ping(int id) {
  if (id < 0 || id > 253) return false;
  int result = st_.Ping(static_cast<uint8_t>(id));
  last_detected_ = (result >= 0) ? core::DetectedState::ONLINE : core::DetectedState::NO_RESPONSE;
  return result >= 0;
}

bool ServoBus::readModel(int id, int* model_out) {
  if (id < 0 || id > 253 || model_out == nullptr) return false;
  int model = st_.readWord(static_cast<uint8_t>(id), SMS_STS_MODEL_L);
  if (model < 0) return false;
  *model_out = model;
  return true;
}

bool ServoBus::startScan(int lo, int hi) {
  if (scan_state_ == ScanState::RUNNING) return false;

  if (lo > hi) {
    int tmp = lo;
    lo = hi;
    hi = tmp;
  }
  lo = constrain(lo, 0, 253);
  hi = constrain(hi, 0, 253);
  if (hi - lo + 1 > kMaxScanRange) {
    hi = lo + kMaxScanRange - 1;
  }

  scan_result_ = ScanResult{};
  scan_result_.lo = lo;
  scan_result_.hi = hi;
  scan_next_id_ = lo;
  scan_state_ = ScanState::RUNNING;
  scan_started_ms_ = millis();
  return true;
}

void ServoBus::update(uint32_t now_ms) {
  (void)now_ms;
  if (scan_state_ != ScanState::RUNNING) return;

  // Exactly one Ping() per tick — see the ScanState comment in ServoBus.h
  // for why this is "incremental with bounded per-ID blocking", not
  // non-blocking. Measures its own duration (micros()) so the actual
  // per-ID and total cost is evidence, not a claim — see
  // ScanResult::max_ping_us / elapsed_ms.
  const int id = scan_next_id_;
  const uint32_t ping_start_us = micros();
  const bool responded = st_.Ping(static_cast<uint8_t>(id)) >= 0;
  const uint32_t ping_us = micros() - ping_start_us;
  if (ping_us > scan_result_.max_ping_us) {
    scan_result_.max_ping_us = ping_us;
  }

  if (responded) {
    if (scan_result_.found_count < kMaxScanIds) {
      scan_result_.found_ids[scan_result_.found_count] = id;
    }
    scan_result_.found_count++;
  }

  scan_next_id_++;
  if (scan_next_id_ > scan_result_.hi) {
    scan_state_ = ScanState::COMPLETE;
    scan_result_.elapsed_ms = millis() - scan_started_ms_;
    last_detected_ = (scan_result_.found_count > 0) ? core::DetectedState::ONLINE
                                                       : core::DetectedState::NO_RESPONSE;
  }
}

bool ServoBus::safeOff(int id) {
  if (id < 0 || id > 253) return false;
  int result = st_.EnableTorque(static_cast<uint8_t>(id), 0);
  return result >= 0;
}

bool ServoBus::readRuntimeState(int id, RuntimeState* out) {
  if (id < 0 || id > 253 || out == nullptr) return false;

  int ping = st_.Ping(static_cast<uint8_t>(id));
  if (ping < 0) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }
  last_detected_ = core::DetectedState::ONLINE;

  out->present_position    = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_POSITION_L);
  out->present_speed       = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_SPEED_L);
  out->present_load        = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_LOAD_L);
  out->present_voltage     = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_VOLTAGE);
  out->present_temperature = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_TEMPERATURE);
  out->torque_enable       = st_.readByte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE);

  return true;
}

}  // namespace servo
}  // namespace matdog
