#include "ServoBus.h"

#include "../config/Pins.h"

namespace matdog {
namespace servo {

const char* toString(SafeOffResult result) {
  switch (result) {
    case SafeOffResult::VERIFIED_OFF:           return "VERIFIED_OFF";
    case SafeOffResult::UNVERIFIED_NO_RESPONSE: return "UNVERIFIED_NO_RESPONSE";
    case SafeOffResult::VERIFY_FAILED:          return "VERIFY_FAILED";
  }
  return "UNKNOWN";
}

bool ServoBus::begin() {
  servo_uart_.begin(
      build::kServoBusBaud,
      SERIAL_8N1,
      pins::kServoRx,
      pins::kServoTx);

  st_.pSerial = &servo_uart_;

  // IOTimeOut is deliberately NOT touched here — see kDiagnosticTimeoutMs/
  // kOperationalTimeoutMs and ScopedIOTimeout in ServoBus.h (Session 2.2
  // Finding C, refined Session 2.3 Finding 1). Every method below names
  // its own timeout explicitly via the guard; nothing relies on whatever
  // IOTimeOut happens to already be set to.

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
  ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);
  int result = st_.Ping(static_cast<uint8_t>(id));
  last_detected_ = (result >= 0) ? core::DetectedState::ONLINE : core::DetectedState::NO_RESPONSE;
  return result >= 0;
}

bool ServoBus::readModel(int id, int* model_out) {
  if (id < 0 || id > 253 || model_out == nullptr) return false;
  ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);
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
  // ScanResult::max_ping_us / elapsed_ms. ScopedIOTimeout bounds this
  // single Ping() to the diagnostic timeout and restores the previous
  // value immediately after, every tick.
  const int id = scan_next_id_;
  const uint32_t ping_start_us = micros();
  bool responded;
  {
    ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);
    responded = st_.Ping(static_cast<uint8_t>(id)) >= 0;
  }
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

SafeOffResult ServoBus::safeOff(int id) {
  if (id < 0 || id > 253) return SafeOffResult::UNVERIFIED_NO_RESPONSE;

  // kOperationalTimeoutMs, NOT kDiagnosticTimeoutMs (Session 2.3 Finding 1)
  // — this is the safety de-escalation path, reachable from any
  // OperatingMode, and must not inherit a timeout tuned for
  // MAINTENANCE-only absence detection.
  ScopedIOTimeout guard(st_, kOperationalTimeoutMs);

  // The write's own ACK (SCS::Ack(), via EnableTorque -> writeByte) is
  // informational only — see the SafeOffResult comment in ServoBus.h for
  // why it must never be the safety claim by itself. Issue it, then always
  // verify with an independent, read-only TorqueEnable readback regardless
  // of what the write's ACK reported.
  st_.EnableTorque(static_cast<uint8_t>(id), 0);

  int torque_enable = st_.readByte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE);

  if (torque_enable < 0) {
    // No readback response at all: we cannot prove anything, so we must
    // not claim success — regardless of whether the write itself ACKed.
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return SafeOffResult::UNVERIFIED_NO_RESPONSE;
  }

  last_detected_ = core::DetectedState::ONLINE;  // readback proves the servo is present

  return (torque_enable == 0) ? SafeOffResult::VERIFIED_OFF : SafeOffResult::VERIFY_FAILED;
}

bool ServoBus::readRuntimeState(int id, RuntimeState* out) {
  if (id < 0 || id > 253 || out == nullptr) return false;

  // kOperationalTimeoutMs, NOT kDiagnosticTimeoutMs (Session 2.3 Finding 1)
  // — a future motion controller will naturally reuse this for operational
  // state reads; it must not silently inherit a timeout tuned for
  // MAINTENANCE-only absence detection just because @SERVO READ happens to
  // be MAINTENANCE-gated today.
  ScopedIOTimeout guard(st_, kOperationalTimeoutMs);

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
