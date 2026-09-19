#include "DalyBms.h"

#include "../config/BuildConfig.h"
#include "../config/Pins.h"

namespace matdog {
namespace power {

bool DalyBms::begin() {
  bms_uart_.begin(
      build::kDalyBusBaud,
      SERIAL_8N1,
      pins::kDalyRx,
      pins::kDalyTx);

  init_ = core::InitializationState::INITIALIZED;  // transport ready; detected_ stays UNKNOWN until first poll
  return true;
}

core::AvailabilityStatus DalyBms::availability() const {
  core::AvailabilityStatus a;
  a.init = init_;
  a.detected = detected_;
  // G2: derived from the active hardware profile (core/Availability.h).
  a.expected = core::expectedStateForBattery(build::kProfileExpectations);
  return a;
}

bool DalyBms::requestKeyConfigRead() {
  if (key_write_.outstanding()) {
    return false;  // the write's own read-back owns the next KEY read
  }
  return queueKeyConfigRead();
}

bool DalyBms::queueKeyConfigRead() {
  if (!bus_.requestKeyConfigRead()) {
    return false;
  }
  key_read_result_ = DalyKeyReadResult::PENDING;
  key_read_rx_bytes_ = 0;
  return true;
}

DalyKeyWriteInputs DalyBms::keyWriteInputs(bool maintenance_mode, bool bus_busy,
                                           uint32_t now_ms) const {
  DalyKeyWriteInputs in;
  in.maintenance_mode = maintenance_mode;
  in.write_already_attempted = key_write_.status().transmitted;
  in.bus_busy = bus_busy;
  in.telemetry_ok = last_result_ == DalyCommResult::OK && sample_.valid;
  in.telemetry_age_ms = now_ms - last_result_ms_;
  in.alarms_clear = sample_.alarms[0] == 0 && sample_.alarms[1] == 0 &&
                    sample_.alarms[2] == 0 && sample_.alarms[3] == 0;
  in.last_key_read_ok = key_read_result_ == DalyKeyReadResult::OK;
  in.snapshot = key_snapshot_;
  in.now_ms = now_ms;
  return in;
}

DalyKeyWriteGate DalyBms::requestKeyLogicDischarge(core::OperatingMode mode) {
  // Any transaction in flight or queued - including telemetry - refuses:
  // the preconditions must describe the bus the write will actually use.
  const bool bus_busy = bus_.inFlight() || bus_.operatorTransactionOutstanding() ||
                        key_write_.outstanding();
  DalyKeyWriteGate gate = evaluateDalyKeyWrite(
      keyWriteInputs(mode == core::OperatingMode::MAINTENANCE, bus_busy, millis()));

  if (gate.decision == DalyKeyWriteDecision::START && !bus_.requestKeyLogicDischargeWrite()) {
    gate.decision = DalyKeyWriteDecision::REFUSE;  // unreachable when idle; fail closed
    gate.refusal = DalyKeyWriteRefusal::BUS_BUSY;
  }

  switch (gate.decision) {
    case DalyKeyWriteDecision::START:              key_write_.accept(); break;
    case DalyKeyWriteDecision::ALREADY_CONFIGURED: key_write_.alreadyConfigured(); break;
    case DalyKeyWriteDecision::REFUSE:             key_write_.refuse(gate.refusal); break;
  }
  return gate;
}

// The one point where bytes leave for the BMS. The frame is chosen by enum
// from the three constants in DalyProtocol.h (two FC03 reads, the one FC06
// KEY write); scripts/static_audit.py requires this to be the only
// bms_uart_ write in the firmware.
void DalyBms::startTransaction(DalyRequest request) {
  while (bms_uart_.available()) {
    bms_uart_.read();
  }
  if (request == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE) {
    key_write_.markTransmitted();  // before the bytes: a power loss mid-frame still counts
  }
  bms_uart_.write(dalyRequestFrame(request), kDalyRequestLen);
  bms_uart_.flush();

  rx_len_ = 0;
  bus_.markSent(millis());
}

void DalyBms::finishTransaction() {
  switch (bus_.inFlightRequest()) {
    case DalyRequest::KEY_CONFIG:                handleKeyConfigResponse(); break;
    case DalyRequest::KEY_LOGIC_DISCHARGE_WRITE: handleKeyLogicWriteResponse(); break;
    case DalyRequest::TELEMETRY:                 handleTelemetryResponse(); break;
  }
}

void DalyBms::handleTelemetryResponse() {
  last_result_ms_ = millis();
  last_result_ = parseDalyTelemetry(rx_buf_, rx_len_, last_result_ms_, &sample_);
  // Bytes received but malformed are still "not a clean detection" - the
  // closest fit in the 4-state DetectedState model.
  detected_ = last_result_ == DalyCommResult::OK ? core::DetectedState::ONLINE
                                                 : core::DetectedState::NO_RESPONSE;
}

void DalyBms::handleKeyConfigResponse() {
  key_read_result_ms_ = millis();
  key_read_rx_bytes_ = rx_len_;
  key_read_result_ = toKeyReadResult(
      parseDalyKeyConfig(rx_buf_, rx_len_, key_read_result_ms_, &key_snapshot_));

  if (key_write_readback_running_) {
    key_write_readback_running_ = false;
    key_write_.onReadback(key_read_result_, key_snapshot_.key_logic_raw, key_read_result_ms_);
  }
}

void DalyBms::handleKeyLogicWriteResponse() {
  key_write_.onAck(parseDalyKeyLogicWriteAck(rx_buf_, rx_len_), rx_len_, millis());
  // Whatever the acknowledgement said, read the register back: a timed-out
  // write may still have been applied, and an OK one is not yet verified.
  key_write_readback_due_ = true;
}

void DalyBms::update(uint32_t now_ms) {
  if (!bus_.inFlight()) {
    // Last look before the one write can leave: every DALY-state
    // precondition again (the mode was checked when it was accepted).
    DalyRequest queued = DalyRequest::TELEMETRY;
    if (bus_.queuedOperatorRequest(&queued) &&
        queued == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE) {
      const DalyKeyWriteGate gate = evaluateDalyKeyWrite(keyWriteInputs(true, false, now_ms));
      if (gate.decision != DalyKeyWriteDecision::START) {
        bus_.cancelQueuedOperatorRequest();
        key_write_.cancelBeforeTransmit(gate);
      }
    }

    DalyRequest request = DalyRequest::TELEMETRY;
    if (bus_.startNext(now_ms, &request)) {
      startTransaction(request);
    }
    return;
  }

  const size_t expected_len = dalyResponseLen(bus_.inFlightRequest());
  while (bms_uart_.available() && rx_len_ < expected_len) {
    rx_buf_[rx_len_++] = static_cast<uint8_t>(bms_uart_.read());
  }

  const bool complete = (rx_len_ == expected_len);
  if (complete || bus_.deadlinePassed(now_ms)) {
    finishTransaction();
    bus_.finish(now_ms);

    if (key_write_readback_due_) {
      key_write_readback_due_ = false;
      if (queueKeyConfigRead()) {
        key_write_readback_running_ = true;
      } else {
        key_write_.onReadback(DalyKeyReadResult::TIMEOUT, 0, millis());  // cannot queue: fail closed
      }
    }
  }
}

}  // namespace power
}  // namespace matdog
