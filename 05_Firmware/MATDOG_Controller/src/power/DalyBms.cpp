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
  if (!bus_.requestKeyConfigRead()) {
    return false;
  }
  key_read_result_ = DalyKeyReadResult::PENDING;
  key_read_rx_bytes_ = 0;
  return true;
}

// The one point where bytes leave for the BMS. The frame is chosen from the
// two constant FC03 reads by enum; scripts/static_audit.py requires this to
// be the only bms_uart_ write in the firmware.
void DalyBms::startTransaction(DalyRequest request) {
  while (bms_uart_.available()) {
    bms_uart_.read();
  }
  bms_uart_.write(dalyRequestFrame(request), kDalyRequestLen);
  bms_uart_.flush();

  rx_len_ = 0;
  bus_.markSent(millis());
}

void DalyBms::finishTransaction() {
  if (bus_.inFlightRequest() == DalyRequest::KEY_CONFIG) {
    handleKeyConfigResponse();
  } else {
    handleTelemetryResponse();
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
}

void DalyBms::update(uint32_t now_ms) {
  if (!bus_.inFlight()) {
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
  }
}

}  // namespace power
}  // namespace matdog
