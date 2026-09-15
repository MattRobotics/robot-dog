#include "DalyBms.h"

#include "../config/BuildConfig.h"
#include "../config/Pins.h"

namespace matdog {
namespace power {

namespace {

constexpr uint8_t kQuery[] = {
    0xD2, 0x03, 0x00, 0x00,
    0x00, 0x3E, 0xD7, 0xB9,
};

uint16_t crc16Modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

uint16_t reg16(const uint8_t* rx, size_t reg) {
  const size_t p = 3 + reg * 2;
  return (static_cast<uint16_t>(rx[p]) << 8) | rx[p + 1];
}

const char* stateName(uint16_t s) {
  switch (s) {
    case 0: return "STATIONARY";
    case 1: return "CHARGING";
    case 2: return "DISCHARGING";
    default: return "UNKNOWN";
  }
}

}  // namespace

const char* toString(DalyCommResult result) {
  switch (result) {
    case DalyCommResult::NEVER_POLLED: return "NEVER_POLLED";
    case DalyCommResult::OK:           return "OK";
    case DalyCommResult::TIMEOUT:      return "TIMEOUT";
    case DalyCommResult::CRC_FAIL:     return "CRC_FAIL";
    case DalyCommResult::BAD_HEADER:   return "BAD_HEADER";
  }
  return "UNKNOWN";
}

bool DalyBms::begin() {
  bms_uart_.begin(
      build::kDalyBusBaud,
      SERIAL_8N1,
      pins::kDalyRx,
      pins::kDalyTx);

  poll_state_ = PollState::IDLE;
  init_ = core::InitializationState::INITIALIZED;  // transport ready; detected_ stays UNKNOWN until first poll
  return true;
}

core::AvailabilityStatus DalyBms::availability() const {
  core::AvailabilityStatus a;
  a.init = init_;
  a.detected = detected_;
  a.expected = build::kBatteryAvailable ? core::ExpectedState::REQUIRED
                                          : core::ExpectedState::EXPECTED_OFFLINE;
  return a;
}

void DalyBms::sendQuery() {
  while (bms_uart_.available()) {
    bms_uart_.read();
  }
  bms_uart_.write(kQuery, sizeof(kQuery));
  bms_uart_.flush();

  rx_len_ = 0;
  request_sent_ms_ = millis();
  poll_state_ = PollState::AWAITING_RESPONSE;
}

void DalyBms::handleResponse() {
  last_result_ms_ = millis();

  if (rx_len_ != kExpectedResponseLen) {
    last_result_ = DalyCommResult::TIMEOUT;
    detected_ = core::DetectedState::NO_RESPONSE;
    return;
  }

  if (rx_buf_[0] != 0xD2 || rx_buf_[1] != 0x03 || rx_buf_[2] != 0x7C) {
    last_result_ = DalyCommResult::BAD_HEADER;
    // Bytes were received but malformed — closest fit in the 4-state
    // DetectedState model is still "not a clean detection".
    detected_ = core::DetectedState::NO_RESPONSE;
    return;
  }

  const uint16_t crc_calc = crc16Modbus(rx_buf_, rx_len_ - 2);
  const uint16_t crc_wire =
      static_cast<uint16_t>(rx_buf_[rx_len_ - 2]) |
      (static_cast<uint16_t>(rx_buf_[rx_len_ - 1]) << 8);

  if (crc_calc != crc_wire) {
    last_result_ = DalyCommResult::CRC_FAIL;
    detected_ = core::DetectedState::NO_RESPONSE;
    return;
  }

  DalySample s;
  s.valid = true;
  s.sampled_at_ms = last_result_ms_;

  s.cell_count = reg16(rx_buf_, 49);
  s.temp_count = reg16(rx_buf_, 50);

  const uint16_t cell_n = s.cell_count < 32 ? s.cell_count : 32;
  for (uint16_t i = 0; i < cell_n; ++i) {
    s.cell_mv[i] = reg16(rx_buf_, i);
  }

  const uint16_t temp_n = s.temp_count < 8 ? s.temp_count : 8;
  for (uint16_t i = 0; i < temp_n; ++i) {
    const uint16_t raw = reg16(rx_buf_, 32 + i);
    s.temp_valid[i] = (raw != 0x00FF);
    s.temp_c[i] = s.temp_valid[i] ? static_cast<int16_t>(raw) - 40 : 0;
  }

  s.pack_voltage_v = reg16(rx_buf_, 40) * 0.1f;
  s.pack_current_a = (static_cast<int32_t>(reg16(rx_buf_, 41)) - 30000) * 0.1f;
  s.soc_percent = reg16(rx_buf_, 42) * 0.1f;
  s.remaining_ah = reg16(rx_buf_, 48) * 0.1f;

  s.cell_max_mv = reg16(rx_buf_, 43);
  s.cell_min_mv = reg16(rx_buf_, 44);
  s.cell_avg_mv = reg16(rx_buf_, 55);
  s.cell_delta_mv = reg16(rx_buf_, 56);

  s.temp_max_c = static_cast<int16_t>(reg16(rx_buf_, 45)) - 40;
  s.temp_min_c = static_cast<int16_t>(reg16(rx_buf_, 46)) - 40;

  s.state_name = stateName(reg16(rx_buf_, 47));
  s.cycles = reg16(rx_buf_, 51);

  s.charge_mos_on = reg16(rx_buf_, 53) == 1;
  s.discharge_mos_on = reg16(rx_buf_, 54) == 1;

  s.alarms[0] = reg16(rx_buf_, 58);
  s.alarms[1] = reg16(rx_buf_, 59);
  s.alarms[2] = reg16(rx_buf_, 60);
  s.alarms[3] = reg16(rx_buf_, 61);

  sample_ = s;
  last_result_ = DalyCommResult::OK;
  detected_ = core::DetectedState::ONLINE;
}

void DalyBms::update(uint32_t now_ms) {
  switch (poll_state_) {
    case PollState::IDLE:
      if (now_ms - last_poll_start_ms_ >= kPollIntervalMs) {
        last_poll_start_ms_ = now_ms;
        sendQuery();
      }
      break;

    case PollState::AWAITING_RESPONSE: {
      while (bms_uart_.available() && rx_len_ < kExpectedResponseLen) {
        rx_buf_[rx_len_++] = static_cast<uint8_t>(bms_uart_.read());
      }

      const bool complete = (rx_len_ == kExpectedResponseLen);
      const bool timed_out = (now_ms - request_sent_ms_) > kResponseTimeoutMs;

      if (complete || timed_out) {
        handleResponse();
        poll_state_ = PollState::IDLE;
      }
      break;
    }
  }
}

}  // namespace power
}  // namespace matdog
