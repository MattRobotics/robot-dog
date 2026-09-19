#include "DalyProtocol.h"

namespace matdog {
namespace power {

namespace {

// Telemetry registers are numbered from 0x0000, so the 0xD2 decoder keeps
// the register indices it was validated with (G3).
uint16_t reg16(const uint8_t* rx, uint16_t reg) {
  return dalyRegister(rx, kDalyTelemetryStartReg, reg);
}

const char* stateName(uint16_t s) {
  switch (s) {
    case 0: return "STATIONARY";
    case 1: return "CHARGING";
    case 2: return "DISCHARGING";
    default: return "UNKNOWN";
  }
}

struct ExpectedHeader {
  uint8_t address;
  uint8_t byte_count;
};

ExpectedHeader expectedHeader(DalyRequest request) {
  if (request == DalyRequest::KEY_CONFIG) {
    return {kDalyKeyConfigReplyAddress, static_cast<uint8_t>(kDalyKeyConfigRegCount * 2)};
  }
  return {kDalyTelemetryAddress, static_cast<uint8_t>(kDalyTelemetryRegCount * 2)};
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

const char* toString(DalyKeyLogic logic) {
  switch (logic) {
    case DalyKeyLogic::UNKNOWN:                    return "UNKNOWN";
    case DalyKeyLogic::KEY_DISABLED:               return "DISABLED";
    case DalyKeyLogic::DISCHARGE_AND_SLEEP:        return "DISCHARGE_AND_SLEEP";
    case DalyKeyLogic::DISCHARGE:                  return "DISCHARGE";
    case DalyKeyLogic::CHARGE_AND_DISCHARGE:       return "CHARGE_AND_DISCHARGE";
    case DalyKeyLogic::CHARGE_DISCHARGE_AND_SLEEP: return "CHARGE_DISCHARGE_AND_SLEEP";
  }
  return "UNKNOWN";
}

DalyKeyLogic decodeDalyKeyLogic(uint16_t raw) {
  switch (raw) {
    case 0x0055: return DalyKeyLogic::KEY_DISABLED;
    case 0x00A5: return DalyKeyLogic::DISCHARGE_AND_SLEEP;
    case 0x005A: return DalyKeyLogic::DISCHARGE;
    case 0x00AA: return DalyKeyLogic::CHARGE_AND_DISCHARGE;
    case 0x00A6: return DalyKeyLogic::CHARGE_DISCHARGE_AND_SLEEP;
    default:     return DalyKeyLogic::UNKNOWN;
  }
}

const char* toString(DalyKeyReadResult result) {
  switch (result) {
    case DalyKeyReadResult::NOT_REQUESTED: return "NOT_REQUESTED";
    case DalyKeyReadResult::PENDING:       return "PENDING";
    case DalyKeyReadResult::OK:            return "OK";
    case DalyKeyReadResult::TIMEOUT:       return "TIMEOUT";
    case DalyKeyReadResult::CRC_FAIL:      return "CRC_FAIL";
    case DalyKeyReadResult::BAD_HEADER:    return "BAD_HEADER";
  }
  return "UNKNOWN";
}

DalyKeyReadResult toKeyReadResult(DalyCommResult result) {
  switch (result) {
    case DalyCommResult::OK:         return DalyKeyReadResult::OK;
    case DalyCommResult::CRC_FAIL:   return DalyKeyReadResult::CRC_FAIL;
    case DalyCommResult::BAD_HEADER: return DalyKeyReadResult::BAD_HEADER;
    case DalyCommResult::TIMEOUT:
    case DalyCommResult::NEVER_POLLED:
      break;
  }
  return DalyKeyReadResult::TIMEOUT;
}

uint16_t dalyRegister(const uint8_t* rx, uint16_t start_reg, uint16_t reg) {
  const size_t p = dalyResponseOffset(start_reg, reg);
  return static_cast<uint16_t>((static_cast<uint16_t>(rx[p]) << 8) | rx[p + 1]);
}

DalyCommResult validateDalyResponse(DalyRequest request, const uint8_t* rx, size_t len) {
  if (len != dalyResponseLen(request)) {
    return DalyCommResult::TIMEOUT;
  }

  const ExpectedHeader h = expectedHeader(request);
  if (rx[0] != h.address || rx[1] != kDalyReadFunction || rx[2] != h.byte_count) {
    return DalyCommResult::BAD_HEADER;
  }

  const uint16_t crc_calc = crc16Modbus(rx, len - 2);
  const uint16_t crc_wire =
      static_cast<uint16_t>(rx[len - 2]) |
      static_cast<uint16_t>(static_cast<uint16_t>(rx[len - 1]) << 8);
  if (crc_calc != crc_wire) {
    return DalyCommResult::CRC_FAIL;
  }
  return DalyCommResult::OK;
}

DalyCommResult parseDalyTelemetry(const uint8_t* rx, size_t len, uint32_t sampled_at_ms,
                                  DalySample* out) {
  const DalyCommResult check = validateDalyResponse(DalyRequest::TELEMETRY, rx, len);
  if (check != DalyCommResult::OK) {
    return check;
  }

  DalySample s;
  s.valid = true;
  s.sampled_at_ms = sampled_at_ms;

  s.cell_count = reg16(rx, 49);
  s.temp_count = reg16(rx, 50);

  const uint16_t cell_n = s.cell_count < 32 ? s.cell_count : 32;
  for (uint16_t i = 0; i < cell_n; ++i) {
    s.cell_mv[i] = reg16(rx, i);
  }

  const uint16_t temp_n = s.temp_count < 8 ? s.temp_count : 8;
  for (uint16_t i = 0; i < temp_n; ++i) {
    const uint16_t raw = reg16(rx, 32 + i);
    s.temp_valid[i] = (raw != 0x00FF);
    s.temp_c[i] = s.temp_valid[i] ? static_cast<int16_t>(raw) - 40 : 0;
  }

  s.pack_voltage_v = reg16(rx, 40) * 0.1f;
  s.pack_current_a = (static_cast<int32_t>(reg16(rx, 41)) - 30000) * 0.1f;
  s.soc_percent = reg16(rx, 42) * 0.1f;
  s.remaining_ah = reg16(rx, 48) * 0.1f;

  s.cell_max_mv = reg16(rx, 43);
  s.cell_min_mv = reg16(rx, 44);
  s.cell_avg_mv = reg16(rx, 55);
  s.cell_delta_mv = reg16(rx, 56);

  s.temp_max_c = static_cast<int16_t>(reg16(rx, 45)) - 40;
  s.temp_min_c = static_cast<int16_t>(reg16(rx, 46)) - 40;

  s.state_name = stateName(reg16(rx, 47));
  s.cycles = reg16(rx, 51);

  s.charge_mos_on = reg16(rx, 53) == 1;
  s.discharge_mos_on = reg16(rx, 54) == 1;

  s.alarms[0] = reg16(rx, 58);
  s.alarms[1] = reg16(rx, 59);
  s.alarms[2] = reg16(rx, 60);
  s.alarms[3] = reg16(rx, 61);

  *out = s;
  return DalyCommResult::OK;
}

DalyCommResult parseDalyKeyConfig(const uint8_t* rx, size_t len, uint32_t sampled_at_ms,
                                  DalyKeyConfigSnapshot* out) {
  const DalyCommResult check = validateDalyResponse(DalyRequest::KEY_CONFIG, rx, len);
  if (check != DalyCommResult::OK) {
    return check;
  }

  DalyKeyConfigSnapshot k;
  k.valid = true;
  k.sampled_at_ms = sampled_at_ms;

  k.key_logic_raw = dalyRegister(rx, kDalyKeyConfigStartReg, kDalyRegKeyLogic);
  k.key_logic = decodeDalyKeyLogic(k.key_logic_raw);

  k.charge_mos_control = dalyRegister(rx, kDalyKeyConfigStartReg, kDalyRegChargeMosControl);
  k.discharge_mos_control =
      dalyRegister(rx, kDalyKeyConfigStartReg, kDalyRegDischargeMosControl);

  k.sleep_time_raw = dalyRegister(rx, kDalyKeyConfigStartReg, kDalyRegSleepTime);
  k.sleep_time_seconds = static_cast<uint32_t>(k.sleep_time_raw) * kDalySleepSecondsPerCount;

  *out = k;
  return DalyCommResult::OK;
}

bool DalyBusScheduler::requestKeyConfigRead() {
  if (keyConfigReadOutstanding()) {
    return false;
  }
  key_requested_ = true;
  return true;
}

bool DalyBusScheduler::startNext(uint32_t now_ms, DalyRequest* started) {
  if (in_flight_ || now_ms - idle_since_ms_ < kDalyBusQuietGapMs) {
    return false;
  }

  const bool telemetry_due = now_ms - last_poll_start_ms_ >= kDalyPollIntervalMs;
  const bool key_first = key_requested_ && !(telemetry_due && last_finished_was_key_);

  if (key_first) {
    key_requested_ = false;
    in_flight_request_ = DalyRequest::KEY_CONFIG;
  } else if (telemetry_due) {
    last_poll_start_ms_ = now_ms;
    in_flight_request_ = DalyRequest::TELEMETRY;
  } else {
    return false;
  }

  in_flight_ = true;
  sent_ms_ = now_ms;
  *started = in_flight_request_;
  return true;
}

bool DalyBusScheduler::deadlinePassed(uint32_t now_ms) const {
  return in_flight_ && (now_ms - sent_ms_) > dalyResponseTimeoutMs(in_flight_request_);
}

void DalyBusScheduler::finish(uint32_t now_ms) {
  if (!in_flight_) {
    return;
  }
  last_finished_was_key_ = in_flight_request_ == DalyRequest::KEY_CONFIG;
  in_flight_ = false;
  idle_since_ms_ = now_ms;
}

}  // namespace power
}  // namespace matdog
