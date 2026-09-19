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

const char* toString(DalyKeyWriteAck ack) {
  switch (ack) {
    case DalyKeyWriteAck::NONE:       return "NONE";
    case DalyKeyWriteAck::PENDING:    return "PENDING";
    case DalyKeyWriteAck::OK:         return "OK";
    case DalyKeyWriteAck::TIMEOUT:    return "TIMEOUT";
    case DalyKeyWriteAck::CRC_FAIL:   return "CRC_FAIL";
    case DalyKeyWriteAck::BAD_HEADER: return "BAD_HEADER";
    case DalyKeyWriteAck::BAD_ECHO:   return "BAD_ECHO";
  }
  return "UNKNOWN";
}

const char* toString(DalyKeyReadback readback) {
  switch (readback) {
    case DalyKeyReadback::NONE:            return "NONE";
    case DalyKeyReadback::PENDING:         return "PENDING";
    case DalyKeyReadback::VERIFIED:        return "VERIFIED";
    case DalyKeyReadback::PENDING_RESTART: return "PENDING_RESTART";
    case DalyKeyReadback::MISMATCH:        return "MISMATCH";
    case DalyKeyReadback::READ_FAILED:     return "READ_FAILED";
  }
  return "UNKNOWN";
}

const char* toString(DalyKeyWriteState state) {
  switch (state) {
    case DalyKeyWriteState::NOT_REQUESTED:      return "NOT_REQUESTED";
    case DalyKeyWriteState::REFUSED:            return "REFUSED";
    case DalyKeyWriteState::ALREADY_CONFIGURED: return "ALREADY_CONFIGURED";
    case DalyKeyWriteState::PENDING:            return "PENDING";
    case DalyKeyWriteState::COMPLETE:           return "COMPLETE";
  }
  return "UNKNOWN";
}

const char* toString(DalyKeyWriteRefusal refusal) {
  switch (refusal) {
    case DalyKeyWriteRefusal::NONE:                    return "NONE";
    case DalyKeyWriteRefusal::NOT_IN_MAINTENANCE_MODE: return "NOT_IN_MAINTENANCE_MODE";
    case DalyKeyWriteRefusal::WRITE_ALREADY_ATTEMPTED: return "WRITE_ALREADY_ATTEMPTED";
    case DalyKeyWriteRefusal::BUS_BUSY:                return "BUS_BUSY";
    case DalyKeyWriteRefusal::BMS_COMM_NOT_OK:         return "BMS_COMM_NOT_OK";
    case DalyKeyWriteRefusal::BMS_ALARM_ACTIVE:        return "BMS_ALARM_ACTIVE";
    case DalyKeyWriteRefusal::NO_KEY_SNAPSHOT:         return "NO_KEY_SNAPSHOT";
    case DalyKeyWriteRefusal::KEY_SNAPSHOT_STALE:      return "KEY_SNAPSHOT_STALE";
    case DalyKeyWriteRefusal::KEY_LOGIC_NOT_DISABLED:  return "KEY_LOGIC_NOT_DISABLED";
    case DalyKeyWriteRefusal::MOS_CONTROL_NOT_ENABLED: return "MOS_CONTROL_NOT_ENABLED";
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
  if (request == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE) {
    return DalyCommResult::BAD_HEADER;  // FC03 validator only; see parseDalyKeyLogicWriteAck
  }
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

DalyKeyWriteGate evaluateDalyKeyWrite(const DalyKeyWriteInputs& in) {
  DalyKeyWriteGate g;
  const DalyKeyConfigSnapshot& k = in.snapshot;

  if (!in.maintenance_mode) {
    g.refusal = DalyKeyWriteRefusal::NOT_IN_MAINTENANCE_MODE;
  } else if (in.write_already_attempted) {
    g.refusal = DalyKeyWriteRefusal::WRITE_ALREADY_ATTEMPTED;
  } else if (in.bus_busy) {
    g.refusal = DalyKeyWriteRefusal::BUS_BUSY;
  } else if (!in.telemetry_ok || in.telemetry_age_ms > kDalyKeyWriteMaxTelemetryAgeMs) {
    g.refusal = DalyKeyWriteRefusal::BMS_COMM_NOT_OK;
  } else if (!in.alarms_clear) {
    g.refusal = DalyKeyWriteRefusal::BMS_ALARM_ACTIVE;
  } else if (!k.valid || !in.last_key_read_ok) {
    g.refusal = DalyKeyWriteRefusal::NO_KEY_SNAPSHOT;
  } else if (in.now_ms - k.sampled_at_ms > kDalyKeyWriteMaxSnapshotAgeMs) {
    g.refusal = DalyKeyWriteRefusal::KEY_SNAPSHOT_STALE;
  } else if (k.key_logic_raw == kDalyKeyLogicDischargeRaw) {
    g.decision = DalyKeyWriteDecision::ALREADY_CONFIGURED;
  } else if (k.key_logic_raw != kDalyKeyLogicDisabledRaw) {
    g.refusal = DalyKeyWriteRefusal::KEY_LOGIC_NOT_DISABLED;
  } else if (k.charge_mos_control != 1 || k.discharge_mos_control != 1) {
    g.refusal = DalyKeyWriteRefusal::MOS_CONTROL_NOT_ENABLED;
  } else {
    g.decision = DalyKeyWriteDecision::START;
  }
  return g;
}

DalyKeyWriteAck parseDalyKeyLogicWriteAck(const uint8_t* rx, size_t len) {
  const uint8_t* sent = kDalyKeyLogicDischargeWrite.bytes;
  if (len != kDalyKeyLogicWriteResponseLen) {
    return DalyKeyWriteAck::TIMEOUT;
  }
  if (rx[0] != kDalyKeyConfigReplyAddress || rx[1] != sent[1]) {
    return DalyKeyWriteAck::BAD_HEADER;
  }
  const uint16_t crc_calc = crc16Modbus(rx, len - 2);
  const uint16_t crc_wire =
      static_cast<uint16_t>(rx[len - 2]) |
      static_cast<uint16_t>(static_cast<uint16_t>(rx[len - 1]) << 8);
  if (crc_calc != crc_wire) {
    return DalyKeyWriteAck::CRC_FAIL;
  }
  for (size_t i = 2; i < 6; ++i) {
    if (rx[i] != sent[i]) {
      return DalyKeyWriteAck::BAD_ECHO;
    }
  }
  return DalyKeyWriteAck::OK;
}

DalyKeyReadback classifyDalyKeyReadback(DalyKeyReadResult read, uint16_t key_logic_raw) {
  if (read != DalyKeyReadResult::OK) {
    return DalyKeyReadback::READ_FAILED;
  }
  if (key_logic_raw == kDalyKeyLogicDischargeRaw) {
    return DalyKeyReadback::VERIFIED;
  }
  if (key_logic_raw == kDalyKeyLogicDisabledRaw) {
    return DalyKeyReadback::PENDING_RESTART;
  }
  return DalyKeyReadback::MISMATCH;
}

bool DalyKeyWriteTracker::holdsWriteRecord() const {
  return status_.transmitted || status_.state == DalyKeyWriteState::PENDING;
}

void DalyKeyWriteTracker::refuse(DalyKeyWriteRefusal reason) {
  status_.last_refusal = reason;
  if (!holdsWriteRecord()) {
    status_.state = DalyKeyWriteState::REFUSED;
  }
}

void DalyKeyWriteTracker::alreadyConfigured() {
  if (!holdsWriteRecord()) {
    status_.last_refusal = DalyKeyWriteRefusal::NONE;
    status_.state = DalyKeyWriteState::ALREADY_CONFIGURED;
  }
}

void DalyKeyWriteTracker::cancelBeforeTransmit(const DalyKeyWriteGate& gate) {
  if (status_.transmitted || status_.state != DalyKeyWriteState::PENDING) {
    return;
  }
  status_.ack = DalyKeyWriteAck::NONE;
  if (gate.decision == DalyKeyWriteDecision::ALREADY_CONFIGURED) {
    status_.last_refusal = DalyKeyWriteRefusal::NONE;
    status_.state = DalyKeyWriteState::ALREADY_CONFIGURED;
  } else {
    status_.last_refusal = gate.refusal;
    status_.state = DalyKeyWriteState::REFUSED;
  }
}

void DalyKeyWriteTracker::accept() {
  status_.state = DalyKeyWriteState::PENDING;
  status_.last_refusal = DalyKeyWriteRefusal::NONE;
  status_.ack = DalyKeyWriteAck::PENDING;
  status_.readback = DalyKeyReadback::NONE;
}

void DalyKeyWriteTracker::onAck(DalyKeyWriteAck ack, size_t rx_bytes, uint32_t now_ms) {
  status_.ack = ack;
  status_.ack_rx_bytes = rx_bytes;
  status_.ack_at_ms = now_ms;
  status_.readback = DalyKeyReadback::PENDING;
}

void DalyKeyWriteTracker::onReadback(DalyKeyReadResult read, uint16_t key_logic_raw,
                                     uint32_t now_ms) {
  status_.readback = classifyDalyKeyReadback(read, key_logic_raw);
  // A failed read-back has no value; never report the previous snapshot's.
  status_.readback_raw = read == DalyKeyReadResult::OK ? key_logic_raw : 0;
  status_.readback_at_ms = now_ms;
  status_.state = DalyKeyWriteState::COMPLETE;
}

bool dalyKeyWritePreTransmitCheck(DalyBusScheduler* bus, DalyKeyWriteTracker* tracker,
                                  const DalyKeyWriteInputs& live) {
  DalyRequest queued = DalyRequest::TELEMETRY;
  if (!bus->queuedOperatorRequest(&queued) ||
      queued != DalyRequest::KEY_LOGIC_DISCHARGE_WRITE) {
    return true;
  }
  const DalyKeyWriteGate gate = evaluateDalyKeyWrite(live);
  if (gate.decision == DalyKeyWriteDecision::START) {
    return true;
  }
  bus->cancelQueuedOperatorRequest();
  tracker->cancelBeforeTransmit(gate);
  return false;
}

bool DalyBusScheduler::requestOperator(DalyRequest request) {
  if (operatorTransactionOutstanding()) {
    return false;
  }
  queued_ = true;
  queued_request_ = request;
  return true;
}

bool DalyBusScheduler::queuedOperatorRequest(DalyRequest* request) const {
  if (!queued_) {
    return false;
  }
  *request = queued_request_;
  return true;
}

bool DalyBusScheduler::startNext(uint32_t now_ms, DalyRequest* started) {
  if (in_flight_ || now_ms - idle_since_ms_ < kDalyBusQuietGapMs) {
    return false;
  }

  const bool telemetry_due = now_ms - last_poll_start_ms_ >= kDalyPollIntervalMs;
  const bool operator_first = queued_ && !(telemetry_due && last_finished_was_operator_);

  if (operator_first) {
    queued_ = false;
    in_flight_request_ = queued_request_;
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
  last_finished_was_operator_ = in_flight_request_ != DalyRequest::TELEMETRY;
  in_flight_ = false;
  idle_since_ms_ = now_ms;
}

}  // namespace power
}  // namespace matdog
