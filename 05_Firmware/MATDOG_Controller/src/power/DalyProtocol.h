#ifndef MATDOG_POWER_DALY_PROTOCOL_H
#define MATDOG_POWER_DALY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

// DALY Smart K-Series wire protocol, read-only by construction.
//
// Deliberately <stdint.h>, not <Arduino.h>: the offline host suite
// (scripts/tests/test_daly_protocol.cpp) links this translation unit, so the
// frames, CRC, decoders and bus scheduling under test are the ones that ship.
// DalyBms owns the UART and does the I/O this module decides.
//
// READ-ONLY EXCEPT ONE SEMANTIC WRITE. The only bytes that can ever reach the
// RS485 bus are the two constant FC03 read frames and ONE constant FC06 frame
// - KEY logic register 0x0120 := 0x005A (DISCHARGE) - selected by
// dalyRequestFrame() from an enum. Nothing here takes a register or value to
// transmit. scripts/static_audit.py fails the build on any other frame,
// function code, register or value, a CRC recipe change, or any other
// bms_uart_ transmit path.
//
// TWO PERSONALITIES, TWO REGISTER MAPS. The same BMS answers two Modbus
// addresses whose register numbering differs (pack voltage is 0x28 on one and
// 0x38 on the other). A register from one map must never be assumed to exist
// on the other.
//
//   0xD2 telemetry  - DALY Protocols page. Live-validated on MATDOG's unit
//                     (G3, 2026-09-18). Request address 0xD2, reply 0xD2.
//   0x81 parameters - request 0x80 + board number (1), reply 0x50 + board.
//                     Found by static analysis of DALY's official BMSTool
//                     V1.14.79 (2026-09-19): the read frame below is
//                     byte-identical to the parameter-block read it sends. NO
//                     public DALY document publishes this map. The READ was
//                     live-verified on MATDOG's unit on 2026-09-19 (KEY logic
//                     0x0055, DISABLED). The WRITE is not live-validated.

namespace matdog {
namespace power {

// ---------------------------------------------------------------------------
// CRC-16/MODBUS (constexpr: the frames below are built and checked with it at
// compile time)
// ---------------------------------------------------------------------------

constexpr uint16_t crc16Modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Whitelisted request frames (the complete transmit vocabulary)
// ---------------------------------------------------------------------------

enum class DalyRequest : uint8_t {
  TELEMETRY                 = 0,  // 0xD2, FC03 0x0000 x 0x003E
  KEY_CONFIG                = 1,  // 0x81, FC03 0x0100 x 0x0078
  KEY_LOGIC_DISCHARGE_WRITE = 2,  // 0x81, FC06 0x0120 := 0x005A (the one write)
};

constexpr size_t kDalyRequestLen = 8;

inline constexpr uint8_t kDalyTelemetryRequest[kDalyRequestLen] = {
    0xD2, 0x03, 0x00, 0x00,
    0x00, 0x3E, 0xD7, 0xB9,
};

inline constexpr uint8_t kDalyKeyConfigRequest[kDalyRequestLen] = {
    0x81, 0x03, 0x01, 0x00,
    0x00, 0x78, 0x5B, 0xD4,
};

struct DalyFrame {
  uint8_t bytes[kDalyRequestLen];
};

// The ONE permitted DALY write: FC06, register 0x0120 (KEY logic) := 0x005A
// (DISCHARGE: KEY OFF -> discharge MOS OFF, charge MOS kept). Takes no
// arguments. Bytes 0-5 are the exact frame DALY BMSTool V1.14.79 builds for
// this setting (SendModbusData_Func0x06: address 0x80 + board 1, big-endian
// register and value); crc16Modbus() appends the CRC, low byte first.
constexpr DalyFrame dalyKeyLogicDischargeWriteFrame() {
  DalyFrame f = {{0x81, 0x06, 0x01, 0x20, 0x00, 0x5A, 0x00, 0x00}};
  const uint16_t crc = crc16Modbus(f.bytes, kDalyRequestLen - 2);
  f.bytes[6] = static_cast<uint8_t>(crc & 0xFF);
  f.bytes[7] = static_cast<uint8_t>(crc >> 8);
  return f;
}

inline constexpr DalyFrame kDalyKeyLogicDischargeWrite = dalyKeyLogicDischargeWriteFrame();

// The ONLY source of transmitted bytes. Takes an enum, never a buffer.
constexpr const uint8_t* dalyRequestFrame(DalyRequest request) {
  return request == DalyRequest::KEY_CONFIG                  ? kDalyKeyConfigRequest
         : request == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE ? kDalyKeyLogicDischargeWrite.bytes
                                                             : kDalyTelemetryRequest;
}

// ---------------------------------------------------------------------------
// Frame shapes
// ---------------------------------------------------------------------------

constexpr uint8_t kDalyReadFunction = 0x03;

constexpr uint8_t kDalyTelemetryAddress = 0xD2;       // request and reply
constexpr uint16_t kDalyTelemetryStartReg = 0x0000;
constexpr uint16_t kDalyTelemetryRegCount = 0x003E;

constexpr uint8_t kDalyKeyConfigRequestAddress = 0x81;  // 0x80 + board 1
constexpr uint8_t kDalyKeyConfigReplyAddress = 0x51;    // 0x50 + board 1
constexpr uint16_t kDalyKeyConfigStartReg = 0x0100;
constexpr uint16_t kDalyKeyConfigRegCount = 0x0078;

// address + function + byte count, then 2 bytes per register, then CRC.
constexpr size_t kDalyResponseHeaderLen = 3;
constexpr size_t dalyResponseLen(uint16_t reg_count) {
  return kDalyResponseHeaderLen + static_cast<size_t>(reg_count) * 2 + 2;
}
constexpr size_t kDalyTelemetryResponseLen = dalyResponseLen(kDalyTelemetryRegCount);   // 129
constexpr size_t kDalyKeyConfigResponseLen = dalyResponseLen(kDalyKeyConfigRegCount);   // 245
constexpr size_t kDalyMaxResponseLen = kDalyKeyConfigResponseLen;

// FC06 reply: address, function, register (2), value (2), CRC (2).
constexpr size_t kDalyKeyLogicWriteResponseLen = 8;

constexpr size_t dalyResponseLen(DalyRequest request) {
  return request == DalyRequest::KEY_CONFIG                  ? kDalyKeyConfigResponseLen
         : request == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE ? kDalyKeyLogicWriteResponseLen
                                                             : kDalyTelemetryResponseLen;
}

// Byte offset of `reg`'s high byte in a complete FC03 response to a read
// that started at `start_reg`. Every decoder goes through this, never a
// hand-counted byte index.
constexpr size_t dalyResponseOffset(uint16_t start_reg, uint16_t reg) {
  return kDalyResponseHeaderLen + static_cast<size_t>(reg - start_reg) * 2;
}

// 0x81 parameter registers used by the KEY probe (BMSTool V1.14.79).
constexpr uint16_t kDalyRegSleepTime = 0x0115;
constexpr uint16_t kDalyRegKeyLogic = 0x0120;
constexpr uint16_t kDalyRegChargeMosControl = 0x0121;
constexpr uint16_t kDalyRegDischargeMosControl = 0x0122;

// BMSTool displays the sleep-time register as raw x 10 seconds.
constexpr uint32_t kDalySleepSecondsPerCount = 10;

// KEY logic values the write path reasons about (full table: DalyKeyLogic).
constexpr uint16_t kDalyKeyLogicDisabledRaw = 0x0055;   // live value on 2026-09-19
constexpr uint16_t kDalyKeyLogicDischargeRaw = 0x005A;  // the one permitted target

constexpr bool dalyRequestIsWellFormed(const uint8_t* f, uint8_t address, uint16_t start_reg,
                                       uint16_t reg_count) {
  return f[0] == address && f[1] == kDalyReadFunction &&
         f[2] == static_cast<uint8_t>(start_reg >> 8) &&
         f[3] == static_cast<uint8_t>(start_reg & 0xFF) &&
         f[4] == static_cast<uint8_t>(reg_count >> 8) &&
         f[5] == static_cast<uint8_t>(reg_count & 0xFF) &&
         crc16Modbus(f, kDalyRequestLen - 2) ==
             static_cast<uint16_t>(f[6] | (static_cast<uint16_t>(f[7]) << 8));
}

static_assert(dalyRequestIsWellFormed(kDalyTelemetryRequest, kDalyTelemetryAddress,
                                      kDalyTelemetryStartReg, kDalyTelemetryRegCount),
              "0xD2 telemetry request must be FC03 0x0000 x 0x003E with a valid CRC");
static_assert(dalyRequestIsWellFormed(kDalyKeyConfigRequest, kDalyKeyConfigRequestAddress,
                                      kDalyKeyConfigStartReg, kDalyKeyConfigRegCount),
              "0x81 parameter request must be FC03 0x0100 x 0x0078 with a valid CRC");
static_assert(kDalyTelemetryResponseLen == 129, "0xD2 reply is D2 03 7C + 124 data + CRC");
static_assert(kDalyKeyConfigResponseLen == 245, "0x81 reply is 51 03 F0 + 240 data + CRC");
static_assert(kDalyRegDischargeMosControl < kDalyKeyConfigStartReg + kDalyKeyConfigRegCount &&
                  kDalyRegSleepTime >= kDalyKeyConfigStartReg,
              "KEY probe registers must lie inside the 0x81 block that is read");
static_assert(kDalyKeyLogicDischargeWrite.bytes[0] == kDalyKeyConfigRequestAddress &&
                  kDalyKeyLogicDischargeWrite.bytes[1] != kDalyReadFunction &&
                  kDalyKeyLogicDischargeWrite.bytes[2] == (kDalyRegKeyLogic >> 8) &&
                  kDalyKeyLogicDischargeWrite.bytes[3] == (kDalyRegKeyLogic & 0xFF) &&
                  kDalyKeyLogicDischargeWrite.bytes[4] == (kDalyKeyLogicDischargeRaw >> 8) &&
                  kDalyKeyLogicDischargeWrite.bytes[5] == (kDalyKeyLogicDischargeRaw & 0xFF),
              "the one write must be KEY logic 0x0120 := 0x005A at address 0x81");

// ---------------------------------------------------------------------------
// Results and decoded models
// ---------------------------------------------------------------------------

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

// Register 0x0120 values, exactly as BMSTool V1.14.79 writes/decodes them.
// Anything else is UNKNOWN - never guessed. KEY_DISABLED (printed
// "DISABLED"): the Arduino-ESP32 core #defines DISABLED (esp32-hal-gpio.h).
enum class DalyKeyLogic : uint8_t {
  UNKNOWN                    = 0,
  KEY_DISABLED               = 1,  // 0x0055
  DISCHARGE_AND_SLEEP        = 2,  // 0x00A5
  DISCHARGE                  = 3,  // 0x005A
  CHARGE_AND_DISCHARGE       = 4,  // 0x00AA
  CHARGE_DISCHARGE_AND_SLEEP = 5,  // 0x00A6
};

const char* toString(DalyKeyLogic logic);
DalyKeyLogic decodeDalyKeyLogic(uint16_t raw);

// Outcome of the one-shot 0x81 parameter read. Kept apart from the 0xD2
// telemetry result: a silent 0x81 address says nothing about whether the
// BMS itself is present.
enum class DalyKeyReadResult : uint8_t {
  NOT_REQUESTED = 0,
  PENDING       = 1,
  OK            = 2,
  TIMEOUT       = 3,
  CRC_FAIL      = 4,
  BAD_HEADER    = 5,
};

const char* toString(DalyKeyReadResult result);
DalyKeyReadResult toKeyReadResult(DalyCommResult result);

// What the BMS reported. Deliberately no field is derived from, or feeds,
// the MOS state in DalySample: configuration and live state stay separate.
struct DalyKeyConfigSnapshot {
  bool valid = false;
  uint32_t sampled_at_ms = 0;

  uint16_t key_logic_raw = 0;
  DalyKeyLogic key_logic = DalyKeyLogic::UNKNOWN;

  uint16_t charge_mos_control = 0;
  uint16_t discharge_mos_control = 0;

  uint16_t sleep_time_raw = 0;
  uint32_t sleep_time_seconds = 0;
};

// ---------------------------------------------------------------------------
// The one KEY write: acknowledgement, read-back, preconditions, status
// ---------------------------------------------------------------------------

// Acknowledgement of the FC06 write, as DALY BMSTool V1.14.79 accepts it
// (SerialPort_ParseModbusRtu + CheckModbusResult0x06): exactly 8 bytes, reply
// address 0x50 + board (0x51), function 0x06, valid CRC, register AND value
// echoed. Anything else is a failure - never "some bytes arrived".
enum class DalyKeyWriteAck : uint8_t {
  NONE       = 0,
  PENDING    = 1,
  OK         = 2,
  TIMEOUT    = 3,  // wrong length / nothing (an exception reply is 5 bytes)
  CRC_FAIL   = 4,
  BAD_HEADER = 5,  // reply address or function wrong (a raw echo is 0x81)
  BAD_ECHO   = 6,  // register or value not echoed exactly
};

// FC03 read-back after every transmitted write. The acknowledgement alone
// never counts as verification.
enum class DalyKeyReadback : uint8_t {
  NONE            = 0,
  PENDING         = 1,
  VERIFIED        = 2,  // 0x0120 reads 0x005A
  PENDING_RESTART = 3,  // still 0x0055: BMSTool asks for a BMS restart after
                        // every setting; restart is a separate hardware gate
  MISMATCH        = 4,  // any other value
  READ_FAILED     = 5,  // the read-back transaction itself failed
};

enum class DalyKeyWriteState : uint8_t {
  NOT_REQUESTED      = 0,
  REFUSED            = 1,  // zero bytes transmitted
  ALREADY_CONFIGURED = 2,  // zero bytes transmitted
  PENDING            = 3,  // accepted; write queued, in flight or read-back pending
  COMPLETE           = 4,  // write transmitted and read-back finished
};

// First failing precondition, in evaluation order.
enum class DalyKeyWriteRefusal : uint8_t {
  NONE                    = 0,
  NOT_IN_MAINTENANCE_MODE = 1,
  WRITE_ALREADY_ATTEMPTED = 2,  // at most one transmitted write per boot
  BUS_BUSY                = 3,
  BMS_COMM_NOT_OK         = 4,
  BMS_ALARM_ACTIVE        = 5,
  NO_KEY_SNAPSHOT         = 6,  // never read, or the last read failed
  KEY_SNAPSHOT_STALE      = 7,
  KEY_LOGIC_NOT_DISABLED  = 8,  // only 0x0055 may be changed; nothing migrates
  MOS_CONTROL_NOT_ENABLED = 9,  // 0x0121/0x0122 must both read 1
};

enum class DalyKeyWriteDecision : uint8_t { START, ALREADY_CONFIGURED, REFUSE };

const char* toString(DalyKeyWriteAck ack);
const char* toString(DalyKeyReadback readback);
const char* toString(DalyKeyWriteState state);
const char* toString(DalyKeyWriteRefusal refusal);

// A valid 0x81 snapshot must be at most this old when the write is accepted
// and again when it is about to be sent (the operator runs @BMS KEY READ
// first). 0xD2 telemetry must be this recent (one missed 2 s poll plus a
// deferred one still pass).
constexpr uint32_t kDalyKeyWriteMaxSnapshotAgeMs = 30000;
constexpr uint32_t kDalyKeyWriteMaxTelemetryAgeMs = 5000;

// Everything the gate looks at. Filled by DalyBms; pure so the host suite
// tests the real rules.
struct DalyKeyWriteInputs {
  bool maintenance_mode = false;
  bool write_already_attempted = false;
  bool bus_busy = true;
  bool telemetry_ok = false;          // last 0xD2 result OK with a valid sample
  uint32_t telemetry_age_ms = 0;
  bool alarms_clear = false;          // all four 0xD2 alarm words zero
  bool last_key_read_ok = false;      // the most recent 0x81 read succeeded
  DalyKeyConfigSnapshot snapshot;
  uint32_t now_ms = 0;
};

struct DalyKeyWriteGate {
  DalyKeyWriteDecision decision = DalyKeyWriteDecision::REFUSE;
  DalyKeyWriteRefusal refusal = DalyKeyWriteRefusal::NONE;
};

DalyKeyWriteGate evaluateDalyKeyWrite(const DalyKeyWriteInputs& in);

// Classifies a complete FC06 reply against kDalyKeyLogicDischargeWrite.
DalyKeyWriteAck parseDalyKeyLogicWriteAck(const uint8_t* rx, size_t len);
DalyKeyReadback classifyDalyKeyReadback(DalyKeyReadResult read, uint16_t key_logic_raw);

// RAM-only record of the write for @BMS KEY WRITE STATUS. Nothing persists:
// if the ESP32 loses power right after the write, the BMS register is the
// only state that matters and @BMS KEY READ recovers it after reboot - no
// second write is ever needed to "finish" the first.
struct DalyKeyWriteStatus {
  DalyKeyWriteState state = DalyKeyWriteState::NOT_REQUESTED;
  DalyKeyWriteRefusal last_refusal = DalyKeyWriteRefusal::NONE;
  bool transmitted = false;  // the write frame left the UART this boot
  DalyKeyWriteAck ack = DalyKeyWriteAck::NONE;
  size_t ack_rx_bytes = 0;
  uint32_t ack_at_ms = 0;
  DalyKeyReadback readback = DalyKeyReadback::NONE;
  uint16_t readback_raw = 0;
  uint32_t readback_at_ms = 0;
};

class DalyKeyWriteTracker {
 public:
  const DalyKeyWriteStatus& status() const { return status_; }
  bool outstanding() const { return status_.state == DalyKeyWriteState::PENDING; }

  // Zero-TX outcomes. They never overwrite a pending or transmitted write.
  void refuse(DalyKeyWriteRefusal reason);
  void alreadyConfigured();
  // Accepted; the frame will go out at the next idle bus boundary.
  void accept();
  // The gate re-checked just before transmitting and said no: zero TX.
  void cancelBeforeTransmit(const DalyKeyWriteGate& gate);
  void markTransmitted() { status_.transmitted = true; }
  // The FC06 reply was classified; the read-back is always requested next.
  void onAck(DalyKeyWriteAck ack, size_t rx_bytes, uint32_t now_ms);
  void onReadback(DalyKeyReadResult read, uint16_t key_logic_raw, uint32_t now_ms);

 private:
  bool holdsWriteRecord() const;

  DalyKeyWriteStatus status_;
};

// ---------------------------------------------------------------------------
// Response validation and decoding
// ---------------------------------------------------------------------------

// Big-endian register value from a response that passed validation.
uint16_t dalyRegister(const uint8_t* rx, uint16_t start_reg, uint16_t reg);

// FC03 replies only (TELEMETRY, KEY_CONFIG). Length first (a short or long
// reply is TIMEOUT), then address/function/byte count (BAD_HEADER), then CRC
// (CRC_FAIL) - the order DalyBms used for telemetry before this module
// existed. The FC06 reply has its own validator above.
DalyCommResult validateDalyResponse(DalyRequest request, const uint8_t* rx, size_t len);

// Each writes *out ONLY when it returns OK, so a failed transaction can
// never replace the last valid sample/snapshot.
DalyCommResult parseDalyTelemetry(const uint8_t* rx, size_t len, uint32_t sampled_at_ms,
                                  DalySample* out);
DalyCommResult parseDalyKeyConfig(const uint8_t* rx, size_t len, uint32_t sampled_at_ms,
                                  DalyKeyConfigSnapshot* out);

// ---------------------------------------------------------------------------
// Bus scheduling: the one transaction owner
// ---------------------------------------------------------------------------

constexpr uint32_t kDalyPollIntervalMs = 2000;
constexpr uint32_t kDalyTelemetryTimeoutMs = 750;
// 245 bytes take 255 ms at 9600 8N1 (129 take 134 ms against a 750 ms
// timeout); 1000 ms keeps a comparable margin.
constexpr uint32_t kDalyKeyConfigTimeoutMs = 1000;
// The 8-byte FC06 reply takes 8 ms on the wire; the BMS may commit the
// setting before answering, so the same 1000 ms budget applies.
constexpr uint32_t kDalyKeyWriteTimeoutMs = 1000;
// Silence enforced after every transaction before the next request: longer
// than a whole 245-byte reply, so a late answer finishes (and is flushed)
// before anything else is sent. Never binding for telemetry alone, which
// ends <= 750 ms into its 2000 ms period.
constexpr uint32_t kDalyBusQuietGapMs = 300;

constexpr uint32_t dalyResponseTimeoutMs(DalyRequest request) {
  return request == DalyRequest::KEY_CONFIG                  ? kDalyKeyConfigTimeoutMs
         : request == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE ? kDalyKeyWriteTimeoutMs
                                                             : kDalyTelemetryTimeoutMs;
}

// Decides which transaction may start on the single RS485 bus. Pure: time
// is passed in and no I/O happens here, so the host suite tests the real
// scheduling. Two kinds share it: the periodic 0xD2 telemetry poll and ONE
// operator transaction at a time (the KEY read, or the KEY write). Invariants:
//   - never more than one transaction in flight;
//   - a transaction is never cancelled or pre-empted once started;
//   - a queued operator transaction starts at the next idle boundary, ahead
//     of a due telemetry poll, but defers at most one poll: when the previous
//     transaction was itself an operator transaction, a due poll goes first;
//   - telemetry keeps its 2000 ms cadence and resumes on its own after an
//     operator transaction completes or times out.
class DalyBusScheduler {
 public:
  // Queue one operator transaction. False while one is queued or in flight.
  bool requestKeyConfigRead() { return requestOperator(DalyRequest::KEY_CONFIG); }
  bool requestKeyLogicDischargeWrite() {
    return requestOperator(DalyRequest::KEY_LOGIC_DISCHARGE_WRITE);
  }
  bool operatorTransactionOutstanding() const { return queued_ || operatorInFlight(); }
  // The queued operator transaction, if any (not yet started).
  bool queuedOperatorRequest(DalyRequest* request) const;
  // Drops a queued operator transaction that must not start (zero TX).
  void cancelQueuedOperatorRequest() { queued_ = false; }

  // Idle only: picks the transaction to start now, if any, and marks it in
  // flight. False (and *started untouched) when nothing may start.
  bool startNext(uint32_t now_ms, DalyRequest* started);
  // Deadline base, taken after the request bytes have left the UART.
  void markSent(uint32_t sent_ms) { sent_ms_ = sent_ms; }

  bool inFlight() const { return in_flight_; }
  DalyRequest inFlightRequest() const { return in_flight_request_; }
  bool deadlinePassed(uint32_t now_ms) const;
  // In flight -> idle; the quiet gap starts now.
  void finish(uint32_t now_ms);

 private:
  bool requestOperator(DalyRequest request);
  bool operatorInFlight() const {
    return in_flight_ && in_flight_request_ != DalyRequest::TELEMETRY;
  }

  bool in_flight_ = false;
  DalyRequest in_flight_request_ = DalyRequest::TELEMETRY;
  bool queued_ = false;
  DalyRequest queued_request_ = DalyRequest::KEY_CONFIG;
  bool last_finished_was_operator_ = false;
  uint32_t sent_ms_ = 0;
  uint32_t last_poll_start_ms_ = 0;
  uint32_t idle_since_ms_ = 0;
};

}  // namespace power
}  // namespace matdog

#endif  // MATDOG_POWER_DALY_PROTOCOL_H
