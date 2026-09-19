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
// READ-ONLY BY CONSTRUCTION. The only bytes that can ever reach the RS485 bus
// are the two constant FC03 frames below, selected by dalyRequestFrame() from
// an enum. There is no frame builder: nothing here takes a register or value
// to transmit. scripts/static_audit.py fails the build on a third frame, a
// function code other than 0x03, a CRC that does not match, or any other
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
//                     V1.14.79 (2026-09-19): the frame below is byte-identical
//                     to the parameter-block read it sends. NO public DALY
//                     document publishes this map, and it is NOT yet
//                     live-validated on MATDOG's unit.

namespace matdog {
namespace power {

// ---------------------------------------------------------------------------
// Whitelisted request frames (the complete transmit vocabulary)
// ---------------------------------------------------------------------------

enum class DalyRequest : uint8_t {
  TELEMETRY  = 0,  // 0xD2, FC03 0x0000 x 0x003E
  KEY_CONFIG = 1,  // 0x81, FC03 0x0100 x 0x0078
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

// The ONLY source of transmitted bytes. Takes an enum, never a buffer.
constexpr const uint8_t* dalyRequestFrame(DalyRequest request) {
  return request == DalyRequest::KEY_CONFIG ? kDalyKeyConfigRequest : kDalyTelemetryRequest;
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

constexpr size_t dalyResponseLen(DalyRequest request) {
  return request == DalyRequest::KEY_CONFIG ? kDalyKeyConfigResponseLen
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

// ---------------------------------------------------------------------------
// CRC-16/MODBUS (constexpr so the frames above are checked at compile time)
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
// Response validation and decoding
// ---------------------------------------------------------------------------

// Big-endian register value from a response that passed validation.
uint16_t dalyRegister(const uint8_t* rx, uint16_t start_reg, uint16_t reg);

// Length first (a short or long reply is TIMEOUT), then address/function/
// byte count (BAD_HEADER), then CRC (CRC_FAIL) - the order DalyBms used for
// telemetry before this module existed.
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
// Silence enforced after every transaction before the next request: longer
// than a whole 245-byte reply, so a late answer finishes (and is flushed)
// before anything else is sent. Never binding for telemetry alone, which
// ends <= 750 ms into its 2000 ms period.
constexpr uint32_t kDalyBusQuietGapMs = 300;

constexpr uint32_t dalyResponseTimeoutMs(DalyRequest request) {
  return request == DalyRequest::KEY_CONFIG ? kDalyKeyConfigTimeoutMs
                                            : kDalyTelemetryTimeoutMs;
}

// Decides which read-only transaction may start on the single RS485 bus.
// Pure: time is passed in and no I/O happens here, so the host suite tests
// the real scheduling. Invariants:
//   - never more than one transaction in flight;
//   - a transaction is never cancelled or pre-empted once started;
//   - a requested KEY read starts at the next idle boundary, ahead of a due
//     telemetry poll, but defers at most one poll: when the previous
//     transaction was itself a KEY read, a due poll goes first;
//   - telemetry keeps its 2000 ms cadence and resumes on its own after a
//     KEY read completes or times out.
class DalyBusScheduler {
 public:
  // Queues one KEY read. False while one is already queued or in flight.
  bool requestKeyConfigRead();
  bool keyConfigReadOutstanding() const { return key_requested_ || keyInFlight(); }

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
  bool keyInFlight() const { return in_flight_ && in_flight_request_ == DalyRequest::KEY_CONFIG; }

  bool in_flight_ = false;
  DalyRequest in_flight_request_ = DalyRequest::TELEMETRY;
  bool key_requested_ = false;
  bool last_finished_was_key_ = false;
  uint32_t sent_ms_ = 0;
  uint32_t last_poll_start_ms_ = 0;
  uint32_t idle_since_ms_ = 0;
};

}  // namespace power
}  // namespace matdog

#endif  // MATDOG_POWER_DALY_PROTOCOL_H
