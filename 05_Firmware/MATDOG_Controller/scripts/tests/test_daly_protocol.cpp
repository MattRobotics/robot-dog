// Offline host tests for the DALY wire protocol (src/power/DalyProtocol.*):
// the two whitelisted FC03 request frames, CRC-16/MODBUS, response
// validation, the 0xD2 telemetry decoder, the 0x81 KEY/parameter decoder,
// the single-owner bus scheduler, and the ONE permitted write (KEY logic
// 0x0120 := 0x005A): its exact frame, acknowledgement validation (as DALY
// BMSTool V1.14.79 accepts it), preconditions, read-back classification and
// status tracking.
//
// Links the REAL firmware translation unit, not a host-side copy. Synthetic
// responses are assembled from LITERAL byte positions (the ones recovered
// from DALY BMSTool V1.14.79: sleep 45/46, KEY logic 67/68, charge control
// 69/70, discharge control 71/72), never from the production offset helper,
// so the register-based offset calculation is proven rather than assumed.
//
// Same conventions as test_servo_population.cpp: no framework, a CHECK
// macro and a pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "../../src/power/DalyProtocol.h"

using namespace matdog::power;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (!(cond)) {                                                             \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                          \
  } while (0)

#define CHECK_EQ(actual, expected)                                             \
  do {                                                                         \
    ++g_checks;                                                                \
    const long a_ = (long)(actual);                                            \
    const long e_ = (long)(expected);                                          \
    if (a_ != e_) {                                                            \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,      \
                  __FILE__, __LINE__, #actual, a_, e_);                        \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected)                                           \
  do {                                                                         \
    ++g_checks;                                                                \
    const double a_ = (double)(actual);                                        \
    const double e_ = (double)(expected);                                      \
    if (std::fabs(a_ - e_) > 1e-3) {                                           \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == %f, expected %f\n", g_case,        \
                  __FILE__, __LINE__, #actual, a_, e_);                        \
    }                                                                          \
  } while (0)

// ---------------------------------------------------------------------------
// Synthetic frame helpers (literal positions, independent of production
// offset arithmetic)
// ---------------------------------------------------------------------------

static void putBe16(uint8_t* buf, size_t hi_index, uint16_t value) {
  buf[hi_index] = static_cast<uint8_t>(value >> 8);
  buf[hi_index + 1] = static_cast<uint8_t>(value & 0xFF);
}

static void appendCrc(uint8_t* buf, size_t total_len) {
  const uint16_t crc = crc16Modbus(buf, total_len - 2);
  buf[total_len - 2] = static_cast<uint8_t>(crc & 0xFF);  // Modbus: low byte first
  buf[total_len - 1] = static_cast<uint8_t>(crc >> 8);
}

struct KeyFields {
  uint16_t sleep_raw;
  uint16_t key_logic_raw;
  uint16_t charge_ctrl;
  uint16_t discharge_ctrl;
};

// 245 bytes: 51 03 F0 | 240 data bytes | CRC. Every other data byte is 0xEE
// so a decoder reading the wrong offset cannot land on a plausible value.
static void buildKeyResponse(uint8_t* buf, const KeyFields& f) {
  std::memset(buf, 0xEE, 245);
  buf[0] = 0x51;
  buf[1] = 0x03;
  buf[2] = 0xF0;
  putBe16(buf, 45, f.sleep_raw);       // register 0x0115
  putBe16(buf, 67, f.key_logic_raw);   // register 0x0120
  putBe16(buf, 69, f.charge_ctrl);     // register 0x0121
  putBe16(buf, 71, f.discharge_ctrl);  // register 0x0122
  appendCrc(buf, 245);
}

static bool sameSnapshot(const DalyKeyConfigSnapshot& a, const DalyKeyConfigSnapshot& b) {
  return a.valid == b.valid && a.sampled_at_ms == b.sampled_at_ms &&
         a.key_logic_raw == b.key_logic_raw && a.key_logic == b.key_logic &&
         a.charge_mos_control == b.charge_mos_control &&
         a.discharge_mos_control == b.discharge_mos_control &&
         a.sleep_time_raw == b.sleep_time_raw &&
         a.sleep_time_seconds == b.sleep_time_seconds;
}

// ---------------------------------------------------------------------------
// 1 / 2: CRC and exact request bytes
// ---------------------------------------------------------------------------

static void test_crc16_known_vectors() {
  g_case = "crc16_known_vectors";
  // CRC-16/MODBUS catalogue check value.
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_EQ(crc16Modbus(check, sizeof(check)), 0x4B37);

  // Both request frames: CRC of the first six bytes equals the last two
  // (low byte first), as captured live (D7 B9) and from BMSTool (5B D4).
  CHECK_EQ(crc16Modbus(kDalyTelemetryRequest, 6), 0xB9D7);
  CHECK_EQ(crc16Modbus(kDalyKeyConfigRequest, 6), 0xD45B);
}

static void test_request_frames_exact() {
  g_case = "request_frames_exact";
  const uint8_t telemetry[8] = {0xD2, 0x03, 0x00, 0x00, 0x00, 0x3E, 0xD7, 0xB9};
  const uint8_t key[8] = {0x81, 0x03, 0x01, 0x00, 0x00, 0x78, 0x5B, 0xD4};
  CHECK_EQ(kDalyRequestLen, 8);
  CHECK(std::memcmp(kDalyTelemetryRequest, telemetry, 8) == 0);
  CHECK(std::memcmp(kDalyKeyConfigRequest, key, 8) == 0);

  // The selector returns exactly the two constant frames, nothing else.
  CHECK(dalyRequestFrame(DalyRequest::TELEMETRY) == kDalyTelemetryRequest);
  CHECK(dalyRequestFrame(DalyRequest::KEY_CONFIG) == kDalyKeyConfigRequest);

  // Both are Modbus READ (FC03); neither is a write.
  CHECK_EQ(dalyRequestFrame(DalyRequest::TELEMETRY)[1], 0x03);
  CHECK_EQ(dalyRequestFrame(DalyRequest::KEY_CONFIG)[1], 0x03);
}

static void test_response_shapes() {
  g_case = "response_shapes";
  CHECK_EQ(dalyResponseLen(DalyRequest::TELEMETRY), 129);
  CHECK_EQ(dalyResponseLen(DalyRequest::KEY_CONFIG), 245);
  CHECK_EQ(kDalyMaxResponseLen, 245);
  CHECK_EQ(kDalyKeyConfigRegCount * 2, 0xF0);
  CHECK_EQ(kDalyTelemetryRegCount * 2, 0x7C);
}

// ---------------------------------------------------------------------------
// Register offsets: register arithmetic == BMSTool's literal byte indexes
// ---------------------------------------------------------------------------

static void test_register_offsets() {
  g_case = "register_offsets";
  CHECK_EQ(dalyResponseOffset(0x0100, kDalyRegSleepTime), 3 + 0x15 * 2);
  CHECK_EQ(dalyResponseOffset(0x0100, kDalyRegKeyLogic), 3 + 0x20 * 2);
  CHECK_EQ(dalyResponseOffset(0x0100, kDalyRegChargeMosControl), 3 + 0x21 * 2);
  CHECK_EQ(dalyResponseOffset(0x0100, kDalyRegDischargeMosControl), 3 + 0x22 * 2);

  CHECK_EQ(dalyResponseOffset(kDalyKeyConfigStartReg, kDalyRegSleepTime), 45);
  CHECK_EQ(dalyResponseOffset(kDalyKeyConfigStartReg, kDalyRegKeyLogic), 67);
  CHECK_EQ(dalyResponseOffset(kDalyKeyConfigStartReg, kDalyRegChargeMosControl), 69);
  CHECK_EQ(dalyResponseOffset(kDalyKeyConfigStartReg, kDalyRegDischargeMosControl), 71);

  // Telemetry (block starts at 0x0000): the historical 3 + reg * 2.
  CHECK_EQ(dalyResponseOffset(kDalyTelemetryStartReg, 40), 83);
  CHECK_EQ(dalyResponseOffset(kDalyTelemetryStartReg, 61), 125);
}

// ---------------------------------------------------------------------------
// 3 / 4 / 5 / 6: valid 0x81 response and field decoding
// ---------------------------------------------------------------------------

static void test_key_config_valid_response() {
  g_case = "key_config_valid_response";
  uint8_t rx[245];
  buildKeyResponse(rx, {360, 0x0055, 1, 0});

  DalyKeyConfigSnapshot k;
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 12345, &k), DalyCommResult::OK);
  CHECK(k.valid);
  CHECK_EQ(k.sampled_at_ms, 12345);
  CHECK_EQ(k.key_logic_raw, 0x0055);
  CHECK(k.key_logic == DalyKeyLogic::KEY_DISABLED);
  CHECK_EQ(k.charge_mos_control, 1);
  CHECK_EQ(k.discharge_mos_control, 0);
  CHECK_EQ(k.sleep_time_raw, 360);
  CHECK_EQ(k.sleep_time_seconds, 3600);
  CHECK_EQ(validateDalyResponse(DalyRequest::KEY_CONFIG, rx, sizeof(rx)), DalyCommResult::OK);
}

static void test_key_logic_mappings() {
  g_case = "key_logic_mappings";
  struct Case {
    uint16_t raw;
    DalyKeyLogic logic;
    const char* name;
  };
  const Case cases[] = {
      {0x0055, DalyKeyLogic::KEY_DISABLED, "DISABLED"},
      {0x00A5, DalyKeyLogic::DISCHARGE_AND_SLEEP, "DISCHARGE_AND_SLEEP"},
      {0x005A, DalyKeyLogic::DISCHARGE, "DISCHARGE"},
      {0x00AA, DalyKeyLogic::CHARGE_AND_DISCHARGE, "CHARGE_AND_DISCHARGE"},
      {0x00A6, DalyKeyLogic::CHARGE_DISCHARGE_AND_SLEEP, "CHARGE_DISCHARGE_AND_SLEEP"},
  };
  for (const Case& c : cases) {
    CHECK(decodeDalyKeyLogic(c.raw) == c.logic);
    CHECK(std::strcmp(toString(c.logic), c.name) == 0);

    // And through a complete frame, so the value really comes from 0x0120.
    uint8_t rx[245];
    buildKeyResponse(rx, {0, c.raw, 0, 0});
    DalyKeyConfigSnapshot k;
    CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::OK);
    CHECK_EQ(k.key_logic_raw, c.raw);
    CHECK(k.key_logic == c.logic);
  }

  // Anything else is UNKNOWN - including byte-swapped and neighbouring values.
  const uint16_t unknown[] = {0x0000, 0x00FF, 0xFFFF, 0x5500, 0x5A00, 0x0056, 0x00A7, 0x0155};
  for (uint16_t raw : unknown) {
    CHECK(decodeDalyKeyLogic(raw) == DalyKeyLogic::UNKNOWN);
  }
  CHECK(std::strcmp(toString(DalyKeyLogic::UNKNOWN), "UNKNOWN") == 0);
}

static void test_sleep_time_width() {
  g_case = "sleep_time_width";
  uint8_t rx[245];
  DalyKeyConfigSnapshot k;

  buildKeyResponse(rx, {0xFFFF, 0x0055, 1, 1});
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::OK);
  CHECK_EQ(k.sleep_time_raw, 0xFFFF);
  CHECK_EQ(k.sleep_time_seconds, 655350UL);  // no 16-bit wrap

  buildKeyResponse(rx, {0, 0x0055, 1, 1});
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::OK);
  CHECK_EQ(k.sleep_time_seconds, 0);
}

static void test_mos_controls_from_their_own_registers() {
  g_case = "mos_controls_from_their_own_registers";
  uint8_t rx[245];
  DalyKeyConfigSnapshot k;

  buildKeyResponse(rx, {0, 0x0055, 1, 0});
  putBe16(rx, 65, 0x7777);  // 0x011F, neighbour below KEY logic
  putBe16(rx, 73, 0x8888);  // 0x0123, neighbour above discharge control
  appendCrc(rx, 245);
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::OK);
  CHECK_EQ(k.charge_mos_control, 1);
  CHECK_EQ(k.discharge_mos_control, 0);

  buildKeyResponse(rx, {0, 0x0055, 0, 1});
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::OK);
  CHECK_EQ(k.charge_mos_control, 0);
  CHECK_EQ(k.discharge_mos_control, 1);
}

// ---------------------------------------------------------------------------
// 7 / 8 / 9 / 10: rejection, and a failure never replaces a valid snapshot
// ---------------------------------------------------------------------------

static void test_bad_header_rejected() {
  g_case = "bad_header_rejected";
  struct Mutation {
    size_t index;
    uint8_t value;
  };
  const Mutation mutations[] = {
      {0, 0x81},  // request address echoed back instead of 0x50 + board
      {0, 0xD2},  // telemetry personality
      {0, 0x52},  // another board number
      {1, 0x83},  // Modbus exception function
      {1, 0x06},  // a write echo is never an acceptable read reply
      {2, 0x7C},  // telemetry byte count
  };
  for (const Mutation& m : mutations) {
    uint8_t rx[245];
    buildKeyResponse(rx, {360, 0x005A, 1, 1});
    rx[m.index] = m.value;
    appendCrc(rx, 245);  // CRC valid: the header alone must reject it
    DalyKeyConfigSnapshot k;
    CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::BAD_HEADER);
    CHECK(!k.valid);
  }
}

static void test_crc_failure_rejected() {
  g_case = "crc_failure_rejected";
  uint8_t rx[245];
  DalyKeyConfigSnapshot k;

  buildKeyResponse(rx, {360, 0x005A, 1, 1});
  rx[67] ^= 0x01;  // corrupt KEY logic after the CRC was computed
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::CRC_FAIL);
  CHECK(!k.valid);

  buildKeyResponse(rx, {360, 0x005A, 1, 1});
  rx[244] ^= 0x80;  // corrupt the CRC itself
  CHECK_EQ(parseDalyKeyConfig(rx, sizeof(rx), 1, &k), DalyCommResult::CRC_FAIL);
  CHECK(!k.valid);
}

static void test_truncated_response_rejected() {
  g_case = "truncated_response_rejected";
  uint8_t rx[246];
  buildKeyResponse(rx, {360, 0x005A, 1, 1});
  DalyKeyConfigSnapshot k;

  CHECK_EQ(parseDalyKeyConfig(rx, 244, 1, &k), DalyCommResult::TIMEOUT);
  CHECK_EQ(parseDalyKeyConfig(rx, 0, 1, &k), DalyCommResult::TIMEOUT);
  CHECK_EQ(parseDalyKeyConfig(rx, 246, 1, &k), DalyCommResult::TIMEOUT);

  // A Modbus exception reply (51 83 02 CRC) is 5 bytes: not accepted.
  uint8_t exception[5] = {0x51, 0x83, 0x02, 0, 0};
  appendCrc(exception, sizeof(exception));
  CHECK_EQ(parseDalyKeyConfig(exception, sizeof(exception), 1, &k), DalyCommResult::TIMEOUT);
  CHECK(!k.valid);
}

static void test_failed_read_keeps_previous_snapshot() {
  g_case = "failed_read_keeps_previous_snapshot";
  uint8_t good[245];
  buildKeyResponse(good, {360, 0x0055, 1, 1});
  DalyKeyConfigSnapshot k;
  CHECK_EQ(parseDalyKeyConfig(good, sizeof(good), 1000, &k), DalyCommResult::OK);
  const DalyKeyConfigSnapshot before = k;

  uint8_t bad[245];
  buildKeyResponse(bad, {1, 0x005A, 0, 0});
  bad[0] = 0xD2;
  appendCrc(bad, 245);
  CHECK_EQ(parseDalyKeyConfig(bad, sizeof(bad), 2000, &k), DalyCommResult::BAD_HEADER);
  CHECK(sameSnapshot(k, before));

  buildKeyResponse(bad, {1, 0x005A, 0, 0});
  bad[100] ^= 0xFF;
  CHECK_EQ(parseDalyKeyConfig(bad, sizeof(bad), 3000, &k), DalyCommResult::CRC_FAIL);
  CHECK(sameSnapshot(k, before));

  CHECK_EQ(parseDalyKeyConfig(bad, 12, 4000, &k), DalyCommResult::TIMEOUT);
  CHECK(sameSnapshot(k, before));
}

static void test_key_read_result_mapping() {
  g_case = "key_read_result_mapping";
  CHECK(toKeyReadResult(DalyCommResult::OK) == DalyKeyReadResult::OK);
  CHECK(toKeyReadResult(DalyCommResult::TIMEOUT) == DalyKeyReadResult::TIMEOUT);
  CHECK(toKeyReadResult(DalyCommResult::CRC_FAIL) == DalyKeyReadResult::CRC_FAIL);
  CHECK(toKeyReadResult(DalyCommResult::BAD_HEADER) == DalyKeyReadResult::BAD_HEADER);
  CHECK(std::strcmp(toString(DalyKeyReadResult::NOT_REQUESTED), "NOT_REQUESTED") == 0);
  CHECK(std::strcmp(toString(DalyKeyReadResult::PENDING), "PENDING") == 0);
}

// ---------------------------------------------------------------------------
// 11: the 0xD2 telemetry decoder behaves exactly as validated in G3
// ---------------------------------------------------------------------------

// 129 bytes: D2 03 7C | 62 registers | CRC. Register n's high byte sits at
// the literal position 3 + 2n used by the G3-validated decoder.
static void buildTelemetryResponse(uint8_t* buf) {
  std::memset(buf, 0, 129);
  buf[0] = 0xD2;
  buf[1] = 0x03;
  buf[2] = 0x7C;
  putBe16(buf, 3 + 2 * 0, 3700);   // cell 1
  putBe16(buf, 3 + 2 * 1, 3710);   // cell 2
  putBe16(buf, 3 + 2 * 2, 3690);   // cell 3
  putBe16(buf, 3 + 2 * 32, 65);    // temp 1 = 25 C
  putBe16(buf, 3 + 2 * 33, 0x00FF);  // temp 2 invalid
  putBe16(buf, 3 + 2 * 40, 110);   // 11.0 V
  putBe16(buf, 3 + 2 * 41, 29999); // -0.1 A
  putBe16(buf, 3 + 2 * 42, 393);   // 39.3 %
  putBe16(buf, 3 + 2 * 43, 3710);  // cell max
  putBe16(buf, 3 + 2 * 44, 3690);  // cell min
  putBe16(buf, 3 + 2 * 45, 65);    // temp max 25 C
  putBe16(buf, 3 + 2 * 46, 60);    // temp min 20 C
  putBe16(buf, 3 + 2 * 47, 2);     // DISCHARGING
  putBe16(buf, 3 + 2 * 48, 500);   // 50.0 Ah
  putBe16(buf, 3 + 2 * 49, 3);     // cell count
  putBe16(buf, 3 + 2 * 50, 2);     // temp count
  putBe16(buf, 3 + 2 * 51, 7);     // cycles
  putBe16(buf, 3 + 2 * 53, 1);     // charge MOS ON
  putBe16(buf, 3 + 2 * 54, 0);     // discharge MOS OFF
  putBe16(buf, 3 + 2 * 55, 3700);  // cell avg
  putBe16(buf, 3 + 2 * 56, 20);    // cell delta
  putBe16(buf, 3 + 2 * 58, 0x0001);
  putBe16(buf, 3 + 2 * 59, 0x0203);
  putBe16(buf, 3 + 2 * 60, 0x0000);
  putBe16(buf, 3 + 2 * 61, 0x8000);
  appendCrc(buf, 129);
}

static void test_telemetry_parser_unchanged() {
  g_case = "telemetry_parser_unchanged";
  uint8_t rx[129];
  buildTelemetryResponse(rx);

  DalySample s;
  CHECK_EQ(parseDalyTelemetry(rx, sizeof(rx), 777, &s), DalyCommResult::OK);
  CHECK(s.valid);
  CHECK_EQ(s.sampled_at_ms, 777);
  CHECK_EQ(s.cell_count, 3);
  CHECK_EQ(s.cell_mv[0], 3700);
  CHECK_EQ(s.cell_mv[1], 3710);
  CHECK_EQ(s.cell_mv[2], 3690);
  CHECK_EQ(s.cell_mv[3], 0);
  CHECK_EQ(s.temp_count, 2);
  CHECK(s.temp_valid[0]);
  CHECK_EQ(s.temp_c[0], 25);
  CHECK(!s.temp_valid[1]);
  CHECK_EQ(s.temp_c[1], 0);
  CHECK_NEAR(s.pack_voltage_v, 11.0);
  CHECK_NEAR(s.pack_current_a, -0.1);
  CHECK_NEAR(s.soc_percent, 39.3);
  CHECK_NEAR(s.remaining_ah, 50.0);
  CHECK_EQ(s.cell_max_mv, 3710);
  CHECK_EQ(s.cell_min_mv, 3690);
  CHECK_EQ(s.cell_avg_mv, 3700);
  CHECK_EQ(s.cell_delta_mv, 20);
  CHECK_EQ(s.temp_max_c, 25);
  CHECK_EQ(s.temp_min_c, 20);
  CHECK(std::strcmp(s.state_name, "DISCHARGING") == 0);
  CHECK_EQ(s.cycles, 7);
  CHECK(s.charge_mos_on);
  CHECK(!s.discharge_mos_on);
  CHECK_EQ(s.alarms[0], 0x0001);
  CHECK_EQ(s.alarms[1], 0x0203);
  CHECK_EQ(s.alarms[2], 0x0000);
  CHECK_EQ(s.alarms[3], 0x8000);

  // Oversized cell count is clamped to the 32-cell table, as before.
  putBe16(rx, 3 + 2 * 49, 40);
  appendCrc(rx, 129);
  DalySample clamped;
  CHECK_EQ(parseDalyTelemetry(rx, sizeof(rx), 1, &clamped), DalyCommResult::OK);
  CHECK_EQ(clamped.cell_count, 40);
}

static void test_telemetry_rejections_unchanged() {
  g_case = "telemetry_rejections_unchanged";
  uint8_t rx[129];
  DalySample good;
  buildTelemetryResponse(rx);
  CHECK_EQ(parseDalyTelemetry(rx, sizeof(rx), 5, &good), DalyCommResult::OK);

  DalySample s = good;
  CHECK_EQ(parseDalyTelemetry(rx, 128, 6, &s), DalyCommResult::TIMEOUT);
  CHECK_EQ(s.sampled_at_ms, 5);  // untouched

  buildTelemetryResponse(rx);
  rx[0] = 0x51;  // the 0x81 personality's reply is not telemetry
  appendCrc(rx, 129);
  CHECK_EQ(parseDalyTelemetry(rx, sizeof(rx), 6, &s), DalyCommResult::BAD_HEADER);
  CHECK_EQ(s.sampled_at_ms, 5);

  buildTelemetryResponse(rx);
  rx[2] = 0xF0;
  appendCrc(rx, 129);
  CHECK_EQ(parseDalyTelemetry(rx, sizeof(rx), 6, &s), DalyCommResult::BAD_HEADER);

  buildTelemetryResponse(rx);
  rx[50] ^= 0x10;
  CHECK_EQ(parseDalyTelemetry(rx, sizeof(rx), 6, &s), DalyCommResult::CRC_FAIL);
  CHECK_EQ(s.sampled_at_ms, 5);

  // A complete, valid 0x81 reply is never accepted as telemetry.
  uint8_t key[245];
  buildKeyResponse(key, {360, 0x0055, 1, 1});
  CHECK_EQ(parseDalyTelemetry(key, sizeof(key), 6, &s), DalyCommResult::TIMEOUT);
  CHECK_EQ(s.sampled_at_ms, 5);
}

// ---------------------------------------------------------------------------
// 12: single-owner scheduling and return to normal telemetry polling
// ---------------------------------------------------------------------------

static void test_scheduler_telemetry_cadence() {
  g_case = "scheduler_telemetry_cadence";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::KEY_CONFIG;

  CHECK(!bus.startNext(1500, &r));  // first poll at 2000 ms, as before
  CHECK(bus.startNext(2000, &r));
  CHECK(r == DalyRequest::TELEMETRY);
  bus.markSent(2008);
  CHECK(!bus.startNext(2100, &r));  // never a second transaction in flight
  CHECK(!bus.deadlinePassed(2758));
  CHECK(bus.deadlinePassed(2759));  // 750 ms after the bytes left
  bus.finish(2300);
  CHECK(!bus.startNext(3999, &r));
  CHECK(bus.startNext(4000, &r));
  CHECK(r == DalyRequest::TELEMETRY);
}

static void test_scheduler_key_waits_for_active_telemetry() {
  g_case = "scheduler_key_waits_for_active_telemetry";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;

  CHECK(bus.startNext(2000, &r));
  CHECK(r == DalyRequest::TELEMETRY);
  CHECK(bus.requestKeyConfigRead());
  CHECK(!bus.requestKeyConfigRead());  // one outstanding at a time
  CHECK(bus.operatorTransactionOutstanding());
  CHECK(!bus.startNext(2100, &r));     // telemetry not cancelled or overlapped
  CHECK(bus.inFlight());
  CHECK(bus.inFlightRequest() == DalyRequest::TELEMETRY);

  bus.finish(2200);
  CHECK(!bus.startNext(2499, &r));     // quiet gap after the telemetry reply
  CHECK(bus.startNext(2500, &r));
  CHECK(r == DalyRequest::KEY_CONFIG);
  CHECK(!bus.requestKeyConfigRead());  // still outstanding while in flight
  bus.markSent(2508);
  bus.finish(2800);
  CHECK(!bus.operatorTransactionOutstanding());

  CHECK(!bus.startNext(3999, &r));     // telemetry cadence kept
  CHECK(bus.startNext(4000, &r));
  CHECK(r == DalyRequest::TELEMETRY);
}

static void test_scheduler_key_defers_one_poll_then_resumes() {
  g_case = "scheduler_key_defers_one_poll_then_resumes";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;

  CHECK(bus.startNext(2000, &r));
  bus.finish(2150);
  CHECK(bus.requestKeyConfigRead());
  CHECK(bus.startNext(4000, &r));      // KEY and poll both due: KEY first
  CHECK(r == DalyRequest::KEY_CONFIG);
  bus.markSent(4008);
  bus.finish(4300);                    // completed
  CHECK(!bus.startNext(4599, &r));
  CHECK(bus.startNext(4600, &r));      // deferred poll resumes by itself
  CHECK(r == DalyRequest::TELEMETRY);
  bus.finish(4700);
  CHECK(!bus.startNext(6599, &r));
  CHECK(bus.startNext(6600, &r));      // and re-anchors its cadence
  CHECK(r == DalyRequest::TELEMETRY);
}

static void test_scheduler_key_timeout_then_resumes() {
  g_case = "scheduler_key_timeout_then_resumes";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;

  CHECK(bus.requestKeyConfigRead());
  CHECK(bus.startNext(1600, &r));
  CHECK(r == DalyRequest::KEY_CONFIG);
  bus.markSent(1608);
  CHECK(!bus.deadlinePassed(2608));
  CHECK(bus.deadlinePassed(2609));     // 1000 ms KEY deadline
  bus.finish(2609);
  CHECK(!bus.operatorTransactionOutstanding());
  CHECK(!bus.startNext(2908, &r));
  CHECK(bus.startNext(2909, &r));
  CHECK(r == DalyRequest::TELEMETRY);
  CHECK(bus.requestKeyConfigRead());   // a new read may be requested again
}

static void test_scheduler_key_cannot_starve_telemetry() {
  g_case = "scheduler_key_cannot_starve_telemetry";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;

  CHECK(bus.requestKeyConfigRead());
  CHECK(bus.startNext(2000, &r));
  CHECK(r == DalyRequest::KEY_CONFIG);  // defers the 2000 ms poll
  bus.finish(2400);
  CHECK(bus.requestKeyConfigRead());    // requested again immediately
  CHECK(bus.startNext(2700, &r));
  CHECK(r == DalyRequest::TELEMETRY);   // but the deferred poll goes first
  bus.finish(2850);
  CHECK(bus.startNext(3150, &r));
  CHECK(r == DalyRequest::KEY_CONFIG);
}

static void test_scheduler_simulated_bus() {
  g_case = "scheduler_simulated_bus";
  // 10 minutes of 1 ms ticks with a KEY read requested every 700 ms: never
  // two transactions at once, telemetry never deferred by more than one KEY
  // read (+ quiet gaps), every KEY read eventually runs.
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;
  uint32_t last_telemetry = 0;
  uint32_t worst_telemetry_gap = 0;
  uint32_t telemetry_count = 0;
  uint32_t key_count = 0;
  uint32_t requested = 0;
  uint32_t lcg = 12345;

  uint32_t reply_at = 0;  // 0 = this transaction is never answered
  for (uint32_t t = 1500; t < 600000; ++t) {
    if (t % 700 == 0 && bus.requestKeyConfigRead()) ++requested;

    if (!bus.inFlight()) {
      if (bus.startNext(t, &r)) {
        bus.markSent(t + 8);
        // Replies arrive 150..600 ms after the request, or never (timeout).
        lcg = lcg * 1103515245u + 12345u;
        reply_at = ((lcg >> 16) % 5 == 0) ? 0 : t + 150 + (lcg >> 8) % 450;
        if (r == DalyRequest::TELEMETRY) {
          if (last_telemetry != 0 && t - last_telemetry > worst_telemetry_gap) {
            worst_telemetry_gap = t - last_telemetry;
          }
          last_telemetry = t;
          ++telemetry_count;
        } else {
          ++key_count;
        }
      }
      continue;
    }

    if ((reply_at != 0 && t >= reply_at) || bus.deadlinePassed(t)) {
      bus.finish(t);
    }
  }

  CHECK(telemetry_count > 200);
  CHECK(key_count > 200);
  CHECK(requested == key_count || requested == key_count + 1);
  // 2000 ms period + at most one KEY read (1000 ms + 8) + two quiet gaps.
  CHECK(worst_telemetry_gap <= 2000 + 1008 + 2 * 300 + 1000);
}


// ---------------------------------------------------------------------------
// The one KEY write: frame, acknowledgement, gate, tracker, scheduling
// ---------------------------------------------------------------------------

// The expected reply, written out from BMSTool's receive path: address
// 0x50 + board, function 0x06, register and value echoed, CRC-16/MODBUS.
static const uint8_t kGoodWriteAck[8] = {0x51, 0x06, 0x01, 0x20, 0x00, 0x5A, 0x05, 0x97};

static void test_write_frame_exact() {
  g_case = "write_frame_exact";
  const uint8_t expected[8] = {0x81, 0x06, 0x01, 0x20, 0x00, 0x5A, 0x16, 0x07};
  CHECK(std::memcmp(kDalyKeyLogicDischargeWrite.bytes, expected, 8) == 0);
  CHECK(dalyRequestFrame(DalyRequest::KEY_LOGIC_DISCHARGE_WRITE) ==
        kDalyKeyLogicDischargeWrite.bytes);
  // The CRC really is crc16Modbus of the six payload bytes, low byte first.
  CHECK_EQ(crc16Modbus(expected, 6), 0x0716);
  // And the reply's CRC, for the independently written expected ACK.
  CHECK_EQ(crc16Modbus(kGoodWriteAck, 6), 0x9705);
  CHECK_EQ(dalyResponseLen(DalyRequest::KEY_LOGIC_DISCHARGE_WRITE), 8);
  CHECK_EQ(dalyResponseTimeoutMs(DalyRequest::KEY_LOGIC_DISCHARGE_WRITE), 1000);
  // Only the reads use the FC03 validator.
  CHECK_EQ(validateDalyResponse(DalyRequest::KEY_LOGIC_DISCHARGE_WRITE, kGoodWriteAck, 8),
           DalyCommResult::BAD_HEADER);
}

static void ackWith(uint8_t* buf, size_t index, uint8_t value) {
  std::memcpy(buf, kGoodWriteAck, 8);
  buf[index] = value;
  appendCrc(buf, 8);
}

static void test_write_ack_validation() {
  g_case = "write_ack_validation";
  uint8_t rx[9];
  CHECK_EQ(parseDalyKeyLogicWriteAck(kGoodWriteAck, 8), DalyKeyWriteAck::OK);

  std::memcpy(rx, kGoodWriteAck, 8); rx[7] ^= 0x01;              // bad CRC
  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::CRC_FAIL);
  std::memcpy(rx, kGoodWriteAck, 8); rx[5] ^= 0x01;              // data changed after CRC
  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::CRC_FAIL);

  // A raw echo of our own request (address 0x81) is not an acknowledgement.
  CHECK_EQ(parseDalyKeyLogicWriteAck(kDalyKeyLogicDischargeWrite.bytes, 8),
           DalyKeyWriteAck::BAD_HEADER);
  ackWith(rx, 0, 0xD2);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_HEADER);
  ackWith(rx, 0, 0x52);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_HEADER);
  ackWith(rx, 1, 0x03);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_HEADER);
  ackWith(rx, 1, 0x86);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_HEADER);
  ackWith(rx, 1, 0x10);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_HEADER);

  // Register / value not echoed exactly (CRC valid).
  ackWith(rx, 2, 0x00);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_ECHO);
  ackWith(rx, 3, 0x21);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_ECHO);
  ackWith(rx, 3, 0x22);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_ECHO);
  ackWith(rx, 5, 0x55);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_ECHO);
  ackWith(rx, 5, 0xAA);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_ECHO);
  ackWith(rx, 4, 0x01);  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 8), DalyKeyWriteAck::BAD_ECHO);

  // Truncated, silent, oversized, or a Modbus exception (5 bytes).
  CHECK_EQ(parseDalyKeyLogicWriteAck(kGoodWriteAck, 7), DalyKeyWriteAck::TIMEOUT);
  CHECK_EQ(parseDalyKeyLogicWriteAck(kGoodWriteAck, 0), DalyKeyWriteAck::TIMEOUT);
  std::memcpy(rx, kGoodWriteAck, 8); rx[8] = 0;
  CHECK_EQ(parseDalyKeyLogicWriteAck(rx, 9), DalyKeyWriteAck::TIMEOUT);
  uint8_t exception[5] = {0x51, 0x86, 0x02, 0, 0};
  appendCrc(exception, 5);
  CHECK_EQ(parseDalyKeyLogicWriteAck(exception, 5), DalyKeyWriteAck::TIMEOUT);
}

// A healthy set of gate inputs that START; each test breaks one thing.
static DalyKeyWriteInputs goodInputs() {
  DalyKeyWriteInputs in;
  in.maintenance_mode = true;
  in.write_already_attempted = false;
  in.bus_busy = false;
  in.telemetry_ok = true;
  in.telemetry_age_ms = 400;
  in.alarms_clear = true;
  in.last_key_read_ok = true;
  in.snapshot.valid = true;
  in.snapshot.sampled_at_ms = 100000;
  in.snapshot.key_logic_raw = 0x0055;
  in.snapshot.key_logic = DalyKeyLogic::KEY_DISABLED;
  in.snapshot.charge_mos_control = 1;
  in.snapshot.discharge_mos_control = 1;
  in.now_ms = 110000;
  return in;
}

static void expectRefused(const DalyKeyWriteInputs& in, DalyKeyWriteRefusal reason) {
  const DalyKeyWriteGate g = evaluateDalyKeyWrite(in);
  CHECK(g.decision == DalyKeyWriteDecision::REFUSE);
  CHECK_EQ(g.refusal, reason);
}

static void test_write_gate() {
  g_case = "write_gate";
  DalyKeyWriteInputs in = goodInputs();
  DalyKeyWriteGate g = evaluateDalyKeyWrite(in);
  CHECK(g.decision == DalyKeyWriteDecision::START);
  CHECK_EQ(g.refusal, DalyKeyWriteRefusal::NONE);

  in = goodInputs(); in.maintenance_mode = false;
  expectRefused(in, DalyKeyWriteRefusal::NOT_IN_MAINTENANCE_MODE);
  in = goodInputs(); in.write_already_attempted = true;
  expectRefused(in, DalyKeyWriteRefusal::WRITE_ALREADY_ATTEMPTED);
  in = goodInputs(); in.bus_busy = true;
  expectRefused(in, DalyKeyWriteRefusal::BUS_BUSY);
  in = goodInputs(); in.telemetry_ok = false;
  expectRefused(in, DalyKeyWriteRefusal::BMS_COMM_NOT_OK);
  in = goodInputs(); in.telemetry_age_ms = kDalyKeyWriteMaxTelemetryAgeMs + 1;
  expectRefused(in, DalyKeyWriteRefusal::BMS_COMM_NOT_OK);
  in = goodInputs(); in.telemetry_age_ms = kDalyKeyWriteMaxTelemetryAgeMs;
  CHECK(evaluateDalyKeyWrite(in).decision == DalyKeyWriteDecision::START);
  in = goodInputs(); in.alarms_clear = false;
  expectRefused(in, DalyKeyWriteRefusal::BMS_ALARM_ACTIVE);
  in = goodInputs(); in.snapshot.valid = false;
  expectRefused(in, DalyKeyWriteRefusal::NO_KEY_SNAPSHOT);
  in = goodInputs(); in.last_key_read_ok = false;  // an older valid snapshot is not enough
  expectRefused(in, DalyKeyWriteRefusal::NO_KEY_SNAPSHOT);
  in = goodInputs(); in.now_ms = in.snapshot.sampled_at_ms + kDalyKeyWriteMaxSnapshotAgeMs + 1;
  expectRefused(in, DalyKeyWriteRefusal::KEY_SNAPSHOT_STALE);
  in = goodInputs(); in.now_ms = in.snapshot.sampled_at_ms + kDalyKeyWriteMaxSnapshotAgeMs;
  CHECK(evaluateDalyKeyWrite(in).decision == DalyKeyWriteDecision::START);
  CHECK_EQ(kDalyKeyWriteMaxSnapshotAgeMs, 30000);

  // Only 0x0055 may be changed; 0x005A is a no-op; everything else refuses.
  in = goodInputs(); in.snapshot.key_logic_raw = 0x005A;
  g = evaluateDalyKeyWrite(in);
  CHECK(g.decision == DalyKeyWriteDecision::ALREADY_CONFIGURED);
  const uint16_t others[] = {0x00A5, 0x00AA, 0x00A6, 0x0000, 0x00FF, 0x5500, 0x5A00, 0xFFFF};
  for (uint16_t raw : others) {
    in = goodInputs(); in.snapshot.key_logic_raw = raw;
    expectRefused(in, DalyKeyWriteRefusal::KEY_LOGIC_NOT_DISABLED);
  }

  in = goodInputs(); in.snapshot.charge_mos_control = 0;
  expectRefused(in, DalyKeyWriteRefusal::MOS_CONTROL_NOT_ENABLED);
  in = goodInputs(); in.snapshot.discharge_mos_control = 0;
  expectRefused(in, DalyKeyWriteRefusal::MOS_CONTROL_NOT_ENABLED);
  in = goodInputs(); in.snapshot.discharge_mos_control = 2;
  expectRefused(in, DalyKeyWriteRefusal::MOS_CONTROL_NOT_ENABLED);

  // Evaluation order is deterministic: the first failing check is reported.
  in = goodInputs(); in.maintenance_mode = false; in.bus_busy = true; in.alarms_clear = false;
  expectRefused(in, DalyKeyWriteRefusal::NOT_IN_MAINTENANCE_MODE);
  in = goodInputs(); in.bus_busy = true; in.snapshot.key_logic_raw = 0x005A;
  expectRefused(in, DalyKeyWriteRefusal::BUS_BUSY);
}

static void test_readback_classification() {
  g_case = "readback_classification";
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::OK, 0x005A), DalyKeyReadback::VERIFIED);
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::OK, 0x0055),
           DalyKeyReadback::PENDING_RESTART);
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::OK, 0x00AA), DalyKeyReadback::MISMATCH);
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::OK, 0x0000), DalyKeyReadback::MISMATCH);
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::TIMEOUT, 0x005A),
           DalyKeyReadback::READ_FAILED);
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::CRC_FAIL, 0x005A),
           DalyKeyReadback::READ_FAILED);
  CHECK_EQ(classifyDalyKeyReadback(DalyKeyReadResult::BAD_HEADER, 0x005A),
           DalyKeyReadback::READ_FAILED);
}

static void test_write_tracker() {
  g_case = "write_tracker";
  DalyKeyWriteTracker t;
  CHECK_EQ(t.status().state, DalyKeyWriteState::NOT_REQUESTED);
  CHECK(!t.status().transmitted);

  t.refuse(DalyKeyWriteRefusal::KEY_SNAPSHOT_STALE);
  CHECK_EQ(t.status().state, DalyKeyWriteState::REFUSED);
  CHECK_EQ(t.status().last_refusal, DalyKeyWriteRefusal::KEY_SNAPSHOT_STALE);

  t.accept();
  CHECK_EQ(t.status().state, DalyKeyWriteState::PENDING);
  CHECK(t.outstanding());
  CHECK_EQ(t.status().ack, DalyKeyWriteAck::PENDING);
  // A second command while pending is refused without clobbering the record.
  t.refuse(DalyKeyWriteRefusal::BUS_BUSY);
  CHECK_EQ(t.status().state, DalyKeyWriteState::PENDING);
  CHECK_EQ(t.status().last_refusal, DalyKeyWriteRefusal::BUS_BUSY);

  t.markTransmitted();
  t.onAck(DalyKeyWriteAck::OK, 8, 5000);
  // An acknowledgement alone is not verification.
  CHECK_EQ(t.status().ack, DalyKeyWriteAck::OK);
  CHECK_EQ(t.status().readback, DalyKeyReadback::PENDING);
  CHECK_EQ(t.status().state, DalyKeyWriteState::PENDING);

  t.onReadback(DalyKeyReadResult::OK, 0x005A, 5600);
  CHECK_EQ(t.status().state, DalyKeyWriteState::COMPLETE);
  CHECK_EQ(t.status().readback, DalyKeyReadback::VERIFIED);
  CHECK_EQ(t.status().readback_raw, 0x005A);
  CHECK(!t.outstanding());

  // Later zero-TX outcomes never overwrite the transmitted write's record.
  t.refuse(DalyKeyWriteRefusal::WRITE_ALREADY_ATTEMPTED);
  t.alreadyConfigured();
  CHECK_EQ(t.status().state, DalyKeyWriteState::COMPLETE);
  CHECK_EQ(t.status().readback, DalyKeyReadback::VERIFIED);
  CHECK(t.status().transmitted);

  // Read-back of the old value after an OK acknowledgement is not a pass.
  DalyKeyWriteTracker u;
  u.accept(); u.markTransmitted(); u.onAck(DalyKeyWriteAck::OK, 8, 1);
  u.onReadback(DalyKeyReadResult::OK, 0x0055, 2);
  CHECK_EQ(u.status().readback, DalyKeyReadback::PENDING_RESTART);

  // A failed acknowledgement is still followed by (and reports) the read-back.
  DalyKeyWriteTracker v;
  v.accept(); v.markTransmitted(); v.onAck(DalyKeyWriteAck::TIMEOUT, 0, 1);
  CHECK_EQ(v.status().readback, DalyKeyReadback::PENDING);
  v.onReadback(DalyKeyReadResult::OK, 0x005A, 2);
  CHECK_EQ(v.status().ack, DalyKeyWriteAck::TIMEOUT);
  CHECK_EQ(v.status().readback, DalyKeyReadback::VERIFIED);

  // A failed read-back carries no value (never the previous snapshot's).
  DalyKeyWriteTracker x;
  x.accept(); x.markTransmitted(); x.onAck(DalyKeyWriteAck::OK, 8, 1);
  x.onReadback(DalyKeyReadResult::TIMEOUT, 0x0055, 2);
  CHECK_EQ(x.status().readback, DalyKeyReadback::READ_FAILED);
  CHECK_EQ(x.status().readback_raw, 0);
  CHECK_EQ(x.status().state, DalyKeyWriteState::COMPLETE);

  // Refused by the last check before transmitting: zero TX, REFUSED.
  DalyKeyWriteTracker w;
  w.accept();
  DalyKeyWriteGate gate;
  gate.refusal = DalyKeyWriteRefusal::BMS_ALARM_ACTIVE;
  w.cancelBeforeTransmit(gate);
  CHECK_EQ(w.status().state, DalyKeyWriteState::REFUSED);
  CHECK_EQ(w.status().last_refusal, DalyKeyWriteRefusal::BMS_ALARM_ACTIVE);
  CHECK(!w.status().transmitted);
  CHECK_EQ(w.status().ack, DalyKeyWriteAck::NONE);
}

static void test_scheduler_write_and_readback() {
  g_case = "scheduler_write_and_readback";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;
  DalyRequest q = DalyRequest::TELEMETRY;

  CHECK(bus.startNext(2000, &r));               // telemetry in flight
  CHECK(r == DalyRequest::TELEMETRY);
  CHECK(bus.requestKeyLogicDischargeWrite());   // queued behind it
  CHECK(bus.queuedOperatorRequest(&q));
  CHECK(q == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE);
  CHECK(!bus.requestKeyConfigRead());           // one operator transaction at a time
  CHECK(!bus.requestKeyLogicDischargeWrite());
  CHECK(!bus.startNext(2100, &r));              // no overlap with the active poll
  bus.finish(2150);
  CHECK(!bus.startNext(2449, &r));              // quiet gap
  CHECK(bus.startNext(2450, &r));
  CHECK(r == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE);
  CHECK(!bus.queuedOperatorRequest(&q));
  bus.markSent(2458);
  CHECK(!bus.deadlinePassed(3458));
  CHECK(bus.deadlinePassed(3459));             // 1000 ms write deadline
  bus.finish(2500);                            // acknowledged

  CHECK(bus.requestKeyConfigRead());           // the chained read-back
  CHECK(bus.startNext(2800, &r));
  CHECK(r == DalyRequest::KEY_CONFIG);
  bus.finish(3100);
  CHECK(!bus.operatorTransactionOutstanding());
  CHECK(!bus.startNext(3999, &r));
  CHECK(bus.startNext(4000, &r));              // telemetry resumes on its own
  CHECK(r == DalyRequest::TELEMETRY);
}

static void test_scheduler_write_timeout_then_resumes() {
  g_case = "scheduler_write_timeout_then_resumes";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;

  CHECK(bus.startNext(2000, &r));
  bus.finish(2150);
  CHECK(bus.requestKeyLogicDischargeWrite());
  CHECK(bus.startNext(4000, &r));              // due poll deferred for the write
  CHECK(r == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE);
  bus.markSent(4008);
  CHECK(bus.deadlinePassed(5009));
  bus.finish(5009);                            // timed out
  CHECK(bus.requestKeyConfigRead());           // read-back still requested
  CHECK(bus.startNext(5309, &r));
  CHECK(r == DalyRequest::TELEMETRY);          // but the deferred poll goes first
  bus.finish(5450);
  CHECK(bus.startNext(5750, &r));
  CHECK(r == DalyRequest::KEY_CONFIG);
  bus.finish(6050);
  CHECK(bus.startNext(7309, &r));              // cadence re-anchored at 5309
  CHECK(r == DalyRequest::TELEMETRY);
}

static void test_scheduler_cancel_queued_write() {
  g_case = "scheduler_cancel_queued_write";
  DalyBusScheduler bus;
  DalyRequest r = DalyRequest::TELEMETRY;
  CHECK(bus.requestKeyLogicDischargeWrite());
  bus.cancelQueuedOperatorRequest();            // refused at the last check
  CHECK(!bus.operatorTransactionOutstanding());
  CHECK(!bus.startNext(1900, &r));              // nothing goes out
  CHECK(bus.startNext(2000, &r));
  CHECK(r == DalyRequest::TELEMETRY);           // only telemetry
}

static void test_pre_transmit_mode_recheck() {
  g_case = "pre_transmit_mode_recheck";
  DalyRequest r = DalyRequest::TELEMETRY;

  // Accepted in MAINTENANCE, then the operator switched to RUN before the
  // write reached the idle bus boundary: it must never leave the UART.
  {
    DalyBusScheduler bus;
    DalyKeyWriteTracker t;
    DalyKeyWriteInputs live = goodInputs();
    CHECK(evaluateDalyKeyWrite(live).decision == DalyKeyWriteDecision::START);
    CHECK(bus.requestKeyLogicDischargeWrite());
    t.accept();

    live.maintenance_mode = false;  // mode read on the pre-transmit update
    CHECK(!dalyKeyWritePreTransmitCheck(&bus, &t, live));
    CHECK_EQ(t.status().state, DalyKeyWriteState::REFUSED);
    CHECK_EQ(t.status().last_refusal, DalyKeyWriteRefusal::NOT_IN_MAINTENANCE_MODE);
    CHECK(!t.status().transmitted);
    CHECK_EQ(t.status().ack, DalyKeyWriteAck::NONE);
    CHECK(!bus.operatorTransactionOutstanding());
    CHECK(!bus.startNext(1999, &r));            // nothing at all goes out early
    CHECK(bus.startNext(2000, &r));
    CHECK(r == DalyRequest::TELEMETRY);          // telemetry carries on; no write
    bus.finish(2150);
    CHECK(!bus.startNext(3999, &r));
    CHECK(bus.startNext(4000, &r));
    CHECK(r == DalyRequest::TELEMETRY);
  }

  // Still MAINTENANCE at the pre-transmit check: the write proceeds.
  {
    DalyBusScheduler bus;
    DalyKeyWriteTracker t;
    const DalyKeyWriteInputs live = goodInputs();
    CHECK(bus.requestKeyLogicDischargeWrite());
    t.accept();
    CHECK(dalyKeyWritePreTransmitCheck(&bus, &t, live));
    CHECK_EQ(t.status().state, DalyKeyWriteState::PENDING);
    CHECK(bus.startNext(1600, &r));
    CHECK(r == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE);
  }

  // Deferred behind a due poll, then RUN on the next boundary: the check
  // runs again and cancels it then.
  {
    DalyBusScheduler bus;
    DalyKeyWriteTracker t;
    DalyKeyWriteInputs live = goodInputs();
    CHECK(bus.requestKeyConfigRead());
    CHECK(bus.startNext(2000, &r));              // a KEY read runs first
    bus.finish(2300);
    CHECK(bus.requestKeyLogicDischargeWrite());
    t.accept();
    CHECK(dalyKeyWritePreTransmitCheck(&bus, &t, live));
    CHECK(bus.startNext(4000, &r));              // due poll goes before a 2nd operator txn
    CHECK(r == DalyRequest::TELEMETRY);
    bus.finish(4150);
    live.maintenance_mode = false;
    CHECK(!dalyKeyWritePreTransmitCheck(&bus, &t, live));
    CHECK_EQ(t.status().last_refusal, DalyKeyWriteRefusal::NOT_IN_MAINTENANCE_MODE);
    CHECK(!bus.operatorTransactionOutstanding());
    CHECK(!bus.startNext(4450, &r));             // nothing queued, poll not due
    CHECK(bus.startNext(6000, &r));
    CHECK(r == DalyRequest::TELEMETRY);          // only telemetry ever follows
    CHECK(!t.status().transmitted);
  }

  // Any other DALY-state change is caught the same way (here: an alarm).
  {
    DalyBusScheduler bus;
    DalyKeyWriteTracker t;
    DalyKeyWriteInputs live = goodInputs();
    CHECK(bus.requestKeyLogicDischargeWrite());
    t.accept();
    live.alarms_clear = false;
    CHECK(!dalyKeyWritePreTransmitCheck(&bus, &t, live));
    CHECK_EQ(t.status().last_refusal, DalyKeyWriteRefusal::BMS_ALARM_ACTIVE);
  }

  // No write queued: the check touches nothing (a queued KEY read survives).
  {
    DalyBusScheduler bus;
    DalyKeyWriteTracker t;
    DalyKeyWriteInputs live = goodInputs();
    live.maintenance_mode = false;
    CHECK(bus.requestKeyConfigRead());
    CHECK(dalyKeyWritePreTransmitCheck(&bus, &t, live));
    CHECK(bus.operatorTransactionOutstanding());
    CHECK_EQ(t.status().state, DalyKeyWriteState::NOT_REQUESTED);
  }
}

// The BMS is now commissioned (0x0120 = 0x005A, live 2026-09-19). The write
// command is retained for a replaced or factory-reset BMS, so prove it can
// no longer transmit anything in the configured state.
static void test_post_commissioning_state_cannot_write() {
  g_case = "post_commissioning_state_cannot_write";
  DalyRequest r = DalyRequest::TELEMETRY;

  DalyKeyWriteInputs live = goodInputs();
  live.snapshot.key_logic_raw = 0x005A;               // what the unit reads today
  live.snapshot.key_logic = DalyKeyLogic::DISCHARGE;
  const DalyKeyWriteGate gate = evaluateDalyKeyWrite(live);
  CHECK(gate.decision == DalyKeyWriteDecision::ALREADY_CONFIGURED);
  CHECK(gate.decision != DalyKeyWriteDecision::START);

  // Accepted while still 0x0055, but the register reads 0x005A by the time
  // the bus is idle: the queued write is dropped with zero bytes sent.
  DalyBusScheduler bus;
  DalyKeyWriteTracker t;
  CHECK(bus.requestKeyLogicDischargeWrite());
  t.accept();
  CHECK(!dalyKeyWritePreTransmitCheck(&bus, &t, live));
  CHECK_EQ(t.status().state, DalyKeyWriteState::ALREADY_CONFIGURED);
  CHECK(!t.status().transmitted);
  CHECK(!bus.operatorTransactionOutstanding());
  CHECK(bus.startNext(2000, &r));
  CHECK(r == DalyRequest::TELEMETRY);                 // only telemetry ever follows

  // Only 0x0055 may ever start a write - every other recognized value, and
  // every unknown one, refuses.
  const uint16_t never[] = {0x005A, 0x00A5, 0x00AA, 0x00A6, 0x0000, 0x00FF, 0xFFFF, 0x5500};
  for (uint16_t raw : never) {
    DalyKeyWriteInputs in = goodInputs();
    in.snapshot.key_logic_raw = raw;
    CHECK(evaluateDalyKeyWrite(in).decision != DalyKeyWriteDecision::START);
  }
  DalyKeyWriteInputs disabled = goodInputs();
  disabled.snapshot.key_logic_raw = 0x0055;
  CHECK(evaluateDalyKeyWrite(disabled).decision == DalyKeyWriteDecision::START);
}

int main() {
  std::printf("MATDOG DALY protocol / KEY probe / KEY write offline tests\n");

  test_crc16_known_vectors();
  test_request_frames_exact();
  test_response_shapes();
  test_register_offsets();
  test_key_config_valid_response();
  test_key_logic_mappings();
  test_sleep_time_width();
  test_mos_controls_from_their_own_registers();
  test_bad_header_rejected();
  test_crc_failure_rejected();
  test_truncated_response_rejected();
  test_failed_read_keeps_previous_snapshot();
  test_key_read_result_mapping();
  test_telemetry_parser_unchanged();
  test_telemetry_rejections_unchanged();
  test_scheduler_telemetry_cadence();
  test_scheduler_key_waits_for_active_telemetry();
  test_scheduler_key_defers_one_poll_then_resumes();
  test_scheduler_key_timeout_then_resumes();
  test_scheduler_key_cannot_starve_telemetry();
  test_scheduler_simulated_bus();
  test_write_frame_exact();
  test_write_ack_validation();
  test_write_gate();
  test_readback_classification();
  test_write_tracker();
  test_scheduler_write_and_readback();
  test_scheduler_write_timeout_then_resumes();
  test_scheduler_cancel_queued_write();
  test_pre_transmit_mode_recheck();
  test_post_commissioning_state_cannot_write();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("DALY_PROTOCOL_TESTS = FAIL\n");
    return 1;
  }
  std::printf("DALY_PROTOCOL_TESTS = PASS\n");
  return 0;
}
