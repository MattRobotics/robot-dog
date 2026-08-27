/*
 * MATDOG ST-3215-C018 CHARACTERIZE V1 — ESP32-S3 firmware
 *
 * PURPOSE — exactly two measurements, nothing else:
 *
 *   A. the prime-only positional effect at the EXACT final parameters
 *      TorqueLimit 300 / Speed 365 / Acc 50;
 *   B. a real servo-rail power-cycle disappearance / reappearance trace.
 *
 * THIS IS NOT A PROVISIONER. It cannot provision. It has no allocation table,
 * no target IDs, no canonical profile table and no unlock primitive.
 *
 * WHAT IT CAN WRITE — the complete, statically enumerated list:
 *
 *      0x28 TorqueEnable = 0    (RAM, value 0 ONLY, never 1, never 128)
 *      0x30 TorqueLimit  = 300  (RAM, that one value ONLY)
 *      WritePosEx(<position it just read itself>, 365, 50)
 *
 * There is no other write of any kind. No EEPROM write. No unlock. No ID
 * write. No PositionOffset write. No centering move. No arbitrary position.
 * No mid-position calibration. No factory reset. No broadcast write.
 *
 * COMMAND SURFACE — exactly four commands, nothing else parses:
 *      @ARM
 *      @PRIME <SESSION_TOKEN>
 *      @TRACE <SESSION_TOKEN>
 *      @HELP
 *
 * @ARM performs ZERO servo writes. It scans 0..253, gates, and emits the
 * as-found 71 bytes, so the host can make that evidence durable on disk
 * before @PRIME is even possible.
 *
 * THIS TOOL SELECTS NO CONSTANT. It reports measurements. PRIME_MAX_DELTA_TICKS,
 * COLD_ABSENCE_DEBOUNCE_MS and COLD_RETURN_STABLE_SAMPLES are chosen by a human
 * after reviewing the evidence this tool produces, and the provisioner's
 * hardware-freeze block is not touched by running it.
 *
 * Host   : matdog_st3215_characterize_v1.py
 * Design : CHARACTERIZATION_DESIGN.md
 */

#include <Arduino.h>
#include <SCServo.h>

// --------------------------------------------------------------------------
// Transport — identical to the frozen survey and provisioner firmware
// --------------------------------------------------------------------------

static constexpr int SERVO_TX_PIN = 17;
static constexpr int SERVO_RX_PIN = 18;
static constexpr uint32_t SERVO_BAUD = 1000000;

HardwareSerial ServoUART(1);
SMS_STS st;

static const char *FIRMWARE_NAME = "matdog_st3215_characterize_v1";
static const char *FIRMWARE_VERSION = "1.0.0";

// This harness measures. It never decides. Kept as an explicit, greppable
// declaration so that "did the characterizer pick a threshold?" has a
// one-line answer.
static constexpr bool CHARACTERIZER_SELECTS_CONSTANTS = false;

// --------------------------------------------------------------------------
// State map — 71 bytes, 0x00 .. 0x46 inclusive
// --------------------------------------------------------------------------

static constexpr uint8_t SNAPSHOT_START = 0x00;
static constexpr uint8_t SNAPSHOT_LEN = 71;

static constexpr uint8_t REG_MODEL = 0x03;           // 2 bytes, NOT 0x00
static constexpr uint8_t REG_ID = 0x05;
static constexpr uint8_t REG_BAUD = 0x06;
static constexpr uint8_t REG_RESPONSE_STATUS = 0x08;
static constexpr uint8_t REG_POSITION_OFFSET = 0x1F; // 2 bytes, read-only here
static constexpr uint8_t REG_TORQUE_ENABLE = 0x28;
static constexpr uint8_t REG_TORQUE_LIMIT = 0x30;    // 2 bytes, RAM
static constexpr uint8_t REG_LOCK = 0x37;            // read-only here
static constexpr uint8_t REG_PRESENT_POSITION = 0x38;
static constexpr uint8_t REG_PRESENT_VOLTAGE = 0x3E;
static constexpr uint8_t REG_PRESENT_TEMPERATURE = 0x3F;
static constexpr uint8_t REG_STATUS = 0x40;

static constexpr uint16_t EXPECTED_MODEL = 777;
static constexpr uint8_t EXPECTED_RESPONSE_STATUS = 1;
static constexpr uint8_t EXPECTED_BAUD = 0;

static constexpr int ENCODER_COUNTS = 4096;
static constexpr int HALF_TURN = 2048;
static constexpr int POSITION_OFFSET_MIN = -2048;
static constexpr int POSITION_OFFSET_MAX = 2047;

// --------------------------------------------------------------------------
// CHARACTERIZATION A — the exact final prime parameters
//
// Frozen by 04_MATDOG_C018_CANONICAL_PROFILE_V1.md section 6 and mandated by
// MATDOG_ST3215_PROVISIONER_V1_DESIGN.md section 8. Measuring at anything else
// would reproduce the gap that blocked the freeze in the first place.
// --------------------------------------------------------------------------

static constexpr uint16_t CHAR_TORQUE_LIMIT = 300;
static constexpr uint16_t CHAR_SPEED = 365;
static constexpr uint8_t CHAR_ACC = 50;

static constexpr uint8_t PRIME_REPETITIONS = 5;

// --------------------------------------------------------------------------
// Sampling and guard constants — REUSED from frozen QC V6.1
// (matdog_servo_commissioning.ino, validated across the 26-run campaign on
// this exact 17-servo population, same bus, same 1 Mbps, same SCServo build).
// --------------------------------------------------------------------------

static constexpr uint32_t MON_PERIOD_US = 2000;            // QC_FAST_PERIOD_US
static constexpr int MON_STATIONARY_SPEED = 10;            // QC characterization
static constexpr uint32_t MON_STABLE_SAMPLES = 40;         // QC characterization
static constexpr int MON_OVERCURRENT_RAW = 308;            // QC_OVERCURRENT_2A_RAW
static constexpr uint32_t MON_OVERCURRENT_SAMPLES = 50;
static constexpr int MON_STALL_CURRENT_RAW = 416;          // QC_STALL_CURRENT_RAW
static constexpr uint32_t MON_STALL_CURRENT_SAMPLES = 10;
static constexpr int MON_THERMAL_LIMIT_C = 70;
static constexpr int MON_VOLTAGE_MIN = 40;
static constexpr int MON_VOLTAGE_MAX = 140;
static constexpr uint32_t MON_VOLTAGE_SAMPLES = 50;
static constexpr uint32_t MON_STATUS_SAMPLES = 25;         // QC_STATUS_PERSIST_SAMPLES
static constexpr uint32_t MON_TELEMETRY_LOSS_SAMPLES = 25;
static constexpr uint32_t WRITE_SETTLE_MS = 20;            // QC forceTorqueOffVerified

// --------------------------------------------------------------------------
// SAFETY ABORT — NOT a candidate value for PRIME_MAX_DELTA_TICKS
//
// Provenance: QC V6.1 aborts a monitored move on 16 ticks of travel against
// the commanded direction (directionalTravel < -16). That is the one validated
// MATDOG precedent for "this much unintended travel is an abort, not a
// tolerance", and it is reused here unchanged as a gross-motion safety stop.
//
// The threshold this harness exists to inform will be selected from the
// measured deltas reported below and is expected to be far smaller. Do not
// promote this number into PRIME_MAX_DELTA_TICKS.
// --------------------------------------------------------------------------

static constexpr int PRIME_SAFETY_ABORT_TICKS = 16;

// Observation window bounds. NOT safety thresholds: torque is off, the EEPROM
// is never unlocked, and both bounds only decide how long we look.
static constexpr uint32_t PRIME_SETTLE_MIN_MS = 250;
static constexpr uint32_t PRIME_SETTLE_MAX_MS = 2000;
static constexpr uint32_t PRIME_INTER_REP_DWELL_MS = 500;

// --------------------------------------------------------------------------
// CHARACTERIZATION B — trace parameters
//
// All of these are DATA-COLLECTION parameters. None of them is, or may become,
// COLD_ABSENCE_DEBOUNCE_MS or COLD_RETURN_STABLE_SAMPLES. The trace classifies
// nothing: it records every absence episode it sees, with measured timestamps,
// and a human reads the episode table afterwards.
//
// Resolution note: SCSerial::IOTimeOut is 100 ms in the installed library, so a
// failing Ping costs ~100 ms. Absence timing therefore cannot be resolved finer
// than ~100 ms by any Ping-based detector, this one or the provisioner's.
// --------------------------------------------------------------------------

static constexpr uint32_t TRACE_POLL_PERIOD_MS = 20;
static constexpr uint32_t TRACE_IO_TIMEOUT_MS = 100;       // SCSerial default
static constexpr uint32_t TRACE_ARM_WINDOW_MS = 300000;    // operator patience
static constexpr uint32_t TRACE_OBSERVE_MIN_MS = 20000;
static constexpr uint32_t TRACE_OBSERVE_MAX_MS = 60000;
static constexpr uint32_t TRACE_HEARTBEAT_MS = 1000;
static constexpr uint8_t TRACE_MAX_TRACES = 2;
static constexpr uint8_t TRACE_MAX_EPISODES = 8;

// Data-collection endpoint only, explicitly not the final stability threshold.
static constexpr uint32_t TRACE_STABLE_TARGET = 20;

// --------------------------------------------------------------------------
// Session state
// --------------------------------------------------------------------------

enum SessionState : uint8_t {
  SESSION_IDLE = 0,
  SESSION_ARMED,
  SESSION_PRIME_DONE,
  SESSION_TRACE_DONE,
  SESSION_FAULT,
};

// Stage gate for the write allowlist. A write is refused unless the current
// stage is the one that owns that register.
enum Stage : uint8_t {
  STAGE_NONE = 0,
  STAGE_PRIME,      // TorqueEnable=0, TorqueLimit=300, WritePosEx(current)
  STAGE_RECOVERY,   // TorqueEnable=0 only, on a failure path
};

static SessionState sessionState = SESSION_IDLE;
static Stage stage = STAGE_NONE;

static uint8_t sessionId = 0;
static char sessionToken[9] = {0};
static uint32_t sessionBeforeDigest = 0;
static uint8_t sessionTraceCount = 0;

// --------------------------------------------------------------------------
// Result types
//
// Declared before any function because the .ino preprocessor injects function
// prototypes above the body of this file.
// --------------------------------------------------------------------------

struct ScanResult {
  int found;
  int id;
  int model;
  uint8_t pingStatus;
  uint8_t modelStatus;
};

struct PrimeTiming {
  int32_t p0;
  uint32_t readStartUs;
  uint32_t readDoneUs;
  uint32_t primeStartUs;
  uint32_t primeDoneUs;
  uint32_t offStartUs;
  uint32_t offDoneUs;
  int primeAck;
  uint8_t primeStatus;
  int offAck;
  uint8_t offStatus;
  bool primeIssued;
  bool offIssued;
};

struct SettleResult {
  bool ok;
  const char *reason;
  bool settled;
  uint32_t samples;
  uint32_t elapsedUs;
  uint32_t settledUs;
  int firstPos;
  int finalPos;
  int32_t peakAbsDelta;
  int32_t minDelta;
  int32_t maxDelta;
  int peakAbsSpeed;
  int peakAbsLoad;
  int peakAbsCurrent;
  int minVoltage;
  int maxVoltage;
  int maxTemp;
  int lastCurrent;
  int lastVoltage;
  int lastTemp;
  uint8_t worstStatus;
};

struct AbsenceEpisode {
  uint32_t lastPresentMs;
  uint32_t firstAbsentMs;
  uint32_t firstReturnMs;
  uint32_t absentPolls;
  uint32_t durationMs;
  uint32_t gapMs;
  uint32_t returnToStableMs;
  uint32_t returnToStablePolls;
  uint32_t uncleanAfterReturn;
  uint32_t streakBreaks;
  bool ended;
  bool stableReached;
};

// --------------------------------------------------------------------------
// Small helpers
// --------------------------------------------------------------------------

static bool validId(int id) { return id >= 0 && id <= 253; }

static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// PositionOffset ONLY. Two's complement, little endian. Never shared with the
// sign-magnitude decoders the library uses for speed / load / current.
static int16_t i16le_twos(const uint8_t *p) {
  return (int16_t)u16le(p);
}

// Explicit floor modulo returning 0..modulus-1. Raw C++ % is never applied to
// a possibly-negative numerator anywhere in this file.
static int32_t floorMod(int32_t value, int32_t modulus) {
  int32_t result = value % modulus;
  if (result < 0) {
    result += modulus;
  }
  return result;
}

static int32_t physicalRaw(int32_t displayed, int32_t signedOffset) {
  return floorMod(displayed + signedOffset, ENCODER_COUNTS);
}

// Smallest signed distance on the 4096 circle. Safe across the 0/4095 wrap:
// circularDelta(0, 4095) == +1 and circularDelta(4095, 0) == -1. The exact
// half turn resolves negative (-2048), never +2048.
static int32_t circularDelta(int32_t a, int32_t b) {
  return floorMod(a - b + HALF_TURN, ENCODER_COUNTS) - HALF_TURN;
}

static uint32_t fnv1a32(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

// --------------------------------------------------------------------------
// WRITE CHOKE POINT
//
// Every servo write in this firmware goes through charWriteWord,
// charTorqueOffCommand or charPrimeWritePosEx. There is no other call site of
// any write primitive in this file, and the host runner asserts that
// statically against this source.
//
// The mid-position calibration helper in the library is implemented as a byte
// write of 128 to 0x28, so value 128 at 0x28 is refused here explicitly and
// the helper itself is never called.
// --------------------------------------------------------------------------

static bool charWriteAllowed(uint8_t addr, uint8_t width, uint16_t value) {
  if (addr == REG_TORQUE_ENABLE) {
    if (width != 1) return false;
    if (value != 0) return false;      // never 1, never 128
    return stage == STAGE_PRIME || stage == STAGE_RECOVERY;
  }
  if (addr == REG_TORQUE_LIMIT) {
    if (width != 2) return false;
    if (value != CHAR_TORQUE_LIMIT) return false;
    return stage == STAGE_PRIME;
  }
  return false;
}

// Word write, verified by exact readback. The only word register reachable is
// TorqueLimit, and only with value 300.
static bool charWriteWordVerified(uint8_t id, uint8_t addr, uint16_t value,
                                  const char *name) {
  if (!charWriteAllowed(addr, 2, value)) {
    Serial.printf("WRITE_REFUSED ADDR=0x%02X WIDTH=2 VALUE=%u STAGE=%u\n", addr,
                  value, (unsigned)stage);
    return false;
  }

  const int ack = st.writeWord(id, addr, value);
  const uint8_t writeStatus = st.Error;

  delay(WRITE_SETTLE_MS);

  const int readback = st.readWord(id, addr);
  const uint8_t readStatus = st.Error;

  // ACK semantics: 1 = success, 0 = failure. "< 0" is never used.
  const bool ok = (ack == 1) && (writeStatus == 0) &&
                  (readback == (int)value) && (readStatus == 0);

  Serial.printf(
      "WRITE ADDR=0x%02X NAME=%s WIDTH=2 EXPECT=%u ACK=%d STATUS=0x%02X "
      "READBACK=%d READ_STATUS=0x%02X RESULT=%s\n",
      addr, name, value, ack, writeStatus, readback, readStatus,
      ok ? "OK" : "FAIL");
  return ok;
}

// Torque OFF, command only. Deliberately does no delay and no printing, so
// that nothing lengthens the interval between the prime and the OFF. The
// caller verifies the readback afterwards.
static bool charTorqueOffCommand(uint8_t id, int &ack, uint8_t &status,
                                 uint32_t &startUs, uint32_t &doneUs) {
  ack = 0;
  status = 0xFF;
  startUs = 0;
  doneUs = 0;

  if (!charWriteAllowed(REG_TORQUE_ENABLE, 1, 0)) {
    Serial.printf("WRITE_REFUSED ADDR=0x%02X WIDTH=1 VALUE=0 STAGE=%u\n",
                  REG_TORQUE_ENABLE, (unsigned)stage);
    return false;
  }

  startUs = micros();
  ack = st.writeByte(id, REG_TORQUE_ENABLE, 0);
  status = st.Error;
  doneUs = micros();
  return true;
}

// Torque OFF, commanded and proven by exact readback.
static bool charTorqueOffVerified(uint8_t id, const char *context) {
  int ack = 0;
  uint8_t status = 0xFF;
  uint32_t startUs = 0;
  uint32_t doneUs = 0;

  if (!charTorqueOffCommand(id, ack, status, startUs, doneUs)) {
    Serial.printf("TORQUE_OFF CONTEXT=%s RESULT=REFUSED\n", context);
    return false;
  }

  delay(WRITE_SETTLE_MS);
  const int readback = st.readByte(id, REG_TORQUE_ENABLE);
  const uint8_t readStatus = st.Error;

  const bool ok = (ack == 1) && (status == 0) && (readback == 0) &&
                  (readStatus == 0);

  Serial.printf(
      "WRITE ADDR=0x%02X NAME=TorqueEnable WIDTH=1 EXPECT=0 ACK=%d "
      "STATUS=0x%02X READBACK=%d READ_STATUS=0x%02X RESULT=%s\n",
      REG_TORQUE_ENABLE, ack, status, readback, readStatus, ok ? "OK" : "FAIL");
  Serial.printf("TORQUE_OFF CONTEXT=%s RESULT=%s\n", context,
                ok ? "CONFIRMED" : "FAILED");
  return ok;
}

// THE ONLY MOTION PRIMITIVE.
//
// It takes no position argument. It reads PresentPosition itself, validates the
// 0..4095 domain itself, and commands that exact value. A caller therefore
// cannot express "go somewhere else": an arbitrary goal is not refused at
// runtime, it is unrepresentable. Speed and acceleration are pinned to the
// canonical constants and are not parameters either.
//
// Returns with no printing and no delay after the command, so the caller can
// command Torque OFF as the very next statement.
static bool charPrimeWritePosEx(uint8_t id, PrimeTiming &t) {
  t.p0 = -1;
  t.primeIssued = false;
  t.primeAck = 0;
  t.primeStatus = 0xFF;

  if (stage != STAGE_PRIME) {
    Serial.printf("MOTION_REFUSED STAGE=%u\n", (unsigned)stage);
    return false;
  }

  t.readStartUs = micros();
  const int present = st.ReadPos(id);
  const uint8_t readStatus = st.Error;
  t.readDoneUs = micros();

  if (present < 0 || present > 4095 || readStatus != 0) {
    Serial.printf("MOTION_REFUSED POSITION_READ=%d STATUS=0x%02X\n", present,
                  readStatus);
    return false;
  }
  t.p0 = present;

  t.primeStartUs = micros();
  t.primeAck = st.WritePosEx(id, (s16)present, CHAR_SPEED, CHAR_ACC);
  t.primeStatus = st.Error;
  t.primeDoneUs = micros();
  t.primeIssued = true;

  return t.primeAck == 1 && t.primeStatus == 0;
}

// --------------------------------------------------------------------------
// Read-only helpers
// --------------------------------------------------------------------------

static bool readSnapshot71(uint8_t id, uint8_t *raw) {
  const int ping = st.Ping(id);
  if (ping != id || st.Error != 0) {
    Serial.printf("SNAPSHOT_ABORT PING=%d STATUS=0x%02X\n", ping, st.Error);
    return false;
  }

  const int n = st.Read(id, SNAPSHOT_START, raw, SNAPSHOT_LEN);
  const uint8_t status = st.Error;

  if (n != SNAPSHOT_LEN || status != 0) {
    Serial.printf("SNAPSHOT_ABORT READ_LEN=%d STATUS=0x%02X\n", n, status);
    return false;
  }
  return true;
}

// Raw bytes are emitted BEFORE any decoding, so the host can persist the
// evidence before it interprets it.
static void emitSnapshot(const char *slot, uint8_t id, const uint8_t *raw) {
  Serial.printf("SNAPSHOT_BEGIN SLOT=%s ID=%u START=0x00 LEN=%u\n", slot, id,
                (unsigned)SNAPSHOT_LEN);

  Serial.print("RAW71_HEX=");
  for (uint8_t i = 0; i < SNAPSHOT_LEN; ++i) {
    Serial.printf("%02X", raw[i]);
  }
  Serial.println();

  Serial.printf("SNAPSHOT_RAW_COMPLETE SLOT=%s\n", slot);

  const int32_t offset = i16le_twos(raw + REG_POSITION_OFFSET);
  const int32_t displayed = u16le(raw + REG_PRESENT_POSITION);

  Serial.printf("SNAP SLOT=%s MODEL=%u ID=%u BAUD=%u RESPONSE_STATUS=%u\n", slot,
                u16le(raw + REG_MODEL), raw[REG_ID], raw[REG_BAUD],
                raw[REG_RESPONSE_STATUS]);
  Serial.printf(
      "SNAP SLOT=%s OFFSET=%ld TORQUE=%u LOCK=%u TORQUE_LIMIT=%u PRESENT=%ld "
      "PHYSICAL_RAW=%ld\n",
      slot, (long)offset, raw[REG_TORQUE_ENABLE], raw[REG_LOCK],
      u16le(raw + REG_TORQUE_LIMIT), (long)displayed,
      (long)physicalRaw(displayed, offset));
  Serial.printf("SNAP SLOT=%s VOLTAGE=%u TEMPERATURE=%u STATUS=0x%02X\n", slot,
                raw[REG_PRESENT_VOLTAGE], raw[REG_PRESENT_TEMPERATURE],
                raw[REG_STATUS]);

  Serial.printf("SNAPSHOT_END SLOT=%s\n", slot);
}

static bool captureAndEmit(const char *slot, uint8_t id, uint8_t *raw) {
  if (!readSnapshot71(id, raw)) {
    return false;
  }
  emitSnapshot(slot, id, raw);
  return true;
}

// --------------------------------------------------------------------------
// Fault convergence — always ends at Torque OFF, never moves anything
// --------------------------------------------------------------------------

static void enterFault(uint8_t id, const char *reason) {
  Serial.printf("FAULT REASON=%s\n", reason);

  stage = STAGE_RECOVERY;

  bool torqueOk = false;
  if (validId(id)) {
    torqueOk = charTorqueOffVerified(id, "RECOVERY");
    if (!torqueOk) {
      torqueOk = charTorqueOffVerified(id, "RECOVERY_RETRY");
    }
  }

  Serial.printf("RECOVERY TORQUE_OFF=%s\n", torqueOk ? "OK" : "FAILED");
  Serial.println("EEPROM_WRITES=NONE");
  Serial.println("REMOVE_SERVO_POWER_NOW");

  sessionState = SESSION_FAULT;
  stage = STAGE_NONE;
}

// --------------------------------------------------------------------------
// SCAN — read-only
// --------------------------------------------------------------------------

static ScanResult runFullScan() {
  ScanResult r = {0, -1, -1, 0xFF, 0xFF};

  Serial.println("SCAN_BEGIN RANGE=0..253");

  for (int id = 0; id <= 253; ++id) {
    const int ping = st.Ping((uint8_t)id);
    if (ping < 0) {
      continue;
    }
    const uint8_t pingStatus = st.Error;

    const int model = st.readWord((uint8_t)id, REG_MODEL);
    const uint8_t modelStatus = st.Error;

    Serial.printf(
        "FOUND ID=%d MODEL=%d PING_STATUS=0x%02X MODEL_STATUS=0x%02X\n", id,
        model, pingStatus, modelStatus);

    if (r.found == 0) {
      r.id = id;
      r.model = model;
      r.pingStatus = pingStatus;
      r.modelStatus = modelStatus;
    }
    r.found++;
  }

  Serial.printf("SCAN_RESULT FOUND=%d\n", r.found);
  Serial.println("SCAN_COMPLETE");
  return r;
}

// --------------------------------------------------------------------------
// DISCOVERY GATE — every item mandatory, evaluated before any write is possible
// --------------------------------------------------------------------------

static bool gateFail(const char *name, long actual, const char *expected) {
  Serial.printf("GATE %s ACTUAL=%ld EXPECTED=%s RESULT=FAIL\n", name, actual,
                expected);
  return false;
}

static bool discoveryGate(const ScanResult &scan, const uint8_t *raw) {
  if (scan.found != 1) {
    return gateFail("RESPONDERS", scan.found, "1");
  }
  if (scan.pingStatus != 0) {
    return gateFail("PING_STATUS", scan.pingStatus, "0x00");
  }
  if (scan.modelStatus != 0) {
    return gateFail("MODEL_STATUS", scan.modelStatus, "0x00");
  }
  if (scan.model != (int)EXPECTED_MODEL) {
    return gateFail("SCAN_MODEL", scan.model, "777");
  }
  if (u16le(raw + REG_MODEL) != EXPECTED_MODEL) {
    return gateFail("MODEL_0x03", u16le(raw + REG_MODEL), "777");
  }
  if (raw[REG_ID] != (uint8_t)scan.id) {
    return gateFail("ID_REGISTER", raw[REG_ID], "discovered id");
  }
  if (raw[REG_BAUD] != EXPECTED_BAUD) {
    return gateFail("BAUD_REGISTER", raw[REG_BAUD], "0");
  }
  if (raw[REG_RESPONSE_STATUS] != EXPECTED_RESPONSE_STATUS) {
    return gateFail("RESPONSE_STATUS", raw[REG_RESPONSE_STATUS], "1");
  }

  // PositionOffset is READ and REPORTED only. This harness never writes it and
  // never requires it to be zero: the unit under characterization has not been
  // provisioned. The bound only rejects a corrupt word.
  const int32_t offset = i16le_twos(raw + REG_POSITION_OFFSET);
  if (offset < POSITION_OFFSET_MIN || offset > POSITION_OFFSET_MAX) {
    return gateFail("POSITION_OFFSET", offset, "-2048..2047");
  }

  const int32_t displayed = u16le(raw + REG_PRESENT_POSITION);
  if (displayed < 0 || displayed > 4095) {
    return gateFail("PRESENT_POSITION", displayed, "0..4095");
  }
  if (raw[REG_LOCK] > 1) {
    return gateFail("LOCK", raw[REG_LOCK], "0 or 1");
  }
  if (raw[REG_TORQUE_ENABLE] != 0) {
    return gateFail("TORQUE_ENABLE", raw[REG_TORQUE_ENABLE], "0");
  }

  const int voltage = raw[REG_PRESENT_VOLTAGE];
  if (voltage < MON_VOLTAGE_MIN || voltage > MON_VOLTAGE_MAX) {
    return gateFail("VOLTAGE", voltage, "40..140");
  }
  const int temperature = raw[REG_PRESENT_TEMPERATURE];
  if (temperature > MON_THERMAL_LIMIT_C) {
    return gateFail("TEMPERATURE", temperature, "<=70");
  }
  if (raw[REG_STATUS] != 0) {
    return gateFail("STATUS", raw[REG_STATUS], "0x00");
  }

  Serial.println("GATE_RESULT PASS");
  return true;
}

// --------------------------------------------------------------------------
// @ARM — zero servo writes
// --------------------------------------------------------------------------

static void reportScope() {
  Serial.println("SCOPE_BEGIN");
  Serial.println("  MEASURES=PRIME_DELTA,POWER_CYCLE_TIMING");
  Serial.println("  WRITES=TORQUE_ENABLE_0,TORQUE_LIMIT_300,WRITEPOSEX_CURRENT");
  Serial.printf("  PRIME_PARAMS=TL:%u,SPEED:%u,ACC:%u\n", CHAR_TORQUE_LIMIT,
                CHAR_SPEED, CHAR_ACC);
  Serial.println("  EEPROM_WRITE=NEVER");
  Serial.println("  ID_WRITE=NEVER");
  Serial.println("  POSITION_OFFSET_WRITE=NEVER");
  Serial.println("  CENTERING_MOVE=NEVER");
  Serial.println("  ARBITRARY_POSITION=UNREPRESENTABLE");
  Serial.println("  TORQUE_ON=NEVER");
  Serial.printf("  SELECTS_CONSTANTS=%s\n",
                CHARACTERIZER_SELECTS_CONSTANTS ? "YES" : "NO");
  Serial.println("  PRIME_MAX_DELTA_TICKS=DEFERRED_TO_REVIEW");
  Serial.println("  COLD_ABSENCE_DEBOUNCE_MS=DEFERRED_TO_REVIEW");
  Serial.println("  COLD_RETURN_STABLE_SAMPLES=DEFERRED_TO_REVIEW");
  Serial.println("  PROVISIONER_HARDWARE_FREEZE=UNCHANGED_BLOCKED");
  Serial.println("SCOPE_END");
}

static void clearSession() {
  sessionState = SESSION_IDLE;
  stage = STAGE_NONE;
  sessionId = 0;
  sessionToken[0] = '\0';
  sessionBeforeDigest = 0;
  sessionTraceCount = 0;
}

static void runArm() {
  clearSession();

  Serial.println();
  Serial.println("====================================");
  Serial.println(" CHARACTERIZE ARM");
  Serial.println("====================================");

  reportScope();

  const ScanResult scan = runFullScan();

  if (scan.found != 1) {
    Serial.printf("ARM_ABORT: EXPECTED_ONE_RESPONDER_FOUND=%d\n", scan.found);
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("ARM_RESULT FAIL");
    return;
  }

  Serial.printf("ARM_DISCOVERED_ID=%d\n", scan.id);

  uint8_t raw[SNAPSHOT_LEN];
  if (!captureAndEmit("BEFORE", (uint8_t)scan.id, raw)) {
    Serial.println("ARM_ABORT: BEFORE_SNAPSHOT_FAILED");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("ARM_RESULT FAIL");
    return;
  }

  if (!discoveryGate(scan, raw)) {
    Serial.println("ARM_ABORT: DISCOVERY_GATE_FAILED");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("ARM_RESULT FAIL");
    return;
  }

  sessionId = (uint8_t)scan.id;
  sessionBeforeDigest = fnv1a32(raw, SNAPSHOT_LEN);

  const uint32_t entropy = esp_random();
  const uint32_t bound =
      entropy ^ sessionBeforeDigest ^ ((uint32_t)sessionId << 8);
  snprintf(sessionToken, sizeof(sessionToken), "%08lX", (unsigned long)bound);

  Serial.printf("BEFORE_DIGEST=%08lX\n", (unsigned long)sessionBeforeDigest);
  Serial.printf("PRIME_REPETITIONS=%u\n", PRIME_REPETITIONS);
  Serial.printf("PRIME_SAFETY_ABORT_TICKS=%d ROLE=GROSS_MOTION_ABORT "
                "NOT_A_THRESHOLD_CANDIDATE=1\n",
                PRIME_SAFETY_ABORT_TICKS);
  Serial.printf("SESSION_TOKEN=%s\n", sessionToken);
  Serial.println("EEPROM_WRITES=NONE");
  Serial.println("MOTION=NONE");
  Serial.println("ARM_RESULT PASS");

  sessionState = SESSION_ARMED;
  Serial.println("WAIT_PRIME");
}

// --------------------------------------------------------------------------
// CHARACTERIZATION A — prime only
// --------------------------------------------------------------------------

// Post-prime observation at the proven QC polling interval (2000 us, 500 Hz).
// Read-only. Never commands anything.
static bool primeSettleObserve(uint8_t id, int32_t p0, SettleResult &s) {
  s.ok = false;
  s.reason = "";
  s.settled = false;
  s.samples = 0;
  s.elapsedUs = 0;
  s.settledUs = 0;
  s.firstPos = -1;
  s.finalPos = -1;
  s.peakAbsDelta = 0;
  s.minDelta = 0;
  s.maxDelta = 0;
  s.peakAbsSpeed = 0;
  s.peakAbsLoad = 0;
  s.peakAbsCurrent = 0;
  s.minVoltage = 255;
  s.maxVoltage = 0;
  s.maxTemp = 0;
  s.lastCurrent = -1;
  s.lastVoltage = -1;
  s.lastTemp = -1;
  s.worstStatus = 0;

  const uint32_t start = micros();
  uint32_t next = start;

  uint32_t stableSamples = 0;
  uint32_t failures = 0;
  uint32_t overcurrentSamples = 0;
  uint32_t stallCurrentSamples = 0;
  uint32_t voltageBadSamples = 0;
  uint32_t statusBadSamples = 0;

  while ((uint32_t)(micros() - start) < PRIME_SETTLE_MAX_MS * 1000UL) {
    while ((int32_t)(micros() - next) < 0) {
      delayMicroseconds(20);
    }
    next += MON_PERIOD_US;

    if (st.FeedBack(id) < 0) {
      if (++failures >= MON_TELEMETRY_LOSS_SAMPLES) {
        s.reason = "TELEMETRY_LOST_PERSISTENT";
        s.elapsedUs = micros() - start;
        return false;
      }
      continue;
    }
    failures = 0;

    const int pos = st.ReadPos(-1);
    const int speed = st.ReadSpeed(-1);
    const int load = st.ReadLoad(-1);
    const int voltage = st.ReadVoltage(-1);
    const int temperature = st.ReadTemper(-1);
    const int moving = st.ReadMove(-1);
    const int current = st.ReadCurrent(-1);
    const uint8_t status = st.Error;

    if (pos < 0 || pos > 4095) {
      s.reason = "POSITION_OUT_OF_DOMAIN";
      s.elapsedUs = micros() - start;
      return false;
    }

    s.samples++;
    if (s.firstPos < 0) {
      s.firstPos = pos;
    }
    s.finalPos = pos;

    const int32_t delta = circularDelta(pos, p0);
    if (delta < s.minDelta) s.minDelta = delta;
    if (delta > s.maxDelta) s.maxDelta = delta;
    if (abs((int)delta) > s.peakAbsDelta) s.peakAbsDelta = abs((int)delta);

    if (abs(speed) > s.peakAbsSpeed) s.peakAbsSpeed = abs(speed);
    if (abs(load) > s.peakAbsLoad) s.peakAbsLoad = abs(load);
    if (abs(current) > s.peakAbsCurrent) s.peakAbsCurrent = abs(current);
    if (voltage < s.minVoltage) s.minVoltage = voltage;
    if (voltage > s.maxVoltage) s.maxVoltage = voltage;
    if (temperature > s.maxTemp) s.maxTemp = temperature;
    if (status > s.worstStatus) s.worstStatus = status;
    s.lastCurrent = current;
    s.lastVoltage = voltage;
    s.lastTemp = temperature;

    // ---- guards, all reused from frozen QC V6.1 ----------------------
    if (abs((int)delta) > PRIME_SAFETY_ABORT_TICKS) {
      s.reason = "GROSS_MOTION_ABORT";
      s.elapsedUs = micros() - start;
      return false;
    }
    if (abs(current) >= MON_STALL_CURRENT_RAW) {
      if (++stallCurrentSamples >= MON_STALL_CURRENT_SAMPLES) {
        s.reason = "STALL_CURRENT";
        s.elapsedUs = micros() - start;
        return false;
      }
    } else {
      stallCurrentSamples = 0;
    }
    if (abs(current) >= MON_OVERCURRENT_RAW) {
      if (++overcurrentSamples >= MON_OVERCURRENT_SAMPLES) {
        s.reason = "OVERCURRENT";
        s.elapsedUs = micros() - start;
        return false;
      }
    } else {
      overcurrentSamples = 0;
    }
    if (voltage < MON_VOLTAGE_MIN || voltage > MON_VOLTAGE_MAX) {
      if (++voltageBadSamples >= MON_VOLTAGE_SAMPLES) {
        s.reason = "VOLTAGE_OUT_OF_RANGE";
        s.elapsedUs = micros() - start;
        return false;
      }
    } else {
      voltageBadSamples = 0;
    }
    if (temperature > MON_THERMAL_LIMIT_C) {
      s.reason = "THERMAL_LIMIT";
      s.elapsedUs = micros() - start;
      return false;
    }
    if (status != 0) {
      if (++statusBadSamples >= MON_STATUS_SAMPLES) {
        s.reason = "STATUS_FAULT_PERSISTENT";
        s.elapsedUs = micros() - start;
        return false;
      }
    } else {
      statusBadSamples = 0;
    }

    // ---- QC characterization settle criterion ------------------------
    const uint32_t elapsed = micros() - start;
    const bool stationary = abs(speed) <= MON_STATIONARY_SPEED && moving == 0;

    if (stationary && elapsed >= PRIME_SETTLE_MIN_MS * 1000UL) {
      if (++stableSamples >= MON_STABLE_SAMPLES) {
        s.settled = true;
        s.settledUs = elapsed;
        s.elapsedUs = elapsed;
        s.ok = true;
        return true;
      }
    } else if (!stationary) {
      stableSamples = 0;
    }
  }

  s.elapsedUs = micros() - start;
  s.reason = "SETTLE_WINDOW_EXPIRED";
  return false;
}

static bool runPrimeRepetition(uint8_t id, uint8_t index, const char *&reason) {
  Serial.printf("PRIME_REP_BEGIN INDEX=%u OF=%u\n", index, PRIME_REPETITIONS);

  // 1. Torque OFF and exact readback.
  if (!charTorqueOffVerified(id, "PRE_PRIME")) {
    reason = "PRE_PRIME_TORQUE_OFF_FAILED";
    return false;
  }

  // 2. TorqueLimit = 300 and exact readback.
  if (!charWriteWordVerified(id, REG_TORQUE_LIMIT, CHAR_TORQUE_LIMIT,
                             "TorqueLimit")) {
    reason = "TORQUE_LIMIT_WRITE_FAILED";
    return false;
  }

  // Value-initialised: every timestamp is 0 unless the corresponding command
  // actually executed, so a refused primitive can never print stale timing.
  PrimeTiming t = {};
  t.offStatus = 0xFF;
  t.primeStatus = 0xFF;

  // 3/4/5. Read PresentPosition, validate the domain, command that position.
  // 6.     Torque is now assumed ON.
  // 7.     Command Torque OFF immediately, with nothing in between.
  const bool primeOk = charPrimeWritePosEx(id, t);
  t.offIssued = charTorqueOffCommand(id, t.offAck, t.offStatus, t.offStartUs,
                                     t.offDoneUs);

  // 8. Prove the OFF landed.
  delay(WRITE_SETTLE_MS);
  const int torqueReadback = st.readByte(id, REG_TORQUE_ENABLE);
  const uint8_t torqueReadStatus = st.Error;
  const bool offVerified = t.offIssued && t.offAck == 1 && t.offStatus == 0 &&
                           torqueReadback == 0 && torqueReadStatus == 0;

  Serial.printf("PRIME_P0 INDEX=%u P0=%ld\n", index, (long)t.p0);
  Serial.printf(
      "PRIME_CMD INDEX=%u GOAL=%ld SPEED=%u ACC=%u ACK=%d STATUS=0x%02X\n",
      index, (long)t.p0, CHAR_SPEED, CHAR_ACC, t.primeAck, t.primeStatus);
  Serial.printf(
      "PRIME_OFF INDEX=%u ACK=%d STATUS=0x%02X READBACK=%d READ_STATUS=0x%02X "
      "RESULT=%s\n",
      index, t.offAck, t.offStatus, torqueReadback, torqueReadStatus,
      offVerified ? "CONFIRMED" : "FAILED");

  if (t.primeIssued && t.offIssued) {
    Serial.printf(
        "PRIME_TIMING INDEX=%u READ_US=%lu WRITEPOSEX_US=%lu EXPOSURE_US=%lu "
        "TOTAL_US=%lu\n",
        index, (unsigned long)(t.readDoneUs - t.readStartUs),
        (unsigned long)(t.primeDoneUs - t.primeStartUs),
        (unsigned long)(t.offDoneUs - t.primeDoneUs),
        (unsigned long)(t.offDoneUs - t.primeStartUs));
  }

  if (!offVerified) {
    reason = "PRIME_SAFE_OFF_FAILED";
    return false;
  }
  if (!primeOk) {
    reason = "PRIME_WRITE_ACK_FAILED";
    return false;
  }

  // 9/10/11. Observe at the proven QC interval, then report.
  SettleResult s;
  const bool settleOk = primeSettleObserve(id, t.p0, s);

  const int32_t deltaSettled =
      s.finalPos >= 0 ? circularDelta(s.finalPos, t.p0) : 0;
  const int32_t deltaImmediate =
      s.firstPos >= 0 ? circularDelta(s.firstPos, t.p0) : 0;

  Serial.printf(
      "PRIME_DELTA INDEX=%u P0=%ld P_FIRST=%d P_SETTLED=%d "
      "DELTA_IMMEDIATE=%ld DELTA_TICKS=%ld PEAK_ABS_DELTA=%ld MIN_DELTA=%ld "
      "MAX_DELTA=%ld\n",
      index, (long)t.p0, s.firstPos, s.finalPos, (long)deltaImmediate,
      (long)deltaSettled, (long)s.peakAbsDelta, (long)s.minDelta,
      (long)s.maxDelta);
  Serial.printf(
      "PRIME_TELEMETRY INDEX=%u SAMPLES=%lu ELAPSED_US=%lu SETTLED=%s "
      "SETTLED_US=%lu PEAK_ABS_SPEED=%d PEAK_ABS_LOAD=%d PEAK_ABS_CURRENT=%d "
      "CURRENT=%d VOLTAGE=%d TEMPERATURE=%d MIN_VOLTAGE=%d MAX_VOLTAGE=%d "
      "MAX_TEMPERATURE=%d STATUS=0x%02X\n",
      index, (unsigned long)s.samples, (unsigned long)s.elapsedUs,
      s.settled ? "YES" : "NO", (unsigned long)s.settledUs, s.peakAbsSpeed,
      s.peakAbsLoad, s.peakAbsCurrent, s.lastCurrent, s.lastVoltage, s.lastTemp,
      s.minVoltage, s.maxVoltage, s.maxTemp, s.worstStatus);

  if (!settleOk) {
    reason = s.reason;
    return false;
  }

  Serial.printf("PRIME_REP_RESULT INDEX=%u RESULT=OK\n", index);
  return true;
}

static void runPrime(const char *token) {
  if (sessionState != SESSION_ARMED) {
    Serial.println("PRIME_ABORT: NO_ARMED_SESSION");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }
  if (strncmp(token, sessionToken, sizeof(sessionToken)) != 0) {
    Serial.println("PRIME_ABORT: TOKEN_MISMATCH");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }

  const uint8_t id = sessionId;

  Serial.println();
  Serial.println("====================================");
  Serial.println(" CHARACTERIZATION A — PRIME ONLY");
  Serial.println("====================================");
  Serial.printf("PRIME_PARAMS TORQUE_LIMIT=%u SPEED=%u ACC=%u REPETITIONS=%u\n",
                CHAR_TORQUE_LIMIT, CHAR_SPEED, CHAR_ACC, PRIME_REPETITIONS);
  Serial.printf("PRIME_SAMPLING PERIOD_US=%lu SETTLE_SPEED=%d "
                "SETTLE_SAMPLES=%lu MIN_MS=%lu MAX_MS=%lu\n",
                (unsigned long)MON_PERIOD_US, MON_STATIONARY_SPEED,
                (unsigned long)MON_STABLE_SAMPLES,
                (unsigned long)PRIME_SETTLE_MIN_MS,
                (unsigned long)PRIME_SETTLE_MAX_MS);

  // Cheap read-only re-confirmation that we are still talking to the same,
  // safe servo. The full gate ran at @ARM.
  if (st.Ping(id) != id || st.Error != 0) {
    enterFault(id, "PRIME_PRECONDITION_PING_FAILED");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }
  if (st.readWord(id, REG_MODEL) != (int)EXPECTED_MODEL || st.Error != 0) {
    enterFault(id, "PRIME_PRECONDITION_MODEL_MISMATCH");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }
  if (st.readByte(id, REG_TORQUE_ENABLE) != 0 || st.Error != 0) {
    enterFault(id, "PRIME_PRECONDITION_TORQUE_NOT_OFF");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }

  stage = STAGE_PRIME;

  for (uint8_t i = 1; i <= PRIME_REPETITIONS; ++i) {
    const char *reason = "UNKNOWN";
    if (!runPrimeRepetition(id, i, reason)) {
      Serial.printf("PRIME_REP_RESULT INDEX=%u RESULT=FAIL REASON=%s\n", i,
                    reason);
      enterFault(id, reason);
      Serial.println("PRIME_RESULT FAIL");
      return;
    }
    if (i < PRIME_REPETITIONS) {
      delay(PRIME_INTER_REP_DWELL_MS);
    }
  }

  // Final proof that the servo is left with torque off.
  if (!charTorqueOffVerified(id, "POST_PRIME")) {
    enterFault(id, "POST_PRIME_TORQUE_OFF_FAILED");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }

  stage = STAGE_NONE;

  uint8_t raw[SNAPSHOT_LEN];
  if (!captureAndEmit("AFTER_PRIME", id, raw)) {
    enterFault(id, "AFTER_PRIME_SNAPSHOT_FAILED");
    Serial.println("PRIME_RESULT FAIL");
    return;
  }

  Serial.println("PRIME_MAX_DELTA_TICKS_SELECTION=DEFERRED_TO_REVIEW");
  Serial.println("EEPROM_WRITES=NONE");
  Serial.println("PRIME_RESULT PASS");

  sessionState = SESSION_PRIME_DONE;
  Serial.println("WAIT_TRACE");
}

// --------------------------------------------------------------------------
// CHARACTERIZATION B — true power-cycle trace, strictly read-only
// --------------------------------------------------------------------------

static void emitEpisodes(const AbsenceEpisode *eps, uint8_t count,
                         uint8_t traceIndex) {
  for (uint8_t i = 0; i < count; ++i) {
    const AbsenceEpisode &e = eps[i];
    Serial.printf(
        "TRACE_EPISODE TRACE=%u INDEX=%u LAST_PRESENT_T_MS=%lu "
        "FIRST_ABSENT_T_MS=%lu FIRST_RETURN_T_MS=%lu ABSENT_POLLS=%lu "
        "DURATION_MS=%lu GAP_MS=%lu ENDED=%d STABLE=%d "
        "RETURN_TO_STABLE_MS=%lu RETURN_TO_STABLE_POLLS=%lu "
        "UNCLEAN_AFTER_RETURN=%lu STREAK_BREAKS=%lu\n",
        traceIndex, (unsigned)(i + 1), (unsigned long)e.lastPresentMs,
        (unsigned long)e.firstAbsentMs, (unsigned long)e.firstReturnMs,
        (unsigned long)e.absentPolls, (unsigned long)e.durationMs,
        (unsigned long)e.gapMs, e.ended ? 1 : 0, e.stableReached ? 1 : 0,
        (unsigned long)e.returnToStableMs,
        (unsigned long)e.returnToStablePolls,
        (unsigned long)e.uncleanAfterReturn, (unsigned long)e.streakBreaks);
  }
}

static void runTrace(const char *token) {
  if (sessionState != SESSION_PRIME_DONE && sessionState != SESSION_TRACE_DONE) {
    Serial.println("TRACE_ABORT: PRIME_NOT_COMPLETE");
    Serial.println("TRACE_RESULT FAIL");
    return;
  }
  if (strncmp(token, sessionToken, sizeof(sessionToken)) != 0) {
    Serial.println("TRACE_ABORT: TOKEN_MISMATCH");
    Serial.println("TRACE_RESULT FAIL");
    return;
  }
  if (sessionTraceCount >= TRACE_MAX_TRACES) {
    Serial.printf("TRACE_ABORT: TRACE_LIMIT_REACHED=%u\n", TRACE_MAX_TRACES);
    Serial.println("TRACE_RESULT FAIL");
    return;
  }

  const uint8_t id = sessionId;
  const uint8_t traceIndex = sessionTraceCount + 1;

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" CHARACTERIZATION B — POWER CYCLE %u\n", traceIndex);
  Serial.println("====================================");
  Serial.println("TRACE_MODE=READ_ONLY");
  Serial.printf("TRACE_CONFIG TRACE=%u POLL_PERIOD_MS=%lu IO_TIMEOUT_MS=%lu "
                "ARM_WINDOW_MS=%lu OBSERVE_MIN_MS=%lu OBSERVE_MAX_MS=%lu "
                "STABLE_TARGET=%lu\n",
                traceIndex, (unsigned long)TRACE_POLL_PERIOD_MS,
                (unsigned long)TRACE_IO_TIMEOUT_MS,
                (unsigned long)TRACE_ARM_WINDOW_MS,
                (unsigned long)TRACE_OBSERVE_MIN_MS,
                (unsigned long)TRACE_OBSERVE_MAX_MS,
                (unsigned long)TRACE_STABLE_TARGET);

  // Precondition, read-only: the target is present and torque is off. If this
  // fails we have not written anything and we do not start.
  if (st.Ping(id) != id || st.Error != 0) {
    Serial.println("TRACE_ABORT: PRECONDITION_TARGET_ABSENT");
    Serial.println("TRACE_RESULT FAIL");
    return;
  }
  const int torqueNow = st.readByte(id, REG_TORQUE_ENABLE);
  if (torqueNow != 0 || st.Error != 0) {
    enterFault(id, "TRACE_PRECONDITION_TORQUE_NOT_OFF");
    Serial.println("TRACE_RESULT FAIL");
    return;
  }
  Serial.printf("TRACE_PRECONDITION TRACE=%u PING=OK TORQUE_ENABLE=%d\n",
                traceIndex, torqueNow);

  Serial.println();
  Serial.println("************************************");
  Serial.println("*  POWER CYCLE NEW01 SERVO RAIL NOW *");
  Serial.println("*  SERVO RAIL OFF, THEN ON          *");
  Serial.println("*  LEAVE ESP32 USB POWER CONNECTED  *");
  Serial.println("************************************");
  Serial.println();

  AbsenceEpisode eps[TRACE_MAX_EPISODES];
  uint8_t episodeCount = 0;
  int activeEpisode = -1;      // index of the episode whose return we track
  bool inAbsence = false;

  const uint32_t armStart = millis();
  uint32_t observeStart = 0;
  bool observing = false;

  uint32_t lastPresentMs = armStart;
  uint32_t lastHeartbeat = armStart;

  uint32_t polls = 0;
  uint32_t cleanPolls = 0;
  uint32_t failPolls = 0;
  uint32_t pingStreak = 0;
  uint32_t cleanStreak = 0;
  uint32_t pollsSinceReturn = 0;

  const char *termination = "ARM_TIMEOUT";
  bool episodeOverflow = false;

  while (true) {
    const uint32_t tStart = millis();
    const uint32_t usStart = micros();

    const int ping = st.Ping(id);
    const uint8_t pingStatus = st.Error;
    const bool pingOk = (ping == (int)id) && (pingStatus == 0);

    int model = -1;
    uint8_t modelStatus = 0xFF;
    if (pingOk) {
      model = st.readWord(id, REG_MODEL);
      modelStatus = st.Error;
    }
    const bool clean = pingOk && model == (int)EXPECTED_MODEL &&
                       modelStatus == 0;
    const uint32_t durUs = micros() - usStart;

    polls++;
    if (clean) {
      cleanPolls++;
    }
    if (!pingOk) {
      failPolls++;
    }

    if (!observing && !pingOk) {
      observing = true;
      observeStart = tStart;
      Serial.printf("TRACE_PHASE TRACE=%u PHASE=OBSERVE T_MS=%lu\n", traceIndex,
                    (unsigned long)tStart);
    }

    // ---- episode state machine -------------------------------------
    if (!pingOk) {
      pingStreak = 0;
      cleanStreak = 0;
      if (!inAbsence) {
        inAbsence = true;
        if (episodeCount < TRACE_MAX_EPISODES) {
          AbsenceEpisode &e = eps[episodeCount];
          e.lastPresentMs = lastPresentMs;
          e.firstAbsentMs = tStart;
          e.firstReturnMs = 0;
          e.absentPolls = 1;
          e.durationMs = 0;
          e.gapMs = 0;
          e.returnToStableMs = 0;
          e.returnToStablePolls = 0;
          e.uncleanAfterReturn = 0;
          e.streakBreaks = 0;
          e.ended = false;
          e.stableReached = false;
          activeEpisode = episodeCount;
          episodeCount++;
          Serial.printf(
              "TRACE_EVENT TRACE=%u KIND=ABSENCE_BEGIN EP=%u T_MS=%lu "
              "LAST_PRESENT_T_MS=%lu\n",
              traceIndex, (unsigned)episodeCount, (unsigned long)tStart,
              (unsigned long)lastPresentMs);
        } else {
          episodeOverflow = true;
          activeEpisode = -1;
        }
      } else if (activeEpisode >= 0) {
        eps[activeEpisode].absentPolls++;
      }
    } else {
      lastPresentMs = tStart;
      pingStreak++;

      if (inAbsence) {
        inAbsence = false;
        pollsSinceReturn = 0;
        if (activeEpisode >= 0) {
          AbsenceEpisode &e = eps[activeEpisode];
          e.firstReturnMs = tStart;
          e.durationMs = tStart - e.firstAbsentMs;
          e.gapMs = tStart - e.lastPresentMs;
          e.ended = true;
          Serial.printf(
              "TRACE_EVENT TRACE=%u KIND=ABSENCE_END EP=%u T_MS=%lu "
              "ABSENT_POLLS=%lu DURATION_MS=%lu GAP_MS=%lu\n",
              traceIndex, (unsigned)(activeEpisode + 1), (unsigned long)tStart,
              (unsigned long)e.absentPolls, (unsigned long)e.durationMs,
              (unsigned long)e.gapMs);
        }
      }

      if (activeEpisode >= 0 && eps[activeEpisode].ended) {
        pollsSinceReturn++;
      }

      if (clean) {
        cleanStreak++;
      } else {
        if (cleanStreak > 0 && activeEpisode >= 0 &&
            eps[activeEpisode].ended && !eps[activeEpisode].stableReached) {
          eps[activeEpisode].streakBreaks++;
        }
        cleanStreak = 0;
        if (activeEpisode >= 0 && eps[activeEpisode].ended &&
            !eps[activeEpisode].stableReached) {
          eps[activeEpisode].uncleanAfterReturn++;
        }
        Serial.printf(
            "TRACE_EVENT TRACE=%u KIND=RETURN_UNCLEAN T_MS=%lu MODEL=%d "
            "MSTAT=0x%02X\n",
            traceIndex, (unsigned long)tStart, model, modelStatus);
      }

      if (activeEpisode >= 0 && eps[activeEpisode].ended &&
          !eps[activeEpisode].stableReached &&
          cleanStreak >= TRACE_STABLE_TARGET) {
        AbsenceEpisode &e = eps[activeEpisode];
        e.stableReached = true;
        e.returnToStableMs = tStart - e.firstReturnMs;
        e.returnToStablePolls = pollsSinceReturn;
        Serial.printf(
            "TRACE_EVENT TRACE=%u KIND=RETURN_STABLE EP=%u T_MS=%lu "
            "SAMPLES=%lu MS_SINCE_FIRST_RETURN=%lu POLLS_SINCE_RETURN=%lu "
            "UNCLEAN_AFTER_RETURN=%lu STREAK_BREAKS=%lu\n",
            traceIndex, (unsigned)(activeEpisode + 1), (unsigned long)tStart,
            (unsigned long)cleanStreak, (unsigned long)e.returnToStableMs,
            (unsigned long)e.returnToStablePolls,
            (unsigned long)e.uncleanAfterReturn,
            (unsigned long)e.streakBreaks);
      }
    }

    // ---- per-poll record, full detail once observing ----------------
    if (observing) {
      Serial.printf(
          "TRACE TRACE=%u N=%lu T_MS=%lu DUR_US=%lu PING=%d PSTAT=0x%02X "
          "MODEL=%d MSTAT=0x%02X CLEAN=%d PSTREAK=%lu CSTREAK=%lu EP=%d\n",
          traceIndex, (unsigned long)polls, (unsigned long)tStart,
          (unsigned long)durUs, ping, pingStatus, model, modelStatus,
          clean ? 1 : 0, (unsigned long)pingStreak, (unsigned long)cleanStreak,
          activeEpisode >= 0 ? (activeEpisode + 1) : 0);
    } else if ((uint32_t)(tStart - lastHeartbeat) >= TRACE_HEARTBEAT_MS) {
      lastHeartbeat = tStart;
      Serial.printf(
          "TRACE_ARM_HEARTBEAT TRACE=%u T_MS=%lu ELAPSED_MS=%lu POLLS=%lu "
          "CLEAN=%lu FAIL=%lu\n",
          traceIndex, (unsigned long)tStart,
          (unsigned long)(tStart - armStart), (unsigned long)polls,
          (unsigned long)cleanPolls, (unsigned long)failPolls);
    }

    // ---- termination ------------------------------------------------
    if (observing) {
      const uint32_t observed = tStart - observeStart;
      if (observed >= TRACE_OBSERVE_MAX_MS) {
        termination = "OBSERVE_WINDOW_MAX";
        break;
      }
      if (observed >= TRACE_OBSERVE_MIN_MS && episodeCount > 0 && !inAbsence &&
          cleanStreak >= TRACE_STABLE_TARGET) {
        termination = "STABLE_AFTER_ABSENCE";
        break;
      }
    } else if ((uint32_t)(tStart - armStart) >= TRACE_ARM_WINDOW_MS) {
      termination = "ARM_TIMEOUT";
      break;
    }

    delay(TRACE_POLL_PERIOD_MS);
  }

  // ---- report ------------------------------------------------------
  uint32_t longestAbsence = 0;
  uint32_t shortestAbsence = 0xFFFFFFFFUL;
  uint8_t endedEpisodes = 0;
  uint8_t stableEpisodes = 0;
  for (uint8_t i = 0; i < episodeCount; ++i) {
    if (!eps[i].ended) {
      continue;
    }
    endedEpisodes++;
    if (eps[i].durationMs > longestAbsence) longestAbsence = eps[i].durationMs;
    if (eps[i].durationMs < shortestAbsence) shortestAbsence = eps[i].durationMs;
    if (eps[i].stableReached) stableEpisodes++;
  }
  if (endedEpisodes == 0) {
    shortestAbsence = 0;
  }

  emitEpisodes(eps, episodeCount, traceIndex);

  Serial.printf(
      "TRACE_SUMMARY TRACE=%u TERMINATION=%s POLLS=%lu CLEAN=%lu FAIL=%lu "
      "EPISODES=%u ENDED=%u STABLE=%u OVERFLOW=%d LONGEST_ABSENCE_MS=%lu "
      "SHORTEST_ABSENCE_MS=%lu ARM_ELAPSED_MS=%lu OBSERVE_ELAPSED_MS=%lu\n",
      traceIndex, termination, (unsigned long)polls, (unsigned long)cleanPolls,
      (unsigned long)failPolls, (unsigned)episodeCount,
      (unsigned)endedEpisodes, (unsigned)stableEpisodes,
      episodeOverflow ? 1 : 0, (unsigned long)longestAbsence,
      (unsigned long)shortestAbsence,
      (unsigned long)(observing ? observeStart - armStart : millis() - armStart),
      (unsigned long)(observing ? millis() - observeStart : 0));

  Serial.printf("TRACE_RESOLUTION TRACE=%u PRESENT_MS=%lu ABSENT_MS=%lu "
                "BASIS=SCSERIAL_IOTIMEOUT\n",
                traceIndex, (unsigned long)TRACE_POLL_PERIOD_MS,
                (unsigned long)TRACE_IO_TIMEOUT_MS);
  Serial.println("COLD_ABSENCE_DEBOUNCE_MS_SELECTION=DEFERRED_TO_REVIEW");
  Serial.println("COLD_RETURN_STABLE_SAMPLES_SELECTION=DEFERRED_TO_REVIEW");

  const bool usable = endedEpisodes > 0 && stableEpisodes > 0;

  // An unusable trace (the operator never cycled the rail, or the return never
  // stabilised) does NOT consume a trace slot and does NOT emit a cold
  // snapshot: the operator can simply re-issue @TRACE.
  if (!usable) {
    Serial.println("EEPROM_WRITES=NONE");
    Serial.printf("TRACE_RESULT INDEX=%u RESULT=INCOMPLETE REASON=%s\n",
                  traceIndex, termination);
    Serial.println("TRACE_NOT_CONSUMED");
    Serial.println("WAIT_TRACE");
    return;
  }

  sessionTraceCount = traceIndex;
  sessionState = SESSION_TRACE_DONE;

  uint8_t raw[SNAPSHOT_LEN];
  const char *slot = (traceIndex == 1) ? "COLD1" : "COLD2";
  if (!captureAndEmit(slot, id, raw)) {
    Serial.printf("TRACE_SNAPSHOT_FAILED TRACE=%u SLOT=%s\n", traceIndex, slot);
    Serial.printf("TRACE_RESULT INDEX=%u RESULT=INCOMPLETE REASON=%s\n",
                  traceIndex, "POST_TRACE_SNAPSHOT_FAILED");
    return;
  }

  Serial.println("EEPROM_WRITES=NONE");
  Serial.printf("TRACE_RESULT INDEX=%u RESULT=PASS REASON=%s\n", traceIndex,
                termination);
  if (sessionTraceCount < TRACE_MAX_TRACES) {
    Serial.println("WAIT_TRACE");
  } else {
    Serial.println("TRACE_LIMIT_REACHED");
  }
}

// --------------------------------------------------------------------------
// Command surface
// --------------------------------------------------------------------------

static void printHelp() {
  Serial.println();
  Serial.println("COMMAND_SURFACE_BEGIN");
  Serial.println("  @ARM");
  Serial.println("  @PRIME <SESSION_TOKEN>");
  Serial.println("  @TRACE <SESSION_TOKEN>");
  Serial.println("  @HELP");
  Serial.println("COMMAND_SURFACE_END");
  Serial.println("GENERIC_WRITE           : NOT IMPLEMENTED");
  Serial.println("ARBITRARY_ADDRESS_VALUE : NOT IMPLEMENTED");
  Serial.println("ARBITRARY_GOAL_POSITION : NOT IMPLEMENTED");
  Serial.println("ARBITRARY_ID            : NOT IMPLEMENTED");
  Serial.println("EEPROM_WRITE            : NOT IMPLEMENTED");
  Serial.println("EEPROM_UNLOCK           : NOT IMPLEMENTED");
  Serial.println("ID_WRITE                : NOT IMPLEMENTED");
  Serial.println("POSITION_OFFSET_WRITE   : NOT IMPLEMENTED");
  Serial.println("MID_POSITION_CALIBRATION: NOT IMPLEMENTED");
  Serial.println("FACTORY_RESET           : NOT IMPLEMENTED");
  Serial.println("BROADCAST_WRITE         : NOT IMPLEMENTED");
  Serial.println("TORQUE_ON               : NOT IMPLEMENTED");
  Serial.println("CENTERING_MOVE          : NOT IMPLEMENTED");

  Serial.printf("WRITE_ALLOWLIST=0x%02X:1:0,0x%02X:2:%u\n", REG_TORQUE_ENABLE,
                REG_TORQUE_LIMIT, CHAR_TORQUE_LIMIT);
  Serial.printf("MOTION_ALLOWLIST=WRITEPOSEX:CURRENT_POSITION:%u:%u\n",
                CHAR_SPEED, CHAR_ACC);
  Serial.println();
}

static bool parseBare(const String &cmd, const char *name) {
  return cmd == name;
}

static bool parseTokenArg(const String &cmd, const char *prefix, char *out,
                          size_t outLen) {
  const size_t prefixLen = strlen(prefix);
  if (!cmd.startsWith(prefix)) {
    return false;
  }
  String arg = cmd.substring(prefixLen);
  if (arg.length() != 8) {
    return false;
  }
  for (size_t i = 0; i < 8; ++i) {
    const char c = arg[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) {
      return false;
    }
  }
  strncpy(out, arg.c_str(), outLen - 1);
  out[outLen - 1] = '\0';
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(1500);

  ServoUART.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &ServoUART;

  clearSession();

  Serial.println();
  Serial.println("====================================");
  Serial.println(" MATDOG ST3215 CHARACTERIZE V1");
  Serial.println("====================================");
  Serial.printf("FIRMWARE         : %s %s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
  Serial.printf("Servo UART       : %lu baud\n", SERVO_BAUD);
  Serial.printf("TX               : GPIO%d\n", SERVO_TX_PIN);
  Serial.printf("RX               : GPIO%d\n", SERVO_RX_PIN);
  Serial.println();
  Serial.println("Startup motion   : IMPOSSIBLE BY DESIGN");
  Serial.println("@ARM writes      : NONE");
  Serial.println("This is NOT a provisioner.");
  reportScope();
  Serial.println("CHARACTERIZER_READY");

  printHelp();
}

void loop() {
  if (!Serial.available()) {
    delay(1);
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  if (parseBare(cmd, "@HELP")) {
    printHelp();
    return;
  }

  if (parseBare(cmd, "@ARM")) {
    runArm();
    return;
  }

  char token[9];
  if (parseTokenArg(cmd, "@PRIME ", token, sizeof(token))) {
    runPrime(token);
    return;
  }
  if (parseTokenArg(cmd, "@TRACE ", token, sizeof(token))) {
    runTrace(token);
    return;
  }

  Serial.println("ERROR: UNKNOWN_COMMAND");
  Serial.println("EEPROM_WRITES=NONE");
  Serial.println("MOTION=NONE");
}
