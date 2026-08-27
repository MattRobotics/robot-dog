/*
 * MATDOG ST-3215-C018 PROVISIONER V1 — ESP32-S3 firmware
 *
 * Profile : MATDOG_C018_V1
 * Center  : physical RAW 2048
 * Host    : matdog_st3215_provisioner_v1.py
 * Design  : MATDOG_ST3215_PROVISIONER_V1_DESIGN.md
 *
 * COMMAND SURFACE — exactly three commands, nothing else parses:
 *     @BEGIN <PHYSICAL_LABEL>
 *     @EXECUTE <SESSION_TOKEN>
 *     @HELP
 *
 * There is no generic EEPROM write command, no arbitrary ID command, no
 * arbitrary GoalPosition command, no CalibrationOfs, no host-exposed Torque ON,
 * no factory reset and no broadcast write. Every servo write passes through
 * provWriteByte / provWriteWord / provWritePosEx, which enforce an
 * (address, value, stage) allowlist before a byte reaches the bus.
 *
 * @BEGIN performs ZERO servo writes: the host must make the BEFORE evidence
 * durable on disk before @EXECUTE is even possible.
 *
 * HARDWARE FREEZE IS BLOCKED — see PROVISIONER_HARDWARE_FREEZE_BLOCKED and
 * design section 11.2. @EXECUTE aborts before the first write while any
 * UNRESOLVED constant remains.
 */

#include <Arduino.h>
#include <SCServo.h>

// --------------------------------------------------------------------------
// Transport — identical to the frozen survey firmware
// --------------------------------------------------------------------------

static constexpr int SERVO_TX_PIN = 17;
static constexpr int SERVO_RX_PIN = 18;
static constexpr uint32_t SERVO_BAUD = 1000000;

HardwareSerial ServoUART(1);
SMS_STS st;

static const char *FIRMWARE_NAME = "matdog_st3215_provisioner_v1";
static const char *FIRMWARE_VERSION = "1.0.0";
static const char *PROFILE_ID = "MATDOG_C018_V1";

// --------------------------------------------------------------------------
// HARDWARE FREEZE BLOCK
//
// Set to false ONLY after every UNRESOLVED constant below has been measured on
// the pilot and this file has been updated with the measured values.
// --------------------------------------------------------------------------

static constexpr bool PROVISIONER_HARDWARE_FREEZE_BLOCKED = false;

// --------------------------------------------------------------------------
// State map — 71 bytes, 0x00 .. 0x46 inclusive
// --------------------------------------------------------------------------

static constexpr uint8_t SNAPSHOT_START = 0x00;
static constexpr uint8_t SNAPSHOT_LEN = 71;

static constexpr uint8_t REG_MODEL = 0x03;          // 2 bytes, NOT 0x00
static constexpr uint8_t REG_ID = 0x05;
static constexpr uint8_t REG_BAUD = 0x06;
static constexpr uint8_t REG_RETURN_DELAY = 0x07;   // preserve
static constexpr uint8_t REG_RESPONSE_STATUS = 0x08;
static constexpr uint8_t REG_UNLOAD_CONDITION = 0x12; // preserve
static constexpr uint8_t REG_LED_ALARM = 0x13;      // preserve
static constexpr uint8_t REG_ANGULAR_RESOLUTION = 0x1E; // preserve
static constexpr uint8_t REG_POSITION_OFFSET = 0x1F; // 2 bytes
static constexpr uint8_t REG_TORQUE_ENABLE = 0x28;
static constexpr uint8_t REG_ACC = 0x29;            // WritePosEx block base
static constexpr uint8_t REG_TORQUE_LIMIT = 0x30;   // 2 bytes
static constexpr uint8_t REG_LOCK = 0x37;
static constexpr uint8_t REG_PRESENT_POSITION = 0x38; // 2 bytes
static constexpr uint8_t REG_PRESENT_VOLTAGE = 0x3E;
static constexpr uint8_t REG_PRESENT_TEMPERATURE = 0x3F;
static constexpr uint8_t REG_STATUS = 0x40;

static constexpr uint16_t EXPECTED_MODEL = 777;
static constexpr uint8_t EXPECTED_RESPONSE_STATUS = 1;
static constexpr uint8_t EXPECTED_BAUD = 0;

static constexpr int ENCODER_COUNTS = 4096;
static constexpr int RAW_CENTER = 2048;
static constexpr int POSITION_OFFSET_MIN = -2048;
static constexpr int POSITION_OFFSET_MAX = 2047;

// Acceptance after settling — canonical profile doc section 5.
// NEVER widened. A correction exists to reach this band, never to relax it.
static constexpr int CENTER_ACCEPT_TICKS = 1;

// --------------------------------------------------------------------------
// STICTION RE-APPROACH — measured on NEW01, pilot session
// sessions/NEW01__20260827_110858Z (V2 firmware d1de988d..., 2026-08-27).
//
//   commanded step +3 from rest -> ZERO motion
//       MOTION_SUMMARY START=1960 TARGET=1963 FINAL=1960
//       POSITION_RANGE=1960..1960 PEAK_SPEED=0 PEAK_LOAD=40 FIRST_MOTION_US=0
//   commanded step +6 from rest -> moved +5, landed 1 short of the command
//       MOTION_SUMMARY START=1960 TARGET=1966 FINAL=1965
//       POSITION_RANGE=1960..1965 PEAK_SPEED=100 PEAK_LOAD=68
//       FIRST_MOTION_US=64672
//
// The servo does not fail to *aim*; it fails to *start*. Below roughly 5 ticks
// of position error the closed loop asks for less effort (load 40/1000) than
// the joint needs to break static friction, so the move never begins. Once it
// does begin it lands within 1 tick of whatever it was told.
//
// V2 tried to compensate by aiming past the goal (correctionGoal = goal -
// error). That is wrong for this mechanism: the servo then starts, tracks
// correctly, and stops at the deliberately-wrong target — which is exactly
// what happened (commanded 1966, landed 1965, physical 2050, error +2, FAIL).
//
// V3 keeps the goal honest and fixes the *step*: retreat to a staging point far
// enough below the goal to guarantee motion, then re-approach the true goal
// from there. Both segments are large, so both actually move, and the final
// segment lands on the real target.
//
// CENTER_STAGING_TICKS is twice the smallest step observed to move (6). It is a
// STAGING DISTANCE, not an acceptance threshold: if it were ever too small the
// approach simply would not happen and the stage fails closed at +/-1. It can
// never widen acceptance and can never turn a miss into a PASS.
static constexpr int CENTER_STAGING_TICKS = 12;

// Explicit bound. No unbounded retry: at most this many staging re-approaches,
// after which the stage fails closed. Attempt 1 measures the terminal deadband,
// attempt 2 compensates it, attempt 3 is spare.
static constexpr int CENTER_MAX_CORRECTIONS = 3;

// Canonical centering parameters — canonical profile doc section 6.
static constexpr uint16_t CENTER_TORQUE_LIMIT = 300;
static constexpr uint16_t CENTER_SPEED = 365;
static constexpr uint8_t CENTER_ACC = 50;

// --------------------------------------------------------------------------
// Physical allocation — the firmware owns this mapping
// --------------------------------------------------------------------------

struct PhysicalUnit {
  const char *label;
  uint8_t targetId;
  const char *joint;
  uint8_t coldCycles;   // pilot policy: NEW01 requires two
};

static const PhysicalUnit UNITS[] = {
  {"M22",   13, "LF_HIP",        1},
  {"ELR01", 12, "LF_UPPER",      1},
  {"M33",   11, "LF_LOWER",      1},

  {"NEW01", 23, "RF_HIP",        2},   // pilot
  {"ELR03", 22, "RF_UPPER",      1},
  {"NEW03", 21, "RF_LOWER",      1},

  {"NEW06", 33, "RH_HIP",        1},
  {"ELR02", 32, "RH_UPPER",      1},
  {"NEW05", 31, "RH_LOWER",      1},

  {"M43",   43, "LH_HIP",        1},
  {"M42",   42, "LH_UPPER",      1},
  {"M41",   41, "LH_LOWER",      1},

  {"M31",   51, "NECK_ROTATION", 1},
  {"M11",   52, "NECK_PITCH",    1},
  {"NEW04", 53, "HEAD_ROTATION", 1},
  {"NEW02", 54, "HEAD_PITCH",    1},
  {"ELR04", 55, "JAW",           1},
};

static constexpr size_t UNIT_COUNT = sizeof(UNITS) / sizeof(UNITS[0]);

// --------------------------------------------------------------------------
// Canonical persistent profile — the complete write allowlist for stage 2
//
// PositionOffset (0x1F) and ID (0x05) are deliberately NOT here: they have
// their own dedicated late stages. Baud (0x06) is verification-only and is
// never written. Everything absent from this table is preserved.
// --------------------------------------------------------------------------

struct ProfileField {
  uint8_t addr;
  uint8_t width;
  uint16_t value;
  const char *name;
};

static const ProfileField PROFILE[] = {
  {0x09, 2, 0,    "MinAngle"},
  {0x0B, 2, 4095, "MaxAngle"},
  {0x0D, 1, 70,   "MaxTemperature"},
  {0x0E, 1, 140,  "MaxVoltage"},
  {0x0F, 1, 40,   "MinVoltage"},
  {0x10, 2, 1000, "MaxTorque"},
  {0x15, 1, 32,   "P"},
  {0x16, 1, 32,   "D"},
  {0x17, 1, 0,    "I"},
  {0x18, 2, 16,   "MinStartupForce"},
  {0x1A, 1, 1,    "CWDead"},
  {0x1B, 1, 1,    "CCWDead"},
  {0x1C, 2, 310,  "ProtectionCurrent"},
  {0x21, 1, 0,    "Mode"},
  {0x22, 1, 20,   "ProtectionTorque"},
  {0x23, 1, 200,  "ProtectionTime"},
  {0x24, 1, 80,   "OverloadTorque"},
  {0x25, 1, 10,   "SpeedClosedLoopP"},
  {0x26, 1, 200,  "OverCurrentProtectionTime"},
  {0x27, 1, 200,  "VelocityClosedLoopI"},
};

static constexpr size_t PROFILE_COUNT = sizeof(PROFILE) / sizeof(PROFILE[0]);

// Read-only audit list, printed by @HELP. Never written.
static const uint8_t PRESERVE_ONLY[] = {
  REG_BAUD, REG_RETURN_DELAY, REG_RESPONSE_STATUS,
  REG_UNLOAD_CONDITION, REG_LED_ALARM, REG_ANGULAR_RESOLUTION,
};

static constexpr size_t PRESERVE_COUNT =
    sizeof(PRESERVE_ONLY) / sizeof(PRESERVE_ONLY[0]);

// --------------------------------------------------------------------------
// Motion monitoring constants — REUSED from frozen QC V6.1
// (matdog_servo_commissioning.ino, validated across the 26-run campaign).
// --------------------------------------------------------------------------

static constexpr uint32_t MON_PERIOD_US = 2000;            // QC_FAST_PERIOD_US
static constexpr int MON_STATIONARY_SPEED = 10;
static constexpr uint32_t MON_STABLE_SAMPLES = 40;         // 80 ms
static constexpr int MON_REACHED_RESIDUAL = 3;

// --------------------------------------------------------------------------
// TERMINAL DEADBAND COMPENSATION — measured on NEW01, pilot session
// sessions/NEW01__20260827_115248Z (V3 firmware c67f1801..., 2026-08-27).
//
// Once the staging retreat guarantees the move actually starts, every single
// move still stops a fixed distance short of the commanded target, in the
// direction of travel:
//
//     start 1966 -> cmd 1963 -> rest 1965   down, 2 short
//     start 1965 -> cmd 1951 -> rest 1953   down, 2 short
//     start 1953 -> cmd 1963 -> rest 1961   up,   2 short
//     start 1961 -> cmd 1951 -> rest 1953   down, 2 short
//     start 1953 -> cmd 1963 -> rest 1961   up,   2 short
//
// 5/5, both directions, exactly 2 ticks. This is a terminal position deadband,
// not stiction: FIRST_MOTION_US was ~58 ms and PEAK_SPEED 200 on every one of
// them. The servo aims where it is told and then stops early.
//
// The compensation is therefore NOT a hard-coded 2. The bias is measured
// in-session from the previous standardised re-approach and fed back:
//
//     bias -= error_of_previous_reapproach
//
// so a unit with a different deadband converges on its own value, and a unit
// whose deadband exceeds the already-validated correction window fails closed
// instead of being forced. Attempt 1 always runs unbiased, because only a
// re-approach measures the deadband for the standard geometry — the initial
// centering move may arrive from any direction and distance.
static constexpr int CENTER_MAX_BIAS_TICKS = MON_REACHED_RESIDUAL;
static constexpr int MON_NO_PROGRESS_RESIDUAL = 32;
static constexpr uint32_t MON_NO_PROGRESS_US = 1500000UL;
static constexpr int MON_DIRECTION_TOL = 16;
static constexpr int MON_OVERCURRENT_RAW = 308;            // ~2.0 A
static constexpr uint32_t MON_OVERCURRENT_SAMPLES = 50;    // 100 ms
static constexpr int MON_STALL_CURRENT_RAW = 416;          // ~2.7 A
static constexpr uint32_t MON_STALL_CURRENT_SAMPLES = 10;  // 20 ms
static constexpr int MON_OVERLOAD_LOAD_RAW = 800;
static constexpr int MON_RATED_CURRENT_RAW = 139;
static constexpr int MON_STALL_LOW_SPEED_RAW = 50;
static constexpr uint32_t MON_STALL_NO_PROGRESS_US = 250000UL;
static constexpr uint32_t MON_STALL_SIGNATURE_SAMPLES = 50;
static constexpr int MON_THERMAL_LIMIT_C = 70;
static constexpr int MON_THERMAL_CONFIRMATIONS = 3;
static constexpr uint32_t MON_THERMAL_CONFIRM_DELAY_MS = 5;
static constexpr int MON_VOLTAGE_MIN = 40;
static constexpr int MON_VOLTAGE_MAX = 140;
static constexpr uint32_t MON_VOLTAGE_SAMPLES = 50;        // 100 ms
static constexpr uint32_t MON_STATUS_SAMPLES = 25;         // 50 ms
static constexpr uint32_t MON_TELEMETRY_LOSS_SAMPLES = 25; // 50 ms
static constexpr uint32_t WRITE_SETTLE_MS = 20;

// --------------------------------------------------------------------------
// SELECTED CHARACTERIZATION CONSTANTS — frozen from NEW01 evidence.
// PRIME: 5/5 runs at speed 365 acc 50, delta and peak absolute delta = 0.
// Selected PRIME_MAX_DELTA_TICKS = 1 tick.
// COLD: true OFF->ON absences measured at 21720 ms and 5280 ms.
static constexpr int PRIME_MAX_DELTA_TICKS = 1;            // SELECTED FROM NEW01 CHARACTERIZATION

// Cold thresholds selected from the same two NEW01 power cycles:
// debounce = 1500 ms; return stability = 10 samples.
static constexpr uint32_t COLD_ABSENCE_DEBOUNCE_MS = 1500; // SELECTED FROM NEW01 CHARACTERIZATION
static constexpr uint32_t COLD_RETURN_STABLE_SAMPLES = 10; // SELECTED FROM NEW01 CHARACTERIZATION

// Operator patience bound, NOT a safety constant: torque is off, EEPROM is
// locked, and the cold phase is strictly read-only.
static constexpr uint32_t COLD_OPERATOR_WINDOW_MS = 180000;
static constexpr uint32_t COLD_POLL_INTERVAL_MS = 50;

// --------------------------------------------------------------------------
// Session state
// --------------------------------------------------------------------------

enum SessionState : uint8_t {
  SESSION_IDLE = 0,
  SESSION_WAIT_EXECUTE,
  SESSION_RUNNING,
  SESSION_DONE,
  SESSION_FAULT,
};

// Stage gate for the write allowlist. A write is refused unless the current
// stage is the one that owns that register.
enum Stage : uint8_t {
  STAGE_NONE = 0,
  STAGE_TORQUE_SAFETY,   // TorqueEnable = 0 only
  STAGE_PROFILE,         // profile table + lock
  STAGE_PRIME,           // TorqueLimit, WritePosEx at current position
  STAGE_CENTER,          // TorqueEnable = 1, WritePosEx at center goal
  STAGE_OFFSET_ZERO,     // PositionOffset = 0 + lock
  STAGE_ID_RECODE,       // ID = bound target + lock
  STAGE_RECOVERY,        // best-effort lock + torque off on a failure path
};

static SessionState sessionState = SESSION_IDLE;
static Stage stage = STAGE_NONE;

static char sessionLabel[16] = {0};
static uint8_t sessionSourceId = 0;
static uint8_t sessionTargetId = 0;
static uint8_t sessionColdCycles = 1;
static int32_t sessionOldOffset = 0;
static char sessionToken[9] = {0};
static uint32_t sessionBeforeDigest = 0;
static uint8_t sessionBefore[SNAPSHOT_LEN];

static bool eepromUnlocked = false;
static bool torqueTouched = false;

// --------------------------------------------------------------------------
// Result types
//
// Declared before any function because the .ino preprocessor injects
// function prototypes above the body of this file.
// --------------------------------------------------------------------------

struct WriteOutcome {
  bool allowed;
  int ack;
  uint8_t writeStatus;
  int readback;
  uint8_t readStatus;
  bool ok;
};

struct ScanResult {
  int found;
  int id;
  int model;
  uint8_t pingStatus;
  uint8_t modelStatus;
  bool targetOccupied;
};

struct MotionSummary {
  int startPos;
  int target;
  int finalPos;
  uint32_t samples;
  uint32_t elapsedUs;
  uint32_t firstMotionUs;
  int minPos, maxPos;
  int peakAbsSpeed, peakAbsLoad, peakAbsCurrent;
  int minVoltage, maxVoltage, maxTemp;
  uint8_t worstStatus;
};


// --------------------------------------------------------------------------
// Small helpers
// --------------------------------------------------------------------------

static bool validId(int id) { return id >= 0 && id <= 253; }

static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// PositionOffset ONLY. Two's complement, little endian. Never shared with the
// sign-magnitude decoders used for speed / load / current.
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

static int32_t centerGoalDisplayed(int32_t signedOffset) {
  return floorMod(RAW_CENTER - signedOffset, ENCODER_COUNTS);
}

// Boundary-safe staging point for a re-approach.
//
// Normal case: stage below the approach goal, preserving the validated V5
// final approach direction and backlash side.
//
// Low-domain exception: if staging below would leave unsigned 0..4095, stage
// above the approach goal and re-approach downward. This avoids any 4095<->0
// wrap and therefore cannot turn a small boundary correction into a near-full
// revolution.
//
// Returns -1 only if neither same-domain staging candidate is legal.
static int32_t centerStagingGoal(int32_t goal, int32_t stagingTicks) {
  const int32_t below = goal - stagingTicks;
  if (below >= 0 && below <= 4095) {
    return below;
  }

  const int32_t above = goal + stagingTicks;
  if (above >= 0 && above <= 4095) {
    return above;
  }

  return -1;
}

// Smallest signed distance on the 4096 circle, used for tolerance checks.
static int32_t circularDelta(int32_t a, int32_t b) {
  return floorMod(a - b + RAW_CENTER, ENCODER_COUNTS) - RAW_CENTER;
}

static uint32_t fnv1a32(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

static const PhysicalUnit *findUnit(const char *label) {
  for (size_t i = 0; i < UNIT_COUNT; ++i) {
    if (strcmp(UNITS[i].label, label) == 0) {
      return &UNITS[i];
    }
  }
  return nullptr;
}

static uint16_t profileFieldFromRaw(const uint8_t *raw, const ProfileField &f) {
  return f.width == 2 ? u16le(raw + f.addr) : (uint16_t)raw[f.addr];
}

// Reused verbatim from QC V6.1 qcComputeMoveTimeoutMs(). Reproduces the
// logged window_ms values of the 26-run campaign exactly.
static uint32_t computeMoveTimeoutMs(int startPos, int target,
                                     uint16_t speedCmd) {
  const uint32_t distance = (uint32_t)abs(target - startPos);

  uint32_t expectedFloorTps;
  if (speedCmd == 0) {
    expectedFloorTps = 450;
  } else if (speedCmd <= 2) {
    expectedFloorTps = 1;
  } else {
    expectedFloorTps = speedCmd / 2;
    if (expectedFloorTps < 40) {
      expectedFloorTps = 40;
    }
  }

  uint32_t travelMs =
      (distance * 1000UL + expectedFloorTps - 1) / expectedFloorTps;

  uint32_t marginMs = (speedCmd <= 2) ? 2500UL : 2200UL;
  if (speedCmd == 0) {
    marginMs = 2500UL;
  }

  uint32_t result = travelMs + marginMs;
  if (result < 2000UL) {
    result = 2000UL;
  }
  if (result > 30000UL) {
    result = 30000UL;
  }
  return result;
}

// --------------------------------------------------------------------------
// WRITE CHOKE POINT
//
// Every servo write in this firmware goes through one of the three functions
// below. There is no other call site of writeByte / writeWord / EnableTorque /
// WritePosEx / unLockEprom / LockEprom outside this section.
//
// CalibrationOfs() is never called. Note it is implemented as
// writeByte(ID, 0x28, 128), so value 128 at 0x28 is explicitly refused here.
// --------------------------------------------------------------------------

static bool writeAllowed(uint8_t addr, uint8_t width, uint16_t value) {
  switch (addr) {
    case REG_TORQUE_ENABLE:
      // 0 always; 1 only inside the authorized centering transaction.
      // 128 (CalibrationOfs) is refused unconditionally.
      if (width != 1) return false;
      if (value == 0) {
        return stage == STAGE_TORQUE_SAFETY || stage == STAGE_PRIME ||
               stage == STAGE_CENTER || stage == STAGE_RECOVERY;
      }
      if (value == 1) {
        return stage == STAGE_CENTER;
      }
      return false;

    case REG_LOCK:
      if (width != 1) return false;
      if (value != 0 && value != 1) return false;
      if (value == 0) {
        return stage == STAGE_PROFILE || stage == STAGE_OFFSET_ZERO ||
               stage == STAGE_ID_RECODE;
      }
      return stage == STAGE_PROFILE || stage == STAGE_OFFSET_ZERO ||
             stage == STAGE_ID_RECODE || stage == STAGE_RECOVERY;

    case REG_TORQUE_LIMIT:
      return width == 2 && value == CENTER_TORQUE_LIMIT &&
             stage == STAGE_PRIME;

    case REG_POSITION_OFFSET:
      return width == 2 && value == 0 && stage == STAGE_OFFSET_ZERO;

    case REG_ID:
      return width == 1 && stage == STAGE_ID_RECODE &&
             value == sessionTargetId && validId((int)value);

    default:
      break;
  }

  if (stage != STAGE_PROFILE) {
    return false;
  }
  for (size_t i = 0; i < PROFILE_COUNT; ++i) {
    if (PROFILE[i].addr == addr) {
      return PROFILE[i].width == width && PROFILE[i].value == value;
    }
  }
  return false;
}


static WriteOutcome provWrite(uint8_t id, uint8_t addr, uint8_t width,
                              uint16_t value, const char *name) {
  WriteOutcome out = {false, 0, 0xFF, -1, 0xFF, false};

  if (!writeAllowed(addr, width, value)) {
    Serial.printf("WRITE_REFUSED ADDR=0x%02X WIDTH=%u VALUE=%u STAGE=%u\n",
                  addr, width, value, (unsigned)stage);
    return out;
  }
  out.allowed = true;

  out.ack = (width == 2) ? st.writeWord(id, addr, value)
                         : st.writeByte(id, addr, (uint8_t)value);
  out.writeStatus = st.Error;

  delay(WRITE_SETTLE_MS);

  out.readback = (width == 2) ? st.readWord(id, addr) : st.readByte(id, addr);
  out.readStatus = st.Error;

  // ACK semantics: 1 = success, 0 = failure. "< 0" is never used.
  out.ok = (out.ack == 1) && (out.writeStatus == 0) &&
           (out.readback == (int)value) && (out.readStatus == 0);

  Serial.printf(
      "WRITE ADDR=0x%02X NAME=%s WIDTH=%u EXPECT=%u ACK=%d STATUS=0x%02X "
      "READBACK=%d READ_STATUS=0x%02X RESULT=%s\n",
      addr, name, width, value, out.ack, out.writeStatus, out.readback,
      out.readStatus, out.ok ? "OK" : "FAIL");

  return out;
}

static bool provWriteByte(uint8_t id, uint8_t addr, uint8_t value,
                          const char *name) {
  return provWrite(id, addr, 1, value, name).ok;
}

static bool provWriteWord(uint8_t id, uint8_t addr, uint16_t value,
                          const char *name) {
  return provWrite(id, addr, 2, value, name).ok;
}

// The ONLY motion primitive. Position domain is validated before the call so
// WritePosEx's negative sign-magnitude branch is unreachable.
static bool provWritePosEx(uint8_t id, int32_t position, const char *why) {
  if (stage != STAGE_PRIME && stage != STAGE_CENTER) {
    Serial.printf("MOTION_REFUSED STAGE=%u\n", (unsigned)stage);
    return false;
  }
  if (position < 0 || position > 4095) {
    Serial.printf("MOTION_REFUSED POSITION_OUT_OF_DOMAIN=%ld\n", (long)position);
    return false;
  }

  int ack = st.WritePosEx(id, (s16)position, CENTER_SPEED, CENTER_ACC);
  uint8_t status = st.Error;

  Serial.printf("MOTION WHY=%s GOAL=%ld SPEED=%u ACC=%u ACK=%d STATUS=0x%02X\n",
                why, (long)position, CENTER_SPEED, CENTER_ACC, ack, status);

  return ack == 1 && status == 0;
}

// ID recode is the ONE write whose ACK is advisory: the servo may already be
// answering under the new ID, so a readback at the old ID is meaningless.
// It still passes writeAllowed(), so no write escapes the allowlist. Identity
// proof (ping target / old-ID silence / model / full snapshot) is authoritative.
static bool provWriteIdAdvisory(uint8_t sourceId, uint8_t targetId, int &ack,
                                uint8_t &status) {
  ack = 0;
  status = 0xFF;

  if (!writeAllowed(REG_ID, 1, targetId)) {
    Serial.printf("WRITE_REFUSED ADDR=0x%02X WIDTH=1 VALUE=%u STAGE=%u\n",
                  REG_ID, targetId, (unsigned)stage);
    return false;
  }

  ack = st.writeByte(sourceId, REG_ID, targetId);
  status = st.Error;

  Serial.printf("ID_WRITE SOURCE=%u TARGET=%u ACK=%d STATUS=0x%02X "
                "AUTHORITY=ADVISORY\n",
                sourceId, targetId, ack, status);
  return true;
}


// --------------------------------------------------------------------------
// Verified safety primitives
// --------------------------------------------------------------------------

static bool torqueOffVerified(uint8_t id, const char *context) {
  Stage saved = stage;
  if (stage != STAGE_TORQUE_SAFETY && stage != STAGE_PRIME &&
      stage != STAGE_CENTER && stage != STAGE_RECOVERY) {
    stage = STAGE_TORQUE_SAFETY;
  }
  bool ok = provWriteByte(id, REG_TORQUE_ENABLE, 0, "TorqueEnable");
  stage = saved;

  Serial.printf("TORQUE_OFF CONTEXT=%s RESULT=%s\n", context,
                ok ? "CONFIRMED" : "FAILED");
  return ok;
}

static bool unlockVerified(uint8_t id) {
  bool ok = provWriteByte(id, REG_LOCK, 0, "Lock");
  if (ok) {
    eepromUnlocked = true;
  }
  Serial.printf("EEPROM_UNLOCK RESULT=%s\n", ok ? "OK" : "FAIL");
  return ok;
}

static bool lockVerified(uint8_t id) {
  bool ok = provWriteByte(id, REG_LOCK, 1, "Lock");
  if (ok) {
    eepromUnlocked = false;
  }
  Serial.printf("EEPROM_LOCK RESULT=%s\n", ok ? "OK" : "FAIL");
  return ok;
}

// --------------------------------------------------------------------------
// Read-only helpers
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// POSITION CONFIRMATION before a snapshot whose position decides acceptance
//
// Evidence, ELR03 pilot 2026-08-27 (session NEW01-style run, V4 firmware
// 4fccf23b...): the OFFSET_ZERO snapshot was taken immediately after the
// unlock/write/lock transaction and sampled PresentPosition = 2050, giving
// ERROR=2 and a fail-closed abort. Two independent READ-ONLY probes minutes
// later both read 2049 — the value centering had already accepted at +1 — with
// torque off the whole time and the shaft never commanded. The shaft did not
// move: the first sample after an EEPROM transaction was a transient.
//
// Every other marginal quantity in this firmware is confirmed before it is
// judged: thermalGuard takes three fresh reads, monitoredMove requires
// MON_STABLE_SAMPLES stable samples. The position that gates the +/-1 centre
// acceptance was the one exception. It no longer is.
//
// This does NOT widen CENTER_ACCEPT_TICKS and does not change what is
// accepted. It only refuses to judge +/-1 from a single transient sample.
// Torque is off and nothing is commanded here: this is strictly read-only.
static constexpr uint32_t POSITION_CONFIRM_MAX_SAMPLES = 250;  // 500 ms @ 500 Hz

static bool positionSettled(uint8_t id, const char *&reason) {
  int last = -1;
  uint32_t agree = 0;

  for (uint32_t i = 0; i < POSITION_CONFIRM_MAX_SAMPLES; ++i) {
    delayMicroseconds(MON_PERIOD_US);
    const int p = st.ReadPos(id);
    if (p < 0 || p > 4095) {
      reason = "POSITION_CONFIRM_READ_FAILED";
      return false;
    }
    if (p == last) {
      if (++agree >= MON_STABLE_SAMPLES) {
        Serial.printf("POSITION_SETTLED VALUE=%d STABLE_SAMPLES=%lu\n", p,
                      (unsigned long)agree);
        return true;
      }
    } else {
      last = p;
      agree = 1;
    }
  }

  reason = "POSITION_NEVER_SETTLED";
  return false;
}

static bool readSnapshot71(uint8_t id, uint8_t *raw) {
  int ping = st.Ping(id);
  if (ping != id || st.Error != 0) {
    Serial.printf("SNAPSHOT_ABORT PING=%d STATUS=0x%02X\n", ping, st.Error);
    return false;
  }

  int n = st.Read(id, SNAPSHOT_START, raw, SNAPSHOT_LEN);
  uint8_t status = st.Error;

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

  Serial.printf("SNAP SLOT=%s MODEL=%u ID=%u BAUD=%u RESPONSE_STATUS=%u\n",
                slot, u16le(raw + REG_MODEL), raw[REG_ID], raw[REG_BAUD],
                raw[REG_RESPONSE_STATUS]);
  Serial.printf(
      "SNAP SLOT=%s OFFSET=%ld TORQUE=%u LOCK=%u TORQUE_LIMIT=%u "
      "PRESENT=%ld PHYSICAL_RAW=%ld\n",
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

static bool profileExact(const uint8_t *raw, bool verbose) {
  bool ok = true;
  for (size_t i = 0; i < PROFILE_COUNT; ++i) {
    uint16_t actual = profileFieldFromRaw(raw, PROFILE[i]);
    if (actual != PROFILE[i].value) {
      ok = false;
      if (verbose) {
        Serial.printf("PROFILE_MISMATCH ADDR=0x%02X NAME=%s ACTUAL=%u TARGET=%u\n",
                      PROFILE[i].addr, PROFILE[i].name, actual,
                      PROFILE[i].value);
      }
    }
  }
  return ok;
}

// --------------------------------------------------------------------------
// Fault convergence — no implicit movement, ever
// --------------------------------------------------------------------------

static void enterFault(uint8_t id, const char *reason) {
  Serial.printf("FAULT REASON=%s\n", reason);

  Stage saved = stage;
  stage = STAGE_RECOVERY;

  bool torqueOk = false;
  bool lockOk = !eepromUnlocked;

  if (validId(id)) {
    torqueOk = provWriteByte(id, REG_TORQUE_ENABLE, 0, "TorqueEnable");
    if (eepromUnlocked) {
      lockOk = provWriteByte(id, REG_LOCK, 1, "Lock");
      if (lockOk) {
        eepromUnlocked = false;
      }
    }
  }

  stage = saved;

  Serial.printf("RECOVERY TORQUE_OFF=%s EEPROM_LOCK=%s\n",
                torqueOk ? "OK" : "FAILED", lockOk ? "OK" : "FAILED");

  if (!torqueOk || !lockOk) {
    Serial.println("CUT_SERVO_POWER_NOW");
  }

  sessionState = SESSION_FAULT;
  stage = STAGE_NONE;
}

// --------------------------------------------------------------------------
// SCAN — read-only
// --------------------------------------------------------------------------


static ScanResult runFullScan(uint8_t targetId) {
  ScanResult r = {0, -1, -1, 0xFF, 0xFF, false};

  Serial.println("SCAN_BEGIN RANGE=0..253");

  for (int id = 0; id <= 253; ++id) {
    int ping = st.Ping((uint8_t)id);
    if (ping < 0) {
      continue;
    }
    uint8_t pingStatus = st.Error;

    int model = st.readWord((uint8_t)id, REG_MODEL);
    uint8_t modelStatus = st.Error;

    Serial.printf(
        "FOUND ID=%d MODEL=%d PING_STATUS=0x%02X MODEL_STATUS=0x%02X\n", id,
        model, pingStatus, modelStatus);

    if (r.found == 0) {
      r.id = id;
      r.model = model;
      r.pingStatus = pingStatus;
      r.modelStatus = modelStatus;
    }
    if (id == (int)targetId) {
      r.targetOccupied = true;
    }
    r.found++;
  }

  Serial.printf("SCAN_RESULT FOUND=%d\n", r.found);
  Serial.println("SCAN_COMPLETE");
  return r;
}

// --------------------------------------------------------------------------
// PRE-PROVISION SAFETY GATE
// --------------------------------------------------------------------------

static bool gateFail(const char *name, long actual, const char *expected) {
  Serial.printf("GATE %s ACTUAL=%ld EXPECTED=%s RESULT=FAIL\n", name, actual,
                expected);
  return false;
}

static bool safetyGate(const ScanResult &scan, const uint8_t *raw,
                       uint8_t targetId) {
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

  const uint16_t model = u16le(raw + REG_MODEL);
  if (model != EXPECTED_MODEL) {
    return gateFail("MODEL_0x03", model, "777");
  }
  if (raw[REG_ID] != (uint8_t)scan.id) {
    return gateFail("ID_REGISTER", raw[REG_ID], "discovered source id");
  }
  if (raw[REG_BAUD] != EXPECTED_BAUD) {
    return gateFail("BAUD_REGISTER", raw[REG_BAUD], "0");
  }
  if (raw[REG_RESPONSE_STATUS] != EXPECTED_RESPONSE_STATUS) {
    return gateFail("RESPONSE_STATUS", raw[REG_RESPONSE_STATUS], "1");
  }

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

  if (!validId((int)targetId)) {
    return gateFail("TARGET_ID", targetId, "0..253");
  }

  Serial.println("GATE_RESULT PASS");
  return true;
}

// --------------------------------------------------------------------------
// @BEGIN — zero servo writes
// --------------------------------------------------------------------------

static void reportSelectedConstants() {
  Serial.printf(
      "SELECTED_CONSTANT NAME=PRIME_MAX_DELTA_TICKS VALUE=%d "
      "SOURCE=NEW01_CHARACTERIZATION\n",
      PRIME_MAX_DELTA_TICKS);
  Serial.printf(
      "SELECTED_CONSTANT NAME=COLD_ABSENCE_DEBOUNCE_MS VALUE=%lu "
      "SOURCE=NEW01_CHARACTERIZATION\n",
      (unsigned long)COLD_ABSENCE_DEBOUNCE_MS);
  Serial.printf(
      "SELECTED_CONSTANT NAME=COLD_RETURN_STABLE_SAMPLES VALUE=%lu "
      "SOURCE=NEW01_CHARACTERIZATION\n",
      (unsigned long)COLD_RETURN_STABLE_SAMPLES);
  Serial.printf("HARDWARE_FREEZE=%s\n",
                PROVISIONER_HARDWARE_FREEZE_BLOCKED ? "BLOCKED" : "CLEARED");
}

static void clearSession() {
  sessionState = SESSION_IDLE;
  stage = STAGE_NONE;
  sessionLabel[0] = '\0';
  sessionSourceId = 0;
  sessionTargetId = 0;
  sessionColdCycles = 1;
  sessionOldOffset = 0;
  sessionToken[0] = '\0';
  sessionBeforeDigest = 0;
  eepromUnlocked = false;
  torqueTouched = false;
}

static void runBegin(const char *label) {
  clearSession();

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" PROVISION BEGIN — %s\n", label);
  Serial.println("====================================");

  const PhysicalUnit *unit = findUnit(label);
  if (unit == nullptr) {
    Serial.printf("BEGIN_ABORT: UNKNOWN_PHYSICAL_LABEL=%s\n", label);
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("BEGIN_RESULT FAIL");
    return;
  }

  Serial.printf("BEGIN_LABEL=%s\n", unit->label);
  Serial.printf("BEGIN_TARGET_ID=%u\n", unit->targetId);
  Serial.printf("BEGIN_TARGET_JOINT=%s\n", unit->joint);
  Serial.printf("BEGIN_COLD_CYCLES_REQUIRED=%u\n", unit->coldCycles);
  Serial.printf("BEGIN_PROFILE=%s\n", PROFILE_ID);

  ScanResult scan = runFullScan(unit->targetId);

  if (scan.found != 1) {
    Serial.printf("BEGIN_ABORT: EXPECTED_ONE_RESPONDER_FOUND=%d\n", scan.found);
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("BEGIN_RESULT FAIL");
    return;
  }

  Serial.printf("BEGIN_SOURCE_ID=%d\n", scan.id);
  Serial.printf("BEGIN_TARGET_ID_OCCUPIED=%s\n",
                scan.targetOccupied ? "YES" : "NO");

  // The single responder IS the target when source == target. Occupancy only
  // matters when they differ, and with exactly one responder it cannot happen.
  if (scan.targetOccupied && scan.id != (int)unit->targetId) {
    Serial.println("BEGIN_ABORT: TARGET_ID_OCCUPIED_BY_ANOTHER_UNIT");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("BEGIN_RESULT FAIL");
    return;
  }

  uint8_t raw[SNAPSHOT_LEN];
  if (!captureAndEmit("BEFORE", (uint8_t)scan.id, raw)) {
    Serial.println("BEGIN_ABORT: BEFORE_SNAPSHOT_FAILED");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("BEGIN_RESULT FAIL");
    return;
  }

  if (!safetyGate(scan, raw, unit->targetId)) {
    Serial.println("BEGIN_ABORT: PRE_PROVISION_SAFETY_GATE_FAILED");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("BEGIN_RESULT FAIL");
    return;
  }

  // Delta preview — audit only, no writes.
  int writes = 0;
  int skips = 0;
  for (size_t i = 0; i < PROFILE_COUNT; ++i) {
    uint16_t actual = profileFieldFromRaw(raw, PROFILE[i]);
    bool needsWrite = actual != PROFILE[i].value;
    Serial.printf(
        "DELTA ADDR=0x%02X NAME=%s WIDTH=%u CURRENT=%u TARGET=%u ACTION=%s\n",
        PROFILE[i].addr, PROFILE[i].name, PROFILE[i].width, actual,
        PROFILE[i].value, needsWrite ? "WRITE" : "SKIP");
    needsWrite ? writes++ : skips++;
  }
  Serial.printf("DELTA_COUNT WRITE=%d SKIP=%d\n", writes, skips);

  const int32_t offset = i16le_twos(raw + REG_POSITION_OFFSET);
  const int32_t displayed = u16le(raw + REG_PRESENT_POSITION);

  Serial.printf("CENTER_PLAN OLD_OFFSET=%ld PRESENT=%ld PHYSICAL_RAW=%ld "
                "CENTER_GOAL_DISPLAYED=%ld TARGET_PHYSICAL_RAW=%d\n",
                (long)offset, (long)displayed,
                (long)physicalRaw(displayed, offset),
                (long)centerGoalDisplayed(offset), RAW_CENTER);

  Serial.printf("ID_PLAN SOURCE=%d TARGET=%u ACTION=%s\n", scan.id,
                unit->targetId,
                scan.id == (int)unit->targetId ? "SKIP" : "RECODE");

  // Bind the session.
  strncpy(sessionLabel, unit->label, sizeof(sessionLabel) - 1);
  sessionSourceId = (uint8_t)scan.id;
  sessionTargetId = unit->targetId;
  sessionColdCycles = unit->coldCycles;
  sessionOldOffset = offset;
  memcpy(sessionBefore, raw, SNAPSHOT_LEN);
  sessionBeforeDigest = fnv1a32(raw, SNAPSHOT_LEN);

  uint32_t entropy = esp_random();
  uint32_t bound = entropy ^ sessionBeforeDigest ^
                   ((uint32_t)sessionSourceId << 8) ^
                   ((uint32_t)sessionTargetId << 16);
  snprintf(sessionToken, sizeof(sessionToken), "%08lX", (unsigned long)bound);

  Serial.printf("BEFORE_DIGEST=%08lX\n", (unsigned long)sessionBeforeDigest);

  reportSelectedConstants();

  Serial.printf("SESSION_TOKEN=%s\n", sessionToken);
  Serial.println("EEPROM_WRITES=NONE");
  Serial.println("MOTION=NONE");
  Serial.println("BEGIN_RESULT PASS");

  sessionState = SESSION_WAIT_EXECUTE;
  Serial.println("WAIT_EXECUTE");
}

// --------------------------------------------------------------------------
// Centering — firmware-local watchdog, independent of the host
// --------------------------------------------------------------------------

static bool thermalGuard(uint8_t id, const char *&reason) {
  int initial = st.ReadTemper(id);
  if (initial < 0) {
    reason = "THERMAL_READ_FAILED";
    return false;
  }
  if (initial <= MON_THERMAL_LIMIT_C) {
    return true;
  }
  for (int i = 0; i < MON_THERMAL_CONFIRMATIONS; ++i) {
    delay(MON_THERMAL_CONFIRM_DELAY_MS);
    int confirm = st.ReadTemper(id);
    if (confirm < 0) {
      reason = "THERMAL_CONFIRM_READ_FAILED";
      return false;
    }
    if (confirm <= MON_THERMAL_LIMIT_C) {
      return true;   // transient
    }
  }
  reason = "THERMAL_CONFIRMED_OVER_LIMIT";
  return false;
}


static void printMotionSummary(const MotionSummary &m) {
  Serial.printf("MOTION_SUMMARY START=%d TARGET=%d FINAL=%d ERROR=%d\n",
                m.startPos, m.target, m.finalPos, m.finalPos - m.target);
  Serial.printf("MOTION_SUMMARY SAMPLES=%lu ELAPSED_US=%lu FIRST_MOTION_US=%lu\n",
                (unsigned long)m.samples, (unsigned long)m.elapsedUs,
                (unsigned long)m.firstMotionUs);
  Serial.printf("MOTION_SUMMARY POSITION_RANGE=%d..%d PEAK_SPEED=%d "
                "PEAK_LOAD=%d PEAK_CURRENT=%d\n",
                m.minPos, m.maxPos, m.peakAbsSpeed, m.peakAbsLoad,
                m.peakAbsCurrent);
  Serial.printf("MOTION_SUMMARY VOLTAGE_RANGE=%d..%d MAX_TEMP_RAW=%d "
                "WORST_STATUS=0x%02X\n",
                m.minVoltage, m.maxVoltage, m.maxTemp, m.worstStatus);
}

// Blocking, host-independent. Never reads Serial. Converges to torque OFF.
static bool monitoredMove(uint8_t id, int target, MotionSummary &m,
                          const char *&reason) {
  int n = st.FeedBack(id);
  if (n < 0 || st.Error != 0) {
    reason = "PRE_MOVE_FEEDBACK_FAILED";
    return false;
  }

  const int startPos = st.ReadPos(-1);
  if (startPos < 0 || startPos > 4095) {
    reason = "PRE_MOVE_POSITION_OUT_OF_DOMAIN";
    return false;
  }

  m.startPos = startPos;
  m.target = target;
  m.finalPos = startPos;
  m.samples = 0;
  m.elapsedUs = 0;
  m.firstMotionUs = 0;
  m.minPos = m.maxPos = startPos;
  m.peakAbsSpeed = m.peakAbsLoad = m.peakAbsCurrent = 0;
  m.minVoltage = 255;
  m.maxVoltage = 0;
  m.maxTemp = 0;
  m.worstStatus = 0;

  const int direction = (target > startPos) ? +1 : ((target < startPos) ? -1 : 0);
  const uint32_t timeoutUs =
      computeMoveTimeoutMs(startPos, target, CENTER_SPEED) * 1000UL;

  Serial.printf("MOVE_BEGIN START=%d TARGET=%d DIRECTION=%d WINDOW_MS=%lu\n",
                startPos, target, direction,
                (unsigned long)(timeoutUs / 1000UL));

  if (!provWritePosEx(id, target, "CENTER")) {
    reason = "CENTER_GOAL_WRITE_FAILED";
    return false;
  }

  const uint32_t moveStart = micros();
  uint32_t next = moveStart;

  uint32_t stableSamples = 0;
  uint32_t consecutiveFailures = 0;
  uint32_t overcurrentSamples = 0;
  uint32_t stallCurrentSamples = 0;
  uint32_t stallSignatureSamples = 0;
  uint32_t voltageBadSamples = 0;
  uint32_t statusBadSamples = 0;

  int lastProgressPos = startPos;
  uint32_t lastProgressUs = moveStart;
  bool firstMotionSeen = false;

  while ((uint32_t)(micros() - moveStart) < timeoutUs) {
    while ((int32_t)(micros() - next) < 0) {
      delayMicroseconds(20);
    }
    next += MON_PERIOD_US;

    if (st.FeedBack(id) < 0) {
      if (++consecutiveFailures >= MON_TELEMETRY_LOSS_SAMPLES) {
        reason = "TELEMETRY_LOST_PERSISTENT";
        m.elapsedUs = micros() - moveStart;
        return false;
      }
      continue;
    }
    consecutiveFailures = 0;

    const int pos = st.ReadPos(-1);
    const int speed = st.ReadSpeed(-1);
    const int load = st.ReadLoad(-1);
    const int voltage = st.ReadVoltage(-1);
    const int temperature = st.ReadTemper(-1);
    const int moving = st.ReadMove(-1);
    const int current = st.ReadCurrent(-1);
    const uint8_t status = st.Error;

    if (pos < 0 || pos > 4095) {
      reason = "POSITION_OUT_OF_DOMAIN";
      m.elapsedUs = micros() - moveStart;
      return false;
    }

    m.samples++;
    m.finalPos = pos;
    if (pos < m.minPos) m.minPos = pos;
    if (pos > m.maxPos) m.maxPos = pos;
    if (abs(speed) > m.peakAbsSpeed) m.peakAbsSpeed = abs(speed);
    if (abs(load) > m.peakAbsLoad) m.peakAbsLoad = abs(load);
    if (abs(current) > m.peakAbsCurrent) m.peakAbsCurrent = abs(current);
    if (voltage < m.minVoltage) m.minVoltage = voltage;
    if (voltage > m.maxVoltage) m.maxVoltage = voltage;
    if (temperature > m.maxTemp) m.maxTemp = temperature;
    if (status > m.worstStatus) m.worstStatus = status;

    // ---- electrical / thermal guards -------------------------------
    if (abs(current) >= MON_STALL_CURRENT_RAW) {
      if (++stallCurrentSamples >= MON_STALL_CURRENT_SAMPLES) {
        reason = "STALL_CURRENT";
        m.elapsedUs = micros() - moveStart;
        return false;
      }
    } else {
      stallCurrentSamples = 0;
    }

    if (abs(current) >= MON_OVERCURRENT_RAW) {
      if (++overcurrentSamples >= MON_OVERCURRENT_SAMPLES) {
        reason = "OVERCURRENT";
        m.elapsedUs = micros() - moveStart;
        return false;
      }
    } else {
      overcurrentSamples = 0;
    }

    if (voltage < MON_VOLTAGE_MIN || voltage > MON_VOLTAGE_MAX) {
      if (++voltageBadSamples >= MON_VOLTAGE_SAMPLES) {
        reason = "VOLTAGE_SAFETY_PERSISTENT";
        m.elapsedUs = micros() - moveStart;
        return false;
      }
    } else {
      voltageBadSamples = 0;
    }

    if (status != 0) {
      if (++statusBadSamples >= MON_STATUS_SAMPLES) {
        reason = "SERVO_STATUS_PERSISTENT";
        m.elapsedUs = micros() - moveStart;
        return false;
      }
    } else {
      statusBadSamples = 0;
    }

    if (temperature > MON_THERMAL_LIMIT_C) {
      const char *thermalReason = "THERMAL";
      if (!thermalGuard(id, thermalReason)) {
        reason = thermalReason;
        m.elapsedUs = micros() - moveStart;
        return false;
      }
    }

    // ---- progress tracking ------------------------------------------
    if (!firstMotionSeen && abs(pos - startPos) >= 1) {
      firstMotionSeen = true;
      m.firstMotionUs = micros() - moveStart;
    }
    if (abs(pos - lastProgressPos) >= 1) {
      lastProgressPos = pos;
      lastProgressUs = micros();
    }

    // ---- composite mechanical stall: load ALONE never stops a move ---
    const bool noRecentProgress =
        (uint32_t)(micros() - lastProgressUs) >= MON_STALL_NO_PROGRESS_US;
    if (abs(load) >= MON_OVERLOAD_LOAD_RAW &&
        abs(current) >= MON_RATED_CURRENT_RAW &&
        abs(speed) <= MON_STALL_LOW_SPEED_RAW && noRecentProgress) {
      if (++stallSignatureSamples >= MON_STALL_SIGNATURE_SAMPLES) {
        reason = "MECHANICAL_STALL_SIGNATURE";
        m.elapsedUs = micros() - moveStart;
        return false;
      }
    } else {
      stallSignatureSamples = 0;
    }

    // ---- wrong direction ---------------------------------------------
    int directionalTravel = 0;
    if (direction > 0) {
      directionalTravel = pos - startPos;
    } else if (direction < 0) {
      directionalTravel = startPos - pos;
    }
    if (direction != 0 && directionalTravel < -MON_DIRECTION_TOL) {
      reason = "WRONG_DIRECTION";
      m.elapsedUs = micros() - moveStart;
      return false;
    }

    // ---- no progress ---------------------------------------------------
    const int residual = abs(target - pos);
    if (residual > MON_NO_PROGRESS_RESIDUAL &&
        (uint32_t)(micros() - lastProgressUs) > MON_NO_PROGRESS_US) {
      reason = firstMotionSeen ? "NO_PROGRESS" : "NO_MOTION";
      m.elapsedUs = micros() - moveStart;
      return false;
    }

    // ---- settle --------------------------------------------------------
    if (abs(speed) <= MON_STATIONARY_SPEED && moving == 0) {
      if (++stableSamples >= MON_STABLE_SAMPLES) {
        m.elapsedUs = micros() - moveStart;
        if (residual > MON_REACHED_RESIDUAL) {
          reason = "SETTLED_OFF_TARGET";
          return false;
        }
        return true;
      }
    } else {
      stableSamples = 0;
    }
  }

  m.elapsedUs = micros() - moveStart;
  reason = "MOVE_TIMEOUT";
  return false;
}

// One authorized motion segment: Torque ON verified, monitored move, Torque OFF
// verified. Torque OFF is unconditional — it is attempted on the success path
// and on every failure path, so this function always converges to torque off or
// to an explicit failure the caller escalates to enterFault().
//
// The MotionSummary is value-initialised because monitoredMove() returns early,
// before filling it, when the pre-move FeedBack or position read fails.
static bool centerSegment(uint8_t id, int32_t target, const char *why, const char *&reason) {
  if (stage != STAGE_CENTER) {
    reason = "CENTER_SEGMENT_STAGE_REFUSED";
    return false;
  }
  if (target < 0 || target > 4095) {
    reason = "CENTER_SEGMENT_GOAL_OUT_OF_DOMAIN";
    return false;
  }

  Serial.printf("CENTER_SEGMENT WHY=%s TARGET_DISPLAYED=%ld\n", why,
                (long)target);

  if (!provWriteByte(id, REG_TORQUE_ENABLE, 1, "TorqueEnable")) {
    reason = "CENTER_SEGMENT_TORQUE_ON_FAILED";
    return false;
  }

  MotionSummary m{};
  const bool moved = monitoredMove(id, (int)target, m, reason);
  printMotionSummary(m);

  // Unconditional, success or failure.
  const bool offOk = torqueOffVerified(id, why);

  if (!moved) {
    return false;
  }
  if (!offOk) {
    reason = "CENTER_SEGMENT_TORQUE_OFF_FAILED";
    return false;
  }
  return true;
}

static bool runCentering(uint8_t id, int32_t oldOffset, const char *&reason) {
  Serial.println("STAGE_BEGIN NAME=CENTERING");

  // --- prime -------------------------------------------------------
  stage = STAGE_PRIME;

  if (!torqueOffVerified(id, "PRE_PRIME")) {
    reason = "PRE_PRIME_TORQUE_OFF_FAILED";
    return false;
  }

  if (!provWriteWord(id, REG_TORQUE_LIMIT, CENTER_TORQUE_LIMIT, "TorqueLimit")) {
    reason = "TORQUE_LIMIT_WRITE_FAILED";
    return false;
  }

  int beforePrime = st.ReadPos(id);
  if (beforePrime < 0 || beforePrime > 4095) {
    reason = "PRIME_POSITION_READ_FAILED";
    return false;
  }
  Serial.printf("PRIME_POSITION_BEFORE=%d\n", beforePrime);

  const bool primeAck = provWritePosEx(id, beforePrime, "PRIME");

  delay(WRITE_SETTLE_MS);
  int primeTorque = st.readByte(id, REG_TORQUE_ENABLE);
  Serial.printf("PRIME_TORQUE_AFTER=%d\n", primeTorque);

  // WritePosEx can arm torque — observed in 26/26 campaign runs. Force OFF
  // unconditionally before evaluating anything else.
  torqueTouched = true;
  if (!torqueOffVerified(id, "PRIME")) {
    reason = "PRIME_SAFE_OFF_FAILED";
    return false;
  }
  if (!primeAck) {
    reason = "PRIME_WRITE_ACK_FAILED";
    return false;
  }

  int afterPrime = st.ReadPos(id);
  if (afterPrime < 0 || afterPrime > 4095) {
    reason = "PRIME_POSITION_VERIFY_FAILED";
    return false;
  }

  const int32_t primeDelta = circularDelta(afterPrime, beforePrime);
  Serial.printf("PRIME_POSITION_AFTER=%d PRIME_DELTA=%ld WINDOW=%d\n",
                afterPrime, (long)primeDelta, PRIME_MAX_DELTA_TICKS);

  if (abs((int)primeDelta) > PRIME_MAX_DELTA_TICKS) {
    reason = "PRIME_MOVED_BEYOND_WINDOW";
    return false;
  }
  Serial.println("PRIME_RESULT PASS");

  // --- authorized motion transaction --------------------------------
  stage = STAGE_CENTER;

  const int32_t goal = centerGoalDisplayed(oldOffset);
  if (goal < 0 || goal > 4095) {
    reason = "CENTER_GOAL_OUT_OF_DOMAIN";
    return false;
  }
  Serial.printf("CENTER_GOAL_DISPLAYED=%ld\n", (long)goal);

  if (!centerSegment(id, goal, "POST_CENTER", reason)) {
    return false;
  }

  // --- acceptance on PHYSICAL raw, after settling -------------------
  int settled = st.ReadPos(id);
  if (settled < 0 || settled > 4095) {
    reason = "CENTER_SETTLE_READ_FAILED";
    return false;
  }
  int32_t physical = physicalRaw(settled, oldOffset);
  int32_t error = circularDelta(physical, RAW_CENTER);

  Serial.printf("CENTER_SETTLED DISPLAYED=%d PHYSICAL_RAW=%ld ERROR_TICKS=%ld "
                "TOLERANCE=%d\n",
                settled, (long)physical, (long)error, CENTER_ACCEPT_TICKS);

  // --- bounded staging re-approach ----------------------------------
  // Entered only when the settle missed +/-1 but is still inside the
  // correction window. At most CENTER_MAX_CORRECTIONS attempts, then closed.
  int32_t bias = 0;

  for (int attempt = 1;
       abs((int)error) > CENTER_ACCEPT_TICKS && attempt <= CENTER_MAX_CORRECTIONS;
       ++attempt) {

    if (abs((int)error) > MON_REACHED_RESIDUAL) {
      reason = "CENTER_ERROR_OUT_OF_CORRECTION_WINDOW";
      return false;
    }

    // Attempt 1 runs unbiased and measures the deadband for the standard
    // geometry. Later attempts feed the previous re-approach error back.
    if (attempt > 1) {
      bias -= error;
      if (abs((int)bias) > CENTER_MAX_BIAS_TICKS) {
        reason = "CENTER_BIAS_OUT_OF_WINDOW";
        return false;
      }
    }

    const int32_t approach = goal + bias;
    if (approach < 0 || approach > 4095) {
      reason = "CENTER_APPROACH_GOAL_OUT_OF_DOMAIN";
      return false;
    }
    const int32_t staging = centerStagingGoal(approach, CENTER_STAGING_TICKS);
    if (staging < 0 || staging > 4095) {
      reason = "CENTER_STAGING_GOAL_OUT_OF_DOMAIN";
      return false;
    }

    Serial.printf("CENTER_CORRECTION BEGIN ATTEMPT=%d OF=%d ERROR_TICKS=%ld "
                  "BIAS_TICKS=%ld STAGING_DISPLAYED=%ld APPROACH_DISPLAYED=%ld "
                  "GOAL_DISPLAYED=%ld WHY=%s\n",
                  attempt, CENTER_MAX_CORRECTIONS, (long)error, (long)bias,
                  (long)staging, (long)approach, (long)goal,
                  "TERMINAL_DEADBAND_COMPENSATION");

    if (!centerSegment(id, staging, "CENTER_STAGING", reason)) {
      return false;
    }
    if (!centerSegment(id, approach, "CENTER_REAPPROACH", reason)) {
      return false;
    }

    settled = st.ReadPos(id);
    if (settled < 0 || settled > 4095) {
      reason = "CENTER_CORRECTION_SETTLE_READ_FAILED";
      return false;
    }
    physical = physicalRaw(settled, oldOffset);
    error = circularDelta(physical, RAW_CENTER);

    Serial.printf("CENTER_CORRECTION_SETTLED ATTEMPT=%d BIAS_TICKS=%ld "
                  "DISPLAYED=%d PHYSICAL_RAW=%ld ERROR_TICKS=%ld TOLERANCE=%d\n",
                  attempt, (long)bias, settled, (long)physical, (long)error,
                  CENTER_ACCEPT_TICKS);
  }

  // Single acceptance gate. Never widened, never bypassed.
  if (abs((int)error) > CENTER_ACCEPT_TICKS) {
    reason = "CENTER_ERROR_OUT_OF_TOLERANCE";
    return false;
  }

  Serial.println("STAGE_RESULT NAME=CENTERING PASS");
  return true;
}

// --------------------------------------------------------------------------
// Cold power-cycle detection — read-only, ESP32-RAM state machine
// --------------------------------------------------------------------------

static bool detectColdCycle(uint8_t id, uint8_t index, const char *&reason) {
  Serial.printf("COLD_CYCLE_BEGIN INDEX=%u REQUIRED=%u\n", index,
                sessionColdCycles);

  // Precondition: the target must be present NOW. An ESP32 reset destroys this
  // state machine, so a fresh boot can never satisfy a cold cycle.
  if (st.Ping(id) != id) {
    reason = "COLD_PRECONDITION_TARGET_ABSENT";
    return false;
  }
  Serial.println("COLD_STATE=PRESENT");
  Serial.println("POWER CYCLE SERVO NOW");

  const uint32_t windowStart = millis();
  uint32_t absentSince = 0;
  bool absenceConfirmed = false;
  uint32_t stableCount = 0;

  while ((uint32_t)(millis() - windowStart) < COLD_OPERATOR_WINDOW_MS) {
    delay(COLD_POLL_INTERVAL_MS);

    const bool present = (st.Ping(id) == id);

    if (!absenceConfirmed) {
      if (!present) {
        if (absentSince == 0) {
          absentSince = millis();
          Serial.println("COLD_STATE=ABSENT_CANDIDATE");
        } else if ((uint32_t)(millis() - absentSince) >=
                   COLD_ABSENCE_DEBOUNCE_MS) {
          absenceConfirmed = true;
          Serial.printf("COLD_STATE=ABSENT_CONFIRMED DEBOUNCE_MS=%lu\n",
                        (unsigned long)COLD_ABSENCE_DEBOUNCE_MS);
        }
      } else {
        if (absentSince != 0) {
          Serial.println("COLD_STATE=ABSENCE_ABORTED_TOO_SHORT");
        }
        absentSince = 0;
      }
      continue;
    }

    // Absence confirmed: wait for a stable return.
    if (present && st.Error == 0) {
      stableCount++;
      if (stableCount == 1) {
        Serial.println("COLD_STATE=RETURNED");
      }
      if (stableCount >= COLD_RETURN_STABLE_SAMPLES) {
        int model = st.readWord(id, REG_MODEL);
        if (model != (int)EXPECTED_MODEL || st.Error != 0) {
          reason = "COLD_RETURN_MODEL_MISMATCH";
          return false;
        }
        Serial.printf("COLD_STATE=STABLE SAMPLES=%lu\n",
                      (unsigned long)stableCount);
        return true;
      }
    } else {
      stableCount = 0;
    }
  }

  reason = absenceConfirmed ? "COLD_RETURN_TIMEOUT" : "COLD_ABSENCE_TIMEOUT";
  return false;
}

static bool verifyColdState(const uint8_t *raw, const char *&reason) {
  if (u16le(raw + REG_MODEL) != EXPECTED_MODEL) {
    reason = "COLD_MODEL_MISMATCH";
    return false;
  }
  if (raw[REG_ID] != sessionTargetId) {
    reason = "COLD_ID_MISMATCH";
    return false;
  }
  if (raw[REG_BAUD] != EXPECTED_BAUD) {
    reason = "COLD_BAUD_MISMATCH";
    return false;
  }
  if (i16le_twos(raw + REG_POSITION_OFFSET) != 0) {
    reason = "COLD_OFFSET_NOT_ZERO";
    return false;
  }
  if (!profileExact(raw, true)) {
    reason = "COLD_PROFILE_MISMATCH";
    return false;
  }
  if (raw[REG_LOCK] != 1) {
    reason = "COLD_LOCK_NOT_SET";
    return false;
  }
  if (raw[REG_TORQUE_ENABLE] != 0) {
    reason = "COLD_TORQUE_NOT_OFF";
    return false;
  }

  const int32_t displayed = u16le(raw + REG_PRESENT_POSITION);
  const int32_t error = circularDelta(displayed, RAW_CENTER);
  Serial.printf("COLD_POSITION DISPLAYED=%ld ERROR_TICKS=%ld TOLERANCE=%d\n",
                (long)displayed, (long)error, CENTER_ACCEPT_TICKS);
  if (abs((int)error) > CENTER_ACCEPT_TICKS) {
    reason = "COLD_POSITION_OUT_OF_TOLERANCE";
    return false;
  }

  // Observed, never asserted, never treated as operating policy.
  Serial.printf("COLD_OBSERVED_TORQUE_LIMIT=%u POLICY=NOT_PERSISTENT_OPERATING\n",
                u16le(raw + REG_TORQUE_LIMIT));
  return true;
}

// --------------------------------------------------------------------------
// @EXECUTE
// --------------------------------------------------------------------------

static void runExecute(const char *token) {
  if (sessionState != SESSION_WAIT_EXECUTE) {
    Serial.println("EXECUTE_ABORT: NO_SESSION_WAITING");
    Serial.println("EXECUTE_RESULT FAIL");
    return;
  }
  if (sessionToken[0] == '\0' || strcmp(token, sessionToken) != 0) {
    Serial.println("EXECUTE_ABORT: SESSION_TOKEN_MISMATCH");
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  // Single-use token: consumed now, whatever happens next.
  sessionToken[0] = '\0';
  sessionState = SESSION_RUNNING;

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" PROVISION EXECUTE — %s\n", sessionLabel);
  Serial.println("====================================");

  if (PROVISIONER_HARDWARE_FREEZE_BLOCKED) {
    Serial.println("EXECUTE_ABORT: HARDWARE_FREEZE_BLOCKED");
    reportSelectedConstants();
    Serial.println("EEPROM_WRITES=NONE");
    Serial.println("MOTION=NONE");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  const uint8_t sourceId = sessionSourceId;
  const uint8_t targetId = sessionTargetId;
  const int32_t oldOffset = sessionOldOffset;
  const char *reason = "UNKNOWN";
  uint8_t raw[SNAPSHOT_LEN];

  // ---- stage 1: torque off ----------------------------------------
  stage = STAGE_TORQUE_SAFETY;
  Serial.println("STAGE_BEGIN NAME=TORQUE_OFF");
  if (!torqueOffVerified(sourceId, "ENTRY")) {
    enterFault(sourceId, "ENTRY_TORQUE_OFF_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }
  Serial.println("STAGE_RESULT NAME=TORQUE_OFF PASS");

  // ---- stage 2: canonical profile ---------------------------------
  stage = STAGE_PROFILE;
  Serial.println("STAGE_BEGIN NAME=PROFILE");

  bool needed[PROFILE_COUNT];
  int writeCount = 0;
  for (size_t i = 0; i < PROFILE_COUNT; ++i) {
    uint16_t actual = profileFieldFromRaw(sessionBefore, PROFILE[i]);
    needed[i] = actual != PROFILE[i].value;
    if (needed[i]) writeCount++;
  }
  Serial.printf("PROFILE_DELTA_WRITES=%d\n", writeCount);

  if (writeCount > 0) {
    if (!unlockVerified(sourceId)) {
      enterFault(sourceId, "PROFILE_UNLOCK_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
    for (size_t i = 0; i < PROFILE_COUNT; ++i) {
      if (!needed[i]) continue;
      bool ok = PROFILE[i].width == 2
                    ? provWriteWord(sourceId, PROFILE[i].addr, PROFILE[i].value,
                                    PROFILE[i].name)
                    : provWriteByte(sourceId, PROFILE[i].addr,
                                    (uint8_t)PROFILE[i].value, PROFILE[i].name);
      if (!ok) {
        enterFault(sourceId, "PROFILE_WRITE_VERIFY_FAILED");
        Serial.println("EXECUTE_RESULT FAIL");
        clearSession();
        return;
      }
    }
    if (!lockVerified(sourceId)) {
      enterFault(sourceId, "PROFILE_LOCK_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
  } else {
    Serial.println("PROFILE_TRANSACTION=SKIPPED_ALREADY_CANONICAL");
  }

  if (!captureAndEmit("PROFILE", sourceId, raw)) {
    enterFault(sourceId, "PROFILE_SNAPSHOT_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }
  if (!profileExact(raw, true) || raw[REG_LOCK] != 1) {
    enterFault(sourceId, "PROFILE_VERIFY_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }
  Serial.println("STAGE_RESULT NAME=PROFILE PASS");

  // ---- stage 3: centering ------------------------------------------
  if (!runCentering(sourceId, oldOffset, reason)) {
    enterFault(sourceId, reason);
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }
  if (!captureAndEmit("CENTERED", sourceId, raw)) {
    enterFault(sourceId, "CENTERED_SNAPSHOT_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  // ---- stage 4: PositionOffset = 0 ---------------------------------
  stage = STAGE_OFFSET_ZERO;
  Serial.println("STAGE_BEGIN NAME=OFFSET_ZERO");

  if (!torqueOffVerified(sourceId, "PRE_OFFSET")) {
    enterFault(sourceId, "PRE_OFFSET_TORQUE_OFF_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  int posBeforeOffset = st.ReadPos(sourceId);
  if (!unlockVerified(sourceId) ||
      !provWriteWord(sourceId, REG_POSITION_OFFSET, 0, "PositionOffset") ||
      !lockVerified(sourceId)) {
    enterFault(sourceId, "OFFSET_ZERO_TRANSACTION_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  if (!positionSettled(sourceId, reason)) {
    enterFault(sourceId, reason);
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  if (!captureAndEmit("OFFSET_ZERO", sourceId, raw)) {
    enterFault(sourceId, "OFFSET_ZERO_SNAPSHOT_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }
  {
    const int32_t newOffset = i16le_twos(raw + REG_POSITION_OFFSET);
    const int32_t displayed = u16le(raw + REG_PRESENT_POSITION);
    const int32_t physical = physicalRaw(displayed, newOffset);
    const int32_t err = circularDelta(physical, RAW_CENTER);

    Serial.printf("OFFSET_ZERO_PROOF OLD_OFFSET=%ld NEW_OFFSET=%ld "
                  "POS_BEFORE=%d DISPLAYED=%ld PHYSICAL_RAW=%ld ERROR=%ld\n",
                  (long)oldOffset, (long)newOffset, posBeforeOffset,
                  (long)displayed, (long)physical, (long)err);

    if (newOffset != 0 || raw[REG_LOCK] != 1 ||
        raw[REG_TORQUE_ENABLE] != 0 || abs((int)err) > CENTER_ACCEPT_TICKS) {
      enterFault(sourceId, "OFFSET_ZERO_VERIFY_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
  }
  Serial.println("STAGE_RESULT NAME=OFFSET_ZERO PASS");

  // ---- stage 5: ID recode ------------------------------------------
  stage = STAGE_ID_RECODE;
  Serial.println("STAGE_BEGIN NAME=ID_RECODE");

  if (sourceId == targetId) {
    Serial.println("ID_RECODE=SKIPPED_SOURCE_EQUALS_TARGET");
  } else {
    if (!torqueOffVerified(sourceId, "PRE_RECODE")) {
      enterFault(sourceId, "PRE_RECODE_TORQUE_OFF_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
    if (!unlockVerified(sourceId)) {
      enterFault(sourceId, "RECODE_UNLOCK_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }

    // ONE write, through the allowlist. No retry, ever.
    int ack = 0;
    uint8_t writeStatus = 0xFF;
    if (!provWriteIdAdvisory(sourceId, targetId, ack, writeStatus)) {
      enterFault(sourceId, "ID_WRITE_REFUSED_BY_ALLOWLIST");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }

    delay(WRITE_SETTLE_MS);

    const bool targetResponds = (st.Ping(targetId) == (int)targetId);
    const uint8_t targetPingStatus = st.Error;
    const bool oldResponds = (st.Ping(sourceId) == (int)sourceId);

    Serial.printf("ID_PROOF TARGET_RESPONDS=%s TARGET_STATUS=0x%02X "
                  "OLD_RESPONDS=%s\n",
                  targetResponds ? "YES" : "NO", targetPingStatus,
                  oldResponds ? "YES" : "NO");

    if (!targetResponds || oldResponds || targetPingStatus != 0) {
      Serial.println("FAIL_AMBIGUOUS_RECODE");
      // Best effort on whichever identity answers; no retry, no rollback.
      enterFault(targetResponds ? targetId : sourceId, "FAIL_AMBIGUOUS_RECODE");
      Serial.println("CUT_SERVO_POWER_NOW");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }

    int model = st.readWord(targetId, REG_MODEL);
    if (model != (int)EXPECTED_MODEL || st.Error != 0) {
      Serial.println("FAIL_AMBIGUOUS_RECODE");
      enterFault(targetId, "RECODE_MODEL_MISMATCH");
      Serial.println("CUT_SERVO_POWER_NOW");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }

    // Lock is restored under the NEW identity.
    if (!lockVerified(targetId)) {
      enterFault(targetId, "RECODE_LOCK_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
  }
  Serial.println("STAGE_RESULT NAME=ID_RECODE PASS");

  // ---- stage 6: warm final ------------------------------------------
  stage = STAGE_NONE;
  Serial.println("STAGE_BEGIN NAME=WARM_FINAL");

  if (!positionSettled(targetId, reason)) {
    enterFault(targetId, reason);
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }

  if (!captureAndEmit("WARM", targetId, raw)) {
    enterFault(targetId, "WARM_SNAPSHOT_FAILED");
    Serial.println("EXECUTE_RESULT FAIL");
    clearSession();
    return;
  }
  {
    const int32_t offset = i16le_twos(raw + REG_POSITION_OFFSET);
    const int32_t displayed = u16le(raw + REG_PRESENT_POSITION);
    const int32_t err = circularDelta(displayed, RAW_CENTER);
    const bool ok = u16le(raw + REG_MODEL) == EXPECTED_MODEL &&
                    raw[REG_ID] == targetId && raw[REG_BAUD] == EXPECTED_BAUD &&
                    offset == 0 && profileExact(raw, true) &&
                    raw[REG_LOCK] == 1 && raw[REG_TORQUE_ENABLE] == 0 &&
                    abs((int)err) <= CENTER_ACCEPT_TICKS;

    Serial.printf("WARM_PROOF ID=%u MODEL=%u BAUD=%u OFFSET=%ld LOCK=%u "
                  "TORQUE=%u DISPLAYED=%ld ERROR=%ld RESULT=%s\n",
                  raw[REG_ID], u16le(raw + REG_MODEL), raw[REG_BAUD],
                  (long)offset, raw[REG_LOCK], raw[REG_TORQUE_ENABLE],
                  (long)displayed, (long)err, ok ? "PASS" : "FAIL");

    if (!ok) {
      enterFault(targetId, "WARM_VERIFY_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
  }
  Serial.println("STAGE_RESULT NAME=WARM_FINAL PASS");

  // ---- stage 7: cold power cycle(s) ---------------------------------
  for (uint8_t cycle = 1; cycle <= sessionColdCycles; ++cycle) {
    if (!detectColdCycle(targetId, cycle, reason)) {
      Serial.printf("COLD_CYCLE_RESULT INDEX=%u FAIL REASON=%s\n", cycle,
                    reason);
      enterFault(targetId, reason);
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }

    char slot[8];
    snprintf(slot, sizeof(slot), "COLD%u", cycle);

    if (!captureAndEmit(slot, targetId, raw)) {
      Serial.printf("COLD_CYCLE_RESULT INDEX=%u FAIL REASON=%s\n", cycle,
                    "COLD_SNAPSHOT_FAILED");
      enterFault(targetId, "COLD_SNAPSHOT_FAILED");
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
    if (!verifyColdState(raw, reason)) {
      Serial.printf("COLD_CYCLE_RESULT INDEX=%u FAIL REASON=%s\n", cycle,
                    reason);
      enterFault(targetId, reason);
      Serial.println("EXECUTE_RESULT FAIL");
      clearSession();
      return;
    }
    Serial.printf("COLD_CYCLE_RESULT INDEX=%u PASS\n", cycle);
  }

  Serial.printf("PROVISION_LABEL=%s\n", sessionLabel);
  Serial.printf("PROVISION_SOURCE_ID=%u\n", sourceId);
  Serial.printf("PROVISION_TARGET_ID=%u\n", targetId);
  Serial.printf("PROVISION_PROFILE=%s\n", PROFILE_ID);
  Serial.println("EXECUTE_RESULT PASS");

  sessionState = SESSION_DONE;
  clearSession();
}

// --------------------------------------------------------------------------
// Parser — the only entry point for host input
// --------------------------------------------------------------------------

static void printHelp() {
  Serial.println();
  Serial.println("COMMAND_SURFACE_BEGIN");
  Serial.println("  @BEGIN <PHYSICAL_LABEL>");
  Serial.println("  @EXECUTE <SESSION_TOKEN>");
  Serial.println("  @HELP");
  Serial.println("COMMAND_SURFACE_END");
  Serial.println("GENERIC_EEPROM_WRITE   : NOT IMPLEMENTED");
  Serial.println("ARBITRARY_ID_WRITE     : NOT IMPLEMENTED");
  Serial.println("ARBITRARY_GOAL_POSITION: NOT IMPLEMENTED");
  Serial.println("CALIBRATION_OFS        : NOT IMPLEMENTED");
  Serial.println("HOST_TORQUE_ON         : NOT IMPLEMENTED");
  Serial.println("FACTORY_RESET          : NOT IMPLEMENTED");
  Serial.println("BROADCAST_WRITE        : NOT IMPLEMENTED");
  Serial.println("BAUD_WRITE             : NOT IMPLEMENTED");

  Serial.print("PROFILE_ALLOWLIST=");
  for (size_t i = 0; i < PROFILE_COUNT; ++i) {
    Serial.printf("%s0x%02X", i ? "," : "", PROFILE[i].addr);
  }
  Serial.println();

  Serial.print("PRESERVE_ONLY=");
  for (size_t i = 0; i < PRESERVE_COUNT; ++i) {
    Serial.printf("%s0x%02X", i ? "," : "", PRESERVE_ONLY[i]);
  }
  Serial.println();

  Serial.print("ALLOCATION=");
  for (size_t i = 0; i < UNIT_COUNT; ++i) {
    Serial.printf("%s%s:%u", i ? "," : "", UNITS[i].label, UNITS[i].targetId);
  }
  Serial.println();
  Serial.println();
}

static bool parseLabel(const String &cmd, char *out, size_t outLen) {
  if (!cmd.startsWith("@BEGIN ")) {
    return false;
  }
  String arg = cmd.substring(7);
  if (arg.length() == 0 || arg.length() >= outLen) {
    return false;
  }
  for (size_t i = 0; i < arg.length(); ++i) {
    char c = arg[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
      return false;
    }
  }
  strncpy(out, arg.c_str(), outLen - 1);
  out[outLen - 1] = '\0';
  return true;
}

static bool parseToken(const String &cmd, char *out, size_t outLen) {
  if (!cmd.startsWith("@EXECUTE ")) {
    return false;
  }
  String arg = cmd.substring(9);
  if (arg.length() != 8) {
    return false;
  }
  for (size_t i = 0; i < 8; ++i) {
    char c = arg[i];
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
  Serial.println(" MATDOG ST3215 PROVISIONER V1");
  Serial.println("====================================");
  Serial.printf("FIRMWARE         : %s %s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
  Serial.printf("PROFILE          : %s\n", PROFILE_ID);
  Serial.printf("Servo UART       : %lu baud\n", SERVO_BAUD);
  Serial.printf("TX               : GPIO%d\n", SERVO_TX_PIN);
  Serial.printf("RX               : GPIO%d\n", SERVO_RX_PIN);
  Serial.println();
  Serial.println("Startup motion   : IMPOSSIBLE BY DESIGN");
  Serial.println("@BEGIN writes    : NONE");
  Serial.println("Center           : PHYSICAL RAW 2048");
  Serial.printf("Center params    : TL=%u SPEED=%u ACC=%u\n",
                CENTER_TORQUE_LIMIT, CENTER_SPEED, CENTER_ACC);
  reportSelectedConstants();
  Serial.println("PROVISIONER_READY");

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

  if (cmd == "@HELP") {
    printHelp();
    return;
  }

  char label[16];
  if (parseLabel(cmd, label, sizeof(label))) {
    runBegin(label);
    return;
  }

  char token[9];
  if (parseToken(cmd, token, sizeof(token))) {
    runExecute(token);
    return;
  }

  Serial.println("ERROR: UNKNOWN_COMMAND");
  Serial.println("EEPROM_WRITES=NONE");
  Serial.println("MOTION=NONE");
}
