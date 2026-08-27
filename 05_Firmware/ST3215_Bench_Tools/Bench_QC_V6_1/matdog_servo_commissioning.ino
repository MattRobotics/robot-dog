#include <Arduino.h>
#include <SCServo.h>
#include "esp_heap_caps.h"
#include <FFat.h>
#include <FS.h>

static constexpr int SERVO_TX_PIN = 17;
static constexpr int SERVO_RX_PIN = 18;
static constexpr uint32_t SERVO_BAUD = 1000000;

HardwareSerial ServoUART(1);
SMS_STS st;


enum QCPhase : uint8_t {
  QC_STATIC_PRE      = 1,
  QC_PREPOSITION     = 2,
  QC_SLOW_DOWN       = 3,
  QC_SLOW_UP         = 4,
  QC_MIN_PROBE       = 5,
  QC_MAX_DOWN        = 6,
  QC_MAX_UP          = 7,
  QC_MAX_PRECISION   = 8,
  QC_STATIC_POST     = 9,
  QC_TRANSFER        = 10,
  QC_STEP45_MEDIUM   = 11
};

struct __attribute__((packed)) QCFullSample {
  uint32_t t_us;
  uint16_t goal_position;
  uint16_t goal_speed;
  uint8_t goal_acc;
  uint8_t phase;
  uint8_t proto_status;
  uint8_t flags;
  uint8_t raw[15];       // exact SRAM block 56..70
};


enum QCMoveOutcome : uint8_t {
  QCM_REACHED = 0,
  QCM_SETTLED_OFF_TARGET = 1,
  QCM_NO_MOTION = 2,
  QCM_NO_PROGRESS = 3,
  QCM_WINDOW_END = 4,
  QCM_PROTECT_CURRENT = 5,
  QCM_PROTECT_LOAD = 6,
  QCM_PROTECT_STATUS = 7,
  QCM_PROTECT_DIRECTION = 8,
  QCM_PROTECT_STALL = 9,
  QCM_OBSERVATION_COMPLETE = 10
};

struct QCMoveResult {
  QCMoveOutcome outcome;

  int startPos;
  int targetPos;
  int finalPos;

  int minPos;
  int maxPos;

  int peakAbsSpeed;
  int peakAbsLoad;
  int peakAbsCurrent;

  uint32_t elapsedUs;
  uint32_t firstMotionUs;

  bool firstMotionSeen;
};

struct __attribute__((packed)) QCFileHeader {
  char magic[8];
  uint32_t version;
  uint32_t servo_id;
  uint32_t sample_hz;
  uint32_t sample_size;
  uint32_t sample_count;
  uint32_t elapsed_us;
  uint32_t result_code;
  char abort_reason[48];
};

static QCFullSample *qcBuffer = nullptr;
static uint32_t qcCount = 0;
static uint32_t qcStartUs = 0;

static constexpr uint32_t QC_FAST_HZ = 500;
static constexpr uint32_t QC_FAST_PERIOD_US = 2000;
static constexpr uint32_t QC_FAST_MAX_SAMPLES = 90000;

// Stop starting/continuing motion early enough to guarantee cleanup
// before the user's absolute 180 s limit.
static constexpr uint32_t QC_GLOBAL_ABORT_US = 175000000UL;

// QC V6.1 standardized observation windows.
static constexpr uint32_t QC_SLOW_SEGMENT_WINDOW_MS = 6000UL;
static constexpr uint32_t QC_MEDIUM_SEGMENT_WINDOW_MS = 2500UL;
static constexpr uint32_t QC_MAX_FULL_WINDOW_MS = 7000UL;

// Do not start unrestricted MAX too late in the run.
static constexpr uint32_t QC_MAX_START_LATEST_US = 145000000UL;

// Stop launching new precision/reversal probes while enough
// time still remains for deterministic cleanup.
static constexpr uint32_t QC_PRECISION_STOP_US = 163000000UL;

// If a very poor servo consumes excessive time before/during
// minimum-speed characterization, finish the current
// observation and preserve the rest of the dataset.
static constexpr uint32_t QC_MIN_CONTINUE_LIMIT_US = 145000000UL;
static constexpr uint32_t QC_PROTOCOL_VERSION = 61;


static constexpr int QC_THERMAL_LIMIT_C = 70;
static constexpr uint32_t QC_THERMAL_CHECK_PERIOD_US = 200000UL; // 5 Hz
static constexpr uint32_t QC_THERMAL_CONFIRM_DELAY_MS = 5;
static constexpr int QC_THERMAL_CONFIRM_SAMPLES = 3;

static uint32_t qcNextThermalCheckUs = 0;
static uint32_t qcThermalTransientCount = 0;
static uint32_t qcThermalConfirmedCount = 0;


// ------------------------------------------------------------------
// V4 CHARACTERIZATION POLICY
//
// Performance anomalies NEVER abort the whole QC.
// They are measured, recorded and the current excitation is stopped.
//
// The existing MATDOG V25 hard-current value is reused here only
// as a protective cutoff for the CURRENT PHASE, never as a quality
// threshold.
// ------------------------------------------------------------------

static constexpr float QC_CURRENT_MA_PER_RAW = 6.5f;

// Manufacturer references, rounded conservatively upward.
static constexpr int QC_NOLOAD_CURRENT_RAW = 28;   // ~180 mA
static constexpr int QC_RATED_CURRENT_RAW = 139;  // ~900 mA
static constexpr int QC_OVERCURRENT_2A_RAW = 308; // ~2.0 A
static constexpr int QC_STALL_CURRENT_RAW = 416;  // ~2.7 A

// Feetech overload reference: ~80% of full drive/stall.
static constexpr int QC_OVERLOAD_LOAD_RAW = 800;

// We intentionally intervene earlier than Feetech's own
// nominal 2-second electronic protection to protect a
// mechanically damaged bench servo.
static constexpr uint32_t QC_OVERCURRENT_PERSIST_SAMPLES = 50; // 100 ms
static constexpr uint32_t QC_STALL_CURRENT_PERSIST_SAMPLES = 10; // 20 ms

// Composite mechanical-jam signature.
// Load ALONE can never stop the test.
static constexpr int QC_STALL_LOW_SPEED_RAW = 50;
static constexpr uint32_t QC_STALL_NO_PROGRESS_US = 250000UL;
static constexpr uint32_t QC_STALL_SIGNATURE_SAMPLES = 50; // additional 100 ms

static constexpr uint32_t QC_STATUS_PERSIST_SAMPLES = 25;  // ~50 ms
static constexpr uint32_t QC_VOLTAGE_PERSIST_SAMPLES = 50; // ~100 ms

static constexpr uint8_t QC_STATUS_VOLTAGE  = (1u << 0);
static constexpr uint8_t QC_STATUS_SENSOR   = (1u << 1);
static constexpr uint8_t QC_STATUS_TEMP     = (1u << 2);
static constexpr uint8_t QC_STATUS_CURRENT  = (1u << 3);
static constexpr uint8_t QC_STATUS_ANGLE    = (1u << 4);
static constexpr uint8_t QC_STATUS_OVERLOAD = (1u << 5);

static uint32_t qcPerformanceEvents = 0;
static uint32_t qcProtectiveStops = 0;
static uint32_t qcNoMotionEvents = 0;
static uint32_t qcNoProgressEvents = 0;
static uint32_t qcOffTargetEvents = 0;
static uint32_t qcWindowEndEvents = 0;
static uint32_t qcSkippedTests = 0;

static uint32_t qcMinAttempts = 0;
static uint32_t qcMinResponses = 0;

static int qcObservedMin = 4095;
static int qcObservedMax = 0;

static bool qcUnsafeForMax = false;




static bool qcLastAvailable = false;
static bool qcLastPassed = false;
static uint32_t qcLastElapsedUs = 0;
static int qcLastServoId = -1;
static char qcLastReason[48] = {0};




struct QCSample {
  uint32_t t_us;

  int16_t position;
  int16_t speed;
  int16_t load;
  int16_t current;

  uint16_t goalPosition;
  uint16_t goalSpeed;

  uint8_t voltage;
  uint8_t temperature;
  uint8_t moving;
  uint8_t status;
  uint8_t phase;
  uint8_t flags;
};

static constexpr uint32_t QC_HZ = 500;
static constexpr uint32_t QC_HARD_LIMIT_S = 180;
static constexpr uint32_t QC_MAX_SAMPLES =
    QC_HZ * QC_HARD_LIMIT_S;


struct MotionStats {
  uint32_t samples;
  uint32_t feedbackFailures;
  uint32_t missedDeadlines;
  uint32_t movingSamples;

  int startPos;
  int targetPos;
  int finalPos;

  int minPos;
  int maxPos;
  int minSpeed;
  int maxSpeed;
  int minLoad;
  int maxLoad;
  int minVoltage;
  int maxVoltage;
  int minTemp;
  int maxTemp;
  int minCurrent;
  int maxCurrent;

  uint32_t firstMotionUs;
  uint32_t settledUs;

  int64_t maxAbsSpeedAccel;
};


void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  @PING <id>");
  Serial.println("  @SCAN <min_id> <max_id>");
  Serial.println("  @READ <id>");
  Serial.println("  @NORMALIZE_MATDOG <id>");
  Serial.println("  @BENCH <id> <duration_ms>");
  Serial.println("  @CAPTURE <id> <duration_ms> <hz>");
    Serial.println("  @SAFE_OFF <id>");
  Serial.println("  @RAMTEST <id>");
  Serial.println("  @QC_FAST <id>");
  Serial.println("  @DUMP_RAW <id>");
  Serial.println("  @HELP");
  Serial.println();
}

bool validID(int id) {
  return id >= 0 && id <= 253;
}

void readServo(int id) {

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" READ-ONLY SNAPSHOT ID %d\n", id);
  Serial.println("====================================");

  int ping = st.Ping((uint8_t)id);

  if (ping < 0) {
    Serial.printf("PING              : TIMEOUT\n");
    Serial.println("READ_ABORTED");
    return;
  }

  Serial.printf("PING              : OK ID=%d STATUS=0x%02X\n",
                ping, st.Error);

  int model       = st.readWord(id, SMS_STS_MODEL_L);
  int storedID    = st.readByte(id, SMS_STS_ID);
  int baudReg     = st.readByte(id, SMS_STS_BAUD_RATE);

  int minLimit    = st.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
  int maxLimit    = st.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);

  int cwDead      = st.readByte(id, SMS_STS_CW_DEAD);
  int ccwDead     = st.readByte(id, SMS_STS_CCW_DEAD);

  int offsetRaw   = st.readWord(id, SMS_STS_OFS_L);
  int mode        = st.readByte(id, SMS_STS_MODE);

  // Extended EEPROM configuration snapshot.
  // Addresses cross-checked against the current NormaCore ST3215 register map.
  int maxTorque             = st.readWord(id, 0x10);
  int pCoef                 = st.readByte(id, 0x15);
  int dCoef                 = st.readByte(id, 0x16);
  int iCoef                 = st.readByte(id, 0x17);
  int minStartupForce       = st.readWord(id, 0x18);
  int protectionCurrent     = st.readWord(id, 0x1C);
  int protectionTorque      = st.readByte(id, 0x22);
  int protectionTime        = st.readByte(id, 0x23);
  int overloadTorque        = st.readByte(id, 0x24);
  int speedClosedLoopP      = st.readByte(id, 0x25);
  int overCurrentProtTime   = st.readByte(id, 0x26);
  int velocityClosedLoopI   = st.readByte(id, 0x27);

  // Runtime SRAM configuration.
  int torque      = st.readByte(id, SMS_STS_TORQUE_ENABLE);
  int acc         = st.readByte(id, SMS_STS_ACC);
  int goalSpeed   = st.readWord(id, SMS_STS_GOAL_SPEED_L);
  int torqueLimit = st.readWord(id, SMS_STS_TORQUE_LIMIT_L);
  int lock        = st.readByte(id, SMS_STS_LOCK);

  int position    = st.ReadPos(id);
  int speed       = st.ReadSpeed(id);
  int load        = st.ReadLoad(id);
  int voltage     = st.ReadVoltage(id);
  int temperature = st.ReadTemper(id);
  int moving      = st.ReadMove(id);
  int current     = st.ReadCurrent(id);

  Serial.println();
  Serial.println("--- EEPROM / CONFIG ---");
  Serial.printf("MODEL RAW         : %d (0x%04X)\n", model, model);
  Serial.printf("ID REGISTER       : %d\n", storedID);
  Serial.printf("BAUD REGISTER     : %d\n", baudReg);
  Serial.printf("MIN LIMIT RAW     : %d\n", minLimit);
  Serial.printf("MAX LIMIT RAW     : %d\n", maxLimit);
  Serial.printf("CW DEAD           : %d\n", cwDead);
  Serial.printf("CCW DEAD          : %d\n", ccwDead);
  Serial.printf("OFFSET RAW        : %d (0x%04X)\n", offsetRaw, offsetRaw);
  Serial.printf("MODE              : %d\n", mode);
  Serial.printf("MAX TORQUE        : %d\n", maxTorque);
  Serial.printf("P COEF            : %d\n", pCoef);
  Serial.printf("D COEF            : %d\n", dCoef);
  Serial.printf("I COEF            : %d\n", iCoef);
  Serial.printf("MIN START FORCE   : %d\n", minStartupForce);
  Serial.printf("PROT CURRENT      : %d\n", protectionCurrent);
  Serial.printf("PROT TORQUE       : %d\n", protectionTorque);
  Serial.printf("PROT TIME         : %d\n", protectionTime);
  Serial.printf("OVERLOAD TORQUE   : %d\n", overloadTorque);
  Serial.printf("SPEED LOOP P      : %d\n", speedClosedLoopP);
  Serial.printf("OVERCURR TIME     : %d\n", overCurrentProtTime);
  Serial.printf("VELOCITY LOOP I   : %d\n", velocityClosedLoopI);

  Serial.println();
  Serial.println("--- SRAM / RUNTIME ---");
  Serial.printf("TORQUE ENABLE     : %d\n", torque);
  Serial.printf("ACC               : %d\n", acc);
  Serial.printf("GOAL SPEED        : %d\n", goalSpeed);
  Serial.printf("TORQUE LIMIT      : %d\n", torqueLimit);
  Serial.printf("EEPROM LOCK       : %d\n", lock);
  Serial.printf("PRESENT POSITION  : %d\n", position);
  Serial.printf("PRESENT SPEED     : %d\n", speed);
  Serial.printf("PRESENT LOAD      : %d\n", load);
  Serial.printf("VOLTAGE RAW       : %d\n", voltage);
  Serial.printf("TEMPERATURE       : %d\n", temperature);
  Serial.printf("MOVING            : %d\n", moving);
  Serial.printf("CURRENT RAW       : %d\n", current);

  Serial.println();
  Serial.println("READ_COMPLETE");
  Serial.println("NO WRITES PERFORMED");
  Serial.println("NO MOTION COMMANDS SENT");
}

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(1500);

  ServoUART.begin(
    SERVO_BAUD,
    SERIAL_8N1,
    SERVO_RX_PIN,
    SERVO_TX_PIN
  );

  st.pSerial = &ServoUART;

  Serial.println();
  Serial.println("====================================");
  Serial.println(" MATDOG SERVO COMMISSIONING");
  Serial.println("====================================");
  Serial.printf("Servo UART      : %lu baud\n", SERVO_BAUD);
  Serial.printf("TX              : GPIO%d\n", SERVO_TX_PIN);
  Serial.printf("RX              : GPIO%d\n", SERVO_RX_PIN);

  Serial.println();
  Serial.println("Startup motion    : DISABLED");
  Serial.println("Manual QC motion  : AUTHORIZATION REQUIRED");
  Serial.println("EEPROM writes   : DISABLED");
  Serial.println("Automatic ping  : DISABLED");
  Serial.println();

  Serial.println("COMMISSIONING_READY");

  printHelp();
}

bool parseIDCommand(const String &cmd, const char *verb, int &id) {
  String prefix = "@";
  prefix += verb;
  prefix += " ";

  if (!cmd.startsWith(prefix)) {
    return false;
  }

  String arg = cmd.substring(prefix.length());
  arg.trim();

  if (arg.length() < 1 || arg.length() > 3) {
    return false;
  }

  int value = 0;

  for (size_t i = 0; i < arg.length(); ++i) {
    char c = arg.charAt(i);

    if (c < '0' || c > '9') {
      return false;
    }

    value = value * 10 + (c - '0');
  }

  if (!validID(value)) {
    return false;
  }

  id = value;
  return true;
}


bool parseUnsignedStrict(const String &arg, int minValue, int maxValue, int &value) {
  if (arg.length() == 0) {
    return false;
  }

  long result = 0;

  for (size_t i = 0; i < arg.length(); ++i) {
    char c = arg.charAt(i);

    if (c < '0' || c > '9') {
      return false;
    }

    result = result * 10 + (c - '0');

    if (result > maxValue) {
      return false;
    }
  }

  if (result < minValue || result > maxValue) {
    return false;
  }

  value = (int)result;
  return true;
}

bool parseBenchCommand(const String &cmd, int &id, int &durationMs) {
  const String prefix = "@BENCH ";

  if (!cmd.startsWith(prefix)) {
    return false;
  }

  String args = cmd.substring(prefix.length());
  args.trim();

  int separator = args.indexOf(' ');

  if (separator <= 0) {
    return false;
  }

  // Exactly two arguments: ID and duration.
  if (args.indexOf(' ', separator + 1) >= 0) {
    return false;
  }

  String idArg = args.substring(0, separator);
  String durationArg = args.substring(separator + 1);

  int parsedID = -1;
  int parsedDuration = -1;

  if (!parseUnsignedStrict(idArg, 0, 253, parsedID)) {
    return false;
  }

  // Read-only benchmark is intentionally bounded.
  if (!parseUnsignedStrict(durationArg, 100, 10000, parsedDuration)) {
    return false;
  }

  id = parsedID;
  durationMs = parsedDuration;
  return true;
}

void benchmarkServo(int id, int durationMs) {
  Serial.println();
  Serial.println("====================================");
  Serial.printf(" READ-ONLY FEEDBACK BENCH ID %d\n", id);
  Serial.println("====================================");
  Serial.printf("REQUESTED_DURATION_MS : %d\n", durationMs);

  int ping = st.Ping((uint8_t)id);

  if (ping < 0) {
    Serial.println("PING               : TIMEOUT");
    Serial.println("BENCH_ABORTED");
    return;
  }

  Serial.printf("PING               : OK ID=%d STATUS=0x%02X\n",
                ping, st.Error);

  int torque = st.readByte(id, SMS_STS_TORQUE_ENABLE);

  if (torque < 0) {
    Serial.println("TORQUE_PRECHECK    : READ_ERROR");
    Serial.println("BENCH_ABORTED");
    return;
  }

  Serial.printf("TORQUE_PRECHECK    : %d\n", torque);

  if (torque != 0) {
    Serial.println("BENCH_ABORTED      : TORQUE_NOT_OFF");
    return;
  }

  uint32_t success = 0;
  uint32_t failures = 0;
  uint32_t statusErrorSamples = 0;
  uint8_t statusOr = 0;

  uint32_t minDt = 0;
  uint32_t maxDt = 0;
  uint64_t sumDt = 0;

  bool haveSample = false;

  int minPos = 0, maxPos = 0;
  int minSpeed = 0, maxSpeed = 0;
  int minLoad = 0, maxLoad = 0;
  int minVoltage = 0, maxVoltage = 0;
  int minTemp = 0, maxTemp = 0;
  int minCurrent = 0, maxCurrent = 0;
  int movingSeen = 0;

  const uint32_t requestedUs = (uint32_t)durationMs * 1000UL;
  const uint32_t benchStart = micros();

  while ((uint32_t)(micros() - benchStart) < requestedUs) {
    const uint32_t sampleStart = micros();

    int n = st.FeedBack(id);

    const uint32_t sampleEnd = micros();
    const uint32_t dt = (uint32_t)(sampleEnd - sampleStart);

    if (n < 0) {
      failures++;
      continue;
    }

    const uint8_t sampleStatus = st.Error;

    if (sampleStatus != 0) {
      statusErrorSamples++;
      statusOr |= sampleStatus;
    }

    int pos = st.ReadPos(-1);
    int speed = st.ReadSpeed(-1);
    int load = st.ReadLoad(-1);
    int voltage = st.ReadVoltage(-1);
    int temperature = st.ReadTemper(-1);
    int moving = st.ReadMove(-1);
    int current = st.ReadCurrent(-1);

    if (!haveSample) {
      minDt = maxDt = dt;

      minPos = maxPos = pos;
      minSpeed = maxSpeed = speed;
      minLoad = maxLoad = load;
      minVoltage = maxVoltage = voltage;
      minTemp = maxTemp = temperature;
      minCurrent = maxCurrent = current;

      haveSample = true;
    } else {
      if (dt < minDt) minDt = dt;
      if (dt > maxDt) maxDt = dt;

      if (pos < minPos) minPos = pos;
      if (pos > maxPos) maxPos = pos;

      if (speed < minSpeed) minSpeed = speed;
      if (speed > maxSpeed) maxSpeed = speed;

      if (load < minLoad) minLoad = load;
      if (load > maxLoad) maxLoad = load;

      if (voltage < minVoltage) minVoltage = voltage;
      if (voltage > maxVoltage) maxVoltage = voltage;

      if (temperature < minTemp) minTemp = temperature;
      if (temperature > maxTemp) maxTemp = temperature;

      if (current < minCurrent) minCurrent = current;
      if (current > maxCurrent) maxCurrent = current;
    }

    if (moving != 0) {
      movingSeen++;
    }

    sumDt += dt;
    success++;
  }

  const uint32_t elapsedUs = (uint32_t)(micros() - benchStart);

  Serial.println();
  Serial.println("--- TRANSPORT PERFORMANCE ---");
  Serial.printf("ELAPSED_US          : %lu\n", (unsigned long)elapsedUs);
  Serial.printf("SUCCESS_SAMPLES     : %lu\n", (unsigned long)success);
  Serial.printf("FEEDBACK_FAILURES   : %lu\n", (unsigned long)failures);
  Serial.printf("STATUS_ERROR_SAMPLES: %lu\n", (unsigned long)statusErrorSamples);
  Serial.printf("STATUS_OR           : 0x%02X\n", statusOr);

  if (success > 0 && elapsedUs > 0) {
    float hz = ((float)success * 1000000.0f) / (float)elapsedUs;
    float meanDt = (float)sumDt / (float)success;

    Serial.printf("EFFECTIVE_HZ        : %.2f\n", hz);
    Serial.printf("FEEDBACK_DT_MIN_US  : %lu\n", (unsigned long)minDt);
    Serial.printf("FEEDBACK_DT_MEAN_US : %.2f\n", meanDt);
    Serial.printf("FEEDBACK_DT_MAX_US  : %lu\n", (unsigned long)maxDt);
  }

  Serial.println();
  Serial.println("--- OBSERVED TELEMETRY RANGE ---");

  if (haveSample) {
    Serial.printf("POSITION            : %d .. %d\n", minPos, maxPos);
    Serial.printf("SPEED               : %d .. %d\n", minSpeed, maxSpeed);
    Serial.printf("LOAD                : %d .. %d\n", minLoad, maxLoad);
    Serial.printf("VOLTAGE RAW         : %d .. %d\n", minVoltage, maxVoltage);
    Serial.printf("TEMPERATURE         : %d .. %d\n", minTemp, maxTemp);
    Serial.printf("CURRENT RAW         : %d .. %d\n", minCurrent, maxCurrent);
    Serial.printf("MOVING NONZERO      : %d samples\n", movingSeen);
  }

  Serial.println();
  Serial.println("BENCH_COMPLETE");
  Serial.println("NO WRITES PERFORMED");
  Serial.println("NO MOTION COMMANDS SENT");
}


bool parseCaptureCommand(const String &cmd, int &id, int &durationMs, int &hz) {
  const String prefix = "@CAPTURE ";

  if (!cmd.startsWith(prefix)) {
    return false;
  }

  String args = cmd.substring(prefix.length());
  args.trim();

  int s1 = args.indexOf(' ');
  if (s1 <= 0) return false;

  int s2 = args.indexOf(' ', s1 + 1);
  if (s2 <= s1 + 1) return false;

  if (args.indexOf(' ', s2 + 1) >= 0) return false;

  String idArg  = args.substring(0, s1);
  String durArg = args.substring(s1 + 1, s2);
  String hzArg  = args.substring(s2 + 1);

  int parsedID = -1;
  int parsedDuration = -1;
  int parsedHz = -1;

  if (!parseUnsignedStrict(idArg, 0, 253, parsedID)) return false;
  if (!parseUnsignedStrict(durArg, 100, 10000, parsedDuration)) return false;
  if (!parseUnsignedStrict(hzArg, 10, 1000, parsedHz)) return false;

  id = parsedID;
  durationMs = parsedDuration;
  hz = parsedHz;
  return true;
}

void captureFixedRate(int id, int durationMs, int hz) {
  Serial.println();
  Serial.println("====================================");
  Serial.printf(" FIXED-RATE READ-ONLY CAPTURE ID %d\n", id);
  Serial.println("====================================");

  int ping = st.Ping((uint8_t)id);

  if (ping < 0) {
    Serial.println("PING               : TIMEOUT");
    Serial.println("CAPTURE_ABORTED");
    return;
  }

  int torque = st.readByte(id, SMS_STS_TORQUE_ENABLE);

  if (torque < 0) {
    Serial.println("TORQUE_PRECHECK    : READ_ERROR");
    Serial.println("CAPTURE_ABORTED");
    return;
  }

  if (torque != 0) {
    Serial.printf("TORQUE_PRECHECK    : %d\n", torque);
    Serial.println("CAPTURE_ABORTED    : TORQUE_NOT_OFF");
    return;
  }

  const uint32_t periodUs =
      (uint32_t)(1000000UL / (uint32_t)hz);

  const uint32_t targetSamples =
      ((uint32_t)durationMs * (uint32_t)hz) / 1000UL;

  uint32_t success = 0;
  uint32_t failures = 0;
  uint32_t missedDeadlines = 0;
  uint32_t statusErrors = 0;
  uint8_t statusOr = 0;

  uint32_t minInterval = 0;
  uint32_t maxInterval = 0;
  uint64_t sumInterval = 0;

  uint32_t minFeedback = 0;
  uint32_t maxFeedback = 0;
  uint64_t sumFeedback = 0;

  uint32_t previousSampleStart = 0;
  bool havePrevious = false;

  int minPos = 0, maxPos = 0;
  int minSpeed = 0, maxSpeed = 0;
  int minLoad = 0, maxLoad = 0;
  int minVoltage = 0, maxVoltage = 0;
  int minTemp = 0, maxTemp = 0;
  int minCurrent = 0, maxCurrent = 0;
  uint32_t movingSamples = 0;
  bool haveTelemetry = false;

  const uint32_t captureStart = micros();
  uint32_t nextDeadline = captureStart;

  for (uint32_t i = 0; i < targetSamples; ++i) {

    while ((int32_t)(micros() - nextDeadline) < 0) {
      delayMicroseconds(20);
    }

    const uint32_t sampleStart = micros();

    if ((int32_t)(sampleStart - nextDeadline) >= (int32_t)periodUs) {
      missedDeadlines++;
    }

    if (havePrevious) {
      const uint32_t interval =
          (uint32_t)(sampleStart - previousSampleStart);

      if (success <= 1) {
        minInterval = maxInterval = interval;
      } else {
        if (interval < minInterval) minInterval = interval;
        if (interval > maxInterval) maxInterval = interval;
      }

      sumInterval += interval;
    }

    previousSampleStart = sampleStart;
    havePrevious = true;

    const uint32_t feedbackStart = micros();
    int n = st.FeedBack(id);
    const uint32_t feedbackEnd = micros();

    const uint32_t feedbackUs =
        (uint32_t)(feedbackEnd - feedbackStart);

    if (n < 0) {
      failures++;
      nextDeadline += periodUs;
      continue;
    }

    if (success == 0) {
      minFeedback = maxFeedback = feedbackUs;
    } else {
      if (feedbackUs < minFeedback) minFeedback = feedbackUs;
      if (feedbackUs > maxFeedback) maxFeedback = feedbackUs;
    }

    sumFeedback += feedbackUs;

    uint8_t sampleStatus = st.Error;

    if (sampleStatus != 0) {
      statusErrors++;
      statusOr |= sampleStatus;
    }

    int pos         = st.ReadPos(-1);
    int speed       = st.ReadSpeed(-1);
    int load        = st.ReadLoad(-1);
    int voltage     = st.ReadVoltage(-1);
    int temperature = st.ReadTemper(-1);
    int moving      = st.ReadMove(-1);
    int current     = st.ReadCurrent(-1);

    if (!haveTelemetry) {
      minPos = maxPos = pos;
      minSpeed = maxSpeed = speed;
      minLoad = maxLoad = load;
      minVoltage = maxVoltage = voltage;
      minTemp = maxTemp = temperature;
      minCurrent = maxCurrent = current;
      haveTelemetry = true;
    } else {
      if (pos < minPos) minPos = pos;
      if (pos > maxPos) maxPos = pos;

      if (speed < minSpeed) minSpeed = speed;
      if (speed > maxSpeed) maxSpeed = speed;

      if (load < minLoad) minLoad = load;
      if (load > maxLoad) maxLoad = load;

      if (voltage < minVoltage) minVoltage = voltage;
      if (voltage > maxVoltage) maxVoltage = voltage;

      if (temperature < minTemp) minTemp = temperature;
      if (temperature > maxTemp) maxTemp = temperature;

      if (current < minCurrent) minCurrent = current;
      if (current > maxCurrent) maxCurrent = current;
    }

    if (moving != 0) movingSamples++;

    success++;
    nextDeadline += periodUs;
  }

  const uint32_t elapsedUs =
      (uint32_t)(micros() - captureStart);

  Serial.println();
  Serial.println("--- FIXED RATE PERFORMANCE ---");
  Serial.printf("REQUESTED_HZ         : %d\n", hz);
  Serial.printf("PERIOD_US            : %lu\n",
                (unsigned long)periodUs);
  Serial.printf("TARGET_SAMPLES       : %lu\n",
                (unsigned long)targetSamples);
  Serial.printf("SUCCESS_SAMPLES      : %lu\n",
                (unsigned long)success);
  Serial.printf("FEEDBACK_FAILURES    : %lu\n",
                (unsigned long)failures);
  Serial.printf("MISSED_DEADLINES     : %lu\n",
                (unsigned long)missedDeadlines);
  Serial.printf("STATUS_ERROR_SAMPLES : %lu\n",
                (unsigned long)statusErrors);
  Serial.printf("STATUS_OR            : 0x%02X\n", statusOr);
  Serial.printf("ELAPSED_US           : %lu\n",
                (unsigned long)elapsedUs);

  if (success > 1) {
    float meanInterval =
        (float)sumInterval / (float)(success - 1);

    float effectiveHz =
        1000000.0f / meanInterval;

    Serial.printf("EFFECTIVE_HZ         : %.3f\n", effectiveHz);
    Serial.printf("INTERVAL_MIN_US      : %lu\n",
                  (unsigned long)minInterval);
    Serial.printf("INTERVAL_MEAN_US     : %.2f\n", meanInterval);
    Serial.printf("INTERVAL_MAX_US      : %lu\n",
                  (unsigned long)maxInterval);
  }

  if (success > 0) {
    float meanFeedback =
        (float)sumFeedback / (float)success;

    Serial.printf("FEEDBACK_MIN_US      : %lu\n",
                  (unsigned long)minFeedback);
    Serial.printf("FEEDBACK_MEAN_US     : %.2f\n", meanFeedback);
    Serial.printf("FEEDBACK_MAX_US      : %lu\n",
                  (unsigned long)maxFeedback);
  }

  Serial.println();
  Serial.println("--- TELEMETRY RANGE ---");

  if (haveTelemetry) {
    Serial.printf("POSITION             : %d .. %d\n", minPos, maxPos);
    Serial.printf("SPEED                : %d .. %d\n", minSpeed, maxSpeed);
    Serial.printf("LOAD                 : %d .. %d\n", minLoad, maxLoad);
    Serial.printf("VOLTAGE RAW          : %d .. %d\n",
                  minVoltage, maxVoltage);
    Serial.printf("TEMPERATURE          : %d .. %d\n",
                  minTemp, maxTemp);
    Serial.printf("CURRENT RAW          : %d .. %d\n",
                  minCurrent, maxCurrent);
    Serial.printf("MOVING NONZERO       : %lu\n",
                  (unsigned long)movingSamples);
  }

  Serial.println();
  Serial.println("CAPTURE_COMPLETE");
  Serial.println("NO WRITES PERFORMED");
  Serial.println("NO MOTION COMMANDS SENT");
}




void initMotionStats(MotionStats &m, int startPos, int targetPos) {
  m.samples = 0;
  m.feedbackFailures = 0;
  m.missedDeadlines = 0;
  m.movingSamples = 0;

  m.startPos = startPos;
  m.targetPos = targetPos;
  m.finalPos = startPos;

  m.minPos = m.maxPos = startPos;
  m.minSpeed = m.maxSpeed = 0;
  m.minLoad = m.maxLoad = 0;
  m.minVoltage = m.maxVoltage = 0;
  m.minTemp = m.maxTemp = 0;
  m.minCurrent = m.maxCurrent = 0;

  m.firstMotionUs = 0;
  m.settledUs = 0;

  m.maxAbsSpeedAccel = 0;
}

bool runMotionLeg(
    int id,
    int target,
    int expectedDirection,
    MotionStats &m,
    String &abortReason) {

  const int SPEED_CMD = 100;
  const int ACC_CMD = 10;

  const uint32_t PERIOD_US = 2000;       // 500 Hz
  const uint32_t TIMEOUT_US = 2500000;   // 2.5 s
  const uint32_t FIRST_MOTION_LIMIT_US = 1000000;

  const int SETTLE_ERROR_TICKS = 2;
  const int SETTLE_SPEED = 5;
  const int SETTLE_SAMPLES = 25;         // 50 ms
  const int WRONG_DIR_TOL = 3;
  const int OVERSHOOT_LIMIT = 12;

  int n = st.FeedBack(id);

  if (n < 0) {
    abortReason = "INITIAL_FEEDBACK_FAILED";
    return false;
  }

  if (st.Error != 0) {
    abortReason = "INITIAL_STATUS_ERROR";
    return false;
  }

  int phaseStartPos = st.ReadPos(-1);

  initMotionStats(m, phaseStartPos, target);

  int writeResult =
      st.WritePosEx(
        (uint8_t)id,
        (s16)target,
        (u16)SPEED_CMD,
        (u8)ACC_CMD
      );

  if (writeResult < 0) {
    abortReason = "GOAL_WRITE_FAILED";
    return false;
  }

  const uint32_t phaseStart = micros();
  uint32_t nextDeadline = phaseStart;

  int stableCount = 0;
  int consecutiveFeedbackFailures = 0;

  bool havePreviousSpeed = false;
  int previousSpeed = 0;
  uint32_t previousSpeedTime = 0;

  bool haveTelemetry = false;

  while ((uint32_t)(micros() - phaseStart) < TIMEOUT_US) {

    while ((int32_t)(micros() - nextDeadline) < 0) {
      delayMicroseconds(20);
    }

    const uint32_t sampleTime = micros();

    if ((int32_t)(sampleTime - nextDeadline) >=
        (int32_t)PERIOD_US) {
      m.missedDeadlines++;
    }

    nextDeadline += PERIOD_US;

    n = st.FeedBack(id);

    if (n < 0) {
      m.feedbackFailures++;
      consecutiveFeedbackFailures++;

      if (consecutiveFeedbackFailures >= 3) {
        abortReason = "3_CONSECUTIVE_FEEDBACK_FAILURES";
        return false;
      }

      continue;
    }

    consecutiveFeedbackFailures = 0;

    if (st.Error != 0) {
      abortReason = "SERVO_STATUS_ERROR";
      return false;
    }

    int pos         = st.ReadPos(-1);
    int speed       = st.ReadSpeed(-1);
    int load        = st.ReadLoad(-1);
    int voltage     = st.ReadVoltage(-1);
    int temperature = st.ReadTemper(-1);
    int moving      = st.ReadMove(-1);
    int current     = st.ReadCurrent(-1);

    if (pos < 0 || pos > 4095) {
      abortReason = "POSITION_OUT_OF_DOMAIN";
      return false;
    }

    const uint32_t elapsed =
        (uint32_t)(sampleTime - phaseStart);

    if (!haveTelemetry) {
      m.minPos = m.maxPos = pos;
      m.minSpeed = m.maxSpeed = speed;
      m.minLoad = m.maxLoad = load;
      m.minVoltage = m.maxVoltage = voltage;
      m.minTemp = m.maxTemp = temperature;
      m.minCurrent = m.maxCurrent = current;
      haveTelemetry = true;
    } else {
      if (pos < m.minPos) m.minPos = pos;
      if (pos > m.maxPos) m.maxPos = pos;

      if (speed < m.minSpeed) m.minSpeed = speed;
      if (speed > m.maxSpeed) m.maxSpeed = speed;

      if (load < m.minLoad) m.minLoad = load;
      if (load > m.maxLoad) m.maxLoad = load;

      if (voltage < m.minVoltage) m.minVoltage = voltage;
      if (voltage > m.maxVoltage) m.maxVoltage = voltage;

      if (temperature < m.minTemp) m.minTemp = temperature;
      if (temperature > m.maxTemp) m.maxTemp = temperature;

      if (current < m.minCurrent) m.minCurrent = current;
      if (current > m.maxCurrent) m.maxCurrent = current;
    }

    if (moving != 0) {
      m.movingSamples++;
    }

    m.samples++;
    m.finalPos = pos;

    if (m.firstMotionUs == 0) {
      if (abs(pos - phaseStartPos) >= 2 || abs(speed) > 2) {
        m.firstMotionUs = elapsed;
      }
    }

    if (m.firstMotionUs == 0 &&
        elapsed > FIRST_MOTION_LIMIT_US) {

      abortReason = "NO_MOTION_WITHIN_1S";
      return false;
    }

    if (expectedDirection < 0 &&
        pos > phaseStartPos + WRONG_DIR_TOL) {

      abortReason = "WRONG_DIRECTION";
      return false;
    }

    if (expectedDirection > 0 &&
        pos < phaseStartPos - WRONG_DIR_TOL) {

      abortReason = "WRONG_DIRECTION";
      return false;
    }

    if (expectedDirection < 0 &&
        pos < target - OVERSHOOT_LIMIT) {

      abortReason = "EXCESSIVE_OVERSHOOT";
      return false;
    }

    if (expectedDirection > 0 &&
        pos > target + OVERSHOOT_LIMIT) {

      abortReason = "EXCESSIVE_OVERSHOOT";
      return false;
    }

    if (havePreviousSpeed) {
      uint32_t dt =
          (uint32_t)(sampleTime - previousSpeedTime);

      if (dt > 0) {
        int64_t dv =
            (int64_t)speed - (int64_t)previousSpeed;

        int64_t accel =
            (dv * 1000000LL) / (int64_t)dt;

        if (accel < 0) accel = -accel;

        if (accel > m.maxAbsSpeedAccel) {
          m.maxAbsSpeedAccel = accel;
        }
      }
    }

    previousSpeed = speed;
    previousSpeedTime = sampleTime;
    havePreviousSpeed = true;

    if (abs(pos - target) <= SETTLE_ERROR_TICKS &&
        abs(speed) <= SETTLE_SPEED &&
        moving == 0) {

      stableCount++;

      if (stableCount >= SETTLE_SAMPLES) {
        m.settledUs = elapsed;
        m.finalPos = pos;
        return true;
      }

    } else {
      stableCount = 0;
    }
  }

  abortReason = "PHASE_TIMEOUT";
  return false;
}

void printMotionStats(const char *name, const MotionStats &m) {
  Serial.println();
  Serial.printf("--- %s ---\n", name);

  Serial.printf("START_POSITION       : %d\n", m.startPos);
  Serial.printf("TARGET_POSITION      : %d\n", m.targetPos);
  Serial.printf("FINAL_POSITION       : %d\n", m.finalPos);
  Serial.printf("FINAL_ERROR_TICKS    : %d\n",
                m.finalPos - m.targetPos);

  Serial.printf("SAMPLES              : %lu\n",
                (unsigned long)m.samples);
  Serial.printf("FEEDBACK_FAILURES    : %lu\n",
                (unsigned long)m.feedbackFailures);
  Serial.printf("MISSED_DEADLINES     : %lu\n",
                (unsigned long)m.missedDeadlines);

  Serial.printf("FIRST_MOTION_US      : %lu\n",
                (unsigned long)m.firstMotionUs);
  Serial.printf("SETTLED_US           : %lu\n",
                (unsigned long)m.settledUs);

  Serial.printf("POSITION_RANGE       : %d .. %d\n",
                m.minPos, m.maxPos);
  Serial.printf("SPEED_RANGE          : %d .. %d\n",
                m.minSpeed, m.maxSpeed);
  Serial.printf("LOAD_RANGE           : %d .. %d\n",
                m.minLoad, m.maxLoad);
  Serial.printf("VOLTAGE_RAW_RANGE    : %d .. %d\n",
                m.minVoltage, m.maxVoltage);
  Serial.printf("TEMPERATURE_RANGE    : %d .. %d\n",
                m.minTemp, m.maxTemp);
  Serial.printf("CURRENT_RAW_RANGE    : %d .. %d\n",
                m.minCurrent, m.maxCurrent);

  Serial.printf("MOVING_SAMPLES       : %lu\n",
                (unsigned long)m.movingSamples);

  Serial.printf("MAX_ABS_SPEED_ACCEL  : %lld\n",
                (long long)m.maxAbsSpeedAccel);
}


bool forceTorqueOffVerified(int id, const char *context) {
  Serial.printf("%s TORQUE_OFF     : COMMAND\n", context);

  int wr = st.EnableTorque((uint8_t)id, 0);

  delay(20);

  int rb = st.readByte(id, SMS_STS_TORQUE_ENABLE);

  Serial.printf("%s TORQUE_READBACK: %d\n", context, rb);

  if (wr < 0 || rb != 0) {
    Serial.printf("%s TORQUE_OFF     : FAILED\n", context);
    return false;
  }

  Serial.printf("%s TORQUE_OFF     : CONFIRMED\n", context);
  return true;
}

void runSafeOff(int id) {
  Serial.println();
  Serial.println("====================================");
  Serial.printf(" SAFE TORQUE OFF — ID %d\n", id);
  Serial.println("====================================");

  int ping = st.Ping((uint8_t)id);

  if (ping < 0) {
    Serial.println("SAFE_OFF_ABORT: PING_TIMEOUT");
    return;
  }

  bool ok = forceTorqueOffVerified(id, "SAFE");

  if (ok) {
    Serial.println("SAFE_OFF_RESULT    : PASS");
  } else {
    Serial.println("SAFE_OFF_RESULT    : FAIL");
    Serial.println("CUT_SERVO_POWER_NOW");
  }
}

void runAuthorizedMicro32() {
  const int ID = 32;

  const int EXPECTED_START = 3932;
  const int OUT_TARGET = 3868;
  const int RETURN_TARGET = 3932;

  const int SPEED_CMD = 100;
  const int ACC_CMD = 10;

  Serial.println();
  Serial.println("====================================");
  Serial.println(" AUTHORIZED MICRO MOTION TEST — ID 32");
  Serial.println("====================================");
  Serial.println("PATH               : 3932 -> 3868 -> 3932");
  Serial.println("DELTA              : 64 ticks");
  Serial.println("GOAL SPEED         : 100");
  Serial.println("ACC                : 10");
  Serial.println("TELEMETRY          : 500 Hz");
  Serial.println("EEPROM WRITES      : NONE");
  Serial.println();

  bool torqueTouched = false;
  bool testPassed = false;
  String abortReason = "";

  MotionStats outbound;
  MotionStats inbound;

  int ping = st.Ping((uint8_t)ID);

  if (ping < 0) {
    Serial.println("ABORT: PING_TIMEOUT");
    return;
  }

  if (st.Error != 0) {
    Serial.printf("ABORT: STATUS=0x%02X\n", st.Error);
    return;
  }

  int mode = st.readByte(ID, SMS_STS_MODE);
  int torque = st.readByte(ID, SMS_STS_TORQUE_ENABLE);

  if (mode < 0 || torque < 0) {
    Serial.println("ABORT: PRECHECK_READ_FAILED");
    return;
  }

  if (mode != 0) {
    Serial.printf("ABORT: MODE_IS_%d_NOT_0\n", mode);
    return;
  }

  if (torque != 0) {
    Serial.printf("ABORT: TORQUE_ALREADY_%d\n", torque);
    return;
  }

  if (st.FeedBack(ID) < 0) {
    Serial.println("ABORT: FEEDBACK_PRECHECK_FAILED");
    return;
  }

  if (st.Error != 0) {
    Serial.printf("ABORT: FEEDBACK_STATUS=0x%02X\n", st.Error);
    return;
  }

  int startPos = st.ReadPos(-1);
  int startSpeed = st.ReadSpeed(-1);
  int startLoad = st.ReadLoad(-1);
  int startVoltage = st.ReadVoltage(-1);
  int startTemp = st.ReadTemper(-1);
  int startCurrent = st.ReadCurrent(-1);

  Serial.printf("PRECHECK_POSITION  : %d\n", startPos);
  Serial.printf("PRECHECK_SPEED     : %d\n", startSpeed);
  Serial.printf("PRECHECK_LOAD      : %d\n", startLoad);
  Serial.printf("PRECHECK_VOLTAGE   : %d\n", startVoltage);
  Serial.printf("PRECHECK_TEMP      : %d\n", startTemp);
  Serial.printf("PRECHECK_CURRENT   : %d\n", startCurrent);
  Serial.printf("PRECHECK_STATUS    : 0x%02X\n", st.Error);
  Serial.printf("PRECHECK_TORQUE    : %d\n", torque);
  Serial.printf("PRECHECK_MODE      : %d\n", mode);

  /*
   * Hard gate for this one specifically authorized path.
   * Any different starting position means NO MOTION.
   */
  if (startPos != EXPECTED_START) {
    Serial.printf(
      "ABORT: START_POSITION_EXPECTED_%d_GOT_%d\n",
      EXPECTED_START,
      startPos
    );
    return;
  }

  /*
   * PRIME GOAL = current position while torque is still OFF.
   */
  int primeResult =
      st.WritePosEx(
        (uint8_t)ID,
        (s16)startPos,
        (u16)SPEED_CMD,
        (u8)ACC_CMD
      );

  delay(20);

  int torqueAfterPrime =
      st.readByte(ID, SMS_STS_TORQUE_ENABLE);

  Serial.printf("PRIME_WRITE_RESULT : %d\n", primeResult);
  Serial.printf("PRIME_TORQUE_AFTER : %d\n", torqueAfterPrime);

  /*
   * Observed on real ST-3215-C018:
   * a position-profile write can leave Torque Enable = 1.
   *
   * Therefore every PRIME is followed unconditionally by an
   * explicit Torque OFF + verified readback before continuing.
   */
  if (!forceTorqueOffVerified(ID, "PRIME")) {
    Serial.println("ABORT: PRIME_SAFE_OFF_FAILED");
    Serial.println("CUT_SERVO_POWER_NOW");
    return;
  }

  if (primeResult < 0) {
    Serial.println("ABORT: PRIME_WRITE_ACK_FAILED");
    return;
  }

  Serial.println("PRIME              : SAFE");
  Serial.println("TORQUE_ON          : COMMAND");

  /*
   * From this point forward cleanup MUST attempt torque OFF.
   */
  torqueTouched = true;

  int torqueOnResult =
      st.EnableTorque((uint8_t)ID, 1);

  if (torqueOnResult < 0) {
    abortReason = "TORQUE_ON_WRITE_FAILED";
  } else {

    delay(20);

    int torqueReadback =
        st.readByte(ID, SMS_STS_TORQUE_ENABLE);

    if (torqueReadback != 1) {
      abortReason = "TORQUE_ON_NOT_CONFIRMED";
    } else {

      Serial.println("TORQUE_ON          : CONFIRMED");
      Serial.println("OUTBOUND           : START");

      if (!runMotionLeg(
            ID,
            OUT_TARGET,
            -1,
            outbound,
            abortReason)) {

        // abortReason already set

      } else {

        Serial.println("OUTBOUND           : SETTLED");
        Serial.println("RETURN             : START");

        if (!runMotionLeg(
              ID,
              RETURN_TARGET,
              +1,
              inbound,
              abortReason)) {

          // abortReason already set

        } else {
          Serial.println("RETURN             : SETTLED");
          testPassed = true;
        }
      }
    }
  }

  /*
   * Mandatory safe cleanup after any torque-on attempt.
   */
  Serial.println("TORQUE_OFF         : COMMAND");

  int torqueOffResult =
      st.EnableTorque((uint8_t)ID, 0);

  delay(20);

  int torqueFinal =
      st.readByte(ID, SMS_STS_TORQUE_ENABLE);

  if (torqueOffResult < 0 || torqueFinal != 0) {
    Serial.println("CRITICAL: TORQUE_OFF_UNCONFIRMED");
    Serial.printf("TORQUE_FINAL_READ  : %d\n", torqueFinal);
    Serial.println("CUT_SERVO_POWER_NOW");
  } else {
    Serial.println("TORQUE_OFF         : CONFIRMED");
  }

  if (outbound.samples > 0) {
    printMotionStats("OUTBOUND 3932 -> 3868", outbound);
  }

  if (inbound.samples > 0) {
    printMotionStats("RETURN 3868 -> 3932", inbound);
  }

  Serial.println();

  if (testPassed && torqueFinal == 0) {
    Serial.println("MICRO32_RESULT     : PASS");
  } else {
    Serial.printf("MICRO32_RESULT     : ABORT/FAIL (%s)\n",
                  abortReason.c_str());
  }

  Serial.printf("FINAL_TORQUE       : %d\n", torqueFinal);
  Serial.println("EEPROM_WRITES      : NONE");
  Serial.println("MICRO32_COMPLETE");
}



void runNormalizeMatdog(int ID) {

  // ================================================================
  // ELROBOT -> MATDOG CONTROL-PROFILE NORMALIZER
  //
  // INTENTIONALLY PRESERVED:
  //   ID
  //   BaudRate
  //   Min/MaxAngleLimit
  //   CW/CCW deadband
  //   PositionOffset
  //   Mode
  //   I coefficient
  //   MinStartupForce
  //   ProtectionTorque
  //   ProtectionTime
  //   SpeedClosedLoopP
  //   OverCurrentProtectionTime
  //   VelocityClosedLoopI
  //
  // EEPROM WRITES:
  //   0x15 PCoef              16 -> 32
  //   0x16 DCoef               0 -> 32
  //   0x1C ProtectionCurrent 500 -> 310
  //   0x24 OverloadTorque     25 -> 80
  //   0x10 MaxTorque         500 -> 1000   [LAST EEPROM WRITE]
  //
  // RAM WRITE:
  //   0x30 TorqueLimit       500 -> 1000   [FINAL WRITE]
  //
  // NO POSITION COMMANDS.
  // NO OFFSET WRITES.
  // ================================================================

  const uint8_t REG_MAX_TORQUE_L       = 0x10;
  const uint8_t REG_P_COEF             = 0x15;
  const uint8_t REG_D_COEF             = 0x16;
  const uint8_t REG_I_COEF             = 0x17;
  const uint8_t REG_MIN_START_FORCE_L  = 0x18;
  const uint8_t REG_PROT_CURRENT_L     = 0x1C;
  const uint8_t REG_PROT_TORQUE        = 0x22;
  const uint8_t REG_PROT_TIME          = 0x23;
  const uint8_t REG_OVERLOAD_TORQUE    = 0x24;
  const uint8_t REG_SPEED_LOOP_P       = 0x25;
  const uint8_t REG_OVERCURR_TIME      = 0x26;
  const uint8_t REG_VELOCITY_LOOP_I    = 0x27;

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" MATDOG PROFILE NORMALIZER — ID %d\n", ID);
  Serial.println("====================================");
  Serial.println("NO MOTION COMMANDS WILL BE SENT");

  // ---------------------------------------------------------------
  // PING
  // ---------------------------------------------------------------

  int ping = st.Ping((uint8_t)ID);

  if (ping < 0 || st.Error != 0) {
    Serial.println("NORMALIZE_ABORTED   : PING_OR_STATUS");
    Serial.println("EEPROM_WRITES       : NONE");
    return;
  }

  // ---------------------------------------------------------------
  // Force/verify torque OFF before doing anything else.
  // This is a safety write only.
  // ---------------------------------------------------------------

  int torque = st.readByte(ID, SMS_STS_TORQUE_ENABLE);

  if (torque < 0) {
    Serial.println("NORMALIZE_ABORTED   : TORQUE_READ_FAILED");
    Serial.println("EEPROM_WRITES       : NONE");
    return;
  }

  if (torque != 0) {
    Serial.printf("TORQUE_PRE          : %d\n", torque);
    Serial.println("FORCING_TORQUE_OFF");

    st.EnableTorque(ID, 0);
    delay(20);

    torque = st.readByte(ID, SMS_STS_TORQUE_ENABLE);

    if (torque != 0) {
      Serial.println("NORMALIZE_ABORTED   : TORQUE_OFF_FAILED");
      Serial.println("EEPROM_WRITES       : NONE");
      return;
    }
  }

  Serial.println("TORQUE_OFF          : CONFIRMED");

  // ---------------------------------------------------------------
  // READ COMPLETE AS-FOUND SIGNATURE
  // ---------------------------------------------------------------

  int model            = st.readWord(ID, SMS_STS_MODEL_L);
  int storedID         = st.readByte(ID, SMS_STS_ID);
  int baudReg          = st.readByte(ID, SMS_STS_BAUD_RATE);

  int minLimit         = st.readWord(ID, SMS_STS_MIN_ANGLE_LIMIT_L);
  int maxLimit         = st.readWord(ID, SMS_STS_MAX_ANGLE_LIMIT_L);

  int cwDead           = st.readByte(ID, SMS_STS_CW_DEAD);
  int ccwDead          = st.readByte(ID, SMS_STS_CCW_DEAD);

  int offsetBefore     = st.readWord(ID, SMS_STS_OFS_L);
  int mode             = st.readByte(ID, SMS_STS_MODE);

  int maxTorque        = st.readWord(ID, REG_MAX_TORQUE_L);
  int pCoef            = st.readByte(ID, REG_P_COEF);
  int dCoef            = st.readByte(ID, REG_D_COEF);
  int iCoef            = st.readByte(ID, REG_I_COEF);
  int minStart         = st.readWord(ID, REG_MIN_START_FORCE_L);

  int protCurrent      = st.readWord(ID, REG_PROT_CURRENT_L);
  int protTorque       = st.readByte(ID, REG_PROT_TORQUE);
  int protTime         = st.readByte(ID, REG_PROT_TIME);
  int overloadTorque   = st.readByte(ID, REG_OVERLOAD_TORQUE);

  int speedLoopP       = st.readByte(ID, REG_SPEED_LOOP_P);
  int overCurrentTime  = st.readByte(ID, REG_OVERCURR_TIME);
  int velocityLoopI    = st.readByte(ID, REG_VELOCITY_LOOP_I);

  int torqueLimit      = st.readWord(ID, SMS_STS_TORQUE_LIMIT_L);
  int lockBefore       = st.readByte(ID, SMS_STS_LOCK);

  bool readFailed =
      model < 0 ||
      storedID < 0 ||
      baudReg < 0 ||
      minLimit < 0 ||
      maxLimit < 0 ||
      cwDead < 0 ||
      ccwDead < 0 ||
      offsetBefore < 0 ||
      mode < 0 ||
      maxTorque < 0 ||
      pCoef < 0 ||
      dCoef < 0 ||
      iCoef < 0 ||
      minStart < 0 ||
      protCurrent < 0 ||
      protTorque < 0 ||
      protTime < 0 ||
      overloadTorque < 0 ||
      speedLoopP < 0 ||
      overCurrentTime < 0 ||
      velocityLoopI < 0 ||
      torqueLimit < 0 ||
      lockBefore < 0;

  if (readFailed) {
    Serial.println("NORMALIZE_ABORTED   : PRE_READ_FAILED");
    Serial.println("EEPROM_WRITES       : NONE");
    st.EnableTorque(ID, 0);
    return;
  }

  Serial.println();
  Serial.println("--- AS-FOUND ---");

  Serial.printf("MODEL              : %d\n", model);
  Serial.printf("ID                 : %d\n", storedID);
  Serial.printf("BAUD               : %d\n", baudReg);
  Serial.printf("MIN/MAX LIMIT      : %d / %d\n", minLimit, maxLimit);
  Serial.printf("CW/CCW DEAD        : %d / %d\n", cwDead, ccwDead);
  Serial.printf("OFFSET             : %d (0x%04X)\n",
                offsetBefore, offsetBefore);
  Serial.printf("MODE               : %d\n", mode);

  Serial.printf("MAX TORQUE         : %d\n", maxTorque);
  Serial.printf("P / I / D          : %d / %d / %d\n",
                pCoef, iCoef, dCoef);
  Serial.printf("MIN START FORCE    : %d\n", minStart);
  Serial.printf("PROT CURRENT       : %d\n", protCurrent);
  Serial.printf("PROT TORQUE/TIME   : %d / %d\n",
                protTorque, protTime);
  Serial.printf("OVERLOAD TORQUE    : %d\n", overloadTorque);
  Serial.printf("SPEED LOOP P       : %d\n", speedLoopP);
  Serial.printf("OVERCURR TIME      : %d\n", overCurrentTime);
  Serial.printf("VELOCITY LOOP I    : %d\n", velocityLoopI);

  Serial.printf("TORQUE LIMIT       : %d\n", torqueLimit);
  Serial.printf("EEPROM LOCK        : %d\n", lockBefore);

  // ---------------------------------------------------------------
  // EXACT ELROBOT SIGNATURE GATE
  //
  // PositionOffset is deliberately NOT constrained because it is
  // joint-specific. It is captured and must be identical afterward.
  // ---------------------------------------------------------------

  bool immutableSignature =
      model == 777 &&
      storedID == ID &&
      baudReg == 0 &&
      minLimit == 0 &&
      maxLimit == 4095 &&
      cwDead == 1 &&
      ccwDead == 1 &&
      mode == 0 &&
      iCoef == 0 &&
      minStart == 16 &&
      protTorque == 20 &&
      protTime == 200 &&
      speedLoopP == 10 &&
      overCurrentTime == 200 &&
      velocityLoopI == 200 &&
      lockBefore == 1;

  bool changedValuesKnown =
      (maxTorque == 500 || maxTorque == 1000) &&
      (pCoef == 16 || pCoef == 32) &&
      (dCoef == 0 || dCoef == 32) &&
      (protCurrent == 500 || protCurrent == 310) &&
      (overloadTorque == 25 || overloadTorque == 80) &&
      (torqueLimit == 500 || torqueLimit == 1000);

  bool sourceExact =
      maxTorque == 500 &&
      pCoef == 16 &&
      dCoef == 0 &&
      protCurrent == 500 &&
      overloadTorque == 25 &&
      torqueLimit == 500;

  bool targetExact =
      maxTorque == 1000 &&
      pCoef == 32 &&
      dCoef == 32 &&
      protCurrent == 310 &&
      overloadTorque == 80 &&
      torqueLimit == 1000;

  if (!immutableSignature || !changedValuesKnown) {
    Serial.println();
    Serial.println("NORMALIZE_ABORTED   : UNKNOWN_OR_INVALID_PROFILE");
    Serial.println("EEPROM_WRITES       : NONE");
    st.EnableTorque(ID, 0);
    return;
  }

  Serial.println();
  if (sourceExact) {
    Serial.println("PROFILE_STATE       : ELROBOT_SOURCE_EXACT");
  } else if (targetExact) {
    Serial.println("PROFILE_STATE       : MATDOG_ALREADY_NORMALIZED");
    Serial.println("NORMALIZE_RESULT    : PASS");
    Serial.println("NO WRITES REQUIRED");
    st.EnableTorque(ID, 0);
    return;
  } else {
    Serial.println("PROFILE_STATE       : KNOWN_PARTIAL_STATE_RESUME");
  }

  Serial.println("PRECHECK_RESULT     : PASS");

  // ---------------------------------------------------------------
  // EEPROM UNLOCK
  // ---------------------------------------------------------------

  int unlockAck = st.unLockEprom(ID);
  delay(20);

  int lockState = st.readByte(ID, SMS_STS_LOCK);

  if (unlockAck < 0 || lockState != 0) {
    st.LockEprom(ID);
    st.EnableTorque(ID, 0);

    Serial.println("NORMALIZE_ABORTED   : EEPROM_UNLOCK_FAILED");
    return;
  }

  Serial.println("EEPROM_UNLOCK       : CONFIRMED");

  // ---------------------------------------------------------------
  // EEPROM WRITE 1 — P 16 -> 32
  // ---------------------------------------------------------------

  int wr = 0;
  int rb = pCoef;

  if (pCoef != 32) {
    wr = st.writeByte(ID, REG_P_COEF, 32);
    delay(20);
    rb = st.readByte(ID, REG_P_COEF);

    if (wr < 0 || rb != 32) {
      st.LockEprom(ID);
      st.EnableTorque(ID, 0);
      Serial.println("NORMALIZE_ABORTED   : P_WRITE_VERIFY_FAILED");
      return;
    }

    Serial.println("P_COEF              : 16 -> 32 VERIFIED");
  } else {
    Serial.println("P_COEF              : 32 ALREADY TARGET");
  }

  // ---------------------------------------------------------------
  // EEPROM WRITE 2 — D 0 -> 32
  // ---------------------------------------------------------------

  if (dCoef != 32) {
    wr = st.writeByte(ID, REG_D_COEF, 32);
    delay(20);
    rb = st.readByte(ID, REG_D_COEF);

    if (wr < 0 || rb != 32) {
      st.LockEprom(ID);
      st.EnableTorque(ID, 0);
      Serial.println("NORMALIZE_ABORTED   : D_WRITE_VERIFY_FAILED");
      return;
    }

    Serial.println("D_COEF              : 0 -> 32 VERIFIED");
  } else {
    Serial.println("D_COEF              : 32 ALREADY TARGET");
  }

  // ---------------------------------------------------------------
  // EEPROM WRITE 3 — ProtectionCurrent 500 -> 310
  // ---------------------------------------------------------------

  if (protCurrent != 310) {
    wr = st.writeWord(ID, REG_PROT_CURRENT_L, 310);
    delay(20);
    rb = st.readWord(ID, REG_PROT_CURRENT_L);

    if (wr < 0 || rb != 310) {
      st.LockEprom(ID);
      st.EnableTorque(ID, 0);
      Serial.println("NORMALIZE_ABORTED   : PROT_CURRENT_VERIFY_FAILED");
      return;
    }

    Serial.println("PROT_CURRENT        : 500 -> 310 VERIFIED");
  } else {
    Serial.println("PROT_CURRENT        : 310 ALREADY TARGET");
  }

  // ---------------------------------------------------------------
  // EEPROM WRITE 4 — OverloadTorque 25 -> 80
  // ---------------------------------------------------------------

  if (overloadTorque != 80) {
    wr = st.writeByte(ID, REG_OVERLOAD_TORQUE, 80);
    delay(20);
    rb = st.readByte(ID, REG_OVERLOAD_TORQUE);

    if (wr < 0 || rb != 80) {
      st.LockEprom(ID);
      st.EnableTorque(ID, 0);
      Serial.println("NORMALIZE_ABORTED   : OVERLOAD_VERIFY_FAILED");
      return;
    }

    Serial.println("OVERLOAD_TORQUE     : 25 -> 80 VERIFIED");
  } else {
    Serial.println("OVERLOAD_TORQUE     : 80 ALREADY TARGET");
  }

  // ---------------------------------------------------------------
  // EEPROM WRITE 5 — MaxTorque 500 -> 1000
  // Deliberately LAST persistent performance-enabling write.
  // ---------------------------------------------------------------

  if (maxTorque != 1000) {
    wr = st.writeWord(ID, REG_MAX_TORQUE_L, 1000);
    delay(20);
    rb = st.readWord(ID, REG_MAX_TORQUE_L);

    if (wr < 0 || rb != 1000) {
      st.LockEprom(ID);
      st.EnableTorque(ID, 0);
      Serial.println("NORMALIZE_ABORTED   : MAX_TORQUE_VERIFY_FAILED");
      return;
    }

    Serial.println("MAX_TORQUE          : 500 -> 1000 VERIFIED");
  } else {
    Serial.println("MAX_TORQUE          : 1000 ALREADY TARGET");
  }

  // ---------------------------------------------------------------
  // LOCK EEPROM BEFORE RAM PERFORMANCE LIMIT IS CHANGED.
  // ---------------------------------------------------------------

  int lockAck = st.LockEprom(ID);
  delay(20);

  lockState = st.readByte(ID, SMS_STS_LOCK);

  if (lockAck < 0 || lockState != 1) {
    st.EnableTorque(ID, 0);
    Serial.println("NORMALIZE_ABORTED   : EEPROM_RELOCK_FAILED");
    Serial.println("ATTENTION           : CHECK EEPROM LOCK MANUALLY");
    return;
  }

  Serial.println("EEPROM_LOCK         : CONFIRMED");

  // ---------------------------------------------------------------
  // VERIFY ALL PERSISTENT TARGETS BEFORE TorqueLimit increase.
  // ---------------------------------------------------------------

  int vMaxTorque      = st.readWord(ID, REG_MAX_TORQUE_L);
  int vP              = st.readByte(ID, REG_P_COEF);
  int vD              = st.readByte(ID, REG_D_COEF);
  int vProtCurrent    = st.readWord(ID, REG_PROT_CURRENT_L);
  int vOverload       = st.readByte(ID, REG_OVERLOAD_TORQUE);

  if (vMaxTorque != 1000 ||
      vP != 32 ||
      vD != 32 ||
      vProtCurrent != 310 ||
      vOverload != 80) {

    st.EnableTorque(ID, 0);
    Serial.println("NORMALIZE_ABORTED   : EEPROM_FINAL_VERIFY_FAILED");
    return;
  }

  Serial.println("EEPROM_TARGETS      : VERIFIED");

  // ---------------------------------------------------------------
  // FINAL PERFORMANCE WRITE — RAM TorqueLimit 500 -> 1000
  // Torque is still OFF.
  // ---------------------------------------------------------------

  int torqueLimitAfter = torqueLimit;

  if (torqueLimit != 1000) {
    wr = st.writeWord(ID, SMS_STS_TORQUE_LIMIT_L, 1000);
    delay(20);

    torqueLimitAfter =
        st.readWord(ID, SMS_STS_TORQUE_LIMIT_L);

    if (wr < 0 || torqueLimitAfter != 1000) {
      st.EnableTorque(ID, 0);
      Serial.println("NORMALIZE_ABORTED   : TORQUE_LIMIT_VERIFY_FAILED");
      return;
    }

    Serial.println("TORQUE_LIMIT        : 500 -> 1000 VERIFIED");
  } else {
    Serial.println("TORQUE_LIMIT        : 1000 ALREADY TARGET");
  }

  // ---------------------------------------------------------------
  // IMMUTABILITY AUDIT
  // ---------------------------------------------------------------

  int idAfter        = st.readByte(ID, SMS_STS_ID);
  int baudAfter      = st.readByte(ID, SMS_STS_BAUD_RATE);
  int minAfter       = st.readWord(ID, SMS_STS_MIN_ANGLE_LIMIT_L);
  int maxAfter       = st.readWord(ID, SMS_STS_MAX_ANGLE_LIMIT_L);
  int cwAfter        = st.readByte(ID, SMS_STS_CW_DEAD);
  int ccwAfter       = st.readByte(ID, SMS_STS_CCW_DEAD);
  int offsetAfter    = st.readWord(ID, SMS_STS_OFS_L);
  int modeAfter      = st.readByte(ID, SMS_STS_MODE);

  int iAfter         = st.readByte(ID, REG_I_COEF);
  int minStartAfter  = st.readWord(ID, REG_MIN_START_FORCE_L);
  int protTorqueAfter= st.readByte(ID, REG_PROT_TORQUE);
  int protTimeAfter  = st.readByte(ID, REG_PROT_TIME);
  int speedPAfter    = st.readByte(ID, REG_SPEED_LOOP_P);
  int overCurrAfter  = st.readByte(ID, REG_OVERCURR_TIME);
  int velIAfter      = st.readByte(ID, REG_VELOCITY_LOOP_I);

  bool immutablePass =
      idAfter == storedID &&
      baudAfter == baudReg &&
      minAfter == minLimit &&
      maxAfter == maxLimit &&
      cwAfter == cwDead &&
      ccwAfter == ccwDead &&
      offsetAfter == offsetBefore &&
      modeAfter == mode &&
      iAfter == iCoef &&
      minStartAfter == minStart &&
      protTorqueAfter == protTorque &&
      protTimeAfter == protTime &&
      speedPAfter == speedLoopP &&
      overCurrAfter == overCurrentTime &&
      velIAfter == velocityLoopI;

  // ---------------------------------------------------------------
  // FINAL TORQUE OFF — independent final safety action.
  // ---------------------------------------------------------------

  st.EnableTorque(ID, 0);
  delay(20);

  int torqueFinal = st.readByte(ID, SMS_STS_TORQUE_ENABLE);
  int lockFinal   = st.readByte(ID, SMS_STS_LOCK);

  Serial.println();
  Serial.println("--- FINAL AUDIT ---");

  Serial.printf("OFFSET BEFORE      : %d (0x%04X)\n",
                offsetBefore, offsetBefore);
  Serial.printf("OFFSET AFTER       : %d (0x%04X)\n",
                offsetAfter, offsetAfter);

  Serial.printf("IMMUTABLE_FIELDS   : %s\n",
                immutablePass ? "PASS" : "FAIL");

  Serial.printf("FINAL_TORQUE       : %d\n", torqueFinal);
  Serial.printf("FINAL_EEPROM_LOCK  : %d\n", lockFinal);

  bool finalPass =
      immutablePass &&
      torqueFinal == 0 &&
      lockFinal == 1 &&
      st.readWord(ID, REG_MAX_TORQUE_L) == 1000 &&
      st.readByte(ID, REG_P_COEF) == 32 &&
      st.readByte(ID, REG_D_COEF) == 32 &&
      st.readWord(ID, REG_PROT_CURRENT_L) == 310 &&
      st.readByte(ID, REG_OVERLOAD_TORQUE) == 80 &&
      st.readWord(ID, SMS_STS_TORQUE_LIMIT_L) == 1000;

  if (finalPass) {
    Serial.println("NORMALIZE_RESULT    : PASS");
  } else {
    Serial.println("NORMALIZE_RESULT    : FAIL");
  }

  Serial.println("NO POSITION COMMANDS SENT");
  Serial.println("POSITION OFFSET     : NOT WRITTEN");
}

void runRamRecorderTest(int ID) {
  const uint32_t TEST_HZ = 500;
  const uint32_t TEST_MS = 5000;
  const uint32_t PERIOD_US = 1000000UL / TEST_HZ;
  const uint32_t TEST_SAMPLES = TEST_HZ * TEST_MS / 1000UL;

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" PSRAM RAW RECORDER TEST — ID %d\n", ID);
  Serial.println("====================================");

  Serial.printf("QCSample bytes       : %u\n", (unsigned)sizeof(QCSample));
  Serial.printf("QC max samples       : %lu\n",
                (unsigned long)QC_MAX_SAMPLES);
  Serial.printf("QC max buffer bytes  : %lu\n",
                (unsigned long)(QC_MAX_SAMPLES * sizeof(QCSample)));

  if (!psramFound()) {
    Serial.println("PSRAM_FOUND          : NO");
    Serial.println("RAMTEST_ABORTED");
    return;
  }

  Serial.println("PSRAM_FOUND          : YES");
  Serial.printf("PSRAM_TOTAL_BYTES    : %u\n",
                (unsigned)ESP.getPsramSize());
  Serial.printf("PSRAM_FREE_BEFORE    : %u\n",
                (unsigned)ESP.getFreePsram());

  int ping = st.Ping((uint8_t)ID);

  if (ping < 0 || st.Error != 0) {
    Serial.println("PING                 : FAIL");
    Serial.println("RAMTEST_ABORTED");
    return;
  }

  int torque = st.readByte(ID, SMS_STS_TORQUE_ENABLE);

  Serial.printf("TORQUE_PRECHECK      : %d\n", torque);

  if (torque != 0) {
    Serial.println("RAMTEST_ABORTED      : TORQUE_NOT_OFF");
    return;
  }

  QCSample *buffer = static_cast<QCSample *>(
      heap_caps_malloc(
          QC_MAX_SAMPLES * sizeof(QCSample),
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (!buffer) {
    Serial.println("PSRAM_ALLOC          : FAILED");
    Serial.println("RAMTEST_ABORTED");
    return;
  }

  Serial.println("PSRAM_ALLOC          : PASS");
  Serial.printf("PSRAM_FREE_ALLOCATED : %u\n",
                (unsigned)ESP.getFreePsram());

  uint32_t success = 0;
  uint32_t failures = 0;
  uint32_t missedDeadlines = 0;
  uint32_t statusErrors = 0;

  uint32_t intervalMin = 0;
  uint32_t intervalMax = 0;
  uint64_t intervalSum = 0;

  uint32_t previousStart = 0;

  uint32_t captureStart = micros();
  uint32_t nextDeadline = captureStart;

  for (uint32_t i = 0; i < TEST_SAMPLES; ++i) {

    while ((int32_t)(micros() - nextDeadline) < 0) {
      delayMicroseconds(20);
    }

    uint32_t sampleStart = micros();

    if ((int32_t)(sampleStart - nextDeadline) >=
        (int32_t)PERIOD_US) {
      missedDeadlines++;
    }

    if (i > 0) {
      uint32_t dt = sampleStart - previousStart;

      if (i == 1) {
        intervalMin = intervalMax = dt;
      } else {
        if (dt < intervalMin) intervalMin = dt;
        if (dt > intervalMax) intervalMax = dt;
      }

      intervalSum += dt;
    }

    previousStart = sampleStart;

    int n = st.FeedBack(ID);

    if (n < 0) {
      failures++;
      nextDeadline += PERIOD_US;
      continue;
    }

    QCSample &q = buffer[success];

    q.t_us = sampleStart - captureStart;
    q.position = (int16_t)st.ReadPos(-1);
    q.speed = (int16_t)st.ReadSpeed(-1);
    q.load = (int16_t)st.ReadLoad(-1);
    q.current = (int16_t)st.ReadCurrent(-1);

    q.goalPosition = 0xFFFF;
    q.goalSpeed = 0xFFFF;

    q.voltage = (uint8_t)st.ReadVoltage(-1);
    q.temperature = (uint8_t)st.ReadTemper(-1);
    q.moving = (uint8_t)st.ReadMove(-1);
    q.status = (uint8_t)st.Error;
    q.phase = 0;
    q.flags = 0;

    if (q.status != 0) {
      statusErrors++;
    }

    success++;
    nextDeadline += PERIOD_US;
  }

  uint32_t elapsedUs = micros() - captureStart;

  // Simple integrity checksum over captured records.
  uint32_t checksum = 2166136261u;

  const uint8_t *raw =
      reinterpret_cast<const uint8_t *>(buffer);

  const size_t rawBytes =
      (size_t)success * sizeof(QCSample);

  for (size_t i = 0; i < rawBytes; ++i) {
    checksum ^= raw[i];
    checksum *= 16777619u;
  }

  Serial.println();
  Serial.println("--- RECORDER RESULT ---");

  Serial.printf("TARGET_SAMPLES       : %lu\n",
                (unsigned long)TEST_SAMPLES);
  Serial.printf("SUCCESS_SAMPLES      : %lu\n",
                (unsigned long)success);
  Serial.printf("FEEDBACK_FAILURES    : %lu\n",
                (unsigned long)failures);
  Serial.printf("MISSED_DEADLINES     : %lu\n",
                (unsigned long)missedDeadlines);
  Serial.printf("STATUS_ERRORS        : %lu\n",
                (unsigned long)statusErrors);
  Serial.printf("ELAPSED_US           : %lu\n",
                (unsigned long)elapsedUs);

  if (success > 1) {
    float meanInterval =
        (float)intervalSum / (float)(success - 1);

    Serial.printf("INTERVAL_MIN_US      : %lu\n",
                  (unsigned long)intervalMin);
    Serial.printf("INTERVAL_MEAN_US     : %.2f\n",
                  meanInterval);
    Serial.printf("INTERVAL_MAX_US      : %lu\n",
                  (unsigned long)intervalMax);
  }

  Serial.printf("RAW_BYTES_USED       : %u\n",
                (unsigned)rawBytes);
  Serial.printf("RAW_FNV1A32          : 0x%08lX\n",
                (unsigned long)checksum);

  if (success > 0) {
    const QCSample &first = buffer[0];
    const QCSample &last = buffer[success - 1];

    Serial.println();
    Serial.printf("FIRST t/pos/speed    : %lu / %d / %d\n",
                  (unsigned long)first.t_us,
                  first.position,
                  first.speed);

    Serial.printf("LAST  t/pos/speed    : %lu / %d / %d\n",
                  (unsigned long)last.t_us,
                  last.position,
                  last.speed);

    Serial.printf("LAST temp/current    : %u / %d\n",
                  last.temperature,
                  last.current);
  }

  heap_caps_free(buffer);

  Serial.printf("PSRAM_FREE_AFTER     : %u\n",
                (unsigned)ESP.getFreePsram());

  Serial.println();

  if (success == TEST_SAMPLES &&
      failures == 0 &&
      missedDeadlines == 0 &&
      statusErrors == 0) {

    Serial.println("RAMTEST_RESULT       : PASS");
  } else {
    Serial.println("RAMTEST_RESULT       : FAIL");
  }

  Serial.println("NO MOTION COMMANDS SENT");
  Serial.println("NO EEPROM WRITES");
}


static uint16_t qcU16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int qcSigned15(uint16_t v) {
  if (v & 0x8000) return -(int)(v & 0x7FFF);
  return (int)v;
}

static int qcSignedLoad(uint16_t v) {
  if (v & (1 << 10)) return -(int)(v & ~(1 << 10));
  return (int)v;
}

static int qcPosition(const QCFullSample &q) {
  return (int)qcU16(&q.raw[0]);
}

static int qcSpeed(const QCFullSample &q) {
  return qcSigned15(qcU16(&q.raw[2]));
}

static int qcLoad(const QCFullSample &q) {
  return qcSignedLoad(qcU16(&q.raw[4]));
}

static int qcVoltage(const QCFullSample &q) {
  return q.raw[6];
}

static int qcTemperature(const QCFullSample &q) {
  return q.raw[7];
}

static int qcServoStatus(const QCFullSample &q) {
  return q.raw[9];       // register 65
}

static int qcMoving(const QCFullSample &q) {
  return q.raw[10];      // register 66
}

static int qcCurrent(const QCFullSample &q) {
  return qcSigned15(qcU16(&q.raw[13]));
}

static bool qcGlobalTimeOK() {
  return (uint32_t)(micros() - qcStartUs) < QC_GLOBAL_ABORT_US;
}

static bool qcReadAndStore(
    int id,
    uint8_t phase,
    uint16_t goalPosition,
    uint16_t goalSpeed,
    uint8_t goalAcc,
    QCFullSample *out = nullptr) {

  if (!qcBuffer || qcCount >= QC_FAST_MAX_SAMPLES) {
    return false;
  }

  QCFullSample &q = qcBuffer[qcCount];

  uint8_t raw[15];

  int n = st.Read(
      (uint8_t)id,
      SMS_STS_PRESENT_POSITION_L,
      raw,
      sizeof(raw));

  if (n != (int)sizeof(raw)) {
    return false;
  }

  q.t_us = (uint32_t)(micros() - qcStartUs);
  q.goal_position = goalPosition;
  q.goal_speed = goalSpeed;
  q.goal_acc = goalAcc;
  q.phase = phase;
  q.proto_status = st.Error;
  q.flags = 0;

  memcpy(q.raw, raw, sizeof(raw));

  if (out) *out = q;

  qcCount++;
  return true;
}


static bool qcThermalGuard(
    int id,
    String &reason) {

  const uint32_t now = micros();

  /*
   * Thermal safety deliberately runs much slower than mechanical
   * telemetry. Raw temperature in the 500 Hz record is NEVER altered.
   */
  if ((int32_t)(now - qcNextThermalCheckUs) < 0) {
    return true;
  }

  qcNextThermalCheckUs =
      now + QC_THERMAL_CHECK_PERIOD_US;

  /*
   * Direct single-register read:
   * Present Temperature = 0x3F / decimal 63.
   *
   * This mirrors the V25 mitigation for transient corrupted
   * temperature values observed in bulk RAM telemetry.
   */
  int initial =
      st.readByte(
          id,
          SMS_STS_PRESENT_TEMPERATURE);

  if (initial < 0) {
    reason = "THERMAL_DIRECT_READ_FAILED";
    return false;
  }

  if (initial <= QC_THERMAL_LIMIT_C) {
    return true;
  }

  int confirmations[QC_THERMAL_CONFIRM_SAMPLES];

  Serial.printf(
      "THERMAL_CANDIDATE initial=%d limit=%d\n",
      initial,
      QC_THERMAL_LIMIT_C);

  for (int i = 0;
       i < QC_THERMAL_CONFIRM_SAMPLES;
       ++i) {

    delay(QC_THERMAL_CONFIRM_DELAY_MS);

    confirmations[i] =
        st.readByte(
            id,
            SMS_STS_PRESENT_TEMPERATURE);

    if (confirmations[i] < 0) {
      reason = "THERMAL_CONFIRM_READ_FAILED";
      return false;
    }

    Serial.printf(
        "THERMAL_CONFIRM index=%d temp=%d\n",
        i + 1,
        confirmations[i]);
  }

  bool allHigh = true;

  for (int i = 0;
       i < QC_THERMAL_CONFIRM_SAMPLES;
       ++i) {

    if (confirmations[i] <=
        QC_THERMAL_LIMIT_C) {

      allHigh = false;
    }
  }

  /*
   * Critical V25 rule:
   *
   * TRANSIENT COUNT / BUDGET EXHAUSTION IS NEVER
   * EQUIVALENT TO THERMAL CONFIRMATION.
   *
   * A real thermal abort requires all three fresh direct
   * confirmations to remain above the configured limit.
   */
  if (allHigh) {
    qcThermalConfirmedCount++;

    Serial.printf(
        "THERMAL_CONFIRMED initial=%d confirmations=[%d,%d,%d]\n",
        initial,
        confirmations[0],
        confirmations[1],
        confirmations[2]);

    reason = "THERMAL_CONFIRMED_OVER_LIMIT";
    return false;
  }

  qcThermalTransientCount++;

  Serial.printf(
      "THERMAL_TRANSIENT initial=%d confirmations=[%d,%d,%d] transient_count=%lu\n",
      initial,
      confirmations[0],
      confirmations[1],
      confirmations[2],
      (unsigned long)qcThermalTransientCount);

  return true;
}

static bool qcStaticCapture(
    int id,
    uint32_t durationMs,
    uint8_t phase,
    String &reason) {

  uint32_t start = micros();
  uint32_t next = start;

  int consecutiveFailures = 0;

  while ((uint32_t)(micros() - start) <
         durationMs * 1000UL) {

    if (!qcGlobalTimeOK()) {
      reason = "GLOBAL_TIME_LIMIT";
      return false;
    }

    while ((int32_t)(micros() - next) < 0) {
      delayMicroseconds(20);
    }

    QCFullSample q;

    if (!qcReadAndStore(
          id, phase, 0xFFFF, 0xFFFF, 0xFF, &q)) {

      consecutiveFailures++;

      if (consecutiveFailures >= 3) {
        reason = "3_CONSECUTIVE_FEEDBACK_FAILURES";
        return false;
      }

    } else {
      consecutiveFailures = 0;

      if (q.proto_status != 0) {
        reason = "PROTOCOL_STATUS_ERROR";
        return false;
      }

      if (qcServoStatus(q) != 0) {
        reason = "SERVO_STATUS_ERROR";
        return false;
      }

      if (qcPosition(q) < 0 || qcPosition(q) > 4095) {
        reason = "POSITION_OUT_OF_DOMAIN";
        return false;
      }

      if (!qcThermalGuard(id, reason)) {
        return false;
      }

      if (qcVoltage(q) < 90 || qcVoltage(q) > 130) {
        reason = "VOLTAGE_OUT_OF_RANGE";
        return false;
      }
    }

    next += QC_FAST_PERIOD_US;
  }

  return true;
}


static uint32_t qcComputeMoveTimeoutMs(
    int startPos,
    int target,
    uint16_t speedCmd,
    uint8_t phase) {

  const uint32_t distance =
      (uint32_t)abs(target - startPos);

  /*
   * Conservative minimum expected velocity.
   *
   * GoalSpeed=1 is intentionally treated as ~1 tick/s.
   * Normal moves use only half the commanded rate as timeout
   * expectation. GoalSpeed=0 (no speed limit / maximum) uses
   * a deliberately conservative 450 tick/s floor.
   */
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
      (distance * 1000UL +
       expectedFloorTps - 1) /
      expectedFloorTps;

  uint32_t marginMs =
      (speedCmd <= 2) ? 2500UL : 2200UL;

  if (speedCmd == 0) {
    marginMs = 2500UL;
  }

  uint32_t result =
      travelMs + marginMs;

  if (result < 2000UL) {
    result = 2000UL;
  }

  // Stall detection remains much faster; this is only the
  // absolute per-move deadline.
  if (result > 30000UL) {
    result = 30000UL;
  }

  return result;
}


static const char *qcOutcomeName(
    QCMoveOutcome outcome) {

  switch (outcome) {
    case QCM_REACHED:
      return "REACHED";

    case QCM_SETTLED_OFF_TARGET:
      return "SETTLED_OFF_TARGET";

    case QCM_NO_MOTION:
      return "NO_MOTION";

    case QCM_NO_PROGRESS:
      return "NO_PROGRESS";

    case QCM_WINDOW_END:
      return "WINDOW_END";

    case QCM_PROTECT_CURRENT:
      return "PROTECT_CURRENT";

    case QCM_PROTECT_LOAD:
      return "PROTECT_LOAD";

    case QCM_PROTECT_STATUS:
      return "PROTECT_STATUS";

    case QCM_PROTECT_DIRECTION:
      return "PROTECT_DIRECTION";

    case QCM_PROTECT_STALL:
      return "PROTECT_STALL";

    case QCM_OBSERVATION_COMPLETE:
      return "OBSERVATION_COMPLETE";
  }

  return "UNKNOWN";
}

static bool qcOutcomeNeedsRecovery(
    QCMoveOutcome outcome) {

  return
      outcome == QCM_PROTECT_CURRENT ||
      outcome == QCM_PROTECT_LOAD ||
      outcome == QCM_PROTECT_STATUS ||
      outcome == QCM_PROTECT_DIRECTION ||
      outcome == QCM_PROTECT_STALL;
}

static void qcRegisterOutcome(
    QCMoveOutcome outcome) {

  if (outcome == QCM_SETTLED_OFF_TARGET) {
    qcOffTargetEvents++;
    return;
  }

  if (outcome == QCM_REACHED ||
      outcome == QCM_OBSERVATION_COMPLETE) {
    return;
  }

  qcPerformanceEvents++;

  switch (outcome) {
    case QCM_SETTLED_OFF_TARGET:
      break;

    case QCM_NO_MOTION:
      qcNoMotionEvents++;
      break;

    case QCM_NO_PROGRESS:
      qcNoProgressEvents++;
      break;

    case QCM_WINDOW_END:
      qcWindowEndEvents++;
      break;

    case QCM_PROTECT_CURRENT:
    case QCM_PROTECT_LOAD:
    case QCM_PROTECT_STATUS:
    case QCM_PROTECT_DIRECTION:
    case QCM_PROTECT_STALL:
      qcProtectiveStops++;
      break;

    default:
      break;
  }
}

static bool qcRetryWritePos(
    int id,
    int target,
    uint16_t speedCmd,
    uint8_t accCmd) {

  for (int attempt = 0;
       attempt < 3;
       ++attempt) {

    int wr = st.WritePosEx(
        (uint8_t)id,
        (s16)target,
        speedCmd,
        accCmd);

    if (wr >= 0) {
      return true;
    }

    delay(10);
  }

  return false;
}

static bool qcRearmAtCurrent(
    int id,
    int requestedHoldPosition,
    String &fatalReason) {

  // First remove all torque deterministically.
  if (!forceTorqueOffVerified(
        id,
        "QC_PROTECT")) {

    fatalReason =
        "PROTECTIVE_TORQUE_OFF_FAILED";

    return false;
  }

  delay(150);

  QCFullSample offSample;

  if (!qcReadAndStore(
        id,
        QC_TRANSFER,
        0xFFFF,
        0xFFFF,
        0xFF,
        &offSample)) {

    fatalReason =
        "PROTECTIVE_RECOVERY_READ_FAILED";

    return false;
  }

  if (!qcThermalGuard(
        id,
        fatalReason)) {

    return false;
  }

  const int actualPosition =
      qcPosition(offSample);

  /*
   * PRIME at the ACTUAL current position.
   *
   * As already proven on this C018 firmware, WritePosEx can
   * implicitly enable torque. Therefore PRIME is immediately
   * followed by another verified torque OFF before the explicit
   * torque enable.
   */
  if (!qcRetryWritePos(
        id,
        actualPosition,
        100,
        10)) {

    fatalReason =
        "PROTECTIVE_PRIME_WRITE_FAILED";

    return false;
  }

  delay(20);

  if (!forceTorqueOffVerified(
        id,
        "QC_PROTECT_PRIME")) {

    fatalReason =
        "PROTECTIVE_PRIME_OFF_FAILED";

    return false;
  }

  if (st.EnableTorque(
        (uint8_t)id,
        1) < 0) {

    fatalReason =
        "PROTECTIVE_REENABLE_FAILED";

    return false;
  }

  delay(20);

  if (st.readByte(
        id,
        SMS_STS_TORQUE_ENABLE) != 1) {

    fatalReason =
        "PROTECTIVE_REENABLE_NOT_CONFIRMED";

    return false;
  }

  Serial.printf(
      "PROTECTIVE_RECOVERY: REARMED_AT=%d\n",
      actualPosition);

  return true;
}

static bool qcNeutralizeMove(
    int id,
    int holdPosition,
    bool protective,
    String &fatalReason) {

  if (!qcRetryWritePos(
        id,
        holdPosition,
        100,
        10)) {

    fatalReason =
        "NEUTRALIZE_WRITE_FAILED";

    return false;
  }

  delay(25);

  if (protective) {
    return qcRearmAtCurrent(
        id,
        holdPosition,
        fatalReason);
  }

  return true;
}

static bool qcPathWasBlocked(
    const QCMoveResult &r) {

  if (r.outcome == QCM_NO_MOTION ||
      r.outcome == QCM_NO_PROGRESS ||
      r.outcome == QCM_PROTECT_CURRENT ||
      r.outcome == QCM_PROTECT_LOAD ||
      r.outcome == QCM_PROTECT_STATUS ||
      r.outcome == QCM_PROTECT_DIRECTION ||
      r.outcome == QCM_PROTECT_STALL) {

    return true;
  }

  // Window expiry alone is not automatically a blocked path.
  // Use the actually measured residual error.
  if ((r.outcome == QCM_WINDOW_END ||
       r.outcome == QCM_SETTLED_OFF_TARGET) &&
      abs(r.finalPos - r.targetPos) > 64) {

    return true;
  }

  return false;
}


static bool qcMinSettleBarrier(
    int id,
    int holdPosition,
    String &fatalReason) {

  const uint32_t start = micros();
  uint32_t next = start;

  int stable = 0;
  int failures = 0;

  while ((uint32_t)(micros() - start) <
         1000000UL) {

    if (!qcGlobalTimeOK()) {
      fatalReason = "GLOBAL_TIME_LIMIT";
      return false;
    }

    while ((int32_t)(micros() - next) < 0) {
      delayMicroseconds(20);
    }

    QCFullSample q;

    if (!qcReadAndStore(
          id,
          QC_TRANSFER,
          (uint16_t)holdPosition,
          100,
          10,
          &q)) {

      failures++;

      if (failures >= 25) {
        fatalReason =
            "MIN_SETTLE_TELEMETRY_LOST";
        return false;
      }

      next += QC_FAST_PERIOD_US;
      continue;
    }

    failures = 0;

    if (q.proto_status != 0) {
      fatalReason =
          "MIN_SETTLE_PROTOCOL_ERROR";
      return false;
    }

    if (qcServoStatus(q) &
        QC_STATUS_SENSOR) {

      fatalReason =
          "MIN_SETTLE_SENSOR_ERROR";
      return false;
    }

    if (!qcThermalGuard(
          id,
          fatalReason)) {

      return false;
    }

    if (abs(qcSpeed(q)) <= 10 &&
        qcMoving(q) == 0) {

      stable++;

      // ~80 ms continuously stationary @500 Hz.
      if (stable >= 40) {

        Serial.printf(
            "MIN_SETTLE_CONFIRMED pos=%d elapsed_ms=%lu\n",
            qcPosition(q),
            (unsigned long)(
                (micros() - start) / 1000UL));

        return true;
      }

    } else {
      stable = 0;
    }

    next += QC_FAST_PERIOD_US;
  }

  /*
   * If dynamic settling was unusually long, do not turn this
   * into a performance FAIL. Use the already-reviewed
   * verified torque-OFF/re-arm path to guarantee a stationary
   * starting condition for the next measurement.
   */
  Serial.println(
      "MIN_SETTLE_FALLBACK: VERIFIED_OFF_REARM");

  return qcRearmAtCurrent(
      id,
      holdPosition,
      fatalReason);
}

static bool qcCharacterizeMove(
    int id,
    int target,
    uint16_t speedCmd,
    uint8_t accCmd,
    uint8_t phase,
    uint32_t windowOverrideMs,
    uint32_t minimumObservationMs,
    String &fatalReason,
    QCMoveResult &result) {

  QCFullSample before;

  if (!qcReadAndStore(
        id,
        phase,
        (uint16_t)target,
        speedCmd,
        accCmd,
        &before)) {

    fatalReason =
        "CHAR_PRE_FEEDBACK_FAILED";

    return false;
  }

  const int startPos =
      qcPosition(before);

  result.outcome = QCM_WINDOW_END;

  result.startPos = startPos;
  result.targetPos = target;
  result.finalPos = startPos;

  result.minPos = startPos;
  result.maxPos = startPos;

  result.peakAbsSpeed = 0;
  result.peakAbsLoad = 0;
  result.peakAbsCurrent = 0;

  result.elapsedUs = 0;
  result.firstMotionUs = 0;
  result.firstMotionSeen = false;

  if (startPos < qcObservedMin) {
    qcObservedMin = startPos;
  }

  if (startPos > qcObservedMax) {
    qcObservedMax = startPos;
  }

  int direction = 0;

  if (target > startPos) {
    direction = +1;
  } else if (target < startPos) {
    direction = -1;
  }

  uint32_t windowMs =
      windowOverrideMs;

  if (windowMs == 0) {
    windowMs =
        qcComputeMoveTimeoutMs(
            startPos,
            target,
            speedCmd,
            phase);
  }

  if (minimumObservationMs == 0) {
    minimumObservationMs = 400;
  }

  Serial.printf(
      "MEASURE_BEGIN phase=%u start=%d target=%d "
      "speed=%u acc=%u window_ms=%lu\n",
      phase,
      startPos,
      target,
      speedCmd,
      accCmd,
      (unsigned long)windowMs);

  if (!qcRetryWritePos(
        id,
        target,
        speedCmd,
        accCmd)) {

    fatalReason =
        "CHAR_GOAL_WRITE_FAILED";

    return false;
  }

  const uint32_t moveStart = micros();
  uint32_t next = moveStart;

  uint32_t lastProgressUs =
      moveStart;

  int lastProgressPos =
      startPos;

  int stableSamples = 0;
  int consecutiveFailures = 0;

  uint32_t overcurrentSamples = 0;
  uint32_t stallCurrentSamples = 0;
  uint32_t stallSignatureSamples = 0;

  uint32_t protectionStatusSamples = 0;
  uint32_t voltageBadSamples = 0;

  int lastPos =
      startPos;

  bool finished = false;

  while ((uint32_t)(micros() - moveStart) <
         windowMs * 1000UL) {

    if (!qcGlobalTimeOK()) {
      fatalReason =
          "GLOBAL_TIME_LIMIT";

      return false;
    }

    while ((int32_t)(micros() - next) < 0) {
      delayMicroseconds(20);
    }

    QCFullSample q;

    if (!qcReadAndStore(
          id,
          phase,
          (uint16_t)target,
          speedCmd,
          accCmd,
          &q)) {

      consecutiveFailures++;

      /*
       * At 500 Hz, 25 consecutive misses means roughly
       * 50 ms with no valid telemetry. That is an integrity
       * problem, not a servo-performance result.
       */
      if (consecutiveFailures >= 25) {
        fatalReason =
            "TELEMETRY_LOST_PERSISTENT";

        return false;
      }

      next += QC_FAST_PERIOD_US;
      continue;
    }

    consecutiveFailures = 0;

    const int pos =
        qcPosition(q);

    const int speed =
        qcSpeed(q);

    const int load =
        qcLoad(q);

    const int current =
        qcCurrent(q);

    const int voltage =
        qcVoltage(q);

    const uint8_t status =
        (uint8_t)qcServoStatus(q);

    lastPos = pos;

    if (pos < result.minPos) {
      result.minPos = pos;
    }

    if (pos > result.maxPos) {
      result.maxPos = pos;
    }

    if (pos < qcObservedMin) {
      qcObservedMin = pos;
    }

    if (pos > qcObservedMax) {
      qcObservedMax = pos;
    }

    if (abs(speed) >
        result.peakAbsSpeed) {

      result.peakAbsSpeed =
          abs(speed);
    }

    if (abs(load) >
        result.peakAbsLoad) {

      result.peakAbsLoad =
          abs(load);
    }

    if (abs(current) >
        result.peakAbsCurrent) {

      result.peakAbsCurrent =
          abs(current);
    }

    // Protocol status from the reply itself.
    if (q.proto_status != 0) {
      fatalReason =
          "PROTOCOL_STATUS_ERROR";

      return false;
    }

    /*
     * Sensor failure means position telemetry can no longer
     * be trusted. This is one of the few legitimate global
     * abort conditions.
     */
    if (status & QC_STATUS_SENSOR) {
      fatalReason =
          "SERVO_SENSOR_ERROR";

      return false;
    }

    /*
     * Temperature remains handled by the proven V25
     * confirmation filter: a single bad byte cannot abort.
     */
    if (!qcThermalGuard(
          id,
          fatalReason)) {

      return false;
    }

    /*
     * Voltage is a true electrical-safety/integrity gate,
     * not a performance score. C018 operating range is
     * nominally 4.0..14.0 V.
     */
    if ((status & QC_STATUS_VOLTAGE) ||
        voltage < 40 ||
        voltage > 140) {

      voltageBadSamples++;

      if (voltageBadSamples >=
          QC_VOLTAGE_PERSIST_SAMPLES) {

        fatalReason =
            "VOLTAGE_SAFETY_PERSISTENT";

        return false;
      }

    } else {
      voltageBadSamples = 0;
    }

    /*
     * Current / overload / angle status from the servo's
     * own protection logic.
     *
     * This stops the CURRENT excitation and safely recovers,
     * but remains CHARACTERIZATION DATA, not a global QC fail.
     */
    if (status &
        (QC_STATUS_CURRENT |
         QC_STATUS_ANGLE |
         QC_STATUS_OVERLOAD)) {

      protectionStatusSamples++;

      if (protectionStatusSamples >=
          QC_STATUS_PERSIST_SAMPLES) {

        result.outcome =
            QCM_PROTECT_STATUS;

        finished = true;
        break;
      }

    } else {
      protectionStatusSamples = 0;
    }

    /*
     * Feetech C018 stall current = 2.7A.
     * Current feedback = 6.5mA/raw.
     *
     * Near true stall-current level: short persistence is
     * sufficient for a protective stop.
     */
    if (abs(current) >=
        QC_STALL_CURRENT_RAW) {

      stallCurrentSamples++;

      if (stallCurrentSamples >=
          QC_STALL_CURRENT_PERSIST_SAMPLES) {

        result.outcome =
            QCM_PROTECT_CURRENT;

        finished = true;
        break;
      }

    } else {
      stallCurrentSamples = 0;
    }

    /*
     * Manufacturer over-current protection reference is
     * >2A for 2s. On the bench we stop this excitation much
     * earlier (100ms) to avoid stressing a known-damaged unit.
     *
     * Still NOT a quality score.
     */
    if (abs(current) >=
        QC_OVERCURRENT_2A_RAW) {

      overcurrentSamples++;

      if (overcurrentSamples >=
          QC_OVERCURRENT_PERSIST_SAMPLES) {

        result.outcome =
            QCM_PROTECT_CURRENT;

        finished = true;
        break;
      }

    } else {
      overcurrentSamples = 0;
    }

    /*
     * COMPOSITE MECHANICAL STALL / BINDING SIGNATURE.
     *
     * Feetech load feedback is drive output:
     * 1000 ~= 100%.
     *
     * Therefore load can be near 1000 during perfectly
     * legitimate maximum-speed operation.
     *
     * NEVER stop because of load alone.
     */
    const bool noRecentProgress =
        (uint32_t)(micros() - lastProgressUs) >=
        QC_STALL_NO_PROGRESS_US;

    const bool highDrive =
        abs(load) >= QC_OVERLOAD_LOAD_RAW;

    const bool meaningfulCurrent =
        abs(current) >= QC_RATED_CURRENT_RAW;

    const bool lowActualSpeed =
        abs(speed) <= QC_STALL_LOW_SPEED_RAW;

    if (highDrive &&
        meaningfulCurrent &&
        lowActualSpeed &&
        noRecentProgress) {

      stallSignatureSamples++;

      if (stallSignatureSamples >=
          QC_STALL_SIGNATURE_SAMPLES) {

        result.outcome =
            QCM_PROTECT_STALL;

        finished = true;
        break;
      }

    } else {
      stallSignatureSamples = 0;
    }

    int directionalTravel = 0;

    if (direction > 0) {
      directionalTravel =
          pos - startPos;

    } else if (direction < 0) {
      directionalTravel =
          startPos - pos;
    }

    /*
     * A small amount of encoder jitter is measurement data.
     * More than 16 ticks in the commanded opposite direction
     * is treated as potentially uncontrolled motion:
     * stop this excitation, recover, continue if possible.
     */
    if (direction != 0 &&
        directionalTravel < -16) {

      result.outcome =
          QCM_PROTECT_DIRECTION;

      finished = true;
      break;
    }

    if (!result.firstMotionSeen &&
        abs(pos - startPos) >= 1) {

      result.firstMotionSeen = true;

      result.firstMotionUs =
          micros() - moveStart;
    }

    if (abs(pos - lastProgressPos) >= 1) {
      lastProgressPos = pos;
      lastProgressUs = micros();
    }

    const uint32_t elapsedMs =
        (micros() - moveStart) / 1000UL;

    const int residual =
        abs(target - pos);

    /*
     * Minimum-speed probes intentionally hold an observation
     * window. Do NOT classify their lack of progress as an
     * execution failure.
     */
    if (phase != QC_MIN_PROBE &&
        residual > 32 &&
        (uint32_t)(micros() - lastProgressUs) >
            1500000UL) {

      result.outcome =
          result.firstMotionSeen
          ? QCM_NO_PROGRESS
          : QCM_NO_MOTION;

      finished = true;
      break;
    }

    /*
     * Servo can legitimately stop away from target.
     * That is exactly what we want to measure.
     *
     * If it never moved at all, allow at least 1.2 s before
     * declaring NO_MOTION, so slower/bad units are not
     * prematurely classified.
     */
    const bool stationary =
        abs(speed) <= 10 &&
        qcMoving(q) == 0;

    bool allowedToSettle =
        elapsedMs >= minimumObservationMs;

    if (!result.firstMotionSeen &&
        residual > 3) {

      allowedToSettle =
          elapsedMs >= 1200;
    }

    if (stationary &&
        allowedToSettle) {

      stableSamples++;

      if (stableSamples >= 40) {

        if (!result.firstMotionSeen &&
            residual > 3) {

          result.outcome =
              QCM_NO_MOTION;

        } else if (residual <= 3) {

          result.outcome =
              QCM_REACHED;

        } else {

          result.outcome =
              QCM_SETTLED_OFF_TARGET;
        }

        finished = true;
        break;
      }

    } else {
      stableSamples = 0;
    }

    next += QC_FAST_PERIOD_US;
  }

  if (!finished) {
    result.outcome =
        (phase == QC_MIN_PROBE)
        ? QCM_OBSERVATION_COMPLETE
        : QCM_WINDOW_END;
  }

  result.finalPos =
      lastPos;

  result.elapsedUs =
      micros() - moveStart;

  const bool protective =
      qcOutcomeNeedsRecovery(
          result.outcome) ||
      qcPathWasBlocked(result);

  /*
   * Every characterization command ends by moving the goal
   * to the ACTUAL measured position. A protective event also
   * performs verified torque OFF + safe re-arm.
   */
  if (!qcNeutralizeMove(
        id,
        lastPos,
        protective,
        fatalReason)) {

    return false;
  }

  if (phase == QC_MIN_PROBE) {

    if (!qcMinSettleBarrier(
          id,
          lastPos,
          fatalReason)) {

      return false;
    }
  }

  qcRegisterOutcome(
      result.outcome);

  if (qcPathWasBlocked(result) &&
      (phase == QC_PREPOSITION ||
       phase == QC_SLOW_UP ||
       phase == QC_SLOW_DOWN ||
       phase == QC_STEP45_MEDIUM ||
       phase == QC_TRANSFER)) {

    qcUnsafeForMax = true;

    Serial.println(
        "MAX_SAFETY_GATE: ARMED_BY_MEASURED_PATH_BLOCK");
  }

  Serial.printf(
      "MEASURE_END phase=%u outcome=%s "
      "start=%d target=%d final=%d error=%d "
      "range=%d..%d peak_speed=%d "
      "peak_load=%d peak_current=%d "
      "first_motion_ms=%.3f elapsed_ms=%.3f\n",
      phase,
      qcOutcomeName(result.outcome),
      result.startPos,
      result.targetPos,
      result.finalPos,
      result.finalPos - result.targetPos,
      result.minPos,
      result.maxPos,
      result.peakAbsSpeed,
      result.peakAbsLoad,
      result.peakAbsCurrent,
      result.firstMotionSeen
        ? result.firstMotionUs / 1000.0f
        : -1.0f,
      result.elapsedUs / 1000.0f);

  return true;
}

static bool qcMove(
    int id,
    int target,
    uint16_t speedCmd,
    uint8_t accCmd,
    uint8_t phase,
    uint32_t timeoutMs,
    String &reason) {

  QCFullSample before;

  if (!qcReadAndStore(
        id, phase, target, speedCmd, accCmd, &before)) {
    reason = "PRE_MOVE_FEEDBACK_FAILED";
    return false;
  }

  int startPos = qcPosition(before);

  if (startPos < 0 || startPos > 4095) {
    reason = "PRE_MOVE_POSITION_INVALID";
    return false;
  }

  int direction = 0;
  if (target > startPos) direction = +1;
  if (target < startPos) direction = -1;

  const int commandDistance =
      abs(target - startPos);

  /*
   * TRUE minimum-speed probe:
   * target is only a few encoder ticks away at GoalSpeed=1.
   * It MUST physically advance. Starting inside the normal
   * +/-3 settling window is NOT sufficient.
   */
  const bool trueMinProbe =
      (phase == QC_MIN_PROBE &&
       speedCmd == 1 &&
       commandDistance <= 4);

  const int settleTolerance =
      trueMinProbe ? 1 : 3;

  const int minimumDirectionalTravel =
      trueMinProbe ? 2 : 0;

  int maxDirectionalTravel = 0;

  uint32_t computedTimeoutMs =
      qcComputeMoveTimeoutMs(
          startPos,
          target,
          speedCmd,
          phase);

  if (timeoutMs < computedTimeoutMs) {
    timeoutMs = computedTimeoutMs;
  }

  Serial.printf(
      "MOVE_BEGIN phase=%u start=%d target=%d "
      "distance=%d speed=%u acc=%u timeout_ms=%lu%s\n",
      phase,
      startPos,
      target,
      commandDistance,
      speedCmd,
      accCmd,
      (unsigned long)timeoutMs,
      trueMinProbe ? " TRUE_MIN" : "");

  int wr = st.WritePosEx(
      (uint8_t)id,
      (s16)target,
      speedCmd,
      accCmd);

  if (wr < 0) {
    reason = "GOAL_WRITE_ACK_FAILED";
    return false;
  }

  uint32_t moveStart = micros();
  uint32_t next = moveStart;

  int stable = 0;
  int consecutiveFailures = 0;

  int highCurrentCount = 0;
  int highLoadCount = 0;
  int badVoltageCount = 0;

  int lastProgressPos = startPos;
  uint32_t lastProgressUs = moveStart;

  uint32_t stallLimitUs;

  if (speedCmd <= 2) {
    stallLimitUs = 4500000UL;
  } else if (speedCmd <= 200) {
    stallLimitUs = 1800000UL;
  } else {
    stallLimitUs = 900000UL;
  }

  while ((uint32_t)(micros() - moveStart) <
         timeoutMs * 1000UL) {

    if (!qcGlobalTimeOK()) {
      reason = "GLOBAL_TIME_LIMIT";
      return false;
    }

    while ((int32_t)(micros() - next) < 0) {
      delayMicroseconds(20);
    }

    QCFullSample q;

    if (!qcReadAndStore(
          id, phase, target, speedCmd, accCmd, &q)) {

      consecutiveFailures++;

      if (consecutiveFailures >= 3) {
        reason = "3_CONSECUTIVE_FEEDBACK_FAILURES";
        return false;
      }

      next += QC_FAST_PERIOD_US;
      continue;
    }

    consecutiveFailures = 0;

    const int pos = qcPosition(q);
    const int speed = qcSpeed(q);
    const int load = qcLoad(q);
    const int current = qcCurrent(q);
    const int voltage = qcVoltage(q);
    const int temp = qcTemperature(q);
    const int moving = qcMoving(q);

    if (q.proto_status != 0) {
      reason = "PROTOCOL_STATUS_ERROR";
      return false;
    }

    if (qcServoStatus(q) != 0) {
      reason = "SERVO_STATUS_ERROR";
      return false;
    }

    if (pos < 0 || pos > 4095) {
      reason = "POSITION_OUT_OF_DOMAIN";
      return false;
    }

    if (!qcThermalGuard(id, reason)) {
      return false;
    }

    if (voltage < 90 || voltage > 130) {
      badVoltageCount++;
      if (badVoltageCount >= 25) {
        reason = "VOLTAGE_OUT_OF_RANGE_PERSISTENT";
        return false;
      }
    } else {
      badVoltageCount = 0;
    }

    if (abs(current) > 300) {
      highCurrentCount++;
      if (highCurrentCount >= 25) {
        reason = "HIGH_CURRENT_PERSISTENT";
        return false;
      }
    } else {
      highCurrentCount = 0;
    }

    if (abs(load) > 900) {
      highLoadCount++;
      if (highLoadCount >= 25) {
        reason = "HIGH_LOAD_PERSISTENT";
        return false;
      }
    } else {
      highLoadCount = 0;
    }

    // Wrong-direction gate.
    if ((uint32_t)(micros() - moveStart) > 50000UL) {

      const int wrongTolerance =
          trueMinProbe ? 1 : 5;

      if (direction < 0 &&
          pos > startPos + wrongTolerance) {
        reason = "WRONG_DIRECTION";
        return false;
      }

      if (direction > 0 &&
          pos < startPos - wrongTolerance) {
        reason = "WRONG_DIRECTION";
        return false;
      }
    }

    int directionalTravel = 0;

    if (direction > 0) {
      directionalTravel = pos - startPos;
    } else if (direction < 0) {
      directionalTravel = startPos - pos;
    }

    if (directionalTravel >
        maxDirectionalTravel) {
      maxDirectionalTravel =
          directionalTravel;
    }

    if (abs(pos - lastProgressPos) >= 1) {
      lastProgressPos = pos;
      lastProgressUs = micros();
    }

    if (direction != 0 &&
        (uint32_t)(micros() - lastProgressUs) >
        stallLimitUs &&
        abs(pos - target) > 3) {

      reason = "NO_PROGRESS_STALL";
      return false;
    }

    if (abs(pos - target) <= settleTolerance &&
        abs(speed) <= 10 &&
        moving == 0 &&
        maxDirectionalTravel >=
            minimumDirectionalTravel) {

      stable++;

      if (stable >= 25) {

        Serial.printf(
            "MOVE_DONE phase=%u target=%d final=%d "
            "error=%d travel=%d elapsed_ms=%lu%s\n",
            phase,
            target,
            pos,
            pos - target,
            maxDirectionalTravel,
            (unsigned long)(
                (micros() - moveStart) / 1000UL),
            trueMinProbe ? " TRUE_MIN_PASS" : "");

        return true;
      }

    } else {
      stable = 0;
    }

    next += QC_FAST_PERIOD_US;
  }

  reason = "MOVE_TIMEOUT";
  return false;
}

static String qcFindFreePath() {
  char path[40];

  for (int i = 0; i < 1000; ++i) {
    snprintf(
      path,
      sizeof(path),
      "/qc_fast_id32_%03d.bin",
      i);

    if (!FFat.exists(path)) {
      return String(path);
    }
  }

  return String();
}

static void qcPrintPhaseSummary(uint8_t phase) {
  uint32_t count = 0;

  int minPos = 99999, maxPos = -99999;
  int minSpeed = 99999, maxSpeed = -99999;
  int maxAbsLoad = 0;
  int maxAbsCurrent = 0;
  int minTemp = 99999, maxTemp = -99999;

  uint8_t protoOr = 0;
  uint8_t servoOr = 0;

  for (uint32_t i = 0; i < qcCount; ++i) {
    const QCFullSample &q = qcBuffer[i];

    if (q.phase != phase) continue;

    count++;

    int pos = qcPosition(q);
    int speed = qcSpeed(q);
    int load = qcLoad(q);
    int current = qcCurrent(q);
    int temp = qcTemperature(q);

    if (pos < minPos) minPos = pos;
    if (pos > maxPos) maxPos = pos;

    if (speed < minSpeed) minSpeed = speed;
    if (speed > maxSpeed) maxSpeed = speed;

    if (abs(load) > maxAbsLoad) maxAbsLoad = abs(load);
    if (abs(current) > maxAbsCurrent) maxAbsCurrent = abs(current);

    if (temp < minTemp) minTemp = temp;
    if (temp > maxTemp) maxTemp = temp;

    protoOr |= q.proto_status;
    servoOr |= (uint8_t)qcServoStatus(q);
  }

  if (count == 0) return;

  Serial.printf(
    "PHASE %u: n=%lu pos=%d..%d speed=%d..%d "
    "max|load|=%d max|current|=%d temp=%d..%d "
    "protoOR=0x%02X servoOR=0x%02X\n",
    phase,
    (unsigned long)count,
    minPos, maxPos,
    minSpeed, maxSpeed,
    maxAbsLoad,
    maxAbsCurrent,
    minTemp, maxTemp,
    protoOr,
    servoOr);
}

static bool qcSaveFile(
    int id,
    bool passed,
    const String &reason,
    String &savedPath) {

  savedPath = qcFindFreePath();

  if (savedPath.length() == 0) {
    return false;
  }

  File f = FFat.open(savedPath, FILE_WRITE);

  if (!f) {
    return false;
  }

  QCFileHeader h;
  memset(&h, 0, sizeof(h));

  memcpy(h.magic, "MATQC01", 7);
  h.version = QC_PROTOCOL_VERSION;
  h.servo_id = id;
  h.sample_hz = QC_FAST_HZ;
  h.sample_size = sizeof(QCFullSample);
  h.sample_count = qcCount;
  h.elapsed_us = (uint32_t)(micros() - qcStartUs);
  h.result_code = passed ? 0 : 1;

  reason.toCharArray(
      h.abort_reason,
      sizeof(h.abort_reason));

  size_t hw = f.write(
      reinterpret_cast<const uint8_t *>(&h),
      sizeof(h));

  size_t expectedRaw =
      (size_t)qcCount * sizeof(QCFullSample);

  size_t dw = f.write(
      reinterpret_cast<const uint8_t *>(qcBuffer),
      expectedRaw);

  f.flush();
  f.close();

  return hw == sizeof(h) && dw == expectedRaw;
}

static void runQCFast(int ID) {

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" MATDOG QC_FAST FULL-ANGLE — ID %d\n", ID);
  Serial.println("====================================");
  Serial.println("HARD MOTION LIMIT : 175 s + cleanup");
  Serial.println("ABSOLUTE LIMIT    : <180 s");
  Serial.println("QC PROTOCOL       : V6.1");
  Serial.println("QC BINARY VERSION : 61");
  Serial.println("RAW RATE          : 500 Hz");
  Serial.println("EEPROM WRITES     : NONE");
  Serial.println();

  String reason = "";
  bool passed = false;
  bool torqueWasEnabled = false;

  if (qcBuffer) {
    heap_caps_free(qcBuffer);
    qcBuffer = nullptr;
  }

  qcLastAvailable = false;
  qcLastPassed = false;
  qcLastElapsedUs = 0;
  qcLastServoId = -1;
  qcLastReason[0] = '\0';

  qcCount = 0;

  qcThermalTransientCount = 0;
  qcThermalConfirmedCount = 0;
  qcNextThermalCheckUs = 0;

  qcPerformanceEvents = 0;
  qcProtectiveStops = 0;
  qcNoMotionEvents = 0;
  qcNoProgressEvents = 0;
  qcOffTargetEvents = 0;
  qcWindowEndEvents = 0;
  qcSkippedTests = 0;

  qcMinAttempts = 0;
  qcMinResponses = 0;

  qcObservedMin = 4095;
  qcObservedMax = 0;
  qcUnsafeForMax = false;

  qcStartUs = micros();

  // ---------- PRECHECK ----------
  if (!psramFound()) {
    reason = "PSRAM_NOT_FOUND";
    goto cleanup;
  }

  qcBuffer = static_cast<QCFullSample *>(
      heap_caps_malloc(
          QC_FAST_MAX_SAMPLES * sizeof(QCFullSample),
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (!qcBuffer) {
    reason = "PSRAM_ALLOC_FAILED";
    goto cleanup;
  }

  Serial.printf("RAW SAMPLE SIZE   : %u bytes\n",
                (unsigned)sizeof(QCFullSample));
  Serial.printf("RAW BUFFER MAX    : %lu samples / %lu bytes\n",
                (unsigned long)QC_FAST_MAX_SAMPLES,
                (unsigned long)(
                    QC_FAST_MAX_SAMPLES *
                    sizeof(QCFullSample)));

  if (st.Ping((uint8_t)ID) < 0 || st.Error != 0) {
    reason = "PING_FAILED";
    goto cleanup;
  }

  {
    int mode = st.readByte(ID, SMS_STS_MODE);
    int torque = st.readByte(ID, SMS_STS_TORQUE_ENABLE);
    int minLimit = st.readWord(ID, SMS_STS_MIN_ANGLE_LIMIT_L);
    int maxLimit = st.readWord(ID, SMS_STS_MAX_ANGLE_LIMIT_L);

    Serial.printf("MODE             : %d\n", mode);
    Serial.printf("TORQUE PRE       : %d\n", torque);
    Serial.printf("LIMITS           : %d .. %d\n",
                  minLimit, maxLimit);

    if (mode != 0) {
      reason = "MODE_NOT_0";
      goto cleanup;
    }

    if (torque != 0) {
      reason = "TORQUE_NOT_OFF_AT_ENTRY";
      goto cleanup;
    }

    if (minLimit != 0 || maxLimit != 4095) {
      reason = "LIMITS_NOT_0_4095";
      goto cleanup;
    }
  }

  if (!qcStaticCapture(
        ID, 2000, QC_STATIC_PRE, reason)) {
    goto cleanup;
  }

  // ---------- PRIME ----------
  {
    QCFullSample q;

    if (!qcReadAndStore(
          ID, QC_PREPOSITION,
          0xFFFF, 0xFFFF, 0xFF, &q)) {
      reason = "PRIME_FEEDBACK_FAILED";
      goto cleanup;
    }

    int startPos = qcPosition(q);

    Serial.printf("START POSITION   : %d\n", startPos);
    Serial.printf("START TEMP       : %d\n", qcTemperature(q));
    Serial.printf("START VOLTAGE    : %d\n", qcVoltage(q));
    Serial.printf("START STATUS     : 0x%02X\n",
                  qcServoStatus(q));

    if (qcServoStatus(q) != 0) {
      reason = "SERVO_STATUS_NONZERO_AT_ENTRY";
      goto cleanup;
    }

    int prime = st.WritePosEx(
        (uint8_t)ID,
        (s16)startPos,
        300,
        20);

    delay(20);

    int primeTorque =
        st.readByte(ID, SMS_STS_TORQUE_ENABLE);

    Serial.printf("PRIME RESULT     : %d\n", prime);
    Serial.printf("PRIME TORQUE     : %d\n", primeTorque);

    if (!forceTorqueOffVerified(ID, "QC_PRIME")) {
      reason = "PRIME_SAFE_OFF_FAILED";
      goto cleanup;
    }

    if (prime < 0) {
      reason = "PRIME_WRITE_ACK_FAILED";
      goto cleanup;
    }
  }

  // Explicit, verified torque ON.
  if (st.EnableTorque((uint8_t)ID, 1) < 0) {
    reason = "TORQUE_ON_WRITE_FAILED";
    goto cleanup;
  }

  torqueWasEnabled = true;

  delay(20);

  if (st.readByte(ID, SMS_STS_TORQUE_ENABLE) != 1) {
    reason = "TORQUE_ON_NOT_CONFIRMED";
    goto cleanup;
  }

  Serial.println("TORQUE ON        : CONFIRMED");

  // ========================================================
  // QC PROTOCOL V4 — CHARACTERIZATION
  //
  // Poor performance is DATA.
  // Only unrecoverable safety/integrity conditions go to
  // global cleanup as an execution failure.
  // ========================================================

  Serial.println();
  Serial.println(
      "QC_MODE          : CHARACTERIZATION");
  Serial.println(
      "PERFORMANCE_FAIL : NEVER GLOBAL ABORT");

  Serial.println(
      "FEETECH C018 REF : 0.222s/60deg +/-10% @12V");
  Serial.println(
      "FEETECH C018 REF : I_no_load=180mA I_rated=900mA I_stall=2700mA");
  Serial.println(
      "FEETECH C018 REF : backlash<=0.5deg, voltage=4..14V");
  Serial.printf(
      "CURRENT RAW REF  : no_load~%d rated~%d overcurrent~%d stall~%d\n",
      QC_NOLOAD_CURRENT_RAW,
      QC_RATED_CURRENT_RAW,
      QC_OVERCURRENT_2A_RAW,
      QC_STALL_CURRENT_RAW);

  {
    QCMoveResult r;
    int current = 0;

    QCFullSample initialSample;

    if (!qcReadAndStore(
          ID,
          QC_PREPOSITION,
          0xFFFF,
          0xFFFF,
          0xFF,
          &initialSample)) {

      reason =
          "V4_INITIAL_POSITION_READ_FAILED";

      goto cleanup;
    }

    current =
        qcPosition(initialSample);

    qcObservedMin = current;
    qcObservedMax = current;

    // --------------------------------------------------------
    // RANGE DISCOVERY / PREPOSITION
    // --------------------------------------------------------
    Serial.println();
    Serial.println(
        "PHASE: RANGE DISCOVERY -> LOW END");

    if (!qcCharacterizeMove(
          ID,
          0,
          300,
          20,
          QC_PREPOSITION,
          0,
          400,
          reason,
          r)) {

      goto cleanup;
    }

    current = r.finalPos;

    // --------------------------------------------------------
    // SLOW FULL-RANGE UP
    // --------------------------------------------------------
    Serial.println();
    Serial.println(
        "PHASE: SLOW FULL RANGE UP");

    {
      const int targets[] = {
        512, 1024, 1536, 2048,
        2560, 3072, 3584, 4095
      };

      for (int target : targets) {

        if (target <= current + 8) {
          continue;
        }

        if (!qcCharacterizeMove(
              ID,
              target,
              128,
              10,
              QC_SLOW_UP,
              QC_SLOW_SEGMENT_WINDOW_MS,
              400,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        if (r.outcome == QCM_WINDOW_END || qcPathWasBlocked(r)) {
          Serial.printf(
              "SLOW_UP_PATH_LIMIT measured_at=%d "
              "outcome=%s\n",
              current,
              qcOutcomeName(r.outcome));

          break;
        }
      }
    }

    // --------------------------------------------------------
    // SLOW FULL-RANGE DOWN
    // --------------------------------------------------------
    Serial.println();
    Serial.println(
        "PHASE: SLOW FULL RANGE DOWN");

    {
      const int targets[] = {
        3584, 3072, 2560, 2048,
        1536, 1024, 512, 0
      };

      for (int target : targets) {

        if (target >= current - 8) {
          continue;
        }

        if (!qcCharacterizeMove(
              ID,
              target,
              128,
              10,
              QC_SLOW_DOWN,
              QC_SLOW_SEGMENT_WINDOW_MS,
              400,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        if (r.outcome == QCM_WINDOW_END || qcPathWasBlocked(r)) {
          Serial.printf(
              "SLOW_DOWN_PATH_LIMIT measured_at=%d "
              "outcome=%s\n",
              current,
              qcOutcomeName(r.outcome));

          break;
        }
      }
    }

    Serial.printf(
        "MEASURED_ENVELOPE after_slow=%d..%d "
        "coverage=%d ticks\n",
        qcObservedMin,
        qcObservedMax,
        qcObservedMax - qcObservedMin);

    // --------------------------------------------------------
    // STANDARDIZED 45-DEGREE MEDIUM-SPEED TEST
    //
    // This is also our real robot-like partial-motion test.
    // Each segment is ~45 degrees / 512 ticks.
    // --------------------------------------------------------
    Serial.println();
    Serial.println(
        "PHASE: 45-DEG MEDIUM STEP UP/DOWN");

    {
      const int mediumLow =
          qcObservedMin;

      const int mediumHigh =
          qcObservedMax;

      const int targetsUp[] = {
        512, 1024, 1536, 2048,
        2560, 3072, 3584, 4095
      };

      const int targetsDown[] = {
        3584, 3072, 2560, 2048,
        1536, 1024, 512, 0
      };

      Serial.printf(
          "STEP45_REFERENCE ticks=512 degrees~=45 "
          "safe_envelope=%d..%d\n",
          mediumLow,
          mediumHigh);

      // Move to the low end of the already measured envelope.
      if (!qcCharacterizeMove(
            ID,
            mediumLow,
            600,
            25,
            QC_TRANSFER,
            0,
            250,
            reason,
            r)) {

        goto cleanup;
      }

      current = r.finalPos;

      Serial.println("STEP45_DIRECTION: UP");

      for (int target : targetsUp) {

        if (target > mediumHigh + 3) {
          break;
        }

        if (target <= current + 8) {
          continue;
        }

        if (!qcCharacterizeMove(
              ID,
              target,
              600,
              25,
              QC_STEP45_MEDIUM,
              QC_MEDIUM_SEGMENT_WINDOW_MS,
              250,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        Serial.printf(
            "STEP45_RESULT dir=UP target=%d final=%d "
            "error=%d outcome=%s\n",
            target,
            r.finalPos,
            r.finalPos - target,
            qcOutcomeName(r.outcome));

        if (r.outcome == QCM_WINDOW_END || qcPathWasBlocked(r)) {
          break;
        }
      }

      Serial.println("STEP45_DIRECTION: DOWN");

      for (int target : targetsDown) {

        if (target < mediumLow - 3) {
          break;
        }

        if (target >= current - 8) {
          continue;
        }

        if (!qcCharacterizeMove(
              ID,
              target,
              600,
              25,
              QC_STEP45_MEDIUM,
              QC_MEDIUM_SEGMENT_WINDOW_MS,
              250,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        Serial.printf(
            "STEP45_RESULT dir=DOWN target=%d final=%d "
            "error=%d outcome=%s\n",
            target,
            r.finalPos,
            r.finalPos - target,
            qcOutcomeName(r.outcome));

        if (r.outcome == QCM_WINDOW_END || qcPathWasBlocked(r)) {
          break;
        }
      }
    }

    // --------------------------------------------------------
    // ADAPTIVE MINIMUM-SPEED CHARACTERIZATION
    //
    // We do NOT ask the servo to move just 3 ticks anymore.
    // The target is 64 ticks away to create meaningful
    // position error, while only a short observation window
    // is used.
    //
    // speed = 1,2,4,8 until the first measurable response.
    // No response is a valid result and does not abort.
    // --------------------------------------------------------
    Serial.println();
    Serial.println(
        "PHASE: ADAPTIVE MIN SPEED — 8 SECTORS");

    {
      const int zones[] = {
        256, 768, 1280, 1792,
        2304, 2816, 3328, 3840
      };

      const uint16_t probeSpeeds[] = {
        1, 2, 4, 8
      };

      const uint32_t probeWindowsMs[] = {
        900, 600, 400, 300
      };

      for (int zone : zones) {

        if ((uint32_t)(micros() - qcStartUs) >=
            QC_MIN_CONTINUE_LIMIT_US) {

          qcSkippedTests++;

          Serial.println(
              "MIN_REMAINDER_SKIPPED_TIME_BUDGET");

          break;
        }


        /*
         * Do not repeatedly drive a known mechanically
         * unreachable region. The slow sweep already measured
         * the accessible envelope.
         */
        if (zone < qcObservedMin + 96 ||
            zone > qcObservedMax - 96) {

          qcSkippedTests++;

          Serial.printf(
              "MIN_ZONE nominal=%d "
              "SKIPPED_UNREACHABLE envelope=%d..%d\n",
              zone,
              qcObservedMin,
              qcObservedMax);

          continue;
        }

        if (!qcCharacterizeMove(
              ID,
              zone,
              600,
              25,
              QC_TRANSFER,
              0,
              300,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        if (abs(current - zone) > 48 ||
            qcPathWasBlocked(r)) {

          qcSkippedTests++;

          Serial.printf(
              "MIN_ZONE nominal=%d "
              "SKIPPED_TRANSFER final=%d outcome=%s\n",
              zone,
              current,
              qcOutcomeName(r.outcome));

          continue;
        }

        Serial.printf(
            "MIN_ZONE nominal=%d actual=%d\n",
            zone,
            current);

        // Two real directions.
        for (int directionIndex = 0;
             directionIndex < 2;
             ++directionIndex) {

          const int direction =
              directionIndex == 0 ? +1 : -1;

          bool responseFound = false;

          for (int speedIndex = 0;
               speedIndex < 4;
               ++speedIndex) {

            const int probeTarget =
                current +
                direction * 64;

            if (probeTarget < 0 ||
                probeTarget > 4095) {

              qcSkippedTests++;
              break;
            }

            qcMinAttempts++;

            if (!qcCharacterizeMove(
                  ID,
                  probeTarget,
                  probeSpeeds[speedIndex],
                  1,
                  QC_MIN_PROBE,
                  probeWindowsMs[speedIndex],
                  probeWindowsMs[speedIndex],
                  reason,
                  r)) {

              goto cleanup;
            }

            current = r.finalPos;

            const int directionalTravel =
                direction > 0
                ? r.maxPos - r.startPos
                : r.startPos - r.minPos;

            Serial.printf(
                "MIN_RESULT zone=%d dir=%+d "
                "cmd_speed=%u travel=%d "
                "first_motion_ms=%.3f "
                "peak_present_speed=%d outcome=%s\n",
                zone,
                direction,
                probeSpeeds[speedIndex],
                directionalTravel,
                r.firstMotionSeen
                  ? r.firstMotionUs / 1000.0f
                  : -1.0f,
                r.peakAbsSpeed,
                qcOutcomeName(r.outcome));

            /*
             * Any real directional encoder displacement is
             * retained as a measured response.
             *
             * It is NOT a pass/fail threshold.
             */
            if (directionalTravel >= 1) {

              qcMinResponses++;

              Serial.printf(
                  "MIN_EFFECTIVE_COMMAND_CANDIDATE "
                  "zone=%d dir=%+d speed=%u travel=%d\n",
                  zone,
                  direction,
                  probeSpeeds[speedIndex],
                  directionalTravel);

              responseFound = true;
              break;
            }

            /*
             * If mechanical/current protection was needed,
             * do not keep increasing excitation in this same
             * direction. Recovery already occurred safely.
             */
            if (qcOutcomeNeedsRecovery(
                  r.outcome)) {

              break;
            }
          }

          if (!responseFound) {
            Serial.printf(
                "MIN_NO_RESPONSE zone=%d dir=%+d "
                "through_speed=8\n",
                zone,
                direction);
          }
        }
      }
    }

    // --------------------------------------------------------
    // MAX-SPEED RANGE TEST
    //
    // Healthy units command the real 0/4095 endpoints.
    // A mechanically limited unit uses the envelope measured
    // by the slow phase rather than repeatedly slamming into
    // an already discovered hard region.
    // --------------------------------------------------------
    Serial.println();
    Serial.println(
        "PHASE: MAX SPEED — BOTH DIRECTIONS");

    {
      int maxLow =
          qcObservedMin <= 16
          ? 0
          : qcObservedMin;

      int maxHigh =
          qcObservedMax >= 4079
          ? 4095
          : qcObservedMax;

      const int span =
          maxHigh - maxLow;

      Serial.printf(
          "MAX_TEST_ENVELOPE=%d..%d span=%d\n",
          maxLow,
          maxHigh,
          span);

      if ((uint32_t)(micros() - qcStartUs) >=
          QC_MAX_START_LATEST_US) {

        qcSkippedTests++;

        Serial.println(
            "MAX_FULL_RANGE SKIPPED_TIME_BUDGET");

      } else if (qcUnsafeForMax) {

        qcSkippedTests++;

        Serial.println(
            "MAX_FULL_RANGE SKIPPED_SAFETY: "
            "previous measured path/protective event");

      } else if (span < 256) {

        qcSkippedTests++;

        Serial.println(
            "MAX_FULL_RANGE SKIPPED: "
            "measured envelope too small");

      } else {

        if (!qcCharacterizeMove(
              ID,
              maxHigh,
              600,
              30,
              QC_TRANSFER,
              0,
              300,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        Serial.printf(
            "MAX_DIRECTION: %d -> %d\n",
            current,
            maxLow);

        if (!qcCharacterizeMove(
              ID,
              maxLow,
              0,
              50,
              QC_MAX_DOWN,
              QC_MAX_FULL_WINDOW_MS,
              300,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        Serial.printf(
            "MAX_DIRECTION: %d -> %d\n",
            current,
            maxHigh);

        if (!qcCharacterizeMove(
              ID,
              maxHigh,
              0,
              50,
              QC_MAX_UP,
              QC_MAX_FULL_WINDOW_MS,
              300,
              reason,
              r)) {

          goto cleanup;
        }

        current = r.finalPos;

        // ----------------------------------------------------
        // MAX-SPEED PRECISION / REVERSAL
        // ----------------------------------------------------
        Serial.println();
        Serial.println(
            "PHASE: MAX SPEED PRECISION / REVERSAL");

        const int zones[] = {
          3840, 3328, 2816, 2304,
          1792, 1280, 768, 256
        };

        for (int zone : zones) {

          if ((uint32_t)(micros() - qcStartUs) >=
              QC_PRECISION_STOP_US) {

            qcSkippedTests++;

            Serial.println(
                "PRECISION_REMAINDER_SKIPPED_TIME_BUDGET");

            break;
          }


          if (zone < maxLow + 64 ||
              zone > maxHigh - 64) {

            qcSkippedTests++;
            continue;
          }

          const int high =
              zone + 32;

          const int low =
              zone - 32;

          // Approach from above.
          if (!qcCharacterizeMove(
                ID,
                high,
                600,
                30,
                QC_TRANSFER,
                0,
                250,
                reason,
                r)) {

            goto cleanup;
          }

          current = r.finalPos;

          if (qcPathWasBlocked(r)) {
            qcSkippedTests++;

            if (qcOutcomeNeedsRecovery(
                  r.outcome)) {

              break;
            }

            continue;
          }

          if (!qcCharacterizeMove(
                ID,
                zone,
                0,
                50,
                QC_MAX_PRECISION,
                0,
                200,
                reason,
                r)) {

            goto cleanup;
          }

          current = r.finalPos;

          // Approach same target from below.
          if (!qcCharacterizeMove(
                ID,
                low,
                600,
                30,
                QC_TRANSFER,
                0,
                250,
                reason,
                r)) {

            goto cleanup;
          }

          current = r.finalPos;

          if (qcPathWasBlocked(r)) {
            qcSkippedTests++;

            if (qcOutcomeNeedsRecovery(
                  r.outcome)) {

              break;
            }

            continue;
          }

          if (!qcCharacterizeMove(
                ID,
                zone,
                0,
                50,
                QC_MAX_PRECISION,
                0,
                200,
                reason,
                r)) {

            goto cleanup;
          }

          current = r.finalPos;
        }
      }
    }
  }

  passed = true;

cleanup:

  // Mandatory fail-closed cleanup after any possible control write.
  Serial.println();
  Serial.println("===== QC CLEANUP =====");

  if (!forceTorqueOffVerified(ID, "QC_FINAL")) {
    Serial.println("CRITICAL: TORQUE OFF NOT VERIFIED");
    Serial.println("CUT SERVO POWER NOW");
    passed = false;

    if (reason.length() == 0) {
      reason = "FINAL_TORQUE_OFF_FAILED";
    }
  }

  // Post-test static telemetry only if communication still works.
  if (qcBuffer && st.Ping((uint8_t)ID) >= 0) {
    String postReason;

    if (!qcStaticCapture(
          ID, 1000, QC_STATIC_POST, postReason)) {

      passed = false;

      if (reason.length() == 0) {
        reason = "POST_STATIC_" + postReason;
      }
    }
  }

  uint32_t elapsed =
      (uint32_t)(micros() - qcStartUs);

  Serial.println();
  Serial.println("===== QC SUMMARY =====");
  Serial.printf("RESULT            : %s\n",
                passed
                ? "EXECUTION_COMPLETE"
                : "SAFETY_OR_INTEGRITY_ABORT");
  Serial.printf("REASON            : %s\n",
                reason.length() ? reason.c_str() : "NONE");
  Serial.printf("ELAPSED_US        : %lu\n",
                (unsigned long)elapsed);
  Serial.printf("RAW_SAMPLES       : %lu\n",
                (unsigned long)qcCount);
  Serial.printf("RAW_BYTES         : %lu\n",
                (unsigned long)(
                    qcCount * sizeof(QCFullSample)));

  Serial.printf("THERMAL_TRANSIENTS: %lu\n",
                (unsigned long)qcThermalTransientCount);
  Serial.printf("THERMAL_CONFIRMED : %lu\n",
                (unsigned long)qcThermalConfirmedCount);

  Serial.printf("PERFORMANCE_EVENTS: %lu\n",
                (unsigned long)qcPerformanceEvents);

  Serial.printf("PROTECTIVE_STOPS  : %lu\n",
                (unsigned long)qcProtectiveStops);

  Serial.printf("NO_MOTION_EVENTS  : %lu\n",
                (unsigned long)qcNoMotionEvents);

  Serial.printf("NO_PROGRESS_EVENTS: %lu\n",
                (unsigned long)qcNoProgressEvents);

  Serial.printf("SETTLED_RESIDUALS : %lu\n",
                (unsigned long)qcOffTargetEvents);

  Serial.printf("WINDOW_END_EVENTS : %lu\n",
                (unsigned long)qcWindowEndEvents);

  Serial.printf("SKIPPED_TESTS     : %lu\n",
                (unsigned long)qcSkippedTests);

  Serial.printf("MIN_ATTEMPTS      : %lu\n",
                (unsigned long)qcMinAttempts);

  Serial.printf("MIN_RESPONSES     : %lu\n",
                (unsigned long)qcMinResponses);

  Serial.printf("OBSERVED_ENVELOPE : %d .. %d\n",
                qcObservedMin,
                qcObservedMax);

  if (qcBuffer) {
    for (uint8_t phase = QC_STATIC_PRE;
         phase <= QC_STEP45_MEDIUM;
         ++phase) {
      qcPrintPhaseSummary(phase);
    }
  }

  Serial.println();
  Serial.println("RAW_STORAGE        : PSRAM_RETAINED");
  Serial.printf("RAW_RETAINED_BYTES : %lu\n",
                (unsigned long)(
                    qcCount * sizeof(QCFullSample)));

  int finalTorque =
      st.readByte(ID, SMS_STS_TORQUE_ENABLE);

  Serial.printf("FINAL_TORQUE      : %d\n", finalTorque);
  Serial.println("EEPROM_WRITES     : NONE");

  qcLastAvailable =
      (qcBuffer != nullptr && qcCount > 0);

  qcLastPassed =
      passed &&
      finalTorque == 0 &&
      elapsed < 180000000UL;

  qcLastElapsedUs = elapsed;
  qcLastServoId = ID;

  reason.toCharArray(
      qcLastReason,
      sizeof(qcLastReason));

  if (qcLastPassed && qcLastAvailable) {
    Serial.println("QC_EXECUTION      : COMPLETE");
  } else {
    Serial.println("QC_EXECUTION      : SAFETY_ABORT");
  }

  Serial.printf("RAW_AVAILABLE     : %s\n",
                qcLastAvailable ? "YES" : "NO");

  Serial.println("QC_FAST_COMPLETE");
}


static uint32_t qcRawFnv1a() {
  uint32_t h = 2166136261u;

  const uint8_t *p =
      reinterpret_cast<const uint8_t *>(qcBuffer);

  size_t n =
      (size_t)qcCount * sizeof(QCFullSample);

  for (size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 16777619u;
  }

  return h;
}

static bool qcSerialWriteAll(
    const uint8_t *data,
    size_t bytes) {

  size_t sent = 0;

  while (sent < bytes) {
    size_t chunk = bytes - sent;

    if (chunk > 4096) {
      chunk = 4096;
    }

    size_t n = Serial.write(
        data + sent,
        chunk);

    if (n == 0) {
      delay(1);
      continue;
    }

    sent += n;
  }

  return true;
}

static void runDumpRaw(int id) {
  if (!qcLastAvailable ||
      !qcBuffer ||
      qcLastServoId != id) {

    Serial.println("RAW_DUMP_ABORT: NO_DATASET");
    return;
  }

  // Absolutely no motion is allowed during dump.
  int torque =
      st.readByte(id, SMS_STS_TORQUE_ENABLE);

  if (torque != 0) {
    Serial.printf(
      "RAW_DUMP_ABORT: TORQUE=%d NOT_OFF\n",
      torque);
    return;
  }

  QCFileHeader h;
  memset(&h, 0, sizeof(h));

  memcpy(h.magic, "MATQC01", 7);
  h.version = QC_PROTOCOL_VERSION;
  h.servo_id = id;
  h.sample_hz = QC_FAST_HZ;
  h.sample_size = sizeof(QCFullSample);
  h.sample_count = qcCount;
  h.elapsed_us = qcLastElapsedUs;
  h.result_code = qcLastPassed ? 0 : 1;

  strncpy(
      h.abort_reason,
      qcLastReason,
      sizeof(h.abort_reason) - 1);

  const size_t rawBytes =
      (size_t)qcCount *
      sizeof(QCFullSample);

  const size_t payloadBytes =
      sizeof(QCFileHeader) + rawBytes;

  uint32_t fnv = qcRawFnv1a();

  Serial.printf(
    "RAW_DUMP_BEGIN payload=%lu header=%u samples=%lu sample_size=%u raw_fnv=0x%08lX\n",
    (unsigned long)payloadBytes,
    (unsigned)sizeof(QCFileHeader),
    (unsigned long)qcCount,
    (unsigned)sizeof(QCFullSample),
    (unsigned long)fnv);

  Serial.flush();

  qcSerialWriteAll(
      reinterpret_cast<const uint8_t *>(&h),
      sizeof(h));

  qcSerialWriteAll(
      reinterpret_cast<const uint8_t *>(qcBuffer),
      rawBytes);

  Serial.flush();

  Serial.println();
  Serial.println("RAW_DUMP_END");
}


bool parseScanCommand(
    const String &cmd,
    int &minID,
    int &maxID) {

  const String prefix = "@SCAN ";

  if (!cmd.startsWith(prefix)) {
    return false;
  }

  String args = cmd.substring(prefix.length());
  args.trim();

  int sep = args.indexOf(' ');

  if (sep <= 0) {
    return false;
  }

  if (args.indexOf(' ', sep + 1) >= 0) {
    return false;
  }

  String a = args.substring(0, sep);
  String b = args.substring(sep + 1);

  int lo = -1;
  int hi = -1;

  if (!parseUnsignedStrict(a, 0, 253, lo)) {
    return false;
  }

  if (!parseUnsignedStrict(b, 0, 253, hi)) {
    return false;
  }

  if (lo > hi) {
    return false;
  }

  minID = lo;
  maxID = hi;
  return true;
}

void runReadOnlyScan(
    int minID,
    int maxID) {

  Serial.println();
  Serial.println("====================================");
  Serial.printf(
      " READ-ONLY SERVO SCAN %d..%d\n",
      minID,
      maxID);
  Serial.println("====================================");

  int found = 0;

  for (int id = minID; id <= maxID; ++id) {

    int ping = st.Ping((uint8_t)id);

    if (ping < 0) {
      continue;
    }

    int model =
        st.readWord(id, SMS_STS_MODEL_L);

    int torque =
        st.readByte(id, SMS_STS_TORQUE_ENABLE);

    int mode =
        st.readByte(id, SMS_STS_MODE);

    int pos =
        st.ReadPos(id);

    Serial.printf(
        "FOUND ID=%d MODEL=%d STATUS=0x%02X "
        "MODE=%d TORQUE=%d POSITION=%d\n",
        id,
        model,
        st.Error,
        mode,
        torque,
        pos);

    found++;
  }

  Serial.printf("SCAN_FOUND       : %d\n", found);
  Serial.println("SCAN_COMPLETE");
  Serial.println("NO WRITES PERFORMED");
  Serial.println("NO MOTION COMMANDS SENT");
}

void loop() {

  if (!Serial.available()) {
    delay(2);
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  /*
   * Fail-closed transport rule:
   * only lines explicitly beginning with '@' are host commands.
   *
   * This intentionally discards echoed firmware output, terminal noise,
   * boot banners and partial lines.
   */
  if (!cmd.startsWith("@")) {
    return;
  }

  if (cmd == "@HELP") {
    printHelp();
    return;
  }

  int safeID = -1;

  if (parseIDCommand(cmd, "SAFE_OFF", safeID)) {
    runSafeOff(safeID);
    return;
  }

  int normalizeID = -1;

  if (parseIDCommand(cmd, "NORMALIZE_MATDOG", normalizeID)) {
    runNormalizeMatdog(normalizeID);
    return;
  }

  int ramID = -1;

  if (parseIDCommand(cmd, "RAMTEST", ramID)) {
    runRamRecorderTest(ramID);
    return;
  }

  int qcID = -1;

  if (parseIDCommand(cmd, "QC_FAST", qcID)) {
    runQCFast(qcID);
    return;
  }

  int dumpID = -1;

  if (parseIDCommand(cmd, "DUMP_RAW", dumpID)) {
    runDumpRaw(dumpID);
    return;
  }

  int benchID = -1;
  int benchDurationMs = -1;

  if (parseBenchCommand(cmd, benchID, benchDurationMs)) {
    benchmarkServo(benchID, benchDurationMs);
    return;
  }

  int captureID = -1;
  int captureDurationMs = -1;
  int captureHz = -1;

  if (parseCaptureCommand(
        cmd,
        captureID,
        captureDurationMs,
        captureHz)) {

    captureFixedRate(
      captureID,
      captureDurationMs,
      captureHz
    );

    return;
  }

  int scanMin = -1;
  int scanMax = -1;

  if (parseScanCommand(cmd, scanMin, scanMax)) {
    runReadOnlyScan(scanMin, scanMax);
    return;
  }

  int id = -1;

  if (parseIDCommand(cmd, "PING", id)) {

    Serial.printf("PING ID %d...\n", id);

    int result = st.Ping((uint8_t)id);

    if (result >= 0) {
      Serial.printf("PING_OK ID=%d STATUS=0x%02X\n",
                    result, st.Error);
    } else {
      Serial.printf("PING_TIMEOUT ID=%d\n", id);
    }

    return;
  }

  if (parseIDCommand(cmd, "READ", id)) {
    readServo(id);
    return;
  }

  Serial.printf("ERROR: INVALID_COMMAND '%s'\n", cmd.c_str());
}

