#include <Arduino.h>
#include <SCServo.h>

static constexpr int SERVO_TX_PIN = 17;
static constexpr int SERVO_RX_PIN = 18;
static constexpr uint32_t SERVO_BAUD = 1000000;

static constexpr uint8_t SNAPSHOT_START = 0x00;
static constexpr uint8_t SNAPSHOT_LEN = 71;   // 0x00 .. 0x46 inclusive
static constexpr uint8_t REG_RESPONSE_STATUS = 0x08;
static constexpr uint8_t REG_TORQUE_ENABLE = 0x28;

HardwareSerial ServoUART(1);
SMS_STS st;

// SAFE_OFF is deliberately gated by a successful immediately-prior
// full snapshot of the same servo. This guarantees that the source
// ResponseStatus was observed before the only write this firmware permits.
bool snapshotGateValid = false;
uint8_t snapshotGateId = 0;
uint8_t snapshotGateResponseStatus = 0;

static bool validId(int id) {
  return id >= 0 && id <= 253;
}

static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t i16le(const uint8_t *p) {
  return (int16_t)u16le(p);
}

static void invalidateSnapshotGate() {
  snapshotGateValid = false;
  snapshotGateId = 0;
  snapshotGateResponseStatus = 0;
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  @SCAN <min_id> <max_id>");
  Serial.println("  @SNAPSHOT71 <id>");
  Serial.println("  @SAFE_OFF <id>");
  Serial.println("  @HELP");
  Serial.println();
}

static void runScan(int minId, int maxId) {
  invalidateSnapshotGate();

  Serial.println();
  Serial.println("====================================");
  Serial.printf(" READ-ONLY SCAN %d..%d\n", minId, maxId);
  Serial.println("====================================");

  int found = 0;

  for (int id = minId; id <= maxId; ++id) {
    int ping = st.Ping((uint8_t)id);

    if (ping < 0) {
      continue;
    }

    uint8_t pingStatus = st.Error;

    int model = st.readWord((uint8_t)id, SMS_STS_MODEL_L);
    uint8_t modelStatus = st.Error;

    Serial.printf(
        "FOUND ID=%d MODEL=%d PING_STATUS=0x%02X MODEL_STATUS=0x%02X\n",
        id,
        model,
        pingStatus,
        modelStatus);

    ++found;
  }

  Serial.printf("SCAN_RESULT FOUND=%d\n", found);
  Serial.println("SCAN_COMPLETE");
}

static bool readSnapshot71(uint8_t id, uint8_t *raw) {
  int ping = st.Ping(id);

  if (ping != id || st.Error != 0) {
    Serial.printf(
        "SNAPSHOT71_ABORT PING=%d STATUS=0x%02X\n",
        ping,
        st.Error);
    return false;
  }

  int n = st.Read(id, SNAPSHOT_START, raw, SNAPSHOT_LEN);
  uint8_t readStatus = st.Error;

  if (n != SNAPSHOT_LEN || readStatus != 0) {
    Serial.printf(
        "SNAPSHOT71_ABORT READ_LEN=%d STATUS=0x%02X\n",
        n,
        readStatus);
    return false;
  }

  return true;
}

static void runSnapshot71(int id) {
  invalidateSnapshotGate();

  uint8_t raw[SNAPSHOT_LEN];

  if (!readSnapshot71((uint8_t)id, raw)) {
    return;
  }

  // Raw bytes are emitted BEFORE any decoding.
  Serial.printf(
      "SNAPSHOT71_BEGIN ID=%d START=0x00 LEN=%u STATUS=0x%02X\n",
      id,
      (unsigned)SNAPSHOT_LEN,
      st.Error);

  Serial.print("RAW71_HEX=");
  for (uint8_t i = 0; i < SNAPSHOT_LEN; ++i) {
    Serial.printf("%02X", raw[i]);
  }
  Serial.println();

  Serial.println("SNAPSHOT71_RAW_COMPLETE");

  const uint16_t model = u16le(raw + 0x03);
  const uint8_t storedId = raw[0x05];
  const uint8_t baud = raw[0x06];
  const uint8_t responseStatus = raw[0x08];
  const int16_t offset = i16le(raw + 0x1F);
  const uint8_t torque = raw[0x28];
  const uint16_t torqueLimit = u16le(raw + 0x30);
  const uint8_t lock = raw[0x37];
  const uint16_t presentPosition = u16le(raw + 0x38);

  Serial.printf("MODEL_0x03=%u\n", model);
  Serial.printf("ID_REGISTER=%u\n", storedId);
  Serial.printf("BAUD_REGISTER=%u\n", baud);
  Serial.printf("RESPONSE_STATUS=%u\n", responseStatus);
  Serial.printf("POSITION_OFFSET_SIGNED=%d\n", offset);
  Serial.printf("TORQUE_ENABLE=%u\n", torque);
  Serial.printf("TORQUE_LIMIT=%u\n", torqueLimit);
  Serial.printf("EEPROM_LOCK=%u\n", lock);
  Serial.printf("PRESENT_POSITION_RAW16=%u\n", presentPosition);

  snapshotGateValid = true;
  snapshotGateId = (uint8_t)id;
  snapshotGateResponseStatus = responseStatus;

  Serial.println("SNAPSHOT71_RESULT PASS");
  Serial.println("SNAPSHOT71_END");
}

static void runSafeOff(int id) {
  Serial.println();
  Serial.println("====================================");
  Serial.printf(" VERIFIED TORQUE OFF — ID %d\n", id);
  Serial.println("====================================");

  if (!snapshotGateValid || snapshotGateId != (uint8_t)id) {
    Serial.println("SAFE_OFF_ABORT: SNAPSHOT71_GATE_REQUIRED");
    return;
  }

  if (snapshotGateResponseStatus != 1) {
    Serial.printf(
        "SAFE_OFF_ABORT: RESPONSE_STATUS=%u EXPECTED=1\n",
        snapshotGateResponseStatus);
    invalidateSnapshotGate();
    return;
  }

  // Single-use gate.
  invalidateSnapshotGate();

  int wr = st.EnableTorque((uint8_t)id, 0);
  uint8_t writeStatus = st.Error;

  delay(20);

  int rb = st.readByte((uint8_t)id, REG_TORQUE_ENABLE);
  uint8_t readStatus = st.Error;

  Serial.printf("SAFE_OFF_WRITE_RESULT=%d\n", wr);
  Serial.printf("SAFE_OFF_WRITE_STATUS=0x%02X\n", writeStatus);
  Serial.printf("SAFE_OFF_READBACK=%d\n", rb);
  Serial.printf("SAFE_OFF_READ_STATUS=0x%02X\n", readStatus);

  if (wr != 1 ||
      writeStatus != 0 ||
      rb != 0 ||
      readStatus != 0) {
    Serial.println("SAFE_OFF_RESULT FAIL");
    Serial.println("CUT_SERVO_POWER_NOW");
    return;
  }

  Serial.println("SAFE_OFF_RESULT PASS");
}

static bool parseOneId(const String &cmd, const char *verb, int &id) {
  String pattern = "@";
  pattern += verb;

  int parsed = -1;
  char extra = '\0';

  int count = sscanf(
      cmd.c_str(),
      (pattern + " %d %c").c_str(),
      &parsed,
      &extra);

  if (count != 1 || !validId(parsed)) {
    return false;
  }

  id = parsed;
  return true;
}

static bool parseScan(const String &cmd, int &lo, int &hi) {
  int parsedLo = -1;
  int parsedHi = -1;
  char extra = '\0';

  int count = sscanf(
      cmd.c_str(),
      "@SCAN %d %d %c",
      &parsedLo,
      &parsedHi,
      &extra);

  if (count != 2 ||
      !validId(parsedLo) ||
      !validId(parsedHi) ||
      parsedLo > parsedHi) {
    return false;
  }

  lo = parsedLo;
  hi = parsedHi;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(1500);

  ServoUART.begin(
      SERVO_BAUD,
      SERIAL_8N1,
      SERVO_RX_PIN,
      SERVO_TX_PIN);

  st.pSerial = &ServoUART;

  Serial.println();
  Serial.println("====================================");
  Serial.println(" MATDOG ST3215 SOURCE SURVEY V1");
  Serial.println("====================================");
  Serial.printf("Servo UART       : %lu baud\n", SERVO_BAUD);
  Serial.printf("TX               : GPIO%d\n", SERVO_TX_PIN);
  Serial.printf("RX               : GPIO%d\n", SERVO_RX_PIN);
  Serial.println();
  Serial.println("Startup motion   : IMPOSSIBLE BY DESIGN");
  Serial.println("Torque ON        : NOT IMPLEMENTED");
  Serial.println("Position writes  : NOT IMPLEMENTED");
  Serial.println("EEPROM unlock    : NOT IMPLEMENTED");
  Serial.println("EEPROM writes    : NOT IMPLEMENTED");
  Serial.println("ID change        : NOT IMPLEMENTED");
  Serial.println("CalibrationOfs   : NOT IMPLEMENTED");
  Serial.println("Broadcast writes : NOT IMPLEMENTED");
  Serial.println("Allowed write    : TorqueEnable=0 ONLY");
  Serial.println();
  Serial.println("SOURCE_SURVEY_READY");

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
    invalidateSnapshotGate();
    printHelp();
    return;
  }

  int lo = -1;
  int hi = -1;

  if (parseScan(cmd, lo, hi)) {
    runScan(lo, hi);
    return;
  }

  int id = -1;

  if (parseOneId(cmd, "SNAPSHOT71", id)) {
    runSnapshot71(id);
    return;
  }

  if (parseOneId(cmd, "SAFE_OFF", id)) {
    runSafeOff(id);
    return;
  }

  invalidateSnapshotGate();
  Serial.println("ERROR: INVALID_COMMAND");
  printHelp();
}
