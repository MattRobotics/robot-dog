/*
 * MATDOG FULL LEG CALIBRATOR V1 — ESP32-S3 firmware
 *
 * The first current-architecture, Station-free, ESP32-S3-native calibration
 * engine for the 12 MATDOG leg servos.
 *
 * Host    : matdog_full_leg_calibrator_runner.py
 * Profile : MATDOG_C018_V1  (read and verified, NEVER written)
 * Bus     : UART1, GPIO17 TX / GPIO18 RX, 1 Mbps, Seeed Bus Servo Driver
 *
 * ARCHITECTURE
 *   ASUS host runner  <-- USB CDC -->  THIS FIRMWARE  <-- UART -->  12 servos
 *   NormaCore Station is NOT in the control path and is not a dependency.
 *   The host never streams raw GoalPosition. The host asks for a calibration
 *   operation; this firmware owns every bus write and every motion safety
 *   decision.
 *
 * EEPROM-WRITE-FREE BY DESIGN
 *   writeAllowed() below is an address allowlist. The ONLY writable addresses
 *   are TorqueEnable (0x28) and TorqueLimit (0x30), both RAM, plus the RAM
 *   block written by WritePosEx (Acc/GoalPosition/GoalTime/GoalSpeed).
 *   Every EEPROM address — including Lock (0x37), PositionOffset (0x1F) and
 *   ID (0x05) — falls through to `return false`. unLockEprom(), LockEprom()
 *   and CalibrationOfs() are never called from this translation unit.
 *   CalibrationOfs() is writeByte(ID, 0x28, 128), so value 128 at 0x28 is
 *   refused explicitly rather than by omission.
 *
 * ONE GOALPOSITION AUTHORITY
 *   flcWritePosEx() is the single function in this firmware that can move a
 *   servo. There is no other call site of WritePosEx / RegWritePosEx /
 *   SyncWritePosEx / WriteSpe / WheelMode.
 *
 * NO BROADCAST
 *   Every write takes a concrete unicast id validated against the census.
 *   Broadcast id 254 is refused by validLegId().
 *
 * PROVENANCE OF CONSTANTS
 *   Every safety-relevant constant below carries one of four tags:
 *     [VALIDATED]        current-hardware invariant, evidence cited
 *     [GENERIC]          reusable mechanism from frozen MATDOG ESP32 tooling
 *     [HISTORICAL]       LF V25 candidate, PREVIOUS installation, not current truth
 *     [CHARACTERIZE]     no current evidence — hard-blocks motion
 *   Any [CHARACTERIZE] constant that is still unresolved makes
 *   characterizationOutstanding() true, which blocks every motion mode.
 *
 * HISTORICAL LF V25 IS AN ALGORITHMIC ORACLE ONLY
 *   LF V25 numeric hardware results describe an installation that no longer
 *   exists: all 12 leg servos were removed, bench-provisioned to
 *   PositionOffset = 0 and remounted, and the HIP_MAX endstop carrier was
 *   redesigned and remounted. Additionally the pre-recenter backup
 *   09_Logs/Calibration/C5_R_digital_recenter/2026-07-10_132236Z_st3215_pre_recenter_backup.json
 *   shows the fleet was NOT in a homogeneous runtime torque state: old bus ids
 *   13 and 23 held runtime TorqueLimit 500 while the other ten held 1000.
 *   No LF V25 number may authorize current hardware.
 */

#include <Arduino.h>
#include <SCServo.h>
#include <esp_system.h>

// Build-stage authorization. Single source of truth, fail-closed by default.
#include "flc_stage_config.h"

// The bounded contact/endpoint state machine. Pure C++, no Arduino dependency,
// so tests/flc_detector_harness.cpp compiles the identical logic on the host and
// the offline fault-injection suite exercises the real engine.
#include "flc_contact_detector.h"

// The H3/H4 calibration state machine: baseline, probe, contact, retreat,
// re-approach, repeatability, abort/recovery. Also pure C++ and also compiled
// by the host harness, so the firmware and the fault-injection tests run the
// SAME motion decision logic. There is no second implementation.
#include "flc_calibration_engine.h"

// Generated, explicit Geometry Compiler V5 endpoint/parking evidence and the
// dependency-aware H5/H6 plan shared with the native harness.
#include "flc_leg_plan.h"

// Declared here, ahead of every function, because the Arduino builder inserts
// generated prototypes immediately after the includes. A function taking or
// returning one of these types would otherwise be prototyped before the type
// exists. Their definitions stay minimal; the tables that use them live in
// their own sections below.

struct WriteOutcome {
  bool allowed;
  int ack;
  uint8_t writeStatus;
  int readback;
  uint8_t readStatus;
  bool ok;
};

//: One row of the 12-joint table. See the JOINT SPEC TABLE section for the data
//: and for why direction is deliberately left unmeasured.
struct JointSpec {
  uint8_t busId;
  const char *jointName;
  const char *unitLabel;
  uint8_t leg;
  uint8_t kind;
  float declaredMinRad;
  float declaredMaxRad;
  float geomContactMinRad;
  float geomContactMaxRad;
};

//: Per-joint census result for the CURRENT physical session.
struct CensusEntry {
  bool present;
  bool identityOk;
  uint16_t model;
  int16_t positionOffset;
  uint8_t torqueEnable;
  uint16_t torqueLimit;
  uint16_t presentPosition;
  uint8_t voltage;
  uint8_t temperature;
  uint8_t statusByte;
  uint8_t responseStatus;
  bool profileMatch;
};

//: What H3 measured about one joint, valid only for the census epoch that was
//: current when it was measured.
struct JointCharacterization {
  bool valid;
  uint32_t sessionGeneration;
  uint32_t censusEpoch;
  int8_t rawProbeSign;
  FlcBaseline baseline;
  uint16_t contactThresholdRaw;
  uint16_t retreatTicks;
  uint16_t repeatabilityToleranceTicks;
  uint16_t observedSpreadTicks;
  int restTick;
  int origin;
};

// H2 evidence is kept in RAM and bound to one boot/session/census generation.
// Sample spread proves encoder stability only. It is deliberately NOT treated
// as a characterized physical-pose accuracy tolerance.
struct ManualQ0Evidence {
  bool valid;
  uint32_t sessionGeneration;
  uint32_t censusEpoch;
  int centreTick;
  int minTick;
  int maxTick;
  int spreadTicks;
  uint16_t samples;
};

// Explicit semantic operator witness. The number is the MATDOG kinematic
// mapping in q = direction * signed_tick_delta(raw, q0), never the tautological
// fact that a position servo follows a larger raw target with a larger raw
// encoder value.
struct JointDirectionWitness {
  bool valid;
  uint32_t sessionGeneration;
  uint32_t censusEpoch;
  int8_t direction;
};

// Candidate calibration produced in this RAM session. It is sufficient to
// express a geometry parking angle in raw ticks, but is never promoted to the
// canonical calibration by this firmware.
struct JointCalibrationEvidence {
  bool valid;
  uint32_t sessionGeneration;
  uint32_t censusEpoch;
  int8_t direction;
  int derivedQ0Tick;
  int minContactTick;
  int maxContactTick;
  int tier;
};

// ==========================================================================
// Transport — identical to the frozen survey/provisioner firmware
// 05_Firmware/ST3215_Bench_Tools/Source_Signature_Survey_V1/
// ==========================================================================

static constexpr int SERVO_TX_PIN = 17;   // [VALIDATED] ARCHITECTURE.md
static constexpr int SERVO_RX_PIN = 18;   // [VALIDATED] ARCHITECTURE.md
static constexpr uint32_t SERVO_BAUD = 1000000;  // [VALIDATED] C018 profile baud 0

HardwareSerial ServoUART(1);
SMS_STS st;

static const char *FIRMWARE_NAME = "matdog_full_leg_calibrator_v1";
static const char *FIRMWARE_VERSION = "1.1.0";
static const char *PROTOCOL_ID = "FLC1";
static const char *PROTOCOL_SCOPE = "CALIBRATOR_LOCAL_NOT_FINAL_RUNTIME_PROTOCOL";
static const char *PROFILE_ID = "MATDOG_C018_V1";

// ==========================================================================
// Register map — authoritative addresses, verified against the C018 profile
// 06_Software/Matdog_Core/config/MATDOG_ST3215_C018_V1.yaml
// ==========================================================================

static constexpr uint8_t SNAPSHOT_START = 0x00;
static constexpr uint8_t SNAPSHOT_LEN = 71;   // 0x00 .. 0x46 inclusive

static constexpr uint8_t REG_MODEL = 0x03;            // 2 bytes, NOT 0x00
static constexpr uint8_t REG_ID = 0x05;
static constexpr uint8_t REG_BAUD = 0x06;
static constexpr uint8_t REG_RESPONSE_STATUS = 0x08;
static constexpr uint8_t REG_POSITION_OFFSET = 0x1F;  // 2 bytes, EEPROM
static constexpr uint8_t REG_TORQUE_ENABLE = 0x28;
static constexpr uint8_t REG_GOAL_POSITION = 0x2A;    // 2 bytes, RAM (read back)
static constexpr uint8_t REG_TORQUE_LIMIT = 0x30;     // 2 bytes, RAM
static constexpr uint8_t REG_LOCK = 0x37;             // EEPROM lock
static constexpr uint8_t REG_PRESENT_POSITION = 0x38; // 2 bytes
static constexpr uint8_t REG_PRESENT_SPEED = 0x3A;    // 2 bytes
static constexpr uint8_t REG_PRESENT_LOAD = 0x3C;     // 2 bytes
static constexpr uint8_t REG_PRESENT_VOLTAGE = 0x3E;
static constexpr uint8_t REG_PRESENT_TEMPERATURE = 0x3F;
static constexpr uint8_t REG_STATUS = 0x40;
static constexpr uint8_t REG_PRESENT_CURRENT = 0x45;  // 2 bytes

static constexpr uint16_t EXPECTED_MODEL = 777;          // [VALIDATED] 17/17 units
static constexpr uint8_t EXPECTED_RESPONSE_STATUS = 1;   // [VALIDATED] 17/17 units
static constexpr uint8_t EXPECTED_BAUD = 0;              // [VALIDATED] 1 Mbps
static constexpr int16_t EXPECTED_POSITION_OFFSET = 0;   // [VALIDATED] provisioning campaign

// The 20 persistent-profile registers. Read and compared, NEVER written.
// [VALIDATED] fingerprint ae3ae5ce6ddda1c003fcde9f8ce12fed7f1272639b183e1ba23e27eae0cc5b4d
struct ProfileReg {
  uint8_t addr;
  uint8_t width;
  uint16_t value;
  const char *name;
};

static constexpr ProfileReg PROFILE[] = {
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

// ==========================================================================
// Encoder domain — unsigned 0..4095, no signed wrap trick anywhere
// ==========================================================================

// Domain constants and the tick math live in flc_contact_detector.h so the
// firmware and the host harness cannot disagree about the encoder domain.
static constexpr int ENCODER_MODULUS = FLC_ENCODER_MODULUS;
static constexpr int ENCODER_MAX = FLC_ENCODER_MAX;
static constexpr int RAW_ELECTRICAL_CENTER = 2048;   // servo centre, NOT q0

static inline int signedTickDelta(int presentTick, int referenceTick) {
  return flcSignedTickDelta(presentTick, referenceTick);
}

// ==========================================================================
// HARDWARE VALIDATION STAGE GATE — H0..H7
//
// Build the FULL calibrator now; unlock hardware progressively.
// This firmware ships at H0. Raising it is a deliberate source change made
// only after the evidence for the next stage exists.
// ==========================================================================

enum HardwareStage : uint8_t {
  H0_ESP32_ONLY = FLC_STAGE_H0_ESP32_ONLY,
  H1_CENSUS_READONLY = FLC_STAGE_H1_CENSUS_READONLY,
  H2_MANUAL_Q0 = FLC_STAGE_H2_MANUAL_Q0,
  H3_JOINT_CHARACTERIZE = FLC_STAGE_H3_JOINT_CHARACTERIZE,
  H4_JOINT_CALIBRATE = FLC_STAGE_H4_JOINT_CALIBRATE,
  H5_LEG = FLC_STAGE_H5_LEG,
  H6_FOUR_LEGS = FLC_STAGE_H6_FOUR_LEGS,
  H7_FREEZE = FLC_STAGE_H7_FREEZE,
};

// The highest stage this build is authorized to execute. Single source:
// flc_stage_config.h, default H0, overridable only by an explicit build flag.
// Raising it widens which named operation may be attempted; it does NOT bypass
// identity, census, direction or any hard servo guard.
static constexpr uint8_t AUTHORIZED_STAGE = FLC_AUTHORIZED_STAGE;

// Whether this build may use the conservative H3 bootstrap envelope. Still
// requires the operator to confirm it in the live session.
static constexpr bool H3_BOOTSTRAP_BUILD_APPROVED = (FLC_H3_BOOTSTRAP_APPROVED != 0);

// ==========================================================================
// CALIBRATION RUNTIME POLICY — centralized, provenance-labelled
//
// Nothing here is scattered into the state machine. Generic global safety
// limits are separate from joint-specific geometric travel limits.
// ==========================================================================

// ---- Generic servo-protection guards -------------------------------------
// These mirror the servo's own EEPROM protection registers and the frozen
// QC V6.1 / Provisioner V6 monitoring loop. They are mechanism-level and do
// not depend on mechanical load, so they carry over to the assembled leg.
static constexpr uint32_t MON_PERIOD_US = 2000;            // [GENERIC] QC V6.1
static constexpr uint32_t WRITE_SETTLE_MS = 20;            // [GENERIC] Provisioner V6
static constexpr int MON_THERMAL_LIMIT_C = 70;             // [VALIDATED] MaxTemperature 0x0D
static constexpr int MON_THERMAL_CONFIRMATIONS = 3;        // [GENERIC] QC V6.1
static constexpr int MON_VOLTAGE_MIN = 40;                 // [VALIDATED] MinVoltage 0x0F
static constexpr int MON_VOLTAGE_MAX = 140;                // [VALIDATED] MaxVoltage 0x0E
static constexpr uint32_t MON_VOLTAGE_SAMPLES = 50;        // [GENERIC] QC V6.1, 100 ms
static constexpr uint32_t MON_TELEMETRY_LOSS_SAMPLES = 25; // [GENERIC] QC V6.1, 50 ms
static constexpr int MON_STATIONARY_SPEED = 10;            // [GENERIC] QC V6.1

// Hard overcurrent abort. [VALIDATED] derived from the servo's own EEPROM
// ProtectionCurrent (0x1C = 310) in the frozen MATDOG_C018_V1 profile: the
// firmware aborts strictly BELOW the level at which the servo would protect
// itself, so the calibrator stops first. This is a protection ceiling, not a
// contact-detection threshold — contact is decided from the per-joint
// median/MAD baseline measured by H3.
static constexpr uint16_t MON_OVERCURRENT_HARD_RAW = 250;

// ---- Manual q0 capture (read-only, torque OFF) ---------------------------
static constexpr uint16_t Q0_SAMPLES_DEFAULT = 64;         // [GENERIC]
static constexpr uint16_t Q0_SAMPLES_MIN = 16;
static constexpr uint16_t Q0_SAMPLES_MAX = 256;
static constexpr uint32_t Q0_SAMPLE_INTERVAL_MS = 10;      // [GENERIC]
// A torque-OFF joint held by hand must be quiet. This bounds acceptable
// dispersion; it is a data-quality gate, not a motion safety constant.
static constexpr int Q0_MAX_SPREAD_TICKS = 12;             // [GENERIC]
// Spline-indexed mounting can leave a residual of roughly +/-5 deg == +/-57
// ticks. Accept a generous plausibility window; do NOT assert q0 == 2048.
static constexpr int Q0_PLAUSIBLE_RESIDUAL_TICKS = 120;    // [GENERIC]

// ---- Contact search — HISTORICAL CANDIDATES, NOT CURRENT TRUTH -----------
// Every value below comes from LF V25 on the PREVIOUS installation. They are
// recorded so the algorithm shape is reviewable, and they are deliberately
// NOT wired into any motion path. See characterizationOutstanding().
static constexpr uint16_t HIST_TORQUE_LIMIT = 500;         // [HISTORICAL] LF V25
static constexpr uint16_t HIST_GOAL_SPEED = 160;           // [HISTORICAL] LF V25
static constexpr uint8_t HIST_ACCELERATION = 8;            // [HISTORICAL] LF V25
static constexpr uint16_t HIST_COARSE_STEP_TICKS = 64;     // [HISTORICAL] LF V25
static constexpr uint16_t HIST_FINE_STEP_TICKS = 8;        // [HISTORICAL] LF V25
static constexpr uint16_t HIST_BACKOFF_TICKS = 96;         // [HISTORICAL] LF V25
static constexpr uint16_t HIST_HARD_CURRENT_ABORT_RAW = 200;// [HISTORICAL] LF V25
static constexpr uint16_t HIST_MIN_CONTACT_TRAVEL_TICKS = 24;// [HISTORICAL] LF V25
static constexpr uint8_t HIST_CONTACT_PERSISTENCE_SAMPLES = 3;// [HISTORICAL] LF V25
static constexpr uint16_t HIST_MAX_PROGRESS_TICKS = 2;     // [HISTORICAL] LF V25
static constexpr uint16_t HIST_REPEATABILITY_TOLERANCE_TICKS = 16; // [HISTORICAL] LF V25

// ---- Parameters requiring new hardware evidence, CLASSIFIED ---------------
//
// The first version of this firmware treated all eight as pre-motion blockers.
// That was a deadlock: H4 needed them, H3 was supposed to measure them, and H3
// was blocked by them. The fix is to classify each by WHEN it can be known,
// rather than pretending a value measurable only by moving can gate the move.
//
//   CLASS_A_PRE_MOTION   must be known BEFORE any H3 motion. Cannot be measured
//                        first, so it is supplied by the explicitly-approved,
//                        deliberately conservative H3 bootstrap envelope
//                        (flc_stage_config.h) and refuted/refined by H3.
//   CLASS_B_MEASURED_H3  measured DURING H3 on this build.
//   CLASS_C_DERIVED      computed from H3/H4 data; never a standalone constant.
//   CLASS_D_ACCEPTANCE   post-measure gate. Decides whether a result is
//                        ACCEPTED. Must NEVER block acquiring the measurement,
//                        which is exactly what it exists to judge.
//
// Only CLASS_A can gate motion, and only while the bootstrap is unapproved.
static constexpr uint16_t UNRESOLVED_U16 = 0xFFFF;

enum ParamClass : uint8_t {
  CLASS_A_PRE_MOTION = 0,
  CLASS_B_MEASURED_H3,
  CLASS_C_DERIVED,
  CLASS_D_ACCEPTANCE,
};

static const char *paramClassLabel(uint8_t c) {
  switch (c) {
    case CLASS_A_PRE_MOTION: return "A_PRE_MOTION";
    case CLASS_B_MEASURED_H3: return "B_MEASURED_H3";
    case CLASS_C_DERIVED: return "C_DERIVED";
    case CLASS_D_ACCEPTANCE: return "D_ACCEPTANCE";
    default: return "UNKNOWN";
  }
}

struct CharacterizationParam {
  const char *name;
  uint16_t value;
  uint8_t paramClass;
  const char *why;
};

static constexpr CharacterizationParam CHARACTERIZATION[] = {
  {"CONTACT_TORQUE_LIMIT", UNRESOLVED_U16, CLASS_A_PRE_MOTION,
   "needed to move at all; supplied by the approved H3 bootstrap envelope, "
   "which is deliberately below both the provisioner bench value (300) and "
   "LF V25 (500), and is refined by H3"},
  {"CONTACT_GOAL_SPEED", UNRESOLVED_U16, CLASS_A_PRE_MOTION,
   "needed to move at all; bootstrap 60 is far below LF V25 160"},
  {"CONTACT_ACCELERATION", UNRESOLVED_U16, CLASS_A_PRE_MOTION,
   "needed to move at all; bootstrap matches the slowest historical value"},
  {"CONTACT_CURRENT_THRESHOLD_RAW", 0, CLASS_C_DERIVED,
   "NOT a global constant. The detector derives the contact threshold from the "
   "per-joint free-motion median/MAD baseline that H3 measures, so no fleet-wide "
   "current threshold is needed or wanted"},
  {"CONTACT_RETREAT_TICKS", UNRESOLVED_U16, CLASS_B_MEASURED_H3,
   "H3 retreats with the bootstrap distance and reports what was actually "
   "achieved; H4 uses the measured value"},
  {"CONTACT_REPEATABILITY_TOLERANCE_TICKS", UNRESOLVED_U16, CLASS_D_ACCEPTANCE,
   "judges whether two contacts agree. Post-measure: it cannot gate the "
   "approaches whose spread it evaluates"},
  {"ENDPOINT_VS_URDF_TOLERANCE_TICKS", UNRESOLVED_U16, CLASS_D_ACCEPTANCE,
   "judges measured span against URDF geometry. Post-measure only: an unknown "
   "band leaves the result CANDIDATE rather than blocking the measurement"},
  {"MANUAL_Q0_VS_DERIVED_Q0_TOLERANCE_TICKS", UNRESOLVED_U16, CLASS_D_ACCEPTANCE,
   "judges manual pose against derived q0. It must never prevent acquiring the "
   "data needed to derive q0 in the first place"},
};
static constexpr size_t CHARACTERIZATION_COUNT =
    sizeof(CHARACTERIZATION) / sizeof(CHARACTERIZATION[0]);

//: Unresolved CLASS_A parameters — the only ones that can block first motion.
static size_t preMotionOutstanding() {
  size_t n = 0;
  for (size_t i = 0; i < CHARACTERIZATION_COUNT; ++i) {
    if (CHARACTERIZATION[i].paramClass == CLASS_A_PRE_MOTION &&
        CHARACTERIZATION[i].value == UNRESOLVED_U16) {
      ++n;
    }
  }
  return n;
}

static size_t characterizationOutstanding() {
  size_t n = 0;
  for (size_t i = 0; i < CHARACTERIZATION_COUNT; ++i) {
    if (CHARACTERIZATION[i].value == UNRESOLVED_U16) ++n;
  }
  return n;
}

// ==========================================================================
// JOINT SPEC TABLE — one generic engine, twelve data rows
//
// bus_id / unit  : 06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml
// declared limits: MATDOG_JOINT_CALIBRATION.yaml joint_groups (URDF)
// geometric contact: Geometry Compiler V5 endpoint profile
//   09_Logs/Validation_Reports/Geometry_Compiler/
//   2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_ENDPOINT_PROFILE.json
//
// Geometry survived the reassembly — it describes the design, not the build
// (MATDOG_CALIBRATION_RESET_2026-08-27.md). Hardware calibration did not.
//
// NOTE ON SYMMETRY: the hip geometric contacts are NOT symmetric and NOT equal
// across legs. LF/LH reach -46.012 deg at min; RF/RH reach +46.012 deg at max;
// rh_hip min is -45.156 while lf_hip min is -46.012. Each row is taken from the
// endpoint profile individually. Do not "simplify" this table.
//
// NOTE ON DIRECTION: encoder sign per joint is DELIBERATELY UNKNOWN (0).
// The pre-2026-08-27 directions in MATDOG_JOINT_CALIBRATION.yaml describe an
// installation that no longer exists and are stale by policy. Direction must be
// MEASURED on this build before any motion mode can use it.
// ==========================================================================

enum LegId : uint8_t { LEG_LF = 0, LEG_RF, LEG_RH, LEG_LH, LEG_COUNT };
enum JointKind : uint8_t { KIND_HIP = 0, KIND_UPPER, KIND_LOWER };

static const char *LEG_LABEL[LEG_COUNT] = {"LF", "RF", "RH", "LH"};
static const char *KIND_LABEL[3] = {"HIP", "UPPER", "LOWER"};

static constexpr JointSpec JOINTS[] = {
  // LF
  {13, "lf_hip_joint",       "M22",   LEG_LF, KIND_HIP,   -0.785398163397f,  0.785398163397f, -0.803055986689f,  0.789284248f},
  {12, "lf_upper_leg_joint", "ELR01", LEG_LF, KIND_UPPER, -0.916297857297f,  2.138028333693f, -0.909889226f,     2.127120026f},
  {11, "lf_lower_leg_joint", "M33",   LEG_LF, KIND_LOWER, -1.605702911835f,  0.654498469498f, -1.606998273f,     0.666361254f},
  // RF
  {23, "rf_hip_joint",       "NEW01", LEG_RF, KIND_HIP,   -0.785398163397f,  0.785398163397f, -0.789284248f,     0.803055986689f},
  {22, "rf_upper_leg_joint", "ELR03", LEG_RF, KIND_UPPER, -0.916297857297f,  2.138028333693f, -0.909889226f,     2.127120026f},
  {21, "rf_lower_leg_joint", "NEW03", LEG_RF, KIND_LOWER, -1.605702911835f,  0.654498469498f, -1.606998273f,     0.666361254f},
  // RH
  {33, "rh_hip_joint",       "NEW06", LEG_RH, KIND_HIP,   -0.785398163397f,  0.785398163397f, -0.788125240f,     0.803055986689f},
  {32, "rh_upper_leg_joint", "ELR02", LEG_RH, KIND_UPPER, -0.916297857297f,  2.138028333693f, -0.909889226f,     2.127120026f},
  {31, "rh_lower_leg_joint", "NEW05", LEG_RH, KIND_LOWER, -1.605702911835f,  0.654498469498f, -1.606998273f,     0.666361254f},
  // LH
  {43, "lh_hip_joint",       "M43",   LEG_LH, KIND_HIP,   -0.785398163397f,  0.785398163397f, -0.803055986689f,  0.788125240f},
  {42, "lh_upper_leg_joint", "M42",   LEG_LH, KIND_UPPER, -0.916297857297f,  2.138028333693f, -0.909889226f,     2.127120026f},
  {41, "lh_lower_leg_joint", "M41",   LEG_LH, KIND_LOWER, -1.605702911835f,  0.654498469498f, -1.606998273f,     0.666361254f},
};
static constexpr size_t JOINT_COUNT = sizeof(JOINTS) / sizeof(JOINTS[0]);

static bool geometryPlanUsable() {
  if (FLC_GEOMETRY_JOINT_COUNT != (int)JOINT_COUNT ||
      FLC_ENDPOINT_GEOMETRY_PLAN_COUNT != (int)(JOINT_COUNT * 2) ||
      !flcGeometryDependencyGraphAcyclic() ||
      !flcGeometryGeneratedTopologicalOrderValid()) {
    return false;
  }
  int noParking = 0;
  int parking = 0;
  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {
    const FlcEndpointGeometryPlan &row = FLC_ENDPOINT_GEOMETRY_PLANS[i];
    if ((int)row.targetJoint >= (int)JOINT_COUNT ||
        row.motionAuthorizationProvenance !=
            FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY) {
      return false;
    }
    if (row.parkingOutcome == FLC_NO_PARKING_REQUIRED) {
      ++noParking;
    } else if (row.parkingOutcome == FLC_PARKING_REQUIRED_1DOF &&
               row.auxiliaryJoint != FLC_GEOMETRY_JOINT_NONE) {
      ++parking;
    } else {
      return false;
    }
  }
  return noParking == 18 && parking == 6;
}

// Head ids 51..55 are allocated in the repository but the head is not built.
// A LEGS_12 session must expect them ABSENT and must not require them.
static constexpr uint8_t HEAD_IDS[] = {51, 52, 53, 54, 55};
static constexpr size_t HEAD_ID_COUNT = sizeof(HEAD_IDS) / sizeof(HEAD_IDS[0]);

static int jointIndexForId(int id) {
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (JOINTS[i].busId == id) return (int)i;
  }
  return -1;
}

// Broadcast id 254 can never satisfy this.
static bool validLegId(int id) { return jointIndexForId(id) >= 0; }

// ==========================================================================
// Session state
// ==========================================================================

enum SessionState : uint8_t {
  SESSION_IDLE = 0,
  SESSION_CENSUS_OK,
  SESSION_FAULT,
};

// Write stages. A write is refused unless the current stage owns that register.
enum Stage : uint8_t {
  STAGE_NONE = 0,
  STAGE_TORQUE_SAFETY,   // TorqueEnable = 0 only
  STAGE_MOTION_PREP,     // TorqueLimit
  STAGE_MOTION,          // TorqueEnable = 1, WritePosEx
};

static SessionState sessionState = SESSION_IDLE;
static Stage stage = STAGE_NONE;
static char lastFault[160] = "NONE";

// Volatile connection/session identity. `bootSessionId` changes on every ESP32
// reset and is never persisted. `activeHostSessionId` comes from an explicit
// @SESSION_BEGIN lease held by one open USB CDC connection. A host evidence
// file can record these values but cannot restore either one after reset.
static uint32_t bootSessionId = 0;
static uint32_t activeHostSessionId = 0;
static uint32_t sessionGeneration = 0;

static CensusEntry census[JOINT_COUNT];
static bool censusFresh = false;
static uint32_t censusEpoch = 0;

// --------------------------------------------------------------------------
// Per-session characterization store
//
// H3 writes it; H4/H5/H6 read it. RAM only, and tied to the census epoch: a
// reset or a fresh census clears it, so a later stage can never run on evidence
// gathered from a physical setup that is no longer the verified one.
// --------------------------------------------------------------------------

static JointCharacterization characterization[JOINT_COUNT];
static ManualQ0Evidence manualQ0[JOINT_COUNT];
static JointDirectionWitness directionWitness[JOINT_COUNT];
static JointCalibrationEvidence calibrationEvidence[JOINT_COUNT];

//: Operator confirmation of the bootstrap envelope, for the CURRENT session.
static bool bootstrapSessionApproved = false;
static uint32_t bootstrapSessionGeneration = 0;
static uint32_t bootstrapCensusEpoch = 0;

static void clearCharacterization(const char *why) {
  for (size_t i = 0; i < JOINT_COUNT; ++i) characterization[i] = JointCharacterization();
  if (why != nullptr) Serial.printf("CHARACTERIZATION_CLEARED WHY=%s\n", why);
}

static void clearManualQ0(const char *why) {
  for (size_t i = 0; i < JOINT_COUNT; ++i) manualQ0[i] = ManualQ0Evidence();
  if (why != nullptr) Serial.printf("MANUAL_Q0_CLEARED WHY=%s\n", why);
}

static void clearDirectionWitnesses(const char *why) {
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    directionWitness[i] = JointDirectionWitness();
  }
  if (why != nullptr) Serial.printf("DIRECTION_WITNESSES_CLEARED WHY=%s\n", why);
}

static void clearCalibrationEvidence(const char *why) {
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    calibrationEvidence[i] = JointCalibrationEvidence();
  }
  if (why != nullptr) Serial.printf("CALIBRATION_EVIDENCE_CLEARED WHY=%s\n", why);
}

static void clearBootstrapApproval() {
  bootstrapSessionApproved = false;
  bootstrapSessionGeneration = 0;
  bootstrapCensusEpoch = 0;
}

static void clearSessionEvidence(const char *why) {
  clearCharacterization(why);
  clearManualQ0(why);
  clearDirectionWitnesses(why);
  clearCalibrationEvidence(why);
  clearBootstrapApproval();
}

static bool activeSessionUsable() {
  return activeHostSessionId != 0 && sessionState != SESSION_FAULT;
}

static bool requireActiveSession(const char *mode) {
  if (sessionState == SESSION_FAULT) {
    Serial.printf("%s_REFUSED REASON=SESSION_FAULT_LATCHED FAULT=%s\n", mode,
                  lastFault);
    Serial.println("CUT_SERVO_POWER_NOW");
    return false;
  }
  if (activeHostSessionId == 0) {
    Serial.printf("%s_REFUSED REASON=NO_ACTIVE_PERSISTENT_SESSION\n", mode);
    Serial.printf("%s_HINT=@SESSION_BEGIN_<HOST_NONCE>\n", mode);
    return false;
  }
  return true;
}

static bool characterizationUsable(size_t index) {
  return activeSessionUsable() && censusFresh && index < JOINT_COUNT &&
         characterization[index].valid &&
         characterization[index].sessionGeneration == sessionGeneration &&
         characterization[index].censusEpoch == censusEpoch;
}

static bool manualQ0Usable(size_t index) {
  return activeSessionUsable() && censusFresh && index < JOINT_COUNT &&
         manualQ0[index].valid &&
         manualQ0[index].sessionGeneration == sessionGeneration &&
         manualQ0[index].censusEpoch == censusEpoch;
}

static bool allManualQ0Usable() {
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (!manualQ0Usable(i)) return false;
  }
  return true;
}

static bool directionWitnessUsable(size_t index) {
  return activeSessionUsable() && censusFresh && index < JOINT_COUNT &&
         directionWitness[index].valid &&
         directionWitness[index].sessionGeneration == sessionGeneration &&
         directionWitness[index].censusEpoch == censusEpoch &&
         (directionWitness[index].direction == 1 ||
          directionWitness[index].direction == -1);
}

static bool calibrationEvidenceUsable(size_t index) {
  return activeSessionUsable() && censusFresh && index < JOINT_COUNT &&
         calibrationEvidence[index].valid &&
         calibrationEvidence[index].sessionGeneration == sessionGeneration &&
         calibrationEvidence[index].censusEpoch == censusEpoch &&
         (calibrationEvidence[index].direction == 1 ||
          calibrationEvidence[index].direction == -1);
}

//: Bootstrap is usable only when the BUILD allows it AND the operator confirmed
//: it in this session. Neither alone is sufficient.
static bool bootstrapUsable() {
  return H3_BOOTSTRAP_BUILD_APPROVED && bootstrapSessionApproved &&
         activeSessionUsable() && censusFresh &&
         bootstrapSessionGeneration == sessionGeneration &&
         bootstrapCensusEpoch == censusEpoch;
}

static void invalidateCensus(const char *why) {
  censusFresh = false;
  for (size_t i = 0; i < JOINT_COUNT; ++i) census[i] = CensusEntry();
  if (why != nullptr) {
    Serial.printf("CENSUS_INVALIDATED WHY=%s\n", why);
  }
}

static void setFault(const char *reason) {
  snprintf(lastFault, sizeof(lastFault), "%s", reason);
  sessionState = SESSION_FAULT;
  censusFresh = false;
  clearBootstrapApproval();
}

// ==========================================================================
// WRITE CHOKE POINT
//
// Every servo write in this firmware goes through flcWrite() or
// flcWritePosEx(). There is no other call site of writeByte / writeWord /
// EnableTorque / WritePosEx anywhere below.
// ==========================================================================

static bool writeAllowed(uint8_t addr, uint8_t width, uint16_t value) {
  switch (addr) {
    case REG_TORQUE_ENABLE:
      if (width != 1) return false;
      // 128 is CalibrationOfs — refused unconditionally, at every stage.
      if (value == 0) {
        return stage == STAGE_TORQUE_SAFETY || stage == STAGE_MOTION_PREP ||
               stage == STAGE_MOTION;
      }
      if (value == 1) {
        return stage == STAGE_MOTION;
      }
      return false;

    case REG_TORQUE_LIMIT:
      // Runtime only, and only inside an authorized motion transaction.
      return width == 2 && stage == STAGE_MOTION_PREP && value > 0 && value <= 1000;

    default:
      // Everything else — including Lock 0x37, PositionOffset 0x1F, ID 0x05
      // and all 20 persistent-profile registers — is unreachable.
      return false;
  }
}

static WriteOutcome flcWrite(uint8_t id, uint8_t addr, uint8_t width,
                             uint16_t value, const char *name) {
  WriteOutcome out = {false, 0, 0xFF, -1, 0xFF, false};

  if (!validLegId(id)) {
    Serial.printf("WRITE_REFUSED_NOT_A_LEG_ID ID=%u\n", id);
    return out;
  }
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

  out.ok = (out.ack == 1) && (out.writeStatus == 0) &&
           (out.readback == (int)value) && (out.readStatus == 0);

  Serial.printf(
      "WRITE ID=%u ADDR=0x%02X NAME=%s WIDTH=%u EXPECT=%u ACK=%d STATUS=0x%02X "
      "READBACK=%d READ_STATUS=0x%02X RESULT=%s\n",
      id, addr, name, width, value, out.ack, out.writeStatus, out.readback,
      out.readStatus, out.ok ? "OK" : "FAIL");

  return out;
}

// THE ONLY MOTION PRIMITIVE IN THIS FIRMWARE.
// Unreachable at H0: motion modes are refused long before this is called, and
// the stage gate refuses again here as a second, independent barrier.
static bool flcWritePosEx(uint8_t id, int32_t position, uint16_t speed,
                          uint8_t acc, const char *why) {
  if (stage != STAGE_MOTION) {
    Serial.printf("MOTION_REFUSED STAGE=%u\n", (unsigned)stage);
    return false;
  }
  if (!validLegId(id)) {
    Serial.printf("MOTION_REFUSED_NOT_A_LEG_ID ID=%u\n", id);
    return false;
  }
  // Unsigned domain enforced before the call, so WritePosEx's negative
  // sign-magnitude branch is unreachable.
  if (position < 0 || position > ENCODER_MAX) {
    Serial.printf("MOTION_REFUSED POSITION_OUT_OF_DOMAIN=%ld\n", (long)position);
    return false;
  }
  // H3 is the lowest stage at which anything may move at all.
  if (AUTHORIZED_STAGE < H3_JOINT_CHARACTERIZE) {
    Serial.printf("MOTION_REFUSED HARDWARE_STAGE_LOCKED AUTHORIZED=H%u\n",
                  (unsigned)AUTHORIZED_STAGE);
    return false;
  }
  // The pre-motion parameters must come from somewhere. Either they are
  // resolved outright, or the operator has explicitly approved the conservative
  // bootstrap envelope for this session. Post-measure acceptance tolerances
  // (CLASS_D) deliberately do NOT gate here — they judge results, not motion.
  if (preMotionOutstanding() > 0 && !bootstrapUsable()) {
    Serial.printf("MOTION_REFUSED PRE_MOTION_PARAMS_UNRESOLVED=%u BOOTSTRAP_APPROVED=NO\n",
                  (unsigned)preMotionOutstanding());
    return false;
  }

  int ack = st.WritePosEx(id, (s16)position, speed, acc);
  uint8_t status = st.Error;

  Serial.printf("MOTION WHY=%s ID=%u GOAL=%ld SPEED=%u ACC=%u ACK=%d STATUS=0x%02X\n",
                why, id, (long)position, speed, acc, ack, status);

  return ack == 1 && status == 0;
}

// ==========================================================================
// Telemetry
// ==========================================================================

static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t i16le(const uint8_t *p) {
  return (int16_t)u16le(p);
}

static bool readSnapshot(uint8_t id, uint8_t *raw) {
  int ping = st.Ping(id);
  if (ping != id || st.Error != 0) return false;

  int n = st.Read(id, SNAPSHOT_START, raw, SNAPSHOT_LEN);
  if (n != SNAPSHOT_LEN || st.Error != 0) return false;

  return true;
}

// ==========================================================================
// SAFE_OFF — unicast torque OFF to every positively identified leg servo,
// with individual readback. No broadcast. Idempotent. Callable after any fault.
// ==========================================================================

static bool safeOffOne(uint8_t id, bool &sawResponder) {
  Stage saved = stage;
  stage = STAGE_TORQUE_SAFETY;

  int probe = st.readByte(id, REG_TORQUE_ENABLE);
  if (probe < 0 || st.Error != 0) {
    stage = saved;
    const int index = jointIndexForId(id);
    // Before a successful census an absent servo cannot hold torque. Once the
    // session has positively identified this unit, losing it during SAFE_OFF is
    // not proof of torque OFF: fail loudly and require physical power removal.
    if (index >= 0 && census[index].present) {
      Serial.printf("SAFE_OFF ID=%u RESULT=FAIL_KNOWN_RESPONDER_LOST\n", id);
      return false;
    }
    Serial.printf("SAFE_OFF ID=%u RESULT=NO_RESPONDER EVIDENCE=NOT_PREVIOUSLY_IDENTIFIED\n",
                  id);
    return true;
  }

  sawResponder = true;

  if (probe == 0) {
    stage = saved;
    Serial.printf("SAFE_OFF ID=%u RESULT=ALREADY_OFF\n", id);
    return true;
  }

  WriteOutcome out = flcWrite(id, REG_TORQUE_ENABLE, 1, 0, "TorqueEnable");
  stage = saved;

  if (!out.ok) {
    Serial.printf("SAFE_OFF ID=%u RESULT=FAIL\n", id);
    return false;
  }
  Serial.printf("SAFE_OFF ID=%u RESULT=OFF\n", id);
  return true;
}

//: Torque OFF every known leg servo without the full command framing. Used as
//: the unconditional exit of every multi-joint orchestration.
static bool runSafeOffQuiet() {
  bool allOk = true;
  bool sawResponder = false;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (!safeOffOne(JOINTS[i].busId, sawResponder)) allOk = false;
  }
  stage = STAGE_NONE;
  Serial.printf("SAFE_OFF_SWEEP RESULT=%s RESPONDERS_SEEN=%s\n",
                allOk ? "PASS" : "FAIL", sawResponder ? "YES" : "NO");
  if (!allOk) {
    setFault("SAFE_OFF_SWEEP_FAILED");
    Serial.println("CUT_SERVO_POWER_NOW");
  }
  return allOk;
}

static void runSafeOff() {
  Serial.println("SAFE_OFF_BEGIN");

  bool allOk = true;
  bool sawResponder = false;

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (!safeOffOne(JOINTS[i].busId, sawResponder)) allOk = false;
  }

  stage = STAGE_NONE;

  Serial.printf("SAFE_OFF_RESPONDERS_SEEN=%s\n", sawResponder ? "YES" : "NO");
  Serial.printf("SAFE_OFF_RESULT %s\n", allOk ? "PASS" : "FAIL");
  if (!allOk) {
    setFault("SAFE_OFF_FAILED");
    Serial.println("CUT_SERVO_POWER_NOW");
  }
  Serial.println("SAFE_OFF_END");
}

// ==========================================================================
// STATUS / INFO — no servo writes, no bus traffic required
// ==========================================================================

static void runStatus() {
  Serial.println("STATUS_BEGIN");
  Serial.printf("FIRMWARE_NAME=%s\n", FIRMWARE_NAME);
  Serial.printf("FIRMWARE_VERSION=%s\n", FIRMWARE_VERSION);
  Serial.printf("BUILD_GIT_SHA=%s\n", FLC_BUILD_GIT_SHA);
  Serial.printf("BUILD_WORKTREE_DIRTY=%s\n",
                FLC_BUILD_WORKTREE_DIRTY ? "YES" : "NO");
  Serial.printf("BUILD_DATE=%s\n", __DATE__);
  Serial.printf("BUILD_TIME=%s\n", __TIME__);
  Serial.printf("PROTOCOL_ID=%s\n", PROTOCOL_ID);
  Serial.printf("PROTOCOL_SCOPE=%s\n", PROTOCOL_SCOPE);
  Serial.printf("PROFILE_ID=%s\n", PROFILE_ID);
  Serial.printf("STATION_IN_CONTROL_PATH=NO\n");
  Serial.printf("EEPROM_WRITE_SURFACE=NONE\n");
  Serial.printf("BROADCAST_WRITE=NEVER\n");
  Serial.printf("GOAL_POSITION_AUTHORITY=flcWritePosEx\n");
  Serial.printf("GOAL_POSITION_DOMAIN=0..%d\n", ENCODER_MAX);
  Serial.printf("SERVO_UART_BAUD=%lu\n", (unsigned long)SERVO_BAUD);
  Serial.printf("SERVO_TX_GPIO=%d\n", SERVO_TX_PIN);
  Serial.printf("SERVO_RX_GPIO=%d\n", SERVO_RX_PIN);
  Serial.printf("EXPECTED_LEG_SERVOS=%u\n", (unsigned)JOINT_COUNT);
  Serial.printf("HEAD_SERVOS_EXPECTED_PRESENT=NO\n");
  Serial.printf("AUTHORIZED_HARDWARE_STAGE=H%u\n", (unsigned)AUTHORIZED_STAGE);
  int noParkingPlans = 0;
  int parkingPlans = 0;
  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {
    if (FLC_ENDPOINT_GEOMETRY_PLANS[i].parkingOutcome ==
        FLC_NO_PARKING_REQUIRED) {
      ++noParkingPlans;
    } else {
      ++parkingPlans;
    }
  }
  Serial.printf("GEOMETRY_ENDPOINT_PLANS=%d\n", FLC_ENDPOINT_GEOMETRY_PLAN_COUNT);
  Serial.printf("GEOMETRY_NO_PARKING_REQUIRED=%d\n", noParkingPlans);
  Serial.printf("GEOMETRY_PARKING_REQUIRED_1DOF=%d\n", parkingPlans);
  Serial.printf("GEOMETRY_DEPENDENCIES=%d\n", FLC_GEOMETRY_DEPENDENCY_COUNT);
  Serial.printf("GEOMETRY_PARKING_FILE_SHA256=%s\n",
                FLC_GEOMETRY_PARKING_FILE_SHA256);
  Serial.printf("GEOMETRY_PARKING_SEMANTIC_SHA256=%s\n",
                FLC_GEOMETRY_PARKING_SEMANTIC_SHA256);
  Serial.printf("GEOMETRY_SAFETY_POLICY_FILE_SHA256=%s\n",
                FLC_GEOMETRY_SAFETY_POLICY_FILE_SHA256);
  Serial.printf("GEOMETRY_ARTIFACT_MOTION_AUTHORIZATION=%s\n",
                FLC_GEOMETRY_ARTIFACT_GRANTS_MOTION_AUTHORIZATION ? "YES" : "NO");
  Serial.printf("GEOMETRY_PLAN_SELF_CHECK=%s\n",
                geometryPlanUsable() ? "PASS" : "FAIL");
  Serial.printf("BOOT_SESSION_ID=%08lX\n", (unsigned long)bootSessionId);
  Serial.printf("ACTIVE_HOST_SESSION_ID=%08lX\n",
                (unsigned long)activeHostSessionId);
  Serial.printf("SESSION_GENERATION=%lu\n", (unsigned long)sessionGeneration);
  Serial.printf("SESSION_STATE=%u\n", (unsigned)sessionState);
  Serial.printf("CENSUS_FRESH=%s\n", censusFresh ? "YES" : "NO");
  Serial.printf("CENSUS_EPOCH=%lu\n", (unsigned long)censusEpoch);
  Serial.printf("LAST_FAULT=%s\n", lastFault);

  size_t outstanding = characterizationOutstanding();
  Serial.printf("CHARACTERIZATION_OUTSTANDING=%u\n", (unsigned)outstanding);
  Serial.printf("PRE_MOTION_OUTSTANDING=%u\n", (unsigned)preMotionOutstanding());
  for (size_t i = 0; i < CHARACTERIZATION_COUNT; ++i) {
    if (CHARACTERIZATION[i].value == UNRESOLVED_U16) {
      Serial.printf("CHARACTERIZATION_REQUIRED NAME=%s CLASS=%s WHY=%s\n",
                    CHARACTERIZATION[i].name,
                    paramClassLabel(CHARACTERIZATION[i].paramClass),
                    CHARACTERIZATION[i].why);
    }
  }

  // Bootstrap visibility: a session can never silently be running on the
  // conservative first-motion envelope rather than measured values.
  Serial.printf("H3_BOOTSTRAP_BUILD_APPROVED=%s\n",
                H3_BOOTSTRAP_BUILD_APPROVED ? "YES" : "NO");
  Serial.printf("H3_BOOTSTRAP_SESSION_APPROVED=%s\n",
                bootstrapSessionApproved ? "YES" : "NO");
  Serial.printf("H3_BOOTSTRAP_ACTIVE=%s\n", bootstrapUsable() ? "YES" : "NO");
  if (bootstrapUsable()) {
    const FlcMotionEnvelope envelope = flcBootstrapEnvelope();
    Serial.printf("H3_BOOTSTRAP_ENVELOPE ORIGIN=%s TORQUE_LIMIT=%u SPEED=%u ACC=%u "
                  "RETREAT=%u\n",
                  flcParameterOriginLabel(envelope.origin), envelope.torqueLimit,
                  envelope.goalSpeed, envelope.acceleration, envelope.retreatTicks);
  }

  size_t characterized = 0;
  size_t manualQ0Count = 0;
  size_t witnessCount = 0;
  size_t calibratedCount = 0;
  bool anyJointCalibrationReady = false;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (manualQ0Usable(i)) ++manualQ0Count;
    if (directionWitnessUsable(i)) ++witnessCount;
    if (calibrationEvidenceUsable(i)) ++calibratedCount;
    if (manualQ0Usable(i) && directionWitnessUsable(i) &&
        characterizationUsable(i)) {
      anyJointCalibrationReady = true;
    }
    if (characterizationUsable(i)) {
      ++characterized;
      Serial.printf("JOINT_CHARACTERIZED ID=%u JOINT=%s ORIGIN=%s RAW_PROBE_SIGN=%d "
                    "CONTACT_THRESHOLD_RAW=%u RETREAT_TICKS=%u\n",
                    JOINTS[i].busId, JOINTS[i].jointName,
                    flcParameterOriginLabel(characterization[i].origin),
                    (int)characterization[i].rawProbeSign,
                    characterization[i].contactThresholdRaw,
                    characterization[i].retreatTicks);
    }
  }
  Serial.printf("JOINTS_CHARACTERIZED=%u/%u\n", (unsigned)characterized,
                (unsigned)JOINT_COUNT);
  Serial.printf("MANUAL_Q0_CANDIDATES=%u/%u\n", (unsigned)manualQ0Count,
                (unsigned)JOINT_COUNT);
  Serial.printf("DIRECTION_WITNESSES=%u/%u\n", (unsigned)witnessCount,
                (unsigned)JOINT_COUNT);
  Serial.printf("CALIBRATION_CANDIDATES=%u/%u\n", (unsigned)calibratedCount,
                (unsigned)JOINT_COUNT);

  Serial.printf("MOTION_UNLOCKED=%s\n",
                (activeSessionUsable() && censusFresh &&
                 AUTHORIZED_STAGE >= H3_JOINT_CHARACTERIZE &&
                 (preMotionOutstanding() == 0 || bootstrapUsable()))
                    ? "YES" : "NO");
  Serial.printf("CALIBRATE_JOINT_AVAILABLE=%s\n",
                (AUTHORIZED_STAGE >= H4_JOINT_CALIBRATE &&
                 anyJointCalibrationReady)
                    ? "YES" : "NO");
  Serial.println("PROMOTION_TO_CANONICAL=REQUIRES_EXPLICIT_SEPARATE_GATE");
  Serial.println("STATUS_RESULT PASS");
  Serial.println("STATUS_END");
}

// ==========================================================================
// LEGS_12_CENSUS — read-only identity and invariant gate
// ==========================================================================

static void runCensus() {
  Serial.println("CENSUS_BEGIN");
  if (!requireActiveSession("CENSUS")) {
    Serial.println("CENSUS_RESULT REFUSED");
    Serial.println("CENSUS_END");
    return;
  }
  Serial.printf("CENSUS_EXPECTED_IDS=11,12,13,21,22,23,31,32,33,41,42,43\n");

  invalidateCensus(nullptr);
  ++censusEpoch;
  // A new census means a new physical session epoch. Every dependent evidence
  // object is dropped; no host-side file can recreate any of it.
  clearSessionEvidence("NEW_CENSUS_EPOCH");

  size_t presentCount = 0;
  size_t identityOkCount = 0;
  bool anyFailure = false;
  uint8_t raw[SNAPSHOT_LEN];

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const JointSpec &spec = JOINTS[i];
    CensusEntry &e = census[i];
    e = CensusEntry();

    if (!readSnapshot(spec.busId, raw)) {
      Serial.printf("CENSUS_SERVO ID=%u JOINT=%s UNIT=%s RESULT=MISSING\n",
                    spec.busId, spec.jointName, spec.unitLabel);
      anyFailure = true;
      continue;
    }

    e.present = true;
    ++presentCount;

    e.model = u16le(raw + REG_MODEL);
    e.responseStatus = raw[REG_RESPONSE_STATUS];
    e.positionOffset = i16le(raw + REG_POSITION_OFFSET);
    e.torqueEnable = raw[REG_TORQUE_ENABLE];
    e.torqueLimit = u16le(raw + REG_TORQUE_LIMIT);
    e.presentPosition = u16le(raw + REG_PRESENT_POSITION);
    e.voltage = raw[REG_PRESENT_VOLTAGE];
    e.temperature = raw[REG_PRESENT_TEMPERATURE];
    e.statusByte = raw[REG_STATUS];

    uint8_t storedId = raw[REG_ID];
    uint8_t baud = raw[REG_BAUD];

    // 20-register persistent profile comparison — read only, never written.
    e.profileMatch = true;
    for (size_t p = 0; p < PROFILE_COUNT; ++p) {
      uint16_t observed = (PROFILE[p].width == 2) ? u16le(raw + PROFILE[p].addr)
                                                  : raw[PROFILE[p].addr];
      if (observed != PROFILE[p].value) {
        e.profileMatch = false;
        Serial.printf("CENSUS_PROFILE_MISMATCH ID=%u REG=%s ADDR=0x%02X "
                      "EXPECT=%u OBSERVED=%u\n",
                      spec.busId, PROFILE[p].name, PROFILE[p].addr,
                      PROFILE[p].value, observed);
      }
    }

    bool modelOk = (e.model == EXPECTED_MODEL);
    bool idOk = (storedId == spec.busId);
    bool baudOk = (baud == EXPECTED_BAUD);
    bool respOk = (e.responseStatus == EXPECTED_RESPONSE_STATUS);
    bool offsetOk = (e.positionOffset == EXPECTED_POSITION_OFFSET);
    bool posOk = (e.presentPosition <= ENCODER_MAX);
    bool voltOk = (e.voltage >= MON_VOLTAGE_MIN && e.voltage <= MON_VOLTAGE_MAX);
    bool tempOk = (e.temperature < MON_THERMAL_LIMIT_C);
    bool statusOk = (e.statusByte == 0);

    e.identityOk = modelOk && idOk && baudOk && respOk && offsetOk &&
                   e.profileMatch && posOk && voltOk && tempOk && statusOk;

    if (e.identityOk) ++identityOkCount; else anyFailure = true;

    Serial.printf(
        "CENSUS_SERVO ID=%u JOINT=%s UNIT=%s MODEL=%u ID_REG=%u BAUD=%u "
        "RESP=%u OFFSET=%d TORQUE=%u TORQUE_LIMIT=%u POS=%u VOLT=%u TEMP=%u "
        "STATUS=0x%02X PROFILE=%s RESULT=%s\n",
        spec.busId, spec.jointName, spec.unitLabel, e.model, storedId, baud,
        e.responseStatus, e.positionOffset, e.torqueEnable, e.torqueLimit,
        e.presentPosition, e.voltage, e.temperature, e.statusByte,
        e.profileMatch ? "MATCH" : "MISMATCH",
        e.identityOk ? "OK" : "FAIL");

    if (!modelOk)  Serial.printf("CENSUS_REJECT ID=%u WHY=WRONG_MODEL\n", spec.busId);
    if (!idOk)     Serial.printf("CENSUS_REJECT ID=%u WHY=ID_REGISTER_MISMATCH\n", spec.busId);
    if (!offsetOk) Serial.printf("CENSUS_REJECT ID=%u WHY=NONZERO_POSITION_OFFSET\n", spec.busId);
    if (!respOk)   Serial.printf("CENSUS_REJECT ID=%u WHY=RESPONSE_STATUS\n", spec.busId);
    if (!posOk)    Serial.printf("CENSUS_REJECT ID=%u WHY=IMPOSSIBLE_POSITION\n", spec.busId);
    if (!voltOk)   Serial.printf("CENSUS_REJECT ID=%u WHY=VOLTAGE_OUT_OF_RANGE\n", spec.busId);
    if (!tempOk)   Serial.printf("CENSUS_REJECT ID=%u WHY=TEMPERATURE\n", spec.busId);
    if (!statusOk) Serial.printf("CENSUS_REJECT ID=%u WHY=STATUS_ERROR\n", spec.busId);
    if (e.torqueEnable != 0) {
      Serial.printf("CENSUS_WARN ID=%u WHY=TORQUE_UNEXPECTEDLY_ON\n", spec.busId);
      anyFailure = true;
    }
  }

  // Unexpected extra responders on the leg bus, including the head band.
  size_t unexpected = 0;
  for (int id = 1; id <= 60; ++id) {
    if (jointIndexForId(id) >= 0) continue;
    int ping = st.Ping((uint8_t)id);
    if (ping < 0) continue;
    ++unexpected;
    bool isHead = false;
    for (size_t h = 0; h < HEAD_ID_COUNT; ++h) if (HEAD_IDS[h] == id) isHead = true;
    Serial.printf("CENSUS_UNEXPECTED_RESPONDER ID=%d CLASS=%s\n",
                  id, isHead ? "HEAD_NOT_EXPECTED_INSTALLED" : "UNKNOWN");
    anyFailure = true;
  }

  Serial.printf("CENSUS_PRESENT=%u/%u\n", (unsigned)presentCount, (unsigned)JOINT_COUNT);
  Serial.printf("CENSUS_IDENTITY_OK=%u/%u\n", (unsigned)identityOkCount, (unsigned)JOINT_COUNT);
  Serial.printf("CENSUS_UNEXPECTED=%u\n", (unsigned)unexpected);
  Serial.printf("CENSUS_HEAD_ABSENCE_EXPECTED=YES\n");

  censusFresh = (!anyFailure) && (identityOkCount == JOINT_COUNT);

  if (censusFresh) {
    sessionState = SESSION_CENSUS_OK;
    Serial.println("CENSUS_RESULT PASS");
  } else {
    sessionState = SESSION_IDLE;
    if (presentCount == 0) {
      Serial.println("CENSUS_FAIL_REASON=NO_RESPONDERS");
    } else {
      Serial.println("CENSUS_FAIL_REASON=EXPECTED_LEG_INVARIANTS_NOT_SATISFIED");
    }
    Serial.println("CENSUS_RESULT FAIL");
  }
  Serial.println("CENSUS_END");
}

// ==========================================================================
// Precondition gate shared by every non-STATUS operating mode
// ==========================================================================

static bool requireStage(uint8_t needed, const char *mode) {
  if (AUTHORIZED_STAGE >= needed) return true;
  Serial.printf("%s_REFUSED REASON=HARDWARE_STAGE_LOCKED NEEDS=H%u AUTHORIZED=H%u\n",
                mode, (unsigned)needed, (unsigned)AUTHORIZED_STAGE);
  return false;
}

static bool requireFreshCensus(const char *mode) {
  if (censusFresh) return true;
  Serial.printf("%s_REFUSED REASON=NO_FRESH_CENSUS_IN_THIS_SESSION\n", mode);
  return false;
}

//: Pre-motion parameter gate. Only CLASS_A can block, and only while the
//: bootstrap envelope is unapproved — see the classification table above.
static bool requirePreMotionParameters(const char *mode) {
  const size_t outstanding = preMotionOutstanding();
  if (outstanding == 0 || bootstrapUsable()) return true;
  Serial.printf("%s_REFUSED REASON=PRE_MOTION_PARAMS_UNRESOLVED COUNT=%u\n",
                mode, (unsigned)outstanding);
  for (size_t i = 0; i < CHARACTERIZATION_COUNT; ++i) {
    if (CHARACTERIZATION[i].paramClass == CLASS_A_PRE_MOTION &&
        CHARACTERIZATION[i].value == UNRESOLVED_U16) {
      Serial.printf("%s_BLOCKED_BY NAME=%s CLASS=%s\n", mode,
                    CHARACTERIZATION[i].name,
                    paramClassLabel(CHARACTERIZATION[i].paramClass));
    }
  }
  return false;
}

// ==========================================================================
// CAPTURE_MANUAL_Q0 — read-only with respect to motion, torque must be OFF
//
// Emits manual_pose_q0_candidate ONLY. This is never promoted to final q0 here.
// ==========================================================================

static void runCaptureManualQ0(uint16_t samples) {
  Serial.println("CAPTURE_Q0_BEGIN");

  if (!requireActiveSession("CAPTURE_Q0")) {
    Serial.println("CAPTURE_Q0_RESULT REFUSED");
    Serial.println("CAPTURE_Q0_END");
    return;
  }
  if (!requireStage(H2_MANUAL_Q0, "CAPTURE_Q0")) {
    Serial.println("CAPTURE_Q0_RESULT REFUSED");
    Serial.println("CAPTURE_Q0_END");
    return;
  }
  if (!requireFreshCensus("CAPTURE_Q0")) {
    Serial.println("CAPTURE_Q0_RESULT REFUSED");
    Serial.println("CAPTURE_Q0_END");
    return;
  }

  if (samples < Q0_SAMPLES_MIN) samples = Q0_SAMPLES_MIN;
  if (samples > Q0_SAMPLES_MAX) samples = Q0_SAMPLES_MAX;

  Serial.printf("CAPTURE_Q0_SAMPLES=%u\n", samples);
  Serial.printf("CAPTURE_Q0_OUTPUT=manual_pose_q0_candidate\n");
  Serial.printf("CAPTURE_Q0_PROMOTION=NOT_FINAL_Q0\n");

  bool allOk = true;
  ManualQ0Evidence staged[JOINT_COUNT] = {};

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const JointSpec &spec = JOINTS[i];

    // Torque must be OFF. Verified per joint, never assumed.
    int torque = st.readByte(spec.busId, REG_TORQUE_ENABLE);
    if (torque < 0 || st.Error != 0) {
      Serial.printf("CAPTURE_Q0_JOINT ID=%u JOINT=%s RESULT=NO_RESPONDER\n",
                    spec.busId, spec.jointName);
      allOk = false;
      continue;
    }
    if (torque != 0) {
      Serial.printf("CAPTURE_Q0_JOINT ID=%u JOINT=%s RESULT=REFUSED_TORQUE_ON\n",
                    spec.busId, spec.jointName);
      allOk = false;
      continue;
    }

    // Samples are summarized by the shared circular helper in
    // flc_contact_detector.h, which the host test suite exercises directly.
    static int captured[Q0_SAMPLES_MAX];
    uint16_t got = 0;

    for (uint16_t s = 0; s < samples && got < Q0_SAMPLES_MAX; ++s) {
      int pos = st.ReadPos(spec.busId);
      if (pos < 0 || st.Error != 0 || pos > ENCODER_MAX) continue;
      captured[got++] = pos;
      delay(Q0_SAMPLE_INTERVAL_MS);
    }

    if (got < Q0_SAMPLES_MIN) {
      Serial.printf("CAPTURE_Q0_JOINT ID=%u JOINT=%s RESULT=INSUFFICIENT_SAMPLES GOT=%u\n",
                    spec.busId, spec.jointName, got);
      allOk = false;
      continue;
    }

    FlcTickSummary summary = flcSummarizeTicks(captured, (int)got);
    int centre = summary.centreTick;
    int minTick = summary.minTick;
    int maxTick = summary.maxTick;
    int spread = summary.spreadTicks;
    int residual = signedTickDelta(centre, RAW_ELECTRICAL_CENTER);
    float residualDeg = (float)residual * 360.0f / (float)ENCODER_MODULUS;

    bool stable = (spread <= Q0_MAX_SPREAD_TICKS);
    bool plausible = (abs(residual) <= Q0_PLAUSIBLE_RESIDUAL_TICKS);

    Serial.printf(
        "CAPTURE_Q0_JOINT ID=%u JOINT=%s UNIT=%s SAMPLES=%u CENTRE=%d MIN=%d "
        "MAX=%d SPREAD=%d RESIDUAL_TICKS=%d RESIDUAL_DEG=%.3f STABLE=%s "
        "PLAUSIBLE=%s RESULT=%s\n",
        spec.busId, spec.jointName, spec.unitLabel, got, centre, minTick,
        maxTick, spread, residual, residualDeg,
        stable ? "YES" : "NO", plausible ? "YES" : "NO",
        (stable && plausible) ? "OK" : "FAIL");

    if (!stable || !plausible) allOk = false;
    if (stable && plausible) {
      staged[i].valid = true;
      staged[i].sessionGeneration = sessionGeneration;
      staged[i].censusEpoch = censusEpoch;
      staged[i].centreTick = centre;
      staged[i].minTick = minTick;
      staged[i].maxTick = maxTick;
      staged[i].spreadTicks = spread;
      staged[i].samples = got;
    }
  }

  if (allOk) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) manualQ0[i] = staged[i];
    // Any downstream evidence belonged to the previous physical q0 capture.
    clearDirectionWitnesses("NEW_MANUAL_Q0_CAPTURE");
    clearCharacterization("NEW_MANUAL_Q0_CAPTURE");
    clearCalibrationEvidence("NEW_MANUAL_Q0_CAPTURE");
    clearBootstrapApproval();
  } else {
    clearManualQ0("CAPTURE_Q0_INCOMPLETE");
    clearDirectionWitnesses("CAPTURE_Q0_INCOMPLETE");
    clearCharacterization("CAPTURE_Q0_INCOMPLETE");
    clearCalibrationEvidence("CAPTURE_Q0_INCOMPLETE");
    clearBootstrapApproval();
  }

  Serial.printf("CAPTURE_Q0_RESULT %s\n", allOk ? "PASS" : "FAIL");
  Serial.println("CAPTURE_Q0_END");
}

// --------------------------------------------------------------------------
// DIRECTION WITNESS — semantic current-build evidence, no servo write
// --------------------------------------------------------------------------

static void runWitnessDirection(int id, int8_t direction, const char *semantic) {
  Serial.println("WITNESS_DIRECTION_BEGIN");
  if (!requireActiveSession("WITNESS_DIRECTION") ||
      !requireStage(H2_MANUAL_Q0, "WITNESS_DIRECTION") ||
      !requireFreshCensus("WITNESS_DIRECTION")) {
    Serial.println("WITNESS_DIRECTION_RESULT REFUSED");
    Serial.println("WITNESS_DIRECTION_END");
    return;
  }
  if (!validLegId(id) || (direction != 1 && direction != -1)) {
    Serial.printf("WITNESS_DIRECTION_REFUSED REASON=INVALID_SEMANTIC_WITNESS ID=%d\n",
                  id);
    Serial.println("WITNESS_DIRECTION_RESULT REFUSED");
    Serial.println("WITNESS_DIRECTION_END");
    return;
  }
  const int index = jointIndexForId(id);
  if (!manualQ0Usable((size_t)index)) {
    Serial.println("WITNESS_DIRECTION_REFUSED REASON=NO_CURRENT_MANUAL_Q0_CANDIDATE");
    Serial.println("WITNESS_DIRECTION_RESULT REFUSED");
    Serial.println("WITNESS_DIRECTION_END");
    return;
  }

  JointDirectionWitness &witness = directionWitness[index];
  witness.valid = true;
  witness.sessionGeneration = sessionGeneration;
  witness.censusEpoch = censusEpoch;
  witness.direction = direction;

  // Changing the semantic mapping invalidates evidence interpreted under an
  // earlier witness.
  characterization[index] = JointCharacterization();
  calibrationEvidence[index] = JointCalibrationEvidence();

  Serial.printf("WITNESS_DIRECTION ID=%u JOINT=%s SEMANTIC=%s DIRECTION=%d "
                "MEANING=q_equals_direction_times_signed_raw_delta\n",
                JOINTS[index].busId, JOINTS[index].jointName, semantic,
                (int)direction);
  Serial.println("WITNESS_DIRECTION_RESULT PASS");
  Serial.println("WITNESS_DIRECTION_END");
}

// ==========================================================================
// CALIBRATE_JOINT / CALIBRATE_LEG / CALIBRATE_ALL_LEGS
//
// The generic engine exists as one code path for all 12 joints. It is gated at
// its entry: no leg-specific variant, and no way to reach motion while the
// stage gate or the characterization gate refuses.
// ==========================================================================

// --------------------------------------------------------------------------
// ST3215 hardware adapter — the bridge between the generic engine and the bus.
//
// The engine holds no Arduino or SCServo dependency; these callbacks are the
// only place the two meet. commandPosition routes to flcWritePosEx(), which
// remains the single GoalPosition authority in this firmware.
// --------------------------------------------------------------------------

//: The engine calls this immediately before every read and every
//: motion-capable write. It is the single place where "the session that
//: authorized this motion is still the session executing it" is decided, so a
//: reset, a reconnect, a new census or a latched fault stops motion in-flight
//: rather than at the next command boundary. Torque-OFF stays reachable
//: because flcEndMotion() does not route through here.
static bool portValidateContext(void *ctx, uint8_t id) {
  (void)ctx;
  if (sessionState == SESSION_FAULT) return false;
  if (!activeSessionUsable() || !censusFresh) return false;
  const int index = jointIndexForId((int)id);
  if (index < 0) return false;
  const CensusEntry &entry = census[index];
  return entry.present && entry.identityOk && entry.profileMatch;
}

static bool portReadTelemetry(void *ctx, uint8_t id, FlcObservation *out) {
  (void)ctx;
  if (out == nullptr) return false;

  // One FeedBack() pulls the whole RAM telemetry block, so every field below
  // belongs to the SAME sample of the SAME servo rather than to a mix of reads.
  if (st.FeedBack((int)id) == -1 || st.Error != 0) return false;

  const int position = st.ReadPos(-1);
  const int speed = st.ReadSpeed(-1);
  const int current = st.ReadCurrent(-1);
  const int voltage = st.ReadVoltage(-1);
  const int temperature = st.ReadTemper(-1);
  if (position < 0 || position > ENCODER_MAX) return false;

  const int torqueEnable = st.readByte(id, REG_TORQUE_ENABLE);
  if (torqueEnable < 0 || st.Error != 0) return false;
  const int torqueLimit = st.readWord(id, REG_TORQUE_LIMIT);
  if (torqueLimit < 0 || st.Error != 0) return false;
  const int goal = st.readWord(id, REG_GOAL_POSITION);
  if (goal < 0 || st.Error != 0) return false;
  const int statusByte = st.readByte(id, REG_STATUS);
  if (statusByte < 0 || st.Error != 0) return false;

  out->telemetryValid = true;
  out->driverError = false;
  out->statusByte = (uint8_t)statusByte;
  out->torqueEnabled = (torqueEnable != 0);
  out->torqueLimit = (uint16_t)torqueLimit;
  out->goalPosition = (uint16_t)goal;
  out->position = (uint16_t)position;
  out->velocity = (int16_t)speed;
  out->current = (uint16_t)(current < 0 ? 0 : current);
  out->temperature = (uint8_t)(temperature < 0 ? 0 : temperature);
  out->voltage = (uint8_t)(voltage < 0 ? 0 : voltage);
  out->elapsedMs = 0;   // filled in by the engine against its own budget clock
  return true;
}

static bool portCommandPosition(void *ctx, uint8_t id, int position, uint16_t speed,
                                uint8_t acc) {
  (void)ctx;
  // Each adapter callback owns its write-stage lifetime. TorqueEnable's
  // callback must not leave a hidden global stage armed, and restoring its
  // stage must not make the following position command spuriously fail.
  const Stage saved = stage;
  stage = STAGE_MOTION;
  const bool ok = flcWritePosEx(id, position, speed, acc, "engine");
  stage = saved;
  return ok;
}

static bool portSetTorqueLimit(void *ctx, uint8_t id, uint16_t limit) {
  (void)ctx;
  Stage saved = stage;
  stage = STAGE_MOTION_PREP;
  const bool ok = flcWrite(id, REG_TORQUE_LIMIT, 2, limit, "TorqueLimit").ok;
  stage = saved;
  return ok;
}

static bool portSetTorqueEnable(void *ctx, uint8_t id, bool enable) {
  (void)ctx;
  Stage saved = stage;
  // Torque ON only inside an authorized motion transaction; torque OFF is
  // always permitted, which is what makes recovery paths safe.
  stage = enable ? STAGE_MOTION : STAGE_TORQUE_SAFETY;
  const bool ok = flcWrite(id, REG_TORQUE_ENABLE, 1, enable ? 1 : 0, "TorqueEnable").ok;
  stage = saved;
  return ok;
}

static uint32_t portNowMs(void *ctx) {
  (void)ctx;
  return millis();
}

static void portIdle(void *ctx, uint32_t ms) {
  (void)ctx;
  delay(ms);
}

static void portTrace(void *ctx, const char *line) {
  (void)ctx;
  Serial.printf("ENGINE %s\n", line);
}

static FlcServoPort makeServoPort() {
  FlcServoPort port;
  port.validateContext = portValidateContext;
  port.readTelemetry = portReadTelemetry;
  port.commandPosition = portCommandPosition;
  port.setTorqueLimit = portSetTorqueLimit;
  port.setTorqueEnable = portSetTorqueEnable;
  port.nowMs = portNowMs;
  port.idle = portIdle;
  port.trace = portTrace;
  port.ctx = nullptr;
  return port;
}

// Generic guard set shared by every motion path. Joint-specific travel limits
// come from the envelope and the joint spec, not from here.
static FlcContactConfig makeGuards() {
  FlcContactConfig guards;
  guards.maxProgressTicks = 2;
  guards.maxVelocityRaw = MON_STATIONARY_SPEED;
  guards.targetReachedToleranceTicks = 10;
  guards.minTravelTicks = 24;
  guards.persistenceSamples = 3;
  guards.startupGraceSamples = 4;
  guards.hardCurrentAbortRaw = MON_OVERCURRENT_HARD_RAW;
  guards.expectedTorqueLimit = 0;   // set per transaction by the engine
  guards.thermalLimitC = MON_THERMAL_LIMIT_C;
  guards.voltageMin = MON_VOLTAGE_MIN;
  guards.voltageMax = MON_VOLTAGE_MAX;
  guards.travelBudgetTicks = 0;     // set per transaction by the engine
  guards.timeBudgetMs = 0;          // set per transaction by the engine
  guards.probeSign = 0;             // measured, never assumed
  return guards;
}

// Build the fresh-telemetry watch used while one or more explicitly excluded
// joints move. Every other joint must remain at its H2 q0 candidate with torque
// OFF. GoalPosition is intentionally unconstrained for those torque-off joints:
// it may contain a stale RAM value, but their actual position may not drift.
static bool makeQ0MotionWatch(uint16_t excludedMask, FlcMotionWatch &watch) {
  watch = FlcMotionWatch();
  if (!allManualQ0Usable()) return false;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if ((excludedMask & (uint16_t)(1U << i)) != 0) continue;
    if (watch.count >= FLC_MAX_WATCHED_JOINTS) return false;
    FlcWatchJoint &entry = watch.joints[watch.count++];
    entry.busId = JOINTS[i].busId;
    entry.expectedTick = manualQ0[i].centreTick;
    entry.toleranceTicks = Q0_MAX_SPREAD_TICKS;
    entry.expectedTorqueState = 0;
    entry.expectedTorqueLimit = 0;
    entry.expectedGoalTick = -1;
  }
  return true;
}


// --------------------------------------------------------------------------
// Acceptance gates — POST-MEASURE, never pre-motion
//
// These decide whether a measurement is ACCEPTED. They must never prevent the
// measurement itself: a tolerance that is unknown leaves the result CANDIDATE.
// --------------------------------------------------------------------------

static FlcAcceptanceGates makeAcceptanceGates(const JointSpec &spec) {
  // Value-initialised: an acceptance gate that is never assigned must read as
  // UNKNOWN, never as whatever the stack happened to contain. An indeterminate
  // "tolerance known" byte could otherwise promote a result to ACCEPTED, which
  // is the one outcome this firmware may never fabricate.
  FlcAcceptanceGates gates = FlcAcceptanceGates();

  // Repeatability band is characterized per build. Until H3 has run we use the
  // measured spread from characterization; absent that, the result is refused
  // rather than accepted on a guessed band.
  gates.repeatabilityToleranceKnown = false;
  gates.repeatabilityToleranceTicks = 0;
  gates.endpointVsUrdfToleranceKnown = false;
  gates.endpointVsUrdfToleranceTicks = 0;

  // The manual-vs-derived q0 agreement tolerance has NOT been characterized on
  // this build. Leaving it explicitly unknown is what makes H4 report
  // BLOCKED_TOLERANCE_UNVALIDATED instead of inventing an acceptance.
  gates.manualVsDerivedQ0ToleranceKnown = false;
  gates.manualVsDerivedQ0ToleranceTicks = 0;

  const float spanRad = spec.geomContactMaxRad - spec.geomContactMinRad;
  gates.expectedSpanTicks = (int)(spanRad * (float)ENCODER_MODULUS / 6.283185307f + 0.5f);
  return gates;
}

static int angleToTicks(float rad) {
  const float ticks = rad * (float)ENCODER_MODULUS / 6.283185307f;
  return (int)(ticks >= 0.0f ? ticks + 0.5f : ticks - 0.5f);
}

// H3 must not unknowingly drive through one of the six geometry-obstructed
// endpoint paths. The semantic q<->raw witness lets us select an endpoint that
// the canonical table explicitly marks NO_PARKING_REQUIRED. This is still only
// raw encoder response/contact characterization; it does not "measure"
// kinematic direction by watching a position servo follow its raw command.
static int8_t h3NoParkingRawProbeSign(int jointIndex) {
  if (jointIndex < 0 || jointIndex >= (int)JOINT_COUNT ||
      !directionWitnessUsable((size_t)jointIndex)) {
    return 0;
  }
  const JointSpec &spec = JOINTS[jointIndex];
  const FlcEndpointGeometryPlan *minPlan =
      flcGeometryPlanFor(spec.jointName, "min");
  const FlcEndpointGeometryPlan *maxPlan =
      flcGeometryPlanFor(spec.jointName, "max");
  const int8_t direction = directionWitness[jointIndex].direction;
  if (minPlan != nullptr &&
      minPlan->parkingOutcome == FLC_NO_PARKING_REQUIRED) {
    return (int8_t)-direction;
  }
  if (maxPlan != nullptr &&
      maxPlan->parkingOutcome == FLC_NO_PARKING_REQUIRED) {
    return direction;
  }
  return 0;
}

// --------------------------------------------------------------------------
// Shared preflight for every motion mode
// --------------------------------------------------------------------------

static bool motionPreflight(const char *mode, uint8_t neededStage, int jointIndex,
                            bool requireCharacterized) {
  bool ok = true;
  if (!geometryPlanUsable()) {
    Serial.printf("%s_REFUSED REASON=INVALID_GEOMETRY_PLAN\n", mode);
    setFault("INVALID_GEOMETRY_PLAN");
    ok = false;
  }
  if (!requireActiveSession(mode)) ok = false;
  if (!requireStage(neededStage, mode)) ok = false;
  if (!requireFreshCensus(mode)) ok = false;
  if (!requirePreMotionParameters(mode)) ok = false;

  if (jointIndex < 0) {
    if (!allManualQ0Usable()) {
      Serial.printf("%s_REFUSED REASON=INCOMPLETE_MANUAL_Q0_EVIDENCE\n", mode);
      ok = false;
    }
    // A whole-leg or whole-robot run needs a semantic witness for EVERY joint
    // it may touch, including a cross-leg parking auxiliary. Checking here means
    // the operator is refused before anything moves rather than part-way in.
    if (neededStage >= H3_JOINT_CHARACTERIZE) {
      for (size_t i = 0; i < JOINT_COUNT; ++i) {
        if (directionWitnessUsable(i)) continue;
        Serial.printf("%s_REFUSED REASON=NO_CURRENT_SEMANTIC_DIRECTION_WITNESS ID=%u\n",
                      mode, JOINTS[i].busId);
        ok = false;
      }
      if (!ok) {
        Serial.printf("%s_HINT=@WITNESS_DIRECTION_<ID>_<Q_PLUS_RAW_...>_CONFIRM\n",
                      mode);
      }
    }
  } else if (!manualQ0Usable((size_t)jointIndex)) {
    Serial.printf("%s_REFUSED REASON=NO_CURRENT_MANUAL_Q0_CANDIDATE\n", mode);
    ok = false;
  }

  if (jointIndex >= 0 && neededStage >= H3_JOINT_CHARACTERIZE &&
      !directionWitnessUsable((size_t)jointIndex)) {
    Serial.printf("%s_REFUSED REASON=NO_CURRENT_SEMANTIC_DIRECTION_WITNESS\n",
                  mode);
    Serial.printf("%s_HINT=@WITNESS_DIRECTION_<ID>_<Q_PLUS_RAW_...>_CONFIRM\n",
                  mode);
    ok = false;
  }

  if (jointIndex >= 0) {
    const CensusEntry &entry = census[jointIndex];
    if (!entry.present || !entry.identityOk) {
      Serial.printf("%s_REFUSED REASON=JOINT_IDENTITY_NOT_VERIFIED\n", mode);
      ok = false;
    }
    if (requireCharacterized && !characterizationUsable((size_t)jointIndex)) {
      Serial.printf("%s_REFUSED REASON=JOINT_NOT_CHARACTERIZED_IN_THIS_SESSION\n", mode);
      Serial.printf("%s_HINT=RUN @CHARACTERIZE_JOINT FIRST\n", mode);
      ok = false;
    }
  }
  if (!ok) {
    Serial.printf("%s_NOTE=LF_V25_NUMERIC_RESULTS_CANNOT_AUTHORIZE_CURRENT_HARDWARE\n", mode);
  }
  return ok;
}

// --------------------------------------------------------------------------
// H3 — @CHARACTERIZE_JOINT
//
// Measures direction, free-motion baseline and one supervised bounded contact
// with retreat. This is what resolves the parameters H4 needs, and it is
// deliberately NOT blocked by the acceptance tolerances it exists to inform.
// --------------------------------------------------------------------------

static void runCharacterizeJoint(int id) {
  Serial.println("CHARACTERIZE_JOINT_BEGIN");

  if (!validLegId(id)) {
    Serial.printf("CHARACTERIZE_JOINT_REFUSED REASON=NOT_A_LEG_ID ID=%d\n", id);
    Serial.println("CHARACTERIZE_JOINT_RESULT REFUSED");
    Serial.println("CHARACTERIZE_JOINT_END");
    return;
  }
  const int index = jointIndexForId(id);
  const JointSpec &spec = JOINTS[index];
  Serial.printf("CHARACTERIZE_JOINT_TARGET ID=%u JOINT=%s UNIT=%s\n",
                spec.busId, spec.jointName, spec.unitLabel);

  if (!motionPreflight("CHARACTERIZE_JOINT", H3_JOINT_CHARACTERIZE, index, false)) {
    Serial.println("CHARACTERIZE_JOINT_RESULT REFUSED");
    Serial.println("CHARACTERIZE_JOINT_END");
    return;
  }
  if (!bootstrapUsable()) {
    Serial.println("CHARACTERIZE_JOINT_REFUSED REASON=H3_BOOTSTRAP_NOT_APPROVED");
    Serial.printf("CHARACTERIZE_JOINT_BOOTSTRAP_BUILD_APPROVED=%s SESSION_APPROVED=%s\n",
                  H3_BOOTSTRAP_BUILD_APPROVED ? "YES" : "NO",
                  bootstrapSessionApproved ? "YES" : "NO");
    Serial.println("CHARACTERIZE_JOINT_HINT=@APPROVE_BOOTSTRAP CONFIRM");
    Serial.println("CHARACTERIZE_JOINT_RESULT REFUSED");
    Serial.println("CHARACTERIZE_JOINT_END");
    return;
  }

  const FlcMotionEnvelope envelope = flcBootstrapEnvelope();
  Serial.printf("CHARACTERIZE_JOINT_ENVELOPE ORIGIN=%s TORQUE_LIMIT=%u SPEED=%u ACC=%u "
                "STEP=%u RETREAT=%u TRAVEL_BUDGET=%u TIME_BUDGET=%lu\n",
                flcParameterOriginLabel(envelope.origin), envelope.torqueLimit,
                envelope.goalSpeed, envelope.acceleration, envelope.stepTicks,
                envelope.retreatTicks, envelope.travelBudgetTicks,
                (unsigned long)envelope.timeBudgetMs);

  // H3 drives a real contact. Choosing the probe side from the canonical table
  // is what keeps it off the six geometry-obstructed endpoint paths, which no
  // amount of current limiting would make safe.
  const int8_t rawProbeSign = h3NoParkingRawProbeSign(index);
  if (rawProbeSign != 1 && rawProbeSign != -1) {
    Serial.println("CHARACTERIZE_JOINT_REFUSED REASON=NO_NO_PARKING_ENDPOINT_FOR_PROBE");
    Serial.println("CHARACTERIZE_JOINT_RESULT REFUSED");
    Serial.println("CHARACTERIZE_JOINT_END");
    return;
  }
  Serial.printf("CHARACTERIZE_JOINT_RAW_PROBE_SIGN=%d PROVENANCE=NO_PARKING_REQUIRED\n",
                (int)rawProbeSign);

  FlcMotionWatch watch;
  if (!makeQ0MotionWatch((uint16_t)(1U << index), watch)) {
    Serial.println("CHARACTERIZE_JOINT_REFUSED REASON=INCOMPLETE_MANUAL_Q0_EVIDENCE");
    Serial.println("CHARACTERIZE_JOINT_RESULT REFUSED");
    Serial.println("CHARACTERIZE_JOINT_END");
    return;
  }

  const int startTick = census[index].presentPosition;
  FlcServoPort port = makeServoPort();
  const FlcContactConfig guards = makeGuards();

  const FlcCharacterizationResult result = flcCharacterizeJoint(
      port, (uint8_t)id, startTick, rawProbeSign, envelope, guards, 48, 96, &watch);

  Serial.printf("CHARACTERIZE_JOINT_STATUS=%s REASON=%s\n",
                flcEngineStatusLabel(result.status),
                flcAbortReasonLabel(result.abortReason));
  // Deliberately NOT called DIRECTION: a position servo following its own raw
  // command proves the encoder responds, not the MATDOG kinematic mapping.
  Serial.printf("CHARACTERIZE_JOINT_ENCODER_RESPONSE RESPONDS=%d RAW_PROBE_SIGN=%d "
                "TRAVEL=%d START=%d END=%d\n",
                result.encoderResponse.responds ? 1 : 0,
                (int)result.encoderResponse.rawProbeSign,
                result.encoderResponse.observedTravel,
                result.encoderResponse.startTick, result.encoderResponse.endTick);
  Serial.printf("CHARACTERIZE_JOINT_BASELINE SAMPLES=%d MEDIAN_CURRENT=%u MAD=%u "
                "MIN=%u MAX=%u PEAK_SPEED=%u\n",
                result.baseline.samples, result.baseline.baseline.medianCurrent,
                result.baseline.baseline.madCurrent, result.baseline.minCurrent,
                result.baseline.maxCurrent, result.baseline.peakSpeed);
  Serial.printf("CHARACTERIZE_JOINT_CONTACT TICK=%d TRAVEL=%d PEAK_CURRENT=%u "
                "CURRENT_SUPPORTED=%d\n",
                result.probeContact.contactTick, result.probeContact.travelTicks,
                result.probeContact.peakCurrent,
                result.probeContact.currentSupportedContact ? 1 : 0);
  Serial.printf("CHARACTERIZE_JOINT_RETREAT ACHIEVED=%d TRACKING_RECOVERED=%d "
                "CURRENT_RECOVERED=%d REST_CURRENT=%u\n",
                result.probeRetreat.achievedTicks,
                result.probeRetreat.trackingRecovered ? 1 : 0,
                result.probeRetreat.currentRecovered ? 1 : 0,
                result.probeRetreat.restCurrent);
  Serial.printf("CHARACTERIZE_JOINT_CONTACT2 TICK=%d SPREAD=%u REPEATABILITY_BAND=%u\n",
                result.probeContact2.contactTick, result.observedContactSpreadTicks,
                result.characterizedRepeatabilityToleranceTicks);

  // Whatever happened, this joint must end released.
  const bool released = flcEndMotion(port, (uint8_t)id);
  Serial.printf("CHARACTERIZE_JOINT_TORQUE_OFF_VERIFIED=%s\n", released ? "YES" : "NO");
  if (!released) {
    setFault("CHARACTERIZE_JOINT_TORQUE_OFF_FAILED");
    Serial.println("CUT_SERVO_POWER_NOW");
  }

  if (result.status == FLC_ENGINE_OK && result.complete && released) {
    JointCharacterization &store = characterization[index];
    store.valid = true;
    store.sessionGeneration = sessionGeneration;
    store.censusEpoch = censusEpoch;
    store.rawProbeSign = result.encoderResponse.rawProbeSign;
    store.baseline = result.baseline.baseline;
    store.contactThresholdRaw = result.characterizedContactThresholdRaw;
    store.retreatTicks = result.characterizedRetreatTicks;
    store.repeatabilityToleranceTicks = result.characterizedRepeatabilityToleranceTicks;
    store.observedSpreadTicks = result.observedContactSpreadTicks;
    store.restTick = result.finalRetreat.toTick;
    store.origin = FLC_ORIGIN_CHARACTERIZED_CURRENT_HARDWARE;

    Serial.printf("CHARACTERIZE_JOINT_OUTPUT ORIGIN=%s RAW_PROBE_SIGN=%d "
                  "CONTACT_THRESHOLD_RAW=%u RETREAT_TICKS=%u OBSERVED_SPREAD=%u "
                  "REPEATABILITY_BAND=%u REST_TICK=%d\n",
                  flcParameterOriginLabel(store.origin), (int)store.rawProbeSign,
                  store.contactThresholdRaw, store.retreatTicks,
                  store.observedSpreadTicks, store.repeatabilityToleranceTicks,
                  store.restTick);
    Serial.println("CHARACTERIZE_JOINT_RESULT PASS");
  } else {
    Serial.println("CHARACTERIZE_JOINT_RESULT FAIL");
  }
  Serial.println("CHARACTERIZE_JOINT_END");
}

// --------------------------------------------------------------------------
// H4 / H5 / H6 — ONE shared production orchestration
//
// H4, H5 and H6 differ ONLY in which joints their selection mask names. All
// three build the identical 12-row plan catalog and execute the identical
// flcRunCalibrationPlan() in flc_calibration_engine.h — the same translation
// unit the offline harness drives. There is no firmware-local calibration
// loop, so the code that is tested is the code that moves the robot.
//
// Dependency order, parking and restore come from the generated geometry plan
// inside the engine, never from a leg-local distal-first heuristic here.
// --------------------------------------------------------------------------

//: Build one engine plan row. Every row is filled for all 12 joints because
//: the engine needs the whole catalog to hold un-selected joints at q0 and to
//: express a cross-leg parking angle in raw ticks.
static bool buildJointPlan(int index, FlcJointPlan &plan) {
  const JointSpec &spec = JOINTS[index];
  const JointCharacterization &store = characterization[index];

  plan = FlcJointPlan();
  plan.geometryJoint = (FlcGeometryJoint)index;
  plan.busId = spec.busId;
  plan.characterized = characterizationUsable((size_t)index);
  plan.startTick = store.restTick;
  plan.baseline = store.baseline;

  // The q0 watch is the H2 manual pose candidate. It is evidence about where
  // the joint physically is, and is never treated as a derived calibration.
  plan.q0WatchKnown = manualQ0Usable((size_t)index);
  plan.q0WatchTick = plan.q0WatchKnown ? manualQ0[index].centreTick : -1;
  plan.q0WatchToleranceTicks = Q0_MAX_SPREAD_TICKS;

  // Prior calibration in THIS session only. A joint calibrated earlier in the
  // run can serve as a parking auxiliary; nothing historical may.
  plan.calibrationKnown = calibrationEvidenceUsable((size_t)index);
  plan.knownDirection = plan.calibrationKnown ? calibrationEvidence[index].direction : 0;
  plan.knownQ0Tick = plan.calibrationKnown ? calibrationEvidence[index].derivedQ0Tick : -1;

  FlcDirectionEvidence &evidence = plan.directionEvidence;
  evidence.rawLoTick = -1;          // filled by the engine from measurement
  evidence.rawHiTick = -1;
  evidence.qMinTicks = angleToTicks(spec.geomContactMinRad);
  evidence.qMaxTicks = angleToTicks(spec.geomContactMaxRad);
  evidence.manualQ0Known = plan.q0WatchKnown;
  evidence.manualQ0Tick = plan.q0WatchTick;
  // H2 sample spread proves encoder stability, NOT physical pose accuracy, so
  // it must not become a pose-uncertainty tolerance.
  evidence.manualQ0PoseUncertaintyKnown = false;
  evidence.manualQ0PoseUncertaintyTicks = 0;
  evidence.operatorWitness =
      directionWitnessUsable((size_t)index) ? directionWitness[index].direction : 0;

  FlcMotionEnvelope envelope = flcBootstrapEnvelope();
  if (store.retreatTicks > 0) envelope.retreatTicks = store.retreatTicks;
  plan.envelope = flcClampEnvelope(envelope);

  plan.gates = makeAcceptanceGates(spec);
  if (plan.characterized && store.repeatabilityToleranceTicks > 0) {
    plan.gates.repeatabilityToleranceKnown = true;
    plan.gates.repeatabilityToleranceTicks = store.repeatabilityToleranceTicks;
  }
  return plan.characterized;
}

//: Fill the whole catalog. Returns the mask of joints that are ready to be
//: calibrated; callers intersect it with what they were asked to do.
static uint16_t buildPlanCatalog(FlcJointPlan *plans) {
  uint16_t ready = 0;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (buildJointPlan((int)i, plans[i])) ready |= (uint16_t)(1U << i);
  }
  return ready;
}

static void reportParking(const char *mode, const JointSpec &spec,
                          const char *side, const FlcEndpointExecution &e) {
  if (e.parkingOutcome == FLC_NO_PARKING_REQUIRED) {
    Serial.printf("%s_JOINT_%s_PARKING ID=%u OUTCOME=NO_PARKING_REQUIRED "
                  "PROVENANCE_VERIFIED=%d ENDPOINT_INDEX=%u\n",
                  mode, side, spec.busId,
                  e.parking.noParkingProvenanceVerified ? 1 : 0,
                  e.geometryEndpointIndex);
    return;
  }
  const char *auxName = flcGeometryJointName(e.parking.auxiliaryJoint);
  Serial.printf("%s_JOINT_%s_PARKING ID=%u OUTCOME=PARKING_REQUIRED_1DOF AUX=%s "
                "AUX_ID=%u PREREQ_VERIFIED=%d ENTERED=%d TRACKING_VERIFIED=%d "
                "SAVED=%d PARK=%d ACHIEVED=%d RESTORE_ATTEMPTED=%d "
                "RESTORE_VERIFIED=%d RESTORED=%d ENDPOINT_INDEX=%u\n",
                mode, side, spec.busId, auxName != 0 ? auxName : "NONE",
                e.parking.auxiliaryBusId,
                e.parking.prerequisiteVerified ? 1 : 0,
                e.parking.entered ? 1 : 0,
                e.parking.trackingVerified ? 1 : 0,
                e.parking.savedTick, e.parking.parkingTick,
                e.parking.achievedTick,
                e.parking.restoreAttempted ? 1 : 0,
                e.parking.restoreVerified ? 1 : 0,
                e.parking.restoredTick, e.geometryEndpointIndex);
}

static void reportJointResult(const char *mode, const JointSpec &spec,
                              const FlcJointCalibrationResult &r) {
  Serial.printf("%s_JOINT_BEGIN ID=%u JOINT=%s UNIT=%s\n", mode, spec.busId,
                spec.jointName, spec.unitLabel);
  Serial.printf("%s_JOINT_STATUS=%s REASON=%s TIER=%s\n", mode,
                flcEngineStatusLabel(r.status), flcAbortReasonLabel(r.abortReason),
                flcResultTierLabel(r.tier));
  reportParking(mode, spec, "MIN", r.minExecution);
  reportParking(mode, spec, "MAX", r.maxExecution);
  Serial.printf("%s_JOINT_MIN_ENDPOINT TICK=%d SPREAD=%d ACCEPTED=%d "
                "RETURNED_TO_Q0=%d TORQUE_OFF_VERIFIED=%d\n", mode,
                r.minEndpoint.contactTick, r.minEndpoint.repeatability.spreadTicks,
                r.minEndpoint.accepted ? 1 : 0,
                r.minExecution.targetReturnedToQ0 ? 1 : 0,
                r.minExecution.targetTorqueOffVerified ? 1 : 0);
  Serial.printf("%s_JOINT_MAX_ENDPOINT TICK=%d SPREAD=%d ACCEPTED=%d "
                "RETURNED_TO_Q0=%d TORQUE_OFF_VERIFIED=%d\n", mode,
                r.maxEndpoint.contactTick, r.maxEndpoint.repeatability.spreadTicks,
                r.maxEndpoint.accepted ? 1 : 0,
                r.maxExecution.targetReturnedToQ0 ? 1 : 0,
                r.maxExecution.targetTorqueOffVerified ? 1 : 0);
  Serial.printf("%s_JOINT_SPAN MEASURED=%d EXPECTED=%d ERROR=%d RAW_LO=%d RAW_HI=%d\n",
                mode, r.measuredSpanTicks, r.expectedSpanTicks, r.spanErrorTicks,
                r.rawLoTick, r.rawHiTick);
  Serial.printf("%s_JOINT_DIRECTION RESOLVED=%d METHOD=%s GEOMETRY_DECISIVE=%d "
                "MARGIN=%d SEPARATION=%d\n", mode, (int)r.direction,
                flcDirectionMethodLabel(r.directionMethod),
                r.directionResolution.geometryDecisive ? 1 : 0,
                r.directionResolution.marginTicks,
                r.directionResolution.separationTicks);
  // The two q0 concepts stay separate and are never averaged.
  Serial.printf("%s_JOINT_MANUAL_POSE_Q0_CANDIDATE TICK=%d\n", mode,
                r.manualPoseQ0CandidateTick);
  Serial.printf("%s_JOINT_DERIVED_Q0_CANDIDATE TICK=%d\n", mode,
                r.derivedQ0FinalTick);
  Serial.printf("%s_JOINT_Q0_CROSSCHECK STATUS=%s ERROR_TICKS=%d\n", mode,
                flcQ0CrosscheckStatusLabel(r.q0CrosscheckStatus),
                r.manualVsDerivedQ0ErrorTicks);
  Serial.printf("%s_JOINT_PROMOTION=REQUIRES_EXPLICIT_SEPARATE_GATE\n", mode);
  Serial.printf("%s_JOINT_RESULT %s\n", mode,
                r.status == FLC_ENGINE_OK ? "PASS" : "FAIL");
  Serial.printf("%s_JOINT_END ID=%u\n", mode, spec.busId);
}

//: The ONE calibration entry point behind H4, H5 and H6.
static bool runCalibrationSelection(uint16_t requestedMask, const char *mode) {
  static FlcJointPlan plans[JOINT_COUNT];
  const uint16_t readyMask = buildPlanCatalog(plans);

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const uint16_t bit = (uint16_t)(1U << i);
    if ((requestedMask & bit) != 0 && (readyMask & bit) == 0) {
      Serial.printf("%s_JOINT_REFUSED ID=%u REASON=NOT_CHARACTERIZED_IN_THIS_SESSION\n",
                    mode, JOINTS[i].busId);
    }
  }
  const uint16_t selection = (uint16_t)(requestedMask & readyMask);
  if (selection != requestedMask) {
    Serial.printf("%s_REFUSED REASON=INCOMPLETE_CHARACTERIZATION REQUESTED=%04X READY=%04X\n",
                  mode, requestedMask, readyMask);
    return false;
  }

  FlcServoPort port = makeServoPort();
  const FlcContactConfig guards = makeGuards();
  const FlcCalibrationRunResult run =
      flcRunCalibrationPlan(port, guards, plans, (int)JOINT_COUNT, selection);

  Serial.printf("%s_PLAN SELECTION=%04X REQUESTED=%d EXECUTION_ORDER_COUNT=%d\n",
                mode, run.selectionMask, run.jointsRequested, run.executionCount);
  for (int i = 0; i < run.executionCount; ++i) {
    const char *name = flcGeometryJointName(run.executionOrder[i]);
    Serial.printf("%s_PLAN_ORDER %d=%s\n", mode, i, name != 0 ? name : "UNKNOWN");
  }
  Serial.printf("%s_PLAN_PARKING REQUIRED=%d NO_PARKING=%d RESTORED=%d\n", mode,
                run.parkingRequiredCount, run.noParkingCount,
                run.parkingRestoreCount);

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (!run.resultPresent[i]) continue;
    reportJointResult(mode, JOINTS[i], run.joints[i]);

    // Chaining WITHIN a run is the engine's own business; this stores the
    // result so a LATER command in the same physical session can use the joint
    // as a parking prerequisite. Only a fully OK joint qualifies, and a failed
    // one clears any evidence it previously had rather than leaving it stale.
    const FlcJointCalibrationResult &r = run.joints[i];
    JointCalibrationEvidence &store = calibrationEvidence[i];
    if (r.status == FLC_ENGINE_OK &&
        (r.direction == 1 || r.direction == -1) && r.derivedQ0FinalTick >= 0) {
      store.valid = true;
      store.sessionGeneration = sessionGeneration;
      store.censusEpoch = censusEpoch;
      store.direction = r.direction;
      store.derivedQ0Tick = r.derivedQ0FinalTick;
      store.minContactTick = r.minContactTick;
      store.maxContactTick = r.maxContactTick;
      store.tier = r.tier;
    } else {
      store = JointCalibrationEvidence();
    }
  }

  if (run.status != FLC_ENGINE_OK) {
    const char *failed = flcGeometryJointName(run.failedJoint);
    Serial.printf("%s_ABORTED_AT JOINT=%s ID=%u STATUS=%s REASON=%s\n", mode,
                  failed != 0 ? failed : "NONE", run.failedBusId,
                  flcEngineStatusLabel(run.status),
                  flcAbortReasonLabel(run.abortReason));
  }
  Serial.printf("%s_SAFE_OFF_VERIFIED=%s\n", mode,
                run.safeOffVerified ? "YES" : "NO");
  Serial.printf("%s_JOINTS_OK=%d/%d\n", mode, run.jointsOk, run.jointsRequested);

  // A run that cannot prove torque is released is a latched hard fault: no
  // further motion command may be accepted until the operator intervenes.
  if (run.hardFaultCutPowerNow || !run.safeOffVerified) {
    setFault("CALIBRATION_SAFE_OFF_UNVERIFIED");
    Serial.println("CUT_SERVO_POWER_NOW");
    return false;
  }
  return run.status == FLC_ENGINE_OK && run.jointsOk == run.jointsRequested;
}

static void runCalibrateJoint(int id) {
  Serial.println("CALIBRATE_JOINT_BEGIN");

  if (!validLegId(id)) {
    Serial.printf("CALIBRATE_JOINT_REFUSED REASON=NOT_A_LEG_ID ID=%d\n", id);
    Serial.println("CALIBRATE_JOINT_RESULT REFUSED");
    Serial.println("CALIBRATE_JOINT_END");
    return;
  }
  const int index = jointIndexForId(id);
  const JointSpec &spec = JOINTS[index];
  Serial.printf("CALIBRATE_JOINT_TARGET ID=%u JOINT=%s UNIT=%s\n", spec.busId,
                spec.jointName, spec.unitLabel);

  if (!motionPreflight("CALIBRATE_JOINT", H4_JOINT_CALIBRATE, index, true)) {
    Serial.println("CALIBRATE_JOINT_RESULT REFUSED");
    Serial.println("CALIBRATE_JOINT_END");
    return;
  }

  const bool ok =
      runCalibrationSelection((uint16_t)(1U << index), "CALIBRATE_JOINT");
  Serial.printf("CALIBRATE_JOINT_RESULT %s\n", ok ? "PASS" : "FAIL");
  Serial.println("CALIBRATE_JOINT_END");
}

//: Mask of the three joints of one leg. Ordering inside the leg is NOT decided
//: here — the engine derives it from the generated dependency graph, which is
//: why a naive LOWER->UPPER->HIP rule cannot creep back in.
static uint16_t legJointMask(int leg) {
  uint16_t mask = 0;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (JOINTS[i].leg == (uint8_t)leg) mask |= (uint16_t)(1U << i);
  }
  return mask;
}

static void runCalibrateLeg(int leg) {
  Serial.println("CALIBRATE_LEG_BEGIN");

  if (leg < 0 || leg >= LEG_COUNT) {
    Serial.println("CALIBRATE_LEG_REFUSED REASON=UNKNOWN_LEG");
    Serial.println("CALIBRATE_LEG_RESULT REFUSED");
    Serial.println("CALIBRATE_LEG_END");
    return;
  }
  Serial.printf("CALIBRATE_LEG_TARGET LEG=%s\n", LEG_LABEL[leg]);

  const uint16_t mask = legJointMask(leg);
  if (mask == 0 || __builtin_popcount(mask) != FLC_JOINTS_PER_LEG) {
    Serial.println("CALIBRATE_LEG_REFUSED REASON=INCOMPLETE_LEG_PLAN");
    Serial.println("CALIBRATE_LEG_RESULT REFUSED");
    Serial.println("CALIBRATE_LEG_END");
    return;
  }

  if (!motionPreflight("CALIBRATE_LEG", H5_LEG, -1, false)) {
    Serial.println("CALIBRATE_LEG_RESULT REFUSED");
    Serial.println("CALIBRATE_LEG_END");
    return;
  }

  // A leg whose parking auxiliary lives on ANOTHER leg cannot be calibrated in
  // isolation unless that auxiliary already has current-session calibration.
  // The engine enforces this; naming it here makes the refusal legible.
  const bool ok = runCalibrationSelection(mask, "CALIBRATE_LEG");
  Serial.printf("CALIBRATE_LEG_RESULT %s\n", ok ? "PASS" : "FAIL");
  Serial.println("CALIBRATE_LEG_END");
}

static void runCalibrateAll() {
  Serial.println("CALIBRATE_ALL_BEGIN");
  Serial.println("CALIBRATE_ALL_TARGET LEGS=LF,RF,RH,LH JOINTS=12");

  if (!motionPreflight("CALIBRATE_ALL", H6_FOUR_LEGS, -1, false)) {
    Serial.println("CALIBRATE_ALL_RESULT REFUSED");
    Serial.println("CALIBRATE_ALL_END");
    return;
  }

  // H6 is H4 with every bit set. There is no separate four-leg loop, so the
  // orchestration the offline suite exercises is exactly this one.
  const bool ok =
      runCalibrationSelection(FLC_ALL_GEOMETRY_JOINTS_MASK, "CALIBRATE_ALL");
  Serial.println("CALIBRATE_ALL_PROMOTION=REQUIRES_EXPLICIT_SEPARATE_GATE");
  Serial.printf("CALIBRATE_ALL_RESULT %s\n", ok ? "PASS" : "FAIL");
  Serial.println("CALIBRATE_ALL_END");
}

// ==========================================================================
// Command surface
// ==========================================================================

static void runSessionBegin(uint32_t hostSessionId) {
  Serial.println("SESSION_BEGIN_BEGIN");
  if (hostSessionId == 0) {
    Serial.println("SESSION_BEGIN_REFUSED REASON=ZERO_HOST_SESSION_ID");
    Serial.println("SESSION_BEGIN_RESULT REFUSED");
    Serial.println("SESSION_BEGIN_END");
    return;
  }
  if (sessionState == SESSION_FAULT) {
    Serial.printf("SESSION_BEGIN_REFUSED REASON=SESSION_FAULT_LATCHED FAULT=%s\n",
                  lastFault);
    Serial.println("CUT_SERVO_POWER_NOW");
    Serial.println("SESSION_BEGIN_RESULT REFUSED");
    Serial.println("SESSION_BEGIN_END");
    return;
  }
  if (activeHostSessionId == hostSessionId) {
    Serial.printf("SESSION_BEGIN_ACTIVE_HOST_SESSION_ID=%08lX\n",
                  (unsigned long)activeHostSessionId);
    Serial.println("SESSION_BEGIN_RESULT PASS");
    Serial.println("SESSION_BEGIN_END");
    return;
  }

  // A new lease never inherits state from a previous host process.
  if (!runSafeOffQuiet()) {
    setFault("SESSION_BEGIN_SAFE_OFF_FAILED");
    Serial.println("SESSION_BEGIN_RESULT FAIL");
    Serial.println("SESSION_BEGIN_END");
    return;
  }
  invalidateCensus(nullptr);
  clearSessionEvidence("NEW_HOST_SESSION");
  activeHostSessionId = hostSessionId;
  ++sessionGeneration;
  sessionState = SESSION_IDLE;
  snprintf(lastFault, sizeof(lastFault), "%s", "NONE");

  Serial.printf("SESSION_BEGIN_BOOT_SESSION_ID=%08lX\n",
                (unsigned long)bootSessionId);
  Serial.printf("SESSION_BEGIN_ACTIVE_HOST_SESSION_ID=%08lX\n",
                (unsigned long)activeHostSessionId);
  Serial.printf("SESSION_BEGIN_GENERATION=%lu\n",
                (unsigned long)sessionGeneration);
  Serial.println("SESSION_BEGIN_RESULT PASS");
  Serial.println("SESSION_BEGIN_END");
}

static void runSessionEnd() {
  Serial.println("SESSION_END_BEGIN");
  const bool safe = runSafeOffQuiet();
  invalidateCensus(nullptr);
  clearSessionEvidence("HOST_SESSION_ENDED");
  activeHostSessionId = 0;
  const bool endOk = safe && sessionState != SESSION_FAULT;
  if (endOk) sessionState = SESSION_IDLE;
  Serial.printf("SESSION_END_SAFE_OFF_VERIFIED=%s\n", safe ? "YES" : "NO");
  Serial.printf("SESSION_END_FAULT_LATCHED=%s\n",
                sessionState == SESSION_FAULT ? "YES" : "NO");
  Serial.printf("SESSION_END_RESULT %s\n", endOk ? "PASS" : "FAIL");
  if (!endOk) Serial.println("CUT_SERVO_POWER_NOW");
  Serial.println("SESSION_END_END");
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  @SESSION_BEGIN <8-hex-host-id>");
  Serial.println("  @SESSION_END");
  Serial.println("  @STATUS");
  Serial.println("  @CENSUS");
  Serial.println("  @CAPTURE_Q0 <samples>");
  Serial.println("  @WITNESS_DIRECTION <bus_id> <Q_PLUS_RAW_INCREASES|Q_PLUS_RAW_DECREASES> CONFIRM");
  Serial.println("  @APPROVE_BOOTSTRAP CONFIRM");
  Serial.println("  @CHARACTERIZE_JOINT <bus_id>");
  Serial.println("  @CALIBRATE_JOINT <bus_id>");
  Serial.println("  @CALIBRATE_LEG <LF|RF|RH|LH>");
  Serial.println("  @CALIBRATE_ALL");
  Serial.println("  @SAFE_OFF");
  Serial.println("  @HELP");
  Serial.println();
}

// Operator confirmation that the conservative bootstrap envelope may be used
// for H3 in THIS session. Requires the build flag as well; neither alone is
// sufficient, and a census invalidation clears it.
static void runApproveBootstrap() {
  Serial.println("APPROVE_BOOTSTRAP_BEGIN");
  if (!requireActiveSession("APPROVE_BOOTSTRAP") ||
      !requireFreshCensus("APPROVE_BOOTSTRAP")) {
    Serial.println("APPROVE_BOOTSTRAP_RESULT REFUSED");
    Serial.println("APPROVE_BOOTSTRAP_END");
    return;
  }
  if (!allManualQ0Usable()) {
    Serial.println("APPROVE_BOOTSTRAP_REFUSED REASON=MANUAL_Q0_CAPTURE_REQUIRED");
    Serial.println("APPROVE_BOOTSTRAP_RESULT REFUSED");
    Serial.println("APPROVE_BOOTSTRAP_END");
    return;
  }
  if (!H3_BOOTSTRAP_BUILD_APPROVED) {
    Serial.println("APPROVE_BOOTSTRAP_REFUSED REASON=BUILD_NOT_COMPILED_WITH_BOOTSTRAP");
    Serial.println("APPROVE_BOOTSTRAP_RESULT REFUSED");
    Serial.println("APPROVE_BOOTSTRAP_END");
    return;
  }
  if (AUTHORIZED_STAGE < H3_JOINT_CHARACTERIZE) {
    Serial.printf("APPROVE_BOOTSTRAP_REFUSED REASON=STAGE_BELOW_H3 AUTHORIZED=H%u\n",
                  (unsigned)AUTHORIZED_STAGE);
    Serial.println("APPROVE_BOOTSTRAP_RESULT REFUSED");
    Serial.println("APPROVE_BOOTSTRAP_END");
    return;
  }
  bootstrapSessionApproved = true;
  bootstrapSessionGeneration = sessionGeneration;
  bootstrapCensusEpoch = censusEpoch;
  const FlcMotionEnvelope envelope = flcBootstrapEnvelope();
  Serial.printf("APPROVE_BOOTSTRAP_ENVELOPE ORIGIN=%s TORQUE_LIMIT=%u SPEED=%u ACC=%u "
                "RETREAT=%u\n",
                flcParameterOriginLabel(envelope.origin), envelope.torqueLimit,
                envelope.goalSpeed, envelope.acceleration, envelope.retreatTicks);
  Serial.println("APPROVE_BOOTSTRAP_NOTE=NOT_A_MEASUREMENT_NEVER_CANONICAL");
  Serial.println("APPROVE_BOOTSTRAP_RESULT PASS");
  Serial.println("APPROVE_BOOTSTRAP_END");
}

static int legFromLabel(const char *s) {
  for (int i = 0; i < LEG_COUNT; ++i) {
    if (strcasecmp(s, LEG_LABEL[i]) == 0) return i;
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(1500);

  bootSessionId = esp_random();
  if (bootSessionId == 0) bootSessionId = 1;

  ServoUART.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &ServoUART;

  invalidateCensus(nullptr);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" MATDOG FULL LEG CALIBRATOR V1");
  Serial.println("====================================");
  Serial.printf("Firmware         : %s %s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
  Serial.printf("Build Git SHA    : %s\n", FLC_BUILD_GIT_SHA);
  Serial.printf("Boot session id  : %08lX\n", (unsigned long)bootSessionId);
  Serial.printf("Protocol         : %s (%s)\n", PROTOCOL_ID, PROTOCOL_SCOPE);
  Serial.printf("Servo UART       : %lu baud\n", (unsigned long)SERVO_BAUD);
  Serial.printf("TX               : GPIO%d\n", SERVO_TX_PIN);
  Serial.printf("RX               : GPIO%d\n", SERVO_RX_PIN);
  Serial.printf("Authorized stage : H%u\n", (unsigned)AUTHORIZED_STAGE);
  Serial.println();
  Serial.println("Startup motion   : IMPOSSIBLE BY DESIGN");
  Serial.println("EEPROM writes    : NOT IMPLEMENTED");
  Serial.println("CalibrationOfs   : NOT IMPLEMENTED");
  Serial.println("Factory reset    : NOT IMPLEMENTED");
  Serial.println("ID change        : NOT IMPLEMENTED");
  Serial.println("Broadcast writes : NOT IMPLEMENTED");
  Serial.println("Station in path  : NO");
  Serial.println();
  Serial.println("FULL_LEG_CALIBRATOR_READY");

  printHelp();
}

void loop() {
  if (!Serial.available()) {
    delay(1);
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "@HELP") { printHelp(); return; }
  if (cmd == "@STATUS") { runStatus(); return; }
  if (cmd == "@SAFE_OFF") { runSafeOff(); return; }
  if (cmd == "@SESSION_END") { runSessionEnd(); return; }
  if (cmd == "@CENSUS") { runCensus(); return; }
  if (cmd == "@CALIBRATE_ALL") { runCalibrateAll(); return; }
  if (cmd == "@APPROVE_BOOTSTRAP CONFIRM") { runApproveBootstrap(); return; }

  int value = -1;
  char extra = '\0';

  unsigned long hostSession = 0;
  if (sscanf(cmd.c_str(), "@SESSION_BEGIN %lx %c", &hostSession, &extra) == 1) {
    runSessionBegin((uint32_t)hostSession);
    return;
  }

  int witnessId = -1;
  char witnessSemantic[32] = {0};
  char confirm[16] = {0};
  if (sscanf(cmd.c_str(), "@WITNESS_DIRECTION %d %31s %15s %c", &witnessId,
             witnessSemantic, confirm, &extra) == 3) {
    if (strcmp(confirm, "CONFIRM") != 0) {
      runWitnessDirection(witnessId, 0, witnessSemantic);
    } else if (strcmp(witnessSemantic, "Q_PLUS_RAW_INCREASES") == 0) {
      runWitnessDirection(witnessId, 1, witnessSemantic);
    } else if (strcmp(witnessSemantic, "Q_PLUS_RAW_DECREASES") == 0) {
      runWitnessDirection(witnessId, -1, witnessSemantic);
    } else {
      runWitnessDirection(witnessId, 0, witnessSemantic);
    }
    return;
  }

  if (sscanf(cmd.c_str(), "@CHARACTERIZE_JOINT %d %c", &value, &extra) == 1) {
    runCharacterizeJoint(value);
    return;
  }

  if (sscanf(cmd.c_str(), "@CAPTURE_Q0 %d %c", &value, &extra) == 1) {
    runCaptureManualQ0((uint16_t)constrain(value, 0, (int)Q0_SAMPLES_MAX));
    return;
  }
  if (cmd == "@CAPTURE_Q0") { runCaptureManualQ0(Q0_SAMPLES_DEFAULT); return; }

  if (sscanf(cmd.c_str(), "@CALIBRATE_JOINT %d %c", &value, &extra) == 1) {
    runCalibrateJoint(value);
    return;
  }

  char legBuf[8] = {0};
  if (sscanf(cmd.c_str(), "@CALIBRATE_LEG %7s %c", legBuf, &extra) == 1) {
    runCalibrateLeg(legFromLabel(legBuf));
    return;
  }

  Serial.println("ERROR: INVALID_COMMAND");
  printHelp();
}
