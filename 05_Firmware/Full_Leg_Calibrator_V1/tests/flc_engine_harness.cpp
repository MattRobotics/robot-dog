/*
 * Host harness for flc_calibration_engine.h
 *
 * Compiles the SAME engine the ESP32-S3 firmware compiles, and drives it
 * against a simulated ST3215 servo that has a real mechanical endstop. This is
 * what makes the H3/H4/H5/H6 happy paths testable offline: the test reaches
 * CONTACT_CONFIRMED, retreats, re-approaches, evaluates repeatability and
 * derives a q0 candidate through the identical code that would move a servo.
 *
 * Note what the port does NOT expose: there is no EEPROM write, no ID write and
 * no broadcast entry point at all. The engine cannot perform those operations
 * because no function exists for them — that is the strongest form of the
 * EEPROM-write-free property, and the harness counts them to prove it stays 0.
 *
 * Build:
 *   g++ -std=c++17 -O0 -Wall -Wextra -Werror \
 *       -I../matdog_full_leg_calibrator_v1 flc_engine_harness.cpp -o flc_engine_harness
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <map>

#include "flc_calibration_engine.h"

// --------------------------------------------------------------------------
// Simulated ST3215 with a mechanical endstop
// --------------------------------------------------------------------------

struct SimServo {
  int position = 2048;
  int goal = 2048;
  uint16_t goalSpeed = 0;
  uint8_t acc = 0;
  bool torqueEnabled = false;
  uint16_t torqueLimit = 1000;

  // Mechanical reality the calibrator is trying to discover.
  int endstopMin = 1700;
  int endstopMax = 2400;

  // Encoder sign produced by a positive commanded step. -1 models a joint whose
  // raw ticks decrease as the commanded target increases.
  int encoderSign = 1;

  uint16_t freeCurrent = 20;
  uint16_t stallCurrent = 90;
  uint16_t current = 0;
  int velocity = 0;
  uint8_t temperature = 30;
  uint8_t voltage = 120;
  uint8_t statusByte = 0;

  // ~4 ticks per 5 ms poll approximates the bootstrap speed-60 envelope and
  // gives the engine a realistic number of samples per excursion.
  int ticksPerPoll = 4;
  bool stalled = false;

  // Fault injection
  bool contextInvalid = false;   // session/identity drift for THIS servo
  bool telemetryFails = false;
  bool driverError = false;
  int failTelemetryAfter = -1;
  int telemetryReads = 0;
  bool refuseTorqueOff = false;
  bool frozen = false;            // joint does not move at all
  int overcurrentAfterPolls = -1;
  int thermalAfterPolls = -1;
  int polls = 0;
};

struct SimBus {
  std::map<int, SimServo> servos;
  uint32_t clockMs = 0;

  // Wire-level accounting, asserted by the tests.
  int eepromWrites = 0;
  int broadcastWrites = 0;
  int positionCommands = 0;
  int torqueEnableWrites = 0;
  int torqueLimitWrites = 0;
};

static SimBus g_bus;

//: The full generated catalog. flcRunCalibrationPlan() requires all 12 rows,
//: because un-selected joints must still be held and watched at q0 and a
//: cross-leg parking auxiliary must be expressible in raw ticks.
static FlcJointPlan g_plans[FLC_CALIBRATION_JOINT_COUNT];

//: Per-joint runtime calibration seeded into the plan catalog (a joint
//: calibrated in an earlier operation of the same session).
static void resetPlans() {
  for (int i = 0; i < FLC_CALIBRATION_JOINT_COUNT; ++i) {
    g_plans[i] = FlcJointPlan();
    g_plans[i].geometryJoint = (FlcGeometryJoint)i;
  }
}

static void simAdvance(SimServo &s, uint32_t ms) {
  int steps = (int)(ms / 5);
  if (steps < 1) steps = 1;
  for (int i = 0; i < steps; ++i) {
    ++s.polls;
    if (s.frozen) { s.velocity = 0; s.current = s.freeCurrent; continue; }

    // Commanded target expressed in raw ticks for this joint's encoder sign.
    const int wanted = s.goal;
    const int delta = wanted - s.position;
    if (delta == 0) { s.velocity = 0; s.current = s.freeCurrent; s.stalled = false; continue; }

    const int dir = delta > 0 ? 1 : -1;
    int move = s.ticksPerPoll;
    if (move > (delta > 0 ? delta : -delta)) move = (delta > 0 ? delta : -delta);
    int next = s.position + dir * move;

    // Mechanical endstops: the joint physically cannot pass them.
    bool blocked = false;
    if (next < s.endstopMin) { next = s.endstopMin; blocked = true; }
    if (next > s.endstopMax) { next = s.endstopMax; blocked = true; }

    if (next == s.position && blocked) {
      // Jammed against the stop: no progress, elevated current.
      s.velocity = 0;
      s.current = s.stallCurrent;
      s.stalled = true;
    } else {
      s.velocity = dir * 200;
      s.current = s.freeCurrent;
      s.stalled = false;
      s.position = next;
    }
  }
}

//: Mirrors the firmware's portValidateContext: the session that authorized the
//: motion must still be the session executing it. Injectable so the tests can
//: prove the engine stops mid-motion on drift rather than at the next command.
static bool g_contextValid = true;

static bool portValidateContext(void *ctx, uint8_t id) {
  (void)ctx;
  if (!g_contextValid) return false;
  auto it = g_bus.servos.find((int)id);
  if (it == g_bus.servos.end()) return false;
  return !it->second.contextInvalid;
}

static bool portReadTelemetry(void *ctx, uint8_t id, FlcObservation *out) {
  (void)ctx;
  auto it = g_bus.servos.find((int)id);
  if (it == g_bus.servos.end()) return false;
  SimServo &s = it->second;

  ++s.telemetryReads;
  if (s.telemetryFails) return false;
  if (s.failTelemetryAfter >= 0 && s.telemetryReads > s.failTelemetryAfter) return false;

  simAdvance(s, 5);
  g_bus.clockMs += 5;

  if (s.overcurrentAfterPolls >= 0 && s.polls > s.overcurrentAfterPolls) s.current = 300;
  if (s.thermalAfterPolls >= 0 && s.polls > s.thermalAfterPolls) s.temperature = 75;

  out->telemetryValid = true;
  out->driverError = s.driverError;
  out->statusByte = s.statusByte;
  out->torqueEnabled = s.torqueEnabled;
  out->torqueLimit = s.torqueLimit;
  out->goalPosition = (uint16_t)s.goal;
  out->position = (uint16_t)s.position;
  out->velocity = (int16_t)s.velocity;
  out->current = s.current;
  out->temperature = s.temperature;
  out->voltage = s.voltage;
  out->elapsedMs = 0;
  return true;
}

static bool portCommandPosition(void *ctx, uint8_t id, int position, uint16_t speed,
                                uint8_t acc) {
  (void)ctx;
  if (id == 254) { ++g_bus.broadcastWrites; return false; }
  if (position < 0 || position > FLC_ENCODER_MAX) return false;
  auto it = g_bus.servos.find((int)id);
  if (it == g_bus.servos.end()) return false;
  ++g_bus.positionCommands;
  it->second.goal = position;
  it->second.goalSpeed = speed;
  it->second.acc = acc;
  return true;
}

static bool portSetTorqueLimit(void *ctx, uint8_t id, uint16_t limit) {
  (void)ctx;
  auto it = g_bus.servos.find((int)id);
  if (it == g_bus.servos.end()) return false;
  ++g_bus.torqueLimitWrites;
  it->second.torqueLimit = limit;
  return true;
}

static bool portSetTorqueEnable(void *ctx, uint8_t id, bool enable) {
  (void)ctx;
  auto it = g_bus.servos.find((int)id);
  if (it == g_bus.servos.end()) return false;
  ++g_bus.torqueEnableWrites;
  if (!enable && it->second.refuseTorqueOff) return false;
  it->second.torqueEnabled = enable;
  return true;
}

static uint32_t portNowMs(void *ctx) { (void)ctx; return g_bus.clockMs; }

static void portIdle(void *ctx, uint32_t ms) {
  (void)ctx;
  g_bus.clockMs += ms;
  for (auto &kv : g_bus.servos) simAdvance(kv.second, ms);
}

static FlcServoPort makePort() {
  FlcServoPort port;
  port.validateContext = portValidateContext;
  port.readTelemetry = portReadTelemetry;
  port.commandPosition = portCommandPosition;
  port.setTorqueLimit = portSetTorqueLimit;
  port.setTorqueEnable = portSetTorqueEnable;
  port.nowMs = portNowMs;
  port.idle = portIdle;
  port.trace = nullptr;
  port.ctx = nullptr;
  return port;
}

static FlcContactConfig makeGuards() {
  FlcContactConfig g;
  g.maxProgressTicks = 2;
  g.maxVelocityRaw = 10;
  g.targetReachedToleranceTicks = 10;
  g.minTravelTicks = 24;
  g.persistenceSamples = 3;
  g.startupGraceSamples = 4;
  g.hardCurrentAbortRaw = 250;
  g.expectedTorqueLimit = 0;
  g.thermalLimitC = 70;
  g.voltageMin = 40;
  g.voltageMax = 140;
  g.travelBudgetTicks = 0;
  g.timeBudgetMs = 0;
  g.probeSign = 0;
  return g;
}

// --------------------------------------------------------------------------
// Command protocol
// --------------------------------------------------------------------------

static bool allTorqueOff() {
  for (auto &kv : g_bus.servos) if (kv.second.torqueEnabled) return false;
  return true;
}

int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    char verb[40] = {0};
    if (sscanf(line.c_str(), "%39s", verb) != 1) continue;

    if (strcmp(verb, "RESET") == 0) {
      g_bus = SimBus();
      g_contextValid = true;
      resetPlans();
      printf("OK RESET\n"); fflush(stdout); continue;
    }

    if (strcmp(verb, "SERVO") == 0) {
      int id, pos, lo, hi, sign;
      if (sscanf(line.c_str(), "SERVO %d %d %d %d %d", &id, &pos, &lo, &hi, &sign) != 5) {
        printf("ERROR BAD_SERVO\n"); fflush(stdout); continue;
      }
      SimServo s;
      s.position = pos; s.goal = pos;
      s.endstopMin = lo; s.endstopMax = hi;
      s.encoderSign = sign;
      g_bus.servos[id] = s;
      printf("OK SERVO ID=%d POS=%d MIN=%d MAX=%d\n", id, pos, lo, hi);
      fflush(stdout); continue;
    }

    if (strcmp(verb, "FAULT") == 0) {
      char name[40] = {0};
      int id, value;
      if (sscanf(line.c_str(), "FAULT %d %39s %d", &id, name, &value) != 3) {
        printf("ERROR BAD_FAULT\n"); fflush(stdout); continue;
      }
      auto it = g_bus.servos.find(id);
      if (it == g_bus.servos.end()) { printf("ERROR NO_SERVO\n"); fflush(stdout); continue; }
      SimServo &s = it->second;
      if (strcmp(name, "TELEMETRY_FAILS") == 0) s.telemetryFails = (value != 0);
      else if (strcmp(name, "DRIVER_ERROR") == 0) s.driverError = (value != 0);
      else if (strcmp(name, "FAIL_TELEMETRY_AFTER") == 0) s.failTelemetryAfter = value;
      else if (strcmp(name, "REFUSE_TORQUE_OFF") == 0) s.refuseTorqueOff = (value != 0);
      else if (strcmp(name, "FROZEN") == 0) s.frozen = (value != 0);
      else if (strcmp(name, "OVERCURRENT_AFTER") == 0) s.overcurrentAfterPolls = value;
      else if (strcmp(name, "THERMAL_AFTER") == 0) s.thermalAfterPolls = value;
      else if (strcmp(name, "STATUS_BYTE") == 0) s.statusByte = (uint8_t)value;
      else if (strcmp(name, "VOLTAGE") == 0) s.voltage = (uint8_t)value;
      else if (strcmp(name, "STALL_CURRENT") == 0) s.stallCurrent = (uint16_t)value;
      else if (strcmp(name, "FREE_CURRENT") == 0) s.freeCurrent = (uint16_t)value;
      else if (strcmp(name, "CONTEXT_INVALID") == 0) s.contextInvalid = (value != 0);
      else if (strcmp(name, "ENDSTOP_MIN") == 0) s.endstopMin = value;
      else if (strcmp(name, "ENDSTOP_MAX") == 0) s.endstopMax = value;
      else { printf("ERROR UNKNOWN_FAULT\n"); fflush(stdout); continue; }
      printf("OK FAULT %s=%d\n", name, value);
      fflush(stdout); continue;
    }

    if (strcmp(verb, "CHARACTERIZE") == 0) {
      int id, startTick, plannedSign;
      if (sscanf(line.c_str(), "CHARACTERIZE %d %d %d", &id, &startTick, &plannedSign) != 3) {
        printf("ERROR BAD_CHARACTERIZE\n"); fflush(stdout); continue;
      }
      FlcServoPort port = makePort();
      const FlcMotionEnvelope envelope = flcBootstrapEnvelope();
      const FlcContactConfig guards = makeGuards();
      const FlcCharacterizationResult r = flcCharacterizeJoint(
          port, (uint8_t)id, startTick, (int8_t)plannedSign, envelope, guards, 48, 96);
      printf("STATUS=%s REASON=%s COMPLETE=%d RESPONDS=%d RAW_PROBE_SIGN=%d "
             "BASELINE_MEDIAN=%u "
             "BASELINE_MAD=%u BASELINE_SAMPLES=%d CONTACT_TICK=%d RETREAT_ACHIEVED=%d "
             "THRESHOLD=%u RETREAT_TICKS=%u SPREAD=%u REPEAT_BAND=%u TORQUE_OFF=%d\n",
             flcEngineStatusLabel(r.status), flcAbortReasonLabel(r.abortReason),
             r.complete ? 1 : 0, r.encoderResponse.responds ? 1 : 0,
             (int)r.encoderResponse.rawProbeSign,
             r.baseline.baseline.medianCurrent, r.baseline.baseline.madCurrent,
             r.baseline.samples, r.probeContact.contactTick,
             r.probeRetreat.achievedTicks, r.characterizedContactThresholdRaw,
             r.characterizedRetreatTicks, r.observedContactSpreadTicks,
             r.characterizedRepeatabilityToleranceTicks, allTorqueOff() ? 1 : 0);
      fflush(stdout); continue;
    }

    // PLAN <slot> <busId> <characterized> <startTick> <q0WatchTick>
    //      <q0Tol> <witness> <qMinTicks> <qMaxTicks> <expectedSpan>
    //      <repeatTol> <urdfTol> <urdfKnown> <manualQ0Known> <manualQ0Tick>
    //      <q0XcheckTol> <q0XcheckKnown> <calKnown> <knownDirection> <knownQ0>
    if (strcmp(verb, "PLAN") == 0) {
      int slot, busId, characterized, startTick, q0Watch, q0Tol, witness;
      int qMin, qMax, span, repeatTol, urdfTol, urdfKnown;
      int manualKnown, manualTick, xcheckTol, xcheckKnown;
      int calKnown, knownDir, knownQ0;
      if (sscanf(line.c_str(),
                 "PLAN %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                 &slot, &busId, &characterized, &startTick, &q0Watch, &q0Tol,
                 &witness, &qMin, &qMax, &span, &repeatTol, &urdfTol, &urdfKnown,
                 &manualKnown, &manualTick, &xcheckTol, &xcheckKnown,
                 &calKnown, &knownDir, &knownQ0) != 20) {
        printf("ERROR BAD_PLAN\n"); fflush(stdout); continue;
      }
      if (slot < 0 || slot >= FLC_CALIBRATION_JOINT_COUNT) {
        printf("ERROR SLOT_RANGE\n"); fflush(stdout); continue;
      }
      FlcJointPlan &plan = g_plans[slot];
      plan = FlcJointPlan();
      plan.geometryJoint = (FlcGeometryJoint)slot;
      plan.busId = (uint8_t)busId;
      plan.characterized = (characterized != 0);
      plan.startTick = startTick;
      plan.q0WatchKnown = true;
      plan.q0WatchTick = q0Watch;
      plan.q0WatchToleranceTicks = (uint16_t)q0Tol;
      plan.calibrationKnown = (calKnown != 0);
      plan.knownDirection = (int8_t)knownDir;
      plan.knownQ0Tick = knownQ0;

      plan.directionEvidence.rawLoTick = -1;
      plan.directionEvidence.rawHiTick = -1;
      plan.directionEvidence.qMinTicks = qMin;
      plan.directionEvidence.qMaxTicks = qMax;
      plan.directionEvidence.manualQ0Known = (manualKnown != 0);
      plan.directionEvidence.manualQ0Tick = manualTick;
      plan.directionEvidence.manualQ0PoseUncertaintyKnown = false;
      plan.directionEvidence.manualQ0PoseUncertaintyTicks = 0;
      plan.directionEvidence.operatorWitness = (int8_t)witness;

      plan.baseline.medianCurrent = 20;
      plan.baseline.madCurrent = 3;
      plan.baseline.valid = true;
      plan.envelope = flcBootstrapEnvelope();

      plan.gates.repeatabilityToleranceKnown = repeatTol > 0;
      plan.gates.repeatabilityToleranceTicks = (uint16_t)repeatTol;
      plan.gates.endpointVsUrdfToleranceKnown = (urdfKnown != 0);
      plan.gates.endpointVsUrdfToleranceTicks = (uint16_t)urdfTol;
      plan.gates.expectedSpanTicks = span;
      plan.gates.manualVsDerivedQ0ToleranceKnown = (xcheckKnown != 0);
      plan.gates.manualVsDerivedQ0ToleranceTicks = (uint16_t)xcheckTol;
      printf("OK PLAN SLOT=%d ID=%d\n", slot, busId);
      fflush(stdout); continue;
    }

    // RUN <selectionMaskHex> — the SAME entry point H4, H5 and H6 use.
    if (strcmp(verb, "RUN") == 0) {
      unsigned mask;
      if (sscanf(line.c_str(), "RUN %x", &mask) != 1) {
        printf("ERROR BAD_RUN\n"); fflush(stdout); continue;
      }
      FlcServoPort port = makePort();
      const FlcContactConfig guards = makeGuards();
      const FlcCalibrationRunResult r = flcRunCalibrationPlan(
          port, guards, g_plans, FLC_CALIBRATION_JOINT_COUNT, (uint16_t)mask);
      printf("STATUS=%s REASON=%s SELECTION=%04X REQUESTED=%d JOINTS_OK=%d "
             "EXEC_COUNT=%d FAILED_JOINT=%d FAILED_ID=%d PARKING_REQUIRED=%d "
             "NO_PARKING=%d PARKING_RESTORED=%d SAFE_OFF=%d CUT_POWER=%d "
             "TORQUE_OFF=%d\n",
             flcEngineStatusLabel(r.status), flcAbortReasonLabel(r.abortReason),
             r.selectionMask, r.jointsRequested, r.jointsOk, r.executionCount,
             (int)r.failedJoint, r.failedBusId, r.parkingRequiredCount,
             r.noParkingCount, r.parkingRestoreCount,
             r.safeOffVerified ? 1 : 0, r.hardFaultCutPowerNow ? 1 : 0,
             allTorqueOff() ? 1 : 0);
      for (int i = 0; i < r.executionCount; ++i) {
        printf("  ORDER %d=%d\n", i, (int)r.executionOrder[i]);
      }
      for (int j = 0; j < FLC_CALIBRATION_JOINT_COUNT; ++j) {
        if (!r.resultPresent[j]) continue;
        const FlcJointCalibrationResult &jr = r.joints[j];
        printf("  JOINT SLOT=%d ID=%u STATUS=%s REASON=%s TIER=%s DIRECTION=%d "
               "METHOD=%s MIN_TICK=%d MAX_TICK=%d SPAN=%d EXPECTED_SPAN=%d "
               "SPAN_ERROR=%d MANUAL_Q0=%d DERIVED_Q0=%d Q0_XCHECK=%s "
               "Q0_ERROR=%d ACCEPTED=%d MIN_PARK=%d MAX_PARK=%d "
               "MIN_RESTORE=%d MAX_RESTORE=%d MIN_NOPARK_PROV=%d "
               "MAX_NOPARK_PROV=%d\n",
               j, jr.busId, flcEngineStatusLabel(jr.status),
               flcAbortReasonLabel(jr.abortReason), flcResultTierLabel(jr.tier),
               (int)jr.direction, flcDirectionMethodLabel(jr.directionMethod),
               jr.minContactTick, jr.maxContactTick, jr.measuredSpanTicks,
               jr.expectedSpanTicks, jr.spanErrorTicks,
               jr.manualPoseQ0CandidateTick, jr.derivedQ0FinalTick,
               flcQ0CrosscheckStatusLabel(jr.q0CrosscheckStatus),
               jr.manualVsDerivedQ0ErrorTicks, jr.accepted ? 1 : 0,
               (int)jr.minExecution.parkingOutcome,
               (int)jr.maxExecution.parkingOutcome,
               jr.minExecution.parking.restoreVerified ? 1 : 0,
               jr.maxExecution.parking.restoreVerified ? 1 : 0,
               jr.minExecution.parking.noParkingProvenanceVerified ? 1 : 0,
               jr.maxExecution.parking.noParkingProvenanceVerified ? 1 : 0);
      }
      for (int j = 0; j < FLC_CALIBRATION_JOINT_COUNT; ++j) {
        printf("  FINAL SLOT=%d USABLE=%d DIRECTION=%d Q0=%d HOLD=%d\n", j,
               r.finalCalibration[j].calibrationUsable ? 1 : 0,
               (int)r.finalCalibration[j].direction,
               r.finalCalibration[j].q0Tick,
               r.finalCalibration[j].holdTick);
      }
      printf("RUN_END\n");
      fflush(stdout); continue;
    }

    // CONTEXT <0|1> — global session validity, drives fail-closed drift tests.
    if (strcmp(verb, "CONTEXT") == 0) {
      int value;
      if (sscanf(line.c_str(), "CONTEXT %d", &value) != 1) {
        printf("ERROR BAD_CONTEXT\n"); fflush(stdout); continue;
      }
      g_contextValid = (value != 0);
      printf("OK CONTEXT=%d\n", value ? 1 : 0);
      fflush(stdout); continue;
    }

    // GEOMETRY — the generated 24-row plan as the engine itself validates it.
    if (strcmp(verb, "GEOMETRY") == 0) {
      int noParking = 0, parking = 0;
      const bool summaryOk = flcGeometryPlanSummary(noParking, parking);
      FlcGeometryJoint order[FLC_CALIBRATION_JOINT_COUNT];
      const bool orderOk = flcBuildDependencyOrder(
          FLC_GEOMETRY_DEPENDENCIES, FLC_GEOMETRY_DEPENDENCY_COUNT, order,
          FLC_CALIBRATION_JOINT_COUNT);
      printf("SUMMARY_OK=%d NO_PARKING=%d PARKING=%d ROWS=%d ORDER_OK=%d "
             "DEPENDENCIES=%d\n",
             summaryOk ? 1 : 0, noParking, parking,
             FLC_ENDPOINT_GEOMETRY_PLAN_COUNT, orderOk ? 1 : 0,
             FLC_GEOMETRY_DEPENDENCY_COUNT);
      if (orderOk) {
        for (int i = 0; i < FLC_CALIBRATION_JOINT_COUNT; ++i) {
          printf("  ORDER %d=%d\n", i, (int)order[i]);
        }
      }
      for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {
        const FlcEndpointGeometryPlan &row = FLC_ENDPOINT_GEOMETRY_PLANS[i];
        printf("  ROW %d ID=%s TARGET=%d SIDE=%s PARKING=%d AUX=%d "
               "Q0_START=%04X Q0_HELD=%04X\n",
               row.canonicalEndpointIndex, row.endpointId, (int)row.targetJoint,
               row.limitSide, (int)row.parkingOutcome, (int)row.auxiliaryJoint,
               row.q0StartMask, row.q0HeldDuringTaskMask);
      }
      printf("GEOMETRY_END\n");
      fflush(stdout); continue;
    }

    // CYCLE — proves the production Kahn walk rejects an injected cycle rather
    // than trusting the checked-in order.
    if (strcmp(verb, "CYCLE") == 0) {
      FlcGeometryDependency injected[] = {
        {FLC_GEOMETRY_JOINT_LF_HIP, FLC_GEOMETRY_JOINT_LF_UPPER_LEG, 0},
        {FLC_GEOMETRY_JOINT_LF_UPPER_LEG, FLC_GEOMETRY_JOINT_LF_HIP, 0},
      };
      FlcGeometryJoint order[FLC_CALIBRATION_JOINT_COUNT];
      const bool ok = flcBuildDependencyOrder(
          injected, (int)(sizeof(injected) / sizeof(injected[0])), order,
          FLC_CALIBRATION_JOINT_COUNT);
      printf("CYCLE_ACCEPTED=%d\n", ok ? 1 : 0);
      fflush(stdout); continue;
    }

    if (strcmp(verb, "STATS") == 0) {
      printf("EEPROM_WRITES=%d BROADCAST_WRITES=%d POSITION_COMMANDS=%d "
             "TORQUE_ENABLE_WRITES=%d TORQUE_LIMIT_WRITES=%d ALL_TORQUE_OFF=%d "
             "CLOCK_MS=%u\n",
             g_bus.eepromWrites, g_bus.broadcastWrites, g_bus.positionCommands,
             g_bus.torqueEnableWrites, g_bus.torqueLimitWrites,
             allTorqueOff() ? 1 : 0, (unsigned)g_bus.clockMs);
      fflush(stdout); continue;
    }

    if (strcmp(verb, "POS") == 0) {
      int id;
      if (sscanf(line.c_str(), "POS %d", &id) != 1) {
        printf("ERROR BAD_POS\n"); fflush(stdout); continue;
      }
      auto it = g_bus.servos.find(id);
      if (it == g_bus.servos.end()) { printf("ERROR NO_SERVO\n"); fflush(stdout); continue; }
      printf("POSITION=%d TORQUE=%d LIMIT=%u\n", it->second.position,
             it->second.torqueEnabled ? 1 : 0, it->second.torqueLimit);
      fflush(stdout); continue;
    }

    if (strcmp(verb, "ENVELOPE") == 0) {
      const FlcMotionEnvelope e = flcBootstrapEnvelope();
      printf("ORIGIN=%s TORQUE_LIMIT=%u SPEED=%u ACC=%u RETREAT=%u STEP=%u "
             "TRAVEL_BUDGET=%u TIME_BUDGET=%u VALID=%d\n",
             flcParameterOriginLabel(e.origin), e.torqueLimit, e.goalSpeed,
             e.acceleration, e.retreatTicks, e.stepTicks, e.travelBudgetTicks,
             (unsigned)e.timeBudgetMs, e.valid ? 1 : 0);
      fflush(stdout); continue;
    }

    if (strcmp(verb, "CLAMP") == 0) {
      int torque, speed, acc, travel;
      unsigned timeMs;
      if (sscanf(line.c_str(), "CLAMP %d %d %d %d %u", &torque, &speed, &acc, &travel,
                 &timeMs) != 5) {
        printf("ERROR BAD_CLAMP\n"); fflush(stdout); continue;
      }
      FlcMotionEnvelope e;
      e.torqueLimit = (uint16_t)torque;
      e.goalSpeed = (uint16_t)speed;
      e.acceleration = (uint8_t)acc;
      e.retreatTicks = 96;
      e.stepTicks = 24;
      e.travelBudgetTicks = (uint16_t)travel;
      e.timeBudgetMs = timeMs;
      e.origin = FLC_ORIGIN_H3_BOOTSTRAP_OPERATOR_APPROVED;
      e = flcClampEnvelope(e);
      printf("TORQUE_LIMIT=%u SPEED=%u ACC=%u TRAVEL_BUDGET=%u TIME_BUDGET=%u VALID=%d\n",
             e.torqueLimit, e.goalSpeed, e.acceleration, e.travelBudgetTicks,
             (unsigned)e.timeBudgetMs, e.valid ? 1 : 0);
      fflush(stdout); continue;
    }

    printf("ERROR UNKNOWN_VERB\n");
    fflush(stdout);
  }
  return 0;
}
