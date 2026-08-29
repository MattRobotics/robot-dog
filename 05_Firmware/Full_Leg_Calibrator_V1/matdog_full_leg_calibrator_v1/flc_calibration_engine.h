/*
 * MATDOG FULL LEG CALIBRATOR V1 — H3/H4 calibration state machine
 *
 * PURE C++. No Arduino, no SCServo, no globals, no direct I/O, no clock of its
 * own. All hardware access goes through FlcServoPort function pointers, so the
 * identical translation unit is compiled into:
 *
 *   1. the ESP32-S3 firmware, where the port drives real ST3215 servos, and
 *   2. tests/flc_engine_harness.cpp, where the port drives a simulated servo
 *      with a real mechanical endstop.
 *
 * THIS IS THE ONLY IMPLEMENTATION OF THE MOTION DECISION LOGIC. There is no
 * Python re-implementation of approach, contact, retreat, re-approach or
 * repeatability. The Python simulator covers policy/census/session only.
 *
 * Layering:
 *   flc_contact_detector.h   elementary per-sample contact decision
 *   flc_calibration_engine.h  <-- THIS FILE: baseline, probe, contact, retreat,
 *                                 re-approach, repeatability, abort/recovery
 *   matdog_full_leg_calibrator_v1.ino  transport, adapter, parser, safety
 *
 * Every function here is bounded in both travel and time, refuses to leave the
 * unsigned 0..4095 domain, and returns a structured reason on failure.
 */

#ifndef FLC_CALIBRATION_ENGINE_H
#define FLC_CALIBRATION_ENGINE_H

#include <stdint.h>

#include "flc_contact_detector.h"
#include "flc_leg_plan.h"
#include "flc_stage_config.h"

// --------------------------------------------------------------------------
// Where a runtime number came from. A bootstrap envelope and a characterized
// result are different things and can never be mistaken for one another.
// --------------------------------------------------------------------------

enum FlcParameterOrigin {
  FLC_ORIGIN_NONE = 0,
  //: operator-approved conservative first-motion envelope; NOT a measurement
  FLC_ORIGIN_H3_BOOTSTRAP_OPERATOR_APPROVED,
  //: measured on THIS build by a completed H3 characterization
  FLC_ORIGIN_CHARACTERIZED_CURRENT_HARDWARE,
};

inline const char *flcParameterOriginLabel(int origin) {
  switch (origin) {
    case FLC_ORIGIN_H3_BOOTSTRAP_OPERATOR_APPROVED:
      return "H3_BOOTSTRAP_OPERATOR_APPROVED";
    case FLC_ORIGIN_CHARACTERIZED_CURRENT_HARDWARE:
      return "CHARACTERIZED_CURRENT_HARDWARE";
    default:
      return "NONE";
  }
}

// --------------------------------------------------------------------------
// Motion envelope. Clamped against the absolute ceilings on construction, so a
// bad build flag cannot widen it.
// --------------------------------------------------------------------------

struct FlcMotionEnvelope {
  uint16_t torqueLimit;
  uint16_t goalSpeed;
  uint8_t acceleration;
  uint16_t retreatTicks;
  uint16_t stepTicks;
  uint16_t travelBudgetTicks;
  uint32_t timeBudgetMs;
  int origin;
  bool valid;
};

inline FlcMotionEnvelope flcClampEnvelope(FlcMotionEnvelope e) {
  if (e.torqueLimit > FLC_ABSOLUTE_MAX_TORQUE_LIMIT) e.torqueLimit = FLC_ABSOLUTE_MAX_TORQUE_LIMIT;
  if (e.goalSpeed > FLC_ABSOLUTE_MAX_GOAL_SPEED) e.goalSpeed = FLC_ABSOLUTE_MAX_GOAL_SPEED;
  if (e.acceleration > FLC_ABSOLUTE_MAX_ACCELERATION) e.acceleration = FLC_ABSOLUTE_MAX_ACCELERATION;
  if (e.travelBudgetTicks > FLC_ABSOLUTE_MAX_TRAVEL_BUDGET_TICKS)
    e.travelBudgetTicks = FLC_ABSOLUTE_MAX_TRAVEL_BUDGET_TICKS;
  if (e.timeBudgetMs > FLC_ABSOLUTE_MAX_TIME_BUDGET_MS)
    e.timeBudgetMs = FLC_ABSOLUTE_MAX_TIME_BUDGET_MS;
  e.valid = e.torqueLimit > 0 && e.goalSpeed > 0 && e.stepTicks > 0 &&
            e.travelBudgetTicks > 0 && e.timeBudgetMs > 0 &&
            e.origin != FLC_ORIGIN_NONE;
  return e;
}

//: The conservative envelope H3 is permitted to use, once approved.
inline FlcMotionEnvelope flcBootstrapEnvelope() {
  FlcMotionEnvelope e;
  e.torqueLimit = FLC_BOOTSTRAP_TORQUE_LIMIT;
  e.goalSpeed = FLC_BOOTSTRAP_GOAL_SPEED;
  e.acceleration = FLC_BOOTSTRAP_ACCELERATION;
  e.retreatTicks = FLC_BOOTSTRAP_RETREAT_TICKS;
  e.stepTicks = 24;
  e.travelBudgetTicks = 600;
  e.timeBudgetMs = 20000;
  e.origin = FLC_ORIGIN_H3_BOOTSTRAP_OPERATOR_APPROVED;
  return flcClampEnvelope(e);
}

// --------------------------------------------------------------------------
// Hardware port. The engine never touches a servo except through this.
// --------------------------------------------------------------------------

struct FlcServoPort {
  //: Current host session/generation/census and servo identity are still valid.
  //: This is checked immediately before every read and every motion-capable
  //: write. Torque-OFF remains callable even after context loss.
  bool (*validateContext)(void *ctx, uint8_t id);
  //: Fresh telemetry for `id`. False on transport failure or stale data.
  bool (*readTelemetry)(void *ctx, uint8_t id, FlcObservation *out);
  //: The ONLY way the engine can move anything. Domain-checked by the caller.
  bool (*commandPosition)(void *ctx, uint8_t id, int position, uint16_t speed, uint8_t acc);
  bool (*setTorqueLimit)(void *ctx, uint8_t id, uint16_t limit);
  bool (*setTorqueEnable)(void *ctx, uint8_t id, bool enable);
  uint32_t (*nowMs)(void *ctx);
  void (*idle)(void *ctx, uint32_t ms);
  //: Optional structured trace sink; may be null.
  void (*trace)(void *ctx, const char *line);
  void *ctx;
};

inline bool flcPortUsable(const FlcServoPort &port) {
  return port.validateContext != 0 && port.readTelemetry != 0 &&
         port.commandPosition != 0 &&
         port.setTorqueLimit != 0 && port.setTorqueEnable != 0 && port.nowMs != 0;
}

// --------------------------------------------------------------------------
// Engine-level outcomes
// --------------------------------------------------------------------------

enum FlcEngineStatus {
  FLC_ENGINE_OK = 0,
  FLC_ENGINE_REFUSED_PORT,
  FLC_ENGINE_REFUSED_ENVELOPE,
  FLC_ENGINE_REFUSED_DOMAIN,
  FLC_ENGINE_ABORTED,
  FLC_ENGINE_RETREAT_FAILED,
  FLC_ENGINE_REPEATABILITY_FAILED,
  FLC_ENGINE_TORQUE_SETUP_FAILED,
  FLC_ENGINE_TELEMETRY_FAILED,
  FLC_ENGINE_DIRECTION_UNRESOLVED,
  FLC_ENGINE_BASELINE_FAILED,
  FLC_ENGINE_PREREQUISITE_DRIFT,
  FLC_ENGINE_TORQUE_OFF_FAILED,
  FLC_ENGINE_INVALID_PLAN,
  FLC_ENGINE_DEPENDENCY_CYCLE,
  FLC_ENGINE_Q0_CROSSCHECK_FAILED,
  FLC_ENGINE_GEOMETRY_INCONSISTENT,
  FLC_ENGINE_PARKING_FAILED,
  FLC_ENGINE_RESTORE_FAILED,
  FLC_ENGINE_CONTEXT_DRIFT,
};

inline const char *flcEngineStatusLabel(int status) {
  switch (status) {
    case FLC_ENGINE_OK: return "OK";
    case FLC_ENGINE_REFUSED_PORT: return "REFUSED_PORT";
    case FLC_ENGINE_REFUSED_ENVELOPE: return "REFUSED_ENVELOPE";
    case FLC_ENGINE_REFUSED_DOMAIN: return "REFUSED_DOMAIN";
    case FLC_ENGINE_ABORTED: return "ABORTED";
    case FLC_ENGINE_RETREAT_FAILED: return "RETREAT_FAILED";
    case FLC_ENGINE_REPEATABILITY_FAILED: return "REPEATABILITY_FAILED";
    case FLC_ENGINE_TORQUE_SETUP_FAILED: return "TORQUE_SETUP_FAILED";
    case FLC_ENGINE_TELEMETRY_FAILED: return "TELEMETRY_FAILED";
    case FLC_ENGINE_DIRECTION_UNRESOLVED: return "DIRECTION_UNRESOLVED";
    case FLC_ENGINE_BASELINE_FAILED: return "BASELINE_FAILED";
    case FLC_ENGINE_PREREQUISITE_DRIFT: return "PREREQUISITE_DRIFT";
    case FLC_ENGINE_TORQUE_OFF_FAILED: return "TORQUE_OFF_FAILED";
    case FLC_ENGINE_INVALID_PLAN: return "INVALID_PLAN";
    case FLC_ENGINE_DEPENDENCY_CYCLE: return "DEPENDENCY_CYCLE";
    case FLC_ENGINE_Q0_CROSSCHECK_FAILED: return "Q0_CROSSCHECK_FAILED";
    case FLC_ENGINE_GEOMETRY_INCONSISTENT: return "GEOMETRY_INCONSISTENT";
    case FLC_ENGINE_PARKING_FAILED: return "PARKING_FAILED";
    case FLC_ENGINE_RESTORE_FAILED: return "RESTORE_FAILED";
    case FLC_ENGINE_CONTEXT_DRIFT: return "IDENTITY_OR_SESSION_DRIFT";
    default: return "UNKNOWN";
  }
}

// --------------------------------------------------------------------------
// SHARED GUARD CHECK
//
// Every segment that commands motion — direction probe, free-motion baseline,
// contact approach, retreat, return-to-neutral, parking and restore — routes
// its per-sample safety decision through this ONE function. Previously each
// segment repeated a slightly different subset, which is exactly how a guard
// goes missing from the path nobody looked at.
//
// `expectTorqueLimit` is 0 when the caller is not inside a torque transaction
// that pinned a specific limit (e.g. before flcBeginMotion has run).
// --------------------------------------------------------------------------

inline int flcCheckGuards(const FlcObservation &o, const FlcContactConfig &guards,
                          uint32_t elapsedMs, uint32_t timeBudgetMs,
                          uint16_t expectTorqueLimit, bool requireTorqueOn,
                          int commandedTarget) {
  return flcCheckObservationGuards(o, guards, elapsedMs, timeBudgetMs,
                                   expectTorqueLimit,
                                   requireTorqueOn ? (int8_t)1 : (int8_t)-1,
                                   commandedTarget);
}

inline int flcEngineStatusForAbort(int abortReason) {
  if (abortReason == FLC_ABORT_TELEMETRY_TIMEOUT) return FLC_ENGINE_TELEMETRY_FAILED;
  if (abortReason == FLC_ABORT_PREREQUISITE_DRIFT)
    return FLC_ENGINE_PREREQUISITE_DRIFT;
  if (abortReason == FLC_ABORT_INVALID_PLAN) return FLC_ENGINE_INVALID_PLAN;
  if (abortReason == FLC_ABORT_DEPENDENCY_CYCLE) return FLC_ENGINE_DEPENDENCY_CYCLE;
  if (abortReason == FLC_ABORT_TORQUE_OFF_UNVERIFIED)
    return FLC_ENGINE_TORQUE_OFF_FAILED;
  if (abortReason == FLC_ABORT_CONTEXT_DRIFT)
    return FLC_ENGINE_CONTEXT_DRIFT;
  return FLC_ENGINE_ABORTED;
}

inline int flcReadFreshTelemetry(const FlcServoPort &port, uint8_t id,
                                 FlcObservation *out) {
  if (port.validateContext == 0 ||
      !port.validateContext(port.ctx, id)) return FLC_ABORT_CONTEXT_DRIFT;
  if (port.readTelemetry == 0 || !port.readTelemetry(port.ctx, id, out))
    return FLC_ABORT_TELEMETRY_TIMEOUT;
  return FLC_ABORT_NONE;
}

inline bool flcCommandPositionChecked(const FlcServoPort &port, uint8_t id,
                                      int position, uint16_t speed,
                                      uint8_t acceleration) {
  return port.validateContext != 0 &&
         port.validateContext(port.ctx, id) &&
         port.commandPosition(port.ctx, id, position, speed, acceleration);
}

#define FLC_MAX_WATCHED_JOINTS 12

struct FlcWatchJoint {
  uint8_t busId;
  int expectedTick;
  uint16_t toleranceTicks;
  int8_t expectedTorqueState;  // -1 unconstrained, 0 OFF, 1 ON
  uint16_t expectedTorqueLimit;
  int expectedGoalTick;        // -1 means do not compare GoalPosition
};

struct FlcMotionWatch {
  int count;
  FlcWatchJoint joints[FLC_MAX_WATCHED_JOINTS];
};

inline int flcCheckMotionWatch(const FlcServoPort &port,
                               const FlcMotionWatch *watch,
                               const FlcContactConfig &guards,
                               uint8_t *failedBusId) {
  if (failedBusId != 0) *failedBusId = 0;
  if (watch == 0 || watch->count == 0) return FLC_ABORT_NONE;
  if (watch->count < 0 || watch->count > FLC_MAX_WATCHED_JOINTS)
    return FLC_ABORT_INVALID_PLAN;

  for (int i = 0; i < watch->count; ++i) {
    const FlcWatchJoint &expected = watch->joints[i];
    FlcObservation o = {};
    const int readFault = flcReadFreshTelemetry(port, expected.busId, &o);
    if (readFault != FLC_ABORT_NONE) {
      if (failedBusId != 0) *failedBusId = expected.busId;
      return readFault;
    }
    const int fault = flcCheckObservationGuards(
        o, guards, 0, 0, expected.expectedTorqueLimit,
        expected.expectedTorqueState, expected.expectedGoalTick);
    if (fault != FLC_ABORT_NONE) {
      if (failedBusId != 0) *failedBusId = expected.busId;
      return fault;
    }
    if (expected.expectedTick < 0 || expected.expectedTick > FLC_ENCODER_MAX ||
        flcCircularDistance(o.position, expected.expectedTick) >
            (int)expected.toleranceTicks) {
      if (failedBusId != 0) *failedBusId = expected.busId;
      return FLC_ABORT_PREREQUISITE_DRIFT;
    }
  }
  return FLC_ABORT_NONE;
}

inline int flcCheckGuardsAndWatch(const FlcServoPort &port,
                                  const FlcObservation &o,
                                  const FlcContactConfig &guards,
                                  uint32_t elapsedMs,
                                  uint32_t timeBudgetMs,
                                  uint16_t expectedTorqueLimit,
                                  bool requireTorqueOn,
                                  int commandedTarget,
                                  const FlcMotionWatch *watch,
                                  uint8_t *failedBusId = 0) {
  const int ownFault = flcCheckGuards(o, guards, elapsedMs, timeBudgetMs,
                                      expectedTorqueLimit, requireTorqueOn,
                                      commandedTarget);
  if (ownFault != FLC_ABORT_NONE) return ownFault;
  return flcCheckMotionWatch(port, watch, guards, failedBusId);
}

// --------------------------------------------------------------------------
// Torque transaction. Every motion path enters through here and, critically,
// LEAVES through flcEndMotion() on every exit including every failure.
// --------------------------------------------------------------------------

inline bool flcEndMotion(const FlcServoPort &port, uint8_t id) {
  if (port.setTorqueEnable == 0 || port.readTelemetry == 0) return false;
  if (!port.setTorqueEnable(port.ctx, id, false)) return false;
  FlcObservation o = {};
  if (!port.readTelemetry(port.ctx, id, &o)) return false;
  // A transport acknowledgement is not proof of release. Fresh readable
  // telemetry must explicitly report torque OFF.
  return o.telemetryValid && !o.driverError && !o.torqueEnabled;
}

inline void flcRequireTorqueOff(const FlcServoPort &port, uint8_t id,
                                int &status, int &abortReason) {
  if (!flcEndMotion(port, id)) {
    status = FLC_ENGINE_TORQUE_OFF_FAILED;
    abortReason = FLC_ABORT_TORQUE_OFF_UNVERIFIED;
  }
}

struct FlcBeginMotionResult {
  int status;
  int abortReason;
  int actualStartTick;
  bool holdGoalVerified;
  bool torqueOnVerified;
};

inline FlcBeginMotionResult flcBeginMotion(
    const FlcServoPort &port, uint8_t id, int expectedStartTick,
    uint16_t startToleranceTicks, const FlcMotionEnvelope &envelope,
    const FlcContactConfig &guards, const FlcMotionWatch *watch = 0) {
  FlcBeginMotionResult r = {};
  r.status = FLC_ENGINE_OK;
  r.abortReason = FLC_ABORT_NONE;
  r.actualStartTick = -1;

  if (!flcPortUsable(port)) { r.status = FLC_ENGINE_REFUSED_PORT; return r; }
  if (!envelope.valid) { r.status = FLC_ENGINE_REFUSED_ENVELOPE; return r; }
  if (expectedStartTick < 0 || expectedStartTick > FLC_ENCODER_MAX) {
    r.status = FLC_ENGINE_REFUSED_DOMAIN;
    r.abortReason = FLC_ABORT_POSITION_OUT_OF_DOMAIN;
    return r;
  }

  // First observe with torque OFF. This catches a stale/foreign transaction and
  // establishes the real start position instead of trusting an old RAM value.
  FlcObservation before = {};
  int readFault = flcReadFreshTelemetry(port, id, &before);
  if (readFault != FLC_ABORT_NONE) {
    r.status = flcEngineStatusForAbort(readFault);
    r.abortReason = readFault;
    return r;
  }
  int fault = flcCheckObservationGuards(before, guards, 0, 0, 0, 0, -1);
  if (fault != FLC_ABORT_NONE) {
    r.status = flcEngineStatusForAbort(fault);
    r.abortReason = fault;
    flcEndMotion(port, id);
    return r;
  }
  r.actualStartTick = before.position;
  if (flcCircularDistance(r.actualStartTick, expectedStartTick) >
      (int)startToleranceTicks) {
    r.status = FLC_ENGINE_PREREQUISITE_DRIFT;
    r.abortReason = FLC_ABORT_PREREQUISITE_DRIFT;
    return r;
  }

  // TorqueLimit is a RAM safety bound. Set it while torque is still OFF, before
  // touching GoalPosition or TorqueEnable. A failed write cannot create motion.
  if (!port.validateContext(port.ctx, id)) {
    r.status = FLC_ENGINE_CONTEXT_DRIFT;
    r.abortReason = FLC_ABORT_CONTEXT_DRIFT;
    return r;
  }
  if (!port.setTorqueLimit(port.ctx, id, envelope.torqueLimit)) {
    r.status = FLC_ENGINE_TORQUE_SETUP_FAILED;
    return r;
  }

  // Critical ordering: preload GoalPosition to the PRESENT position while
  // torque is still OFF. Enabling torque against a stale retained goal can move
  // a joint before the first guarded segment has even started.
  if (!flcCommandPositionChecked(port, id, r.actualStartTick,
                                 envelope.goalSpeed,
                                 envelope.acceleration)) {
    r.status = FLC_ENGINE_ABORTED;
    r.abortReason = FLC_ABORT_DRIVER_ERROR;
    return r;
  }
  FlcObservation held = {};
  readFault = flcReadFreshTelemetry(port, id, &held);
  if (readFault != FLC_ABORT_NONE) {
    r.status = flcEngineStatusForAbort(readFault);
    r.abortReason = readFault;
    return r;
  }
  fault = flcCheckObservationGuards(held, guards, 0, 0,
                                    envelope.torqueLimit, 0,
                                    r.actualStartTick);
  if (fault != FLC_ABORT_NONE ||
      flcCircularDistance(held.position, r.actualStartTick) >
          (int)startToleranceTicks) {
    r.status = flcEngineStatusForAbort(
        fault == FLC_ABORT_NONE ? FLC_ABORT_PREREQUISITE_DRIFT : fault);
    r.abortReason =
        fault == FLC_ABORT_NONE ? FLC_ABORT_PREREQUISITE_DRIFT : fault;
    return r;
  }
  r.holdGoalVerified = true;

  fault = flcCheckMotionWatch(port, watch, guards, 0);
  if (fault != FLC_ABORT_NONE) {
    r.status = flcEngineStatusForAbort(fault);
    r.abortReason = fault;
    return r;
  }

  if (!port.validateContext(port.ctx, id)) {
    r.status = FLC_ENGINE_CONTEXT_DRIFT;
    r.abortReason = FLC_ABORT_CONTEXT_DRIFT;
    return r;
  }
  if (!port.setTorqueEnable(port.ctx, id, true)) {
    r.status = FLC_ENGINE_TORQUE_SETUP_FAILED;
    if (!flcEndMotion(port, id)) {
      r.status = FLC_ENGINE_TORQUE_OFF_FAILED;
      r.abortReason = FLC_ABORT_TORQUE_OFF_UNVERIFIED;
    }
    return r;
  }

  FlcObservation armed = {};
  readFault = flcReadFreshTelemetry(port, id, &armed);
  if (readFault != FLC_ABORT_NONE) {
    r.status = flcEngineStatusForAbort(readFault);
    r.abortReason = readFault;
  } else {
    fault = flcCheckObservationGuards(armed, guards, 0, 0, envelope.torqueLimit,
                                      1, r.actualStartTick);
    if (fault != FLC_ABORT_NONE ||
        flcCircularDistance(armed.position, r.actualStartTick) >
            (int)startToleranceTicks) {
      r.status = flcEngineStatusForAbort(
          fault == FLC_ABORT_NONE ? FLC_ABORT_PREREQUISITE_DRIFT : fault);
      r.abortReason =
          fault == FLC_ABORT_NONE ? FLC_ABORT_PREREQUISITE_DRIFT : fault;
    } else {
      r.torqueOnVerified = true;
      fault = flcCheckMotionWatch(port, watch, guards, 0);
      if (fault != FLC_ABORT_NONE) {
        r.status = flcEngineStatusForAbort(fault);
        r.abortReason = fault;
      }
    }
  }

  if (r.status != FLC_ENGINE_OK && !flcEndMotion(port, id)) {
    r.status = FLC_ENGINE_TORQUE_OFF_FAILED;
    r.abortReason = FLC_ABORT_TORQUE_OFF_UNVERIFIED;
  }
  return r;
}

// --------------------------------------------------------------------------
// Free-motion baseline
//
// Measures what "moving normally" looks like for THIS joint on THIS build, so
// the contact threshold is derived from the joint rather than inherited from
// another installation. This is why CONTACT_CURRENT_THRESHOLD_RAW does not
// exist as a global constant.
// --------------------------------------------------------------------------

struct FlcBaselineResult {
  int status;
  FlcBaseline baseline;
  int samples;
  uint16_t minCurrent;
  uint16_t maxCurrent;
  int startTick;
  int endTick;
  int travelTicks;
  uint16_t peakSpeed;
  int abortReason;
};

inline uint16_t flcMedianU16(uint16_t *values, int count) {
  for (int i = 1; i < count; ++i) {  // insertion sort; count is small
    uint16_t key = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > key) { values[j + 1] = values[j]; --j; }
    values[j + 1] = key;
  }
  return values[count / 2];
}

//: Drive a short, bounded free-motion excursion and characterize it.
inline FlcBaselineResult flcMeasureFreeMotionBaseline(
    const FlcServoPort &port, uint8_t id, int startTick, int8_t probeSign,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    uint16_t excursionTicks, const FlcMotionWatch *watch = 0) {
  FlcBaselineResult result;
  result.status = FLC_ENGINE_OK;
  result.baseline.medianCurrent = 0;
  result.baseline.madCurrent = 0;
  result.baseline.valid = false;
  result.samples = 0;
  result.minCurrent = 0xFFFF;
  result.maxCurrent = 0;
  result.startTick = startTick;
  result.endTick = startTick;
  result.travelTicks = 0;
  result.peakSpeed = 0;
  result.abortReason = FLC_ABORT_NONE;

  if (!flcPortUsable(port)) { result.status = FLC_ENGINE_REFUSED_PORT; return result; }
  if (!envelope.valid) { result.status = FLC_ENGINE_REFUSED_ENVELOPE; return result; }
  if (probeSign != 1 && probeSign != -1) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    return result;
  }
  if (!flcPlanStaysInDomain(startTick, excursionTicks, probeSign)) {
    result.status = FLC_ENGINE_REFUSED_DOMAIN;
    result.abortReason = FLC_ABORT_WRAP_BOUNDARY;
    return result;
  }

  const int target = startTick + probeSign * (int)excursionTicks;
  if (!flcCommandPositionChecked(port, id, target, envelope.goalSpeed,
                                 envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  uint16_t currents[64];
  int count = 0;
  int previousPosition = startTick;
  int stagnantSamples = 0;
  const uint32_t began = port.nowMs(port.ctx);

  while (count < 64) {
    const uint32_t now = port.nowMs(port.ctx);
    const uint32_t elapsed = now - began;
    if (elapsed > envelope.timeBudgetMs) {
      result.status = FLC_ENGINE_ABORTED;
      result.abortReason = FLC_ABORT_TIME_BUDGET_EXCEEDED;
      return result;
    }

    FlcObservation o;
    const int readFault = flcReadFreshTelemetry(port, id, &o);
    if (readFault != FLC_ABORT_NONE) {
      result.status = flcEngineStatusForAbort(readFault);
      result.abortReason = readFault;
      return result;
    }
    // Same guard set as every other motion segment — see flcCheckGuards.
    const int guardFault = flcCheckGuardsAndWatch(
        port, o, guards, elapsed, envelope.timeBudgetMs, envelope.torqueLimit,
        true, target, watch);
    if (guardFault != FLC_ABORT_NONE) {
      result.abortReason = guardFault;
      result.status = (guardFault == FLC_ABORT_TELEMETRY_TIMEOUT)
                          ? FLC_ENGINE_TELEMETRY_FAILED : FLC_ENGINE_ABORTED;
      return result;
    }

    const int travel = flcDirectionalProgress(o.position, startTick, probeSign);
    const int reverse = flcDirectionalProgress(o.position, startTick,
                                                (int8_t)-probeSign);
    if (reverse > (int)guards.targetReachedToleranceTicks) {
      result.status = FLC_ENGINE_ABORTED;
      result.abortReason = FLC_ABORT_WRONG_DIRECTION;
      return result;
    }
    if (travel > result.travelTicks) result.travelTicks = travel;
    result.endTick = o.position;

    if (flcCircularDistance(o.position, previousPosition) <=
            (int)guards.maxProgressTicks &&
        flcCircularDistance(o.position, target) >
            (int)guards.targetReachedToleranceTicks) {
      ++stagnantSamples;
    } else {
      stagnantSamples = 0;
    }
    previousPosition = o.position;
    if (stagnantSamples >=
        (int)guards.startupGraceSamples + (int)guards.persistenceSamples) {
      result.status = FLC_ENGINE_BASELINE_FAILED;
      result.abortReason = FLC_ABORT_TARGET_DID_NOT_MOVE;
      return result;
    }

    const int speedMag = o.velocity < 0 ? -(int)o.velocity : (int)o.velocity;
    if (speedMag > (int)result.peakSpeed) result.peakSpeed = (uint16_t)speedMag;

    // Only sample current while genuinely moving: a stopped servo's current is
    // not a free-motion baseline.
    if (speedMag > (int)guards.maxVelocityRaw) {
      currents[count++] = o.current;
      if (o.current < result.minCurrent) result.minCurrent = o.current;
      if (o.current > result.maxCurrent) result.maxCurrent = o.current;
    }

    if (flcCircularDistance(o.position, target) <= (int)guards.targetReachedToleranceTicks) break;
    if (port.idle != 0) port.idle(port.ctx, 5);
  }

  result.samples = count;
  if (count < 5) {
    // Too few moving samples to characterize anything.
    result.status = FLC_ENGINE_BASELINE_FAILED;
    if (result.minCurrent == 0xFFFF) result.minCurrent = 0;
    return result;
  }

  const uint16_t median = flcMedianU16(currents, count);
  uint16_t deviations[64];
  for (int i = 0; i < count; ++i) {
    deviations[i] = (uint16_t)(currents[i] > median ? currents[i] - median : median - currents[i]);
  }
  result.baseline.medianCurrent = median;
  result.baseline.madCurrent = flcMedianU16(deviations, count);
  result.baseline.valid = true;
  return result;
}

// --------------------------------------------------------------------------
// Bounded approach to a mechanical contact
//
// Advances the commanded target in bounded steps along probeSign, feeding every
// fresh observation to flcDetectorObserve(). Returns on the first
// CONTACT_CONFIRMED or HARD_ABORT, or when a budget is exhausted.
// --------------------------------------------------------------------------

struct FlcApproachResult {
  int status;
  int abortReason;
  int contactTick;
  int startTick;
  int travelTicks;
  uint32_t elapsedMs;
  uint16_t peakCurrent;
  uint8_t peakTemperature;
  bool currentSupportedContact;
  int observations;
};

inline FlcApproachResult flcRunApproach(
    const FlcServoPort &port, uint8_t id, int startTick, int8_t probeSign,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guardsIn,
    const FlcBaseline &baseline, const FlcMotionWatch *watch = 0) {
  FlcApproachResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.contactTick = -1;
  result.startTick = startTick;
  result.travelTicks = 0;
  result.elapsedMs = 0;
  result.peakCurrent = 0;
  result.peakTemperature = 0;
  result.currentSupportedContact = false;
  result.observations = 0;

  if (!flcPortUsable(port)) { result.status = FLC_ENGINE_REFUSED_PORT; return result; }
  if (!envelope.valid) { result.status = FLC_ENGINE_REFUSED_ENVELOPE; return result; }
  if (probeSign != 1 && probeSign != -1) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }

  FlcContactConfig guards = guardsIn;
  guards.probeSign = probeSign;
  guards.expectedTorqueLimit = envelope.torqueLimit;
  guards.travelBudgetTicks = envelope.travelBudgetTicks;
  guards.timeBudgetMs = envelope.timeBudgetMs;

  FlcContactDetector detector;
  flcDetectorInit(detector, guards, baseline, startTick);

  const uint32_t began = port.nowMs(port.ctx);
  int commandedTarget = startTick;
  bool haveTarget = false;

  while (true) {
    const uint32_t elapsed = port.nowMs(port.ctx) - began;
    result.elapsedMs = elapsed;
    if (elapsed > envelope.timeBudgetMs) {
      result.status = FLC_ENGINE_ABORTED;
      result.abortReason = FLC_ABORT_TIME_BUDGET_EXCEEDED;
      return result;
    }

    // Advance the target when we do not have one yet, or when the servo has
    // arrived at the current one. Contact is declared by the detector when the
    // servo stalls SHORT of a target, so arriving is never contact.
    if (!haveTarget) {
      const int remaining = (int)envelope.travelBudgetTicks - result.travelTicks;
      if (remaining <= 0) {
        result.status = FLC_ENGINE_ABORTED;
        result.abortReason = FLC_ABORT_TRAVEL_BUDGET_EXCEEDED;
        return result;
      }
      int step = (int)envelope.stepTicks;
      if (step > remaining) step = remaining;

      const int from = commandedTarget;
      if (!flcPlanStaysInDomain(from, step, probeSign)) {
        result.status = FLC_ENGINE_REFUSED_DOMAIN;
        result.abortReason = FLC_ABORT_WRAP_BOUNDARY;
        return result;
      }
      commandedTarget = from + probeSign * step;
      if (commandedTarget < 0 || commandedTarget > FLC_ENCODER_MAX) {
        result.status = FLC_ENGINE_REFUSED_DOMAIN;
        result.abortReason = FLC_ABORT_POSITION_OUT_OF_DOMAIN;
        return result;
      }
      if (!flcCommandPositionChecked(port, id, commandedTarget,
                                     envelope.goalSpeed,
                                     envelope.acceleration)) {
        result.status = FLC_ENGINE_ABORTED;
        result.abortReason = FLC_ABORT_DRIVER_ERROR;
        return result;
      }
      haveTarget = true;
    }

    FlcObservation o;
    const int readFault = flcReadFreshTelemetry(port, id, &o);
    if (readFault != FLC_ABORT_NONE) {
      result.status = flcEngineStatusForAbort(readFault);
      result.abortReason = readFault;
      return result;
    }
    o.elapsedMs = elapsed;
    ++result.observations;

    const int state = flcDetectorObserve(detector, o, commandedTarget);
    if (o.current > result.peakCurrent) result.peakCurrent = o.current;
    if (o.temperature > result.peakTemperature) result.peakTemperature = o.temperature;
    result.travelTicks = flcDirectionalProgress(o.position, startTick, probeSign);

    if (state == FLC_HARD_ABORT) {
      result.status = FLC_ENGINE_ABORTED;
      result.abortReason = detector.abortReason;
      return result;
    }
    const int watchFault = flcCheckMotionWatch(port, watch, guards, 0);
    if (watchFault != FLC_ABORT_NONE) {
      result.status = flcEngineStatusForAbort(watchFault);
      result.abortReason = watchFault;
      return result;
    }
    if (state == FLC_CONTACT_CONFIRMED) {
      result.contactTick = detector.contactTick;
      result.currentSupportedContact = detector.currentSupportedContact;
      result.status = FLC_ENGINE_OK;
      return result;
    }
    // Arrived where commanded: ask for the next step.
    if (flcCircularDistance(o.position, commandedTarget) <=
        (int)guards.targetReachedToleranceTicks) {
      haveTarget = false;
    }
    if (port.idle != 0) port.idle(port.ctx, 5);
  }
}

// --------------------------------------------------------------------------
// Retreat from contact, and verify the joint actually recovered
// --------------------------------------------------------------------------

struct FlcRetreatResult {
  int status;
  int abortReason;
  int fromTick;
  int toTick;
  int achievedTicks;
  bool trackingRecovered;
  bool currentRecovered;
  uint16_t restCurrent;
};

inline FlcRetreatResult flcRetreatAndVerify(
    const FlcServoPort &port, uint8_t id, int contactTick, int8_t probeSign,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    const FlcBaseline &baseline, const FlcMotionWatch *watch = 0) {
  FlcRetreatResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.fromTick = contactTick;
  result.toTick = contactTick;
  result.achievedTicks = 0;
  result.trackingRecovered = false;
  result.currentRecovered = false;
  result.restCurrent = 0;

  const int8_t retreatSign = (int8_t)-probeSign;
  if (!flcPlanStaysInDomain(contactTick, envelope.retreatTicks, retreatSign)) {
    result.status = FLC_ENGINE_REFUSED_DOMAIN;
    result.abortReason = FLC_ABORT_WRAP_BOUNDARY;
    return result;
  }

  const int target = contactTick + retreatSign * (int)envelope.retreatTicks;
  if (!flcCommandPositionChecked(port, id, target, envelope.goalSpeed,
                                 envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  const uint32_t began = port.nowMs(port.ctx);
  int previousPosition = contactTick;
  int stagnantSamples = 0;
  while (true) {
    const uint32_t elapsed = port.nowMs(port.ctx) - began;
    if (elapsed > envelope.timeBudgetMs) {
      result.status = FLC_ENGINE_RETREAT_FAILED;
      result.abortReason = FLC_ABORT_TIME_BUDGET_EXCEEDED;
      return result;
    }

    FlcObservation o;
    const int readFault = flcReadFreshTelemetry(port, id, &o);
    if (readFault != FLC_ABORT_NONE) {
      result.status = flcEngineStatusForAbort(readFault);
      result.abortReason = readFault;
      return result;
    }
    const int guardFault = flcCheckGuardsAndWatch(
        port, o, guards, elapsed, envelope.timeBudgetMs, envelope.torqueLimit,
        true, target, watch);
    if (guardFault != FLC_ABORT_NONE) {
      result.status = (guardFault == FLC_ABORT_TELEMETRY_TIMEOUT)
                          ? FLC_ENGINE_TELEMETRY_FAILED : FLC_ENGINE_ABORTED;
      result.abortReason = guardFault;
      return result;
    }

    result.toTick = o.position;
    result.achievedTicks = flcDirectionalProgress(o.position, contactTick, retreatSign);
    result.restCurrent = o.current;

    const int reverse = flcDirectionalProgress(o.position, contactTick, probeSign);
    if (reverse > (int)guards.targetReachedToleranceTicks) {
      result.status = FLC_ENGINE_RETREAT_FAILED;
      result.abortReason = FLC_ABORT_WRONG_DIRECTION;
      return result;
    }
    if (flcCircularDistance(o.position, previousPosition) <=
            (int)guards.maxProgressTicks &&
        flcCircularDistance(o.position, target) >
            (int)guards.targetReachedToleranceTicks) {
      ++stagnantSamples;
    } else {
      stagnantSamples = 0;
    }
    previousPosition = o.position;
    if (stagnantSamples >=
        (int)guards.startupGraceSamples + (int)guards.persistenceSamples) {
      result.status = FLC_ENGINE_RETREAT_FAILED;
      result.abortReason = FLC_ABORT_TARGET_DID_NOT_MOVE;
      return result;
    }

    if (flcCircularDistance(o.position, target) <= (int)guards.targetReachedToleranceTicks) {
      // Tracking recovered: the joint followed the retreat command to within
      // the normal settle tolerance, so it is no longer jammed against the stop.
      result.trackingRecovered = true;
      // Current recovered: back in the free-motion population rather than the
      // elevated contact population.
      result.currentRecovered =
          !baseline.valid || o.current < flcBaselineContactThreshold(baseline);
      break;
    }
    if (port.idle != 0) port.idle(port.ctx, 5);
  }

  // A retreat that moved a token amount has not actually cleared the endstop.
  const int required = (int)envelope.retreatTicks / 2;
  if (result.achievedTicks < required || !result.trackingRecovered ||
      !result.currentRecovered) {
    result.status = FLC_ENGINE_RETREAT_FAILED;
    result.abortReason = FLC_ABORT_TRACKING_FAILURE;
  }
  return result;
}

// --------------------------------------------------------------------------
// One endpoint = approach, retreat+verify, second INDEPENDENT approach,
// repeatability. A single contact is never an endpoint.
// --------------------------------------------------------------------------

struct FlcEndpointResult {
  int status;
  int abortReason;
  FlcApproachResult first;
  FlcRetreatResult retreat;
  FlcApproachResult second;
  //: Retreat performed AFTER the second approach. A joint is never left jammed
  //: against a mechanical stop, and the next operation needs a known start.
  FlcRetreatResult finalRetreat;
  FlcRepeatability repeatability;
  int contactTick;
  //: Where the joint actually came to rest. Use this, not the contact tick, as
  //: the start of any subsequent motion.
  int restTick;
  bool accepted;
};

inline FlcEndpointResult flcMeasureEndpoint(
    const FlcServoPort &port, uint8_t id, int startTick, int8_t probeSign,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    const FlcBaseline &baseline, uint16_t repeatabilityToleranceTicks,
    const FlcMotionWatch *watch = 0) {
  FlcEndpointResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.contactTick = -1;
  result.restTick = startTick;
  result.accepted = false;
  result.repeatability.firstTick = -1;
  result.repeatability.secondTick = -1;
  result.repeatability.spreadTicks = 0;
  result.repeatability.contactTick = -1;
  result.repeatability.accepted = false;

  result.first = flcRunApproach(port, id, startTick, probeSign, envelope, guards,
                                baseline, watch);
  if (result.first.status != FLC_ENGINE_OK) {
    result.status = result.first.status;
    result.abortReason = result.first.abortReason;
    return result;
  }

  result.retreat = flcRetreatAndVerify(port, id, result.first.contactTick, probeSign,
                                       envelope, guards, baseline, watch);
  if (result.retreat.status != FLC_ENGINE_OK) {
    result.status = result.retreat.status;
    result.abortReason = result.retreat.abortReason;
    return result;
  }

  // Second approach starts from where the retreat actually ended, so it is a
  // genuinely independent traversal rather than a replay of the first.
  result.second = flcRunApproach(port, id, result.retreat.toTick, probeSign, envelope,
                                 guards, baseline, watch);
  if (result.second.status != FLC_ENGINE_OK) {
    result.status = result.second.status;
    result.abortReason = result.second.abortReason;
    return result;
  }

  // Release the stop before evaluating anything. Leaving the joint loaded
  // against a mechanical endstop while the caller decides what to do next would
  // hold torque against the structure for an unbounded time, and would give the
  // next endpoint a start position that does not match where the joint is.
  result.finalRetreat = flcRetreatAndVerify(port, id, result.second.contactTick,
                                            probeSign, envelope, guards, baseline,
                                            watch);
  result.restTick = result.finalRetreat.toTick;
  if (result.finalRetreat.status != FLC_ENGINE_OK) {
    result.status = result.finalRetreat.status;
    result.abortReason = result.finalRetreat.abortReason;
    return result;
  }

  result.repeatability = flcEvaluateRepeatability(
      result.first.contactTick, result.second.contactTick, repeatabilityToleranceTicks);
  result.contactTick = result.repeatability.contactTick;

  if (!result.repeatability.accepted) {
    result.status = FLC_ENGINE_REPEATABILITY_FAILED;
    return result;
  }
  result.accepted = true;
  return result;
}

// --------------------------------------------------------------------------
// Return to a known tick between endpoints
//
// Each endpoint is approached FROM the neutral start, never straight from the
// opposite endpoint. That keeps every traversal to roughly one joint half-range
// instead of the full span, which both bounds the swept volume and keeps the
// travel budget comfortably below the half-revolution ambiguity limit.
// --------------------------------------------------------------------------

struct FlcReturnResult {
  int status;
  int abortReason;
  int targetTick;
  int achievedTick;
  int errorTicks;
};

inline FlcReturnResult flcReturnTo(const FlcServoPort &port, uint8_t id, int targetTick,
                                   const FlcMotionEnvelope &envelope,
                                   const FlcContactConfig &guards,
                                   const FlcMotionWatch *watch = 0) {
  FlcReturnResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.targetTick = targetTick;
  result.achievedTick = targetTick;
  result.errorTicks = 0;

  if (targetTick < 0 || targetTick > FLC_ENCODER_MAX) {
    result.status = FLC_ENGINE_REFUSED_DOMAIN;
    result.abortReason = FLC_ABORT_POSITION_OUT_OF_DOMAIN;
    return result;
  }
  if (!flcCommandPositionChecked(port, id, targetTick, envelope.goalSpeed,
                                 envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  const uint32_t began = port.nowMs(port.ctx);
  while (true) {
    if (port.nowMs(port.ctx) - began > envelope.timeBudgetMs) {
      result.status = FLC_ENGINE_ABORTED;
      result.abortReason = FLC_ABORT_TIME_BUDGET_EXCEEDED;
      return result;
    }
    FlcObservation o;
    const int readFault = flcReadFreshTelemetry(port, id, &o);
    if (readFault != FLC_ABORT_NONE) {
      result.status = flcEngineStatusForAbort(readFault);
      result.abortReason = readFault;
      return result;
    }
    const int guardFault = flcCheckGuardsAndWatch(
        port, o, guards, port.nowMs(port.ctx) - began, envelope.timeBudgetMs,
        envelope.torqueLimit, true, targetTick, watch);
    if (guardFault != FLC_ABORT_NONE) {
      result.status = (guardFault == FLC_ABORT_TELEMETRY_TIMEOUT)
                          ? FLC_ENGINE_TELEMETRY_FAILED : FLC_ENGINE_ABORTED;
      result.abortReason = guardFault;
      return result;
    }

    result.achievedTick = o.position;
    result.errorTicks = flcCircularDistance(o.position, targetTick);
    if (result.errorTicks <= (int)guards.targetReachedToleranceTicks) break;
    if (port.idle != 0) port.idle(port.ctx, 5);
  }
  return result;
}

//: Travel budget for one approach: the expected distance plus 30% margin and a
//: fixed 64-tick allowance, clamped to the absolute ceiling. Derived from
//: geometry rather than being a magic constant.
inline uint16_t flcTravelBudgetForDistance(int expectedTicks) {
  int magnitude = expectedTicks < 0 ? -expectedTicks : expectedTicks;
  long budget = (long)magnitude + (long)magnitude * 3 / 10 + 64;
  if (budget > FLC_ABSOLUTE_MAX_TRAVEL_BUDGET_TICKS)
    budget = FLC_ABSOLUTE_MAX_TRAVEL_BUDGET_TICKS;
  if (budget < 64) budget = 64;
  return (uint16_t)budget;
}

// --------------------------------------------------------------------------
// ENCODER RESPONSE vs KINEMATIC DIRECTION — two different things
//
// A position-controlled ST3215 will, by construction, move its raw encoder
// toward whatever raw target it is given. So commanding `startTick + N` and
// observing that raw went up proves only that the servo is alive and in
// position mode. It says NOTHING about the MATDOG joint convention
//
//     q = direction * signed_tick_delta(raw, q0)
//
// because `direction` is a property of how the horn and linkage were physically
// mounted, not of the servo's control loop. Treating the probe result as the
// kinematic direction would be tautological — it would always return +1.
//
// This file therefore separates:
//
//   flcVerifyEncoderResponse()  sanity: does raw track the commanded target?
//   flcResolveJointDirection()  the real question, resolved from evidence
//
// The kinematic direction is resolved from the ASYMMETRY of the URDF limits.
// Both endpoints are measured direction-agnostically, giving raw_lo and raw_hi.
// Two hypotheses remain:
//
//   direction = +1  ->  raw_lo is the q_min stop, q0 = raw_lo + |q_min|
//   direction = -1  ->  raw_lo is the q_max stop, q0 = raw_lo + |q_max|
//
// The two hypotheses place q0 ||q_min| - |q_max|| ticks apart. For the MATDOG
// upper and lower joints that separation is large (about 793 and 613 ticks) and
// the manual q0 witness discriminates decisively. For the hips it is only about
// 9 ticks — far inside the manual-pose uncertainty — so geometry ALONE CANNOT
// resolve a hip, and this code says so instead of guessing.
// --------------------------------------------------------------------------

enum FlcDirectionMethod {
  FLC_DIRECTION_UNRESOLVED = 0,
  //: resolved from URDF limit asymmetry cross-checked against the manual q0
  FLC_DIRECTION_GEOMETRY_AND_MANUAL_Q0,
  //: declared by the operator after physically observing the joint move
  FLC_DIRECTION_OPERATOR_WITNESS,
  //: operator declared it AND geometry independently agreed
  FLC_DIRECTION_OPERATOR_AND_GEOMETRY_AGREE,
};

inline const char *flcDirectionMethodLabel(int method) {
  switch (method) {
    case FLC_DIRECTION_GEOMETRY_AND_MANUAL_Q0: return "GEOMETRY_AND_MANUAL_Q0";
    case FLC_DIRECTION_OPERATOR_WITNESS: return "OPERATOR_WITNESS";
    case FLC_DIRECTION_OPERATOR_AND_GEOMETRY_AGREE: return "OPERATOR_AND_GEOMETRY_AGREE";
    default: return "UNRESOLVED";
  }
}

//: Sanity check only. Confirms the servo follows a commanded raw target; it is
//: NOT the kinematic direction. Named so it cannot be mistaken for one.
struct FlcEncoderResponseResult {
  int status;
  int abortReason;
  bool responds;
  int observedTravel;
  int startTick;
  int endTick;
  int8_t rawProbeSign;
};

inline FlcEncoderResponseResult flcVerifyEncoderResponse(
    const FlcServoPort &port, uint8_t id, int startTick,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    uint16_t probeTicks, int8_t requestedRawProbeSign = 0,
    const FlcMotionWatch *watch = 0) {
  FlcEncoderResponseResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.responds = false;
  result.observedTravel = 0;
  result.startTick = startTick;
  result.endTick = startTick;
  result.rawProbeSign = 0;

  if (!flcPortUsable(port)) { result.status = FLC_ENGINE_REFUSED_PORT; return result; }
  if (!envelope.valid) { result.status = FLC_ENGINE_REFUSED_ENVELOPE; return result; }

  // Probe toward whichever side has room, so a joint already near a boundary is
  // not pushed out of the unsigned domain. An EXPLICIT semantic choice is never
  // silently reversed: that could turn a generated no-parking endpoint into a
  // parking-required one.
  int8_t probeSign = requestedRawProbeSign;
  const bool explicitSign = probeSign == 1 || probeSign == -1;
  if (!explicitSign) probeSign = +1;
  if (!flcPlanStaysInDomain(startTick, probeTicks, probeSign) && !explicitSign)
    probeSign = (int8_t)-probeSign;
  if (!flcPlanStaysInDomain(startTick, probeTicks, probeSign)) {
    result.status = FLC_ENGINE_REFUSED_DOMAIN;
    result.abortReason = FLC_ABORT_WRAP_BOUNDARY;
    return result;
  }
  result.rawProbeSign = probeSign;

  const int target = startTick + probeSign * (int)probeTicks;
  if (!flcCommandPositionChecked(port, id, target, envelope.goalSpeed,
                                 envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  const uint32_t began = port.nowMs(port.ctx);
  while (true) {
    const uint32_t elapsed = port.nowMs(port.ctx) - began;
    if (elapsed > envelope.timeBudgetMs) break;

    FlcObservation o;
    const int readFault = flcReadFreshTelemetry(port, id, &o);
    if (readFault != FLC_ABORT_NONE) {
      result.status = flcEngineStatusForAbort(readFault);
      result.abortReason = readFault;
      return result;
    }
    const int guardFault = flcCheckGuardsAndWatch(
        port, o, guards, elapsed, envelope.timeBudgetMs, envelope.torqueLimit,
        true, target, watch);
    if (guardFault != FLC_ABORT_NONE) {
      result.status = (guardFault == FLC_ABORT_TELEMETRY_TIMEOUT)
                          ? FLC_ENGINE_TELEMETRY_FAILED : FLC_ENGINE_ABORTED;
      result.abortReason = guardFault;
      return result;
    }

    result.endTick = o.position;
    if (flcCircularDistance(o.position, target) <= (int)guards.targetReachedToleranceTicks) break;
    if (port.idle != 0) port.idle(port.ctx, 5);
  }

  const int delta = flcSignedTickDelta(result.endTick, startTick);
  result.observedTravel = delta < 0 ? -delta : delta;

  // The encoder must follow the commanded side. If it moved the OTHER way the
  // servo is miswired or not in position mode — a hard fault, not a direction.
  if (delta * probeSign < 0 && result.observedTravel > (int)guards.targetReachedToleranceTicks) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }
  if (result.observedTravel < (int)probeTicks / 2) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_TARGET_DID_NOT_MOVE;
    return result;
  }
  result.responds = true;
  return result;
}

// --------------------------------------------------------------------------
// Kinematic direction resolution
// --------------------------------------------------------------------------

struct FlcDirectionEvidence {
  //: measured contacts, direction-agnostic: rawLo < rawHi along the encoder
  int rawLoTick;
  int rawHiTick;
  //: URDF/Geometry Compiler angles in ticks; qMin is negative, qMax positive
  int qMinTicks;
  int qMaxTicks;
  //: manual-pose q0 candidate, if H2 produced a usable one for this joint
  bool manualQ0Known;
  int manualQ0Tick;
  //: Physical pose accuracy, established independently of encoder stability.
  //: H2 sample spread MUST NOT populate this field.
  bool manualQ0PoseUncertaintyKnown;
  uint16_t manualQ0PoseUncertaintyTicks;
  //: operator-declared sign after physically watching the joint move
  int8_t operatorWitness;   // 0 = none
};

struct FlcDirectionResolution {
  int status;
  int method;
  int8_t direction;
  int q0IfPositive;
  int q0IfNegative;
  int q0PositiveFromMin;
  int q0PositiveFromMax;
  int q0NegativeFromMin;
  int q0NegativeFromMax;
  int endpointQ0DisagreementTicks;
  int separationTicks;      //: how far apart the two hypotheses place q0
  int errorIfPositive;
  int errorIfNegative;
  int marginTicks;          //: how decisively the manual q0 favours the winner
  bool geometryDecisive;
  const char *diagnosis;
};

inline int flcCircularMidpoint(int a, int b) {
  return flcNormalizeTick(a + flcSignedTickDelta(b, a) / 2);
}

inline FlcDirectionResolution flcResolveJointDirection(const FlcDirectionEvidence &e) {
  FlcDirectionResolution r;
  r.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
  r.method = FLC_DIRECTION_UNRESOLVED;
  r.direction = 0;
  r.errorIfPositive = 0;
  r.errorIfNegative = 0;
  r.marginTicks = 0;
  r.geometryDecisive = false;
  r.diagnosis = "";
  r.q0IfPositive = -1;
  r.q0IfNegative = -1;
  r.q0PositiveFromMin = -1;
  r.q0PositiveFromMax = -1;
  r.q0NegativeFromMin = -1;
  r.q0NegativeFromMax = -1;
  r.endpointQ0DisagreementTicks = 0;

  if (e.rawLoTick < 0 || e.rawLoTick > FLC_ENCODER_MAX ||
      e.rawHiTick < 0 || e.rawHiTick > FLC_ENCODER_MAX ||
      e.rawHiTick <= e.rawLoTick || e.qMinTicks >= 0 || e.qMaxTicks <= 0 ||
      (e.operatorWitness != 0 && e.operatorWitness != 1 &&
       e.operatorWitness != -1)) {
    r.diagnosis = "invalid endpoint, URDF-limit, or operator-witness evidence";
    return r;
  }

  // Each hypothesis derives q0 independently from BOTH physical endpoints.
  // Their disagreement is also the endpoint-span/URDF inconsistency residual.
  r.q0PositiveFromMin = flcNormalizeTick(e.rawLoTick - e.qMinTicks);
  r.q0PositiveFromMax = flcNormalizeTick(e.rawHiTick - e.qMaxTicks);
  r.q0IfPositive = flcCircularMidpoint(r.q0PositiveFromMin,
                                       r.q0PositiveFromMax);
  r.q0NegativeFromMin = flcNormalizeTick(e.rawHiTick + e.qMinTicks);
  r.q0NegativeFromMax = flcNormalizeTick(e.rawLoTick + e.qMaxTicks);
  r.q0IfNegative = flcCircularMidpoint(r.q0NegativeFromMin,
                                       r.q0NegativeFromMax);
  const int positiveResidual = flcCircularDistance(r.q0PositiveFromMin,
                                                    r.q0PositiveFromMax);
  const int negativeResidual = flcCircularDistance(r.q0NegativeFromMin,
                                                    r.q0NegativeFromMax);
  r.endpointQ0DisagreementTicks =
      positiveResidual > negativeResidual ? positiveResidual : negativeResidual;
  r.separationTicks = flcCircularDistance(r.q0IfPositive, r.q0IfNegative);

  // Geometry can only discriminate if the two hypotheses put q0 meaningfully
  // further apart than the manual pose could plausibly be wrong by.
  const int required = 2 * (int)e.manualQ0PoseUncertaintyTicks;
  int8_t geometryDirection = 0;

  if (e.manualQ0Known && e.manualQ0PoseUncertaintyKnown &&
      r.separationTicks > required) {
    const int errPos = flcCircularDistance(r.q0IfPositive, e.manualQ0Tick);
    const int errNeg = flcCircularDistance(r.q0IfNegative, e.manualQ0Tick);
    r.errorIfPositive = errPos;
    r.errorIfNegative = errNeg;
    r.marginTicks = errPos > errNeg ? errPos - errNeg : errNeg - errPos;

    // The winner must also be plausible in absolute terms, not merely better.
    const int winnerError = errPos < errNeg ? errPos : errNeg;
    if (r.marginTicks > required &&
        winnerError <= (int)e.manualQ0PoseUncertaintyTicks) {
      geometryDirection = (errPos < errNeg) ? (int8_t)1 : (int8_t)-1;
      r.geometryDecisive = true;
    }
  }

  const bool haveOperator = (e.operatorWitness == 1 || e.operatorWitness == -1);

  if (haveOperator && r.geometryDecisive) {
    if (e.operatorWitness != geometryDirection) {
      r.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
      r.diagnosis =
          "operator witness and geometry disagree; refusing rather than picking one";
      return r;
    }
    r.direction = geometryDirection;
    r.method = FLC_DIRECTION_OPERATOR_AND_GEOMETRY_AGREE;
    r.status = FLC_ENGINE_OK;
    r.diagnosis = "operator witness confirmed by URDF limit asymmetry";
    return r;
  }
  if (haveOperator) {
    r.direction = e.operatorWitness;
    r.method = FLC_DIRECTION_OPERATOR_WITNESS;
    r.status = FLC_ENGINE_OK;
    r.diagnosis =
        "operator witness only; joint limits too symmetric for geometry to confirm";
    return r;
  }
  if (r.geometryDecisive) {
    r.direction = geometryDirection;
    r.method = FLC_DIRECTION_GEOMETRY_AND_MANUAL_Q0;
    r.status = FLC_ENGINE_OK;
    r.diagnosis = "resolved from URDF limit asymmetry against the manual q0 candidate";
    return r;
  }

  r.diagnosis =
      "no operator witness, and the joint limits are too symmetric for the manual "
      "q0 candidate to discriminate; direction cannot be inferred";
  return r;
}

// --------------------------------------------------------------------------
// H3 — characterize one joint
//
// Produces the evidence H4 needs. It may run on a bootstrap envelope, and it is
// deliberately NOT blocked by the acceptance tolerances it exists to inform.
//
// H3 does NOT resolve the kinematic direction: that needs BOTH endpoints, which
// only H4 measures. H3 verifies the encoder responds, measures the free-motion
// baseline, and proves the joint can be driven into a real stop and released
// twice — which is what yields a measured repeatability band.
// --------------------------------------------------------------------------

struct FlcCharacterizationResult {
  int status;
  int abortReason;
  uint8_t busId;
  int startTick;
  FlcEncoderResponseResult encoderResponse;
  FlcBaselineResult baseline;
  FlcApproachResult probeContact;
  FlcRetreatResult probeRetreat;
  FlcApproachResult probeContact2;
  FlcRetreatResult finalRetreat;
  FlcReturnResult returnToStart;
  //: characterized outputs, origin CHARACTERIZED_CURRENT_HARDWARE on success
  uint16_t characterizedContactThresholdRaw;
  uint16_t characterizedRetreatTicks;
  uint16_t characterizedRepeatabilityToleranceTicks;
  uint16_t observedContactSpreadTicks;
  uint16_t peakCurrent;
  uint8_t peakTemperature;
  int restTick;
  bool complete;
};

//: Repeatability acceptance band from a measured spread. Twice the observed
//: spread with an 8-tick floor: generous enough that a joint reproducing its
//: characterization behaviour passes, tight enough to catch a joint that does
//: not. Derived from this build's evidence, not inherited from LF V25.
inline uint16_t flcRepeatabilityBandFromSpread(uint16_t observedSpreadTicks) {
  uint32_t band = (uint32_t)observedSpreadTicks * 2u;
  if (band < 8u) band = 8u;
  if (band > 64u) band = 64u;
  return (uint16_t)band;
}

inline FlcCharacterizationResult flcCharacterizeJoint(
    const FlcServoPort &port, uint8_t id, int startTick,
    int8_t requestedRawProbeSign,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    uint16_t responseProbeTicks, uint16_t baselineExcursionTicks,
    const FlcMotionWatch *watch = 0) {
  FlcCharacterizationResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.busId = id;
  result.startTick = startTick;
  result.characterizedContactThresholdRaw = 0;
  result.characterizedRetreatTicks = 0;
  result.characterizedRepeatabilityToleranceTicks = 0;
  result.observedContactSpreadTicks = 0;
  result.peakCurrent = 0;
  result.peakTemperature = 0;
  result.restTick = startTick;
  result.complete = false;

  if (requestedRawProbeSign != 0 && requestedRawProbeSign != 1 &&
      requestedRawProbeSign != -1) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }

  const FlcBeginMotionResult begun = flcBeginMotion(
      port, id, startTick, guards.targetReachedToleranceTicks, envelope, guards,
      watch);
  if (begun.status != FLC_ENGINE_OK) {
    result.status = begun.status;
    result.abortReason = begun.abortReason;
    return result;
  }

  // H3A — encoder response sanity. This deliberately reports RAW probe sign,
  // never MATDOG kinematic direction.
  result.encoderResponse = flcVerifyEncoderResponse(port, id, startTick, envelope,
                                                    guards, responseProbeTicks,
                                                    requestedRawProbeSign, watch);
  if (result.encoderResponse.status != FLC_ENGINE_OK) {
    result.status = result.encoderResponse.status;
    result.abortReason = result.encoderResponse.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }

  const int8_t rawProbeSign = result.encoderResponse.rawProbeSign;

  // H3B — free-motion baseline, from which the contact threshold is derived.
  result.baseline = flcMeasureFreeMotionBaseline(port, id, result.encoderResponse.endTick,
                                                 rawProbeSign, envelope, guards,
                                                 baselineExcursionTicks, watch);
  if (result.baseline.status != FLC_ENGINE_OK) {
    result.status = result.baseline.status;
    result.abortReason = result.baseline.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }

  // H3C — one supervised bounded approach to a real contact, plus retreat.
  result.probeContact = flcRunApproach(
      port, id, result.baseline.endTick, rawProbeSign, envelope, guards,
      result.baseline.baseline, watch);
  if (result.probeContact.status != FLC_ENGINE_OK) {
    result.status = result.probeContact.status;
    result.abortReason = result.probeContact.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }
  result.peakCurrent = result.probeContact.peakCurrent;
  result.peakTemperature = result.probeContact.peakTemperature;

  result.probeRetreat = flcRetreatAndVerify(port, id, result.probeContact.contactTick,
                                            rawProbeSign, envelope, guards,
                                            result.baseline.baseline, watch);
  if (result.probeRetreat.status != FLC_ENGINE_OK) {
    result.status = result.probeRetreat.status;
    result.abortReason = result.probeRetreat.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }

  // H3D — a SECOND independent approach, so the repeatability band is MEASURED
  // on this build rather than inherited from somewhere else.
  result.probeContact2 = flcRunApproach(
      port, id, result.probeRetreat.toTick, rawProbeSign, envelope, guards,
      result.baseline.baseline, watch);
  if (result.probeContact2.status != FLC_ENGINE_OK) {
    result.status = result.probeContact2.status;
    result.abortReason = result.probeContact2.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }
  if (result.probeContact2.peakCurrent > result.peakCurrent)
    result.peakCurrent = result.probeContact2.peakCurrent;

  result.observedContactSpreadTicks = (uint16_t)flcCircularDistance(
      result.probeContact.contactTick, result.probeContact2.contactTick);

  // Release the stop so the joint does not end characterization loaded.
  result.finalRetreat = flcRetreatAndVerify(port, id, result.probeContact2.contactTick,
                                            rawProbeSign, envelope, guards,
                                            result.baseline.baseline, watch);
  if (result.finalRetreat.status != FLC_ENGINE_OK) {
    result.status = result.finalRetreat.status;
    result.abortReason = result.finalRetreat.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }
  result.restTick = result.finalRetreat.toTick;

  // H3 ends at its known start pose, not merely clear of the contact. This is
  // required before a different joint may move under the generated q=0 plan.
  result.returnToStart = flcReturnTo(port, id, startTick, envelope, guards, watch);
  if (result.returnToStart.status != FLC_ENGINE_OK) {
    result.status = result.returnToStart.status;
    result.abortReason = result.returnToStart.abortReason;
    flcRequireTorqueOff(port, id, result.status, result.abortReason);
    return result;
  }
  result.restTick = result.returnToStart.achievedTick;

  result.characterizedContactThresholdRaw =
      flcBaselineContactThreshold(result.baseline.baseline);
  result.characterizedRetreatTicks = (uint16_t)result.probeRetreat.achievedTicks;
  result.characterizedRepeatabilityToleranceTicks =
      flcRepeatabilityBandFromSpread(result.observedContactSpreadTicks);
  result.complete = true;

  flcRequireTorqueOff(port, id, result.status, result.abortReason);
  if (result.status != FLC_ENGINE_OK) result.complete = false;
  return result;
}

// --------------------------------------------------------------------------
// H4 / H5 / H6 — dependency-aware calibration over the generated geometry plan
//
// One implementation, one entry point. H4, H5 and H6 are the same
// flcRunCalibrationPlan() call under different selection masks, so the
// orchestration exercised offline is the orchestration the firmware executes.
// --------------------------------------------------------------------------

#define FLC_JOINTS_PER_LEG 3
#define FLC_LEG_COUNT 4
#define FLC_CALIBRATION_JOINT_COUNT FLC_GEOMETRY_JOINT_COUNT
#define FLC_ALL_GEOMETRY_JOINTS_MASK ((uint16_t)0x0FFFu)

// A result is never promoted by this firmware. ACCEPTED means only that every
// explicitly characterized acceptance gate passed in the current session.
enum FlcResultTier {
  FLC_TIER_MEASURED = 0,
  FLC_TIER_CANDIDATE,
  FLC_TIER_ACCEPTED,
  FLC_TIER_PROMOTED,
};

inline const char *flcResultTierLabel(int tier) {
  switch (tier) {
    case FLC_TIER_MEASURED: return "MEASURED";
    case FLC_TIER_CANDIDATE: return "CANDIDATE";
    case FLC_TIER_ACCEPTED: return "ACCEPTED";
    case FLC_TIER_PROMOTED: return "PROMOTED";
    default: return "UNKNOWN";
  }
}

enum FlcQ0CrosscheckStatus {
  FLC_Q0_CROSSCHECK_NOT_RUN = 0,
  FLC_Q0_CROSSCHECK_MATCH,
  FLC_Q0_CROSSCHECK_MISMATCH,
  FLC_Q0_BLOCKED_TOLERANCE_UNVALIDATED,
  FLC_Q0_BLOCKED_MANUAL_CANDIDATE_MISSING,
};

inline const char *flcQ0CrosscheckStatusLabel(int status) {
  switch (status) {
    case FLC_Q0_CROSSCHECK_MATCH: return "MATCH";
    case FLC_Q0_CROSSCHECK_MISMATCH: return "MISMATCH";
    case FLC_Q0_BLOCKED_TOLERANCE_UNVALIDATED:
      return "BLOCKED_TOLERANCE_UNVALIDATED";
    case FLC_Q0_BLOCKED_MANUAL_CANDIDATE_MISSING:
      return "BLOCKED_MANUAL_Q0_MISSING";
    default: return "NOT_RUN";
  }
}

struct FlcAcceptanceGates {
  bool repeatabilityToleranceKnown;
  uint16_t repeatabilityToleranceTicks;
  bool endpointVsUrdfToleranceKnown;
  uint16_t endpointVsUrdfToleranceTicks;
  int expectedSpanTicks;
  bool manualVsDerivedQ0ToleranceKnown;
  uint16_t manualVsDerivedQ0ToleranceTicks;
};

// One row per generated geometry joint, indexed by FlcGeometryJoint. H2 and H3
// evidence stay distinct; a manual q0 candidate is never silently promoted to
// a derived calibration and the two values are never averaged.
struct FlcJointPlan {
  FlcGeometryJoint geometryJoint;
  uint8_t busId;
  bool characterized;
  int startTick;
  bool q0WatchKnown;
  int q0WatchTick;
  uint16_t q0WatchToleranceTicks;
  bool calibrationKnown;
  int8_t knownDirection;
  int knownQ0Tick;
  FlcDirectionEvidence directionEvidence;
  FlcBaseline baseline;
  FlcMotionEnvelope envelope;
  FlcAcceptanceGates gates;
};

struct FlcRuntimeCalibration {
  bool holdKnown;
  int holdTick;
  uint16_t holdToleranceTicks;
  bool calibrationUsable;
  int8_t direction;
  int q0Tick;
};

struct FlcParkingTransaction {
  int status;
  int abortReason;
  bool required;
  bool noParkingProvenanceVerified;
  bool prerequisiteVerified;
  bool entered;
  bool trackingVerified;
  bool restoreAttempted;
  bool restoreVerified;
  uint8_t auxiliaryBusId;
  FlcGeometryJoint auxiliaryJoint;
  int savedTick;
  int parkingTick;
  int achievedTick;
  int restoredTick;
};

struct FlcEndpointExecution {
  int status;
  int abortReason;
  uint8_t geometryEndpointIndex;
  FlcParkingOutcome parkingOutcome;
  bool q0WatchVerified;
  bool targetHoldGoalVerified;
  bool targetReturnedToQ0;
  bool targetTorqueOffVerified;
  FlcParkingTransaction parking;
  FlcEndpointResult measurement;
  FlcReturnResult returnToQ0;
};

struct FlcJointCalibrationResult {
  int status;
  int abortReason;
  uint8_t busId;
  FlcGeometryJoint geometryJoint;
  int8_t direction;
  int directionMethod;
  FlcDirectionResolution directionResolution;
  FlcEndpointResult minEndpoint;
  FlcEndpointResult maxEndpoint;
  FlcEndpointExecution minExecution;
  FlcEndpointExecution maxExecution;
  int rawLoTick;
  int rawHiTick;
  int minContactTick;
  int maxContactTick;
  int measuredSpanTicks;
  int expectedSpanTicks;
  int spanErrorTicks;
  int manualPoseQ0CandidateTick;
  int derivedQ0FinalTick;
  int q0Tick;  // compatibility alias for derivedQ0FinalTick
  int manualVsDerivedQ0ErrorTicks;
  int q0CrosscheckStatus;
  int tier;
  bool accepted;
};

struct FlcCalibrationRunResult {
  int status;
  int abortReason;
  uint16_t selectionMask;
  int jointsRequested;
  int jointsOk;
  int executionCount;
  FlcGeometryJoint executionOrder[FLC_CALIBRATION_JOINT_COUNT];
  bool resultPresent[FLC_CALIBRATION_JOINT_COUNT];
  FlcJointCalibrationResult joints[FLC_CALIBRATION_JOINT_COUNT];
  FlcRuntimeCalibration finalCalibration[FLC_CALIBRATION_JOINT_COUNT];
  FlcGeometryJoint failedJoint;
  uint8_t failedBusId;
  int parkingRequiredCount;
  int noParkingCount;
  int parkingRestoreCount;
  bool safeOffVerified;
  bool hardFaultCutPowerNow;
};

inline int flcAbsInt(int value) { return value < 0 ? -value : value; }

inline bool flcGeometryPlanSummary(int &noParking, int &parking) {
  noParking = 0;
  parking = 0;
  if (FLC_ENDPOINT_GEOMETRY_PLAN_COUNT != 24) return false;
  bool seen[24] = {false};
  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {
    const FlcEndpointGeometryPlan &row = FLC_ENDPOINT_GEOMETRY_PLANS[i];
    if (row.canonicalEndpointIndex >= 24 ||
        seen[row.canonicalEndpointIndex] ||
        (int)row.targetJoint != i / 2 ||
        row.motionAuthorizationProvenance !=
            FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY ||
        row.q0StartMask != FLC_ALL_GEOMETRY_JOINTS_MASK) return false;
    seen[row.canonicalEndpointIndex] = true;
    if (row.parkingOutcome == FLC_NO_PARKING_REQUIRED) {
      if (row.auxiliaryJoint != FLC_GEOMETRY_JOINT_NONE) return false;
      ++noParking;
    } else if (row.parkingOutcome == FLC_PARKING_REQUIRED_1DOF) {
      if ((int)row.auxiliaryJoint >= FLC_CALIBRATION_JOINT_COUNT ||
          row.auxiliaryJoint == row.targetJoint) return false;
      ++parking;
    } else {
      return false;
    }
  }
  return noParking == 18 && parking == 6;
}

// Generic Kahn implementation used by production and by the cycle-injection
// test. This prevents a checked-in order from masking a newly introduced cycle.
inline bool flcBuildDependencyOrder(const FlcGeometryDependency *dependencies,
                                    int dependencyCount,
                                    FlcGeometryJoint *order,
                                    int capacity) {
  if (dependencies == 0 || dependencyCount < 0 || order == 0 ||
      capacity < FLC_CALIBRATION_JOINT_COUNT) return false;
  uint8_t indegree[FLC_CALIBRATION_JOINT_COUNT] = {0};
  bool emitted[FLC_CALIBRATION_JOINT_COUNT] = {false};
  for (int i = 0; i < dependencyCount; ++i) {
    const int prerequisite = (int)dependencies[i].prerequisiteJoint;
    const int target = (int)dependencies[i].targetJoint;
    if (prerequisite < 0 || prerequisite >= FLC_CALIBRATION_JOINT_COUNT ||
        target < 0 || target >= FLC_CALIBRATION_JOINT_COUNT ||
        prerequisite == target || indegree[target] == 255) return false;
    ++indegree[target];
  }
  for (int output = 0; output < FLC_CALIBRATION_JOINT_COUNT; ++output) {
    int ready = -1;
    for (int joint = 0; joint < FLC_CALIBRATION_JOINT_COUNT; ++joint) {
      if (!emitted[joint] && indegree[joint] == 0) {
        ready = joint;
        break;
      }
    }
    if (ready < 0) return false;
    emitted[ready] = true;
    order[output] = (FlcGeometryJoint)ready;
    for (int edge = 0; edge < dependencyCount; ++edge) {
      if ((int)dependencies[edge].prerequisiteJoint != ready) continue;
      const int target = (int)dependencies[edge].targetJoint;
      if (indegree[target] == 0) return false;
      --indegree[target];
    }
  }
  return true;
}

inline bool flcPlanCatalogValid(const FlcJointPlan *plans, int planCount) {
  if (plans == 0 || planCount != FLC_CALIBRATION_JOINT_COUNT) return false;
  bool busIdSeen[254] = {false};
  for (int i = 0; i < planCount; ++i) {
    const FlcJointPlan &plan = plans[i];
    if ((int)plan.geometryJoint != i || plan.busId == 0 || plan.busId >= 254 ||
        busIdSeen[plan.busId] || !plan.q0WatchKnown ||
        plan.q0WatchTick < 0 || plan.q0WatchTick > FLC_ENCODER_MAX ||
        plan.q0WatchToleranceTicks == 0 || !plan.envelope.valid) return false;
    busIdSeen[plan.busId] = true;
  }
  return true;
}

inline bool flcAnglePicoRadToTicks(int64_t anglePicoRad, int *ticksOut) {
  if (ticksOut == 0) return false;
  static const int64_t TWO_PI_PICORAD = 6283185307179LL;
  const int64_t numerator = anglePicoRad * (int64_t)FLC_ENCODER_MODULUS;
  int64_t rounded;
  if (numerator >= 0)
    rounded = (numerator + TWO_PI_PICORAD / 2) / TWO_PI_PICORAD;
  else
    rounded = (numerator - TWO_PI_PICORAD / 2) / TWO_PI_PICORAD;
  if (rounded <= -FLC_HALF_ENCODER_RANGE ||
      rounded >= FLC_HALF_ENCODER_RANGE) return false;
  *ticksOut = (int)rounded;
  return true;
}

inline bool flcRawTickForQ(const FlcRuntimeCalibration &calibration,
                           int qTicks, int *rawTickOut) {
  if (rawTickOut == 0 || !calibration.calibrationUsable ||
      (calibration.direction != 1 && calibration.direction != -1) ||
      calibration.q0Tick < 0 || calibration.q0Tick > FLC_ENCODER_MAX) return false;
  const long raw = (long)calibration.q0Tick +
                   (long)calibration.direction * (long)qTicks;
  if (raw < 0 || raw > FLC_ENCODER_MAX) return false;
  *rawTickOut = (int)raw;
  return true;
}

inline bool flcBuildQ0Watch(const FlcJointPlan *plans,
                            const FlcRuntimeCalibration *runtime,
                            uint16_t mask, FlcMotionWatch &watch) {
  watch.count = 0;
  if ((mask & (uint16_t)~FLC_ALL_GEOMETRY_JOINTS_MASK) != 0) return false;
  for (int joint = 0; joint < FLC_CALIBRATION_JOINT_COUNT; ++joint) {
    if ((mask & (uint16_t)(1U << joint)) == 0) continue;
    if (!runtime[joint].holdKnown || watch.count >= FLC_MAX_WATCHED_JOINTS)
      return false;
    FlcWatchJoint &entry = watch.joints[watch.count++];
    entry.busId = plans[joint].busId;
    entry.expectedTick = runtime[joint].holdTick;
    entry.toleranceTicks = runtime[joint].holdToleranceTicks;
    entry.expectedTorqueState = 0;
    entry.expectedTorqueLimit = 0;
    entry.expectedGoalTick = runtime[joint].holdTick;
  }
  return true;
}

inline bool flcAddArmedWatch(FlcMotionWatch &watch, uint8_t busId,
                             int expectedTick, uint16_t toleranceTicks,
                             uint16_t torqueLimit) {
  if (watch.count < 0 || watch.count >= FLC_MAX_WATCHED_JOINTS ||
      expectedTick < 0 || expectedTick > FLC_ENCODER_MAX) return false;
  FlcWatchJoint &entry = watch.joints[watch.count++];
  entry.busId = busId;
  entry.expectedTick = expectedTick;
  entry.toleranceTicks = toleranceTicks;
  entry.expectedTorqueState = 1;
  entry.expectedTorqueLimit = torqueLimit;
  entry.expectedGoalTick = expectedTick;
  return true;
}

inline bool flcSafeOffAll(const FlcServoPort &port,
                          const FlcJointPlan *plans, int planCount,
                          uint8_t *failedBusId = 0) {
  bool allVerified = true;
  if (failedBusId != 0) *failedBusId = 0;
  for (int i = 0; i < planCount; ++i) {
    if (!flcEndMotion(port, plans[i].busId)) {
      allVerified = false;
      if (failedBusId != 0 && *failedBusId == 0)
        *failedBusId = plans[i].busId;
    }
  }
  return allVerified;
}

// Establishes a coherent q=0 watch state while torque remains OFF. The
// GoalPosition write cannot move the servo, but removes every stale retained
// goal before later torque transactions and makes watched-goal checks exact.
inline int flcPrepareQ0WatchState(const FlcServoPort &port,
                                  const FlcJointPlan *plans,
                                  FlcRuntimeCalibration *runtime,
                                  const FlcContactConfig &guards,
                                  uint8_t *failedBusId) {
  if (failedBusId != 0) *failedBusId = 0;
  for (int joint = 0; joint < FLC_CALIBRATION_JOINT_COUNT; ++joint) {
    const FlcJointPlan &plan = plans[joint];
    FlcObservation before = {};
    int readFault = flcReadFreshTelemetry(port, plan.busId, &before);
    if (readFault != FLC_ABORT_NONE) {
      if (failedBusId != 0) *failedBusId = plan.busId;
      return readFault;
    }
    int fault = flcCheckObservationGuards(before, guards, 0, 0, 0, 0, -1);
    if (fault != FLC_ABORT_NONE ||
        flcCircularDistance(before.position, plan.q0WatchTick) >
            (int)plan.q0WatchToleranceTicks) {
      if (failedBusId != 0) *failedBusId = plan.busId;
      return fault == FLC_ABORT_NONE ? FLC_ABORT_PREREQUISITE_DRIFT : fault;
    }
    if (!flcCommandPositionChecked(port, plan.busId, plan.q0WatchTick,
                                   plan.envelope.goalSpeed,
                                   plan.envelope.acceleration)) {
      if (failedBusId != 0) *failedBusId = plan.busId;
      return port.validateContext(port.ctx, plan.busId)
                 ? FLC_ABORT_DRIVER_ERROR
                 : FLC_ABORT_CONTEXT_DRIFT;
    }
    FlcObservation held = {};
    readFault = flcReadFreshTelemetry(port, plan.busId, &held);
    if (readFault != FLC_ABORT_NONE) {
      if (failedBusId != 0) *failedBusId = plan.busId;
      return readFault;
    }
    fault = flcCheckObservationGuards(held, guards, 0, 0, 0, 0,
                                      plan.q0WatchTick);
    if (fault != FLC_ABORT_NONE ||
        flcCircularDistance(held.position, plan.q0WatchTick) >
            (int)plan.q0WatchToleranceTicks) {
      if (failedBusId != 0) *failedBusId = plan.busId;
      return fault == FLC_ABORT_NONE ? FLC_ABORT_PREREQUISITE_DRIFT : fault;
    }
    runtime[joint].holdKnown = true;
    runtime[joint].holdTick = plan.q0WatchTick;
    runtime[joint].holdToleranceTicks = plan.q0WatchToleranceTicks;
  }
  return FLC_ABORT_NONE;
}

inline FlcParkingTransaction flcEnterEndpointParking(
    const FlcServoPort &port, const FlcEndpointGeometryPlan &geometry,
    const FlcJointPlan *plans, const FlcRuntimeCalibration *runtime,
    const FlcContactConfig &guards) {
  FlcParkingTransaction tx = {};
  tx.status = FLC_ENGINE_OK;
  tx.abortReason = FLC_ABORT_NONE;
  tx.auxiliaryJoint = FLC_GEOMETRY_JOINT_NONE;
  tx.savedTick = -1;
  tx.parkingTick = -1;
  tx.achievedTick = -1;
  tx.restoredTick = -1;

  if (geometry.parkingOutcome == FLC_NO_PARKING_REQUIRED) {
    tx.noParkingProvenanceVerified =
        geometry.auxiliaryJoint == FLC_GEOMETRY_JOINT_NONE;
    tx.restoreVerified = tx.noParkingProvenanceVerified;
    if (!tx.noParkingProvenanceVerified) {
      tx.status = FLC_ENGINE_INVALID_PLAN;
      tx.abortReason = FLC_ABORT_INVALID_PLAN;
    }
    return tx;
  }
  if (geometry.parkingOutcome != FLC_PARKING_REQUIRED_1DOF) {
    tx.status = FLC_ENGINE_INVALID_PLAN;
    tx.abortReason = FLC_ABORT_INVALID_PLAN;
    return tx;
  }

  tx.required = true;
  const int auxiliary = (int)geometry.auxiliaryJoint;
  if (auxiliary < 0 || auxiliary >= FLC_CALIBRATION_JOINT_COUNT ||
      !runtime[auxiliary].calibrationUsable) {
    tx.status = FLC_ENGINE_PREREQUISITE_DRIFT;
    tx.abortReason = FLC_ABORT_PREREQUISITE_DRIFT;
    return tx;
  }
  tx.prerequisiteVerified = true;
  tx.auxiliaryJoint = geometry.auxiliaryJoint;
  tx.auxiliaryBusId = plans[auxiliary].busId;
  tx.savedTick = runtime[auxiliary].holdTick;

  int parkingQTicks = 0;
  if (!flcAnglePicoRadToTicks(geometry.auxiliaryAnglePicoRad,
                              &parkingQTicks) ||
      !flcRawTickForQ(runtime[auxiliary], parkingQTicks, &tx.parkingTick)) {
    tx.status = FLC_ENGINE_REFUSED_DOMAIN;
    tx.abortReason = FLC_ABORT_POSITION_OUT_OF_DOMAIN;
    return tx;
  }

  FlcMotionWatch watch = {};
  const uint16_t auxiliaryMask = flcGeometryJointMask(geometry.auxiliaryJoint);
  if (!flcBuildQ0Watch(plans, runtime,
                       (uint16_t)(FLC_ALL_GEOMETRY_JOINTS_MASK &
                                  (uint16_t)~auxiliaryMask), watch)) {
    tx.status = FLC_ENGINE_INVALID_PLAN;
    tx.abortReason = FLC_ABORT_INVALID_PLAN;
    return tx;
  }

  const FlcJointPlan &auxPlan = plans[auxiliary];
  const FlcBeginMotionResult begun = flcBeginMotion(
      port, auxPlan.busId, tx.savedTick, runtime[auxiliary].holdToleranceTicks,
      auxPlan.envelope, guards, &watch);
  if (begun.status != FLC_ENGINE_OK) {
    tx.status = begun.status;
    tx.abortReason = begun.abortReason;
    return tx;
  }
  tx.entered = true;
  const FlcReturnResult moved = flcReturnTo(
      port, auxPlan.busId, tx.parkingTick, auxPlan.envelope, guards, &watch);
  tx.achievedTick = moved.achievedTick;
  if (moved.status != FLC_ENGINE_OK) {
    tx.status = FLC_ENGINE_PARKING_FAILED;
    tx.abortReason = moved.abortReason;
    flcRequireTorqueOff(port, auxPlan.busId, tx.status, tx.abortReason);
    return tx;
  }
  tx.trackingVerified =
      flcCircularDistance(tx.achievedTick, tx.parkingTick) <=
      (int)guards.targetReachedToleranceTicks;
  if (!tx.trackingVerified) {
    tx.status = FLC_ENGINE_PARKING_FAILED;
    tx.abortReason = FLC_ABORT_TRACKING_FAILURE;
    flcRequireTorqueOff(port, auxPlan.busId, tx.status, tx.abortReason);
  }
  return tx;
}

inline bool flcExitEndpointParking(
    const FlcServoPort &port, const FlcEndpointGeometryPlan &geometry,
    const FlcJointPlan *plans, const FlcRuntimeCalibration *runtime,
    const FlcContactConfig &guards, bool allowRestoreMotion,
    bool targetAtQ0, FlcParkingTransaction &tx) {
  if (!tx.required) return tx.restoreVerified;
  if (!tx.entered) return false;
  const int auxiliary = (int)tx.auxiliaryJoint;
  const FlcJointPlan &auxPlan = plans[auxiliary];
  bool ok = true;

  if (allowRestoreMotion) {
    tx.restoreAttempted = true;
    FlcMotionWatch watch = {};
    uint16_t mask = geometry.q0HeldDuringTaskMask;
    if (targetAtQ0)
      mask = (uint16_t)(FLC_ALL_GEOMETRY_JOINTS_MASK &
                        (uint16_t)~flcGeometryJointMask(tx.auxiliaryJoint));
    if (!flcBuildQ0Watch(plans, runtime, mask, watch)) {
      tx.status = FLC_ENGINE_RESTORE_FAILED;
      tx.abortReason = FLC_ABORT_INVALID_PLAN;
      ok = false;
    } else {
      const FlcReturnResult restored = flcReturnTo(
          port, auxPlan.busId, tx.savedTick, auxPlan.envelope, guards, &watch);
      tx.restoredTick = restored.achievedTick;
      if (restored.status != FLC_ENGINE_OK ||
          flcCircularDistance(tx.restoredTick, tx.savedTick) >
              (int)runtime[auxiliary].holdToleranceTicks) {
        tx.status = FLC_ENGINE_RESTORE_FAILED;
        tx.abortReason = restored.abortReason == FLC_ABORT_NONE
                             ? FLC_ABORT_RESTORE_FAILED
                             : restored.abortReason;
        ok = false;
      }
    }
  } else {
    tx.status = FLC_ENGINE_RESTORE_FAILED;
    tx.abortReason = FLC_ABORT_RESTORE_FAILED;
    ok = false;
  }

  if (!flcEndMotion(port, auxPlan.busId)) {
    tx.status = FLC_ENGINE_TORQUE_OFF_FAILED;
    tx.abortReason = FLC_ABORT_TORQUE_OFF_UNVERIFIED;
    ok = false;
  }
  tx.restoreVerified = ok && tx.restoreAttempted;
  return tx.restoreVerified;
}

inline FlcEndpointExecution flcExecutePlannedEndpoint(
    const FlcServoPort &port, const FlcJointPlan &targetPlan,
    const FlcEndpointGeometryPlan &geometry, int8_t rawProbeSign,
    const FlcMotionEnvelope &endpointEnvelope,
    const FlcJointPlan *plans, const FlcRuntimeCalibration *runtime,
    const FlcContactConfig &guards) {
  FlcEndpointExecution execution = {};
  execution.status = FLC_ENGINE_OK;
  execution.abortReason = FLC_ABORT_NONE;
  execution.geometryEndpointIndex = geometry.canonicalEndpointIndex;
  execution.parkingOutcome = geometry.parkingOutcome;

  execution.parking = flcEnterEndpointParking(
      port, geometry, plans, runtime, guards);
  if (execution.parking.status != FLC_ENGINE_OK) {
    execution.status = execution.parking.status;
    execution.abortReason = execution.parking.abortReason;
    return execution;
  }

  FlcMotionWatch taskWatch = {};
  if (!flcBuildQ0Watch(plans, runtime, geometry.q0HeldDuringTaskMask,
                       taskWatch)) {
    execution.status = FLC_ENGINE_INVALID_PLAN;
    execution.abortReason = FLC_ABORT_INVALID_PLAN;
  } else if (execution.parking.required &&
             !flcAddArmedWatch(taskWatch,
                               execution.parking.auxiliaryBusId,
                               execution.parking.parkingTick,
                               guards.targetReachedToleranceTicks,
                               plans[(int)execution.parking.auxiliaryJoint]
                                   .envelope.torqueLimit)) {
    execution.status = FLC_ENGINE_INVALID_PLAN;
    execution.abortReason = FLC_ABORT_INVALID_PLAN;
  } else {
    execution.q0WatchVerified = true;
  }

  bool targetArmed = false;
  bool targetOff = true;
  if (execution.status == FLC_ENGINE_OK) {
    const int targetIndex = (int)targetPlan.geometryJoint;
    const FlcBeginMotionResult begun = flcBeginMotion(
        port, targetPlan.busId, runtime[targetIndex].holdTick,
        runtime[targetIndex].holdToleranceTicks, endpointEnvelope, guards,
        &taskWatch);
    execution.targetHoldGoalVerified = begun.holdGoalVerified;
    if (begun.status != FLC_ENGINE_OK) {
      execution.status = begun.status;
      execution.abortReason = begun.abortReason;
      targetOff = flcEndMotion(port, targetPlan.busId);
    } else {
      targetArmed = true;
      execution.measurement = flcMeasureEndpoint(
          port, targetPlan.busId, runtime[targetIndex].holdTick, rawProbeSign,
          endpointEnvelope, guards, targetPlan.baseline,
          targetPlan.gates.repeatabilityToleranceTicks, &taskWatch);
      if (execution.measurement.status != FLC_ENGINE_OK) {
        execution.status = execution.measurement.status;
        execution.abortReason = execution.measurement.abortReason;
      } else {
        execution.returnToQ0 = flcReturnTo(
            port, targetPlan.busId, runtime[targetIndex].holdTick,
            endpointEnvelope, guards, &taskWatch);
        if (execution.returnToQ0.status != FLC_ENGINE_OK) {
          execution.status = execution.returnToQ0.status;
          execution.abortReason = execution.returnToQ0.abortReason;
        } else {
          execution.targetReturnedToQ0 = true;
        }
      }
      targetOff = flcEndMotion(port, targetPlan.busId);
    }
  }

  (void)targetArmed;
  execution.targetTorqueOffVerified = targetOff;
  if (!targetOff) {
    execution.status = FLC_ENGINE_TORQUE_OFF_FAILED;
    execution.abortReason = FLC_ABORT_TORQUE_OFF_UNVERIFIED;
  }

  const bool restored = flcExitEndpointParking(
      port, geometry, plans, runtime, guards,
      targetOff, execution.targetReturnedToQ0, execution.parking);
  if (!restored && execution.parking.required) {
    if (execution.parking.status == FLC_ENGINE_TORQUE_OFF_FAILED ||
        execution.status == FLC_ENGINE_OK) {
      execution.status = execution.parking.status;
      execution.abortReason = execution.parking.abortReason;
    }
  }
  return execution;
}

inline FlcJointCalibrationResult flcCalibratePlannedJoint(
    const FlcServoPort &port, const FlcJointPlan *plans,
    const FlcRuntimeCalibration *runtime, FlcGeometryJoint targetJoint,
    const FlcContactConfig &guards) {
  FlcJointCalibrationResult result = {};
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.geometryJoint = targetJoint;
  result.direction = 0;
  result.directionMethod = FLC_DIRECTION_UNRESOLVED;
  result.rawLoTick = -1;
  result.rawHiTick = -1;
  result.minContactTick = -1;
  result.maxContactTick = -1;
  result.manualPoseQ0CandidateTick = -1;
  result.derivedQ0FinalTick = -1;
  result.q0Tick = -1;
  result.q0CrosscheckStatus = FLC_Q0_CROSSCHECK_NOT_RUN;
  result.tier = FLC_TIER_MEASURED;

  const int target = (int)targetJoint;
  if (target < 0 || target >= FLC_CALIBRATION_JOINT_COUNT) {
    result.status = FLC_ENGINE_INVALID_PLAN;
    result.abortReason = FLC_ABORT_INVALID_PLAN;
    return result;
  }
  const FlcJointPlan &plan = plans[target];
  result.busId = plan.busId;
  result.expectedSpanTicks = plan.gates.expectedSpanTicks;
  if (!plan.characterized || !plan.baseline.valid || !plan.envelope.valid) {
    result.status = FLC_ENGINE_PREREQUISITE_DRIFT;
    result.abortReason = FLC_ABORT_PREREQUISITE_DRIFT;
    return result;
  }
  if (!plan.gates.repeatabilityToleranceKnown ||
      plan.gates.repeatabilityToleranceTicks == 0) {
    result.status = FLC_ENGINE_REPEATABILITY_FAILED;
    return result;
  }

  // Endpoint-specific geometry prerequisites must be selected BEFORE moving.
  // Therefore H4 requires an explicit current-build semantic witness. It is not
  // a stale numeric direction and is later cross-checked against both measured
  // endpoints plus the H2 manual q0 evidence.
  const int8_t witness = plan.directionEvidence.operatorWitness;
  if (witness != 1 && witness != -1) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }

  const FlcEndpointGeometryPlan *minGeometry =
      flcGeometryPlanFor(flcGeometryJointName(targetJoint), "min");
  const FlcEndpointGeometryPlan *maxGeometry =
      flcGeometryPlanFor(flcGeometryJointName(targetJoint), "max");
  if (minGeometry == 0 || maxGeometry == 0 ||
      minGeometry->targetJoint != targetJoint ||
      maxGeometry->targetJoint != targetJoint) {
    result.status = FLC_ENGINE_INVALID_PLAN;
    result.abortReason = FLC_ABORT_INVALID_PLAN;
    return result;
  }

  FlcMotionEnvelope minEnvelope = plan.envelope;
  minEnvelope.travelBudgetTicks =
      flcTravelBudgetForDistance(plan.directionEvidence.qMinTicks);
  minEnvelope = flcClampEnvelope(minEnvelope);
  FlcMotionEnvelope maxEnvelope = plan.envelope;
  maxEnvelope.travelBudgetTicks =
      flcTravelBudgetForDistance(plan.directionEvidence.qMaxTicks);
  maxEnvelope = flcClampEnvelope(maxEnvelope);

  result.minExecution = flcExecutePlannedEndpoint(
      port, plan, *minGeometry, (int8_t)-witness, minEnvelope, plans,
      runtime, guards);
  result.minEndpoint = result.minExecution.measurement;
  if (result.minExecution.status != FLC_ENGINE_OK) {
    result.status = result.minExecution.status;
    result.abortReason = result.minExecution.abortReason;
    return result;
  }
  result.minContactTick = result.minEndpoint.contactTick;

  result.maxExecution = flcExecutePlannedEndpoint(
      port, plan, *maxGeometry, witness, maxEnvelope, plans, runtime, guards);
  result.maxEndpoint = result.maxExecution.measurement;
  if (result.maxExecution.status != FLC_ENGINE_OK) {
    result.status = result.maxExecution.status;
    result.abortReason = result.maxExecution.abortReason;
    return result;
  }
  result.maxContactTick = result.maxEndpoint.contactTick;

  result.rawLoTick = result.minContactTick < result.maxContactTick
                         ? result.minContactTick
                         : result.maxContactTick;
  result.rawHiTick = result.minContactTick > result.maxContactTick
                         ? result.minContactTick
                         : result.maxContactTick;
  result.measuredSpanTicks = result.rawHiTick - result.rawLoTick;
  result.spanErrorTicks =
      result.measuredSpanTicks - plan.gates.expectedSpanTicks;
  if (result.measuredSpanTicks <= 0 ||
      result.measuredSpanTicks >= FLC_HALF_ENCODER_RANGE) {
    result.status = FLC_ENGINE_GEOMETRY_INCONSISTENT;
    result.abortReason = FLC_ABORT_ENDPOINT_ORDER_CONTRADICTION;
    return result;
  }

  FlcDirectionEvidence evidence = plan.directionEvidence;
  evidence.rawLoTick = result.rawLoTick;
  evidence.rawHiTick = result.rawHiTick;
  result.directionResolution = flcResolveJointDirection(evidence);
  if (result.directionResolution.status != FLC_ENGINE_OK ||
      result.directionResolution.direction != witness) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }
  result.direction = result.directionResolution.direction;
  result.directionMethod = result.directionResolution.method;
  result.derivedQ0FinalTick = result.direction > 0
                                  ? result.directionResolution.q0IfPositive
                                  : result.directionResolution.q0IfNegative;
  result.q0Tick = result.derivedQ0FinalTick;
  result.tier = FLC_TIER_CANDIDATE;

  if (plan.directionEvidence.manualQ0Known) {
    result.manualPoseQ0CandidateTick = plan.directionEvidence.manualQ0Tick;
    result.manualVsDerivedQ0ErrorTicks = flcCircularDistance(
        result.manualPoseQ0CandidateTick, result.derivedQ0FinalTick);
    if (!plan.gates.manualVsDerivedQ0ToleranceKnown) {
      result.q0CrosscheckStatus = FLC_Q0_BLOCKED_TOLERANCE_UNVALIDATED;
    } else if (result.manualVsDerivedQ0ErrorTicks <=
               (int)plan.gates.manualVsDerivedQ0ToleranceTicks) {
      result.q0CrosscheckStatus = FLC_Q0_CROSSCHECK_MATCH;
    } else {
      result.q0CrosscheckStatus = FLC_Q0_CROSSCHECK_MISMATCH;
      result.status = FLC_ENGINE_Q0_CROSSCHECK_FAILED;
      result.abortReason = FLC_ABORT_Q0_CROSSCHECK_FAILED;
      return result;
    }
  } else {
    result.q0CrosscheckStatus = FLC_Q0_BLOCKED_MANUAL_CANDIDATE_MISSING;
  }

  if (plan.gates.endpointVsUrdfToleranceKnown &&
      flcAbsInt(result.spanErrorTicks) >
          (int)plan.gates.endpointVsUrdfToleranceTicks) {
    result.status = FLC_ENGINE_GEOMETRY_INCONSISTENT;
    result.abortReason = FLC_ABORT_ENDPOINT_GEOMETRY_MISMATCH;
    return result;
  }

  if (plan.gates.endpointVsUrdfToleranceKnown &&
      result.q0CrosscheckStatus == FLC_Q0_CROSSCHECK_MATCH) {
    result.tier = FLC_TIER_ACCEPTED;
    result.accepted = true;
  }
  return result;
}

inline FlcCalibrationRunResult flcRunCalibrationPlan(
    const FlcServoPort &port, const FlcContactConfig &guards,
    const FlcJointPlan *plans, int planCount, uint16_t selectionMask) {
  FlcCalibrationRunResult run = {};
  run.status = FLC_ENGINE_OK;
  run.abortReason = FLC_ABORT_NONE;
  run.selectionMask = selectionMask;
  run.failedJoint = FLC_GEOMETRY_JOINT_NONE;

  int sourceNoParking = 0;
  int sourceParking = 0;
  FlcGeometryJoint dependencyOrder[FLC_CALIBRATION_JOINT_COUNT];
  if (!flcPortUsable(port) ||
      !flcPlanCatalogValid(plans, planCount) ||
      !flcGeometryPlanSummary(sourceNoParking, sourceParking)) {
    run.status = !flcPortUsable(port) ? FLC_ENGINE_REFUSED_PORT
                                     : FLC_ENGINE_INVALID_PLAN;
    run.abortReason = FLC_ABORT_INVALID_PLAN;
    return run;
  }
  if (!flcBuildDependencyOrder(FLC_GEOMETRY_DEPENDENCIES,
                               FLC_GEOMETRY_DEPENDENCY_COUNT,
                               dependencyOrder,
                               FLC_CALIBRATION_JOINT_COUNT)) {
    run.status = FLC_ENGINE_DEPENDENCY_CYCLE;
    run.abortReason = FLC_ABORT_DEPENDENCY_CYCLE;
    return run;
  }
  selectionMask &= FLC_ALL_GEOMETRY_JOINTS_MASK;
  run.selectionMask = selectionMask;
  if (selectionMask == 0) {
    run.status = FLC_ENGINE_INVALID_PLAN;
    run.abortReason = FLC_ABORT_INVALID_PLAN;
    return run;
  }

  FlcRuntimeCalibration runtime[FLC_CALIBRATION_JOINT_COUNT] = {};
  for (int joint = 0; joint < FLC_CALIBRATION_JOINT_COUNT; ++joint) {
    runtime[joint].holdKnown = plans[joint].q0WatchKnown;
    runtime[joint].holdTick = plans[joint].q0WatchTick;
    runtime[joint].holdToleranceTicks = plans[joint].q0WatchToleranceTicks;
    runtime[joint].calibrationUsable =
        plans[joint].calibrationKnown &&
        (plans[joint].knownDirection == 1 ||
         plans[joint].knownDirection == -1) &&
        plans[joint].knownQ0Tick >= 0 &&
        plans[joint].knownQ0Tick <= FLC_ENCODER_MAX;
    runtime[joint].direction = plans[joint].knownDirection;
    runtime[joint].q0Tick = plans[joint].knownQ0Tick;
    if ((selectionMask & (uint16_t)(1U << joint)) != 0)
      ++run.jointsRequested;
  }

  uint8_t preflightFailedId = 0;
  const int preflightFault = flcPrepareQ0WatchState(
      port, plans, runtime, guards, &preflightFailedId);
  if (preflightFault != FLC_ABORT_NONE) {
    run.status = flcEngineStatusForAbort(preflightFault);
    run.abortReason = preflightFault;
    run.failedBusId = preflightFailedId;
  }

  for (int ordinal = 0;
       ordinal < FLC_CALIBRATION_JOINT_COUNT && run.status == FLC_ENGINE_OK;
       ++ordinal) {
    const FlcGeometryJoint joint = dependencyOrder[ordinal];
    const int jointIndex = (int)joint;
    if ((selectionMask & (uint16_t)(1U << jointIndex)) == 0) continue;
    run.executionOrder[run.executionCount++] = joint;

    // Every generated predecessor must already have usable current-session
    // calibration (or explicit prior evidence when outside this selection).
    for (int edge = 0; edge < FLC_GEOMETRY_DEPENDENCY_COUNT; ++edge) {
      if (FLC_GEOMETRY_DEPENDENCIES[edge].targetJoint != joint) continue;
      const int prerequisite =
          (int)FLC_GEOMETRY_DEPENDENCIES[edge].prerequisiteJoint;
      if (!runtime[prerequisite].calibrationUsable) {
        run.status = FLC_ENGINE_PREREQUISITE_DRIFT;
        run.abortReason = FLC_ABORT_PREREQUISITE_DRIFT;
        run.failedJoint = joint;
        run.failedBusId = plans[jointIndex].busId;
        break;
      }
    }
    if (run.status != FLC_ENGINE_OK) break;

    FlcJointCalibrationResult jointResult = flcCalibratePlannedJoint(
        port, plans, runtime, joint, guards);
    run.resultPresent[jointIndex] = true;
    run.joints[jointIndex] = jointResult;
    if (jointResult.minExecution.parkingOutcome == FLC_PARKING_REQUIRED_1DOF)
      ++run.parkingRequiredCount;
    else
      ++run.noParkingCount;
    if (jointResult.maxExecution.parkingOutcome == FLC_PARKING_REQUIRED_1DOF)
      ++run.parkingRequiredCount;
    else
      ++run.noParkingCount;
    if (jointResult.minExecution.parking.restoreVerified &&
        jointResult.minExecution.parking.required)
      ++run.parkingRestoreCount;
    if (jointResult.maxExecution.parking.restoreVerified &&
        jointResult.maxExecution.parking.required)
      ++run.parkingRestoreCount;

    if (jointResult.status != FLC_ENGINE_OK) {
      run.status = jointResult.status;
      run.abortReason = jointResult.abortReason;
      run.failedJoint = joint;
      run.failedBusId = plans[jointIndex].busId;
      break;
    }

    runtime[jointIndex].calibrationUsable = true;
    runtime[jointIndex].direction = jointResult.direction;
    runtime[jointIndex].q0Tick = jointResult.derivedQ0FinalTick;
    ++run.jointsOk;
  }

  uint8_t safeOffFailedId = 0;
  run.safeOffVerified = flcSafeOffAll(port, plans, planCount,
                                      &safeOffFailedId);
  if (!run.safeOffVerified) {
    run.status = FLC_ENGINE_TORQUE_OFF_FAILED;
    run.abortReason = FLC_ABORT_TORQUE_OFF_UNVERIFIED;
    run.hardFaultCutPowerNow = true;
    if (run.failedBusId == 0) run.failedBusId = safeOffFailedId;
  }
  for (int joint = 0; joint < FLC_CALIBRATION_JOINT_COUNT; ++joint)
    run.finalCalibration[joint] = runtime[joint];
  return run;
}

#endif  // FLC_CALIBRATION_ENGINE_H
