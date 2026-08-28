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
  return port.readTelemetry != 0 && port.commandPosition != 0 &&
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
    default: return "UNKNOWN";
  }
}

// --------------------------------------------------------------------------
// Torque transaction. Every motion path enters through here and, critically,
// LEAVES through flcEndMotion() on every exit including every failure.
// --------------------------------------------------------------------------

inline bool flcEndMotion(const FlcServoPort &port, uint8_t id) {
  if (port.setTorqueEnable == 0) return false;
  bool ok = port.setTorqueEnable(port.ctx, id, false);
  FlcObservation o;
  if (port.readTelemetry != 0 && port.readTelemetry(port.ctx, id, &o)) {
    // Readback is authoritative: a servo that still reports torque on has NOT
    // been released, whatever the write acknowledged.
    if (o.torqueEnabled) return false;
  }
  return ok;
}

inline int flcBeginMotion(const FlcServoPort &port, uint8_t id,
                          const FlcMotionEnvelope &envelope) {
  if (!flcPortUsable(port)) return FLC_ENGINE_REFUSED_PORT;
  if (!envelope.valid) return FLC_ENGINE_REFUSED_ENVELOPE;
  if (!port.setTorqueLimit(port.ctx, id, envelope.torqueLimit))
    return FLC_ENGINE_TORQUE_SETUP_FAILED;
  if (!port.setTorqueEnable(port.ctx, id, true)) return FLC_ENGINE_TORQUE_SETUP_FAILED;
  return FLC_ENGINE_OK;
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
    uint16_t excursionTicks) {
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
  if (!port.commandPosition(port.ctx, id, target, envelope.goalSpeed, envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  uint16_t currents[64];
  int count = 0;
  const uint32_t began = port.nowMs(port.ctx);

  while (count < 64) {
    const uint32_t now = port.nowMs(port.ctx);
    const uint32_t elapsed = now - began;
    if (elapsed > envelope.timeBudgetMs) break;

    FlcObservation o;
    if (!port.readTelemetry(port.ctx, id, &o)) {
      result.status = FLC_ENGINE_TELEMETRY_FAILED;
      result.abortReason = FLC_ABORT_TELEMETRY_TIMEOUT;
      return result;
    }
    // The independent hard guards apply during baseline exactly as during a
    // contact approach: a thermal or voltage excursion here is still a fault.
    if (o.statusByte != 0) { result.abortReason = FLC_ABORT_STATUS_ERROR; result.status = FLC_ENGINE_ABORTED; return result; }
    if ((int)o.temperature >= guards.thermalLimitC) { result.abortReason = FLC_ABORT_THERMAL; result.status = FLC_ENGINE_ABORTED; return result; }
    if ((int)o.voltage < guards.voltageMin || (int)o.voltage > guards.voltageMax) { result.abortReason = FLC_ABORT_VOLTAGE; result.status = FLC_ENGINE_ABORTED; return result; }
    if (o.current >= guards.hardCurrentAbortRaw) { result.abortReason = FLC_ABORT_OVERCURRENT; result.status = FLC_ENGINE_ABORTED; return result; }

    const int travel = flcDirectionalProgress(o.position, startTick, probeSign);
    if (travel > result.travelTicks) result.travelTicks = travel;
    result.endTick = o.position;

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
    const FlcBaseline &baseline) {
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
      if (!port.commandPosition(port.ctx, id, commandedTarget, envelope.goalSpeed,
                                envelope.acceleration)) {
        result.status = FLC_ENGINE_ABORTED;
        result.abortReason = FLC_ABORT_DRIVER_ERROR;
        return result;
      }
      haveTarget = true;
    }

    FlcObservation o;
    if (!port.readTelemetry(port.ctx, id, &o)) {
      result.status = FLC_ENGINE_TELEMETRY_FAILED;
      result.abortReason = FLC_ABORT_TELEMETRY_TIMEOUT;
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
    const FlcBaseline &baseline) {
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
  if (!port.commandPosition(port.ctx, id, target, envelope.goalSpeed, envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  const uint32_t began = port.nowMs(port.ctx);
  while (true) {
    const uint32_t elapsed = port.nowMs(port.ctx) - began;
    if (elapsed > envelope.timeBudgetMs) {
      result.status = FLC_ENGINE_RETREAT_FAILED;
      result.abortReason = FLC_ABORT_TIME_BUDGET_EXCEEDED;
      return result;
    }

    FlcObservation o;
    if (!port.readTelemetry(port.ctx, id, &o)) {
      result.status = FLC_ENGINE_TELEMETRY_FAILED;
      result.abortReason = FLC_ABORT_TELEMETRY_TIMEOUT;
      return result;
    }
    if (o.statusByte != 0) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_STATUS_ERROR; return result; }
    if (!o.torqueEnabled) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_TORQUE_UNEXPECTEDLY_OFF; return result; }
    if (o.current >= guards.hardCurrentAbortRaw) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_OVERCURRENT; return result; }
    if ((int)o.temperature >= guards.thermalLimitC) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_THERMAL; return result; }

    result.toTick = o.position;
    result.achievedTicks = flcDirectionalProgress(o.position, contactTick, retreatSign);
    result.restCurrent = o.current;

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
  if (result.achievedTicks < required || !result.trackingRecovered) {
    result.status = FLC_ENGINE_RETREAT_FAILED;
    result.abortReason = FLC_ABORT_NONE;
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
    const FlcBaseline &baseline, uint16_t repeatabilityToleranceTicks) {
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

  result.first = flcRunApproach(port, id, startTick, probeSign, envelope, guards, baseline);
  if (result.first.status != FLC_ENGINE_OK) {
    result.status = result.first.status;
    result.abortReason = result.first.abortReason;
    return result;
  }

  result.retreat = flcRetreatAndVerify(port, id, result.first.contactTick, probeSign,
                                       envelope, guards, baseline);
  if (result.retreat.status != FLC_ENGINE_OK) {
    result.status = result.retreat.status;
    result.abortReason = result.retreat.abortReason;
    return result;
  }

  // Second approach starts from where the retreat actually ended, so it is a
  // genuinely independent traversal rather than a replay of the first.
  result.second = flcRunApproach(port, id, result.retreat.toTick, probeSign, envelope,
                                 guards, baseline);
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
                                            probeSign, envelope, guards, baseline);
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
                                   const FlcContactConfig &guards) {
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
  if (!port.commandPosition(port.ctx, id, targetTick, envelope.goalSpeed,
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
    if (!port.readTelemetry(port.ctx, id, &o)) {
      result.status = FLC_ENGINE_TELEMETRY_FAILED;
      result.abortReason = FLC_ABORT_TELEMETRY_TIMEOUT;
      return result;
    }
    if (o.statusByte != 0) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_STATUS_ERROR; return result; }
    if (!o.torqueEnabled) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_TORQUE_UNEXPECTEDLY_OFF; return result; }
    if (o.current >= guards.hardCurrentAbortRaw) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_OVERCURRENT; return result; }
    if ((int)o.temperature >= guards.thermalLimitC) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_THERMAL; return result; }

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
// Direction measurement — MEASURED, never inherited from the leg label
// --------------------------------------------------------------------------

struct FlcDirectionResult {
  int status;
  int abortReason;
  int8_t encoderSign;   //: raw tick sign produced by a positive commanded step
  int observedTravel;
  int startTick;
  int endTick;
};

inline FlcDirectionResult flcMeasureDirection(
    const FlcServoPort &port, uint8_t id, int startTick,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    uint16_t probeTicks) {
  FlcDirectionResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.encoderSign = 0;
  result.observedTravel = 0;
  result.startTick = startTick;
  result.endTick = startTick;

  if (!flcPortUsable(port)) { result.status = FLC_ENGINE_REFUSED_PORT; return result; }
  if (!envelope.valid) { result.status = FLC_ENGINE_REFUSED_ENVELOPE; return result; }
  if (!flcPlanStaysInDomain(startTick, probeTicks, +1)) {
    result.status = FLC_ENGINE_REFUSED_DOMAIN;
    result.abortReason = FLC_ABORT_WRAP_BOUNDARY;
    return result;
  }

  const int target = startTick + (int)probeTicks;
  if (!port.commandPosition(port.ctx, id, target, envelope.goalSpeed, envelope.acceleration)) {
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_DRIVER_ERROR;
    return result;
  }

  const uint32_t began = port.nowMs(port.ctx);
  while (true) {
    const uint32_t elapsed = port.nowMs(port.ctx) - began;
    if (elapsed > envelope.timeBudgetMs) break;

    FlcObservation o;
    if (!port.readTelemetry(port.ctx, id, &o)) {
      result.status = FLC_ENGINE_TELEMETRY_FAILED;
      result.abortReason = FLC_ABORT_TELEMETRY_TIMEOUT;
      return result;
    }
    if (o.statusByte != 0) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_STATUS_ERROR; return result; }
    if (o.current >= guards.hardCurrentAbortRaw) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_OVERCURRENT; return result; }
    if ((int)o.temperature >= guards.thermalLimitC) { result.status = FLC_ENGINE_ABORTED; result.abortReason = FLC_ABORT_THERMAL; return result; }

    result.endTick = o.position;
    if (flcCircularDistance(o.position, target) <= (int)guards.targetReachedToleranceTicks) break;
    if (port.idle != 0) port.idle(port.ctx, 5);
  }

  const int delta = flcSignedTickDelta(result.endTick, startTick);
  result.observedTravel = delta < 0 ? -delta : delta;
  // Require a decisive excursion: a couple of ticks of noise is not a direction.
  if (result.observedTravel < (int)probeTicks / 2) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_TARGET_DID_NOT_MOVE;
    return result;
  }
  result.encoderSign = delta > 0 ? (int8_t)1 : (int8_t)-1;
  return result;
}

// --------------------------------------------------------------------------
// H3 — characterize one joint
//
// Produces the evidence that resolves the parameters H4 needs. This is the
// operation that breaks the old deadlock: it may run on a bootstrap envelope,
// and it is NOT blocked by the acceptance tolerances it exists to inform.
// --------------------------------------------------------------------------

struct FlcCharacterizationResult {
  int status;
  int abortReason;
  uint8_t busId;
  int startTick;
  FlcDirectionResult direction;
  FlcBaselineResult baseline;
  FlcApproachResult probeContact;
  FlcRetreatResult probeRetreat;
  FlcApproachResult probeContact2;
  FlcRetreatResult finalRetreat;
  //: characterized outputs, origin CHARACTERIZED_CURRENT_HARDWARE on success
  uint16_t characterizedContactThresholdRaw;
  uint16_t characterizedRetreatTicks;
  uint16_t characterizedRepeatabilityToleranceTicks;
  uint16_t observedContactSpreadTicks;
  uint16_t peakCurrent;
  uint8_t peakTemperature;
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
    const FlcServoPort &port, uint8_t id, int startTick, int8_t plannedProbeSign,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    uint16_t directionProbeTicks, uint16_t baselineExcursionTicks) {
  FlcCharacterizationResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.busId = id;
  result.startTick = startTick;
  result.characterizedContactThresholdRaw = 0;
  result.characterizedRetreatTicks = 0;
  result.observedContactSpreadTicks = 0;
  result.peakCurrent = 0;
  result.peakTemperature = 0;
  result.complete = false;

  const int begun = flcBeginMotion(port, id, envelope);
  if (begun != FLC_ENGINE_OK) {
    result.status = begun;
    flcEndMotion(port, id);
    return result;
  }

  // H3A — measure which way the encoder moves for a positive commanded step.
  result.direction = flcMeasureDirection(port, id, startTick, envelope, guards,
                                         directionProbeTicks);
  if (result.direction.status != FLC_ENGINE_OK) {
    result.status = result.direction.status;
    result.abortReason = result.direction.abortReason;
    flcEndMotion(port, id);
    return result;
  }
  // The measurement must agree with the direction the caller intended to probe;
  // if it does not, the plan and the physical joint disagree.
  if (plannedProbeSign != 0 && result.direction.encoderSign != plannedProbeSign) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    flcEndMotion(port, id);
    return result;
  }
  const int8_t probeSign = result.direction.encoderSign;

  // H3B — free-motion baseline, from which the contact threshold is derived.
  result.baseline = flcMeasureFreeMotionBaseline(port, id, result.direction.endTick,
                                                 probeSign, envelope, guards,
                                                 baselineExcursionTicks);
  if (result.baseline.status != FLC_ENGINE_OK) {
    result.status = result.baseline.status;
    result.abortReason = result.baseline.abortReason;
    flcEndMotion(port, id);
    return result;
  }

  // H3C — one supervised bounded approach to a real contact, plus retreat, to
  // prove the joint can be approached and released safely on this build.
  result.probeContact = flcRunApproach(port, id, result.baseline.endTick, probeSign,
                                       envelope, guards, result.baseline.baseline);
  if (result.probeContact.status != FLC_ENGINE_OK) {
    result.status = result.probeContact.status;
    result.abortReason = result.probeContact.abortReason;
    flcEndMotion(port, id);
    return result;
  }
  result.peakCurrent = result.probeContact.peakCurrent;
  result.peakTemperature = result.probeContact.peakTemperature;

  result.probeRetreat = flcRetreatAndVerify(port, id, result.probeContact.contactTick,
                                            probeSign, envelope, guards,
                                            result.baseline.baseline);
  if (result.probeRetreat.status != FLC_ENGINE_OK) {
    result.status = result.probeRetreat.status;
    result.abortReason = result.probeRetreat.abortReason;
    flcEndMotion(port, id);
    return result;
  }

  // H3D — a SECOND independent approach to the same stop. Without this, H3
  // could not report a repeatability spread, and H4 would have to inherit a
  // tolerance from somewhere else. This measures it on this build instead.
  result.probeContact2 = flcRunApproach(port, id, result.probeRetreat.toTick, probeSign,
                                        envelope, guards, result.baseline.baseline);
  if (result.probeContact2.status != FLC_ENGINE_OK) {
    result.status = result.probeContact2.status;
    result.abortReason = result.probeContact2.abortReason;
    flcEndMotion(port, id);
    return result;
  }
  if (result.probeContact2.peakCurrent > result.peakCurrent)
    result.peakCurrent = result.probeContact2.peakCurrent;

  const int spread = flcCircularDistance(result.probeContact.contactTick,
                                         result.probeContact2.contactTick);
  result.observedContactSpreadTicks = (uint16_t)spread;

  // Release the stop again so the joint does not end characterization loaded.
  result.finalRetreat = flcRetreatAndVerify(port, id, result.probeContact2.contactTick,
                                            probeSign, envelope, guards,
                                            result.baseline.baseline);
  if (result.finalRetreat.status != FLC_ENGINE_OK) {
    result.status = result.finalRetreat.status;
    result.abortReason = result.finalRetreat.abortReason;
    flcEndMotion(port, id);
    return result;
  }

  result.characterizedContactThresholdRaw =
      flcBaselineContactThreshold(result.baseline.baseline);
  result.characterizedRetreatTicks = (uint16_t)result.probeRetreat.achievedTicks;
  // Repeatability band derived from the spread this joint actually produced:
  // twice the observed spread, with an 8-tick floor so a lucky pair of
  // identical contacts cannot yield a band no real joint could satisfy.
  result.characterizedRepeatabilityToleranceTicks =
      flcRepeatabilityBandFromSpread(result.observedContactSpreadTicks);
  result.complete = true;

  // Characterization always ends with the joint released.
  if (!flcEndMotion(port, id)) {
    result.status = FLC_ENGINE_TORQUE_SETUP_FAILED;
    result.complete = false;
  }
  return result;
}

// --------------------------------------------------------------------------
// H4 — calibrate one joint: both endpoints, span, direction, q0 candidate
// --------------------------------------------------------------------------

//: How far a result has travelled toward becoming canonical calibration.
enum FlcResultTier {
  FLC_TIER_MEASURED = 0,   //: hardware produced numbers
  FLC_TIER_CANDIDATE,      //: internally consistent, derivation succeeded
  FLC_TIER_ACCEPTED,       //: passed the post-measure acceptance gates
  FLC_TIER_PROMOTED,       //: written into canonical calibration — NEVER here
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

struct FlcAcceptanceGates {
  bool repeatabilityToleranceKnown;
  uint16_t repeatabilityToleranceTicks;
  bool endpointVsUrdfToleranceKnown;
  uint16_t endpointVsUrdfToleranceTicks;
  //: expected span between the two geometric contacts, in ticks
  int expectedSpanTicks;
};

struct FlcJointCalibrationResult {
  int status;
  int abortReason;
  uint8_t busId;
  int8_t direction;
  FlcEndpointResult minEndpoint;
  FlcReturnResult returnToNeutral;
  FlcEndpointResult maxEndpoint;
  int minContactTick;
  int maxContactTick;
  int measuredSpanTicks;
  int expectedSpanTicks;
  int spanErrorTicks;
  int q0Tick;
  int tier;
  bool accepted;
};

//: q = direction * signed_tick_delta(raw, q0) — so q0 sits `-qMin` from the min
//: contact along the probe direction. Solved in ticks to stay integer-exact.
inline int flcDeriveQ0Tick(int minContactTick, int8_t direction,
                           int minAngleTicksFromZero) {
  // minAngleTicksFromZero is negative for a min endpoint below zero.
  return flcNormalizeTick(minContactTick - direction * minAngleTicksFromZero);
}

inline FlcJointCalibrationResult flcCalibrateJoint(
    const FlcServoPort &port, uint8_t id, int startTick, int8_t measuredDirection,
    int minAngleTicksFromZero, int maxAngleTicksFromZero,
    const FlcMotionEnvelope &envelope, const FlcContactConfig &guards,
    const FlcBaseline &baseline, const FlcAcceptanceGates &gates) {
  FlcJointCalibrationResult result;
  result.status = FLC_ENGINE_OK;
  result.abortReason = FLC_ABORT_NONE;
  result.busId = id;
  result.direction = measuredDirection;
  result.minContactTick = -1;
  result.maxContactTick = -1;
  result.measuredSpanTicks = 0;
  result.expectedSpanTicks = gates.expectedSpanTicks;
  result.spanErrorTicks = 0;
  result.q0Tick = -1;
  result.tier = FLC_TIER_MEASURED;
  result.accepted = false;

  if (measuredDirection != 1 && measuredDirection != -1) {
    result.status = FLC_ENGINE_DIRECTION_UNRESOLVED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }
  if (!gates.repeatabilityToleranceKnown) {
    // Without a repeatability band there is no way to decide whether two
    // contacts agree, so the measurement itself is not meaningful.
    result.status = FLC_ENGINE_REPEATABILITY_FAILED;
    return result;
  }

  const int begun = flcBeginMotion(port, id, envelope);
  if (begun != FLC_ENGINE_OK) {
    result.status = begun;
    flcEndMotion(port, id);
    return result;
  }

  // Each endpoint is sized from ITS OWN expected distance from q0, so a joint
  // with a wide range does not inherit a budget meant for a narrow one.
  FlcMotionEnvelope minEnvelope = envelope;
  minEnvelope.travelBudgetTicks = flcTravelBudgetForDistance(minAngleTicksFromZero);
  minEnvelope = flcClampEnvelope(minEnvelope);

  FlcMotionEnvelope maxEnvelope = envelope;
  maxEnvelope.travelBudgetTicks = flcTravelBudgetForDistance(maxAngleTicksFromZero);
  maxEnvelope = flcClampEnvelope(maxEnvelope);

  // The min endpoint lies at negative q, i.e. against the -direction probe.
  const int8_t minProbeSign = (int8_t)-measuredDirection;
  result.minEndpoint = flcMeasureEndpoint(port, id, startTick, minProbeSign, minEnvelope,
                                          guards, baseline,
                                          gates.repeatabilityToleranceTicks);
  if (result.minEndpoint.status != FLC_ENGINE_OK) {
    result.status = result.minEndpoint.status;
    result.abortReason = result.minEndpoint.abortReason;
    flcEndMotion(port, id);
    return result;
  }
  result.minContactTick = result.minEndpoint.contactTick;

  // Return to neutral before the opposite endpoint. Approaching MAX straight
  // from the MIN rest would make one traversal as long as the whole joint span.
  result.returnToNeutral = flcReturnTo(port, id, startTick, minEnvelope, guards);
  if (result.returnToNeutral.status != FLC_ENGINE_OK) {
    result.status = result.returnToNeutral.status;
    result.abortReason = result.returnToNeutral.abortReason;
    flcEndMotion(port, id);
    return result;
  }

  const int8_t maxProbeSign = measuredDirection;
  result.maxEndpoint = flcMeasureEndpoint(port, id, result.returnToNeutral.achievedTick,
                                          maxProbeSign, maxEnvelope, guards, baseline,
                                          gates.repeatabilityToleranceTicks);
  if (result.maxEndpoint.status != FLC_ENGINE_OK) {
    result.status = result.maxEndpoint.status;
    result.abortReason = result.maxEndpoint.abortReason;
    flcEndMotion(port, id);
    return result;
  }
  result.maxContactTick = result.maxEndpoint.contactTick;

  // Every calibration path ends torque OFF, success or not.
  if (!flcEndMotion(port, id)) {
    result.status = FLC_ENGINE_TORQUE_SETUP_FAILED;
    return result;
  }

  const int span = flcSignedTickDelta(result.maxContactTick, result.minContactTick);
  const int signedSpan = span * measuredDirection;
  if (signedSpan <= 0) {
    // MAX did not end up beyond MIN along the measured direction.
    result.status = FLC_ENGINE_ABORTED;
    result.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return result;
  }
  result.measuredSpanTicks = signedSpan;
  result.spanErrorTicks = signedSpan - gates.expectedSpanTicks;

  result.q0Tick = flcDeriveQ0Tick(result.minContactTick, measuredDirection,
                                  minAngleTicksFromZero);
  result.tier = FLC_TIER_CANDIDATE;

  // Post-measure acceptance. An unknown tolerance cannot certify agreement, so
  // the result stays CANDIDATE rather than being forced to ACCEPTED.
  if (gates.endpointVsUrdfToleranceKnown) {
    const int error = result.spanErrorTicks < 0 ? -result.spanErrorTicks : result.spanErrorTicks;
    if (error <= (int)gates.endpointVsUrdfToleranceTicks) {
      result.tier = FLC_TIER_ACCEPTED;
      result.accepted = true;
    }
  }
  (void)maxAngleTicksFromZero;
  return result;
}

// --------------------------------------------------------------------------
// H5 / H6 — leg and four-leg orchestration
//
// ONE generic calibrator + per-joint plans. There is deliberately no
// LfStateMachine / RfStateMachine / RhStateMachine / LhStateMachine: a leg is
// just an ordered list of FlcJointPlan rows, and four legs is an ordered list
// of legs. Both live here rather than in the .ino so the offline suite drives
// the same orchestration the firmware runs.
// --------------------------------------------------------------------------

#define FLC_JOINTS_PER_LEG 3
#define FLC_LEG_COUNT 4

struct FlcJointPlan {
  uint8_t busId;
  bool characterized;
  int8_t direction;
  int startTick;
  int minAngleTicksFromZero;
  int maxAngleTicksFromZero;
  FlcBaseline baseline;
  FlcMotionEnvelope envelope;
  FlcAcceptanceGates gates;
};

struct FlcLegResult {
  int status;
  int jointCount;
  int jointsOk;
  int failedBusId;
  FlcJointCalibrationResult joints[FLC_JOINTS_PER_LEG];
};

//: Calibrate the joints of one leg in the given order, stopping at the first
//: failure. Ordering is the caller's: the firmware runs distal-first so the
//: limb stays folded while proximal joints are still uncalibrated.
inline FlcLegResult flcCalibrateLeg(const FlcServoPort &port,
                                    const FlcContactConfig &guards,
                                    const FlcJointPlan *plans, int count) {
  FlcLegResult result;
  result.status = FLC_ENGINE_OK;
  result.jointCount = count;
  result.jointsOk = 0;
  result.failedBusId = 0;

  if (count > FLC_JOINTS_PER_LEG) count = FLC_JOINTS_PER_LEG;

  for (int i = 0; i < count; ++i) {
    const FlcJointPlan &plan = plans[i];
    if (!plan.characterized) {
      result.status = FLC_ENGINE_PREREQUISITE_DRIFT;
      result.failedBusId = plan.busId;
      break;
    }
    result.joints[i] = flcCalibrateJoint(
        port, plan.busId, plan.startTick, plan.direction, plan.minAngleTicksFromZero,
        plan.maxAngleTicksFromZero, plan.envelope, guards, plan.baseline, plan.gates);

    if (result.joints[i].status != FLC_ENGINE_OK) {
      result.status = result.joints[i].status;
      result.failedBusId = plan.busId;
      // Stop the leg: continuing would move a limb whose neighbouring joint is
      // in an unverified state.
      break;
    }
    ++result.jointsOk;
  }

  // Whole-leg release, whatever happened.
  for (int i = 0; i < count; ++i) flcEndMotion(port, plans[i].busId);
  return result;
}

struct FlcAllLegsResult {
  int status;
  int legsOk;
  int jointsOk;
  int failedLegIndex;
  FlcLegResult legs[FLC_LEG_COUNT];
};

//: Calibrate four legs in the supplied order. `plans` is a flat array of
//: legCount * FLC_JOINTS_PER_LEG rows. One session result, no promotion.
inline FlcAllLegsResult flcCalibrateAllLegs(const FlcServoPort &port,
                                            const FlcContactConfig &guards,
                                            const FlcJointPlan *plans, int legCount) {
  FlcAllLegsResult result;
  result.status = FLC_ENGINE_OK;
  result.legsOk = 0;
  result.jointsOk = 0;
  result.failedLegIndex = -1;

  if (legCount > FLC_LEG_COUNT) legCount = FLC_LEG_COUNT;

  for (int leg = 0; leg < legCount; ++leg) {
    result.legs[leg] =
        flcCalibrateLeg(port, guards, plans + leg * FLC_JOINTS_PER_LEG,
                        FLC_JOINTS_PER_LEG);
    result.jointsOk += result.legs[leg].jointsOk;

    if (result.legs[leg].status != FLC_ENGINE_OK) {
      result.status = result.legs[leg].status;
      result.failedLegIndex = leg;
      break;
    }
    ++result.legsOk;
  }

  // Global release across every joint mentioned in the plan.
  for (int i = 0; i < legCount * FLC_JOINTS_PER_LEG; ++i) {
    flcEndMotion(port, plans[i].busId);
  }
  return result;
}

#endif  // FLC_CALIBRATION_ENGINE_H
