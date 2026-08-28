/*
 * MATDOG FULL LEG CALIBRATOR V1 — bounded contact/endpoint detection engine
 *
 * PURE C++. No Arduino, no SCServo, no I/O, no globals, no time source.
 * Every input arrives as an explicit observation. This is deliberate: the exact
 * same translation unit is compiled twice —
 *
 *   1. into the ESP32-S3 firmware (matdog_full_leg_calibrator_v1.ino), and
 *   2. into a host test harness (tests/flc_detector_harness.cpp)
 *
 * so the state machine that would drive real servos is the state machine the
 * offline fault-injection suite exercises. There is no second implementation to
 * drift out of sync.
 *
 * ALGORITHMIC PROVENANCE — LF V25 (historical oracle, previous installation):
 *   09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/
 *     source/software/drivers/st3215/src/auto_calibrate/matdog.rs
 *   Reused as CONCEPTS ONLY:
 *     - hybrid contact evidence: low velocity AND low progress AND enough travel
 *       AND the commanded target still ahead — never a single channel;
 *     - persistence over consecutive samples rather than one sample;
 *     - a startup grace window after each new commanded target;
 *     - median/MAD free-motion current baseline instead of a fixed threshold;
 *     - bounded retreat then re-approach for repeatability;
 *     - hard abort on driver error / status / torque-state / goal mismatch.
 *   Reused as NUMBERS: nothing. Every LF V25 numeric threshold describes an
 *   installation that no longer exists. The caller must supply a config whose
 *   values are justified for the CURRENT build.
 */

#ifndef FLC_CONTACT_DETECTOR_H
#define FLC_CONTACT_DETECTOR_H

#include <stdint.h>
#include <stdlib.h>

// --------------------------------------------------------------------------
// Encoder domain — unsigned 0..4095 everywhere. No signed wrap trick.
// --------------------------------------------------------------------------

static const int FLC_ENCODER_MODULUS = 4096;
static const int FLC_ENCODER_MAX = 4095;
static const int FLC_HALF_ENCODER_RANGE = 2048;

// Euclidean modulo. C++ '%' truncates toward zero, so a bare '%' on a negative
// numerator yields a negative residue and breaks every wrap-boundary result.
// Python's '%' is already Euclidean, which is why matdog_joint_math.py can use
// the operator directly and this header cannot.
inline int flcMod(int value, int modulus) {
  int r = value % modulus;
  return (r < 0) ? r + modulus : r;
}

inline int flcNormalizeTick(int tick) { return flcMod(tick, FLC_ENCODER_MODULUS); }

// Mirrors matdog_joint_math.signed_tick_delta(). Result in [-2048, 2047].
inline int flcSignedTickDelta(int presentTick, int referenceTick) {
  int present = flcNormalizeTick(presentTick);
  int reference = flcNormalizeTick(referenceTick);
  return flcMod(present - reference + FLC_HALF_ENCODER_RANGE, FLC_ENCODER_MODULUS) -
         FLC_HALF_ENCODER_RANGE;
}

inline int flcDirectionalProgress(int present, int reference, int probeSign) {
  int delta = flcSignedTickDelta(present, reference) * probeSign;
  return delta > 0 ? delta : 0;
}

inline int flcCircularDistance(int a, int b) {
  int d = flcSignedTickDelta(a, b);
  return d < 0 ? -d : d;
}

// A joint travelling from `start` by `ticks` in `probeSign` must stay strictly
// inside the unsigned domain. Crossing 0 or 4095 is refused rather than wrapped:
// the MATDOG leg joints all live far from the boundary, so a planned crossing
// means the inputs are wrong, not that a wrap is needed.
inline bool flcPlanStaysInDomain(int start, int ticks, int probeSign) {
  if (start < 0 || start > FLC_ENCODER_MAX) return false;
  if (ticks < 0) return false;
  long target = (long)start + (long)probeSign * (long)ticks;
  return target >= 0 && target <= FLC_ENCODER_MAX;
}

// --------------------------------------------------------------------------
// Circular tick statistics — used by the manual-q0 capture
//
// Accumulated as SIGNED DELTAS from the first sample, never as raw ticks. A
// joint resting near the 4095/0 boundary would otherwise give a linear mean of
// ~2047 and a linear spread of ~4095: a nonsense centre and a false
// instability. Mirrors matdog_joint_math.circular_tick_summary().
// --------------------------------------------------------------------------

struct FlcTickSummary {
  bool valid;
  int count;
  int centreTick;
  int minTick;
  int maxTick;
  int spreadTicks;
};

inline FlcTickSummary flcSummarizeTicks(const int *ticks, int count) {
  FlcTickSummary summary = {false, 0, 0, 0, 0, 0};
  if (ticks == 0 || count <= 0) return summary;

  int reference = flcNormalizeTick(ticks[0]);
  long deltaSum = 0;
  int minDelta = 0;
  int maxDelta = 0;

  for (int i = 0; i < count; ++i) {
    int delta = flcSignedTickDelta(ticks[i], reference);
    if (i == 0 || delta < minDelta) minDelta = delta;
    if (i == 0 || delta > maxDelta) maxDelta = delta;
    deltaSum += delta;
  }

  summary.valid = true;
  summary.count = count;
  summary.centreTick = flcNormalizeTick(reference + (int)(deltaSum / count));
  summary.minTick = flcNormalizeTick(reference + minDelta);
  summary.maxTick = flcNormalizeTick(reference + maxDelta);
  summary.spreadTicks = maxDelta - minDelta;
  return summary;
}

// --------------------------------------------------------------------------
// Structured outcomes — a refusal always carries a machine-readable reason.
// --------------------------------------------------------------------------

enum FlcContactState {
  FLC_FREE_MOTION = 0,
  FLC_CONFIRMING,
  FLC_CONTACT_CONFIRMED,
  FLC_HARD_ABORT,
};

enum FlcAbortReason {
  FLC_ABORT_NONE = 0,
  FLC_ABORT_TELEMETRY_TIMEOUT,
  FLC_ABORT_DRIVER_ERROR,
  FLC_ABORT_STATUS_ERROR,
  FLC_ABORT_TORQUE_UNEXPECTEDLY_OFF,
  FLC_ABORT_TORQUE_LIMIT_MISMATCH,
  FLC_ABORT_GOAL_MISMATCH,
  FLC_ABORT_OVERCURRENT,
  FLC_ABORT_THERMAL,
  FLC_ABORT_VOLTAGE,
  FLC_ABORT_TRAVEL_BUDGET_EXCEEDED,
  FLC_ABORT_TIME_BUDGET_EXCEEDED,
  FLC_ABORT_POSITION_OUT_OF_DOMAIN,
  FLC_ABORT_WRAP_BOUNDARY,
  FLC_ABORT_CONTACT_TOO_EARLY,
  FLC_ABORT_TARGET_DID_NOT_MOVE,
  FLC_ABORT_WRONG_DIRECTION,
};

inline const char *flcAbortReasonLabel(int reason) {
  switch (reason) {
    case FLC_ABORT_NONE: return "NONE";
    case FLC_ABORT_TELEMETRY_TIMEOUT: return "TELEMETRY_TIMEOUT";
    case FLC_ABORT_DRIVER_ERROR: return "DRIVER_ERROR";
    case FLC_ABORT_STATUS_ERROR: return "STATUS_ERROR";
    case FLC_ABORT_TORQUE_UNEXPECTEDLY_OFF: return "TORQUE_UNEXPECTEDLY_OFF";
    case FLC_ABORT_TORQUE_LIMIT_MISMATCH: return "TORQUE_LIMIT_MISMATCH";
    case FLC_ABORT_GOAL_MISMATCH: return "GOAL_MISMATCH";
    case FLC_ABORT_OVERCURRENT: return "OVERCURRENT";
    case FLC_ABORT_THERMAL: return "THERMAL";
    case FLC_ABORT_VOLTAGE: return "VOLTAGE";
    case FLC_ABORT_TRAVEL_BUDGET_EXCEEDED: return "TRAVEL_BUDGET_EXCEEDED";
    case FLC_ABORT_TIME_BUDGET_EXCEEDED: return "TIME_BUDGET_EXCEEDED";
    case FLC_ABORT_POSITION_OUT_OF_DOMAIN: return "POSITION_OUT_OF_DOMAIN";
    case FLC_ABORT_WRAP_BOUNDARY: return "WRAP_BOUNDARY";
    case FLC_ABORT_CONTACT_TOO_EARLY: return "CONTACT_TOO_EARLY";
    case FLC_ABORT_TARGET_DID_NOT_MOVE: return "TARGET_DID_NOT_MOVE";
    case FLC_ABORT_WRONG_DIRECTION: return "WRONG_DIRECTION";
    default: return "UNKNOWN";
  }
}

// --------------------------------------------------------------------------
// Configuration — supplied by the caller, never defaulted to a historical value
// --------------------------------------------------------------------------

struct FlcContactConfig {
  // Contact evidence
  uint16_t maxProgressTicks;        // per-sample advance that still counts as stalled
  uint16_t maxVelocityRaw;          // PresentSpeed magnitude that counts as stopped
  uint16_t targetReachedToleranceTicks;
  uint16_t minTravelTicks;          // guards "contact" declared before moving
  uint8_t persistenceSamples;       // consecutive confirmations required
  uint8_t startupGraceSamples;      // ignored samples after a new commanded target

  // Hard guards
  uint16_t hardCurrentAbortRaw;
  uint16_t expectedTorqueLimit;
  int thermalLimitC;
  int voltageMin;
  int voltageMax;

  // Budgets
  uint16_t travelBudgetTicks;
  uint32_t timeBudgetMs;

  // Direction/consistency
  int8_t probeSign;                 // +1 or -1; 0 is invalid and refuses
};

// A free-motion current baseline, so the contact threshold adapts to the joint
// instead of trusting a fixed number carried over from another installation.
struct FlcBaseline {
  uint16_t medianCurrent;
  uint16_t madCurrent;
  bool valid;
};

inline uint16_t flcBaselineContactThreshold(const FlcBaseline &b) {
  uint32_t margin = (uint32_t)b.madCurrent * 4u;
  if (margin < 5u) margin = 5u;
  uint32_t t = (uint32_t)b.medianCurrent + margin;
  return (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
}

struct FlcObservation {
  bool telemetryValid;
  bool driverError;
  uint8_t statusByte;
  bool torqueEnabled;
  uint16_t torqueLimit;
  uint16_t goalPosition;
  uint16_t position;
  int16_t velocity;
  uint16_t current;
  uint8_t temperature;
  uint8_t voltage;
  uint32_t elapsedMs;
};

// --------------------------------------------------------------------------
// The detector
// --------------------------------------------------------------------------

struct FlcContactDetector {
  FlcContactConfig config;
  FlcBaseline baseline;
  int startPosition;
  int previousPosition;
  int activeTarget;
  bool hasActiveTarget;
  uint8_t targetSamplesSeen;
  uint8_t confirmingSamples;
  int abortReason;
  int contactTick;
  bool currentSupportedContact;
  uint16_t peakCurrent;
  uint8_t peakTemperature;
};

inline void flcDetectorInit(FlcContactDetector &d, const FlcContactConfig &config,
                            const FlcBaseline &baseline, int startPosition) {
  d.config = config;
  d.baseline = baseline;
  d.startPosition = startPosition;
  d.previousPosition = startPosition;
  d.activeTarget = 0;
  d.hasActiveTarget = false;
  d.targetSamplesSeen = 0;
  d.confirmingSamples = 0;
  d.abortReason = FLC_ABORT_NONE;
  d.contactTick = -1;
  d.currentSupportedContact = false;
  d.peakCurrent = 0;
  d.peakTemperature = 0;
}

inline int flcDetectorObserve(FlcContactDetector &d, const FlcObservation &o,
                              int commandedTarget) {
  // --- Channel 0: is the observation itself usable? ---
  if (!o.telemetryValid) {
    d.abortReason = FLC_ABORT_TELEMETRY_TIMEOUT;
    return FLC_HARD_ABORT;
  }
  if (o.driverError) {
    d.abortReason = FLC_ABORT_DRIVER_ERROR;
    return FLC_HARD_ABORT;
  }
  if (o.position > FLC_ENCODER_MAX) {
    d.abortReason = FLC_ABORT_POSITION_OUT_OF_DOMAIN;
    return FLC_HARD_ABORT;
  }
  if (commandedTarget < 0 || commandedTarget > FLC_ENCODER_MAX) {
    d.abortReason = FLC_ABORT_POSITION_OUT_OF_DOMAIN;
    return FLC_HARD_ABORT;
  }
  if (d.config.probeSign != 1 && d.config.probeSign != -1) {
    d.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return FLC_HARD_ABORT;
  }

  if (o.current > d.peakCurrent) d.peakCurrent = o.current;
  if (o.temperature > d.peakTemperature) d.peakTemperature = o.temperature;

  // --- Channel 1: hard servo/state guards, checked before any contact logic ---
  if (o.statusByte != 0) {
    d.abortReason = FLC_ABORT_STATUS_ERROR;
    return FLC_HARD_ABORT;
  }
  if (!o.torqueEnabled) {
    d.abortReason = FLC_ABORT_TORQUE_UNEXPECTEDLY_OFF;
    return FLC_HARD_ABORT;
  }
  if (o.torqueLimit != d.config.expectedTorqueLimit) {
    d.abortReason = FLC_ABORT_TORQUE_LIMIT_MISMATCH;
    return FLC_HARD_ABORT;
  }
  if (o.goalPosition != (uint16_t)commandedTarget) {
    d.abortReason = FLC_ABORT_GOAL_MISMATCH;
    return FLC_HARD_ABORT;
  }
  if (o.current >= d.config.hardCurrentAbortRaw) {
    d.abortReason = FLC_ABORT_OVERCURRENT;
    return FLC_HARD_ABORT;
  }
  if ((int)o.temperature >= d.config.thermalLimitC) {
    d.abortReason = FLC_ABORT_THERMAL;
    return FLC_HARD_ABORT;
  }
  if ((int)o.voltage < d.config.voltageMin || (int)o.voltage > d.config.voltageMax) {
    d.abortReason = FLC_ABORT_VOLTAGE;
    return FLC_HARD_ABORT;
  }

  // --- Channel 2: budgets ---
  int travel = flcDirectionalProgress(o.position, d.startPosition, d.config.probeSign);
  if (travel > (int)d.config.travelBudgetTicks) {
    d.abortReason = FLC_ABORT_TRAVEL_BUDGET_EXCEEDED;
    return FLC_HARD_ABORT;
  }
  if (o.elapsedMs > d.config.timeBudgetMs) {
    d.abortReason = FLC_ABORT_TIME_BUDGET_EXCEEDED;
    return FLC_HARD_ABORT;
  }
  // Motion away from the probe direction beyond the settle tolerance means the
  // commanded sign and the physical joint disagree.
  int reverse = flcDirectionalProgress(o.position, d.startPosition, (int8_t)-d.config.probeSign);
  if (reverse > (int)d.config.targetReachedToleranceTicks) {
    d.abortReason = FLC_ABORT_WRONG_DIRECTION;
    return FLC_HARD_ABORT;
  }

  // --- Target bookkeeping: a new commanded target restarts the grace window ---
  if (!d.hasActiveTarget || d.activeTarget != commandedTarget) {
    d.activeTarget = commandedTarget;
    d.hasActiveTarget = true;
    d.targetSamplesSeen = 0;
    d.previousPosition = o.position;
    d.confirmingSamples = 0;
    return FLC_FREE_MOTION;
  }
  if (d.targetSamplesSeen < 255) d.targetSamplesSeen++;

  int progress = flcDirectionalProgress(o.position, d.previousPosition, d.config.probeSign);
  d.previousPosition = o.position;

  int speedMagnitude = o.velocity < 0 ? -(int)o.velocity : (int)o.velocity;
  bool lowVelocity = speedMagnitude <= (int)d.config.maxVelocityRaw;
  bool lowProgress = progress <= (int)d.config.maxProgressTicks;
  bool enoughTravel = travel >= (int)d.config.minTravelTicks;
  int goalError = flcCircularDistance(o.position, commandedTarget);
  bool targetAhead =
      flcSignedTickDelta(commandedTarget, o.position) * d.config.probeSign > 0;

  // The servo reached where it was told to go: that is not an endstop.
  if (goalError <= (int)d.config.targetReachedToleranceTicks) {
    d.confirmingSamples = 0;
    return FLC_FREE_MOTION;
  }
  // Grace window: a fresh target has not had time to produce motion yet.
  if (d.targetSamplesSeen <= d.config.startupGraceSamples) {
    d.confirmingSamples = 0;
    return FLC_FREE_MOTION;
  }
  // Still accelerating or still travelling.
  if (!lowVelocity || !lowProgress || !targetAhead) {
    d.confirmingSamples = 0;
    return FLC_FREE_MOTION;
  }

  // Stalled short of the target. If it stalled before moving a credible
  // distance, that is an obstruction or a dead joint, not the endstop.
  if (!enoughTravel) {
    d.confirmingSamples = 0;
    if (d.targetSamplesSeen >= d.config.startupGraceSamples + d.config.persistenceSamples) {
      d.abortReason = (travel == 0) ? FLC_ABORT_TARGET_DID_NOT_MOVE
                                    : FLC_ABORT_CONTACT_TOO_EARLY;
      return FLC_HARD_ABORT;
    }
    return FLC_FREE_MOTION;
  }

  // Persistence: a single stalled sample is never contact.
  if (d.confirmingSamples < 255) d.confirmingSamples++;
  if (d.confirmingSamples < d.config.persistenceSamples) {
    return FLC_CONFIRMING;
  }

  // Current is corroborating evidence, recorded but not required on its own —
  // a mechanical endstop reached at low speed can sit below the threshold.
  if (d.baseline.valid && o.current >= flcBaselineContactThreshold(d.baseline)) {
    d.currentSupportedContact = true;
  }
  d.contactTick = o.position;
  return FLC_CONTACT_CONFIRMED;
}

// --------------------------------------------------------------------------
// Repeatability across two independent approaches to the same endpoint
// --------------------------------------------------------------------------

struct FlcRepeatability {
  int firstTick;
  int secondTick;
  int spreadTicks;
  int contactTick;      // circular midpoint of the two approaches
  bool accepted;
};

inline FlcRepeatability flcEvaluateRepeatability(int firstTick, int secondTick,
                                                 uint16_t toleranceTicks) {
  FlcRepeatability r;
  r.firstTick = firstTick;
  r.secondTick = secondTick;
  r.spreadTicks = flcCircularDistance(firstTick, secondTick);
  int half = flcSignedTickDelta(secondTick, firstTick) / 2;
  r.contactTick = flcNormalizeTick(firstTick + half);
  r.accepted = (r.spreadTicks <= (int)toleranceTicks);
  return r;
}

#endif  // FLC_CONTACT_DETECTOR_H
