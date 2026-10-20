#ifndef MATDOG_MOTION_FIXTURE_POSE_CLASSIFIER_H
#define MATDOG_MOTION_FIXTURE_POSE_CLASSIFIER_H

// Commissioning pose *recognition only*. This function never creates targets,
// grants motor authority, bypasses joint limits or validates a motion corridor.
// Order: [LF(H,U,L), RF(H,U,L), RH(H,U,L), LH(H,U,L)]. SI radians.
#include <cmath>
#include <stddef.h>

namespace matdog { namespace motion {

enum class FixturePoseKind : unsigned char {
  UNKNOWN,
  PRONE_COMPATIBLE,
};

// Why a pose stayed UNKNOWN, so the operator gets a diagnosis, not a badge.
enum class FixturePoseReason : unsigned char {
  OK, NOT_FIXTURE, NOT_STATIONARY, INVALID_THRESHOLDS, NONFINITE,
  HIP_BAND, UPPER_BAND, LOWER_BAND, MAX_DEVIATION, RMS_DEVIATION,
};
inline const char* fixturePoseReasonName(FixturePoseReason r) {
  static const char* names[] = {"OK", "NOT_FIXTURE", "NOT_STATIONARY", "INVALID_THRESHOLDS",
    "NONFINITE", "HIP_BAND", "UPPER_BAND", "LOWER_BAND", "MAX_DEVIATION", "RMS_DEVIATION"};
  const unsigned i = static_cast<unsigned>(r);
  return i < sizeof(names) / sizeof(names[0]) ? names[i] : "UNKNOWN";
}

inline FixturePoseReason fixtureProneJointReason(unsigned joint,double a) {
  if(!std::isfinite(a))return FixturePoseReason::NONFINITE;
  switch(joint%3){
    case 0: if(std::abs(a)>.35)return FixturePoseReason::HIP_BAND;break;
    case 1: if(a<.87||a>1.92)return FixturePoseReason::UPPER_BAND;break;
    case 2: if(a> -1.22||a< -2.0)return FixturePoseReason::LOWER_BAND;break;
  }
  return FixturePoseReason::OK;
}

struct FixturePoseClassification {
  FixturePoseKind kind = FixturePoseKind::UNKNOWN;
  double maxNominalErrorRad = 0.0;
  double rmsNominalErrorRad = 0.0;
  unsigned maxErrorJointIndex = 0;
  FixturePoseReason reason = FixturePoseReason::INVALID_THRESHOLDS;
  unsigned failedJointIndex = 12;  // 12 = no single joint
};

// The *nominal* vector must be extracted from the frozen fixture route at its
// first knot, never inferred from EEPROM or from a previous tracking target.
// The 10-degree recognition envelope is a configurable INITIAL heuristic,
// NOT motion admission and NOT the reviewed 6-mrad tracking tube.
// actual and nominal each point to 12 values in the order above.
inline FixturePoseClassification classifyFixtureProne(
    const double* actual,
    const double* nominal,
    bool fixtureConfirmed,
    bool stationary,
    double maxNominalDeviationRad = 0.17453292519943295,
    double maxRmsDeviationRad = 0.12217304763960307) {
  FixturePoseClassification out;
  if (actual == nullptr || nominal == nullptr) return out;
  if (!std::isfinite(maxNominalDeviationRad) || maxNominalDeviationRad <= 0.0 ||
      !std::isfinite(maxRmsDeviationRad) || maxRmsDeviationRad <= 0.0 ||
      maxRmsDeviationRad > maxNominalDeviationRad) return out;
  out.reason = !fixtureConfirmed ? FixturePoseReason::NOT_FIXTURE : FixturePoseReason::NOT_STATIONARY;
  if (!fixtureConfirmed || !stationary) return out;

  double squaredSum = 0.0;
  for (size_t j = 0; j < 12; ++j) {
    const double a = actual[j], n = nominal[j];
    out.failedJointIndex = static_cast<unsigned>(j);
    out.reason = FixturePoseReason::NONFINITE;
    if (!std::isfinite(a) || !std::isfinite(n)) return out;
    const double deviation = std::abs(a - n);
    if (deviation > out.maxNominalErrorRad) {
      out.maxNominalErrorRad = deviation;
      out.maxErrorJointIndex = static_cast<unsigned>(j);
    }
    squaredSum += deviation * deviation;
    // A folded leg must retain the physically consistent signs. Values are
    // broad recognizer checks, never a replacement for collision clearance.
    out.reason=fixtureProneJointReason(static_cast<unsigned>(j),a);
    if(out.reason!=FixturePoseReason::OK)return out;
  }
  out.rmsNominalErrorRad = std::sqrt(squaredSum / 12.0);
  out.failedJointIndex = out.maxErrorJointIndex;
  if (out.maxNominalErrorRad > maxNominalDeviationRad) { out.reason = FixturePoseReason::MAX_DEVIATION; return out; }
  out.failedJointIndex = 12;
  if (out.rmsNominalErrorRad > maxRmsDeviationRad) { out.reason = FixturePoseReason::RMS_DEVIATION; return out; }
  out.reason = FixturePoseReason::OK;
  out.kind = FixturePoseKind::PRONE_COMPATIBLE;
  return out;
}

}} // namespace matdog::motion
#endif
