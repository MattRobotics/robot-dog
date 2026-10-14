#ifndef MATDOG_MOTION_FIXTURE_POSE_CLASSIFIER_H
#define MATDOG_MOTION_FIXTURE_POSE_CLASSIFIER_H

// Commissioning pose *recognition only*. This function never creates targets,
// grants motor authority, bypasses joint limits or validates a motion corridor.
// Order: [LF(H,U,L), RF(H,U,L), RH(H,U,L), LH(H,U,L)]. SI radians.
#include <array>
#include <cmath>
#include <cstddef>

namespace matdog { namespace motion {

enum class FixturePoseKind : unsigned char {
  UNKNOWN,
  PRONE_COMPATIBLE,
};

struct FixturePoseClassification {
  FixturePoseKind kind = FixturePoseKind::UNKNOWN;
  double maxNominalErrorRad = 0.0;
  double rmsNominalErrorRad = 0.0;
  unsigned maxErrorJointIndex = 0;
};

// The *nominal* vector must be extracted from the frozen fixture route at its
// first knot, never inferred from EEPROM or from a previous tracking target.
// The 10-degree recognition envelope is a configurable INITIAL heuristic,
// NOT motion admission and NOT the reviewed 6-mrad tracking tube.
inline FixturePoseClassification classifyFixtureProne(
    const std::array<double, 12>& actual,
    const std::array<double, 12>& nominal,
    bool fixtureConfirmed,
    bool stationary,
    double maxNominalDeviationRad = 0.17453292519943295,
    double maxRmsDeviationRad = 0.12217304763960307) {
  FixturePoseClassification out;
  if (!fixtureConfirmed || !stationary ||
      !std::isfinite(maxNominalDeviationRad) || maxNominalDeviationRad <= 0.0 ||
      !std::isfinite(maxRmsDeviationRad) || maxRmsDeviationRad <= 0.0 ||
      maxRmsDeviationRad > maxNominalDeviationRad) return out;

  double squaredSum = 0.0;
  for (std::size_t j = 0; j < 12; ++j) {
    const double a = actual[j], n = nominal[j];
    if (!std::isfinite(a) || !std::isfinite(n)) return out;
    const double deviation = std::abs(a - n);
    if (deviation > out.maxNominalErrorRad) {
      out.maxNominalErrorRad = deviation;
      out.maxErrorJointIndex = static_cast<unsigned>(j);
    }
    squaredSum += deviation * deviation;
    // A folded leg must retain the physically consistent signs. Values are
    // broad recognizer checks, never a replacement for collision clearance.
    switch (j % 3) {
      case 0: if (std::abs(a) > 0.35) return out; break; // hip near neutral
      case 1: if (a < 0.87 || a > 1.92) return out; break; // upper down/folded
      case 2: if (a > -1.22 || a < -2.0) return out; break; // lower folded
    }
  }
  out.rmsNominalErrorRad = std::sqrt(squaredSum / 12.0);
  if (out.maxNominalErrorRad <= maxNominalDeviationRad &&
      out.rmsNominalErrorRad <= maxRmsDeviationRad)
    out.kind = FixturePoseKind::PRONE_COMPATIBLE;
  return out;
}

}} // namespace matdog::motion
#endif
