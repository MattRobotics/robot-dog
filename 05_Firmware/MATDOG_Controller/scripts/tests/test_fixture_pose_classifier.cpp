#include "motion/FixturePoseClassifier.h"
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
using namespace matdog::motion;
static constexpr double d = 3.14159265358979323846 / 180.0;
static std::array<double, 12> toRad(std::array<double, 12> x) {
  for (auto& a : x) a *= d;
  return x;
}
int main() {
  // Frozen route fixture first knot and rounded real encoder-derived 2026-10-13.
  const auto nominal = toRad({-1.0,83.74,-88.5,1.0,83.74,-88.5,1.0,90.0,-89.0,-1.0,90.0,-89.0});
  const auto actual = toRad({-2.29,80.16,-91.76,2.37,79.54,-90.62,-2.55,82.53,-91.41,2.29,82.35,-91.41});
  const auto r = classifyFixtureProne(actual, nominal, true, true);
  assert(r.kind == FixturePoseKind::PRONE_COMPATIBLE);
  assert(r.maxErrorJointIndex == 10); // LH UPPER
  assert(std::abs(r.maxNominalErrorRad / d - 7.65) < 0.01);
  assert(r.maxNominalErrorRad > 0.006); // NOT route-aligned
  assert(classifyFixtureProne(actual, nominal, false, true).kind == FixturePoseKind::UNKNOWN);
  assert(classifyFixtureProne(actual, nominal, true, false).kind == FixturePoseKind::UNKNOWN);
  auto corrupted = actual; corrupted[4] += 20*d;
  assert(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  corrupted = actual; corrupted[7] = -actual[7];
  assert(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  corrupted = actual; corrupted[9] = std::numeric_limits<double>::quiet_NaN();
  assert(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  corrupted = nominal; for(auto& x : corrupted) x = 0;
  assert(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  assert(classifyFixtureProne(actual, nominal, true, true, 0.006, 0.006).kind == FixturePoseKind::UNKNOWN);
  std::cout << "FIXTURE_PRONE_CLASSIFIER_PASS checks=9 max_delta_deg="
            << r.maxNominalErrorRad / d << " rms_delta_deg=" << r.rmsNominalErrorRad / d << '\n';
}
