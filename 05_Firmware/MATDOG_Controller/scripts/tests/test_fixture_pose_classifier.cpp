#include "motion/FixturePoseClassifier.h"
#include <array>
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
  unsigned checks = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { std::cerr << "FAIL line " << __LINE__ << ": " << #expr << "\n"; return 1; } } while (0)
  // Frozen route fixture first knot and rounded real encoder-derived 2026-10-13.
  const auto nominal = toRad({-1.0,83.74,-88.5,1.0,83.74,-88.5,1.0,90.0,-89.0,-1.0,90.0,-89.0});
  const auto actual = toRad({-2.29,80.16,-91.76,2.37,79.54,-90.62,-2.55,82.53,-91.41,2.29,82.35,-91.41});
  const auto r = classifyFixtureProne(actual, nominal, true, true);
  CHECK(r.kind == FixturePoseKind::PRONE_COMPATIBLE);
  CHECK(r.maxErrorJointIndex == 10); // LH UPPER
  CHECK(std::abs(r.maxNominalErrorRad / d - 7.65) < 0.01);
  CHECK(r.maxNominalErrorRad > 0.006); // NOT route-aligned
  CHECK(classifyFixtureProne(actual, nominal, false, true).kind == FixturePoseKind::UNKNOWN);
  CHECK(classifyFixtureProne(actual, nominal, true, false).kind == FixturePoseKind::UNKNOWN);
  auto corrupted = actual; corrupted[4] += 20*d;
  CHECK(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  corrupted = actual; corrupted[7] = -actual[7];
  CHECK(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  corrupted = actual; corrupted[9] = std::numeric_limits<double>::quiet_NaN();
  CHECK(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  corrupted = nominal; for(auto& x : corrupted) x = 0;
  CHECK(classifyFixtureProne(corrupted, nominal, true, true).kind == FixturePoseKind::UNKNOWN);
  CHECK(classifyFixtureProne(actual, nominal, true, true, 0.006, 0.006).kind == FixturePoseKind::UNKNOWN);
  std::cout << "FIXTURE_PRONE_CLASSIFIER_PASS checks=" << checks << " max_delta_deg="
            << r.maxNominalErrorRad / d << " rms_delta_deg=" << r.rmsNominalErrorRad / d << '\n';
}
