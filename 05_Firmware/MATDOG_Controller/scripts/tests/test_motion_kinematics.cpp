#include "../../src/motion/LegKinematics.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

using namespace matdog::motion;
static unsigned checks = 0, failures = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; \
  std::printf("FAIL line %d: %s\n", __LINE__, #condition); } } while (0)
static bool near(double a, double b, double tolerance = 1e-12) {
  return std::abs(a - b) <= tolerance;
}
static void roundTrip(LegId leg, LegJointAngles q) {
  const auto fk = forwardKinematics(leg, q);
  CHECK(fk.status == KinematicsStatus::OK);
  IkOptions options;
  options.seed = q;
  const auto ik = inverseKinematics(leg, fk.footOriginM, options);
  CHECK(ik.status == KinematicsStatus::OK);
  CHECK(ik.reachability == Reachability::REACHABLE);
  CHECK(ik.residualM <= options.toleranceM);
  CHECK(near(ik.joints.hip, q.hip, 1e-8));
  CHECK(near(ik.joints.upper, q.upper, 1e-8));
  CHECK(near(ik.joints.lower, q.lower, 1e-8));
  const auto repeated = inverseKinematics(leg, fk.footOriginM, options);
  CHECK(repeated.status == ik.status && repeated.solutions == ik.solutions);
  CHECK(repeated.joints.hip == ik.joints.hip && repeated.joints.upper == ik.joints.upper &&
        repeated.joints.lower == ik.joints.lower && repeated.residualM == ik.residualM);
}
int main() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (unsigned index = 0; index < 4; ++index) {
    const auto leg = static_cast<LegId>(index);
    const auto* model = legModel(leg);
    CHECK(model != nullptr);
    const auto zero = forwardKinematics(leg, {});
    // Independent canonical q=0 golden positions, including foot eccentricity.
    CHECK(near(zero.footOriginM.x, index < 2 ? 0.2195 : -0.0055));
    CHECK(near(zero.footOriginM.y, (index == 0 || index == 3) ? 0.094 : -0.094));
    CHECK(near(zero.footOriginM.z, index < 2 ? -0.0934 : -0.1134));
    for (unsigned r = 0; r < 3; ++r)
      for (unsigned c = 0; c < 3; ++c)
        CHECK(near(zero.baseFromFootRotation[r][c], r == c ? 1.0 : 0.0));
    roundTrip(leg, {});
    // All 27 corners/midpoints of the URDF limit box, including exact limits.
    for (unsigned h = 0; h < 3; ++h)
      for (unsigned u = 0; u < 3; ++u)
        for (unsigned l = 0; l < 3; ++l) {
          double q[3]; const unsigned selection[3] = {h, u, l};
          for (unsigned j = 0; j < 3; ++j)
            q[j] = selection[j] == 0 ? model->limits[j].lower :
                   (selection[j] == 2 ? model->limits[j].upper :
                    0.5 * (model->limits[j].lower + model->limits[j].upper));
          roundTrip(leg, {q[0], q[1], q[2]});
        }
    // Hip Jacobian singularity: sagittal z=0, both hip branches coalesce.
    // Derive the upper angle from the exact model for a lower angle of 0.5 rad.
    const double singularUpper = std::atan2(
        model->lowerOrigin.z - model->footOrigin.x * std::sin(0.5) +
        model->footOrigin.z * std::cos(0.5),
        model->footOrigin.x * std::cos(0.5) + model->footOrigin.z * std::sin(0.5)) + std::acos(-1.0);
    const auto singularFk = forwardKinematics(leg, {0.0, singularUpper, 0.5});
    CHECK(singularFk.status == KinematicsStatus::OK);
    const auto singularIk = inverseKinematics(leg, singularFk.footOriginM);
    CHECK(singularIk.status == KinematicsStatus::OK);
    CHECK(singularIk.residualM <= 1e-9);
    for (double delta : {-1e-7, 1e-7}) {
      const auto fk = forwardKinematics(leg, {0.1, singularUpper + delta, 0.5});
      CHECK(inverseKinematics(leg, fk.footOriginM).status == KinematicsStatus::OK);
    }
    auto unreachable = inverseKinematics(leg, {10.0, 0.0, 0.0});
    CHECK(unreachable.status == KinematicsStatus::UNREACHABLE_GEOMETRY);
    CHECK(unreachable.reachability == Reachability::UNREACHABLE);
    CHECK(inverseKinematics(leg, model->hipOrigin).status == KinematicsStatus::UNREACHABLE_GEOMETRY);
    CHECK(inverseKinematics(leg, {1e308, -1e308, 1e308}).status == KinematicsStatus::UNREACHABLE_GEOMETRY);
    // Fully extended planar chain exists geometrically, but requires lower=phi
    // outside this URDF's upper bound (including either equivalent branch).
    const double a = -model->lowerOrigin.z;
    const double b = std::hypot(model->footOrigin.x, model->footOrigin.z);
    const Vector3 extension = {model->hipOrigin.x,
        model->hipOrigin.y + model->upperOrigin.y + model->footOrigin.y,
        model->hipOrigin.z - a - b};
    CHECK(inverseKinematics(leg, extension).status == KinematicsStatus::UNREACHABLE_LIMITS);
    CHECK(forwardKinematics(leg, {model->limits[0].upper + 1e-10, 0, 0}).status == KinematicsStatus::JOINT_LIMIT);
    for (double bad : {nan, inf, -inf}) {
      CHECK(forwardKinematics(leg, {bad, 0, 0}).status == KinematicsStatus::INVALID_INPUT);
      CHECK(forwardKinematics(leg, {0, bad, 0}).status == KinematicsStatus::INVALID_INPUT);
      CHECK(forwardKinematics(leg, {0, 0, bad}).status == KinematicsStatus::INVALID_INPUT);
      CHECK(inverseKinematics(leg, {bad, 0, 0}).status == KinematicsStatus::INVALID_INPUT);
      CHECK(inverseKinematics(leg, {0, bad, 0}).status == KinematicsStatus::INVALID_INPUT);
      CHECK(inverseKinematics(leg, {0, 0, bad}).status == KinematicsStatus::INVALID_INPUT);
      IkOptions options; options.seed.hip = bad;
      CHECK(inverseKinematics(leg, zero.footOriginM, options).reachability == Reachability::UNKNOWN);
    }
    for (double tolerance : {nan, inf, -1.0, 0.0, 1e-15, 1.0}) {
      IkOptions options; options.toleranceM = tolerance;
      CHECK(inverseKinematics(leg, zero.footOriginM, options).status == KinematicsStatus::INVALID_INPUT);
    }
    IkOptions options; options.seed.lower = model->limits[2].upper + 1.0;
    CHECK(inverseKinematics(leg, zero.footOriginM, options).status == KinematicsStatus::JOINT_LIMIT);
  }
  CHECK(legModel(static_cast<LegId>(255)) == nullptr);
  CHECK(forwardKinematics(static_cast<LegId>(255), {}).status == KinematicsStatus::INVALID_INPUT);
  CHECK(inverseKinematics(static_cast<LegId>(255), {}).status == KinematicsStatus::INVALID_INPUT);
  // Symmetry is a test of URDF transforms, never an extra production sign rule.
  for (const auto pair : {0u, 3u}) {
    const auto left = forwardKinematics(static_cast<LegId>(pair), {0.2, 0.4, -0.5});
    const auto right = forwardKinematics(static_cast<LegId>(pair == 0 ? 1 : 2), {-0.2, 0.4, -0.5});
    CHECK(near(left.footOriginM.x, right.footOriginM.x));
    CHECK(near(left.footOriginM.y, -right.footOriginM.y));
    CHECK(near(left.footOriginM.z, right.footOriginM.z));
  }
  const auto front = forwardKinematics(LegId::LF, {0.2, 0.4, -0.5});
  const auto rear = forwardKinematics(LegId::LH, {0.2, 0.4, -0.5});
  CHECK(near(front.footOriginM.x - rear.footOriginM.x, 0.225));
  CHECK(near(front.footOriginM.y, rear.footOriginM.y));
  CHECK(near(front.footOriginM.z - rear.footOriginM.z, 0.020));
  std::printf("MOTION_KINEMATICS: %u checks, %u failures\n", checks, failures);
  return failures ? 1 : 0;
}
