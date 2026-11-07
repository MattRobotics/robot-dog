#include "LegKinematicsInternal.h"

#include <cmath>
#include <limits>

namespace matdog {
namespace motion {
namespace {
constexpr double kPi = 3.14159265358979323846;
// Roundoff guards, not an extension of the physical workspace/URDF limits.
constexpr double kDomainEpsilon = 1e-12;
constexpr double kLimitEpsilonRad = 1e-12;

double clamp(double value, double low, double high) {
  return value < low ? low : (value > high ? high : value);
}

double norm(const Vector3& p) { return std::hypot(std::hypot(p.x, p.y), p.z); }

// The generator requires every limit interval to be strictly inside (-pi, pi).
// Hence each angle has at most one equivalent representative in that interval.
bool boundedAngle(double raw, const JointLimit& limit, double& value) {
  // |raw| <= pi: the nearest multiple of 2pi is 0 (at exactly pi the
  // round-half-even quotient is 0 too), so remainder(raw, 2pi) == raw bit for
  // bit. The soft-double remainder is only paid for angles outside (-pi, pi].
  value = std::abs(raw) <= kPi ? raw : std::remainder(raw, 2.0 * kPi);
  if (value < limit.lower - kLimitEpsilonRad ||
      value > limit.upper + kLimitEpsilonRad) return false;
  value = clamp(value, limit.lower, limit.upper);
  return true;
}
// Constants of one virtual chain: b = planar foot offset, phi = its angle,
// outer = reach of the chain. They depend only on the generated model and on
// the caller's hipFrameZ (0 or -radius), never on the target. Memoized exactly
// (same inputs, same bits) so the analytic IK pays hypot/atan2 for them once
// per chain instead of on every solve. Motion, STAND planning and the offline
// tools call the IK from one task; no other context solves IK.
struct ChainConstants { double fx, fz, lateral, a, hipFrameZ, b, phi, outer; bool valid; };
const ChainConstants& chainConstants(const LegModel& m, double a, double lateral, double hipFrameZ) {
  static ChainConstants cache[8]{};
  static unsigned next = 0;
  for (const auto& c : cache)
    if (c.valid && c.fx == m.footOrigin.x && c.fz == m.footOrigin.z && c.lateral == lateral && c.a == a && c.hipFrameZ == hipFrameZ) return c;
  auto& c = cache[next]; next = (next + 1) % 8;
  c.fx = m.footOrigin.x; c.fz = m.footOrigin.z; c.lateral = lateral; c.a = a; c.hipFrameZ = hipFrameZ;
  c.b = std::hypot(m.footOrigin.x, m.footOrigin.z);
  c.phi = std::atan2(m.footOrigin.x, -m.footOrigin.z);
  c.outer = std::hypot(lateral, a + c.b) + std::abs(hipFrameZ);
  c.valid = true;
  return c;
}
}  // namespace

IkResult inverseKinematics(LegId leg, const Vector3& target,
                           const IkOptions& options) {
  const LegModel* model = legModel(leg);
  return model ? detail::inverseWithModel(*model, target, options, 0.0) : IkResult{};
}

IkResult detail::inverseWithModel(const LegModel& geometry, const Vector3& target,
                                   const IkOptions& options, double hipFrameZ,
                                   AnalyticBranch* branch) {
  IkResult result;
  const LegModel* model = &geometry;
  if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
      !std::isfinite(target.z) || !std::isfinite(options.toleranceM) ||
      options.toleranceM < 1e-12 || options.toleranceM > 1e-3) return result;
  // The seed only has to be a valid FK input (finite, inside the URDF
  // limits): the same status forwardWithModel would return, without its trig.
  const auto& seed = options.seed;
  if (!std::isfinite(seed.hip) || !std::isfinite(seed.upper) || !std::isfinite(seed.lower)) {
    result.status = FkResult{}.status;
    return result;
  }
  const double seedValues[3] = {seed.hip, seed.upper, seed.lower};
  for (unsigned i = 0; i < 3; ++i)
    if (seedValues[i] < model->limits[i].lower || seedValues[i] > model->limits[i].upper) {
      result.status = KinematicsStatus::JOINT_LIMIT;
      return result;
    }
  const Vector3 p = {target.x - model->hipOrigin.x,
                     target.y - model->hipOrigin.y,
                     target.z - model->hipOrigin.z};
  const double a = -model->lowerOrigin.z;
  const double lateral = model->upperOrigin.y + model->footOrigin.y;
  const auto& chain = chainConstants(geometry, a, lateral, hipFrameZ);
  const double b = chain.b;
  const double radius = std::hypot(p.y, p.z);
  const double outer = chain.outer;
  result.status = KinematicsStatus::UNREACHABLE_GEOMETRY;
  result.reachability = Reachability::UNREACHABLE;
  // Reject huge finite inputs before squaring; hypot is overflow-safe.
  if (norm(p) > outer + kDomainEpsilon ||
      radius < std::abs(lateral) - kDomainEpsilon) return result;

  const double alpha = std::atan2(p.z, p.y);
  const double hipDelta = std::acos(clamp(lateral / radius, -1.0, 1.0));
  const double phi = chain.phi;
  bool geometryExists = false;
  bool limitedCandidateExists = false;
  struct Candidate { LegJointAngles q; double distance; AnalyticBranch branch; };
  Candidate candidates[4]{};
  unsigned count = 0;

  // Two hip branches, each with two planar elbow branches. Both are required
  // even for a target generated by FK; choosing a fixed knee sign is unsound.
  for (unsigned hipBranch = 0; hipBranch < 2; ++hipBranch) {
    const double h = alpha + (hipBranch == 0 ? hipDelta : -hipDelta);
    const double hipV = -std::sin(h) * p.y + std::cos(h) * p.z;
    const double v = hipV - hipFrameZ;
    const double cosine = (p.x * p.x + v * v - a * a - b * b) / (2.0 * a * b);
    if (cosine < -1.0 - kDomainEpsilon || cosine > 1.0 + kDomainEpsilon) continue;
    geometryExists = true;
    const double elbow = std::acos(clamp(cosine, -1.0, 1.0));
    // sin is odd, cos even: one evaluation serves both elbow branches, and
    // atan2 is odd in its first argument (the sign is applied after the
    // reduction in every libm), so one atan2 serves both too, bit for bit.
    const double toTarget = std::atan2(-p.x, -v), se = std::sin(elbow), ce = std::cos(elbow);
    const double elbowAngle = std::atan2(b * se, a + b * ce);
    for (unsigned elbowBranch = 0; elbowBranch < 2; ++elbowBranch) {
      const double beta = elbowBranch == 0 ? elbow : -elbow;
      const double u = toTarget - (elbowBranch == 0 ? elbowAngle : -elbowAngle);
      LegJointAngles q{};
      if (!boundedAngle(h, model->limits[0], q.hip) ||
          !boundedAngle(u, model->limits[1], q.upper) ||
          !boundedAngle(beta + phi, model->limits[2], q.lower)) continue;
      limitedCandidateExists = true;
      const double dh = q.hip - options.seed.hip;
      const double du = q.upper - options.seed.upper;
      const double dl = q.lower - options.seed.lower;
      candidates[count++] = {q, dh * dh + du * du + dl * dl,
                             {static_cast<int8_t>(hipV >= 0.0 ? 1 : -1), static_cast<int8_t>(beta >= 0.0 ? 1 : -1)}};
    }
  }
  // The seed-nearest candidate that FK verifies inside the tolerance: the same
  // choice and first-branch tie break as verifying every candidate, with FK
  // only until the first verified one (normally one). solutions counts the
  // verified candidates, i.e. 1 when OK.
  for (unsigned i = 1; i < count; ++i)
    for (unsigned j = i; j > 0 && candidates[j].distance < candidates[j - 1].distance; --j) {
      const Candidate t = candidates[j]; candidates[j] = candidates[j - 1]; candidates[j - 1] = t;
    }
  for (unsigned i = 0; i < count && result.solutions == 0; ++i) {
    const auto& c = candidates[i];
    const FkResult fk = forwardWithModel(geometry, c.q, hipFrameZ);
    const double residual = norm({target.x - fk.footOriginM.x,
                                  target.y - fk.footOriginM.y,
                                  target.z - fk.footOriginM.z});
    if (fk.status != KinematicsStatus::OK || !std::isfinite(residual) ||
        residual > options.toleranceM) continue;
    result.solutions = 1;
    result.joints = c.q;
    result.achievedFootOriginM = fk.footOriginM;
    result.residualM = residual;
    if (branch) *branch = c.branch;
  }
  if (result.solutions != 0) {
    result.status = KinematicsStatus::OK;
    result.reachability = Reachability::REACHABLE;
  } else if (limitedCandidateExists) {
    result.status = KinematicsStatus::NUMERICAL_FAILURE;
    result.reachability = Reachability::UNKNOWN;
  } else if (geometryExists) {
    result.status = KinematicsStatus::UNREACHABLE_LIMITS;
  }
  return result;
}

}  // namespace motion
}  // namespace matdog
