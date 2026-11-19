#include "LegKinematicsInternal.h"

#include <cmath>
#include <limits>

namespace matdog {
namespace motion {
namespace {
using detail::Vec3f;
constexpr float kPi = 3.14159265358979323846f;

float clamp(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

// Decimetre offsets from finite targets: a square overflows only for a target
// beyond float range, and that infinity fails the reach test as it should.
float norm(const Vec3f& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

// The generator requires every limit interval to be strictly inside (-pi, pi).
// Hence each angle has at most one equivalent representative in that interval.
bool boundedAngle(float raw, float lower, float upper, float& value) {
  // |raw| <= pi: the nearest multiple of 2pi is 0, so remainder(raw, 2pi) ==
  // raw bit for bit; the remainder is only paid outside (-pi, pi].
  value = std::abs(raw) <= kPi ? raw : std::remainder(raw, 2.0f * kPi);
  if (value < lower - detail::kLimitEpsilonRad ||
      value > upper + detail::kLimitEpsilonRad) return false;
  value = clamp(value, lower, upper);
  return true;
}
// Constants of one virtual chain: b = planar foot offset, phi = its angle,
// outer = reach of the chain. They depend only on the generated model and on
// the caller's hipFrameZ (0 or -radius), never on the target. Memoized exactly
// (same inputs, same bits) so the analytic IK pays hypot/atan2 for them once
// per chain instead of on every solve. Motion, STAND planning and the offline
// tools call the IK from one task; no other context solves IK.
struct ChainConstants { float fx, fz, lateral, a, hipFrameZ, b, phi, outer; bool valid; };
const ChainConstants& chainConstants(const detail::ModelF& m, float a, float lateral, float hipFrameZ) {
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
  IkResult result;
  if (!legModel(leg) || !std::isfinite(target.x) || !std::isfinite(target.y) ||
      !std::isfinite(target.z) || !std::isfinite(options.toleranceM) ||
      options.toleranceM < double(detail::kResidualM) || options.toleranceM > 1e-3) return result;
  // The seed only has to be a valid FK input (finite, inside the URDF
  // limits): the same status forwardKinematics would return, without its trig.
  detail::JointsF seed;
  result.status = detail::jointsF(leg, options.seed, seed);
  if (result.status != KinematicsStatus::OK) return result;
  const auto ik = detail::inverseWithModel(*detail::modelF(leg), detail::toF(target), seed,
                                           static_cast<float>(options.toleranceM), 0.0f);
  result.status = ik.status;
  result.reachability = ik.reachability;
  result.joints = detail::toD(ik.joints);
  result.achievedFootOriginM = detail::toD(ik.achievedFootOriginM);
  result.residualM = ik.residualM;
  result.solutions = ik.solutions;
  return result;
}

detail::IkF detail::inverseWithModel(const ModelF& model, const Vec3f& target, const JointsF& seed,
                                                       float toleranceM, float hipFrameZ, AnalyticBranch* branch) {
  IkF result;
  if (!(toleranceM >= kResidualM && toleranceM <= 1e-3f)) return result;
  const Vec3f p = {target.x - model.hipOrigin.x,
                   target.y - model.hipOrigin.y,
                   target.z - model.hipOrigin.z};
  const float a = -model.lowerOrigin.z;
  const float lateral = model.upperOrigin.y + model.footOrigin.y;
  const auto& chain = chainConstants(model, a, lateral, hipFrameZ);
  const float b = chain.b;
  const float radius = std::sqrt(p.y * p.y + p.z * p.z);
  const float outer = chain.outer;
  result.status = KinematicsStatus::UNREACHABLE_GEOMETRY;
  result.reachability = Reachability::UNREACHABLE;
  // Also rejects a target beyond float range (infinite norm).
  if (!(norm(p) <= outer + kDomainEpsilon) ||
      radius < std::abs(lateral) - kDomainEpsilon) return result;

  const float alpha = std::atan2(p.z, p.y);
  const float hipDelta = std::acos(clamp(lateral / radius, -1.0f, 1.0f));
  const float phi = chain.phi;
  bool geometryExists = false;
  bool limitedCandidateExists = false;
  struct Candidate { JointsF q; float distance; AnalyticBranch branch; };
  Candidate candidates[4]{};
  unsigned count = 0;

  // Two hip branches, each with two planar elbow branches. Both are required
  // even for a target generated by FK; choosing a fixed knee sign is unsound.
  for (unsigned hipBranch = 0; hipBranch < 2; ++hipBranch) {
    const float h = alpha + (hipBranch == 0 ? hipDelta : -hipDelta);
    const float hipV = -std::sin(h) * p.y + std::cos(h) * p.z;
    const float v = hipV - hipFrameZ;
    const float cosine = (p.x * p.x + v * v - a * a - b * b) / (2.0f * a * b);
    if (cosine < -1.0f - kDomainEpsilon || cosine > 1.0f + kDomainEpsilon) continue;
    geometryExists = true;
    const float elbow = std::acos(clamp(cosine, -1.0f, 1.0f));
    // sin is odd, cos even: one evaluation serves both elbow branches, and
    // atan2 is odd in its first argument (the sign is applied after the
    // reduction in every libm), so one atan2 serves both too, bit for bit.
    const float toTarget = std::atan2(-p.x, -v), se = std::sin(elbow), ce = std::cos(elbow);
    const float elbowAngle = std::atan2(b * se, a + b * ce);
    for (unsigned elbowBranch = 0; elbowBranch < 2; ++elbowBranch) {
      const float beta = elbowBranch == 0 ? elbow : -elbow;
      const float u = toTarget - (elbowBranch == 0 ? elbowAngle : -elbowAngle);
      JointsF q{};
      if (!boundedAngle(h, model.lower[0], model.upper[0], q.hip) ||
          !boundedAngle(u, model.lower[1], model.upper[1], q.upper) ||
          !boundedAngle(beta + phi, model.lower[2], model.upper[2], q.lower)) continue;
      limitedCandidateExists = true;
      const float dh = q.hip - seed.hip;
      const float du = q.upper - seed.upper;
      const float dl = q.lower - seed.lower;
      candidates[count++] = {q, dh * dh + du * du + dl * dl,
                             {static_cast<int8_t>(hipV >= 0.0f ? 1 : -1), static_cast<int8_t>(beta >= 0.0f ? 1 : -1)}};
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
    const FkF fk = forwardWithModel(model, c.q, hipFrameZ);
    const float residual = norm({target.x - fk.footOriginM.x,
                                 target.y - fk.footOriginM.y,
                                 target.z - fk.footOriginM.z});
    if (fk.status != KinematicsStatus::OK || !std::isfinite(residual) ||
        residual > toleranceM) continue;
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
