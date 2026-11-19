#include "TiltedContactIk.h"
#include "LegKinematicsInternal.h"
#include "MotionHotPath.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
using detail::Vec3f;
using detail::JointsF;
using detail::ContactF;
using detail::TiltedIkF;
bool finite(Vector3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
bool finite(Vec3f v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
float& at(JointsF& q, unsigned i) { return i == 0 ? q.hip : i == 1 ? q.upper : q.lower; }
float get(const JointsF& q, unsigned i) { return i == 0 ? q.hip : i == 1 ? q.upper : q.lower; }
Vec3f sub(Vec3f a, Vec3f b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
float norm(Vec3f a) { return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
bool solve3(const float j[3][3], const float b[3], float x[3]) {
  float m[3][4];
  for (unsigned r = 0; r < 3; ++r) { for (unsigned c = 0; c < 3; ++c) m[r][c] = j[r][c]; m[r][3] = b[r]; }
  for (unsigned c = 0; c < 3; ++c) {
    unsigned p = c;
    for (unsigned r = c + 1; r < 3; ++r) if (std::abs(m[r][c]) > std::abs(m[p][c])) p = r;
    if (!(std::abs(m[p][c]) > detail::kPivot)) return false;
    for (unsigned k = 0; k < 4; ++k) { float t = m[p][k]; m[p][k] = m[c][k]; m[c][k] = t; }
    const float d = m[c][c];
    for (unsigned k = c; k < 4; ++k) m[c][k] /= d;
    for (unsigned r = 0; r < 3; ++r) if (r != c) { const float f = m[r][c]; for (unsigned k = c; k < 4; ++k) m[r][k] -= f * m[c][k]; }
  }
  for (unsigned r = 0; r < 3; ++r) { x[r] = m[r][3]; if (!std::isfinite(x[r])) return false; }
  return true;
}
// Residual of the contact reference against the target; false when the forward model is not usable at q.
bool residual(LegId leg, const JointsF& q, const Vec3f& normal, const Vec3f& target, Vec3f& r, ContactF& c) {
  c = detail::contactForward(leg, q, normal);
  if (c.status != ContactStatus::OK) return false;
  r = sub(c.referenceM, target);
  return finite(r);
}

Vec3f add(Vec3f a, Vec3f b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3f scale(Vec3f a, float t) { return {a.x*t, a.y*t, a.z*t}; }
float dot(Vec3f a, Vec3f b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3f cross(Vec3f a, Vec3f b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
// Differential of the existing X/Y/Y URDF chain and cylinder-contact model.
// Joint axes are expressed in base_link; no geometry or contact policy changes.
// The forward contact result at q has already been computed by residual().
bool contactJacobian(LegId leg, const JointsF& q, Vec3f n, const ContactF& c, float J[3][3]) {
  const auto* m = detail::modelF(leg);
  if (!m || c.status != ContactStatus::OK) return false;
  // Preserve the central-difference solver's validity at q +/- 1e-6 rad.
  constexpr float h = 1e-6f;
  for (unsigned j = 0; j < 3; ++j) {
    const float qj = get(q, j);
    if (qj-h < m->lower[j] || qj+h > m->upper[j]) return false;
  }
  const float sh = std::sin(q.hip), ch = std::cos(q.hip);
  const float su = std::sin(q.upper), cu = std::cos(q.upper);
  const Vec3f hip = m->hipOrigin;
  const Vec3f u = m->upperOrigin;
  const Vec3f upper = add(hip, {u.x, ch*u.y-sh*u.z, sh*u.y+ch*u.z});
  const Vec3f l = m->lowerOrigin;
  const Vec3f rotatedLower = {cu*l.x+su*l.z, l.y, -su*l.x+cu*l.z};
  const Vec3f lower = add(upper,
      {rotatedLower.x, ch*rotatedLower.y-sh*rotatedLower.z,
       sh*rotatedLower.y+ch*rotatedLower.z});
  const Vec3f axes[3] = {{1,0,0}, {0,ch,sh}, {0,ch,sh}};
  const Vec3f origins[3] = {hip, upper, lower};

  const Vec3f axis = c.cylinderAxis;
  const float component = dot(axis, n);
  const Vec3f perpendicular = sub(n, scale(axis, component));
  const float d = norm(perpendicular);
  if (!(d > detail::kDegenerate)) return false;
  const float invD = 1.0f/d, invD3 = invD*invD*invD;
  const float radius = detail::kContactF.radiusM;
  for (unsigned j = 0; j < 3; ++j) {
    const Vec3f dc = cross(axes[j], sub(c.cylinderCenterM, origins[j]));
    const Vec3f da = cross(axes[j], axis);
    const Vec3f dp = add(scale(da, -component),
                         scale(axis, -dot(da, n)));
    const Vec3f dUnit = add(scale(dp, invD),
                            scale(perpendicular, -dot(perpendicular, dp)*invD3));
    const Vec3f deriv = add(dc, scale(dUnit, -radius));
    J[0][j] = deriv.x; J[1][j] = deriv.y; J[2][j] = deriv.z;
  }
  return true;
}
// Public-API checks of TiltedIkOptions and of a double seed (INVALID_INPUT as before);
// the tolerance must be one the FP32 core can verify.
bool validOptions(LegId leg, const TiltedIkOptions& o, JointsF& seed) {
  return legModel(leg) && std::isfinite(o.toleranceM) && o.toleranceM >= double(detail::kResidualM) && o.maxIterations != 0 &&
         detail::jointsF(leg, o.seed, seed) == KinematicsStatus::OK;
}
WorldTiltedIkResult worldResult(const TiltedIkF& r, const detail::PoseF& pose) {
  WorldTiltedIkResult out;
  out.base = detail::toD(r);
  out.status = r.status;
  if (out.status == TiltedIkResult::Status::OK) out.achievedWorldM = detail::toWorld(pose, r.contact.referenceM);
  return out;
}
}
TiltedIkResult detail::toD(const TiltedIkF& r) {
  TiltedIkResult out;
  out.status = r.status; out.joints = toD(r.joints); out.contact = toD(r.contact);
  out.residualM = r.residualM; out.condition = r.condition; out.iterations = r.iterations;
  out.newtonSteps = r.newtonSteps; out.fkEvaluations = r.fkEvaluations; out.solves = r.solves;
  return out;
}
bool contactReferenceJacobian(LegId leg,const LegJointAngles& q,Vector3 normal,
                              const ContactResult& contact,double J[3][3]) {
  const double length=std::sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
  JointsF image;
  if(!finite(normal)||detail::jointsF(leg,q,image)!=KinematicsStatus::OK||!std::isfinite(length)||length<=1e-12)return false;
  ContactF c;c.status=contact.status;c.cylinderAxis=detail::toF(contact.cylinderAxis);c.cylinderCenterM=detail::toF(contact.cylinderCenterM);
  float j[3][3];
  const Vec3f n=detail::toF(normal);
  if(!contactJacobian(leg,image,scale(n,1.0f/norm(n)),c,j))return false;
  for(unsigned r=0;r<3;++r)for(unsigned col=0;col<3;++col)J[r][col]=j[r][col];
  return true;
}
TiltedIkF detail::tiltedInverse(LegId leg, const Vec3f& target, const Vec3f& n, const JointsF& seed,
                                                  float toleranceM, unsigned maxIterations, bool requireNominalStrip) {
  TiltedIkF out;
  const auto* m = modelF(leg);
  if (!finite(target) || !finite(n)) return out;
  JointsF q = seed;
  Vec3f r{}; ContactF c{};
  out.solves = 1; ++out.fkEvaluations;
  if (!residual(leg, q, n, target, r, c)) { out.status = TiltedIkResult::Status::NO_CONVERGENCE; return out; }
  // Squared residuals: the same order and acceptance, no sqrt per comparison.
  const float tol2 = toleranceM * toleranceM;
  float r2 = dot(r, r);
  for (unsigned it = 0; it < maxIterations; ++it) {
    out.iterations = it;
    // Stop at the acceptance tolerance: a further polish step moves the
    // joints by far less than a RAW tick.
    if (r2 <= tol2) break;
    ++out.newtonSteps;
    float J[3][3];
    if (!contactJacobian(leg, q, n, c, J)) { out.status = TiltedIkResult::Status::JOINT_LIMIT; return out; }
    const float rhs[3] = {-r.x, -r.y, -r.z};
    float dq[3];
    if (!solve3(J, rhs, dq)) { out.status = TiltedIkResult::Status::SINGULAR; return out; }
    // Halve the step until it stays inside the limits and does not increase the residual; never clip.
    float step = 1.0f; bool accepted = false;
    for (unsigned k = 0; k < 10 && !accepted; ++k, step *= .5f) {
      JointsF qn = q;
      for (unsigned j = 0; j < 3; ++j) at(qn, j) += step * dq[j];
      Vec3f rn{}; ContactF cn{};
      if (!inLimits(*m, qn)) continue;
      ++out.fkEvaluations;
      if (!residual(leg, qn, n, target, rn, cn)) continue;
      const float rn2 = dot(rn, rn);
      if (rn2 < r2 || rn2 <= tol2) { q = qn; r = rn; r2 = rn2; c = cn; accepted = true; }
    }
    if (!accepted) {
      if (r2 <= tol2) break;  // already inside the acceptance tolerance, no better step exists
      out.status = inLimits(*m, q) ? TiltedIkResult::Status::NO_CONVERGENCE : TiltedIkResult::Status::JOINT_LIMIT; return out;
    }
  }
  if (!(r2 <= tol2)) { out.status = TiltedIkResult::Status::NO_CONVERGENCE; return out; }
  // Conditioning of the converged Jacobian (infinity-norm), reported for the caller's policy.
  float J[3][3], inv[3][3], jn = 0, in = 0;
  if (!contactJacobian(leg, q, n, c, J)) { out.status = TiltedIkResult::Status::JOINT_LIMIT; return out; }
  if (!invert3(J, inv)) { out.status = TiltedIkResult::Status::SINGULAR; return out; }
  for (unsigned k = 0; k < 3; ++k) {
    float a = 0, b = 0;
    for (unsigned col = 0; col < 3; ++col) { a += std::abs(J[k][col]); b += std::abs(inv[k][col]); }
    jn = std::fmax(jn, a); in = std::fmax(in, b);
  }
  out.condition = jn * in;
  out.joints = q; out.contact = c; out.residualM = std::sqrt(r2);
  if (requireNominalStrip && c.support != SupportMode::NOMINAL_STRIP) { out.status = TiltedIkResult::Status::CONTACT_MODE; return out; }
  out.status = TiltedIkResult::Status::OK;
  return out;
}
TiltedIkResult contactInverseKinematicsGroundNormal(LegId leg, const Vector3& target, const Vector3& normal, const TiltedIkOptions& o) {
  JointsF seed;
  if (!finite(target) || !finite(normal) || !validOptions(leg, o, seed)) return {};
  const double nl = std::sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
  if (!(nl > 1e-12)) return {};
  const Vec3f n = detail::toF(normal);
  return detail::toD(detail::tiltedInverse(leg, detail::toF(target), scale(n, 1.0f/norm(n)), seed,
                                           static_cast<float>(o.toleranceM), o.maxIterations, o.requireNominalStrip));
}
MATDOG_MOTION_HOT WorldTiltedIkResult worldContactInverseKinematicsTilted(LegId leg, const Vector3& worldTarget, const BodyPose& pose, const TiltedIkOptions& o) {
  detail::PoseF f; JointsF seed;
  if (!detail::poseF(pose, f) || !finite(worldTarget) || !validOptions(leg, o, seed)) return {};
  const Vec3f n = detail::groundNormal(f);
  return worldResult(detail::tiltedInverse(leg, detail::toBase(f, worldTarget), scale(n, 1.0f/norm(n)), seed,
                                           static_cast<float>(o.toleranceM), o.maxIterations, o.requireNominalStrip), f);
}
TiltedIkF detail::edgeTiltedInverse(LegId leg, const Vector3& worldTarget, const PoseF& pose, JointsF seed,
                                                      float toleranceM, unsigned maxIterations) {
  // Foot axis in the support frame: R * Rx(hip) * y (UPPER/LOWER keep it).
  auto tilt = [&](float hip) { return std::asin(std::fmin(1.0f, std::abs(pose.rotation[2][1]*std::cos(hip) + pose.rotation[2][2]*std::sin(hip)))); };
  const Vec3f g = groundNormal(pose), n = scale(g, 1.0f/norm(g));
  auto solve = [&](float lift, const JointsF& s) {
    return tiltedInverse(leg, toBase(pose, {worldTarget.x, worldTarget.y, worldTarget.z + double(lift)}), n, s, toleranceM, maxIterations, false);
  };
  float lift = edgeContactLift(tilt(seed.hip));
  auto r = solve(lift, seed);
  if (r.status == TiltedIkResult::Status::OK && std::abs(edgeContactLift(r.contact.axisTiltRad) - lift) > 2e-5f) {
    lift = edgeContactLift(r.contact.axisTiltRad); seed = r.joints;
    const unsigned solves = r.solves, newton = r.newtonSteps, fk = r.fkEvaluations;
    r = solve(lift, seed);
    r.solves += solves; r.newtonSteps += newton; r.fkEvaluations += fk;
  }
  return r;
}
WorldTiltedIkResult worldEdgeContactInverseKinematicsTilted(LegId leg, const Vector3& worldTarget, const BodyPose& pose, TiltedIkOptions o) {
  detail::PoseF f; JointsF seed;
  if (!detail::poseF(pose, f) || !finite(worldTarget) || !validOptions(leg, o, seed)) return {};
  return worldResult(detail::edgeTiltedInverse(leg, worldTarget, f, seed, static_cast<float>(o.toleranceM), o.maxIterations), f);
}
} }
