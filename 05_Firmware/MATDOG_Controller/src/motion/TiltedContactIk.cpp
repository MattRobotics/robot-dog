#include "TiltedContactIk.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool finite(Vector3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
bool finiteQ(const LegJointAngles& q) { return std::isfinite(q.hip) && std::isfinite(q.upper) && std::isfinite(q.lower); }
double& at(LegJointAngles& q, unsigned i) { return i == 0 ? q.hip : i == 1 ? q.upper : q.lower; }
double get(const LegJointAngles& q, unsigned i) { return i == 0 ? q.hip : i == 1 ? q.upper : q.lower; }
bool inLimits(LegId leg, const LegJointAngles& q) {
  const auto* m = legModel(leg);
  for (unsigned i = 0; i < 3; ++i) if (get(q, i) < m->limits[i].lower || get(q, i) > m->limits[i].upper) return false;
  return true;
}
Vector3 sub(Vector3 a, Vector3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
double norm(Vector3 a) { return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
bool solve3(const double j[3][3], const double b[3], double x[3]) {
  double m[3][4];
  for (unsigned r = 0; r < 3; ++r) { for (unsigned c = 0; c < 3; ++c) m[r][c] = j[r][c]; m[r][3] = b[r]; }
  for (unsigned c = 0; c < 3; ++c) {
    unsigned p = c;
    for (unsigned r = c + 1; r < 3; ++r) if (std::abs(m[r][c]) > std::abs(m[p][c])) p = r;
    if (!(std::abs(m[p][c]) > 1e-12)) return false;
    for (unsigned k = 0; k < 4; ++k) { double t = m[p][k]; m[p][k] = m[c][k]; m[c][k] = t; }
    const double d = m[c][c];
    for (unsigned k = c; k < 4; ++k) m[c][k] /= d;
    for (unsigned r = 0; r < 3; ++r) if (r != c) { const double f = m[r][c]; for (unsigned k = c; k < 4; ++k) m[r][k] -= f * m[c][k]; }
  }
  for (unsigned r = 0; r < 3; ++r) { x[r] = m[r][3]; if (!std::isfinite(x[r])) return false; }
  return true;
}
// Residual of the contact reference against the target; false when the forward model is not usable at q.
bool residual(LegId leg, const LegJointAngles& q, const Vector3& normal, const Vector3& target, Vector3& r, ContactResult& c) {
  c = contactForwardKinematics(leg, q, normal);
  if (c.status != ContactStatus::OK) return false;
  r = sub(c.referenceM, target);
  return finite(r);
}

Vector3 add(Vector3 a, Vector3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vector3 scale(Vector3 a, double t) { return {a.x*t, a.y*t, a.z*t}; }
double dot(Vector3 a, Vector3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vector3 cross(Vector3 a, Vector3 b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
// Differential of the existing X/Y/Y URDF chain and cylinder-contact model.
// Joint axes are expressed in base_link; no geometry or contact policy changes.
// The forward contact result at q has already been computed by residual().
bool contactJacobian(LegId leg, const LegJointAngles& q, Vector3 n,
                     const ContactResult& c, double J[3][3]) {
  const auto* m = legModel(leg);
  if (!m || c.status != ContactStatus::OK) return false;
  // Preserve the central-difference solver's validity at q +/- 1e-6 rad.
  constexpr double h = 1e-6;
  for (unsigned j = 0; j < 3; ++j) {
    const double qj = get(q, j);
    if (qj-h < m->limits[j].lower || qj+h > m->limits[j].upper) return false;
  }
  const double sh = std::sin(q.hip), ch = std::cos(q.hip);
  const double su = std::sin(q.upper), cu = std::cos(q.upper);
  const Vector3 hip = m->hipOrigin;
  const Vector3 u = m->upperOrigin;
  const Vector3 upper = add(hip, {u.x, ch*u.y-sh*u.z, sh*u.y+ch*u.z});
  const Vector3 l = m->lowerOrigin;
  const Vector3 rotatedLower = {cu*l.x+su*l.z, l.y, -su*l.x+cu*l.z};
  const Vector3 lower = add(upper,
      {rotatedLower.x, ch*rotatedLower.y-sh*rotatedLower.z,
       sh*rotatedLower.y+ch*rotatedLower.z});
  const Vector3 axes[3] = {{1,0,0}, {0,ch,sh}, {0,ch,sh}};
  const Vector3 origins[3] = {hip, upper, lower};

  const Vector3 axis = c.cylinderAxis;
  const double component = dot(axis, n);
  const Vector3 perpendicular = sub(n, scale(axis, component));
  const double d = norm(perpendicular);
  if (!(d > 1e-9)) return false;
  const double invD = 1.0/d, invD3 = invD*invD*invD;
  const double radius = footContactModel().radiusM;
  for (unsigned j = 0; j < 3; ++j) {
    const Vector3 dc = cross(axes[j], sub(c.cylinderCenterM, origins[j]));
    const Vector3 da = cross(axes[j], axis);
    const Vector3 dp = add(scale(da, -component),
                           scale(axis, -dot(da, n)));
    const Vector3 dUnit = add(scale(dp, invD),
                              scale(perpendicular, -dot(perpendicular, dp)*invD3));
    const Vector3 deriv = add(dc, scale(dUnit, -radius));
    J[0][j] = deriv.x; J[1][j] = deriv.y; J[2][j] = deriv.z;
  }
  return true;
}
}
TiltedIkResult contactInverseKinematicsGroundNormal(LegId leg, const Vector3& target, const Vector3& normal, const TiltedIkOptions& o) {
  TiltedIkResult out;
  if (!legModel(leg) || !finite(target) || !finite(normal) || !finiteQ(o.seed) || !(o.toleranceM > 0) || o.maxIterations == 0) return out;
  const double nl = norm(normal);
  if (!(nl > 1e-12)) return out;
  const Vector3 n = {normal.x/nl, normal.y/nl, normal.z/nl};
  if (!inLimits(leg, o.seed)) return out;  // a seed outside the limits is rejected, never clamped
  LegJointAngles q = o.seed;
  Vector3 r{}; ContactResult c{};
  ++out.fkEvaluations;
  if (!residual(leg, q, n, target, r, c)) { out.status = TiltedIkResult::Status::NO_CONVERGENCE; return out; }
  for (unsigned it = 0; it < o.maxIterations; ++it) {
    out.iterations = it;
    // Stop at the acceptance tolerance. One servo tick at the foot is ~2e-4 m;
    // a further polish step costs a full soft-double FK + Jacobian on the
    // ESP32-S3 and moves the joints by <1e-7 rad (no RAW change).
    if (norm(r) <= o.toleranceM) break;
    ++out.newtonSteps;
    double J[3][3];
    if (!contactJacobian(leg, q, n, c, J)) { out.status = TiltedIkResult::Status::JOINT_LIMIT; return out; }
    const double rhs[3] = {-r.x, -r.y, -r.z};
    double dq[3];
    if (!solve3(J, rhs, dq)) { out.status = TiltedIkResult::Status::SINGULAR; return out; }
    // Halve the step until it stays inside the limits and does not increase the residual; never clip.
    double step = 1.0; bool accepted = false;
    for (unsigned k = 0; k < 10 && !accepted; ++k, step *= .5) {
      LegJointAngles qn = q;
      for (unsigned j = 0; j < 3; ++j) at(qn, j) += step * dq[j];
      Vector3 rn{}; ContactResult cn{};
      if (!inLimits(leg, qn)) continue;
      ++out.fkEvaluations;
      if (!residual(leg, qn, n, target, rn, cn)) continue;
      if (norm(rn) < norm(r) || norm(rn) <= o.toleranceM) { q = qn; r = rn; c = cn; accepted = true; }
    }
    if (!accepted) {
      if (norm(r) <= o.toleranceM) break;  // already inside the acceptance tolerance, no better step exists
      out.status = inLimits(leg, q) ? TiltedIkResult::Status::NO_CONVERGENCE : TiltedIkResult::Status::JOINT_LIMIT; return out;
    }
  }
  if (!(norm(r) <= o.toleranceM)) { out.status = TiltedIkResult::Status::NO_CONVERGENCE; return out; }
  // Conditioning of the converged Jacobian (infinity-norm), reported for the caller's policy.
  double J[3][3], inv[3][3], jn = 0, in = 0;
  if (!contactJacobian(leg, q, n, c, J)) { out.status = TiltedIkResult::Status::JOINT_LIMIT; return out; }
  for (unsigned col = 0; col < 3; ++col) {
    double e[3] = {0, 0, 0}, x[3]; e[col] = 1;
    if (!solve3(J, e, x)) { out.status = TiltedIkResult::Status::SINGULAR; return out; }
    for (unsigned k = 0; k < 3; ++k) inv[k][col] = x[k];
  }
  for (unsigned k = 0; k < 3; ++k) {
    double a = 0, b = 0;
    for (unsigned col = 0; col < 3; ++col) { a += std::abs(J[k][col]); b += std::abs(inv[k][col]); }
    jn = std::fmax(jn, a); in = std::fmax(in, b);
  }
  out.condition = jn * in;
  out.joints = q; out.contact = c; out.residualM = norm(r);
  if (o.requireNominalStrip && c.support != SupportMode::NOMINAL_STRIP) { out.status = TiltedIkResult::Status::CONTACT_MODE; return out; }
  out.status = TiltedIkResult::Status::OK;
  return out;
}
WorldTiltedIkResult worldContactInverseKinematicsTilted(LegId leg, const Vector3& worldTarget, const BodyPose& pose, const TiltedIkOptions& o) {
  WorldTiltedIkResult out;
  Vector3 base;
  if (!pointToBase(pose, worldTarget, base)) return out;
  // World +Z expressed in base_link is the third row of the rotation: R^T e_z.
  const Vector3 normal = {pose.rotation[2][0], pose.rotation[2][1], pose.rotation[2][2]};
  out.base = contactInverseKinematicsGroundNormal(leg, base, normal, o);
  out.status = out.base.status;
  if (out.status != TiltedIkResult::Status::OK) return out;
  if (!pointToWorld(pose, out.base.contact.referenceM, out.achievedWorldM)) out.status = TiltedIkResult::Status::INVALID_INPUT;
  return out;
}
} }
