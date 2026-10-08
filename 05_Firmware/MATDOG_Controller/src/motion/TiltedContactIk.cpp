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
  if (!residual(leg, q, n, target, r, c)) { out.status = TiltedIkResult::Status::NO_CONVERGENCE; return out; }
  const double h = 1e-6;
  for (unsigned it = 0; it < o.maxIterations; ++it) {
    out.iterations = it;
    if (norm(r) <= 1e-13) break;  // polish well below the acceptance tolerance: quadratic convergence makes it cheap
    double J[3][3];
    for (unsigned j = 0; j < 3; ++j) {
      LegJointAngles qp = q, qm = q; at(qp, j) += h; at(qm, j) -= h;
      Vector3 rp{}, rm{}; ContactResult cp{}, cm{};
      if (!residual(leg, qp, n, target, rp, cp) || !residual(leg, qm, n, target, rm, cm)) { out.status = TiltedIkResult::Status::JOINT_LIMIT; return out; }
      J[0][j] = (rp.x - rm.x)/(2*h); J[1][j] = (rp.y - rm.y)/(2*h); J[2][j] = (rp.z - rm.z)/(2*h);
    }
    const double rhs[3] = {-r.x, -r.y, -r.z};
    double dq[3];
    if (!solve3(J, rhs, dq)) { out.status = TiltedIkResult::Status::SINGULAR; return out; }
    // Halve the step until it stays inside the limits and does not increase the residual; never clip.
    double step = 1.0; bool accepted = false;
    for (unsigned k = 0; k < 10 && !accepted; ++k, step *= .5) {
      LegJointAngles qn = q;
      for (unsigned j = 0; j < 3; ++j) at(qn, j) += step * dq[j];
      Vector3 rn{}; ContactResult cn{};
      if (!inLimits(leg, qn) || !residual(leg, qn, n, target, rn, cn)) continue;
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
  for (unsigned j = 0; j < 3; ++j) {
    LegJointAngles qp = q, qm = q; at(qp, j) += h; at(qm, j) -= h;
    Vector3 rp{}, rm{}; ContactResult cp{}, cm{};
    if (!residual(leg, qp, n, target, rp, cp) || !residual(leg, qm, n, target, rm, cm)) { out.status = TiltedIkResult::Status::JOINT_LIMIT; return out; }
    J[0][j] = (rp.x - rm.x)/(2*h); J[1][j] = (rp.y - rm.y)/(2*h); J[2][j] = (rp.z - rm.z)/(2*h);
  }
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
