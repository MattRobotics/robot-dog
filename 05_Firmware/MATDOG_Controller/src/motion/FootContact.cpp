#include "FootContact.h"
#include "FootContactData.h"
#include "LegKinematicsInternal.h"
#include "MotionHotPath.h"
#include <cmath>

namespace matdog { namespace motion {
namespace {
using detail::Vec3f;
using detail::JointsF;
Vec3f add(Vec3f a, Vec3f b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3f scale(Vec3f a, float s) { return {a.x*s, a.y*s, a.z*s}; }
float dot(Vec3f a, Vec3f b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
// Unit axes/normals and decimetre offsets: no overflow or underflow to guard.
float norm(Vec3f a) { return std::sqrt(dot(a,a)); }
Vec3f rotate(const float r[3][3], Vec3f a) {
  return {r[0][0]*a.x+r[0][1]*a.y+r[0][2]*a.z,
          r[1][0]*a.x+r[1][1]*a.y+r[1][2]*a.z,
          r[2][0]*a.x+r[2][1]*a.y+r[2][2]*a.z};
}
double dotD(Vector3 a, Vector3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
bool finite(Vector3 a) { return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z); }
MATDOG_MOTION_HOT ContactStatus convert(KinematicsStatus s) {
  switch (s) {
    case KinematicsStatus::OK: return ContactStatus::OK;
    case KinematicsStatus::JOINT_LIMIT: return ContactStatus::JOINT_LIMIT;
    case KinematicsStatus::UNREACHABLE_GEOMETRY: return ContactStatus::UNREACHABLE_GEOMETRY;
    case KinematicsStatus::UNREACHABLE_LIMITS: return ContactStatus::UNREACHABLE_LIMITS;
    case KinematicsStatus::NUMERICAL_FAILURE: return ContactStatus::NUMERICAL_FAILURE;
    default: return ContactStatus::INVALID_INPUT;
  }
}
using detail::kContactF;
}
const FootContactModel& footContactModel() { return generated::kContact; }
const char* contactGeometrySha256() { return generated::kContactSha256; }
ContactResult detail::toD(const ContactF& c) {
  ContactResult r;
  r.status = c.status; r.support = c.support;
  r.cylinderCenterM = toD(c.cylinderCenterM); r.cylinderAxis = toD(c.cylinderAxis); r.radialDown = toD(c.radialDown);
  r.referenceM = toD(c.referenceM); r.stripEndAM = toD(c.stripEndAM); r.stripEndBM = toD(c.stripEndBM);
  r.lowestCoreM = toD(c.lowestCoreM); r.axisTiltRad = c.axisTiltRad;
  return r;
}
MATDOG_MOTION_HOT float detail::edgeContactLift(float tilt) {
  const auto& m = kContactF;
  tilt = std::abs(tilt);
  if (!std::isfinite(tilt) || tilt <= m.nominalTiltRad) return 0.0f;
  auto rigid = [&](float t) { return .5f*m.supportWidthM*std::sin(t) + m.filletRadiusM*(1.0f - std::cos(t)); };
  return rigid(tilt) - rigid(m.nominalTiltRad);
}
double edgeContactLiftM(double tilt) { return detail::edgeContactLift(static_cast<float>(tilt)); }

MATDOG_MOTION_HOT detail::ContactF detail::contactForward(LegId leg, const JointsF& q, const Vec3f& normal) {
  ContactF result;
  const auto fk = forwardWithModel(*modelF(leg), q, 0.0f);
  result.status = convert(fk.status);
  if (fk.status != KinematicsStatus::OK) return result;
  const auto& m = kContactF;
  result.cylinderAxis = rotate(fk.baseFromFootRotation, m.axisInFoot);
  result.cylinderAxis = scale(result.cylinderAxis, 1.0f/norm(result.cylinderAxis));
  result.cylinderCenterM = add(fk.footOriginM, rotate(fk.baseFromFootRotation, m.centerInFootM));
  const float component = dot(result.cylinderAxis, normal);
  const Vec3f perpendicular = add(normal, scale(result.cylinderAxis, -component));
  const float perpendicularNorm = norm(perpendicular);
  if (perpendicularNorm <= kDegenerate) {
    result.status = ContactStatus::DEGENERATE;
    return result;
  }
  result.radialDown = scale(perpendicular, -1.0f/perpendicularNorm);
  result.referenceM = add(result.cylinderCenterM, scale(result.radialDown, m.radiusM));
  const Vec3f halfStrip = scale(result.cylinderAxis, m.supportWidthM/2.0f);
  result.stripEndAM = add(result.referenceM, scale(halfStrip, -1.0f));
  result.stripEndBM = add(result.referenceM, halfStrip);
  result.lowestCoreM = dot(normal, result.stripEndAM) <= dot(normal, result.stripEndBM)
                      ? result.stripEndAM : result.stripEndBM;
  result.axisTiltRad = std::asin(std::fmin(1.0f, std::abs(component)));
  result.support = result.axisTiltRad <= m.nominalTiltRad
                   ? SupportMode::NOMINAL_STRIP : SupportMode::EDGE_BIASED;
  return result;
}

ContactResult contactForwardKinematics(LegId leg, const LegJointAngles& q, Vector3 normal) {
  ContactResult result;
  if (!finite(normal)) return result;
  const double length = std::sqrt(dotD(normal, normal));
  if (!std::isfinite(length) || length <= 1e-12) return result;
  JointsF image;
  const KinematicsStatus status = detail::jointsF(leg, q, image);
  if (status != KinematicsStatus::OK) { result.status = convert(status); return result; }
  const Vec3f n = detail::toF(normal);
  return detail::toD(detail::contactForward(leg, image, scale(n, 1.0f/norm(n))));
}

bool lowestTreadHeightM(const ContactResult& contact, Vector3 up, double& heightM) {
  heightM=0;
  const double length=std::sqrt(dotD(up,up));
  if(contact.status!=ContactStatus::OK||!finite(up)||!std::isfinite(length)||length<=1e-12||
     !finite(contact.lowestCoreM)||!std::isfinite(contact.axisTiltRad))return false;
  // lowestCoreM already includes the central strip endpoint. The fillet
  // extends below it; edgeContactLiftM is relative to nominal tilt, so is
  // deliberately NOT used as an absolute ground-height correction.
  const double inverse=1.0/length;
  heightM=dotD({up.x*inverse,up.y*inverse,up.z*inverse},contact.lowestCoreM)-
          footContactModel().filletRadiusM*(1.0-std::cos(contact.axisTiltRad));
  return std::isfinite(heightM);
}

MATDOG_MOTION_HOT detail::ContactIkF detail::contactInverse(LegId leg, const Vec3f& target, const JointsF& seed,
                                                            float toleranceM, bool requireNominalStrip) {
  ContactIkF result;
  // +Z ground and cos(h)>0 imply radialDown = Rx(h)*(0,0,-1).
  // Center is Rfoot*(0,0,r): use distal Z+r and separate hip-frame Z=-r.
  // This is a virtual analytic chain, never a changed URDF/foot-link origin.
  ModelF contactModel = *modelF(leg);
  const float radius = kContactF.radiusM;
  contactModel.footOrigin.z += radius;
  AnalyticBranch branch;
  const auto ik = inverseWithModel(contactModel, target, seed, toleranceM, -radius, &branch);
  result.status = convert(ik.status);
  result.reachability = ik.reachability;
  if (ik.status != KinematicsStatus::OK) return result;
  const auto contact = contactForward(leg, ik.joints, {0.0f, 0.0f, 1.0f});
  const float residual = norm(add(target, scale(contact.referenceM, -1.0f)));
  // Independent full orientation/contact evaluation verifies the reduction.
  if (contact.status != ContactStatus::OK || !std::isfinite(residual) ||
      residual > toleranceM) {
    result.status = ContactStatus::NUMERICAL_FAILURE;
    result.reachability = Reachability::UNKNOWN;
    return result;
  }
  if (requireNominalStrip && contact.support != SupportMode::NOMINAL_STRIP) {
    result.status = ContactStatus::CONTACT_MODE;
    return result;
  }
  result.joints = ik.joints;
  result.contact = contact;
  result.residualM = residual;
  result.solutions = ik.solutions;
  result.branch = {branch.hip, branch.elbow};
  return result;
}

ContactIkResult contactInverseKinematics(LegId leg, const Vector3& target,
                                         const ContactIkOptions& options) {
  ContactIkResult result;
  if (!legModel(leg)) return result;
  if (!finite(target) || !std::isfinite(options.kinematics.toleranceM) ||
      options.kinematics.toleranceM < double(detail::kResidualM) || options.kinematics.toleranceM > 1e-3) return result;
  JointsF seed;
  const KinematicsStatus status = detail::jointsF(leg, options.kinematics.seed, seed);
  if (status != KinematicsStatus::OK) { result.status = convert(status); return result; }
  const auto ik = detail::contactInverse(leg, detail::toF(target), seed, static_cast<float>(options.kinematics.toleranceM),
                                         options.requireNominalStrip);
  result.status = ik.status;
  result.reachability = ik.reachability;
  if (ik.status != ContactStatus::OK) return result;
  result.joints = detail::toD(ik.joints);
  result.contact = detail::toD(ik.contact);
  result.residualM = ik.residualM;
  result.solutions = ik.solutions;
  result.branch = ik.branch;
  return result;
}
} }
