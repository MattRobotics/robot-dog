#include "FootContact.h"
#include "FootContactData.h"
#include "LegKinematicsInternal.h"
#include <cmath>

namespace matdog { namespace motion {
namespace {
Vector3 add(Vector3 a, Vector3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vector3 scale(Vector3 a, double s) { return {a.x*s, a.y*s, a.z*s}; }
double dot(Vector3 a, Vector3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
double norm(Vector3 a) { return std::hypot(std::hypot(a.x,a.y),a.z); }
bool finite(Vector3 a) { return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z); }
Vector3 rotate(const double r[3][3], Vector3 a) {
  return {r[0][0]*a.x+r[0][1]*a.y+r[0][2]*a.z,
          r[1][0]*a.x+r[1][1]*a.y+r[1][2]*a.z,
          r[2][0]*a.x+r[2][1]*a.y+r[2][2]*a.z};
}
ContactStatus convert(KinematicsStatus s) {
  switch (s) {
    case KinematicsStatus::OK: return ContactStatus::OK;
    case KinematicsStatus::JOINT_LIMIT: return ContactStatus::JOINT_LIMIT;
    case KinematicsStatus::UNREACHABLE_GEOMETRY: return ContactStatus::UNREACHABLE_GEOMETRY;
    case KinematicsStatus::UNREACHABLE_LIMITS: return ContactStatus::UNREACHABLE_LIMITS;
    case KinematicsStatus::NUMERICAL_FAILURE: return ContactStatus::NUMERICAL_FAILURE;
    default: return ContactStatus::INVALID_INPUT;
  }
}
}
const FootContactModel& footContactModel() { return generated::kContact; }
const char* contactGeometrySha256() { return generated::kContactSha256; }

ContactResult contactForwardKinematics(LegId leg, const LegJointAngles& q, Vector3 normal) {
  ContactResult result;
  if (!finite(normal)) return result;
  const double length = norm(normal);
  if (!std::isfinite(length) || length <= 1e-12) return result;
  normal = scale(normal, 1.0/length);
  const auto fk = forwardKinematics(leg, q);
  result.status = convert(fk.status);
  if (fk.status != KinematicsStatus::OK) return result;
  const auto& m = footContactModel();
  result.cylinderAxis = rotate(fk.baseFromFootRotation, m.axisInFoot);
  result.cylinderAxis = scale(result.cylinderAxis, 1.0/norm(result.cylinderAxis));
  result.cylinderCenterM = add(fk.footOriginM, rotate(fk.baseFromFootRotation, m.centerInFootM));
  const double component = dot(result.cylinderAxis, normal);
  const Vector3 perpendicular = add(normal, scale(result.cylinderAxis, -component));
  const double perpendicularNorm = norm(perpendicular);
  if (perpendicularNorm <= 1e-9) {
    result.status = ContactStatus::DEGENERATE;
    return result;
  }
  result.radialDown = scale(perpendicular, -1.0/perpendicularNorm);
  result.referenceM = add(result.cylinderCenterM, scale(result.radialDown, m.radiusM));
  const Vector3 halfStrip = scale(result.cylinderAxis, m.supportWidthM/2.0);
  result.stripEndAM = add(result.referenceM, scale(halfStrip, -1.0));
  result.stripEndBM = add(result.referenceM, halfStrip);
  result.lowestCoreM = dot(normal, result.stripEndAM) <= dot(normal, result.stripEndBM)
                      ? result.stripEndAM : result.stripEndBM;
  result.axisTiltRad = std::asin(std::fmin(1.0, std::abs(component)));
  result.support = result.axisTiltRad <= m.nominalTiltRad
                   ? SupportMode::NOMINAL_STRIP : SupportMode::EDGE_BIASED;
  return result;
}

ContactIkResult contactInverseKinematics(LegId leg, const Vector3& target,
                                          const ContactIkOptions& options) {
  ContactIkResult result;
  const auto* original = legModel(leg);
  if (!original) return result;
  // +Z ground and cos(h)>0 imply radialDown = Rx(h)*(0,0,-1).
  // Center is Rfoot*(0,0,r): use distal Z+r and separate hip-frame Z=-r.
  // This is a virtual analytic chain, never a changed URDF/foot-link origin.
  LegModel contactModel = *original;
  const double radius = footContactModel().radiusM;
  contactModel.footOrigin.z += radius;
  detail::AnalyticBranch branch;
  const auto ik = detail::inverseWithModel(contactModel, target, options.kinematics,
                                           -radius, &branch);
  result.status = convert(ik.status);
  result.reachability = ik.reachability;
  if (ik.status != KinematicsStatus::OK) return result;
  const auto contact = contactForwardKinematics(leg, ik.joints);
  const double residual = norm(add(target, scale(contact.referenceM, -1.0)));
  // Independent full orientation/contact evaluation verifies the reduction.
  if (contact.status != ContactStatus::OK || !std::isfinite(residual) ||
      residual > options.kinematics.toleranceM) {
    result.status = ContactStatus::NUMERICAL_FAILURE;
    result.reachability = Reachability::UNKNOWN;
    return result;
  }
  if (options.requireNominalStrip && contact.support != SupportMode::NOMINAL_STRIP) {
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
} }
