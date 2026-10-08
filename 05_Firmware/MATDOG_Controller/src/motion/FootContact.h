#ifndef MATDOG_MOTION_FOOT_CONTACT_H
#define MATDOG_MOTION_FOOT_CONTACT_H
#include "LegKinematics.h"
namespace matdog { namespace motion {
struct FootContactModel {
  Vector3 centerInFootM, axisInFoot;
  double radiusM, treadWidthM, filletRadiusM, supportWidthM, nominalTiltRad;
};
const FootContactModel& footContactModel();
const char* contactGeometrySha256();
enum class ContactStatus : uint8_t {
  OK, INVALID_INPUT, JOINT_LIMIT, DEGENERATE,
  UNREACHABLE_GEOMETRY, UNREACHABLE_LIMITS, CONTACT_MODE, NUMERICAL_FAILURE
};
enum class SupportMode : uint8_t { UNKNOWN, NOMINAL_STRIP, EDGE_BIASED };
struct ContactResult {
  ContactStatus status = ContactStatus::INVALID_INPUT;
  SupportMode support = SupportMode::UNKNOWN;
  Vector3 cylinderCenterM{}, cylinderAxis{}, radialDown{};
  // Same continuous physical reference as Python/C4. On a tilted finite
  // cylinder the lowest support-strip endpoint is a different point.
  Vector3 referenceM{}, stripEndAM{}, stripEndBM{}, lowestCoreM{};
  double axisTiltRad = 0.0;
};
// Every output is in base_link. The supplied ground normal is normalized.
// No assumption that the reference equals foot_link or lies on a ground plane.
ContactResult contactForwardKinematics(LegId leg, const LegJointAngles& q,
                                      Vector3 groundNormalBase = {0.0, 0.0, 1.0});
struct ContactBranch { int8_t hip = 0; int8_t elbow = 0; };
struct ContactIkOptions {
  IkOptions kinematics{};
  bool requireNominalStrip = true;
};
struct ContactIkResult {
  ContactStatus status = ContactStatus::INVALID_INPUT;
  Reachability reachability = Reachability::UNKNOWN;
  LegJointAngles joints{};
  ContactResult contact{};
  double residualM = 0.0;
  ContactBranch branch{};
  uint8_t solutions = 0;
};
// Exact analytic contact IK for ground normal +Z in base_link only.
// This deliberate G2 boundary covers parallel-body C4-A/C4-C. Future tilted
// body/terrain IK requires an explicit new solver, never a silent approximation.
// As in Python, nominal-strip policy applies to the selected seed-nearest solution.
// Only status==OK makes joints usable; CONTACT_MODE is not an accepted target.
ContactIkResult contactInverseKinematics(LegId leg, const Vector3& targetBaseM,
                            const ContactIkOptions& options = ContactIkOptions{});
} }
#endif
