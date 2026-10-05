#ifndef MATDOG_MOTION_TILTED_CONTACT_IK_H
#define MATDOG_MOTION_TILTED_CONTACT_IK_H
#include "BodyPose.h"
namespace matdog { namespace motion {
// G5-A explicit extension of G2 contact IK to a ground normal that is NOT +Z in base_link (level-body assumption
// removed). It is a NEW solver beside the accepted analytic one: contactInverseKinematics / worldContactInverseKinematics
// and supportsFlatContactIk are unchanged. It reuses contactForwardKinematics (which already accepts a ground normal) as
// the single contact model, so the G2 cylinder, strip and nominal-tilt policy are identical.
//
// Method: damped-free Newton iteration on the three joint angles with a central-difference Jacobian of
// contactForwardKinematics, started from the caller's seed (the previous valid solution). Fixed iteration bound, no heap,
// no clipping: a step that would leave the URDF limits is halved and, if still outside, rejected. The seed selects the
// branch exactly as the analytic solver's seed-nearest rule. For ground normal +Z the result agrees with the analytic IK.
struct TiltedIkOptions {
  LegJointAngles seed{};
  double toleranceM = 1e-9;
  unsigned maxIterations = 40;
  bool requireNominalStrip = true;
};
struct TiltedIkResult {
  enum class Status : uint8_t { OK, INVALID_INPUT, NO_CONVERGENCE, SINGULAR, JOINT_LIMIT, CONTACT_MODE };
  Status status = Status::INVALID_INPUT;
  LegJointAngles joints{};
  ContactResult contact{};
  double residualM = 0.0, condition = 0.0;
  unsigned iterations = 0;
};
// target and groundNormal are in base_link; the normal points away from the ground (up). Use joints only when status == OK.
TiltedIkResult contactInverseKinematicsGroundNormal(LegId leg, const Vector3& targetBaseM, const Vector3& groundNormalBase,
                                                    const TiltedIkOptions& options);
// Flat world ground (+Z) and any valid body pose: the ground normal in base_link is R^T e_z.
struct WorldTiltedIkResult { TiltedIkResult::Status status = TiltedIkResult::Status::INVALID_INPUT; TiltedIkResult base{}; Vector3 achievedWorldM{}; };
WorldTiltedIkResult worldContactInverseKinematicsTilted(LegId leg, const Vector3& worldTarget, const BodyPose& pose,
                                                       const TiltedIkOptions& options);
} }
#endif
