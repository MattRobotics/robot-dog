#ifndef MATDOG_MOTION_TILT_COMPENSATION_H
#define MATDOG_MOTION_TILT_COMPENSATION_H
#include "StandTrajectory.h"
#include "TiltedContactIk.h"
namespace matdog { namespace motion {
// Body roll/pitch compensation of the canonical STAND (G5-A). The correction acts on the desired BodyPose: the base_link
// rotation becomes R = Ry(pitch) Rx(roll) (yaw 0, translation unchanged), the four G2 world contacts stay exactly where
// they are (world-locked), the Cartesian foot targets are R^T (contact - t), and the joint targets come from the tilted
// contact IK. Joint angles are never offset directly.
//   roll  +r: left side up (rotation about +X),   pitch +p: nose down (rotation about +Y)   (MATDOG frame convention)
struct TiltCompensation {
  enum class Status : uint8_t { OK, INVALID_INPUT, IK_FAILURE };
  Status status = Status::INVALID_INPUT;
  uint8_t failedLeg = 255;
  TiltedIkResult::Status legStatus = TiltedIkResult::Status::OK;
  BodyPose pose{};
  JointTargetFrame target{};                // valid only for Status::OK (atomic over all 12 joints)
  Vector3 contactsWorldM[4]{};              // achieved contact references, world frame
  double maxContactDriftM = 0, minJointMarginRad = 0, maxCondition = 0;
  unsigned maxIterations = 0;
};
BodyPose bodyPoseFromRollPitch(const Vector3& translationM, double rollRad, double pitchRad);
// seeds: the previous valid joint solution per leg (branch selector); pass the stand seeds for a cold start.
TiltCompensation compensateStand(const StandDefinition& stand, double bodyHeightM, double rollRad, double pitchRad,
                                 const LegJointAngles seeds[4], bool requireNominalStrip = true);
} }
#endif
