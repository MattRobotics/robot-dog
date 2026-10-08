#ifndef MATDOG_MOTION_POSE_SUPPORT_H
#define MATDOG_MOTION_POSE_SUPPORT_H
#include "BodyPose.h"
#include "StandTrajectory.h"
namespace matdog { namespace motion {
// Pose analysis policy only. This does not authorize lifecycle entry or motion.
enum class PoseSupportRegime : uint8_t {
  FOOT_SUPPORT, BODY_SUPPORT, BODY_AND_FOOT_SUPPORT, FREE_SPACE,
  TRANSITIONAL_SUPPORT, UNSUPPORTED
};
struct IntentionalContacts {
  bool base = false;
  uint8_t feet = 0;  // bits LF, RF, RH, LH; no device identifiers
};
struct PoseContactPolicy {
  PoseSupportRegime regime = PoseSupportRegime::UNSUPPORTED;
  IntentionalContacts contacts{};
};
enum class GroundLinkClass : uint8_t { BASE, FOOT, OTHER_LINK };
enum class GroundContactVerdict : uint8_t {
  CLEAR, INTENTIONAL_SUPPORT, FORBIDDEN_CONTACT, PENETRATION, INVALID_INPUT
};
bool validContactPolicy(const PoseContactPolicy& policy);
// Permitting support never permits penetration. Tolerance is an explicit
// geometric numerical parameter, not a compliance or actuator safety limit.
GroundContactVerdict classifyGroundContact(const PoseContactPolicy& policy,
    GroundLinkClass kind, LegId leg, double minimumZ, double toleranceM);
enum class PoseEvidence : uint8_t { VALIDATED_STATIC, CANDIDATE, RESEARCH_ONLY };
struct SemanticPose {
  const char* name;
  BodyPose body;
  JointTargetFrame joints;
  PoseContactPolicy support;
  PoseEvidence evidence;
  Vector3 cadUrdfComWorldM;
  double supportMarginM;
  double jointLimitMarginRad;
};
// Static pose records are not transitions or startup evidence.
const SemanticPose* researchedPose(uint8_t index);
uint8_t researchedPoseCount();
const char* poseSourceDigest();
} }
#endif
