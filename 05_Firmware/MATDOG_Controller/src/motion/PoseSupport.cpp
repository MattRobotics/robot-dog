#include "PoseSupport.h"
#include "PoseReferenceData.h"
#include <cmath>
namespace matdog { namespace motion {
bool validContactPolicy(const PoseContactPolicy& p) {
  if (p.contacts.feet > 15) return false;
  switch (p.regime) {
    case PoseSupportRegime::FOOT_SUPPORT: return !p.contacts.base && p.contacts.feet != 0;
    case PoseSupportRegime::BODY_SUPPORT: return p.contacts.base && p.contacts.feet == 0;
    case PoseSupportRegime::BODY_AND_FOOT_SUPPORT: return p.contacts.base && p.contacts.feet != 0;
    case PoseSupportRegime::FREE_SPACE: return !p.contacts.base && p.contacts.feet == 0;
    case PoseSupportRegime::TRANSITIONAL_SUPPORT: return p.contacts.base || p.contacts.feet != 0;
    default: return false;
  }
}
GroundContactVerdict classifyGroundContact(const PoseContactPolicy& p, GroundLinkClass kind,
                                         LegId leg, double z, double tolerance) {
  if (!validContactPolicy(p) || !std::isfinite(z) || !std::isfinite(tolerance) ||
      tolerance < 0 || static_cast<unsigned>(kind)>2 ||
      (kind==GroundLinkClass::FOOT && static_cast<unsigned>(leg)>3))
    return GroundContactVerdict::INVALID_INPUT;
  if (z < -tolerance) return GroundContactVerdict::PENETRATION;
  if (z > tolerance) return GroundContactVerdict::CLEAR;
  const bool allowed = (kind==GroundLinkClass::BASE && p.contacts.base) ||
      (kind==GroundLinkClass::FOOT && (p.contacts.feet & (1u<<static_cast<unsigned>(leg))));
  return allowed ? GroundContactVerdict::INTENTIONAL_SUPPORT : GroundContactVerdict::FORBIDDEN_CONTACT;
}
const SemanticPose* researchedPose(uint8_t index) {
  return index < generated::kPoseCount ? &generated::kPoses[index] : nullptr;
}
uint8_t researchedPoseCount() { return generated::kPoseCount; }
const char* poseSourceDigest() { return generated::kPoseSourceDigest; }
} }
