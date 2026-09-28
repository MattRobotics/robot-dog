#include "StartupAcquisition.h"
#include <cmath>
namespace matdog { namespace motion {
AcquisitionResult evaluateStartup(const StartupObservation& o) {
  AcquisitionResult result;
  switch(o.condition) {
    case StartupCondition::SUSPENDED_REFERENCE:
      result.status=AcquisitionStatus::SUSPENDED_PATH_UNPROVEN;
      result.support=SupportRegime::EXTERNAL_SUSPENSION;return result;
    case StartupCondition::FLOOR_CONTACT_UNVERIFIED:
      result.status=AcquisitionStatus::FLOOR_ACQUISITION_UNPROVEN;
      result.support=SupportRegime::UNVERIFIED_FLOOR;return result;
    case StartupCondition::VERIFIED_LOW_STANCE:break;
    default:return result;
  }
  result.support=SupportRegime::UNVERIFIED_FLOOR;
  if(!o.fresh||!o.stationary||!o.fourFootSupportConfirmed||!o.flatGroundConfirmed||
     !o.collisionClearanceReviewed||!o.supportAndLoadReviewed) {
    result.status=AcquisitionStatus::MISSING_EVIDENCE;return result;
  }
  if(!o.joints.valid||!validBodyPose(o.body)) {result.status=AcquisitionStatus::INVALID_OBSERVATION;return result;}
  const auto& d=canonicalStandDefinition();
  const auto p=o.body.translationM;
  if(std::abs(p.x)>1e-9||std::abs(p.y)>1e-9||std::abs(p.z-d.lowBodyHeightM)>1e-9) {
    result.status=AcquisitionStatus::BODY_POSE_MISMATCH;return result;
  }
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)
    if(std::abs(o.body.rotation[i][j]-(i==j?1.0:0.0))>1e-10) {
      result.status=AcquisitionStatus::BODY_POSE_MISMATCH;return result;
    }
  const auto stand=generateStandTarget(d);
  const auto low=sampleStandPath(d,0.0,stand.target.legs);
  if(!stand.target.valid||!low.target.valid) {result.status=AcquisitionStatus::REFERENCE_FAILURE;return result;}
  for(unsigned i=0;i<4;++i) {
    const auto leg=static_cast<LegId>(i);
    const auto c=contactForwardKinematics(leg,o.joints.legs[i]);
    if(c.status!=ContactStatus::OK) {
      result.status=c.status==ContactStatus::JOINT_LIMIT?AcquisitionStatus::JOINT_LIMIT:AcquisitionStatus::INVALID_OBSERVATION;
      return result;
    }
    const auto q=o.joints.legs[i], expected=low.target.legs[i];
    if(std::abs(q.hip-expected.hip)>1e-8||std::abs(q.upper-expected.upper)>1e-8||
       std::abs(q.lower-expected.lower)>1e-8) {result.status=AcquisitionStatus::LOW_STANCE_MISMATCH;return result;}
    Vector3 world;
    if(!pointToWorld(o.body,c.referenceM,world)||c.support!=SupportMode::NOMINAL_STRIP||
       std::hypot(std::hypot(world.x-d.contactsWorldM[i].x,world.y-d.contactsWorldM[i].y),world.z)>1e-9) {
      result.status=AcquisitionStatus::CONTACT_MISMATCH;return result;
    }
  }
  result.status=AcquisitionStatus::READY;
  result.support=SupportRegime::FOUR_CONTACT_LOCKED;
  result.lowStance=low;
  return result;
}
} }
