#include "../../src/motion/PoseSupport.h"
#include "../../src/motion/StartupAcquisition.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
using namespace matdog::motion;
int main() {
  unsigned combinations=0;
  for (unsigned regime=0; regime<7; ++regime) {
    for (unsigned base=0; base<2; ++base) for (unsigned feet=0; feet<32; ++feet) {
      PoseContactPolicy p{static_cast<PoseSupportRegime>(regime),{base!=0,static_cast<uint8_t>(feet)}};
      bool expected=feet<16 && ((regime==0 && !base && feet) ||
        (regime==1 && base && !feet) || (regime==2 && base && feet) ||
        (regime==3 && !base && !feet) || (regime==4 && (base || feet)));
      assert(validContactPolicy(p)==expected); ++combinations;
      if (!expected) {
        assert(classifyGroundContact(p,GroundLinkClass::BASE,LegId::LF,0,1e-6)==GroundContactVerdict::INVALID_INPUT);
        continue;
      }
      for (unsigned kind=0; kind<3; ++kind) for (unsigned leg=0; leg<4; ++leg) {
        const auto k=static_cast<GroundLinkClass>(kind); const auto l=static_cast<LegId>(leg);
        const bool allowed=(kind==0 && base) || (kind==1 && (feet&(1u<<leg)));
        assert(classifyGroundContact(p,k,l,0,1e-6)==(allowed?GroundContactVerdict::INTENTIONAL_SUPPORT:GroundContactVerdict::FORBIDDEN_CONTACT));
        assert(classifyGroundContact(p,k,l,-2e-6,1e-6)==GroundContactVerdict::PENETRATION);
        assert(classifyGroundContact(p,k,l,2e-6,1e-6)==GroundContactVerdict::CLEAR);
      }
    }
  }
  PoseContactPolicy p{PoseSupportRegime::BODY_SUPPORT,{true,0}};
  assert(classifyGroundContact(p,GroundLinkClass::FOOT,static_cast<LegId>(9),0,0)==GroundContactVerdict::INVALID_INPUT);
  assert(classifyGroundContact(p,static_cast<GroundLinkClass>(9),LegId::LF,0,0)==GroundContactVerdict::INVALID_INPUT);
  for (double bad : {std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    assert(classifyGroundContact(p,GroundLinkClass::BASE,LegId::LF,bad,0)==GroundContactVerdict::INVALID_INPUT);
    assert(classifyGroundContact(p,GroundLinkClass::BASE,LegId::LF,0,bad)==GroundContactVerdict::INVALID_INPUT);
  }
  assert(classifyGroundContact(p,GroundLinkClass::BASE,LegId::LF,0,-1)==GroundContactVerdict::INVALID_INPUT);
  assert(researchedPoseCount()==6 && researchedPose(6)==nullptr && researchedPose(255)==nullptr);
  assert(poseSourceDigest()[0]!='\0');
  for (uint8_t i=0;i<researchedPoseCount();++i) {
    const auto& pose=*researchedPose(i);
    assert(validBodyPose(pose.body) && validContactPolicy(pose.support));
    assert(pose.joints.valid && pose.evidence==PoseEvidence::VALIDATED_STATIC);
    assert(pose.supportMarginM>0 && pose.jointLimitMarginRad>=-1e-12);
    for (unsigned leg=0;leg<4;++leg)
      assert(forwardKinematics(static_cast<LegId>(leg),pose.joints.legs[leg]).status==KinematicsStatus::OK);
    StartupObservation o{};o.condition=StartupCondition::FLOOR_CONTACT_UNVERIFIED;
    o.body=pose.body;o.joints=pose.joints;o.fresh=o.stationary=true;
    o.fourFootSupportConfirmed=o.flatGroundConfirmed=o.collisionClearanceReviewed=o.supportAndLoadReviewed=true;
    assert(evaluateStartup(o).status==AcquisitionStatus::FLOOR_ACQUISITION_UNPROVEN);
  }
  assert(std::abs(legModel(LegId::LF)->hipOrigin.z-legModel(LegId::RH)->hipOrigin.z-.020)<1e-14);
  std::printf("POSE_SUPPORT = PASS: %u policy combinations; ground classes, invalid inputs, 6 static targets, unchanged startup gate\n",combinations);
}
