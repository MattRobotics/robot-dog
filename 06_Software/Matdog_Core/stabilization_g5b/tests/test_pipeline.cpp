#include "Fixtures.h"
#include "InertialInput.h"
#include <cstdio>
#include <initializer_list>
#include <limits>
using namespace fixture;
static unsigned checks=0,failures=0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL line %d: %s\n",__LINE__,#x); } } while (0)
static bool near(double x,double y,double tolerance=1e-10) { return std::abs(x-y)<=tolerance; }
static void refused(const PipelineOutput& o, Reason r) {
  CHECK(o.reason==r); CHECK(!o.target.valid);
  for (const auto& q : o.target.legs) CHECK(q.hip==0 && q.upper==0 && q.lower==0);
  std::printf("FAULT_CASE %s target_valid=%d joints=ZERO latch=%d attitude=%u ik=%u correction_rad=(%.12g,%.12g) convergence=NA overshoot=NA\n",
    reasonName(o.reason),o.target.valid,o.faultLatched,static_cast<unsigned>(o.attitudeStatus),static_cast<unsigned>(o.ikStatus),
    o.correction.rollCorrectionRad,o.correction.pitchCorrectionRad);
}
static StabilizationPipeline armed(PipelineConfig c=config(),NominalPose n=nominal()) {
  StabilizationPipeline p; CHECK(p.configure(c)); CHECK(p.enable(n,0)); return p;
}
int main() {
  const auto n=nominal(); const auto c=config();
  { StabilizationPipeline p; CHECK(!p.configure(PipelineConfig{})); CHECK(!p.enable(n,0));
    refused(p.update(sample(0,0,0,1),n,support(0),0,0),Reason::NOT_CONFIGURED);
    auto bad=c; bad.stabilizer.integralGainPerS=maxStableIntegralGain(c.maxSampleGapS,2); CHECK(!p.configure(bad));
    bad=c; bad.bounds.evidence=Evidence::UNAVAILABLE; CHECK(!p.configure(bad));
    bad=c; bad.bounds.joints[0][0].lower=-1; CHECK(!p.configure(bad));
    bad=c; bad.referenceRevision=0; CHECK(!p.configure(bad));
    bad=c; bad.dampingTimeS=.1; CHECK(!p.configure(bad));
    bad=c; bad.maxIkIterations=41; CHECK(!p.configure(bad));
    CHECK(p.configure(c)); CHECK(!p.enable(NominalPose{},0)); CHECK(p.enable(n,0)); CHECK(!p.configure(c)); }
  { auto p=armed(); auto o=p.update(sample(0,0,.02,1),n,support(.02),.02,.02);
    CHECK(o.reason==Reason::OK && o.target.valid); CHECK(o.maxContactResidualM<1e-9); CHECK(o.minJointMarginRad>c.minJointMarginRad);
    CHECK(o.correction.rollCorrectionRad==0 && o.correction.pitchCorrectionRad==0); CHECK(!o.experimental);
    CHECK(o.inputSequence==1 && o.target.sequence==1 && o.validUntilS>.02);
    for (unsigned i=0;i<4;++i) { CHECK(near(o.target.legs[i].hip,n.anchor.legs[i].hip)); CHECK(near(o.target.legs[i].upper,n.anchor.legs[i].upper)); }
    InertialInput missing; CHECK(missing.readiness==InertialRevision::UNAVAILABLE); // pipeline did not require masses
  }
  for (double roll : {-.02,0.,.02}) for (double pitch : {-.02,0.,.02}) {
    auto p=armed(); auto o=p.update(sample(roll,pitch,.02,1),n,support(.02),.02,.02);
    CHECK(o.target.valid); CHECK(near(o.correction.rollCorrectionRad,-c.stabilizer.integralGainPerS*.02*roll));
    CHECK(near(o.correction.pitchCorrectionRad,-c.stabilizer.integralGainPerS*.02*pitch));
    CHECK(o.maxContactResidualM<c.ikToleranceM); CHECK(o.maxJointDeltaRad<=c.maxJointRateRadS*.02);
  }
  { auto cc=c; cc.stabilizer.rollReferenceRad=.012; cc.stabilizer.pitchReferenceRad=-.009;
    auto p=armed(cc); const auto o=p.update(sample(.012,-.009,.02,1),n,support(.02),.02,.02);
    CHECK(o.target.valid); CHECK(near(o.correction.rollCorrectionRad,0)); CHECK(near(o.correction.pitchCorrectionRad,0)); }
  { auto nn=n; const double yaw=.6; nn.pose.rotation[0][0]=std::cos(yaw); nn.pose.rotation[0][1]=-std::sin(yaw);
    nn.pose.rotation[1][0]=std::sin(yaw); nn.pose.rotation[1][1]=std::cos(yaw); nn.pose.translationM.x=.03; nn.pose.translationM.y=-.02;
    auto f=support(.02); for (auto& foot : f.contacts) {
      const double x=foot.worldM.x,y=foot.worldM.y; foot.worldM={.03+std::cos(yaw)*x-std::sin(yaw)*y,-.02+std::sin(yaw)*x+std::cos(yaw)*y,0};
    }
    auto p=armed(c,nn); auto o=p.update(sample(.01,-.01,.02,1),nn,f,.02,.02);
    CHECK(o.target.valid); CHECK(near(std::atan2(o.correctedPose.rotation[1][0],o.correctedPose.rotation[0][0]),yaw));
    CHECK(near(o.correctedPose.translationM.x,.03) && near(o.correctedPose.translationM.y,-.02)); CHECK(o.maxContactResidualM<1e-9);
  }
  { auto p=armed(); auto baseline=armed(); const auto s=sample(.02,0,.02,1);
    CHECK(p.update(s,n,support(.02),.02,.02).target.valid); CHECK(baseline.update(s,n,support(.02),.02,.02).target.valid);
    refused(p.update(s,n,support(.02),.03,.03),Reason::DUPLICATE_SAMPLE);
    const auto a=p.update(sample(.02,0,.04,2),n,support(.04,2),.04,.04);
    const auto b=baseline.update(sample(.02,0,.04,2),n,support(.04,2),.04,.04);
    CHECK(a.target.valid && b.target.valid && near(a.correction.rollCorrectionRad,b.correction.rollCorrectionRad));
    auto changed=sample(.025,0,.04,2); refused(p.update(changed,n,support(.04,2),.05,.05),Reason::INCOHERENT_SAMPLE);
  }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    const auto o=p.update(sample(0,0,.02,1),n,support(.12,2),.12,.12); refused(o,Reason::IMU_REJECTED); CHECK(o.attitudeStatus==AttitudeStatus::STALE); }
  { auto p=armed(); auto s=sample(0,0,.02,1); s.connected=false; refused(p.update(s,n,support(.02),.02,.02),Reason::DISCONNECTED); }
  { auto p=armed(); auto s=sample(0,0,.02,1); s.orientation.w=2;
    const auto o=p.update(s,n,support(.02),.02,.02); refused(o,Reason::IMU_REJECTED); CHECK(o.attitudeStatus==AttitudeStatus::BAD_QUATERNION_NORM); }
  { auto p=armed(); auto s=sample(0,0,.02,1); s.orientation.x=std::numeric_limits<double>::quiet_NaN();
    refused(p.update(s,n,support(.02),.02,.02),Reason::IMU_REJECTED); }
  { auto p=armed(); auto s=sample(0,0,.02,1); s.orientation.accuracyRad=-.01; refused(p.update(s,n,support(.02),.02,.02),Reason::INCOHERENT_SAMPLE); }
  { auto p=armed(); auto s=sample(0,0,.02,1); s.orientation.accuracyStatus=4; refused(p.update(s,n,support(.02),.02,.02),Reason::INCOHERENT_SAMPLE); }
  { auto p=armed(); auto s=sample(0,0,.02,1); s.orientation.accuracyStatus=1;
    auto o=p.update(s,n,support(.02),.02,.02); refused(o,Reason::IMU_REJECTED); CHECK(o.attitudeStatus==AttitudeStatus::ACCURACY_STATUS_LOW); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    refused(p.update(sample(0,0,.01,2),n,support(.01,2),.01,.01),Reason::INVALID_TIME); }
  { auto p=armed(); auto s=sample(0,0,.02,1); CHECK(p.update(s,n,support(.02),.02,.02).target.valid);
    auto f=support(.02); f.contacts[0].state=ContactState::UNKNOWN;
    refused(p.update(s,n,f,.03,.03),Reason::UNCONFIRMED_CONTACT); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    auto f=support(.04,1); refused(p.update(sample(0,0,.04,2),n,f,.04,.04),Reason::SUPPORT_ORDER); }
  { auto cc=c; cc.allowMovingPlaneExperiment=true; auto p=armed(cc);
    CHECK(p.update(sample(0,0,.02,1),n,moving(.02,1,0,0),.02,.02).target.valid);
    refused(p.update(sample(0,0,.04,2),n,moving(.04,2,0,.01),.04,.04),Reason::PLANE_MOTION); }
  { StabilizationPipeline p; CHECK(p.configure(c)); auto nn=n; nn.anchor.legs[0].hip=c.bounds.joints[0][0].upper; CHECK(!p.enable(nn,0)); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,2),n,support(.02),.02,.02).target.valid);
    auto o=p.update(sample(0,0,.03,1),n,support(.03,2),.03,.03); refused(o,Reason::IMU_REJECTED); CHECK(o.attitudeStatus==AttitudeStatus::SEQUENCE_REGRESSION); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    refused(p.update(sample(0,0,.02,2),n,support(.03,2),.03,.03),Reason::INCOHERENT_SAMPLE); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    auto s=sample(0,0,.04,2); s.acquisitionEpoch=2; refused(p.update(s,n,support(.04,2),.04,.04),Reason::EPOCH_CHANGED); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    refused(p.update(sample(0,0,.09,4),n,support(.09,4),.09,.09),Reason::SAMPLE_GAP); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    const auto o=p.update(sample(0,0,.06,4),n,support(.06,4),.065,.075);
    CHECK(o.target.valid && o.skippedSamples==2 && near(o.measurementAgeS,.005) && near(o.measurementToCommandS,.015)); }
  { auto p=armed(); refused(p.update(sample(0,0,.02,1),n,support(.02),.02,.05),Reason::COMMAND_TOO_LATE); }
  { auto p=armed(); refused(p.update(sample(0,0,.04,1),n,support(.02),.02,.02),Reason::IMU_REJECTED); }
  { auto p=armed(); auto s=support(.02); s.contacts[2].state=ContactState::UNKNOWN;
    refused(p.update(sample(0,0,.02,1),n,s,.02,.02),Reason::UNCONFIRMED_CONTACT); }
  { auto p=armed(); auto s=support(.02); s.contacts[0].confidence=.5;
    refused(p.update(sample(0,0,.02,1),n,s,.02,.02),Reason::UNCONFIRMED_CONTACT); }
  { auto p=armed(); refused(p.update(sample(0,0,.02,1),n,moving(.02,1,.001,0),.02,.02),Reason::PLANE_GATE); }
  { auto p=armed(); auto s=support(.02); s.normalWorld={0,0,0}; refused(p.update(sample(0,0,.02,1),n,s,.02,.02),Reason::INVALID_SUPPORT); }
  { auto p=armed(); CHECK(p.update(sample(0,0,.02,1),n,support(.02),.02,.02).target.valid);
    auto s=support(.04,2); s.contacts[0].worldM.x+=.001; refused(p.update(sample(0,0,.04,2),n,s,.04,.04),Reason::PLANE_MOTION); }
  { auto p=armed(); auto far=support(.02); far.contacts[3].worldM.x+=1;
    refused(p.update(sample(0,0,.02,1),n,far,.02,.02),Reason::IK_FAILURE); }
  { auto cc=c; cc.bounds.joints[0][1].upper=n.anchor.legs[0].upper+.0011;
    auto p=armed(cc); refused(p.update(sample(0,-.04,.02,1),n,support(.02),.02,.02),Reason::OPERATIONAL_LIMIT); }
  { auto cc=c; cc.maxJointStepRad=1e-7; auto p=armed(cc);
    refused(p.update(sample(.02,0,.02,1),n,support(.02),.02,.02),Reason::TARGET_DISCONTINUITY); }
  { auto cc=c; cc.maxIkCondition=1; auto p=armed(cc);
    refused(p.update(sample(.02,0,.02,1),n,support(.02),.02,.02),Reason::CONDITION_LIMIT); }
  { auto p=armed(); double t=0; PipelineOutput o;
    for (unsigned i=1;i<=200;++i) { t=.02*i; o=p.update(sample(0,.3,t,i),n,support(t,i),t,t); CHECK(o.target.valid); }
    CHECK(o.correction.saturated); CHECK(near(o.correction.pitchCorrectionRad,-c.stabilizer.maxCorrectionRad));
    const double held=o.correction.pitchCorrectionRad;
    auto bad=sample(0,0,t+.02,201); bad.connected=false; refused(p.update(bad,n,support(t+.02,201),t+.02,t+.02),Reason::DISCONNECTED);
    CHECK(p.faultLatched()); CHECK(!p.enable(n,t+.02)); CHECK(!p.resetFault()); CHECK(!p.configure(c));
    refused(p.update(sample(0,0,t+.04,202),n,support(t+.04,202),t+.04,t+.04),Reason::FAULT_LATCHED);
    p.disable(); CHECK(!p.resetFault());
    for (unsigned i=3;i<60;++i) { o=p.update(sample(0,0,t+.02*i,200+i),n,support(t+.02*i,200+i),t+.02*i,t+.02*i); CHECK(!o.target.valid); }
    CHECK(held!=0 && o.correction.pitchCorrectionRad==0); CHECK(p.resetFault());
    CHECK(p.enable(n,t+1.2)); o=p.update(sample(0,0,t+1.22,1),n,support(t+1.22),t+1.22,t+1.22); CHECK(o.target.valid); }
  { auto p=armed(); CHECK(p.update(sample(.01,0,.02,1),n,support(.02),.02,.02).target.valid); p.disable();
    refused(p.update(sample(0,0,.04,2),n,support(.04,2),.04,.04),Reason::DISABLED); CHECK(p.enable(n,.04)); }
  { auto cc=c; cc.allowMovingPlaneExperiment=true; auto p=armed(cc); double maxResidual=0;
    for (unsigned i=1;i<=150;++i) {
      const double t=.02*i; const auto o=p.update(sample(0,0,t,i),n,moving(t,i,.00004*i,.0001*i),t,t);
      CHECK(o.target.valid && o.experimental); maxResidual=std::fmax(maxResidual,o.maxContactResidualM);
    }
    CHECK(maxResidual<cc.ikToleranceM);
    std::printf("MOVING_PLANE roll_final_rad=.006 pitch_final_rad=.015 max_contact_residual_m=%.12g\n",maxResidual); }
  { auto cc=c; cc.allowDampingExperiment=true; cc.dampingTimeS=.04; cc.maxTiltRateRadS=1;
    auto p=armed(cc); const TiltRateSample rate{.1,0,.02,1,true};
    auto o=p.update(sample(.02,0,.02,1),n,support(.02),.02,.02,rate);
    CHECK(o.target.valid && o.experimental); CHECK(near(o.correction.rollCorrectionRad,-.02*(.02+.04*.1)));
    refused(p.update(sample(.02,0,.04,2),n,support(.04,2),.04,.04,rate),Reason::INVALID_RATE); }
  std::printf("G5B_PIPELINE checks=%u failures=%u = %s\n",checks,failures,failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
