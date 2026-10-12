#include "MotionFeedback.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool positive(double x){return std::isfinite(x)&&x>0;}
bool evidence(EvidenceSource s,FeedbackMode m){return s==EvidenceSource::EXTERNAL_MEASURED||(m==FeedbackMode::SIMULATION&&s==EvidenceSource::SYNTHETIC);}
void components(LegJointAngles q,double a[3]){a[0]=q.hip;a[1]=q.upper;a[2]=q.lower;}
}
const char* feedbackReasonName(FeedbackReason r){
  const char* names[]={"OK","NOT_REQUESTED","INVALID_POLICY","INVALID_COMMAND","STALE_OBSERVATION",
  "UNOBSERVABLE_HEIGHT","UNOBSERVABLE_SUPPORT","INVALID_POSE","UNRECOVERABLE_POSE","ROUTE_REQUIRED",
  "ROUTE_MISMATCH","IK_FAILURE","BRANCH_CHANGE","JOINT_LIMIT","CONTINUITY","VELOCITY_LIMIT",
  "ACCELERATION_LIMIT","ASSESSMENT_REQUIRED","ASSESSMENT_STALE","COLLISION","SUPPORT_INVALID",
  "CONTINUOUS_BOUNDS_MISSING","FEEDBACK_PENDING","TRACKING_ERROR","SEGMENT_TIMEOUT","INHIBITED",
  "FAULT","VERIFY_PENDING","COMMAND_EXPIRED","SEQUENCE_REPLAY","STOPPED","FOOTPRINT_MISMATCH"};
  const unsigned i=static_cast<unsigned>(r);return i<sizeof(names)/sizeof(names[0])?names[i]:"UNKNOWN";
}
bool validFeedbackPolicy(const FeedbackPolicy& p){
  if(!p.envelopesApproved||!positive(p.maxJointStepRad)||!positive(p.maxTrackingErrorRad)||
    !positive(p.observationMaxAgeS)||!positive(p.assessmentMaxAgeS)||!positive(p.segmentTimeoutS)||
    !positive(p.heightToleranceM)||!positive(p.contactToleranceM)||!positive(p.samplePeriodS))return false;
  if(p.mode!=FeedbackMode::SIMULATION&&p.mode!=FeedbackMode::PHYSICAL_REVIEW)return false;
  if(p.mode==FeedbackMode::PHYSICAL_REVIEW&&!p.loadedRatesVerified)return false;
  for(unsigned i=0;i<4;++i){double v[3],a[3];components(p.maxVelocityRadS[i],v);components(p.maxAccelerationRadS2[i],a);
    const auto* m=legModel(static_cast<LegId>(i));
    for(unsigned j=0;j<3;++j)if(!positive(v[j])||!positive(a[j])||
      !std::isfinite(p.jointLimits[i][j].lower)||!std::isfinite(p.jointLimits[i][j].upper)||
      p.jointLimits[i][j].lower>=p.jointLimits[i][j].upper||p.jointLimits[i][j].lower<m->limits[j].lower||
      p.jointLimits[i][j].upper>m->limits[j].upper)return false;}
  return true;
}
FeedbackReason checkObservation(const AdaptiveObservation& o,const FeedbackPolicy& p,double now){
  if(o.fault)return FeedbackReason::FAULT;
  if(o.inhibited)return FeedbackReason::INHIBITED;
  if(!std::isfinite(now)||!std::isfinite(o.stampS)||o.stampS>now||now-o.stampS>p.observationMaxAgeS)return FeedbackReason::STALE_OBSERVATION;
  if(!evidence(o.heightSource,p.mode))return FeedbackReason::UNOBSERVABLE_HEIGHT;
  if(!evidence(o.supportSource,p.mode)||!o.flatPlaneConfirmed||
    (!o.bellySupported&&!o.externalSupportConfirmed&&o.confirmedFeet==0)||o.confirmedFeet>15)return FeedbackReason::UNOBSERVABLE_SUPPORT;
  if(!o.actual.valid||!supportsFlatContactIk(o.body)||o.body.translationM.z<0)return FeedbackReason::INVALID_POSE;
  for(unsigned i=0;i<4;++i){double q[3];components(o.actual.legs[i],q);
    if(forwardKinematics(static_cast<LegId>(i),o.actual.legs[i]).status!=KinematicsStatus::OK)return FeedbackReason::JOINT_LIMIT;
    for(unsigned j=0;j<3;++j)if(q[j]<p.jointLimits[i][j].lower||q[j]>p.jointLimits[i][j].upper)return FeedbackReason::JOINT_LIMIT;}
  return FeedbackReason::OK;
}
FeedbackReason checkSegment(const LocomotionFrame& f,const JointTargetFrame& previous,const FeedbackPolicy& p){
  if(!f.target.valid)return f.status==GaitStatus::BRANCH_CHANGE?FeedbackReason::BRANCH_CHANGE:FeedbackReason::IK_FAILURE;
  for(unsigned i=0;i<4;++i){double q[3],old[3],v[3],a[3],vl[3],al[3];components(f.target.legs[i],q);components(previous.legs[i],old);
    components(f.velocityRadS[i],v);components(f.accelerationRadS2[i],a);components(p.maxVelocityRadS[i],vl);components(p.maxAccelerationRadS2[i],al);
    for(unsigned j=0;j<3;++j){if(!std::isfinite(q[j])||q[j]<p.jointLimits[i][j].lower||q[j]>p.jointLimits[i][j].upper)return FeedbackReason::JOINT_LIMIT;
      if(std::abs(q[j]-old[j])>p.maxJointStepRad)return FeedbackReason::CONTINUITY;
      if(!std::isfinite(v[j])||std::abs(v[j])>vl[j])return FeedbackReason::VELOCITY_LIMIT;
      if(!std::isfinite(a[j])||std::abs(a[j])>al[j])return FeedbackReason::ACCELERATION_LIMIT;}}
  return FeedbackReason::OK;
}
FeedbackReason checkAssessment(const SegmentAssessment& a,uint32_t sequence,const FeedbackPolicy& p,double now){
  if(!std::isfinite(now)||a.sequence!=sequence||!std::isfinite(a.stampS)||a.stampS>now||now-a.stampS>p.assessmentMaxAgeS)return FeedbackReason::ASSESSMENT_STALE;
  if(!evidence(a.source,p.mode)||!a.geometryIdentityVerified)return FeedbackReason::ASSESSMENT_REQUIRED;
  if(!a.sweptCollisionClear)return FeedbackReason::COLLISION;
  if(!a.supportCompatible)return FeedbackReason::SUPPORT_INVALID;
  if(!a.continuousBoundsConfirmed)return FeedbackReason::CONTINUOUS_BOUNDS_MISSING;
  if(p.mode==FeedbackMode::PHYSICAL_REVIEW&&!a.loadedTrackingQualified)return FeedbackReason::ASSESSMENT_REQUIRED;
  return FeedbackReason::OK;
}
FeedbackReason checkTracking(const AdaptiveObservation& o,const FeedbackFrame& f,const FeedbackPolicy& p){
  if(!o.acknowledged||o.acknowledgedSequence!=f.target.sequence||
     o.stampS+1e-9<f.timeS+p.samplePeriodS)return FeedbackReason::FEEDBACK_PENDING;
  for(unsigned i=0;i<4;++i){double actual[3],target[3];components(o.actual.legs[i],actual);components(f.target.legs[i],target);
    for(unsigned j=0;j<3;++j)if(std::abs(actual[j]-target[j])>p.maxTrackingErrorRad)return FeedbackReason::TRACKING_ERROR;}
  if(std::abs(o.body.translationM.z-f.body.translationM.z)>p.heightToleranceM||
     std::hypot(o.body.translationM.x-f.body.translationM.x,o.body.translationM.y-f.body.translationM.y)>p.contactToleranceM)return FeedbackReason::TRACKING_ERROR;
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)
    if(std::abs(o.body.rotation[i][j]-f.body.rotation[i][j])>p.maxTrackingErrorRad)return FeedbackReason::TRACKING_ERROR;
  for(unsigned i=0;i<4;++i)if(f.requiredFeet&(1<<i)){
    const auto fk=contactForwardKinematics(static_cast<LegId>(i),o.actual.legs[i]);Vector3 world;
    if(fk.status!=ContactStatus::OK||fk.support!=SupportMode::NOMINAL_STRIP||!pointToWorld(o.body,fk.referenceM,world)||
       std::abs(world.z)>p.contactToleranceM)return FeedbackReason::SUPPORT_INVALID;
    if(std::hypot(world.x-f.feetWorldM[i].x,world.y-f.feetWorldM[i].y)>p.contactToleranceM)return FeedbackReason::TRACKING_ERROR;
  }
  if((o.confirmedFeet&f.requiredFeet)!=f.requiredFeet||(f.bellySupportRequired&&!o.bellySupported)||
     (f.externalSupportRequired&&!o.externalSupportConfirmed))return FeedbackReason::SUPPORT_INVALID;
  return FeedbackReason::OK;
}
FeedbackReason checkHeldSupport(const AdaptiveObservation& o,const LocomotionFrame& previous,const FeedbackFrame& pending){
  uint8_t previousFeet=0;
  for(unsigned i=0;i<4;++i)if(previous.cartesian.phases[i].stance)previousFeet|=1<<i;
  const uint8_t held=previousFeet&pending.requiredFeet;
  if((o.confirmedFeet&held)!=held||(pending.bellySupportRequired&&!o.bellySupported)||
     (pending.externalSupportRequired&&!o.externalSupportConfirmed))return FeedbackReason::SUPPORT_INVALID;
  return FeedbackReason::OK;
}
LocomotionFrame observedSolution(const AdaptiveObservation& o){
  LocomotionFrame f;f.cartesian.body=o.body;f.cartesian.valid=true;
  for(unsigned i=0;i<4;++i){const auto leg=static_cast<LegId>(i);const auto fk=contactForwardKinematics(leg,o.actual.legs[i]);
    if(fk.status!=ContactStatus::OK||!pointToWorld(o.body,fk.referenceM,f.cartesian.feet[i]))return {};
    ContactIkOptions opts;opts.requireNominalStrip=false;opts.kinematics.seed=o.actual.legs[i];
    auto ik=worldContactInverseKinematics(leg,f.cartesian.feet[i],o.body,opts);
    if(ik.status!=WorldContactIkResult::Status::OK)return {};
    const auto reconstructed=ik.baseResult.joints,actual=o.actual.legs[i];
    if(std::abs(reconstructed.hip-actual.hip)>1e-7||std::abs(reconstructed.upper-actual.upper)>1e-7||
       std::abs(reconstructed.lower-actual.lower)>1e-7)return {};
    f.branches[i]=ik.baseResult.branch;f.target.legs[i]=o.actual.legs[i];f.cartesian.phases[i].stance=(o.confirmedFeet&(1<<i))!=0;}
  f.target.valid=true;f.status=GaitStatus::OK;return f;
}
FeedbackFrame feedbackFrame(const LocomotionFrame& f,uint32_t seq){
  FeedbackFrame out;out.target=f.target;out.target.sequence=seq;out.body=f.cartesian.body;
  for(unsigned i=0;i<4;++i){out.feetWorldM[i]=f.cartesian.feet[i];out.phases[i]=f.cartesian.phases[i];
    out.velocityRadS[i]=f.velocityRadS[i];out.accelerationRadS2[i]=f.accelerationRadS2[i];
    if(f.cartesian.phases[i].stance)out.requiredFeet|=1<<i;}
  out.reason=FeedbackReason::ASSESSMENT_REQUIRED;return out;
}
} }
