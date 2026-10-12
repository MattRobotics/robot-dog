#include "FeedbackWalk.h"
#include "TimedStand.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool positive(double v){return std::isfinite(v)&&v>0;}
bool zero(const PlanarVelocityCommand& c){return c.vxMps==0&&c.vyMps==0&&c.yawRateRadS==0;}
Vector3 rotate(const BodyPose& b,Vector3 v){return {b.rotation[0][0]*v.x+b.rotation[0][1]*v.y,b.rotation[1][0]*v.x+b.rotation[1][1]*v.y,v.z};}
double distance(Vector3 a,Vector3 b){return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);}
}
FeedbackFrame FeedbackWalk::reject(FeedbackReason r){state_=FeedbackWalkState::REJECTED;reason_=r;pendingValid_=false;FeedbackFrame f;f.reason=r;return f;}
bool FeedbackWalk::validCommand(const PlanarVelocityCommand& c,double now) const {
  return std::isfinite(now)&&std::isfinite(c.stampS)&&c.stampS<=now&&positive(c.ttlS)&&c.ttlS<=specification_.maxCommandTtlS&&
    now-c.stampS<c.ttlS&&c.sequence>0&&std::isfinite(c.vxMps)&&std::isfinite(c.vyMps)&&std::isfinite(c.yawRateRadS)&&
    std::abs(c.vxMps)<=specification_.maxVxMps&&std::abs(c.vyMps)<=specification_.maxVyMps&&std::abs(c.yawRateRadS)<=specification_.maxYawRateRadS;
}
bool FeedbackWalk::start(const AdaptiveStand& stand,const AdaptiveObservation& o,const PlanarVelocityCommand& c,
                         const WalkSpecification& s,const FeedbackPolicy& p,double now){
  *this=FeedbackWalk{};policy_=p;specification_=s;lastNow_=now;targetSequence_=s.firstTargetSequence;
  if(!validFeedbackPolicy(p)){reject(FeedbackReason::INVALID_POLICY);return false;}
  if(stand.state()!=AdaptiveStandState::STAND||!positive(s.periodS)||!positive(s.rampDurationS)||!positive(s.liftM)||
    !positive(s.maxVxMps)||!positive(s.maxVyMps)||!positive(s.maxYawRateRadS)||!positive(s.maxCommandTtlS)||s.firstTargetSequence==0||
    !validCommand(c,now)||zero(c)){reject(FeedbackReason::INVALID_COMMAND);return false;}
  auto r=checkObservation(o,p,now);if(r!=FeedbackReason::OK){reject(r);return false;}
  if(o.confirmedFeet!=15||o.bellySupported||!o.stationary){reject(FeedbackReason::SUPPORT_INVALID);return false;}
  previous_=observedSolution(o);if(!previous_.target.valid){reject(FeedbackReason::IK_FAILURE);return false;}
  origin_=o.body;origin_.translationM.z=0;
  const auto& d=canonicalStandDefinition();
  for(unsigned i=0;i<4;++i){Vector3 expected;pointToWorld(origin_,d.contactsWorldM[i],expected);
    if(distance(expected,previous_.cartesian.feet[i])>p.contactToleranceM){reject(FeedbackReason::FOOTPRINT_MISMATCH);return false;}}
  gait_.type=GaitType::WALK;gait_.heightM=o.body.translationM.z;gait_.liftM=s.liftM;gait_.duty=s.duty;
  gait_.advanceXM=c.vxMps*s.periodS;gait_.advanceYM=c.vyMps*s.periodS;gait_.yawRad=c.yawRateRadS*s.periodS;
  if(!validGaitParameters(gait_)){reject(FeedbackReason::INVALID_COMMAND);return false;}
  command_=c;state_=FeedbackWalkState::STARTING;reason_=FeedbackReason::OK;return true;
}
bool FeedbackWalk::command(const PlanarVelocityCommand& c,double now){
  if(state_==FeedbackWalkState::REJECTED||state_==FeedbackWalkState::IDLE||state_==FeedbackWalkState::STAND)return false;
  if(!validCommand(c,now)||now<lastNow_||c.stampS<command_.stampS){reason_=FeedbackReason::INVALID_COMMAND;return false;}
  if(c.sequence<=command_.sequence){reason_=FeedbackReason::SEQUENCE_REPLAY;return false;}
  if(zero(c)){command_=c;requestStop();return true;}
  // Parameter changes require the present support cycle to finish. A future
  // start uses the new command; never rewrite anchors under a lifted foot.
  if(c.vxMps!=command_.vxMps||c.vyMps!=command_.vyMps||c.yawRateRadS!=command_.yawRateRadS){requestStop();reason_=FeedbackReason::INVALID_COMMAND;return false;}
  command_=c;return true;
}
void FeedbackWalk::requestStop(){
  if(state_==FeedbackWalkState::STARTING||state_==FeedbackWalkState::WALK){
    stopCycle_=std::fmax(1.,std::ceil(cycles_));
    // Wait for an all-foot integer phase before changing touchdown anchors.
    // Integrate smoothstep deceleration for exactly one further cycle.
    stopClockS_=stopCycle_*specification_.periodS+.5*specification_.rampDurationS;
    state_=FeedbackWalkState::STOPPING;reason_=FeedbackReason::STOPPED;
  }
}
bool FeedbackWalk::assess(const SegmentAssessment& a,double now){
  if(!pendingValid_)return false;
  auto r=checkAssessment(a,pending_.target.sequence,policy_,now);
  if(r!=FeedbackReason::OK){reject(r);return false;}
  if(policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&!a.dynamicStabilityQualified){reject(FeedbackReason::SUPPORT_INVALID);return false;}
  assessment_=a;assessed_=true;pending_.executable=true;
  pending_.dynamicStabilityProven=policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&a.dynamicStabilityQualified;
  pending_.physicalReadiness=policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&a.loadedTrackingQualified;return true;
}
LocomotionFrame FeedbackWalk::generate(double nextClock){
  double cycle,rate,acceleration;
  if(state_==FeedbackWalkState::STOPPING&&nextClock>=stopClockS_){
    const double duration=2*specification_.periodS;
    const double u=std::fmin(1.,(nextClock-stopClockS_)/duration);
    const auto law=quinticTimeLaw(u,duration);
    const double integral=2.5*u*u*u*u-3*u*u*u*u*u+u*u*u*u*u*u;
    cycle=u==1?stopCycle_+1:stopCycle_+2*(u-integral);
    rate=(1-law.progress)/specification_.periodS;
    acceleration=-law.velocity/specification_.periodS;
  } else if(nextClock<specification_.rampDurationS){
    // Integral of smoothstep rate: displacement .5*duration/period at entry end.
    const double u=nextClock/specification_.rampDurationS;
    const auto law=quinticTimeLaw(u,specification_.rampDurationS);
    cycle=specification_.rampDurationS/specification_.periodS*(2.5*u*u*u*u-3*u*u*u*u*u+u*u*u*u*u*u);
    rate=law.progress/specification_.periodS;acceleration=law.velocity/specification_.periodS;
  } else {cycle=(nextClock-.5*specification_.rampDurationS)/specification_.periodS;rate=1/specification_.periodS;acceleration=0;}
  auto cart=gaitCartesian(gait_,cycle,rate,acceleration,state_==FeedbackWalkState::STOPPING&&nextClock>=stopClockS_?stopCycle_+1:-1);
  BodyPose local=cart.body;
  Vector3 translated;pointToWorld(origin_,local.translationM,translated);cart.body.translationM=translated;
  for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j)cart.body.rotation[i][j]=origin_.rotation[i][0]*local.rotation[0][j]+origin_.rotation[i][1]*local.rotation[1][j];
  cart.bodyVelocity=rotate(origin_,cart.bodyVelocity);cart.bodyAcceleration=rotate(origin_,cart.bodyAcceleration);
  for(unsigned i=0;i<4;++i){Vector3 world;pointToWorld(origin_,cart.feet[i],world);cart.feet[i]=world;cart.velocity[i]=rotate(origin_,cart.velocity[i]);cart.acceleration[i]=rotate(origin_,cart.acceleration[i]);}
  return solveGait(gait_,cart,&previous_);
}
FeedbackFrame FeedbackWalk::update(const AdaptiveObservation& o,double now){
  if(state_==FeedbackWalkState::IDLE||state_==FeedbackWalkState::REJECTED||state_==FeedbackWalkState::STAND){FeedbackFrame f;f.reason=reason_;return f;}
  if(now<lastNow_)return reject(FeedbackReason::STALE_OBSERVATION);
  lastNow_=now;
  auto r=checkObservation(o,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
  if(o.bellySupported)return reject(FeedbackReason::SUPPORT_INVALID);
  if(now-command_.stampS>=command_.ttlS){requestStop();reason_=FeedbackReason::COMMAND_EXPIRED;}
  if(pendingValid_){
    r=checkHeldSupport(o,previous_,pending_);if(r!=FeedbackReason::OK)return reject(r);
    if(now-pendingSince_>policy_.segmentTimeoutS)return reject(FeedbackReason::SEGMENT_TIMEOUT);
    if(!assessed_){pending_.reason=FeedbackReason::ASSESSMENT_REQUIRED;return pending_;}
    r=checkAssessment(assessment_,pending_.target.sequence,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
    r=checkTracking(o,pending_,policy_);if(r==FeedbackReason::FEEDBACK_PENDING){pending_.reason=r;return pending_;}
    if(r!=FeedbackReason::OK)return reject(r);
    if(now-pendingSince_+1e-9<policy_.samplePeriodS){pending_.reason=FeedbackReason::FEEDBACK_PENDING;return pending_;}
    if(state_==FeedbackWalkState::STOPPING&&pendingSolution_.cartesian.cycles==stopCycle_+1&&!o.stationary){pending_.reason=FeedbackReason::VERIFY_PENDING;return pending_;}
    previous_=pendingSolution_;clockS_=pendingClockS_;cycles_=previous_.cartesian.cycles;pendingValid_=false;
    if(state_==FeedbackWalkState::STOPPING&&cycles_==stopCycle_+1){state_=FeedbackWalkState::STAND;reason_=FeedbackReason::STOPPED;FeedbackFrame f;f.reason=reason_;f.body=previous_.cartesian.body;return f;}
    if(state_==FeedbackWalkState::STARTING&&clockS_>=specification_.rampDurationS)state_=FeedbackWalkState::WALK;
  }
  pendingClockS_=clockS_+policy_.samplePeriodS;pendingSolution_=generate(pendingClockS_);
  r=checkSegment(pendingSolution_,previous_.target,policy_);if(r!=FeedbackReason::OK)return reject(r);
  if(targetSequence_==0||targetSequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
  pending_=feedbackFrame(pendingSolution_,targetSequence_++);pending_.timeS=now;
  pendingValid_=true;assessed_=false;pendingSince_=now;return pending_;
}
} }
