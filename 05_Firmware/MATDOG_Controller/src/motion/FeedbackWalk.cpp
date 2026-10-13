#include "FeedbackWalk.h"
#include "TimedStand.h"
#include "StlContactGait.h"
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
  if((s.stlContactReference&&(!positive(s.standShiftDurationS)||!std::isfinite(s.swayXM)))||stand.state()!=AdaptiveStandState::STAND||!positive(s.periodS)||!positive(s.rampDurationS)||!positive(s.liftM)||
    !positive(s.maxVxMps)||!positive(s.maxVyMps)||!positive(s.maxYawRateRadS)||!positive(s.maxCommandTtlS)||s.firstTargetSequence==0||
    !validCommand(c,now)||zero(c)){reject(FeedbackReason::INVALID_COMMAND);return false;}
  auto r=checkObservation(o,p,now);if(r!=FeedbackReason::OK){reject(r);return false;}
  if(o.confirmedFeet!=15||o.bellySupported||!o.stationary){reject(FeedbackReason::SUPPORT_INVALID);return false;}
  previous_=observedSolution(o);if(!previous_.target.valid){reject(FeedbackReason::IK_FAILURE);return false;}
  origin_=o.body;origin_.translationM.z=0;
  const auto& d=canonicalStandDefinition();
  for(unsigned i=0;i<4;++i){Vector3 expected;pointToWorld(origin_,d.contactsWorldM[i],expected);
    if(distance(expected,previous_.cartesian.feet[i])>p.contactToleranceM){reject(FeedbackReason::FOOTPRINT_MISMATCH);return false;}}
  gait_.swayXM=s.swayXM;gait_.type=GaitType::WALK;gait_.heightM=o.body.translationM.z;gait_.liftM=s.liftM;gait_.duty=s.duty;
  gait_.advanceXM=c.vxMps*s.periodS;gait_.advanceYM=c.vyMps*s.periodS;gait_.yawRad=c.yawRateRadS*s.periodS;
  if(!validGaitParameters(gait_)){reject(FeedbackReason::INVALID_COMMAND);return false;}
  command_=c;state_=FeedbackWalkState::STARTING;reason_=FeedbackReason::OK;return true;
}
bool FeedbackWalk::command(const PlanarVelocityCommand& c,double now){
  if(state_==FeedbackWalkState::REJECTED||state_==FeedbackWalkState::IDLE||state_==FeedbackWalkState::STAND)return false;
  if(!validCommand(c,now)||now<lastNow_||c.stampS<command_.stampS){reason_=FeedbackReason::INVALID_COMMAND;return false;}
  if(c.sequence<=command_.sequence){reason_=FeedbackReason::SEQUENCE_REPLAY;return false;}
  if(zero(c)){queuedVelocity_=false;command_=c;requestStop();return true;}
  // Parameter changes retain the old anchors through deceleration, then
  // rebase ONLY at independently tracked all-foot STAND. The transition goes
  // through zero velocity and the same G4 ramp; it never changes a lifted foot.
  if(queuedVelocity_||c.vxMps!=command_.vxMps||c.vyMps!=command_.vyMps||c.yawRateRadS!=command_.yawRateRadS){
    requestStop();queuedCommand_=c;queuedVelocity_=true;command_=c;return true;
  }
  command_=c;return true;
}
void FeedbackWalk::requestStop(){
  queuedVelocity_=false; // explicit STOP/expiry always cancels a queued restart
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
  const double entry=specification_.stlContactReference?specification_.standShiftDurationS:0;
  const double walkClock=nextClock-entry;
  double cycle=0,rate=0,acceleration=0;
  CartesianSample cart;
  if(nextClock<entry){
    const auto law=quinticTimeLaw(nextClock/entry,entry);
    cart=gaitHold(gait_,0,gait_.heightM,-gait_.swayXM*law.progress,0,-gait_.swayXM*law.velocity,0,-gait_.swayXM*law.acceleration);
  } else if(state_==FeedbackWalkState::STOPPING&&entry>0&&walkClock>=stopClockS_+2*specification_.periodS){
    const auto law=quinticTimeLaw(std::fmin(1.,(walkClock-stopClockS_-2*specification_.periodS)/entry),entry);
    cart=gaitHold(gait_,stopCycle_+1,gait_.heightM,-gait_.swayXM*(1-law.progress),0,gait_.swayXM*law.velocity,0,gait_.swayXM*law.acceleration);
    cart.cycles=stopCycle_+1;
  } else {
  if(state_==FeedbackWalkState::STOPPING&&walkClock>=stopClockS_){
    const double duration=2*specification_.periodS;
    const double u=std::fmin(1.,(walkClock-stopClockS_)/duration);
    const auto law=quinticTimeLaw(u,duration);
    const double integral=2.5*u*u*u*u-3*u*u*u*u*u+u*u*u*u*u*u;
    cycle=u==1?stopCycle_+1:stopCycle_+2*(u-integral);
    rate=(1-law.progress)/specification_.periodS;
    acceleration=-law.velocity/specification_.periodS;
  } else if(walkClock<specification_.rampDurationS){
    // Integral of smoothstep rate: displacement .5*duration/period at entry end.
    const double u=walkClock/specification_.rampDurationS;
    const auto law=quinticTimeLaw(u,specification_.rampDurationS);
    cycle=specification_.rampDurationS/specification_.periodS*(2.5*u*u*u*u-3*u*u*u*u*u+u*u*u*u*u*u);
    rate=law.progress/specification_.periodS;acceleration=law.velocity/specification_.periodS;
  } else {cycle=(walkClock-.5*specification_.rampDurationS)/specification_.periodS;rate=1/specification_.periodS;acceleration=0;}
  cart=gaitCartesian(gait_,cycle,rate,acceleration,state_==FeedbackWalkState::STOPPING&&walkClock>=stopClockS_?stopCycle_+1:-1);
  }
  BodyPose local=cart.body;
  Vector3 translated;pointToWorld(origin_,local.translationM,translated);cart.body.translationM=translated;
  for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j)cart.body.rotation[i][j]=origin_.rotation[i][0]*local.rotation[0][j]+origin_.rotation[i][1]*local.rotation[1][j];
  cart.bodyVelocity=rotate(origin_,cart.bodyVelocity);cart.bodyAcceleration=rotate(origin_,cart.bodyAcceleration);
  for(unsigned i=0;i<4;++i){Vector3 world;pointToWorld(origin_,cart.feet[i],world);cart.feet[i]=world;cart.velocity[i]=rotate(origin_,cart.velocity[i]);cart.acceleration[i]=rotate(origin_,cart.acceleration[i]);}
  return specification_.stlContactReference?solveStlContactGait(gait_,cart,&previous_):solveGait(gait_,cart,&previous_);
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
    const double shift=specification_.stlContactReference?specification_.standShiftDurationS:0;
    if(state_==FeedbackWalkState::STOPPING&&cycles_==stopCycle_+1&&clockS_>=stopClockS_+2*specification_.periodS+2*shift-1e-9){
      if(queuedVelocity_&&validCommand(queuedCommand_,now)){
        previous_=observedSolution(o);if(!previous_.target.valid)return reject(FeedbackReason::IK_FAILURE);
        origin_=o.body;origin_.translationM.z=0;
        gait_.advanceXM=queuedCommand_.vxMps*specification_.periodS;
        gait_.advanceYM=queuedCommand_.vyMps*specification_.periodS;gait_.yawRad=queuedCommand_.yawRateRadS*specification_.periodS;
        if(!validGaitParameters(gait_))return reject(FeedbackReason::INVALID_COMMAND);
        command_=queuedCommand_;queuedVelocity_=false;clockS_=cycles_=0;stopCycle_=-1;stopClockS_=0;state_=FeedbackWalkState::STARTING;
      } else {state_=FeedbackWalkState::STAND;reason_=FeedbackReason::STOPPED;FeedbackFrame f;f.reason=reason_;f.body=previous_.cartesian.body;return f;}
    }
    if(state_==FeedbackWalkState::STARTING&&clockS_>=specification_.rampDurationS+(specification_.stlContactReference?specification_.standShiftDurationS:0))state_=FeedbackWalkState::WALK;
  }
  pendingClockS_=clockS_+policy_.samplePeriodS;pendingSolution_=generate(pendingClockS_);
  r=checkSegment(pendingSolution_,previous_.target,policy_);if(r!=FeedbackReason::OK)return reject(r);
  if(targetSequence_==0||targetSequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
  pending_=feedbackFrame(pendingSolution_,targetSequence_++);pending_.timeS=now;
  pendingValid_=true;assessed_=false;pendingSince_=now;return pending_;
}
} }
