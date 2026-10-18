#include "FeedbackWalk.h"
#include "TimedStand.h"
#include "StlContactGait.h"
#include "FirstStrideReference.h"
#include <cmath>
#include <limits>
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
    (!specification_.firstStrideOnly||(c.vxMps==specification_.maxVxMps&&c.vyMps==0&&c.yawRateRadS==0))&&
    std::abs(c.vxMps)<=specification_.maxVxMps&&std::abs(c.vyMps)<=specification_.maxVyMps&&std::abs(c.yawRateRadS)<=specification_.maxYawRateRadS;
}
bool FeedbackWalk::start(const AdaptiveStand& stand,const AdaptiveObservation& o,const PlanarVelocityCommand& c,
                         const WalkSpecification& s,const FeedbackPolicy& p,double now){
  static const FeedbackWalk kIdle{};*this=kIdle;policy_=p;   // no multi-KB temporary on the loop stack
if(s.maxJointStepRad>0)policy_.maxJointStepRad=s.maxJointStepRad;
  if(s.maxVelocityRadS>0||s.maxAccelerationRadS2>0){
    if(!positive(s.maxVelocityRadS)||!positive(s.maxAccelerationRadS2)){reject(FeedbackReason::INVALID_POLICY);return false;}
    for(unsigned i=0;i<4;++i){policy_.maxVelocityRadS[i]={s.maxVelocityRadS,s.maxVelocityRadS,s.maxVelocityRadS};policy_.maxAccelerationRadS2[i]={s.maxAccelerationRadS2,s.maxAccelerationRadS2,s.maxAccelerationRadS2};}
  }if(s.compiledFirstStride)policy_.samplePeriodS*=s.firstStrideTimeScale/9;specification_=s;lastNow_=now;targetSequence_=s.firstTargetSequence;
  if(!validFeedbackPolicy(policy_)||!std::isfinite(s.firstStrideTimeScale)||s.firstStrideTimeScale<1||s.firstStrideTimeScale>100||(s.compiledFirstStride&&(!s.firstStrideOnly||s.firstStrideTimeScale<9))){reject(FeedbackReason::INVALID_POLICY);return false;}
  if(s.compiledFirstStride&&(!compiledFirstStrideIntegrityPinned()||s.swayXM!=compiledFirstStrideSwayM()||s.periodS!=90||s.rampDurationS!=90||s.liftM!=.005||s.duty!=.8||s.standShiftDurationS!=8||!s.stlContactReference||!s.smoothStlReference||std::abs(s.maxVxMps-.005/(90*s.firstStrideTimeScale))>1e-12||s.maxVyMps!=0||s.maxYawRateRadS!=0)){reject(FeedbackReason::INVALID_POLICY);return false;}
  if((s.stlContactReference&&(!positive(s.standShiftDurationS)||!std::isfinite(s.swayXM)))||stand.state()!=AdaptiveStandState::STAND||!positive(s.periodS)||!positive(s.rampDurationS)||!positive(s.liftM)||
    !positive(s.maxVxMps)||(!std::isfinite(s.maxVyMps)||s.maxVyMps<0)||(!std::isfinite(s.maxYawRateRadS)||s.maxYawRateRadS<0)||!positive(s.maxCommandTtlS)||s.firstTargetSequence==0||
    !validCommand(c,now)||zero(c)){reject(FeedbackReason::INVALID_COMMAND);return false;}
  auto r=checkObservation(o,p,now);if(r!=FeedbackReason::OK){reject(r);return false;}
  if((p.mode==FeedbackMode::FIXTURE_REVIEW?!o.externalSupportConfirmed:o.confirmedFeet!=15)||o.bellySupported||!o.stationary){reject(FeedbackReason::SUPPORT_INVALID);return false;}
  auto planning=o;
  if(p.mode==FeedbackMode::FIXTURE_REVIEW){
    // Fixture anchors belong to the verified nominal STAND, not to encoder
    // quantization/backlash. Measured feedback remains o and the executor's
    // following monitor; never turn this planning reference into a measurement.
    const auto& held=stand.referenceTarget();
    if(!held.valid){reject(FeedbackReason::INVALID_COMMAND);return false;}
    for(unsigned i=0;i<4;++i){const auto& a=o.actual.legs[i];const auto& q=held.legs[i];
      if(std::abs(a.hip-q.hip)>p.maxTrackingErrorRad||std::abs(a.upper-q.upper)>p.maxTrackingErrorRad||std::abs(a.lower-q.lower)>p.maxTrackingErrorRad){reject(FeedbackReason::TRACKING_ERROR);return false;}}
    planning.actual=held;planning.body={};planning.body.translationM.z=.15;
  }
  else if(s.compiledFirstStride)planning.body.translationM.z=.15;
  previous_=observedSolution(planning);if(!previous_.target.valid){reject(FeedbackReason::IK_FAILURE);return false;}
  origin_=planning.body;origin_.translationM.z=0;
  const auto& d=canonicalStandDefinition();
  for(unsigned i=0;i<4;++i){Vector3 expected;pointToWorld(origin_,d.contactsWorldM[i],expected);
    if(distance(expected,previous_.cartesian.feet[i])>p.contactToleranceM){reject(FeedbackReason::FOOTPRINT_MISMATCH);return false;}}
  gait_.swayXM=s.swayXM;gait_.type=GaitType::WALK;gait_.heightM=planning.body.translationM.z;gait_.liftM=s.liftM;gait_.duty=s.duty;
  gait_.advanceXM=c.vxMps*s.periodS*(s.firstStrideOnly?s.firstStrideTimeScale:1);gait_.advanceYM=c.vyMps*s.periodS;gait_.yawRad=c.yawRateRadS*s.periodS;
  if(!validGaitParameters(gait_)){reject(FeedbackReason::INVALID_COMMAND);return false;}
  if(s.timeDriven){
    if(!positive(s.streamMaxObservationAgeS)||!positive(s.streamMaxGapS)||s.streamMaxGapS<policy_.samplePeriodS||!s.regionMinRad||!s.regionMaxRad||
       s.firstStrideOnly||s.compiledFirstStride||!positive(s.maxVelocityRadS)){reject(FeedbackReason::INVALID_POLICY);return false;}
    policy_.observationMaxAgeS=s.streamMaxObservationAgeS;startS_=lastFrameS_=now;frames_=0;
  }
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
  if(specification_.firstStrideOnly){reject(FeedbackReason::STOPPED);return;}
  queuedVelocity_=false; // explicit STOP/expiry always cancels a queued restart
  if(state_==FeedbackWalkState::STARTING||state_==FeedbackWalkState::WALK){
    // ceil() of a phase that is an integer up to rounding must not schedule one
    // more cycle: tolerance = 8 ULP of the phase, as legPhase().
    const double ulps=8*std::numeric_limits<double>::epsilon()*std::fmax(1.,std::abs(cycles_));
    stopCycle_=std::fmax(1.,std::ceil(cycles_-ulps));
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
  pending_.physicalReadiness=policy_.mode==FeedbackMode::FIXTURE_REVIEW?a.fixtureTrialEnvelopeReviewed:policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&a.loadedTrackingQualified;return true;
}
LocomotionFrame FeedbackWalk::generate(double nextClock){
  const double entry=specification_.stlContactReference?specification_.standShiftDurationS:0;
  const double walkClock=nextClock-entry;
  double cycle=0,rate=0,acceleration=0;
  CartesianSample cart;
  if(specification_.firstStrideOnly&&nextClock>=entry+specification_.periodS){
    const auto law=quinticTimeLaw(std::fmin(1.,(nextClock-entry-specification_.periodS)/entry),entry);
    cart=gaitHold(gait_,1,gait_.heightM,-gait_.swayXM*(1-law.progress),0,gait_.swayXM*law.velocity,0,gait_.swayXM*law.acceleration);cart.cycles=1;
  }else if(specification_.firstStrideOnly&&nextClock>=entry){
    const auto law=quinticTimeLaw((nextClock-entry)/specification_.periodS,specification_.periodS);
    cart=gaitCartesian(gait_,law.progress,law.velocity,law.acceleration,1);
  }else if(nextClock<entry){
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
    // Remaining displacement in s=1-u: 1-2(u-I(u)) = s^4(5-6s+2s^2) >= 0, exactly
    // 0 at u=1, so the terminal cycle is reached exactly and never exceeded
    // (the former 2(u-I(u)) cancelled to terminal+1 ULP after 10 cycles).
    const double s=1-u;
    cycle=(stopCycle_+1)-s*s*s*s*(5-s*(6-2*s));
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
  if(specification_.smoothStlReference&&specification_.stlContactReference){
    cart.stlSmoothing=1;
    if(nextClock<entry){const auto law=quinticTimeLaw(nextClock/entry,entry);cart.stlSmoothing=law.progress;cart.stlSmoothingVelocity=law.velocity;cart.stlSmoothingAcceleration=law.acceleration;}
    else if(specification_.firstStrideOnly&&nextClock>=entry+specification_.periodS){const auto law=quinticTimeLaw(std::fmin(1.,(nextClock-entry-specification_.periodS)/entry),entry);cart.stlSmoothing=1-law.progress;cart.stlSmoothingVelocity=-law.velocity;cart.stlSmoothingAcceleration=-law.acceleration;}
    else if(state_==FeedbackWalkState::STOPPING&&walkClock>=stopClockS_+2*specification_.periodS){const auto law=quinticTimeLaw(std::fmin(1.,(walkClock-stopClockS_-2*specification_.periodS)/entry),entry);cart.stlSmoothing=1-law.progress;cart.stlSmoothingVelocity=-law.velocity;cart.stlSmoothingAcceleration=-law.acceleration;}
  }
  BodyPose local=cart.body;
  Vector3 translated;pointToWorld(origin_,local.translationM,translated);cart.body.translationM=translated;
  for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j)cart.body.rotation[i][j]=origin_.rotation[i][0]*local.rotation[0][j]+origin_.rotation[i][1]*local.rotation[1][j];
  cart.bodyVelocity=rotate(origin_,cart.bodyVelocity);cart.bodyAcceleration=rotate(origin_,cart.bodyAcceleration);
  for(unsigned i=0;i<4;++i){Vector3 world;pointToWorld(origin_,cart.feet[i],world);cart.feet[i]=world;cart.velocity[i]=rotate(origin_,cart.velocity[i]);cart.acceleration[i]=rotate(origin_,cart.acceleration[i]);}
  if(specification_.compiledFirstStride)return compiledFirstStrideReference(cart,previous_,nextClock,specification_.firstStrideTimeScale);
  auto f=specification_.stlContactReference?solveStlContactGait(gait_,cart,&previous_):solveGait(gait_,cart,&previous_);
  if(specification_.firstStrideOnly){const double s=specification_.firstStrideTimeScale;for(unsigned i=0;i<4;++i){auto& v=f.velocityRadS[i];auto& a=f.accelerationRadS2[i];v={v.hip/s,v.upper/s,v.lower/s};a={a.hip/(s*s),a.upper/(s*s),a.lower/(s*s)};}}
  return f;
}
FeedbackFrame FeedbackWalk::update(const AdaptiveObservation& o,double now){
  if(state_==FeedbackWalkState::IDLE||state_==FeedbackWalkState::REJECTED||state_==FeedbackWalkState::STAND){FeedbackFrame f;f.reason=reason_;return f;}
  if(now<lastNow_)return reject(FeedbackReason::STALE_OBSERVATION);
  lastNow_=now;
  auto r=checkObservation(o,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
  if(o.bellySupported)return reject(FeedbackReason::SUPPORT_INVALID);
  if(now-command_.stampS>=command_.ttlS){if(specification_.firstStrideOnly)return reject(FeedbackReason::COMMAND_EXPIRED);requestStop();reason_=FeedbackReason::COMMAND_EXPIRED;}
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
    if(specification_.firstStrideOnly&&clockS_>=specification_.periodS+2*specification_.standShiftDurationS-1e-9){
      if(!o.stationary)return reject(FeedbackReason::SUPPORT_INVALID);
      state_=FeedbackWalkState::STAND;reason_=FeedbackReason::STOPPED;FeedbackFrame f;f.reason=reason_;f.body=previous_.cartesian.body;return f;
    }
    const double shift=specification_.stlContactReference?specification_.standShiftDurationS:0;
    if(state_==FeedbackWalkState::STOPPING&&cycles_==stopCycle_+1&&clockS_>=stopClockS_+2*specification_.periodS+2*shift-1e-9){
      if(queuedVelocity_&&validCommand(queuedCommand_,now)){
        auto planning=o;if(policy_.mode==FeedbackMode::FIXTURE_REVIEW){planning.body=previous_.cartesian.body;}
        previous_=observedSolution(planning);if(!previous_.target.valid)return reject(FeedbackReason::IK_FAILURE);
        origin_=planning.body;origin_.translationM.z=0;
        gait_.advanceXM=queuedCommand_.vxMps*specification_.periodS;
        gait_.advanceYM=queuedCommand_.vyMps*specification_.periodS;gait_.yawRad=queuedCommand_.yawRateRadS*specification_.periodS;
        if(!validGaitParameters(gait_))return reject(FeedbackReason::INVALID_COMMAND);
        command_=queuedCommand_;queuedVelocity_=false;clockS_=cycles_=0;stopCycle_=-1;stopClockS_=0;state_=FeedbackWalkState::STARTING;
      } else {state_=FeedbackWalkState::STAND;reason_=FeedbackReason::STOPPED;FeedbackFrame f;f.reason=reason_;f.body=previous_.cartesian.body;return f;}
    }
    if(state_==FeedbackWalkState::STARTING&&clockS_>=specification_.rampDurationS+(specification_.stlContactReference?specification_.standShiftDurationS:0))state_=FeedbackWalkState::WALK;
  }
  pendingClockS_=clockS_+policy_.samplePeriodS/(specification_.firstStrideOnly?specification_.firstStrideTimeScale:1);
  if(specification_.firstStrideOnly)pendingClockS_=std::fmin(pendingClockS_,specification_.periodS+2*specification_.standShiftDurationS);
  pendingSolution_=generate(pendingClockS_);
  r=checkSegment(pendingSolution_,previous_.target,policy_);if(r!=FeedbackReason::OK)return reject(r);
  if(targetSequence_==0||targetSequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
  pending_=feedbackFrame(pendingSolution_,targetSequence_++);pending_.timeS=now;
  if(policy_.mode==FeedbackMode::FIXTURE_REVIEW){pending_.requiredFeet=0;pending_.externalSupportRequired=true;}
  pendingValid_=true;assessed_=false;pendingSince_=now;return pending_;
}
FeedbackFrame FeedbackWalk::streamUpdate(const AdaptiveObservation& o,double now){
  if(!specification_.timeDriven){FeedbackFrame f;f.reason=FeedbackReason::INVALID_POLICY;return f;}
  if(state_==FeedbackWalkState::IDLE||state_==FeedbackWalkState::REJECTED||state_==FeedbackWalkState::STAND){FeedbackFrame f;f.reason=reason_;return f;}
  if(!std::isfinite(now)||now<lastNow_)return reject(FeedbackReason::STALE_OBSERVATION);
  lastNow_=now;
  auto r=checkObservation(o,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
  if(o.bellySupported||(policy_.mode==FeedbackMode::FIXTURE_REVIEW&&!o.externalSupportConfirmed))return reject(FeedbackReason::SUPPORT_INVALID);
  if(state_==FeedbackWalkState::HOLD){
    // STAND only from measurements: the executor reports the final target
    // settled (fresh samples inside the tube) and the joints are stationary.
    if(o.acknowledged&&o.acknowledgedSequence==holdSequence_&&o.stationary){
      state_=FeedbackWalkState::STAND;reason_=FeedbackReason::STOPPED;FeedbackFrame f;f.reason=reason_;f.body=previous_.cartesian.body;return f;}
    if(now-holdSinceS_>policy_.segmentTimeoutS)return reject(FeedbackReason::SEGMENT_TIMEOUT);
    FeedbackFrame f;f.reason=FeedbackReason::VERIFY_PENDING;f.body=previous_.cartesian.body;return f;
  }
  if((state_==FeedbackWalkState::STARTING||state_==FeedbackWalkState::WALK)&&now-command_.stampS>=command_.ttlS){requestStop();reason_=FeedbackReason::COMMAND_EXPIRED;}
  const double dt=now-lastFrameS_;
  if(frames_&&(dt<=0||dt>specification_.streamMaxGapS))return reject(FeedbackReason::CONTINUITY);  // never plan through an unvalidated gap
  const double shift=specification_.stlContactReference?specification_.standShiftDurationS:0;
  double clock=now-startS_;
  // The coordinated stop ends EXACTLY on its terminal instant: the last frame
  // is the all-foot terminal pose, never beyond it.
  const double stopEnd=stopClockS_+2*specification_.periodS+2*shift;
  const bool terminal=state_==FeedbackWalkState::STOPPING&&clock-shift>=stopEnd-1e-9;
  if(terminal)clock=stopEnd+shift;
  pendingSolution_=generate(clock);
  // Rates over the real elapsed interval: the analytic joint velocity bound
  // also bounds the chord of the interval (no skipped, unvalidated jump).
  FeedbackPolicy p=policy_;
  double vmax=1e9;for(unsigned i=0;i<4;++i)vmax=std::fmin(vmax,std::fmin(p.maxVelocityRadS[i].hip,std::fmin(p.maxVelocityRadS[i].upper,p.maxVelocityRadS[i].lower)));
  p.maxJointStepRad=vmax*std::fmax(dt,1e-6)+1e-9;
  r=checkSegment(pendingSolution_,previous_.target,p);if(r!=FeedbackReason::OK)return reject(r);
  for(unsigned i=0;i<12;++i){const auto& l=pendingSolution_.target.legs[i/3];const double q=i%3==0?l.hip:i%3==1?l.upper:l.lower;
    if(!(q>=specification_.regionMinRad[i]&&q<=specification_.regionMaxRad[i]))return reject(FeedbackReason::COLLISION);}  // outside the certified box
  if(targetSequence_==0||targetSequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
  pending_=feedbackFrame(pendingSolution_,targetSequence_++);pending_.timeS=now;
  if(policy_.mode==FeedbackMode::FIXTURE_REVIEW){pending_.requiredFeet=0;pending_.externalSupportRequired=true;}
  pending_.executable=true;pending_.physicalReadiness=policy_.mode==FeedbackMode::FIXTURE_REVIEW;pending_.reason=FeedbackReason::OK;
  previous_=pendingSolution_;clockS_=clock;cycles_=previous_.cartesian.cycles;lastFrameS_=now;++frames_;
  if(state_==FeedbackWalkState::STARTING&&clock>=specification_.rampDurationS+shift)state_=FeedbackWalkState::WALK;
  if(terminal){state_=FeedbackWalkState::HOLD;holdSequence_=pending_.target.sequence;holdSinceS_=now;}
  return pending_;
}
} }
