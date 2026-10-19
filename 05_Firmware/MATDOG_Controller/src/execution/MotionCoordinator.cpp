#include "MotionCoordinator.h"
#include <cmath>
#include <cstring>
#include "../motion/FixturePoseClassifier.h"
#include "../motion/OperationalWalkProfile.h"
static_assert(matdog::motion::operational_walk::kMaxProfileVelocityRadS<=matdog::execution::MotionExecutor::kWalkRateCeilingRadS&&
              matdog::motion::operational_walk::kMaxProfileAccelerationRadS2<=matdog::execution::MotionExecutor::kWalkAccelerationCeilingRadS2,
              "every walk profile envelope must fit the executor ceilings");
namespace matdog { namespace execution {
void MotionCoordinator::begin(MotionExecutor* e,actuator::SafeActuatorPolicy* p){executor_=e;policy_=p;configured_=false;walking_=false;pending_stand_=pending_walk_=false;submitted_sequence_=0;arm_generation_=0;}
void MotionCoordinator::synchronizeSession(){
 if(!executor_||arm_generation_==executor_->status().arm_generation)return;
 arm_generation_=executor_->status().arm_generation;
 submitted_sequence_=0;assessment_={};frame_=motion::FeedbackFrame{};walking_=false;
 static const motion::FeedbackWalk kIdleWalk{};
 stand_.stop();walk_=kIdleWalk;scheduler_.stop();walk_profile_=nullptr;final_height_m_=.150;
}
bool MotionCoordinator::configure(const motion::FeedbackPolicy& p,const motion::WalkSpecification& w,const motion::AcquisitionRoute* a){
 if(executor_&&executor_->active())return false;
 if(!motion::validFeedbackPolicy(p))return false;
 feedback_policy_=p;walk_specification_=w;
 static const motion::AcquisitionRoute kEmptyRoute{};   // no multi-KB temporary on the loop stack
 if(a)acquisition_=*a;else acquisition_=kEmptyRoute;
 // Build the static timing table before PRIME, never while holding torque.
 if(!stand_.prepareTimedRoute(acquisition_,p))return false;
 configured_=true;return true;
}
bool MotionCoordinator::configureCommissioning(const CommissioningStandConfig& c){
 if(executor_&&executor_->active())return false;
 if(!c.enabled){clearCommissioning();return true;}
 if(!configured_||feedback_policy_.mode!=motion::FeedbackMode::FIXTURE_REVIEW||!feedback_policy_.fixtureLimitVariant||!acquisition_.pairedProne||
    (!c.measuredStart&&!c.certificate.present)||c.keepoutCount>CommissioningStandConfig::kMaxKeepouts||
    (c.measuredStart&&(!(c.proneMaxDeviationRad<=CommissioningStandConfig::kCertifiedBandRad+1e-12)||
                       !(feedback_policy_.maxTrackingErrorRad+feedback_policy_.maxJointStepRad+motion::tolerance::kQuantizationRad+motion::tolerance::kModelUncertaintyRad<CommissioningStandConfig::kCertifiedTubeRad)))||
    !std::isfinite(c.pathMarginRad)||c.pathMarginRad<=0||!std::isfinite(c.minStageDurationS)||c.minStageDurationS<=0||
    !std::isfinite(c.proneMaxDeviationRad)||!std::isfinite(c.proneMaxRmsRad)||c.proneMaxRmsRad<=0||c.proneMaxRmsRad>c.proneMaxDeviationRad)return false;
 // The entry proof's folded boundary includes the live LOWER hard limit
 // plus an explicit local model allowance. Never apply it to wider limits.
 if(c.measuredStart)for(unsigned leg=0;leg<4;++leg)
  if(feedback_policy_.jointLimits[leg][2].lower<motion::tolerance::kEntryLowerMeasuredMinRad[leg])return false;
 commissioning_=c;
 entry_plan_=motion::FixtureEntryPlan{};return true;
}
void MotionCoordinator::observe(const motion::AdaptiveObservation& o){observation_=o;}
bool MotionCoordinator::assess(const motion::SegmentAssessment& a,uint32_t now){
 const bool ok=walking_?walk_.assess(a,walkTime(now)):stand_.assess(a,walkTime(now));
 if(ok)assessment_=a;
 return ok;
}
const char* MotionCoordinator::startStand(const motion::MotionCommand& c,uint32_t now){
   if(walking_)return "WALK_MUST_STOP_FIRST";
   if(commissioning_.enabled&&commissioning_.measuredStart&&stand_.state()==motion::AdaptiveStandState::STAND){
    // Already standing: change height from the verified solution (or acknowledge the same height).
    if(!motion::standHeightReachable(c.target_height_m,feedback_policy_))return "HEIGHT_OUTSIDE_LIVE_LIMITS";
    if(std::abs(stand_.standHeightM()-c.target_height_m)<1e-9){final_height_m_=c.target_height_m;return nullptr;}
    final_height_m_=c.target_height_m;
    return stand_.requestHeightAdjust(c.target_height_m,executor_->status().accepted_sequence+1,walkTime(now))?nullptr:"HEIGHT_ADJUST_REFUSED";
   }
   motion::AdaptiveStandRequest r;r.targetHeightM=c.target_height_m;r.firstSequence=executor_->status().accepted_sequence+1;
   final_height_m_=.150;
   if(commissioning_.enabled&&commissioning_.measuredStart){final_height_m_=c.target_height_m;r.targetHeightM=.150;}
   if(acquisition_.count||acquisition_.pairedProne)r.acquisition=&acquisition_;
   if(commissioning_.enabled){
    // The entry starts at the armed (held) pose, never at a nominal knot.
    motion::JointTargetFrame start;start.valid=true;
    for(unsigned i=0;i<12;++i){auto& q=start.legs[i/3];(i%3==0?q.hip:i%3==1?q.upper:q.lower)=executor_->targetRadians()[i];}
    const auto head=motion::pairedProneSample(0,0,true);if(!head.target.valid)return "ROUTE_HEAD_INVALID";
    if(commissioning_.measuredStart){
     // Reuse the pre-arm path only for the same held start (two rounded
     // micro-radian conversions). Avoid repeating geometry work under torque.
     bool same=entry_plan_.status==motion::EntryStatus::OK&&entry_plan_.start.valid;
     for(unsigned i=0;i<12&&same;++i){const auto& a=entry_plan_.start.legs[i/3];const auto& b=start.legs[i/3];
      const double x=i%3==0?a.hip:i%3==1?a.upper:a.lower,y=i%3==0?b.hip:i%3==1?b.upper:b.lower;
      same=std::abs(x-y)<=2e-6;}
     if(!same){
      motion::MeasuredEntryConfig m;m.keepouts=commissioning_.keepouts;m.keepoutCount=commissioning_.keepoutCount;m.pathMarginRad=commissioning_.pathMarginRad;
      entry_plan_=motion::planMeasuredEntry(start,head.target,feedback_policy_,m,commissioning_.minStageDurationS);
     }
    } else entry_plan_=motion::planFixtureEntry(start,head.target,feedback_policy_,commissioning_.certificate,commissioning_.minStageDurationS);
    if(entry_plan_.status!=motion::EntryStatus::OK){entry_reason_=motion::entryStatusName(entry_plan_.status);return entry_reason_;}
    r.entry=&entry_plan_;r.proneMaxDeviationRad=commissioning_.proneMaxDeviationRad;r.proneMaxRmsRad=commissioning_.proneMaxRmsRad;
   }
   entry_reason_="";
   return stand_.request(r,feedback_policy_,walkTime(now))?nullptr:motion::feedbackReasonName(stand_.reason());
}
const char* MotionCoordinator::preArmCheck(){
 const auto& o=observation_;
 if(!o.actual.valid||!o.stationary){entry_reason_="PRE_ARM_FEEDBACK_REQUIRED";return entry_reason_;}
 const auto head=motion::pairedProneSample(0,0,true);if(!head.target.valid)return "ROUTE_HEAD_INVALID";
 double actual[12]{},nominal[12]{};
 for(unsigned i=0;i<12;++i){const auto& a=o.actual.legs[i/3];const auto& n=head.target.legs[i/3];
  actual[i]=i%3==0?a.hip:i%3==1?a.upper:a.lower;nominal[i]=i%3==0?n.hip:i%3==1?n.upper:n.lower;}
 const auto prone=motion::classifyFixtureProne(actual,nominal,o.externalSupportConfirmed,o.stationary,
                                              commissioning_.proneMaxDeviationRad,commissioning_.proneMaxRmsRad);
 if(prone.kind!=motion::FixturePoseKind::PRONE_COMPATIBLE){entry_reason_=motion::fixturePoseReasonName(prone.reason);return "PRE_ARM_NOT_PRONE";}
 // The session primes a LOWER resting past its folded limit just inside it
 // (MotionSession::insideTick, bounded there); plan from that same start.
 motion::JointTargetFrame start;start.valid=true;
 for(unsigned i=0;i<12;++i){double q=actual[i];
  const double lo=feedback_policy_.jointLimits[i/3][i%3].lower;
  if(i%3==2&&q<lo)q=lo+kPrimeInwardMarginTicks*2*3.14159265358979323846/4096;
  auto& s=start.legs[i/3];(i%3==0?s.hip:i%3==1?s.upper:s.lower)=q;}
 motion::MeasuredEntryConfig m;m.keepouts=commissioning_.keepouts;m.keepoutCount=commissioning_.keepoutCount;m.pathMarginRad=commissioning_.pathMarginRad;
 entry_plan_=motion::planMeasuredEntry(start,head.target,feedback_policy_,m,commissioning_.minStageDurationS);
 if(entry_plan_.status!=motion::EntryStatus::OK){entry_reason_=motion::entryStatusName(entry_plan_.status);return entry_reason_;}
 entry_reason_="";return nullptr;
}
const char* MotionCoordinator::accept(const motion::MotionCommand& c,uint32_t now){
 synchronizeSession();
 if(!configured_)return "PLANNER_ARTIFACT_REQUIRED";
 if(pending_stand_)return "ARMING_IN_PROGRESS";
 // WALK owns its prerequisite transition; there is no dependency between UI buttons.
 if(c.intent==motion::Intent::BASE_VELOCITY&&commissioning_.measuredStart&&!walking_){
  const bool zero=c.vx==0&&c.vy==0&&c.yaw_rate==0;
  if(zero){if(pending_walk_)stop(now);return nullptr;}
  const auto* profile=motion::operational_walk::profileForVelocity(c.vx);
  if(!profile||c.vy!=0||c.yaw_rate!=0)return "WALK_SPEED_OUTSIDE_PROFILE";
  if(!motion::operational_walk::fitsLimits(*profile,feedback_policy_,feedback_policy_.maxTrackingErrorRad))return "WALK_PROFILE_OUTSIDE_LIMITS";
  if(stand_.state()!=motion::AdaptiveStandState::STAND||std::abs(stand_.standHeightM()-.150)>1e-9){
   auto prerequisite=c;prerequisite.intent=motion::Intent::STAND;prerequisite.target_height_m=.150;
   if(const char* why=accept(prerequisite,now))return why;
   pending_walk_command_=c;pending_walk_=true;return nullptr;
  }
 }

 if((!executor_||!executor_->active())&&executor_&&c.intent==motion::Intent::STAND&&commissioning_.enabled&&commissioning_.measuredStart&&armer_){
  // Operational STAND from READY: the explicit command starts the controlled
  // torque sequence; the planner starts only once the session is ARMED.
  // The certified prone entry always ends at 150 mm; a lower height is a controlled
  // descent after it. Refuse a height the LIVE limits cannot hold before any torque.
  if(!motion::standHeightReachable(c.target_height_m,feedback_policy_))return "HEIGHT_OUTSIDE_LIVE_LIMITS";
  if(const char* why=preArmCheck())return why;
  if(const char* why=armer_->armForCommand(now))return why;
  pending_stand_=true;arming_failure_=nullptr;pending_command_=c;pending_since_ms_=now;return nullptr;
 }
 if(!executor_||!executor_->active())return "EXECUTOR_NOT_ARMED";
 switch(c.intent){
  case motion::Intent::STAND:return startStand(c,now);
  case motion::Intent::GAIT_SELECT:return nullptr; // only WALK survived service validation
  case motion::Intent::BASE_VELOCITY:{
   motion::PlanarVelocityCommand v;v.vxMps=c.vx;v.vyMps=c.vy;v.yawRateRadS=c.yaw_rate;v.stampS=c.issued_at_ms/1000.0;v.ttlS=c.ttl_ms/1000.0;v.sequence=c.sequence;
   if(commissioning_.measuredStart){
    // Operational: the request was age-checked against its own TTL by the
    // contract; the stride runs on the Controller clock with the reviewed
    // internal TTL, renewed only by the operator heartbeat (renewWalk).
    const bool zero=v.vxMps==0&&v.vyMps==0&&v.yawRateRadS==0;
    if(walking_&&!zero)return "WALK_IN_PROGRESS";            // change speed: WALK STOP, then START
    if(!walking_&&!zero){
     if(stand_.state()!=motion::AdaptiveStandState::STAND)return "STAND_VERIFIED_REQUIRED";
     if(std::abs(stand_.standHeightM()-.150)>1e-9)return "WALK_REQUIRES_150MM_STAND";   // the certified WALK box is built at 150 mm
     const auto* profile=motion::operational_walk::profileForVelocity(v.vxMps);
     if(!profile||v.vyMps!=0||v.yawRateRadS!=0)return "WALK_SPEED_NOT_CERTIFIED";
     // The profile's certified box +/- the tube must sit inside the live limits.
     if(!motion::operational_walk::fitsLimits(*profile,feedback_policy_,feedback_policy_.maxTrackingErrorRad))return "WALK_PROFILE_OUTSIDE_LIMITS";
     // Walk-only joint rates, selected between frames; stand rates come back at STAND.
     if(!executor_->selectRates(profile->maxVelocityRadS,profile->maxAccelerationRadS2))return "EXECUTOR_NOT_READY";
     walk_profile_=profile;
    }
    v.stampS=walkTime(now);v.ttlS=std::fmin(v.ttlS,walk_specification_.maxCommandTtlS);
    if(walking_)v.sequence=walk_command_.sequence+1;           // after the internal renewals
   }
   if(walking_)return walk_.command(v,walkTime(now))?nullptr:motion::feedbackReasonName(walk_.reason());
   if(v.vxMps==0&&v.vyMps==0&&v.yawRateRadS==0)return nullptr;
   auto spec=commissioning_.measuredStart&&walk_profile_?motion::operational_walk::specification(*walk_profile_):walk_specification_;
   spec.firstTargetSequence=executor_->status().accepted_sequence+1;
   if(!walk_.start(stand_,observation_,v,spec,feedback_policy_,walkTime(now))){executor_->restoreArmedRates();walk_profile_=nullptr;return motion::feedbackReasonName(walk_.reason());}
   walk_command_=v;walking_=true;active_spec_=spec;return nullptr;
  }
  default:return "ACTION_NOT_VALIDATED";
 }
}
void MotionCoordinator::stop(uint32_t){pending_walk_=false;pending_stand_=false;if(session_&&(session_->scope()==TrialScope::SUPERVISED_FIXTURE_STAND||session_->scope()==TrialScope::SUPERVISED_FIXTURE_WALK||session_->scope()==TrialScope::FIXTURE_COMMISSIONING)){stand_.stop();session_->revoke("LOCAL_FIXTURE_STOP_SAFE_OFF");return;}if(walking_)walk_.requestStop();else {stand_.stop();if(session_)session_->revoke("LOCAL_OPERATOR_STOP");}}
bool MotionCoordinator::renewWalk(uint32_t now){
 if(!walking_||!commissioning_.measuredStart)return false;
 const auto st=walk_.state();if(st!=motion::FeedbackWalkState::STARTING&&st!=motion::FeedbackWalkState::WALK)return false;
 auto v=walk_command_;v.stampS=walkTime(now);v.sequence=walk_command_.sequence+1;
 if(!walk_.command(v,walkTime(now)))return false;
 walk_command_=v;return true;
}
const char* MotionCoordinator::state() const {
 if(!walking_)return motion::adaptiveStandStateName(stand_.state());
 switch(walk_.state()){case motion::FeedbackWalkState::IDLE:return "IDLE";case motion::FeedbackWalkState::STARTING:return "WALK_STARTING";case motion::FeedbackWalkState::WALK:return "WALK";case motion::FeedbackWalkState::STOPPING:case motion::FeedbackWalkState::HOLD:return "STOPPING";case motion::FeedbackWalkState::STAND:return "STAND";default:return "REJECTED";}
}
void MotionCoordinator::update(const MotionExecutionFacts& facts,const JointFeedback& fb,uint32_t now){update(facts,fb,now,static_cast<uint64_t>(now)*1000);}
// Time-driven WALK (Issue #52): the reference follows the monotonic clock at
// the scheduler's absolute release instants; one coordinated frame per
// release, never waiting for the previous frame to be reached. Tracking is the
// executor's asynchronous per-joint monitor; HOLD -> STAND only when the final
// target is settled by measurement.
void MotionCoordinator::streamWalk(const MotionExecutionFacts& facts,const JointFeedback& fb,uint32_t now,uint64_t now_us){
 const auto& w=active_spec_;
 if(!executor_->streaming()){
  if(executor_->status().state!=ExecutorState::READY)return;
  StreamConfig c;c.tube_rad=feedback_policy_.maxTrackingErrorRad;c.max_lag_s=w.streamMaxLagS;c.settle_speed_rad_s=w.streamSettleSpeedRadS;
  c.feedback_max_age_us=static_cast<uint32_t>(std::llround(w.streamFeedbackMaxAgeS*1e6));c.hold_timeout_us=static_cast<uint32_t>(std::llround(w.streamHoldTimeoutS*1e6));
  c.max_frame_gap_us=static_cast<uint32_t>(std::llround(w.streamMaxGapS*1e6));c.persist_samples=static_cast<uint8_t>(w.streamPersistSamples);
  c.region_min_rad=w.regionMinRad;c.region_max_rad=w.regionMaxRad;
  if(!executor_->beginStream(c,fb,now_us)||!scheduler_.start(now_us+static_cast<uint64_t>(std::llround(w.streamPeriodS*1e6)),static_cast<uint32_t>(std::llround(w.streamPeriodS*1e6)))){executor_->stop("STREAM_BEGIN_REFUSED");return;}
 }
 executor_->monitor(facts,fb,now,now_us);
 if(!executor_->streaming())return;   // stopped by the monitor: SAFE_OFF path
 auto o=observation_;const auto& st=executor_->status();
 o.acknowledged=st.settled_sequence!=0;o.acknowledgedSequence=st.settled_sequence;
 const auto release=scheduler_.poll(now_us);
 if(!release.release&&walk_.state()!=motion::FeedbackWalkState::HOLD)return;
 frame_=walk_.streamUpdate(o,(now_us/1e6));
 if(walk_.state()==motion::FeedbackWalkState::STAND){
  if(!executor_->endStream()){executor_->stop("HOLD_NOT_SETTLED");return;}
  walking_=false;scheduler_.stop();executor_->restoreArmedRates();walk_profile_=nullptr;return;}
 if(walk_.state()==motion::FeedbackWalkState::REJECTED){executor_->stop(motion::feedbackReasonName(frame_.reason));return;}
 if(!release.release||!frame_.target.valid||!frame_.executable||frame_.target.sequence==submitted_sequence_)return;
 // streamUpdate validated continuity/rates over the real interval, limits,
 // support and the certified region box for this frame; the executor repeats
 // the joint-space checks against the live mapping before the first byte.
 FrameValidation v;v.sequence=frame_.target.sequence;v.policy_epoch=policy_->epoch();v.geometry=policy_->currentGeometryTag();
 v.collision_clear=v.support_valid=v.path_continuous=v.configuration_valid=true;
 const bool final=walk_.state()==motion::FeedbackWalkState::HOLD;
 if(executor_->stream(frame_.target,v,now_us,now_us+scheduler_.periodUs(),final,facts,fb,now,now_us))submitted_sequence_=frame_.target.sequence;
}
// Same executor, scheduler, geometry generator and assessment path as WALK /
// finite motion. Only structural STAND endpoints enter HOLD; microframes use TX.
void MotionCoordinator::streamStand(const MotionExecutionFacts& facts,const JointFeedback& fb,uint32_t now,uint64_t now_us){
 executor_->monitor(facts,fb,now,now_us);
 if(!executor_->active())return;
 bool arrived=false;
 if(executor_->status().state==ExecutorState::HOLD){
  if(!executor_->endStream())return;
  arrived=true;
 }
 const uint32_t period=static_cast<uint32_t>(std::llround(feedback_policy_.samplePeriodS*1e6));
 // A second call after Controller assessment transmits the SAME generated
 // reference at this release; it must not consume a second scheduler tick.
 const bool refreshed=frame_.target.valid&&!frame_.executable&&assessment_.sequence==frame_.target.sequence;
 if(arrived||refreshed||!scheduler_.running()||scheduler_.poll(now_us).release){
  auto o=observation_;
  if(arrived)o.stationary=executor_->measuredStandQuiet();
  o.acknowledgedSequence=stand_.pendingFinal()?executor_->status().completed_sequence:executor_->status().tx_sequence;
  o.acknowledged=o.acknowledgedSequence!=0;
  frame_=stand_.update(o,now_us/1e6,true);stand_release_us_=now_us;
 }
 if(stand_.state()==motion::AdaptiveStandState::REJECTED){executor_->stop(motion::feedbackReasonName(frame_.reason));return;}
 if(stand_.state()==motion::AdaptiveStandState::STAND){
  scheduler_.stop();
  if(commissioning_.measuredStart&&std::abs(final_height_m_-stand_.standHeightM())>1e-9&&
     !stand_.requestHeightAdjust(final_height_m_,executor_->status().accepted_sequence+1,now_us/1e6))executor_->stop("HEIGHT_ADJUST_REFUSED");
  return;
 }
 if(executor_->status().state==ExecutorState::READY){
  if(stand_.pendingFinal()&&frame_.target.sequence==submitted_sequence_)return;
  for(unsigned i=0;i<12;++i){stand_min_[i]=feedback_policy_.jointLimits[i/3][i%3].lower;stand_max_[i]=feedback_policy_.jointLimits[i/3][i%3].upper;}
  StreamConfig c;c.tube_rad=feedback_policy_.maxTrackingErrorRad;c.max_lag_s=0;c.settle_speed_rad_s=motion::operational_walk::kStreamSettleSpeedRadS;
  c.local_path_box=true;c.max_step_rad=feedback_policy_.maxJointStepRad;
  c.feedback_max_age_us=MotionExecutor::kTimedFeedbackMaxAgeMs*1000;
  c.hold_timeout_us=executor_->trackingTimeoutUs();c.max_frame_gap_us=MotionExecutor::kTimedMaxFrameGapMs*1000;c.persist_samples=3;
  c.region_min_rad=stand_min_;c.region_max_rad=stand_max_;
  if(!executor_->beginStream(c,fb,now_us)){executor_->stop("STREAM_BEGIN_REFUSED");return;}
  stand_first_release_us_=now_us+period;
  scheduler_.start(stand_first_release_us_,period);return;
 }
 if(now_us<stand_first_release_us_)return;
 if(!frame_.target.valid||!frame_.executable||!frame_.physicalReadiness||frame_.target.sequence==submitted_sequence_)return;
 // The assessment retains the original collision/swept/support certificate.
 FrameValidation v;v.sequence=frame_.target.sequence;v.policy_epoch=policy_->epoch();v.geometry=policy_->currentGeometryTag();
 v.collision_clear=assessment_.sweptCollisionClear;v.support_valid=assessment_.supportCompatible;
 v.path_continuous=assessment_.continuousBoundsConfirmed;v.configuration_valid=assessment_.geometryIdentityVerified;
 v.final=stand_.pendingFinal();
 if(executor_->stream(frame_.target,v,now_us,stand_release_us_+period,v.final,facts,fb,now,now_us)){
  submitted_sequence_=frame_.target.sequence;stand_.noteSubmitted(frame_.target.sequence);
 }
}
void MotionCoordinator::update(const MotionExecutionFacts& facts,const JointFeedback& fb,uint32_t now,uint64_t now_us){
 if(executor_&&executor_->status().safe_off_required)pending_walk_=false;
 synchronizeSession();
 if(pending_stand_&&executor_){
  if(executor_->active()&&session_&&session_->state()==MotionSessionState::ARMED){
   pending_stand_=false;
   if(const char* why=startStand(pending_command_,now)){pending_walk_=false;entry_reason_=why;arming_failure_=why;if(session_)session_->revoke(why);return;}
  } else if(!session_||session_->state()==MotionSessionState::OFF||session_->state()==MotionSessionState::SAFE_OFF_REQUIRED||
            static_cast<uint32_t>(now-pending_since_ms_)>15000){
   pending_walk_=false;pending_stand_=false;entry_reason_=session_&&session_->state()!=MotionSessionState::OFF?session_->reason():
     session_&&*session_->lastRevoke()?session_->lastRevoke():"ARMING_FAILED";
   arming_failure_=static_cast<uint32_t>(now-pending_since_ms_)>15000&&session_&&session_->state()!=MotionSessionState::OFF&&
     session_->state()!=MotionSessionState::SAFE_OFF_REQUIRED?"ARMING_TIMEOUT":entry_reason_;
   if(session_&&session_->state()!=MotionSessionState::OFF&&session_->state()!=MotionSessionState::SAFE_OFF_REQUIRED)session_->revoke("ARMING_TIMEOUT");
  }
 }
 if(!configured_||!executor_||!policy_||!executor_->active())return;
 if(fb.pending()){
  if(executor_->streaming())executor_->monitor(facts,fb,now,now_us);
  return; // hold the current target; never advance an uncertain trajectory
 }

 if(!walking_&&executor_->timedSegmentsActive()&&
    (stand_.state()==motion::AdaptiveStandState::STAND||stand_.state()==motion::AdaptiveStandState::VERIFY_STAND)){
  executor_->monitor(facts,fb,now,now_us);
  observation_.stationary=executor_->measuredStandQuiet();
 }
 if(pending_walk_&&stand_.state()==motion::AdaptiveStandState::STAND&&executor_->status().state==ExecutorState::READY){
  auto command=pending_walk_command_;pending_walk_=false;
  command.issued_at_ms=now;
  if(const char* why=accept(command,now)){entry_reason_=why;executor_->stop(why);return;}
 }

 if(walking_&&active_spec_.timeDriven){streamWalk(facts,fb,now,now_us);return;}
 const bool timed=!walking_&&executor_->timedSegmentsActive();
 if(!walking_&&(timed||executor_->streaming())){streamStand(facts,fb,now,now_us);return;}
 auto o=observation_;o.acknowledged=executor_->status().completed_sequence!=0;o.acknowledgedSequence=executor_->status().completed_sequence;
 // Observations remain externally measured. Targets never overwrite actual
 // joints, base height or contact evidence in order to manufacture tracking.
 frame_=walking_?walk_.update(o,(now_us/1e6)):stand_.update(o,(now_us/1e6));
 // The certified entry just verified its 150 mm STAND and a lower height was requested:
 // continue in the SAME update, so the command is never completed at the intermediate height.
 if(!walking_&&stand_.state()==motion::AdaptiveStandState::STAND&&commissioning_.measuredStart&&std::abs(final_height_m_-stand_.standHeightM())>1e-9&&
    executor_->status().state==ExecutorState::READY){
  if(!stand_.requestHeightAdjust(final_height_m_,executor_->status().accepted_sequence+1,now_us/1e6)){executor_->stop("HEIGHT_ADJUST_REFUSED");return;}
 }
 if(walking_&&walk_.state()==motion::FeedbackWalkState::STAND){walking_=false;executor_->restoreArmedRates();return;}
 if((walking_&&walk_.state()==motion::FeedbackWalkState::REJECTED)||(!walking_&&stand_.state()==motion::AdaptiveStandState::REJECTED)){executor_->stop(motion::feedbackReasonName(frame_.reason));return;}
 if(feedback_policy_.mode==motion::FeedbackMode::SIMULATION||!frame_.physicalReadiness)return;
 if(!frame_.target.valid||!frame_.executable||frame_.target.sequence==submitted_sequence_)return;
 FrameValidation v;v.sequence=frame_.target.sequence;v.policy_epoch=policy_->epoch();v.geometry=policy_->currentGeometryTag();v.collision_clear=assessment_.sweptCollisionClear;v.support_valid=assessment_.supportCompatible;v.path_continuous=assessment_.continuousBoundsConfirmed;v.configuration_valid=assessment_.geometryIdentityVerified;
 const uint32_t ttl=static_cast<uint32_t>(feedback_policy_.segmentTimeoutS*1000);
 v.final=walking_||stand_.pendingFinal();
 if(executor_->submit(frame_.target,v,now,ttl,feedback_policy_.samplePeriodS,facts,fb,now)){
  submitted_sequence_=frame_.target.sequence;if(!walking_)stand_.noteSubmitted(frame_.target.sequence);
 }
}
} }
