#include "MotionCoordinator.h"
#include <cmath>
#include "../motion/FixturePoseClassifier.h"
namespace matdog { namespace execution {
void MotionCoordinator::begin(MotionExecutor* e,actuator::SafeActuatorPolicy* p){executor_=e;policy_=p;configured_=false;walking_=false;submitted_sequence_=0;arm_generation_=0;}
void MotionCoordinator::synchronizeSession(){
 if(!executor_||arm_generation_==executor_->status().arm_generation)return;
 arm_generation_=executor_->status().arm_generation;
 submitted_sequence_=0;assessment_={};frame_=motion::FeedbackFrame{};walking_=false;
 stand_.stop();walk_=motion::FeedbackWalk{};
}
bool MotionCoordinator::configure(const motion::FeedbackPolicy& p,const motion::WalkSpecification& w,const motion::AcquisitionRoute* a){
 if(executor_&&executor_->active())return false;
 if(!motion::validFeedbackPolicy(p))return false;
 feedback_policy_=p;walk_specification_=w;acquisition_=a?*a:motion::AcquisitionRoute{};configured_=true;return true;
}
bool MotionCoordinator::configureCommissioning(const CommissioningStandConfig& c){
 if(executor_&&executor_->active())return false;
 if(!c.enabled){clearCommissioning();return true;}
 if(!configured_||feedback_policy_.mode!=motion::FeedbackMode::FIXTURE_REVIEW||!feedback_policy_.fixtureLimitVariant||!acquisition_.pairedProne||
    (!c.measuredStart&&!c.certificate.present)||c.keepoutCount>CommissioningStandConfig::kMaxKeepouts||
    (c.measuredStart&&(!(c.proneMaxDeviationRad<=CommissioningStandConfig::kCertifiedBandRad+1e-12)||
                       !(feedback_policy_.maxTrackingErrorRad<=CommissioningStandConfig::kCertifiedTubeRad+1e-12)))||
    !std::isfinite(c.pathMarginRad)||c.pathMarginRad<=0||!std::isfinite(c.minStageDurationS)||c.minStageDurationS<=0||
    !std::isfinite(c.proneMaxDeviationRad)||!std::isfinite(c.proneMaxRmsRad)||c.proneMaxRmsRad<=0||c.proneMaxRmsRad>c.proneMaxDeviationRad)return false;
 commissioning_=c;entry_plan_=motion::FixtureEntryPlan{};return true;
}
void MotionCoordinator::observe(const motion::AdaptiveObservation& o){observation_=o;}
bool MotionCoordinator::assess(const motion::SegmentAssessment& a,uint32_t now){
 const bool ok=walking_?walk_.assess(a,now/1000.0):stand_.assess(a,now/1000.0);
 if(ok)assessment_=a;
 return ok;
}
const char* MotionCoordinator::startStand(const motion::MotionCommand& c,uint32_t now){
   if(session_&&session_->scope()==TrialScope::FIXTURE_MICRO)return "AUTONOMOUS_STAND_SCOPE_REQUIRED";
   if(walking_)return "WALK_MUST_STOP_FIRST";
   motion::AdaptiveStandRequest r;r.targetHeightM=c.target_height_m;r.firstSequence=executor_->status().accepted_sequence+1;
   if(acquisition_.count||acquisition_.pairedProne)r.acquisition=&acquisition_;
   if(commissioning_.enabled){
    if(!session_||session_->scope()!=TrialScope::FIXTURE_COMMISSIONING)return "COMMISSIONING_SCOPE_REQUIRED";
    // The entry starts at the armed (held) pose, never at a nominal knot.
    motion::JointTargetFrame start;start.valid=true;
    for(unsigned i=0;i<12;++i){auto& q=start.legs[i/3];(i%3==0?q.hip:i%3==1?q.upper:q.lower)=executor_->targetRadians()[i];}
    const auto head=motion::pairedProneSample(0,0,true);if(!head.target.valid)return "ROUTE_HEAD_INVALID";
    if(commissioning_.measuredStart){
     motion::MeasuredEntryConfig m;m.keepouts=commissioning_.keepouts;m.keepoutCount=commissioning_.keepoutCount;m.pathMarginRad=commissioning_.pathMarginRad;
     entry_plan_=motion::planMeasuredEntry(start,head.target,feedback_policy_,m,commissioning_.minStageDurationS);
    } else entry_plan_=motion::planFixtureEntry(start,head.target,feedback_policy_,commissioning_.certificate,commissioning_.minStageDurationS);
    if(entry_plan_.status!=motion::EntryStatus::OK){entry_reason_=motion::entryStatusName(entry_plan_.status);return entry_reason_;}
    r.entry=&entry_plan_;r.proneMaxDeviationRad=commissioning_.proneMaxDeviationRad;r.proneMaxRmsRad=commissioning_.proneMaxRmsRad;
   }
   entry_reason_="";
   return stand_.request(r,feedback_policy_,now/1000.0)?nullptr:motion::feedbackReasonName(stand_.reason());
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
 if((!executor_||!executor_->active())&&executor_&&c.intent==motion::Intent::STAND&&commissioning_.enabled&&commissioning_.measuredStart&&armer_){
  // Operational STAND from READY: the explicit command starts the controlled
  // torque sequence; the planner starts only once the session is ARMED.
  if(const char* why=preArmCheck())return why;
  if(const char* why=armer_->armForCommand(now))return why;
  pending_stand_=true;pending_command_=c;pending_since_ms_=now;return nullptr;
 }
 if(!executor_||!executor_->active())return "EXECUTOR_NOT_ARMED";
 switch(c.intent){
  case motion::Intent::STAND:return startStand(c,now);
  case motion::Intent::GAIT_SELECT:return nullptr; // only WALK survived service validation
  case motion::Intent::BASE_VELOCITY:{
   if(session_&&session_->scope()!=TrialScope::AUTONOMOUS_WALK&&session_->scope()!=TrialScope::SUPERVISED_FIXTURE_WALK&&
      session_->scope()!=TrialScope::FIXTURE_COMMISSIONING)return "LOCAL_WALK_SCOPE_REQUIRED";
   motion::PlanarVelocityCommand v;v.vxMps=c.vx;v.vyMps=c.vy;v.yawRateRadS=c.yaw_rate;v.stampS=c.issued_at_ms/1000.0;v.ttlS=c.ttl_ms/1000.0;v.sequence=c.sequence;
   if(walking_)return walk_.command(v,now/1000.0)?nullptr:motion::feedbackReasonName(walk_.reason());
   if(v.vxMps==0&&v.vyMps==0&&v.yawRateRadS==0)return nullptr;
   auto spec=walk_specification_;spec.firstTargetSequence=executor_->status().accepted_sequence+1;
   if(!walk_.start(stand_,observation_,v,spec,feedback_policy_,now/1000.0))return motion::feedbackReasonName(walk_.reason());
   walking_=true;return nullptr;
  }
  default:return "ACTION_NOT_VALIDATED";
 }
}
void MotionCoordinator::stop(uint32_t){pending_stand_=false;if(session_&&(session_->scope()==TrialScope::SUPERVISED_FIXTURE_STAND||session_->scope()==TrialScope::SUPERVISED_FIXTURE_WALK||session_->scope()==TrialScope::FIXTURE_COMMISSIONING)){stand_.stop();session_->revoke("LOCAL_FIXTURE_STOP_SAFE_OFF");return;}if(walking_)walk_.requestStop();else {stand_.stop();if(session_)session_->revoke("LOCAL_OPERATOR_STOP");}}
const char* MotionCoordinator::state() const {
 if(!walking_)return motion::adaptiveStandStateName(stand_.state());
 switch(walk_.state()){case motion::FeedbackWalkState::IDLE:return "IDLE";case motion::FeedbackWalkState::STARTING:return "WALK_STARTING";case motion::FeedbackWalkState::WALK:return "WALK";case motion::FeedbackWalkState::STOPPING:return "STOPPING";case motion::FeedbackWalkState::STAND:return "STAND";default:return "REJECTED";}
}
void MotionCoordinator::update(const MotionExecutionFacts& facts,const JointFeedback& fb,uint32_t now){
 synchronizeSession();
 if(pending_stand_&&executor_){
  if(executor_->active()&&session_&&session_->state()==MotionSessionState::ARMED){
   pending_stand_=false;
   if(const char* why=startStand(pending_command_,now)){entry_reason_=why;if(session_)session_->revoke(why);return;}
  } else if(!session_||session_->state()==MotionSessionState::OFF||session_->state()==MotionSessionState::SAFE_OFF_REQUIRED||
            static_cast<uint32_t>(now-pending_since_ms_)>15000){
   pending_stand_=false;entry_reason_=session_&&session_->state()!=MotionSessionState::OFF?session_->reason():"ARMING_FAILED";
   if(session_&&session_->state()!=MotionSessionState::OFF&&session_->state()!=MotionSessionState::SAFE_OFF_REQUIRED)session_->revoke("ARMING_TIMEOUT");
  }
 }
 if(!configured_||!executor_||!policy_||!executor_->active())return;
 auto o=observation_;o.acknowledged=executor_->status().completed_sequence!=0;o.acknowledgedSequence=executor_->status().completed_sequence;
 // Observations remain externally measured. Targets never overwrite actual
 // joints, base height or contact evidence in order to manufacture tracking.
 frame_=walking_?walk_.update(o,now/1000.0):stand_.update(o,now/1000.0);
 if(walking_&&walk_.state()==motion::FeedbackWalkState::STAND){walking_=false;return;}
 if((walking_&&walk_.state()==motion::FeedbackWalkState::REJECTED)||(!walking_&&stand_.state()==motion::AdaptiveStandState::REJECTED)){executor_->stop(motion::feedbackReasonName(frame_.reason));return;}
 if(feedback_policy_.mode==motion::FeedbackMode::SIMULATION||!frame_.physicalReadiness)return;
 if(!frame_.target.valid||!frame_.executable||frame_.target.sequence==submitted_sequence_)return;
 FrameValidation v;v.sequence=frame_.target.sequence;v.policy_epoch=policy_->epoch();v.geometry=policy_->currentGeometryTag();v.collision_clear=assessment_.sweptCollisionClear;v.support_valid=assessment_.supportCompatible;v.path_continuous=assessment_.continuousBoundsConfirmed;v.configuration_valid=assessment_.geometryIdentityVerified;
 const uint32_t ttl=static_cast<uint32_t>(feedback_policy_.segmentTimeoutS*1000);
 if(executor_->submit(frame_.target,v,now,ttl,feedback_policy_.samplePeriodS,facts,fb,now))submitted_sequence_=frame_.target.sequence;
}
} }
