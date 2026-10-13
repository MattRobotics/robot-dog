#include "MotionCoordinator.h"
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
void MotionCoordinator::observe(const motion::AdaptiveObservation& o){observation_=o;}
bool MotionCoordinator::assess(const motion::SegmentAssessment& a,uint32_t now){
 const bool ok=walking_?walk_.assess(a,now/1000.0):stand_.assess(a,now/1000.0);
 if(ok)assessment_=a;
 return ok;
}
const char* MotionCoordinator::accept(const motion::MotionCommand& c,uint32_t now){
 synchronizeSession();
 if(!configured_)return "PLANNER_ARTIFACT_REQUIRED";
 if(!executor_||!executor_->active())return "EXECUTOR_NOT_ARMED";
 switch(c.intent){
  case motion::Intent::STAND:{
   if(session_&&session_->scope()==TrialScope::FIXTURE_MICRO)return "AUTONOMOUS_STAND_SCOPE_REQUIRED";
   if(walking_)return "WALK_MUST_STOP_FIRST";
   motion::AdaptiveStandRequest r;r.targetHeightM=c.target_height_m;r.firstSequence=executor_->status().accepted_sequence+1;
   if(acquisition_.count||acquisition_.pairedProne)r.acquisition=&acquisition_;
   return stand_.request(r,feedback_policy_,now/1000.0)?nullptr:motion::feedbackReasonName(stand_.reason());
  }
  case motion::Intent::GAIT_SELECT:return nullptr; // only WALK survived service validation
  case motion::Intent::BASE_VELOCITY:{
   if(session_&&session_->scope()!=TrialScope::AUTONOMOUS_WALK&&session_->scope()!=TrialScope::SUPERVISED_FIXTURE_WALK)return "LOCAL_WALK_SCOPE_REQUIRED";
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
void MotionCoordinator::stop(uint32_t){if(session_&&(session_->scope()==TrialScope::SUPERVISED_FIXTURE_STAND||session_->scope()==TrialScope::SUPERVISED_FIXTURE_WALK)){stand_.stop();session_->revoke("LOCAL_FIXTURE_STOP_SAFE_OFF");return;}if(walking_)walk_.requestStop();else {stand_.stop();if(session_)session_->revoke("LOCAL_OPERATOR_STOP");}}
const char* MotionCoordinator::state() const {
 if(!walking_)return motion::adaptiveStandStateName(stand_.state());
 switch(walk_.state()){case motion::FeedbackWalkState::IDLE:return "IDLE";case motion::FeedbackWalkState::STARTING:return "WALK_STARTING";case motion::FeedbackWalkState::WALK:return "WALK";case motion::FeedbackWalkState::STOPPING:return "STOPPING";case motion::FeedbackWalkState::STAND:return "STAND";default:return "REJECTED";}
}
void MotionCoordinator::update(const MotionExecutionFacts& facts,const JointFeedback& fb,uint32_t now){
 synchronizeSession();
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
