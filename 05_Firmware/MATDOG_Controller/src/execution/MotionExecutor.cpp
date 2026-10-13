#include "MotionExecutor.h"
#include "../servo/ServoPopulation.h"
#include <cmath>
#include <cstring>
namespace matdog { namespace execution {
const char* toString(ExecutorState s) { switch(s){case ExecutorState::OFF:return "OFF";case ExecutorState::READY:return "READY";case ExecutorState::WRITING:return "WRITING";case ExecutorState::TRACKING:return "TRACKING";case ExecutorState::SAFE_OFF_REQUIRED:return "SAFE_OFF_REQUIRED";default:return "FAULT";} }
void MotionExecutor::begin(actuator::SafeActuatorPolicy* p,actuator::ActuatorRuntime* r,const actuator::CalibrationGeometryProfile* g,const actuator::GeometryProvenance* v,const core::ActuatorAuthorityArbiter* a) { policy_=p;runtime_=r;geometry_=g;provenance_=v;arbiter_=a;status_=ExecutorStatus{}; }
const char* MotionExecutor::gate(const MotionExecutionFacts& f,uint32_t now) const {
 if(!policy_||!runtime_||!geometry_||!provenance_||!arbiter_)return "NOT_BOUND";
 if(f.fault)return "FAULT";
 if(arbiter_->inhibited())return "INHIBITED";
 if(!f.calibration_accepted)return "CALIBRATION_NOT_ACCEPTED";
 if(!f.envelopes_approved)return "LIMITS_NOT_APPROVED";
 if(!f.hardware_qualified||!f.sequential_bus_qualified)return "HARDWARE_NOT_QUALIFIED";
 if(!f.operator_permit||!f.permit_generation||static_cast<int32_t>(now-f.permit_expires_at_ms)>=0)return "PERMIT_REVOKED_OR_EXPIRED";
 if(!f.bms_ok||!f.power_ok)return "POWER_OR_BMS";
 if(f.mode!=core::OperatingMode::RUN)return "MODE";
 if(f.lease.owner!=core::ActuatorAuthority::MOTION||(arbiter_->current()!=f.lease.owner||arbiter_->generation()!=f.lease.generation))return "AUTHORITY_REVOKED";
 if(!geometry_->provenanceMatches(*provenance_)||policy_->currentGeometryTag()!=geometry_->provenanceTag())return "GEOMETRY_MISMATCH";
 if(active()&&(permit_generation_!=f.permit_generation||lease_.generation!=f.lease.generation||policy_epoch_!=policy_->epoch()))return "SESSION_CHANGED";
 return nullptr;
}
bool MotionExecutor::feedbackValid(const JointFeedback& f,uint32_t now) const {
 for(unsigned i=0;i<12;i++)if(!f.read_ok[i]||!f.torque_on[i]||!std::isfinite(f.actual_rad[i])||static_cast<int32_t>(now-f.sampled_at_ms[i])<0||now-f.sampled_at_ms[i]>envelope_.feedback_max_age_ms)return false;
 return true;
}
bool MotionExecutor::reject(const char* reason) { status_.reason=reason; if(active())stop(reason);return false; }
bool MotionExecutor::arm(const MotionExecutionFacts& f,const ExecutionEnvelope& e,const JointFeedback& fb,uint32_t now) {
 if(status_.state!=ExecutorState::OFF)return reject("EXECUTOR_BUSY");
 if(const char* reason=gate(f,now))return reject(reason);
 if(!e.approved||!std::isfinite(e.max_velocity_rad_s)||!std::isfinite(e.max_acceleration_rad_s2)||!std::isfinite(e.tracking_tolerance_rad)||e.max_velocity_rad_s<=0||e.max_acceleration_rad_s2<=0||e.tracking_tolerance_rad<=0||!e.feedback_max_age_ms||!e.batch_timeout_ms||!e.tracking_timeout_ms||e.feedback_max_age_ms>3000||e.batch_timeout_ms>3000||e.tracking_timeout_ms>30000)return reject("ENVELOPE_UNAPPROVED");
 envelope_=e;if(!feedbackValid(fb,now))return reject("FEEDBACK_OR_TORQUE");
 // A separately supervised prime/torque procedure is required before arm.
 // Arm itself issues no torque or position writes.
 for(unsigned i=0;i<12;i++){actuator::ActuatorCommand cmd;uint8_t bus;if(!resolve(i,fb.actual_rad[i],&cmd,&bus))return reject("START_OUTSIDE_LIMITS");previous_[i]=target_[i]=fb.actual_rad[i];velocity_[i]=0;}
 lease_=f.lease;permit_generation_=f.permit_generation;policy_epoch_=policy_->epoch();
 if(++arm_generation_==0)++arm_generation_;
 status_=ExecutorStatus{};status_.arm_generation=arm_generation_;status_.state=ExecutorState::READY;status_.reason="ARMED";return true;
}
bool MotionExecutor::resolve(uint8_t i,double q,actuator::ActuatorCommand* cmd,uint8_t* bus) const {
 if(i>=12||!cmd||!bus||!std::isfinite(q)||std::fabs(q)>2000)return false;
 // Canonical allocation is lower/upper/hip, semantic frames are hip/upper/lower.
 const auto* servo=servo::legServoAt((i/3)*3+(2-i%3));if(!servo)return false;
 calibration::JointIdentity identity;identity.leg=static_cast<calibration::Leg>(i/3);identity.joint=static_cast<calibration::JointKind>(i%3);std::strncpy(identity.physical_unit,servo->physical_unit,sizeof(identity.physical_unit)-1);
 const auto* j=geometry_->findJoint(identity);if(!j||j->bus_id!=servo->bus_id)return false;
 const auto* t=policy_->transforms().find(identity,policy_->currentGeometryTag());
 const auto* l=policy_->limits().find(identity,policy_->currentGeometryTag());if(!t||!l)return false;
 const auto urad=static_cast<actuator::MicroRad>(std::llround(q*1e6));uint16_t tick=0;
 if(actuator::resolveUrdfQToRaw(*geometry_,*provenance_,*t,urad,&tick)!=actuator::TargetResolveStatus::OK||!l->contains(tick))return false;
 *cmd=actuator::ActuatorCommand{};cmd->operation=actuator::ActuatorOperation::POSITION_COMMAND;cmd->joint=identity;cmd->target_tick=tick;cmd->target_urad=urad;*bus=j->bus_id;return true;
}
bool MotionExecutor::submit(const motion::JointTargetFrame& frame,const FrameValidation& v,uint32_t issued,uint32_t ttl,double dt,const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now) {
 if(status_.state!=ExecutorState::READY)return reject("EXECUTOR_BUSY");
 if(const char* reason=gate(f,now))return reject(reason);
 if(!frame.valid||!frame.sequence||static_cast<int32_t>(frame.sequence-status_.accepted_sequence)<=0)return reject("FRAME_SEQUENCE");
 if(!ttl||ttl>30000||static_cast<int32_t>(now-issued)<0||now-issued>=ttl)return reject("FRAME_EXPIRED");
 if(!std::isfinite(dt)||dt<=0||dt>30)return reject("SEGMENT_DURATION");
 if(v.sequence!=frame.sequence||v.policy_epoch!=policy_->epoch()||v.geometry!=policy_->currentGeometryTag()||!v.collision_clear||!v.support_valid||!v.path_continuous||!v.configuration_valid)return reject("SEGMENT_UNVALIDATED");
 if(!feedbackValid(fb,now))return reject("STALE_FEEDBACK");
 for(unsigned i=0;i<12;i++){actuator::ActuatorCommand actual;uint8_t bus;if(!resolve(i,fb.actual_rad[i],&actual,&bus))return reject("ACTUAL_OUTSIDE_LIMITS");}
 double candidate[12];
 for(unsigned leg=0;leg<4;leg++){candidate[leg*3]=frame.legs[leg].hip;candidate[leg*3+1]=frame.legs[leg].upper;candidate[leg*3+2]=frame.legs[leg].lower;}
 // Check the entire frame BEFORE the first write. Late failures cannot leave
 // eleven legal joints executing while the twelfth was never validated.
 for(unsigned i=0;i<12;i++) {
  actuator::ActuatorCommand cmd;uint8_t bus;if(!resolve(i,candidate[i],&cmd,&bus))return reject("TARGET_OR_MAPPING");
  if(std::fabs(fb.actual_rad[i]-previous_[i])>envelope_.tracking_tolerance_rad)return reject("TRACKING_ERROR");
  const double vel=(candidate[i]-previous_[i])/dt,acc=(vel-velocity_[i])/dt;
  if(std::fabs(vel)>envelope_.max_velocity_rad_s||std::fabs(acc)>envelope_.max_acceleration_rad_s2)return reject("RATE_LIMIT");
  pending_velocity_[i]=vel;
 }
 for(unsigned i=0;i<12;i++)target_[i]=candidate[i];
 validation_=v;deadline_ms_=issued+ttl;status_.accepted_sequence=frame.sequence;status_.submitted_at_ms=now;status_.next_joint=0;status_.state=ExecutorState::WRITING;status_.reason="EXECUTING";return true;
}
void MotionExecutor::update(const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now) {
 if(!active())return;
 if(const char* r=gate(f,now)){stop(r);return;}
 if(!feedbackValid(fb,now)){stop("STALE_FEEDBACK");return;}
 for(unsigned i=0;i<12;i++){actuator::ActuatorCommand actual;uint8_t bus;if(!resolve(i,fb.actual_rad[i],&actual,&bus)){stop("ACTUAL_OUTSIDE_LIMITS");return;}}
 if(status_.state==ExecutorState::READY){
  double error=0;for(unsigned i=0;i<12;i++)error=std::fmax(error,std::fabs(fb.actual_rad[i]-target_[i]));
  status_.max_tracking_error_rad=error;
  if(error>envelope_.tracking_tolerance_rad)stop("HOLD_TRACKING_ERROR");
  return;
 }
 if(static_cast<int32_t>(now-deadline_ms_)>=0){stop("COMMAND_TIMEOUT");return;}
 if(status_.state==ExecutorState::WRITING) {
  if(now-status_.submitted_at_ms>envelope_.batch_timeout_ms){stop("BUS_BATCH_TIMEOUT");return;}
  for(unsigned j=0;j<12;j++) {
   const double lo=std::fmin(previous_[j],target_[j])-envelope_.tracking_tolerance_rad;
   const double hi=std::fmax(previous_[j],target_[j])+envelope_.tracking_tolerance_rad;
   if(fb.actual_rad[j]<lo||fb.actual_rad[j]>hi){stop("TRACKING_DIVERGENCE");return;}
  }
  const unsigned i=status_.next_joint;actuator::ActuatorCommand cmd;uint8_t bus;
  if(!resolve(i,target_[i],&cmd,&bus)){stop("MAPPING_OR_LIMITS_CHANGED");return;}
  actuator::ActuatorTransaction tx;
  if(policy_->plan(cmd,f.lease,f.mode,&tx)!=actuator::WriteDecision::ACCEPT){stop("POLICY_REJECTED");return;}
  if(runtime_->execute(&tx,bus,nullptr)!=actuator::ExecuteResult::WRITTEN){stop("BACKEND_UNCERTAIN_OR_REJECTED");return;}
  if(++status_.next_joint==12){status_.state=ExecutorState::TRACKING;tracking_since_ms_=now;}
  return;
 }
 double error=0;for(unsigned i=0;i<12;i++)error=std::fmax(error,std::fabs(fb.actual_rad[i]-target_[i]));status_.max_tracking_error_rad=error;
 // Feedback must postdate all writes, including the last joint's write.
 bool fresh=true;for(unsigned i=0;i<12;i++)fresh=fresh&&static_cast<int32_t>(fb.sampled_at_ms[i]-tracking_since_ms_)>0;
 if(error<=envelope_.tracking_tolerance_rad&&fresh){for(unsigned i=0;i<12;i++){previous_[i]=target_[i];velocity_[i]=pending_velocity_[i];}status_.completed_sequence=status_.accepted_sequence;status_.state=ExecutorState::READY;status_.reason="TRACKED";}
 else if(now-tracking_since_ms_>=envelope_.tracking_timeout_ms)stop("TRACKING_TIMEOUT");
}
void MotionExecutor::stop(const char* r) { if(active()||status_.safe_off_required){status_.state=ExecutorState::SAFE_OFF_REQUIRED;status_.safe_off_required=true;}status_.reason=r; }
void MotionExecutor::confirmSafeOff(bool off) { if(status_.safe_off_required&&off){status_.safe_off_required=false;status_.state=ExecutorState::OFF;} }
} }
