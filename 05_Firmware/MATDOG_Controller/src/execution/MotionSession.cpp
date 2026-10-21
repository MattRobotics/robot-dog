#include "MotionSession.h"
#include "../servo/ServoPopulation.h"
#include <cmath>
#include "../motion/TimedStand.h"
namespace matdog { namespace execution {
void MotionSession::begin(actuator::SafeActuatorPolicy* p,actuator::ActuatorRuntime* r,MotionExecutor* e,core::ActuatorAuthorityArbiter* a,
 const actuator::CalibrationGeometryProfile* g,const actuator::GeometryProvenance* v){policy_=p;runtime_=r;executor_=e;arbiter_=a;geometry_=g;provenance_=v;}
const char* MotionSession::gate(const LocalTrialEvidence& e,const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now) const {
 if(!policy_||!runtime_||!executor_||!arbiter_||!geometry_||!provenance_)return "NOT_BOUND";
 // Runtime conditions only: external qualification is not a motion permit.
 if(!e.imu_fresh)return e.scope_loss?e.scope_loss:"IMU_UNAVAILABLE";
 if(!f.calibration_accepted||!f.envelopes_approved)return "RESTORE_OR_LIMITS_REQUIRED";
 if(f.mode!=core::OperatingMode::RUN||f.fault||!f.power_ok||!f.bms_ok||arbiter_->inhibited())return "MODE_POWER_FAULT";
 if(!f.hardware_qualified||!f.sequential_bus_qualified)return "LIVE_HARDWARE_UNAVAILABLE";
 if(state_!=MotionSessionState::OFF && (policy_->epoch()!=epoch_||arbiter_->generation()!=lease_.generation||
    arbiter_->current()!=core::ActuatorAuthority::MOTION))return "SESSION_CHANGED";
 if(!geometry_->provenanceMatches(*provenance_))return "GEOMETRY_MISMATCH";
 for(unsigned i=0;i<12;++i){
  if(static_cast<int32_t>(now-fb.sampled_at_ms[i])<0)return "FEEDBACK_CLOCK_INVALID";
  if(now-fb.sampled_at_ms[i]>envelope_.feedback_max_age_ms)return "STALE_FEEDBACK";
  if((!fb.read_ok[i]&&!fb.recovering[i])||!std::isfinite(fb.actual_rad[i]))return "FEEDBACK_INVALID";
 }
 return nullptr;
}
bool MotionSession::resolve(unsigned i,double q,actuator::ActuatorCommand& cmd,uint8_t& bus) const {
 ResolvedJoint r;if(!policy_||!geometry_||!provenance_||resolveJoint(*policy_,*geometry_,*provenance_,i,q,&r)!=JointResolve::OK)return false;
 cmd=r.command;bus=r.bus;return true;
}
bool MotionSession::insideTick(unsigned i,double q,uint16_t max_ticks,double& inside,uint8_t& ticks) const {
 // Only a LOWER resting at its FOLDED (URDF-lower) end qualifies; anything
 // else outside its limit is an abnormal pose and stays refused.
 if(i>=12||i%3!=2||!std::isfinite(q)||max_ticks==0)return false;
 ResolvedJoint r;if(resolveJoint(*policy_,*geometry_,*provenance_,i,q,&r)!=JointResolve::OUTSIDE_LIMITS)return false;
 const auto* l=r.limit;if(l->max_tick-l->min_tick<4)return false;
 calibration::JointIdentity id;const auto* t=actuator::slotIdentity(i,&id)?policy_->transforms().find(id,geometry_->provenanceTag()):nullptr;if(!t)return false;
 // kPrimeInwardMarginTicks inside the edge, so a holding deadband cannot
 // place the measured position outside the limit (ACTUAL_OUTSIDE_LIMITS).
 const uint16_t tick=r.tick,target=tick<l->min_tick?l->min_tick+kPrimeInwardMarginTicks:l->max_tick-kPrimeInwardMarginTicks;
 const unsigned distance=tick<target?target-tick:tick-target;if(distance>max_ticks)return false;
 actuator::MicroRad urad=0;if(actuator::resolveRawToUrdfQ(*geometry_,*provenance_,*t,target,&urad)!=actuator::TargetResolveStatus::OK)return false;
 // The inward target must move AWAY from the folded stop (q increases); compared on the integer micro-radian grid.
 actuator::MicroRad rest=0;if(!actuator::radiansToMicroRad(q,&rest)||urad<=rest)return false;
 inside=actuator::microRadToRadians(urad);ticks=static_cast<uint8_t>(distance);return true;
}
const char* MotionSession::planPrime(const double measured[12],TrialScope scope,uint16_t clamp_max_ticks,PrimePlan* out) const {
 if(!out||!policy_||!geometry_||!provenance_)return "NOT_BOUND";
 PrimePlan p;
 for(unsigned i=0;i<12;++i){actuator::ActuatorCommand cmd;uint8_t bus;
  p.rest_rad[i]=p.hold_rad[i]=measured[i];
  if(resolve(i,measured[i],cmd,bus))continue;
  // Commissioning only: a measured rest just outside the hard limit (LOWER
  // folded at its stop) primes at the nearest inside tick, never beyond the
  // limit and never further than the evidence allows. The Safe Actuator layer
  // still checks every target.
  uint8_t ticks=0;
  if(scope!=TrialScope::FIXTURE_COMMISSIONING||!clamp_max_ticks||
     !insideTick(i,measured[i],clamp_max_ticks,p.hold_rad[i],ticks)||!resolve(i,p.hold_rad[i],cmd,bus))return "PRIME_REQUIRES_OFF_AND_LIMITS";
  p.clamp_mask|=static_cast<uint16_t>(1u<<i);p.clamp_ticks[i]=ticks;}
 *out=p;return nullptr;
}
bool MotionSession::request(const LocalTrialEvidence& e,const MotionExecutionFacts& f,const ExecutionEnvelope& envelope,const JointFeedback& fb,bool image,uint32_t now){
 if(state_!=MotionSessionState::OFF){reason_="SESSION_BUSY";return false;}
 if(!image){reason_="MOTION_AUTHORIZED_0";return false;}
 envelope_=envelope;
 if(!envelope.approved||!envelope.feedback_max_age_ms||envelope.feedback_max_age_ms>3000||
    !envelope.batch_timeout_ms||envelope.batch_timeout_ms>3000||!envelope.tracking_timeout_ms||envelope.tracking_timeout_ms>30000||
    !std::isfinite(envelope.max_velocity_rad_s)||envelope.max_velocity_rad_s<=0||
    !std::isfinite(envelope.max_acceleration_rad_s2)||envelope.max_acceleration_rad_s2<=0||!std::isfinite(envelope.tracking_tolerance_rad)||
    envelope.tracking_tolerance_rad<=0){reason_="ENVELOPE_UNAPPROVED";return false;}
 if(const char* r=gate(e,f,fb,now)){reason_=r;return false;}
 if(fb.pending()){reason_="FEEDBACK_RETRY_PENDING";return false;}
 // PRIME writes goals with torque OFF only; the pose itself goes through the one PRIME plan.
 for(unsigned i=0;i<12;++i)if(fb.torque_on[i]){reason_="PRIME_REQUIRES_OFF_AND_LIMITS";return false;}
 PrimePlan plan;if(const char* r=planPrime(fb.actual_rad,e.scope,e.prime_clamp_max_ticks,&plan)){reason_=r;return false;}
 for(unsigned i=0;i<12;++i){primed_rad_[i]=plan.hold_rad[i];rest_rad_[i]=plan.rest_rad[i];clamp_ticks_[i]=plan.clamp_ticks[i];}
 clamp_mask_=plan.clamp_mask;
 // The Controller may already hold the MOTION lease idle (READY with
 // authority granted): adopt exactly that lease, never mint a second one.
 if(f.lease.valid()&&f.lease.owner==core::ActuatorAuthority::MOTION&&arbiter_->current()==core::ActuatorAuthority::MOTION&&
    arbiter_->generation()==f.lease.generation)lease_=f.lease;
 else if(arbiter_->request(core::ActuatorAuthority::MOTION,f.mode,&lease_)!=core::AuthorityResult::GRANTED){reason_="EXCLUSIVE_OWNER_REQUIRED";return false;}
 evidence_=e;epoch_=policy_->epoch();permit_generation_=f.permit_generation;next_=0;started_at_ms_=now;
 write_attempts_=0;failed_joint_=-1;failure_detail_="";last_revoke_="";
 // PRIME failure records describe the latest admitted session only; a full or
 // stale buffer must never be able to refuse or revoke a session.
 prime_diagnostics_.clear();prime_diagnostics_.used=true;
 state_=MotionSessionState::PRIMING;reason_="PRIMING_PRESENT";return true;
}
bool MotionSession::recordPrimeRejection(uint8_t bus,uint16_t target,uint32_t now,const char* reason,bool backend_called){
 auto* record=prime_diagnostics_.append();if(!record)return false;
 *record=diagnostics::PrimeRejection{};record->slot=next_;record->id=bus;record->attempt=write_attempts_+1;
 record->target_position=target;record->session_ms=now;record->reason=reason;
 if(backend_called&&runtime_&&runtime_->lastGoalWriteCapture(&record->goal)){
  // Never attach a previous joint's cached transaction to a new rejection.
  if(record->goal.id!=bus||record->goal.position!=target)record->goal=diagnostics::GoalWriteCapture{};
 }
 return true;
}
bool MotionSession::requestFixtureMicro(unsigned joint,double delta,double duration,const FrameValidation& path,
 const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now){
 if(state_!=MotionSessionState::ARMED||evidence_.scope!=TrialScope::FIXTURE_MICRO||micro_active_||
    executor_->status().state!=ExecutorState::READY){reason_="FIXTURE_ARMED_REQUIRED";return false;}
 if(const char* r=gate(evidence_,f,fb,now)){revoke(r);return false;}
 if(joint>=12||!std::isfinite(delta)||!std::isfinite(duration)||duration<=0||duration>30||
    !std::isfinite(evidence_.max_fixture_excursion_rad)||evidence_.max_fixture_excursion_rad<=0||
    std::abs(delta)>evidence_.max_fixture_excursion_rad||delta==0||
    1.875*std::abs(delta)/duration>envelope_.max_velocity_rad_s||
    5.773502692*std::abs(delta)/(duration*duration)>envelope_.max_acceleration_rad_s2){reason_="MICRO_ENVELOPE";return false;}
 if(path.policy_epoch!=policy_->epoch()||path.geometry!=policy_->currentGeometryTag()||
    !path.collision_clear||!path.support_valid||!path.path_continuous||!path.configuration_valid){reason_="FIXTURE_PATH_CERTIFICATE_REQUIRED";return false;}
 micro_start_={};micro_start_.valid=true;
 for(unsigned i=0;i<4;++i)micro_start_.legs[i]={fb.actual_rad[i*3],fb.actual_rad[i*3+1],fb.actual_rad[i*3+2]};
 actuator::ActuatorCommand command;uint8_t bus;
 if(!resolve(joint,fb.actual_rad[joint]+delta,command,bus)){reason_="MICRO_TARGET_LIMIT";return false;}
 micro_path_=path;micro_joint_=joint;micro_delta_=delta;micro_duration_=duration;micro_intervals_=static_cast<unsigned>(std::ceil(duration/.05));
 micro_step_=0;micro_sequence_=executor_->status().completed_sequence;micro_issued_at_ms_=now;micro_active_=true;reason_="FIXTURE_MICRO_ACTIVE";return true;
}
void MotionSession::revoke(const char* r){micro_active_=false;reason_=r;if(state_!=MotionSessionState::OFF&&state_!=MotionSessionState::SAFE_OFF_REQUIRED)last_revoke_=r;
 if(state_!=MotionSessionState::OFF){state_=MotionSessionState::SAFE_OFF_REQUIRED;motion::RefusalRecord why;why.site="SESSION";executor_->stop(r,&why);}}
void MotionSession::confirmSafeOff(){
 if(state_!=MotionSessionState::SAFE_OFF_REQUIRED)return;
 arbiter_->release(lease_);lease_={};state_=MotionSessionState::OFF;reason_="SAFE_OFF_VERIFIED";
}
void MotionSession::update(const LocalTrialEvidence& e,const MotionExecutionFacts& input,const JointFeedback& fb,uint32_t now){
 if(state_==MotionSessionState::OFF||state_==MotionSessionState::SAFE_OFF_REQUIRED)return;
 if(const char* r=gate(e,input,fb,now)){revoke(r);return;}
 if(fb.pending()){
  if(state_!=MotionSessionState::ARMED&&now-started_at_ms_>=envelope_.batch_timeout_ms)revoke("PRIME_ARM_TIMEOUT");
  return;
 }
 auto f=input;f.lease=lease_;
 if(state_==MotionSessionState::ARMED){
  if(!executor_->active()){revoke(executor_->status().reason);return;}
  if(!micro_active_)return;
  if(now-micro_issued_at_ms_>static_cast<uint32_t>(envelope_.tracking_timeout_ms+1000)){revoke("MICRO_TIMEOUT");return;}
  if(executor_->status().state!=ExecutorState::READY||executor_->status().completed_sequence!=micro_sequence_)return;
  if(micro_step_==micro_intervals_){micro_active_=false;reason_="FIXTURE_MICRO_TRACKED";return;}
  const auto law=motion::quinticTimeLaw(static_cast<double>(micro_step_+1)/micro_intervals_,micro_duration_);
  auto target=micro_start_;target.sequence=micro_sequence_+1;
  auto& q=target.legs[micro_joint_/3];double* value=micro_joint_%3==0?&q.hip:micro_joint_%3==1?&q.upper:&q.lower;
  *value+=micro_delta_*law.progress;
  auto v=micro_path_;v.sequence=target.sequence;
  const double dt=micro_duration_/micro_intervals_;
  if(!executor_->submit(target,v,now,1000,dt,f,fb,now)){revoke(executor_->status().reason);return;}
  ++micro_step_;micro_sequence_=target.sequence;micro_issued_at_ms_=now;return;
 }
 if(now-started_at_ms_>=envelope_.batch_timeout_ms){revoke("PRIME_ARM_TIMEOUT");return;}
 // A clamped joint may only travel between its measured rest and its primed
 // inside tick; every other joint must stay at its readback.
 for(unsigned i=0;i<12;++i){const double lo=std::fmin(rest_rad_[i],primed_rad_[i])-envelope_.tracking_tolerance_rad,hi=std::fmax(rest_rad_[i],primed_rad_[i])+envelope_.tracking_tolerance_rad;
  if(fb.actual_rad[i]<lo||fb.actual_rad[i]>hi){revoke("PRIME_DRIFT");return;}}
 if(state_==MotionSessionState::ARMING){
  // Old pre-prime snapshots cannot prove the holding drive profile. Wait
  // within the admission deadline for all twelve post-energize readbacks.
  if(!input.prime_profiles_verified)return;
  bool fresh=true;for(unsigned i=0;i<12;++i)fresh=fresh&&fb.torque_on[i]&&static_cast<int32_t>(fb.sampled_at_ms[i]-started_at_ms_)>0;
  if(!fresh)return;
  // Clamped joints must have settled at their inside target before arm.
  for(unsigned i=0;i<12;++i)if(std::abs(fb.actual_rad[i]-primed_rad_[i])>envelope_.tracking_tolerance_rad)return;
  // The executor holds the PRIME goals (what the servos were commanded), with the expected contact of each clamped LOWER.
  const ArmHold hold{primed_rad_,rest_rad_,clamp_mask_};
  if(!executor_->arm(f,envelope_,fb,now,&hold)){revoke(executor_->status().reason);return;}
  state_=MotionSessionState::ARMED;reason_="ARMED";return;
 }
 actuator::ActuatorCommand cmd;uint8_t bus;if(!resolve(next_,primed_rad_[next_],cmd,bus)){revoke("PRIME_MAPPING_OR_LIMITS");return;}
 if(state_==MotionSessionState::PRIMING && fb.torque_on[next_]){revoke("TORQUE_CHANGED_DURING_PRIME");return;}
 if(state_==MotionSessionState::ENERGIZING)cmd.operation=actuator::ActuatorOperation::TORQUE_ENABLE;
 const bool energize=state_==MotionSessionState::ENERGIZING;
 actuator::ActuatorTransaction tx;
 const auto decision=policy_->plan(cmd,lease_,f.mode,&tx);
 if(decision!=actuator::WriteDecision::ACCEPT){
  failed_joint_=static_cast<int8_t>(next_);failure_detail_=actuator::toString(decision);
  if(!energize)recordPrimeRejection(bus,cmd.target_tick,now,failure_detail_,false);
  revoke(energize?"ENERGIZE_POLICY_REJECTED":"PRIME_POLICY_REJECTED");return;}
 actuator::WriteDecision commit=actuator::WriteDecision::ACCEPT;
 const auto result=runtime_->execute(&tx,bus,&commit);
 // An admitted STOP can revoke this session from the UART wait callback.
 // Do not overwrite that decision or consume a retry after unwinding it.
 if(state_==MotionSessionState::SAFE_OFF_REQUIRED)return;
 if(result!=actuator::ExecuteResult::WRITTEN){
  failed_joint_=static_cast<int8_t>(next_);
  failure_detail_=result==actuator::ExecuteResult::NOT_EXECUTED?actuator::toString(commit):actuator::toString(result);
  const bool backend=result==actuator::ExecuteResult::BACKEND_REJECTED||result==actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF;
  if(!energize)recordPrimeRejection(bus,cmd.target_tick,now,failure_detail_,backend);
  if(backend&&++write_attempts_<kWriteAttempts){reason_=energize?"ENERGIZE_WRITE_RETRY":"PRIME_WRITE_RETRY";return;}
  revoke(result==actuator::ExecuteResult::NOT_EXECUTED?(energize?"ENERGIZE_COMMIT_REJECTED":"PRIME_COMMIT_REJECTED"):
         result==actuator::ExecuteResult::BACKEND_REJECTED?(energize?"ENERGIZE_BACKEND_NOT_APPLIED":"PRIME_BACKEND_NOT_APPLIED"):
         result==actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF?(energize?"ENERGIZE_BACKEND_NO_RESPONSE":"PRIME_BACKEND_NO_RESPONSE"):
         (energize?"ENERGIZE_BACKEND_UNAVAILABLE":"PRIME_BACKEND_UNAVAILABLE"));return;}
 write_attempts_=0;
 if(++next_==12){next_=0;started_at_ms_=now;state_=state_==MotionSessionState::PRIMING?MotionSessionState::ENERGIZING:MotionSessionState::ARMING;}
}
} }
