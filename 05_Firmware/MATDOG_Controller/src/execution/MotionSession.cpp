#include "MotionSession.h"
#include "../servo/ServoPopulation.h"
#include <cmath>
#include <cstring>
#include "../motion/TimedStand.h"
namespace matdog { namespace execution {
void MotionSession::begin(actuator::SafeActuatorPolicy* p,actuator::ActuatorRuntime* r,MotionExecutor* e,core::ActuatorAuthorityArbiter* a,
 const actuator::CalibrationGeometryProfile* g,const actuator::GeometryProvenance* v){policy_=p;runtime_=r;executor_=e;arbiter_=a;geometry_=g;provenance_=v;}
const char* MotionSession::gate(const LocalTrialEvidence& e,const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now) const {
 if(!policy_||!runtime_||!executor_||!arbiter_||!geometry_||!provenance_)return "NOT_BOUND";
 if(!e.generation||static_cast<int32_t>(now-e.expires_at_ms)>=0||!e.local_operator_present||!e.physical_power_cut_verified||
    !e.installation_verified||!e.servo_output_ceiling_verified||!e.imu_fresh)return "LOCAL_TRIAL_EVIDENCE_REQUIRED";
 if(e.scope==TrialScope::FIXTURE_MICRO||e.scope==TrialScope::SUPERVISED_FIXTURE_STAND||e.scope==TrialScope::SUPERVISED_FIXTURE_WALK){if(!e.fixture_support_verified)return "FIXTURE_SUPPORT_REQUIRED";}
 else if(e.scope==TrialScope::AUTONOMOUS_STAND||e.scope==TrialScope::AUTONOMOUS_WALK){
   if(!e.support_load_observable)return "AUTONOMOUS_SUPPORT_LOAD_REQUIRED";
   if(e.scope==TrialScope::AUTONOMOUS_WALK&&!e.loaded_dynamics_qualified)return "WALK_DYNAMICS_REQUIRED";
 }
 else return "TRIAL_SCOPE_INVALID";
 if(!f.calibration_accepted||!f.envelopes_approved)return "RESTORE_OR_LIMITS_REQUIRED";
 if(!f.operator_permit||!f.permit_generation||static_cast<int32_t>(now-f.permit_expires_at_ms)>=0)return "PERMIT_REVOKED_OR_EXPIRED";
 if(f.mode!=core::OperatingMode::RUN||f.fault||!f.power_ok||!f.bms_ok||arbiter_->inhibited())return "MODE_POWER_FAULT";
 if(!f.hardware_qualified||!f.sequential_bus_qualified)return "SCOPED_HARDWARE_TIMING_REQUIRED";
 if(state_!=MotionSessionState::OFF && (e.generation!=evidence_.generation||e.scope!=evidence_.scope||
    f.permit_generation!=permit_generation_||policy_->epoch()!=epoch_||arbiter_->generation()!=lease_.generation||
    arbiter_->current()!=core::ActuatorAuthority::MOTION))return "SESSION_CHANGED";
 if(!geometry_->provenanceMatches(*provenance_))return "GEOMETRY_MISMATCH";
 for(unsigned i=0;i<12;++i)if(!fb.read_ok[i]||!std::isfinite(fb.actual_rad[i])||static_cast<int32_t>(now-fb.sampled_at_ms[i])<0||
    now-fb.sampled_at_ms[i]>envelope_.feedback_max_age_ms)return "STALE_FEEDBACK";
 return nullptr;
}
bool MotionSession::resolve(unsigned i,double q,actuator::ActuatorCommand& cmd,uint8_t& bus) const {
 if(i>=12||!std::isfinite(q))return false;
 const auto* servo=servo::legServoAt((i/3)*3+2-i%3);if(!servo)return false;
 calibration::JointIdentity id;id.leg=static_cast<calibration::Leg>(i/3);id.joint=static_cast<calibration::JointKind>(i%3);
 std::strncpy(id.physical_unit,servo->physical_unit,sizeof(id.physical_unit)-1);
 const auto* t=policy_->transforms().find(id,geometry_->provenanceTag());const auto* l=policy_->limits().find(id,geometry_->provenanceTag());
 const auto* g=geometry_->findJoint(id);if(!t||!l||!g||g->bus_id!=servo->bus_id)return false;
 const auto urad=static_cast<actuator::MicroRad>(std::llround(q*1e6));uint16_t tick=0;
 if(actuator::resolveUrdfQToRaw(*geometry_,*provenance_,*t,urad,&tick)!=actuator::TargetResolveStatus::OK||!l->contains(tick))return false;
 cmd={};cmd.operation=actuator::ActuatorOperation::POSITION_COMMAND;cmd.joint=id;cmd.target_tick=tick;cmd.target_urad=urad;bus=servo->bus_id;return true;
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
 for(unsigned i=0;i<12;++i){actuator::ActuatorCommand cmd;uint8_t bus;
  if(fb.torque_on[i]||!resolve(i,fb.actual_rad[i],cmd,bus)){reason_="PRIME_REQUIRES_OFF_AND_LIMITS";return false;}
  primed_rad_[i]=fb.actual_rad[i];}
 if(arbiter_->request(core::ActuatorAuthority::MOTION,f.mode,&lease_)!=core::AuthorityResult::GRANTED){reason_="EXCLUSIVE_OWNER_REQUIRED";return false;}
 evidence_=e;epoch_=policy_->epoch();permit_generation_=f.permit_generation;next_=0;started_at_ms_=now;
 state_=MotionSessionState::PRIMING;reason_="PRIMING_PRESENT";return true;
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
void MotionSession::revoke(const char* r){micro_active_=false;reason_=r;if(state_!=MotionSessionState::OFF){state_=MotionSessionState::SAFE_OFF_REQUIRED;executor_->stop(r);}}
void MotionSession::confirmSafeOff(){
 if(state_!=MotionSessionState::SAFE_OFF_REQUIRED)return;
 arbiter_->release(lease_);lease_={};state_=MotionSessionState::OFF;reason_="SAFE_OFF_VERIFIED";
}
void MotionSession::update(const LocalTrialEvidence& e,const MotionExecutionFacts& input,const JointFeedback& fb,uint32_t now){
 if(state_==MotionSessionState::OFF||state_==MotionSessionState::SAFE_OFF_REQUIRED)return;
 if(const char* r=gate(e,input,fb,now)){revoke(r);return;}
 auto f=input;f.lease=lease_;
 if(state_==MotionSessionState::ARMED){
  if(!executor_->active()){revoke("EXECUTOR_LOST");return;}
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
 for(unsigned i=0;i<12;++i)if(std::abs(fb.actual_rad[i]-primed_rad_[i])>envelope_.tracking_tolerance_rad){revoke("PRIME_DRIFT");return;}
 if(state_==MotionSessionState::ARMING){
  // Old pre-prime snapshots cannot prove the holding drive profile. Wait
  // within the admission deadline for all twelve post-energize readbacks.
  if(!input.prime_profiles_verified)return;
  bool fresh=true;for(unsigned i=0;i<12;++i)fresh=fresh&&fb.torque_on[i]&&static_cast<int32_t>(fb.sampled_at_ms[i]-started_at_ms_)>0;
  if(!fresh)return;
  if(!executor_->arm(f,envelope_,fb,now)){revoke(executor_->status().reason);return;}
  state_=MotionSessionState::ARMED;reason_="ARMED_LOCAL_SCOPED_PERMIT";return;
 }
 actuator::ActuatorCommand cmd;uint8_t bus;if(!resolve(next_,primed_rad_[next_],cmd,bus)){revoke("PRIME_MAPPING_OR_LIMITS");return;}
 if(state_==MotionSessionState::PRIMING && fb.torque_on[next_]){revoke("TORQUE_CHANGED_DURING_PRIME");return;}
 if(state_==MotionSessionState::ENERGIZING)cmd.operation=actuator::ActuatorOperation::TORQUE_ENABLE;
 actuator::ActuatorTransaction tx;
 if(policy_->plan(cmd,lease_,f.mode,&tx)!=actuator::WriteDecision::ACCEPT||runtime_->execute(&tx,bus,nullptr)!=actuator::ExecuteResult::WRITTEN){revoke("PRIME_BACKEND_OR_POLICY");return;}
 if(++next_==12){next_=0;started_at_ms_=now;state_=state_==MotionSessionState::PRIMING?MotionSessionState::ENERGIZING:MotionSessionState::ARMING;}
}
} }
