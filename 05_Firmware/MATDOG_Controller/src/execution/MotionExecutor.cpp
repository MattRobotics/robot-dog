#include "MotionExecutor.h"
#include "../servo/ServoPopulation.h"
#include <cmath>
#include <cstring>
namespace matdog { namespace execution {
const char* toString(ExecutorState s) { switch(s){case ExecutorState::OFF:return "OFF";case ExecutorState::READY:return "READY";case ExecutorState::WRITING:return "WRITING";case ExecutorState::TRACKING:return "TRACKING";case ExecutorState::SAFE_OFF_REQUIRED:return "SAFE_OFF_REQUIRED";case ExecutorState::STREAMING:return "STREAMING";case ExecutorState::HOLD:return "HOLD";default:return "FAULT";} }
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
const char* MotionExecutor::feedbackFailure(const JointFeedback& f,uint32_t now) const {
 for(unsigned i=0;i<12;i++){
  if(static_cast<int32_t>(now-f.sampled_at_ms[i])<0)return "FEEDBACK_CLOCK_INVALID";
  if(now-f.sampled_at_ms[i]>envelope_.feedback_max_age_ms)return "STALE_FEEDBACK";
  if(!f.read_ok[i]||!std::isfinite(f.actual_rad[i]))return "FEEDBACK_INVALID";
  if(!f.torque_on[i])return "TORQUE_LOST";
 }
 return nullptr;
}
bool MotionExecutor::reject(const char* reason) { status_.reason=reason; if(active())stop(reason);return false; }
bool MotionExecutor::arm(const MotionExecutionFacts& f,const ExecutionEnvelope& e,const JointFeedback& fb,uint32_t now) {
 if(status_.state!=ExecutorState::OFF)return reject("EXECUTOR_BUSY");
 if(const char* reason=gate(f,now))return reject(reason);
 if(!e.approved||!std::isfinite(e.max_velocity_rad_s)||!std::isfinite(e.max_acceleration_rad_s2)||!std::isfinite(e.tracking_tolerance_rad)||e.max_velocity_rad_s<=0||e.max_acceleration_rad_s2<=0||e.tracking_tolerance_rad<=0||!e.feedback_max_age_ms||!e.batch_timeout_ms||!e.tracking_timeout_ms||e.feedback_max_age_ms>3000||e.batch_timeout_ms>3000||e.tracking_timeout_ms>30000)return reject("ENVELOPE_UNAPPROVED");
 if(e.timed_segments&&(!e.frame_writes||!std::isfinite(e.arrival_tolerance_rad)||e.arrival_tolerance_rad<=0||e.arrival_tolerance_rad>e.tracking_tolerance_rad))return reject("ENVELOPE_UNAPPROVED");
 envelope_=armed_envelope_=e;if(const char* why=feedbackFailure(fb,now))return reject(why);
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
 if(const char* why=feedbackFailure(fb,now))return reject(why);
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
 segment_duration_ms_=static_cast<uint32_t>(std::ceil(dt*1000));
 validation_=v;deadline_ms_=issued+ttl;status_.accepted_sequence=frame.sequence;status_.submitted_at_ms=now;status_.next_joint=0;status_.state=ExecutorState::WRITING;status_.reason="EXECUTING";return true;
}
void MotionExecutor::update(const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now) {
 if(!active())return;
 if(const char* r=gate(f,now)){stop(r);return;}
 if(streaming())return;   // streaming: monitor() owns feedback/tracking (asynchronous, per joint)
 if(const char* why=feedbackFailure(fb,now)){stop(why);return;}
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
  if(envelope_.frame_writes){
   // The whole frame in ONE coordinated transaction (validated in submit()).
   const char* why="";
   if(!frameTx(target_,f,now*1000ull,&why)){stop(why);return;}
   status_.next_joint=12;status_.state=ExecutorState::TRACKING;tracking_since_ms_=now;return;
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
 // Continuous measured following guard, including while waiting to settle.
 // The independent [previous,target] box is included in the swept proof.
 if(envelope_.timed_segments)for(unsigned i=0;i<12;++i){
  if(fb.actual_rad[i]<std::fmin(previous_[i],target_[i])-envelope_.tracking_tolerance_rad||
     fb.actual_rad[i]>std::fmax(previous_[i],target_[i])+envelope_.tracking_tolerance_rad){stop("TRACKING_DIVERGENCE");return;}
 }
 // Feedback must postdate all writes, including the last joint's write.
 bool fresh=true;for(unsigned i=0;i<12;i++){
  // Finite TX uses the 32-bit millis epoch; compare in that same wrapping
  // domain, not against the feedback producer's extended microsecond clock.
  const uint32_t written=envelope_.frame_writes?static_cast<uint32_t>(status_.last_tx_complete_us/1000):tracking_since_ms_;
  fresh=fresh&&static_cast<int32_t>(fb.sampled_at_ms[i]-written)>0;
 }
 const double arrival=envelope_.timed_segments&&validation_.final?envelope_.arrival_tolerance_rad:envelope_.tracking_tolerance_rad;
 const bool released=!envelope_.timed_segments||now-tracking_since_ms_>=segment_duration_ms_;
 if(error<=arrival&&fresh&&released){for(unsigned i=0;i<12;i++){previous_[i]=target_[i];velocity_[i]=pending_velocity_[i];}status_.completed_sequence=status_.accepted_sequence;status_.state=ExecutorState::READY;status_.reason=envelope_.timed_segments&&!validation_.final?"TIMED_TRACKED":"TRACKED";}
 else if(now-tracking_since_ms_>=envelope_.tracking_timeout_ms)stop(envelope_.timed_segments&&validation_.final?"ARRIVAL_TIMEOUT":"TRACKING_TIMEOUT");
}
// ---------------------------------------------------------------------------
// Streaming (Issue #52, 2026-10-15)
// ---------------------------------------------------------------------------
bool MotionExecutor::beginStream(const StreamConfig& c,const JointFeedback& fb,uint64_t now_us) {
 if(status_.state!=ExecutorState::READY)return false;
 if(!std::isfinite(c.tube_rad)||c.tube_rad<=0||c.tube_rad>envelope_.tracking_tolerance_rad+1e-12||!std::isfinite(c.max_lag_s)||c.max_lag_s<=0||c.max_lag_s>.5||
    !std::isfinite(c.settle_speed_rad_s)||c.settle_speed_rad_s<0||!c.feedback_max_age_us||c.feedback_max_age_us>500000||!c.hold_timeout_us||
    !c.max_frame_gap_us||c.max_frame_gap_us>500000||!c.persist_samples||c.persist_samples>20||!c.region_min_rad||!c.region_max_rad)return false;
 for(unsigned i=0;i<12;i++)if(!fb.read_ok[i]||!fb.torque_on[i])return false;
 stream_=c;history_count_=0;history_head_=0;
 Sent& h=history_[0];h.at_us=now_us;h.sequence=status_.accepted_sequence;for(unsigned i=0;i<12;i++)h.q[i]=target_[i];
 history_count_=1;last_release_us_=now_us;hold_sequence_=0;hold_since_us_=0;
 for(unsigned i=0;i<12;i++){checked_us_[i]=fb.sampled_at_us[i];exceed_[i]=0;status_.stream_error_rad[i]=0;}
 status_.validated_sequence=status_.tx_sequence=status_.observed_sequence=status_.accepted_sequence;status_.settled_sequence=0;
 status_.stream_peak_error_rad=0;status_.worst_joint=0xff;
 status_.state=ExecutorState::STREAMING;status_.reason="STREAMING";return true;
}
bool MotionExecutor::frameTx(const double target[12],const MotionExecutionFacts& f,uint64_t now_us,const char** why) {
 actuator::ActuatorFrameCommand frame;uint8_t bus[12];
 for(unsigned i=0;i<12;i++)if(!resolve(i,target[i],&frame.joints[i],&bus[i])){*why="TARGET_OR_MAPPING";return false;}
 actuator::ActuatorFrameTransaction tx;
 if(policy_->planFrame(frame,f.lease,f.mode,&tx)!=actuator::WriteDecision::ACCEPT){*why="POLICY_REJECTED";return false;}
 actuator::ActuatorBackend::FrameTxReport report;
 const auto r=runtime_->executeFrame(&tx,bus,nullptr,&report);
 if(r==actuator::FrameExecuteResult::TX_COMPLETE_UNCONFIRMED){
  const uint64_t at=now_us+(report.tx_complete_us-report.tx_started_us);
  status_.last_tx_complete_us=at;return true;}
 // Zero bytes (refused) and uncertain transmissions both end the stream:
 // the reference cannot continue past a frame that was not delivered.
 *why=r==actuator::FrameExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF?"FRAME_TX_UNCERTAIN":"FRAME_REFUSED_ZERO_BYTES";return false;
}
bool MotionExecutor::stream(const motion::JointTargetFrame& frame,const FrameValidation& v,uint64_t release_us,uint64_t deadline_us,bool final,
                            const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now,uint64_t now_us) {
 if(status_.state!=ExecutorState::STREAMING){++status_.frames_refused;return reject("STREAM_NOT_ACTIVE");}
 if(const char* reason=gate(f,now))return reject(reason);
 if(!frame.valid||!frame.sequence||static_cast<int32_t>(frame.sequence-status_.validated_sequence)<=0)return reject("FRAME_SEQUENCE");
 if(release_us<=last_release_us_||release_us>now_us||deadline_us<=now_us)return reject("FRAME_DEADLINE");
 if(release_us-last_release_us_>stream_.max_frame_gap_us)return reject("STREAM_GAP");
 if(v.sequence!=frame.sequence||v.policy_epoch!=policy_->epoch()||v.geometry!=policy_->currentGeometryTag()||!v.collision_clear||!v.support_valid||!v.path_continuous||!v.configuration_valid)return reject("SEGMENT_UNVALIDATED");
 double candidate[12];
 for(unsigned leg=0;leg<4;leg++){candidate[leg*3]=frame.legs[leg].hip;candidate[leg*3+1]=frame.legs[leg].upper;candidate[leg*3+2]=frame.legs[leg].lower;}
 const double dt=(release_us-last_release_us_)*1e-6;
 const Sent& last=history_[(history_head_+history_count_-1)%kHistory];
 const double dt_prev=history_count_>1?(last.at_us-history_[(history_head_+history_count_-2)%kHistory].at_us)*1e-6:0;
 // The WHOLE frame is checked before the first byte.
 for(unsigned i=0;i<12;i++){
  actuator::ActuatorCommand cmd;uint8_t bus;
  if(!resolve(i,candidate[i],&cmd,&bus))return reject("TARGET_OR_MAPPING");
  if(!(candidate[i]>=stream_.region_min_rad[i]&&candidate[i]<=stream_.region_max_rad[i]))return reject("TARGET_OUTSIDE_CERTIFIED_REGION");
  const double vel=(candidate[i]-target_[i])/dt;
  if(std::fabs(vel)>envelope_.max_velocity_rad_s+1e-9)return reject("RATE_LIMIT");
  if(dt_prev>0){const double acc=(vel-velocity_[i])/(.5*(dt+dt_prev));if(std::fabs(acc)>envelope_.max_acceleration_rad_s2+1e-9)return reject("ACCELERATION_LIMIT");}
  pending_velocity_[i]=vel;
 }
 status_.validated_sequence=frame.sequence;
 const char* why="";
 if(!frameTx(candidate,f,now_us,&why)){++status_.frames_refused;return reject(why);}
 for(unsigned i=0;i<12;i++){previous_[i]=target_[i];target_[i]=candidate[i];velocity_[i]=pending_velocity_[i];}
 Sent& h=history_[(history_head_+history_count_)%kHistory];
 if(history_count_==kHistory)history_head_=(history_head_+1)%kHistory;else ++history_count_;
 h.at_us=status_.last_tx_complete_us;h.sequence=frame.sequence;for(unsigned i=0;i<12;i++)h.q[i]=candidate[i];
 status_.tx_sequence=status_.accepted_sequence=frame.sequence;++status_.frames_tx;status_.submitted_at_ms=now;last_release_us_=release_us;
 if(final){status_.state=ExecutorState::HOLD;hold_sequence_=frame.sequence;hold_since_us_=now_us;status_.reason="HOLD";}
 monitor(f,fb,now,now_us);
 return status_.state==ExecutorState::STREAMING||status_.state==ExecutorState::HOLD;
}
double MotionExecutor::referenceRange(unsigned j,uint64_t from,uint64_t to,double* lo,double* hi) const {
 // Goal values the servo was driven towards during [from,to]: every goal whose
 // active interval [sent, next sent) intersects the window.
 bool any=false;double a=0,b=0;
 for(unsigned k=0;k<history_count_;k++){const Sent& h=history_[(history_head_+k)%kHistory];
  if(h.at_us>to)break;
  if(k+1<history_count_&&history_[(history_head_+k+1)%kHistory].at_us<=from)continue;
  const double q=h.q[j];if(!any){a=b=q;any=true;}else{a=std::fmin(a,q);b=std::fmax(b,q);}}
 if(!any){const Sent& h=history_[history_head_];a=b=h.q[j];}
 *lo=a;*hi=b;return b-a;
}
void MotionExecutor::monitor(const MotionExecutionFacts& f,const JointFeedback& fb,uint32_t now,uint64_t now_us) {
 if(!streaming())return;
 if(const char* r=gate(f,now)){stop(r);return;}
 if(status_.state==ExecutorState::STREAMING&&now_us>last_release_us_&&now_us-last_release_us_>stream_.max_frame_gap_us){stop("STREAM_STARVED");return;}
 uint64_t oldest=UINT64_MAX;bool settled=true;
 const double settle_tolerance=envelope_.arrival_tolerance_rad>0?std::fmin(envelope_.arrival_tolerance_rad,stream_.tube_rad):stream_.tube_rad;
 for(unsigned i=0;i<12;i++){
  if(!fb.sampled_at_us[i]||fb.sampled_at_us[i]>now_us){stop("FEEDBACK_CLOCK_INVALID");return;}
  if(now_us-fb.sampled_at_us[i]>stream_.feedback_max_age_us){stop("STREAM_FEEDBACK_STALE");return;}
  if(!fb.read_ok[i]||!std::isfinite(fb.actual_rad[i])){stop("FEEDBACK_INVALID");return;}
  if(!fb.torque_on[i]){stop("TORQUE_LOST");return;}
  actuator::ActuatorCommand actual;uint8_t bus;if(!resolve(i,fb.actual_rad[i],&actual,&bus)){stop("ACTUAL_OUTSIDE_LIMITS");return;}
  // Geometry: the measured joint must stay in the certified box (+/- tube).
  if(fb.actual_rad[i]<stream_.region_min_rad[i]-stream_.tube_rad||fb.actual_rad[i]>stream_.region_max_rad[i]+stream_.tube_rad){stop("OUTSIDE_CERTIFIED_REGION");return;}
  if(fb.sampled_at_us[i]>checked_us_[i]){
   const uint64_t t=fb.sampled_at_us[i],lag=static_cast<uint64_t>(stream_.max_lag_s*1e6);
   double lo,hi;referenceRange(i,t>lag?t-lag:0,t,&lo,&hi);
   const double q=fb.actual_rad[i],e=q<lo?lo-q:q>hi?q-hi:0;
   status_.stream_error_rad[i]=e;if(e>status_.stream_peak_error_rad){status_.stream_peak_error_rad=e;status_.worst_joint=static_cast<uint8_t>(i);}
   if(e>stream_.tube_rad){if(++exceed_[i]>=stream_.persist_samples){stop("TRACKING_LAG_PERSISTENT");return;}}else exceed_[i]=0;
   checked_us_[i]=t;
  }
  if(fb.sampled_at_us[i]<oldest)oldest=fb.sampled_at_us[i];
  settled=settled&&std::fabs(fb.actual_rad[i]-target_[i])<=settle_tolerance&&std::fabs(fb.speed_rad_s[i])<=stream_.settle_speed_rad_s;
 }
 for(unsigned k=history_count_;k-->0;){const Sent& h=history_[(history_head_+k)%kHistory];if(h.at_us<oldest){if(h.sequence>status_.observed_sequence)status_.observed_sequence=h.sequence;break;}}
 double error=0;for(unsigned i=0;i<12;i++)error=std::fmax(error,std::fabs(fb.actual_rad[i]-target_[i]));status_.max_tracking_error_rad=error;
 if(status_.state==ExecutorState::HOLD){
  if(status_.observed_sequence==hold_sequence_&&settled)status_.settled_sequence=hold_sequence_;
  else if(now_us-hold_since_us_>stream_.hold_timeout_us)stop("HOLD_SETTLE_TIMEOUT");
 }
}
bool MotionExecutor::endStream() {
 if(status_.state!=ExecutorState::HOLD||status_.settled_sequence!=hold_sequence_||!hold_sequence_)return false;
 for(unsigned i=0;i<12;i++){previous_[i]=target_[i];velocity_[i]=0;}
 status_.completed_sequence=status_.accepted_sequence=hold_sequence_;status_.state=ExecutorState::READY;status_.reason="TRACKED";return true;
}
bool MotionExecutor::selectRates(double v,double a) {
 if(status_.state!=ExecutorState::READY||!std::isfinite(v)||!std::isfinite(a)||v<=0||a<=0||
    v>kWalkRateCeilingRadS||a>kWalkAccelerationCeilingRadS2)return false;
 envelope_.max_velocity_rad_s=v;envelope_.max_acceleration_rad_s2=a;return true;
}
void MotionExecutor::stop(const char* r) { if(active()||status_.safe_off_required){status_.state=ExecutorState::SAFE_OFF_REQUIRED;status_.safe_off_required=true;}status_.reason=r; }
void MotionExecutor::confirmSafeOff(bool off) { if(status_.safe_off_required&&off){status_.safe_off_required=false;status_.state=ExecutorState::OFF;} }
} }
