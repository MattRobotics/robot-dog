#include "Controller.h"
#include "LocalMotionCommand.h"
#include "../motion/FirstStrideReference.h"
#include "../servo/ServoPopulation.h"
#include "../actuator/CalibrationGeometryProfileData.h"
#include <esp_random.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cerrno>
namespace matdog { namespace core {
namespace {
bool uintToken(const char* s,uint32_t& v){char* e=nullptr;errno=0;const unsigned long x=strtoul(s,&e,10);if(!*s||*e||*s=='-'||errno==ERANGE||x>UINT32_MAX)return false;v=x;return true;}
bool realToken(const char* s,double& v){char* e=nullptr;v=strtod(s,&e);return e!=s&&!*e&&std::isfinite(v);}
bool fixture(execution::TrialScope s){return s==execution::TrialScope::SUPERVISED_FIXTURE_STAND||s==execution::TrialScope::SUPERVISED_FIXTURE_WALK;}
}
bool Controller::localValidationCommand(const char* line,uint32_t now,char* out,size_t size){
 const char* reason="USAGE_OR_LIVE_GATE";bool ok=false,timed=false;uint32_t issued=now;
 if(line&&!strncmp(line,"AT ",3)){const char* end=strchr(line+3,' ');if(!end||end-line>16){snprintf(out,size,"VALIDATION=REFUSED REASON=LOCAL_COMMAND_CLOCK_INVALID");return false;}char stamp[16]{};memcpy(stamp,line+3,end-line-3);if(!uintToken(stamp,issued)||static_cast<int32_t>(now-issued)<0||now-issued>250){snprintf(out,size,"VALIDATION=REFUSED REASON=LOCAL_COMMAND_CLOCK_STALE");return false;}line=end+1;timed=true;}

 auto reply=[&](){snprintf(out,size,"VALIDATION=%s REASON=%s SESSION=%s SEGMENT=%u NONCE=%lu LIMITS=%u UPTIME_MS=%lu WALK_VX=%.17g PLANNER=%s",ok?"PASS":"REFUSED",reason,operational_session_.stateName(),motion_coordinator_.proneSegment(),(unsigned long)validation_nonce_,runtime_restore_.snapshot().approved_joints,(unsigned long)now,reviewed_motion_bound_?reviewed_motion_.walk.maxVxMps:0,motion_coordinator_.state());return ok;};
 if(!line||strlen(line)>240)return reply();
 char copy[241];strcpy(copy,line);const char* t[18]{};unsigned n=0;char* save=nullptr;
 for(char* p=strtok_r(copy," ",&save);p;p=strtok_r(nullptr," ",&save)){if(n==18)return reply();t[n++]=p;}
 if(!n)return reply();
 if(!strcmp(t[0],"STOP")){
  operational_session_.revoke("LOCAL_OPERATOR_STOP");motion_executor_.stop("LOCAL_OPERATOR_STOP");
  operational_facts_.operator_permit=false;validation_present_=false;validation_walk_admitted_=false;validation_presence_ms_=0;
  validation_nonce_=esp_random();if(!validation_nonce_)validation_nonce_=1;
  ok=true;reason="SAFE_OFF_REQUESTED";return reply();
 }
 if(!strcmp(t[0],"STATUS")&&n==1){ok=true;reason=boot_safe_off_pending_?"BOOT_SAFE_OFF_PENDING":validation_profile_.reason();return reply();}
 if(!strcmp(t[0],"JOINT")&&n==2){uint32_t j;if(!uintToken(t[1],j)||j>=12)return reply();const auto& r=operational_producer_.raw(j);const auto* servo=servo::legServoAt((j/3)*3+2-j%3);const auto fb=operational_producer_.snapshot(now,reviewed_motion_bound_?reviewed_motion_.execution.feedback_max_age_ms:500);
  snprintf(out,size,"JOINT slot=%lu id=%u configured_unit=%s raw=%u torque=%d speed_raw=%d goal=%d goal_speed=%d goal_acc=%d output=%d temp=%d voltage_dv=%d status=%d stamp_ms=%lu age_ms=%lu transaction_ms=%lu mapped_valid=%u actual_rad=%.9f",(unsigned long)j,servo->bus_id,servo->physical_unit,r.position,r.torque,r.speed,r.goal_position,r.goal_speed,r.goal_acceleration,r.torque_limit,r.temperature,r.voltage,r.status,(unsigned long)r.sampled_at_ms,(unsigned long)(now-r.sampled_at_ms),(unsigned long)r.transaction_ms,fb.read_ok[j]?1u:0u,fb.actual_rad[j]);return true;}
 if(!strcmp(t[0],"SENSORS")&&n==1){const auto i=imu_.orientationSnapshot();const auto& b=daly_.sample();snprintf(out,size,"SENSORS uptime_ms=%lu imu_valid=%u imu_stamp_ms=%lu quaternion=%.9f,%.9f,%.9f,%.9f bms_valid=%u bms_stamp_ms=%lu soc=%.3f power=%s fault=%u boot_safe_off_pending=%u",(unsigned long)now,i.valid?1u:0u,(unsigned long)i.sampled_at_ms,i.wxyz[0],i.wxyz[1],i.wxyz[2],i.wxyz[3],b.valid?1u:0u,(unsigned long)b.sampled_at_ms,b.soc_percent,core::toString(power_state_.state()),system_state_.systemHealth()==SystemHealth::FAULT?1u:0u,boot_safe_off_pending_?1u:0u);return true;}
 if(wifi_.configBusy()){reason="NETWORK_CONFIGURATION_TRANSACTION_ACTIVE";return reply();}
 if(!build::kSupervisedValidationCapable){reason="VALIDATION_CAPABILITY_NOT_COMPILED";return reply();}
 if(runtime_mode_!=execution::RuntimeTestMode::GROUND_VALIDATION&&runtime_mode_!=execution::RuntimeTestMode::AUTONOMOUS){
  reason="REVIEWED_RUNTIME_MODE_REQUIRED";return reply();
 }
 if(!strcmp(t[0],"PROFILE")){
  if(authority_.current()!=ActuatorAuthority::NONE||motion_service_.busy()||boot_safe_off_pending_){reason="SAFE_IDLE_REQUIRED";return reply();}
  unbindCommissioning("REVIEWED_PROFILE_LOADING");
  reviewed_motion_bound_=false;validation_present_=false;local_motion_observation_={};local_trial_evidence_={};operational_facts_.operator_permit=false;servo_bus_.setValidationTimeout(0);servo_bus_.setValidationDrive(0,0);validation_walk_admitted_=false;
  ok=validation_profile_.card(line+8);reason=validation_profile_.reason();return reply();
 }
 if(!timed){reason="LOCAL_ISSUED_CLOCK_REQUIRED";return reply();}
 // Reviewed-card actuation and the commissioning policy never share a runtime.
 if(commissioning_bound_){reason="COMMISSIONING_RUNTIME_BOUND_USE_@MOTION_COMMISSIONING";return reply();}
 if(!strcmp(t[0],"ADMIT")&&n==3&&!strcmp(t[2],"CONFIRM_CURRENT_INSTALLATION_AND_REVIEWED_PROFILE")){
  if(!execution::runtimeModeAllowsScope(runtime_mode_,validation_profile_.runtime.scope)){reason="RUNTIME_MODE_SCOPE_MISMATCH";return reply();}
  uint32_t nonce;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!validation_profile_.sealed()||boot_safe_off_pending_||authority_.current()!=ActuatorAuthority::NONE||motion_service_.busy())return reply();
  if(servo_preflight_.state()!=servo::ServoPreflight::State::COMPLETE||!servo_preflight_.result().allPass()){reason="LIVE_SERVO_PROFILE_CHECK_REQUIRED";return reply();}
  auto& p=validation_profile_;calibration::RuntimeRestoreRequest facts;facts.expected_generation=1;facts.current_installation_confirmed=true;facts.executor_idle=true;facts.motion_permit_revoked=true;
  facts.torque_off_confirmed=operational_producer_.rawFresh(now,p.runtime.execution.feedback_max_age_ms);
  for(unsigned i=0;i<12;++i)facts.torque_off_confirmed=facts.torque_off_confirmed&&operational_producer_.raw(i).torque==0&&operational_producer_.raw(i).speed==0;
  const auto stored=persistence_.readAcknowledgedRecord(geometry_profile_,&validation_record_);size_t encoded=0;
  if(stored.load_status!=calibration::LoadStatus::OK||stored.generation!=1||validation_record_.generation!=runtime_restore_.snapshot().restored_generation||calibration::encodeCalibrationRecord(validation_record_,validation_record_bytes_,sizeof(validation_record_bytes_),&encoded)!=calibration::CalibrationRecordStatus::OK){reason="CURRENT_ACKNOWLEDGED_RECORD_REQUIRED";return reply();}
  update::Sha256 hash;hash.update(validation_record_bytes_,encoded);uint8_t digest[32];char actual_sha[65];hash.finish(digest);update::toHex(digest,actual_sha,sizeof(actual_sha));
  if(strcmp(actual_sha,p.record_sha256)){reason="REVIEWED_RECORD_DIGEST_MISMATCH";return reply();}
  for(unsigned i=0;i<12;++i){const auto& j=validation_record_.joint[i];calibration::JointIdentity id;id.leg=static_cast<calibration::Leg>(j.leg);id.joint=static_cast<calibration::JointKind>(j.joint);calibration::setPhysicalUnit(&id,j.unit);const auto* tr=actuator_policy_.transforms().find(id,geometry_profile_.provenanceTag());if(!tr||tr->q0_tick!=j.q0_tick){reason="RESTORE_TRANSFORM_RECORD_MISMATCH";return reply();}}
  p.workspace.provenance=actuator::geometry_data::kProvenance;
  p.workspace.scope=calibration::OperationalLimitScope::CERTIFIED_ROUTE;strcpy(p.workspace.route_sha256,motion::pairedProneDigest(p.runtime.feedback.fixtureLimitVariant));strcpy(p.workspace.corridor_proof_sha256,p.corridor_proof_sha256);
  const auto admission=runtime_restore_.qualifyOperationalLimits(geometry_profile_,actuator::geometry_data::kProvenance,&p.workspace,p.workspace.artifact_sha256,facts,actuator_policy_.transforms(),actuator_policy_.limits(),motion::pairedProneDigest(p.runtime.feedback.fixtureLimitVariant));
  if(admission!=calibration::OperationalAdmissionStatus::APPROVED){reason=calibration::toString(admission);return reply();}
  auto r=p.runtime;r.policy_epoch=actuator_policy_.epoch();strcpy(r.route_sha256,motion::pairedProneDigest(p.runtime.feedback.fixtureLimitVariant));
  r.fixture_bus_timing_qualified=fixture(r.scope);r.autonomous_bus_timing_qualified=!fixture(r.scope);r.feedback.routeTubeReviewed=true;r.feedback.routeFirstKnot=p.first_knot;r.feedback.routeLastKnot=p.last_knot;
  r.feedback.routeJointTubeRad=p.joint_tube_rad-r.execution.tracking_tolerance_rad-p.encoder_uncertainty_rad-p.mapping_uncertainty_rad-r.feedback.maxJointStepRad/2-p.drive_speed*p.speed_scale*r.execution.feedback_max_age_ms/1000;r.feedback.routePositionTubeM=p.position_tube_m;r.feedback.routeRotationTube=p.rotation_tube;
  // Integer admission insets one tick. Use its actual semantic bounds rather
  // than silently extending rounded endpoints beyond the approved raw limits.
  for(unsigned i=0;i<12;++i){auto id=p.workspace.joint[i].joint;const auto* tr=actuator_policy_.transforms().find(id,geometry_profile_.provenanceTag());const auto* lim=actuator_policy_.limits().find(id,geometry_profile_.provenanceTag());actuator::MicroRad lo,hi;
   if(!tr||!lim||actuator::resolveRawToUrdfQ(geometry_profile_,actuator::geometry_data::kProvenance,*tr,lim->min_tick,&lo)!=actuator::TargetResolveStatus::OK||actuator::resolveRawToUrdfQ(geometry_profile_,actuator::geometry_data::kProvenance,*tr,lim->max_tick,&hi)!=actuator::TargetResolveStatus::OK){reason="LIMIT_TRANSFORM_FAILED";return reply();}
   r.feedback.jointLimits[i/3][i%3]={std::fmin(lo,hi)*1e-6,std::fmax(lo,hi)*1e-6};
  }
  // Qualification of the numeric target envelope is separate from the pose
  // corridor. All subsequent targets must remain inside the admitted limits.
  for(unsigned k=p.first_knot;k<=p.last_knot;++k){const auto f=motion::pairedProneSample(k==251?250:k,k==251?1:0,p.runtime.feedback.fixtureLimitVariant);
   for(unsigned i=0;i<4;++i){const double q[]={f.target.legs[i].hip,f.target.legs[i].upper,f.target.legs[i].lower};for(unsigned j=0;j<3;++j){const auto& b=r.feedback.jointLimits[i][j];if(q[j]-p.joint_tube_rad<b.lower||q[j]+p.joint_tube_rad>b.upper){reason="CORRIDOR_TUBE_OUTSIDE_APPROVED_LIMITS";return reply();}}}
  }
  if(p.last_knot!=251){reason="COMPLETE_STAND_CORRIDOR_REQUIRED";return reply();}
  if(r.scope==execution::TrialScope::SUPERVISED_FIXTURE_WALK||r.scope==execution::TrialScope::AUTONOMOUS_WALK){
   unsigned joint=0;if(!motion::compiledFirstStrideFitsLimits(r.feedback,p.walk_tube_rad,&joint)){
    snprintf(out,size,"VALIDATION=REFUSED REASON=WALK_TUBE_OUTSIDE_APPROVED_LIMITS JOINT=%u",joint);return false;
   }
  }
  ok=bindReviewedMotionRuntime(r);if(ok){servo_bus_.setValidationTimeout(p.transaction_timeout_ms);servo_bus_.setValidationDrive(p.drive_speed,p.drive_acceleration);}
  reason=ok?"RAM_PROFILE_ADMITTED_NO_PERMIT":"RUNTIME_PROFILE_REJECTED";return reply();
 }
 if(!strcmp(t[0],"CEILING")&&n==4&&!strcmp(t[3],"CONFIRM_REDUCE_SRAM_OUTPUT_WHILE_TORQUE_OFF")){
  uint32_t nonce,j;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!uintToken(t[2],j)||j>=12||!validation_present_||now-validation_presence_ms_>1000||!reviewed_motion_bound_||authority_.current()!=ActuatorAuthority::NONE||!operational_producer_.rawFresh(now,reviewed_motion_.execution.feedback_max_age_ms))return reply();
  for(unsigned i=0;i<12;++i)if(operational_producer_.raw(i).torque!=0)return reply();
  const auto* servo=servo::legServoAt((j/3)*3+2-j%3);
  ok=actuator_backend_.reduceValidationOutputWhileOff(servo->bus_id,validation_profile_.output_ceiling)==actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  reason=ok?"OUTPUT_REDUCED_REQUIRE_FRESH_READBACK":"REDUCTION_REJECTED_NO_INCREASE_ALLOWED";return reply();
 }
 if(!strcmp(t[0],"PRESENT")&&n==3){
  const bool suspended=fixture(reviewed_motion_.scope);
  if(strcmp(t[2],suspended?"CONFIRM_PRESENT_FIXTURE_POWER_CUT_FIXED_POSE_UNCHANGED":"CONFIRM_PRESENT_POWER_CUT_AND_CURRENT_EXTERNAL_SUPPORT_OBSERVER"))return reply();
  uint32_t nonce;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!reviewed_motion_bound_||local_motion_observation_.heightSource!=motion::EvidenceSource::EXTERNAL_MEASURED)return reply();
  validation_present_=true;validation_presence_ms_=issued;if(suspended)local_motion_observation_.supportStampS=issued/1000.0;
  local_trial_evidence_.local_operator_present=true;ok=true;reason="LOCAL_DEADMAN_REFRESHED";return reply();
 }
 if(!strcmp(t[0],"POSE")&&n==9&&!strcmp(t[8],"CONFIRM_MEASURED_FIXED_FIXTURE_NO_FOOT_LOAD")){
  uint32_t nonce;double v[6];if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!reviewed_motion_bound_||!fixture(reviewed_motion_.scope))return reply();
  for(unsigned i=0;i<6;++i)if(!realToken(t[i+2],v[i]))return reply();
  motion::AdaptiveObservation o;o.body.translationM={v[0],v[1],v[2]};const double cr=cos(v[3]),sr=sin(v[3]),cp=cos(v[4]),sp=sin(v[4]),cy=cos(v[5]),sy=sin(v[5]);
  const double rot[3][3]={{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},{sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}};memcpy(o.body.rotation,rot,sizeof(rot));
  const auto& fixed=validation_profile_.fixture_pose;const auto a=o.body.translationM,b=fixed.translationM;
  if(std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z)>validation_profile_.position_tube_m){reason="FIXTURE_POSE_OUTSIDE_REVIEWED_FREE_VOLUME";return reply();}
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)if(std::abs(o.body.rotation[i][j]-fixed.rotation[i][j])>validation_profile_.rotation_tube){reason="FIXTURE_ORIENTATION_OUTSIDE_REVIEWED_FREE_VOLUME";return reply();}
  o.stampS=now/1000.0;o.heightSource=o.supportSource=motion::EvidenceSource::EXTERNAL_MEASURED;o.externalSupportConfirmed=o.flatPlaneConfirmed=o.stationary=true;
  ok=publishLocalMotionObservation(o);reason=ok?"MEASURED_FIXTURE_POSE_ACCEPTED_NO_LOAD_CLAIM":"POSE_OUTSIDE_FEEDBACK_ENVELOPE";return reply();
 }
 if(!strcmp(t[0],"OBSERVE")&&n==15&&!strcmp(t[14],"CONFIRM_MEASURED_POSE_AND_BEARING_SUPPORT")){
  uint32_t nonce,stamp,feet,belly,stationary,external;double v[6];
  if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!reviewed_motion_bound_||fixture(reviewed_motion_.scope)||!uintToken(t[2],stamp)||static_cast<int32_t>(now-stamp)<0||now-stamp>1000||!uintToken(t[9],feet)||feet>15||!uintToken(t[10],belly)||belly>1||!uintToken(t[11],stationary)||stationary>1||!uintToken(t[12],external)||external>1||strcmp(t[13],validation_profile_.support_proof_sha256))return reply();
  for(unsigned i=0;i<6;++i)if(!realToken(t[i+3],v[i]))return reply();
  motion::AdaptiveObservation o;o.body.translationM={v[0],v[1],v[2]};const double cr=cos(v[3]),sr=sin(v[3]),cp=cos(v[4]),sp=sin(v[4]),cy=cos(v[5]),sy=sin(v[5]);
  const double rot[3][3]={{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},{sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}};memcpy(o.body.rotation,rot,sizeof(rot));
  o.stampS=stamp/1000.0;o.heightSource=o.supportSource=motion::EvidenceSource::EXTERNAL_MEASURED;o.confirmedFeet=feet;o.bellySupported=belly;o.stationary=stationary;o.externalSupportConfirmed=external;o.flatPlaneConfirmed=true;
  ok=publishLocalMotionObservation(o);reason=ok?"EXTERNAL_LOAD_OBSERVATION_ACCEPTED":"EXTERNAL_OBSERVATION_REJECTED";return reply();
 }
 if(!strcmp(t[0],"GRANT")&&n==4&&!strcmp(t[3],"CONFIRM_LOCAL_SUPERVISED_PRIME_HOLDING_OUTPUT_CEILING")){
  uint32_t nonce,ttl;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!uintToken(t[2],ttl)||ttl<1000||ttl>60000||!validation_present_||now-validation_presence_ms_>1000)return reply();
  for(unsigned i=0;i<12;++i)if(operational_producer_.raw(i).speed!=0){reason="STATIONARY_ENCODERS_REQUIRED_FOR_PRIME";return reply();}
  execution::LocalTrialEvidence e;e.scope=reviewed_motion_.scope;e.generation=++validation_sequence_;e.expires_at_ms=issued+ttl;
  e.local_operator_present=e.physical_power_cut_verified=e.installation_verified=e.servo_output_ceiling_verified=true;e.fixture_support_verified=fixture(e.scope);e.support_load_observable=!fixture(e.scope)&&motion::checkObservation(local_motion_observation_,reviewed_motion_.feedback,now/1000.0)==motion::FeedbackReason::OK;e.loaded_dynamics_qualified=e.scope==execution::TrialScope::AUTONOMOUS_WALK&&validation_profile_.dynamics_proof_sha256[0];e.imu_fresh=operational_facts_.hardware_qualified;
  ok=requestLocalMotionTrial(e,e.generation,e.expires_at_ms);reason=operational_session_.reason();
  if(ok){validation_nonce_=esp_random();if(!validation_nonce_)validation_nonce_=1;}return reply();
 }
 if(!strcmp(t[0],"RENEW")&&n==4&&!strcmp(t[3],"CONFIRM_CONTINUED_LOCAL_SUPERVISION_AND_GATES")){
  uint32_t nonce,ttl;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!uintToken(t[2],ttl)||ttl<1000||ttl>60000||!validation_present_||now-validation_presence_ms_>1000||operational_session_.state()!=execution::MotionSessionState::ARMED)return reply();
  operational_facts_.permit_expires_at_ms=local_trial_evidence_.expires_at_ms=issued+ttl;ok=true;reason="FINITE_PERMIT_EXPLICITLY_RENEWED";return reply();
 }
 if((!strcmp(t[0],"WALK_START")||!strcmp(t[0],"WALK"))&&n==6){
  uint32_t nonce;double v[3];if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||strcmp(t[5],fixture(reviewed_motion_.scope)?"CONFIRM_SUSPENDED_WALK_PROOF_AND_CLEARANCE":"CONFIRM_LOADED_WALK_CONTACT_COMPLIANCE_DYNAMICS_PROOF")||(reviewed_motion_.scope!=execution::TrialScope::SUPERVISED_FIXTURE_WALK&&reviewed_motion_.scope!=execution::TrialScope::AUTONOMOUS_WALK)||!validation_present_||now-validation_presence_ms_>1000||operational_session_.state()!=execution::MotionSessionState::ARMED)return reply();
  for(unsigned i=0;i<3;++i)if(!realToken(t[i+2],v[i]))return reply();
  if(v[0]!=reviewed_motion_.walk.maxVxMps||v[1]!=0||v[2]!=0){reason="OUTSIDE_CERTIFIED_FIRST_STRIDE_PARAMETERS";return reply();}
  const bool start=!strcmp(t[0],"WALK_START");
  if(!validation_walk_admitted_&&!start){reason="EXPLICIT_FIRST_STRIDE_START_REQUIRED";return reply();}
  if(validation_walk_admitted_&&start){reason="FIRST_STRIDE_ALREADY_ACTIVE";return reply();}
  if(!validation_walk_admitted_){
   if(strcmp(motion_coordinator_.state(),"STAND")||motion_executor_.status().state!=execution::ExecutorState::READY){reason="STAND_VERIFIED_REQUIRED";return reply();}
   validation_walk_admitted_=true;validation_walk_started_=false;
   auto ready=motion_service_.readiness();ready.walk_qualified=true;ready.max_vx=reviewed_motion_.walk.maxVxMps;ready.max_vy=reviewed_motion_.walk.maxVyMps;ready.max_yaw_rate=reviewed_motion_.walk.maxYawRateRadS;motion_service_.refresh(ready,now);
  }

  auto c=localMotionCommand(motion::Intent::BASE_VELOCITY,++validation_sequence_,issued,200);c.vx=v[0];c.vy=v[1];c.yaw_rate=v[2];
  const auto a=motion_service_.command(c,now);ok=a.outcome==motion::CommandOutcome::ACCEPTED;if(!ok)validation_walk_admitted_=false;reason=a.reason;return reply();
 }
 if(!strcmp(t[0],"NEXT")&&n==4&&!strcmp(t[3],"CONFIRM_TRACKING_DIRECTIONS_CLEARANCE_POWER_STOP")){
  uint32_t nonce,segment;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!uintToken(t[2],segment)||!validation_present_||now-validation_presence_ms_>1000||operational_session_.state()!=execution::MotionSessionState::ARMED||motion_executor_.status().state!=execution::ExecutorState::READY)return reply();
  ok=motion_coordinator_.approveProneSegment(segment);reason=ok?"ONE_CERTIFIED_SEGMENT_APPROVED":"SEGMENT_NOT_CURRENT_OR_PENDING";return reply();
 }
 if(!strcmp(t[0],"STAND")&&n==2){
  uint32_t nonce;if(!uintToken(t[1],nonce)||nonce!=validation_nonce_||!validation_present_||operational_session_.state()!=execution::MotionSessionState::ARMED)return reply();
  auto c=localMotionCommand(motion::Intent::STAND,++validation_sequence_,issued,1000);
  const auto a=motion_service_.command(c,now);ok=a.outcome==motion::CommandOutcome::ACCEPTED;reason=a.reason;return reply();
 }
 return reply();
}
} }
