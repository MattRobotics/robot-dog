// Runtime test policy adapter: @MOTION COMMISSIONING ... and @MOTION TELEMETRY.
// USB console only (never registered with HTTP). RAM-only state; nothing here
// writes NVS/EEPROM or changes a servo configuration register. Every actuation
// still goes through MotionSession -> MotionExecutor -> Safe Actuator layer.
#include "../motion/OperationalWalkProfile.h"
#include "Controller.h"
#include "LocalMotionCommand.h"
#include "../motion/FirstStrideReference.h"
#include "../motion/PairedProneRoute.h"
#include "../servo/ServoPopulation.h"
#include "../actuator/CalibrationGeometryProfileData.h"
#include <esp_random.h>
#include <esp_heap_caps.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cerrno>
namespace matdog { namespace core {
namespace {
bool uintArg(const char* s,uint32_t& v){char* e=nullptr;errno=0;const unsigned long x=strtoul(s,&e,10);if(!*s||*e||*s=='-'||errno==ERANGE||x>UINT32_MAX)return false;v=x;return true;}
bool intArg(const char* s,long& v){char* e=nullptr;errno=0;v=strtol(s,&e,10);return *s&&!*e&&errno!=ERANGE;}
bool hex64(const char* s){if(!s||strlen(s)!=64)return false;bool any=false;for(unsigned i=0;i<64;++i){const char c=s[i];if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;any|=c!='0';}return any;}
calibration::JointIdentity slotIdentity(unsigned i){
 calibration::JointIdentity id;const auto* servo=servo::legServoAt((i/3)*3+2-i%3);id.leg=static_cast<calibration::Leg>(i/3);id.joint=static_cast<calibration::JointKind>(i%3);
 if(servo)strncpy(id.physical_unit,servo->physical_unit,sizeof(id.physical_unit)-1);return id;
}
double& slot(motion::JointTargetFrame& f,unsigned i){auto& q=f.legs[i/3];return i%3==0?q.hip:i%3==1?q.upper:q.lower;}
const char* entryName(const execution::MotionCoordinator& c){
 const auto& s=c.stand();if(!s.entryActive())return "NONE";
 if(!s.entryPending())return "DONE";return s.entryStage()==0?"E0":"E1";
}
}
void Controller::unbindCommissioning(const char* reason){
 reviewed_motion_bound_=false;reviewed_motion_={};
 commissioning_bound_=false;motion_coordinator_.clearCommissioning();
 local_motion_observation_={};local_trial_evidence_={};operational_facts_.operator_permit=false;
 validation_present_=false;validation_walk_admitted_=false;operational_walk_bound_=false;operational_walk_reason_="WALK_NOT_BOUND";
 servo_bus_.setValidationTimeout(0);servo_bus_.setValidationDrive(0,0);
 // Commissioning limits never outlive their policy; RESTORE (transforms) stays.
 if(runtime_restore_.snapshot().commissioning_limits_active||runtime_restore_.snapshot().operational_limits_approved)runtime_restore_.revokeLimits(actuator_policy_.limits());
 commissioning_reason_=reason;
}
void Controller::updateLivePose(){
 live_pose_valid_=false;
 for(unsigned i=0;i<12;++i)if(!operational_feedback_.read_ok[i])return;
 double actual[12]{},nominal[12]{};const auto head=motion::pairedProneSample(0,0,true);if(!head.target.valid)return;
 bool stationary=true;
 for(unsigned i=0;i<12;++i){actual[i]=operational_feedback_.actual_rad[i];const auto& n=head.target.legs[i/3];nominal[i]=i%3==0?n.hip:i%3==1?n.upper:n.lower;
  stationary=stationary&&operational_producer_.raw(i).speed==0;}
 // Diagnostic view only: fixture support as declared by the bound policy.
 live_pose_=motion::classifyFixtureProne(actual,nominal,commissioning_bound_||runtime_mode_==execution::RuntimeTestMode::DIAGNOSTIC,stationary,
   commissioning_params_.prone_max_deviation_rad,commissioning_params_.prone_max_rms_rad);
 live_pose_valid_=true;
}
void Controller::emitTelemetry(uint32_t now){
 if(!telemetry_period_ms_||now-telemetry_last_ms_<telemetry_period_ms_)return;
 telemetry_last_ms_=now;
 static char buffer[6144];
 memcpy(buffer,"@TEL ",5);
 if(!motion_service_.statusJson(buffer+5,sizeof(buffer)-1805,now))return;
 size_t used=strlen(buffer);if(!used||buffer[used-1]!='}')return;--used;
 auto add=[&](const char* format,auto... args){if(used>=sizeof(buffer))return;const int n=snprintf(buffer+used,sizeof(buffer)-used,format,args...);if(n>0)used+=static_cast<size_t>(n)<sizeof(buffer)-used?n:sizeof(buffer)-used-1;};
 const auto imu=imu_.orientationSnapshot();
 add(",\"tel\":1,\"boot_nonce\":%lu,\"tel_seq\":%lu,\"imu\":{\"valid\":%s,\"stamp_ms\":%lu},\"servos\":[",(unsigned long)boot_nonce_,(unsigned long)++telemetry_seq_,imu.valid?"true":"false",(unsigned long)imu.sampled_at_ms);
 for(unsigned i=0;i<12;++i){const auto& r=operational_producer_.raw(i);const auto* servo=servo::legServoAt((i/3)*3+2-i%3);
  add("%s{\"slot\":%u,\"id\":%u,\"raw\":%u,\"mapped\":%s,\"energized\":%d,\"speed\":%d,\"goal\":%d,\"temp\":%d,\"volt_dv\":%d,\"status\":%d,\"age_ms\":%lu}",i?",":"",i,servo?servo->bus_id:0,r.position,operational_feedback_.read_ok[i]?"true":"false",
   r.torque,r.speed,r.goal_position,r.temperature,r.voltage,r.status,(unsigned long)(now-r.sampled_at_ms));}
 add("],\"runtime\":{\"heap_free\":%lu,\"heap_min_free\":%lu,\"heap_largest\":%lu,\"loop_stack_min_free\":%lu}}\r\n",
   (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),(unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
   (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),(unsigned long)uxTaskGetStackHighWaterMark(nullptr));
 // Admit a complete record only; USB backpressure must never delay Motion.
 // tel_seq advances on a dropped record so the bridge can observe the gap.
 if(used<sizeof(buffer)-1&&Serial.availableForWrite()>=static_cast<int>(used))
  Serial.write(reinterpret_cast<const uint8_t*>(buffer),used);
}
// Shared by @MOTION COMMISSIONING ADMIT (supervised, certificate, per-segment
// approvals) and the automatic OPERATIONAL admission at READY (measured-start
// entry, continuous route). Hard limits, route-fit and live checks are the same.
bool Controller::admitRuntimePolicy(bool operational,uint32_t issued,uint32_t now,const char** out){
  const char* reason="ADMIT";
  struct Out{const char*& r;const char** o;~Out(){if(o)*o=r;}} sink{reason,out};
  const auto& cal=runtime_restore_.snapshot();
  if(servo_preflight_.state()!=servo::ServoPreflight::State::COMPLETE||!servo_preflight_.result().allPass()){reason="LIVE_SERVO_PROFILE_CHECK_REQUIRED";return false;}
  if(!cal.restored||!cal.calibration_accepted){reason="RESTORE_REQUIRED";return false;}
  if(!operational&&!commissioning_entry_.present){reason="ENTRY_CERTIFICATE_REQUIRED";return false;}
  if(!execution::validCommissioningParameters(commissioning_params_,&reason))return false;
  calibration::RuntimeRestoreRequest facts;facts.expected_generation=cal.restored_generation;facts.current_installation_confirmed=true;
  facts.executor_idle=true;facts.motion_permit_revoked=true;
  facts.torque_off_confirmed=operational_producer_.rawFresh(now,commissioning_params_.feedback_max_age_ms);
  for(unsigned i=0;i<12;++i)facts.torque_off_confirmed=facts.torque_off_confirmed&&operational_producer_.raw(i).torque==0&&operational_producer_.raw(i).speed==0;
  if(!facts.torque_off_confirmed){reason="TORQUE_OFF_STATIONARY_FRESH_REQUIRED";return false;}
  // A reviewed card profile and this policy are never bound together.
  validation_profile_.clear();unbindCommissioning("ADMIT_IN_PROGRESS");
  const auto admission=runtime_restore_.qualifyCommissioningLimits(geometry_profile_,actuator::geometry_data::kProvenance,commissioning_params_.inset_ticks,facts,actuator_policy_.transforms(),actuator_policy_.limits());
  if(admission!=calibration::OperationalAdmissionStatus::APPROVED){reason=calibration::toString(admission);commissioning_reason_=reason;return false;}
  double limits[12][2];
  for(unsigned i=0;i<12;++i){const auto id=slotIdentity(i);const auto* tr=actuator_policy_.transforms().find(id,geometry_profile_.provenanceTag());const auto* lim=actuator_policy_.limits().find(id,geometry_profile_.provenanceTag());actuator::MicroRad lo,hi;
   if(!tr||!lim||actuator::resolveRawToUrdfQ(geometry_profile_,actuator::geometry_data::kProvenance,*tr,lim->min_tick,&lo)!=actuator::TargetResolveStatus::OK||
      actuator::resolveRawToUrdfQ(geometry_profile_,actuator::geometry_data::kProvenance,*tr,lim->max_tick,&hi)!=actuator::TargetResolveStatus::OK){
    unbindCommissioning("LIMIT_TRANSFORM_FAILED");reason=commissioning_reason_;return false;}
   limits[i][0]=std::fmin(lo,hi)*1e-6;limits[i][1]=std::fmax(lo,hi)*1e-6;}
  // Operational (2026-10-15): the continuous fixture WALK profile is bound as a
  // SPECIFICATION when its certified target union (+ tracking tube) fits the
  // live gen-1 limits; motion still needs a verified STAND and an explicit
  // Dashboard WALK START. The compiled first stride stays a USB/historical path.
  execution::CommissioningParameters params=commissioning_params_;
  if(operational){params.walk_enabled=false;
    // Issue #52: operational STAND rates/frame period derived in kStandProfile.
    if(!execution::applyOperationalStandProfile(params,&reason)){unbindCommissioning("STAND_PROFILE_REJECTED");return false;}}
  operational_walk_bound_=false;operational_walk_reason_="WALK_NOT_BOUND";
  // Stack budget (2026-10-16 boot-loop root cause): the runtime, route and
  // entry config are kilobytes; Controller-thread static storage, reset here.
  static execution::ReviewedMotionRuntime r;static const execution::ReviewedMotionRuntime kEmptyRuntime{};r=kEmptyRuntime;
  if(!execution::buildCommissioningRuntime(params,limits,cal.restored_generation,actuator_policy_.epoch(),&r)){unbindCommissioning("RUNTIME_PROFILE_REJECTED");reason=commissioning_reason_;return false;}
  // The certified fixture route must fit the hard limits as a whole.
  for(unsigned k=0;k<=motion::kPairedProneSegments;++k){const auto f=motion::pairedProneSample(k==motion::kPairedProneSegments?k-1:k,k==motion::kPairedProneSegments?1:0,true);
   for(unsigned i=0;i<12;++i){const auto& q=f.target.legs[i/3];const double v=i%3==0?q.hip:i%3==1?q.upper:q.lower;
    if(!f.target.valid||v<limits[i][0]||v>limits[i][1]){unbindCommissioning("ROUTE_OUTSIDE_COMMISSIONING_LIMITS");reason=commissioning_reason_;return false;}}}
  if(params.walk_enabled){unsigned joint=0;
   if(!motion::compiledFirstStrideFitsLimits(r.feedback,motion::compiledFirstStrideTubeRad(),&joint)){unbindCommissioning("WALK_TUBE_OUTSIDE_COMMISSIONING_LIMITS");reason=commissioning_reason_;return false;}}
  if(operational){unsigned joint=12;
   if(motion::operational_walk::fitsLimits(r.feedback,r.execution.tracking_tolerance_rad,&joint)){
    r.walk=motion::operational_walk::specification();operational_walk_bound_=true;operational_walk_reason_="WALK_BOUND";
    // WALK FAST is NOT part of this image: the speed cap stays at the STANDARD maximum, so a FAST
    // request is refused by the Controller (VELOCITY_OUTSIDE_REVIEWED_SCOPE). Deferred, not removed.
   } else {r.walk=motion::WalkSpecification{};operational_walk_reason_="WALK_RANGE_OUTSIDE_LIMITS";}  // STAND stays available
  }
  static motion::AcquisitionRoute route;static const motion::AcquisitionRoute kEmptyRoute{};route=kEmptyRoute;
  route.pairedProne=true;route.supervisedSegments=!operational;route.timeScale=r.stand_time_scale;
  if(operational){route.phaseTimed=true;route.phaseVelocityRadS=params.max_velocity_rad_s;route.phaseAccelerationRadS2=params.max_acceleration_rad_s2;
    r.execution.frame_writes=true;r.execution.timed_segments=true;
    r.execution.arrival_tolerance_rad=motion::tolerance::kArrivalRad;}
  route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
  static execution::CommissioningStandConfig cc;static const execution::CommissioningStandConfig kEmptyConfig{};cc=kEmptyConfig;cc.enabled=true;cc.certificate=commissioning_entry_;cc.minStageDurationS=params.entry_min_stage_s;
  if(operational){
   // Measured-start entry, same two-stage planner and CAD keep-out. V1's
   // compiled region proof covers every recognized start (+/-10 deg) and
   // both stages with the wider CAD tube; actual raw/URDF limits still apply.
   // Physical mounting/cover uncertainty remains a fixture qualification.
   cc.measuredStart=true;cc.pathMarginRad=.001;cc.keepoutCount=1;
   auto& k=cc.keepouts[0];k.id="LF_UPPER_LOWER_CONTACT";k.a=k.b=2;k.ca=1;k.cb=0;k.bound=-92.070442*3.14159265358979323846/180.0;
  }
  cc.proneMaxDeviationRad=commissioning_params_.prone_max_deviation_rad;cc.proneMaxRmsRad=commissioning_params_.prone_max_rms_rad;
  if(!motion_coordinator_.configure(r.feedback,r.walk,&route)||!motion_coordinator_.configureCommissioning(cc)){unbindCommissioning("PLANNER_CONFIGURATION_REJECTED");reason=commissioning_reason_;return false;}
  reviewed_motion_=r;reviewed_motion_bound_=true;commissioning_bound_=true;
  servo_bus_.setValidationTimeout(commissioning_params_.transaction_timeout_ms);servo_bus_.setValidationDrive(commissioning_params_.drive_speed,commissioning_params_.drive_acceleration);
  // Body height and support are route model references, NOT sensor measurements.
  // Operational commands do not require an operator support declaration.
  motion::AdaptiveObservation o;o.body=motion::pairedProneSample(0,0,true).cartesian.body;o.supportStampS=issued/1000.0;
  o.heightSource=o.supportSource=motion::EvidenceSource::EXTERNAL_MEASURED;o.externalSupportConfirmed=o.flatPlaneConfirmed=o.stationary=true;
  local_motion_observation_=o;local_trial_evidence_={};
  if(operational)runtime_mode_=execution::RuntimeTestMode::OPERATIONAL;
  reason=operational?"OPERATIONAL_LIMITS_AND_PLANNER_BOUND":"COMMISSIONING_LIMITS_QUALIFIED_NO_PERMIT";commissioning_reason_=reason;return true;
}
bool Controller::commissioningCommand(const char* line,uint32_t now,char* out,size_t size){
 if(line&&!strncmp(line,"TELEMETRY ",10)){
  uint32_t hz;if(!uintArg(line+10,hz)||hz>20){snprintf(out,size,"TELEMETRY=REFUSED REASON=HZ_0_TO_20");return false;}
  telemetry_period_ms_=hz?1000/hz:0;telemetry_last_ms_=now;
  snprintf(out,size,"TELEMETRY=%s HZ=%lu BOOT_NONCE=%lu FORMAT=@TEL_JSON_V1",hz?"ON":"OFF",(unsigned long)hz,(unsigned long)boot_nonce_);return true;
 }
 if(!line||strncmp(line,"COMMISSIONING ",14)){snprintf(out,size,"COMMISSIONING=REFUSED REASON=USAGE");return false;}
 line+=14;
 const char* reason="USAGE_OR_LIVE_GATE";bool ok=false,timed=false;uint32_t issued=now;
 if(!strncmp(line,"AT ",3)){const char* end=strchr(line+3,' ');char stamp[16]{};
  if(!end||end-line>16){snprintf(out,size,"COMMISSIONING=REFUSED REASON=LOCAL_COMMAND_CLOCK_INVALID");return false;}
  memcpy(stamp,line+3,end-line-3);
  if(!uintArg(stamp,issued)||static_cast<int32_t>(now-issued)<0||now-issued>250){snprintf(out,size,"COMMISSIONING=REFUSED REASON=LOCAL_COMMAND_CLOCK_STALE");return false;}
  line=end+1;timed=true;}
 const auto& cal=runtime_restore_.snapshot();
 auto reply=[&](){
  snprintf(out,size,"COMMISSIONING=%s REASON=%s MODE=%s BOUND=%u LIMITS=%s RESTORE=%s SESSION=%s ENTRY=%s SEGMENT=%u PLANNER=%s POSE=%s POSE_REASON=%s NONCE=%lu UPTIME_MS=%lu CLAMP=0x%03x EXECUTOR=%s",
   ok?"PASS":"REFUSED",reason,execution::toString(runtime_mode_),commissioning_bound_?1u:0u,
   cal.commissioning_limits_active?"COMMISSIONING_CONTACT_INSET":cal.operational_limits_approved?"REVIEWED_ROUTE":"NONE",
   cal.restored?"RESTORED":calibration::toString(cal.restore_status),operational_session_.stateName(),entryName(motion_coordinator_),
   motion_coordinator_.proneSegment(),motion_coordinator_.state(),live_pose_valid_?(live_pose_.kind==motion::FixturePoseKind::PRONE_COMPATIBLE?"PRONE_COMPATIBLE":"UNKNOWN"):"NOT_EVALUATED",
   live_pose_valid_?motion::fixturePoseReasonName(live_pose_.reason):"FEEDBACK_UNMAPPED_OR_STALE",(unsigned long)validation_nonce_,(unsigned long)now,
   operational_session_.primeClampMask(),execution::toString(motion_executor_.status().state));
  return ok;};
 if(strlen(line)>300)return reply();
 char copy[301];strcpy(copy,line);const char* t[20]{};unsigned n=0;char* save=nullptr;
 for(char* p=strtok_r(copy," ",&save);p;p=strtok_r(nullptr," ",&save)){if(n==20)return reply();t[n++]=p;}
 if(!n)return reply();
 const bool idle=authority_.current()==ActuatorAuthority::NONE&&!motion_service_.busy()&&operational_session_.state()==execution::MotionSessionState::OFF&&!motion_executor_.active();
 // --- de-escalation and read-only diagnostics: no nonce, no clock ---------
 if(!strcmp(t[0],"STOP")){
  operational_session_.revoke("LOCAL_OPERATOR_STOP");motion_executor_.stop("LOCAL_OPERATOR_STOP");
  operational_facts_.operator_permit=false;validation_present_=false;validation_walk_admitted_=false;validation_presence_ms_=0;
  validation_nonce_=esp_random();if(!validation_nonce_)validation_nonce_=1;ok=true;reason="SAFE_OFF_REQUESTED";return reply();
 }
 if(!strcmp(t[0],"STATUS")&&n==1){ok=true;reason=boot_safe_off_pending_?"BOOT_SAFE_OFF_PENDING":commissioning_reason_;return reply();}
 // Maintenance procedure (servo swap/remount): suspend the automatic boot
 // RESTORE in RAM until RESUME or reboot. Manual RESTORE stays available.
 if(!strcmp(t[0],"AUTO_RESTORE")&&n==2&&(!strcmp(t[1],"SUSPEND")||!strcmp(t[1],"RESUME"))){
  if(!idle){reason="SAFE_IDLE_REQUIRED";return reply();}
  auto_restore_suspended_=!strcmp(t[1],"SUSPEND");
  if(!auto_restore_suspended_){restore_refused_permanent_=false;restore_refused_transient_=false;}
  ok=true;reason=auto_restore_suspended_?"AUTO_RESTORE_SUSPENDED_RAM_ONLY":"AUTO_RESTORE_RESUMED";return reply();
 }
 if(!strcmp(t[0],"PARAMS")&&n==1){
  char text[760];ok=execution::formatCommissioningParameters(commissioning_params_,text,sizeof(text));
  snprintf(out,size,"COMMISSIONING_PARAMS mode=%s %s entry_certificate=%s",execution::toString(runtime_mode_),ok?text:"FORMAT_OVERFLOW",commissioning_entry_.present?commissioning_entry_.sha256:"NONE");return ok;
 }
 if(!strcmp(t[0],"CLASSIFY")&&n==1){
  updateLivePose();
  if(!live_pose_valid_){reason=cal.restored?"FEEDBACK_UNMAPPED_OR_STALE":"RESTORE_REQUIRED";return reply();}
  const auto head=motion::pairedProneSample(0,0,true);int used=snprintf(out,size,"COMMISSIONING_CLASSIFY pose=%s reason=%s joint=%u max_dev_deg=%.3f rms_dev_deg=%.3f route_tube_match=%d dev_deg=",
   live_pose_.kind==motion::FixturePoseKind::PRONE_COMPATIBLE?"PRONE_COMPATIBLE":"UNKNOWN",motion::fixturePoseReasonName(live_pose_.reason),live_pose_.failedJointIndex<12?live_pose_.failedJointIndex:live_pose_.maxErrorJointIndex,
   live_pose_.maxNominalErrorRad*57.29577951308232,live_pose_.rmsNominalErrorRad*57.29577951308232,
   reviewed_motion_bound_?motion::pairedProneCursor(local_motion_observation_,reviewed_motion_.feedback).segment:-1);
  for(unsigned i=0;i<12&&used>0&&static_cast<size_t>(used)<size;++i){const auto& q=head.target.legs[i/3];const double nominal=i%3==0?q.hip:i%3==1?q.upper:q.lower;
   used+=snprintf(out+used,size-used,"%s%.2f",i?",":"",(operational_feedback_.actual_rad[i]-nominal)*57.29577951308232);}
  return true;
 }
 if(wifi_.configBusy()){reason="NETWORK_CONFIGURATION_TRANSACTION_ACTIVE";return reply();}
 if(!build::kSupervisedValidationCapable){reason="VALIDATION_CAPABILITY_NOT_COMPILED";return reply();}
 // --- RAM-only policy selection, safe idle only ---------------------------
 if(!strcmp(t[0],"MODE")&&n==2){
  execution::RuntimeTestMode mode;if(!execution::parseRuntimeTestMode(t[1],&mode)){reason="UNKNOWN_MODE";return reply();}
  if(!idle||boot_safe_off_pending_){reason="SAFE_IDLE_REQUIRED";return reply();}
  unbindCommissioning("MODE_CHANGED");validation_profile_.clear();runtime_mode_=mode;ok=true;
  reason=mode==execution::RuntimeTestMode::DIAGNOSTIC?"DIAGNOSTIC_NO_ACTUATION":mode==execution::RuntimeTestMode::FIXTURE_COMMISSIONING?
    "FIXTURE_COMMISSIONING_SELECTED_NO_PERMIT":"REVIEWED_PROFILE_PATH_@MOTION_VALIDATION";
  commissioning_reason_=reason;return reply();
 }
 if(!strcmp(t[0],"SET")&&n==3){
  if(runtime_mode_!=execution::RuntimeTestMode::FIXTURE_COMMISSIONING){reason="FIXTURE_COMMISSIONING_MODE_REQUIRED";return reply();}
  if(!idle){reason="SAFE_IDLE_REQUIRED";return reply();}
  auto next=commissioning_params_;
  if(!execution::setCommissioningParameter(next,t[1],t[2],&reason))return reply();
  unbindCommissioning("PARAMETERS_CHANGED_READMIT_REQUIRED");commissioning_params_=next;ok=true;return reply();
 }
 if(!strcmp(t[0],"ENTRY")&&n==15){
  if(runtime_mode_!=execution::RuntimeTestMode::FIXTURE_COMMISSIONING){reason="FIXTURE_COMMISSIONING_MODE_REQUIRED";return reply();}
  if(!idle){reason="SAFE_IDLE_REQUIRED";return reply();}
  motion::FixtureEntryCertificate c;c.present=true;c.start.valid=true;
  for(unsigned i=0;i<12;++i){long u;if(!intArg(t[i+1],u)||u<-3200000||u>3200000){reason="ENTRY_START_URAD";return reply();}slot(c.start,i)=u*1e-6;}
  uint32_t tube;if(!uintArg(t[13],tube)||tube==0||tube>20000||!hex64(t[14])){reason="ENTRY_TUBE_OR_DIGEST";return reply();}
  c.tubeRad=tube*1e-6;strcpy(c.sha256,t[14]);c.goal=motion::pairedProneSample(0,0,true).target;
  if(!c.goal.valid){reason="ROUTE_HEAD_INVALID";return reply();}
  unbindCommissioning("ENTRY_CERTIFICATE_LOADED_READMIT_REQUIRED");commissioning_entry_=c;ok=true;reason="ENTRY_CERTIFICATE_RAM_ONLY";return reply();
 }
 // --- everything below needs the local issued clock and the current nonce -
 if(!timed){reason="LOCAL_ISSUED_CLOCK_REQUIRED";return reply();}
 uint32_t nonce=0;if(n<2||!uintArg(t[1],nonce)||nonce!=validation_nonce_){reason="NONCE";return reply();}
 const bool present=validation_present_&&now-validation_presence_ms_<=1000;
 if(!strcmp(t[0],"ADMIT")&&n==3&&!strcmp(t[2],"CONFIRM_FIXTURE_SUPPORT_FREE_VOLUME_POWER_CUT_INSTALLATION")){
  if(runtime_mode_!=execution::RuntimeTestMode::FIXTURE_COMMISSIONING){reason="FIXTURE_COMMISSIONING_MODE_REQUIRED";return reply();}
  if(!idle||boot_safe_off_pending_){reason="SAFE_IDLE_REQUIRED";return reply();}
  ok=admitRuntimePolicy(false,issued,now,&reason);return reply();
 }
 if(!commissioning_bound_){reason="COMMISSIONING_ADMIT_REQUIRED";return reply();}
 if(!strcmp(t[0],"PRESENT")&&n==3&&!strcmp(t[2],"CONFIRM_PRESENT_FIXTURE_POWER_CUT_FREE_VOLUME")){
  validation_present_=true;validation_presence_ms_=issued;local_motion_observation_.supportStampS=issued/1000.0;
  local_trial_evidence_.local_operator_present=true;ok=true;reason="LOCAL_DEADMAN_REFRESHED";return reply();
 }
 if(!strcmp(t[0],"GRANT")&&n==4){
  uint32_t ttl;if(!uintArg(t[2],ttl)||ttl<1000||ttl>60000||!present){reason="PRESENCE_OR_TTL";return reply();}
  if(operational_session_.state()!=execution::MotionSessionState::OFF){reason="SESSION_BUSY";return reply();}
  for(unsigned i=0;i<12;++i)if(operational_producer_.raw(i).speed!=0||operational_producer_.raw(i).torque!=0){reason="STATIONARY_TORQUE_OFF_REQUIRED_FOR_PRIME";return reply();}
  // A folded LOWER resting just outside its hard limit is primed inward; the
  // operator must name that explicitly. Nothing else is ever re-aligned.
  bool inward=false;
  for(unsigned i=2;i<12;i+=3){const auto id=slotIdentity(i);const auto* tr=actuator_policy_.transforms().find(id,geometry_profile_.provenanceTag());const auto* lim=actuator_policy_.limits().find(id,geometry_profile_.provenanceTag());
   uint16_t tick=0;if(tr&&lim&&operational_feedback_.read_ok[i]&&actuator::resolveUrdfQToRaw(geometry_profile_,actuator::geometry_data::kProvenance,*tr,static_cast<actuator::MicroRad>(std::llround(operational_feedback_.actual_rad[i]*1e6)),&tick)==actuator::TargetResolveStatus::OK)
    inward=inward||!lim->contains(tick);}
  if(strcmp(t[3],inward?"CONFIRM_FIXTURE_PRIME_HOLD_LOWER_INWARD":"CONFIRM_FIXTURE_PRIME_HOLD")){reason=inward?"LOWER_INWARD_PRIME_CONFIRMATION_REQUIRED":"PRIME_CONFIRMATION_REQUIRED";return reply();}
  execution::LocalTrialEvidence e;e.scope=execution::TrialScope::FIXTURE_COMMISSIONING;e.generation=++validation_sequence_;e.expires_at_ms=issued+ttl;
  e.local_operator_present=e.physical_power_cut_verified=e.installation_verified=e.servo_output_ceiling_verified=e.fixture_support_verified=true;
  e.imu_fresh=operational_facts_.hardware_qualified;e.prime_clamp_max_ticks=commissioning_params_.prime_clamp_ticks;
  operational_facts_.operator_permit=true;operational_facts_.permit_generation=e.generation;operational_facts_.permit_expires_at_ms=e.expires_at_ms;
  local_trial_evidence_=e;
  ok=operational_session_.request(e,operational_facts_,reviewed_motion_.execution,operational_feedback_,build::kSupervisedValidationCapable&&commissioning_bound_&&present,now);
  if(!ok)operational_facts_.operator_permit=false;else{validation_nonce_=esp_random();if(!validation_nonce_)validation_nonce_=1;}
  reason=operational_session_.reason();return reply();
 }
 if(!strcmp(t[0],"RENEW")&&n==4&&!strcmp(t[3],"CONFIRM_CONTINUED_LOCAL_SUPERVISION")){
  uint32_t ttl;if(!uintArg(t[2],ttl)||ttl<1000||ttl>60000||!present||operational_session_.state()!=execution::MotionSessionState::ARMED){reason="PRESENCE_TTL_OR_NOT_ARMED";return reply();}
  operational_facts_.permit_expires_at_ms=local_trial_evidence_.expires_at_ms=issued+ttl;ok=true;reason="FINITE_PERMIT_EXPLICITLY_RENEWED";return reply();
 }
 if(!strcmp(t[0],"STAND")&&n==2){
  if(!present||operational_session_.state()!=execution::MotionSessionState::ARMED){reason="PRESENCE_OR_NOT_ARMED";return reply();}
  auto c=localMotionCommand(motion::Intent::STAND,++validation_sequence_,issued,1000);
  const auto a=motion_service_.command(c,now);ok=a.outcome==motion::CommandOutcome::ACCEPTED;reason=a.reason;return reply();
 }
 if(!strcmp(t[0],"NEXT")&&n==4&&!strcmp(t[3],"CONFIRM_DIRECTIONS_CLEARANCE_TRACKING_STOP")){
  if(!present||operational_session_.state()!=execution::MotionSessionState::ARMED||motion_executor_.status().state!=execution::ExecutorState::READY){reason="PRESENCE_ARMED_READY_REQUIRED";return reply();}
  const auto& stand=motion_coordinator_.stand();uint32_t segment=0;
  if(!strcmp(t[2],"E0")||!strcmp(t[2],"E1")){ok=motion_coordinator_.approveEntryStage(t[2][1]-'0');reason=ok?"ONE_ENTRY_STAGE_APPROVED":"ENTRY_STAGE_NOT_CURRENT_OR_PENDING";}
  else if(!strcmp(t[2],"PHASE")){
   if(commissioning_params_.approval==execution::SegmentApproval::SEGMENT){reason="POLICY_APPROVAL_SEGMENT";return reply();}
   unsigned last=stand.proneSegment();const auto phase=motion::pairedPronePhase(last);
   while(last+1<motion::kPairedProneSegments&&motion::pairedPronePhase(last+1)==phase)++last;
   ok=motion_coordinator_.approveProneThrough(last);reason=ok?"CURRENT_PHASE_APPROVED":"SEGMENT_NOT_CURRENT_OR_PENDING";}
  else if(!strcmp(t[2],"ALL")){
   if(commissioning_params_.approval!=execution::SegmentApproval::CONTINUOUS){reason="POLICY_APPROVAL_NOT_CONTINUOUS";return reply();}
   ok=motion_coordinator_.approveProneThrough(motion::kPairedProneSegments-1);reason=ok?"REMAINING_ROUTE_APPROVED_DEADMAN_HELD":"SEGMENT_NOT_CURRENT_OR_PENDING";}
  else if(uintArg(t[2],segment)){ok=motion_coordinator_.approveProneSegment(segment);reason=ok?"ONE_CERTIFIED_SEGMENT_APPROVED":"SEGMENT_NOT_CURRENT_OR_PENDING";}
  return reply();
 }
 if((!strcmp(t[0],"WALK_START")||!strcmp(t[0],"WALK"))&&n==3&&!strcmp(t[2],"CONFIRM_SUSPENDED_FIRST_STRIDE_CLEARANCE")){
  if(!commissioning_params_.walk_enabled){reason="WALK_NOT_ENABLED_IN_POLICY";return reply();}
  if(!present||operational_session_.state()!=execution::MotionSessionState::ARMED){reason="PRESENCE_OR_NOT_ARMED";return reply();}
  const bool start=!strcmp(t[0],"WALK_START");
  if(!validation_walk_admitted_&&!start){reason="EXPLICIT_FIRST_STRIDE_START_REQUIRED";return reply();}
  if(validation_walk_admitted_&&start){reason="FIRST_STRIDE_ALREADY_ACTIVE";return reply();}
  if(!validation_walk_admitted_){
   if(strcmp(motion_coordinator_.state(),"STAND")||motion_executor_.status().state!=execution::ExecutorState::READY){reason="STAND_VERIFIED_REQUIRED";return reply();}
   validation_walk_admitted_=true;validation_walk_started_=false;
   auto ready=motion_service_.readiness();ready.walk_qualified=true;ready.max_vx=reviewed_motion_.walk.maxVxMps;ready.max_vy=0;ready.max_yaw_rate=0;motion_service_.refresh(ready,now);
  }
  auto c=localMotionCommand(motion::Intent::BASE_VELOCITY,++validation_sequence_,issued,200);c.vx=reviewed_motion_.walk.maxVxMps;
  const auto a=motion_service_.command(c,now);ok=a.outcome==motion::CommandOutcome::ACCEPTED;if(!ok)validation_walk_admitted_=false;reason=a.reason;return reply();
 }
 return reply();
}
} }
