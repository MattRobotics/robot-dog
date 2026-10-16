#include "MotionService.h"
#include "../motion/OperationalWalkProfile.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdarg>
namespace matdog { namespace core {
namespace {
struct JsonWriter {
 char* out;size_t cap,used=0;bool ok=true;
 void add(const char* format,...) { if(!ok||used>=cap){ok=false;return;}va_list args;va_start(args,format);const int n=vsnprintf(out+used,cap-used,format,args);va_end(args);if(n<0||static_cast<size_t>(n)>=cap-used){ok=false;out[cap-1]=0;return;}used+=n; }
};
bool sameCommand(const motion::MotionCommand& a,const motion::MotionCommand& b) {
 return a.version==b.version&&!strcmp(a.id,b.id)&&a.sequence==b.sequence&&a.issued_at_ms==b.issued_at_ms&&a.ttl_ms==b.ttl_ms&&a.owner==b.owner&&a.intent==b.intent&&a.target_height_m==b.target_height_m&&a.vx==b.vx&&a.vy==b.vy&&a.yaw_rate==b.yaw_rate&&a.x==b.x&&a.y==b.y&&a.z==b.z&&a.roll==b.roll&&a.pitch==b.pitch&&a.yaw==b.yaw&&!strcmp(a.gait,b.gait)&&!strcmp(a.action_id,b.action_id);
}
}
void MotionService::begin(execution::MotionExecutor* e,SemanticMotionPlanner* p) { executor_=e;planner_=p;readiness_=MotionReadiness{};last_ack_=active_ack_=motion::MotionAck{};active_intent_=motion::Intent::UNKNOWN;active_command_=active_zero_velocity_=false;arming_seen_=false;telemetry_=MotionTelemetry{};for(auto& r:replay_)r=Replay{};command_owner_=motion::ClientOwner::UNKNOWN;teleop_deadline_ms_=active_arm_generation_=0; }
const char* MotionService::refusal() const {
 if(readiness_.fault)return "FAULT";
 if(readiness_.inhibited)return "INHIBITED";
 if(!readiness_.calibration_restored)return "RESTORE_REQUIRED";
 if(!readiness_.calibration_accepted)return "CALIBRATION_NOT_ACCEPTED";
 if(!readiness_.limits_approved)return "OPERATIONAL_LIMITS_REQUIRED";
 if(!readiness_.hardware_approved)return "HARDWARE_QUALIFICATION_REQUIRED";
 if(!readiness_.operational_permit)return "OPERATOR_GO_REQUIRED";
 if(readiness_.local_fixture_session)return "LOCAL_FIXTURE_MICRO_ONLY";
 if(!readiness_.support_observable)return "SUPPORT_UNOBSERVABLE";
 if(!readiness_.power_ok||!readiness_.bms_ok)return "POWER_OR_BMS";
 if(readiness_.mode!=OperatingMode::RUN)return "MODE_RUN_REQUIRED";
 if(!readiness_.authority_live)return "MOTION_AUTHORITY_REQUIRED";
 if(!executor_||!planner_)return "EXECUTOR_NOT_BOUND";
 return nullptr;
}
bool MotionService::available(const motion::MotionAction& a) const { return a.implemented&&((!readiness_.local_operator_owns_session&&(command_owner_!=motion::ClientOwner::LOCAL||!active_command_))||a.intent==motion::Intent::STOP||a.intent==motion::Intent::STATUS)&&(a.intent!=motion::Intent::BASE_VELOCITY||readiness_.walk_qualified)&&(a.intent==motion::Intent::STOP||a.intent==motion::Intent::STATUS||!refusal()); }
void MotionService::finishActive(motion::CommandOutcome outcome,const char* reason) {
 if(!active_command_)return;
 active_ack_.outcome=outcome;active_ack_.reason=reason;
 active_ack_.executor_state=executor_?execution::toString(executor_->status().state):"OFF";
 if(static_cast<unsigned>(active_ack_.owner)<3){
  auto& row=replay_[static_cast<unsigned>(active_ack_.owner)];
  if(row.sequence==active_ack_.sequence&&!strcmp(row.ack.id,active_ack_.id))row.ack=active_ack_;
 }
 if(last_ack_.owner==active_ack_.owner&&last_ack_.sequence==active_ack_.sequence&&!strcmp(last_ack_.id,active_ack_.id))last_ack_=active_ack_;
 active_command_=false;command_owner_=motion::ClientOwner::UNKNOWN;teleop_deadline_ms_=0;
}
void MotionService::refresh(const MotionReadiness& r,uint32_t now) {
 readiness_=r;
 // An accepted operational STAND arms its own session: adopt that arm
 // generation once, at the hand-over, instead of reading it as a change.
 if(planner_&&planner_->arming()){if(executor_)active_arm_generation_=executor_->status().arm_generation;arming_seen_=true;}
 else if(arming_seen_){arming_seen_=false;if(active_command_&&executor_&&executor_->active())active_arm_generation_=executor_->status().arm_generation;}
 if(active_command_&&executor_&&active_arm_generation_!=executor_->status().arm_generation)
  finishActive(motion::CommandOutcome::ABORTED,"SESSION_CHANGED");
 const char* gate=refusal();
 if((gate&&((executor_&&executor_->active()&&!r.local_fixture_session)||active_command_))||
    (teleop_deadline_ms_&&static_cast<int32_t>(now-teleop_deadline_ms_)>=0))disconnected(now);
 if(!active_command_)return;
 const auto state=executor_?executor_->status().state:execution::ExecutorState::OFF;
 const bool standing=planner_&&!strcmp(planner_->state(),"STAND")&&state==execution::ExecutorState::READY;
 if(active_intent_==motion::Intent::STOP){
  if(executor_&&state==execution::ExecutorState::OFF&&!executor_->status().safe_off_required)
   finishActive(motion::CommandOutcome::COMPLETED,"SAFE_OFF_VERIFIED");
  else if(standing)finishActive(motion::CommandOutcome::COMPLETED,"COORDINATED_STOP_VERIFIED");
 } else if(planner_&&planner_->arming()){
  // Operational STAND waiting for its controlled torque sequence (session).
 } else if(executor_&&(!executor_->active()||executor_->status().safe_off_required)){
  finishActive(motion::CommandOutcome::ABORTED,executor_->status().reason);
 } else if(!gate&&standing&&active_intent_==motion::Intent::STAND){
  finishActive(motion::CommandOutcome::COMPLETED,"STAND_VERIFIED");
 } else if(!gate&&standing&&active_intent_==motion::Intent::BASE_VELOCITY&&(active_zero_velocity_||readiness_.first_stride_only)){
  finishActive(motion::CommandOutcome::COMPLETED,"ZERO_VELOCITY_VERIFIED");
 }
}
void MotionService::disconnected(uint32_t now) {
 const char* gate=refusal();
 if(planner_)planner_->stop(now);
 if(executor_&&(gate||!planner_||!planner_->coordinatedStopping()))executor_->stop("INPUT_LOST_OR_EXPIRED");
 // STOP must retain its acknowledgement until SAFE_OFF or the all-foot stop
 // is independently verified. Other accepted work is explicitly aborted.
 if(active_intent_!=motion::Intent::STOP)finishActive(motion::CommandOutcome::ABORTED,gate?gate:"INPUT_LOST_OR_EXPIRED");
 teleop_deadline_ms_=0;command_owner_=motion::ClientOwner::UNKNOWN;
}
motion::MotionAck MotionService::command(const motion::MotionCommand& incoming,uint32_t now) {
 motion::MotionAck ack;std::memcpy(ack.id,incoming.id,sizeof(ack.id));ack.id[sizeof(ack.id)-1]=0;ack.sequence=incoming.sequence;ack.owner=incoming.owner;ack.executor_state=executor_?execution::toString(executor_->status().state):"OFF";
 const char* why=nullptr;if(!motion::validCommand(incoming,now,&why)){ack.reason=why;last_ack_=ack;return ack;}
 why=nullptr;
 auto& r=replay_[static_cast<unsigned>(incoming.owner)];
 if(incoming.intent!=motion::Intent::STOP) {
  if(incoming.sequence==r.sequence){if(sameCommand(incoming,r.command))return r.ack;ack.reason="COMMAND_REPLAY_MISMATCH";last_ack_=ack;return ack;}
  if(static_cast<int32_t>(incoming.sequence-r.sequence)<=0){ack.reason="SEQUENCE_REPLAY";last_ack_=ack;return ack;}
  if(r.sequence&&!strcmp(incoming.id,r.command.id)){ack.reason="COMMAND_ID_REUSED";last_ack_=ack;return ack;}
 }
 motion::MotionCommand c=incoming;
 if(c.intent==motion::Intent::ACTION_EXECUTE) {
  const auto* a=motion::findAction(c.action_id);
  if(!a)why="UNKNOWN_ACTION";else if(!a->implemented)why="ACTION_NOT_VALIDATED";else {c.intent=a->intent;if(!strcmp(a->id,"gait.walk"))std::strcpy(c.gait,"WALK");}
 }
 if(!why&&readiness_.local_operator_owns_session&&c.owner!=motion::ClientOwner::LOCAL&&c.intent!=motion::Intent::STOP&&c.intent!=motion::Intent::STATUS&&c.intent!=motion::Intent::CAPABILITIES)why="LOCAL_OPERATOR_CONTROLS_SESSION";
 if(!why)switch(c.intent) {
  case motion::Intent::STOP:
   finishActive(motion::CommandOutcome::ABORTED,"STOP_REQUESTED");
   if(planner_)planner_->stop(now);
   if(executor_&&(!planner_||!planner_->coordinatedStopping()))executor_->stop("OPERATOR_STOP");
   teleop_deadline_ms_=0;command_owner_=motion::ClientOwner::UNKNOWN;ack.outcome=(planner_&&planner_->coordinatedStopping())||(executor_&&executor_->status().safe_off_required)?motion::CommandOutcome::ACCEPTED:motion::CommandOutcome::COMPLETED;break;
  case motion::Intent::STATUS:case motion::Intent::CAPABILITIES:ack.outcome=motion::CommandOutcome::COMPLETED;break;
  case motion::Intent::LIE_DOWN:case motion::Intent::BODY_POSE:case motion::Intent::ACTION_EXECUTE:why="ACTION_NOT_VALIDATED";break;
  case motion::Intent::STAND:
   if(c.target_height_m<.06||c.target_height_m>.150)why="HEIGHT_OUTSIDE_SOFTWARE_ENVELOPE";
   else why=refusal();
   break;
  case motion::Intent::GAIT_SELECT:
   if(strcmp(c.gait,"WALK"))why="GAIT_NOT_VALIDATED";else why=refusal();
   break;
  case motion::Intent::BASE_VELOCITY:
   if(std::fabs(c.vx)>.03||std::fabs(c.vy)>.02||std::fabs(c.yaw_rate)>.15)why="VELOCITY_OUTSIDE_SOFTWARE_ENVELOPE";else if((why=refusal())){}else if(!readiness_.walk_qualified)why="LOCAL_WALK_ADMISSION_REQUIRED";else if(std::fabs(c.vx)>readiness_.max_vx||std::fabs(c.vy)>readiness_.max_vy||std::fabs(c.yaw_rate)>readiness_.max_yaw_rate)why="VELOCITY_OUTSIDE_REVIEWED_SCOPE";
   break;
  default:why="INTENT";break;
 }
 if(!why&&c.intent!=motion::Intent::STOP&&ack.outcome!=motion::CommandOutcome::COMPLETED) {
  if(command_owner_!=motion::ClientOwner::UNKNOWN&&command_owner_!=c.owner)why="COMMAND_OWNER_BUSY";
  else if(active_command_&&(active_intent_!=motion::Intent::BASE_VELOCITY||c.intent!=motion::Intent::BASE_VELOCITY))why="COMMAND_IN_PROGRESS";
  else {
   why=planner_->accept(c,now);
   if(!why){
    if(c.intent==motion::Intent::GAIT_SELECT)ack.outcome=motion::CommandOutcome::COMPLETED;
    else {finishActive(motion::CommandOutcome::ABORTED,"SUPERSEDED");ack.outcome=motion::CommandOutcome::ACCEPTED;command_owner_=c.owner;}
    if(c.intent==motion::Intent::BASE_VELOCITY)teleop_deadline_ms_=c.issued_at_ms+c.ttl_ms;
   }
  }
 }
 ack.reason=why?why:(c.intent==motion::Intent::GAIT_SELECT?"GAIT_SELECTED":"OK");ack.executor_state=executor_?execution::toString(executor_->status().state):"OFF";
 if(static_cast<int32_t>(incoming.sequence-r.sequence)>0){r.sequence=incoming.sequence;r.command=incoming;r.ack=ack;}
 if(ack.outcome==motion::CommandOutcome::ACCEPTED){active_ack_=ack;active_arm_generation_=executor_?executor_->status().arm_generation:0;active_intent_=c.intent;active_command_=true;active_zero_velocity_=c.intent==motion::Intent::BASE_VELOCITY&&c.vx==0&&c.vy==0&&c.yaw_rate==0;}
 last_ack_=ack;return ack;
}
bool MotionService::ackJson(const motion::MotionAck& a,char* out,size_t cap) const {
 if(!out||!cap)return false;
 JsonWriter w{out,cap};
 // IDs were validated as ASCII tokens. Invalid envelope IDs are never echoed.
 char id[49]{};for(unsigned i=0;i<48&&a.id[i];i++){const char c=a.id[i];id[i]=((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='.'||c=='_'||c=='-')?c:'_';}
 w.add("{\"version\":1,\"id\":\"%s\",\"sequence\":%lu,\"ack\":true,\"outcome\":\"%s\",\"reason\":\"%s\",\"owner\":\"%s\",\"executor_state\":\"%s\"}",id,(unsigned long)a.sequence,motion::toString(a.outcome),a.reason,motion::toString(a.owner),a.executor_state);return w.ok;
}
bool MotionService::capabilitiesJson(char* out,size_t cap) const {
 if(!out||!cap)return false;
 JsonWriter w{out,cap};const auto* refusal_reason=refusal();
 w.add("{\"version\":1,\"walk\":{\"period_s\":%.17g,\"lift_m\":%.17g,\"strides_m\":[%.17g,%.17g,%.17g],\"fast\":{\"period_s\":%.17g,\"lift_m\":%.17g,\"stride_m\":%.17g,\"experimental\":true}},\"first_stride_only\":%s,\"hardware_approved\":%s,\"teleoperation\":%s,\"limits\":{\"vx_m_s\":%.17g,\"vy_m_s\":%.17g,\"yaw_rate_rad_s\":%.17g},\"gaits\":[\"WALK\"],\"actions\":[",motion::operational_walk::kPeriodS,motion::operational_walk::kLiftM,motion::operational_walk::kStrideM[0],motion::operational_walk::kStrideM[1],motion::operational_walk::kStrideM[2],motion::operational_walk::kFastPeriodS,motion::operational_walk::kFastLiftM,motion::operational_walk::kFastStrideM[0],readiness_.first_stride_only?"true":"false",readiness_.hardware_approved?"true":"false",(refusal_reason||!readiness_.walk_qualified||(command_owner_==motion::ClientOwner::LOCAL&&active_command_))?"false":"true",readiness_.walk_qualified?readiness_.max_vx:0,readiness_.walk_qualified?readiness_.max_vy:0,readiness_.walk_qualified?readiness_.max_yaw_rate:0);
 for(size_t i=0;i<motion::kMotionActionCount;i++){const auto& a=motion::actionCatalog()[i];w.add("%s{\"id\":\"%s\",\"category\":\"%s\",\"label\":\"%s\",\"parameters\":\"%s\",\"prerequisites\":\"%s\",\"capability\":\"%s\",\"implemented\":%s,\"implementation\":\"%s\",\"available\":%s,\"reason\":\"%s\"}",i?",":"",a.id,a.category,a.label,a.parameters,(a.intent==motion::Intent::STOP||a.intent==motion::Intent::STATUS)?"none":a.prerequisites,a.capability,a.implemented?"true":"false",a.implemented?"software_tested":"planned",available(a)?"true":"false",available(a)?"OK":a.implemented?(refusal_reason?refusal_reason:(readiness_.local_operator_owns_session||(command_owner_==motion::ClientOwner::LOCAL&&active_command_))?"LOCAL_OPERATOR_CONTROLS_SESSION":a.intent==motion::Intent::BASE_VELOCITY&&!readiness_.walk_qualified?"LOCAL_WALK_ADMISSION_REQUIRED":"UNAVAILABLE"):"ACTION_NOT_VALIDATED");}
 w.add("]}");return w.ok;
}
bool MotionService::statusJson(char* out,size_t cap,uint32_t now) const {
 if(!out||!cap)return false;
 JsonWriter w{out,cap};const bool stale=!telemetry_.joints_valid||static_cast<int32_t>(now-telemetry_.sampled_at_ms)<0||now-telemetry_.sampled_at_ms>telemetry_.feedback_max_age_ms;
 const auto* s=executor_?&executor_->status():nullptr;const char* r=refusal();
 w.add("{\"version\":1,\"now_ms\":%lu,\"last_sequence\":%lu,\"owner\":\"%s\",\"executor_state\":\"%s\",\"stand_state\":\"%s\",\"fault\":%s,\"reason\":\"%s\",\"stale\":%s,\"tracking_error_rad\":%.6f,\"completed_sequence\":%lu,\"telemetry\":{\"sampled_at_ms\":%lu,\"actual_rad\":",(unsigned long)now,(unsigned long)last_ack_.sequence,motion::toString(command_owner_),s?execution::toString(s->state):"OFF",planner_?planner_->state():"OBSERVE",readiness_.fault?"true":"false",r?r:(s?s->reason:"OFF"),stale?"true":"false",s?s->max_tracking_error_rad:0,(unsigned long)(s?s->completed_sequence:0),(unsigned long)telemetry_.sampled_at_ms);
 if(telemetry_.joints_valid){w.add("[");for(unsigned i=0;i<12;i++)w.add("%s%.8f",i?",":"",telemetry_.actual_rad[i]);w.add("]");}else w.add("null");
 // Per-joint measured angles: null where unmapped/unhealthy, never a zero.
 w.add(",\"actual_rad_by_joint\":[");for(unsigned i=0;i<12;++i){if(telemetry_.joint_mapped[i])w.add("%s%.8f",i?",":"",telemetry_.actual_rad[i]);else w.add("%snull",i?",":"");}w.add("]");
 w.add(",\"joint_at_ms\":[");for(unsigned i=0;i<12;++i)w.add("%s%lu",i?",":"",static_cast<unsigned long>(telemetry_.joint_at_ms[i]));w.add("]");
 w.add(",\"raw_ticks\":");if(telemetry_.raw_valid){w.add("[");for(unsigned i=0;i<12;++i)w.add("%s%u",i?",":"",telemetry_.raw_ticks[i]);w.add("]");}else w.add("null");
 w.add(",\"target_rad\":");if(s&&s->accepted_sequence){w.add("[");for(unsigned i=0;i<12;i++)w.add("%s%.8f",i?",":"",executor_->targetRadians()[i]);w.add("]");}else w.add("null");
 w.add(",\"quaternion\":");if(telemetry_.quaternion_valid&&now-telemetry_.quaternion_at_ms<=500){w.add("[%.8f,%.8f,%.8f,%.8f]",telemetry_.quaternion[0],telemetry_.quaternion[1],telemetry_.quaternion[2],telemetry_.quaternion[3]);}else w.add("null");
 w.add(",\"base_height_m\":");if(telemetry_.height_valid)w.add("%.6f",telemetry_.base_height_m);else w.add("null");
 w.add(",\"source\":\"%s\",\"feet\":null},\"stale_scope\":\"MOTION_JOINT_FEEDBACK\"",telemetry_.source);
 {unsigned mapped=0,raw=0;uint32_t raw_age=0;uint16_t mask=0;
  for(unsigned i=0;i<12;++i){if(telemetry_.joint_mapped[i]){++mapped;mask|=static_cast<uint16_t>(1u<<i);}
   if(telemetry_.raw_ok[i]&&static_cast<int32_t>(now-telemetry_.joint_at_ms[i])>=0&&now-telemetry_.joint_at_ms[i]<=telemetry_.feedback_max_age_ms){++raw;const uint32_t a=now-telemetry_.joint_at_ms[i];if(a>raw_age)raw_age=a;}}
  const bool imu=telemetry_.quaternion_valid&&static_cast<int32_t>(now-telemetry_.quaternion_at_ms)>=0;
  w.add(",\"channels\":{\"imu\":{\"valid\":%s,\"age_ms\":",imu?"true":"false");
  if(imu)w.add("%lu",static_cast<unsigned long>(now-telemetry_.quaternion_at_ms));else w.add("null");
  w.add("},\"raw\":{\"count\":%u,\"max_age_ms\":",raw);if(raw)w.add("%lu",static_cast<unsigned long>(raw_age));else w.add("null");
  w.add("},\"mapped\":{\"count\":%u,\"mask\":%u},\"target\":{\"valid\":%s},\"motion_feedback_stale\":%s}",
   mapped,mask,s&&s->accepted_sequence?"true":"false",stale?"true":"false");}
 w.add(",\"restore\":{\"decision\":\"%s\",\"reason\":\"%s\",\"trigger\":\"%s\",\"generation\":%lu,\"fingerprint\":\"%016llx\"}",
  telemetry_.restore_decision,telemetry_.restore_reason,telemetry_.restore_trigger,(unsigned long)telemetry_.restore_generation,(unsigned long long)telemetry_.restore_fingerprint);
 w.add(",\"operational\":{\"system_health\":\"%s\",\"calibration\":\"%s\",\"joint_feedback\":\"%s\",\"motion_authority\":\"%s\",\"phase\":\"%s\",\"reason\":\"%s\",\"mode\":\"%s\",\"entry_reason\":\"%s\",\"operator_present\":%s,\"walk\":\"%s\",\"walk_cycles\":%.3f,\"walk_drive\":%s}",
  telemetry_.system_health,telemetry_.calibration_state,telemetry_.joint_feedback,telemetry_.motion_authority,telemetry_.readiness_phase,
  telemetry_.readiness_reason,telemetry_.operating_mode,telemetry_.entry_reason,telemetry_.operator_present?"true":"false",telemetry_.walk,telemetry_.walk_cycles,telemetry_.walk_drive?"true":"false");
 w.add(",\"active_command\":");
 char active[512]{};if(active_ack_.sequence&&ackJson(active_ack_,active,sizeof(active)))w.add("%s",active);else w.add("null");
 w.add(",\"readiness\":{\"restore\":%s,\"calibration_accepted\":%s,\"limits_approved\":%s,\"permit\":%s,\"support_observable\":%s,\"hardware_approved\":%s}",
 readiness_.calibration_restored?"true":"false",readiness_.calibration_accepted?"true":"false",readiness_.limits_approved?"true":"false",readiness_.operational_permit?"true":"false",readiness_.support_observable?"true":"false",readiness_.hardware_approved?"true":"false");
 w.add(",\"local_session\":{\"state\":\"%s\",\"reason\":\"%s\"}",telemetry_.session_state,telemetry_.session_reason);
 w.add(",\"validation\":{\"capable\":%s,\"scope\":\"%s\",\"segment\":%u,\"profile_sha256\":\"%s\",\"foot_load_proven\":%s}",telemetry_.validation_capable?"true":"false",telemetry_.fixture_scope?"SUSPENDED_FIXTURE":telemetry_.load_scope?"EXTERNAL_LOAD_REVIEW":"UNQUALIFIED",telemetry_.supervised_segment,telemetry_.session_profile_sha256,telemetry_.foot_load_proven?"true":"false");
 w.add(",\"commissioning\":{\"mode\":\"%s\",\"limits\":\"%s\",\"approved_joints\":%u,\"commissioning_joints\":%u,\"restore\":\"%s\",\"pose\":\"%s\",\"pose_reason\":\"%s\",\"pose_joint\":%d,\"pose_max_dev_rad\":",
  telemetry_.runtime_mode,telemetry_.limits_scope,telemetry_.approved_joints,telemetry_.commissioning_joints,telemetry_.restore,telemetry_.pose_class,telemetry_.pose_reason,telemetry_.pose_joint);
 if(telemetry_.pose_max_dev_rad>=0)w.add("%.6f",telemetry_.pose_max_dev_rad);else w.add("null");
 w.add(",\"pose_rms_dev_rad\":");if(telemetry_.pose_rms_dev_rad>=0)w.add("%.6f",telemetry_.pose_rms_dev_rad);else w.add("null");
 w.add(",\"entry\":\"%s\",\"phase\":\"%s\",\"segment\":%u,\"session\":\"%s\",\"fixture\":\"%s\",\"prime_clamp_mask\":%u,\"reason\":\"%s\"}",
  telemetry_.entry,telemetry_.phase,telemetry_.supervised_segment,telemetry_.session_state,telemetry_.fixture_declared?"OPERATOR_DECLARED":"NONE",telemetry_.prime_clamp_mask,telemetry_.commissioning_reason);
 w.add("}");return w.ok;
}
} }
