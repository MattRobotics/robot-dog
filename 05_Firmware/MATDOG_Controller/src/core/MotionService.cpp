#include "MotionService.h"
#include "../actuator/CalibrationTargetResolver.h"
#include <esp_timer.h>
#include <cmath>
#include <cstdio>
#include <cstring>
namespace matdog { namespace core {
namespace {
const char* boolean(bool b){return b?"true":"false";}
}
void MotionService::update(){
 if(!active_||!motion_)return;
 if(motion_->state()==simple::State::STANDING){activeAck_.outcome=motion::CommandOutcome::COMPLETED;activeAck_.reason="STAND_REACHED";active_=false;}
 if(motion_->state()==simple::State::FAULT||motion_->state()==simple::State::STOPPED){activeAck_.outcome=motion::CommandOutcome::ABORTED;activeAck_.reason=motion_->reason();active_=false;}
 activeAck_.executor_state=simple::name(motion_->state());
}
motion::MotionAck MotionService::command(const motion::MotionCommand& c,uint32_t now){
 motion::MotionAck a;a.sequence=c.sequence;a.owner=c.owner;
 const char* why=nullptr;
 if(!motion::validCommand(c,now,&why)){a.reason=why;return a;}
 memcpy(a.id,c.id,sizeof(a.id));
 if(!motion_){a.reason="NOT_INITIALIZED";return a;}
 a.executor_state=simple::name(motion_->state());
 if(c.intent==motion::Intent::STOP){motion_->stop();a.outcome=motion::CommandOutcome::ACCEPTED;a.reason="SAFE_OFF_REQUESTED";return last_=a;}
 const unsigned owner=static_cast<unsigned>(c.owner);
 if(owner>=3){a.reason="INVALID_OWNER";return last_=a;}
 auto& replay=replay_[owner];
 // IDs include the browser instance; a page reload may restart sequence at 1.
 if(replay.sequence&&!strcmp(c.id,replay.id)){if(c.sequence==replay.sequence)return replay.ack;a.reason="COMMAND_ID_REUSED";return last_=a;}
 auto result=[&](const char* r){a.outcome=r?motion::CommandOutcome::REJECTED:motion::CommandOutcome::ACCEPTED;a.reason=r?r:"COMMAND_ACCEPTED";
   replay.sequence=c.sequence;memcpy(replay.id,c.id,sizeof(replay.id));replay.ack=a;last_=a;
   if(!r&&c.intent!=motion::Intent::STATUS&&c.intent!=motion::Intent::CAPABILITIES){activeAck_=a;active_=true;}return a;};
 const uint64_t us=esp_timer_get_time();
 if(c.intent==motion::Intent::STATUS||c.intent==motion::Intent::CAPABILITIES)return result(nullptr);
 if(c.intent==motion::Intent::BASE_VELOCITY&&c.vx==0&&c.vy==0&&c.yaw_rate==0){motion_->walkStop();return result(nullptr);}
 if(c.intent!=motion::Intent::STAND&&c.intent!=motion::Intent::BASE_VELOCITY)return result("COMMAND_NOT_IN_SIMPLE_V1");
 if(!ready_)return result(reason_);
 if(prepare_&&(why=prepare_(context_)))return result(why);
 if(c.intent==motion::Intent::STAND)return result(motion_->stand(c.target_height_m,us));
 if(c.vy!=0||c.yaw_rate!=0)return result("STANDARD_FORWARD_PRESETS_ONLY");
 for(unsigned i=0;i<3;++i)if(std::abs(c.vx-simple::kStrideM[i]/simple::kWalkPeriodS)<1e-10)return result(motion_->walk(i,us));
 return result("STANDARD_STRIDE_REQUIRED");
}
bool MotionService::ackJson(const motion::MotionAck& a,char* out,size_t cap) const {
 char id[49]{};for(unsigned i=0;i<48&&a.id[i];++i){char c=a.id[i];id[i]=((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='.'||c=='_'||c=='-')?c:'_';}
 const int n=snprintf(out,cap,"{\"version\":1,\"ack\":true,\"id\":\"%s\",\"sequence\":%lu,\"owner\":\"%s\",\"outcome\":\"%s\",\"reason\":\"%s\",\"executor_state\":\"%s\"}",id,(unsigned long)a.sequence,motion::toString(a.owner),motion::toString(a.outcome),a.reason,a.executor_state);
 return n>0&&static_cast<size_t>(n)<cap;
}
bool MotionService::capabilitiesJson(char* out,size_t cap) const {
 const int n=snprintf(out,cap,"{\"version\":1,\"motion_stack\":\"MATDOG_SIMPLE_V1\",\"ready\":%s,\"reason\":\"%s\",\"height\":{\"nominal_min_mm\":0,\"nominal_max_mm\":150,\"workspace_min_mm\":%.0f,\"workspace_max_mm\":150},\"walk\":{\"period_s\":%.2f,\"lift_m\":%.3f,\"strides_m\":[%.3f,%.3f,%.3f]},\"commands\":[\"STAND\",\"WALK_STANDARD\",\"WALK_STOP\",\"STOP\"],\"hardware_qualified\":{\"stand\":false,\"walk\":false,\"imu_compensation\":false}}",boolean(ready_),reason_,simple::kMinStandM*1000,simple::kWalkPeriodS,simple::kWalkLiftM,simple::kStrideM[0],simple::kStrideM[1],simple::kStrideM[2]);
 return n>0&&static_cast<size_t>(n)<cap;
}
bool MotionService::statusJson(char* out,size_t cap,uint32_t now) const {
 if(!motion_||!out||!cap)return false;
 const uint64_t us=esp_timer_get_time();const bool fresh=motion_->fresh(us);
 size_t used=0;bool ok=true;
 auto add=[&](const char* fmt,auto... args){if(!ok)return;const int n=snprintf(out+used,cap-used,fmt,args...);if(n<0||static_cast<size_t>(n)>=cap-used){ok=false;return;}used+=n;};
 add("{\"version\":1,\"now_ms\":%lu,\"state\":\"%s\",\"phase\":\"%s\",\"pose_region\":\"%s\",\"ready\":%s,\"reason\":\"%s\",\"readiness_reason\":\"%s\",\"stale\":%s,\"fault\":%s,\"safe_off_verified\":%s,\"safe_off_mask\":%u,\"torque_active\":%s,\"height_valid\":%s,\"height_source\":\"ENCODER_IMU_FLAT_GROUND_ESTIMATE\",\"requested_height_mm\":%.1f,\"height_estimate_mm\":%.1f,\"walk_cycles\":%.6f,\"imu_correction_limited\":%s,\"imu_valid\":%s,\"imu_age_ms\":%llu,\"period_us\":%lu,\"releases\":%lu,\"missed_releases\":%lu,\"max_gap_us\":%llu,\"active_command\":{\"outcome\":\"%s\",\"reason\":\"%s\"},\"joints\":[",
 (unsigned long)now,simple::name(motion_->state()),motion_->phase(),motion_->poseRegion(us),boolean(ready_),motion_->reason(),reason_,boolean(!fresh),boolean(motion_->state()==simple::State::FAULT),boolean(motion_->offVerified()),motion_->offMask(),boolean(motion_->energized()),boolean(motion_->heightValid()),motion_->requestedHeightM()*1000,motion_->heightM()*1000,motion_->walkCycles(),boolean(motion_->stabilizationLimited()),boolean(motion_->imuValid()),(unsigned long long)(us>=motion_->imuStampUs()?(us-motion_->imuStampUs())/1000:0),(unsigned long)simple::kPeriodUs,(unsigned long)motion_->releases(),(unsigned long)motion_->missed(),(unsigned long long)motion_->maxGapUs(),motion::toString(activeAck_.outcome),activeAck_.reason);
 // Raw torque/goal fields retain their dated control observation. SAFE_OFF is
 // separate, newer evidence from 13 independent post-stop torque reads.
 // Appended after joints below to keep the existing wire contract compatible.
 for(unsigned i=0;i<12;++i){const auto& f=motion_->feedback()[i];const auto* unit=actuator::slotServo(i);add("%s{\"slot\":%u,\"id\":%u,\"joint\":\"%s\",\"valid\":%s,\"raw\":%u,\"speed\":%d,\"load\":%d,\"current\":%d,\"temperature\":%d,\"voltage_dv\":%d,\"status\":%d,\"goal\":%d,\"torque\":%d,\"age_ms\":%llu,\"control_age_ms\":%llu}",i?",":"",i,unit?unit->bus_id:0,unit?unit->joint:"UNKNOWN",boolean(f.valid),f.raw,f.speed,f.load,f.current,f.temp,f.voltage,f.status,f.goal,f.torque,(unsigned long long)(f.valid&&us>=f.stampUs?(us-f.stampUs)/1000:0),(unsigned long long)(f.controlStampUs&&us>=f.controlStampUs?(us-f.controlStampUs)/1000:0));}
 add("%s","]");
 add(",\"power_reason\":\"%s\",\"bms_age_ms\":%lu,\"max_release_work_us\":%lu,\"joint_torque_source\":\"DATED_CONTROL_READ\",\"safe_off_source\":\"POST_STOP_TORQUE_READ_13\",\"safe_off_verified_us\":%llu",
   motion_->powerReason(),(unsigned long)motion_->bmsAgeMs(),(unsigned long)motion_->maxReleaseWorkUs(),(unsigned long long)motion_->offVerifiedUs());
 if(motion_->referenceFaultSlot()<12){const auto& f=motion_->referenceFaultFeedback();
   add(",\"reference_guard\":{\"slot\":%u,\"raw\":%u,\"reference_raw\":%u,\"torque\":%d,\"goal\":%d,\"observed_us\":%llu}",
     motion_->referenceFaultSlot(),f.raw,motion_->referenceFaultRaw(),f.torque,f.goal,(unsigned long long)f.controlStampUs);}

 // Digital Twin contract already parsed by the existing viewer (telemetry v1):
 // the same 12 ACTUAL angles and BNO085 quaternion SimpleMotion holds, no new
 // read. sampled_at_ms dates the oldest joint sample; actual_rad when all 12
 // are fresh, otherwise null plus actual_rad_by_joint (null: never read).
 uint64_t oldest=us;bool all=fresh;
 for(unsigned i=0;i<12;++i){const auto& f=motion_->feedback()[i];if(f.valid&&f.stampUs<oldest)oldest=f.stampUs;all=all&&f.valid;}
 const uint64_t ageMs=(us-oldest)/1000;
 add(",\"telemetry\":{\"sampled_at_ms\":%lu,\"actual_rad\":",(unsigned long)(ageMs<now?now-ageMs:0));
 if(all){for(unsigned i=0;i<12;++i)add("%s%.6f",i?",":"[",motion_->feedback()[i].q);add("%s","]");}
 else{add("%s","null,\"actual_rad_by_joint\":[");
   for(unsigned i=0;i<12;++i){const auto& f=motion_->feedback()[i];if(f.valid)add("%s%.6f",i?",":"",f.q);else add("%snull",i?",":"");}
   add("%s","]");}
 add("%s",",\"quaternion\":");
 if(motion_->imuValid()){const double* q=motion_->quaternion();add("[%.6f,%.6f,%.6f,%.6f]",q[0],q[1],q[2],q[3]);}else add("%s","null");
 add("%s",",\"source\":\"CONTROLLER\"},\"stale_scope\":\"MOTION_JOINT_FEEDBACK\"}");
 return ok;
}
} }
