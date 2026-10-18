#include "OperationalFeedback.h"
#include "../servo/ServoPopulation.h"
#include <cstring>
namespace matdog { namespace execution {
void OperationalFeedback::ingest(unsigned i,const RawJointObservation& sample,const actuator::CalibrationGeometryProfile& geometry,
 const actuator::GeometryProvenance& provenance,const actuator::JointTransformTable& transforms){
 if(i>=12)return;
 raw_[i]=sample;feedback_.read_ok[i]=false;feedback_.torque_on[i]=sample.torque==1;
 feedback_.sampled_at_ms[i]=sample.sampled_at_ms;feedback_.actual_rad[i]=0;
 feedback_.sampled_at_us[i]=sample.sampled_at_us;feedback_.speed_rad_s[i]=0;
 if(!sample.read_ok||sample.position>=4096||sample.torque<0||sample.torque>1||sample.status!=0||
    sample.temperature<0||sample.temperature>70||sample.voltage<=0)return;
 const auto* servo=servo::legServoAt((i/3)*3+(2-i%3));if(!servo)return;
 calibration::JointIdentity identity;identity.leg=static_cast<calibration::Leg>(i/3);
 identity.joint=static_cast<calibration::JointKind>(i%3);
 std::strncpy(identity.physical_unit,servo->physical_unit,sizeof(identity.physical_unit)-1);
 const auto* t=transforms.find(identity,geometry.provenanceTag());
 const auto* g=geometry.findJoint(identity);
 if(!t||!g||g->bus_id!=servo->bus_id)return;
 actuator::MicroRad q=0;
 if(actuator::resolveRawToUrdfQ(geometry,provenance,*t,sample.position,&q)!=actuator::TargetResolveStatus::OK)return;
 feedback_.actual_rad[i]=q/1e6;feedback_.read_ok[i]=true;
 // Speed in the joint frame: q = sign * (raw - q0) scale, same direction as the position map.
 {actuator::MicroRad q1=0;const uint16_t next=static_cast<uint16_t>(sample.position<4095?sample.position+1:sample.position-1);
  if(actuator::resolveRawToUrdfQ(geometry,provenance,*t,next,&q1)==actuator::TargetResolveStatus::OK){
   const double per_tick=(q1-q)/1e6*(sample.position<4095?1:-1);feedback_.speed_rad_s[i]=sample.speed_signed*per_tick;}}
 last_valid_.read_ok[i]=true;last_valid_.sampled_at_ms[i]=feedback_.sampled_at_ms[i];
 last_valid_.sampled_at_us[i]=feedback_.sampled_at_us[i];last_valid_.actual_rad[i]=feedback_.actual_rad[i];
 last_valid_.torque_on[i]=feedback_.torque_on[i];last_valid_.speed_rad_s[i]=feedback_.speed_rad_s[i];
}
JointFeedback OperationalFeedback::snapshot(uint32_t now,uint32_t age) const {
 auto f=feedback_;
 for(unsigned i=0;i<12;++i){
  // Failed transport / malformed position: retry within the ORIGINAL age
  // limit, keeping the last valid timestamp and explicitly invalid feedback.
  // Valid physical alarms are never hidden by this recovery window.
  const auto& r=raw_[i];
  if((!r.read_ok||(r.position>=4096&&r.status==0&&r.temperature>=0&&r.temperature<=70&&r.voltage>0&&r.torque>=0&&r.torque<=1))&&last_valid_.read_ok[i]&&age&&
     static_cast<int32_t>(now-last_valid_.sampled_at_ms[i])>=0&&now-last_valid_.sampled_at_ms[i]<=age){
   f.recovering[i]=true;f.read_ok[i]=false;
   f.sampled_at_ms[i]=last_valid_.sampled_at_ms[i];f.sampled_at_us[i]=last_valid_.sampled_at_us[i];
   f.actual_rad[i]=last_valid_.actual_rad[i];f.torque_on[i]=last_valid_.torque_on[i];
  }
  if(!age||static_cast<int32_t>(now-f.sampled_at_ms[i])<0||now-f.sampled_at_ms[i]>age)f.read_ok[i]=false;
 }
 return f;
}
bool OperationalFeedback::rawFresh(uint32_t now,uint32_t age) const {
 if(!age)return false;
 for(const auto& r:raw_)if(!r.read_ok||r.position>=4096||static_cast<int32_t>(now-r.sampled_at_ms)<0||now-r.sampled_at_ms>age)return false;
 return true;
}
} }
