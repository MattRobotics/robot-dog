#include "OperationalFeedback.h"
#include "../motion/OperationalTolerance.h"
namespace matdog { namespace execution {
void OperationalFeedback::ingest(unsigned i,const RawJointObservation& sample,const actuator::CalibrationGeometryProfile& geometry,
 const actuator::GeometryProvenance& provenance,const actuator::JointTransformTable& transforms){
 if(i>=12)return;
 raw_[i]=sample;feedback_.read_ok[i]=false;feedback_.torque_on[i]=sample.torque==1;
 feedback_.sampled_at_ms[i]=sample.sampled_at_ms;feedback_.actual_rad[i]=0;
 feedback_.sampled_at_us[i]=sample.sampled_at_us;feedback_.speed_rad_s[i]=0;
 if(!sample.read_ok||sample.position>=4096||sample.torque<0||sample.torque>1||sample.status!=0||
    sample.temperature<0||sample.temperature>70||sample.voltage<=0)return;
 const auto* servo=actuator::slotServo(i);calibration::JointIdentity identity;
 if(!servo||!actuator::slotIdentity(i,&identity))return;
 const auto* t=transforms.find(identity,geometry.provenanceTag());
 const auto* g=geometry.findJoint(identity);
 if(!t||!g||g->bus_id!=servo->bus_id)return;
 // The canonical RAW -> radians conversion: the same radians a RAW limit end on this tick produces.
 double q=0;
 if(actuator::resolveRawToRadians(geometry,provenance,*t,sample.position,&q)!=actuator::TargetResolveStatus::OK)return;
 feedback_.actual_rad[i]=q;feedback_.read_ok[i]=true;
 // Speed in the joint frame: q = sign * (raw - q0) scale, same direction as the position map.
 // A register step/s is a velocity quantum, not the finite difference of
 // two rounded micro-radian positions (which alternates 1533/1534 urad).
 feedback_.speed_rad_s[i]=sample.speed_signed*actuator::jointDirection(geometry,identity)*motion::tolerance::kTickRad;
 // The previous valid sample travels with the latest one (common-instant reconstruction of the path box).
 if(last_valid_.read_ok[i]&&last_valid_.sampled_at_us[i]&&last_valid_.sampled_at_us[i]<feedback_.sampled_at_us[i]){
  previous_at_us_[i]=last_valid_.sampled_at_us[i];previous_rad_[i]=last_valid_.actual_rad[i];}
 last_valid_.read_ok[i]=true;last_valid_.sampled_at_ms[i]=feedback_.sampled_at_ms[i];
 last_valid_.sampled_at_us[i]=feedback_.sampled_at_us[i];last_valid_.actual_rad[i]=feedback_.actual_rad[i];
 last_valid_.torque_on[i]=feedback_.torque_on[i];last_valid_.speed_rad_s[i]=feedback_.speed_rad_s[i];
}
JointFeedback OperationalFeedback::snapshot(uint32_t now,uint32_t age) const {
 auto f=feedback_;
 for(unsigned i=0;i<12;++i){f.previous_at_us[i]=previous_at_us_[i];f.previous_rad[i]=previous_rad_[i];}
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
