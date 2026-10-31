#pragma once
#include "DalyProtocol.h"

namespace matdog { namespace power {
// The existing 3 s Motion deadline remains a STOP boundary. The 5 s DALY
// contract distinguishes a missed renewal from persistent monitoring loss;
// it does NOT grant unmonitored motion or automatic recovery.
enum class MotionPower : uint8_t { OK, ALARM, STATE_NOT_RUN, NEVER_SAMPLED, MOS_OFF, STALE, COMM_LOST };
inline MotionPower motionPower(const DalySample& s,uint32_t now,bool running) {
  if(s.valid)for(auto alarm:s.alarms)if(alarm)return MotionPower::ALARM;
  if(!running)return MotionPower::STATE_NOT_RUN;
  if(!s.valid)return MotionPower::NEVER_SAMPLED;
  if(!s.discharge_mos_on)return MotionPower::MOS_OFF;
  const uint32_t age=now-s.sampled_at_ms;
  if(age>kDalyTelemetryFreshnessMs)return MotionPower::COMM_LOST;
  if(age>3000)return MotionPower::STALE;
  return MotionPower::OK;
}
inline const char* name(MotionPower p) {
  switch(p){
    case MotionPower::OK:return "READY";
    case MotionPower::ALARM:return "BMS_ALARM";
    case MotionPower::STATE_NOT_RUN:return "POWER_STATE_NOT_RUN";
    case MotionPower::NEVER_SAMPLED:return "BMS_SAMPLE_UNAVAILABLE";
    case MotionPower::MOS_OFF:return "BMS_DISCHARGE_MOS_OFF";
    case MotionPower::STALE:return "BMS_TELEMETRY_STALE";
    case MotionPower::COMM_LOST:return "BMS_COMMUNICATION_LOST";
  }
  return "BMS_SAMPLE_UNAVAILABLE";
}
} }
