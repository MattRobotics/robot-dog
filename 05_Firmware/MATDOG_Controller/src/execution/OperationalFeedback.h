#ifndef MATDOG_EXECUTION_OPERATIONAL_FEEDBACK_H
#define MATDOG_EXECUTION_OPERATIONAL_FEEDBACK_H
#include "MotionExecutor.h"
namespace matdog { namespace execution {
struct RawJointObservation {
 bool read_ok=false;
 uint16_t position=0;
 int torque=-1,temperature=-1,voltage=-1,status=-1,torque_limit=-1,goal_position=-1,speed=-1,goal_speed=-1,goal_acceleration=-1;
 uint32_t transaction_ms=0;
 uint32_t sampled_at_ms=0;
 uint64_t sampled_at_us=0;      // read completion on the monotonic clock
 int speed_signed=0;            // steps/s, sign from bit 15 (fast block)
 int load=-1,current=-1;        // kept from the fast block (Issue #52)
 uint32_t control_at_ms=0;      // when the slow 40..49 block was last read
};
// A retry must be able to renew an expired control block even when the
// round-robin control cursor points elsewhere. Otherwise the fast read is
// discarded forever at this slot and all remaining encoders starve.
inline bool controlReadDue(bool fast,unsigned slot,unsigned next,uint32_t now,
                           uint32_t last_control,uint32_t period,
                           uint32_t sampled_control,uint32_t max_age){
 return !fast||(slot==next&&now-last_control>=period)||now-sampled_control>=max_age;
}
// A successful fast batch services three distinct slots before planner work.
// Do not truncate this quota with a 2 ms wall slice: a valid synchronous read
// can consume that slice by itself. An error ends the batch for supervision.
inline bool operationalPollReadAllowed(bool fast,unsigned completed,bool previous_ok){
 return previous_ok&&completed<(fast?3u:1u);
}
// Pure read producer boundary. Failure replaces the slot; never replay a last
// successful value as fresh. No q0 constants, target substitution or support.
class OperationalFeedback {
 public:
 void ingest(unsigned slot,const RawJointObservation&,const actuator::CalibrationGeometryProfile&,
             const actuator::GeometryProvenance&,const actuator::JointTransformTable&);
 JointFeedback snapshot(uint32_t now_ms,uint32_t max_age_ms) const;
 bool rawFresh(uint32_t now_ms,uint32_t max_age_ms) const;
 const RawJointObservation& raw(unsigned slot) const {return raw_[slot<12?slot:0];}
 private:
 RawJointObservation raw_[12]{};
 JointFeedback last_valid_{};
 JointFeedback feedback_{};
};
} }
#endif
