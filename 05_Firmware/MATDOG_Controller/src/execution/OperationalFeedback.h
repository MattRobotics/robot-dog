#ifndef MATDOG_EXECUTION_OPERATIONAL_FEEDBACK_H
#define MATDOG_EXECUTION_OPERATIONAL_FEEDBACK_H
#include "MotionExecutor.h"
namespace matdog { namespace execution {
struct RawJointObservation {
 bool read_ok=false;
 uint16_t position=0;
 int torque=-1,temperature=-1,voltage=-1,status=-1;
 uint32_t sampled_at_ms=0;
};
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
 JointFeedback feedback_{};
};
} }
#endif
