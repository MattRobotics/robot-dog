#ifndef MATDOG_MOTION_STAND_TRANSITION_H
#define MATDOG_MOTION_STAND_TRANSITION_H
#include "MotionState.h"
#include "StartupAcquisition.h"
#include "TimedStand.h"
namespace matdog { namespace motion {
// Single pure owner of startup evidence -> full trajectory -> completion.
class StandTransition {
 public:
  MotionState state() const { return lifecycle_.state(); }
  bool apply(MotionEvent event);
  bool begin(const StartupObservation& observation,const TimingSpec& timing);
  TimedStandSample next();
  AcquisitionStatus acquisitionStatus() const { return acquisition_; }
  TimedStatus trajectoryStatus() const { return trajectory_; }
  const TimedStandMetrics& metrics() const { return sampler_.metrics(); }
 private:
  MotionStateMachine lifecycle_{};
  TimedStand sampler_{};
  AcquisitionStatus acquisition_=AcquisitionStatus::UNKNOWN_POSE;
  TimedStatus trajectory_=TimedStatus::NOT_READY;
  uint32_t expectedSamples_=0;
};
} }
#endif
