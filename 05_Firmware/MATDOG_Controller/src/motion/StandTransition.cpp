#include "StandTransition.h"
namespace matdog { namespace motion {
bool StandTransition::apply(MotionEvent event) {
  if(!lifecycle_.apply(event))return false;
  if(event==MotionEvent::STOP||event==MotionEvent::DISABLE||event==MotionEvent::FAULT) {
    sampler_.stop();acquisition_=AcquisitionStatus::UNKNOWN_POSE;trajectory_=TimedStatus::NOT_READY;
  }
  return true;
}
bool StandTransition::begin(const StartupObservation& observation,const TimingSpec& timing) {
  if(state()!=MotionState::IDLE)return false;
  const auto acquisition=evaluateStartup(observation);acquisition_=acquisition.status;
  if(acquisition_!=AcquisitionStatus::READY) {sampler_.stop();return false;}
  trajectory_=sampler_.initialize(canonicalStandDefinition(),timing);
  if(trajectory_!=TimedStatus::OK)return false;
  expectedSamples_=timing.intervals+1;
  return lifecycle_.beginVerifiedStand();
}
TimedStandSample StandTransition::next() {
  TimedStandSample result;
  if(state()!=MotionState::STAND_TRANSITION) {
    if(state()==MotionState::STAND)result.status=TimedStatus::COMPLETE;
    return result;
  }
  result=sampler_.next();trajectory_=result.status;
  if(result.status!=TimedStatus::OK||!result.geometry.target.valid) {
    lifecycle_.apply(MotionEvent::FAULT);sampler_.stop();return result;
  }
  if(sampler_.metrics().emitted==expectedSamples_ && result.progress==1.0)
    lifecycle_.completeVerifiedStand();
  return result;
}
} }
