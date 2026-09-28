#ifndef MATDOG_MOTION_TIMED_STAND_H
#define MATDOG_MOTION_TIMED_STAND_H
#include "BodyPose.h"
#include "StandTrajectory.h"
namespace matdog { namespace motion {
struct TimingSpec { double durationS = 0.0; uint32_t intervals = 0; };
bool validTiming(const TimingSpec& spec);
TimingSpec timingFromPeriod(double periodS, uint32_t intervals);
struct TimeLawSample { bool valid=false; double progress=0, velocity=0, acceleration=0; };
// u=t/T in [0,1]. Derivatives are ds/dt and d2s/dt2, not joint rates.
TimeLawSample quinticTimeLaw(double u, double durationS);
enum class TrajectoryPhase : uint8_t { ACQUISITION_HOLD, CONTACT_LOCKED_RISE };
enum class TimedStatus : uint8_t { OK, INVALID_TIMING, PATH_FAILURE, SINGULAR_DERIVATIVE, BRANCH_CHANGE, NOT_READY, COMPLETE };
struct TimedStandSample {
  TimedStatus status=TimedStatus::NOT_READY;
  StandSample geometry{};  // contains the existing semantic 12-joint frame
  BodyPose body{};
  double timeS=0.0, normalizedTime=0.0, progress=0.0;
  TrajectoryPhase phase=TrajectoryPhase::ACQUISITION_HOLD;
  LegJointAngles velocityRadS[4]{}, accelerationRadS2[4]{};
};
struct TimedStandMetrics {
  LegJointAngles peakVelocityRadS[4]{}, peakAccelerationRadS2[4]{};
  double durationS=0.0, minJointLimitMarginRad=0.0, maxContactResidualM=0.0, maxContactDriftM=0.0;
  uint32_t emitted=0, branchChanges[4]{};
};
// Stateless evaluator: separates normalized geometric path from the clock law.
// Returns analytic joint derivatives from the contact Jacobian and directional
// Hessian. Singular/ill-conditioned linear solves fail; no fabricated rates.
TimedStandSample evaluateTimedStand(const StandDefinition& definition, double normalizedTime,
                                   double durationS, const LegJointAngles seeds[4]);
// Pure offline sampler, not startup authorization. The lifecycle wrapper owns
// startup gating. Peaks are sampled maxima, not certified continuous bounds.
class TimedStand {
 public:
  TimedStatus initialize(const StandDefinition& definition, const TimingSpec& timing);
  TimedStandSample next();
  void stop() { ready_=false; }
  const TimedStandMetrics& metrics() const { return metrics_; }
 private:
  StandDefinition definition_{};
  TimingSpec timing_{};
  StandSample previous_{};
  TimedStandMetrics metrics_{};
  Vector3 firstContacts_[4]{};
  bool ready_=false;
};
} }
#endif
