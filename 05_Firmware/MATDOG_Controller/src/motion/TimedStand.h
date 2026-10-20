#ifndef MATDOG_MOTION_TIMED_STAND_H
#define MATDOG_MOTION_TIMED_STAND_H
#include "BodyPose.h"
#include "StandTrajectory.h"
#include "MotionLaw.h"
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
// Joint-space dynamics of the fixed-contact path per unit of its progress coordinate p in [0,1]:
// qs = dq/dp, qss = d2q/dp2 (analytic contact Jacobian/Hessian, same solver as evaluateTimedStand).
// Peaks are maxima over `samples`+1 progress samples (dense numerical survey, not a continuous
// bound). They turn a joint-space envelope {v,a} into progress-space limits for a time law:
//   p' <= v / qsMax,  p'' <= (a - qssMax*p'^2) / qsMax.
struct StandPathDynamics { bool valid=false; double displacementInfRad=0, qsMaxRadPerProgress=0, qssMaxRadPerProgress=0; };
StandPathDynamics standPathDynamics(const StandDefinition& definition, const LegJointAngles seeds[4], unsigned samples);
// Same fixed-contact geometry as evaluateTimedStand, but the clock is supplied by the caller
// (progress p, its rate p' and acceleration p''), e.g. from an S-curve: joint velocity = qs*p',
// joint acceleration = qss*p'^2 + qs*p''.
TimedStandSample evaluateStandWithLaw(const StandDefinition& definition, const TimeLawSample& law, const LegJointAngles seeds[4]);
// Height-transition law (Core Motion V5): the joint envelope {v, a} and an acceleration ramp time become progress-space
// limits through the measured path dynamics above, and an S-curve over progress [0,1] gives the time. The duration is
// DERIVED from the envelope and the real IK derivatives, never chosen; the geometry is unchanged.
struct StandHeightLaw {
  bool valid=false;
  ScurveProfile profile;               // progress 0..1 over time
  double durationS=0, progressVMax=0, progressAMax=0, progressJMax=0;
  double jointVRadS=0, jointARadS2=0;  // the envelope it was planned for
  StandPathDynamics path;
  // Clock at real time t in [0,durationS]. When the planned duration was stretched to a floor (durationS > profile), the
  // S-curve is dilated uniformly (v/k, a/k^2): no dwell, no discontinuity, every limit only gets safer.
  TimeLawSample at(double t) const {
    TimeLawSample r;
    if(!valid||durationS<=0)return r;
    const double k=profile.durationS()/durationS;
    const auto q=profile.at(t*k);
    r.valid=true;r.progress=q.s<0?0:q.s>1?1:q.s;r.velocity=q.v*k;r.acceleration=q.a*k*k;return r;
  }
};
bool planStandHeightLaw(const StandDefinition& definition, const LegJointAngles seeds[4], double jointVRadS, double jointARadS2,
                        double accelRampS, double minDurationS, StandHeightLaw* out);
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
