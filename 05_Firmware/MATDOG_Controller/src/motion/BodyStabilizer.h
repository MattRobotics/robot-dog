#ifndef MATDOG_MOTION_BODY_STABILIZER_H
#define MATDOG_MOTION_BODY_STABILIZER_H
#include "ImuAttitude.h"
namespace matdog { namespace motion {
// G5-A body roll/pitch stabilizer core. It produces a BODY ORIENTATION correction (roll, pitch) that the caller applies
// to the desired BodyPose; Cartesian foot targets and IK follow (TiltCompensation.h). It never touches joint angles.
//
// Law: integral action on the measured tilt, because the plant from commanded body tilt to measured body tilt is an
// identity (position-controlled legs) plus an external disturbance:
//     c[k+1] = clamp_rate_and_range( c[k] - ki * dt * deadband(m[k] - reference) )
// with m the measured tilt (ImuAttitude), c the commanded correction. There is NO default configuration and no gain is
// approved for hardware. The configuration is only accepted if its integral gain lies below the stability bound
// derived from the sampling interval and an explicitly stated loop delay (maxStableIntegralGain), which is a
// mathematical constraint, not a tuning value.
//
// Safety contract: the stabilizer is a bounded, rate-limited, fail-safe function of validated attitude. An invalid, stale
// or missing attitude never produces a new correction: the output ramps to zero at the configured rate limit, and after
// holdBeforeFaultS without a valid sample the stabilizer latches FAULT (output zero) until an explicit resetFault().
// It does not command actuators and contains no actuator limit: the actuator safety layer is a separate, later layer.
double maxStableIntegralGain(double dtS, unsigned delaySteps);
struct StabilizerConfig {
  double integralGainPerS = 0;   // ki, 1/s
  double maxCorrectionRad = 0;   // |roll| and |pitch| command bound
  double maxRateRadS = 0;        // command rate bound (also the ramp-down rate)
  double deadbandRad = 0;        // measured-tilt dead band (>= 0)
  double sampleDtS = 0;          // nominal step
  unsigned loopDelaySteps = 0;   // whole steps from command to measurement; MUST come from a measurement
  double holdBeforeFaultS = 0;   // time without a valid new sample before FAULT latches
  double rollReferenceRad = 0;   // measured tilt that corresponds to "level" (mounting offset; UNMEASURED unless calibrated)
  double pitchReferenceRad = 0;
};
bool validStabilizerConfig(const StabilizerConfig& c);
enum class StabilizerState : uint8_t { DISABLED, ACTIVE, RAMPING_DOWN, FAULT };
struct StabilizerOutput {
  double rollCorrectionRad = 0, pitchCorrectionRad = 0;
  StabilizerState state = StabilizerState::DISABLED;
  AttitudeStatus attitudeStatus = AttitudeStatus::OK;
  bool saturated = false;
};
class BodyStabilizer {
 public:
  bool configure(const StabilizerConfig& config);
  bool enable(double nowS);      // only from DISABLED (command zero); never from FAULT or while ramping
  void disable();                // the correction is removed at the bounded rate, then the state is DISABLED
  void resetFault();             // clears a latched FAULT once the command has ramped to zero; stays DISABLED until enable()
  // One control step. `status` is the monitor result for the latest snapshot; `attitude` is meaningful only for OK.
  // `isNewSample` gates the integrator: the same sample is never integrated twice.
  StabilizerOutput step(AttitudeStatus status, const Attitude& attitude, bool isNewSample, double nowS);
  const StabilizerConfig& config() const { return cfg_; }
  StabilizerState state() const { return state_; }
 private:
  StabilizerConfig cfg_{};
  StabilizerState state_ = StabilizerState::DISABLED;
  double roll_ = 0, pitch_ = 0, lastNowS_ = 0, lastValidS_ = 0;
  bool configured_ = false, haveNow_ = false, pendingDisable_ = false;
};
} }
#endif
