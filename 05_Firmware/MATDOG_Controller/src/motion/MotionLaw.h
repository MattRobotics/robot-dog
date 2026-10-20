#ifndef MATDOG_MOTION_MOTION_LAW_H
#define MATDOG_MOTION_MOTION_LAW_H
// Common Motion time law and dynamic-profile contract (Core Motion V5, 2026-10-20).
//
// One infrastructure for STAND, height transitions and WALK STANDARD:
//  * a jerk-limited ("S-curve") rest-to-rest law in closed form, evaluated at ANY
//    real release instant (position, velocity, acceleration, jerk), with a time
//    that is DERIVED from {v, a, j} and the path length - never chosen;
//  * a jerk-limited speed change (distance = (v0+v1)/2 * T, exact for the
//    symmetric profile) usable for junctions and boundary speeds;
//  * a DynamicProfile contract that ties reference velocity/acceleration to the
//    frame period, the per-frame step cap, the ST3215 GoalSpeed/ACC pair and the
//    release-jitter the executor can absorb. The *maximum safety envelopes* stay
//    in the profile; the instantaneous reference varies below them.
// Pure functions: no allocation, no platform dependency, host-testable. Nothing
// here changes a limit; callers pass the already-qualified envelope.
#include <stdint.h>

namespace matdog { namespace motion {

struct LawLimits { double vMax = 0, aMax = 0, jMax = 0; };   // path units/s, /s^2, /s^3 (all > 0)

struct SpeedChange { double timeS = 0, distance = 0, peakAccel = 0; bool valid = false; };
// Jerk-limited change of speed from v0 to v1 (>=0): triangular acceleration when
// |dv| <= a^2/j, trapezoidal otherwise. distance = (v0+v1)/2 * timeS.
SpeedChange speedChange(double v0, double v1, double aMax, double jMax);

struct LawSample { double s = 0, v = 0, a = 0, j = 0; };

// Rest-to-rest S-curve over `length` with seven phases (j+, a, j-, cruise, j-, a, j+).
class ScurveProfile {
 public:
  bool plan(double length, const LawLimits& limits);
  bool valid() const { return valid_; }
  double length() const { return length_; }
  double durationS() const { return duration_; }
  double peakVelocity() const { return vPeak_; }
  double peakAcceleration() const { return aPeak_; }
  // Clamps t into [0, duration]. s runs 0..length; v and a are exact derivatives.
  LawSample at(double t) const;
 private:
  bool valid_ = false;
  double length_ = 0, duration_ = 0, vPeak_ = 0, aPeak_ = 0, j_ = 0;
  double t1_ = 0, t2_ = 0, tv_ = 0;        // jerk phase, acceleration plateau, cruise
  double tEnd_[7]{}, sEnd_[7]{}, vEnd_[7]{}, aEnd_[7]{};
  double jerk_[7]{};
};

// Convenience: derived minimum time (0 if invalid).
double scurveMinTime(double length, const LawLimits& limits);
// Time a quintic smoothstep needs for the SAME peak velocity and acceleration
// (peaks 1.875*L/T and 5.7735*L/T^2): the arbitrary-shape baseline the S-curve replaces.
double quinticMinTime(double length, double vMax, double aMax);

// ---- DynamicProfile: coherence of reference, frame, period, drive pair and feedback ----
struct DynamicProfile {
  const char* name = "";
  double velocityRadS = 0, accelerationRadS2 = 0, jerkRadS3 = 0;   // reference envelope (jerk 0 = trapezoid, unlimited)
  double periodS = 0, maxStepRad = 0;                               // frame period and per-frame step cap
  uint16_t goalSpeed = 0; uint8_t acc = 0;                          // ST3215 RAM GoalSpeed (steps/s) and ACC (x100 steps/s^2)
};
struct DynamicAnalysis {
  bool valid = false; const char* reason = "";
  double stepAtNominalRad = 0;        // v * period
  double stepHeadroomMs = 0;          // (maxStep/v - period): lateness absorbed WITHOUT slowing the frame
  double accelSafeLatenessMs = 0;     // largest release lateness whose secant acceleration stays <= aMax (executor rule)
  double driveSpeedRadS = 0, driveAccelRadS2 = 0;   // register ceilings in rad/s, rad/s^2 (nominal SDK scales)
  double driveSpeedRatio = 0, driveAccelRatio = 0;  // ceiling / reference: must stay above 1
};
// Servo register scales used by the SDK/profile docs: 2*pi/4096 rad per step; ACC unit = 100 steps/s^2.
constexpr double kServoTickRad = 6.28318530717958647692 / 4096.0;
DynamicAnalysis analyzeDynamicProfile(const DynamicProfile& profile);
// Largest release lateness L (seconds) for which one late frame (interval period+L) followed by an on-time frame keeps
// |secant acceleration| <= aMax, with the generator advancing min(elapsed, maxStep/v) of path time per frame. Searched up
// to maxLatenessS; the result is the FIRST violating lateness (the secant acceleration is not monotone in the lateness).
double accelSafeLatenessS(double velocityRadS, double accelerationRadS2, double periodS, double maxStepRad, double maxLatenessS);
// Largest reference velocity v <= vCap whose lateness tolerance (above) is at least targetLatenessS (0 if none; vCap if it already is).
double velocityForLateness(double vCap, double accelerationRadS2, double periodS, double maxStepRad, double targetLatenessS);
// Stream rule (OperationalWalkProfile.h kStreamMaxGapS, MotionExecutor kTimedMaxFrameGapMs): a release gap above this is refused.
constexpr double kStreamMaxGapS = 0.100;

} }
#endif
