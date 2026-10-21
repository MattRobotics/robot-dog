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

// Acceleration ramp time that sets the jerk limit of derived laws (j = a / ramp): 5 frames at 20 ms.
constexpr double kLawAccelRampS = 0.1;
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
// Largest release lateness L (seconds) a cruise at v can absorb: one late frame (interval period+L) must advance at most
// maxStep, so its secant drops to min(v, maxStep/(period+L)); that deceleration must stay <= aMax over the mean
// interval. The return to speed afterwards is ramped by retimeRelease and never adds a violation. Beyond L no target on
// the path satisfies both the step and the acceleration bound (a true infeasibility of the profile, refused before TX).
// Feasibility uses the exact step cap; search resolution never changes this contract.
// Searched up to maxLatenessS; the result is the FIRST violating lateness (the deceleration is not monotone in L).
double accelSafeLatenessS(double velocityRadS, double accelerationRadS2, double periodS, double maxStepRad, double maxLatenessS);
// Largest reference velocity v <= vCap whose lateness tolerance (above) is at least targetLatenessS (0 if none; vCap if it already is).
double velocityForLateness(double vCap, double accelerationRadS2, double periodS, double maxStepRad, double targetLatenessS);
// Full searches run by velocityForLateness (a repeated identical request returns the kept result). Host tests use it
// to prove that no pre-arm / STAND / height path repeats the ~1.1 s ESP32-S3 search after configuration.
unsigned long velocityForLatenessSearches();
// Stream rule (OperationalWalkProfile.h kStreamMaxGapS, MotionExecutor kTimedMaxFrameGapMs): a release gap above this is refused.
constexpr double kStreamMaxGapS = 0.100;
// One progression contract for every streamed STAND phase (Core Motion V6): the release lateness a cruise absorbs
// without an infeasible frame - loop granularity plus one short service stall (the V5 height-transition decision,
// now shared by the entry stages, the paired route and the height transitions). Each law's cruise velocity is DERIVED
// from it through velocityForLateness(); envelopes, step cap and executor limits are unchanged.
constexpr double kStreamLatenessToleranceS = 0.012;

// ---- Canonical time units of Motion (Core Motion V6) ----------------------------------------------------------------
// Every instant is a uint64 microsecond count of the Controller's MonotonicClock64. Seconds are DERIVED from it by one
// correctly rounded conversion, for instants and intervals alike; a duration configured in seconds becomes
// microseconds by one rounding. No module converts a clock reading by its own formula.
inline double secondsFromMicros(uint64_t us) { return static_cast<double>(us) / 1e6; }
uint64_t microsFromSeconds(double s);   // nearest microsecond; 0 for a non-positive or non-finite duration

// ---- Release-secant contract (Core Motion V6) ----------------------------------------------------------------------
// A streamed frame is a 12-joint target at its REFERENCE instant: the release instant the generator materialized it
// for, kept unchanged until the frame is transmitted or discarded (never re-stamped). Two consecutive transmitted
// frames define a secant velocity, two secants an acceleration over the mean of their intervals. For any reference
// with |q''| <= A that secant acceleration is <= A whatever the sampling, so the rule is exact, not a heuristic.
// The executor owns the committed state (the last TRANSMITTED frame, updated only after a successful TX); the
// generator's retimer reads that same state and applies this same function, so a frame the generator emits is a frame
// the executor admits bit for bit. The executor still re-checks every frame, independently, before the first byte.
struct ReleaseState { const double* q = nullptr; const double* v = nullptr; uint64_t atUs = 0, intervalUs = 0; };
struct ReleaseEnvelope { double velocityRadS = 0, accelerationRadS2 = 0, stepRad = 0; };   // stepRad 0: no step bound
enum class ReleaseVerdict : uint8_t { OK, CLOCK, STEP, VELOCITY, ACCELERATION };
const char* releaseVerdictName(ReleaseVerdict verdict);
struct ReleaseCheck {
  ReleaseVerdict verdict = ReleaseVerdict::CLOCK;
  unsigned joint = 12;                                     // first violating slot (joint order), 12 = none
  double q = 0, qLast = 0, v = 0, vPrev = 0, dtS = 0, dtPrevS = 0, value = 0, limit = 0;   // its operands
  // Over ALL joints: a bound a larger progress would worsen (step, velocity, acceleration in the direction of motion)
  // or one only a larger progress could satisfy (deceleration beyond the envelope).
  bool tooFast = false, tooSlow = false;
};
ReleaseCheck checkRelease(const ReleaseState& last, const double q[12], uint64_t atUs, const ReleaseEnvelope& envelope);

// Real time -> path progress (Core Motion V6; replaces the isolated min(elapsed, step/vmax) clamp). Returns the largest
// progress tau in [tauLast, tauMax] whose target satisfies checkRelease against the committed state, where
// tauMax = min(phase end, tauLast + real time elapsed since the committed frame): the reference never runs ahead of
// real time and never catches up in a burst. On time this is tauMax (one evaluation); a late release, a polyline
// corner or the return to speed after a slowdown is bounded by the same step/velocity/acceleration rule as the
// executor. No admissible progress means the release came later than the profile can decelerate within its step:
// `check` then carries the binding operands. `sample` evaluates the SAME path the frame is built from.
using ProgressSample = bool (*)(const void* context, double tau, double q[12]);
struct RetimeResult { bool found = false, sampleFailed = false; double tau = 0; ReleaseCheck check{}; unsigned evaluations = 0; };
constexpr unsigned kRetimeIterations = 14;   // normal search; an empty result is refined to floating-point precision
RetimeResult retimeRelease(const ReleaseState& last, uint64_t atUs, const ReleaseEnvelope& envelope,
                           double tauLast, double tauMax, ProgressSample sample, const void* context);

} }
#endif
