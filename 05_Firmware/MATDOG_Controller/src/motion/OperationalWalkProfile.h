#ifndef MATDOG_MOTION_OPERATIONAL_WALK_PROFILE_H
#define MATDOG_MOTION_OPERATIONAL_WALK_PROFILE_H
// Continuous supervised WALK on the fixture (operator directive 2026-10-15).
//
// G4 WALK gait (FeedbackWalk, point-foot IK), body held by the fixture at the
// canonical 0.15 m planning height. Crawl order LF, LH, RF, RH (offsets 0,
// .25, .5, .75), duty 0.80: one leg in swing at a time with a short
// four-foot overlap, so the coordinated stop lands on an all-foot phase. Swing: quintic
// smoothstep advance, lift 64*H*u^3(1-u)^3 (peak H, zero velocity and
// acceleration at lift-off and touch-down). Smooth ramp-in over one period,
// coordinated stop over two periods back to the all-foot STAND.
//
// Only the strides below are offered; every one was solved offline with the
// production IK against the REAL gen-1 limits and swept against the canonical
// meshes (09_Logs/Validation_Reports/Preflight_Review_2026-10-15/). Joint rate
// envelope = the largest peak of the presets + margin; the servo drive profile
// is a register ceiling ~1.5x above it. Tracking tolerance is NOT changed.
#include <cmath>
#include "FeedbackWalk.h"

namespace matdog { namespace motion { namespace operational_walk {

constexpr double kPeriodS = 7.0;
constexpr double kRampS = 7.0;          // smooth ramp-in, one full cycle
constexpr double kLiftM = 0.015;
constexpr double kDuty = 0.80;          // > .75: all four feet down at every integer cycle
constexpr double kStrideM[3] = {0.020, 0.030, 0.040};   // advance per cycle
constexpr unsigned kStrideCount = 3;
constexpr double kMaxVelocityRadS = 1.0;      // presets peak 0.84 rad/s
constexpr double kMaxAccelerationRadS2 = 4.5; // presets peak 3.69 rad/s^2
constexpr double kMaxJointStepRad = 0.05;     // 1.0 rad/s x 50 ms
constexpr double kCommandTtlS = 0.25;         // internal, renewed by heartbeat
// ST3215 register ceilings while walking: GOAL_SPEED steps/s (1000 = 1.53
// rad/s, ~1/3 of the rated 0.222 s/60 deg), ACC x100 steps/s^2 (50 = 7.7 rad/s^2).
constexpr uint16_t kDriveSpeed = 1000;
constexpr uint8_t kDriveAcceleration = 50;

// Streaming timing (Issue #52, 2026-10-15). Offline hypotheses, NOT hardware
// measurements: 50 Hz frames (SYNC_WRITE 104 B = 1.04 ms at 1 Mbaud); a gap
// above 5 periods ends the walk instead of planning through it (inside the gap
// the chord between two in-box targets stays in the convex certified box and
// its rate is checked over the real interval); the measured
// joint vector must be younger than 120 ms and each joint sample younger than
// 100 ms; a joint further than the 6 mrad tube from every goal sent in the last
// 150 ms (servo lag tau + feedback age) for 3 consecutive samples has failed
// to follow. Settled = |speed| <= 1 register step (1.53 mrad/s) at the target.
extern const double kTargetMinRad[12];
extern const double kTargetMaxRad[12];
constexpr double kStreamPeriodS = 0.020, kStreamMaxGapS = 0.100, kStreamObservationAgeS = 0.120, kStreamFeedbackAgeS = 0.100;
constexpr double kStreamMaxLagS = 0.150, kStreamSettleSpeedRadS = 2 * 3.14159265358979323846 / 4096, kStreamHoldTimeoutS = 3.0;
constexpr unsigned kStreamPersistSamples = 3;

// ---- Runtime walk profiles (Issue #52, 2026-10-15) -------------------------
// ONE table: gait geometry, planner joint-rate envelope, the ST3215 RAM drive
// pair written while walking (GoalSpeed SRAM 46/47 steps/s, ACC SRAM 41 x100
// steps/s^2 - register commands to the servo's own profile generator, NOT
// measured physical rates), and the certified per-joint target box. EEPROM,
// PID and protections are never touched; the canonical C018 profile stays.
//   STANDARD: the reviewed presets above (20/30/40 mm, 7 s, lift 15, duty .8).
//   FAST    : EXPERIMENTAL, never a default, NOT validated on hardware:
//             60 mm / 3 s / lift 10 mm / duty .75 = 20 mm/s equivalent on the
//             fixture (sweep: peak 1.310 rad/s, 10.69 rad/s^2, jerk 267 rad/s^3).
//             Drive 1200/100 = 1.84 rad/s, 15.3 rad/s^2 register ceilings
//             (x1.40 over the reference); planner envelope 1.55 / 13.0.
struct DriveProfile { uint16_t goalSpeed; uint8_t acceleration; };
struct WalkProfile {
  const char* name; bool experimental;
  double periodS, rampS, liftM, duty;
  const double* strides; unsigned strideCount;
  double maxVelocityRadS, maxAccelerationRadS2, maxJointStepRad;
  DriveProfile drive;
  const double* regionMinRad; const double* regionMaxRad;
};
constexpr double kFastStrideM[1] = {0.060};
constexpr DriveProfile kStandardDrive{kDriveSpeed, kDriveAcceleration};
constexpr DriveProfile kFastDrive{1200, 100};
extern const double kFastTargetMinRad[12];
extern const double kFastTargetMaxRad[12];
extern const WalkProfile kStandardProfile;
extern const WalkProfile kFastProfile;
constexpr unsigned kProfileCount = 2;
extern const WalkProfile* const kProfiles[kProfileCount];
// ---- Operational STAND profile (Issue #52, 2026-10-15) ----------------------
// The certified paired route GEOMETRY is unchanged; its time law is the
// junction-limited look-ahead profile (motion/PhaseTimedRoute.h). Every frame
// is still tracked before the next (6 mrad tube) - the route certificate's
// assumption - so a frame step must not exceed the tube:
//   v = 0.29 rad/s, dt = 20 ms  ->  5.8 mrad per frame  (< tube 6 mrad, max_step 6 mrad)
//   envelope a = 2.0 rad/s^2 (commissioning cap), planner a = a/2 so that a
//   polyline-corner jump (<= a_planner*dt) plus segment acceleration stays
//   inside the executor's finite-difference check.
// Drive pair (RAM, STAND frames): GoalSpeed 300 steps/s = 0.46 rad/s (x1.6 over v),
// ACC 20 = 3.07 rad/s^2 (x1.5 over the envelope). Entry stages derive their
// durations from the same rates (minimum stage 1 s, the compiled floor).
// Offline numbers; NOT measured on hardware.
struct StandProfile { double velocityRadS, accelerationRadS2, samplePeriodS, maxStepRad, minEntryStageS; DriveProfile drive; };
constexpr StandProfile kStandProfile{0.29, 2.0, 0.020, 0.006, 1.0, {300, 20}};
static_assert(kStandProfile.velocityRadS * kStandProfile.samplePeriodS <= kStandProfile.maxStepRad, "STAND frame step must stay inside the tube");

// Executor/planner ceilings must cover every profile (checked in MotionCoordinator.cpp).
constexpr double kMaxProfileVelocityRadS = 1.55, kMaxProfileAccelerationRadS2 = 13.0;

inline double strideVelocity(unsigned i) { return i < kStrideCount ? kStrideM[i] / kPeriodS : 0; }
inline double profileVelocity(const WalkProfile& p, unsigned i) { return i < p.strideCount ? p.strides[i] / p.periodS : 0; }
// The profile whose preset velocity is exactly vx, or nullptr (never rounded).
const WalkProfile* profileForVelocity(double vx);
inline bool isPresetVelocity(double vx) { return profileForVelocity(vx) != nullptr; }
WalkSpecification specification(const WalkProfile& profile);
inline WalkSpecification specification() { return specification(kStandardProfile); }
// Live gen-1 limits must contain the profile box +/- the tracking tube.
bool fitsLimits(const WalkProfile& profile, const FeedbackPolicy& policy, double tube_rad, unsigned* failed_joint = nullptr);

// Union of every commanded joint target of the three presets (ramp, steady
// cycles, coordinated stop), radians, slot leg*3+{HIP,UPPER,LOWER}; produced
// by scripts/tests/test_operational_motion.cpp (OPERATIONAL_WALK_RANGES) and
// pinned there. Admission adds the tracking tube and requires the live gen-1
// limits to contain it.
extern const double kTargetMinRad[12];
extern const double kTargetMaxRad[12];
inline bool fitsLimits(const FeedbackPolicy& policy, double tube_rad, unsigned* failed_joint = nullptr) { return fitsLimits(kStandardProfile, policy, tube_rad, failed_joint); }

} } }
#endif
