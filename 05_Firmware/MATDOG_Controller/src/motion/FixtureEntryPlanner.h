#ifndef MATDOG_MOTION_FIXTURE_ENTRY_PLANNER_H
#define MATDOG_MOTION_FIXTURE_ENTRY_PLANNER_H
#include "MotionFeedback.h"
#include "StartPoseArchitecture.h"
#include "MotionLaw.h"
namespace matdog { namespace motion {
// Pure joint-space entry from the MEASURED armed pose to the head of the
// fixture route. Stage 0 moves only the four LOWER joints (out of the folded
// stop region); stage 1 moves HIP and UPPER with LOWER held at the head value.
// Each moving joint follows the existing quintic time law, so every stage
// starts and ends at rest. This module grants nothing: it rejects a start that
// is outside the joint limits or outside the runtime-loaded entry certificate,
// and AdaptiveStand/Executor/Safe Actuator layer re-check every sample.
enum class EntryStatus : uint8_t { OK, INVALID_INPUT, LIMIT, CERTIFICATE_REQUIRED, START_OUTSIDE_CERTIFIED_REGION, GOAL_MISMATCH, RATE_POLICY,
  // Measured-start planner (2026-10-14): actionable path refusals.
  PATH_START_OUTSIDE_LIMITS, PATH_START_IN_KEEPOUT, PATH_LIMIT, PATH_KEEPOUT };
const char* entryStatusName(EntryStatus status);
// Offline certificate from a MEASURED start to the route head, with a joint
// tube. DISENGAGE must cover the Cartesian product of the four LOWER ranges,
// including independent inward PRIME offsets; ALIGN covers its straight stage.
// Encoder/install accuracy and modified cover compatibility need external
// measurement bounds. RAM-only; the digest is
// the certificate file, recorded for traceability, not an approval by itself.
struct FixtureEntryCertificate {
  bool present=false;
  JointTargetFrame start{}, goal{};
  double tubeRad=0;
  char sha256[65]{};
};
constexpr unsigned kFixtureEntryStages=2;
struct FixtureEntryPlan {
  EntryStatus status=EntryStatus::INVALID_INPUT;
  JointTargetFrame start{}, middle{}, goal{};
  double durationS[kFixtureEntryStages]{};
  // Measured-start entry (Core Motion V5): the stage clock is the common S-curve law over the stage progress
  // [0,1] (planMeasuredEntry); planFixtureEntry keeps the certified quintic. When durationS exceeds the law's own
  // time (whole-sample rounding, floor) the law is dilated uniformly (v/k, a/k^2): never a dwell.
  // Only the progress limits are stored (the plan is closed form and re-evaluated per sample): keeps the plan small on the
  // 16 KB Controller stack (a stored profile added ~700 B to every copy of the plan).
  bool lawValid[kFixtureEntryStages]{};
  double lawV[kFixtureEntryStages]{}, lawA[kFixtureEntryStages]{}, lawJ[kFixtureEntryStages]{};   // progress limits per stage
  unsigned failedJoint=12;          // slot LF hip..LH lower, 12 = none
  double startDeviationRad=0;       // max |start - certificate start|
  // Measured-start planner diagnostics (which sample/keep-out, how much room).
  unsigned failedSample=0;          // index in the sampled two-stage path
  int failedKeepout=-1;
  double clearanceRad=0, requiredClearanceRad=0;
};
// Measured-start entry (operator directive 2026-10-14). Plans from the HELD
// measured pose to the route head; no certificate tube, no nominal-start
// requirement. The specific sampled path must keep tracking + margin from the
// hard limits and every keep-out (a start already close to a stop may only
// move away from it). Recognition (classifier) is a separate, earlier layer;
// tracking is checked while executing. Rates/durations as planFixtureEntry.
struct MeasuredEntryConfig {
  const startpose::JointKeepOut* keepouts=nullptr;
  unsigned keepoutCount=0;
  double pathMarginRad=.001;
};
FixtureEntryPlan planMeasuredEntry(const JointTargetFrame& start,const JointTargetFrame& goal,
                                   const FeedbackPolicy& policy,const MeasuredEntryConfig& config,
                                   double minStageDurationS);
// minStageDurationS bounds the slowest observable stage; rates come from the
// feedback policy (velocity, acceleration, per-sample step at samplePeriodS).
FixtureEntryPlan planFixtureEntry(const JointTargetFrame& start,const JointTargetFrame& goal,
                                  const FeedbackPolicy& policy,const FixtureEntryCertificate& certificate,
                                  double minStageDurationS);
// u in [0,1] within one stage. Returns an invalid frame for a bad stage/u.
LocomotionFrame fixtureEntrySample(const FixtureEntryPlan& plan,unsigned stage,double u,const BodyPose& body);
// The same twelve targets (slot order) without the display feet: what the release retimer searches over.
bool fixtureEntryJoints(const FixtureEntryPlan& plan,unsigned stage,double u,double q[12]);
} }
#endif
