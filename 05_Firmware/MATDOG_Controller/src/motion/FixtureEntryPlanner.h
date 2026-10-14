#ifndef MATDOG_MOTION_FIXTURE_ENTRY_PLANNER_H
#define MATDOG_MOTION_FIXTURE_ENTRY_PLANNER_H
#include "MotionFeedback.h"
namespace matdog { namespace motion {
// Pure joint-space entry from the MEASURED armed pose to the head of the
// fixture route. Stage 0 moves only the four LOWER joints (out of the folded
// stop region); stage 1 moves HIP and UPPER with LOWER held at the head value.
// Each moving joint follows the existing quintic time law, so every stage
// starts and ends at rest. This module grants nothing: it rejects a start that
// is outside the joint limits or outside the runtime-loaded entry certificate,
// and AdaptiveStand/Executor/Safe Actuator layer re-check every sample.
enum class EntryStatus : uint8_t { OK, INVALID_INPUT, LIMIT, CERTIFICATE_REQUIRED, START_OUTSIDE_CERTIFIED_REGION, GOAL_MISMATCH, RATE_POLICY };
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
  unsigned failedJoint=12;          // slot LF hip..LH lower, 12 = none
  double startDeviationRad=0;       // max |start - certificate start|
};
// minStageDurationS bounds the slowest observable stage; rates come from the
// feedback policy (velocity, acceleration, per-sample step at samplePeriodS).
FixtureEntryPlan planFixtureEntry(const JointTargetFrame& start,const JointTargetFrame& goal,
                                  const FeedbackPolicy& policy,const FixtureEntryCertificate& certificate,
                                  double minStageDurationS);
// u in [0,1] within one stage. Returns an invalid frame for a bad stage/u.
LocomotionFrame fixtureEntrySample(const FixtureEntryPlan& plan,unsigned stage,double u,const BodyPose& body);
} }
#endif
