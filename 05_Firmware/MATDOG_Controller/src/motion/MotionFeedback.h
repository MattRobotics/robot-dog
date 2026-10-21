#ifndef MATDOG_MOTION_FEEDBACK_H
#define MATDOG_MOTION_FEEDBACK_H
#include "Gait.h"
namespace matdog { namespace motion {
// This pure boundary contains SI observations and evidence, never actuator authority.
enum class EvidenceSource : uint8_t { UNKNOWN, SYNTHETIC, EXTERNAL_MEASURED, OFFLINE_CERTIFIED };
enum class FeedbackMode : uint8_t { SIMULATION, PHYSICAL_REVIEW, FIXTURE_REVIEW };
enum class FeedbackReason : uint8_t {
  OK, NOT_REQUESTED, INVALID_POLICY, INVALID_COMMAND, STALE_OBSERVATION,
  UNOBSERVABLE_HEIGHT, UNOBSERVABLE_SUPPORT, INVALID_POSE, UNRECOVERABLE_POSE,
  ROUTE_REQUIRED, ROUTE_MISMATCH, IK_FAILURE, BRANCH_CHANGE, JOINT_LIMIT,
  CONTINUITY, VELOCITY_LIMIT, ACCELERATION_LIMIT, ASSESSMENT_REQUIRED,
  ASSESSMENT_STALE, COLLISION, SUPPORT_INVALID, CONTINUOUS_BOUNDS_MISSING,
  FEEDBACK_PENDING, TRACKING_ERROR, SEGMENT_TIMEOUT, INHIBITED, FAULT,
  VERIFY_PENDING, COMMAND_EXPIRED, SEQUENCE_REPLAY, STOPPED, FOOTPRINT_MISMATCH, OUTSIDE_CERTIFIED_REGION
};
const char* feedbackReasonName(FeedbackReason reason);
struct FeedbackPolicy {
  FeedbackMode mode = FeedbackMode::PHYSICAL_REVIEW;
  JointLimit jointLimits[4][3]{}; // approved operational envelope, no inferred defaults
  LegJointAngles maxVelocityRadS[4]{}, maxAccelerationRadS2[4]{};
  double maxJointStepRad=0, maxTrackingErrorRad=0;
  double observationMaxAgeS=0, assessmentMaxAgeS=0, segmentTimeoutS=0;
  double heightToleranceM=0, contactToleranceM=0, samplePeriodS=0;
  bool envelopesApproved=false, loadedRatesVerified=false;
  bool fixtureTrialRatesReviewed=false; // bounded qualification trial, never loaded-hardware PASS
  bool routeTubeReviewed=false;
  bool fixtureLimitVariant=false; // named certified derivative; FIXTURE_REVIEW only
  unsigned routeFirstKnot=0,routeLastKnot=251;
  double routeJointTubeRad=0,routePositionTubeM=0,routeRotationTube=0;
};
struct AdaptiveObservation {
  JointTargetFrame actual{};
  BodyPose body{}; // world support plane Z=0; translation is base_link, NOT IMU height
  // stampS = the ENCODER vector: the oldest of its per-joint sample instants.
  // supportStampS = the support/presence declaration (heartbeat); it never
  // refreshes stampS (Issue #52: a heartbeat is not a new encoder reading).
  double stampS=0, supportStampS=0;
  EvidenceSource heightSource=EvidenceSource::UNKNOWN;
  EvidenceSource supportSource=EvidenceSource::UNKNOWN;
  uint8_t confirmedFeet=0;
  bool bellySupported=false, flatPlaneConfirmed=false, stationary=false;
  bool externalSupportConfirmed=false; // measured fixture support, never inferred from feet/IMU
  bool inhibited=false, fault=false;
  bool acknowledged=false;
  uint32_t acknowledgedSequence=0;
};
struct SegmentAssessment {
  uint32_t sequence=0;
  double stampS=0;
  EvidenceSource source=EvidenceSource::UNKNOWN;
  bool geometryIdentityVerified=false;
  bool sweptCollisionClear=false, supportCompatible=false;
  bool continuousBoundsConfirmed=false; // sampled kinematics alone is insufficient
  bool fixtureTrialEnvelopeReviewed=false;
  bool loadedTrackingQualified=false;
  bool dynamicStabilityQualified=false; // independent loaded-hardware WALK evidence
};
struct FeedbackFrame {
  JointTargetFrame target{};
  BodyPose body{};
  Vector3 feetWorldM[4]{};
  LegJointAngles velocityRadS[4]{}, accelerationRadS2[4]{};
  LegPhase phases[4]{};
  uint8_t requiredFeet=0;
  bool bellySupportRequired=false;
  bool externalSupportRequired=false;
  bool executable=false, physicalReadiness=false, dynamicStabilityProven=false;
  double timeS=0;
  FeedbackReason reason=FeedbackReason::NOT_REQUESTED;
};
// First-refusal operands (Core Motion V6 diagnostics): which verifier refused, at which joint slot (0..11, LF hip ..
// LH lower), the compared value and its bound; for release dynamics the secant operands; for the path box the
// reconstruction instant and uncertainty. Captured once, at the first refusal of an armed session, and printed once by
// the Controller's existing first-stop record: no streaming, no extra channel, no work on the frame path.
struct RefusalRecord {
  bool valid=false;
  const char* site="";       // e.g. EXECUTOR_PRE_TX, RETIMER, PLANNER_SEGMENT, PATH_BOX, MONITOR
  const char* reason="";
  int8_t joint=-1;
  uint32_t sequence=0;
  double value=0, limit=0;                            // compared quantity and its bound
  double q=0, qLast=0, v=0, vPrev=0, dtS=0, dtPrevS=0; // release secant operands (0 when not applicable)
  uint64_t atUs=0;                                     // reference instant, or the common instant of the path box
  uint64_t sampleUs=0; double uncertainty=0;           // path box: binding joint's sample instant and uncertainty
};
bool validFeedbackPolicy(const FeedbackPolicy& policy);
// Observation validity for the planner: freshness, evidence, support, body pose and the joint MODEL (FK). Measured
// joints are compared with the live hard limits in the RAW domain by the executor (ARM, every snapshot), with the same
// resolver that admitted them - one authority, no second comparison in a differently rounded domain.
FeedbackReason checkObservation(const AdaptiveObservation& observation,
                                const FeedbackPolicy& policy, double nowS);
// Commanded targets: live limits (inclusive), step continuity and the analytic rates of the path.
FeedbackReason checkSegment(const LocomotionFrame& candidate, const JointTargetFrame& previous,
                            const FeedbackPolicy& policy, RefusalRecord* why=nullptr);
FeedbackReason checkAssessment(const SegmentAssessment& assessment, uint32_t sequence,
                               const FeedbackPolicy& policy, double nowS, bool submitted=false);
FeedbackReason checkTracking(const AdaptiveObservation& observation, const FeedbackFrame& frame,
                             const FeedbackPolicy& policy);
// Before ACK, require support already held by both frames; a future touchdown
// cannot be treated as an already established contact.
FeedbackReason checkHeldSupport(const AdaptiveObservation& observation, const LocomotionFrame& previous,
                                const FeedbackFrame& pending);
// Infer the branch of observed semantic angles through FK -> G2 IK. No encoder interpolation.
LocomotionFrame observedSolution(const AdaptiveObservation& observation);
FeedbackFrame feedbackFrame(const LocomotionFrame& solution, uint32_t sequence);
} }
#endif
