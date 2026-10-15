#ifndef MATDOG_MOTION_FEEDBACK_WALK_H
#define MATDOG_MOTION_FEEDBACK_WALK_H
#include "AdaptiveStand.h"
namespace matdog { namespace motion {
struct PlanarVelocityCommand {
  double vxMps=0, vyMps=0, yawRateRadS=0;
  double stampS=0, ttlS=0;
  uint32_t sequence=0;
};
struct WalkSpecification {
  double periodS=0, rampDurationS=0, liftM=0, duty=0;
  double swayXM=0,standShiftDurationS=0;
  bool firstStrideOnly=false; // explicit suspended one-cycle clock adapter, G4 unchanged
  double firstStrideTimeScale=1,maxJointStepRad=0;
  // Optional walk-only joint rate envelope (>0 overrides the stand policy).
  double maxVelocityRadS=0, maxAccelerationRadS2=0;
  bool compiledFirstStride=false;
  bool smoothStlReference=false; // C2 reference; requires separate bearing-contact qualification
  bool stlContactReference=false; // explicit qualification axis, not authority
  double maxVxMps=0, maxVyMps=0, maxYawRateRadS=0;
  double maxCommandTtlS=0;
  uint32_t firstTargetSequence=1;
  // Time-driven streaming (Issue #52, 2026-10-15). The phase follows the
  // monotonic clock passed to streamUpdate(), never the completion of the
  // previous microframe; tracking is monitored asynchronously by the executor.
  // Every frame is still checked for continuity and rates over the REAL
  // elapsed time, joint limits, support and the certified region box.
  bool timeDriven=false;
  double streamMaxObservationAgeS=0;  // age bound of the measured joint vector
  double streamMaxGapS=0;             // largest release gap planned through, else stop
  const double* regionMinRad=nullptr; // certified per-joint box (12), slot leg*3+joint
  const double* regionMaxRad=nullptr;
  // Executor streaming contract for this walk (MotionCoordinator -> StreamConfig).
  double streamPeriodS=0, streamMaxLagS=0, streamFeedbackMaxAgeS=0, streamSettleSpeedRadS=0, streamHoldTimeoutS=0;
  unsigned streamPersistSamples=0;
};
// HOLD: time-driven only; final all-foot target emitted, waiting for measured settle.
enum class FeedbackWalkState : uint8_t { IDLE, STARTING, WALK, STOPPING, STAND, REJECTED, HOLD };
// G4 geometry with feedback-paced phase, explicit witnesses and coordinated stop.
// It does not claim dynamic stability or own actuator/network access.
class FeedbackWalk {
 public:
  bool start(const AdaptiveStand& stand,const AdaptiveObservation& observation,
             const PlanarVelocityCommand& command,const WalkSpecification& specification,
             const FeedbackPolicy& policy,double nowS);
  bool command(const PlanarVelocityCommand& command,double nowS);
  FeedbackFrame update(const AdaptiveObservation& observation,double nowS);
  // Time-driven streaming step: one frame for the instant nowS (monotonic,
  // seconds). Never waits for the previous frame to be reached.
  FeedbackFrame streamUpdate(const AdaptiveObservation& observation,double nowS);
  uint32_t holdSequence() const {return holdSequence_;}
  double clock() const {return clockS_;}
  bool assess(const SegmentAssessment& assessment,double nowS);
  void requestStop();
  FeedbackWalkState state() const {return state_;}
  FeedbackReason reason() const {return reason_;}
  double cycles() const {return cycles_;}
 private:
  FeedbackFrame reject(FeedbackReason reason);
  bool validCommand(const PlanarVelocityCommand& command,double nowS) const;
  LocomotionFrame generate(double nextClockS);
  FeedbackPolicy policy_{};
  WalkSpecification specification_{};
  PlanarVelocityCommand command_{},queuedCommand_{};
  bool queuedVelocity_=false;
  GaitParameters gait_{};
  BodyPose origin_{};
  LocomotionFrame previous_{}, pendingSolution_{};
  FeedbackFrame pending_{};
  SegmentAssessment assessment_{};
  FeedbackWalkState state_=FeedbackWalkState::IDLE;
  FeedbackReason reason_=FeedbackReason::NOT_REQUESTED;
  double clockS_=0, pendingClockS_=0, cycles_=0, stopCycle_=-1, stopClockS_=0;
  double lastNow_=0, pendingSince_=0;
  uint32_t targetSequence_=1;
  double startS_=0,lastFrameS_=0,holdSinceS_=0;uint32_t frames_=0,holdSequence_=0;
  bool pendingValid_=false, assessed_=false;
};
} }
#endif
