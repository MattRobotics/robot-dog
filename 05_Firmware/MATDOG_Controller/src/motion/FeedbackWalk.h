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
  bool stlContactReference=false; // explicit qualification axis, not authority
  double maxVxMps=0, maxVyMps=0, maxYawRateRadS=0;
  double maxCommandTtlS=0;
  uint32_t firstTargetSequence=1;
};
enum class FeedbackWalkState : uint8_t { IDLE, STARTING, WALK, STOPPING, STAND, REJECTED };
// G4 geometry with feedback-paced phase, explicit witnesses and coordinated stop.
// It does not claim dynamic stability or own actuator/network access.
class FeedbackWalk {
 public:
  bool start(const AdaptiveStand& stand,const AdaptiveObservation& observation,
             const PlanarVelocityCommand& command,const WalkSpecification& specification,
             const FeedbackPolicy& policy,double nowS);
  bool command(const PlanarVelocityCommand& command,double nowS);
  FeedbackFrame update(const AdaptiveObservation& observation,double nowS);
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
  bool pendingValid_=false, assessed_=false;
};
} }
#endif
