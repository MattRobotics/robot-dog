#ifndef MATDOG_EXECUTION_MOTION_COORDINATOR_H
#define MATDOG_EXECUTION_MOTION_COORDINATOR_H
#include "../core/MotionService.h"
#include "../motion/AdaptiveStand.h"
#include "../motion/FeedbackWalk.h"
namespace matdog { namespace execution {
// The semantic service, planner and executor share one Controller thread.
// Configuration/evidence are supplied locally by reviewed artifacts, never Web.
class MotionCoordinator : public core::SemanticMotionPlanner {
 public:
  void begin(MotionExecutor*,actuator::SafeActuatorPolicy*);
  bool configure(const motion::FeedbackPolicy&,const motion::WalkSpecification&,const motion::AcquisitionRoute*);
  void observe(const motion::AdaptiveObservation&);
  bool assess(const motion::SegmentAssessment&,uint32_t now_ms);
  const char* accept(const motion::MotionCommand&,uint32_t now_ms) override;
  void stop(uint32_t now_ms) override;
  const char* state() const override;
  void update(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
  const motion::FeedbackFrame& frame() const { return frame_; }
  bool coordinatedStopping() const override { return walking_&&walk_.state()==motion::FeedbackWalkState::STOPPING; }
 private:
  void synchronizeSession();
  MotionExecutor* executor_=nullptr;
  actuator::SafeActuatorPolicy* policy_=nullptr;
  motion::AdaptiveStand stand_{};
  motion::FeedbackWalk walk_{};
  motion::FeedbackPolicy feedback_policy_{};
  motion::WalkSpecification walk_specification_{};
  motion::AcquisitionRoute acquisition_{};
  motion::AdaptiveObservation observation_{};
  motion::FeedbackFrame frame_{};
  motion::SegmentAssessment assessment_{};
  bool configured_=false,walking_=false;
  uint32_t submitted_sequence_=0,arm_generation_=0;
};
} }
#endif
