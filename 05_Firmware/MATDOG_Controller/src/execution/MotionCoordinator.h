#ifndef MATDOG_EXECUTION_MOTION_COORDINATOR_H
#define MATDOG_EXECUTION_MOTION_COORDINATOR_H
#include "../core/MotionService.h"
#include "../motion/AdaptiveStand.h"
#include "../motion/FeedbackWalk.h"
#include "MotionSession.h"
#include "../core/MotionClock.h"
#include "../motion/OperationalWalkProfile.h"
#include "../motion/OperationalTolerance.h"
namespace matdog { namespace execution {
// Fixture commissioning STAND: broad PRONE recognition plus a staged entry
// from the armed pose to the fixture route head, certified offline from the
// measured start (RAM-only reference). Rates/limits come from the feedback policy.
struct CommissioningStandConfig {
  bool enabled=false;
  motion::FixtureEntryCertificate certificate{};
  double minStageDurationS=3, proneMaxDeviationRad=0, proneMaxRmsRad=0;
  // Operational mode (2026-10-14): plan the entry from the measured held pose
  // (planMeasuredEntry) — no certificate tube. Keep-outs come from CAD/FCL.
  bool measuredStart=false;
  static constexpr unsigned kMaxKeepouts=8;
  motion::startpose::JointKeepOut keepouts[kMaxKeepouts]{};
  unsigned keepoutCount=0;
  double pathMarginRad=.001;
  // Offline certificate MATDOG_MEASURED_ENTRY_REGION_CERTIFICATE_V1 (2026-10-15,
  // certify_measured_entry_region.py): the 120 non-adjacent pairs and the 12
  // adjacent revolutes of all four legs are clear for EVERY start within this
  // band of the route head, both entry stages, with the compiled CAD tube.
  // See TOLERANCE_CONTRACT_V1: adjacent domains stay inside URDF; physical
  // installation uncertainty is not validated by the encoder. A wider
  // runtime band or tube leaves the certified region: refused, never clamped.
  static constexpr double kCertifiedBandRad=.17453292519943295, kCertifiedTubeRad=motion::tolerance::kGeometryTubeRad;
};
// Arms the motion session on demand (prime at the measured pose -> energize
// -> arm) for an explicit command. Implemented by the Controller.
class SessionArmer {
 public:
  virtual ~SessionArmer()=default;
  virtual const char* armForCommand(uint32_t now_ms)=0;  // nullptr = arming started
};
// The semantic service, planner and executor share one Controller thread.
// Configuration/evidence are supplied locally by reviewed artifacts, never Web.
class MotionCoordinator : public core::SemanticMotionPlanner {
 public:
  void begin(MotionExecutor*,actuator::SafeActuatorPolicy*);
  bool configure(const motion::FeedbackPolicy&,const motion::WalkSpecification&,const motion::AcquisitionRoute*);
  void bindSession(MotionSession* session){session_=session;}
  void bindArmer(SessionArmer* armer){armer_=armer;}
  bool arming() const override {return pending_stand_;}
  const char* armingFailure() const override {return arming_failure_;}
  const char* lastEntryReason() const {return entry_reason_;}
  bool walking() const {return walking_;}
  double walkCycles() const {return walking_?walk_.cycles():0;}
  // Operational first stride (2026-10-15): the 250 ms walk TTL is renewed by
  // the Controller ONLY while the authenticated operator heartbeat is current;
  // without it the stride expires (COMMAND_EXPIRED -> SAFE_OFF). Never starts one.
  bool renewWalk(uint32_t now_ms);
  unsigned proneSegment() const{return stand_.proneSegment();}
  bool approveProneSegment(unsigned s){return stand_.approveProneSegment(s);}
  bool configureCommissioning(const CommissioningStandConfig&);
  void clearCommissioning(){commissioning_=CommissioningStandConfig{};entry_plan_=motion::FixtureEntryPlan{};}
  bool approveEntryStage(unsigned s){return stand_.approveEntryStage(s);}
  bool approveProneThrough(unsigned last){return stand_.approveProneThrough(last);}
  const motion::AdaptiveStand& stand() const {return stand_;}
  const motion::FixtureEntryPlan& entryPlan() const {return entry_plan_;}
  const CommissioningStandConfig& commissioning() const {return commissioning_;}
  void observe(const motion::AdaptiveObservation&);
  bool assess(const motion::SegmentAssessment&,uint32_t now_ms);
  const char* accept(const motion::MotionCommand&,uint32_t now_ms) override;
  void stop(uint32_t now_ms) override;
  const char* state() const override;
  void update(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
  // now_us: the Controller's monotonic 64-bit clock (Issue #52 streaming).
  void update(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms,uint64_t now_us);
  const core::SchedulerCounters& schedulerCounters() const {return scheduler_.counters();}
  // The Controller's monotonic clock: the walk uses ONE time base (seconds of
  // this clock) for start, commands, renewals and streaming. Unbound (host
  // tests): now_ms/1000, with update(...,now_ms) passing now_ms*1000 us.
  void bindClock(const core::MonotonicClock64* clock){clock_=clock;}
  bool streamingWalk() const {return walking_&&active_spec_.timeDriven;}
  // The running walk profile (drive pair latched by the Controller), or nullptr.
  const motion::operational_walk::WalkProfile* walkProfile() const {return walking_?walk_profile_:nullptr;}
  const motion::FeedbackFrame& frame() const { return frame_; }
  bool coordinatedStopping() const override { return walking_&&walk_.state()==motion::FeedbackWalkState::STOPPING; }
 private:
  void synchronizeSession();
  const char* startStand(const motion::MotionCommand&,uint32_t now_ms);
  // Read-only dry run of recognition + measured-start entry on the published
  // observation, BEFORE any torque: a STAND that the same checks would refuse
  // after arming is refused here, so a refused pose is never energized.
  const char* preArmCheck();
  SessionArmer* armer_=nullptr;
  // Operational STAND height: the certified entry always ends at 150 mm; a lower request is
  // reached afterwards by AdaptiveStand::requestHeightAdjust.
  double final_height_m_=.150;
  bool pending_walk_=false;motion::MotionCommand pending_walk_command_{};
  bool pending_stand_=false;motion::MotionCommand pending_command_{};uint32_t pending_since_ms_=0;
  motion::PlanarVelocityCommand walk_command_{};
  const char* entry_reason_="";const char* arming_failure_=nullptr;
  MotionExecutor* executor_=nullptr;
  MotionSession* session_=nullptr;
  actuator::SafeActuatorPolicy* policy_=nullptr;
  motion::AdaptiveStand stand_{};
  motion::FeedbackWalk walk_{};
  motion::FeedbackPolicy feedback_policy_{};
  motion::WalkSpecification walk_specification_{};
  motion::AcquisitionRoute acquisition_{};
  motion::AdaptiveObservation observation_{};
  motion::FeedbackFrame frame_{};
  motion::SegmentAssessment assessment_{};
  CommissioningStandConfig commissioning_{};
  motion::FixtureEntryPlan entry_plan_{};
  bool configured_=false,walking_=false;
  core::PeriodicScheduler scheduler_{};
  motion::WalkSpecification active_spec_{};
  const motion::operational_walk::WalkProfile* walk_profile_=nullptr;
  const core::MonotonicClock64* clock_=nullptr;
  double walkTime(uint32_t now_ms) const {return clock_?(clock_->now()/1e6):now_ms/1000.0;}
  void streamWalk(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms,uint64_t now_us);
  uint32_t submitted_sequence_=0,arm_generation_=0;
};
} }
#endif
