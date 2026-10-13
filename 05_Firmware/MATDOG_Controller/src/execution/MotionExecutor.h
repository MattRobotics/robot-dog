#ifndef MATDOG_EXECUTION_MOTION_EXECUTOR_H
#define MATDOG_EXECUTION_MOTION_EXECUTOR_H
#include "../actuator/ActuatorRuntime.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../motion/StandTrajectory.h"
namespace matdog { namespace execution {
enum class ExecutorState : uint8_t { OFF, READY, WRITING, TRACKING, SAFE_OFF_REQUIRED, FAULT };
struct MotionExecutionFacts {
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
  bool calibration_accepted=false, envelopes_approved=false, hardware_qualified=false;
  bool operator_permit=false, bms_ok=false, power_ok=false, fault=false;
  bool sequential_bus_qualified=false;
  bool prime_profiles_verified=false; // twelve live GoalSpeed/ACC readbacks
  uint32_t permit_generation=0, permit_expires_at_ms=0;
};
// These limits must come from an approved artifact; zero defaults approve nothing.
struct ExecutionEnvelope {
  double max_velocity_rad_s=0, max_acceleration_rad_s2=0, tracking_tolerance_rad=0;
  uint32_t feedback_max_age_ms=0, batch_timeout_ms=0, tracking_timeout_ms=0;
  bool approved=false;
};
struct JointFeedback {
  bool read_ok[12]{}, torque_on[12]{};
  uint32_t sampled_at_ms[12]{};
  double actual_rad[12]{};
};
struct FrameValidation {
  uint32_t sequence=0, policy_epoch=0;
  actuator::GeometryProvenanceTag geometry=actuator::kNoGeometryProvenance;
  bool collision_clear=false, support_valid=false, path_continuous=false, configuration_valid=false;
};
struct ExecutorStatus {
  ExecutorState state=ExecutorState::OFF;
  const char* reason="NOT_ARMED";
  uint32_t accepted_sequence=0, completed_sequence=0, submitted_at_ms=0;
  uint32_t arm_generation=0;
  uint8_t next_joint=0;
  double max_tracking_error_rad=0;
  bool safe_off_required=false;
};
// Owns neither bus nor authority. The Controller supplies an existing lease and
// the ONE ActuatorRuntime. No raw write or authority grant API exists here.
class MotionExecutor {
 public:
  void begin(actuator::SafeActuatorPolicy*,actuator::ActuatorRuntime*,const actuator::CalibrationGeometryProfile*,const actuator::GeometryProvenance*,const core::ActuatorAuthorityArbiter*);
  bool arm(const MotionExecutionFacts&, const ExecutionEnvelope&, const JointFeedback&,uint32_t now_ms);
  bool submit(const motion::JointTargetFrame&,const FrameValidation&,uint32_t issued_at_ms,uint32_t ttl_ms,double segment_duration_s,const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
  void update(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
  void stop(const char* reason="STOP");
  void confirmSafeOff(bool all_verified_off);
  const ExecutorStatus& status() const { return status_; }
  bool active() const { return status_.state==ExecutorState::READY||status_.state==ExecutorState::WRITING||status_.state==ExecutorState::TRACKING; }
  const double* targetRadians() const { return target_; }
 private:
  const char* gate(const MotionExecutionFacts&,uint32_t) const;
  bool feedbackValid(const JointFeedback&,uint32_t) const;
  bool resolve(uint8_t index,double radians,actuator::ActuatorCommand*,uint8_t*) const;
  bool reject(const char* reason);
  actuator::SafeActuatorPolicy* policy_=nullptr;
  actuator::ActuatorRuntime* runtime_=nullptr;
  const actuator::CalibrationGeometryProfile* geometry_=nullptr;
  const actuator::GeometryProvenance* provenance_=nullptr;
  const core::ActuatorAuthorityArbiter* arbiter_=nullptr;
  ExecutionEnvelope envelope_{};
  ExecutorStatus status_{};
  FrameValidation validation_{};
  double target_[12]{},previous_[12]{},velocity_[12]{},pending_velocity_[12]{};
  uint32_t deadline_ms_=0,tracking_since_ms_=0,policy_epoch_=0,permit_generation_=0;
  core::AuthorityLease lease_{};
  uint32_t arm_generation_=0;
};
const char* toString(ExecutorState);
} }
#endif
