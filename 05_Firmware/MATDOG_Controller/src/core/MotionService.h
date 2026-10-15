#ifndef MATDOG_CORE_MOTION_SERVICE_H
#define MATDOG_CORE_MOTION_SERVICE_H
#include "../motion/MotionContract.h"
#include "../execution/MotionExecutor.h"
namespace matdog { namespace core {
struct MotionReadiness {
 bool calibration_restored=false, calibration_accepted=false, limits_approved=false;
 bool walk_qualified=false,first_stride_only=false;double max_vx=0,max_vy=0,max_yaw_rate=0;
 bool hardware_approved=false, operational_permit=false, support_observable=false;
 bool local_fixture_session=false; // local session owns micro-test lifecycle
 bool local_operator_owns_session=false; // ownership persists between segments/strides
 bool power_ok=false, bms_ok=false, fault=false, inhibited=false, authority_live=false;
 OperatingMode mode=OperatingMode::MAINTENANCE;
};
class SemanticMotionPlanner {
 public:
  virtual ~SemanticMotionPlanner()=default;
  virtual const char* accept(const motion::MotionCommand&,uint32_t)=0;
  virtual void stop(uint32_t)=0;
  virtual const char* state() const=0;
  virtual bool coordinatedStopping() const { return false; }
};
struct MotionTelemetry {
 bool joints_valid=false, quaternion_valid=false, height_valid=false,raw_valid=false;
 uint32_t joint_at_ms[12]{};
 uint16_t raw_ticks[12]{};
 uint32_t sampled_at_ms=0, quaternion_at_ms=0;
 double actual_rad[12]{}, quaternion[4]{1,0,0,0}, base_height_m=0;
 bool validation_capable=false,fixture_scope=false,load_scope=false,foot_load_proven=false;unsigned supervised_segment=0;
 const char* source="UNKNOWN";
 const char* session_profile_sha256="";
 const char* session_state="OFF";const char* session_reason="LOCAL_TRIAL_EVIDENCE_REQUIRED";
 // Runtime test policy view (read-only). Distinct reasons, never one badge.
 const char* runtime_mode="DIAGNOSTIC";const char* limits_scope="NONE";const char* restore="NOT_RUN";
 const char* pose_class="NOT_EVALUATED";const char* pose_reason="";int pose_joint=-1;
 double pose_max_dev_rad=-1,pose_rms_dev_rad=-1;
 const char* entry="NONE";const char* phase="";const char* commissioning_reason="";
 bool fixture_declared=false;uint8_t approved_joints=0,commissioning_joints=0;uint16_t prime_clamp_mask=0;
 // Independent channel facts (14 Oct: one "stale" hid fresh IMU/raw data).
 // joint_mapped[i]: a RESTORED transform produced actual_rad[i] from a fresh,
 // healthy read. raw_ok[i]: the raw register read itself succeeded.
 bool joint_mapped[12]{}, raw_ok[12]{};
 // Issue #48 restore state (static strings owned by the policy layer).
 const char* restore_decision="NOT_EVALUATED";const char* restore_reason="";const char* restore_trigger="NONE";
 uint32_t restore_generation=0;uint64_t restore_fingerprint=0;
};
// Controller owns this transport-independent intention service. A transport
// submits a typed command and receives an ack; it cannot acquire a lease.
class MotionService {
 public:
  void begin(execution::MotionExecutor*,SemanticMotionPlanner*);
  void refresh(const MotionReadiness&,uint32_t now_ms);
  motion::MotionAck command(const motion::MotionCommand&,uint32_t now_ms);
  bool available(const motion::MotionAction&) const;
  const char* refusal() const;
  bool statusJson(char*,size_t,uint32_t now_ms) const;
  bool capabilitiesJson(char*,size_t) const;
  bool ackJson(const motion::MotionAck&,char*,size_t) const;
  void publishTelemetry(const MotionTelemetry& t) { telemetry_=t; }
  bool busy() const { return executor_&&(executor_->active()||executor_->status().safe_off_required); }
  const MotionReadiness& readiness() const { return readiness_; }
  const motion::MotionAck& lastAck() const { return last_ack_; }
  const motion::MotionAck& activeAck() const { return active_ack_; }
  void disconnected(uint32_t now_ms);
 private:
  void finishActive(motion::CommandOutcome,const char*);
  struct Replay { uint32_t sequence=0;motion::MotionCommand command{};motion::MotionAck ack{}; };
  execution::MotionExecutor* executor_=nullptr;
  SemanticMotionPlanner* planner_=nullptr;
  MotionReadiness readiness_{};
  MotionTelemetry telemetry_{};
  motion::MotionAck last_ack_{},active_ack_{};
  motion::Intent active_intent_=motion::Intent::UNKNOWN;
  bool active_command_=false,active_zero_velocity_=false;
  Replay replay_[3]{};
  motion::ClientOwner command_owner_=motion::ClientOwner::UNKNOWN;
  uint32_t teleop_deadline_ms_=0,active_arm_generation_=0;
};
} }
#endif
