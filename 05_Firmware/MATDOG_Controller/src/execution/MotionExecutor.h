#ifndef MATDOG_EXECUTION_MOTION_EXECUTOR_H
#define MATDOG_EXECUTION_MOTION_EXECUTOR_H
#include "../actuator/ActuatorRuntime.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../motion/StandTrajectory.h"
#include "../motion/MotionLaw.h"
#include "../motion/MotionFeedback.h"
namespace matdog { namespace execution {
// STREAMING/HOLD (Issue #52): time-driven frames, one coordinated bus transaction
// per frame, asynchronous per-joint tracking, shared by timed STAND and WALK.
// WRITING/TRACKING retain finite commands and measured checkpoints.
enum class ExecutorState : uint8_t { OFF, READY, WRITING, TRACKING, SAFE_OFF_REQUIRED, FAULT, STREAMING, HOLD };
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
  // Issue #52: segment frames (prime/entry/STAND) as ONE coordinated frame
  // transaction instead of twelve single writes.
  bool frame_writes=false;
  // Operational timed STAND selects streaming; finite submit accepts only
  // measured endpoints with this arrival tolerance. Zero retains legacy arrival.
  double arrival_tolerance_rad=0;
  bool timed_segments=false;
};
struct JointFeedback {
  bool read_ok[12]{}, torque_on[12]{}, recovering[12]{};
  bool pending() const {for(bool r:recovering)if(r)return true;return false;}
  uint32_t sampled_at_ms[12]{};
  double actual_rad[12]{};
  // Issue #52: real per-joint sample instant on the monotonic 64-bit clock
  // (read completion; timestamp 0 is invalid), and measured signed rad/s.
  // Speed zero is a valid measurement, not a missing-value marker.
  uint64_t sampled_at_us[12]{};
  double speed_rad_s[12]{};
  // Core Motion V6: the previous valid sample of each joint (instant 0 = none), kept by the producer so a common
  // instant can be reconstructed between two real samples of the same joint. Never a substitute for a fresh sample.
  uint64_t previous_at_us[12]{};
  double previous_rad[12]{};
};
struct FrameValidation {
  uint32_t sequence=0, policy_epoch=0;
  actuator::GeometryProvenanceTag geometry=actuator::kNoGeometryProvenance;
  bool collision_clear=false, support_valid=false, path_continuous=false, configuration_valid=false;
  bool final=true; // phase/entry endpoint: require arrival, not just following
};
struct ExecutorStatus {
  ExecutorState state=ExecutorState::OFF;
  const char* reason="NOT_ARMED";
  uint32_t accepted_sequence=0, completed_sequence=0, submitted_at_ms=0;
  uint32_t arm_generation=0;
  uint8_t next_joint=0;
  double max_tracking_error_rad=0;
  bool safe_off_required=false;
  // Streaming sequences (Issue #52). TX is NOT application; observed = every
  // joint sampled after that frame's TX-complete; settled = HOLD target reached
  // by fresh samples, inside the tube, below the configured speed ceiling.
  // STAND additionally requires measured position stability across samples.
  uint32_t validated_sequence=0, tx_sequence=0, observed_sequence=0, settled_sequence=0;
  uint32_t frames_tx=0, frames_refused=0;
  uint32_t last_frame_interval_us=0, max_frame_interval_us=0;
  uint64_t last_tx_complete_us=0;
  double stream_error_rad[12]{}, stream_peak_error_rad=0;
  uint8_t worst_joint=0xff;
  int8_t feedback_failed_joint=-1;
  uint32_t feedback_checked_ms=0, feedback_sample_ms=0, feedback_budget_ms=0;
};
// Streaming contract. Tracking is a following monitor: geometry safety comes
// from the certified box (every target inside region, measured inside region
// +/- tube or immediate stop); a joint further than tube_rad from every
// reference it was commanded during the last max_lag_s for persist_samples
// consecutive fresh samples is a physical failure to follow -> stop.
struct StreamConfig {
  double tube_rad=0, max_lag_s=0, settle_speed_rad_s=0;
  // STAND: the measured CONFIGURATION must lie in the tube of the endpoint active at a common instant (path box),
  // independent of timestamp-aligned per-joint tracking. No WALK lag allowance.
  bool local_path_box=false;
  double max_step_rad=0;
  // Path box reconstruction: physical joint acceleration bound between two samples of a joint (rad/s^2). HYPOTHESIS:
  // the ST3215 ACC register of the STAND drive pair, verified by readback; not a metrological bound. 0 = only joints
  // sampled exactly at the common instant can be placed there.
  double sample_acceleration_bound_rad_s2=0;
  uint32_t feedback_max_age_us=0, hold_timeout_us=0, max_frame_gap_us=0;
  uint8_t persist_samples=0;
  const double* region_min_rad=nullptr; const double* region_max_rad=nullptr;
};
// The ONE slot binding and radians -> RAW resolution of Motion (Core Motion V6), shared by MotionSession (PRIME,
// ENERGIZE) and MotionExecutor (ARM, every frame, every measured sample): slot identity and servo, live transform and
// RAW limit, the Geometry V5 joint on the same bus, the resolver's canonical conversion. The RAW tick is returned even
// outside the limit, so a caller can tell an expected contact from a mapping fault.
enum class JointResolve : uint8_t { OK, MAPPING, OUTSIDE_LIMITS };
struct ResolvedJoint { actuator::ActuatorCommand command{}; uint8_t bus=0; uint16_t tick=0; const actuator::JointLimit* limit=nullptr; };
JointResolve resolveJoint(const actuator::SafeActuatorPolicy&,const actuator::CalibrationGeometryProfile&,const actuator::GeometryProvenance&,
                          unsigned slot,double q_rad,ResolvedJoint* out);
// What an armed session starts from (MotionSession::planPrime): the goal registers PRIME wrote (inside the RAW limits)
// and, for a folded LOWER primed off its stop, the measured rest at the expected contact.
struct ArmHold { const double* hold_rad=nullptr; const double* rest_rad=nullptr; uint16_t contact_mask=0; };
// Owns neither bus nor authority. The Controller supplies an existing lease and
// the ONE ActuatorRuntime. No raw write or authority grant API exists here.
class MotionExecutor {
 public:
  void begin(actuator::SafeActuatorPolicy*,actuator::ActuatorRuntime*,const actuator::CalibrationGeometryProfile*,const actuator::GeometryProvenance*,const core::ActuatorAuthorityArbiter*);
  // hold == nullptr: the measured pose is the held reference. With a hold, the reference is the PRIME goal of every
  // joint (the commanded, RAW-quantized target), the measured pose must follow it within the tracking tolerance, and a
  // contact-masked joint may be MEASURED between its rest and its hard limit - never deeper than the rest - until the
  // first settled endpoint (the existing exit checkpoint). Targets are never admitted outside the RAW limits.
  bool arm(const MotionExecutionFacts&, const ExecutionEnvelope&, const JointFeedback&,uint32_t now_ms,const ArmHold* hold=nullptr);
  bool submit(const motion::JointTargetFrame&,const FrameValidation&,uint32_t issued_at_ms,uint32_t ttl_ms,double segment_duration_s,const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
  // Walk-only joint rate envelope (2026-10-15). Accepted only between frames
  // (READY), never above the compiled ceilings, never touching tracking,
  // feedback or timeouts. restoreArmedRates() returns to the armed envelope.
  // Covers every operational walk profile envelope (OperationalWalkProfile.h,
  // static_assert in MotionCoordinator.cpp); FAST experimental = 1.55 / 13.0.
  static constexpr double kWalkRateCeilingRadS=1.6, kWalkAccelerationCeilingRadS2=13.5;
  bool selectRates(double max_velocity_rad_s,double max_acceleration_rad_s2);
  void restoreArmedRates(){ if(armed_envelope_.approved){envelope_.max_velocity_rad_s=armed_envelope_.max_velocity_rad_s;envelope_.max_acceleration_rad_s2=armed_envelope_.max_acceleration_rad_s2;} }
  uint32_t trackingTimeoutUs() const {return envelope_.tracking_timeout_ms*1000;}
  double maxVelocityRadS() const { return envelope_.max_velocity_rad_s; }
  bool timedSegmentsActive() const {return active()&&envelope_.timed_segments&&!streaming();}
  static constexpr uint32_t kTimedFeedbackMaxAgeMs=100, kTimedMaxFrameGapMs=100;
  void update(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms,uint64_t now_us=0);
  // ---- streaming (Issue #52) ----
  bool beginStream(const StreamConfig&,const JointFeedback&,uint64_t now_us);
  // Validates the WHOLE frame (12 identities, raw ticks, URDF/gen-1 limits,
  // region box, rates over the real interval, lease/epoch/permit, deadline)
  // before one coordinated transaction. final=true enters HOLD.
  bool stream(const motion::JointTargetFrame&,const FrameValidation&,uint64_t release_us,uint64_t deadline_us,bool final,
              const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms,uint64_t now_us);
  // Asynchronous tracking/settle for one feedback snapshot.
  void monitor(const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms,uint64_t now_us);
  // HOLD settled -> READY holding the final target (segment mode resumes).
  bool endStream();
  bool measuredStandQuiet() const {return stand_quiet_;}
  bool streaming() const { return status_.state==ExecutorState::STREAMING||status_.state==ExecutorState::HOLD; }
  // The first stop of an armed session keeps its operands (why: the planner's or another verifier's record; nullptr
  // records the reason alone). Later stops never overwrite it.
  void stop(const char* reason="STOP",const motion::RefusalRecord* why=nullptr);
  void confirmSafeOff(bool all_verified_off);
  const ExecutorStatus& status() const { return status_; }
  bool active() const { return status_.state==ExecutorState::READY||status_.state==ExecutorState::WRITING||status_.state==ExecutorState::TRACKING||streaming(); }
  const double* targetRadians() const { return target_; }
  // The committed release (last TRANSMITTED stream frame): what the generator's retimer and stream() both check against.
  motion::ReleaseState releaseState() const { return {target_,velocity_,last_release_us_,previous_release_interval_us_}; }
  motion::ReleaseEnvelope releaseEnvelope() const { return {envelope_.max_velocity_rad_s,envelope_.max_acceleration_rad_s2,stream_.local_path_box?stream_.max_step_rad:0}; }
  const motion::RefusalRecord& refusal() const { return refusal_; }
  uint16_t contactMask() const { return contact_mask_; }
 private:
  uint32_t segment_duration_ms_=0;
  const char* gate(const MotionExecutionFacts&,uint32_t) const;
  const char* feedbackFailure(const JointFeedback&,uint32_t);
  bool resolve(uint8_t index,double radians,actuator::ActuatorCommand*,uint8_t*) const;
  // A MEASURED position: inside the RAW limits, or (until the exit checkpoint) inside the expected contact band.
  bool actualAdmissible(unsigned slot,double radians) const;
  bool reject(const char* reason);
  bool reject(const char* reason,const motion::RefusalRecord& why);
  actuator::SafeActuatorPolicy* policy_=nullptr;
  actuator::ActuatorRuntime* runtime_=nullptr;
  const actuator::CalibrationGeometryProfile* geometry_=nullptr;
  const actuator::GeometryProvenance* provenance_=nullptr;
  const core::ActuatorAuthorityArbiter* arbiter_=nullptr;
  ExecutionEnvelope envelope_{};
  ExecutionEnvelope armed_envelope_{};
  ExecutorStatus status_{};
  FrameValidation validation_{};
  double target_[12]{},previous_[12]{},velocity_[12]{},pending_velocity_[12]{};
  uint32_t deadline_ms_=0,tracking_since_ms_=0,policy_epoch_=0,permit_generation_=0;
  uint64_t tracking_started_us_=0;
  core::AuthorityLease lease_{};
  uint32_t arm_generation_=0;
  // streaming state
  bool frameTx(const double target[12],const MotionExecutionFacts&,uint64_t now_us,const char** why);
  double referenceRange(unsigned joint,uint64_t from_us,uint64_t to_us,double* lo,double* hi) const;
  // Path box at a common instant: PASS, FAIL (outside beyond its uncertainty) or UNKNOWN (not certifiable).
  enum class BoxVerdict : uint8_t { PASS, FAIL, UNKNOWN };
  BoxVerdict pathBox(const JointFeedback&,uint64_t* instant,motion::RefusalRecord* why) const;
  bool observeStandQuiet(const JointFeedback&,uint64_t now_us);
  bool stand_quiet_=false;
  uint64_t quiet_since_us_[12]{},quiet_sample_us_[12]{};
  double quiet_min_[12]{},quiet_max_[12]{};
  StreamConfig stream_{};
  static constexpr unsigned kHistory=48;
  struct Sent { uint64_t at_us=0; uint32_t sequence=0; bool held_seed=false; double q[12]{}; };
  Sent history_[kHistory]{};
  unsigned history_count_=0, history_head_=0;
  uint64_t previous_release_interval_us_=0;
  uint64_t last_release_us_=0, hold_since_us_=0, checked_us_[12]{};
  uint8_t exceed_[12]{};
  uint32_t hold_sequence_=0;
  // Expected contact (ArmHold): RAW band [lo, hi] of each masked joint, cleared at the first settled endpoint.
  uint16_t contact_mask_=0, contact_lo_[12]{}, contact_hi_[12]{};
  uint64_t box_instant_us_=0;  // last common instant the path box judged
  uint8_t box_unknown_=0;      // consecutive distinct instants that could not be certified
  motion::RefusalRecord refusal_{};
};
const char* toString(ExecutorState);
} }
#endif
