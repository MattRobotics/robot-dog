#ifndef MATDOG_EXECUTION_MOTION_SESSION_H
#define MATDOG_EXECUTION_MOTION_SESSION_H
#include "MotionExecutor.h"
namespace matdog { namespace execution {
enum class TrialScope : uint8_t { FIXTURE_MICRO, AUTONOMOUS_STAND, AUTONOMOUS_WALK, SUPERVISED_FIXTURE_STAND, SUPERVISED_FIXTURE_WALK,
 FIXTURE_COMMISSIONING }; // runtime commissioning policy; never loadable from reviewed cards
struct LocalTrialEvidence {
 TrialScope scope=TrialScope::FIXTURE_MICRO;
 uint32_t generation=0,expires_at_ms=0;
 bool local_operator_present=false,physical_power_cut_verified=false;
 bool fixture_support_verified=false,installation_verified=false;
 bool servo_output_ceiling_verified=false,imu_fresh=false;
 bool support_load_observable=false,loaded_dynamics_qualified=false;
 double max_fixture_excursion_rad=0; // reviewed local micro-test limit; no default
 // FIXTURE_COMMISSIONING only: a joint resting up to this many ticks OUTSIDE
 // its hard limit (folded LOWER at the stop) is primed at the nearest inside
 // tick instead of its readback. 0 = exact readback prime only.
 uint16_t prime_clamp_max_ticks=0;
};
constexpr uint16_t kPrimeInwardMarginTicks=2;
enum class MotionSessionState : uint8_t { OFF, PRIMING, ENERGIZING, ARMING, ARMED, SAFE_OFF_REQUIRED };
// LOCAL Controller lifecycle, no HTTP binding. A separate operator permit and
// scoped evidence are required. Prime writes PRESENT targets while torque is
// OFF; torque then comes through the same SafeActuatorRuntime. One bus op/tick.
class MotionSession {
 public:
 void begin(actuator::SafeActuatorPolicy*,actuator::ActuatorRuntime*,MotionExecutor*,
            core::ActuatorAuthorityArbiter*,const actuator::CalibrationGeometryProfile*,const actuator::GeometryProvenance*);
 bool request(const LocalTrialEvidence&,const MotionExecutionFacts&,const ExecutionEnvelope&,
              const JointFeedback&,bool approved_image,uint32_t now_ms);
 void update(const LocalTrialEvidence&,const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
 bool requestFixtureMicro(unsigned joint,double delta_rad,double duration_s,const FrameValidation& path,
                          const MotionExecutionFacts&,const JointFeedback&,uint32_t now_ms);
 void revoke(const char* reason);
 void confirmSafeOff();
 MotionSessionState state() const {return state_;}
 TrialScope scope() const {return evidence_.scope;}
 const char* stateName() const {
  const char* names[]={"OFF","PRIMING","ENERGIZING","ARMING","ARMED","SAFE_OFF_REQUIRED"};
  return names[static_cast<unsigned>(state_)];
 }
 const char* reason() const {return reason_;}
 core::AuthorityLease lease() const {return lease_;}
 bool safeOffRequired() const {return state_==MotionSessionState::SAFE_OFF_REQUIRED;}
 private:
 const char* gate(const LocalTrialEvidence&,const MotionExecutionFacts&,const JointFeedback&,uint32_t) const;
 bool resolve(unsigned,double,actuator::ActuatorCommand&,uint8_t&) const;
 bool insideTick(unsigned joint,double q,uint16_t max_ticks,double& inside,uint8_t& ticks) const;
 actuator::SafeActuatorPolicy* policy_=nullptr;actuator::ActuatorRuntime* runtime_=nullptr;
 MotionExecutor* executor_=nullptr;core::ActuatorAuthorityArbiter* arbiter_=nullptr;
 const actuator::CalibrationGeometryProfile* geometry_=nullptr;const actuator::GeometryProvenance* provenance_=nullptr;
 core::AuthorityLease lease_{};LocalTrialEvidence evidence_{};ExecutionEnvelope envelope_{};
 MotionSessionState state_=MotionSessionState::OFF;const char* reason_="LOCAL_TRIAL_EVIDENCE_REQUIRED";
 motion::JointTargetFrame micro_start_{};FrameValidation micro_path_{};
 bool micro_active_=false;unsigned micro_joint_=0,micro_step_=0,micro_intervals_=0;
 double micro_delta_=0,micro_duration_=0;
 uint32_t micro_sequence_=0,micro_issued_at_ms_=0;
 double primed_rad_[12]{},rest_rad_[12]{};uint8_t next_=0;uint32_t epoch_=0,permit_generation_=0,started_at_ms_=0;
 uint16_t clamp_mask_=0;uint8_t clamp_ticks_[12]{};
 // PRIME/ENERGIZE write diagnostics (2026-10-17): the policy decision or the
 // backend outcome and the joint, instead of one merged reason.
 uint8_t write_attempts_=0;int8_t failed_joint_=-1;const char* failure_detail_="";const char* last_revoke_="";
 public:
 // A goal write with torque OFF and a torque-enable are idempotent and verified
 // by readback: a missed reply on the half-duplex bus is retried on the next
 // tick (same joint, same target, re-planned through the policy) before the
 // session is revoked. A policy refusal is never retried.
 static constexpr uint8_t kWriteAttempts=3;
 int failedJoint() const {return failed_joint_;}
 const char* failureDetail() const {return failure_detail_;}
 // The cause that ended the last session (kept after SAFE_OFF_VERIFIED).
 const char* lastRevoke() const {return last_revoke_;}
 public:
 // Joints primed at a clamped inside tick, and by how many ticks (diagnostics).
 uint16_t primeClampMask() const {return clamp_mask_;}
 uint8_t primeClampTicks(unsigned i) const {return i<12?clamp_ticks_[i]:0;}
 const double* primedRadians() const {return primed_rad_;}
};
} }
#endif
