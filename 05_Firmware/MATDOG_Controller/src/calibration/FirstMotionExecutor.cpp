#include "FirstMotionExecutor.h"

namespace matdog {
namespace calibration {

void FirstMotionExecutor::begin(actuator::SafeActuatorPolicy* policy,
                                actuator::ActuatorRuntime* runtime,
                                const actuator::CalibrationGeometryProfile* geometry,
                                const actuator::GeometryProvenance* expected_provenance,
                                const FirstMotionConfig& config) {
  policy_ = policy;
  runtime_ = runtime;
  geometry_ = geometry;
  expected_provenance_ = expected_provenance;
  config_ = config;
  status_ = FirstMotionStatus{};
}

bool FirstMotionExecutor::start(const FirstMotionRequest& request,
                                const FirstMotionContext& context, uint32_t now_ms) {
  (void)context;
  (void)now_ms;

  // Already running: a second start() while one attempt is outstanding
  // would create two callers each believing they own the sequencing - the
  // same class of hazard SafeActuatorPolicy's own "at most one outstanding
  // plan" rule exists to prevent.
  if (active()) return false;

  if (policy_ == nullptr || runtime_ == nullptr || geometry_ == nullptr ||
      expected_provenance_ == nullptr || !request.joint.valid() ||
      !request.joint.unitKnown() || request.delta_ticks == 0) {
    status_ = FirstMotionStatus{};
    status_.state = FirstMotionState::FAILED_NO_MOTION;
    status_.failure = FirstMotionFailure::REJECT_PRECONDITIONS;
    return false;
  }

  request_ = request;
  bus_id_ = request.bus_id;
  status_ = FirstMotionStatus{};
  status_.state = FirstMotionState::TORQUE_ENABLE_PENDING;
  return true;
}

void FirstMotionExecutor::update(const FirstMotionContext& context, uint32_t now_ms,
                                 bool telemetry_available,
                                 const actuator::TelemetrySample& telemetry) {
  switch (status_.state) {
    case FirstMotionState::TORQUE_ENABLE_PENDING:
      stepTorqueEnable(context, now_ms);
      return;
    case FirstMotionState::POSITION_COMMAND_PENDING:
      stepPositionCommand(context, now_ms);
      return;
    case FirstMotionState::MONITORING:
      stepMonitoring(now_ms, telemetry_available, telemetry);
      return;
    case FirstMotionState::IDLE:
    case FirstMotionState::COMPLETE:
    case FirstMotionState::FAILED_NO_MOTION:
    case FirstMotionState::SAFE_OFF_REQUIRED:
      return;  // not started, or a previous attempt's terminal state stands
  }
}

void FirstMotionExecutor::abort() {
  switch (status_.state) {
    case FirstMotionState::TORQUE_ENABLE_PENDING:
      // Torque was never verified applied on this path.
      finish(FirstMotionState::FAILED_NO_MOTION, FirstMotionFailure::OPERATOR_ABORT,
            status_.last_policy_decision);
      return;
    case FirstMotionState::POSITION_COMMAND_PENDING:
    case FirstMotionState::MONITORING:
      // Torque WAS verified applied to reach either of these states.
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::OPERATOR_ABORT,
            status_.last_policy_decision);
      return;
    case FirstMotionState::IDLE:
    case FirstMotionState::COMPLETE:
    case FirstMotionState::FAILED_NO_MOTION:
    case FirstMotionState::SAFE_OFF_REQUIRED:
      return;  // already terminal or never started
  }
}

void FirstMotionExecutor::finish(FirstMotionState state, FirstMotionFailure failure,
                                 actuator::WriteDecision decision) {
  status_.state = state;
  status_.failure = failure;
  status_.last_policy_decision = decision;
}

void FirstMotionExecutor::stepTorqueEnable(const FirstMotionContext& context, uint32_t now_ms) {
  (void)now_ms;
  actuator::ActuatorCommand cmd{};
  cmd.operation = actuator::ActuatorOperation::TORQUE_ENABLE;
  cmd.joint = request_.joint;

  actuator::ActuatorTransaction txn{};
  const actuator::WriteDecision plan_decision =
      policy_->plan(cmd, context.lease, context.mode, &txn);
  status_.last_policy_decision = plan_decision;
  if (plan_decision != actuator::WriteDecision::ACCEPT) {
    finish(FirstMotionState::FAILED_NO_MOTION, FirstMotionFailure::REJECT_PRECONDITIONS,
          plan_decision);
    return;
  }

  actuator::WriteDecision commit_decision = plan_decision;
  const actuator::ExecuteResult result = runtime_->execute(&txn, bus_id_, &commit_decision);
  status_.last_policy_decision = commit_decision;
  switch (result) {
    case actuator::ExecuteResult::WRITTEN:
      status_.state = FirstMotionState::POSITION_COMMAND_PENDING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::TORQUE_ENABLE_UNCERTAIN,
            commit_decision);
      return;
    case actuator::ExecuteResult::NOT_EXECUTED:
    case actuator::ExecuteResult::NO_BACKEND:
    case actuator::ExecuteResult::NO_RAW_TARGET:
    case actuator::ExecuteResult::BACKEND_REJECTED:
      // Verified NOT applied (or never attempted) - safe, no SAFE_OFF needed.
      finish(FirstMotionState::FAILED_NO_MOTION, FirstMotionFailure::TORQUE_ENABLE_REJECTED,
            commit_decision);
      return;
  }
}

void FirstMotionExecutor::stepPositionCommand(const FirstMotionContext& context,
                                              uint32_t now_ms) {
  // Reached only after TorqueEnable was VERIFIED applied - every exit below
  // is therefore SAFE_OFF_REQUIRED, never FAILED_NO_MOTION.
  const actuator::JointTransform* transform =
      policy_->transforms().find(request_.joint, policy_->currentGeometryTag());
  if (transform == nullptr) {
    finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::REJECT_TARGET_RESOLUTION,
          actuator::WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
    return;
  }

  // The SAME checked q<->raw resolver CalibrationExecutionEngine uses
  // internally (CR3-M2) - resolved here too, independently, only so the
  // absolute raw target is available for MotionDeadmanMonitor's arrival
  // check below. No modulo/signed-wrap target semantics either place.
  uint16_t target_tick = 0;
  const actuator::TargetResolveStatus resolve_status = actuator::resolveDeltaFromQ0(
      *geometry_, *expected_provenance_, *transform, request_.delta_ticks, &target_tick);
  if (resolve_status != actuator::TargetResolveStatus::OK) {
    finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::REJECT_TARGET_RESOLUTION,
          actuator::WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
    return;
  }
  status_.target_tick = target_tick;

  actuator::ActuatorCommand cmd{};
  cmd.operation = actuator::ActuatorOperation::DIRECTION_VERIFY;
  cmd.joint = request_.joint;
  cmd.delta_ticks = request_.delta_ticks;
  cmd.target_tick = target_tick;

  actuator::ActuatorTransaction txn{};
  const actuator::WriteDecision plan_decision =
      policy_->plan(cmd, context.lease, context.mode, &txn);
  status_.last_policy_decision = plan_decision;
  if (plan_decision != actuator::WriteDecision::ACCEPT) {
    // Torque is on; the policy would not accept this specific move (permit
    // revoked, envelope budget withdrawn, authority lost between steps,
    // ...). The joint holds its ORIGINAL q0 position but remains energized -
    // the safe default is to force SAFE_OFF, not leave it ambiguously held.
    finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::POSITION_COMMAND_REJECTED,
          plan_decision);
    return;
  }

  actuator::WriteDecision commit_decision = plan_decision;
  const actuator::ExecuteResult result = runtime_->execute(&txn, bus_id_, &commit_decision);
  status_.last_policy_decision = commit_decision;
  switch (result) {
    case actuator::ExecuteResult::WRITTEN:
      deadman_.begin(config_.deadman, target_tick, now_ms);
      status_.state = FirstMotionState::MONITORING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::POSITION_COMMAND_UNCERTAIN,
            commit_decision);
      return;
    case actuator::ExecuteResult::NOT_EXECUTED:
    case actuator::ExecuteResult::NO_BACKEND:
    case actuator::ExecuteResult::NO_RAW_TARGET:
    case actuator::ExecuteResult::BACKEND_REJECTED:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::POSITION_COMMAND_REJECTED,
            commit_decision);
      return;
  }
}

void FirstMotionExecutor::stepMonitoring(uint32_t now_ms, bool telemetry_available,
                                         const actuator::TelemetrySample& telemetry) {
  const actuator::MotionDeadmanVerdict verdict =
      telemetry_available ? deadman_.evaluate(telemetry, now_ms) : deadman_.poll(now_ms);
  switch (verdict) {
    case actuator::MotionDeadmanVerdict::CONTINUE:
      return;
    case actuator::MotionDeadmanVerdict::ARRIVED:
      finish(FirstMotionState::COMPLETE, FirstMotionFailure::NONE,
            actuator::WriteDecision::ACCEPT);
      return;
    case actuator::MotionDeadmanVerdict::STALE_TELEMETRY:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::STALE_TELEMETRY,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::COMMUNICATION_LOST:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::COMMUNICATION_LOST,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF:
      // Torque already reads 0 - SAFE_OFF is still requested (never wrong,
      // idempotent) rather than trusting a single readback as the final word.
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::TORQUE_UNEXPECTEDLY_OFF,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::STALLED:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::STALLED,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::TIMED_OUT:
      finish(FirstMotionState::SAFE_OFF_REQUIRED, FirstMotionFailure::MOTION_TIMEOUT,
            status_.last_policy_decision);
      return;
  }
}

const char* toString(FirstMotionState state) {
  switch (state) {
    case FirstMotionState::IDLE:                     return "IDLE";
    case FirstMotionState::TORQUE_ENABLE_PENDING:     return "TORQUE_ENABLE_PENDING";
    case FirstMotionState::POSITION_COMMAND_PENDING:  return "POSITION_COMMAND_PENDING";
    case FirstMotionState::MONITORING:                return "MONITORING";
    case FirstMotionState::COMPLETE:                  return "COMPLETE";
    case FirstMotionState::FAILED_NO_MOTION:          return "FAILED_NO_MOTION";
    case FirstMotionState::SAFE_OFF_REQUIRED:         return "SAFE_OFF_REQUIRED";
  }
  return "UNKNOWN";
}

const char* toString(FirstMotionFailure failure) {
  switch (failure) {
    case FirstMotionFailure::NONE:                       return "NONE";
    case FirstMotionFailure::REJECT_PRECONDITIONS:       return "REJECT_PRECONDITIONS";
    case FirstMotionFailure::TORQUE_ENABLE_REJECTED:     return "TORQUE_ENABLE_REJECTED";
    case FirstMotionFailure::TORQUE_ENABLE_UNCERTAIN:    return "TORQUE_ENABLE_UNCERTAIN";
    case FirstMotionFailure::REJECT_TARGET_RESOLUTION:   return "REJECT_TARGET_RESOLUTION";
    case FirstMotionFailure::POSITION_COMMAND_REJECTED:  return "POSITION_COMMAND_REJECTED";
    case FirstMotionFailure::POSITION_COMMAND_UNCERTAIN: return "POSITION_COMMAND_UNCERTAIN";
    case FirstMotionFailure::STALE_TELEMETRY:            return "STALE_TELEMETRY";
    case FirstMotionFailure::COMMUNICATION_LOST:         return "COMMUNICATION_LOST";
    case FirstMotionFailure::TORQUE_UNEXPECTEDLY_OFF:    return "TORQUE_UNEXPECTEDLY_OFF";
    case FirstMotionFailure::STALLED:                    return "STALLED";
    case FirstMotionFailure::MOTION_TIMEOUT:              return "MOTION_TIMEOUT";
    case FirstMotionFailure::OPERATOR_ABORT:              return "OPERATOR_ABORT";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
