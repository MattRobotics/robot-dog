#include "CalibrationExecutionEngine.h"

namespace matdog {
namespace calibration {

actuator::ActuatorOperation operationForIntent(CalibrationIntent intent) {
  switch (intent) {
    case CalibrationIntent::CONTACT_PROBE:
      return actuator::ActuatorOperation::CALIBRATION_CONTACT_PROBE;
    case CalibrationIntent::AUXILIARY_MOVE:
      return actuator::ActuatorOperation::CALIBRATION_AUXILIARY_MOVE;
    case CalibrationIntent::DIRECTION_VERIFY:
      return actuator::ActuatorOperation::DIRECTION_VERIFY;
    // Categorically non-executing — see the file comment. Neither of these,
    // nor an unrecognised value, ever maps to something execute() can send.
    case CalibrationIntent::NONE:
    case CalibrationIntent::RESTORE:
    case CalibrationIntent::ABORT:
      return actuator::ActuatorOperation::NONE;
  }
  return actuator::ActuatorOperation::NONE;
}

void CalibrationExecutionEngine::begin(actuator::SafeActuatorPolicy* policy,
                                       actuator::ActuatorRuntime* runtime) {
  policy_ = policy;
  runtime_ = runtime;
}

CalibrationExecutionResult CalibrationExecutionEngine::execute(
    const CalibrationExecutionRequest& request, const CalibrationExecutionContext& context,
    uint8_t bus_id) {
  CalibrationExecutionResult result{};

  // RESTORE and ABORT are categorically non-executing: neither ever builds
  // an ActuatorCommand or touches the policy, which is what makes "authority
  // loss must produce ZERO restore motion" true by construction rather than
  // a checked exception. See RestorePlan (CalibrationDomain.h) — restore is
  // state only, never an action this layer performs.
  if (request.intent == CalibrationIntent::RESTORE) {
    result.outcome = CalibrationExecutionOutcome::RESTORE_ACKNOWLEDGED_NO_MOTION;
    return result;
  }
  if (request.intent == CalibrationIntent::ABORT) {
    result.outcome = CalibrationExecutionOutcome::ABORT_IS_LIFECYCLE_NOT_MOTION;
    return result;
  }

  const actuator::ActuatorOperation operation = operationForIntent(request.intent);
  if (operation == actuator::ActuatorOperation::NONE) {
    // NONE, or a corrupted enum value — fail closed rather than guess.
    result.outcome = CalibrationExecutionOutcome::REJECT_UNKNOWN_INTENT;
    return result;
  }
  if (!context.session_active) {
    result.outcome = CalibrationExecutionOutcome::REJECT_NO_SESSION;
    return result;
  }
  // A replay session authorises nothing physical, however complete it looks
  // — checked here too, in front of the policy call, as defence in depth
  // alongside SafeActuatorPolicy's own bootstrap-context origin check.
  if (context.origin == CalibrationOrigin::HISTORICAL_REPLAY) {
    result.outcome = CalibrationExecutionOutcome::REJECT_REPLAY_ORIGIN;
    return result;
  }

  actuator::ActuatorCommand command{};
  command.operation = operation;
  command.joint = request.joint;
  command.endpoint_leg = request.endpoint_leg;
  command.endpoint_joint = request.endpoint_joint;
  command.endpoint_side = request.endpoint_side;
  command.delta_ticks = request.direction_verify_delta_ticks;
  // target_tick / target_urad deliberately left at their zero default — see
  // the file comment: this class never computes a raw<->q conversion, so
  // every command it builds carries only what SafeActuatorPolicy's own
  // accepted-transform/accepted-limit checks can independently validate.

  actuator::ActuatorTransaction transaction{};
  const actuator::WriteDecision plan_decision =
      policy_ != nullptr ? policy_->plan(command, context.lease, context.mode, &transaction)
                        : actuator::WriteDecision::REJECT_NO_ARBITER;

  result.outcome = CalibrationExecutionOutcome::ROUTED_TO_POLICY;
  result.policy_decision = plan_decision;

  if (plan_decision != actuator::WriteDecision::ACCEPT) {
    result.execute_result = actuator::ExecuteResult::NOT_EXECUTED;
    return result;
  }

  if (runtime_ == nullptr) {
    // ACCEPT was planned but there is no adapter to execute it. Abort
    // rather than leave the policy's one outstanding-transaction slot
    // stuck forever — abort is always safe and final; a new plan is
    // possible again immediately afterward.
    if (policy_ != nullptr) policy_->abort(&transaction);
    result.execute_result = actuator::ExecuteResult::NO_BACKEND;
    return result;
  }

  actuator::WriteDecision commit_decision = plan_decision;
  result.execute_result = runtime_->execute(&transaction, bus_id, &commit_decision);
  result.policy_decision = commit_decision;
  return result;
}

const char* toString(CalibrationIntent intent) {
  switch (intent) {
    case CalibrationIntent::NONE:             return "NONE";
    case CalibrationIntent::CONTACT_PROBE:    return "CONTACT_PROBE";
    case CalibrationIntent::AUXILIARY_MOVE:   return "AUXILIARY_MOVE";
    case CalibrationIntent::DIRECTION_VERIFY: return "DIRECTION_VERIFY";
    case CalibrationIntent::RESTORE:          return "RESTORE";
    case CalibrationIntent::ABORT:            return "ABORT";
  }
  return "UNKNOWN";
}

const char* toString(CalibrationExecutionOutcome outcome) {
  switch (outcome) {
    case CalibrationExecutionOutcome::REJECT_UNKNOWN_INTENT:
      return "REJECT_UNKNOWN_INTENT";
    case CalibrationExecutionOutcome::REJECT_NO_SESSION:
      return "REJECT_NO_SESSION";
    case CalibrationExecutionOutcome::REJECT_REPLAY_ORIGIN:
      return "REJECT_REPLAY_ORIGIN";
    case CalibrationExecutionOutcome::RESTORE_ACKNOWLEDGED_NO_MOTION:
      return "RESTORE_ACKNOWLEDGED_NO_MOTION";
    case CalibrationExecutionOutcome::ABORT_IS_LIFECYCLE_NOT_MOTION:
      return "ABORT_IS_LIFECYCLE_NOT_MOTION";
    case CalibrationExecutionOutcome::ROUTED_TO_POLICY:
      return "ROUTED_TO_POLICY";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
