#include "ActuatorRuntime.h"

namespace matdog {
namespace actuator {

BackendCallKind backendCallFor(ActuatorOperation operation) {
  switch (operation) {
    case ActuatorOperation::TORQUE_ENABLE:
      return BackendCallKind::ENABLE_TORQUE;
    case ActuatorOperation::POSITION_COMMAND:
      return BackendCallKind::WRITE_GOAL_POSITION;
    case ActuatorOperation::CALIBRATION_CONTACT_PROBE:
    case ActuatorOperation::DIRECTION_VERIFY:
    case ActuatorOperation::CALIBRATION_AUXILIARY_MOVE:
    case ActuatorOperation::NONE:
      return BackendCallKind::NONE;
  }
  return BackendCallKind::NONE;
}

void ActuatorRuntime::begin(SafeActuatorPolicy* policy, ActuatorBackend* backend) {
  policy_ = policy;
  backend_ = backend;
}

ExecuteResult ActuatorRuntime::execute(ActuatorTransaction* transaction, uint8_t bus_id,
                                       WriteDecision* decision_out) {
  const WriteDecision decision =
      policy_ != nullptr ? policy_->commit(transaction) : WriteDecision::REJECT_NO_ARBITER;
  if (decision_out != nullptr) *decision_out = decision;
  if (decision != WriteDecision::ACCEPT) return ExecuteResult::NOT_EXECUTED;
  if (backend_ == nullptr) return ExecuteResult::NO_BACKEND;

  const ActuatorCommand& cmd = transaction->command;
  bool ok = false;
  switch (backendCallFor(cmd.operation)) {
    case BackendCallKind::ENABLE_TORQUE:
      ok = backend_->enableTorque(bus_id);
      break;
    case BackendCallKind::WRITE_GOAL_POSITION:
      ok = backend_->writeGoalPosition(bus_id, cmd.target_tick);
      break;
    case BackendCallKind::NONE:
      return ExecuteResult::NO_RAW_TARGET;
  }
  return ok ? ExecuteResult::WRITTEN : ExecuteResult::BACKEND_REJECTED;
}

const char* toString(ExecuteResult result) {
  switch (result) {
    case ExecuteResult::NOT_EXECUTED:     return "NOT_EXECUTED";
    case ExecuteResult::NO_BACKEND:       return "NO_BACKEND";
    case ExecuteResult::NO_RAW_TARGET:    return "NO_RAW_TARGET";
    case ExecuteResult::WRITTEN:          return "WRITTEN";
    case ExecuteResult::BACKEND_REJECTED: return "BACKEND_REJECTED";
  }
  return "UNKNOWN";
}

const char* toString(BackendCallKind kind) {
  switch (kind) {
    case BackendCallKind::NONE:                return "NONE";
    case BackendCallKind::ENABLE_TORQUE:       return "ENABLE_TORQUE";
    case BackendCallKind::WRITE_GOAL_POSITION: return "WRITE_GOAL_POSITION";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
