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
    case ActuatorOperation::CALIBRATION_SEQUENCE_MOVE:
      return BackendCallKind::WRITE_GOAL_POSITION;
    case ActuatorOperation::CALIBRATION_TORQUE_LIMIT:
      return BackendCallKind::WRITE_CALIBRATION_TORQUE_LIMIT;
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
  BackendWriteOutcome outcome = BackendWriteOutcome::UNCERTAIN;
  switch (backendCallFor(cmd.operation)) {
    case BackendCallKind::ENABLE_TORQUE:
      outcome = backend_->enableTorque(bus_id);
      break;
    case BackendCallKind::WRITE_GOAL_POSITION:
      outcome = backend_->writeGoalPosition(bus_id, cmd.target_tick, cmd.motion_profile);
      break;
    case BackendCallKind::WRITE_CALIBRATION_TORQUE_LIMIT:
      outcome = backend_->writeCalibrationTorqueLimit(bus_id);
      break;
    case BackendCallKind::NONE:
      return ExecuteResult::NO_RAW_TARGET;
  }
  switch (outcome) {
    case BackendWriteOutcome::VERIFIED_APPLIED:     return ExecuteResult::WRITTEN;
    case BackendWriteOutcome::VERIFIED_NOT_APPLIED: return ExecuteResult::BACKEND_REJECTED;
    case BackendWriteOutcome::UNCERTAIN:            return ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF;
  }
  // Fail closed on a corrupted enum value exactly like isKnownOperation():
  // an outcome this code does not recognise must never be read as success.
  return ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF;
}

FrameExecuteResult ActuatorRuntime::executeFrame(ActuatorFrameTransaction* transaction,
                                                 const uint8_t bus_ids[12],
                                                 WriteDecision* decision_out,
                                                 ActuatorBackend::FrameTxReport* report) {
  if (report != nullptr) *report = ActuatorBackend::FrameTxReport{};
  // Bus ids are checked BEFORE the commit so a bad allocation consumes the
  // transaction without a byte: twelve distinct members of the canonical set.
  bool ids_ok = bus_ids != nullptr && canonical_ids_ != nullptr && canonical_count_ == 12;
  for (unsigned i = 0; ids_ok && i < 12; ++i) {
    bool member = false;
    for (unsigned k = 0; k < canonical_count_; ++k) member = member || canonical_ids_[k] == bus_ids[i];
    for (unsigned j = 0; j < i; ++j) ids_ok = ids_ok && bus_ids[j] != bus_ids[i];
    ids_ok = ids_ok && member;
  }
  if (!ids_ok) {
    if (policy_ != nullptr) policy_->abortFrame(transaction);
    if (decision_out != nullptr) *decision_out = WriteDecision::REJECT_FRAME_SHAPE;
    return FrameExecuteResult::INVALID_BUS_IDS;
  }
  const WriteDecision decision =
      policy_ != nullptr ? policy_->commitFrame(transaction) : WriteDecision::REJECT_NO_ARBITER;
  if (decision_out != nullptr) *decision_out = decision;
  if (decision != WriteDecision::ACCEPT) return FrameExecuteResult::NOT_EXECUTED;
  if (backend_ == nullptr) return FrameExecuteResult::NO_BACKEND;
  uint16_t ticks[12];
  for (unsigned i = 0; i < 12; ++i) ticks[i] = transaction->frame.joints[i].target_tick;
  ActuatorBackend::FrameTxReport local{};
  ActuatorBackend::FrameTxReport* r = report != nullptr ? report : &local;
  const BackendWriteOutcome outcome = backend_->writeGoalPositionFrame(bus_ids, ticks, r);
  if (outcome == BackendWriteOutcome::VERIFIED_NOT_APPLIED && !r->tx_started) return FrameExecuteResult::NOT_SENT;
  if (outcome == BackendWriteOutcome::UNCERTAIN && r->tx_started && r->tx_complete) return FrameExecuteResult::TX_COMPLETE_UNCONFIRMED;
  // Anything else - a started but incomplete TX, a backend claiming APPLIED
  // for a frame it cannot confirm, a corrupted value - is uncertain.
  return FrameExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF;
}

const char* toString(FrameExecuteResult result) {
  switch (result) {
    case FrameExecuteResult::NOT_EXECUTED:            return "NOT_EXECUTED";
    case FrameExecuteResult::NO_BACKEND:              return "NO_BACKEND";
    case FrameExecuteResult::INVALID_BUS_IDS:         return "INVALID_BUS_IDS";
    case FrameExecuteResult::NOT_SENT:                return "NOT_SENT";
    case FrameExecuteResult::TX_COMPLETE_UNCONFIRMED: return "TX_COMPLETE_UNCONFIRMED";
    case FrameExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF: return "UNCERTAIN_REQUIRES_SAFE_OFF";
  }
  return "UNKNOWN";
}

const char* toString(ExecuteResult result) {
  switch (result) {
    case ExecuteResult::NOT_EXECUTED:     return "NOT_EXECUTED";
    case ExecuteResult::NO_BACKEND:       return "NO_BACKEND";
    case ExecuteResult::NO_RAW_TARGET:    return "NO_RAW_TARGET";
    case ExecuteResult::WRITTEN:          return "WRITTEN";
    case ExecuteResult::BACKEND_REJECTED: return "BACKEND_REJECTED";
    case ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF: return "UNCERTAIN_REQUIRES_SAFE_OFF";
  }
  return "UNKNOWN";
}

const char* toString(BackendWriteOutcome outcome) {
  switch (outcome) {
    case BackendWriteOutcome::VERIFIED_APPLIED:     return "VERIFIED_APPLIED";
    case BackendWriteOutcome::VERIFIED_NOT_APPLIED: return "VERIFIED_NOT_APPLIED";
    case BackendWriteOutcome::UNCERTAIN:            return "UNCERTAIN";
  }
  return "UNKNOWN";
}

const char* toString(BackendCallKind kind) {
  switch (kind) {
    case BackendCallKind::NONE:                return "NONE";
    case BackendCallKind::ENABLE_TORQUE:       return "ENABLE_TORQUE";
    case BackendCallKind::WRITE_GOAL_POSITION: return "WRITE_GOAL_POSITION";
    case BackendCallKind::WRITE_CALIBRATION_TORQUE_LIMIT:
      return "WRITE_CALIBRATION_TORQUE_LIMIT";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
