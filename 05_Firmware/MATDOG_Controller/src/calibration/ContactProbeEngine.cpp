#include "ContactProbeEngine.h"

namespace matdog {
namespace calibration {

void ContactProbeEngine::begin(actuator::SafeActuatorPolicy* policy,
                               actuator::ActuatorRuntime* runtime,
                               CalibrationExecutionEngine* engine,
                               const actuator::CalibrationGeometryProfile* geometry,
                               const actuator::GeometryProvenance* expected_provenance,
                               const ContactProbeConfig& config) {
  policy_ = policy;
  runtime_ = runtime;
  engine_ = engine;
  geometry_ = geometry;
  expected_provenance_ = expected_provenance;
  config_ = config;
  status_ = ContactProbeStatus{};
}

bool ContactProbeEngine::start(const ContactProbeRequest& request,
                               const ContactProbeContext& context, uint32_t now_ms) {
  (void)context;
  (void)now_ms;

  if (active()) return false;

  if (policy_ == nullptr || runtime_ == nullptr || engine_ == nullptr || geometry_ == nullptr ||
      expected_provenance_ == nullptr || !request.joint.valid() ||
      !request.joint.unitKnown() || request.repeatability_tolerance_ticks == 0) {
    status_ = ContactProbeStatus{};
    status_.phase = ContactProbePhase::FAILED_NO_MOTION;
    status_.failure = ContactProbeFailure::REJECT_PRECONDITIONS;
    return false;
  }

  request_ = request;
  status_ = ContactProbeStatus{};
  status_.phase = ContactProbePhase::TORQUE_ENABLE_PENDING;
  status_.pass = 1;
  return true;
}

void ContactProbeEngine::update(const ContactProbeContext& context, uint32_t now_ms,
                                bool telemetry_available,
                                const actuator::TelemetrySample& telemetry) {
  switch (status_.phase) {
    case ContactProbePhase::TORQUE_ENABLE_PENDING:
      stepTorqueEnable(context, now_ms);
      return;
    case ContactProbePhase::APPROACH_PENDING:
      stepApproachPending(context, now_ms);
      return;
    case ContactProbePhase::APPROACH_MONITORING:
      stepApproachMonitoring(now_ms, telemetry_available, telemetry);
      return;
    case ContactProbePhase::BACKOFF_PENDING:
      stepBackoffPending(context, now_ms);
      return;
    case ContactProbePhase::BACKOFF_MONITORING:
      stepBackoffMonitoring(now_ms, telemetry_available, telemetry);
      return;
    case ContactProbePhase::IDLE:
    case ContactProbePhase::COMPLETE:
    case ContactProbePhase::FAILED_NO_MOTION:
    case ContactProbePhase::SAFE_OFF_REQUIRED:
      return;
  }
}

void ContactProbeEngine::abort() {
  switch (status_.phase) {
    case ContactProbePhase::TORQUE_ENABLE_PENDING:
      finish(ContactProbePhase::FAILED_NO_MOTION, ContactProbeFailure::OPERATOR_ABORT,
            status_.last_policy_decision);
      return;
    case ContactProbePhase::APPROACH_PENDING:
    case ContactProbePhase::APPROACH_MONITORING:
    case ContactProbePhase::BACKOFF_PENDING:
    case ContactProbePhase::BACKOFF_MONITORING:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::OPERATOR_ABORT,
            status_.last_policy_decision);
      return;
    case ContactProbePhase::IDLE:
    case ContactProbePhase::COMPLETE:
    case ContactProbePhase::FAILED_NO_MOTION:
    case ContactProbePhase::SAFE_OFF_REQUIRED:
      return;
  }
}

void ContactProbeEngine::finish(ContactProbePhase phase, ContactProbeFailure failure,
                                actuator::WriteDecision decision) {
  status_.phase = phase;
  status_.failure = failure;
  status_.last_policy_decision = decision;
}

bool ContactProbeEngine::resolveTarget(actuator::MicroRad target_urad,
                                       uint16_t* raw_tick_out) const {
  const actuator::JointTransform* transform =
      policy_->transforms().find(request_.joint, policy_->currentGeometryTag());
  if (transform == nullptr) return false;
  return actuator::resolveUrdfQToRaw(*geometry_, *expected_provenance_, *transform, target_urad,
                                     raw_tick_out) == actuator::TargetResolveStatus::OK;
}

void ContactProbeEngine::stepTorqueEnable(const ContactProbeContext& context, uint32_t now_ms) {
  (void)now_ms;
  actuator::ActuatorCommand cmd{};
  cmd.operation = actuator::ActuatorOperation::TORQUE_ENABLE;
  cmd.joint = request_.joint;

  actuator::ActuatorTransaction txn{};
  const actuator::WriteDecision plan_decision =
      policy_->plan(cmd, context.lease, context.mode, &txn);
  status_.last_policy_decision = plan_decision;
  if (plan_decision != actuator::WriteDecision::ACCEPT) {
    finish(ContactProbePhase::FAILED_NO_MOTION, ContactProbeFailure::REJECT_PRECONDITIONS,
          plan_decision);
    return;
  }

  actuator::WriteDecision commit_decision = plan_decision;
  const actuator::ExecuteResult result =
      runtime_->execute(&txn, request_.bus_id, &commit_decision);
  status_.last_policy_decision = commit_decision;
  switch (result) {
    case actuator::ExecuteResult::WRITTEN:
      status_.phase = ContactProbePhase::APPROACH_PENDING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::TORQUE_ENABLE_UNCERTAIN,
            commit_decision);
      return;
    case actuator::ExecuteResult::NOT_EXECUTED:
    case actuator::ExecuteResult::NO_BACKEND:
    case actuator::ExecuteResult::NO_RAW_TARGET:
    case actuator::ExecuteResult::BACKEND_REJECTED:
      finish(ContactProbePhase::FAILED_NO_MOTION, ContactProbeFailure::TORQUE_ENABLE_REJECTED,
            commit_decision);
      return;
  }
}

namespace {
CalibrationExecutionContext toExecutionContext(const ContactProbeContext& context) {
  CalibrationExecutionContext ctx{};
  ctx.session_active = context.session_active;
  ctx.origin = context.origin;
  ctx.lease = context.lease;
  ctx.mode = context.mode;
  return ctx;
}
}  // namespace

void ContactProbeEngine::stepApproachPending(const ContactProbeContext& context,
                                             uint32_t now_ms) {
  // Reached only after TorqueEnable was VERIFIED applied - every exit below
  // is therefore SAFE_OFF_REQUIRED, never FAILED_NO_MOTION.
  uint16_t target_tick = 0;
  if (!resolveTarget(request_.approach_target_urad, &target_tick)) {
    finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::REJECT_TARGET_RESOLUTION,
          actuator::WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
    return;
  }

  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::CONTACT_PROBE;
  req.joint = request_.joint;
  req.endpoint_leg = request_.endpoint_leg;
  req.endpoint_joint = request_.endpoint_joint;
  req.endpoint_side = request_.endpoint_side;
  req.target_urad = request_.approach_target_urad;

  const CalibrationExecutionResult result =
      engine_->execute(req, toExecutionContext(context), request_.bus_id);
  status_.last_policy_decision = result.policy_decision;

  if (result.outcome != CalibrationExecutionOutcome::ROUTED_TO_POLICY) {
    finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMAND_REJECTED,
          result.policy_decision);
    return;
  }
  switch (result.execute_result) {
    case actuator::ExecuteResult::WRITTEN:
      deadman_.begin(config_.approach_deadman, target_tick, now_ms);
      status_.phase = ContactProbePhase::APPROACH_MONITORING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMAND_UNCERTAIN,
            result.policy_decision);
      return;
    default:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMAND_REJECTED,
            result.policy_decision);
      return;
  }
}

void ContactProbeEngine::stepApproachMonitoring(uint32_t now_ms, bool telemetry_available,
                                                const actuator::TelemetrySample& telemetry) {
  const actuator::MotionDeadmanVerdict verdict =
      telemetry_available ? deadman_.evaluate(telemetry, now_ms) : deadman_.poll(now_ms);
  switch (verdict) {
    case actuator::MotionDeadmanVerdict::CONTINUE:
      return;

    case actuator::MotionDeadmanVerdict::STALLED: {
      // THE contact signal - see the file comment for why a sustained stall,
      // not a threshold on an uncharacterized current/load register, is
      // what this engine treats as possible contact.
      const uint16_t stalled_at = static_cast<uint16_t>(deadman_.lastProgressPosition());
      if (status_.pass == 1) {
        status_.coarse_tick = stalled_at;
        status_.pass = 2;
        status_.phase = ContactProbePhase::BACKOFF_PENDING;
        return;
      }
      status_.fine_tick_1 = stalled_at;
      const int32_t delta =
          static_cast<int32_t>(status_.fine_tick_1) - static_cast<int32_t>(status_.coarse_tick);
      const int32_t magnitude = delta < 0 ? -delta : delta;
      if (magnitude <= static_cast<int32_t>(request_.repeatability_tolerance_ticks)) {
        finish(ContactProbePhase::COMPLETE, ContactProbeFailure::NONE,
              actuator::WriteDecision::ACCEPT);
      } else {
        finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::REPEATABILITY_FAILED,
              status_.last_policy_decision);
      }
      return;
    }

    case actuator::MotionDeadmanVerdict::ARRIVED:
      // Reached the commanded boundary with no stall ever observed - not
      // contact evidence on either pass; fabricating one is exactly what
      // this engine must not do.
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::NO_CONTACT_DETECTED,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::STALE_TELEMETRY:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::STALE_TELEMETRY,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::COMMUNICATION_LOST:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMUNICATION_LOST,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::TIMED_OUT:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::MOTION_TIMEOUT,
            status_.last_policy_decision);
      return;
  }
}

void ContactProbeEngine::stepBackoffPending(const ContactProbeContext& context,
                                            uint32_t now_ms) {
  uint16_t target_tick = 0;
  if (!resolveTarget(request_.backoff_target_urad, &target_tick)) {
    finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::REJECT_TARGET_RESOLUTION,
          actuator::WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
    return;
  }

  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::CONTACT_PROBE;
  req.joint = request_.joint;
  req.endpoint_leg = request_.endpoint_leg;
  req.endpoint_joint = request_.endpoint_joint;
  req.endpoint_side = request_.endpoint_side;
  req.target_urad = request_.backoff_target_urad;

  const CalibrationExecutionResult result =
      engine_->execute(req, toExecutionContext(context), request_.bus_id);
  status_.last_policy_decision = result.policy_decision;

  if (result.outcome != CalibrationExecutionOutcome::ROUTED_TO_POLICY) {
    finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMAND_REJECTED,
          result.policy_decision);
    return;
  }
  switch (result.execute_result) {
    case actuator::ExecuteResult::WRITTEN:
      deadman_.begin(config_.backoff_deadman, target_tick, now_ms);
      status_.phase = ContactProbePhase::BACKOFF_MONITORING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMAND_UNCERTAIN,
            result.policy_decision);
      return;
    default:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMAND_REJECTED,
            result.policy_decision);
      return;
  }
}

void ContactProbeEngine::stepBackoffMonitoring(uint32_t now_ms, bool telemetry_available,
                                               const actuator::TelemetrySample& telemetry) {
  const actuator::MotionDeadmanVerdict verdict =
      telemetry_available ? deadman_.evaluate(telemetry, now_ms) : deadman_.poll(now_ms);
  switch (verdict) {
    case actuator::MotionDeadmanVerdict::CONTINUE:
      return;
    case actuator::MotionDeadmanVerdict::ARRIVED:
      status_.phase = ContactProbePhase::APPROACH_PENDING;
      return;
    case actuator::MotionDeadmanVerdict::STALLED:
      // Unexpected during backoff: this corridor was already proven clear
      // on the way in, so a stall here is a genuine anomaly, not evidence.
      finish(ContactProbePhase::SAFE_OFF_REQUIRED,
            ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF, status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::STALE_TELEMETRY:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::STALE_TELEMETRY,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::COMMUNICATION_LOST:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::COMMUNICATION_LOST,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF,
            status_.last_policy_decision);
      return;
    case actuator::MotionDeadmanVerdict::TIMED_OUT:
      finish(ContactProbePhase::SAFE_OFF_REQUIRED, ContactProbeFailure::MOTION_TIMEOUT,
            status_.last_policy_decision);
      return;
  }
}

ContactWitness ContactProbeEngine::witness() const {
  if (status_.phase != ContactProbePhase::COMPLETE) return ContactWitness{};
  const uint16_t deviation = status_.coarse_tick > status_.fine_tick_1
                                 ? status_.coarse_tick - status_.fine_tick_1
                                 : status_.fine_tick_1 - status_.coarse_tick;
  // Only one independent repeat exists in this two-pass design (see the file
  // comment) - the single deviation is reported as both bounds rather than
  // fabricating a second, unmeasured one.
  return makeContactWitness(deviation, deviation, request_.repeatability_tolerance_ticks);
}

const char* toString(ContactProbePhase phase) {
  switch (phase) {
    case ContactProbePhase::IDLE:                  return "IDLE";
    case ContactProbePhase::TORQUE_ENABLE_PENDING:  return "TORQUE_ENABLE_PENDING";
    case ContactProbePhase::APPROACH_PENDING:       return "APPROACH_PENDING";
    case ContactProbePhase::APPROACH_MONITORING:    return "APPROACH_MONITORING";
    case ContactProbePhase::BACKOFF_PENDING:        return "BACKOFF_PENDING";
    case ContactProbePhase::BACKOFF_MONITORING:     return "BACKOFF_MONITORING";
    case ContactProbePhase::COMPLETE:               return "COMPLETE";
    case ContactProbePhase::FAILED_NO_MOTION:       return "FAILED_NO_MOTION";
    case ContactProbePhase::SAFE_OFF_REQUIRED:      return "SAFE_OFF_REQUIRED";
  }
  return "UNKNOWN";
}

const char* toString(ContactProbeFailure failure) {
  switch (failure) {
    case ContactProbeFailure::NONE:                     return "NONE";
    case ContactProbeFailure::REJECT_PRECONDITIONS:     return "REJECT_PRECONDITIONS";
    case ContactProbeFailure::TORQUE_ENABLE_REJECTED:   return "TORQUE_ENABLE_REJECTED";
    case ContactProbeFailure::TORQUE_ENABLE_UNCERTAIN:  return "TORQUE_ENABLE_UNCERTAIN";
    case ContactProbeFailure::REJECT_TARGET_RESOLUTION: return "REJECT_TARGET_RESOLUTION";
    case ContactProbeFailure::COMMAND_REJECTED:         return "COMMAND_REJECTED";
    case ContactProbeFailure::COMMAND_UNCERTAIN:        return "COMMAND_UNCERTAIN";
    case ContactProbeFailure::NO_CONTACT_DETECTED:      return "NO_CONTACT_DETECTED";
    case ContactProbeFailure::REPEATABILITY_FAILED:     return "REPEATABILITY_FAILED";
    case ContactProbeFailure::STALE_TELEMETRY:          return "STALE_TELEMETRY";
    case ContactProbeFailure::COMMUNICATION_LOST:       return "COMMUNICATION_LOST";
    case ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF:  return "TORQUE_UNEXPECTEDLY_OFF";
    case ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF:
      return "UNEXPECTED_STALL_DURING_BACKOFF";
    case ContactProbeFailure::MOTION_TIMEOUT:            return "MOTION_TIMEOUT";
    case ContactProbeFailure::OPERATOR_ABORT:            return "OPERATOR_ABORT";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
