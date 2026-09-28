#include "FullLegCalibrationExecutor.h"

namespace matdog {
namespace calibration {

namespace {

ContactProbeContext toContactProbeContext(const FullLegCalibrationContext& context) {
  ContactProbeContext out{};
  out.session_active = context.session_active;
  out.origin = context.origin;
  out.lease = context.lease;
  out.mode = context.mode;
  return out;
}

ContactEvidence buildEvidence(const ContactProfileKey& key, const ContactProbeEngine& probe) {
  ContactEvidence evidence{};
  evidence.key = key;
  evidence.state = EvidenceState::PROMOTED;
  evidence.origin = CalibrationOrigin::LIVE_SESSION;
  evidence.detection = ContactState::CONTACT_CONFIRMED;
  evidence.witness = probe.witness();
  evidence.coarse_tick = probe.status().coarse_tick;
  evidence.fine_tick_1 = probe.status().fine_tick_1;
  // Only one independent repeat exists in ContactProbeEngine's two-pass
  // design (see its own file comment) - reported as both the second AND
  // third measurement rather than fabricating an unmeasured one.
  evidence.fine_tick_2 = probe.status().fine_tick_1;
  evidence.repeatability_ticks = evidence.witness.max_deviation_ticks;
  evidence.has_measurement = true;
  return evidence;
}

}  // namespace

void FullLegCalibrationExecutor::begin(actuator::SafeActuatorPolicy* policy,
                                       actuator::ActuatorRuntime* runtime,
                                       CalibrationExecutionEngine* engine,
                                       const actuator::CalibrationGeometryProfile* geometry,
                                       const actuator::GeometryProvenance* expected_provenance,
                                       const FullLegCalibrationConfig& config) {
  policy_ = policy;
  runtime_ = runtime;
  engine_ = engine;
  geometry_ = geometry;
  expected_provenance_ = expected_provenance;
  config_ = config;
  status_ = FullLegCalibrationStatus{};
  min_evidence_ = ContactEvidence{};
  max_evidence_ = ContactEvidence{};
}

bool FullLegCalibrationExecutor::start(const FullLegCalibrationRequest& request,
                                       const FullLegCalibrationContext& context,
                                       uint32_t now_ms) {
  if (active()) return false;

  if (policy_ == nullptr || runtime_ == nullptr || engine_ == nullptr || geometry_ == nullptr ||
      expected_provenance_ == nullptr || !request.probe_joint.valid() ||
      !request.probe_joint.unitKnown() || !request.auxiliary_joint.valid() ||
      !request.auxiliary_joint.unitKnown() || request.probe_bus_id == 0 ||
      request.auxiliary_bus_id == 0 || request.probe_bus_id == request.auxiliary_bus_id ||
      request.min_repeatability_tolerance_ticks == 0 ||
      request.max_repeatability_tolerance_ticks == 0) {
    request_ = request;
    status_ = FullLegCalibrationStatus{};
    status_.phase = FullLegCalibrationPhase::FAILED;
    status_.failure = FullLegCalibrationFailure::REJECT_PRECONDITIONS;
    return false;
  }

  request_ = request;
  status_ = FullLegCalibrationStatus{};
  min_evidence_ = ContactEvidence{};
  max_evidence_ = ContactEvidence{};

  ContactProbeConfig probe_config{};
  probe_config.approach_deadman = config_.probe_approach_deadman;
  probe_config.backoff_deadman = config_.probe_backoff_deadman;
  probe_.begin(policy_, runtime_, engine_, geometry_, expected_provenance_, probe_config);

  ContactProbeRequest probe_request{};
  probe_request.joint = request_.probe_joint;
  probe_request.bus_id = request_.probe_bus_id;
  probe_request.endpoint_leg = request_.endpoint_leg;
  probe_request.endpoint_joint = request_.endpoint_joint;
  probe_request.endpoint_side = ContactSide::MIN_SIDE;
  probe_request.approach_target_urad = request_.min_approach_urad;
  probe_request.backoff_target_urad = request_.min_backoff_urad;
  probe_request.repeatability_tolerance_ticks = request_.min_repeatability_tolerance_ticks;

  if (!probe_.start(probe_request, toContactProbeContext(context), now_ms)) {
    status_.phase = FullLegCalibrationPhase::FAILED;
    status_.failure = FullLegCalibrationFailure::REJECT_PRECONDITIONS;
    return false;
  }

  status_.phase = FullLegCalibrationPhase::UPPER_MIN_PROBE;
  return true;
}

void FullLegCalibrationExecutor::routeToSafeOffAfterLoss() {
  if (status_.phase == FullLegCalibrationPhase::UPPER_MIN_PROBE) {
    status_.phase = FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF;
  } else if (status_.phase != FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF &&
             status_.phase != FullLegCalibrationPhase::FINAL_SAFE_OFF) {
    status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
  }
  // Already servicing a SAFE_OFF phase: stay: the failure is now recorded,
  // so completion of that phase routes to FAILED instead of continuing.
}

void FullLegCalibrationExecutor::update(const FullLegCalibrationContext& context,
                                        uint32_t now_ms, bool telemetry_available,
                                        const actuator::TelemetrySample& telemetry,
                                        bool primary_safe_off_verified,
                                        bool auxiliary_safe_off_verified) {
  if (!active()) return;

  // Dynamic prerequisites are re-checked on every tick, including while an
  // owned ContactProbeEngine sub-probe is mid-flight - see the file comment.
  const bool continuation_ok =
      context.session_active && context.origin == CalibrationOrigin::LIVE_SESSION &&
      context.mode == core::OperatingMode::MAINTENANCE && context.motion_permit_active &&
      context.lease.valid() && context.lease.owner == core::ActuatorAuthority::CALIBRATION &&
      context.authority == core::ActuatorAuthority::CALIBRATION &&
      context.authority_generation == context.lease.generation && !context.authority_inhibited;

  if (!continuation_ok) {
    if (status_.phase == FullLegCalibrationPhase::UPPER_MIN_PROBE ||
        status_.phase == FullLegCalibrationPhase::UPPER_MAX_PROBE) {
      probe_.abort();
    }
    if (status_.failure == FullLegCalibrationFailure::NONE) {
      status_.failure = FullLegCalibrationFailure::DYNAMIC_PREREQUISITE_LOST;
    }
    routeToSafeOffAfterLoss();
    // Fall through: a SAFE_OFF-servicing phase we just entered (or already
    // occupied) still gets serviced this same tick, below.
  }

  switch (status_.phase) {
    case FullLegCalibrationPhase::UPPER_MIN_PROBE:
      probe_.update(toContactProbeContext(context), now_ms, telemetry_available, telemetry);
      if (!probe_.active()) handleMinProbeTerminal();
      return;
    case FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF:
      if (primary_safe_off_verified) {
        status_.phase = (status_.failure == FullLegCalibrationFailure::NONE)
                            ? FullLegCalibrationPhase::AUX_TORQUE_ENABLE
                            : FullLegCalibrationPhase::FAILED;
      }
      return;
    case FullLegCalibrationPhase::AUX_TORQUE_ENABLE:
      stepAuxTorqueEnable(context);
      return;
    case FullLegCalibrationPhase::AUX_MOVE_PENDING:
      stepAuxMovePending(context, now_ms);
      return;
    case FullLegCalibrationPhase::AUX_MOVE_MONITORING:
      stepAuxMoveMonitoring(context, now_ms, telemetry_available, telemetry);
      return;
    case FullLegCalibrationPhase::UPPER_MAX_PROBE:
      probe_.update(toContactProbeContext(context), now_ms, telemetry_available, telemetry);
      if (!probe_.active()) handleMaxProbeTerminal();
      return;
    case FullLegCalibrationPhase::FINAL_SAFE_OFF:
      if (primary_safe_off_verified && auxiliary_safe_off_verified) {
        status_.phase = (status_.failure == FullLegCalibrationFailure::NONE)
                            ? FullLegCalibrationPhase::COMPLETE
                            : FullLegCalibrationPhase::FAILED;
      }
      return;
    case FullLegCalibrationPhase::IDLE:
    case FullLegCalibrationPhase::COMPLETE:
    case FullLegCalibrationPhase::FAILED:
      return;
  }
}

void FullLegCalibrationExecutor::handleMinProbeTerminal() {
  status_.last_policy_decision = probe_.status().last_policy_decision;
  if (probe_.status().phase == ContactProbePhase::COMPLETE) {
    ContactProfileKey key{};
    key.leg = request_.endpoint_leg;
    key.joint = request_.endpoint_joint;
    key.side = ContactSide::MIN_SIDE;
    min_evidence_ = buildEvidence(key, probe_);
  } else {
    status_.failure = FullLegCalibrationFailure::UPPER_MIN_PROBE_FAILED;
  }
  status_.phase = FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF;
}

void FullLegCalibrationExecutor::handleMaxProbeTerminal() {
  status_.last_policy_decision = probe_.status().last_policy_decision;
  if (probe_.status().phase == ContactProbePhase::COMPLETE) {
    ContactProfileKey key{};
    key.leg = request_.endpoint_leg;
    key.joint = request_.endpoint_joint;
    key.side = ContactSide::MAX_SIDE;
    max_evidence_ = buildEvidence(key, probe_);
  } else {
    status_.failure = FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED;
  }
  status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
}

void FullLegCalibrationExecutor::stepAuxTorqueEnable(const FullLegCalibrationContext& context) {
  actuator::ActuatorCommand cmd{};
  cmd.operation = actuator::ActuatorOperation::TORQUE_ENABLE;
  cmd.joint = request_.auxiliary_joint;

  actuator::ActuatorTransaction txn{};
  const actuator::WriteDecision plan_decision =
      policy_->plan(cmd, context.lease, context.mode, &txn);
  status_.last_policy_decision = plan_decision;
  if (plan_decision != actuator::WriteDecision::ACCEPT) {
    status_.failure = FullLegCalibrationFailure::AUX_TORQUE_ENABLE_REJECTED;
    status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
    return;
  }

  actuator::WriteDecision commit_decision = plan_decision;
  const actuator::ExecuteResult result =
      runtime_->execute(&txn, request_.auxiliary_bus_id, &commit_decision);
  status_.last_policy_decision = commit_decision;
  switch (result) {
    case actuator::ExecuteResult::WRITTEN:
      status_.phase = FullLegCalibrationPhase::AUX_MOVE_PENDING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      status_.failure = FullLegCalibrationFailure::AUX_TORQUE_ENABLE_UNCERTAIN;
      status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
      return;
    case actuator::ExecuteResult::NOT_EXECUTED:
    case actuator::ExecuteResult::NO_BACKEND:
    case actuator::ExecuteResult::NO_RAW_TARGET:
    case actuator::ExecuteResult::BACKEND_REJECTED:
      status_.failure = FullLegCalibrationFailure::AUX_TORQUE_ENABLE_REJECTED;
      status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
      return;
  }
}

void FullLegCalibrationExecutor::stepAuxMovePending(const FullLegCalibrationContext& context,
                                                     uint32_t now_ms) {
  const actuator::JointTransform* transform =
      policy_->transforms().find(request_.auxiliary_joint, policy_->currentGeometryTag());
  uint16_t target_tick = 0;
  if (transform == nullptr ||
      actuator::resolveUrdfQToRaw(*geometry_, *expected_provenance_, *transform,
                                  request_.auxiliary_park_target_urad,
                                  &target_tick) != actuator::TargetResolveStatus::OK) {
    status_.failure = FullLegCalibrationFailure::AUX_MOVE_TARGET_RESOLUTION;
    status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
    return;
  }

  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::AUXILIARY_MOVE;
  req.joint = request_.auxiliary_joint;
  req.endpoint_leg = request_.endpoint_leg;
  req.endpoint_joint = request_.endpoint_joint;
  req.endpoint_side = ContactSide::MAX_SIDE;
  req.target_urad = request_.auxiliary_park_target_urad;

  CalibrationExecutionContext exec_ctx{};
  exec_ctx.session_active = context.session_active;
  exec_ctx.origin = context.origin;
  exec_ctx.lease = context.lease;
  exec_ctx.mode = context.mode;

  const CalibrationExecutionResult result =
      engine_->execute(req, exec_ctx, request_.auxiliary_bus_id);
  status_.last_policy_decision = result.policy_decision;
  if (result.outcome != CalibrationExecutionOutcome::ROUTED_TO_POLICY) {
    status_.failure = FullLegCalibrationFailure::AUX_MOVE_REJECTED;
    status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
    return;
  }
  switch (result.execute_result) {
    case actuator::ExecuteResult::WRITTEN:
      aux_deadman_.begin(config_.aux_move_deadman, target_tick, now_ms);
      status_.phase = FullLegCalibrationPhase::AUX_MOVE_MONITORING;
      return;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_UNCERTAIN;
      status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
      return;
    default:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_REJECTED;
      status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
      return;
  }
}

void FullLegCalibrationExecutor::stepAuxMoveMonitoring(const FullLegCalibrationContext& context,
                                                        uint32_t now_ms, bool telemetry_available,
                                                        const actuator::TelemetrySample& telemetry) {
  const actuator::MotionDeadmanVerdict verdict =
      telemetry_available ? aux_deadman_.evaluate(telemetry, now_ms) : aux_deadman_.poll(now_ms);
  switch (verdict) {
    case actuator::MotionDeadmanVerdict::CONTINUE:
      return;
    case actuator::MotionDeadmanVerdict::ARRIVED: {
      // The auxiliary now holds its parked pose (torque stays ON - that IS
      // the park); start the SAME owned probe_ again, this time for the
      // MAX side. probe_ is terminal (COMPLETE) from the MIN side, so
      // active() is false and start() is callable.
      ContactProbeRequest max_request{};
      max_request.joint = request_.probe_joint;
      max_request.bus_id = request_.probe_bus_id;
      max_request.endpoint_leg = request_.endpoint_leg;
      max_request.endpoint_joint = request_.endpoint_joint;
      max_request.endpoint_side = ContactSide::MAX_SIDE;
      max_request.approach_target_urad = request_.max_approach_urad;
      max_request.backoff_target_urad = request_.max_backoff_urad;
      max_request.repeatability_tolerance_ticks = request_.max_repeatability_tolerance_ticks;
      if (!probe_.start(max_request, toContactProbeContext(context), now_ms)) {
        status_.failure = FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED;
        status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
        return;
      }
      status_.phase = FullLegCalibrationPhase::UPPER_MAX_PROBE;
      return;
    }
    case actuator::MotionDeadmanVerdict::STALLED:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_STALLED;
      break;
    case actuator::MotionDeadmanVerdict::STALE_TELEMETRY:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_STALE_TELEMETRY;
      break;
    case actuator::MotionDeadmanVerdict::COMMUNICATION_LOST:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_COMMUNICATION_LOST;
      break;
    case actuator::MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_TORQUE_UNEXPECTEDLY_OFF;
      break;
    case actuator::MotionDeadmanVerdict::TIMED_OUT:
      status_.failure = FullLegCalibrationFailure::AUX_MOVE_TIMEOUT;
      break;
  }
  status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
}

void FullLegCalibrationExecutor::abort() {
  if (!active()) return;
  if (status_.failure == FullLegCalibrationFailure::NONE) {
    status_.failure = FullLegCalibrationFailure::OPERATOR_ABORT;
  }
  switch (status_.phase) {
    case FullLegCalibrationPhase::UPPER_MIN_PROBE:
      probe_.abort();
      status_.phase = FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF;
      return;
    case FullLegCalibrationPhase::UPPER_MAX_PROBE:
      probe_.abort();
      status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
      return;
    case FullLegCalibrationPhase::AUX_TORQUE_ENABLE:
    case FullLegCalibrationPhase::AUX_MOVE_PENDING:
    case FullLegCalibrationPhase::AUX_MOVE_MONITORING:
      status_.phase = FullLegCalibrationPhase::FINAL_SAFE_OFF;
      return;
    case FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF:
    case FullLegCalibrationPhase::FINAL_SAFE_OFF:
      return;  // already servicing SAFE_OFF; the recorded failure alone
                // is enough to route to FAILED once it completes
    case FullLegCalibrationPhase::IDLE:
    case FullLegCalibrationPhase::COMPLETE:
    case FullLegCalibrationPhase::FAILED:
      return;
  }
}

const char* toString(FullLegCalibrationPhase phase) {
  switch (phase) {
    case FullLegCalibrationPhase::IDLE:                return "IDLE";
    case FullLegCalibrationPhase::UPPER_MIN_PROBE:     return "UPPER_MIN_PROBE";
    case FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF:  return "UPPER_MIN_SAFE_OFF";
    case FullLegCalibrationPhase::AUX_TORQUE_ENABLE:   return "AUX_TORQUE_ENABLE";
    case FullLegCalibrationPhase::AUX_MOVE_PENDING:    return "AUX_MOVE_PENDING";
    case FullLegCalibrationPhase::AUX_MOVE_MONITORING: return "AUX_MOVE_MONITORING";
    case FullLegCalibrationPhase::UPPER_MAX_PROBE:     return "UPPER_MAX_PROBE";
    case FullLegCalibrationPhase::FINAL_SAFE_OFF:      return "FINAL_SAFE_OFF";
    case FullLegCalibrationPhase::COMPLETE:            return "COMPLETE";
    case FullLegCalibrationPhase::FAILED:              return "FAILED";
  }
  return "UNKNOWN";
}

const char* toString(FullLegCalibrationFailure failure) {
  switch (failure) {
    case FullLegCalibrationFailure::NONE:                       return "NONE";
    case FullLegCalibrationFailure::REJECT_PRECONDITIONS:       return "REJECT_PRECONDITIONS";
    case FullLegCalibrationFailure::UPPER_MIN_PROBE_FAILED:     return "UPPER_MIN_PROBE_FAILED";
    case FullLegCalibrationFailure::AUX_TORQUE_ENABLE_REJECTED:
      return "AUX_TORQUE_ENABLE_REJECTED";
    case FullLegCalibrationFailure::AUX_TORQUE_ENABLE_UNCERTAIN:
      return "AUX_TORQUE_ENABLE_UNCERTAIN";
    case FullLegCalibrationFailure::AUX_MOVE_REJECTED:          return "AUX_MOVE_REJECTED";
    case FullLegCalibrationFailure::AUX_MOVE_UNCERTAIN:         return "AUX_MOVE_UNCERTAIN";
    case FullLegCalibrationFailure::AUX_MOVE_TARGET_RESOLUTION:
      return "AUX_MOVE_TARGET_RESOLUTION";
    case FullLegCalibrationFailure::AUX_MOVE_STALE_TELEMETRY:   return "AUX_MOVE_STALE_TELEMETRY";
    case FullLegCalibrationFailure::AUX_MOVE_COMMUNICATION_LOST:
      return "AUX_MOVE_COMMUNICATION_LOST";
    case FullLegCalibrationFailure::AUX_MOVE_TORQUE_UNEXPECTEDLY_OFF:
      return "AUX_MOVE_TORQUE_UNEXPECTEDLY_OFF";
    case FullLegCalibrationFailure::AUX_MOVE_STALLED:           return "AUX_MOVE_STALLED";
    case FullLegCalibrationFailure::AUX_MOVE_TIMEOUT:           return "AUX_MOVE_TIMEOUT";
    case FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED:     return "UPPER_MAX_PROBE_FAILED";
    case FullLegCalibrationFailure::DYNAMIC_PREREQUISITE_LOST:
      return "DYNAMIC_PREREQUISITE_LOST";
    case FullLegCalibrationFailure::OPERATOR_ABORT:             return "OPERATOR_ABORT";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
