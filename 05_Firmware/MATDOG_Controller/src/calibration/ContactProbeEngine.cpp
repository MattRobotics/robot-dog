#include "ContactProbeEngine.h"

namespace matdog {
namespace calibration {

namespace {

int32_t absDiff(int32_t a, int32_t b) { return a > b ? a - b : b - a; }

// ST3215 speed/current registers carry direction in bit 15; the magnitude is
// what V25's speed_magnitude() compared. -1 (not read) stays -1.
int32_t magnitude(int32_t raw) { return raw < 0 ? -1 : (raw & 0x7FFF); }

// V25 median(): sorted[len / 2].
uint16_t medianOf(const uint16_t* values, uint8_t count) {
  uint16_t sorted[kSearchBaselineMaxSamples] = {0};
  for (uint8_t i = 0; i < count; ++i) {
    uint16_t v = values[i];
    uint8_t j = i;
    while (j > 0 && sorted[j - 1] > v) {
      sorted[j] = sorted[j - 1];
      --j;
    }
    sorted[j] = v;
  }
  return sorted[count / 2];
}

bool sampleUsable(const actuator::TelemetrySample& s) {
  return s.read_ok && s.present_position >= 0 && s.present_position < 4096 &&
         s.torque_enable >= 0 && s.present_speed >= 0 && s.present_current >= 0 &&
         s.present_temperature >= 0;
}

CalibrationExecutionContext toExecutionContext(const ContactProbeContext& context) {
  CalibrationExecutionContext ctx{};
  ctx.session_active = context.session_active;
  ctx.origin = context.origin;
  ctx.lease = context.lease;
  ctx.mode = context.mode;
  return ctx;
}

bool corridorUsable(const actuator::CalibrationSearchCorridor& c) {
  if (!c.valid() || c.guard_tick >= 4096 || c.entry_tick >= 4096) return false;
  const int32_t entry = actuator::searchDepth(c, c.entry_tick);
  const int32_t contact = actuator::searchDepth(c, c.contact_tick);
  const int32_t guard = actuator::searchDepth(c, c.guard_tick);
  return entry > 0 && entry <= contact && contact <= guard;
}

}  // namespace

uint16_t searchBaselineThreshold(uint16_t median_current, uint16_t mad_current) {
  uint32_t margin = static_cast<uint32_t>(mad_current) * 4u;
  if (margin < 5u) margin = 5u;
  const uint32_t threshold = static_cast<uint32_t>(median_current) + margin;
  return threshold > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(threshold);
}

// --- ContactSearchDetector (V25 HybridContactDetector::observe) -------------

void ContactSearchDetector::begin(const actuator::CalibrationSearchCorridor& corridor,
                                  uint16_t start_position, int32_t acceptance_entry_depth) {
  corridor_ = corridor;
  start_position_ = start_position;
  previous_position_ = start_position;
  acceptance_entry_depth_ = acceptance_entry_depth;
  active_target_ = -1;
  target_samples_seen_ = 0;
  confirming_samples_ = 0;
}

ContactDetectorState ContactSearchDetector::observe(uint16_t position, int32_t speed_magnitude,
                                                    uint16_t commanded_target) {
  // A new target restarts the start-up count and the persistence run.
  if (active_target_ != static_cast<int32_t>(commanded_target)) {
    active_target_ = commanded_target;
    target_samples_seen_ = 0;
    previous_position_ = position;
    confirming_samples_ = 0;
    return ContactDetectorState::FREE_MOTION;
  }
  if (target_samples_seen_ < 255) ++target_samples_seen_;

  const int32_t depth_now = actuator::searchDepth(corridor_, position);
  const int32_t travel = depth_now - actuator::searchDepth(corridor_, start_position_);
  const int32_t progress = depth_now - actuator::searchDepth(corridor_, previous_position_);
  previous_position_ = position;

  // An unread speed (-1) can never count as "low": fail closed toward FREE.
  const bool low_velocity = speed_magnitude >= 0 && speed_magnitude <= kSearchMaxVelocityRaw;
  const bool low_progress = progress <= static_cast<int32_t>(kSearchMaxProgressTicks);
  const bool enough_travel = travel >= static_cast<int32_t>(kSearchMinContactTravelTicks);
  const int32_t goal_error = absDiff(position, commanded_target);
  const bool inside_acceptance =
      depth_now >= acceptance_entry_depth_ &&
      depth_now <= actuator::searchDepth(corridor_, corridor_.guard_tick);
  const int32_t settle_tolerance = inside_acceptance ? kSearchStaticToleranceTicks
                                                     : kSearchOutsideCorridorSettleToleranceTicks;
  const bool target_ahead = actuator::searchDepth(corridor_, commanded_target) > depth_now;

  if (goal_error <= settle_tolerance) {
    confirming_samples_ = 0;
    return ContactDetectorState::FREE_MOTION;
  }
  if (target_samples_seen_ <= kSearchTargetStartupSamples) {
    confirming_samples_ = 0;
    return ContactDetectorState::FREE_MOTION;
  }
  if (enough_travel && low_progress && low_velocity && target_ahead) {
    if (confirming_samples_ < 255) ++confirming_samples_;
    if (confirming_samples_ >= kSearchPersistenceSamples) {
      return inside_acceptance ? ContactDetectorState::CONTACT_CONFIRMED
                               : ContactDetectorState::EARLY_STALL;
    }
    return ContactDetectorState::CONTACT_SUSPECTED;
  }
  confirming_samples_ = 0;
  return ContactDetectorState::FREE_MOTION;
}

// --- ContactProbeEngine -----------------------------------------------------

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
  if (active()) return false;

  if (policy_ == nullptr || runtime_ == nullptr || engine_ == nullptr || geometry_ == nullptr ||
      expected_provenance_ == nullptr || !request.joint.valid() ||
      !request.joint.unitKnown() || request.repeatability_tolerance_ticks == 0 ||
      !corridorUsable(request.corridor)) {
    status_ = ContactProbeStatus{};
    status_.phase = ContactProbePhase::FAILED_NO_MOTION;
    status_.failure = ContactProbeFailure::REJECT_PRECONDITIONS;
    return false;
  }

  request_ = request;
  status_ = ContactProbeStatus{};
  status_.phase = ContactProbePhase::TORQUE_ENABLE_PENDING;
  status_.pass = 1;
  has_good_sample_ = false;
  last_good_ms_ = now_ms;
  started_ms_ = now_ms;
  has_cadence_sample_ = false;
  last_cadence_ms_ = 0;
  last_cadence_position_ = -1;
  step_written_ms_ = now_ms;
  step_ticks_ = 0;
  pass_start_position_ = 0;
  pass_started_ = false;
  baseline_closed_ = false;
  return true;
}

void ContactProbeEngine::update(const ContactProbeContext& context, uint32_t now_ms,
                                bool telemetry_available,
                                const actuator::TelemetrySample& telemetry) {
  if (!active()) return;

  // Every usable sample is recorded whatever the phase, so a pass always
  // starts from where the joint really is.
  if (telemetry_available && sampleUsable(telemetry)) {
    has_good_sample_ = true;
    last_good_ms_ = now_ms;
    status_.last_position = telemetry.present_position;
    status_.last_speed = magnitude(telemetry.present_speed);
    status_.last_current = magnitude(telemetry.present_current);
  }

  switch (status_.phase) {
    case ContactProbePhase::TORQUE_ENABLE_PENDING:
      stepTorqueEnable(context);
      return;
    case ContactProbePhase::STEP_PENDING:
      stepWrite(context, now_ms);
      return;
    case ContactProbePhase::STEP_MONITORING:
      stepMonitor(now_ms, telemetry_available, telemetry);
      return;
    case ContactProbePhase::BACKOFF_PENDING:
      stepBackoffWrite(context, now_ms);
      return;
    case ContactProbePhase::BACKOFF_MONITORING:
      stepBackoffMonitor(now_ms, telemetry_available, telemetry);
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
    case ContactProbePhase::STEP_PENDING:
    case ContactProbePhase::STEP_MONITORING:
    case ContactProbePhase::BACKOFF_PENDING:
    case ContactProbePhase::BACKOFF_MONITORING:
      failSafeOff(ContactProbeFailure::OPERATOR_ABORT);
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

uint16_t ContactProbeEngine::tickAtDepth(int32_t d) const {
  const int32_t tick = static_cast<int32_t>(request_.corridor.home_tick) +
                       static_cast<int32_t>(request_.corridor.probe_sign) * d;
  // 4096 is never a valid GoalPosition: the engine and policy refuse it.
  return (tick < 0 || tick >= 4096) ? static_cast<uint16_t>(4096) : static_cast<uint16_t>(tick);
}

bool ContactProbeEngine::issueTarget(const ContactProbeContext& context, uint16_t target_tick,
                                     actuator::ExecuteResult* result_out) {
  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::CONTACT_PROBE;
  req.joint = request_.joint;
  req.endpoint_leg = request_.endpoint_leg;
  req.endpoint_joint = request_.endpoint_joint;
  req.endpoint_side = request_.endpoint_side;
  req.calibration_search = true;
  req.search_target_tick = target_tick;
  req.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;

  const CalibrationExecutionResult result =
      engine_->execute(req, toExecutionContext(context), request_.bus_id);
  status_.last_policy_decision = result.policy_decision;
  if (result_out != nullptr) *result_out = result.execute_result;
  if (result.outcome != CalibrationExecutionOutcome::ROUTED_TO_POLICY) {
    failSafeOff(ContactProbeFailure::COMMAND_REJECTED);
    return false;
  }
  switch (result.execute_result) {
    case actuator::ExecuteResult::WRITTEN:
      return true;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      failSafeOff(ContactProbeFailure::COMMAND_UNCERTAIN);
      return false;
    default:
      failSafeOff(ContactProbeFailure::COMMAND_REJECTED);
      return false;
  }
}

void ContactProbeEngine::stepTorqueEnable(const ContactProbeContext& context) {
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
      status_.phase = ContactProbePhase::STEP_PENDING;
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

void ContactProbeEngine::beginPass(uint8_t pass, uint16_t start_position) {
  status_.pass = pass;
  pass_started_ = true;
  pass_start_position_ = start_position;
  status_.target_tick = start_position;
  // Pass 2 may accept a contact up to 32 ticks HOME-ward of pass 1's (V25
  // adaptive_contact_acceptance_bounds) - never farther toward the guard.
  int32_t acceptance_entry = depth(request_.corridor.entry_tick);
  if (pass == 2) {
    const int32_t adaptive = depth(status_.pass1_contact_tick) - kSearchAdaptiveScoutTicks;
    if (adaptive < acceptance_entry) acceptance_entry = adaptive;
  }
  detector_.begin(request_.corridor, start_position, acceptance_entry);
}

// Torque is VERIFIED applied from here on: every failure is SAFE_OFF_REQUIRED.
void ContactProbeEngine::stepWrite(const ContactProbeContext& context, uint32_t now_ms) {
  if (!pass_started_) {
    if (!has_good_sample_) {
      if (now_ms - started_ms_ >= kSearchMaxTelemetryAgeMs) {
        failSafeOff(ContactProbeFailure::STALE_TELEMETRY);
      }
      return;
    }
    beginPass(status_.pass, static_cast<uint16_t>(status_.last_position));
  }

  const int32_t target_depth = depth(status_.target_tick);
  const int32_t entry_depth = depth(request_.corridor.entry_tick);
  const int32_t guard_depth = depth(request_.corridor.guard_tick);
  int32_t next_depth = 0;
  ContactSearchStage stage = ContactSearchStage::FINE_SEARCH;
  if (status_.pass == 1 && target_depth < entry_depth) {
    // Free travel toward the corridor: 64-tick steps, the last one clamped
    // onto the entry so fine search always begins exactly there.
    stage = ContactSearchStage::COARSE_TRANSIT;
    step_ticks_ = kSearchCoarseStepTicks;
    next_depth = target_depth + kSearchCoarseStepTicks;
    if (next_depth > entry_depth) next_depth = entry_depth;
  } else {
    step_ticks_ = kSearchFineStepTicks;
    next_depth = target_depth + kSearchFineStepTicks;
  }
  // V25 passed_guard(): the step that would pass the guard is never issued.
  if (next_depth > guard_depth) {
    failSafeOff(ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
    return;
  }

  const uint16_t next = tickAtDepth(next_depth);
  if (!issueTarget(context, next, nullptr)) return;
  status_.target_tick = next;
  status_.stage = stage;
  if (status_.step_count < 0xFFFF) ++status_.step_count;
  step_written_ms_ = now_ms;
  status_.phase = ContactProbePhase::STEP_MONITORING;
}

bool ContactProbeEngine::sampleSafe(const actuator::TelemetrySample& telemetry) {
  if (telemetry.torque_enable == 0) {
    failSafeOff(ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF);
    return false;
  }
  if (magnitude(telemetry.present_current) >= kSearchHardCurrentAbortRaw) {
    failSafeOff(ContactProbeFailure::HARD_CURRENT_ABORT);
    return false;
  }
  if (telemetry.present_temperature > kSearchTemperatureLimitC) {
    failSafeOff(ContactProbeFailure::OVER_TEMPERATURE);
    return false;
  }
  return true;
}

void ContactProbeEngine::stepMonitor(uint32_t now_ms, bool telemetry_available,
                                     const actuator::TelemetrySample& telemetry) {
  if (!telemetry_available) {
    if (now_ms - last_good_ms_ >= kSearchMaxTelemetryAgeMs) {
      failSafeOff(ContactProbeFailure::STALE_TELEMETRY);
    }
    return;
  }
  if (!sampleUsable(telemetry)) {
    if (now_ms - last_good_ms_ >= kSearchMaxTelemetryAgeMs) {
      failSafeOff(ContactProbeFailure::COMMUNICATION_LOST);
    }
    return;
  }
  if (!sampleSafe(telemetry)) return;
  // V25 consumed one observation per 20 ms bus poll; its sample-count rules
  // mean what they meant there only at that cadence.
  if (has_cadence_sample_ && now_ms - last_cadence_ms_ < kSearchSampleIntervalMs) return;

  const uint16_t position = static_cast<uint16_t>(telemetry.present_position);
  const int32_t speed = magnitude(telemetry.present_speed);

  // The moving-current baseline: the first 64 ticks of pass-1 travel.
  if (status_.pass == 1 && !baseline_closed_) {
    if (depth(position) - depth(pass_start_position_) > kSearchBaselineTravelTicks) {
      baseline_closed_ = true;
    } else {
      const bool moving =
          (last_cadence_position_ >= 0 && position != last_cadence_position_) || speed > 0;
      if (moving && status_.baseline_samples < kSearchBaselineMaxSamples) {
        baseline_[status_.baseline_samples++] =
            static_cast<uint16_t>(magnitude(telemetry.present_current));
      }
    }
  }
  has_cadence_sample_ = true;
  last_cadence_ms_ = now_ms;
  last_cadence_position_ = position;

  const int32_t goal_error = absDiff(position, status_.target_tick);
  if (goal_error <= kSearchStaticToleranceTicks) {
    status_.phase = ContactProbePhase::STEP_PENDING;  // reached: take the next step
    return;
  }

  switch (detector_.observe(position, speed, status_.target_tick)) {
    case ContactDetectorState::CONTACT_CONFIRMED:
      onCandidate(position);
      return;
    case ContactDetectorState::EARLY_STALL:
      failSafeOff(ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR);
      return;
    case ContactDetectorState::FREE_MOTION:
    case ContactDetectorState::CONTACT_SUSPECTED:
      break;
  }

  if (now_ms - step_written_ms_ >= kSearchSettleWindowMs) {
    uint16_t tracking_limit = static_cast<uint16_t>(step_ticks_ + 4);
    if (tracking_limit < kSearchTrackingErrorFloorTicks) {
      tracking_limit = kSearchTrackingErrorFloorTicks;
    }
    if (goal_error > tracking_limit) {
      failSafeOff(ContactProbeFailure::TRACKING_FAILED);
    } else {
      status_.phase = ContactProbePhase::STEP_PENDING;  // bounded lag: continue
    }
  }
}

void ContactProbeEngine::onCandidate(uint16_t position) {
  status_.last_candidate_tick = position;
  if (status_.pass == 1) {
    if (status_.baseline_samples < kSearchBaselineMinSamples) {
      failSafeOff(ContactProbeFailure::INSUFFICIENT_BASELINE);
      return;
    }
    const uint16_t median = medianOf(baseline_, status_.baseline_samples);
    uint16_t deviations[kSearchBaselineMaxSamples] = {0};
    for (uint8_t i = 0; i < status_.baseline_samples; ++i) {
      deviations[i] = static_cast<uint16_t>(absDiff(baseline_[i], median));
    }
    status_.baseline_median_current = median;
    status_.baseline_mad_current = medianOf(deviations, status_.baseline_samples);
    status_.pass1_contact_tick = position;
    status_.phase = ContactProbePhase::BACKOFF_PENDING;
    return;
  }

  // Pass 2 must reproduce pass 1's depth: a candidate more than one fine
  // step HOME-ward of it is a friction plateau, stepped past (V25 "friction
  // plateau bypass"). The guard still bounds how far that can go.
  const int32_t lag = depth(status_.pass1_contact_tick) - depth(position);
  if (lag > static_cast<int32_t>(kSearchFineScoutLagToleranceTicks)) {
    if (status_.plateau_bypass_count < 0xFFFF) ++status_.plateau_bypass_count;
    status_.phase = ContactProbePhase::STEP_PENDING;
    return;
  }
  status_.pass2_contact_tick = position;
  if (absDiff(status_.pass1_contact_tick, position) <=
      static_cast<int32_t>(request_.repeatability_tolerance_ticks)) {
    finish(ContactProbePhase::COMPLETE, ContactProbeFailure::NONE,
          actuator::WriteDecision::ACCEPT);
  } else {
    failSafeOff(ContactProbeFailure::REPEATABILITY_FAILED);
  }
}

void ContactProbeEngine::stepBackoffWrite(const ContactProbeContext& context, uint32_t now_ms) {
  const int32_t backoff_depth = depth(status_.pass1_contact_tick) - kSearchBackoffTicks;
  if (backoff_depth < 0) {  // V25 crossed_home()
    failSafeOff(ContactProbeFailure::BACKOFF_CROSSES_HOME);
    return;
  }
  const uint16_t target = tickAtDepth(backoff_depth);
  if (!issueTarget(context, target, nullptr)) return;
  status_.target_tick = target;
  status_.stage = ContactSearchStage::BACKOFF;
  deadman_.begin(config_.backoff_deadman, target, now_ms);
  status_.phase = ContactProbePhase::BACKOFF_MONITORING;
}

void ContactProbeEngine::stepBackoffMonitor(uint32_t now_ms, bool telemetry_available,
                                            const actuator::TelemetrySample& telemetry) {
  actuator::TelemetrySample sample = telemetry;
  const bool usable = telemetry_available && sampleUsable(telemetry);
  if (usable) {
    if (magnitude(telemetry.present_current) >= kSearchHardCurrentAbortRaw) {
      failSafeOff(ContactProbeFailure::HARD_CURRENT_ABORT);
      return;
    }
    if (telemetry.present_temperature > kSearchTemperatureLimitC) {
      failSafeOff(ContactProbeFailure::OVER_TEMPERATURE);
      return;
    }
  } else if (telemetry_available) {
    sample.read_ok = false;  // a partial read is not a position
  }

  const actuator::MotionDeadmanVerdict verdict =
      telemetry_available ? deadman_.evaluate(sample, now_ms) : deadman_.poll(now_ms);
  switch (verdict) {
    case actuator::MotionDeadmanVerdict::CONTINUE:
      return;
    case actuator::MotionDeadmanVerdict::ARRIVED: {
      // V25 backoff_and_verify(): the contact pressure must have released.
      const uint16_t threshold = searchBaselineThreshold(status_.baseline_median_current,
                                                         status_.baseline_mad_current);
      const int32_t current = magnitude(sample.present_current);
      if (current < 0 || current > threshold) {
        failSafeOff(ContactProbeFailure::CURRENT_NOT_RECOVERED);
        return;
      }
      beginPass(2, static_cast<uint16_t>(sample.present_position));
      status_.phase = ContactProbePhase::STEP_PENDING;
      return;
    }
    case actuator::MotionDeadmanVerdict::STALLED:
      failSafeOff(ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);
      return;
    case actuator::MotionDeadmanVerdict::STALE_TELEMETRY:
      failSafeOff(ContactProbeFailure::STALE_TELEMETRY);
      return;
    case actuator::MotionDeadmanVerdict::COMMUNICATION_LOST:
      failSafeOff(ContactProbeFailure::COMMUNICATION_LOST);
      return;
    case actuator::MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF:
      failSafeOff(ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF);
      return;
    case actuator::MotionDeadmanVerdict::TIMED_OUT:
      failSafeOff(ContactProbeFailure::MOTION_TIMEOUT);
      return;
  }
}

ContactWitness ContactProbeEngine::witness() const {
  if (status_.phase != ContactProbePhase::COMPLETE) return ContactWitness{};
  const uint16_t deviation =
      static_cast<uint16_t>(absDiff(status_.pass1_contact_tick, status_.pass2_contact_tick));
  // Two independent fine passes: the single deviation is reported as both
  // bounds rather than fabricating a third, unmeasured one.
  return makeContactWitness(deviation, deviation, request_.repeatability_tolerance_ticks);
}

const char* toString(ContactProbePhase phase) {
  switch (phase) {
    case ContactProbePhase::IDLE:                  return "IDLE";
    case ContactProbePhase::TORQUE_ENABLE_PENDING: return "TORQUE_ENABLE_PENDING";
    case ContactProbePhase::STEP_PENDING:          return "STEP_PENDING";
    case ContactProbePhase::STEP_MONITORING:       return "STEP_MONITORING";
    case ContactProbePhase::BACKOFF_PENDING:       return "BACKOFF_PENDING";
    case ContactProbePhase::BACKOFF_MONITORING:    return "BACKOFF_MONITORING";
    case ContactProbePhase::COMPLETE:              return "COMPLETE";
    case ContactProbePhase::FAILED_NO_MOTION:      return "FAILED_NO_MOTION";
    case ContactProbePhase::SAFE_OFF_REQUIRED:     return "SAFE_OFF_REQUIRED";
  }
  return "UNKNOWN";
}

const char* toString(ContactProbeFailure failure) {
  switch (failure) {
    case ContactProbeFailure::NONE:                            return "NONE";
    case ContactProbeFailure::REJECT_PRECONDITIONS:            return "REJECT_PRECONDITIONS";
    case ContactProbeFailure::TORQUE_ENABLE_REJECTED:          return "TORQUE_ENABLE_REJECTED";
    case ContactProbeFailure::TORQUE_ENABLE_UNCERTAIN:         return "TORQUE_ENABLE_UNCERTAIN";
    case ContactProbeFailure::COMMAND_REJECTED:                return "COMMAND_REJECTED";
    case ContactProbeFailure::COMMAND_UNCERTAIN:               return "COMMAND_UNCERTAIN";
    case ContactProbeFailure::NO_CONTACT_BEFORE_GUARD:         return "NO_CONTACT_BEFORE_GUARD";
    case ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR:    return "EARLY_STALL_OUTSIDE_CORRIDOR";
    case ContactProbeFailure::TRACKING_FAILED:                 return "TRACKING_FAILED";
    case ContactProbeFailure::REPEATABILITY_FAILED:            return "REPEATABILITY_FAILED";
    case ContactProbeFailure::HARD_CURRENT_ABORT:              return "HARD_CURRENT_ABORT";
    case ContactProbeFailure::OVER_TEMPERATURE:                return "OVER_TEMPERATURE";
    case ContactProbeFailure::STALE_TELEMETRY:                 return "STALE_TELEMETRY";
    case ContactProbeFailure::COMMUNICATION_LOST:              return "COMMUNICATION_LOST";
    case ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF:         return "TORQUE_UNEXPECTEDLY_OFF";
    case ContactProbeFailure::INSUFFICIENT_BASELINE:           return "INSUFFICIENT_BASELINE";
    case ContactProbeFailure::BACKOFF_CROSSES_HOME:            return "BACKOFF_CROSSES_HOME";
    case ContactProbeFailure::CURRENT_NOT_RECOVERED:           return "CURRENT_NOT_RECOVERED";
    case ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF:
      return "UNEXPECTED_STALL_DURING_BACKOFF";
    case ContactProbeFailure::MOTION_TIMEOUT:                  return "MOTION_TIMEOUT";
    case ContactProbeFailure::OPERATOR_ABORT:                  return "OPERATOR_ABORT";
  }
  return "UNKNOWN";
}

const char* toString(ContactSearchStage stage) {
  switch (stage) {
    case ContactSearchStage::NONE:           return "NONE";
    case ContactSearchStage::COARSE_TRANSIT: return "COARSE_TRANSIT";
    case ContactSearchStage::FINE_SEARCH:    return "FINE_SEARCH";
    case ContactSearchStage::BACKOFF:        return "BACKOFF";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
