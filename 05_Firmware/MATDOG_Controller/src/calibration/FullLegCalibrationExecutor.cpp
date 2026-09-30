#include "FullLegCalibrationExecutor.h"

namespace matdog {
namespace calibration {

namespace {

int32_t absDiff(int32_t a, int32_t b) { return a > b ? a - b : b - a; }

// ST3215 speed/current carry direction in bit 15; -1 (not read) stays -1.
int32_t magnitude(int32_t raw) { return raw < 0 ? -1 : (raw & 0x7FFF); }

constexpr uint8_t kSlotHip = static_cast<uint8_t>(JointKind::HIP);
constexpr uint8_t kSlotUpper = static_cast<uint8_t>(JointKind::UPPER);
constexpr uint8_t kSlotLower = static_cast<uint8_t>(JointKind::LOWER);

constexpr uint8_t kRestPreflight = 0;
constexpr uint8_t kRestFinal = 1;

// INITIAL_RECOVERY sub-states.
enum RecoverSub : uint8_t {
  R_INSPECT = 0,
  R_LIMIT = 1,
  R_TORQUE = 2,
  R_VERIFY = 3,
  R_MOVE = 4,
  R_SAFE_OFF = 5,
  R_FINAL = 6,
};

ContactProbeContext toProbeContext(const FullLegCalibrationContext& c) {
  ContactProbeContext out{};
  out.session_active = c.session_active;
  out.origin = c.origin;
  out.lease = c.lease;
  out.mode = c.mode;
  return out;
}

CalibrationExecutionContext toExecutionContext(const FullLegCalibrationContext& c) {
  CalibrationExecutionContext out{};
  out.session_active = c.session_active;
  out.origin = c.origin;
  out.lease = c.lease;
  out.mode = c.mode;
  return out;
}

// Ticks for a URDF q in micro-radians (4096 ticks per revolution), rounded.
int32_t uradToTicks(actuator::MicroRad q) {
  const int64_t num = static_cast<int64_t>(q) * 4096;
  const int64_t den = static_cast<int64_t>(actuator::kMicroRadPerRevolution);
  return static_cast<int32_t>(num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den));
}

// V25 circular_midpoint_tick() without wrap: both contacts lie in 0..4095 and
// within the repeatability band of each other.
uint16_t midpoint(uint16_t a, uint16_t b) {
  return static_cast<uint16_t>((static_cast<int32_t>(a) + static_cast<int32_t>(b)) / 2);
}

}  // namespace

// --- frames -----------------------------------------------------------------

const actuator::TelemetrySample* FullLegTelemetryFrame::find(uint8_t bus) const {
  if (bus == 0) return nullptr;
  for (uint8_t i = 0; i < count; ++i) {
    if (bus_id[i] == bus) return &sample[i];
  }
  return nullptr;
}

bool FullLegTelemetryFrame::add(uint8_t bus, const actuator::TelemetrySample& s) {
  if (bus == 0 || count >= kFullLegMaxTelemetry || find(bus) != nullptr) return false;
  bus_id[count] = bus;
  sample[count] = s;
  ++count;
  return true;
}

bool FullLegSafeOffFrame::verifiedFor(uint8_t bus) const {
  if (bus == 0) return false;
  for (uint8_t i = 0; i < count; ++i) {
    if (bus_id[i] == bus) return verified[i];
  }
  return false;
}

bool FullLegSafeOffFrame::add(uint8_t bus, bool ok) {
  if (bus == 0 || count >= kFullLegPopulation) return false;
  bus_id[count] = bus;
  verified[count] = ok;
  ++count;
  return true;
}

// --- diagnostics (V25 derive_joint_evidence) ----------------------------------

FullLegJointDiagnostics deriveFullLegJointDiagnostics(const FullLegCalibrationRequest& request,
                                                      JointKind joint,
                                                      const ContactEvidence& min_side,
                                                      const ContactEvidence& max_side) {
  FullLegJointDiagnostics d{};
  const uint8_t k = static_cast<uint8_t>(joint);
  if (k >= kJointKindCount || !min_side.has_measurement || !max_side.has_measurement) return d;
  const int32_t dir = request.direction[k];
  if (dir != 1 && dir != -1) return d;
  d.evaluated = true;
  // The fine contact of each side: midpoint of the two accepted passes.
  d.min_contact_tick = midpoint(min_side.coarse_tick, min_side.fine_tick_1);
  d.max_contact_tick = midpoint(max_side.coarse_tick, max_side.fine_tick_1);
  const int32_t min_delta = uradToTicks(request.urdf_lower[k]);  // URDF limits as the model
  const int32_t max_delta = uradToTicks(request.urdf_upper[k]);  // endpoints, as V25 did
  const int32_t expected = max_delta - min_delta;
  const int32_t measured =
      (static_cast<int32_t>(d.max_contact_tick) - static_cast<int32_t>(d.min_contact_tick)) * dir;
  d.ordered = measured > 0;
  d.expected_span_ticks = static_cast<uint16_t>(expected > 0 ? expected : 0);
  d.measured_span_ticks = static_cast<uint16_t>(measured > 0 ? measured : 0);
  if (expected > 0 && measured > 0) {
    d.scale_permille = static_cast<uint16_t>((measured * 1000 + expected / 2) / expected);
    // affine zero: MIN contact + dir * (-min_delta) * measured / expected.
    const int32_t num = -min_delta * measured;
    const int32_t zero_distance = (num + expected / 2) / expected;
    int32_t zero = static_cast<int32_t>(d.min_contact_tick) + dir * zero_distance;
    if (zero < 0) zero = 0;
    if (zero > 4095) zero = 4095;
    d.affine_zero_tick = static_cast<uint16_t>(zero);
    d.affine_shift_from_q0_ticks =
        static_cast<uint16_t>(absDiff(zero, request.joint[k].q0_tick));
  }
  // Fixed-scale: the q0 each contact implies; their disagreement is reported only.
  const int32_t zero_from_min = static_cast<int32_t>(d.min_contact_tick) - dir * min_delta;
  const int32_t zero_from_max = static_cast<int32_t>(d.max_contact_tick) - dir * max_delta;
  d.fixed_endpoint_disagreement_ticks = static_cast<uint16_t>(absDiff(zero_from_min, zero_from_max));
  d.accepted = d.ordered && d.scale_permille >= kAffineScaleMinPermille &&
               d.scale_permille <= kAffineScaleMaxPermille &&
               d.affine_shift_from_q0_ticks <= kModelZeroMaxShiftTicks;
  return d;
}

// --- lifecycle ------------------------------------------------------------------

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
}

bool FullLegCalibrationExecutor::start(const FullLegCalibrationRequest& request,
                                       const FullLegCalibrationContext& context,
                                       uint32_t now_ms) {
  if (active()) return false;

  bool ok = policy_ != nullptr && runtime_ != nullptr && engine_ != nullptr &&
            geometry_ != nullptr && expected_provenance_ != nullptr &&
            isKnownLeg(request.leg) && request.repeatability_tolerance_ticks > 0 &&
            request.torque_limit > 0 && request.population_count == kFullLegPopulation;
  for (uint8_t k = 0; ok && k < kJointKindCount; ++k) {
    const FullLegJoint& j = request.joint[k];
    ok = j.identity.valid() && j.identity.unitKnown() && j.bus_id != 0 &&
         j.identity.leg == request.leg && static_cast<uint8_t>(j.identity.joint) == k &&
         (request.direction[k] == 1 || request.direction[k] == -1) &&
         request.corridor[k][0].valid() && request.corridor[k][1].valid();
  }
  if (ok && request.has_rear_park) {
    const FullLegJoint& p = request.park;
    ok = p.identity.valid() && p.identity.unitKnown() && p.bus_id != 0 &&
         p.identity.leg != request.leg;
    for (uint8_t k = 0; ok && k < kJointKindCount; ++k) ok = p.bus_id != request.joint[k].bus_id;
  }
  // The population is every leg joint of the robot, each once, and includes
  // the leg and its park joint.
  for (uint8_t i = 0; ok && i < request.population_count; ++i) {
    const FullLegJoint& j = request.population[i];
    ok = j.identity.valid() && j.identity.unitKnown() && j.bus_id != 0;
    for (uint8_t m = 0; ok && m < i; ++m) ok = request.population[m].bus_id != j.bus_id;
  }
  auto inPopulation = [&](uint8_t bus) {
    for (uint8_t i = 0; i < request.population_count; ++i) {
      if (request.population[i].bus_id == bus) return true;
    }
    return false;
  };
  for (uint8_t k = 0; ok && k < kJointKindCount; ++k) ok = inPopulation(request.joint[k].bus_id);
  if (ok && request.has_rear_park) ok = inPopulation(request.park.bus_id);

  if (!ok || !continuationOk(context)) {
    request_ = request;
    status_ = FullLegCalibrationStatus{};
    status_.step = FullLegStep::FAILED;
    status_.failure = FullLegFailure::REJECT_PRECONDITIONS;
    return false;
  }

  request_ = request;
  status_ = FullLegCalibrationStatus{};
  now_ms_ = now_ms;
  for (uint8_t k = 0; k < kJointKindCount; ++k) {
    for (uint8_t s = 0; s < kContactSideCount; ++s) contacts_[k][s] = ContactEvidence{};
    diagnostics_[k] = FullLegJointDiagnostics{};
  }
  diagnostics_accepted_ = false;
  prerequisites_verified_ = false;
  for (uint8_t s = 0; s < kSlotCount; ++s) slot_[s] = SlotState{};
  for (uint8_t i = 0; i < kFullLegPopulation; ++i) population_[i] = PopulationState{};
  recover_index_ = 0;
  recover_bus_ = 0;
  watch_index_ = 0;
  cleanup_ = false;
  return_after_diagnostics_failure_ = false;
  safe_off_verified_mask_ = 0;

  ContactProbeConfig probe_config{};
  probe_config.backoff_deadman = config_.probe_backoff_deadman;
  probe_.begin(policy_, runtime_, engine_, geometry_, expected_provenance_, probe_config);

  status_.step = FullLegStep::VERIFY_REST;
  enterPhase(CalibrationPhase::PREFLIGHT, now_ms);
  return true;
}

bool FullLegCalibrationExecutor::continuationOk(const FullLegCalibrationContext& c) const {
  return c.session_active && c.origin == CalibrationOrigin::LIVE_SESSION &&
         c.mode == core::OperatingMode::MAINTENANCE && c.motion_permit_active &&
         c.lease.valid() && c.lease.owner == core::ActuatorAuthority::CALIBRATION &&
         c.authority == core::ActuatorAuthority::CALIBRATION &&
         c.authority_generation == c.lease.generation && !c.authority_inhibited;
}

const FullLegJoint& FullLegCalibrationExecutor::slotJoint(uint8_t slot) const {
  return slot == kSlotPark ? request_.park : request_.joint[slot < kJointKindCount ? slot : 0];
}

bool FullLegCalibrationExecutor::isParticipantBus(uint8_t bus, uint8_t* slot) const {
  for (uint8_t s = 0; s < kSlotCount; ++s) {
    if (s == kSlotPark && !request_.has_rear_park) continue;
    if (slotJoint(s).bus_id == bus) {
      if (slot != nullptr) *slot = s;
      return true;
    }
  }
  return false;
}

// --- phase programs -----------------------------------------------------------

void FullLegCalibrationExecutor::addStep(Op op, uint8_t slot, uint16_t tick,
                                         actuator::MicroRad urad) {
  if (program_count_ >= kMaxProgram) return;
  ProgramStep& p = program_[program_count_++];
  p.op = op;
  p.slot = slot;
  p.target_tick = tick;
  p.target_urad = urad;
}

void FullLegCalibrationExecutor::buildProgram() {
  program_count_ = 0;
  program_index_ = 0;
  const FullLegCalibrationRequest& r = request_;
  const uint16_t hip_q0 = r.joint[kSlotHip].q0_tick;
  const uint16_t upper_q0 = r.joint[kSlotUpper].q0_tick;
  const uint16_t lower_q0 = r.joint[kSlotLower].q0_tick;
  const bool hip_poses_differ = r.upper_for_hip_min_urad != r.upper_for_hip_max_urad;

  switch (status_.phase) {
    case CalibrationPhase::PREFLIGHT:
      addStep(Op::VERIFY_REST, kRestPreflight);
      break;
    case CalibrationPhase::INITIAL_RECOVERY:
      addStep(Op::RECOVER_ALL, 0);
      break;
    case CalibrationPhase::PARKING:
      if (r.has_rear_park) {
        addStep(Op::ENERGIZE, kSlotPark);
        addStep(Op::MOVE, kSlotPark, r.park_target_tick, r.park_target_urad);
        addStep(Op::HOLD, kSlotPark);
      }
      break;
    case CalibrationPhase::UPPER_MIN:
      // V25 prerequisites_for(UPPER): HIP and LOWER held at q=0.
      addStep(Op::ENERGIZE, kSlotHip);
      addStep(Op::MOVE, kSlotHip, hip_q0, 0);
      addStep(Op::HOLD, kSlotHip);
      addStep(Op::ENERGIZE, kSlotLower);
      addStep(Op::MOVE, kSlotLower, lower_q0, 0);
      addStep(Op::HOLD, kSlotLower);
      addStep(Op::ENERGIZE, kSlotUpper);
      addStep(Op::PROBE, kSlotUpper);
      break;
    case CalibrationPhase::UPPER_MAX:
      addStep(Op::PROBE, kSlotUpper);
      break;
    case CalibrationPhase::UPPER_HORIZONTAL:
      addStep(Op::MOVE, kSlotUpper, r.upper_for_lower_tick, r.upper_for_lower_urad);
      addStep(Op::HOLD, kSlotUpper);
      break;
    case CalibrationPhase::LOWER_MIN:
      addStep(Op::RELEASE, kSlotLower);
      addStep(Op::PROBE, kSlotLower);
      break;
    case CalibrationPhase::LOWER_MAX:
      addStep(Op::PROBE, kSlotLower);
      break;
    case CalibrationPhase::LOWER_FOLDED:
      addStep(Op::MOVE, kSlotLower, r.lower_folded_tick, r.lower_folded_urad);
      addStep(Op::HOLD, kSlotLower);
      if (r.upper_for_hip_min_urad != r.upper_for_lower_urad) {
        addStep(Op::RELEASE, kSlotUpper);
        addStep(Op::MOVE, kSlotUpper, r.upper_for_hip_min_tick, r.upper_for_hip_min_urad);
        addStep(Op::HOLD, kSlotUpper);
      }
      break;
    case CalibrationPhase::HIP_MIN:
      addStep(Op::RELEASE, kSlotHip);
      addStep(Op::PROBE, kSlotHip);
      break;
    case CalibrationPhase::HIP_MAX:
      if (hip_poses_differ) {
        // V25 per-side clearance: HIP back to q0, UPPER to the MAX pose.
        addStep(Op::MOVE, kSlotHip, hip_q0, 0);
        addStep(Op::HOLD, kSlotHip);
        addStep(Op::RELEASE, kSlotUpper);
        addStep(Op::MOVE, kSlotUpper, r.upper_for_hip_max_tick, r.upper_for_hip_max_urad);
        addStep(Op::HOLD, kSlotUpper);
        addStep(Op::RELEASE, kSlotHip);
      }
      addStep(Op::PROBE, kSlotHip);
      break;
    case CalibrationPhase::DIAGNOSTICS:
      addStep(Op::DIAGNOSE, 0);
      break;
    case CalibrationPhase::RETURN_HIP:
      addStep(Op::MOVE, kSlotHip, hip_q0, 0);
      addStep(Op::HOLD, kSlotHip);
      break;
    case CalibrationPhase::RETURN_LOWER_HELD:
      addStep(Op::RELEASE, kSlotLower);
      addStep(Op::MOVE, kSlotLower, lower_q0, 0);
      addStep(Op::HOLD, kSlotLower);
      break;
    case CalibrationPhase::RETURN_UPPER:
      addStep(Op::RELEASE, kSlotUpper);
      addStep(Op::MOVE, kSlotUpper, upper_q0, 0);
      addStep(Op::HOLD, kSlotUpper);
      break;
    case CalibrationPhase::RESTORE_PARKING:
      if (r.has_rear_park) {
        addStep(Op::RELEASE, kSlotPark);
        addStep(Op::MOVE, kSlotPark, r.park.q0_tick, 0);
        addStep(Op::HOLD, kSlotPark);
      }
      break;
    case CalibrationPhase::CLEANUP:
      addStep(Op::SAFE_OFF_ALL, 0);
      break;
    case CalibrationPhase::TORQUE_OFF:
      if (cleanup_) {
        addStep(Op::SAFE_OFF_ALL, 0);
      } else if (r.recovery_only) {
        // Recovery run: no CLEANUP phase precedes this one.
        addStep(Op::SAFE_OFF_ALL, 0);
        addStep(Op::VERIFY_REST, kRestFinal);
      } else {
        addStep(Op::VERIFY_REST, kRestFinal);
      }
      break;
  }
}

void FullLegCalibrationExecutor::enterPhase(CalibrationPhase phase, uint32_t now_ms) {
  status_.phase = phase;
  status_.phase_changes++;
  sub_ = 0;
  op_started_ms_ = now_ms;
  settled_samples_ = 0;
  recover_index_ = 0;
  prerequisites_verified_ = false;
  buildProgram();
}

void FullLegCalibrationExecutor::advanceProgram(uint32_t now_ms) {
  sub_ = 0;
  settled_samples_ = 0;
  op_started_ms_ = now_ms;
  ++program_index_;
  if (program_index_ < program_count_) return;
  nextPhase(now_ms);
}

// Program exhausted: the next phase. At most one phase change per update(),
// so the caller can report every phase to the session in V25 order.
void FullLegCalibrationExecutor::nextPhase(uint32_t now_ms) {
  if (cleanup_) {
    status_.step = FullLegStep::FAILED;
    return;
  }
  switch (status_.phase) {
    case CalibrationPhase::PREFLIGHT:         enterPhase(CalibrationPhase::INITIAL_RECOVERY, now_ms); return;
    case CalibrationPhase::INITIAL_RECOVERY:
      // A recovery run ends here: straight to the terminal safe state.
      enterPhase(request_.recovery_only ? CalibrationPhase::TORQUE_OFF : CalibrationPhase::PARKING,
                 now_ms);
      return;
    case CalibrationPhase::PARKING:           enterPhase(CalibrationPhase::UPPER_MIN, now_ms); return;
    case CalibrationPhase::UPPER_MIN:         enterPhase(CalibrationPhase::UPPER_MAX, now_ms); return;
    case CalibrationPhase::UPPER_MAX:         enterPhase(CalibrationPhase::UPPER_HORIZONTAL, now_ms); return;
    case CalibrationPhase::UPPER_HORIZONTAL:  enterPhase(CalibrationPhase::LOWER_MIN, now_ms); return;
    case CalibrationPhase::LOWER_MIN:         enterPhase(CalibrationPhase::LOWER_MAX, now_ms); return;
    case CalibrationPhase::LOWER_MAX:         enterPhase(CalibrationPhase::LOWER_FOLDED, now_ms); return;
    case CalibrationPhase::LOWER_FOLDED:      enterPhase(CalibrationPhase::HIP_MIN, now_ms); return;
    case CalibrationPhase::HIP_MIN:           enterPhase(CalibrationPhase::HIP_MAX, now_ms); return;
    case CalibrationPhase::HIP_MAX:           enterPhase(CalibrationPhase::DIAGNOSTICS, now_ms); return;
    case CalibrationPhase::DIAGNOSTICS:       enterPhase(CalibrationPhase::RETURN_HIP, now_ms); return;
    case CalibrationPhase::RETURN_HIP:        enterPhase(CalibrationPhase::RETURN_LOWER_HELD, now_ms); return;
    case CalibrationPhase::RETURN_LOWER_HELD: enterPhase(CalibrationPhase::RETURN_UPPER, now_ms); return;
    case CalibrationPhase::RETURN_UPPER:      enterPhase(CalibrationPhase::RESTORE_PARKING, now_ms); return;
    case CalibrationPhase::RESTORE_PARKING:   enterPhase(CalibrationPhase::CLEANUP, now_ms); return;
    case CalibrationPhase::CLEANUP:           enterPhase(CalibrationPhase::TORQUE_OFF, now_ms); return;
    case CalibrationPhase::TORQUE_OFF: {
      // 6/6 contacts AND diagnostics AND a verified rest: COMPLETE, nothing
      // less. A recovery run: all twelve joints actively recovered, no contact.
      const bool done =
          request_.recovery_only
              ? status_.recovered_joints == request_.population_count &&
                    status_.contacts_accepted == 0
              : status_.contacts_accepted == kFullLegContactCount && diagnostics_accepted_;
      status_.step = (status_.failure == FullLegFailure::NONE && done) ? FullLegStep::COMPLETE
                                                                       : FullLegStep::FAILED;
      if (status_.step == FullLegStep::FAILED && status_.failure == FullLegFailure::NONE) {
        status_.failure = request_.recovery_only ? FullLegFailure::INITIAL_RECOVERY_NOT_SETTLED
                                                 : FullLegFailure::DIAGNOSTICS_REJECTED;
      }
      return;
    }
  }
}

void FullLegCalibrationExecutor::fail(FullLegFailure failure) {
  if (cleanup_) return;  // the first failure is the record; cleanup is already running
  status_.failure = failure;
  status_.failed_phase = status_.phase;
  if (probe_.active()) probe_.abort();  // no write: the abort only records
  prerequisites_verified_ = false;
  cleanup_ = true;
  safe_off_verified_mask_ = 0;
  // Any phase may go straight to the terminal safe state (isLegalPhaseTransition).
  enterPhase(CalibrationPhase::TORQUE_OFF, now_ms_);
}

bool FullLegCalibrationExecutor::monitoringActive() const {
  if (cleanup_) return false;
  if (status_.phase == CalibrationPhase::CLEANUP || status_.phase == CalibrationPhase::TORQUE_OFF) {
    return false;
  }
  return true;
}

void FullLegCalibrationExecutor::abort() {
  if (!active()) return;
  fail(FullLegFailure::OPERATOR_ABORT);
}

void FullLegCalibrationExecutor::phaseReportRejected() {
  if (!active()) return;
  fail(FullLegFailure::PHASE_REPORT_REJECTED);
}

// --- monitoring -----------------------------------------------------------------

bool FullLegCalibrationExecutor::sampleUsable(const actuator::TelemetrySample* s) const {
  return s != nullptr && s->read_ok && s->present_position >= 0 && s->present_position < 4096 &&
         s->torque_enable >= 0 && s->present_speed >= 0 && s->present_current >= 0 &&
         s->present_temperature >= 0 && s->goal_position >= 0 && s->torque_limit >= 0 &&
         s->servo_status >= 0;
}

bool FullLegCalibrationExecutor::commonSafety(const actuator::TelemetrySample& s) {
  if (s.servo_status != 0) {
    fail(FullLegFailure::SERVO_STATUS_FAULT);
    return false;
  }
  if (magnitude(s.present_current) >= kSearchHardCurrentAbortRaw) {
    fail(FullLegFailure::HARD_CURRENT_ABORT);
    return false;
  }
  if (s.present_temperature > kSearchTemperatureLimitC) {
    fail(FullLegFailure::OVER_TEMPERATURE);
    return false;
  }
  return true;
}

bool FullLegCalibrationExecutor::monitorHeld(uint32_t now_ms, const FullLegTelemetryFrame& t) {
  uint8_t held = 0;
  for (uint8_t s = 0; s < kSlotCount; ++s) {
    SlotState& st = slot_[s];
    if (!st.held) continue;
    ++held;
    const FullLegJoint& j = slotJoint(s);
    const actuator::TelemetrySample* sample = t.find(j.bus_id);
    if (!sampleUsable(sample)) {
      if (now_ms - st.last_good_ms >= kSequenceMaxTelemetryAgeMs) {
        fail(FullLegFailure::STALE_TELEMETRY);
        return false;
      }
      continue;
    }
    st.last_good_ms = now_ms;
    if (!commonSafety(*sample)) return false;
    if (sample->torque_enable != 1 ||
        sample->torque_limit != static_cast<int32_t>(request_.torque_limit) ||
        sample->goal_position != static_cast<int32_t>(st.target_tick)) {
      fail(FullLegFailure::HELD_JOINT_READBACK);
      return false;
    }
    if (absDiff(sample->present_position, st.target_tick) >
        static_cast<int32_t>(kSequenceStaticToleranceTicks)) {
      fail(FullLegFailure::HELD_JOINT_DRIFT);
      return false;
    }
    if (magnitude(sample->present_speed) > static_cast<int32_t>(kSequenceHeldSpeedAbortRaw)) {
      if (++st.fast_samples >= kSequenceHeldSpeedAbortSamples) {
        fail(FullLegFailure::HELD_JOINT_SPEED);
        return false;
      }
    } else {
      st.fast_samples = 0;
    }
  }
  status_.held_count = held;
  return true;
}

bool FullLegCalibrationExecutor::monitorBystander(uint32_t now_ms, const FullLegTelemetryFrame& t) {
  // Only once INITIAL_RECOVERY has recorded where every joint rests.
  if (!population_[0].entry_known) return true;
  uint8_t i = 0;
  if (!watchIndex(&i)) return true;
  const FullLegJoint& j = request_.population[i];
  PopulationState& ps = population_[i];
  const actuator::TelemetrySample* sample = t.find(j.bus_id);
  if (!sampleUsable(sample)) {
    if (now_ms - ps.last_good_ms >= kSequenceMaxTelemetryAgeMs) {
      fail(FullLegFailure::STALE_TELEMETRY);
      return false;
    }
    watch_index_ = static_cast<uint8_t>((i + 1) % request_.population_count);
    return true;
  }
  ps.last_good_ms = now_ms;
  watch_index_ = static_cast<uint8_t>((i + 1) % request_.population_count);
  if (!commonSafety(*sample)) return false;
  uint8_t slot = 0;
  const bool participant = isParticipantBus(j.bus_id, &slot);
  if (sample->torque_enable != 0) {
    fail(participant ? FullLegFailure::PASSIVE_JOINT_MOVED : FullLegFailure::BYSTANDER_MOVED);
    return false;
  }
  if (participant) {
    if (absDiff(sample->present_position, j.q0_tick) >
        static_cast<int32_t>(kSequencePassiveCorridorTicks)) {
      fail(FullLegFailure::PASSIVE_JOINT_MOVED);
      return false;
    }
  } else if (absDiff(sample->present_position, ps.entry_tick) >
             static_cast<int32_t>(kSequenceBystanderDriftTicks)) {
    fail(FullLegFailure::BYSTANDER_MOVED);
    return false;
  }
  return true;
}

// A population index to watch this tick: not energized, not the op's joint.
// The op's own joint is excluded explicitly: between its TorqueEnable write
// and the verifying sample it is torque-on but not yet marked energized, and
// the op itself checks it on that very sample.
bool FullLegCalibrationExecutor::watchIndex(uint8_t* out) const {
  if (request_.population_count == 0) return false;
  uint8_t op_bus = 0;
  if (program_index_ < program_count_) {
    const ProgramStep& p = program_[program_index_];
    if (p.op == Op::ENERGIZE || p.op == Op::MOVE || p.op == Op::PROBE) op_bus = slotJoint(p.slot).bus_id;
  }
  for (uint8_t n = 0; n < request_.population_count; ++n) {
    const uint8_t i = static_cast<uint8_t>((watch_index_ + n) % request_.population_count);
    const uint8_t bus = request_.population[i].bus_id;
    uint8_t slot = 0;
    if (bus == op_bus) continue;
    if (isParticipantBus(bus, &slot) && slot_[slot].energized) continue;
    *out = i;
    return true;
  }
  return false;
}

// --- requests to the caller -----------------------------------------------------

uint8_t FullLegCalibrationExecutor::telemetryRequest(uint8_t* buses, uint8_t max) const {
  if (!active() || buses == nullptr) return 0;
  uint8_t n = 0;
  auto add = [&](uint8_t bus) {
    if (bus == 0 || n >= max) return;
    for (uint8_t m = 0; m < n; ++m) {
      if (buses[m] == bus) return;
    }
    buses[n++] = bus;
  };
  const bool have_op = program_index_ < program_count_;
  const Op op = have_op ? program_[program_index_].op : Op::DIAGNOSE;
  if (have_op && op == Op::SAFE_OFF_ALL) return 0;  // the SAFE_OFF readbacks are the evidence

  for (uint8_t s = 0; s < kSlotCount; ++s) {
    if (slot_[s].held) add(slotJoint(s).bus_id);
  }
  if (have_op) {
    switch (op) {
      case Op::ENERGIZE:
      case Op::MOVE:
      case Op::PROBE:
        add(slotJoint(program_[program_index_].slot).bus_id);
        break;
      case Op::RECOVER_ALL:
      case Op::VERIFY_REST:
        if (recover_index_ < request_.population_count && sub_ != R_SAFE_OFF) {
          add(request_.population[recover_index_].bus_id);
        }
        break;
      default:
        break;
    }
  }
  uint8_t w = 0;
  if (population_[0].entry_known && monitoringActive() && watchIndex(&w)) {
    add(request_.population[w].bus_id);
  }
  return n;
}

uint8_t FullLegCalibrationExecutor::safeOffRequest(uint8_t* buses, uint8_t max) const {
  if (!active() || buses == nullptr || program_index_ >= program_count_) return 0;
  const ProgramStep& p = program_[program_index_];
  uint8_t n = 0;
  if (p.op == Op::RECOVER_ALL && sub_ == R_SAFE_OFF && recover_bus_ != 0 && max > 0) {
    buses[n++] = recover_bus_;
  } else if (p.op == Op::SAFE_OFF_ALL) {
    for (uint8_t i = 0; i < request_.population_count && n < max; ++i) {
      if ((safe_off_verified_mask_ & (1u << i)) == 0) buses[n++] = request_.population[i].bus_id;
    }
  }
  return n;
}

// --- writes (each exactly one backend call) -------------------------------------

bool FullLegCalibrationExecutor::routeResult(actuator::ExecuteResult result,
                                             FullLegFailure rejected) {
  switch (result) {
    case actuator::ExecuteResult::WRITTEN:
      return true;
    case actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF:
      fail(FullLegFailure::WRITE_UNCERTAIN);
      return false;
    default:
      fail(rejected);
      return false;
  }
}

bool FullLegCalibrationExecutor::writePrime(const FullLegCalibrationContext& ctx,
                                            const FullLegJoint& j, uint16_t tick) {
  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::SEQUENCE_MOVE;
  req.joint = j.identity;
  req.endpoint_leg = request_.leg;
  req.sequence_move = actuator::SequenceMoveKind::PRIME_AT_PRESENT;
  req.sequence_phase = status_.phase;
  req.prime_tick = tick;
  req.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;
  const CalibrationExecutionResult r = engine_->execute(req, toExecutionContext(ctx), j.bus_id);
  status_.last_policy_decision = r.policy_decision;
  if (r.outcome != CalibrationExecutionOutcome::ROUTED_TO_POLICY) {
    fail(FullLegFailure::PRIME_REJECTED);
    return false;
  }
  return routeResult(r.execute_result, FullLegFailure::PRIME_REJECTED);
}

bool FullLegCalibrationExecutor::writeMove(const FullLegCalibrationContext& ctx,
                                           const FullLegJoint& j, actuator::MicroRad target_urad) {
  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::SEQUENCE_MOVE;
  req.joint = j.identity;
  req.endpoint_leg = request_.leg;
  req.sequence_move = actuator::SequenceMoveKind::TO_PLAN_TARGET;
  req.sequence_phase = status_.phase;
  req.target_urad = target_urad;
  req.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;
  const CalibrationExecutionResult r = engine_->execute(req, toExecutionContext(ctx), j.bus_id);
  status_.last_policy_decision = r.policy_decision;
  if (r.outcome != CalibrationExecutionOutcome::ROUTED_TO_POLICY) {
    fail(FullLegFailure::MOVE_REJECTED);
    return false;
  }
  return routeResult(r.execute_result, FullLegFailure::MOVE_REJECTED);
}

static bool planAndExecute(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
                           actuator::ActuatorOperation operation, const JointIdentity& joint,
                           uint8_t bus, const FullLegCalibrationContext& ctx,
                           actuator::WriteDecision* decision, actuator::ExecuteResult* result) {
  actuator::ActuatorCommand cmd{};
  cmd.operation = operation;
  cmd.joint = joint;
  actuator::ActuatorTransaction txn{};
  *decision = policy->plan(cmd, ctx.lease, ctx.mode, &txn);
  if (*decision != actuator::WriteDecision::ACCEPT) {
    *result = actuator::ExecuteResult::NOT_EXECUTED;
    return false;
  }
  actuator::WriteDecision commit = *decision;
  *result = runtime->execute(&txn, bus, &commit);
  *decision = commit;
  return true;
}

bool FullLegCalibrationExecutor::writeTorqueLimit(const FullLegCalibrationContext& ctx,
                                                  const FullLegJoint& j) {
  actuator::WriteDecision d = actuator::WriteDecision::REJECT_NO_ARBITER;
  actuator::ExecuteResult r = actuator::ExecuteResult::NOT_EXECUTED;
  planAndExecute(policy_, runtime_, actuator::ActuatorOperation::CALIBRATION_TORQUE_LIMIT,
                 j.identity, j.bus_id, ctx, &d, &r);
  status_.last_policy_decision = d;
  return routeResult(r, FullLegFailure::TORQUE_LIMIT_REJECTED);
}

bool FullLegCalibrationExecutor::writeTorqueOn(const FullLegCalibrationContext& ctx,
                                               const FullLegJoint& j) {
  actuator::WriteDecision d = actuator::WriteDecision::REJECT_NO_ARBITER;
  actuator::ExecuteResult r = actuator::ExecuteResult::NOT_EXECUTED;
  planAndExecute(policy_, runtime_, actuator::ActuatorOperation::TORQUE_ENABLE, j.identity,
                 j.bus_id, ctx, &d, &r);
  status_.last_policy_decision = d;
  return routeResult(r, FullLegFailure::TORQUE_ENABLE_REJECTED);
}

// --- ops ------------------------------------------------------------------------

// V25 prepare_motor(): GoalPosition := present (verified), RAM TorqueLimit
// (verified), TorqueEnable (verified), then one fresh sample must show all
// three. One write per tick.
void FullLegCalibrationExecutor::stepEnergize(const FullLegCalibrationContext& ctx,
                                              uint32_t now_ms, const FullLegTelemetryFrame& t) {
  const uint8_t slot = program_[program_index_].slot;
  const FullLegJoint& j = slotJoint(slot);
  SlotState& st = slot_[slot];
  const actuator::TelemetrySample* s = t.find(j.bus_id);
  status_.step = FullLegStep::PRIME;

  if (sub_ == 0) {
    if (st.energized) {  // already torque-on (e.g. the MAX side of a probe)
      advanceProgram(now_ms);
      return;
    }
    if (!sampleUsable(s)) {
      if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
      return;
    }
    if (!commonSafety(*s)) return;
    if (s->torque_enable != 0 ||
        absDiff(s->present_position, j.q0_tick) > static_cast<int32_t>(kSequencePassiveCorridorTicks)) {
      fail(FullLegFailure::PASSIVE_JOINT_MOVED);
      return;
    }
    op_prime_tick_ = static_cast<uint16_t>(s->present_position);
    if (!writePrime(ctx, j, op_prime_tick_)) return;
    sub_ = 1;
    return;
  }
  if (sub_ == 1) {
    status_.step = FullLegStep::TORQUE_LIMIT;
    if (!writeTorqueLimit(ctx, j)) return;
    sub_ = 2;
    return;
  }
  if (sub_ == 2) {
    status_.step = FullLegStep::TORQUE_ON;
    if (!writeTorqueOn(ctx, j)) return;
    op_started_ms_ = now_ms;
    sub_ = 3;
    return;
  }
  // sub_ == 3: a sample read after the TorqueEnable write.
  status_.step = FullLegStep::VERIFY_ENERGIZED;
  if (!sampleUsable(s)) {
    if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
    return;
  }
  if (!commonSafety(*s)) return;
  if (s->torque_enable != 1 || s->torque_limit != static_cast<int32_t>(request_.torque_limit) ||
      s->goal_position != static_cast<int32_t>(op_prime_tick_)) {
    fail(FullLegFailure::ENERGIZE_NOT_VERIFIED);
    return;
  }
  st.energized = true;
  st.target_tick = op_prime_tick_;
  st.last_good_ms = now_ms;
  advanceProgram(now_ms);
}

// A reviewed plan move at the V25 envelope, then V25's StableTargetGate:
// within 10 ticks at |speed| <= 4 for 4 consecutive samples spanning >= 400 ms.
void FullLegCalibrationExecutor::stepMove(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                                          const FullLegTelemetryFrame& t) {
  const ProgramStep& p = program_[program_index_];
  const FullLegJoint& j = slotJoint(p.slot);
  SlotState& st = slot_[p.slot];
  const actuator::TelemetrySample* s = t.find(j.bus_id);
  status_.op_target_tick = p.target_tick;

  if (sub_ == 0) {
    status_.step = FullLegStep::MOVE_WRITE;
    if (!st.energized || st.held) {  // a program bug: never move a limp or held joint
      fail(FullLegFailure::MOVE_REJECTED);
      return;
    }
    const int32_t from = sampleUsable(s) ? s->present_position : st.target_tick;
    if (!writeMove(ctx, j, p.target_urad)) return;
    st.target_tick = p.target_tick;
    const uint32_t distance = static_cast<uint32_t>(absDiff(from, p.target_tick));
    op_deadline_ms_ = now_ms + kSequenceMotionTimeoutMs + distance * 1000u / kSequenceMinTicksPerSecond;
    op_started_ms_ = now_ms;
    settled_samples_ = 0;
    sub_ = 1;
    return;
  }

  status_.step = FullLegStep::MOVE_MONITOR;
  if (static_cast<int32_t>(now_ms - op_deadline_ms_) > 0) {
    fail(FullLegFailure::MOVE_TIMEOUT);
    return;
  }
  if (!sampleUsable(s)) {
    if (now_ms - st.last_good_ms >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
    return;
  }
  st.last_good_ms = now_ms;
  if (!commonSafety(*s)) return;
  if (s->torque_enable != 1 || s->torque_limit != static_cast<int32_t>(request_.torque_limit) ||
      s->goal_position != static_cast<int32_t>(p.target_tick)) {
    fail(FullLegFailure::MOVE_READBACK);
    return;
  }
  const bool qualifies =
      absDiff(s->present_position, p.target_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks) &&
      magnitude(s->present_speed) <= static_cast<int32_t>(kSequenceSettleMaxSpeedRaw);
  if (!qualifies) {
    settled_samples_ = 0;
    return;
  }
  if (settled_samples_ == 0) settle_first_ms_ = now_ms;
  if (settled_samples_ < 255) ++settled_samples_;
  if (settled_samples_ >= kSequenceSettledSamples &&
      now_ms - settle_first_ms_ >= kSequenceSettleWindowMs) {
    advanceProgram(now_ms);
  }
}

void FullLegCalibrationExecutor::stepProbe(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                                           const FullLegTelemetryFrame& t) {
  const uint8_t slot = program_[program_index_].slot;
  const FullLegJoint& j = slotJoint(slot);
  JointKind joint = JointKind::UPPER;
  ContactSide side = ContactSide::MIN_SIDE;
  status_.step = FullLegStep::PROBE;
  if (!actuator::sequenceProbeEndpoint(status_.phase, &joint, &side) ||
      static_cast<uint8_t>(joint) != slot) {
    fail(FullLegFailure::REJECT_PRECONDITIONS);
    return;
  }

  if (sub_ == 0) {
    // V25 validate_transition_entry(): the held set must be EXACTLY this
    // phase's prerequisites, each at its plan pose.
    bool want[kSlotCount] = {false, false, false, request_.has_rear_park};
    uint16_t want_tick[kSlotCount] = {0, 0, 0, request_.park_target_tick};
    switch (joint) {
      case JointKind::UPPER:
        want[kSlotHip] = true;   want_tick[kSlotHip] = request_.joint[kSlotHip].q0_tick;
        want[kSlotLower] = true; want_tick[kSlotLower] = request_.joint[kSlotLower].q0_tick;
        break;
      case JointKind::LOWER:
        want[kSlotHip] = true;   want_tick[kSlotHip] = request_.joint[kSlotHip].q0_tick;
        want[kSlotUpper] = true; want_tick[kSlotUpper] = request_.upper_for_lower_tick;
        break;
      case JointKind::HIP:
        want[kSlotUpper] = true;
        want_tick[kSlotUpper] = side == ContactSide::MIN_SIDE ? request_.upper_for_hip_min_tick
                                                              : request_.upper_for_hip_max_tick;
        want[kSlotLower] = true; want_tick[kSlotLower] = request_.lower_folded_tick;
        break;
    }
    for (uint8_t s = 0; s < kSlotCount; ++s) {
      if (slot_[s].held != want[s] || (want[s] && slot_[s].target_tick != want_tick[s])) {
        fail(FullLegFailure::HELD_SET_MISMATCH);
        return;
      }
    }
    if (!slot_[slot].energized || slot_[slot].held) {
      fail(FullLegFailure::HELD_SET_MISMATCH);
      return;
    }
    ContactProbeRequest pr{};
    pr.joint = j.identity;
    pr.bus_id = j.bus_id;
    pr.endpoint_leg = request_.leg;
    pr.endpoint_joint = joint;
    pr.endpoint_side = side;
    pr.corridor = request_.corridor[slot][static_cast<uint8_t>(side)];
    pr.repeatability_tolerance_ticks = request_.repeatability_tolerance_ticks;
    pr.start_torque_verified = true;
    pr.expected_torque_limit = request_.torque_limit;
    // Held prerequisites verified: the policy now admits this endpoint's search.
    prerequisites_verified_ = true;
    if (!probe_.start(pr, toProbeContext(ctx), now_ms)) {
      prerequisites_verified_ = false;
      fail(joint == JointKind::UPPER ? (side == ContactSide::MIN_SIDE ? FullLegFailure::UPPER_MIN_PROBE_FAILED
                                                                      : FullLegFailure::UPPER_MAX_PROBE_FAILED)
           : joint == JointKind::LOWER ? (side == ContactSide::MIN_SIDE ? FullLegFailure::LOWER_MIN_PROBE_FAILED
                                                                        : FullLegFailure::LOWER_MAX_PROBE_FAILED)
                                       : (side == ContactSide::MIN_SIDE ? FullLegFailure::HIP_MIN_PROBE_FAILED
                                                                        : FullLegFailure::HIP_MAX_PROBE_FAILED));
      return;
    }
    sub_ = 1;
    return;
  }

  const actuator::TelemetrySample* s = t.find(j.bus_id);
  actuator::TelemetrySample empty{};
  probe_.update(toProbeContext(ctx), now_ms, s != nullptr, s != nullptr ? *s : empty);
  status_.last_policy_decision = probe_.status().last_policy_decision;
  if (probe_.active()) return;

  prerequisites_verified_ = false;
  if (probe_.status().phase != ContactProbePhase::COMPLETE) {
    fail(joint == JointKind::UPPER ? (side == ContactSide::MIN_SIDE ? FullLegFailure::UPPER_MIN_PROBE_FAILED
                                                                    : FullLegFailure::UPPER_MAX_PROBE_FAILED)
         : joint == JointKind::LOWER ? (side == ContactSide::MIN_SIDE ? FullLegFailure::LOWER_MIN_PROBE_FAILED
                                                                      : FullLegFailure::LOWER_MAX_PROBE_FAILED)
                                     : (side == ContactSide::MIN_SIDE ? FullLegFailure::HIP_MIN_PROBE_FAILED
                                                                      : FullLegFailure::HIP_MAX_PROBE_FAILED));
    return;
  }
  ContactEvidence& e = contacts_[slot][static_cast<uint8_t>(side)];
  e = ContactEvidence{};
  e.key.leg = request_.leg;
  e.key.joint = joint;
  e.key.side = side;
  e.state = EvidenceState::PROMOTED;
  e.origin = CalibrationOrigin::LIVE_SESSION;
  e.detection = ContactState::CONTACT_CONFIRMED;
  e.witness = probe_.witness();
  // The two accepted fine passes (no coarse contact scout exists): pass 1,
  // then the repeatability-confirming pass 2, reported as both later fields
  // rather than fabricating an unmeasured third one.
  e.coarse_tick = probe_.status().pass1_contact_tick;
  e.fine_tick_1 = probe_.status().pass2_contact_tick;
  e.fine_tick_2 = probe_.status().pass2_contact_tick;
  e.repeatability_ticks = e.witness.max_deviation_ticks;
  e.has_measurement = true;
  if (status_.contacts_accepted < kFullLegContactCount) ++status_.contacts_accepted;
  // The probe leaves the joint torque-on, GoalPosition = the released contact.
  slot_[slot].target_tick = probe_.status().pass2_contact_tick;
  slot_[slot].last_good_ms = now_ms;
  advanceProgram(now_ms);
}

// V25 normalize_all_matdog_joints_to_q0(), one joint at a time, bounded.
void FullLegCalibrationExecutor::stepRecover(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                                             const FullLegTelemetryFrame& t,
                                             const FullLegSafeOffFrame& safe_off) {
  if (recover_index_ >= request_.population_count && sub_ != R_FINAL) {
    sub_ = R_FINAL;
    recover_index_ = 0;
    op_started_ms_ = now_ms;
  }
  const FullLegJoint& j = request_.population[recover_index_ < request_.population_count
                                                  ? recover_index_ : 0];
  const actuator::TelemetrySample* s = t.find(j.bus_id);
  status_.op_bus = j.bus_id;
  status_.op_target_tick = j.q0_tick;

  switch (sub_) {
    case R_INSPECT: {
      status_.step = FullLegStep::INSPECT;
      if (!sampleUsable(s)) {
        if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
        return;
      }
      if (!commonSafety(*s)) return;
      if (s->torque_enable != 0) {
        fail(FullLegFailure::PREFLIGHT_TORQUE_ON);
        return;
      }
      // Every joint is actively commanded to q0, however close it already is
      // (see INITIAL_RECOVERY in the header): no "already there" shortcut.
      const int32_t distance = absDiff(s->present_position, j.q0_tick);
      if (distance > static_cast<int32_t>(actuator::kSequencePrimeMaxDistanceTicks)) {
        fail(FullLegFailure::INITIAL_RECOVERY_OUT_OF_RANGE);
        return;
      }
      op_prime_tick_ = static_cast<uint16_t>(s->present_position);
      recover_bus_ = j.bus_id;
      status_.step = FullLegStep::PRIME;
      if (!writePrime(ctx, j, op_prime_tick_)) return;
      sub_ = R_LIMIT;
      return;
    }
    case R_LIMIT:
      status_.step = FullLegStep::TORQUE_LIMIT;
      if (!writeTorqueLimit(ctx, j)) return;
      sub_ = R_TORQUE;
      return;
    case R_TORQUE:
      status_.step = FullLegStep::TORQUE_ON;
      if (!writeTorqueOn(ctx, j)) return;
      op_started_ms_ = now_ms;
      sub_ = R_VERIFY;
      return;
    case R_VERIFY: {
      status_.step = FullLegStep::VERIFY_ENERGIZED;
      if (!sampleUsable(s)) {
        if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
        return;
      }
      if (!commonSafety(*s)) return;
      if (s->torque_enable != 1 || s->torque_limit != static_cast<int32_t>(request_.torque_limit) ||
          s->goal_position != static_cast<int32_t>(op_prime_tick_)) {
        fail(FullLegFailure::ENERGIZE_NOT_VERIFIED);
        return;
      }
      status_.step = FullLegStep::MOVE_WRITE;
      if (!writeMove(ctx, j, 0)) return;  // q=0: the promoted q0 tick
      const uint32_t distance = static_cast<uint32_t>(absDiff(s->present_position, j.q0_tick));
      op_deadline_ms_ = now_ms + kSequenceMotionTimeoutMs + distance * 1000u / kSequenceMinTicksPerSecond;
      op_started_ms_ = now_ms;
      settled_samples_ = 0;
      sub_ = R_MOVE;
      return;
    }
    case R_MOVE: {
      status_.step = FullLegStep::MOVE_MONITOR;
      if (static_cast<int32_t>(now_ms - op_deadline_ms_) > 0) {
        fail(FullLegFailure::MOVE_TIMEOUT);
        return;
      }
      if (!sampleUsable(s)) {
        if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
        return;
      }
      op_started_ms_ = now_ms;
      if (!commonSafety(*s)) return;
      if (s->torque_enable != 1 || s->torque_limit != static_cast<int32_t>(request_.torque_limit) ||
          s->goal_position != static_cast<int32_t>(j.q0_tick)) {
        fail(FullLegFailure::MOVE_READBACK);
        return;
      }
      const bool qualifies =
          absDiff(s->present_position, j.q0_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks) &&
          magnitude(s->present_speed) <= static_cast<int32_t>(kSequenceSettleMaxSpeedRaw);
      if (!qualifies) {
        settled_samples_ = 0;
        return;
      }
      if (settled_samples_ == 0) settle_first_ms_ = now_ms;
      if (settled_samples_ < 255) ++settled_samples_;
      if (settled_samples_ >= kSequenceSettledSamples &&
          now_ms - settle_first_ms_ >= kSequenceSettleWindowMs) {
        sub_ = R_SAFE_OFF;  // the caller SAFE_OFFs recover_bus_ after this update
        op_started_ms_ = now_ms;
        if (status_.recovered_joints < 255) ++status_.recovered_joints;
      }
      return;
    }
    case R_SAFE_OFF:
      status_.step = FullLegStep::SAFE_OFF_ONE;
      if (!safe_off.verifiedFor(recover_bus_)) return;  // re-requested every tick
      recover_bus_ = 0;
      ++recover_index_;
      sub_ = R_INSPECT;
      op_started_ms_ = now_ms;
      return;
    case R_FINAL:
    default: {
      // Every leg joint at q0 (<= 10) with torque off; where each rests is the
      // bystander reference from here on.
      status_.step = FullLegStep::VERIFY_REST;
      if (!sampleUsable(s)) {
        if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
        return;
      }
      if (!commonSafety(*s)) return;
      if (s->torque_enable != 0 ||
          absDiff(s->present_position, j.q0_tick) > static_cast<int32_t>(kSequenceStaticToleranceTicks)) {
        fail(FullLegFailure::INITIAL_RECOVERY_NOT_SETTLED);
        return;
      }
      PopulationState& ps = population_[recover_index_];
      ps.entry_tick = static_cast<uint16_t>(s->present_position);
      ps.has_good = true;
      ps.last_good_ms = now_ms;
      ++recover_index_;
      op_started_ms_ = now_ms;
      if (recover_index_ >= request_.population_count) {
        for (uint8_t i = 0; i < request_.population_count; ++i) population_[i].entry_known = true;
        recover_index_ = 0;
        advanceProgram(now_ms);
      }
      return;
    }
  }
}

void FullLegCalibrationExecutor::stepVerifyRest(uint32_t now_ms, const FullLegTelemetryFrame& t) {
  const uint8_t mode = program_[program_index_].slot;
  status_.step = FullLegStep::VERIFY_REST;
  if (recover_index_ >= request_.population_count) {
    recover_index_ = 0;
    advanceProgram(now_ms);
    return;
  }
  const FullLegJoint& j = request_.population[recover_index_];
  status_.op_bus = j.bus_id;
  const actuator::TelemetrySample* s = t.find(j.bus_id);
  if (!sampleUsable(s)) {
    if (now_ms - op_started_ms_ >= kSequenceMaxTelemetryAgeMs) fail(FullLegFailure::STALE_TELEMETRY);
    return;
  }
  op_started_ms_ = now_ms;
  if (!commonSafety(*s)) return;
  if (s->torque_enable != 0) {
    fail(mode == kRestPreflight ? FullLegFailure::PREFLIGHT_TORQUE_ON
                                : FullLegFailure::REST_NOT_VERIFIED);
    return;
  }
  if (mode == kRestFinal) {
    uint8_t slot = 0;
    const PopulationState& ps = population_[recover_index_];
    if (isParticipantBus(j.bus_id, &slot)) {
      if (absDiff(s->present_position, j.q0_tick) > static_cast<int32_t>(kSequenceRestToleranceTicks)) {
        fail(FullLegFailure::REST_NOT_VERIFIED);
        return;
      }
    } else if (ps.entry_known &&
               absDiff(s->present_position, ps.entry_tick) >
                   static_cast<int32_t>(kSequenceBystanderDriftTicks)) {
      fail(FullLegFailure::BYSTANDER_MOVED);
      return;
    }
  }
  ++recover_index_;
}

void FullLegCalibrationExecutor::stepSafeOffAll(const FullLegSafeOffFrame& safe_off,
                                                uint32_t now_ms) {
  status_.step = FullLegStep::SAFE_OFF_ALL;
  for (uint8_t i = 0; i < request_.population_count; ++i) {
    if (safe_off.verifiedFor(request_.population[i].bus_id)) {
      safe_off_verified_mask_ = static_cast<uint16_t>(safe_off_verified_mask_ | (1u << i));
    }
  }
  const uint16_t all = static_cast<uint16_t>((1u << request_.population_count) - 1u);
  if ((safe_off_verified_mask_ & all) != all) return;  // retried every tick until verified
  for (uint8_t s = 0; s < kSlotCount; ++s) slot_[s] = SlotState{};
  status_.held_count = 0;
  advanceProgram(now_ms);
}

void FullLegCalibrationExecutor::runDiagnostics() {
  status_.step = FullLegStep::VERIFY_REST;
  bool all = status_.contacts_accepted == kFullLegContactCount;
  for (uint8_t k = 0; k < kJointKindCount; ++k) {
    diagnostics_[k] = deriveFullLegJointDiagnostics(request_, static_cast<JointKind>(k),
                                                    contacts_[k][0], contacts_[k][1]);
    all = all && diagnostics_[k].accepted;
  }
  diagnostics_accepted_ = all;
  if (!all) {
    // Not a safety failure: the leg is still returned to q0 through the
    // reviewed RETURN phases, and the run ends FAILED.
    status_.failure = FullLegFailure::DIAGNOSTICS_REJECTED;
    status_.failed_phase = CalibrationPhase::DIAGNOSTICS;
    return_after_diagnostics_failure_ = true;
  }
}

// --- the tick ---------------------------------------------------------------------

void FullLegCalibrationExecutor::update(const FullLegCalibrationContext& context, uint32_t now_ms,
                                        const FullLegTelemetryFrame& telemetry,
                                        const FullLegSafeOffFrame& safe_off) {
  if (!active()) return;
  now_ms_ = now_ms;

  // Every tick, before any op: dynamic prerequisites, then every held joint,
  // then one bystander. Any failure here sends the run to its SAFE_OFF.
  if (!cleanup_ && !continuationOk(context)) fail(FullLegFailure::DYNAMIC_PREREQUISITE_LOST);
  if (monitoringActive() && !monitorHeld(now_ms, telemetry)) return;
  if (monitoringActive() && !monitorBystander(now_ms, telemetry)) return;

  if (program_index_ >= program_count_) {  // an empty phase (a rear leg's PARKING)
    nextPhase(now_ms);
    return;
  }

  const ProgramStep& p = program_[program_index_];
  status_.op_joint = static_cast<JointKind>(p.slot < kJointKindCount ? p.slot : 0);
  status_.op_bus = (p.op == Op::ENERGIZE || p.op == Op::MOVE || p.op == Op::PROBE ||
                    p.op == Op::HOLD || p.op == Op::RELEASE)
                       ? slotJoint(p.slot).bus_id
                       : status_.op_bus;
  switch (p.op) {
    case Op::ENERGIZE:
      stepEnergize(context, now_ms, telemetry);
      return;
    case Op::MOVE:
      stepMove(context, now_ms, telemetry);
      return;
    case Op::HOLD: {
      SlotState& st = slot_[p.slot];
      if (!st.energized) {
        fail(FullLegFailure::HELD_SET_MISMATCH);
        return;
      }
      st.held = true;
      st.fast_samples = 0;
      st.last_good_ms = now_ms;
      advanceProgram(now_ms);
      return;
    }
    case Op::RELEASE:
      slot_[p.slot].held = false;
      advanceProgram(now_ms);
      return;
    case Op::PROBE:
      stepProbe(context, now_ms, telemetry);
      return;
    case Op::DIAGNOSE:
      runDiagnostics();
      advanceProgram(now_ms);
      return;
    case Op::RECOVER_ALL:
      stepRecover(context, now_ms, telemetry, safe_off);
      return;
    case Op::VERIFY_REST:
      stepVerifyRest(now_ms, telemetry);
      return;
    case Op::SAFE_OFF_ALL:
      stepSafeOffAll(safe_off, now_ms);
      return;
  }
}

// --- names ------------------------------------------------------------------------

const char* toString(FullLegStep step) {
  switch (step) {
    case FullLegStep::IDLE:             return "IDLE";
    case FullLegStep::INSPECT:          return "INSPECT";
    case FullLegStep::PRIME:            return "PRIME";
    case FullLegStep::TORQUE_LIMIT:     return "TORQUE_LIMIT";
    case FullLegStep::TORQUE_ON:        return "TORQUE_ON";
    case FullLegStep::VERIFY_ENERGIZED: return "VERIFY_ENERGIZED";
    case FullLegStep::MOVE_WRITE:       return "MOVE_WRITE";
    case FullLegStep::MOVE_MONITOR:     return "MOVE_MONITOR";
    case FullLegStep::PROBE:            return "PROBE";
    case FullLegStep::SAFE_OFF_ONE:     return "SAFE_OFF_ONE";
    case FullLegStep::VERIFY_REST:      return "VERIFY_REST";
    case FullLegStep::SAFE_OFF_ALL:     return "SAFE_OFF_ALL";
    case FullLegStep::COMPLETE:         return "COMPLETE";
    case FullLegStep::FAILED:           return "FAILED";
  }
  return "UNKNOWN";
}

const char* toString(FullLegFailure failure) {
  switch (failure) {
    case FullLegFailure::NONE:                          return "NONE";
    case FullLegFailure::REJECT_PRECONDITIONS:          return "REJECT_PRECONDITIONS";
    case FullLegFailure::PREFLIGHT_TORQUE_ON:           return "PREFLIGHT_TORQUE_ON";
    case FullLegFailure::INITIAL_RECOVERY_OUT_OF_RANGE: return "INITIAL_RECOVERY_OUT_OF_RANGE";
    case FullLegFailure::INITIAL_RECOVERY_NOT_SETTLED:  return "INITIAL_RECOVERY_NOT_SETTLED";
    case FullLegFailure::PRIME_REJECTED:                return "PRIME_REJECTED";
    case FullLegFailure::TORQUE_LIMIT_REJECTED:         return "TORQUE_LIMIT_REJECTED";
    case FullLegFailure::TORQUE_ENABLE_REJECTED:        return "TORQUE_ENABLE_REJECTED";
    case FullLegFailure::WRITE_UNCERTAIN:               return "WRITE_UNCERTAIN";
    case FullLegFailure::ENERGIZE_NOT_VERIFIED:         return "ENERGIZE_NOT_VERIFIED";
    case FullLegFailure::MOVE_REJECTED:                 return "MOVE_REJECTED";
    case FullLegFailure::MOVE_TIMEOUT:                  return "MOVE_TIMEOUT";
    case FullLegFailure::MOVE_READBACK:                 return "MOVE_READBACK";
    case FullLegFailure::HELD_JOINT_DRIFT:              return "HELD_JOINT_DRIFT";
    case FullLegFailure::HELD_JOINT_SPEED:              return "HELD_JOINT_SPEED";
    case FullLegFailure::HELD_JOINT_READBACK:           return "HELD_JOINT_READBACK";
    case FullLegFailure::HELD_SET_MISMATCH:             return "HELD_SET_MISMATCH";
    case FullLegFailure::PASSIVE_JOINT_MOVED:           return "PASSIVE_JOINT_MOVED";
    case FullLegFailure::BYSTANDER_MOVED:               return "BYSTANDER_MOVED";
    case FullLegFailure::SERVO_STATUS_FAULT:            return "SERVO_STATUS_FAULT";
    case FullLegFailure::HARD_CURRENT_ABORT:            return "HARD_CURRENT_ABORT";
    case FullLegFailure::OVER_TEMPERATURE:              return "OVER_TEMPERATURE";
    case FullLegFailure::STALE_TELEMETRY:               return "STALE_TELEMETRY";
    case FullLegFailure::UPPER_MIN_PROBE_FAILED:        return "UPPER_MIN_PROBE_FAILED";
    case FullLegFailure::UPPER_MAX_PROBE_FAILED:        return "UPPER_MAX_PROBE_FAILED";
    case FullLegFailure::LOWER_MIN_PROBE_FAILED:        return "LOWER_MIN_PROBE_FAILED";
    case FullLegFailure::LOWER_MAX_PROBE_FAILED:        return "LOWER_MAX_PROBE_FAILED";
    case FullLegFailure::HIP_MIN_PROBE_FAILED:          return "HIP_MIN_PROBE_FAILED";
    case FullLegFailure::HIP_MAX_PROBE_FAILED:          return "HIP_MAX_PROBE_FAILED";
    case FullLegFailure::DIAGNOSTICS_REJECTED:          return "DIAGNOSTICS_REJECTED";
    case FullLegFailure::REST_NOT_VERIFIED:             return "REST_NOT_VERIFIED";
    case FullLegFailure::DYNAMIC_PREREQUISITE_LOST:     return "DYNAMIC_PREREQUISITE_LOST";
    case FullLegFailure::PHASE_REPORT_REJECTED:         return "PHASE_REPORT_REJECTED";
    case FullLegFailure::OPERATOR_ABORT:                return "OPERATOR_ABORT";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
