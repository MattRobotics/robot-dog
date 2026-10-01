#include "FullLegCalibrationFinalizer.h"

#include <stdio.h>

namespace matdog {
namespace calibration {

namespace {

// Fixed order for every loop and every export line: the V25 measurement order.
const JointKind kJointOrder[] = {JointKind::UPPER, JointKind::LOWER, JointKind::HIP};
constexpr uint8_t kJointOrderCount = 3;
const ContactSide kSideOrder[] = {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE};

const FullLegJointRef& planJoint(const FullLegPlan& plan, JointKind kind) {
  switch (kind) {
    case JointKind::UPPER: return plan.upper;
    case JointKind::HIP:   return plan.hip;
    case JointKind::LOWER: return plan.lower;
  }
  return plan.upper;
}

bool contextComplete(const FullLegFinalizeContext& c) {
  return c.manager != nullptr && c.policy != nullptr && c.geometry != nullptr &&
         c.expected_provenance != nullptr && c.permit != nullptr && c.authorization != nullptr &&
         c.arbiter != nullptr;
}

bool contactMatches(const ContactEvidence& evidence, Leg leg, JointKind joint, ContactSide side) {
  return evidence.has_measurement && evidence.key.leg == leg && evidence.key.joint == joint &&
         evidence.key.side == side;
}

// The session cause is coarse metadata for @CALIBRATION STATUS. The record's
// executor_failure and failure fields are the precise account.
CalibrationFailure sessionCauseFor(FullLegFinalizeFailure failure, FullLegFailure executor_failure) {
  switch (failure) {
    case FullLegFinalizeFailure::EXECUTOR_FAILED:
      switch (executor_failure) {
        case FullLegFailure::HARD_CURRENT_ABORT:
          return CalibrationFailure::HARD_CURRENT_ABORT;
        case FullLegFailure::STALE_TELEMETRY:
          return CalibrationFailure::TELEMETRY_STALE;
        case FullLegFailure::MOVE_TIMEOUT:
          return CalibrationFailure::MOTION_TIMEOUT;
        case FullLegFailure::HELD_JOINT_DRIFT:
        case FullLegFailure::PASSIVE_JOINT_MOVED:
        case FullLegFailure::BYSTANDER_MOVED:
          return CalibrationFailure::STATIC_JOINT_MOVED;
        case FullLegFailure::UPPER_MIN_PROBE_FAILED:
        case FullLegFailure::UPPER_MAX_PROBE_FAILED:
        case FullLegFailure::LOWER_MIN_PROBE_FAILED:
        case FullLegFailure::LOWER_MAX_PROBE_FAILED:
        case FullLegFailure::HIP_MIN_PROBE_FAILED:
        case FullLegFailure::HIP_MAX_PROBE_FAILED:
          return CalibrationFailure::CONTACT_WITNESS_REJECTED;
        case FullLegFailure::DIAGNOSTICS_REJECTED:
          return CalibrationFailure::AFFINE_GATE_REJECTED;
        case FullLegFailure::OPERATOR_ABORT:
          return CalibrationFailure::OPERATOR_ABORT;
        default:
          return CalibrationFailure::AUTHORITY_LOST;
      }
    case FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED:
    case FullLegFinalizeFailure::CONTACTS_INCOMPLETE:
    case FullLegFinalizeFailure::CONTACT_REJECTED:
      return CalibrationFailure::CONTACT_WITNESS_REJECTED;
    case FullLegFinalizeFailure::DIAGNOSTICS_REJECTED:
    case FullLegFinalizeFailure::ENVELOPE_NOT_READY:
    case FullLegFinalizeFailure::LIMIT_ADMISSION_REJECTED:
    case FullLegFinalizeFailure::LIMIT_VERIFY_FAILED:
      return CalibrationFailure::AFFINE_GATE_REJECTED;
    default:
      return CalibrationFailure::AUTHORITY_LOST;
  }
}

void fillStatic(const FullLegFinalizeContext& context, const FullLegPlan& plan,
                const FullLegRunOutcome& outcome, FullLegRecord* record) {
  record->leg = plan.leg;
  record->present = true;
  record->session_id = outcome.session_id_at_start;
  record->geometry = outcome.geometry_at_start;
  record->executor_failure = outcome.failure;
  record->executor_failed_phase = outcome.failed_phase;
  record->has_rear_park = plan.request.has_rear_park;
  if (plan.request.has_rear_park) {
    record->park_leg = plan.request.park.identity.leg;
    record->park_joint = plan.request.park.identity.joint;
    record->park_bus_id = plan.request.park.bus_id;
    record->park_target_urad = plan.request.park_target_urad;
  }
  record->parameters_approved = context.parameters.approved;
  record->contact_margin_ticks = context.parameters.contact_margin_ticks;
  record->contacts_expected = kFullLegContactsExpected;
  record->contacts_measured = outcome.contacts_measured;
  record->diagnostics_accepted = outcome.diagnostics_accepted;

  for (uint8_t i = 0; i < kJointOrderCount; ++i) {
    const JointKind kind = kJointOrder[i];
    const uint8_t k = static_cast<uint8_t>(kind);
    FullLegJointRecord& joint = record->joint(kind);
    joint.identity = planJoint(plan, kind).identity;
    joint.bus_id = planJoint(plan, kind).bus_id;
    joint.contact[0] = outcome.contacts[k][0];
    joint.contact[1] = outcome.contacts[k][1];
    joint.diagnostics = outcome.diagnostics[k];
    if (context.policy != nullptr) {
      const actuator::JointTransform* transform =
          context.policy->transforms().find(joint.identity, context.policy->currentGeometryTag());
      if (transform != nullptr) {
        joint.q0_present = transform->present;
        joint.q0_state = transform->state;
        joint.q0_origin = transform->origin;
        joint.q0_geometry = transform->geometry;
        joint.q0_tick = transform->q0_tick;
      }
    }
  }
}

// Steps 3-7. Returns the first refusal, or NONE with the session COMPLETED.
FullLegFinalizeFailure runLifecycle(const FullLegFinalizeContext& context, const FullLegPlan& plan,
                                    const FullLegRunOutcome& outcome, FullLegRecord* record) {
  CalibrationManager& manager = *context.manager;
  actuator::SafeActuatorPolicy& policy = *context.policy;

  if (!outcome.complete) return FullLegFinalizeFailure::EXECUTOR_FAILED;

  const CalibrationSessionStatus& session = manager.status();
  if (session.session_id != outcome.session_id_at_start || session.leg != plan.leg ||
      session.origin != CalibrationOrigin::LIVE_SESSION) {
    return FullLegFinalizeFailure::SESSION_MISMATCH;
  }
  if (session.state != SessionState::ACTIVE) return FullLegFinalizeFailure::SESSION_NOT_ACTIVE;

  // --- 3. six contacts, each for this leg's exact (joint, side) ------------
  uint8_t measured = 0;
  for (uint8_t i = 0; i < kJointOrderCount; ++i) {
    for (const ContactSide side : kSideOrder) {
      const JointKind kind = kJointOrder[i];
      const ContactEvidence& e =
          outcome.contacts[static_cast<uint8_t>(kind)][static_cast<uint8_t>(side)];
      if (!e.has_measurement) continue;
      if (!contactMatches(e, plan.leg, kind, side)) {
        return FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED;
      }
      ++measured;
    }
  }
  if (measured != kFullLegContactsExpected || outcome.contacts_measured != kFullLegContactsExpected) {
    return FullLegFinalizeFailure::CONTACTS_INCOMPLETE;
  }
  if (!outcome.diagnostics_accepted) return FullLegFinalizeFailure::DIAGNOSTICS_REJECTED;
  for (uint8_t i = 0; i < kJointOrderCount; ++i) {
    if (!outcome.diagnostics[static_cast<uint8_t>(kJointOrder[i])].accepted) {
      return FullLegFinalizeFailure::DIAGNOSTICS_REJECTED;
    }
  }

  // --- 4. record all six, each exactly once --------------------------------
  for (uint8_t i = 0; i < kJointOrderCount; ++i) {
    FullLegJointRecord& joint = record->joint(kJointOrder[i]);
    for (uint8_t s = 0; s < kContactSideCount; ++s) {
      if (!manager.recordContact(joint.contact[s])) return FullLegFinalizeFailure::CONTACT_REJECTED;
      joint.contact_recorded[s] = true;
      ++record->contacts_accepted;
    }
  }

  // --- 5. envelopes, each from its own two contacts --------------------------
  for (uint8_t i = 0; i < kJointOrderCount; ++i) {
    FullLegJointRecord& joint = record->joint(kJointOrder[i]);
    actuator::ContactDerivedEnvelopeRequest request{};
    request.joint = joint.identity;
    request.min_side_evidence = joint.contact[0];
    request.max_side_evidence = joint.contact[1];
    request.min_side_geometry = outcome.geometry_at_start;
    request.max_side_geometry = outcome.geometry_at_start;
    request.safety_margin_ticks = context.parameters.contact_margin_ticks;
    joint.envelope_status = actuator::buildContactDerivedEnvelope(
        *context.geometry, *context.expected_provenance, request, &joint.envelope);
    if (joint.envelope_status != actuator::EnvelopeBuildStatus::READY) {
      return FullLegFinalizeFailure::ENVELOPE_NOT_READY;
    }
  }

  // --- 6. limits -------------------------------------------------------------
  const actuator::GeometryProvenanceTag current = policy.currentGeometryTag();
  if (!context.parameters.approved) {
    // A placeholder envelope is reported, never offered to the policy.
    for (uint8_t i = 0; i < kJointOrderCount; ++i) {
      record->joint(kJointOrder[i]).limit = FullLegLimitAdmission::NOT_ADMITTED_UNAPPROVED_PARAMETERS;
    }
  } else {
    actuator::JointLimit limits[kJointOrderCount];
    // Validate the whole set first: a late refusal must not leave part of a
    // leg's bounds in the table.
    for (uint8_t i = 0; i < kJointOrderCount; ++i) {
      FullLegJointRecord& joint = record->joint(kJointOrder[i]);
      limits[i].identity = joint.identity;
      limits[i].state = EvidenceState::PROMOTED;
      limits[i].origin = CalibrationOrigin::LIVE_SESSION;
      limits[i].geometry = joint.envelope.geometry;
      limits[i].min_tick = joint.envelope.min_tick;
      limits[i].max_tick = joint.envelope.max_tick;
      limits[i].present = joint.envelope.present;
      const actuator::LimitAdmission verdict = policy.validateOperationalLimit(limits[i]);
      if (verdict != actuator::LimitAdmission::ADMITTED) {
        joint.limit = FullLegLimitAdmission::REJECTED_BY_POLICY;
        joint.limit_policy_detail = verdict;
        return FullLegFinalizeFailure::LIMIT_ADMISSION_REJECTED;
      }
    }
    for (uint8_t i = 0; i < kJointOrderCount; ++i) {
      FullLegJointRecord& joint = record->joint(kJointOrder[i]);
      const actuator::LimitAdmission verdict = policy.admitOperationalLimit(limits[i]);
      if (verdict != actuator::LimitAdmission::ADMITTED) {
        joint.limit = FullLegLimitAdmission::REJECTED_BY_POLICY;
        joint.limit_policy_detail = verdict;
        return FullLegFinalizeFailure::LIMIT_ADMISSION_REJECTED;
      }
    }
    for (uint8_t i = 0; i < kJointOrderCount; ++i) {
      FullLegJointRecord& joint = record->joint(kJointOrder[i]);
      const actuator::JointLimit* found = policy.limits().find(joint.identity, current);
      if (found == nullptr || !found->present || found->min_tick != limits[i].min_tick ||
          found->max_tick != limits[i].max_tick) {
        joint.limit = FullLegLimitAdmission::VERIFY_FAILED;
        return FullLegFinalizeFailure::LIMIT_VERIFY_FAILED;
      }
      joint.limit = FullLegLimitAdmission::ADMITTED;
    }
  }

  // --- 7. session close --------------------------------------------------------
  // The executor reported every phase live; TORQUE_OFF is its last one, and a
  // repeat of it is legal only if it has not been reported yet.
  if (manager.status().last_reported_phase != CalibrationPhase::TORQUE_OFF &&
      !manager.noteExecutionPhase(CalibrationPhase::TORQUE_OFF)) {
    return FullLegFinalizeFailure::PHASE_REPORT_REJECTED;
  }
  if (!manager.completeSession()) return FullLegFinalizeFailure::SESSION_COMPLETE_REJECTED;
  return FullLegFinalizeFailure::NONE;
}

}  // namespace

FullLegEnvelopeParameters productionEnvelopeParameters() {
  FullLegEnvelopeParameters parameters{};
  parameters.approved = kFullLegOperationalParametersApproved;
  parameters.contact_margin_ticks = kFullLegPlaceholderContactMarginTicks;
  return parameters;
}

FullLegRunOutcome outcomeFromExecutor(const FullLegCalibrationExecutor& executor,
                                      actuator::GeometryProvenanceTag geometry_at_start,
                                      uint32_t session_id_at_start) {
  FullLegRunOutcome outcome{};
  const FullLegStep step = executor.status().step;
  outcome.terminal = step == FullLegStep::COMPLETE || step == FullLegStep::FAILED;
  outcome.complete = step == FullLegStep::COMPLETE;
  outcome.failure = executor.status().failure;
  outcome.failed_phase = executor.status().failed_phase;
  outcome.contacts_measured = executor.status().contacts_accepted;
  for (uint8_t k = 0; k < kJointKindCount; ++k) {
    for (uint8_t s = 0; s < kContactSideCount; ++s) {
      outcome.contacts[k][s] = executor.contact(static_cast<JointKind>(k), static_cast<ContactSide>(s));
    }
    outcome.diagnostics[k] = executor.diagnostics(static_cast<JointKind>(k));
  }
  outcome.diagnostics_accepted = executor.diagnosticsAccepted();
  outcome.geometry_at_start = geometry_at_start;
  outcome.session_id_at_start = session_id_at_start;
  return outcome;
}

FullLegFinalizeFailure finalizeFullLeg(const FullLegFinalizeContext& context,
                                       const FullLegPlan& plan, const FullLegRunOutcome& outcome,
                                       FullLegRecord* record) {
  if (record == nullptr) return FullLegFinalizeFailure::CONTEXT_INCOMPLETE;
  *record = FullLegRecord{};
  record->leg = plan.leg;

  // Not terminal: the executor is still moving hardware. Touch nothing.
  if (!outcome.terminal) {
    record->failure = FullLegFinalizeFailure::RUN_NOT_TERMINAL;
    return record->failure;
  }

  fillStatic(context, plan, outcome, record);

  FullLegFinalizeFailure failure = FullLegFinalizeFailure::CONTEXT_INCOMPLETE;
  bool session_is_ours = false;
  if (contextComplete(context)) {
    failure = runLifecycle(context, plan, outcome, record);

    const CalibrationSessionStatus& session = context.manager->status();
    session_is_ours = session.session_id == outcome.session_id_at_start &&
                      session.leg == plan.leg && session.origin == CalibrationOrigin::LIVE_SESSION;
    if (failure != FullLegFinalizeFailure::NONE && session_is_ours && context.manager->sessionLive()) {
      if (failure == FullLegFinalizeFailure::EXECUTOR_FAILED &&
          outcome.failure == FullLegFailure::OPERATOR_ABORT) {
        context.manager->abortSession();
      } else {
        context.manager->failSession(sessionCauseFor(failure, outcome.failure));
      }
    }
  }

  // --- 8. permit and operator authorization go on every path -----------------
  if (context.permit != nullptr) context.permit->revoke(CalibrationPermitRevokeReason::EXPLICIT);
  if (context.authorization != nullptr) context.authorization->revoke();

  // --- 9. post-conditions ------------------------------------------------------
  if (context.manager != nullptr) {
    record->session_completed = session_is_ours && context.manager->status().state == SessionState::COMPLETED;
  }
  record->permit_revoked = context.permit != nullptr && context.authorization != nullptr &&
                           !context.permit->active() && !context.authorization->operator_authorized;
  record->authority_released = context.manager != nullptr && context.arbiter != nullptr &&
                               !context.manager->status().holds_authority &&
                               context.arbiter->current() == core::ActuatorAuthority::NONE;

  if (failure == FullLegFinalizeFailure::NONE) {
    if (!record->session_completed) {
      failure = FullLegFinalizeFailure::CLEANUP_SESSION_NOT_TERMINAL;
    } else if (!record->authority_released) {
      failure = FullLegFinalizeFailure::CLEANUP_AUTHORITY_HELD;
    } else if (!record->permit_revoked) {
      failure = FullLegFinalizeFailure::CLEANUP_PERMIT_ACTIVE;
    }
  }

  record->failure = failure;
  // 6/6 recorded contacts or nothing: a failed finalization reports none accepted.
  if (failure != FullLegFinalizeFailure::NONE || record->contacts_accepted != kFullLegContactsExpected) {
    record->verdict = FullLegVerdict::FAILED;
    record->contacts_accepted = 0;
    if (failure == FullLegFinalizeFailure::NONE) failure = FullLegFinalizeFailure::CONTACTS_INCOMPLETE;
    record->failure = failure;
    return failure;
  }

  record->hardware_contact_calibrated = true;
  bool all_admitted = context.parameters.approved;
  for (uint8_t i = 0; i < kJointOrderCount; ++i) {
    if (record->joint(kJointOrder[i]).limit != FullLegLimitAdmission::ADMITTED) all_admitted = false;
  }
  record->operational_envelope_accepted = all_admitted;
  record->verdict = all_admitted ? FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED
                                 : FullLegVerdict::HARDWARE_CONTACT_CALIBRATED;
  return FullLegFinalizeFailure::NONE;
}

// ---------------------------------------------------------------------------
// Store
// ---------------------------------------------------------------------------

void FullLegEvidenceStore::reset() {
  for (uint8_t i = 0; i < kLegCount; ++i) records_[i] = FullLegRecord{};
}

void FullLegEvidenceStore::put(const FullLegRecord& record) {
  if (!record.present || !isKnownLeg(record.leg)) return;
  const uint8_t index = static_cast<uint8_t>(record.leg);
  const uint16_t attempts = static_cast<uint16_t>(records_[index].attempts + 1);
  records_[index] = record;
  records_[index].attempts = attempts;
}

const FullLegRecord* FullLegEvidenceStore::find(Leg leg) const {
  if (!isKnownLeg(leg)) return nullptr;
  const FullLegRecord& slot = records_[static_cast<uint8_t>(leg)];
  return slot.present ? &slot : nullptr;
}

uint8_t FullLegEvidenceStore::legsPresent() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kLegCount; ++i) n += records_[i].present ? 1 : 0;
  return n;
}

uint8_t FullLegEvidenceStore::legsContactCalibrated() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kLegCount; ++i) {
    n += (records_[i].hardware_contact_calibrated &&
          records_[i].contacts_accepted == kFullLegContactsExpected)
             ? 1
             : 0;
  }
  return n;
}

uint8_t FullLegEvidenceStore::legsEnvelopeAccepted() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kLegCount; ++i) n += records_[i].operational_envelope_accepted ? 1 : 0;
  return n;
}

uint8_t FullLegEvidenceStore::totalContactsAccepted() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kLegCount; ++i) {
    if (records_[i].present && records_[i].hardware_contact_calibrated) {
      n = static_cast<uint8_t>(n + records_[i].contacts_accepted);
    }
  }
  return n;
}

// ---------------------------------------------------------------------------
// Export
// ---------------------------------------------------------------------------

namespace {

// Every line below stays well under this with the LONGEST enum names
// (host-tested); snprintf would otherwise truncate a line silently.
constexpr size_t kLineBytes = 512;

void tagHex(actuator::GeometryProvenanceTag tag, char* out, size_t out_size) {
  snprintf(out, out_size, "%08lx%08lx", static_cast<unsigned long>(tag >> 32),
           static_cast<unsigned long>(tag & 0xFFFFFFFFULL));
}

const char* unitOf(const JointIdentity& identity) {
  return identity.unitKnown() ? identity.physical_unit : "-";
}

}  // namespace

void exportFullLegEvidence(const FullLegEvidenceStore& store,
                           actuator::GeometryProvenanceTag current_geometry,
                           const FullLegEnvelopeParameters& parameters, FullLegExportSink sink,
                           void* user) {
  if (sink == nullptr) return;
  char line[kLineBytes];
  char hex[20];
  char hex2[20];

  tagHex(current_geometry, hex, sizeof hex);
  snprintf(line, sizeof line,
           "CALIBRATION_EVIDENCE_EXPORT=BEGIN format=2 geometry=%s parameters_approved=%u "
           "contact_margin_ticks=%u contacts_per_leg=%u total_contacts_expected=%u",
           hex, parameters.approved ? 1u : 0u,
           static_cast<unsigned>(parameters.contact_margin_ticks),
           static_cast<unsigned>(kFullLegContactsExpected),
           static_cast<unsigned>(kFullCalibrationContactsExpected));
  sink(user, line);

  const Leg legs[] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
  for (uint8_t li = 0; li < kLegCount; ++li) {
    const Leg leg = legs[li];
    const FullLegRecord* record = store.find(leg);
    if (record == nullptr) {
      snprintf(line, sizeof line,
               "CALIBRATION_EVIDENCE_LEG leg=%s present=0 attempts=0 verdict=%s "
               "contacts_expected=%u contacts_accepted=0",
               toString(leg), toString(FullLegVerdict::NOT_RUN),
               static_cast<unsigned>(kFullLegContactsExpected));
      sink(user, line);
      continue;
    }

    tagHex(record->geometry, hex, sizeof hex);
    snprintf(line, sizeof line,
             "CALIBRATION_EVIDENCE_LEG leg=%s present=1 attempts=%u session=%lu geometry=%s "
             "verdict=%s contacts_expected=%u contacts_measured=%u contacts_accepted=%u "
             "diagnostics_accepted=%u contact_calibrated=%u envelope_accepted=%u",
             toString(leg), static_cast<unsigned>(record->attempts),
             static_cast<unsigned long>(record->session_id), hex, toString(record->verdict),
             static_cast<unsigned>(record->contacts_expected),
             static_cast<unsigned>(record->contacts_measured),
             static_cast<unsigned>(record->contacts_accepted),
             record->diagnostics_accepted ? 1u : 0u,
             record->hardware_contact_calibrated ? 1u : 0u,
             record->operational_envelope_accepted ? 1u : 0u);
    sink(user, line);
    snprintf(line, sizeof line,
             "CALIBRATION_EVIDENCE_LEG_CLOSE leg=%s failure=%s executor_failure=%s failed_phase=%s "
             "session_completed=%u permit_revoked=%u authority_released=%u parameters_approved=%u",
             toString(leg), toString(record->failure), toString(record->executor_failure),
             record->executor_failure == FullLegFailure::NONE ? "-"
                                                              : toString(record->executor_failed_phase),
             record->session_completed ? 1u : 0u, record->permit_revoked ? 1u : 0u,
             record->authority_released ? 1u : 0u, record->parameters_approved ? 1u : 0u);
    sink(user, line);

    if (record->has_rear_park) {
      snprintf(line, sizeof line,
               "CALIBRATION_EVIDENCE_PARK leg=%s required=1 park_leg=%s park_joint=%s park_bus=%u "
               "park_target_urad=%ld",
               toString(leg), toString(record->park_leg), toString(record->park_joint),
               static_cast<unsigned>(record->park_bus_id),
               static_cast<long>(record->park_target_urad));
    } else {
      snprintf(line, sizeof line, "CALIBRATION_EVIDENCE_PARK leg=%s required=0", toString(leg));
    }
    sink(user, line);

    for (uint8_t ji = 0; ji < kJointOrderCount; ++ji) {
      const JointKind kind = kJointOrder[ji];
      const FullLegJointRecord& joint = record->joint(kind);
      tagHex(joint.q0_geometry, hex2, sizeof hex2);
      snprintf(line, sizeof line,
               "CALIBRATION_EVIDENCE_Q0 leg=%s joint=%s unit=%s bus=%u present=%u q0_tick=%u "
               "state=%s origin=%s geometry=%s",
               toString(leg), toString(kind), unitOf(joint.identity),
               static_cast<unsigned>(joint.bus_id), joint.q0_present ? 1u : 0u,
               static_cast<unsigned>(joint.q0_tick), toString(joint.q0_state),
               toString(joint.q0_origin), hex2);
      sink(user, line);
    }

    for (uint8_t ji = 0; ji < kJointOrderCount; ++ji) {
      const JointKind kind = kJointOrder[ji];
      const FullLegJointRecord& joint = record->joint(kind);
      for (uint8_t si = 0; si < kContactSideCount; ++si) {
        const ContactEvidence& c = joint.contact[si];
        snprintf(line, sizeof line,
                 "CALIBRATION_EVIDENCE_CONTACT leg=%s joint=%s side=%s recorded=%u measured=%u "
                 "detection=%s state=%s origin=%s scout_tick=%u fine1_tick=%u fine2_tick=%u "
                 "repeatability_ticks=%u witness_accepted=%u",
                 toString(leg), toString(kind), si == 0 ? "MIN" : "MAX",
                 joint.contact_recorded[si] ? 1u : 0u, c.has_measurement ? 1u : 0u,
                 toString(c.detection), toString(c.state), toString(c.origin),
                 static_cast<unsigned>(c.coarse_tick), static_cast<unsigned>(c.fine_tick_1),
                 static_cast<unsigned>(c.fine_tick_2),
                 static_cast<unsigned>(c.repeatability_ticks), c.witness.accepted() ? 1u : 0u);
        sink(user, line);
      }
      const FullLegJointDiagnostics& d = joint.diagnostics;
      snprintf(line, sizeof line,
               "CALIBRATION_EVIDENCE_DIAG leg=%s joint=%s evaluated=%u min_contact=%u max_contact=%u "
               "ordered=%u span=%u/%u scale_permille=%u affine_q0=%u q0_shift=%u "
               "fixed_disagreement=%u accepted=%u",
               toString(leg), toString(kind), d.evaluated ? 1u : 0u,
               static_cast<unsigned>(d.min_contact_tick), static_cast<unsigned>(d.max_contact_tick),
               d.ordered ? 1u : 0u, static_cast<unsigned>(d.measured_span_ticks),
               static_cast<unsigned>(d.expected_span_ticks), static_cast<unsigned>(d.scale_permille),
               static_cast<unsigned>(d.affine_zero_tick),
               static_cast<unsigned>(d.affine_shift_from_q0_ticks),
               static_cast<unsigned>(d.fixed_endpoint_disagreement_ticks), d.accepted ? 1u : 0u);
      sink(user, line);
    }

    for (uint8_t ji = 0; ji < kJointOrderCount; ++ji) {
      const JointKind kind = kJointOrder[ji];
      const FullLegJointRecord& joint = record->joint(kind);
      snprintf(line, sizeof line,
               "CALIBRATION_EVIDENCE_ENVELOPE leg=%s joint=%s source=%s status=%s present=%u "
               "min_tick=%u max_tick=%u placeholder=%u",
               toString(leg), toString(kind), actuator::toString(joint.envelope.source),
               actuator::toString(joint.envelope_status), joint.envelope.present ? 1u : 0u,
               static_cast<unsigned>(joint.envelope.min_tick),
               static_cast<unsigned>(joint.envelope.max_tick), record->parameters_approved ? 0u : 1u);
      sink(user, line);

      snprintf(line, sizeof line, "CALIBRATION_EVIDENCE_LIMIT leg=%s joint=%s admission=%s policy=%s",
               toString(leg), toString(kind), toString(joint.limit),
               joint.limit == FullLegLimitAdmission::REJECTED_BY_POLICY
                   ? actuator::toString(joint.limit_policy_detail)
                   : "-");
      sink(user, line);
    }
  }

  snprintf(line, sizeof line,
           "CALIBRATION_EVIDENCE_EXPORT=END legs_present=%u legs_contact_calibrated=%u "
           "legs_envelope_accepted=%u total_contacts_expected=%u total_contacts_accepted=%u "
           "all_contact_calibrated=%u",
           static_cast<unsigned>(store.legsPresent()),
           static_cast<unsigned>(store.legsContactCalibrated()),
           static_cast<unsigned>(store.legsEnvelopeAccepted()),
           static_cast<unsigned>(kFullCalibrationContactsExpected),
           static_cast<unsigned>(store.totalContactsAccepted()),
           store.allLegsContactCalibrated() ? 1u : 0u);
  sink(user, line);
}

const char* toString(FullLegVerdict verdict) {
  switch (verdict) {
    case FullLegVerdict::NOT_RUN:                             return "NOT_RUN";
    case FullLegVerdict::FAILED:                              return "FAILED";
    case FullLegVerdict::HARDWARE_CONTACT_CALIBRATED:         return "HARDWARE_CONTACT_CALIBRATED";
    case FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED: return "FINAL_OPERATIONAL_ENVELOPE_ACCEPTED";
  }
  return "UNKNOWN";
}

const char* toString(FullLegLimitAdmission admission) {
  switch (admission) {
    case FullLegLimitAdmission::NOT_EVALUATED:                       return "NOT_EVALUATED";
    case FullLegLimitAdmission::ADMITTED:                            return "ADMITTED";
    case FullLegLimitAdmission::NOT_ADMITTED_UNAPPROVED_PARAMETERS:  return "NOT_ADMITTED_UNAPPROVED_PARAMETERS";
    case FullLegLimitAdmission::REJECTED_BY_POLICY:                  return "REJECTED_BY_POLICY";
    case FullLegLimitAdmission::VERIFY_FAILED:                       return "VERIFY_FAILED";
  }
  return "UNKNOWN";
}

const char* toString(FullLegFinalizeFailure failure) {
  switch (failure) {
    case FullLegFinalizeFailure::NONE:                         return "NONE";
    case FullLegFinalizeFailure::CONTEXT_INCOMPLETE:           return "CONTEXT_INCOMPLETE";
    case FullLegFinalizeFailure::RUN_NOT_TERMINAL:             return "RUN_NOT_TERMINAL";
    case FullLegFinalizeFailure::EXECUTOR_FAILED:              return "EXECUTOR_FAILED";
    case FullLegFinalizeFailure::SESSION_NOT_ACTIVE:           return "SESSION_NOT_ACTIVE";
    case FullLegFinalizeFailure::SESSION_MISMATCH:             return "SESSION_MISMATCH";
    case FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED:   return "CONTACT_EVIDENCE_MALFORMED";
    case FullLegFinalizeFailure::CONTACTS_INCOMPLETE:          return "CONTACTS_INCOMPLETE";
    case FullLegFinalizeFailure::DIAGNOSTICS_REJECTED:         return "DIAGNOSTICS_REJECTED";
    case FullLegFinalizeFailure::CONTACT_REJECTED:             return "CONTACT_REJECTED";
    case FullLegFinalizeFailure::ENVELOPE_NOT_READY:           return "ENVELOPE_NOT_READY";
    case FullLegFinalizeFailure::LIMIT_ADMISSION_REJECTED:     return "LIMIT_ADMISSION_REJECTED";
    case FullLegFinalizeFailure::LIMIT_VERIFY_FAILED:          return "LIMIT_VERIFY_FAILED";
    case FullLegFinalizeFailure::PHASE_REPORT_REJECTED:        return "PHASE_REPORT_REJECTED";
    case FullLegFinalizeFailure::SESSION_COMPLETE_REJECTED:    return "SESSION_COMPLETE_REJECTED";
    case FullLegFinalizeFailure::CLEANUP_SESSION_NOT_TERMINAL: return "CLEANUP_SESSION_NOT_TERMINAL";
    case FullLegFinalizeFailure::CLEANUP_AUTHORITY_HELD:       return "CLEANUP_AUTHORITY_HELD";
    case FullLegFinalizeFailure::CLEANUP_PERMIT_ACTIVE:        return "CLEANUP_PERMIT_ACTIVE";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
