#include "CalibrationQ0EvidencePreparation.h"

namespace matdog {
namespace actuator {

Q0EvidencePackageFacts frozenQ0EvidencePackageFacts() {
  Q0EvidencePackageFacts facts{};
  facts.formal_population_pass = q0_evidence_data::kFormalPopulationPass;
  facts.nominal_zero_pose_confirmed = q0_evidence_data::kNominalZeroPoseConfirmed;
  facts.torque_off_verified_before_capture =
      q0_evidence_data::kTorqueOffVerifiedBeforeCapture;
  return facts;
}

Q0EvidencePreparationStatus validateQ0EvidencePackageFacts(
    const Q0EvidencePackageFacts& facts) {
  if (!facts.formal_population_pass) {
    return Q0EvidencePreparationStatus::REJECT_CAPTURE_POPULATION_NOT_PASS;
  }
  if (!facts.nominal_zero_pose_confirmed) {
    return Q0EvidencePreparationStatus::REJECT_CAPTURE_Q0_POSE_NOT_CONFIRMED;
  }
  if (!facts.torque_off_verified_before_capture) {
    return Q0EvidencePreparationStatus::REJECT_CAPTURE_TORQUE_NOT_OFF;
  }
  return Q0EvidencePreparationStatus::READY;
}

namespace {

uint16_t circularDistanceFromCenter(uint16_t tick) {
  int32_t d = static_cast<int32_t>(tick) -
              static_cast<int32_t>(calibration::kServoRawCenter);
  if (d < 0) d = -d;
  const int32_t wrapped = kTicksPerRevolution - d;
  if (wrapped < d) d = wrapped;
  return static_cast<uint16_t>(d);
}

Q0BootstrapCandidate candidateFromFrozenRecord(
    const q0_evidence_data::Q0CandidateRecord& r) {
  Q0BootstrapCandidate c{};
  c.status = Q0BootstrapStatus::CANDIDATE;
  c.geometry = geometryProvenanceTag(q0_evidence_data::kSourceGeometry);
  c.bus_id = r.bus_id;
  c.capture_session_id = q0_evidence_data::kCaptureSessionId;
  c.sample_count = q0_evidence_data::kSampleCount;
  c.stability_spread_ticks = q0_evidence_data::kStabilitySpreadTicks;

  c.evidence.measured = true;
  c.evidence.estimator = calibration::Q0Estimator::MANUAL_ZERO_POSE;
  c.evidence.state = calibration::EvidenceState::CANDIDATE;
  c.evidence.origin = calibration::CalibrationOrigin::LIVE_SESSION;
  c.evidence.identity.leg = r.leg;
  c.evidence.identity.joint = r.joint;
  calibration::setPhysicalUnit(&c.evidence.identity, r.physical_unit);
  c.evidence.tick = r.q0_tick;
  c.evidence.shift_from_digital_home_ticks = circularDistanceFromCenter(r.q0_tick);
  c.evidence.endpoint_disagreement_ticks = 0;
  c.evidence.scale_permille = 0;
  c.evidence.accepted_by_gate = false;
  return c;
}

bool sameRecordIdentity(const q0_evidence_data::Q0CandidateRecord& a,
                        const q0_evidence_data::Q0CandidateRecord& b) {
  if (a.leg != b.leg || a.joint != b.joint) return false;
  for (uint8_t i = 0; i < calibration::kPhysicalUnitLabelBytes; ++i) {
    const char ac = a.physical_unit[i];
    const char bc = b.physical_unit[i];
    if (ac != bc) return false;
    if (ac == '\0') break;
  }
  return true;
}

bool sameCandidateSlot(const Q0BootstrapCandidate& a, const Q0BootstrapCandidate& b) {
  return calibration::identityPermitsEvidenceReuse(a.evidence.identity, b.evidence.identity) ||
         a.bus_id == b.bus_id;
}

// The single acceptance -> promotion step shared by the fresh and the frozen
// path, so neither can drift from the CR3 rules.
Q0EvidencePreparationStatus acceptAndPromote(
    const CalibrationGeometryProfile& current_profile,
    const GeometryProvenance& expected_current_geometry,
    const Q0BootstrapCandidate& candidate, uint32_t capture_session_id,
    JointTransform* transform_out) {
  const AcceptedQ0 accepted =
      acceptQ0Candidate(current_profile, expected_current_geometry, candidate);
  if (!accepted.accepted()) return Q0EvidencePreparationStatus::REJECT_CANDIDATE;

  Q0PromotionRequest request{};
  request.explicit_currentness_confirmation = true;
  request.capture_session_id = capture_session_id;
  const PromotedQ0 promoted =
      promoteAcceptedQ0(current_profile, expected_current_geometry, accepted, request);
  if (!promoted.promoted()) return Q0EvidencePreparationStatus::REJECT_PROMOTION;

  *transform_out = promoted.transform;
  return Q0EvidencePreparationStatus::READY;
}

}  // namespace

Q0EvidencePreparation prepareFreshQ0Evidence(
    const CalibrationGeometryProfile& current_profile,
    const GeometryProvenance& expected_current_geometry,
    const FreshQ0Capture& capture,
    bool explicit_current_installation_confirmation) {
  Q0EvidencePreparation out{};

  if (!explicit_current_installation_confirmation) {
    out.status =
        Q0EvidencePreparationStatus::REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED;
    return out;
  }

  if (!capture.complete || capture.candidates == nullptr ||
      capture.candidate_count != calibration::kLegServoSlotCount ||
      capture.capture_session_id == 0) {
    out.status = Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE;
    return out;
  }
  if (!capture.population_pass) {
    out.status = Q0EvidencePreparationStatus::REJECT_CAPTURE_POPULATION_NOT_PASS;
    return out;
  }

  if (!current_profile.bound() ||
      !current_profile.provenanceMatches(expected_current_geometry)) {
    out.status = Q0EvidencePreparationStatus::REJECT_SOURCE_GEOMETRY;
    return out;
  }

  for (uint8_t i = 0; i < capture.candidate_count; ++i) {
    for (uint8_t j = 0; j < i; ++j) {
      if (sameCandidateSlot(capture.candidates[i], capture.candidates[j])) {
        out.status = Q0EvidencePreparationStatus::REJECT_RECORD_DUPLICATE;
        out.transform_count = 0;
        out.failed_record_index = i;
        return out;
      }
    }

    const Q0EvidencePreparationStatus step =
        acceptAndPromote(current_profile, expected_current_geometry,
                         capture.candidates[i], capture.capture_session_id,
                         &out.transforms[out.transform_count]);
    if (step != Q0EvidencePreparationStatus::READY) {
      out.status = step;
      out.transform_count = 0;
      out.failed_record_index = i;
      return out;
    }
    ++out.transform_count;
  }

  out.status = Q0EvidencePreparationStatus::READY;
  return out;
}

bool freshQ0CaptureIsPromoted(const FreshQ0Capture& capture,
                              const JointTransformTable& transforms,
                              GeometryProvenanceTag current_geometry) {
  if (!capture.complete || capture.candidates == nullptr ||
      capture.candidate_count != calibration::kLegServoSlotCount ||
      current_geometry == kNoGeometryProvenance) {
    return false;
  }
  for (uint8_t i = 0; i < capture.candidate_count; ++i) {
    const Q0BootstrapCandidate& candidate = capture.candidates[i];
    const JointTransform* held = transforms.find(candidate.evidence.identity, current_geometry);
    if (held == nullptr || !held->present ||
        held->state != calibration::EvidenceState::PROMOTED ||
        held->q0_tick != candidate.evidence.tick) {
      return false;
    }
  }
  return true;
}

Q0EvidencePreparation prepareCurrentQ0Evidence(
    const CalibrationGeometryProfile& current_profile,
    const GeometryProvenance& expected_current_geometry,
    bool explicit_current_installation_confirmation) {
  Q0EvidencePreparation out{};

  if (!explicit_current_installation_confirmation) {
    out.status =
        Q0EvidencePreparationStatus::REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED;
    return out;
  }

  const Q0EvidencePreparationStatus package_status =
      validateQ0EvidencePackageFacts(frozenQ0EvidencePackageFacts());
  if (package_status != Q0EvidencePreparationStatus::READY) {
    out.status = package_status;
    return out;
  }

  if (!current_profile.bound() ||
      !current_profile.provenanceMatches(expected_current_geometry) ||
      !sameProvenance(q0_evidence_data::kSourceGeometry,
                      expected_current_geometry)) {
    out.status = Q0EvidencePreparationStatus::REJECT_SOURCE_GEOMETRY;
    return out;
  }

  if (q0_evidence_data::kRecordCount != calibration::kLegServoSlotCount) {
    out.status = Q0EvidencePreparationStatus::REJECT_RECORD_COUNT;
    return out;
  }

  for (uint8_t i = 0; i < q0_evidence_data::kRecordCount; ++i) {
    for (uint8_t j = 0; j < i; ++j) {
      if (sameRecordIdentity(q0_evidence_data::kRecords[i],
                             q0_evidence_data::kRecords[j]) ||
          q0_evidence_data::kRecords[i].bus_id ==
              q0_evidence_data::kRecords[j].bus_id) {
        out.status = Q0EvidencePreparationStatus::REJECT_RECORD_DUPLICATE;
        out.transform_count = 0;
        out.failed_record_index = i;
        return out;
      }
    }

    const Q0BootstrapCandidate candidate =
        candidateFromFrozenRecord(q0_evidence_data::kRecords[i]);
    const Q0EvidencePreparationStatus step =
        acceptAndPromote(current_profile, expected_current_geometry, candidate,
                         q0_evidence_data::kCaptureSessionId,
                         &out.transforms[out.transform_count]);
    if (step != Q0EvidencePreparationStatus::READY) {
      out.status = step;
      out.transform_count = 0;
      out.failed_record_index = i;
      return out;
    }
    ++out.transform_count;
  }

  out.status = Q0EvidencePreparationStatus::READY;
  return out;
}

const char* toString(Q0EvidencePreparationStatus status) {
  switch (status) {
    case Q0EvidencePreparationStatus::NOT_EVALUATED: return "NOT_EVALUATED";
    case Q0EvidencePreparationStatus::READY: return "READY";
    case Q0EvidencePreparationStatus::REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED:
      return "REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED";
    case Q0EvidencePreparationStatus::REJECT_SOURCE_GEOMETRY:
      return "REJECT_SOURCE_GEOMETRY";
    case Q0EvidencePreparationStatus::REJECT_RECORD_COUNT:
      return "REJECT_RECORD_COUNT";
    case Q0EvidencePreparationStatus::REJECT_RECORD_DUPLICATE:
      return "REJECT_RECORD_DUPLICATE";
    case Q0EvidencePreparationStatus::REJECT_CANDIDATE:
      return "REJECT_CANDIDATE";
    case Q0EvidencePreparationStatus::REJECT_PROMOTION:
      return "REJECT_PROMOTION";
    case Q0EvidencePreparationStatus::REJECT_CAPTURE_POPULATION_NOT_PASS:
      return "REJECT_CAPTURE_POPULATION_NOT_PASS";
    case Q0EvidencePreparationStatus::REJECT_CAPTURE_Q0_POSE_NOT_CONFIRMED:
      return "REJECT_CAPTURE_Q0_POSE_NOT_CONFIRMED";
    case Q0EvidencePreparationStatus::REJECT_CAPTURE_TORQUE_NOT_OFF:
      return "REJECT_CAPTURE_TORQUE_NOT_OFF";
    case Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE:
      return "REJECT_FRESH_CAPTURE_NOT_COMPLETE";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
