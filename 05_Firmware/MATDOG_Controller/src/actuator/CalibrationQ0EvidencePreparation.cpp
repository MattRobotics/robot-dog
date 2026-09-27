#include "CalibrationQ0EvidencePreparation.h"

namespace matdog {
namespace actuator {

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

}  // namespace

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
        out.failed_record_index = i;
        return out;
      }
    }

    const Q0BootstrapCandidate candidate =
        candidateFromFrozenRecord(q0_evidence_data::kRecords[i]);
    const AcceptedQ0 accepted = acceptQ0Candidate(
        current_profile, expected_current_geometry, candidate);
    if (!accepted.accepted()) {
      out.status = Q0EvidencePreparationStatus::REJECT_CANDIDATE;
      out.failed_record_index = i;
      return out;
    }

    Q0PromotionRequest request{};
    request.explicit_currentness_confirmation = true;
    request.capture_session_id = q0_evidence_data::kCaptureSessionId;
    const PromotedQ0 promoted = promoteAcceptedQ0(
        current_profile, expected_current_geometry, accepted, request);
    if (!promoted.promoted()) {
      out.status = Q0EvidencePreparationStatus::REJECT_PROMOTION;
      out.failed_record_index = i;
      return out;
    }

    out.transforms[out.transform_count++] = promoted.transform;
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
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
