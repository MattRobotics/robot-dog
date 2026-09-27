#include "CalibrationQ0Promotion.h"

namespace matdog {
namespace actuator {

namespace {

uint16_t circularDistanceFromRawCenter(uint16_t tick) {
  int32_t delta = static_cast<int32_t>(tick) -
                  static_cast<int32_t>(calibration::kServoRawCenter);
  if (delta < 0) delta = -delta;
  const int32_t wrapped = kTicksPerRevolution - delta;
  if (wrapped < delta) delta = wrapped;
  return static_cast<uint16_t>(delta);
}

bool candidateShapeIsCurrent(const Q0BootstrapCandidate& candidate) {
  const calibration::Q0Evidence& e = candidate.evidence;
  return candidate.status == Q0BootstrapStatus::CANDIDATE &&
         e.measured &&
         e.estimator == calibration::Q0Estimator::MANUAL_ZERO_POSE &&
         e.state == calibration::EvidenceState::CANDIDATE &&
         e.origin == calibration::CalibrationOrigin::LIVE_SESSION &&
         !e.accepted_by_gate;
}

}  // namespace

AcceptedQ0 acceptQ0Candidate(const CalibrationGeometryProfile& profile,
                             const GeometryProvenance& expected_provenance,
                             const Q0BootstrapCandidate& candidate) {
  AcceptedQ0 out{};

  if (!candidateShapeIsCurrent(candidate)) {
    out.status = Q0AcceptanceStatus::REJECT_NOT_CANDIDATE;
    return out;
  }

  if (!profile.bound() ||
      !profile.provenanceMatches(expected_provenance) ||
      candidate.geometry == kNoGeometryProvenance ||
      candidate.geometry != profile.provenanceTag()) {
    out.status = Q0AcceptanceStatus::REJECT_GEOMETRY;
    return out;
  }

  const calibration::Q0Evidence& e = candidate.evidence;
  if (!e.identity.valid() || !e.identity.unitKnown()) {
    out.status = Q0AcceptanceStatus::REJECT_IDENTITY;
    return out;
  }
  const GeometryJointRecord* joint = profile.findJoint(e.identity);
  if (joint == nullptr) {
    out.status = Q0AcceptanceStatus::REJECT_IDENTITY;
    return out;
  }
  if (joint->bus_id != candidate.bus_id) {
    out.status = Q0AcceptanceStatus::REJECT_BUS_BINDING;
    return out;
  }
  if (candidate.capture_session_id == 0) {
    out.status = Q0AcceptanceStatus::REJECT_SESSION;
    return out;
  }
  if (candidate.sample_count < kQ0AcceptanceMinSamples ||
      candidate.sample_count > kQ0BootstrapMaxSamples) {
    out.status = Q0AcceptanceStatus::REJECT_SAMPLE_POLICY;
    return out;
  }
  if (candidate.stability_spread_ticks > kQ0AcceptanceMaxSpreadTicks) {
    out.status = Q0AcceptanceStatus::REJECT_STABILITY;
    return out;
  }
  if (e.tick >= static_cast<uint16_t>(kTicksPerRevolution)) {
    out.status = Q0AcceptanceStatus::REJECT_RAW_DOMAIN;
    return out;
  }

  const uint16_t shift = circularDistanceFromRawCenter(e.tick);
  if (e.shift_from_digital_home_ticks != shift) {
    out.status = Q0AcceptanceStatus::REJECT_MALFORMED_DIAGNOSTIC;
    return out;
  }
  if (shift > kQ0PlausibilityTicks) {
    out.status = Q0AcceptanceStatus::REJECT_PLAUSIBILITY;
    return out;
  }

  out.status = Q0AcceptanceStatus::ACCEPTED;
  out.evidence = e;
  out.evidence.state = calibration::EvidenceState::ACCEPTED;
  out.evidence.accepted_by_gate = true;
  out.geometry = candidate.geometry;
  out.bus_id = candidate.bus_id;
  out.capture_session_id = candidate.capture_session_id;
  out.sample_count = candidate.sample_count;
  out.stability_spread_ticks = candidate.stability_spread_ticks;
  return out;
}

PromotedQ0 promoteAcceptedQ0(const CalibrationGeometryProfile& profile,
                             const GeometryProvenance& expected_provenance,
                             const AcceptedQ0& accepted,
                             const Q0PromotionRequest& request) {
  PromotedQ0 out{};

  if (!accepted.accepted() ||
      accepted.evidence.origin != calibration::CalibrationOrigin::LIVE_SESSION) {
    out.status = Q0PromotionStatus::REJECT_NOT_ACCEPTED;
    return out;
  }
  if (!profile.bound() ||
      !profile.provenanceMatches(expected_provenance) ||
      accepted.geometry == kNoGeometryProvenance ||
      accepted.geometry != profile.provenanceTag()) {
    out.status = Q0PromotionStatus::REJECT_GEOMETRY;
    return out;
  }
  if (!request.explicit_currentness_confirmation) {
    out.status = Q0PromotionStatus::REJECT_CURRENTNESS_NOT_CONFIRMED;
    return out;
  }
  if (request.capture_session_id == 0 ||
      request.capture_session_id != accepted.capture_session_id) {
    out.status = Q0PromotionStatus::REJECT_SESSION_MISMATCH;
    return out;
  }
  if (profile.findJoint(accepted.evidence.identity) == nullptr) {
    out.status = Q0PromotionStatus::REJECT_GEOMETRY;
    return out;
  }

  out.transform.identity = accepted.evidence.identity;
  out.transform.state = calibration::EvidenceState::PROMOTED;
  out.transform.origin = calibration::CalibrationOrigin::LIVE_SESSION;
  out.transform.geometry = accepted.geometry;
  out.transform.q0_tick = accepted.evidence.tick;
  out.transform.present = true;
  out.status = Q0PromotionStatus::PROMOTED;
  return out;
}

const char* toString(Q0AcceptanceStatus status) {
  switch (status) {
    case Q0AcceptanceStatus::NOT_EVALUATED: return "NOT_EVALUATED";
    case Q0AcceptanceStatus::ACCEPTED: return "ACCEPTED";
    case Q0AcceptanceStatus::REJECT_NOT_CANDIDATE: return "REJECT_NOT_CANDIDATE";
    case Q0AcceptanceStatus::REJECT_GEOMETRY: return "REJECT_GEOMETRY";
    case Q0AcceptanceStatus::REJECT_IDENTITY: return "REJECT_IDENTITY";
    case Q0AcceptanceStatus::REJECT_BUS_BINDING: return "REJECT_BUS_BINDING";
    case Q0AcceptanceStatus::REJECT_SESSION: return "REJECT_SESSION";
    case Q0AcceptanceStatus::REJECT_SAMPLE_POLICY: return "REJECT_SAMPLE_POLICY";
    case Q0AcceptanceStatus::REJECT_STABILITY: return "REJECT_STABILITY";
    case Q0AcceptanceStatus::REJECT_RAW_DOMAIN: return "REJECT_RAW_DOMAIN";
    case Q0AcceptanceStatus::REJECT_PLAUSIBILITY: return "REJECT_PLAUSIBILITY";
    case Q0AcceptanceStatus::REJECT_MALFORMED_DIAGNOSTIC:
      return "REJECT_MALFORMED_DIAGNOSTIC";
  }
  return "UNKNOWN";
}

const char* toString(Q0PromotionStatus status) {
  switch (status) {
    case Q0PromotionStatus::NOT_EVALUATED: return "NOT_EVALUATED";
    case Q0PromotionStatus::PROMOTED: return "PROMOTED";
    case Q0PromotionStatus::REJECT_NOT_ACCEPTED: return "REJECT_NOT_ACCEPTED";
    case Q0PromotionStatus::REJECT_GEOMETRY: return "REJECT_GEOMETRY";
    case Q0PromotionStatus::REJECT_CURRENTNESS_NOT_CONFIRMED:
      return "REJECT_CURRENTNESS_NOT_CONFIRMED";
    case Q0PromotionStatus::REJECT_SESSION_MISMATCH: return "REJECT_SESSION_MISMATCH";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
