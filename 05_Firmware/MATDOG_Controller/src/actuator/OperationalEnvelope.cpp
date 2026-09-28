#include "OperationalEnvelope.h"

#include "CalibrationTargetResolver.h"

namespace matdog {
namespace actuator {

namespace {

uint16_t minTick(uint16_t a, uint16_t b) { return a < b ? a : b; }
uint16_t maxTick(uint16_t a, uint16_t b) { return a > b ? a : b; }

}  // namespace

EnvelopeBuildStatus buildGeometryDerivedEnvelope(
    const CalibrationGeometryProfile& profile, const GeometryProvenance& expected_provenance,
    const JointTransform& transform, const GeometryDerivedEnvelopeRequest& request,
    OperationalEnvelope* out) {
  if (out != nullptr) *out = OperationalEnvelope{};
  if (out == nullptr) return EnvelopeBuildStatus::REJECT_NO_GEOMETRY;

  if (!profile.bound() || !profile.provenanceMatches(expected_provenance) ||
      profile.provenanceTag() == kNoGeometryProvenance) {
    return EnvelopeBuildStatus::REJECT_NO_GEOMETRY;
  }

  const GeometryJointRecord* joint = profile.findJoint(request.joint);
  if (joint == nullptr) return EnvelopeBuildStatus::REJECT_UNKNOWN_JOINT;

  if (!transform.usableProvenance() || !transformMayBeAppliedTo(transform, request.joint) ||
      transform.geometry != profile.provenanceTag()) {
    return EnvelopeBuildStatus::REJECT_NO_TRANSFORM;
  }

  if (request.required_min_urad > request.required_max_urad) {
    return EnvelopeBuildStatus::REJECT_WORKSPACE_MALFORMED;
  }
  // Strictly inside, never merely "not outside": this function verifies the
  // caller's own workspace claim rather than silently clipping it to the
  // URDF domain, so a caller that got the workspace wrong finds out here
  // instead of quietly receiving a narrower envelope than it asked for.
  if (request.required_min_urad < joint->urdf_lower ||
      request.required_max_urad > joint->urdf_upper) {
    return EnvelopeBuildStatus::REJECT_WORKSPACE_OUTSIDE_URDF;
  }
  if (request.safety_margin_urad < 0) return EnvelopeBuildStatus::REJECT_MARGIN_INVALID;

  const int64_t margined_min =
      static_cast<int64_t>(request.required_min_urad) + request.safety_margin_urad;
  const int64_t margined_max =
      static_cast<int64_t>(request.required_max_urad) - request.safety_margin_urad;
  if (margined_min > margined_max) return EnvelopeBuildStatus::REJECT_MARGIN_COLLAPSES_RANGE;

  uint16_t tick_a = 0;
  uint16_t tick_b = 0;
  const TargetResolveStatus status_a = resolveUrdfQToRaw(
      profile, expected_provenance, transform, static_cast<MicroRad>(margined_min), &tick_a);
  const TargetResolveStatus status_b = resolveUrdfQToRaw(
      profile, expected_provenance, transform, static_cast<MicroRad>(margined_max), &tick_b);
  if (status_a != TargetResolveStatus::OK || status_b != TargetResolveStatus::OK) {
    return EnvelopeBuildStatus::REJECT_TARGET_RESOLUTION;
  }

  // Direction (urdf_motor_direction) can be -1, so the numerically smaller
  // URDF angle does not always resolve to the numerically smaller raw tick -
  // take min/max of the two resolved ticks rather than assuming an order.
  out->identity = request.joint;
  out->source = EnvelopeSource::DERIVED_FROM_GEOMETRY;
  out->geometry = profile.provenanceTag();
  out->min_tick = minTick(tick_a, tick_b);
  out->max_tick = maxTick(tick_a, tick_b);
  out->present = true;
  return EnvelopeBuildStatus::READY;
}

namespace {

bool contactSideUsable(const calibration::ContactEvidence& evidence,
                       const calibration::JointIdentity& joint,
                       GeometryProvenanceTag side_geometry, GeometryProvenanceTag current_geometry,
                       EnvelopeBuildStatus* reject_status) {
  // Only the identity axis ContactProfileKey actually carries - see the file
  // comment on why physical-unit re-verification is not available here.
  if (evidence.key.leg != joint.leg || evidence.key.joint != joint.joint) {
    *reject_status = EnvelopeBuildStatus::REJECT_CONTACT_WRONG_SLOT;
    return false;
  }
  if (!evidence.has_measurement || !calibration::mayPromote(evidence.origin) ||
      !calibration::isOperationalEvidence(evidence.state)) {
    *reject_status = EnvelopeBuildStatus::REJECT_CONTACT_NOT_CURRENT;
    return false;
  }
  if (!calibration::isContactEvidence(evidence.detection)) {
    *reject_status = EnvelopeBuildStatus::REJECT_CONTACT_NOT_CONFIRMED;
    return false;
  }
  if (!evidence.witness.accepted()) {
    *reject_status = EnvelopeBuildStatus::REJECT_CONTACT_WITNESS_REJECTED;
    return false;
  }
  if (side_geometry == kNoGeometryProvenance || side_geometry != current_geometry) {
    *reject_status = EnvelopeBuildStatus::REJECT_CONTACT_GEOMETRY_MISMATCH;
    return false;
  }
  return true;
}

}  // namespace

EnvelopeBuildStatus buildContactDerivedEnvelope(
    const CalibrationGeometryProfile& profile, const GeometryProvenance& expected_provenance,
    const ContactDerivedEnvelopeRequest& request, OperationalEnvelope* out) {
  if (out != nullptr) *out = OperationalEnvelope{};
  if (out == nullptr) return EnvelopeBuildStatus::REJECT_NO_GEOMETRY;

  if (!profile.bound() || !profile.provenanceMatches(expected_provenance) ||
      profile.provenanceTag() == kNoGeometryProvenance) {
    return EnvelopeBuildStatus::REJECT_NO_GEOMETRY;
  }
  const GeometryProvenanceTag current_geometry = profile.provenanceTag();

  EnvelopeBuildStatus reject = EnvelopeBuildStatus::REJECT_MISSING_CONTACT_EVIDENCE;
  if (!request.min_side_evidence.has_measurement || !request.max_side_evidence.has_measurement) {
    return EnvelopeBuildStatus::REJECT_MISSING_CONTACT_EVIDENCE;
  }
  if (!contactSideUsable(request.min_side_evidence, request.joint, request.min_side_geometry,
                         current_geometry, &reject)) {
    return reject;
  }
  if (!contactSideUsable(request.max_side_evidence, request.joint, request.max_side_geometry,
                         current_geometry, &reject)) {
    return reject;
  }

  // fine_tick_1 is the SECOND, repeatability-confirming pass - the
  // authoritative confirmed position once the witness has accepted both
  // passes agree (see ContactProbeEngine's own file comment).
  const uint16_t lo = minTick(request.min_side_evidence.fine_tick_1,
                              request.max_side_evidence.fine_tick_1);
  const uint16_t hi = maxTick(request.min_side_evidence.fine_tick_1,
                              request.max_side_evidence.fine_tick_1);

  const int32_t margined_lo = static_cast<int32_t>(lo) + request.safety_margin_ticks;
  const int32_t margined_hi = static_cast<int32_t>(hi) - request.safety_margin_ticks;
  if (margined_lo > margined_hi) return EnvelopeBuildStatus::REJECT_CONTACT_ORDER_INVALID;
  if (margined_lo < 0 || margined_hi > 4095) {
    return EnvelopeBuildStatus::REJECT_CONTACT_ORDER_INVALID;
  }

  out->identity = request.joint;
  out->source = EnvelopeSource::MEASURED_CONTACT;
  out->geometry = current_geometry;
  out->min_tick = static_cast<uint16_t>(margined_lo);
  out->max_tick = static_cast<uint16_t>(margined_hi);
  out->present = true;
  return EnvelopeBuildStatus::READY;
}

const char* toString(EnvelopeSource source) {
  switch (source) {
    case EnvelopeSource::NONE:                  return "NONE";
    case EnvelopeSource::MEASURED_CONTACT:      return "MEASURED_CONTACT";
    case EnvelopeSource::DERIVED_FROM_GEOMETRY: return "DERIVED_FROM_GEOMETRY";
  }
  return "UNKNOWN";
}

const char* toString(EnvelopeBuildStatus status) {
  switch (status) {
    case EnvelopeBuildStatus::NOT_EVALUATED:                 return "NOT_EVALUATED";
    case EnvelopeBuildStatus::READY:                         return "READY";
    case EnvelopeBuildStatus::REJECT_NO_GEOMETRY:            return "REJECT_NO_GEOMETRY";
    case EnvelopeBuildStatus::REJECT_UNKNOWN_JOINT:          return "REJECT_UNKNOWN_JOINT";
    case EnvelopeBuildStatus::REJECT_NO_TRANSFORM:           return "REJECT_NO_TRANSFORM";
    case EnvelopeBuildStatus::REJECT_WORKSPACE_MALFORMED:    return "REJECT_WORKSPACE_MALFORMED";
    case EnvelopeBuildStatus::REJECT_WORKSPACE_OUTSIDE_URDF:
      return "REJECT_WORKSPACE_OUTSIDE_URDF";
    case EnvelopeBuildStatus::REJECT_MARGIN_INVALID:         return "REJECT_MARGIN_INVALID";
    case EnvelopeBuildStatus::REJECT_MARGIN_COLLAPSES_RANGE:
      return "REJECT_MARGIN_COLLAPSES_RANGE";
    case EnvelopeBuildStatus::REJECT_TARGET_RESOLUTION:      return "REJECT_TARGET_RESOLUTION";
    case EnvelopeBuildStatus::REJECT_MISSING_CONTACT_EVIDENCE:
      return "REJECT_MISSING_CONTACT_EVIDENCE";
    case EnvelopeBuildStatus::REJECT_CONTACT_WRONG_SLOT:     return "REJECT_CONTACT_WRONG_SLOT";
    case EnvelopeBuildStatus::REJECT_CONTACT_NOT_CURRENT:    return "REJECT_CONTACT_NOT_CURRENT";
    case EnvelopeBuildStatus::REJECT_CONTACT_NOT_CONFIRMED:
      return "REJECT_CONTACT_NOT_CONFIRMED";
    case EnvelopeBuildStatus::REJECT_CONTACT_WITNESS_REJECTED:
      return "REJECT_CONTACT_WITNESS_REJECTED";
    case EnvelopeBuildStatus::REJECT_CONTACT_GEOMETRY_MISMATCH:
      return "REJECT_CONTACT_GEOMETRY_MISMATCH";
    case EnvelopeBuildStatus::REJECT_CONTACT_ORDER_INVALID:
      return "REJECT_CONTACT_ORDER_INVALID";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
