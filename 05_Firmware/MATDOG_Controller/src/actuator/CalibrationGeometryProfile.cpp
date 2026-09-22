#include "CalibrationGeometryProfile.h"

namespace matdog {
namespace actuator {

namespace {

bool sameHash(const char* a, const char* b) {
  for (uint8_t i = 0; i < kSha256HexBytes; ++i) {
    if (a[i] != b[i]) return false;
    if (a[i] == '\0') return true;
  }
  return true;
}

}  // namespace

int64_t ticksToMicroRadMagnitude(int32_t ticks) {
  int64_t magnitude = ticks;
  if (magnitude < 0) magnitude = -magnitude;
  // Ceiling division. Combined with kMicroRadPerRevolution already being
  // rounded up, the result is never smaller than the true excursion - a bound
  // check can therefore not pass by rounding.
  return (magnitude * kMicroRadPerRevolution + (kTicksPerRevolution - 1)) / kTicksPerRevolution;
}

bool sameProvenance(const GeometryProvenance& a, const GeometryProvenance& b) {
  return sameHash(a.urdf_sha256, b.urdf_sha256) &&
         sameHash(a.mesh_manifest_sha256, b.mesh_manifest_sha256) &&
         sameHash(a.endpoint_semantic_sha256, b.endpoint_semantic_sha256) &&
         sameHash(a.parking_semantic_sha256, b.parking_semantic_sha256) &&
         sameHash(a.safety_policy_semantic_sha256, b.safety_policy_semantic_sha256) &&
         sameHash(a.allocation_sha256, b.allocation_sha256);
}

bool isExecutable(const GeometryEndpointRecord& endpoint) {
  // Both conditions, and neither implies the other. Sixteen endpoints have a
  // perfectly real geometric contact that lies beyond the URDF limit, and
  // eight of those even pass the clearance policy - which is exactly why
  // "clearance PASS" must not be allowed to mean "may be commanded".
  if (endpoint.domain != TargetDomain::EXECUTABLE_URDF_DOMAIN) return false;
  return endpoint.clearance == ClearancePolicyResult::PASS;
}

void CalibrationGeometryProfile::bind(const GeometryProvenance* provenance,
                                      const GeometryJointRecord* joints, uint8_t joint_count,
                                      const GeometryEndpointRecord* endpoints,
                                      uint8_t endpoint_count) {
  if (provenance == nullptr || joints == nullptr || endpoints == nullptr ||
      joint_count == 0 || endpoint_count == 0) {
    clear();
    return;
  }
  provenance_ = provenance;
  joints_ = joints;
  joint_count_ = joint_count;
  endpoints_ = endpoints;
  endpoint_count_ = endpoint_count;
}

void CalibrationGeometryProfile::clear() {
  provenance_ = nullptr;
  joints_ = nullptr;
  endpoints_ = nullptr;
  joint_count_ = 0;
  endpoint_count_ = 0;
}

bool CalibrationGeometryProfile::provenanceMatches(const GeometryProvenance& expected) const {
  if (!bound()) return false;
  return sameProvenance(*provenance_, expected);
}

const GeometryJointRecord* CalibrationGeometryProfile::findJoint(
    const calibration::JointIdentity& identity) const {
  if (!bound()) return nullptr;
  if (!identity.valid() || !identity.unitKnown()) return nullptr;
  for (uint8_t i = 0; i < joint_count_; ++i) {
    // BOTH axes. A bus id is an address, not an identity: after the
    // 2026-08-27 reassembly bus 11 is still "LF lower" but answers as unit
    // M33, and the unit the LF V25 archive calls M11 is now the neck.
    if (calibration::identityPermitsEvidenceReuse(joints_[i].identity, identity)) {
      return &joints_[i];
    }
  }
  return nullptr;
}

const GeometryEndpointRecord* CalibrationGeometryProfile::findEndpoint(
    calibration::Leg leg, calibration::JointKind joint, calibration::ContactSide side) const {
  if (!bound()) return nullptr;
  if (!calibration::isKnownLeg(leg) || !calibration::isKnownJointKind(joint) ||
      !calibration::isKnownContactSide(side)) {
    return nullptr;
  }
  for (uint8_t i = 0; i < endpoint_count_; ++i) {
    const GeometryEndpointRecord& record = endpoints_[i];
    if (record.leg == leg && record.joint == joint && record.side == side) return &record;
  }
  return nullptr;
}

bool CalibrationGeometryProfile::withinDirectionVerifyEnvelope(
    const calibration::JointIdentity& identity, int32_t delta_ticks) const {
  const GeometryJointRecord* joint = findJoint(identity);
  if (joint == nullptr) return false;
  // A zero excursion verifies nothing. Refusing it keeps "the move was
  // authorised" from ever meaning "no move happened".
  if (delta_ticks == 0) return false;
  if (joint->clear_half_span <= 0) return false;
  return ticksToMicroRadMagnitude(delta_ticks) <= static_cast<int64_t>(joint->clear_half_span);
}

bool JointTransform::usableProvenance() const {
  if (!present) return false;
  if (direction != 1 && direction != -1) return false;
  if (!identity.valid() || !identity.unitKnown()) return false;
  if (!calibration::mayPromote(origin)) return false;
  return calibration::isOperationalEvidence(state);
}

bool transformMayBeAppliedTo(const JointTransform& transform,
                             const calibration::JointIdentity& current) {
  if (!transform.usableProvenance()) return false;
  return calibration::identityPermitsEvidenceReuse(transform.identity, current);
}

const char* toString(TargetDomain domain) {
  switch (domain) {
    case TargetDomain::EXECUTABLE_URDF_DOMAIN: return "EXECUTABLE_URDF_DOMAIN";
    case TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS:
      return "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS";
  }
  return "UNKNOWN";
}

const char* toString(ParkingOutcome outcome) {
  switch (outcome) {
    case ParkingOutcome::NOT_NEEDED: return "NOT_NEEDED";
    case ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND: return "FEASIBLE_1DOF_PLAN_FOUND";
  }
  return "UNKNOWN";
}

const char* toString(ClearancePolicyResult result) {
  switch (result) {
    case ClearancePolicyResult::PASS: return "PASS";
    case ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD:
      return "UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD";
    case ClearancePolicyResult::FAIL: return "FAIL";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
