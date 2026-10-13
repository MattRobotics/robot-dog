#include "CalibrationRuntimeRestore.h"

#include <string.h>

namespace matdog {
namespace calibration {
namespace {

bool runtimeSafe(const RuntimeRestoreRequest& request) {
  return request.executor_idle && request.torque_off_confirmed && request.motion_permit_revoked;
}

bool validSha(const char* text) {
  if (text == nullptr) return false;
  bool nonzero = false;
  for (uint8_t i = 0; i < 64; ++i) {
    const char c = text[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    nonzero |= c != '0';
  }
  return text[64] == '\0' && nonzero;
}

}  // namespace

void CalibrationRuntimeRestore::invalidate(actuator::JointTransformTable& transforms,
                                          actuator::ActuatorLimitTable& limits) {
  transforms.clear();
  limits.clear();
  snapshot_.restored = false;
  snapshot_.calibration_accepted = false;
  snapshot_.operational_limits_approved = false;
  snapshot_.restored_generation = 0;
  snapshot_.geometry = actuator::kNoGeometryProvenance;
  snapshot_.restored_joints = 0;
  snapshot_.approved_joints = 0;
  snapshot_.motion_permit = false;
}

RuntimeRestoreStatus CalibrationRuntimeRestore::restore(
    CalibrationPersistenceService& persistence,
    const actuator::CalibrationGeometryProfile& profile,
    const actuator::GeometryProvenance& expected_provenance,
    const RuntimeRestoreRequest& request, actuator::JointTransformTable& transforms,
    actuator::ActuatorLimitTable& limits) {
  ++snapshot_.restore_attempts;
  auto reject = [&](RuntimeRestoreStatus status) {
    snapshot_.restore_status = status;
    return status;
  };
  // A running executor must be stopped/revoked by its owner before mutation.
  if (!runtimeSafe(request)) return reject(RuntimeRestoreStatus::REJECT_UNSAFE_RUNTIME);
  invalidate(transforms, limits);
  snapshot_.failed_joint = kRecordJointCount;
  snapshot_.limits_status = OperationalAdmissionStatus::NOT_RUN;
  snapshot_.record_status = CalibrationRecordStatus::OK;
  if (!request.current_installation_confirmed) return reject(RuntimeRestoreStatus::REJECT_CURRENTNESS);
  if (!profile.bound() || !profile.provenanceMatches(expected_provenance) ||
      profile.jointCount() != kRecordJointCount) return reject(RuntimeRestoreStatus::REJECT_GEOMETRY);
  if (request.expected_generation == 0) return reject(RuntimeRestoreStatus::REJECT_GENERATION);

  CalibrationRecord record;
  const ServiceRecordReadResult read = persistence.readAcknowledgedRecord(profile, &record);
  snapshot_.persistence = persistence.snapshot().verdict;
  if (read.guard == ServiceGuard::NVS_NOT_READY) return reject(RuntimeRestoreStatus::REJECT_NVS_UNAVAILABLE);
  if (read.guard == ServiceGuard::WRITES_BLOCKED) return reject(RuntimeRestoreStatus::REJECT_UNCERTAIN_WRITE);
  if (read.load_status != LoadStatus::OK) return reject(RuntimeRestoreStatus::REJECT_LOAD);
  if (read.generation != request.expected_generation || record.generation != read.generation) {
    return reject(RuntimeRestoreStatus::REJECT_GENERATION);
  }
  snapshot_.record_status = validateCalibrationRecord(record, profile);
  if (snapshot_.record_status != CalibrationRecordStatus::OK) return reject(RuntimeRestoreStatus::REJECT_RECORD);

  actuator::JointTransformTable staged;
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const CalibrationRecordJointV1& joint = record.joint[i];
    actuator::JointTransform transform;
    transform.identity.leg = static_cast<Leg>(joint.leg);
    transform.identity.joint = static_cast<JointKind>(joint.joint);
    setPhysicalUnit(&transform.identity, joint.unit);
    // Preserve the recorded provenance; RESTORE does not masquerade as a fresh
    // capture. validateCalibrationRecord already recomputed its acceptance.
    transform.state = static_cast<EvidenceState>(joint.q0_state);
    transform.origin = static_cast<CalibrationOrigin>(joint.q0_origin);
    transform.geometry = record.geometry_tag;
    transform.q0_tick = joint.q0_tick;
    transform.present = true;
    if (!staged.admit(transform)) {
      snapshot_.failed_joint = i;
      return reject(RuntimeRestoreStatus::REJECT_ADMISSION);
    }
    identity_[i] = transform.identity;
    q0_[i] = transform.q0_tick;
    const uint16_t a = joint.contact[0].fine_tick_2;
    const uint16_t b = joint.contact[1].fine_tick_2;
    contact_min_[i] = a < b ? a : b;
    contact_max_[i] = a > b ? a : b;
    const uint16_t repeat_a = joint.contact[0].repeatability_ticks;
    const uint16_t repeat_b = joint.contact[1].repeatability_ticks;
    contact_uncertainty_[i] = repeat_a > repeat_b ? repeat_a : repeat_b;
  }
  if (staged.size() != kRecordJointCount) return reject(RuntimeRestoreStatus::REJECT_ADMISSION);
  transforms = staged;  // one Controller-thread publication, never partial
  snapshot_.restored = true;
  snapshot_.calibration_accepted = record.calibration_accepted == 1;
  snapshot_.restored_generation = record.generation;
  snapshot_.geometry = record.geometry_tag;
  snapshot_.restored_joints = staged.size();
  snapshot_.restore_status = RuntimeRestoreStatus::RESTORED;
  return snapshot_.restore_status;
}

OperationalAdmissionStatus CalibrationRuntimeRestore::qualifyOperationalLimits(
    const actuator::CalibrationGeometryProfile& profile,
    const actuator::GeometryProvenance& expected_provenance,
    const OperationalWorkspaceEvidence* evidence, const char* pinned_artifact_sha256,
    const RuntimeRestoreRequest& facts, const actuator::JointTransformTable& transforms,
    actuator::ActuatorLimitTable& limits,const char* pinned_route_sha256) {
  auto reject = [&](OperationalAdmissionStatus status) {
    snapshot_.limits_status = status;
    return status;
  };
  if (!runtimeSafe(facts)) return reject(OperationalAdmissionStatus::REJECT_UNSAFE_RUNTIME);
  limits.clear();
  snapshot_.operational_limits_approved = false;
  snapshot_.approved_joints = 0;
  snapshot_.failed_joint = kRecordJointCount;
  snapshot_.envelope_status = actuator::EnvelopeBuildStatus::NOT_EVALUATED;
  if (!snapshot_.restored || !snapshot_.calibration_accepted || transforms.size() != kRecordJointCount) {
    return reject(OperationalAdmissionStatus::REJECT_NOT_RESTORED);
  }
  if (!facts.current_installation_confirmed) return reject(OperationalAdmissionStatus::REJECT_NOT_RESTORED);
  if (evidence == nullptr) return reject(OperationalAdmissionStatus::REJECT_MISSING_EVIDENCE);
  if (evidence->schema != 1 || !validSha(evidence->artifact_sha256) ||
      !validSha(pinned_artifact_sha256) ||
      strcmp(evidence->artifact_sha256, pinned_artifact_sha256) != 0) {
    return reject(OperationalAdmissionStatus::REJECT_ARTIFACT_IDENTITY);
  }
  if (!profile.bound() || !profile.provenanceMatches(expected_provenance) ||
      !profile.provenanceMatches(evidence->provenance) || profile.provenanceTag() != snapshot_.geometry) {
    return reject(OperationalAdmissionStatus::REJECT_GEOMETRY);
  }
  if (evidence->calibration_generation != snapshot_.restored_generation ||
      facts.expected_generation != snapshot_.restored_generation) return reject(OperationalAdmissionStatus::REJECT_GENERATION);
  if(evidence->scope==OperationalLimitScope::CERTIFIED_ROUTE){
    if(!pinned_route_sha256||!validSha(pinned_route_sha256)||!validSha(evidence->route_sha256)||
       strcmp(pinned_route_sha256,evidence->route_sha256)||!validSha(evidence->corridor_proof_sha256))
      return reject(OperationalAdmissionStatus::REJECT_ARTIFACT_IDENTITY);
  }else if(evidence->scope!=OperationalLimitScope::FULL_JOINT_BOX)return reject(OperationalAdmissionStatus::REJECT_WORKSPACE_EVIDENCE);
  if (!evidence->workspace_reviewed) return reject(OperationalAdmissionStatus::REJECT_WORKSPACE_EVIDENCE);
  if (!evidence->simultaneous_configuration_checked) return reject(OperationalAdmissionStatus::REJECT_CONFIGURATION_EVIDENCE);
  if (!evidence->collision_clearance_checked) return reject(OperationalAdmissionStatus::REJECT_COLLISION_EVIDENCE);
  if (!evidence->margins_reviewed || evidence->contact_inset_ticks == 0) return reject(OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);

  actuator::ActuatorLimitTable staged;
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    snapshot_.failed_joint = i;
    const auto& workspace = evidence->joint[i];
    if (!identityPermitsEvidenceReuse(identity_[i], workspace.joint)) {
      return reject(OperationalAdmissionStatus::REJECT_JOINT_IDENTITY);
    }
    if (workspace.safety_margin_urad <= 0 ||
        evidence->contact_inset_ticks <= contact_uncertainty_[i]) {
      return reject(OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);
    }
    const actuator::JointTransform* transform = transforms.find(identity_[i], snapshot_.geometry);
    if (transform == nullptr || transform->q0_tick != q0_[i]) return reject(OperationalAdmissionStatus::REJECT_TRANSFORM);
    actuator::OperationalEnvelope envelope;
    snapshot_.envelope_status = actuator::buildGeometryDerivedEnvelope(
        profile, expected_provenance, *transform, workspace, &envelope);
    if (snapshot_.envelope_status != actuator::EnvelopeBuildStatus::READY) {
      return reject(OperationalAdmissionStatus::REJECT_ENVELOPE);
    }
    // Checked q->raw uses nearest-tick rounding. Inset one raw count on each
    // side so quantization cannot expand the approved micro-radian interval.
    // This is a conservative integer conversion, not an invented safety margin.
    if (static_cast<int32_t>(envelope.max_tick) - envelope.min_tick <= 2) {
      return reject(OperationalAdmissionStatus::REJECT_ENVELOPE);
    }
    ++envelope.min_tick;
    --envelope.max_tick;
    const int32_t lower = static_cast<int32_t>(contact_min_[i]) + evidence->contact_inset_ticks;
    const int32_t upper = static_cast<int32_t>(contact_max_[i]) - evidence->contact_inset_ticks;
    // Never silently clip the reviewed workspace: that can omit the targets
    // it was meant to cover. Require the whole derived envelope to fit inside
    // the confirmed physical contacts with an explicitly reviewed inset.
    if (lower > upper || envelope.min_tick < lower || envelope.max_tick > upper) {
      return reject(OperationalAdmissionStatus::REJECT_CONTACT_CLEARANCE);
    }
    actuator::JointLimit limit;
    limit.identity = identity_[i];
    limit.state = EvidenceState::PROMOTED;
    limit.origin = CalibrationOrigin::LIVE_SESSION;
    limit.geometry = snapshot_.geometry;
    limit.min_tick = envelope.min_tick;
    limit.max_tick = envelope.max_tick;
    limit.present = true;
    if (!staged.admit(limit)) return reject(OperationalAdmissionStatus::REJECT_ADMISSION);
  }
  if (staged.size() != kRecordJointCount) return reject(OperationalAdmissionStatus::REJECT_ADMISSION);
  limits = staged;
  snapshot_.failed_joint = kRecordJointCount;
  snapshot_.approved_joints = staged.size();
  snapshot_.operational_limits_approved = true;
  snapshot_.limits_status = OperationalAdmissionStatus::APPROVED;
  return snapshot_.limits_status;
}

const char* toString(RuntimeRestoreStatus status) {
  switch (status) {
#define CASE(s) case RuntimeRestoreStatus::s: return #s
    CASE(NOT_RUN); CASE(RESTORED); CASE(REJECT_UNSAFE_RUNTIME); CASE(REJECT_CURRENTNESS);
    CASE(REJECT_GEOMETRY); CASE(REJECT_NVS_UNAVAILABLE); CASE(REJECT_UNCERTAIN_WRITE);
    CASE(REJECT_LOAD); CASE(REJECT_GENERATION); CASE(REJECT_RECORD); CASE(REJECT_ADMISSION);
#undef CASE
  }
  return "UNKNOWN";
}

const char* toString(OperationalAdmissionStatus status) {
  switch (status) {
#define CASE(s) case OperationalAdmissionStatus::s: return #s
    CASE(NOT_RUN); CASE(APPROVED); CASE(REJECT_UNSAFE_RUNTIME); CASE(REJECT_NOT_RESTORED);
    CASE(REJECT_MISSING_EVIDENCE); CASE(REJECT_ARTIFACT_IDENTITY); CASE(REJECT_GEOMETRY);
    CASE(REJECT_GENERATION); CASE(REJECT_WORKSPACE_EVIDENCE); CASE(REJECT_CONFIGURATION_EVIDENCE);
    CASE(REJECT_COLLISION_EVIDENCE); CASE(REJECT_MARGIN_EVIDENCE); CASE(REJECT_JOINT_IDENTITY);
    CASE(REJECT_TRANSFORM); CASE(REJECT_ENVELOPE); CASE(REJECT_CONTACT_CLEARANCE); CASE(REJECT_ADMISSION);
#undef CASE
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
