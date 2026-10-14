#ifndef MATDOG_CALIBRATION_RUNTIME_RESTORE_H
#define MATDOG_CALIBRATION_RUNTIME_RESTORE_H

#include "CalibrationPersistenceService.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/OperationalEnvelope.h"

namespace matdog {
namespace calibration {

// Pure RAM service: no servo, NVS writes, authority acquisition or permit issue.
// The Controller supplies facts from its own executor/safety state, never HTTP
// parameters. LOAD, RESTORE, calibration acceptance, limit approval and motion
// permission are deliberately separate events.
struct RuntimeRestoreRequest {
  uint32_t expected_generation = 0;
  // Confirmation must describe the physical units/mounting, not merely a scan
  // of bus addresses. ST3215 IDs alone cannot identify a swapped physical unit.
  bool current_installation_confirmed = false;
  bool executor_idle = false;
  bool torque_off_confirmed = false;
  bool motion_permit_revoked = false;
};

enum class RuntimeRestoreStatus : uint8_t {
  NOT_RUN = 0,
  RESTORED,
  REJECT_UNSAFE_RUNTIME,
  REJECT_CURRENTNESS,
  REJECT_GEOMETRY,
  REJECT_NVS_UNAVAILABLE,
  REJECT_UNCERTAIN_WRITE,
  REJECT_LOAD,
  REJECT_GENERATION,
  REJECT_RECORD,
  REJECT_ADMISSION,
};
const char* toString(RuntimeRestoreStatus status);

// A reviewed OFFLINE artifact, compiled into the Controller, not a request
// accepted from a host. The SHA is pinned separately by the firmware caller.
// Collision/configuration evidence must cover the simultaneous 12-joint box,
// not merely each single-joint path with its neighbours at q=0. If it only
// covers corridors, it cannot approve this box: use per-segment qualification.
// No production artifact is fabricated by this implementation.
enum class OperationalLimitScope : uint8_t { FULL_JOINT_BOX, CERTIFIED_ROUTE };
struct OperationalWorkspaceEvidence {
  OperationalLimitScope scope=OperationalLimitScope::FULL_JOINT_BOX;
  char route_sha256[65]{};
  char corridor_proof_sha256[65]{};
  uint16_t schema = 1;
  char artifact_sha256[actuator::kSha256HexBytes] = {};
  actuator::GeometryProvenance provenance{};
  uint32_t calibration_generation = 0;
  bool workspace_reviewed = false;
  bool simultaneous_configuration_checked = false;
  bool collision_clearance_checked = false;
  bool margins_reviewed = false;
  uint16_t contact_inset_ticks = 0;
  // Canonical slot order LF,RF,RH,LH x HIP,UPPER,LOWER, deliberately different
  // from the generated profile's alphabetical URDF table order.
  actuator::GeometryDerivedEnvelopeRequest joint[kRecordJointCount];
};

enum class OperationalAdmissionStatus : uint8_t {
  NOT_RUN = 0,
  APPROVED,
  REJECT_UNSAFE_RUNTIME,
  REJECT_NOT_RESTORED,
  REJECT_MISSING_EVIDENCE,
  REJECT_ARTIFACT_IDENTITY,
  REJECT_GEOMETRY,
  REJECT_GENERATION,
  REJECT_WORKSPACE_EVIDENCE,
  REJECT_CONFIGURATION_EVIDENCE,
  REJECT_COLLISION_EVIDENCE,
  REJECT_MARGIN_EVIDENCE,
  REJECT_JOINT_IDENTITY,
  REJECT_TRANSFORM,
  REJECT_ENVELOPE,
  REJECT_CONTACT_CLEARANCE,
  REJECT_ADMISSION,
};
const char* toString(OperationalAdmissionStatus status);

struct RuntimeCalibrationSnapshot {
  uint32_t restore_attempts = 0;
  RuntimeRestoreStatus restore_status = RuntimeRestoreStatus::NOT_RUN;
  PersistenceVerdict persistence = PersistenceVerdict::NOT_RUN;
  CalibrationRecordStatus record_status = CalibrationRecordStatus::OK;
  OperationalAdmissionStatus limits_status = OperationalAdmissionStatus::NOT_RUN;
  actuator::EnvelopeBuildStatus envelope_status = actuator::EnvelopeBuildStatus::NOT_EVALUATED;
  uint8_t failed_joint = kRecordJointCount;
  uint8_t restored_joints = 0;
  uint8_t approved_joints = 0;
  uint32_t restored_generation = 0;
  actuator::GeometryProvenanceTag geometry = actuator::kNoGeometryProvenance;
  bool restored = false;
  bool calibration_accepted = false;  // revalidated 24/24 evidence only
  bool operational_limits_approved = false;
  // Fixture-commissioning scope only: the limit table holds the gen-1 contact
  // box with a reviewed inset (intersected with the URDF domain). It is NOT a
  // reviewed route/workspace approval and never sets approved_joints or
  // operational_limits_approved; collision is checked per trajectory.
  bool commissioning_limits_active = false;
  uint8_t commissioning_joints = 0;
  uint16_t commissioning_inset_ticks = 0;
  // This service never issues a permit, including after all limits qualify.
  bool motion_permit = false;
};

// Bounds of the runtime-selectable commissioning inset. The lower bound is
// still checked per joint against the recorded contact repeatability.
constexpr uint16_t kCommissioningMinInsetTicks = 2;
constexpr uint16_t kCommissioningMaxInsetTicks = 64;

class CalibrationRuntimeRestore {
 public:
  RuntimeRestoreStatus restore(
      CalibrationPersistenceService& persistence,
      const actuator::CalibrationGeometryProfile& profile,
      const actuator::GeometryProvenance& expected_provenance,
      const RuntimeRestoreRequest& request,
      actuator::JointTransformTable& transforms, actuator::ActuatorLimitTable& limits);

  // All 12 limits are staged and published together. On an evidence rejection
  // in a safe idle context, all old limits are revoked. Contact endpoints only
  // constrain the reviewed workspace; they never supply operational approval.
  OperationalAdmissionStatus qualifyOperationalLimits(
      const actuator::CalibrationGeometryProfile& profile,
      const actuator::GeometryProvenance& expected_provenance,
      const OperationalWorkspaceEvidence* evidence, const char* pinned_artifact_sha256,
      const RuntimeRestoreRequest& runtime_facts,
      const actuator::JointTransformTable& transforms, actuator::ActuatorLimitTable& limits,
      const char* pinned_route_sha256=nullptr);

  // Fixture commissioning: twelve hard limits = [contact_min + inset,
  // contact_max - inset] of the RESTORED acknowledged record, intersected with
  // the URDF interval, then the same one-tick quantization inset. Requires a
  // restored, accepted record, a confirmed installation and a safe idle
  // runtime; the inset must exceed every joint's recorded contact
  // repeatability. All twelve are staged and published together; on any
  // rejection every old limit stays revoked.
  OperationalAdmissionStatus qualifyCommissioningLimits(
      const actuator::CalibrationGeometryProfile& profile,
      const actuator::GeometryProvenance& expected_provenance, uint16_t contact_inset_ticks,
      const RuntimeRestoreRequest& runtime_facts,
      const actuator::JointTransformTable& transforms, actuator::ActuatorLimitTable& limits);

  void invalidate(actuator::JointTransformTable& transforms, actuator::ActuatorLimitTable& limits);
  const RuntimeCalibrationSnapshot& snapshot() const { return snapshot_; }

 private:
  RuntimeCalibrationSnapshot snapshot_{};
  JointIdentity identity_[kRecordJointCount]{};
  uint16_t q0_[kRecordJointCount]{};
  uint16_t contact_min_[kRecordJointCount]{};
  uint16_t contact_max_[kRecordJointCount]{};
  uint16_t contact_uncertainty_[kRecordJointCount]{};
};

}  // namespace calibration
}  // namespace matdog
#endif
