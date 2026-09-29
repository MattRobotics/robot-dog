#ifndef MATDOG_ACTUATOR_CALIBRATION_Q0_EVIDENCE_PREPARATION_H
#define MATDOG_ACTUATOR_CALIBRATION_Q0_EVIDENCE_PREPARATION_H

#include "ActuatorWritePolicy.h"
#include "CalibrationQ0EvidenceData.h"

namespace matdog {
namespace actuator {

enum class Q0EvidencePreparationStatus : uint8_t {
  NOT_EVALUATED = 0,
  READY = 1,
  REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED = 2,
  REJECT_SOURCE_GEOMETRY = 3,
  REJECT_RECORD_COUNT = 4,
  REJECT_RECORD_DUPLICATE = 5,
  REJECT_CANDIDATE = 6,
  REJECT_PROMOTION = 7,
  REJECT_CAPTURE_POPULATION_NOT_PASS = 8,
  REJECT_CAPTURE_Q0_POSE_NOT_CONFIRMED = 9,
  REJECT_CAPTURE_TORQUE_NOT_OFF = 10,
  REJECT_FRESH_CAPTURE_NOT_COMPLETE = 11,
};

struct Q0EvidencePackageFacts {
  bool formal_population_pass = false;
  bool nominal_zero_pose_confirmed = false;
  bool torque_off_verified_before_capture = false;
};

// Pure validation boundary for the persisted capture package. Tests can
// independently prove every mandatory capture-level prerequisite fails closed.
Q0EvidencePackageFacts frozenQ0EvidencePackageFacts();
Q0EvidencePreparationStatus validateQ0EvidencePackageFacts(
    const Q0EvidencePackageFacts& facts);

struct Q0EvidencePreparation {
  Q0EvidencePreparationStatus status = Q0EvidencePreparationStatus::NOT_EVALUATED;
  JointTransform transforms[calibration::kLegServoSlotCount]{};
  uint8_t transform_count = 0;
  uint8_t failed_record_index = 0xFF;

  bool ready() const {
    return status == Q0EvidencePreparationStatus::READY &&
           transform_count == calibration::kLegServoSlotCount;
  }
};

// PRODUCTION PATH. Promotes the twelve q0 candidates of the CURRENT-BOOT
// read-only capture (CalibrationQ0CaptureSession::freshCapture()) through the
// exact CR3 acceptance and promotion functions. The frozen CR2-C package is
// never consulted: a new maintenance capture replaces the previous q0 evidence
// because the same joint identities are re-admitted over the old transforms.
//
// All twelve candidates are validated before any transform is returned, so a
// single bad candidate refuses the whole set (transform_count stays 0). The
// caller admits the returned transforms only when ready().
//
// The explicit confirmation asserts that the operator aligned the legs to the
// nominal URDF q=0 pose for THIS capture and that nothing was reassembled since.
Q0EvidencePreparation prepareFreshQ0Evidence(
    const CalibrationGeometryProfile& current_profile,
    const GeometryProvenance& expected_current_geometry,
    const FreshQ0Capture& capture,
    bool explicit_current_installation_confirmation);

// True only when the transform table already holds, for every one of the
// capture's twelve joints under the CURRENT geometry, a PROMOTED transform
// whose q0 is exactly the capture's measured tick. A complete capture that was
// never promoted (or was superseded) is therefore not "current" for
// calibration. Stateless on purpose: nothing to reset, nothing to go stale.
bool freshQ0CaptureIsPromoted(const FreshQ0Capture& capture,
                              const JointTransformTable& transforms,
                              GeometryProvenanceTag current_geometry);

// HISTORICAL REFERENCE / REGRESSION ORACLE ONLY. Rehydrates the frozen CR2-C
// (2026-09-27) CANDIDATE summary and pushes it through the same CR3 functions.
// No production command calls it any more: promoting stale evidence over a
// fresh capture was the bug this module's fresh path replaces. It stays so the
// preserved CR2-C package keeps proving that the CR3 gates still accept the
// physically validated q0 values.
Q0EvidencePreparation prepareCurrentQ0Evidence(
    const CalibrationGeometryProfile& current_profile,
    const GeometryProvenance& expected_current_geometry,
    bool explicit_current_installation_confirmation);

const char* toString(Q0EvidencePreparationStatus status);

}  // namespace actuator
}  // namespace matdog

#endif
