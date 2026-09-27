#ifndef MATDOG_ACTUATOR_CALIBRATION_Q0_EVIDENCE_PREPARATION_H
#define MATDOG_ACTUATOR_CALIBRATION_Q0_EVIDENCE_PREPARATION_H

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
};

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

// Rehydrates the frozen CR2-C CANDIDATE summary, then goes through the exact
// CR3 acceptance and promotion functions. It never directly manufactures a
// PROMOTED JointTransform.
//
// The explicit current-installation confirmation is deliberately required
// every time a caller wants to promote this persisted evidence after boot.
// It is the point at which an operator asserts that no servo/mechanical
// reassembly occurred after the 2026-09-27 capture. Future persistence may
// replace this with a stronger installation identity, but it must never vanish.
Q0EvidencePreparation prepareCurrentQ0Evidence(
    const CalibrationGeometryProfile& current_profile,
    const GeometryProvenance& expected_current_geometry,
    bool explicit_current_installation_confirmation);

const char* toString(Q0EvidencePreparationStatus status);

}  // namespace actuator
}  // namespace matdog

#endif
