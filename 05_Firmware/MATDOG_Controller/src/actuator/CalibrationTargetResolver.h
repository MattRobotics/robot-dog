#ifndef MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H
#define MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H

#include <stdint.h>

#include "CalibrationGeometryProfile.h"

namespace matdog {
namespace actuator {

// CR3-M2: the one checked q <-> raw conversion boundary.
//
// Absolute GoalPosition stays in the unsigned ST3215 domain 0..4095. There is
// deliberately no modulo/wrap target arithmetic. If a requested q would cross
// either raw boundary, resolution fails instead of wrapping to the other side.
enum class TargetResolveStatus : uint8_t {
  OK = 0,
  REJECT_NULL_OUTPUT = 1,
  REJECT_GEOMETRY = 2,
  REJECT_TRANSFORM = 3,
  REJECT_JOINT = 4,
  REJECT_DIRECTION = 5,
  REJECT_URDF_LIMIT = 6,
  REJECT_RAW_DOMAIN = 7,
};

TargetResolveStatus resolveUrdfQToRaw(const CalibrationGeometryProfile& profile,
                                      const GeometryProvenance& expected_provenance,
                                      const JointTransform& transform,
                                      MicroRad q_urad,
                                      uint16_t* raw_tick_out);

TargetResolveStatus resolveRawToUrdfQ(const CalibrationGeometryProfile& profile,
                                      const GeometryProvenance& expected_provenance,
                                      const JointTransform& transform,
                                      uint16_t raw_tick,
                                      MicroRad* q_urad_out);

// DIRECTION_VERIFY is a raw excursion around current q0. Its SAFETY envelope
// is independent of direction, but the absolute ST3215 GoalPosition still
// needs a current promoted q0. No wrap is permitted.
TargetResolveStatus resolveDeltaFromQ0(const CalibrationGeometryProfile& profile,
                                       const GeometryProvenance& expected_provenance,
                                       const JointTransform& transform,
                                       int32_t delta_ticks,
                                       uint16_t* raw_tick_out);

const char* toString(TargetResolveStatus status);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H
