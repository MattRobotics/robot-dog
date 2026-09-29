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
  REJECT_OVERTRAVEL = 8,  // contact-probe allowance above kContactProbeMaxOvertravelTicks
};

// Hardware finding 2026-09-29, operator-approved: the most a CONTACT_PROBE
// approach may be commanded PAST the canonical Geometry V5 contact, in raw
// ticks, so the stall detector can observe a physical stop lying at (or a few
// ticks short of) the modelled contact instead of "arriving" on it. It is an
// allowance on the probe's commanded boundary only: the canonical contact, the
// backoff, every other operation and every envelope/limit are untouched.
constexpr uint16_t kContactProbeMaxOvertravelTicks = 16;

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

// The ONE sanctioned way to command a GoalPosition past a Geometry V5
// contact. Resolves `contact_urad` exactly like resolveUrdfQToRaw() (so the
// contact itself must lie inside the URDF domain), then continues
// `overtravel_ticks` further in `side`'s approach direction - q decreasing for
// MIN_SIDE, increasing for MAX_SIDE - in the raw domain. overtravel_ticks
// above kContactProbeMaxOvertravelTicks is REJECT_OVERTRAVEL; the result must
// stay inside 0..4095 (no wrap). overtravel_ticks == 0 is exactly
// resolveUrdfQToRaw(contact_urad).
TargetResolveStatus resolveContactProbeApproachToRaw(const CalibrationGeometryProfile& profile,
                                                     const GeometryProvenance& expected_provenance,
                                                     const JointTransform& transform,
                                                     MicroRad contact_urad,
                                                     calibration::ContactSide side,
                                                     uint16_t overtravel_ticks,
                                                     uint16_t* raw_tick_out);

const char* toString(TargetResolveStatus status);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H
