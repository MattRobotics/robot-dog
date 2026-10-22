#ifndef MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H
#define MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H

#include <stdint.h>

#include "CalibrationGeometryProfile.h"
#include "../servo/ServoPopulation.h"

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
  REJECT_SEARCH_CORRIDOR = 8,  // the calibration search corridor is not self-consistent
};

// Calibration endpoint search corridor (2026-09-29 hardware finding, operator
// decision; semantics ported from the LF V25 hardware oracle, matdog.rs
// build_profile()/contact_acceptance_bounds()). LF_UPPER's real MIN stop was
// found ~23 ticks PAST the Geometry V5 contact, beyond the URDF limit, so an
// endpoint search must be able to continue past both - but only inside this
// one bounded corridor, and only for a CALIBRATION contact search:
//   guard = URDF limit + 64 ticks in the probe direction   (GUARD_OVERSHOOT_TICKS)
//   entry = URDF limit - 64 ticks back toward q0            (CONTACT_ACCEPTANCE_INNER_TICKS)
// Fine search and contact acceptance live in [entry, guard]; no search target
// may pass the guard. It changes NOTHING about the canonical contact, the URDF
// domain, stand/gait limits or JointLimits.
constexpr uint16_t kCalibrationSearchGuardOvershootTicks = 64;
constexpr uint16_t kCalibrationSearchAcceptanceInnerTicks = 64;

// All raw ticks, resolved for one joint, one side and the current promoted q0.
struct CalibrationSearchCorridor {
  int8_t probe_sign = 0;             // raw direction toward this side's endpoint (+1/-1)
  uint16_t home_tick = 0;            // promoted q0
  uint16_t contact_tick = 0;         // canonical Geometry V5 contact (reference only)
  uint16_t urdf_limit_tick = 0;      // this side's URDF limit
  uint16_t opposite_limit_tick = 0;  // the other side's URDF limit
  uint16_t entry_tick = 0;           // urdf_limit - probe_sign*64
  uint16_t guard_tick = 0;           // urdf_limit + probe_sign*64
  bool valid() const { return probe_sign == 1 || probe_sign == -1; }
};

// Signed distance of `tick` from q0 in the probe direction (positive = toward
// the endpoint). Every corridor comparison is made on this one axis, so no
// caller ever reasons about raw-direction signs itself.
int32_t searchDepth(const CalibrationSearchCorridor& corridor, uint16_t tick);
// True when a search target is inside [opposite URDF limit, guard].
bool searchCorridorAdmits(const CalibrationSearchCorridor& corridor, uint16_t tick);
// True when a position is inside the contact acceptance band [entry, guard].
bool searchCorridorAccepts(const CalibrationSearchCorridor& corridor, uint16_t tick);

TargetResolveStatus resolveUrdfQToRaw(const CalibrationGeometryProfile& profile,
                                      const GeometryProvenance& expected_provenance,
                                      const JointTransform& transform,
                                      MicroRad q_urad,
                                      uint16_t* raw_tick_out);

// Observation-only conversion, including a recorded folded contact just outside
// the URDF. Callers must enforce the recorded mechanical span. Command
// conversion remains resolveRadiansToRaw and still enforces the URDF.
TargetResolveStatus resolveRawObservation(const CalibrationGeometryProfile& profile,
  const GeometryProvenance& expected_provenance,const JointTransform& transform,
  uint16_t raw_tick,MicroRad* q_urad_out);

TargetResolveStatus resolveRawToUrdfQ(const CalibrationGeometryProfile& profile,
                                      const GeometryProvenance& expected_provenance,
                                      const JointTransform& transform,
                                      uint16_t raw_tick,
                                      MicroRad* q_urad_out);

// ---------------------------------------------------------------------------
// Core Motion V6: the one joint convention of every runtime consumer
// ---------------------------------------------------------------------------
//
// Semantic joint slot: leg*3 + {HIP, UPPER, LOWER}, legs LF, RF, RH, LH (the
// order of every Motion frame, feedback vector and telemetry array). The
// canonical allocation lists each leg LOWER, UPPER, HIP, so slot i is
// canonical leg servo (i/3)*3 + 2 - i%3. Motion, feedback, telemetry and
// maintenance take a slot's servo and JointIdentity from here only.
// Inline: only units that use a slot link the servo population table.
inline const servo::CanonicalServo* slotServo(unsigned slot) {
  if (slot >= servo::kLegServoCount) return nullptr;
  return servo::legServoAt(static_cast<uint8_t>((slot / 3) * 3 + 2 - slot % 3));
}
inline bool slotIdentity(unsigned slot, calibration::JointIdentity* out) {
  const servo::CanonicalServo* unit = slotServo(slot);
  if (out == nullptr || unit == nullptr) return false;
  *out = calibration::JointIdentity{};
  out->leg = static_cast<calibration::Leg>(slot / 3);
  out->joint = static_cast<calibration::JointKind>(slot % 3);
  calibration::setPhysicalUnit(out, unit->physical_unit);
  return out->valid() && out->unitKnown();
}

// Joint radians are DERIVED from the integer micro-radian domain above through
// ONE conversion, q / 1e6: the correctly rounded quotient. (q * 1e-6 rounds
// twice and can differ by one ULP for the same integer.) A RAW limit end and a
// RAW encoder sample on the same tick therefore give bit-identical radians,
// so an inclusive comparison of encoder-derived values never fails by rounding
// and never passes one tick beyond.
inline double microRadToRadians(MicroRad q) { return static_cast<double>(q) / 1e6; }
// Nearest micro-radian. false for a non-finite or out-of-domain angle (never a
// wrapped or saturated value); the URDF check of the resolver still applies.
bool radiansToMicroRad(double q_rad, MicroRad* q_urad_out);
// RAW tick -> joint radians: encoder samples and RAW limit ends.
TargetResolveStatus resolveRawToRadians(const CalibrationGeometryProfile& profile,
                                        const GeometryProvenance& expected_provenance,
                                        const JointTransform& transform,
                                        uint16_t raw_tick,
                                        double* q_rad_out);
// Joint radians -> RAW tick: every commanded target. q_urad_out is optional.
TargetResolveStatus resolveRadiansToRaw(const CalibrationGeometryProfile& profile,
                                        const GeometryProvenance& expected_provenance,
                                        const JointTransform& transform,
                                        double q_rad,
                                        uint16_t* raw_tick_out,
                                        MicroRad* q_urad_out = nullptr);

// DIRECTION_VERIFY is a raw excursion around current q0. Its SAFETY envelope
// is independent of direction, but the absolute ST3215 GoalPosition still
// needs a current promoted q0. No wrap is permitted.
TargetResolveStatus resolveDeltaFromQ0(const CalibrationGeometryProfile& profile,
                                       const GeometryProvenance& expected_provenance,
                                       const JointTransform& transform,
                                       int32_t delta_ticks,
                                       uint16_t* raw_tick_out);

// Resolves the search corridor for (joint of `transform`, `side`) from the
// bound Geometry V5 profile: the canonical contact and both URDF limits go
// through resolveUrdfQToRaw(); direction from jointDirection(). Refuses
// (REJECT_SEARCH_CORRIDOR) unless home < entry <= contact <= guard in depth
// order, the opposite limit lies behind home, and the guard is in 0..4095.
TargetResolveStatus resolveCalibrationSearchCorridor(const CalibrationGeometryProfile& profile,
                                                     const GeometryProvenance& expected_provenance,
                                                     const JointTransform& transform,
                                                     calibration::Leg endpoint_leg,
                                                     calibration::JointKind endpoint_joint,
                                                     calibration::ContactSide side,
                                                     CalibrationSearchCorridor* out);

const char* toString(TargetResolveStatus status);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_TARGET_RESOLVER_H
