#ifndef MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_PLAN_H
#define MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_PLAN_H

#include <stdint.h>

#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationGeometryProfile.h"
#include "CalibrationDomain.h"
#include "FullLegCalibrationExecutor.h"

// The pure resolver that turns "calibrate leg <LF|RF|RH|LH>" into the one
// FullLegCalibrationRequest the executor runs. It is the ONLY place a leg name
// becomes a servo bus id, an endpoint record, an auxiliary decision or a
// contact/backoff/park number:
//
//   canonical allocation  ->  JointIdentity + bus id        (per leg, per joint)
//   Geometry V5 profile   ->  cross-check (identity AND bus id must agree)
//   Geometry V5 endpoints ->  MIN/MAX contact, clearance, auxiliary requirement
//   generic backoff rule  ->  re-approach point, geometrically verified
//
// Nothing here is a per-leg table. The parking matrix
// (LF -> LH_UPPER, RF -> RH_UPPER, RH -> none, LH -> none) is what the
// CURRENT compiled Geometry V5 data resolves to; a regenerated profile would
// resolve differently without touching this file, and scripts/tests pins the
// present matrix so that change is visible in review.
//
// Pure: no Arduino, no ServoBus, no time, no authority. Refusal is a status.

namespace matdog {
namespace calibration {

enum class FullLegPlanStatus : uint8_t {
  OK = 0,
  REJECT_NULL_OUTPUT = 1,
  REJECT_UNKNOWN_LEG = 2,
  REJECT_GEOMETRY_UNBOUND = 3,       // profile unbound or not the expected provenance
  REJECT_CANONICAL_IDENTITY = 4,     // allocation has no (or an ambiguous) row for the joint
  REJECT_GEOMETRY_JOINT = 5,         // Geometry V5 lacks the joint, or its bus id disagrees
  REJECT_ENDPOINT_MISSING = 6,
  REJECT_ENDPOINT_NOT_EXECUTABLE = 7,
  REJECT_MIN_NEEDS_AUXILIARY = 8,    // the executor parks an auxiliary for MAX only
  REJECT_MAX_PLAN_INCONSISTENT = 9,  // parking outcome and has_auxiliary disagree
  REJECT_AUXILIARY_IDENTITY = 10,
  REJECT_NO_TRANSFORM = 11,          // no current promoted q0 for a joint the plan needs
  REJECT_BACKOFF = 12,               // the generic backoff rule could not be verified
  REJECT_TARGET_RESOLUTION = 13,     // the checked resolver refused a plan target
};

// Same value the LF V25 precedent cites (CalibrationFailure::REPEATABILITY_EXCEEDED),
// reused for both sides of every leg - not re-derived per leg.
constexpr uint16_t kFullLegRepeatabilityToleranceTicks = 16;

// A re-approach that travels less than this many repeatability tolerances is
// not a re-approach: both passes would stall within noise of each other and
// the repeatability check would prove nothing.
constexpr uint16_t kFullLegMinReapproachToleranceMultiple = 8;

// Hardware finding 2026-09-29 (operator-approved): LF_UPPER's MIN stop sat 4-5
// ticks short of the Geometry V5 contact, inside the 4-tick arrival tolerance,
// so an approach commanded exactly to the contact could "arrive" on the stop
// and read as NO_CONTACT_DETECTED. Both approach passes of both sides are
// therefore commanded this many raw ticks past the canonical contact (which
// itself is unchanged and stays the evidence reference); the backoff is not.
// Equal to the reviewed repeatability tolerance; never above the policy's
// actuator::kContactProbeMaxOvertravelTicks.
constexpr uint16_t kFullLegApproachOvertravelTicks = 16;
static_assert(kFullLegApproachOvertravelTicks <= actuator::kContactProbeMaxOvertravelTicks,
              "the Full-Leg approach allowance may never exceed the policy's maximum");

struct FullLegJointRef {
  JointIdentity identity{};
  uint8_t bus_id = 0;  // 0 = unresolved (never a valid ST3215 id)
};

struct FullLegPlan {
  Leg leg = Leg::LF;
  FullLegJointRef upper{};
  FullLegJointRef hip{};
  FullLegJointRef lower{};
  // Fully populated; request.auxiliary_required / auxiliary_* come from the
  // Geometry V5 MAX endpoint record, never from the caller.
  FullLegCalibrationRequest request{};
};

// One (leg, joint) -> semantic identity + bus id, from the canonical allocation,
// cross-checked against the bound Geometry V5 profile (both the identity and the
// bus id must agree). Exactly one canonical row must match.
FullLegPlanStatus resolveLegJoint(const actuator::CalibrationGeometryProfile& profile, Leg leg,
                                  JointKind joint, FullLegJointRef* out);

// The generic backoff (re-approach) point for one endpoint: halfway between
// URDF q=0 and the contact angle, on the contact's side. VERIFIED, not assumed:
//   - same side as the contact, strictly between q=0 and the contact;
//   - no farther from q=0 than the compiled `clear` angle on that side (so it
//     lies inside the region the geometry compiler proved clear);
//   - both the contact and the backoff resolve through the checked
//     URDF-q -> raw resolver with this joint's q0 transform;
//   - the raw travel between them is at least kFullLegMinReapproachToleranceMultiple
//     repeatability tolerances.
FullLegPlanStatus deriveBackoffUrad(const actuator::CalibrationGeometryProfile& profile,
                                    const actuator::GeometryProvenance& expected_provenance,
                                    const actuator::JointTransform& transform,
                                    const actuator::GeometryEndpointRecord& endpoint,
                                    uint16_t repeatability_tolerance_ticks,
                                    actuator::MicroRad* backoff_urad_out);

FullLegPlanStatus resolveFullLegPlan(const actuator::CalibrationGeometryProfile& profile,
                                     const actuator::GeometryProvenance& expected_provenance,
                                     const actuator::JointTransformTable& transforms, Leg leg,
                                     FullLegPlan* out);

const char* toString(FullLegPlanStatus status);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_PLAN_H
