#ifndef MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_PLAN_H
#define MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_PLAN_H

#include <stdint.h>

#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationGeometryProfile.h"
#include "../actuator/CalibrationSequencePlan.h"
#include "CalibrationDomain.h"
#include "FullLegCalibrationExecutor.h"

// The pure resolver that turns "calibrate leg <LF|RF|RH|LH>" into the one
// FullLegCalibrationRequest the 24-contact sequence executor runs: all SIX
// endpoints of the leg (UPPER, LOWER, HIP x MIN, MAX). It is the ONLY place a
// leg name becomes servo bus ids, search corridors, prerequisite poses, a park
// joint or q0 ticks:
//
//   canonical allocation    ->  JointIdentity + bus id, for all 12 leg joints
//   Geometry V5 profile     ->  cross-check (identity AND bus id); URDF domain;
//                               canonical MIN/MAX contacts (corridor reference)
//   promoted q0 transforms  ->  q0 ticks (current boot only)
//   CalibrationSequencePlan ->  the geometry-validated prerequisite poses and the
//                               rear park joint/pose of a front leg
//   search corridor         ->  actuator::resolveCalibrationSearchCorridor(), per
//                               joint and side, for this installation's q0
//
// Nothing here is a per-leg table: a regenerated profile or plan resolves
// differently without touching this file; scripts/tests pins the present
// matrix (24 profiles) so any change is visible in review.
//
// Pure: no Arduino, no ServoBus, no time, no authority. Refusal is a status.

namespace matdog {
namespace calibration {

enum class FullLegPlanStatus : uint8_t {
  OK = 0,
  REJECT_NULL_OUTPUT = 1,
  REJECT_UNKNOWN_LEG = 2,
  REJECT_GEOMETRY_UNBOUND = 3,       // profile unbound or not the expected provenance
  REJECT_CANONICAL_IDENTITY = 4,     // allocation has no (or an ambiguous) row for a joint
  REJECT_GEOMETRY_JOINT = 5,         // Geometry V5 lacks the joint, or its bus id disagrees
  REJECT_ENDPOINT_MISSING = 6,       // a MIN/MAX endpoint record is missing
  REJECT_NO_SEQUENCE_PLAN = 7,       // no plan, a plan for another model, or none for the leg
  REJECT_SEQUENCE_NOT_VALIDATED = 8, // the plan's geometry validation refused this leg
  REJECT_PARK_INCONSISTENT = 9,      // the plan's park joint is not V5's UPPER MAX auxiliary
  REJECT_NO_TRANSFORM = 10,          // no current promoted q0 for a joint the run touches
  REJECT_NO_DIRECTION = 11,
  REJECT_SEARCH_CORRIDOR = 12,       // a side's calibration search corridor is not coherent
  REJECT_TARGET_RESOLUTION = 13,     // the checked resolver refused a plan target
};

// Same value the LF V25 precedent uses (REPEATABILITY_TOLERANCE_TICKS), for
// every side of every leg - not re-derived per leg.
constexpr uint16_t kFullLegRepeatabilityToleranceTicks = 16;
// The LF V25 RAM TorqueLimit (matdog.rs TORQUE_LIMIT). static_audit.py pins
// it equal to ServoBus::kReviewedRamTorqueLimit, the one value the backend writes.
constexpr uint16_t kFullLegCalibrationTorqueLimit = 500;

struct FullLegJointRef {
  JointIdentity identity{};
  uint8_t bus_id = 0;  // 0 = unresolved (never a valid ST3215 id)
};

struct FullLegPlan {
  Leg leg = Leg::LF;
  FullLegJointRef upper{};
  FullLegJointRef hip{};
  FullLegJointRef lower{};
  FullLegJointRef park{};  // bus 0 when the leg parks nothing
  FullLegCalibrationRequest request{};
};

// One (leg, joint) -> semantic identity + bus id, from the canonical allocation,
// cross-checked against the bound Geometry V5 profile (both the identity and the
// bus id must agree). Exactly one canonical row must match.
FullLegPlanStatus resolveLegJoint(const actuator::CalibrationGeometryProfile& profile, Leg leg,
                                  JointKind joint, FullLegJointRef* out);

FullLegPlanStatus resolveFullLegPlan(const actuator::CalibrationGeometryProfile& profile,
                                     const actuator::GeometryProvenance& expected_provenance,
                                     const actuator::JointTransformTable& transforms,
                                     const actuator::CalibrationSequencePlan* sequence_plan,
                                     Leg leg, FullLegPlan* out);

const char* toString(FullLegPlanStatus status);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_PLAN_H
