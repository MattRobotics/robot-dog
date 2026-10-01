#include "FullLegCalibrationPlan.h"

#include "../servo/ServoPopulation.h"
#include "CalibrationPopulationEvidence.h"

namespace matdog {
namespace calibration {

namespace {

bool sameJoint(const JointIdentity& a, const JointIdentity& b) {
  return a.leg == b.leg && a.joint == b.joint;
}

// Resolves one joint the run touches: canonical identity/bus (cross-checked
// against Geometry V5) and its current promoted q0.
FullLegPlanStatus resolveRunJoint(const actuator::CalibrationGeometryProfile& profile,
                                  const actuator::JointTransformTable& transforms, Leg leg,
                                  JointKind kind, FullLegJointRef* ref, FullLegJoint* out) {
  const FullLegPlanStatus s = resolveLegJoint(profile, leg, kind, ref);
  if (s != FullLegPlanStatus::OK) return s;
  const actuator::JointTransform* t = transforms.find(ref->identity, profile.provenanceTag());
  if (t == nullptr || !t->present) return FullLegPlanStatus::REJECT_NO_TRANSFORM;
  out->identity = ref->identity;
  out->bus_id = ref->bus_id;
  out->q0_tick = t->q0_tick;
  return FullLegPlanStatus::OK;
}

FullLegPlanStatus resolveTick(const actuator::CalibrationGeometryProfile& profile,
                              const actuator::GeometryProvenance& provenance,
                              const actuator::JointTransformTable& transforms,
                              const JointIdentity& joint, actuator::MicroRad q, uint16_t* tick) {
  const actuator::JointTransform* t = transforms.find(joint, profile.provenanceTag());
  if (t == nullptr || !t->present) return FullLegPlanStatus::REJECT_NO_TRANSFORM;
  return actuator::resolveUrdfQToRaw(profile, provenance, *t, q, tick) ==
                 actuator::TargetResolveStatus::OK
             ? FullLegPlanStatus::OK
             : FullLegPlanStatus::REJECT_TARGET_RESOLUTION;
}

}  // namespace

FullLegPlanStatus resolveLegJoint(const actuator::CalibrationGeometryProfile& profile, Leg leg,
                                  JointKind joint, FullLegJointRef* out) {
  if (out == nullptr) return FullLegPlanStatus::REJECT_NULL_OUTPUT;
  *out = FullLegJointRef{};
  if (!isKnownLeg(leg) || !isKnownJointKind(joint)) return FullLegPlanStatus::REJECT_UNKNOWN_LEG;
  if (!profile.bound()) return FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND;

  // Exactly one installed canonical leg row may name this (leg, joint). Two
  // would make the bus ambiguous; none would mean the allocation lost a joint.
  uint8_t matches = 0;
  FullLegJointRef found{};
  for (uint8_t i = 0; i < servo::kLegServoCount; ++i) {
    const servo::CanonicalServo* canonical = servo::legServoAt(i);
    if (canonical == nullptr) break;
    JointIdentity identity{};
    if (!semanticIdentityFromCanonical(*canonical, &identity)) continue;
    if (identity.leg != leg || identity.joint != joint) continue;
    ++matches;
    found.identity = identity;
    found.bus_id = canonical->bus_id;
  }
  if (matches != 1 || found.bus_id == 0) return FullLegPlanStatus::REJECT_CANONICAL_IDENTITY;

  // Cross-check: Geometry V5 must know this exact joint (slot AND physical
  // unit, see findJoint) and must place it on the same bus.
  const actuator::GeometryJointRecord* record = profile.findJoint(found.identity);
  if (record == nullptr || record->bus_id != found.bus_id) {
    return FullLegPlanStatus::REJECT_GEOMETRY_JOINT;
  }

  *out = found;
  return FullLegPlanStatus::OK;
}

FullLegPlanStatus resolveFullLegPlan(const actuator::CalibrationGeometryProfile& profile,
                                     const actuator::GeometryProvenance& expected_provenance,
                                     const actuator::JointTransformTable& transforms,
                                     const actuator::CalibrationSequencePlan* sequence_plan,
                                     Leg leg, FullLegPlan* out) {
  if (out == nullptr) return FullLegPlanStatus::REJECT_NULL_OUTPUT;
  *out = FullLegPlan{};
  if (!isKnownLeg(leg)) return FullLegPlanStatus::REJECT_UNKNOWN_LEG;
  if (!profile.bound() || !profile.provenanceMatches(expected_provenance) ||
      profile.provenanceTag() == actuator::kNoGeometryProvenance) {
    return FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND;
  }
  if (sequence_plan == nullptr ||
      !actuator::sequencePlanMatchesGeometry(*sequence_plan, expected_provenance)) {
    return FullLegPlanStatus::REJECT_NO_SEQUENCE_PLAN;
  }
  const actuator::SequenceLegPlan* leg_plan = actuator::findSequenceLeg(*sequence_plan, leg);
  if (leg_plan == nullptr) return FullLegPlanStatus::REJECT_NO_SEQUENCE_PLAN;
  if (!leg_plan->geometry_validated) return FullLegPlanStatus::REJECT_SEQUENCE_NOT_VALIDATED;

  FullLegPlan plan{};
  plan.leg = leg;
  FullLegCalibrationRequest& r = plan.request;
  r.leg = leg;
  r.repeatability_tolerance_ticks = kFullLegRepeatabilityToleranceTicks;
  r.torque_limit = kFullLegCalibrationTorqueLimit;

  FullLegJointRef* refs[kJointKindCount] = {&plan.hip, &plan.upper, &plan.lower};
  for (uint8_t k = 0; k < kJointKindCount; ++k) {
    const JointKind kind = static_cast<JointKind>(k);
    FullLegPlanStatus s = resolveRunJoint(profile, transforms, leg, kind, refs[k], &r.joint[k]);
    if (s != FullLegPlanStatus::OK) return s;
    const actuator::GeometryJointRecord* record = profile.findJoint(r.joint[k].identity);
    const int8_t direction = actuator::jointDirection(profile, r.joint[k].identity);
    if (record == nullptr) return FullLegPlanStatus::REJECT_GEOMETRY_JOINT;
    if (direction != 1 && direction != -1) return FullLegPlanStatus::REJECT_NO_DIRECTION;
    r.direction[k] = direction;
    r.urdf_lower[k] = record->urdf_lower;
    r.urdf_upper[k] = record->urdf_upper;

    const actuator::JointTransform* t = transforms.find(r.joint[k].identity, profile.provenanceTag());
    for (uint8_t side = 0; side < kContactSideCount; ++side) {
      const ContactSide cs = static_cast<ContactSide>(side);
      if (profile.findEndpoint(leg, kind, cs) == nullptr) {
        return FullLegPlanStatus::REJECT_ENDPOINT_MISSING;
      }
      const actuator::TargetResolveStatus cr = actuator::resolveCalibrationSearchCorridor(
          profile, expected_provenance, *t, leg, kind, cs, &r.corridor[k][side]);
      if (cr == actuator::TargetResolveStatus::REJECT_SEARCH_CORRIDOR) {
        return FullLegPlanStatus::REJECT_SEARCH_CORRIDOR;
      }
      if (cr != actuator::TargetResolveStatus::OK) return FullLegPlanStatus::REJECT_TARGET_RESOLUTION;
    }
  }

  // The prerequisite poses, as URDF q (the command) and resolved ticks (what
  // a held joint's GoalPosition must read back).
  const JointIdentity& upper = r.joint[static_cast<uint8_t>(JointKind::UPPER)].identity;
  const JointIdentity& lower = r.joint[static_cast<uint8_t>(JointKind::LOWER)].identity;
  r.upper_for_lower_urad = leg_plan->upper_for_lower;
  r.upper_for_hip_min_urad = leg_plan->upper_for_hip_min;
  r.upper_for_hip_max_urad = leg_plan->upper_for_hip_max;
  r.lower_folded_urad = leg_plan->lower_folded;
  FullLegPlanStatus s = resolveTick(profile, expected_provenance, transforms, upper,
                                    r.upper_for_lower_urad, &r.upper_for_lower_tick);
  if (s == FullLegPlanStatus::OK) {
    s = resolveTick(profile, expected_provenance, transforms, upper, r.upper_for_hip_min_urad,
                    &r.upper_for_hip_min_tick);
  }
  if (s == FullLegPlanStatus::OK) {
    s = resolveTick(profile, expected_provenance, transforms, upper, r.upper_for_hip_max_urad,
                    &r.upper_for_hip_max_tick);
  }
  if (s == FullLegPlanStatus::OK) {
    s = resolveTick(profile, expected_provenance, transforms, lower, r.lower_folded_urad,
                    &r.lower_folded_tick);
  }
  if (s != FullLegPlanStatus::OK) return s;

  // A front leg parks its rear neighbour's UPPER for the whole sequence. The
  // plan's park joint and pose must be exactly the auxiliary and parked pose
  // Geometry V5 itself found for this leg's UPPER MAX path obstruction.
  const actuator::GeometryEndpointRecord* upper_max =
      profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MAX_SIDE);
  const bool v5_parks = upper_max != nullptr && upper_max->has_auxiliary;
  if (leg_plan->has_rear_park != v5_parks) return FullLegPlanStatus::REJECT_PARK_INCONSISTENT;
  if (leg_plan->has_rear_park) {
    if (leg_plan->park_leg != upper_max->auxiliary_leg ||
        leg_plan->park_joint != upper_max->auxiliary_joint ||
        leg_plan->park_target != upper_max->auxiliary_target || leg_plan->park_leg == leg) {
      return FullLegPlanStatus::REJECT_PARK_INCONSISTENT;
    }
    s = resolveRunJoint(profile, transforms, leg_plan->park_leg, leg_plan->park_joint, &plan.park,
                        &r.park);
    if (s != FullLegPlanStatus::OK) return s;
    r.has_rear_park = true;
    r.park_target_urad = leg_plan->park_target;
    s = resolveTick(profile, expected_provenance, transforms, r.park.identity, r.park_target_urad,
                    &r.park_target_tick);
    if (s != FullLegPlanStatus::OK) return s;
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      if (sameJoint(r.park.identity, r.joint[k].identity) || r.park.bus_id == r.joint[k].bus_id) {
        return FullLegPlanStatus::REJECT_PARK_INCONSISTENT;
      }
    }
  }

  // Every leg joint of the robot: INITIAL_RECOVERY and bystander monitoring.
  r.population_count = 0;
  for (uint8_t l = 0; l < kLegCount; ++l) {
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      FullLegJointRef ref{};
      s = resolveRunJoint(profile, transforms, static_cast<Leg>(l), static_cast<JointKind>(k), &ref,
                          &r.population[r.population_count]);
      if (s != FullLegPlanStatus::OK) return s;
      ++r.population_count;
    }
  }

  *out = plan;
  return FullLegPlanStatus::OK;
}

const char* toString(FullLegPlanStatus status) {
  switch (status) {
    case FullLegPlanStatus::OK: return "OK";
    case FullLegPlanStatus::REJECT_NULL_OUTPUT: return "REJECT_NULL_OUTPUT";
    case FullLegPlanStatus::REJECT_UNKNOWN_LEG: return "REJECT_UNKNOWN_LEG";
    case FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND: return "REJECT_GEOMETRY_UNBOUND";
    case FullLegPlanStatus::REJECT_CANONICAL_IDENTITY: return "REJECT_CANONICAL_IDENTITY";
    case FullLegPlanStatus::REJECT_GEOMETRY_JOINT: return "REJECT_GEOMETRY_JOINT";
    case FullLegPlanStatus::REJECT_ENDPOINT_MISSING: return "REJECT_ENDPOINT_MISSING";
    case FullLegPlanStatus::REJECT_NO_SEQUENCE_PLAN: return "REJECT_NO_SEQUENCE_PLAN";
    case FullLegPlanStatus::REJECT_SEQUENCE_NOT_VALIDATED: return "REJECT_SEQUENCE_NOT_VALIDATED";
    case FullLegPlanStatus::REJECT_PARK_INCONSISTENT: return "REJECT_PARK_INCONSISTENT";
    case FullLegPlanStatus::REJECT_NO_TRANSFORM: return "REJECT_NO_TRANSFORM";
    case FullLegPlanStatus::REJECT_NO_DIRECTION: return "REJECT_NO_DIRECTION";
    case FullLegPlanStatus::REJECT_SEARCH_CORRIDOR: return "REJECT_SEARCH_CORRIDOR";
    case FullLegPlanStatus::REJECT_TARGET_RESOLUTION: return "REJECT_TARGET_RESOLUTION";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
