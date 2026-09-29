#include "FullLegCalibrationPlan.h"

#include <initializer_list>

#include "../servo/ServoPopulation.h"
#include "CalibrationPopulationEvidence.h"

namespace matdog {
namespace calibration {

namespace {

// Half of a signed micro-radian value, rounded toward zero. The result is
// never farther from q=0 than the input, so the rule cannot overshoot the
// contact by rounding.
actuator::MicroRad halfTowardZero(actuator::MicroRad value) { return value / 2; }

int64_t absTicksBetween(uint16_t a, uint16_t b) {
  const int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
  return d < 0 ? -d : d;
}

bool sameJoint(const JointIdentity& a, const JointIdentity& b) {
  return a.leg == b.leg && a.joint == b.joint;
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

FullLegPlanStatus deriveBackoffUrad(const actuator::CalibrationGeometryProfile& profile,
                                    const actuator::GeometryProvenance& expected_provenance,
                                    const actuator::JointTransform& transform,
                                    const actuator::GeometryEndpointRecord& endpoint,
                                    uint16_t repeatability_tolerance_ticks,
                                    actuator::MicroRad* backoff_urad_out) {
  if (backoff_urad_out == nullptr) return FullLegPlanStatus::REJECT_NULL_OUTPUT;
  *backoff_urad_out = 0;

  const actuator::MicroRad contact = endpoint.contact;
  const actuator::MicroRad clear = endpoint.clear;
  const actuator::MicroRad backoff = halfTowardZero(contact);

  // Same side as the contact and strictly between q=0 and it.
  const bool positive = endpoint.side == ContactSide::MAX_SIDE;
  if (positive) {
    if (!(contact > 0 && backoff > 0 && backoff < contact)) return FullLegPlanStatus::REJECT_BACKOFF;
    // Inside the region the geometry compiler proved clear on this side.
    if (!(clear > 0 && backoff <= clear)) return FullLegPlanStatus::REJECT_BACKOFF;
  } else {
    if (!(contact < 0 && backoff < 0 && backoff > contact)) return FullLegPlanStatus::REJECT_BACKOFF;
    if (!(clear < 0 && backoff >= clear)) return FullLegPlanStatus::REJECT_BACKOFF;
  }

  // Both points must survive the one checked URDF-q -> raw conversion.
  uint16_t contact_raw = 0;
  uint16_t backoff_raw = 0;
  if (actuator::resolveUrdfQToRaw(profile, expected_provenance, transform, contact,
                                  &contact_raw) != actuator::TargetResolveStatus::OK ||
      actuator::resolveUrdfQToRaw(profile, expected_provenance, transform, backoff,
                                  &backoff_raw) != actuator::TargetResolveStatus::OK) {
    return FullLegPlanStatus::REJECT_TARGET_RESOLUTION;
  }

  // A re-approach shorter than a few repeatability tolerances proves nothing.
  const int64_t minimum_travel = static_cast<int64_t>(repeatability_tolerance_ticks) *
                                 static_cast<int64_t>(kFullLegMinReapproachToleranceMultiple);
  if (repeatability_tolerance_ticks == 0 ||
      absTicksBetween(contact_raw, backoff_raw) < minimum_travel) {
    return FullLegPlanStatus::REJECT_BACKOFF;
  }

  *backoff_urad_out = backoff;
  return FullLegPlanStatus::OK;
}

FullLegPlanStatus resolveFullLegPlan(const actuator::CalibrationGeometryProfile& profile,
                                     const actuator::GeometryProvenance& expected_provenance,
                                     const actuator::JointTransformTable& transforms, Leg leg,
                                     FullLegPlan* out) {
  if (out == nullptr) return FullLegPlanStatus::REJECT_NULL_OUTPUT;
  *out = FullLegPlan{};
  if (!isKnownLeg(leg)) return FullLegPlanStatus::REJECT_UNKNOWN_LEG;
  if (!profile.bound() || !profile.provenanceMatches(expected_provenance) ||
      profile.provenanceTag() == actuator::kNoGeometryProvenance) {
    return FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND;
  }

  FullLegPlan plan{};
  plan.leg = leg;

  FullLegPlanStatus s = resolveLegJoint(profile, leg, JointKind::UPPER, &plan.upper);
  if (s != FullLegPlanStatus::OK) return s;
  s = resolveLegJoint(profile, leg, JointKind::HIP, &plan.hip);
  if (s != FullLegPlanStatus::OK) return s;
  s = resolveLegJoint(profile, leg, JointKind::LOWER, &plan.lower);
  if (s != FullLegPlanStatus::OK) return s;

  const actuator::GeometryEndpointRecord* min_endpoint =
      profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MIN_SIDE);
  const actuator::GeometryEndpointRecord* max_endpoint =
      profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MAX_SIDE);
  if (min_endpoint == nullptr || max_endpoint == nullptr) {
    return FullLegPlanStatus::REJECT_ENDPOINT_MISSING;
  }
  if (!actuator::isExecutable(*min_endpoint) || !actuator::isExecutable(*max_endpoint)) {
    return FullLegPlanStatus::REJECT_ENDPOINT_NOT_EXECUTABLE;
  }
  // The executor parks an auxiliary for the MAX side only. A MIN side that
  // needed one would be silently mis-run, so it is a refusal here.
  if (min_endpoint->has_auxiliary ||
      min_endpoint->parking != actuator::ParkingOutcome::NOT_NEEDED) {
    return FullLegPlanStatus::REJECT_MIN_NEEDS_AUXILIARY;
  }
  // The compiler's outcome and the auxiliary record must tell one story.
  const bool max_needs_auxiliary =
      max_endpoint->parking == actuator::ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND;
  if (max_needs_auxiliary != max_endpoint->has_auxiliary) {
    return FullLegPlanStatus::REJECT_MAX_PLAN_INCONSISTENT;
  }

  const actuator::GeometryProvenanceTag tag = profile.provenanceTag();
  const actuator::JointTransform* upper_transform = transforms.find(plan.upper.identity, tag);
  if (upper_transform == nullptr || !upper_transform->present) {
    return FullLegPlanStatus::REJECT_NO_TRANSFORM;
  }

  FullLegCalibrationRequest request{};
  request.probe_joint = plan.upper.identity;
  request.probe_bus_id = plan.upper.bus_id;
  request.endpoint_leg = leg;
  request.endpoint_joint = JointKind::UPPER;
  request.min_repeatability_tolerance_ticks = kFullLegRepeatabilityToleranceTicks;
  request.max_repeatability_tolerance_ticks = kFullLegRepeatabilityToleranceTicks;
  request.min_approach_urad = min_endpoint->contact;
  request.max_approach_urad = max_endpoint->contact;
  request.approach_overtravel_ticks = kFullLegApproachOvertravelTicks;

  s = deriveBackoffUrad(profile, expected_provenance, *upper_transform, *min_endpoint,
                        kFullLegRepeatabilityToleranceTicks, &request.min_backoff_urad);
  if (s != FullLegPlanStatus::OK) return s;
  s = deriveBackoffUrad(profile, expected_provenance, *upper_transform, *max_endpoint,
                        kFullLegRepeatabilityToleranceTicks, &request.max_backoff_urad);
  if (s != FullLegPlanStatus::OK) return s;

  // Both commanded approach points (contact + allowance) must exist in the raw
  // domain for this installation's q0 before anything moves.
  for (const actuator::GeometryEndpointRecord* endpoint : {min_endpoint, max_endpoint}) {
    uint16_t approach_raw = 0;
    if (actuator::resolveContactProbeApproachToRaw(
            profile, expected_provenance, *upper_transform, endpoint->contact, endpoint->side,
            kFullLegApproachOvertravelTicks, &approach_raw) != actuator::TargetResolveStatus::OK) {
      return FullLegPlanStatus::REJECT_TARGET_RESOLUTION;
    }
  }

  request.auxiliary_required = max_needs_auxiliary;
  if (max_needs_auxiliary) {
    FullLegJointRef aux{};
    s = resolveLegJoint(profile, max_endpoint->auxiliary_leg, max_endpoint->auxiliary_joint, &aux);
    if (s != FullLegPlanStatus::OK) return FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY;
    if (sameJoint(aux.identity, plan.upper.identity) || aux.bus_id == plan.upper.bus_id) {
      return FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY;
    }
    const actuator::JointTransform* aux_transform = transforms.find(aux.identity, tag);
    if (aux_transform == nullptr || !aux_transform->present) {
      return FullLegPlanStatus::REJECT_NO_TRANSFORM;
    }
    uint16_t aux_raw = 0;
    if (actuator::resolveUrdfQToRaw(profile, expected_provenance, *aux_transform,
                                    max_endpoint->auxiliary_target,
                                    &aux_raw) != actuator::TargetResolveStatus::OK) {
      return FullLegPlanStatus::REJECT_TARGET_RESOLUTION;
    }
    request.auxiliary_joint = aux.identity;
    request.auxiliary_bus_id = aux.bus_id;
    request.auxiliary_park_target_urad = max_endpoint->auxiliary_target;
  }

  plan.request = request;
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
    case FullLegPlanStatus::REJECT_ENDPOINT_NOT_EXECUTABLE: return "REJECT_ENDPOINT_NOT_EXECUTABLE";
    case FullLegPlanStatus::REJECT_MIN_NEEDS_AUXILIARY: return "REJECT_MIN_NEEDS_AUXILIARY";
    case FullLegPlanStatus::REJECT_MAX_PLAN_INCONSISTENT: return "REJECT_MAX_PLAN_INCONSISTENT";
    case FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY: return "REJECT_AUXILIARY_IDENTITY";
    case FullLegPlanStatus::REJECT_NO_TRANSFORM: return "REJECT_NO_TRANSFORM";
    case FullLegPlanStatus::REJECT_BACKOFF: return "REJECT_BACKOFF";
    case FullLegPlanStatus::REJECT_TARGET_RESOLUTION: return "REJECT_TARGET_RESOLUTION";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
