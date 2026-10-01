#include "CalibrationTargetResolver.h"

namespace matdog {
namespace actuator {

namespace {

int64_t roundDivSigned(int64_t numerator, int64_t denominator) {
  if (denominator <= 0) return 0;
  if (numerator >= 0) return (numerator + denominator / 2) / denominator;
  return -((-numerator + denominator / 2) / denominator);
}

TargetResolveStatus commonChecks(const CalibrationGeometryProfile& profile,
                                 const GeometryProvenance& expected_provenance,
                                 const JointTransform& transform,
                                 const GeometryJointRecord** joint_out,
                                 int8_t* direction_out) {
  if (!profile.bound() ||
      !profile.provenanceMatches(expected_provenance) ||
      profile.provenanceTag() == kNoGeometryProvenance) {
    return TargetResolveStatus::REJECT_GEOMETRY;
  }
  if (!transform.usableProvenance() ||
      !transformMayBeAppliedTo(transform, transform.identity) ||
      transform.geometry != profile.provenanceTag()) {
    return TargetResolveStatus::REJECT_TRANSFORM;
  }

  const GeometryJointRecord* joint = profile.findJoint(transform.identity);
  if (joint == nullptr) return TargetResolveStatus::REJECT_JOINT;
  const int8_t direction = jointDirection(profile, transform.identity);
  if (direction != 1 && direction != -1) return TargetResolveStatus::REJECT_DIRECTION;

  if (joint_out != nullptr) *joint_out = joint;
  if (direction_out != nullptr) *direction_out = direction;
  return TargetResolveStatus::OK;
}

}  // namespace

TargetResolveStatus resolveUrdfQToRaw(const CalibrationGeometryProfile& profile,
                                      const GeometryProvenance& expected_provenance,
                                      const JointTransform& transform,
                                      MicroRad q_urad,
                                      uint16_t* raw_tick_out) {
  if (raw_tick_out == nullptr) return TargetResolveStatus::REJECT_NULL_OUTPUT;

  const GeometryJointRecord* joint = nullptr;
  int8_t direction = 0;
  const TargetResolveStatus common =
      commonChecks(profile, expected_provenance, transform, &joint, &direction);
  if (common != TargetResolveStatus::OK) return common;

  if (q_urad < joint->urdf_lower || q_urad > joint->urdf_upper) {
    return TargetResolveStatus::REJECT_URDF_LIMIT;
  }

  const int64_t signed_ticks =
      roundDivSigned(static_cast<int64_t>(q_urad) * kTicksPerRevolution,
                     kMicroRadPerRevolution);
  const int64_t raw =
      static_cast<int64_t>(transform.q0_tick) +
      static_cast<int64_t>(direction) * signed_ticks;

  if (raw < 0 || raw >= kTicksPerRevolution) {
    return TargetResolveStatus::REJECT_RAW_DOMAIN;
  }

  *raw_tick_out = static_cast<uint16_t>(raw);
  return TargetResolveStatus::OK;
}

TargetResolveStatus resolveRawToUrdfQ(const CalibrationGeometryProfile& profile,
                                      const GeometryProvenance& expected_provenance,
                                      const JointTransform& transform,
                                      uint16_t raw_tick,
                                      MicroRad* q_urad_out) {
  if (q_urad_out == nullptr) return TargetResolveStatus::REJECT_NULL_OUTPUT;
  if (raw_tick >= static_cast<uint16_t>(kTicksPerRevolution)) {
    return TargetResolveStatus::REJECT_RAW_DOMAIN;
  }

  const GeometryJointRecord* joint = nullptr;
  int8_t direction = 0;
  const TargetResolveStatus common =
      commonChecks(profile, expected_provenance, transform, &joint, &direction);
  if (common != TargetResolveStatus::OK) return common;

  const int64_t raw_delta =
      static_cast<int64_t>(raw_tick) - static_cast<int64_t>(transform.q0_tick);
  const int64_t q =
      roundDivSigned(raw_delta * static_cast<int64_t>(direction) *
                         kMicroRadPerRevolution,
                     kTicksPerRevolution);

  if (q < joint->urdf_lower || q > joint->urdf_upper) {
    return TargetResolveStatus::REJECT_URDF_LIMIT;
  }
  *q_urad_out = static_cast<MicroRad>(q);
  return TargetResolveStatus::OK;
}

TargetResolveStatus resolveDeltaFromQ0(const CalibrationGeometryProfile& profile,
                                       const GeometryProvenance& expected_provenance,
                                       const JointTransform& transform,
                                       int32_t delta_ticks,
                                       uint16_t* raw_tick_out) {
  if (raw_tick_out == nullptr) return TargetResolveStatus::REJECT_NULL_OUTPUT;

  const TargetResolveStatus common =
      commonChecks(profile, expected_provenance, transform, nullptr, nullptr);
  if (common != TargetResolveStatus::OK) return common;

  const int64_t raw =
      static_cast<int64_t>(transform.q0_tick) + static_cast<int64_t>(delta_ticks);
  if (raw < 0 || raw >= kTicksPerRevolution) {
    return TargetResolveStatus::REJECT_RAW_DOMAIN;
  }
  *raw_tick_out = static_cast<uint16_t>(raw);
  return TargetResolveStatus::OK;
}

int32_t searchDepth(const CalibrationSearchCorridor& corridor, uint16_t tick) {
  return (static_cast<int32_t>(tick) - static_cast<int32_t>(corridor.home_tick)) *
         static_cast<int32_t>(corridor.probe_sign);
}

bool searchCorridorAdmits(const CalibrationSearchCorridor& corridor, uint16_t tick) {
  if (!corridor.valid() || tick >= kTicksPerRevolution) return false;
  const int32_t depth = searchDepth(corridor, tick);
  return depth >= searchDepth(corridor, corridor.opposite_limit_tick) &&
         depth <= searchDepth(corridor, corridor.guard_tick);
}

bool searchCorridorAccepts(const CalibrationSearchCorridor& corridor, uint16_t tick) {
  if (!corridor.valid() || tick >= kTicksPerRevolution) return false;
  const int32_t depth = searchDepth(corridor, tick);
  return depth >= searchDepth(corridor, corridor.entry_tick) &&
         depth <= searchDepth(corridor, corridor.guard_tick);
}

TargetResolveStatus resolveCalibrationSearchCorridor(const CalibrationGeometryProfile& profile,
                                                     const GeometryProvenance& expected_provenance,
                                                     const JointTransform& transform,
                                                     calibration::Leg endpoint_leg,
                                                     calibration::JointKind endpoint_joint,
                                                     calibration::ContactSide side,
                                                     CalibrationSearchCorridor* out) {
  if (out == nullptr) return TargetResolveStatus::REJECT_NULL_OUTPUT;
  *out = CalibrationSearchCorridor{};

  const GeometryJointRecord* joint = nullptr;
  int8_t direction = 0;
  const TargetResolveStatus common =
      commonChecks(profile, expected_provenance, transform, &joint, &direction);
  if (common != TargetResolveStatus::OK) return common;
  // The corridor belongs to the probed joint itself.
  if (transform.identity.leg != endpoint_leg || transform.identity.joint != endpoint_joint) {
    return TargetResolveStatus::REJECT_JOINT;
  }
  const GeometryEndpointRecord* endpoint = profile.findEndpoint(endpoint_leg, endpoint_joint, side);
  if (endpoint == nullptr) return TargetResolveStatus::REJECT_SEARCH_CORRIDOR;

  const bool min_side = side == calibration::ContactSide::MIN_SIDE;
  CalibrationSearchCorridor c{};
  c.probe_sign = static_cast<int8_t>(direction * (min_side ? -1 : 1));
  c.home_tick = transform.q0_tick;
  // The canonical contact is a REFERENCE, never a commanded target: sixteen of
  // the twenty-four V5 contacts (every HIP and LOWER endpoint) lie just
  // outside the URDF domain, so it is converted without the URDF-domain check
  // and then required to lie inside [entry, guard] below. Every commanded
  // search target stays bounded by the URDF-derived opposite limit and guard.
  {
    const int64_t signed_ticks = roundDivSigned(
        static_cast<int64_t>(endpoint->contact) * kTicksPerRevolution, kMicroRadPerRevolution);
    const int64_t raw = static_cast<int64_t>(transform.q0_tick) +
                        static_cast<int64_t>(direction) * signed_ticks;
    if (raw < 0 || raw >= kTicksPerRevolution) return TargetResolveStatus::REJECT_RAW_DOMAIN;
    c.contact_tick = static_cast<uint16_t>(raw);
  }
  TargetResolveStatus s =
      resolveUrdfQToRaw(profile, expected_provenance, transform,
                        min_side ? joint->urdf_lower : joint->urdf_upper, &c.urdf_limit_tick);
  if (s != TargetResolveStatus::OK) return s;
  s = resolveUrdfQToRaw(profile, expected_provenance, transform,
                        min_side ? joint->urdf_upper : joint->urdf_lower, &c.opposite_limit_tick);
  if (s != TargetResolveStatus::OK) return s;

  const int64_t entry = static_cast<int64_t>(c.urdf_limit_tick) -
                        static_cast<int64_t>(c.probe_sign) * kCalibrationSearchAcceptanceInnerTicks;
  const int64_t guard = static_cast<int64_t>(c.urdf_limit_tick) +
                        static_cast<int64_t>(c.probe_sign) * kCalibrationSearchGuardOvershootTicks;
  if (entry < 0 || entry >= kTicksPerRevolution || guard < 0 || guard >= kTicksPerRevolution) {
    return TargetResolveStatus::REJECT_RAW_DOMAIN;
  }
  c.entry_tick = static_cast<uint16_t>(entry);
  c.guard_tick = static_cast<uint16_t>(guard);

  // home < entry <= contact <= guard along the probe axis; the other side's
  // limit behind home. Anything else is a geometry/q0 combination this search
  // was never designed for, and is refused rather than improvised around.
  const int32_t entry_depth = searchDepth(c, c.entry_tick);
  const int32_t contact_depth = searchDepth(c, c.contact_tick);
  const int32_t guard_depth = searchDepth(c, c.guard_tick);
  if (!(entry_depth > 0 && entry_depth <= contact_depth && contact_depth <= guard_depth &&
        searchDepth(c, c.opposite_limit_tick) < 0)) {
    return TargetResolveStatus::REJECT_SEARCH_CORRIDOR;
  }
  *out = c;
  return TargetResolveStatus::OK;
}

const char* toString(TargetResolveStatus status) {
  switch (status) {
    case TargetResolveStatus::OK: return "OK";
    case TargetResolveStatus::REJECT_NULL_OUTPUT: return "REJECT_NULL_OUTPUT";
    case TargetResolveStatus::REJECT_GEOMETRY: return "REJECT_GEOMETRY";
    case TargetResolveStatus::REJECT_TRANSFORM: return "REJECT_TRANSFORM";
    case TargetResolveStatus::REJECT_JOINT: return "REJECT_JOINT";
    case TargetResolveStatus::REJECT_DIRECTION: return "REJECT_DIRECTION";
    case TargetResolveStatus::REJECT_URDF_LIMIT: return "REJECT_URDF_LIMIT";
    case TargetResolveStatus::REJECT_RAW_DOMAIN: return "REJECT_RAW_DOMAIN";
    case TargetResolveStatus::REJECT_SEARCH_CORRIDOR: return "REJECT_SEARCH_CORRIDOR";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
