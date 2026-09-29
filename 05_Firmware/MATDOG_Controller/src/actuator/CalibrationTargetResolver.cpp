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

TargetResolveStatus resolveContactProbeApproachToRaw(const CalibrationGeometryProfile& profile,
                                                     const GeometryProvenance& expected_provenance,
                                                     const JointTransform& transform,
                                                     MicroRad contact_urad,
                                                     calibration::ContactSide side,
                                                     uint16_t overtravel_ticks,
                                                     uint16_t* raw_tick_out,
                                                     uint16_t* applied_overtravel_ticks_out) {
  if (raw_tick_out == nullptr) return TargetResolveStatus::REJECT_NULL_OUTPUT;
  if (overtravel_ticks > kContactProbeMaxOvertravelTicks) {
    return TargetResolveStatus::REJECT_OVERTRAVEL;
  }

  uint16_t contact_raw = 0;
  const TargetResolveStatus contact_status =
      resolveUrdfQToRaw(profile, expected_provenance, transform, contact_urad, &contact_raw);
  if (contact_status != TargetResolveStatus::OK) return contact_status;

  // resolveUrdfQToRaw() already validated the transform, so the direction is
  // known good here. Beyond the contact means q further from 0 on this side:
  // negative for MIN, positive for MAX, then mapped through the joint's raw
  // direction exactly as the forward conversion does.
  const int8_t direction = jointDirection(profile, transform.identity);
  const int64_t q_sign = (side == calibration::ContactSide::MIN_SIDE) ? -1 : 1;
  const int64_t step = static_cast<int64_t>(direction) * q_sign;

  // Clamp to the URDF joint limit (operator decision 2026-09-29): the largest
  // n <= overtravel_ticks whose tick still converts back INSIDE the declared
  // URDF domain through the one checked raw->q conversion. The allowance is a
  // ceiling, never a travel amount; nothing past the URDF limit is ever
  // produced. q is monotonic in n, so the first valid n from the top is it.
  for (int32_t n = overtravel_ticks; n >= 0; --n) {
    const int64_t raw = static_cast<int64_t>(contact_raw) + step * n;
    if (raw < 0 || raw >= kTicksPerRevolution) continue;
    MicroRad q_back = 0;
    if (resolveRawToUrdfQ(profile, expected_provenance, transform, static_cast<uint16_t>(raw),
                          &q_back) != TargetResolveStatus::OK) {
      continue;
    }
    *raw_tick_out = static_cast<uint16_t>(raw);
    if (applied_overtravel_ticks_out != nullptr) {
      *applied_overtravel_ticks_out = static_cast<uint16_t>(n);
    }
    return TargetResolveStatus::OK;
  }
  return TargetResolveStatus::REJECT_URDF_LIMIT;
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
    case TargetResolveStatus::REJECT_OVERTRAVEL: return "REJECT_OVERTRAVEL";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
