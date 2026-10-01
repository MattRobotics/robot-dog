#include "CalibrationSequencePlan.h"

namespace matdog {
namespace actuator {

using calibration::CalibrationPhase;
using calibration::ContactSide;
using calibration::JointIdentity;
using calibration::JointKind;
using calibration::Leg;

namespace {

bool sameHex(const char* a, const char* b) {
  if (a == nullptr || b == nullptr) return false;
  for (uint8_t i = 0; i < kSha256HexBytes; ++i) {
    if (a[i] != b[i]) return false;
    if (a[i] == '\0') return i == kSha256HexBytes - 1;  // exactly 64 hex digits
  }
  return false;
}

bool isLegJoint(const SequenceLegPlan& p, const JointIdentity& j, JointKind kind) {
  return j.valid() && j.leg == p.leg && j.joint == kind;
}

bool isParkJoint(const SequenceLegPlan& p, const JointIdentity& j) {
  return p.has_rear_park && j.valid() && j.leg == p.park_leg && j.joint == p.park_joint;
}

}  // namespace

bool sequencePlanMatchesGeometry(const CalibrationSequencePlan& plan,
                                 const GeometryProvenance& provenance) {
  return sameHex(plan.urdf_sha256, provenance.urdf_sha256) &&
         sameHex(plan.mesh_manifest_sha256, provenance.mesh_manifest_sha256);
}

const SequenceLegPlan* findSequenceLeg(const CalibrationSequencePlan& plan, Leg leg) {
  if (!calibration::isKnownLeg(leg)) return nullptr;
  const SequenceLegPlan& p = plan.legs[static_cast<uint8_t>(leg)];
  return p.leg == leg ? &p : nullptr;
}

bool sequenceProbeEndpoint(CalibrationPhase phase, JointKind* joint, ContactSide* side) {
  JointKind j = JointKind::UPPER;
  ContactSide s = ContactSide::MIN_SIDE;
  switch (phase) {
    case CalibrationPhase::UPPER_MIN: j = JointKind::UPPER; s = ContactSide::MIN_SIDE; break;
    case CalibrationPhase::UPPER_MAX: j = JointKind::UPPER; s = ContactSide::MAX_SIDE; break;
    case CalibrationPhase::LOWER_MIN: j = JointKind::LOWER; s = ContactSide::MIN_SIDE; break;
    case CalibrationPhase::LOWER_MAX: j = JointKind::LOWER; s = ContactSide::MAX_SIDE; break;
    case CalibrationPhase::HIP_MIN:   j = JointKind::HIP;   s = ContactSide::MIN_SIDE; break;
    case CalibrationPhase::HIP_MAX:   j = JointKind::HIP;   s = ContactSide::MAX_SIDE; break;
    default:
      return false;
  }
  if (joint != nullptr) *joint = j;
  if (side != nullptr) *side = s;
  return true;
}

bool sequenceParticipant(const SequenceLegPlan& p, const JointIdentity& j) {
  if (!j.valid()) return false;
  return j.leg == p.leg || isParkJoint(p, j);
}

bool sequencePlanTargetAllowed(const SequenceLegPlan& p, CalibrationPhase phase,
                               const JointIdentity& moving, MicroRad target) {
  if (!p.geometry_validated || !moving.valid()) return false;
  const bool hip = isLegJoint(p, moving, JointKind::HIP);
  const bool upper = isLegJoint(p, moving, JointKind::UPPER);
  const bool lower = isLegJoint(p, moving, JointKind::LOWER);
  const bool park = isParkJoint(p, moving);
  const bool hip_poses_differ = p.upper_for_hip_min != p.upper_for_hip_max;

  switch (phase) {
    case CalibrationPhase::INITIAL_RECOVERY:
      // Every leg joint of the robot, to q=0 and nowhere else.
      return target == 0;
    case CalibrationPhase::PARKING:
      return park && target == p.park_target;
    case CalibrationPhase::UPPER_MIN:
      return (hip || lower) && target == 0;
    case CalibrationPhase::UPPER_HORIZONTAL:
      return upper && target == p.upper_for_lower;
    case CalibrationPhase::LOWER_FOLDED:
      return (lower && target == p.lower_folded) || (upper && target == p.upper_for_hip_min);
    case CalibrationPhase::HIP_MAX:
      return hip_poses_differ &&
             ((hip && target == 0) || (upper && target == p.upper_for_hip_max));
    case CalibrationPhase::RETURN_HIP:
      return hip && target == 0;
    case CalibrationPhase::RETURN_LOWER_HELD:
      return lower && target == 0;
    case CalibrationPhase::RETURN_UPPER:
      return upper && target == 0;
    case CalibrationPhase::RESTORE_PARKING:
      return park && target == 0;
    case CalibrationPhase::PREFLIGHT:
    case CalibrationPhase::UPPER_MAX:
    case CalibrationPhase::LOWER_MIN:
    case CalibrationPhase::LOWER_MAX:
    case CalibrationPhase::HIP_MIN:
    case CalibrationPhase::DIAGNOSTICS:
    case CalibrationPhase::CLEANUP:
    case CalibrationPhase::TORQUE_OFF:
      return false;
  }
  return false;
}

bool sequenceEnergizeAllowed(const SequenceLegPlan& p, CalibrationPhase phase,
                             const JointIdentity& moving) {
  if (!p.geometry_validated || !moving.valid()) return false;
  switch (phase) {
    case CalibrationPhase::INITIAL_RECOVERY:
      return true;  // any leg joint of the robot; the caller checked it is one
    case CalibrationPhase::PARKING:
      return isParkJoint(p, moving);
    case CalibrationPhase::UPPER_MIN:
      return moving.leg == p.leg;
    default:
      return false;
  }
}

const char* toString(SequenceMoveKind kind) {
  switch (kind) {
    case SequenceMoveKind::NONE:             return "NONE";
    case SequenceMoveKind::PRIME_AT_PRESENT: return "PRIME_AT_PRESENT";
    case SequenceMoveKind::TO_PLAN_TARGET:   return "TO_PLAN_TARGET";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
