#ifndef MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_H
#define MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_H

#include <stdint.h>

#include "../calibration/CalibrationDomain.h"

// The Controller's view of the offline Geometry Compiler V5 result.
//
// Pure: <stdint.h> plus the calibration domain model. No Arduino, no mesh, no
// forward kinematics, no collision maths. The device NEVER recomputes
// geometry - it verifies provenance and executes prevalidated plan primitives.
//
//   URDF + collision meshes
//        -> Geometry Compiler V5                  (offline, unchanged)
//        -> canonical bundle JSON                 (09_Logs, unchanged)
//        -> matdog_calibration_geometry_export.py (pure reduction)
//        -> CalibrationGeometryProfileData.h      (generated constexpr tables)
//        -> THIS                                  (lookup + invariants)
//
// WHY A GENERATED TABLE AND NOT A PARSED BLOB: no JSON parser, no heap and no
// mesh on the ESP32-S3, and the profile cannot be swapped under a running
// image. Regenerating geometry costs a rebuild, which is the correct price for
// data that will one day authorise motion.
//
// THE THREE MEANINGS THIS TYPE KEEPS APART, because collapsing any two is the
// most dangerous thing available here:
//
//   geometric contact   !=  executable target
//   clearance PASS      !=  motion authorization
//   diagnostic endpoint !=  a place the robot may be commanded to
//
// The canonical V5 bundle contains 24 geometric contacts of which only 8 lie
// inside the URDF limits. The other 16 are DIAGNOSTIC: the mechanical contact
// happens just beyond the declared limit, so reaching it is a measurement
// question, not a motion permission. isExecutable() is the only door, and
// static_audit.py fails the build if anything else tries to open one.

namespace matdog {
namespace actuator {

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------
//
// Integer micro-radians. The compiler's own bisection resolution is 1e-4 rad,
// so 1e-6 rad is two orders finer than the evidence and cannot lose a
// distinction the geometry actually made. Integer, because a float comparison
// against a safety bound is a comparison whose failure mode depends on the
// compiler.
using MicroRad = int32_t;

// ST3215 / MATDOG_C018_V1: 4096 counts per revolution, unsigned 0..4095 goal
// domain. A SERVO PROFILE fact, verified for all 17 units by the 2026-08-27
// campaign - it is not calibration and says nothing about joint zero.
constexpr int32_t kTicksPerRevolution = 4096;

// ceil(2*pi * 1e6). Rounded UP on purpose: every magnitude computed from it is
// then an over-estimate, so a marginal excursion is rejected rather than
// admitted.
constexpr int64_t kMicroRadPerRevolution = 6283186;

// |ticks| expressed as a micro-radian magnitude, rounded UP. Conservative in
// the only direction that matters: the answer is never smaller than the true
// excursion, so a bound check can never pass by rounding.
int64_t ticksToMicroRadMagnitude(int32_t ticks);

// ---------------------------------------------------------------------------
// Vocabulary - copied from the canonical bundle, not invented here
// ---------------------------------------------------------------------------

// matdog.geometry_path_parking.v2 `target_domain`.
enum class TargetDomain : uint8_t {
  // The geometric contact lies inside the URDF joint limits. Eight of the 24.
  EXECUTABLE_URDF_DOMAIN = 0,
  // The geometric contact lies BEYOND the declared URDF limit. Sixteen of the
  // 24. Evidence about where the mechanism stops - never a motion target.
  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS = 1,
};

// matdog.geometry_path_parking.v2 `outcome`.
enum class ParkingOutcome : uint8_t {
  // The direct q=0 -> target path is collision free. Eighteen of the 24.
  NOT_NEEDED = 0,
  // The direct path is obstructed; one auxiliary joint must be parked first.
  // Six of the 24, and the compiler needed no 2-DOF plan for any of them.
  FEASIBLE_1DOF_PLAN_FOUND = 1,
};

// matdog.geometry_safety_policy.v1 `policy_result`, threshold 3 mm.
enum class ClearancePolicyResult : uint8_t {
  PASS = 0,
  // A conservative lower bound below the threshold. The true clearance may be
  // fine; the evidence does not prove it. Eight of the 24, all of them
  // DIAGNOSTIC endpoints. UNRESOLVED IS NOT PASS.
  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD = 1,
  // An exact distance below the threshold. Zero of the 24 today.
  FAIL = 2,
};

// ---------------------------------------------------------------------------
// Provenance
// ---------------------------------------------------------------------------

constexpr uint8_t kSha256HexBytes = 65;  // 64 hex digits + NUL

// The immutable inputs the canonical V5 run was gated on. The Controller acts
// on a profile only when these match what it was built to expect: a geometry
// plan is only as valid as the model it came from.
struct GeometryProvenance {
  char urdf_sha256[kSha256HexBytes];
  char mesh_manifest_sha256[kSha256HexBytes];
  char endpoint_semantic_sha256[kSha256HexBytes];
  char parking_semantic_sha256[kSha256HexBytes];
  char safety_policy_semantic_sha256[kSha256HexBytes];
  char allocation_sha256[kSha256HexBytes];
};

bool sameProvenance(const GeometryProvenance& a, const GeometryProvenance& b);

// ---------------------------------------------------------------------------
// Geometry provenance tag
// ---------------------------------------------------------------------------
//
// A compact identity for one GeometryProvenance, so a piece of calibration
// evidence can record WHICH model it was measured under without carrying 390
// bytes of hex per record.
//
// THIS IS AN IDENTITY TAG, NOT A SECURITY DIGEST. It answers "was this
// measured under the model that is loaded right now?", which is a question
// about accidental drift - a rebuild from a different URDF, a regenerated
// profile - and not about an adversary. The six SHA-256 hashes remain the
// authority and stay available on the bound profile for reporting.
//
// THREE AXES, DELIBERATELY SEPARATE. Conflating any two of them loses a
// distinction that matters:
//
//   physical unit identity   WHICH SERVO produced the evidence
//                            (JointIdentity; survives a geometry change)
//   calibration provenance   HOW GOOD the evidence is
//                            (EvidenceState + CalibrationOrigin)
//   geometry provenance      WHICH MODEL it was measured against
//                            (this tag; survives a servo change)
//
// A servo swap invalidates the first and leaves the other two intact. A URDF
// change invalidates the third and leaves the other two intact.
using GeometryProvenanceTag = uint64_t;

// Never a valid tag: an unbound record can never be current evidence.
constexpr GeometryProvenanceTag kNoGeometryProvenance = 0;

// FNV-1a over the six hashes, with separators so a field boundary cannot be
// forged by shifting characters between adjacent fields. Never returns 0.
GeometryProvenanceTag geometryProvenanceTag(const GeometryProvenance& provenance);

// ---------------------------------------------------------------------------
// Records
// ---------------------------------------------------------------------------

struct GeometryJointRecord {
  calibration::JointIdentity identity;  // leg, kind, PHYSICAL UNIT - never a bus id alone
  uint8_t bus_id;                       // transport metadata; cross-checked against URDF motorId
  // THE JOINT'S DIRECTION. Hardware-contract data, not a recalibration datum.
  //
  // The canonical URDF carries the per-joint motorDirection and those
  // directions were validated on real hardware. The 2026-08-27 reprovisioning
  // changed the physical units, the PositionOffset baseline and the raw q0
  // installation. It did NOT change the servo model, the mounting
  // orientation, the joint mechanical architecture, the URDF joint axes or
  // motorDirection - so direction did not become unknown.
  //
  //   q0              CURRENT INSTALLATION CALIBRATION DATA - measured
  //   motorDirection  CURRENT URDF / HARDWARE CONTRACT DATA - read, not measured
  //
  // Replacing a servo with the same type in the same mounting needs a new q0
  // capture; it does NOT need direction re-verification. Direction is only
  // reconsidered when the servo's physical orientation, the transmission
  // topology, the servo type / encoder convention, the URDF joint axis or
  // motorDirection change - or when contradictory hardware evidence appears.
  // Every one of those changes the URDF, and therefore the provenance tag.
  int8_t urdf_motor_direction;
  MicroRad urdf_lower;
  MicroRad urdf_upper;
  // The SYMMETRIC proven-clear half-span around q=0 for this joint alone, all
  // others at q=0: min over both sides of the last angle the compiler sampled
  // and found clear. Symmetric on purpose - a direction-verification move is
  // commanded as a raw tick delta BEFORE the joint's sign is known, so the
  // same magnitude must be clear whichever way it actually turns.
  MicroRad clear_half_span;
  uint16_t provisioned_center_raw;      // 2026-08-27 cold verify; a PRIOR, never a q0
  int16_t provisioned_center_error_ticks;
};

struct GeometryEndpointRecord {
  calibration::Leg leg;
  calibration::JointKind joint;
  calibration::ContactSide side;
  TargetDomain domain;
  ParkingOutcome parking;
  ClearancePolicyResult clearance;
  MicroRad contact;         // the bisected geometric contact angle
  MicroRad clear;           // the last sampled-clear angle before it (signed)
  MicroRad declared_limit;  // the URDF limit for this side
  bool has_auxiliary;
  calibration::Leg auxiliary_leg;
  calibration::JointKind auxiliary_joint;
  MicroRad auxiliary_target;  // the parked pose the compiler found, exactly
};

// The one door. An endpoint may be considered as a motion target only when its
// geometric contact lies inside the URDF limits AND the clearance policy
// returned PASS. UNRESOLVED and FAIL are both refusals, and a DIAGNOSTIC
// endpoint is refused however clean its clearance looks.
bool isExecutable(const GeometryEndpointRecord& endpoint);

// ---------------------------------------------------------------------------
// The profile
// ---------------------------------------------------------------------------

class CalibrationGeometryProfile {
 public:
  // Null pointers or a zero count leave the profile unbound; unbound is a
  // refusal, not a permission.
  void bind(const GeometryProvenance* provenance,
            const GeometryJointRecord* joints, uint8_t joint_count,
            const GeometryEndpointRecord* endpoints, uint8_t endpoint_count);
  void clear();

  bool bound() const { return provenance_ != nullptr && joint_count_ > 0 && endpoint_count_ > 0; }

  // The Controller must be able to say WHICH model it is acting on, and refuse
  // any other. Compares all six hashes.
  bool provenanceMatches(const GeometryProvenance& expected) const;

  // kNoGeometryProvenance when nothing is bound, so an unbound profile can
  // never stamp or match a record.
  GeometryProvenanceTag provenanceTag() const;

  const GeometryProvenance* provenance() const { return provenance_; }
  uint8_t jointCount() const { return joint_count_; }
  uint8_t endpointCount() const { return endpoint_count_; }

  // Both identity axes must agree - slot AND physical unit. Slot agreement
  // alone is exactly what the 2026-08-27 reassembly made unsafe.
  const GeometryJointRecord* findJoint(const calibration::JointIdentity& identity) const;

  const GeometryEndpointRecord* findEndpoint(calibration::Leg leg,
                                             calibration::JointKind joint,
                                             calibration::ContactSide side) const;

  // Is a raw excursion of |delta_ticks| from the captured q0 tick inside the
  // proven-clear symmetric envelope for this joint? Answers in tick space on
  // purpose: the magnitude of a tick delta is a servo-profile fact and needs
  // neither q0 nor direction, which is what makes a direction-verification
  // move checkable BEFORE either exists.
  bool withinDirectionVerifyEnvelope(const calibration::JointIdentity& identity,
                                     int32_t delta_ticks) const;

 private:
  const GeometryProvenance* provenance_ = nullptr;
  const GeometryJointRecord* joints_ = nullptr;
  const GeometryEndpointRecord* endpoints_ = nullptr;
  uint8_t joint_count_ = 0;
  uint8_t endpoint_count_ = 0;
};

// ---------------------------------------------------------------------------
// Raw tick <-> URDF q transform
// ---------------------------------------------------------------------------
//
// The missing link of the whole bootstrap, and deliberately absent today:
//
//   q0        MEASURED, read-only, with torque off, at the manually aligned
//             URDF q=0 pose. It is NOT 2048 - the provisioned raw centre is a
//             servo-level fact and a sanity prior, nothing more. This is the
//             ONLY half that a reprovisioning invalidates.
//   direction NOT carried here at all. It is contract data read from the
//             bound profile's GeometryJointRecord::urdf_motor_direction, and
//             it is already hardware-validated. Storing a measured copy would
//             create a second source of truth that could silently disagree
//             with the URDF the geometry plan was compiled against.
//
// The two are therefore invalidated by DIFFERENT events, which is the point:
// a same-type servo replacement in the same mounting invalidates q0 and leaves
// direction untouched, while a URDF change moves the provenance tag and makes
// the whole transform stale.
//
// Until q0 exists with operational provenance, every angle-targeted
// calibration operation resolves to a refusal. That is the correct state after
// CALIBRATION_RESET_PENDING_FULL_RECALIBRATION, not a gap to paper over.
struct JointTransform {
  calibration::JointIdentity identity{};
  calibration::EvidenceState state = calibration::EvidenceState::UNKNOWN;
  calibration::CalibrationOrigin origin = calibration::CalibrationOrigin::NONE;
  // WHICH MODEL this was measured under. q0 is captured at "the nominal URDF
  // q=0 pose", so it is meaningless against a different URDF - the pose it
  // refers to is not the same pose.
  GeometryProvenanceTag geometry = kNoGeometryProvenance;
  uint16_t q0_tick = 0;
  // No `direction` field, deliberately. See jointDirection() below: direction
  // is resolved from the bound profile, never stored as measured evidence.
  bool present = false;

  // Operational provenance, the same test calibration::q0MayBeAppliedTo
  // applies: measured on the current installation and promoted to operational
  // calibration. A replay can reach neither.
  bool usableProvenance() const;

  // The geometry axis, kept separate from usableProvenance() on purpose: a
  // transform can be perfectly measured and still describe a model that is no
  // longer loaded. That record stays historically valid; it is simply not
  // CURRENT operational evidence.
  bool boundToGeometry() const { return geometry != kNoGeometryProvenance; }
};

bool transformMayBeAppliedTo(const JointTransform& transform,
                             const calibration::JointIdentity& current);

// The joint's direction, from the CURRENT URDF via the bound profile. Returns
// 0 when the profile is unbound or does not know the joint - which fails
// closed, because 0 is not a usable sign.
//
// Resolved rather than stored on purpose. A change to the URDF's motorDirection
// changes the URDF hash, which changes the geometry provenance tag, which makes
// every transform measured under the old model stale. One source of truth, one
// invalidation path.
int8_t jointDirection(const CalibrationGeometryProfile& profile,
                      const calibration::JointIdentity& joint);

const char* toString(TargetDomain domain);
const char* toString(ParkingOutcome outcome);
const char* toString(ClearancePolicyResult result);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_H
