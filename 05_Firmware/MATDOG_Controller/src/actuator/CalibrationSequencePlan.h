#ifndef MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_H
#define MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_H

#include <stdint.h>

#include "../calibration/CalibrationDomain.h"
#include "CalibrationGeometryProfile.h"

// The 24-contact Full Calibration sequence plan: the LF V25 hardware-oracle
// state machine (matdog.rs run_lf_state_machine / prerequisites_for),
// generalized to LF, RF, RH and LH, with every prerequisite pose resolved
// and collision-checked on the CURRENT URDF + collision meshes.
//
// WHY A SECOND AUTHORIZATION OBJECT
// ---------------------------------
// The Geometry V5 endpoint profile (CalibrationGeometryProfileData.h) evaluates
// every endpoint from q=0 with all other joints at q=0 (plus one parking joint
// for the two obstructed front UPPER MAX paths). V25 never probed that way: it
// holds the leg's other joints at reviewed prerequisite poses while one joint
// searches (UPPER horizontal for the LOWER, UPPER raised + LOWER folded for the
// HIP) and moves between those poses in a fixed order. None of those
// configurations is covered by the V5 artifacts, and the V5 compiler marks all
// sixteen HIP/LOWER endpoints DIAGNOSTIC for its own q=0 context. This plan is
// what authorizes them instead - and only as a step of THIS sequence:
//
//   06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py
//       evaluates, on the same URDF (SHA-pinned) and the same V5 scene/mesh
//       kernel, every segment of every leg's sequence up to the calibration
//       guard (URDF limit + 64 ticks), and exports this plan's data
//       (CalibrationSequencePlanData.h) with the artifact's SHA256.
//
// The canonical contact, the URDF domain and the corridor still come from
// Geometry V5; this plan only adds the poses the OTHER joints hold.
//
// WHAT A PHASE MAY DO (the one table the policy and the executor share)
// ----------------------------------------------------------------------
//   INITIAL_RECOVERY  every leg joint of the robot -> q=0 (one at a time)
//   PARKING           the rear UPPER of a front leg -> park (Geometry V5 pose)
//   UPPER_MIN         leg HIP -> 0, leg LOWER -> 0 (prerequisites), then probe
//   UPPER_MAX         probe only
//   UPPER_HORIZONTAL  leg UPPER -> upper_for_lower   (V25 UPPER_90)
//   LOWER_MIN/MAX     probe only
//   LOWER_FOLDED      leg LOWER -> lower_folded      (V25 LOWER_FOLDED)
//                     leg UPPER -> upper_for_hip_min (V25 hip_upper_clearance)
//   HIP_MIN           probe only
//   HIP_MAX           leg HIP -> 0 and leg UPPER -> upper_for_hip_max, only
//                     when the two HIP clearance poses differ; then probe
//   RETURN_HIP        leg HIP -> 0
//   RETURN_LOWER_HELD leg LOWER -> 0
//   RETURN_UPPER      leg UPPER -> 0
//   RESTORE_PARKING   the rear UPPER -> 0
// Energizing a limp joint (GoalPosition := present, RAM TorqueLimit, torque
// on) is allowed only where a phase first needs that joint: INITIAL_RECOVERY
// (any leg joint), PARKING (the park joint), UPPER_MIN (the leg's three).
//
// Pure: <stdint.h> plus already-pure MATDOG units.

namespace matdog {
namespace actuator {

struct SequenceLegPlan {
  calibration::Leg leg = calibration::Leg::LF;
  // The geometry artifact's verdict for this leg's WHOLE sequence. False is a
  // refusal of every sequence move and every sequence probe for the leg.
  bool geometry_validated = false;
  // Front legs park their rear neighbour's UPPER for the whole session
  // (V25 LF: LH UPPER; Geometry V5 names the same joint for LF/RF UPPER MAX).
  bool has_rear_park = false;
  calibration::Leg park_leg = calibration::Leg::LF;
  calibration::JointKind park_joint = calibration::JointKind::UPPER;
  MicroRad park_target = 0;
  // Prerequisite poses of the leg's own joints (URDF q, micro-radians).
  MicroRad upper_for_lower = 0;    // UPPER held while the LOWER is probed
  MicroRad upper_for_hip_min = 0;  // UPPER held while HIP MIN is probed
  MicroRad upper_for_hip_max = 0;  // UPPER held while HIP MAX is probed
  MicroRad lower_folded = 0;       // LOWER held while the HIP is probed
};

struct CalibrationSequencePlan {
  const char* schema = nullptr;
  const char* artifact_sha256 = nullptr;       // the geometry validation artifact
  const char* urdf_sha256 = nullptr;           // must equal GeometryProvenance::urdf_sha256
  const char* mesh_manifest_sha256 = nullptr;  // must equal ::mesh_manifest_sha256
  SequenceLegPlan legs[calibration::kLegCount];
};

// A plan is usable only for the robot model it was computed on.
bool sequencePlanMatchesGeometry(const CalibrationSequencePlan& plan,
                                 const GeometryProvenance& provenance);

// The leg's plan, or nullptr (unknown leg, or a record for another leg).
const SequenceLegPlan* findSequenceLeg(const CalibrationSequencePlan& plan,
                                       calibration::Leg leg);

// The endpoint a measurement phase probes. False for every other phase.
bool sequenceProbeEndpoint(calibration::CalibrationPhase phase, calibration::JointKind* joint,
                           calibration::ContactSide* side);

// The leg's three joints, plus the rear park joint of a front leg.
bool sequenceParticipant(const SequenceLegPlan& leg_plan, const calibration::JointIdentity& joint);

// May `moving` be driven to exactly `target` (URDF q) in `phase`? See the
// table above; any other (phase, joint, target) is refused.
bool sequencePlanTargetAllowed(const SequenceLegPlan& leg_plan,
                               calibration::CalibrationPhase phase,
                               const calibration::JointIdentity& moving, MicroRad target);

// May a limp `moving` be energized in `phase`? See the table above.
bool sequenceEnergizeAllowed(const SequenceLegPlan& leg_plan,
                             calibration::CalibrationPhase phase,
                             const calibration::JointIdentity& moving);

// A GoalPosition prime (the V25 prepare_motor() write of the present position
// before TorqueEnable) is only ever issued near q=0: every joint the sequence
// energizes was first normalized to q=0 by INITIAL_RECOVERY. Also the
// automatic INITIAL_RECOVERY reach (V25 STARTUP_HOME_RECOVERY_LIMIT_TICKS).
constexpr uint16_t kSequencePrimeMaxDistanceTicks = 64;

enum class SequenceMoveKind : uint8_t {
  NONE             = 0,
  PRIME_AT_PRESENT = 1,  // GoalPosition := present position, torque still OFF
  TO_PLAN_TARGET   = 2,  // GoalPosition := a plan target of the current phase
};

const char* toString(SequenceMoveKind kind);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_H
