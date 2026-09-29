#ifndef MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_FINALIZER_H
#define MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_FINALIZER_H

#include <stddef.h>
#include <stdint.h>

#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationGeometryProfile.h"
#include "../actuator/OperationalEnvelope.h"
#include "../core/ActuatorAuthority.h"
#include "CalibrationDomain.h"
#include "CalibrationManager.h"
#include "CalibrationMotionPermit.h"
#include "FullLegCalibrationExecutor.h"
#include "FullLegCalibrationPlan.h"

// Closes the evidence lifecycle of ONE Full Leg run, and holds the RAM record
// of every leg run in this power-up.
//
// The executor proves two UPPER contacts and leaves the servos SAFE_OFF. That
// is a measurement, not a calibration: nothing has yet been recorded in the
// session, turned into an envelope, offered to the actuator policy, or
// released. finalizeFullLeg() does that, once, in one fixed order:
//
//   1  the run is terminal; a FAILED run only cleans up
//   2  the live session is the one the run started under (leg AND session id)
//   3  recordContact(MIN) and recordContact(MAX), each exactly once
//   4  UPPER / HIP / LOWER envelopes are built
//   5  limits: admitted ONLY under approved parameters (see below)
//   6  noteExecutionPhase(TORQUE_OFF), then completeSession()
//   7  permit + operator authorization revoked
//   8  post-conditions verified: session terminal, authority NONE, permit off
//
// A refusal at any step ends the session as FAILED, revokes the permit, and
// records why. Nothing after the refusing step runs.
//
// TWO LEVELS, NEVER CONFLATED
// ---------------------------
//   HARDWARE_CONTACT_CALIBRATED
//       Both UPPER contacts were witnessed, recorded exactly once, the UPPER
//       envelope derived from them is READY, the servos are SAFE_OFF and the
//       session completed cleanly. This is what a hardware run can honestly
//       claim TODAY.
//   FINAL_OPERATIONAL_ENVELOPE_ACCEPTED
//       Additionally, UPPER / HIP / LOWER envelopes were built from APPROVED
//       parameters and all three JointLimits were admitted and read back.
//
// The second level needs a reviewed stand/gait workspace and margin. The
// repository has none (MATDOG_JOINT_CALIBRATION.yaml records
// first_stand_limit_rad as null for all twelve joints); the numbers below are
// placeholders and productionEnvelopeParameters() marks them unapproved. An
// unapproved run builds and REPORTS its envelopes but never offers them to
// SafeActuatorPolicy - a placeholder must not become a POSITION_COMMAND bound.
// Approving parameters is a deliberate, reviewed source change to
// kFullLegOperationalParametersApproved, which static_audit.py pins to false.
//
// Pure: no Arduino, no ServoBus, no time. The Controller owns the collaborators
// and calls this exactly once per terminal executor.

namespace matdog {
namespace calibration {

// Placeholders, in the sense above. Reused unchanged from the LF precedent.
constexpr bool kFullLegOperationalParametersApproved = false;
constexpr uint16_t kFullLegPlaceholderUpperMarginTicks = 8;
constexpr actuator::MicroRad kFullLegPlaceholderHipLowerMarginUrad = 50000;

struct FullLegEnvelopeParameters {
  bool approved = false;
  // Symmetric inset applied to both confirmed UPPER contact ticks.
  uint16_t upper_margin_ticks = 0;
  // Symmetric inset applied to the full URDF domain of HIP and LOWER.
  actuator::MicroRad hip_lower_margin_urad = 0;
};

FullLegEnvelopeParameters productionEnvelopeParameters();

enum class FullLegVerdict : uint8_t {
  NOT_RUN = 0,
  FAILED = 1,
  HARDWARE_CONTACT_CALIBRATED = 2,
  FINAL_OPERATIONAL_ENVELOPE_ACCEPTED = 3,
};

enum class FullLegLimitAdmission : uint8_t {
  NOT_EVALUATED = 0,
  ADMITTED = 1,
  NOT_ADMITTED_UNAPPROVED_PARAMETERS = 2,
  REJECTED_BY_POLICY = 3,
  VERIFY_FAILED = 4,
};

enum class FullLegFinalizeFailure : uint8_t {
  NONE = 0,
  CONTEXT_INCOMPLETE = 1,
  RUN_NOT_TERMINAL = 2,
  EXECUTOR_FAILED = 3,            // the executor's own failure is in executor_failure
  SESSION_NOT_ACTIVE = 4,         // the session ended (or never was ACTIVE) under the run
  SESSION_MISMATCH = 5,           // a different leg or session id than the run started under
  CONTACT_EVIDENCE_MALFORMED = 6, // wrong leg / joint / side, or no measurement
  MIN_CONTACT_REJECTED = 7,
  MAX_CONTACT_REJECTED = 8,
  UPPER_ENVELOPE_NOT_READY = 9,
  HIP_ENVELOPE_NOT_READY = 10,    // approved parameters only
  LOWER_ENVELOPE_NOT_READY = 11,  // approved parameters only
  LIMIT_ADMISSION_REJECTED = 12,
  LIMIT_VERIFY_FAILED = 13,
  PHASE_REPORT_REJECTED = 14,
  SESSION_COMPLETE_REJECTED = 15,
  CLEANUP_SESSION_NOT_TERMINAL = 16,
  CLEANUP_AUTHORITY_HELD = 17,
  CLEANUP_PERMIT_ACTIVE = 18,
};

// What the finalizer needs to know about a finished run. Snapshotted by the
// Controller from the executor at the moment it turns terminal.
struct FullLegRunOutcome {
  bool terminal = false;
  bool complete = false;  // executor phase == COMPLETE (both sides witnessed, SAFE_OFF verified)
  FullLegCalibrationFailure failure = FullLegCalibrationFailure::NONE;
  ContactEvidence min_contact{};
  ContactEvidence max_contact{};
  // The geometry the run STARTED under and the session it ran in. The
  // envelope builder refuses evidence whose geometry is no longer current.
  actuator::GeometryProvenanceTag geometry_at_start = actuator::kNoGeometryProvenance;
  uint32_t session_id_at_start = 0;
};

FullLegRunOutcome outcomeFromExecutor(const FullLegCalibrationExecutor& executor,
                                      actuator::GeometryProvenanceTag geometry_at_start,
                                      uint32_t session_id_at_start);

struct FullLegJointRecord {
  JointIdentity identity{};
  uint8_t bus_id = 0;

  // The q0 this leg's evidence rests on, as the policy held it at finalization.
  bool q0_present = false;
  EvidenceState q0_state = EvidenceState::UNKNOWN;
  CalibrationOrigin q0_origin = CalibrationOrigin::NONE;
  actuator::GeometryProvenanceTag q0_geometry = actuator::kNoGeometryProvenance;
  uint16_t q0_tick = 0;

  actuator::EnvelopeBuildStatus envelope_status = actuator::EnvelopeBuildStatus::NOT_EVALUATED;
  actuator::OperationalEnvelope envelope{};

  FullLegLimitAdmission limit = FullLegLimitAdmission::NOT_EVALUATED;
  actuator::LimitAdmission limit_policy_detail = actuator::LimitAdmission::ADMITTED;  // valid only for REJECTED_BY_POLICY
};

struct FullLegRecord {
  Leg leg = Leg::LF;
  bool present = false;  // a run reached finalization
  uint16_t attempts = 0;
  uint32_t session_id = 0;
  actuator::GeometryProvenanceTag geometry = actuator::kNoGeometryProvenance;

  // Indexed by static_cast<uint8_t>(JointKind): HIP, UPPER, LOWER.
  FullLegJointRecord joints[kJointKindCount];

  bool auxiliary_required = false;
  Leg auxiliary_leg = Leg::LF;
  JointKind auxiliary_joint = JointKind::UPPER;
  uint8_t auxiliary_bus_id = 0;
  actuator::MicroRad auxiliary_park_target_urad = 0;

  bool min_recorded = false;
  bool max_recorded = false;
  ContactEvidence min_contact{};
  ContactEvidence max_contact{};

  bool parameters_approved = false;
  uint16_t upper_margin_ticks = 0;
  actuator::MicroRad hip_lower_margin_urad = 0;

  bool session_completed = false;
  bool permit_revoked = false;
  bool authority_released = false;

  bool hardware_contact_calibrated = false;
  bool operational_envelope_accepted = false;
  FullLegVerdict verdict = FullLegVerdict::NOT_RUN;
  FullLegFinalizeFailure failure = FullLegFinalizeFailure::NONE;
  FullLegCalibrationFailure executor_failure = FullLegCalibrationFailure::NONE;

  const FullLegJointRecord& joint(JointKind kind) const {
    return joints[static_cast<uint8_t>(kind)];
  }
  FullLegJointRecord& joint(JointKind kind) { return joints[static_cast<uint8_t>(kind)]; }
};

struct FullLegFinalizeContext {
  CalibrationManager* manager = nullptr;
  actuator::SafeActuatorPolicy* policy = nullptr;
  const actuator::CalibrationGeometryProfile* geometry = nullptr;
  const actuator::GeometryProvenance* expected_provenance = nullptr;
  CalibrationMotionPermit* permit = nullptr;
  CalibrationMotionAuthorizationState* authorization = nullptr;
  const core::ActuatorAuthorityArbiter* arbiter = nullptr;
  FullLegEnvelopeParameters parameters{};
};

// Runs steps 1-8 above. `*record` is rebuilt from scratch (attempts is
// carried over by FullLegEvidenceStore, not here) and always describes what
// happened, on success and on every refusal. Returns NONE only for the two
// success verdicts. Never touches the servos: SAFE_OFF was the executor's.
FullLegFinalizeFailure finalizeFullLeg(const FullLegFinalizeContext& context,
                                       const FullLegPlan& plan, const FullLegRunOutcome& outcome,
                                       FullLegRecord* record);

// ---------------------------------------------------------------------------
// The run in flight
// ---------------------------------------------------------------------------
//
// What the command handler hands to the Controller when it starts a run, so the
// Controller can finalize it when the executor turns terminal - including a run
// the operator aborted or whose session was lost. `armed` is set only by a
// start() that the executor accepted, and cleared only by finalization: while
// it is set, no other leg may start a session.
struct FullLegRunState {
  bool armed = false;
  FullLegPlan plan{};
  actuator::GeometryProvenanceTag geometry_at_start = actuator::kNoGeometryProvenance;
  uint32_t session_id_at_start = 0;

  void arm(const FullLegPlan& run_plan, actuator::GeometryProvenanceTag geometry,
           uint32_t session_id) {
    armed = true;
    plan = run_plan;
    geometry_at_start = geometry;
    session_id_at_start = session_id;
  }
  void clear() { *this = FullLegRunState(); }
};

// ---------------------------------------------------------------------------
// The RAM record of every leg run in this power-up
// ---------------------------------------------------------------------------
//
// Four slots, one per leg, no persistence: reset() is the only way to clear
// it and a reboot starts empty. A leg run again replaces its own slot (the
// newest attempt is the record; attempts counts them) and never touches
// another leg's, so one leg's failure cannot falsify another's result.
class FullLegEvidenceStore {
 public:
  void reset();

  // Stores the record for `record.leg`, incrementing that leg's attempt count.
  void put(const FullLegRecord& record);

  const FullLegRecord* find(Leg leg) const;

  uint8_t legsPresent() const;
  uint8_t legsContactCalibrated() const;
  uint8_t legsEnvelopeAccepted() const;
  // All four legs finished HARDWARE_CONTACT_CALIBRATED or better.
  bool allLegsContactCalibrated() const { return legsContactCalibrated() == kLegCount; }

 private:
  FullLegRecord records_[kLegCount];
};

// Emits the record as deterministic single-line key=value text, one call per
// line, in a fixed leg and joint order. `sink` receives a NUL-terminated line
// without a trailing newline. The same store always yields the same lines.
typedef void (*FullLegExportSink)(void* user, const char* line);

void exportFullLegEvidence(const FullLegEvidenceStore& store,
                           actuator::GeometryProvenanceTag current_geometry,
                           const FullLegEnvelopeParameters& parameters, FullLegExportSink sink,
                           void* user);

const char* toString(FullLegVerdict verdict);
const char* toString(FullLegLimitAdmission admission);
const char* toString(FullLegFinalizeFailure failure);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_FINALIZER_H
