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
// FULL CALIBRATION HAS ONE DEFINITION: 4 legs x 3 joints x MIN/MAX = 24
// mechanical contact witnesses. A leg is HARDWARE_CONTACT_CALIBRATED only with
// all SIX of its contacts - UPPER, LOWER and HIP, each MIN and MAX - accepted.
// Two (the old UPPER-only definition) or five are a FAILED leg, never a
// partial success, and the four-leg result needs 24/24.
//
// The executor proves six contacts and leaves every servo SAFE_OFF. That is a
// measurement, not a calibration: finalizeFullLeg() closes it, once, in one
// fixed order:
//
//   1  the run is terminal; a FAILED run only cleans up
//   2  the live session is the one the run started under (leg AND session id)
//   3  6/6 contacts measured, each for this leg's exact (joint, side), and the
//      executor's V25 diagnostics accepted
//   4  recordContact() for all six, each exactly once
//   5  UPPER / LOWER / HIP envelopes, each from its own two contacts
//   6  limits: admitted ONLY under approved parameters (see below)
//   7  noteExecutionPhase(TORQUE_OFF), then completeSession()
//   8  permit + operator authorization revoked
//   9  post-conditions verified: session terminal, authority NONE, permit off
//
// A refusal at any step ends the session as FAILED, revokes the permit, and
// records why. Nothing after the refusing step runs.
//
// TWO LEVELS, NEVER CONFLATED
// ---------------------------
//   HARDWARE_CONTACT_CALIBRATED
//       All six contacts witnessed and recorded exactly once, diagnostics
//       accepted, the three contact envelopes READY, every servo SAFE_OFF,
//       the session completed cleanly.
//   FINAL_OPERATIONAL_ENVELOPE_ACCEPTED
//       Additionally, the three envelopes were built with APPROVED parameters
//       and all three JointLimits were admitted and read back.
//
// The second level needs a reviewed stand/gait margin. The repository has
// none; the margin below is a placeholder and productionEnvelopeParameters()
// marks it unapproved. An unapproved run builds and REPORTS its envelopes but
// never offers them to SafeActuatorPolicy. Approving is a deliberate, reviewed
// source change to kFullLegOperationalParametersApproved, which
// static_audit.py pins to false.
//
// Pure: no Arduino, no ServoBus, no time.

namespace matdog {
namespace calibration {

constexpr bool kFullLegOperationalParametersApproved = false;
// Placeholder inset applied to both confirmed contact ticks of every joint.
constexpr uint16_t kFullLegPlaceholderContactMarginTicks = 8;
// The acceptance arithmetic of the 24-contact definition.
constexpr uint8_t kFullLegContactsExpected = kFullLegContactCount;  // 6 per leg
constexpr uint8_t kFullCalibrationContactsExpected =
    static_cast<uint8_t>(kLegCount * kFullLegContactsExpected);   // 24

struct FullLegEnvelopeParameters {
  bool approved = false;
  uint16_t contact_margin_ticks = 0;
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
  CONTACTS_INCOMPLETE = 7,        // fewer than 6/6 contacts: never a leg success
  DIAGNOSTICS_REJECTED = 8,
  CONTACT_REJECTED = 9,           // the session refused to record a contact
  ENVELOPE_NOT_READY = 10,
  LIMIT_ADMISSION_REJECTED = 11,
  LIMIT_VERIFY_FAILED = 12,
  PHASE_REPORT_REJECTED = 13,
  SESSION_COMPLETE_REJECTED = 14,
  CLEANUP_SESSION_NOT_TERMINAL = 15,
  CLEANUP_AUTHORITY_HELD = 16,
  CLEANUP_PERMIT_ACTIVE = 17,
};

// What the finalizer needs to know about a finished run. Snapshotted by the
// Controller from the executor at the moment it turns terminal.
struct FullLegRunOutcome {
  bool terminal = false;
  bool complete = false;  // executor COMPLETE: 6/6, diagnostics, verified rest
  FullLegFailure failure = FullLegFailure::NONE;
  CalibrationPhase failed_phase = CalibrationPhase::PREFLIGHT;
  uint8_t contacts_measured = 0;
  ContactEvidence contacts[kJointKindCount][kContactSideCount];
  FullLegJointDiagnostics diagnostics[kJointKindCount];
  bool diagnostics_accepted = false;
  // The geometry the run STARTED under and the session it ran in.
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

  // [side]: MIN, MAX.
  ContactEvidence contact[kContactSideCount];
  bool contact_recorded[kContactSideCount] = {false, false};
  FullLegJointDiagnostics diagnostics{};

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

  bool has_rear_park = false;
  Leg park_leg = Leg::LF;
  JointKind park_joint = JointKind::UPPER;
  uint8_t park_bus_id = 0;
  actuator::MicroRad park_target_urad = 0;

  uint8_t contacts_expected = kFullLegContactsExpected;
  uint8_t contacts_measured = 0;  // probes that completed
  uint8_t contacts_accepted = 0;  // recorded in the session - 6 or the leg failed
  bool diagnostics_accepted = false;

  bool parameters_approved = false;
  uint16_t contact_margin_ticks = 0;

  bool session_completed = false;
  bool permit_revoked = false;
  bool authority_released = false;

  bool hardware_contact_calibrated = false;
  bool operational_envelope_accepted = false;
  FullLegVerdict verdict = FullLegVerdict::NOT_RUN;
  FullLegFinalizeFailure failure = FullLegFinalizeFailure::NONE;
  FullLegFailure executor_failure = FullLegFailure::NONE;
  CalibrationPhase executor_failed_phase = CalibrationPhase::PREFLIGHT;

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

// Runs steps 1-9 above. `*record` is rebuilt from scratch (attempts is
// carried over by FullLegEvidenceStore, not here) and always describes what
// happened, on success and on every refusal. Returns NONE only for the two
// success verdicts. Never touches the servos: SAFE_OFF was the executor's.
FullLegFinalizeFailure finalizeFullLeg(const FullLegFinalizeContext& context,
                                       const FullLegPlan& plan, const FullLegRunOutcome& outcome,
                                       FullLegRecord* record);

// ---------------------------------------------------------------------------
// The run in flight
// ---------------------------------------------------------------------------
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
class FullLegEvidenceStore {
 public:
  void reset();
  void put(const FullLegRecord& record);
  const FullLegRecord* find(Leg leg) const;

  uint8_t legsPresent() const;
  uint8_t legsContactCalibrated() const;
  uint8_t legsEnvelopeAccepted() const;
  // Contacts accepted across all four legs; the definition's total is 24.
  uint8_t totalContactsAccepted() const;
  // All four legs HARDWARE_CONTACT_CALIBRATED or better AND 24/24 contacts.
  bool allLegsContactCalibrated() const {
    return legsContactCalibrated() == kLegCount &&
           totalContactsAccepted() == kFullCalibrationContactsExpected;
  }

 private:
  FullLegRecord records_[kLegCount];
};

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
