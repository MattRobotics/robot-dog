#include "RestoreAttestationPolicy.h"

namespace matdog {
namespace calibration {

RestoreVerdict evaluateRestore(const RestoreFacts& f, const InstallationAttestation& a,
                               const RestoreRequestFacts& q) {
  RestoreVerdict v;
  auto refuse = [&](RestoreReason reason) { v.decision = RestoreDecision::REFUSED; v.reason = reason; return v; };
  auto pending = [&](RestoreReason reason) { v.decision = RestoreDecision::PENDING_OPERATOR_REVIEW; v.reason = reason; return v; };
  const bool manual = q.trigger == RestoreTrigger::OPERATOR_MANUAL;

  // Safety context first: nothing below may be evaluated as "ready" while
  // the runtime could be moving or the mode is not MAINTENANCE.
  if (!f.maintenance) return refuse(RestoreReason::NOT_MAINTENANCE);
  if (!f.runtime_idle) return refuse(RestoreReason::RUNTIME_NOT_IDLE);
  if (!f.nvs_ready) return refuse(RestoreReason::NVS_NOT_READY);
  if (f.write_outcome_uncertain) return refuse(RestoreReason::WRITE_OUTCOME_UNCERTAIN);
  if (!f.record_valid_acknowledged || f.record_generation == 0)
    return refuse(RestoreReason::NO_VALID_ACKNOWLEDGED_RECORD);
  if (!f.record_inspection_ok) return refuse(RestoreReason::RECORD_REJECTED);
  if (manual && q.requested_generation != f.record_generation) return refuse(RestoreReason::GENERATION_MISMATCH);

  if (f.already_restored && f.restored_generation == f.record_generation) {
    v.decision = RestoreDecision::ALREADY_RESTORED; v.reason = RestoreReason::OK; return v;
  }

  // Boot: without a pinned, matching, enabled attestation there is nothing
  // to attempt; the read-only preflight is not even started for it.
  if (!manual) {
    if (!a.present) return pending(RestoreReason::INSTALLATION_ATTESTATION_ABSENT);
    if (a.generation != f.record_generation || a.record_fingerprint != f.record_fingerprint)
      return pending(RestoreReason::INSTALLATION_ATTESTATION_MISMATCH);
    if (!a.auto_restore_enabled) return pending(RestoreReason::AUTO_RESTORE_DISABLED);
  }

  // Live read-only evidence: required for both triggers, never sufficient.
  if (!f.preflight_complete) return manual ? refuse(RestoreReason::PREFLIGHT_NOT_COMPLETE)
                                           : pending(RestoreReason::PREFLIGHT_NOT_COMPLETE);
  if (f.preflight_pass != 12) return refuse(RestoreReason::PREFLIGHT_FAILED);
  if (f.torque_off != 12) return refuse(RestoreReason::TORQUE_NOT_OFF);
  if (!f.live_positions_inside_record) {
    v.failed_joint = f.live_position_failed_joint;
    return refuse(RestoreReason::LIVE_POSITION_OUTSIDE_RECORD);
  }

  if (manual) {
    if (!q.operator_confirmed_installation) return refuse(RestoreReason::OPERATOR_CONFIRMATION_REQUIRED);
    v.decision = RestoreDecision::ALLOW_OPERATOR; v.reason = RestoreReason::OK; return v;
  }
  v.decision = RestoreDecision::ALLOW_AUTOMATIC; v.reason = RestoreReason::OK; return v;
}

const char* toString(RestoreDecision d) {
  switch (d) {
    case RestoreDecision::ALLOW_AUTOMATIC: return "ALLOW_AUTOMATIC";
    case RestoreDecision::ALLOW_OPERATOR: return "ALLOW_OPERATOR";
    case RestoreDecision::ALREADY_RESTORED: return "ALREADY_RESTORED";
    case RestoreDecision::PENDING_OPERATOR_REVIEW: return "PENDING_OPERATOR_REVIEW";
    case RestoreDecision::REFUSED: return "REFUSED";
  }
  return "UNKNOWN";
}

const char* toString(RestoreReason r) {
  switch (r) {
#define CASE(s) case RestoreReason::s: return #s
    CASE(OK); CASE(NOT_MAINTENANCE); CASE(RUNTIME_NOT_IDLE); CASE(NVS_NOT_READY);
    CASE(NO_VALID_ACKNOWLEDGED_RECORD); CASE(WRITE_OUTCOME_UNCERTAIN); CASE(RECORD_REJECTED);
    CASE(GENERATION_MISMATCH); CASE(PREFLIGHT_NOT_COMPLETE); CASE(PREFLIGHT_FAILED);
    CASE(TORQUE_NOT_OFF); CASE(LIVE_POSITION_OUTSIDE_RECORD); CASE(AUTO_RESTORE_DISABLED);
    CASE(INSTALLATION_ATTESTATION_ABSENT); CASE(INSTALLATION_ATTESTATION_MISMATCH);
    CASE(OPERATOR_CONFIRMATION_REQUIRED);
#undef CASE
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
