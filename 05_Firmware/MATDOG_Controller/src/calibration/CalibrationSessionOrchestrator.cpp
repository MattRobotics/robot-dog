#include "CalibrationSessionOrchestrator.h"

namespace matdog {
namespace calibration {

SessionStartFromQ0Result startCalibrationSessionFromQ0Evidence(
    CalibrationManager& manager, Q0CaptureState q0_capture_state,
    const PopulationEvidenceBuildResult& q0_population, Leg leg, core::OperatingMode mode) {
  SessionStartFromQ0Result out{};

  // MISSING: no completed capture exists to reuse.
  if (q0_capture_state != Q0CaptureState::COMPLETE) {
    out.status = SessionStartFromQ0Status::REJECT_Q0_CAPTURE_NOT_COMPLETE;
    return out;
  }
  // STALE/MISMATCHED: the capture completed, but its own population gate
  // did not pass, or the evidence does not carry live-session provenance -
  // populationIsCurrentPass() is the SAME current-pass predicate
  // CalibrationManager::activate() itself re-checks below, applied here
  // first so the specific rejection reason is distinguishable from a
  // generic manager refusal.
  if (q0_population.status != PopulationEvidenceBuildStatus::PASS ||
      q0_population.evidence.origin != CalibrationOrigin::LIVE_SESSION ||
      !populationIsCurrentPass(q0_population.evidence)) {
    out.status = SessionStartFromQ0Status::REJECT_POPULATION_NOT_PASS;
    return out;
  }
  if (!isKnownLeg(leg)) {
    out.status = SessionStartFromQ0Status::REJECT_INVALID_LEG;
    return out;
  }

  const SessionResult started = manager.startSession(leg, mode, CalibrationOrigin::LIVE_SESSION);
  if (started != SessionResult::STARTED) {
    out.status = SessionStartFromQ0Status::REJECT_MANAGER;
    out.manager_result = started;
    return out;
  }

  if (!manager.submitPopulationEvidence(q0_population.evidence)) {
    out.status = SessionStartFromQ0Status::REJECT_MANAGER;
    out.manager_result = manager.status().last_result;
    manager.abortSession();  // never leave a PREFLIGHT session holding authority on refusal
    return out;
  }

  const SessionResult activated = manager.activate();
  if (activated != SessionResult::OK) {
    out.status = SessionStartFromQ0Status::REJECT_MANAGER;
    out.manager_result = activated;
    manager.abortSession();  // idempotent if activate() already ended the session itself
    return out;
  }

  out.status = SessionStartFromQ0Status::STARTED;
  out.manager_result = SessionResult::OK;
  return out;
}

const char* toString(SessionStartFromQ0Status status) {
  switch (status) {
    case SessionStartFromQ0Status::NOT_EVALUATED: return "NOT_EVALUATED";
    case SessionStartFromQ0Status::STARTED:       return "STARTED";
    case SessionStartFromQ0Status::REJECT_Q0_CAPTURE_NOT_COMPLETE:
      return "REJECT_Q0_CAPTURE_NOT_COMPLETE";
    case SessionStartFromQ0Status::REJECT_POPULATION_NOT_PASS:
      return "REJECT_POPULATION_NOT_PASS";
    case SessionStartFromQ0Status::REJECT_INVALID_LEG: return "REJECT_INVALID_LEG";
    case SessionStartFromQ0Status::REJECT_MANAGER:     return "REJECT_MANAGER";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
