#include "CalibrationManager.h"

namespace matdog {
namespace calibration {

void CalibrationManager::begin(core::ActuatorAuthorityArbiter* arbiter) {
  arbiter_ = arbiter;
  lease_ = core::AuthorityLease{};
  status_ = CalibrationSessionStatus{};
  status_.hardware_motion_authorized = hardwareMotionAuthorized();
  // The repository declares every joint value stale until a full
  // recalibration happens. The manager reports that rather than implying
  // otherwise by silence.
  status_.current_calibration_stale = true;
}

bool CalibrationManager::leaseStillValid() const {
  // Owner AND generation. Matching the owner alone would let this session keep
  // running on a lease that belongs to a later one.
  return arbiter_ != nullptr && lease_.valid() &&
         arbiter_->current() == core::ActuatorAuthority::CALIBRATION &&
         arbiter_->generation() == lease_.generation;
}

void CalibrationManager::releaseAuthority() {
  if (arbiter_ != nullptr && lease_.valid()) {
    // release() re-checks owner and generation itself, so a lease that has
    // already been superseded cannot clear a newer session's authority.
    arbiter_->release(lease_);
  }
  lease_ = core::AuthorityLease{};
  status_.holds_authority = false;
  status_.lease_generation = 0;
}

void CalibrationManager::endSession(SessionState end_state, CalibrationFailure cause) {
  status_.failure = cause;
  // Restore intent is computed from where execution actually was, using the
  // last phase a future execution layer reported. With nothing reported, there
  // is nothing engaged to restore.
  const CalibrationPhase phase = status_.execution_phase_reported
                                     ? status_.last_reported_phase
                                     : CalibrationPhase::TORQUE_OFF;
  status_.restore = restorePlanFor(phase, cause);
  status_.state = end_state;

  // Authority is released on EVERY terminal path, without exception. A
  // suspended CALIBRATION authority is the thing that must never exist.
  releaseAuthority();

  switch (end_state) {
    case SessionState::COMPLETED: status_.sessions_completed++; break;
    case SessionState::ABORTED:   status_.sessions_aborted++; break;
    case SessionState::FAILED:    status_.sessions_failed++; break;
    default: break;
  }
}

SessionResult CalibrationManager::startSession(Leg leg, core::OperatingMode mode,
                                               CalibrationOrigin origin) {
  if (arbiter_ == nullptr) {
    status_.last_result = SessionResult::REJECTED_NO_ARBITER;
    return status_.last_result;
  }
  if (sessionLive()) {
    status_.last_result = SessionResult::REJECTED_BUSY;
    return status_.last_result;
  }
  if (!isKnownLeg(leg) || origin == CalibrationOrigin::NONE) {
    status_.last_result = SessionResult::REJECTED_INVALID_LEG;
    return status_.last_result;
  }

  // The repository's own refusal, enforced here. A replay commands nothing and
  // is allowed; anything that would touch the robot is not.
  if (origin == CalibrationOrigin::LIVE_SESSION && !hardwareMotionAuthorized()) {
    status_.last_result = SessionResult::REJECTED_MOTION_BLOCKED;
    status_.failure = CalibrationFailure::STALE_CALIBRATION_REFUSED;
    return status_.last_result;
  }

  // The REAL arbiter. No second lock is created. Mode compatibility is the
  // arbiter's to judge, not this manager's to second-guess.
  core::AuthorityLease lease;
  const core::AuthorityResult granted =
      arbiter_->request(core::ActuatorAuthority::CALIBRATION, mode, &lease);
  if (granted != core::AuthorityResult::GRANTED || !lease.valid()) {
    status_.last_result = SessionResult::REJECTED_NO_AUTHORITY;
    return status_.last_result;
  }

  lease_ = lease;
  status_.state = SessionState::PREFLIGHT;
  status_.failure = CalibrationFailure::NONE;
  status_.origin = origin;
  status_.leg = leg;
  status_.holds_authority = true;
  status_.lease_generation = lease.generation;
  status_.restore = RestorePlan{};
  status_.population = LegPopulationEvidence{};
  status_.population_verdict = PopulationVerdict::NOT_EVALUATED;
  status_.last_reported_phase = CalibrationPhase::PREFLIGHT;
  status_.execution_phase_reported = false;
  status_.contacts_recorded = 0;
  status_.sessions_started++;
  status_.last_result = SessionResult::STARTED;
  return status_.last_result;
}

bool CalibrationManager::submitPopulationEvidence(const LegPopulationEvidence& evidence) {
  if (status_.state != SessionState::PREFLIGHT) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return false;
  }
  // Evidence provenance must match the session's. A replay may not be admitted
  // as live population evidence.
  if (evidence.origin != status_.origin) {
    status_.last_result = SessionResult::REJECTED_POPULATION_GATE;
    return false;
  }
  status_.population = evidence;
  status_.population_verdict = evaluateLegPopulation(evidence);
  status_.last_result = SessionResult::OK;
  return status_.population_verdict != PopulationVerdict::INVALID;
}

SessionResult CalibrationManager::activate() {
  if (status_.state != SessionState::PREFLIGHT) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return status_.last_result;
  }
  if (!leaseStillValid()) {
    endSession(SessionState::FAILED, CalibrationFailure::AUTHORITY_LOST);
    status_.last_result = SessionResult::REJECTED_NO_AUTHORITY;
    return status_.last_result;
  }

  // A live session needs a CURRENT population pass. The last formal Full-Leg
  // result on record is 6/12 and it is historical; historical evidence can
  // never satisfy this, by construction.
  if (status_.origin == CalibrationOrigin::LIVE_SESSION &&
      !populationIsCurrentPass(status_.population)) {
    status_.last_result = SessionResult::REJECTED_POPULATION_GATE;
    return status_.last_result;
  }

  status_.state = SessionState::ACTIVE;
  status_.last_result = SessionResult::OK;
  return status_.last_result;
}

bool CalibrationManager::noteExecutionPhase(CalibrationPhase phase) {
  if (status_.state != SessionState::ACTIVE) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return false;
  }
  // The first report may be any phase at or after PREFLIGHT; later ones must
  // be legal steps. This manager validates the report; it does not produce it.
  if (status_.execution_phase_reported &&
      !isLegalPhaseTransition(status_.last_reported_phase, phase)) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return false;
  }
  if (!status_.execution_phase_reported &&
      !isLegalPhaseTransition(CalibrationPhase::PREFLIGHT, phase) &&
      phase != CalibrationPhase::PREFLIGHT) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return false;
  }
  status_.last_reported_phase = phase;
  status_.execution_phase_reported = true;
  status_.last_result = SessionResult::OK;
  return true;
}

bool CalibrationManager::recordContact(const ContactEvidence& evidence) {
  if (status_.state != SessionState::ACTIVE) return false;
  if (!evidence.key.valid()) return false;
  // Evidence belongs to the leg under calibration...
  if (evidence.key.leg != status_.leg) return false;
  // ...and to the session's provenance. Mixing origins is how a replayed
  // record would end up looking like a live measurement.
  if (evidence.origin != status_.origin) return false;
  // Only a confirmed contact inside an explicitly-banded witness is evidence.
  if (!isContactEvidence(evidence.detection)) return false;
  if (!evidence.witness.accepted()) return false;

  status_.contacts_recorded++;
  return true;
}

void CalibrationManager::abortSession() {
  // Idempotent: aborting when nothing is live is a no-op, not an error that
  // could disturb whatever state the manager is in now.
  if (!sessionLive()) return;
  endSession(SessionState::ABORTED, CalibrationFailure::OPERATOR_ABORT);
}

void CalibrationManager::failSession(CalibrationFailure cause) {
  if (!sessionLive()) return;
  endSession(SessionState::FAILED,
             cause == CalibrationFailure::NONE ? CalibrationFailure::OPERATOR_ABORT : cause);
}

bool CalibrationManager::completeSession() {
  // Only an ACTIVE session can complete, and only once execution has reported
  // that it reached the end of the sequence. A session cannot declare success
  // from PREFLIGHT or from the middle of a measurement.
  if (status_.state != SessionState::ACTIVE) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return false;
  }
  if (!status_.execution_phase_reported ||
      (status_.last_reported_phase != CalibrationPhase::CLEANUP &&
       status_.last_reported_phase != CalibrationPhase::TORQUE_OFF)) {
    status_.last_result = SessionResult::REJECTED_WRONG_STATE;
    return false;
  }
  endSession(SessionState::COMPLETED, CalibrationFailure::NONE);
  status_.last_result = SessionResult::OK;
  return true;
}

void CalibrationManager::update(core::OperatingMode mode) {
  if (!sessionLive()) return;

  // Authority can disappear from under a session: a fatal-fault force clear,
  // an operating-mode change that strands CALIBRATION, an operator reset. The
  // session must notice and fail closed rather than keep believing it owns the
  // actuators.
  if (!leaseStillValid()) {
    // It is already gone - do not try to release it, and in particular do not
    // release whatever holds it now.
    lease_ = core::AuthorityLease{};
    status_.holds_authority = false;
    status_.lease_generation = 0;
    endSession(SessionState::FAILED, CalibrationFailure::AUTHORITY_LOST);
    return;
  }

  // The mode may have moved to one CALIBRATION cannot live in without the
  // arbiter having been told yet.
  if (!core::isModeCompatible(mode, core::ActuatorAuthority::CALIBRATION)) {
    endSession(SessionState::FAILED, CalibrationFailure::AUTHORITY_LOST);
  }
}

void CalibrationManager::reset() {
  if (sessionLive()) {
    endSession(SessionState::ABORTED, CalibrationFailure::OPERATOR_ABORT);
  }
  releaseAuthority();

  const CalibrationSessionStatus previous = status_;
  status_ = CalibrationSessionStatus{};
  status_.hardware_motion_authorized = hardwareMotionAuthorized();
  status_.current_calibration_stale = true;
  // Counters describe the boot, not the session, so they survive a reset.
  status_.sessions_started = previous.sessions_started;
  status_.sessions_completed = previous.sessions_completed;
  status_.sessions_aborted = previous.sessions_aborted;
  status_.sessions_failed = previous.sessions_failed;
}

const char* toString(SessionState state) {
  switch (state) {
    case SessionState::NO_SESSION: return "NO_SESSION";
    case SessionState::PREFLIGHT:  return "PREFLIGHT";
    case SessionState::ACTIVE:     return "ACTIVE";
    case SessionState::COMPLETED:  return "COMPLETED";
    case SessionState::ABORTED:    return "ABORTED";
    case SessionState::FAILED:     return "FAILED";
  }
  return "UNKNOWN";
}

const char* toString(SessionResult result) {
  switch (result) {
    case SessionResult::OK:                       return "OK";
    case SessionResult::STARTED:                  return "STARTED";
    case SessionResult::REJECTED_BUSY:            return "REJECTED_BUSY";
    case SessionResult::REJECTED_NO_AUTHORITY:    return "REJECTED_NO_AUTHORITY";
    case SessionResult::REJECTED_MOTION_BLOCKED:  return "REJECTED_MOTION_BLOCKED";
    case SessionResult::REJECTED_INVALID_LEG:     return "REJECTED_INVALID_LEG";
    case SessionResult::REJECTED_NO_ARBITER:      return "REJECTED_NO_ARBITER";
    case SessionResult::REJECTED_WRONG_STATE:     return "REJECTED_WRONG_STATE";
    case SessionResult::REJECTED_POPULATION_GATE: return "REJECTED_POPULATION_GATE";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
