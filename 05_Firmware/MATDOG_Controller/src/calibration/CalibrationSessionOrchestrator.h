#ifndef MATDOG_CALIBRATION_CALIBRATION_SESSION_ORCHESTRATOR_H
#define MATDOG_CALIBRATION_CALIBRATION_SESSION_ORCHESTRATOR_H

#include <stdint.h>

#include "CalibrationManager.h"
#include "CalibrationPopulationEvidence.h"
#include "CalibrationQ0CaptureSession.h"

// CR3 continuation, Objective A — the smallest coherent production path that
// starts a live CalibrationManager session using evidence the ALREADY
// reviewed CR2-B read-only capture flow produced, without duplicating
// population discovery.
//
// Pure: <stdint.h> plus already-pure MATDOG units. No Arduino, no ServoBus.
//
// scripts/static_audit.py's check_calibration_population_evidence() confines
// calibration::buildCurrentLegPopulationEvidence() to exactly one reviewed
// caller, CalibrationQ0CaptureSession.cpp. This file NEVER calls it and
// cannot produce population evidence of its own: it only reads the
// ALREADY-COMPUTED PopulationEvidenceBuildResult a completed
// Q0CaptureState::COMPLETE capture already produced, and fails closed if
// that capture is missing, incomplete, or did not reach a PASS verdict.
//
// NO PHYSICAL WRITE IS REACHABLE FROM STARTING A SESSION. startSession()
// only acquires ActuatorAuthority::CALIBRATION through the real arbiter (a
// RAM arbitration, exactly like every other authority grant in this
// codebase) - it has no path to ServoBus and commands nothing.

namespace matdog {
namespace calibration {

enum class SessionStartFromQ0Status : uint8_t {
  NOT_EVALUATED                  = 0,
  STARTED                        = 1,
  REJECT_Q0_CAPTURE_NOT_COMPLETE = 2,  // missing: no completed capture this boot
  REJECT_POPULATION_NOT_PASS     = 3,  // stale/mismatched/incomplete population
  REJECT_INVALID_LEG             = 4,
  REJECT_MANAGER                 = 5,  // CalibrationManager itself refused - see manager_result
};

struct SessionStartFromQ0Result {
  SessionStartFromQ0Status status = SessionStartFromQ0Status::NOT_EVALUATED;
  // Valid when status is STARTED or REJECT_MANAGER - which specific
  // CalibrationManager step (startSession/submitPopulationEvidence/
  // activate) produced it.
  SessionResult manager_result = SessionResult::OK;
};

// Drives CalibrationManager's own startSession()/submitPopulationEvidence()/
// activate() sequence exactly once, gated on the CR2-B capture's own
// COMPLETE state and a PASS population verdict measured under the SAME live
// session/origin the manager itself is about to open - fails closed on
// anything less. On any failure after startSession() has already acquired
// authority, aborts the session so a rejected attempt never leaves
// CALIBRATION authority held (CalibrationManager::abortSession() is
// idempotent when nothing is live, so calling it unconditionally on every
// failure path is always safe).
SessionStartFromQ0Result startCalibrationSessionFromQ0Evidence(
    CalibrationManager& manager, Q0CaptureState q0_capture_state,
    const PopulationEvidenceBuildResult& q0_population, Leg leg, core::OperatingMode mode);

const char* toString(SessionStartFromQ0Status status);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_SESSION_ORCHESTRATOR_H
