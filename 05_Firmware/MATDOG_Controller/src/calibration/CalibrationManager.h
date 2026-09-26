#ifndef MATDOG_CALIBRATION_CALIBRATION_MANAGER_H
#define MATDOG_CALIBRATION_CALIBRATION_MANAGER_H

#include <stdint.h>

#include "../core/ActuatorAuthority.h"
#include "CalibrationDomain.h"

// ---------------------------------------------------------------------------
// HARDWARE MOTION GATE — the repository's own refusal, as a compile-time fact
// ---------------------------------------------------------------------------
// 06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml, in the
// calibration_reset: block that is the CURRENT authority:
//
//     state: CALIBRATION_RESET_PENDING_FULL_RECALIBRATION
//     all_joint_data_below_is_stale: true
//     hardware_motion_authorized: false
//
// A live calibration session is therefore refused. Unblocking it requires a
// real recalibration and a reviewed change to that YAML, not a flag flip.
// scripts/static_audit.py FAILS the build if the source default here is
// anything but 0.
//
// NOTE, because the YAML itself warns about it: robot.calibration_status in
// that file still reads DIGITAL_ZERO_CALIBRATED_AND_VERIFIED. It was
// deliberately left stale because six tools hard-assert the exact string, and
// the file says to read calibration_reset: instead and treats those tools as
// unsafe against hardware. Reading the enum gives the opposite of the truth.
#ifndef MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED
#define MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED 0
#endif

namespace matdog {
namespace calibration {

// Pure, like the domain model it sits on. It owns SESSION LIFECYCLE and
// EVIDENCE. It owns no ServoBus, no UART, no Wi-Fi, no OTA, no scheduler and
// no global SAFE_OFF, and it cannot command a joint: there is no transport in
// it to command one with. scripts/static_audit.py fails the build if one
// appears.
//
//   CommandRouter
//        |
//        v
//   CalibrationManager            <- THIS: session lifecycle + evidence
//        |
//        v
//   ActuatorAuthority::CALIBRATION   (the real arbiter, never a second lock)
//        |
//        v
//   future calibration execution engine   [NOT IMPLEMENTED]
//        |
//        v
//   future Safe Actuator Layer -> ServoBus   [NOT CONNECTED]
//
// LAYERING NOTE: the 18 LF V25 states (UPPER_MIN, HIP_MAX, RETURN_HIP, ...)
// are HARDWARE EXECUTION phases and belong to that future engine. This manager
// does not drive them. It records which one a future execution layer last
// reported, and only so restore intent can be expressed against it.

enum class SessionState : uint8_t {
  NO_SESSION = 0,  // nothing running; no authority held
  PREFLIGHT  = 1,  // authority held, eligibility being established
  ACTIVE     = 2,  // eligibility satisfied; execution may proceed
  COMPLETED  = 3,  // clean end
  ABORTED    = 4,  // ended by the operator
  FAILED     = 5,  // ended by a failure
};

enum class SessionResult : uint8_t {
  OK                        = 0,
  STARTED                   = 1,
  REJECTED_BUSY             = 2,  // a session already exists
  REJECTED_NO_AUTHORITY     = 3,  // the arbiter refused CALIBRATION
  REJECTED_MOTION_BLOCKED   = 4,  // hardware_motion_authorized: false
  REJECTED_INVALID_LEG      = 5,
  REJECTED_NO_ARBITER       = 6,  // fail closed: nothing to ask
  REJECTED_WRONG_STATE      = 7,
  REJECTED_POPULATION_GATE  = 8,  // the leg population gate did not pass
};

struct CalibrationSessionStatus {
  SessionState state = SessionState::NO_SESSION;
  SessionResult last_result = SessionResult::OK;
  CalibrationFailure failure = CalibrationFailure::NONE;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  Leg leg = Leg::LF;

  bool holds_authority = false;
  uint32_t lease_generation = 0;

  // Eligibility, established during PREFLIGHT.
  LegPopulationEvidence population{};
  PopulationVerdict population_verdict = PopulationVerdict::NOT_EVALUATED;

  // Reported BY a future execution layer, never driven by this manager.
  CalibrationPhase last_reported_phase = CalibrationPhase::PREFLIGHT;
  bool execution_phase_reported = false;

  // Intent only. Nothing here moves a joint.
  RestorePlan restore{};

  uint16_t contacts_recorded = 0;
  uint32_t sessions_started = 0;
  uint32_t sessions_completed = 0;
  uint32_t sessions_aborted = 0;
  uint32_t sessions_failed = 0;

  // Mirrors the repository's refusal so a reader of @CALIBRATION STATUS sees
  // it without opening the YAML.
  bool hardware_motion_authorized = false;
  bool current_calibration_stale = true;
};

class CalibrationManager {
 public:
  static constexpr bool hardwareMotionAuthorized() {
    return MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED != 0;
  }

  // arbiter may be nullptr; that is a refusal, not a permission.
  void begin(core::ActuatorAuthorityArbiter* arbiter);

  // Acquires ActuatorAuthority::CALIBRATION and enters PREFLIGHT. A
  // LIVE_SESSION is refused while hardware motion is not authorized; a
  // HISTORICAL_REPLAY session commands nothing and is allowed, which is what
  // lets the offline oracle replay exercise this manager.
  SessionResult startSession(Leg leg, core::OperatingMode mode, CalibrationOrigin origin);

  // Eligibility input. The population evidence comes FROM servo/ServoCensus;
  // this manager never scans a bus.
  bool submitPopulationEvidence(const LegPopulationEvidence& evidence);

  // PREFLIGHT -> ACTIVE. A live session requires a current population PASS;
  // historical evidence can never supply one.
  SessionResult activate();

  // Reported by a future execution layer. Accepted only while ACTIVE, and only
  // as a legal step along the recovered order.
  bool noteExecutionPhase(CalibrationPhase phase);

  // Refuses evidence whose origin or leg disagrees with the session's, so a
  // replay cannot smuggle in a live record or the reverse.
  bool recordContact(const ContactEvidence& evidence);

  void abortSession();
  void failSession(CalibrationFailure cause);
  bool completeSession();

  // Every Controller tick. Detects authority lost from underneath the session
  // and fails closed.
  void update(core::OperatingMode mode);

  void reset();

  const CalibrationSessionStatus& status() const { return status_; }
  bool sessionLive() const {
    return status_.state == SessionState::PREFLIGHT || status_.state == SessionState::ACTIVE;
  }

 private:
  void releaseAuthority();
  void endSession(SessionState end_state, CalibrationFailure cause);
  bool leaseStillValid() const;

  core::ActuatorAuthorityArbiter* arbiter_ = nullptr;
  core::AuthorityLease lease_{};
  CalibrationSessionStatus status_{};
};

const char* toString(SessionState state);
const char* toString(SessionResult result);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_MANAGER_H
