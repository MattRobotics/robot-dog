#ifndef MATDOG_CALIBRATION_FIRST_MOTION_EXECUTOR_H
#define MATDOG_CALIBRATION_FIRST_MOTION_EXECUTOR_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../actuator/MotionDeadman.h"
#include "CalibrationDomain.h"

// CR3 Priority 3 — the first controlled bounded-motion path, prepared but
// never executed against real hardware by this offline session.
//
// Pure: <stdint.h> plus the already-pure MATDOG units it composes. No
// Arduino, no ServoBus, no Serial, no clock of its own. Every tick's "now"
// and every telemetry sample are supplied by the caller — the same contract
// as MotionDeadman.h and every other decision core in this codebase, and
// what makes this host-testable against a fake backend/fake clock instead of
// real hardware.
//
// THE CHAIN THIS DRIVES (CR3 brief, Priority 3), each already-reviewed step
// reused rather than re-implemented:
//
//   explicit operator authorization -> CalibrationMotionPermit ACTIVE
//        (verified live, every tick, by SafeActuatorPolicy's bootstrap
//        context — this class re-checks nothing here; it just supplies the
//        same lease/mode the caller already holds to plan()/commit())
//   -> valid CALIBRATION authority              (SafeActuatorPolicy::evaluate())
//   -> TorqueEnable                              (SafeActuatorPolicy + ActuatorRuntime;
//                                                  TORQUE_ENABLE has no
//                                                  CalibrationIntent, so this
//                                                  one step goes directly
//                                                  through plan()/commit(),
//                                                  not CalibrationExecutionEngine)
//   -> bounded target resolution                 (actuator::resolveDeltaFromQ0 —
//                                                  the SAME checked resolver
//                                                  CalibrationExecutionEngine
//                                                  uses internally, called
//                                                  here too only so the
//                                                  resolved raw tick is
//                                                  available for deadman
//                                                  arrival detection; CR3-M2
//                                                  is not re-implemented)
//   -> slow single-joint motion                  (CalibrationExecutionEngine,
//                                                  DIRECTION_VERIFY intent —
//                                                  see the file comment below
//                                                  for why this is the chosen
//                                                  primitive)
//   -> telemetry monitoring / timeout / deadman  (actuator::MotionDeadmanMonitor)
//   -> SAFE_OFF / controlled termination         (NEVER called from here —
//                                                  see the note below)
//
// WHY DIRECTION_VERIFY
// ---------------------
// It is the one operation the existing architecture already designed for
// exactly this purpose: "does this joint move, and does it move the way the
// contract says" (ActuatorWritePolicy.h). It needs no endpoint plan, no
// parking, no contact witness — only a live session, a promoted q0
// transform and an operator/geometry-agreed tick budget — so it is the
// smallest reachable proof-of-life, not a shortcut around a stronger
// prerequisite. It stays what it always was: OPTIONAL, never a Full
// Calibration or acceptance prerequisite (ActuatorWritePolicy.h,
// check_direction_is_contractual()) — this executor does not change that.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY
// ----------------------------------------------
// This class never names ServoBus::safeOff() and cannot: it has no ServoBus
// reference, the same structural absence ActuatorWritePolicy.h,
// ActuatorRuntime.h and CalibrationExecutionEngine.h already document for
// themselves. Instead, every terminal state that leaves torque possibly
// applied is reported as FirstMotionState::SAFE_OFF_REQUIRED — the caller,
// which DOES own the real ServoBus, is responsible for invoking the real,
// ungated safeOff() immediately on seeing it. See FirstMotionState below for
// exactly which terminal states those are and why.

namespace matdog {
namespace calibration {

struct FirstMotionRequest {
  JointIdentity joint{};
  uint8_t bus_id = 0;
  // Signed raw-tick excursion from the captured q0 tick, already bounded by
  // the operator/geometry envelope this class does not itself invent — see
  // SafeActuatorPolicy::evaluateBootstrapEnvelope() and
  // CalibrationBootstrapContext::direction_verify_tick_budget.
  int32_t delta_ticks = 0;
};

enum class FirstMotionState : uint8_t {
  IDLE                     = 0,  // not started, or a previous attempt's
                                  // terminal state was already read
  TORQUE_ENABLE_PENDING    = 1,  // about to attempt TorqueEnable this tick
  POSITION_COMMAND_PENDING = 2,  // torque VERIFIED applied; about to attempt
                                  // the bounded GoalPosition write this tick
  MONITORING               = 3,  // position commanded; watching telemetry
  // --- terminal states --------------------------------------------------
  COMPLETE                 = 4,  // arrived within tolerance; torque remains
                                  // applied, holding the new position — this
                                  // class does not decide what happens next
  FAILED_NO_MOTION         = 5,  // ended before torque was ever VERIFIED
                                  // applied; no SAFE_OFF is required by this
                                  // attempt (though it is never wrong to send
                                  // one as routine session hygiene)
  SAFE_OFF_REQUIRED        = 6,  // torque was VERIFIED applied at some point
                                  // in this attempt and the attempt did not
                                  // reach a clean COMPLETE — the caller MUST
                                  // invoke the real, independent SAFE_OFF now
};

enum class FirstMotionFailure : uint8_t {
  NONE                       = 0,
  REJECT_PRECONDITIONS       = 1,  // the TorqueEnable plan() itself refused
                                    // (permit/authority/mode/session/etc.)
  TORQUE_ENABLE_REJECTED     = 2,  // backend VERIFIED torque did NOT apply
  TORQUE_ENABLE_UNCERTAIN    = 3,  // backend could not verify either way
  REJECT_TARGET_RESOLUTION   = 4,  // the checked q<->raw resolver refused
  // Policy refused the position command itself (e.g.
  // REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE, REJECT_NO_CALIBRATION_MOTION_PERMIT -
  // see status().last_policy_decision for exactly which), or the backend
  // VERIFIED the write did NOT apply — either way, after torque was already
  // verified on.
  POSITION_COMMAND_REJECTED  = 5,
  POSITION_COMMAND_UNCERTAIN = 6,  // backend could not verify either way
  STALE_TELEMETRY            = 7,
  COMMUNICATION_LOST         = 8,
  TORQUE_UNEXPECTEDLY_OFF    = 9,
  STALLED                    = 10,
  MOTION_TIMEOUT              = 11,
  OPERATOR_ABORT              = 12,
  DYNAMIC_PREREQUISITE_LOST   = 13,
};

struct FirstMotionConfig {
  actuator::MotionDeadmanConfig deadman{};
};

struct FirstMotionStatus {
  FirstMotionState state = FirstMotionState::IDLE;
  FirstMotionFailure failure = FirstMotionFailure::NONE;
  // The absolute raw target this attempt resolved to, once resolution
  // succeeds — 0..4095, meaningful only once past TORQUE_ENABLE_PENDING.
  uint16_t target_tick = 0;
  actuator::WriteDecision last_policy_decision = actuator::WriteDecision::REJECT_NO_ARBITER;
};

// Everything this call is allowed to look at — a snapshot the caller already
// holds (the real arbiter's current lease, the real operating mode). Never
// cached beyond one start(); every subsequent tick re-supplies it via
// update(), exactly like CalibrationExecutionEngine's own context.
struct FirstMotionContext {
  bool session_active = false;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;

  // Dynamic continuation gates, supplied fresh by Controller every tick.
  // These are intentionally not cached by the executor. In particular,
  // MONITORING must not continue merely because the position command was
  // valid when it was issued: permit/session/authority may disappear while
  // the servo is still moving.
  bool motion_permit_active = false;
  core::ActuatorAuthority authority = core::ActuatorAuthority::NONE;
  uint32_t authority_generation = 0;
  bool authority_inhibited = false;
};

class FirstMotionExecutor {
 public:
  // No pointer is owned; all may be nullptr, which is a refusal (start()
  // fails closed), never a permission.
  void begin(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
            const actuator::CalibrationGeometryProfile* geometry,
            const actuator::GeometryProvenance* expected_provenance,
            const FirstMotionConfig& config);

  // Starts a new attempt. Refuses (returns false, no state change) if one is
  // already in progress (anything other than IDLE/a terminal state) or if
  // the request/context is not even well-formed enough to plan against —
  // this performs NO backend call itself; the first update() tick does.
  bool start(const FirstMotionRequest& request, const FirstMotionContext& context,
            uint32_t now_ms);

  // Advances the state machine by AT MOST one backend call per tick —
  // exactly the same "bounded per-tick blocking" discipline
  // Controller::updateQ0Capture() already follows. `telemetry_available`
  // distinguishes "polled and this is the result" from "no attempt this
  // tick" (see MotionDeadmanMonitor::poll() vs evaluate()) — this class
  // polls nothing itself; the caller supplies both.
  void update(const FirstMotionContext& context, uint32_t now_ms,
             bool telemetry_available, const actuator::TelemetrySample& telemetry);

  // Operator-requested abort at any point after start(). Safe by
  // construction: routes to SAFE_OFF_REQUIRED if and only if torque was
  // already verified applied in this attempt.
  void abort();

  const FirstMotionStatus& status() const { return status_; }
  // The bus id the CURRENT (or most recently started) attempt targets -
  // valid from the moment start() returns true, including after a terminal
  // state, so a caller can still address SAFE_OFF correctly once active()
  // has become false. 0 (never a valid ST3215 id) before the first start().
  uint8_t busId() const { return bus_id_; }
  bool active() const {
    return status_.state != FirstMotionState::IDLE &&
          status_.state != FirstMotionState::COMPLETE &&
          status_.state != FirstMotionState::FAILED_NO_MOTION &&
          status_.state != FirstMotionState::SAFE_OFF_REQUIRED;
  }

 private:
  void finish(FirstMotionState state, FirstMotionFailure failure, actuator::WriteDecision decision);
  void stepTorqueEnable(const FirstMotionContext& context, uint32_t now_ms);
  void stepPositionCommand(const FirstMotionContext& context, uint32_t now_ms);
  void stepMonitoring(uint32_t now_ms, bool telemetry_available,
                     const actuator::TelemetrySample& telemetry);

  actuator::SafeActuatorPolicy* policy_ = nullptr;
  actuator::ActuatorRuntime* runtime_ = nullptr;
  const actuator::CalibrationGeometryProfile* geometry_ = nullptr;
  const actuator::GeometryProvenance* expected_provenance_ = nullptr;
  FirstMotionConfig config_{};

  FirstMotionRequest request_{};
  uint8_t bus_id_ = 0;
  FirstMotionStatus status_{};
  actuator::MotionDeadmanMonitor deadman_;
};

const char* toString(FirstMotionState state);
const char* toString(FirstMotionFailure failure);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_FIRST_MOTION_EXECUTOR_H
