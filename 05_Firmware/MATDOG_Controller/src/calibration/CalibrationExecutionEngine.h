#ifndef MATDOG_CALIBRATION_CALIBRATION_EXECUTION_ENGINE_H
#define MATDOG_CALIBRATION_CALIBRATION_EXECUTION_ENGINE_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "CalibrationDomain.h"

// The Calibration Execution boundary — I5, V3 handoff §13/§15.11.
//
// BINDING CONTRACT (V3 §15.11), AS AMENDED 2026-09-30: V3 made LF V25's
// 18-phase sequence (calibration::CalibrationPhase, CalibrationDomain.h) a
// historical oracle only. The operator's 2026-09-30 scope correction (TRUE
// Full Calibration = 24 contacts) superseded that: the V25 full-leg state
// machine, generalized to all four legs, IS the production sequence now -
// owned by FullLegCalibrationExecutor, not by this class. This engine still
// sequences nothing and decides nothing per phase: it carries the
// executor's current phase as an opaque label (sequence_phase) to the
// policy, which checks it against its own bootstrap copy. static_audit.py
// pins that the label is only ever declared and forwarded here, never
// compared or switched on.
//
// Deliberately generic and intent-based instead: CalibrationIntent names
// what the operator/session wants, not which of eighteen historical steps
// it resembles.
//
//   CalibrationManager               session lifecycle + evidence [UNCHANGED]
//        |  (session_active, origin, lease — a snapshot, never cached here)
//        v
//   CalibrationExecutionEngine        <- THIS: intent -> operation routing
//        |
//        v
//   SafeActuatorPolicy                the decision core            [UNCHANGED]
//        |
//        v
//   ActuatorRuntime                   the offline runtime adapter  [UNCHANGED, I4]
//        |
//        v
//   ActuatorBackend                   fake/test only — no production impl
//
// WHAT THIS OWNS, AND WHAT IT DELIBERATELY DOES NOT
// --------------------------------------------------
// Owns: the CalibrationIntent -> ActuatorOperation mapping, and the rule
// that RESTORE and ABORT never reach a backend call.
//
// Does NOT own session lifecycle or evidence — CalibrationManager remains
// the one owner. This class takes a CalibrationExecutionContext snapshot
// per call; it never stores one, never advances a session, never calls
// CalibrationManager itself.
//
// Does NOT own persistence or promotion. Whether an accepted calibration is
// eventually written by ActuatorAuthority::PROVISIONING or by a separate
// transaction remains TO_DESIGN (CALIBRATION_SOURCE_PRECEDENCE.md §7) — I5
// does not decide it, and this class has no durable-write path of any kind.
//
// CR3 resolves the final absolute raw target here through the ONE checked
// CalibrationTargetResolver. The policy remains the authority/geometry
// decision core; this layer performs no duplicate kinematic arithmetic.
//
// Does NOT resolve a joint identity to a bus id — exactly the same boundary
// ActuatorRuntime already draws; the caller supplies it.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY
// ----------------------------------------------
// There is no CalibrationIntent for removing torque and therefore no path
// to one — the same structural absence ActuatorWritePolicy.h and
// ActuatorRuntime.h already document for themselves.
// scripts/static_audit.py's check_calibration_execution_engine_boundaries()
// enforces that this file never names ServoBus::safeOff() or a torque
// removal, and that RESTORE/ABORT never reach the backend.
//
// RESTORE INTENT != ABORT != SAFE_OFF (V3 §16.3)
// -------------------------------------------------
// RESTORE is acknowledged as intent only — see CalibrationDomain.h's
// RestorePlan, "State only. No motion is produced by this model." — and
// ABORT is a lifecycle result the caller relays to
// CalibrationManager::abortSession(), never a move either. Both are
// categorically non-executing in execute() below: neither operation ever
// constructs an ActuatorCommand or calls the policy/runtime, which is what
// makes "authority loss must produce ZERO restore motion" true by
// construction rather than by a checked exception.

namespace matdog {
namespace calibration {

// What the operator/session currently wants. Deliberately NOT the 18-phase
// sequence — see the file comment.
enum class CalibrationIntent : uint8_t {
  NONE             = 0,  // no operation requested — never executes
  CONTACT_PROBE    = 1,  // -> ActuatorOperation::CALIBRATION_CONTACT_PROBE
  AUXILIARY_MOVE   = 2,  // -> ActuatorOperation::CALIBRATION_AUXILIARY_MOVE
  DIRECTION_VERIFY = 3,  // -> ActuatorOperation::DIRECTION_VERIFY (optional diagnostic)
  RESTORE          = 4,  // intent only — never reaches the backend, by construction
  ABORT            = 5,  // lifecycle only — never reaches the backend, by construction
  // -> ActuatorOperation::CALIBRATION_SEQUENCE_MOVE: a 24-contact Full
  // Calibration sequence move (plan target of the live phase) or the V25
  // GoalPosition prime at the present position. See CalibrationSequencePlan.h.
  SEQUENCE_MOVE    = 6,
};

// The operation an intent maps to, or NONE for the two non-executing
// intents and for an unknown/corrupted value. Pure and total.
actuator::ActuatorOperation operationForIntent(CalibrationIntent intent);

// What this call is allowed to look at — a snapshot the caller already
// holds (CalibrationManager::status(), the real arbiter's current lease).
// Never cached, never advanced by this class.
struct CalibrationExecutionContext {
  bool session_active = false;
  // A replay session authorises nothing physical however complete it
  // looks — the same rule CalibrationBootstrapContext already states.
  // Checked here too, in front of the policy call, as defence in depth: a
  // caller that forgets to route this into the policy's own bootstrap
  // context still cannot make a replay session write anything.
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
};

// The geometry-selected target/plan REFERENCE — a key into the bound
// CalibrationGeometryProfile, never a resolved numeric target. Mirrors
// ActuatorCommand's own endpoint_leg/joint/side + joint fields exactly, so
// building the command below is a direct copy, not a second scheme.
struct CalibrationExecutionRequest {
  CalibrationIntent intent = CalibrationIntent::NONE;

  // The joint that MOVES. For AUXILIARY_MOVE this is the auxiliary, not the
  // endpoint being calibrated — see ActuatorCommand.joint.
  JointIdentity joint{};

  // Which endpoint plan authorises this (CONTACT_PROBE: the joint being
  // probed; AUXILIARY_MOVE: the endpoint the parking serves).
  Leg endpoint_leg = Leg::LF;
  JointKind endpoint_joint = JointKind::HIP;
  ContactSide endpoint_side = ContactSide::MIN_SIDE;

  // DIRECTION_VERIFY only: a signed raw-tick excursion, gated against the
  // session's operator-approved budget and the geometry envelope entirely
  // inside SafeActuatorPolicy — this class invents no value and applies no
  // threshold of its own.
  int32_t direction_verify_delta_ticks = 0;

  // CONTACT_PROBE / AUXILIARY_MOVE: requested URDF-frame target. The policy
  // still checks the endpoint, parking and contact-side safety constraints.
  actuator::MicroRad target_urad = 0;

  // CONTACT_PROBE only: a raw-tick step of the staged endpoint search. The
  // policy bounds `search_target_tick` by the endpoint's calibration search
  // corridor (actuator::resolveCalibrationSearchCorridor); target_urad is not
  // used. Any other intent carrying it is refused before the policy.
  bool calibration_search = false;
  uint16_t search_target_tick = 0;

  // The GoalPosition speed envelope (actuator::MotionProfile). The policy
  // refuses CALIBRATION_SEARCH outside the calibration moves.
  actuator::MotionProfile motion_profile = actuator::MotionProfile::BOUNDED_DEFAULT;

  // SEQUENCE_MOVE only. TO_PLAN_TARGET uses target_urad (resolved here by the
  // checked resolver, re-derived again by the policy); PRIME_AT_PRESENT uses
  // prime_tick, the joint's present position. Any other intent carrying a
  // sequence move kind is refused before the policy.
  actuator::SequenceMoveKind sequence_move = actuator::SequenceMoveKind::NONE;
  CalibrationPhase sequence_phase = CalibrationPhase::PREFLIGHT;
  uint16_t prime_tick = 0;
};

enum class CalibrationExecutionOutcome : uint8_t {
  REJECT_UNKNOWN_INTENT          = 0,  // NONE, or a corrupted enum value
  REJECT_NO_SESSION              = 1,  // context.session_active == false
  REJECT_REPLAY_ORIGIN           = 2,  // HISTORICAL_REPLAY may never execute
  RESTORE_ACKNOWLEDGED_NO_MOTION = 3,  // RESTORE: state only, see RestorePlan
  ABORT_IS_LIFECYCLE_NOT_MOTION  = 4,  // ABORT: relay to CalibrationManager, not a move
  ROUTED_TO_POLICY               = 5,
  REJECT_NO_GEOMETRY_BINDING      = 6,
  REJECT_NO_TRANSFORM             = 7,
  REJECT_TARGET_RESOLUTION        = 8,
  REJECT_BUS_BINDING              = 9,
};

struct CalibrationExecutionResult {
  CalibrationExecutionOutcome outcome = CalibrationExecutionOutcome::REJECT_UNKNOWN_INTENT;
  // Populated only when outcome == ROUTED_TO_POLICY.
  actuator::WriteDecision policy_decision = actuator::WriteDecision::REJECT_NO_ARBITER;
  actuator::ExecuteResult execute_result = actuator::ExecuteResult::NOT_EXECUTED;
};

class CalibrationExecutionEngine {
 public:
  // Neither pointer is owned; both may be nullptr, which is a refusal
  // (ROUTED_TO_POLICY -> REJECT_NO_ARBITER via the policy's own fail-closed
  // handling), never a permission.
  void begin(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
             const actuator::CalibrationGeometryProfile* geometry,
             const actuator::GeometryProvenance* expected_provenance);

  // Translates one intent into at most one policy plan()+commit() and at
  // most one backend call — never more, and never any at all for
  // NONE/RESTORE/ABORT/an unrecognised intent.
  CalibrationExecutionResult execute(const CalibrationExecutionRequest& request,
                                     const CalibrationExecutionContext& context,
                                     uint8_t bus_id);

 private:
  actuator::SafeActuatorPolicy* policy_ = nullptr;
  actuator::ActuatorRuntime* runtime_ = nullptr;
  const actuator::CalibrationGeometryProfile* geometry_ = nullptr;
  const actuator::GeometryProvenance* expected_provenance_ = nullptr;
};

const char* toString(CalibrationIntent intent);
const char* toString(CalibrationExecutionOutcome outcome);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_EXECUTION_ENGINE_H
