#ifndef MATDOG_ACTUATOR_ACTUATOR_WRITE_POLICY_H
#define MATDOG_ACTUATOR_ACTUATOR_WRITE_POLICY_H

#include <stdint.h>

#include "../calibration/CalibrationDomain.h"
#include "../core/ActuatorAuthority.h"
#include "../core/OperatingMode.h"
#include "CalibrationGeometryProfile.h"

// The Safe Actuator Layer's decision core.
//
// Pure: <stdint.h> plus the two already-pure MATDOG units it reasons about.
// No <Arduino.h>, no ServoBus, no UART, no Wi-Fi, no OTA, no scheduler, no
// Serial. scripts/tests/test_actuator_write_policy.cpp links the REAL policy,
// the same contract as core/ActuatorAuthority, network/WifiPolicy,
// update/OtaPolicy and calibration/CalibrationDomain.
//
//   Calibration Execution Engine        [NOT IMPLEMENTED]
//        |
//        v
//   SafeActuatorPolicy                  <- THIS: the decision
//        |
//        v
//   ActuatorAuthority verification      core/ActuatorAuthority.h
//        |
//        v
//   runtime adapter -> ServoBus         [TO_IMPLEMENT - see SAFE_ACTUATOR_LAYER.md §7]
//
// WHAT THIS IS NOT
// ----------------
// It is not a transport and it cannot become one: there is no bus handle in
// it, so an ACCEPT is a decision, never an action. It is not a second
// authority model - it asks the one arbiter and never remembers the answer.
// It is not a second calibration state machine - it holds no session.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY
// --------------------------------------------
// There is no operation class for removing torque. A safety de-escalation is
// therefore not merely permitted around this policy - it is INEXPRESSIBLE
// through it, so no future edit can accidentally make SAFE_OFF depend on an
// authority check. ServoBus::safeOff() stays the ungated path, and
// scripts/static_audit.py enforces that ServoBus cannot reference this layer.

namespace matdog {
namespace actuator {

// ---------------------------------------------------------------------------
// Operation classes - only what current evidence justifies
// ---------------------------------------------------------------------------
//
// Deliberately absent: any persistent/provisioning write. The repository does
// not settle whether persisting an accepted calibration belongs to
// ActuatorAuthority::PROVISIONING or to a separate post-acceptance
// transaction (CALIBRATION_SOURCE_PRECEDENCE.md §7 records it as TO_DESIGN),
// so this type makes a persistent write inexpressible rather than guessing an
// owner for it - the same discipline that keeps bus_id out of JointIdentity.
// static_audit.py fails the build if such a class is added here.
//
//   runtime actuator command  !=  persistent provisioning/configuration write
//
// The three calibration classes are NOT a mechanical expansion. Each is tied to
// a DIFFERENT authorisation object, which is the only reason it exists:
//
//   DIRECTION_VERIFY            OPTIONAL / DIAGNOSTIC ONLY - see below. The
//                               symmetric proven-clear envelope around q=0. It
//                               has no endpoint plan.
//   CALIBRATION_AUXILIARY_MOVE  a parking plan's auxiliary joint - a DIFFERENT
//                               joint from the one being calibrated. The
//                               compiler makes that distinction explicit for all
//                               six obstructed endpoints; collapsing it would
//                               let a probe command move an unrelated leg.
//   CALIBRATION_CONTACT_PROBE   the endpoint's own task path toward contact.
//
// Backoff and restore are deliberately NOT separate classes: both retreat along
// a corridor the same plan already validated, so they are the same intent with a
// different target. Adding classes for them would be the mechanical expansion
// the handoff warns against.
enum class ActuatorOperation : uint8_t {
  NONE                       = 0,  // the absence of an operation, never an operation
  TORQUE_ENABLE              = 1,  // APPLY torque. Removing it is SAFE_OFF's job, not this layer's.
  POSITION_COMMAND           = 2,  // a single-joint goal position under accepted limits
  CALIBRATION_CONTACT_PROBE  = 3,  // a bounded approach expecting a contact witness
  // OPTIONAL / DIAGNOSTIC ONLY. NOT part of the calibration contract.
  //
  // Joint direction is HARDWARE-CONTRACT DATA read from the URDF (see
  // CalibrationGeometryProfile::jointDirection), already validated on real
  // hardware, and it is NOT re-measured during normal calibration or after a
  // same-type servo replacement in the same mounting. This operation is
  // therefore never required for calibration acceptance and never required
  // for stand authorization.
  //
  // It survives only as a development/maintenance diagnostic - "does this
  // joint move, and does it move the way the contract says" after a
  // mechanical repair. Its budget defaults to 0 = not authorised, and nothing
  // in the calibration path consults it.
  DIRECTION_VERIFY           = 4,  // a micro excursion from the captured q0 tick
  CALIBRATION_AUXILIARY_MOVE = 5,  // park/unpark a plan's auxiliary joint
};

constexpr uint8_t kActuatorOperationCount = 6;

bool isKnownOperation(ActuatorOperation operation);

// An operation that actually commands something. NONE is not one.
bool isCommandOperation(ActuatorOperation operation);

// Which operations carry a target at all. TORQUE_ENABLE does not.
bool operationNeedsTarget(ActuatorOperation operation);

// The three authorisation routes, mutually exclusive by construction. Which one
// an operation takes is a property of the operation, never of the caller.
//
// POSITION_COMMAND keeps the ORIGINAL route and is not weakened by anything
// below: it still requires an accepted joint bound, and none exists.
bool operationUsesAcceptedLimits(ActuatorOperation operation);
// DIRECTION_VERIFY only: the geometry envelope, checked in tick space. An
// OPTIONAL diagnostic route - no calibration step takes it.
bool operationUsesBootstrapEnvelope(ActuatorOperation operation);
// The plan-bound calibration moves: a V5 endpoint record authorises them.
bool operationUsesEndpointPlan(ActuatorOperation operation);

// Owner -> operation eligibility.
//
// MOTION may not issue a contact probe: a probe exists to drive a joint into
// a mechanical endstop, which is a calibration measurement, not motion.
// DIAGNOSTICS/QC/PROVISIONING have no runtime actuator command today and one
// is not invented for them here.
bool operationPermittedForOwner(core::ActuatorAuthority owner, ActuatorOperation operation);

// ---------------------------------------------------------------------------
// Limits - provenance first, value second
// ---------------------------------------------------------------------------
//
// MATDOG_JOINT_CALIBRATION.yaml, the current authority, records
// safe_limit_rad / first_stand_limit_rad / measured_contact_rad as
// {min: null, max: null} for ALL TWELVE leg joints, under
// CALIBRATION_RESET_PENDING_FULL_RECALIBRATION. There is not one accepted
// joint bound in the repository today.
//
// So the correct answer for every position-class command is
// REJECT_NO_ACCEPTED_LIMITS, and a fallback to historical values is exactly
// the failure this type exists to prevent: a limit is usable only if its own
// provenance says it is operational calibration measured on the current
// installation, for the same joint slot AND the same physical servo.
struct JointLimit {
  calibration::JointIdentity identity{};
  calibration::EvidenceState state = calibration::EvidenceState::UNKNOWN;
  calibration::CalibrationOrigin origin = calibration::CalibrationOrigin::NONE;
  // WHICH MODEL this bound was derived under. A safe operational limit
  // descends from a contact target that came from the geometry plan and from
  // a MODEL-versus-REAL comparison whose model half is the URDF. Change the
  // URDF and the comparison no longer describes this robot.
  GeometryProvenanceTag geometry = kNoGeometryProvenance;
  uint16_t min_tick = 0;
  uint16_t max_tick = 0;
  bool present = false;  // false means "no bound exists", never "use the full range"

  // Operational provenance, checked exactly like calibration::q0MayBeAppliedTo:
  // measured on the current installation (LIVE_SESSION) and promoted to
  // operational calibration (PROMOTED). A replay can reach neither.
  bool usableProvenance() const;

  // The geometry axis, separate from usableProvenance() on purpose: a bound
  // can be perfectly measured and still belong to a model that is no longer
  // loaded. It stays historically recorded; it is not CURRENT evidence.
  bool boundToGeometry() const { return geometry != kNoGeometryProvenance; }

  // A bound with min > max is corrupt, not permissive.
  bool ordered() const { return min_tick <= max_tick; }

  bool contains(uint16_t tick) const { return present && ordered() && tick >= min_tick && tick <= max_tick; }
};

// Both axes of identity must agree before a stored limit may be applied to a
// commanded joint - slot agreement alone is what the 2026-08-27 reassembly
// made unsafe.
bool limitMayBeAppliedTo(const JointLimit& limit, const calibration::JointIdentity& current);

// The accepted-limit store. Empty at boot and empty today, because nothing in
// the repository qualifies to fill it.
//
// admit() is the only way in and it REFUSES anything whose provenance does not
// qualify, so "historical values leaked into runtime limits" is not a review
// item - it is a rejected call. static_audit.py additionally forbids any
// runtime translation unit from calling it.
class ActuatorLimitTable {
 public:
  static constexpr uint8_t kCapacity = calibration::kLegServoSlotCount;  // 12 leg joints

  void clear();

  // Returns false and stores nothing unless the limit is present, ordered,
  // identified by slot AND physical unit, carries operational provenance AND
  // names the geometry it was measured under. Re-admitting the same joint
  // replaces its entry.
  bool admit(const JointLimit& limit);

  // CURRENT operational evidence: identity AND geometry must both match.
  // A zero tag matches nothing, so an unbound policy finds nothing.
  const JointLimit* find(const calibration::JointIdentity& joint,
                         GeometryProvenanceTag geometry) const;

  // The historical view: identity only. A record measured under a superseded
  // model is still ON RECORD and findable here - it is simply never returned
  // by find(), so it can never authorise a write. Reporting surfaces use
  // this; the decision path must not.
  const JointLimit* findAny(const calibration::JointIdentity& joint) const;

  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }

 private:
  JointLimit entries_[kCapacity];
  uint8_t count_ = 0;
};

// The accepted raw<->q transform store. Same shape and same gate as the limit
// table: admit() refuses anything without operational provenance, so a
// historical q0 or an unmeasured direction cannot become a live transform.
// Empty today, because no q0 has been captured on the current installation.
class JointTransformTable {
 public:
  static constexpr uint8_t kCapacity = calibration::kLegServoSlotCount;  // 12 leg joints

  void clear();
  bool admit(const JointTransform& transform);

  // CURRENT operational evidence: identity AND geometry must both match.
  const JointTransform* find(const calibration::JointIdentity& joint,
                             GeometryProvenanceTag geometry) const;

  // The historical view: identity only, geometry ignored. See the note on
  // ActuatorLimitTable::findAny().
  const JointTransform* findAny(const calibration::JointIdentity& joint) const;

  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }

 private:
  JointTransform entries_[kCapacity];
  uint8_t count_ = 0;
};

// ---------------------------------------------------------------------------
// The calibration bootstrap context
// ---------------------------------------------------------------------------
//
// What the CURRENT calibration session has established. Supplied by the
// CalibrationManager; the policy never invents any of it, and every field
// defaults to the refusing value.
//
// It is not a second calibration state machine. It carries exactly the four
// facts the geometry-authorised operations must check and nothing else.
struct CalibrationBootstrapContext {
  bool session_active = false;
  // A replay session authorises nothing physical, however complete it looks.
  calibration::CalibrationOrigin origin = calibration::CalibrationOrigin::NONE;

  // Snapshot of a separately validated RAM-only calibration motion permit.
  // Defaults refuse every CALIBRATION-owned write.
  bool motion_permit_active = false;
  uint32_t motion_permit_generation = 0;
  uint32_t motion_permit_session_id = 0;
  uint32_t motion_permit_authority_generation = 0;

  // OPTIONAL DIAGNOSTIC BUDGET. NOT a calibration prerequisite and NOT a gate
  // anything waits on.
  //
  // The operator-approved excursion for the optional DIRECTION_VERIFY
  // diagnostic, in raw ticks. Zero means NOT AUTHORISED, which is the normal
  // state: no calibration step, no acceptance decision and no stand
  // authorization consults this field. Direction itself comes from the URDF.
  //
  // When a diagnostic IS authorised, geometry says what is clear and the
  // session says how much of that it may use; both must agree.
  int32_t direction_verify_tick_budget = 0;

  // Which endpoint, if any, currently has its auxiliary joint parked. The
  // compiler validated every direct path with ALL other joints at q=0, so a
  // parked auxiliary invalidates a NOT_NEEDED direct path just as surely as a
  // missing one invalidates an obstructed plan.
  bool auxiliary_parked = false;
  calibration::Leg parked_leg = calibration::Leg::LF;
  calibration::JointKind parked_joint = calibration::JointKind::HIP;
  calibration::ContactSide parked_side = calibration::ContactSide::MIN_SIDE;
};

// ---------------------------------------------------------------------------
// The decision
// ---------------------------------------------------------------------------

enum class WriteDecision : uint8_t {
  ACCEPT                    = 0,
  REJECT_NO_ARBITER         = 1,   // fail closed: nothing to ask
  REJECT_NO_AUTHORITY       = 2,   // nobody owns the actuators, or the lease is empty
  REJECT_WRONG_OWNER        = 3,   // the lease names an owner that no longer holds it
  REJECT_STALE_GENERATION   = 4,   // right owner, earlier session
  REJECT_INHIBITED          = 5,   // an exclusive-activity hold is in place (OTA)
  REJECT_MODE               = 6,   // OperatingMode incompatible with the owner
  REJECT_UNKNOWN_OPERATION  = 7,   // NONE, or a corrupted enum value
  REJECT_OPERATION_NOT_PERMITTED = 8,  // known operation, wrong owner for it
  REJECT_INVALID_JOINT      = 9,   // unknown slot, or no physical unit label
  REJECT_NO_ACCEPTED_LIMITS = 10,  // no bound with operational provenance exists
  REJECT_TARGET_OUT_OF_BOUNDS = 11,
  REJECT_TRANSACTION_STATE  = 12,  // replay, resume after abort, or a second outstanding plan
  REJECT_STALE_EPOCH        = 13,  // the policy was reset under the transaction

  // --- calibration bootstrap, geometry-authorised --------------------------
  REJECT_NO_GEOMETRY_PROFILE        = 14,  // no compiled geometry bound
  REJECT_GEOMETRY_PROVENANCE        = 15,  // the profile is not the model this build expects
  REJECT_NO_CALIBRATION_SESSION     = 16,  // no live session, or a replay session
  REJECT_UNKNOWN_GEOMETRY_JOINT     = 17,  // the joint is not in the compiled model
  REJECT_NO_ENVELOPE_BUDGET         = 18,  // the session authorised no excursion
  REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE = 19,  // beyond the proven-clear span, or beyond the budget
  REJECT_NO_ENDPOINT_PLAN           = 20,  // the compiler produced no plan for this endpoint
  REJECT_ENDPOINT_NOT_EXECUTABLE    = 21,  // diagnostic endpoint, or clearance not PASS
  REJECT_PARKING_REQUIRED           = 22,  // the plan is obstructed and nothing is parked
  REJECT_UNEXPECTED_PARKING         = 23,  // a direct path validated at q=0, with something parked
  REJECT_WRONG_AUXILIARY_JOINT      = 24,  // not the auxiliary joint the plan names
  REJECT_AUXILIARY_TARGET           = 25,  // not the parked pose the compiler found
  REJECT_NO_ACCEPTED_TRANSFORM      = 26,  // no raw<->q transform with live, promoted provenance
  REJECT_TARGET_OUTSIDE_URDF_LIMITS = 27,  // outside the joint's declared URDF domain

  // Evidence exists for this joint and is well-formed, but it was measured
  // under a DIFFERENT geometry model. It stays on record; it is not current.
  REJECT_EVIDENCE_GEOMETRY_MISMATCH = 28,
  REJECT_NO_CALIBRATION_MOTION_PERMIT = 29,
  // A contact-probe overtravel allowance on anything but a CONTACT_PROBE
  // approach to the canonical contact, above kContactProbeMaxOvertravelTicks,
  // or not landing on exactly the URDF-clamped tick that allowance resolves to.
  REJECT_PROBE_OVERTRAVEL = 30,
};

// ---------------------------------------------------------------------------
// The transaction
// ---------------------------------------------------------------------------

enum class TransactionState : uint8_t {
  IDLE      = 0,
  PLANNED   = 1,
  COMMITTED = 2,
  REJECTED  = 3,
  ABORTED   = 4,
};

struct ActuatorCommand {
  ActuatorOperation operation = ActuatorOperation::NONE;

  // The joint that MOVES. For an auxiliary move this is NOT the joint being
  // calibrated - that is the whole point of the class.
  calibration::JointIdentity joint{};

  // Absolute unsigned ST3215 target. CR3 resolves calibration geometry
  // operations through the one checked q<->raw resolver before planning.
  uint16_t target_tick = 0;

  // DIRECTION_VERIFY only: a SIGNED excursion from the captured q0 tick. Ticks,
  // not radians, because the magnitude of a tick delta is a servo-profile fact
  // that needs neither q0 nor direction - which is exactly what makes this move
  // checkable before either exists.
  int32_t delta_ticks = 0;

  // The plan-bound moves: a target in the URDF joint frame. Reaching it needs
  // the raw<->q transform, so it is refused until one is accepted.
  MicroRad target_urad = 0;

  // Which endpoint plan authorises this command. For a contact probe this is
  // the joint being probed; for an auxiliary move it is the endpoint the
  // parking serves, while `joint` above is the auxiliary being moved.
  calibration::Leg endpoint_leg = calibration::Leg::LF;
  calibration::JointKind endpoint_joint = calibration::JointKind::HIP;
  calibration::ContactSide endpoint_side = calibration::ContactSide::MIN_SIDE;

  // CALIBRATION_CONTACT_PROBE approach only (hardware finding 2026-09-29,
  // operator-approved): the requested ceiling, in raw ticks, on how far past
  // the canonical contact `target_urad` names target_tick may lie in the
  // endpoint side's approach direction. The tick actually commanded is that
  // ceiling clamped to the URDF joint limit - see
  // resolveContactProbeApproachToRaw(). 0 for every other command; non-zero
  // anywhere else is REJECT_PROBE_OVERTRAVEL.
  uint16_t contact_probe_overtravel_ticks = 0;
};

// What a planned write knows about the world it was planned in. Every field is
// EVIDENCE OF WHAT WAS TRUE AT PLAN TIME, never a permit: commit() re-reads all
// of it from the live arbiter. There is no "check once then write later" path.
struct ActuatorTransaction {
  uint32_t id = 0;  // 0 means "never planned"
  TransactionState state = TransactionState::IDLE;
  ActuatorCommand command{};
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
  uint32_t policy_epoch = 0;
  WriteDecision last_decision = WriteDecision::REJECT_TRANSACTION_STATE;

  bool planned() const { return id != 0 && state == TransactionState::PLANNED; }
};

struct ActuatorPolicyCounters {
  uint32_t plans = 0;
  uint32_t plan_rejections = 0;
  uint32_t commits = 0;
  uint32_t commit_rejections = 0;
  uint32_t aborts = 0;
  uint32_t resets = 0;
};

// The result of offering one measured operational bound to the policy. The
// ONLY route into limits() from a live calibration run.
enum class LimitAdmission : uint8_t {
  ADMITTED                  = 0,
  REJECT_NOT_OPERATIONAL    = 1,  // absent, unordered, unknown unit, or not LIVE_SESSION + PROMOTED
  REJECT_TICK_OUT_OF_RANGE  = 2,  // outside the unsigned ST3215 domain 0..4095
  REJECT_GEOMETRY_NOT_CURRENT = 3,  // no current geometry, or the bound names a different model
  REJECT_NO_TRANSFORM       = 4,  // no current promoted q0 for this joint
  REJECT_TABLE_REJECTED     = 5,  // the limit table itself refused (capacity)
};

const char* toString(LimitAdmission admission);

class SafeActuatorPolicy {
 public:
  // arbiter may be nullptr; that is a refusal, not a permission.
  void begin(const core::ActuatorAuthorityArbiter* arbiter);

  // Invalidates every outstanding transaction by advancing the epoch, and
  // clears the accepted-limit store. Boot, fault, owner teardown, re-init.
  void reset();

  // Plans one write. Captures the lease, the mode and the epoch into `out` and
  // returns the decision that would be made right now.
  //
  // AT MOST ONE OUTSTANDING PLAN. A second plan while one is still PLANNED is
  // refused rather than queued: two live transactions would each believe they
  // are the one about to write, which is the same class of bug the arbiter's
  // ALREADY_OWNED refusal exists to prevent. It is also what makes replay of a
  // COPIED transaction impossible - see commit().
  WriteDecision plan(const ActuatorCommand& command, const core::AuthorityLease& lease,
                     core::OperatingMode mode, ActuatorTransaction* out);

  // Re-verifies EVERYTHING against live state and consumes the transaction.
  // The captured lease is compared against the arbiter as it is now, not
  // trusted. A transaction whose id is not the outstanding one - a copy kept
  // by a caller, a replay of a committed one, a resumed abort - is refused on
  // identity alone, before any of its contents are considered.
  WriteDecision commit(ActuatorTransaction* transaction);

  // Ends a transaction without committing. An aborted transaction can never be
  // resumed; a new one must be planned.
  void abort(ActuatorTransaction* transaction);

  // Side-effect-free precheck of admitOperationalLimit(): the same verdict,
  // nothing stored. A caller with several bounds to admit as a set validates
  // them all first, so a late refusal cannot leave a partial set behind.
  LimitAdmission validateOperationalLimit(const JointLimit& limit) const;

  // Admits one MEASURED operational bound into limits(). Refuses, storing
  // nothing, unless the bound carries operational provenance, sits inside the
  // ST3215 raw domain, names EXACTLY the geometry this policy currently
  // holds, and the joint has a current promoted q0. It is the only
  // production route into limits(); static_audit.py confines its callers.
  LimitAdmission admitOperationalLimit(const JointLimit& limit);

  ActuatorLimitTable& limits() { return limits_; }
  const ActuatorLimitTable& limits() const { return limits_; }

  JointTransformTable& transforms() { return transforms_; }
  const JointTransformTable& transforms() const { return transforms_; }

  // Binds the compiled geometry AND the provenance this build expects it to
  // have. Both are required: a profile whose provenance cannot be matched is
  // a profile from a different robot.
  void bindGeometry(const CalibrationGeometryProfile* profile,
                    const GeometryProvenance* expected_provenance);

  void setBootstrapContext(const CalibrationBootstrapContext& context);
  const CalibrationBootstrapContext& bootstrapContext() const { return bootstrap_; }

  // The tag every stored record must carry to count as current evidence:
  // the bound profile's, and only when its provenance is the one this build
  // expects. Unbound, mismatched or half-bound all yield
  // kNoGeometryProvenance, which matches no record.
  GeometryProvenanceTag currentGeometryTag() const;

  uint32_t epoch() const { return epoch_; }
  bool hasOutstandingTransaction() const { return outstanding_id_ != 0; }
  const ActuatorPolicyCounters& counters() const { return counters_; }
  WriteDecision lastDecision() const { return last_decision_; }

 private:
  // The whole rule set, in one place, evaluated identically at plan and at
  // commit. Anything that can change between the two - authority, generation,
  // inhibit, mode - is read from the arbiter here, never from the transaction.
  WriteDecision evaluate(const ActuatorCommand& command, const core::AuthorityLease& lease,
                         core::OperatingMode mode) const;

  WriteDecision record(WriteDecision decision);

  WriteDecision evaluateBootstrapEnvelope(const ActuatorCommand& command) const;
  WriteDecision evaluateEndpointPlan(const ActuatorCommand& command) const;
  WriteDecision geometryPreconditions() const;

  const core::ActuatorAuthorityArbiter* arbiter_ = nullptr;
  const CalibrationGeometryProfile* geometry_ = nullptr;
  const GeometryProvenance* expected_provenance_ = nullptr;
  CalibrationBootstrapContext bootstrap_{};
  JointTransformTable transforms_{};
  ActuatorLimitTable limits_{};
  uint32_t epoch_ = 1;         // never 0: a zeroed transaction must not match
  uint32_t next_id_ = 1;       // never 0: 0 means "never planned"
  uint32_t outstanding_id_ = 0;
  ActuatorPolicyCounters counters_{};
  WriteDecision last_decision_ = WriteDecision::REJECT_NO_ARBITER;
};

const char* toString(ActuatorOperation operation);
const char* toString(WriteDecision decision);
const char* toString(TransactionState state);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_ACTUATOR_WRITE_POLICY_H
