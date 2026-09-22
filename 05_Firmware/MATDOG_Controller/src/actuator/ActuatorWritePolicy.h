#ifndef MATDOG_ACTUATOR_ACTUATOR_WRITE_POLICY_H
#define MATDOG_ACTUATOR_ACTUATOR_WRITE_POLICY_H

#include <stdint.h>

#include "../calibration/CalibrationDomain.h"
#include "../core/ActuatorAuthority.h"
#include "../core/OperatingMode.h"

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
enum class ActuatorOperation : uint8_t {
  NONE                      = 0,  // the absence of an operation, never an operation
  TORQUE_ENABLE             = 1,  // APPLY torque. Removing it is SAFE_OFF's job, not this layer's.
  POSITION_COMMAND          = 2,  // a single-joint goal position
  CALIBRATION_CONTACT_PROBE = 3,  // a bounded approach expecting a contact witness
};

constexpr uint8_t kActuatorOperationCount = 4;

bool isKnownOperation(ActuatorOperation operation);

// An operation that actually commands something. NONE is not one.
bool isCommandOperation(ActuatorOperation operation);

// Which operations carry a target tick, and therefore need accepted bounds.
bool operationNeedsTarget(ActuatorOperation operation);

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
  uint16_t min_tick = 0;
  uint16_t max_tick = 0;
  bool present = false;  // false means "no bound exists", never "use the full range"

  // Operational provenance, checked exactly like calibration::q0MayBeAppliedTo:
  // measured on the current installation (LIVE_SESSION) and promoted to
  // operational calibration (PROMOTED). A replay can reach neither.
  bool usableProvenance() const;

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
  // identified by slot AND physical unit, and carries operational provenance.
  // Re-admitting the same joint replaces its entry.
  bool admit(const JointLimit& limit);

  const JointLimit* find(const calibration::JointIdentity& joint) const;

  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }

 private:
  JointLimit entries_[kCapacity];
  uint8_t count_ = 0;
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
  calibration::JointIdentity joint{};
  uint16_t target_tick = 0;  // meaningless unless operationNeedsTarget(operation)
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

  ActuatorLimitTable& limits() { return limits_; }
  const ActuatorLimitTable& limits() const { return limits_; }

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

  const core::ActuatorAuthorityArbiter* arbiter_ = nullptr;
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
