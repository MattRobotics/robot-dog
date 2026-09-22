#include "ActuatorWritePolicy.h"

namespace matdog {
namespace actuator {

using calibration::JointIdentity;

// ---------------------------------------------------------------------------
// Operation classes
// ---------------------------------------------------------------------------

bool isKnownOperation(ActuatorOperation operation) {
  // Fail closed on a corrupted value. The underlying type is fixed at uint8_t,
  // so casting an out-of-range byte to this enum is well defined - which is
  // exactly why it has to be checked rather than assumed.
  return static_cast<uint8_t>(operation) < kActuatorOperationCount;
}

bool isCommandOperation(ActuatorOperation operation) {
  return isKnownOperation(operation) && operation != ActuatorOperation::NONE;
}

bool operationNeedsTarget(ActuatorOperation operation) {
  switch (operation) {
    case ActuatorOperation::POSITION_COMMAND:
    case ActuatorOperation::CALIBRATION_CONTACT_PROBE:
      return true;
    case ActuatorOperation::TORQUE_ENABLE:
    case ActuatorOperation::NONE:
      return false;
  }
  return false;
}

bool operationPermittedForOwner(core::ActuatorAuthority owner, ActuatorOperation operation) {
  if (!core::isWriteCapableOwner(owner)) return false;
  if (!isCommandOperation(operation)) return false;

  switch (owner) {
    case core::ActuatorAuthority::CALIBRATION:
      // The only owner that may probe a contact: driving a joint into a
      // mechanical endstop is a calibration measurement.
      return true;
    case core::ActuatorAuthority::MOTION:
      return operation != ActuatorOperation::CALIBRATION_CONTACT_PROBE;
    case core::ActuatorAuthority::DIAGNOSTICS:
    case core::ActuatorAuthority::QC:
    case core::ActuatorAuthority::PROVISIONING:
      // No runtime actuator command exists for these owners today, and one is
      // not invented here. PROVISIONING in particular is the EEPROM/persistent
      // side of the boundary, which this layer deliberately cannot express.
      return false;
    case core::ActuatorAuthority::NONE:
      return false;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------

bool JointLimit::usableProvenance() const {
  if (!present) return false;
  if (!ordered()) return false;
  if (!identity.valid() || !identity.unitKnown()) return false;
  // Exactly the test calibration::q0MayBeAppliedTo applies, and for the same
  // reason: a replayed bound describes a servo that is no longer in that
  // joint, and in the LF V25 case no longer in a leg at all.
  if (!calibration::mayPromote(origin)) return false;
  return calibration::isOperationalEvidence(state);
}

bool limitMayBeAppliedTo(const JointLimit& limit, const JointIdentity& current) {
  if (!limit.usableProvenance()) return false;
  return calibration::identityPermitsEvidenceReuse(limit.identity, current);
}

void ActuatorLimitTable::clear() {
  for (uint8_t i = 0; i < kCapacity; ++i) entries_[i] = JointLimit{};
  count_ = 0;
}

bool ActuatorLimitTable::admit(const JointLimit& limit) {
  // The single gate. Anything that cannot prove operational provenance is not
  // stored at all, so there is no "degraded" entry for a later reader to
  // misread as a bound.
  if (!limit.usableProvenance()) return false;

  for (uint8_t i = 0; i < count_; ++i) {
    if (calibration::identityPermitsEvidenceReuse(entries_[i].identity, limit.identity)) {
      entries_[i] = limit;
      return true;
    }
  }
  if (count_ >= kCapacity) return false;
  entries_[count_++] = limit;
  return true;
}

const JointLimit* ActuatorLimitTable::find(const JointIdentity& joint) const {
  if (!joint.valid() || !joint.unitKnown()) return nullptr;
  for (uint8_t i = 0; i < count_; ++i) {
    if (limitMayBeAppliedTo(entries_[i], joint)) return &entries_[i];
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// The policy
// ---------------------------------------------------------------------------

void SafeActuatorPolicy::begin(const core::ActuatorAuthorityArbiter* arbiter) {
  arbiter_ = arbiter;
  reset();
}

void SafeActuatorPolicy::reset() {
  // Advancing the epoch is what invalidates transactions a caller still holds:
  // they carry the old value and can never match again. Skipping 0 keeps a
  // zero-initialized transaction from matching a freshly reset policy.
  ++epoch_;
  if (epoch_ == 0) ++epoch_;
  outstanding_id_ = 0;
  limits_.clear();
  counters_.resets++;
  last_decision_ = WriteDecision::REJECT_TRANSACTION_STATE;
}

WriteDecision SafeActuatorPolicy::record(WriteDecision decision) {
  last_decision_ = decision;
  return decision;
}

WriteDecision SafeActuatorPolicy::evaluate(const ActuatorCommand& command,
                                           const core::AuthorityLease& lease,
                                           core::OperatingMode mode) const {
  // Fail closed: with no arbiter there is nothing to ask, which is a refusal
  // and not a free pass.
  if (arbiter_ == nullptr) return WriteDecision::REJECT_NO_ARBITER;

  if (!isCommandOperation(command.operation)) return WriteDecision::REJECT_UNKNOWN_OPERATION;

  // A bus id is an address, not an identity (CalibrationDomain.h): after the
  // 2026-08-27 reassembly, the slot alone no longer says which servo answers.
  // Every operation therefore needs the physical unit label, not only the ones
  // that consult a limit.
  if (!command.joint.valid() || !command.joint.unitKnown()) {
    return WriteDecision::REJECT_INVALID_JOINT;
  }

  if (!lease.valid() || !core::isWriteCapableOwner(lease.owner)) {
    return WriteDecision::REJECT_NO_AUTHORITY;
  }

  // Everything below this line is read from the arbiter, never from the
  // transaction: these are exactly the facts that can change between planning
  // a write and issuing it.
  if (arbiter_->inhibited()) return WriteDecision::REJECT_INHIBITED;

  const core::ActuatorAuthority current = arbiter_->current();
  if (current == core::ActuatorAuthority::NONE) return WriteDecision::REJECT_NO_AUTHORITY;
  if (lease.owner != current) return WriteDecision::REJECT_WRONG_OWNER;
  if (lease.generation != arbiter_->generation()) return WriteDecision::REJECT_STALE_GENERATION;

  if (!core::isModeCompatible(mode, lease.owner)) return WriteDecision::REJECT_MODE;

  if (!operationPermittedForOwner(lease.owner, command.operation)) {
    return WriteDecision::REJECT_OPERATION_NOT_PERMITTED;
  }

  if (operationNeedsTarget(command.operation)) {
    const JointLimit* limit = limits_.find(command.joint);
    // Today this is always the answer: MATDOG_JOINT_CALIBRATION.yaml holds no
    // accepted bound for any of the 12 leg joints. The safe result of a
    // missing bound is a refusal, never a fallback to a historical value.
    if (limit == nullptr) return WriteDecision::REJECT_NO_ACCEPTED_LIMITS;
    if (!limit->contains(command.target_tick)) return WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS;
  }

  return WriteDecision::ACCEPT;
}

WriteDecision SafeActuatorPolicy::plan(const ActuatorCommand& command,
                                       const core::AuthorityLease& lease,
                                       core::OperatingMode mode, ActuatorTransaction* out) {
  if (out == nullptr) {
    counters_.plan_rejections++;
    return record(WriteDecision::REJECT_TRANSACTION_STATE);
  }

  *out = ActuatorTransaction{};

  // One outstanding plan at a time. A second would create two callers each
  // believing they are the one about to write.
  if (outstanding_id_ != 0) {
    out->last_decision = WriteDecision::REJECT_TRANSACTION_STATE;
    out->state = TransactionState::REJECTED;
    counters_.plan_rejections++;
    return record(WriteDecision::REJECT_TRANSACTION_STATE);
  }

  const WriteDecision decision = evaluate(command, lease, mode);

  out->command = command;
  out->lease = lease;
  out->mode = mode;
  out->policy_epoch = epoch_;
  out->last_decision = decision;

  if (decision != WriteDecision::ACCEPT) {
    // A rejected plan produces no transaction to commit. It still carries the
    // reason, so a caller can report why without re-deriving it.
    out->state = TransactionState::REJECTED;
    counters_.plan_rejections++;
    return record(decision);
  }

  out->id = next_id_++;
  if (next_id_ == 0) next_id_ = 1;
  out->state = TransactionState::PLANNED;
  outstanding_id_ = out->id;
  counters_.plans++;
  return record(WriteDecision::ACCEPT);
}

WriteDecision SafeActuatorPolicy::commit(ActuatorTransaction* transaction) {
  if (transaction == nullptr) {
    counters_.commit_rejections++;
    return record(WriteDecision::REJECT_TRANSACTION_STATE);
  }

  // Identity first, contents second. A replay of a committed transaction, a
  // resumed abort, a transaction from before a reset and a copy a caller kept
  // all fail here, before anything they claim is even looked at.
  //
  // Order matters: a transaction that survived a reset is reported as a STALE
  // EPOCH rather than a generic state error, because that is the one a caller
  // can act on - it means re-plan, not "you used the API wrong".
  if (!transaction->planned()) {
    transaction->last_decision = WriteDecision::REJECT_TRANSACTION_STATE;
    counters_.commit_rejections++;
    return record(WriteDecision::REJECT_TRANSACTION_STATE);
  }

  if (transaction->policy_epoch != epoch_) {
    transaction->state = TransactionState::REJECTED;
    transaction->last_decision = WriteDecision::REJECT_STALE_EPOCH;
    counters_.commit_rejections++;
    return record(WriteDecision::REJECT_STALE_EPOCH);
  }

  // A copy of the outstanding transaction, or one whose turn has passed.
  if (transaction->id != outstanding_id_) {
    transaction->state = TransactionState::REJECTED;
    transaction->last_decision = WriteDecision::REJECT_TRANSACTION_STATE;
    counters_.commit_rejections++;
    return record(WriteDecision::REJECT_TRANSACTION_STATE);
  }

  // The re-verification. The captured lease is an argument to the same rule set
  // the plan ran, not a shortcut around it: the arbiter is read again here.
  const WriteDecision decision = evaluate(transaction->command, transaction->lease,
                                          transaction->mode);

  outstanding_id_ = 0;  // consumed either way; a refused commit cannot be retried
  transaction->last_decision = decision;

  if (decision != WriteDecision::ACCEPT) {
    transaction->state = TransactionState::REJECTED;
    counters_.commit_rejections++;
    return record(decision);
  }

  transaction->state = TransactionState::COMMITTED;
  counters_.commits++;
  return record(WriteDecision::ACCEPT);
}

void SafeActuatorPolicy::abort(ActuatorTransaction* transaction) {
  if (transaction == nullptr) return;
  if (transaction->id != 0 && transaction->id == outstanding_id_) outstanding_id_ = 0;
  if (transaction->state == TransactionState::PLANNED ||
      transaction->state == TransactionState::IDLE) {
    transaction->state = TransactionState::ABORTED;
    transaction->last_decision = WriteDecision::REJECT_TRANSACTION_STATE;
    counters_.aborts++;
  }
}

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

const char* toString(ActuatorOperation operation) {
  switch (operation) {
    case ActuatorOperation::NONE:                      return "NONE";
    case ActuatorOperation::TORQUE_ENABLE:             return "TORQUE_ENABLE";
    case ActuatorOperation::POSITION_COMMAND:          return "POSITION_COMMAND";
    case ActuatorOperation::CALIBRATION_CONTACT_PROBE: return "CALIBRATION_CONTACT_PROBE";
  }
  return "UNKNOWN";
}

const char* toString(WriteDecision decision) {
  switch (decision) {
    case WriteDecision::ACCEPT:                      return "ACCEPT";
    case WriteDecision::REJECT_NO_ARBITER:           return "REJECT_NO_ARBITER";
    case WriteDecision::REJECT_NO_AUTHORITY:         return "REJECT_NO_AUTHORITY";
    case WriteDecision::REJECT_WRONG_OWNER:          return "REJECT_WRONG_OWNER";
    case WriteDecision::REJECT_STALE_GENERATION:     return "REJECT_STALE_GENERATION";
    case WriteDecision::REJECT_INHIBITED:            return "REJECT_INHIBITED";
    case WriteDecision::REJECT_MODE:                 return "REJECT_MODE";
    case WriteDecision::REJECT_UNKNOWN_OPERATION:    return "REJECT_UNKNOWN_OPERATION";
    case WriteDecision::REJECT_OPERATION_NOT_PERMITTED: return "REJECT_OPERATION_NOT_PERMITTED";
    case WriteDecision::REJECT_INVALID_JOINT:        return "REJECT_INVALID_JOINT";
    case WriteDecision::REJECT_NO_ACCEPTED_LIMITS:   return "REJECT_NO_ACCEPTED_LIMITS";
    case WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS: return "REJECT_TARGET_OUT_OF_BOUNDS";
    case WriteDecision::REJECT_TRANSACTION_STATE:    return "REJECT_TRANSACTION_STATE";
    case WriteDecision::REJECT_STALE_EPOCH:          return "REJECT_STALE_EPOCH";
  }
  return "UNKNOWN";
}

const char* toString(TransactionState state) {
  switch (state) {
    case TransactionState::IDLE:      return "IDLE";
    case TransactionState::PLANNED:   return "PLANNED";
    case TransactionState::COMMITTED: return "COMMITTED";
    case TransactionState::REJECTED:  return "REJECTED";
    case TransactionState::ABORTED:   return "ABORTED";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
