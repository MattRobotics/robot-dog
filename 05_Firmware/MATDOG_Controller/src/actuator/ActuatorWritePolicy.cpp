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
    case ActuatorOperation::DIRECTION_VERIFY:
    case ActuatorOperation::CALIBRATION_AUXILIARY_MOVE:
      return true;
    case ActuatorOperation::TORQUE_ENABLE:
    case ActuatorOperation::NONE:
      return false;
  }
  return false;
}

bool operationUsesAcceptedLimits(ActuatorOperation operation) {
  // Exactly one operation, unchanged by the calibration bootstrap. A geometry
  // plan is evidence about the MODEL; it is not an accepted bound on the
  // current machine and must never be allowed to stand in for one here.
  return operation == ActuatorOperation::POSITION_COMMAND;
}

bool operationUsesBootstrapEnvelope(ActuatorOperation operation) {
  return operation == ActuatorOperation::DIRECTION_VERIFY;
}

bool operationUsesEndpointPlan(ActuatorOperation operation) {
  return operation == ActuatorOperation::CALIBRATION_CONTACT_PROBE ||
         operation == ActuatorOperation::CALIBRATION_AUXILIARY_MOVE;
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
      // Motion gets torque and position commands. Every calibration-bootstrap
      // class is refused: a probe drives a joint into a mechanical endstop, an
      // auxiliary move parks a leg that is not the one being commanded, and a
      // direction verification is an optional diagnostic that has no place in
      // a motion owner's vocabulary.
      return operation == ActuatorOperation::TORQUE_ENABLE ||
             operation == ActuatorOperation::POSITION_COMMAND;
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
  // And it must say which model it was measured under. A bound that cannot
  // name its geometry could never be invalidated when that geometry changes,
  // which is the whole point of carrying the tag.
  if (!limit.boundToGeometry()) return false;

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

const JointLimit* ActuatorLimitTable::findAny(const JointIdentity& joint) const {
  if (!joint.valid() || !joint.unitKnown()) return nullptr;
  for (uint8_t i = 0; i < count_; ++i) {
    if (limitMayBeAppliedTo(entries_[i], joint)) return &entries_[i];
  }
  return nullptr;
}

const JointLimit* ActuatorLimitTable::find(const JointIdentity& joint,
                                           GeometryProvenanceTag geometry) const {
  // Fail closed on an unbound caller: kNoGeometryProvenance matches nothing,
  // so a policy with no geometry loaded finds no evidence at all.
  if (geometry == kNoGeometryProvenance) return nullptr;
  const JointLimit* entry = findAny(joint);
  if (entry == nullptr) return nullptr;
  return entry->geometry == geometry ? entry : nullptr;
}

// ---------------------------------------------------------------------------
// Accepted raw<->q transforms
// ---------------------------------------------------------------------------

void JointTransformTable::clear() {
  for (uint8_t i = 0; i < kCapacity; ++i) entries_[i] = JointTransform{};
  count_ = 0;
}

bool JointTransformTable::admit(const JointTransform& transform) {
  // The single gate, identical in shape to the limit table's: a transform that
  // cannot prove it was measured on the current installation and promoted to
  // operational calibration is not stored at all.
  if (!transform.usableProvenance()) return false;
  // q0 is captured at the nominal URDF q=0 pose, so it is only meaningful
  // against the URDF that defines that pose. A transform must name it.
  if (!transform.boundToGeometry()) return false;

  for (uint8_t i = 0; i < count_; ++i) {
    if (calibration::identityPermitsEvidenceReuse(entries_[i].identity, transform.identity)) {
      entries_[i] = transform;
      return true;
    }
  }
  if (count_ >= kCapacity) return false;
  entries_[count_++] = transform;
  return true;
}

const JointTransform* JointTransformTable::findAny(const JointIdentity& joint) const {
  if (!joint.valid() || !joint.unitKnown()) return nullptr;
  for (uint8_t i = 0; i < count_; ++i) {
    if (transformMayBeAppliedTo(entries_[i], joint)) return &entries_[i];
  }
  return nullptr;
}

const JointTransform* JointTransformTable::find(const JointIdentity& joint,
                                                GeometryProvenanceTag geometry) const {
  if (geometry == kNoGeometryProvenance) return nullptr;
  const JointTransform* entry = findAny(joint);
  if (entry == nullptr) return nullptr;
  return entry->geometry == geometry ? entry : nullptr;
}

// ---------------------------------------------------------------------------
// The policy
// ---------------------------------------------------------------------------

void SafeActuatorPolicy::begin(const core::ActuatorAuthorityArbiter* arbiter) {
  arbiter_ = arbiter;
  reset();
}

void SafeActuatorPolicy::bindGeometry(const CalibrationGeometryProfile* profile,
                                      const GeometryProvenance* expected_provenance) {
  // Either both or neither. A profile with nothing to check it against is a
  // profile from an unknown robot.
  if (profile == nullptr || expected_provenance == nullptr) {
    geometry_ = nullptr;
    expected_provenance_ = nullptr;
    return;
  }
  geometry_ = profile;
  expected_provenance_ = expected_provenance;
}

void SafeActuatorPolicy::setBootstrapContext(const CalibrationBootstrapContext& context) {
  bootstrap_ = context;
}

void SafeActuatorPolicy::reset() {
  // Advancing the epoch is what invalidates transactions a caller still holds:
  // they carry the old value and can never match again. Skipping 0 keeps a
  // zero-initialized transaction from matching a freshly reset policy.
  ++epoch_;
  if (epoch_ == 0) ++epoch_;
  outstanding_id_ = 0;
  limits_.clear();
  transforms_.clear();
  // A session does not survive a reset, and neither does a parked auxiliary:
  // after a fault the policy cannot know where the robot is standing.
  bootstrap_ = CalibrationBootstrapContext{};
  counters_.resets++;
  last_decision_ = WriteDecision::REJECT_TRANSACTION_STATE;
}

GeometryProvenanceTag SafeActuatorPolicy::currentGeometryTag() const {
  if (geometry_ == nullptr || expected_provenance_ == nullptr) return kNoGeometryProvenance;
  if (!geometry_->bound()) return kNoGeometryProvenance;
  // Not merely "some geometry is loaded" - it must be the geometry this build
  // expects. A profile that fails provenanceMatches() stamps and matches
  // nothing, so evidence cannot survive a model swap by accident.
  if (!geometry_->provenanceMatches(*expected_provenance_)) return kNoGeometryProvenance;
  return geometry_->provenanceTag();
}

WriteDecision SafeActuatorPolicy::record(WriteDecision decision) {
  last_decision_ = decision;
  return decision;
}

WriteDecision SafeActuatorPolicy::geometryPreconditions() const {
  // Shared by every geometry-authorised operation, in the order a reader would
  // ask them: is there a model, is it THIS robot's model, and is there a live
  // session to act under.
  if (geometry_ == nullptr || expected_provenance_ == nullptr || !geometry_->bound()) {
    return WriteDecision::REJECT_NO_GEOMETRY_PROFILE;
  }
  if (!geometry_->provenanceMatches(*expected_provenance_)) {
    return WriteDecision::REJECT_GEOMETRY_PROVENANCE;
  }
  if (!bootstrap_.session_active) return WriteDecision::REJECT_NO_CALIBRATION_SESSION;
  // A replay reproduces historical behaviour. It authorises nothing physical.
  if (!calibration::mayPromote(bootstrap_.origin)) {
    return WriteDecision::REJECT_NO_CALIBRATION_SESSION;
  }
  return WriteDecision::ACCEPT;
}

WriteDecision SafeActuatorPolicy::evaluateBootstrapEnvelope(
    const ActuatorCommand& command) const {
  const GeometryJointRecord* joint = geometry_->findJoint(command.joint);
  if (joint == nullptr) return WriteDecision::REJECT_UNKNOWN_GEOMETRY_JOINT;

  // The session's budget and the geometry's envelope are INDEPENDENT gates and
  // both must pass. Geometry says what is clear; the operator says how much of
  // it this session may use. Neither can grant what the other refuses.
  if (bootstrap_.direction_verify_tick_budget <= 0) {
    return WriteDecision::REJECT_NO_ENVELOPE_BUDGET;
  }
  if (command.delta_ticks == 0) return WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE;

  int32_t magnitude = command.delta_ticks;
  if (magnitude < 0) magnitude = -magnitude;
  if (magnitude > bootstrap_.direction_verify_tick_budget) {
    return WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE;
  }
  if (!geometry_->withinDirectionVerifyEnvelope(command.joint, command.delta_ticks)) {
    return WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE;
  }

  // Deliberately NO transform requirement: this diagnostic is commanded as a
  // raw tick delta and needs neither q0 nor a direction. Its safety comes from
  // the envelope being symmetric - the same tick magnitude is proven clear in
  // both directions, so the move is bounded whichever way the joint turns.
  return WriteDecision::ACCEPT;
}

WriteDecision SafeActuatorPolicy::evaluateEndpointPlan(const ActuatorCommand& command) const {
  const GeometryEndpointRecord* endpoint =
      geometry_->findEndpoint(command.endpoint_leg, command.endpoint_joint,
                              command.endpoint_side);
  if (endpoint == nullptr) return WriteDecision::REJECT_NO_ENDPOINT_PLAN;

  // THE door, and the only one. Sixteen of the twenty-four canonical endpoints
  // are DIAGNOSTIC - the mechanism contacts just beyond the declared URDF limit
  // - and eight of those still pass the 3 mm clearance policy. A clean
  // clearance on a diagnostic endpoint is not a permission to go there.
  if (!isExecutable(*endpoint)) return WriteDecision::REJECT_ENDPOINT_NOT_EXECUTABLE;

  const bool parking_planned = endpoint->parking == ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND;
  const bool parked_here =
      bootstrap_.auxiliary_parked && bootstrap_.parked_leg == command.endpoint_leg &&
      bootstrap_.parked_joint == command.endpoint_joint &&
      bootstrap_.parked_side == command.endpoint_side;

  if (command.operation == ActuatorOperation::CALIBRATION_AUXILIARY_MOVE) {
    if (!parking_planned) return WriteDecision::REJECT_UNEXPECTED_PARKING;
    if (!endpoint->has_auxiliary) return WriteDecision::REJECT_NO_ENDPOINT_PLAN;
    // The auxiliary must be the joint the compiler named, on both identity
    // axes, and it must not be the joint being calibrated.
    const GeometryJointRecord* moving = geometry_->findJoint(command.joint);
    if (moving == nullptr) return WriteDecision::REJECT_UNKNOWN_GEOMETRY_JOINT;
    if (moving->identity.leg != endpoint->auxiliary_leg ||
        moving->identity.joint != endpoint->auxiliary_joint) {
      return WriteDecision::REJECT_WRONG_AUXILIARY_JOINT;
    }
    // Exactly the parked pose, or exactly q=0 to unpark. Nothing in between:
    // the compiler validated those two configurations and no others.
    if (command.target_urad != endpoint->auxiliary_target && command.target_urad != 0) {
      return WriteDecision::REJECT_AUXILIARY_TARGET;
    }
    if (command.target_urad < moving->urdf_lower || command.target_urad > moving->urdf_upper) {
      return WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS;
    }
    if (transforms_.find(command.joint, currentGeometryTag()) == nullptr) {
      return transforms_.findAny(command.joint) != nullptr
                 ? WriteDecision::REJECT_EVIDENCE_GEOMETRY_MISMATCH
                 : WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM;
    }
    return WriteDecision::ACCEPT;
  }

  // --- CALIBRATION_CONTACT_PROBE ------------------------------------------
  // The probe moves the endpoint's own joint; a command naming a different one
  // is not this plan's probe.
  const GeometryJointRecord* moving = geometry_->findJoint(command.joint);
  if (moving == nullptr) return WriteDecision::REJECT_UNKNOWN_GEOMETRY_JOINT;
  if (moving->identity.leg != command.endpoint_leg ||
      moving->identity.joint != command.endpoint_joint) {
    return WriteDecision::REJECT_NO_ENDPOINT_PLAN;
  }

  if (parking_planned && !parked_here) return WriteDecision::REJECT_PARKING_REQUIRED;
  // The mirror image, and just as necessary: every direct path was validated
  // with ALL other joints at q=0, so a parked auxiliary invalidates a
  // NOT_NEEDED plan exactly as a missing one invalidates an obstructed plan.
  if (!parking_planned && bootstrap_.auxiliary_parked) {
    return WriteDecision::REJECT_UNEXPECTED_PARKING;
  }

  if (command.target_urad < moving->urdf_lower || command.target_urad > moving->urdf_upper) {
    return WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS;
  }
  // Travelling past the geometric contact is travelling into the mechanism.
  const MicroRad contact = endpoint->contact;
  if (endpoint->side == calibration::ContactSide::MIN_SIDE) {
    if (command.target_urad < contact) return WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS;
  } else if (command.target_urad > contact) {
    return WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS;
  }

  if (transforms_.find(command.joint, currentGeometryTag()) == nullptr) {
    return transforms_.findAny(command.joint) != nullptr
               ? WriteDecision::REJECT_EVIDENCE_GEOMETRY_MISMATCH
               : WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM;
  }
  return WriteDecision::ACCEPT;
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

  if (operationUsesAcceptedLimits(command.operation)) {
    const JointLimit* limit = limits_.find(command.joint, currentGeometryTag());
    if (limit == nullptr && limits_.findAny(command.joint) != nullptr) {
      // The bound exists and is well-formed, but it belongs to a different
      // model. It stays on record and is reportable; it authorises nothing.
      return WriteDecision::REJECT_EVIDENCE_GEOMETRY_MISMATCH;
    }
    // Today this is always the answer: MATDOG_JOINT_CALIBRATION.yaml holds no
    // accepted bound for any of the 12 leg joints. The safe result of a
    // missing bound is a refusal, never a fallback to a historical value - and
    // in particular never a fallback to the geometry profile, which describes
    // the MODEL and not the machine.
    if (limit == nullptr) return WriteDecision::REJECT_NO_ACCEPTED_LIMITS;
    if (!limit->contains(command.target_tick)) return WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS;
    return WriteDecision::ACCEPT;
  }

  if (operationUsesBootstrapEnvelope(command.operation) ||
      operationUsesEndpointPlan(command.operation)) {
    const WriteDecision preconditions = geometryPreconditions();
    if (preconditions != WriteDecision::ACCEPT) return preconditions;
    return operationUsesBootstrapEnvelope(command.operation)
               ? evaluateBootstrapEnvelope(command)
               : evaluateEndpointPlan(command);
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
    case ActuatorOperation::DIRECTION_VERIFY:           return "DIRECTION_VERIFY";
    case ActuatorOperation::CALIBRATION_AUXILIARY_MOVE: return "CALIBRATION_AUXILIARY_MOVE";
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
    case WriteDecision::REJECT_NO_GEOMETRY_PROFILE:  return "REJECT_NO_GEOMETRY_PROFILE";
    case WriteDecision::REJECT_GEOMETRY_PROVENANCE:  return "REJECT_GEOMETRY_PROVENANCE";
    case WriteDecision::REJECT_NO_CALIBRATION_SESSION: return "REJECT_NO_CALIBRATION_SESSION";
    case WriteDecision::REJECT_UNKNOWN_GEOMETRY_JOINT: return "REJECT_UNKNOWN_GEOMETRY_JOINT";
    case WriteDecision::REJECT_NO_ENVELOPE_BUDGET:   return "REJECT_NO_ENVELOPE_BUDGET";
    case WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE:
      return "REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE";
    case WriteDecision::REJECT_NO_ENDPOINT_PLAN:     return "REJECT_NO_ENDPOINT_PLAN";
    case WriteDecision::REJECT_ENDPOINT_NOT_EXECUTABLE:
      return "REJECT_ENDPOINT_NOT_EXECUTABLE";
    case WriteDecision::REJECT_PARKING_REQUIRED:     return "REJECT_PARKING_REQUIRED";
    case WriteDecision::REJECT_UNEXPECTED_PARKING:   return "REJECT_UNEXPECTED_PARKING";
    case WriteDecision::REJECT_WRONG_AUXILIARY_JOINT:
      return "REJECT_WRONG_AUXILIARY_JOINT";
    case WriteDecision::REJECT_AUXILIARY_TARGET:     return "REJECT_AUXILIARY_TARGET";
    case WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM:
      return "REJECT_NO_ACCEPTED_TRANSFORM";
    case WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS:
      return "REJECT_TARGET_OUTSIDE_URDF_LIMITS";
    case WriteDecision::REJECT_EVIDENCE_GEOMETRY_MISMATCH:
      return "REJECT_EVIDENCE_GEOMETRY_MISMATCH";
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
