#include "CalibrationDomain.h"

namespace matdog {
namespace calibration {

bool isKnownLeg(Leg leg) { return static_cast<uint8_t>(leg) < kLegCount; }
bool isKnownJointKind(JointKind joint) { return static_cast<uint8_t>(joint) < kJointKindCount; }
bool isKnownContactSide(ContactSide side) {
  return static_cast<uint8_t>(side) < kContactSideCount;
}

uint16_t contactProfileIndex(const ContactProfileKey& key) {
  // Fail closed: an invalid key gets the out-of-range sentinel, never a
  // clamped or wrapped index that would silently alias a real profile.
  if (!key.valid()) return kContactProfileCount;
  const uint16_t leg = static_cast<uint16_t>(key.leg);
  const uint16_t joint = static_cast<uint16_t>(key.joint);
  const uint16_t side = static_cast<uint16_t>(key.side);
  return static_cast<uint16_t>((leg * kJointKindCount + joint) * kContactSideCount + side);
}

bool contactProfileFromIndex(uint16_t index, ContactProfileKey* out) {
  if (out == nullptr || index >= kContactProfileCount) return false;
  const uint16_t side = index % kContactSideCount;
  const uint16_t rest = static_cast<uint16_t>(index / kContactSideCount);
  const uint16_t joint = rest % kJointKindCount;
  const uint16_t leg = static_cast<uint16_t>(rest / kJointKindCount);
  out->leg = static_cast<Leg>(leg);
  out->joint = static_cast<JointKind>(joint);
  out->side = static_cast<ContactSide>(side);
  return true;
}

namespace {

void appendBounded(char* out, size_t out_size, size_t& w, const char* text) {
  if (text == nullptr) return;
  while (*text != '\0' && w + 1 < out_size) out[w++] = *text++;
}

void appendUnsigned(char* out, size_t out_size, size_t& w, uint32_t value) {
  char digits[10];
  size_t n = 0;
  do {
    digits[n++] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
  } while (value != 0 && n < sizeof(digits));
  while (n > 0 && w + 1 < out_size) out[w++] = digits[--n];
}

}  // namespace

void formatProfileToken(const ContactProfileKey& key, uint8_t historical_motor_id,
                        char* out, size_t out_size) {
  if (out == nullptr || out_size == 0) return;
  size_t w = 0;
  if (!key.valid()) {
    appendBounded(out, out_size, w, "INVALID");
    out[w < out_size ? w : out_size - 1] = '\0';
    return;
  }
  // "{LEG}_{JOINT}_M{id}_{SIDE}" - matdog.rs::build_profile().
  appendBounded(out, out_size, w, toString(key.leg));
  appendBounded(out, out_size, w, "_");
  appendBounded(out, out_size, w, toString(key.joint));
  appendBounded(out, out_size, w, "_M");
  appendUnsigned(out, out_size, w, historical_motor_id);
  appendBounded(out, out_size, w, "_");
  appendBounded(out, out_size, w, toString(key.side));
  out[w < out_size ? w : out_size - 1] = '\0';
}

bool isContactEvidence(ContactState state) { return state == ContactState::CONTACT_CONFIRMED; }

bool isContactFailure(ContactState state) {
  return state == ContactState::EARLY_STALL || state == ContactState::HARD_ABORT;
}

bool isLegalEvidenceTransition(EvidenceState from, EvidenceState to) {
  // Rejection is reachable from any non-terminal state: a gate may refuse at
  // any point.
  if (to == EvidenceState::REJECTED) {
    return from != EvidenceState::REJECTED && from != EvidenceState::PROMOTED;
  }
  switch (from) {
    case EvidenceState::UNKNOWN:   return to == EvidenceState::MEASURED;
    case EvidenceState::MEASURED:  return to == EvidenceState::CANDIDATE;
    case EvidenceState::CANDIDATE: return to == EvidenceState::ACCEPTED;
    case EvidenceState::ACCEPTED:  return to == EvidenceState::PROMOTED;
    // Terminal. In particular PROMOTED is not a springboard back into the
    // pipeline - a new measurement starts a new record.
    case EvidenceState::PROMOTED:  return false;
    case EvidenceState::REJECTED:  return false;
  }
  return false;
}

bool isOperationalEvidence(EvidenceState state) { return state == EvidenceState::PROMOTED; }

bool mayPromote(CalibrationOrigin origin) {
  // THE safety property of this whole module. A replay of the LF V25 oracle
  // reproduces historical behaviour; it says nothing about the machine that
  // exists today, whose servos were all removed, re-provisioned and remounted
  // on 2026-08-27. Letting a replay reach PROMOTED would turn a regression
  // test into an authorisation to move the robot.
  return origin == CalibrationOrigin::LIVE_SESSION;
}

namespace {

// The oracle's linear order (matdog.rs::LfSessionState). UPPER and LOWER are
// proven before HIP is attempted - hardware_profile_allowed() refuses every
// isolated HIP profile precisely to enforce this.
constexpr CalibrationPhase kPhaseOrder[kCalibrationPhaseCount] = {
    CalibrationPhase::PREFLIGHT,        CalibrationPhase::INITIAL_RECOVERY,
    CalibrationPhase::PARKING,          CalibrationPhase::UPPER_MIN,
    CalibrationPhase::UPPER_MAX,        CalibrationPhase::UPPER_HORIZONTAL,
    CalibrationPhase::LOWER_MIN,        CalibrationPhase::LOWER_MAX,
    CalibrationPhase::LOWER_FOLDED,     CalibrationPhase::HIP_MIN,
    CalibrationPhase::HIP_MAX,          CalibrationPhase::DIAGNOSTICS,
    CalibrationPhase::RETURN_HIP,       CalibrationPhase::RETURN_LOWER_HELD,
    CalibrationPhase::RETURN_UPPER,     CalibrationPhase::RESTORE_PARKING,
    CalibrationPhase::CLEANUP,          CalibrationPhase::TORQUE_OFF,
};

bool phaseRank(CalibrationPhase phase, uint8_t* out_rank) {
  for (uint8_t i = 0; i < kCalibrationPhaseCount; ++i) {
    if (kPhaseOrder[i] == phase) {
      *out_rank = i;
      return true;
    }
  }
  return false;
}

}  // namespace

bool isLegalPhaseTransition(CalibrationPhase from, CalibrationPhase to) {
  uint8_t from_rank = 0;
  uint8_t to_rank = 0;
  if (!phaseRank(from, &from_rank) || !phaseRank(to, &to_rank)) return false;

  // TORQUE_OFF is terminal: it is the verified safe state on every exit path.
  if (from == CalibrationPhase::TORQUE_OFF) return false;

  // The normal path advances exactly one step. No skipping - that is what
  // would let HIP be reached without the UPPER/LOWER proof.
  if (to_rank == from_rank + 1) return true;

  // Any active phase may jump straight into the restore sequence or to the
  // terminal safe state. A failure must never have to walk the happy path to
  // become safe.
  if (to == CalibrationPhase::TORQUE_OFF) return true;
  if (to == CalibrationPhase::RETURN_HIP && from_rank < to_rank) return true;

  return false;
}

bool isMeasurementPhase(CalibrationPhase phase) {
  switch (phase) {
    case CalibrationPhase::UPPER_MIN:
    case CalibrationPhase::UPPER_MAX:
    case CalibrationPhase::LOWER_MIN:
    case CalibrationPhase::LOWER_MAX:
    case CalibrationPhase::HIP_MIN:
    case CalibrationPhase::HIP_MAX:
      return true;
    default:
      return false;
  }
}

bool isRestorePhase(CalibrationPhase phase) {
  switch (phase) {
    case CalibrationPhase::RETURN_HIP:
    case CalibrationPhase::RETURN_LOWER_HELD:
    case CalibrationPhase::RETURN_UPPER:
    case CalibrationPhase::RESTORE_PARKING:
      return true;
    default:
      return false;
  }
}

bool isTerminalPhase(CalibrationPhase phase) { return phase == CalibrationPhase::TORQUE_OFF; }

bool requiresRestore(CalibrationFailure failure) {
  // Nothing to restore if nothing was ever engaged, and a lost authority must
  // NOT trigger a restore sequence: restore implies commanding joints, and
  // without authority that is exactly what must not happen. Torque off remains
  // required either way - it is the one de-escalation that is never arbitrated.
  switch (failure) {
    case CalibrationFailure::NONE:
    case CalibrationFailure::AUTHORITY_LOST:
    case CalibrationFailure::STALE_CALIBRATION_REFUSED:
      return false;
    default:
      return true;
  }
}

RestorePlan restorePlanFor(CalibrationPhase phase, CalibrationFailure failure) {
  RestorePlan plan{};
  plan.cause = failure;
  plan.torque_off_required = true;  // verified on EVERY exit, success or failure
  plan.required = requiresRestore(failure) && !isTerminalPhase(phase);
  plan.resume_from = plan.required ? CalibrationPhase::RETURN_HIP : CalibrationPhase::TORQUE_OFF;
  return plan;
}

const char* toString(Leg leg) {
  switch (leg) {
    case Leg::LF: return "LF";
    case Leg::RF: return "RF";
    case Leg::RH: return "RH";
    case Leg::LH: return "LH";
  }
  return "UNKNOWN";
}

const char* toString(JointKind joint) {
  switch (joint) {
    case JointKind::HIP:   return "HIP";
    case JointKind::UPPER: return "UPPER";
    case JointKind::LOWER: return "LOWER";
  }
  return "UNKNOWN";
}

const char* toString(ContactSide side) {
  // The oracle's token labels. Deliberately "MIN"/"MAX" even though the
  // enumerators cannot be - see the note in the header.
  switch (side) {
    case ContactSide::MIN_SIDE: return "MIN";
    case ContactSide::MAX_SIDE: return "MAX";
  }
  return "UNKNOWN";
}

const char* toString(ContactState state) {
  switch (state) {
    case ContactState::FREE_MOTION:       return "FREE_MOTION";
    case ContactState::CONTACT_SUSPECTED: return "CONTACT_SUSPECTED";
    case ContactState::CONTACT_CONFIRMED: return "CONTACT_CONFIRMED";
    case ContactState::EARLY_STALL:       return "EARLY_STALL";
    case ContactState::HARD_ABORT:        return "HARD_ABORT";
  }
  return "UNKNOWN";
}

const char* toString(EvidenceState state) {
  switch (state) {
    case EvidenceState::UNKNOWN:   return "UNKNOWN";
    case EvidenceState::MEASURED:  return "MEASURED";
    case EvidenceState::CANDIDATE: return "CANDIDATE";
    case EvidenceState::ACCEPTED:  return "ACCEPTED";
    case EvidenceState::PROMOTED:  return "PROMOTED";
    case EvidenceState::REJECTED:  return "REJECTED";
  }
  return "UNKNOWN_STATE";
}

const char* toString(CalibrationOrigin origin) {
  switch (origin) {
    case CalibrationOrigin::NONE:              return "NONE";
    case CalibrationOrigin::HISTORICAL_REPLAY: return "HISTORICAL_REPLAY";
    case CalibrationOrigin::LIVE_SESSION:      return "LIVE_SESSION";
  }
  return "UNKNOWN";
}

const char* toString(CalibrationPhase phase) {
  switch (phase) {
    case CalibrationPhase::PREFLIGHT:         return "PREFLIGHT";
    case CalibrationPhase::INITIAL_RECOVERY:  return "INITIAL_RECOVERY";
    case CalibrationPhase::PARKING:           return "PARKING";
    case CalibrationPhase::UPPER_MIN:         return "UPPER_MIN";
    case CalibrationPhase::UPPER_MAX:         return "UPPER_MAX";
    case CalibrationPhase::UPPER_HORIZONTAL:  return "UPPER_HORIZONTAL";
    case CalibrationPhase::LOWER_MIN:         return "LOWER_MIN";
    case CalibrationPhase::LOWER_MAX:         return "LOWER_MAX";
    case CalibrationPhase::LOWER_FOLDED:      return "LOWER_FOLDED";
    case CalibrationPhase::HIP_MIN:           return "HIP_MIN";
    case CalibrationPhase::HIP_MAX:           return "HIP_MAX";
    case CalibrationPhase::DIAGNOSTICS:       return "DIAGNOSTICS";
    case CalibrationPhase::RETURN_HIP:        return "RETURN_HIP";
    case CalibrationPhase::RETURN_LOWER_HELD: return "RETURN_LOWER_HELD";
    case CalibrationPhase::RETURN_UPPER:      return "RETURN_UPPER";
    case CalibrationPhase::RESTORE_PARKING:   return "RESTORE_PARKING";
    case CalibrationPhase::CLEANUP:           return "CLEANUP";
    case CalibrationPhase::TORQUE_OFF:        return "TORQUE_OFF";
  }
  return "UNKNOWN";
}

const char* toString(CalibrationFailure failure) {
  switch (failure) {
    case CalibrationFailure::NONE:                      return "NONE";
    case CalibrationFailure::HARD_CURRENT_ABORT:        return "HARD_CURRENT_ABORT";
    case CalibrationFailure::EARLY_STALL:               return "EARLY_STALL";
    case CalibrationFailure::CONTACT_WITNESS_REJECTED:  return "CONTACT_WITNESS_REJECTED";
    case CalibrationFailure::REPEATABILITY_EXCEEDED:    return "REPEATABILITY_EXCEEDED";
    case CalibrationFailure::MOTION_TIMEOUT:            return "MOTION_TIMEOUT";
    case CalibrationFailure::TELEMETRY_STALE:           return "TELEMETRY_STALE";
    case CalibrationFailure::STATIC_JOINT_MOVED:        return "STATIC_JOINT_MOVED";
    case CalibrationFailure::AFFINE_GATE_REJECTED:      return "AFFINE_GATE_REJECTED";
    case CalibrationFailure::ISOLATED_HIP_BLOCKED:      return "ISOLATED_HIP_BLOCKED";
    case CalibrationFailure::OPERATOR_ABORT:            return "OPERATOR_ABORT";
    case CalibrationFailure::AUTHORITY_LOST:            return "AUTHORITY_LOST";
    case CalibrationFailure::STALE_CALIBRATION_REFUSED: return "STALE_CALIBRATION_REFUSED";
  }
  return "UNKNOWN";
}

const char* toString(DirectionSource source) {
  switch (source) {
    case DirectionSource::UNKNOWN:          return "UNKNOWN";
    case DirectionSource::SPEC_CONSTANT:    return "SPEC_CONSTANT";
    case DirectionSource::MEASURED_WITNESS: return "MEASURED_WITNESS";
    case DirectionSource::CONFLICTING:      return "CONFLICTING";
  }
  return "UNKNOWN_SOURCE";
}

const char* toString(Q0Estimator estimator) {
  switch (estimator) {
    case Q0Estimator::NONE:        return "NONE";
    case Q0Estimator::FIXED_SCALE: return "FIXED_SCALE";
    case Q0Estimator::AFFINE:      return "AFFINE";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
