#include "CalibrationPersistenceState.h"

namespace matdog {
namespace calibration {

const char* toString(PersistenceClass cls) {
  switch (cls) {
    case PersistenceClass::IO_ERROR:                    return "IO_ERROR";
    case PersistenceClass::NEVER_INITIALIZED_OR_ERASED: return "NEVER_INITIALIZED_OR_ERASED";
    case PersistenceClass::NOTHING_ACKNOWLEDGED:           return "NOTHING_ACKNOWLEDGED";
    case PersistenceClass::CONSISTENT:                  return "CONSISTENT";
    case PersistenceClass::PENDING_RECORD_ABSENT:       return "PENDING_RECORD_ABSENT";
    case PersistenceClass::PENDING_RECORD_PRESENT:      return "PENDING_RECORD_PRESENT";
    case PersistenceClass::AWAITING_ACK:                return "AWAITING_ACK";
    case PersistenceClass::AWAITING_ACK_RECORD_LOST:    return "AWAITING_ACK_RECORD_LOST";
    case PersistenceClass::ACKNOWLEDGED_GENERATION_LOST:   return "ACKNOWLEDGED_GENERATION_LOST";
    case PersistenceClass::RECORD_AHEAD_OF_MARKER:      return "RECORD_AHEAD_OF_MARKER";
    case PersistenceClass::RECORD_GENERATION_CONFLICT:  return "RECORD_GENERATION_CONFLICT";
    case PersistenceClass::MARKER_MISSING:              return "MARKER_MISSING";
    case PersistenceClass::MARKER_CORRUPT:              return "MARKER_CORRUPT";
    case PersistenceClass::MARKER_INCOMPATIBLE:         return "MARKER_INCOMPATIBLE";
    case PersistenceClass::RECORD_INCOMPATIBLE:         return "RECORD_INCOMPATIBLE";
  }
  return "UNKNOWN";
}

const char* toString(ReconciliationAction action) {
  switch (action) {
    case ReconciliationAction::ADOPT_VALID_RECORD:        return "ADOPT_VALID_RECORD";
    case ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED: return "DECLARE_NOTHING_ACKNOWLEDGED";
  }
  return "UNKNOWN";
}

const char* toString(AckPlanStatus status) {
  switch (status) {
    case AckPlanStatus::OK:                      return "OK";
    case AckPlanStatus::ALREADY_ACKNOWLEDGED:    return "ALREADY_ACKNOWLEDGED";
    case AckPlanStatus::BAD_ARGUMENT:            return "BAD_ARGUMENT";
    case AckPlanStatus::STORAGE_UNUSABLE:        return "STORAGE_UNUSABLE";
    case AckPlanStatus::NOT_AWAITING:            return "NOT_AWAITING";
    case AckPlanStatus::GENERATION_MISMATCH:     return "GENERATION_MISMATCH";
    case AckPlanStatus::RECORD_NOT_VALID:        return "RECORD_NOT_VALID";
    case AckPlanStatus::RECONCILIATION_REQUIRED: return "RECONCILIATION_REQUIRED";
  }
  return "UNKNOWN";
}

const char* toString(ReconciliationStatus status) {
  switch (status) {
    case ReconciliationStatus::OK:                   return "OK";
    case ReconciliationStatus::NOT_REQUIRED:         return "NOT_REQUIRED";
    case ReconciliationStatus::ACTION_NOT_ALLOWED:   return "ACTION_NOT_ALLOWED";
    case ReconciliationStatus::GENERATION_NOT_VALID: return "GENERATION_NOT_VALID";
    case ReconciliationStatus::BAD_ARGUMENT:         return "BAD_ARGUMENT";
  }
  return "UNKNOWN";
}

namespace {

bool needsReconciliation(PersistenceClass cls) {
  switch (cls) {
    case PersistenceClass::PENDING_RECORD_ABSENT:
    case PersistenceClass::PENDING_RECORD_PRESENT:
    case PersistenceClass::AWAITING_ACK:
    case PersistenceClass::AWAITING_ACK_RECORD_LOST:
    case PersistenceClass::ACKNOWLEDGED_GENERATION_LOST:
    case PersistenceClass::RECORD_AHEAD_OF_MARKER:
    case PersistenceClass::RECORD_GENERATION_CONFLICT:
    case PersistenceClass::MARKER_MISSING:
    case PersistenceClass::MARKER_CORRUPT:
    case PersistenceClass::RECORD_INCOMPATIBLE:
      return true;
    // IO_ERROR: nothing is known, nothing can be reconciled. MARKER_INCOMPATIBLE:
    // another schema owns the marker; this build must not overwrite it.
    case PersistenceClass::IO_ERROR:
    case PersistenceClass::MARKER_INCOMPATIBLE:
    case PersistenceClass::NEVER_INITIALIZED_OR_ERASED:
    case PersistenceClass::NOTHING_ACKNOWLEDGED:
    case PersistenceClass::CONSISTENT:
      return false;
  }
  return false;
}

bool anyValid(const PersistenceInputs& in) {
  return in.slot[0].state == SlotState::VALID || in.slot[1].state == SlotState::VALID;
}

// Index of the valid slot holding `generation`, or -1.
int validSlotAt(const PersistenceInputs& in, uint32_t generation) {
  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].state == SlotState::VALID && in.slot[i].generation_hint == generation) return i;
  }
  return -1;
}

void finish(PersistenceAssessment* a, const PersistenceInputs& in) {
  a->reconciliation_required = needsReconciliation(a->cls);
  a->save_allowed = a->cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED ||
                    a->cls == PersistenceClass::NOTHING_ACKNOWLEDGED ||
                    a->cls == PersistenceClass::CONSISTENT;
  a->record_available = a->cls == PersistenceClass::CONSISTENT;
  a->ack_allowed = a->cls == PersistenceClass::AWAITING_ACK;
  a->allowed_actions = 0;
  if (a->reconciliation_required) {
    a->allowed_actions |= kReconcileDeclareBit;
    // Two valid records with one generation make "adopt generation g" ambiguous.
    if (anyValid(in) && a->cls != PersistenceClass::RECORD_GENERATION_CONFLICT) {
      a->allowed_actions |= kReconcileAdoptBit;
    }
  }
}

}  // namespace

PersistenceAssessment classifyPersistence(const PersistenceInputs& in) {
  PersistenceAssessment a;  // IO_ERROR until proven otherwise: fail closed

  bool unreadable = in.marker.state == MarkerObservation::UNREAD ||
                    in.marker.state == MarkerObservation::IO_ERROR;
  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].state == SlotState::UNREAD || in.slot[i].state == SlotState::IO_ERROR) {
      unreadable = true;
    }
  }
  if (unreadable) {
    a.cls = PersistenceClass::IO_ERROR;
    finish(&a, in);
    return a;
  }

  switch (in.marker.state) {
    case MarkerObservation::ABSENT:
      a.cls = (in.slot[0].state == SlotState::ABSENT && in.slot[1].state == SlotState::ABSENT)
                  ? PersistenceClass::NEVER_INITIALIZED_OR_ERASED
                  : PersistenceClass::MARKER_MISSING;
      finish(&a, in);
      return a;
    case MarkerObservation::CORRUPT:
      a.cls = PersistenceClass::MARKER_CORRUPT;
      finish(&a, in);
      return a;
    case MarkerObservation::INCOMPATIBLE:
      a.cls = PersistenceClass::MARKER_INCOMPATIBLE;
      finish(&a, in);
      return a;
    case MarkerObservation::VALID:
      break;
    case MarkerObservation::UNREAD:
    case MarkerObservation::IO_ERROR:
      finish(&a, in);  // unreachable: handled above
      return a;
  }

  const SaveMarker& m = in.marker.marker;
  const uint32_t A = m.acknowledged_generation;
  const uint32_t B = m.begun_generation;
  const bool pending = m.state == SaveMarkerState::PENDING;
  const bool awaiting = m.state == SaveMarkerState::AWAITING_ACK;
  const bool in_flight = pending || awaiting;  // a generation above A is expected: B
  a.acknowledged_generation = A;
  a.pending_generation = pending ? B : 0;
  a.awaiting_generation = awaiting ? B : 0;

  const int acknowledged_idx = A > 0 ? validSlotAt(in, A) : -1;
  a.acknowledged_record_intact = acknowledged_idx >= 0;
  if (acknowledged_idx >= 0) a.acknowledged_slot = static_cast<CalibrationSlot>(acknowledged_idx);

  // Two valid records with one generation only matter when that generation is
  // one the marker speaks about; elsewhere (older, discarded) they are leftovers.
  const uint32_t dup = in.slot[0].generation_hint;
  if (in.slot[0].state == SlotState::VALID && in.slot[1].state == SlotState::VALID &&
      dup == in.slot[1].generation_hint && ((A > 0 && dup == A) || (in_flight && dup == B))) {
    a.cls = PersistenceClass::RECORD_GENERATION_CONFLICT;
    finish(&a, in);
    return a;
  }

  // A generation above the acknowledged one is acceptable only if the marker has
  // already begun it: the in-flight one (== B) or one explicitly discarded by an
  // earlier reconciliation (< B; generations are never reused, so it cannot be
  // mistaken for the in-flight one). Anything above begun_generation is a record
  // the marker knows nothing about.
  auto acceptableAboveAcknowledged = [&](uint32_t g) { return g <= B; };

  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].state == SlotState::VALID && in.slot[i].generation_hint > A &&
        !acceptableAboveAcknowledged(in.slot[i].generation_hint)) {
      a.cls = PersistenceClass::RECORD_AHEAD_OF_MARKER;
      finish(&a, in);
      return a;
    }
  }
  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].state != SlotState::INCOMPATIBLE) continue;
    const uint32_t g = in.slot[i].generation_hint;
    const bool above = g > A && (in_flight || g > B);
    const bool is_acknowledged_gen = A > 0 && g == A && acknowledged_idx < 0;
    if (above || is_acknowledged_gen) {
      a.cls = PersistenceClass::RECORD_INCOMPATIBLE;
      finish(&a, in);
      return a;
    }
  }

  // Priority over every in-flight state: a lost acknowledged generation is never
  // hidden behind a newer, unacknowledged one.
  if (A > 0 && !a.acknowledged_record_intact) {
    for (int i = 0; i < kCalibrationSlotCount; ++i) {
      if (in.slot[i].state == SlotState::VALID && in.slot[i].generation_hint < A) {
        a.older_record_survives = true;
      }
    }
    a.cls = PersistenceClass::ACKNOWLEDGED_GENERATION_LOST;
    finish(&a, in);
    return a;
  }

  if (pending) {
    a.cls = validSlotAt(in, B) >= 0 ? PersistenceClass::PENDING_RECORD_PRESENT
                                    : PersistenceClass::PENDING_RECORD_ABSENT;
  } else if (awaiting) {
    a.cls = validSlotAt(in, B) >= 0 ? PersistenceClass::AWAITING_ACK
                                    : PersistenceClass::AWAITING_ACK_RECORD_LOST;
  } else {
    a.cls = A == 0 ? PersistenceClass::NOTHING_ACKNOWLEDGED : PersistenceClass::CONSISTENT;
  }
  finish(&a, in);
  return a;
}

ReconciliationStatus planReconciliation(const PersistenceInputs& in, ReconciliationAction action,
                                        uint32_t generation, SaveMarker* out) {
  if (out == nullptr) return ReconciliationStatus::BAD_ARGUMENT;
  const PersistenceAssessment a = classifyPersistence(in);
  if (!a.reconciliation_required) return ReconciliationStatus::NOT_REQUIRED;

  uint32_t begun = 0;
  if (in.marker.state == MarkerObservation::VALID) begun = in.marker.marker.begun_generation;
  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].generation_hint > begun) begun = in.slot[i].generation_hint;
  }

  SaveMarker m;
  m.state = SaveMarkerState::IDLE;
  switch (action) {
    case ReconciliationAction::ADOPT_VALID_RECORD:
      if ((a.allowed_actions & kReconcileAdoptBit) == 0) return ReconciliationStatus::ACTION_NOT_ALLOWED;
      if (generation == 0 || validSlotAt(in, generation) < 0) return ReconciliationStatus::GENERATION_NOT_VALID;
      // Two valid records with this generation: adopting it would be ambiguous.
      if (in.slot[0].state == SlotState::VALID && in.slot[1].state == SlotState::VALID &&
          in.slot[0].generation_hint == generation && in.slot[1].generation_hint == generation) {
        return ReconciliationStatus::GENERATION_NOT_VALID;
      }
      m.acknowledged_generation = generation;
      if (generation > begun) begun = generation;
      break;
    case ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED:
      if ((a.allowed_actions & kReconcileDeclareBit) == 0) return ReconciliationStatus::ACTION_NOT_ALLOWED;
      m.acknowledged_generation = 0;
      if (begun == 0) begun = 1;
      break;
    default:
      return ReconciliationStatus::ACTION_NOT_ALLOWED;
  }
  m.begun_generation = begun;
  if (validateSaveMarker(m) != SaveMarkerStatus::OK) return ReconciliationStatus::BAD_ARGUMENT;
  *out = m;
  return ReconciliationStatus::OK;
}

AckPlanStatus planAcknowledgment(const PersistenceInputs& in, uint32_t generation,
                                 SaveMarker* out) {
  if (out == nullptr || generation == 0) return AckPlanStatus::BAD_ARGUMENT;
  const PersistenceAssessment a = classifyPersistence(in);
  switch (a.cls) {
    case PersistenceClass::IO_ERROR:
      return AckPlanStatus::STORAGE_UNUSABLE;
    case PersistenceClass::AWAITING_ACK: {
      if (generation != a.awaiting_generation) return AckPlanStatus::GENERATION_MISMATCH;
      SaveMarker m;
      m.state = SaveMarkerState::IDLE;
      m.acknowledged_generation = generation;
      m.begun_generation = generation;
      if (validateSaveMarker(m) != SaveMarkerStatus::OK) return AckPlanStatus::BAD_ARGUMENT;
      *out = m;
      return AckPlanStatus::OK;
    }
    case PersistenceClass::AWAITING_ACK_RECORD_LOST:
      return generation == a.awaiting_generation ? AckPlanStatus::RECORD_NOT_VALID
                                                 : AckPlanStatus::GENERATION_MISMATCH;
    case PersistenceClass::PENDING_RECORD_PRESENT:
    case PersistenceClass::PENDING_RECORD_ABSENT:
      // The SAVE never reached its verified marker: that record is incomplete.
      return generation == a.pending_generation ? AckPlanStatus::RECORD_NOT_VALID
                                               : AckPlanStatus::GENERATION_MISMATCH;
    case PersistenceClass::CONSISTENT:
      return generation == a.acknowledged_generation ? AckPlanStatus::ALREADY_ACKNOWLEDGED
                                                     : AckPlanStatus::NOT_AWAITING;
    case PersistenceClass::NEVER_INITIALIZED_OR_ERASED:
    case PersistenceClass::NOTHING_ACKNOWLEDGED:
      return AckPlanStatus::NOT_AWAITING;
    case PersistenceClass::RECORD_INCOMPATIBLE:
      for (int i = 0; i < kCalibrationSlotCount; ++i) {
        if (in.slot[i].state == SlotState::INCOMPATIBLE && in.slot[i].generation_hint == generation) {
          return AckPlanStatus::RECORD_NOT_VALID;
        }
      }
      return AckPlanStatus::RECONCILIATION_REQUIRED;
    case PersistenceClass::ACKNOWLEDGED_GENERATION_LOST:
    case PersistenceClass::RECORD_AHEAD_OF_MARKER:
    case PersistenceClass::RECORD_GENERATION_CONFLICT:
    case PersistenceClass::MARKER_MISSING:
    case PersistenceClass::MARKER_CORRUPT:
    case PersistenceClass::MARKER_INCOMPATIBLE:
      return AckPlanStatus::RECONCILIATION_REQUIRED;
  }
  return AckPlanStatus::RECONCILIATION_REQUIRED;
}

}  // namespace calibration
}  // namespace matdog
