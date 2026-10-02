#include "CalibrationPersistenceState.h"

namespace matdog {
namespace calibration {

const char* toString(PersistenceClass cls) {
  switch (cls) {
    case PersistenceClass::IO_ERROR:                    return "IO_ERROR";
    case PersistenceClass::NEVER_INITIALIZED_OR_ERASED: return "NEVER_INITIALIZED_OR_ERASED";
    case PersistenceClass::NOTHING_CONFIRMED:           return "NOTHING_CONFIRMED";
    case PersistenceClass::CONSISTENT:                  return "CONSISTENT";
    case PersistenceClass::PENDING_RECORD_ABSENT:       return "PENDING_RECORD_ABSENT";
    case PersistenceClass::PENDING_RECORD_PRESENT:      return "PENDING_RECORD_PRESENT";
    case PersistenceClass::CONFIRMED_GENERATION_LOST:   return "CONFIRMED_GENERATION_LOST";
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
    case ReconciliationAction::DECLARE_NOTHING_CONFIRMED: return "DECLARE_NOTHING_CONFIRMED";
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
    case PersistenceClass::CONFIRMED_GENERATION_LOST:
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
    case PersistenceClass::NOTHING_CONFIRMED:
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
                    a->cls == PersistenceClass::NOTHING_CONFIRMED ||
                    a->cls == PersistenceClass::CONSISTENT;
  a->record_available = a->cls == PersistenceClass::CONSISTENT;
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
  const uint32_t C = m.completed_generation;
  const uint32_t B = m.begun_generation;
  const bool pending = m.state == SaveMarkerState::PENDING;
  a.confirmed_generation = C;
  a.pending_generation = pending ? B : 0;

  const int confirmed_idx = C > 0 ? validSlotAt(in, C) : -1;
  a.confirmed_record_intact = confirmed_idx >= 0;
  if (confirmed_idx >= 0) a.confirmed_slot = static_cast<CalibrationSlot>(confirmed_idx);

  // Two valid records with one generation only matter when that generation is
  // one the marker speaks about; elsewhere (older, discarded) they are leftovers.
  const uint32_t dup = in.slot[0].generation_hint;
  if (in.slot[0].state == SlotState::VALID && in.slot[1].state == SlotState::VALID &&
      dup == in.slot[1].generation_hint && ((C > 0 && dup == C) || (pending && dup == B))) {
    a.cls = PersistenceClass::RECORD_GENERATION_CONFLICT;
    finish(&a, in);
    return a;
  }

  // A generation above the confirmed one is acceptable only as: the pending one,
  // or (marker COMPLETED) one explicitly discarded by an earlier reconciliation,
  // which leaves begun_generation >= it.
  auto acceptableAboveConfirmed = [&](uint32_t g) {
    return pending ? g == B : g <= B;
  };

  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].state == SlotState::VALID && in.slot[i].generation_hint > C &&
        !acceptableAboveConfirmed(in.slot[i].generation_hint)) {
      a.cls = PersistenceClass::RECORD_AHEAD_OF_MARKER;
      finish(&a, in);
      return a;
    }
  }
  for (int i = 0; i < kCalibrationSlotCount; ++i) {
    if (in.slot[i].state != SlotState::INCOMPATIBLE) continue;
    const uint32_t g = in.slot[i].generation_hint;
    const bool above = g > C && !(pending ? false : g <= B);
    const bool is_confirmed_gen = C > 0 && g == C && confirmed_idx < 0;
    if (above || is_confirmed_gen) {
      a.cls = PersistenceClass::RECORD_INCOMPATIBLE;
      finish(&a, in);
      return a;
    }
  }

  if (C > 0 && !a.confirmed_record_intact) {
    for (int i = 0; i < kCalibrationSlotCount; ++i) {
      if (in.slot[i].state == SlotState::VALID && in.slot[i].generation_hint < C) {
        a.older_record_survives = true;
      }
    }
    a.cls = PersistenceClass::CONFIRMED_GENERATION_LOST;
    finish(&a, in);
    return a;
  }

  if (pending) {
    a.cls = validSlotAt(in, B) >= 0 ? PersistenceClass::PENDING_RECORD_PRESENT
                                    : PersistenceClass::PENDING_RECORD_ABSENT;
  } else {
    a.cls = C == 0 ? PersistenceClass::NOTHING_CONFIRMED : PersistenceClass::CONSISTENT;
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
  m.state = SaveMarkerState::COMPLETED;
  switch (action) {
    case ReconciliationAction::ADOPT_VALID_RECORD:
      if ((a.allowed_actions & kReconcileAdoptBit) == 0) return ReconciliationStatus::ACTION_NOT_ALLOWED;
      if (generation == 0 || validSlotAt(in, generation) < 0) return ReconciliationStatus::GENERATION_NOT_VALID;
      // Two valid records with this generation: adopting it would be ambiguous.
      if (in.slot[0].state == SlotState::VALID && in.slot[1].state == SlotState::VALID &&
          in.slot[0].generation_hint == generation && in.slot[1].generation_hint == generation) {
        return ReconciliationStatus::GENERATION_NOT_VALID;
      }
      m.completed_generation = generation;
      if (generation > begun) begun = generation;
      break;
    case ReconciliationAction::DECLARE_NOTHING_CONFIRMED:
      if ((a.allowed_actions & kReconcileDeclareBit) == 0) return ReconciliationStatus::ACTION_NOT_ALLOWED;
      m.completed_generation = 0;
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

}  // namespace calibration
}  // namespace matdog
