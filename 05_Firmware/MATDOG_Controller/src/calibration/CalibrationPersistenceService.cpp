#include "CalibrationPersistenceService.h"

#include <string.h>

namespace matdog {
namespace calibration {

namespace {

PersistenceVerdict verdictOf(const LoadResult& r) {
  const PersistenceAssessment& a = r.assessment;
  switch (a.cls) {
    case PersistenceClass::IO_ERROR:
      return PersistenceVerdict::STORAGE_ERROR;
    case PersistenceClass::NEVER_INITIALIZED_OR_ERASED:
    case PersistenceClass::NOTHING_ACKNOWLEDGED:
      return PersistenceVerdict::NO_RECORD;
    case PersistenceClass::CONSISTENT:
      return r.status == LoadStatus::OK ? PersistenceVerdict::VALID_ACKNOWLEDGED
                                        : PersistenceVerdict::STORAGE_ERROR;
    case PersistenceClass::PENDING_RECORD_ABSENT:
    case PersistenceClass::PENDING_RECORD_PRESENT:
      return PersistenceVerdict::SAVE_PENDING;
    case PersistenceClass::AWAITING_ACK:
      return PersistenceVerdict::AWAITING_ACK;
    case PersistenceClass::AWAITING_ACK_RECORD_LOST:
      return PersistenceVerdict::AWAITING_ACK_RECORD_LOST;
    case PersistenceClass::ACKNOWLEDGED_GENERATION_LOST:
      for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) {
        if (r.report[i].state == SlotState::CORRUPT) return PersistenceVerdict::RECORD_CORRUPT;
      }
      return PersistenceVerdict::ACKNOWLEDGED_GENERATION_LOST;
    case PersistenceClass::MARKER_MISSING:
      return PersistenceVerdict::MARKER_MISSING;
    case PersistenceClass::MARKER_CORRUPT:
      return PersistenceVerdict::MARKER_CORRUPT;
    case PersistenceClass::MARKER_INCOMPATIBLE:
      return PersistenceVerdict::MARKER_INCOMPATIBLE;
    case PersistenceClass::RECORD_INCOMPATIBLE:
      return PersistenceVerdict::RECORD_INCOMPATIBLE;
    case PersistenceClass::RECORD_AHEAD_OF_MARKER:
    case PersistenceClass::RECORD_GENERATION_CONFLICT:
      return PersistenceVerdict::RECONCILIATION_REQUIRED;
  }
  return PersistenceVerdict::STORAGE_ERROR;
}

bool isUncertainAck(AckStatus s) {
  return s == AckStatus::MARKER_WRITE_FAILED || s == AckStatus::MARKER_VERIFY_FAILED;
}

}  // namespace

const char* toString(PersistenceVerdict verdict) {
  switch (verdict) {
    case PersistenceVerdict::NOT_RUN:                      return "NOT_RUN";
    case PersistenceVerdict::NVS_UNAVAILABLE:              return "NVS_UNAVAILABLE";
    case PersistenceVerdict::STORAGE_ERROR:                return "STORAGE_ERROR";
    case PersistenceVerdict::NO_RECORD:                    return "NO_RECORD";
    case PersistenceVerdict::VALID_ACKNOWLEDGED:           return "VALID_ACKNOWLEDGED";
    case PersistenceVerdict::SAVE_PENDING:                 return "SAVE_PENDING";
    case PersistenceVerdict::AWAITING_ACK:                 return "AWAITING_ACK";
    case PersistenceVerdict::AWAITING_ACK_RECORD_LOST:     return "AWAITING_ACK_RECORD_LOST";
    case PersistenceVerdict::ACKNOWLEDGED_GENERATION_LOST: return "ACKNOWLEDGED_GENERATION_LOST";
    case PersistenceVerdict::RECORD_CORRUPT:               return "RECORD_CORRUPT";
    case PersistenceVerdict::MARKER_MISSING:               return "MARKER_MISSING";
    case PersistenceVerdict::MARKER_CORRUPT:               return "MARKER_CORRUPT";
    case PersistenceVerdict::MARKER_INCOMPATIBLE:          return "MARKER_INCOMPATIBLE";
    case PersistenceVerdict::RECORD_INCOMPATIBLE:          return "RECORD_INCOMPATIBLE";
    case PersistenceVerdict::RECONCILIATION_REQUIRED:      return "RECONCILIATION_REQUIRED";
  }
  return "UNKNOWN";
}

const char* toString(MarkerObservation observation) {
  switch (observation) {
    case MarkerObservation::UNREAD:       return "UNREAD";
    case MarkerObservation::ABSENT:       return "ABSENT";
    case MarkerObservation::IO_ERROR:     return "IO_ERROR";
    case MarkerObservation::CORRUPT:      return "CORRUPT";
    case MarkerObservation::INCOMPATIBLE: return "INCOMPATIBLE";
    case MarkerObservation::VALID:        return "VALID";
  }
  return "UNKNOWN";
}

const char* toString(StorageIoStatus status) {
  switch (status) {
    case StorageIoStatus::OK:               return "OK";
    case StorageIoStatus::ABSENT:           return "ABSENT";
    case StorageIoStatus::IO_ERROR:         return "IO_ERROR";
    case StorageIoStatus::NO_SPACE:         return "NO_SPACE";
    case StorageIoStatus::BUFFER_TOO_SMALL: return "BUFFER_TOO_SMALL";
    case StorageIoStatus::NOT_MODIFIED:     return "NOT_MODIFIED";
  }
  return "UNKNOWN";
}

const char* toString(ServiceGuard guard) {
  switch (guard) {
    case ServiceGuard::OK:             return "OK";
    case ServiceGuard::NVS_NOT_READY:  return "NVS_NOT_READY";
    case ServiceGuard::WRITES_BLOCKED: return "WRITES_BLOCKED";
  }
  return "UNKNOWN";
}

const char* toString(ReconcileStatus status) {
  switch (status) {
    case ReconcileStatus::OK:                                  return "OK";
    case ReconcileStatus::NVS_NOT_READY:                       return "NVS_NOT_READY";
    case ReconcileStatus::WRITES_BLOCKED:                      return "WRITES_BLOCKED";
    case ReconcileStatus::BAD_ARGUMENT:                        return "BAD_ARGUMENT";
    case ReconcileStatus::LOAD_FAILED:                         return "LOAD_FAILED";
    case ReconcileStatus::NOT_REQUIRED:                        return "NOT_REQUIRED";
    case ReconcileStatus::ACTION_NOT_ALLOWED:                  return "ACTION_NOT_ALLOWED";
    case ReconcileStatus::GENERATION_NOT_VALID:                return "GENERATION_NOT_VALID";
    case ReconcileStatus::DECLARE_REFUSED_ACKNOWLEDGED_INTACT: return "DECLARE_REFUSED_ACKNOWLEDGED_INTACT";
    case ReconcileStatus::DISCARD_NOT_CONFIRMED:               return "DISCARD_NOT_CONFIRMED";
    case ReconcileStatus::MARKER_NOT_WRITTEN:                  return "MARKER_NOT_WRITTEN";
    case ReconcileStatus::MARKER_WRITE_FAILED:                 return "MARKER_WRITE_FAILED";
    case ReconcileStatus::MARKER_VERIFY_FAILED:                return "MARKER_VERIFY_FAILED";
    case ReconcileStatus::POSTCONDITION_FAILED:                return "POSTCONDITION_FAILED";
  }
  return "UNKNOWN";
}

void CalibrationPersistenceService::begin(NvsInitStatus nvs, int32_t esp_error) {
  snapshot_.nvs = nvs;
  snapshot_.esp_error = esp_error;
}

void CalibrationPersistenceService::refresh(const LoadResult& r) {
  ++snapshot_.load_count;
  snapshot_.verdict = verdictOf(r);
  if (snapshot_.load_count == 1) snapshot_.boot_verdict = snapshot_.verdict;
  snapshot_.load_status = r.status;
  snapshot_.degraded = r.degraded;
  snapshot_.loaded_generation = r.status == LoadStatus::OK ? r.generation : 0;
  snapshot_.loaded_slot = r.slot;
  for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) snapshot_.slot[i] = r.report[i];
  snapshot_.marker = r.marker;
  snapshot_.assessment = r.assessment;
}

void CalibrationPersistenceService::load(const actuator::CalibrationGeometryProfile& profile) {
  if (!nvsReady()) {
    ++snapshot_.load_count;
    snapshot_.verdict = PersistenceVerdict::NVS_UNAVAILABLE;
    if (snapshot_.load_count == 1) snapshot_.boot_verdict = snapshot_.verdict;
    snapshot_.load_status = LoadStatus::IO_ERROR;
    snapshot_.degraded = false;
    snapshot_.loaded_generation = 0;
    for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) snapshot_.slot[i] = SlotReport();
    snapshot_.marker = MarkerReport();
    snapshot_.assessment = PersistenceAssessment();
    return;
  }
  // The decoded record is a by-product: it is dropped at once, never kept.
  const LoadResult r = store_.load(profile, &scratch_);
  memset(static_cast<void*>(&scratch_), 0, sizeof(scratch_));
  refresh(r);
}

bool CalibrationPersistenceService::writesBlocked() const {
  return store_.writeState() != WriteState::OPEN || reconcile_uncertain_ || ack_uncertain_;
}

bool CalibrationPersistenceService::calibrationAvailable() const {
  return nvsReady() && snapshot_.verdict == PersistenceVerdict::VALID_ACKNOWLEDGED &&
         !writesBlocked();
}

ServiceSaveResult CalibrationPersistenceService::save(
    const CalibrationRecord& record, const actuator::CalibrationGeometryProfile& profile) {
  ServiceSaveResult out;
  if (!nvsReady()) {
    out.guard = ServiceGuard::NVS_NOT_READY;
    return out;
  }
  if (writesBlocked()) {
    out.guard = ServiceGuard::WRITES_BLOCKED;
    return out;
  }
  out.guard = ServiceGuard::OK;
  out.save = store_.save(record, profile);
  out.uncertain = out.save.status != SaveStatus::OK && store_.writeState() != WriteState::OPEN;
  // `record` may be scratchRecord(): load() overwrites it, so refresh last.
  load(profile);
  return out;
}

ServiceAckResult CalibrationPersistenceService::acknowledge(
    uint32_t generation, const actuator::CalibrationGeometryProfile& profile) {
  ServiceAckResult out;
  if (!nvsReady()) {
    out.guard = ServiceGuard::NVS_NOT_READY;
    return out;
  }
  out.guard = ServiceGuard::OK;
  out.ack = store_.acknowledge(generation, profile);
  out.uncertain = isUncertainAck(out.ack.status);
  if (out.uncertain) ack_uncertain_ = true;
  load(profile);
  // A positive read-back of a CONSISTENT store is the evidence the doubt needed.
  if (ack_uncertain_ && snapshot_.verdict == PersistenceVerdict::VALID_ACKNOWLEDGED &&
      snapshot_.loaded_generation == generation) {
    ack_uncertain_ = false;
  }
  return out;
}

ReconcileResult CalibrationPersistenceService::reconcile(
    ReconciliationAction action, uint32_t generation, bool discard_confirmed,
    const actuator::CalibrationGeometryProfile& profile) {
  ReconcileResult r;
  if (!nvsReady()) {
    r.status = ReconcileStatus::NVS_NOT_READY;
    return r;
  }
  if (writesBlocked()) {
    r.status = ReconcileStatus::WRITES_BLOCKED;
    return r;
  }
  const bool adopt = action == ReconciliationAction::ADOPT_VALID_RECORD;
  if ((!adopt && action != ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED) ||
      (adopt && generation == 0)) {
    r.status = ReconcileStatus::BAD_ARGUMENT;
    return r;
  }

  // The decision rests on a LOAD made now, never on the boot snapshot.
  const LoadResult fresh = store_.load(profile, &scratch_);
  memset(static_cast<void*>(&scratch_), 0, sizeof(scratch_));
  refresh(fresh);
  r.before = fresh.assessment.cls;
  r.acknowledged_before = fresh.assessment.acknowledged_generation;
  if (fresh.assessment.cls == PersistenceClass::IO_ERROR) {
    r.status = ReconcileStatus::LOAD_FAILED;
    return r;
  }

  PersistenceInputs inputs;
  for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) inputs.slot[i] = fresh.report[i];
  inputs.marker = fresh.marker;
  r.plan = planReconciliation(inputs, action, generation, &r.marker);
  switch (r.plan) {
    case ReconciliationStatus::OK:
      break;
    case ReconciliationStatus::NOT_REQUIRED:
      r.status = ReconcileStatus::NOT_REQUIRED;
      return r;
    case ReconciliationStatus::ACTION_NOT_ALLOWED:
      r.status = ReconcileStatus::ACTION_NOT_ALLOWED;
      return r;
    case ReconciliationStatus::GENERATION_NOT_VALID:
      r.status = ReconcileStatus::GENERATION_NOT_VALID;
      return r;
    case ReconciliationStatus::BAD_ARGUMENT:
      r.status = ReconcileStatus::BAD_ARGUMENT;
      return r;
  }

  if (!adopt && fresh.assessment.acknowledged_record_intact) {
    r.status = ReconcileStatus::DECLARE_REFUSED_ACKNOWLEDGED_INTACT;
    return r;
  }

  const uint32_t ack = fresh.assessment.acknowledged_generation;
  if (adopt) {
    r.discards = ack > generation;
    for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) {
      if (fresh.report[i].state == SlotState::VALID && fresh.report[i].generation_hint > generation) {
        r.discards = true;
      }
    }
  } else {
    r.discards = ack > 0;
    for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) {
      if (fresh.report[i].state == SlotState::VALID ||
          fresh.report[i].state == SlotState::INCOMPATIBLE) {
        r.discards = true;
      }
    }
  }
  if (r.discards && !discard_confirmed) {
    r.status = ReconcileStatus::DISCARD_NOT_CONFIRMED;
    return r;
  }

  uint8_t wire[kSaveMarkerScratchBytes];
  size_t wire_len = 0;
  if (encodeSaveMarker(r.marker, wire, sizeof(wire), &wire_len) != SaveMarkerStatus::OK) {
    r.status = ReconcileStatus::BAD_ARGUMENT;
    return r;
  }
  const StorageIoStatus wrote = storage_->writeMarker(wire, wire_len);
  if (wrote == StorageIoStatus::NOT_MODIFIED) {
    r.status = ReconcileStatus::MARKER_NOT_WRITTEN;
    return r;
  }
  if (wrote != StorageIoStatus::OK) {
    r.status = ReconcileStatus::MARKER_WRITE_FAILED;
    r.uncertain = true;
    reconcile_uncertain_ = true;
    load(profile);
    r.after = snapshot_.assessment.cls;
    return r;
  }
  uint8_t back[kSaveMarkerScratchBytes];
  size_t back_len = 0;
  SaveMarker decoded;
  if (storage_->readMarker(back, sizeof(back), &back_len) != StorageIoStatus::OK ||
      back_len != wire_len || memcmp(back, wire, wire_len) != 0 ||
      decodeSaveMarker(back, back_len, &decoded) != SaveMarkerStatus::OK ||
      !(decoded == r.marker)) {
    r.status = ReconcileStatus::MARKER_VERIFY_FAILED;
    r.uncertain = true;
    reconcile_uncertain_ = true;
    load(profile);
    r.after = snapshot_.assessment.cls;
    return r;
  }

  load(profile);
  r.after = snapshot_.assessment.cls;
  const PersistenceClass expected =
      adopt ? PersistenceClass::CONSISTENT : PersistenceClass::NOTHING_ACKNOWLEDGED;
  if (r.after != expected) {
    r.status = ReconcileStatus::POSTCONDITION_FAILED;
    r.uncertain = true;
    reconcile_uncertain_ = true;
    return r;
  }
  r.status = ReconcileStatus::OK;
  return r;
}

}  // namespace calibration
}  // namespace matdog
