#include "CalibrationRecordStore.h"

#include <string.h>

namespace matdog {
namespace calibration {

const char* toString(CalibrationSlot slot) { return slot == CalibrationSlot::A ? "A" : "B"; }

const char* toString(SlotState state) {
  switch (state) {
    case SlotState::UNREAD:       return "UNREAD";
    case SlotState::ABSENT:       return "ABSENT";
    case SlotState::IO_ERROR:     return "IO_ERROR";
    case SlotState::CORRUPT:      return "CORRUPT";
    case SlotState::INCOMPATIBLE: return "INCOMPATIBLE";
    case SlotState::VALID:        return "VALID";
  }
  return "UNKNOWN";
}

const char* toString(LoadStatus status) {
  switch (status) {
    case LoadStatus::OK:                      return "OK";
    case LoadStatus::NOT_FOUND:               return "NOT_FOUND";
    case LoadStatus::IO_ERROR:                return "IO_ERROR";
    case LoadStatus::INCOMPATIBLE:            return "INCOMPATIBLE";
    case LoadStatus::RECONCILIATION_REQUIRED: return "RECONCILIATION_REQUIRED";
    case LoadStatus::BAD_ARGUMENT:            return "BAD_ARGUMENT";
  }
  return "UNKNOWN";
}

const char* toString(SaveStatus status) {
  switch (status) {
    case SaveStatus::OK:                      return "OK";
    case SaveStatus::BAD_ARGUMENT:            return "BAD_ARGUMENT";
    case SaveStatus::INVALID_RECORD:          return "INVALID_RECORD";
    case SaveStatus::STORAGE_UNUSABLE:        return "STORAGE_UNUSABLE";
    case SaveStatus::GENERATION_EXHAUSTED:    return "GENERATION_EXHAUSTED";
    case SaveStatus::RECONCILIATION_REQUIRED: return "RECONCILIATION_REQUIRED";
    case SaveStatus::WRITE_FAILED:            return "WRITE_FAILED";
    case SaveStatus::NO_SPACE:                return "NO_SPACE";
    case SaveStatus::READBACK_FAILED:         return "READBACK_FAILED";
    case SaveStatus::READBACK_MISMATCH:       return "READBACK_MISMATCH";
    case SaveStatus::MARKER_WRITE_FAILED:     return "MARKER_WRITE_FAILED";
    case SaveStatus::MARKER_VERIFY_FAILED:    return "MARKER_VERIFY_FAILED";
    case SaveStatus::BLOCKED_UNCERTAIN_WRITE: return "BLOCKED_UNCERTAIN_WRITE";
  }
  return "UNKNOWN";
}

const char* toString(SavePhase phase) {
  switch (phase) {
    case SavePhase::NONE:                    return "NONE";
    case SavePhase::VALIDATE:                return "VALIDATE";
    case SavePhase::SCAN:                    return "SCAN";
    case SavePhase::CLASSIFY:                return "CLASSIFY";
    case SavePhase::MARKER_PENDING_WRITE:    return "MARKER_PENDING_WRITE";
    case SavePhase::MARKER_PENDING_VERIFY:   return "MARKER_PENDING_VERIFY";
    case SavePhase::RECORD_WRITE:            return "RECORD_WRITE";
    case SavePhase::RECORD_VERIFY:           return "RECORD_VERIFY";
    case SavePhase::MARKER_COMPLETED_WRITE:  return "MARKER_COMPLETED_WRITE";
    case SavePhase::MARKER_COMPLETED_VERIFY: return "MARKER_COMPLETED_VERIFY";
    case SavePhase::DONE:                    return "DONE";
  }
  return "UNKNOWN";
}

const char* toString(WriteState state) {
  switch (state) {
    case WriteState::OPEN:                    return "OPEN";
    case WriteState::BLOCKED_UNCERTAIN_WRITE: return "BLOCKED_UNCERTAIN_WRITE";
  }
  return "UNKNOWN";
}

void CalibrationRecordStore::readMarkerReport(MarkerReport* rep, bool* io_error) {
  *rep = MarkerReport{};
  size_t length = 0;
  const StorageIoStatus io = storage_->readMarker(buffer_, sizeof(buffer_), &length);
  if (io == StorageIoStatus::ABSENT) {
    rep->state = MarkerObservation::ABSENT;
    return;
  }
  if (io == StorageIoStatus::BUFFER_TOO_SMALL) {
    rep->state = MarkerObservation::CORRUPT;  // a V1 marker can never be this large
    rep->detail = SaveMarkerStatus::BAD_LENGTH;
    return;
  }
  if (io != StorageIoStatus::OK || length > sizeof(buffer_)) {
    rep->state = MarkerObservation::IO_ERROR;
    *io_error = true;
    return;
  }
  const SaveMarkerStatus st = decodeSaveMarker(buffer_, length, &rep->marker);
  rep->detail = st;
  if (st == SaveMarkerStatus::OK) {
    rep->state = MarkerObservation::VALID;
  } else {
    rep->marker = SaveMarker{};
    rep->state = isForeignSaveMarker(st) ? MarkerObservation::INCOMPATIBLE
                                         : MarkerObservation::CORRUPT;
  }
}

void CalibrationRecordStore::scan(const actuator::CalibrationGeometryProfile& profile, Scan* out) {
  *out = Scan{};
  for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) {
    const CalibrationSlot slot = static_cast<CalibrationSlot>(i);
    SlotReport& rep = out->load.report[i];
    size_t length = 0;
    const StorageIoStatus io = storage_->read(slot, buffer_, sizeof(buffer_), &length);
    if (io == StorageIoStatus::ABSENT) {
      rep.state = SlotState::ABSENT;
      continue;
    }
    if (io == StorageIoStatus::BUFFER_TOO_SMALL) {
      rep.state = SlotState::CORRUPT;  // a V1 slot can never be this large
      rep.detail = CalibrationRecordStatus::BAD_LENGTH;
      continue;
    }
    if (io != StorageIoStatus::OK || length > sizeof(buffer_)) {
      rep.state = SlotState::IO_ERROR;
      out->io_error = true;
      continue;
    }

    uint16_t schema = 0;
    uint32_t generation = 0;
    CalibrationRecordStatus st = inspectCalibrationEnvelope(buffer_, length, &schema, &generation);
    if (st != CalibrationRecordStatus::OK) {
      rep.state = SlotState::CORRUPT;
      rep.detail = st;
      continue;
    }
    rep.generation_hint = generation;
    if (generation > out->max_generation) out->max_generation = generation;

    st = decodeCalibrationRecord(buffer_, length, &decoded_[i]);
    if (st == CalibrationRecordStatus::OK) st = validateCalibrationRecord(decoded_[i], profile);
    rep.detail = st;
    if (st == CalibrationRecordStatus::OK) {
      rep.state = SlotState::VALID;
    } else {
      rep.state = classifyCalibrationRecordStatus(st) == CalibrationRecordClass::INCOMPATIBLE
                      ? SlotState::INCOMPATIBLE
                      : SlotState::CORRUPT;
    }
  }

  readMarkerReport(&out->load.marker, &out->io_error);
  if (out->load.marker.state == MarkerObservation::VALID) {
    const SaveMarker& m = out->load.marker.marker;
    if (m.completed_generation > out->max_generation) out->max_generation = m.completed_generation;
    if (m.begun_generation > out->max_generation) out->max_generation = m.begun_generation;
  }

  PersistenceInputs in;
  for (uint8_t i = 0; i < kCalibrationSlotCount; ++i) in.slot[i] = out->load.report[i];
  in.marker = out->load.marker;
  out->load.assessment = classifyPersistence(in);
}

LoadResult CalibrationRecordStore::load(const actuator::CalibrationGeometryProfile& profile,
                                        CalibrationRecord* out) {
  LoadResult result;
  if (storage_ == nullptr || out == nullptr) {
    result.status = LoadStatus::BAD_ARGUMENT;
    return result;
  }
  Scan s;
  scan(profile, &s);
  result = s.load;

  const PersistenceAssessment& a = result.assessment;
  switch (a.cls) {
    case PersistenceClass::CONSISTENT: {
      const uint8_t pick = static_cast<uint8_t>(a.confirmed_slot);
      const SlotState other = result.report[1 - pick].state;
      result.status = LoadStatus::OK;
      result.slot = a.confirmed_slot;
      result.generation = decoded_[pick].generation;
      result.degraded = other != SlotState::VALID && other != SlotState::ABSENT;
      *out = decoded_[pick];
      return result;
    }
    case PersistenceClass::NEVER_INITIALIZED_OR_ERASED:
    case PersistenceClass::NOTHING_CONFIRMED:
      result.status = LoadStatus::NOT_FOUND;
      return result;
    case PersistenceClass::IO_ERROR:
      result.status = LoadStatus::IO_ERROR;  // fail closed: never select on partial knowledge
      return result;
    case PersistenceClass::MARKER_INCOMPATIBLE:
    case PersistenceClass::RECORD_INCOMPATIBLE:
      result.status = LoadStatus::INCOMPATIBLE;
      return result;
    default:
      result.status = LoadStatus::RECONCILIATION_REQUIRED;
      return result;
  }
}

SaveStatus CalibrationRecordStore::writeAndVerifyMarker(const SaveMarker& marker,
                                                        StorageIoStatus* io, bool* unmodified) {
  *unmodified = false;
  size_t length = 0;
  if (encodeSaveMarker(marker, buffer_, sizeof(buffer_), &length) != SaveMarkerStatus::OK) {
    *io = StorageIoStatus::OK;
    *unmodified = true;  // nothing was handed to storage
    return SaveStatus::MARKER_WRITE_FAILED;
  }
  const StorageIoStatus wio = storage_->writeMarker(buffer_, length);
  if (wio != StorageIoStatus::OK) {
    *io = wio;
    *unmodified = wio == StorageIoStatus::NOT_MODIFIED;
    return SaveStatus::MARKER_WRITE_FAILED;
  }
  size_t read_length = 0;
  const StorageIoStatus rio = storage_->readMarker(verify_, sizeof(verify_), &read_length);
  if (rio != StorageIoStatus::OK) {
    *io = rio;
    return SaveStatus::MARKER_VERIFY_FAILED;
  }
  SaveMarker back;
  if (read_length != length || memcmp(verify_, buffer_, length) != 0 ||
      decodeSaveMarker(verify_, read_length, &back) != SaveMarkerStatus::OK || !(back == marker)) {
    *io = StorageIoStatus::OK;
    return SaveStatus::MARKER_VERIFY_FAILED;
  }
  *io = StorageIoStatus::OK;
  return SaveStatus::OK;
}

SaveResult CalibrationRecordStore::save(const CalibrationRecord& record,
                                        const actuator::CalibrationGeometryProfile& profile) {
  SaveResult result;
  if (storage_ == nullptr) return result;  // BAD_ARGUMENT

  // Refuse before reading or validating anything: a retry after an uncertain
  // write could overwrite the last record known to be good.
  if (write_state_ != WriteState::OPEN) {
    result.status = SaveStatus::BLOCKED_UNCERTAIN_WRITE;
    return result;
  }

  // 1. Validate before touching storage. The generation is the store's business:
  // validate a copy carrying a placeholder.
  result.phase = SavePhase::VALIDATE;
  decoded_[0] = record;
  decoded_[0].generation = 1;
  const CalibrationRecordStatus vst = validateCalibrationRecord(decoded_[0], profile);
  if (vst != CalibrationRecordStatus::OK) {
    result.status = SaveStatus::INVALID_RECORD;
    result.validation = vst;
    return result;
  }

  result.phase = SavePhase::SCAN;
  Scan s;
  scan(profile, &s);
  const PersistenceAssessment& a = s.load.assessment;
  result.persistence = a.cls;
  if (s.io_error || a.cls == PersistenceClass::IO_ERROR) {
    result.status = SaveStatus::STORAGE_UNUSABLE;
    result.io = StorageIoStatus::IO_ERROR;
    return result;
  }

  result.phase = SavePhase::CLASSIFY;
  if (!a.save_allowed) {
    result.status = SaveStatus::RECONCILIATION_REQUIRED;
    return result;
  }
  if (s.max_generation == 0xFFFFFFFFu) {
    result.status = SaveStatus::GENERATION_EXHAUSTED;
    return result;
  }
  const uint32_t new_generation = s.max_generation + 1;
  const uint32_t confirmed = a.confirmed_generation;

  // Target: never the slot holding the confirmed record.
  uint8_t target = 0;
  if (a.cls == PersistenceClass::CONSISTENT) {
    target = 1 - static_cast<uint8_t>(a.confirmed_slot);
    result.previous_record_intact = true;
    result.previous_generation = confirmed;
  } else {
    // Nothing confirmed to protect: take an absent slot, else a corrupt one, else
    // a foreign one, else a leftover valid one; ties go to the lower generation.
    auto rank = [](SlotState st) {
      return st == SlotState::ABSENT ? 0 : st == SlotState::CORRUPT ? 1 : st == SlotState::INCOMPATIBLE ? 2 : 3;
    };
    const int ra = rank(s.load.report[0].state);
    const int rb = rank(s.load.report[1].state);
    if (rb < ra || (rb == ra && s.load.report[1].generation_hint < s.load.report[0].generation_hint)) {
      target = 1;
    }
  }
  result.slot = static_cast<CalibrationSlot>(target);

  CalibrationRecord to_write = record;
  to_write.generation = new_generation;
  size_t length = 0;
  const CalibrationRecordStatus est = encodeCalibrationRecord(to_write, buffer_, sizeof(buffer_), &length);
  if (est != CalibrationRecordStatus::OK) {
    result.status = SaveStatus::INVALID_RECORD;
    result.validation = est;
    return result;
  }

  // 2. Publish the new generation as PENDING and verify it. A storage that says
  // it did not touch the marker leaves everything as found; anything else may
  // have left a PENDING marker behind.
  SaveMarker pending;
  pending.state = SaveMarkerState::PENDING;
  pending.completed_generation = confirmed;
  pending.begun_generation = new_generation;
  result.phase = SavePhase::MARKER_PENDING_WRITE;
  bool unmodified = false;
  SaveStatus mst = writeAndVerifyMarker(pending, &result.io, &unmodified);
  if (mst != SaveStatus::OK) {
    if (!unmodified) write_state_ = WriteState::BLOCKED_UNCERTAIN_WRITE;
    if (mst == SaveStatus::MARKER_VERIFY_FAILED) result.phase = SavePhase::MARKER_PENDING_VERIFY;
    result.status = mst;
    return result;
  }

  // From here on the marker says a SAVE is in flight: every failure is uncertain.
  // 3. Write the record to the inactive slot. The marker used the scratch, so
  // encode again (deterministic; the same call succeeded above).
  result.phase = SavePhase::RECORD_WRITE;
  if (encodeCalibrationRecord(to_write, buffer_, sizeof(buffer_), &length) != CalibrationRecordStatus::OK) {
    write_state_ = WriteState::BLOCKED_UNCERTAIN_WRITE;
    result.status = SaveStatus::INVALID_RECORD;
    return result;
  }
  const StorageIoStatus wio = storage_->write(result.slot, buffer_, length);
  if (wio != StorageIoStatus::OK) {
    write_state_ = WriteState::BLOCKED_UNCERTAIN_WRITE;
    result.status = wio == StorageIoStatus::NO_SPACE ? SaveStatus::NO_SPACE : SaveStatus::WRITE_FAILED;
    result.io = wio;
    return result;
  }

  // 4. Full read-back.
  result.phase = SavePhase::RECORD_VERIFY;
  size_t read_length = 0;
  const StorageIoStatus rio = storage_->read(result.slot, verify_, sizeof(verify_), &read_length);
  if (rio != StorageIoStatus::OK) {
    write_state_ = WriteState::BLOCKED_UNCERTAIN_WRITE;
    result.status = SaveStatus::READBACK_FAILED;
    result.io = rio;
    return result;
  }
  if (read_length != length || memcmp(verify_, buffer_, length) != 0) {
    write_state_ = WriteState::BLOCKED_UNCERTAIN_WRITE;
    result.status = SaveStatus::READBACK_MISMATCH;
    return result;
  }

  // 5-6. Confirm: COMPLETED marker, read back.
  SaveMarker done;
  done.state = SaveMarkerState::COMPLETED;
  done.completed_generation = new_generation;
  done.begun_generation = new_generation;
  result.phase = SavePhase::MARKER_COMPLETED_WRITE;
  mst = writeAndVerifyMarker(done, &result.io, &unmodified);
  if (mst != SaveStatus::OK) {
    write_state_ = WriteState::BLOCKED_UNCERTAIN_WRITE;
    if (mst == SaveStatus::MARKER_VERIFY_FAILED) result.phase = SavePhase::MARKER_COMPLETED_VERIFY;
    result.status = mst;
    return result;
  }

  // 7. Only now.
  result.phase = SavePhase::DONE;
  result.status = SaveStatus::OK;
  result.generation = new_generation;
  return result;
}

}  // namespace calibration
}  // namespace matdog
