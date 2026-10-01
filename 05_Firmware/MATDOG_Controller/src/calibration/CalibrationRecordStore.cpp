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
    case LoadStatus::OK:              return "OK";
    case LoadStatus::NOT_FOUND:       return "NOT_FOUND";
    case LoadStatus::IO_ERROR:        return "IO_ERROR";
    case LoadStatus::INCOMPATIBLE:    return "INCOMPATIBLE";
    case LoadStatus::NO_VALID_RECORD: return "NO_VALID_RECORD";
    case LoadStatus::BAD_ARGUMENT:    return "BAD_ARGUMENT";
  }
  return "UNKNOWN";
}

const char* toString(SaveStatus status) {
  switch (status) {
    case SaveStatus::OK:                   return "OK";
    case SaveStatus::BAD_ARGUMENT:         return "BAD_ARGUMENT";
    case SaveStatus::INVALID_RECORD:       return "INVALID_RECORD";
    case SaveStatus::STORAGE_UNUSABLE:     return "STORAGE_UNUSABLE";
    case SaveStatus::GENERATION_EXHAUSTED: return "GENERATION_EXHAUSTED";
    case SaveStatus::WRITE_FAILED:         return "WRITE_FAILED";
    case SaveStatus::NO_SPACE:             return "NO_SPACE";
    case SaveStatus::READBACK_FAILED:      return "READBACK_FAILED";
    case SaveStatus::READBACK_MISMATCH:    return "READBACK_MISMATCH";
  }
  return "UNKNOWN";
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

  if (s.io_error) {
    result.status = LoadStatus::IO_ERROR;  // fail closed: never select on partial knowledge
    return result;
  }

  const bool a_valid = result.report[0].state == SlotState::VALID;
  const bool b_valid = result.report[1].state == SlotState::VALID;
  if (a_valid && b_valid && result.report[0].generation_hint == result.report[1].generation_hint) {
    result.status = LoadStatus::NO_VALID_RECORD;  // equal generations contradict the protocol
    return result;
  }
  if (a_valid || b_valid) {
    uint8_t pick = 0;
    if (a_valid && b_valid) {
      pick = result.report[1].generation_hint > result.report[0].generation_hint ? 1 : 0;
    } else {
      pick = a_valid ? 0 : 1;
    }
    const SlotState other = result.report[1 - pick].state;
    result.status = LoadStatus::OK;
    result.slot = static_cast<CalibrationSlot>(pick);
    result.generation = decoded_[pick].generation;
    result.degraded = other != SlotState::VALID && other != SlotState::ABSENT;
    *out = decoded_[pick];
    return result;
  }

  const SlotState a = result.report[0].state;
  const SlotState b = result.report[1].state;
  if (a == SlotState::ABSENT && b == SlotState::ABSENT) {
    result.status = LoadStatus::NOT_FOUND;
  } else if (a == SlotState::INCOMPATIBLE || b == SlotState::INCOMPATIBLE) {
    result.status = LoadStatus::INCOMPATIBLE;
  } else {
    result.status = LoadStatus::NO_VALID_RECORD;
  }
  return result;
}

SaveResult CalibrationRecordStore::save(const CalibrationRecord& record,
                                        const actuator::CalibrationGeometryProfile& profile) {
  SaveResult result;
  if (storage_ == nullptr) return result;  // BAD_ARGUMENT

  // Validate before touching storage. The generation is the store's business:
  // validate a copy carrying a placeholder.
  decoded_[0] = record;
  decoded_[0].generation = 1;
  const CalibrationRecordStatus vst = validateCalibrationRecord(decoded_[0], profile);
  if (vst != CalibrationRecordStatus::OK) {
    result.status = SaveStatus::INVALID_RECORD;
    result.validation = vst;
    return result;
  }

  Scan s;
  scan(profile, &s);
  if (s.io_error) {
    result.status = SaveStatus::STORAGE_UNUSABLE;
    result.io = StorageIoStatus::IO_ERROR;
    return result;
  }
  if (s.max_generation == 0xFFFFFFFFu) {
    result.status = SaveStatus::GENERATION_EXHAUSTED;
    return result;
  }

  // Target: never the slot holding the selected record.
  const SlotState a = s.load.report[0].state;
  const SlotState b = s.load.report[1].state;
  uint8_t target = 0;
  if (a == SlotState::VALID || b == SlotState::VALID) {
    uint8_t selected = 0;
    if (a == SlotState::VALID && b == SlotState::VALID) {
      selected = s.load.report[1].generation_hint > s.load.report[0].generation_hint ? 1 : 0;
    } else {
      selected = a == SlotState::VALID ? 0 : 1;
    }
    target = 1 - selected;
    result.previous_record_intact = true;
    result.previous_generation = s.load.report[selected].generation_hint;
  } else {
    // Nothing valid to protect: take an absent slot, else a corrupt one, else
    // a foreign one; ties go to the lower generation.
    auto rank = [](SlotState st) { return st == SlotState::ABSENT ? 0 : st == SlotState::CORRUPT ? 1 : 2; };
    const int ra = rank(a);
    const int rb = rank(b);
    if (rb < ra || (rb == ra && s.load.report[1].generation_hint < s.load.report[0].generation_hint)) {
      target = 1;
    }
  }
  result.slot = static_cast<CalibrationSlot>(target);
  result.generation = s.max_generation + 1;

  CalibrationRecord to_write = record;
  to_write.generation = result.generation;
  size_t length = 0;
  const CalibrationRecordStatus est = encodeCalibrationRecord(to_write, buffer_, sizeof(buffer_), &length);
  if (est != CalibrationRecordStatus::OK) {
    result.status = SaveStatus::INVALID_RECORD;
    result.validation = est;
    result.generation = 0;
    return result;
  }

  const StorageIoStatus wio = storage_->write(result.slot, buffer_, length);
  if (wio != StorageIoStatus::OK) {
    result.status = wio == StorageIoStatus::NO_SPACE ? SaveStatus::NO_SPACE : SaveStatus::WRITE_FAILED;
    result.io = wio;
    result.generation = 0;
    return result;
  }

  size_t read_length = 0;
  const StorageIoStatus rio = storage_->read(result.slot, verify_, sizeof(verify_), &read_length);
  if (rio != StorageIoStatus::OK) {
    result.status = SaveStatus::READBACK_FAILED;
    result.io = rio;
    result.generation = 0;
    return result;
  }
  if (read_length != length || memcmp(verify_, buffer_, length) != 0) {
    result.status = SaveStatus::READBACK_MISMATCH;
    result.generation = 0;
    return result;
  }
  result.status = SaveStatus::OK;
  return result;
}

}  // namespace calibration
}  // namespace matdog
