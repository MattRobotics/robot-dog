#ifndef MATDOG_CALIBRATION_CALIBRATION_RECORD_STORE_H
#define MATDOG_CALIBRATION_CALIBRATION_RECORD_STORE_H

#include <stddef.h>
#include <stdint.h>

#include "CalibrationRecord.h"

// CALIBRATION RECORD STORE - A/B slot selection, generations and the commit
// protocol, over a minimal storage interface (P2: no Controller integration).
//
// Two slots, "A" and "B". A save always targets the slot that does NOT hold the
// record currently selected as valid, so a failure at any point of the write
// leaves the previous record untouched. The previous slot is never pre-erased
// and there is no "current slot" key: the valid slot with the highest
// generation IS the current record, decided from the data alone at every load.
//
// Pure: no Arduino, no NVS. The real backend lives in
// CalibrationRecordNvsBackend; host tests use a fault-injecting fake.
//
// Loading a record is NOT permission to use it. A loaded record is evidence;
// admitting any value from it into the motion path is a separate, later step.

namespace matdog {
namespace calibration {

enum class CalibrationSlot : uint8_t { A = 0, B = 1 };
constexpr uint8_t kCalibrationSlotCount = 2;

// Slot capacity: V1 record plus headroom, bounded and statically allocated.
constexpr size_t kCalibrationSlotScratchBytes = 1536;

inline CalibrationSlot otherSlot(CalibrationSlot s) {
  return s == CalibrationSlot::A ? CalibrationSlot::B : CalibrationSlot::A;
}
const char* toString(CalibrationSlot slot);

enum class StorageIoStatus : uint8_t {
  OK = 0,
  ABSENT,              // nothing stored under this slot (read only)
  IO_ERROR,            // storage failed (open, read, write, commit, bad handle...)
  NO_SPACE,            // storage is full (write only)
  BUFFER_TOO_SMALL,    // stored blob larger than the scratch (read only)
};

// What the store needs from storage. Never erases a whole partition and never
// touches anything but the two calibration slots.
class CalibrationRecordStorage {
 public:
  virtual ~CalibrationRecordStorage() {}
  // On OK, *length is the stored blob length (<= capacity).
  virtual StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity,
                               size_t* length) = 0;
  // Replace the slot content. Must be durable on OK (backend commits).
  virtual StorageIoStatus write(CalibrationSlot slot, const uint8_t* data, size_t length) = 0;
};

enum class SlotState : uint8_t {
  UNREAD = 0,
  ABSENT,
  IO_ERROR,
  CORRUPT,       // damaged or self-contradictory (see CalibrationRecordStatus)
  INCOMPATIBLE,  // intact, other schema / geometry / installation
  VALID,
};
const char* toString(SlotState state);

struct SlotReport {
  SlotState state = SlotState::UNREAD;
  CalibrationRecordStatus detail = CalibrationRecordStatus::OK;
  // Generation read from an envelope-intact blob (valid, incompatible or
  // semantically corrupt): keeps generations monotonic. 0 when unknown.
  uint32_t generation_hint = 0;
};

enum class LoadStatus : uint8_t {
  OK = 0,           // a valid record was selected
  NOT_FOUND,        // both slots absent: a clean "never saved"
  IO_ERROR,         // storage failed on a slot: fail closed, nothing selected
  INCOMPATIBLE,     // no valid record and at least one slot is intact-but-foreign
  NO_VALID_RECORD,  // no valid record and at least one slot is corrupt
  BAD_ARGUMENT,
};
const char* toString(LoadStatus status);

struct LoadResult {
  LoadStatus status = LoadStatus::NO_VALID_RECORD;
  CalibrationSlot slot = CalibrationSlot::A;  // meaningful when status == OK
  uint32_t generation = 0;                    // idem
  // The selected record is older than a slot that exists but is unusable:
  // the newest save was lost, the previous calibration was recovered.
  bool degraded = false;
  SlotReport report[kCalibrationSlotCount];
};

enum class SaveStatus : uint8_t {
  OK = 0,
  BAD_ARGUMENT,
  INVALID_RECORD,       // refused by validation before touching storage
  STORAGE_UNUSABLE,     // a slot could not be read: refusing to write blind
  GENERATION_EXHAUSTED,
  WRITE_FAILED,         // storage error while writing
  NO_SPACE,
  READBACK_FAILED,      // could not read the new slot back
  READBACK_MISMATCH,    // new slot differs from what was written
};
const char* toString(SaveStatus status);

struct SaveResult {
  SaveStatus status = SaveStatus::BAD_ARGUMENT;
  CalibrationSlot slot = CalibrationSlot::A;  // slot targeted (valid on OK and on write failures)
  uint32_t generation = 0;                    // generation written (valid on OK)
  CalibrationRecordStatus validation = CalibrationRecordStatus::OK;  // INVALID_RECORD detail
  StorageIoStatus io = StorageIoStatus::OK;                          // storage detail
  // After a failed save, the record that remains selected (the previous one).
  bool previous_record_intact = false;
  uint32_t previous_generation = 0;
};

class CalibrationRecordStore {
 public:
  explicit CalibrationRecordStore(CalibrationRecordStorage* storage) : storage_(storage) {}

  // Read both slots, decode, validate against `profile`, select the valid one
  // with the highest generation. Never writes.
  LoadResult load(const actuator::CalibrationGeometryProfile& profile, CalibrationRecord* out);

  // validate -> encode -> inactive slot -> write (backend commits) -> read back
  // and compare. `record.generation` is ignored: the store assigns
  // max(known generations) + 1. On any failure the previously selected record
  // is untouched and the result says which slot/step failed.
  SaveResult save(const CalibrationRecord& record,
                  const actuator::CalibrationGeometryProfile& profile);

 private:
  struct Scan {
    LoadResult load;
    uint32_t max_generation = 0;
    bool io_error = false;
  };
  void scan(const actuator::CalibrationGeometryProfile& profile, Scan* scan);

  CalibrationRecordStorage* storage_;
  uint8_t buffer_[kCalibrationSlotScratchBytes];
  uint8_t verify_[kCalibrationSlotScratchBytes];
  CalibrationRecord decoded_[kCalibrationSlotCount];
};

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_RECORD_STORE_H
