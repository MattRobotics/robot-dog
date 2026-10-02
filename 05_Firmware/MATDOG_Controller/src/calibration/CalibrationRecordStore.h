#ifndef MATDOG_CALIBRATION_CALIBRATION_RECORD_STORE_H
#define MATDOG_CALIBRATION_CALIBRATION_RECORD_STORE_H

#include <stddef.h>
#include <stdint.h>

#include "CalibrationPersistenceState.h"
#include "CalibrationRecord.h"
#include "CalibrationSaveMarker.h"

// CALIBRATION RECORD STORE - A/B slot selection, generations and the commit
// protocol, over a minimal storage interface (P2: no Controller integration).
//
// Two slots, "A" and "B". A save always targets the slot that does NOT hold the
// confirmed record, so a failure at any point of the write leaves the previous
// record untouched. The previous slot is never pre-erased and there is no
// "current slot" key.
//
// SAVE MARKER (P2.4). A third value, stored apart from the slots, says which
// generation was last CONFIRMED and whether a SAVE is in flight. SAVE is:
//   1. validate the record and the store state;
//   2. publish the new generation as PENDING in the marker, and verify it;
//   3. write the record to the inactive slot;
//   4. read it back and compare it whole;
//   5. publish the marker as COMPLETED for that generation;
//   6. read the marker back and verify it;
//   7. only now SaveStatus::OK.
// After step 2 the storage is no longer "as it was": any failure from then on
// (including a storage that says it did not modify anything) blocks this
// instance. A reboot classifies the result with classifyPersistence()
// (CalibrationPersistenceState.h); it never repairs, promotes or erases.
// load() serves a record ONLY in the CONSISTENT class; every other class says
// why and what explicit reconciliation (not implemented here) it needs.
//
// WRITE-UNCERTAINTY BLOCK (session-local, RAM only). Once a write has been
// attempted and its outcome is not certain to have left storage untouched or
// fully verified, this store instance refuses every further save() with
// BLOCKED_UNCERTAIN_WRITE. load() and diagnostics stay available and never
// clear the block. Only a new instance (a reboot) clears it - and then the
// persistent marker, not RAM, keeps an unreconciled SAVE from being retried.
//
// Pure: no Arduino, no NVS. The real backend lives in
// CalibrationRecordNvsBackend; host tests use a fault-injecting fake.
//
// Loading a record is NOT permission to use it. A loaded record is evidence;
// admitting any value from it into the motion path is a separate, later step.

namespace matdog {
namespace calibration {

// Slot capacity: V1 record plus headroom, bounded and statically allocated.
constexpr size_t kCalibrationSlotScratchBytes = 1536;

const char* toString(CalibrationSlot slot);

enum class StorageIoStatus : uint8_t {
  OK = 0,
  ABSENT,              // nothing stored under this key (read only)
  IO_ERROR,            // storage failed (open, read, write, commit, bad handle...)
  NO_SPACE,            // storage reported it is full (write only); the backend cannot
                       // promise that nothing was published, so the store treats it
                       // as an uncertain write
  BUFFER_TOO_SMALL,    // stored blob larger than the scratch (read only)
  NOT_MODIFIED,        // write only: failed BEFORE modifying anything (e.g. the storage
                       // could not be opened). Storage is untouched.
};

// What the store needs from storage: the two record slots and the save marker.
// Never erases a whole partition and never touches anything else.
class CalibrationRecordStorage {
 public:
  virtual ~CalibrationRecordStorage() {}
  // On OK, *length is the stored blob length (<= capacity).
  virtual StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity,
                               size_t* length) = 0;
  // Replace the slot content. Must be durable on OK (backend commits). Any
  // result other than OK and NOT_MODIFIED means the slot may hold anything:
  // nothing, the old blob, a partial update, or the complete new blob.
  virtual StorageIoStatus write(CalibrationSlot slot, const uint8_t* data, size_t length) = 0;
  // Same contract for the save marker, a value stored apart from the slots.
  virtual StorageIoStatus readMarker(uint8_t* buffer, size_t capacity, size_t* length) = 0;
  virtual StorageIoStatus writeMarker(const uint8_t* data, size_t length) = 0;
};

const char* toString(SlotState state);

enum class LoadStatus : uint8_t {
  // The confirmed record was selected: PersistenceClass::CONSISTENT, the only
  // class that serves a record.
  OK = 0,
  // No calibration is stored: NEVER_INITIALIZED_OR_ERASED (indistinguishable from
  // a total loss) or NOTHING_CONFIRMED. A fresh Full Calibration is required.
  NOT_FOUND,
  IO_ERROR,         // storage failed on a slot or the marker: fail closed
  INCOMPATIBLE,     // the confirmed record or the marker is intact but foreign
  // Slots and marker disagree (interrupted SAVE, lost confirmed generation,
  // missing/corrupt marker, record ahead of marker...). Nothing is served;
  // `assessment` says which and what explicit reconciliation is allowed.
  RECONCILIATION_REQUIRED,
  BAD_ARGUMENT,
};
const char* toString(LoadStatus status);

struct LoadResult {
  LoadStatus status = LoadStatus::RECONCILIATION_REQUIRED;
  CalibrationSlot slot = CalibrationSlot::A;  // meaningful when status == OK
  uint32_t generation = 0;                    // idem
  // The selected record is fine but the other slot is damaged or foreign.
  bool degraded = false;
  SlotReport report[kCalibrationSlotCount];
  MarkerReport marker;
  PersistenceAssessment assessment;
};

enum class SaveStatus : uint8_t {
  OK = 0,
  BAD_ARGUMENT,
  INVALID_RECORD,       // refused by validation before touching storage
  STORAGE_UNUSABLE,     // a slot or the marker could not be read: refusing to write blind
  GENERATION_EXHAUSTED,
  // The persistent state is not CONSISTENT/empty: an earlier SAVE was never
  // reconciled, a confirmed generation is lost, the marker is missing... Nothing
  // was written; see `persistence`. Explicit reconciliation is required.
  RECONCILIATION_REQUIRED,
  WRITE_FAILED,         // storage error while writing a record slot
  NO_SPACE,
  READBACK_FAILED,      // could not read the new record back
  READBACK_MISMATCH,    // new record differs from what was written
  MARKER_WRITE_FAILED,  // storage error while writing the marker (see `phase`)
  MARKER_VERIFY_FAILED, // marker read-back failed or differs from what was written
  BLOCKED_UNCERTAIN_WRITE,  // refused: an earlier write left storage in an uncertain state
                            // (see WriteState); nothing was read or written
};
const char* toString(SaveStatus status);

// Where a SAVE stopped (SaveResult::phase), in protocol order.
enum class SavePhase : uint8_t {
  NONE = 0,
  VALIDATE,
  SCAN,
  CLASSIFY,
  MARKER_PENDING_WRITE,
  MARKER_PENDING_VERIFY,
  RECORD_WRITE,
  RECORD_VERIFY,
  MARKER_COMPLETED_WRITE,
  MARKER_COMPLETED_VERIFY,
  DONE,
};
const char* toString(SavePhase phase);

struct SaveResult {
  SaveStatus status = SaveStatus::BAD_ARGUMENT;
  SavePhase phase = SavePhase::NONE;          // last phase entered
  CalibrationSlot slot = CalibrationSlot::A;  // slot targeted (valid on OK and on write failures)
  uint32_t generation = 0;                    // generation confirmed (valid on OK only)
  CalibrationRecordStatus validation = CalibrationRecordStatus::OK;  // INVALID_RECORD detail
  StorageIoStatus io = StorageIoStatus::OK;                          // storage detail
  // Persistent state found before this attempt (valid once the scan was done).
  PersistenceClass persistence = PersistenceClass::IO_ERROR;
  // The CONFIRMED record was not the write target and this attempt did not touch
  // it. False when no confirmed record existed or no scan was done.
  bool previous_record_intact = false;
  uint32_t previous_generation = 0;  // its generation, valid with previous_record_intact
};

// Session-local write state of one store instance. Never persisted.
enum class WriteState : uint8_t {
  OPEN = 0,                 // saves allowed
  BLOCKED_UNCERTAIN_WRITE,  // a write outcome is uncertain: saves refused until reboot
};
const char* toString(WriteState state);

class CalibrationRecordStore {
 public:
  explicit CalibrationRecordStore(CalibrationRecordStorage* storage) : storage_(storage) {}

  // Read both slots and the marker, classify them together and, only if the
  // state is CONSISTENT, select the confirmed record. Never writes.
  LoadResult load(const actuator::CalibrationGeometryProfile& profile, CalibrationRecord* out);

  // The SAVE protocol above. `record.generation` is ignored: the store assigns
  // max(every generation seen on disk, marker.begun) + 1. On any failure the
  // previously confirmed record is untouched and the result says which phase
  // failed. SaveStatus::OK is returned only after the record AND the COMPLETED
  // marker were read back and verified. Every outcome after the PENDING marker
  // was published that is not a full success moves the instance to
  // WriteState::BLOCKED_UNCERTAIN_WRITE.
  SaveResult save(const CalibrationRecord& record,
                  const actuator::CalibrationGeometryProfile& profile);

  // Diagnostics only. load() never changes it.
  WriteState writeState() const { return write_state_; }

 private:
  struct Scan {
    LoadResult load;
    uint32_t max_generation = 0;  // highest generation seen on disk, slots and marker
    bool io_error = false;
  };
  void scan(const actuator::CalibrationGeometryProfile& profile, Scan* scan);
  // Marker read + decode.
  void readMarkerReport(MarkerReport* report, bool* io_error);
  // Write `marker`, read it back, compare. OK only if verified. `*unmodified` is
  // true only when the storage itself said the marker was left alone.
  SaveStatus writeAndVerifyMarker(const SaveMarker& marker, StorageIoStatus* io,
                                  bool* unmodified);

  CalibrationRecordStorage* storage_;
  WriteState write_state_ = WriteState::OPEN;
  uint8_t buffer_[kCalibrationSlotScratchBytes];
  uint8_t verify_[kCalibrationSlotScratchBytes];
  CalibrationRecord decoded_[kCalibrationSlotCount];
};

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_RECORD_STORE_H
