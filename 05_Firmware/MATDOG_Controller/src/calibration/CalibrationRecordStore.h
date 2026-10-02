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
// acknowledged record, so a failure at any point of the write leaves the
// acknowledged record untouched. The previous slot is never pre-erased and there
// is no "current slot" key.
//
// COMMIT vs ACKNOWLEDGMENT (P2.4.1). "Verified on flash" is not "acknowledged by
// the caller": the reply SAVE=OK can be lost after the last write. The marker
// (CalibrationSaveMarker.h) therefore keeps the last ACKNOWLEDGED generation
// apart from the generation just written, and the slot of the acknowledged one
// stays protected until the new one is acknowledged:
//   save():
//     1. validate the record and the store state (a SAVE is refused while a
//        generation awaits its ACK or any doubt is unresolved);
//     2. marker PENDING(acknowledged = G, begun = G+1), read back;
//     3. write the record to the slot that does NOT hold G;
//     4. read it back and compare it whole;
//     5. marker AWAITING_ACK(acknowledged = G, begun = G+1), read back;
//     6. only now SaveStatus::OK == "G+1 verified, awaiting ACK".
//   acknowledge(G+1):
//     7. rescan; accepted only for the generation the marker awaits;
//     8. marker IDLE(acknowledged = G+1), read back; only then G+1 replaces G and
//        the old slot becomes reusable.
// The device emitting SAVE=OK, the caller receiving it and the device recording
// the ACK are three different events; only the third changes persistent state.
// Nothing promotes a generation automatically: not a reboot, not a valid record
// with the highest generation, not a lost reply.
//
// After step 2 the storage is no longer "as it was": any failure from then on
// (including a storage that says it did not modify anything) blocks this
// instance. A reboot classifies the result with classifyPersistence()
// (CalibrationPersistenceState.h); it never repairs, promotes or erases.
// load() serves a record ONLY in the CONSISTENT class; every other class says
// why and what explicit acknowledgment / reconciliation (not implemented here as
// a Controller command) it needs.
//
// WRITE-UNCERTAINTY BLOCK (session-local, RAM only). Once a write has been
// attempted and its outcome is not certain to have left storage untouched or
// fully verified, this store instance refuses every further save() with
// BLOCKED_UNCERTAIN_WRITE. load() and diagnostics stay available and never
// clear the block; acknowledge() does not depend on it because it rescans
// storage every time. Only a new instance (a reboot) clears it - and then the
// persistent marker, not RAM, keeps an unresolved SAVE from being retried.
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
  // The acknowledged record was selected: PersistenceClass::CONSISTENT, the only
  // class that serves a record.
  OK = 0,
  // No calibration is stored: NEVER_INITIALIZED_OR_ERASED (indistinguishable from
  // a total loss) or NOTHING_ACKNOWLEDGED. A fresh Full Calibration is required.
  NOT_FOUND,
  IO_ERROR,         // storage failed on a slot or the marker: fail closed
  INCOMPATIBLE,     // the acknowledged record or the marker is intact but foreign
  // A generation was saved and verified but never acknowledged. Nothing is served
  // (not even the previous generation, which is protected, not blessed): the
  // caller must acknowledge it, or an explicit reconciliation must decide.
  ACKNOWLEDGMENT_REQUIRED,
  // Slots and marker disagree (interrupted SAVE, lost acknowledged generation,
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
  // A previous SAVE was verified but its generation was never acknowledged. Nothing
  // was written: the protected slot may only be released by acknowledge().
  ACKNOWLEDGMENT_REQUIRED,
  // The persistent state is not CONSISTENT/empty: an earlier SAVE was never
  // finished, an acknowledged generation is lost, the marker is missing... Nothing
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
  MARKER_AWAITING_ACK_WRITE,
  MARKER_AWAITING_ACK_VERIFY,
  DONE,
};
const char* toString(SavePhase phase);

struct SaveResult {
  SaveStatus status = SaveStatus::BAD_ARGUMENT;
  SavePhase phase = SavePhase::NONE;          // last phase entered
  CalibrationSlot slot = CalibrationSlot::A;  // slot targeted (valid on OK and on write failures)
  uint32_t generation = 0;                    // generation verified, awaiting ACK (valid on OK only)
  CalibrationRecordStatus validation = CalibrationRecordStatus::OK;  // INVALID_RECORD detail
  StorageIoStatus io = StorageIoStatus::OK;                          // storage detail
  // Persistent state found before this attempt (valid once the scan was done).
  PersistenceClass persistence = PersistenceClass::IO_ERROR;
  // The ACKNOWLEDGED record was not the write target and this attempt did not touch
  // it. False when no acknowledged record existed or no scan was done.
  bool previous_record_intact = false;
  uint32_t previous_generation = 0;  // its generation, valid with previous_record_intact
};

enum class AckStatus : uint8_t {
  OK = 0,                    // the ACK is persistent and verified: the generation is acknowledged
  ALREADY_ACKNOWLEDGED,      // idempotent repeat (e.g. the reply to an earlier ACK was lost)
  BAD_ARGUMENT,              // generation 0 / no storage
  STORAGE_UNUSABLE,          // a slot or the marker could not be read
  NOT_AWAITING,              // no generation awaits an ACK
  GENERATION_MISMATCH,       // not the generation that awaits (or is in flight)
  RECORD_NOT_VALID,          // the record is incomplete, unverified, lost or incompatible
  RECONCILIATION_REQUIRED,   // unresolved state: only explicit reconciliation helps
  MARKER_WRITE_FAILED,       // storage error recording the ACK: state is uncertain
  MARKER_VERIFY_FAILED,      // the ACK marker could not be verified: state is uncertain
};
const char* toString(AckStatus status);

struct AckResult {
  AckStatus status = AckStatus::BAD_ARGUMENT;
  uint32_t generation = 0;  // the generation the caller asked to acknowledge
  StorageIoStatus io = StorageIoStatus::OK;
  PersistenceClass persistence = PersistenceClass::IO_ERROR;  // found before acting
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
  // state is CONSISTENT, select the acknowledged record. Never writes.
  LoadResult load(const actuator::CalibrationGeometryProfile& profile, CalibrationRecord* out);

  // The SAVE protocol above. `record.generation` is ignored: the store assigns
  // max(every generation seen on disk, marker.begun) + 1. On any failure the
  // previously acknowledged record is untouched and the result says which phase
  // failed. SaveStatus::OK is returned only after the record AND the AWAITING_ACK
  // marker were read back and verified; it means "verified, awaiting ACK", not
  // "acknowledged". Every outcome after the PENDING marker was published that is
  // not a full success moves the instance to WriteState::BLOCKED_UNCERTAIN_WRITE.
  SaveResult save(const CalibrationRecord& record,
                  const actuator::CalibrationGeometryProfile& profile);

  // The caller's explicit acknowledgment of `generation` (the one SAVE returned).
  // Pure storage API: P3a exposes it as a command later. Accepts only the
  // generation the persistent marker awaits, whose record is VALID, with the
  // previous acknowledged record intact. OK only after the ACK marker was read
  // back; then, and only then, the previous slot is reusable. Idempotent for an
  // already acknowledged generation. Every other state is refused without writing.
  AckResult acknowledge(uint32_t generation, const actuator::CalibrationGeometryProfile& profile);

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
