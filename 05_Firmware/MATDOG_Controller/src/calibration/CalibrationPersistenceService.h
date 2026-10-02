#ifndef MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_SERVICE_H
#define MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_SERVICE_H

#include <stdint.h>

#include "CalibrationRecordNvsBackend.h"
#include "CalibrationRecordStore.h"

// PERSISTENCE SERVICE (P3a) - the Controller-facing owner of one
// CalibrationRecordStore. Pure: no Arduino, no NVS call of its own (the backend
// is injected as a CalibrationRecordStorage), no servo bus, no actuator policy,
// no JointTransformTable. It adds exactly what the Controller needs and the
// store does not give:
//   - the boot LOAD result in one stable vocabulary (PersistenceVerdict);
//   - a read-only snapshot for diagnostics;
//   - SAVE / ACK / RECONCILE entry points that refuse when the backend is not
//     READY or an earlier write left the outcome uncertain;
//   - the one persistent action the store lacks, an explicit reconciliation
//     with marker read-back, built only from the P2.4.1 public pieces
//     (planReconciliation + the storage marker slot).
//
// WHAT IT NEVER DOES. Decide that a calibration may be USED (a valid
// acknowledged record is evidence, "available" is not an authorization),
// restore or admit anything into the motion path, touch a servo, erase or
// format storage, acknowledge or reconcile on its own. Every persistent change
// is an explicit call.

namespace matdog {
namespace calibration {

enum class PersistenceVerdict : uint8_t {
  NOT_RUN = 0,                    // no LOAD ran since boot
  NVS_UNAVAILABLE,                // backend not READY (partition missing, geometry, init...)
  STORAGE_ERROR,                  // a slot or the marker could not be read
  NO_RECORD,                      // nothing stored / nothing acknowledged: Full Calibration needed
  VALID_ACKNOWLEDGED,             // CONSISTENT: acknowledged record is valid (NOT a motion permit)
  SAVE_PENDING,                   // an interrupted SAVE
  AWAITING_ACK,                   // verified on flash, never acknowledged
  AWAITING_ACK_RECORD_LOST,
  ACKNOWLEDGED_GENERATION_LOST,   // marker attests G, no valid record of G (no damaged slot seen)
  RECORD_CORRUPT,                 // as above, but a damaged record slot was seen
  MARKER_MISSING,
  MARKER_CORRUPT,
  MARKER_INCOMPATIBLE,
  RECORD_INCOMPATIBLE,
  RECONCILIATION_REQUIRED,        // record ahead of marker / generation conflict
};
const char* toString(PersistenceVerdict verdict);
const char* toString(MarkerObservation observation);
const char* toString(StorageIoStatus status);

struct PersistenceSnapshot {
  uint32_t load_count = 0;               // LOADs run since boot (boot LOAD = 1)
  NvsInitStatus nvs = NvsInitStatus::NOT_INITIALIZED;
  int32_t esp_error = 0;
  PersistenceVerdict boot_verdict = PersistenceVerdict::NOT_RUN;
  PersistenceVerdict verdict = PersistenceVerdict::NOT_RUN;  // most recent LOAD
  LoadStatus load_status = LoadStatus::IO_ERROR;
  bool degraded = false;
  uint32_t loaded_generation = 0;        // meaningful for VALID_ACKNOWLEDGED
  CalibrationSlot loaded_slot = CalibrationSlot::A;
  SlotReport slot[kCalibrationSlotCount];
  MarkerReport marker;
  PersistenceAssessment assessment;
};

// Why a persistent write was not even attempted.
enum class ServiceGuard : uint8_t {
  OK = 0,
  NVS_NOT_READY,
  WRITES_BLOCKED,  // a write outcome is uncertain: only a reboot clears it
};
const char* toString(ServiceGuard guard);

struct ServiceSaveResult {
  ServiceGuard guard = ServiceGuard::NVS_NOT_READY;
  SaveResult save;       // meaningful when guard == OK
  bool uncertain = false;  // the outcome may differ from what the status says
};

struct ServiceAckResult {
  ServiceGuard guard = ServiceGuard::NVS_NOT_READY;
  AckResult ack;         // meaningful when guard == OK
  bool uncertain = false;  // MARKER_WRITE_FAILED / MARKER_VERIFY_FAILED
};

enum class ReconcileStatus : uint8_t {
  OK = 0,                       // marker written, read back, state re-classified
  NVS_NOT_READY,
  WRITES_BLOCKED,
  BAD_ARGUMENT,
  LOAD_FAILED,                  // the fresh LOAD found storage unreadable
  NOT_REQUIRED,                 // state needs no reconciliation (or none is possible)
  ACTION_NOT_ALLOWED,
  GENERATION_NOT_VALID,
  DECLARE_REFUSED_ACKNOWLEDGED_INTACT,  // the acknowledged record is fine: ADOPT it instead
  DISCARD_NOT_CONFIRMED,        // would discard a generation and the caller did not say so
  MARKER_NOT_WRITTEN,           // storage certified it did nothing
  MARKER_WRITE_FAILED,          // uncertain
  MARKER_VERIFY_FAILED,         // uncertain
  POSTCONDITION_FAILED,         // marker verified but the state is not the planned one
};
const char* toString(ReconcileStatus status);

struct ReconcileResult {
  ReconcileStatus status = ReconcileStatus::BAD_ARGUMENT;
  ReconciliationStatus plan = ReconciliationStatus::BAD_ARGUMENT;
  PersistenceClass before = PersistenceClass::IO_ERROR;
  PersistenceClass after = PersistenceClass::IO_ERROR;
  SaveMarker marker;                 // the marker planned / written
  bool discards = false;             // the action drops an acknowledged or newer valid generation
  uint32_t acknowledged_before = 0;  // marker's acknowledged generation before the action
  bool uncertain = false;
};

class CalibrationPersistenceService {
 public:
  explicit CalibrationPersistenceService(CalibrationRecordStorage* storage)
      : storage_(storage), store_(storage) {}

  // Records the backend's init outcome. Touches no storage.
  void begin(NvsInitStatus nvs, int32_t esp_error);

  // Read-only LOAD: scans the slots and the marker, classifies, refreshes the
  // snapshot. Never writes, never serves the record to anyone. With a backend
  // that is not READY nothing is read at all.
  void load(const actuator::CalibrationGeometryProfile& profile);

  const PersistenceSnapshot& snapshot() const { return snapshot_; }

  // The stored calibration is valid and acknowledged AND no write outcome is in
  // doubt. This is information, never permission to move or to admit
  // JointTransforms.
  bool calibrationAvailable() const;
  // Saves / reconciliations are refused for the rest of this boot.
  bool writesBlocked() const;
  WriteState storeWriteState() const { return store_.writeState(); }
  bool ackUncertain() const { return ack_uncertain_; }
  bool reconcileUncertain() const { return reconcile_uncertain_; }

  // Scratch for the record about to be saved (the Controller owns the only copy;
  // no second 1.4 KiB buffer on any stack).
  CalibrationRecord* scratchRecord() { return &scratch_; }

  // P2.4.1 SAVE of `record` (normally scratchRecord()). The caller has already
  // proven the physical prerequisites (CalibrationSaveGate).
  ServiceSaveResult save(const CalibrationRecord& record,
                         const actuator::CalibrationGeometryProfile& profile);

  ServiceAckResult acknowledge(uint32_t generation,
                               const actuator::CalibrationGeometryProfile& profile);

  // Explicit reconciliation. Does a FRESH load, validates the action against it,
  // writes the single marker planReconciliation() returns, reads it back and
  // decodes it, then re-classifies. `generation` is used by ADOPT only.
  // `discard_confirmed` must be true for any action that drops an acknowledged
  // or newer valid generation.
  ReconcileResult reconcile(ReconciliationAction action, uint32_t generation,
                            bool discard_confirmed,
                            const actuator::CalibrationGeometryProfile& profile);

 private:
  bool nvsReady() const { return snapshot_.nvs == NvsInitStatus::READY; }
  void refresh(const LoadResult& result);
  void resolveAckUncertaintyIfConsistent();

  CalibrationRecordStorage* storage_;
  CalibrationRecordStore store_;
  PersistenceSnapshot snapshot_;
  bool ack_uncertain_ = false;
  bool reconcile_uncertain_ = false;
  CalibrationRecord scratch_;
};

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_SERVICE_H
