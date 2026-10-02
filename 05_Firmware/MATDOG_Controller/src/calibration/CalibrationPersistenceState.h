#ifndef MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_STATE_H
#define MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_STATE_H

#include <stddef.h>
#include <stdint.h>

#include "CalibrationRecord.h"
#include "CalibrationSaveMarker.h"

// PERSISTENCE STATE - what the two record slots and the save marker, read
// together after a (re)boot, say about the stored calibration (P2.4).
//
// Pure: no Arduino, no NVS, no storage access. The inputs are observations the
// store already made; the output is a verdict plus the explicit reconciliation
// the contract allows. Nothing here repairs, promotes or erases anything.
//
// RULES.
//   - The marker is the only authority on what was CONFIRMED. A valid record is
//     evidence, not confirmation: a record newer than the marker is never
//     promoted automatically, and a record is never served just because it has
//     the highest generation.
//   - If the marker attests generation G as completed and only an older one
//     survives, the storage is NOT healthy: CONFIRMED_GENERATION_LOST.
//   - While a SAVE is PENDING nothing may be overwritten and nothing is served
//     until an explicit reconciliation.
//   - "Never used" and "completely erased" look identical when everything that
//     could tell them apart lived in the same partition. Both are
//     NEVER_INITIALIZED_OR_ERASED: no calibration is available, no motion may be
//     authorized from it, a fresh validated Full Calibration has to be SAVEd.

namespace matdog {
namespace calibration {

enum class CalibrationSlot : uint8_t { A = 0, B = 1 };
constexpr uint8_t kCalibrationSlotCount = 2;

inline CalibrationSlot otherSlot(CalibrationSlot s) {
  return s == CalibrationSlot::A ? CalibrationSlot::B : CalibrationSlot::A;
}

enum class SlotState : uint8_t {
  UNREAD = 0,
  ABSENT,
  IO_ERROR,
  CORRUPT,       // damaged or self-contradictory (see CalibrationRecordStatus)
  INCOMPATIBLE,  // intact, other schema / geometry / installation
  VALID,
};

struct SlotReport {
  SlotState state = SlotState::UNREAD;
  CalibrationRecordStatus detail = CalibrationRecordStatus::OK;
  // Generation read from an envelope-intact blob (valid, incompatible or
  // semantically corrupt): keeps generations monotonic. 0 when unknown.
  uint32_t generation_hint = 0;
};

enum class MarkerObservation : uint8_t {
  UNREAD = 0,
  ABSENT,        // nothing stored under the marker key
  IO_ERROR,
  CORRUPT,       // damaged or self-contradictory (see SaveMarkerStatus)
  INCOMPATIBLE,  // intact envelope, other schema
  VALID,
};

struct MarkerReport {
  MarkerObservation state = MarkerObservation::UNREAD;
  SaveMarkerStatus detail = SaveMarkerStatus::OK;
  SaveMarker marker;  // meaningful when state == VALID
};

struct PersistenceInputs {
  SlotReport slot[kCalibrationSlotCount];
  MarkerReport marker;
};

enum class PersistenceClass : uint8_t {
  // Storage could not be read (any slot or the marker): nothing can be said.
  IO_ERROR = 0,
  // No marker and no record. Never used OR completely erased: indistinguishable.
  NEVER_INITIALIZED_OR_ERASED,
  // Marker COMPLETED with no confirmed generation (an interrupted first SAVE or
  // a lost confirmation was explicitly reconciled). No calibration available.
  NOTHING_CONFIRMED,
  // Marker COMPLETED and the record of that generation is valid. The only
  // healthy state with data.
  CONSISTENT,
  // Marker PENDING and the record of the pending generation is absent/damaged.
  PENDING_RECORD_ABSENT,
  // Marker PENDING and the record of the pending generation is valid: a SAVE
  // that was written but never confirmed. NOT promoted.
  PENDING_RECORD_PRESENT,
  // The marker attests generation G; no valid record has generation G.
  // `older_record_survives` says whether an older valid record remains.
  CONFIRMED_GENERATION_LOST,
  // A valid record has a generation the marker never attested nor left pending.
  RECORD_AHEAD_OF_MARKER,
  // Both slots hold valid records with the same generation.
  RECORD_GENERATION_CONFLICT,
  // Records present, no marker (a layout from before the marker, a lost
  // marker, or a wiped marker): nothing is confirmed.
  MARKER_MISSING,
  MARKER_CORRUPT,
  // Marker written by another schema: this build must not interpret or replace it.
  MARKER_INCOMPATIBLE,
  // The record of the confirmed (or a newer) generation is intact but belongs to
  // another schema / geometry / installation.
  RECORD_INCOMPATIBLE,
};
const char* toString(PersistenceClass cls);

// Explicit, operator-driven reconciliation. NOT executed by anything in P2.4;
// planReconciliation() only says which marker the action would leave behind.
enum class ReconciliationAction : uint8_t {
  // Adopt the valid record of generation `g` as the confirmed one.
  ADOPT_VALID_RECORD = 1,
  // Declare that no stored generation is confirmed (fresh calibration required).
  DECLARE_NOTHING_CONFIRMED = 2,
};
constexpr uint8_t kReconcileAdoptBit = 1u << 0;
constexpr uint8_t kReconcileDeclareBit = 1u << 1;
const char* toString(ReconciliationAction action);

struct PersistenceAssessment {
  PersistenceClass cls = PersistenceClass::IO_ERROR;

  // A new SAVE may start: only NEVER_INITIALIZED_OR_ERASED, NOTHING_CONFIRMED,
  // CONSISTENT. Every other class refuses until reconciled.
  bool save_allowed = false;
  // The stored calibration may be loaded as evidence: only CONSISTENT.
  bool record_available = false;

  // Facts, valid when the marker is VALID (0 otherwise).
  uint32_t confirmed_generation = 0;  // marker.completed_generation
  uint32_t pending_generation = 0;    // marker.begun_generation while PENDING
  // The valid record of confirmed_generation exists (also meaningful when the
  // class is PENDING_*: the previous confirmation is still on disk).
  bool confirmed_record_intact = false;
  CalibrationSlot confirmed_slot = CalibrationSlot::A;
  // CONFIRMED_GENERATION_LOST: a valid record of an older generation remains.
  bool older_record_survives = false;

  bool reconciliation_required = false;
  uint8_t allowed_actions = 0;  // kReconcile*Bit
};

// Pure decision. Same inputs, same verdict.
PersistenceAssessment classifyPersistence(const PersistenceInputs& inputs);

enum class ReconciliationStatus : uint8_t {
  OK = 0,
  NOT_REQUIRED,          // the state needs no reconciliation (or none is possible)
  ACTION_NOT_ALLOWED,    // not offered for this class
  GENERATION_NOT_VALID,  // ADOPT: no valid record has that generation, or two do
  BAD_ARGUMENT,
};
const char* toString(ReconciliationStatus status);

// The marker an explicit reconciliation would write. Guarantee, tested: feeding
// *out back in with the same slots classifies as CONSISTENT (ADOPT) or
// NOTHING_CONFIRMED (DECLARE) - never anything that serves an unconfirmed
// record. begun_generation is raised above every generation seen on disk so
// leftovers are recognised as discarded and later generations stay monotonic.
ReconciliationStatus planReconciliation(const PersistenceInputs& inputs,
                                        ReconciliationAction action, uint32_t generation,
                                        SaveMarker* out);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_STATE_H
