#ifndef MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_STATE_H
#define MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_STATE_H

#include <stddef.h>
#include <stdint.h>

#include "CalibrationRecord.h"
#include "CalibrationSaveMarker.h"

// PERSISTENCE STATE - what the two record slots and the save marker, read
// together after a (re)boot, say about the stored calibration (P2.4, P2.4.1).
//
// Pure: no Arduino, no NVS, no storage access. The inputs are observations the
// store already made; the output is a verdict plus the explicit reconciliation
// and acknowledgment the contract allows. Nothing here repairs, promotes or
// erases anything.
//
// RULES.
//   - The marker is the only authority on what the CALLER ACKNOWLEDGED. A valid
//     record is evidence, not acknowledgment: a record newer than the marker, or
//     one the marker only calls "verified, awaiting ACK", is never promoted and
//     never served. Only an explicit ACK (or an explicit reconciliation) does.
//   - The acknowledged generation's slot is protected: a SAVE is refused in every
//     state in which an unacknowledged generation or an unresolved doubt exists.
//   - If the marker attests acknowledged generation G and only an older one
//     survives, the storage is NOT healthy: ACKNOWLEDGED_GENERATION_LOST. This
//     takes priority over every in-flight state.
//   - While a SAVE is PENDING, or a generation awaits its ACK, nothing may be
//     overwritten and nothing is served until ACK or explicit reconciliation.
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
  // Marker IDLE with no acknowledged generation (a first SAVE was explicitly
  // discarded, or a lost acknowledgment was explicitly reconciled). No
  // calibration available.
  NOTHING_ACKNOWLEDGED,
  // Marker IDLE and the record of the acknowledged generation is valid. The only
  // healthy state with data.
  CONSISTENT,
  // Marker PENDING and the record of the pending generation is absent/damaged.
  PENDING_RECORD_ABSENT,
  // Marker PENDING and the record of the pending generation is valid: a SAVE
  // that was written but never reached its verified marker. NOT promoted.
  PENDING_RECORD_PRESENT,
  // Marker AWAITING_ACK, the record of that generation is valid and the
  // acknowledged generation (if any) is intact: the SAVE finished, the caller has
  // not acknowledged it. NOT promoted; the previous generation stays protected.
  // The one state in which acknowledge() is allowed.
  AWAITING_ACK,
  // Marker AWAITING_ACK but the record it vouches for is absent/damaged.
  AWAITING_ACK_RECORD_LOST,
  // The marker attests acknowledged generation G; no valid record has generation G.
  // `older_record_survives` says whether an older valid record remains.
  ACKNOWLEDGED_GENERATION_LOST,
  // A valid record has a generation the marker never attested nor left pending.
  RECORD_AHEAD_OF_MARKER,
  // Both slots hold valid records with the same generation.
  RECORD_GENERATION_CONFLICT,
  // Records present, no marker (a layout from before the marker, a lost
  // marker, or a wiped marker): nothing is acknowledged.
  MARKER_MISSING,
  MARKER_CORRUPT,
  // Marker written by another schema: this build must not interpret or replace it.
  MARKER_INCOMPATIBLE,
  // The record of the acknowledged (or a newer) generation is intact but belongs to
  // another schema / geometry / installation.
  RECORD_INCOMPATIBLE,
};
const char* toString(PersistenceClass cls);

// Explicit, operator-driven reconciliation. planReconciliation() only says which
// marker the action would leave behind. Adopting the generation that awaits its
// ACK is an operator acknowledgment; adopting another one discards it.
enum class ReconciliationAction : uint8_t {
  // Adopt the valid record of generation `g` as the acknowledged one.
  ADOPT_VALID_RECORD = 1,
  // Declare that no stored generation is acknowledged (fresh calibration required).
  DECLARE_NOTHING_ACKNOWLEDGED = 2,
};
constexpr uint8_t kReconcileAdoptBit = 1u << 0;
constexpr uint8_t kReconcileDeclareBit = 1u << 1;
const char* toString(ReconciliationAction action);

struct PersistenceAssessment {
  PersistenceClass cls = PersistenceClass::IO_ERROR;

  // A new SAVE may start: only NEVER_INITIALIZED_OR_ERASED, NOTHING_ACKNOWLEDGED,
  // CONSISTENT. Every other class refuses until acknowledged / reconciled.
  bool save_allowed = false;
  // The stored calibration may be loaded as evidence: only CONSISTENT.
  bool record_available = false;
  // acknowledge() may record an ACK: only AWAITING_ACK.
  bool ack_allowed = false;

  // Facts, valid when the marker is VALID (0 otherwise).
  uint32_t acknowledged_generation = 0;  // marker.acknowledged_generation
  uint32_t pending_generation = 0;    // marker.begun_generation while PENDING
  uint32_t awaiting_generation = 0;   // marker.begun_generation while AWAITING_ACK
  // The valid record of acknowledged_generation exists (also meaningful when the
  // class is PENDING_* / AWAITING_*: the previous acknowledgment is still on disk).
  bool acknowledged_record_intact = false;
  CalibrationSlot acknowledged_slot = CalibrationSlot::A;
  // ACKNOWLEDGED_GENERATION_LOST: a valid record of an older generation remains.
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
// NOTHING_ACKNOWLEDGED (DECLARE) - never anything that serves an unacknowledged
// record. begun_generation is raised above every generation seen on disk so
// leftovers are recognised as discarded and later generations stay monotonic.
// ADOPT never promotes a corrupt, incompatible or semantically invalid record
// (only slots observed VALID qualify) nor an ambiguous duplicate.
ReconciliationStatus planReconciliation(const PersistenceInputs& inputs,
                                        ReconciliationAction action, uint32_t generation,
                                        SaveMarker* out);

enum class AckPlanStatus : uint8_t {
  OK = 0,                    // *out is the marker that records the ACK
  ALREADY_ACKNOWLEDGED,      // idempotent: this exact generation is already acknowledged
  BAD_ARGUMENT,              // null out / generation 0
  STORAGE_UNUSABLE,          // storage could not be read
  NOT_AWAITING,              // no generation awaits an ACK
  GENERATION_MISMATCH,       // another generation is the one in question
  RECORD_NOT_VALID,          // that generation's record is not a verified, valid one
  RECONCILIATION_REQUIRED,   // unresolved state: only explicit reconciliation helps
};
const char* toString(AckPlanStatus status);

// Pure decision for the caller's acknowledgment of `generation`. OK only from the
// AWAITING_ACK class with generation == the awaited one; *out is then
// IDLE(acknowledged = generation, begun = generation). The previous acknowledged
// slot becomes reusable only once that marker is persistent. Nothing else ever
// yields OK, so a lost, corrupt, mismatching or premature ACK cannot promote
// anything.
AckPlanStatus planAcknowledgment(const PersistenceInputs& inputs, uint32_t generation,
                                 SaveMarker* out);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_PERSISTENCE_STATE_H
