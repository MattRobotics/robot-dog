#ifndef MATDOG_CALIBRATION_CALIBRATION_SAVE_GATE_H
#define MATDOG_CALIBRATION_CALIBRATION_SAVE_GATE_H

#include <stdint.h>

#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationQ0Bootstrap.h"
#include "CalibrationRecord.h"
#include "FullLegCalibrationFinalizer.h"

// SAVE GATE (P3a) - every physical and logical prerequisite of a persistent SAVE
// of the Full Calibration, evaluated in a fixed order from facts the Controller
// copies in. Pure: no Arduino, no storage, no servo bus; it reads, it never
// admits a transform, never starts anything, never writes.
//
// "Not verifiable" is a refusal: a fact the Controller cannot prove stays at its
// refusing default and the gate names the first prerequisite that is missing.
//
// What it does NOT repeat: the record's own validation. buildCalibrationRecord()
// and validateCalibrationRecord() judge the evidence (24/24, digests, identity,
// encoder direction, geometry, q0 acceptance rules); the gate only calls them
// and adds what no record can say about itself: the live state of the robot and
// that the record's q0 is the q0 of the CURRENT, PROMOTED capture.

namespace matdog {
namespace calibration {

enum class SaveGateReason : uint8_t {
  OK = 0,
  PERSISTENCE_NOT_READY,        // backend not READY (partition missing, init failed...)
  WRITES_BLOCKED,               // an earlier write left the outcome uncertain
  STORAGE_STATE_NOT_SAVEABLE,   // classification forbids a SAVE (pending, awaiting ACK, ...)
  NOT_IN_MAINTENANCE_MODE,
  CALIBRATION_SESSION_LIVE,
  LEG_RUN_NOT_FINALIZED,
  MOTION_EXECUTOR_ACTIVE,
  Q0_CAPTURE_ACTIVE,
  SERVO_DIAGNOSTIC_BUSY,
  AUTHORITY_NOT_NONE,
  MOTION_PERMIT_ACTIVE,
  OPERATOR_AUTHORIZATION_ACTIVE,
  SAFE_OFF_NOT_PROVEN,
  FULL_CALIBRATION_NOT_24_OF_24,
  LEG_RUN_NOT_CLOSED,           // a leg record lacks session_completed/permit_revoked/authority_released
  GEOMETRY_NOT_CURRENT,
  Q0_CAPTURE_NOT_COMPLETE,
  Q0_NOT_PROMOTED,
  RECORD_NOT_BUILDABLE,         // see record_status
  RECORD_NOT_VALID,             // see record_status
  RECORD_NOT_ACCEPTED,          // not calibration_accepted, or claims approved parameters
  RECORD_Q0_MISMATCH,           // see joint_index
};
const char* toString(SaveGateReason reason);

// Everything the gate may look at. Defaults refuse.
struct SaveGateFacts {
  // persistence
  bool persistence_ready = false;
  bool storage_save_allowed = false;
  bool writes_blocked = true;
  // robot state
  bool maintenance_mode = false;
  bool session_live = true;
  bool leg_run_armed = true;
  bool motion_executor_active = true;      // first motion or Full Calibration leg executor
  bool q0_capture_active = true;
  bool servo_diagnostic_busy = true;
  bool authority_none = false;
  bool motion_permit_active = true;
  bool operator_authorized = true;
  bool first_motion_safe_off_proven = false;
  // evidence
  const FullLegEvidenceStore* evidence = nullptr;
  const actuator::CalibrationGeometryProfile* profile = nullptr;
  actuator::GeometryProvenanceTag geometry_tag = actuator::kNoGeometryProvenance;
  const char* build_id = nullptr;
  // q0
  bool q0_capture_state_complete = false;
  actuator::FreshQ0Capture q0_capture;
  const actuator::JointTransformTable* transforms = nullptr;
};

struct SaveGateResult {
  SaveGateReason reason = SaveGateReason::PERSISTENCE_NOT_READY;
  CalibrationRecordStatus record_status = CalibrationRecordStatus::OK;  // RECORD_NOT_* detail
  uint8_t joint_index = 0xFF;                                          // RECORD_Q0_MISMATCH detail
  bool ok() const { return reason == SaveGateReason::OK; }
};

// Evaluates the prerequisites in the order of SaveGateReason. On OK `*record`
// holds the validated, exportable record (generation 0; the store assigns it).
// On any other result `*record` is unspecified and must not be saved. `record`
// is caller-owned scratch (the persistence service owns the only one).
SaveGateResult evaluateSaveGate(const SaveGateFacts& facts, CalibrationRecord* record);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_SAVE_GATE_H
