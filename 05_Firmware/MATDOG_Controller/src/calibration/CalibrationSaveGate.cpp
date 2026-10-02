#include "CalibrationSaveGate.h"

#include "../actuator/CalibrationQ0EvidencePreparation.h"

namespace matdog {
namespace calibration {

namespace {

SaveGateResult refuse(SaveGateReason reason) {
  SaveGateResult r;
  r.reason = reason;
  return r;
}

bool legClosed(const FullLegRecord* leg, actuator::GeometryProvenanceTag tag) {
  return leg != nullptr && leg->present && leg->session_completed && leg->permit_revoked &&
         leg->authority_released && leg->geometry == tag;
}

}  // namespace

const char* toString(SaveGateReason reason) {
  switch (reason) {
    case SaveGateReason::OK:                            return "OK";
    case SaveGateReason::PERSISTENCE_NOT_READY:         return "PERSISTENCE_NOT_READY";
    case SaveGateReason::WRITES_BLOCKED:                return "WRITES_BLOCKED";
    case SaveGateReason::STORAGE_STATE_NOT_SAVEABLE:    return "STORAGE_STATE_NOT_SAVEABLE";
    case SaveGateReason::NOT_IN_MAINTENANCE_MODE:       return "NOT_IN_MAINTENANCE_MODE";
    case SaveGateReason::CALIBRATION_SESSION_LIVE:      return "CALIBRATION_SESSION_LIVE";
    case SaveGateReason::LEG_RUN_NOT_FINALIZED:         return "LEG_RUN_NOT_FINALIZED";
    case SaveGateReason::MOTION_EXECUTOR_ACTIVE:        return "MOTION_EXECUTOR_ACTIVE";
    case SaveGateReason::Q0_CAPTURE_ACTIVE:             return "Q0_CAPTURE_ACTIVE";
    case SaveGateReason::SERVO_DIAGNOSTIC_BUSY:         return "SERVO_DIAGNOSTIC_BUSY";
    case SaveGateReason::AUTHORITY_NOT_NONE:            return "AUTHORITY_NOT_NONE";
    case SaveGateReason::MOTION_PERMIT_ACTIVE:          return "MOTION_PERMIT_ACTIVE";
    case SaveGateReason::OPERATOR_AUTHORIZATION_ACTIVE: return "OPERATOR_AUTHORIZATION_ACTIVE";
    case SaveGateReason::SAFE_OFF_NOT_PROVEN:           return "SAFE_OFF_NOT_PROVEN";
    case SaveGateReason::FULL_CALIBRATION_NOT_24_OF_24: return "FULL_CALIBRATION_NOT_24_OF_24";
    case SaveGateReason::LEG_RUN_NOT_CLOSED:            return "LEG_RUN_NOT_CLOSED";
    case SaveGateReason::GEOMETRY_NOT_CURRENT:          return "GEOMETRY_NOT_CURRENT";
    case SaveGateReason::Q0_CAPTURE_NOT_COMPLETE:       return "Q0_CAPTURE_NOT_COMPLETE";
    case SaveGateReason::Q0_NOT_PROMOTED:               return "Q0_NOT_PROMOTED";
    case SaveGateReason::RECORD_NOT_BUILDABLE:          return "RECORD_NOT_BUILDABLE";
    case SaveGateReason::RECORD_NOT_VALID:              return "RECORD_NOT_VALID";
    case SaveGateReason::RECORD_NOT_ACCEPTED:           return "RECORD_NOT_ACCEPTED";
    case SaveGateReason::RECORD_Q0_MISMATCH:            return "RECORD_Q0_MISMATCH";
  }
  return "UNKNOWN";
}

SaveGateResult evaluateSaveGate(const SaveGateFacts& f, CalibrationRecord* record) {
  if (record == nullptr) return refuse(SaveGateReason::RECORD_NOT_BUILDABLE);

  if (!f.persistence_ready) return refuse(SaveGateReason::PERSISTENCE_NOT_READY);
  if (f.writes_blocked) return refuse(SaveGateReason::WRITES_BLOCKED);
  if (!f.storage_save_allowed) return refuse(SaveGateReason::STORAGE_STATE_NOT_SAVEABLE);
  if (!f.maintenance_mode) return refuse(SaveGateReason::NOT_IN_MAINTENANCE_MODE);
  if (f.session_live) return refuse(SaveGateReason::CALIBRATION_SESSION_LIVE);
  if (f.leg_run_armed) return refuse(SaveGateReason::LEG_RUN_NOT_FINALIZED);
  if (f.motion_executor_active) return refuse(SaveGateReason::MOTION_EXECUTOR_ACTIVE);
  if (f.q0_capture_active) return refuse(SaveGateReason::Q0_CAPTURE_ACTIVE);
  if (f.servo_diagnostic_busy) return refuse(SaveGateReason::SERVO_DIAGNOSTIC_BUSY);
  if (!f.authority_none) return refuse(SaveGateReason::AUTHORITY_NOT_NONE);
  if (f.motion_permit_active) return refuse(SaveGateReason::MOTION_PERMIT_ACTIVE);
  if (f.operator_authorized) return refuse(SaveGateReason::OPERATOR_AUTHORIZATION_ACTIVE);
  if (!f.first_motion_safe_off_proven) return refuse(SaveGateReason::SAFE_OFF_NOT_PROVEN);

  if (f.evidence == nullptr || f.profile == nullptr || !f.profile->bound() ||
      f.transforms == nullptr) {
    return refuse(SaveGateReason::GEOMETRY_NOT_CURRENT);
  }
  if (!f.evidence->allLegsContactCalibrated()) {
    return refuse(SaveGateReason::FULL_CALIBRATION_NOT_24_OF_24);
  }
  if (f.geometry_tag == actuator::kNoGeometryProvenance ||
      f.geometry_tag != f.profile->provenanceTag()) {
    return refuse(SaveGateReason::GEOMETRY_NOT_CURRENT);
  }
  for (uint8_t l = 0; l < kLegCount; ++l) {
    if (!legClosed(f.evidence->find(static_cast<Leg>(l)), f.geometry_tag)) {
      return refuse(SaveGateReason::LEG_RUN_NOT_CLOSED);
    }
  }

  if (!f.q0_capture_state_complete || !f.q0_capture.complete || f.q0_capture.candidates == nullptr ||
      f.q0_capture.candidate_count != kLegServoSlotCount) {
    return refuse(SaveGateReason::Q0_CAPTURE_NOT_COMPLETE);
  }
  if (f.q0_capture.capture_session_id == 0 ||
      f.promoted_capture_session_id != f.q0_capture.capture_session_id ||
      f.promoted_geometry != f.geometry_tag ||
      !actuator::freshQ0CaptureIsPromoted(f.q0_capture, *f.transforms, f.geometry_tag)) {
    return refuse(SaveGateReason::Q0_NOT_PROMOTED);
  }

  CalibrationRecordSource source;
  source.evidence = f.evidence;
  source.profile = f.profile;
  source.build_id = f.build_id;
  for (uint8_t i = 0; i < f.q0_capture.candidate_count; ++i) {
    const actuator::Q0BootstrapCandidate& c = f.q0_capture.candidates[i];
    const uint8_t index = legSlotIndex(c.evidence.identity.leg, c.evidence.identity.joint);
    if (index >= kRecordJointCount) return refuse(SaveGateReason::Q0_NOT_PROMOTED);
    source.q0_capture[index].estimator = c.evidence.estimator;
    source.q0_capture[index].sample_count = c.sample_count;
    source.q0_capture[index].stability_spread_ticks = c.stability_spread_ticks;
  }

  const CalibrationRecordStatus built = buildCalibrationRecord(source, record);
  if (built != CalibrationRecordStatus::OK) {
    SaveGateResult r = refuse(SaveGateReason::RECORD_NOT_BUILDABLE);
    r.record_status = built;
    return r;
  }
  // The store owns the generation; validation needs a non-zero one, as in save().
  record->generation = 1;
  const CalibrationRecordStatus valid = validateCalibrationRecord(*record, *f.profile);
  record->generation = 0;
  if (valid != CalibrationRecordStatus::OK) {
    SaveGateResult r = refuse(SaveGateReason::RECORD_NOT_VALID);
    r.record_status = valid;
    return r;
  }
  if (record->calibration_accepted != 1 || record->parameters_approved != 0) {
    return refuse(SaveGateReason::RECORD_NOT_ACCEPTED);
  }

  // The record's q0 must be the q0 of THIS promoted capture, joint by joint.
  for (uint8_t i = 0; i < f.q0_capture.candidate_count; ++i) {
    const actuator::Q0BootstrapCandidate& c = f.q0_capture.candidates[i];
    const uint8_t index = legSlotIndex(c.evidence.identity.leg, c.evidence.identity.joint);
    const actuator::JointTransform* held = f.transforms->find(c.evidence.identity, f.geometry_tag);
    if (held == nullptr || record->joint[index].q0_tick != c.evidence.tick ||
        record->joint[index].q0_tick != held->q0_tick) {
      SaveGateResult r = refuse(SaveGateReason::RECORD_Q0_MISMATCH);
      r.joint_index = index;
      return r;
    }
  }

  SaveGateResult ok;
  ok.reason = SaveGateReason::OK;
  return ok;
}

}  // namespace calibration
}  // namespace matdog
