#include "CommandRouter.h"

#include <stdlib.h>
#include <string.h>

#include "../actuator/ActuatorWritePolicy.h"
#include "../calibration/CalibrationPersistenceService.h"
#include "../calibration/CalibrationSaveGate.h"
#include "../calibration/CalibrationMotionPermit.h"
#include "../calibration/FirstMotionExecutor.h"
#include "../calibration/FullLegCalibrationExecutor.h"
#include "../calibration/FullLegCalibrationFinalizer.h"
#include "../config/BuildConfig.h"

// @CALIBRATION PERSIST ... (P3a). The only code in the router that talks to
// the persistence service. It never touches the servo bus, the JointTransform
// table, the authority arbiter or the motion permit: the SAVE gate reads them,
// the service writes NVS, and nothing here restores or admits a calibration.

namespace matdog {
namespace core {

namespace {

constexpr const char* kSaveConfirmToken = "CONFIRM_SAVE_FULL_CALIBRATION";
constexpr const char* kDiscardConfirmToken = "CONFIRM_DISCARD";
constexpr size_t kMaxTokens = 6;

// Splits the already-uppercased line into space separated tokens. Returns the
// token count, or kMaxTokens + 1 when there are too many / the line is too long.
size_t tokenize(const char* text, char* storage, size_t storage_size, const char** tokens) {
  const size_t len = strlen(text);
  if (len >= storage_size) return kMaxTokens + 1;
  memcpy(storage, text, len + 1);
  size_t count = 0;
  char* cursor = storage;
  while (*cursor != '\0') {
    while (*cursor == ' ') ++cursor;
    if (*cursor == '\0') break;
    if (count == kMaxTokens) return kMaxTokens + 1;
    tokens[count++] = cursor;
    while (*cursor != '\0' && *cursor != ' ') ++cursor;
    if (*cursor == ' ') *cursor++ = '\0';
  }
  return count;
}

// Plain decimal generation, 1..UINT32_MAX. Anything else is rejected: no sign,
// no hex, no trailing junk.
bool parseGeneration(const char* token, uint32_t* out) {
  if (token == nullptr || *token == '\0' || strlen(token) > 10) return false;
  for (const char* c = token; *c != '\0'; ++c) {
    if (*c < '0' || *c > '9') return false;
  }
  const unsigned long long value = strtoull(token, nullptr, 10);
  if (value == 0 || value > 0xFFFFFFFFull) return false;
  *out = static_cast<uint32_t>(value);
  return true;
}

void printUsage() {
  Serial.println("USAGE=@CALIBRATION PERSIST STATUS | SAVE CHECK | "
                 "SAVE CONFIRM_SAVE_FULL_CALIBRATION | ACK <generation> | "
                 "RECONCILE ADOPT <generation> [CONFIRM_DISCARD] | "
                 "RECONCILE DECLARE_NOTHING [CONFIRM_DISCARD]");
}

}  // namespace

const char* CommandRouter::persistenceQuietViolation() const {
  if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
    return "NOT_IN_MAINTENANCE_MODE";
  }
  if (modules_.calibration->sessionLive()) return "CALIBRATION_SESSION_LIVE";
  if (modules_.full_leg_run->armed) return "LEG_RUN_NOT_FINALIZED";
  if (motionExecutorBusy()) return "MOTION_EXECUTOR_ACTIVE";
  if (q0CaptureOwnsServoDiagnostics()) return "Q0_CAPTURE_ACTIVE";
  if (servoDiagnosticBusy()) return "SERVO_DIAGNOSTIC_BUSY";
  if (modules_.authority->current() != ActuatorAuthority::NONE) return "AUTHORITY_NOT_NONE";
  if (modules_.motion_permit->active()) return "MOTION_PERMIT_ACTIVE";
  if (modules_.motion_authorization->operator_authorized ||
      modules_.motion_authorization->token.valid()) {
    return "OPERATOR_AUTHORIZATION_ACTIVE";
  }
  return nullptr;
}

void CommandRouter::buildSaveGateFacts(calibration::SaveGateFacts* f) const {
  const calibration::PersistenceSnapshot& snap = modules_.persistence->snapshot();
  f->persistence_ready = snap.nvs == calibration::NvsInitStatus::READY;
  f->storage_save_allowed = snap.assessment.save_allowed;
  f->writes_blocked = modules_.persistence->writesBlocked();

  f->maintenance_mode = modules_.operating_mode->mode() == OperatingMode::MAINTENANCE;
  f->session_live = modules_.calibration->sessionLive();
  f->leg_run_armed = modules_.full_leg_run->armed;
  f->motion_executor_active = motionExecutorBusy();
  f->q0_capture_active = q0CaptureOwnsServoDiagnostics();
  f->servo_diagnostic_busy = servoDiagnosticBusy();
  f->authority_none = modules_.authority->current() == ActuatorAuthority::NONE;
  f->motion_permit_active = modules_.motion_permit->active();
  f->operator_authorized = modules_.motion_authorization->operator_authorized ||
                           modules_.motion_authorization->token.valid();

  // SAFE_OFF evidence without any bus traffic: the Full Calibration executor
  // only reports a finished leg after a verified SAFE_OFF of every joint (that
  // is what the evidence record attests); the first-motion executor leaves
  // drive enabled in COMPLETE / SAFE_OFF_REQUIRED until the Controller's own
  // SAFE_OFF read-back came back VERIFIED_OFF. Anything else is not proof.
  const calibration::FirstMotionState fm = modules_.first_motion->status().state;
  const bool drive_may_remain = fm == calibration::FirstMotionState::COMPLETE ||
                                 fm == calibration::FirstMotionState::SAFE_OFF_REQUIRED;
  f->first_motion_safe_off_proven =
      modules_.first_motion_safe_off_result != nullptr &&
      !modules_.first_motion->active() &&
      (!drive_may_remain ||
       *modules_.first_motion_safe_off_result == servo::SafeOffResult::VERIFIED_OFF);

  f->evidence = modules_.full_leg_evidence;
  f->profile = modules_.geometry_profile;
  f->geometry_tag = modules_.actuator_policy->currentGeometryTag();
  f->build_id = build::kBuildId;

  f->q0_capture_state_complete =
      modules_.q0_capture->status().state == calibration::Q0CaptureState::COMPLETE;
  f->q0_capture = modules_.q0_capture->freshCapture();
  f->transforms = &modules_.actuator_policy->transforms();
}

void CommandRouter::printPersistenceStatus() {
  const calibration::CalibrationPersistenceService& svc = *modules_.persistence;
  const calibration::PersistenceSnapshot& s = svc.snapshot();
  const calibration::PersistenceAssessment& a = s.assessment;

  Serial.println("CALIBRATION_PERSIST=STATUS");
  Serial.printf("NVS=%s ESP_ERROR=%ld PARTITION=%s\n", calibration::toString(s.nvs),
                (long)s.esp_error, calibration::kMatdogNvsPartitionLabel);
  Serial.printf("LOAD_COUNT=%lu LOAD_STATUS=%s BOOT_VERDICT=%s LAST_LOAD_VERDICT=%s DEGRADED=%u\n",
                (unsigned long)s.load_count, calibration::toString(s.load_status),
                calibration::toString(s.boot_verdict), calibration::toString(s.verdict),
                s.degraded ? 1u : 0u);
  Serial.printf("MARKER=%s", calibration::toString(s.marker.state));
  if (s.marker.state == calibration::MarkerObservation::VALID) {
    Serial.printf(" MARKER_STATE=%s", calibration::toString(s.marker.marker.state));
  } else if (s.marker.state == calibration::MarkerObservation::CORRUPT ||
             s.marker.state == calibration::MarkerObservation::INCOMPATIBLE) {
    Serial.printf(" MARKER_DETAIL=%s", calibration::toString(s.marker.detail));
  }
  Serial.println();
  Serial.printf("CLASS=%s ACKNOWLEDGED_GENERATION=%lu PENDING_GENERATION=%lu "
                "AWAITING_ACK_GENERATION=%lu ACKNOWLEDGED_RECORD_INTACT=%u\n",
                calibration::toString(a.cls), (unsigned long)a.acknowledged_generation,
                (unsigned long)a.pending_generation, (unsigned long)a.awaiting_generation,
                a.acknowledged_record_intact ? 1u : 0u);
  for (uint8_t i = 0; i < calibration::kCalibrationSlotCount; ++i) {
    const calibration::SlotReport& slot = s.slot[i];
    Serial.printf("SLOT_%s=%s GENERATION_HINT=%lu DETAIL=%s\n",
                  calibration::toString(static_cast<calibration::CalibrationSlot>(i)),
                  calibration::toString(slot.state), (unsigned long)slot.generation_hint,
                  calibration::toString(slot.detail));
  }
  Serial.printf("STORAGE_SAVE_ALLOWED=%u ACK_ALLOWED=%u RECONCILIATION_REQUIRED=%u "
                "ALLOWED_ACTIONS=%s%s%s\n",
                a.save_allowed ? 1u : 0u, a.ack_allowed ? 1u : 0u,
                a.reconciliation_required ? 1u : 0u,
                (a.allowed_actions & calibration::kReconcileAdoptBit) ? "ADOPT " : "",
                (a.allowed_actions & calibration::kReconcileDeclareBit) ? "DECLARE_NOTHING" : "",
                a.allowed_actions == 0 ? "NONE" : "");
  Serial.printf("WRITE_STATE=%s ACK_UNCERTAIN=%u RECONCILE_UNCERTAIN=%u WRITES_BLOCKED=%u\n",
                calibration::toString(svc.storeWriteState()), svc.ackUncertain() ? 1u : 0u,
                svc.reconcileUncertain() ? 1u : 0u, svc.writesBlocked() ? 1u : 0u);
  if (svc.writesBlocked()) {
    Serial.println("WRITE_BLOCK_NOTE=a write outcome is uncertain; only a reboot re-reads and clears it");
  }
  Serial.printf("CALIBRATION_AVAILABLE=%u MOTION_AUTHORIZED=0 RESTORE=NOT_IMPLEMENTED\n",
                svc.calibrationAvailable() ? 1u : 0u);
  Serial.println("CALIBRATION_PERSIST_NOTE available is evidence, not a motion permit; "
                 "no JointTransform is admitted from storage");
}

void CommandRouter::handlePersistCommand(const String& upper) {
  if (modules_.persistence == nullptr) {
    Serial.println("CALIBRATION_PERSIST=BLOCKED");
    Serial.println("REASON=PERSISTENCE_NOT_BOUND");
    return;
  }

  static const char kPrefix[] = "@CALIBRATION PERSIST";
  char storage[96];
  const char* tokens[kMaxTokens];
  const size_t count = tokenize(upper.c_str() + (sizeof(kPrefix) - 1), storage, sizeof(storage), tokens);
  if (count > kMaxTokens || count == 0) {
    Serial.println("CALIBRATION_PERSIST=REFUSED");
    Serial.println("REASON=USAGE");
    printUsage();
    return;
  }
  calibration::CalibrationPersistenceService& svc = *modules_.persistence;
  const actuator::CalibrationGeometryProfile& profile = *modules_.geometry_profile;

  if (count == 1 && strcmp(tokens[0], "STATUS") == 0) {
    printPersistenceStatus();
    return;
  }

  if (strcmp(tokens[0], "SAVE") == 0 && count == 2) {
    const bool check_only = strcmp(tokens[1], "CHECK") == 0;
    if (!check_only && strcmp(tokens[1], kSaveConfirmToken) != 0) {
      Serial.println("CALIBRATION_PERSIST_SAVE=REFUSED");
      Serial.println("REASON=USAGE_OR_CONFIRMATION");
      printUsage();
      return;
    }
    // Facts are sampled once and the gate evaluated on exactly them. The
    // record it builds lands in the service-owned scratch (no second buffer).
    calibration::SaveGateFacts facts;
    buildSaveGateFacts(&facts);
    const calibration::SaveGateResult gate =
        calibration::evaluateSaveGate(facts, svc.scratchRecord());
    if (!gate.ok()) {
      Serial.printf("CALIBRATION_PERSIST_SAVE=%s\n", check_only ? "CHECK_REFUSED" : "REFUSED");
      Serial.printf("REASON=%s\n", calibration::toString(gate.reason));
      if (gate.reason == calibration::SaveGateReason::RECORD_NOT_BUILDABLE ||
          gate.reason == calibration::SaveGateReason::RECORD_NOT_VALID) {
        Serial.printf("RECORD_STATUS=%s\n", calibration::toString(gate.record_status));
      }
      if (gate.reason == calibration::SaveGateReason::RECORD_Q0_MISMATCH) {
        Serial.printf("JOINT_INDEX=%u\n", (unsigned)gate.joint_index);
      }
      Serial.println("PERSISTED=0");
      return;
    }
    if (check_only) {
      Serial.println("CALIBRATION_PERSIST_SAVE=CHECK_OK");
      Serial.println("NOTE every SAVE prerequisite is proven now; nothing was written. "
                     "SAVE CONFIRM_SAVE_FULL_CALIBRATION would write the next "
                     "generation to NVS; an ACK is required afterwards");
      Serial.println("PERSISTED=0");
      return;
    }

    const calibration::ServiceSaveResult r = svc.save(*svc.scratchRecord(), profile);
    if (r.guard != calibration::ServiceGuard::OK) {
      Serial.println("CALIBRATION_PERSIST_SAVE=REFUSED");
      Serial.printf("REASON=%s\n", calibration::toString(r.guard));
      Serial.println("PERSISTED=0");
      return;
    }
    if (r.save.status == calibration::SaveStatus::OK) {
      // Verified on flash, NOT concluded: the generation is protected and
      // unusable until the operator acknowledges it.
      Serial.printf("CALIBRATION_PERSIST_SAVE=WRITTEN_AWAITING_ACK generation=%lu slot=%s\n",
                    (unsigned long)r.save.generation, calibration::toString(r.save.slot));
      Serial.printf("ACK_REQUIRED=@CALIBRATION PERSIST ACK %lu\n",
                    (unsigned long)r.save.generation);
      Serial.println("SAVE_CONCLUDED=0 CALIBRATION_AVAILABLE=0 MOTION_AUTHORIZED=0");
      return;
    }
    Serial.printf("CALIBRATION_PERSIST_SAVE=%s\n", r.uncertain ? "UNCERTAIN" : "FAILED");
    Serial.printf("REASON=%s PHASE=%s IO=%s PERSISTENCE=%s\n", calibration::toString(r.save.status),
                  calibration::toString(r.save.phase), calibration::toString(r.save.io),
                  calibration::toString(r.save.persistence));
    if (r.save.status == calibration::SaveStatus::INVALID_RECORD) {
      Serial.printf("RECORD_STATUS=%s\n", calibration::toString(r.save.validation));
    }
    if (r.uncertain) {
      Serial.println("WRITES_BLOCKED=1 NOTE the outcome is uncertain; further SAVE/ACK/RECONCILE "
                     "are refused until reboot, then run PERSIST STATUS");
    }
    Serial.println("SAVE_CONCLUDED=0 MOTION_AUTHORIZED=0");
    return;
  }

  if (strcmp(tokens[0], "ACK") == 0) {
    uint32_t generation = 0;
    if (count != 2 || !parseGeneration(tokens[1], &generation)) {
      Serial.println("CALIBRATION_PERSIST_ACK=REFUSED");
      Serial.println("REASON=USAGE_GENERATION_REQUIRED");
      printUsage();
      return;
    }
    if (const char* reason = persistenceQuietViolation()) {
      Serial.println("CALIBRATION_PERSIST_ACK=BLOCKED");
      Serial.printf("REASON=%s\n", reason);
      return;
    }
    const calibration::ServiceAckResult r = svc.acknowledge(generation, profile);
    if (r.guard != calibration::ServiceGuard::OK) {
      Serial.println("CALIBRATION_PERSIST_ACK=BLOCKED");
      Serial.printf("REASON=%s\n", calibration::toString(r.guard));
      return;
    }
    const char* outcome = "REFUSED";
    switch (r.ack.status) {
      case calibration::AckStatus::OK:
        outcome = "OK";
        break;
      case calibration::AckStatus::ALREADY_ACKNOWLEDGED:
        outcome = "ALREADY_ACKNOWLEDGED";
        break;
      case calibration::AckStatus::GENERATION_MISMATCH:
        outcome = "WRONG_GENERATION";
        break;
      case calibration::AckStatus::RECORD_NOT_VALID:
        outcome = "INVALID_RECORD";
        break;
      case calibration::AckStatus::MARKER_WRITE_FAILED:
      case calibration::AckStatus::MARKER_VERIFY_FAILED:
        outcome = "UNCERTAIN";
        break;
      default:
        break;
    }
    Serial.printf("CALIBRATION_PERSIST_ACK=%s generation=%lu\n", outcome,
                  (unsigned long)generation);
    Serial.printf("REASON=%s IO=%s PERSISTENCE=%s\n", calibration::toString(r.ack.status),
                  calibration::toString(r.ack.io), calibration::toString(r.ack.persistence));
    if (r.uncertain) {
      Serial.println("WRITES_BLOCKED=1 NOTE the ACK outcome is uncertain; reboot and run PERSIST STATUS");
    }
    const calibration::PersistenceSnapshot& snap = svc.snapshot();
    Serial.printf("ACKNOWLEDGED_GENERATION=%lu VERDICT=%s CALIBRATION_AVAILABLE=%u "
                  "MOTION_AUTHORIZED=0\n",
                  (unsigned long)snap.assessment.acknowledged_generation,
                  calibration::toString(snap.verdict), svc.calibrationAvailable() ? 1u : 0u);
    return;
  }

  if (strcmp(tokens[0], "RECONCILE") == 0 && count >= 2) {
    calibration::ReconciliationAction action = calibration::ReconciliationAction::ADOPT_VALID_RECORD;
    uint32_t generation = 0;
    size_t next = 2;
    bool args_ok = false;
    if (strcmp(tokens[1], "ADOPT") == 0) {
      args_ok = count >= 3 && parseGeneration(tokens[2], &generation);
      next = 3;
    } else if (strcmp(tokens[1], "DECLARE_NOTHING") == 0) {
      action = calibration::ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED;
      args_ok = true;
    }
    bool discard_confirmed = false;
    if (args_ok && next < count) {
      if (next + 1 == count && strcmp(tokens[next], kDiscardConfirmToken) == 0) {
        discard_confirmed = true;
      } else {
        args_ok = false;
      }
    }
    if (!args_ok) {
      Serial.println("CALIBRATION_PERSIST_RECONCILE=REFUSED");
      Serial.println("REASON=USAGE_OR_CONFIRMATION");
      printUsage();
      return;
    }
    if (const char* reason = persistenceQuietViolation()) {
      Serial.println("CALIBRATION_PERSIST_RECONCILE=BLOCKED");
      Serial.printf("REASON=%s\n", reason);
      return;
    }
    const calibration::ReconcileResult r =
        svc.reconcile(action, generation, discard_confirmed, profile);
    Serial.printf("CALIBRATION_PERSIST_RECONCILE=%s\n",
                  r.status == calibration::ReconcileStatus::OK ? "OK" : "REFUSED");
    Serial.printf("ACTION=%s REASON=%s PLAN=%s\n", calibration::toString(action),
                  calibration::toString(r.status), calibration::toString(r.plan));
    Serial.printf("BEFORE=%s AFTER=%s DISCARDS=%u ACKNOWLEDGED_BEFORE=%lu\n",
                  calibration::toString(r.before), calibration::toString(r.after),
                  r.discards ? 1u : 0u, (unsigned long)r.acknowledged_before);
    if (r.status == calibration::ReconcileStatus::DISCARD_NOT_CONFIRMED) {
      Serial.println("NOTE this would discard a generation; repeat with CONFIRM_DISCARD. "
                     "Nothing was written");
    }
    if (r.uncertain) {
      Serial.println("WRITES_BLOCKED=1 NOTE the marker outcome is uncertain; reboot and run PERSIST STATUS");
    }
    Serial.println("MOTION_AUTHORIZED=0");
    return;
  }

  Serial.println("CALIBRATION_PERSIST=REFUSED");
  Serial.println("REASON=USAGE");
  printUsage();
}

}  // namespace core
}  // namespace matdog
