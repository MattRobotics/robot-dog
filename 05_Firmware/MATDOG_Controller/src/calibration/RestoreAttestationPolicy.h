#ifndef MATDOG_CALIBRATION_RESTORE_ATTESTATION_POLICY_H
#define MATDOG_CALIBRATION_RESTORE_ATTESTATION_POLICY_H
// Issue #48: decides WHETHER a RAM-only gen-N RESTORE may run, automatically
// at boot or on an explicit operator request. Pure: plain facts in, decision
// out. No NVS, servo, authority, permit or limit access, and no include of a
// persistence header (audit boundary P3a). The Controller gathers the facts
// and, only on ALLOW_*, calls CalibrationRuntimeRestore::restore() itself.
//
// ST3215 IDs cannot prove which physical unit is mounted. Automatic RESTORE
// therefore needs a reviewed installation attestation pinned in the firmware
// build (record generation + content fingerprint); a manual RESTORE needs a
// present operator who confirms the mechanical installation. Live evidence
// (12/12 read-only preflight, torque OFF, raw inside the recorded spans) is
// required in BOTH cases; it can refuse, never substitute for, attestation.
// A RESTORE never approves JointLimits, never grants authority or a permit.
#include <stdint.h>

namespace matdog {
namespace calibration {

enum class RestoreTrigger : uint8_t { BOOT_AUTOMATIC, OPERATOR_MANUAL };

enum class RestoreDecision : uint8_t {
  ALLOW_AUTOMATIC,          // boot: attested installation + live evidence
  ALLOW_OPERATOR,           // manual: operator confirmation + live evidence
  ALREADY_RESTORED,         // idempotent: same generation already in RAM
  PENDING_OPERATOR_REVIEW,  // valid record, but attestation absent/mismatched
  REFUSED,                  // a precondition failed; reason says which
};

enum class RestoreReason : uint8_t {
  OK,
  NOT_MAINTENANCE,
  RUNTIME_NOT_IDLE,          // executor/session active, authority held or permit live
  NVS_NOT_READY,
  NO_VALID_ACKNOWLEDGED_RECORD,
  WRITE_OUTCOME_UNCERTAIN,
  RECORD_REJECTED,           // inspection failed (geometry, schema, semantics)
  GENERATION_MISMATCH,
  PREFLIGHT_NOT_COMPLETE,
  PREFLIGHT_FAILED,          // fewer than 12/12 expected units/profile/offset
  TORQUE_NOT_OFF,
  LIVE_POSITION_OUTSIDE_RECORD,
  AUTO_RESTORE_DISABLED,
  INSTALLATION_ATTESTATION_ABSENT,
  INSTALLATION_ATTESTATION_MISMATCH,
  OPERATOR_CONFIRMATION_REQUIRED,
};

struct RestoreFacts {
  bool maintenance = false;
  bool runtime_idle = false;  // executor idle, session OFF, authority NONE, no permit
  bool nvs_ready = false;
  bool record_valid_acknowledged = false;  // verdict VALID_ACKNOWLEDGED, intact
  bool write_outcome_uncertain = false;
  bool record_inspection_ok = false;
  uint32_t record_generation = 0;
  uint64_t record_fingerprint = 0;
  bool preflight_complete = false;
  uint8_t preflight_pass = 0;              // of 12
  uint8_t torque_off = 0;                  // of 12, read this boot
  bool live_positions_inside_record = false;
  uint8_t live_position_failed_joint = 12; // 12 = none
  bool already_restored = false;
  uint32_t restored_generation = 0;
};

// Compiled into the build after operator review. present=false by default:
// nothing in this repository fabricates an attestation.
struct InstallationAttestation {
  bool present = false;
  bool auto_restore_enabled = false;
  uint32_t generation = 0;
  uint64_t record_fingerprint = 0;
};

struct RestoreRequestFacts {
  RestoreTrigger trigger = RestoreTrigger::BOOT_AUTOMATIC;
  uint32_t requested_generation = 0;     // manual: what the operator typed/saw
  bool operator_confirmed_installation = false;
};

struct RestoreVerdict {
  RestoreDecision decision = RestoreDecision::REFUSED;
  RestoreReason reason = RestoreReason::NO_VALID_ACKNOWLEDGED_RECORD;
  uint8_t failed_joint = 12;
};

RestoreVerdict evaluateRestore(const RestoreFacts& facts, const InstallationAttestation& attestation,
                               const RestoreRequestFacts& request);
const char* toString(RestoreDecision decision);
const char* toString(RestoreReason reason);

}  // namespace calibration
}  // namespace matdog
#endif
