#ifndef MATDOG_CONFIG_INSTALLATION_ATTESTATION_H
#define MATDOG_CONFIG_INSTALLATION_ATTESTATION_H
// Issue #48 boot-time RESTORE pin. Edited ONLY after the operator reviewed the
// mechanical installation against the record printed at boot as
//   CALIBRATION_BOOT_RESTORE ... generation=<g> fingerprint=<16 hex>
// and decided that this build may restore that exact record into RAM at boot.
// Default: ABSENT. Boot then reports PENDING_OPERATOR_REVIEW and never
// restores by itself. A pin never approves limits, authority or motion.
#include "../calibration/RestoreAttestationPolicy.h"
namespace matdog { namespace config {
constexpr bool kInstallationAttestationPresent = false;
constexpr bool kAutoRestoreEnabled = false;
constexpr uint32_t kAttestedGeneration = 0;
constexpr uint64_t kAttestedRecordFingerprint = 0;
inline calibration::InstallationAttestation installationAttestation() {
  calibration::InstallationAttestation a;
  a.present = kInstallationAttestationPresent;
  a.auto_restore_enabled = kAutoRestoreEnabled;
  a.generation = kAttestedGeneration;
  a.record_fingerprint = kAttestedRecordFingerprint;
  return a;
}
} }
#endif
