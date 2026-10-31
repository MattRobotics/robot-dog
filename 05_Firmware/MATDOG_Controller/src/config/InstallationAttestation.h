#ifndef MATDOG_CONFIG_INSTALLATION_ATTESTATION_H
#define MATDOG_CONFIG_INSTALLATION_ATTESTATION_H
// Boot RESTORE configuration (operator directive 2026-10-14): AUTOMATIC.
// No fingerprint pin or attestation is required; the live read-only evidence
// (preflight 12/12, torque OFF, raw inside recorded spans) is the gate.
// A maintenance procedure may suspend it in RAM (@CALIBRATION AUTO_RESTORE
// SUSPEND); nothing here needs a rebuild to make the robot usable.
#include "../calibration/RestoreAttestationPolicy.h"
namespace matdog { namespace config {
constexpr bool kAutoRestoreEnabled = true;
inline calibration::InstallationAttestation installationAttestation(bool ram_suspended = false) {
  calibration::InstallationAttestation a;
  a.auto_restore_enabled = kAutoRestoreEnabled && !ram_suspended;
  return a;
}
} }
#endif
