#ifndef MATDOG_CALIBRATION_CALIBRATION_MOTION_PERMIT_H
#define MATDOG_CALIBRATION_CALIBRATION_MOTION_PERMIT_H

#include <stdint.h>

#include "../core/ActuatorAuthority.h"
#include "../core/OperatingMode.h"
#include "../core/SystemState.h"
#include "CalibrationDomain.h"

namespace matdog {
namespace calibration {

// CR3-M4: temporary calibration-only physical-motion permission.
//
// This is deliberately NOT the global hardware_motion_authorized state.
// It is RAM-only and bound to one live calibration session plus one current
// ActuatorAuthority lease generation. It has no persistence API.
//
// A reboot/default construction therefore starts revoked. Session completion,
// abort/failure, authority loss, mode change, inhibit/fault or explicit revoke
// all make the permit unusable. SAFE_OFF is outside this type entirely.
enum class CalibrationPermitStatus : uint8_t {
  REVOKED = 0,
  ACTIVE = 1,
  REJECT_NO_OPERATOR_AUTH = 2,
  REJECT_PROFILE = 3,
  REJECT_MODE = 4,
  REJECT_SYSTEM_HEALTH = 5,
  REJECT_SESSION = 6,
  REJECT_POPULATION = 7,
  REJECT_GEOMETRY = 8,
  REJECT_TRANSFORMS = 9,
  REJECT_AUTHORITY = 10,
  REJECT_INHIBITED = 11,
};

enum class CalibrationPermitRevokeReason : uint8_t {
  NONE = 0,
  EXPLICIT = 1,
  SESSION_ENDED = 2,
  AUTHORITY_LOST = 3,
  MODE_INCOMPATIBLE = 4,
  SYSTEM_FAULT = 5,
  INHIBITED = 6,
  RESET = 7,
};

struct CalibrationMotionPermitFacts {
  bool explicit_operator_authorization = false;
  bool robot_powered_profile = false;
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
  core::SystemHealth system_health = core::SystemHealth::BOOTING;

  bool session_active = false;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  uint32_t session_id = 0;

  bool current_population_pass = false;
  bool current_geometry_bound = false;
  bool promoted_transforms_complete = false;

  core::ActuatorAuthority authority = core::ActuatorAuthority::NONE;
  uint32_t authority_generation = 0;
  bool authority_inhibited = false;
};

struct CalibrationMotionPermitToken {
  uint32_t permit_generation = 0;
  uint32_t session_id = 0;
  uint32_t authority_generation = 0;

  bool valid() const {
    return permit_generation != 0 && session_id != 0 && authority_generation != 0;
  }
};

class CalibrationMotionPermit {
 public:
  // Explicit grant only. A grant never happens implicitly from facts becoming
  // healthy; the operator-authorization fact must be true on this call.
  CalibrationPermitStatus grant(const CalibrationMotionPermitFacts& facts,
                                CalibrationMotionPermitToken* out_token);

  // Re-checks every dynamic prerequisite against the exact token that was
  // granted. Does not renew or extend a permit.
  CalibrationPermitStatus check(const CalibrationMotionPermitFacts& facts,
                                const CalibrationMotionPermitToken& token) const;

  void revoke(CalibrationPermitRevokeReason reason);
  void reset();

  bool active() const { return active_; }
  uint32_t generation() const { return generation_; }
  CalibrationPermitRevokeReason lastRevokeReason() const { return last_revoke_reason_; }

 private:
  CalibrationPermitStatus evaluateFacts(const CalibrationMotionPermitFacts& facts) const;

  bool active_ = false;
  uint32_t generation_ = 0;
  uint32_t bound_session_id_ = 0;
  uint32_t bound_authority_generation_ = 0;
  CalibrationPermitRevokeReason last_revoke_reason_ =
      CalibrationPermitRevokeReason::RESET;
};

const char* toString(CalibrationPermitStatus status);
const char* toString(CalibrationPermitRevokeReason reason);

}  // namespace calibration
}  // namespace matdog

#endif
