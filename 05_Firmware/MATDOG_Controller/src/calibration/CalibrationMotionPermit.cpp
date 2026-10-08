#include "CalibrationMotionPermit.h"

namespace matdog {
namespace calibration {

CalibrationPermitStatus CalibrationMotionPermit::evaluateFacts(
    const CalibrationMotionPermitFacts& facts) const {
  if (!facts.explicit_operator_authorization) {
    return CalibrationPermitStatus::REJECT_NO_OPERATOR_AUTH;
  }
  if (!facts.robot_powered_profile) return CalibrationPermitStatus::REJECT_PROFILE;
  if (facts.mode != core::OperatingMode::MAINTENANCE) {
    return CalibrationPermitStatus::REJECT_MODE;
  }
  if (facts.system_health != core::SystemHealth::READY) {
    return CalibrationPermitStatus::REJECT_SYSTEM_HEALTH;
  }
  if (!facts.session_active || facts.origin != CalibrationOrigin::LIVE_SESSION ||
      facts.session_id == 0) {
    return CalibrationPermitStatus::REJECT_SESSION;
  }
  if (!facts.current_population_pass) return CalibrationPermitStatus::REJECT_POPULATION;
  if (!facts.current_geometry_bound) return CalibrationPermitStatus::REJECT_GEOMETRY;
  if (facts.startup_recovery_only ? !facts.startup_reference_qualified : !facts.promoted_transforms_complete)
    return CalibrationPermitStatus::REJECT_TRANSFORMS;
  if (facts.authority != core::ActuatorAuthority::CALIBRATION ||
      facts.authority_generation == 0) {
    return CalibrationPermitStatus::REJECT_AUTHORITY;
  }
  if (facts.authority_inhibited) return CalibrationPermitStatus::REJECT_INHIBITED;
  return CalibrationPermitStatus::ACTIVE;
}

CalibrationPermitStatus CalibrationMotionPermit::grant(
    const CalibrationMotionPermitFacts& facts,
    CalibrationMotionPermitToken* out_token) {
  if (out_token != nullptr) *out_token = CalibrationMotionPermitToken{};

  const CalibrationPermitStatus evaluated = evaluateFacts(facts);
  if (evaluated != CalibrationPermitStatus::ACTIVE) {
    // A failed grant never leaves an older permit alive.
    revoke(CalibrationPermitRevokeReason::EXPLICIT);
    return evaluated;
  }

  ++generation_;
  if (generation_ == 0) ++generation_;
  active_ = true;
  bound_startup_recovery_only_=facts.startup_recovery_only;
  bound_session_id_ = facts.session_id;
  bound_authority_generation_ = facts.authority_generation;
  last_revoke_reason_ = CalibrationPermitRevokeReason::NONE;

  if (out_token != nullptr) {
    out_token->startup_recovery_only=facts.startup_recovery_only;
    out_token->permit_generation = generation_;
    out_token->session_id = bound_session_id_;
    out_token->authority_generation = bound_authority_generation_;
  }
  return CalibrationPermitStatus::ACTIVE;
}

CalibrationPermitStatus CalibrationMotionPermit::check(
    const CalibrationMotionPermitFacts& facts,
    const CalibrationMotionPermitToken& token) {
  if (!active_ || !token.valid() ||
      token.permit_generation != generation_ ||
      token.session_id != bound_session_id_ ||
      token.authority_generation != bound_authority_generation_) {
    return CalibrationPermitStatus::REVOKED;
  }

  if (facts.startup_recovery_only!=bound_startup_recovery_only_ ||
      token.startup_recovery_only!=bound_startup_recovery_only_) {
    revoke(CalibrationPermitRevokeReason::PREREQUISITE_LOST);return CalibrationPermitStatus::REVOKED;
  }
  const CalibrationPermitStatus evaluated = evaluateFacts(facts);
  if (evaluated != CalibrationPermitStatus::ACTIVE) {
    CalibrationPermitRevokeReason reason = CalibrationPermitRevokeReason::PREREQUISITE_LOST;
    switch (evaluated) {
      case CalibrationPermitStatus::REJECT_NO_OPERATOR_AUTH:
        reason = CalibrationPermitRevokeReason::EXPLICIT;
        break;
      case CalibrationPermitStatus::REJECT_MODE:
        reason = CalibrationPermitRevokeReason::MODE_INCOMPATIBLE;
        break;
      case CalibrationPermitStatus::REJECT_SYSTEM_HEALTH:
        reason = CalibrationPermitRevokeReason::SYSTEM_FAULT;
        break;
      case CalibrationPermitStatus::REJECT_SESSION:
        reason = CalibrationPermitRevokeReason::SESSION_ENDED;
        break;
      case CalibrationPermitStatus::REJECT_AUTHORITY:
        reason = CalibrationPermitRevokeReason::AUTHORITY_LOST;
        break;
      case CalibrationPermitStatus::REJECT_INHIBITED:
        reason = CalibrationPermitRevokeReason::INHIBITED;
        break;
      case CalibrationPermitStatus::REVOKED:
      case CalibrationPermitStatus::ACTIVE:
      case CalibrationPermitStatus::REJECT_PROFILE:
      case CalibrationPermitStatus::REJECT_POPULATION:
      case CalibrationPermitStatus::REJECT_GEOMETRY:
      case CalibrationPermitStatus::REJECT_TRANSFORMS:
        break;
    }
    revoke(reason);
    return evaluated;
  }

  if (facts.session_id != bound_session_id_) {
    revoke(CalibrationPermitRevokeReason::SESSION_ENDED);
    return CalibrationPermitStatus::REJECT_SESSION;
  }
  if (facts.authority_generation != bound_authority_generation_) {
    revoke(CalibrationPermitRevokeReason::AUTHORITY_LOST);
    return CalibrationPermitStatus::REJECT_AUTHORITY;
  }
  return CalibrationPermitStatus::ACTIVE;
}

void CalibrationMotionPermit::revoke(CalibrationPermitRevokeReason reason) {
  active_ = false;
  bound_session_id_ = 0;
  bound_authority_generation_ = 0;
  last_revoke_reason_ =
      reason == CalibrationPermitRevokeReason::NONE
          ? CalibrationPermitRevokeReason::EXPLICIT
          : reason;
}

void CalibrationMotionPermit::reset() {
  // Advance the generation as well: a token copied before reset can never
  // become current again even if later session/authority numbers repeat.
  ++generation_;
  if (generation_ == 0) ++generation_;
  active_ = false;
  bound_session_id_ = 0;
  bound_authority_generation_ = 0;
  last_revoke_reason_ = CalibrationPermitRevokeReason::RESET;
}

CalibrationMotionPermitFacts buildCalibrationMotionPermitFacts(
    const CalibrationMotionPermitLiveInputs& inputs) {
  CalibrationMotionPermitFacts facts{};
  facts.explicit_operator_authorization = inputs.operator_calibration_motion_authorized;
  facts.robot_powered_profile = inputs.robot_powered_profile;
  facts.mode = inputs.mode;
  facts.system_health = inputs.system_health;
  facts.session_active = inputs.session_active;
  facts.origin = inputs.origin;
  facts.session_id = inputs.session_id;
  facts.current_population_pass = inputs.current_population_pass;
  facts.current_geometry_bound = inputs.current_geometry_bound;
  facts.promoted_transforms_complete = inputs.promoted_transforms_complete;
  facts.startup_recovery_only=inputs.startup_recovery_only;
  facts.startup_reference_qualified=inputs.startup_reference_qualified;
  facts.authority = inputs.authority;
  facts.authority_generation = inputs.authority_generation;
  facts.authority_inhibited = inputs.authority_inhibited;
  return facts;
}

const char* toString(CalibrationPermitStatus status) {
  switch (status) {
    case CalibrationPermitStatus::REVOKED: return "REVOKED";
    case CalibrationPermitStatus::ACTIVE: return "ACTIVE";
    case CalibrationPermitStatus::REJECT_NO_OPERATOR_AUTH: return "REJECT_NO_OPERATOR_AUTH";
    case CalibrationPermitStatus::REJECT_PROFILE: return "REJECT_PROFILE";
    case CalibrationPermitStatus::REJECT_MODE: return "REJECT_MODE";
    case CalibrationPermitStatus::REJECT_SYSTEM_HEALTH: return "REJECT_SYSTEM_HEALTH";
    case CalibrationPermitStatus::REJECT_SESSION: return "REJECT_SESSION";
    case CalibrationPermitStatus::REJECT_POPULATION: return "REJECT_POPULATION";
    case CalibrationPermitStatus::REJECT_GEOMETRY: return "REJECT_GEOMETRY";
    case CalibrationPermitStatus::REJECT_TRANSFORMS: return "REJECT_TRANSFORMS";
    case CalibrationPermitStatus::REJECT_AUTHORITY: return "REJECT_AUTHORITY";
    case CalibrationPermitStatus::REJECT_INHIBITED: return "REJECT_INHIBITED";
  }
  return "UNKNOWN";
}

const char* toString(CalibrationPermitRevokeReason reason) {
  switch (reason) {
    case CalibrationPermitRevokeReason::NONE: return "NONE";
    case CalibrationPermitRevokeReason::EXPLICIT: return "EXPLICIT";
    case CalibrationPermitRevokeReason::SESSION_ENDED: return "SESSION_ENDED";
    case CalibrationPermitRevokeReason::AUTHORITY_LOST: return "AUTHORITY_LOST";
    case CalibrationPermitRevokeReason::MODE_INCOMPATIBLE: return "MODE_INCOMPATIBLE";
    case CalibrationPermitRevokeReason::SYSTEM_FAULT: return "SYSTEM_FAULT";
    case CalibrationPermitRevokeReason::INHIBITED: return "INHIBITED";
    case CalibrationPermitRevokeReason::RESET: return "RESET";
    case CalibrationPermitRevokeReason::PREREQUISITE_LOST: return "PREREQUISITE_LOST";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
