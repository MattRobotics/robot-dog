#include "OperationalReadiness.h"

namespace matdog {
namespace core {
namespace {
constexpr uint32_t kRetryMs = 5000;      // transient restore/admission refusals
constexpr uint32_t kPreflightRetryMs = 15000;
bool due(bool attempted, uint32_t since, uint32_t now, uint32_t period) {
  return !attempted || static_cast<uint32_t>(now - since) >= period;
}
}  // namespace

const char* OperationalReadiness::authorityBlocker(const ReadinessFacts& f) {
  if (f.health_fault) return "HEALTH_FAULT";
  if (f.inhibited) return "AUTHORITY_INHIBITED";
  if (!f.census_complete || !f.census_pass) return "SERVO_CENSUS";
  if (!f.preflight_pass) return "SERVO_PREFLIGHT";
  if (!f.restored) return "RESTORE_REQUIRED";
  if (!f.raw_fresh) return "RAW_FEEDBACK_LOST";
  if (!f.mapped_fresh) return "JOINT_FEEDBACK_LOST";
  if (!f.imu_fresh) return "IMU_STALE";
  if (!f.bms_ok) return "BMS";
  if (!f.power_ok) return "POWER";
  if (!f.operational_bound) return "OPERATIONAL_LIMITS";
  if (f.mode != OperatingMode::RUN) return "MODE_NOT_RUN";
  return nullptr;
}

ReadinessAction OperationalReadiness::step(const ReadinessFacts& f) {
  auto& s = status_;
  s.joint_feedback = f.mapped_fresh ? "AVAILABLE" : f.raw_fresh ? "RAW_ONLY" : "UNAVAILABLE";
  s.calibration = f.restored ? "RESTORED" : f.restore_refused_permanent ? "REFUSED" : "NOT_RESTORED";
  s.system_health = f.health_fault ? "FAULT" : "BOOTING";
  auto hold = [&](ReadinessPhase phase, const char* reason) {
    s.phase = phase; s.reason = reason;
    s.motion_authority = was_granted_ ? "REVOKED" : "NOT_GRANTED";
    if (phase == ReadinessPhase::BLOCKED || phase == ReadinessPhase::MAINTENANCE_HOLD) s.system_health = f.health_fault ? "FAULT" : "DEGRADED";
    return ReadinessAction::NONE;
  };

  // 1. Supervision of an existing grant outranks everything else.
  const char* blocker = authorityBlocker(f);
  if (f.authority_motion) {
    if (blocker) {
      ++revocations_;
      s.phase = ReadinessPhase::BLOCKED; s.reason = blocker; s.motion_authority = "REVOKED";
      s.system_health = f.health_fault ? "FAULT" : "DEGRADED";
      return f.session_active ? ReadinessAction::REVOKE_SESSION : ReadinessAction::RELEASE_AUTHORITY;
    }
    s.phase = ReadinessPhase::READY; s.reason = "OK"; s.system_health = "READY"; s.motion_authority = "GRANTED";
    return ReadinessAction::NONE;
  }
  if (f.health_fault) return hold(ReadinessPhase::BLOCKED, "HEALTH_FAULT");

  // 2. Boot sequence, re-entered automatically after any loss.
  if (!f.boot_safe_off_done) return hold(ReadinessPhase::BOOTING, "BOOT_SAFE_OFF_PENDING");
  if (!f.census_complete) return hold(ReadinessPhase::CENSUS, "SERVO_CENSUS_RUNNING");
  if (!f.census_pass) return hold(ReadinessPhase::BLOCKED, "SERVO_CENSUS_INCOMPLETE");
  if (!f.preflight_complete || !f.preflight_pass) {
    if (f.preflight_running) return hold(ReadinessPhase::PREFLIGHT, "SERVO_PREFLIGHT_RUNNING");
    if (f.preflight_complete && !f.preflight_pass && !due(preflight_requested_, preflight_requested_ms_, f.now_ms, kPreflightRetryMs))
      return hold(ReadinessPhase::BLOCKED, "SERVO_PREFLIGHT_FAILED");
    if (f.mode != OperatingMode::MAINTENANCE) return hold(ReadinessPhase::BLOCKED, "PREFLIGHT_NEEDS_MAINTENANCE");
    preflight_requested_ = true; preflight_requested_ms_ = f.now_ms;
    hold(ReadinessPhase::PREFLIGHT, "SERVO_PREFLIGHT_STARTED");
    return ReadinessAction::START_PREFLIGHT;
  }
  if (!f.persistence_loaded) return hold(ReadinessPhase::RESTORE, "PERSISTENCE_LOAD_PENDING");
  if (!f.restored) {
    if (f.restore_refused_permanent) return hold(ReadinessPhase::BLOCKED, f.restore_reason && *f.restore_reason ? f.restore_reason : "RESTORE_REFUSED");
    if (!due(restore_attempted_, restore_attempt_ms_, f.now_ms, kRetryMs))
      return hold(ReadinessPhase::RESTORE, f.restore_refused_transient && f.restore_reason ? f.restore_reason : "RESTORE_PENDING");
    if (f.mode != OperatingMode::MAINTENANCE) return hold(ReadinessPhase::BLOCKED, "RESTORE_NEEDS_MAINTENANCE");
    restore_attempted_ = true; restore_attempt_ms_ = f.now_ms;
    hold(ReadinessPhase::RESTORE, "RESTORE_ATTEMPT");
    return ReadinessAction::RESTORE;
  }
  if (!f.raw_fresh || !f.mapped_fresh) return hold(ReadinessPhase::RESTORE, f.raw_fresh ? "JOINT_FEEDBACK_PENDING" : "RAW_FEEDBACK_PENDING");
  if (!f.operational_bound) {
    if (!due(admit_attempted_, admit_attempt_ms_, f.now_ms, kRetryMs))
      return hold(ReadinessPhase::ADMIT, f.admit_failed && f.admit_reason ? f.admit_reason : "ADMIT_PENDING");
    admit_attempted_ = true; admit_attempt_ms_ = f.now_ms;
    hold(ReadinessPhase::ADMIT, "ADMIT_ATTEMPT");
    return ReadinessAction::ADMIT_OPERATIONAL;
  }
  if (f.mode != OperatingMode::RUN) {
    if (f.operator_maintenance_hold) return hold(ReadinessPhase::MAINTENANCE_HOLD, "OPERATOR_MAINTENANCE");
    if (f.authority_other) return hold(ReadinessPhase::RUN_ENTRY, "SERVICE_AUTHORITY_HELD");
    hold(ReadinessPhase::RUN_ENTRY, "ENTERING_RUN");
    return ReadinessAction::ENTER_RUN;
  }
  if (blocker) return hold(ReadinessPhase::BLOCKED, blocker);
  if (f.authority_other) return hold(ReadinessPhase::BLOCKED, "SERVICE_AUTHORITY_HELD");
  ++grants_; was_granted_ = true;
  s.phase = ReadinessPhase::READY; s.reason = "AUTHORITY_GRANT"; s.system_health = "READY"; s.motion_authority = "GRANTED";
  return ReadinessAction::GRANT_AUTHORITY;
}

const char* toString(ReadinessAction a) {
  switch (a) {
    case ReadinessAction::NONE: return "NONE";
    case ReadinessAction::START_PREFLIGHT: return "START_PREFLIGHT";
    case ReadinessAction::RESTORE: return "RESTORE";
    case ReadinessAction::ADMIT_OPERATIONAL: return "ADMIT_OPERATIONAL";
    case ReadinessAction::ENTER_RUN: return "ENTER_RUN";
    case ReadinessAction::GRANT_AUTHORITY: return "GRANT_AUTHORITY";
    case ReadinessAction::RELEASE_AUTHORITY: return "RELEASE_AUTHORITY";
    case ReadinessAction::REVOKE_SESSION: return "REVOKE_SESSION";
  }
  return "UNKNOWN";
}
const char* toString(ReadinessPhase p) {
  switch (p) {
    case ReadinessPhase::BOOTING: return "BOOTING";
    case ReadinessPhase::CENSUS: return "CENSUS";
    case ReadinessPhase::PREFLIGHT: return "PREFLIGHT";
    case ReadinessPhase::RESTORE: return "RESTORE";
    case ReadinessPhase::ADMIT: return "ADMIT";
    case ReadinessPhase::RUN_ENTRY: return "RUN_ENTRY";
    case ReadinessPhase::READY: return "READY";
    case ReadinessPhase::BLOCKED: return "BLOCKED";
    case ReadinessPhase::MAINTENANCE_HOLD: return "MAINTENANCE_HOLD";
  }
  return "UNKNOWN";
}

}  // namespace core
}  // namespace matdog
