#ifndef MATDOG_CORE_OPERATIONAL_READINESS_H
#define MATDOG_CORE_OPERATIONAL_READINESS_H
// Operator directive 2026-10-14 (final): MATDOG must become operational by
// itself when the hardware and the stored calibration are valid.
//
//   POWER ON -> CENSUS 12/12 -> PREFLIGHT 12/12 -> RESTORE GEN-1 (RAM) ->
//   JOINT FEEDBACK 12/12 -> OPERATIONAL LIMITS/PLANNER -> RUN -> MOTION
//   AUTHORITY GRANTED
//
// Pure decision core: plain facts in, ONE action per tick out. It never
// touches a servo, NVS or the radio, and none of its actions moves a joint or
// enables torque: torque comes only from an explicit Motion command through
// MotionSession (prime at the measured pose -> energize -> arm). Authority is
// re-evaluated every tick: a lost requirement revokes it (SAFE_OFF if a
// session is live) and a restored requirement grants it again — nothing here
// can lock the robot permanently or needs a rebuild.
#include <stdint.h>
#include "OperatingMode.h"

namespace matdog {
namespace core {

enum class ReadinessAction : uint8_t {
  NONE,
  START_PREFLIGHT,     // read-only ST3215 identity/profile pass
  RESTORE,             // RAM-only gen-N transforms (policy decides)
  ADMIT_OPERATIONAL,   // hard limits from gen-1 contacts + planner binding
  ENTER_RUN,           // operating mode only; the radio role is NOT changed
  GRANT_AUTHORITY,     // MOTION lease held idle by the Controller
  RELEASE_AUTHORITY,   // idle lease returned (no session live)
  REVOKE_SESSION,      // live session -> SAFE_OFF (STOP procedure)
};
enum class ReadinessPhase : uint8_t { BOOTING, CENSUS, PREFLIGHT, RESTORE, ADMIT, RUN_ENTRY, READY, BLOCKED, MAINTENANCE_HOLD };

struct ReadinessFacts {
  uint32_t now_ms = 0;
  bool health_ready = false, health_fault = false;
  bool boot_safe_off_done = false;
  bool census_complete = false, census_pass = false;
  bool preflight_running = false, preflight_complete = false, preflight_pass = false;
  bool persistence_loaded = false;
  bool restored = false;                 // RuntimeCalibrationSnapshot::restored
  bool restore_refused_permanent = false;// record absent/corrupt/incompatible/live mismatch
  bool restore_refused_transient = false;// idle/torque/preflight not yet true
  const char* restore_reason = "";
  bool mapped_fresh = false;             // 12/12 calibrated joints, fresh
  bool raw_fresh = false;                // 12/12 raw ticks, fresh
  bool operational_bound = false;        // limits + planner admitted
  bool admit_failed = false; const char* admit_reason = "";
  bool imu_fresh = false, bms_ok = false, power_ok = false, inhibited = false;
  OperatingMode mode = OperatingMode::MAINTENANCE;
  bool operator_maintenance_hold = false;// operator asked for MAINTENANCE explicitly
  bool authority_motion = false, authority_other = false;
  bool session_active = false;           // PRIMING..ARMED
};

struct ReadinessStatus {
  ReadinessPhase phase = ReadinessPhase::BOOTING;
  const char* system_health = "BOOTING";      // READY | BOOTING | DEGRADED | FAULT
  const char* calibration = "NOT_RESTORED";   // RESTORED | NOT_RESTORED | REFUSED
  const char* joint_feedback = "UNAVAILABLE"; // AVAILABLE | RAW_ONLY | UNAVAILABLE
  const char* motion_authority = "NOT_GRANTED"; // GRANTED | NOT_GRANTED | REVOKED
  const char* reason = "BOOTING";
};

class OperationalReadiness {
 public:
  ReadinessAction step(const ReadinessFacts& facts);
  const ReadinessStatus& status() const { return status_; }
  uint32_t grants() const { return grants_; }
  uint32_t revocations() const { return revocations_; }
  // Requirement that blocks authority right now, or nullptr when all hold.
  static const char* authorityBlocker(const ReadinessFacts& f);
 private:
  ReadinessStatus status_{};
  uint32_t preflight_requested_ms_ = 0, restore_attempt_ms_ = 0, admit_attempt_ms_ = 0;
  bool preflight_requested_ = false, restore_attempted_ = false, admit_attempted_ = false;
  bool was_granted_ = false;
  uint32_t grants_ = 0, revocations_ = 0;
};

const char* toString(ReadinessAction action);
const char* toString(ReadinessPhase phase);

}  // namespace core
}  // namespace matdog
#endif
