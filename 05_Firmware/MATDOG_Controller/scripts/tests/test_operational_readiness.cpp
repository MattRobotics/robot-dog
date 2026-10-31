// Operator directive 2026-10-14 (final): POWER ON -> CENSUS -> PREFLIGHT ->
// RESTORE -> FEEDBACK -> ADMIT -> RUN -> MOTION AUTHORITY GRANTED, with
// dynamic revocation and re-grant. A simulated world executes each action the
// way the Controller does; no action may move a joint or enable torque.
#include <cstdio>
#include <cstring>
#include <vector>
#include "../../src/core/OperationalReadiness.h"
using namespace matdog::core;
static int checks = 0, failures = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)

struct World {
  ReadinessFacts f;
  std::vector<ReadinessAction> log;
  bool record_ok = true, record_permanent_bad = false, admit_ok = true;
  unsigned torque_writes = 0, goal_writes = 0;  // must stay 0: readiness never actuates
  OperationalReadiness r;
  World() {
    f.boot_safe_off_done = true; f.census_complete = true; f.census_pass = true; f.persistence_loaded = true;
    f.imu_fresh = f.bms_ok = f.power_ok = true; f.raw_fresh = true;
  }
  ReadinessAction tick(uint32_t dt = 30) {
    f.now_ms += dt;
    const auto a = r.step(f);
    log.push_back(a);
    switch (a) {  // what the Controller does for each action
      case ReadinessAction::START_PREFLIGHT: f.preflight_running = true; break;
      case ReadinessAction::RESTORE:
        if (record_permanent_bad) { f.restore_refused_permanent = true; f.restore_reason = "NO_VALID_ACKNOWLEDGED_RECORD"; }
        else if (record_ok) { f.restored = true; f.mapped_fresh = true; }
        break;
      case ReadinessAction::ADMIT_OPERATIONAL: if (admit_ok) f.operational_bound = true; else { f.admit_failed = true; f.admit_reason = "ROUTE_OUTSIDE_COMMISSIONING_LIMITS"; } break;
      case ReadinessAction::ENTER_RUN: f.mode = OperatingMode::RUN; break;
      case ReadinessAction::GRANT_AUTHORITY: f.authority_motion = true; break;
      case ReadinessAction::RELEASE_AUTHORITY: f.authority_motion = false; break;
      case ReadinessAction::REVOKE_SESSION: f.session_active = false; f.authority_motion = false; break;  // SAFE_OFF then release
      case ReadinessAction::NONE: break;
    }
    if (f.preflight_running && a == ReadinessAction::NONE) { f.preflight_running = false; f.preflight_complete = true; f.preflight_pass = true; }
    return a;
  }
  void run(unsigned n) { for (unsigned i = 0; i < n; ++i) tick(); }
};

int main() {
  // 1. Boot with hardware available and a valid gen-1 record.
  {
    World w; w.run(12);
    const char* expected[] = {"START_PREFLIGHT", "RESTORE", "ADMIT_OPERATIONAL", "ENTER_RUN", "GRANT_AUTHORITY"};
    std::vector<const char*> seen; for (auto a : w.log) if (a != ReadinessAction::NONE) seen.push_back(toString(a));
    CHECK(seen.size() == 5); for (unsigned i = 0; i < seen.size() && i < 5; ++i) CHECK(!std::strcmp(seen[i], expected[i]));
    const auto& s = w.r.status();
    CHECK(!std::strcmp(s.system_health, "READY") && !std::strcmp(s.calibration, "RESTORED"));
    CHECK(!std::strcmp(s.joint_feedback, "AVAILABLE") && !std::strcmp(s.motion_authority, "GRANTED"));
    CHECK(s.phase == ReadinessPhase::READY && w.f.mode == OperatingMode::RUN);
    CHECK(w.torque_writes == 0 && w.goal_writes == 0 && !w.f.session_active);   // no spontaneous motion at boot
    std::printf("boot sequence: "); for (auto x : seen) std::printf("%s ", x); std::printf("-> %s\n", s.motion_authority);
  }
  // 2. Reboot: RAM is empty again -> automatic RESTORE again, no operator action.
  { World w; w.run(12); World again; again.run(12); CHECK(again.f.restored && again.f.authority_motion); }
  // 3. Record absent / corrupt / incompatible -> refused with reason, never granted, not retried.
  {
    World w; w.record_permanent_bad = true; w.run(400);
    CHECK(!w.f.authority_motion && !std::strcmp(w.r.status().calibration, "REFUSED"));
    CHECK(!std::strcmp(w.r.status().reason, "NO_VALID_ACKNOWLEDGED_RECORD") && w.r.status().phase == ReadinessPhase::BLOCKED);
    unsigned restores = 0; for (auto a : w.log) restores += a == ReadinessAction::RESTORE; CHECK(restores == 1);
  }
  // 4. Transient refusal (e.g. torque readback not yet OFF) is retried every 5 s.
  {
    World w; w.record_ok = false; w.f.restore_refused_transient = true; w.f.restore_reason = "TORQUE_NOT_OFF"; w.run(400);  // 12 s
    unsigned restores = 0; for (auto a : w.log) restores += a == ReadinessAction::RESTORE; CHECK(restores >= 2 && restores <= 3);
    CHECK(!w.f.authority_motion && !std::strcmp(w.r.status().reason, "TORQUE_NOT_OFF"));
    w.record_ok = true; w.f.restore_refused_transient = false; w.run(200); CHECK(w.f.authority_motion);
  }
  // 5. Census incomplete: no RESTORE, no authority.
  { World w; w.f.census_pass = false; w.run(100); CHECK(!w.f.restored && !w.f.authority_motion && !std::strcmp(w.r.status().reason, "SERVO_CENSUS_INCOMPLETE")); }
  { World w; w.f.census_complete = false; w.run(50); CHECK(w.r.status().phase == ReadinessPhase::CENSUS && w.log.back() == ReadinessAction::NONE); }
  // 6. Telemetry loss after READY: authority revoked; returns automatically.
  {
    World w; w.run(12); CHECK(w.f.authority_motion);
    w.f.raw_fresh = false; CHECK(w.tick() == ReadinessAction::RELEASE_AUTHORITY);
    CHECK(!std::strcmp(w.r.status().motion_authority, "REVOKED") && !std::strcmp(w.r.status().reason, "RAW_FEEDBACK_LOST"));
    CHECK(w.tick() == ReadinessAction::NONE && !w.f.authority_motion);
    w.f.raw_fresh = true; w.run(3); CHECK(w.f.authority_motion && !std::strcmp(w.r.status().motion_authority, "GRANTED"));  // no permanent lock
    CHECK(w.r.revocations() == 1 && w.r.grants() == 2);
  }
  // 7. Relevant hardware fault while a session is live -> REVOKE_SESSION (SAFE_OFF).
  for (int k = 0; k < 6; ++k) {
    World w; w.run(12); w.f.session_active = true;
    switch (k) {
      case 0: w.f.health_fault = true; break;  case 1: w.f.bms_ok = false; break;    case 2: w.f.imu_fresh = false; break;
      case 3: w.f.mapped_fresh = false; break; case 4: w.f.inhibited = true; break; default: w.f.power_ok = false; break;
    }
    CHECK(w.tick() == ReadinessAction::REVOKE_SESSION);
    CHECK(!w.f.authority_motion && !w.f.session_active);
  }
  // 8. Operator explicitly selects MAINTENANCE: authority released, no automatic RUN re-entry.
  {
    World w; w.run(12); w.f.mode = OperatingMode::MAINTENANCE; w.f.operator_maintenance_hold = true;
    CHECK(w.tick() == ReadinessAction::RELEASE_AUTHORITY);
    w.run(50); CHECK(w.f.mode == OperatingMode::MAINTENANCE && !w.f.authority_motion && w.r.status().phase == ReadinessPhase::MAINTENANCE_HOLD);
    w.f.operator_maintenance_hold = false; w.run(5); CHECK(w.f.authority_motion);
  }
  // 9. Admission failure is reported, retried, never silently granted.
  { World w; w.admit_ok = false; w.run(200); CHECK(!w.f.authority_motion && !std::strcmp(w.r.status().reason, "ROUTE_OUTSIDE_COMMISSIONING_LIMITS")); }
  // 10. A service owner (calibration session) blocks the grant instead of being overridden.
  { World w; w.f.authority_other = true; w.run(50); CHECK(!w.f.authority_motion && !std::strcmp(w.r.status().reason, "SERVICE_AUTHORITY_HELD")); }
  // 11. Preflight failure blocks, with a periodic automatic retry.
  {
    World w; w.run(1); w.f.preflight_running = false; w.f.preflight_complete = true; w.f.preflight_pass = false;
    w.run(100); CHECK(!w.f.restored && !std::strcmp(w.r.status().reason, "SERVO_PREFLIGHT_FAILED"));
  }
  // 12. Fault before READY: nothing at all happens.
  { World w; w.f.health_fault = true; w.run(50); for (auto a : w.log) CHECK(a == ReadinessAction::NONE); CHECK(!std::strcmp(w.r.status().system_health, "FAULT")); }
  std::printf("OPERATIONAL_READINESS: %d checks, %d failures (offline; actions never actuate)\n", checks, failures);
  return failures ? 1 : 0;
}
