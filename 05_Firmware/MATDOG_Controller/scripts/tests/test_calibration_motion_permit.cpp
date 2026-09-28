#include <cstdio>

#include "../../src/calibration/CalibrationMotionPermit.h"

using namespace matdog::calibration;
using namespace matdog::core;

static int g_checks = 0;
static int g_failures = 0;

static void check(bool ok, const char* label) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("FAIL: %s\n", label);
  }
}

static CalibrationMotionPermitFacts goodFacts() {
  CalibrationMotionPermitFacts f{};
  f.explicit_operator_authorization = true;
  f.robot_powered_profile = true;
  f.mode = OperatingMode::MAINTENANCE;
  f.system_health = SystemHealth::READY;
  f.session_active = true;
  f.origin = CalibrationOrigin::LIVE_SESSION;
  f.session_id = 44;
  f.current_population_pass = true;
  f.current_geometry_bound = true;
  f.promoted_transforms_complete = true;
  f.authority = ActuatorAuthority::CALIBRATION;
  f.authority_generation = 7;
  f.authority_inhibited = false;
  return f;
}

static void test_grant_requires_every_fact() {
  CalibrationMotionPermit permit;
  CalibrationMotionPermitToken token{};
  auto f = goodFacts();

  check(permit.grant(f, &token) == CalibrationPermitStatus::ACTIVE,
        "complete prerequisites grant");
  check(token.valid(), "grant returns bound token");
  check(permit.check(f, token) == CalibrationPermitStatus::ACTIVE,
        "current token validates");

  struct Mutation {
    const char* label;
    void (*apply)(CalibrationMotionPermitFacts&);
    CalibrationPermitStatus expected;
  };
  const Mutation cases[] = {
      {"operator", [](auto& x){ x.explicit_operator_authorization=false; },
       CalibrationPermitStatus::REJECT_NO_OPERATOR_AUTH},
      {"profile", [](auto& x){ x.robot_powered_profile=false; },
       CalibrationPermitStatus::REJECT_PROFILE},
      {"mode", [](auto& x){ x.mode=OperatingMode::RUN; },
       CalibrationPermitStatus::REJECT_MODE},
      {"health", [](auto& x){ x.system_health=SystemHealth::FAULT; },
       CalibrationPermitStatus::REJECT_SYSTEM_HEALTH},
      {"session", [](auto& x){ x.session_active=false; },
       CalibrationPermitStatus::REJECT_SESSION},
      {"origin", [](auto& x){ x.origin=CalibrationOrigin::HISTORICAL_REPLAY; },
       CalibrationPermitStatus::REJECT_SESSION},
      {"population", [](auto& x){ x.current_population_pass=false; },
       CalibrationPermitStatus::REJECT_POPULATION},
      {"geometry", [](auto& x){ x.current_geometry_bound=false; },
       CalibrationPermitStatus::REJECT_GEOMETRY},
      {"transforms", [](auto& x){ x.promoted_transforms_complete=false; },
       CalibrationPermitStatus::REJECT_TRANSFORMS},
      {"authority", [](auto& x){ x.authority=ActuatorAuthority::NONE; },
       CalibrationPermitStatus::REJECT_AUTHORITY},
      {"inhibit", [](auto& x){ x.authority_inhibited=true; },
       CalibrationPermitStatus::REJECT_INHIBITED},
  };

  for (const auto& c : cases) {
    CalibrationMotionPermit p;
    CalibrationMotionPermitToken t{};
    auto bad = goodFacts();
    c.apply(bad);
    check(p.grant(bad, &t) == c.expected, c.label);
    check(!p.active(), "failed grant leaves no permit active");
    check(!t.valid(), "failed grant returns no token");
  }
}

static void test_session_and_authority_binding() {
  CalibrationMotionPermit permit;
  CalibrationMotionPermitToken token{};
  auto f = goodFacts();
  check(permit.grant(f, &token) == CalibrationPermitStatus::ACTIVE, "grant for binding test");

  auto changed = f;
  changed.session_id++;
  check(permit.check(changed, token) == CalibrationPermitStatus::REJECT_SESSION,
        "new session cannot reuse token");
  check(!permit.active(), "session drift revokes the permit, not just this one check");

  // check() revokes as a side effect of ANY mismatch - by design, a stale
  // permit must require a fresh explicit grant rather than resume (Priority 1
  // invariant). Re-grant before isolating the next, independent mismatch, or
  // this second check would trivially return REVOKED from the first one.
  CalibrationMotionPermitToken token2{};
  check(permit.grant(f, &token2) == CalibrationPermitStatus::ACTIVE,
        "fresh grant before the authority-generation case");

  changed = f;
  changed.authority_generation++;
  check(permit.check(changed, token2) == CalibrationPermitStatus::REJECT_AUTHORITY,
        "new authority generation cannot reuse token");

  CalibrationMotionPermitToken copied = token2;
  permit.reset();
  check(permit.check(f, copied) == CalibrationPermitStatus::REVOKED,
        "reset invalidates copied token");
}

static void test_dynamic_prerequisite_loss_expires_token() {
  CalibrationMotionPermit permit;
  CalibrationMotionPermitToken token{};
  auto f = goodFacts();
  check(permit.grant(f, &token) == CalibrationPermitStatus::ACTIVE,
        "grant for expiry test");

  auto bad = f;
  bad.mode = OperatingMode::RUN;
  check(permit.check(bad, token) == CalibrationPermitStatus::REJECT_MODE,
        "mode incompatibility rejects");
  check(!permit.active(), "mode incompatibility revokes permit");
  check(permit.lastRevokeReason() ==
            CalibrationPermitRevokeReason::MODE_INCOMPATIBLE,
        "mode incompatibility records revoke reason");

  // Returning to healthy facts must NOT resurrect the old token.
  check(permit.check(f, token) == CalibrationPermitStatus::REVOKED,
        "old token cannot revive after mode recovers");

  CalibrationMotionPermitToken token2{};
  check(permit.grant(f, &token2) == CalibrationPermitStatus::ACTIVE,
        "fresh explicit grant required after expiry");

  bad = f;
  bad.system_health = SystemHealth::FAULT;
  check(permit.check(bad, token2) ==
            CalibrationPermitStatus::REJECT_SYSTEM_HEALTH,
        "fatal health loss rejects");
  check(!permit.active(), "fatal health loss revokes");
  check(permit.lastRevokeReason() ==
            CalibrationPermitRevokeReason::SYSTEM_FAULT,
        "fatal health loss records fault revoke");

  CalibrationMotionPermitToken token3{};
  check(permit.grant(f, &token3) == CalibrationPermitStatus::ACTIVE,
        "fresh grant after fault");
  bad = f;
  bad.authority_generation++;
  check(permit.check(bad, token3) ==
            CalibrationPermitStatus::REJECT_AUTHORITY,
        "authority generation loss rejects");
  check(!permit.active(), "authority generation loss revokes");
  check(permit.lastRevokeReason() ==
            CalibrationPermitRevokeReason::AUTHORITY_LOST,
        "authority loss records revoke reason");
}

static void test_revoke_is_final_for_token() {
  CalibrationMotionPermit permit;
  CalibrationMotionPermitToken token{};
  auto f = goodFacts();
  permit.grant(f, &token);
  permit.revoke(CalibrationPermitRevokeReason::SESSION_ENDED);
  check(!permit.active(), "session end revokes");
  check(permit.lastRevokeReason() == CalibrationPermitRevokeReason::SESSION_ENDED,
        "revoke reason retained");
  check(permit.check(f, token) == CalibrationPermitStatus::REVOKED,
        "revoked token stays unusable");

  CalibrationMotionPermitToken token2{};
  check(permit.grant(f, &token2) == CalibrationPermitStatus::ACTIVE,
        "new explicit grant is possible");
  check(token2.permit_generation != token.permit_generation,
        "new permit has new generation");
}

int main() {
  test_grant_requires_every_fact();
  test_session_and_authority_binding();
  test_dynamic_prerequisite_loss_expires_token();
  test_revoke_is_final_for_token();
  std::printf("test_calibration_motion_permit: %d checks, %d failures\n",
              g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
