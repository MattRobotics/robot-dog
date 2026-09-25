// Offline adversarial tests for the Calibration Execution boundary
// (src/calibration/CalibrationExecutionEngine.*) — I5, V3 handoff §13/§15.11.
//
// Links the REAL engine, the REAL SafeActuatorPolicy, the REAL
// ActuatorRuntime and the REAL ActuatorAuthorityArbiter against a fake
// backend — the same contract as test_actuator_write_policy.cpp and
// test_actuator_runtime.cpp. A mocked policy or arbiter here would test the
// mock's idea of the rules, not the shipped ones.
//
// BINDING (V3 §15.11): the LF V25 18-phase sequence never appears in this
// file. It is a historical oracle exercised only by
// test_calibration_domain.cpp's replay, never this architecture.
//
// NO HOST TEST IN THIS FILE IS HARDWARE VALIDATION. There is still no
// production ActuatorBackend anywhere in this firmware, and
// MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED stays 0.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/calibration/CalibrationExecutionEngine.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::ActuatorRuntime;
using matdog::actuator::CalibrationGeometryProfile;
using matdog::actuator::ExecuteResult;
using matdog::actuator::GeometryProvenance;
using matdog::actuator::SafeActuatorPolicy;
using matdog::actuator::WriteDecision;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::AuthorityLease;
using matdog::core::AuthorityResult;
using matdog::core::OperatingMode;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                            \
  } while (0)

#define CHECK_EQ(actual, expected)                                         \
  do {                                                                     \
    ++g_checks;                                                            \
    const long a_ = (long)(actual);                                        \
    const long e_ = (long)(expected);                                      \
    if (a_ != e_) {                                                        \
      ++g_failures;                                                        \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,  \
                  __FILE__, __LINE__, #actual, a_, e_);                    \
    }                                                                      \
  } while (0)

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                              \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));          \
    }                                                                          \
  } while (0)

namespace {

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

JointIdentity joint(Leg leg, JointKind kind, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = kind;
  setPhysicalUnit(&id, unit);
  return id;
}

// The real compiled endpoints this suite depends on (verified against
// src/actuator/CalibrationGeometryProfileData.h):
//   LF HIP  MIN_SIDE -> DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, unit "M22"
//   LF UPPER MIN_SIDE -> EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED,
//                        unit "ELR01", contact=-909889 urad (MIN_SIDE), so a
//                        target_urad of 0 (this engine's zero default) lies
//                        strictly on the safe side of contact and inside the
//                        joint's URDF bounds — reaching the transform check
//                        is therefore a fact about the real compiled data,
//                        not an assumption this file makes.
JointIdentity lfHip() { return joint(Leg::LF, JointKind::HIP, "M22"); }
JointIdentity lfUpper() { return joint(Leg::LF, JointKind::UPPER, "ELR01"); }

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&actuator::geometry_data::kProvenance, actuator::geometry_data::kJoints,
              actuator::geometry_data::kJointCount, actuator::geometry_data::kEndpoints,
              actuator::geometry_data::kEndpointCount);
  return profile;
}

AuthorityLease grant(ActuatorAuthorityArbiter& arbiter, ActuatorAuthority owner,
                     OperatingMode mode) {
  AuthorityLease lease{};
  const AuthorityResult result = arbiter.request(owner, mode, &lease);
  CHECK(result == AuthorityResult::GRANTED);
  return lease;
}

class FakeActuatorBackend : public actuator::ActuatorBackend {
 public:
  bool enableTorque(uint8_t) override {
    ++calls;
    return true;
  }
  bool writeGoalPosition(uint8_t, uint16_t) override {
    ++calls;
    return true;
  }
  int calls = 0;
};

// One fully-wired rig: real arbiter, real policy, real runtime, real
// engine, fake backend. Every test builds one so no state leaks between
// cases.
struct Rig {
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  ActuatorRuntime runtime;
  CalibrationExecutionEngine engine;
  FakeActuatorBackend backend;

  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    policy.begin(&arbiter);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime);
  }
};

CalibrationExecutionContext liveContext(const AuthorityLease& lease, OperatingMode mode) {
  CalibrationExecutionContext ctx{};
  ctx.session_active = true;
  ctx.origin = CalibrationOrigin::LIVE_SESSION;
  ctx.lease = lease;
  ctx.mode = mode;
  return ctx;
}

CalibrationExecutionRequest contactProbe(JointIdentity id, Leg leg, JointKind ep_joint,
                                         ContactSide side) {
  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::CONTACT_PROBE;
  req.joint = id;
  req.endpoint_leg = leg;
  req.endpoint_joint = ep_joint;
  req.endpoint_side = side;
  return req;
}

// ---------------------------------------------------------------------------
// 1. authority loss -> no restore command
// ---------------------------------------------------------------------------

void test_authority_loss_produces_zero_restore_motion() {
  g_case = "authority loss -> no restore";
  Rig rig;

  // "Authority loss": an empty/invalid lease, exactly what a session left
  // holding a stale reference after ActuatorAuthorityArbiter::forceClear()
  // would see.
  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::RESTORE;
  CalibrationExecutionContext ctx{};  // session_active=false, empty lease

  const CalibrationExecutionResult result = rig.engine.execute(req, ctx, /*bus_id=*/11);

  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::RESTORE_ACKNOWLEDGED_NO_MOTION);
  CHECK_EQ(rig.backend.calls, 0);

  // Even with a fully live, valid session and a genuine lease, RESTORE still
  // never reaches the backend — the property does not depend on authority
  // state at all, which is the point: it is true by construction.
  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  const CalibrationExecutionResult result2 =
      rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), /*bus_id=*/11);
  CHECK_EQ((int)result2.outcome, (int)CalibrationExecutionOutcome::RESTORE_ACKNOWLEDGED_NO_MOTION);
  CHECK_EQ(rig.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 2. stale authority generation rejected
// ---------------------------------------------------------------------------

void test_stale_authority_generation_rejected() {
  g_case = "stale generation";
  Rig rig;

  const AuthorityLease first = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  CHECK(rig.arbiter.release(first) == AuthorityResult::RELEASED);
  const AuthorityLease second = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                      OperatingMode::MAINTENANCE);
  CHECK(first.generation != second.generation);

  CalibrationExecutionRequest req{};
  req.intent = CalibrationIntent::DIRECTION_VERIFY;
  req.joint = lfUpper();
  req.direction_verify_delta_ticks = 50;

  const CalibrationExecutionResult stale =
      rig.engine.execute(req, liveContext(first, OperatingMode::MAINTENANCE), /*bus_id=*/12);
  CHECK_EQ((int)stale.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
  CHECK_EQ((int)stale.policy_decision, (int)WriteDecision::REJECT_STALE_GENERATION);
  CHECK_EQ(rig.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 3. diagnostic endpoint cannot become executable
// ---------------------------------------------------------------------------

void test_diagnostic_endpoint_cannot_become_executable() {
  g_case = "diagnostic endpoint refused";
  Rig rig;
  CalibrationGeometryProfile profile = boundProfile();
  rig.policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);

  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  rig.policy.setBootstrapContext(bootstrap);

  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  // LF HIP MIN_SIDE is DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS — a clean
  // request that is refused purely because this endpoint is never a motion
  // target, whatever its clearance verdict.
  const CalibrationExecutionRequest req =
      contactProbe(lfHip(), Leg::LF, JointKind::HIP, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result =
      rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), /*bus_id=*/13);

  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
  CHECK_EQ((int)result.policy_decision, (int)WriteDecision::REJECT_ENDPOINT_NOT_EXECUTABLE);
  CHECK_EQ(rig.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 4. stale/wrong geometry provenance rejected
// ---------------------------------------------------------------------------

void test_wrong_geometry_provenance_rejected() {
  g_case = "wrong geometry provenance";
  Rig rig;
  CalibrationGeometryProfile profile = boundProfile();
  GeometryProvenance impostor = actuator::geometry_data::kProvenance;
  impostor.urdf_sha256[0] = (impostor.urdf_sha256[0] == 'a') ? 'b' : 'a';
  rig.policy.bindGeometry(&profile, &impostor);

  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result =
      rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), /*bus_id=*/12);

  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
  CHECK_EQ((int)result.policy_decision, (int)WriteDecision::REJECT_GEOMETRY_PROVENANCE);
  CHECK_EQ(rig.backend.calls, 0);

  // Completely unbound geometry is the same refusal family, different member.
  Rig rig2;
  const AuthorityLease lease2 = grant(rig2.arbiter, ActuatorAuthority::CALIBRATION,
                                      OperatingMode::MAINTENANCE);
  const CalibrationExecutionResult unbound =
      rig2.engine.execute(req, liveContext(lease2, OperatingMode::MAINTENANCE), /*bus_id=*/12);
  CHECK_EQ((int)unbound.policy_decision, (int)WriteDecision::REJECT_NO_GEOMETRY_PROFILE);
  CHECK_EQ(rig2.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 5. historical replay cannot promote operational evidence
// ---------------------------------------------------------------------------

void test_historical_replay_origin_refused_before_the_policy_is_even_asked() {
  g_case = "replay origin refused";
  Rig rig;
  CalibrationGeometryProfile profile = boundProfile();
  rig.policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  CalibrationExecutionContext ctx{};
  ctx.session_active = true;
  ctx.origin = CalibrationOrigin::HISTORICAL_REPLAY;  // never promotable, never physical
  ctx.lease = lease;
  ctx.mode = OperatingMode::MAINTENANCE;

  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result = rig.engine.execute(req, ctx, /*bus_id=*/12);

  // Refused by THIS layer, before policy.plan() is even called — defence in
  // depth alongside the domain-level guarantee (mayPromote(HISTORICAL_REPLAY)
  // == false, proven in test_calibration_domain.cpp).
  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::REJECT_REPLAY_ORIGIN);
  CHECK_EQ((int)rig.policy.counters().plans, 0);
  CHECK_EQ(rig.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 6. no q0/current transform -> no raw target
// ---------------------------------------------------------------------------

void test_no_accepted_transform_means_no_raw_target() {
  g_case = "no accepted transform";
  Rig rig;
  CalibrationGeometryProfile profile = boundProfile();
  rig.policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);

  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  rig.policy.setBootstrapContext(bootstrap);

  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  CHECK(rig.policy.transforms().empty());  // the shipped state: nothing admitted

  // LF UPPER MIN_SIDE is EXECUTABLE with no parking required, so this
  // request clears every earlier gate and is refused for exactly one
  // reason: no accepted raw<->q transform exists. The engine's target_urad
  // stays at its zero default throughout — it never computes one.
  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result =
      rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), /*bus_id=*/12);

  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
  CHECK_EQ((int)result.policy_decision, (int)WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
  CHECK_EQ(rig.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 7. CALIBRATION contact probe allowed only under CALIBRATION
// ---------------------------------------------------------------------------

void test_contact_probe_eligible_only_for_calibration_owner() {
  g_case = "contact probe owner eligibility";
  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);

  // CALIBRATION passes the eligibility gate — it reaches a GEOMETRY-level
  // decision (proven by tests 3/4/6 above), never REJECT_OPERATION_NOT_PERMITTED.
  // Every other write-capable owner except MOTION (test 8) is refused at
  // the eligibility gate itself, before geometry is ever consulted.
  for (ActuatorAuthority owner : {ActuatorAuthority::DIAGNOSTICS, ActuatorAuthority::QC,
                                  ActuatorAuthority::PROVISIONING}) {
    Rig rig;
    const AuthorityLease lease = grant(rig.arbiter, owner, OperatingMode::MAINTENANCE);
    const CalibrationExecutionResult result =
        rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), /*bus_id=*/12);
    CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
    CHECK_EQ((int)result.policy_decision, (int)WriteDecision::REJECT_OPERATION_NOT_PERMITTED);
    CHECK_EQ(rig.backend.calls, 0);
  }
}

// ---------------------------------------------------------------------------
// 8. MOTION cannot execute contact probe
// ---------------------------------------------------------------------------

void test_motion_cannot_execute_contact_probe() {
  g_case = "motion cannot probe";
  Rig rig;
  const AuthorityLease motion = grant(rig.arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result =
      rig.engine.execute(req, liveContext(motion, OperatingMode::RUN), /*bus_id=*/12);

  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
  CHECK_EQ((int)result.policy_decision, (int)WriteDecision::REJECT_OPERATION_NOT_PERMITTED);
  CHECK_EQ(rig.backend.calls, 0);
}

// ---------------------------------------------------------------------------
// 9. abort, restore intent and SAFE_OFF remain distinct
// ---------------------------------------------------------------------------

void test_abort_restore_and_safe_off_remain_distinct() {
  g_case = "abort vs restore vs safe_off";
  Rig rig;
  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  const CalibrationExecutionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CalibrationExecutionRequest restore{};
  restore.intent = CalibrationIntent::RESTORE;
  const CalibrationExecutionResult restore_result = rig.engine.execute(restore, ctx, 12);

  CalibrationExecutionRequest abort{};
  abort.intent = CalibrationIntent::ABORT;
  const CalibrationExecutionResult abort_result = rig.engine.execute(abort, ctx, 12);

  // Two distinct, separately named outcomes — never collapsed into one
  // generic "ended" result, and neither ever reaches the backend.
  CHECK_EQ((int)restore_result.outcome,
          (int)CalibrationExecutionOutcome::RESTORE_ACKNOWLEDGED_NO_MOTION);
  CHECK_EQ((int)abort_result.outcome,
          (int)CalibrationExecutionOutcome::ABORT_IS_LIFECYCLE_NOT_MOTION);
  CHECK(restore_result.outcome != abort_result.outcome);
  CHECK_EQ(rig.backend.calls, 0);

  // SAFE_OFF has no representation anywhere in this type at all — it is
  // structurally outside this layer, the same way it is outside
  // ActuatorWritePolicy and ActuatorRuntime. There is no third intent value
  // this suite could exercise for it; its absence from CalibrationIntent's
  // six values, verified once here, is the whole proof.
  CHECK_STR(toString(CalibrationIntent::NONE), "NONE");
  CHECK_STR(toString(CalibrationIntent::CONTACT_PROBE), "CONTACT_PROBE");
  CHECK_STR(toString(CalibrationIntent::AUXILIARY_MOVE), "AUXILIARY_MOVE");
  CHECK_STR(toString(CalibrationIntent::DIRECTION_VERIFY), "DIRECTION_VERIFY");
  CHECK_STR(toString(CalibrationIntent::RESTORE), "RESTORE");
  CHECK_STR(toString(CalibrationIntent::ABORT), "ABORT");
}

// ---------------------------------------------------------------------------
// 10. unknown/unsupported execution intent fails closed
// ---------------------------------------------------------------------------

void test_unknown_intent_fails_closed() {
  g_case = "unknown intent fails closed";
  Rig rig;
  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  const CalibrationExecutionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CalibrationExecutionRequest none_req{};
  none_req.intent = CalibrationIntent::NONE;
  const CalibrationExecutionResult none_result = rig.engine.execute(none_req, ctx, 12);
  CHECK_EQ((int)none_result.outcome, (int)CalibrationExecutionOutcome::REJECT_UNKNOWN_INTENT);

  CalibrationExecutionRequest corrupt_req{};
  corrupt_req.intent = static_cast<CalibrationIntent>(200);
  const CalibrationExecutionResult corrupt_result = rig.engine.execute(corrupt_req, ctx, 12);
  CHECK_EQ((int)corrupt_result.outcome, (int)CalibrationExecutionOutcome::REJECT_UNKNOWN_INTENT);

  CHECK_EQ(rig.backend.calls, 0);
  CHECK_EQ((int)rig.policy.counters().plans, 0);  // never even reached the policy
}

// ---------------------------------------------------------------------------
// operationForIntent() — pure, total
// ---------------------------------------------------------------------------

void test_operation_for_intent_is_total() {
  g_case = "operation_for_intent";
  CHECK_EQ((int)operationForIntent(CalibrationIntent::CONTACT_PROBE),
          (int)actuator::ActuatorOperation::CALIBRATION_CONTACT_PROBE);
  CHECK_EQ((int)operationForIntent(CalibrationIntent::AUXILIARY_MOVE),
          (int)actuator::ActuatorOperation::CALIBRATION_AUXILIARY_MOVE);
  CHECK_EQ((int)operationForIntent(CalibrationIntent::DIRECTION_VERIFY),
          (int)actuator::ActuatorOperation::DIRECTION_VERIFY);
  CHECK_EQ((int)operationForIntent(CalibrationIntent::NONE),
          (int)actuator::ActuatorOperation::NONE);
  CHECK_EQ((int)operationForIntent(CalibrationIntent::RESTORE),
          (int)actuator::ActuatorOperation::NONE);
  CHECK_EQ((int)operationForIntent(CalibrationIntent::ABORT),
          (int)actuator::ActuatorOperation::NONE);
}

void test_to_string_fails_closed_on_corrupted_value() {
  g_case = "to_string corrupted";
  CHECK_STR(toString(static_cast<CalibrationIntent>(200)), "UNKNOWN");
  CHECK_STR(toString(static_cast<CalibrationExecutionOutcome>(200)), "UNKNOWN");
}

}  // namespace

int main() {
  test_authority_loss_produces_zero_restore_motion();
  test_stale_authority_generation_rejected();
  test_diagnostic_endpoint_cannot_become_executable();
  test_wrong_geometry_provenance_rejected();
  test_historical_replay_origin_refused_before_the_policy_is_even_asked();
  test_no_accepted_transform_means_no_raw_target();
  test_contact_probe_eligible_only_for_calibration_owner();
  test_motion_cannot_execute_contact_probe();
  test_abort_restore_and_safe_off_remain_distinct();
  test_unknown_intent_fails_closed();
  test_operation_for_intent_is_total();
  test_to_string_fails_closed_on_corrupted_value();

  std::printf("test_calibration_execution_engine: %d checks, %d failures\n", g_checks,
             g_failures);
  return g_failures == 0 ? 0 : 1;
}
