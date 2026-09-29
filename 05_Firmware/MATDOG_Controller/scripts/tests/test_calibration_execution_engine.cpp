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
#include <cstdlib>
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
  actuator::BackendWriteOutcome enableTorque(uint8_t) override {
    ++calls;
    return actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  }
  actuator::BackendWriteOutcome writeGoalPosition(uint8_t, uint16_t target_tick,
                                                  actuator::MotionProfile profile) override {
    ++calls;
    last_target_tick = target_tick;
    last_profile = profile;
    return actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  }
  int calls = 0;
  uint16_t last_target_tick = 0;
  actuator::MotionProfile last_profile = actuator::MotionProfile::BOUNDED_DEFAULT;
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
  CalibrationGeometryProfile profile;

  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance);
  }
};

actuator::JointTransform promotedTransform(JointIdentity id, uint16_t q0 = 2048) {
  actuator::JointTransform t{};
  t.identity = id;
  t.geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.q0_tick = q0;
  t.present = true;
  return t;
}

void armCalibrationPermit(Rig& rig, const AuthorityLease& lease, int32_t budget = 64) {
  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  bootstrap.motion_permit_active = true;
  bootstrap.motion_permit_generation = 1;
  bootstrap.motion_permit_session_id = 1;
  bootstrap.motion_permit_authority_generation = lease.generation;
  bootstrap.direction_verify_tick_budget = budget;
  rig.policy.setBootstrapContext(bootstrap);
}

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
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper(), 2100)));
  armCalibrationPermit(rig, second);

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
  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  rig.policy.setBootstrapContext(bootstrap);

  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  CHECK(rig.policy.transforms().admit(promotedTransform(lfHip(), 1996)));
  armCalibrationPermit(rig, lease);

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
  GeometryProvenance impostor = actuator::geometry_data::kProvenance;
  impostor.urdf_sha256[0] = (impostor.urdf_sha256[0] == 'a') ? 'b' : 'a';
  rig.policy.bindGeometry(&rig.profile, &impostor);

  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig, lease);

  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result =
      rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), 12);
  CHECK_EQ((int)result.outcome,
           (int)CalibrationExecutionOutcome::REJECT_NO_GEOMETRY_BINDING);
  CHECK_EQ(rig.backend.calls, 0);

  Rig rig2;
  rig2.policy.bindGeometry(nullptr, nullptr);
  const AuthorityLease lease2 = grant(rig2.arbiter, ActuatorAuthority::CALIBRATION,
                                      OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig2, lease2);
  const CalibrationExecutionResult unbound =
      rig2.engine.execute(req, liveContext(lease2, OperatingMode::MAINTENANCE), 12);
  CHECK_EQ((int)unbound.outcome,
           (int)CalibrationExecutionOutcome::REJECT_NO_GEOMETRY_BINDING);
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
  const AuthorityLease lease = grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig, lease);
  CHECK(rig.policy.transforms().empty());  // the shipped state: nothing admitted

  // LF UPPER MIN_SIDE is EXECUTABLE with no parking required, so this
  // request clears every earlier gate and is refused for exactly one
  // reason: no accepted raw<->q transform exists. The engine's target_urad
  // stays at its zero default throughout — it never computes one.
  const CalibrationExecutionRequest req =
      contactProbe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  const CalibrationExecutionResult result =
      rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), /*bus_id=*/12);

  CHECK_EQ((int)result.outcome, (int)CalibrationExecutionOutcome::REJECT_NO_TRANSFORM);
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
    CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper(), 2100)));
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
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper(), 2100)));
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

// ---------------------------------------------------------------------------
// Staged endpoint search (2026-09-29): a CONTACT_PROBE step is a raw tick
// bounded by the endpoint's calibration search corridor (URDF limit + 64 on
// the probe side, the other URDF limit behind), re-derived by the policy. The
// V25 calibration speed profile exists only for the calibration moves. The
// contact+16 / URDF-clamp allowance this replaces is gone.
// ---------------------------------------------------------------------------

struct UpperCase {
  Leg leg;
  const char* unit;
  uint8_t bus;
  uint16_t q0;  // the 2026-09-29 22:05 current-boot capture
};
constexpr UpperCase kUppers[4] = {{Leg::LF, "ELR01", 12, 2086},
                                  {Leg::RF, "ELR03", 22, 2108},
                                  {Leg::RH, "ELR02", 32, 2042},
                                  {Leg::LH, "M42", 42, 2072}};

void armProbeRig(Rig& rig, const UpperCase& u, ContactSide side, AuthorityLease* lease_out) {
  CHECK(rig.policy.transforms().admit(
      promotedTransform(joint(u.leg, JointKind::UPPER, u.unit), u.q0)));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  const actuator::GeometryEndpointRecord* ep =
      rig.profile.findEndpoint(u.leg, JointKind::UPPER, side);
  CHECK(ep != nullptr);
  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  bootstrap.motion_permit_active = true;
  bootstrap.motion_permit_generation = 1;
  bootstrap.motion_permit_session_id = 1;
  bootstrap.motion_permit_authority_generation = lease.generation;
  bootstrap.auxiliary_parked =
      ep != nullptr && ep->parking == actuator::ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND;
  bootstrap.parked_leg = u.leg;
  bootstrap.parked_joint = JointKind::UPPER;
  bootstrap.parked_side = side;
  rig.policy.setBootstrapContext(bootstrap);
  *lease_out = lease;
}

actuator::CalibrationSearchCorridor corridorOf(const Rig& rig, const UpperCase& u, ContactSide side) {
  actuator::CalibrationSearchCorridor c{};
  CHECK(actuator::resolveCalibrationSearchCorridor(
            rig.profile, actuator::geometry_data::kProvenance,
            promotedTransform(joint(u.leg, JointKind::UPPER, u.unit), u.q0), u.leg, JointKind::UPPER,
            side, &c) == actuator::TargetResolveStatus::OK);
  return c;
}

CalibrationExecutionRequest searchStep(const UpperCase& u, ContactSide side, uint16_t tick) {
  CalibrationExecutionRequest req =
      contactProbe(joint(u.leg, JointKind::UPPER, u.unit), u.leg, JointKind::UPPER, side);
  req.calibration_search = true;
  req.search_target_tick = tick;
  req.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;
  return req;
}

void test_corridor_is_v25_guard_and_entry_every_leg_both_sides() {
  g_case = "corridor: guard = URDF limit + 64, entry = URDF limit - 64, contact inside";
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
      Rig rig;
      const actuator::CalibrationSearchCorridor c = corridorOf(rig, u, side);
      const actuator::GeometryJointRecord* j = rig.profile.findJoint(joint(u.leg, JointKind::UPPER, u.unit));
      uint16_t limit = 0;
      CHECK(actuator::resolveUrdfQToRaw(rig.profile, actuator::geometry_data::kProvenance,
                                        promotedTransform(joint(u.leg, JointKind::UPPER, u.unit), u.q0),
                                        side == ContactSide::MIN_SIDE ? j->urdf_lower : j->urdf_upper,
                                        &limit) == actuator::TargetResolveStatus::OK);
      CHECK_EQ(c.urdf_limit_tick, limit);
      CHECK_EQ(c.guard_tick, limit + c.probe_sign * 64);
      CHECK_EQ(c.entry_tick, limit - c.probe_sign * 64);
      CHECK_EQ(c.home_tick, u.q0);
      // Guard-to-contact room covers the hand-found LF MIN stop (~23 past contact).
      CHECK(actuator::searchDepth(c, c.guard_tick) - actuator::searchDepth(c, c.contact_tick) >= 64);
      // The raw direction follows the joint's URDF direction and the side.
      const int dir = u.leg == Leg::RF || u.leg == Leg::RH ? -1 : 1;
      CHECK_EQ(c.probe_sign, dir * (side == ContactSide::MIN_SIDE ? -1 : 1));
    }
  }
  {
    Rig rig;  // a q0 that pushes the MAX guard past raw 4095 is refused, never wrapped
    actuator::CalibrationSearchCorridor c{};
    CHECK(actuator::resolveCalibrationSearchCorridor(
              rig.profile, actuator::geometry_data::kProvenance,
              promotedTransform(joint(Leg::LF, JointKind::UPPER, "ELR01"), 2700), Leg::LF,
              JointKind::UPPER, ContactSide::MAX_SIDE, &c) != actuator::TargetResolveStatus::OK);
    CHECK(!c.valid());
  }
  {
    Rig rig;  // the corridor belongs to the probed joint itself
    actuator::CalibrationSearchCorridor c{};
    CHECK(actuator::resolveCalibrationSearchCorridor(
              rig.profile, actuator::geometry_data::kProvenance,
              promotedTransform(joint(Leg::LF, JointKind::UPPER, "ELR01"), 2086), Leg::RF,
              JointKind::UPPER, ContactSide::MIN_SIDE, &c) == actuator::TargetResolveStatus::REJECT_JOINT);
  }
}

void test_search_step_bounded_by_the_corridor_every_leg_both_sides() {
  g_case = "search step: guard and opposite URDF limit admitted, one tick past either refused";
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
      const actuator::CalibrationSearchCorridor c = [&] { Rig r; return corridorOf(r, u, side); }();
      const int s = c.probe_sign;
      struct { int tick; bool ok; } cases[] = {{c.guard_tick, true},
                                               {c.guard_tick + s, false},
                                               {c.contact_tick, true},
                                               {c.entry_tick, true},
                                               {c.opposite_limit_tick, true},
                                               {c.opposite_limit_tick - s, false}};
      for (const auto& k : cases) {
        Rig rig;
        AuthorityLease lease{};
        armProbeRig(rig, u, side, &lease);
        const CalibrationExecutionResult r = rig.engine.execute(
            searchStep(u, side, static_cast<uint16_t>(k.tick)),
            liveContext(lease, OperatingMode::MAINTENANCE), u.bus);
        CHECK_EQ((int)r.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
        CHECK_EQ((int)r.policy_decision,
                 (int)(k.ok ? WriteDecision::ACCEPT : WriteDecision::REJECT_CALIBRATION_SEARCH));
        CHECK_EQ(rig.backend.calls, k.ok ? 1 : 0);
        if (k.ok) {
          CHECK_EQ(rig.backend.last_target_tick, k.tick);
          CHECK((int)rig.backend.last_profile == (int)actuator::MotionProfile::CALIBRATION_SEARCH);
        }
      }
      // The non-search CONTACT_PROBE rule is unchanged: never past the contact.
      Rig rig;
      AuthorityLease lease{};
      armProbeRig(rig, u, side, &lease);
      CalibrationExecutionRequest legacy =
          contactProbe(joint(u.leg, JointKind::UPPER, u.unit), u.leg, JointKind::UPPER, side);
      const actuator::MicroRad contact = rig.profile.findEndpoint(u.leg, JointKind::UPPER, side)->contact;
      legacy.target_urad = contact + (side == ContactSide::MIN_SIDE ? -1 : 1);
      CHECK_EQ((int)rig.engine.execute(legacy, liveContext(lease, OperatingMode::MAINTENANCE), u.bus)
                   .policy_decision,
               (int)WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS);
    }
  }
}

void test_search_and_calibration_profile_are_calibration_only() {
  g_case = "calibration search / speed profile: refused for every non-calibration command";
  const UpperCase& u = kUppers[0];
  const JointIdentity id = joint(u.leg, JointKind::UPPER, u.unit);
  Rig rig;
  AuthorityLease lease{};
  armProbeRig(rig, u, ContactSide::MAX_SIDE, &lease);
  CHECK(rig.policy.transforms().admit(promotedTransform(joint(Leg::LH, JointKind::UPPER, "M42"), 2072)));

  // Engine: a search step on another intent fails closed before the policy.
  CalibrationExecutionRequest dv{};
  dv.intent = CalibrationIntent::DIRECTION_VERIFY;
  dv.joint = id;
  dv.direction_verify_delta_ticks = 16;
  dv.calibration_search = true;
  CHECK_EQ((int)rig.engine.execute(dv, liveContext(lease, OperatingMode::MAINTENANCE), u.bus).outcome,
           (int)CalibrationExecutionOutcome::REJECT_TARGET_RESOLUTION);
  CHECK_EQ(rig.backend.calls, 0);

  // Policy, directly: the search flag on anything but CONTACT_PROBE...
  for (actuator::ActuatorOperation op :
       {actuator::ActuatorOperation::TORQUE_ENABLE, actuator::ActuatorOperation::POSITION_COMMAND,
        actuator::ActuatorOperation::DIRECTION_VERIFY,
        actuator::ActuatorOperation::CALIBRATION_AUXILIARY_MOVE}) {
    actuator::ActuatorCommand cmd{};
    cmd.operation = op;
    cmd.joint = id;
    cmd.target_tick = 2048;
    cmd.calibration_search = true;
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)rig.policy.plan(cmd, lease, OperatingMode::MAINTENANCE, &txn),
             (int)WriteDecision::REJECT_CALIBRATION_SEARCH);
  }
  // ...and the calibration speed profile on anything but the two calibration moves.
  for (actuator::ActuatorOperation op :
       {actuator::ActuatorOperation::TORQUE_ENABLE, actuator::ActuatorOperation::POSITION_COMMAND,
        actuator::ActuatorOperation::DIRECTION_VERIFY}) {
    actuator::ActuatorCommand cmd{};
    cmd.operation = op;
    cmd.joint = id;
    cmd.target_tick = 2048;
    cmd.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)rig.policy.plan(cmd, lease, OperatingMode::MAINTENANCE, &txn),
             (int)WriteDecision::REJECT_MOTION_PROFILE);
  }
  {  // a corrupted profile value is refused even for CONTACT_PROBE
    actuator::ActuatorCommand cmd{};
    cmd.operation = actuator::ActuatorOperation::CALIBRATION_CONTACT_PROBE;
    cmd.joint = id;
    cmd.endpoint_leg = u.leg;
    cmd.endpoint_joint = JointKind::UPPER;
    cmd.endpoint_side = ContactSide::MAX_SIDE;
    cmd.calibration_search = true;
    cmd.target_tick = corridorOf(rig, u, ContactSide::MAX_SIDE).contact_tick;
    cmd.motion_profile = static_cast<actuator::MotionProfile>(7);
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)rig.policy.plan(cmd, lease, OperatingMode::MAINTENANCE, &txn),
             (int)WriteDecision::REJECT_MOTION_PROFILE);
  }
  {  // stand/gait: MOTION owner, POSITION_COMMAND - neither flag, ever
    Rig motion_rig;
    CHECK(motion_rig.policy.transforms().admit(promotedTransform(id, u.q0)));
    const AuthorityLease motion =
        grant(motion_rig.arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);
    actuator::ActuatorCommand cmd{};
    cmd.operation = actuator::ActuatorOperation::POSITION_COMMAND;
    cmd.joint = id;
    cmd.target_tick = 2048;
    cmd.calibration_search = true;
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)motion_rig.policy.plan(cmd, motion, OperatingMode::RUN, &txn),
             (int)WriteDecision::REJECT_CALIBRATION_SEARCH);
    cmd.calibration_search = false;
    cmd.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;
    CHECK_EQ((int)motion_rig.policy.plan(cmd, motion, OperatingMode::RUN, &txn),
             (int)WriteDecision::REJECT_MOTION_PROFILE);
  }
  {  // the auxiliary park may use the calibration profile, at its exact pose only
    CalibrationExecutionRequest aux{};
    aux.intent = CalibrationIntent::AUXILIARY_MOVE;
    aux.joint = joint(Leg::LH, JointKind::UPPER, "M42");
    aux.endpoint_leg = Leg::LF;
    aux.endpoint_joint = JointKind::UPPER;
    aux.endpoint_side = ContactSide::MAX_SIDE;
    aux.target_urad = 610865;
    aux.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;
    const CalibrationExecutionResult r =
        rig.engine.execute(aux, liveContext(lease, OperatingMode::MAINTENANCE), 42);
    CHECK_EQ((int)r.policy_decision, (int)WriteDecision::ACCEPT);
    CHECK((int)rig.backend.last_profile == (int)actuator::MotionProfile::CALIBRATION_SEARCH);
  }
  CHECK(std::strcmp(actuator::toString(actuator::MotionProfile::CALIBRATION_SEARCH),
                    "CALIBRATION_SEARCH") == 0);
  CHECK(std::strcmp(actuator::toString(static_cast<actuator::MotionProfile>(7)), "UNKNOWN") == 0);
}

int main() {
  test_corridor_is_v25_guard_and_entry_every_leg_both_sides();
  test_search_step_bounded_by_the_corridor_every_leg_both_sides();
  test_search_and_calibration_profile_are_calibration_only();
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
