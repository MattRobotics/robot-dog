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
  actuator::BackendWriteOutcome writeGoalPosition(uint8_t, uint16_t target_tick) override {
    ++calls;
    last_target_tick = target_tick;
    return actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  }
  int calls = 0;
  uint16_t last_target_tick = 0;
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
// Contact-probe overtravel allowance (hardware finding 2026-09-29,
// operator-approved): LF_UPPER's MIN stop sat 4-5 ticks short of the Geometry V5
// contact, inside the 4-tick arrival tolerance. Both approach passes may now be
// commanded past the canonical contact by at most 16 raw ticks, CLAMPED to the
// URDF joint limit - CONTACT_PROBE only, anchored on exactly the contact, never
// the backoff. Hand-computed oracle (1 tick = 1533.98 urad, independent of q0
// and of the raw direction): UPPER MIN contact -909889 urad is 593 ticks out,
// the URDF lower limit -916298 still admits 597 (598 converts to -917321) ->
// 4 ticks of room; MAX contact 2127120 is 1387 ticks out, the URDF upper
// 2138028 admits 1393 (1394 converts to 2138369) -> 6 ticks of room.
// ---------------------------------------------------------------------------

struct UpperCase {
  Leg leg;
  const char* unit;
  uint8_t bus;
  uint16_t q0;  // the 2026-09-29 current-boot capture
};
constexpr UpperCase kUppers[4] = {{Leg::LF, "ELR01", 12, 2088},
                                  {Leg::RF, "ELR03", 22, 2108},
                                  {Leg::RH, "ELR02", 32, 2042},
                                  {Leg::LH, "M42", 42, 2088}};

// A live CALIBRATION rig for one UPPER joint and side: transform admitted,
// permit armed, and the auxiliary reported parked exactly when the compiled
// plan for that endpoint requires it (LF/RF MAX).
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

constexpr int kUpperMinRoomTicks = 4;
constexpr int kUpperMaxRoomTicks = 6;

bool insideUrdf(const Rig& rig, const UpperCase& u, int tick) {
  if (tick < 0 || tick > 4095) return false;
  actuator::MicroRad q = 0;
  return actuator::resolveRawToUrdfQ(rig.profile, actuator::geometry_data::kProvenance,
                                     promotedTransform(joint(u.leg, JointKind::UPPER, u.unit), u.q0),
                                     static_cast<uint16_t>(tick), &q) ==
         actuator::TargetResolveStatus::OK;
}

uint16_t contactTick(const Rig& rig, const UpperCase& u, ContactSide side) {
  const actuator::GeometryEndpointRecord* ep =
      rig.profile.findEndpoint(u.leg, JointKind::UPPER, side);
  uint16_t tick = 0;
  CHECK(actuator::resolveUrdfQToRaw(rig.profile, actuator::geometry_data::kProvenance,
                                    promotedTransform(joint(u.leg, JointKind::UPPER, u.unit), u.q0),
                                    ep->contact, &tick) == actuator::TargetResolveStatus::OK);
  return tick;
}

void test_overtravel_clamped_to_urdf_limit_both_sides_all_legs() {
  g_case = "overtravel: min(16, room to the URDF limit) past the contact, MIN and MAX, 4 legs";
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
      Rig rig;
      AuthorityLease lease{};
      armProbeRig(rig, u, side, &lease);
      const actuator::GeometryEndpointRecord* ep =
          rig.profile.findEndpoint(u.leg, JointKind::UPPER, side);
      CalibrationExecutionRequest req =
          contactProbe(joint(u.leg, JointKind::UPPER, u.unit), u.leg, JointKind::UPPER, side);
      req.target_urad = ep->contact;  // the canonical contact, unchanged
      req.contact_probe_overtravel_ticks = 16;
      const CalibrationExecutionResult r =
          rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), u.bus);
      CHECK_EQ((int)r.outcome, (int)CalibrationExecutionOutcome::ROUTED_TO_POLICY);
      CHECK_EQ((int)r.policy_decision, (int)WriteDecision::ACCEPT);
      CHECK_EQ((int)r.execute_result, (int)actuator::ExecuteResult::WRITTEN);
      const int contact = contactTick(rig, u, side);
      const int written = rig.backend.last_target_tick;
      const int applied = written > contact ? written - contact : contact - written;
      // The oracle's room, not the 16-tick ceiling.
      CHECK_EQ(applied, side == ContactSide::MIN_SIDE ? kUpperMinRoomTicks : kUpperMaxRoomTicks);
      CHECK(applied <= 16);
      // On the FAR side of the contact from q0, whichever way the raw axis runs.
      const int q0 = u.q0;
      const int contact_travel = contact > q0 ? contact - q0 : q0 - contact;
      const int written_travel = written > q0 ? written - q0 : q0 - written;
      CHECK_EQ(written_travel, contact_travel + applied);
      // Never beyond the URDF limit: the commanded tick is inside, the next one
      // further is not (the clamp is tight, not merely conservative).
      const int further = written + (written > contact ? 1 : -1);
      CHECK(insideUrdf(rig, u, written));
      CHECK(!insideUrdf(rig, u, further));
    }
  }
}

void test_zero_overtravel_is_exactly_the_canonical_contact() {
  g_case = "overtravel 0: unchanged canonical contact target";
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
      Rig rig;
      AuthorityLease lease{};
      armProbeRig(rig, u, side, &lease);
      CalibrationExecutionRequest req =
          contactProbe(joint(u.leg, JointKind::UPPER, u.unit), u.leg, JointKind::UPPER, side);
      req.target_urad = rig.profile.findEndpoint(u.leg, JointKind::UPPER, side)->contact;
      const CalibrationExecutionResult r =
          rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), u.bus);
      CHECK_EQ((int)r.policy_decision, (int)WriteDecision::ACCEPT);
      CHECK_EQ(rig.backend.last_target_tick, contactTick(rig, u, side));
    }
  }
}

void test_overtravel_ceiling_is_not_a_travel_amount() {
  g_case = "overtravel: 16 is an absolute ceiling, not a mandatory travel amount";
  const UpperCase& u = kUppers[0];
  Rig rig;
  const actuator::JointTransform t = promotedTransform(joint(u.leg, JointKind::UPPER, u.unit), u.q0);
  const actuator::GeometryProvenance& prov = actuator::geometry_data::kProvenance;
  struct { actuator::MicroRad contact; ContactSide side; } deep[] = {
      {-800000, ContactSide::MIN_SIDE},   // ~75 ticks of room before the URDF lower limit
      {2000000, ContactSide::MAX_SIDE}};  // ~90 ticks of room before the URDF upper limit
  for (const auto& d : deep) {
    uint16_t base = 0;
    CHECK(actuator::resolveUrdfQToRaw(rig.profile, prov, t, d.contact, &base) ==
          actuator::TargetResolveStatus::OK);
    for (uint16_t want : {0, 1, 8, 16}) {
      uint16_t tick = 0, applied = 999;
      CHECK(actuator::resolveContactProbeApproachToRaw(rig.profile, prov, t, d.contact, d.side,
                                                       want, &tick, &applied) ==
            actuator::TargetResolveStatus::OK);
      CHECK_EQ(applied, want);  // plenty of room: exactly what was asked, never more
      CHECK_EQ(std::abs((int)tick - (int)base), (int)want);
    }
    uint16_t tick = 0;
    CHECK(actuator::resolveContactProbeApproachToRaw(rig.profile, prov, t, d.contact, d.side, 17,
                                                     &tick) ==
          actuator::TargetResolveStatus::REJECT_OVERTRAVEL);
  }
  // At the real UPPER contacts a request below the room is honoured as-is and
  // one above it is clamped to the room - never raised to 16.
  for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
    const actuator::MicroRad contact = rig.profile.findEndpoint(u.leg, JointKind::UPPER, side)->contact;
    const int room = side == ContactSide::MIN_SIDE ? kUpperMinRoomTicks : kUpperMaxRoomTicks;
    for (uint16_t want : {0, 2, 16}) {
      uint16_t tick = 0, applied = 999;
      CHECK(actuator::resolveContactProbeApproachToRaw(rig.profile, prov, t, contact, side, want,
                                                       &tick, &applied) ==
            actuator::TargetResolveStatus::OK);
      CHECK_EQ((int)applied, (int)want < room ? (int)want : room);
    }
  }
}

void test_overtravel_above_16_refused_at_every_layer() {
  g_case = "overtravel: the bound cannot exceed 16";
  const UpperCase& u = kUppers[0];
  const JointIdentity id = joint(u.leg, JointKind::UPPER, u.unit);
  for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
    Rig rig;
    AuthorityLease lease{};
    armProbeRig(rig, u, side, &lease);
    const actuator::MicroRad contact = rig.profile.findEndpoint(u.leg, JointKind::UPPER, side)->contact;
    const actuator::JointTransform t = promotedTransform(id, u.q0);

    // Resolver: 16 is the ceiling, 17 is not a number it will produce.
    uint16_t tick16 = 0, tick17 = 0;
    CHECK(actuator::resolveContactProbeApproachToRaw(rig.profile, actuator::geometry_data::kProvenance,
                                                     t, contact, side, 16, &tick16) ==
          actuator::TargetResolveStatus::OK);
    CHECK(actuator::resolveContactProbeApproachToRaw(rig.profile, actuator::geometry_data::kProvenance,
                                                     t, contact, side, 17, &tick17) ==
          actuator::TargetResolveStatus::REJECT_OVERTRAVEL);
    CHECK_EQ((int)actuator::kContactProbeMaxOvertravelTicks, 16);

    // Engine: 17 cannot even be resolved - nothing reaches the backend.
    CalibrationExecutionRequest req = contactProbe(id, u.leg, JointKind::UPPER, side);
    req.target_urad = contact;
    req.contact_probe_overtravel_ticks = 17;
    CalibrationExecutionResult r =
        rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), u.bus);
    CHECK_EQ((int)r.outcome, (int)CalibrationExecutionOutcome::REJECT_TARGET_RESOLUTION);
    CHECK_EQ(rig.backend.calls, 0);

    // Policy, called directly with a hand-built command: 17 ticks, a tick one
    // past the clamped point (outside the URDF domain), the UNclamped
    // contact+16, a tick short of the clamped point and 200 are all refused.
    const int contact_tick = contactTick(rig, u, side);
    const int step = (tick16 > contact_tick) ? 1 : -1;
    struct { uint16_t overtravel; int tick; WriteDecision expected; } bad[] = {
        {17, tick16 + step, WriteDecision::REJECT_PROBE_OVERTRAVEL},
        {16, tick16 + step, WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS},
        {16, contact_tick + 16 * step, WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS},
        {16, tick16 - step, WriteDecision::REJECT_PROBE_OVERTRAVEL},
        {200, tick16, WriteDecision::REJECT_PROBE_OVERTRAVEL}};
    for (const auto& b : bad) {
      actuator::ActuatorCommand cmd{};
      cmd.operation = actuator::ActuatorOperation::CALIBRATION_CONTACT_PROBE;
      cmd.joint = id;
      cmd.endpoint_leg = u.leg;
      cmd.endpoint_joint = JointKind::UPPER;
      cmd.endpoint_side = side;
      cmd.target_urad = contact;
      cmd.target_tick = static_cast<uint16_t>(b.tick);
      cmd.contact_probe_overtravel_ticks = b.overtravel;
      actuator::ActuatorTransaction txn{};
      CHECK_EQ((int)rig.policy.plan(cmd, lease, OperatingMode::MAINTENANCE, &txn),
               (int)b.expected);
    }
    // ...and exactly the URDF-clamped point is accepted.
    actuator::ActuatorCommand ok{};
    ok.operation = actuator::ActuatorOperation::CALIBRATION_CONTACT_PROBE;
    ok.joint = id;
    ok.endpoint_leg = u.leg;
    ok.endpoint_joint = JointKind::UPPER;
    ok.endpoint_side = side;
    ok.target_urad = contact;
    ok.target_tick = tick16;
    ok.contact_probe_overtravel_ticks = 16;
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)rig.policy.plan(ok, lease, OperatingMode::MAINTENANCE, &txn),
             (int)WriteDecision::ACCEPT);
    rig.policy.abort(&txn);
  }
}

void test_overtravel_is_contact_probe_only() {
  g_case = "overtravel: non-CONTACT_PROBE commands can never carry it";
  const UpperCase& u = kUppers[0];
  const JointIdentity id = joint(u.leg, JointKind::UPPER, u.unit);
  Rig rig;
  AuthorityLease lease{};
  armProbeRig(rig, u, ContactSide::MAX_SIDE, &lease);

  // Engine: AUXILIARY_MOVE and DIRECTION_VERIFY refuse it before the policy
  // (the auxiliary's own transform is admitted so that refusal, not a missing
  // transform, is what is exercised).
  CHECK(rig.policy.transforms().admit(promotedTransform(joint(Leg::LH, JointKind::UPPER, "M42"), 2088)));
  CalibrationExecutionRequest aux{};
  aux.intent = CalibrationIntent::AUXILIARY_MOVE;
  aux.joint = joint(Leg::LH, JointKind::UPPER, "M42");
  aux.endpoint_leg = Leg::LF;
  aux.endpoint_joint = JointKind::UPPER;
  aux.endpoint_side = ContactSide::MAX_SIDE;
  aux.target_urad = 610865;
  aux.contact_probe_overtravel_ticks = 16;
  CHECK_EQ((int)rig.engine.execute(aux, liveContext(lease, OperatingMode::MAINTENANCE), 42).outcome,
           (int)CalibrationExecutionOutcome::REJECT_TARGET_RESOLUTION);
  CalibrationExecutionRequest dv{};
  dv.intent = CalibrationIntent::DIRECTION_VERIFY;
  dv.joint = id;
  dv.direction_verify_delta_ticks = 16;
  dv.contact_probe_overtravel_ticks = 16;
  CHECK_EQ((int)rig.engine.execute(dv, liveContext(lease, OperatingMode::MAINTENANCE), u.bus).outcome,
           (int)CalibrationExecutionOutcome::REJECT_TARGET_RESOLUTION);
  CHECK_EQ(rig.backend.calls, 0);

  // Policy, directly: every other operation, for every owner that could hold
  // it - including the future stand/gait POSITION_COMMAND under MOTION.
  for (actuator::ActuatorOperation op :
       {actuator::ActuatorOperation::TORQUE_ENABLE, actuator::ActuatorOperation::POSITION_COMMAND,
        actuator::ActuatorOperation::DIRECTION_VERIFY,
        actuator::ActuatorOperation::CALIBRATION_AUXILIARY_MOVE}) {
    actuator::ActuatorCommand cmd{};
    cmd.operation = op;
    cmd.joint = id;
    cmd.endpoint_leg = u.leg;
    cmd.endpoint_joint = JointKind::UPPER;
    cmd.endpoint_side = ContactSide::MAX_SIDE;
    cmd.target_tick = 2048;
    cmd.contact_probe_overtravel_ticks = 1;
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)rig.policy.plan(cmd, lease, OperatingMode::MAINTENANCE, &txn),
             (int)WriteDecision::REJECT_PROBE_OVERTRAVEL);
  }
  {
    Rig motion_rig;
    CHECK(motion_rig.policy.transforms().admit(promotedTransform(id, u.q0)));
    const AuthorityLease motion =
        grant(motion_rig.arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);
    actuator::ActuatorCommand cmd{};
    cmd.operation = actuator::ActuatorOperation::POSITION_COMMAND;
    cmd.joint = id;
    cmd.target_tick = 2048;
    cmd.contact_probe_overtravel_ticks = 16;
    actuator::ActuatorTransaction txn{};
    CHECK_EQ((int)motion_rig.policy.plan(cmd, motion, OperatingMode::RUN, &txn),
             (int)WriteDecision::REJECT_PROBE_OVERTRAVEL);
  }
}

void test_backoff_can_never_carry_the_overtravel() {
  g_case = "overtravel: the backoff target is unchanged and cannot carry it";
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
      Rig rig;
      AuthorityLease lease{};
      armProbeRig(rig, u, side, &lease);
      const JointIdentity id = joint(u.leg, JointKind::UPPER, u.unit);
      const actuator::MicroRad backoff =
          rig.profile.findEndpoint(u.leg, JointKind::UPPER, side)->contact / 2;
      CalibrationExecutionRequest req = contactProbe(id, u.leg, JointKind::UPPER, side);
      req.target_urad = backoff;
      req.contact_probe_overtravel_ticks = 16;
      CalibrationExecutionResult r =
          rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), u.bus);
      CHECK_EQ((int)r.policy_decision, (int)WriteDecision::REJECT_PROBE_OVERTRAVEL);
      CHECK_EQ(rig.backend.calls, 0);
      // Without it, the backoff resolves exactly as before.
      req.contact_probe_overtravel_ticks = 0;
      r = rig.engine.execute(req, liveContext(lease, OperatingMode::MAINTENANCE), u.bus);
      CHECK_EQ((int)r.policy_decision, (int)WriteDecision::ACCEPT);
      uint16_t expected = 0;
      CHECK(actuator::resolveUrdfQToRaw(rig.profile, actuator::geometry_data::kProvenance,
                                        promotedTransform(id, u.q0), backoff, &expected) ==
            actuator::TargetResolveStatus::OK);
      CHECK_EQ(rig.backend.last_target_tick, expected);
    }
  }
}

int main() {
  test_overtravel_clamped_to_urdf_limit_both_sides_all_legs();
  test_overtravel_ceiling_is_not_a_travel_amount();
  test_zero_overtravel_is_exactly_the_canonical_contact();
  test_overtravel_above_16_refused_at_every_layer();
  test_overtravel_is_contact_probe_only();
  test_backoff_can_never_carry_the_overtravel();
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
