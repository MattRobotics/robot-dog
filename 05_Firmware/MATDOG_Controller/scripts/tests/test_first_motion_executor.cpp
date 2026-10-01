// Offline adversarial tests for the CR3 Priority 3 first-motion executor
// (src/calibration/FirstMotionExecutor.*).
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, ActuatorAuthorityArbiter
// and the checked target resolver, against a fake backend and synthetic
// telemetry - the same contract as test_calibration_execution_engine.cpp and
// test_actuator_runtime.cpp. A mocked policy/resolver here would test the
// mock's idea of the rules, not the shipped ones.
//
// NO HARDWARE VALIDATION. The fake backend never touches ServoBus, and
// MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED stays 0.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/calibration/FirstMotionExecutor.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::ActuatorRuntime;
using matdog::actuator::BackendWriteOutcome;
using matdog::actuator::CalibrationBootstrapContext;
using matdog::actuator::CalibrationGeometryProfile;
using matdog::actuator::MotionDeadmanConfig;
using matdog::actuator::SafeActuatorPolicy;
using matdog::actuator::TelemetrySample;
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
    const long a_ = (long)(actual);                                       \
    const long e_ = (long)(expected);                                     \
    if (a_ != e_) {                                                       \
      ++g_failures;                                                       \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case, \
                  __FILE__, __LINE__, #actual, a_, e_);                   \
    }                                                                      \
  } while (0)

namespace {

JointIdentity lfUpper() {
  JointIdentity id{};
  id.leg = Leg::LF;
  id.joint = JointKind::UPPER;
  setPhysicalUnit(&id, "ELR01");
  return id;
}

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&actuator::geometry_data::kProvenance, actuator::geometry_data::kJoints,
              actuator::geometry_data::kJointCount, actuator::geometry_data::kEndpoints,
              actuator::geometry_data::kEndpointCount);
  return profile;
}

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

AuthorityLease grant(ActuatorAuthorityArbiter& arbiter, ActuatorAuthority owner,
                     OperatingMode mode) {
  AuthorityLease lease{};
  const AuthorityResult result = arbiter.request(owner, mode, &lease);
  CHECK(result == AuthorityResult::GRANTED);
  return lease;
}

void armCalibrationPermit(SafeActuatorPolicy& policy, const AuthorityLease& lease,
                          int32_t budget = 64) {
  CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  bootstrap.motion_permit_active = true;
  bootstrap.motion_permit_generation = 1;
  bootstrap.motion_permit_session_id = 1;
  bootstrap.motion_permit_authority_generation = lease.generation;
  bootstrap.direction_verify_tick_budget = budget;
  policy.setBootstrapContext(bootstrap);
}

FirstMotionContext liveContext(const AuthorityLease& lease, OperatingMode mode) {
  FirstMotionContext ctx{};
  ctx.session_active = true;
  ctx.origin = CalibrationOrigin::LIVE_SESSION;
  ctx.lease = lease;
  ctx.mode = mode;

  // Fresh dynamic continuation facts. Production Controller supplies these
  // every tick; tests must model the same live state rather than relying on
  // the refusing defaults.
  ctx.motion_permit_active = true;
  ctx.authority = ActuatorAuthority::CALIBRATION;
  ctx.authority_generation = lease.generation;
  ctx.authority_inhibited = false;
  return ctx;
}

MotionDeadmanConfig deadmanConfig() {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = 4;
  return c;
}

TelemetrySample telemetry(int32_t position, int32_t torque_enable, uint32_t at_ms) {
  TelemetrySample s{};
  s.read_ok = true;
  s.sampled_at_ms = at_ms;
  s.present_position = position;
  s.torque_enable = torque_enable;
  return s;
}

// Records every call, with per-call-kind steerable results across a SEQUENCE
// (not just one fixed value) so a test can arm "the Nth call returns X"
// without a second fixture - the executor makes at most one backend call per
// update() tick, so this models a real multi-tick attempt precisely.
class FakeActuatorBackend : public actuator::ActuatorBackend {
 public:
  BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    ++enable_torque_calls;
    last_bus_id = bus_id;
    return enable_torque_result;
  }
  BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick,
                                        actuator::MotionProfile profile) override {
    ++write_goal_position_calls;
    last_bus_id = bus_id;
    last_target_tick = target_tick;
    last_profile = profile;
    return write_goal_position_result;
  }
  BackendWriteOutcome writeCalibrationTorqueLimit(uint8_t bus_id) override {
    ++torque_limit_calls;
    last_bus_id = bus_id;
    return torque_limit_result;
  }
  actuator::MotionProfile last_profile = actuator::MotionProfile::BOUNDED_DEFAULT;

  int enable_torque_calls = 0;
  int write_goal_position_calls = 0;
  int torque_limit_calls = 0;
  uint8_t last_bus_id = 0;
  uint16_t last_target_tick = 0;
  BackendWriteOutcome enable_torque_result = BackendWriteOutcome::VERIFIED_APPLIED;
  BackendWriteOutcome write_goal_position_result = BackendWriteOutcome::VERIFIED_APPLIED;
  BackendWriteOutcome torque_limit_result = BackendWriteOutcome::VERIFIED_APPLIED;

  int totalCalls() const {
    return enable_torque_calls + write_goal_position_calls + torque_limit_calls;
  }
};

struct Rig {
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  CalibrationGeometryProfile profile;
  FirstMotionExecutor executor;

  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    runtime.begin(&policy, &backend);
    FirstMotionConfig config{};
    config.deadman = deadmanConfig();
    executor.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance, config);
  }
};

FirstMotionRequest request(int32_t delta_ticks = 32, uint8_t bus_id = 12) {
  FirstMotionRequest r{};
  r.joint = lfUpper();
  r.bus_id = bus_id;
  r.delta_ticks = delta_ticks;
  return r;
}

// ---------------------------------------------------------------------------
// The full happy path: exactly one backend call per tick, ends COMPLETE.
// ---------------------------------------------------------------------------

void test_happy_path_one_backend_call_per_tick_then_arrives() {
  g_case = "happy path";
  Rig rig;
  CHECK(rig.policy.limits().empty());  // sanity: unrelated to this path
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(/*delta_ticks=*/32), ctx, /*now_ms=*/1000));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::TORQUE_ENABLE_PENDING);
  CHECK(rig.executor.active());

  // Tick 1: TorqueEnable only.
  rig.executor.update(ctx, 1010, false, TelemetrySample{});
  CHECK_EQ(rig.backend.enable_torque_calls, 1);
  CHECK_EQ(rig.backend.write_goal_position_calls, 0);
  CHECK_EQ(rig.backend.last_bus_id, 12);
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::POSITION_COMMAND_PENDING);

  // Tick 2: GoalPosition only. q0=2048, delta=+32 -> target 2080.
  rig.executor.update(ctx, 1020, false, TelemetrySample{});
  CHECK_EQ(rig.backend.enable_torque_calls, 1);
  CHECK_EQ(rig.backend.write_goal_position_calls, 1);
  CHECK_EQ(rig.backend.last_target_tick, 2080);
  // DIRECTION_VERIFY never inherits the calibration search speed profile.
  CHECK((int)rig.backend.last_profile == (int)actuator::MotionProfile::BOUNDED_DEFAULT);
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::MONITORING);
  CHECK_EQ(rig.executor.status().target_tick, 2080);

  // Tick 3+: telemetry monitoring, no further backend calls at all.
  rig.executor.update(ctx, 1200, true, telemetry(2000, 1, 1200));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::MONITORING);
  rig.executor.update(ctx, 1400, true, telemetry(2080, 1, 1400));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::COMPLETE);
  CHECK_EQ(rig.executor.status().failure, (int)FirstMotionFailure::NONE);
  CHECK_EQ(rig.backend.totalCalls(), 2);  // never more than the two writes total
  CHECK(!rig.executor.active());
}

// ---------------------------------------------------------------------------
// Preconditions: no permit -> refused before any backend call.
// ---------------------------------------------------------------------------

void test_no_permit_rejects_before_any_backend_call() {
  g_case = "no permit -> no backend call";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  // Deliberately no armCalibrationPermit() call - no motion permit exists.
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(), ctx, 1000));
  rig.executor.update(ctx, 1010, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::FAILED_NO_MOTION);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::REJECT_PRECONDITIONS);
  CHECK_EQ(rig.backend.totalCalls(), 0);
}

// ---------------------------------------------------------------------------
// TorqueEnable verified-rejected: safe, no SAFE_OFF needed.
// ---------------------------------------------------------------------------

void test_torque_enable_verified_rejected_is_failed_no_motion() {
  g_case = "torque enable verified rejected";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  rig.backend.enable_torque_result = BackendWriteOutcome::VERIFIED_NOT_APPLIED;
  CHECK(rig.executor.start(request(), ctx, 1000));
  rig.executor.update(ctx, 1010, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::FAILED_NO_MOTION);
  CHECK_EQ((int)rig.executor.status().failure,
          (int)FirstMotionFailure::TORQUE_ENABLE_REJECTED);
  CHECK_EQ(rig.backend.totalCalls(), 1);
}

// ---------------------------------------------------------------------------
// TorqueEnable UNCERTAIN: must escalate, distinctly from a verified failure.
// ---------------------------------------------------------------------------

void test_torque_enable_uncertain_requires_safe_off() {
  g_case = "torque enable uncertain -> SAFE_OFF_REQUIRED";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  rig.backend.enable_torque_result = BackendWriteOutcome::UNCERTAIN;
  CHECK(rig.executor.start(request(), ctx, 1000));
  rig.executor.update(ctx, 1010, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure,
          (int)FirstMotionFailure::TORQUE_ENABLE_UNCERTAIN);
  // No GoalPosition attempt after an uncertain torque outcome.
  CHECK_EQ(rig.backend.write_goal_position_calls, 0);
}

// ---------------------------------------------------------------------------
// Position command UNCERTAIN after torque confirmed on: must escalate.
// ---------------------------------------------------------------------------

void test_position_command_uncertain_after_torque_on_requires_safe_off() {
  g_case = "position command uncertain -> SAFE_OFF_REQUIRED";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  rig.backend.write_goal_position_result = BackendWriteOutcome::UNCERTAIN;
  CHECK(rig.executor.start(request(), ctx, 1000));
  rig.executor.update(ctx, 1010, false, TelemetrySample{});  // torque enable: applied
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::POSITION_COMMAND_PENDING);
  rig.executor.update(ctx, 1020, false, TelemetrySample{});  // position command: uncertain

  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure,
          (int)FirstMotionFailure::POSITION_COMMAND_UNCERTAIN);
}

// ---------------------------------------------------------------------------
// Permit revoked between the two steps: torque is on, position refused ->
// still SAFE_OFF_REQUIRED, never a silent "did nothing".
// ---------------------------------------------------------------------------

void test_permit_revoked_between_steps_still_requires_safe_off() {
  g_case = "permit revoked mid-attempt -> SAFE_OFF_REQUIRED";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(), ctx, 1000));
  rig.executor.update(ctx, 1010, false, TelemetrySample{});  // torque enable: applied
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::POSITION_COMMAND_PENDING);

  // The permit is withdrawn (e.g. session ended) before the second step.
  rig.policy.setBootstrapContext(CalibrationBootstrapContext{});
  rig.executor.update(ctx, 1020, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure,
          (int)FirstMotionFailure::POSITION_COMMAND_REJECTED);
  CHECK_EQ(rig.backend.write_goal_position_calls, 0);  // policy refused before any backend call
}

// ---------------------------------------------------------------------------
// Monitoring escalation: stall / timeout / communication loss all require
// SAFE_OFF, distinctly reported.
// ---------------------------------------------------------------------------

void reachMonitoring(Rig& rig, const FirstMotionContext& ctx) {
  rig.executor.update(ctx, 1010, false, TelemetrySample{});
  rig.executor.update(ctx, 1020, false, TelemetrySample{});
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::MONITORING);
}

void test_stall_during_monitoring_requires_safe_off() {
  g_case = "stall -> SAFE_OFF_REQUIRED";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);

  rig.executor.update(ctx, 1100, true, telemetry(2000, 1, 1100));
  rig.executor.update(ctx, 3100, true, telemetry(2000, 1, 3100));  // no progress for 2000ms
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::STALLED);
}

void test_timeout_during_monitoring_requires_safe_off() {
  g_case = "timeout -> SAFE_OFF_REQUIRED";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);  // motion started at 1020

  rig.executor.update(ctx, 13100, true, telemetry(2050, 1, 13100));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::MOTION_TIMEOUT);
}

void test_communication_lost_during_monitoring_requires_safe_off() {
  g_case = "comm lost -> SAFE_OFF_REQUIRED";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);

  TelemetrySample failed{};
  failed.read_ok = false;
  failed.sampled_at_ms = 4100;
  rig.executor.update(ctx, 4100, true, failed);
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::COMMUNICATION_LOST);
}

void test_false_positive_progress_keeps_continuing() {
  // A normal-looking, slowly-but-genuinely-progressing move must NOT be
  // reported as stalled or otherwise aborted - the false-positive case CR3
  // explicitly asks to be covered.
  g_case = "genuine slow progress is not a false stall";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);

  rig.executor.update(ctx, 1500, true, telemetry(2010, 1, 1500));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::MONITORING);
  rig.executor.update(ctx, 2900, true, telemetry(2040, 1, 2900));  // progress resets the window
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::MONITORING);
  rig.executor.update(ctx, 3200, true, telemetry(2078, 1, 3200));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::COMPLETE);
}

// ---------------------------------------------------------------------------
// Dynamic prerequisite loss while MONITORING must not wait for deadman/
// timeout. Torque is already on, therefore every such loss requires SAFE_OFF.
// ---------------------------------------------------------------------------

void test_permit_loss_during_monitoring_requires_safe_off_immediately() {
  g_case = "permit loss while monitoring";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
            OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  FirstMotionContext ctx =
      liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);

  ctx.motion_permit_active = false;
  rig.executor.update(ctx, 1030, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state,
           (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure,
           (int)FirstMotionFailure::DYNAMIC_PREREQUISITE_LOST);
}

void test_authority_loss_during_monitoring_requires_safe_off_immediately() {
  g_case = "authority loss while monitoring";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
            OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  FirstMotionContext ctx =
      liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);

  ctx.authority = ActuatorAuthority::NONE;
  ctx.authority_generation = lease.generation + 1;
  rig.executor.update(ctx, 1030, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state,
           (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure,
           (int)FirstMotionFailure::DYNAMIC_PREREQUISITE_LOST);
}

void test_session_loss_before_torque_needs_no_safe_off() {
  g_case = "session loss before torque";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION,
            OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  FirstMotionContext ctx =
      liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(), ctx, 1000));

  ctx.session_active = false;
  rig.executor.update(ctx, 1001, false, TelemetrySample{});

  CHECK_EQ((int)rig.executor.status().state,
           (int)FirstMotionState::FAILED_NO_MOTION);
  CHECK_EQ((int)rig.executor.status().failure,
           (int)FirstMotionFailure::DYNAMIC_PREREQUISITE_LOST);
  CHECK_EQ(rig.backend.totalCalls(), 0);
}

// ---------------------------------------------------------------------------
// abort() safety: routes to SAFE_OFF_REQUIRED iff torque was already on.
// ---------------------------------------------------------------------------

void test_abort_before_torque_confirmed_needs_no_safe_off() {
  g_case = "abort before torque on";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::TORQUE_ENABLE_PENDING);

  rig.executor.abort();
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::FAILED_NO_MOTION);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::OPERATOR_ABORT);
  CHECK_EQ(rig.backend.totalCalls(), 0);
}

void test_abort_after_torque_confirmed_requires_safe_off() {
  g_case = "abort after torque on";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  rig.executor.update(ctx, 1010, false, TelemetrySample{});
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::POSITION_COMMAND_PENDING);

  rig.executor.abort();
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::OPERATOR_ABORT);
}

void test_abort_during_monitoring_requires_safe_off() {
  g_case = "abort during monitoring";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  CHECK(rig.executor.start(request(), ctx, 1000));
  reachMonitoring(rig, ctx);

  rig.executor.abort();
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.executor.status().failure, (int)FirstMotionFailure::OPERATOR_ABORT);
}

// ---------------------------------------------------------------------------
// A second start() while one is outstanding is refused.
// ---------------------------------------------------------------------------

void test_second_start_while_active_is_refused() {
  g_case = "second start refused while active";
  Rig rig;
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.executor.start(request(), ctx, 1000));
  CHECK(!rig.executor.start(request(), ctx, 1001));
  // The original attempt's state is undisturbed.
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::TORQUE_ENABLE_PENDING);

  // Once terminal, a new start() is accepted again.
  rig.executor.abort();
  CHECK(rig.executor.start(request(), ctx, 1002));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::TORQUE_ENABLE_PENDING);
}

void test_zero_delta_and_no_transform_refused_at_start() {
  g_case = "malformed request refused at start";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  armCalibrationPermit(rig.policy, lease);
  const FirstMotionContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(!rig.executor.start(request(/*delta_ticks=*/0), ctx, 1000));
  CHECK_EQ((int)rig.executor.status().state, (int)FirstMotionState::FAILED_NO_MOTION);
}

}  // namespace

int main() {
  test_happy_path_one_backend_call_per_tick_then_arrives();
  test_no_permit_rejects_before_any_backend_call();
  test_torque_enable_verified_rejected_is_failed_no_motion();
  test_torque_enable_uncertain_requires_safe_off();
  test_position_command_uncertain_after_torque_on_requires_safe_off();
  test_permit_revoked_between_steps_still_requires_safe_off();
  test_stall_during_monitoring_requires_safe_off();
  test_timeout_during_monitoring_requires_safe_off();
  test_communication_lost_during_monitoring_requires_safe_off();
  test_false_positive_progress_keeps_continuing();
  test_permit_loss_during_monitoring_requires_safe_off_immediately();
  test_authority_loss_during_monitoring_requires_safe_off_immediately();
  test_session_loss_before_torque_needs_no_safe_off();
  test_abort_before_torque_confirmed_needs_no_safe_off();
  test_abort_after_torque_confirmed_requires_safe_off();
  test_abort_during_monitoring_requires_safe_off();
  test_second_start_while_active_is_refused();
  test_zero_delta_and_no_transform_refused_at_start();

  std::printf("test_first_motion_executor: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
