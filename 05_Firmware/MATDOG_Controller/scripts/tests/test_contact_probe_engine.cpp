// Offline adversarial tests for the CR3 Priority 5 contact-probe engine
// (src/calibration/ContactProbeEngine.*).
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, CalibrationExecutionEngine,
// ActuatorAuthorityArbiter and checked target resolver against a fake backend
// and synthetic telemetry - the same contract as
// test_calibration_execution_engine.cpp and test_first_motion_executor.cpp.
//
// NO HARDWARE VALIDATION, and no fabricated physical measurement: every
// "contact" in this file is a synthetic stall this test constructs, never
// presented as a real UPPER endpoint result.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/calibration/ContactProbeEngine.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::ActuatorRuntime;
using matdog::actuator::BackendWriteOutcome;
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

// LF UPPER, MIN_SIDE - one of the 8 real EXECUTABLE_URDF_DOMAIN endpoints
// (verified against src/actuator/CalibrationGeometryProfileData.h): unit
// "ELR01", bus 12, urdf_motor_direction=1, contact=-909889 urad.
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

// Resolves the SAME way the engine under test does, so this file never
// hand-computes tick arithmetic that could silently drift from the real
// resolver's rounding.
uint16_t resolvedTick(actuator::MicroRad target_urad, uint16_t q0 = 2048) {
  CalibrationGeometryProfile profile = boundProfile();
  const actuator::JointTransform transform = promotedTransform(lfUpper(), q0);
  uint16_t tick = 0;
  const actuator::TargetResolveStatus status = actuator::resolveUrdfQToRaw(
      profile, actuator::geometry_data::kProvenance, transform, target_urad, &tick);
  CHECK(status == actuator::TargetResolveStatus::OK);
  return tick;
}

AuthorityLease grant(ActuatorAuthorityArbiter& arbiter, ActuatorAuthority owner,
                     OperatingMode mode) {
  AuthorityLease lease{};
  const AuthorityResult result = arbiter.request(owner, mode, &lease);
  CHECK(result == AuthorityResult::GRANTED);
  return lease;
}

void armCalibrationPermit(SafeActuatorPolicy& policy, const AuthorityLease& lease) {
  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  bootstrap.motion_permit_active = true;
  bootstrap.motion_permit_generation = 1;
  bootstrap.motion_permit_session_id = 1;
  bootstrap.motion_permit_authority_generation = lease.generation;
  policy.setBootstrapContext(bootstrap);
}

ContactProbeContext liveContext(const AuthorityLease& lease, OperatingMode mode) {
  ContactProbeContext ctx{};
  ctx.session_active = true;
  ctx.origin = CalibrationOrigin::LIVE_SESSION;
  ctx.lease = lease;
  ctx.mode = mode;
  return ctx;
}

MotionDeadmanConfig approachDeadman() {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 15000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = 4;
  return c;
}

MotionDeadmanConfig backoffDeadman() {
  return approachDeadman();
}

TelemetrySample telemetry(int32_t position, int32_t torque_enable, uint32_t at_ms) {
  TelemetrySample s{};
  s.read_ok = true;
  s.sampled_at_ms = at_ms;
  s.present_position = position;
  s.torque_enable = torque_enable;
  return s;
}

class FakeActuatorBackend : public actuator::ActuatorBackend {
 public:
  BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    ++enable_torque_calls;
    last_bus_id = bus_id;
    return enable_torque_result;
  }
  BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick) override {
    ++write_goal_position_calls;
    last_bus_id = bus_id;
    last_target_tick = target_tick;
    return write_goal_position_result;
  }

  int enable_torque_calls = 0;
  int write_goal_position_calls = 0;
  uint8_t last_bus_id = 0;
  uint16_t last_target_tick = 0;
  BackendWriteOutcome enable_torque_result = BackendWriteOutcome::VERIFIED_APPLIED;
  BackendWriteOutcome write_goal_position_result = BackendWriteOutcome::VERIFIED_APPLIED;
};

struct Rig {
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  ActuatorRuntime runtime;
  CalibrationExecutionEngine engine;
  FakeActuatorBackend backend;
  CalibrationGeometryProfile profile;
  ContactProbeEngine probe;

  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance);
    ContactProbeConfig config{};
    config.approach_deadman = approachDeadman();
    config.backoff_deadman = backoffDeadman();
    probe.begin(&policy, &runtime, &engine, &profile, &actuator::geometry_data::kProvenance,
               config);
  }
};

// contact = -909889 urad exactly (the compiled endpoint value); backoff is a
// deliberately-clear, strictly-less-extreme point on the same (MIN_SIDE)
// side, both explicit request fields per the file comment - this engine
// invents neither.
ContactProbeRequest request(uint8_t bus_id = 12) {
  ContactProbeRequest r{};
  r.joint = lfUpper();
  r.bus_id = bus_id;
  r.endpoint_leg = Leg::LF;
  r.endpoint_joint = JointKind::UPPER;
  r.endpoint_side = ContactSide::MIN_SIDE;
  r.approach_target_urad = -909889;
  r.backoff_target_urad = -700000;
  r.repeatability_tolerance_ticks = 16;  // documented historical LF V25 band,
                                         // supplied explicitly, never defaulted
  return r;
}

void primeRig(Rig& rig, const AuthorityLease& lease) {
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  armCalibrationPermit(rig.policy, lease);
}

// ---------------------------------------------------------------------------
// Full two-pass happy path: approach 1 (stall) -> backoff -> approach 2
// (consistent stall) -> COMPLETE with a valid witness.
// ---------------------------------------------------------------------------

void test_full_two_pass_happy_path_produces_accepted_witness() {
  g_case = "two-pass happy path";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  const uint16_t approach_target = resolvedTick(request().approach_target_urad);
  const uint16_t backoff_target = resolvedTick(request().backoff_target_urad);
  const uint16_t stall_at = approach_target + 20;  // short of full arrival

  CHECK(rig.probe.start(request(), ctx, 1000));

  // TorqueEnable.
  rig.probe.update(ctx, 1010, false, TelemetrySample{});
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::APPROACH_PENDING);

  // Approach 1 command.
  rig.probe.update(ctx, 1020, false, TelemetrySample{});
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::APPROACH_MONITORING);
  CHECK_EQ(rig.backend.last_target_tick, approach_target);

  // Approach 1 stalls short of the boundary - possible contact.
  rig.probe.update(ctx, 1500, true, telemetry(stall_at, 1, 1500));
  rig.probe.update(ctx, 3500, true, telemetry(stall_at, 1, 3500));  // 2000ms, no progress
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::BACKOFF_PENDING);
  CHECK_EQ(rig.probe.status().pass, 2);
  CHECK_EQ(rig.probe.status().coarse_tick, stall_at);

  // Backoff command + arrival.
  rig.probe.update(ctx, 3510, false, TelemetrySample{});
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::BACKOFF_MONITORING);
  CHECK_EQ(rig.backend.last_target_tick, backoff_target);
  rig.probe.update(ctx, 3600, true, telemetry(backoff_target, 1, 3600));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::APPROACH_PENDING);

  // Approach 2 command.
  rig.probe.update(ctx, 3610, false, TelemetrySample{});
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::APPROACH_MONITORING);
  CHECK_EQ(rig.backend.last_target_tick, approach_target);

  // Approach 2 stalls within tolerance (16 ticks) of the first stall.
  const uint16_t second_stall = stall_at + 6;
  rig.probe.update(ctx, 4000, true, telemetry(second_stall, 1, 4000));
  rig.probe.update(ctx, 6000, true, telemetry(second_stall, 1, 6000));

  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::NONE);
  CHECK_EQ(rig.probe.status().fine_tick_1, second_stall);
  const ContactWitness w = rig.probe.witness();
  CHECK(w.evaluated);
  CHECK(w.tolerance_set);
  CHECK_EQ(w.min_deviation_ticks, 6);
  CHECK_EQ(w.max_deviation_ticks, 6);
  CHECK(w.accepted());
  CHECK(!rig.probe.active());
}

// ---------------------------------------------------------------------------
// Repeatability failure: second approach stalls, but far from the first.
// ---------------------------------------------------------------------------

void test_repeatability_failure_rejects_rather_than_records() {
  g_case = "repeatability failure";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const uint16_t approach_target = resolvedTick(request().approach_target_urad);
  const uint16_t backoff_target = resolvedTick(request().backoff_target_urad);
  const uint16_t stall_at = approach_target + 20;

  CHECK(rig.probe.start(request(), ctx, 1000));
  rig.probe.update(ctx, 1010, false, TelemetrySample{});
  rig.probe.update(ctx, 1020, false, TelemetrySample{});
  rig.probe.update(ctx, 1500, true, telemetry(stall_at, 1, 1500));
  rig.probe.update(ctx, 3500, true, telemetry(stall_at, 1, 3500));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::BACKOFF_PENDING);
  rig.probe.update(ctx, 3510, false, TelemetrySample{});
  rig.probe.update(ctx, 3600, true, telemetry(backoff_target, 1, 3600));
  rig.probe.update(ctx, 3610, false, TelemetrySample{});

  // Second stall is 40 ticks away - outside the 16-tick tolerance.
  const uint16_t second_stall = stall_at + 40;
  rig.probe.update(ctx, 4000, true, telemetry(second_stall, 1, 4000));
  rig.probe.update(ctx, 6000, true, telemetry(second_stall, 1, 6000));

  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::REPEATABILITY_FAILED);
}

// ---------------------------------------------------------------------------
// False positive resolved by the second pass: first pass "stalls" (noise),
// second pass arrives cleanly - no contact is recorded.
// ---------------------------------------------------------------------------

void test_arrival_without_any_stall_is_not_contact() {
  g_case = "arrival without stall is not contact";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const uint16_t approach_target = resolvedTick(request().approach_target_urad);

  CHECK(rig.probe.start(request(), ctx, 1000));
  rig.probe.update(ctx, 1010, false, TelemetrySample{});
  rig.probe.update(ctx, 1020, false, TelemetrySample{});

  // Progresses all the way to the commanded boundary with no stall.
  rig.probe.update(ctx, 1200, true, telemetry(approach_target + 100, 1, 1200));
  rig.probe.update(ctx, 1400, true, telemetry(approach_target, 1, 1400));

  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::NO_CONTACT_DETECTED);
}

// ---------------------------------------------------------------------------
// Communication dropout / stale telemetry / timeout during approach.
// ---------------------------------------------------------------------------

void reachApproachMonitoring(Rig& rig, const ContactProbeContext& ctx) {
  CHECK(rig.probe.start(request(), ctx, 1000));
  rig.probe.update(ctx, 1010, false, TelemetrySample{});
  rig.probe.update(ctx, 1020, false, TelemetrySample{});
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::APPROACH_MONITORING);
}

void test_communication_dropout_during_approach_requires_safe_off() {
  g_case = "comm dropout during approach";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  reachApproachMonitoring(rig, ctx);

  TelemetrySample failed{};
  failed.read_ok = false;
  failed.sampled_at_ms = 4100;
  rig.probe.update(ctx, 4100, true, failed);
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::COMMUNICATION_LOST);
}

void test_stale_telemetry_during_approach_requires_safe_off() {
  g_case = "stale telemetry during approach";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  reachApproachMonitoring(rig, ctx);

  rig.probe.update(ctx, 4100, false, TelemetrySample{});  // no attempt at all
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::STALE_TELEMETRY);
}

void test_timeout_during_approach_requires_safe_off() {
  g_case = "timeout during approach";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  reachApproachMonitoring(rig, ctx);  // approach started at 1020, budget 15000ms

  rig.probe.update(ctx, 16100, true, telemetry(1800, 1, 16100));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::MOTION_TIMEOUT);
}

// ---------------------------------------------------------------------------
// Travel guard: the policy itself refuses a target beyond the endpoint's
// geometric contact - this engine must surface that as SAFE_OFF_REQUIRED,
// never silently proceed.
// ---------------------------------------------------------------------------

void test_travel_guard_beyond_contact_is_refused_by_policy() {
  g_case = "travel guard rejects beyond-contact target";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  // Between urdf_lower (-916298, so this still resolves to a raw tick) and
  // contact (-909889, so it is travelling INTO the mechanism past the
  // endpoint's own boundary) - isolates the POLICY's contact-side check
  // from the checked resolver's own, more fundamental URDF-limit check.
  ContactProbeRequest r = request();
  r.approach_target_urad = -912000;
  CHECK(rig.probe.start(r, ctx, 1000));
  rig.probe.update(ctx, 1010, false, TelemetrySample{});  // torque enable
  rig.probe.update(ctx, 1020, false, TelemetrySample{});  // approach command refused

  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::COMMAND_REJECTED);
  CHECK_EQ(rig.backend.write_goal_position_calls, 0);  // never reached the backend
}

// ---------------------------------------------------------------------------
// Unexpected stall during backoff is a real anomaly, not evidence.
// ---------------------------------------------------------------------------

void test_unexpected_stall_during_backoff_requires_safe_off() {
  g_case = "unexpected stall during backoff";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const uint16_t approach_target = resolvedTick(request().approach_target_urad);
  const uint16_t stall_at = approach_target + 20;

  CHECK(rig.probe.start(request(), ctx, 1000));
  rig.probe.update(ctx, 1010, false, TelemetrySample{});
  rig.probe.update(ctx, 1020, false, TelemetrySample{});
  rig.probe.update(ctx, 1500, true, telemetry(stall_at, 1, 1500));
  rig.probe.update(ctx, 3500, true, telemetry(stall_at, 1, 3500));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::BACKOFF_PENDING);
  rig.probe.update(ctx, 3510, false, TelemetrySample{});
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::BACKOFF_MONITORING);

  // Backoff itself stalls - never expected on an already-proven-clear corridor.
  rig.probe.update(ctx, 4000, true, telemetry(stall_at + 3, 1, 4000));
  rig.probe.update(ctx, 6000, true, telemetry(stall_at + 3, 1, 6000));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure,
          (int)ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);
}

// ---------------------------------------------------------------------------
// Authority/permit loss mid-attempt.
// ---------------------------------------------------------------------------

void test_permit_loss_before_approach_requires_safe_off() {
  g_case = "permit loss before approach command";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.probe.start(request(), ctx, 1000));
  rig.probe.update(ctx, 1010, false, TelemetrySample{});  // torque enable: applied
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::APPROACH_PENDING);

  rig.policy.setBootstrapContext(actuator::CalibrationBootstrapContext{});  // permit withdrawn
  rig.probe.update(ctx, 1020, false, TelemetrySample{});

  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::COMMAND_REJECTED);
}

void test_authority_loss_before_torque_enable_is_failed_no_motion() {
  g_case = "authority loss before torque enable";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.probe.start(request(), ctx, 1000));
  CHECK(rig.arbiter.release(lease) == AuthorityResult::RELEASED);  // authority lost
  rig.probe.update(ctx, 1010, false, TelemetrySample{});

  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::FAILED_NO_MOTION);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::REJECT_PRECONDITIONS);
  CHECK_EQ(rig.backend.enable_torque_calls, 0);
}

// ---------------------------------------------------------------------------
// Operator abort at each phase.
// ---------------------------------------------------------------------------

void test_abort_before_torque_confirmed_needs_no_safe_off() {
  g_case = "abort before torque on";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.probe.start(request(), ctx, 1000));
  rig.probe.abort();
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::FAILED_NO_MOTION);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::OPERATOR_ABORT);
}

void test_abort_during_approach_monitoring_requires_safe_off() {
  g_case = "abort during approach monitoring";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  reachApproachMonitoring(rig, ctx);

  rig.probe.abort();
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::OPERATOR_ABORT);
}

// ---------------------------------------------------------------------------
// Malformed request / already-active refusal.
// ---------------------------------------------------------------------------

void test_zero_tolerance_refused_at_start() {
  g_case = "zero tolerance refused";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  ContactProbeRequest r = request();
  r.repeatability_tolerance_ticks = 0;
  CHECK(!rig.probe.start(r, ctx, 1000));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::FAILED_NO_MOTION);
}

void test_second_start_while_active_is_refused() {
  g_case = "second start refused while active";
  Rig rig;
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  primeRig(rig, lease);
  const ContactProbeContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.probe.start(request(), ctx, 1000));
  CHECK(!rig.probe.start(request(), ctx, 1001));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::TORQUE_ENABLE_PENDING);
}

void test_witness_empty_before_complete() {
  g_case = "witness empty before complete";
  Rig rig;
  CHECK(!rig.probe.witness().evaluated);
}

}  // namespace

int main() {
  test_full_two_pass_happy_path_produces_accepted_witness();
  test_repeatability_failure_rejects_rather_than_records();
  test_arrival_without_any_stall_is_not_contact();
  test_communication_dropout_during_approach_requires_safe_off();
  test_stale_telemetry_during_approach_requires_safe_off();
  test_timeout_during_approach_requires_safe_off();
  test_travel_guard_beyond_contact_is_refused_by_policy();
  test_unexpected_stall_during_backoff_requires_safe_off();
  test_permit_loss_before_approach_requires_safe_off();
  test_authority_loss_before_torque_enable_is_failed_no_motion();
  test_abort_before_torque_confirmed_needs_no_safe_off();
  test_abort_during_approach_monitoring_requires_safe_off();
  test_zero_tolerance_refused_at_start();
  test_second_start_while_active_is_refused();
  test_witness_empty_before_complete();

  std::printf("test_contact_probe_engine: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
