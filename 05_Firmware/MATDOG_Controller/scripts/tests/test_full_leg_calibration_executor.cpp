// Offline adversarial tests for the CR3 continuation Full Leg Calibration
// sequencer (src/calibration/FullLegCalibrationExecutor.*).
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, CalibrationExecutionEngine,
// ContactProbeEngine, ActuatorAuthorityArbiter and checked target resolver
// against a fake backend and synthetic telemetry - the same contract as
// test_contact_probe_engine.cpp and test_first_motion_executor.cpp.
//
// NO HARDWARE VALIDATION, and no fabricated physical measurement: every
// "contact" and every "arrival" in this file is a synthetic value this test
// constructs, never presented as a real endpoint result.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/OperationalEnvelope.h"
#include "../../src/calibration/FullLegCalibrationExecutor.h"

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

// LF UPPER (bus 12) and LH UPPER (bus 42) - the probed joint and the ONE
// named auxiliary the compiled LF_UPPER:MAX plan requires parked, verified
// against src/actuator/CalibrationGeometryProfileData.h.
JointIdentity lfUpper() {
  JointIdentity id{};
  id.leg = Leg::LF;
  id.joint = JointKind::UPPER;
  setPhysicalUnit(&id, "ELR01");
  return id;
}

JointIdentity lhUpper() {
  JointIdentity id{};
  id.leg = Leg::LH;
  id.joint = JointKind::UPPER;
  setPhysicalUnit(&id, "M42");
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

uint16_t resolvedTick(actuator::MicroRad target_urad, JointIdentity id, uint16_t q0 = 2048) {
  CalibrationGeometryProfile profile = boundProfile();
  const actuator::JointTransform transform = promotedTransform(id, q0);
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

TelemetrySample telemetry(int32_t position, int32_t torque_enable, uint32_t at_ms) {
  TelemetrySample s{};
  s.read_ok = true;
  s.sampled_at_ms = at_ms;
  s.present_position = position;
  s.torque_enable = torque_enable;
  return s;
}

MotionDeadmanConfig probeDeadman() {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 15000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = 4;
  return c;
}

MotionDeadmanConfig auxMoveDeadman() {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = 4;
  return c;
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
  FullLegCalibrationExecutor full;

  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance);
    FullLegCalibrationConfig config{};
    config.probe_approach_deadman = probeDeadman();
    config.probe_backoff_deadman = probeDeadman();
    config.aux_move_deadman = auxMoveDeadman();
    full.begin(&policy, &runtime, &engine, &profile, &actuator::geometry_data::kProvenance, config);
  }
};

// Both transforms a full leg run needs on the current installation: the
// probed joint AND the one named auxiliary.
void primeTransforms(Rig& rig) {
  CHECK(rig.policy.transforms().admit(promotedTransform(lfUpper())));
  CHECK(rig.policy.transforms().admit(promotedTransform(lhUpper())));
}

// Mirrors what Controller's own per-tick bootstrap-context refresh does in
// production: rebuilds the ENTIRE CalibrationBootstrapContext from live
// state on every call, including the auxiliary-parked fields, which track
// full.auxiliaryParked() exactly the way Controller reads it from
// full_leg_calibration_.auxiliaryParked() every tick.
void refreshBootstrap(Rig& rig, const AuthorityLease& lease, bool auxiliary_parked) {
  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  bootstrap.motion_permit_active = true;
  bootstrap.motion_permit_generation = 1;
  bootstrap.motion_permit_session_id = 1;
  bootstrap.motion_permit_authority_generation = lease.generation;
  bootstrap.auxiliary_parked = auxiliary_parked;
  bootstrap.parked_leg = Leg::LF;
  bootstrap.parked_joint = JointKind::UPPER;
  bootstrap.parked_side = ContactSide::MAX_SIDE;
  rig.policy.setBootstrapContext(bootstrap);
}

FullLegCalibrationContext liveContext(const AuthorityLease& lease, OperatingMode mode) {
  FullLegCalibrationContext ctx{};
  ctx.session_active = true;
  ctx.origin = CalibrationOrigin::LIVE_SESSION;
  ctx.lease = lease;
  ctx.mode = mode;
  ctx.motion_permit_active = true;
  ctx.authority = ActuatorAuthority::CALIBRATION;
  ctx.authority_generation = lease.generation;
  ctx.authority_inhibited = false;
  return ctx;
}

// The exact CR3 LF_UPPER values: MIN/MAX contact and clear targets from the
// compiled Geometry V5 bundle, and the ONE named auxiliary parked pose
// (610865 urad, LH_UPPER) the LF_UPPER:MAX endpoint plan requires exactly.
FullLegCalibrationRequest request() {
  FullLegCalibrationRequest r{};
  r.probe_joint = lfUpper();
  r.probe_bus_id = 12;
  r.endpoint_leg = Leg::LF;
  r.endpoint_joint = JointKind::UPPER;
  r.min_approach_urad = -909889;
  r.min_backoff_urad = -700000;
  r.min_repeatability_tolerance_ticks = 16;
  r.max_approach_urad = 2127120;
  r.max_backoff_urad = 1500000;
  r.max_repeatability_tolerance_ticks = 16;
  r.auxiliary_joint = lhUpper();
  r.auxiliary_bus_id = 42;
  r.auxiliary_park_target_urad = 610865;
  return r;
}

// Drives start() through the MIN-side two-pass probe to its COMPLETE
// terminal (UPPER_MIN_SAFE_OFF), then services that SAFE_OFF so the caller
// lands at AUX_TORQUE_ENABLE - the common prefix every AUX/MAX-phase test
// below needs, factored out once rather than repeated per test.
void driveMinProbeAndSafeOff(Rig& rig, const FullLegCalibrationContext& ctx, uint32_t* t) {
  const FullLegCalibrationRequest req = request();
  const uint16_t min_approach = resolvedTick(req.min_approach_urad, lfUpper());
  const uint16_t min_backoff = resolvedTick(req.min_backoff_urad, lfUpper());

  CHECK(rig.full.start(req, ctx, *t));
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // torque enable
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // approach1 command
  CHECK_EQ(rig.backend.last_target_tick, min_approach);
  const uint16_t stall1 = min_approach + 20;
  *t = 1500; rig.full.update(ctx, *t, true, telemetry(stall1, 1, *t), false, false);
  *t = 3500; rig.full.update(ctx, *t, true, telemetry(stall1, 1, *t), false, false);  // -> backoff
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // backoff command
  CHECK_EQ(rig.backend.last_target_tick, min_backoff);
  *t += 90; rig.full.update(ctx, *t, true, telemetry(min_backoff, 1, *t), false, false);  // arrival -> approach2
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // approach2 command
  const uint16_t stall2 = stall1 + 6;
  *t += 490; rig.full.update(ctx, *t, true, telemetry(stall2, 1, *t), false, false);
  *t += 2000; rig.full.update(ctx, *t, true, telemetry(stall2, 1, *t), false, false);  // -> COMPLETE

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::NONE);

  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, /*primary=*/true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_TORQUE_ENABLE);
}

// ---------------------------------------------------------------------------
// Full happy path: MIN probe -> SAFE_OFF -> aux park -> MAX probe ->
// dual SAFE_OFF -> COMPLETE, with witnessed evidence on both sides.
// ---------------------------------------------------------------------------

void test_happy_path_full_leg_completes_with_evidence_both_sides() {
  g_case = "full leg happy path";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, /*auxiliary_parked=*/false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);
  CHECK(!rig.full.minSideEvidence().has_measurement == false);  // sanity: has_measurement true
  CHECK(rig.full.minSideEvidence().has_measurement);
  CHECK(rig.full.minSideEvidence().witness.accepted());

  const FullLegCalibrationRequest req = request();
  const uint16_t aux_target = resolvedTick(req.auxiliary_park_target_urad, lhUpper());
  const uint16_t max_approach = resolvedTick(req.max_approach_urad, lfUpper());
  const uint16_t max_backoff = resolvedTick(req.max_backoff_urad, lfUpper());

  refreshBootstrap(rig, lease, /*auxiliary_parked=*/false);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux torque enable
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_MOVE_PENDING);
  CHECK_EQ(rig.backend.last_bus_id, 42);

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux move command
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_MOVE_MONITORING);
  CHECK_EQ(rig.backend.last_bus_id, 42);
  CHECK_EQ(rig.backend.last_target_tick, aux_target);

  t += 200; rig.full.update(ctx, t, true, telemetry(aux_target, 1, t), false, false);  // arrival
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);
  CHECK(rig.full.auxiliaryParked());

  refreshBootstrap(rig, lease, /*auxiliary_parked=*/true);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // MAX torque enable (bus 12)
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // MAX approach1 command
  CHECK_EQ(rig.backend.last_bus_id, 12);
  CHECK_EQ(rig.backend.last_target_tick, max_approach);
  const uint16_t max_stall1 = max_approach - 20;
  t += 500; rig.full.update(ctx, t, true, telemetry(max_stall1, 1, t), false, false);
  t += 2000; rig.full.update(ctx, t, true, telemetry(max_stall1, 1, t), false, false);  // -> backoff
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // backoff command
  CHECK_EQ(rig.backend.last_target_tick, max_backoff);
  t += 90; rig.full.update(ctx, t, true, telemetry(max_backoff, 1, t), false, false);  // arrival -> approach2
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // approach2 command
  const uint16_t max_stall2 = max_stall1 - 6;
  t += 490; rig.full.update(ctx, t, true, telemetry(max_stall2, 1, t), false, false);
  t += 2000; rig.full.update(ctx, t, true, telemetry(max_stall2, 1, t), false, false);  // -> probe COMPLETE

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::NONE);
  CHECK(rig.full.primarySafeOffPending());
  CHECK(rig.full.auxiliarySafeOffPending());
  CHECK(!rig.full.auxiliaryParked());  // no longer probing; SAFE_OFF doesn't consult it anyway

  // Only ONE of the two verified: must not complete yet.
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, /*primary=*/true, /*aux=*/false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, /*primary=*/true, /*aux=*/true);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::COMPLETE);
  CHECK(!rig.full.active());

  CHECK(rig.full.maxSideEvidence().has_measurement);
  CHECK(rig.full.maxSideEvidence().witness.accepted());

  // The two witnessed sides are exactly what buildContactDerivedEnvelope()
  // needs - confirm they actually build a present, ordered UPPER envelope.
  actuator::ContactDerivedEnvelopeRequest env_req{};
  env_req.joint = lfUpper();
  env_req.min_side_evidence = rig.full.minSideEvidence();
  env_req.max_side_evidence = rig.full.maxSideEvidence();
  env_req.min_side_geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
  env_req.max_side_geometry = env_req.min_side_geometry;
  env_req.safety_margin_ticks = 8;
  actuator::OperationalEnvelope envelope{};
  const actuator::EnvelopeBuildStatus env_status = actuator::buildContactDerivedEnvelope(
      rig.profile, actuator::geometry_data::kProvenance, env_req, &envelope);
  CHECK(env_status == actuator::EnvelopeBuildStatus::READY);
  CHECK(envelope.present);
  CHECK(envelope.ordered());
}

// ---------------------------------------------------------------------------
// MIN probe fails (arrival without any stall = no contact detected): must
// SAFE_OFF bus 12 and terminate FAILED without ever touching the auxiliary.
// ---------------------------------------------------------------------------

void test_min_probe_failure_never_reaches_aux_or_max() {
  g_case = "MIN probe failure stops before AUX";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const FullLegCalibrationRequest req = request();
  const uint16_t min_approach = resolvedTick(req.min_approach_urad, lfUpper());

  uint32_t t = 1000;
  CHECK(rig.full.start(req, ctx, t));
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // approach1 command
  // Arrives cleanly at the boundary with no stall ever observed.
  t += 300; rig.full.update(ctx, t, true, telemetry(min_approach, 1, t), false, false);

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::UPPER_MIN_PROBE_FAILED);
  CHECK(rig.full.primarySafeOffPending());
  CHECK(!rig.full.auxiliarySafeOffPending());
  CHECK_EQ(rig.backend.enable_torque_calls, 1);  // never reached AUX_TORQUE_ENABLE

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, /*primary=*/true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
  CHECK_EQ(rig.backend.enable_torque_calls, 1);  // still just the one, on bus 12
  CHECK(!rig.full.minSideEvidence().has_measurement);
}

// ---------------------------------------------------------------------------
// Auxiliary TorqueEnable comes back UNCERTAIN: dual SAFE_OFF, then FAILED.
// ---------------------------------------------------------------------------

void test_aux_torque_enable_uncertain_requires_dual_safe_off_and_fails() {
  g_case = "aux torque-enable uncertain";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);

  rig.backend.enable_torque_result = BackendWriteOutcome::UNCERTAIN;
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure,
          (int)FullLegCalibrationFailure::AUX_TORQUE_ENABLE_UNCERTAIN);
  CHECK(rig.full.primarySafeOffPending());
  CHECK(rig.full.auxiliarySafeOffPending());

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);  // aux still pending
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, true);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
}

// ---------------------------------------------------------------------------
// Auxiliary parking stalls before arriving: dual SAFE_OFF, then FAILED.
// ---------------------------------------------------------------------------

void test_aux_move_stalled_requires_dual_safe_off_and_fails() {
  g_case = "aux move stalled";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux move command
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_MOVE_MONITORING);

  const uint16_t stuck = rig.backend.last_target_tick > 100 ? rig.backend.last_target_tick - 100
                                                             : rig.backend.last_target_tick + 100;
  t += 500; rig.full.update(ctx, t, true, telemetry(stuck, 1, t), false, false);
  t += 2000; rig.full.update(ctx, t, true, telemetry(stuck, 1, t), false, false);  // no progress -> STALLED

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::AUX_MOVE_STALLED);

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, true);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
}

// ---------------------------------------------------------------------------
// MAX probe itself fails after a successful park (repeatability mismatch):
// dual SAFE_OFF, then FAILED - the parked auxiliary is never left energised.
// ---------------------------------------------------------------------------

void test_max_probe_failure_after_park_requires_dual_safe_off_and_fails() {
  g_case = "MAX probe repeatability failure after park";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const FullLegCalibrationRequest req = request();
  const uint16_t aux_target = resolvedTick(req.auxiliary_park_target_urad, lhUpper());
  const uint16_t max_approach = resolvedTick(req.max_approach_urad, lfUpper());

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux move command
  t += 200; rig.full.update(ctx, t, true, telemetry(aux_target, 1, t), false, false);  // arrival
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);

  refreshBootstrap(rig, lease, true);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // MAX torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // MAX approach1 command
  const uint16_t stall1 = max_approach - 20;
  t += 500; rig.full.update(ctx, t, true, telemetry(stall1, 1, t), false, false);
  t += 2000; rig.full.update(ctx, t, true, telemetry(stall1, 1, t), false, false);  // -> backoff
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // backoff command
  t += 90; rig.full.update(ctx, t, true, telemetry(rig.backend.last_target_tick, 1, t), false, false);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // approach2 command
  const uint16_t stall2 = stall1 - 200;  // far outside the 16-tick tolerance
  t += 500; rig.full.update(ctx, t, true, telemetry(stall2, 1, t), false, false);
  t += 2000; rig.full.update(ctx, t, true, telemetry(stall2, 1, t), false, false);  // -> repeatability failure

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED);
  CHECK(rig.full.primarySafeOffPending());
  CHECK(rig.full.auxiliarySafeOffPending());

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, true);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
  CHECK(!rig.full.maxSideEvidence().has_measurement);
}

// ---------------------------------------------------------------------------
// Dynamic prerequisite loss mid MIN-probe monitoring: caught within one
// tick, not only at the next backend call.
// ---------------------------------------------------------------------------

void test_dynamic_prerequisite_lost_during_min_probe_monitoring() {
  g_case = "permit lost mid MIN probe";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const FullLegCalibrationRequest req = request();

  uint32_t t = 1000;
  CHECK(rig.full.start(req, ctx, t));
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // approach1 command
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_PROBE);

  ctx.motion_permit_active = false;  // pulled mid-flight, no new backend call pending
  t += 200; rig.full.update(ctx, t, true, telemetry(1000, 1, t), false, false);

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure,
          (int)FullLegCalibrationFailure::DYNAMIC_PREREQUISITE_LOST);

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
}

// ---------------------------------------------------------------------------
// Dynamic prerequisite loss mid auxiliary-move monitoring: dual SAFE_OFF.
// ---------------------------------------------------------------------------

void test_dynamic_prerequisite_lost_during_aux_move_monitoring() {
  g_case = "authority lost mid aux park";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux move command
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_MOVE_MONITORING);

  ctx.authority_inhibited = true;  // e.g. an OTA session claiming the exclusive hold
  t += 200; rig.full.update(ctx, t, true, telemetry(rig.backend.last_target_tick - 50, 1, t), false, false);

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure,
          (int)FullLegCalibrationFailure::DYNAMIC_PREREQUISITE_LOST);

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, true);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
}

// ---------------------------------------------------------------------------
// Operator abort during MIN-probe monitoring and during MAX-probe
// monitoring: both must request the correct SAFE_OFF set.
// ---------------------------------------------------------------------------

void test_operator_abort_during_min_probe() {
  g_case = "operator abort during MIN probe";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  uint32_t t = 1000;
  CHECK(rig.full.start(request(), ctx, t));
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // approach1 command
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_PROBE);

  rig.full.abort();
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::OPERATOR_ABORT);
  CHECK(!rig.full.auxiliarySafeOffPending());

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
}

void test_operator_abort_during_max_probe() {
  g_case = "operator abort during MAX probe";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const FullLegCalibrationRequest req = request();
  const uint16_t aux_target = resolvedTick(req.auxiliary_park_target_urad, lhUpper());

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // aux move command
  t += 200; rig.full.update(ctx, t, true, telemetry(aux_target, 1, t), false, false);  // arrival
  refreshBootstrap(rig, lease, true);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // MAX torque enable
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);  // MAX approach1 command
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);

  rig.full.abort();
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::OPERATOR_ABORT);
  CHECK(rig.full.primarySafeOffPending());
  CHECK(rig.full.auxiliarySafeOffPending());

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, true);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
}

// ---------------------------------------------------------------------------
// Structural checks: no second concurrent start(), bus id accessors, and
// auxiliaryParked() is true ONLY during the MAX probe phase.
// ---------------------------------------------------------------------------

void test_second_start_refused_while_active() {
  g_case = "second start refused while active";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(rig.full.start(request(), ctx, 1000));
  CHECK(!rig.full.start(request(), ctx, 1001));
}

void test_bus_id_and_parked_accessors() {
  g_case = "bus id and parked accessors";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK_EQ(rig.full.primaryBusId(), 0);  // never a valid ST3215 id, before start()
  CHECK_EQ(rig.full.auxiliaryBusId(), 0);
  CHECK(!rig.full.auxiliaryParked());

  uint32_t t = 1000;
  driveMinProbeAndSafeOff(rig, ctx, &t);
  CHECK_EQ(rig.full.primaryBusId(), 12);
  CHECK_EQ(rig.full.auxiliaryBusId(), 42);
  CHECK(!rig.full.auxiliaryParked());  // AUX_TORQUE_ENABLE: not yet parked
}

void test_witness_empty_before_complete() {
  g_case = "evidence empty before probes complete";
  Rig rig;
  primeTransforms(rig);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

  CHECK(!rig.full.minSideEvidence().has_measurement);
  CHECK(!rig.full.maxSideEvidence().has_measurement);
  CHECK(rig.full.start(request(), ctx, 1000));
  CHECK(!rig.full.minSideEvidence().has_measurement);
  CHECK(!rig.full.maxSideEvidence().has_measurement);
}

}  // namespace

int main() {
  test_happy_path_full_leg_completes_with_evidence_both_sides();
  test_min_probe_failure_never_reaches_aux_or_max();
  test_aux_torque_enable_uncertain_requires_dual_safe_off_and_fails();
  test_aux_move_stalled_requires_dual_safe_off_and_fails();
  test_max_probe_failure_after_park_requires_dual_safe_off_and_fails();
  test_dynamic_prerequisite_lost_during_min_probe_monitoring();
  test_dynamic_prerequisite_lost_during_aux_move_monitoring();
  test_operator_abort_during_min_probe();
  test_operator_abort_during_max_probe();
  test_second_start_refused_while_active();
  test_bus_id_and_parked_accessors();
  test_witness_empty_before_complete();

  std::printf("test_full_leg_calibration_executor: %d checks, %d failures\n", g_checks,
             g_failures);
  return g_failures == 0 ? 0 : 1;
}
