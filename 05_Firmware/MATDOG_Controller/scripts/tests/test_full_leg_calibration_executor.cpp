// Offline adversarial tests for the CR3 continuation Full Leg Calibration
// sequencer (src/calibration/FullLegCalibrationExecutor.*).
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, CalibrationExecutionEngine,
// ContactProbeEngine, ActuatorAuthorityArbiter and checked target resolver
// against a fake backend and synthetic telemetry - the same contract as
// test_contact_probe_engine.cpp and test_first_motion_executor.cpp.
//
// 4-leg generalization: the LF (aux LH_UPPER), RF (aux RH_UPPER), RH (no aux)
// and LH (no aux) sequences are all driven here. The per-leg bus ids / units
// below are this test's own ORACLE (config/MATDOG_SERVO_ALLOCATION.yaml),
// deliberately literal so a wrong dynamic lookup in production cannot hide.
//
// NO HARDWARE VALIDATION, and no fabricated physical measurement: every
// "contact" and every "arrival" in this file is a synthetic value this test
// constructs, never presented as a real endpoint result.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

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

JointIdentity upperOf(Leg leg, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = JointKind::UPPER;
  setPhysicalUnit(&id, unit);
  return id;
}

// The test oracle for the four UPPER servos and the parking matrix Geometry V5
// currently resolves: LF -> LH_UPPER, RF -> RH_UPPER, RH/LH -> none.
struct LegFixture {
  Leg leg;
  JointIdentity upper;
  uint8_t upper_bus;
  bool aux_required;
  JointIdentity aux;
  uint8_t aux_bus;
};

LegFixture fixtureFor(Leg leg) {
  switch (leg) {
    case Leg::LF: return {leg, upperOf(Leg::LF, "ELR01"), 12, true, upperOf(Leg::LH, "M42"), 42};
    case Leg::RF: return {leg, upperOf(Leg::RF, "ELR03"), 22, true, upperOf(Leg::RH, "ELR02"), 32};
    case Leg::RH: return {leg, upperOf(Leg::RH, "ELR02"), 32, false, JointIdentity{}, 0};
    case Leg::LH: return {leg, upperOf(Leg::LH, "M42"), 42, false, JointIdentity{}, 0};
  }
  return {};
}

constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};

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
    touched[bus_id] = true;
    return enable_torque_result;
  }
  BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick) override {
    ++write_goal_position_calls;
    last_bus_id = bus_id;
    touched[bus_id] = true;
    last_target_tick = target_tick;
    return write_goal_position_result;
  }

  int enable_torque_calls = 0;
  int write_goal_position_calls = 0;
  uint8_t last_bus_id = 0;
  uint16_t last_target_tick = 0;
  bool touched[256] = {false};  // every bus id any write ever addressed
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

// Every transform a full leg run needs on the current installation: the
// probed joint AND, when one is required, the one named auxiliary.
void primeTransforms(Rig& rig, const LegFixture& f) {
  CHECK(rig.policy.transforms().admit(promotedTransform(f.upper)));
  if (f.aux_required) CHECK(rig.policy.transforms().admit(promotedTransform(f.aux)));
}
void primeTransforms(Rig& rig) { primeTransforms(rig, fixtureFor(Leg::LF)); }

// Mirrors what Controller's own per-tick bootstrap-context refresh does in
// production: rebuilds the ENTIRE CalibrationBootstrapContext from live
// state on every call, including the auxiliary-parked fields, which track
// full.auxiliaryParked() exactly the way Controller reads it from
// full_leg_calibration_.auxiliaryParked() every tick.
void refreshBootstrap(Rig& rig, const AuthorityLease& lease, bool auxiliary_parked,
                      Leg leg = Leg::LF) {
  actuator::CalibrationBootstrapContext bootstrap{};
  bootstrap.session_active = true;
  bootstrap.origin = CalibrationOrigin::LIVE_SESSION;
  bootstrap.motion_permit_active = true;
  bootstrap.motion_permit_generation = 1;
  bootstrap.motion_permit_session_id = 1;
  bootstrap.motion_permit_authority_generation = lease.generation;
  bootstrap.auxiliary_parked = auxiliary_parked;
  bootstrap.parked_leg = leg;
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

// The Geometry V5 contact targets and the ONE named auxiliary parked pose
// (610865 urad) - read from the compiled data, exactly as the production
// plan resolver reads them; the backoff is the generic midpoint rule
// (contact/2 = halfway back to q0), NOT a per-leg hand-typed number.
FullLegCalibrationRequest requestFor(const LegFixture& f) {
  const actuator::CalibrationGeometryProfile profile = boundProfile();
  const actuator::GeometryEndpointRecord* min_ep =
      profile.findEndpoint(f.leg, JointKind::UPPER, ContactSide::MIN_SIDE);
  const actuator::GeometryEndpointRecord* max_ep =
      profile.findEndpoint(f.leg, JointKind::UPPER, ContactSide::MAX_SIDE);
  CHECK(min_ep != nullptr && max_ep != nullptr);
  FullLegCalibrationRequest r{};
  r.probe_joint = f.upper;
  r.probe_bus_id = f.upper_bus;
  r.endpoint_leg = f.leg;
  r.endpoint_joint = JointKind::UPPER;
  r.min_approach_urad = min_ep->contact;
  r.min_backoff_urad = min_ep->contact / 2;
  r.min_repeatability_tolerance_ticks = 16;
  r.max_approach_urad = max_ep->contact;
  r.max_backoff_urad = max_ep->contact / 2;
  r.max_repeatability_tolerance_ticks = 16;
  r.auxiliary_required = f.aux_required;
  if (f.aux_required) {
    r.auxiliary_joint = f.aux;
    r.auxiliary_bus_id = f.aux_bus;
    r.auxiliary_park_target_urad = max_ep->auxiliary_target;
  }
  return r;
}

FullLegCalibrationRequest request() { return requestFor(fixtureFor(Leg::LF)); }

// +1 when the backoff tick is above the approach tick (stall lies above the
// approach), -1 otherwise: keeps the synthetic stall between backoff and
// approach whichever way the joint's URDF direction maps urad to ticks.
int towardBackoff(uint16_t approach, uint16_t backoff) { return backoff > approach ? 1 : -1; }

// Drives start() through the MIN-side two-pass probe to its COMPLETE
// terminal (UPPER_MIN_SAFE_OFF), then services that SAFE_OFF (primary only).
// The caller inspects the phase it lands in: AUX_TORQUE_ENABLE with an
// auxiliary, UPPER_MAX_PROBE without one.
void driveMinProbeThenSafeOff(Rig& rig, const LegFixture& f, const FullLegCalibrationContext& ctx,
                              uint32_t* t) {
  const FullLegCalibrationRequest req = requestFor(f);
  const uint16_t min_approach = resolvedTick(req.min_approach_urad, f.upper);
  const uint16_t min_backoff = resolvedTick(req.min_backoff_urad, f.upper);
  const int dir = towardBackoff(min_approach, min_backoff);

  CHECK(rig.full.start(req, ctx, *t));
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // torque enable
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // approach1 command
  CHECK_EQ(rig.backend.last_target_tick, min_approach);
  const uint16_t stall1 = static_cast<uint16_t>(min_approach + dir * 20);
  *t = 1500; rig.full.update(ctx, *t, true, telemetry(stall1, 1, *t), false, false);
  *t = 3500; rig.full.update(ctx, *t, true, telemetry(stall1, 1, *t), false, false);  // -> backoff
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // backoff command
  CHECK_EQ(rig.backend.last_target_tick, min_backoff);
  *t += 90; rig.full.update(ctx, *t, true, telemetry(min_backoff, 1, *t), false, false);  // arrival -> approach2
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // approach2 command
  const uint16_t stall2 = static_cast<uint16_t>(stall1 + dir * 6);
  *t += 490; rig.full.update(ctx, *t, true, telemetry(stall2, 1, *t), false, false);
  *t += 2000; rig.full.update(ctx, *t, true, telemetry(stall2, 1, *t), false, false);  // -> COMPLETE

  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::NONE);

  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, /*primary=*/true, false);
}

// The LF-with-auxiliary common prefix every AUX/MAX-phase test below needs.
void driveMinProbeAndSafeOff(Rig& rig, const FullLegCalibrationContext& ctx, uint32_t* t) {
  driveMinProbeThenSafeOff(rig, fixtureFor(Leg::LF), ctx, t);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_TORQUE_ENABLE);
}

// Drives the MAX-side two-pass probe (already started: phase UPPER_MAX_PROBE)
// through its COMPLETE terminal, landing in FINAL_SAFE_OFF.
void driveMaxProbeToFinalSafeOff(Rig& rig, const LegFixture& f,
                                 const FullLegCalibrationContext& ctx, uint32_t* t) {
  const FullLegCalibrationRequest req = requestFor(f);
  const uint16_t max_approach = resolvedTick(req.max_approach_urad, f.upper);
  const uint16_t max_backoff = resolvedTick(req.max_backoff_urad, f.upper);
  const int dir = towardBackoff(max_approach, max_backoff);

  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // MAX torque enable
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // MAX approach1 command
  CHECK_EQ(rig.backend.last_bus_id, f.upper_bus);
  CHECK_EQ(rig.backend.last_target_tick, max_approach);
  const uint16_t stall1 = static_cast<uint16_t>(max_approach + dir * 20);
  *t += 500; rig.full.update(ctx, *t, true, telemetry(stall1, 1, *t), false, false);
  *t += 2000; rig.full.update(ctx, *t, true, telemetry(stall1, 1, *t), false, false);  // -> backoff
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // backoff command
  CHECK_EQ(rig.backend.last_target_tick, max_backoff);
  *t += 90; rig.full.update(ctx, *t, true, telemetry(max_backoff, 1, *t), false, false);  // -> approach2
  *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);  // approach2 command
  const uint16_t stall2 = static_cast<uint16_t>(stall1 + dir * 6);
  *t += 490; rig.full.update(ctx, *t, true, telemetry(stall2, 1, *t), false, false);
  *t += 2000; rig.full.update(ctx, *t, true, telemetry(stall2, 1, *t), false, false);  // -> probe COMPLETE
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

// ---------------------------------------------------------------------------
// 4-leg generalization. The oracle above (fixtureFor) says LF -> aux LH_UPPER,
// RF -> aux RH_UPPER, RH/LH -> no auxiliary. Every leg is driven end to end.
// ---------------------------------------------------------------------------

// Drives one complete, successful full-leg sequence for `f` and returns the
// terminal phase. Asserts along the way that the auxiliary phases happen if,
// and only if, the leg needs an auxiliary, and that no write ever addresses
// any bus other than the leg's own UPPER (and, if required, its ONE auxiliary).
void runCompleteLeg(Rig& rig, const LegFixture& f, uint32_t* t) {
  primeTransforms(rig, f);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false, f.leg);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  const FullLegCalibrationRequest req = requestFor(f);

  driveMinProbeThenSafeOff(rig, f, ctx, t);
  CHECK_EQ(rig.full.primaryBusId(), f.upper_bus);
  CHECK_EQ(rig.full.auxiliaryBusId(), f.aux_bus);  // 0 when no auxiliary
  CHECK_EQ(rig.full.auxiliaryRequired(), f.aux_required);

  if (f.aux_required) {
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_TORQUE_ENABLE);
    const uint16_t aux_target = resolvedTick(req.auxiliary_park_target_urad, f.aux);
    *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_MOVE_PENDING);
    CHECK_EQ(rig.backend.last_bus_id, f.aux_bus);
    *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, false);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::AUX_MOVE_MONITORING);
    CHECK_EQ(rig.backend.last_bus_id, f.aux_bus);
    CHECK_EQ(rig.backend.last_target_tick, aux_target);
    *t += 200; rig.full.update(ctx, *t, true, telemetry(aux_target, 1, *t), false, false);
    CHECK(rig.full.auxiliaryParked());
    refreshBootstrap(rig, lease, true, f.leg);
  } else {
    // NOT_NEEDED: straight to the MAX probe, no AUX_* phase, never parked.
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);
    CHECK(!rig.full.auxiliaryParked());
    CHECK(!rig.full.auxiliarySafeOffPending());
    refreshBootstrap(rig, lease, false, f.leg);
  }
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);

  driveMaxProbeToFinalSafeOff(rig, f, ctx, t);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::NONE);
  CHECK(rig.full.primarySafeOffPending());
  CHECK_EQ(rig.full.auxiliarySafeOffPending(), f.aux_required);

  if (f.aux_required) {
    // Both must be verified: the primary alone is not enough.
    *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, true, false);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
    *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, false, true);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
    *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, true, true);
  } else {
    // The auxiliary flag is irrelevant and must not be needed: the primary
    // alone completes the leg.
    *t += 10; rig.full.update(ctx, *t, false, TelemetrySample{}, true, false);
  }
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::COMPLETE);
  CHECK(!rig.full.active());
  CHECK(rig.full.minSideEvidence().has_measurement);
  CHECK(rig.full.maxSideEvidence().has_measurement);
  CHECK(rig.full.minSideEvidence().witness.accepted());
  CHECK(rig.full.maxSideEvidence().witness.accepted());
  CHECK((int)rig.full.minSideEvidence().key.leg == (int)f.leg);
  CHECK((int)rig.full.maxSideEvidence().key.leg == (int)f.leg);

  // Bus discipline: writes reached ONLY the primary and (if required) the
  // one named auxiliary; never bus 0.
  int touched_count = 0;
  for (int b = 0; b < 256; ++b) {
    if (!rig.backend.touched[b]) continue;
    ++touched_count;
    CHECK(b == f.upper_bus || (f.aux_required && b == f.aux_bus));
  }
  CHECK(!rig.backend.touched[0]);
  CHECK_EQ(touched_count, f.aux_required ? 2 : 1);
}

void test_every_leg_completes_with_the_right_auxiliary_matrix() {
  for (const Leg leg : kAllLegs) {
    g_case = "4-leg complete sequence";
    Rig rig;
    uint32_t t = 1000;
    runCompleteLeg(rig, fixtureFor(leg), &t);
  }
}

// The evidence a leg produces is bound to ITS OWN leg's UPPER identity and a
// READY contact-derived envelope builds from it for all four legs.
void test_every_leg_evidence_builds_a_ready_upper_envelope() {
  for (const Leg leg : kAllLegs) {
    g_case = "4-leg evidence builds envelope";
    const LegFixture f = fixtureFor(leg);
    Rig rig;
    uint32_t t = 1000;
    runCompleteLeg(rig, f, &t);
    actuator::ContactDerivedEnvelopeRequest env_req{};
    env_req.joint = f.upper;
    env_req.min_side_evidence = rig.full.minSideEvidence();
    env_req.max_side_evidence = rig.full.maxSideEvidence();
    env_req.min_side_geometry =
        actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
    env_req.max_side_geometry = env_req.min_side_geometry;
    env_req.safety_margin_ticks = 8;
    actuator::OperationalEnvelope envelope{};
    const actuator::EnvelopeBuildStatus st = actuator::buildContactDerivedEnvelope(
        rig.profile, actuator::geometry_data::kProvenance, env_req, &envelope);
    CHECK(st == actuator::EnvelopeBuildStatus::READY);
    CHECK(envelope.present);
    CHECK(envelope.ordered());
  }
}

void test_start_accepts_no_auxiliary_and_rejects_bad_required_auxiliary() {
  g_case = "start(): auxiliary requirement validation";
  const LegFixture rh = fixtureFor(Leg::RH);
  const LegFixture lf = fixtureFor(Leg::LF);

  {  // no auxiliary: empty identity + bus 0 is accepted
    Rig rig;
    primeTransforms(rig, rh);
    const AuthorityLease lease =
        grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
    refreshBootstrap(rig, lease, false, rh.leg);
    const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
    FullLegCalibrationRequest r = requestFor(rh);
    CHECK(!r.auxiliary_required);
    CHECK_EQ(r.auxiliary_bus_id, 0);
    CHECK(!r.auxiliary_joint.unitKnown());
    CHECK(rig.full.start(r, ctx, 1000));
  }
  {  // no auxiliary: a stray non-empty auxiliary field is ignored, never used
    Rig rig;
    primeTransforms(rig, rh);
    const AuthorityLease lease =
        grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
    refreshBootstrap(rig, lease, false, rh.leg);
    const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
    FullLegCalibrationRequest r = requestFor(rh);
    r.auxiliary_joint = fixtureFor(Leg::LH).upper;
    r.auxiliary_bus_id = 42;
    CHECK(rig.full.start(r, ctx, 1000));
    CHECK_EQ(rig.full.auxiliaryBusId(), 0);
    CHECK(!rig.full.auxiliarySafeOffPending());
  }
  {  // aux required, but empty identity
    Rig rig;
    primeTransforms(rig, lf);
    const AuthorityLease lease =
        grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
    refreshBootstrap(rig, lease, false, lf.leg);
    const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
    FullLegCalibrationRequest r = requestFor(lf);
    r.auxiliary_joint = JointIdentity{};
    CHECK(!rig.full.start(r, ctx, 1000));
  }
  {  // aux required, but bus 0
    Rig rig;
    primeTransforms(rig, lf);
    const AuthorityLease lease =
        grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
    refreshBootstrap(rig, lease, false, lf.leg);
    const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
    FullLegCalibrationRequest r = requestFor(lf);
    r.auxiliary_bus_id = 0;
    CHECK(!rig.full.start(r, ctx, 1000));
  }
  {  // aux required, but the same bus as the primary
    Rig rig;
    primeTransforms(rig, lf);
    const AuthorityLease lease =
        grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
    refreshBootstrap(rig, lease, false, lf.leg);
    const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
    FullLegCalibrationRequest r = requestFor(lf);
    r.auxiliary_bus_id = r.probe_bus_id;
    CHECK(!rig.full.start(r, ctx, 1000));
  }
  {  // the default is fail-closed: a value-initialised request requires an auxiliary
    FullLegCalibrationRequest r{};
    CHECK(r.auxiliary_required);
  }
}

// Without an auxiliary the final SAFE_OFF services exactly ONE servo: the
// executor asks for the primary only and never reports the auxiliary pending,
// so the Controller can never be led to safeOff(0).
void test_no_auxiliary_final_safe_off_is_primary_only_and_never_bus_zero() {
  g_case = "no-aux SAFE_OFF primary only";
  for (const Leg leg : {Leg::RH, Leg::LH}) {
    const LegFixture f = fixtureFor(leg);
    Rig rig;
    uint32_t t = 1000;
    primeTransforms(rig, f);
    const AuthorityLease lease =
        grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
    refreshBootstrap(rig, lease, false, f.leg);
    const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);

    driveMinProbeThenSafeOff(rig, f, ctx, &t);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);
    rig.full.abort();  // mid MAX probe
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
    CHECK(rig.full.primarySafeOffPending());
    CHECK(!rig.full.auxiliarySafeOffPending());
    CHECK_EQ(rig.full.auxiliaryBusId(), 0);
    t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, false);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::OPERATOR_ABORT);
  }
}

// A no-aux leg whose MAX probe is refused by the policy at its first command
// (here: the approach target lies outside the URDF domain) must SAFE_OFF the
// primary alone and end FAILED - never an AUX_* phase, never a stuck
// FINAL_SAFE_OFF, and the already-witnessed MIN side stays untouched.
void test_no_auxiliary_max_probe_refused_ends_failed_after_primary_safe_off() {
  g_case = "no-aux MAX probe refused";
  const LegFixture f = fixtureFor(Leg::RH);
  Rig rig;
  primeTransforms(rig, f);
  const AuthorityLease lease =
      grant(rig.arbiter, ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE);
  refreshBootstrap(rig, lease, false, f.leg);
  const FullLegCalibrationContext ctx = liveContext(lease, OperatingMode::MAINTENANCE);
  FullLegCalibrationRequest req = requestFor(f);
  req.max_approach_urad = 900000000;  // far outside every URDF domain

  uint32_t t = 1000;
  const uint16_t min_approach = resolvedTick(req.min_approach_urad, f.upper);
  const uint16_t min_backoff = resolvedTick(req.min_backoff_urad, f.upper);
  const int dir = towardBackoff(min_approach, min_backoff);
  CHECK(rig.full.start(req, ctx, t));
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);
  const uint16_t stall1 = static_cast<uint16_t>(min_approach + dir * 20);
  t = 1500; rig.full.update(ctx, t, true, telemetry(stall1, 1, t), false, false);
  t = 3500; rig.full.update(ctx, t, true, telemetry(stall1, 1, t), false, false);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);
  t += 90; rig.full.update(ctx, t, true, telemetry(min_backoff, 1, t), false, false);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);
  const uint16_t stall2 = static_cast<uint16_t>(stall1 + dir * 6);
  t += 490; rig.full.update(ctx, t, true, telemetry(stall2, 1, t), false, false);
  t += 2000; rig.full.update(ctx, t, true, telemetry(stall2, 1, t), false, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);

  // MAX torque enable, then the refused approach command.
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);
  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, false, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FINAL_SAFE_OFF);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED);
  CHECK(rig.full.primarySafeOffPending());
  CHECK(!rig.full.auxiliarySafeOffPending());
  CHECK(rig.full.minSideEvidence().has_measurement);
  CHECK(!rig.full.maxSideEvidence().has_measurement);

  t += 10; rig.full.update(ctx, t, false, TelemetrySample{}, true, false);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
  CHECK(!rig.backend.touched[0]);
}

// RF's auxiliary is RH_UPPER (bus 32), never LH_UPPER, and its park pose is
// resolved through RH_UPPER's own transform (dir -1), not LF's.
void test_rf_auxiliary_is_rh_upper_with_its_own_transform() {
  g_case = "RF aux is RH_UPPER bus 32";
  const LegFixture f = fixtureFor(Leg::RF);
  CHECK_EQ(f.upper_bus, 22);
  CHECK_EQ(f.aux_bus, 32);
  CHECK((int)f.aux.leg == (int)Leg::RH);
  Rig rig;
  uint32_t t = 1000;
  runCompleteLeg(rig, f, &t);
  CHECK(rig.backend.touched[22]);
  CHECK(rig.backend.touched[32]);
  CHECK(!rig.backend.touched[42]);
  CHECK(!rig.backend.touched[12]);
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
  test_every_leg_completes_with_the_right_auxiliary_matrix();
  test_every_leg_evidence_builds_a_ready_upper_envelope();
  test_start_accepts_no_auxiliary_and_rejects_bad_required_auxiliary();
  test_no_auxiliary_final_safe_off_is_primary_only_and_never_bus_zero();
  test_no_auxiliary_max_probe_refused_ends_failed_after_primary_safe_off();
  test_rf_auxiliary_is_rh_upper_with_its_own_transform();

  std::printf("test_full_leg_calibration_executor: %d checks, %d failures\n", g_checks,
             g_failures);
  return g_failures == 0 ? 0 : 1;
}
