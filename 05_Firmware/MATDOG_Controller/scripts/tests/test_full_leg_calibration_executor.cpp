// Offline tests for the Full Leg Calibration sequencer
// (src/calibration/FullLegCalibrationExecutor.*) running the 2026-09-29 staged
// endpoint search on both UPPER sides of all four legs.
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, CalibrationExecutionEngine,
// ContactProbeEngine, arbiter, Geometry V5 profile and checked resolvers against
// kinematic_servo_sim.h, and ticks the executor exactly the way
// Controller::updateFullLegCalibration() does (bootstrap refreshed every tick
// from auxiliaryParked(), telemetry from the aux bus only while it is being
// parked, SAFE_OFF pending flags sampled before update() and SAFE_OFF forced
// after it). The per-leg bus ids/units are this test's own ORACLE
// (config/MATDOG_SERVO_ALLOCATION.yaml), deliberately literal.
//
// NO HARDWARE VALIDATION, no fabricated measurement: every stop, plateau and
// current here is synthetic.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <initializer_list>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/OperationalEnvelope.h"
#include "../../src/calibration/FullLegCalibrationExecutor.h"
#include "kinematic_servo_sim.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::ActuatorRuntime;
using matdog::actuator::BackendWriteOutcome;
using matdog::actuator::CalibrationGeometryProfile;
using matdog::actuator::CalibrationSearchCorridor;
using matdog::actuator::MotionDeadmanConfig;
using matdog::actuator::MotionProfile;
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

JointIdentity upperOf(Leg leg, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = JointKind::UPPER;
  setPhysicalUnit(&id, unit);
  return id;
}

// The parking matrix Geometry V5 resolves today: LF -> LH_UPPER,
// RF -> RH_UPPER, RH/LH -> none. q0 = the 2026-09-29 22:05 capture.
struct LegFixture {
  Leg leg;
  JointIdentity upper;
  uint8_t upper_bus;
  uint16_t upper_q0;
  bool aux_required;
  JointIdentity aux;
  uint8_t aux_bus;
  uint16_t aux_q0;
};

LegFixture fixtureFor(Leg leg) {
  switch (leg) {
    case Leg::LF:
      return {leg, upperOf(Leg::LF, "ELR01"), 12, 2086, true, upperOf(Leg::LH, "M42"), 42, 2072};
    case Leg::RF:
      return {leg, upperOf(Leg::RF, "ELR03"), 22, 2108, true, upperOf(Leg::RH, "ELR02"), 32, 2042};
    case Leg::RH:
      return {leg, upperOf(Leg::RH, "ELR02"), 32, 2042, false, JointIdentity{}, 0, 0};
    case Leg::LH:
      return {leg, upperOf(Leg::LH, "M42"), 42, 2072, false, JointIdentity{}, 0, 0};
  }
  return {};
}

constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
constexpr int kHardwareStopBeyondContact = 23;  // LF_UPPER MIN, found by hand

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&actuator::geometry_data::kProvenance, actuator::geometry_data::kJoints,
              actuator::geometry_data::kJointCount, actuator::geometry_data::kEndpoints,
              actuator::geometry_data::kEndpointCount);
  return profile;
}

actuator::JointTransform promotedTransform(JointIdentity id, uint16_t q0) {
  actuator::JointTransform t{};
  t.identity = id;
  t.geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.q0_tick = q0;
  t.present = true;
  return t;
}

// Controller::begin()'s Full-Leg figures.
MotionDeadmanConfig controllerMove(uint16_t arrival) {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = arrival;
  c.nominal_travel_ticks_per_s = kSearchMinExpectedTicksPerSecond;
  return c;
}

class LegBackend : public simk::SimBackend {
 public:
  BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    if (bus_id == uncertain_torque_bus) return BackendWriteOutcome::UNCERTAIN;
    return simk::SimBackend::enableTorque(bus_id);
  }
  uint8_t uncertain_torque_bus = 0;
};

struct LegRig {
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  ActuatorRuntime runtime;
  CalibrationExecutionEngine engine;
  LegBackend backend;
  CalibrationGeometryProfile profile;
  FullLegCalibrationExecutor full;
  AuthorityLease lease{};
  LegFixture f;
  FullLegCalibrationRequest req{};
  bool permit = true;
  int safe_off_calls[256] = {0};
  bool touched_bus_zero = false;

  explicit LegRig(const LegFixture& fx) : f(fx) {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance);
    FullLegCalibrationConfig config{};
    config.probe_backoff_deadman = controllerMove(kSearchStaticToleranceTicks + 2);
    config.aux_move_deadman = controllerMove(kSearchStaticToleranceTicks);
    full.begin(&policy, &runtime, &engine, &profile, &actuator::geometry_data::kProvenance, config);

    const actuator::JointTransform t = promotedTransform(f.upper, f.upper_q0);
    CHECK(policy.transforms().admit(t));
    if (f.aux_required) CHECK(policy.transforms().admit(promotedTransform(f.aux, f.aux_q0)));
    CHECK(arbiter.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &lease) ==
          AuthorityResult::GRANTED);

    req.probe_joint = f.upper;
    req.probe_bus_id = f.upper_bus;
    req.endpoint_leg = f.leg;
    req.endpoint_joint = JointKind::UPPER;
    req.min_repeatability_tolerance_ticks = 16;
    req.max_repeatability_tolerance_ticks = 16;
    CHECK(actuator::resolveCalibrationSearchCorridor(profile, actuator::geometry_data::kProvenance, t,
                                                     f.leg, JointKind::UPPER, ContactSide::MIN_SIDE,
                                                     &req.min_search) == actuator::TargetResolveStatus::OK);
    CHECK(actuator::resolveCalibrationSearchCorridor(profile, actuator::geometry_data::kProvenance, t,
                                                     f.leg, JointKind::UPPER, ContactSide::MAX_SIDE,
                                                     &req.max_search) == actuator::TargetResolveStatus::OK);
    req.auxiliary_required = f.aux_required;
    if (f.aux_required) {
      req.auxiliary_joint = f.aux;
      req.auxiliary_bus_id = f.aux_bus;
      req.auxiliary_park_target_urad =
          profile.findEndpoint(f.leg, JointKind::UPPER, ContactSide::MAX_SIDE)->auxiliary_target;
      backend.joint[f.aux_bus].pos = f.aux_q0;
    }
    backend.joint[f.upper_bus].pos = f.upper_q0;
  }

  // Hard stops `beyond` ticks past each canonical contact.
  void placeStops(int min_beyond, int max_beyond, bool min_stop = true, bool max_stop = true) {
    simk::SimJoint& j = backend.joint[f.upper_bus];
    for (const bool min_side : {true, false}) {
      if (min_side ? !min_stop : !max_stop) continue;
      const CalibrationSearchCorridor& c = min_side ? req.min_search : req.max_search;
      const int d = actuator::searchDepth(c, c.contact_tick) + (min_side ? min_beyond : max_beyond);
      const double stop = c.home_tick + c.probe_sign * d;
      if (c.probe_sign < 0) {
        j.has_stop_low = true;
        j.stop_low = stop;
      } else {
        j.has_stop_high = true;
        j.stop_high = stop;
      }
    }
  }
  int stopFor(bool min_side) const {
    const CalibrationSearchCorridor& c = min_side ? req.min_search : req.max_search;
    const simk::SimJoint& j = backend.joint[f.upper_bus];
    return static_cast<int>(c.probe_sign < 0 ? j.stop_low : j.stop_high);
  }

  FullLegCalibrationContext ctx() const {
    FullLegCalibrationContext c{};
    c.session_active = true;
    c.origin = CalibrationOrigin::LIVE_SESSION;
    c.lease = lease;
    c.mode = OperatingMode::MAINTENANCE;
    c.motion_permit_active = permit;
    c.authority = ActuatorAuthority::CALIBRATION;
    c.authority_generation = lease.generation;
    c.authority_inhibited = false;
    return c;
  }

  void refreshBootstrap() {
    actuator::CalibrationBootstrapContext b{};
    b.session_active = true;
    b.origin = CalibrationOrigin::LIVE_SESSION;
    b.motion_permit_active = permit;
    b.motion_permit_generation = 1;
    b.motion_permit_session_id = 1;
    b.motion_permit_authority_generation = lease.generation;
    b.auxiliary_parked = full.auxiliaryParked();
    b.parked_leg = full.endpointLeg();
    b.parked_joint = full.endpointJoint();
    b.parked_side = ContactSide::MAX_SIDE;
    policy.setBootstrapContext(b);
  }
};

using Hook = std::function<void(LegRig&, uint32_t)>;

// Controller::updateFullLegCalibration(), tick for tick.
uint32_t runLeg(LegRig& rig, const Hook& hook = nullptr, uint32_t limit_ms = 400000) {
  uint32_t t = 1000;
  rig.refreshBootstrap();
  CHECK(rig.full.start(rig.req, rig.ctx(), t));
  const uint32_t started = t;
  bool primary_cache = false, aux_cache = false;
  while (rig.full.active() && t - started < limit_ms) {
    t += 10;
    rig.backend.advance(t, 10);
    if (hook) hook(rig, t);
    rig.refreshBootstrap();
    const bool was_primary = rig.full.primarySafeOffPending();
    const bool was_aux = rig.full.auxiliarySafeOffPending();
    if (!was_primary) primary_cache = false;
    if (!was_aux) aux_cache = false;
    const FullLegCalibrationPhase p = rig.full.status().phase;
    const bool aux_phase = p == FullLegCalibrationPhase::AUX_MOVE_PENDING ||
                           p == FullLegCalibrationPhase::AUX_MOVE_MONITORING;
    const uint8_t bus = aux_phase ? rig.full.auxiliaryBusId() : rig.full.primaryBusId();
    rig.full.update(rig.ctx(), t, true, rig.backend.joint[bus].sample(t), was_primary && primary_cache,
                    was_aux && aux_cache);
    if (rig.full.primarySafeOffPending()) {
      const uint8_t b = rig.full.primaryBusId();
      if (b == 0) rig.touched_bus_zero = true;
      ++rig.safe_off_calls[b];
      rig.backend.joint[b].torque = false;
      primary_cache = true;
    }
    if (rig.full.auxiliarySafeOffPending()) {
      const uint8_t b = rig.full.auxiliaryBusId();
      if (b == 0) rig.touched_bus_zero = true;
      ++rig.safe_off_calls[b];
      rig.backend.joint[b].torque = false;
      aux_cache = true;
    }
  }
  return t - started;
}

void checkBusDiscipline(const LegRig& rig) {
  for (const simk::GoalWrite& w : rig.backend.writes) {
    CHECK(w.bus == rig.f.upper_bus || (rig.f.aux_required && w.bus == rig.f.aux_bus));
    CHECK(w.bus != 0);
    CHECK(w.profile == MotionProfile::CALIBRATION_SEARCH);
  }
  CHECK(!rig.touched_bus_zero);
  if (!rig.f.aux_required) {
    for (int b = 0; b < 256; ++b) {
      if (b != rig.f.upper_bus) CHECK_EQ(rig.backend.torque_writes[b], 0);
    }
  }
}

// --- the whole leg, all four legs ----------------------------------------------

void test_every_leg_completes_at_the_physical_stops() {
  for (const Leg leg : kAllLegs) {
    g_case = "4-leg staged search: COMPLETE, contacts at the physical stops, fast";
    LegRig rig(fixtureFor(leg));
    rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
    const uint32_t elapsed = runLeg(rig);
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::COMPLETE);
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::NONE);
    const ContactEvidence& lo = rig.full.minSideEvidence();
    const ContactEvidence& hi = rig.full.maxSideEvidence();
    CHECK(lo.has_measurement && lo.witness.accepted());
    CHECK(hi.has_measurement && hi.witness.accepted());
    CHECK((int)lo.key.leg == (int)leg && (int)hi.key.leg == (int)leg);
    CHECK((int)lo.key.side == (int)ContactSide::MIN_SIDE);
    CHECK((int)hi.key.side == (int)ContactSide::MAX_SIDE);
    CHECK(std::abs(lo.coarse_tick - rig.stopFor(true)) <= 1);
    CHECK(std::abs(lo.fine_tick_1 - rig.stopFor(true)) <= 1);
    CHECK(std::abs(hi.coarse_tick - rig.stopFor(false)) <= 1);
    CHECK(std::abs(hi.fine_tick_1 - rig.stopFor(false)) <= 1);
    checkBusDiscipline(rig);
    // Both servos SAFE_OFF at the end, the auxiliary only where one was used.
    CHECK(!rig.backend.joint[rig.f.upper_bus].torque);
    if (rig.f.aux_required) {
      CHECK(!rig.backend.joint[rig.f.aux_bus].torque);
      CHECK(rig.safe_off_calls[rig.f.aux_bus] >= 1);
      CHECK_EQ(rig.backend.torque_writes[rig.f.aux_bus], 1);
      CHECK_EQ(rig.backend.goalWritesTo(rig.f.aux_bus), 1);  // one park write, nothing else
    }
    // The old single slow probe could not finish ONE side in 12 s; the staged
    // search does the whole leg (both sides, two passes each, the park) in
    // well under a minute of simulated time.
    CHECK(elapsed < 60000);

    // The evidence builds a READY UPPER envelope (the finalizer's next step).
    actuator::ContactDerivedEnvelopeRequest env{};
    env.joint = rig.f.upper;
    env.min_side_evidence = lo;
    env.max_side_evidence = hi;
    env.min_side_geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
    env.max_side_geometry = env.min_side_geometry;
    env.safety_margin_ticks = 8;
    actuator::OperationalEnvelope envelope{};
    CHECK(actuator::buildContactDerivedEnvelope(rig.profile, actuator::geometry_data::kProvenance,
                                                env, &envelope) == actuator::EnvelopeBuildStatus::READY);
  }
}

void test_aux_park_is_exact_and_only_during_max() {
  for (const Leg leg : {Leg::LF, Leg::RF}) {
    g_case = "LF/RF: the auxiliary parks at the compiler's exact pose, only for MAX";
    LegRig rig(fixtureFor(leg));
    rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
    bool parked_during_min = false, parked_during_max = false;
    runLeg(rig, [&](LegRig& r, uint32_t) {
      const FullLegCalibrationPhase p = r.full.status().phase;
      if (p == FullLegCalibrationPhase::UPPER_MIN_PROBE && r.full.auxiliaryParked()) parked_during_min = true;
      if (p == FullLegCalibrationPhase::UPPER_MAX_PROBE && r.full.auxiliaryParked()) parked_during_max = true;
    });
    CHECK(!parked_during_min);
    CHECK(parked_during_max);
    uint16_t park_tick = 0;
    CHECK(actuator::resolveUrdfQToRaw(rig.profile, actuator::geometry_data::kProvenance,
                                      promotedTransform(rig.f.aux, rig.f.aux_q0),
                                      rig.req.auxiliary_park_target_urad, &park_tick) ==
          actuator::TargetResolveStatus::OK);
    int aux_writes = 0;
    for (const simk::GoalWrite& w : rig.backend.writes) {
      if (w.bus != rig.f.aux_bus) continue;
      ++aux_writes;
      CHECK_EQ(w.tick, park_tick);
      CHECK(w.profile == MotionProfile::CALIBRATION_SEARCH);
    }
    CHECK_EQ(aux_writes, 1);
    CHECK_EQ(rig.req.auxiliary_park_target_urad, 610865);
  }
}

// --- failures route through SAFE_OFF ---------------------------------------------

void test_min_side_failure_never_reaches_aux_or_max() {
  g_case = "no MIN stop: UPPER_MIN_PROBE_FAILED, primary SAFE_OFF, auxiliary never touched";
  LegRig rig(fixtureFor(Leg::LF));
  rig.placeStops(0, kHardwareStopBeyondContact, /*min_stop=*/false, /*max_stop=*/true);
  runLeg(rig);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::UPPER_MIN_PROBE_FAILED);
  CHECK_EQ((int)rig.full.probeStatus().failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
  CHECK(!rig.backend.joint[12].torque);
  CHECK_EQ(rig.backend.torque_writes[42], 0);
  CHECK_EQ(rig.backend.goalWritesTo(42), 0);
  CHECK_EQ(rig.safe_off_calls[42], 0);
}

void test_max_side_failure_after_park_needs_dual_safe_off() {
  g_case = "no MAX stop after park: UPPER_MAX_PROBE_FAILED, primary AND auxiliary SAFE_OFF";
  LegRig rig(fixtureFor(Leg::RF));
  rig.placeStops(kHardwareStopBeyondContact, 0, true, /*max_stop=*/false);
  runLeg(rig);
  CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED);
  CHECK(rig.full.minSideEvidence().has_measurement);  // the MIN side stays witnessed
  CHECK(!rig.backend.joint[22].torque);
  CHECK(!rig.backend.joint[32].torque);
  CHECK(rig.safe_off_calls[32] >= 1);
}

void test_aux_move_stall_needs_dual_safe_off() {
  g_case = "auxiliary blocked before its park pose: AUX_MOVE_STALLED, dual SAFE_OFF";
  LegRig rig(fixtureFor(Leg::LF));
  rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
  simk::SimJoint& aux = rig.backend.joint[42];
  aux.has_stop_high = true;  // LH_UPPER parks toward +q = +raw (direction +1)
  aux.stop_high = rig.f.aux_q0 + 100;
  runLeg(rig);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::AUX_MOVE_STALLED);
  CHECK(!rig.backend.joint[12].torque);
  CHECK(!rig.backend.joint[42].torque);
}

void test_aux_torque_uncertain_needs_dual_safe_off() {
  g_case = "auxiliary TorqueEnable uncertain: AUX_TORQUE_ENABLE_UNCERTAIN, dual SAFE_OFF";
  LegRig rig(fixtureFor(Leg::RF));
  rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
  rig.backend.uncertain_torque_bus = 32;
  runLeg(rig);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::AUX_TORQUE_ENABLE_UNCERTAIN);
  CHECK(rig.safe_off_calls[22] >= 1);
  CHECK(rig.safe_off_calls[32] >= 1);
}

void test_dynamic_prerequisite_loss_stops_every_phase() {
  const FullLegCalibrationPhase phases[3] = {FullLegCalibrationPhase::UPPER_MIN_PROBE,
                                             FullLegCalibrationPhase::AUX_MOVE_MONITORING,
                                             FullLegCalibrationPhase::UPPER_MAX_PROBE};
  for (const FullLegCalibrationPhase when : phases) {
    g_case = "permit lost mid-phase: DYNAMIC_PREREQUISITE_LOST through SAFE_OFF";
    LegRig rig(fixtureFor(Leg::LF));
    rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
    int seen = 0;
    runLeg(rig, [&](LegRig& r, uint32_t) {
      if (r.full.status().phase == when && ++seen == 20) r.permit = false;
    });
    CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::DYNAMIC_PREREQUISITE_LOST);
    CHECK(!rig.backend.joint[12].torque);
    if (when != FullLegCalibrationPhase::UPPER_MIN_PROBE) CHECK(!rig.backend.joint[42].torque);
  }
}

void test_operator_abort_min_and_max() {
  for (const bool during_max : {false, true}) {
    for (const Leg leg : {Leg::LF, Leg::RH}) {
      g_case = "operator abort: OPERATOR_ABORT, every servo in use SAFE_OFF, never bus 0";
      LegRig rig(fixtureFor(leg));
      rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
      const FullLegCalibrationPhase when =
          during_max ? FullLegCalibrationPhase::UPPER_MAX_PROBE : FullLegCalibrationPhase::UPPER_MIN_PROBE;
      int seen = 0;
      runLeg(rig, [&](LegRig& r, uint32_t) {
        if (r.full.status().phase == when && ++seen == 30) r.full.abort();
      });
      CHECK_EQ((int)rig.full.status().phase, (int)FullLegCalibrationPhase::FAILED);
      CHECK_EQ((int)rig.full.status().failure, (int)FullLegCalibrationFailure::OPERATOR_ABORT);
      CHECK(!rig.backend.joint[rig.f.upper_bus].torque);
      CHECK(!rig.touched_bus_zero);
      if (rig.f.aux_required && during_max) CHECK(!rig.backend.joint[rig.f.aux_bus].torque);
      if (!rig.f.aux_required) CHECK_EQ(rig.full.auxiliaryBusId(), 0);
    }
  }
}

// --- start() validation ------------------------------------------------------------

void test_start_validation() {
  g_case = "start(): corridors, tolerances, auxiliary requirement";
  {
    LegRig rig(fixtureFor(Leg::RH));
    FullLegCalibrationRequest r = rig.req;
    CHECK(!r.auxiliary_required);
    CHECK_EQ(r.auxiliary_bus_id, 0);
    rig.refreshBootstrap();
    CHECK(rig.full.start(r, rig.ctx(), 1000));
    CHECK(!rig.full.start(r, rig.ctx(), 1000));  // second start while active
  }
  {
    LegRig rig(fixtureFor(Leg::RH));
    FullLegCalibrationRequest r = rig.req;
    r.auxiliary_joint = fixtureFor(Leg::LH).upper;  // stray, ignored without a requirement
    r.auxiliary_bus_id = 42;
    CHECK(rig.full.start(r, rig.ctx(), 1000));
    CHECK_EQ(rig.full.auxiliaryBusId(), 0);
  }
  for (int broken = 0; broken < 6; ++broken) {
    LegRig rig(fixtureFor(Leg::LF));
    FullLegCalibrationRequest r = rig.req;
    switch (broken) {
      case 0: r.min_search.probe_sign = 0; break;
      case 1: r.max_search = CalibrationSearchCorridor{}; break;
      case 2: r.min_repeatability_tolerance_ticks = 0; break;
      case 3: r.auxiliary_joint = JointIdentity{}; break;
      case 4: r.auxiliary_bus_id = 0; break;
      case 5: r.auxiliary_bus_id = r.probe_bus_id; break;
    }
    CHECK(!rig.full.start(r, rig.ctx(), 1000));
    CHECK_EQ(rig.backend.torque_writes[12] + rig.backend.torque_writes[42], 0);
  }
  FullLegCalibrationRequest defaults{};
  CHECK(defaults.auxiliary_required);  // fail-closed default
}

void test_parked_accessor_tracks_only_the_max_probe() {
  g_case = "auxiliaryParked(): only while the LF/RF MAX probe runs; never for RH/LH";
  for (const Leg leg : kAllLegs) {
    LegRig rig(fixtureFor(leg));
    rig.placeStops(kHardwareStopBeyondContact, kHardwareStopBeyondContact);
    bool ever_parked = false;
    runLeg(rig, [&](LegRig& r, uint32_t) {
      if (r.full.auxiliaryParked()) {
        ever_parked = true;
        CHECK((int)r.full.status().phase == (int)FullLegCalibrationPhase::UPPER_MAX_PROBE);
      }
    });
    CHECK_EQ(ever_parked, rig.f.aux_required);
    CHECK(!rig.full.auxiliaryParked());
  }
}

}  // namespace

int main() {
  test_every_leg_completes_at_the_physical_stops();
  test_aux_park_is_exact_and_only_during_max();
  test_min_side_failure_never_reaches_aux_or_max();
  test_max_side_failure_after_park_needs_dual_safe_off();
  test_aux_move_stall_needs_dual_safe_off();
  test_aux_torque_uncertain_needs_dual_safe_off();
  test_dynamic_prerequisite_loss_stops_every_phase();
  test_operator_abort_min_and_max();
  test_start_validation();
  test_parked_accessor_tracks_only_the_max_probe();

  std::printf("test_full_leg_calibration_executor: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
