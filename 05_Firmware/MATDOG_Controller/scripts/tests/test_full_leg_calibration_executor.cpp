// Offline tests for the 24-contact Full Calibration sequence executor
// (src/calibration/FullLegCalibrationExecutor.*): the LF V25 hardware-oracle
// state machine generalized to LF, RF, RH and LH, run END TO END.
//
// Links the REAL SafeActuatorPolicy (bound to the real Geometry V5 profile AND
// the real geometry-validated CalibrationSequencePlanData.h), ActuatorRuntime,
// CalibrationExecutionEngine, ContactProbeEngine, arbiter and the real
// resolveFullLegPlan() against a TWELVE-joint kinematic model
// (kinematic_servo_sim.h), and ticks the executor exactly the way
// Controller::updateFullLegCalibration() does:
//
//   bootstrap context refreshed from the executor at the start of the tick
//   -> calibration telemetry for exactly the buses telemetryRequest() names
//   -> update() (at most one backend write)
//   -> the V25 phase report, refused when the order is illegal
//   -> the independent SAFE_OFF of exactly the buses safeOffRequest() names.
//
// The model is deliberately hostile where the real servo is: TorqueEnable
// drives to whatever GoalPosition already holds (every joint starts with a
// STALE goal register far from its pose and the EEPROM TorqueLimit 1000), a
// position-controlled joint settles 4-5 ticks short of its goal, and stops,
// plateaus, slow zones, current, temperature, status, register changes and
// read failures are injectable per joint.
//
// The bus ids / units / q0 are this test's own ORACLE, deliberately literal
// (config/MATDOG_SERVO_ALLOCATION.yaml and the CR2-C q0 capture).
//
// NO HARDWARE VALIDATION, no fabricated measurement: every stop here is
// synthetic, placed from the V25 LF hardware contacts' offsets to the URDF
// limits (README of the LF V25 oracle archive), never from a new run.

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationSequencePlanData.h"
#include "../../src/calibration/FullLegCalibrationExecutor.h"
#include "../../src/calibration/FullLegCalibrationPlan.h"
#include "../../src/calibration/ThermalConfirmation.h"
#include "../../src/calibration/StartupRecoveryReference.h"
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

constexpr uint8_t kHip = static_cast<uint8_t>(JointKind::HIP);
constexpr uint8_t kUpper = static_cast<uint8_t>(JointKind::UPPER);
constexpr uint8_t kLower = static_cast<uint8_t>(JointKind::LOWER);
constexpr uint8_t kMin = static_cast<uint8_t>(ContactSide::MIN_SIDE);
constexpr uint8_t kMax = static_cast<uint8_t>(ContactSide::MAX_SIDE);

struct JointOracle {
  Leg leg;
  JointKind joint;
  uint8_t bus;
  const char* unit;
  uint16_t q0;
};

// config/MATDOG_SERVO_ALLOCATION.yaml + the CR2-C q0 capture, literal.
constexpr JointOracle kOracle[12] = {
    {Leg::LF, JointKind::LOWER, 11, "M33", 2087},   {Leg::LF, JointKind::UPPER, 12, "ELR01", 2100},
    {Leg::LF, JointKind::HIP, 13, "M22", 1996},     {Leg::RF, JointKind::LOWER, 21, "NEW03", 1985},
    {Leg::RF, JointKind::UPPER, 22, "ELR03", 2092}, {Leg::RF, JointKind::HIP, 23, "NEW01", 2030},
    {Leg::RH, JointKind::LOWER, 31, "NEW05", 2034}, {Leg::RH, JointKind::UPPER, 32, "ELR02", 2042},
    {Leg::RH, JointKind::HIP, 33, "NEW06", 2081},   {Leg::LH, JointKind::LOWER, 41, "M41", 2073},
    {Leg::LH, JointKind::UPPER, 42, "M42", 2089},   {Leg::LH, JointKind::HIP, 43, "M43", 2035},
};
constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
constexpr uint16_t kStaleGoal = 3000;  // what GoalPosition holds before anyone primes it

const JointOracle& oracleFor(Leg leg, JointKind joint) {
  for (const JointOracle& o : kOracle) {
    if (o.leg == leg && o.joint == joint) return o;
  }
  return kOracle[0];
}

JointIdentity identityOf(const JointOracle& o) {
  JointIdentity id{};
  id.leg = o.leg;
  id.joint = o.joint;
  setPhysicalUnit(&id, o.unit);
  return id;
}

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&actuator::geometry_data::kProvenance, actuator::geometry_data::kJoints,
              actuator::geometry_data::kJointCount, actuator::geometry_data::kEndpoints,
              actuator::geometry_data::kEndpointCount);
  return profile;
}

actuator::JointTransform promotedTransform(const JointIdentity& id, uint16_t q0) {
  actuator::JointTransform t{};
  t.identity = id;
  t.geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.q0_tick = q0;
  t.present = true;
  return t;
}

// Controller::begin()'s Full-Leg backoff deadman, figure for figure.
MotionDeadmanConfig controllerBackoff() {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = kSearchTelemetryTimeoutMs;  // Controller::begin()
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = kSearchBackoffSettleToleranceTicks;
  c.nominal_travel_ticks_per_s = kSearchMinExpectedTicksPerSecond;
  return c;
}

// The stop of each probed side, as an offset from that side's URDF limit
// along the probe direction (+ = beyond the limit). The V25 LF hardware
// contacts (oracle README) against the same URDF: HIP -42.803/+39.375 deg
// (-487/+448 ticks vs +-512), UPPER -53.525/+122.607 (-609/+1395 vs
// -597/+1394), LOWER -91.846/+34.277 (-1045/+390 vs -1047/+427).
struct StopSpec {
  bool present[3][2] = {{true, true}, {true, true}, {true, true}};
  int beyond_limit[3][2] = {{-25, -54}, {12, 1}, {-2, -37}};  // [HIP|UPPER|LOWER][MIN|MAX]
};
StopSpec v25LfExact() {
  StopSpec s;
  s.beyond_limit[kHip][kMax] = -64;  // +448: exactly the corridor entry, as V25 measured
  return s;
}

enum class Ev : uint8_t { GOAL, LIMIT, TORQUE, SAFE_OFF };
struct Event {
  Ev kind;
  uint8_t bus;
  uint16_t tick;
  CalibrationPhase phase;
  int position;
  bool torque_before;
};

class SeqBackend : public simk::SimBackend {
 public:
  BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    log(Ev::TORQUE, bus_id, 0);
    if (bus_id == uncertain_torque_bus) {
      joint[bus_id].torque = true;  // uncertain: assume the worst
      return BackendWriteOutcome::UNCERTAIN;
    }
    return simk::SimBackend::enableTorque(bus_id);
  }
  BackendWriteOutcome writeCalibrationTorqueLimit(uint8_t bus_id) override {
    log(Ev::LIMIT, bus_id, 0);
    if (bus_id == reject_limit_bus) return BackendWriteOutcome::VERIFIED_NOT_APPLIED;
    if (bus_id == limit_reverts_bus) {
      ++torque_limit_writes[bus_id];
      return BackendWriteOutcome::VERIFIED_APPLIED;  // ...but the register reads 1000 again
    }
    return simk::SimBackend::writeCalibrationTorqueLimit(bus_id);
  }
  BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick,
                                        MotionProfile profile) override {
    log(Ev::GOAL, bus_id, target_tick);
    return simk::SimBackend::writeGoalPosition(bus_id, target_tick, profile);
  }
  void log(Ev kind, uint8_t bus, uint16_t tick) {
    events.push_back(Event{kind, bus, tick,
                           exec != nullptr ? exec->status().phase : CalibrationPhase::PREFLIGHT,
                           joint[bus].position(), joint[bus].torque});
    if (std::getenv("FULL_LEG_TRACE") != nullptr) {
      static const char* names[] = {"GOAL", "LIMIT", "TORQUE", "SAFE_OFF"};
      std::printf("    ev %-8s bus=%u tick=%u phase=%s pos=%d torque=%d\n",
                  names[static_cast<int>(kind)], (unsigned)bus, (unsigned)tick,
                  toString(events.back().phase), joint[bus].position(), (int)joint[bus].torque);
    }
  }

  const FullLegCalibrationExecutor* exec = nullptr;
  std::vector<Event> events;
  uint8_t uncertain_torque_bus = 0;
  uint8_t reject_limit_bus = 0;
  uint8_t limit_reverts_bus = 0;
};

// Controller::ServoThermalReadPort on the model: a DIRECT PresentTemperature
// read returns the joint's true temperature (never a glitched bulk sample).
struct SimThermalPort : ThermalReadPort {
  explicit SimThermalPort(simk::SimBackend* b) : backend(b) {}
  bool readPresentTemperatureDirect(uint8_t bus_id, int32_t* celsius) override {
    ++direct_reads[bus_id];
    if (fail_direct[bus_id] || backend->joint[bus_id].read_fails) return false;
    *celsius = backend->joint[bus_id].temperature;
    return true;
  }
  void delayMs(uint32_t ms) override { waited_ms += ms; }
  simk::SimBackend* backend;
  int direct_reads[256] = {0};
  bool fail_direct[256] = {false};
  uint32_t waited_ms = 0;
};

struct Rig;
using Hook = std::function<void(Rig&)>;

struct Rig {
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  ActuatorRuntime runtime;
  CalibrationExecutionEngine engine;
  SeqBackend backend;
  CalibrationGeometryProfile profile;
  FullLegCalibrationExecutor full;
  AuthorityLease lease{};
  FullLegPlan plan{};
  Leg leg;
  bool plan_ok = false;
  bool permit = true;
  bool session = true;
  bool report_phases = true;  // the Controller does not report a recovery-only run
  uint32_t t = 1000;
  FullLegSafeOffFrame safe_off_frame{};
  bool safe_off_fails[256] = {false};
  int safe_off_calls[256] = {0};
  std::vector<CalibrationPhase> phases;
  bool phase_violation = false;
  bool phase_skipped = false;
  bool reported_any = false;
  CalibrationPhase last_reported = CalibrationPhase::PREFLIGHT;
  uint32_t seen_changes = 0;
  int held_checks[20] = {0};
  int held_violations = 0;
  int bystander_torque_violations = 0;
  int start_pos[256] = {0};
  // Per probe phase, the (pass, stage) of every search write on the probed
  // bus, consecutive repeats folded: the V25 stage order of that endpoint.
  std::vector<std::pair<int, ContactSearchStage>> probe_trace[32];
  // A one-off bulk-telemetry temperature anomaly per bus: the next `count`
  // bulk samples read `value` while the servo is really at its temperature.
  struct BulkGlitch { int value = 0; int count = 0; };
  BulkGlitch bulk_temperature_glitch[256];
  SimThermalPort thermal_port{&backend};
  std::vector<ThermalConfirmation> thermal_log;
  ThermalConfirmationState thermal_state[256];
  // Each endpoint's probe verdict as the probe itself reported it:
  // {scout, fine 1, fine 2}, 0 = not completed.
  uint16_t probe_result[3][2][3] = {};
  // The final partial coarse-scout step each endpoint took (0 = none).
  uint16_t probe_partial[3][2] = {};
  // Held-joint evidence the executor recorded (CALIBRATION_HELD_SPEED_TRANSIENT).
  std::vector<FullLegHeldObservation> transients;
  // Per bus: consecutive samples handed to the executor with |speed| > 40,
  // and the longest such run (the retired D5 condition, as the monitor saw it).
  int fast_run[256] = {0};
  int max_fast_run[256] = {0};

  explicit Rig(Leg l, const StopSpec& stops = StopSpec(), int deadband = 4) : leg(l) {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    policy.bindSequencePlan(&actuator::sequence_plan_data::kPlan);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance);
    FullLegCalibrationConfig config{};
    config.probe_backoff_deadman = controllerBackoff();
    full.begin(&policy, &runtime, &engine, &profile, &actuator::geometry_data::kProvenance, config);
    backend.exec = &full;
    backend.torque_enable_holds_pose = false;  // the real ST3215

    for (const JointOracle& o : kOracle) {
      CHECK(policy.transforms().admit(promotedTransform(identityOf(o), o.q0)));
      simk::SimJoint& j = backend.joint[o.bus];
      j.pos = o.q0 + 3;  // manual placement next to the pose
      j.goal = kStaleGoal;
      j.torque = false;
      j.torque_limit = 1000;  // EEPROM default until the executor's RAM write
      j.deadband_ticks = deadband;
    }
    CHECK(arbiter.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &lease) ==
          AuthorityResult::GRANTED);
    plan_ok = resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, policy.transforms(),
                                 &actuator::sequence_plan_data::kPlan, leg, &plan) ==
              FullLegPlanStatus::OK;
    CHECK(plan_ok);
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      for (uint8_t s = 0; s < kContactSideCount; ++s) {
        if (stops.present[k][s]) placeStop(k, s, stops.beyond_limit[k][s]);
      }
    }
  }

  const FullLegCalibrationRequest& req() const { return plan.request; }
  uint8_t bus(uint8_t k) const { return plan.request.joint[k].bus_id; }
  simk::SimJoint& sim(uint8_t k) { return backend.joint[bus(k)]; }

  void placeStop(uint8_t k, uint8_t side, int beyond_limit) {
    const CalibrationSearchCorridor& c = plan.request.corridor[k][side];
    const int d = actuator::searchDepth(c, c.urdf_limit_tick) + beyond_limit;
    const double stop = c.home_tick + c.probe_sign * d;
    simk::SimJoint& j = backend.joint[bus(k)];
    if (c.probe_sign < 0) {
      j.has_stop_low = true;
      j.stop_low = stop;
    } else {
      j.has_stop_high = true;
      j.stop_high = stop;
    }
  }
  void removeStop(uint8_t k, uint8_t side) {
    const CalibrationSearchCorridor& c = plan.request.corridor[k][side];
    simk::SimJoint& j = backend.joint[bus(k)];
    if (c.probe_sign < 0) {
      j.has_stop_low = false;
    } else {
      j.has_stop_high = false;
    }
  }
  int stopTick(uint8_t k, uint8_t side) const {
    const CalibrationSearchCorridor& c = plan.request.corridor[k][side];
    const simk::SimJoint& j = backend.joint[plan.request.joint[k].bus_id];
    return static_cast<int>(c.probe_sign < 0 ? j.stop_low : j.stop_high);
  }

  FullLegCalibrationContext ctx() const {
    FullLegCalibrationContext c{};
    c.startup_motion_permit=full.request().startup_recovery;
    c.session_active = session;
    c.origin = CalibrationOrigin::LIVE_SESSION;
    c.lease = lease;
    c.mode = OperatingMode::MAINTENANCE;
    c.motion_permit_active = permit;
    c.authority = arbiter.current();
    c.authority_generation = arbiter.generation();
    c.authority_inhibited = arbiter.inhibited();
    return c;
  }

  // Controller::updateCalibrationMotionPermit()'s bootstrap refresh.
  void refreshBootstrap() {
    actuator::CalibrationBootstrapContext b{};
    b.session_active = session;
    b.origin = CalibrationOrigin::LIVE_SESSION;
    b.motion_permit_active = permit;
    b.motion_permit_generation = permit ? 1 : 0;
    b.motion_permit_session_id = 1;
    b.motion_permit_authority_generation = lease.generation;
    b.auxiliary_parked = false;
    b.sequence_active = full.sequenceActive();
    b.startup_recovery_only=full.request().startup_recovery;
    b.sequence_leg = full.leg();
    b.sequence_phase = full.sequencePhase();
    b.sequence_prerequisites_verified = full.prerequisitesVerified();
    full.recoveryGrant(&b);
    policy.setBootstrapContext(b);
  }

  bool isRunJoint(uint8_t b) const {
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      if (plan.request.joint[k].bus_id == b) return true;
    }
    return plan.request.has_rear_park && plan.request.park.bus_id == b;
  }

  // Every tick of a probe: the held set is EXACTLY V25's prerequisites_for(),
  // on the real (modelled) servos - torque on, GoalPosition at the plan pose,
  // present within 10 ticks - and nothing else in the robot is torque-on.
  void auditHeld() {
    JointKind pj = JointKind::UPPER;
    ContactSide ps = ContactSide::MIN_SIDE;
    const CalibrationPhase phase = full.status().phase;
    if (!actuator::sequenceProbeEndpoint(phase, &pj, &ps)) return;
    if (full.status().step != FullLegStep::PROBE) return;
    const ContactProbePhase pp = full.probeStatus().phase;
    if (pp != ContactProbePhase::BASELINE_PENDING && pp != ContactProbePhase::BASELINE_MONITORING &&
        pp != ContactProbePhase::STEP_PENDING && pp != ContactProbePhase::STEP_MONITORING &&
        pp != ContactProbePhase::RELEASE_PENDING && pp != ContactProbePhase::RELEASE_VERIFYING &&
        pp != ContactProbePhase::BACKOFF_PENDING && pp != ContactProbePhase::BACKOFF_MONITORING) {
      return;
    }
    const FullLegCalibrationRequest& r = plan.request;
    auto expect = [&](uint8_t b, uint16_t goal) {
      const simk::SimJoint& j = backend.joint[b];
      if (!j.torque || j.goal != goal || std::abs(j.position() - goal) > 10) ++held_violations;
    };
    switch (pj) {
      case JointKind::UPPER:
        expect(bus(kHip), r.joint[kHip].q0_tick);
        expect(bus(kLower), r.joint[kLower].q0_tick);
        break;
      case JointKind::LOWER:
        expect(bus(kHip), r.joint[kHip].q0_tick);
        expect(bus(kUpper), r.upper_for_lower_tick);
        break;
      case JointKind::HIP:
        expect(bus(kUpper), ps == ContactSide::MIN_SIDE ? r.upper_for_hip_min_tick
                                                        : r.upper_for_hip_max_tick);
        expect(bus(kLower), r.lower_folded_tick);
        break;
    }
    if (r.has_rear_park) expect(r.park.bus_id, r.park_target_tick);
    if (!backend.joint[bus(static_cast<uint8_t>(pj))].torque) ++held_violations;
    for (const JointOracle& o : kOracle) {
      if (!isRunJoint(o.bus) && backend.joint[o.bus].torque) ++bystander_torque_violations;
    }
    ++held_checks[static_cast<uint8_t>(phase)];
  }

  // Controller::updateFullLegCalibration(), step for step.
  void tick(const Hook& hook) {
    t += 10;
    backend.advance(t, 10);
    if (hook) hook(*this);
    refreshBootstrap();
    uint8_t buses[kFullLegMaxTelemetry] = {0};
    const uint8_t n = full.telemetryRequest(buses, kFullLegMaxTelemetry);
    FullLegTelemetryFrame frame{};
    bool thermal_pending=false, direct_used=false;
    for (uint8_t i = 0; i < n; ++i) {
      actuator::TelemetrySample sample = backend.joint[buses[i]].sample(t);
      BulkGlitch& g = bulk_temperature_glitch[buses[i]];
      if (g.count > 0 && sample.read_ok) {
        sample.present_temperature = g.value;
        --g.count;
      }
      if (sample.read_ok) {
        auto& state=thermal_state[buses[i]];
        const bool due=state.directReadDue(t);
        const ThermalConfirmation th=due && direct_used && !state.expired(t) ? state.result()
            : state.update(&thermal_port,buses[i],sample.present_temperature,t);
        if(due) direct_used=true;
        thermal_pending |= th.decision==ThermalDecision::PENDING;
        sample.present_temperature=th.decision==ThermalDecision::PENDING ? 70 : th.published_c;
        if(th.decision!=ThermalDecision::NORMAL && th.decision!=ThermalDecision::PENDING) thermal_log.push_back(th);
      }
      frame.add(buses[i], sample);
      const bool fast = sample.read_ok && sample.present_speed >= 0 &&
                        (sample.present_speed & 0x7FFF) > 40;
      fast_run[buses[i]] = fast ? fast_run[buses[i]] + 1 : 0;
      max_fast_run[buses[i]] = std::max(max_fast_run[buses[i]], fast_run[buses[i]]);
    }
    const size_t writes_before = backend.writes.size();
    if(thermal_pending) {
      full.monitorOnly(ctx(),t,frame);
      CHECK(backend.writes.size()==writes_before); // no goal/torque write pending classification
    } else full.update(ctx(), t, frame, safe_off_frame);
    for (uint8_t i = 0; i < full.heldSpeedTransientsThisTick(); ++i) {
      transients.push_back(full.heldSpeedTransient(i));
    }
    if (full.status().step == FullLegStep::PROBE) {
      const uint8_t ph = static_cast<uint8_t>(full.status().phase);
      for (size_t i = writes_before; i < backend.writes.size() && ph < 32; ++i) {
        if (backend.writes[i].bus != full.probeRequest().bus_id) continue;
        const std::pair<int, ContactSearchStage> e{full.probeStatus().pass, full.probeStatus().stage};
        if (probe_trace[ph].empty() || probe_trace[ph].back() != e) probe_trace[ph].push_back(e);
      }
    }
    if (full.probeStatus().phase == ContactProbePhase::COMPLETE) {
      const ContactProbeRequest& pr = full.probeRequest();
      uint16_t* r = probe_result[static_cast<uint8_t>(pr.endpoint_joint)][static_cast<uint8_t>(pr.endpoint_side)];
      r[0] = full.probeStatus().scout_tick;
      r[1] = full.probeStatus().pass1_contact_tick;
      r[2] = full.probeStatus().pass2_contact_tick;
      probe_partial[static_cast<uint8_t>(pr.endpoint_joint)][static_cast<uint8_t>(pr.endpoint_side)] =
          full.probeStatus().scout_partial_step_ticks;
    }

    const uint32_t changes = full.status().phase_changes;
    if (changes != seen_changes) {
      if (changes - seen_changes > 1) phase_skipped = true;  // a phase never reported
      seen_changes = changes;
      const CalibrationPhase p = full.status().phase;
      phases.push_back(p);
      if (report_phases) {
        const bool legal = reported_any ? isLegalPhaseTransition(last_reported, p)
                                        : (p == CalibrationPhase::PREFLIGHT ||
                                           isLegalPhaseTransition(CalibrationPhase::PREFLIGHT, p));
        if (!legal) {
          phase_violation = true;
          full.phaseReportRejected();
        } else {
          last_reported = p;
          reported_any = true;
        }
      }
    }
    auditHeld();

    uint8_t off[kFullLegPopulation] = {0};
    const uint8_t m = full.safeOffRequest(off, kFullLegPopulation);
    safe_off_frame = FullLegSafeOffFrame{};
    for (uint8_t i = 0; i < m; ++i) {
      const uint8_t b = off[i];
      ++safe_off_calls[b];
      backend.log(Ev::SAFE_OFF, b, 0);
      if (!safe_off_fails[b]) backend.joint[b].torque = false;
      safe_off_frame.add(b, !safe_off_fails[b]);
    }
  }

  bool start() {
    for (const JointOracle& o : kOracle) start_pos[o.bus] = backend.joint[o.bus].position();
    refreshBootstrap();
    return full.start(plan.request, ctx(), t);
  }

  uint32_t run(const Hook& hook = nullptr, uint32_t limit_ms = 1500000) {
    const uint32_t begin = t;
    CHECK(start());
    while (full.active() && t - begin < limit_ms) tick(hook);
    CHECK(!full.active());
    return t - begin;
  }

  int torqueOnCount() const {
    int n = 0;
    for (const JointOracle& o : kOracle) n += backend.joint[o.bus].torque ? 1 : 0;
    return n;
  }
};

// A hook that fires once, the first tick `pred` holds.
Hook once(std::function<bool(Rig&)> pred, std::function<void(Rig&)> act) {
  auto fired = std::make_shared<bool>(false);
  return [=](Rig& r) {
    if (*fired || !pred(r)) return;
    *fired = true;
    act(r);
  };
}
// Probing `phase`, at least `steps` search steps in.
std::function<bool(Rig&)> probing(CalibrationPhase phase, uint16_t steps = 3) {
  return [=](Rig& r) {
    return r.full.status().phase == phase && r.full.status().step == FullLegStep::PROBE &&
           r.full.probeStatus().phase == ContactProbePhase::STEP_MONITORING &&
           r.full.probeStatus().step_count >= steps;
  };
}
std::function<bool(Rig&)> inPhaseStep(CalibrationPhase phase, FullLegStep step) {
  return [=](Rig& r) { return r.full.status().phase == phase && r.full.status().step == step; };
}

const std::vector<CalibrationPhase>& v25Order() {
  static const std::vector<CalibrationPhase> order = {
      CalibrationPhase::PREFLIGHT,        CalibrationPhase::INITIAL_RECOVERY,
      CalibrationPhase::PARKING,          CalibrationPhase::UPPER_MIN,
      CalibrationPhase::UPPER_MAX,        CalibrationPhase::UPPER_HORIZONTAL,
      CalibrationPhase::LOWER_MIN,        CalibrationPhase::LOWER_MAX,
      CalibrationPhase::LOWER_FOLDED,     CalibrationPhase::HIP_MIN,
      CalibrationPhase::HIP_MAX,          CalibrationPhase::DIAGNOSTICS,
      CalibrationPhase::RETURN_HIP,       CalibrationPhase::RETURN_LOWER_HELD,
      CalibrationPhase::RETURN_UPPER,     CalibrationPhase::RESTORE_PARKING,
      CalibrationPhase::CLEANUP,          CalibrationPhase::TORQUE_OFF,
  };
  return order;
}

// Every write, on every bus, obeys V25 prepare_motor(): after any SAFE_OFF
// (and at the start) a joint is torque-enabled only after GoalPosition was
// primed to its PRESENT position and the RAM TorqueLimit was written, in that
// order; nothing ever reaches bus 0; every goal write is the calibration
// profile; outside INITIAL_RECOVERY only the leg and its park joint are
// written; nothing ever moved toward the stale goal register.
void checkWriteDiscipline(Rig& rig) {
  enum { COLD, PRIMED, LIMITED, ON };
  int state[256];
  for (int& s : state) s = COLD;
  int violations = 0, foreign = 0, bus_zero = 0;
  for (const Event& e : rig.backend.events) {
    if (e.bus == 0) ++bus_zero;
    if (e.phase != CalibrationPhase::INITIAL_RECOVERY && e.kind != Ev::SAFE_OFF &&
        !rig.isRunJoint(e.bus)) {
      ++foreign;
    }
    int& s = state[e.bus];
    switch (e.kind) {
      case Ev::SAFE_OFF:
        s = COLD;
        break;
      case Ev::GOAL:
        if (s == COLD) {
          if (e.tick != e.position || e.torque_before) ++violations;  // prime == present, torque off
          s = PRIMED;
        } else if (s != ON) {
          ++violations;  // a second goal before torque-on
        }
        break;
      case Ev::LIMIT:
        if (s != PRIMED) ++violations;
        s = LIMITED;
        break;
      case Ev::TORQUE:
        if (s != LIMITED) ++violations;
        s = ON;
        break;
    }
  }
  CHECK_EQ(violations, 0);
  CHECK_EQ(foreign, 0);
  CHECK_EQ(bus_zero, 0);
  for (const simk::GoalWrite& w : rig.backend.writes) CHECK(w.profile == MotionProfile::CALIBRATION_SEARCH);
  // The stale goal (3000) is > 800 ticks from every q0: an unprimed torque-on
  // of a joint that only ever goes back to q0 would have dragged it far out of
  // this window. (The run's own joints travel their whole range legitimately;
  // for them the prime-before-torque ordering above is the proof.)
  for (const JointOracle& o : kOracle) {
    const simk::SimJoint& j = rig.backend.joint[o.bus];
    if (!rig.isRunJoint(o.bus)) {
      const int lo = std::min<int>(o.q0, rig.start_pos[o.bus]) - 12;
      const int hi = std::max<int>(o.q0, rig.start_pos[o.bus]) + 12;
      CHECK((j.lo_seen > 4095 || j.lo_seen >= lo) && (j.hi_seen < 0 || j.hi_seen <= hi));
    }
  }
}

// Whatever happened, the run ends with every leg joint of the robot SAFE_OFF
// verified and torque-off, and the executor no longer active.
void checkSafeEnd(Rig& rig) {
  CHECK(!rig.full.active());
  CHECK_EQ(rig.torqueOnCount(), 0);
  for (const JointOracle& o : kOracle) CHECK(rig.safe_off_calls[o.bus] >= 1);
  CHECK(!rig.full.prerequisitesVerified());
  CHECK(!rig.full.sequenceActive());
  checkWriteDiscipline(rig);
}

// INITIAL_RECOVERY commanded EVERY leg joint of the robot: one prime, one
// TorqueLimit, one TorqueEnable, one q0 goal and a SAFE_OFF each, in order.
void checkRecoveredAll(Rig& rig) {
  CHECK_EQ(rig.full.status().recovered_joints, 12);
  for (const JointOracle& o : kOracle) {
    int goal = 0, lim = 0, torque = 0, off = 0, q0_goal = 0;
    for (const Event& e : rig.backend.events) {
      if (e.bus != o.bus || e.phase != CalibrationPhase::INITIAL_RECOVERY) continue;
      goal += e.kind == Ev::GOAL;
      lim += e.kind == Ev::LIMIT;
      torque += e.kind == Ev::TORQUE;
      off += e.kind == Ev::SAFE_OFF;
      q0_goal += (e.kind == Ev::GOAL && e.tick == o.q0);
    }
    CHECK_EQ(goal, 2);  // prime + the q0 move
    CHECK_EQ(lim, 1);
    CHECK_EQ(torque, 1);
    CHECK(q0_goal >= 1);
    CHECK(off >= 1);
  }
}

// V25 measure_lf_contact_side_efficient(), for every endpoint of the leg: the
// one generic ContactProbeEngine ran baseline, coarse scout (its free-space
// part labelled transit), release, backoff, fine 1, release, backoff, fine 2,
// release - nothing else, in that order.
bool isV25SearchTrace(const std::vector<std::pair<int, ContactSearchStage>>& got) {
  using S = ContactSearchStage;
  std::vector<std::pair<int, S>> full = {
      {0, S::BASELINE}, {0, S::COARSE_TRANSIT}, {0, S::COARSE_SCOUT}, {0, S::RELEASE},
      {0, S::BACKOFF},  {1, S::FINE_SEARCH},    {1, S::RELEASE},      {1, S::BACKOFF},
      {2, S::FINE_SEARCH}, {2, S::RELEASE}};
  if (got == full) return true;
  full.erase(full.begin() + 1);
  return got == full;
}

void checkComplete(Rig& rig) {
  const FullLegCalibrationStatus& s = rig.full.status();
  CHECK_EQ((int)s.step, (int)FullLegStep::COMPLETE);
  CHECK_EQ((int)s.failure, (int)FullLegFailure::NONE);
  CHECK_EQ(s.contacts_accepted, 6);
  CHECK(rig.full.diagnosticsAccepted());
  CHECK(rig.phases == v25Order());
  CHECK(!rig.phase_violation);
  CHECK(!rig.phase_skipped);
  CHECK_EQ(rig.held_violations, 0);
  CHECK_EQ(rig.bystander_torque_violations, 0);
  for (const CalibrationPhase p : {CalibrationPhase::UPPER_MIN, CalibrationPhase::UPPER_MAX,
                                   CalibrationPhase::LOWER_MIN, CalibrationPhase::LOWER_MAX,
                                   CalibrationPhase::HIP_MIN, CalibrationPhase::HIP_MAX}) {
    CHECK(rig.held_checks[static_cast<uint8_t>(p)] > 0);
  }
  // Six contacts, each at its physical stop: the coarse scout (reference
  // evidence) and two repeatable fine passes (metrology).
  for (const CalibrationPhase p : {CalibrationPhase::UPPER_MIN, CalibrationPhase::UPPER_MAX,
                                   CalibrationPhase::LOWER_MIN, CalibrationPhase::LOWER_MAX,
                                   CalibrationPhase::HIP_MIN, CalibrationPhase::HIP_MAX}) {
    const bool v25 = isV25SearchTrace(rig.probe_trace[static_cast<uint8_t>(p)]);
    if (!v25) std::printf("    non-V25 search trace in %s\n", toString(p));
    CHECK(v25);
  }
  for (uint8_t k = 0; k < kJointKindCount; ++k) {
    for (uint8_t side = 0; side < kContactSideCount; ++side) {
      const ContactEvidence& e = rig.full.contact(static_cast<JointKind>(k), static_cast<ContactSide>(side));
      CHECK(e.has_measurement);
      CHECK(e.witness.accepted());
      CHECK((int)e.key.leg == (int)rig.leg);
      CHECK((int)e.key.joint == (int)k);
      CHECK((int)e.key.side == (int)side);
      const int stop = rig.stopTick(k, side);
      // Recorded exactly as the one generic probe measured it: the scout as
      // coarse_tick (reference), the fine passes as fine_tick_1/2.
      CHECK_EQ(e.coarse_tick, rig.probe_result[k][side][0]);
      CHECK_EQ(e.fine_tick_1, rig.probe_result[k][side][1]);
      CHECK_EQ(e.fine_tick_2, rig.probe_result[k][side][2]);
      CHECK(std::abs((int)e.coarse_tick - stop) <= 2);  // the scout
      CHECK(std::abs((int)e.fine_tick_1 - stop) <= 2);
      CHECK(std::abs((int)e.fine_tick_2 - stop) <= 2);
      CHECK_EQ(e.repeatability_ticks, std::abs((int)e.fine_tick_1 - (int)e.fine_tick_2));
      CHECK(actuator::searchCorridorAccepts(rig.req().corridor[k][side], e.coarse_tick));
      const FullLegJointDiagnostics& d = rig.full.diagnostics(static_cast<JointKind>(k));
      CHECK(d.evaluated && d.accepted && d.ordered);
    }
  }
  // Back at rest near q0: the leg within the V25 home tolerance, every other
  // joint where the session left it.
  for (const JointOracle& o : kOracle) {
    const simk::SimJoint& j = rig.backend.joint[o.bus];
    CHECK(!j.torque);
    CHECK(std::abs(j.position() - o.q0) <= (int)kSequenceRestToleranceTicks);
  }
  checkRecoveredAll(rig);
  checkSafeEnd(rig);
}

// --- the full sequence, all four legs -------------------------------------------

void test_every_leg_completes_the_full_v25_sequence() {
  for (const Leg leg : kAllLegs) {
    g_case = "full V25 sequence, 6/6 contacts";
    std::printf("  leg %s\n", toString(leg));
    Rig rig(leg);
    CHECK(rig.plan_ok);
    const uint32_t elapsed = rig.run();
    checkComplete(rig);
    // A real run on V25's envelope: minutes, not hours.
    CHECK(elapsed < 900000);
    std::printf("    elapsed=%u ms writes=%u\n", (unsigned)elapsed, (unsigned)rig.backend.events.size());
  }
}

void test_v25_lf_hardware_contacts_replayed_exactly() {
  g_case = "V25 LF hardware contact set replayed";
  Rig rig(Leg::LF, v25LfExact());
  rig.run();
  checkComplete(rig);
  // The V25 LF HIP MAX contact (+448 from q0) lies EXACTLY on the corridor
  // entry: accepted, with zero margin - the known hardware risk this suite
  // pins (see the next case).
  const CalibrationSearchCorridor& c = rig.req().corridor[kHip][kMax];
  CHECK_EQ(actuator::searchDepth(c, c.entry_tick), 448);
  CHECK_EQ(actuator::searchDepth(c, rig.full.contact(JointKind::HIP, ContactSide::MAX_SIDE).coarse_tick), 448);
}

// [T3,T4,T10] The evidence fields are the probe's own three measurements, each
// in its place - checked where they differ: the scout meets a transient
// obstruction 20 ticks short, fine 2 finds the stop 5 ticks deeper than fine 1.
void test_evidence_maps_scout_and_fine_passes() {
  g_case = "evidence: coarse_tick = scout, fine_tick_1/2 = the fine passes, diagnostics fine-only";
  Rig rig(Leg::LF);
  const CalibrationSearchCorridor c = rig.req().corridor[kUpper][kMin];
  const int stop0 = rig.stopTick(kUpper, kMin);
  const int shallow = stop0 - c.probe_sign * 20;
  auto armed = std::make_shared<int>(0);
  rig.run([&, armed](Rig& r) {
    if (r.full.status().phase != CalibrationPhase::UPPER_MIN || r.full.status().step != FullLegStep::PROBE) return;
    const ContactProbeStatus& st = r.full.probeStatus();
    simk::SimJoint& j = r.sim(kUpper);
    if (*armed == 0 && st.pass == 0 && st.phase == ContactProbePhase::STEP_MONITORING) {
      *armed = 1;
      j.plateau = true;
      j.plateau_tick = shallow;
      j.plateau_breakaway = 1000;
    } else if (*armed == 1 && st.scout_valid && st.phase == ContactProbePhase::BACKOFF_MONITORING) {
      *armed = 2;
      j.plateau = false;
    } else if (*armed == 2 && st.pass == 2) {
      *armed = 3;
      if (c.probe_sign < 0) j.stop_low -= 5; else j.stop_high += 5;
    }
  });
  CHECK_EQ(*armed, 3);
  CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::COMPLETE);
  const ContactEvidence& e = rig.full.contact(JointKind::UPPER, ContactSide::MIN_SIDE);
  CHECK_EQ(e.coarse_tick, rig.probe_result[kUpper][kMin][0]);
  CHECK_EQ(e.fine_tick_1, rig.probe_result[kUpper][kMin][1]);
  CHECK_EQ(e.fine_tick_2, rig.probe_result[kUpper][kMin][2]);
  CHECK(std::abs((int)e.coarse_tick - shallow) <= 2);
  CHECK(std::abs((int)e.fine_tick_1 - stop0) <= 2);
  CHECK(std::abs((int)e.fine_tick_2 - (stop0 + c.probe_sign * 5)) <= 2);
  CHECK_EQ(e.repeatability_ticks, std::abs((int)e.fine_tick_1 - (int)e.fine_tick_2));
  CHECK(e.witness.accepted());
  const FullLegJointDiagnostics& d = rig.full.diagnostics(JointKind::UPPER);
  CHECK_EQ(d.min_contact_tick, ((int)e.fine_tick_1 + (int)e.fine_tick_2) / 2);
  checkSafeEnd(rig);
}

// The final partial coarse-scout step, all 24 endpoints, both directions: a
// stop at guard - 18 (URDF + 46, the phase-independent reach) is found on
// every endpoint of every leg. Wherever the 64-tick grid would have ended short
// of it, the one partial step to the guard is what reaches it.
void test_partial_scout_step_all_24_endpoints() {
  int partial_used = 0;
  for (const Leg leg : kAllLegs) {
    g_case = "partial scout step: stops at guard - 18 on all 6 endpoints, every leg";
    StopSpec s;
    for (auto& joint : s.beyond_limit) for (int& b : joint) b = 64 - 18;
    Rig rig(leg, s);
    rig.run();
    checkComplete(rig);
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      for (uint8_t side = 0; side < kContactSideCount; ++side) {
        const uint16_t partial = rig.probe_partial[k][side];
        CHECK(partial < 64);
        partial_used += partial > 0;
        const CalibrationSearchCorridor& c = rig.req().corridor[k][side];
        const ContactEvidence& e = rig.full.contact(static_cast<JointKind>(k), static_cast<ContactSide>(side));
        CHECK_EQ(actuator::searchDepth(c, c.guard_tick) - actuator::searchDepth(c, e.coarse_tick), 18);
      }
    }
  }
  std::printf("    partial scout step used on %d of 24 endpoints\n", partial_used);
  CHECK(partial_used > 0);
}

// The LF V25 runtime PresentTemperature confirmation, through the real
// Controller-equivalent path: one bulk sample > 70 C with the servo really at
// 32 C is a transient (the run continues); a real overheat, or an over-limit
// sample that cannot be confirmed, still aborts to SAFE_OFF.
void test_thermal_confirmation_in_the_sequence() {
  auto parking_move = [](Rig& r) {
    return r.full.status().phase == CalibrationPhase::PARKING &&
           r.full.status().step == FullLegStep::MOVE_MONITOR;
  };
  {
    g_case = "2026-09-30 M42: one > 70 C sample while parking, 32 C direct -> transient, 6/6";
    Rig rig(Leg::LF);
    const uint8_t park = rig.req().park.bus_id;
    CHECK_EQ(park, 42);
    rig.run(once(parking_move, [park](Rig& r) { r.bulk_temperature_glitch[park] = {255, 1}; }));
    checkComplete(rig);
    CHECK_EQ(rig.thermal_log.size(), 1u);
    if (!rig.thermal_log.empty()) {
      const ThermalConfirmation& th = rig.thermal_log.front();
      CHECK(th.decision == ThermalDecision::TRANSIENT);
      CHECK_EQ(th.bus_id, park);
      CHECK_EQ(th.samples[0], 255);
      CHECK_EQ(th.published_c, 35);
    }
    CHECK_EQ(rig.thermal_port.direct_reads[park], 3);
    CHECK_EQ(rig.thermal_port.waited_ms, 0u);
  }
  {
    g_case = "a one-sample transient on the PROBED joint mid-search -> transient, 6/6";
    Rig rig(Leg::RF);
    rig.run(once(probing(CalibrationPhase::UPPER_MIN, 4),
                 [](Rig& r) { r.bulk_temperature_glitch[r.bus(kUpper)] = {90, 1}; }));
    checkComplete(rig);
    CHECK_EQ(rig.thermal_log.size(), 1u);
  }
  {
    g_case = "a real overheat of the park joint (bulk AND direct 75 C) -> CONFIRMED, abort";
    Rig rig(Leg::LF);
    const uint8_t park = rig.req().park.bus_id;
    rig.run(once(parking_move, [park](Rig& r) { r.backend.joint[park].temperature = 75; }));
    CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::FAILED);
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::OVER_TEMPERATURE);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::PARKING);
    CHECK(!rig.thermal_log.empty() && rig.thermal_log.front().decision == ThermalDecision::CONFIRMED);
    rig.backend.joint[park].temperature = 35;
    checkSafeEnd(rig);
  }
  {
    g_case = "an over-limit sample whose confirmation read fails -> abort (fail closed)";
    Rig rig(Leg::LF);
    const uint8_t park = rig.req().park.bus_id;
    rig.run(once(parking_move, [park](Rig& r) {
      r.bulk_temperature_glitch[park] = {255, 1};
      r.thermal_port.fail_direct[park] = true;
    }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::OVER_TEMPERATURE);
    CHECK(!rig.thermal_log.empty() &&
          rig.thermal_log.front().decision == ThermalDecision::CONFIRMATION_READ_FAILED);
    rig.thermal_port.fail_direct[park] = false;
    checkSafeEnd(rig);
  }
  {
    g_case = "no over-limit sample: not one confirmation read in a whole 6/6 run";
    Rig rig(Leg::RH);
    rig.run();
    checkComplete(rig);
    CHECK(rig.thermal_log.empty());
    int reads = 0;
    for (int b = 0; b < 256; ++b) reads += rig.thermal_port.direct_reads[b];
    CHECK_EQ(reads, 0);
    CHECK_EQ(rig.thermal_port.waited_ms, 0u);
  }
  for (int fault=0; fault<3; ++fault) {
    g_case="thermal verdict pending: active readback/current/UART still fail closed without new goals";
    Rig rig(Leg::RF); bool triggered=false, injected=false;
    rig.run([&](Rig& r) {
      if (!triggered && parking_move(r)) {
        r.bulk_temperature_glitch[r.req().park.bus_id]={95,1}; triggered=true;
      } else if (triggered && !injected && r.thermal_state[r.req().park.bus_id].pending()) {
        if(fault==0) r.backend.joint[r.req().park.bus_id].goal+=30;
        if(fault==1) r.backend.joint[r.req().park.bus_id].current_override=200;
        if(fault==2) r.backend.joint[r.req().park.bus_id].read_fails=true;
        injected=true;
      }
    });
    CHECK(triggered && injected);
    const FullLegFailure expect[]={FullLegFailure::MOVE_READBACK,FullLegFailure::HARD_CURRENT_ABORT,
                                   FullLegFailure::STALE_TELEMETRY};
    CHECK(rig.full.status().failure==expect[fault]);
    rig.backend.joint[rig.req().park.bus_id].read_fails=false; checkSafeEnd(rig);
  }
}

void test_stop_one_tick_before_the_corridor_entry_fails_closed() {
  g_case = "HIP MAX stop one tick before the entry: EARLY stall, 5/6, FAILED";
  StopSpec s = v25LfExact();
  s.beyond_limit[kHip][kMax] = -65;
  Rig rig(Leg::LF, s);
  rig.run();
  CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::FAILED);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::HIP_MAX_PROBE_FAILED);
  CHECK_EQ((int)rig.full.probeStatus().failure, (int)ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR);
  CHECK_EQ(rig.full.status().contacts_accepted, 5);
  checkSafeEnd(rig);
}

void test_realistic_servo_behaviour_still_completes() {
  {
    g_case = "5-tick tracking shortfall, every joint";
    Rig rig(Leg::RF, StopSpec(), 5);
    rig.run();
    checkComplete(rig);
  }
  {
    g_case = "friction plateau inside the UPPER MIN corridor before the stop";
    Rig rig(Leg::RH);
    simk::SimJoint& j = rig.sim(kUpper);
    const CalibrationSearchCorridor& c = rig.req().corridor[kUpper][kMin];
    j.plateau = true;
    j.plateau_tick = rig.stopTick(kUpper, kMin) - c.probe_sign * 20;
    j.plateau_breakaway = 12;
    rig.run();
    checkComplete(rig);
    CHECK(rig.full.contact(JointKind::UPPER, ContactSide::MIN_SIDE).coarse_tick != (uint16_t)j.plateau_tick);
  }
  {
    g_case = "temporary slowdown during the LOWER transit";
    Rig rig(Leg::LH);
    simk::SimJoint& j = rig.sim(kLower);
    const CalibrationSearchCorridor& c = rig.req().corridor[kLower][kMin];
    j.slow_zone = true;
    const double a = c.home_tick + c.probe_sign * 200.0, b = c.home_tick + c.probe_sign * 500.0;
    j.slow_lo = a < b ? a : b;
    j.slow_hi = a < b ? b : a;
    j.slow_speed_tps = 60;
    rig.run();
    checkComplete(rig);
  }
  {
    g_case = "speed-register noise on every joint";
    Rig rig(Leg::LF);
    for (const JointOracle& o : kOracle) {
      rig.backend.joint[o.bus].speed_noise_every = 7;
      rig.backend.joint[o.bus].speed_noise_raw = 3;
    }
    rig.run();
    checkComplete(rig);
  }
}

// --- INITIAL RECOVERY -----------------------------------------------------------

void test_recovery_only_run_commands_all_twelve_joints() {
  for (const Leg leg : kAllLegs) {
    g_case = "recovery-only: all 12 actively recovered, verified, torque off";
    Rig rig(leg);
    rig.plan.request.recovery_only = true;
    rig.report_phases = false;
    rig.run();
    const FullLegCalibrationStatus& s = rig.full.status();
    CHECK_EQ((int)s.step, (int)FullLegStep::COMPLETE);
    CHECK_EQ((int)s.failure, (int)FullLegFailure::NONE);
    CHECK_EQ(s.contacts_accepted, 0);
    const std::vector<CalibrationPhase> expect = {CalibrationPhase::PREFLIGHT,
                                                  CalibrationPhase::INITIAL_RECOVERY,
                                                  CalibrationPhase::TORQUE_OFF};
    CHECK(rig.phases == expect);
    checkRecoveredAll(rig);
    checkSafeEnd(rig);
    // No probe ever ran, no joint left q0 by more than the tracking shortfall.
    CHECK_EQ((int)rig.full.probeStatus().phase, (int)ContactProbePhase::IDLE);
    for (const JointOracle& o : kOracle) {
      CHECK(std::abs(rig.backend.joint[o.bus].position() - o.q0) <= (int)kSequenceStaticToleranceTicks);
      CHECK(rig.backend.joint[o.bus].torque_limit == 500);  // RAM only, never restored (V25)
    }
  }
}

void test_recovery_commands_a_joint_already_at_q0() {
  g_case = "a joint already exactly at q0 is still commanded";
  Rig rig(Leg::RH);
  for (const JointOracle& o : kOracle) rig.backend.joint[o.bus].pos = o.q0;
  rig.plan.request.recovery_only = true;
  rig.report_phases = false;
  rig.run();
  CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::COMPLETE);
  checkRecoveredAll(rig);
}

void test_recovery_out_of_range_moves_nothing() {
  g_case = "INITIAL_RECOVERY: a joint 80 ticks from q0 is not moved";
  Rig rig(Leg::LF);
  const JointOracle& o = oracleFor(Leg::RH, JointKind::HIP);
  rig.backend.joint[o.bus].pos = o.q0 + 80;
  rig.run();
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::INITIAL_RECOVERY_OUT_OF_RANGE);
  CHECK_EQ(rig.full.status().contacts_accepted, 0);
  for (const Event& e : rig.backend.events) {
    if (e.bus == o.bus) CHECK(e.kind == Ev::SAFE_OFF);
  }
  checkSafeEnd(rig);
}

void test_recovery_that_does_not_stay_at_q0_fails() {
  g_case = "INITIAL_RECOVERY: a joint that sags after torque-off";
  Rig rig(Leg::RF);
  const JointOracle& o = oracleFor(Leg::LH, JointKind::LOWER);
  // After all twelve were actively recovered and SAFE_OFF, before the final
  // all-at-q0 check reaches it: 11 ticks off q0 (> the 10-tick V25 gate).
  rig.run(once([&](Rig& r) { return r.full.status().phase == CalibrationPhase::INITIAL_RECOVERY &&
                                    r.full.status().recovered_joints == 12 &&
                                    r.full.status().step == FullLegStep::VERIFY_REST; },
               [&](Rig& r) { r.backend.joint[o.bus].pos = o.q0 + 11; }));
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::INITIAL_RECOVERY_NOT_SETTLED);
  checkSafeEnd(rig);
}

void test_recovery_move_that_never_settles_times_out() {
  g_case = "INITIAL_RECOVERY: a joint blocked short of q0";
  Rig rig(Leg::LH);
  const JointOracle& o = oracleFor(Leg::LF, JointKind::UPPER);
  rig.backend.joint[o.bus].pos = o.q0 + 40;
  rig.backend.joint[o.bus].has_stop_low = true;
  rig.backend.joint[o.bus].stop_low = o.q0 + 25;
  rig.run();
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_TIMEOUT);
  CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::INITIAL_RECOVERY);
  checkSafeEnd(rig);
}


void prepareObservedAbort(Rig& rig) {
  // Rebase the real resolver to the 3 October promoted Q0 values.
  for (const auto& item : std::vector<std::pair<uint8_t,uint16_t>>{{21,1997},{22,2106},{32,2058}}) {
    for (const JointOracle& o : kOracle) if (o.bus == item.first) {
      CHECK(rig.policy.transforms().admit(promotedTransform(identityOf(o),item.second)));
      rig.backend.joint[o.bus].pos=item.second;
    }
  }
  CHECK(resolveFullLegPlan(rig.profile, actuator::geometry_data::kProvenance, rig.policy.transforms(),
                          &actuator::sequence_plan_data::kPlan, Leg::RF, &rig.plan)==FullLegPlanStatus::OK);
  for (uint8_t k=0;k<3;++k) for (uint8_t side=0;side<2;++side) rig.placeStop(k,side,StopSpec().beyond_limit[k][side]);
  CHECK(rig.start());
  const uint32_t started=rig.t;
  while(rig.full.active() && rig.t-started<300000) {
    rig.tick(nullptr);
    if (rig.full.status().phase==CalibrationPhase::LOWER_MAX && rig.sim(kLower).position()<=2352 &&
        rig.sim(kLower).position()>=2340) { rig.full.abort(); break; }
  }
  while(rig.full.active() && rig.t-started<310000) rig.tick(nullptr);
  CHECK(rig.full.status().failure==FullLegFailure::OPERATOR_ABORT);
  CHECK(std::abs(rig.backend.joint[21].position()-2348)<=10);
  CHECK(std::abs(rig.backend.joint[22].position()-1080)<=10);
  CHECK(std::abs(rig.backend.joint[32].position()-1665)<=10);
  // Torque-off residuals from the recorded session; all differences are
  // inside the witness's allowed passive drift, with no new goal command.
  rig.backend.joint[21].pos=2348;rig.backend.joint[22].pos=1080;rig.backend.joint[32].pos=1665;
  rig.report_phases=false;
}
void finishRecovery(Rig& rig, const Hook& hook=nullptr) {
  uint32_t start=rig.t;
  while(rig.full.active() && rig.t-start<100000) rig.tick(hook);
  CHECK(!rig.full.active());
}
void test_post_abort_all_supported_phases() {
  for (Leg leg : kAllLegs) for(CalibrationPhase phase : {CalibrationPhase::UPPER_MIN,CalibrationPhase::UPPER_MAX,
      CalibrationPhase::LOWER_MIN,CalibrationPhase::LOWER_MAX}) {
    g_case="all four legs: recover only proven UPPER/LOWER probe phases";
    Rig rig(leg);CHECK(rig.start());
    uint32_t start=rig.t;
    while(rig.full.active() && rig.t-start<200000) {
      rig.tick(nullptr);
      if(rig.full.status().phase==phase && rig.full.status().step==FullLegStep::PROBE) { rig.full.abort();break; }
    }
    while(rig.full.active() && rig.t-start<210000)rig.tick(nullptr);
    CHECK(rig.full.status().failure==FullLegFailure::OPERATOR_ABORT);
    rig.report_phases=false;
    CHECK(rig.full.startPostAbortRecovery(rig.req(),rig.ctx(),rig.t));finishRecovery(rig);
    CHECK(rig.full.status().step==FullLegStep::COMPLETE);checkSafeEnd(rig);
  }
  g_case="HIP phase has no automatic recovery qualification";
  Rig rig(Leg::RF);CHECK(rig.start());
  uint32_t start=rig.t;
  while(rig.full.active() && rig.t-start<200000) {
    rig.tick(nullptr);
    if(rig.full.status().phase==CalibrationPhase::HIP_MIN && rig.full.status().step==FullLegStep::PROBE) {rig.full.abort();break;}
  }
  while(rig.full.active() && rig.t-start<210000)rig.tick(nullptr);
  const auto writes=rig.backend.events.size();
  CHECK(!rig.full.startPostAbortRecovery(rig.req(),rig.ctx(),rig.t));
  CHECK(rig.full.status().failure==FullLegFailure::POST_ABORT_PHASE_UNPROVEN);
  CHECK(writes==rig.backend.events.size());checkSafeEnd(rig);
}

void test_post_abort_recovery() {
  {
    g_case="observed RF LOWER MAX ABORT -> LOWER, UPPER, rear park, all 12 Q0 + SAFE_OFF";
    Rig rig(Leg::RF);prepareObservedAbort(rig);
    const auto first=rig.backend.events.size();
    CHECK(rig.full.startPostAbortRecovery(rig.req(),rig.ctx(),rig.t));finishRecovery(rig);
    CHECK(rig.full.status().step==FullLegStep::COMPLETE);
    CHECK(rig.full.status().recovered_joints==12);
    CHECK(rig.full.status().contacts_accepted==0);
    for(const auto& joint : rig.full.request().population) CHECK(std::abs(rig.backend.joint[joint.bus_id].position()-joint.q0_tick)<=10);
    checkSafeEnd(rig);
    std::vector<uint8_t> returns;
    for(size_t i=first;i<rig.backend.events.size();++i) {
      const Event& e=rig.backend.events[i];
      if(e.kind==Ev::GOAL && (e.phase==CalibrationPhase::RETURN_LOWER_HELD ||
          e.phase==CalibrationPhase::RETURN_UPPER || e.phase==CalibrationPhase::RESTORE_PARKING)) returns.push_back(e.bus);
    }
    CHECK(returns==std::vector<uint8_t>({21,22,32}));
  }
  for (int fault=0;fault<6;++fault) {
    g_case="post-ABORT: unsafe dependency/UART/current/thermal/timeout/permit -> SAFE_OFF";
    Rig rig(Leg::RF);prepareObservedAbort(rig);
    if(fault==0) rig.backend.joint[22].pos=2106; // unsafe LOWER return with wrong UPPER dependency
    if(fault==1) rig.backend.joint[21].read_fails=true;
    const auto first=rig.backend.events.size();
    CHECK(rig.full.startPostAbortRecovery(rig.req(),rig.ctx(),rig.t));
    Hook hook=once([](Rig& r) { return r.full.status().phase==CalibrationPhase::RETURN_LOWER_HELD &&
                                      r.full.status().step==FullLegStep::MOVE_MONITOR; },
        [fault](Rig& r) {
          if(fault==2) r.sim(kLower).current_override=200;
          if(fault==3) r.sim(kLower).temperature=80;
          if(fault==4) { r.sim(kLower).has_stop_low=true;r.sim(kLower).stop_low=2200; }
          if(fault==5) r.permit=false;
        });
    finishRecovery(rig,hook);
    CHECK(rig.full.status().step==FullLegStep::FAILED);
    const FullLegFailure expected[]={FullLegFailure::POST_ABORT_POSE_MISMATCH,FullLegFailure::STALE_TELEMETRY,
        FullLegFailure::HARD_CURRENT_ABORT,FullLegFailure::OVER_TEMPERATURE,FullLegFailure::MOVE_TIMEOUT,
        FullLegFailure::DYNAMIC_PREREQUISITE_LOST};
    CHECK(rig.full.status().failure==expected[fault]);
    if(fault<2) for(size_t i=first;i<rig.backend.events.size();++i) CHECK(rig.backend.events[i].kind==Ev::SAFE_OFF);
    rig.backend.joint[21].read_fails=false;rig.sim(kLower).temperature=34;checkSafeEnd(rig);
  }
  {
    g_case="new controller has no post-ABORT movement proof";
    Rig rig(Leg::RF);CHECK(!rig.full.startPostAbortRecovery(rig.req(),rig.ctx(),rig.t));CHECK(rig.backend.events.empty());
  }
  {
    g_case="changed promoted Q0 cannot reuse an ABORT witness";
    Rig rig(Leg::RF);prepareObservedAbort(rig);
    auto changed=rig.req();changed.population[0].q0_tick++;
    const auto count=rig.backend.events.size();CHECK(!rig.full.startPostAbortRecovery(changed,rig.ctx(),rig.t));
    CHECK(rig.full.status().failure==FullLegFailure::POST_ABORT_Q0_CHANGED);CHECK(count==rig.backend.events.size());
  }
}

// --- held prerequisites ---------------------------------------------------------

// The CALIBRATION_HELD_ROLE_FAILURE record names the exact motor.
void checkHeldRoleFailure(Rig& rig, uint8_t bus, uint8_t slot, int held_target,
                          FullLegFailure expect, CalibrationPhase phase) {
  const FullLegHeldObservation& f = rig.full.heldRoleFailure();
  CHECK(f.valid);
  CHECK_EQ((int)f.failure, (int)expect);
  CHECK_EQ((int)f.phase, (int)phase);
  CHECK_EQ(f.bus_id, bus);
  const FullLegJoint& j = slot == 3 ? rig.req().park : rig.req().joint[slot];
  CHECK_EQ((int)f.identity.leg, (int)j.identity.leg);
  CHECK_EQ((int)f.identity.joint, (int)j.identity.joint);
  CHECK_EQ(f.target_tick, held_target);
  CHECK(f.has_sample);
  if (expect == FullLegFailure::STALE_TELEMETRY) {
    CHECK(!f.sample_usable);  // the last good sample, and how old it is
    CHECK(f.sample_age_ms >= kSequenceMaxTelemetryAgeMs);
  } else {
    CHECK(f.sample_usable);
    CHECK_EQ(f.sample_age_ms, 0u);
  }
  CHECK_EQ(f.position_error, std::abs(f.present_position - held_target));
  CHECK(f.present_speed >= 0 && f.goal_position >= 0 && f.torque_enable >= 0 &&
        f.torque_limit >= 0 && f.present_current >= 0 && f.present_temperature >= 0 &&
        f.servo_status >= 0);
  // ...and what the run was driving at that moment: the probe.
  JointKind pj = JointKind::UPPER;
  ContactSide ps = ContactSide::MIN_SIDE;
  CHECK(actuator::sequenceProbeEndpoint(phase, &pj, &ps));
  CHECK(f.active_is_probe);
  CHECK_EQ(f.active_bus, rig.bus(static_cast<uint8_t>(pj)));
  CHECK_EQ((int)f.active_joint, (int)pj);
  CHECK_EQ((int)f.active_side, (int)ps);
  CHECK(f.active_target_tick != 0);
  CHECK(f.active_position >= 0);
}


struct HeldCase {
  const char* name;
  Leg leg;
  CalibrationPhase phase;
  uint8_t held_slot;  // kHip/kUpper/kLower, or 3 = the rear park joint
  std::function<void(simk::SimJoint&)> act;
  FullLegFailure expect;
  uint8_t contacts;
};

void test_held_joint_violations_fail_closed() {
  const HeldCase cases[] = {
      {"held HIP pushed 25 ticks during LOWER MIN", Leg::LF, CalibrationPhase::LOWER_MIN, kHip,
       [](simk::SimJoint& j) { j.pos += 25; }, FullLegFailure::HELD_JOINT_DRIFT, 2},
      {"held LOWER loses torque during HIP MIN", Leg::RH, CalibrationPhase::HIP_MIN, kLower,
       [](simk::SimJoint& j) { j.torque = false; }, FullLegFailure::HELD_JOINT_READBACK, 4},
      {"held UPPER telemetry lost during LOWER MAX", Leg::LH, CalibrationPhase::LOWER_MAX, kUpper,
       [](simk::SimJoint& j) { j.read_fails = true; }, FullLegFailure::STALE_TELEMETRY, 3},
      {"held LOWER GoalPosition rewritten during HIP MAX", Leg::LF, CalibrationPhase::HIP_MAX, kLower,
       [](simk::SimJoint& j) { j.goal += 30; }, FullLegFailure::HELD_JOINT_READBACK, 5},
      {"held UPPER TorqueLimit back to 1000 during LOWER MIN", Leg::RF, CalibrationPhase::LOWER_MIN,
       kUpper, [](simk::SimJoint& j) { j.torque_limit = 1000; }, FullLegFailure::HELD_JOINT_READBACK, 2},
      {"held UPPER hard current during HIP MIN", Leg::LH, CalibrationPhase::HIP_MIN, kUpper,
       [](simk::SimJoint& j) { j.current_override = 250; }, FullLegFailure::HARD_CURRENT_ABORT, 4},
      {"held LOWER status fault during UPPER MAX", Leg::RH, CalibrationPhase::UPPER_MAX, kLower,
       [](simk::SimJoint& j) { j.status = 0x20; }, FullLegFailure::SERVO_STATUS_FAULT, 1},
      {"held HIP over temperature during LOWER MAX", Leg::LF, CalibrationPhase::LOWER_MAX, kHip,
       [](simk::SimJoint& j) { j.temperature = 75; }, FullLegFailure::OVER_TEMPERATURE, 3},
      {"rear park UPPER pushed during UPPER MAX", Leg::LF, CalibrationPhase::UPPER_MAX, 3,
       [](simk::SimJoint& j) { j.pos += 25; }, FullLegFailure::HELD_JOINT_DRIFT, 1},
      {"rear park UPPER torque lost during HIP MIN", Leg::RF, CalibrationPhase::HIP_MIN, 3,
       [](simk::SimJoint& j) { j.torque = false; }, FullLegFailure::HELD_JOINT_READBACK, 4},
  };
  for (const HeldCase& hc : cases) {
    g_case = hc.name;
    Rig rig(hc.leg);
    const uint8_t b = hc.held_slot == 3 ? rig.req().park.bus_id : rig.bus(hc.held_slot);
    int held_target = -1;
    rig.run(once(probing(hc.phase), [&](Rig& r) {
      held_target = r.backend.joint[b].goal;
      hc.act(r.backend.joint[b]);
    }));
    CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::FAILED);
    CHECK_EQ((int)rig.full.status().failure, (int)hc.expect);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)hc.phase);
    CHECK_EQ(rig.full.status().contacts_accepted, hc.contacts);
    checkHeldRoleFailure(rig, b, hc.held_slot, held_target, hc.expect, hc.phase);
    if (hc.expect == FullLegFailure::OVER_TEMPERATURE) {
      // The V25 confirmation CONFIRMED it (>= 2 of 3 direct reads) first.
      bool confirmed = false;
      for (const ThermalConfirmation& th : rig.thermal_log) {
        confirmed = confirmed || (th.bus_id == b && th.decision == ThermalDecision::CONFIRMED);
      }
      CHECK(confirmed);
    }
    // The held joint is watched - no telemetry-loss case aside, the failure
    // came within a few ticks, not after the probe finished.
    CHECK(rig.full.contact(rig.full.probeRequest().endpoint_joint,
                           rig.full.probeRequest().endpoint_side).has_measurement == false);
    for (const JointOracle& o : kOracle) rig.backend.joint[o.bus].read_fails = false;
    checkSafeEnd(rig);
  }
}

// --- held-role supervision: LF V25 ActivelyHeld, never a speed abort -------------
//
// V25 validate_lf_role_observation(LfMotorRole::ActivelyHeld): fresh
// telemetry, status, hard current, temperature, TorqueEnable / TorqueLimit /
// GoalPosition readback, position within STATIC_TOLERANCE_TICKS (10) of the
// held target. LF_HELD_MAX_SPEED_RAW (4) is used by the StableTargetGate that
// PROMOTES a moved joint to held and by INITIAL_RECOVERY's settle - not by the
// supervision of an already-held joint.
//   MOVING / SETTLING TO BECOME HELD: speed matters.
//   ALREADY HELD DURING ANOTHER JOINT'S PROBE: position / readback / safety
//   matter; instantaneous speed alone does not abort.

// 2026-09-30 hardware, LF UPPER MAX: ~66 ms into a coarse step, the probed
// UPPER accelerating, a held joint (LF HIP, LF LOWER or the LH UPPER park)
// reported |speed| > 40 on two consecutive samples while its position stayed
// inside the 10-tick hold; the retired post-V25 D5 rule aborted the run. Here:
// the held joint's speed register reads 60 for the whole rest of the probe,
// position / torque / goal / limit / current / temperature all valid. No
// abort - the leg completes 6/6 - and the transient is one diagnostic record.
void test_held_speed_transient_is_not_an_abort() {
  for (const Leg leg : {Leg::LF, Leg::RF, Leg::RH, Leg::LH}) {
    for (const uint8_t slot : {kHip, kLower, (uint8_t)3}) {
      Rig rig(leg);
      if (slot == 3 && !rig.req().has_rear_park) continue;
      g_case = slot == kHip ? "held HIP |speed| 60 during UPPER MAX: not an abort"
               : slot == kLower ? "held LOWER |speed| 60 during UPPER MAX: not an abort"
                                : "held rear park |speed| 60 during UPPER MAX: not an abort";
      const uint8_t b = slot == 3 ? rig.req().park.bus_id : rig.bus(slot);
      int held_target = -1;
      bool armed = false, cleared = false;
      rig.run([&](Rig& r) {
        if (!armed && probing(CalibrationPhase::UPPER_MAX, 3)(r)) {
          armed = true;
          held_target = r.backend.joint[b].goal;
          r.backend.joint[b].speed_override = 60;
        } else if (armed && !cleared && r.full.status().phase != CalibrationPhase::UPPER_MAX) {
          cleared = true;  // before this joint is moved again (a settle needs <= 4)
          r.backend.joint[b].speed_override = -1;
        }
      });
      CHECK(armed && cleared);
      checkComplete(rig);
      CHECK(!rig.full.heldRoleFailure().valid);
      CHECK(rig.max_fast_run[b] >= 2);  // the monitor saw consecutive samples > 40
      int records = 0;
      for (const FullLegHeldObservation& o : rig.transients) {
        if (o.bus_id != b) continue;
        ++records;
        CHECK(o.valid);
        CHECK_EQ((int)o.failure, (int)FullLegFailure::NONE);
        CHECK_EQ((int)o.phase, (int)CalibrationPhase::UPPER_MAX);
        const FullLegJoint& j = slot == 3 ? rig.req().park : rig.req().joint[slot];
        CHECK_EQ((int)o.identity.leg, (int)j.identity.leg);
        CHECK_EQ((int)o.identity.joint, (int)j.identity.joint);
        CHECK_EQ(o.target_tick, held_target);
        CHECK(o.position_error <= (int)kSequenceStaticToleranceTicks);
        CHECK_EQ(o.present_speed, 60);
        CHECK_EQ(o.goal_position, held_target);
        CHECK_EQ(o.torque_enable, 1);
        CHECK_EQ(o.torque_limit, rig.req().torque_limit);
        CHECK(o.sample_usable);
        CHECK(o.active_is_probe);
        CHECK_EQ(o.active_bus, rig.bus(kUpper));
        CHECK_EQ((int)o.active_joint, (int)JointKind::UPPER);
        CHECK_EQ((int)o.active_side, (int)ContactSide::MAX_SIDE);
      }
      CHECK_EQ(records, 1);  // one rising edge, one record - not one per sample
      CHECK_EQ(rig.full.heldSpeedTransientTotal(), 1);
      checkSafeEnd(rig);
    }
  }
}

// The transient diagnostic is bounded: a held joint whose speed register
// flickers above 40 every other 20 ms slot for a whole probe produces many
// rising edges, all counted, at most kHeldSpeedTransientEventCap recorded -
// and still no abort.
void test_held_speed_transient_records_are_bounded() {
  g_case = "flickering held-joint speed: bounded records, no abort";
  Rig rig(Leg::RH);
  const uint8_t b = rig.bus(kHip);
  bool armed = false, cleared = false;
  rig.run([&](Rig& r) {
    if (!armed && probing(CalibrationPhase::UPPER_MAX, 1)(r)) {
      armed = true;
      r.backend.joint[b].speed_noise_every = 2;
      r.backend.joint[b].speed_noise_raw = 60;
    } else if (armed && !cleared && r.full.status().phase != CalibrationPhase::UPPER_MAX) {
      cleared = true;
      r.backend.joint[b].speed_noise_every = 0;
    }
  });
  checkComplete(rig);
  CHECK(!rig.full.heldRoleFailure().valid);
  CHECK(rig.full.heldSpeedTransientTotal() > kHeldSpeedTransientEventCap);
  CHECK_EQ(rig.transients.size(), (size_t)kHeldSpeedTransientEventCap);
  for (const FullLegHeldObservation& o : rig.transients) CHECK_EQ(o.bus_id, b);
  checkSafeEnd(rig);
}

// The physical hold invariant is the position: exactly 10 ticks off the held
// target is inside the hold, 11 fails closed, on every leg and every held
// joint of UPPER MAX.
void test_held_position_drift_boundary() {
  for (const Leg leg : {Leg::LF, Leg::RF, Leg::RH, Leg::LH}) {
    for (const uint8_t slot : {kHip, kLower, (uint8_t)3}) {
      for (const int off : {10, -10, 11, -11}) {
        Rig rig(leg);
        if (slot == 3 && !rig.req().has_rear_park) continue;
        g_case = std::abs(off) == 10 ? "held joint exactly 10 ticks off its target: inside the hold"
                                     : "held joint 11 ticks off its target: HELD_JOINT_DRIFT";
        const uint8_t b = slot == 3 ? rig.req().park.bus_id : rig.bus(slot);
        int held_target = -1;
        rig.run(once(probing(CalibrationPhase::UPPER_MAX), [&](Rig& r) {
          held_target = r.backend.joint[b].goal;
          r.backend.joint[b].pos = held_target + off;
        }));
        if (std::abs(off) == 10) {
          checkComplete(rig);
          CHECK(!rig.full.heldRoleFailure().valid);
        } else {
          CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::HELD_JOINT_DRIFT);
          CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::UPPER_MAX);
          checkHeldRoleFailure(rig, b, slot, held_target, FullLegFailure::HELD_JOINT_DRIFT,
                               CalibrationPhase::UPPER_MAX);
          CHECK_EQ(rig.full.heldRoleFailure().position_error, 11);
        }
        checkSafeEnd(rig);
      }
    }
  }
}

// V25 StableTargetGate: a moved joint is promoted to held only once it is
// within 10 ticks AND |speed| <= LF_HELD_MAX_SPEED_RAW (4) for 4 samples over
// >= 400 ms. A joint at its target whose speed reads 5 is never promoted (the
// move times out, nothing is probed); 4 is promoted.
void test_speed_gates_the_promotion_to_held() {
  for (const Leg leg : {Leg::LF, Leg::RF, Leg::RH, Leg::LH}) {
    for (const int speed : {5, 4}) {
      g_case = speed == 5 ? "UPPER MIN prerequisite HIP at target, speed 5: never held"
                          : "UPPER MIN prerequisite HIP at target, speed 4: held";
      Rig rig(leg);
      const uint8_t b = rig.bus(kHip);
      bool armed = false, cleared = false;
      rig.run([&](Rig& r) {
        const bool moving_hip = r.full.status().phase == CalibrationPhase::UPPER_MIN &&
                                r.full.status().step == FullLegStep::MOVE_MONITOR &&
                                r.full.status().op_bus == b;
        if (!armed && moving_hip) {
          armed = true;
          r.backend.joint[b].speed_override = speed;
        } else if (armed && !cleared && !moving_hip) {
          cleared = true;
          r.backend.joint[b].speed_override = -1;
        }
      });
      CHECK(armed);
      if (speed == 5) {
        CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_TIMEOUT);
        CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::UPPER_MIN);
        CHECK(rig.probe_trace[static_cast<uint8_t>(CalibrationPhase::UPPER_MIN)].empty());
        CHECK_EQ(rig.held_checks[static_cast<uint8_t>(CalibrationPhase::UPPER_MIN)], 0);
        CHECK(!rig.full.heldRoleFailure().valid);  // it never became a held joint
      } else {
        checkComplete(rig);
      }
      checkSafeEnd(rig);
    }
  }
  for (const int speed : {5, 4}) {
    g_case = speed == 5 ? "PARKING rear park at target, speed 5: never held"
                        : "PARKING rear park at target, speed 4: held";
    Rig rig(Leg::LF);
    const uint8_t b = rig.req().park.bus_id;
    bool armed = false, cleared = false;
    rig.run([&](Rig& r) {
      const bool moving = r.full.status().phase == CalibrationPhase::PARKING &&
                          r.full.status().step == FullLegStep::MOVE_MONITOR;
      if (!armed && moving) {
        armed = true;
        r.backend.joint[b].speed_override = speed;
      } else if (armed && !cleared && !moving) {
        cleared = true;
        r.backend.joint[b].speed_override = -1;
      }
    });
    CHECK(armed);
    if (speed == 5) {
      CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_TIMEOUT);
      CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::PARKING);
      CHECK_EQ(rig.full.status().contacts_accepted, 0);
    } else {
      checkComplete(rig);
    }
    checkSafeEnd(rig);
  }
}

// INITIAL_RECOVERY quiescence: a recovered joint at q0 whose speed reads 5 is
// never settled (MOVE_TIMEOUT, not all twelve recovered); 4 settles.
void test_speed_gates_the_recovery_settle() {
  for (const int speed : {5, 4}) {
    g_case = speed == 5 ? "INITIAL_RECOVERY joint at q0, speed 5: never settled"
                        : "INITIAL_RECOVERY joint at q0, speed 4: settled";
    Rig rig(Leg::LH);
    const uint8_t b = rig.req().population[0].bus_id;
    bool armed = false, cleared = false;
    rig.run([&](Rig& r) {
      const bool first = r.full.status().phase == CalibrationPhase::INITIAL_RECOVERY &&
                         r.full.status().step == FullLegStep::MOVE_MONITOR &&
                         r.full.status().recovered_joints == 0;
      if (!armed && first) {
        armed = true;
        r.backend.joint[b].speed_override = speed;
      } else if (armed && !cleared && !first) {
        cleared = true;
        r.backend.joint[b].speed_override = -1;
      }
    });
    CHECK(armed);
    if (speed == 5) {
      CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_TIMEOUT);
      CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::INITIAL_RECOVERY);
      CHECK_EQ(rig.full.status().recovered_joints, 0);
    } else {
      checkComplete(rig);
    }
    checkSafeEnd(rig);
  }
}

// --- the probed joint -------------------------------------------------------------

void test_probe_failures_stop_the_sequence_at_the_right_count() {
  struct Case {
    const char* name;
    Leg leg;
    uint8_t k, side;
    FullLegFailure expect;
    ContactProbeFailure probe;
    uint8_t contacts;
  };
  const Case cases[] = {
      {"no LOWER MIN stop before the guard: 2/6", Leg::LF, kLower, kMin,
       FullLegFailure::LOWER_MIN_PROBE_FAILED, ContactProbeFailure::NO_CONTACT_BEFORE_GUARD, 2},
      {"no HIP MAX stop before the guard: 5/6", Leg::RH, kHip, kMax,
       FullLegFailure::HIP_MAX_PROBE_FAILED, ContactProbeFailure::NO_CONTACT_BEFORE_GUARD, 5},
      {"no UPPER MIN stop before the guard: 0/6", Leg::RF, kUpper, kMin,
       FullLegFailure::UPPER_MIN_PROBE_FAILED, ContactProbeFailure::NO_CONTACT_BEFORE_GUARD, 0},
      {"no LOWER MAX stop before the guard: 3/6", Leg::LH, kLower, kMax,
       FullLegFailure::LOWER_MAX_PROBE_FAILED, ContactProbeFailure::NO_CONTACT_BEFORE_GUARD, 3},
  };
  for (const Case& c : cases) {
    g_case = c.name;
    Rig rig(c.leg);
    rig.removeStop(c.k, c.side);
    rig.run();
    CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::FAILED);
    CHECK_EQ((int)rig.full.status().failure, (int)c.expect);
    CHECK_EQ((int)rig.full.probeStatus().failure, (int)c.probe);
    CHECK_EQ(rig.full.status().contacts_accepted, c.contacts);
    // The search never went past the guard.
    const CalibrationSearchCorridor& cor = rig.req().corridor[c.k][c.side];
    const simk::SimJoint& j = rig.sim(c.k);
    const double deepest = cor.probe_sign > 0 ? j.hi_seen : j.lo_seen;
    CHECK(actuator::searchDepth(cor, static_cast<uint16_t>(deepest)) <=
          actuator::searchDepth(cor, cor.guard_tick));
    checkSafeEnd(rig);
  }
  {
    g_case = "HIP MIN hard stop 100 ticks before the limit: early stall, 4/6";
    StopSpec s;
    s.beyond_limit[kHip][kMin] = -100;
    Rig rig(Leg::LH, s);
    rig.run();
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::HIP_MIN_PROBE_FAILED);
    // Blocked 100 ticks short under a 64-tick coarse step: the pressing
    // current trips the hard abort before (or instead of) the stall verdict.
    // Either way it is never a contact.
    const ContactProbeFailure pf = rig.full.probeStatus().failure;
    CHECK(pf == ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR ||
          pf == ContactProbeFailure::HARD_CURRENT_ABORT);
    CHECK(!rig.full.contact(JointKind::HIP, ContactSide::MIN_SIDE).has_measurement);
    CHECK_EQ(rig.full.status().contacts_accepted, 4);
    checkSafeEnd(rig);
  }
  {
    g_case = "probed LOWER hard current";
    Rig rig(Leg::RF);
    rig.run(once(probing(CalibrationPhase::LOWER_MAX), [](Rig& r) { r.sim(kLower).current_override = 250; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::LOWER_MAX_PROBE_FAILED);
    CHECK_EQ((int)rig.full.probeStatus().failure, (int)ContactProbeFailure::HARD_CURRENT_ABORT);
    checkSafeEnd(rig);
  }
  {
    g_case = "probed UPPER TorqueLimit changed";
    Rig rig(Leg::LF);
    rig.run(once(probing(CalibrationPhase::UPPER_MIN), [](Rig& r) { r.sim(kUpper).torque_limit = 1000; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::UPPER_MIN_PROBE_FAILED);
    CHECK_EQ((int)rig.full.probeStatus().failure, (int)ContactProbeFailure::TORQUE_LIMIT_CHANGED);
    checkSafeEnd(rig);
  }
  {
    g_case = "probed UPPER communication lost";
    Rig rig(Leg::RH);
    rig.run(once(probing(CalibrationPhase::UPPER_MAX), [](Rig& r) { r.sim(kUpper).read_fails = true; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::UPPER_MAX_PROBE_FAILED);
    CHECK_EQ(rig.full.status().contacts_accepted, 1);
    rig.sim(kUpper).read_fails = false;
    checkSafeEnd(rig);
  }
  {
    g_case = "probed HIP GoalPosition rewritten by someone else";
    Rig rig(Leg::LH);
    rig.run(once(probing(CalibrationPhase::HIP_MIN), [](Rig& r) { r.sim(kHip).goal += 40; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::HIP_MIN_PROBE_FAILED);
    CHECK_EQ((int)rig.full.probeStatus().failure, (int)ContactProbeFailure::GOAL_READBACK_MISMATCH);
    checkSafeEnd(rig);
  }
  {
    g_case = "probed HIP status fault";
    Rig rig(Leg::LF);
    rig.run(once(probing(CalibrationPhase::HIP_MAX), [](Rig& r) { r.sim(kHip).status = 0x08; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::HIP_MAX_PROBE_FAILED);
    CHECK_EQ((int)rig.full.probeStatus().failure, (int)ContactProbeFailure::SERVO_STATUS_FAULT);
    checkSafeEnd(rig);
  }
  {
    g_case = "backoff stalls: the joint cannot leave the stop";
    Rig rig(Leg::RF);
    rig.run(once([](Rig& r) { return r.full.status().phase == CalibrationPhase::LOWER_MIN &&
                                     r.full.probeStatus().phase == ContactProbePhase::BACKOFF_MONITORING; },
                 [](Rig& r) {
                   simk::SimJoint& j = r.sim(kLower);
                   j.plateau = true;  // hold it where it is whatever the target error
                   j.plateau_tick = j.pos;
                   j.plateau_breakaway = 4000;
                 }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::LOWER_MIN_PROBE_FAILED);
    CHECK_EQ(rig.full.status().contacts_accepted, 2);
    checkSafeEnd(rig);
  }
}

// --- transitions, parking, dynamic prerequisites --------------------------------

void test_transition_and_parking_failures() {
  {
    g_case = "UPPER_HORIZONTAL transition never settles";
    Rig rig(Leg::LH);
    auto frozen = std::make_shared<double>(-1);
    rig.run([frozen](Rig& r) {
      if (r.full.status().phase == CalibrationPhase::UPPER_HORIZONTAL &&
          r.full.status().step == FullLegStep::MOVE_MONITOR) {
        if (*frozen < 0) *frozen = r.sim(kUpper).pos;
        r.sim(kUpper).pos = *frozen;
      }
    });
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_TIMEOUT);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::UPPER_HORIZONTAL);
    CHECK_EQ(rig.full.status().contacts_accepted, 2);
    checkSafeEnd(rig);
  }
  {
    g_case = "rear park never reaches its pose";
    Rig rig(Leg::LF);
    const uint8_t park = rig.req().park.bus_id;
    CHECK(rig.req().has_rear_park && park == 42);
    auto frozen = std::make_shared<double>(-1);
    rig.run([frozen, park](Rig& r) {
      if (r.full.status().phase == CalibrationPhase::PARKING &&
          r.full.status().step == FullLegStep::MOVE_MONITOR) {
        if (*frozen < 0) *frozen = r.backend.joint[park].pos;
        r.backend.joint[park].pos = *frozen;
      }
    });
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_TIMEOUT);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::PARKING);
    CHECK_EQ(rig.full.status().contacts_accepted, 0);
    checkSafeEnd(rig);
  }
  {
    g_case = "rear park telemetry lost while parking";
    Rig rig(Leg::RF);
    const uint8_t park = rig.req().park.bus_id;
    CHECK(park == 32);
    rig.run(once(inPhaseStep(CalibrationPhase::PARKING, FullLegStep::MOVE_MONITOR),
                 [park](Rig& r) { r.backend.joint[park].read_fails = true; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::STALE_TELEMETRY);
    rig.backend.joint[park].read_fails = false;
    checkSafeEnd(rig);
  }
  {
    g_case = "authority lost mid-probe";
    Rig rig(Leg::LF);
    rig.run(once(probing(CalibrationPhase::LOWER_MIN),
                 [](Rig& r) { r.arbiter.forceClear(AuthorityClearReason::BOOT); }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::DYNAMIC_PREREQUISITE_LOST);
    checkSafeEnd(rig);
  }
  {
    g_case = "permit lost mid-probe";
    Rig rig(Leg::RH);
    rig.run(once(probing(CalibrationPhase::HIP_MAX), [](Rig& r) { r.permit = false; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::DYNAMIC_PREREQUISITE_LOST);
    CHECK_EQ(rig.full.status().contacts_accepted, 5);
    checkSafeEnd(rig);
  }
  {
    g_case = "session lost mid-transition";
    Rig rig(Leg::LH);
    rig.run(once(inPhaseStep(CalibrationPhase::LOWER_FOLDED, FullLegStep::MOVE_MONITOR),
                 [](Rig& r) { r.session = false; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::DYNAMIC_PREREQUISITE_LOST);
    checkSafeEnd(rig);
  }
  {
    g_case = "promoted transforms withdrawn mid-run: the next move is refused";
    Rig rig(Leg::RF);
    // The first tick of UPPER_HORIZONTAL, before its move is written.
    rig.run(once(inPhaseStep(CalibrationPhase::UPPER_HORIZONTAL, FullLegStep::PROBE),
                 [](Rig& r) { r.policy.transforms().clear(); }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::MOVE_REJECTED);
    checkSafeEnd(rig);
  }
  {
    g_case = "operator abort mid-HIP MIN";
    Rig rig(Leg::LF);
    rig.run(once(probing(CalibrationPhase::HIP_MIN), [](Rig& r) { r.full.abort(); }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::OPERATOR_ABORT);
    CHECK_EQ(rig.full.status().contacts_accepted, 4);
    checkSafeEnd(rig);
  }
  {
    g_case = "session refuses the phase order";
    Rig rig(Leg::RH);
    rig.run(once([](Rig& r) { return r.full.status().phase == CalibrationPhase::LOWER_FOLDED; },
                 [](Rig& r) { r.full.phaseReportRejected(); }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::PHASE_REPORT_REJECTED);
    checkSafeEnd(rig);
  }
}

void test_bystanders_and_passive_participants() {
  {
    g_case = "a non-participant pushed 20 ticks";
    Rig rig(Leg::LF);
    const JointOracle& o = oracleFor(Leg::RH, JointKind::HIP);
    rig.run(once(probing(CalibrationPhase::LOWER_MIN), [&](Rig& r) { r.backend.joint[o.bus].pos += 20; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::BYSTANDER_MOVED);
    // Caught by the round-robin watch while it happened, not at the end.
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::LOWER_MIN);
    CHECK(rig.full.status().contacts_accepted <= 2);
    checkSafeEnd(rig);
  }
  {
    g_case = "a non-participant gains torque";
    Rig rig(Leg::RH);
    const JointOracle& o = oracleFor(Leg::LF, JointKind::LOWER);
    rig.run(once(probing(CalibrationPhase::UPPER_MAX), [&](Rig& r) {
      r.backend.joint[o.bus].goal = r.backend.joint[o.bus].position();
      r.backend.joint[o.bus].torque = true;
    }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::BYSTANDER_MOVED);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::UPPER_MAX);
    checkSafeEnd(rig);
  }
  {
    g_case = "the leg's limp UPPER pushed out of its corridor while parking";
    Rig rig(Leg::LF);
    rig.run(once(inPhaseStep(CalibrationPhase::PARKING, FullLegStep::MOVE_MONITOR),
                 [](Rig& r) { r.sim(kUpper).pos += 40; }));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::PASSIVE_JOINT_MOVED);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::PARKING);
    checkSafeEnd(rig);
  }
  {
    g_case = "a leg joint torque-on at PREFLIGHT: nothing is written";
    Rig rig(Leg::RF);
    rig.backend.joint[22].goal = rig.backend.joint[22].position();
    rig.backend.joint[22].torque = true;
    rig.run();
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::PREFLIGHT_TORQUE_ON);
    for (const Event& e : rig.backend.events) CHECK(e.kind == Ev::SAFE_OFF);
    checkSafeEnd(rig);
  }
}

void test_energize_write_failures() {
  {
    g_case = "TorqueEnable uncertain";
    Rig rig(Leg::LF);
    rig.backend.uncertain_torque_bus = rig.bus(kLower);
    rig.run(once([](Rig& r) { return r.full.status().phase == CalibrationPhase::UPPER_MIN; },
                 [](Rig&) {}));
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::WRITE_UNCERTAIN);
    checkSafeEnd(rig);
  }
  {
    g_case = "TorqueLimit refused: that joint is never torque-enabled";
    Rig rig(Leg::RH);
    const uint8_t b = oracleFor(Leg::LH, JointKind::HIP).bus;
    rig.backend.reject_limit_bus = b;
    rig.run();
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::TORQUE_LIMIT_REJECTED);
    for (const Event& e : rig.backend.events) CHECK(!(e.bus == b && e.kind == Ev::TORQUE));
    checkSafeEnd(rig);
  }
  {
    g_case = "TorqueLimit reads back 1000 after the write";
    Rig rig(Leg::LH);
    rig.backend.limit_reverts_bus = oracleFor(Leg::RF, JointKind::UPPER).bus;
    rig.run();
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::ENERGIZE_NOT_VERIFIED);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)CalibrationPhase::INITIAL_RECOVERY);
    checkSafeEnd(rig);
  }
}

void test_safe_off_is_retried_until_verified() {
  g_case = "SAFE_OFF not verified: retried every tick, never COMPLETE";
  Rig rig(Leg::LF);
  const uint8_t b = rig.bus(kHip);
  rig.start();
  bool aborted = false;
  for (int i = 0; i < 200000 && rig.full.active(); ++i) {
    rig.tick(nullptr);
    if (!aborted && rig.full.status().phase == CalibrationPhase::HIP_MIN &&
        rig.full.status().step == FullLegStep::PROBE) {
      rig.safe_off_fails[b] = true;
      rig.full.abort();
      aborted = true;
    }
    if (aborted && rig.safe_off_calls[b] == 50) break;
  }
  CHECK(aborted);
  CHECK(rig.full.active());  // still waiting for the one unverified bus
  CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::SAFE_OFF_ALL);
  CHECK(rig.safe_off_calls[b] >= 50);
  rig.safe_off_fails[b] = false;
  for (int i = 0; i < 1000 && rig.full.active(); ++i) rig.tick(nullptr);
  CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::FAILED);
  CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::OPERATOR_ABORT);
  checkSafeEnd(rig);
}

void test_diagnostics_rejection_returns_the_leg_then_fails() {
  g_case = "diagnostics rejected: RETURN phases run, then FAILED";
  Rig rig(Leg::RF);
  // A model span the contacts cannot match (scale far outside 850..1150).
  rig.plan.request.urdf_upper[kHip] = rig.plan.request.urdf_upper[kHip] * 3;
  rig.run();
  const FullLegCalibrationStatus& s = rig.full.status();
  CHECK_EQ((int)s.step, (int)FullLegStep::FAILED);
  CHECK_EQ((int)s.failure, (int)FullLegFailure::DIAGNOSTICS_REJECTED);
  CHECK_EQ(s.contacts_accepted, 6);
  CHECK(!rig.full.diagnosticsAccepted());
  CHECK(!rig.full.diagnostics(JointKind::HIP).accepted);
  CHECK(rig.phases == v25Order());  // the reviewed return, not a cut to TORQUE_OFF
  for (const JointOracle& o : kOracle) {
    CHECK(std::abs(rig.backend.joint[o.bus].position() - o.q0) <= (int)kSequenceRestToleranceTicks);
  }
  checkSafeEnd(rig);
}

// --- start refusals ----------------------------------------------------------------

void test_start_refuses_incomplete_requests() {
  g_case = "start refusals";
  {
    Rig rig(Leg::LF);
    rig.plan.request.population_count = 11;
    CHECK(!rig.start());
    CHECK_EQ((int)rig.full.status().failure, (int)FullLegFailure::REJECT_PRECONDITIONS);
  }
  {
    Rig rig(Leg::RF);
    rig.plan.request.corridor[kHip][kMax] = CalibrationSearchCorridor{};
    CHECK(!rig.start());
  }
  {
    Rig rig(Leg::RH);
    rig.plan.request.torque_limit = 0;
    CHECK(!rig.start());
  }
  {
    Rig rig(Leg::LH);
    rig.permit = false;
    CHECK(!rig.start());
  }
  {
    Rig rig(Leg::LF);
    rig.plan.request.park.bus_id = rig.bus(kUpper);  // the park may not be a leg joint
    CHECK(!rig.start());
  }
  {
    Rig rig(Leg::LF);
    rig.plan.request.population[4].bus_id = rig.plan.request.population[3].bus_id;
    CHECK(!rig.start());
  }
  // Nothing was ever written by a refused start.
  Rig rig(Leg::RH);
  rig.plan.request.repeatability_tolerance_ticks = 0;
  CHECK(!rig.start());
  CHECK(rig.backend.events.empty());
}

void test_diagnostics_math_is_v25() {
  g_case = "V25 derive_joint_evidence";
  FullLegCalibrationRequest r{};
  r.direction[kHip] = 1;
  r.urdf_lower[kHip] = -785398;  // -512 ticks
  r.urdf_upper[kHip] = 785398;   // +512
  r.joint[kHip].q0_tick = 2048;
  ContactEvidence lo{}, hi{};
  lo.has_measurement = hi.has_measurement = true;
  lo.coarse_tick = lo.fine_tick_1 = lo.fine_tick_2 = 2048 - 487;  // V25 LF HIP MIN
  hi.coarse_tick = hi.fine_tick_1 = hi.fine_tick_2 = 2048 + 448;  // V25 LF HIP MAX
  const FullLegJointDiagnostics d = deriveFullLegJointDiagnostics(r, JointKind::HIP, lo, hi);
  CHECK(d.evaluated && d.ordered && d.accepted);
  CHECK_EQ(d.expected_span_ticks, 1024);
  CHECK_EQ(d.measured_span_ticks, 935);
  CHECK_EQ(d.scale_permille, 913);
  CHECK(d.affine_shift_from_q0_ticks <= 20);
  // Reversed contacts are not ordered, whatever the span.
  const FullLegJointDiagnostics rev = deriveFullLegJointDiagnostics(r, JointKind::HIP, hi, lo);
  CHECK(!rev.ordered && !rev.accepted);
  // A missing side is not evaluated.
  ContactEvidence none{};
  CHECK(!deriveFullLegJointDiagnostics(r, JointKind::HIP, lo, none).evaluated);
  // [T10] matdog_test.rs coarse_scout_is_persisted_but_cannot_change_fine_metrology_or_q0:
  // only the two FINE passes enter the contact (their midpoint, V25
  // contact_result_tick); the scout, however far off, changes nothing.
  {
    ContactEvidence lo2 = lo, hi2 = hi;
    lo2.coarse_tick = 2048 - 460;
    hi2.coarse_tick = 2048 + 400;
    const FullLegJointDiagnostics d2 = deriveFullLegJointDiagnostics(r, JointKind::HIP, lo2, hi2);
    CHECK_EQ(d2.min_contact_tick, d.min_contact_tick);
    CHECK_EQ(d2.max_contact_tick, d.max_contact_tick);
    CHECK_EQ(d2.scale_permille, d.scale_permille);
    CHECK_EQ(d2.affine_zero_tick, d.affine_zero_tick);
    ContactEvidence hi3 = hi;
    hi3.fine_tick_1 = 2048 + 446;
    hi3.fine_tick_2 = 2048 + 451;
    CHECK_EQ(deriveFullLegJointDiagnostics(r, JointKind::HIP, lo, hi3).max_contact_tick, 2048 + 448);
  }
  // Scale out of band.
  hi.coarse_tick = hi.fine_tick_1 = hi.fine_tick_2 = 2048 + 300;
  CHECK(!deriveFullLegJointDiagnostics(r, JointKind::HIP, lo, hi).accepted);
}

void test_names() {
  g_case = "names";
  for (uint8_t i = 0; i <= static_cast<uint8_t>(FullLegFailure::STARTUP_POSE_OUTSIDE_CERTIFICATE); ++i) {
    CHECK(std::strcmp(toString(static_cast<FullLegFailure>(i)), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= static_cast<uint8_t>(FullLegStep::FAILED); ++i) {
    CHECK(std::strcmp(toString(static_cast<FullLegStep>(i)), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<FullLegFailure>(200)), "UNKNOWN") == 0);
}

void prepareStartup(Rig& r) {
  r.policy.transforms().clear();r.report_phases=false;
  for (const auto& ref:kStartupReference) {
    auto& j=r.backend.joint[ref.bus];j.pos=ref.q0;j.goal=3000;j.torque=false;
  }
  r.backend.joint[21].pos=2348;r.backend.joint[22].pos=1080;r.backend.joint[32].pos=1665;
}
void startStartup(Rig& r) {
  auto context=r.ctx();context.startup_motion_permit=true;
  CHECK(r.full.startStartupRecovery(context,r.t));
}
void test_startup_recovery() {
  {
    g_case="startup: correct reference, no witness, exact RF/RH residuals, sequential return, SAFE_OFF";
    Rig r(Leg::RF);prepareStartup(r);startStartup(r);finishRecovery(r);
    CHECK(r.full.status().step==FullLegStep::COMPLETE);
    CHECK_EQ(r.full.status().recovered_joints,12);CHECK_EQ(r.full.status().contacts_accepted,0);
    CHECK(r.policy.transforms().empty());CHECK_EQ(r.torqueOnCount(),0);
    std::vector<uint8_t> returns;
    for (const auto& event:r.backend.events) {
      if (event.kind==Ev::GOAL && event.torque_before) {
        returns.push_back(event.bus);CHECK(event.tick==startupReference(event.bus)->q0);
      }
      if (event.kind==Ev::TORQUE) {
        const auto previous=std::find_if(r.backend.events.begin(),r.backend.events.end(),[&](const Event& e){
          return e.kind==Ev::GOAL && e.bus==event.bus && !e.torque_before && e.tick==e.position;});
        CHECK(previous!=r.backend.events.end());
      }
    }
    CHECK(returns==std::vector<uint8_t>({21,22,32}));
    for (const auto& ref:kStartupReference) CHECK(std::abs(r.backend.joint[ref.bus].position()-ref.q0)<=10);
    CHECK(!r.full.startPostAbortRecovery(r.req(),r.ctx(),r.t));
  }
  for (int fault=0;fault<9;++fault) {
    g_case="startup: geometric rejection and failures keep SAFE_OFF";
    Rig r(Leg::RF);prepareStartup(r);
    if (fault==0) r.backend.joint[22].pos=2106;
    if (fault==1) r.backend.joint[21].read_fails=true;
    if (fault==2) r.backend.joint[13].pos=1990; // absolute support band, not drift about a wrong entry
    startStartup(r);
    Hook hook=once([](Rig& rig){return rig.full.status().phase==CalibrationPhase::RETURN_LOWER_HELD &&
                                  rig.full.status().step==FullLegStep::MOVE_MONITOR;},[fault](Rig& rig){
      if(fault==3) rig.backend.joint[21].current_override=200;
      if(fault==4) rig.backend.joint[21].temperature=80;
      if(fault==5) {rig.backend.joint[21].has_stop_low=true;rig.backend.joint[21].stop_low=2200;}
      if(fault==6) rig.permit=false;
      if(fault==7) rig.backend.joint[32].pos=1649; // q=409 outside absolute 388..408
      if(fault==8) rig.backend.joint[21].read_fails=true;
    });
    finishRecovery(r,hook);
    CHECK(r.full.status().step==FullLegStep::FAILED);
    if(fault<3) for(const auto& event:r.backend.events) CHECK(event.kind==Ev::SAFE_OFF);
    CHECK_EQ(r.torqueOnCount(),0);CHECK(r.policy.transforms().empty());
  }
  {
    g_case="startup cannot use an ordinary permit or caller-crafted request";
    Rig r(Leg::RF);prepareStartup(r);
    CHECK(!r.full.startStartupRecovery(r.ctx(),r.t));
    auto request=r.req();request.startup_recovery=true;request.recovery_only=true;
    CHECK(!r.full.start(request,r.ctx(),r.t));CHECK(r.backend.events.empty());
  }
}

}  // namespace

int main() {
  test_every_leg_completes_the_full_v25_sequence();
  test_v25_lf_hardware_contacts_replayed_exactly();
  test_evidence_maps_scout_and_fine_passes();
  test_partial_scout_step_all_24_endpoints();
  test_thermal_confirmation_in_the_sequence();
  test_stop_one_tick_before_the_corridor_entry_fails_closed();
  test_realistic_servo_behaviour_still_completes();
  test_recovery_only_run_commands_all_twelve_joints();
  test_recovery_commands_a_joint_already_at_q0();
  test_recovery_out_of_range_moves_nothing();
  test_recovery_that_does_not_stay_at_q0_fails();
  test_recovery_move_that_never_settles_times_out();
  test_held_joint_violations_fail_closed();
  test_held_speed_transient_is_not_an_abort();
  test_held_speed_transient_records_are_bounded();
  test_held_position_drift_boundary();
  test_speed_gates_the_promotion_to_held();
  test_speed_gates_the_recovery_settle();
  test_probe_failures_stop_the_sequence_at_the_right_count();
  test_transition_and_parking_failures();
  test_bystanders_and_passive_participants();
  test_energize_write_failures();
  test_safe_off_is_retried_until_verified();
  test_diagnostics_rejection_returns_the_leg_then_fails();
  test_start_refuses_incomplete_requests();
  test_diagnostics_math_is_v25();
  test_post_abort_recovery();
  test_post_abort_all_supported_phases();
  test_names();
  test_startup_recovery();

  std::printf("test_full_leg_calibration_executor: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
