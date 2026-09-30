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
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = kSearchStaticToleranceTicks + 2;
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
    b.sequence_leg = full.leg();
    b.sequence_phase = full.sequencePhase();
    b.sequence_prerequisites_verified = full.prerequisitesVerified();
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
    if (pp != ContactProbePhase::STEP_PENDING && pp != ContactProbePhase::STEP_MONITORING &&
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
    for (uint8_t i = 0; i < n; ++i) frame.add(buses[i], backend.joint[buses[i]].sample(t));
    full.update(ctx(), t, frame, safe_off_frame);

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
  // Six contacts, each at its physical stop, two repeatable passes.
  for (uint8_t k = 0; k < kJointKindCount; ++k) {
    for (uint8_t side = 0; side < kContactSideCount; ++side) {
      const ContactEvidence& e = rig.full.contact(static_cast<JointKind>(k), static_cast<ContactSide>(side));
      CHECK(e.has_measurement);
      CHECK(e.witness.accepted());
      CHECK((int)e.key.leg == (int)rig.leg);
      CHECK((int)e.key.joint == (int)k);
      CHECK((int)e.key.side == (int)side);
      const int stop = rig.stopTick(k, side);
      CHECK(std::abs((int)e.coarse_tick - stop) <= 2);
      CHECK(std::abs((int)e.fine_tick_1 - stop) <= 2);
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

// --- held prerequisites ---------------------------------------------------------

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
      {"held HIP excessive speed during UPPER MIN", Leg::RF, CalibrationPhase::UPPER_MIN, kHip,
       [](simk::SimJoint& j) { j.speed_override = 60; }, FullLegFailure::HELD_JOINT_SPEED, 0},
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
    rig.run(once(probing(hc.phase), [&](Rig& r) { hc.act(r.backend.joint[b]); }));
    CHECK_EQ((int)rig.full.status().step, (int)FullLegStep::FAILED);
    CHECK_EQ((int)rig.full.status().failure, (int)hc.expect);
    CHECK_EQ((int)rig.full.status().failed_phase, (int)hc.phase);
    CHECK_EQ(rig.full.status().contacts_accepted, hc.contacts);
    // The held joint is watched - no telemetry-loss case aside, the failure
    // came within a few ticks, not after the probe finished.
    CHECK(rig.full.contact(rig.full.probeRequest().endpoint_joint,
                           rig.full.probeRequest().endpoint_side).has_measurement == false);
    for (const JointOracle& o : kOracle) rig.backend.joint[o.bus].read_fails = false;
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
  lo.coarse_tick = lo.fine_tick_1 = 2048 - 487;  // V25 LF HIP MIN
  hi.coarse_tick = hi.fine_tick_1 = 2048 + 448;  // V25 LF HIP MAX
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
  // Scale out of band.
  hi.coarse_tick = hi.fine_tick_1 = 2048 + 300;
  CHECK(!deriveFullLegJointDiagnostics(r, JointKind::HIP, lo, hi).accepted);
}

void test_names() {
  g_case = "names";
  for (uint8_t i = 0; i <= static_cast<uint8_t>(FullLegFailure::OPERATOR_ABORT); ++i) {
    CHECK(std::strcmp(toString(static_cast<FullLegFailure>(i)), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= static_cast<uint8_t>(FullLegStep::FAILED); ++i) {
    CHECK(std::strcmp(toString(static_cast<FullLegStep>(i)), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<FullLegFailure>(200)), "UNKNOWN") == 0);
}

}  // namespace

int main() {
  test_every_leg_completes_the_full_v25_sequence();
  test_v25_lf_hardware_contacts_replayed_exactly();
  test_stop_one_tick_before_the_corridor_entry_fails_closed();
  test_realistic_servo_behaviour_still_completes();
  test_recovery_only_run_commands_all_twelve_joints();
  test_recovery_commands_a_joint_already_at_q0();
  test_recovery_out_of_range_moves_nothing();
  test_recovery_that_does_not_stay_at_q0_fails();
  test_recovery_move_that_never_settles_times_out();
  test_held_joint_violations_fail_closed();
  test_probe_failures_stop_the_sequence_at_the_right_count();
  test_transition_and_parking_failures();
  test_bystanders_and_passive_participants();
  test_energize_write_failures();
  test_safe_off_is_retried_until_verified();
  test_diagnostics_rejection_returns_the_leg_then_fails();
  test_start_refuses_incomplete_requests();
  test_diagnostics_math_is_v25();
  test_names();

  std::printf("test_full_leg_calibration_executor: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
