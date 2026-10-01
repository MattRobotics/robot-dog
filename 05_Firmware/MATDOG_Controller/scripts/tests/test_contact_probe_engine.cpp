// Offline tests for the staged calibration endpoint search
// (src/calibration/ContactProbeEngine.*), 2026-09-29; the LF V25 coarse
// contact scout restored 2026-09-30 (numbered cases [T1]..[T15] below).
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, CalibrationExecutionEngine,
// ActuatorAuthorityArbiter, Geometry V5 profile and checked resolvers, and drives
// them against kinematic_servo_sim.h - a synthetic ST3215 model reproducing the
// LF_UPPER hardware behaviours (settling 4-5 ticks short of a goal; a hard stop
// ~23 ticks PAST the modelled contact; friction plateaus) and the LF V25
// ones the coarse scout exists for (a chamfer/friction plateau a fine pass
// stalls on; a contact exactly on the corridor entry).
//
// NO HARDWARE VALIDATION, no fabricated measurement: every stop, plateau and
// current in this file is a synthetic construct.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <vector>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/calibration/ContactProbeEngine.h"
#include "kinematic_servo_sim.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::ActuatorRuntime;
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
constexpr ContactSide kSides[2] = {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE};
// LF_UPPER's real MIN stop, found by hand on 2026-09-29: ~23 ticks past the
// modelled contact.
constexpr int kHardwareStopBeyondContact = 23;
// The LF UPPER MIN hardware offset, on both sides. With the final partial
// coarse-scout step every stop down to guard - 18 is found whatever the grid
// phase (from q0 the MAX-side 64-tick grid ends at depth 1399, so +23 = 1410
// is reached by the partial step - see test_final_partial_scout_step).
int reachableBeyond(ContactSide) { return kHardwareStopBeyondContact; }

JointIdentity upperOf(const UpperCase& u) {
  JointIdentity id{};
  id.leg = u.leg;
  id.joint = JointKind::UPPER;
  setPhysicalUnit(&id, u.unit);
  return id;
}

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

MotionDeadmanConfig backoffDeadman() {  // Controller::begin()'s figures
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = kSearchTelemetryTimeoutMs;  // Controller::begin()
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = kSearchBackoffSettleToleranceTicks;
  c.nominal_travel_ticks_per_s = kSearchMinExpectedTicksPerSecond;
  return c;
}

struct WriteLog {
  uint16_t tick;
  uint8_t pass;
  ContactSearchStage stage;
  MotionProfile profile;
};

struct ProbeRig {
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  ActuatorRuntime runtime;
  CalibrationExecutionEngine engine;
  simk::SimBackend backend;
  CalibrationGeometryProfile profile;
  ContactProbeEngine probe;
  AuthorityLease lease{};
  CalibrationSearchCorridor corridor{};
  UpperCase u;
  ContactSide side;
  std::vector<WriteLog> log;
  int start_position = 0;  // where the joint was when the probe started

  ProbeRig(const UpperCase& uc, ContactSide s) : u(uc), side(s) {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    runtime.begin(&policy, &backend);
    engine.begin(&policy, &runtime, &profile, &actuator::geometry_data::kProvenance);
    ContactProbeConfig config{};
    config.backoff_deadman = backoffDeadman();
    probe.begin(&policy, &runtime, &engine, &profile, &actuator::geometry_data::kProvenance, config);
    const actuator::JointTransform t = promotedTransform(upperOf(u), u.q0);
    CHECK(policy.transforms().admit(t));
    CHECK(actuator::resolveCalibrationSearchCorridor(profile, actuator::geometry_data::kProvenance,
                                                     t, u.leg, JointKind::UPPER, side,
                                                     &corridor) ==
          actuator::TargetResolveStatus::OK);
    AuthorityResult r = arbiter.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE,
                                        &lease);
    CHECK(r == AuthorityResult::GRANTED);
    setPermit(true);
    simk::SimJoint& j = joint();
    j.pos = u.q0;
  }

  // The parked flag exactly as Controller derives it for this endpoint.
  void setPermit(bool active) {
    const actuator::GeometryEndpointRecord* ep = profile.findEndpoint(u.leg, JointKind::UPPER, side);
    actuator::CalibrationBootstrapContext b{};
    b.session_active = true;
    b.origin = CalibrationOrigin::LIVE_SESSION;
    b.motion_permit_active = active;
    b.motion_permit_generation = 1;
    b.motion_permit_session_id = 1;
    b.motion_permit_authority_generation = lease.generation;
    b.auxiliary_parked = ep != nullptr && ep->parking == actuator::ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND;
    b.parked_leg = u.leg;
    b.parked_joint = JointKind::UPPER;
    b.parked_side = side;
    policy.setBootstrapContext(b);
  }

  simk::SimJoint& joint() { return backend.joint[u.bus]; }
  int depth(int tick) const { return actuator::searchDepth(corridor, static_cast<uint16_t>(tick)); }
  int tickAt(int d) const { return corridor.home_tick + corridor.probe_sign * d; }

  // A hard stop `beyond_contact` ticks past the canonical contact (negative:
  // short of it), on the probe side; the other side left free.
  void placeStop(int beyond_contact) {
    const int stop = tickAt(depth(corridor.contact_tick) + beyond_contact);
    simk::SimJoint& j = joint();
    if (corridor.probe_sign < 0) {
      j.has_stop_low = true;
      j.stop_low = stop;
    } else {
      j.has_stop_high = true;
      j.stop_high = stop;
    }
  }
  int stopTick() const {
    const simk::SimJoint& j = backend.joint[u.bus];
    return static_cast<int>(corridor.probe_sign < 0 ? j.stop_low : j.stop_high);
  }

  ContactProbeRequest request() const {
    ContactProbeRequest r{};
    r.joint = upperOf(u);
    r.bus_id = u.bus;
    r.endpoint_leg = u.leg;
    r.endpoint_joint = JointKind::UPPER;
    r.endpoint_side = side;
    r.corridor = corridor;
    r.repeatability_tolerance_ticks = 16;
    r.expected_torque_limit = 500;
    return r;
  }

  ContactProbeContext ctx() const {
    ContactProbeContext c{};
    c.session_active = true;
    c.origin = CalibrationOrigin::LIVE_SESSION;
    c.lease = lease;
    c.mode = OperatingMode::MAINTENANCE;
    return c;
  }
};

using Hook = std::function<void(ProbeRig&, uint32_t, bool*, TelemetrySample*)>;

// Ticks the probe like Controller::updateFullLegCalibration(): physics, then
// this tick's sample, then update(). Stops on a terminal phase.
uint32_t runProbe(ProbeRig& rig, uint32_t dt_ms = 10, uint32_t limit_ms = 180000,
                  const Hook& hook = nullptr) {
  uint32_t t = 1000;
  rig.start_position = rig.joint().position();
  CHECK(rig.probe.start(rig.request(), rig.ctx(), t));
  const uint32_t started = t;
  while (rig.probe.active() && t - started < limit_ms) {
    t += dt_ms;
    rig.backend.advance(t, dt_ms);
    bool available = true;
    TelemetrySample s = rig.joint().sample(t);
    if (hook) hook(rig, t, &available, &s);
    const size_t before = rig.backend.writes.size();
    rig.probe.update(rig.ctx(), t, available, s);
    for (size_t i = before; i < rig.backend.writes.size(); ++i) {
      rig.log.push_back(WriteLog{rig.backend.writes[i].tick, rig.probe.status().pass,
                                 rig.probe.status().stage, rig.backend.writes[i].profile});
    }
    if (rig.probe.status().phase == ContactProbePhase::SAFE_OFF_REQUIRED) {
      rig.joint().torque = false;  // the caller's independent SAFE_OFF
    }
  }
  return t - started;
}

// --- 1. the V25 detector rule by rule ---------------------------------------

void test_detector_rules() {
  g_case = "detector: V25 HybridContactDetector rules";
  ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);  // LF MIN, probe_sign -1
  const CalibrationSearchCorridor& c = rig.corridor;
  const int entry_d = rig.depth(c.entry_tick);
  const uint16_t start = static_cast<uint16_t>(rig.tickAt(entry_d - 150));
  auto fresh = [&](int32_t acceptance_entry) {
    ContactSearchDetector d;
    d.begin(c, start, acceptance_entry);
    return d;
  };
  const uint16_t stuck = static_cast<uint16_t>(rig.tickAt(entry_d + 40));   // inside corridor
  const uint16_t target12 = static_cast<uint16_t>(rig.tickAt(entry_d + 52));  // 12 ahead

  // A: the full sequence - new target, 4 start-up samples, 3 persistent.
  {
    ContactSearchDetector d = fresh(entry_d);
    CHECK(d.observe(stuck, 0, target12) == ContactDetectorState::FREE_MOTION);  // new target
    for (int i = 0; i < 4; ++i) CHECK(d.observe(stuck, 0, target12) == ContactDetectorState::FREE_MOTION);
    CHECK(d.observe(stuck, 0, target12) == ContactDetectorState::CONTACT_SUSPECTED);
    CHECK(d.observe(stuck, 0, target12) == ContactDetectorState::CONTACT_SUSPECTED);
    CHECK(d.observe(stuck, 0, target12) == ContactDetectorState::CONTACT_CONFIRMED);
  }
  // B: speed above 10 is not a stop, however long.
  {
    ContactSearchDetector d = fresh(entry_d);
    for (int i = 0; i < 20; ++i) CHECK(d.observe(stuck, 11, target12) == ContactDetectorState::FREE_MOTION);
  }
  // C: progress above 2 ticks per sample is motion.
  {
    ContactSearchDetector d = fresh(entry_d);
    d.observe(stuck, 0, target12);
    int pos = stuck;
    for (int i = 0; i < 20; ++i) {
      pos += c.probe_sign * 3;
      CHECK(d.observe(static_cast<uint16_t>(pos), 0, static_cast<uint16_t>(pos + c.probe_sign * 40)) !=
            ContactDetectorState::CONTACT_CONFIRMED);
    }
  }
  // D: within 10 ticks of the target is "arrived" (the servo settling short).
  {
    ContactSearchDetector d = fresh(entry_d);
    const uint16_t target10 = static_cast<uint16_t>(rig.tickAt(entry_d + 50));
    for (int i = 0; i < 20; ++i) CHECK(d.observe(stuck, 0, target10) == ContactDetectorState::FREE_MOTION);
  }
  // E: the same persistence OUTSIDE the corridor is an early stall, never contact.
  {
    ContactSearchDetector d = fresh(entry_d);
    const uint16_t early = static_cast<uint16_t>(rig.tickAt(entry_d - 60));
    const uint16_t ahead = static_cast<uint16_t>(rig.tickAt(entry_d - 40));  // 20 ahead > 16
    ContactDetectorState last = ContactDetectorState::FREE_MOTION;
    for (int i = 0; i < 8; ++i) last = d.observe(early, 0, ahead);
    CHECK(last == ContactDetectorState::EARLY_STALL);
  }
  // E2: outside the corridor an error of <= 16 is still "settling" (V36 evidence).
  {
    ContactSearchDetector d = fresh(entry_d);
    const uint16_t early = static_cast<uint16_t>(rig.tickAt(entry_d - 60));
    const uint16_t ahead = static_cast<uint16_t>(rig.tickAt(entry_d - 45));  // 15 ahead
    for (int i = 0; i < 12; ++i) CHECK(d.observe(early, 0, ahead) == ContactDetectorState::FREE_MOTION);
  }
  // F: less than 24 ticks of travel since the pass started is never contact.
  {
    ContactSearchDetector d;
    const uint16_t near_start = static_cast<uint16_t>(rig.tickAt(entry_d + 20));
    d.begin(c, near_start, entry_d);
    const uint16_t p = static_cast<uint16_t>(rig.tickAt(entry_d + 40));  // travel 20
    const uint16_t t = static_cast<uint16_t>(rig.tickAt(entry_d + 55));
    for (int i = 0; i < 12; ++i) CHECK(d.observe(p, 0, t) == ContactDetectorState::FREE_MOTION);
  }
  // G: a target behind the joint is not "pushing into" anything.
  {
    ContactSearchDetector d = fresh(entry_d);
    const uint16_t behind = static_cast<uint16_t>(rig.tickAt(entry_d + 20));
    for (int i = 0; i < 12; ++i) CHECK(d.observe(stuck, 0, behind) == ContactDetectorState::FREE_MOTION);
  }
  // H: the fine passes' adaptive bound admits up to 32 ticks HOME-ward of the scout.
  {
    const uint16_t shallow = static_cast<uint16_t>(rig.tickAt(entry_d - 20));
    const uint16_t ahead = static_cast<uint16_t>(rig.tickAt(entry_d - 5));
    ContactSearchDetector strict = fresh(entry_d);
    ContactSearchDetector adaptive = fresh(entry_d - 30);
    ContactDetectorState s1 = ContactDetectorState::FREE_MOTION, s2 = s1;
    for (int i = 0; i < 8; ++i) {
      s1 = strict.observe(shallow, 0, ahead);
      s2 = adaptive.observe(shallow, 0, ahead);
    }
    CHECK(s1 == ContactDetectorState::FREE_MOTION);  // 15 ahead, outside: settling
    CHECK(s2 == ContactDetectorState::CONTACT_CONFIRMED);
  }
  // I: a new target restarts start-up and persistence.
  {
    ContactSearchDetector d = fresh(entry_d);
    for (int i = 0; i < 7; ++i) d.observe(stuck, 0, target12);  // SUSPECTED x2
    const uint16_t next = static_cast<uint16_t>(rig.tickAt(entry_d + 58));
    CHECK(d.observe(stuck, 0, next) == ContactDetectorState::FREE_MOTION);
    for (int i = 0; i < 4; ++i) CHECK(d.observe(stuck, 0, next) == ContactDetectorState::FREE_MOTION);
    CHECK(d.observe(stuck, 0, next) == ContactDetectorState::CONTACT_SUSPECTED);
  }
  // J: an unread speed never counts as low.
  {
    ContactSearchDetector d = fresh(entry_d);
    for (int i = 0; i < 12; ++i) CHECK(d.observe(stuck, -1, target12) == ContactDetectorState::FREE_MOTION);
  }
  CHECK_EQ(searchBaselineThreshold(30, 0), 35);   // median + max(4*MAD, 5)
  CHECK_EQ(searchBaselineThreshold(30, 3), 42);
}

// --- 1b. the V25 coarse-scout rules, pure --------------------------------------

CalibrationSearchCorridor syntheticCorridor(int8_t sign, uint16_t entry, uint16_t guard) {
  CalibrationSearchCorridor c{};
  c.probe_sign = sign;
  c.home_tick = 2048;
  c.entry_tick = entry;
  c.guard_tick = guard;
  c.contact_tick = entry;
  c.urdf_limit_tick = static_cast<uint16_t>(entry + sign * 64);
  return c;
}

void test_v25_scout_rules() {
  g_case = "[T7,T8] V25 fine_contact_reproduces_coarse_depth / adaptive bounds, verbatim";
  // matdog_test.rs fine_contact_scout_depth_gate_is_direction_independent_and_bounded.
  for (int8_t sign : {(int8_t)1, (int8_t)-1}) {
    const CalibrationSearchCorridor c = syntheticCorridor(sign, sign > 0 ? 2800 : 1300,
                                                          sign > 0 ? 2928 : 1172);
    const uint16_t scout = sign > 0 ? 2900 : 1200;
    const uint16_t one_step_before = static_cast<uint16_t>(scout - sign * 8);
    const uint16_t too_early = static_cast<uint16_t>(scout - sign * 9);
    const uint16_t beyond_scout = static_cast<uint16_t>(scout + sign * 4);
    CHECK(searchFineContactReproducesScout(c, scout, scout));
    CHECK(searchFineContactReproducesScout(c, one_step_before, scout));
    CHECK(!searchFineContactReproducesScout(c, too_early, scout));
    CHECK(searchFineContactReproducesScout(c, beyond_scout, scout));
    CHECK_EQ(searchScoutLagTicks(c, too_early, scout), 9);
    CHECK_EQ(searchScoutLagTicks(c, beyond_scout, scout), 0);
  }
  // "Normal V23 fine/coarse offsets remain valid" and the V23 M11 MAX chamfer.
  const CalibrationSearchCorridor neg = syntheticCorridor(-1, 1300, 1172);
  const CalibrationSearchCorridor pos = syntheticCorridor(1, 2800, 2928);
  CHECK(searchFineContactReproducesScout(neg, 1438, 1434));
  CHECK(searchFineContactReproducesScout(pos, 3443, 3446));
  CHECK(searchFineContactReproducesScout(pos, 3093, 3097));
  CHECK(!searchFineContactReproducesScout(neg, 1666, 1652));  // 14 ticks: a chamfer plateau
  CHECK_EQ(kSearchFineScoutLagToleranceTicks, kSearchFineStepTicks);
  // matdog_test.rs v41_adaptive_fine_corridor_accepts_observed_hip_max_plateau_without_moving_guard:
  // LF HIP MAX, static bounds 1472..1600, scout 1600 -> adaptive high 1632; 1617 inside.
  const CalibrationSearchCorridor hip_max = syntheticCorridor(-1, 1600, 1472);
  CHECK_EQ(searchAdaptiveAcceptanceEntryDepth(hip_max, 1600), 2048 - 1632);
  CHECK(actuator::searchDepth(hip_max, 1617) >= searchAdaptiveAcceptanceEntryDepth(hip_max, 1600));
  CHECK(actuator::searchDepth(hip_max, 1617) < actuator::searchDepth(hip_max, hip_max.entry_tick));
  // Extended HOME-ward only: a scout deep in the corridor never moves the entry deeper.
  CHECK_EQ(searchAdaptiveAcceptanceEntryDepth(hip_max, 1480),
           actuator::searchDepth(hip_max, hip_max.entry_tick));
  CHECK_EQ(kSearchAdaptiveScoutTicks, 32);
}

// --- 2. the V25 stage order and its step rules -------------------------------

struct Seg {
  uint8_t pass;
  ContactSearchStage stage;
  size_t first;
  size_t count;
};

std::vector<Seg> segments(const ProbeRig& rig) {
  std::vector<Seg> out;
  for (size_t i = 0; i < rig.log.size(); ++i) {
    const WriteLog& w = rig.log[i];
    if (!out.empty() && out.back().pass == w.pass && out.back().stage == w.stage) {
      ++out.back().count;
    } else {
      out.push_back(Seg{w.pass, w.stage, i, 1});
    }
  }
  return out;
}

using S = ContactSearchStage;

// V25 measure_lf_contact_side_efficient(), write by write: baseline, coarse
// scout (its pre-corridor steps labelled transit), release, backoff, fine 1,
// release, backoff, fine 2, release.
bool isV25Order(const std::vector<Seg>& segs) {
  const std::vector<std::pair<int, S>> full = {
      {0, S::BASELINE}, {0, S::COARSE_TRANSIT}, {0, S::COARSE_SCOUT}, {0, S::RELEASE},
      {0, S::BACKOFF},  {1, S::FINE_SEARCH},    {1, S::RELEASE},      {1, S::BACKOFF},
      {2, S::FINE_SEARCH}, {2, S::RELEASE}};
  std::vector<std::pair<int, S>> got;
  for (const Seg& s : segs) got.push_back({s.pass, s.stage});
  if (got == full) return true;
  std::vector<std::pair<int, S>> no_transit = full;
  no_transit.erase(no_transit.begin() + 1);
  return got == no_transit;
}

// Rules for every write of every attempt, whatever its outcome.
void checkWriteBounds(ProbeRig& rig) {
  const int guard_d = rig.depth(rig.corridor.guard_tick);
  for (const WriteLog& w : rig.log) {
    CHECK(w.profile == MotionProfile::CALIBRATION_SEARCH);
    CHECK(actuator::searchCorridorAdmits(rig.corridor, w.tick));
    CHECK(rig.depth(w.tick) <= guard_d);  // never past the guard
  }
  // No fine metrology write before an accepted scout.
  bool scouted = false;
  for (const WriteLog& w : rig.log) {
    if (w.pass == 0 && w.stage == S::RELEASE) scouted = true;
    if (w.stage == S::FINE_SEARCH || w.pass > 0) CHECK(scouted);
  }
}

void checkStepDiscipline(ProbeRig& rig) {
  const ContactProbeStatus& st = rig.probe.status();
  const int entry_d = rig.depth(rig.corridor.entry_tick);
  checkWriteBounds(rig);
  const std::vector<Seg> segs = segments(rig);
  CHECK(isV25Order(segs));
  if (!isV25Order(segs)) return;
  int prev = -1;
  for (const Seg& sg : segs) {
    for (size_t i = sg.first; i < sg.first + sg.count; ++i) {
      const WriteLog& w = rig.log[i];
      const int d = rig.depth(w.tick);
      const int delta = prev < 0 ? 0 : d - rig.depth(prev);
      switch (sg.stage) {
        case S::BASELINE:  // one 64-tick move from where the joint was
          CHECK_EQ(sg.count, 1);
          CHECK_EQ(d - rig.depth(rig.start_position), 64);
          break;
        case S::COARSE_TRANSIT:
        case S::COARSE_SCOUT:
          CHECK_EQ(sg.pass, 0);
          if (sg.stage == S::COARSE_TRANSIT) CHECK(d < entry_d);
          if (sg.stage == S::COARSE_SCOUT) CHECK(d >= entry_d);
          // 64-tick target steps; the first one from where the baseline left
          // the joint (inside its 10-tick settle band); at most one final
          // partial step, which targets the guard itself and is the last.
          if (rig.log[i - 1].stage == S::BASELINE) {
            CHECK(delta >= 64 - 10 && delta <= 64 + 10);
          } else if (delta != 64) {
            CHECK(d == rig.depth(rig.corridor.guard_tick));
            CHECK(delta > 0 && delta < 64);
            CHECK(i + 1 == rig.log.size() || rig.log[i + 1].stage == S::RELEASE);
            CHECK_EQ(st.scout_partial_step_ticks, delta);
          }
          break;
        case S::RELEASE: {
          CHECK_EQ(sg.count, 1);
          const uint16_t want = sg.pass == 0 ? st.scout_tick
                                             : sg.pass == 1 ? st.pass1_contact_tick
                                                            : st.pass2_contact_tick;
          CHECK_EQ(w.tick, want);
          CHECK(d <= rig.depth(prev));  // never deeper than the step that met the stop
          break;
        }
        case S::BACKOFF:
          CHECK_EQ(sg.count, 1);
          CHECK_EQ(d, rig.depth(sg.pass == 0 ? st.scout_tick : st.pass1_contact_tick) - 96);
          break;
        case S::FINE_SEARCH:
          CHECK(sg.pass == 1 || sg.pass == 2);
          // Exactly 8 target ticks; the first one of a pass from where the
          // backoff left the joint (inside its 12-tick arrival band).
          if (rig.log[i - 1].stage == S::BACKOFF) {
            CHECK(delta >= 8 - 12 && delta <= 8 + 12);
          } else {
            CHECK_EQ(delta, 8);
          }
          break;
        default:
          CHECK(false);
      }
      prev = w.tick;
    }
  }
  CHECK_EQ(rig.log.back().tick, st.pass2_contact_tick);  // ends released on fine 2
  CHECK_EQ(st.target_tick, st.pass2_contact_tick);
}

// --- [T1] [T6] every search: baseline, coarse scout, two fine passes -----------

void test_real_stop_detected_every_leg_both_sides() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      for (int beyond : {reachableBeyond(side), 0, -5, -20}) {
        g_case = "[T1,T6] real stop: baseline, coarse scout, fine 1, fine 2 - COMPLETE at the stop";
        ProbeRig rig(u, side);
        rig.placeStop(beyond);
        const uint32_t elapsed = runProbe(rig);
        const ContactProbeStatus& st = rig.probe.status();
        CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
        CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NONE);
        CHECK(st.scout_valid);
        CHECK(std::abs(st.scout_tick - rig.stopTick()) <= 1);
        CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
        CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
        CHECK(rig.probe.witness().accepted());
        CHECK_EQ(rig.probe.witness().max_deviation_ticks,
                 std::abs(st.pass1_contact_tick - st.pass2_contact_tick));
        CHECK_EQ(st.plateau_bypass_count, 0);
        CHECK_EQ(st.kinematic_plateau_count, 0);
        CHECK(st.baseline_samples >= kSearchBaselineMinSamples);
        checkStepDiscipline(rig);
        // Bounded contact energy (V25): a coarse-scout target runs at most one
        // coarse step + the settle band past the physical stop, a fine target
        // one fine step + the band. TorqueLimit 500 and the 200-raw abort
        // bound the pressure behind it.
        for (const WriteLog& w : rig.log) {
          const int past = rig.depth(w.tick) - rig.depth(rig.stopTick());
          if (w.stage == S::COARSE_SCOUT || w.stage == S::COARSE_TRANSIT) CHECK(past <= 64 + 10);
          if (w.stage == S::FINE_SEARCH) CHECK(past <= 8 + 10);
        }
        CHECK(elapsed < (side == ContactSide::MIN_SIDE ? 20000u : 30000u));
        CHECK(rig.joint().torque);  // COMPLETE: SAFE_OFF is the executor's next step
      }
    }
  }
}

// --- [T2] the scout is not free-space transit ------------------------------------

void test_scout_is_distinct_from_free_space_transit() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      g_case = "[T2] transit steps stay short of the corridor; the scout is a confirmed contact in it";
      ProbeRig rig(u, side);
      rig.placeStop(reachableBeyond(rig.side));
      runProbe(rig);
      const ContactProbeStatus& st = rig.probe.status();
      CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
      int transit = 0, scout_steps = 0;
      for (const WriteLog& w : rig.log) {
        transit += w.stage == S::COARSE_TRANSIT;
        scout_steps += w.stage == S::COARSE_SCOUT;
      }
      CHECK(transit > 0);
      CHECK(scout_steps > 0);
      // A detector-confirmed contact inside the STATIC corridor - not where
      // the baseline or any transit step ended.
      CHECK(actuator::searchCorridorAccepts(rig.corridor, st.scout_tick));
      CHECK(rig.depth(st.scout_tick) >= rig.depth(rig.log[0].tick) + 24);
      CHECK_EQ(st.last_candidate_tick, st.pass2_contact_tick);
    }
  }
  {
    g_case = "[T2] free-space travel through the whole corridor never makes a scout";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    runProbe(rig);  // no stop at all
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
    CHECK(!st.scout_valid);
    CHECK_EQ(st.scout_tick, 0);
    for (const WriteLog& w : rig.log) {
      CHECK_EQ(w.pass, 0);
      CHECK(w.stage == S::BASELINE || w.stage == S::COARSE_TRANSIT || w.stage == S::COARSE_SCOUT);
    }
    checkWriteBounds(rig);
  }
}

// --- [T3] [T4] [T10] the scout is reference evidence, never metrology ------------

void shiftStop(ProbeRig& r, int delta_depth) {  // + = deeper along the probe direction
  simk::SimJoint& j = r.joint();
  if (r.corridor.probe_sign < 0) j.stop_low -= delta_depth; else j.stop_high += delta_depth;
}

void test_scout_is_reference_evidence_not_metrology() {
  {
    g_case = "[T3,T4,T10] coarse loading: scout 6 deeper, metrology = the two fine passes";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(6);  // synthetic: the 64-tick press deflects the stop 6 deeper
    bool restored = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!restored && r.probe.status().scout_valid &&
          r.probe.status().phase == ContactProbePhase::BACKOFF_MONITORING) {
        restored = true;
        shiftStop(r, -6);
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.depth(st.scout_tick) - rig.depth(rig.stopTick()) - 6) <= 1);  // stored
    CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
    const ContactWitness w = rig.probe.witness();
    CHECK(w.accepted());
    // Repeatability is fine-to-fine: the 6-tick scout offset is not in it.
    CHECK_EQ(w.max_deviation_ticks, std::abs(st.pass1_contact_tick - st.pass2_contact_tick));
    CHECK(w.max_deviation_ticks <= 2);
  }
  {
    // matdog_test.rs v38_2026_08_01_failure_is_coarse_loading_not_mechanical_change:
    // scout 1416, fine 1440 - 24 apart, beyond the 16-tick band - is a pass.
    g_case = "[T4,T10] V38: a scout 24 ticks short of the fine contacts still COMPLETEs";
    ProbeRig rig(kUppers[2], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;  // synthetic: a transient obstruction during the coarse pass only
    j.plateau_tick = rig.tickAt(rig.depth(rig.stopTick()) - 24);
    j.plateau_breakaway = 1000;
    bool cleared = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!cleared && r.probe.status().scout_valid &&
          r.probe.status().phase == ContactProbePhase::BACKOFF_MONITORING) {
        cleared = true;
        r.joint().plateau = false;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.depth(st.scout_tick) - (rig.depth(rig.stopTick()) - 24)) <= 1);
    CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.scout_tick - st.pass1_contact_tick) > 16);
    CHECK(rig.probe.witness().accepted());
    checkStepDiscipline(rig);
  }
}

// --- [T5] the scout is followed by the reviewed backoff --------------------------

void test_scout_backoff_is_the_reviewed_backoff() {
  {
    g_case = "[T5] after the scout: release on it, then 96 ticks back";
    ProbeRig rig(kUppers[3], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    const std::vector<Seg> segs = segments(rig);
    for (size_t k = 0; k + 1 < segs.size(); ++k) {
      if (segs[k].pass == 0 && segs[k].stage == S::RELEASE) {
        CHECK_EQ(rig.log[segs[k].first].tick, st.scout_tick);
        CHECK(segs[k + 1].stage == S::BACKOFF && segs[k + 1].pass == 0);
        CHECK_EQ(rig.depth(rig.log[segs[k + 1].first].tick), rig.depth(st.scout_tick) - 96);
      }
    }
  }
  {
    g_case = "[T5] current still high after the SCOUT backoff: CURRENT_NOT_RECOVERED, no fine pass";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().scout_valid) r.joint().current_override = 150;
    });
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::CURRENT_NOT_RECOVERED);
    CHECK_EQ(rig.probe.status().pass, 0);
    CHECK_EQ(rig.probe.status().pass1_contact_tick, 0);
  }
  {
    g_case = "[T5] an obstruction on the SCOUT backoff path is an anomaly, never contact";
    ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool blocked = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      const ContactProbeStatus& st = r.probe.status();
      if (!blocked && st.scout_valid) {
        blocked = true;
        simk::SimJoint& j = r.joint();
        const double back = r.tickAt(r.depth(st.scout_tick) - 40);
        if (r.corridor.probe_sign < 0) { j.has_stop_high = true; j.stop_high = back; }
        else { j.has_stop_low = true; j.stop_low = back; }
      }
    });
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);
    CHECK_EQ(rig.probe.status().pass1_contact_tick, 0);
  }
}

// --- [T7] [T8] [T9] both fine passes use the scout -------------------------------

void test_fine_pass_plateau_is_bypassed_against_the_scout() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      g_case = "[T7,T8] a fine-pass plateau 20 ticks short of the scout is stepped past, both passes";
      ProbeRig rig(u, side);
      rig.placeStop(reachableBeyond(rig.side));
      bool armed = false;
      uint16_t bypass_after_pass1 = 0;
      runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
        const ContactProbeStatus& st = r.probe.status();
        if (!armed && st.pass == 1) {
          armed = true;
          simk::SimJoint& j = r.joint();
          j.plateau = true;
          j.plateau_tick = r.tickAt(r.depth(st.scout_tick) - 20);
          j.plateau_breakaway = 18;  // holds through a full fine step of error
        }
        if (st.pass == 1 && st.pass1_contact_tick != 0 && bypass_after_pass1 == 0) {
          bypass_after_pass1 = st.plateau_bypass_count;
        }
      });
      const ContactProbeStatus& st = rig.probe.status();
      CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
      CHECK(bypass_after_pass1 >= 1);                       // fine pass 1 bypassed it
      CHECK(st.plateau_bypass_count > bypass_after_pass1);  // ...and so did fine pass 2
      CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
      CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
      checkStepDiscipline(rig);
    }
  }
  {
    g_case = "[T8] V23 M11 chamfer: 14 ticks short of the scout is bypassed";
    ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool armed = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!armed && r.probe.status().pass == 1) {
        armed = true;
        r.joint().plateau = true;
        r.joint().plateau_tick = r.tickAt(r.depth(r.probe.status().scout_tick) - 14);
        r.joint().plateau_breakaway = 18;
      }
    });
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(rig.probe.status().plateau_bypass_count >= 2);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    g_case = "[T8] within one fine step of the scout (6 ticks) IS the contact, per V25";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool armed = false;
    int plateau = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!armed && r.probe.status().pass == 1) {
        armed = true;
        plateau = r.tickAt(r.depth(r.probe.status().scout_tick) - 6);
        r.joint().plateau = true;
        r.joint().plateau_tick = plateau;
        r.joint().plateau_breakaway = 60;  // holds any fine-step error, yields to the backoff
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK_EQ(st.plateau_bypass_count, 0);
    CHECK(std::abs(st.pass1_contact_tick - plateau) <= 1);
    CHECK(std::abs(st.pass2_contact_tick - plateau) <= 1);
  }
  {
    // The case V25 kept the coarse scout for; without it (#34/#35) this
    // plateau was indistinguishable from a stop and was frozen as the endpoint.
    g_case = "[T7] a plateau holding a full fine step on BOTH passes no longer reads as contact";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 30);
    j.plateau_breakaway = 18;
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(st.scout_tick - rig.stopTick()) <= 1);  // the coarse press breaks through
    CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
    CHECK(st.plateau_bypass_count >= 2);
  }
}

void test_fine_passes_use_the_adaptive_corridor() {
  for (const UpperCase& u : kUppers) {
    g_case = "[T7] scout ON the entry (V25 LF HIP MAX); fine contacts 5 short: adaptive corridor";
    ProbeRig rig(u, ContactSide::MIN_SIDE);
    rig.placeStop(rig.depth(rig.corridor.entry_tick) - rig.depth(rig.corridor.contact_tick));
    bool moved = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!moved && r.probe.status().scout_valid &&
          r.probe.status().phase == ContactProbePhase::BACKOFF_MONITORING) {
        moved = true;
        shiftStop(r, -5);
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.depth(st.scout_tick) - rig.depth(rig.corridor.entry_tick)) <= 1);
    CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
    // Outside the static corridor, inside the scout-extended one.
    CHECK(!actuator::searchCorridorAccepts(rig.corridor, st.pass1_contact_tick));
    CHECK(rig.depth(st.pass1_contact_tick) >=
          searchAdaptiveAcceptanceEntryDepth(rig.corridor, st.scout_tick));
  }
}

void test_fine_pass_2_is_independent_and_scout_referenced() {
  {
    g_case = "[T9] fine pass 2 is judged against the SCOUT, not against fine pass 1";
    ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool armed1 = false, armed2 = false;
    uint16_t bypass_at_pass2 = 0;
    int shallow1 = 0, shallow2 = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      const ContactProbeStatus& st = r.probe.status();
      simk::SimJoint& j = r.joint();
      if (!armed1 && st.pass == 1) {  // fine 1 accepts a plateau 6 short of the scout
        armed1 = true;
        shallow1 = r.tickAt(r.depth(st.scout_tick) - 6);
        j.plateau = true;
        j.plateau_tick = shallow1;
        j.plateau_breakaway = 60;
      }
      if (!armed2 && st.pass == 2) {  // fine 2 meets one 13 short of the scout, 7 of fine 1
        armed2 = true;
        bypass_at_pass2 = st.plateau_bypass_count;
        shallow2 = r.tickAt(r.depth(st.scout_tick) - 13);
        j.plateau_tick = shallow2;
        j.plateau_breakaway = 18;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(st.pass1_contact_tick - shallow1) <= 1);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);  // bypassed: 13 > 8
    CHECK(st.plateau_bypass_count > bypass_at_pass2);
    CHECK(std::abs(st.pass2_contact_tick - shallow2) > 8);
    checkStepDiscipline(rig);
  }
  {
    g_case = "[T9] fine pass 2 finds the stop 30 ticks deeper: REPEATABILITY_FAILED";
    ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    bool moved = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!moved && r.probe.status().pass == 2) {
        moved = true;
        shiftStop(r, 30);
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::REPEATABILITY_FAILED);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);  // it really measured again
    CHECK(!rig.probe.witness().evaluated);
  }
  {
    g_case = "[T9] fine pass 1 alone is never enough: stop gone for fine pass 2 -> fails closed";
    ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    bool removed = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!removed && r.probe.status().pass == 2) {
        removed = true;
        r.joint().has_stop_low = r.joint().has_stop_high = false;
      }
    });
    CHECK(rig.probe.status().pass1_contact_tick != 0);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
    CHECK(!rig.probe.witness().evaluated);
    checkWriteBounds(rig);
  }
}

void test_kinematic_plateau_needs_the_scout() {
  for (const UpperCase& u : kUppers) {
    g_case = "[T7] V25 confirm_kinematic_plateau: a dithering stop is a fine contact";
    ProbeRig rig(u, ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool armed = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!armed && r.probe.status().pass == 1) {
        armed = true;
        r.joint().stop_dither_ticks = 3;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(st.kinematic_plateau_count >= 2);  // one per fine pass
    CHECK(std::abs(st.scout_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 3);
    CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 3);
    checkStepDiscipline(rig);
  }
  for (const UpperCase& u : kUppers) {
    // A dithering stop 5 ticks inside the corridor: two more 64-tick scout
    // steps still fit before the guard, so a settle window ends with the
    // target > 68 ticks ahead - where a fine pass would confirm a kinematic
    // plateau and the scout must not.
    g_case = "[T7,T11] the coarse scout has NO kinematic-plateau path (V25 scout = None)";
    ProbeRig rig(u, ContactSide::MIN_SIDE);
    rig.placeStop(rig.depth(rig.corridor.entry_tick) + 5 - rig.depth(rig.corridor.contact_tick));
    rig.joint().stop_dither_ticks = 3;
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::TRACKING_FAILED);
    CHECK(rig.depth(st.target_tick) - rig.depth(st.last_position) > 68);
    CHECK(!st.scout_valid);
    CHECK_EQ(st.kinematic_plateau_count, 0);
    checkWriteBounds(rig);
  }
  {
    g_case = "[T7] a kinematic plateau far from the scout is a tracking failure, never contact";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool armed = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!armed && r.probe.status().pass == 1) {  // a dithering obstruction 40 short of the scout
        armed = true;
        shiftStop(r, -40);
        r.joint().stop_dither_ticks = 3;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::TRACKING_FAILED);
    CHECK(st.kinematic_plateau_count >= 1);
    CHECK_EQ(st.pass1_contact_tick, 0);
  }
  {
    // V25: |present - scout| <= ADAPTIVE_FINE_SCOUT_TICKS for every sample.
    g_case = "[T7] a kinematic plateau 40 ticks BEYOND the scout is not a contact";
    ProbeRig rig(kUppers[3], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;  // a transient obstruction makes the scout 40 short...
    j.plateau_tick = rig.tickAt(rig.depth(rig.stopTick()) - 40);
    j.plateau_breakaway = 1000;
    bool cleared = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!cleared && r.probe.status().scout_valid &&
          r.probe.status().phase == ContactProbePhase::BACKOFF_MONITORING) {
        cleared = true;  // ...then the fine passes meet the real, dithering stop
        r.joint().plateau = false;
        r.joint().stop_dither_ticks = 3;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK(std::abs(rig.depth(st.scout_tick) - (rig.depth(rig.stopTick()) - 40)) <= 1);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::TRACKING_FAILED);
    CHECK(st.kinematic_plateau_count >= 1);
    CHECK_EQ(st.pass1_contact_tick, 0);
  }
  {
    // V25: the 3 plateau samples span <= KINEMATIC_PLATEAU_POSITION_SPAN_TICKS.
    g_case = "[T7] a plateau spanning 5 ticks (> 3) is not a kinematic contact";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    bool armed = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (!armed && r.probe.status().pass == 1) {
        armed = true;
        r.joint().stop_dither_ticks = 5;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::TRACKING_FAILED);
    CHECK(st.kinematic_plateau_count >= 1);
    CHECK_EQ(st.pass1_contact_tick, 0);
  }
}

// --- [T11] no scout, no endpoint ---------------------------------------------------

void test_missing_scout_makes_the_endpoint_unacceptable() {
  struct Case {
    const char* name;
    std::function<void(ProbeRig&)> setup;
  };
  const Case cases[] = {
      {"no stop at all", [](ProbeRig&) {}},
      {"stop before the corridor (early stall)",
       [](ProbeRig& r) {
         r.joint().press_gain = 2;
         r.placeStop(r.depth(r.corridor.entry_tick) - 100 - r.depth(r.corridor.contact_tick));
       }},
      {"dithering stop the scout cannot confirm",
       [](ProbeRig& r) {
         r.placeStop(reachableBeyond(r.side));
         r.joint().stop_dither_ticks = 3;
       }},
  };
  for (const Case& c : cases) {
    for (ContactSide side : kSides) {
      g_case = "[T11] the scout failed: no fine pass, no witness, SAFE_OFF";
      ProbeRig rig(kUppers[3], side);
      c.setup(rig);
      runProbe(rig);
      const ContactProbeStatus& st = rig.probe.status();
      CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
      CHECK(!st.scout_valid);
      CHECK_EQ(st.pass1_contact_tick, 0);
      CHECK_EQ(st.pass2_contact_tick, 0);
      CHECK(!rig.probe.witness().evaluated);
      for (const WriteLog& w : rig.log) {
        CHECK_EQ(w.pass, 0);
        CHECK(w.stage != S::FINE_SEARCH && w.stage != S::RELEASE && w.stage != S::BACKOFF);
      }
    }
  }
}

// --- [T12] no stop: the guard ends it, fail-closed ----------------------------------

void test_no_stop_fails_closed_at_the_guard() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      g_case = "[T12] no stop: NO_CONTACT_BEFORE_GUARD, never a target past the guard";
      ProbeRig rig(u, side);
      runProbe(rig);
      const ContactProbeStatus& st = rig.probe.status();
      const int guard_d = rig.depth(rig.corridor.guard_tick);
      CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
      CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
      CHECK_EQ(st.last_candidate_tick, 0);
      CHECK(!st.scout_valid);
      CHECK_EQ(rig.depth(st.target_tick), guard_d);  // the final partial step, then refused
      CHECK(st.scout_partial_step_ticks > 0 && st.scout_partial_step_ticks < 64);
      int at_guard = 0;
      for (const WriteLog& w : rig.log) at_guard += rig.depth(w.tick) == guard_d;
      CHECK_EQ(at_guard, 1);  // never repeated
      const simk::SimJoint& j = rig.joint();
      const int deepest = rig.corridor.probe_sign < 0 ? (int)j.lo_seen : (int)j.hi_seen;
      CHECK(rig.depth(deepest) <= guard_d);
      checkWriteBounds(rig);
    }
  }
}

// --- the final bounded partial coarse-scout step (current-installation
// deviation, 2026-09-30) --------------------------------------------------------
//
// V25 ended the scout when the next 64-tick step would pass the guard, so a
// stop deeper than (last full target - 11) was never scouted: a grid-phase
// blind gap before the guard (LF UPPER MIN's measured stop fell in it). Now
// ONE final partial step targets the guard itself. Detection still needs the
// target more than 10 ticks past the stop, and the fine passes (8-tick grid,
// unchanged) a fine target 11..18 past it, so every stop down to guard - 18 is
// found whatever the phase; the last 10 ticks before the guard stay silent
// and fail closed.

struct Grid {
  int last_full = -100000;  // deepest full 64-tick coarse target
  int partial = 0;          // the partial step's size, 0 = none
  int partial_target = -100000;
  int guard_writes = 0;
};

Grid scoutGrid(const ProbeRig& rig) {
  Grid g;
  const int guard_d = rig.depth(rig.corridor.guard_tick);
  int prev = rig.start_position;
  for (const WriteLog& w : rig.log) {
    const int d = rig.depth(w.tick);
    if (w.pass == 0 && (w.stage == S::COARSE_TRANSIT || w.stage == S::COARSE_SCOUT)) {
      const int delta = d - rig.depth(prev);
      if (delta == 64 || prev == rig.start_position) {
        g.last_full = std::max(g.last_full, d);
      } else {
        g.partial = delta;
        g.partial_target = d;
      }
    }
    if (d == guard_d && w.stage != S::RELEASE) ++g.guard_writes;
    if (w.stage != S::RELEASE) prev = w.tick;
  }
  return g;
}

void test_final_partial_scout_step() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      // The grid of this rig, learned from a free run (no stop at all).
      ProbeRig free_rig(u, side);
      runProbe(free_rig);
      const int guard_d = free_rig.depth(free_rig.corridor.guard_tick);
      const Grid fg = scoutGrid(free_rig);
      {
        g_case = "[P1-P4,P8,P9] no stop: full 64-tick steps, ONE partial step to the guard, then refused";
        const ContactProbeStatus& st = free_rig.probe.status();
        CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
        CHECK(fg.partial > 0 && fg.partial < 64);
        CHECK_EQ(fg.partial_target, guard_d);
        CHECK_EQ(fg.last_full + fg.partial, guard_d);
        CHECK(fg.last_full + 64 > guard_d);  // V25 would have stopped here
        CHECK_EQ(fg.guard_writes, 1);         // never repeated
        CHECK_EQ(st.scout_partial_step_ticks, fg.partial);
        checkWriteBounds(free_rig);
        // Every coarse step but the first (from the baseline end) and the
        // partial one is exactly 64 ticks.
        int prev = -1, full = 0;
        for (const WriteLog& w : free_rig.log) {
          if (w.pass != 0 || (w.stage != S::COARSE_TRANSIT && w.stage != S::COARSE_SCOUT)) {
            prev = w.tick;
            continue;
          }
          const int delta = free_rig.depth(w.tick) - free_rig.depth(prev);
          if (free_rig.depth(w.tick) != guard_d && free_rig.log[0].tick != prev) {
            CHECK_EQ(delta, 64);
            ++full;
          }
          prev = w.tick;
        }
        CHECK(full > 0);
      }
      // Stops in the V25 blind gap (deeper than last_full - 11), down to the
      // phase-independent reach guard - 18.
      for (int stop_d : {fg.last_full - 11 + 1, (fg.last_full - 11 + guard_d - 18) / 2, guard_d - 18}) {
        g_case = "[P5-P7,P10] a stop in the old grid gap is scouted by the partial step; fine passes unchanged";
        if (stop_d <= fg.last_full - 11) continue;
        ProbeRig rig(u, side);
        rig.placeStop(stop_d - rig.depth(rig.corridor.contact_tick));
        runProbe(rig);
        const ContactProbeStatus& st = rig.probe.status();
        CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
        CHECK(st.scout_valid);
        CHECK(std::abs(rig.depth(st.scout_tick) - stop_d) <= 1);           // scout = reference
        CHECK(std::abs(rig.depth(st.pass1_contact_tick) - stop_d) <= 1);   // fine metrology
        CHECK(std::abs(rig.depth(st.pass2_contact_tick) - stop_d) <= 1);
        CHECK(st.scout_partial_step_ticks > 0 && st.scout_partial_step_ticks < 64);
        CHECK_EQ(rig.probe.witness().max_deviation_ticks,
                 std::abs(st.pass1_contact_tick - st.pass2_contact_tick));
        const Grid g = scoutGrid(rig);
        CHECK_EQ(g.partial_target, guard_d);
        checkStepDiscipline(rig);
        for (const WriteLog& w : rig.log) CHECK(rig.depth(w.tick) <= guard_d);
      }
      {
        g_case = "[P8] a stop in the last 10 ticks before the guard: no contact signature -> fails closed";
        ProbeRig rig(u, side);
        rig.placeStop(guard_d - 5 - rig.depth(rig.corridor.contact_tick));
        runProbe(rig);
        const ContactProbeStatus& st = rig.probe.status();
        CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
        CHECK(!st.scout_valid);
        CHECK_EQ(rig.depth(st.target_tick), guard_d);
        CHECK_EQ(scoutGrid(rig).guard_writes, 1);
        checkWriteBounds(rig);
      }
    }
  }
  // [P7] The fine passes are unchanged: 8-tick steps only, no partial step of
  // their own. A stop 11..17 ticks short of the guard is scouted (coarse reach
  // guard - 11) but a fine pass finds it only if its own 8-tick grid lands a
  // target 11..18 past it before the guard - otherwise it fails closed there.
  // (In this model the fine grid's phase is fixed - backoff 96 = 12 fine steps,
  // 4-tick settle - so that target is always stop + 12 and only 11 ticks short
  // misses; on hardware the offset can be anywhere in 11..18.)
  int fine_misses = 0, fine_hits = 0;
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      for (int short_of_guard = 11; short_of_guard <= 17; ++short_of_guard) {
        g_case = "[P7] fine passes keep the 8-tick grid near the guard (no fine partial step)";
        ProbeRig rig(u, side);
        const int guard_d = rig.depth(rig.corridor.guard_tick);
        const int stop_d = guard_d - short_of_guard;
        rig.placeStop(stop_d - rig.depth(rig.corridor.contact_tick));
        runProbe(rig);
        const ContactProbeStatus& st = rig.probe.status();
        CHECK(st.scout_valid);  // the partial coarse step reaches it
        int prev = -1;
        int last_fine = -100000;
        for (const WriteLog& w : rig.log) {
          if (w.stage == S::FINE_SEARCH && prev >= 0 && rig.log.front().tick != prev) {
            const int delta = rig.depth(w.tick) - rig.depth(prev);
            if (delta != 8) CHECK(delta >= 8 - 12 && delta <= 8 + 12 && rig.depth(prev) < stop_d - 60);
          }
          if (w.stage == S::FINE_SEARCH) last_fine = std::max(last_fine, rig.depth(w.tick));
          if (w.stage != S::RELEASE) prev = w.tick;
          CHECK(rig.depth(w.tick) <= guard_d);
        }
        if (st.phase == ContactProbePhase::COMPLETE) {
          ++fine_hits;
          CHECK(std::abs(rig.depth(st.pass1_contact_tick) - stop_d) <= 1);
        } else {
          ++fine_misses;
          CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
          CHECK(st.pass >= 1);                    // missed by a FINE pass, not the scout
          CHECK(last_fine - stop_d <= 10);        // no fine target ran past it by > 10
          CHECK(last_fine + 8 > guard_d);         // the next 8-tick fine step would pass the guard
        }
      }
    }
  }
  std::printf("    near-guard stops: fine grid found %d, missed (fail closed) %d\n", fine_hits, fine_misses);
  CHECK(fine_misses > 0);  // the band really is fine-grid dependent
  CHECK(fine_hits > 0);

  // [P11] every per-sample violation during the partial step fails closed there.
  struct Fault {
    ContactProbeFailure expect;
    std::function<void(bool*, TelemetrySample*)> apply;
  };
  const Fault faults[] = {
      {ContactProbeFailure::HARD_CURRENT_ABORT, [](bool*, TelemetrySample* s) { s->present_current = 250; }},
      {ContactProbeFailure::GOAL_READBACK_MISMATCH, [](bool*, TelemetrySample* s) { s->goal_position += 30; }},
      {ContactProbeFailure::TORQUE_LIMIT_CHANGED, [](bool*, TelemetrySample* s) { s->torque_limit = 1000; }},
      {ContactProbeFailure::STALE_TELEMETRY, [](bool* a, TelemetrySample*) { *a = false; }},
  };
  for (ContactSide side : kSides) {
    for (const Fault& f : faults) {
      g_case = "[P11] telemetry / current / GoalPosition / TorqueLimit faults during the partial step";
      ProbeRig rig(kUppers[1], side);
      rig.placeStop(rig.depth(rig.corridor.guard_tick) - 18 - rig.depth(rig.corridor.contact_tick));
      bool armed = false;
      runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool* available, TelemetrySample* s) {
        const ContactProbeStatus& st = r.probe.status();
        if (!armed && st.pass == 0 && st.phase == ContactProbePhase::STEP_MONITORING &&
            st.target_tick == r.corridor.guard_tick) {
          armed = true;
        }
        if (armed) f.apply(available, s);
      });
      CHECK(armed);
      CHECK_EQ((int)rig.probe.status().failure, (int)f.expect);
      CHECK_EQ(rig.probe.status().target_tick, rig.corridor.guard_tick);
      CHECK(!rig.probe.status().scout_valid);
    }
  }
}

// --- [H1] the V25 backoff StableTargetGate -------------------------------------

void test_settle_gate_rules() {
  g_case = "[H1] SearchSettleGate = V25 StableTargetGate::observe_at, rule by rule";
  const uint16_t target = 1500;
  {  // one good sample is not settled, nor are four within 400 ms
    SearchSettleGate g;
    CHECK(!g.observe(1500, 0, target, 1000));
    CHECK(!g.observe(1500, 0, target, 1020));
    CHECK(!g.observe(1500, 0, target, 1040));
    CHECK(!g.observe(1500, 0, target, 1060));  // 4 samples, 60 ms
    CHECK_EQ(g.consecutive(), 4);
    CHECK(!g.observe(1500, 0, target, 1399));  // 399 ms
    CHECK(g.observe(1500, 0, target, 1400));   // >= 400 ms and >= 4 samples
  }
  {  // 400 ms of ONE sample is not enough: 4 consecutive samples are required
    SearchSettleGate g;
    CHECK(!g.observe(1500, 0, target, 1000));
    CHECK(!g.observe(1500, 0, target, 1500));
    CHECK(!g.observe(1500, 0, target, 2000));
    CHECK(g.observe(1500, 0, target, 2500));
  }
  {  // band edge and speed edge are inclusive; one beyond resets both counters
    SearchSettleGate g;
    CHECK(!g.observe(1512, 4, target, 1000));
    CHECK(!g.observe(1488, 4, target, 1100));
    CHECK(!g.observe(1513, 0, target, 1200));  // 13 ticks: out of the band -> reset
    CHECK_EQ(g.consecutive(), 0);
    CHECK(!g.observe(1500, 0, target, 1300));
    CHECK(!g.observe(1500, 0, target, 1400));
    CHECK(!g.observe(1500, 0, target, 1500));
    CHECK(!g.observe(1500, 0, target, 1600));  // 4 samples but only 300 ms since the reset
    CHECK(g.observe(1500, 0, target, 1700));
  }
  {  // moving inside the band never qualifies; an unread speed never does
    SearchSettleGate g;
    for (uint32_t t = 1000; t < 3000; t += 20) CHECK(!g.observe(1500, 5, target, t));
    for (uint32_t t = 3000; t < 5000; t += 20) CHECK(!g.observe(1500, -1, target, t));
    CHECK_EQ(g.consecutive(), 0);
  }
  {  // oscillation: every excursion restarts the 400 ms window
    SearchSettleGate g;
    bool settled = false;
    for (uint32_t t = 1000; t < 5000; t += 20) {
      const bool out = ((t - 1000) / 20) % 10 == 9;  // one sample in ten 20 ticks off
      settled = settled || g.observe(out ? 1520 : 1500, 0, target, t);
    }
    CHECK(!settled);
  }
  CHECK_EQ(kSearchBackoffSettleToleranceTicks, kSearchStaticToleranceTicks + 2);
}

// Per tick: the phase BEFORE the update, and whether the sample the engine is
// about to see qualifies for the V25 gate.
struct BackoffTick {
  uint32_t t;
  ContactProbePhase phase;
  uint8_t pass;
  bool qualifies;
};

// No fine pass ever starts before the V25 gate held for >= 4 consecutive
// qualifying samples over >= 400 ms: checked independently of the engine, on
// the samples it was given.
void checkBackoffsSettled(const std::vector<BackoffTick>& ticks, int expected_backoffs) {
  int backoffs = 0;
  uint32_t run_start = 0;
  int run = 0;
  for (size_t i = 0; i + 1 < ticks.size(); ++i) {
    const BackoffTick& k = ticks[i];
    if (k.phase != ContactProbePhase::BACKOFF_MONITORING) {
      run = 0;
      continue;
    }
    if (k.qualifies) {
      if (run == 0) run_start = k.t;
      ++run;
    } else {
      run = 0;
    }
    const BackoffTick& next = ticks[i + 1];
    if (next.phase == ContactProbePhase::STEP_PENDING && next.pass == k.pass + 1) {
      ++backoffs;  // this tick's update settled the backoff and began the next fine pass
      CHECK(run >= kSearchBackoffSettledSamples);
      CHECK(k.t - run_start >= kSearchBackoffSettleWindowMs);
    }
  }
  CHECK_EQ(backoffs, expected_backoffs);
}

void test_backoff_waits_for_the_v25_settle_gate() {
  for (uint32_t dt : {2u, 10u}) {
    for (const UpperCase& u : kUppers) {
      for (ContactSide side : kSides) {
        g_case = "[H1] no fine pass before the backoff settled (4 samples, >= 400 ms, |v| <= 4)";
        ProbeRig rig(u, side);
        rig.placeStop(reachableBeyond(rig.side));
        std::vector<BackoffTick> ticks;
        runProbe(rig, dt, 180000, [&](ProbeRig& r, uint32_t t, bool* available, TelemetrySample* s) {
          const ContactProbeStatus& st = r.probe.status();
          const bool q = *available && s->read_ok &&
                         std::abs(s->present_position - (int)st.target_tick) <= 12 &&
                         s->present_speed >= 0 && s->present_speed <= 4;
          ticks.push_back(BackoffTick{t, st.phase, st.pass, q});
        });
        CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
        checkBackoffsSettled(ticks, 2);  // after the scout and after fine pass 1
      }
    }
  }
  {
    g_case = "[H1] in the band but still moving (|v| > 4): never settled -> MOTION_TIMEOUT";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    uint32_t backoff_at = 0, failed_at = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample* s) {
      const ContactProbeStatus& st = r.probe.status();
      if (st.phase == ContactProbePhase::BACKOFF_MONITORING) {
        if (backoff_at == 0) backoff_at = t;
        s->present_speed = 5;  // one raw above LF_HELD_MAX_SPEED_RAW, whatever the position
      }
      failed_at = t;
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::MOTION_TIMEOUT);
    CHECK(st.stage == S::BACKOFF);
    CHECK_EQ(st.pass, 0);
    CHECK_EQ(st.pass1_contact_tick, 0);
    CHECK(failed_at - backoff_at >= 12000);  // the deadman's budget, fail closed
    for (const WriteLog& w : rig.log) CHECK(w.stage != S::FINE_SEARCH);
  }
  {
    g_case = "[H1] oscillation out of the band restarts the gate: never settled -> MOTION_TIMEOUT";
    ProbeRig rig(kUppers[2], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    uint32_t n = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample* s) {
      const ContactProbeStatus& st = r.probe.status();
      if (st.phase == ContactProbePhase::BACKOFF_MONITORING &&
          std::abs(s->present_position - (int)st.target_tick) <= 12 && ++n % 20 == 0) {
        s->present_position += 25;  // out of the band for one sample every 200 ms
      }
    });
    // Fail closed either way: the deadman's own stall watch (the same
    // excursion value for 2 s) or its motion deadline - never settled.
    const ContactProbeFailure f = rig.probe.status().failure;
    CHECK(f == ContactProbeFailure::MOTION_TIMEOUT ||
          f == ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);
    CHECK_EQ(rig.probe.status().pass1_contact_tick, 0);
    for (const WriteLog& w : rig.log) CHECK(w.stage != S::FINE_SEARCH);
  }
  {
    g_case = "[H1] oscillation that dies out: settled >= 400 ms after the last excursion";
    ProbeRig rig(kUppers[3], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    uint32_t in_band_since = 0, last_excursion = 0, next_pass_at = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample* s) {
      const ContactProbeStatus& st = r.probe.status();
      if (st.pass == 0 && st.phase == ContactProbePhase::BACKOFF_MONITORING &&
          std::abs(s->present_position - (int)st.target_tick) <= 12) {
        if (in_band_since == 0) in_band_since = t;
        if (t - in_band_since < 600 && (t - in_band_since) % 150 == 0) {
          s->present_position += 25;
          last_excursion = t;
        }
      }
      if (next_pass_at == 0 && st.pass == 1) next_pass_at = t;
    });
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(last_excursion != 0);
    CHECK(next_pass_at - last_excursion >= kSearchBackoffSettleWindowMs);
  }
  for (uint32_t high_ms : {300u, 700u}) {
    // Decelerating current (synthetic): high for high_ms after the joint enters
    // the band. V25 checks recovery on the SETTLED observation (>= 400 ms in),
    // so 300 ms of it passes and 700 ms of it fails - a check made on the first
    // in-band sample would fail both.
    g_case = "[H1] current recovery is judged only once the backoff has settled";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    uint32_t in_band_since = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample* s) {
      const ContactProbeStatus& st = r.probe.status();
      if (st.pass == 0 && st.phase == ContactProbePhase::BACKOFF_MONITORING &&
          std::abs(s->present_position - (int)st.target_tick) <= 12) {
        if (in_band_since == 0) in_band_since = t;
        if (t - in_band_since < high_ms) s->present_current = 150;
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    if (high_ms < kSearchBackoffSettleWindowMs) {
      CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    } else {
      CHECK_EQ((int)st.failure, (int)ContactProbeFailure::CURRENT_NOT_RECOVERED);
      CHECK_EQ(st.pass1_contact_tick, 0);
    }
  }
  {
    g_case = "[H1] TorqueEnable lost while settling in the band: TORQUE_UNEXPECTEDLY_OFF";
    ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample* s) {
      const ContactProbeStatus& st = r.probe.status();
      if (st.phase == ContactProbePhase::BACKOFF_MONITORING &&
          std::abs(s->present_position - (int)st.target_tick) <= 12) {
        s->torque_enable = 0;
      }
    });
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF);
    CHECK(rig.probe.status().stage == S::BACKOFF);
  }
}

// --- 4. the servo's own settling shortfall is never contact ------------------

void test_settling_shortfall_is_never_contact() {
  for (int deadband : {4, 5}) {
    g_case = "4-5 tick settling shortfall at every step: no candidate without a stop";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.joint().deadband_ticks = deadband;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
    CHECK_EQ(rig.probe.status().last_candidate_tick, 0);

    g_case = "4-5 tick settling shortfall with the real stop: contact at the stop";
    ProbeRig rig2(kUppers[0], ContactSide::MIN_SIDE);
    rig2.joint().deadband_ticks = deadband;
    rig2.placeStop(reachableBeyond(rig2.side));
    runProbe(rig2);
    CHECK_EQ((int)rig2.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig2.probe.status().scout_tick - rig2.stopTick()) <= 1);
    CHECK(std::abs(rig2.probe.status().pass1_contact_tick - rig2.stopTick()) <= 1);
  }
}

// --- 5. brief stops and yielding friction are never contact -------------------

void test_brief_stop_and_yielding_friction_are_not_contact() {
  {
    g_case = "a 100 ms stop inside the corridor is not contact";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 30);
    j.plateau_breakaway = 1000;  // holds regardless of error...
    j.plateau_hold_ms = 100;     // ...but only for 100 ms
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().scout_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    g_case = "friction that yields to <= 10 ticks of error is not contact";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 30);
    j.plateau_breakaway = 10;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
    CHECK_EQ(rig.probe.status().plateau_bypass_count, 0);
  }
}

// --- [T13] early friction before the valid region never becomes contact ------------

void test_early_friction_never_becomes_contact() {
  for (int short_of_entry : {100, 10}) {
    g_case = "[T13] stop before the corridor entry: EARLY_STALL_OUTSIDE_CORRIDOR, never a scout";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.joint().press_gain = 2;
    const int entry_d = rig.depth(rig.corridor.entry_tick);
    rig.placeStop(entry_d - short_of_entry - rig.depth(rig.corridor.contact_tick));
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR);
    CHECK(!st.scout_valid);
    CHECK_EQ(st.pass1_contact_tick, 0);
  }
  for (ContactSide side : kSides) {
    g_case = "[T13] holding friction during coarse transit: EARLY_STALL, never a scout";
    ProbeRig rig(kUppers[3], side);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.entry_tick) - 100);
    j.plateau_breakaway = 1000;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR);
    CHECK(!rig.probe.status().scout_valid);
  }
  for (ContactSide side : kSides) {
    g_case = "[T13] friction yielding inside the V36 16-tick outside band: scout at the real stop";
    ProbeRig rig(kUppers[0], side);
    rig.placeStop(reachableBeyond(rig.side));
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.entry_tick) - 100);
    j.plateau_breakaway = kSearchOutsideCorridorSettleToleranceTicks;
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(st.scout_tick - rig.stopTick()) <= 1);
    CHECK(std::abs(st.scout_tick - (int)j.plateau_tick) > 32);
  }
}

// --- the V25 moving-current baseline ------------------------------------------------

void test_baseline_is_the_v25_forward_move() {
  {
    g_case = "baseline: ONE 64-tick move from the present pose, moving current recorded";
    ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
    CHECK(rig.log.size() >= 2);
    CHECK(rig.log[0].stage == S::BASELINE);
    CHECK_EQ(rig.depth(rig.log[0].tick) - rig.depth(rig.start_position), 64);
    CHECK(st.baseline_samples >= kSearchBaselineMinSamples);
    CHECK_EQ(st.baseline_median_current, rig.joint().current_moving);  // synthetic model value
    CHECK_EQ(st.baseline_mad_current, 0);
  }
  {
    g_case = "baseline that would pass the guard: BASELINE_PASSES_GUARD, no goal written";
    ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
    rig.joint().pos = rig.tickAt(rig.depth(rig.corridor.guard_tick) - 30);
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::BASELINE_PASSES_GUARD);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK(rig.log.empty());
  }
  {
    g_case = "blocked baseline: fewer than 6 moving samples in 12 s -> INSUFFICIENT_BASELINE";
    ProbeRig rig(kUppers[2], ContactSide::MAX_SIDE);
    rig.placeStop(12 - rig.depth(rig.corridor.contact_tick));
    const uint32_t elapsed = runProbe(rig);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::INSUFFICIENT_BASELINE);
    CHECK(rig.probe.status().baseline_samples < kSearchBaselineMinSamples);
    CHECK(elapsed >= kSearchBaselineTimeoutMs && elapsed <= kSearchBaselineTimeoutMs + 100);
    CHECK_EQ(rig.log.size(), 1u);
  }
  {
    // V25 acquire_moving_current_baseline_forward() returns the stats when
    // its deadline expires with >= 6 moving samples, arrived or not; the
    // coarse scout then steps from where the joint IS.
    g_case = "baseline timeout with enough samples proceeds (V25); the scout then fails closed";
    ProbeRig rig(kUppers[3], ContactSide::MIN_SIDE);
    simk::SimJoint& j = rig.joint();
    const double a = rig.tickAt(40), b = rig.tickAt(200);
    j.slow_zone = true;
    j.slow_lo = a < b ? a : b;
    j.slow_hi = a < b ? b : a;
    j.slow_speed_tps = 0.5;  // still "moving" (speed register 1), never arrives in 12 s
    uint32_t started = 0, scout_at = 0;
    int pos_at_scout = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample*) {
      if (started == 0) started = t;
      if (scout_at == 0 && r.probe.status().phase == ContactProbePhase::STEP_PENDING) {
        scout_at = t;
        pos_at_scout = r.joint().position();
      }
    });
    const ContactProbeStatus& st = rig.probe.status();
    CHECK(scout_at - started >= kSearchBaselineTimeoutMs);        // ended by the deadline...
    CHECK(std::abs(rig.depth(pos_at_scout) - 64) > 10);            // ...not by arriving
    CHECK(rig.log.size() >= 2);
    CHECK(rig.log[1].stage == S::COARSE_TRANSIT);
    CHECK(std::abs(rig.depth(rig.log[1].tick) - (rig.depth(pos_at_scout) + 64)) <= 1);  // from the pose
    CHECK(st.baseline_samples >= kSearchBaselineMinSamples);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::TRACKING_FAILED);
    CHECK(!st.scout_valid);
  }
}

// --- [T14] every stage fails closed on every per-observation violation ---------------

void expectFailure(ProbeRig& rig, ContactProbeFailure f) {
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)f);
}

void test_every_stage_fails_closed() {
  // `in` arms the fault the first tick the probe is in that stage; the
  // verdict must come in that SAME stage and pass - a later stage catching
  // it (e.g. the backoff noticing what an unverified release let through)
  // does not count.
  struct Where {
    const char* name;
    std::function<bool(const ContactProbeStatus&)> in;
  };
  const Where stages[] = {
      {"BASELINE", [](const ContactProbeStatus& s) { return s.phase == ContactProbePhase::BASELINE_MONITORING; }},
      {"COARSE_TRANSIT", [](const ContactProbeStatus& s) {
         return s.stage == S::COARSE_TRANSIT && s.phase == ContactProbePhase::STEP_MONITORING; }},
      {"COARSE_SCOUT", [](const ContactProbeStatus& s) {
         return s.stage == S::COARSE_SCOUT && s.phase == ContactProbePhase::STEP_MONITORING; }},
      {"RELEASE (scout)", [](const ContactProbeStatus& s) { return s.pass == 0 && s.stage == S::RELEASE; }},
      {"BACKOFF (scout)", [](const ContactProbeStatus& s) {
         return s.pass == 0 && s.phase == ContactProbePhase::BACKOFF_MONITORING; }},
      {"FINE 1", [](const ContactProbeStatus& s) {
         return s.pass == 1 && s.phase == ContactProbePhase::STEP_MONITORING; }},
      {"RELEASE (fine 1)", [](const ContactProbeStatus& s) { return s.pass == 1 && s.stage == S::RELEASE; }},
      {"BACKOFF (fine 1)", [](const ContactProbeStatus& s) {
         return s.pass == 1 && s.phase == ContactProbePhase::BACKOFF_MONITORING; }},
      {"FINE 2", [](const ContactProbeStatus& s) {
         return s.pass == 2 && s.phase == ContactProbePhase::STEP_MONITORING; }},
      {"RELEASE (fine 2)", [](const ContactProbeStatus& s) { return s.pass == 2 && s.stage == S::RELEASE; }},
  };
  struct Fault {
    ContactProbeFailure expect;
    std::function<void(bool*, TelemetrySample*)> apply;
  };
  const Fault faults[] = {
      {ContactProbeFailure::HARD_CURRENT_ABORT, [](bool*, TelemetrySample* s) { s->present_current = 250; }},
      {ContactProbeFailure::GOAL_READBACK_MISMATCH, [](bool*, TelemetrySample* s) { s->goal_position += 30; }},
      {ContactProbeFailure::TORQUE_LIMIT_CHANGED, [](bool*, TelemetrySample* s) { s->torque_limit = 1000; }},
      {ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF, [](bool*, TelemetrySample* s) { s->torque_enable = 0; }},
      {ContactProbeFailure::SERVO_STATUS_FAULT, [](bool*, TelemetrySample* s) { s->servo_status = 0x20; }},
      {ContactProbeFailure::OVER_TEMPERATURE, [](bool*, TelemetrySample* s) { s->present_temperature = 71; }},
      {ContactProbeFailure::STALE_TELEMETRY, [](bool* a, TelemetrySample*) { *a = false; }},
  };
  for (const Where& w : stages) {
    for (const Fault& f : faults) {
      g_case = "[T14] a violation in any stage fails closed with its own verdict";
      ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
      rig.placeStop(reachableBeyond(rig.side));
      bool armed = false;
      uint32_t armed_at = 0, failed_at = 0;
      ContactSearchStage armed_stage = S::NONE;
      uint8_t armed_pass = 0;
      runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool* available, TelemetrySample* s) {
        if (!armed && w.in(r.probe.status())) {
          armed = true;
          armed_at = t;
          armed_stage = r.probe.status().stage;
          armed_pass = r.probe.status().pass;
        }
        if (armed) {
          f.apply(available, s);
          failed_at = t;
        }
      });
      if (!armed) std::printf("    stage %s never reached\n", w.name);
      CHECK(armed);
      expectFailure(rig, f.expect);
      CHECK(rig.probe.status().stage == armed_stage);  // caught where it happened
      CHECK_EQ(rig.probe.status().pass, armed_pass);
      if (f.expect == ContactProbeFailure::STALE_TELEMETRY) {
        // V25 TELEMETRY_TIMEOUT (matdog.rs L73): 2 s without a new usable
        // sample, everywhere - the oracle's literal value, not the constant.
        CHECK(failed_at - armed_at >= 2000 - 100 && failed_at - armed_at <= 2000 + 100);
      } else {
        CHECK(failed_at - armed_at <= 30);  // on the first samples, not later
      }
      CHECK(!rig.probe.witness().evaluated);
    }
  }
}

// --- 8. the remaining aborts ------------------------------------------------------------

void test_safety_aborts() {
  {
    g_case = "pressing current >= 200 raw (a jam past the torque-limit cap): HARD_CURRENT_ABORT";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    rig.joint().press_gain = 25;
    rig.joint().press_current_max = -1;
    runProbe(rig);
    expectFailure(rig, ContactProbeFailure::HARD_CURRENT_ABORT);
  }
  {
    g_case = "failed reads for 3 s: COMMUNICATION_LOST";
    ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample* s) {
      if (r.probe.status().step_count >= 2) s->read_ok = false;
    });
    expectFailure(rig, ContactProbeFailure::COMMUNICATION_LOST);
  }
  {
    g_case = "current stays high after the FINE-1 backoff: CURRENT_NOT_RECOVERED";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().pass1_contact_tick != 0) r.joint().current_override = 150;
    });
    expectFailure(rig, ContactProbeFailure::CURRENT_NOT_RECOVERED);
    CHECK(rig.probe.status().baseline_samples >= kSearchBaselineMinSamples);
    CHECK_EQ(rig.probe.status().pass2_contact_tick, 0);
  }
  {
    g_case = "permit withdrawn mid-transit: next step refused by the policy";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().step_count == 2) r.setPermit(false);
    });
    expectFailure(rig, ContactProbeFailure::COMMAND_REJECTED);
    CHECK_EQ((int)rig.probe.status().last_policy_decision,
             (int)actuator::WriteDecision::REJECT_NO_CALIBRATION_MOTION_PERMIT);
  }
  {
    g_case = "a sample missing the readback registers is not usable";
    ProbeRig rig(kUppers[3], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    uint32_t cut = 0, failed_after = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample* s) {
      if (r.probe.status().step_count >= 2) {
        if (cut == 0) cut = t;
        s->torque_limit = -1;  // register not read
        failed_after = t - cut;
      }
    });
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK(failed_after >= 2000 - 10);  // no sample, never a good one (V25 2 s)
    CHECK(failed_after <= 2000 + 20);
  }
  {
    g_case = "expected TorqueLimit 0 is a refusal";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    ContactProbeRequest r = rig.request();
    r.expected_torque_limit = 0;
    CHECK(!rig.probe.start(r, rig.ctx(), 1000));
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::REJECT_PRECONDITIONS);
  }
}

void test_start_torque_verified_and_release() {
  g_case = "sequence start: no TorqueEnable of its own; ends released ON the stop";
  ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
  rig.placeStop(reachableBeyond(rig.side));
  rig.joint().torque = true;  // the sequence energized it (prime, limit, torque)
  rig.joint().goal = rig.joint().position();
  ContactProbeRequest r = rig.request();
  r.start_torque_verified = true;
  CHECK(rig.probe.start(r, rig.ctx(), 1000));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::BASELINE_PENDING);
  uint32_t t = 1000;
  while (rig.probe.active() && t < 200000) {
    t += 10;
    rig.backend.advance(t, 10);
    rig.probe.update(rig.ctx(), t, true, rig.joint().sample(t));
  }
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
  CHECK_EQ(rig.backend.torque_writes[rig.u.bus], 0);
  // V25 stop_pressure(): the last write parks GoalPosition on the fine-2
  // contact, so the joint rests on the stop without pressing into it.
  CHECK(!rig.backend.writes.empty());
  CHECK_EQ(rig.backend.writes.back().tick, rig.probe.status().pass2_contact_tick);
  CHECK(rig.joint().torque);
  CHECK(std::abs(rig.joint().goal - rig.joint().position()) <= 4);
}

// --- 9. cadence: a brief stop can never confirm at a fast tick rate -----------

void test_cadence_keeps_v25_timing_at_any_tick_rate() {
  for (uint32_t dt : {2u, 5u, 10u}) {
    // The detector counts 20-ms samples, never Controller ticks: a stall is
    // confirmed only after 3 consecutive 20-ms samples (2 intervals after the
    // first) - and, when the joint meets the stop right after a new target,
    // only after that target's 4 start-up samples as well (the fine case).
    g_case = "contact confirmation counts 20-ms samples whatever the tick rate";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    uint32_t pressing_since[3] = {0, 0, 0};
    uint32_t confirmed_after[3] = {0, 0, 0};
    runProbe(rig, dt, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample*) {
      const ContactProbeStatus& st = r.probe.status();
      const uint8_t p = st.pass;
      if (p > 1) return;
      const bool done = p == 0 ? st.scout_valid : st.pass1_contact_tick != 0;
      if (done || st.phase != ContactProbePhase::STEP_MONITORING) {
        if (confirmed_after[p] == 0 && done && pressing_since[p] != 0) {
          confirmed_after[p] = t - pressing_since[p];
        }
        return;
      }
      const bool at_stop = r.joint().position() == r.stopTick();
      const bool ahead = r.depth(st.target_tick) - r.depth(r.stopTick()) > 10;
      if (at_stop && ahead && pressing_since[p] == 0) pressing_since[p] = t;
      if (!(at_stop && ahead)) pressing_since[p] = 0;
    });
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(confirmed_after[0] >= (kSearchPersistenceSamples - 1) * kSearchSampleIntervalMs);
    CHECK(confirmed_after[1] >= 6 * kSearchSampleIntervalMs);
  }
}

// --- 10. abort, refusals ------------------------------------------------------

void test_abort_and_refusals() {
  for (S stage : {S::BASELINE, S::COARSE_SCOUT, S::RELEASE, S::FINE_SEARCH}) {
    g_case = "abort mid-search requires SAFE_OFF, in every stage";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().stage == stage) r.probe.abort();
    });
    expectFailure(rig, ContactProbeFailure::OPERATOR_ABORT);
  }
  {
    g_case = "abort before torque is verified needs no SAFE_OFF";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    CHECK(rig.probe.start(rig.request(), rig.ctx(), 1000));
    rig.probe.abort();
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::FAILED_NO_MOTION);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::OPERATOR_ABORT);
  }
  {
    g_case = "start refuses an unusable corridor or tolerance";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    ContactProbeRequest r = rig.request();
    r.repeatability_tolerance_ticks = 0;
    CHECK(!rig.probe.start(r, rig.ctx(), 1000));
    r = rig.request();
    r.corridor.probe_sign = 0;
    CHECK(!rig.probe.start(r, rig.ctx(), 1000));
    r = rig.request();
    std::swap(r.corridor.entry_tick, r.corridor.guard_tick);  // guard before entry
    CHECK(!rig.probe.start(r, rig.ctx(), 1000));
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::REJECT_PRECONDITIONS);
    CHECK_EQ(rig.backend.torque_writes[rig.u.bus], 0);
  }
  {
    g_case = "second start while active is refused";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    CHECK(rig.probe.start(rig.request(), rig.ctx(), 1000));
    CHECK(!rig.probe.start(rig.request(), rig.ctx(), 1000));
  }
  {
    g_case = "witness is empty before COMPLETE";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    CHECK(!rig.probe.witness().evaluated);
  }
}

void test_to_string_total() {
  g_case = "toString totality";
  for (int p = 0; p <= (int)ContactProbePhase::RELEASE_VERIFYING; ++p) {
    CHECK(std::strcmp(toString(static_cast<ContactProbePhase>(p)), "UNKNOWN") != 0);
  }
  for (int f = 0; f <= (int)ContactProbeFailure::SCOUT_MISSING; ++f) {
    CHECK(std::strcmp(toString(static_cast<ContactProbeFailure>(f)), "UNKNOWN") != 0);
  }
  for (int s = 0; s <= (int)ContactSearchStage::RELEASE; ++s) {
    CHECK(std::strcmp(toString(static_cast<ContactSearchStage>(s)), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<ContactProbeFailure>(99)), "UNKNOWN") == 0);
}

// --- 11. adversarial model cases (review of the synthetic model itself) --------

void test_adversarial_model_cases() {
  {
    g_case = "settling 9 ticks short (just inside V25's 10-tick band): still no false contact";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.joint().deadband_ticks = 9;
    rig.placeStop(reachableBeyond(rig.side));
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    // The detector tolerates it (3 clean 20-ms samples still occur) and the
    // scout finds the real stop; V25's backoff StableTargetGate does not: a
    // resting joint reading 25 raw every 80 ms never holds |speed| <= 4 for
    // 400 ms, so the backoff never settles and the attempt fails closed.
    g_case = "speed-register noise in 1 of 4 20-ms slots: scout at the stop, backoff never settles";
    ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    rig.joint().speed_noise_every = 4;
    rig.joint().speed_noise_raw = 25;
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK(st.scout_valid);
    CHECK(std::abs(st.scout_tick - rig.stopTick()) <= 1);
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::MOTION_TIMEOUT);
    CHECK(st.stage == S::BACKOFF);
    CHECK_EQ(st.pass1_contact_tick, 0);
  }
  {
    g_case = "speed noise in every other 20-ms slot: never 3 clean samples -> fails closed";
    ProbeRig rig(kUppers[2], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    rig.joint().speed_noise_every = 2;
    rig.joint().speed_noise_raw = 25;
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK(st.failure == ContactProbeFailure::TRACKING_FAILED ||
          st.failure == ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
    CHECK(!st.scout_valid);
    checkWriteBounds(rig);
  }
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      for (double tps : {30.0, 90.0}) {
        // V25's coarse scout lets a 900-ms settle window end at most 68 ticks
        // behind its 64-tick target, so a joint crawling below V25's 80
        // ticks/s motion floor may run out of steps before the guard: that is
        // a fail-closed end, never a contact. Above the floor it completes.
        g_case = "temporary slowdown inside the corridor is never contact";
        ProbeRig rig(u, side);
        rig.placeStop(reachableBeyond(rig.side));
        simk::SimJoint& j = rig.joint();
        const int a = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 40);
        const int b = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 10);
        j.slow_zone = true;
        j.slow_lo = a < b ? a : b;
        j.slow_hi = a < b ? b : a;
        j.slow_speed_tps = tps;
        runProbe(rig);
        const ContactProbeStatus& st = rig.probe.status();
        if (tps > kSearchMinExpectedTicksPerSecond) {
          CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
        }
        if (st.phase == ContactProbePhase::COMPLETE) {
          CHECK(std::abs(st.scout_tick - rig.stopTick()) <= 1);
          CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
          CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
        } else {
          CHECK(st.failure == ContactProbeFailure::TRACKING_FAILED ||
                st.failure == ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
          CHECK(!st.scout_valid);
          CHECK_EQ(st.last_candidate_tick, 0);
        }
        checkWriteBounds(rig);
      }
    }
  }
  {
    g_case = "an obstruction during the FINE-1 backoff is an anomaly, never contact";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(reachableBeyond(rig.side));
    rig.joint().press_gain = 2;
    bool blocked = false;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      const ContactProbeStatus& st = r.probe.status();
      if (!blocked && st.pass1_contact_tick != 0) {
        blocked = true;  // a stop 40 ticks back toward q0, on the backoff path
        simk::SimJoint& j = r.joint();
        const double back = r.tickAt(r.depth(st.pass1_contact_tick) - 40);
        if (r.corridor.probe_sign < 0) { j.has_stop_high = true; j.stop_high = back; }
        else { j.has_stop_low = true; j.stop_low = back; }
      }
    });
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);
    CHECK_EQ(rig.probe.status().pass2_contact_tick, 0);
  }
  {
    g_case = "a stop exactly at the corridor entry is accepted (inclusive bound, V25 LF HIP MAX)";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.placeStop(rig.depth(rig.corridor.entry_tick) - rig.depth(rig.corridor.contact_tick));
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.depth(rig.probe.status().scout_tick) - rig.depth(rig.corridor.entry_tick)) <= 1);
    checkStepDiscipline(rig);
  }
}

}  // namespace

int main() {
  test_detector_rules();
  test_v25_scout_rules();
  test_real_stop_detected_every_leg_both_sides();
  test_scout_is_distinct_from_free_space_transit();
  test_scout_is_reference_evidence_not_metrology();
  test_scout_backoff_is_the_reviewed_backoff();
  test_fine_pass_plateau_is_bypassed_against_the_scout();
  test_fine_passes_use_the_adaptive_corridor();
  test_fine_pass_2_is_independent_and_scout_referenced();
  test_kinematic_plateau_needs_the_scout();
  test_missing_scout_makes_the_endpoint_unacceptable();
  test_no_stop_fails_closed_at_the_guard();
  test_final_partial_scout_step();
  test_settling_shortfall_is_never_contact();
  test_brief_stop_and_yielding_friction_are_not_contact();
  test_early_friction_never_becomes_contact();
  test_baseline_is_the_v25_forward_move();
  test_settle_gate_rules();
  test_backoff_waits_for_the_v25_settle_gate();
  test_every_stage_fails_closed();
  test_safety_aborts();
  test_start_torque_verified_and_release();
  test_cadence_keeps_v25_timing_at_any_tick_rate();
  test_abort_and_refusals();
  test_to_string_total();
  test_adversarial_model_cases();

  std::printf("test_contact_probe_engine: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
