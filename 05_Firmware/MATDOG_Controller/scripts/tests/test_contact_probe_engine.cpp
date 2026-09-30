// Offline tests for the staged calibration endpoint search
// (src/calibration/ContactProbeEngine.*), 2026-09-29.
//
// Links the REAL SafeActuatorPolicy, ActuatorRuntime, CalibrationExecutionEngine,
// ActuatorAuthorityArbiter, Geometry V5 profile and checked resolvers, and drives
// them against kinematic_servo_sim.h - a synthetic ST3215 model reproducing the
// LF_UPPER hardware behaviours (settling 4-5 ticks short of a goal; a hard stop
// ~23 ticks PAST the modelled contact; friction plateaus).
//
// NO HARDWARE VALIDATION, no fabricated measurement: every stop, plateau and
// current in this file is a synthetic construct.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstdlib>
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
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = kSearchStaticToleranceTicks + 2;
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
  // H: pass 2's adaptive bound admits up to 32 ticks HOME-ward of pass 1.
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

// --- 2. the real stop, every leg, both sides ---------------------------------

void checkStepDiscipline(ProbeRig& rig) {
  const int entry_d = rig.depth(rig.corridor.entry_tick);
  const int guard_d = rig.depth(rig.corridor.guard_tick);
  const ContactProbeStatus& st = rig.probe.status();
  int prev = rig.u.q0;
  bool seen_backoff = false;
  bool seen_fine = false;
  // A COMPLETE probe's LAST write is the V25 stop_pressure() release:
  // GoalPosition := the accepted pass-2 contact, never deeper than the last
  // step. It is checked on its own, then excluded from the step discipline.
  size_t steps = rig.log.size();
  if (st.phase == ContactProbePhase::COMPLETE) {
    CHECK(steps >= 2);
    const WriteLog& release = rig.log.back();
    CHECK_EQ(release.tick, st.pass2_contact_tick);
    CHECK_EQ(st.target_tick, st.pass2_contact_tick);
    CHECK(rig.depth(release.tick) <= rig.depth(rig.log[steps - 2].tick));
    CHECK(release.profile == MotionProfile::CALIBRATION_SEARCH);
    --steps;
  }
  for (size_t i = 0; i < steps; ++i) {
    const WriteLog& w = rig.log[i];
    CHECK(w.profile == MotionProfile::CALIBRATION_SEARCH);
    CHECK(actuator::searchCorridorAdmits(rig.corridor, w.tick));
    CHECK(rig.depth(w.tick) <= guard_d);  // never past the guard
    const int delta = rig.depth(w.tick) - rig.depth(prev);
    if (w.stage == ContactSearchStage::BACKOFF) {
      CHECK(!seen_backoff);
      seen_backoff = true;
      CHECK_EQ(rig.depth(w.tick), rig.depth(st.pass1_contact_tick) - 96);  // V25 BACKOFF_TICKS
    } else if (w.stage == ContactSearchStage::COARSE_TRANSIT) {
      CHECK(!seen_fine);
      CHECK(w.pass == 1);
      CHECK(delta > 0 && delta <= 64);
      CHECK(rig.depth(w.tick) <= entry_d);  // coarse never enters the corridor
    } else {
      seen_fine = true;
      CHECK(w.stage == ContactSearchStage::FINE_SEARCH);
      CHECK(rig.depth(prev) >= entry_d || w.pass == 2 || i == 0);
      // Fine steps are exactly 8 target ticks; the first one of a pass steps
      // from where the joint actually is (within its settle band).
      if (i > 0 && rig.log[i - 1].stage != ContactSearchStage::BACKOFF) {
        CHECK_EQ(delta, 8);
      } else {
        CHECK(delta >= 8 - 12 && delta <= 8 + 12);
      }
    }
    prev = w.tick;
  }
  CHECK(seen_backoff);
  CHECK(seen_fine);
}

void test_real_stop_detected_every_leg_both_sides() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      for (int beyond : {kHardwareStopBeyondContact, 0, -5, 50}) {
        g_case = "real stop (contact-5 .. contact+50): COMPLETE at the stop, every leg, MIN+MAX";
        ProbeRig rig(u, side);
        rig.placeStop(beyond);
        const uint32_t elapsed = runProbe(rig);
        const ContactProbeStatus& st = rig.probe.status();
        CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
        CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NONE);
        CHECK(std::abs(st.pass1_contact_tick - rig.stopTick()) <= 1);
        CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
        CHECK(rig.probe.witness().accepted());
        CHECK_EQ(st.plateau_bypass_count, 0);
        checkStepDiscipline(rig);
        // Bounded contact energy: the commanded target never ran more than one
        // settle band + one fine step (10 + 8) past the physical stop.
        for (const WriteLog& w : rig.log) {
          CHECK(rig.depth(w.tick) - rig.depth(rig.stopTick()) <= 18);
        }
        // Fast: the whole two-pass side in well under the old 25 s single probe
        // for MIN; MAX travels ~1350 ticks from q0 first.
        CHECK(elapsed < (side == ContactSide::MIN_SIDE ? 12000u : 20000u));
        CHECK(rig.joint().torque);  // COMPLETE: SAFE_OFF is the executor's next step
      }
    }
  }
}

// --- 3. no stop: the guard ends it, fail-closed -------------------------------

void test_no_stop_fails_closed_at_the_guard() {
  for (const UpperCase& u : kUppers) {
    for (ContactSide side : kSides) {
      g_case = "no stop: NO_CONTACT_BEFORE_GUARD, never a target past the guard";
      ProbeRig rig(u, side);
      runProbe(rig);
      const ContactProbeStatus& st = rig.probe.status();
      const int guard_d = rig.depth(rig.corridor.guard_tick);
      CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
      CHECK_EQ((int)st.failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
      CHECK_EQ(st.last_candidate_tick, 0);
      CHECK_EQ(st.pass1_contact_tick, 0);
      CHECK(rig.depth(st.target_tick) <= guard_d);
      CHECK(rig.depth(st.target_tick) > guard_d - 8);  // it really did reach the guard
      const simk::SimJoint& j = rig.joint();
      const int deepest = rig.corridor.probe_sign < 0 ? (int)j.lo_seen : (int)j.hi_seen;
      CHECK(rig.depth(deepest) <= guard_d);
    }
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
    rig2.placeStop(kHardwareStopBeyondContact);
    runProbe(rig2);
    CHECK_EQ((int)rig2.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig2.probe.status().pass1_contact_tick - rig2.stopTick()) <= 1);
  }
}

// --- 5. brief stops and yielding friction are never contact -------------------

void test_brief_stop_and_yielding_friction_are_not_contact() {
  {
    g_case = "a 100 ms stop inside the corridor is not contact";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 30);
    j.plateau_breakaway = 1000;  // holds regardless of error...
    j.plateau_hold_ms = 100;     // ...but only for 100 ms
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    g_case = "friction that yields to <= 10 ticks of error is not contact";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    simk::SimJoint& j = rig.joint();
    j.plateau = true;
    j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 30);
    j.plateau_breakaway = 10;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
}

// --- 6. a stall before the corridor is an anomaly -----------------------------

void test_stall_before_corridor_is_early_stall() {
  for (int short_of_entry : {100, 10}) {
    g_case = "stop before the corridor entry: EARLY_STALL_OUTSIDE_CORRIDOR, never contact";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.joint().press_gain = 2;  // keep the hard-current abort out of this case
    const int entry_d = rig.depth(rig.corridor.entry_tick);
    rig.placeStop(entry_d - short_of_entry - rig.depth(rig.corridor.contact_tick));
    runProbe(rig);
    const ContactProbeStatus& st = rig.probe.status();
    CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)st.failure, (int)ContactProbeFailure::EARLY_STALL_OUTSIDE_CORRIDOR);
    CHECK_EQ(st.pass1_contact_tick, 0);
  }
}

// --- 7. pass 2 must reproduce pass 1 ------------------------------------------

void test_pass2_friction_plateau_is_stepped_past() {
  g_case = "pass-2 plateau > 8 ticks short of pass 1: bypassed, contact at the stop";
  ProbeRig rig(kUppers[3], ContactSide::MAX_SIDE);
  rig.placeStop(kHardwareStopBeyondContact);
  bool armed = false;
  runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
    if (!armed && r.probe.status().pass == 2) {
      armed = true;
      simk::SimJoint& j = r.joint();
      j.plateau = true;
      j.plateau_tick = r.tickAt(r.depth(r.probe.status().pass1_contact_tick) - 20);
      j.plateau_breakaway = 18;  // holds through one full fine step of error
    }
  });
  const ContactProbeStatus& st = rig.probe.status();
  CHECK_EQ((int)st.phase, (int)ContactProbePhase::COMPLETE);
  CHECK(st.plateau_bypass_count >= 1);
  CHECK(std::abs(st.pass2_contact_tick - rig.stopTick()) <= 1);
}

void test_non_repeatable_contact_fails() {
  g_case = "pass 2 finds the stop 30 ticks deeper: REPEATABILITY_FAILED";
  ProbeRig rig(kUppers[0], ContactSide::MAX_SIDE);
  rig.placeStop(0);
  bool moved = false;
  runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
    if (!moved && r.probe.status().pass == 2) {
      moved = true;
      simk::SimJoint& j = r.joint();
      if (r.corridor.probe_sign < 0) j.stop_low -= 30; else j.stop_high += 30;
    }
  });
  const ContactProbeStatus& st = rig.probe.status();
  CHECK_EQ((int)st.phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)st.failure, (int)ContactProbeFailure::REPEATABILITY_FAILED);
  CHECK(!rig.probe.witness().evaluated);
}

// Documented limitation (see ContactProbeEngine.h): with no coarse contact
// scout, a plateau that holds against a FULL fine step of error (up to 18
// ticks) on both passes is kinematically identical to a stop.
void test_limitation_plateau_holding_full_fine_error_on_both_passes() {
  g_case = "LIMITATION: a plateau holding >= 18 ticks of error on both passes reads as contact";
  ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
  rig.placeStop(kHardwareStopBeyondContact);
  simk::SimJoint& j = rig.joint();
  j.plateau = true;
  j.plateau_tick = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 30);
  j.plateau_breakaway = 18;
  runProbe(rig);
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
  CHECK(std::abs(rig.probe.status().pass1_contact_tick - (int)j.plateau_tick) <= 1);
}

// --- 8. safety aborts -----------------------------------------------------------

void expectFailure(ProbeRig& rig, ContactProbeFailure f) {
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
  CHECK_EQ((int)rig.probe.status().failure, (int)f);
}

void test_safety_aborts() {
  {
    g_case = "pressing current >= 200 raw: HARD_CURRENT_ABORT";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    rig.joint().press_gain = 25;
    runProbe(rig);
    expectFailure(rig, ContactProbeFailure::HARD_CURRENT_ABORT);
  }
  {
    g_case = "torque drops mid-search: TORQUE_UNEXPECTEDLY_OFF";
    ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().stage == ContactSearchStage::FINE_SEARCH) r.joint().torque = false;
    });
    expectFailure(rig, ContactProbeFailure::TORQUE_UNEXPECTEDLY_OFF);
  }
  {
    g_case = "temperature above 70 C: OVER_TEMPERATURE";
    ProbeRig rig(kUppers[2], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().step_count >= 3) r.joint().temperature = 71;
    });
    expectFailure(rig, ContactProbeFailure::OVER_TEMPERATURE);
  }
  {
    g_case = "no telemetry for 3 s: STALE_TELEMETRY";
    ProbeRig rig(kUppers[3], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    uint32_t cut_at = 0;
    uint32_t failed_after = 0;
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t t, bool* available, TelemetrySample*) {
      if (r.probe.status().step_count >= 2) {
        if (cut_at == 0) cut_at = t;
        *available = false;
        failed_after = t - cut_at;
      }
    });
    expectFailure(rig, ContactProbeFailure::STALE_TELEMETRY);
    CHECK(failed_after >= 2990 && failed_after <= 3020);
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
    g_case = "current stays high after the backoff: CURRENT_NOT_RECOVERED";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().pass1_contact_tick != 0) r.joint().current_override = 150;
    });
    expectFailure(rig, ContactProbeFailure::CURRENT_NOT_RECOVERED);
    CHECK(rig.probe.status().baseline_samples >= kSearchBaselineMinSamples);
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
}

// --- 8b. the rest of V25's per-observation readback ------------------------------

void test_v25_register_readback_aborts() {
  {
    g_case = "RAM TorqueLimit no longer 500: TORQUE_LIMIT_CHANGED";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().step_count >= 3) r.joint().torque_limit = 1000;
    });
    expectFailure(rig, ContactProbeFailure::TORQUE_LIMIT_CHANGED);
  }
  {
    g_case = "servo status byte set: SERVO_STATUS_FAULT";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().step_count >= 3) r.joint().status = 0x20;
    });
    expectFailure(rig, ContactProbeFailure::SERVO_STATUS_FAULT);
  }
  {
    g_case = "GoalPosition not what the probe commanded: GOAL_READBACK_MISMATCH";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample* s) {
      if (r.probe.status().step_count >= 3) s->goal_position = s->goal_position + 30;
    });
    expectFailure(rig, ContactProbeFailure::GOAL_READBACK_MISMATCH);
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
    CHECK(failed_after >= 2990);  // treated as no sample, never as a good one
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
  rig.placeStop(kHardwareStopBeyondContact);
  rig.joint().torque = true;  // the sequence energized it (prime, limit, torque)
  rig.joint().goal = rig.joint().position();
  ContactProbeRequest r = rig.request();
  r.start_torque_verified = true;
  CHECK(rig.probe.start(r, rig.ctx(), 1000));
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::STEP_PENDING);
  uint32_t t = 1000;
  while (rig.probe.active() && t < 200000) {
    t += 10;
    rig.backend.advance(t, 10);
    rig.probe.update(rig.ctx(), t, true, rig.joint().sample(t));
  }
  CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
  CHECK_EQ(rig.backend.torque_writes[rig.u.bus], 0);
  // V25 stop_pressure(): the last write parks GoalPosition on the pass-2
  // contact, so the joint rests on the stop without pressing into it.
  CHECK(!rig.backend.writes.empty());
  CHECK_EQ(rig.backend.writes.back().tick, rig.probe.status().pass2_contact_tick);
  CHECK(rig.joint().torque);
  CHECK(std::abs(rig.joint().goal - rig.joint().position()) <= 4);
}

// --- 9. cadence: a brief stop can never confirm at a fast tick rate -----------

void test_cadence_keeps_v25_timing_at_any_tick_rate() {
  for (uint32_t dt : {2u, 5u, 10u}) {
    g_case = "contact confirmation needs >= 7 x 20 ms of stall whatever the tick rate";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    uint32_t pressing_since = 0;
    uint32_t confirmed_after = 0;
    runProbe(rig, dt, 180000, [&](ProbeRig& r, uint32_t t, bool*, TelemetrySample*) {
      const ContactProbeStatus& st = r.probe.status();
      if (st.pass != 1 || st.pass1_contact_tick != 0) {
        if (confirmed_after == 0 && st.pass1_contact_tick != 0 && pressing_since != 0) {
          confirmed_after = t - pressing_since;
        }
        return;
      }
      const bool at_stop = r.joint().position() == r.stopTick();
      const bool ahead = r.depth(st.target_tick) - r.depth(r.stopTick()) > 10;
      if (at_stop && ahead && pressing_since == 0) pressing_since = t;
      if (!(at_stop && ahead)) pressing_since = 0;
    });
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(confirmed_after >= 6 * kSearchSampleIntervalMs);
  }
}

// --- 10. abort, refusals ------------------------------------------------------

void test_abort_and_refusals() {
  {
    g_case = "abort mid-search requires SAFE_OFF";
    ProbeRig rig(kUppers[0], ContactSide::MIN_SIDE);
    rig.placeStop(0);
    runProbe(rig, 10, 180000, [&](ProbeRig& r, uint32_t, bool*, TelemetrySample*) {
      if (r.probe.status().stage == ContactSearchStage::FINE_SEARCH) r.probe.abort();
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
  for (int p = 0; p <= (int)ContactProbePhase::SAFE_OFF_REQUIRED; ++p) {
    CHECK(std::strcmp(toString(static_cast<ContactProbePhase>(p)), "UNKNOWN") != 0);
  }
  for (int f = 0; f <= (int)ContactProbeFailure::OPERATOR_ABORT; ++f) {
    CHECK(std::strcmp(toString(static_cast<ContactProbeFailure>(f)), "UNKNOWN") != 0);
  }
  for (int s = 0; s <= (int)ContactSearchStage::BACKOFF; ++s) {
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
    rig.placeStop(kHardwareStopBeyondContact);
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    g_case = "speed-register noise in 1 of 4 20-ms slots: the real stop is still found";
    ProbeRig rig(kUppers[1], ContactSide::MIN_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    rig.joint().speed_noise_every = 4;
    rig.joint().speed_noise_raw = 25;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    g_case = "speed noise in every other 20-ms slot: never 3 clean samples -> fails closed";
    ProbeRig rig(kUppers[2], ContactSide::MAX_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    rig.joint().speed_noise_every = 2;
    rig.joint().speed_noise_raw = 25;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::TRACKING_FAILED);
    CHECK_EQ(rig.probe.status().pass1_contact_tick, 0);
  }
  {
    g_case = "temporary slowdown to 30 ticks/s inside the corridor is not contact";
    ProbeRig rig(kUppers[3], ContactSide::MIN_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
    simk::SimJoint& j = rig.joint();
    const int a = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 40);
    const int b = rig.tickAt(rig.depth(rig.corridor.contact_tick) - 10);
    j.slow_zone = true;
    j.slow_lo = a < b ? a : b;
    j.slow_hi = a < b ? b : a;
    j.slow_speed_tps = 30;
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
    CHECK(std::abs(rig.probe.status().pass1_contact_tick - rig.stopTick()) <= 1);
  }
  {
    g_case = "pass 1 alone is never enough: stop gone for pass 2 -> fails closed";
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
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::SAFE_OFF_REQUIRED);
    CHECK_EQ((int)rig.probe.status().failure, (int)ContactProbeFailure::NO_CONTACT_BEFORE_GUARD);
    CHECK(!rig.probe.witness().evaluated);
  }
  {
    g_case = "an obstruction during the backoff is an anomaly, never contact";
    ProbeRig rig(kUppers[1], ContactSide::MAX_SIDE);
    rig.placeStop(kHardwareStopBeyondContact);
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
    g_case = "a stop exactly at the corridor entry is accepted (inclusive bound)";
    ProbeRig rig(kUppers[2], ContactSide::MIN_SIDE);
    rig.placeStop(rig.depth(rig.corridor.entry_tick) - rig.depth(rig.corridor.contact_tick));
    runProbe(rig);
    CHECK_EQ((int)rig.probe.status().phase, (int)ContactProbePhase::COMPLETE);
  }
}

}  // namespace

int main() {
  test_detector_rules();
  test_real_stop_detected_every_leg_both_sides();
  test_no_stop_fails_closed_at_the_guard();
  test_settling_shortfall_is_never_contact();
  test_brief_stop_and_yielding_friction_are_not_contact();
  test_stall_before_corridor_is_early_stall();
  test_pass2_friction_plateau_is_stepped_past();
  test_non_repeatable_contact_fails();
  test_limitation_plateau_holding_full_fine_error_on_both_passes();
  test_safety_aborts();
  test_v25_register_readback_aborts();
  test_start_torque_verified_and_release();
  test_cadence_keeps_v25_timing_at_any_tick_rate();
  test_abort_and_refusals();
  test_to_string_total();
  test_adversarial_model_cases();

  std::printf("test_contact_probe_engine: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
