// Offline tests for the pure 24-contact Full Calibration plan resolver
// (src/calibration/FullLegCalibrationPlan.*) and the geometry-validated
// sequence plan it consumes (src/actuator/CalibrationSequencePlan*.h).
//
// THE 24-PROFILE MATRIX: 4 legs x {UPPER, LOWER, HIP} x {MIN, MAX}. Every
// profile is checked against oracles that are deliberately LITERAL - they
// restate config/MATDOG_SERVO_ALLOCATION.yaml, the URDF directions and
// limits, the compiled Geometry V5 contacts and the validated sequence poses,
// and recompute every raw tick independently - so a wrong dynamic lookup or a
// silently regenerated table cannot pass by being wrong in the same way as the
// code under test. Every joint gets a DIFFERENT q0, so a tick resolved
// against the wrong joint's transform shows up as a number, not a coincidence.
//
// NO HARDWARE VALIDATION. Nothing here measured anything.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationSequencePlanData.h"
#include "../../src/calibration/ContactProbeEngine.h"
#include "../../src/calibration/FullLegCalibrationPlan.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::CalibrationGeometryProfile;
using matdog::actuator::CalibrationSearchCorridor;
using matdog::actuator::CalibrationSequencePlan;
using matdog::actuator::MicroRad;
using matdog::actuator::SequenceLegPlan;

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
constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
constexpr JointKind kAllJoints[3] = {JointKind::HIP, JointKind::UPPER, JointKind::LOWER};

// --- literal oracles ---------------------------------------------------------

struct JointOracle {
  Leg leg;
  JointKind joint;
  uint8_t bus;
  const char* unit;
  int8_t direction;  // URDF motorDirection
  uint16_t q0;       // a DIFFERENT synthetic q0 per joint
};
// config/MATDOG_SERVO_ALLOCATION.yaml; directions from the URDF.
constexpr JointOracle kOracle[12] = {
    {Leg::LF, JointKind::LOWER, 11, "M33", 1, 2011},   {Leg::LF, JointKind::UPPER, 12, "ELR01", 1, 2023},
    {Leg::LF, JointKind::HIP, 13, "M22", 1, 2037},     {Leg::RF, JointKind::LOWER, 21, "NEW03", -1, 2041},
    {Leg::RF, JointKind::UPPER, 22, "ELR03", -1, 2053}, {Leg::RF, JointKind::HIP, 23, "NEW01", 1, 2067},
    {Leg::RH, JointKind::LOWER, 31, "NEW05", -1, 2071}, {Leg::RH, JointKind::UPPER, 32, "ELR02", -1, 2083},
    {Leg::RH, JointKind::HIP, 33, "NEW06", -1, 2097},   {Leg::LH, JointKind::LOWER, 41, "M41", 1, 2101},
    {Leg::LH, JointKind::UPPER, 42, "M42", 1, 2113},   {Leg::LH, JointKind::HIP, 43, "M43", -1, 2127},
};

// URDF limits (micro-radians), identical for the four legs.
constexpr MicroRad kUrdfLower[3] = {-785398, -916298, -1605703};  // HIP, UPPER, LOWER
constexpr MicroRad kUrdfUpper[3] = {785398, 2138028, 654498};

// Compiled Geometry V5 contacts, [leg][joint][MIN|MAX] (micro-radians).
struct ContactOracle {
  Leg leg;
  MicroRad hip[2], upper[2], lower[2];
};
constexpr ContactOracle kContacts[4] = {
    {Leg::LF, {-803056, 789284}, {-909889, 2127120}, {-1606998, 666361}},
    {Leg::RF, {-789284, 803056}, {-909889, 2127120}, {-1606998, 666361}},
    {Leg::RH, {-788125, 803056}, {-909889, 2127120}, {-1606998, 666361}},
    {Leg::LH, {-803056, 788125}, {-909889, 2127120}, {-1606998, 666361}},
};

// The validated sequence poses (CalibrationSequencePlanData.h, regenerated
// from 09_Logs/Validation_Reports/Full_Calibration_Sequence_Geometry_2026-09-30).
// V25 matdog.rs: UPPER_90 1024 ticks, UPPER_85 967, LOWER_FOLDED -990,
// hip_upper_clearance LF 90/85, RF 85/90, rear 90/90. The ONE documented
// deviation: the rear LOWER fold -455 ticks instead of -990 (0.04 mm body
// clearance at -990 in nominal CAD, 2.02 mm at -455).
struct PoseOracle {
  Leg leg;
  int32_t upper_for_lower_ticks, upper_for_hip_min_ticks, upper_for_hip_max_ticks, folded_ticks;
  bool park;
  Leg park_leg;
  uint8_t park_bus;
  int32_t park_ticks;  // 35 deg = 398 ticks
};
constexpr PoseOracle kPoses[4] = {
    {Leg::LF, 1024, 1024, 967, -990, true, Leg::LH, 42, 398},
    {Leg::RF, 1024, 967, 1024, -990, true, Leg::RH, 32, 398},
    {Leg::RH, 1024, 1024, 1024, -455, false, Leg::RH, 0, 0},
    {Leg::LH, 1024, 1024, 1024, -455, false, Leg::LH, 0, 0},
};

const JointOracle& oracleFor(Leg leg, JointKind joint) {
  for (const JointOracle& o : kOracle) {
    if (o.leg == leg && o.joint == joint) return o;
  }
  return kOracle[0];
}
const ContactOracle& contactsFor(Leg leg) {
  for (const ContactOracle& c : kContacts) {
    if (c.leg == leg) return c;
  }
  return kContacts[0];
}
const PoseOracle& posesFor(Leg leg) {
  for (const PoseOracle& p : kPoses) {
    if (p.leg == leg) return p;
  }
  return kPoses[0];
}
MicroRad contactOf(Leg leg, JointKind joint, int side) {
  const ContactOracle& c = contactsFor(leg);
  switch (joint) {
    case JointKind::HIP: return c.hip[side];
    case JointKind::UPPER: return c.upper[side];
    case JointKind::LOWER: return c.lower[side];
  }
  return 0;
}

// Independent restatement of the q -> raw rule: 4096 ticks per 6283186 urad,
// rounded half away from zero, raw = q0 + direction * ticks.
int32_t ticksOf(MicroRad q) {
  const int64_t num = static_cast<int64_t>(q) * 4096;
  return static_cast<int32_t>(num >= 0 ? (num + 3141593) / 6283186 : -((-num + 3141593) / 6283186));
}
int32_t rawOf(const JointOracle& o, MicroRad q) { return o.q0 + o.direction * ticksOf(q); }

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

void primeAll(actuator::JointTransformTable& table, int shift = 0) {
  table.clear();
  for (const JointOracle& o : kOracle) {
    CHECK(table.admit(promotedTransform(identityOf(o), static_cast<uint16_t>(o.q0 + shift))));
  }
}

const CalibrationSequencePlan& kPlan = actuator::sequence_plan_data::kPlan;

FullLegPlanStatus planOf(const CalibrationGeometryProfile& profile,
                         const actuator::JointTransformTable& table, Leg leg, FullLegPlan* out,
                         const CalibrationSequencePlan* plan = &kPlan) {
  return resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, table, plan, leg, out);
}

struct Copy {
  actuator::GeometryJointRecord joints[32];
  actuator::GeometryEndpointRecord endpoints[64];
  uint8_t joint_count;
  uint8_t endpoint_count;
  CalibrationGeometryProfile profile;

  Copy() : joint_count(actuator::geometry_data::kJointCount),
           endpoint_count(actuator::geometry_data::kEndpointCount) {
    for (uint8_t i = 0; i < joint_count; ++i) joints[i] = actuator::geometry_data::kJoints[i];
    for (uint8_t i = 0; i < endpoint_count; ++i) endpoints[i] = actuator::geometry_data::kEndpoints[i];
  }
  void bind() {
    profile.bind(&actuator::geometry_data::kProvenance, joints, joint_count, endpoints, endpoint_count);
  }
  actuator::GeometryEndpointRecord* endpoint(Leg leg, JointKind joint, ContactSide side) {
    for (uint8_t i = 0; i < endpoint_count; ++i) {
      if (endpoints[i].leg == leg && endpoints[i].joint == joint && endpoints[i].side == side) {
        return &endpoints[i];
      }
    }
    return nullptr;
  }
  actuator::GeometryJointRecord* joint(Leg leg, JointKind kind) {
    for (uint8_t i = 0; i < joint_count; ++i) {
      if (joints[i].identity.leg == leg && joints[i].identity.joint == kind) return &joints[i];
    }
    return nullptr;
  }
};

// --- the data the resolver is fed ---------------------------------------------

void test_geometry_v5_data_is_what_the_oracle_states() {
  g_case = "Geometry V5 pin: 12 joints, 24 endpoints";
  CalibrationGeometryProfile profile = boundProfile();
  CHECK(profile.bound());
  CHECK_EQ(actuator::kMicroRadPerRevolution, 6283186);
  CHECK_EQ(actuator::kTicksPerRevolution, 4096);
  for (const JointOracle& o : kOracle) {
    const actuator::GeometryJointRecord* r = profile.findJoint(identityOf(o));
    CHECK(r != nullptr);
    if (r == nullptr) continue;
    CHECK_EQ(r->bus_id, o.bus);
    CHECK_EQ(actuator::jointDirection(profile, identityOf(o)), o.direction);
    const uint8_t k = static_cast<uint8_t>(o.joint);
    CHECK_EQ(r->urdf_lower, kUrdfLower[k]);
    CHECK_EQ(r->urdf_upper, kUrdfUpper[k]);
  }
  int endpoints = 0;
  for (const Leg leg : kAllLegs) {
    for (const JointKind joint : kAllJoints) {
      for (int side = 0; side < 2; ++side) {
        const actuator::GeometryEndpointRecord* e =
            profile.findEndpoint(leg, joint, static_cast<ContactSide>(side));
        CHECK(e != nullptr);
        if (e == nullptr) continue;
        ++endpoints;
        CHECK_EQ(e->contact, contactOf(leg, joint, side));
        CHECK_EQ(e->declared_limit, side == 0 ? kUrdfLower[static_cast<uint8_t>(joint)]
                                              : kUrdfUpper[static_cast<uint8_t>(joint)]);
        // Only the UPPER contacts lie inside the URDF domain; the sixteen
        // HIP/LOWER ones are DIAGNOSTIC for V5's own q=0 context, which is
        // exactly why the sequence plan exists.
        CHECK(actuator::isExecutable(*e) == (joint == JointKind::UPPER));
      }
    }
  }
  CHECK_EQ(endpoints, 24);
}

void test_sequence_plan_data_is_the_validated_artifact() {
  g_case = "sequence plan data pin";
  CHECK(actuator::sequencePlanMatchesGeometry(kPlan, actuator::geometry_data::kProvenance));
  CHECK(std::strcmp(kPlan.schema, "matdog.full_calibration_sequence_geometry.v1") == 0);
  CHECK(std::strlen(kPlan.artifact_sha256) == 64);
  for (const Leg leg : kAllLegs) {
    const SequenceLegPlan* p = actuator::findSequenceLeg(kPlan, leg);
    CHECK(p != nullptr);
    if (p == nullptr) continue;
    const PoseOracle& o = posesFor(leg);
    CHECK(p->geometry_validated);
    CHECK(p->has_rear_park == o.park);
    CHECK_EQ(ticksOf(p->upper_for_lower), o.upper_for_lower_ticks);
    CHECK_EQ(ticksOf(p->upper_for_hip_min), o.upper_for_hip_min_ticks);
    CHECK_EQ(ticksOf(p->upper_for_hip_max), o.upper_for_hip_max_ticks);
    CHECK_EQ(ticksOf(p->lower_folded), o.folded_ticks);
    if (o.park) {
      CHECK(p->park_leg == o.park_leg);
      CHECK(p->park_joint == JointKind::UPPER);
      CHECK_EQ(ticksOf(p->park_target), o.park_ticks);
    }
    // Every pose is inside the URDF domain of its joint.
    CHECK(p->upper_for_lower >= kUrdfLower[kUpper] && p->upper_for_lower <= kUrdfUpper[kUpper]);
    CHECK(p->upper_for_hip_min >= kUrdfLower[kUpper] && p->upper_for_hip_min <= kUrdfUpper[kUpper]);
    CHECK(p->upper_for_hip_max >= kUrdfLower[kUpper] && p->upper_for_hip_max <= kUrdfUpper[kUpper]);
    CHECK(p->lower_folded >= kUrdfLower[kLower] && p->lower_folded <= kUrdfUpper[kLower]);
  }
  CHECK(actuator::findSequenceLeg(kPlan, static_cast<Leg>(9)) == nullptr);
  // A plan for another robot model is not evidence about this one.
  actuator::GeometryProvenance other = actuator::geometry_data::kProvenance;
  other.mesh_manifest_sha256[3] = other.mesh_manifest_sha256[3] == 'a' ? 'b' : 'a';
  CHECK(!actuator::sequencePlanMatchesGeometry(kPlan, other));
}

void test_canonical_identity_and_bus_for_all_twelve_joints() {
  g_case = "canonical identity + bus, 12 joints";
  CalibrationGeometryProfile profile = boundProfile();
  for (const JointOracle& o : kOracle) {
    FullLegJointRef ref{};
    CHECK(resolveLegJoint(profile, o.leg, o.joint, &ref) == FullLegPlanStatus::OK);
    CHECK_EQ(ref.bus_id, o.bus);
    CHECK(ref.identity.leg == o.leg);
    CHECK(ref.identity.joint == o.joint);
    CHECK(std::strcmp(ref.identity.physical_unit, o.unit) == 0);
  }
  FullLegJointRef ref{};
  CHECK(resolveLegJoint(profile, Leg::LF, JointKind::UPPER, nullptr) ==
        FullLegPlanStatus::REJECT_NULL_OUTPUT);
  CHECK(resolveLegJoint(profile, static_cast<Leg>(9), JointKind::UPPER, &ref) ==
        FullLegPlanStatus::REJECT_UNKNOWN_LEG);
  CalibrationGeometryProfile unbound;
  CHECK(resolveLegJoint(unbound, Leg::LF, JointKind::UPPER, &ref) ==
        FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND);
  Copy c;
  c.joint(Leg::LF, JointKind::HIP)->bus_id = 99;
  c.bind();
  CHECK(resolveLegJoint(c.profile, Leg::LF, JointKind::HIP, &ref) ==
        FullLegPlanStatus::REJECT_GEOMETRY_JOINT);
  CHECK_EQ(ref.bus_id, 0);
  Copy d;
  setPhysicalUnit(&d.joint(Leg::RH, JointKind::LOWER)->identity, "WRONG");
  d.bind();
  CHECK(resolveLegJoint(d.profile, Leg::RH, JointKind::LOWER, &ref) ==
        FullLegPlanStatus::REJECT_GEOMETRY_JOINT);
}

// --- THE 24-PROFILE MATRIX ---------------------------------------------------------

void test_the_24_endpoint_profiles() {
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);
  int profiles = 0;
  for (const Leg leg : kAllLegs) {
    g_case = "24-profile matrix";
    FullLegPlan plan{};
    CHECK(planOf(profile, table, leg, &plan) == FullLegPlanStatus::OK);
    const FullLegCalibrationRequest& r = plan.request;
    CHECK(r.leg == leg);
    CHECK_EQ(r.torque_limit, 500);
    CHECK_EQ(r.repeatability_tolerance_ticks, 16);
    CHECK(!r.recovery_only);
    for (const JointKind joint : kAllJoints) {
      const uint8_t k = static_cast<uint8_t>(joint);
      const JointOracle& o = oracleFor(leg, joint);
      const FullLegJoint& j = r.joint[k];
      // identity, bus, q0 from THIS joint's transform, direction
      CHECK_EQ(j.bus_id, o.bus);
      CHECK(j.identity.leg == leg && j.identity.joint == joint);
      CHECK(std::strcmp(j.identity.physical_unit, o.unit) == 0);
      CHECK_EQ(j.q0_tick, o.q0);
      CHECK_EQ(r.direction[k], o.direction);
      CHECK_EQ(r.urdf_lower[k], kUrdfLower[k]);
      CHECK_EQ(r.urdf_upper[k], kUrdfUpper[k]);
      for (int side = 0; side < 2; ++side) {
        ++profiles;
        const CalibrationSearchCorridor& c = r.corridor[k][side];
        const int sign = o.direction * (side == 0 ? -1 : 1);
        const int32_t limit = rawOf(o, side == 0 ? kUrdfLower[k] : kUrdfUpper[k]);
        const int32_t opposite = rawOf(o, side == 0 ? kUrdfUpper[k] : kUrdfLower[k]);
        CHECK(c.valid());
        CHECK_EQ(c.probe_sign, sign);                               // search (coarse+fine) direction
        CHECK_EQ(c.home_tick, o.q0);                                // this joint's q0
        CHECK_EQ(c.contact_tick, rawOf(o, contactOf(leg, joint, side)));  // canonical endpoint
        CHECK_EQ(c.urdf_limit_tick, limit);
        CHECK_EQ(c.opposite_limit_tick, opposite);
        CHECK_EQ(c.entry_tick, limit - sign * 64);                  // search entry
        CHECK_EQ(c.guard_tick, limit + sign * 64);                  // guard
        // The canonical contact lies inside the acceptance band.
        CHECK(actuator::searchCorridorAccepts(c, c.contact_tick));
        // A coarse 64-tick transit from q0 reaches the entry and never the guard.
        CHECK(actuator::searchDepth(c, c.entry_tick) > 0);
        CHECK(actuator::searchDepth(c, c.guard_tick) - actuator::searchDepth(c, c.entry_tick) == 128);
        // The 96-tick backoff from anywhere in the band stays admitted and
        // never crosses q0.
        for (const uint16_t at : {c.entry_tick, c.contact_tick, c.guard_tick}) {
          const int32_t back = static_cast<int32_t>(at) - sign * kSearchBackoffTicks;
          CHECK(actuator::searchCorridorAdmits(c, static_cast<uint16_t>(back)));
          CHECK(actuator::searchDepth(c, static_cast<uint16_t>(back)) > 0);
        }
        // One tick past the guard is refused as a search target.
        CHECK(!actuator::searchCorridorAdmits(c, static_cast<uint16_t>(c.guard_tick + sign)));
      }
    }

    // Prerequisite poses: URDF q from the plan, ticks from the HELD joint's
    // own q0 and direction.
    const PoseOracle& po = posesFor(leg);
    const JointOracle& up = oracleFor(leg, JointKind::UPPER);
    const JointOracle& lo = oracleFor(leg, JointKind::LOWER);
    CHECK_EQ(r.upper_for_lower_tick, up.q0 + up.direction * po.upper_for_lower_ticks);
    CHECK_EQ(r.upper_for_hip_min_tick, up.q0 + up.direction * po.upper_for_hip_min_ticks);
    CHECK_EQ(r.upper_for_hip_max_tick, up.q0 + up.direction * po.upper_for_hip_max_ticks);
    CHECK_EQ(r.lower_folded_tick, lo.q0 + lo.direction * po.folded_ticks);
    CHECK_EQ(ticksOf(r.upper_for_lower_urad), po.upper_for_lower_ticks);
    CHECK_EQ(ticksOf(r.lower_folded_urad), po.folded_ticks);

    // Parking: the front legs park the rear UPPER (Geometry V5's own UPPER MAX
    // auxiliary), the rear legs park nothing.
    CHECK(r.has_rear_park == po.park);
    if (po.park) {
      const JointOracle& pk = oracleFor(po.park_leg, JointKind::UPPER);
      CHECK_EQ(r.park.bus_id, po.park_bus);
      CHECK_EQ(plan.park.bus_id, po.park_bus);
      CHECK(r.park.identity.leg == po.park_leg && r.park.identity.joint == JointKind::UPPER);
      CHECK_EQ(r.park.q0_tick, pk.q0);
      CHECK_EQ(r.park_target_tick, pk.q0 + pk.direction * po.park_ticks);
    } else {
      CHECK_EQ(r.park.bus_id, 0);
      CHECK_EQ(r.park_target_tick, 0);
    }

    // The population: every leg joint of the robot, once, at its own q0.
    CHECK_EQ(r.population_count, 12);
    for (const JointOracle& o : kOracle) {
      int seen = 0;
      for (uint8_t i = 0; i < r.population_count; ++i) {
        if (r.population[i].bus_id == o.bus) {
          ++seen;
          CHECK_EQ(r.population[i].q0_tick, o.q0);
          CHECK(r.population[i].identity.leg == o.leg && r.population[i].identity.joint == o.joint);
        }
      }
      CHECK_EQ(seen, 1);
    }
  }
  CHECK_EQ(profiles, 24);
}

// --- the phase table: held sets, prerequisites, restore path ---------------------

void test_phase_table_allows_exactly_the_v25_moves() {
  g_case = "phase table";
  const CalibrationPhase all[] = {
      CalibrationPhase::PREFLIGHT,        CalibrationPhase::INITIAL_RECOVERY, CalibrationPhase::PARKING,
      CalibrationPhase::UPPER_MIN,        CalibrationPhase::UPPER_MAX,        CalibrationPhase::UPPER_HORIZONTAL,
      CalibrationPhase::LOWER_MIN,        CalibrationPhase::LOWER_MAX,        CalibrationPhase::LOWER_FOLDED,
      CalibrationPhase::HIP_MIN,          CalibrationPhase::HIP_MAX,          CalibrationPhase::DIAGNOSTICS,
      CalibrationPhase::RETURN_HIP,       CalibrationPhase::RETURN_LOWER_HELD, CalibrationPhase::RETURN_UPPER,
      CalibrationPhase::RESTORE_PARKING,  CalibrationPhase::CLEANUP,          CalibrationPhase::TORQUE_OFF,
  };
  for (const Leg leg : kAllLegs) {
    const SequenceLegPlan& p = *actuator::findSequenceLeg(kPlan, leg);
    const JointIdentity hip = identityOf(oracleFor(leg, JointKind::HIP));
    const JointIdentity upper = identityOf(oracleFor(leg, JointKind::UPPER));
    const JointIdentity lower = identityOf(oracleFor(leg, JointKind::LOWER));
    // A joint of another leg that is NOT this leg's park joint.
    const Leg other_leg = leg == Leg::LF ? Leg::RF : Leg::LF;
    const JointIdentity foreign = identityOf(oracleFor(other_leg, JointKind::HIP));
    JointIdentity park{};
    if (p.has_rear_park) park = identityOf(oracleFor(p.park_leg, JointKind::UPPER));
    const MicroRad targets[] = {0, p.park_target, p.upper_for_lower, p.upper_for_hip_min,
                                p.upper_for_hip_max, p.lower_folded, 123456};
    const bool differ = p.upper_for_hip_min != p.upper_for_hip_max;

    for (const CalibrationPhase phase : all) {
      for (const MicroRad t : targets) {
        auto expect = [&](const JointIdentity& j, bool allowed) {
          CHECK(actuator::sequencePlanTargetAllowed(p, phase, j, t) == allowed);
        };
        const bool zero = t == 0;
        switch (phase) {
          case CalibrationPhase::INITIAL_RECOVERY:  // every leg joint, to q=0 only
            expect(hip, zero); expect(upper, zero); expect(lower, zero); expect(foreign, zero);
            if (p.has_rear_park) expect(park, zero);
            break;
          case CalibrationPhase::PARKING:
            expect(hip, false); expect(upper, false); expect(lower, false); expect(foreign, false);
            if (p.has_rear_park) expect(park, t == p.park_target);
            break;
          case CalibrationPhase::UPPER_MIN:  // V25 prerequisites_for(UPPER): HIP, LOWER at q=0
            expect(hip, zero); expect(lower, zero); expect(upper, false); expect(foreign, false);
            break;
          case CalibrationPhase::UPPER_HORIZONTAL:
            expect(upper, t == p.upper_for_lower); expect(hip, false); expect(lower, false);
            break;
          case CalibrationPhase::LOWER_FOLDED:
            expect(lower, t == p.lower_folded); expect(upper, t == p.upper_for_hip_min);
            expect(hip, false); expect(foreign, false);
            break;
          case CalibrationPhase::HIP_MAX:
            expect(hip, differ && zero); expect(upper, differ && t == p.upper_for_hip_max);
            expect(lower, false);
            break;
          case CalibrationPhase::RETURN_HIP:
            expect(hip, zero); expect(upper, false); expect(lower, false);
            break;
          case CalibrationPhase::RETURN_LOWER_HELD:
            expect(lower, zero); expect(hip, false); expect(upper, false);
            break;
          case CalibrationPhase::RETURN_UPPER:
            expect(upper, zero); expect(hip, false); expect(lower, false);
            break;
          case CalibrationPhase::RESTORE_PARKING:
            expect(hip, false); expect(upper, false); expect(lower, false);
            if (p.has_rear_park) expect(park, zero);
            break;
          default:  // the probes move only through the probe path; the rest never move
            expect(hip, false); expect(upper, false); expect(lower, false); expect(foreign, false);
            if (p.has_rear_park) expect(park, false);
            break;
        }
        // Outside INITIAL_RECOVERY a foreign joint never moves.
        if (phase != CalibrationPhase::INITIAL_RECOVERY) {
          CHECK(!actuator::sequencePlanTargetAllowed(p, phase, foreign, t));
        }
      }
      // Energizing a limp joint: only where a phase first needs it.
      CHECK(actuator::sequenceEnergizeAllowed(p, phase, foreign) ==
            (phase == CalibrationPhase::INITIAL_RECOVERY));
      CHECK(actuator::sequenceEnergizeAllowed(p, phase, upper) ==
            (phase == CalibrationPhase::INITIAL_RECOVERY || phase == CalibrationPhase::UPPER_MIN));
      if (p.has_rear_park) {
        CHECK(actuator::sequenceEnergizeAllowed(p, phase, park) ==
              (phase == CalibrationPhase::INITIAL_RECOVERY || phase == CalibrationPhase::PARKING));
      }
    }
    // A plan that failed its geometry validation authorizes nothing.
    SequenceLegPlan unvalidated = p;
    unvalidated.geometry_validated = false;
    CHECK(!actuator::sequencePlanTargetAllowed(unvalidated, CalibrationPhase::INITIAL_RECOVERY, hip, 0));
    CHECK(!actuator::sequenceEnergizeAllowed(unvalidated, CalibrationPhase::UPPER_MIN, hip));
  }

  // The six measurement phases map to exactly the six endpoints, in V25 order.
  JointKind j = JointKind::HIP;
  ContactSide s = ContactSide::MIN_SIDE;
  CHECK(actuator::sequenceProbeEndpoint(CalibrationPhase::UPPER_MIN, &j, &s) && j == JointKind::UPPER && s == ContactSide::MIN_SIDE);
  CHECK(actuator::sequenceProbeEndpoint(CalibrationPhase::UPPER_MAX, &j, &s) && j == JointKind::UPPER && s == ContactSide::MAX_SIDE);
  CHECK(actuator::sequenceProbeEndpoint(CalibrationPhase::LOWER_MIN, &j, &s) && j == JointKind::LOWER && s == ContactSide::MIN_SIDE);
  CHECK(actuator::sequenceProbeEndpoint(CalibrationPhase::LOWER_MAX, &j, &s) && j == JointKind::LOWER && s == ContactSide::MAX_SIDE);
  CHECK(actuator::sequenceProbeEndpoint(CalibrationPhase::HIP_MIN, &j, &s) && j == JointKind::HIP && s == ContactSide::MIN_SIDE);
  CHECK(actuator::sequenceProbeEndpoint(CalibrationPhase::HIP_MAX, &j, &s) && j == JointKind::HIP && s == ContactSide::MAX_SIDE);
  int probes = 0;
  for (const CalibrationPhase phase : all) probes += actuator::sequenceProbeEndpoint(phase, nullptr, nullptr);
  CHECK_EQ(probes, 6);
}

// --- refusals -------------------------------------------------------------------

void test_plan_refusals() {
  g_case = "plan refusals";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);
  FullLegPlan plan{};

  CHECK(planOf(profile, table, Leg::LF, &plan, nullptr) == FullLegPlanStatus::REJECT_NO_SEQUENCE_PLAN);
  CHECK(resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, table, &kPlan, Leg::LF,
                           nullptr) == FullLegPlanStatus::REJECT_NULL_OUTPUT);
  CHECK(planOf(profile, table, static_cast<Leg>(9), &plan) == FullLegPlanStatus::REJECT_UNKNOWN_LEG);
  CalibrationGeometryProfile unbound;
  CHECK(planOf(unbound, table, Leg::LF, &plan) == FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND);
  actuator::GeometryProvenance other = actuator::geometry_data::kProvenance;
  other.urdf_sha256[0] = other.urdf_sha256[0] == 'a' ? 'b' : 'a';
  CHECK(resolveFullLegPlan(profile, other, table, &kPlan, Leg::LF, &plan) ==
        FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND);

  // A sequence plan computed on another model, or one that failed validation.
  CalibrationSequencePlan wrong_model = kPlan;
  wrong_model.urdf_sha256 = "0000000000000000000000000000000000000000000000000000000000000000";
  CHECK(planOf(profile, table, Leg::RF, &plan, &wrong_model) == FullLegPlanStatus::REJECT_NO_SEQUENCE_PLAN);
  CalibrationSequencePlan not_validated = kPlan;
  not_validated.legs[static_cast<uint8_t>(Leg::RH)].geometry_validated = false;
  CHECK(planOf(profile, table, Leg::RH, &plan, &not_validated) ==
        FullLegPlanStatus::REJECT_SEQUENCE_NOT_VALIDATED);
  CHECK(planOf(profile, table, Leg::LH, &plan, &not_validated) == FullLegPlanStatus::OK);
  CalibrationSequencePlan misfiled = kPlan;
  misfiled.legs[static_cast<uint8_t>(Leg::LH)].leg = Leg::RH;
  CHECK(planOf(profile, table, Leg::LH, &plan, &misfiled) == FullLegPlanStatus::REJECT_NO_SEQUENCE_PLAN);

  // The park must be exactly Geometry V5's own UPPER MAX auxiliary.
  {
    CalibrationSequencePlan p = kPlan;
    p.legs[static_cast<uint8_t>(Leg::LF)].park_target += 1000;
    CHECK(planOf(profile, table, Leg::LF, &plan, &p) ==
          FullLegPlanStatus::REJECT_PARK_INCONSISTENT);
  }
  {
    CalibrationSequencePlan p = kPlan;
    p.legs[static_cast<uint8_t>(Leg::RF)].park_leg = Leg::LH;
    CHECK(planOf(profile, table, Leg::RF, &plan, &p) == FullLegPlanStatus::REJECT_PARK_INCONSISTENT);
  }
  {
    CalibrationSequencePlan p = kPlan;
    p.legs[static_cast<uint8_t>(Leg::RH)].has_rear_park = true;  // V5 parks nothing for RH
    CHECK(planOf(profile, table, Leg::RH, &plan, &p) == FullLegPlanStatus::REJECT_PARK_INCONSISTENT);
  }
  {
    CalibrationSequencePlan p = kPlan;
    p.legs[static_cast<uint8_t>(Leg::LF)].has_rear_park = false;  // V5 requires LH_UPPER parked
    CHECK(planOf(profile, table, Leg::LF, &plan, &p) == FullLegPlanStatus::REJECT_PARK_INCONSISTENT);
  }
  // A prerequisite pose the checked resolver refuses (outside the URDF).
  {
    CalibrationSequencePlan p = kPlan;
    p.legs[static_cast<uint8_t>(Leg::LH)].lower_folded = -1700000;
    CHECK(planOf(profile, table, Leg::LH, &plan, &p) == FullLegPlanStatus::REJECT_TARGET_RESOLUTION);
  }
  // Geometry V5 missing an endpoint of the leg.
  {
    Copy c;
    c.endpoint(Leg::RF, JointKind::LOWER, ContactSide::MAX_SIDE)->leg = Leg::LF;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::RF, &plan) == FullLegPlanStatus::REJECT_ENDPOINT_MISSING);
  }
  // A canonical contact the corridor cannot contain.
  {
    Copy c;
    c.endpoint(Leg::LH, JointKind::HIP, ContactSide::MAX_SIDE)->contact = 600000;  // 34 deg
    c.bind();
    CHECK(planOf(c.profile, table, Leg::LH, &plan) == FullLegPlanStatus::REJECT_SEARCH_CORRIDOR);
  }
}

void test_plan_requires_all_twelve_current_transforms() {
  g_case = "12 current transforms";
  CalibrationGeometryProfile profile = boundProfile();
  FullLegPlan plan{};
  actuator::JointTransformTable empty;
  for (const Leg leg : kAllLegs) {
    CHECK(planOf(profile, empty, leg, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
  }
  // Any ONE of the twelve missing refuses every leg: INITIAL_RECOVERY and the
  // bystander watch need all of them.
  for (const JointOracle& missing : kOracle) {
    actuator::JointTransformTable t;
    for (const JointOracle& o : kOracle) {
      if (o.bus != missing.bus) CHECK(t.admit(promotedTransform(identityOf(o), o.q0)));
    }
    for (const Leg leg : kAllLegs) {
      CHECK(planOf(profile, t, leg, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
    }
  }
  // A transform measured under another geometry is not current evidence.
  actuator::JointTransformTable stale;
  for (const JointOracle& o : kOracle) {
    actuator::JointTransform tr = promotedTransform(identityOf(o), o.q0);
    if (o.bus == 33) tr.geometry = tr.geometry + 1;
    CHECK(stale.admit(tr));
  }
  CHECK(planOf(profile, stale, Leg::LF, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
  // A q0 so close to raw 0 that a corridor would wrap: refused, never wrapped.
  actuator::JointTransformTable low;
  for (const JointOracle& o : kOracle) {
    CHECK(low.admit(promotedTransform(identityOf(o), o.bus == 12 ? 100 : o.q0)));
  }
  CHECK(planOf(profile, low, Leg::LF, &plan) != FullLegPlanStatus::OK);
  CHECK(planOf(profile, low, Leg::RH, &plan) == FullLegPlanStatus::OK);
}

void test_refused_plan_leaves_no_partial_request() {
  g_case = "no partial request on refusal";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  FullLegPlan plan{};
  plan.request.joint[kUpper].bus_id = 77;  // stale garbage from an earlier plan
  plan.request.population_count = 12;
  plan.request.has_rear_park = true;
  CHECK(planOf(profile, table, Leg::LF, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
  CHECK_EQ(plan.request.joint[kUpper].bus_id, 0);
  CHECK_EQ(plan.request.population_count, 0);
  CHECK(!plan.request.has_rear_park);
  CHECK_EQ(plan.request.torque_limit, 0);
}

void test_status_names_are_unique_and_stable() {
  g_case = "status names";
  const int n = static_cast<int>(FullLegPlanStatus::REJECT_TARGET_RESOLUTION) + 1;
  for (int i = 0; i < n; ++i) {
    const char* a = toString(static_cast<FullLegPlanStatus>(i));
    CHECK(std::strcmp(a, "UNKNOWN") != 0);
    for (int k = i + 1; k < n; ++k) CHECK(std::strcmp(a, toString(static_cast<FullLegPlanStatus>(k))) != 0);
  }
  CHECK(std::strcmp(toString(static_cast<FullLegPlanStatus>(200)), "UNKNOWN") == 0);
}

}  // namespace

int main() {
  test_geometry_v5_data_is_what_the_oracle_states();
  test_sequence_plan_data_is_the_validated_artifact();
  test_canonical_identity_and_bus_for_all_twelve_joints();
  test_the_24_endpoint_profiles();
  test_phase_table_allows_exactly_the_v25_moves();
  test_plan_refusals();
  test_plan_requires_all_twelve_current_transforms();
  test_refused_plan_leaves_no_partial_request();
  test_status_names_are_unique_and_stable();

  std::printf("test_full_leg_calibration_plan: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
