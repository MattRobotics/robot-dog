// Offline tests for the pure four-leg plan resolver
// (src/calibration/FullLegCalibrationPlan.*).
//
// The oracle tables below (bus ids, physical units, the parking matrix, the
// contact numbers) are deliberately LITERAL: they restate
// config/MATDOG_SERVO_ALLOCATION.yaml and the compiled Geometry V5 result, so a
// wrong dynamic lookup or a silently regenerated profile cannot pass by being
// wrong in the same way as the code under test.
//
// NO HARDWARE VALIDATION. Nothing here measured anything.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/calibration/FullLegCalibrationPlan.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::CalibrationGeometryProfile;

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

struct JointOracle {
  Leg leg;
  JointKind joint;
  uint8_t bus;
  const char* unit;
};

// config/MATDOG_SERVO_ALLOCATION.yaml, the 12 leg joints.
constexpr JointOracle kOracle[12] = {
    {Leg::LF, JointKind::LOWER, 11, "M33"},   {Leg::LF, JointKind::UPPER, 12, "ELR01"},
    {Leg::LF, JointKind::HIP, 13, "M22"},     {Leg::RF, JointKind::LOWER, 21, "NEW03"},
    {Leg::RF, JointKind::UPPER, 22, "ELR03"}, {Leg::RF, JointKind::HIP, 23, "NEW01"},
    {Leg::RH, JointKind::LOWER, 31, "NEW05"}, {Leg::RH, JointKind::UPPER, 32, "ELR02"},
    {Leg::RH, JointKind::HIP, 33, "NEW06"},   {Leg::LH, JointKind::LOWER, 41, "M41"},
    {Leg::LH, JointKind::UPPER, 42, "M42"},   {Leg::LH, JointKind::HIP, 43, "M43"},
};

constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};

const JointOracle& oracleFor(Leg leg, JointKind joint) {
  for (const JointOracle& o : kOracle) {
    if (o.leg == leg && o.joint == joint) return o;
  }
  return kOracle[0];
}

// The parking matrix the CURRENT compiled Geometry V5 resolves.
struct ParkingOracle {
  Leg leg;
  bool aux_required;
  Leg aux_leg;
  uint8_t aux_bus;
};
constexpr ParkingOracle kParking[4] = {
    {Leg::LF, true, Leg::LH, 42},
    {Leg::RF, true, Leg::RH, 32},
    {Leg::RH, false, Leg::RH, 0},
    {Leg::LH, false, Leg::LH, 0},
};

// Current compiled Geometry V5, UPPER endpoints (urad).
constexpr actuator::MicroRad kUpperMinContact = -909889;
constexpr actuator::MicroRad kUpperMaxContact = 2127120;
constexpr actuator::MicroRad kAuxiliaryParkTarget = 610865;

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

actuator::JointTransform promotedTransform(const JointIdentity& id, uint16_t q0 = 2048) {
  actuator::JointTransform t{};
  t.identity = id;
  t.geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.q0_tick = q0;
  t.present = true;
  return t;
}

// All 12 leg transforms, as after @CALIBRATION Q0 PROMOTE.
void primeAll(actuator::JointTransformTable& table, uint16_t q0 = 2048) {
  table.clear();
  for (const JointOracle& o : kOracle) {
    CHECK(table.admit(promotedTransform(identityOf(o), q0)));
  }
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
    for (uint8_t i = 0; i < endpoint_count; ++i) {
      endpoints[i] = actuator::geometry_data::kEndpoints[i];
    }
  }
  void bind() {
    profile.bind(&actuator::geometry_data::kProvenance, joints, joint_count, endpoints,
                 endpoint_count);
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

FullLegPlanStatus planOf(const CalibrationGeometryProfile& profile,
                         const actuator::JointTransformTable& table, Leg leg, FullLegPlan* out) {
  return resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, table, leg, out);
}

// ---------------------------------------------------------------------------

void test_geometry_v5_data_is_what_the_oracle_states() {
  g_case = "geometry v5 data pin";
  CalibrationGeometryProfile profile = boundProfile();
  CHECK(profile.bound());
  for (Leg leg : kAllLegs) {
    const actuator::GeometryEndpointRecord* lo =
        profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MIN_SIDE);
    const actuator::GeometryEndpointRecord* hi =
        profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MAX_SIDE);
    CHECK(lo != nullptr && hi != nullptr);
    if (lo == nullptr || hi == nullptr) continue;
    CHECK_EQ(lo->contact, kUpperMinContact);
    CHECK_EQ(hi->contact, kUpperMaxContact);
    CHECK(actuator::isExecutable(*lo));
    CHECK(actuator::isExecutable(*hi));
    CHECK(!lo->has_auxiliary);
  }
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
    // The resolved identity is exactly what Geometry V5 itself knows.
    const actuator::GeometryJointRecord* record = profile.findJoint(ref.identity);
    CHECK(record != nullptr);
    if (record != nullptr) CHECK_EQ(record->bus_id, o.bus);
  }
}

void test_resolve_leg_joint_refusals() {
  g_case = "resolveLegJoint refusals";
  CalibrationGeometryProfile profile = boundProfile();
  FullLegJointRef ref{};
  CHECK(resolveLegJoint(profile, Leg::LF, JointKind::UPPER, nullptr) ==
        FullLegPlanStatus::REJECT_NULL_OUTPUT);
  CHECK(resolveLegJoint(profile, static_cast<Leg>(9), JointKind::UPPER, &ref) ==
        FullLegPlanStatus::REJECT_UNKNOWN_LEG);
  CHECK_EQ(ref.bus_id, 0);
  CHECK(resolveLegJoint(profile, Leg::LF, static_cast<JointKind>(9), &ref) ==
        FullLegPlanStatus::REJECT_UNKNOWN_LEG);

  CalibrationGeometryProfile unbound;
  CHECK(resolveLegJoint(unbound, Leg::LF, JointKind::UPPER, &ref) ==
        FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND);
  CHECK_EQ(ref.bus_id, 0);

  // Geometry V5 placing the joint on a different bus is a refusal, not a
  // silent preference for either side.
  Copy c;
  actuator::GeometryJointRecord* lf_upper = c.joint(Leg::LF, JointKind::UPPER);
  CHECK(lf_upper != nullptr);
  if (lf_upper == nullptr) return;
  lf_upper->bus_id = 99;
  c.bind();
  CHECK(resolveLegJoint(c.profile, Leg::LF, JointKind::UPPER, &ref) ==
        FullLegPlanStatus::REJECT_GEOMETRY_JOINT);
  CHECK_EQ(ref.bus_id, 0);
  // Other legs are unaffected.
  CHECK(resolveLegJoint(c.profile, Leg::RF, JointKind::UPPER, &ref) == FullLegPlanStatus::OK);

  // Geometry V5 knowing a different physical unit for the slot is a refusal.
  Copy d;
  actuator::GeometryJointRecord* rf_upper = d.joint(Leg::RF, JointKind::UPPER);
  CHECK(rf_upper != nullptr);
  if (rf_upper == nullptr) return;
  setPhysicalUnit(&rf_upper->identity, "WRONG");
  d.bind();
  CHECK(resolveLegJoint(d.profile, Leg::RF, JointKind::UPPER, &ref) ==
        FullLegPlanStatus::REJECT_GEOMETRY_JOINT);
}

void test_parking_matrix_is_exactly_the_current_geometry_v5_result() {
  g_case = "parking matrix";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);

  for (const ParkingOracle& p : kParking) {
    FullLegPlan plan{};
    CHECK(planOf(profile, table, p.leg, &plan) == FullLegPlanStatus::OK);
    CHECK(plan.leg == p.leg);
    CHECK(plan.request.auxiliary_required == p.aux_required);
    if (p.aux_required) {
      CHECK_EQ(plan.request.auxiliary_bus_id, p.aux_bus);
      CHECK(plan.request.auxiliary_joint.leg == p.aux_leg);
      CHECK(plan.request.auxiliary_joint.joint == JointKind::UPPER);
      CHECK(plan.request.auxiliary_joint.unitKnown());
      CHECK_EQ(plan.request.auxiliary_park_target_urad, kAuxiliaryParkTarget);
    } else {
      // No auxiliary at all: bus 0 and an empty identity, never a stale one.
      CHECK_EQ(plan.request.auxiliary_bus_id, 0);
      CHECK(!plan.request.auxiliary_joint.unitKnown());
      CHECK_EQ(plan.request.auxiliary_park_target_urad, 0);
    }
  }
}

void test_plan_carries_dynamic_identity_endpoints_and_tolerances() {
  g_case = "plan contents per leg";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);

  for (Leg leg : kAllLegs) {
    FullLegPlan plan{};
    CHECK(planOf(profile, table, leg, &plan) == FullLegPlanStatus::OK);

    const JointOracle& upper = oracleFor(leg, JointKind::UPPER);
    const JointOracle& hip = oracleFor(leg, JointKind::HIP);
    const JointOracle& lower = oracleFor(leg, JointKind::LOWER);
    CHECK_EQ(plan.upper.bus_id, upper.bus);
    CHECK_EQ(plan.hip.bus_id, hip.bus);
    CHECK_EQ(plan.lower.bus_id, lower.bus);
    CHECK(plan.upper.identity.joint == JointKind::UPPER);
    CHECK(plan.hip.identity.joint == JointKind::HIP);
    CHECK(plan.lower.identity.joint == JointKind::LOWER);

    const FullLegCalibrationRequest& r = plan.request;
    CHECK_EQ(r.probe_bus_id, upper.bus);
    CHECK(r.probe_joint.leg == leg);
    CHECK(r.probe_joint.joint == JointKind::UPPER);
    CHECK(std::strcmp(r.probe_joint.physical_unit, upper.unit) == 0);
    CHECK(r.endpoint_leg == leg);
    CHECK(r.endpoint_joint == JointKind::UPPER);
    CHECK_EQ(r.min_repeatability_tolerance_ticks, kFullLegRepeatabilityToleranceTicks);
    CHECK_EQ(r.max_repeatability_tolerance_ticks, kFullLegRepeatabilityToleranceTicks);

    // Endpoint numbers are read from the record, not typed.
    const actuator::GeometryEndpointRecord* lo =
        profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MIN_SIDE);
    const actuator::GeometryEndpointRecord* hi =
        profile.findEndpoint(leg, JointKind::UPPER, ContactSide::MAX_SIDE);
    CHECK(lo != nullptr && hi != nullptr);
    if (lo == nullptr || hi == nullptr) continue;
    CHECK_EQ(r.min_approach_urad, lo->contact);
    CHECK_EQ(r.max_approach_urad, hi->contact);
    CHECK_EQ(r.min_backoff_urad, lo->contact / 2);
    CHECK_EQ(r.max_backoff_urad, hi->contact / 2);
    // Hardware finding 2026-09-29: both approach passes may run up to 16 raw
    // ticks past the canonical contact, clamped to the URDF limit; the contact
    // numbers above stay the Geometry V5 ones and the backoff stays contact/2.
    CHECK_EQ(r.approach_overtravel_ticks, 16);
    CHECK_EQ(r.approach_overtravel_ticks, kFullLegApproachOvertravelTicks);
    CHECK(r.approach_overtravel_ticks <= actuator::kContactProbeMaxOvertravelTicks);

    // No LF-only residue: the old hard-coded MIN backoff must not survive.
    CHECK(r.min_backoff_urad != -700000);
  }
}

void test_plan_targets_pass_the_checked_resolver_for_every_leg() {
  g_case = "plan targets resolve";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);
  const actuator::GeometryProvenanceTag tag = profile.provenanceTag();

  for (Leg leg : kAllLegs) {
    FullLegPlan plan{};
    CHECK(planOf(profile, table, leg, &plan) == FullLegPlanStatus::OK);
    const actuator::JointTransform* t = table.find(plan.upper.identity, tag);
    CHECK(t != nullptr);
    if (t == nullptr) continue;
    const actuator::MicroRad targets[4] = {plan.request.min_approach_urad,
                                           plan.request.min_backoff_urad,
                                           plan.request.max_approach_urad,
                                           plan.request.max_backoff_urad};
    uint16_t raw[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
      CHECK(actuator::resolveUrdfQToRaw(profile, actuator::geometry_data::kProvenance, *t,
                                        targets[i], &raw[i]) ==
            actuator::TargetResolveStatus::OK);
    }
    // The backoff is a real re-approach: at least 8 tolerances from the contact.
    const int min_travel = kFullLegRepeatabilityToleranceTicks * kFullLegMinReapproachToleranceMultiple;
    CHECK(std::abs(static_cast<int>(raw[0]) - static_cast<int>(raw[1])) >= min_travel);
    CHECK(std::abs(static_cast<int>(raw[2]) - static_cast<int>(raw[3])) >= min_travel);
    // The commanded approach points the plan records: past each contact by
    // the URDF room (4 MIN / 6 MAX, see test_calibration_execution_engine.cpp),
    // not by the 16-tick ceiling; further from q0 than the contact; inside the
    // URDF domain with the next tick outside it; the URDF-limit tick within a
    // rounding tick of the target.
    const FullLegProbeBoundary* bounds[2] = {&plan.min_probe, &plan.max_probe};
    const int room[2] = {4, 6};
    for (int k = 0; k < 2; ++k) {
      const FullLegProbeBoundary& b = *bounds[k];
      const int contact = raw[k * 2];
      const int q0 = static_cast<int>(t->q0_tick);
      CHECK_EQ(b.contact_tick, contact);
      CHECK_EQ(b.applied_overtravel_ticks, room[k]);
      CHECK_EQ(std::abs(static_cast<int>(b.target_tick) - contact), room[k]);
      CHECK_EQ(std::abs(static_cast<int>(b.target_tick) - q0), std::abs(contact - q0) + room[k]);
      actuator::MicroRad q = 0;
      CHECK(actuator::resolveRawToUrdfQ(profile, actuator::geometry_data::kProvenance, *t,
                                        b.target_tick, &q) == actuator::TargetResolveStatus::OK);
      const int further = static_cast<int>(b.target_tick) + (static_cast<int>(b.target_tick) > contact ? 1 : -1);
      CHECK(actuator::resolveRawToUrdfQ(profile, actuator::geometry_data::kProvenance, *t,
                                        static_cast<uint16_t>(further), &q) ==
            actuator::TargetResolveStatus::REJECT_URDF_LIMIT);
      CHECK(std::abs(static_cast<int>(b.urdf_limit_tick) - static_cast<int>(b.target_tick)) <= 1);
    }
    if (plan.request.auxiliary_required) {
      const actuator::JointTransform* at = table.find(plan.request.auxiliary_joint, tag);
      CHECK(at != nullptr);
      uint16_t aux_raw = 0;
      if (at != nullptr) {
        CHECK(actuator::resolveUrdfQToRaw(profile, actuator::geometry_data::kProvenance, *at,
                                          plan.request.auxiliary_park_target_urad,
                                          &aux_raw) == actuator::TargetResolveStatus::OK);
      }
    }
  }
}

void test_backoff_rule_lies_inside_the_compiled_clear_region() {
  g_case = "backoff inside clear region";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);
  const actuator::GeometryProvenanceTag tag = profile.provenanceTag();
  for (Leg leg : kAllLegs) {
    const JointOracle& upper = oracleFor(leg, JointKind::UPPER);
    const actuator::JointTransform* t = table.find(identityOf(upper), tag);
    CHECK(t != nullptr);
    if (t == nullptr) continue;
    for (ContactSide side : {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE}) {
      const actuator::GeometryEndpointRecord* e =
          profile.findEndpoint(leg, JointKind::UPPER, side);
      CHECK(e != nullptr);
      if (e == nullptr) continue;
      actuator::MicroRad backoff = 0;
      CHECK(deriveBackoffUrad(profile, actuator::geometry_data::kProvenance, *t, *e,
                              kFullLegRepeatabilityToleranceTicks, &backoff) ==
            FullLegPlanStatus::OK);
      if (side == ContactSide::MAX_SIDE) {
        CHECK(backoff > 0 && backoff < e->contact && backoff <= e->clear);
      } else {
        CHECK(backoff < 0 && backoff > e->contact && backoff >= e->clear);
      }
    }
  }
}

void test_backoff_rule_rejects_unverifiable_points() {
  g_case = "backoff refusals";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  primeAll(table);
  const actuator::GeometryProvenanceTag tag = profile.provenanceTag();
  const JointOracle& lf = oracleFor(Leg::LF, JointKind::UPPER);
  const actuator::JointTransform* t = table.find(identityOf(lf), tag);
  CHECK(t != nullptr);
  if (t == nullptr) return;

  const actuator::GeometryEndpointRecord* real_max =
      profile.findEndpoint(Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE);
  CHECK(real_max != nullptr);
  if (real_max == nullptr) return;

  actuator::MicroRad out = 12345;
  const uint16_t tol = kFullLegRepeatabilityToleranceTicks;
  const actuator::GeometryProvenance& prov = actuator::geometry_data::kProvenance;

  CHECK(deriveBackoffUrad(profile, prov, *t, *real_max, tol, nullptr) ==
        FullLegPlanStatus::REJECT_NULL_OUTPUT);

  // A `clear` that is not on the contact's side, or zero, leaves no verified
  // clear region to back off into.
  actuator::GeometryEndpointRecord e = *real_max;
  e.clear = 0;
  CHECK(deriveBackoffUrad(profile, prov, *t, e, tol, &out) == FullLegPlanStatus::REJECT_BACKOFF);
  CHECK_EQ(out, 0);

  // A backoff beyond the compiled clear angle is outside the proven region.
  e = *real_max;
  e.clear = real_max->contact / 4;
  CHECK(deriveBackoffUrad(profile, prov, *t, e, tol, &out) == FullLegPlanStatus::REJECT_BACKOFF);

  // A contact on the wrong side of q=0 for the endpoint's side.
  e = *real_max;
  e.contact = -real_max->contact;
  CHECK(deriveBackoffUrad(profile, prov, *t, e, tol, &out) == FullLegPlanStatus::REJECT_BACKOFF);

  // A contact so close to q=0 that the re-approach travels less than 8 tolerances.
  e = *real_max;
  e.contact = 100000;  // ~65 ticks, half is ~32 ticks of travel
  e.clear = 100000;
  CHECK(deriveBackoffUrad(profile, prov, *t, e, tol, &out) == FullLegPlanStatus::REJECT_BACKOFF);

  // Zero tolerance can never verify a re-approach.
  CHECK(deriveBackoffUrad(profile, prov, *t, *real_max, 0, &out) ==
        FullLegPlanStatus::REJECT_BACKOFF);

  // A contact past the URDF limit is refused by the checked resolver.
  e = *real_max;
  e.contact = 2200000;
  e.clear = 2100000;
  CHECK(deriveBackoffUrad(profile, prov, *t, e, tol, &out) ==
        FullLegPlanStatus::REJECT_TARGET_RESOLUTION);

  // A transform for another geometry cannot resolve anything.
  actuator::JointTransform stale = *t;
  stale.geometry = tag + 1;
  CHECK(deriveBackoffUrad(profile, prov, stale, *real_max, tol, &out) ==
        FullLegPlanStatus::REJECT_TARGET_RESOLUTION);
}

void test_plan_refuses_when_q0_would_push_a_target_out_of_the_raw_domain() {
  g_case = "raw domain";
  CalibrationGeometryProfile profile = boundProfile();
  FullLegPlan plan{};
  // q0 so close to raw 0 that the MIN contact (~593 ticks) would wrap: the
  // checked resolver refuses rather than wrapping, so the plan refuses too.
  actuator::JointTransformTable low;
  primeAll(low, 100);
  CHECK(planOf(profile, low, Leg::LF, &plan) == FullLegPlanStatus::REJECT_TARGET_RESOLUTION);
  CHECK_EQ(plan.request.probe_bus_id, 0);
}

void test_plan_requires_current_transforms() {
  g_case = "missing / stale transforms";
  CalibrationGeometryProfile profile = boundProfile();
  FullLegPlan plan{};

  actuator::JointTransformTable empty;
  for (Leg leg : kAllLegs) {
    CHECK(planOf(profile, empty, leg, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
    CHECK_EQ(plan.request.probe_bus_id, 0);
  }

  // The probed joint's transform alone is enough for a no-aux leg ...
  actuator::JointTransformTable only_upper;
  CHECK(only_upper.admit(promotedTransform(identityOf(oracleFor(Leg::RH, JointKind::UPPER)))));
  CHECK(planOf(profile, only_upper, Leg::RH, &plan) == FullLegPlanStatus::OK);

  // ... but LF also needs LH_UPPER's, and RF needs RH_UPPER's.
  actuator::JointTransformTable lf_only;
  CHECK(lf_only.admit(promotedTransform(identityOf(oracleFor(Leg::LF, JointKind::UPPER)))));
  CHECK(planOf(profile, lf_only, Leg::LF, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
  CHECK(lf_only.admit(promotedTransform(identityOf(oracleFor(Leg::LH, JointKind::UPPER)))));
  CHECK(planOf(profile, lf_only, Leg::LF, &plan) == FullLegPlanStatus::OK);

  actuator::JointTransformTable rf_only;
  CHECK(rf_only.admit(promotedTransform(identityOf(oracleFor(Leg::RF, JointKind::UPPER)))));
  CHECK(planOf(profile, rf_only, Leg::RF, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
  CHECK(rf_only.admit(promotedTransform(identityOf(oracleFor(Leg::RH, JointKind::UPPER)))));
  CHECK(planOf(profile, rf_only, Leg::RF, &plan) == FullLegPlanStatus::OK);

  // A transform measured under another geometry is not current evidence.
  actuator::JointTransformTable stale;
  actuator::JointTransform t = promotedTransform(identityOf(oracleFor(Leg::RH, JointKind::UPPER)));
  t.geometry = t.geometry + 1;
  CHECK(stale.admit(t));
  CHECK(planOf(profile, stale, Leg::RH, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
}

void test_plan_refuses_inconsistent_geometry_endpoint_records() {
  g_case = "inconsistent endpoint records";
  actuator::JointTransformTable table;
  primeAll(table);
  FullLegPlan plan{};

  {  // MAX outcome says a plan is needed but names no auxiliary
    Copy c;
    c.endpoint(Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE)->has_auxiliary = false;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::LF, &plan) ==
          FullLegPlanStatus::REJECT_MAX_PLAN_INCONSISTENT);
  }
  {  // MAX outcome says NOT_NEEDED but names an auxiliary
    Copy c;
    c.endpoint(Leg::RH, JointKind::UPPER, ContactSide::MAX_SIDE)->has_auxiliary = true;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::RH, &plan) ==
          FullLegPlanStatus::REJECT_MAX_PLAN_INCONSISTENT);
  }
  {  // MIN side needing an auxiliary is not something the executor can run
    Copy c;
    actuator::GeometryEndpointRecord* lo = c.endpoint(Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
    lo->has_auxiliary = true;
    lo->parking = actuator::ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::LF, &plan) ==
          FullLegPlanStatus::REJECT_MIN_NEEDS_AUXILIARY);
  }
  {  // A diagnostic (outside-URDF) endpoint is never a motion target
    Copy c;
    c.endpoint(Leg::LH, JointKind::UPPER, ContactSide::MAX_SIDE)->domain =
        actuator::TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::LH, &plan) ==
          FullLegPlanStatus::REJECT_ENDPOINT_NOT_EXECUTABLE);
  }
  {  // Clearance not PASS
    Copy c;
    c.endpoint(Leg::RF, JointKind::UPPER, ContactSide::MIN_SIDE)->clearance =
        actuator::ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::RF, &plan) ==
          FullLegPlanStatus::REJECT_ENDPOINT_NOT_EXECUTABLE);
  }
  {  // The auxiliary may not be the probed joint itself
    Copy c;
    actuator::GeometryEndpointRecord* hi = c.endpoint(Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE);
    hi->auxiliary_leg = Leg::LF;
    hi->auxiliary_joint = JointKind::UPPER;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::LF, &plan) ==
          FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY);
  }
  {  // The named auxiliary must resolve to an installed canonical joint
    Copy c;
    c.endpoint(Leg::RF, JointKind::UPPER, ContactSide::MAX_SIDE)->auxiliary_joint =
        static_cast<JointKind>(9);
    c.bind();
    CHECK(planOf(c.profile, table, Leg::RF, &plan) ==
          FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY);
  }
  {  // The requested leg's endpoint is absent from the bundle
    Copy c;
    c.endpoint(Leg::RH, JointKind::UPPER, ContactSide::MAX_SIDE)->leg = Leg::LF;  // now a dup of LF's
    c.endpoint(Leg::RH, JointKind::UPPER, ContactSide::MIN_SIDE)->joint = JointKind::HIP;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::RH, &plan) == FullLegPlanStatus::REJECT_ENDPOINT_MISSING);
  }
  {  // Geometry V5 disagreeing with the canonical bus for the joint
    Copy c;
    c.joint(Leg::LH, JointKind::UPPER)->bus_id = 41;
    c.bind();
    CHECK(planOf(c.profile, table, Leg::LH, &plan) == FullLegPlanStatus::REJECT_GEOMETRY_JOINT);
    // ... and it also refuses LF, whose auxiliary is exactly that joint.
    CHECK(planOf(c.profile, table, Leg::LF, &plan) ==
          FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY);
  }
}

void test_plan_refuses_unbound_or_mismatched_provenance_and_null_output() {
  g_case = "plan refusals";
  actuator::JointTransformTable table;
  primeAll(table);
  FullLegPlan plan{};

  CalibrationGeometryProfile unbound;
  CHECK(planOf(unbound, table, Leg::LF, &plan) == FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND);

  CalibrationGeometryProfile profile = boundProfile();
  CHECK(resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, table, Leg::LF,
                           nullptr) == FullLegPlanStatus::REJECT_NULL_OUTPUT);
  CHECK(planOf(profile, table, static_cast<Leg>(9), &plan) ==
        FullLegPlanStatus::REJECT_UNKNOWN_LEG);

  actuator::GeometryProvenance other = actuator::geometry_data::kProvenance;
  other.urdf_sha256[0] = (other.urdf_sha256[0] == 'a') ? 'b' : 'a';
  CHECK(resolveFullLegPlan(profile, other, table, Leg::LF, &plan) ==
        FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND);
  CHECK_EQ(plan.request.probe_bus_id, 0);
}

void test_refused_plan_leaves_no_partial_request() {
  g_case = "no partial request on refusal";
  CalibrationGeometryProfile profile = boundProfile();
  actuator::JointTransformTable table;
  FullLegPlan plan{};
  plan.request.probe_bus_id = 77;  // stale garbage from an earlier plan
  plan.request.auxiliary_required = false;
  CHECK(planOf(profile, table, Leg::LF, &plan) == FullLegPlanStatus::REJECT_NO_TRANSFORM);
  CHECK_EQ(plan.request.probe_bus_id, 0);
  // Fail-closed default: an unresolved request still claims an auxiliary is required.
  CHECK(plan.request.auxiliary_required);
}

void test_status_names_are_unique_and_stable() {
  g_case = "status names";
  const FullLegPlanStatus all[] = {
      FullLegPlanStatus::OK,
      FullLegPlanStatus::REJECT_NULL_OUTPUT,
      FullLegPlanStatus::REJECT_UNKNOWN_LEG,
      FullLegPlanStatus::REJECT_GEOMETRY_UNBOUND,
      FullLegPlanStatus::REJECT_CANONICAL_IDENTITY,
      FullLegPlanStatus::REJECT_GEOMETRY_JOINT,
      FullLegPlanStatus::REJECT_ENDPOINT_MISSING,
      FullLegPlanStatus::REJECT_ENDPOINT_NOT_EXECUTABLE,
      FullLegPlanStatus::REJECT_MIN_NEEDS_AUXILIARY,
      FullLegPlanStatus::REJECT_MAX_PLAN_INCONSISTENT,
      FullLegPlanStatus::REJECT_AUXILIARY_IDENTITY,
      FullLegPlanStatus::REJECT_NO_TRANSFORM,
      FullLegPlanStatus::REJECT_BACKOFF,
      FullLegPlanStatus::REJECT_TARGET_RESOLUTION,
  };
  const int n = static_cast<int>(sizeof(all) / sizeof(all[0]));
  for (int i = 0; i < n; ++i) {
    CHECK(std::strcmp(toString(all[i]), "UNKNOWN") != 0);
    for (int j = i + 1; j < n; ++j) CHECK(std::strcmp(toString(all[i]), toString(all[j])) != 0);
  }
  CHECK(std::strcmp(toString(static_cast<FullLegPlanStatus>(200)), "UNKNOWN") == 0);
}

}  // namespace

int main() {
  test_geometry_v5_data_is_what_the_oracle_states();
  test_canonical_identity_and_bus_for_all_twelve_joints();
  test_resolve_leg_joint_refusals();
  test_parking_matrix_is_exactly_the_current_geometry_v5_result();
  test_plan_carries_dynamic_identity_endpoints_and_tolerances();
  test_plan_targets_pass_the_checked_resolver_for_every_leg();
  test_backoff_rule_lies_inside_the_compiled_clear_region();
  test_backoff_rule_rejects_unverifiable_points();
  test_plan_refuses_when_q0_would_push_a_target_out_of_the_raw_domain();
  test_plan_requires_current_transforms();
  test_plan_refuses_inconsistent_geometry_endpoint_records();
  test_plan_refuses_unbound_or_mismatched_provenance_and_null_output();
  test_refused_plan_leaves_no_partial_request();
  test_status_names_are_unique_and_stable();

  std::printf("test_full_leg_calibration_plan: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
