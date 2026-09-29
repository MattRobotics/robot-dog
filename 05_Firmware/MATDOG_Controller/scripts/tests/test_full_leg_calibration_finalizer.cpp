// Offline tests for the Full Leg evidence lifecycle
// (src/calibration/FullLegCalibrationFinalizer.*) and the operational-limit
// gate it uses (SafeActuatorPolicy::validate/admitOperationalLimit).
//
// The real CalibrationManager, ActuatorAuthorityArbiter, CalibrationMotionPermit,
// SafeActuatorPolicy tables, Geometry V5 profile and envelope builders are
// linked; only the executor's OUTCOME is synthesized (FullLegRunOutcome), so
// this suite proves what happens AFTER two contacts were measured. It measured
// nothing itself: every tick below is a Geometry V5 contact angle pushed
// through the checked URDF-q -> raw resolver.
//
// NO HARDWARE VALIDATION.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
#include "../../src/calibration/FullLegCalibrationFinalizer.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::actuator::CalibrationGeometryProfile;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::OperatingMode;
using matdog::core::SystemHealth;

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

#define CHECK_STR(actual, expected)                                                    \
  do {                                                                                 \
    ++g_checks;                                                                        \
    if (std::strcmp((actual), (expected)) != 0) {                                      \
      ++g_failures;                                                                    \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case,        \
                  __FILE__, __LINE__, #actual, (actual), (expected));                  \
    }                                                                                  \
  } while (0)

namespace {

constexpr CalibrationOrigin kLive = CalibrationOrigin::LIVE_SESSION;
constexpr uint16_t kWitnessBand = 16;

struct JointOracle {
  Leg leg;
  JointKind joint;
  uint8_t bus;
  const char* unit;
  uint16_t q0;  // the CR2-C q0 of the hardware runbook
};

// config/MATDOG_SERVO_ALLOCATION.yaml + the CR2-C q0 cross-check values.
constexpr JointOracle kOracle[12] = {
    {Leg::LF, JointKind::LOWER, 11, "M33", 2087},   {Leg::LF, JointKind::UPPER, 12, "ELR01", 2100},
    {Leg::LF, JointKind::HIP, 13, "M22", 1996},     {Leg::RF, JointKind::LOWER, 21, "NEW03", 1985},
    {Leg::RF, JointKind::UPPER, 22, "ELR03", 2092}, {Leg::RF, JointKind::HIP, 23, "NEW01", 2030},
    {Leg::RH, JointKind::LOWER, 31, "NEW05", 2034}, {Leg::RH, JointKind::UPPER, 32, "ELR02", 2042},
    {Leg::RH, JointKind::HIP, 33, "NEW06", 2081},   {Leg::LH, JointKind::LOWER, 41, "M41", 2073},
    {Leg::LH, JointKind::UPPER, 42, "M42", 2089},   {Leg::LH, JointKind::HIP, 43, "M43", 2035},
};

constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};

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

const ParkingOracle& parkingFor(Leg leg) {
  for (const ParkingOracle& p : kParking) {
    if (p.leg == leg) return p;
  }
  return kParking[0];
}

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

actuator::JointTransform promotedTransform(const JointOracle& o) {
  actuator::JointTransform t{};
  t.identity = identityOf(o);
  t.geometry = actuator::geometryProvenanceTag(actuator::geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = kLive;
  t.q0_tick = o.q0;
  t.present = true;
  return t;
}

ContactEvidence contactAt(Leg leg, ContactSide side, uint16_t tick) {
  ContactEvidence e{};
  e.key.leg = leg;
  e.key.joint = JointKind::UPPER;
  e.key.side = side;
  e.state = EvidenceState::PROMOTED;
  e.origin = kLive;
  e.detection = ContactState::CONTACT_CONFIRMED;
  e.witness = makeContactWitness(0, 0, kWitnessBand);
  e.coarse_tick = tick;
  e.fine_tick_1 = tick;
  e.fine_tick_2 = tick;
  e.repeatability_ticks = 0;
  e.has_measurement = true;
  return e;
}

LegPopulationEvidence fullPopulation() {
  LegPopulationEvidence e{};
  e.evaluated = true;
  e.origin = kLive;
  e.observed_mask = (1u << kLegServoSlotCount) - 1u;
  e.session_ms = 42;
  return e;
}

struct Rig {
  ActuatorAuthorityArbiter arbiter;
  CalibrationManager manager;
  actuator::SafeActuatorPolicy policy;
  CalibrationGeometryProfile profile;
  CalibrationMotionPermit permit;
  CalibrationMotionAuthorizationState authorization;
  FullLegFinalizeContext context;

  explicit Rig(bool approved = false) {
    arbiter.reset(AuthorityClearReason::BOOT);
    manager.begin(&arbiter);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &actuator::geometry_data::kProvenance);
    for (const JointOracle& o : kOracle) CHECK(policy.transforms().admit(promotedTransform(o)));

    context.manager = &manager;
    context.policy = &policy;
    context.geometry = &profile;
    context.expected_provenance = &actuator::geometry_data::kProvenance;
    context.permit = &permit;
    context.authorization = &authorization;
    context.arbiter = &arbiter;
    context.parameters = productionEnvelopeParameters();
    context.parameters.approved = approved;
  }

  // SESSION START <leg> + PERMIT GRANT, exactly the state a run starts under.
  bool startLeg(Leg leg) {
    if (manager.startSession(leg, OperatingMode::MAINTENANCE, kLive) != SessionResult::STARTED) {
      return false;
    }
    if (!manager.submitPopulationEvidence(fullPopulation())) return false;
    if (manager.activate() != SessionResult::OK) return false;

    CalibrationMotionPermitFacts facts{};
    facts.explicit_operator_authorization = true;
    facts.robot_powered_profile = true;
    facts.mode = OperatingMode::MAINTENANCE;
    facts.system_health = SystemHealth::READY;
    facts.session_active = true;
    facts.origin = kLive;
    facts.session_id = manager.status().session_id;
    facts.current_population_pass = true;
    facts.current_geometry_bound = true;
    facts.promoted_transforms_complete = true;
    facts.authority = ActuatorAuthority::CALIBRATION;
    facts.authority_generation = arbiter.generation();
    facts.authority_inhibited = false;
    CalibrationMotionPermitToken token{};
    if (permit.grant(facts, &token) != CalibrationPermitStatus::ACTIVE) return false;
    authorization.operator_authorized = true;
    authorization.token = token;
    return true;
  }

  FullLegPlan planFor(Leg leg) {
    FullLegPlan plan{};
    CHECK(resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, policy.transforms(),
                             leg, &plan) == FullLegPlanStatus::OK);
    return plan;
  }

  uint16_t tickOf(Leg leg, ContactSide side) {
    const actuator::GeometryEndpointRecord* endpoint =
        profile.findEndpoint(leg, JointKind::UPPER, side);
    const actuator::JointTransform* transform = policy.transforms().find(
        identityOf(oracleFor(leg, JointKind::UPPER)), policy.currentGeometryTag());
    uint16_t tick = 0;
    CHECK(endpoint != nullptr && transform != nullptr);
    if (endpoint == nullptr || transform == nullptr) return 0;
    CHECK(actuator::resolveUrdfQToRaw(profile, actuator::geometry_data::kProvenance, *transform,
                                      endpoint->contact, &tick) ==
          actuator::TargetResolveStatus::OK);
    return tick;
  }

  FullLegRunOutcome goodOutcome(Leg leg) {
    FullLegRunOutcome outcome{};
    outcome.terminal = true;
    outcome.complete = true;
    outcome.failure = FullLegCalibrationFailure::NONE;
    outcome.min_contact = contactAt(leg, ContactSide::MIN_SIDE, tickOf(leg, ContactSide::MIN_SIDE));
    outcome.max_contact = contactAt(leg, ContactSide::MAX_SIDE, tickOf(leg, ContactSide::MAX_SIDE));
    outcome.geometry_at_start = policy.currentGeometryTag();
    outcome.session_id_at_start = manager.status().session_id;
    return outcome;
  }

  void expectCleanBetweenLegs() {
    CHECK(!manager.sessionLive());
    CHECK(!manager.status().holds_authority);
    CHECK(arbiter.current() == ActuatorAuthority::NONE);
    CHECK(!permit.active());
    CHECK(!authorization.operator_authorized);
    CHECK(!authorization.token.valid());
  }
};

struct Lines {
  std::vector<std::string> v;
};

void collect(void* user, const char* line) { static_cast<Lines*>(user)->v.push_back(line); }

bool has(const Lines& lines, const std::string& needle) {
  for (const std::string& l : lines.v) {
    if (l.find(needle) != std::string::npos) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------

void test_production_parameters_are_placeholders_and_unapproved() {
  g_case = "production parameters";
  const FullLegEnvelopeParameters p = productionEnvelopeParameters();
  CHECK(!p.approved);
  CHECK(!kFullLegOperationalParametersApproved);
  CHECK_EQ(p.upper_margin_ticks, 8);
  CHECK_EQ(p.hip_lower_margin_urad, 50000);
  CHECK_EQ(kFullLegPlaceholderUpperMarginTicks, 8);
  CHECK_EQ(kFullLegPlaceholderHipLowerMarginUrad, 50000);
}

void test_policy_operational_limit_gate() {
  g_case = "policy operational limit gate";
  Rig rig;
  const actuator::GeometryProvenanceTag current = rig.policy.currentGeometryTag();
  const JointIdentity id = identityOf(oracleFor(Leg::LF, JointKind::UPPER));

  actuator::JointLimit good{};
  good.identity = id;
  good.state = EvidenceState::PROMOTED;
  good.origin = kLive;
  good.geometry = current;
  good.min_tick = 600;
  good.max_tick = 3400;
  good.present = true;

  CHECK(rig.policy.validateOperationalLimit(good) == actuator::LimitAdmission::ADMITTED);
  // validate has no side effect.
  CHECK(rig.policy.limits().find(id, current) == nullptr);

  actuator::JointLimit replay = good;
  replay.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  CHECK(rig.policy.admitOperationalLimit(replay) == actuator::LimitAdmission::REJECT_NOT_OPERATIONAL);
  actuator::JointLimit unpromoted = good;
  unpromoted.state = EvidenceState::ACCEPTED;
  CHECK(rig.policy.admitOperationalLimit(unpromoted) ==
        actuator::LimitAdmission::REJECT_NOT_OPERATIONAL);
  actuator::JointLimit absent = good;
  absent.present = false;
  CHECK(rig.policy.admitOperationalLimit(absent) == actuator::LimitAdmission::REJECT_NOT_OPERATIONAL);

  actuator::JointLimit too_high = good;
  too_high.max_tick = 4096;
  CHECK(rig.policy.admitOperationalLimit(too_high) ==
        actuator::LimitAdmission::REJECT_TICK_OUT_OF_RANGE);

  actuator::JointLimit stale = good;
  stale.geometry = current ^ 0x1;
  CHECK(rig.policy.admitOperationalLimit(stale) ==
        actuator::LimitAdmission::REJECT_GEOMETRY_NOT_CURRENT);
  actuator::JointLimit untagged = good;
  untagged.geometry = actuator::kNoGeometryProvenance;
  CHECK(rig.policy.admitOperationalLimit(untagged) ==
        actuator::LimitAdmission::REJECT_GEOMETRY_NOT_CURRENT);
  actuator::JointLimit inverted = good;
  inverted.min_tick = 3400;
  inverted.max_tick = 600;
  CHECK(rig.policy.admitOperationalLimit(inverted) ==
        actuator::LimitAdmission::REJECT_NOT_OPERATIONAL);
  actuator::JointLimit no_unit = good;
  no_unit.identity = JointIdentity{};
  no_unit.identity.leg = Leg::LF;
  no_unit.identity.joint = JointKind::UPPER;
  CHECK(rig.policy.admitOperationalLimit(no_unit) ==
        actuator::LimitAdmission::REJECT_NOT_OPERATIONAL);

  // No current transform for the joint: no limit, whatever the numbers say.
  Rig bare;
  bare.policy.transforms().clear();
  CHECK(bare.policy.admitOperationalLimit(good) == actuator::LimitAdmission::REJECT_NO_TRANSFORM);
  CHECK(bare.policy.limits().find(id, current) == nullptr);

  // Nothing stored by any refusal above.
  CHECK(rig.policy.limits().find(id, current) == nullptr);

  // The one good limit lands and is readable under the CURRENT tag only.
  CHECK(rig.policy.admitOperationalLimit(good) == actuator::LimitAdmission::ADMITTED);
  const actuator::JointLimit* found = rig.policy.limits().find(id, current);
  CHECK(found != nullptr);
  if (found != nullptr) {
    CHECK_EQ(found->min_tick, 600);
    CHECK_EQ(found->max_tick, 3400);
  }
  CHECK(rig.policy.limits().find(id, current ^ 0x1) == nullptr);
}

void test_unapproved_lifecycle_for_every_leg() {
  g_case = "unapproved lifecycle, 4 legs";
  for (Leg leg : kAllLegs) {
    Rig rig;
    CHECK(rig.startLeg(leg));
    const FullLegPlan plan = rig.planFor(leg);
    const FullLegRunOutcome outcome = rig.goodOutcome(leg);
    const uint32_t session_id = rig.manager.status().session_id;

    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::NONE);

    // The two contacts landed in the session exactly once each.
    CHECK_EQ(rig.manager.status().contacts_recorded, 2);
    CHECK(record.min_recorded);
    CHECK(record.max_recorded);
    CHECK(rig.manager.status().state == SessionState::COMPLETED);
    CHECK(record.session_completed);

    // Cleanup.
    rig.expectCleanBetweenLegs();
    CHECK(record.permit_revoked);
    CHECK(record.authority_released);

    // Level 1 only: contact calibrated, envelope NOT accepted, no limit stored.
    CHECK(record.hardware_contact_calibrated);
    CHECK(!record.operational_envelope_accepted);
    CHECK(record.verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
    CHECK(record.failure == FullLegFinalizeFailure::NONE);
    CHECK(!record.parameters_approved);
    for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
      const JointIdentity id = identityOf(oracleFor(leg, kind));
      CHECK(rig.policy.limits().find(id, rig.policy.currentGeometryTag()) == nullptr);
      CHECK(record.joint(kind).limit == FullLegLimitAdmission::NOT_ADMITTED_UNAPPROVED_PARAMETERS);
    }

    // Identity, bus, q0 and provenance in the record.
    CHECK(record.leg == leg);
    CHECK_EQ(record.session_id, session_id);
    CHECK(record.geometry == rig.policy.currentGeometryTag());
    for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
      const JointOracle& o = oracleFor(leg, kind);
      const FullLegJointRecord& j = record.joint(kind);
      CHECK_EQ(j.bus_id, o.bus);
      CHECK(std::strcmp(j.identity.physical_unit, o.unit) == 0);
      CHECK(j.q0_present);
      CHECK_EQ(j.q0_tick, o.q0);
      CHECK(j.q0_state == EvidenceState::PROMOTED);
      CHECK(j.q0_origin == kLive);
      CHECK(j.q0_geometry == rig.policy.currentGeometryTag());
    }

    // UPPER envelope: READY, derived from the two contacts, inset by 8 ticks.
    const FullLegJointRecord& upper = record.joint(JointKind::UPPER);
    CHECK(upper.envelope_status == actuator::EnvelopeBuildStatus::READY);
    CHECK(upper.envelope.source == actuator::EnvelopeSource::MEASURED_CONTACT);
    const uint16_t a = outcome.min_contact.fine_tick_1;
    const uint16_t b = outcome.max_contact.fine_tick_1;
    const uint16_t lo = a < b ? a : b;
    const uint16_t hi = a < b ? b : a;
    CHECK_EQ(upper.envelope.min_tick, lo + 8);
    CHECK_EQ(upper.envelope.max_tick, hi - 8);
    CHECK(upper.envelope.geometry == rig.policy.currentGeometryTag());

    // Parking metadata straight from the plan.
    const ParkingOracle& park = parkingFor(leg);
    CHECK(record.auxiliary_required == park.aux_required);
    if (park.aux_required) {
      CHECK(record.auxiliary_leg == park.aux_leg);
      CHECK(record.auxiliary_joint == JointKind::UPPER);
      CHECK_EQ(record.auxiliary_bus_id, park.aux_bus);
      CHECK_EQ(record.auxiliary_park_target_urad, 610865);
    } else {
      CHECK_EQ(record.auxiliary_bus_id, 0);
    }
  }
}

void test_approved_lifecycle_admits_current_limits_for_every_leg() {
  g_case = "approved lifecycle, 4 legs";
  for (Leg leg : kAllLegs) {
    Rig rig(/*approved=*/true);
    CHECK(rig.startLeg(leg));
    const FullLegPlan plan = rig.planFor(leg);
    const FullLegRunOutcome outcome = rig.goodOutcome(leg);

    FullLegRecord record{};
    const FullLegFinalizeFailure failure = finalizeFullLeg(rig.context, plan, outcome, &record);
    if (failure != FullLegFinalizeFailure::NONE) {
      std::printf("  approved run leg=%s failure=%s\n", toString(leg), toString(failure));
      for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
        std::printf("    %s envelope=%s limit=%s\n", toString(kind),
                    actuator::toString(record.joint(kind).envelope_status),
                    toString(record.joint(kind).limit));
      }
    }
    CHECK(failure == FullLegFinalizeFailure::NONE);
    CHECK(record.verdict == FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED);
    CHECK(record.hardware_contact_calibrated);
    CHECK(record.operational_envelope_accepted);
    CHECK(record.parameters_approved);
    rig.expectCleanBetweenLegs();

    for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
      const JointIdentity id = identityOf(oracleFor(leg, kind));
      const FullLegJointRecord& j = record.joint(kind);
      CHECK(j.envelope_status == actuator::EnvelopeBuildStatus::READY);
      CHECK(j.limit == FullLegLimitAdmission::ADMITTED);
      const actuator::JointLimit* found =
          rig.policy.limits().find(id, rig.policy.currentGeometryTag());
      CHECK(found != nullptr);
      if (found == nullptr) continue;
      CHECK(found->present);
      CHECK(found->state == EvidenceState::PROMOTED);
      CHECK(found->origin == kLive);
      CHECK(found->geometry == rig.policy.currentGeometryTag());
      CHECK_EQ(found->min_tick, j.envelope.min_tick);
      CHECK_EQ(found->max_tick, j.envelope.max_tick);
      CHECK(found->min_tick <= found->max_tick);
      // Not readable under a foreign tag.
      CHECK(rig.policy.limits().find(id, rig.policy.currentGeometryTag() ^ 0x1) == nullptr);
    }
    // HIP and LOWER come from the URDF domain, not from contacts.
    CHECK(record.joint(JointKind::HIP).envelope.source ==
          actuator::EnvelopeSource::DERIVED_FROM_GEOMETRY);
    CHECK(record.joint(JointKind::LOWER).envelope.source ==
          actuator::EnvelopeSource::DERIVED_FROM_GEOMETRY);
  }
}

void test_run_not_terminal_touches_nothing() {
  g_case = "run not terminal";
  Rig rig;
  CHECK(rig.startLeg(Leg::LF));
  const FullLegPlan plan = rig.planFor(Leg::LF);
  FullLegRunOutcome outcome = rig.goodOutcome(Leg::LF);
  outcome.terminal = false;
  outcome.complete = false;

  FullLegRecord record{};
  CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
        FullLegFinalizeFailure::RUN_NOT_TERMINAL);
  CHECK(record.failure == FullLegFinalizeFailure::RUN_NOT_TERMINAL);
  CHECK(!record.present);  // never committed by the store
  CHECK(record.verdict == FullLegVerdict::NOT_RUN);

  // Mid-run: session, permit and authority are exactly as they were.
  CHECK(rig.manager.status().state == SessionState::ACTIVE);
  CHECK_EQ(rig.manager.status().contacts_recorded, 0);
  CHECK(rig.permit.active());
  CHECK(rig.authorization.operator_authorized);
  CHECK(rig.arbiter.current() == ActuatorAuthority::CALIBRATION);

  FullLegEvidenceStore store;
  store.commit(record);
  CHECK_EQ(store.legsPresent(), 0);
}

void test_executor_failure_cleans_up_without_evidence() {
  g_case = "executor failure";
  {
    Rig rig;
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan plan = rig.planFor(Leg::RF);
    FullLegRunOutcome outcome{};
    outcome.terminal = true;
    outcome.complete = false;
    outcome.failure = FullLegCalibrationFailure::AUX_MOVE_TIMEOUT;
    outcome.geometry_at_start = rig.policy.currentGeometryTag();
    outcome.session_id_at_start = rig.manager.status().session_id;

    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::EXECUTOR_FAILED);
    CHECK(record.verdict == FullLegVerdict::FAILED);
    CHECK(record.executor_failure == FullLegCalibrationFailure::AUX_MOVE_TIMEOUT);
    CHECK(!record.hardware_contact_calibrated);
    CHECK(!record.operational_envelope_accepted);
    CHECK(!record.min_recorded && !record.max_recorded);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
    CHECK(rig.manager.status().state == SessionState::FAILED);
    rig.expectCleanBetweenLegs();
    CHECK(record.permit_revoked && record.authority_released);
    CHECK(!record.session_completed);
    CHECK(record.joint(JointKind::UPPER).limit == FullLegLimitAdmission::NOT_EVALUATED);
  }
  {
    // An operator abort ends the session ABORTED, not FAILED.
    Rig rig;
    CHECK(rig.startLeg(Leg::LH));
    const FullLegPlan plan = rig.planFor(Leg::LH);
    FullLegRunOutcome outcome{};
    outcome.terminal = true;
    outcome.failure = FullLegCalibrationFailure::OPERATOR_ABORT;
    outcome.geometry_at_start = rig.policy.currentGeometryTag();
    outcome.session_id_at_start = rig.manager.status().session_id;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::EXECUTOR_FAILED);
    CHECK(rig.manager.status().state == SessionState::ABORTED);
    rig.expectCleanBetweenLegs();
  }
  {
    // A complete-looking record with a failed executor phase never passes.
    Rig rig;
    CHECK(rig.startLeg(Leg::RH));
    const FullLegPlan plan = rig.planFor(Leg::RH);
    FullLegRunOutcome outcome = rig.goodOutcome(Leg::RH);
    outcome.complete = false;
    outcome.failure = FullLegCalibrationFailure::UPPER_MAX_PROBE_FAILED;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::EXECUTOR_FAILED);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
    CHECK(record.verdict == FullLegVerdict::FAILED);
  }
}

void test_session_mismatch_never_touches_a_foreign_session() {
  g_case = "session mismatch";
  {
    // The live session is RF's; the finished run was LF's.
    Rig rig;
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan lf_plan = rig.planFor(Leg::LF);
    FullLegRunOutcome outcome = rig.goodOutcome(Leg::LF);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, lf_plan, outcome, &record) ==
          FullLegFinalizeFailure::SESSION_MISMATCH);
    CHECK(record.verdict == FullLegVerdict::FAILED);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
    // The foreign session is neither completed, failed nor aborted.
    CHECK(rig.manager.status().state == SessionState::ACTIVE);
    CHECK(rig.arbiter.current() == ActuatorAuthority::CALIBRATION);
    // The permit still goes: nothing may keep moving on a run that ended.
    CHECK(!rig.permit.active());
    CHECK(!rig.authorization.operator_authorized);
    CHECK(record.permit_revoked);
    CHECK(!record.authority_released);
    CHECK(!record.session_completed);
    // Ownership of the foreign session stays with its owner.
    rig.manager.abortSession();
  }
  {
    // Same leg, different session id (a newer session replaced the run's).
    Rig rig;
    CHECK(rig.startLeg(Leg::LF));
    const FullLegPlan plan = rig.planFor(Leg::LF);
    FullLegRunOutcome outcome = rig.goodOutcome(Leg::LF);
    outcome.session_id_at_start += 1;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::SESSION_MISMATCH);
    CHECK(rig.manager.status().state == SessionState::ACTIVE);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
    rig.manager.abortSession();
  }
  {
    // The run's own session ended under it (operator SESSION ABORT mid-run).
    Rig rig;
    CHECK(rig.startLeg(Leg::LH));
    const FullLegPlan plan = rig.planFor(Leg::LH);
    const FullLegRunOutcome outcome = rig.goodOutcome(Leg::LH);
    rig.manager.abortSession();
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::SESSION_NOT_ACTIVE);
    CHECK(record.verdict == FullLegVerdict::FAILED);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
    CHECK(rig.manager.status().state == SessionState::ABORTED);
    rig.expectCleanBetweenLegs();
    CHECK(record.permit_revoked && record.authority_released);
  }
}

void test_contact_evidence_refusals() {
  g_case = "contact evidence refusals";
  struct Case {
    const char* label;
    void (*mutate)(FullLegRunOutcome&);
    FullLegFinalizeFailure expected;
    int contacts_recorded;
    bool min_recorded;
    bool max_recorded;
  };
  const Case cases[] = {
      {"min wrong side",
       [](FullLegRunOutcome& o) { o.min_contact.key.side = ContactSide::MAX_SIDE; },
       FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED, 0, false, false},
      {"max wrong leg",
       [](FullLegRunOutcome& o) {
         o.max_contact.key.leg =
             static_cast<Leg>((static_cast<uint8_t>(o.max_contact.key.leg) + 1) % kLegCount);
       },
       FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED, 0, false, false},
      {"min wrong joint",
       [](FullLegRunOutcome& o) { o.min_contact.key.joint = JointKind::HIP; },
       FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED, 0, false, false},
      {"max not measured",
       [](FullLegRunOutcome& o) { o.max_contact.has_measurement = false; },
       FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED, 0, false, false},
      {"min not confirmed",
       [](FullLegRunOutcome& o) { o.min_contact.detection = ContactState::CONTACT_SUSPECTED; },
       FullLegFinalizeFailure::MIN_CONTACT_REJECTED, 0, false, false},
      {"min witness rejected",
       [](FullLegRunOutcome& o) {
         o.min_contact.witness = makeContactWitness(0, kWitnessBand + 1, kWitnessBand);
       },
       FullLegFinalizeFailure::MIN_CONTACT_REJECTED, 0, false, false},
      {"min replay origin",
       [](FullLegRunOutcome& o) { o.min_contact.origin = CalibrationOrigin::HISTORICAL_REPLAY; },
       FullLegFinalizeFailure::MIN_CONTACT_REJECTED, 0, false, false},
      {"max not confirmed",
       [](FullLegRunOutcome& o) { o.max_contact.detection = ContactState::EARLY_STALL; },
       FullLegFinalizeFailure::MAX_CONTACT_REJECTED, 1, true, false},
      {"max witness rejected",
       [](FullLegRunOutcome& o) {
         o.max_contact.witness = makeContactWitness(0, kWitnessBand + 1, kWitnessBand);
       },
       FullLegFinalizeFailure::MAX_CONTACT_REJECTED, 1, true, false},
  };

  for (Leg leg : kAllLegs) {
    for (const Case& c : cases) {
      g_case = c.label;
      Rig rig;
      CHECK(rig.startLeg(leg));
      const FullLegPlan plan = rig.planFor(leg);
      FullLegRunOutcome outcome = rig.goodOutcome(leg);
      c.mutate(outcome);
      FullLegRecord record{};
      CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == c.expected);
      CHECK_EQ(rig.manager.status().contacts_recorded, c.contacts_recorded);
      CHECK(record.min_recorded == c.min_recorded);
      CHECK(record.max_recorded == c.max_recorded);
      CHECK(record.verdict == FullLegVerdict::FAILED);
      CHECK(!record.hardware_contact_calibrated);
      CHECK(!record.operational_envelope_accepted);
      CHECK(rig.manager.status().state == SessionState::FAILED);
      CHECK(!record.session_completed);
      rig.expectCleanBetweenLegs();
      CHECK(record.permit_revoked && record.authority_released);
      for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
        CHECK(rig.policy.limits().find(identityOf(oracleFor(leg, kind)),
                                       rig.policy.currentGeometryTag()) == nullptr);
      }
    }
  }
}

void test_envelope_refusals_leave_no_limit_and_no_pass() {
  g_case = "envelope refusals";
  {
    // Contacts measured under a geometry that is no longer current.
    Rig rig(/*approved=*/true);
    CHECK(rig.startLeg(Leg::LF));
    const FullLegPlan plan = rig.planFor(Leg::LF);
    FullLegRunOutcome outcome = rig.goodOutcome(Leg::LF);
    outcome.geometry_at_start = rig.policy.currentGeometryTag() ^ 0x1;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::UPPER_ENVELOPE_NOT_READY);
    CHECK(record.joint(JointKind::UPPER).envelope_status ==
          actuator::EnvelopeBuildStatus::REJECT_CONTACT_GEOMETRY_MISMATCH);
    // Recorded before the envelope was attempted; still not a pass.
    CHECK_EQ(rig.manager.status().contacts_recorded, 2);
    CHECK(record.verdict == FullLegVerdict::FAILED);
    CHECK(!record.hardware_contact_calibrated);
    CHECK(rig.manager.status().state == SessionState::FAILED);
    rig.expectCleanBetweenLegs();
    for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
      CHECK(rig.policy.limits().find(identityOf(oracleFor(Leg::LF, kind)),
                                     rig.policy.currentGeometryTag()) == nullptr);
    }
  }
  {
    // Contacts closer together than twice the margin: the envelope collapses.
    Rig rig;
    CHECK(rig.startLeg(Leg::RH));
    const FullLegPlan plan = rig.planFor(Leg::RH);
    FullLegRunOutcome outcome = rig.goodOutcome(Leg::RH);
    outcome.min_contact.fine_tick_1 = 2000;
    outcome.max_contact.fine_tick_1 = 2010;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::UPPER_ENVELOPE_NOT_READY);
    CHECK(record.joint(JointKind::UPPER).envelope_status ==
          actuator::EnvelopeBuildStatus::REJECT_CONTACT_ORDER_INVALID);
    CHECK(record.verdict == FullLegVerdict::FAILED);
    CHECK(rig.manager.status().state == SessionState::FAILED);
    rig.expectCleanBetweenLegs();
  }
  {
    // Approved parameters but HIP has lost its transform: nothing is admitted,
    // not even the UPPER limit that could have been.
    Rig rig(/*approved=*/true);
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan plan = rig.planFor(Leg::RF);
    const FullLegRunOutcome outcome = rig.goodOutcome(Leg::RF);
    actuator::JointTransformTable trimmed;
    for (const JointOracle& o : kOracle) {
      if (o.leg == Leg::RF && o.joint == JointKind::HIP) continue;
      CHECK(trimmed.admit(promotedTransform(o)));
    }
    rig.policy.transforms() = trimmed;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::HIP_ENVELOPE_NOT_READY);
    CHECK(record.joint(JointKind::HIP).envelope_status ==
          actuator::EnvelopeBuildStatus::REJECT_NO_TRANSFORM);
    CHECK(record.verdict == FullLegVerdict::FAILED);
    CHECK(!record.operational_envelope_accepted);
    for (JointKind kind : {JointKind::UPPER, JointKind::HIP, JointKind::LOWER}) {
      CHECK(rig.policy.limits().find(identityOf(oracleFor(Leg::RF, kind)),
                                     rig.policy.currentGeometryTag()) == nullptr);
    }
    rig.expectCleanBetweenLegs();
  }
  {
    // Unapproved parameters: HIP/LOWER refusal is REPORTED but is not a run
    // failure - level 1 does not depend on placeholder envelopes.
    Rig rig;
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan plan = rig.planFor(Leg::RF);
    const FullLegRunOutcome outcome = rig.goodOutcome(Leg::RF);
    actuator::JointTransformTable trimmed;
    for (const JointOracle& o : kOracle) {
      if (o.leg == Leg::RF && o.joint == JointKind::HIP) continue;
      CHECK(trimmed.admit(promotedTransform(o)));
    }
    rig.policy.transforms() = trimmed;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::NONE);
    CHECK(record.joint(JointKind::HIP).envelope_status ==
          actuator::EnvelopeBuildStatus::REJECT_NO_TRANSFORM);
    CHECK(record.verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
    CHECK(!record.operational_envelope_accepted);
  }
}

void test_context_incomplete_still_revokes_the_permit() {
  g_case = "context incomplete";
  Rig rig;
  CHECK(rig.startLeg(Leg::LF));
  const FullLegPlan plan = rig.planFor(Leg::LF);
  const FullLegRunOutcome outcome = rig.goodOutcome(Leg::LF);

  FullLegFinalizeContext broken = rig.context;
  broken.manager = nullptr;
  FullLegRecord record{};
  CHECK(finalizeFullLeg(broken, plan, outcome, &record) ==
        FullLegFinalizeFailure::CONTEXT_INCOMPLETE);
  CHECK(record.verdict == FullLegVerdict::FAILED);
  CHECK(!rig.permit.active());
  CHECK(!rig.authorization.operator_authorized);
  CHECK(!record.authority_released);

  CHECK(finalizeFullLeg(rig.context, plan, outcome, nullptr) ==
        FullLegFinalizeFailure::CONTEXT_INCOMPLETE);
  rig.manager.abortSession();
}

void test_four_legs_sequential_in_one_boot_session() {
  g_case = "four legs sequential";
  Rig rig;
  FullLegEvidenceStore store;
  store.reset();
  uint32_t last_session_id = 0;

  for (Leg leg : kAllLegs) {
    CHECK(rig.startLeg(leg));
    CHECK(rig.manager.status().leg == leg);
    CHECK(rig.manager.status().session_id > last_session_id);
    last_session_id = rig.manager.status().session_id;

    const FullLegPlan plan = rig.planFor(leg);
    const FullLegRunOutcome outcome = rig.goodOutcome(leg);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::NONE);
    store.commit(record);
    rig.expectCleanBetweenLegs();
    CHECK(record.verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
  }

  CHECK_EQ(store.legsPresent(), 4);
  CHECK_EQ(store.legsContactCalibrated(), 4);
  CHECK_EQ(store.legsEnvelopeAccepted(), 0);
  CHECK(store.allLegsContactCalibrated());
  CHECK_EQ(rig.manager.status().sessions_completed, 4);
  CHECK_EQ(rig.manager.status().sessions_failed, 0);
  for (Leg leg : kAllLegs) {
    const FullLegRecord* r = store.find(leg);
    CHECK(r != nullptr);
    if (r == nullptr) continue;
    CHECK(r->leg == leg);
    CHECK_EQ(r->attempts, 1);
    CHECK(r->hardware_contact_calibrated);
  }
}

void test_one_leg_failing_does_not_falsify_the_others() {
  g_case = "isolated failure";
  Rig rig;
  FullLegEvidenceStore store;

  for (Leg leg : kAllLegs) {
    CHECK(rig.startLeg(leg));
    const FullLegPlan plan = rig.planFor(leg);
    FullLegRunOutcome outcome = rig.goodOutcome(leg);
    if (leg == Leg::RF) outcome.max_contact.detection = ContactState::EARLY_STALL;
    FullLegRecord record{};
    const FullLegFinalizeFailure failure = finalizeFullLeg(rig.context, plan, outcome, &record);
    CHECK((failure == FullLegFinalizeFailure::NONE) == (leg != Leg::RF));
    store.commit(record);
    rig.expectCleanBetweenLegs();
  }

  CHECK_EQ(store.legsPresent(), 4);
  CHECK_EQ(store.legsContactCalibrated(), 3);
  CHECK(!store.allLegsContactCalibrated());
  CHECK(store.find(Leg::LF)->verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
  CHECK(store.find(Leg::RF)->verdict == FullLegVerdict::FAILED);
  CHECK(store.find(Leg::RF)->failure == FullLegFinalizeFailure::MAX_CONTACT_REJECTED);
  CHECK(store.find(Leg::RH)->verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
  CHECK(store.find(Leg::LH)->verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);

  // The failed leg can be rerun: it replaces only its own slot.
  CHECK(rig.startLeg(Leg::RF));
  const FullLegPlan plan = rig.planFor(Leg::RF);
  const FullLegRunOutcome outcome = rig.goodOutcome(Leg::RF);
  FullLegRecord record{};
  CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::NONE);
  store.commit(record);
  CHECK_EQ(store.find(Leg::RF)->attempts, 2);
  CHECK(store.find(Leg::RF)->verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
  CHECK_EQ(store.find(Leg::LF)->attempts, 1);
  CHECK(store.allLegsContactCalibrated());
}

void test_store_reset_and_bounds() {
  g_case = "store";
  FullLegEvidenceStore store;
  CHECK_EQ(store.legsPresent(), 0);
  CHECK(store.find(Leg::LF) == nullptr);
  CHECK(store.find(static_cast<Leg>(9)) == nullptr);

  FullLegRecord record{};
  record.leg = static_cast<Leg>(9);
  record.present = true;
  store.commit(record);  // unknown leg: ignored
  CHECK_EQ(store.legsPresent(), 0);

  record.leg = Leg::RH;
  store.commit(record);
  CHECK_EQ(store.legsPresent(), 1);
  CHECK(store.find(Leg::RH) != nullptr);
  store.reset();
  CHECK_EQ(store.legsPresent(), 0);
  CHECK(store.find(Leg::RH) == nullptr);
}

void test_export_is_deterministic_and_complete() {
  g_case = "export";
  Rig rig;
  FullLegEvidenceStore store;
  const actuator::GeometryProvenanceTag tag = rig.policy.currentGeometryTag();

  // Empty store: header, four NOT_RUN legs, footer.
  {
    Lines lines;
    exportFullLegEvidence(store, tag, rig.context.parameters, &collect, &lines);
    CHECK_EQ(lines.v.size(), 6);
    CHECK(has(lines, "CALIBRATION_EVIDENCE_EXPORT=BEGIN format=1"));
    CHECK(has(lines, "CALIBRATION_EVIDENCE_LEG leg=LF present=0 attempts=0 verdict=NOT_RUN"));
    CHECK(has(lines, "CALIBRATION_EVIDENCE_LEG leg=LH present=0 attempts=0 verdict=NOT_RUN"));
    CHECK(has(lines, "legs_present=0 legs_contact_calibrated=0 legs_envelope_accepted=0 "
                     "all_contact_calibrated=0"));
  }

  for (Leg leg : kAllLegs) {
    CHECK(rig.startLeg(leg));
    const FullLegPlan plan = rig.planFor(leg);
    FullLegRunOutcome outcome = rig.goodOutcome(leg);
    if (leg == Leg::RH) outcome.min_contact.detection = ContactState::CONTACT_SUSPECTED;
    FullLegRecord record{};
    finalizeFullLeg(rig.context, plan, outcome, &record);
    store.commit(record);
  }

  Lines first;
  Lines second;
  exportFullLegEvidence(store, tag, rig.context.parameters, &collect, &first);
  exportFullLegEvidence(store, tag, rig.context.parameters, &collect, &second);
  CHECK(first.v == second.v);
  // 1 header + 4 legs x (LEG + AUX + 3 Q0 + 2 CONTACT + 3 ENVELOPE + 3 LIMIT) + 1 footer.
  CHECK_EQ(first.v.size(), 1 + 4 * 13 + 1);
  for (const std::string& line : first.v) CHECK(line.size() < 383);

  CHECK(has(first, "CALIBRATION_EVIDENCE_EXPORT=BEGIN format=1 geometry="));
  CHECK(has(first, "parameters_approved=0 upper_margin_ticks=8 hip_lower_margin_urad=50000"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_LEG leg=LF present=1 attempts=1"));
  CHECK(has(first, "verdict=HARDWARE_CONTACT_CALIBRATED contact_calibrated=1 envelope_accepted=0"));
  CHECK(has(first, "leg=RH present=1 attempts=1"));
  CHECK(has(first, "verdict=FAILED contact_calibrated=0 envelope_accepted=0 "
                   "failure=MIN_CONTACT_REJECTED"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_AUX leg=LF required=1 aux_leg=LH aux_joint=UPPER aux_bus=42 "
                   "park_target_urad=610865"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_AUX leg=RF required=1 aux_leg=RH aux_joint=UPPER aux_bus=32"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_AUX leg=RH required=0"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_AUX leg=LH required=0"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_Q0 leg=LF joint=UPPER unit=ELR01 bus=12 present=1 q0_tick=2100"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_Q0 leg=LH joint=LOWER unit=M41 bus=41 present=1 q0_tick=2073"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_CONTACT leg=LF joint=UPPER side=MIN recorded=1"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_CONTACT leg=LF joint=UPPER side=MAX recorded=1"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_LIMIT leg=LF joint=UPPER "
                   "admission=NOT_ADMITTED_UNAPPROVED_PARAMETERS"));
  CHECK(has(first, "placeholder=1"));
  CHECK(has(first, "legs_present=4 legs_contact_calibrated=3 legs_envelope_accepted=0 "
                   "all_contact_calibrated=0"));

  // A null sink is a no-op, not a crash.
  exportFullLegEvidence(store, tag, rig.context.parameters, nullptr, nullptr);

  // Leg order in the export is fixed: LF, RF, RH, LH.
  size_t lf = 0, rf = 0, rh = 0, lh = 0;
  for (size_t i = 0; i < first.v.size(); ++i) {
    if (first.v[i].rfind("CALIBRATION_EVIDENCE_LEG leg=LF", 0) == 0) lf = i;
    if (first.v[i].rfind("CALIBRATION_EVIDENCE_LEG leg=RF", 0) == 0) rf = i;
    if (first.v[i].rfind("CALIBRATION_EVIDENCE_LEG leg=RH", 0) == 0) rh = i;
    if (first.v[i].rfind("CALIBRATION_EVIDENCE_LEG leg=LH", 0) == 0) lh = i;
  }
  CHECK(lf < rf && rf < rh && rh < lh);
}

void test_outcome_from_executor() {
  g_case = "outcomeFromExecutor";
  FullLegCalibrationExecutor executor;  // never begun: start() must refuse
  FullLegRunOutcome idle = outcomeFromExecutor(executor, 7, 3);
  CHECK(!idle.terminal);
  CHECK(!idle.complete);
  CHECK(idle.geometry_at_start == 7);
  CHECK_EQ(idle.session_id_at_start, 3);

  FullLegCalibrationRequest request{};
  FullLegCalibrationContext ctx{};
  CHECK(!executor.start(request, ctx, 0));
  FullLegRunOutcome failed = outcomeFromExecutor(executor, 7, 3);
  CHECK(failed.terminal);
  CHECK(!failed.complete);
  CHECK(failed.failure == FullLegCalibrationFailure::REJECT_PRECONDITIONS);
}

void test_enum_strings_are_distinct_and_named() {
  g_case = "toString";
  CHECK_STR(toString(FullLegVerdict::HARDWARE_CONTACT_CALIBRATED), "HARDWARE_CONTACT_CALIBRATED");
  CHECK_STR(toString(FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED),
            "FINAL_OPERATIONAL_ENVELOPE_ACCEPTED");
  CHECK_STR(toString(FullLegVerdict::NOT_RUN), "NOT_RUN");
  CHECK_STR(toString(FullLegVerdict::FAILED), "FAILED");
  CHECK_STR(toString(FullLegLimitAdmission::ADMITTED), "ADMITTED");
  CHECK_STR(toString(FullLegFinalizeFailure::NONE), "NONE");
  CHECK_STR(toString(actuator::LimitAdmission::REJECT_NO_TRANSFORM), "REJECT_NO_TRANSFORM");
  for (uint8_t i = 0; i <= 18; ++i) {
    CHECK(std::strcmp(toString(static_cast<FullLegFinalizeFailure>(i)), "UNKNOWN") != 0);
  }
}

}  // namespace

int main() {
  test_production_parameters_are_placeholders_and_unapproved();
  test_policy_operational_limit_gate();
  test_unapproved_lifecycle_for_every_leg();
  test_approved_lifecycle_admits_current_limits_for_every_leg();
  test_run_not_terminal_touches_nothing();
  test_executor_failure_cleans_up_without_evidence();
  test_session_mismatch_never_touches_a_foreign_session();
  test_contact_evidence_refusals();
  test_envelope_refusals_leave_no_limit_and_no_pass();
  test_context_incomplete_still_revokes_the_permit();
  test_four_legs_sequential_in_one_boot_session();
  test_one_leg_failing_does_not_falsify_the_others();
  test_store_reset_and_bounds();
  test_export_is_deterministic_and_complete();
  test_outcome_from_executor();
  test_enum_strings_are_distinct_and_named();

  std::printf("full_leg_calibration_finalizer: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
