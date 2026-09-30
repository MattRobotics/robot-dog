// Offline tests for the Full Leg evidence lifecycle and the 24-contact
// definition (src/calibration/FullLegCalibrationFinalizer.*), against the REAL
// CalibrationManager, arbiter, motion permit, SafeActuatorPolicy tables,
// Geometry V5 profile, sequence plan and plan resolver.
//
// FULL CALIBRATION = 4 legs x 3 joints x MIN/MAX = 24 contact witnesses.
// Proven here, not asserted in prose:
//   - 2/6 (the old UPPER-only scope) and 5/6 are a FAILED leg, 0 accepted;
//   - only 6/6 recorded contacts make a leg HARDWARE_CONTACT_CALIBRATED;
//   - 23/24 is not Full Calibration; only 24/24 gives all_contact_calibrated.
//
// NO HARDWARE VALIDATION. Every contact tick here is synthetic.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationSequencePlanData.h"
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
constexpr JointKind kJoints[3] = {JointKind::UPPER, JointKind::LOWER, JointKind::HIP};
constexpr Leg kAllLegs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};

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

ContactEvidence contactAt(Leg leg, JointKind joint, ContactSide side, uint16_t tick) {
  ContactEvidence e{};
  e.key.leg = leg;
  e.key.joint = joint;
  e.key.side = side;
  e.state = EvidenceState::PROMOTED;
  e.origin = kLive;
  e.detection = ContactState::CONTACT_CONFIRMED;
  e.witness = makeContactWitness(0, 2, kWitnessBand);
  e.coarse_tick = tick;
  e.fine_tick_1 = tick;
  e.fine_tick_2 = tick;
  e.repeatability_ticks = 2;
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
    policy.bindSequencePlan(&actuator::sequence_plan_data::kPlan);
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

  // SESSION START <leg> + PERMIT GRANT, then the executor's live V25 phase
  // reports up to TORQUE_OFF - exactly the state a finished run leaves.
  bool startLeg(Leg leg, bool report_phases = true) {
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
    if (report_phases) {
      for (uint8_t p = 0; p <= static_cast<uint8_t>(CalibrationPhase::TORQUE_OFF); ++p) {
        if (!manager.noteExecutionPhase(static_cast<CalibrationPhase>(p))) return false;
      }
    }
    return true;
  }

  FullLegPlan planFor(Leg leg) {
    FullLegPlan plan{};
    CHECK(resolveFullLegPlan(profile, actuator::geometry_data::kProvenance, policy.transforms(),
                             &actuator::sequence_plan_data::kPlan, leg, &plan) ==
          FullLegPlanStatus::OK);
    return plan;
  }

  // A complete, accepted 6/6 outcome at the canonical contacts of the plan.
  FullLegRunOutcome goodOutcome(const FullLegPlan& plan) {
    FullLegRunOutcome outcome{};
    outcome.terminal = true;
    outcome.complete = true;
    outcome.failure = FullLegFailure::NONE;
    for (const JointKind joint : kJoints) {
      const uint8_t k = static_cast<uint8_t>(joint);
      for (uint8_t s = 0; s < kContactSideCount; ++s) {
        outcome.contacts[k][s] = contactAt(plan.leg, joint, static_cast<ContactSide>(s),
                                           plan.request.corridor[k][s].contact_tick);
      }
      outcome.diagnostics[k] = deriveFullLegJointDiagnostics(plan.request, joint, outcome.contacts[k][0],
                                                             outcome.contacts[k][1]);
      CHECK(outcome.diagnostics[k].accepted);
    }
    outcome.contacts_measured = 6;
    outcome.diagnostics_accepted = true;
    outcome.geometry_at_start = policy.currentGeometryTag();
    outcome.session_id_at_start = manager.status().session_id;
    return outcome;
  }

  // The executor stopped after `measured` contacts in V25 order
  // (UPPER MIN, UPPER MAX, LOWER MIN, LOWER MAX, HIP MIN, HIP MAX).
  FullLegRunOutcome partialOutcome(const FullLegPlan& plan, uint8_t measured, FullLegFailure why) {
    FullLegRunOutcome outcome = goodOutcome(plan);
    outcome.complete = false;
    outcome.failure = why;
    outcome.diagnostics_accepted = false;
    uint8_t n = 0;
    for (const JointKind joint : kJoints) {
      for (uint8_t s = 0; s < kContactSideCount; ++s) {
        if (n++ >= measured) outcome.contacts[static_cast<uint8_t>(joint)][s] = ContactEvidence{};
      }
      outcome.diagnostics[static_cast<uint8_t>(joint)] = FullLegJointDiagnostics{};
    }
    outcome.contacts_measured = measured;
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

void expectFailedLeg(const FullLegRecord& record) {
  CHECK(record.verdict == FullLegVerdict::FAILED);
  CHECK(!record.hardware_contact_calibrated);
  CHECK(!record.operational_envelope_accepted);
  CHECK_EQ(record.contacts_accepted, 0);
  CHECK_EQ(record.contacts_expected, 6);
  for (const JointKind joint : kJoints) {
    CHECK(record.joint(joint).limit != FullLegLimitAdmission::ADMITTED);
  }
}

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

void test_the_definition_is_24_contacts() {
  g_case = "24-contact definition";
  CHECK_EQ(kFullLegContactsExpected, 6);
  CHECK_EQ(kFullLegContactCount, 6);
  CHECK_EQ(kFullCalibrationContactsExpected, 24);
  CHECK_EQ(kLegCount, 4);
  CHECK_EQ(kJointKindCount, 3);
  CHECK_EQ(kContactSideCount, 2);
  const FullLegEnvelopeParameters p = productionEnvelopeParameters();
  CHECK(!p.approved);
  CHECK(!kFullLegOperationalParametersApproved);
  CHECK_EQ(p.contact_margin_ticks, 8);
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

void test_six_of_six_is_a_calibrated_leg_for_every_leg() {
  for (const Leg leg : kAllLegs) {
    g_case = "6/6 -> HARDWARE_CONTACT_CALIBRATED";
    Rig rig;
    CHECK(rig.startLeg(leg));
    const FullLegPlan plan = rig.planFor(leg);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, rig.goodOutcome(plan), &record) == FullLegFinalizeFailure::NONE);
    CHECK(record.verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
    CHECK(record.hardware_contact_calibrated);
    CHECK(!record.operational_envelope_accepted);  // placeholder margin, unapproved
    CHECK_EQ(record.contacts_expected, 6);
    CHECK_EQ(record.contacts_measured, 6);
    CHECK_EQ(record.contacts_accepted, 6);
    CHECK_EQ(rig.manager.status().contacts_recorded, 6);  // each exactly once
    CHECK(record.session_completed && record.permit_revoked && record.authority_released);
    CHECK(rig.manager.status().state == SessionState::COMPLETED);
    for (const JointKind joint : kJoints) {
      const FullLegJointRecord& j = record.joint(joint);
      CHECK(j.contact_recorded[0] && j.contact_recorded[1]);
      CHECK(j.envelope_status == actuator::EnvelopeBuildStatus::READY);
      CHECK(j.envelope.source == actuator::EnvelopeSource::MEASURED_CONTACT);
      CHECK(j.limit == FullLegLimitAdmission::NOT_ADMITTED_UNAPPROVED_PARAMETERS);
      CHECK(j.diagnostics.accepted);
      CHECK(j.q0_present && j.q0_tick == oracleFor(leg, joint).q0);
      // The envelope is the two contacts inset by the placeholder margin.
      const uint16_t a = j.contact[0].fine_tick_1, b = j.contact[1].fine_tick_1;
      CHECK_EQ(j.envelope.min_tick, (a < b ? a : b) + 8);
      CHECK_EQ(j.envelope.max_tick, (a < b ? b : a) - 8);
      // Unapproved: nothing offered to the policy.
      CHECK(rig.policy.limits().find(j.identity, rig.policy.currentGeometryTag()) == nullptr);
    }
    CHECK(record.has_rear_park == (leg == Leg::LF || leg == Leg::RF));
    rig.expectCleanBetweenLegs();
  }
}

void test_approved_parameters_admit_all_three_limits() {
  for (const Leg leg : kAllLegs) {
    g_case = "approved: FINAL_OPERATIONAL_ENVELOPE_ACCEPTED";
    Rig rig(true);
    CHECK(rig.startLeg(leg));
    const FullLegPlan plan = rig.planFor(leg);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, rig.goodOutcome(plan), &record) == FullLegFinalizeFailure::NONE);
    CHECK(record.verdict == FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED);
    for (const JointKind joint : kJoints) {
      const FullLegJointRecord& j = record.joint(joint);
      CHECK(j.limit == FullLegLimitAdmission::ADMITTED);
      const actuator::JointLimit* l = rig.policy.limits().find(j.identity, rig.policy.currentGeometryTag());
      CHECK(l != nullptr && l->min_tick == j.envelope.min_tick && l->max_tick == j.envelope.max_tick);
    }
  }
}

void test_partial_legs_are_failed_legs() {
  struct Case {
    const char* name;
    uint8_t measured;
    FullLegFailure why;
  };
  // 2/6 is exactly the superseded UPPER-only "Full Leg" of PR #34.
  const Case cases[] = {
      {"0/6 -> FAILED", 0, FullLegFailure::UPPER_MIN_PROBE_FAILED},
      {"1/6 -> FAILED", 1, FullLegFailure::UPPER_MAX_PROBE_FAILED},
      {"2/6 (UPPER only) -> FAILED", 2, FullLegFailure::LOWER_MIN_PROBE_FAILED},
      {"3/6 -> FAILED", 3, FullLegFailure::LOWER_MAX_PROBE_FAILED},
      {"4/6 -> FAILED", 4, FullLegFailure::HIP_MIN_PROBE_FAILED},
      {"5/6 -> FAILED", 5, FullLegFailure::HIP_MAX_PROBE_FAILED},
  };
  for (const Case& c : cases) {
    for (const Leg leg : kAllLegs) {
      g_case = c.name;
      Rig rig;
      CHECK(rig.startLeg(leg));
      const FullLegPlan plan = rig.planFor(leg);
      FullLegRecord record{};
      CHECK(finalizeFullLeg(rig.context, plan, rig.partialOutcome(plan, c.measured, c.why), &record) ==
            FullLegFinalizeFailure::EXECUTOR_FAILED);
      expectFailedLeg(record);
      CHECK_EQ(record.contacts_measured, c.measured);
      CHECK(record.executor_failure == c.why);
      CHECK_EQ(rig.manager.status().contacts_recorded, 0);  // nothing recorded from a failed run
      CHECK(rig.manager.status().state == SessionState::FAILED);
      rig.expectCleanBetweenLegs();
    }
  }
}

void test_an_outcome_claiming_success_with_fewer_than_six_is_refused() {
  for (uint8_t measured = 0; measured < 6; ++measured) {
    g_case = "complete=true but < 6 contacts: CONTACTS_INCOMPLETE";
    Rig rig;
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan plan = rig.planFor(Leg::RF);
    FullLegRunOutcome outcome = rig.partialOutcome(plan, measured, FullLegFailure::NONE);
    outcome.complete = true;  // a lying (or buggy) executor snapshot
    outcome.diagnostics_accepted = true;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::CONTACTS_INCOMPLETE);
    expectFailedLeg(record);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
  }
  {
    g_case = "six measured but the count says five";
    Rig rig;
    CHECK(rig.startLeg(Leg::LH));
    const FullLegPlan plan = rig.planFor(Leg::LH);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.contacts_measured = 5;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::CONTACTS_INCOMPLETE);
    expectFailedLeg(record);
  }
}

void test_contact_and_diagnostic_refusals() {
  {
    g_case = "a contact filed under the wrong joint";
    Rig rig;
    CHECK(rig.startLeg(Leg::LF));
    const FullLegPlan plan = rig.planFor(Leg::LF);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.contacts[static_cast<uint8_t>(JointKind::HIP)][1].key.joint = JointKind::LOWER;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED);
    expectFailedLeg(record);
  }
  {
    g_case = "a contact of another leg";
    Rig rig;
    CHECK(rig.startLeg(Leg::RH));
    const FullLegPlan plan = rig.planFor(Leg::RH);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.contacts[static_cast<uint8_t>(JointKind::LOWER)][0].key.leg = Leg::LH;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) ==
          FullLegFinalizeFailure::CONTACT_EVIDENCE_MALFORMED);
    expectFailedLeg(record);
  }
  {
    g_case = "one joint's diagnostics rejected";
    Rig rig;
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan plan = rig.planFor(Leg::RF);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.diagnostics[static_cast<uint8_t>(JointKind::LOWER)].accepted = false;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::DIAGNOSTICS_REJECTED);
    expectFailedLeg(record);
    CHECK_EQ(rig.manager.status().contacts_recorded, 0);
  }
  {
    g_case = "the session refuses the sixth contact";
    Rig rig;
    CHECK(rig.startLeg(Leg::LH));
    const FullLegPlan plan = rig.planFor(Leg::LH);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    // The HIP MAX witness is not accepted: recordContact() refuses it, after
    // five were recorded. The leg is FAILED with 0 accepted, not 5.
    outcome.contacts[static_cast<uint8_t>(JointKind::HIP)][1].witness = makeContactWitness(0, 40, kWitnessBand);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::CONTACT_REJECTED);
    expectFailedLeg(record);
  }
  {
    g_case = "an envelope that cannot be built";
    Rig rig;
    CHECK(rig.startLeg(Leg::LF));
    const FullLegPlan plan = rig.planFor(Leg::LF);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.geometry_at_start = outcome.geometry_at_start ^ 0x1;  // captured under another model
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::ENVELOPE_NOT_READY);
    expectFailedLeg(record);
  }
}

void test_session_and_context_refusals() {
  {
    g_case = "not terminal: touches nothing";
    Rig rig;
    CHECK(rig.startLeg(Leg::LF));
    const FullLegPlan plan = rig.planFor(Leg::LF);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.terminal = false;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::RUN_NOT_TERMINAL);
    CHECK(rig.manager.sessionLive());
    CHECK(rig.permit.active());
  }
  {
    g_case = "a different session than the run started under";
    Rig rig;
    CHECK(rig.startLeg(Leg::RF));
    const FullLegPlan plan = rig.planFor(Leg::RF);
    FullLegRunOutcome outcome = rig.goodOutcome(plan);
    outcome.session_id_at_start += 1;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, outcome, &record) == FullLegFinalizeFailure::SESSION_MISMATCH);
    expectFailedLeg(record);
    CHECK(!rig.permit.active());
  }
  {
    g_case = "the session was opened for another leg";
    Rig rig;
    CHECK(rig.startLeg(Leg::RH));
    const FullLegPlan plan = rig.planFor(Leg::LH);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan, rig.goodOutcome(plan), &record) ==
          FullLegFinalizeFailure::SESSION_MISMATCH);
    expectFailedLeg(record);
    CHECK(rig.manager.sessionLive());  // never touches a foreign session
    CHECK(!rig.permit.active());       // ...but the permit always goes
  }
  {
    g_case = "context incomplete still revokes the permit";
    Rig rig;
    CHECK(rig.startLeg(Leg::LF));
    const FullLegPlan plan = rig.planFor(Leg::LF);
    FullLegFinalizeContext partial = rig.context;
    partial.policy = nullptr;
    FullLegRecord record{};
    CHECK(finalizeFullLeg(partial, plan, rig.goodOutcome(plan), &record) ==
          FullLegFinalizeFailure::CONTEXT_INCOMPLETE);
    expectFailedLeg(record);
    CHECK(!rig.permit.active());
  }
  {
    g_case = "operator abort ends the session ABORTED";
    Rig rig;
    CHECK(rig.startLeg(Leg::LH));
    const FullLegPlan plan = rig.planFor(Leg::LH);
    FullLegRecord record{};
    CHECK(finalizeFullLeg(rig.context, plan,
                          rig.partialOutcome(plan, 3, FullLegFailure::OPERATOR_ABORT), &record) ==
          FullLegFinalizeFailure::EXECUTOR_FAILED);
    expectFailedLeg(record);
    CHECK(rig.manager.status().state == SessionState::ABORTED);
  }
}

// --- the four-leg result ----------------------------------------------------------

void finalizeInto(Rig& rig, FullLegEvidenceStore& store, Leg leg, uint8_t measured) {
  CHECK(rig.startLeg(leg));
  const FullLegPlan plan = rig.planFor(leg);
  FullLegRecord record{};
  finalizeFullLeg(rig.context, plan,
                  measured == 6 ? rig.goodOutcome(plan)
                                : rig.partialOutcome(plan, measured, FullLegFailure::HIP_MAX_PROBE_FAILED),
                  &record);
  store.put(record);
  rig.expectCleanBetweenLegs();
}

void test_only_24_of_24_is_full_calibration() {
  {
    g_case = "4 x 6/6 in one boot = 24/24";
    Rig rig;
    FullLegEvidenceStore store;
    for (const Leg leg : kAllLegs) {
      CHECK(!store.allLegsContactCalibrated());
      finalizeInto(rig, store, leg, 6);
    }
    CHECK_EQ(store.legsPresent(), 4);
    CHECK_EQ(store.legsContactCalibrated(), 4);
    CHECK_EQ(store.totalContactsAccepted(), 24);
    CHECK(store.allLegsContactCalibrated());
    CHECK_EQ(store.legsEnvelopeAccepted(), 0);  // never claimed without approved parameters
  }
  {
    g_case = "three legs 6/6, one leg 5/6: 18/24, not Full Calibration";
    Rig rig;
    FullLegEvidenceStore store;
    finalizeInto(rig, store, Leg::LF, 6);
    finalizeInto(rig, store, Leg::RF, 6);
    finalizeInto(rig, store, Leg::RH, 6);
    finalizeInto(rig, store, Leg::LH, 5);
    CHECK_EQ(store.legsPresent(), 4);
    CHECK_EQ(store.legsContactCalibrated(), 3);
    CHECK_EQ(store.totalContactsAccepted(), 18);
    CHECK(!store.allLegsContactCalibrated());
    CHECK(store.find(Leg::LH)->verdict == FullLegVerdict::FAILED);
    // The failed leg does not falsify the others.
    CHECK(store.find(Leg::RH)->verdict == FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
  }
  {
    g_case = "23/24 (a forged 5-contact 'calibrated' record) is not Full Calibration";
    Rig rig;
    FullLegEvidenceStore store;
    finalizeInto(rig, store, Leg::LF, 6);
    finalizeInto(rig, store, Leg::RF, 6);
    finalizeInto(rig, store, Leg::RH, 6);
    finalizeInto(rig, store, Leg::LH, 6);
    FullLegRecord forged = *store.find(Leg::LH);
    forged.contacts_accepted = 5;  // still flagged calibrated
    store.put(forged);
    CHECK_EQ(store.totalContactsAccepted(), 23);
    CHECK_EQ(store.legsContactCalibrated(), 3);
    CHECK(!store.allLegsContactCalibrated());
  }
  {
    g_case = "the old UPPER-only total (4 x 2 = 8) is not Full Calibration";
    Rig rig;
    FullLegEvidenceStore store;
    for (const Leg leg : kAllLegs) finalizeInto(rig, store, leg, 2);
    CHECK_EQ(store.legsContactCalibrated(), 0);
    CHECK_EQ(store.totalContactsAccepted(), 0);
    CHECK(!store.allLegsContactCalibrated());
  }
  {
    g_case = "a failed retry replaces the leg's record; a later 6/6 restores it";
    Rig rig;
    FullLegEvidenceStore store;
    finalizeInto(rig, store, Leg::LF, 4);
    CHECK_EQ(store.find(Leg::LF)->attempts, 1);
    CHECK_EQ(store.legsContactCalibrated(), 0);
    finalizeInto(rig, store, Leg::LF, 6);
    CHECK_EQ(store.find(Leg::LF)->attempts, 2);
    CHECK_EQ(store.legsContactCalibrated(), 1);
    CHECK_EQ(store.totalContactsAccepted(), 6);
  }
}

void test_store_bounds() {
  g_case = "store";
  FullLegEvidenceStore store;
  CHECK_EQ(store.legsPresent(), 0);
  CHECK(store.find(static_cast<Leg>(9)) == nullptr);
  FullLegRecord record{};
  record.leg = static_cast<Leg>(9);
  record.present = true;
  store.put(record);
  CHECK_EQ(store.legsPresent(), 0);
  record.leg = Leg::RH;
  record.present = false;
  store.put(record);  // an absent record is not a run
  CHECK_EQ(store.legsPresent(), 0);
  record.present = true;
  store.put(record);
  CHECK_EQ(store.legsPresent(), 1);
  CHECK_EQ(store.totalContactsAccepted(), 0);
  store.reset();
  CHECK_EQ(store.legsPresent(), 0);
}

void test_export_is_deterministic_and_carries_the_definition() {
  g_case = "export";
  Rig rig;
  FullLegEvidenceStore store;
  const actuator::GeometryProvenanceTag tag = rig.policy.currentGeometryTag();
  {
    Lines lines;
    exportFullLegEvidence(store, tag, rig.context.parameters, &collect, &lines);
    CHECK_EQ(lines.v.size(), 6);
    CHECK(has(lines, "CALIBRATION_EVIDENCE_EXPORT=BEGIN format=2"));
    CHECK(has(lines, "contacts_per_leg=6 total_contacts_expected=24"));
    CHECK(has(lines, "CALIBRATION_EVIDENCE_LEG leg=LF present=0 attempts=0 verdict=NOT_RUN "
                     "contacts_expected=6 contacts_accepted=0"));
    CHECK(has(lines, "total_contacts_expected=24 total_contacts_accepted=0 all_contact_calibrated=0"));
  }
  finalizeInto(rig, store, Leg::LF, 6);
  finalizeInto(rig, store, Leg::RF, 6);
  finalizeInto(rig, store, Leg::RH, 2);
  finalizeInto(rig, store, Leg::LH, 6);

  Lines first, second;
  exportFullLegEvidence(store, tag, rig.context.parameters, &collect, &first);
  exportFullLegEvidence(store, tag, rig.context.parameters, &collect, &second);
  CHECK(first.v == second.v);
  // 1 + 4 x (LEG + LEG_CLOSE + PARK + 3 Q0 + 6 CONTACT + 3 DIAG + 3 ENVELOPE + 3 LIMIT) + 1.
  CHECK_EQ(first.v.size(), 1 + 4 * 21 + 1);
  for (const std::string& line : first.v) CHECK(line.size() < 400);
  CHECK(has(first, "CALIBRATION_EVIDENCE_LEG leg=LF present=1 attempts=1"));
  CHECK(has(first, "verdict=HARDWARE_CONTACT_CALIBRATED contacts_expected=6 contacts_measured=6 "
                   "contacts_accepted=6"));
  CHECK(has(first, "verdict=FAILED contacts_expected=6 contacts_measured=2 contacts_accepted=0"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_LEG_CLOSE leg=RH failure=EXECUTOR_FAILED "
                   "executor_failure=HIP_MAX_PROBE_FAILED failed_phase=PREFLIGHT session_completed=0 "
                   "permit_revoked=1 authority_released=1 parameters_approved=0"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_LEG_CLOSE leg=LF failure=NONE executor_failure=NONE "
                   "failed_phase=- session_completed=1 permit_revoked=1 authority_released=1 "
                   "parameters_approved=0"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_PARK leg=LF required=1 park_leg=LH park_joint=UPPER "
                   "park_bus=42 park_target_urad=610865"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_PARK leg=RF required=1 park_leg=RH park_joint=UPPER park_bus=32"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_PARK leg=RH required=0"));
  CHECK(has(first, "CALIBRATION_EVIDENCE_Q0 leg=LF joint=HIP unit=M22 bus=13 present=1 q0_tick=1996"));
  for (const char* j : {"UPPER", "LOWER", "HIP"}) {
    for (const char* s : {"MIN", "MAX"}) {
      CHECK(has(first, std::string("CALIBRATION_EVIDENCE_CONTACT leg=LH joint=") + j + " side=" + s +
                           " recorded=1 measured=1"));
    }
    CHECK(has(first, std::string("CALIBRATION_EVIDENCE_DIAG leg=LF joint=") + j + " evaluated=1"));
    CHECK(has(first, std::string("CALIBRATION_EVIDENCE_LIMIT leg=LF joint=") + j +
                         " admission=NOT_ADMITTED_UNAPPROVED_PARAMETERS"));
  }
  CHECK(has(first, "CALIBRATION_EVIDENCE_CONTACT leg=RH joint=HIP side=MAX recorded=0 measured=0"));
  CHECK(has(first, "legs_present=4 legs_contact_calibrated=3 legs_envelope_accepted=0 "
                   "total_contacts_expected=24 total_contacts_accepted=18 all_contact_calibrated=0"));
  exportFullLegEvidence(store, tag, rig.context.parameters, nullptr, nullptr);  // no crash

  // The final line of a 24/24 store.
  Rig all;
  FullLegEvidenceStore full;
  for (const Leg leg : kAllLegs) finalizeInto(all, full, leg, 6);
  Lines done;
  exportFullLegEvidence(full, tag, all.context.parameters, &collect, &done);
  CHECK(done.v.back() ==
        "CALIBRATION_EVIDENCE_EXPORT=END legs_present=4 legs_contact_calibrated=4 "
        "legs_envelope_accepted=0 total_contacts_expected=24 total_contacts_accepted=24 "
        "all_contact_calibrated=1");
}

// The longest value of every field at once: no line may reach the buffer
// (a silently truncated evidence line would drop its last fields).
void test_export_never_truncates() {
  g_case = "export worst-case line length";
  FullLegEvidenceStore store;
  for (const Leg leg : kAllLegs) {
    FullLegRecord r{};
    r.leg = leg;
    r.present = true;
    r.session_id = 4294967295u;
    r.geometry = 0xFFFFFFFFFFFFFFFFULL;
    r.verdict = FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED;
    r.failure = FullLegFinalizeFailure::CLEANUP_SESSION_NOT_TERMINAL;
    r.executor_failure = FullLegFailure::INITIAL_RECOVERY_OUT_OF_RANGE;
    r.executor_failed_phase = CalibrationPhase::RETURN_LOWER_HELD;
    r.has_rear_park = true;
    r.park_target_urad = -2147483647;
    for (const JointKind joint : kJoints) {
      FullLegJointRecord& j = r.joint(joint);
      setPhysicalUnit(&j.identity, "WWWWWWWW");
      j.bus_id = 255;
      j.q0_tick = 65535;
      j.q0_state = EvidenceState::PROMOTED;
      j.q0_origin = CalibrationOrigin::HISTORICAL_REPLAY;
      for (ContactEvidence& c : j.contact) {
        c.detection = ContactState::CONTACT_CONFIRMED;
        c.coarse_tick = c.fine_tick_1 = c.repeatability_ticks = 65535;
      }
      j.diagnostics.min_contact_tick = j.diagnostics.max_contact_tick = 65535;
      j.envelope_status = actuator::EnvelopeBuildStatus::REJECT_CONTACT_GEOMETRY_MISMATCH;
      j.envelope.source = actuator::EnvelopeSource::DERIVED_FROM_GEOMETRY;
      j.limit = FullLegLimitAdmission::NOT_ADMITTED_UNAPPROVED_PARAMETERS;
    }
    for (int a = 0; a < 70; ++a) store.put(r);  // attempts > 2 digits
  }
  Lines lines;
  FullLegEnvelopeParameters p{};
  p.contact_margin_ticks = 65535;
  exportFullLegEvidence(store, 0xFFFFFFFFFFFFFFFFULL, p, &collect, &lines);
  size_t longest = 0;
  for (const std::string& line : lines.v) longest = line.size() > longest ? line.size() : longest;
  CHECK(longest < 480);  // the buffer is 512
  CHECK(has(lines, "parameters_approved=0"));  // the LEG_CLOSE tail survives
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
  CHECK(failed.failure == FullLegFailure::REJECT_PRECONDITIONS);
  CHECK_EQ(failed.contacts_measured, 0);
}

void test_enum_strings() {
  g_case = "toString";
  CHECK_STR(toString(FullLegVerdict::HARDWARE_CONTACT_CALIBRATED), "HARDWARE_CONTACT_CALIBRATED");
  CHECK_STR(toString(FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED),
            "FINAL_OPERATIONAL_ENVELOPE_ACCEPTED");
  CHECK_STR(toString(FullLegFinalizeFailure::CONTACTS_INCOMPLETE), "CONTACTS_INCOMPLETE");
  for (uint8_t i = 0; i <= static_cast<uint8_t>(FullLegFinalizeFailure::CLEANUP_PERMIT_ACTIVE); ++i) {
    CHECK(std::strcmp(toString(static_cast<FullLegFinalizeFailure>(i)), "UNKNOWN") != 0);
  }
  CHECK_STR(toString(static_cast<FullLegFinalizeFailure>(200)), "UNKNOWN");
}

}  // namespace

int main() {
  test_the_definition_is_24_contacts();
  test_policy_operational_limit_gate();
  test_six_of_six_is_a_calibrated_leg_for_every_leg();
  test_approved_parameters_admit_all_three_limits();
  test_partial_legs_are_failed_legs();
  test_an_outcome_claiming_success_with_fewer_than_six_is_refused();
  test_contact_and_diagnostic_refusals();
  test_session_and_context_refusals();
  test_only_24_of_24_is_full_calibration();
  test_store_bounds();
  test_export_is_deterministic_and_carries_the_definition();
  test_export_never_truncates();
  test_outcome_from_executor();
  test_enum_strings();

  std::printf("test_full_leg_calibration_finalizer: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
