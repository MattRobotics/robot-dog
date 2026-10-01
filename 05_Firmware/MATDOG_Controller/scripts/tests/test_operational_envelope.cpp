// Offline tests for the CR3 Priority 6 operational-envelope builder
// (src/actuator/OperationalEnvelope.*).
//
// Links the REAL geometry profile, the REAL checked target resolver and the
// REAL calibration domain predicates (mayPromote/isOperationalEvidence/
// isContactEvidence) - no mocks for the rules under test. No fabricated
// contact measurement: every ContactEvidence here is synthetic test
// fixture data, never presented as a real UPPER endpoint result.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
#include "../../src/actuator/OperationalEnvelope.h"

using namespace matdog;
using namespace matdog::actuator;
using matdog::calibration::CalibrationOrigin;
using matdog::calibration::ContactEvidence;
using matdog::calibration::ContactSide;
using matdog::calibration::ContactState;
using matdog::calibration::EvidenceState;
using matdog::calibration::JointIdentity;
using matdog::calibration::JointKind;
using matdog::calibration::Leg;
using matdog::calibration::makeContactWitness;
using matdog::calibration::setPhysicalUnit;

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

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                              \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));          \
    }                                                                          \
  } while (0)

namespace {

JointIdentity joint(Leg leg, JointKind kind, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = kind;
  setPhysicalUnit(&id, unit);
  return id;
}

// LF_HIP: unit "M22", bus 13, encoder_direction=-1 (current-installation
// witness 2026-10-01), urdf range +/-785398 urad (CalibrationGeometryProfileData.h).
JointIdentity lfHip() { return joint(Leg::LF, JointKind::HIP, "M22"); }
// LH_HIP: unit "M43", bus 43, encoder_direction=+1 (slot evidence). LF_HIP and
// LH_HIP together prove the min/max-tick ordering never assumes a sign.
JointIdentity lhHip() { return joint(Leg::LH, JointKind::HIP, "M43"); }
// LF_UPPER: the same executable endpoint used throughout the CR3 test suite.
JointIdentity lfUpper() { return joint(Leg::LF, JointKind::UPPER, "ELR01"); }

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&geometry_data::kProvenance, geometry_data::kJoints, geometry_data::kJointCount,
              geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return profile;
}

JointTransform promotedTransform(JointIdentity id, uint16_t q0 = 2048) {
  JointTransform t{};
  t.identity = id;
  t.geometry = geometryProvenanceTag(geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.q0_tick = q0;
  t.present = true;
  return t;
}

// Resolves the SAME way the builder under test does, so expected tick values
// are never hand-computed.
uint16_t resolvedTick(JointIdentity id, MicroRad target_urad, uint16_t q0 = 2048) {
  CalibrationGeometryProfile profile = boundProfile();
  const JointTransform transform = promotedTransform(id, q0);
  uint16_t tick = 0;
  const TargetResolveStatus status =
      resolveUrdfQToRaw(profile, geometry_data::kProvenance, transform, target_urad, &tick);
  CHECK(status == TargetResolveStatus::OK);
  return tick;
}

GeometryDerivedEnvelopeRequest hipRequest(MicroRad lo = -400000, MicroRad hi = 400000,
                                          MicroRad margin = 50000) {
  GeometryDerivedEnvelopeRequest r{};
  r.joint = lfHip();
  r.required_min_urad = lo;
  r.required_max_urad = hi;
  r.safety_margin_urad = margin;
  return r;
}

ContactEvidence contactEvidence(Leg leg, JointKind kind, uint16_t fine_tick,
                                CalibrationOrigin origin = CalibrationOrigin::LIVE_SESSION,
                                EvidenceState state = EvidenceState::PROMOTED,
                                ContactState detection = ContactState::CONTACT_CONFIRMED,
                                bool witness_accepted = true) {
  ContactEvidence e{};
  e.key.leg = leg;
  e.key.joint = kind;
  e.key.side = ContactSide::MIN_SIDE;
  e.state = state;
  e.origin = origin;
  e.detection = detection;
  e.witness = makeContactWitness(/*min_deviation_ticks=*/4, /*max_deviation_ticks=*/4,
                                 witness_accepted ? 16 : 2);
  e.coarse_tick = static_cast<uint16_t>(fine_tick - 10);
  e.fine_tick_1 = fine_tick;
  e.fine_tick_2 = fine_tick;
  e.has_measurement = true;
  return e;
}

ContactDerivedEnvelopeRequest upperRequest(uint16_t min_side_tick = 1400,
                                           uint16_t max_side_tick = 2700,
                                           uint16_t margin = 10) {
  ContactDerivedEnvelopeRequest r{};
  r.joint = lfUpper();
  r.min_side_evidence = contactEvidence(Leg::LF, JointKind::UPPER, min_side_tick);
  r.max_side_evidence = contactEvidence(Leg::LF, JointKind::UPPER, max_side_tick);
  const GeometryProvenanceTag tag = geometryProvenanceTag(geometry_data::kProvenance);
  r.min_side_geometry = tag;
  r.max_side_geometry = tag;
  r.safety_margin_ticks = margin;
  return r;
}

// ---------------------------------------------------------------------------
// Geometry-derived (HIP/LOWER) — happy path, both directions.
// ---------------------------------------------------------------------------

void test_geometry_derived_happy_path_positive_direction() {
  g_case = "geometry-derived happy path, direction +1";
  CalibrationGeometryProfile profile = boundProfile();
  CHECK_EQ(jointDirection(profile, lhHip()), 1);
  const JointTransform transform = promotedTransform(lhHip());
  GeometryDerivedEnvelopeRequest req = hipRequest();
  req.joint = lhHip();
  OperationalEnvelope env{};

  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, req, &env);

  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::READY);
  CHECK(env.present);
  CHECK_EQ((int)env.source, (int)EnvelopeSource::DERIVED_FROM_GEOMETRY);
  CHECK_EQ(env.min_tick, resolvedTick(lhHip(), -350000));
  CHECK_EQ(env.max_tick, resolvedTick(lhHip(), 350000));
  CHECK(env.ordered());
  CHECK_EQ(env.geometry, geometryProvenanceTag(geometry_data::kProvenance));
}

void test_geometry_derived_happy_path_negative_direction() {
  // LF_HIP has encoder_direction=-1: the numerically smaller URDF angle
  // must NOT be assumed to resolve to the numerically smaller raw tick.
  g_case = "geometry-derived happy path, direction -1";
  CalibrationGeometryProfile profile = boundProfile();
  CHECK_EQ(jointDirection(profile, lfHip()), -1);
  const JointTransform transform = promotedTransform(lfHip());
  GeometryDerivedEnvelopeRequest req = hipRequest();
  req.joint = lfHip();
  OperationalEnvelope env{};

  const EnvelopeBuildStatus status =
      buildGeometryDerivedEnvelope(profile, geometry_data::kProvenance, transform, req, &env);

  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::READY);
  CHECK(env.present);
  CHECK(env.ordered());
  const uint16_t a = resolvedTick(lfHip(), -350000);
  const uint16_t b = resolvedTick(lfHip(), 350000);
  CHECK_EQ(env.min_tick, a < b ? a : b);
  CHECK_EQ(env.max_tick, a < b ? b : a);
  // direction=-1 means the negative URDF angle resolves to the LARGER tick.
  CHECK(a > b);
}

// ---------------------------------------------------------------------------
// Geometry-derived — every fail-closed path.
// ---------------------------------------------------------------------------

void test_geometry_derived_rejects_unbound_geometry() {
  g_case = "geometry-derived: unbound geometry";
  CalibrationGeometryProfile profile;  // never bound
  const JointTransform transform = promotedTransform(lfHip());
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(), &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_NO_GEOMETRY);
  CHECK(!env.present);
}

void test_geometry_derived_rejects_unknown_joint() {
  g_case = "geometry-derived: unknown joint";
  CalibrationGeometryProfile profile = boundProfile();
  JointIdentity unknown = joint(Leg::LF, JointKind::HIP, "NOT_A_REAL_UNIT");
  const JointTransform transform = promotedTransform(unknown);
  GeometryDerivedEnvelopeRequest req = hipRequest();
  req.joint = unknown;
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildGeometryDerivedEnvelope(profile, geometry_data::kProvenance, transform, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_UNKNOWN_JOINT);
}

void test_geometry_derived_rejects_missing_transform() {
  g_case = "geometry-derived: no usable transform";
  CalibrationGeometryProfile profile = boundProfile();
  JointTransform transform{};  // present=false: no q0 captured
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(), &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_NO_TRANSFORM);
}

void test_geometry_derived_rejects_historical_replay_transform() {
  g_case = "geometry-derived: replay-origin transform not usable";
  CalibrationGeometryProfile profile = boundProfile();
  JointTransform transform = promotedTransform(lfHip());
  transform.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(), &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_NO_TRANSFORM);
}

void test_geometry_derived_rejects_malformed_workspace() {
  g_case = "geometry-derived: malformed workspace";
  CalibrationGeometryProfile profile = boundProfile();
  const JointTransform transform = promotedTransform(lfHip());
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(/*lo=*/400000, /*hi=*/-400000),
      &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_WORKSPACE_MALFORMED);
}

void test_geometry_derived_rejects_workspace_outside_urdf() {
  g_case = "geometry-derived: workspace outside URDF";
  CalibrationGeometryProfile profile = boundProfile();
  const JointTransform transform = promotedTransform(lfHip());
  OperationalEnvelope env{};
  // urdf range is +/-785398 for LF_HIP.
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(/*lo=*/-800000, /*hi=*/400000),
      &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_WORKSPACE_OUTSIDE_URDF);
}

void test_geometry_derived_rejects_negative_margin() {
  g_case = "geometry-derived: negative margin";
  CalibrationGeometryProfile profile = boundProfile();
  const JointTransform transform = promotedTransform(lfHip());
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(-400000, 400000, /*margin=*/-1),
      &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_MARGIN_INVALID);
}

void test_geometry_derived_rejects_margin_collapsing_range() {
  g_case = "geometry-derived: margin collapses range";
  CalibrationGeometryProfile profile = boundProfile();
  const JointTransform transform = promotedTransform(lfHip());
  OperationalEnvelope env{};
  // Workspace span is 800000 urad wide; a margin of 500000 from EACH end
  // collapses it (500000*2 > 800000).
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform,
      hipRequest(-400000, 400000, /*margin=*/500000), &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_MARGIN_COLLAPSES_RANGE);
}

void test_geometry_derived_zero_margin_is_the_workspace_itself() {
  g_case = "geometry-derived: zero margin allowed";
  CalibrationGeometryProfile profile = boundProfile();
  const JointTransform transform = promotedTransform(lfHip());
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildGeometryDerivedEnvelope(
      profile, geometry_data::kProvenance, transform, hipRequest(-400000, 400000, /*margin=*/0),
      &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::READY);
  const uint16_t a = resolvedTick(lfHip(), -400000);
  const uint16_t b = resolvedTick(lfHip(), 400000);
  CHECK_EQ(env.min_tick, a < b ? a : b);
  CHECK_EQ(env.max_tick, a < b ? b : a);
}

// ---------------------------------------------------------------------------
// Contact-derived (UPPER) — happy path.
// ---------------------------------------------------------------------------

void test_contact_derived_happy_path() {
  g_case = "contact-derived happy path";
  CalibrationGeometryProfile profile = boundProfile();
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, upperRequest(), &env);

  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::READY);
  CHECK(env.present);
  CHECK_EQ((int)env.source, (int)EnvelopeSource::MEASURED_CONTACT);
  CHECK_EQ(env.min_tick, 1410);  // 1400 + 10
  CHECK_EQ(env.max_tick, 2690);  // 2700 - 10
  CHECK(env.ordered());
}

void test_contact_derived_handles_reversed_side_ticks() {
  // The MIN_SIDE evidence need not itself be the numerically smaller tick -
  // the builder must not assume it is.
  g_case = "contact-derived: reversed side tick order";
  CalibrationGeometryProfile profile = boundProfile();
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildContactDerivedEnvelope(
      profile, geometry_data::kProvenance, upperRequest(/*min_side=*/2700, /*max_side=*/1400, 10),
      &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::READY);
  CHECK_EQ(env.min_tick, 1410);
  CHECK_EQ(env.max_tick, 2690);
}

// ---------------------------------------------------------------------------
// Contact-derived — every fail-closed path. This is THE priority-6 UPPER
// requirement: fail closed until real physical contact evidence exists.
// ---------------------------------------------------------------------------

void test_contact_derived_rejects_missing_evidence() {
  g_case = "contact-derived: no evidence at all (the default, real state today)";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req{};
  req.joint = lfUpper();
  // min_side_evidence / max_side_evidence left fully default: has_measurement=false.
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_MISSING_CONTACT_EVIDENCE);
  CHECK(!env.present);
  CHECK_EQ((int)env.source, (int)EnvelopeSource::NONE);
}

void test_contact_derived_rejects_one_sided_evidence() {
  g_case = "contact-derived: only one side present";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.max_side_evidence = ContactEvidence{};  // that side never happened
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_MISSING_CONTACT_EVIDENCE);
}

void test_contact_derived_rejects_wrong_slot() {
  g_case = "contact-derived: evidence from a different joint slot";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.min_side_evidence = contactEvidence(Leg::RF, JointKind::UPPER, 1400);  // wrong leg
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_WRONG_SLOT);
}

void test_contact_derived_rejects_historical_replay_origin() {
  g_case = "contact-derived: historical replay origin";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.min_side_evidence = contactEvidence(Leg::LF, JointKind::UPPER, 1400,
                                          CalibrationOrigin::HISTORICAL_REPLAY);
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_NOT_CURRENT);
}

void test_contact_derived_rejects_unpromoted_state() {
  g_case = "contact-derived: evidence not PROMOTED";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.min_side_evidence = contactEvidence(Leg::LF, JointKind::UPPER, 1400,
                                          CalibrationOrigin::LIVE_SESSION,
                                          EvidenceState::ACCEPTED);
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_NOT_CURRENT);
}

void test_contact_derived_rejects_unconfirmed_detection() {
  g_case = "contact-derived: detection not CONTACT_CONFIRMED";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.min_side_evidence =
      contactEvidence(Leg::LF, JointKind::UPPER, 1400, CalibrationOrigin::LIVE_SESSION,
                      EvidenceState::PROMOTED, ContactState::CONTACT_SUSPECTED);
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_NOT_CONFIRMED);
}

void test_contact_derived_rejects_unaccepted_witness() {
  g_case = "contact-derived: witness not accepted";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.min_side_evidence =
      contactEvidence(Leg::LF, JointKind::UPPER, 1400, CalibrationOrigin::LIVE_SESSION,
                      EvidenceState::PROMOTED, ContactState::CONTACT_CONFIRMED,
                      /*witness_accepted=*/false);
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_WITNESS_REJECTED);
}

void test_contact_derived_rejects_geometry_mismatch() {
  g_case = "contact-derived: evidence measured under a different model";
  CalibrationGeometryProfile profile = boundProfile();
  ContactDerivedEnvelopeRequest req = upperRequest();
  req.min_side_geometry = 0xDEADBEEFULL;  // not the current provenance tag
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, req, &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_GEOMETRY_MISMATCH);
}

void test_contact_derived_rejects_margin_collapsing_range() {
  g_case = "contact-derived: margin collapses range";
  CalibrationGeometryProfile profile = boundProfile();
  // Contact points only 10 ticks apart; a 10-tick margin from each end
  // collapses the range.
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status = buildContactDerivedEnvelope(
      profile, geometry_data::kProvenance, upperRequest(1400, 1410, /*margin=*/10), &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_CONTACT_ORDER_INVALID);
}

void test_contact_derived_rejects_unbound_geometry() {
  g_case = "contact-derived: unbound geometry";
  CalibrationGeometryProfile profile;  // never bound
  OperationalEnvelope env{};
  const EnvelopeBuildStatus status =
      buildContactDerivedEnvelope(profile, geometry_data::kProvenance, upperRequest(), &env);
  CHECK_EQ((int)status, (int)EnvelopeBuildStatus::REJECT_NO_GEOMETRY);
}

// ---------------------------------------------------------------------------
// toString()
// ---------------------------------------------------------------------------

void test_to_string_covers_every_value() {
  g_case = "to_string";
  CHECK_STR(toString(EnvelopeSource::NONE), "NONE");
  CHECK_STR(toString(EnvelopeSource::MEASURED_CONTACT), "MEASURED_CONTACT");
  CHECK_STR(toString(EnvelopeSource::DERIVED_FROM_GEOMETRY), "DERIVED_FROM_GEOMETRY");
  CHECK_STR(toString(static_cast<EnvelopeSource>(200)), "UNKNOWN");

  CHECK_STR(toString(EnvelopeBuildStatus::READY), "READY");
  CHECK_STR(toString(EnvelopeBuildStatus::REJECT_CONTACT_GEOMETRY_MISMATCH),
           "REJECT_CONTACT_GEOMETRY_MISMATCH");
  CHECK_STR(toString(static_cast<EnvelopeBuildStatus>(200)), "UNKNOWN");
}

}  // namespace

int main() {
  test_geometry_derived_happy_path_positive_direction();
  test_geometry_derived_happy_path_negative_direction();
  test_geometry_derived_rejects_unbound_geometry();
  test_geometry_derived_rejects_unknown_joint();
  test_geometry_derived_rejects_missing_transform();
  test_geometry_derived_rejects_historical_replay_transform();
  test_geometry_derived_rejects_malformed_workspace();
  test_geometry_derived_rejects_workspace_outside_urdf();
  test_geometry_derived_rejects_negative_margin();
  test_geometry_derived_rejects_margin_collapsing_range();
  test_geometry_derived_zero_margin_is_the_workspace_itself();
  test_contact_derived_happy_path();
  test_contact_derived_handles_reversed_side_ticks();
  test_contact_derived_rejects_missing_evidence();
  test_contact_derived_rejects_one_sided_evidence();
  test_contact_derived_rejects_wrong_slot();
  test_contact_derived_rejects_historical_replay_origin();
  test_contact_derived_rejects_unpromoted_state();
  test_contact_derived_rejects_unconfirmed_detection();
  test_contact_derived_rejects_unaccepted_witness();
  test_contact_derived_rejects_geometry_mismatch();
  test_contact_derived_rejects_margin_collapsing_range();
  test_contact_derived_rejects_unbound_geometry();
  test_to_string_covers_every_value();

  std::printf("test_operational_envelope: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
