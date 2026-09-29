// Offline tests for promoting the CURRENT-BOOT q0 capture (not the frozen
// CR2-C package): CalibrationQ0CaptureSession::freshCapture(),
// prepareFreshQ0Evidence(), freshQ0CaptureIsPromoted(), the real
// JointTransformTable replace-on-readmit semantics, and the four-leg plan
// resolver consuming the newly promoted values.
//
// The frozen CR2-C package appears here only as a CONTRAST ORACLE: every fresh
// tick below is chosen to differ from the CR2-C tick of the same joint, so a
// regression that promotes the frozen data instead of the capture cannot pass.
//
// NO HARDWARE VALIDATION. Nothing here measured anything.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationQ0EvidencePreparation.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
#include "../../src/calibration/CalibrationQ0CaptureSession.h"
#include "../../src/calibration/FullLegCalibrationPlan.h"
#include "../../src/servo/ServoProfileData.h"

using namespace matdog;
using namespace matdog::actuator;
using namespace matdog::calibration;
using namespace matdog::servo;

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

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile p;
  p.bind(&geometry_data::kProvenance, geometry_data::kJoints, geometry_data::kJointCount,
         geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return p;
}

CensusResult goodCensus() {
  CensusResult c{};
  c.canonical_allocated = canonicalAllocatedCount();
  c.expected_now = expectedNowCount();
  c.present_expected = c.expected_now;
  c.absent_by_design = absentByDesignCount();
  c.scan_lo = kCanonicalScanLo;
  c.scan_hi = kCanonicalScanHi;
  c.verdict = CensusVerdict::PASS;
  return c;
}

PreflightResult goodPreflight() {
  PreflightResult p{};
  p.complete = true;
  p.joints_evaluated = kLegPreflightCount;
  p.pass_count = kLegPreflightCount;
  for (uint8_t i = 0; i < kLegPreflightCount; ++i) {
    const CanonicalServo* canonical = legServoAt(i);
    if (canonical == nullptr) continue;
    JointPreflightRecord& r = p.joints[i];
    r.expected_bus_id = canonical->bus_id;
    r.joint = canonical->joint;
    r.expected_physical_unit = canonical->physical_unit;
    r.observed_bus_id = canonical->bus_id;
    r.model = profile_data::kInvariants.model_expected;
    r.position_offset_read = true;
    r.position_offset = 0;
    r.torque_enable = 0;
    r.present_position = 2000 + i;
    r.profile = ProfileVerdict::MATCH;
    r.profile_registers_read = profile_data::kPersistentRegisterCount;
    r.result = JointPreflightResult::PASS;
  }
  return p;
}

constexpr uint8_t kSamples = 9;

Q0CaptureConfig goodConfig() {
  Q0CaptureConfig c{};
  c.samples_per_joint = kSamples;
  c.stability_budget_specified = true;
  c.max_stability_spread_ticks = 16;
  c.nominal_zero_pose_confirmed = true;
  c.started_at_ms = 1000;
  return c;
}

bool enterSampling(CalibrationQ0CaptureSession& s) {
  return s.start(goodConfig()) && s.markCensusStarted() && s.submitCensus(goodCensus()) &&
         s.markPreflightStarted() && s.submitPreflight(goodPreflight());
}

// Round-robin capture where joint i reads base + step * i on every pass.
bool runCapture(CalibrationQ0CaptureSession& s, int base, int step) {
  if (!enterSampling(s)) return false;
  for (uint8_t pass = 0; pass < kSamples; ++pass) {
    for (uint8_t joint = 0; joint < kLegServoSlotCount; ++joint) {
      Q0ReadRequest req{};
      if (!s.nextReadRequest(&req)) return false;
      Q0ReadObservation obs{};
      obs.bus_id = req.bus_id;
      obs.read_ok = true;
      obs.raw_tick = base + step * joint;
      obs.torque_enable = 0;
      if (!s.recordRead(obs)) return false;
    }
  }
  return s.status().state == Q0CaptureState::COMPLETE;
}

// First capture: 2050..2061. Second capture: 2010..2043 (step 3). Neither
// coincides with the CR2-C tick of the same joint.
constexpr int kFirstBase = 2050, kFirstStep = 1;
constexpr int kSecondBase = 2010, kSecondStep = 3;

uint16_t firstTick(uint8_t i) { return static_cast<uint16_t>(kFirstBase + kFirstStep * i); }
uint16_t secondTick(uint8_t i) { return static_cast<uint16_t>(kSecondBase + kSecondStep * i); }

const JointTransform* transformFor(const Q0EvidencePreparation& prepared,
                                   const JointIdentity& id) {
  for (uint8_t i = 0; i < prepared.transform_count; ++i) {
    if (identityPermitsEvidenceReuse(prepared.transforms[i].identity, id)) {
      return &prepared.transforms[i];
    }
  }
  return nullptr;
}

bool admitAll(JointTransformTable* table, const Q0EvidencePreparation& prepared) {
  uint8_t admitted = 0;
  for (uint8_t i = 0; i < prepared.transform_count; ++i) {
    if (table->admit(prepared.transforms[i])) ++admitted;
  }
  return admitted == prepared.transform_count;
}

// ---------------------------------------------------------------------------

void test_capture_view_lifecycle() {
  g_case = "capture view lifecycle";
  CalibrationQ0CaptureSession s;
  FreshQ0Capture v = s.freshCapture();
  CHECK(!v.complete);
  CHECK(!v.population_pass);

  CHECK(enterSampling(s));
  v = s.freshCapture();
  CHECK(!v.complete);          // sampling is not a finished capture
  CHECK(v.population_pass);    // the population half is already proven
  CHECK(v.capture_session_id != 0);

  s.fail(Q0CaptureFailure::EXTERNAL_ABORT);
  v = s.freshCapture();
  CHECK(!v.complete);

  s.reset();
  CHECK(runCapture(s, kFirstBase, kFirstStep));
  v = s.freshCapture();
  CHECK(v.complete);
  CHECK(v.population_pass);
  CHECK_EQ(v.candidate_count, 12);
  CHECK(v.candidates == s.candidates());
  CHECK_EQ(v.capture_session_id, s.status().capture_session_id);

  s.reset();
  CHECK(!s.freshCapture().complete);
}

void test_fresh_capture_is_promoted_not_frozen() {
  g_case = "fresh candidates, not frozen CR2-C";
  const CalibrationGeometryProfile p = boundProfile();
  CalibrationQ0CaptureSession s;
  CHECK(runCapture(s, kFirstBase, kFirstStep));

  const Q0EvidencePreparation fresh =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, s.freshCapture(), true);
  CHECK(fresh.ready());
  CHECK_EQ(fresh.transform_count, 12);
  CHECK(fresh.failed_record_index == 0xFF);

  const Q0EvidencePreparation frozen =
      prepareCurrentQ0Evidence(p, geometry_data::kProvenance, true);
  CHECK(frozen.ready());

  for (uint8_t i = 0; i < 12; ++i) {
    const JointIdentity& id = s.candidates()[i].evidence.identity;
    const JointTransform* t = transformFor(fresh, id);
    const JointTransform* f = transformFor(frozen, id);
    CHECK(t != nullptr && f != nullptr);
    if (t == nullptr || f == nullptr) continue;
    CHECK_EQ(t->q0_tick, firstTick(i));            // the capture's tick
    CHECK(t->q0_tick != f->q0_tick);               // not the CR2-C tick
    CHECK(t->state == EvidenceState::PROMOTED);
    CHECK(t->origin == CalibrationOrigin::LIVE_SESSION);
    CHECK(t->present);
    CHECK(t->geometry == p.provenanceTag());
  }
}

void test_twelve_of_twelve_admitted_and_replaced() {
  g_case = "12/12 admission and replacement";
  const CalibrationGeometryProfile p = boundProfile();
  const GeometryProvenanceTag tag = p.provenanceTag();

  CalibrationQ0CaptureSession first;
  CHECK(runCapture(first, kFirstBase, kFirstStep));
  CalibrationQ0CaptureSession second;
  CHECK(second.start(goodConfig()));
  second.fail(Q0CaptureFailure::EXTERNAL_ABORT);
  second.reset();
  CHECK(runCapture(second, kSecondBase, kSecondStep));
  CHECK(second.status().capture_session_id != first.status().capture_session_id);

  JointTransformTable table;
  CHECK(!freshQ0CaptureIsPromoted(first.freshCapture(), table, tag));  // nothing admitted yet

  const Q0EvidencePreparation p1 =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, first.freshCapture(), true);
  CHECK(p1.ready());
  CHECK(admitAll(&table, p1));
  CHECK_EQ(table.size(), 12);
  CHECK(freshQ0CaptureIsPromoted(first.freshCapture(), table, tag));
  // The newer capture is complete but was never promoted: not current.
  CHECK(!freshQ0CaptureIsPromoted(second.freshCapture(), table, tag));

  for (uint8_t i = 0; i < 12; ++i) {
    const JointTransform* t = table.find(first.candidates()[i].evidence.identity, tag);
    CHECK(t != nullptr && t->q0_tick == firstTick(i));
  }

  // A second accepted promotion replaces the previous entry of every identity.
  const Q0EvidencePreparation p2 =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, second.freshCapture(), true);
  CHECK(p2.ready());
  CHECK(admitAll(&table, p2));
  CHECK_EQ(table.size(), 12);  // replaced in place, not appended
  CHECK(freshQ0CaptureIsPromoted(second.freshCapture(), table, tag));
  CHECK(!freshQ0CaptureIsPromoted(first.freshCapture(), table, tag));  // superseded
  for (uint8_t i = 0; i < 12; ++i) {
    const JointTransform* t = table.find(second.candidates()[i].evidence.identity, tag);
    CHECK(t != nullptr && t->q0_tick == secondTick(i));
    CHECK(t == nullptr || t->q0_tick != firstTick(i));
  }
}

void test_promoted_check_is_strict() {
  g_case = "freshQ0CaptureIsPromoted strictness";
  const CalibrationGeometryProfile p = boundProfile();
  const GeometryProvenanceTag tag = p.provenanceTag();
  CalibrationQ0CaptureSession s;
  CHECK(runCapture(s, kFirstBase, kFirstStep));
  const Q0EvidencePreparation prepared =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, s.freshCapture(), true);
  CHECK(prepared.ready());

  JointTransformTable full;
  CHECK(admitAll(&full, prepared));
  CHECK(freshQ0CaptureIsPromoted(s.freshCapture(), full, tag));

  CHECK(!freshQ0CaptureIsPromoted(s.freshCapture(), full, kNoGeometryProvenance));
  CHECK(!freshQ0CaptureIsPromoted(s.freshCapture(), full, tag + 1));  // another geometry

  FreshQ0Capture incomplete = s.freshCapture();
  incomplete.complete = false;
  CHECK(!freshQ0CaptureIsPromoted(incomplete, full, tag));
  FreshQ0Capture eleven = s.freshCapture();
  eleven.candidate_count = 11;
  CHECK(!freshQ0CaptureIsPromoted(eleven, full, tag));
  FreshQ0Capture no_candidates = s.freshCapture();
  no_candidates.candidates = nullptr;
  CHECK(!freshQ0CaptureIsPromoted(no_candidates, full, tag));

  JointTransformTable partial;  // 11 of 12 admitted
  for (uint8_t i = 0; i < 11; ++i) CHECK(partial.admit(prepared.transforms[i]));
  CHECK(!freshQ0CaptureIsPromoted(s.freshCapture(), partial, tag));

  JointTransformTable off_by_one;  // one joint holds a different q0
  for (uint8_t i = 0; i < 12; ++i) {
    JointTransform t = prepared.transforms[i];
    if (i == 7) t.q0_tick = static_cast<uint16_t>(t.q0_tick + 1);
    CHECK(off_by_one.admit(t));
  }
  CHECK(!freshQ0CaptureIsPromoted(s.freshCapture(), off_by_one, tag));
}

void test_stale_or_wrong_state_refuses() {
  g_case = "refusals";
  const CalibrationGeometryProfile p = boundProfile();
  CalibrationQ0CaptureSession s;
  CHECK(runCapture(s, kFirstBase, kFirstStep));
  const FreshQ0Capture good = s.freshCapture();

  auto refused = [&](const Q0EvidencePreparation& r, Q0EvidencePreparationStatus want) {
    CHECK(r.status == want);
    CHECK(!r.ready());
    CHECK_EQ(r.transform_count, 0);
  };

  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, good, false),
          Q0EvidencePreparationStatus::REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED);

  CalibrationQ0CaptureSession idle;
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, idle.freshCapture(), true),
          Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE);
  CalibrationQ0CaptureSession sampling;
  CHECK(enterSampling(sampling));
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, sampling.freshCapture(), true),
          Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE);
  CalibrationQ0CaptureSession failed;
  CHECK(enterSampling(failed));
  failed.fail(Q0CaptureFailure::EXTERNAL_ABORT);
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, failed.freshCapture(), true),
          Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE);

  FreshQ0Capture v = good;
  v.candidates = nullptr;
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, v, true),
          Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE);
  v = good;
  v.candidate_count = 11;
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, v, true),
          Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE);
  v = good;
  v.capture_session_id = 0;
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, v, true),
          Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE);
  v = good;
  v.population_pass = false;
  refused(prepareFreshQ0Evidence(p, geometry_data::kProvenance, v, true),
          Q0EvidencePreparationStatus::REJECT_CAPTURE_POPULATION_NOT_PASS);

  // Geometry: a changed expected model, and an unbound profile.
  GeometryProvenance changed = geometry_data::kProvenance;
  changed.urdf_sha256[0] = changed.urdf_sha256[0] == 'a' ? 'b' : 'a';
  refused(prepareFreshQ0Evidence(p, changed, good, true),
          Q0EvidencePreparationStatus::REJECT_SOURCE_GEOMETRY);
  CalibrationGeometryProfile unbound;
  refused(prepareFreshQ0Evidence(unbound, geometry_data::kProvenance, good, true),
          Q0EvidencePreparationStatus::REJECT_SOURCE_GEOMETRY);
}

void test_one_bad_candidate_refuses_the_whole_set() {
  g_case = "one bad candidate";
  const CalibrationGeometryProfile p = boundProfile();
  CalibrationQ0CaptureSession s;
  CHECK(runCapture(s, kFirstBase, kFirstStep));
  const FreshQ0Capture good = s.freshCapture();

  auto with = [&](auto&& mutate, Q0EvidencePreparationStatus want, const char* label) {
    g_case = label;
    Q0BootstrapCandidate mod[12];
    std::memcpy(mod, s.candidates(), sizeof(mod));
    mutate(mod[5]);
    FreshQ0Capture v = good;
    v.candidates = mod;
    const Q0EvidencePreparation r =
        prepareFreshQ0Evidence(p, geometry_data::kProvenance, v, true);
    CHECK(r.status == want);
    CHECK(!r.ready());
    CHECK_EQ(r.transform_count, 0);      // atomic: nothing of the valid 5 leaks out
    CHECK_EQ(r.failed_record_index, 5);
  };

  with([&](Q0BootstrapCandidate& c) { c.evidence.identity = s.candidates()[4].evidence.identity; },
       Q0EvidencePreparationStatus::REJECT_RECORD_DUPLICATE, "wrong identity (duplicate slot)");
  with([&](Q0BootstrapCandidate& c) { c.bus_id = 99; },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "wrong bus binding");
  with([&](Q0BootstrapCandidate& c) { c.geometry = c.geometry + 1; },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "wrong geometry tag");
  with([&](Q0BootstrapCandidate& c) { c.capture_session_id += 1; },
       Q0EvidencePreparationStatus::REJECT_PROMOTION, "candidate from another capture session");
  with([&](Q0BootstrapCandidate& c) { c.stability_spread_ticks = 17; },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "unstable (spread above the CR3 budget)");
  with([&](Q0BootstrapCandidate& c) { c.sample_count = 8; },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "too few samples");
  with([&](Q0BootstrapCandidate& c) {
         c.evidence.tick = 2048 + 81;
         c.evidence.shift_from_digital_home_ticks = 81;
       },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "implausible q0 (beyond 80 ticks)");
  with([&](Q0BootstrapCandidate& c) { c.status = Q0BootstrapStatus::REJECT_UNSTABLE; },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "candidate never reached CANDIDATE");
  with([&](Q0BootstrapCandidate& c) { c.evidence.accepted_by_gate = true; },
       Q0EvidencePreparationStatus::REJECT_CANDIDATE, "pre-accepted evidence is not a candidate");
}

void test_full_leg_plan_consumes_the_new_q0() {
  g_case = "full leg plan consumes new q0";
  const CalibrationGeometryProfile p = boundProfile();
  const GeometryProvenanceTag tag = p.provenanceTag();

  CalibrationQ0CaptureSession first;
  CHECK(runCapture(first, kFirstBase, kFirstStep));
  CalibrationQ0CaptureSession second;
  CHECK(second.start(goodConfig()));
  second.fail(Q0CaptureFailure::EXTERNAL_ABORT);
  second.reset();
  CHECK(runCapture(second, kSecondBase, kSecondStep));

  const Q0EvidencePreparation p1 =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, first.freshCapture(), true);
  const Q0EvidencePreparation p2 =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, second.freshCapture(), true);
  const Q0EvidencePreparation frozen =
      prepareCurrentQ0Evidence(p, geometry_data::kProvenance, true);
  CHECK(p1.ready() && p2.ready() && frozen.ready());

  JointTransformTable empty;
  JointTransformTable t_first, t_second, t_frozen;
  CHECK(admitAll(&t_first, p1));
  CHECK(admitAll(&t_second, p2));
  CHECK(admitAll(&t_frozen, frozen));

  const Leg legs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
  for (Leg leg : legs) {
    FullLegPlan none{};
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, empty, leg, &none) ==
          FullLegPlanStatus::REJECT_NO_TRANSFORM);

    FullLegPlan a{}, b{}, c{};
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, t_first, leg, &a) ==
          FullLegPlanStatus::OK);
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, t_second, leg, &b) ==
          FullLegPlanStatus::OK);
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, t_frozen, leg, &c) ==
          FullLegPlanStatus::OK);

    // The URDF-side request does not depend on q0; the RAW target it resolves to
    // does, and must move by exactly the change of q0 of the UPPER joint.
    const JointTransform* ta = t_first.find(a.upper.identity, tag);
    const JointTransform* tb = t_second.find(b.upper.identity, tag);
    const JointTransform* tc = t_frozen.find(c.upper.identity, tag);
    CHECK(ta != nullptr && tb != nullptr && tc != nullptr);
    if (ta == nullptr || tb == nullptr || tc == nullptr) continue;

    // Every raw point of both search corridors (canonical contact, entry,
    // guard) moves by exactly the change of q0 - the plan is never silently
    // on the frozen values.
    const int32_t d_ab = (int32_t)tb->q0_tick - (int32_t)ta->q0_tick;
    const int32_t d_ac = (int32_t)ta->q0_tick - (int32_t)tc->q0_tick;
    for (int side = 0; side < 2; ++side) {
      const actuator::CalibrationSearchCorridor& ca = side ? a.request.max_search : a.request.min_search;
      const actuator::CalibrationSearchCorridor& cb = side ? b.request.max_search : b.request.min_search;
      const actuator::CalibrationSearchCorridor& cc = side ? c.request.max_search : c.request.min_search;
      CHECK_EQ((long)cb.contact_tick - (long)ca.contact_tick, (long)d_ab);
      CHECK_EQ((long)ca.contact_tick - (long)cc.contact_tick, (long)d_ac);
      CHECK_EQ((long)cb.guard_tick - (long)ca.guard_tick, (long)d_ab);
      CHECK_EQ((long)cb.entry_tick - (long)ca.entry_tick, (long)d_ab);
      CHECK_EQ(ca.home_tick, ta->q0_tick);
      CHECK(ca.contact_tick != cc.contact_tick);
    }

    if (a.request.auxiliary_required) {
      const JointTransform* aux = t_first.find(a.request.auxiliary_joint, tag);
      CHECK(aux != nullptr);
      if (aux != nullptr) {
        // The parked auxiliary is also resolved against the fresh q0.
        const JointTransform* aux_frozen = t_frozen.find(a.request.auxiliary_joint, tag);
        CHECK(aux_frozen != nullptr && aux->q0_tick != aux_frozen->q0_tick);
      }
    }
  }
}

void test_status_names() {
  g_case = "status names";
  CHECK(std::strcmp(toString(Q0EvidencePreparationStatus::REJECT_FRESH_CAPTURE_NOT_COMPLETE),
                    "REJECT_FRESH_CAPTURE_NOT_COMPLETE") == 0);
}

}  // namespace

int main() {
  test_capture_view_lifecycle();
  test_fresh_capture_is_promoted_not_frozen();
  test_twelve_of_twelve_admitted_and_replaced();
  test_promoted_check_is_strict();
  test_stale_or_wrong_state_refuses();
  test_one_bad_candidate_refuses_the_whole_set();
  test_full_leg_plan_consumes_the_new_q0();
  test_status_names();

  std::printf("test_cr3_q0_fresh_promotion: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
