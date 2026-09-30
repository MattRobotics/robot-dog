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
#include "../../src/actuator/CalibrationSequencePlanData.h"
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

const CalibrationSequencePlan& kSeqPlan = sequence_plan_data::kPlan;

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
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, empty, &kSeqPlan, leg, &none) ==
          FullLegPlanStatus::REJECT_NO_TRANSFORM);

    FullLegPlan a{}, b{}, c{};
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, t_first, &kSeqPlan, leg, &a) ==
          FullLegPlanStatus::OK);
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, t_second, &kSeqPlan, leg, &b) ==
          FullLegPlanStatus::OK);
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, t_frozen, &kSeqPlan, leg, &c) ==
          FullLegPlanStatus::OK);

    // The URDF-side request does not depend on q0; the RAW targets it resolves
    // to do, and every one must move by exactly the change of q0 of its joint.
    for (int k = 0; k < (int)kJointKindCount; ++k) {
      const JointTransform* ta = t_first.find(a.request.joint[k].identity, tag);
      const JointTransform* tb = t_second.find(b.request.joint[k].identity, tag);
      const JointTransform* tc = t_frozen.find(c.request.joint[k].identity, tag);
      CHECK(ta != nullptr && tb != nullptr && tc != nullptr);
      if (ta == nullptr || tb == nullptr || tc == nullptr) continue;
      CHECK_EQ((long)a.request.joint[k].q0_tick, (long)ta->q0_tick);
      CHECK_EQ((long)b.request.joint[k].q0_tick, (long)tb->q0_tick);

      // Every raw point of all six search corridors (canonical contact, entry,
      // guard) moves by exactly the change of q0 - the plan is never silently
      // on the frozen values.
      const int32_t d_ab = (int32_t)tb->q0_tick - (int32_t)ta->q0_tick;
      const int32_t d_ac = (int32_t)ta->q0_tick - (int32_t)tc->q0_tick;
      for (int side = 0; side < 2; ++side) {
        const actuator::CalibrationSearchCorridor& ca = a.request.corridor[k][side];
        const actuator::CalibrationSearchCorridor& cb = b.request.corridor[k][side];
        const actuator::CalibrationSearchCorridor& cc = c.request.corridor[k][side];
        CHECK_EQ((long)cb.contact_tick - (long)ca.contact_tick, (long)d_ab);
        CHECK_EQ((long)ca.contact_tick - (long)cc.contact_tick, (long)d_ac);
        CHECK_EQ((long)cb.guard_tick - (long)ca.guard_tick, (long)d_ab);
        CHECK_EQ((long)cb.entry_tick - (long)ca.entry_tick, (long)d_ab);
        CHECK_EQ(ca.home_tick, ta->q0_tick);
        CHECK(ca.contact_tick != cc.contact_tick);
      }
    }
    // The prerequisite poses of the UPPER/LOWER move with their joint's q0 too.
    {
      const JointTransform* ua = t_first.find(a.upper.identity, tag);
      const JointTransform* ub = t_second.find(b.upper.identity, tag);
      const JointTransform* la = t_first.find(a.lower.identity, tag);
      const JointTransform* lb = t_second.find(b.lower.identity, tag);
      CHECK(ua && ub && la && lb);
      if (ua && ub && la && lb) {
        const long du = (long)ub->q0_tick - (long)ua->q0_tick;
        const long dl = (long)lb->q0_tick - (long)la->q0_tick;
        CHECK_EQ((long)b.request.upper_for_lower_tick - (long)a.request.upper_for_lower_tick, du);
        CHECK_EQ((long)b.request.upper_for_hip_min_tick - (long)a.request.upper_for_hip_min_tick, du);
        CHECK_EQ((long)b.request.upper_for_hip_max_tick - (long)a.request.upper_for_hip_max_tick, du);
        CHECK_EQ((long)b.request.lower_folded_tick - (long)a.request.lower_folded_tick, dl);
      }
    }

    if (a.request.has_rear_park) {
      const JointTransform* aux = t_first.find(a.request.park.identity, tag);
      CHECK(aux != nullptr);
      if (aux != nullptr) {
        // The parked rear UPPER is also resolved against the fresh q0.
        const JointTransform* aux_frozen = t_frozen.find(a.request.park.identity, tag);
        CHECK(aux_frozen != nullptr && aux->q0_tick != aux_frozen->q0_tick);
        CHECK_EQ((long)a.request.park.q0_tick, (long)aux->q0_tick);
      }
    }
    // All twelve leg joints, each at its fresh q0.
    CHECK_EQ((long)a.request.population_count, 12L);
    for (uint8_t i = 0; i < a.request.population_count; ++i) {
      const JointTransform* t = t_first.find(a.request.population[i].identity, tag);
      CHECK(t != nullptr && t->q0_tick == a.request.population[i].q0_tick);
    }
  }
}

// --- the mechanically recentred installation ------------------------------------
//
// Current MATDOG: every ST3215 was recentred near its raw mid-range (~2048)
// before mounting, PositionOffset = 0, and the mounting puts URDF q=0 NEAR but
// not AT raw 2048. 2048 is a servo/provisioning fact, never a q0. Installation
// A below is such a mounting; installation B is the same robot with every
// joint remounted by its own realistic offset. The calibration only ever
// commands q0 + direction * q, so identical URDF commands move by exactly
// each joint's q0 change, every search corridor is the same geometry
// translated, and every prerequisite pose follows its own joint's q0.

bool runCaptureTicks(CalibrationQ0CaptureSession& s, const int* ticks) {
  if (!enterSampling(s)) return false;
  for (uint8_t pass = 0; pass < kSamples; ++pass) {
    for (uint8_t joint = 0; joint < kLegServoSlotCount; ++joint) {
      Q0ReadRequest req{};
      if (!s.nextReadRequest(&req)) return false;
      Q0ReadObservation obs{};
      obs.bus_id = req.bus_id;
      obs.read_ok = true;
      obs.raw_tick = ticks[joint];
      obs.torque_enable = 0;
      if (!s.recordRead(obs)) return false;
    }
  }
  return s.status().state == Q0CaptureState::COMPLETE;
}

void test_recentred_installation_translation() {
  g_case = "recentred installation: raw targets = fresh q0 + direction * q, nothing else";
  static_assert(kLegServoSlotCount == 12, "one tick per leg joint");
  // Near but never equal to 2048; B = A + a different offset per joint.
  const int kA[12] = {2079, 2106, 2003, 1990, 2086, 2025, 2040, 2037, 2075, 2068, 2094, 2029};
  const int kOffset[12] = {13, -21, 8, -5, 30, -17, 2, -9, 25, -14, 6, -28};
  int kB[12];
  for (int i = 0; i < 12; ++i) {
    CHECK(kA[i] != (int)kServoRawCenter);
    kB[i] = kA[i] + kOffset[i];
    CHECK(kB[i] != (int)kServoRawCenter);
  }
  const CalibrationGeometryProfile p = boundProfile();
  const GeometryProvenanceTag tag = p.provenanceTag();
  CalibrationQ0CaptureSession ca, cb;
  CHECK(runCaptureTicks(ca, kA));
  CHECK(runCaptureTicks(cb, kB));
  const Q0EvidencePreparation pa =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, ca.freshCapture(), true);
  const Q0EvidencePreparation pb =
      prepareFreshQ0Evidence(p, geometry_data::kProvenance, cb.freshCapture(), true);
  const Q0EvidencePreparation frozen = prepareCurrentQ0Evidence(p, geometry_data::kProvenance, true);
  CHECK(pa.ready() && pb.ready() && frozen.ready());
  JointTransformTable ta, tb;
  CHECK(admitAll(&ta, pa));
  CHECK(admitAll(&tb, pb));

  // (1) All twelve joints: the promoted q0 IS the capture (not CR2-C, not
  // 2048), and identical URDF commands differ by exactly the q0 change.
  int joints = 0;
  for (uint8_t i = 0; i < pa.transform_count; ++i) {
    const JointTransform& a = pa.transforms[i];
    const JointTransform* b = transformFor(pb, a.identity);
    const JointTransform* f = transformFor(frozen, a.identity);
    const GeometryJointRecord* g = p.findJoint(a.identity);
    CHECK(b != nullptr && f != nullptr && g != nullptr);
    if (b == nullptr || f == nullptr || g == nullptr) continue;
    ++joints;
    const JointTransform* pa_t = ta.find(a.identity, tag);
    CHECK(pa_t != nullptr && pa_t->q0_tick == a.q0_tick);
    CHECK(a.q0_tick != kServoRawCenter && b->q0_tick != kServoRawCenter);
    const int32_t d = (int32_t)b->q0_tick - (int32_t)a.q0_tick;
    CHECK(d != 0);
    const MicroRad qs[] = {g->urdf_lower, g->urdf_lower / 2, 0, g->urdf_upper / 3,
                           g->urdf_upper / 2, g->urdf_upper};
    for (MicroRad q : qs) {
      uint16_t ra = 0, rb = 0, rf = 0;
      CHECK(resolveUrdfQToRaw(p, geometry_data::kProvenance, a, q, &ra) == TargetResolveStatus::OK);
      CHECK(resolveUrdfQToRaw(p, geometry_data::kProvenance, *b, q, &rb) == TargetResolveStatus::OK);
      CHECK(resolveUrdfQToRaw(p, geometry_data::kProvenance, *f, q, &rf) == TargetResolveStatus::OK);
      CHECK_EQ((long)rb - (long)ra, (long)d);
      CHECK_EQ((long)ra - (long)rf, (long)a.q0_tick - (long)f->q0_tick);
      if (q == 0) CHECK_EQ(ra, a.q0_tick);  // URDF q=0 is the measured q0, not 2048
    }
  }
  CHECK_EQ(joints, 12);

  // (2) all 24 search corridors and (3) every prerequisite pose, per leg.
  int corridors = 0;
  const Leg legs[4] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
  for (Leg leg : legs) {
    FullLegPlan a{}, b{};
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, ta, &kSeqPlan, leg, &a) == FullLegPlanStatus::OK);
    CHECK(resolveFullLegPlan(p, geometry_data::kProvenance, tb, &kSeqPlan, leg, &b) == FullLegPlanStatus::OK);
    for (int k = 0; k < (int)kJointKindCount; ++k) {
      const JointTransform* qa = ta.find(a.request.joint[k].identity, tag);
      const JointTransform* qb = tb.find(b.request.joint[k].identity, tag);
      CHECK(qa != nullptr && qb != nullptr);
      if (qa == nullptr || qb == nullptr) continue;
      const long d = (long)qb->q0_tick - (long)qa->q0_tick;
      for (int side = 0; side < 2; ++side) {
        const CalibrationSearchCorridor& x = a.request.corridor[k][side];
        const CalibrationSearchCorridor& y = b.request.corridor[k][side];
        ++corridors;
        CHECK_EQ(x.home_tick, qa->q0_tick);  // home = the fresh promoted q0
        CHECK_EQ(y.home_tick, qb->q0_tick);
        CHECK_EQ(x.probe_sign, y.probe_sign);
        CHECK_EQ((long)y.contact_tick - (long)x.contact_tick, d);
        CHECK_EQ((long)y.urdf_limit_tick - (long)x.urdf_limit_tick, d);
        CHECK_EQ((long)y.opposite_limit_tick - (long)x.opposite_limit_tick, d);
        CHECK_EQ((long)y.entry_tick - (long)x.entry_tick, d);
        CHECK_EQ((long)y.guard_tick - (long)x.guard_tick, d);
        // The search geometry itself is invariant under the common translation.
        const uint16_t xs[5] = {x.contact_tick, x.urdf_limit_tick, x.opposite_limit_tick,
                                x.entry_tick, x.guard_tick};
        const uint16_t ys[5] = {y.contact_tick, y.urdf_limit_tick, y.opposite_limit_tick,
                                y.entry_tick, y.guard_tick};
        for (int n = 0; n < 5; ++n) CHECK_EQ(searchDepth(x, xs[n]), searchDepth(y, ys[n]));
        CHECK_EQ(searchDepth(x, x.guard_tick) - searchDepth(x, x.urdf_limit_tick), 64);
        CHECK_EQ(searchDepth(x, x.urdf_limit_tick) - searchDepth(x, x.entry_tick), 64);
      }
    }
    const JointTransform* ua = ta.find(a.upper.identity, tag);
    const JointTransform* ub = tb.find(b.upper.identity, tag);
    const JointTransform* la = ta.find(a.lower.identity, tag);
    const JointTransform* lb = tb.find(b.lower.identity, tag);
    CHECK(ua && ub && la && lb);
    if (ua && ub && la && lb) {
      const long du = (long)ub->q0_tick - (long)ua->q0_tick;
      const long dl = (long)lb->q0_tick - (long)la->q0_tick;
      CHECK_EQ(a.request.upper_for_lower_urad, b.request.upper_for_lower_urad);  // same URDF command
      CHECK_EQ((long)b.request.upper_for_lower_tick - (long)a.request.upper_for_lower_tick, du);
      CHECK_EQ((long)b.request.upper_for_hip_min_tick - (long)a.request.upper_for_hip_min_tick, du);
      CHECK_EQ((long)b.request.upper_for_hip_max_tick - (long)a.request.upper_for_hip_max_tick, du);
      CHECK_EQ((long)b.request.lower_folded_tick - (long)a.request.lower_folded_tick, dl);
    }
    CHECK_EQ(a.request.has_rear_park, leg == Leg::LF || leg == Leg::RF);
    if (a.request.has_rear_park) {
      const JointTransform* pka = ta.find(a.request.park.identity, tag);
      const JointTransform* pkb = tb.find(b.request.park.identity, tag);
      CHECK(pka != nullptr && pkb != nullptr);
      if (pka != nullptr && pkb != nullptr) {
        CHECK_EQ(a.request.park.q0_tick, pka->q0_tick);
        CHECK_EQ(a.request.park_target_urad, b.request.park_target_urad);
        CHECK_EQ((long)b.request.park_target_tick - (long)a.request.park_target_tick,
                 (long)pkb->q0_tick - (long)pka->q0_tick);
      }
    }
    // INITIAL RECOVERY targets: every one of the 12 at its own fresh q0.
    CHECK_EQ((long)a.request.population_count, 12L);
    for (uint8_t i = 0; i < a.request.population_count; ++i) {
      const JointTransform* t = ta.find(a.request.population[i].identity, tag);
      CHECK(t != nullptr && t->q0_tick == a.request.population[i].q0_tick);
    }
  }
  CHECK_EQ(corridors, 24);
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
  test_recentred_installation_translation();
  test_twelve_of_twelve_admitted_and_replaced();
  test_promoted_check_is_strict();
  test_stale_or_wrong_state_refuses();
  test_one_bad_candidate_refuses_the_whole_set();
  test_full_leg_plan_consumes_the_new_q0();
  test_status_names();

  std::printf("test_cr3_q0_fresh_promotion: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
