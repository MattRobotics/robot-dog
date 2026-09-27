#include <cstdio>
#include <cstring>

#include "../../src/calibration/CalibrationPopulationEvidence.h"
#include "../../src/servo/ServoProfileData.h"

using namespace matdog::calibration;
using namespace matdog::servo;

static int g_checks = 0;
static int g_failures = 0;

static void check(bool ok, const char* label) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("FAIL: %s\n", label);
  }
}

static CensusResult goodCensus() {
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

static PreflightResult goodPreflight() {
  PreflightResult p{};
  p.complete = true;
  p.joints_evaluated = kLegPreflightCount;
  p.pass_count = kLegPreflightCount;

  for (uint8_t i = 0; i < kLegPreflightCount; ++i) {
    const CanonicalServo* canonical = legServoAt(i);
    check(canonical != nullptr, "canonical leg slot exists");
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
    r.profile_mismatch_count = 0;
    r.result = JointPreflightResult::PASS;
  }
  return p;
}

static PopulationEvidenceBuildContext currentContext() {
  PopulationEvidenceBuildContext c{};
  c.current_observation_bundle = true;
  c.session_ms = 123456;
  return c;
}

static void test_good_bundle_produces_current_pass() {
  const auto r =
      buildCurrentLegPopulationEvidence(goodCensus(), goodPreflight(), currentContext());
  check(r.status == PopulationEvidenceBuildStatus::PASS, "good bundle status PASS");
  check(r.qualified_slots == kLegServoSlotCount, "12 qualified slots");
  check(r.rejected_slots == 0, "no rejected slots");
  check(observedLegSlotCount(r.evidence) == kLegServoSlotCount, "formal mask is 12/12");
  check(r.evidence.evaluated, "formal evidence evaluated");
  check(r.evidence.origin == CalibrationOrigin::LIVE_SESSION, "formal evidence LIVE_SESSION");
  check(r.evidence.session_ms == 123456u, "formal evidence session provenance");
  check(r.evidence.unexpected_count == 0, "formal evidence no anomaly");
  check(populationIsCurrentPass(r.evidence), "domain accepts formal current evidence");
}

static void test_cached_preflight_is_not_current_evidence() {
  PopulationEvidenceBuildContext stale{};
  stale.session_ms = 999;
  const auto r =
      buildCurrentLegPopulationEvidence(goodCensus(), goodPreflight(), stale);
  check(r.status == PopulationEvidenceBuildStatus::REJECT_SOURCE_NOT_CURRENT,
        "cached source refused");
  check(!r.evidence.evaluated, "cached source produces no evaluated evidence");
  check(r.evidence.origin == CalibrationOrigin::NONE, "cached source has no live origin");
}

static void test_census_range_is_mandatory() {
  CensusResult c = goodCensus();
  c.scan_hi = 43;
  c.not_probed = 5;
  c.verdict = CensusVerdict::RANGE_INCOMPLETE;
  const auto r =
      buildCurrentLegPopulationEvidence(c, goodPreflight(), currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_CENSUS_INCOMPLETE,
        "short census refused");
  check(!populationIsCurrentPass(r.evidence), "short census cannot current-pass");
}

static void test_neck_is_not_a_leg_slot() {
  CensusResult c = goodCensus();
  c.present_expected--;
  c.missing_expected = 1;
  c.missing_ids[0] = 51;
  c.missing_id_count = 1;
  c.verdict = CensusVerdict::PROFILE_MISMATCH;
  const auto r =
      buildCurrentLegPopulationEvidence(c, goodPreflight(), currentContext());
  check(r.status == PopulationEvidenceBuildStatus::PASS,
        "missing neck does not fabricate a leg H1 failure");
  check(populationIsCurrentPass(r.evidence), "leg evidence still current-pass");
}

static void test_missing_leg_is_refused() {
  CensusResult c = goodCensus();
  c.present_expected--;
  c.missing_expected = 1;
  c.missing_ids[0] = 11;
  c.missing_id_count = 1;
  c.verdict = CensusVerdict::PROFILE_MISMATCH;
  const auto r =
      buildCurrentLegPopulationEvidence(c, goodPreflight(), currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_LEG_MISSING_IN_CENSUS,
        "missing leg refused");
  check(!populationIsCurrentPass(r.evidence), "missing leg cannot current-pass");
}

static void test_bus_anomalies_are_refused() {
  CensusResult c = goodCensus();
  c.unexpected_id = 1;
  c.unexpected_ids[0] = 60;
  c.unexpected_id_count = 1;
  c.verdict = CensusVerdict::PROFILE_MISMATCH;
  auto r = buildCurrentLegPopulationEvidence(c, goodPreflight(), currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_BUS_ANOMALY,
        "unexpected responder refused");
  check(r.evidence.unexpected_count == 1, "unexpected responder preserved in evidence");
  check(!populationIsCurrentPass(r.evidence), "anomalous evidence cannot current-pass");

  c = goodCensus();
  c.absent_by_design--;
  c.absent_by_design_present = 1;
  c.absent_by_design_present_ids[0] = 52;
  c.absent_by_design_present_id_count = 1;
  c.verdict = CensusVerdict::PROFILE_MISMATCH;
  r = buildCurrentLegPopulationEvidence(c, goodPreflight(), currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_BUS_ANOMALY,
        "absent-by-design responder refused");
}

static void test_preflight_must_be_complete() {
  PreflightResult p = goodPreflight();
  p.complete = false;
  const auto r =
      buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_PREFLIGHT_INCOMPLETE,
        "incomplete preflight refused");
}

static void test_each_slot_must_be_formally_qualified() {
  PreflightResult p = goodPreflight();
  p.joints[0].torque_enable = 1;
  auto r = buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION,
        "torque-on slot refused");
  check(r.qualified_slots == kLegServoSlotCount - 1, "torque-on slot not qualified");
  check(!populationIsCurrentPass(r.evidence), "torque-on bundle cannot current-pass");

  p = goodPreflight();
  p.joints[3].present_position = 4096;
  r = buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION,
        "out-of-domain raw position refused");

  p = goodPreflight();
  p.joints[4].profile = ProfileVerdict::MISMATCH;
  p.joints[4].profile_mismatch_count = 1;
  r = buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION,
        "profile mismatch refused");

  p = goodPreflight();
  p.joints[7].position_offset = 3;
  r = buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION,
        "nonzero offset refused");
}

static void test_semantic_identity_and_uniqueness_are_required() {
  PreflightResult p = goodPreflight();
  p.joints[2].expected_physical_unit = "WRONG";
  auto r = buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_IDENTITY_MISMATCH,
        "physical-unit configuration mismatch refused");

  p = goodPreflight();
  p.joints[1] = p.joints[0];
  r = buildCurrentLegPopulationEvidence(goodCensus(), p, currentContext());
  check(r.status == PopulationEvidenceBuildStatus::REJECT_DUPLICATE_SLOT,
        "duplicate semantic slot refused");
}

static void test_status_strings_are_total() {
  for (uint8_t i = 0; i <= static_cast<uint8_t>(PopulationEvidenceBuildStatus::PASS); ++i) {
    check(std::strcmp(toString(static_cast<PopulationEvidenceBuildStatus>(i)), "UNKNOWN") != 0,
          "status string is total");
  }
}

int main() {
  std::printf("MATDOG formal calibration population evidence offline tests\n");
  test_good_bundle_produces_current_pass();
  test_cached_preflight_is_not_current_evidence();
  test_census_range_is_mandatory();
  test_neck_is_not_a_leg_slot();
  test_missing_leg_is_refused();
  test_bus_anomalies_are_refused();
  test_preflight_must_be_complete();
  test_each_slot_must_be_formally_qualified();
  test_semantic_identity_and_uniqueness_are_required();
  test_status_strings_are_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("CALIBRATION_POPULATION_EVIDENCE_TESTS = FAIL\n");
    return 1;
  }
  std::printf("CALIBRATION_POPULATION_EVIDENCE_TESTS = PASS\n");
  return 0;
}
