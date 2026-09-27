#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationQ0Bootstrap.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"

using namespace matdog::actuator;
using namespace matdog::calibration;

static int g_checks = 0;
static int g_failures = 0;

static void check(bool ok, const char* label) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("FAIL: %s\n", label);
  }
}

static CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&geometry_data::kProvenance,
               geometry_data::kJoints, geometry_data::kJointCount,
               geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return profile;
}

static JointIdentity identity(Leg leg, JointKind joint, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = joint;
  setPhysicalUnit(&id, unit);
  return id;
}

static LegPopulationEvidence currentPopulation() {
  LegPopulationEvidence e{};
  e.evaluated = true;
  e.origin = CalibrationOrigin::LIVE_SESSION;
  e.observed_mask = static_cast<uint16_t>((1u << kLegServoSlotCount) - 1u);
  e.unexpected_count = 0;
  e.session_ms = 1234;
  return e;
}

static Q0BootstrapRequest goodRequest() {
  Q0BootstrapRequest r{};
  r.identity = identity(Leg::LF, JointKind::UPPER, "ELR01");
  r.bus_id = 12;
  r.capture_session_id = 77;
  r.nominal_zero_pose_confirmed = true;
  r.max_stability_spread_ticks = 2;
  return r;
}

static void fill(Q0CaptureSample* s, uint8_t n, int32_t raw) {
  for (uint8_t i = 0; i < n; ++i) {
    s[i].read_ok = true;
    s[i].raw_tick = raw;
    s[i].torque_enable = 0;
  }
}

static void test_stable_manual_capture_becomes_candidate_only() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0CaptureSample s[5]{};
  fill(s, 5, 2050);
  s[0].raw_tick = 2049;
  s[4].raw_tick = 2051;

  const auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), goodRequest(), s, 5);

  check(r.status == Q0BootstrapStatus::CANDIDATE, "stable capture candidate");
  check(r.geometry == geometryProvenanceTag(geometry_data::kProvenance),
        "candidate geometry bound");
  check(r.bus_id == 12, "candidate bus metadata");
  check(r.capture_session_id == 77, "candidate session bound");
  check(r.sample_count == 5, "candidate sample count");
  check(r.stability_spread_ticks == 1, "candidate spread");
  check(r.evidence.measured, "q0 measured");
  check(r.evidence.estimator == Q0Estimator::MANUAL_ZERO_POSE, "manual estimator");
  check(r.evidence.state == EvidenceState::CANDIDATE, "candidate state only");
  check(r.evidence.origin == CalibrationOrigin::LIVE_SESSION, "live origin");
  check(r.evidence.tick == 2050, "median tick");
  check(!r.evidence.accepted_by_gate, "not accepted");
  check(!q0MayBeAppliedTo(r.evidence, r.evidence.identity),
        "candidate cannot be operational");
}

static void test_2048_is_not_a_gate() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0CaptureSample s[3]{};
  fill(s, 3, 3000);
  const auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), goodRequest(), s, 3);
  check(r.status == Q0BootstrapStatus::CANDIDATE, "stable q0 far from 2048 still recorded");
  check(r.evidence.tick == 3000, "q0 is measured not forced to 2048");
  check(r.evidence.shift_from_digital_home_ticks == 952, "2048 distance is diagnostic only");
}

static void test_wrap_boundary_uses_encoder_measurement_semantics() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0CaptureSample s[3] = {
      {true, 4095, 0}, {true, 0, 0}, {true, 1, 0},
  };
  const auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), goodRequest(), s, 3);
  check(r.status == Q0BootstrapStatus::CANDIDATE, "wrap sample accepted");
  check(r.evidence.tick == 0, "circular median at zero");
  check(r.stability_spread_ticks == 1, "wrap spread one tick");
}

static void test_population_and_geometry_are_mandatory() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0CaptureSample s[3]{};
  fill(s, 3, 2050);

  LegPopulationEvidence historical = currentPopulation();
  historical.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, historical, goodRequest(), s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_POPULATION_NOT_CURRENT,
        "historical population refused");

  CalibrationGeometryProfile unbound;
  r = buildQ0BootstrapCandidate(
      unbound, geometry_data::kProvenance, currentPopulation(), goodRequest(), s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_GEOMETRY_UNBOUND,
        "unbound geometry refused");

  GeometryProvenance wrong = geometry_data::kProvenance;
  wrong.urdf_sha256[0] = (wrong.urdf_sha256[0] == 'a') ? 'b' : 'a';
  r = buildQ0BootstrapCandidate(
      profile, wrong, currentPopulation(), goodRequest(), s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_GEOMETRY_PROVENANCE,
        "wrong geometry provenance refused");
}

static void test_identity_bus_and_pose_are_mandatory() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0CaptureSample s[3]{};
  fill(s, 3, 2050);

  Q0BootstrapRequest q = goodRequest();
  q.identity = identity(Leg::LF, JointKind::UPPER, "WRONG");
  auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_JOINT_NOT_IN_PROFILE,
        "wrong physical unit refused");

  q = goodRequest();
  q.bus_id = 11;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_BUS_ID_MISMATCH,
        "wrong transport binding refused");

  q = goodRequest();
  q.nominal_zero_pose_confirmed = false;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_POSE_NOT_CONFIRMED,
        "manual pose confirmation required");

  q = goodRequest();
  q.capture_session_id = 0;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_INVALID_REQUEST,
        "zero capture session refused");
}

static void test_sample_set_fails_closed() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0BootstrapRequest q = goodRequest();
  Q0CaptureSample s[3]{};
  fill(s, 3, 2050);

  auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 2);
  check(r.status == Q0BootstrapStatus::REJECT_SAMPLE_COUNT, "too few samples refused");

  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, nullptr, 3);
  check(r.status == Q0BootstrapStatus::REJECT_SAMPLE_COUNT, "null sample set refused");

  fill(s, 3, 2050);
  s[1].read_ok = false;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_SAMPLE_READ, "failed read refused");

  fill(s, 3, 2050);
  s[1].torque_enable = 1;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_TORQUE_NOT_OFF, "torque on refused");

  fill(s, 3, 2050);
  s[1].raw_tick = 4096;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_RAW_DOMAIN, "raw 4096 refused");

  fill(s, 3, 2000);
  s[1].raw_tick = 2005;
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_UNSTABLE, "unstable pose refused");
  check(r.stability_spread_ticks == 5, "unstable spread preserved");

  q = goodRequest();
  q.max_stability_spread_ticks = 2048;
  fill(s, 3, 2050);
  r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 3);
  check(r.status == Q0BootstrapStatus::REJECT_INVALID_REQUEST,
        "meaningless half-turn stability budget refused");
}

static void test_even_sample_median_matches_historical_read_only_convention() {
  CalibrationGeometryProfile profile = boundProfile();
  Q0BootstrapRequest q = goodRequest();
  q.max_stability_spread_ticks = 10;
  Q0CaptureSample s[4] = {
      {true, 2000, 0}, {true, 2001, 0}, {true, 2002, 0}, {true, 2003, 0},
  };
  const auto r = buildQ0BootstrapCandidate(
      profile, geometry_data::kProvenance, currentPopulation(), q, s, 4);
  check(r.status == Q0BootstrapStatus::CANDIDATE, "even capture candidate");
  check(r.evidence.tick == 2002, "upper median convention preserved");
}

static void test_status_strings_are_total() {
  for (uint8_t i = 0; i <= static_cast<uint8_t>(Q0BootstrapStatus::CANDIDATE); ++i) {
    check(std::strcmp(toString(static_cast<Q0BootstrapStatus>(i)), "UNKNOWN") != 0,
          "q0 bootstrap status string total");
  }
  check(std::strcmp(toString(Q0Estimator::MANUAL_ZERO_POSE), "MANUAL_ZERO_POSE") == 0,
        "manual estimator string");
}

int main() {
  std::printf("MATDOG CR2 read-only q0 bootstrap offline tests\n");
  test_stable_manual_capture_becomes_candidate_only();
  test_2048_is_not_a_gate();
  test_wrap_boundary_uses_encoder_measurement_semantics();
  test_population_and_geometry_are_mandatory();
  test_identity_bus_and_pose_are_mandatory();
  test_sample_set_fails_closed();
  test_even_sample_median_matches_historical_read_only_convention();
  test_status_strings_are_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("CALIBRATION_Q0_BOOTSTRAP_TESTS = FAIL\n");
    return 1;
  }
  std::printf("CALIBRATION_Q0_BOOTSTRAP_TESTS = PASS\n");
  return 0;
}
