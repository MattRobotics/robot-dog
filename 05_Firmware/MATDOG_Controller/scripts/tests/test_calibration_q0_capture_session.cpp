#include <cstdio>
#include <cstring>

#include "../../src/calibration/CalibrationQ0CaptureSession.h"
#include "../../src/servo/ServoProfileData.h"

using namespace matdog::actuator;
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

static Q0CaptureConfig goodConfig(uint8_t samples = 3, uint16_t spread = 2) {
  Q0CaptureConfig c{};
  c.samples_per_joint = samples;
  c.stability_budget_specified = true;
  c.max_stability_spread_ticks = spread;
  c.nominal_zero_pose_confirmed = true;
  c.started_at_ms = 4242;
  return c;
}

static bool enterSampling(CalibrationQ0CaptureSession& s,
                          CensusResult census = goodCensus(),
                          PreflightResult preflight = goodPreflight()) {
  if (!s.start(goodConfig())) return false;
  if (!s.markCensusStarted()) return false;
  if (!s.submitCensus(census)) return false;
  if (!s.markPreflightStarted()) return false;
  return s.submitPreflight(preflight);
}

static void test_good_session_is_round_robin_and_candidate_only() {
  CalibrationQ0CaptureSession s;
  check(enterSampling(s), "enter sampling");
  check(s.status().state == Q0CaptureState::SAMPLING, "sampling state");
  check(s.populationResult().status == PopulationEvidenceBuildStatus::PASS,
        "formal population PASS");
  check(populationIsCurrentPass(s.populationResult().evidence),
        "formal population current pass");

  const uint32_t session_id = s.status().capture_session_id;
  check(session_id != 0, "capture session id assigned");

  for (uint8_t pass = 0; pass < 3; ++pass) {
    for (uint8_t joint = 0; joint < kLegServoSlotCount; ++joint) {
      Q0ReadRequest req{};
      check(s.nextReadRequest(&req), "read request available");
      const CanonicalServo* canonical = legServoAt(joint);
      check(canonical != nullptr, "canonical joint exists");
      if (canonical == nullptr) continue;
      check(req.valid, "read request valid");
      check(req.bus_id == canonical->bus_id, "round-robin bus order");
      check(req.joint_index == joint, "round-robin joint index");
      check(req.sample_pass == pass, "round-robin sample pass");

      const int32_t delta = static_cast<int32_t>(pass) - 1;
      Q0ReadObservation obs{};
      obs.bus_id = req.bus_id;
      obs.read_ok = true;
      obs.raw_tick = 3000 + joint + delta;
      obs.torque_enable = 0;
      check(s.recordRead(obs), "record read");
    }
  }

  check(s.status().state == Q0CaptureState::COMPLETE, "capture complete");
  check(s.status().failure == Q0CaptureFailure::NONE, "no capture failure");
  check(s.status().candidates_complete == kLegServoSlotCount, "12 candidates complete");
  check(s.status().completed_sample_passes == 3, "three round-robin passes complete");

  const Q0BootstrapCandidate* q0 = s.candidates();
  for (uint8_t i = 0; i < kLegServoSlotCount; ++i) {
    check(q0[i].status == Q0BootstrapStatus::CANDIDATE, "candidate status");
    check(q0[i].capture_session_id == session_id, "candidate session provenance");
    check(q0[i].sample_count == 3, "candidate sample count");
    check(q0[i].stability_spread_ticks == 1, "candidate spread");
    check(q0[i].evidence.estimator == Q0Estimator::MANUAL_ZERO_POSE,
          "manual zero estimator");
    check(q0[i].evidence.state == EvidenceState::CANDIDATE, "candidate evidence state");
    check(!q0[i].evidence.accepted_by_gate, "candidate not accepted");
    check(!q0MayBeAppliedTo(q0[i].evidence, q0[i].evidence.identity),
          "candidate not operational");
    check(q0[i].evidence.tick == static_cast<uint16_t>(3000 + i),
          "measured q0 not forced to 2048");
  }
}

static void test_invalid_config_fails_before_sources() {
  CalibrationQ0CaptureSession s;
  Q0CaptureConfig c = goodConfig();
  c.nominal_zero_pose_confirmed = false;
  check(!s.start(c), "missing pose confirmation refused");
  check(s.status().state == Q0CaptureState::FAILED, "invalid config failed state");
  check(s.status().failure == Q0CaptureFailure::INVALID_CONFIG, "invalid config reason");

  c = goodConfig();
  c.stability_budget_specified = false;
  check(!s.start(c), "implicit stability budget refused");

  c = goodConfig(2, 2);
  check(!s.start(c), "too few samples refused");

  c = goodConfig(33, 2);
  check(!s.start(c), "too many samples refused");
}

static void test_population_failure_stops_before_q0_reads() {
  CalibrationQ0CaptureSession s;
  CensusResult bad = goodCensus();
  bad.unexpected_id = 1;
  bad.unexpected_ids[0] = 60;
  bad.unexpected_id_count = 1;
  bad.verdict = CensusVerdict::PROFILE_MISMATCH;

  check(s.start(goodConfig()), "start population failure case");
  check(s.markCensusStarted(), "mark census started");
  check(s.submitCensus(bad), "store bad census");
  check(s.markPreflightStarted(), "mark preflight started");
  check(!s.submitPreflight(goodPreflight()), "bad population refused");
  check(s.status().state == Q0CaptureState::FAILED, "population failed state");
  check(s.status().failure == Q0CaptureFailure::POPULATION_REJECTED,
        "population failure reason");
  Q0ReadRequest req{};
  check(!s.nextReadRequest(&req), "no q0 read after population failure");
}

static void test_observation_failures_are_immediate() {
  CalibrationQ0CaptureSession s;
  check(enterSampling(s), "sampling for read failure");
  Q0ReadRequest req{};
  check(s.nextReadRequest(&req), "request for read failure");
  Q0ReadObservation obs{};
  obs.bus_id = req.bus_id;
  obs.read_ok = false;
  check(!s.recordRead(obs), "failed read refused");
  check(s.status().failure == Q0CaptureFailure::READ_FAILED, "read failure reason");

  s.reset();
  check(enterSampling(s), "sampling for torque failure");
  check(s.nextReadRequest(&req), "request for torque failure");
  obs = Q0ReadObservation{};
  obs.bus_id = req.bus_id;
  obs.read_ok = true;
  obs.raw_tick = 2000;
  obs.torque_enable = 1;
  check(!s.recordRead(obs), "torque-on read refused");
  check(s.status().failure == Q0CaptureFailure::TORQUE_NOT_OFF, "torque failure reason");

  s.reset();
  check(enterSampling(s), "sampling for raw failure");
  check(s.nextReadRequest(&req), "request for raw failure");
  obs = Q0ReadObservation{};
  obs.bus_id = req.bus_id;
  obs.read_ok = true;
  obs.raw_tick = 4096;
  obs.torque_enable = 0;
  check(!s.recordRead(obs), "out-of-domain raw refused");
  check(s.status().failure == Q0CaptureFailure::RAW_DOMAIN, "raw failure reason");

  s.reset();
  check(enterSampling(s), "sampling for wrong bus failure");
  check(s.nextReadRequest(&req), "request for wrong bus failure");
  obs = Q0ReadObservation{};
  obs.bus_id = static_cast<uint8_t>(req.bus_id + 1);
  obs.read_ok = true;
  obs.raw_tick = 2000;
  obs.torque_enable = 0;
  check(!s.recordRead(obs), "wrong bus observation refused");
  check(s.status().failure == Q0CaptureFailure::WRONG_STATE, "wrong bus reason");
}

static void test_stability_failure_happens_at_candidate_gate() {
  CalibrationQ0CaptureSession s;
  Q0CaptureConfig c = goodConfig(3, 2);
  check(s.start(c), "start stability case");
  check(s.markCensusStarted(), "stability census start");
  check(s.submitCensus(goodCensus()), "stability census submit");
  check(s.markPreflightStarted(), "stability preflight start");
  check(s.submitPreflight(goodPreflight()), "stability preflight pass");

  for (uint8_t pass = 0; pass < 3 && s.active(); ++pass) {
    for (uint8_t joint = 0; joint < kLegServoSlotCount && s.active(); ++joint) {
      Q0ReadRequest req{};
      check(s.nextReadRequest(&req), "stability request");
      Q0ReadObservation obs{};
      obs.bus_id = req.bus_id;
      obs.read_ok = true;
      obs.torque_enable = 0;
      obs.raw_tick = 2000 + joint;
      if (joint == 0 && pass == 2) obs.raw_tick += 10;
      s.recordRead(obs);
    }
  }
  check(s.status().state == Q0CaptureState::FAILED, "unstable candidate fails session");
  check(s.status().failure == Q0CaptureFailure::CANDIDATE_REJECTED,
        "unstable candidate failure reason");
  check(s.candidates()[0].status == Q0BootstrapStatus::REJECT_UNSTABLE,
        "CR2-A remains the stability authority");
}

static void test_restart_gets_new_session_identity() {
  CalibrationQ0CaptureSession s;
  check(s.start(goodConfig()), "first session starts");
  const uint32_t first = s.status().capture_session_id;
  s.fail(Q0CaptureFailure::EXTERNAL_ABORT);
  check(s.start(goodConfig()), "second session starts");
  const uint32_t second = s.status().capture_session_id;
  check(first != 0 && second != 0 && first != second, "session id changes across restart");
}

static void test_strings_are_total() {
  for (uint8_t i = 0; i <= static_cast<uint8_t>(Q0CaptureState::FAILED); ++i) {
    check(std::strcmp(toString(static_cast<Q0CaptureState>(i)), "UNKNOWN") != 0,
          "capture state string total");
  }
  for (uint8_t i = 0; i <= static_cast<uint8_t>(Q0CaptureFailure::EXTERNAL_ABORT); ++i) {
    check(std::strcmp(toString(static_cast<Q0CaptureFailure>(i)), "UNKNOWN") != 0,
          "capture failure string total");
  }
}

int main() {
  std::printf("MATDOG CR2-B same-session q0 acquisition offline tests\n");
  test_good_session_is_round_robin_and_candidate_only();
  test_invalid_config_fails_before_sources();
  test_population_failure_stops_before_q0_reads();
  test_observation_failures_are_immediate();
  test_stability_failure_happens_at_candidate_gate();
  test_restart_gets_new_session_identity();
  test_strings_are_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("CALIBRATION_Q0_CAPTURE_SESSION_TESTS = FAIL\n");
    return 1;
  }
  std::printf("CALIBRATION_Q0_CAPTURE_SESSION_TESTS = PASS\n");
  return 0;
}
