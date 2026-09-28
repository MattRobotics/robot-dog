// Offline tests for the CR3 session-start orchestrator.
// No Arduino, ServoBus, actuator backend or hardware I/O.
//
// The unit under test must:
//   completed current Q0 population
//     -> acquire CALIBRATION authority
//     -> submit the already-produced population evidence
//     -> activate one LIVE_SESSION
//
// It must NEVER create population evidence, grant a motion permit or
// perform a physical write.

#include <cstdio>

#include "../../src/calibration/CalibrationSessionOrchestrator.h"
#include "../../src/core/ActuatorAuthority.h"

using namespace matdog;
using namespace matdog::calibration;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::AuthorityLease;
using matdog::core::AuthorityResult;
using matdog::core::OperatingMode;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                             \
  do {                                                                          \
    ++g_checks;                                                                 \
    if (!(cond)) {                                                              \
      ++g_failures;                                                             \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                           \
  } while (0)

#define CHECK_EQ(actual, expected)                                               \
  do {                                                                           \
    ++g_checks;                                                                  \
    const long a_ = (long)(actual);                                              \
    const long e_ = (long)(expected);                                            \
    if (a_ != e_) {                                                              \
      ++g_failures;                                                              \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n",               \
                  g_case, __FILE__, __LINE__, #actual, a_, e_);                  \
    }                                                                            \
  } while (0)

namespace {

PopulationEvidenceBuildResult currentPassPopulation() {
  PopulationEvidenceBuildResult p{};
  p.status = PopulationEvidenceBuildStatus::PASS;
  p.evidence.evaluated = true;
  p.evidence.origin = CalibrationOrigin::LIVE_SESSION;
  p.evidence.observed_mask =
      static_cast<uint16_t>((1u << kLegServoSlotCount) - 1u);
  p.evidence.unexpected_count = 0;
  p.evidence.session_ms = 1234;
  p.qualified_slots = kLegServoSlotCount;
  p.rejected_slots = 0;
  return p;
}

struct Rig {
  ActuatorAuthorityArbiter arbiter;
  CalibrationManager manager;

  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    manager.begin(&arbiter);
  }
};

void test_incomplete_q0_capture_refused_without_authority() {
  g_case = "incomplete q0 capture";
  Rig r;

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::SAMPLING, currentPassPopulation(),
      Leg::LF, OperatingMode::MAINTENANCE);

  CHECK_EQ((int)out.status,
           (int)SessionStartFromQ0Status::REJECT_Q0_CAPTURE_NOT_COMPLETE);
  CHECK_EQ((int)r.manager.status().state, (int)SessionState::NO_SESSION);
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::NONE);
}

void test_population_build_not_pass_refused_without_authority() {
  g_case = "population build not pass";
  Rig r;
  auto population = currentPassPopulation();
  population.status = PopulationEvidenceBuildStatus::REJECT_SOURCE_NOT_CURRENT;

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::COMPLETE, population,
      Leg::LF, OperatingMode::MAINTENANCE);

  CHECK_EQ((int)out.status,
           (int)SessionStartFromQ0Status::REJECT_POPULATION_NOT_PASS);
  CHECK_EQ((int)r.manager.status().state, (int)SessionState::NO_SESSION);
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::NONE);
}

void test_historical_population_cannot_become_current() {
  g_case = "historical population";
  Rig r;
  auto population = currentPassPopulation();
  population.evidence.origin = CalibrationOrigin::HISTORICAL_REPLAY;

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::COMPLETE, population,
      Leg::LF, OperatingMode::MAINTENANCE);

  CHECK_EQ((int)out.status,
           (int)SessionStartFromQ0Status::REJECT_POPULATION_NOT_PASS);
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::NONE);
}

void test_invalid_leg_refused_before_manager_start() {
  g_case = "invalid leg";
  Rig r;

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::COMPLETE, currentPassPopulation(),
      static_cast<Leg>(99), OperatingMode::MAINTENANCE);

  CHECK_EQ((int)out.status,
           (int)SessionStartFromQ0Status::REJECT_INVALID_LEG);
  CHECK_EQ((int)r.manager.status().state, (int)SessionState::NO_SESSION);
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::NONE);
}

void test_existing_authority_is_not_displaced() {
  g_case = "authority already occupied";
  Rig r;

  AuthorityLease held{};
  CHECK_EQ((int)r.arbiter.request(
               ActuatorAuthority::QC, OperatingMode::MAINTENANCE, &held),
           (int)AuthorityResult::GRANTED);
  CHECK(held.valid());

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::COMPLETE, currentPassPopulation(),
      Leg::LF, OperatingMode::MAINTENANCE);

  CHECK_EQ((int)out.status,
           (int)SessionStartFromQ0Status::REJECT_MANAGER);
  CHECK_EQ((int)out.manager_result,
           (int)SessionResult::REJECTED_NO_AUTHORITY);
  CHECK_EQ((int)r.manager.status().state, (int)SessionState::NO_SESSION);
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::QC);
}

void test_run_mode_is_rejected_by_real_arbiter() {
  g_case = "run mode";
  Rig r;

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::COMPLETE, currentPassPopulation(),
      Leg::LF, OperatingMode::RUN);

  CHECK_EQ((int)out.status,
           (int)SessionStartFromQ0Status::REJECT_MANAGER);
  CHECK_EQ((int)out.manager_result,
           (int)SessionResult::REJECTED_NO_AUTHORITY);
  CHECK_EQ((int)r.manager.status().state, (int)SessionState::NO_SESSION);
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::NONE);
}

void test_happy_path_enters_live_active_session_only() {
  g_case = "happy path";
  Rig r;

  // CR3 session existence remains distinct from final operational motion auth.
  CHECK(!CalibrationManager::hardwareMotionAuthorized());

  const auto out = startCalibrationSessionFromQ0Evidence(
      r.manager, Q0CaptureState::COMPLETE, currentPassPopulation(),
      Leg::LF, OperatingMode::MAINTENANCE);

  CHECK_EQ((int)out.status, (int)SessionStartFromQ0Status::STARTED);
  CHECK_EQ((int)out.manager_result, (int)SessionResult::OK);

  const CalibrationSessionStatus& s = r.manager.status();
  CHECK_EQ((int)s.state, (int)SessionState::ACTIVE);
  CHECK_EQ((int)s.origin, (int)CalibrationOrigin::LIVE_SESSION);
  CHECK_EQ((int)s.leg, (int)Leg::LF);
  CHECK(s.session_id != 0);
  CHECK(s.holds_authority);
  CHECK(s.lease_generation != 0);
  CHECK_EQ(s.lease_generation, r.arbiter.generation());
  CHECK_EQ((int)s.population_verdict, (int)PopulationVerdict::PASS);
  CHECK(populationIsCurrentPass(s.population));
  CHECK_EQ((int)r.arbiter.current(), (int)ActuatorAuthority::CALIBRATION);

  // Starting a calibration session must NOT flip the global operational gate.
  CHECK(!s.hardware_motion_authorized);
  CHECK(!CalibrationManager::hardwareMotionAuthorized());
}

}  // namespace

int main() {
  test_incomplete_q0_capture_refused_without_authority();
  test_population_build_not_pass_refused_without_authority();
  test_historical_population_cannot_become_current();
  test_invalid_leg_refused_before_manager_start();
  test_existing_authority_is_not_displaced();
  test_run_mode_is_rejected_by_real_arbiter();
  test_happy_path_enters_live_active_session_only();

  std::printf("test_calibration_session_orchestrator: %d checks, %d failures\n",
              g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
