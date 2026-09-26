// Offline host tests for the calibration session manager
// (src/calibration/CalibrationManager.*) against the REAL ActuatorAuthority
// arbiter — no mock authority, so the integration under test is the shipped
// one.
//
// The manager is pure: no Arduino runtime, no ServoBus, no UART, no Wi-Fi, no
// OTA, no scheduler, no global SAFE_OFF. It cannot command a joint, and these
// tests cannot make it.
//
// Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/calibration/CalibrationManager.h"
#include "lf_v25_oracle_fixture.h"

using namespace matdog::calibration;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::AuthorityLease;
using matdog::core::AuthorityResult;
using matdog::core::OperatingMode;
namespace fixture = matdog::tests;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (!(cond)) {                                                             \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                          \
  } while (0)

#define CHECK_EQ(actual, expected)                                             \
  do {                                                                         \
    ++g_checks;                                                                \
    const long a_ = (long)(actual);                                            \
    const long e_ = (long)(expected);                                          \
    if (a_ != e_) {                                                            \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,      \
                  __FILE__, __LINE__, #actual, a_, e_);                        \
    }                                                                          \
  } while (0)

#define CHECK_STATE(mgr, expected)                                             \
  do {                                                                         \
    ++g_checks;                                                                \
    if ((mgr).status().state != (expected)) {                                  \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: state == %s, expected %s\n", g_case,     \
                  __FILE__, __LINE__, toString((mgr).status().state),          \
                  toString(expected));                                         \
    }                                                                          \
  } while (0)

// A replay session is the only origin allowed while hardware motion is
// blocked, so it is what most lifecycle tests use.
static constexpr CalibrationOrigin kReplay = CalibrationOrigin::HISTORICAL_REPLAY;

static LegPopulationEvidence fullPopulation(CalibrationOrigin origin) {
  LegPopulationEvidence e{};
  e.evaluated = true;
  e.origin = origin;
  e.observed_mask = (1u << kLegServoSlotCount) - 1u;
  e.session_ms = 42;
  return e;
}

// Drives a manager to ACTIVE with a complete population of the given origin.
static void activateSession(CalibrationManager& mgr, ActuatorAuthorityArbiter& arb,
                            CalibrationOrigin origin = kReplay) {
  (void)arb;
  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, origin),
           (int)SessionResult::STARTED);
  CHECK(mgr.submitPopulationEvidence(fullPopulation(origin)));
  CHECK_EQ((int)mgr.activate(), (int)SessionResult::OK);
  CHECK_STATE(mgr, SessionState::ACTIVE);
}

// ---------------------------------------------------------------------------
// Start / eligibility
// ---------------------------------------------------------------------------

static void test_no_session_at_construction() {
  g_case = "no_session_at_construction";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);

  CHECK_STATE(mgr, SessionState::NO_SESSION);
  CHECK(!mgr.sessionLive());
  CHECK(!mgr.status().holds_authority);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
  // The repository's refusal is surfaced, not implied by silence.
  CHECK(!mgr.status().hardware_motion_authorized);
  CHECK(mgr.status().current_calibration_stale);
}

static void test_start_fails_closed_without_an_arbiter() {
  g_case = "start_fails_closed_without_an_arbiter";
  CalibrationManager mgr;
  mgr.begin(nullptr);
  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::REJECTED_NO_ARBITER);
  CHECK_STATE(mgr, SessionState::NO_SESSION);
}

static void test_live_session_is_blocked_by_the_hardware_motion_gate() {
  g_case = "live_session_is_blocked_by_the_hardware_motion_gate";
  // MATDOG_JOINT_CALIBRATION.yaml: hardware_motion_authorized: false.
  CHECK(!CalibrationManager::hardwareMotionAuthorized());

  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);

  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE,
                                 CalibrationOrigin::LIVE_SESSION),
           (int)SessionResult::REJECTED_MOTION_BLOCKED);
  CHECK_STATE(mgr, SessionState::NO_SESSION);
  CHECK_EQ((int)mgr.status().failure, (int)CalibrationFailure::STALE_CALIBRATION_REFUSED);
  // Refused before the arbiter was even asked: no authority was taken.
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
  CHECK_EQ(arb.counters().grants, 0u);
}

static void test_start_acquires_the_real_calibration_authority() {
  g_case = "start_acquires_the_real_calibration_authority";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);

  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::STARTED);
  CHECK_STATE(mgr, SessionState::PREFLIGHT);
  CHECK(mgr.sessionLive());
  CHECK(mgr.status().holds_authority);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::CALIBRATION);
  // The exact lease generation is retained.
  CHECK_EQ(mgr.status().lease_generation, arb.generation());
  CHECK(mgr.status().lease_generation != 0);
  CHECK_EQ(arb.counters().grants, 1u);
}

static void test_start_is_refused_when_another_owner_holds_authority() {
  g_case = "start_is_refused_when_another_owner_holds_authority";
  const ActuatorAuthority others[] = {ActuatorAuthority::DIAGNOSTICS, ActuatorAuthority::QC,
                                      ActuatorAuthority::PROVISIONING,
                                      ActuatorAuthority::MOTION};
  for (ActuatorAuthority other : others) {
    ActuatorAuthorityArbiter arb;
    arb.reset(AuthorityClearReason::BOOT);
    const OperatingMode mode = (other == ActuatorAuthority::MOTION)
                                   ? OperatingMode::RUN
                                   : OperatingMode::MAINTENANCE;
    AuthorityLease held;
    CHECK_EQ((int)arb.request(other, mode, &held), (int)AuthorityResult::GRANTED);

    CalibrationManager mgr;
    mgr.begin(&arb);
    CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
             (int)SessionResult::REJECTED_NO_AUTHORITY);
    CHECK_STATE(mgr, SessionState::NO_SESSION);
    CHECK(!mgr.status().holds_authority);
    // The incumbent is untouched.
    CHECK_EQ((int)arb.current(), (int)other);
  }
}

static void test_start_is_refused_in_an_incompatible_operating_mode() {
  g_case = "start_is_refused_in_an_incompatible_operating_mode";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);

  // CALIBRATION cannot live in RUN - that is the arbiter's rule and the
  // manager does not get to second-guess it.
  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::RUN, kReplay),
           (int)SessionResult::REJECTED_NO_AUTHORITY);
  CHECK_STATE(mgr, SessionState::NO_SESSION);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
}

static void test_start_is_refused_while_a_session_is_live() {
  g_case = "start_is_refused_while_a_session_is_live";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);

  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::STARTED);
  const uint32_t generation = mgr.status().lease_generation;
  CHECK_EQ((int)mgr.startSession(Leg::RF, OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::REJECTED_BUSY);
  // The live session is undisturbed - same leg, same lease.
  CHECK_STATE(mgr, SessionState::PREFLIGHT);
  CHECK_EQ((int)mgr.status().leg, (int)Leg::LF);
  CHECK_EQ(mgr.status().lease_generation, generation);
}

static void test_invalid_start_arguments() {
  g_case = "invalid_start_arguments";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  CHECK_EQ((int)mgr.startSession(static_cast<Leg>(9), OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::REJECTED_INVALID_LEG);
  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE,
                                 CalibrationOrigin::NONE),
           (int)SessionResult::REJECTED_INVALID_LEG);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
}

// ---------------------------------------------------------------------------
// Population gate
// ---------------------------------------------------------------------------

static void test_population_evidence_must_match_the_session_origin() {
  g_case = "population_evidence_must_match_the_session_origin";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::STARTED);

  // Live evidence cannot be submitted to a replay session, or the reverse.
  CHECK(!mgr.submitPopulationEvidence(fullPopulation(CalibrationOrigin::LIVE_SESSION)));
  CHECK_EQ((int)mgr.status().last_result, (int)SessionResult::REJECTED_POPULATION_GATE);
  CHECK(mgr.submitPopulationEvidence(fullPopulation(kReplay)));
  CHECK_EQ((int)mgr.status().population_verdict, (int)PopulationVerdict::PASS);
}

static void test_population_evidence_is_refused_outside_preflight() {
  g_case = "population_evidence_is_refused_outside_preflight";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  // No session at all.
  CHECK(!mgr.submitPopulationEvidence(fullPopulation(kReplay)));
  activateSession(mgr, arb);
  // Already active - eligibility is settled.
  CHECK(!mgr.submitPopulationEvidence(fullPopulation(kReplay)));
}

static void test_activate_requires_preflight_and_a_verdict() {
  g_case = "activate_requires_preflight_and_a_verdict";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  // Cannot activate without a session.
  CHECK_EQ((int)mgr.activate(), (int)SessionResult::REJECTED_WRONG_STATE);

  CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
           (int)SessionResult::STARTED);
  // A replay session commands nothing, so it may activate without a current
  // population pass - it is not authorising anything.
  CHECK_EQ((int)mgr.activate(), (int)SessionResult::OK);
  CHECK_STATE(mgr, SessionState::ACTIVE);
  // Cannot activate twice.
  CHECK_EQ((int)mgr.activate(), (int)SessionResult::REJECTED_WRONG_STATE);
}

// ---------------------------------------------------------------------------
// Execution-phase reports and evidence
// ---------------------------------------------------------------------------

static void test_execution_phase_reports_are_validated_not_produced() {
  g_case = "execution_phase_reports_are_validated_not_produced";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);

  // Walk the recovered order as a future execution layer would report it.
  for (size_t i = 1; i < fixture::kLfV25PhaseSequenceLength; ++i) {
    CHECK(mgr.noteExecutionPhase(fixture::kLfV25PhaseSequence[i]));
  }
  CHECK_EQ((int)mgr.status().last_reported_phase, (int)CalibrationPhase::TORQUE_OFF);

  // Illegal jumps are refused.
  CalibrationManager other;
  ActuatorAuthorityArbiter arb2;
  arb2.reset(AuthorityClearReason::BOOT);
  other.begin(&arb2);
  activateSession(other, arb2);
  CHECK(!other.noteExecutionPhase(CalibrationPhase::HIP_MAX));  // skips the proof
  CHECK(!other.status().execution_phase_reported);
}

static void test_contact_evidence_admission_rules() {
  g_case = "contact_evidence_admission_rules";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);

  const uint16_t band = fixture::kLfV25ContactWitnessToleranceTicks;

  ContactEvidence good{};
  good.key = {Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE};
  good.origin = kReplay;
  good.detection = ContactState::CONTACT_CONFIRMED;
  good.witness = makeContactWitness(0, 0, band);
  CHECK(mgr.recordContact(good));
  CHECK_EQ(mgr.status().contacts_recorded, 1u);

  // Wrong leg.
  ContactEvidence wrong_leg = good;
  wrong_leg.key.leg = Leg::RF;
  CHECK(!mgr.recordContact(wrong_leg));

  // Wrong origin - a live record cannot enter a replay session.
  ContactEvidence wrong_origin = good;
  wrong_origin.origin = CalibrationOrigin::LIVE_SESSION;
  CHECK(!mgr.recordContact(wrong_origin));

  // Not a confirmed contact.
  for (ContactState s : {ContactState::FREE_MOTION, ContactState::CONTACT_SUSPECTED,
                         ContactState::EARLY_STALL, ContactState::HARD_ABORT}) {
    ContactEvidence bad = good;
    bad.detection = s;
    CHECK(!mgr.recordContact(bad));
  }

  // Witness outside the band, and a witness with no band at all.
  ContactEvidence outside = good;
  outside.witness = makeContactWitness(0, band + 1, band);
  CHECK(!mgr.recordContact(outside));
  ContactEvidence unbanded = good;
  unbanded.witness = ContactWitness{};
  unbanded.witness.evaluated = true;
  CHECK(!mgr.recordContact(unbanded));

  // Invalid key.
  ContactEvidence bad_key = good;
  bad_key.key.joint = static_cast<JointKind>(9);
  CHECK(!mgr.recordContact(bad_key));

  CHECK_EQ(mgr.status().contacts_recorded, 1u);  // only the good one landed
}

// ---------------------------------------------------------------------------
// Terminal paths — authority must never be left suspended
// ---------------------------------------------------------------------------

static void test_abort_releases_authority() {
  g_case = "abort_releases_authority";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);
  CHECK(mgr.noteExecutionPhase(CalibrationPhase::INITIAL_RECOVERY));

  mgr.abortSession();
  CHECK_STATE(mgr, SessionState::ABORTED);
  CHECK(!mgr.status().holds_authority);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
  CHECK_EQ((int)mgr.status().failure, (int)CalibrationFailure::OPERATOR_ABORT);
  // Abort engaged joints, so a restore is owed - and torque off regardless.
  CHECK(mgr.status().restore.required);
  CHECK(mgr.status().restore.torque_off_required);
  CHECK_EQ(mgr.status().sessions_aborted, 1u);

  // Double abort is a no-op and does not disturb the arbiter.
  mgr.abortSession();
  mgr.abortSession();
  CHECK_STATE(mgr, SessionState::ABORTED);
  CHECK_EQ(mgr.status().sessions_aborted, 1u);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
}

static void test_failure_releases_authority_from_every_live_state() {
  g_case = "failure_releases_authority_from_every_live_state";
  const CalibrationFailure causes[] = {
      CalibrationFailure::HARD_CURRENT_ABORT, CalibrationFailure::CONTACT_WITNESS_REJECTED,
      CalibrationFailure::MOTION_TIMEOUT,     CalibrationFailure::TELEMETRY_STALE,
      CalibrationFailure::AFFINE_GATE_REJECTED};

  for (CalibrationFailure cause : causes) {
    // From PREFLIGHT.
    {
      ActuatorAuthorityArbiter arb;
      arb.reset(AuthorityClearReason::BOOT);
      CalibrationManager mgr;
      mgr.begin(&arb);
      CHECK_EQ((int)mgr.startSession(Leg::LF, OperatingMode::MAINTENANCE, kReplay),
               (int)SessionResult::STARTED);
      mgr.failSession(cause);
      CHECK_STATE(mgr, SessionState::FAILED);
      CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
      CHECK_EQ((int)mgr.status().failure, (int)cause);
      // Nothing was ever reported as engaged, so no restore is owed.
      CHECK(!mgr.status().restore.required);
      CHECK(mgr.status().restore.torque_off_required);
    }
    // From ACTIVE with execution engaged.
    {
      ActuatorAuthorityArbiter arb;
      arb.reset(AuthorityClearReason::BOOT);
      CalibrationManager mgr;
      mgr.begin(&arb);
      activateSession(mgr, arb);
      CHECK(mgr.noteExecutionPhase(CalibrationPhase::INITIAL_RECOVERY));
      mgr.failSession(cause);
      CHECK_STATE(mgr, SessionState::FAILED);
      CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
      CHECK(mgr.status().restore.required);
    }
  }
}

static void test_complete_requires_reaching_the_end_of_the_sequence() {
  g_case = "complete_requires_reaching_the_end_of_the_sequence";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);

  // Cannot declare success from the middle.
  CHECK(!mgr.completeSession());
  CHECK(mgr.noteExecutionPhase(CalibrationPhase::INITIAL_RECOVERY));
  CHECK(!mgr.completeSession());
  CHECK_STATE(mgr, SessionState::ACTIVE);

  // Walk to the end.
  for (size_t i = 2; i < fixture::kLfV25PhaseSequenceLength; ++i) {
    CHECK(mgr.noteExecutionPhase(fixture::kLfV25PhaseSequence[i]));
  }
  CHECK(mgr.completeSession());
  CHECK_STATE(mgr, SessionState::COMPLETED);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
  CHECK(!mgr.status().holds_authority);
  // Success still requires torque off, and owes no restore.
  CHECK(!mgr.status().restore.required);
  CHECK(mgr.status().restore.torque_off_required);
  CHECK_EQ(mgr.status().sessions_completed, 1u);

  // Terminal: completing again is refused.
  CHECK(!mgr.completeSession());
}

static void test_reset_releases_and_preserves_counters() {
  g_case = "reset_releases_and_preserves_counters";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);

  mgr.reset();
  CHECK_STATE(mgr, SessionState::NO_SESSION);
  CHECK(!mgr.status().holds_authority);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
  // A reset of a live session counts as an abort.
  CHECK_EQ(mgr.status().sessions_aborted, 1u);
  CHECK_EQ(mgr.status().sessions_started, 1u);
  // The repository's refusal survives a reset.
  CHECK(!mgr.status().hardware_motion_authorized);
  CHECK(mgr.status().current_calibration_stale);

  // Double reset is a no-op.
  mgr.reset();
  CHECK_STATE(mgr, SessionState::NO_SESSION);
  CHECK_EQ(mgr.status().sessions_aborted, 1u);

  // And a new session works afterwards.
  activateSession(mgr, arb);
  CHECK_EQ(mgr.status().sessions_started, 2u);
}

// ---------------------------------------------------------------------------
// Authority loss and stale leases
// ---------------------------------------------------------------------------

static void test_authority_lost_externally_fails_the_session_closed() {
  g_case = "authority_lost_externally_fails_the_session_closed";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);
  CHECK(mgr.noteExecutionPhase(CalibrationPhase::INITIAL_RECOVERY));

  // A fatal-fault force clear takes the authority out from under the session.
  arb.forceClear(AuthorityClearReason::FATAL_FAULT);
  mgr.update(OperatingMode::MAINTENANCE);

  CHECK_STATE(mgr, SessionState::FAILED);
  CHECK_EQ((int)mgr.status().failure, (int)CalibrationFailure::AUTHORITY_LOST);
  CHECK(!mgr.status().holds_authority);
  // Losing authority must NOT schedule a restore: restore commands joints, and
  // commanding joints without authority is precisely what must not happen.
  CHECK(!mgr.status().restore.required);
  CHECK(mgr.status().restore.torque_off_required);
  // No further work is accepted.
  CHECK(!mgr.noteExecutionPhase(CalibrationPhase::PARKING));
  CHECK(!mgr.completeSession());
}

static void test_mode_change_that_strands_calibration_fails_closed() {
  g_case = "mode_change_that_strands_calibration_fails_closed";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb);

  // The arbiter clears a stranded owner on a mode change.
  arb.onOperatingModeChanged(OperatingMode::RUN);
  mgr.update(OperatingMode::RUN);
  CHECK_STATE(mgr, SessionState::FAILED);
  CHECK_EQ((int)mgr.status().failure, (int)CalibrationFailure::AUTHORITY_LOST);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
}

static void test_update_is_quiet_when_nothing_is_live() {
  g_case = "update_is_quiet_when_nothing_is_live";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  for (int i = 0; i < 100; ++i) mgr.update(OperatingMode::MAINTENANCE);
  CHECK_STATE(mgr, SessionState::NO_SESSION);
  CHECK_EQ(mgr.status().sessions_failed, 0u);
}

static void test_a_stale_session_cannot_disturb_a_newer_one() {
  g_case = "a_stale_session_cannot_disturb_a_newer_one";
  // THE bug this lease generation exists to prevent, end to end:
  //   session A acquires CALIBRATION, ends
  //   session B acquires CALIBRATION
  //   a late event from A arrives
  // B must survive untouched.
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);

  CalibrationManager session_a;
  session_a.begin(&arb);
  activateSession(session_a, arb);
  const uint32_t generation_a = session_a.status().lease_generation;
  session_a.abortSession();
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);

  CalibrationManager session_b;
  session_b.begin(&arb);
  activateSession(session_b, arb);
  const uint32_t generation_b = session_b.status().lease_generation;
  CHECK(generation_b != generation_a);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::CALIBRATION);

  // Every late call from A. None of them may touch B.
  session_a.abortSession();
  session_a.failSession(CalibrationFailure::HARD_CURRENT_ABORT);
  session_a.reset();
  session_a.update(OperatingMode::MAINTENANCE);
  (void)session_a.completeSession();
  (void)session_a.noteExecutionPhase(CalibrationPhase::PARKING);

  CHECK_STATE(session_b, SessionState::ACTIVE);
  CHECK(session_b.status().holds_authority);
  CHECK_EQ(session_b.status().lease_generation, generation_b);
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::CALIBRATION);
  CHECK_EQ(arb.generation(), generation_b);

  // B still works normally afterwards.
  CHECK(session_b.noteExecutionPhase(CalibrationPhase::INITIAL_RECOVERY));
  session_b.abortSession();
  CHECK_EQ((int)arb.current(), (int)ActuatorAuthority::NONE);
}

static void test_same_owner_new_generation_is_a_different_session() {
  g_case = "same_owner_new_generation_is_a_different_session";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);

  activateSession(mgr, arb);
  const uint32_t first = mgr.status().lease_generation;
  mgr.abortSession();
  activateSession(mgr, arb);
  const uint32_t second = mgr.status().lease_generation;

  CHECK(second > first);
  CHECK_EQ(arb.generation(), second);
  // The manager tracks the CURRENT generation, not the first one it ever saw.
  mgr.update(OperatingMode::MAINTENANCE);
  CHECK_STATE(mgr, SessionState::ACTIVE);
}

// ---------------------------------------------------------------------------
// The manager owns no hardware
// ---------------------------------------------------------------------------

static void test_manager_cannot_promote_a_replay() {
  g_case = "manager_cannot_promote_a_replay";
  ActuatorAuthorityArbiter arb;
  arb.reset(AuthorityClearReason::BOOT);
  CalibrationManager mgr;
  mgr.begin(&arb);
  activateSession(mgr, arb, kReplay);

  // Everything a replay session records stays a replay. There is no API on the
  // manager that changes an origin or promotes evidence.
  CHECK_EQ((int)mgr.status().origin, (int)CalibrationOrigin::HISTORICAL_REPLAY);
  CHECK(!mayPromote(mgr.status().origin));
  CHECK(!mgr.status().hardware_motion_authorized);
}

static void test_tostring_is_total() {
  g_case = "tostring_is_total";
  for (uint8_t i = 0; i <= (uint8_t)SessionState::FAILED; ++i) {
    CHECK(std::strcmp(toString((SessionState)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)SessionResult::REJECTED_POPULATION_GATE; ++i) {
    CHECK(std::strcmp(toString((SessionResult)i), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(SessionState::NO_SESSION), "NO_SESSION") == 0);
  CHECK(std::strcmp(toString(SessionResult::REJECTED_MOTION_BLOCKED),
                    "REJECTED_MOTION_BLOCKED") == 0);
}

int main() {
  std::printf("MATDOG CalibrationManager offline tests\n");

  test_no_session_at_construction();
  test_start_fails_closed_without_an_arbiter();
  test_live_session_is_blocked_by_the_hardware_motion_gate();
  test_start_acquires_the_real_calibration_authority();
  test_start_is_refused_when_another_owner_holds_authority();
  test_start_is_refused_in_an_incompatible_operating_mode();
  test_start_is_refused_while_a_session_is_live();
  test_invalid_start_arguments();
  test_population_evidence_must_match_the_session_origin();
  test_population_evidence_is_refused_outside_preflight();
  test_activate_requires_preflight_and_a_verdict();
  test_execution_phase_reports_are_validated_not_produced();
  test_contact_evidence_admission_rules();
  test_abort_releases_authority();
  test_failure_releases_authority_from_every_live_state();
  test_complete_requires_reaching_the_end_of_the_sequence();
  test_reset_releases_and_preserves_counters();
  test_authority_lost_externally_fails_the_session_closed();
  test_mode_change_that_strands_calibration_fails_closed();
  test_update_is_quiet_when_nothing_is_live();
  test_a_stale_session_cannot_disturb_a_newer_one();
  test_same_owner_new_generation_is_a_different_session();
  test_manager_cannot_promote_a_replay();
  test_tostring_is_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("CALIBRATION_MANAGER_TESTS = FAIL\n");
    return 1;
  }
  std::printf("CALIBRATION_MANAGER_TESTS = PASS\n");
  return 0;
}
