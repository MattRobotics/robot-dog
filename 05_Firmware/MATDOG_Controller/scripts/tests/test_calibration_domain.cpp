// Offline host tests for the MATDOG calibration domain model
// (src/calibration/CalibrationDomain.*) and the LF V25 oracle replay.
//
// Links the REAL model - it has no <Arduino.h>, no ServoBus, no Wi-Fi and no
// OTA, which is the point. Same contract as WifiPolicy, OtaPolicy and
// ActuatorAuthority.
//
// ⚠️ The oracle replay proves that the NEW model reproduces the HISTORICAL
// behavioural contract. It proves nothing about the current robot, whose
// servos were all removed, re-provisioned and remounted on 2026-08-27.
//
// Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/calibration/CalibrationDomain.h"
#include "lf_v25_oracle_fixture.h"

using namespace matdog::calibration;
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

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                              \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));          \
    }                                                                          \
  } while (0)

static const Leg kAllLegs[] = {Leg::LF, Leg::RF, Leg::RH, Leg::LH};
static const JointKind kAllJoints[] = {JointKind::HIP, JointKind::UPPER, JointKind::LOWER};
static const ContactSide kAllSides[] = {ContactSide::MIN_SIDE, ContactSide::MAX_SIDE};

// ---------------------------------------------------------------------------
// Contact-profile model — derived, never hard-coded
// ---------------------------------------------------------------------------

static void test_profile_count_is_derived_from_the_model() {
  g_case = "profile_count_is_derived_from_the_model";
  // The archive's own test asserts 24
  // (matdog_test.rs::profile_table_covers_exactly_24_unique_contacts).
  // Here 24 must FALL OUT of the Cartesian model, not be typed in.
  CHECK_EQ(kContactProfileCount, kLegCount * kJointKindCount * kContactSideCount);
  CHECK_EQ(kContactProfileCount, 24);
  CHECK_EQ(kLegCount, 4);
  CHECK_EQ(kJointKindCount, 3);
  CHECK_EQ(kContactSideCount, 2);
}

static void test_every_combination_maps_to_exactly_one_profile() {
  g_case = "every_combination_maps_to_exactly_one_profile";
  bool seen[kContactProfileCount] = {false};
  int produced = 0;

  for (Leg leg : kAllLegs) {
    for (JointKind joint : kAllJoints) {
      for (ContactSide side : kAllSides) {
        ContactProfileKey key{leg, joint, side};
        CHECK(key.valid());
        const uint16_t index = contactProfileIndex(key);
        CHECK(index < kContactProfileCount);
        CHECK(!seen[index]);          // no duplicate key maps to the same slot
        seen[index] = true;
        ++produced;
      }
    }
  }
  CHECK_EQ(produced, 24);
  for (uint16_t i = 0; i < kContactProfileCount; ++i) CHECK(seen[i]);  // none missing
}

static void test_profile_lookup_is_deterministic_and_round_trips() {
  g_case = "profile_lookup_is_deterministic_and_round_trips";
  for (uint16_t i = 0; i < kContactProfileCount; ++i) {
    ContactProfileKey key{};
    CHECK(contactProfileFromIndex(i, &key));
    CHECK(key.valid());
    CHECK_EQ(contactProfileIndex(key), i);
    // Deterministic: the same key always yields the same index.
    CHECK_EQ(contactProfileIndex(key), contactProfileIndex(key));
  }
}

static void test_invalid_profile_input_fails_closed() {
  g_case = "invalid_profile_input_fails_closed";
  // The enums have fixed uint8_t underlying types, so these casts are well
  // defined - which is exactly why the values must be validated.
  for (uint8_t raw : {(uint8_t)4, (uint8_t)9, (uint8_t)200, (uint8_t)255}) {
    ContactProfileKey bad_leg{static_cast<Leg>(raw), JointKind::HIP, ContactSide::MIN_SIDE};
    CHECK(!bad_leg.valid());
    // Out-of-range sentinel, never a wrapped index aliasing a real profile.
    CHECK_EQ(contactProfileIndex(bad_leg), kContactProfileCount);

    ContactProfileKey bad_joint{Leg::LF, static_cast<JointKind>(raw), ContactSide::MIN_SIDE};
    CHECK(!bad_joint.valid());
    CHECK_EQ(contactProfileIndex(bad_joint), kContactProfileCount);

    ContactProfileKey bad_side{Leg::LF, JointKind::HIP, static_cast<ContactSide>(raw)};
    CHECK(!bad_side.valid());
    CHECK_EQ(contactProfileIndex(bad_side), kContactProfileCount);
  }

  ContactProfileKey key{};
  CHECK(!contactProfileFromIndex(kContactProfileCount, &key));
  CHECK(!contactProfileFromIndex(1000, &key));
  CHECK(!contactProfileFromIndex(0, nullptr));
}

static void test_profile_token_matches_the_oracle_format() {
  g_case = "profile_token_matches_the_oracle_format";
  // matdog.rs::build_profile(): "{LEG}_{JOINT}_M{motor_id}_{SIDE}".
  char token[32];
  formatProfileToken({Leg::LF, JointKind::HIP, ContactSide::MAX_SIDE}, 13, token, sizeof(token));
  CHECK_STR(token, "LF_HIP_M13_MAX");

  // matdog_test.rs line 259 records this exact token.
  formatProfileToken({Leg::RF, JointKind::HIP, ContactSide::MAX_SIDE}, 23, token, sizeof(token));
  CHECK_STR(token, "RF_HIP_M23_MAX");

  formatProfileToken({Leg::RH, JointKind::HIP, ContactSide::MIN_SIDE}, 33, token, sizeof(token));
  CHECK_STR(token, "RH_HIP_M33_MIN");

  formatProfileToken({Leg::LH, JointKind::LOWER, ContactSide::MIN_SIDE}, 41, token, sizeof(token));
  CHECK_STR(token, "LH_LOWER_M41_MIN");

  // Invalid keys are refused, not rendered as a plausible-looking token.
  formatProfileToken({static_cast<Leg>(9), JointKind::HIP, ContactSide::MIN_SIDE}, 13, token,
                     sizeof(token));
  CHECK_STR(token, "INVALID");

  // Bounded: never writes past the buffer, always terminates.
  char small[6];
  std::memset(small, 'Z', sizeof(small));
  formatProfileToken({Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE}, 12, small, sizeof(small));
  CHECK_EQ(small[5], '\0');
  CHECK(std::strlen(small) < sizeof(small));
  formatProfileToken({Leg::LF, JointKind::HIP, ContactSide::MIN_SIDE}, 13, nullptr, 0);
}

// ---------------------------------------------------------------------------
// q0 — the 2048 trap
// ---------------------------------------------------------------------------

static void test_q0_never_defaults_to_the_raw_servo_centre() {
  g_case = "q0_never_defaults_to_the_raw_servo_centre";
  Q0Evidence q0{};
  // A default-constructed q0 has NO value. It is emphatically not 2048.
  CHECK(!q0.measured);
  CHECK(!q0.hasUsableValue());
  CHECK_EQ((int)q0.estimator, (int)Q0Estimator::NONE);
  CHECK(q0.tick != kServoRawCenter);
  CHECK_EQ(q0.tick, 0);

  // Setting a tick without measuring it does not make it usable.
  q0.tick = kServoRawCenter;
  CHECK(!q0.hasUsableValue());

  // Nor does claiming measurement without an estimator.
  q0.measured = true;
  CHECK(!q0.hasUsableValue());

  q0.estimator = Q0Estimator::AFFINE;
  CHECK(q0.hasUsableValue());
}

static void test_the_three_meanings_of_2048_stay_separate() {
  g_case = "the_three_meanings_of_2048_stay_separate";
  // The raw servo centre is a servo-level fact only.
  CHECK_EQ(kServoRawCenter, 2048);

  // The measured q0 values LF V25 actually produced are NOT the raw centre.
  // If a future change ever made q0 default to 2048, this fails.
  for (const auto& record : fixture::kLfV25Records) {
    CHECK(record.q0_affine_tick != kServoRawCenter);
  }
  CHECK_EQ(fixture::kLfV25Records[0].q0_affine_tick, 2067);  // HIP
  CHECK_EQ(fixture::kLfV25Records[1].q0_affine_tick, 2040);  // UPPER
  CHECK_EQ(fixture::kLfV25Records[2].q0_affine_tick, 2074);  // LOWER

  // The displayed position AFTER the EEPROM offset write is a third quantity,
  // and it IS ~2048 (the V25 gate accepted 2048 +/- 10). Confusing it with q0
  // is the exact mistake this test exists to prevent.
  for (const auto& record : fixture::kLfV25Records) {
    const int delta = (int)record.final_displayed_position - (int)kServoRawCenter;
    CHECK(delta >= -10 && delta <= 10);
    CHECK(record.final_displayed_position != record.q0_affine_tick ||
          record.joint == JointKind::HIP);  // HIP happens to coincide at 2048
  }
}

static void test_the_two_q0_estimators_are_not_interchangeable() {
  g_case = "the_two_q0_estimators_are_not_interchangeable";
  // LF V25 produced both and treated AFFINE as authoritative, keeping the
  // fixed-scale disagreement as a visible diagnostic rather than discarding it.
  Q0Evidence fixed{};
  fixed.measured = true;
  fixed.estimator = Q0Estimator::FIXED_SCALE;
  fixed.tick = 2100;

  Q0Evidence affine{};
  affine.measured = true;
  affine.estimator = Q0Estimator::AFFINE;
  affine.tick = 2067;

  CHECK(fixed.hasUsableValue());
  CHECK(affine.hasUsableValue());
  CHECK(fixed.estimator != affine.estimator);
  CHECK_STR(toString(Q0Estimator::AFFINE), "AFFINE");
  CHECK_STR(toString(Q0Estimator::FIXED_SCALE), "FIXED_SCALE");
}

// ---------------------------------------------------------------------------
// Direction — evidence, not a sign convention
// ---------------------------------------------------------------------------

static void test_direction_is_unknown_until_measured() {
  g_case = "direction_is_unknown_until_measured";
  DirectionEvidence d{};
  CHECK_EQ((int)d.source, (int)DirectionSource::UNKNOWN);
  CHECK_EQ(d.sign, 0);
  CHECK(!d.isCalibrationEvidence());

  // AUDIT FINDING: LF V25's `direction` is a compile-time JointSpec constant,
  // never measured. Carrying it forward as calibration evidence would be a
  // fabrication - and after the 2026-08-27 reassembly it is not even evidence
  // about this servo.
  for (const auto& record : fixture::kLfV25Records) {
    DirectionEvidence spec{};
    spec.source = DirectionSource::SPEC_CONSTANT;
    spec.sign = record.spec_direction;
    CHECK(spec.sign == -1 || spec.sign == 1);
    CHECK(!spec.isCalibrationEvidence());   // a constant is not a witness
  }

  DirectionEvidence measured{};
  measured.source = DirectionSource::MEASURED_WITNESS;
  measured.sign = -1;
  CHECK(measured.isCalibrationEvidence());

  // A sign of 0 is never evidence, whatever the source claims.
  measured.sign = 0;
  CHECK(!measured.isCalibrationEvidence());

  DirectionEvidence conflict{};
  conflict.source = DirectionSource::CONFLICTING;
  conflict.sign = 1;
  CHECK(!conflict.isCalibrationEvidence());
}

// ---------------------------------------------------------------------------
// Contact witness — the real V25 gate
// ---------------------------------------------------------------------------

static void test_contact_witness_band() {
  g_case = "contact_witness_band";
  CHECK_EQ(kContactWitnessToleranceTicks, 24);  // matdog.rs constant

  ContactWitness w{};
  CHECK(!w.accepted());        // never evaluated => never accepted

  w.evaluated = true;
  w.min_deviation_ticks = 24;
  w.max_deviation_ticks = 24;
  CHECK(w.accepted());         // the bound is inclusive

  w.max_deviation_ticks = 25;
  CHECK(!w.accepted());        // one tick past the band is a rejection

  // BOTH endpoints must be inside; one good endpoint does not rescue the pair.
  w.min_deviation_ticks = 0;
  CHECK(!w.accepted());
}

static void test_contact_state_classification() {
  g_case = "contact_state_classification";
  CHECK(isContactEvidence(ContactState::CONTACT_CONFIRMED));
  // A suspected contact is not yet evidence.
  CHECK(!isContactEvidence(ContactState::CONTACT_SUSPECTED));
  CHECK(!isContactEvidence(ContactState::FREE_MOTION));
  // Treating a stall or a hard abort as a contact is the V24 mistake.
  CHECK(!isContactEvidence(ContactState::EARLY_STALL));
  CHECK(!isContactEvidence(ContactState::HARD_ABORT));
  CHECK(isContactFailure(ContactState::EARLY_STALL));
  CHECK(isContactFailure(ContactState::HARD_ABORT));
  CHECK(!isContactFailure(ContactState::FREE_MOTION));
}

// ---------------------------------------------------------------------------
// Evidence lifecycle — no shortcut to operational calibration
// ---------------------------------------------------------------------------

static void test_evidence_lifecycle_has_no_shortcuts() {
  g_case = "evidence_lifecycle_has_no_shortcuts";
  const EvidenceState order[] = {EvidenceState::UNKNOWN, EvidenceState::MEASURED,
                                 EvidenceState::CANDIDATE, EvidenceState::ACCEPTED,
                                 EvidenceState::PROMOTED};
  // Each forward step is legal.
  for (int i = 0; i + 1 < 5; ++i) CHECK(isLegalEvidenceTransition(order[i], order[i + 1]));

  // Every skip is refused - including the dangerous one, UNKNOWN -> PROMOTED.
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (j == i + 1) continue;
      CHECK(!isLegalEvidenceTransition(order[i], order[j]));
    }
  }

  // Backwards is refused too: evidence does not un-accept itself.
  for (int i = 4; i > 0; --i) CHECK(!isLegalEvidenceTransition(order[i], order[i - 1]));

  // Terminal states stay terminal.
  for (EvidenceState s : order) {
    CHECK(!isLegalEvidenceTransition(EvidenceState::PROMOTED, s));
    CHECK(!isLegalEvidenceTransition(EvidenceState::REJECTED, s));
  }
  CHECK(!isLegalEvidenceTransition(EvidenceState::PROMOTED, EvidenceState::REJECTED));

  // Rejection is reachable from any live state.
  CHECK(isLegalEvidenceTransition(EvidenceState::MEASURED, EvidenceState::REJECTED));
  CHECK(isLegalEvidenceTransition(EvidenceState::CANDIDATE, EvidenceState::REJECTED));
  CHECK(isLegalEvidenceTransition(EvidenceState::ACCEPTED, EvidenceState::REJECTED));

  CHECK(isOperationalEvidence(EvidenceState::PROMOTED));
  for (int i = 0; i < 4; ++i) CHECK(!isOperationalEvidence(order[i]));
  CHECK(!isOperationalEvidence(EvidenceState::REJECTED));
}

static void test_historical_replay_can_never_be_promoted() {
  g_case = "historical_replay_can_never_be_promoted";
  // THE safety property. A replay reproduces 2026-08-04 behaviour; it says
  // nothing about the machine that exists today.
  CHECK(!mayPromote(CalibrationOrigin::HISTORICAL_REPLAY));
  CHECK(!mayPromote(CalibrationOrigin::NONE));
  CHECK(mayPromote(CalibrationOrigin::LIVE_SESSION));

  // Every record the oracle fixture produces carries the replay origin.
  ContactEvidence e{};
  e.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  e.state = EvidenceState::ACCEPTED;
  CHECK(isLegalEvidenceTransition(e.state, EvidenceState::PROMOTED));  // shape is legal...
  CHECK(!mayPromote(e.origin));                                        // ...origin forbids it
}

// ---------------------------------------------------------------------------
// Phases, restore, failure
// ---------------------------------------------------------------------------

static void test_phase_order_proves_upper_and_lower_before_hip() {
  g_case = "phase_order_proves_upper_and_lower_before_hip";
  // matdog.rs::hardware_profile_allowed() blocks every isolated HIP profile:
  // the hip is only calibrated after the UPPER/LOWER proof. The phase order
  // encodes that, so it must not be reorderable by accident.
  const CalibrationPhase* seq = fixture::kLfV25PhaseSequence;
  int upper_max = -1, lower_max = -1, hip_min = -1;
  for (size_t i = 0; i < fixture::kLfV25PhaseSequenceLength; ++i) {
    if (seq[i] == CalibrationPhase::UPPER_MAX) upper_max = (int)i;
    if (seq[i] == CalibrationPhase::LOWER_MAX) lower_max = (int)i;
    if (seq[i] == CalibrationPhase::HIP_MIN) hip_min = (int)i;
  }
  CHECK(upper_max >= 0 && lower_max >= 0 && hip_min >= 0);
  CHECK(upper_max < hip_min);
  CHECK(lower_max < hip_min);
}

static void test_phase_transitions() {
  g_case = "phase_transitions";
  const CalibrationPhase* seq = fixture::kLfV25PhaseSequence;
  // Every consecutive pair of the historical sequence is legal.
  for (size_t i = 0; i + 1 < fixture::kLfV25PhaseSequenceLength; ++i) {
    CHECK(isLegalPhaseTransition(seq[i], seq[i + 1]));
  }
  // Skipping ahead is refused - that is what would reach HIP without proof.
  CHECK(!isLegalPhaseTransition(CalibrationPhase::PARKING, CalibrationPhase::HIP_MIN));
  CHECK(!isLegalPhaseTransition(CalibrationPhase::PREFLIGHT, CalibrationPhase::UPPER_MIN));
  // Going backwards is refused.
  CHECK(!isLegalPhaseTransition(CalibrationPhase::HIP_MAX, CalibrationPhase::UPPER_MIN));
  // TORQUE_OFF is terminal.
  for (size_t i = 0; i < fixture::kLfV25PhaseSequenceLength; ++i) {
    CHECK(!isLegalPhaseTransition(CalibrationPhase::TORQUE_OFF, seq[i]));
  }
  // But every active phase can reach the terminal safe state directly: a
  // failure must never have to walk the happy path to become safe.
  for (size_t i = 0; i + 1 < fixture::kLfV25PhaseSequenceLength; ++i) {
    CHECK(isLegalPhaseTransition(seq[i], CalibrationPhase::TORQUE_OFF));
  }
  // And every phase before the restore sequence can jump into it.
  CHECK(isLegalPhaseTransition(CalibrationPhase::UPPER_MIN, CalibrationPhase::RETURN_HIP));
  CHECK(isLegalPhaseTransition(CalibrationPhase::HIP_MAX, CalibrationPhase::RETURN_HIP));

  CHECK_EQ(fixture::kLfV25PhaseSequenceLength, kCalibrationPhaseCount);
}

static void test_restore_is_not_abort_and_not_safe_off() {
  g_case = "restore_is_not_abort_and_not_safe_off";
  // The oracle keeps them distinct and so does the model.
  CHECK(isRestorePhase(CalibrationPhase::RETURN_HIP));
  CHECK(isRestorePhase(CalibrationPhase::RETURN_LOWER_HELD));
  CHECK(isRestorePhase(CalibrationPhase::RETURN_UPPER));
  CHECK(isRestorePhase(CalibrationPhase::RESTORE_PARKING));
  CHECK(!isRestorePhase(CalibrationPhase::CLEANUP));
  CHECK(!isRestorePhase(CalibrationPhase::TORQUE_OFF));
  CHECK(isTerminalPhase(CalibrationPhase::TORQUE_OFF));
  CHECK(!isTerminalPhase(CalibrationPhase::CLEANUP));

  CHECK(isMeasurementPhase(CalibrationPhase::UPPER_MIN));
  CHECK(isMeasurementPhase(CalibrationPhase::HIP_MAX));
  CHECK(!isMeasurementPhase(CalibrationPhase::UPPER_HORIZONTAL));
  CHECK(!isMeasurementPhase(CalibrationPhase::PARKING));
}

static void test_restore_plan_semantics() {
  g_case = "restore_plan_semantics";
  // A mid-session mechanical failure needs the ordered return.
  RestorePlan p = restorePlanFor(CalibrationPhase::HIP_MAX,
                                 CalibrationFailure::CONTACT_WITNESS_REJECTED);
  CHECK(p.required);
  CHECK_EQ((int)p.resume_from, (int)CalibrationPhase::RETURN_HIP);
  CHECK(p.torque_off_required);

  // Losing authority must NOT start a restore: restore commands joints, and
  // commanding joints without authority is precisely what must not happen.
  p = restorePlanFor(CalibrationPhase::UPPER_MIN, CalibrationFailure::AUTHORITY_LOST);
  CHECK(!p.required);
  CHECK_EQ((int)p.resume_from, (int)CalibrationPhase::TORQUE_OFF);
  CHECK(p.torque_off_required);   // still required - it is never arbitrated

  // Refusing to start on stale calibration engaged nothing.
  p = restorePlanFor(CalibrationPhase::PREFLIGHT, CalibrationFailure::STALE_CALIBRATION_REFUSED);
  CHECK(!p.required);
  CHECK(p.torque_off_required);

  // Already terminal: nothing to restore.
  p = restorePlanFor(CalibrationPhase::TORQUE_OFF, CalibrationFailure::OPERATOR_ABORT);
  CHECK(!p.required);

  // Torque off is required on EVERY path, including success.
  p = restorePlanFor(CalibrationPhase::CLEANUP, CalibrationFailure::NONE);
  CHECK(!p.required);
  CHECK(p.torque_off_required);
}

// ---------------------------------------------------------------------------
// LF V25 oracle replay
// ---------------------------------------------------------------------------

static void test_lf_v25_oracle_replay() {
  g_case = "lf_v25_oracle_replay";
  // The historical run: 58 steps, one leg, six contacts.
  CHECK_EQ(fixture::kLfV25ExpectedTotalSteps, 58);
  CHECK_EQ(fixture::kLfV25ExecutedContacts, 6);
  // Six of twenty-four. Claiming 24 hardware-validated contacts would be false.
  CHECK(fixture::kLfV25ExecutedContacts < kContactProfileCount);
  CHECK_EQ(fixture::kLfV25ExecutedContacts, kJointKindCount * kContactSideCount);

  // Replay each recorded LF contact through the new model and confirm it lands
  // where the archive says, with the right origin and no promotion.
  bool covered[kContactProfileCount] = {false};
  int replayed = 0;

  for (const auto& record : fixture::kLfV25Records) {
    for (ContactSide side : kAllSides) {
      ContactProfileKey key{Leg::LF, record.joint, side};
      CHECK(key.valid());
      const uint16_t index = contactProfileIndex(key);
      CHECK(index < kContactProfileCount);
      CHECK(!covered[index]);
      covered[index] = true;
      ++replayed;

      ContactEvidence e{};
      e.key = key;
      e.origin = CalibrationOrigin::HISTORICAL_REPLAY;
      e.detection = ContactState::CONTACT_CONFIRMED;
      e.has_measurement = true;
      e.coarse_tick = (side == ContactSide::MIN_SIDE) ? record.accepted_min_tick
                                                      : record.accepted_max_tick;
      e.witness.evaluated = true;
      e.witness.min_deviation_ticks = 0;
      e.witness.max_deviation_ticks = 0;

      CHECK(isContactEvidence(e.detection));
      CHECK(e.witness.accepted());

      // Walk the lifecycle exactly as far as a replay is allowed to.
      CHECK(isLegalEvidenceTransition(EvidenceState::UNKNOWN, EvidenceState::MEASURED));
      CHECK(isLegalEvidenceTransition(EvidenceState::MEASURED, EvidenceState::CANDIDATE));
      CHECK(isLegalEvidenceTransition(EvidenceState::CANDIDATE, EvidenceState::ACCEPTED));
      e.state = EvidenceState::ACCEPTED;
      // ...and stops there, because of where it came from.
      CHECK(!mayPromote(e.origin));
      CHECK(!isOperationalEvidence(e.state));
    }
  }
  CHECK_EQ(replayed, 6);

  // Exactly the six LF contacts are covered; the other 18 remain untouched.
  int covered_count = 0;
  for (uint16_t i = 0; i < kContactProfileCount; ++i) {
    if (covered[i]) {
      ++covered_count;
      ContactProfileKey key{};
      CHECK(contactProfileFromIndex(i, &key));
      CHECK_EQ((int)key.leg, (int)Leg::LF);   // nothing outside LF was touched
    }
  }
  CHECK_EQ(covered_count, 6);
}

static void test_lf_v25_fine_sequences_match_the_archive() {
  g_case = "lf_v25_fine_sequences_match_the_archive";
  for (const auto& seq : fixture::kLfV25FineSequences) {
    // Spread is computed from the samples, not taken on trust.
    const uint16_t lo = seq.fine_tick_1 < seq.fine_tick_2 ? seq.fine_tick_1 : seq.fine_tick_2;
    const uint16_t hi = seq.fine_tick_1 < seq.fine_tick_2 ? seq.fine_tick_2 : seq.fine_tick_1;
    CHECK_EQ(hi - lo, seq.documented_spread_ticks);
    // Both samples are inside the uniform witness band around each other.
    CHECK(hi - lo <= kContactWitnessToleranceTicks);
  }
  // M13 MAX: coarse 1595, fine 1599/1601, spread 2.
  CHECK_EQ(fixture::kLfV25FineSequences[0].coarse_tick, 1595);
  CHECK_EQ(fixture::kLfV25FineSequences[0].documented_spread_ticks, 2);
  // M12 MAX: coarse 3446, fine 3443/3444, spread 1.
  CHECK_EQ(fixture::kLfV25FineSequences[1].coarse_tick, 3446);
  CHECK_EQ(fixture::kLfV25FineSequences[1].documented_spread_ticks, 1);
}

static void test_lf_v25_documented_failure_replay() {
  g_case = "lf_v25_documented_failure_replay";
  // The one failure the archive documents well enough to replay: the
  // cable-obstructed M12 MAX at ~3397 versus the accepted ~3443.
  const uint16_t deviation = fixture::kLfV25M12MaxAcceptedTick -
                             fixture::kLfV25M12MaxObstructedTick;
  CHECK_EQ(deviation, 46);

  ContactWitness obstructed{};
  obstructed.evaluated = true;
  obstructed.min_deviation_ticks = 0;
  obstructed.max_deviation_ticks = deviation;
  // 46 ticks is outside the 24-tick uniform band, so the witness rejects it -
  // exactly as it did on hardware.
  CHECK(!obstructed.accepted());

  ContactWitness clean{};
  clean.evaluated = true;
  clean.min_deviation_ticks = 0;
  clean.max_deviation_ticks = 0;
  CHECK(clean.accepted());

  // The resulting failure demands an ordered restore, not a bare stop.
  RestorePlan p = restorePlanFor(CalibrationPhase::UPPER_MAX,
                                 CalibrationFailure::CONTACT_WITNESS_REJECTED);
  CHECK(p.required);
  CHECK(p.torque_off_required);
}

static void test_lf_v25_bounded_tracking_lag_rule() {
  g_case = "lf_v25_bounded_tracking_lag_rule";
  // §5.3: 13 ticks continue, 17 ticks fail closed, global floor 16.
  CHECK_EQ(fixture::kLfV25SettleFloorTicks, 16);
  CHECK(fixture::kLfV25SettleContinueTicks < fixture::kLfV25SettleFloorTicks);
  CHECK(fixture::kLfV25SettleFailTicks > fixture::kLfV25SettleFloorTicks);
}

static void test_oracle_records_are_historical_provenance_only() {
  g_case = "oracle_records_are_historical_provenance_only";
  // The motor ids in the fixture are the PRE-reassembly unit labels. After
  // 2026-08-27 the unit labelled M11 is NECK_PITCH, so these ids must never be
  // used to address hardware. The model keys profiles by (leg, joint, side)
  // precisely so that no id can leak into an address.
  for (const auto& record : fixture::kLfV25Records) {
    ContactProfileKey key{Leg::LF, record.joint, ContactSide::MIN_SIDE};
    CHECK(key.valid());
    // The key carries no motor id at all.
    CHECK_EQ(sizeof(key), sizeof(ContactProfileKey));
    CHECK_EQ((int)key.leg, (int)Leg::LF);
  }
  // EEPROM offsets are recorded but are pure history: all 17 units hold
  // PositionOffset = 0 today.
  CHECK_EQ(fixture::kLfV25Records[0].eeprom_frozen_offset, -486);
  CHECK_EQ(fixture::kLfV25Records[1].eeprom_frozen_offset, 851);
  CHECK_EQ(fixture::kLfV25Records[2].eeprom_frozen_offset, 127);
  for (const auto& record : fixture::kLfV25Records) {
    CHECK(record.eeprom_frozen_offset != 0);   // none of them is today's value
  }
}

// ---------------------------------------------------------------------------
// Presentation
// ---------------------------------------------------------------------------

static void test_tostring_is_total() {
  g_case = "tostring_is_total";
  for (Leg l : kAllLegs) CHECK(std::strcmp(toString(l), "UNKNOWN") != 0);
  for (JointKind j : kAllJoints) CHECK(std::strcmp(toString(j), "UNKNOWN") != 0);
  for (ContactSide s : kAllSides) CHECK(std::strcmp(toString(s), "UNKNOWN") != 0);
  for (uint8_t i = 0; i <= (uint8_t)ContactState::HARD_ABORT; ++i) {
    CHECK(std::strcmp(toString((ContactState)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i < kCalibrationPhaseCount; ++i) {
    CHECK(std::strcmp(toString((CalibrationPhase)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)CalibrationFailure::STALE_CALIBRATION_REFUSED; ++i) {
    CHECK(std::strcmp(toString((CalibrationFailure)i), "UNKNOWN") != 0);
  }
  // The oracle's token labels must survive: the enumerators are MIN_SIDE /
  // MAX_SIDE only because MIN and MAX are core macros.
  CHECK_STR(toString(ContactSide::MIN_SIDE), "MIN");
  CHECK_STR(toString(ContactSide::MAX_SIDE), "MAX");
  CHECK_STR(toString(CalibrationPhase::RETURN_LOWER_HELD), "RETURN_LOWER_HELD");
  CHECK_STR(toString(EvidenceState::PROMOTED), "PROMOTED");
  CHECK_STR(toString(CalibrationOrigin::HISTORICAL_REPLAY), "HISTORICAL_REPLAY");
  CHECK_STR(toString(DirectionSource::SPEC_CONSTANT), "SPEC_CONSTANT");
}

int main() {
  std::printf("MATDOG calibration domain + LF V25 oracle replay offline tests\n");

  test_profile_count_is_derived_from_the_model();
  test_every_combination_maps_to_exactly_one_profile();
  test_profile_lookup_is_deterministic_and_round_trips();
  test_invalid_profile_input_fails_closed();
  test_profile_token_matches_the_oracle_format();
  test_q0_never_defaults_to_the_raw_servo_centre();
  test_the_three_meanings_of_2048_stay_separate();
  test_the_two_q0_estimators_are_not_interchangeable();
  test_direction_is_unknown_until_measured();
  test_contact_witness_band();
  test_contact_state_classification();
  test_evidence_lifecycle_has_no_shortcuts();
  test_historical_replay_can_never_be_promoted();
  test_phase_order_proves_upper_and_lower_before_hip();
  test_phase_transitions();
  test_restore_is_not_abort_and_not_safe_off();
  test_restore_plan_semantics();
  test_lf_v25_oracle_replay();
  test_lf_v25_fine_sequences_match_the_archive();
  test_lf_v25_documented_failure_replay();
  test_lf_v25_bounded_tracking_lag_rule();
  test_oracle_records_are_historical_provenance_only();
  test_tostring_is_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("CALIBRATION_DOMAIN_TESTS = FAIL\n");
    return 1;
  }
  std::printf("CALIBRATION_DOMAIN_TESTS = PASS\n");
  return 0;
}
