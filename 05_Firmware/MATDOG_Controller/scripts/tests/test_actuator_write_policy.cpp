// Offline host tests for the Safe Actuator Layer decision core
// (src/actuator/ActuatorWritePolicy.*).
//
// Links the REAL policy AND the REAL arbiter AND the REAL calibration domain
// model - none of them has <Arduino.h>, a bus handle or Serial, which is the
// whole point. A mocked arbiter here would test the mock's idea of authority,
// not the shipped one.
//
// NO HOST TEST IN THIS FILE IS HARDWARE VALIDATION. Every case below proves a
// DECISION. There is no transport in the tree that could act on one: the
// runtime adapter is TO_IMPLEMENT (SAFE_ACTUATOR_LAYER.md §7), and the only
// actuator write that exists anywhere in the firmware is torque OFF inside
// ServoBus::safeOff().
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/actuator/ActuatorWritePolicy.h"

using namespace matdog;
using namespace matdog::actuator;
using matdog::calibration::CalibrationOrigin;
using matdog::calibration::EvidenceState;
using matdog::calibration::JointIdentity;
using matdog::calibration::JointKind;
using matdog::calibration::Leg;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::AuthorityLease;
using matdog::core::AuthorityResult;
using matdog::core::InhibitLease;
using matdog::core::InhibitReason;
using matdog::core::OperatingMode;

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
    const long a_ = (long)(actual);                                        \
    const long e_ = (long)(expected);                                      \
    if (a_ != e_) {                                                        \
      ++g_failures;                                                        \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,  \
                  __FILE__, __LINE__, #actual, a_, e_);                    \
    }                                                                      \
  } while (0)

#define CHECK_DECISION(actual, expected)                                        \
  do {                                                                          \
    ++g_checks;                                                                 \
    const WriteDecision a_ = (actual);                                          \
    const WriteDecision e_ = (expected);                                        \
    if (a_ != e_) {                                                             \
      ++g_failures;                                                             \
      std::printf("  FAIL [%s] %s:%d: %s == %s, expected %s\n", g_case,         \
                  __FILE__, __LINE__, #actual, toString(a_), toString(e_));     \
    }                                                                           \
  } while (0)

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

// The LF joints of the CURRENT installation, keyed by physical unit exactly as
// MATDOG_SERVO_ALLOCATION.yaml keys them. Not bus ids: after the 2026-08-27
// reassembly bus id 11 is still "LF lower" but answers as unit M33.
static JointIdentity joint(Leg leg, JointKind kind, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = kind;
  calibration::setPhysicalUnit(&id, unit);
  return id;
}

static JointIdentity lfLower() { return joint(Leg::LF, JointKind::LOWER, "M33"); }
static JointIdentity lfHip() { return joint(Leg::LF, JointKind::HIP, "M22"); }

static ActuatorCommand command(ActuatorOperation operation, JointIdentity id,
                               uint16_t target = 0) {
  ActuatorCommand cmd{};
  cmd.operation = operation;
  cmd.joint = id;
  cmd.target_tick = target;
  return cmd;
}

// A bound that WOULD qualify: measured on the current installation and promoted
// to operational calibration. Nothing in the repository produces one today -
// MATDOG_JOINT_CALIBRATION.yaml records {min: null, max: null} for all twelve
// leg joints - so this exists only to prove the accept path is reachable at all
// and that the bounds check is real rather than vacuous.
static JointLimit acceptedLimit(JointIdentity id, uint16_t lo, uint16_t hi) {
  JointLimit limit{};
  limit.identity = id;
  limit.state = EvidenceState::PROMOTED;
  limit.origin = CalibrationOrigin::LIVE_SESSION;
  limit.min_tick = lo;
  limit.max_tick = hi;
  limit.present = true;
  return limit;
}

// Grants `owner` and returns its lease. Fails the test if the arbiter refused.
static AuthorityLease grant(ActuatorAuthorityArbiter& arbiter, ActuatorAuthority owner,
                            OperatingMode mode) {
  AuthorityLease lease{};
  const AuthorityResult result = arbiter.request(owner, mode, &lease);
  CHECK(result == AuthorityResult::GRANTED);
  return lease;
}

// ---------------------------------------------------------------------------
// Authority binding
// ---------------------------------------------------------------------------

static void test_no_arbiter_is_a_refusal_not_a_free_pass() {
  g_case = "no arbiter";
  SafeActuatorPolicy policy;  // begin() never called
  ActuatorTransaction txn{};
  AuthorityLease lease{};
  lease.owner = ActuatorAuthority::CALIBRATION;
  lease.generation = 1;

  CHECK_DECISION(
      policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                  OperatingMode::MAINTENANCE, &txn),
      WriteDecision::REJECT_NO_ARBITER);
  CHECK_EQ((int)txn.state, (int)TransactionState::REJECTED);
  CHECK_EQ(txn.id, 0u);
}

static void test_no_authority_rejects() {
  g_case = "no authority";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);

  // An empty lease, and an arbiter holding NONE.
  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()),
                             AuthorityLease{}, OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_NO_AUTHORITY);

  // A structurally valid lease that the arbiter has never issued.
  AuthorityLease invented{};
  invented.owner = ActuatorAuthority::CALIBRATION;
  invented.generation = 7;
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), invented,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_NO_AUTHORITY);
}

static void test_wrong_owner_rejects() {
  g_case = "wrong owner";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);

  const AuthorityLease calibration = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                           OperatingMode::MAINTENANCE);

  // A lease claiming a different owner, carrying the CURRENT generation - so
  // only the owner comparison can catch it.
  AuthorityLease impostor = calibration;
  impostor.owner = ActuatorAuthority::QC;

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), impostor,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_WRONG_OWNER);
}

static void test_stale_generation_rejects() {
  g_case = "stale generation";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);

  // Session one, released. Session two, same owner: the owner matches, so the
  // generation is the only thing standing between a late caller from session
  // one and a write in session two.
  const AuthorityLease first = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  CHECK(arbiter.release(first) == AuthorityResult::RELEASED);
  const AuthorityLease second = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                      OperatingMode::MAINTENANCE);
  CHECK(first.generation != second.generation);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), first,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_STALE_GENERATION);
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), second,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::ACCEPT);
}

static void test_inhibit_rejects_every_operation() {
  g_case = "inhibited";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);

  InhibitLease hold{};
  CHECK(arbiter.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &hold) ==
        AuthorityResult::GRANTED);

  // An inhibit can only be taken from NONE, so no real lease can coexist with
  // one; a caller holding a lease from before the hold is exactly the case.
  AuthorityLease stale{};
  stale.owner = ActuatorAuthority::CALIBRATION;
  stale.generation = 1;

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), stale,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_INHIBITED);
}

static void test_operating_mode_mismatch_rejects() {
  g_case = "mode mismatch";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);

  // CALIBRATION is legal in MAINTENANCE and illegal in RUN: RUN means a
  // deterministic motion loop may be running, and a service owner would starve
  // it. The lease is genuine; only the mode is wrong.
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::RUN, &txn),
                 WriteDecision::REJECT_MODE);

  // And the mirror image: MOTION is legal only in RUN.
  ActuatorAuthorityArbiter motion_arbiter;
  motion_arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy motion_policy;
  motion_policy.begin(&motion_arbiter);
  const AuthorityLease motion = grant(motion_arbiter, ActuatorAuthority::MOTION,
                                      OperatingMode::RUN);
  CHECK_DECISION(motion_policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()),
                                    motion, OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_MODE);
}

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

static void test_invalid_and_corrupt_operations_reject() {
  g_case = "invalid operation";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};

  // NONE is the absence of an operation, never an operation.
  CHECK_DECISION(policy.plan(command(ActuatorOperation::NONE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_UNKNOWN_OPERATION);

  // A corrupted byte cast to the enum. Well defined for a fixed uint8_t base,
  // which is precisely why it must be checked.
  const ActuatorOperation corrupt = static_cast<ActuatorOperation>(200);
  CHECK(!isKnownOperation(corrupt));
  CHECK_DECISION(policy.plan(command(corrupt, lfLower()), lease, OperatingMode::MAINTENANCE,
                             &txn),
                 WriteDecision::REJECT_UNKNOWN_OPERATION);
}

static void test_invalid_joint_identity_rejects() {
  g_case = "invalid joint";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};

  // A slot with no physical unit label. The slot alone stopped being an
  // identity on 2026-08-27, so it is refused for EVERY operation, not only for
  // the ones that consult a bound.
  JointIdentity anonymous{};
  anonymous.leg = Leg::LF;
  anonymous.joint = JointKind::LOWER;
  CHECK(!anonymous.unitKnown());
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, anonymous), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_INVALID_JOINT);

  JointIdentity corrupt = lfLower();
  corrupt.leg = static_cast<Leg>(9);
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, corrupt), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_INVALID_JOINT);
}

static void test_calibration_lease_accepts_an_eligible_calibration_operation() {
  g_case = "calibration accepts";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::ACCEPT);
  CHECK(txn.planned());
  CHECK(txn.id != 0);
  CHECK_EQ(txn.policy_epoch, policy.epoch());
  CHECK_DECISION(policy.commit(&txn), WriteDecision::ACCEPT);
  CHECK_EQ((int)txn.state, (int)TransactionState::COMMITTED);
}

static void test_motion_lease_cannot_issue_a_calibration_only_operation() {
  g_case = "motion vs calibration-only";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease motion = grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  // A contact probe drives a joint into a mechanical endstop on purpose. That
  // is a calibration measurement; a motion owner must not be able to ask for it
  // even with a perfectly valid lease in the right mode.
  CHECK_DECISION(
      policy.plan(command(ActuatorOperation::CALIBRATION_CONTACT_PROBE, lfLower(), 2000),
                  motion, OperatingMode::RUN, &txn),
      WriteDecision::REJECT_OPERATION_NOT_PERMITTED);

  // The same owner CAN ask for torque - it fails later, on limits, not on
  // eligibility, which proves the refusal above is about the operation class.
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), motion,
                             OperatingMode::RUN, &txn),
                 WriteDecision::ACCEPT);
}

static void test_owner_operation_matrix_is_exhaustive() {
  g_case = "owner x operation matrix";
  for (ActuatorAuthority owner :
       {ActuatorAuthority::NONE, ActuatorAuthority::DIAGNOSTICS, ActuatorAuthority::CALIBRATION,
        ActuatorAuthority::QC, ActuatorAuthority::PROVISIONING, ActuatorAuthority::MOTION}) {
    for (ActuatorOperation operation :
         {ActuatorOperation::NONE, ActuatorOperation::TORQUE_ENABLE,
          ActuatorOperation::POSITION_COMMAND, ActuatorOperation::CALIBRATION_CONTACT_PROBE}) {
      const bool permitted = operationPermittedForOwner(owner, operation);
      bool expected = false;
      if (operation != ActuatorOperation::NONE) {
        if (owner == ActuatorAuthority::CALIBRATION) {
          expected = true;
        } else if (owner == ActuatorAuthority::MOTION) {
          expected = (operation != ActuatorOperation::CALIBRATION_CONTACT_PROBE);
        }
      }
      if (permitted != expected) {
        ++g_failures;
        std::printf("  FAIL [%s] owner=%s operation=%s permitted=%d expected=%d\n", g_case,
                    core::toString(owner), toString(operation), (int)permitted, (int)expected);
      }
      ++g_checks;
    }
  }

  // A corrupted owner is refused for every operation.
  const ActuatorAuthority corrupt = static_cast<ActuatorAuthority>(77);
  for (ActuatorOperation operation :
       {ActuatorOperation::TORQUE_ENABLE, ActuatorOperation::POSITION_COMMAND,
        ActuatorOperation::CALIBRATION_CONTACT_PROBE}) {
    CHECK(!operationPermittedForOwner(corrupt, operation));
  }
}

// ---------------------------------------------------------------------------
// Limits and provenance
// ---------------------------------------------------------------------------

static void test_missing_current_limits_reject() {
  g_case = "no accepted limits";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  // This is the SHIPPED state: the limit table is empty because nothing in the
  // repository qualifies to fill it.
  CHECK(policy.limits().empty());

  ActuatorTransaction txn{};
  for (ActuatorOperation operation :
       {ActuatorOperation::POSITION_COMMAND, ActuatorOperation::CALIBRATION_CONTACT_PROBE}) {
    CHECK_DECISION(policy.plan(command(operation, lfLower(), 2048), lease,
                               OperatingMode::MAINTENANCE, &txn),
                   WriteDecision::REJECT_NO_ACCEPTED_LIMITS);
  }

  // 2048 is not a permissive default either. The raw servo centre is a
  // servo-level fact that says nothing about joint zero - it must not become a
  // bound by accident.
  CHECK_DECISION(policy.plan(command(ActuatorOperation::POSITION_COMMAND, lfLower(),
                                     calibration::kServoRawCenter),
                             lease, OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_NO_ACCEPTED_LIMITS);
}

static void test_historical_limits_can_never_be_admitted() {
  g_case = "historical-only limits";
  ActuatorLimitTable table;

  // A replayed bound. Complete, ordered, promoted - and still refused, because
  // a replay describes a servo that is no longer in that joint.
  JointLimit historical = acceptedLimit(lfLower(), 1800, 2300);
  historical.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  CHECK(!historical.usableProvenance());
  CHECK(!table.admit(historical));
  CHECK(table.empty());

  // Measured on the current installation but not yet operational calibration.
  for (EvidenceState state : {EvidenceState::UNKNOWN, EvidenceState::MEASURED,
                              EvidenceState::CANDIDATE, EvidenceState::ACCEPTED,
                              EvidenceState::REJECTED}) {
    JointLimit unpromoted = acceptedLimit(lfLower(), 1800, 2300);
    unpromoted.state = state;
    CHECK(!unpromoted.usableProvenance());
    CHECK(!table.admit(unpromoted));
  }
  CHECK(table.empty());

  // Absent, corrupt or anonymous bounds are refused too - a missing bound is
  // never "the full range".
  JointLimit absent = acceptedLimit(lfLower(), 1800, 2300);
  absent.present = false;
  CHECK(!table.admit(absent));

  JointLimit inverted = acceptedLimit(lfLower(), 2300, 1800);
  CHECK(!inverted.ordered());
  CHECK(!table.admit(inverted));

  JointLimit anonymous = acceptedLimit(lfLower(), 1800, 2300);
  std::memset(anonymous.identity.physical_unit, 0, sizeof(anonymous.identity.physical_unit));
  CHECK(!table.admit(anonymous));
  CHECK(table.empty());
}

static void test_a_limit_applies_only_to_the_same_slot_and_the_same_unit() {
  g_case = "limit identity";
  ActuatorLimitTable table;
  const JointLimit limit = acceptedLimit(lfLower(), 1800, 2300);
  CHECK(table.admit(limit));
  CHECK_EQ(table.size(), 1);

  // Same slot, different physical servo: exactly what the 2026-08-27 reassembly
  // made unsafe.
  CHECK(table.find(joint(Leg::LF, JointKind::LOWER, "ELR01")) == nullptr);
  // Same physical servo, different slot.
  CHECK(table.find(joint(Leg::RF, JointKind::LOWER, "M33")) == nullptr);
  // Both agree.
  CHECK(table.find(lfLower()) != nullptr);
  // Anonymous lookups match nothing.
  JointIdentity anonymous{};
  anonymous.leg = Leg::LF;
  anonymous.joint = JointKind::LOWER;
  CHECK(table.find(anonymous) == nullptr);

  // Re-admitting the same joint replaces rather than duplicates.
  CHECK(table.admit(acceptedLimit(lfLower(), 1900, 2200)));
  CHECK_EQ(table.size(), 1);
  CHECK_EQ(table.find(lfLower())->min_tick, 1900);

  // A second joint is a second entry.
  CHECK(table.admit(acceptedLimit(lfHip(), 1000, 3000)));
  CHECK_EQ(table.size(), 2);
}

static void test_target_bounds_are_enforced_in_both_directions() {
  g_case = "target bounds";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  CHECK(policy.limits().admit(acceptedLimit(lfLower(), 1800, 2300)));
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  struct Case {
    uint16_t target;
    WriteDecision expected;
  };
  for (const Case& c : {Case{1799, WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS},
                        Case{1800, WriteDecision::ACCEPT},
                        Case{2048, WriteDecision::ACCEPT},
                        Case{2300, WriteDecision::ACCEPT},
                        Case{2301, WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS},
                        Case{0, WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS},
                        Case{4095, WriteDecision::REJECT_TARGET_OUT_OF_BOUNDS}}) {
    CHECK_DECISION(policy.plan(command(ActuatorOperation::POSITION_COMMAND, lfLower(), c.target),
                               lease, OperatingMode::MAINTENANCE, &txn),
                   c.expected);
    policy.abort(&txn);
  }

  // A bound admitted for one joint authorises nothing on another.
  CHECK_DECISION(policy.plan(command(ActuatorOperation::POSITION_COMMAND, lfHip(), 2048), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_NO_ACCEPTED_LIMITS);
}

static void test_reset_clears_the_limit_table() {
  g_case = "reset clears limits";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  CHECK(policy.limits().admit(acceptedLimit(lfLower(), 1800, 2300)));
  CHECK_EQ(policy.limits().size(), 1);

  policy.reset();
  CHECK(policy.limits().empty());

  // And a re-begin() lands empty too: bounds are never restored across a
  // re-init, the same way authority is never restored.
  policy.begin(&arbiter);
  CHECK(policy.limits().empty());
}

// ---------------------------------------------------------------------------
// Transaction semantics
// ---------------------------------------------------------------------------

static void test_authority_lost_between_plan_and_commit_rejects() {
  g_case = "authority lost before commit";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  // (a) the owner is force-cleared under the transaction.
  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::ACCEPT);
  arbiter.forceClear(AuthorityClearReason::FATAL_FAULT);
  CHECK_DECISION(policy.commit(&txn), WriteDecision::REJECT_NO_AUTHORITY);
  CHECK_EQ((int)txn.state, (int)TransactionState::REJECTED);

  // (b) the lease is released and re-acquired: same owner, new generation.
  const AuthorityLease again = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  ActuatorTransaction txn_b{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), again,
                             OperatingMode::MAINTENANCE, &txn_b),
                 WriteDecision::ACCEPT);
  CHECK(arbiter.release(again) == AuthorityResult::RELEASED);
  const AuthorityLease third = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);
  (void)third;
  CHECK_DECISION(policy.commit(&txn_b), WriteDecision::REJECT_STALE_GENERATION);

  // (c) an operating-mode change strands the owner under the transaction.
  ActuatorAuthorityArbiter mode_arbiter;
  mode_arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy mode_policy;
  mode_policy.begin(&mode_arbiter);
  const AuthorityLease service = grant(mode_arbiter, ActuatorAuthority::CALIBRATION,
                                       OperatingMode::MAINTENANCE);
  ActuatorTransaction txn_c{};
  CHECK_DECISION(mode_policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), service,
                                  OperatingMode::MAINTENANCE, &txn_c),
                 WriteDecision::ACCEPT);
  mode_arbiter.onOperatingModeChanged(OperatingMode::RUN);
  CHECK_DECISION(mode_policy.commit(&txn_c), WriteDecision::REJECT_NO_AUTHORITY);

  // (d) an exclusivity hold appears under the transaction.
  ActuatorAuthorityArbiter ota_arbiter;
  ota_arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy ota_policy;
  ota_policy.begin(&ota_arbiter);
  const AuthorityLease held = grant(ota_arbiter, ActuatorAuthority::CALIBRATION,
                                    OperatingMode::MAINTENANCE);
  ActuatorTransaction txn_d{};
  CHECK_DECISION(ota_policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), held,
                                 OperatingMode::MAINTENANCE, &txn_d),
                 WriteDecision::ACCEPT);
  ota_arbiter.forceClear(AuthorityClearReason::OWNER_TEARDOWN);
  InhibitLease hold{};
  CHECK(ota_arbiter.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &hold) ==
        AuthorityResult::GRANTED);
  CHECK_DECISION(ota_policy.commit(&txn_d), WriteDecision::REJECT_INHIBITED);
}

static void test_transaction_replay_rejects() {
  g_case = "replay";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::ACCEPT);

  // A caller squirrels away a copy while the transaction is still live.
  const ActuatorTransaction copy = txn;

  CHECK_DECISION(policy.commit(&txn), WriteDecision::ACCEPT);

  // Committing the same object again: it is COMMITTED, so not planned.
  CHECK_DECISION(policy.commit(&txn), WriteDecision::REJECT_TRANSACTION_STATE);

  // Committing the copy: it still LOOKS planned and carries the right epoch,
  // and it is refused on identity - the policy consumed that id already.
  ActuatorTransaction replay = copy;
  CHECK(replay.planned());
  CHECK_EQ(replay.policy_epoch, policy.epoch());
  CHECK_DECISION(policy.commit(&replay), WriteDecision::REJECT_TRANSACTION_STATE);
  CHECK_EQ((int)replay.state, (int)TransactionState::REJECTED);
}

static void test_only_one_transaction_may_be_outstanding() {
  g_case = "one outstanding plan";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction first{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &first),
                 WriteDecision::ACCEPT);
  CHECK(policy.hasOutstandingTransaction());

  ActuatorTransaction second{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfHip()), lease,
                             OperatingMode::MAINTENANCE, &second),
                 WriteDecision::REJECT_TRANSACTION_STATE);
  CHECK_EQ(second.id, 0u);

  // The first still commits: the refusal protected it rather than replacing it.
  CHECK_DECISION(policy.commit(&first), WriteDecision::ACCEPT);
  CHECK(!policy.hasOutstandingTransaction());

  // Now a new plan is possible again.
  ActuatorTransaction third{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfHip()), lease,
                             OperatingMode::MAINTENANCE, &third),
                 WriteDecision::ACCEPT);
  CHECK(third.id != first.id);
}

static void test_aborted_transaction_cannot_resume() {
  g_case = "abort is final";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::ACCEPT);
  policy.abort(&txn);
  CHECK_EQ((int)txn.state, (int)TransactionState::ABORTED);
  CHECK(!policy.hasOutstandingTransaction());

  CHECK_DECISION(policy.commit(&txn), WriteDecision::REJECT_TRANSACTION_STATE);

  // Forcing the state back does not resurrect it: the id is no longer the
  // outstanding one.
  ActuatorTransaction forged = txn;
  forged.state = TransactionState::PLANNED;
  CHECK_DECISION(policy.commit(&forged), WriteDecision::REJECT_TRANSACTION_STATE);
}

static void test_reset_invalidates_outstanding_transactions() {
  g_case = "reset invalidates";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::ACCEPT);
  const uint32_t before = policy.epoch();

  policy.reset();
  CHECK(policy.epoch() != before);
  CHECK(!policy.hasOutstandingTransaction());

  CHECK_DECISION(policy.commit(&txn), WriteDecision::REJECT_STALE_EPOCH);
  CHECK_EQ((int)txn.state, (int)TransactionState::REJECTED);

  // A zero-initialized transaction must not match a freshly reset policy.
  ActuatorTransaction zeroed{};
  CHECK_DECISION(policy.commit(&zeroed), WriteDecision::REJECT_TRANSACTION_STATE);
}

static void test_a_rejected_plan_produces_no_committable_transaction() {
  g_case = "rejected plan is inert";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);

  ActuatorTransaction txn{};
  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()),
                             AuthorityLease{}, OperatingMode::MAINTENANCE, &txn),
                 WriteDecision::REJECT_NO_AUTHORITY);
  CHECK_EQ(txn.id, 0u);
  CHECK(!txn.planned());
  CHECK_EQ((int)txn.state, (int)TransactionState::REJECTED);
  // It carries the reason so a caller need not re-derive it.
  CHECK_DECISION(txn.last_decision, WriteDecision::REJECT_NO_AUTHORITY);
  CHECK(!policy.hasOutstandingTransaction());
  CHECK_DECISION(policy.commit(&txn), WriteDecision::REJECT_TRANSACTION_STATE);
}

static void test_null_pointers_are_refusals() {
  g_case = "null pointers";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  CHECK_DECISION(policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                             OperatingMode::MAINTENANCE, nullptr),
                 WriteDecision::REJECT_TRANSACTION_STATE);
  CHECK_DECISION(policy.commit(nullptr), WriteDecision::REJECT_TRANSACTION_STATE);
  policy.abort(nullptr);  // must not crash
  CHECK(!policy.hasOutstandingTransaction());
}

// ---------------------------------------------------------------------------
// SAFE_OFF stays outside this gate
// ---------------------------------------------------------------------------

static void test_safe_off_is_not_expressible_through_this_layer() {
  g_case = "SAFE_OFF is outside";
  // The structural guarantee, as a test rather than a promise: there is no
  // operation class for REMOVING torque, so no edit to this policy can gate a
  // safety de-escalation on authority, mode, limits or a transaction. The
  // ungated path stays ServoBus::safeOff(), which static_audit.py keeps free of
  // any reference to authority or to this layer.
  CHECK_EQ(kActuatorOperationCount, 4);

  int command_operations = 0;
  for (uint8_t raw = 0; raw < kActuatorOperationCount; ++raw) {
    const ActuatorOperation operation = static_cast<ActuatorOperation>(raw);
    CHECK(isKnownOperation(operation));
    if (isCommandOperation(operation)) ++command_operations;

    // No class may name a torque-off / safe-off / disable operation.
    const char* name = toString(operation);
    CHECK(std::strstr(name, "SAFE_OFF") == nullptr);
    CHECK(std::strstr(name, "TORQUE_OFF") == nullptr);
    CHECK(std::strstr(name, "DISABLE") == nullptr);
  }
  CHECK_EQ(command_operations, 3);

  // TORQUE_ENABLE means APPLY torque and carries no target that could encode
  // "off": the operations that carry a target are the position-class ones.
  CHECK(!operationNeedsTarget(ActuatorOperation::TORQUE_ENABLE));
  CHECK(operationNeedsTarget(ActuatorOperation::POSITION_COMMAND));
  CHECK(operationNeedsTarget(ActuatorOperation::CALIBRATION_CONTACT_PROBE));
  CHECK(!operationNeedsTarget(ActuatorOperation::NONE));
}

static void test_no_persistent_write_class_exists() {
  g_case = "provisioning stays out";
  // The EEPROM/provisioning boundary, enforced by the type rather than by a
  // runtime refusal: no class here can name a persistent write, so this layer
  // cannot become the route for one while the repository still records its
  // owner as TO_DESIGN.
  for (uint8_t raw = 0; raw < kActuatorOperationCount; ++raw) {
    const char* name = toString(static_cast<ActuatorOperation>(raw));
    CHECK(std::strstr(name, "EEPROM") == nullptr);
    CHECK(std::strstr(name, "ID_WRITE") == nullptr);
    CHECK(std::strstr(name, "OFFSET") == nullptr);
    CHECK(std::strstr(name, "PERSIST") == nullptr);
  }
  // And PROVISIONING, which owns that side of the boundary, may issue nothing
  // through this layer.
  for (ActuatorOperation operation :
       {ActuatorOperation::TORQUE_ENABLE, ActuatorOperation::POSITION_COMMAND,
        ActuatorOperation::CALIBRATION_CONTACT_PROBE}) {
    CHECK(!operationPermittedForOwner(ActuatorAuthority::PROVISIONING, operation));
  }
}

// ---------------------------------------------------------------------------
// Bookkeeping
// ---------------------------------------------------------------------------

static void test_counters_track_what_happened() {
  g_case = "counters";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);  // one reset
  const AuthorityLease lease = grant(arbiter, ActuatorAuthority::CALIBRATION,
                                     OperatingMode::MAINTENANCE);

  ActuatorTransaction txn{};
  policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
              OperatingMode::MAINTENANCE, &txn);
  policy.commit(&txn);

  ActuatorTransaction rejected{};
  policy.plan(command(ActuatorOperation::NONE, lfLower()), lease, OperatingMode::MAINTENANCE,
              &rejected);

  ActuatorTransaction aborted{};
  policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfHip()), lease,
              OperatingMode::MAINTENANCE, &aborted);
  policy.abort(&aborted);

  ActuatorTransaction replayed = txn;
  policy.commit(&replayed);

  const ActuatorPolicyCounters& c = policy.counters();
  CHECK_EQ(c.plans, 2u);
  CHECK_EQ(c.plan_rejections, 1u);
  CHECK_EQ(c.commits, 1u);
  CHECK_EQ(c.commit_rejections, 1u);
  CHECK_EQ(c.aborts, 1u);
  CHECK_EQ(c.resets, 1u);
  CHECK_DECISION(policy.lastDecision(), WriteDecision::REJECT_TRANSACTION_STATE);
}

static void test_tostring_is_total() {
  g_case = "toString totality";
  for (uint8_t raw = 0; raw < kActuatorOperationCount; ++raw) {
    CHECK(std::strcmp(toString(static_cast<ActuatorOperation>(raw)), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<ActuatorOperation>(99)), "UNKNOWN") == 0);

  for (uint8_t raw = 0; raw <= (uint8_t)WriteDecision::REJECT_STALE_EPOCH; ++raw) {
    CHECK(std::strcmp(toString(static_cast<WriteDecision>(raw)), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<WriteDecision>(99)), "UNKNOWN") == 0);

  for (uint8_t raw = 0; raw <= (uint8_t)TransactionState::ABORTED; ++raw) {
    CHECK(std::strcmp(toString(static_cast<TransactionState>(raw)), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<TransactionState>(99)), "UNKNOWN") == 0);
}

int main() {
  std::printf("MATDOG Safe Actuator Layer write-policy offline tests\n");

  test_no_arbiter_is_a_refusal_not_a_free_pass();
  test_no_authority_rejects();
  test_wrong_owner_rejects();
  test_stale_generation_rejects();
  test_inhibit_rejects_every_operation();
  test_operating_mode_mismatch_rejects();

  test_invalid_and_corrupt_operations_reject();
  test_invalid_joint_identity_rejects();
  test_calibration_lease_accepts_an_eligible_calibration_operation();
  test_motion_lease_cannot_issue_a_calibration_only_operation();
  test_owner_operation_matrix_is_exhaustive();

  test_missing_current_limits_reject();
  test_historical_limits_can_never_be_admitted();
  test_a_limit_applies_only_to_the_same_slot_and_the_same_unit();
  test_target_bounds_are_enforced_in_both_directions();
  test_reset_clears_the_limit_table();

  test_authority_lost_between_plan_and_commit_rejects();
  test_transaction_replay_rejects();
  test_only_one_transaction_may_be_outstanding();
  test_aborted_transaction_cannot_resume();
  test_reset_invalidates_outstanding_transactions();
  test_a_rejected_plan_produces_no_committable_transaction();
  test_null_pointers_are_refusals();

  test_safe_off_is_not_expressible_through_this_layer();
  test_no_persistent_write_class_exists();

  test_counters_track_what_happened();
  test_tostring_is_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("SAFE_ACTUATOR_POLICY_TESTS = FAIL\n");
    return 1;
  }
  std::printf("SAFE_ACTUATOR_POLICY_TESTS = PASS\n");
  return 0;
}
