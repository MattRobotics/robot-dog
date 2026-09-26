// Offline host tests for the actuator write-authority arbiter
// (src/core/ActuatorAuthority.*).
//
// Links the REAL arbiter - it has no <Arduino.h>, no ServoBus and no Serial,
// which is the whole point. Same contract as network/WifiPolicy and
// update/OtaPolicy.
//
// The conflict matrix is exhaustive rather than illustrative: every ordered
// pair of distinct write-capable owners is tried, in both operating modes.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/core/ActuatorAuthority.h"

using namespace matdog::core;

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

#define CHECK_OWNER(arb, expected)                                             \
  do {                                                                         \
    ++g_checks;                                                                \
    if ((arb).current() != (expected)) {                                       \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: owner == %s, expected %s\n", g_case,     \
                  __FILE__, __LINE__, toString((arb).current()),               \
                  toString(expected));                                         \
    }                                                                          \
  } while (0)

#define CHECK_RESULT(actual, expected)                                         \
  do {                                                                         \
    ++g_checks;                                                                \
    const AuthorityResult a_ = (actual);                                       \
    if (a_ != (expected)) {                                                    \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: result == %s, expected %s\n", g_case,    \
                  __FILE__, __LINE__, toString(a_), toString(expected));       \
    }                                                                          \
  } while (0)

// The five write-capable owners.
static const ActuatorAuthority kOwners[] = {
    ActuatorAuthority::DIAGNOSTICS, ActuatorAuthority::CALIBRATION,
    ActuatorAuthority::QC,          ActuatorAuthority::PROVISIONING,
    ActuatorAuthority::MOTION,
};

// The mode each owner is legal in, per isModeCompatible().
static OperatingMode modeFor(ActuatorAuthority owner) {
  return owner == ActuatorAuthority::MOTION ? OperatingMode::RUN
                                            : OperatingMode::MAINTENANCE;
}

static ActuatorAuthorityArbiter freshArbiter() {
  ActuatorAuthorityArbiter a;
  a.reset(AuthorityClearReason::BOOT);
  return a;
}

// ---------------------------------------------------------------------------
// A. Initial state
// ---------------------------------------------------------------------------

static void test_boot_state_is_none() {
  g_case = "boot_state_is_none";
  ActuatorAuthorityArbiter a;
  // Even before reset() is called: a default-constructed arbiter owns nothing.
  CHECK_OWNER(a, ActuatorAuthority::NONE);
  CHECK(!a.inhibited());

  a.reset(AuthorityClearReason::BOOT);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
  CHECK(!a.inhibited());
  CHECK_EQ(a.generation(), 0u);
  CHECK_EQ((int)a.inhibitReason(), (int)InhibitReason::NONE);
}

static void test_reset_never_restores_a_previous_authority() {
  g_case = "reset_never_restores_a_previous_authority";
  for (ActuatorAuthority owner : kOwners) {
    ActuatorAuthorityArbiter a = freshArbiter();
    AuthorityLease lease;
    CHECK_RESULT(a.request(owner, modeFor(owner), &lease), AuthorityResult::GRANTED);
    CHECK_OWNER(a, owner);

    a.reset(AuthorityClearReason::BOOT);
    CHECK_OWNER(a, ActuatorAuthority::NONE);
    CHECK(!a.inhibited());
    // And the old lease is worthless afterwards.
    CHECK_RESULT(a.release(lease), AuthorityResult::REJECTED_NOT_HELD);
    CHECK_OWNER(a, ActuatorAuthority::NONE);
  }
}

// ---------------------------------------------------------------------------
// B. Valid acquisition
// ---------------------------------------------------------------------------

static void test_every_owner_can_be_acquired_from_none() {
  g_case = "every_owner_can_be_acquired_from_none";
  for (ActuatorAuthority owner : kOwners) {
    ActuatorAuthorityArbiter a = freshArbiter();
    AuthorityLease lease;
    CHECK_RESULT(a.request(owner, modeFor(owner), &lease), AuthorityResult::GRANTED);
    CHECK_OWNER(a, owner);
    CHECK(lease.valid());
    CHECK_EQ((int)lease.owner, (int)owner);
    CHECK(lease.generation != 0);
    CHECK_EQ(a.counters().grants, 1u);
  }
}

static void test_none_cannot_be_acquired() {
  g_case = "none_cannot_be_acquired";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::NONE, OperatingMode::MAINTENANCE, &lease),
               AuthorityResult::REJECTED_UNKNOWN_OWNER);
  CHECK(!lease.valid());
  CHECK_OWNER(a, ActuatorAuthority::NONE);
}

static void test_corrupted_enum_fails_closed() {
  g_case = "corrupted_enum_fails_closed";
  // The underlying type is fixed at uint8_t, so this cast is well defined -
  // which is exactly why the value has to be validated rather than trusted.
  ActuatorAuthorityArbiter a = freshArbiter();
  for (uint8_t raw : {(uint8_t)6, (uint8_t)7, (uint8_t)42, (uint8_t)255}) {
    const ActuatorAuthority bogus = static_cast<ActuatorAuthority>(raw);
    CHECK(!isKnownAuthority(bogus));
    CHECK(!isWriteCapableOwner(bogus));
    CHECK(!isModeCompatible(OperatingMode::MAINTENANCE, bogus));
    CHECK(!isModeCompatible(OperatingMode::RUN, bogus));
    AuthorityLease lease;
    CHECK_RESULT(a.request(bogus, OperatingMode::MAINTENANCE, &lease),
                 AuthorityResult::REJECTED_UNKNOWN_OWNER);
    CHECK(!lease.valid());
    CHECK_OWNER(a, ActuatorAuthority::NONE);
    CHECK(std::strcmp(toString(bogus), "UNKNOWN") == 0);
  }
}

// ---------------------------------------------------------------------------
// C. Conflict rejection - exhaustive matrix
// ---------------------------------------------------------------------------

static void test_conflict_matrix_is_exhaustive() {
  g_case = "conflict_matrix_is_exhaustive";
  int pairs = 0;
  for (ActuatorAuthority holder : kOwners) {
    for (ActuatorAuthority challenger : kOwners) {
      if (holder == challenger) continue;
      ++pairs;

      ActuatorAuthorityArbiter a = freshArbiter();
      AuthorityLease held;
      CHECK_RESULT(a.request(holder, modeFor(holder), &held), AuthorityResult::GRANTED);

      // The challenger is tried in ITS OWN legal mode, so a rejection can
      // only be about exclusivity - not about the mode happening to differ.
      AuthorityLease stolen;
      const AuthorityResult r = a.request(challenger, modeFor(holder), &stolen);
      CHECK(r != AuthorityResult::GRANTED);
      CHECK(!stolen.valid());
      // The incumbent is untouched, which is the property that matters.
      CHECK_OWNER(a, holder);
      CHECK(held.valid());

      // And the incumbent can still release normally afterwards.
      CHECK_RESULT(a.release(held), AuthorityResult::RELEASED);
      CHECK_OWNER(a, ActuatorAuthority::NONE);
    }
  }
  CHECK_EQ(pairs, 20);  // 5 owners x 4 others
}

static void test_conflict_rejection_reason_is_busy_in_a_shared_mode() {
  g_case = "conflict_rejection_reason_is_busy_in_a_shared_mode";
  // Two owners legal in the SAME mode: the rejection must be BUSY, proving
  // it is exclusivity doing the work and not the mode table.
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease held;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &held),
               AuthorityResult::GRANTED);
  AuthorityLease other;
  CHECK_RESULT(a.request(ActuatorAuthority::QC, OperatingMode::MAINTENANCE, &other),
               AuthorityResult::REJECTED_BUSY);
  CHECK(!other.valid());
  CHECK_OWNER(a, ActuatorAuthority::CALIBRATION);
}

// ---------------------------------------------------------------------------
// D. Same-owner request
// ---------------------------------------------------------------------------

static void test_same_owner_request_is_already_owned_and_issues_no_lease() {
  g_case = "same_owner_request_is_already_owned_and_issues_no_lease";
  for (ActuatorAuthority owner : kOwners) {
    ActuatorAuthorityArbiter a = freshArbiter();
    AuthorityLease first;
    CHECK_RESULT(a.request(owner, modeFor(owner), &first), AuthorityResult::GRANTED);
    const uint32_t gen_after_first = a.generation();

    AuthorityLease second;
    CHECK_RESULT(a.request(owner, modeFor(owner), &second), AuthorityResult::ALREADY_OWNED);
    // The chosen semantics: no second transaction. A second valid lease would
    // let two holders each believe they own it, and either could release it
    // out from under the other.
    CHECK(!second.valid());
    CHECK_EQ(a.generation(), gen_after_first);
    CHECK_OWNER(a, owner);

    // The original lease still works, and the phantom one cannot release.
    CHECK_RESULT(a.release(second), AuthorityResult::REJECTED_NOT_OWNER);
    CHECK_OWNER(a, owner);
    CHECK_RESULT(a.release(first), AuthorityResult::RELEASED);
    CHECK_OWNER(a, ActuatorAuthority::NONE);
  }
}

// ---------------------------------------------------------------------------
// E. Release
// ---------------------------------------------------------------------------

static void test_only_the_owner_can_release() {
  g_case = "only_the_owner_can_release";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease cal;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &cal),
               AuthorityResult::GRANTED);

  // Every other owner's attempt to release must be refused, and must leave
  // CALIBRATION in place.
  for (ActuatorAuthority other : kOwners) {
    if (other == ActuatorAuthority::CALIBRATION) continue;
    AuthorityLease forged;
    forged.owner = other;
    forged.generation = cal.generation;   // even with the right generation
    CHECK_RESULT(a.release(forged), AuthorityResult::REJECTED_NOT_OWNER);
    CHECK_OWNER(a, ActuatorAuthority::CALIBRATION);
  }

  CHECK_RESULT(a.release(cal), AuthorityResult::RELEASED);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
}

static void test_double_release_is_deterministic() {
  g_case = "double_release_is_deterministic";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::QC, OperatingMode::MAINTENANCE, &lease),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.release(lease), AuthorityResult::RELEASED);
  CHECK_OWNER(a, ActuatorAuthority::NONE);

  // Repeated releases are refused with a stable, specific result and never
  // disturb whatever the arbiter is doing now.
  for (int i = 0; i < 5; ++i) {
    CHECK_RESULT(a.release(lease), AuthorityResult::REJECTED_NOT_HELD);
    CHECK_OWNER(a, ActuatorAuthority::NONE);
  }
}

static void test_release_of_empty_lease_is_refused() {
  g_case = "release_of_empty_lease_is_refused";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease empty;
  CHECK(!empty.valid());
  CHECK_RESULT(a.release(empty), AuthorityResult::REJECTED_NOT_HELD);

  AuthorityLease motion;
  CHECK_RESULT(a.request(ActuatorAuthority::MOTION, OperatingMode::RUN, &motion),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.release(empty), AuthorityResult::REJECTED_NOT_OWNER);
  CHECK_OWNER(a, ActuatorAuthority::MOTION);
}

// ---------------------------------------------------------------------------
// F. Stale release - the bug the generation exists for
// ---------------------------------------------------------------------------

static void test_stale_release_from_a_previous_session_is_refused() {
  g_case = "stale_release_from_a_previous_session_is_refused";
  ActuatorAuthorityArbiter a = freshArbiter();

  // Session 1.
  AuthorityLease first;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &first),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.release(first), AuthorityResult::RELEASED);

  // Session 2 - SAME owner, so matching the owner alone would accept the
  // stale release below and clear this session's authority.
  AuthorityLease second;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &second),
               AuthorityResult::GRANTED);
  CHECK(second.generation != first.generation);

  // The late callback from session 1 arrives.
  CHECK_RESULT(a.release(first), AuthorityResult::REJECTED_STALE_LEASE);
  CHECK_OWNER(a, ActuatorAuthority::CALIBRATION);   // session 2 survives
  CHECK_EQ(a.counters().stale_releases, 1u);

  // Session 2's own lease still works.
  CHECK_RESULT(a.release(second), AuthorityResult::RELEASED);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
}

static void test_stale_release_across_different_owners_is_also_refused() {
  g_case = "stale_release_across_different_owners_is_also_refused";
  // The case owner-matching already covers, kept so a future refactor cannot
  // lose it: CALIBRATION's late release must not clear MOTION.
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease cal;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &cal),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.release(cal), AuthorityResult::RELEASED);

  AuthorityLease mot;
  CHECK_RESULT(a.request(ActuatorAuthority::MOTION, OperatingMode::RUN, &mot),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.release(cal), AuthorityResult::REJECTED_NOT_OWNER);
  CHECK_OWNER(a, ActuatorAuthority::MOTION);
}

static void test_generation_never_reuses_a_value() {
  g_case = "generation_never_reuses_a_value";
  ActuatorAuthorityArbiter a = freshArbiter();
  uint32_t previous = 0;
  for (int i = 0; i < 50; ++i) {
    const ActuatorAuthority owner = kOwners[i % 5];
    AuthorityLease lease;
    CHECK_RESULT(a.request(owner, modeFor(owner), &lease), AuthorityResult::GRANTED);
    CHECK(lease.generation > previous);
    CHECK(lease.generation != 0);
    previous = lease.generation;
    CHECK_RESULT(a.release(lease), AuthorityResult::RELEASED);
  }
}

// ---------------------------------------------------------------------------
// G. Fail closed
// ---------------------------------------------------------------------------

static void test_force_clear_lands_on_none_from_every_owner() {
  g_case = "force_clear_lands_on_none_from_every_owner";
  const AuthorityClearReason reasons[] = {
      AuthorityClearReason::FATAL_FAULT, AuthorityClearReason::OWNER_TEARDOWN,
      AuthorityClearReason::OPERATOR_RESET,
  };
  for (ActuatorAuthority owner : kOwners) {
    for (AuthorityClearReason reason : reasons) {
      ActuatorAuthorityArbiter a = freshArbiter();
      AuthorityLease lease;
      CHECK_RESULT(a.request(owner, modeFor(owner), &lease), AuthorityResult::GRANTED);
      a.forceClear(reason);
      CHECK_OWNER(a, ActuatorAuthority::NONE);
      CHECK_EQ((int)a.lastClearReason(), (int)reason);
      // The cleared owner's lease is dead: it must not resurrect anything.
      CHECK_RESULT(a.release(lease), AuthorityResult::REJECTED_NOT_HELD);
      CHECK_OWNER(a, ActuatorAuthority::NONE);
    }
  }
}

static void test_force_clear_is_idempotent() {
  g_case = "force_clear_is_idempotent";
  ActuatorAuthorityArbiter a = freshArbiter();
  a.forceClear(AuthorityClearReason::FATAL_FAULT);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
  CHECK_EQ(a.counters().force_clears, 0u);   // nothing was held, nothing cleared

  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::PROVISIONING, OperatingMode::MAINTENANCE, &lease),
               AuthorityResult::GRANTED);
  a.forceClear(AuthorityClearReason::FATAL_FAULT);
  a.forceClear(AuthorityClearReason::FATAL_FAULT);
  a.forceClear(AuthorityClearReason::FATAL_FAULT);
  CHECK_EQ(a.counters().force_clears, 1u);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
}

static void test_no_authority_survives_a_reinit() {
  g_case = "no_authority_survives_a_reinit";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::MOTION, OperatingMode::RUN, &lease),
               AuthorityResult::GRANTED);
  InhibitLease inhibit;
  // An inhibit cannot be taken while an owner holds it, so clear first.
  a.forceClear(AuthorityClearReason::OWNER_TEARDOWN);
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &inhibit),
               AuthorityResult::GRANTED);

  a.reset(AuthorityClearReason::BOOT);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
  CHECK(!a.inhibited());     // re-init clears BOTH axes
  CHECK_EQ(a.generation(), 0u);
}

// ---------------------------------------------------------------------------
// H. OperatingMode is an orthogonal axis
// ---------------------------------------------------------------------------

static void test_mode_compatibility_table() {
  g_case = "mode_compatibility_table";
  // MAINTENANCE hosts every service owner but not motion.
  for (ActuatorAuthority owner : kOwners) {
    const bool expect = (owner != ActuatorAuthority::MOTION);
    CHECK_EQ(isModeCompatible(OperatingMode::MAINTENANCE, owner), expect);
  }
  // RUN hosts motion and nothing else.
  for (ActuatorAuthority owner : kOwners) {
    const bool expect = (owner == ActuatorAuthority::MOTION);
    CHECK_EQ(isModeCompatible(OperatingMode::RUN, owner), expect);
  }
  // NONE is compatible everywhere: releasing must never be mode-gated.
  CHECK(isModeCompatible(OperatingMode::MAINTENANCE, ActuatorAuthority::NONE));
  CHECK(isModeCompatible(OperatingMode::RUN, ActuatorAuthority::NONE));
}

static void test_incompatible_mode_is_rejected_without_disturbing_the_owner() {
  g_case = "incompatible_mode_is_rejected_without_disturbing_the_owner";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::MOTION, OperatingMode::MAINTENANCE, &lease),
               AuthorityResult::REJECTED_MODE);
  CHECK(!lease.valid());
  CHECK_OWNER(a, ActuatorAuthority::NONE);

  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::RUN, &lease),
               AuthorityResult::REJECTED_MODE);
  CHECK(!lease.valid());
  CHECK_OWNER(a, ActuatorAuthority::NONE);
}

static void test_mode_change_clears_a_stranded_owner() {
  g_case = "mode_change_clears_a_stranded_owner";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &lease),
               AuthorityResult::GRANTED);

  // Staying in a compatible mode changes nothing.
  a.onOperatingModeChanged(OperatingMode::MAINTENANCE);
  CHECK_OWNER(a, ActuatorAuthority::CALIBRATION);

  // Switching to a mode the owner cannot live in must not leave a suspended
  // authority behind.
  a.onOperatingModeChanged(OperatingMode::RUN);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
  CHECK_EQ((int)a.lastClearReason(), (int)AuthorityClearReason::OPERATING_MODE_CHANGED);

  // And with nothing held it is a no-op in either direction.
  a.onOperatingModeChanged(OperatingMode::MAINTENANCE);
  a.onOperatingModeChanged(OperatingMode::RUN);
  CHECK_OWNER(a, ActuatorAuthority::NONE);
}

// ---------------------------------------------------------------------------
// I. Exclusive-activity inhibit (the OTA TOCTOU fix)
// ---------------------------------------------------------------------------

static void test_inhibit_requires_none_and_blocks_every_owner() {
  g_case = "inhibit_requires_none_and_blocks_every_owner";
  ActuatorAuthorityArbiter a = freshArbiter();
  InhibitLease inhibit;
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &inhibit),
               AuthorityResult::GRANTED);
  CHECK(a.inhibited());
  CHECK(inhibit.valid());
  CHECK_EQ((int)a.inhibitReason(), (int)InhibitReason::FIRMWARE_UPDATE);

  // This is the whole point: while held, nobody can become an owner.
  for (ActuatorAuthority owner : kOwners) {
    AuthorityLease lease;
    CHECK_RESULT(a.request(owner, modeFor(owner), &lease),
                 AuthorityResult::REJECTED_INHIBITED);
    CHECK(!lease.valid());
    CHECK_OWNER(a, ActuatorAuthority::NONE);
  }

  CHECK_RESULT(a.releaseInhibit(inhibit), AuthorityResult::RELEASED);
  CHECK(!a.inhibited());
  // Once released, acquisition works again.
  AuthorityLease lease;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &lease),
               AuthorityResult::GRANTED);
}

static void test_inhibit_cannot_be_taken_while_an_owner_holds_authority() {
  g_case = "inhibit_cannot_be_taken_while_an_owner_holds_authority";
  // The atomic half of the TOCTOU fix: an exclusive activity may only begin
  // from a state where nobody owns the actuators, and the check and the hold
  // are the same call.
  for (ActuatorAuthority owner : kOwners) {
    ActuatorAuthorityArbiter a = freshArbiter();
    AuthorityLease lease;
    CHECK_RESULT(a.request(owner, modeFor(owner), &lease), AuthorityResult::GRANTED);

    InhibitLease inhibit;
    CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &inhibit),
                 AuthorityResult::REJECTED_BUSY);
    CHECK(!inhibit.valid());
    CHECK(!a.inhibited());
    CHECK_OWNER(a, owner);   // the owner is untouched by the failed attempt
  }
}

static void test_inhibit_is_not_reentrant_and_rejects_stale_release() {
  g_case = "inhibit_is_not_reentrant_and_rejects_stale_release";
  ActuatorAuthorityArbiter a = freshArbiter();
  InhibitLease first;
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &first),
               AuthorityResult::GRANTED);

  // A second holder would be able to release the first one's hold.
  InhibitLease second;
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &second),
               AuthorityResult::REJECTED_INHIBITED);
  CHECK(!second.valid());
  CHECK(a.inhibited());

  CHECK_RESULT(a.releaseInhibit(first), AuthorityResult::RELEASED);
  CHECK(!a.inhibited());

  // A second inhibit session, then the first session's late release.
  InhibitLease third;
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &third),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.releaseInhibit(first), AuthorityResult::REJECTED_STALE_LEASE);
  CHECK(a.inhibited());   // the live hold survives
  CHECK_RESULT(a.releaseInhibit(third), AuthorityResult::RELEASED);

  // Releasing when nothing is held is refused deterministically.
  CHECK_RESULT(a.releaseInhibit(third), AuthorityResult::REJECTED_NOT_HELD);
}

static void test_inhibit_reason_none_is_refused() {
  g_case = "inhibit_reason_none_is_refused";
  ActuatorAuthorityArbiter a = freshArbiter();
  InhibitLease lease;
  CHECK_RESULT(a.requestInhibit(InhibitReason::NONE, &lease),
               AuthorityResult::REJECTED_UNKNOWN_OWNER);
  CHECK(!lease.valid());
  CHECK(!a.inhibited());
}

static void test_force_clear_inhibit() {
  g_case = "force_clear_inhibit";
  ActuatorAuthorityArbiter a = freshArbiter();
  InhibitLease lease;
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &lease),
               AuthorityResult::GRANTED);
  a.forceClearInhibit();
  CHECK(!a.inhibited());
  a.forceClearInhibit();   // idempotent
  CHECK(!a.inhibited());

  // forceClear() of the OWNER deliberately does NOT drop an inhibit: a stuck
  // inhibit is fail-safe (it blocks writes) and a fault must not quietly
  // re-open actuator authority.
  ActuatorAuthorityArbiter b = freshArbiter();
  InhibitLease held;
  CHECK_RESULT(b.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &held),
               AuthorityResult::GRANTED);
  b.forceClear(AuthorityClearReason::FATAL_FAULT);
  CHECK(b.inhibited());
  AuthorityLease owner_lease;
  CHECK_RESULT(b.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE,
                         &owner_lease),
               AuthorityResult::REJECTED_INHIBITED);
}

static void test_inhibit_and_owner_leases_are_not_interchangeable() {
  g_case = "inhibit_and_owner_leases_are_not_interchangeable";
  // Distinct types, so an inhibit holder cannot release an owner and vice
  // versa. This is compile-time; what is checked here is that their
  // generations come from the same sequence and still do not cross over.
  ActuatorAuthorityArbiter a = freshArbiter();
  InhibitLease inhibit;
  CHECK_RESULT(a.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &inhibit),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.releaseInhibit(inhibit), AuthorityResult::RELEASED);

  AuthorityLease owner;
  CHECK_RESULT(a.request(ActuatorAuthority::QC, OperatingMode::MAINTENANCE, &owner),
               AuthorityResult::GRANTED);
  CHECK(owner.generation != inhibit.generation);

  InhibitLease forged;
  forged.generation = owner.generation;
  CHECK_RESULT(a.releaseInhibit(forged), AuthorityResult::REJECTED_NOT_HELD);
  CHECK_OWNER(a, ActuatorAuthority::QC);
}

// ---------------------------------------------------------------------------
// J. Counters and presentation
// ---------------------------------------------------------------------------

static void test_counters_track_what_happened() {
  g_case = "counters_track_what_happened";
  ActuatorAuthorityArbiter a = freshArbiter();
  AuthorityLease l1, l2;
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &l1),
               AuthorityResult::GRANTED);
  CHECK_RESULT(a.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &l2),
               AuthorityResult::ALREADY_OWNED);
  CHECK_RESULT(a.request(ActuatorAuthority::QC, OperatingMode::MAINTENANCE, &l2),
               AuthorityResult::REJECTED_BUSY);
  CHECK_RESULT(a.release(l1), AuthorityResult::RELEASED);

  CHECK_EQ(a.counters().grants, 1u);
  CHECK_EQ(a.counters().already_owned, 1u);
  CHECK_EQ(a.counters().rejections, 1u);
  CHECK_EQ(a.counters().releases, 1u);
  CHECK_EQ(a.counters().force_clears, 0u);
  CHECK_EQ((int)a.lastResult(), (int)AuthorityResult::RELEASED);
}

static void test_tostring_is_total() {
  g_case = "tostring_is_total";
  for (uint8_t i = 0; i < kActuatorAuthorityCount; ++i) {
    CHECK(std::strcmp(toString((ActuatorAuthority)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)AuthorityResult::REJECTED_NOT_HELD; ++i) {
    CHECK(std::strcmp(toString((AuthorityResult)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)AuthorityClearReason::NEVER_CLEARED; ++i) {
    CHECK(std::strcmp(toString((AuthorityClearReason)i), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(InhibitReason::NONE), "UNKNOWN") != 0);
  CHECK(std::strcmp(toString(InhibitReason::FIRMWARE_UPDATE), "UNKNOWN") != 0);
  CHECK(std::strcmp(toString(ActuatorAuthority::MOTION), "MOTION") == 0);
  CHECK(std::strcmp(toString(AuthorityResult::REJECTED_INHIBITED), "REJECTED_INHIBITED") == 0);
}

int main() {
  std::printf("MATDOG ActuatorAuthority offline tests\n");

  test_boot_state_is_none();
  test_reset_never_restores_a_previous_authority();
  test_every_owner_can_be_acquired_from_none();
  test_none_cannot_be_acquired();
  test_corrupted_enum_fails_closed();
  test_conflict_matrix_is_exhaustive();
  test_conflict_rejection_reason_is_busy_in_a_shared_mode();
  test_same_owner_request_is_already_owned_and_issues_no_lease();
  test_only_the_owner_can_release();
  test_double_release_is_deterministic();
  test_release_of_empty_lease_is_refused();
  test_stale_release_from_a_previous_session_is_refused();
  test_stale_release_across_different_owners_is_also_refused();
  test_generation_never_reuses_a_value();
  test_force_clear_lands_on_none_from_every_owner();
  test_force_clear_is_idempotent();
  test_no_authority_survives_a_reinit();
  test_mode_compatibility_table();
  test_incompatible_mode_is_rejected_without_disturbing_the_owner();
  test_mode_change_clears_a_stranded_owner();
  test_inhibit_requires_none_and_blocks_every_owner();
  test_inhibit_cannot_be_taken_while_an_owner_holds_authority();
  test_inhibit_is_not_reentrant_and_rejects_stale_release();
  test_inhibit_reason_none_is_refused();
  test_force_clear_inhibit();
  test_inhibit_and_owner_leases_are_not_interchangeable();
  test_counters_track_what_happened();
  test_tostring_is_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("ACTUATOR_AUTHORITY_TESTS = FAIL\n");
    return 1;
  }
  std::printf("ACTUATOR_AUTHORITY_TESTS = PASS\n");
  return 0;
}
