// Offline host tests for HttpMailbox (src/network/HttpMailbox.h) — I7/I8
// hardening, 2026-09-25: the pure correlation logic behind HttpTransport's
// single-slot request/response mailbox, which decides whether a computed
// response is still wanted or must be dropped because its dispatch()
// already timed out and moved on.
//
// This is the state machine semantics the operator asked to be offline-
// testable; the real FreeRTOS semaphore scheduling around it is not (no
// scheduler on the host) — see HttpTransport.h and HttpMailbox.h for the
// full protocol this class is one piece of.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>

#include "../../src/network/HttpMailbox.h"

using namespace matdog::network;

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

namespace {

void test_initial_state_awaits_nothing() {
  g_case = "initial state";
  HttpMailbox mailbox;
  CHECK(!mailbox.awaitingResponse());
}

void test_begin_dispatch_marks_awaiting() {
  g_case = "beginDispatch";
  HttpMailbox mailbox;
  mailbox.beginDispatch();
  CHECK(mailbox.awaitingResponse());
}

void test_happy_path_deliver_then_reset() {
  g_case = "happy path";
  HttpMailbox mailbox;
  mailbox.beginDispatch();
  CHECK(mailbox.awaitingResponse());  // Controller thread checks this -> signals
  mailbox.delivered();
  CHECK(!mailbox.awaitingResponse());  // clean slate for the next dispatch
}

void test_abandon_clears_awaiting_before_the_response_is_computed() {
  g_case = "abandon before delivery";
  HttpMailbox mailbox;
  mailbox.beginDispatch();
  mailbox.abandon();  // dispatch() timed out on the httpd side
  // The Controller thread processes the same (now-abandoned) request late
  // and must see "nobody is waiting" — the exact bug this class exists to
  // prevent: a stale Give(response_ready_) leaking into a future dispatch.
  CHECK(!mailbox.awaitingResponse());
}

void test_delivered_after_abandon_is_a_harmless_noop() {
  g_case = "delivered after abandon";
  HttpMailbox mailbox;
  mailbox.beginDispatch();
  mailbox.abandon();
  mailbox.delivered();  // Controller thread's unconditional cleanup call
  CHECK(!mailbox.awaitingResponse());
}

void test_a_fresh_dispatch_after_an_abandoned_one_awaits_correctly() {
  // The scenario from the operator's report, expressed at this class's
  // level: request A is abandoned (timed out), then request B begins a
  // genuinely new dispatch. B must be awaited normally — A's abandonment
  // must not "poison" the mailbox for future requests.
  g_case = "fresh dispatch after abandonment";
  HttpMailbox mailbox;

  mailbox.beginDispatch();  // A
  mailbox.abandon();        // A's dispatch() timed out
  CHECK(!mailbox.awaitingResponse());

  // The Controller thread now processes the late A, sees "not awaiting",
  // drops the result, and calls delivered() (its unconditional cleanup).
  mailbox.delivered();
  CHECK(!mailbox.awaitingResponse());

  // A NEW dispatch (B) begins, once the (real) slot_free_ semaphore has let
  // it claim the shared slot — see HttpMailbox.h's protocol diagram for why
  // that ordering is what prevents A and B from ever overlapping here.
  mailbox.beginDispatch();  // B
  CHECK(mailbox.awaitingResponse());  // B's answer, when computed, IS wanted

  mailbox.delivered();
  CHECK(!mailbox.awaitingResponse());
}

void test_delivered_without_a_prior_begin_dispatch_is_safe() {
  // Defensive: delivered() must never be able to leave awaiting_response_
  // in a surprising state even if called out of the documented order.
  g_case = "delivered with no prior beginDispatch";
  HttpMailbox mailbox;
  mailbox.delivered();
  CHECK(!mailbox.awaitingResponse());
}

void test_double_abandon_is_idempotent() {
  g_case = "double abandon";
  HttpMailbox mailbox;
  mailbox.beginDispatch();
  mailbox.abandon();
  mailbox.abandon();
  CHECK(!mailbox.awaitingResponse());
}

void test_repeated_dispatch_delivered_cycles_are_independent() {
  g_case = "repeated cycles";
  HttpMailbox mailbox;
  for (int i = 0; i < 5; ++i) {
    CHECK(!mailbox.awaitingResponse());
    mailbox.beginDispatch();
    CHECK(mailbox.awaitingResponse());
    mailbox.delivered();
    CHECK(!mailbox.awaitingResponse());
  }
}

}  // namespace

int main() {
  test_initial_state_awaits_nothing();
  test_begin_dispatch_marks_awaiting();
  test_happy_path_deliver_then_reset();
  test_abandon_clears_awaiting_before_the_response_is_computed();
  test_delivered_after_abandon_is_a_harmless_noop();
  test_a_fresh_dispatch_after_an_abandoned_one_awaits_correctly();
  test_delivered_without_a_prior_begin_dispatch_is_safe();
  test_double_abandon_is_idempotent();
  test_repeated_dispatch_delivered_cycles_are_independent();

  std::printf("test_http_mailbox: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
