// Offline host tests for the Wi-Fi runtime policy (src/network/WifiPolicy.*):
// the credential gate, the two-phase radio start, the connect deadline, the
// doubling backoff ladder and its ceiling, link-loss handling, operator
// enable/disable, action-failure fail-closed behaviour, the counters, the
// millis() wraparound, and the IPv4 formatter.
//
// Links the REAL firmware translation unit, not a host-side copy — the same
// contract as test_daly_protocol.cpp and test_servo_population.cpp. That is
// only possible because WifiPolicy.* has no <Arduino.h> and no <WiFi.h>
// dependency; the radio calls live entirely in network/WifiManager.cpp,
// which is why this suite can drive the whole lifecycle from a synthetic
// clock and a synthetic link-up flag.
//
// What this suite deliberately does NOT claim: nothing here proves the
// radio associates with a real access point. That is a hardware test.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/network/WifiPolicy.h"

using namespace matdog::network;

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

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

// Mirrors the values in network/WifiManager.h. Written out literally rather
// than included, so a change to the shipped tuning has to be made
// deliberately in both places instead of silently invalidating the
// expectations below.
static WifiPolicyConfig testConfig() {
  return WifiPolicyConfig{
      /*radio_settle_ms   =*/100,
      /*connect_timeout_ms=*/15000,
      /*backoff_initial_ms=*/2000,
      /*backoff_max_ms    =*/60000,
  };
}

// Drives the policy from INACTIVE to CONNECTING with the radio up, which is
// the precondition for most cases below. Returns the clock it stopped at.
static uint32_t driveToConnecting(WifiPolicy& p, uint32_t t0) {
  uint32_t t = t0;
  CHECK_EQ((int)p.update(t, false), (int)WifiAction::NONE);        // INACTIVE -> IDLE
  CHECK_EQ((int)p.state(), (int)WifiState::IDLE);
  t += 1;
  CHECK_EQ((int)p.update(t, false), (int)WifiAction::START_RADIO); // IDLE -> RADIO_STARTING
  CHECK_EQ((int)p.state(), (int)WifiState::RADIO_STARTING);
  t += 100;
  CHECK_EQ((int)p.update(t, false), (int)WifiAction::START_CONNECT);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTING);
  return t;
}

// ---------------------------------------------------------------------------
// Credential gate — the fail-closed case
// ---------------------------------------------------------------------------

static void test_no_credentials_never_starts_the_radio() {
  g_case = "no_credentials_never_starts_the_radio";
  WifiPolicy p;
  p.begin(testConfig(), /*credentials_present=*/false, 1000);

  CHECK_EQ((int)p.state(), (int)WifiState::INACTIVE);
  CHECK_EQ((int)p.fault(), (int)WifiFault::NO_CREDENTIALS);
  CHECK(!p.enabled());
  CHECK(!p.credentialsPresent());

  // No amount of ticking may produce a radio action.
  for (uint32_t i = 0; i < 500; ++i) {
    const WifiAction a = p.update(1000 + i * 37000, false);
    CHECK_EQ((int)a, (int)WifiAction::NONE);
  }
  CHECK_EQ((int)p.state(), (int)WifiState::INACTIVE);
  CHECK(!p.radioStarted());
  CHECK_EQ(p.counters().radio_starts, 0u);
  CHECK_EQ(p.counters().connect_attempts, 0u);
}

static void test_enable_without_credentials_is_refused() {
  g_case = "enable_without_credentials_is_refused";
  WifiPolicy p;
  p.begin(testConfig(), false, 0);

  p.setEnabled(true, 10);
  CHECK(!p.enabled());                                     // refusal, not a silent accept
  CHECK_EQ((int)p.fault(), (int)WifiFault::NO_CREDENTIALS);  // and it is reported
  CHECK_EQ((int)p.update(20, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::INACTIVE);
}

// ---------------------------------------------------------------------------
// Two-phase radio start
// ---------------------------------------------------------------------------

static void test_radio_start_is_two_phase() {
  g_case = "radio_start_is_two_phase";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  CHECK(p.enabled());
  CHECK_EQ((int)p.fault(), (int)WifiFault::NONE);

  CHECK_EQ((int)p.update(0, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::IDLE);

  CHECK_EQ((int)p.update(1, false), (int)WifiAction::START_RADIO);
  CHECK_EQ((int)p.state(), (int)WifiState::RADIO_STARTING);
  CHECK(p.radioStarted());
  CHECK_EQ(p.counters().radio_starts, 1u);

  // The settle window is what keeps WiFi.begin()'s 1000 ms blocking wait
  // from ever being reached: no connect may be issued before it elapses.
  CHECK_EQ((int)p.update(50, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.update(100, false), (int)WifiAction::NONE);   // 99 ms elapsed
  CHECK_EQ((int)p.state(), (int)WifiState::RADIO_STARTING);
  CHECK_EQ(p.counters().connect_attempts, 0u);

  CHECK_EQ((int)p.update(101, false), (int)WifiAction::START_CONNECT);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTING);
  CHECK_EQ(p.counters().connect_attempts, 1u);

  // The radio is started exactly once for the whole lifecycle.
  CHECK_EQ(p.counters().radio_starts, 1u);
}

static void test_one_transition_per_tick() {
  g_case = "one_transition_per_tick";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  // Even with the clock jumping far ahead, a single update() may not walk
  // the machine through several radio operations in one Controller pass.
  CHECK_EQ((int)p.update(1000000, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::IDLE);
  CHECK_EQ((int)p.update(2000000, false), (int)WifiAction::START_RADIO);
  CHECK_EQ((int)p.state(), (int)WifiState::RADIO_STARTING);
}

// ---------------------------------------------------------------------------
// Connect, timeout, backoff ladder
// ---------------------------------------------------------------------------

static void test_connect_success_resets_state() {
  g_case = "connect_success_resets_state";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);

  t += 3000;
  CHECK_EQ((int)p.update(t, true), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);
  CHECK_EQ((int)p.fault(), (int)WifiFault::NONE);
  CHECK_EQ(p.counters().connects, 1u);
  CHECK_EQ(p.backoffMs(), 2000u);
  CHECK_EQ(p.stateSinceMs(), t);

  // A stable link produces no further actions at all.
  for (uint32_t i = 0; i < 200; ++i) {
    CHECK_EQ((int)p.update(t + i * 100, true), (int)WifiAction::NONE);
  }
  CHECK_EQ(p.counters().connect_attempts, 1u);
}

static void test_connect_timeout_enters_backoff() {
  g_case = "connect_timeout_enters_backoff";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  const uint32_t t_connect = driveToConnecting(p, 0);

  // One millisecond short of the deadline: still trying.
  CHECK_EQ((int)p.update(t_connect + 14999, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTING);

  CHECK_EQ((int)p.update(t_connect + 15000, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);
  CHECK_EQ((int)p.fault(), (int)WifiFault::CONNECT_TIMEOUT);
  CHECK_EQ(p.counters().connect_timeouts, 1u);
  CHECK_EQ(p.counters().connects, 0u);
}

static void test_backoff_ladder_doubles_and_is_capped() {
  g_case = "backoff_ladder_doubles_and_is_capped";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);

  const uint32_t expected[] = {2000, 4000, 8000, 16000, 32000, 60000, 60000, 60000};
  for (uint32_t round = 0; round < 8; ++round) {
    const uint32_t wait_for_this_round = p.backoffMs();
    CHECK_EQ(wait_for_this_round, expected[round]);

    // Fail the attempt in flight.
    t += 15000;
    CHECK_EQ((int)p.update(t, false), (int)WifiAction::NONE);
    CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);

    // Nothing happens until the wait for THIS round has elapsed.
    CHECK_EQ((int)p.update(t + wait_for_this_round - 1, false), (int)WifiAction::NONE);
    CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);

    t += wait_for_this_round;
    CHECK_EQ((int)p.update(t, false), (int)WifiAction::START_CONNECT);
    CHECK_EQ((int)p.state(), (int)WifiState::CONNECTING);
    CHECK_EQ(p.counters().connect_attempts, round + 2u);
  }

  // The ceiling holds, and the radio was never restarted to get there.
  CHECK_EQ(p.backoffMs(), 60000u);
  CHECK_EQ(p.counters().radio_starts, 1u);
  CHECK_EQ(p.counters().connect_timeouts, 8u);
}

static void test_success_after_backoff_resets_the_ladder() {
  g_case = "success_after_backoff_resets_the_ladder";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);

  // Each completed round consumes the current wait and doubles the next
  // one: 2000 -> 4000 -> 8000 -> 16000.
  for (uint32_t round = 0; round < 3; ++round) {
    t += 15000;
    p.update(t, false);
    t += p.backoffMs();
    p.update(t, false);
  }
  CHECK_EQ(p.backoffMs(), 16000u);

  t += 500;
  CHECK_EQ((int)p.update(t, true), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);
  CHECK_EQ(p.backoffMs(), 2000u);
  CHECK_EQ((int)p.fault(), (int)WifiFault::NONE);
}

static void test_link_recovered_during_backoff_is_accepted() {
  g_case = "link_recovered_during_backoff_is_accepted";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);

  t += 15000;
  p.update(t, false);
  CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);

  // The driver got there between ticks. The policy must not tear down a
  // working link just to follow its own plan.
  t += 10;
  CHECK_EQ((int)p.update(t, true), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);
  CHECK_EQ(p.counters().connects, 1u);
}

// ---------------------------------------------------------------------------
// Link loss / reconnect
// ---------------------------------------------------------------------------

static void test_link_loss_restarts_ladder_from_the_bottom() {
  g_case = "link_loss_restarts_ladder_from_the_bottom";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);

  // Climb the ladder, then connect: backoff is reset by the success.
  t += 15000; p.update(t, false);
  t += p.backoffMs(); p.update(t, false);
  t += 15000; p.update(t, false);
  t += p.backoffMs(); p.update(t, false);
  t += 10; p.update(t, true);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);

  t += 60000;
  CHECK_EQ((int)p.update(t, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);
  CHECK_EQ((int)p.fault(), (int)WifiFault::LINK_LOST);
  CHECK_EQ(p.counters().link_losses, 1u);
  // A link that was up and dropped is a fresh event, not accumulated
  // retry pressure from an earlier failure.
  CHECK_EQ(p.backoffMs(), 2000u);

  t += 2000;
  CHECK_EQ((int)p.update(t, false), (int)WifiAction::START_CONNECT);
  t += 10;
  CHECK_EQ((int)p.update(t, true), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);
  CHECK_EQ(p.counters().connects, 2u);
}

static void test_reconnect_count_excludes_the_first_association() {
  g_case = "reconnect_count_excludes_the_first_association";
  WifiStatus s{};
  CHECK_EQ(s.reconnects(), 0u);
  s.counters.connects = 1;
  CHECK_EQ(s.reconnects(), 0u);
  s.counters.connects = 4;
  CHECK_EQ(s.reconnects(), 3u);
}

// ---------------------------------------------------------------------------
// Operator enable/disable
// ---------------------------------------------------------------------------

static void test_disable_stops_a_started_radio_exactly_once() {
  g_case = "disable_stops_a_started_radio_exactly_once";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);
  t += 100;
  p.update(t, true);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);

  p.setEnabled(false, ++t);
  CHECK_EQ((int)p.update(++t, true), (int)WifiAction::STOP_RADIO);
  CHECK_EQ((int)p.state(), (int)WifiState::INACTIVE);
  CHECK(!p.radioStarted());

  // Idempotent: no repeated teardown calls into the driver, and link_up
  // still being reported by a radio that is winding down changes nothing.
  for (uint32_t i = 0; i < 100; ++i) {
    CHECK_EQ((int)p.update(++t, true), (int)WifiAction::NONE);
  }
  CHECK_EQ((int)p.state(), (int)WifiState::INACTIVE);
}

static void test_disable_before_radio_start_issues_no_stop() {
  g_case = "disable_before_radio_start_issues_no_stop";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  CHECK_EQ((int)p.update(0, false), (int)WifiAction::NONE);  // -> IDLE, radio untouched
  p.setEnabled(false, 1);
  // Nothing was brought up, so nothing may be torn down.
  CHECK_EQ((int)p.update(2, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::INACTIVE);
}

static void test_reenable_restarts_the_radio() {
  g_case = "reenable_restarts_the_radio";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);
  p.setEnabled(false, ++t);
  CHECK_EQ((int)p.update(++t, false), (int)WifiAction::STOP_RADIO);

  p.setEnabled(true, ++t);
  CHECK_EQ((int)p.update(++t, false), (int)WifiAction::NONE);        // INACTIVE -> IDLE
  CHECK_EQ((int)p.update(++t, false), (int)WifiAction::START_RADIO); // radio comes back up
  CHECK_EQ(p.counters().radio_starts, 2u);
  CHECK_EQ((int)p.fault(), (int)WifiFault::NONE);
}

// ---------------------------------------------------------------------------
// Action failure — fail closed
// ---------------------------------------------------------------------------

static void test_radio_start_failure_goes_back_through_idle() {
  g_case = "radio_start_failure_goes_back_through_idle";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = 0;
  p.update(t, false);
  CHECK_EQ((int)p.update(++t, false), (int)WifiAction::START_RADIO);

  // The radio owner reports the driver refused to start.
  p.reportActionFailed(WifiFault::RADIO_START_FAILED, t);
  CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);
  CHECK_EQ((int)p.fault(), (int)WifiFault::RADIO_START_FAILED);
  CHECK(!p.radioStarted());

  // The retry must be a radio start, never an association against a driver
  // that is not running.
  t += 2000;
  CHECK_EQ((int)p.update(t, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::IDLE);
  CHECK_EQ((int)p.update(++t, false), (int)WifiAction::START_RADIO);
  CHECK_EQ(p.counters().radio_starts, 2u);
  CHECK_EQ(p.counters().connect_attempts, 0u);
}

static void test_connect_call_rejected_enters_backoff() {
  g_case = "connect_call_rejected_enters_backoff";
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  const uint32_t t = driveToConnecting(p, 0);

  p.reportActionFailed(WifiFault::CONNECT_CALL_REJECTED, t);
  CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);
  CHECK_EQ((int)p.fault(), (int)WifiFault::CONNECT_CALL_REJECTED);
  CHECK(p.radioStarted());  // the driver is still up; only the call failed

  CHECK_EQ((int)p.update(t + 2000, false), (int)WifiAction::START_CONNECT);
}

static void test_no_connect_is_ever_issued_while_connected() {
  g_case = "no_connect_is_ever_issued_while_connected";
  // Matters because STAClass::connect() blocks for up to 1000 ms tearing
  // down an existing link before it reconnects.
  WifiPolicy p;
  p.begin(testConfig(), true, 0);
  uint32_t t = driveToConnecting(p, 0);
  t += 10;
  p.update(t, true);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTED);

  for (uint32_t i = 0; i < 200000; i += 4999) {
    const WifiAction a = p.update(t + i, true);
    CHECK(a != WifiAction::START_CONNECT);
    CHECK(a != WifiAction::START_RADIO);
  }
}

// ---------------------------------------------------------------------------
// Clock behaviour
// ---------------------------------------------------------------------------

static void test_millis_wraparound_does_not_stall_the_machine() {
  g_case = "millis_wraparound_does_not_stall_the_machine";
  // millis() wraps every ~49.7 days. An elapsed-time comparison on unsigned
  // arithmetic stays correct across the wrap; an absolute-deadline one does
  // not. Enter CONNECTING just before the wrap and confirm the deadline
  // still fires.
  const uint32_t near_wrap = 0xFFFFFF00u;
  WifiPolicy p;
  p.begin(testConfig(), true, near_wrap - 200);
  uint32_t t = driveToConnecting(p, near_wrap - 200);

  // 14999 ms later, having wrapped through zero: still connecting.
  CHECK_EQ((int)p.update(t + 14999u, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::CONNECTING);
  // 15000 ms later: the deadline fires exactly as it would mid-range.
  CHECK_EQ((int)p.update(t + 15000u, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.state(), (int)WifiState::BACKOFF);

  // And the backoff wait survives the wrap too.
  CHECK_EQ((int)p.update(t + 15000u + 1999u, false), (int)WifiAction::NONE);
  CHECK_EQ((int)p.update(t + 15000u + 2000u, false), (int)WifiAction::START_CONNECT);
}

// ---------------------------------------------------------------------------
// Presentation helpers
// ---------------------------------------------------------------------------

static void test_format_ipv4() {
  g_case = "format_ipv4";
  char buf[16];

  // Octet 0 in the low byte — the layout Arduino's IPAddress produces.
  formatIpv4(0x2A01A8C0u, buf, sizeof(buf));  // 192.168.1.42
  CHECK_STR(buf, "192.168.1.42");

  formatIpv4(0u, buf, sizeof(buf));
  CHECK_STR(buf, "0.0.0.0");

  formatIpv4(0xFFFFFFFFu, buf, sizeof(buf));
  CHECK_STR(buf, "255.255.255.255");

  formatIpv4(0x01010101u, buf, sizeof(buf));
  CHECK_STR(buf, "1.1.1.1");

  // Never writes past the buffer, always terminates.
  char small[8];
  std::memset(small, 'X', sizeof(small));
  formatIpv4(0xFFFFFFFFu, small, sizeof(small));
  CHECK_EQ(small[7], '\0');
  CHECK(std::strlen(small) < sizeof(small));

  // Degenerate inputs are survivable.
  formatIpv4(0u, nullptr, 0);
  char one[1] = {'X'};
  formatIpv4(0x01020304u, one, sizeof(one));
  CHECK_EQ(one[0], '\0');
}

static void test_tostring_is_total() {
  g_case = "tostring_is_total";
  const WifiState states[] = {WifiState::INACTIVE, WifiState::IDLE,
                              WifiState::RADIO_STARTING, WifiState::CONNECTING,
                              WifiState::CONNECTED, WifiState::BACKOFF};
  for (WifiState s : states) {
    CHECK(std::strcmp(toString(s), "UNKNOWN") != 0);
  }
  CHECK_STR(toString(WifiState::RADIO_STARTING), "RADIO_STARTING");

  const WifiFault faults[] = {WifiFault::NONE, WifiFault::NO_CREDENTIALS,
                              WifiFault::CONNECT_TIMEOUT, WifiFault::LINK_LOST,
                              WifiFault::RADIO_START_FAILED,
                              WifiFault::CONNECT_CALL_REJECTED};
  for (WifiFault f : faults) {
    CHECK(std::strcmp(toString(f), "UNKNOWN") != 0);
  }
  CHECK_STR(toString(WifiFault::NO_CREDENTIALS), "NO_CREDENTIALS");
}

static void test_status_snapshot_carries_no_secret() {
  g_case = "status_snapshot_carries_no_secret";
  // A structural guard to pair with the static audit: the snapshot the
  // command surface and the future Web UI read is small, copyable, and has
  // an SSID field but no passphrase field. If someone adds one, this
  // size/round-trip expectation is not what fails — the audit is — but the
  // copyability the transport layers rely on is pinned here.
  WifiStatus a{};
  a.state = WifiState::CONNECTED;
  a.connected = true;
  a.ipv4 = 0x2A01A8C0u;
  a.counters.connects = 2;
  std::memcpy(a.ssid, "matdog-net", 11);

  WifiStatus b = a;  // plain copy, no ownership, no heap
  CHECK_EQ((int)b.state, (int)WifiState::CONNECTED);
  CHECK_EQ(b.ipv4, 0x2A01A8C0u);
  CHECK_EQ(b.reconnects(), 1u);
  CHECK_STR(b.ssid, "matdog-net");
}

int main() {
  std::printf("MATDOG W1 Wi-Fi runtime policy offline tests\n");

  test_no_credentials_never_starts_the_radio();
  test_enable_without_credentials_is_refused();
  test_radio_start_is_two_phase();
  test_one_transition_per_tick();
  test_connect_success_resets_state();
  test_connect_timeout_enters_backoff();
  test_backoff_ladder_doubles_and_is_capped();
  test_success_after_backoff_resets_the_ladder();
  test_link_recovered_during_backoff_is_accepted();
  test_link_loss_restarts_ladder_from_the_bottom();
  test_reconnect_count_excludes_the_first_association();
  test_disable_stops_a_started_radio_exactly_once();
  test_disable_before_radio_start_issues_no_stop();
  test_reenable_restarts_the_radio();
  test_radio_start_failure_goes_back_through_idle();
  test_connect_call_rejected_enters_backoff();
  test_no_connect_is_ever_issued_while_connected();
  test_millis_wraparound_does_not_stall_the_machine();
  test_format_ipv4();
  test_tostring_is_total();
  test_status_snapshot_carries_no_secret();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("WIFI_POLICY_TESTS = FAIL\n");
    return 1;
  }
  std::printf("WIFI_POLICY_TESTS = PASS\n");
  return 0;
}
