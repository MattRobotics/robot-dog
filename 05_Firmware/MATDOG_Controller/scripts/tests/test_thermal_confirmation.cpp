// Offline tests for the LF V25 runtime PresentTemperature over-limit
// confirmation (src/calibration/ThermalConfirmation.*), ported from the
// oracle's port.rs (MATDOG_THERMAL_CONFIRMATION_READS = 3, 50 ms apart,
// classify_matdog_direct_temperature_samples: >= 2 of 3 over 70 C confirms).
//
// Driven through the real classification with a scripted bus port that
// records every direct read (which servo, in what order) and every wait.
// 2026-09-30 hardware: M42 reported one sample > 70 C during PARKING and read
// 32 C a second later - the single-sample abort this port removes.

#include <cstdio>
#include <cstring>
#include <vector>

#include "../../src/calibration/ThermalConfirmation.h"

using namespace matdog::calibration;

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
    const long a_ = (long)(actual);                                       \
    const long e_ = (long)(expected);                                     \
    if (a_ != e_) {                                                       \
      ++g_failures;                                                       \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case, \
                  __FILE__, __LINE__, #actual, a_, e_);                   \
    }                                                                      \
  } while (0)

namespace {

// A scripted bus: each direct read of any servo returns the next scripted
// value (or fails). Every call is recorded in order: 'W' + ms for a wait,
// 'R' + bus for a read.
struct ScriptedPort : ThermalReadPort {
  std::vector<int32_t> values;  // < -1000 = the read fails
  size_t next = 0;
  std::vector<std::pair<char, uint32_t>> calls;
  bool readPresentTemperatureDirect(uint8_t bus_id, int32_t* celsius) override {
    calls.push_back({'R', bus_id});
    if (next >= values.size()) return false;
    const int32_t v = values[next++];
    if (v < -1000) return false;
    *celsius = v;
    return true;
  }
  void delayMs(uint32_t ms) override { calls.push_back({'W', ms}); }
  int reads() const {
    int n = 0;
    for (const auto& c : calls) n += c.first == 'R';
    return n;
  }
};

constexpr int32_t kFail = -2000;

void checkOracleSequence(const ScriptedPort& port, uint8_t bus, int reads) {
  // wait 50, read this servo; wait 50, read this servo - nothing else.
  CHECK_EQ(port.calls.size(), 2u * reads);
  for (int i = 0; i < reads && 2 * i + 1 < (int)port.calls.size(); ++i) {
    CHECK(port.calls[2 * i].first == 'W');
    CHECK_EQ(port.calls[2 * i].second, kThermalConfirmationDelayMs);
    CHECK(port.calls[2 * i + 1].first == 'R');
    CHECK_EQ(port.calls[2 * i + 1].second, bus);  // the SAME servo, never another
  }
}

void test_constants_are_the_oracle() {
  g_case = "V25 port.rs constants";
  CHECK_EQ(kThermalLimitC, 70);                 // MATDOG_EXPECTED_TEMPERATURE_LIMIT_C
  CHECK_EQ(kThermalConfirmationReads, 3);       // MATDOG_THERMAL_CONFIRMATION_READS
  CHECK_EQ(kThermalConfirmationDelayMs, 50u);   // MATDOG_THERMAL_CONFIRMATION_DELAY
  CHECK_EQ(kThermalConfirmedOverLimit, 2);      // classify_...: over_limit >= 2
}

void test_normal_reads_nothing() {
  for (int32_t t : {-1, 0, 32, 69, 70}) {
    g_case = "<= 70 C (or unread): no confirmation read, value unchanged";
    ScriptedPort port;
    port.values = {99, 99};
    const ThermalConfirmation c = confirmPresentTemperature(&port, 42, t);
    CHECK(c.decision == ThermalDecision::NORMAL);
    CHECK_EQ(c.published_c, t);
    CHECK_EQ(c.sample_count, 1);
    CHECK(port.calls.empty());
  }
}

void test_majority_rule() {
  struct Case {
    const char* name;
    int32_t trigger, c1, c2;
    ThermalDecision expect;
    int32_t published;
  };
  const Case cases[] = {
      {"trigger > 70, normal, normal -> TRANSIENT, last normal published", 88, 32, 33,
       ThermalDecision::TRANSIENT, 33},
      {"trigger > 70, > 70, normal -> CONFIRMED", 75, 76, 40, ThermalDecision::CONFIRMED, 76},
      {"trigger > 70, normal, > 70 -> CONFIRMED", 75, 40, 72, ThermalDecision::CONFIRMED, 75},
      {"trigger > 70, > 70, > 70 -> CONFIRMED", 71, 74, 73, ThermalDecision::CONFIRMED, 74},
      {"boundary: 71 is over, 70 is not", 71, 70, 70, ThermalDecision::TRANSIENT, 70},
  };
  for (const Case& k : cases) {
    g_case = k.name;
    ScriptedPort port;
    port.values = {k.c1, k.c2, 99, 99};
    const ThermalConfirmation c = confirmPresentTemperature(&port, 23, k.trigger);
    CHECK(c.decision == k.expect);
    CHECK_EQ(c.published_c, k.published);
    CHECK_EQ(c.sample_count, 3);  // bounded: exactly two confirmation reads
    CHECK_EQ(c.samples[0], k.trigger);
    CHECK_EQ(c.samples[1], k.c1);
    CHECK_EQ(c.samples[2], k.c2);
    CHECK_EQ(port.reads(), 2);
    checkOracleSequence(port, 23, 2);
    if (k.expect == ThermalDecision::CONFIRMED) CHECK(c.published_c > kThermalLimitC);
    if (k.expect == ThermalDecision::TRANSIENT) CHECK(c.published_c <= kThermalLimitC);
  }
}

void test_confirmation_read_failures_fail_closed() {
  {
    g_case = "first confirmation read fails: abort, the trigger stays published";
    ScriptedPort port;
    port.values = {kFail, 32};
    const ThermalConfirmation c = confirmPresentTemperature(&port, 42, 90);
    CHECK(c.decision == ThermalDecision::CONFIRMATION_READ_FAILED);
    CHECK_EQ(c.published_c, 90);
    CHECK(c.published_c > kThermalLimitC);  // the monitors abort, exactly as before
    CHECK_EQ(port.reads(), 1);
    checkOracleSequence(port, 42, 1);
  }
  {
    g_case = "second confirmation read fails: abort";
    ScriptedPort port;
    port.values = {32, kFail};
    const ThermalConfirmation c = confirmPresentTemperature(&port, 42, 90);
    CHECK(c.decision == ThermalDecision::CONFIRMATION_READ_FAILED);
    CHECK(c.published_c > kThermalLimitC);
    CHECK_EQ(port.reads(), 2);
  }
  {
    g_case = "a negative (unread) direct value is a failed read";
    ScriptedPort port;
    port.values = {-1, 32};
    const ThermalConfirmation c = confirmPresentTemperature(&port, 42, 90);
    CHECK(c.decision == ThermalDecision::CONFIRMATION_READ_FAILED);
    CHECK(c.published_c > kThermalLimitC);
  }
  {
    g_case = "no port: cannot confirm, abort";
    const ThermalConfirmation c = confirmPresentTemperature(nullptr, 42, 90);
    CHECK(c.decision == ThermalDecision::CONFIRMATION_READ_FAILED);
    CHECK_EQ(c.published_c, 90);
  }
}

void test_todays_m42_observation() {
  g_case = "2026-09-30 M42: one sample > 70 C during PARKING, direct reads ~32 C -> transient only";
  ScriptedPort port;
  port.values = {32, 32};
  const ThermalConfirmation c = confirmPresentTemperature(&port, 42, 255);
  CHECK(c.decision == ThermalDecision::TRANSIENT);
  CHECK_EQ(c.published_c, 32);
  CHECK(c.published_c <= kThermalLimitC);  // no abort
  checkOracleSequence(port, 42, 2);
}

void test_fresh_direct_reads_each_time() {
  g_case = "every confirmation is its own fresh read (never the trigger value reused)";
  ScriptedPort port;
  port.values = {31, 30};
  const ThermalConfirmation c = confirmPresentTemperature(&port, 11, 80);
  CHECK_EQ(c.samples[1], 31);
  CHECK_EQ(c.samples[2], 30);
  CHECK_EQ(port.next, 2u);
  CHECK(c.decision == ThermalDecision::TRANSIENT);
}

void test_names() {
  g_case = "names";
  for (int d = 0; d <= (int)ThermalDecision::CONFIRMATION_READ_FAILED; ++d) {
    CHECK(std::strcmp(toString(static_cast<ThermalDecision>(d)), "UNKNOWN") != 0);
  }
}

}  // namespace

int main() {
  test_constants_are_the_oracle();
  test_normal_reads_nothing();
  test_majority_rule();
  test_confirmation_read_failures_fail_closed();
  test_todays_m42_observation();
  test_fresh_direct_reads_each_time();
  test_names();
  std::printf("test_thermal_confirmation: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
