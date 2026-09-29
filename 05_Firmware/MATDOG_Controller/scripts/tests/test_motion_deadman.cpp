// Offline host tests for the CR3 Priority 4 telemetry/deadman monitor
// (src/actuator/MotionDeadman.*). Synthetic telemetry only - no hardware.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/MotionDeadman.h"

using namespace matdog::actuator;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK_EQ(actual, expected)                                         \
  do {                                                                     \
    ++g_checks;                                                            \
    const long a_ = (long)(actual);                                       \
    const long e_ = (long)(expected);                                     \
    if (a_ != e_) {                                                       \
      ++g_failures;                                                       \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case, \
                  __FILE__, __LINE__, #actual, a_, e_);                   \
    }                                                                     \
  } while (0)

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                             \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));         \
    }                                                                          \
  } while (0)

namespace {

MotionDeadmanConfig config() {
  MotionDeadmanConfig c{};
  c.max_telemetry_age_ms = 3000;
  c.motion_timeout_ms = 12000;
  c.stall_window_ms = 2000;
  c.stall_progress_ticks = 4;
  c.arrival_tolerance_ticks = 8;
  return c;
}

TelemetrySample good(int32_t position, int32_t torque_enable, uint32_t at_ms) {
  TelemetrySample s{};
  s.read_ok = true;
  s.sampled_at_ms = at_ms;
  s.present_position = position;
  s.torque_enable = torque_enable;
  return s;
}

TelemetrySample failed(uint32_t at_ms) {
  TelemetrySample s{};
  s.read_ok = false;
  s.sampled_at_ms = at_ms;
  return s;
}

void testHealthyMotionContinuesThenArrives() {
  g_case = "healthy motion arrives";
  MotionDeadmanMonitor m;
  m.begin(config(), /*target_tick=*/2100, /*started_at_ms=*/1000);

  // Progressing toward the target, with sufficient movement each step to
  // avoid tripping the stall window.
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1100), 1100), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2040, 1, 1600), 1600), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2080, 1, 2100), 2100), (int)MotionDeadmanVerdict::CONTINUE);
  // Within tolerance (8 ticks) of 2100.
  CHECK_EQ((int)m.evaluate(good(2095, 1, 2600), 2600), (int)MotionDeadmanVerdict::ARRIVED);
}

void testExactArrivalAndBothToleranceEdges() {
  g_case = "arrival tolerance edges";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2092, 1, 1100), 1100), (int)MotionDeadmanVerdict::ARRIVED);

  MotionDeadmanMonitor m2;
  m2.begin(config(), 2100, 1000);
  CHECK_EQ((int)m2.evaluate(good(2108, 1, 1100), 1100), (int)MotionDeadmanVerdict::ARRIVED);

  // One tick beyond tolerance must not arrive (and must count as the first
  // progress sample, not yet stalled).
  MotionDeadmanMonitor m3;
  m3.begin(config(), 2100, 1000);
  CHECK_EQ((int)m3.evaluate(good(2109, 1, 1100), 1100), (int)MotionDeadmanVerdict::CONTINUE);
}

void testSingleDroppedPollDoesNotAbortInsideFreshWindow() {
  g_case = "single dropped poll tolerated";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1100), 1100), (int)MotionDeadmanVerdict::CONTINUE);
  // One failed poll shortly after a good one - well inside the 3s age
  // window - must not be treated as communication loss.
  CHECK_EQ((int)m.evaluate(failed(1300), 1300), (int)MotionDeadmanVerdict::CONTINUE);
  // Recovers on the next good sample.
  CHECK_EQ((int)m.evaluate(good(2010, 1, 1500), 1500), (int)MotionDeadmanVerdict::CONTINUE);
}

void testCommunicationLostOnceAgeWindowElapses() {
  g_case = "communication lost after age window";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1000), 1000), (int)MotionDeadmanVerdict::CONTINUE);
  // Repeated failures; verdict stays CONTINUE until 3000ms have elapsed
  // since the last good sample (at 1000ms).
  CHECK_EQ((int)m.evaluate(failed(2000), 2000), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(failed(3999), 3999), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(failed(4000), 4000), (int)MotionDeadmanVerdict::COMMUNICATION_LOST);
}

void testStaleTelemetryViaPollWithNoAttempts() {
  g_case = "stale telemetry via poll()";
  // A generous stall window isolates staleness: with the default config's
  // 2000ms stall window, a single recorded position (never updated again
  // because no further evaluate() runs in this test) would itself trip
  // STALLED at 3000ms - correct behavior, but not what this test isolates.
  MotionDeadmanConfig c = config();
  c.stall_window_ms = 60000;
  MotionDeadmanMonitor m;
  m.begin(c, 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1000), 1000), (int)MotionDeadmanVerdict::CONTINUE);
  // No poll attempted at all for a while - poll() must still notice.
  CHECK_EQ((int)m.poll(3999), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.poll(4000), (int)MotionDeadmanVerdict::STALE_TELEMETRY);
  // poll() must not mutate state: a genuinely fresh sample right after must
  // still be read on its own merits.
  CHECK_EQ((int)m.evaluate(good(2010, 1, 4010), 4010), (int)MotionDeadmanVerdict::CONTINUE);
}

void testTorqueUnexpectedlyOff() {
  g_case = "torque unexpectedly off";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 0, 1100), 1100),
          (int)MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF);
}

void testArrivalTakesPriorityOverTorqueOff() {
  // Reaching the target and torque dropping in the very same sample is not
  // itself alarming - ARRIVED must win.
  g_case = "arrival outranks torque-off at the target";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2100, 0, 1100), 1100), (int)MotionDeadmanVerdict::ARRIVED);
}

void testStalledWhenNoProgressWithinWindow() {
  g_case = "stalled";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1000), 1000), (int)MotionDeadmanVerdict::CONTINUE);
  // Position barely moves (below stall_progress_ticks=4) across the whole
  // 2000ms stall window.
  CHECK_EQ((int)m.evaluate(good(2001, 1, 2000), 2000), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2002, 1, 2999), 2999), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2002, 1, 3000), 3000), (int)MotionDeadmanVerdict::STALLED);
}

void testProgressReceivedResetsStallWindow() {
  g_case = "progress resets stall window";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1000), 1000), (int)MotionDeadmanVerdict::CONTINUE);
  // Real progress just before the window would have expired.
  CHECK_EQ((int)m.evaluate(good(2010, 1, 2900), 2900), (int)MotionDeadmanVerdict::CONTINUE);
  // A further 2000ms from the NEW progress mark (2900) without progress -
  // would have stalled at 3000 measured from the original mark, but the
  // reset window pushes it to 4900.
  CHECK_EQ((int)m.evaluate(good(2011, 1, 3100), 3100), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2011, 1, 4900), 4900), (int)MotionDeadmanVerdict::STALLED);
}

void testTimeoutOutranksAHealthySample() {
  g_case = "timeout outranks healthy sample";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  // Otherwise-perfect sample, but the 12s motion budget (started at 1000)
  // has already run out by 13000.
  CHECK_EQ((int)m.evaluate(good(2050, 1, 13000), 13000), (int)MotionDeadmanVerdict::TIMED_OUT);
}

void testTimeoutViaPollToo() {
  g_case = "timeout via poll()";
  // A generous age window isolates the timeout condition: poll() never
  // refreshes "last known good", so a long poll()-only stretch would
  // otherwise go stale (correctly) well before the motion budget expires.
  MotionDeadmanConfig c = config();
  c.max_telemetry_age_ms = 60000;
  c.stall_window_ms = 60000;
  MotionDeadmanMonitor m;
  m.begin(c, 2100, 1000);
  CHECK_EQ((int)m.poll(12999), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.poll(13000), (int)MotionDeadmanVerdict::TIMED_OUT);
}

void testIrregularSampleIntervalsDoNotFalselyStallOrStale() {
  // The explicit CR3 requirement: sparse but individually-timely samples
  // must not be penalized just because they are not evenly spaced.
  g_case = "irregular sample intervals tolerated";
  MotionDeadmanMonitor m;
  m.begin(config(), 2100, 1000);
  CHECK_EQ((int)m.evaluate(good(2000, 1, 1050), 1050), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2020, 1, 1090), 1090), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2040, 1, 3800), 3800), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2060, 1, 3820), 3820), (int)MotionDeadmanVerdict::CONTINUE);
}

// ---- travel-aware budget (hardware finding 2026-09-29) --------------------
// The real LF_UPPER MIN first approach: q0 raw 2100 -> Geometry V5 contact
// raw 1507 (593 ticks) at the bounded 40 ticks/s the backend writes. The
// fixed 12 s budget covers 480 ticks, so the joint was still ~113 ticks
// short of the stop when TIMED_OUT fired.
MotionDeadmanConfig travelConfig() {
  MotionDeadmanConfig c = config();
  c.stall_progress_ticks = 2;
  c.arrival_tolerance_ticks = 4;
  c.nominal_travel_ticks_per_s = 40;
  return c;
}

// Position of a joint moving at exactly 40 ticks/s from `from` toward `to`,
// stopped by a hard stop at `stop` (between from and to), t ms after start.
int32_t kinematic(int32_t from, int32_t to, int32_t stop, uint32_t t_ms) {
  const int32_t dir = to < from ? -1 : 1;
  const int32_t travelled = static_cast<int32_t>((t_ms * 40u) / 1000u);
  int32_t pos = from + dir * travelled;
  if (dir < 0 && pos < stop) pos = stop;
  if (dir > 0 && pos > stop) pos = stop;
  return pos;
}

// Drives a monitor at 20 ms ticks until a non-CONTINUE verdict or `limit_ms`.
MotionDeadmanVerdict driveKinematic(MotionDeadmanMonitor& m, int32_t from, int32_t to,
                                    int32_t stop, uint32_t start_ms, uint32_t limit_ms,
                                    uint32_t* at_ms) {
  for (uint32_t t = 20; t <= limit_ms; t += 20) {
    const MotionDeadmanVerdict v =
        m.evaluate(good(kinematic(from, to, stop, t), 1, start_ms + t), start_ms + t);
    if (v != MotionDeadmanVerdict::CONTINUE) {
      *at_ms = t;
      return v;
    }
  }
  *at_ms = limit_ms;
  return MotionDeadmanVerdict::CONTINUE;
}

void testFixedBudgetReproducesHardwareTimeout() {
  g_case = "fixed 12 s budget times out short of the LF_UPPER MIN stop (hardware repro)";
  MotionDeadmanConfig c = travelConfig();
  c.nominal_travel_ticks_per_s = 0;  // the 14881cd production config
  MotionDeadmanMonitor m;
  m.begin(c, /*target_tick=*/1507, /*started_at_ms=*/0);
  uint32_t at = 0;
  CHECK_EQ((int)driveKinematic(m, 2100, 1507, /*stop=*/1512, 0, 60000, &at),
           (int)MotionDeadmanVerdict::TIMED_OUT);
  CHECK_EQ((long)at, 12000L);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L);
  // Where the joint was when the budget ran out: 480 ticks from q0, ~113
  // ticks short of the stop - "moved ~20 deg or more" and no contact.
  CHECK_EQ(kinematic(2100, 1507, 1512, 12000), 1620);
}

void testTravelAwareBudgetReachesTheStopAndStalls() {
  g_case = "travel-aware budget lets the same approach reach the stop and STALL";
  MotionDeadmanMonitor m;
  m.begin(travelConfig(), /*target_tick=*/1507, /*started_at_ms=*/0);
  uint32_t at = 0;
  CHECK_EQ((int)driveKinematic(m, 2100, 1507, /*stop=*/1512, 0, 60000, &at),
           (int)MotionDeadmanVerdict::STALLED);
  // Fixed from the first in-range sample (2100 - 0 ticks at t=20 ms): 12 s
  // plus ceil(593 * 1000 / 40) = 14825 ms.
  CHECK_EQ((long)m.motionBudgetMs(), 12000L + 14825L);
  CHECK_EQ(m.lastProgressPosition() >= 1512 && m.lastProgressPosition() <= 1513, 1);
  // Reached the stop at ~14.7 s, stall confirmed one stall window later.
  CHECK_EQ(at > 14700 && at < 17000, 1);
}

void testTravelAwareBudgetStillTimesOut() {
  g_case = "travel-aware budget is still a hard backstop";
  MotionDeadmanMonitor m;
  m.begin(travelConfig(), /*target_tick=*/1507, /*started_at_ms=*/0);
  // Creeps 3 ticks every 100 ms forever (never stalls, never arrives): a
  // hunting joint. The extended budget must still end it, exactly on time.
  MotionDeadmanVerdict v = MotionDeadmanVerdict::CONTINUE;
  uint32_t t = 0;
  int32_t pos = 2103;  // first sample is 2100: travel 593 -> budget 26825 ms
  while (v == MotionDeadmanVerdict::CONTINUE && t < 100000) {
    t += 100;
    pos = (pos == 2100) ? 2103 : 2100;
    v = m.evaluate(good(pos, 1, t), t);
  }
  CHECK_EQ((int)v, (int)MotionDeadmanVerdict::TIMED_OUT);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L + 14825L);
  CHECK_EQ((long)t, 26900L);  // first 100 ms tick at/after 26825
  CHECK_EQ((int)m.poll(t), (int)MotionDeadmanVerdict::TIMED_OUT);
}

void testOutOfRangeFirstSampleDoesNotExtendBudget() {
  g_case = "a -1/garbage position never defines the travel";
  MotionDeadmanMonitor m;
  m.begin(travelConfig(), /*target_tick=*/1507, /*started_at_ms=*/0);
  CHECK_EQ((int)m.evaluate(good(-1, 1, 20), 20), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L);
  CHECK_EQ((int)m.evaluate(good(4096, 1, 40), 40), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L);
  // A failed read never fixes it either.
  CHECK_EQ((int)m.evaluate(failed(60), 60), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L);
  // The first in-range one does - and only once.
  CHECK_EQ((int)m.evaluate(good(2100, 1, 80), 80), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L + 14825L);
  CHECK_EQ((int)m.evaluate(good(2098, 1, 100), 100), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L + 14825L);
}

void testLateFirstSampleOnlyShortensBudget() {
  g_case = "a late first sample yields a shorter budget, never a longer one";
  MotionDeadmanMonitor m;
  m.begin(travelConfig(), /*target_tick=*/1507, /*started_at_ms=*/0);
  // First good sample 2 s in: the joint has already covered 80 ticks.
  CHECK_EQ((int)m.evaluate(failed(1000), 1000), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2020, 1, 2000), 2000), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L + 12825L);
}

void testZeroRateKeepsTheFixedBudgetBitForBit() {
  g_case = "nominal_travel_ticks_per_s == 0 keeps the fixed budget (DIRECTION_VERIFY)";
  MotionDeadmanMonitor m;
  m.begin(config(), /*target_tick=*/2116, /*started_at_ms=*/0);
  CHECK_EQ((int)m.evaluate(good(2100, 1, 20), 20), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((long)m.motionBudgetMs(), 12000L);
  CHECK_EQ((int)m.evaluate(good(2104, 1, 11999), 11999), (int)MotionDeadmanVerdict::CONTINUE);
  CHECK_EQ((int)m.evaluate(good(2108, 1, 12000), 12000), (int)MotionDeadmanVerdict::TIMED_OUT);
  MotionDeadmanMonitor fresh;
  fresh.begin(config(), 2116, 0);
  CHECK_EQ((int)fresh.poll(12000), (int)MotionDeadmanVerdict::TIMED_OUT);
}

void testToStringCoversEveryValue() {
  g_case = "to_string";
  CHECK_STR(toString(MotionDeadmanVerdict::CONTINUE), "CONTINUE");
  CHECK_STR(toString(MotionDeadmanVerdict::ARRIVED), "ARRIVED");
  CHECK_STR(toString(MotionDeadmanVerdict::STALE_TELEMETRY), "STALE_TELEMETRY");
  CHECK_STR(toString(MotionDeadmanVerdict::COMMUNICATION_LOST), "COMMUNICATION_LOST");
  CHECK_STR(toString(MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF), "TORQUE_UNEXPECTEDLY_OFF");
  CHECK_STR(toString(MotionDeadmanVerdict::STALLED), "STALLED");
  CHECK_STR(toString(MotionDeadmanVerdict::TIMED_OUT), "TIMED_OUT");
  CHECK_STR(toString(static_cast<MotionDeadmanVerdict>(200)), "UNKNOWN");
}

}  // namespace

int main() {
  testHealthyMotionContinuesThenArrives();
  testExactArrivalAndBothToleranceEdges();
  testSingleDroppedPollDoesNotAbortInsideFreshWindow();
  testCommunicationLostOnceAgeWindowElapses();
  testStaleTelemetryViaPollWithNoAttempts();
  testTorqueUnexpectedlyOff();
  testArrivalTakesPriorityOverTorqueOff();
  testStalledWhenNoProgressWithinWindow();
  testProgressReceivedResetsStallWindow();
  testTimeoutOutranksAHealthySample();
  testTimeoutViaPollToo();
  testIrregularSampleIntervalsDoNotFalselyStallOrStale();
  testToStringCoversEveryValue();
  testFixedBudgetReproducesHardwareTimeout();
  testTravelAwareBudgetReachesTheStopAndStalls();
  testTravelAwareBudgetStillTimesOut();
  testOutOfRangeFirstSampleDoesNotExtendBudget();
  testLateFirstSampleOnlyShortensBudget();
  testZeroRateKeepsTheFixedBudgetBitForBit();

  std::printf("test_motion_deadman: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
