// Offline host tests for the LED presentation decision core
// (src/status/LedStatusPolicy.*): the deterministic priority order over
// FAULT / FIRMWARE_UPDATE_IN_PROGRESS / CALIBRATION_IN_PROGRESS / DEGRADED /
// WIFI_CONNECTING / BOOTING / READY, the RGB/brightness effect for each
// state, the triangle-wave breathing envelope, and toString().
//
// Links the REAL firmware translation unit, not a host-side copy — the same
// contract as test_wifi_policy.cpp and test_ota_policy.cpp. That is only
// possible because LedStatusPolicy.* has no <Arduino.h> and no LedRing
// dependency; the one setSolid() call per tick lives entirely in
// status/LedStatusManager.cpp, which is why this suite can drive the whole
// selection/effect logic from a synthetic clock and synthetic inputs.
//
// What this suite deliberately does NOT claim: nothing here proves a WS2812
// frame reaches a real ring, or that the colors read as intended to a human
// eye. That is a hardware/visual review, not a host test.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/status/LedStatusPolicy.h"

using namespace matdog::status;
using matdog::core::SystemHealth;

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

namespace {

LedStatusInputs allClear() {
  LedStatusInputs in;
  in.system_health = SystemHealth::READY;
  in.firmware_update_in_progress = false;
  in.calibration_in_progress = false;
  in.wifi_connecting = false;
  return in;
}

// ---------------------------------------------------------------------------
// selectLedState() — deterministic priority
// ---------------------------------------------------------------------------

void testDefaultIsReadyWhenNothingElseIsTrue() {
  g_case = "default_ready";
  CHECK_EQ((int)selectLedState(allClear()), (int)LedPresentationState::READY);
}

void testEachSoloTriggerSelectsItsOwnState() {
  g_case = "solo_triggers";
  {
    LedStatusInputs in = allClear();
    in.system_health = SystemHealth::FAULT;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::FAULT);
  }
  {
    LedStatusInputs in = allClear();
    in.firmware_update_in_progress = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS);
  }
  {
    LedStatusInputs in = allClear();
    in.calibration_in_progress = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::CALIBRATION_IN_PROGRESS);
  }
  {
    LedStatusInputs in = allClear();
    in.system_health = SystemHealth::DEGRADED;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::DEGRADED);
  }
  {
    LedStatusInputs in = allClear();
    in.wifi_connecting = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::WIFI_CONNECTING);
  }
  {
    LedStatusInputs in = allClear();
    in.system_health = SystemHealth::BOOTING;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::BOOTING);
  }
}

// SystemHealth::MAINTENANCE is a real enumerator that SystemState::update()
// never actually produces (see SystemState.cpp) — but the policy must still
// fail closed to READY rather than crash or fall through undefined, in case
// a future producer of it appears before this table is reviewed again.
void testUnclaimedSystemHealthFallsBackToReady() {
  g_case = "maintenance_falls_back_to_ready";
  LedStatusInputs in = allClear();
  in.system_health = SystemHealth::MAINTENANCE;
  CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::READY);
}

// Every higher-priority row must win over every lower-priority row it can
// coexist with, exactly as documented in the I2 architecture table.
void testPriorityOrderingOnConflict() {
  g_case = "priority_ordering";

  // FAULT beats every other simultaneous trigger.
  {
    LedStatusInputs in;
    in.system_health = SystemHealth::FAULT;
    in.firmware_update_in_progress = true;
    in.calibration_in_progress = true;
    in.wifi_connecting = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::FAULT);
  }

  // FIRMWARE_UPDATE_IN_PROGRESS beats calibration, degraded and wifi, but
  // never beats FAULT.
  {
    LedStatusInputs in = allClear();
    in.firmware_update_in_progress = true;
    in.calibration_in_progress = true;
    in.system_health = SystemHealth::DEGRADED;
    in.wifi_connecting = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS);
  }

  // CALIBRATION_IN_PROGRESS beats degraded and wifi.
  {
    LedStatusInputs in = allClear();
    in.calibration_in_progress = true;
    in.system_health = SystemHealth::DEGRADED;
    in.wifi_connecting = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::CALIBRATION_IN_PROGRESS);
  }

  // DEGRADED beats wifi-connecting and booting.
  {
    LedStatusInputs in = allClear();
    in.system_health = SystemHealth::DEGRADED;
    in.wifi_connecting = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::DEGRADED);
  }

  // WIFI_CONNECTING beats booting (booting cannot realistically coexist with
  // it on the real Controller — Wi-Fi never starts before boot completes —
  // but the priority table must still be total and deterministic).
  {
    LedStatusInputs in = allClear();
    in.system_health = SystemHealth::BOOTING;
    in.wifi_connecting = true;
    CHECK_EQ((int)selectLedState(in), (int)LedPresentationState::WIFI_CONNECTING);
  }
}

// ---------------------------------------------------------------------------
// ledEffectFor() — colors, solid vs. breathing, brightness bounds
// ---------------------------------------------------------------------------

void testSolidStatesIgnoreNowMs() {
  g_case = "solid_states_ignore_time";
  for (LedPresentationState s : {LedPresentationState::FAULT, LedPresentationState::DEGRADED,
                                 LedPresentationState::BOOTING, LedPresentationState::READY}) {
    const LedEffect a = ledEffectFor(s, 0, 60);
    const LedEffect b = ledEffectFor(s, 999999, 60);
    CHECK_EQ(a.r, b.r);
    CHECK_EQ(a.g, b.g);
    CHECK_EQ(a.b, b.b);
    CHECK_EQ(a.brightness, b.brightness);
  }
}

void testFaultIsSolidRedAtMaxBrightness() {
  g_case = "fault_effect";
  const LedEffect e = ledEffectFor(LedPresentationState::FAULT, 12345, 60);
  CHECK_EQ(e.r, 255);
  CHECK_EQ(e.g, 0);
  CHECK_EQ(e.b, 0);
  CHECK_EQ(e.brightness, 60);
}

void testReadyIsDimGreen() {
  g_case = "ready_effect";
  const LedEffect e = ledEffectFor(LedPresentationState::READY, 0, 60);
  CHECK_EQ(e.r, 0);
  CHECK_EQ(e.g, 255);
  CHECK_EQ(e.b, 0);
  CHECK(e.brightness > 0);
  CHECK(e.brightness < 60);  // dim, never the FAULT ceiling
}

void testDegradedIsSolidAmber() {
  g_case = "degraded_effect";
  const LedEffect e = ledEffectFor(LedPresentationState::DEGRADED, 0, 60);
  CHECK(e.r > 0);
  CHECK(e.g > 0);
  CHECK_EQ(e.b, 0);
  CHECK(e.r > e.g);  // amber, not yellow-green: red channel dominates
}

// Breathing states (update/calibration/wifi) must actually vary with time,
// reach the requested ceiling at the wave's peak, and never exceed it.
void testBreathingStatesVaryWithTimeAndRespectCeiling() {
  g_case = "breathing_effects";
  for (LedPresentationState s : {LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS,
                                 LedPresentationState::CALIBRATION_IN_PROGRESS,
                                 LedPresentationState::WIFI_CONNECTING}) {
    uint8_t min_seen = 255;
    uint8_t max_seen = 0;
    bool saw_variation = false;
    uint8_t previous = ledEffectFor(s, 0, 60).brightness;
    for (uint32_t t = 0; t <= 2000; t += 50) {
      const LedEffect e = ledEffectFor(s, t, 60);
      CHECK(e.brightness <= 60);
      if (e.brightness != previous) saw_variation = true;
      previous = e.brightness;
      if (e.brightness < min_seen) min_seen = e.brightness;
      if (e.brightness > max_seen) max_seen = e.brightness;
    }
    CHECK(saw_variation);
    CHECK_EQ(max_seen, 60);      // the wave reaches the requested ceiling
    CHECK(min_seen < max_seen);  // and is strictly dimmer somewhere in the cycle
  }
}

void testBreathingPeriodRepeats() {
  g_case = "breathing_period_repeats";
  // One full 2000 ms cycle must reproduce the same brightness.
  const LedEffect a = ledEffectFor(LedPresentationState::WIFI_CONNECTING, 500, 60);
  const LedEffect b = ledEffectFor(LedPresentationState::WIFI_CONNECTING, 2500, 60);
  CHECK_EQ(a.brightness, b.brightness);
}

void testDegenerateBrightnessCeilingNeverUnderflows() {
  g_case = "degenerate_ceiling";
  // max_brightness at or below the breathing floor must not wrap a uint8_t
  // subtraction; the effect must simply hold the ceiling constant.
  for (uint32_t t = 0; t <= 2000; t += 250) {
    const LedEffect e = ledEffectFor(LedPresentationState::CALIBRATION_IN_PROGRESS, t, 3);
    CHECK_EQ(e.brightness, 3);
  }
  const LedEffect zero = ledEffectFor(LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS, 0, 0);
  CHECK_EQ(zero.brightness, 0);
}

// ---------------------------------------------------------------------------
// toString()
// ---------------------------------------------------------------------------

void testToStringCoversEveryState() {
  g_case = "to_string";
  CHECK_STR(toString(LedPresentationState::READY), "READY");
  CHECK_STR(toString(LedPresentationState::BOOTING), "BOOTING");
  CHECK_STR(toString(LedPresentationState::WIFI_CONNECTING), "WIFI_CONNECTING");
  CHECK_STR(toString(LedPresentationState::DEGRADED), "DEGRADED");
  CHECK_STR(toString(LedPresentationState::CALIBRATION_IN_PROGRESS), "CALIBRATION_IN_PROGRESS");
  CHECK_STR(toString(LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS),
           "FIRMWARE_UPDATE_IN_PROGRESS");
  CHECK_STR(toString(LedPresentationState::FAULT), "FAULT");
}

void testToStringFailsClosedOnCorruptedValue() {
  g_case = "to_string_corrupted";
  const auto corrupted = static_cast<LedPresentationState>(200);
  CHECK_STR(toString(corrupted), "UNKNOWN");
}

// ---------------------------------------------------------------------------
// LedStatusPolicy — the stateful wrapper Controller/LedStatusManager use
// ---------------------------------------------------------------------------

void testPolicyStateTracksLastSelection() {
  g_case = "policy_state_tracking";
  LedStatusPolicy policy;
  CHECK_EQ((int)policy.state(), (int)LedPresentationState::BOOTING);  // constructed default

  LedStatusInputs in = allClear();
  in.system_health = SystemHealth::BOOTING;
  policy.update(in, 0, 60);
  CHECK_EQ((int)policy.state(), (int)LedPresentationState::BOOTING);

  in.system_health = SystemHealth::READY;
  const LedEffect e = policy.update(in, 0, 60);
  CHECK_EQ((int)policy.state(), (int)LedPresentationState::READY);
  CHECK_EQ(e.g, 255);  // the returned effect matches the newly selected state

  in.system_health = SystemHealth::FAULT;
  policy.update(in, 0, 60);
  CHECK_EQ((int)policy.state(), (int)LedPresentationState::FAULT);
}

}  // namespace

int main() {
  testDefaultIsReadyWhenNothingElseIsTrue();
  testEachSoloTriggerSelectsItsOwnState();
  testUnclaimedSystemHealthFallsBackToReady();
  testPriorityOrderingOnConflict();
  testSolidStatesIgnoreNowMs();
  testFaultIsSolidRedAtMaxBrightness();
  testReadyIsDimGreen();
  testDegradedIsSolidAmber();
  testBreathingStatesVaryWithTimeAndRespectCeiling();
  testBreathingPeriodRepeats();
  testDegenerateBrightnessCeilingNeverUnderflows();
  testToStringCoversEveryState();
  testToStringFailsClosedOnCorruptedValue();
  testPolicyStateTracksLastSelection();

  std::printf("test_led_status_policy: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
