// Offline tests link the real, pure LED policy. Frames are checked in physical
// pixel space against the independently frozen 12 o'clock clockwise mapping.
// Hardware brightness/color perception remains a later physical validation.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>

#include "../../src/status/LedStatusPolicy.h"
#include "../../src/power/DalyProtocol.h"

using namespace matdog::status;
using matdog::core::SystemHealth;
using matdog::power::kDalyTelemetryFreshnessMs;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_failures; \
  std::printf("FAIL [%s] line %d: %s\n", g_case, __LINE__, #cond); } } while (0)
#define CHECK_EQ(actual, expected) CHECK((actual) == (expected))
#define CHECK_STR(actual, expected) CHECK(std::strcmp((actual), (expected)) == 0)

namespace {
constexpr uint8_t kExpectedOrder[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0};

LedStatusInputs ready(float soc = 50.0f) {
  LedStatusInputs in;
  in.system_health = SystemHealth::READY;
  in.sample_valid = true;
  in.daly_comm_ok = true;
  in.telemetry_age_ms = 0;
  in.soc_percent = soc;
  return in;
}

void checkEffect(const LedEffect& e, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  CHECK_EQ(e.r, r);
  CHECK_EQ(e.g, g);
  CHECK_EQ(e.b, b);
  CHECK_EQ(e.brightness, brightness);
}

void checkUniform(const LedFrame& f, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  for (const auto& e : f.pixels) checkEffect(e, r, g, b, brightness);
}

void checkBar(const LedFrame& f, uint8_t count, int pulse = -1, uint8_t pulse_brightness = 0) {
  for (uint8_t logical = 0; logical < 12; ++logical) {
    const auto& e = f.pixels[kExpectedOrder[logical]];
    if (logical == pulse) checkEffect(e, 0, 255, 0, pulse_brightness);
    else if (logical < count) checkEffect(e, 0, 255, 0, 20);
    else checkEffect(e, 0, 0, 0, 0);
  }
}

void testPriorityAndFacts() {
  g_case = "all simultaneous priority combinations";
  // Every representable conflict among independent facts and the exclusive
  // health values is exercised, including fresh charging/fault/full facts.
  for (SystemHealth health : {SystemHealth::READY, SystemHealth::BOOTING,
       SystemHealth::DEGRADED, SystemHealth::FAULT, SystemHealth::MAINTENANCE}) {
    for (unsigned mask = 0; mask < 256; ++mask) {
      auto in = ready(100);
      in.system_health = health;
      in.firmware_update_in_progress = (mask & 1) != 0;
      in.calibration_in_progress = (mask & 2) != 0;
      in.wifi_connecting = (mask & 4) != 0;
      in.battery_charging = (mask & 8) != 0;
      in.battery_alarm = (mask & 16) != 0;
      in.charge_complete_verified = (mask & 32) != 0;
      in.battery_warning = (mask & 64) != 0;
      in.battery_critical = (mask & 128) != 0;
      const bool triggered[] = {health == SystemHealth::FAULT,
        in.firmware_update_in_progress, in.calibration_in_progress,
        in.battery_charging && in.battery_alarm, in.battery_critical,
        health == SystemHealth::DEGRADED, in.battery_warning,
        in.wifi_connecting, health == SystemHealth::BOOTING,
        in.charge_complete_verified && !in.battery_alarm, in.battery_charging, true};
      const LedPresentationState states[] = {LedPresentationState::FAULT,
        LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS,
        LedPresentationState::CALIBRATION_IN_PROGRESS, LedPresentationState::CHARGING_FAULT,
        LedPresentationState::BATTERY_CRITICAL, LedPresentationState::DEGRADED,
        LedPresentationState::BATTERY_WARNING, LedPresentationState::WIFI_CONNECTING,
        LedPresentationState::BOOTING, LedPresentationState::CHARGE_COMPLETE_VERIFIED,
        LedPresentationState::CHARGING, LedPresentationState::READY};
      unsigned winner = 0;
      while (!triggered[winner]) ++winner;
      CHECK_EQ(selectLedState(in), states[winner]);
      LedStatusPolicy policy;
      policy.update(in, 0, 60);
      CHECK_EQ(policy.state(), states[winner]);
      CHECK_EQ(policy.snapshot().presentation, states[winner]);
      CHECK_EQ(policy.snapshot().charging, in.battery_charging);
      CHECK_EQ(policy.snapshot().charging_fault, in.battery_charging && in.battery_alarm);
      CHECK_EQ(policy.snapshot().charge_complete_verified,
               in.charge_complete_verified && !in.battery_alarm);
      CHECK_EQ(policy.snapshot().battery_warning, in.battery_warning);
      CHECK_EQ(policy.snapshot().battery_critical, in.battery_critical);
    }
  }
}

void testReservedBatteryFacts() {
  g_case = "reserved facts default false without SOC or alarm inference";
  LedStatusInputs defaults;
  LedStatusSnapshot snapshot;
  CHECK(!defaults.battery_warning);
  CHECK(!defaults.battery_critical);
  CHECK(!snapshot.battery_warning);
  CHECK(!snapshot.battery_critical);
  for (float soc : {-100.0f, 0.0f, 1.0f, 8.33f, 20.0f, 50.0f, 99.99f, 100.0f,
       125.0f, std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::infinity()}) {
    for (unsigned mask = 0; mask < 8; ++mask) {
      auto in = ready(soc);
      in.battery_charging = (mask & 1) != 0;
      in.battery_alarm = (mask & 2) != 0;
      in.charge_complete_verified = (mask & 4) != 0;
      LedStatusPolicy policy;
      policy.update(in, 1500, 60);
      CHECK(!in.battery_warning);
      CHECK(!in.battery_critical);
      CHECK(!policy.snapshot().battery_warning);
      CHECK(!policy.snapshot().battery_critical);
      CHECK(policy.state() != LedPresentationState::BATTERY_WARNING);
      CHECK(policy.state() != LedPresentationState::BATTERY_CRITICAL);
    }
  }
  g_case = "reserved facts pass through independently of DALY and SOC validity";
  // A future battery-policy owner owns these facts' validity. Presentation
  // must neither infer them nor silently apply the DALY freshness gate.
  for (unsigned invalid = 0; invalid < 7; ++invalid) {
    auto in = ready();
    if (invalid == 1) in.sample_valid = false;
    if (invalid == 2) in.daly_comm_ok = false;
    if (invalid == 3) in.telemetry_age_ms = kDalyTelemetryFreshnessMs + 1;
    if (invalid == 4) in.soc_percent = std::numeric_limits<float>::quiet_NaN();
    if (invalid == 5) in.soc_percent = std::numeric_limits<float>::infinity();
    if (invalid == 6) in.telemetry_age_ms = UINT32_MAX;
    LedStatusPolicy policy;
    for (unsigned mask : {0u, 1u, 2u, 3u, 0u}) {
      in.battery_warning = (mask & 1) != 0;
      in.battery_critical = (mask & 2) != 0;
      const auto expected = in.battery_critical ? LedPresentationState::BATTERY_CRITICAL
          : in.battery_warning ? LedPresentationState::BATTERY_WARNING
                               : LedPresentationState::READY;
      policy.update(in, 1500, 60);
      CHECK_EQ(selectLedState(in), expected);
      CHECK_EQ(policy.state(), expected);
      CHECK_EQ(policy.snapshot().battery_warning, in.battery_warning);
      CHECK_EQ(policy.snapshot().battery_critical, in.battery_critical);
      CHECK(!policy.snapshot().charge_complete_verified);
    }
  }
}

void testReservedBatteryEffects() {
  g_case = "reserved battery states render uniform slow breathing";
  const uint8_t warning_envelope[] = {6, 13, 20, 13, 6};
  const uint8_t critical_envelope[] = {6, 18, 30, 18, 6};
  for (bool critical : {false, true}) {
    const auto state = critical ? LedPresentationState::BATTERY_CRITICAL
                                : LedPresentationState::BATTERY_WARNING;
    const uint8_t green = critical ? 0 : 140;
    const uint8_t peak = critical ? 30 : 20;
    auto in = ready(0);
    in.battery_warning = !critical;
    in.battery_critical = critical;
    LedStatusPolicy policy;
    for (unsigned i = 0; i < 5; ++i) {
      const uint8_t expected = critical ? critical_envelope[i] : warning_envelope[i];
      checkEffect(ledEffectFor(state, i * 750, 60), 255, green, 0, expected);
      checkUniform(policy.update(in, i * 750, 60), 255, green, 0, expected);
      CHECK_EQ(policy.state(), state);
    }
    uint8_t low = 255, high = 0;
    for (uint32_t t = 0; t <= 6000; t += 25) {
      const auto e = ledEffectFor(state, t, 60);
      CHECK(e.brightness >= 6 && e.brightness <= peak);
      checkUniform(policy.update(in, t, 60), 255, green, 0, e.brightness);
      CHECK_EQ(e.brightness, ledEffectFor(state, t + 3000, 60).brightness);
      if (e.brightness < low) low = e.brightness;
      if (e.brightness > high) high = e.brightness;
    }
    CHECK_EQ(low, 6);
    CHECK_EQ(high, peak);
  }
  g_case = "reserved battery brightness ceilings include degenerate ranges";
  for (unsigned max = 0; max <= 255; ++max) {
    for (bool critical : {false, true}) {
      const auto state = critical ? LedPresentationState::BATTERY_CRITICAL
                                  : LedPresentationState::BATTERY_WARNING;
      const uint8_t green = critical ? 0 : 140;
      const uint8_t peak = max / (critical ? 2 : 3);
      const uint8_t minimum = peak < 6 ? peak : 6;
      auto in = ready();
      in.battery_warning = !critical;
      in.battery_critical = critical;
      LedStatusPolicy policy;
      checkUniform(policy.update(in, 0, max), 255, green, 0, minimum);
      checkUniform(policy.update(in, 1500, max), 255, green, 0, peak);
      for (uint32_t t : {0u, 750u, 1000u, 1500u, 2250u, 3000u, UINT32_MAX}) {
        const auto e = ledEffectFor(state, t, max);
        CHECK(e.brightness >= minimum && e.brightness <= peak);
        CHECK(e.brightness <= max);
      }
    }
  }
  g_case = "battery envelopes remain distinct from legacy alarm displays";
  checkEffect(ledEffectFor(LedPresentationState::FAULT, 1000, 60), 255, 0, 0, 60);
  checkEffect(ledEffectFor(LedPresentationState::CHARGING_FAULT, 1000, 60), 255, 0, 0, 60);
  checkEffect(ledEffectFor(LedPresentationState::BATTERY_CRITICAL, 1000, 60), 255, 0, 0, 22);
  checkEffect(ledEffectFor(LedPresentationState::DEGRADED, 1500, 60), 255, 140, 0, 30);
  checkEffect(ledEffectFor(LedPresentationState::BATTERY_WARNING, 1500, 60), 255, 140, 0, 20);
}

void testLegacyEffectsAndSubtleBoot() {
  g_case = "unchanged legacy RGB/envelopes";
  for (uint32_t t : {0u, 500u, 1000u, 1500u, 2000u, 999999u, UINT32_MAX}) {
    checkEffect(ledEffectFor(LedPresentationState::FAULT, t, 60), 255, 0, 0, 60);
    checkEffect(ledEffectFor(LedPresentationState::DEGRADED, t, 60), 255, 140, 0, 30);
    checkEffect(ledEffectFor(LedPresentationState::READY, t, 60), 0, 255, 0, 20);
  }
  const uint8_t old_envelope[5] = {6, 33, 60, 33, 6};
  for (unsigned i = 0; i < 5; ++i) {
    checkEffect(ledEffectFor(LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS, i * 500, 60),
                0, 80, 255, old_envelope[i]);
    checkEffect(ledEffectFor(LedPresentationState::CALIBRATION_IN_PROGRESS, i * 500, 60),
                160, 0, 220, old_envelope[i]);
    checkEffect(ledEffectFor(LedPresentationState::WIFI_CONNECTING, i * 500, 60),
                0, 200, 200, old_envelope[i]);
    checkEffect(ledEffectFor(LedPresentationState::CHARGING_FAULT, i * 500, 60),
                255, 0, 0, old_envelope[i]);
  }
  g_case = "subtle white boot and future verified full";
  for (auto state : {LedPresentationState::BOOTING, LedPresentationState::CHARGE_COMPLETE_VERIFIED}) {
    uint8_t low = 255, high = 0;
    for (uint32_t t = 0; t <= 6000; t += 25) {
      const auto e = ledEffectFor(state, t, 60);
      CHECK_EQ(e.r, state == LedPresentationState::BOOTING ? 255 : 0);
      CHECK_EQ(e.g, 255);
      CHECK_EQ(e.b, state == LedPresentationState::BOOTING ? 255 : 0);
      CHECK(e.brightness >= 6 && e.brightness <= 20);
      if (e.brightness < low) low = e.brightness;
      if (e.brightness > high) high = e.brightness;
      CHECK_EQ(e.brightness, ledEffectFor(state, t + 3000, 60).brightness);
    }
    CHECK_EQ(low, 6);
    CHECK_EQ(high, 20);
    CHECK_EQ(ledEffectFor(state, 0, 60).brightness, 6);
    CHECK_EQ(ledEffectFor(state, 1500, 60).brightness, 20);
  }
  g_case = "all brightness ceilings avoid overflow";
  for (unsigned max = 0; max <= 255; ++max) {
    for (auto state : {LedPresentationState::BOOTING,
         LedPresentationState::CHARGE_COMPLETE_VERIFIED,
         LedPresentationState::FIRMWARE_UPDATE_IN_PROGRESS,
         LedPresentationState::CHARGING_FAULT}) {
      for (uint32_t t : {0u, 750u, 1000u, 1500u, UINT32_MAX})
        CHECK(ledEffectFor(state, t, max).brightness <= max);
    }
  }
}

void testQuantizationBoundariesAndPhysicalFrames() {
  g_case = "frozen map and exact rational boundaries";
  CHECK_EQ(kSocPixelCount, 12);
  CHECK_EQ(kSocStartPixel, 1);
  CHECK_EQ(kSocDirection, SocDirection::CLOCKWISE);
  for (unsigned i = 0; i < 12; ++i) CHECK_EQ(kSocPixelOrder[i], kExpectedOrder[i]);
  // 100*n/12 is not representable as float except n=3,6,9,12. Select
  // adjacent representable floats straddling each mathematical boundary.
  // An epsilon or float-intermediate multiplication can incorrectly round
  // the lower value upward; these assertions reject either regression.
  for (unsigned n = 1; n <= 12; ++n) {
    const double boundary = 100.0 * n / 12.0;
    float upper = static_cast<float>(boundary);
    if (static_cast<double>(upper) < boundary)
      upper = std::nextafter(upper, std::numeric_limits<float>::infinity());
    const float lower = std::nextafter(upper, -std::numeric_limits<float>::infinity());
    CHECK(static_cast<double>(lower) < boundary);
    CHECK(static_cast<double>(upper) >= boundary);
    CHECK_EQ(socCompletedSegments(lower), n - 1);
    CHECK_EQ(socCompletedSegments(upper), n);
    if (n % 3 == 0) CHECK_EQ(static_cast<double>(upper), boundary);
    LedStatusPolicy policy;
    checkBar(policy.update(ready(lower), 0, 60), n - 1);
    checkBar(policy.update(ready(upper), 0, 60), n);
  }
  CHECK_EQ(socCompletedSegments(-100), 0);
  CHECK_EQ(socCompletedSegments(-0.01f), 0);
  CHECK_EQ(socCompletedSegments(0), 0);
  CHECK_EQ(socCompletedSegments(8.33f), 0);
  CHECK_EQ(socCompletedSegments(99.99f), 11);
  CHECK_EQ(socCompletedSegments(100), 12);
  CHECK_EQ(socCompletedSegments(101), 12);
  CHECK_EQ(socCompletedSegments(std::numeric_limits<float>::quiet_NaN()), 0);
  CHECK_EQ(socCompletedSegments(std::numeric_limits<float>::infinity()), 0);
  CHECK_EQ(socCompletedSegments(-std::numeric_limits<float>::infinity()), 0);
  for (unsigned n = 0; n <= 12; ++n) {
    const float percent = n == 12 ? 100 : (n + 0.5f) * 100 / 12;
    LedStatusPolicy policy;
    auto in = ready(percent);
    const auto frame = policy.update(in, 123, 60);
    CHECK_EQ(policy.state(), LedPresentationState::READY);
    CHECK(policy.snapshot().soc_valid);
    CHECK_EQ(policy.snapshot().soc_segments, n);
    CHECK_EQ(policy.snapshot().soc_percent, percent);
    checkBar(frame, n);
    checkBar(socBarFrame(n, 60), n);
  }
  checkBar(socBarFrame(255, 60), 12);
  for (float percent : {-99.0f, 101.0f}) {
    LedStatusPolicy policy;
    checkBar(policy.update(ready(percent), 0, 60), percent < 0 ? 0 : 12);
    CHECK(policy.snapshot().soc_valid);
  }
}

void testCachedFreshnessAndIndeterminate() {
  g_case = "shared freshness boundary and cached validity";
  CHECK_EQ(kDalyTelemetryFreshnessMs, 5000u);
  for (unsigned invalid = 0; invalid < 6; ++invalid) {
    for (bool charging : {false, true}) {
      auto in = ready();
      in.battery_charging = charging;
      if (invalid == 0) in.sample_valid = false;
      if (invalid == 1) in.daly_comm_ok = false;
      if (invalid == 2) in.telemetry_age_ms = kDalyTelemetryFreshnessMs + 1;
      if (invalid == 3) in.soc_percent = std::numeric_limits<float>::quiet_NaN();
      if (invalid == 4) in.soc_percent = std::numeric_limits<float>::infinity();
      if (invalid == 5) in.soc_percent = -std::numeric_limits<float>::infinity();
      LedStatusPolicy policy;
      checkUniform(policy.update(in, 0, 60), 255, 140, 0, 6);
      CHECK(!policy.snapshot().soc_valid);
      CHECK_EQ(policy.snapshot().soc_segments, 0);
      CHECK_EQ(policy.state(), charging && invalid >= 3 ? LedPresentationState::CHARGING
                                                       : LedPresentationState::READY);
      checkUniform(policy.update(in, 1500, 60), 255, 140, 0, 20);
      in.system_health = SystemHealth::FAULT;
      checkUniform(policy.update(in, 1500, 60), 255, 0, 0, 60);
      // All telemetry-dependent facts are rejected if the sample/comm/age
      // gate fails; numeric SOC is independently validated for rendering.
      in.system_health = SystemHealth::READY;
      in.battery_charging = true;
      in.battery_alarm = true;
      in.charge_complete_verified = true;
      policy.update(in, 0, 60);
      CHECK_EQ(policy.state(), invalid < 3 ? LedPresentationState::READY
                                          : LedPresentationState::CHARGING_FAULT);
      CHECK(!policy.snapshot().charge_complete_verified);
      in.battery_alarm = false;
      policy.update(in, 0, 60);
      CHECK_EQ(policy.snapshot().charge_complete_verified, invalid >= 3);
    }
  }
  for (uint32_t age : {0u, kDalyTelemetryFreshnessMs - 1, kDalyTelemetryFreshnessMs}) {
    auto in = ready();
    in.telemetry_age_ms = age;
    LedStatusPolicy policy;
    checkBar(policy.update(in, 0, 60), 6);
    CHECK(policy.snapshot().soc_valid);
  }
  auto in = ready();
  in.telemetry_age_ms = UINT32_MAX;
  LedStatusPolicy policy;
  checkUniform(policy.update(in, 0, 60), 255, 140, 0, 6);
  CHECK(!policy.snapshot().soc_valid);
}

void testChargingAndTrueFullSeparation() {
  g_case = "next logical segment breathes; 100 percent remains charging";
  for (unsigned n = 0; n <= 12; ++n) {
    auto in = ready(n == 12 ? 100 : (n + 0.5f) * 100 / 12);
    in.battery_charging = true;
    LedStatusPolicy policy;
    const int pulsing = n == 12 ? 11 : n;
    checkBar(policy.update(in, 0, 60), n, pulsing, 6);
    CHECK_EQ(policy.state(), LedPresentationState::CHARGING);
    CHECK(!policy.snapshot().charge_complete_verified);
    checkBar(policy.update(in, 1500, 60), n, pulsing, 20);
    for (uint32_t t = 0; t <= 3000; t += 75) {
      const auto frame = policy.update(in, t, 60);
      CHECK(frame.pixels[kExpectedOrder[pulsing]].brightness >= 6);
      for (const auto& e : frame.pixels) CHECK(e.brightness <= 20);
    }
  }
  g_case = "no full producer from numeric SOC";
  for (float soc : {0.0f, 99.99f, 100.0f, 125.0f}) {
    for (bool charging : {false, true}) {
      auto in = ready(soc);
      in.battery_charging = charging;
      CHECK(!in.charge_complete_verified);
      CHECK_EQ(selectLedState(in), charging ? LedPresentationState::CHARGING
                                           : LedPresentationState::READY);
    }
  }
  g_case = "reserved verified-full renderer and fault";
  auto in = ready(100);
  in.charge_complete_verified = true;
  LedStatusPolicy policy;
  checkUniform(policy.update(in, 0, 60), 0, 255, 0, 6);
  CHECK_EQ(policy.state(), LedPresentationState::CHARGE_COMPLETE_VERIFIED);
  checkUniform(policy.update(in, 1500, 60), 0, 255, 0, 20);
  in.battery_charging = true;
  CHECK_EQ(selectLedState(in), LedPresentationState::CHARGE_COMPLETE_VERIFIED);
  in.battery_alarm = true;
  checkUniform(policy.update(in, 0, 60), 255, 0, 0, 6);
  CHECK_EQ(policy.state(), LedPresentationState::CHARGING_FAULT);
  checkUniform(policy.update(in, 1000, 60), 255, 0, 0, 60);
  in.battery_charging = false;
  CHECK_EQ(selectLedState(in), LedPresentationState::READY);
}

void testNamesAndInitialState() {
  g_case = "names and initial state";
  const char* names[] = {"READY", "CHARGING", "CHARGE_COMPLETE_VERIFIED", "BOOTING",
    "WIFI_CONNECTING", "BATTERY_WARNING", "DEGRADED", "BATTERY_CRITICAL",
    "CHARGING_FAULT", "CALIBRATION_IN_PROGRESS", "FIRMWARE_UPDATE_IN_PROGRESS", "FAULT"};
  CHECK_EQ(kLedPresentationStateCount, sizeof(names) / sizeof(names[0]));
  for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); ++i)
    CHECK_STR(toString(static_cast<LedPresentationState>(i)), names[i]);
  CHECK_STR(toString(static_cast<LedPresentationState>(200)), "UNKNOWN");
  checkEffect(ledEffectFor(static_cast<LedPresentationState>(200), 0, 60), 0, 0, 0, 0);
  LedStatusPolicy policy;
  CHECK_EQ(policy.state(), LedPresentationState::BOOTING);
  CHECK_EQ(policy.snapshot().presentation, LedPresentationState::BOOTING);
  CHECK(!policy.snapshot().soc_valid);
  CHECK(!policy.snapshot().charge_complete_verified);
  CHECK(!policy.snapshot().battery_warning);
  CHECK(!policy.snapshot().battery_critical);
}
}  // namespace

int main() {
  testPriorityAndFacts();
  testReservedBatteryFacts();
  testReservedBatteryEffects();
  testLegacyEffectsAndSubtleBoot();
  testQuantizationBoundariesAndPhysicalFrames();
  testCachedFreshnessAndIndeterminate();
  testChargingAndTrueFullSeparation();
  testNamesAndInitialState();
  std::printf("test_led_status_policy: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
