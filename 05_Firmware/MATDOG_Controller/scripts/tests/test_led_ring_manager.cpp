#include <cstdio>
#include <cstring>
#include <limits>

#include "../../src/status/LedStatusManager.h"
#include "led_replay_20261110_fixture.h"

using namespace matdog;
using namespace matdog::status;

static unsigned checks = 0;
static unsigned failures = 0;
#define CHECK(expression) do { ++checks; if (!(expression)) { ++failures; \
  std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); } } while (0)

static const led_test::TransportFrame& lastFrame() { return led_test::frames.back(); }
static void checkSolid(uint32_t rgb) {
  CHECK(!led_test::frames.empty());
  if (led_test::frames.empty()) return;
  for (uint32_t pixel : lastFrame().pixels) CHECK(pixel == rgb);
}

static void checkSocBar(unsigned segments) {
  // Independent expected physical order: never derive the assertion from
  // the production mapping whose correctness it is intended to test.
  constexpr uint8_t order[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0};
  CHECK(lastFrame().brightness == 255);
  for (unsigned logical = 0; logical < 12; ++logical) {
    CHECK(lastFrame().pixels[order[logical]] == (logical < segments ? 0x001400u : 0u));
  }
}

static void testUsbOnly() {
  led_test::reset();
  LedRing ring;
  LedStatusManager manager;
  manager.begin(&ring);
  CHECK(ring.begin());
  CHECK(ring.availability().init == core::InitializationState::DEFERRED);
  CHECK(ring.availability().detected == core::DetectedState::UNPOWERED);
  LedFrame frame{};
  for (LedEffect& pixel : frame.pixels) pixel = {255, 255, 255, 255};
  ring.setFrame(frame);
  ring.setSolid(255, 255, 255, 255);
  ring.off();
  CHECK(!ring.startTest());
  CHECK(!ring.startSocTest());
  LedStatusInputs in;
  in.system_health = core::SystemHealth::FAULT;
  for (uint32_t now : {0u, 120u, 600u, 16800u, 0xFFFFFFF0u, 80u}) {
    ring.update(now);
    manager.update(now, in);
  }
  CHECK(manager.snapshot().presentation == LedPresentationState::FAULT);
  CHECK(ring.diagnostic() == LedDiagnostic::NONE);
  CHECK(!ring.testRunning());
  CHECK(!ring.dataPinDriven());
  CHECK(led_test::begin_calls == 0);
  CHECK(led_test::frames.empty());
  CHECK(led_test::pin_modes.size() == 1);
  CHECK(led_test::pin_modes[0].pin == pins::kLedRingDin);
  for (const auto& call : led_test::pin_modes) CHECK(call.mode == INPUT);
}

static void testIndependentFrameBrightness() {
  led_test::reset();
  LedRing ring;
  CHECK(ring.begin());
  CHECK(ring.dataPinDriven());
  CHECK(led_test::begin_calls == 1);
  CHECK(ring.availability().expected == core::ExpectedState::OPTIONAL);
  CHECK(ring.availability().detected == core::DetectedState::UNKNOWN);
  checkSolid(0);
  CHECK(lastFrame().brightness == 60);

  // Literal expected transport channels cover the original NeoPixel rounding.
  ring.setSolid(160, 0, 220, 20);
  checkSolid(0x0D0012);

  // Every original presentation color at every supported brightness must
  // reach the same transport values through the independent-frame path.
  const LedEffect legacy_colors[] = {
      {255, 0, 0, 0}, {0, 80, 255, 0}, {160, 0, 220, 0},
      {255, 140, 0, 0}, {0, 200, 200, 0}};
  LedFrame uniform{};
  for (const LedEffect& color : legacy_colors) {
    for (uint8_t brightness = 0; brightness <= 60; ++brightness) {
      ring.setSolid(color.r, color.g, color.b, brightness);
      const auto expected = lastFrame().pixels;
      for (LedEffect& pixel : uniform.pixels) {
        pixel = {color.r, color.g, color.b, brightness};
      }
      ring.setFrame(uniform);
      CHECK(lastFrame().pixels == expected);
    }
  }
  ring.setSolid(255, 140, 0, 30);
  checkSolid(0x1E1000);
  ring.setSolid(255, 255, 255, 255);
  checkSolid(0x3C3C3C);
  LedFrame frame{};
  frame.pixels[0] = {160, 0, 220, 20};
  frame.pixels[1] = {255, 200, 200, 6};
  frame.pixels[2] = {255, 255, 255, 0};
  frame.pixels[3] = {255, 255, 255, 255};
  frame.pixels[4] = {0, 255, 0, 20};
  ring.setFrame(frame);
  CHECK(lastFrame().brightness == 255);
  CHECK(lastFrame().pixels[0] == 0x0D0012);
  CHECK(lastFrame().pixels[1] == 0x060505);
  CHECK(lastFrame().pixels[2] == 0);
  CHECK(lastFrame().pixels[3] == 0x3C3C3C);
  CHECK(lastFrame().pixels[4] == 0x001400);
  for (unsigned i = 5; i < 12; ++i) CHECK(lastFrame().pixels[i] == 0);
  // Returning from global 255 frame rendering must preserve old solid output.
  ring.setSolid(160, 0, 220, 20);
  checkSolid(0x0D0012);
}

static void testNativeChase() {
  led_test::reset();
  LedRing ring;
  CHECK(ring.begin());
  led_test::now_ms = 37;
  CHECK(ring.startTest());
  CHECK(ring.diagnostic() == LedDiagnostic::CHASE);
  CHECK(lastFrame().brightness == 60);
  checkSolid(0);
  const size_t start_frames = led_test::frames.size();
  for (unsigned step = 0; step < 12; ++step) {
    ring.update(37 + 120 * (step + 1) - 1);
    CHECK(led_test::frames.size() == start_frames + step);
    ring.update(37 + 120 * (step + 1));
    CHECK(ring.testRunning());
    CHECK(lastFrame().brightness == 60);
    for (unsigned pixel = 0; pixel < 12; ++pixel) {
      CHECK(lastFrame().pixels[pixel] == (pixel == step ? 0x000900u : 0u));
    }
  }
  ring.update(37 + 1560 - 1);
  CHECK(ring.testRunning());
  ring.update(37 + 1560);
  CHECK(!ring.testRunning());
  CHECK(ring.diagnostic() == LedDiagnostic::NONE);
  checkSolid(0);
  const size_t complete_frames = led_test::frames.size();
  ring.update(50000);
  CHECK(led_test::frames.size() == complete_frames);

  // Native chase retains its existing one-step-per-eligible-tick semantics.
  led_test::now_ms = 1000;
  CHECK(ring.startTest());
  ring.update(2000);
  CHECK(lastFrame().pixels[0] == 0x000900);
  ring.update(2119);
  CHECK(lastFrame().pixels[0] == 0x000900);
  ring.update(2120);
  CHECK(lastFrame().pixels[1] == 0x000900);
  ring.off();
  CHECK(!ring.testRunning());
}

static void testSocDiagnostic() {
  led_test::reset();
  LedRing ring;
  CHECK(ring.begin());
  constexpr uint32_t start = 123;
  led_test::now_ms = start;
  CHECK(ring.startSocTest());
  CHECK(ring.diagnostic() == LedDiagnostic::SOC_TEST);
  for (unsigned level = 0; level <= 12; ++level) {
    ring.update(start + 600 * level);
    CHECK(ring.testRunning());
    checkSocBar(level);
    const size_t count = led_test::frames.size();
    ring.update(start + 600 * level + 599);
    CHECK(led_test::frames.size() == count);
    checkSocBar(level);
  }
  for (uint32_t elapsed : {7800u, 8400u, 8999u}) {
    ring.update(start + elapsed);
    CHECK(ring.testRunning());
    checkSocBar(12);
  }
  for (unsigned step = 0; step <= 12; ++step) {
    ring.update(start + 9000 + 600 * step);
    CHECK(ring.testRunning());
    checkSocBar(12 - step);
    ring.update(start + 9000 + 600 * step + 599);
    checkSocBar(12 - step);
  }
  ring.update(start + 16800);
  CHECK(!ring.testRunning());
  checkSolid(0);
  const size_t complete_frames = led_test::frames.size();
  ring.update(start + 20000);
  CHECK(led_test::frames.size() == complete_frames);

  // Missing all intermediate updates must never extend the 16.8 s bound.
  led_test::now_ms = start;
  CHECK(ring.startSocTest());
  ring.update(start + 16800);
  CHECK(!ring.testRunning());
  checkSolid(0);

  // A diagnostic crossing the millis() wrap still advances and finishes.
  constexpr uint32_t wrap_start = std::numeric_limits<uint32_t>::max() - 300;
  led_test::now_ms = wrap_start;
  CHECK(ring.startSocTest());
  ring.update(wrap_start + 600u);
  checkSocBar(1);
  ring.update(wrap_start + 12000u);
  checkSocBar(7);
  ring.update(wrap_start + 16800u);
  CHECK(!ring.testRunning());

  // Controller may pass the tick captured just before the command called
  // startSocTest(), whose millis() timestamp is already one ms later.
  led_test::now_ms = 1001;
  CHECK(ring.startSocTest());
  ring.update(1000);
  CHECK(ring.testRunning());
  checkSocBar(0);
  ring.update(1601);
  checkSocBar(1);
  ring.update(17801);
  CHECK(!ring.testRunning());

  // Restart/cancel transitions leave exactly one active diagnostic.
  CHECK(ring.startSocTest());
  CHECK(ring.startTest());
  CHECK(ring.diagnostic() == LedDiagnostic::CHASE);
  CHECK(lastFrame().brightness == 60);
  CHECK(ring.startSocTest());
  CHECK(ring.diagnostic() == LedDiagnostic::SOC_TEST);
  checkSocBar(0);
  ring.off();
  CHECK(ring.diagnostic() == LedDiagnostic::NONE);
}

static void testManagerOwnership() {
  for (bool soc_test : {false, true}) {
    led_test::reset();
    LedRing ring;
    LedStatusManager manager;
    manager.begin(&ring);
    CHECK(ring.begin());
    LedStatusInputs in;
    manager.update(0, in);
    CHECK(manager.state() == LedPresentationState::BOOTING);
    checkSolid(0x060606);
    CHECK(soc_test ? ring.startSocTest() : ring.startTest());
    const size_t count = led_test::frames.size();
    in.system_health = core::SystemHealth::READY;
    in.sample_valid = true;
    in.daly_comm_ok = true;
    in.telemetry_age_ms = 0;
    in.soc_percent = 25;
    manager.update(1, in);
    CHECK(manager.snapshot().soc_valid);
    CHECK(manager.snapshot().soc_percent == 25);
    CHECK(manager.snapshot().soc_segments == 3);
    CHECK(led_test::frames.size() == count);
    in.soc_percent = 100;
    in.battery_charging = true;
    manager.update(2, in);
    CHECK(manager.snapshot().charging);
    CHECK(manager.snapshot().soc_segments == 12);
    CHECK(!manager.snapshot().charge_complete_verified);
    CHECK(led_test::frames.size() == count);
    in.telemetry_age_ms = UINT32_MAX;
    manager.update(3, in);
    CHECK(!manager.snapshot().soc_valid);
    CHECK(!manager.snapshot().charging);
    CHECK(led_test::frames.size() == count);
    in.system_health = core::SystemHealth::FAULT;
    manager.update(4, in);
    CHECK(manager.snapshot().presentation == LedPresentationState::FAULT);
    CHECK(manager.state() == LedPresentationState::FAULT);
    CHECK(led_test::frames.size() == count);
    CHECK(ring.testRunning());
    if (soc_test) {
      ring.update(16800);
    } else {
      for (uint32_t now = 120; now <= 1560; now += 120) ring.update(now);
    }
    CHECK(!ring.testRunning());
    manager.update(16801, in);
    checkSolid(0x3C0000);
    CHECK(manager.snapshot().presentation == LedPresentationState::FAULT);
  }
  LedStatusManager unbound;
  LedStatusInputs in;
  in.system_health = core::SystemHealth::DEGRADED;
  unbound.update(0, in);
  CHECK(unbound.state() == LedPresentationState::DEGRADED);
}

// 14 Oct cyan transient: the manager records every presentation change with
// its selecting inputs, without altering any frame.
static void testTransitionHistory() {
  LedStatusManager manager;  // unbound: no frames, pure policy history
  LedStatusInputs in;
  manager.update(0, in);
  CHECK(manager.transitionCount() == 1);
  CHECK(manager.transition(0).to == LedPresentationState::BOOTING);
  in.system_health = core::SystemHealth::READY;
  in.sample_valid = true; in.daly_comm_ok = true; in.telemetry_age_ms = 0; in.soc_percent = 74;
  manager.update(10, in);
  CHECK(manager.transitionCount() == 2 && manager.transition(0).to == LedPresentationState::READY);
  manager.update(20, in);  // unchanged state: no new record
  CHECK(manager.transitionCount() == 2);
  in.wifi_connecting = true;
  manager.update(30, in);
  in.wifi_connecting = false;
  manager.update(80, in);
  CHECK(manager.transitionCount() == 4);
  CHECK(manager.transition(1).to == LedPresentationState::WIFI_CONNECTING && manager.transition(1).wifi_connecting);
  CHECK(manager.transition(1).at_ms == 30 && manager.transition(0).at_ms == 80);
  CHECK(manager.transition(0).from == LedPresentationState::WIFI_CONNECTING && manager.transition(0).to == LedPresentationState::READY);
  for (uint32_t i = 0; i < 20; ++i) { in.wifi_connecting = (i % 2) == 0; manager.update(100 + i, in); }
  CHECK(manager.transitionCount() == 24);
  CHECK(manager.transition(0).at_ms == 119 && manager.transition(7).at_ms == 112);  // ring keeps the newest 8
}

// Battery P1, the 10/11 hardware session replayed as Controller now drives the
// manager: deferRender(torque) then update() on every sample. The installed
// 450d2c7 skipped update() with torque ON and warned only at 19:58:50Z. At the
// 5 s /status cadence; the device sees every 2 s DALY sample, so on hardware the
// fact can only come earlier.
static void testMotionDeferredRender() {
  led_test::reset();
  LedRing ring;
  LedStatusManager manager;
  manager.begin(&ring);
  CHECK(ring.begin());
  LedStatusInputs in;
  in.system_health = core::SystemHealth::READY;
  in.sample_valid = true;
  in.daly_comm_ok = true;
  uint32_t first_fact = 0, first_recorded = 0, first_frame_after = 0, warning_at_ms = 0;
  unsigned engaged = 0, engaged_frames = 0, later_than_installed = 0;
  bool was_engaged = false, fact_engaged = false;
  for (const LedReplaySample& r : kLedReplay20261110) {
    led_test::now_ms = r.uptime_ms;
    in.telemetry_age_ms = r.age_ms;
    in.sampled_at_ms = r.uptime_ms - r.age_ms;
    in.soc_percent = r.soc_x10 * 0.1f;  // DalyProtocol decodes reg16 * 0.1f
    in.cell_min_mv = r.cell_mv;
    in.pack_voltage_v = r.pack_dv * 0.1f;
    const size_t before = led_test::frames.size();
    manager.deferRender(r.torque);
    manager.update(r.uptime_ms, in);
    CHECK(manager.renderDeferred() == r.torque);
    if (r.torque) { ++engaged; engaged_frames += led_test::frames.size() - before; was_engaged = true; }
    if (!first_fact && manager.snapshot().battery_warning) {
      first_fact = r.utc_hhmmss; fact_engaged = r.torque; warning_at_ms = r.uptime_ms;
    }
    if (!first_recorded && r.warned) first_recorded = r.utc_hhmmss;
    if (r.warned && !manager.snapshot().battery_warning) ++later_than_installed;
    if (!r.torque && was_engaged && !first_frame_after && led_test::frames.size() > before) {
      first_frame_after = r.utc_hhmmss;
      // The catch-up frame is the current warning, never the pre-Motion green bar.
      CHECK(manager.state() == LedPresentationState::BATTERY_WARNING);
      for (uint32_t pixel : lastFrame().pixels) {
        CHECK(pixel == lastFrame().pixels[0] && (pixel & 0xFF) == 0 && (pixel >> 16) > ((pixel >> 8) & 0xFF));
      }
    }
    CHECK(!manager.snapshot().battery_critical);  // 3375 mV / 10.2 V at the end: never critical
  }
  std::printf("replay 10/11: %zu samples, %u engaged, warning fact %06lu%s (installed 450d2c7: %06lu), "
              "frames while engaged %u, first frame after torque OFF %06lu\n",
              sizeof(kLedReplay20261110) / sizeof(kLedReplay20261110[0]), engaged,
              static_cast<unsigned long>(first_fact), fact_engaged ? " torque ON" : "",
              static_cast<unsigned long>(first_recorded), engaged_frames,
              static_cast<unsigned long>(first_frame_after));
  CHECK(engaged == 463);  // 19:20:15Z-19:58:45Z
  CHECK(first_fact == 194655 && fact_engaged);  // second fresh sample at pack <= 10.50 V
  CHECK(first_recorded == 195850);
  CHECK(later_than_installed == 0);
  CHECK(engaged_frames == 0);  // no WS2812/RMT transmission while Motion is engaged
  CHECK(first_frame_after == 195850);
  CHECK(manager.state() == LedPresentationState::BATTERY_WARNING);
  CHECK(manager.transition(0).to == LedPresentationState::BATTERY_WARNING &&
        manager.transition(0).at_ms == warning_at_ms);

  // Same trend continued with torque ON again: critical is immediate at
  // 3350 mV, still without a transmission; torque OFF renders it at once.
  uint32_t now = kLedReplay20261110[sizeof(kLedReplay20261110) / sizeof(kLedReplay20261110[0]) - 1].uptime_ms;
  const size_t frames = led_test::frames.size();
  for (uint16_t cell : {3360, 3351, 3350}) {
    now += 2000;
    led_test::now_ms = now;
    in.telemetry_age_ms = 300; in.sampled_at_ms = now - 300; in.cell_min_mv = cell;
    manager.deferRender(true);
    manager.update(now, in);
    CHECK(manager.snapshot().battery_critical == (cell == 3350));
  }
  CHECK(led_test::frames.size() == frames);
  CHECK(manager.state() == LedPresentationState::BATTERY_CRITICAL);
  now += 2000 - now % 500;  // first half of the 2 Hz blink: red at full ceiling
  led_test::now_ms = now;
  manager.deferRender(false);
  manager.update(now, in);
  CHECK(led_test::frames.size() == frames + 1);
  checkSolid(0x3C0000);
}

int main() {
  CHECK(std::strcmp(toString(LedDiagnostic::NONE), "NONE") == 0);
  CHECK(std::strcmp(toString(LedDiagnostic::CHASE), "CHASE") == 0);
  CHECK(std::strcmp(toString(LedDiagnostic::SOC_TEST), "SOC_TEST") == 0);
  if (build::kLedRailPowered) {
    testIndependentFrameBrightness();
    testNativeChase();
    testSocDiagnostic();
    testManagerOwnership();
    testTransitionHistory();
    testMotionDeferredRender();
  } else {
    testUsbOnly();
  }
  std::printf("test_led_ring_manager[%s]: %u checks, %u failures\n",
              build::kTestProfile, checks, failures);
  return failures == 0 ? 0 : 1;
}
