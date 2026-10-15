// 14 Oct 2026 hardware counterexample, reproduced offline: @TEL reported
// stale=true while the BNO085 quaternion was 8 ms old and all twelve raw
// ticks were fresh, because no RESTORE had mapped the joints. Channels must be
// independent; unmapped joints stay null (never 0.0); the legacy motion
// "stale" keeps its fail-closed meaning for motion consumers.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "../../src/core/MotionService.h"

using namespace matdog;
using namespace matdog::core;
static int checks = 0, failures = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)

static std::string status(MotionService& s, uint32_t now, size_t cap = 4096) {
  std::vector<char> b(cap); if (!s.statusJson(b.data(), b.size(), now)) return std::string();
  return std::string(b.data());
}
static bool has(const std::string& s, const char* needle) { return s.find(needle) != std::string::npos; }

int main() {
  MotionService service; service.begin(nullptr, nullptr);
  MotionTelemetry t;
  // Values copied from the first real @TEL of 2026-10-14T17:14:55Z (sanitized).
  const uint32_t now = 892599;
  const uint16_t raw[12] = {2018,3002,3146,1999,1196,944,2063,1126,994,2049,3023,3131};
  const uint32_t at[12] = {892400,892431,892464,892493,892524,892559,892584,892247,892281,892311,892340,892370};
  for (unsigned i = 0; i < 12; ++i) { t.raw_ticks[i] = raw[i]; t.joint_at_ms[i] = at[i]; t.raw_ok[i] = true; t.joint_mapped[i] = false; t.actual_rad[i] = 0.0; }
  t.raw_valid = true; t.joints_valid = false; t.sampled_at_ms = 892247;
  t.quaternion_valid = true; t.quaternion_at_ms = 892591;
  t.quaternion[0] = .12493896; t.quaternion[1] = .00524902; t.quaternion[2] = .00561523; t.quaternion[3] = -.99212646;
  t.source = "HARDWARE_SERVO_FEEDBACK";
  service.publishTelemetry(t);
  std::string j = status(service, now);
  CHECK(!j.empty());
  CHECK(has(j, "\"stale\":true"));                         // legacy motion meaning: no measured joints
  CHECK(has(j, "\"stale_scope\":\"MOTION_JOINT_FEEDBACK\""));
  CHECK(has(j, "\"imu\":{\"valid\":true,\"age_ms\":8}"));  // the counterexample: IMU is fresh
  CHECK(has(j, "\"raw\":{\"count\":12,\"max_age_ms\":352}"));
  CHECK(has(j, "\"mapped\":{\"count\":0,\"mask\":0}"));
  CHECK(has(j, "\"actual_rad\":null"));
  CHECK(has(j, "\"actual_rad_by_joint\":[null,null,null,null,null,null,null,null,null,null,null,null]"));
  CHECK(!has(j, "\"actual_rad_by_joint\":[0"));            // never a fake zero
  CHECK(has(j, "\"quaternion\":[0.12493896"));
  CHECK(has(j, "\"raw_ticks\":[2018,3002,3146"));
  CHECK(has(j, "\"restore\":{\"decision\":\"NOT_EVALUATED\""));

  // Partial mapping: 11/12 measured -> legacy actual_rad stays null (all-or-
  // nothing for motion), per-joint view shows 11 values and one null.
  for (unsigned i = 0; i < 12; ++i) { t.joint_mapped[i] = i != 4; t.actual_rad[i] = 0.1 * (i + 1); }
  t.joints_valid = false;
  service.publishTelemetry(t);
  j = status(service, now);
  CHECK(has(j, "\"actual_rad\":null"));
  CHECK(has(j, "\"actual_rad_by_joint\":[0.10000000,0.20000000,0.30000000,0.40000000,null,0.60000000"));
  CHECK(has(j, "\"mapped\":{\"count\":11,\"mask\":4079}"));
  CHECK(has(j, "\"motion_feedback_stale\":true"));

  // Fully mapped and fresh -> legacy stale false, all channels valid.
  for (unsigned i = 0; i < 12; ++i) t.joint_mapped[i] = true;
  t.joints_valid = true;
  service.publishTelemetry(t);
  j = status(service, now);
  CHECK(has(j, "\"stale\":false") && has(j, "\"motion_feedback_stale\":false"));
  CHECK(has(j, "\"actual_rad\":[0.10000000"));
  // Same frame 600 ms later: joints too old for motion, IMU independently aged.
  j = status(service, now + 600);
  CHECK(has(j, "\"stale\":true"));
  CHECK(has(j, "\"imu\":{\"valid\":true,\"age_ms\":608}"));
  CHECK(has(j, "\"quaternion\":null"));  // legacy quaternion still bounded at 500 ms

  // IMU invalid while joints are fine: only the IMU channel says so.
  t.quaternion_valid = false; service.publishTelemetry(t);
  j = status(service, now);
  CHECK(has(j, "\"imu\":{\"valid\":false,\"age_ms\":null}") && has(j, "\"stale\":false"));

  // Raw read failures are counted per channel; restore fields are reported.
  t.raw_ok[0] = t.raw_ok[1] = false; t.restore_decision = "PENDING_OPERATOR_REVIEW";
  t.restore_reason = "INSTALLATION_ATTESTATION_ABSENT"; t.restore_trigger = "BOOT"; t.restore_generation = 1; t.restore_fingerprint = 0x9d5323b7d59bb080ull;
  service.publishTelemetry(t);
  j = status(service, now);
  CHECK(has(j, "\"raw\":{\"count\":10"));
  CHECK(has(j, "\"restore\":{\"decision\":\"PENDING_OPERATOR_REVIEW\",\"reason\":\"INSTALLATION_ATTESTATION_ABSENT\",\"trigger\":\"BOOT\",\"generation\":1,\"fingerprint\":\"9d5323b7d59bb080\"}"));
  // Size budget: the status must fit the 4096-byte HTTP text buffer with the
  // transport_readonly and transport stats suffixes (~230 bytes) appended.
  for (unsigned i = 0; i < 12; ++i) t.actual_rad[i] = -1.60570291 + i * 1e-8;
  service.publishTelemetry(t);
  j = status(service, now);
  std::printf("status_json_bytes=%zu\n", j.size());
  CHECK(j.size() + 260 < 4096);
  CHECK(status(service, now, 256).empty());  // a too-small buffer fails, it is never truncated into "valid" JSON
  std::printf("TELEMETRY_CHANNELS: %d checks, %d failures (offline)\n", checks, failures);
  return failures ? 1 : 0;
}
