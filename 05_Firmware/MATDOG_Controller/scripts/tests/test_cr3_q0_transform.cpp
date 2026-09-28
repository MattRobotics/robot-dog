#include <cstdio>
#include <cstring>

#include "../../src/actuator/CalibrationQ0Promotion.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"

using namespace matdog::actuator;
using namespace matdog::calibration;

static int g_checks = 0;
static int g_failures = 0;

static void check(bool ok, const char* label) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("FAIL: %s\n", label);
  }
}

static CalibrationGeometryProfile profile() {
  CalibrationGeometryProfile p;
  p.bind(&geometry_data::kProvenance,
         geometry_data::kJoints, geometry_data::kJointCount,
         geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return p;
}

static uint16_t centerShift(uint16_t tick) {
  int d = static_cast<int>(tick) - static_cast<int>(kServoRawCenter);
  if (d < 0) d = -d;
  const int wrapped = kTicksPerRevolution - d;
  return static_cast<uint16_t>(wrapped < d ? wrapped : d);
}

static Q0BootstrapCandidate candidateFor(const GeometryJointRecord& j,
                                         uint16_t tick,
                                         uint32_t session = 1,
                                         uint8_t samples = 9,
                                         uint16_t spread = 0) {
  Q0BootstrapCandidate c{};
  c.status = Q0BootstrapStatus::CANDIDATE;
  c.geometry = geometryProvenanceTag(geometry_data::kProvenance);
  c.bus_id = j.bus_id;
  c.capture_session_id = session;
  c.sample_count = samples;
  c.stability_spread_ticks = spread;
  c.evidence.measured = true;
  c.evidence.estimator = Q0Estimator::MANUAL_ZERO_POSE;
  c.evidence.state = EvidenceState::CANDIDATE;
  c.evidence.origin = CalibrationOrigin::LIVE_SESSION;
  c.evidence.identity = j.identity;
  c.evidence.tick = tick;
  c.evidence.shift_from_digital_home_ticks = centerShift(tick);
  c.evidence.accepted_by_gate = false;
  return c;
}

static void test_mechanical_threshold_is_independent() {
  check(kQ0PlausibilityTicks == 80, "mechanical q0 threshold is 80 ticks");
  check(static_cast<uint32_t>(kQ0PlausibilityTicks) * 2u * kOutputSplineTeeth <
            static_cast<uint32_t>(kTicksPerRevolution),
        "80 ticks is strictly below exact half tooth");
  check(kQ0PlausibilityTicks > 63, "CR2-C max 63 is not used as threshold");
}

static void test_all_cr2c_candidates_accept_and_promote() {
  // Geometry-table order, with q0 values from the canonical CR2-C package.
  const uint16_t q0[geometry_data::kJointCount] = {
      1996, 2087, 2100, 2035, 2073, 2089,
      2030, 1985, 2092, 2081, 2034, 2042,
  };
  CalibrationGeometryProfile p = profile();

  for (uint8_t i = 0; i < geometry_data::kJointCount; ++i) {
    const auto c = candidateFor(geometry_data::kJoints[i], q0[i], 77);
    const AcceptedQ0 a = acceptQ0Candidate(p, geometry_data::kProvenance, c);
    check(a.accepted(), "CR2-C current candidate accepted");
    check(a.evidence.tick == q0[i], "accepted q0 unchanged");

    Q0PromotionRequest req{};
    req.explicit_currentness_confirmation = true;
    req.capture_session_id = 77;
    const PromotedQ0 promoted =
        promoteAcceptedQ0(p, geometry_data::kProvenance, a, req);
    check(promoted.promoted(), "accepted q0 promoted");
    check(promoted.transform.q0_tick == q0[i], "promoted transform preserves q0");
    check(promoted.transform.geometry == p.provenanceTag(), "promoted geometry current");
  }
}

static void test_acceptance_fails_closed() {
  CalibrationGeometryProfile p = profile();
  Q0BootstrapCandidate c = candidateFor(geometry_data::kJoints[0], 2129); // 81 from centre
  check(acceptQ0Candidate(p, geometry_data::kProvenance, c).status ==
            Q0AcceptanceStatus::REJECT_PLAUSIBILITY,
        "81 tick q0 shift refused");

  c = candidateFor(geometry_data::kJoints[0], 2128); // exactly 80
  check(acceptQ0Candidate(p, geometry_data::kProvenance, c).accepted(),
        "80 tick q0 shift accepted");

  c = candidateFor(geometry_data::kJoints[0], 2048, 1, 8, 0);
  check(acceptQ0Candidate(p, geometry_data::kProvenance, c).status ==
            Q0AcceptanceStatus::REJECT_SAMPLE_POLICY,
        "fewer than nine samples refused");

  c = candidateFor(geometry_data::kJoints[0], 2048, 1, 9, 17);
  check(acceptQ0Candidate(p, geometry_data::kProvenance, c).status ==
            Q0AcceptanceStatus::REJECT_STABILITY,
        "spread beyond 16 refused");

  c = candidateFor(geometry_data::kJoints[0], 2048);
  c.bus_id = 99;
  check(acceptQ0Candidate(p, geometry_data::kProvenance, c).status ==
            Q0AcceptanceStatus::REJECT_BUS_BINDING,
        "wrong bus metadata refused");

  c = candidateFor(geometry_data::kJoints[0], 2048);
  c.evidence.shift_from_digital_home_ticks = 1;
  check(acceptQ0Candidate(p, geometry_data::kProvenance, c).status ==
            Q0AcceptanceStatus::REJECT_MALFORMED_DIAGNOSTIC,
        "tampered centre-shift diagnostic refused");

  c = candidateFor(geometry_data::kJoints[0], 2048);
  const AcceptedQ0 a = acceptQ0Candidate(p, geometry_data::kProvenance, c);
  Q0PromotionRequest req{};
  req.capture_session_id = c.capture_session_id;
  check(promoteAcceptedQ0(p, geometry_data::kProvenance, a, req).status ==
            Q0PromotionStatus::REJECT_CURRENTNESS_NOT_CONFIRMED,
        "promotion needs explicit currentness transaction");
  req.explicit_currentness_confirmation = true;
  req.capture_session_id++;
  check(promoteAcceptedQ0(p, geometry_data::kProvenance, a, req).status ==
            Q0PromotionStatus::REJECT_SESSION_MISMATCH,
        "promotion cannot cross capture session");
}

static JointTransform promotedTransform(const GeometryJointRecord& j, uint16_t q0) {
  JointTransform t{};
  t.identity = j.identity;
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.geometry = geometryProvenanceTag(geometry_data::kProvenance);
  t.q0_tick = q0;
  t.present = true;
  return t;
}

static void test_q_raw_resolver() {
  CalibrationGeometryProfile p = profile();
  const GeometryJointRecord& j = geometry_data::kJoints[2]; // LF upper, direction +1
  JointTransform t = promotedTransform(j, 2100);

  uint16_t raw = 0;
  check(resolveUrdfQToRaw(p, geometry_data::kProvenance, t, 0, &raw) ==
            TargetResolveStatus::OK && raw == 2100,
        "q zero maps exactly to q0");

  const MicroRad q = 500000;
  check(resolveUrdfQToRaw(p, geometry_data::kProvenance, t, q, &raw) ==
            TargetResolveStatus::OK,
        "positive in-range q resolves");
  check(raw > t.q0_tick, "direction +1 increases raw");

  MicroRad decoded = 0;
  check(resolveRawToUrdfQ(p, geometry_data::kProvenance, t, raw, &decoded) ==
            TargetResolveStatus::OK,
        "resolved raw decodes");
  int32_t error = decoded - q;
  if (error < 0) error = -error;
  check(error <= 800, "q/raw roundtrip within one tick quantization");

  check(resolveUrdfQToRaw(p, geometry_data::kProvenance, t,
                          j.urdf_upper + 1, &raw) ==
            TargetResolveStatus::REJECT_URDF_LIMIT,
        "q beyond URDF refused");

  JointTransform edge = t;
  edge.q0_tick = 4090;
  check(resolveUrdfQToRaw(p, geometry_data::kProvenance, edge, 100000, &raw) ==
            TargetResolveStatus::REJECT_RAW_DOMAIN,
        "absolute raw overflow refused instead of wrapped");

  check(resolveDeltaFromQ0(p, geometry_data::kProvenance, t, -10, &raw) ==
            TargetResolveStatus::OK && raw == 2090,
        "direction diagnostic delta resolves around q0");
  edge.q0_tick = 2;
  check(resolveDeltaFromQ0(p, geometry_data::kProvenance, edge, -3, &raw) ==
            TargetResolveStatus::REJECT_RAW_DOMAIN,
        "direction delta never signed-wraps");

  GeometryProvenance wrong = geometry_data::kProvenance;
  wrong.urdf_sha256[0] = wrong.urdf_sha256[0] == 'a' ? 'b' : 'a';
  check(resolveUrdfQToRaw(p, wrong, t, 0, &raw) ==
            TargetResolveStatus::REJECT_GEOMETRY,
        "wrong geometry refuses resolver");
}

static void test_direction_is_from_geometry() {
  CalibrationGeometryProfile p = profile();
  const GeometryJointRecord& j = geometry_data::kJoints[8]; // RF upper, direction -1
  JointTransform t = promotedTransform(j, 2092);
  uint16_t raw = 0;
  check(jointDirection(p, j.identity) == -1, "test joint uses current URDF direction -1");
  check(resolveUrdfQToRaw(p, geometry_data::kProvenance, t, 300000, &raw) ==
            TargetResolveStatus::OK,
        "negative-direction joint resolves");
  check(raw < t.q0_tick, "direction -1 decreases raw for positive q");
}

int main() {
  test_mechanical_threshold_is_independent();
  test_all_cr2c_candidates_accept_and_promote();
  test_acceptance_fails_closed();
  test_q_raw_resolver();
  test_direction_is_from_geometry();

  std::printf("test_cr3_q0_transform: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
