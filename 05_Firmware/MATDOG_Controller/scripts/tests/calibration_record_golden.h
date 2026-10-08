#ifndef MATDOG_TESTS_CALIBRATION_RECORD_GOLDEN_H
#define MATDOG_TESTS_CALIBRATION_RECORD_GOLDEN_H

// The 24/24 Full Calibration of 2026-10-01 (evidence export
// evidence_export_20261001_180841.txt, session dfcecb670d05, geometry tag
// 3713f4ddc43b204e), transcribed as test data. Offline: nothing here talks to
// hardware; the numbers are the ones the robot produced.

#include <string.h>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/calibration/CalibrationRecord.h"

namespace golden {

using namespace matdog;
using namespace matdog::calibration;

struct GoldenContact { uint16_t scout, fine1, fine2, rep; };
struct GoldenDiag { uint16_t min, max, expected, measured, scale, affine, shift, fixed; };
struct GoldenJoint {
  uint8_t leg, joint;
  const char* unit;
  uint8_t bus;
  uint16_t q0;
  GoldenContact c[2];
  GoldenDiag d;
};

// Index = leg * 3 + JointKind (HIP, UPPER, LOWER); legs LF, RF, RH, LH.
constexpr GoldenJoint kJoints[12] = {
    {0, 0, "M22",   13, 1975, {{2511, 2506, 2503, 3}, {1489, 1476, 1472, 4}}, {2504, 1474, 1024, 1030, 1006, 1989, 14, 6}},
    {0, 1, "ELR01", 12, 2078, {{1460, 1463, 1459, 4}, {3479, 3476, 3478, 2}}, {1461, 3477, 1991, 2016, 1013, 2065, 13, 25}},
    {0, 2, "M33",   11, 2104, {{3152, 3149, 3149, 0}, {1706, 1712, 1712, 0}}, {3149, 1712, 1474, 1437, 975, 2128, 24, 37}},
    {1, 0, "NEW01", 23, 2030, {{2551, 2545, 2543, 2}, {1492, 1498, 1497, 1}}, {2544, 1497, 1024, 1047, 1022, 2020, 10, 23}},
    {1, 1, "ELR03", 22, 2108, {{2726, 2730, 2726, 4}, {717, 721, 722, 1}},    {2728, 721, 1991, 2007, 1008, 2126, 18, 16}},
    {1, 2, "NEW03", 21, 1995, {{936, 938, 937, 1}, {2364, 2359, 2356, 3}},    {937, 2357, 1474, 1420, 963, 1946, 49, 54}},
    {2, 0, "NEW06", 33, 2081, {{1557, 1563, 1565, 2}, {2603, 2595, 2595, 0}}, {1564, 2595, 1024, 1031, 1007, 2080, 1, 7}},
    {2, 1, "ELR02", 32, 2061, {{2672, 2668, 2670, 2}, {655, 659, 659, 0}},    {2669, 659, 1991, 2010, 1010, 2066, 5, 19}},
    {2, 2, "NEW05", 31, 2034, {{987, 992, 991, 1}, {2429, 2424, 2421, 3}},    {991, 2422, 1474, 1431, 971, 2007, 27, 43}},
    {3, 0, "M43",   43, 2026, {{1494, 1502, 1502, 0}, {2546, 2541, 2543, 2}}, {1502, 2542, 1024, 1040, 1016, 2022, 4, 16}},
    {3, 1, "M42",   42, 2098, {{1482, 1486, 1488, 2}, {3489, 3484, 3485, 1}}, {1487, 3484, 1991, 1997, 1003, 2086, 12, 6}},
    {3, 2, "M41",   41, 2073, {{3133, 3133, 3133, 0}, {1697, 1702, 1705, 3}}, {3133, 1703, 1474, 1430, 970, 2117, 44, 44}},
};

inline int nibble(char c) { return c <= '9' ? c - '0' : c - 'a' + 10; }

inline const char* provenanceHex(const actuator::GeometryProvenance& p, int i) {
  switch (i) {
    case 0: return p.urdf_sha256;
    case 1: return p.mesh_manifest_sha256;
    case 2: return p.endpoint_semantic_sha256;
    case 3: return p.parking_semantic_sha256;
    case 4: return p.safety_policy_semantic_sha256;
    default: return p.allocation_sha256;
  }
}

inline actuator::CalibrationGeometryProfile boundProfile() {
  actuator::CalibrationGeometryProfile profile;
  profile.bind(&actuator::geometry_data::kProvenance, actuator::geometry_data::kJoints,
               actuator::geometry_data::kJointCount, actuator::geometry_data::kEndpoints,
               actuator::geometry_data::kEndpointCount);
  return profile;
}

inline CalibrationRecord goldenRecord(uint32_t generation = 1) {
  const actuator::CalibrationGeometryProfile profile = boundProfile();
  CalibrationRecord r;
  r.generation = generation;
  r.geometry_tag = profile.provenanceTag();
  for (int i = 0; i < 6; ++i) {
    const char* hex = provenanceHex(actuator::geometry_data::kProvenance, i);
    for (int b = 0; b < 32; ++b) {
      r.digest[i][b] = static_cast<uint8_t>((nibble(hex[2 * b]) << 4) | nibble(hex[2 * b + 1]));
    }
  }
  strcpy(r.build_id, "golden-test");
  r.parameters_approved = 0;
  r.calibration_accepted = 1;
  r.contact_margin_ticks = 8;

  static const uint8_t kSessionPark[4] = {1, 1, 0, 0};
  for (uint8_t l = 0; l < 4; ++l) {
    CalibrationRecordLegV1& leg = r.leg[l];
    leg.leg = l;
    leg.verdict = static_cast<uint8_t>(FullLegVerdict::HARDWARE_CONTACT_CALIBRATED);
    leg.attempts = 1;
    leg.contacts_measured = 6;
    leg.contacts_accepted = 6;
    leg.diagnostics_accepted = 1;
    leg.session_completed = 1;
    leg.permit_revoked = 1;
    leg.authority_released = 1;
    leg.has_rear_park = kSessionPark[l];
    if (leg.has_rear_park) {
      leg.park_leg = l == 0 ? 3 : 2;  // LF parks against LH UPPER, RF against RH UPPER
      leg.park_joint = 1;
      leg.park_bus_id = l == 0 ? 42 : 32;
      leg.park_target_urad = 610865;
    }
  }

  for (uint8_t i = 0; i < 12; ++i) {
    const GoldenJoint& g = kJoints[i];
    CalibrationRecordJointV1& j = r.joint[i];
    j.leg = g.leg;
    j.joint = g.joint;
    strncpy(j.unit, g.unit, sizeof(j.unit) - 1);
    j.bus_id = g.bus;
    const actuator::GeometryJointRecord* installed = nullptr;
    JointIdentity id{};
    id.leg = static_cast<Leg>(g.leg);
    id.joint = static_cast<JointKind>(g.joint);
    setPhysicalUnit(&id, g.unit);
    installed = profile.findJoint(id);
    j.encoder_direction = installed ? installed->encoder_direction : 0;
    j.encoder_direction_source = installed ? static_cast<uint8_t>(installed->encoder_direction_source) : 0;
    j.q0_tick = g.q0;
    j.q0_estimator = static_cast<uint8_t>(Q0Estimator::MANUAL_ZERO_POSE);
    j.q0_state = static_cast<uint8_t>(EvidenceState::PROMOTED);
    j.q0_origin = static_cast<uint8_t>(CalibrationOrigin::LIVE_SESSION);
    j.q0_sample_count = 9;
    j.q0_stability_spread_ticks = 0;
    for (int s = 0; s < 2; ++s) {
      CalibrationRecordContactV1& c = j.contact[s];
      c.detection = static_cast<uint8_t>(ContactState::CONTACT_CONFIRMED);
      c.state = static_cast<uint8_t>(EvidenceState::PROMOTED);
      c.origin = static_cast<uint8_t>(CalibrationOrigin::LIVE_SESSION);
      c.witness_accepted = 1;
      c.scout_tick = g.c[s].scout;
      c.fine_tick_1 = g.c[s].fine1;
      c.fine_tick_2 = g.c[s].fine2;
      c.repeatability_ticks = g.c[s].rep;
    }
    CalibrationRecordDiagnosticsV1& d = j.diagnostics;
    d.evaluated = 1;
    d.ordered = 1;
    d.accepted = 1;
    d.min_contact_tick = g.d.min;
    d.max_contact_tick = g.d.max;
    d.expected_span_ticks = g.d.expected;
    d.measured_span_ticks = g.d.measured;
    d.scale_permille = g.d.scale;
    d.affine_zero_tick = g.d.affine;
    d.affine_shift_from_q0_ticks = g.d.shift;
    d.fixed_endpoint_disagreement_ticks = g.d.fixed;
  }
  return r;
}

}  // namespace golden

#endif  // MATDOG_TESTS_CALIBRATION_RECORD_GOLDEN_H
