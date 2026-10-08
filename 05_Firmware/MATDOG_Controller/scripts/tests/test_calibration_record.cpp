// Offline tests for Calibration Record V1 (src/calibration/CalibrationRecord.*):
// codec, CRC-32, validation and builder. No storage here (see
// test_calibration_record_store.cpp) and no hardware: the golden vector is the
// 24/24 calibration of 2026-10-01, transcribed as data.
//
// The validator re-derives all twelve joint diagnostics through the
// production deriveFullLegJointDiagnostics(); the golden record passing is
// therefore a cross-check of the recorded hardware numbers against the code.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "calibration_record_golden.h"

using namespace matdog;
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
    const long a_ = (long)(actual);                                        \
    const long e_ = (long)(expected);                                      \
    if (a_ != e_) {                                                        \
      ++g_failures;                                                        \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,  \
                  __FILE__, __LINE__, #actual, a_, e_);                    \
    }                                                                      \
  } while (0)

#define CHECK_STATUS(actual, expected)                                                   \
  do {                                                                                   \
    ++g_checks;                                                                          \
    const CalibrationRecordStatus a_ = (actual);                                         \
    if (a_ != (expected)) {                                                              \
      ++g_failures;                                                                      \
      std::printf("  FAIL [%s] %s:%d: %s == %s, expected %s\n", g_case, __FILE__,        \
                  __LINE__, #actual, toString(a_), toString(expected));                  \
    }                                                                                    \
  } while (0)

namespace {

using golden::boundProfile;
using golden::goldenRecord;
using actuator::CalibrationGeometryProfile;
using CRS = CalibrationRecordStatus;

constexpr size_t kSize = kCalibrationRecordV1EncodedBytes;
// Frozen CRC-32 of the golden record encoded at generation 1: any silent change
// of the wire format (field order, width, endianness) breaks this.
constexpr uint32_t kGoldenCrc = 0xA8D1F28Du;

std::vector<uint8_t> encode(const CalibrationRecord& r) {
  std::vector<uint8_t> out(kSize + 64);
  size_t n = 0;
  CHECK_STATUS(encodeCalibrationRecord(r, out.data(), out.size(), &n), CRS::OK);
  out.resize(n);
  return out;
}

void refreshCrc(std::vector<uint8_t>* b) {
  const uint32_t crc = calibrationCrc32(b->data(), b->size() - 4);
  for (int i = 0; i < 4; ++i) (*b)[b->size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
}

CRS validate(const CalibrationRecord& r) { return validateCalibrationRecord(r, boundProfile()); }

// ---------------------------------------------------------------------------

void test_crc32() {
  g_case = "crc32";
  const char* v = "123456789";
  CHECK_EQ(calibrationCrc32(reinterpret_cast<const uint8_t*>(v), 9), 0xCBF43926u);
  CHECK_EQ(calibrationCrc32(nullptr, 0), 0u);
}

void test_golden_vector() {
  g_case = "golden vector";
  const CalibrationRecord r = goldenRecord(1);
  CHECK_STATUS(validate(r), CRS::OK);

  const std::vector<uint8_t> b = encode(r);
  CHECK_EQ(b.size(), 1088);
  CHECK_EQ(b.size(), kCalibrationRecordV1EncodedBytes);
  // Envelope, byte by byte: "MDCR", schema 1, flags 0, length 0x0440, generation 1.
  const uint8_t head[16] = {0x4D, 0x44, 0x43, 0x52, 0x01, 0x00, 0x00, 0x00,
                            0x40, 0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  CHECK(std::memcmp(b.data(), head, 16) == 0);
  // geometry tag (little-endian u64 3713f4ddc43b204e) right after the envelope.
  CHECK_EQ(r.geometry_tag, 0x3713f4ddc43b204eULL);
  const uint8_t tag[8] = {0x4E, 0x20, 0x3B, 0xC4, 0xDD, 0xF4, 0x13, 0x37};
  CHECK(std::memcmp(b.data() + 16, tag, 8) == 0);
  // Trailer is the CRC of everything before it, and matches the frozen value.
  CHECK_EQ(calibrationCrc32(b.data(), b.size() - 4), kGoldenCrc);
  uint32_t trailer = 0;
  for (int i = 3; i >= 0; --i) trailer = (trailer << 8) | b[b.size() - 4 + i];
  CHECK_EQ(trailer, kGoldenCrc);

  // Spot-check a few hardware numbers in the golden table.
  CHECK_EQ(r.joint[0].q0_tick, 1975);   // LF HIP M22
  CHECK_EQ(r.joint[5].bus_id, 21);      // RF LOWER NEW03
  CHECK(std::strcmp(r.joint[11].unit, "M41") == 0);
  CHECK_EQ(r.joint[11].diagnostics.fixed_endpoint_disagreement_ticks, 44);
}

void test_round_trip() {
  g_case = "round trip";
  const CalibrationRecord r = goldenRecord(7);
  const std::vector<uint8_t> b = encode(r);
  CalibrationRecord d;
  CHECK_STATUS(decodeCalibrationRecord(b.data(), b.size(), &d), CRS::OK);
  CHECK_EQ(d.generation, 7);
  CHECK_STATUS(validate(d), CRS::OK);
  const std::vector<uint8_t> again = encode(d);
  CHECK(again == b);
  CHECK(std::strcmp(d.build_id, "golden-test") == 0);
  CHECK_EQ(d.joint[8].contact[1].fine_tick_2, 2421);

  uint16_t schema = 0;
  uint32_t gen = 0;
  CHECK_STATUS(inspectCalibrationEnvelope(b.data(), b.size(), &schema, &gen), CRS::OK);
  CHECK_EQ(schema, 1);
  CHECK_EQ(gen, 7);

  // Small buffer is refused, nothing written.
  uint8_t small[100];
  size_t n = 99;
  CHECK_STATUS(encodeCalibrationRecord(r, small, sizeof(small), &n), CRS::BUFFER_TOO_SMALL);
  CHECK_EQ(n, 0);
}

void test_codec_rejects_damage() {
  g_case = "bit flips and truncation";
  const std::vector<uint8_t> good = encode(goldenRecord(3));
  CalibrationRecord d;

  // Every single-bit flip, anywhere, is refused (CRC, magic or length).
  int accepted = 0;
  for (size_t i = 0; i < good.size(); ++i) {
    for (int bit = 0; bit < 8; ++bit) {
      std::vector<uint8_t> b = good;
      b[i] ^= static_cast<uint8_t>(1u << bit);
      if (decodeCalibrationRecord(b.data(), b.size(), &d) == CRS::OK) ++accepted;
    }
  }
  CHECK_EQ(accepted, 0);

  // Truncation at every length, and a trailing extra byte.
  int ok_truncated = 0;
  for (size_t n = 0; n < good.size(); ++n) {
    if (decodeCalibrationRecord(good.data(), n, &d) == CRS::OK) ++ok_truncated;
  }
  CHECK_EQ(ok_truncated, 0);
  CHECK_STATUS(decodeCalibrationRecord(good.data(), 0, &d), CRS::TRUNCATED);
  CHECK_STATUS(decodeCalibrationRecord(good.data(), 19, &d), CRS::TRUNCATED);
  CHECK_STATUS(decodeCalibrationRecord(good.data(), good.size() - 1, &d), CRS::BAD_LENGTH);
  std::vector<uint8_t> longer = good;
  longer.push_back(0);
  CHECK_STATUS(decodeCalibrationRecord(longer.data(), longer.size(), &d), CRS::BAD_LENGTH);
  CHECK_STATUS(decodeCalibrationRecord(nullptr, 0, &d), CRS::TRUNCATED);

  std::vector<uint8_t> b = good;
  b[0] ^= 0xFF;
  CHECK_STATUS(decodeCalibrationRecord(b.data(), b.size(), &d), CRS::BAD_MAGIC);
  b = good;
  b[good.size() - 1] ^= 0x01;
  CHECK_STATUS(decodeCalibrationRecord(b.data(), b.size(), &d), CRS::BAD_CRC);
  b = good;
  b[100] ^= 0x01;
  CHECK_STATUS(decodeCalibrationRecord(b.data(), b.size(), &d), CRS::BAD_CRC);
}

void test_incompatible_schema() {
  g_case = "schema";
  std::vector<uint8_t> b = encode(goldenRecord(9));
  b[4] = 2;  // schema 2, CRC refreshed: an intact envelope this build cannot read
  refreshCrc(&b);
  CalibrationRecord d;
  CHECK_STATUS(decodeCalibrationRecord(b.data(), b.size(), &d), CRS::UNSUPPORTED_SCHEMA);
  CHECK(classifyCalibrationRecordStatus(CRS::UNSUPPORTED_SCHEMA) == CalibrationRecordClass::INCOMPATIBLE);
  uint16_t schema = 0;
  uint32_t gen = 0;
  CHECK_STATUS(inspectCalibrationEnvelope(b.data(), b.size(), &schema, &gen), CRS::OK);
  CHECK_EQ(schema, 2);
  CHECK_EQ(gen, 9);  // generation stays readable for monotonicity

  CalibrationRecord r = goldenRecord(1);
  r.schema = 2;
  CHECK_STATUS(validate(r), CRS::UNSUPPORTED_SCHEMA);

  // A reserved flag bit set (CRC refreshed): malformed, not silently ignored.
  b = encode(goldenRecord(9));
  b[6] = 1;
  refreshCrc(&b);
  CHECK_STATUS(decodeCalibrationRecord(b.data(), b.size(), &d), CRS::MALFORMED);
}

void test_malformed_fields_refused_by_decoder() {
  g_case = "malformed fields";
  struct Case { size_t offset; uint8_t value; const char* what; };
  // Offsets into the V1 body: header 244 B, legs 4x18, joints 12x64.
  const size_t leg0 = 244;
  const size_t joint0 = 244 + 4 * 18;
  const Case cases[] = {
      {16 + 8 + 6 * 32 + 24, 2, "parameters_approved not a bool"},
      {leg0, 9, "leg index out of range"},
      {leg0 + 1, 99, "verdict out of range"},
      {leg0 + 6, 2, "diagnostics_accepted not a bool"},
      {joint0, 7, "joint leg out of range"},
      {joint0 + 1, 3, "joint kind out of range"},
      {joint0 + 2, 'x' ^ 0x80, "unit label charset"},
      {joint0 + 2 + 8 + 1, 0, "encoder direction zero"},
      {joint0 + 2 + 8 + 2, 5, "encoder direction source out of range"},
      {joint0 + 2 + 8 + 5, 9, "q0 estimator out of range"},
      {joint0 + 2 + 8 + 6, 9, "q0 state out of range"},
      {joint0 + 21, 9, "contact detection out of range"},
      {joint0 + 24, 2, "witness flag not a bool"},
  };
  for (const Case& c : cases) {
    std::vector<uint8_t> b = encode(goldenRecord(1));
    b[c.offset] = c.value;
    refreshCrc(&b);
    CalibrationRecord d;
    const CRS st = decodeCalibrationRecord(b.data(), b.size(), &d);
    ++g_checks;
    if (st != CRS::MALFORMED) {
      ++g_failures;
      std::printf("  FAIL [%s] %s: got %s, expected MALFORMED\n", g_case, c.what, toString(st));
    }
  }
}

void test_provenance_validation() {
  g_case = "provenance";
  CHECK_STATUS(validate(goldenRecord(1)), CRS::OK);

  // Each of the six digests altered, tag recomputed so the record is internally
  // coherent: it is a different geometry -> PROVENANCE_MISMATCH.
  for (int i = 0; i < 6; ++i) {
    CalibrationRecord r = goldenRecord(1);
    r.digest[i][31] ^= 0x01;
    actuator::GeometryProvenance p = actuator::geometry_data::kProvenance;
    static const char kDigits[] = "0123456789abcdef";
    char* dst[6] = {p.urdf_sha256, p.mesh_manifest_sha256, p.endpoint_semantic_sha256,
                    p.parking_semantic_sha256, p.safety_policy_semantic_sha256, p.allocation_sha256};
    for (int k = 0; k < 6; ++k) {
      for (int b = 0; b < 32; ++b) {
        dst[k][2 * b] = kDigits[r.digest[k][b] >> 4];
        dst[k][2 * b + 1] = kDigits[r.digest[k][b] & 15];
      }
      dst[k][64] = '\0';
    }
    r.geometry_tag = actuator::geometryProvenanceTag(p);
    ++g_checks;
    const CRS st = validate(r);
    if (st != CRS::PROVENANCE_MISMATCH) {
      ++g_failures;
      std::printf("  FAIL [%s] digest %d: got %s, expected PROVENANCE_MISMATCH\n", g_case, i, toString(st));
    }
  }

  // A digest altered but the stored tag left alone: the record contradicts itself.
  CalibrationRecord r = goldenRecord(1);
  r.digest[5][0] ^= 0x01;  // allocation digest
  CHECK_STATUS(validate(r), CRS::INCOHERENT);
  // A tag altered with the digests intact.
  r = goldenRecord(1);
  r.geometry_tag ^= 1;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);
  r.geometry_tag = actuator::kNoGeometryProvenance;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);

  CalibrationGeometryProfile unbound;
  CHECK_STATUS(validateCalibrationRecord(goldenRecord(1), unbound), CRS::PROFILE_UNBOUND);
}

void test_identity_validation() {
  g_case = "identity";
  struct Mut { const char* what; void (*apply)(CalibrationRecord*); CRS expect; };
  const Mut muts[] = {
      {"unit label of another joint", [](CalibrationRecord* r) { std::strcpy(r->joint[0].unit, "M99"); }, CRS::IDENTITY_MISMATCH},
      {"two units swapped",
       [](CalibrationRecord* r) { char t[8]; std::memcpy(t, r->joint[0].unit, 8); std::memcpy(r->joint[0].unit, r->joint[1].unit, 8); std::memcpy(r->joint[1].unit, t, 8); },
       CRS::IDENTITY_MISMATCH},
      {"bus id differs from the installation", [](CalibrationRecord* r) { r->joint[0].bus_id = 14; }, CRS::IDENTITY_MISMATCH},
      {"duplicate bus id", [](CalibrationRecord* r) { r->joint[1].bus_id = r->joint[0].bus_id; }, CRS::INCOHERENT},
      {"duplicate unit label", [](CalibrationRecord* r) { std::memcpy(r->joint[3].unit, r->joint[0].unit, 8); }, CRS::INCOHERENT},
      {"encoder direction flipped", [](CalibrationRecord* r) { r->joint[2].encoder_direction = static_cast<int8_t>(-r->joint[2].encoder_direction); }, CRS::IDENTITY_MISMATCH},
      {"encoder direction source flipped", [](CalibrationRecord* r) { r->joint[2].encoder_direction_source = r->joint[2].encoder_direction_source == 1 ? 2 : 1; }, CRS::IDENTITY_MISMATCH},
      {"duplicate joint (same leg+kind twice)", [](CalibrationRecord* r) { r->joint[4].leg = r->joint[3].leg; r->joint[4].joint = r->joint[3].joint; }, CRS::INCOHERENT},
      {"joint in the wrong position", [](CalibrationRecord* r) { CalibrationRecordJointV1 t = r->joint[0]; r->joint[0] = r->joint[1]; r->joint[1] = t; }, CRS::INCOHERENT},
      {"leg record labelled as another leg", [](CalibrationRecord* r) { r->leg[2].leg = 1; }, CRS::INCOHERENT},
  };
  for (const Mut& m : muts) {
    CalibrationRecord r = goldenRecord(1);
    m.apply(&r);
    ++g_checks;
    const CRS st = validate(r);
    if (st != m.expect) {
      ++g_failures;
      std::printf("  FAIL [%s] %s: got %s, expected %s\n", g_case, m.what, toString(st), toString(m.expect));
    }
  }
}

void test_q0_and_contact_validation() {
  g_case = "q0 and contacts";
  struct Mut { const char* what; void (*apply)(CalibrationRecord*); };
  const Mut muts[] = {
      {"q0 implausible (> 80 ticks from centre)", [](CalibrationRecord* r) { r->joint[0].q0_tick = 2200; }},
      {"q0 beyond the 12-bit domain", [](CalibrationRecord* r) { r->joint[0].q0_tick = 4096; }},
      {"q0 too few samples", [](CalibrationRecord* r) { r->joint[0].q0_sample_count = 8; }},
      {"q0 too many samples", [](CalibrationRecord* r) { r->joint[0].q0_sample_count = 33; }},
      {"q0 unstable", [](CalibrationRecord* r) { r->joint[0].q0_stability_spread_ticks = 17; }},
      {"q0 estimator NONE", [](CalibrationRecord* r) { r->joint[0].q0_estimator = static_cast<uint8_t>(Q0Estimator::NONE); }},
      {"q0 not promoted", [](CalibrationRecord* r) { r->joint[0].q0_state = static_cast<uint8_t>(EvidenceState::ACCEPTED); }},
      {"q0 historical origin", [](CalibrationRecord* r) { r->joint[0].q0_origin = static_cast<uint8_t>(CalibrationOrigin::HISTORICAL_REPLAY); }},
      {"contact missing (free motion)", [](CalibrationRecord* r) { r->joint[3].contact[0].detection = 0; }},
      {"contact not promoted", [](CalibrationRecord* r) { r->joint[3].contact[1].state = static_cast<uint8_t>(EvidenceState::ACCEPTED); }},
      {"contact not live", [](CalibrationRecord* r) { r->joint[3].contact[1].origin = static_cast<uint8_t>(CalibrationOrigin::HISTORICAL_REPLAY); }},
      {"witness not accepted", [](CalibrationRecord* r) { r->joint[3].contact[0].witness_accepted = 0; }},
      {"repeatability not |fine1-fine2|", [](CalibrationRecord* r) { r->joint[0].contact[0].repeatability_ticks = 4; }},
      {"repeatability above tolerance",
       [](CalibrationRecord* r) { r->joint[0].contact[0].fine_tick_2 = r->joint[0].contact[0].fine_tick_1 + 17; r->joint[0].contact[0].repeatability_ticks = 17; }},
      {"contact tick beyond 12 bits", [](CalibrationRecord* r) { r->joint[0].contact[0].fine_tick_1 = 5000; }},
  };
  for (const Mut& m : muts) {
    CalibrationRecord r = goldenRecord(1);
    m.apply(&r);
    ++g_checks;
    const CRS st = validate(r);
    if (st != CRS::INCOHERENT) {
      ++g_failures;
      std::printf("  FAIL [%s] %s: got %s, expected INCOHERENT\n", g_case, m.what, toString(st));
    }
  }
}

void test_diagnostics_are_recomputed() {
  g_case = "diagnostics recomputed";
  // Each stored diagnostic field, nudged by one, contradicts the contacts + q0.
  for (int field = 0; field < 8; ++field) {
    CalibrationRecord r = goldenRecord(1);
    CalibrationRecordDiagnosticsV1& d = r.joint[4].diagnostics;
    uint16_t* f[8] = {&d.min_contact_tick, &d.max_contact_tick, &d.expected_span_ticks, &d.measured_span_ticks,
                      &d.scale_permille, &d.affine_zero_tick, &d.affine_shift_from_q0_ticks,
                      &d.fixed_endpoint_disagreement_ticks};
    *f[field] = static_cast<uint16_t>(*f[field] + 1);
    ++g_checks;
    const CRS st = validate(r);
    if (st != CRS::INCOHERENT) {
      ++g_failures;
      std::printf("  FAIL [%s] diagnostics field %d: got %s\n", g_case, field, toString(st));
    }
  }
  CalibrationRecord r = goldenRecord(1);
  r.joint[4].diagnostics.accepted = 0;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);
  r = goldenRecord(1);
  r.joint[4].diagnostics.ordered = 0;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);
  r = goldenRecord(1);
  r.joint[4].diagnostics.evaluated = 0;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);

  // Contacts altered while the stored diagnostics keep the old numbers.
  r = goldenRecord(1);
  r.joint[7].contact[0].fine_tick_1 += 6;
  r.joint[7].contact[0].fine_tick_2 += 6;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);
  // q0 altered (still plausible): the affine shift no longer matches.
  r = goldenRecord(1);
  r.joint[7].q0_tick += 5;
  CHECK_STATUS(validate(r), CRS::INCOHERENT);
}

void test_leg_level_validation() {
  g_case = "leg level";
  struct Mut { const char* what; void (*apply)(CalibrationRecord*); CRS expect; };
  const Mut muts[] = {
      {"parameters_approved = 1", [](CalibrationRecord* r) { r->parameters_approved = 1; }, CRS::FORBIDDEN_AUTHORIZATION},
      {"calibration_accepted = 0", [](CalibrationRecord* r) { r->calibration_accepted = 0; }, CRS::INCOHERENT},
      {"verdict claims operational envelope accepted",
       [](CalibrationRecord* r) { r->leg[1].verdict = static_cast<uint8_t>(FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED); },
       CRS::FORBIDDEN_AUTHORIZATION},
      {"verdict FAILED", [](CalibrationRecord* r) { r->leg[1].verdict = static_cast<uint8_t>(FullLegVerdict::FAILED); }, CRS::INCOHERENT},
      {"5/6 contacts accepted", [](CalibrationRecord* r) { r->leg[0].contacts_accepted = 5; }, CRS::INCOHERENT},
      {"5/6 contacts measured", [](CalibrationRecord* r) { r->leg[0].contacts_measured = 5; }, CRS::INCOHERENT},
      {"diagnostics_accepted = 0", [](CalibrationRecord* r) { r->leg[0].diagnostics_accepted = 0; }, CRS::INCOHERENT},
      {"session not completed", [](CalibrationRecord* r) { r->leg[0].session_completed = 0; }, CRS::INCOHERENT},
      {"permit not revoked", [](CalibrationRecord* r) { r->leg[0].permit_revoked = 0; }, CRS::INCOHERENT},
      {"authority not released", [](CalibrationRecord* r) { r->leg[0].authority_released = 0; }, CRS::INCOHERENT},
      {"zero attempts", [](CalibrationRecord* r) { r->leg[0].attempts = 0; }, CRS::INCOHERENT},
      {"park fields without park", [](CalibrationRecord* r) { r->leg[2].park_bus_id = 32; }, CRS::INCOHERENT},
      {"park bus differs from the joint's", [](CalibrationRecord* r) { r->leg[0].park_bus_id = 41; }, CRS::INCOHERENT},
      {"park leg = own leg", [](CalibrationRecord* r) { r->leg[0].park_leg = 0; }, CRS::INCOHERENT},
      {"park target out of range", [](CalibrationRecord* r) { r->leg[0].park_target_urad = 4000000; }, CRS::INCOHERENT},
      {"generation 0", [](CalibrationRecord* r) { r->generation = 0; }, CRS::MALFORMED},
      {"bool out of range", [](CalibrationRecord* r) { r->leg[0].session_completed = 2; }, CRS::MALFORMED},
      {"unit charset", [](CalibrationRecord* r) { r->joint[0].unit[0] = '!'; }, CRS::MALFORMED},
      {"unit empty", [](CalibrationRecord* r) { std::memset(r->joint[0].unit, 0, 8); }, CRS::MALFORMED},
      {"build id with a control character", [](CalibrationRecord* r) { r->build_id[0] = 1; }, CRS::MALFORMED},
  };
  for (const Mut& m : muts) {
    CalibrationRecord r = goldenRecord(1);
    m.apply(&r);
    ++g_checks;
    const CRS st = validate(r);
    if (st != m.expect) {
      ++g_failures;
      std::printf("  FAIL [%s] %s: got %s, expected %s\n", g_case, m.what, toString(st), toString(m.expect));
    }
  }
  // build id is informational: any printable content is fine, never compared.
  CalibrationRecord r = goldenRecord(1);
  std::strcpy(r.build_id, "other-build");
  CHECK_STATUS(validate(r), CRS::OK);
}

void test_nothing_persisted_authorizes_motion() {
  g_case = "no authority in the record";
  // Structural: the record type carries no session id, permit, token, authority
  // generation, limit or envelope. This is a compile-time inventory of its
  // members plus a check that a decoded record validates without any context.
  CHECK(!kFullLegOperationalParametersApproved);
  const CalibrationRecord r = goldenRecord(1);
  CHECK_EQ(r.parameters_approved, 0);
  const std::vector<uint8_t> b = encode(r);
  // A record claiming approved parameters cannot even be encoded-and-accepted.
  CalibrationRecord claim = r;
  claim.parameters_approved = 1;
  const std::vector<uint8_t> cb = encode(claim);
  CalibrationRecord d;
  CHECK_STATUS(decodeCalibrationRecord(cb.data(), cb.size(), &d), CRS::OK);   // well-formed bytes...
  CHECK_STATUS(validate(d), CRS::FORBIDDEN_AUTHORIZATION);                    // ...never accepted
}

// --- builder ----------------------------------------------------------------------

FullLegEvidenceStore syntheticEvidence(const CalibrationRecord& g) {
  FullLegEvidenceStore store;
  store.reset();
  const actuator::GeometryProvenanceTag tag = g.geometry_tag;
  for (uint8_t l = 0; l < 4; ++l) {
    FullLegRecord rec;
    rec.leg = static_cast<Leg>(l);
    rec.present = true;
    rec.attempts = g.leg[l].attempts;
    rec.session_id = l + 1;
    rec.geometry = tag;
    rec.has_rear_park = g.leg[l].has_rear_park != 0;
    if (rec.has_rear_park) {
      rec.park_leg = static_cast<Leg>(g.leg[l].park_leg);
      rec.park_joint = static_cast<JointKind>(g.leg[l].park_joint);
      rec.park_bus_id = g.leg[l].park_bus_id;
      rec.park_target_urad = g.leg[l].park_target_urad;
    }
    rec.contacts_measured = 6;
    rec.contacts_accepted = 6;
    rec.diagnostics_accepted = true;
    rec.parameters_approved = false;
    rec.contact_margin_ticks = g.contact_margin_ticks;
    rec.session_completed = true;
    rec.permit_revoked = true;
    rec.authority_released = true;
    rec.hardware_contact_calibrated = true;
    rec.verdict = FullLegVerdict::HARDWARE_CONTACT_CALIBRATED;
    for (uint8_t k = 0; k < 3; ++k) {
      const CalibrationRecordJointV1& gj = g.joint[l * 3 + k];
      FullLegJointRecord& j = rec.joints[k];
      j.identity.leg = static_cast<Leg>(l);
      j.identity.joint = static_cast<JointKind>(k);
      setPhysicalUnit(&j.identity, gj.unit);
      j.bus_id = gj.bus_id;
      j.q0_present = true;
      j.q0_state = EvidenceState::PROMOTED;
      j.q0_origin = CalibrationOrigin::LIVE_SESSION;
      j.q0_geometry = tag;
      j.q0_tick = gj.q0_tick;
      for (uint8_t s = 0; s < 2; ++s) {
        ContactEvidence& e = j.contact[s];
        e.key.leg = static_cast<Leg>(l);
        e.key.joint = static_cast<JointKind>(k);
        e.key.side = static_cast<ContactSide>(s);
        e.state = EvidenceState::PROMOTED;
        e.origin = CalibrationOrigin::LIVE_SESSION;
        e.detection = ContactState::CONTACT_CONFIRMED;
        e.witness = makeContactWitness(0, gj.contact[s].repeatability_ticks, kFullLegRepeatabilityToleranceTicks);
        e.coarse_tick = gj.contact[s].scout_tick;
        e.fine_tick_1 = gj.contact[s].fine_tick_1;
        e.fine_tick_2 = gj.contact[s].fine_tick_2;
        e.repeatability_ticks = gj.contact[s].repeatability_ticks;
        e.has_measurement = true;
        j.contact_recorded[s] = true;
      }
      j.diagnostics.evaluated = true;
      j.diagnostics.ordered = true;
      j.diagnostics.accepted = true;
      j.diagnostics.min_contact_tick = gj.diagnostics.min_contact_tick;
      j.diagnostics.max_contact_tick = gj.diagnostics.max_contact_tick;
      j.diagnostics.expected_span_ticks = gj.diagnostics.expected_span_ticks;
      j.diagnostics.measured_span_ticks = gj.diagnostics.measured_span_ticks;
      j.diagnostics.scale_permille = gj.diagnostics.scale_permille;
      j.diagnostics.affine_zero_tick = gj.diagnostics.affine_zero_tick;
      j.diagnostics.affine_shift_from_q0_ticks = gj.diagnostics.affine_shift_from_q0_ticks;
      j.diagnostics.fixed_endpoint_disagreement_ticks = gj.diagnostics.fixed_endpoint_disagreement_ticks;
    }
    store.put(rec);
  }
  return store;
}

CalibrationRecordSource sourceFor(const FullLegEvidenceStore* store, const CalibrationGeometryProfile* profile) {
  CalibrationRecordSource s;
  s.evidence = store;
  s.profile = profile;
  s.build_id = "golden-test";
  for (uint8_t i = 0; i < 12; ++i) {
    s.q0_capture[i].estimator = Q0Estimator::MANUAL_ZERO_POSE;
    s.q0_capture[i].sample_count = 9;
    s.q0_capture[i].stability_spread_ticks = 0;
  }
  return s;
}

void test_builder() {
  g_case = "builder";
  const CalibrationGeometryProfile profile = boundProfile();
  const CalibrationRecord g = goldenRecord(1);
  const FullLegEvidenceStore store = syntheticEvidence(g);
  CalibrationRecordSource src = sourceFor(&store, &profile);

  CalibrationRecord built;
  CHECK_STATUS(buildCalibrationRecord(src, &built), CRS::OK);
  CHECK_EQ(built.generation, 0);  // the store assigns it
  built.generation = 1;
  CHECK_STATUS(validate(built), CRS::OK);
  CHECK(encode(built) == encode(g));  // byte-identical to the golden vector

  // q0 capture facts are an explicit input: a caller that does not state them
  // gets a record that does not validate, never an invented default.
  CalibrationRecordSource blank = sourceFor(&store, &profile);
  for (uint8_t i = 0; i < 12; ++i) blank.q0_capture[i] = CalibrationQ0CaptureMetadata{};
  CHECK_STATUS(buildCalibrationRecord(blank, &built), CRS::OK);
  built.generation = 1;
  CHECK_STATUS(validate(built), CRS::INCOHERENT);

  // Source refusals.
  CHECK_STATUS(buildCalibrationRecord(src, nullptr), CRS::SOURCE_INCOMPLETE);
  CalibrationRecordSource none = sourceFor(nullptr, &profile);
  CHECK_STATUS(buildCalibrationRecord(none, &built), CRS::SOURCE_INCOMPLETE);
  CalibrationGeometryProfile unbound;
  CalibrationRecordSource unb = sourceFor(&store, &unbound);
  CHECK_STATUS(buildCalibrationRecord(unb, &built), CRS::PROFILE_UNBOUND);

  FullLegEvidenceStore empty;
  empty.reset();
  CalibrationRecordSource es = sourceFor(&empty, &profile);
  CHECK_STATUS(buildCalibrationRecord(es, &built), CRS::SOURCE_INCOMPLETE);

  // 23/24: one leg missing.
  FullLegEvidenceStore three;
  three.reset();
  for (uint8_t l = 0; l < 3; ++l) three.put(*store.find(static_cast<Leg>(l)));
  CalibrationRecordSource ts = sourceFor(&three, &profile);
  CHECK_STATUS(buildCalibrationRecord(ts, &built), CRS::SOURCE_INCOMPLETE);

  // Foreign geometry on one leg.
  FullLegEvidenceStore foreign = store;
  FullLegRecord leg = *store.find(Leg::RH);
  leg.geometry ^= 1;
  foreign.put(leg);
  CalibrationRecordSource fs = sourceFor(&foreign, &profile);
  CHECK_STATUS(buildCalibrationRecord(fs, &built), CRS::SOURCE_GEOMETRY_MISMATCH);

  // A leg that claims approved parameters.
  FullLegEvidenceStore approved = store;
  leg = *store.find(Leg::LF);
  leg.parameters_approved = true;
  approved.put(leg);
  CalibrationRecordSource as = sourceFor(&approved, &profile);
  CHECK_STATUS(buildCalibrationRecord(as, &built), CRS::FORBIDDEN_AUTHORIZATION);

  // A joint whose contact was never recorded.
  FullLegEvidenceStore partial = store;
  leg = *store.find(Leg::LH);
  leg.joints[1].contact_recorded[0] = false;
  partial.put(leg);
  CalibrationRecordSource ps = sourceFor(&partial, &profile);
  CHECK_STATUS(buildCalibrationRecord(ps, &built), CRS::SOURCE_INCOMPLETE);

  // A leg that did not finalize cleanly.
  FullLegEvidenceStore failed = store;
  leg = *store.find(Leg::LH);
  leg.failure = FullLegFinalizeFailure::CONTACTS_INCOMPLETE;
  failed.put(leg);
  CalibrationRecordSource fl = sourceFor(&failed, &profile);
  CHECK_STATUS(buildCalibrationRecord(fl, &built), CRS::SOURCE_INCOMPLETE);

  // A leg that is not contact-calibrated builds, but the record is not accepted.
  FullLegEvidenceStore notcal = store;
  leg = *store.find(Leg::LH);
  leg.verdict = FullLegVerdict::FAILED;
  notcal.put(leg);
  CalibrationRecordSource nc = sourceFor(&notcal, &profile);
  CHECK_STATUS(buildCalibrationRecord(nc, &built), CRS::OK);
  CHECK_EQ(built.calibration_accepted, 0);
  built.generation = 1;
  CHECK_STATUS(validate(built), CRS::INCOHERENT);

  // Bus id in the evidence differs from the installed profile.
  FullLegEvidenceStore wrongbus = store;
  leg = *store.find(Leg::LF);
  leg.joints[0].bus_id = 14;
  wrongbus.put(leg);
  CalibrationRecordSource wb = sourceFor(&wrongbus, &profile);
  CHECK_STATUS(buildCalibrationRecord(wb, &built), CRS::IDENTITY_MISMATCH);

  // build id is truncated, sanitised, never overflows.
  CalibrationRecordSource longid = sourceFor(&store, &profile);
  longid.build_id = "an-extremely-long-build-identifier-that-cannot-fit";
  CHECK_STATUS(buildCalibrationRecord(longid, &built), CRS::OK);
  CHECK_EQ(built.build_id[kRecordBuildIdBytes - 1], 0);
  built.generation = 1;
  CHECK_STATUS(validate(built), CRS::OK);
}

void test_status_strings_and_classes() {
  g_case = "status classes";
  const CRS corrupt[] = {CRS::TRUNCATED, CRS::BAD_MAGIC, CRS::BAD_LENGTH, CRS::BAD_CRC, CRS::MALFORMED,
                         CRS::INCOHERENT, CRS::FORBIDDEN_AUTHORIZATION};
  for (CRS s : corrupt) CHECK(classifyCalibrationRecordStatus(s) == CalibrationRecordClass::CORRUPT);
  const CRS foreign[] = {CRS::UNSUPPORTED_SCHEMA, CRS::PROVENANCE_MISMATCH, CRS::IDENTITY_MISMATCH, CRS::PROFILE_UNBOUND};
  for (CRS s : foreign) CHECK(classifyCalibrationRecordStatus(s) == CalibrationRecordClass::INCOMPATIBLE);
  CHECK(classifyCalibrationRecordStatus(CRS::OK) == CalibrationRecordClass::OK);
  CHECK(std::strcmp(toString(CRS::BAD_CRC), "BAD_CRC") == 0);
}

}  // namespace

int main() {
  test_crc32();
  test_golden_vector();
  test_round_trip();
  test_codec_rejects_damage();
  test_incompatible_schema();
  test_malformed_fields_refused_by_decoder();
  test_provenance_validation();
  test_identity_validation();
  test_q0_and_contact_validation();
  test_diagnostics_are_recomputed();
  test_leg_level_validation();
  test_nothing_persisted_authorizes_motion();
  test_builder();
  test_status_strings_and_classes();
  std::printf("test_calibration_record: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
