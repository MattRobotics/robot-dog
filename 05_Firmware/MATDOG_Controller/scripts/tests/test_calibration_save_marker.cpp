// Offline tests for the SAVE marker codec (src/calibration/CalibrationSaveMarker.*):
// explicit field-by-field little-endian serialization, CRC, schema compatibility
// and the state/generation invariants. Pure: no storage, no hardware.

#include <cstdio>
#include <cstring>
#include <vector>

#include "../../src/calibration/CalibrationRecord.h"
#include "../../src/calibration/CalibrationSaveMarker.h"

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

namespace {

SaveMarker mk(SaveMarkerState s, uint32_t c, uint32_t b) {
  SaveMarker m;
  m.state = s;
  m.completed_generation = c;
  m.begun_generation = b;
  return m;
}

std::vector<uint8_t> enc(const SaveMarker& m) {
  std::vector<uint8_t> b(kSaveMarkerV1Bytes);
  size_t n = 0;
  CHECK(encodeSaveMarker(m, b.data(), b.size(), &n) == SaveMarkerStatus::OK);
  CHECK(n == kSaveMarkerV1Bytes);
  return b;
}

void refreshCrc(std::vector<uint8_t>* b) {
  const uint32_t crc = calibrationCrc32(b->data(), b->size() - 4);
  for (int i = 0; i < 4; ++i) (*b)[b->size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
}

void test_layout_is_explicit() {
  g_case = "layout";
  CHECK(kSaveMarkerV1Bytes == 28);
  const std::vector<uint8_t> b = enc(mk(SaveMarkerState::PENDING, 0x01020304u, 0x0A0B0C0Du));
  // magic "MDMK", schema 1, flags 0, length 28, completed, begun, state 2, reserved
  const uint8_t expected_prefix[24] = {0x4D, 0x44, 0x4D, 0x4B, 0x01, 0x00, 0x00, 0x00,
                                       0x1C, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01,
                                       0x0D, 0x0C, 0x0B, 0x0A, 0x02, 0x00, 0x00, 0x00};
  CHECK(std::memcmp(b.data(), expected_prefix, 24) == 0);
  const uint32_t crc = calibrationCrc32(b.data(), 24);
  CHECK(b[24] == (crc & 0xFF) && b[25] == ((crc >> 8) & 0xFF) && b[26] == ((crc >> 16) & 0xFF) &&
        b[27] == ((crc >> 24) & 0xFF));
  // Separate from the 1088-byte record format.
  CHECK(kSaveMarkerV1Bytes != kCalibrationRecordV1EncodedBytes);
  CHECK(kSaveMarkerV1Bytes <= kSaveMarkerScratchBytes);
}

void test_round_trip() {
  g_case = "round trip";
  const SaveMarker cases[] = {
      mk(SaveMarkerState::COMPLETED, 1, 1),         mk(SaveMarkerState::COMPLETED, 0, 1),
      mk(SaveMarkerState::COMPLETED, 5, 9),         mk(SaveMarkerState::PENDING, 0, 1),
      mk(SaveMarkerState::PENDING, 4, 5),           mk(SaveMarkerState::COMPLETED, 0xFFFFFFFFu, 0xFFFFFFFFu),
      mk(SaveMarkerState::PENDING, 0xFFFFFFFEu, 0xFFFFFFFFu)};
  for (const SaveMarker& m : cases) {
    const std::vector<uint8_t> b = enc(m);
    SaveMarker back;
    CHECK(decodeSaveMarker(b.data(), b.size(), &back) == SaveMarkerStatus::OK);
    CHECK(back == m);
  }
}

void test_invariants() {
  g_case = "invariants";
  // Invalid markers are never encoded ...
  const SaveMarker bad[] = {
      mk(SaveMarkerState::COMPLETED, 0, 0),  mk(SaveMarkerState::COMPLETED, 3, 2),
      mk(SaveMarkerState::PENDING, 0, 0),    mk(SaveMarkerState::PENDING, 3, 3),
      mk(SaveMarkerState::PENDING, 4, 3),    mk(static_cast<SaveMarkerState>(0), 1, 1),
      mk(static_cast<SaveMarkerState>(3), 1, 2)};
  uint8_t buf[64];
  for (const SaveMarker& m : bad) {
    size_t n = 99;
    const SaveMarkerStatus st = encodeSaveMarker(m, buf, sizeof(buf), &n);
    CHECK(st == SaveMarkerStatus::INCOHERENT || st == SaveMarkerStatus::MALFORMED);
    CHECK(n == 0);
    CHECK(validateSaveMarker(m) != SaveMarkerStatus::OK);
  }
  // ... and never decoded: forge a blob carrying one with a valid CRC.
  for (const SaveMarker& m : bad) {
    std::vector<uint8_t> b = enc(mk(SaveMarkerState::PENDING, 1, 2));
    b[12] = static_cast<uint8_t>(m.completed_generation);
    b[16] = static_cast<uint8_t>(m.begun_generation);
    b[20] = static_cast<uint8_t>(m.state);
    refreshCrc(&b);
    SaveMarker out = mk(SaveMarkerState::COMPLETED, 77, 77);
    const SaveMarkerStatus st = decodeSaveMarker(b.data(), b.size(), &out);
    CHECK(st == SaveMarkerStatus::INCOHERENT || st == SaveMarkerStatus::MALFORMED);
    CHECK(out == mk(SaveMarkerState::COMPLETED, 77, 77));  // *out untouched on failure
  }
}

void test_every_corruption_is_detected() {
  g_case = "corruption";
  const std::vector<uint8_t> good = enc(mk(SaveMarkerState::PENDING, 6, 7));
  SaveMarker out;
  for (size_t byte = 0; byte < good.size(); ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      std::vector<uint8_t> b = good;
      b[byte] ^= static_cast<uint8_t>(1u << bit);
      CHECK(decodeSaveMarker(b.data(), b.size(), &out) != SaveMarkerStatus::OK);
    }
  }
  for (size_t len = 0; len < good.size(); ++len) {  // every truncation
    CHECK(decodeSaveMarker(good.data(), len, &out) != SaveMarkerStatus::OK);
  }
  std::vector<uint8_t> longer = good;  // trailing garbage
  longer.push_back(0);
  CHECK(decodeSaveMarker(longer.data(), longer.size(), &out) != SaveMarkerStatus::OK);

  std::vector<uint8_t> magic = good;
  magic[0] ^= 1;
  CHECK(decodeSaveMarker(magic.data(), magic.size(), &out) == SaveMarkerStatus::BAD_MAGIC);
  std::vector<uint8_t> crc = good;
  crc[27] ^= 1;
  CHECK(decodeSaveMarker(crc.data(), crc.size(), &out) == SaveMarkerStatus::BAD_CRC);
  CHECK(decodeSaveMarker(good.data(), 5, &out) == SaveMarkerStatus::TRUNCATED);
  CHECK(decodeSaveMarker(nullptr, 28, &out) == SaveMarkerStatus::TRUNCATED);
  CHECK(decodeSaveMarker(good.data(), good.size(), nullptr) == SaveMarkerStatus::TRUNCATED);
  // A record-sized blob is not a marker.
  std::vector<uint8_t> big(kCalibrationRecordV1EncodedBytes, 0);
  CHECK(decodeSaveMarker(big.data(), big.size(), &out) != SaveMarkerStatus::OK);
  // Erased flash reads as 0xFF.
  std::vector<uint8_t> ff(28, 0xFF);
  CHECK(decodeSaveMarker(ff.data(), ff.size(), &out) != SaveMarkerStatus::OK);
}

void test_schema_compatibility() {
  g_case = "schema";
  SaveMarker out;
  // Intact envelope, other schema: foreign, not damage.
  std::vector<uint8_t> b = enc(mk(SaveMarkerState::COMPLETED, 2, 2));
  b[4] = 2;
  refreshCrc(&b);
  SaveMarkerStatus st = decodeSaveMarker(b.data(), b.size(), &out);
  CHECK(st == SaveMarkerStatus::UNSUPPORTED_SCHEMA);
  CHECK(isForeignSaveMarker(st));
  // A longer future schema with a consistent envelope is also foreign.
  std::vector<uint8_t> f(40, 0);
  f[0] = 0x4D; f[1] = 0x44; f[2] = 0x4D; f[3] = 0x4B;
  f[4] = 3;
  f[8] = 40;
  refreshCrc(&f);
  st = decodeSaveMarker(f.data(), f.size(), &out);
  CHECK(st == SaveMarkerStatus::UNSUPPORTED_SCHEMA);
  // Damage is not foreign.
  b = enc(mk(SaveMarkerState::COMPLETED, 2, 2));
  b[13] ^= 1;
  CHECK(!isForeignSaveMarker(decodeSaveMarker(b.data(), b.size(), &out)));
  // V1 with flags / reserved set: refused.
  b = enc(mk(SaveMarkerState::COMPLETED, 2, 2));
  b[6] = 1;
  refreshCrc(&b);
  CHECK(decodeSaveMarker(b.data(), b.size(), &out) == SaveMarkerStatus::MALFORMED);
  b = enc(mk(SaveMarkerState::COMPLETED, 2, 2));
  b[22] = 1;
  refreshCrc(&b);
  CHECK(decodeSaveMarker(b.data(), b.size(), &out) == SaveMarkerStatus::MALFORMED);
  // V1 of the wrong length (header consistent).
  std::vector<uint8_t> w(32, 0);
  std::memcpy(w.data(), enc(mk(SaveMarkerState::COMPLETED, 2, 2)).data(), 24);
  w[8] = 32;
  refreshCrc(&w);
  CHECK(decodeSaveMarker(w.data(), w.size(), &out) == SaveMarkerStatus::BAD_LENGTH);
}

void test_encoder_arguments() {
  g_case = "encoder arguments";
  uint8_t small[27];
  size_t n = 5;
  CHECK(encodeSaveMarker(mk(SaveMarkerState::COMPLETED, 1, 1), small, sizeof(small), &n) ==
        SaveMarkerStatus::BUFFER_TOO_SMALL);
  CHECK(n == 0);
  CHECK(encodeSaveMarker(mk(SaveMarkerState::COMPLETED, 1, 1), nullptr, 28, &n) == SaveMarkerStatus::BUFFER_TOO_SMALL);
  uint8_t ok[28];
  CHECK(encodeSaveMarker(mk(SaveMarkerState::COMPLETED, 1, 1), ok, sizeof(ok), nullptr) ==
        SaveMarkerStatus::BUFFER_TOO_SMALL);
  CHECK(std::strcmp(toString(SaveMarkerStatus::BAD_CRC), "BAD_CRC") == 0);
  CHECK(std::strcmp(toString(SaveMarkerState::PENDING), "PENDING") == 0);
}

}  // namespace

int main() {
  test_layout_is_explicit();
  test_round_trip();
  test_invariants();
  test_every_corruption_is_detected();
  test_schema_compatibility();
  test_encoder_arguments();
  std::printf("calibration save marker: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
