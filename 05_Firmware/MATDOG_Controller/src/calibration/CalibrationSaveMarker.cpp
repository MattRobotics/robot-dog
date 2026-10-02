#include "CalibrationSaveMarker.h"

#include "CalibrationRecord.h"  // calibrationCrc32

namespace matdog {
namespace calibration {

namespace {

constexpr size_t kOffMagic = 0;
constexpr size_t kOffSchema = 4;
constexpr size_t kOffFlags = 6;
constexpr size_t kOffLength = 8;
constexpr size_t kOffCompleted = 12;
constexpr size_t kOffBegun = 16;
constexpr size_t kOffState = 20;
constexpr size_t kOffReserved = 21;  // 3 bytes
constexpr size_t kOffCrc = 24;
static_assert(kOffCrc + kSaveMarkerTrailerBytes == kSaveMarkerV1Bytes, "marker layout drifted");

void putU16(uint8_t* p, size_t off, uint16_t v) {
  p[off] = static_cast<uint8_t>(v);
  p[off + 1] = static_cast<uint8_t>(v >> 8);
}
void putU32(uint8_t* p, size_t off, uint32_t v) {
  for (size_t i = 0; i < 4; ++i) p[off + i] = static_cast<uint8_t>(v >> (8 * i));
}
uint16_t getU16(const uint8_t* p, size_t off) {
  return static_cast<uint16_t>(p[off] | (p[off + 1] << 8));
}
uint32_t getU32(const uint8_t* p, size_t off) {
  return static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24);
}

}  // namespace

const char* toString(SaveMarkerStatus status) {
  switch (status) {
    case SaveMarkerStatus::OK:                 return "OK";
    case SaveMarkerStatus::TRUNCATED:          return "TRUNCATED";
    case SaveMarkerStatus::BAD_MAGIC:          return "BAD_MAGIC";
    case SaveMarkerStatus::BAD_LENGTH:         return "BAD_LENGTH";
    case SaveMarkerStatus::BAD_CRC:            return "BAD_CRC";
    case SaveMarkerStatus::UNSUPPORTED_SCHEMA: return "UNSUPPORTED_SCHEMA";
    case SaveMarkerStatus::MALFORMED:          return "MALFORMED";
    case SaveMarkerStatus::INCOHERENT:         return "INCOHERENT";
    case SaveMarkerStatus::BUFFER_TOO_SMALL:   return "BUFFER_TOO_SMALL";
  }
  return "UNKNOWN";
}

const char* toString(SaveMarkerState state) {
  switch (state) {
    case SaveMarkerState::COMPLETED: return "COMPLETED";
    case SaveMarkerState::PENDING:   return "PENDING";
  }
  return "UNKNOWN";
}

SaveMarkerStatus validateSaveMarker(const SaveMarker& m) {
  switch (m.state) {
    case SaveMarkerState::COMPLETED:
      if (m.begun_generation < m.completed_generation) return SaveMarkerStatus::INCOHERENT;
      if (m.completed_generation == 0 && m.begun_generation == 0) return SaveMarkerStatus::INCOHERENT;
      return SaveMarkerStatus::OK;
    case SaveMarkerState::PENDING:
      return m.begun_generation > m.completed_generation ? SaveMarkerStatus::OK
                                                         : SaveMarkerStatus::INCOHERENT;
  }
  return SaveMarkerStatus::MALFORMED;
}

SaveMarkerStatus encodeSaveMarker(const SaveMarker& marker, uint8_t* out, size_t capacity,
                                  size_t* written) {
  if (out == nullptr || written == nullptr) return SaveMarkerStatus::BUFFER_TOO_SMALL;
  *written = 0;
  if (capacity < kSaveMarkerV1Bytes) return SaveMarkerStatus::BUFFER_TOO_SMALL;
  const SaveMarkerStatus v = validateSaveMarker(marker);
  if (v != SaveMarkerStatus::OK) return v;

  for (size_t i = 0; i < kSaveMarkerV1Bytes; ++i) out[i] = 0;
  putU32(out, kOffMagic, kSaveMarkerMagic);
  putU16(out, kOffSchema, kSaveMarkerSchemaV1);
  putU16(out, kOffFlags, 0);
  putU32(out, kOffLength, static_cast<uint32_t>(kSaveMarkerV1Bytes));
  putU32(out, kOffCompleted, marker.completed_generation);
  putU32(out, kOffBegun, marker.begun_generation);
  out[kOffState] = static_cast<uint8_t>(marker.state);
  putU32(out, kOffCrc, calibrationCrc32(out, kOffCrc));
  *written = kSaveMarkerV1Bytes;
  return SaveMarkerStatus::OK;
}

SaveMarkerStatus decodeSaveMarker(const uint8_t* data, size_t length, SaveMarker* out) {
  if (data == nullptr || out == nullptr) return SaveMarkerStatus::TRUNCATED;
  if (length < kSaveMarkerEnvelopePrefixBytes + kSaveMarkerTrailerBytes) {
    return SaveMarkerStatus::TRUNCATED;
  }
  if (getU32(data, kOffMagic) != kSaveMarkerMagic) return SaveMarkerStatus::BAD_MAGIC;
  if (getU32(data, kOffLength) != length) return SaveMarkerStatus::BAD_LENGTH;
  const size_t crc_off = length - kSaveMarkerTrailerBytes;
  if (getU32(data, crc_off) != calibrationCrc32(data, crc_off)) return SaveMarkerStatus::BAD_CRC;
  if (getU16(data, kOffSchema) != kSaveMarkerSchemaV1) return SaveMarkerStatus::UNSUPPORTED_SCHEMA;
  if (length != kSaveMarkerV1Bytes) return SaveMarkerStatus::BAD_LENGTH;
  if (getU16(data, kOffFlags) != 0) return SaveMarkerStatus::MALFORMED;
  for (size_t i = 0; i < 3; ++i) {
    if (data[kOffReserved + i] != 0) return SaveMarkerStatus::MALFORMED;
  }
  const uint8_t state = data[kOffState];
  if (state != static_cast<uint8_t>(SaveMarkerState::COMPLETED) &&
      state != static_cast<uint8_t>(SaveMarkerState::PENDING)) {
    return SaveMarkerStatus::MALFORMED;
  }
  SaveMarker m;
  m.state = static_cast<SaveMarkerState>(state);
  m.completed_generation = getU32(data, kOffCompleted);
  m.begun_generation = getU32(data, kOffBegun);
  const SaveMarkerStatus v = validateSaveMarker(m);
  if (v != SaveMarkerStatus::OK) return v;
  *out = m;
  return SaveMarkerStatus::OK;
}

}  // namespace calibration
}  // namespace matdog
