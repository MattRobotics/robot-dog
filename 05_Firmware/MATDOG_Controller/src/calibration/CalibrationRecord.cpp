#include "CalibrationRecord.h"

#include <string.h>

#include "../actuator/CalibrationQ0Promotion.h"
#include "FullLegCalibrationExecutor.h"
#include "FullLegCalibrationPlan.h"

namespace matdog {
namespace calibration {

namespace {

// --- wire layout (bytes) -------------------------------------------------------
constexpr size_t kHeaderBytes = 4 + 2 + 2 + 4 + 4 + 8 +
                                kRecordSha256Digests * kRecordSha256Bytes + kRecordBuildIdBytes +
                                1 + 1 + 2;                         // 244
constexpr size_t kLegBytes = 1 + 1 + 2 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 4;
constexpr size_t kContactBytes = 4 + 2 * 4;                          // 12
constexpr size_t kDiagBytes = 3 + 8 * 2;                             // 19
constexpr size_t kJointBytes = 2 + kPhysicalUnitLabelBytes + 3 + 2 + 4 + 2 +
                               kContactSideCount * kContactBytes + kDiagBytes;  // 64

// Leg: leg, verdict, attempts(2), measured, accepted, diag, session, permit,
// authority, has_park, park_leg, park_joint, park_bus, park_urad(4) = 18 bytes.
static_assert(kLegBytes == 18, "leg record layout drifted");
static_assert(kJointBytes == 64, "joint record layout drifted");

constexpr size_t kBodyBytes = kHeaderBytes + kRecordLegCount * kLegBytes +
                              kRecordJointCount * kJointBytes;
constexpr size_t kTotalBytes = kBodyBytes + kCalibrationRecordTrailerBytes;
static_assert(kTotalBytes == kCalibrationRecordV1EncodedBytes,
              "CalibrationRecord V1 encoded size drifted - this is a schema change");

constexpr size_t kOffTotalLength = 8;
constexpr size_t kOffGeneration = 12;

// --- little-endian writer / reader ------------------------------------------------
class Writer {
 public:
  Writer(uint8_t* out, size_t capacity) : p_(out), cap_(capacity) {}
  void u8(uint8_t v) { put(&v, 1); }
  void u16(uint16_t v) {
    const uint8_t b[2] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
    put(b, 2);
  }
  void u32(uint32_t v) {
    const uint8_t b[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
                          static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
    put(b, 4);
  }
  void u64(uint64_t v) {
    u32(static_cast<uint32_t>(v));
    u32(static_cast<uint32_t>(v >> 32));
  }
  void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
  void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
  void bytes(const void* data, size_t n) { put(static_cast<const uint8_t*>(data), n); }
  size_t pos() const { return pos_; }
  bool ok() const { return ok_; }

 private:
  void put(const uint8_t* data, size_t n) {
    if (!ok_ || pos_ + n > cap_) {
      ok_ = false;
      return;
    }
    memcpy(p_ + pos_, data, n);
    pos_ += n;
  }
  uint8_t* p_;
  size_t cap_;
  size_t pos_ = 0;
  bool ok_ = true;
};

class Reader {
 public:
  Reader(const uint8_t* data, size_t length) : p_(data), len_(length) {}
  uint8_t u8() {
    uint8_t v = 0;
    get(&v, 1);
    return v;
  }
  uint16_t u16() {
    uint8_t b[2] = {0, 0};
    get(b, 2);
    return static_cast<uint16_t>(b[0] | (b[1] << 8));
  }
  uint32_t u32() {
    uint8_t b[4] = {0, 0, 0, 0};
    get(b, 4);
    return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
           (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
  }
  uint64_t u64() {
    const uint64_t lo = u32();
    const uint64_t hi = u32();
    return lo | (hi << 32);
  }
  int8_t i8() { return static_cast<int8_t>(u8()); }
  int32_t i32() { return static_cast<int32_t>(u32()); }
  void bytes(void* out, size_t n) { get(static_cast<uint8_t*>(out), n); }
  size_t pos() const { return pos_; }
  bool ok() const { return ok_; }

 private:
  void get(uint8_t* out, size_t n) {
    if (!ok_ || pos_ + n > len_) {
      ok_ = false;
      memset(out, 0, n);
      return;
    }
    memcpy(out, p_ + pos_, n);
    pos_ += n;
  }
  const uint8_t* p_;
  size_t len_;
  size_t pos_ = 0;
  bool ok_ = true;
};

uint32_t readU32At(const uint8_t* data, size_t off) {
  return static_cast<uint32_t>(data[off]) | (static_cast<uint32_t>(data[off + 1]) << 8) |
         (static_cast<uint32_t>(data[off + 2]) << 16) | (static_cast<uint32_t>(data[off + 3]) << 24);
}

// --- small helpers ----------------------------------------------------------------

int32_t absDiff(int32_t a, int32_t b) { return a > b ? a - b : b - a; }

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool sha256HexToBytes(const char* hex, uint8_t out[kRecordSha256Bytes]) {
  for (size_t i = 0; i < kRecordSha256Bytes; ++i) {
    const int hi = hexNibble(hex[2 * i]);
    const int lo = hexNibble(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return hex[2 * kRecordSha256Bytes] == '\0';
}

void sha256BytesToHex(const uint8_t in[kRecordSha256Bytes], char* out /* 65 */) {
  static const char kDigits[] = "0123456789abcdef";
  for (size_t i = 0; i < kRecordSha256Bytes; ++i) {
    out[2 * i] = kDigits[in[i] >> 4];
    out[2 * i + 1] = kDigits[in[i] & 0x0F];
  }
  out[2 * kRecordSha256Bytes] = '\0';
}

// The six digests, in GeometryProvenance field order.
const char* provenanceField(const actuator::GeometryProvenance& p, uint8_t i) {
  switch (i) {
    case 0: return p.urdf_sha256;
    case 1: return p.mesh_manifest_sha256;
    case 2: return p.endpoint_semantic_sha256;
    case 3: return p.parking_semantic_sha256;
    case 4: return p.safety_policy_semantic_sha256;
    default: return p.allocation_sha256;
  }
}
char* provenanceFieldMutable(actuator::GeometryProvenance* p, uint8_t i) {
  return const_cast<char*>(provenanceField(*p, i));
}

bool isBool(uint8_t v) { return v <= 1; }

// A unit label: 1..7 chars of [A-Za-z0-9], NUL, then only NULs.
bool unitLabelWellFormed(const char* unit) {
  size_t n = 0;
  while (n < kPhysicalUnitLabelBytes && unit[n] != '\0') {
    const char c = unit[n];
    const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
    if (!ok) return false;
    ++n;
  }
  if (n == 0 || n == kPhysicalUnitLabelBytes) return false;
  for (size_t i = n; i < kPhysicalUnitLabelBytes; ++i) {
    if (unit[i] != '\0') return false;
  }
  return true;
}

bool buildIdWellFormed(const char* id) {
  size_t n = 0;
  while (n < kRecordBuildIdBytes && id[n] != '\0') {
    if (id[n] < 0x20 || id[n] > 0x7E) return false;
    ++n;
  }
  if (n == kRecordBuildIdBytes) return false;
  for (size_t i = n; i < kRecordBuildIdBytes; ++i) {
    if (id[i] != '\0') return false;
  }
  return true;
}

// --- representability (shared by the decoder and the validator) ---------------------

CalibrationRecordStatus checkRepresentable(const CalibrationRecord& r) {
  if (r.schema != kCalibrationRecordSchemaV1) return CalibrationRecordStatus::UNSUPPORTED_SCHEMA;
  if (r.generation == 0) return CalibrationRecordStatus::MALFORMED;
  if (!isBool(r.parameters_approved) || !isBool(r.calibration_accepted) ||
      !buildIdWellFormed(r.build_id)) {
    return CalibrationRecordStatus::MALFORMED;
  }
  for (uint8_t i = 0; i < kRecordLegCount; ++i) {
    const CalibrationRecordLegV1& l = r.leg[i];
    if (l.leg >= kLegCount || l.verdict > static_cast<uint8_t>(FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED) ||
        !isBool(l.diagnostics_accepted) || !isBool(l.session_completed) ||
        !isBool(l.permit_revoked) || !isBool(l.authority_released) || !isBool(l.has_rear_park) ||
        l.park_leg >= kLegCount || l.park_joint >= kJointKindCount) {
      return CalibrationRecordStatus::MALFORMED;
    }
  }
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const CalibrationRecordJointV1& j = r.joint[i];
    if (j.leg >= kLegCount || j.joint >= kJointKindCount || !unitLabelWellFormed(j.unit) ||
        (j.encoder_direction != 1 && j.encoder_direction != -1) ||
        j.encoder_direction_source < static_cast<uint8_t>(actuator::EncoderDirectionSource::CURRENT_HARDWARE_WITNESS) ||
        j.encoder_direction_source > static_cast<uint8_t>(actuator::EncoderDirectionSource::HISTORICAL_SLOT_UNCHANGED) ||
        j.q0_estimator > static_cast<uint8_t>(Q0Estimator::MANUAL_ZERO_POSE) ||
        j.q0_state > static_cast<uint8_t>(EvidenceState::REJECTED) ||
        j.q0_origin > static_cast<uint8_t>(CalibrationOrigin::LIVE_SESSION)) {
      return CalibrationRecordStatus::MALFORMED;
    }
    for (uint8_t s = 0; s < kContactSideCount; ++s) {
      const CalibrationRecordContactV1& c = j.contact[s];
      if (c.detection > static_cast<uint8_t>(ContactState::HARD_ABORT) ||
          c.state > static_cast<uint8_t>(EvidenceState::REJECTED) ||
          c.origin > static_cast<uint8_t>(CalibrationOrigin::LIVE_SESSION) ||
          !isBool(c.witness_accepted)) {
        return CalibrationRecordStatus::MALFORMED;
      }
    }
    const CalibrationRecordDiagnosticsV1& d = j.diagnostics;
    if (!isBool(d.evaluated) || !isBool(d.ordered) || !isBool(d.accepted)) {
      return CalibrationRecordStatus::MALFORMED;
    }
  }
  return CalibrationRecordStatus::OK;
}

bool diagnosticsEqual(const CalibrationRecordDiagnosticsV1& stored, const FullLegJointDiagnostics& d) {
  return stored.evaluated == (d.evaluated ? 1 : 0) && stored.ordered == (d.ordered ? 1 : 0) &&
         stored.accepted == (d.accepted ? 1 : 0) && stored.min_contact_tick == d.min_contact_tick &&
         stored.max_contact_tick == d.max_contact_tick &&
         stored.expected_span_ticks == d.expected_span_ticks &&
         stored.measured_span_ticks == d.measured_span_ticks &&
         stored.scale_permille == d.scale_permille && stored.affine_zero_tick == d.affine_zero_tick &&
         stored.affine_shift_from_q0_ticks == d.affine_shift_from_q0_ticks &&
         stored.fixed_endpoint_disagreement_ticks == d.fixed_endpoint_disagreement_ticks;
}

CalibrationRecordDiagnosticsV1 toRecordDiagnostics(const FullLegJointDiagnostics& d) {
  CalibrationRecordDiagnosticsV1 r;
  r.evaluated = d.evaluated ? 1 : 0;
  r.ordered = d.ordered ? 1 : 0;
  r.accepted = d.accepted ? 1 : 0;
  r.min_contact_tick = d.min_contact_tick;
  r.max_contact_tick = d.max_contact_tick;
  r.expected_span_ticks = d.expected_span_ticks;
  r.measured_span_ticks = d.measured_span_ticks;
  r.scale_permille = d.scale_permille;
  r.affine_zero_tick = d.affine_zero_tick;
  r.affine_shift_from_q0_ticks = d.affine_shift_from_q0_ticks;
  r.fixed_endpoint_disagreement_ticks = d.fixed_endpoint_disagreement_ticks;
  return r;
}

JointIdentity identityOf(const CalibrationRecordJointV1& j) {
  JointIdentity id{};
  id.leg = static_cast<Leg>(j.leg);
  id.joint = static_cast<JointKind>(j.joint);
  setPhysicalUnit(&id, j.unit);
  return id;
}

constexpr uint16_t kTick12BitLimit = static_cast<uint16_t>(actuator::kTicksPerRevolution);

}  // namespace

// --- status ------------------------------------------------------------------------

CalibrationRecordClass classifyCalibrationRecordStatus(CalibrationRecordStatus status) {
  switch (status) {
    case CalibrationRecordStatus::OK:
      return CalibrationRecordClass::OK;
    case CalibrationRecordStatus::UNSUPPORTED_SCHEMA:
    case CalibrationRecordStatus::PROFILE_UNBOUND:
    case CalibrationRecordStatus::PROVENANCE_MISMATCH:
    case CalibrationRecordStatus::IDENTITY_MISMATCH:
      return CalibrationRecordClass::INCOMPATIBLE;
    default:
      return CalibrationRecordClass::CORRUPT;
  }
}

const char* toString(CalibrationRecordStatus status) {
  switch (status) {
    case CalibrationRecordStatus::OK:                       return "OK";
    case CalibrationRecordStatus::TRUNCATED:                return "TRUNCATED";
    case CalibrationRecordStatus::BAD_MAGIC:                return "BAD_MAGIC";
    case CalibrationRecordStatus::BAD_LENGTH:               return "BAD_LENGTH";
    case CalibrationRecordStatus::BAD_CRC:                  return "BAD_CRC";
    case CalibrationRecordStatus::UNSUPPORTED_SCHEMA:       return "UNSUPPORTED_SCHEMA";
    case CalibrationRecordStatus::MALFORMED:                return "MALFORMED";
    case CalibrationRecordStatus::BUFFER_TOO_SMALL:         return "BUFFER_TOO_SMALL";
    case CalibrationRecordStatus::INCOHERENT:               return "INCOHERENT";
    case CalibrationRecordStatus::FORBIDDEN_AUTHORIZATION:  return "FORBIDDEN_AUTHORIZATION";
    case CalibrationRecordStatus::PROFILE_UNBOUND:          return "PROFILE_UNBOUND";
    case CalibrationRecordStatus::PROVENANCE_MISMATCH:      return "PROVENANCE_MISMATCH";
    case CalibrationRecordStatus::IDENTITY_MISMATCH:        return "IDENTITY_MISMATCH";
    case CalibrationRecordStatus::SOURCE_INCOMPLETE:        return "SOURCE_INCOMPLETE";
    case CalibrationRecordStatus::SOURCE_GEOMETRY_MISMATCH: return "SOURCE_GEOMETRY_MISMATCH";
  }
  return "UNKNOWN";
}

// --- CRC-32 ------------------------------------------------------------------------

uint32_t calibrationCrc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
  }
  return ~crc;
}

// --- encode ------------------------------------------------------------------------

CalibrationRecordStatus encodeCalibrationRecord(const CalibrationRecord& r, uint8_t* out,
                                                size_t capacity, size_t* written) {
  if (written != nullptr) *written = 0;
  if (out == nullptr || capacity < kTotalBytes) return CalibrationRecordStatus::BUFFER_TOO_SMALL;

  Writer w(out, capacity);
  w.u32(kCalibrationRecordMagic);
  w.u16(kCalibrationRecordSchemaV1);
  w.u16(0);  // flags
  w.u32(static_cast<uint32_t>(kTotalBytes));
  w.u32(r.generation);
  w.u64(r.geometry_tag);
  for (uint8_t i = 0; i < kRecordSha256Digests; ++i) w.bytes(r.digest[i], kRecordSha256Bytes);
  w.bytes(r.build_id, kRecordBuildIdBytes);
  w.u8(r.parameters_approved);
  w.u8(r.calibration_accepted);
  w.u16(r.contact_margin_ticks);

  for (uint8_t i = 0; i < kRecordLegCount; ++i) {
    const CalibrationRecordLegV1& l = r.leg[i];
    w.u8(l.leg);
    w.u8(l.verdict);
    w.u16(l.attempts);
    w.u8(l.contacts_measured);
    w.u8(l.contacts_accepted);
    w.u8(l.diagnostics_accepted);
    w.u8(l.session_completed);
    w.u8(l.permit_revoked);
    w.u8(l.authority_released);
    w.u8(l.has_rear_park);
    w.u8(l.park_leg);
    w.u8(l.park_joint);
    w.u8(l.park_bus_id);
    w.i32(l.park_target_urad);
  }

  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const CalibrationRecordJointV1& j = r.joint[i];
    w.u8(j.leg);
    w.u8(j.joint);
    w.bytes(j.unit, kPhysicalUnitLabelBytes);
    w.u8(j.bus_id);
    w.i8(j.encoder_direction);
    w.u8(j.encoder_direction_source);
    w.u16(j.q0_tick);
    w.u8(j.q0_estimator);
    w.u8(j.q0_state);
    w.u8(j.q0_origin);
    w.u8(j.q0_sample_count);
    w.u16(j.q0_stability_spread_ticks);
    for (uint8_t s = 0; s < kContactSideCount; ++s) {
      const CalibrationRecordContactV1& c = j.contact[s];
      w.u8(c.detection);
      w.u8(c.state);
      w.u8(c.origin);
      w.u8(c.witness_accepted);
      w.u16(c.scout_tick);
      w.u16(c.fine_tick_1);
      w.u16(c.fine_tick_2);
      w.u16(c.repeatability_ticks);
    }
    const CalibrationRecordDiagnosticsV1& d = j.diagnostics;
    w.u8(d.evaluated);
    w.u8(d.ordered);
    w.u8(d.accepted);
    w.u16(d.min_contact_tick);
    w.u16(d.max_contact_tick);
    w.u16(d.expected_span_ticks);
    w.u16(d.measured_span_ticks);
    w.u16(d.scale_permille);
    w.u16(d.affine_zero_tick);
    w.u16(d.affine_shift_from_q0_ticks);
    w.u16(d.fixed_endpoint_disagreement_ticks);
  }

  if (!w.ok() || w.pos() != kBodyBytes) return CalibrationRecordStatus::BUFFER_TOO_SMALL;
  w.u32(calibrationCrc32(out, kBodyBytes));
  if (!w.ok() || w.pos() != kTotalBytes) return CalibrationRecordStatus::BUFFER_TOO_SMALL;
  if (written != nullptr) *written = kTotalBytes;
  return CalibrationRecordStatus::OK;
}

// --- decode ------------------------------------------------------------------------

CalibrationRecordStatus inspectCalibrationEnvelope(const uint8_t* data, size_t length,
                                                   uint16_t* schema, uint32_t* generation) {
  if (data == nullptr || length < kCalibrationRecordMinBlobBytes) {
    return CalibrationRecordStatus::TRUNCATED;
  }
  if (readU32At(data, 0) != kCalibrationRecordMagic) return CalibrationRecordStatus::BAD_MAGIC;
  if (readU32At(data, kOffTotalLength) != length) return CalibrationRecordStatus::BAD_LENGTH;
  const size_t crc_off = length - kCalibrationRecordTrailerBytes;
  if (readU32At(data, crc_off) != calibrationCrc32(data, crc_off)) {
    return CalibrationRecordStatus::BAD_CRC;
  }
  if (schema != nullptr) *schema = static_cast<uint16_t>(data[4] | (data[5] << 8));
  if (generation != nullptr) *generation = readU32At(data, kOffGeneration);
  return CalibrationRecordStatus::OK;
}

CalibrationRecordStatus decodeCalibrationRecord(const uint8_t* data, size_t length,
                                                CalibrationRecord* out) {
  if (out == nullptr) return CalibrationRecordStatus::MALFORMED;
  uint16_t schema = 0;
  uint32_t generation = 0;
  CalibrationRecordStatus st = inspectCalibrationEnvelope(data, length, &schema, &generation);
  if (st != CalibrationRecordStatus::OK) return st;
  if (schema != kCalibrationRecordSchemaV1) return CalibrationRecordStatus::UNSUPPORTED_SCHEMA;
  if (length != kTotalBytes) return CalibrationRecordStatus::BAD_LENGTH;

  CalibrationRecord r;
  Reader rd(data, length - kCalibrationRecordTrailerBytes);
  (void)rd.u32();  // magic
  r.schema = rd.u16();
  const uint16_t flags = rd.u16();
  (void)rd.u32();  // length
  r.generation = rd.u32();
  r.geometry_tag = rd.u64();
  for (uint8_t i = 0; i < kRecordSha256Digests; ++i) rd.bytes(r.digest[i], kRecordSha256Bytes);
  rd.bytes(r.build_id, kRecordBuildIdBytes);
  r.parameters_approved = rd.u8();
  r.calibration_accepted = rd.u8();
  r.contact_margin_ticks = rd.u16();

  for (uint8_t i = 0; i < kRecordLegCount; ++i) {
    CalibrationRecordLegV1& l = r.leg[i];
    l.leg = rd.u8();
    l.verdict = rd.u8();
    l.attempts = rd.u16();
    l.contacts_measured = rd.u8();
    l.contacts_accepted = rd.u8();
    l.diagnostics_accepted = rd.u8();
    l.session_completed = rd.u8();
    l.permit_revoked = rd.u8();
    l.authority_released = rd.u8();
    l.has_rear_park = rd.u8();
    l.park_leg = rd.u8();
    l.park_joint = rd.u8();
    l.park_bus_id = rd.u8();
    l.park_target_urad = rd.i32();
  }

  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    CalibrationRecordJointV1& j = r.joint[i];
    j.leg = rd.u8();
    j.joint = rd.u8();
    rd.bytes(j.unit, kPhysicalUnitLabelBytes);
    j.bus_id = rd.u8();
    j.encoder_direction = rd.i8();
    j.encoder_direction_source = rd.u8();
    j.q0_tick = rd.u16();
    j.q0_estimator = rd.u8();
    j.q0_state = rd.u8();
    j.q0_origin = rd.u8();
    j.q0_sample_count = rd.u8();
    j.q0_stability_spread_ticks = rd.u16();
    for (uint8_t s = 0; s < kContactSideCount; ++s) {
      CalibrationRecordContactV1& c = j.contact[s];
      c.detection = rd.u8();
      c.state = rd.u8();
      c.origin = rd.u8();
      c.witness_accepted = rd.u8();
      c.scout_tick = rd.u16();
      c.fine_tick_1 = rd.u16();
      c.fine_tick_2 = rd.u16();
      c.repeatability_ticks = rd.u16();
    }
    CalibrationRecordDiagnosticsV1& d = j.diagnostics;
    d.evaluated = rd.u8();
    d.ordered = rd.u8();
    d.accepted = rd.u8();
    d.min_contact_tick = rd.u16();
    d.max_contact_tick = rd.u16();
    d.expected_span_ticks = rd.u16();
    d.measured_span_ticks = rd.u16();
    d.scale_permille = rd.u16();
    d.affine_zero_tick = rd.u16();
    d.affine_shift_from_q0_ticks = rd.u16();
    d.fixed_endpoint_disagreement_ticks = rd.u16();
  }

  if (!rd.ok() || rd.pos() != kBodyBytes) return CalibrationRecordStatus::BAD_LENGTH;
  if (flags != 0) return CalibrationRecordStatus::MALFORMED;
  st = checkRepresentable(r);
  if (st != CalibrationRecordStatus::OK) return st;
  *out = r;
  return CalibrationRecordStatus::OK;
}

// --- validation ----------------------------------------------------------------------

CalibrationRecordStatus validateCalibrationRecord(const CalibrationRecord& r,
                                                  const actuator::CalibrationGeometryProfile& profile) {
  CalibrationRecordStatus st = checkRepresentable(r);
  if (st != CalibrationRecordStatus::OK) return st;

  // Nothing persisted may authorize anything.
  if (r.parameters_approved != 0) return CalibrationRecordStatus::FORBIDDEN_AUTHORIZATION;
  if (r.calibration_accepted != 1) return CalibrationRecordStatus::INCOHERENT;
  if (!profile.bound()) return CalibrationRecordStatus::PROFILE_UNBOUND;

  // --- geometry provenance: internal coherence first, then against the profile ----
  actuator::GeometryProvenance stored{};
  for (uint8_t i = 0; i < kRecordSha256Digests; ++i) {
    sha256BytesToHex(r.digest[i], provenanceFieldMutable(&stored, i));
  }
  const actuator::GeometryProvenanceTag stored_tag = actuator::geometryProvenanceTag(stored);
  if (r.geometry_tag == actuator::kNoGeometryProvenance || r.geometry_tag != stored_tag) {
    return CalibrationRecordStatus::INCOHERENT;
  }
  if (r.geometry_tag != profile.provenanceTag() || !profile.provenanceMatches(stored)) {
    return CalibrationRecordStatus::PROVENANCE_MISMATCH;
  }

  // --- joints: position, identity, bus, encoder direction, q0, contacts --------------
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const CalibrationRecordJointV1& j = r.joint[i];
    if (j.leg != i / kJointKindCount || j.joint != i % kJointKindCount) {
      return CalibrationRecordStatus::INCOHERENT;  // duplicate / missing / misplaced joint
    }
    for (uint8_t k = 0; k < i; ++k) {
      if (r.joint[k].bus_id == j.bus_id || strncmp(r.joint[k].unit, j.unit, kPhysicalUnitLabelBytes) == 0) {
        return CalibrationRecordStatus::INCOHERENT;
      }
    }
    const JointIdentity identity = identityOf(j);
    const actuator::GeometryJointRecord* installed = profile.findJoint(identity);
    if (installed == nullptr || installed->bus_id != j.bus_id ||
        installed->encoder_direction != j.encoder_direction ||
        static_cast<uint8_t>(installed->encoder_direction_source) != j.encoder_direction_source) {
      return CalibrationRecordStatus::IDENTITY_MISMATCH;
    }

    // q0 acceptance rules, as CalibrationQ0Promotion applied them at capture.
    if (j.q0_estimator != static_cast<uint8_t>(Q0Estimator::MANUAL_ZERO_POSE) ||
        j.q0_state != static_cast<uint8_t>(EvidenceState::PROMOTED) ||
        j.q0_origin != static_cast<uint8_t>(CalibrationOrigin::LIVE_SESSION) ||
        j.q0_tick >= kTick12BitLimit ||
        absDiff(j.q0_tick, kServoRawCenter) > actuator::kQ0PlausibilityTicks ||
        j.q0_sample_count < actuator::kQ0AcceptanceMinSamples ||
        j.q0_sample_count > actuator::kQ0BootstrapMaxSamples ||
        j.q0_stability_spread_ticks > actuator::kQ0AcceptanceMaxSpreadTicks) {
      return CalibrationRecordStatus::INCOHERENT;
    }

    for (uint8_t s = 0; s < kContactSideCount; ++s) {
      const CalibrationRecordContactV1& c = j.contact[s];
      if (c.detection != static_cast<uint8_t>(ContactState::CONTACT_CONFIRMED) ||
          c.state != static_cast<uint8_t>(EvidenceState::PROMOTED) ||
          c.origin != static_cast<uint8_t>(CalibrationOrigin::LIVE_SESSION) ||
          c.witness_accepted != 1 || c.scout_tick >= kTick12BitLimit ||
          c.fine_tick_1 >= kTick12BitLimit || c.fine_tick_2 >= kTick12BitLimit ||
          c.repeatability_ticks != absDiff(c.fine_tick_1, c.fine_tick_2) ||
          c.repeatability_ticks > kFullLegRepeatabilityToleranceTicks) {
        return CalibrationRecordStatus::INCOHERENT;
      }
    }
  }

  // --- legs ----------------------------------------------------------------------------
  for (uint8_t i = 0; i < kRecordLegCount; ++i) {
    const CalibrationRecordLegV1& l = r.leg[i];
    if (l.leg != i) return CalibrationRecordStatus::INCOHERENT;
    if (l.verdict == static_cast<uint8_t>(FullLegVerdict::FINAL_OPERATIONAL_ENVELOPE_ACCEPTED)) {
      return CalibrationRecordStatus::FORBIDDEN_AUTHORIZATION;
    }
    if (l.verdict != static_cast<uint8_t>(FullLegVerdict::HARDWARE_CONTACT_CALIBRATED) ||
        l.attempts == 0 || l.contacts_measured != kFullLegContactsExpected ||
        l.contacts_accepted != kFullLegContactsExpected || l.diagnostics_accepted != 1 ||
        l.session_completed != 1 || l.permit_revoked != 1 || l.authority_released != 1) {
      return CalibrationRecordStatus::INCOHERENT;
    }
    if (l.has_rear_park == 0) {
      if (l.park_leg != 0 || l.park_joint != 0 || l.park_bus_id != 0 || l.park_target_urad != 0) {
        return CalibrationRecordStatus::INCOHERENT;
      }
    } else {
      if (l.park_leg == l.leg || l.park_target_urad > actuator::kMicroRadPerRevolution / 2 ||
          l.park_target_urad < -(actuator::kMicroRadPerRevolution / 2) ||
          r.joint[l.park_leg * kJointKindCount + l.park_joint].bus_id != l.park_bus_id) {
        return CalibrationRecordStatus::INCOHERENT;
      }
    }

    // Re-derive the three joint diagnostics through the production function.
    FullLegCalibrationRequest request{};
    request.leg = static_cast<Leg>(l.leg);
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      const CalibrationRecordJointV1& j = r.joint[i * kJointKindCount + k];
      const actuator::GeometryJointRecord* installed = profile.findJoint(identityOf(j));
      if (installed == nullptr) return CalibrationRecordStatus::IDENTITY_MISMATCH;
      request.direction[k] = installed->encoder_direction;
      request.urdf_lower[k] = installed->urdf_lower;
      request.urdf_upper[k] = installed->urdf_upper;
      request.joint[k].q0_tick = j.q0_tick;
    }
    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      const CalibrationRecordJointV1& j = r.joint[i * kJointKindCount + k];
      ContactEvidence sides[kContactSideCount];
      for (uint8_t s = 0; s < kContactSideCount; ++s) {
        sides[s].has_measurement = true;
        sides[s].fine_tick_1 = j.contact[s].fine_tick_1;
        sides[s].fine_tick_2 = j.contact[s].fine_tick_2;
      }
      const FullLegJointDiagnostics derived =
          deriveFullLegJointDiagnostics(request, static_cast<JointKind>(k), sides[0], sides[1]);
      if (!derived.accepted || !diagnosticsEqual(j.diagnostics, derived)) {
        return CalibrationRecordStatus::INCOHERENT;
      }
    }
  }
  return CalibrationRecordStatus::OK;
}

// --- builder ---------------------------------------------------------------------------

CalibrationRecordStatus buildCalibrationRecord(const CalibrationRecordSource& source,
                                               CalibrationRecord* out) {
  if (out == nullptr) return CalibrationRecordStatus::SOURCE_INCOMPLETE;
  *out = CalibrationRecord{};
  if (source.evidence == nullptr || source.profile == nullptr) {
    return CalibrationRecordStatus::SOURCE_INCOMPLETE;
  }
  const actuator::CalibrationGeometryProfile& profile = *source.profile;
  if (!profile.bound()) return CalibrationRecordStatus::PROFILE_UNBOUND;

  const actuator::GeometryProvenanceTag tag = profile.provenanceTag();
  out->geometry_tag = tag;
  for (uint8_t i = 0; i < kRecordSha256Digests; ++i) {
    if (!sha256HexToBytes(provenanceField(*profile.provenance(), i), out->digest[i])) {
      return CalibrationRecordStatus::PROVENANCE_MISMATCH;
    }
  }
  if (source.build_id != nullptr) {
    for (size_t i = 0; i + 1 < kRecordBuildIdBytes && source.build_id[i] != '\0'; ++i) {
      const char c = source.build_id[i];
      out->build_id[i] = (c >= 0x20 && c <= 0x7E) ? c : '?';
    }
  }

  bool all_calibrated = true;
  for (uint8_t li = 0; li < kRecordLegCount; ++li) {
    const FullLegRecord* src = source.evidence->find(static_cast<Leg>(li));
    if (src == nullptr || !src->present || src->leg != static_cast<Leg>(li) ||
        src->failure != FullLegFinalizeFailure::NONE ||
        src->executor_failure != FullLegFailure::NONE) {
      return CalibrationRecordStatus::SOURCE_INCOMPLETE;
    }
    if (src->geometry != tag) return CalibrationRecordStatus::SOURCE_GEOMETRY_MISMATCH;
    if (src->parameters_approved) return CalibrationRecordStatus::FORBIDDEN_AUTHORIZATION;
    if (li == 0) {
      out->contact_margin_ticks = src->contact_margin_ticks;
    } else if (src->contact_margin_ticks != out->contact_margin_ticks) {
      return CalibrationRecordStatus::INCOHERENT;
    }
    if (src->verdict != FullLegVerdict::HARDWARE_CONTACT_CALIBRATED) all_calibrated = false;

    CalibrationRecordLegV1& l = out->leg[li];
    l.leg = li;
    l.verdict = static_cast<uint8_t>(src->verdict);
    l.attempts = src->attempts;
    l.contacts_measured = src->contacts_measured;
    l.contacts_accepted = src->contacts_accepted;
    l.diagnostics_accepted = src->diagnostics_accepted ? 1 : 0;
    l.session_completed = src->session_completed ? 1 : 0;
    l.permit_revoked = src->permit_revoked ? 1 : 0;
    l.authority_released = src->authority_released ? 1 : 0;
    l.has_rear_park = src->has_rear_park ? 1 : 0;
    if (src->has_rear_park) {
      l.park_leg = static_cast<uint8_t>(src->park_leg);
      l.park_joint = static_cast<uint8_t>(src->park_joint);
      l.park_bus_id = src->park_bus_id;
      l.park_target_urad = src->park_target_urad;
    }

    for (uint8_t k = 0; k < kJointKindCount; ++k) {
      const FullLegJointRecord& sj = src->joints[k];
      const uint8_t index = static_cast<uint8_t>(li * kJointKindCount + k);
      CalibrationRecordJointV1& j = out->joint[index];
      const actuator::GeometryJointRecord* installed = profile.findJoint(sj.identity);
      if (installed == nullptr || installed->bus_id != sj.bus_id) {
        return CalibrationRecordStatus::IDENTITY_MISMATCH;
      }
      if (!sj.q0_present) return CalibrationRecordStatus::SOURCE_INCOMPLETE;
      if (sj.q0_geometry != tag) return CalibrationRecordStatus::SOURCE_GEOMETRY_MISMATCH;

      j.leg = li;
      j.joint = k;
      memcpy(j.unit, sj.identity.physical_unit, kPhysicalUnitLabelBytes);
      j.bus_id = sj.bus_id;
      j.encoder_direction = installed->encoder_direction;
      j.encoder_direction_source = static_cast<uint8_t>(installed->encoder_direction_source);
      j.q0_tick = sj.q0_tick;
      j.q0_state = static_cast<uint8_t>(sj.q0_state);
      j.q0_origin = static_cast<uint8_t>(sj.q0_origin);
      const CalibrationQ0CaptureMetadata& meta = source.q0_capture[index];
      j.q0_estimator = static_cast<uint8_t>(meta.estimator);
      j.q0_sample_count = meta.sample_count;
      j.q0_stability_spread_ticks = meta.stability_spread_ticks;

      for (uint8_t s = 0; s < kContactSideCount; ++s) {
        const ContactEvidence& e = sj.contact[s];
        if (!e.has_measurement || !sj.contact_recorded[s] || e.key.leg != static_cast<Leg>(li) ||
            e.key.joint != static_cast<JointKind>(k) || e.key.side != static_cast<ContactSide>(s)) {
          return CalibrationRecordStatus::SOURCE_INCOMPLETE;
        }
        CalibrationRecordContactV1& c = j.contact[s];
        c.detection = static_cast<uint8_t>(e.detection);
        c.state = static_cast<uint8_t>(e.state);
        c.origin = static_cast<uint8_t>(e.origin);
        c.witness_accepted = e.witness.accepted() ? 1 : 0;
        c.scout_tick = e.coarse_tick;
        c.fine_tick_1 = e.fine_tick_1;
        c.fine_tick_2 = e.fine_tick_2;
        c.repeatability_ticks = e.repeatability_ticks;
      }
      j.diagnostics = toRecordDiagnostics(sj.diagnostics);
    }
  }

  out->parameters_approved = 0;
  out->calibration_accepted = all_calibrated ? 1 : 0;
  return CalibrationRecordStatus::OK;
}

}  // namespace calibration
}  // namespace matdog
