#ifndef MATDOG_CALIBRATION_CALIBRATION_RECORD_H
#define MATDOG_CALIBRATION_CALIBRATION_RECORD_H

#include <stddef.h>
#include <stdint.h>

#include "../actuator/CalibrationGeometryProfile.h"
#include "CalibrationDomain.h"
#include "FullLegCalibrationFinalizer.h"

// CALIBRATION RECORD V1 - the persistable image of one accepted 24-contact
// Full Calibration (P2: format, codec, validation; no storage, no restore).
//
// Pure: no Arduino, no ServoBus, no NVS, no clock, no heap. The bytes here are
// evidence ABOUT a calibration. They are never authority:
//
//   - a decoded or validated record authorizes nothing - no motion, no
//     session, no permit, no ActuatorAuthority, no operational envelope;
//   - operational envelopes, limit admissions and session ids are NOT stored
//     (envelopes are derived from the contacts and must be re-derived and
//     re-approved by whoever uses them);
//   - the record is bound to ONE geometry (six SHA-256 digests) and ONE
//     physical installation (12 unit labels, bus ids, encoder directions) and
//     is rejected whole for any other.
//
// WIRE FORMAT. Explicit little-endian field-by-field serialization. No struct
// is ever dumped, so padding, alignment, enum width and compiler cannot change
// the bytes. ENVELOPE (stable across schemas, so a reader can classify a blob
// it cannot decode):
//
//   off  0  u32  magic  "MDCR" (bytes 4D 44 43 52)
//   off  4  u16  schema
//   off  6  u16  flags (V1: must be 0)
//   off  8  u32  total length of the blob, trailer included
//   off 12  u32  generation (monotonic across saves, never 0)
//   ...     schema-specific body
//   end -4  u32  CRC-32 (IEEE 802.3) over every preceding byte
//
// V1 body (1088 bytes in total): geometry tag, six 32-byte digests, build id,
// parameters_approved, calibration_accepted, contact margin, 4 leg records
// (18 B), 12 joint records (64 B).

namespace matdog {
namespace calibration {

constexpr uint32_t kCalibrationRecordMagic = 0x5243444Du;  // "MDCR" as little-endian bytes
constexpr uint16_t kCalibrationRecordSchemaV1 = 1;
constexpr size_t kCalibrationRecordEnvelopePrefixBytes = 16;
constexpr size_t kCalibrationRecordTrailerBytes = 4;
constexpr size_t kCalibrationRecordMinBlobBytes =
    kCalibrationRecordEnvelopePrefixBytes + kCalibrationRecordTrailerBytes;

constexpr size_t kCalibrationRecordV1EncodedBytes = 1088;

constexpr uint8_t kRecordSha256Digests = 6;
constexpr size_t kRecordSha256Bytes = 32;
constexpr size_t kRecordBuildIdBytes = 24;

// The schema-V1 record is a COMPLETE calibration or nothing.
constexpr uint8_t kRecordLegCount = kLegCount;
constexpr uint8_t kRecordJointCount = kLegServoSlotCount;  // 12

enum class CalibrationRecordStatus : uint8_t {
  OK = 0,
  // --- envelope / codec ---
  TRUNCATED,           // shorter than the envelope
  BAD_MAGIC,
  BAD_LENGTH,          // header length != blob length, or a V1 blob of the wrong size
  BAD_CRC,
  UNSUPPORTED_SCHEMA,  // intact envelope, schema this build cannot read
  MALFORMED,           // enum/bool/charset out of range, reserved bits set
  BUFFER_TOO_SMALL,    // encoder only
  // --- semantic validation ---
  INCOHERENT,          // fields contradict each other (diagnostics, counts, ordering...)
  FORBIDDEN_AUTHORIZATION,  // claims approved parameters / operational acceptance
  PROFILE_UNBOUND,
  PROVENANCE_MISMATCH, // geometry tag / six digests differ from the bound profile
  IDENTITY_MISMATCH,   // unit label, bus id or encoder direction differ from the profile
  // --- builder ---
  SOURCE_INCOMPLETE,
  SOURCE_GEOMETRY_MISMATCH,
};

enum class CalibrationRecordClass : uint8_t {
  OK = 0,
  // Self-damaged or self-contradictory: never produced by a correct writer.
  CORRUPT = 1,
  // Intact, but written for another schema / geometry / installation.
  INCOMPATIBLE = 2,
};

CalibrationRecordClass classifyCalibrationRecordStatus(CalibrationRecordStatus status);
const char* toString(CalibrationRecordStatus status);

// --- in-memory record (explicit-width fields; enums kept as raw bytes so that
//     a decoded value that is out of range can be represented and refused) ---

struct CalibrationRecordContactV1 {
  uint8_t detection = 0;         // ContactState
  uint8_t state = 0;             // EvidenceState
  uint8_t origin = 0;            // CalibrationOrigin
  uint8_t witness_accepted = 0;  // 0/1
  uint16_t scout_tick = 0;
  uint16_t fine_tick_1 = 0;
  uint16_t fine_tick_2 = 0;
  uint16_t repeatability_ticks = 0;
};

struct CalibrationRecordDiagnosticsV1 {
  uint8_t evaluated = 0;
  uint8_t ordered = 0;
  uint8_t accepted = 0;
  uint16_t min_contact_tick = 0;
  uint16_t max_contact_tick = 0;
  uint16_t expected_span_ticks = 0;
  uint16_t measured_span_ticks = 0;
  uint16_t scale_permille = 0;
  uint16_t affine_zero_tick = 0;
  uint16_t affine_shift_from_q0_ticks = 0;
  uint16_t fixed_endpoint_disagreement_ticks = 0;
};

struct CalibrationRecordJointV1 {
  uint8_t leg = 0;    // Leg
  uint8_t joint = 0;  // JointKind
  char unit[kPhysicalUnitLabelBytes] = {0};  // NUL-padded physical unit label
  uint8_t bus_id = 0;
  int8_t encoder_direction = 0;       // -1 / +1
  uint8_t encoder_direction_source = 0;  // actuator::EncoderDirectionSource

  uint16_t q0_tick = 0;
  uint8_t q0_estimator = 0;  // Q0Estimator
  uint8_t q0_state = 0;      // EvidenceState at capture
  uint8_t q0_origin = 0;     // CalibrationOrigin at capture
  uint8_t q0_sample_count = 0;
  uint16_t q0_stability_spread_ticks = 0;

  CalibrationRecordContactV1 contact[kContactSideCount];  // [MIN, MAX]
  CalibrationRecordDiagnosticsV1 diagnostics;
};

struct CalibrationRecordLegV1 {
  uint8_t leg = 0;      // Leg
  uint8_t verdict = 0;  // FullLegVerdict
  uint16_t attempts = 0;
  uint8_t contacts_measured = 0;
  uint8_t contacts_accepted = 0;
  uint8_t diagnostics_accepted = 0;
  uint8_t session_completed = 0;
  uint8_t permit_revoked = 0;
  uint8_t authority_released = 0;
  uint8_t has_rear_park = 0;
  uint8_t park_leg = 0;
  uint8_t park_joint = 0;
  uint8_t park_bus_id = 0;
  int32_t park_target_urad = 0;
};

struct CalibrationRecord {
  uint16_t schema = kCalibrationRecordSchemaV1;
  uint32_t generation = 0;  // assigned by the store at save, never by the builder

  actuator::GeometryProvenanceTag geometry_tag = actuator::kNoGeometryProvenance;
  uint8_t digest[kRecordSha256Digests][kRecordSha256Bytes] = {};  // urdf, mesh, endpoint,
                                                                  // parking, safety, allocation
  char build_id[kRecordBuildIdBytes] = {0};  // informational only, never compared

  uint8_t parameters_approved = 0;   // V1: must be 0
  uint8_t calibration_accepted = 0;  // V1: must be 1 (24/24, all diagnostics accepted)
  uint16_t contact_margin_ticks = 0;  // informational placeholder inset, not an envelope

  CalibrationRecordLegV1 leg[kRecordLegCount];        // index = Leg
  CalibrationRecordJointV1 joint[kRecordJointCount];  // index = leg * 3 + JointKind
};

// --- CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320) ---
uint32_t calibrationCrc32(const uint8_t* data, size_t length);

// --- codec ---

// Serialize. Does not validate semantics (the store validates first); it only
// refuses a buffer that is too small. *written = kCalibrationRecordV1EncodedBytes.
CalibrationRecordStatus encodeCalibrationRecord(const CalibrationRecord& record, uint8_t* out,
                                                size_t capacity, size_t* written);

// Envelope-only check: length, magic, header length, CRC. On OK fills schema and
// generation. This is what lets a reader keep monotonic generations across a
// record it cannot otherwise interpret.
CalibrationRecordStatus inspectCalibrationEnvelope(const uint8_t* data, size_t length,
                                                   uint16_t* schema, uint32_t* generation);

// Envelope + schema + V1 structure. Does NOT consult any geometry profile and
// does NOT judge semantics: use validateCalibrationRecord for that.
CalibrationRecordStatus decodeCalibrationRecord(const uint8_t* data, size_t length,
                                                CalibrationRecord* out);

// --- validation ---

// Everything that makes a record safe to hold as evidence: structure, counts,
// ordering/uniqueness, ids, numeric domains, geometry provenance against the
// bound profile, unit/bus/encoder-direction identity, q0 acceptance rules,
// contact/witness rules and a full re-derivation of the twelve joint
// diagnostics through deriveFullLegJointDiagnostics(). A stored "accepted"
// flag is never trusted: it must equal what is recomputed. Any contradiction
// rejects the whole record.
CalibrationRecordStatus validateCalibrationRecord(const CalibrationRecord& record,
                                                  const actuator::CalibrationGeometryProfile& profile);

// --- builder ---

// Per-joint q0 capture facts that FullLegRecord does not carry. Never defaulted:
// the caller states them (or the record cannot be built).
struct CalibrationQ0CaptureMetadata {
  Q0Estimator estimator = Q0Estimator::NONE;
  uint8_t sample_count = 0;
  uint16_t stability_spread_ticks = 0;
};

struct CalibrationRecordSource {
  const FullLegEvidenceStore* evidence = nullptr;
  const actuator::CalibrationGeometryProfile* profile = nullptr;
  // Indexed by legSlotIndex(leg, joint).
  CalibrationQ0CaptureMetadata q0_capture[kRecordJointCount];
  const char* build_id = nullptr;  // informational; truncated to fit, may be null
};

// Copies what the evidence store and the bound profile say into `out`. The
// generation stays 0. Refuses (SOURCE_*) a source that is incomplete or bound
// to another geometry; every other judgement is validateCalibrationRecord's.
CalibrationRecordStatus buildCalibrationRecord(const CalibrationRecordSource& source,
                                               CalibrationRecord* out);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_RECORD_H
