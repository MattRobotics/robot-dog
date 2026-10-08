#ifndef MATDOG_CALIBRATION_CALIBRATION_SAVE_MARKER_H
#define MATDOG_CALIBRATION_CALIBRATION_SAVE_MARKER_H

#include <stddef.h>
#include <stdint.h>

// SAVE MARKER (schema 2) - the one persistent fact that lets a reboot tell what a
// SAVE reached and what the CALLER has acknowledged (P2.4 / P2.4.1). Pure: no
// Arduino, no NVS, no heap.
//
// "Verified on flash" is not "acknowledged by the caller": a SAVE can publish its
// last marker and still fail to deliver SAVE=OK. Only an explicit ACK for that
// generation, recorded here, lets a generation replace the previous one.
//
// It records, separately from the two record slots:
//   - acknowledged_generation: the last generation the caller acknowledged
//     (0 = none). Its record slot is protected: nothing may overwrite it;
//   - begun_generation: the highest generation a SAVE was ever started for;
//   - state:
//       IDLE          no SAVE in flight and no generation waiting for an ACK;
//       PENDING       a SAVE of begun_generation is in flight (or was interrupted);
//       AWAITING_ACK  begun_generation was written and read back on flash; the
//                     caller has not acknowledged it yet.
//
// Invariants (any violation is INCOHERENT and the marker is rejected whole):
//   IDLE          begun >= acknowledged, and begun >= 1 when acknowledged == 0.
//                 begun > acknowledged means "a later attempt was explicitly
//                 discarded" (reconciliation); never produced by SAVE or ACK.
//   PENDING       begun > acknowledged.
//   AWAITING_ACK  begun > acknowledged.
//
// This is not a journal: one fixed-size value, rewritten in place, no history.
//
// WIRE FORMAT. Explicit little-endian field-by-field serialization (28 bytes):
//   off  0  u32  magic "MDMK" (bytes 4D 44 4D 4B)
//   off  4  u16  schema (2; schema 1 was the never-installed P2.4 layout)
//   off  6  u16  flags (must be 0)
//   off  8  u32  total length, trailer included (28)
//   off 12  u32  acknowledged_generation
//   off 16  u32  begun_generation
//   off 20  u8   state (1 = IDLE, 2 = PENDING, 3 = AWAITING_ACK)
//   off 21  u8[3] reserved (must be 0)
//   off 24  u32  CRC-32 (IEEE 802.3) over every preceding byte
// The envelope (magic, schema, length, CRC) is stable across schemas so a reader
// can tell "intact but written by another schema" from "damaged".

namespace matdog {
namespace calibration {

constexpr uint32_t kSaveMarkerMagic = 0x4B4D444Du;  // "MDMK" as little-endian bytes
constexpr uint16_t kSaveMarkerSchemaV1 = 2;
constexpr size_t kSaveMarkerV1Bytes = 28;
constexpr size_t kSaveMarkerEnvelopePrefixBytes = 12;
constexpr size_t kSaveMarkerTrailerBytes = 4;
// Reader scratch: anything larger than this is not a V1 marker.
constexpr size_t kSaveMarkerScratchBytes = 64;

enum class SaveMarkerState : uint8_t {
  IDLE = 1,
  PENDING = 2,
  AWAITING_ACK = 3,
};

struct SaveMarker {
  SaveMarkerState state = SaveMarkerState::IDLE;
  uint32_t acknowledged_generation = 0;
  uint32_t begun_generation = 0;
};

inline bool operator==(const SaveMarker& a, const SaveMarker& b) {
  return a.state == b.state && a.acknowledged_generation == b.acknowledged_generation &&
         a.begun_generation == b.begun_generation;
}

enum class SaveMarkerStatus : uint8_t {
  OK = 0,
  TRUNCATED,           // shorter than the envelope
  BAD_MAGIC,
  BAD_LENGTH,          // header length != blob length, or a V1 blob of the wrong size
  BAD_CRC,
  UNSUPPORTED_SCHEMA,  // intact envelope, schema this build cannot read
  MALFORMED,           // flags / reserved / state out of range
  INCOHERENT,          // generations contradict the state
  BUFFER_TOO_SMALL,    // encoder only
};

// UNSUPPORTED_SCHEMA is intact-but-foreign; everything else is damage.
inline bool isForeignSaveMarker(SaveMarkerStatus s) {
  return s == SaveMarkerStatus::UNSUPPORTED_SCHEMA;
}
const char* toString(SaveMarkerStatus status);
const char* toString(SaveMarkerState state);

// Checks the invariants above. OK or INCOHERENT/MALFORMED.
SaveMarkerStatus validateSaveMarker(const SaveMarker& marker);

// Refuses (INCOHERENT/MALFORMED) a marker that violates the invariants: an
// invalid marker is never written. *written = kSaveMarkerV1Bytes.
SaveMarkerStatus encodeSaveMarker(const SaveMarker& marker, uint8_t* out, size_t capacity,
                                  size_t* written);

// Envelope + schema + V1 structure + invariants.
SaveMarkerStatus decodeSaveMarker(const uint8_t* data, size_t length, SaveMarker* out);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_SAVE_MARKER_H
