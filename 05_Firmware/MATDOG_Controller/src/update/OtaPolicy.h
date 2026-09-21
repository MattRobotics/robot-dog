#ifndef MATDOG_UPDATE_OTA_POLICY_H
#define MATDOG_UPDATE_OTA_POLICY_H

#include <stddef.h>
#include <stdint.h>

#include "Sha256.h"

// OTA-A decision layer. Deliberately <stdint.h> only: no <Arduino.h>, no
// <esp_ota_ops.h>, no radio, no transport.
//
//     network transport            (OTA-A: not implemented; pluggable)
//           |
//           v
//     update/OtaManager            Controller-facing owner + boot lifecycle
//           |
//           v
//     update/OtaPolicy             THIS FILE - the state machine
//           |
//           v
//     OtaBackend (abstract)        implemented by update/OtaEspBackend.cpp
//           |
//           v
//     inactive OTA application slot
//
// The backend is an interface rather than a direct esp_ota_* call so that
// scripts/tests/test_ota_policy.cpp drives the REAL state machine against a
// fake backend that can fail any individual operation on demand. The logic
// under test is the shipped logic; only the flash I/O is substituted.

namespace matdog {
namespace update {

// ---------------------------------------------------------------------------
// Partition description - our own, so this header stays ESP-IDF-free
// ---------------------------------------------------------------------------

// Mirrors the esp_partition_subtype_t app range. ota_0..ota_15 are
// 0x10..0x1F; anything else (factory 0x00, test 0x20, any data subtype) is
// not a legal OTA application target.
constexpr uint8_t kSubtypeOtaMin = 0x10;
constexpr uint8_t kSubtypeOtaMax = 0x1F;

struct OtaPartitionInfo {
  bool valid = false;
  uint32_t address = 0;
  uint32_t size = 0;
  uint8_t subtype = 0;
  char label[17] = {0};

  bool isOtaAppSlot() const {
    return valid && subtype >= kSubtypeOtaMin && subtype <= kSubtypeOtaMax;
  }
  bool sameAs(const OtaPartitionInfo& other) const {
    return valid && other.valid && address == other.address && size == other.size;
  }
};

// Mirrors esp_ota_img_states_t (esp_flash_partitions.h), verified against the
// installed ESP-IDF v5.5.5 headers.
enum class OtaImgState : uint8_t {
  NEW            = 0,
  PENDING_VERIFY = 1,
  VALID          = 2,
  INVALID        = 3,
  ABORTED        = 4,
  UNDEFINED      = 5,  // ESP_OTA_IMG_UNDEFINED (0xFFFFFFFF) folded to a small value
  UNREADABLE     = 6,  // esp_ota_get_state_partition() returned an error
};

// ---------------------------------------------------------------------------
// Image metadata - what a transport must declare BEFORE any flash is touched
// ---------------------------------------------------------------------------

// Reuses the identity scheme the repository already has. Nothing new is
// invented:
//   build_id  == build::kBuildId, the git short SHA injected by
//                scripts/build.sh (plus "-dirty"), which really is present
//                as a string in the built image.
//   sha256    == the SHA-256 of the exact application binary bytes, the same
//                value scripts/build_manifest.py records as
//                APPLICATION_SHA256.
//   size      == APPLICATION_SIZE from that same manifest.
//
// Deliberately NOT esp_app_desc_t: in an Arduino-ESP32 build its version and
// project_name fields describe arduino-lib-builder, not MATDOG (verified by
// parsing the real binary: version='ee57070', project_name=
// 'arduino-lib-builder'), so it cannot answer "is this the firmware I
// expected?".
constexpr uint32_t kOtaMetadataSchema = 1;
constexpr size_t kOtaBuildIdBytes = 24;  // 12-hex SHA + "-dirty" + NUL, with room

struct OtaImageMetadata {
  uint32_t schema_version = 0;
  uint32_t image_size = 0;
  uint8_t sha256[kSha256DigestBytes] = {0};
  char build_id[kOtaBuildIdBytes] = {0};
};

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

enum class OtaState : uint8_t {
  IDLE              = 0,  // no update in progress; boot target untouched
  TARGET_RESOLVED   = 1,  // metadata accepted, inactive slot resolved and validated,
                          // NO flash written yet
  RECEIVING         = 2,  // write stream open; bytes are landing in the inactive slot
  IMAGE_SEALED      = 3,  // stream closed and the ESP-IDF image check passed
  IDENTITY_VERIFIED = 4,  // size + SHA-256 match what the sender declared
  BOOT_TARGET_SET   = 5,  // otadata updated; the next boot uses the new slot
  FAILED            = 6,  // terminal for this attempt; boot target NEVER touched
};

enum class OtaFault : uint8_t {
  NONE = 0,
  WRONG_STATE,                // an API call that this state does not allow
  NOT_AUTHORIZED,             // the authorization gate refused (or none installed)
  METADATA_SCHEMA,            // unknown metadata schema version
  METADATA_INCOMPLETE,        // size, hash or build id missing/malformed
  IMAGE_EMPTY,                // declared size 0
  RUNNING_IMAGE_UNCONFIRMED,  // running app is PENDING_VERIFY: esp_ota_begin would refuse
  NO_INACTIVE_SLOT,           // backend could not resolve a next update partition
  TARGET_IS_RUNNING,          // resolved target is the running partition
  TARGET_NOT_OTA_SLOT,        // target subtype is outside ota_0..ota_15
  IMAGE_TOO_LARGE,            // declared size exceeds the target partition
  WRITE_OPEN_REJECTED,        // backend refused to open the write stream
  WRITE_FAILED,               // a chunk write failed
  OVERRUN,                    // more bytes arrived than were declared
  INCOMPLETE_STREAM,          // fewer bytes arrived than were declared
  IMAGE_REJECTED,             // ESP-IDF rejected the written image structure
  HASH_MISMATCH,              // our SHA-256 of the received bytes != declared
  BOOT_SWITCH_REJECTED,       // setting the boot partition failed
};

// ---------------------------------------------------------------------------
// OTA-B boundary: the authorization gate
// ---------------------------------------------------------------------------

// OTA-A must not invent a temporary ActuatorAuthority, and must not silently
// become a bypass of the real one when it arrives. The compromise is this
// interface plus one rule: the policy FAILS CLOSED with no gate installed.
//
// Permission during OTA-A is therefore not an implicit "nullptr means yes".
// It is a named, greppable object (OtaStageAGate below) that the Controller
// has to install on purpose, so replacing it with the real authority model is
// a one-line change at one site, and forgetting to is a refusal rather than a
// silent allow.
enum class OtaGateVerdict : uint8_t {
  PERMITTED_OTA_A_NO_AUTHORITY_MODEL_YET = 0,  // the OTA-A placeholder answer
  PERMITTED_BY_AUTHORITY                 = 1,  // OTA-B: the real model said yes
  REFUSED_NO_GATE_INSTALLED              = 2,  // fail closed
  REFUSED_BY_AUTHORITY                   = 3,  // OTA-B: the real model said no
};

class OtaAuthorizationGate {
 public:
  virtual ~OtaAuthorizationGate() = default;
  virtual OtaGateVerdict otaPermitted() const = 0;
};

// The OTA-A placeholder. It permits, and says exactly why it permits: there
// is no authority model yet. TO_IMPLEMENT / OTA-B: replace with a gate backed
// by ActuatorAuthority (NONE/DIAGNOSTICS/CALIBRATION/QC/PROVISIONING/MOTION),
// which must refuse while motion, calibration or a service write transaction
// is active.
class OtaStageAGate : public OtaAuthorizationGate {
 public:
  OtaGateVerdict otaPermitted() const override {
    return OtaGateVerdict::PERMITTED_OTA_A_NO_AUTHORITY_MODEL_YET;
  }
};

bool isPermitted(OtaGateVerdict verdict);

// ---------------------------------------------------------------------------
// Backend: everything the policy needs from flash, as an interface
// ---------------------------------------------------------------------------

class OtaBackend {
 public:
  virtual ~OtaBackend() = default;

  virtual OtaPartitionInfo runningPartition() = 0;
  virtual OtaPartitionInfo nextUpdatePartition() = 0;
  virtual OtaImgState imageState(const OtaPartitionInfo& partition) = 0;

  // esp_ota_begin equivalent. image_size is advisory for erase strategy; the
  // policy has already bounded it against the partition.
  virtual bool beginWrite(const OtaPartitionInfo& target, uint32_t image_size) = 0;
  virtual bool write(const uint8_t* data, uint32_t len) = 0;
  // esp_ota_end equivalent: closes the stream AND runs the ESP-IDF image
  // validation. A false return means the written bytes are not a valid app.
  virtual bool endWrite() = 0;
  // Must be safe to call when no stream is open.
  virtual void abortWrite() = 0;

  virtual bool setBootPartition(const OtaPartitionInfo& target) = 0;

  virtual bool markAppValid() = 0;
  virtual bool rollbackPossible() = 0;
};

// ---------------------------------------------------------------------------
// Observable snapshot
// ---------------------------------------------------------------------------

struct OtaCounters {
  uint32_t updates_started = 0;
  uint32_t updates_committed = 0;
  uint32_t updates_failed = 0;
  uint32_t updates_aborted = 0;
};

struct OtaStatus {
  OtaState state = OtaState::IDLE;
  OtaFault fault = OtaFault::NONE;
  OtaGateVerdict last_gate_verdict = OtaGateVerdict::REFUSED_NO_GATE_INSTALLED;

  OtaPartitionInfo running{};
  OtaPartitionInfo target{};
  OtaImgState running_image_state = OtaImgState::UNDEFINED;

  uint32_t declared_size = 0;
  uint32_t bytes_written = 0;
  uint32_t chunks_written = 0;

  char declared_sha256[kSha256HexBytes] = {0};
  char computed_sha256[kSha256HexBytes] = {0};
  char declared_build_id[kOtaBuildIdBytes] = {0};

  // The single most safety-relevant bit in this struct: has otadata been
  // changed during this session? It must be false in every state except
  // BOOT_TARGET_SET.
  bool boot_target_changed = false;
  bool rollback_possible = false;

  OtaCounters counters{};
};

// ---------------------------------------------------------------------------
// The policy
// ---------------------------------------------------------------------------

class OtaPolicy {
 public:
  // gate may be nullptr; that is a refusal, not a permission.
  void begin(OtaBackend* backend, const OtaAuthorizationGate* gate);

  // Step 1. Validates the metadata and resolves + validates the target.
  // Touches NO flash. IDLE -> TARGET_RESOLVED, or -> FAILED.
  bool prepare(const OtaImageMetadata& metadata);

  // Step 2. Opens the write stream (erases). TARGET_RESOLVED -> RECEIVING.
  bool openStream();

  // Step 3. Streams bytes. Bounded, allocation-free; the image is never held
  // in RAM. Rejects an overrun before writing it.
  bool writeChunk(const uint8_t* data, uint32_t len);

  // Step 4. Closes the stream, runs the ESP-IDF image check, then verifies
  // OUR size + SHA-256. RECEIVING -> IMAGE_SEALED -> IDENTITY_VERIFIED.
  bool finishStream();

  // Step 5. THE ONLY method that changes the boot target, and it is callable
  // from exactly one state. IDENTITY_VERIFIED -> BOOT_TARGET_SET.
  bool commitBootTarget();

  // Idempotent. Safe from any state, including IDLE and after a commit
  // (where it does NOT undo the boot target - see the implementation).
  void abort();

  // Software reset of the state machine back to IDLE. Refuses after a commit,
  // because the pending boot switch is a real fact that must stay reported.
  bool reset();

  OtaState state() const { return status_.state; }
  OtaFault fault() const { return status_.fault; }
  const OtaStatus& status() const { return status_; }

  // Refreshes running-partition facts. Cheap; called by the manager, not per
  // chunk.
  void refreshPartitionFacts();

 private:
  bool failWith(OtaFault fault);
  void resetTransfer();

  OtaBackend* backend_ = nullptr;
  const OtaAuthorizationGate* gate_ = nullptr;
  OtaStatus status_{};
  Sha256 hash_{};
  uint8_t expected_sha_[kSha256DigestBytes] = {0};
};

const char* toString(OtaState state);
const char* toString(OtaFault fault);
const char* toString(OtaImgState state);
const char* toString(OtaGateVerdict verdict);

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_POLICY_H
