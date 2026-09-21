#include "OtaPolicy.h"

namespace matdog {
namespace update {

namespace {

void copyBounded(char* dst, size_t dst_size, const char* src) {
  if (dst == nullptr || dst_size == 0) return;
  size_t i = 0;
  if (src != nullptr) {
    while (i + 1 < dst_size && src[i] != '\0') {
      dst[i] = src[i];
      ++i;
    }
  }
  dst[i] = '\0';
}

bool buildIdLooksValid(const char* build_id) {
  if (build_id == nullptr) return false;
  size_t len = 0;
  while (len < kOtaBuildIdBytes && build_id[len] != '\0') ++len;
  if (len == 0 || len >= kOtaBuildIdBytes) return false;

  // "unknown" is what config/BuildConfig.h falls back to when a build
  // skipped scripts/build.sh. An image that cannot say which commit it came
  // from is not something MATDOG accepts over the air.
  const char kUnknown[] = "unknown";
  bool is_unknown = (len == sizeof(kUnknown) - 1);
  for (size_t i = 0; is_unknown && i < len; ++i) {
    if (build_id[i] != kUnknown[i]) is_unknown = false;
  }
  if (is_unknown) return false;

  // Printable ASCII only: this string is echoed into diagnostics.
  for (size_t i = 0; i < len; ++i) {
    const char c = build_id[i];
    if (c < 0x20 || c > 0x7E) return false;
  }
  return true;
}

}  // namespace

bool isPermitted(OtaGateVerdict verdict) {
  return verdict == OtaGateVerdict::PERMITTED_BY_AUTHORITY;
}

void OtaPolicy::begin(OtaBackend* backend, OtaAuthorizationGate* gate) {
  backend_ = backend;
  gate_ = gate;
  status_ = OtaStatus{};
  resetTransfer();
  refreshPartitionFacts();
}

void OtaPolicy::resetTransfer() {
  hash_.reset();
  status_.declared_size = 0;
  status_.bytes_written = 0;
  status_.chunks_written = 0;
  status_.declared_sha256[0] = '\0';
  status_.computed_sha256[0] = '\0';
  status_.declared_build_id[0] = '\0';
  status_.target = OtaPartitionInfo{};
  for (size_t i = 0; i < kSha256DigestBytes; ++i) expected_sha_[i] = 0;
}

void OtaPolicy::refreshPartitionFacts() {
  if (backend_ == nullptr) return;
  status_.running = backend_->runningPartition();
  status_.running_image_state = backend_->imageState(status_.running);
  status_.rollback_possible = backend_->rollbackPossible();
}

bool OtaPolicy::failWith(OtaFault fault) {
  status_.fault = fault;
  status_.state = OtaState::FAILED;
  status_.counters.updates_failed++;
  // Anything already open in the backend is released. The boot target is
  // deliberately NOT touched here: a failed attempt must leave the device
  // booting exactly what it booted before.
  if (backend_ != nullptr) backend_->abortWrite();
  // And the actuator exclusivity hold is given back, so a failed update
  // cannot leave the robot permanently unable to calibrate. Idempotent and
  // safe when nothing was ever held.
  if (gate_ != nullptr) gate_->endExclusive();
  return false;
}

bool OtaPolicy::prepare(const OtaImageMetadata& metadata) {
  if (backend_ == nullptr) return failWith(OtaFault::WRONG_STATE);

  // A new attempt is only allowed from a settled state. In particular it is
  // refused once a boot target has been set: that switch is pending a reboot
  // and must not be silently replaced.
  if (status_.state != OtaState::IDLE && status_.state != OtaState::FAILED) {
    status_.fault = OtaFault::WRONG_STATE;
    return false;
  }

  resetTransfer();
  status_.fault = OtaFault::NONE;
  status_.state = OtaState::IDLE;
  status_.counters.updates_started++;

  // --- Gate first, and as a HOLD rather than a query. -------------------
  // beginExclusive() both checks that no actuator owner is active and takes
  // the exclusivity hold in one call, so no owner can appear between the
  // check and the rest of the update. Fail closed when no gate is installed.
  const OtaGateVerdict verdict = (gate_ == nullptr)
                                     ? OtaGateVerdict::REFUSED_NO_GATE_INSTALLED
                                     : gate_->beginExclusive();
  status_.last_gate_verdict = verdict;
  if (!isPermitted(verdict)) return failWith(OtaFault::NOT_AUTHORIZED);

  // --- Metadata validity -------------------------------------------------
  if (metadata.schema_version != kOtaMetadataSchema) {
    return failWith(OtaFault::METADATA_SCHEMA);
  }
  if (!buildIdLooksValid(metadata.build_id)) {
    return failWith(OtaFault::METADATA_INCOMPLETE);
  }
  if (metadata.image_size == 0) {
    return failWith(OtaFault::IMAGE_EMPTY);
  }
  // An all-zero digest is the shape an uninitialised or omitted hash field
  // takes. Treat it as absent rather than as a hash that happens to be zero.
  bool sha_present = false;
  for (size_t i = 0; i < kSha256DigestBytes; ++i) {
    if (metadata.sha256[i] != 0) { sha_present = true; break; }
  }
  if (!sha_present) return failWith(OtaFault::METADATA_INCOMPLETE);

  status_.declared_size = metadata.image_size;
  copyBounded(status_.declared_build_id, sizeof(status_.declared_build_id),
              metadata.build_id);
  for (size_t i = 0; i < kSha256DigestBytes; ++i) expected_sha_[i] = metadata.sha256[i];
  toHex(expected_sha_, status_.declared_sha256, sizeof(status_.declared_sha256));

  // --- Running image must be confirmed before a new update --------------
  // Verified against the installed ESP-IDF v5.5.5 header: esp_ota_begin()
  // returns ESP_ERR_OTA_ROLLBACK_INVALID_STATE when rollback is enabled and
  // the running app is still ESP_OTA_IMG_PENDING_VERIFY. The real build has
  // CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y, so this is not hypothetical.
  // Refusing here gives a precise reason instead of an opaque backend error.
  refreshPartitionFacts();
  if (status_.running_image_state == OtaImgState::PENDING_VERIFY ||
      status_.running_image_state == OtaImgState::NEW) {
    return failWith(OtaFault::RUNNING_IMAGE_UNCONFIRMED);
  }

  // --- Target resolution, then the three explicit checks ----------------
  const OtaPartitionInfo target = backend_->nextUpdatePartition();
  if (!target.valid) return failWith(OtaFault::NO_INACTIVE_SLOT);

  // These three are stated separately, in this order, on purpose. The
  // handoff requires each to be an explicit check rather than something
  // inferred from the backend happening to refuse.
  if (target.sameAs(status_.running)) return failWith(OtaFault::TARGET_IS_RUNNING);
  if (!target.isOtaAppSlot())         return failWith(OtaFault::TARGET_NOT_OTA_SLOT);
  if (metadata.image_size > target.size) return failWith(OtaFault::IMAGE_TOO_LARGE);

  status_.target = target;
  status_.state = OtaState::TARGET_RESOLVED;
  return true;
}

bool OtaPolicy::openStream() {
  if (backend_ == nullptr) return failWith(OtaFault::WRONG_STATE);
  if (status_.state != OtaState::TARGET_RESOLVED) {
    status_.fault = OtaFault::WRONG_STATE;
    return false;
  }

  // Re-check the target immediately before the first byte of flash is
  // touched. prepare() may have run many seconds earlier; this is the same
  // discipline the DALY KEY write uses with its pre-transmit re-check.
  if (status_.target.sameAs(status_.running)) return failWith(OtaFault::TARGET_IS_RUNNING);
  if (!status_.target.isOtaAppSlot())         return failWith(OtaFault::TARGET_NOT_OTA_SLOT);

  if (!backend_->beginWrite(status_.target, status_.declared_size)) {
    return failWith(OtaFault::WRITE_OPEN_REJECTED);
  }

  hash_.reset();
  status_.bytes_written = 0;
  status_.chunks_written = 0;
  status_.state = OtaState::RECEIVING;
  return true;
}

bool OtaPolicy::writeChunk(const uint8_t* data, uint32_t len) {
  if (backend_ == nullptr) return failWith(OtaFault::WRONG_STATE);
  if (status_.state != OtaState::RECEIVING) {
    status_.fault = OtaFault::WRONG_STATE;
    return false;
  }
  if (len == 0) return true;  // a zero-length chunk is not an error, just nothing
  if (data == nullptr) return failWith(OtaFault::WRITE_FAILED);

  // Overrun is rejected BEFORE the write, so an oversized stream can never
  // put a single extra byte into the slot.
  if (len > status_.declared_size - status_.bytes_written) {
    return failWith(OtaFault::OVERRUN);
  }

  if (!backend_->write(data, len)) return failWith(OtaFault::WRITE_FAILED);

  hash_.update(data, len);
  status_.bytes_written += len;
  status_.chunks_written++;
  return true;
}

bool OtaPolicy::finishStream() {
  if (backend_ == nullptr) return failWith(OtaFault::WRONG_STATE);
  if (status_.state != OtaState::RECEIVING) {
    status_.fault = OtaFault::WRONG_STATE;
    return false;
  }

  // Stream completeness is OUR check, and it runs before the backend's:
  // a truncated-but-structurally-plausible image must be refused by us, not
  // hoped to be caught downstream.
  if (status_.bytes_written != status_.declared_size) {
    return failWith(OtaFault::INCOMPLETE_STREAM);
  }

  // (1) image validity - the ESP-IDF image structure check inside
  //     esp_ota_end(). This is NOT the same as our hash: it proves the bytes
  //     form a loadable app image, not that they are the app we asked for.
  if (!backend_->endWrite()) return failWith(OtaFault::IMAGE_REJECTED);
  status_.state = OtaState::IMAGE_SEALED;

  // (2) cryptographic hash identity - the bytes we received are exactly the
  //     bytes the sender declared. This is the check that distinguishes "a
  //     valid image" from "the expected image".
  uint8_t computed[kSha256DigestBytes];
  hash_.finish(computed);
  toHex(computed, status_.computed_sha256, sizeof(status_.computed_sha256));
  if (!digestsEqual(computed, expected_sha_)) return failWith(OtaFault::HASH_MISMATCH);

  status_.state = OtaState::IDENTITY_VERIFIED;
  return true;
}

bool OtaPolicy::commitBootTarget() {
  if (backend_ == nullptr) return failWith(OtaFault::WRONG_STATE);

  // The whole safety argument of OTA-A rests on this one guard: the boot
  // target can be changed from exactly one state, which is only reachable
  // after the stream completed, the image passed the ESP-IDF structure check
  // and the hash matched.
  if (status_.state != OtaState::IDENTITY_VERIFIED) {
    status_.fault = OtaFault::WRONG_STATE;
    return false;
  }

  if (!backend_->setBootPartition(status_.target)) {
    return failWith(OtaFault::BOOT_SWITCH_REJECTED);
  }

  status_.boot_target_changed = true;
  status_.state = OtaState::BOOT_TARGET_SET;
  status_.counters.updates_committed++;
  // The exclusivity hold is deliberately KEPT here. A boot switch is pending
  // a reboot, and starting a calibration against an image that is about to be
  // replaced is not something to permit for convenience. The reboot clears it.
  return true;
}

void OtaPolicy::abort() {
  // Idempotent by construction. Note what abort does NOT do: once the boot
  // target has been set, aborting cannot take it back. Undoing it would mean
  // writing otadata again to point at the old slot, which is a second boot
  // switch dressed up as a cancellation - and the new image is already
  // validated, so there is nothing unsafe to undo. The pending switch stays
  // reported.
  if (status_.state == OtaState::BOOT_TARGET_SET) return;
  if (status_.state == OtaState::IDLE) return;

  if (backend_ != nullptr) backend_->abortWrite();
  if (gate_ != nullptr) gate_->endExclusive();
  resetTransfer();
  status_.state = OtaState::IDLE;
  status_.fault = OtaFault::NONE;
  status_.counters.updates_aborted++;
}

bool OtaPolicy::reset() {
  if (status_.state == OtaState::BOOT_TARGET_SET) {
    status_.fault = OtaFault::WRONG_STATE;
    return false;
  }
  if (backend_ != nullptr) backend_->abortWrite();
  if (gate_ != nullptr) gate_->endExclusive();
  resetTransfer();
  status_.state = OtaState::IDLE;
  status_.fault = OtaFault::NONE;
  return true;
}

// ---------------------------------------------------------------------------

const char* toString(OtaState state) {
  switch (state) {
    case OtaState::IDLE:              return "IDLE";
    case OtaState::TARGET_RESOLVED:   return "TARGET_RESOLVED";
    case OtaState::RECEIVING:         return "RECEIVING";
    case OtaState::IMAGE_SEALED:      return "IMAGE_SEALED";
    case OtaState::IDENTITY_VERIFIED: return "IDENTITY_VERIFIED";
    case OtaState::BOOT_TARGET_SET:   return "BOOT_TARGET_SET";
    case OtaState::FAILED:            return "FAILED";
  }
  return "UNKNOWN";
}

const char* toString(OtaFault fault) {
  switch (fault) {
    case OtaFault::NONE:                      return "NONE";
    case OtaFault::WRONG_STATE:               return "WRONG_STATE";
    case OtaFault::NOT_AUTHORIZED:            return "NOT_AUTHORIZED";
    case OtaFault::METADATA_SCHEMA:           return "METADATA_SCHEMA";
    case OtaFault::METADATA_INCOMPLETE:       return "METADATA_INCOMPLETE";
    case OtaFault::IMAGE_EMPTY:               return "IMAGE_EMPTY";
    case OtaFault::RUNNING_IMAGE_UNCONFIRMED: return "RUNNING_IMAGE_UNCONFIRMED";
    case OtaFault::NO_INACTIVE_SLOT:          return "NO_INACTIVE_SLOT";
    case OtaFault::TARGET_IS_RUNNING:         return "TARGET_IS_RUNNING";
    case OtaFault::TARGET_NOT_OTA_SLOT:       return "TARGET_NOT_OTA_SLOT";
    case OtaFault::IMAGE_TOO_LARGE:           return "IMAGE_TOO_LARGE";
    case OtaFault::WRITE_OPEN_REJECTED:       return "WRITE_OPEN_REJECTED";
    case OtaFault::WRITE_FAILED:              return "WRITE_FAILED";
    case OtaFault::OVERRUN:                   return "OVERRUN";
    case OtaFault::INCOMPLETE_STREAM:         return "INCOMPLETE_STREAM";
    case OtaFault::IMAGE_REJECTED:            return "IMAGE_REJECTED";
    case OtaFault::HASH_MISMATCH:             return "HASH_MISMATCH";
    case OtaFault::BOOT_SWITCH_REJECTED:      return "BOOT_SWITCH_REJECTED";
  }
  return "UNKNOWN";
}

const char* toString(OtaImgState state) {
  switch (state) {
    case OtaImgState::NEW:            return "NEW";
    case OtaImgState::PENDING_VERIFY: return "PENDING_VERIFY";
    case OtaImgState::VALID:          return "VALID";
    case OtaImgState::INVALID:        return "INVALID";
    case OtaImgState::ABORTED:        return "ABORTED";
    case OtaImgState::UNDEFINED:      return "UNDEFINED";
    case OtaImgState::UNREADABLE:     return "UNREADABLE";
  }
  return "UNKNOWN";
}

const char* toString(OtaGateVerdict verdict) {
  switch (verdict) {
    case OtaGateVerdict::PERMITTED_BY_AUTHORITY:        return "PERMITTED_BY_AUTHORITY";
    case OtaGateVerdict::REFUSED_NO_GATE_INSTALLED:     return "REFUSED_NO_GATE_INSTALLED";
    case OtaGateVerdict::REFUSED_ACTUATOR_OWNER_ACTIVE: return "REFUSED_ACTUATOR_OWNER_ACTIVE";
    case OtaGateVerdict::REFUSED_ALREADY_EXCLUSIVE:     return "REFUSED_ALREADY_EXCLUSIVE";
    case OtaGateVerdict::REFUSED_BY_AUTHORITY:          return "REFUSED_BY_AUTHORITY";
  }
  return "UNKNOWN";
}

}  // namespace update
}  // namespace matdog
