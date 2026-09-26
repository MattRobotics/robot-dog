#include "OtaEspBackend.h"

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// Audited against the ESP-IDF headers actually installed with esp32:esp32
// 3.3.11 (esp32s3-libs/3.3.11/include, ESP-IDF v5.5.5), not against memory:
//
//   esp_ota_get_running_partition()  returns the app that is really running,
//     ignoring any pending esp_ota_set_boot_partition(); may differ from
//     esp_ota_get_boot_partition() if the bootloader fell back.
//   esp_ota_get_next_update_partition(NULL)  round-robin from the running
//     partition; "the result of this function is never the same as this
//     argument". NULL means invalid otadata or no eligible slot.
//   esp_ota_begin()  ERASES the target. With an explicit size the whole
//     range is erased up front - seconds of blocking for a ~1 MB image. We
//     pass OTA_WITH_SEQUENTIAL_WRITES instead so the erase is incremental,
//     one sector at a time inside esp_ota_write(). The size bound is
//     enforced by OtaPolicy, not delegated to the erase argument.
//     It also returns ESP_ERR_OTA_ROLLBACK_INVALID_STATE when the running
//     app is still PENDING_VERIFY - OtaPolicy refuses earlier, with a
//     clearer reason.
//   esp_ota_end()    closes the stream AND validates the image structure;
//     ESP_ERR_OTA_VALIDATE_FAILED means "not a valid app image".
//   esp_ota_abort()  frees the handle; only valid for an open handle.

namespace matdog {
namespace update {
namespace {

OtaPartitionInfo describe(const esp_partition_t* p) {
  OtaPartitionInfo info{};
  if (p == nullptr) return info;
  info.valid = true;
  info.address = p->address;
  info.size = p->size;
  info.subtype = static_cast<uint8_t>(p->subtype);
  size_t i = 0;
  while (i + 1 < sizeof(info.label) && p->label[i] != '\0') {
    info.label[i] = p->label[i];
    ++i;
  }
  info.label[i] = '\0';
  return info;
}

// Resolves our address-keyed description back to the real partition record.
// Never trusts a caller-supplied address blindly: it must match a partition
// the ESP-IDF partition table actually reports.
const esp_partition_t* lookup(const OtaPartitionInfo& info) {
  if (!info.valid) return nullptr;
  esp_partition_iterator_t it =
      esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  const esp_partition_t* found = nullptr;
  while (it != nullptr) {
    const esp_partition_t* p = esp_partition_get(it);
    if (p != nullptr && p->address == info.address && p->size == info.size) {
      found = p;
      break;
    }
    it = esp_partition_next(it);
  }
  if (it != nullptr) esp_partition_iterator_release(it);
  return found;
}

OtaImgState translate(esp_ota_img_states_t s) {
  switch (s) {
    case ESP_OTA_IMG_NEW:            return OtaImgState::NEW;
    case ESP_OTA_IMG_PENDING_VERIFY: return OtaImgState::PENDING_VERIFY;
    case ESP_OTA_IMG_VALID:          return OtaImgState::VALID;
    case ESP_OTA_IMG_INVALID:        return OtaImgState::INVALID;
    case ESP_OTA_IMG_ABORTED:        return OtaImgState::ABORTED;
    case ESP_OTA_IMG_UNDEFINED:      return OtaImgState::UNDEFINED;
    default:                         return OtaImgState::UNREADABLE;
  }
}

}  // namespace

OtaPartitionInfo OtaEspBackend::runningPartition() {
  return describe(esp_ota_get_running_partition());
}

OtaPartitionInfo OtaEspBackend::nextUpdatePartition() {
  // NULL start_from means "use the running partition", and the API
  // guarantees the result is never that partition.
  return describe(esp_ota_get_next_update_partition(nullptr));
}

OtaImgState OtaEspBackend::imageState(const OtaPartitionInfo& partition) {
  const esp_partition_t* p = lookup(partition);
  if (p == nullptr) return OtaImgState::UNREADABLE;
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  const esp_err_t err = esp_ota_get_state_partition(p, &state);
  if (err == ESP_ERR_NOT_FOUND) {
    // No otadata record for this slot. That is not an error: it is a factory
    // or first image, and there is nothing pending verification.
    return OtaImgState::UNDEFINED;
  }
  if (err != ESP_OK) {
    last_error_ = static_cast<int32_t>(err);
    return OtaImgState::UNREADABLE;
  }
  return translate(state);
}

bool OtaEspBackend::beginWrite(const OtaPartitionInfo& target, uint32_t image_size) {
  (void)image_size;  // bound already enforced by OtaPolicy; see the note above
  if (stream_open_) return false;

  const esp_partition_t* p = lookup(target);
  if (p == nullptr) return false;

  const uint32_t t0 = micros();
  const esp_err_t err = esp_ota_begin(p, OTA_WITH_SEQUENTIAL_WRITES, &handle_);
  const uint32_t dt = micros() - t0;
  if (dt > max_open_us_) max_open_us_ = dt;

  if (err != ESP_OK) {
    last_error_ = static_cast<int32_t>(err);
    handle_ = 0;
    return false;
  }
  stream_open_ = true;
  return true;
}

bool OtaEspBackend::write(const uint8_t* data, uint32_t len) {
  if (!stream_open_ || data == nullptr) return false;

  const uint32_t t0 = micros();
  const esp_err_t err = esp_ota_write(handle_, data, len);
  const uint32_t dt = micros() - t0;
  if (dt > max_write_us_) max_write_us_ = dt;

  if (err != ESP_OK) {
    last_error_ = static_cast<int32_t>(err);
    return false;
  }
  return true;
}

bool OtaEspBackend::endWrite() {
  if (!stream_open_) return false;

  const uint32_t t0 = micros();
  const esp_err_t err = esp_ota_end(handle_);
  const uint32_t dt = micros() - t0;
  if (dt > max_end_us_) max_end_us_ = dt;

  // The handle is freed by esp_ota_end() regardless of result.
  stream_open_ = false;
  handle_ = 0;
  if (err != ESP_OK) {
    last_error_ = static_cast<int32_t>(err);
    return false;
  }
  return true;
}

void OtaEspBackend::abortWrite() {
  // Must be safe with nothing open: OtaPolicy::abort() is idempotent and
  // calls this from any state.
  if (!stream_open_) return;
  const esp_err_t err = esp_ota_abort(handle_);
  if (err != ESP_OK) last_error_ = static_cast<int32_t>(err);
  stream_open_ = false;
  handle_ = 0;
}

bool OtaEspBackend::setBootPartition(const OtaPartitionInfo& target) {
  const esp_partition_t* p = lookup(target);
  if (p == nullptr) return false;

  // Last line of defence, independent of OtaPolicy: never point the boot
  // target at the partition we are executing from.
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running != nullptr && running->address == p->address) return false;

  const esp_err_t err = esp_ota_set_boot_partition(p);
  if (err != ESP_OK) {
    last_error_ = static_cast<int32_t>(err);
    return false;
  }
  return true;
}

bool OtaEspBackend::markAppValid() {
  const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err != ESP_OK) {
    last_error_ = static_cast<int32_t>(err);
    return false;
  }
  return true;
}

bool OtaEspBackend::rollbackPossible() { return esp_ota_check_rollback_is_possible(); }

}  // namespace update
}  // namespace matdog
