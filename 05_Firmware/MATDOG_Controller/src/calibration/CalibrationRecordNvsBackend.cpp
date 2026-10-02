#include "CalibrationRecordNvsBackend.h"

#include <nvs.h>

namespace matdog {
namespace calibration {

namespace {

constexpr const char* kNamespace = "matdog_calrec";

const char* keyOf(CalibrationSlot slot) { return slot == CalibrationSlot::A ? "A" : "B"; }

StorageIoStatus mapWriteError(esp_err_t err) {
  return err == ESP_ERR_NVS_NOT_ENOUGH_SPACE || err == ESP_ERR_NVS_PAGE_FULL
             ? StorageIoStatus::NO_SPACE
             : StorageIoStatus::IO_ERROR;
}

}  // namespace

StorageIoStatus CalibrationRecordNvsBackend::read(CalibrationSlot slot, uint8_t* buffer,
                                                  size_t capacity, size_t* length) {
  if (buffer == nullptr || length == nullptr) return StorageIoStatus::IO_ERROR;
  *length = 0;

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) return StorageIoStatus::ABSENT;  // namespace never created
  if (err != ESP_OK) return StorageIoStatus::IO_ERROR;

  size_t stored = 0;
  err = nvs_get_blob(handle, keyOf(slot), nullptr, &stored);
  StorageIoStatus result = StorageIoStatus::OK;
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    result = StorageIoStatus::ABSENT;
  } else if (err != ESP_OK) {
    result = StorageIoStatus::IO_ERROR;
  } else if (stored > capacity) {
    result = StorageIoStatus::BUFFER_TOO_SMALL;
  } else {
    size_t got = stored;
    err = nvs_get_blob(handle, keyOf(slot), buffer, &got);
    if (err != ESP_OK || got != stored) {
      result = StorageIoStatus::IO_ERROR;
    } else {
      *length = got;
    }
  }
  nvs_close(handle);
  return result;
}

StorageIoStatus CalibrationRecordNvsBackend::write(CalibrationSlot slot, const uint8_t* data,
                                                   size_t length) {
  if (data == nullptr || length == 0) return StorageIoStatus::NOT_MODIFIED;

  // Failing to open touches no calibration slot: the only error the store may
  // retry past. From nvs_set_blob onwards every error is uncertain.
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) return StorageIoStatus::NOT_MODIFIED;

  err = nvs_set_blob(handle, keyOf(slot), data, length);
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  return err == ESP_OK ? StorageIoStatus::OK : mapWriteError(err);
}

}  // namespace calibration
}  // namespace matdog
