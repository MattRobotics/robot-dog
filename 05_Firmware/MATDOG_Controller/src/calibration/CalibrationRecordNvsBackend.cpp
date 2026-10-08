#include "CalibrationRecordNvsBackend.h"

#include <esp_partition.h>
#include <nvs.h>
#include <nvs_flash.h>

namespace matdog {
namespace calibration {

namespace {

constexpr const char* kNamespace = "matdog_calrec";
constexpr const char* kMarkerKey = "M";

const char* keyOf(CalibrationSlot slot) { return slot == CalibrationSlot::A ? "A" : "B"; }

StorageIoStatus mapWriteError(esp_err_t err) {
  return err == ESP_ERR_NVS_NOT_ENOUGH_SPACE || err == ESP_ERR_NVS_PAGE_FULL
             ? StorageIoStatus::NO_SPACE
             : StorageIoStatus::IO_ERROR;
}

}  // namespace

const char* toString(NvsInitStatus status) {
  switch (status) {
    case NvsInitStatus::NOT_INITIALIZED:           return "NOT_INITIALIZED";
    case NvsInitStatus::READY:                     return "READY";
    case NvsInitStatus::PARTITION_MISSING:         return "PARTITION_MISSING";
    case NvsInitStatus::PARTITION_GEOMETRY_MISMATCH: return "PARTITION_GEOMETRY_MISMATCH";
    case NvsInitStatus::NO_FREE_PAGES:             return "NO_FREE_PAGES";
    case NvsInitStatus::NEW_VERSION_FOUND:         return "NEW_VERSION_FOUND";
    case NvsInitStatus::INIT_FAILED:               return "INIT_FAILED";
    case NvsInitStatus::OPEN_FAILED:               return "OPEN_FAILED";
  }
  return "UNKNOWN";
}

NvsInitStatus CalibrationRecordNvsBackend::begin() {
  if (init_status_ == NvsInitStatus::READY) return init_status_;
  last_esp_error_ = 0;

  const esp_partition_t* part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, kMatdogNvsPartitionLabel);
  if (part == nullptr) {
    init_status_ = NvsInitStatus::PARTITION_MISSING;
    return init_status_;
  }
  if (part->address != kMatdogNvsPartitionAddress || part->size != kMatdogNvsPartitionSize) {
    init_status_ = NvsInitStatus::PARTITION_GEOMETRY_MISMATCH;
    return init_status_;
  }

  // By label, and only this partition. Any error is reported as it is: this
  // code never erases and never retries.
  esp_err_t err = nvs_flash_init_partition(kMatdogNvsPartitionLabel);
  last_esp_error_ = err;
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    init_status_ = NvsInitStatus::NO_FREE_PAGES;
    return init_status_;
  }
  if (err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    init_status_ = NvsInitStatus::NEW_VERSION_FOUND;
    return init_status_;
  }
  if (err == ESP_ERR_NVS_PART_NOT_FOUND || err == ESP_ERR_NOT_FOUND) {
    init_status_ = NvsInitStatus::PARTITION_MISSING;
    return init_status_;
  }
  if (err != ESP_OK) {
    init_status_ = NvsInitStatus::INIT_FAILED;
    return init_status_;
  }

  // Read-only probe: a missing namespace is the normal state of a first install.
  nvs_handle_t handle = 0;
  err = nvs_open_from_partition(kMatdogNvsPartitionLabel, kNamespace, NVS_READONLY, &handle);
  if (err == ESP_OK) {
    nvs_close(handle);
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    last_esp_error_ = err;
    init_status_ = NvsInitStatus::OPEN_FAILED;
    return init_status_;
  }
  last_esp_error_ = 0;
  init_status_ = NvsInitStatus::READY;
  return init_status_;
}

StorageIoStatus CalibrationRecordNvsBackend::readKey(const char* key, uint8_t* buffer,
                                                     size_t capacity, size_t* length) {
  if (buffer == nullptr || length == nullptr) return StorageIoStatus::IO_ERROR;
  *length = 0;
  if (init_status_ != NvsInitStatus::READY) return StorageIoStatus::IO_ERROR;

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open_from_partition(kMatdogNvsPartitionLabel, kNamespace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) return StorageIoStatus::ABSENT;  // namespace never created
  if (err != ESP_OK) return StorageIoStatus::IO_ERROR;

  size_t stored = 0;
  err = nvs_get_blob(handle, key, nullptr, &stored);
  StorageIoStatus result = StorageIoStatus::OK;
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    result = StorageIoStatus::ABSENT;
  } else if (err != ESP_OK) {
    result = StorageIoStatus::IO_ERROR;
  } else if (stored > capacity) {
    result = StorageIoStatus::BUFFER_TOO_SMALL;
  } else {
    size_t got = stored;
    err = nvs_get_blob(handle, key, buffer, &got);
    if (err != ESP_OK || got != stored) {
      result = StorageIoStatus::IO_ERROR;
    } else {
      *length = got;
    }
  }
  nvs_close(handle);
  return result;
}

StorageIoStatus CalibrationRecordNvsBackend::writeKey(const char* key, const uint8_t* data,
                                                      size_t length) {
  if (data == nullptr || length == 0) return StorageIoStatus::NOT_MODIFIED;
  if (init_status_ != NvsInitStatus::READY) return StorageIoStatus::NOT_MODIFIED;

  // Failing to open touches nothing: the only error the store may retry past.
  // From nvs_set_blob onwards every error is uncertain.
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open_from_partition(kMatdogNvsPartitionLabel, kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) return StorageIoStatus::NOT_MODIFIED;

  err = nvs_set_blob(handle, key, data, length);
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  return err == ESP_OK ? StorageIoStatus::OK : mapWriteError(err);
}

StorageIoStatus CalibrationRecordNvsBackend::read(CalibrationSlot slot, uint8_t* buffer,
                                                  size_t capacity, size_t* length) {
  return readKey(keyOf(slot), buffer, capacity, length);
}

StorageIoStatus CalibrationRecordNvsBackend::write(CalibrationSlot slot, const uint8_t* data,
                                                   size_t length) {
  return writeKey(keyOf(slot), data, length);
}

StorageIoStatus CalibrationRecordNvsBackend::readMarker(uint8_t* buffer, size_t capacity,
                                                        size_t* length) {
  return readKey(kMarkerKey, buffer, capacity, length);
}

StorageIoStatus CalibrationRecordNvsBackend::writeMarker(const uint8_t* data, size_t length) {
  return writeKey(kMarkerKey, data, length);
}

}  // namespace calibration
}  // namespace matdog
