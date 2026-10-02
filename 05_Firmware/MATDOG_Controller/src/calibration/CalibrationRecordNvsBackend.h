#ifndef MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H
#define MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H

#include "CalibrationRecordStore.h"

// The ESP-IDF half of Calibration Persistence: the only translation unit that
// calls nvs_*. <nvs.h> is included in the .cpp only, so the pure record/store
// code and the Controller headers stay host-testable.
//
// STORAGE (P2.4). The dedicated NVS partition "matdog_nvs" of the P2.3 layout
// (data/nvs, offset 0xFE0000, size 0x10000), namespace "matdog_calrec", three
// blobs: "A" / "B" (record slots) and "M" (save marker). The default "nvs"
// partition is never touched, so the Arduino core's own NVS handling (Wi-Fi,
// Preferences) cannot reach calibration data.
//
// EXPLICIT INITIALIZATION. begin() must be called once before the store is used.
// It checks that the partition exists with the expected geometry, initializes
// ONLY that partition by label and probes the namespace read-only. The outcome
// is a NvsInitStatus the future Controller can check; begin() never retries and
// never repairs. Until begin() returned READY every read is IO_ERROR and every
// write is NOT_MODIFIED (fail closed).
//
// WHAT THIS BACKEND NEVER DOES: initialize the default NVS partition, erase or
// format anything (whole partition, namespace, key), pre-erase the previous
// slot, or react to NO_FREE_PAGES / NEW_VERSION_FOUND / NOT_FOUND / corruption
// by reformatting. NVS libraries may repair their own pages while initializing a
// partition (interrupted page transitions); that is internal to ESP-IDF, cannot
// be observed from here and is not proven by the host stub.
//
// WRITE FAULT MODEL. nvs_set_blob() is not all-or-nothing from the caller's
// point of view. In ESP-IDF it can fail with ESP_ERR_NVS_REMOVE_FAILED ("the
// value wasn't updated because flash write operation has failed. The value was
// written however, and update will be finished after re-initialization of nvs"):
// the new blob may be fully or partly published although an error is returned,
// and a later initialization may complete or discard it. nvs_commit() can fail
// after a published set. The backend cannot tell these apart from errors raised
// before anything was written, so every error from nvs_set_blob() or
// nvs_commit() is returned as IO_ERROR / NO_SPACE, which the store treats as
// "content unknown" and answers with a session-local write block. Only a failure
// to open the namespace, an uninitialized backend or invalid arguments are
// reported as NOT_MODIFIED. nvs_commit() is kept as the API requires; neither it
// nor any host stub proves physical durability.

namespace matdog {
namespace calibration {

enum class NvsInitStatus : uint8_t {
  NOT_INITIALIZED = 0,         // begin() not called yet
  READY,                       // partition initialized, namespace readable or not yet created
  PARTITION_MISSING,           // no partition labelled "matdog_nvs" (wrong partition table)
  PARTITION_GEOMETRY_MISMATCH, // found, but not data/nvs at 0xFE0000 size 0x10000
  NO_FREE_PAGES,               // NOT erased; explicit intervention needed
  NEW_VERSION_FOUND,           // NOT erased; explicit intervention needed
  INIT_FAILED,                 // any other nvs_flash_init_partition error (see lastEspError)
  OPEN_FAILED,                 // initialized, but the namespace probe failed (see lastEspError)
};
const char* toString(NvsInitStatus status);

constexpr const char* kMatdogNvsPartitionLabel = "matdog_nvs";
constexpr uint32_t kMatdogNvsPartitionAddress = 0xFE0000u;
constexpr uint32_t kMatdogNvsPartitionSize = 0x10000u;

class CalibrationRecordNvsBackend : public CalibrationRecordStorage {
 public:
  // Idempotent, never erases. Returns the same value as initStatus().
  NvsInitStatus begin();
  NvsInitStatus initStatus() const { return init_status_; }
  // Raw esp_err_t of the call that decided initStatus() (0 = none/OK).
  int32_t lastEspError() const { return last_esp_error_; }

  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity,
                       size_t* length) override;
  // set_blob + nvs_commit; OK only after the commit succeeded. NOT_MODIFIED
  // only when failing before nvs_set_blob; any later error leaves the slot unknown.
  StorageIoStatus write(CalibrationSlot slot, const uint8_t* data, size_t length) override;
  StorageIoStatus readMarker(uint8_t* buffer, size_t capacity, size_t* length) override;
  StorageIoStatus writeMarker(const uint8_t* data, size_t length) override;

 private:
  StorageIoStatus readKey(const char* key, uint8_t* buffer, size_t capacity, size_t* length);
  StorageIoStatus writeKey(const char* key, const uint8_t* data, size_t length);

  NvsInitStatus init_status_ = NvsInitStatus::NOT_INITIALIZED;
  int32_t last_esp_error_ = 0;
};

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H
