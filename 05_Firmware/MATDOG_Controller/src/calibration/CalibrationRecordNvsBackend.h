#ifndef MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H
#define MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H

#include "CalibrationRecordStore.h"

// The ESP-IDF half of Calibration Persistence: the only translation unit that
// calls nvs_*. <nvs.h> is included in the .cpp only, so the pure record/store
// code and the Controller headers stay host-testable.
//
// Storage: native NVS, dedicated namespace "matdog_calrec", two blobs "A" / "B".
//
// WHAT THIS BACKEND NEVER DOES: nvs_flash_init(), nvs_flash_erase(),
// nvs_erase_all(), nvs_erase_key(). It never pre-erases the previous slot.
//
// KNOWN PLATFORM RISK (Arduino-ESP32 3.3.11, cores/esp32/esp32-hal-misc.c,
// initArduino): at boot the core calls nvs_flash_init() and, on
// ESP_ERR_NVS_NO_FREE_PAGES or ESP_ERR_NVS_NEW_VERSION_FOUND, ERASES THE WHOLE
// NVS PARTITION and re-initializes it. If that happens the calibration record
// is lost together with everything else in NVS (Wi-Fi credentials etc.). The
// store then reports NOT_FOUND and a recalibration is required: fail-closed,
// never a wrong calibration. This is a consequence of the platform and is
// documented, not mitigated here (core and partition table are out of scope).
// Mitigation belongs to a later change, e.g. a dedicated partition.
//
// If NVS is not initialized read() returns IO_ERROR (fail closed) and write()
// returns NOT_MODIFIED (nothing was touched).
//
// WRITE FAULT MODEL. nvs_set_blob() is not all-or-nothing from the caller's
// point of view. In ESP-IDF it can fail with ESP_ERR_NVS_REMOVE_FAILED ("the
// value wasn't updated because flash write operation has failed. The value was
// written however, and update will be finished after re-initialization of nvs"):
// the new blob may be fully or partly published although an error is returned,
// and a later nvs_flash_init() may complete or discard it. nvs_commit() can
// fail after a published set. The backend cannot tell these apart from errors
// raised before anything was written, so every error from nvs_set_blob() or
// nvs_commit() is returned as IO_ERROR / NO_SPACE, which the store treats as
// "slot content unknown" and answers with a session-local write block. Only a
// failure to open the namespace or invalid arguments are reported as
// NOT_MODIFIED. nvs_commit() is kept as the API requires; neither it nor any
// host stub proves physical durability.

namespace matdog {
namespace calibration {

class CalibrationRecordNvsBackend : public CalibrationRecordStorage {
 public:
  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity,
                       size_t* length) override;
  // set_blob + nvs_commit; OK only after the commit succeeded. NOT_MODIFIED
  // only when failing before nvs_set_blob; any later error leaves the slot unknown.
  StorageIoStatus write(CalibrationSlot slot, const uint8_t* data, size_t length) override;
};

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H
