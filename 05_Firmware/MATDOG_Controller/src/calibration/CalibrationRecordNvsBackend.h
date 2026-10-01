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
// If NVS is not initialized every call returns IO_ERROR (fail closed).

namespace matdog {
namespace calibration {

class CalibrationRecordNvsBackend : public CalibrationRecordStorage {
 public:
  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity,
                       size_t* length) override;
  // set_blob + nvs_commit; OK only after the commit succeeded.
  StorageIoStatus write(CalibrationSlot slot, const uint8_t* data, size_t length) override;
};

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_RECORD_NVS_BACKEND_H
