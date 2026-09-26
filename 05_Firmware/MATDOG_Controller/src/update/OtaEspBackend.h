#ifndef MATDOG_UPDATE_OTA_ESP_BACKEND_H
#define MATDOG_UPDATE_OTA_ESP_BACKEND_H

#include <stdint.h>

#include "OtaPolicy.h"

// The ESP-IDF half of OTA-A: the only translation unit in the firmware that
// calls esp_ota_* / esp_partition_*. Everything above it (OtaPolicy,
// OtaBootGuard) is ESP-IDF-free and therefore host-testable.
//
// Deliberately does NOT include <esp_ota_ops.h> here: keeping the ESP-IDF
// headers inside the .cpp means core/Controller.h and core/CommandRouter.h
// can own and read the OTA subsystem without pulling them in, exactly as
// network/WifiManager.h does for <WiFi.h>.

namespace matdog {
namespace update {

class OtaEspBackend : public OtaBackend {
 public:
  OtaPartitionInfo runningPartition() override;
  OtaPartitionInfo nextUpdatePartition() override;
  OtaImgState imageState(const OtaPartitionInfo& partition) override;

  bool beginWrite(const OtaPartitionInfo& target, uint32_t image_size) override;
  bool write(const uint8_t* data, uint32_t len) override;
  bool endWrite() override;
  void abortWrite() override;

  bool setBootPartition(const OtaPartitionInfo& target) override;

  bool markAppValid() override;
  bool rollbackPossible() override;

  // Measured, not claimed - the same discipline as the Wi-Fi tick. Flash
  // erase and write DO block the Controller loop; these say by how much, so
  // the OTA-B integration can argue from numbers instead of adjectives.
  uint32_t maxOpenUs() const { return max_open_us_; }
  uint32_t maxWriteUs() const { return max_write_us_; }
  uint32_t maxEndUs() const { return max_end_us_; }
  bool streamOpen() const { return stream_open_; }
  // Raw esp_err_t of the last failing call, for diagnostics.
  int32_t lastError() const { return last_error_; }

 private:
  uint32_t handle_ = 0;
  bool stream_open_ = false;
  int32_t last_error_ = 0;
  uint32_t max_open_us_ = 0;
  uint32_t max_write_us_ = 0;
  uint32_t max_end_us_ = 0;
};

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_ESP_BACKEND_H
