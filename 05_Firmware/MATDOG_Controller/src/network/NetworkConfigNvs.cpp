#include "NetworkConfigNvs.h"

#include <nvs.h>
#include <string.h>
namespace matdog {
namespace network {
namespace {
constexpr char kNetworkPartition[] = "nvs";
constexpr char kNetworkNamespace[] = "md_net_v1";
} // namespace
bool NetworkConfigNvs::load(NetworkConfig *out) {
  nvs_handle_t handle;
  // Framework already initializes standard nvs. Never initialize, erase or
  // format here.
  load_error_ = nvs_open_from_partition(kNetworkPartition, kNetworkNamespace,
                                        NVS_READONLY, &handle);
  if (load_error_ != ESP_OK) {
    if (load_error_ == ESP_ERR_NVS_NOT_FOUND)
      load_error_ = 0;
    return false;
  }
  uint8_t record[kNetworkRecordBytes];
  size_t size = sizeof(record);
  const esp_err_t read = nvs_get_blob(handle, "active", record, &size);
  const bool ok = read == ESP_OK && decodeNetworkConfig(record, size, out);
  if (!ok && read != ESP_ERR_NVS_NOT_FOUND)
    load_error_ = read == ESP_OK ? -1 : read;
  nvs_close(handle);
  memset(record, 0, sizeof(record));
  return ok;
}
bool NetworkConfigNvs::write(const char *key, const NetworkConfig &c) {
  if (!validNetworkConfig(c))
    return false;
  nvs_handle_t handle;
  if (nvs_open_from_partition(kNetworkPartition, kNetworkNamespace,
                              NVS_READWRITE, &handle) != ESP_OK)
    return false;
  uint8_t record[kNetworkRecordBytes], readback[kNetworkRecordBytes];
  encodeNetworkConfig(c, record);
  size_t size = sizeof(readback);
  const bool ok = nvs_set_blob(handle, key, record, sizeof(record)) == ESP_OK &&
                  nvs_commit(handle) == ESP_OK &&
                  nvs_get_blob(handle, key, readback, &size) == ESP_OK &&
                  size == sizeof(record) && memcmp(record, readback, size) == 0;
  nvs_close(handle);
  memset(record, 0, sizeof(record));
  memset(readback, 0, sizeof(readback));
  return ok;
}
bool NetworkConfigNvs::pending(const NetworkConfig &c) {
  return write("pending", c);
}
bool NetworkConfigNvs::activate(const NetworkConfig &c) {
  return write("active", c);
}
} // namespace network
} // namespace matdog
