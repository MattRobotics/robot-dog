#include "router_nvs_stub.h"

#include <cstring>
#include <esp_partition.h>
#include <nvs.h>
#include <nvs_flash.h>

namespace router_nvs_test {
State state;
void reset() { state = State{}; }
}  // namespace router_nvs_test

namespace {
using router_nvs_test::state;
const esp_partition_t partition = {ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS,
                                  0xFE0000u, 0x10000u, "matdog_nvs"};
bool named(const char* actual, const char* expected) {
  if (actual != nullptr && std::strcmp(actual, expected) == 0) return true;
  ++state.contract_errors;
  return false;
}
}  // namespace

const esp_partition_t* esp_partition_find_first(esp_partition_type_t type,
                                               esp_partition_subtype_t subtype, const char* label) {
  if (!named(label, "matdog_nvs") || type != partition.type || subtype != partition.subtype) return nullptr;
  return &partition;
}

esp_err_t nvs_flash_init_partition(const char* label) {
  if (!named(label, "matdog_nvs")) return ESP_FAIL;
  state.initialized = true;
  return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char* label, const char* ns, nvs_open_mode_t mode,
                                  nvs_handle_t* handle) {
  if (!named(label, "matdog_nvs") || !named(ns, "matdog_calrec")) return ESP_FAIL;
  if (!state.initialized) return ESP_ERR_NVS_NOT_INITIALIZED;
  if (mode == NVS_READONLY && !state.namespace_present) return ESP_ERR_NVS_NOT_FOUND;
  state.opened_rw = mode == NVS_READWRITE;
  if (state.opened_rw) {
    ++state.rw_open_calls;
    state.namespace_present = true;
  }
  *handle = 1;
  return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t, const char* key, void* out, size_t* length) {
  const auto found = state.blobs.find(key);
  if (found == state.blobs.end()) return ESP_ERR_NVS_NOT_FOUND;
  if (out == nullptr) {
    *length = found->second.size();
    return ESP_OK;
  }
  if (*length < found->second.size()) return ESP_ERR_NVS_INVALID_LENGTH;
  std::memcpy(out, found->second.data(), found->second.size());
  *length = found->second.size();
  return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t, const char* key, const void* data, size_t length) {
  ++state.set_calls;
  if (!state.opened_rw) return ESP_ERR_NVS_READ_ONLY;
  const auto* bytes = static_cast<const uint8_t*>(data);
  state.blobs[key].assign(bytes, bytes + length);
  return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t) {
  ++state.commit_calls;
  return state.opened_rw ? ESP_OK : ESP_ERR_NVS_READ_ONLY;
}

void nvs_close(nvs_handle_t) { state.opened_rw = false; }
