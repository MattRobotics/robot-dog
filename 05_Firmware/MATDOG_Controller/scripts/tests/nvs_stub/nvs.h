#ifndef MATDOG_TESTS_NVS_STUB_H
#define MATDOG_TESTS_NVS_STUB_H

// Host stand-in for ESP-IDF <nvs.h>: only the entry points the calibration NVS
// backend may call. nvs_flash_init / nvs_flash_erase / nvs_erase_* are
// deliberately NOT declared, so a backend that used one would not compile.

#include <stddef.h>
#include <stdint.h>

typedef int esp_err_t;
typedef uint32_t nvs_handle_t;

#define ESP_OK 0
#define ESP_ERR_NVS_BASE 0x1100
#define ESP_ERR_NVS_NOT_INITIALIZED (ESP_ERR_NVS_BASE + 0x01)
#define ESP_ERR_NVS_NOT_FOUND (ESP_ERR_NVS_BASE + 0x02)
#define ESP_ERR_NVS_NOT_ENOUGH_SPACE (ESP_ERR_NVS_BASE + 0x05)
#define ESP_ERR_NVS_REMOVE_FAILED (ESP_ERR_NVS_BASE + 0x08)
#define ESP_ERR_NVS_PAGE_FULL (ESP_ERR_NVS_BASE + 0x0a)
#define ESP_ERR_NVS_INVALID_LENGTH (ESP_ERR_NVS_BASE + 0x0c)

typedef enum { NVS_READONLY, NVS_READWRITE } nvs_open_mode_t;

esp_err_t nvs_open(const char* name, nvs_open_mode_t mode, nvs_handle_t* handle);
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out, size_t* length);
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);
esp_err_t nvs_commit(nvs_handle_t handle);
void nvs_close(nvs_handle_t handle);

#endif  // MATDOG_TESTS_NVS_STUB_H
