#ifndef MATDOG_TESTS_NVS_FLASH_STUB_H
#define MATDOG_TESTS_NVS_FLASH_STUB_H

// Host stand-in for ESP-IDF <nvs_flash.h>. Only the by-label initialization is
// declared. nvs_flash_init() (default partition), nvs_flash_erase(),
// nvs_flash_erase_partition() and nvs_flash_deinit*() are deliberately absent:
// a backend that called them would not compile.

#include "nvs.h"

esp_err_t nvs_flash_init_partition(const char* partition_label);

#endif  // MATDOG_TESTS_NVS_FLASH_STUB_H
