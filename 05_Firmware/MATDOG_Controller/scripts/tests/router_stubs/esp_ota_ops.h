#pragma once
#include <cstdint>
struct esp_partition_t { const char* label; uint32_t address; uint32_t size; };
inline const esp_partition_t* esp_ota_get_running_partition() { return nullptr; }
