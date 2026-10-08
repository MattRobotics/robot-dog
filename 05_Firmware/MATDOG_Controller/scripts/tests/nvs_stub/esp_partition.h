#ifndef MATDOG_TESTS_ESP_PARTITION_STUB_H
#define MATDOG_TESTS_ESP_PARTITION_STUB_H

// Host stand-in for the read-only part of <esp_partition.h> the backend uses to
// check the partition table. Every esp_partition_erase_range / write / mmap
// entry point is deliberately absent.

#include <stdint.h>

typedef enum { ESP_PARTITION_TYPE_APP = 0x00, ESP_PARTITION_TYPE_DATA = 0x01 } esp_partition_type_t;
typedef enum {
  ESP_PARTITION_SUBTYPE_DATA_OTA = 0x00,
  ESP_PARTITION_SUBTYPE_DATA_NVS = 0x02,
} esp_partition_subtype_t;

typedef struct {
  esp_partition_type_t type;
  esp_partition_subtype_t subtype;
  uint32_t address;
  uint32_t size;
  char label[17];
} esp_partition_t;

const esp_partition_t* esp_partition_find_first(esp_partition_type_t type,
                                                esp_partition_subtype_t subtype, const char* label);

#endif  // MATDOG_TESTS_ESP_PARTITION_STUB_H
