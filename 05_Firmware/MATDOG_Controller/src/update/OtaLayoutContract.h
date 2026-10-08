#ifndef MATDOG_UPDATE_OTA_LAYOUT_CONTRACT_H
#define MATDOG_UPDATE_OTA_LAYOUT_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

// MATDOG flash layout V1 as the FIRMWARE sees it. ESP-IDF-free, so the same
// check runs on the host (scripts/tests/test_ota_layout_contract.cpp).
//
// The values mirror partitions.csv and scripts/matdog_layout.py; the static
// audit compares all three. The layout identity is compiled into the binary
// (kLayoutMarker, findable with `strings`) and the runtime table is compared
// against the compiled-in contract, so it is a property of the firmware
// itself and never something a client declares in OTA metadata.

namespace matdog {
namespace update {

constexpr char kLayoutId[] = "MATDOG_16M_2x5M_NVS_V1";
extern const char kLayoutMarker[];  // "MATDOG_LAYOUT_ID=" + kLayoutId

struct LayoutPartition {
  const char* label;
  uint8_t type;     // esp_partition_type_t: app 0x00, data 0x01
  uint8_t subtype;
  uint32_t offset;
  uint32_t size;
};

constexpr size_t kLayoutPartitionCount = 7;
extern const LayoutPartition kLayoutV1[kLayoutPartitionCount];

enum class LayoutVerdict : uint8_t {
  OK = 0,
  EMPTY_OR_OVERFLOW,   // no entries, or more than the contract allows
  MATDOG_NVS_MISSING,
  MATDOG_NVS_MOVED,
  NVS_ORDER,           // default NVS must be the first nvs-subtype partition
  MISMATCH,            // any other difference from the contract
};

// `actual` is the runtime table in table order. Exact match required.
LayoutVerdict checkLayoutV1(const LayoutPartition* actual, size_t count);
const char* toString(LayoutVerdict verdict);

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_LAYOUT_CONTRACT_H
