#include "OtaLayoutContract.h"

#include <string.h>

namespace matdog {
namespace update {

// Must stay referenced from linked code (OtaEspBackend::layout_marker_): the
// toolchain here ignores the `retain` attribute, so --gc-sections drops it
// otherwise. The manifest verifier proves the binary's layout by finding it.
const char kLayoutMarker[] = "MATDOG_LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1";

const LayoutPartition kLayoutV1[kLayoutPartitionCount] = {
    {"nvs",        0x01, 0x02, 0x009000u, 0x005000u},
    {"otadata",    0x01, 0x00, 0x00E000u, 0x002000u},
    {"app0",       0x00, 0x10, 0x010000u, 0x500000u},
    {"app1",       0x00, 0x11, 0x510000u, 0x500000u},
    {"ffat",       0x01, 0x81, 0xA10000u, 0x5D0000u},
    {"matdog_nvs", 0x01, 0x02, 0xFE0000u, 0x010000u},
    {"coredump",   0x01, 0x03, 0xFF0000u, 0x010000u},
};

namespace {

constexpr uint8_t kTypeData = 0x01;
constexpr uint8_t kSubtypeNvs = 0x02;
constexpr size_t kMatdogNvsIndex = 5;

bool same(const LayoutPartition& a, const LayoutPartition& b) {
  return a.label != nullptr && strcmp(a.label, b.label) == 0 && a.type == b.type &&
         a.subtype == b.subtype && a.offset == b.offset && a.size == b.size;
}

}  // namespace

LayoutVerdict checkLayoutV1(const LayoutPartition* actual, size_t count) {
  if (actual == nullptr || count == 0 || count > 32) return LayoutVerdict::EMPTY_OR_OVERFLOW;

  const LayoutPartition* matdog = nullptr;
  for (size_t i = 0; i < count; ++i) {
    if (actual[i].label != nullptr && strcmp(actual[i].label, "matdog_nvs") == 0) {
      if (matdog != nullptr) return LayoutVerdict::MATDOG_NVS_MOVED;  // duplicate
      matdog = &actual[i];
    }
  }
  if (matdog == nullptr) return LayoutVerdict::MATDOG_NVS_MISSING;
  if (!same(*matdog, kLayoutV1[kMatdogNvsIndex])) return LayoutVerdict::MATDOG_NVS_MOVED;

  for (size_t i = 0; i < count; ++i) {
    if (actual[i].type == kTypeData && actual[i].subtype == kSubtypeNvs) {
      if (actual[i].label == nullptr || strcmp(actual[i].label, "nvs") != 0) {
        return LayoutVerdict::NVS_ORDER;
      }
      break;
    }
  }

  if (count != kLayoutPartitionCount) return LayoutVerdict::MISMATCH;
  for (size_t i = 0; i < count; ++i) {
    if (!same(actual[i], kLayoutV1[i])) return LayoutVerdict::MISMATCH;
  }
  return LayoutVerdict::OK;
}

const char* toString(LayoutVerdict verdict) {
  switch (verdict) {
    case LayoutVerdict::OK:                return "OK";
    case LayoutVerdict::EMPTY_OR_OVERFLOW: return "EMPTY_OR_OVERFLOW";
    case LayoutVerdict::MATDOG_NVS_MISSING: return "MATDOG_NVS_MISSING";
    case LayoutVerdict::MATDOG_NVS_MOVED:  return "MATDOG_NVS_MOVED";
    case LayoutVerdict::NVS_ORDER:         return "NVS_ORDER";
    case LayoutVerdict::MISMATCH:          return "MISMATCH";
  }
  return "UNKNOWN";
}

}  // namespace update
}  // namespace matdog
