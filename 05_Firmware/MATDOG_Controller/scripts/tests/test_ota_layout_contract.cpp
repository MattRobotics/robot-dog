// Offline host tests for the firmware-side MATDOG layout contract
// (src/update/OtaLayoutContract.*): the runtime partition table is compared
// against the layout compiled into the firmware. No ESP-IDF, no device.

#include <cstdio>
#include <cstring>
#include <vector>

#include "../../src/update/OtaLayoutContract.h"

using namespace matdog::update;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                            \
  } while (0)

#define CHECK_V(vec, expected)                                                   \
  do {                                                                           \
    ++g_checks;                                                                  \
    const LayoutVerdict v_ = checkLayoutV1((vec).data(), (vec).size());          \
    if (v_ != (expected)) {                                                      \
      ++g_failures;                                                              \
      std::printf("  FAIL [%s] %s:%d: verdict %s, expected %s\n", g_case,        \
                  __FILE__, __LINE__, toString(v_), toString(expected));         \
    }                                                                            \
  } while (0)

static std::vector<LayoutPartition> contract() {
  return std::vector<LayoutPartition>(kLayoutV1, kLayoutV1 + kLayoutPartitionCount);
}

static void test_exact_contract() {
  g_case = "exact_contract";
  auto t = contract();
  CHECK_V(t, LayoutVerdict::OK);
  CHECK(t.size() == 7);
  CHECK(std::strcmp(t[0].label, "nvs") == 0);
  CHECK(std::strcmp(t[5].label, "matdog_nvs") == 0);
  CHECK(t[2].size == 0x500000u && t[3].size == 0x500000u);
  CHECK(t[5].offset == 0xFE0000u && t[5].size == 0x10000u);
  CHECK(checkLayoutV1(nullptr, 0) == LayoutVerdict::EMPTY_OR_OVERFLOW);
}

static void test_marker_carries_the_layout_id() {
  g_case = "marker";
  const char want[] = "MATDOG_LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1";
  CHECK(std::strcmp(kLayoutMarker, want) == 0);
  CHECK(std::strcmp(kLayoutMarker + std::strlen("MATDOG_LAYOUT_ID="), kLayoutId) == 0);
}

static void test_legacy_table_refused() {
  g_case = "legacy_app3M_fat9M_16MB";
  // The table the device runs today: 3 MiB slots, no MATDOG NVS.
  std::vector<LayoutPartition> t = {
      {"nvs", 0x01, 0x02, 0x009000u, 0x005000u},
      {"otadata", 0x01, 0x00, 0x00E000u, 0x002000u},
      {"app0", 0x00, 0x10, 0x010000u, 0x300000u},
      {"app1", 0x00, 0x11, 0x310000u, 0x300000u},
      {"ffat", 0x01, 0x81, 0x610000u, 0x9E0000u},
      {"coredump", 0x01, 0x03, 0xFF0000u, 0x010000u},
  };
  CHECK_V(t, LayoutVerdict::MATDOG_NVS_MISSING);
}

static void test_moved_or_resized_matdog_nvs() {
  g_case = "matdog_nvs_moved";
  auto t = contract();
  t[5].offset = 0xFD0000u;
  CHECK_V(t, LayoutVerdict::MATDOG_NVS_MOVED);
  t = contract();
  t[5].size = 0x20000u;
  CHECK_V(t, LayoutVerdict::MATDOG_NVS_MOVED);
  t = contract();
  t.push_back(t[5]);  // duplicate label
  CHECK_V(t, LayoutVerdict::MATDOG_NVS_MOVED);
}

static void test_nvs_order() {
  g_case = "nvs_order";
  auto t = contract();
  std::swap(t[0], t[5]);  // matdog_nvs becomes the first nvs-subtype partition
  CHECK_V(t, LayoutVerdict::NVS_ORDER);
}

static void test_other_deviations() {
  g_case = "other_deviations";
  auto t = contract();
  t[2].size = 0x300000u;  // shrunk app slot
  CHECK_V(t, LayoutVerdict::MISMATCH);

  t = contract();
  t[3].offset = 0x310000u;
  CHECK_V(t, LayoutVerdict::MISMATCH);

  t = contract();
  t.pop_back();  // coredump gone
  CHECK_V(t, LayoutVerdict::MISMATCH);

  t = contract();
  t[4].label = "storage";
  CHECK_V(t, LayoutVerdict::MISMATCH);

  t = contract();
  t[2].subtype = 0x11;
  CHECK_V(t, LayoutVerdict::MISMATCH);

  std::vector<LayoutPartition> big(40, kLayoutV1[0]);
  CHECK_V(big, LayoutVerdict::EMPTY_OR_OVERFLOW);
}

static void test_tostring_is_total() {
  g_case = "tostring";
  for (uint8_t i = 0; i <= (uint8_t)LayoutVerdict::MISMATCH; ++i) {
    CHECK(std::strcmp(toString((LayoutVerdict)i), "UNKNOWN") != 0);
  }
}

int main() {
  std::printf("MATDOG layout contract offline tests\n");
  test_exact_contract();
  test_marker_carries_the_layout_id();
  test_legacy_table_refused();
  test_moved_or_resized_matdog_nvs();
  test_nvs_order();
  test_other_deviations();
  test_tostring_is_total();
  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("OTA_LAYOUT_CONTRACT_TESTS = FAIL\n");
    return 1;
  }
  std::printf("OTA_LAYOUT_CONTRACT_TESTS = PASS\n");
  return 0;
}
