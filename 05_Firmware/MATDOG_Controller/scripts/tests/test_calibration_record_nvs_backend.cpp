// Offline test for CalibrationRecordNvsBackend against a host stand-in of the
// ESP-IDF NVS API (nvs_stub/nvs.h). Proves what the backend asks NVS to do -
// dedicated namespace, keys "A"/"B", commit after set_blob, error mapping -
// and, by construction, that it never calls nvs_flash_init/erase (those are
// not even declared by the stub). No flash is touched.

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <nvs.h>

#include "calibration_record_golden.h"
#include "../../src/calibration/CalibrationRecordNvsBackend.h"

using namespace matdog;
using namespace matdog::calibration;

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

namespace {

struct StubNvs {
  std::map<std::string, std::map<std::string, std::vector<uint8_t>>> ns;
  std::vector<std::string> calls;
  esp_err_t open_error = ESP_OK;
  esp_err_t set_error = ESP_OK;
  esp_err_t commit_error = ESP_OK;
  esp_err_t get_error = ESP_OK;
  std::string open_name;
  nvs_open_mode_t open_mode = NVS_READONLY;
  bool dirty = false;       // set_blob not yet committed
  bool committed_after_set = false;
} g;

}  // namespace

esp_err_t nvs_open(const char* name, nvs_open_mode_t mode, nvs_handle_t* handle) {
  g.calls.push_back(std::string("open:") + name + (mode == NVS_READWRITE ? ":rw" : ":ro"));
  if (g.open_error != ESP_OK) return g.open_error;
  g.open_name = name;
  g.open_mode = mode;
  if (mode == NVS_READONLY && g.ns.find(name) == g.ns.end()) return ESP_ERR_NVS_NOT_FOUND;
  if (mode == NVS_READWRITE) g.ns[name];
  *handle = 1;
  return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t, const char* key, void* out, size_t* length) {
  g.calls.push_back(std::string("get:") + key + (out ? "" : ":size"));
  if (g.get_error != ESP_OK) return g.get_error;
  auto& m = g.ns[g.open_name];
  auto it = m.find(key);
  if (it == m.end()) return ESP_ERR_NVS_NOT_FOUND;
  if (out == nullptr) {
    *length = it->second.size();
    return ESP_OK;
  }
  if (*length < it->second.size()) return ESP_ERR_NVS_INVALID_LENGTH;
  std::memcpy(out, it->second.data(), it->second.size());
  *length = it->second.size();
  return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t, const char* key, const void* value, size_t length) {
  g.calls.push_back(std::string("set:") + key);
  if (g.open_mode != NVS_READWRITE) return -1;
  if (g.set_error != ESP_OK) return g.set_error;
  const uint8_t* p = static_cast<const uint8_t*>(value);
  g.ns[g.open_name][key].assign(p, p + length);
  g.dirty = true;
  return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t) {
  g.calls.push_back("commit");
  if (g.commit_error != ESP_OK) return g.commit_error;
  if (g.dirty) g.committed_after_set = true;
  g.dirty = false;
  return ESP_OK;
}

void nvs_close(nvs_handle_t) { g.calls.push_back("close"); }

namespace {

void reset() { g = StubNvs{}; }

void test_absent_and_round_trip() {
  g_case = "absent / round trip";
  reset();
  CalibrationRecordNvsBackend nvs;
  uint8_t buf[1536];
  size_t n = 77;

  // Namespace not created yet: a clean "never saved", and reading never creates it.
  CHECK(nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::ABSENT);
  CHECK(n == 0);
  CHECK(g.ns.empty());

  const uint8_t data[5] = {1, 2, 3, 4, 5};
  g.calls.clear();
  CHECK(nvs.write(CalibrationSlot::B, data, 5) == StorageIoStatus::OK);
  CHECK(g.ns.count("matdog_calrec") == 1);
  CHECK(g.ns["matdog_calrec"].count("B") == 1);
  CHECK(g.ns["matdog_calrec"].count("A") == 0);
  CHECK(g.committed_after_set);
  // open rw -> set -> commit -> close, in that order.
  CHECK(g.calls.size() == 4);
  CHECK(g.calls[0] == "open:matdog_calrec:rw" && g.calls[1] == "set:B" && g.calls[2] == "commit" && g.calls[3] == "close");

  CHECK(nvs.read(CalibrationSlot::B, buf, sizeof(buf), &n) == StorageIoStatus::OK);
  CHECK(n == 5 && std::memcmp(buf, data, 5) == 0);
  CHECK(nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::ABSENT);

  // A blob larger than the caller's buffer is reported, never truncated.
  CHECK(nvs.read(CalibrationSlot::B, buf, 3, &n) == StorageIoStatus::BUFFER_TOO_SMALL);
}

void test_error_mapping() {
  g_case = "error mapping";
  reset();
  CalibrationRecordNvsBackend nvs;
  uint8_t buf[64];
  size_t n = 0;
  const uint8_t data[2] = {9, 9};

  g.open_error = ESP_ERR_NVS_NOT_INITIALIZED;
  CHECK(nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::IO_ERROR);
  g.open_error = ESP_OK;

  g.set_error = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NO_SPACE);
  g.set_error = ESP_ERR_NVS_PAGE_FULL;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NO_SPACE);
  g.set_error = 0x1234;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::IO_ERROR);
  g.set_error = ESP_OK;

  // A failed commit is never reported as success.
  g.commit_error = 0x1234;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::IO_ERROR);
  g.commit_error = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NO_SPACE);
  g.commit_error = ESP_OK;

  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::OK);
  g.get_error = 0x1234;
  CHECK(nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
  g.get_error = ESP_OK;

  CHECK(nvs.write(CalibrationSlot::A, nullptr, 2) == StorageIoStatus::IO_ERROR);
  CHECK(nvs.write(CalibrationSlot::A, data, 0) == StorageIoStatus::IO_ERROR);
}

void test_store_over_nvs_backend() {
  g_case = "store over NVS backend";
  reset();
  CalibrationRecordNvsBackend nvs;
  CalibrationRecordStore store(&nvs);
  const auto profile = golden::boundProfile();
  CalibrationRecord loaded;

  CHECK(store.load(profile, &loaded).status == LoadStatus::NOT_FOUND);
  CHECK(store.save(golden::goldenRecord(0), profile).status == SaveStatus::OK);
  CHECK(store.save(golden::goldenRecord(0), profile).status == SaveStatus::OK);
  CHECK(g.ns["matdog_calrec"].size() == 2);
  CHECK(g.ns["matdog_calrec"]["A"].size() == kCalibrationRecordV1EncodedBytes);
  CHECK(g.ns["matdog_calrec"]["B"].size() == kCalibrationRecordV1EncodedBytes);
  const LoadResult l = store.load(profile, &loaded);
  CHECK(l.status == LoadStatus::OK && l.generation == 2 && l.slot == CalibrationSlot::B);

  // NVS reports a failed commit on the third save: the old record survives.
  const std::vector<uint8_t> b_before = g.ns["matdog_calrec"]["B"];
  g.commit_error = 0x1234;
  const SaveResult bad = store.save(golden::goldenRecord(0), profile);
  CHECK(bad.status == SaveStatus::WRITE_FAILED);
  g.commit_error = ESP_OK;
  CHECK(g.ns["matdog_calrec"]["B"] == b_before);

  // Only the calibration namespace was ever opened.
  for (const std::string& c : g.calls) {
    if (c.rfind("open:", 0) == 0) CHECK(c.find("open:matdog_calrec:") == 0);
  }
}

}  // namespace

int main() {
  test_absent_and_round_trip();
  test_error_mapping();
  test_store_over_nvs_backend();
  std::printf("test_calibration_record_nvs_backend: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
