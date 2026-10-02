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

// What nvs_set_blob leaves behind when it returns set_error. In ESP-IDF a
// failing set can fail before anything is published (e.g. NOT_ENOUGH_SPACE),
// or after a partial/full publication (ESP_ERR_NVS_REMOVE_FAILED).
enum class SetPublication { NOTHING, PARTIAL, FULL };

struct StubNvs {
  std::map<std::string, std::map<std::string, std::vector<uint8_t>>> ns;
  std::vector<std::string> calls;
  esp_err_t open_error = ESP_OK;
  esp_err_t set_error = ESP_OK;
  SetPublication set_publication = SetPublication::NOTHING;  // only when set_error != ESP_OK
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
  const uint8_t* p = static_cast<const uint8_t*>(value);
  if (g.set_error != ESP_OK) {
    if (g.set_publication == SetPublication::PARTIAL) {
      g.ns[g.open_name][key].assign(p, p + length / 2);
    } else if (g.set_publication == SetPublication::FULL) {
      g.ns[g.open_name][key].assign(p, p + length);
    }
    return g.set_error;
  }
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

  // Failing to open touches no slot: the only write error that is certain.
  g.open_error = ESP_ERR_NVS_NOT_INITIALIZED;
  CHECK(nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NOT_MODIFIED);
  g.open_error = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NOT_MODIFIED);
  g.open_error = ESP_OK;
  CHECK(g.ns.empty() || g.ns["matdog_calrec"].empty());

  g.set_error = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NO_SPACE);
  g.set_error = ESP_ERR_NVS_PAGE_FULL;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NO_SPACE);
  g.set_error = 0x1234;
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::IO_ERROR);
  // REMOVE_FAILED means "written, update finishes after re-init": an error, never OK.
  g.set_error = ESP_ERR_NVS_REMOVE_FAILED;
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

  CHECK(nvs.write(CalibrationSlot::A, nullptr, 2) == StorageIoStatus::NOT_MODIFIED);
  CHECK(nvs.write(CalibrationSlot::A, data, 0) == StorageIoStatus::NOT_MODIFIED);
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

// --- P2.1: nvs_set_blob fault model ------------------------------------------
// The stub does not prove physical durability; it only exercises what the
// store does with every combination of "error returned" x "what was published".

struct Seeded {
  CalibrationRecordNvsBackend nvs;
  CalibrationRecordStore store{&nvs};
  std::vector<uint8_t> b2;  // the last confirmed record (generation 2, slot B)
};

void seed(Seeded* s) {
  reset();
  const auto profile = golden::boundProfile();
  CHECK(s->store.save(golden::goldenRecord(0), profile).status == SaveStatus::OK);  // A/1
  CHECK(s->store.save(golden::goldenRecord(0), profile).status == SaveStatus::OK);  // B/2
  s->b2 = g.ns["matdog_calrec"]["B"];
}

// A reboot: new backend, new store, same flash content (the stub's map).
LoadResult rebootAndLoad(CalibrationRecord* out) {
  CalibrationRecordNvsBackend nvs;
  CalibrationRecordStore store(&nvs);
  return store.load(golden::boundProfile(), out);
}

void test_set_blob_fails_before_publication() {
  g_case = "set_blob fails before publication";
  const auto profile = golden::boundProfile();

  const esp_err_t errors[] = {ESP_ERR_NVS_NOT_ENOUGH_SPACE, ESP_ERR_NVS_PAGE_FULL, 0x1234};
  for (esp_err_t e : errors) {
    Seeded s;
    seed(&s);
    g.set_error = e;
    g.set_publication = SetPublication::NOTHING;
    const SaveResult r = s.store.save(golden::goldenRecord(0), profile);
    CHECK(r.status == SaveStatus::WRITE_FAILED || r.status == SaveStatus::NO_SPACE);
    CHECK(r.generation == 0);
    CHECK(g.ns["matdog_calrec"]["B"] == s.b2);
    CHECK(g.ns["matdog_calrec"]["A"].size() == kCalibrationRecordV1EncodedBytes);  // old A/1
    // The backend cannot tell this from a published failure: the block applies.
    CHECK(s.store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    g.set_error = ESP_OK;
    g.calls.clear();
    CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
    CHECK(g.calls.empty());  // refused without touching NVS at all
    CHECK(g.ns["matdog_calrec"]["B"] == s.b2);

    CalibrationRecord out;
    const LoadResult l = rebootAndLoad(&out);
    CHECK(l.status == LoadStatus::OK && l.generation == 2 && l.slot == CalibrationSlot::B);
  }
}

void test_set_blob_partial_publication() {
  g_case = "set_blob partial publication";
  Seeded s;
  seed(&s);
  const auto profile = golden::boundProfile();

  g.set_error = ESP_ERR_NVS_REMOVE_FAILED;
  g.set_publication = SetPublication::PARTIAL;
  const SaveResult r = s.store.save(golden::goldenRecord(0), profile);
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(r.io == StorageIoStatus::IO_ERROR);
  CHECK(r.previous_record_intact && r.previous_generation == 2);
  CHECK(g.ns["matdog_calrec"]["A"].size() == kCalibrationRecordV1EncodedBytes / 2);
  g.set_error = ESP_OK;

  // Subsequent read: the torn slot is corrupt, B/2 is selected and flagged degraded.
  CalibrationRecord loaded;
  const LoadResult l = s.store.load(profile, &loaded);
  CHECK(l.status == LoadStatus::OK && l.generation == 2 && l.slot == CalibrationSlot::B);
  CHECK(l.degraded);
  CHECK(l.report[0].state == SlotState::CORRUPT);

  // Retry refused, previous record untouched.
  CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(g.ns["matdog_calrec"]["B"] == s.b2);

  // Recovery after a reboot: B/2 selected, and the next save heals slot A.
  CalibrationRecordNvsBackend nvs2;
  CalibrationRecordStore rebooted(&nvs2);
  CHECK(rebooted.load(profile, &loaded).generation == 2);
  const SaveResult healed = rebooted.save(golden::goldenRecord(0), profile);
  CHECK(healed.status == SaveStatus::OK);
  CHECK(healed.slot == CalibrationSlot::A && healed.generation == 3);
  CHECK(g.ns["matdog_calrec"]["B"] == s.b2);
}

void test_set_blob_full_publication_with_error() {
  g_case = "set_blob full publication with error";
  Seeded s;
  seed(&s);
  const auto profile = golden::boundProfile();

  g.set_error = ESP_ERR_NVS_REMOVE_FAILED;
  g.set_publication = SetPublication::FULL;
  const SaveResult r = s.store.save(golden::goldenRecord(0), profile);
  CHECK(r.status == SaveStatus::WRITE_FAILED);  // the error is not retroactively turned into success
  CHECK(r.generation == 0);
  g.set_error = ESP_OK;
  CHECK(g.ns["matdog_calrec"]["A"].size() == kCalibrationRecordV1EncodedBytes);

  // The new record IS in flash and valid, although SAVE reported an error.
  CalibrationRecord loaded;
  const LoadResult l = s.store.load(profile, &loaded);
  CHECK(l.status == LoadStatus::OK && l.generation == 3 && l.slot == CalibrationSlot::A);
  CHECK(!l.degraded);

  // The only guard between a retry and the last confirmed B/2.
  CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(g.ns["matdog_calrec"]["B"] == s.b2);
  CHECK(s.store.load(profile, &loaded).generation == 3);  // load does not unlock
  CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);

  // After a reboot the valid highest generation is selected. This says nothing
  // about whether anyone confirmed it.
  const LoadResult rl = rebootAndLoad(&loaded);
  CHECK(rl.status == LoadStatus::OK && rl.generation == 3 && rl.slot == CalibrationSlot::A);
}

void test_commit_fails_after_publication() {
  g_case = "nvs_commit fails after publication";
  Seeded s;
  seed(&s);
  const auto profile = golden::boundProfile();

  g.commit_error = 0x1234;
  const SaveResult r = s.store.save(golden::goldenRecord(0), profile);
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  g.commit_error = ESP_OK;
  CHECK(s.store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK(s.store.save(golden::goldenRecord(0), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(g.ns["matdog_calrec"]["B"] == s.b2);
  CalibrationRecord loaded;
  const LoadResult rl = rebootAndLoad(&loaded);
  CHECK(rl.status == LoadStatus::OK && rl.generation == 3);
}

void test_open_failure_is_certain_and_retryable() {
  g_case = "open failure allows retry";
  Seeded s;
  seed(&s);
  const auto profile = golden::boundProfile();
  const auto snapshot = g.ns;

  g.open_error = ESP_ERR_NVS_NOT_INITIALIZED;
  SaveResult r = s.store.save(golden::goldenRecord(0), profile);
  // The scan could not even read: refused before any write.
  CHECK(r.status == SaveStatus::STORAGE_UNUSABLE);
  CHECK(s.store.writeState() == WriteState::OPEN);
  g.open_error = ESP_OK;
  CHECK(g.ns == snapshot);

  // Only the write-phase open fails (reads succeed): certain, retry allowed.
  struct FlakyOpen : CalibrationRecordStorage {
    CalibrationRecordNvsBackend inner;
    bool fail_write_open = true;
    StorageIoStatus read(CalibrationSlot sl, uint8_t* b, size_t c, size_t* n) override {
      return inner.read(sl, b, c, n);
    }
    StorageIoStatus write(CalibrationSlot sl, const uint8_t* d, size_t n) override {
      if (!fail_write_open) return inner.write(sl, d, n);
      g.open_error = ESP_ERR_NVS_NOT_INITIALIZED;
      const StorageIoStatus st = inner.write(sl, d, n);
      g.open_error = ESP_OK;
      return st;
    }
  } flaky;
  CalibrationRecordStore store(&flaky);
  r = store.save(golden::goldenRecord(0), profile);
  CHECK(r.status == SaveStatus::WRITE_FAILED && r.io == StorageIoStatus::NOT_MODIFIED);
  CHECK(store.writeState() == WriteState::OPEN);
  CHECK(g.ns == snapshot);
  flaky.fail_write_open = false;
  r = store.save(golden::goldenRecord(0), profile);
  CHECK(r.status == SaveStatus::OK && r.generation == 3 && r.slot == CalibrationSlot::A);
  CHECK(g.ns["matdog_calrec"]["B"] == s.b2);
}

}  // namespace

int main() {
  test_absent_and_round_trip();
  test_error_mapping();
  test_store_over_nvs_backend();
  test_set_blob_fails_before_publication();
  test_set_blob_partial_publication();
  test_set_blob_full_publication_with_error();
  test_commit_fails_after_publication();
  test_open_failure_is_certain_and_retryable();
  std::printf("test_calibration_record_nvs_backend: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
