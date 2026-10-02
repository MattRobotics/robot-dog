// Offline test for CalibrationRecordNvsBackend against a host stand-in of the
// ESP-IDF NVS API (nvs_stub/). It proves what the backend ASKS NVS to do -
// dedicated partition label, explicit init, no default-partition init, no erase
// (those functions are not even declared by the stub), key layout, commit order,
// error mapping - and, together with CalibrationRecordStore, what the SAVE
// protocol leaves behind when the stub injects an error or a power cut at every
// mutating step. No flash is touched. The stub cannot prove physical durability,
// NVS-internal page recovery or real power-loss behaviour of the SPI flash.

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <esp_partition.h>
#include <nvs.h>
#include <nvs_flash.h>

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

// What nvs_set_blob leaves in flash when it fails or the power is cut during it.
enum class Pub { NOTHING, PARTIAL, FULL };

using Flash = std::map<std::string, std::map<std::string, std::vector<uint8_t>>>;

struct Stub {
  // --- persistent: survives reboot() ---
  Flash flash;

  // --- partition table ---
  bool part_present = true;
  esp_partition_t part = {ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, 0xFE0000u,
                          0x10000u, "matdog_nvs"};

  // --- init / open behaviour ---
  esp_err_t init_error = ESP_OK;
  esp_err_t open_error = ESP_OK;  // any nvs_open_from_partition
  bool initialized = false;       // RAM state of the NVS library, lost on reboot

  // --- fault injection, counted over mutating calls (nvs_set_blob, nvs_commit) ---
  int mut = 0;           // mutating calls so far
  int sets = 0;          // nvs_set_blob calls so far
  int cut_at = 0;        // power is cut during this mutating call (0 = never)
  int err_at = 0;        // this mutating call returns err_code (0 = never); power stays on
  Pub fault_pub = Pub::NOTHING;  // what a faulting nvs_set_blob published
  esp_err_t err_code = 0x1234;
  bool dead = false;     // power is off: every call fails, nothing changes

  // --- read faults ---
  esp_err_t get_error = ESP_OK;
  std::string get_error_key;      // empty = any key
  int get_error_min_sets = 0;     // only once this many sets happened

  // --- observation ---
  std::vector<std::string> calls;
  int label_violations = 0;       // a call named a partition other than "matdog_nvs"
  int init_calls = 0;

  // open handle
  std::string h_ns;
  nvs_open_mode_t h_mode = NVS_READONLY;
} g;

void reboot() {
  g.dead = false;
  g.cut_at = 0;
  g.err_at = 0;
  g.mut = 0;
  g.sets = 0;
  g.initialized = false;
  g.get_error = ESP_OK;
  g.open_error = ESP_OK;
  g.init_error = ESP_OK;
  g.fault_pub = Pub::NOTHING;
}

void resetAll() {
  Flash keep;  // nothing kept: brand new flash
  g = Stub{};
  g.flash = keep;
}

void checkLabel(const char* label) {
  if (label == nullptr || std::strcmp(label, "matdog_nvs") != 0) ++g.label_violations;
}

}  // namespace

const esp_partition_t* esp_partition_find_first(esp_partition_type_t type,
                                                esp_partition_subtype_t subtype, const char* label) {
  g.calls.push_back(std::string("find:") + (label ? label : "(null)"));
  if (!g.part_present) return nullptr;
  if (type != g.part.type || subtype != g.part.subtype) return nullptr;
  if (label != nullptr && std::strcmp(label, g.part.label) != 0) return nullptr;
  return &g.part;
}

esp_err_t nvs_flash_init_partition(const char* label) {
  g.calls.push_back(std::string("init:") + (label ? label : "(null)"));
  ++g.init_calls;
  checkLabel(label);
  if (g.dead) return ESP_FAIL;
  if (g.init_error != ESP_OK) return g.init_error;
  g.initialized = true;
  return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char* label, const char* ns, nvs_open_mode_t mode,
                                  nvs_handle_t* handle) {
  g.calls.push_back(std::string("open:") + (label ? label : "(null)") + ":" + ns +
                    (mode == NVS_READWRITE ? ":rw" : ":ro"));
  checkLabel(label);
  if (g.dead) return ESP_FAIL;
  if (!g.initialized) return ESP_ERR_NVS_NOT_INITIALIZED;
  if (g.open_error != ESP_OK) return g.open_error;
  if (mode == NVS_READONLY && g.flash.find(ns) == g.flash.end()) return ESP_ERR_NVS_NOT_FOUND;
  if (mode == NVS_READWRITE) g.flash[ns];
  g.h_ns = ns;
  g.h_mode = mode;
  *handle = 1;
  return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t, const char* key, void* out, size_t* length) {
  g.calls.push_back(std::string("get:") + key + (out ? "" : ":size"));
  if (g.dead) return ESP_FAIL;
  if (g.get_error != ESP_OK && (g.get_error_key.empty() || g.get_error_key == key) &&
      g.sets >= g.get_error_min_sets) {
    return g.get_error;
  }
  auto& m = g.flash[g.h_ns];
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
  if (g.dead) return ESP_FAIL;
  if (g.h_mode != NVS_READWRITE) return ESP_ERR_NVS_READ_ONLY;
  ++g.mut;
  ++g.sets;
  const uint8_t* p = static_cast<const uint8_t*>(value);
  const bool cut = g.cut_at != 0 && g.mut == g.cut_at;
  const bool err = g.err_at != 0 && g.mut == g.err_at;
  if (cut || err) {
    if (g.fault_pub == Pub::PARTIAL) {
      g.flash[g.h_ns][key].assign(p, p + length / 2);
    } else if (g.fault_pub == Pub::FULL) {
      g.flash[g.h_ns][key].assign(p, p + length);
    }
    if (cut) g.dead = true;
    return cut ? ESP_FAIL : g.err_code;
  }
  g.flash[g.h_ns][key].assign(p, p + length);
  return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t) {
  g.calls.push_back("commit");
  if (g.dead) return ESP_FAIL;
  ++g.mut;
  if (g.cut_at != 0 && g.mut == g.cut_at) {
    g.dead = true;
    return ESP_FAIL;
  }
  if (g.err_at != 0 && g.mut == g.err_at) return g.err_code;
  return ESP_OK;
}

void nvs_close(nvs_handle_t) { g.calls.push_back("close"); }

namespace {

const char* const kNs = "matdog_calrec";
const auto profile = golden::boundProfile();

struct Rig {
  CalibrationRecordNvsBackend nvs;
  CalibrationRecordStore store{&nvs};
};

bool boot(Rig* r) {
  const NvsInitStatus st = r->nvs.begin();
  CHECK(st == NvsInitStatus::READY);
  return st == NvsInitStatus::READY;
}

SaveResult doSave(Rig* r) { return r->store.save(golden::goldenRecord(0), profile); }
LoadResult doLoad(Rig* r) {
  CalibrationRecord tmp;
  return r->store.load(profile, &tmp);
}

AckResult doAck(Rig* r, uint32_t generation) { return r->store.acknowledge(generation, profile); }

// SAVE then the caller's explicit ACK of the generation it returned.
SaveResult doSaveAck(Rig* r) {
  const SaveResult s = doSave(r);
  CHECK(s.status == SaveStatus::OK);
  if (s.status == SaveStatus::OK) CHECK(doAck(r, s.generation).status == AckStatus::OK);
  return s;
}

// A: gen 1, B: gen 2, both acknowledged, marker IDLE(2,2). The next SAVE targets slot A.
void seed(Rig* r) {
  resetAll();
  boot(r);
  doSaveAck(r);
  doSaveAck(r);
  g.mut = 0;
  g.sets = 0;
  g.calls.clear();
}

void seedFresh() {
  Rig s;
  seed(&s);
}

std::vector<uint8_t> blobOf(const char* key) { return g.flash[kNs][key]; }

bool decodeFlashMarker(SaveMarker* m) {
  const std::vector<uint8_t>& b = g.flash[kNs]["M"];
  return decodeSaveMarker(b.data(), b.size(), m) == SaveMarkerStatus::OK;
}

PersistenceInputs inputsOf(const LoadResult& l) {
  PersistenceInputs in;
  for (int i = 0; i < kCalibrationSlotCount; ++i) in.slot[i] = l.report[i];
  in.marker = l.marker;
  return in;
}

// The explicit operator action, performed through the backend (not a Controller
// command): plan, encode, write the marker.
bool reconcile(Rig* r, ReconciliationAction action, uint32_t generation) {
  const LoadResult l = doLoad(r);
  SaveMarker m;
  if (planReconciliation(inputsOf(l), action, generation, &m) != ReconciliationStatus::OK) return false;
  uint8_t buf[kSaveMarkerV1Bytes];
  size_t n = 0;
  if (encodeSaveMarker(m, buf, sizeof(buf), &n) != SaveMarkerStatus::OK) return false;
  return r->nvs.writeMarker(buf, n) == StorageIoStatus::OK;
}

void patchSchemaAndRecrc(std::vector<uint8_t>* b, uint16_t schema) {
  (*b)[4] = static_cast<uint8_t>(schema);
  (*b)[5] = static_cast<uint8_t>(schema >> 8);
  const uint32_t crc = calibrationCrc32(b->data(), b->size() - 4);
  for (int i = 0; i < 4; ++i) (*b)[b->size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
}

// ---------------------------------------------------------------------------
// begin(): explicit, verifiable, fail closed, never erases.
// ---------------------------------------------------------------------------

void test_begin_ready_and_idempotent() {
  g_case = "begin READY";
  resetAll();
  CalibrationRecordNvsBackend nvs;
  CHECK(nvs.initStatus() == NvsInitStatus::NOT_INITIALIZED);
  CHECK(nvs.begin() == NvsInitStatus::READY);
  CHECK(nvs.initStatus() == NvsInitStatus::READY && nvs.lastEspError() == 0);
  CHECK(g.init_calls == 1);
  // find (label-filtered) -> init by label -> read-only namespace probe.
  CHECK(g.calls.size() == 3);
  CHECK(g.calls[0] == "find:matdog_nvs");
  CHECK(g.calls[1] == "init:matdog_nvs");
  CHECK(g.calls[2] == "open:matdog_nvs:matdog_calrec:ro");
  // Probing a missing namespace creates nothing: flash is still blank.
  CHECK(g.flash.empty());
  CHECK(nvs.begin() == NvsInitStatus::READY);
  CHECK(g.init_calls == 1);  // READY is not re-initialized
  CHECK(std::string(toString(NvsInitStatus::READY)) == "READY");
}

void test_begin_failures_are_distinct_and_leave_flash_alone() {
  g_case = "begin failures";
  struct Case {
    const char* name;
    void (*arrange)();
    NvsInitStatus expect;
    int32_t esp_error;
  };
  const Case cases[] = {
      {"partition missing", [] { g.part_present = false; }, NvsInitStatus::PARTITION_MISSING, 0},
      {"wrong address", [] { g.part.address = 0xFE0000u + 0x1000u; },
       NvsInitStatus::PARTITION_GEOMETRY_MISMATCH, 0},
      {"wrong size", [] { g.part.size = 0x6000u; }, NvsInitStatus::PARTITION_GEOMETRY_MISMATCH, 0},
      {"no free pages", [] { g.init_error = ESP_ERR_NVS_NO_FREE_PAGES; },
       NvsInitStatus::NO_FREE_PAGES, ESP_ERR_NVS_NO_FREE_PAGES},
      {"new version found", [] { g.init_error = ESP_ERR_NVS_NEW_VERSION_FOUND; },
       NvsInitStatus::NEW_VERSION_FOUND, ESP_ERR_NVS_NEW_VERSION_FOUND},
      {"part not found (nvs)", [] { g.init_error = ESP_ERR_NVS_PART_NOT_FOUND; },
       NvsInitStatus::PARTITION_MISSING, ESP_ERR_NVS_PART_NOT_FOUND},
      {"not found (esp)", [] { g.init_error = ESP_ERR_NOT_FOUND; },
       NvsInitStatus::PARTITION_MISSING, ESP_ERR_NOT_FOUND},
      {"invalid state", [] { g.init_error = ESP_ERR_NVS_INVALID_STATE; },
       NvsInitStatus::INIT_FAILED, ESP_ERR_NVS_INVALID_STATE},
      {"generic fail", [] { g.init_error = ESP_FAIL; }, NvsInitStatus::INIT_FAILED, ESP_FAIL},
      {"unknown code", [] { g.init_error = 0x1234; }, NvsInitStatus::INIT_FAILED, 0x1234},
      {"open fails", [] { g.open_error = ESP_ERR_NVS_INVALID_STATE; }, NvsInitStatus::OPEN_FAILED,
       ESP_ERR_NVS_INVALID_STATE},
  };
  std::vector<NvsInitStatus> seen;
  for (const Case& c : cases) {
    g_case = c.name;
    Rig seeded;
    seed(&seeded);
    const Flash before = g.flash;
    reboot();
    c.arrange();
    g.calls.clear();

    Rig r;
    CHECK(r.nvs.begin() == c.expect);
    CHECK(r.nvs.initStatus() == c.expect);
    CHECK(r.nvs.lastEspError() == c.esp_error);
    seen.push_back(c.expect);
    // Nothing was erased, formatted or written: every call is find/init/open/close.
    CHECK(g.flash == before);
    for (const std::string& s : g.calls) {
      CHECK(s.rfind("find:", 0) == 0 || s.rfind("init:", 0) == 0 || s.rfind("open:", 0) == 0 ||
            s == "close");
    }
    CHECK(g.mut == 0);
    CHECK(g.label_violations == 0);

    // Fail closed: no read, no write, and no store progress while not READY.
    uint8_t buf[64];
    size_t n = 5;
    const uint8_t data[2] = {1, 2};
    CHECK(r.nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR && n == 0);
    CHECK(r.nvs.readMarker(buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
    CHECK(r.nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NOT_MODIFIED);
    CHECK(r.nvs.writeMarker(data, 2) == StorageIoStatus::NOT_MODIFIED);
    CHECK(doLoad(&r).status == LoadStatus::IO_ERROR);
    const SaveResult s = doSave(&r);
    CHECK(s.status == SaveStatus::STORAGE_UNUSABLE);
    CHECK(g.mut == 0 && g.flash == before);

    // No automatic retry inside begin(); an explicit later begin() can succeed.
    const int inits = g.init_calls;
    reboot();
    g.part_present = true;
    g.part.address = 0xFE0000u;
    g.part.size = 0x10000u;
    CHECK(r.nvs.begin() == NvsInitStatus::READY);
    CHECK(g.init_calls == inits + 1 || c.expect == NvsInitStatus::PARTITION_MISSING ||
          c.expect == NvsInitStatus::PARTITION_GEOMETRY_MISMATCH);
    CHECK(doLoad(&r).status == LoadStatus::OK);
  }
  CHECK(seen[3] == NvsInitStatus::NO_FREE_PAGES && seen[4] == NvsInitStatus::NEW_VERSION_FOUND);
  CHECK(std::string(toString(NvsInitStatus::NO_FREE_PAGES)) == "NO_FREE_PAGES");
  CHECK(std::string(toString(NvsInitStatus::NEW_VERSION_FOUND)) == "NEW_VERSION_FOUND");
  CHECK(std::string(toString(NvsInitStatus::PARTITION_MISSING)) == "PARTITION_MISSING");
  CHECK(std::string(toString(NvsInitStatus::PARTITION_GEOMETRY_MISMATCH)) ==
        "PARTITION_GEOMETRY_MISMATCH");
  CHECK(std::string(toString(NvsInitStatus::INIT_FAILED)) == "INIT_FAILED");
  CHECK(std::string(toString(NvsInitStatus::OPEN_FAILED)) == "OPEN_FAILED");
  CHECK(std::string(toString(NvsInitStatus::NOT_INITIALIZED)) == "NOT_INITIALIZED");
}

void test_never_initialized_backend_is_fail_closed() {
  g_case = "backend without begin()";
  resetAll();
  g.initialized = true;  // even if somebody else initialized the library
  CalibrationRecordNvsBackend nvs;
  uint8_t buf[64];
  size_t n = 0;
  const uint8_t data[2] = {1, 2};
  CHECK(nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
  CHECK(nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NOT_MODIFIED);
  CHECK(g.calls.empty());
  CHECK(g.flash.empty());
}

// ---------------------------------------------------------------------------
// Backend primitives
// ---------------------------------------------------------------------------

void test_absent_and_round_trip() {
  g_case = "absent / round trip";
  resetAll();
  Rig r;
  boot(&r);
  uint8_t buf[1536];
  size_t n = 77;

  CHECK(r.nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::ABSENT);
  CHECK(n == 0);
  CHECK(r.nvs.readMarker(buf, sizeof(buf), &n) == StorageIoStatus::ABSENT);
  CHECK(g.flash.empty());  // reading never creates the namespace

  const uint8_t data[5] = {1, 2, 3, 4, 5};
  g.calls.clear();
  CHECK(r.nvs.write(CalibrationSlot::B, data, 5) == StorageIoStatus::OK);
  CHECK(g.calls.size() == 4);
  CHECK(g.calls[0] == "open:matdog_nvs:matdog_calrec:rw" && g.calls[1] == "set:B" &&
        g.calls[2] == "commit" && g.calls[3] == "close");
  CHECK(r.nvs.read(CalibrationSlot::B, buf, sizeof(buf), &n) == StorageIoStatus::OK);
  CHECK(n == 5 && std::memcmp(buf, data, 5) == 0);
  CHECK(r.nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::ABSENT);
  CHECK(r.nvs.read(CalibrationSlot::B, buf, 3, &n) == StorageIoStatus::BUFFER_TOO_SMALL);

  // The marker lives under its own key, apart from the slots.
  g.calls.clear();
  CHECK(r.nvs.writeMarker(data, 3) == StorageIoStatus::OK);
  CHECK(g.calls[1] == "set:M" && g.calls[2] == "commit");
  CHECK(g.flash[kNs].size() == 2 && g.flash[kNs].count("M") == 1 && g.flash[kNs].count("B") == 1);
  CHECK(r.nvs.readMarker(buf, sizeof(buf), &n) == StorageIoStatus::OK && n == 3);
  CHECK(r.nvs.write(CalibrationSlot::A, nullptr, 2) == StorageIoStatus::NOT_MODIFIED);
  CHECK(r.nvs.write(CalibrationSlot::A, data, 0) == StorageIoStatus::NOT_MODIFIED);
  CHECK(r.nvs.writeMarker(data, 0) == StorageIoStatus::NOT_MODIFIED);
}

void test_error_mapping() {
  g_case = "error mapping";
  resetAll();
  Rig r;
  boot(&r);
  uint8_t buf[64];
  size_t n = 0;
  const uint8_t data[2] = {9, 9};

  // Failing to open touches nothing: the only certain write failure.
  g.open_error = ESP_ERR_NVS_INVALID_STATE;
  CHECK(r.nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
  CHECK(r.nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NOT_MODIFIED);
  CHECK(r.nvs.writeMarker(data, 2) == StorageIoStatus::NOT_MODIFIED);
  g.open_error = ESP_OK;
  CHECK(g.flash.empty() || g.flash[kNs].empty());

  // From nvs_set_blob onwards an error leaves the key unknown: never NOT_MODIFIED.
  const struct { esp_err_t e; StorageIoStatus s; } sets[] = {
      {ESP_ERR_NVS_NOT_ENOUGH_SPACE, StorageIoStatus::NO_SPACE},
      {ESP_ERR_NVS_PAGE_FULL, StorageIoStatus::NO_SPACE},
      {0x1234, StorageIoStatus::IO_ERROR},
      {ESP_ERR_NVS_REMOVE_FAILED, StorageIoStatus::IO_ERROR},
      {ESP_ERR_NVS_INVALID_LENGTH, StorageIoStatus::IO_ERROR},
  };
  for (const auto& c : sets) {
    reboot();
    Rig w;
    boot(&w);
    g.err_at = 1;
    g.err_code = c.e;
    CHECK(w.nvs.write(CalibrationSlot::A, data, 2) == c.s);
    reboot();
    Rig wm;
    boot(&wm);
    g.err_at = 1;
    g.err_code = c.e;
    CHECK(wm.nvs.writeMarker(data, 2) == c.s);
  }
  // A failed commit is never success.
  reboot();
  Rig c1;
  boot(&c1);
  g.err_at = 2;
  g.err_code = 0x1234;
  CHECK(c1.nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::IO_ERROR);
  reboot();
  Rig c2;
  boot(&c2);
  g.err_at = 2;
  g.err_code = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
  CHECK(c2.nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::NO_SPACE);

  reboot();
  Rig c3;
  boot(&c3);
  CHECK(c3.nvs.write(CalibrationSlot::A, data, 2) == StorageIoStatus::OK);
  g.get_error = 0x1234;
  CHECK(c3.nvs.read(CalibrationSlot::A, buf, sizeof(buf), &n) == StorageIoStatus::IO_ERROR);
  g.get_error = ESP_OK;
}

// ---------------------------------------------------------------------------
// Store over the backend: first install, total loss
// ---------------------------------------------------------------------------

void test_first_install_and_total_loss() {
  g_case = "first install / total loss";
  resetAll();
  Rig r;
  boot(&r);

  LoadResult l = doLoad(&r);
  CHECK(l.status == LoadStatus::NOT_FOUND);
  CHECK(l.assessment.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED);
  CHECK(!l.assessment.record_available && l.assessment.save_allowed);
  CHECK(g.flash.empty());  // loading never creates anything

  // The first SAVE after a fresh Full Calibration: generation 1, slot A, marker created.
  const SaveResult s1 = doSave(&r);
  CHECK(s1.status == SaveStatus::OK && s1.generation == 1 && s1.slot == CalibrationSlot::A);
  SaveMarker m;
  // SAVE=OK is "verified, awaiting ACK": nothing is acknowledged, nothing is served.
  CHECK(decodeFlashMarker(&m) && m.state == SaveMarkerState::AWAITING_ACK &&
        m.acknowledged_generation == 0 && m.begun_generation == 1);
  CHECK(g.flash[kNs]["A"].size() == kCalibrationRecordV1EncodedBytes);
  CHECK(g.flash[kNs]["M"].size() == kSaveMarkerV1Bytes);
  l = doLoad(&r);
  CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED && !l.assessment.record_available);
  CHECK(l.assessment.cls == PersistenceClass::AWAITING_ACK);
  CHECK(doSave(&r).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK(doAck(&r, 1).status == AckStatus::OK);
  CHECK(decodeFlashMarker(&m) && m.state == SaveMarkerState::IDLE &&
        m.acknowledged_generation == 1 && m.begun_generation == 1);
  l = doLoad(&r);
  CHECK(l.status == LoadStatus::OK && l.generation == 1);
  const SaveResult s2 = doSaveAck(&r);
  CHECK(s2.status == SaveStatus::OK && s2.generation == 2 && s2.slot == CalibrationSlot::B);

  // Whole partition erased out of band: indistinguishable from a first install,
  // and in particular NOT a calibration that may authorize motion.
  g.flash.clear();
  Rig after;
  boot(&after);
  l = doLoad(&after);
  CHECK(l.status == LoadStatus::NOT_FOUND && !l.assessment.record_available);
  CHECK(l.assessment.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED);
  const SaveResult s3 = doSave(&after);
  CHECK(s3.status == SaveStatus::OK && s3.generation == 1);

  // Only the marker erased: the records alone confirm nothing.
  seedFresh();
  g.flash[kNs].erase("M");
  Rig lost;
  boot(&lost);
  l = doLoad(&lost);
  CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
  CHECK(l.assessment.cls == PersistenceClass::MARKER_MISSING && !l.assessment.record_available);
  g.calls.clear();
  CHECK(doSave(&lost).status == SaveStatus::RECONCILIATION_REQUIRED);
  CHECK(g.mut == 0);
  // Both records wiped but the marker survived: the confirmed generation is lost.
  seedFresh();
  g.flash[kNs].erase("A");
  g.flash[kNs].erase("B");
  Rig lost2;
  boot(&lost2);
  l = doLoad(&lost2);
  CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
  CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  CHECK(!l.assessment.older_record_survives);
}

void test_marker_and_record_incompatibilities() {
  g_case = "incoherent marker / records";
  Rig r;

  // Confirmed generation lost, the previous one is valid: NOT healthy.
  seedFresh();
  SaveMarker m;
  m.state = SaveMarkerState::IDLE;
  m.acknowledged_generation = 3;
  m.begun_generation = 3;
  uint8_t buf[kSaveMarkerV1Bytes];
  size_t n = 0;
  CHECK(encodeSaveMarker(m, buf, sizeof(buf), &n) == SaveMarkerStatus::OK);
  g.flash[kNs]["M"].assign(buf, buf + n);
  {
    Rig b;
    boot(&b);
    const LoadResult l = doLoad(&b);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
    CHECK(l.assessment.older_record_survives && !l.assessment.record_available);
    CHECK(!l.assessment.save_allowed);
    const Flash before = g.flash;
    CHECK(doSave(&b).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(g.flash == before);
  }

  // A record newer than anything the marker attested.
  seedFresh();
  m.acknowledged_generation = 1;
  m.begun_generation = 1;
  CHECK(encodeSaveMarker(m, buf, sizeof(buf), &n) == SaveMarkerStatus::OK);
  g.flash[kNs]["M"].assign(buf, buf + n);
  {
    Rig b;
    boot(&b);
    const LoadResult l = doLoad(&b);
    CHECK(l.assessment.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(doSave(&b).status == SaveStatus::RECONCILIATION_REQUIRED);
  }

  // Marker written by another schema: this build neither interprets nor replaces it.
  seedFresh();
  std::vector<uint8_t> foreign = blobOf("M");
  patchSchemaAndRecrc(&foreign, 3);
  g.flash[kNs]["M"] = foreign;
  {
    Rig b;
    boot(&b);
    const LoadResult l = doLoad(&b);
    CHECK(l.status == LoadStatus::INCOMPATIBLE);
    CHECK(l.assessment.cls == PersistenceClass::MARKER_INCOMPATIBLE);
    CHECK(!l.assessment.reconciliation_required && !l.assessment.save_allowed);
    CHECK(doSave(&b).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(g.flash[kNs]["M"] == foreign);
  }

  // Damaged marker.
  seedFresh();
  g.flash[kNs]["M"][13] ^= 0x40;
  {
    Rig b;
    boot(&b);
    const LoadResult l = doLoad(&b);
    CHECK(l.assessment.cls == PersistenceClass::MARKER_CORRUPT && l.status == LoadStatus::RECONCILIATION_REQUIRED);
  }

  // Read error on a slot or the marker: IO_ERROR, nothing written, instance not blocked.
  Rig live;
  seed(&live);
  Rig& r2 = live;
  const Flash before = g.flash;
  for (const char* key : {"A", "B", "M"}) {
    g.get_error = 0x1234;
    g.get_error_key = key;
    g.get_error_min_sets = 0;
    CHECK(doLoad(&r2).status == LoadStatus::IO_ERROR);
    const SaveResult s = doSave(&r2);
    CHECK(s.status == SaveStatus::STORAGE_UNUSABLE);
    CHECK(r2.store.writeState() == WriteState::OPEN);
    CHECK(g.mut == 0 && g.flash == before);
  }
  g.get_error = ESP_OK;
}

// ---------------------------------------------------------------------------
// SAVE under injected errors and power cuts at every mutating step
// ---------------------------------------------------------------------------
// A third SAVE (target slot A) performs exactly six mutating calls:
//   1 set M(PENDING)  2 commit  3 set A  4 commit  5 set M(AWAITING_ACK)  6 commit
// and ends with the generation "verified, awaiting ACK". The caller's ACK is a
// separate operation of two more mutating calls: 1 set M(IDLE)  2 commit.

struct Expect {
  PersistenceClass cls;
  uint32_t generation;  // served when cls == CONSISTENT
};

Expect expected(int event, Pub pub) {
  switch (event) {
    case 1:
      if (pub == Pub::NOTHING) return {PersistenceClass::CONSISTENT, 2};
      if (pub == Pub::PARTIAL) return {PersistenceClass::MARKER_CORRUPT, 0};
      return {PersistenceClass::PENDING_RECORD_ABSENT, 0};
    case 2:
      return {PersistenceClass::PENDING_RECORD_ABSENT, 0};
    case 3:
      if (pub == Pub::FULL) return {PersistenceClass::PENDING_RECORD_PRESENT, 0};
      return {PersistenceClass::PENDING_RECORD_ABSENT, 0};
    case 4:
      return {PersistenceClass::PENDING_RECORD_PRESENT, 0};
    case 5:
      if (pub == Pub::NOTHING) return {PersistenceClass::PENDING_RECORD_PRESENT, 0};
      if (pub == Pub::PARTIAL) return {PersistenceClass::MARKER_CORRUPT, 0};
      return {PersistenceClass::AWAITING_ACK, 0};
    default:
      return {PersistenceClass::AWAITING_ACK, 0};
  }
}

bool isSetEvent(int e) { return e == 1 || e == 3 || e == 5; }

void sweepOne(int event, Pub pub, bool power_cut) {
  Rig seeded;
  seed(&seeded);
  Rig r;
  boot(&r);  // second instance on the same (alive) flash, like the Controller would
  const std::vector<uint8_t> a1 = blobOf("A");
  const std::vector<uint8_t> b2 = blobOf("B");
  const std::vector<uint8_t> m2 = blobOf("M");
  g.mut = 0;
  g.sets = 0;
  g.fault_pub = pub;
  g.err_code = 0x1234;
  if (power_cut) g.cut_at = event; else g.err_at = event;

  const SaveResult res = doSave(&r);
  const Expect ex = expected(event, pub);
  char tag[96];
  std::snprintf(tag, sizeof(tag), "%s event %d pub %d", power_cut ? "cut" : "error", event,
                static_cast<int>(pub));
  g_case = tag;

  // Never a false confirmation: a faulted SAVE never reports OK, never a generation.
  CHECK(res.status != SaveStatus::OK);
  CHECK(res.generation == 0);
  // The last confirmed record is never touched (slot B is not the target).
  CHECK(blobOf("B") == b2);
  CHECK(res.previous_record_intact || event == 1);
  // The inactive slot is the only record slot that can change.
  if (event < 3) CHECK(blobOf("A") == a1);

  // The instance that saw the fault is blocked, and the block really stops everything.
  CHECK(r.store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  if (!power_cut) {
    const Flash frozen = g.flash;
    g.calls.clear();
    g.err_at = 0;
    CHECK(doSave(&r).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
    CHECK(doSave(&r).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);  // two consecutive errors
    CHECK(g.calls.empty());
    CHECK(g.flash == frozen);
    // Same flash, same verdict in the same instance.
    const LoadResult same = doLoad(&r);
    CHECK(same.assessment.cls == ex.cls);
    CHECK(same.status == (ex.cls == PersistenceClass::CONSISTENT
                              ? LoadStatus::OK
                              : ex.cls == PersistenceClass::AWAITING_ACK ? LoadStatus::ACKNOWLEDGMENT_REQUIRED
                                                                         : LoadStatus::RECONCILIATION_REQUIRED));
    CHECK(r.store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);  // load does not unlock
  }

  // ---- reboot: new instance, same flash ----
  reboot();
  Rig after;
  if (!boot(&after)) return;
  CHECK(after.store.writeState() == WriteState::OPEN);
  const LoadResult l = doLoad(&after);
  CHECK(l.assessment.cls == ex.cls);
  if (ex.cls == PersistenceClass::CONSISTENT) {
    CHECK(l.status == LoadStatus::OK && l.generation == ex.generation);
    // What is served is exactly what the marker attests.
    SaveMarker m;
    CHECK(decodeFlashMarker(&m));
    CHECK(m.state == SaveMarkerState::IDLE && m.acknowledged_generation == l.generation);
    CHECK(l.slot == CalibrationSlot::B);
  } else if (ex.cls == PersistenceClass::AWAITING_ACK) {
    // Generation 3 is verified on flash but was never acknowledged: not served, not
    // promoted, generation 2 (slot B) protected. Only an explicit ACK resolves it.
    CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
    CHECK(!l.assessment.record_available && !l.assessment.save_allowed && l.assessment.ack_allowed);
    const Flash frozen = g.flash;
    g.mut = 0;
    CHECK(doSave(&after).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
    CHECK(doAck(&after, 2).status == AckStatus::GENERATION_MISMATCH);
    CHECK(g.mut == 0 && g.flash == frozen);
    CHECK(blobOf("B") == b2);
    CHECK(doAck(&after, 3).status == AckStatus::OK);
    const LoadResult ok3 = doLoad(&after);
    CHECK(ok3.status == LoadStatus::OK && ok3.generation == 3 && ok3.slot == CalibrationSlot::A);
    CHECK(blobOf("B") == b2);
  } else {
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(!l.assessment.record_available && !l.assessment.save_allowed);
    CHECK(l.assessment.reconciliation_required);
    // Nothing is promoted and no SAVE may overwrite anything before reconciliation.
    const Flash frozen = g.flash;
    g.mut = 0;
    const SaveResult refused = doSave(&after);
    CHECK(refused.status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(g.mut == 0 && g.flash == frozen);
    CHECK(blobOf("B") == b2);

    // Explicit reconciliation (through the backend, no Controller): adopt the
    // confirmed generation 2, which is still on disk.
    CHECK(reconcile(&after, ReconciliationAction::ADOPT_VALID_RECORD, 2));
    const LoadResult ok2 = doLoad(&after);
    CHECK(ok2.status == LoadStatus::OK && ok2.generation == 2 && ok2.slot == CalibrationSlot::B);
    CHECK(blobOf("B") == b2);
  }

  // After a legitimate state, the next SAVE succeeds with a strictly higher
  // generation, written to the slot that is not the confirmed one.
  const uint32_t served = doLoad(&after).generation;
  const SaveResult next = doSave(&after);
  CHECK(next.status == SaveStatus::OK);
  CHECK(next.generation > served);
  CHECK(next.slot == (served == 3 ? CalibrationSlot::B : CalibrationSlot::A));
  CHECK(doLoad(&after).status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);  // verified, not yet acknowledged
  CHECK(doAck(&after, next.generation).status == AckStatus::OK);
  CHECK(doLoad(&after).generation == next.generation);
  (void)m2;
}

void test_fault_sweep() {
  g_case = "fault sweep";
  for (int event = 1; event <= 6; ++event) {
    const Pub pubs[] = {Pub::NOTHING, Pub::PARTIAL, Pub::FULL};
    for (Pub pub : pubs) {
      if (!isSetEvent(event) && pub != Pub::NOTHING) continue;  // commits publish nothing new
      sweepOne(event, pub, /*power_cut=*/true);
      sweepOne(event, pub, /*power_cut=*/false);
    }
  }
}

// ---------------------------------------------------------------------------
// Named scenarios from the P2.4 brief that the sweep does not read-fault
// ---------------------------------------------------------------------------

void test_clean_save_step_order() {
  g_case = "clean SAVE call order";
  Rig r;
  seed(&r);
  g.calls.clear();
  const SaveResult s = doSave(&r);
  CHECK(s.status == SaveStatus::OK && s.generation == 3 && s.slot == CalibrationSlot::A);
  CHECK(g.mut == 6);
  // Mutations in order: PENDING marker, record, AWAITING_ACK marker - each committed.
  std::vector<std::string> mutations;
  for (const std::string& c : g.calls) {
    if (c.rfind("set:", 0) == 0 || c == "commit") mutations.push_back(c);
  }
  const std::vector<std::string> want = {"set:M", "commit", "set:A", "commit", "set:M", "commit"};
  CHECK(mutations == want);
  SaveMarker m;
  CHECK(decodeFlashMarker(&m) && m.state == SaveMarkerState::AWAITING_ACK &&
        m.acknowledged_generation == 2 && m.begun_generation == 3);
  // The ACK is its own two-mutation operation and writes the marker only.
  const std::vector<uint8_t> a3 = blobOf("A"), b2 = blobOf("B");
  g.calls.clear();
  g.mut = 0;
  CHECK(doAck(&r, 3).status == AckStatus::OK);
  CHECK(g.mut == 2);
  std::vector<std::string> ack_mutations;
  for (const std::string& c : g.calls) {
    if (c.rfind("set:", 0) == 0 || c == "commit") ack_mutations.push_back(c);
  }
  const std::vector<std::string> want_ack = {"set:M", "commit"};
  CHECK(ack_mutations == want_ack);
  CHECK(blobOf("A") == a3 && blobOf("B") == b2);
  CHECK(decodeFlashMarker(&m) && m.state == SaveMarkerState::IDLE &&
        m.acknowledged_generation == 3 && m.begun_generation == 3);
  CHECK(g.label_violations == 0);
}

void test_readback_failures() {
  g_case = "read-back failures";
  struct Case {
    const char* key;
    int min_sets;
    PersistenceClass cls_after_reboot;
    SaveStatus status;
  };
  const Case cases[] = {
      {"M", 1, PersistenceClass::PENDING_RECORD_ABSENT, SaveStatus::MARKER_VERIFY_FAILED},   // PENDING read-back
      {"A", 2, PersistenceClass::PENDING_RECORD_PRESENT, SaveStatus::READBACK_FAILED},       // record read-back
      {"M", 3, PersistenceClass::AWAITING_ACK, SaveStatus::MARKER_VERIFY_FAILED},            // AWAITING_ACK read-back
  };
  for (const Case& c : cases) {
    Rig seeded;
    seed(&seeded);
    Rig r;
    boot(&r);
    const std::vector<uint8_t> b2 = blobOf("B");
    g.get_error = 0x1234;
    g.get_error_key = c.key;
    g.get_error_min_sets = c.min_sets;
    const SaveResult res = doSave(&r);
    g_case = c.key;
    CHECK(res.status == c.status);
    CHECK(res.generation == 0);  // no OK without a verified read-back
    CHECK(r.store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    CHECK(blobOf("B") == b2);
    g.get_error = ESP_OK;
    CHECK(doSave(&r).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);

    reboot();
    Rig after;
    boot(&after);
    const LoadResult l = doLoad(&after);
    CHECK(l.assessment.cls == c.cls_after_reboot);
    if (c.cls_after_reboot == PersistenceClass::AWAITING_ACK) {
      // The AWAITING_ACK marker WAS published although SAVE reported an error: the
      // caller never got SAVE=OK, so a reboot must not treat generation 3 as
      // confirmed, and generation 2 stays protected.
      CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED && !l.assessment.record_available);
      CHECK(doSave(&after).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
      CHECK(blobOf("B") == b2);
    } else {
      CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
      CHECK(doSave(&after).status == SaveStatus::RECONCILIATION_REQUIRED);
    }
  }
}

void test_pending_block_survives_reboot_and_retries() {
  g_case = "retries never erase the confirmed record";
  Rig seeded;
  seed(&seeded);
  Rig r;
  boot(&r);
  const std::vector<uint8_t> b2 = blobOf("B");
  g.err_at = 4;  // commit after the record write fails
  g.err_code = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
  CHECK(doSave(&r).status != SaveStatus::OK);
  g.err_at = 0;
  for (int i = 0; i < 5; ++i) CHECK(doSave(&r).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);

  // Reboot after reboot: the RAM block is gone, the persistent marker still refuses.
  for (int i = 0; i < 3; ++i) {
    reboot();
    Rig again;
    boot(&again);
    CHECK(again.store.writeState() == WriteState::OPEN);
    CHECK(doSave(&again).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(doLoad(&again).status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(blobOf("B") == b2);
  }
  // The valid-but-unconfirmed generation 3 is never served; declaring "nothing
  // confirmed" is also an explicit act and leaves the data to the operator.
  reboot();
  Rig fin;
  boot(&fin);
  CHECK(reconcile(&fin, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0));
  const LoadResult l = doLoad(&fin);
  CHECK(l.status == LoadStatus::NOT_FOUND && l.assessment.cls == PersistenceClass::NOTHING_ACKNOWLEDGED);
  const SaveResult s = doSave(&fin);
  CHECK(s.status == SaveStatus::OK && s.generation == 4);
}

struct FlakyOpen : CalibrationRecordStorage {
  CalibrationRecordNvsBackend inner;
  bool fail_write_open = true;
  StorageIoStatus read(CalibrationSlot s, uint8_t* b, size_t c, size_t* n) override {
    return inner.read(s, b, c, n);
  }
  StorageIoStatus readMarker(uint8_t* b, size_t c, size_t* n) override {
    return inner.readMarker(b, c, n);
  }
  StorageIoStatus write(CalibrationSlot s, const uint8_t* d, size_t n) override {
    return gate([&] { return inner.write(s, d, n); });
  }
  StorageIoStatus writeMarker(const uint8_t* d, size_t n) override {
    return gate([&] { return inner.writeMarker(d, n); });
  }
  template <typename F>
  StorageIoStatus gate(F f) {
    if (!fail_write_open) return f();
    g.open_error = ESP_ERR_NVS_INVALID_STATE;
    const StorageIoStatus st = f();
    g.open_error = ESP_OK;
    return st;
  }
};

void test_open_failure_is_certain_and_retryable() {
  g_case = "open failure allows retry";
  Rig seeded;
  seed(&seeded);
  Rig r;
  boot(&r);
  const Flash snapshot = g.flash;

  // Open fails at the very first (scan) read: refused before any write.
  g.open_error = ESP_ERR_NVS_INVALID_STATE;
  SaveResult res = doSave(&r);
  CHECK(res.status == SaveStatus::STORAGE_UNUSABLE);
  CHECK(r.store.writeState() == WriteState::OPEN);
  CHECK(g.flash == snapshot);
  g.open_error = ESP_OK;

  // Only the write-phase open fails: certain, nothing published, retry allowed.
  FlakyOpen flaky;
  CHECK(flaky.inner.begin() == NvsInitStatus::READY);
  CalibrationRecordStore store(&flaky);
  CalibrationRecord tmp;
  (void)tmp;
  res = store.save(golden::goldenRecord(0), profile);
  // The PENDING marker was never written: the store is not blocked and flash untouched.
  CHECK(res.status == SaveStatus::MARKER_WRITE_FAILED && res.io == StorageIoStatus::NOT_MODIFIED);
  CHECK(store.writeState() == WriteState::OPEN);
  CHECK(g.flash == snapshot);
  flaky.fail_write_open = false;
  res = store.save(golden::goldenRecord(0), profile);
  CHECK(res.status == SaveStatus::OK && res.generation == 3 && res.slot == CalibrationSlot::A);
}

// ---------------------------------------------------------------------------
// P2.4.1: durable acknowledgment at the NVS-stub level
// ---------------------------------------------------------------------------

// A/1 acknowledged, nothing else. The next SAVE (generation 2) targets slot B.
void seedOne(Rig* r) {
  resetAll();
  boot(r);
  doSaveAck(r);
  g.mut = 0;
  g.sets = 0;
  g.calls.clear();
}

// The review finding: A/1 acknowledged; B/2 written and verified; the last marker is
// published but the caller never gets SAVE=OK (or never ACKs); reboot; a new SAVE
// with a power cut in its write. A/1 must stay byte-for-byte intact, and B/2 must
// not be promoted.
void test_review_finding_regression() {
  g_case = "review finding";
  struct Variant {
    const char* name;
    int cut_at, err_at;
    Pub pub;
    const char* get_key;
    int get_min_sets;
    bool expect_ok;
  };
  const Variant variants[] = {
      {"marker commit error after publication (MARKER_WRITE_FAILED)", 0, 6, Pub::NOTHING, nullptr, 0, false},
      {"marker set error after publication (MARKER_WRITE_FAILED)", 0, 5, Pub::FULL, nullptr, 0, false},
      {"marker read-back error (MARKER_VERIFY_FAILED)", 0, 0, Pub::NOTHING, "M", 3, false},
      {"power loss after marker publication (commit)", 6, 0, Pub::NOTHING, nullptr, 0, false},
      {"power loss after marker publication (set)", 5, 0, Pub::FULL, nullptr, 0, false},
      {"SAVE=OK delivered, ACK never sent", 0, 0, Pub::NOTHING, nullptr, 0, true},
  };
  for (const Variant& v : variants) {
    g_case = v.name;
    Rig first;
    seedOne(&first);
    const std::vector<uint8_t> a1 = blobOf("A");
    g.fault_pub = v.pub;
    g.cut_at = v.cut_at;
    g.err_at = v.err_at;
    if (v.get_key != nullptr) {
      g.get_error = 0x1234;
      g.get_error_key = v.get_key;
      g.get_error_min_sets = v.get_min_sets;
    }
    const SaveResult res = doSave(&first);
    CHECK((res.status == SaveStatus::OK) == v.expect_ok);
    CHECK(blobOf("A") == a1);
    SaveMarker m;
    CHECK(decodeFlashMarker(&m) && m.state == SaveMarkerState::AWAITING_ACK &&
          m.acknowledged_generation == 1 && m.begun_generation == 2);
    const std::vector<uint8_t> b2 = blobOf("B");
    const std::vector<uint8_t> marker = blobOf("M");

    for (int boots = 0; boots < 3; ++boots) {
      reboot();
      Rig after;
      if (!boot(&after)) return;
      const LoadResult l = doLoad(&after);
      CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);  // B/2 is NOT confirmed
      CHECK(l.assessment.cls == PersistenceClass::AWAITING_ACK && !l.assessment.record_available);
      for (int cut : {1, 3, 5}) {  // a new SAVE, cut where its PENDING / record / marker write would be
        reboot();
        Rig again;
        if (!boot(&again)) return;
        g.fault_pub = Pub::PARTIAL;
        g.cut_at = cut;
        g.calls.clear();
        CHECK(doSave(&again).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
        CHECK(g.mut == 0);
        for (const std::string& c : g.calls) CHECK(c.rfind("set:", 0) != 0 && c != "commit");
        CHECK(blobOf("A") == a1);  // the acknowledged generation, byte for byte
        CHECK(blobOf("B") == b2);
        CHECK(blobOf("M") == marker);
      }
    }
    // The generation is not lost: the caller can still acknowledge it. Only then is
    // slot A reusable; the next SAVE writes A, never B.
    reboot();
    Rig fin;
    if (!boot(&fin)) return;
    CHECK(blobOf("A") == a1);
    CHECK(doAck(&fin, 2).status == AckStatus::OK);
    CHECK(blobOf("A") == a1 && blobOf("B") == b2);  // the ACK writes the marker only
    const LoadResult served = doLoad(&fin);
    CHECK(served.status == LoadStatus::OK && served.generation == 2 && served.slot == CalibrationSlot::B);
    const SaveResult next = doSave(&fin);
    CHECK(next.status == SaveStatus::OK && next.generation == 3 && next.slot == CalibrationSlot::A);
    CHECK(blobOf("B") == b2);
  }
}

// Fault on the ACK itself (set M(IDLE), commit). The acknowledged slot is never
// touched; the conservative outcome is never OK; a reboot reads the real state.
void test_ack_fault_sweep() {
  g_case = "ack fault sweep";
  for (int event = 1; event <= 2; ++event) {
    for (Pub pub : {Pub::NOTHING, Pub::PARTIAL, Pub::FULL}) {
      if (event == 2 && pub != Pub::NOTHING) continue;
      for (int power_cut = 0; power_cut < 2; ++power_cut) {
        char tag[80];
        std::snprintf(tag, sizeof(tag), "ack %s event %d pub %d", power_cut ? "cut" : "error", event,
                      static_cast<int>(pub));
        g_case = tag;
        Rig seeded;
        seed(&seeded);
        Rig r;
        boot(&r);
        CHECK(doSave(&r).status == SaveStatus::OK);  // gen 3 -> slot A, awaiting
        const std::vector<uint8_t> a3 = blobOf("A"), b2 = blobOf("B");
        g.mut = 0;
        g.fault_pub = pub;
        if (power_cut) g.cut_at = event; else g.err_at = event;
        const AckResult ack = doAck(&r, 3);
        CHECK(ack.status == AckStatus::MARKER_WRITE_FAILED);  // never OK on a fault
        CHECK(blobOf("A") == a3 && blobOf("B") == b2);
        if (!power_cut) {
          g.err_at = 0;
          CHECK(doSave(&r).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);  // uncertain: no more writes
        }

        reboot();
        Rig after;
        if (!boot(&after)) return;
        g.calls.clear();
        const LoadResult l = doLoad(&after);
        PersistenceClass expect = PersistenceClass::CONSISTENT;  // the ACK did land (reply lost)
        if (event == 1 && pub == Pub::NOTHING) expect = PersistenceClass::AWAITING_ACK;
        if (event == 1 && pub == Pub::PARTIAL) expect = PersistenceClass::MARKER_CORRUPT;
        CHECK(l.assessment.cls == expect);
        if (expect == PersistenceClass::AWAITING_ACK) {
          CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
          reboot();
          Rig retry;
          if (!boot(&retry)) return;
          CHECK(doSave(&retry).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
          CHECK(g.mut == 0 && blobOf("B") == b2);
          CHECK(doAck(&retry, 3).status == AckStatus::OK);  // the ACK can simply be repeated
          CHECK(doLoad(&retry).generation == 3);
        } else if (expect == PersistenceClass::CONSISTENT) {
          CHECK(l.status == LoadStatus::OK && l.generation == 3 && l.slot == CalibrationSlot::A);
          const Flash frozen = g.flash;
          g.mut = 0;
          CHECK(doAck(&after, 3).status == AckStatus::ALREADY_ACKNOWLEDGED);
          CHECK(g.mut == 0 && g.flash == frozen);
          const SaveResult next = doSave(&after);  // only now is slot B reusable
          CHECK(next.status == SaveStatus::OK && next.generation == 4 && next.slot == CalibrationSlot::B);
          CHECK(blobOf("A") == a3);
        } else {
          CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
          const Flash frozen = g.flash;
          g.mut = 0;
          CHECK(doSave(&after).status == SaveStatus::RECONCILIATION_REQUIRED);
          CHECK(doAck(&after, 3).status == AckStatus::RECONCILIATION_REQUIRED);
          CHECK(g.mut == 0 && g.flash == frozen);
          CHECK(reconcile(&after, ReconciliationAction::ADOPT_VALID_RECORD, 3));
          CHECK(doLoad(&after).generation == 3);
        }
      }
    }
  }
  {  // ACK marker read-back fails although the marker landed
    g_case = "ack read-back failure";
    Rig seeded;
    seed(&seeded);
    Rig r;
    boot(&r);
    CHECK(doSave(&r).status == SaveStatus::OK);
    g.sets = 0;
    g.get_error = 0x1234;
    g.get_error_key = "M";
    g.get_error_min_sets = 1;
    CHECK(doAck(&r, 3).status == AckStatus::MARKER_VERIFY_FAILED);
    CHECK(r.store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    reboot();
    Rig after;
    boot(&after);
    CHECK(doLoad(&after).assessment.cls == PersistenceClass::CONSISTENT);
  }
}

// First SAVE on an empty partition: every interruption is distinguishable at boot,
// and no calibration is declared available before an ACK.
void test_first_save_interruptions() {
  g_case = "first SAVE interruptions";
  for (int event = 1; event <= 6; ++event) {
    for (Pub pub : {Pub::NOTHING, Pub::PARTIAL, Pub::FULL}) {
      if (event % 2 == 0 && pub != Pub::NOTHING) continue;
      char tag[64];
      std::snprintf(tag, sizeof(tag), "first SAVE cut %d pub %d", event, static_cast<int>(pub));
      g_case = tag;
      resetAll();
      Rig r;
      boot(&r);
      g.fault_pub = pub;
      g.cut_at = event;
      CHECK(doSave(&r).status != SaveStatus::OK);
      reboot();
      Rig after;
      if (!boot(&after)) return;
      const LoadResult l = doLoad(&after);
      PersistenceClass expect;
      switch (event) {
        case 1: expect = pub == Pub::NOTHING ? PersistenceClass::NEVER_INITIALIZED_OR_ERASED
                         : pub == Pub::PARTIAL ? PersistenceClass::MARKER_CORRUPT
                                               : PersistenceClass::PENDING_RECORD_ABSENT; break;
        case 2: expect = PersistenceClass::PENDING_RECORD_ABSENT; break;
        case 3: expect = pub == Pub::FULL ? PersistenceClass::PENDING_RECORD_PRESENT
                                          : PersistenceClass::PENDING_RECORD_ABSENT; break;
        case 4: expect = PersistenceClass::PENDING_RECORD_PRESENT; break;
        case 5: expect = pub == Pub::NOTHING ? PersistenceClass::PENDING_RECORD_PRESENT
                         : pub == Pub::PARTIAL ? PersistenceClass::MARKER_CORRUPT
                                               : PersistenceClass::AWAITING_ACK; break;
        default: expect = PersistenceClass::AWAITING_ACK; break;
      }
      CHECK(l.assessment.cls == expect);
      CHECK(l.status != LoadStatus::OK && !l.assessment.record_available);
      if (expect == PersistenceClass::AWAITING_ACK) {
        CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
        CHECK(l.assessment.acknowledged_generation == 0);
        CHECK(doSave(&after).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
        CHECK(doAck(&after, 1).status == AckStatus::OK);
        CHECK(doLoad(&after).status == LoadStatus::OK);
      } else if (expect == PersistenceClass::NEVER_INITIALIZED_OR_ERASED) {
        CHECK(l.status == LoadStatus::NOT_FOUND);
        CHECK(doSave(&after).status == SaveStatus::OK);
      } else {
        CHECK(!l.assessment.save_allowed);
        CHECK(doSave(&after).status == SaveStatus::RECONCILIATION_REQUIRED);
      }
    }
  }
}

void test_acknowledged_generation_lost_and_reconciliation() {
  g_case = "acknowledged generation lost";
  for (int kind = 0; kind < 2; ++kind) {  // 0: erased, 1: damaged
    Rig seeded;
    seed(&seeded);
    Rig r;
    boot(&r);
    CHECK(doSave(&r).status == SaveStatus::OK);  // gen 3 awaiting in A; acknowledged is B/2
    const std::vector<uint8_t> a3 = blobOf("A");
    if (kind == 0) g.flash[kNs].erase("B"); else g.flash[kNs]["B"][700] ^= 0x01;
    reboot();
    Rig after;
    boot(&after);
    const LoadResult l = doLoad(&after);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);  // reported, no silent fallback
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED && !l.assessment.record_available);
    const Flash frozen = g.flash;
    g.mut = 0;
    CHECK(doSave(&after).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(doAck(&after, 3).status == AckStatus::RECONCILIATION_REQUIRED);
    CHECK(g.mut == 0 && g.flash == frozen);
    if (kind == 0) {
      CHECK(reconcile(&after, ReconciliationAction::ADOPT_VALID_RECORD, 3));  // operator takes the verified one
      CHECK(doLoad(&after).generation == 3 && blobOf("A") == a3);
    } else {
      CHECK(reconcile(&after, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0));
      CHECK(doLoad(&after).status == LoadStatus::NOT_FOUND);
    }
  }
  {  // reconciling an AWAITING_ACK state: ADOPT the previous one discards the verified one
    g_case = "reconcile awaiting";
    Rig seeded;
    seed(&seeded);
    Rig r;
    boot(&r);
    CHECK(doSave(&r).status == SaveStatus::OK);
    const std::vector<uint8_t> b2 = blobOf("B");
    reboot();
    Rig after;
    boot(&after);
    CHECK(reconcile(&after, ReconciliationAction::ADOPT_VALID_RECORD, 2));
    const LoadResult l = doLoad(&after);
    CHECK(l.status == LoadStatus::OK && l.generation == 2 && blobOf("B") == b2);
    const SaveResult next = doSave(&after);
    CHECK(next.status == SaveStatus::OK && next.generation == 4 && next.slot == CalibrationSlot::A);  // 3 never reused
    CHECK(blobOf("B") == b2);
  }
}

void test_two_consecutive_errors_keep_acknowledged_slot() {
  g_case = "two consecutive errors";
  Rig first;
  seedOne(&first);
  const std::vector<uint8_t> a1 = blobOf("A");
  g.err_at = 6;
  CHECK(doSave(&first).status == SaveStatus::MARKER_WRITE_FAILED);  // error 1: marker published, error returned
  g.err_at = 0;
  CHECK(doSave(&first).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);  // error 2, same instance
  for (int i = 0; i < 3; ++i) {
    reboot();
    Rig again;
    boot(&again);
    g.cut_at = 1;
    g.fault_pub = Pub::PARTIAL;
    CHECK(doSave(&again).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);  // error 2, new instance
    CHECK(doSave(&again).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
    CHECK(g.mut == 0);
    CHECK(blobOf("A") == a1);
  }
}

void test_boundaries_of_the_backend() {
  g_case = "backend boundaries";
  // Every partition-naming call in this whole binary named "matdog_nvs".
  CHECK(g.label_violations == 0);
  CHECK(std::string(kMatdogNvsPartitionLabel) == "matdog_nvs");
  CHECK(kMatdogNvsPartitionAddress == 0xFE0000u && kMatdogNvsPartitionSize == 0x10000u);
}

}  // namespace

int main() {
  test_begin_ready_and_idempotent();
  test_begin_failures_are_distinct_and_leave_flash_alone();
  test_never_initialized_backend_is_fail_closed();
  test_absent_and_round_trip();
  test_error_mapping();
  test_first_install_and_total_loss();
  test_marker_and_record_incompatibilities();
  test_clean_save_step_order();
  test_fault_sweep();
  test_readback_failures();
  test_pending_block_survives_reboot_and_retries();
  test_open_failure_is_certain_and_retryable();
  test_review_finding_regression();
  test_ack_fault_sweep();
  test_first_save_interruptions();
  test_acknowledged_generation_lost_and_reconciliation();
  test_two_consecutive_errors_keep_acknowledged_slot();
  test_boundaries_of_the_backend();
  std::printf("test_calibration_record_nvs_backend: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
