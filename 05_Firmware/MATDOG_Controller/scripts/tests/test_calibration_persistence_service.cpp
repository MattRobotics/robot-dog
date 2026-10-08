// Offline tests for the Controller-facing persistence service
// (src/calibration/CalibrationPersistenceService.*): boot LOAD classification,
// read-only STATUS, SAVE / ACK entry points, and the explicit reconciliation,
// all against a deterministic fake storage with fault injection. No flash, no
// NVS, no hardware.
//
// What is proven here: every boot situation maps to one stable verdict and
// "available" is true only for an acknowledged, valid record with no write in
// doubt; a backend that is not READY is never read or written; LOAD and the
// snapshot never write and never keep the decoded record; the ACK paths
// (ok / repeat / wrong generation / invalid record / uncertain) end in the
// documented state; reconciliation refuses without confirmation, never drops
// the last acknowledged generation silently, and verifies its marker by
// read-back; an uncertain write blocks all further writes.

#include <cstdio>
#include <cstring>
#include <vector>

#include "calibration_record_golden.h"
#include "../../src/calibration/CalibrationPersistenceService.h"

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

#define CHECK_EQ(actual, expected)                                         \
  do {                                                                     \
    ++g_checks;                                                            \
    const long a_ = (long)(actual);                                        \
    const long e_ = (long)(expected);                                      \
    if (a_ != e_) {                                                        \
      ++g_failures;                                                        \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,  \
                  __FILE__, __LINE__, #actual, a_, e_);                    \
    }                                                                      \
  } while (0)

namespace {

using golden::boundProfile;
using golden::goldenRecord;

enum class MarkerFault {
  NONE,
  NOT_MODIFIED,   // storage certifies it did nothing
  IO_NOTHING,     // error, nothing reached storage (uncertain)
  AFTER_DATA,     // all bytes reached storage, error reported
  SILENT,         // reports OK, one byte wrong
};

class FakeStorage : public CalibrationRecordStorage {
 public:
  bool present[2] = {false, false};
  std::vector<uint8_t> data[2];
  bool marker_present = false;
  std::vector<uint8_t> marker_data;
  bool fail_marker_read = false;
  bool fail_slot_read[2] = {false, false};

  MarkerFault marker_fault = MarkerFault::NONE;
  int marker_fault_at = 0;  // 1-based marker write that faults

  int reads = 0;
  int slot_writes = 0;
  int marker_writes = 0;
  int calls() const { return reads + slot_writes + marker_writes; }
  void clearCounters() { reads = slot_writes = marker_writes = 0; }

  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity, size_t* length) override {
    ++reads;
    const int i = static_cast<int>(slot);
    *length = 0;
    if (fail_slot_read[i]) return StorageIoStatus::IO_ERROR;
    if (!present[i]) return StorageIoStatus::ABSENT;
    if (data[i].size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, data[i].data(), data[i].size());
    *length = data[i].size();
    return StorageIoStatus::OK;
  }
  StorageIoStatus readMarker(uint8_t* buffer, size_t capacity, size_t* length) override {
    ++reads;
    *length = 0;
    if (fail_marker_read) return StorageIoStatus::IO_ERROR;
    if (!marker_present) return StorageIoStatus::ABSENT;
    if (marker_data.size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, marker_data.data(), marker_data.size());
    *length = marker_data.size();
    return StorageIoStatus::OK;
  }
  StorageIoStatus write(CalibrationSlot slot, const uint8_t* bytes, size_t length) override {
    ++slot_writes;
    const int i = static_cast<int>(slot);
    present[i] = true;
    data[i].assign(bytes, bytes + length);
    return StorageIoStatus::OK;
  }
  StorageIoStatus writeMarker(const uint8_t* bytes, size_t length) override {
    ++marker_writes;
    const MarkerFault f = marker_writes == marker_fault_at ? marker_fault : MarkerFault::NONE;
    switch (f) {
      case MarkerFault::NONE:
        break;
      case MarkerFault::NOT_MODIFIED:
        return StorageIoStatus::NOT_MODIFIED;
      case MarkerFault::IO_NOTHING:
        return StorageIoStatus::IO_ERROR;
      case MarkerFault::AFTER_DATA:
        marker_present = true;
        marker_data.assign(bytes, bytes + length);
        return StorageIoStatus::IO_ERROR;
      case MarkerFault::SILENT:
        marker_present = true;
        marker_data.assign(bytes, bytes + length);
        marker_data[12] ^= 0x10;
        return StorageIoStatus::OK;
    }
    marker_present = true;
    marker_data.assign(bytes, bytes + length);
    return StorageIoStatus::OK;
  }

  void putSlot(int slot, const CalibrationRecord& r) {
    std::vector<uint8_t> b(kCalibrationRecordV1EncodedBytes);
    size_t n = 0;
    encodeCalibrationRecord(r, b.data(), b.size(), &n);
    b.resize(n);
    present[slot] = true;
    data[slot] = b;
  }
  void putMarker(SaveMarkerState state, uint32_t acknowledged, uint32_t begun) {
    uint8_t b[kSaveMarkerV1Bytes];
    size_t n = 0;
    SaveMarker m;
    m.state = state;
    m.acknowledged_generation = acknowledged;
    m.begun_generation = begun;
    encodeSaveMarker(m, b, sizeof(b), &n);
    marker_present = true;
    marker_data.assign(b, b + n);
  }
  bool decodedMarker(SaveMarker* m) const {
    return marker_present && decodeSaveMarker(marker_data.data(), marker_data.size(), m) == SaveMarkerStatus::OK;
  }
  uint32_t checksum() const {
    uint32_t h = 2166136261u;
    auto mix = [&h](const std::vector<uint8_t>& v) { for (uint8_t c : v) h = (h ^ c) * 16777619u; };
    mix(data[0]);
    mix(data[1]);
    mix(marker_data);
    return h ^ (present[0] ? 1u : 0u) ^ (present[1] ? 2u : 0u) ^ (marker_present ? 4u : 0u);
  }
};

// Storage that already holds `n` generations written by a plain store and acknowledged.
void seed(FakeStorage* fs, int n) {
  CalibrationRecordStore store(fs);
  for (int i = 0; i < n; ++i) {
    const SaveResult r = store.save(goldenRecord(0), boundProfile());
    CHECK(r.status == SaveStatus::OK);
    CHECK(store.acknowledge(r.generation, boundProfile()).status == AckStatus::OK);
  }
  fs->clearCounters();
}

// A service the way Controller::begin() sets it up: begin(READY) then the boot LOAD.
struct Booted {
  FakeStorage* fs;
  CalibrationPersistenceService svc;
  explicit Booted(FakeStorage* storage, NvsInitStatus nvs = NvsInitStatus::READY) : fs(storage), svc(storage) {
    svc.begin(nvs, 0);
    svc.load(boundProfile());
  }
  const PersistenceSnapshot& snap() const { return svc.snapshot(); }
};

bool scratchIsZero(CalibrationPersistenceService& svc) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(svc.scratchRecord());
  for (size_t i = 0; i < sizeof(CalibrationRecord); ++i) {
    if (p[i] != 0) return false;
  }
  return true;
}

void corruptCrc(std::vector<uint8_t>* b) { (*b)[b->size() / 2] ^= 0x01; }

void refreshCrc(std::vector<uint8_t>* b) {
  const uint32_t crc = calibrationCrc32(b->data(), b->size() - 4);
  for (int i = 0; i < 4; ++i) (*b)[b->size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
}

// ---------------------------------------------------------------------------
// BOOT LOAD

void test_boot_no_record() {
  g_case = "boot: no record";
  FakeStorage fs;
  Booted b(&fs);
  CHECK(b.snap().nvs == NvsInitStatus::READY);
  CHECK(b.snap().verdict == PersistenceVerdict::NO_RECORD);
  CHECK(b.snap().boot_verdict == PersistenceVerdict::NO_RECORD);
  CHECK_EQ(b.snap().load_count, 1);
  CHECK(!b.svc.calibrationAvailable());
  CHECK(b.snap().assessment.save_allowed);  // a first install may SAVE
  CHECK_EQ(fs.slot_writes + fs.marker_writes, 0);
}

void test_boot_valid_acknowledged() {
  g_case = "boot: valid acknowledged";
  FakeStorage fs;
  seed(&fs, 1);
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
  CHECK(b.snap().load_status == LoadStatus::OK);
  CHECK_EQ(b.snap().loaded_generation, 1);
  CHECK_EQ(b.snap().assessment.acknowledged_generation, 1);
  CHECK(b.svc.calibrationAvailable());
  // LOAD never writes and does not retain the decoded record.
  CHECK_EQ(fs.slot_writes + fs.marker_writes, 0);
  CHECK(scratchIsZero(b.svc));

  // Two generations: the newer acknowledged one is selected.
  FakeStorage fs2;
  seed(&fs2, 2);
  Booted b2(&fs2);
  CHECK(b2.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
  CHECK_EQ(b2.snap().loaded_generation, 2);
}

void test_boot_nvs_not_ready_touches_nothing() {
  g_case = "boot: nvs not ready";
  const NvsInitStatus bad[] = {NvsInitStatus::PARTITION_MISSING, NvsInitStatus::PARTITION_GEOMETRY_MISMATCH,
                               NvsInitStatus::NO_FREE_PAGES,     NvsInitStatus::NEW_VERSION_FOUND,
                               NvsInitStatus::INIT_FAILED,       NvsInitStatus::OPEN_FAILED,
                               NvsInitStatus::NOT_INITIALIZED};
  for (NvsInitStatus st : bad) {
    FakeStorage fs;
    seed(&fs, 1);  // there IS a perfectly valid record behind a dead backend: it must not be exposed
    const uint32_t before = fs.checksum();
    Booted b(&fs, st);
    CHECK(b.snap().verdict == PersistenceVerdict::NVS_UNAVAILABLE);
    CHECK(b.snap().nvs == st);
    CHECK(!b.svc.calibrationAvailable());
    CHECK_EQ(fs.calls(), 0);  // never read, never written
    CHECK_EQ(fs.checksum(), before);

    CalibrationRecord rec = goldenRecord(0);
    const ServiceSaveResult s = b.svc.save(rec, boundProfile());
    CHECK(s.guard == ServiceGuard::NVS_NOT_READY);
    const ServiceAckResult a = b.svc.acknowledge(1, boundProfile());
    CHECK(a.guard == ServiceGuard::NVS_NOT_READY);
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, true, boundProfile());
    CHECK(r.status == ReconcileStatus::NVS_NOT_READY);
    CHECK_EQ(fs.calls(), 0);
    CHECK_EQ(fs.checksum(), before);
  }
}

void test_boot_pending_marker() {
  g_case = "boot: SAVE pending";
  FakeStorage fs;
  seed(&fs, 1);
  fs.putMarker(SaveMarkerState::PENDING, 1, 2);  // an interrupted SAVE of generation 2
  fs.putSlot(1, goldenRecord(2));
  const uint32_t before = fs.checksum();
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::SAVE_PENDING);
  CHECK_EQ(b.snap().assessment.pending_generation, 2);
  CHECK(!b.svc.calibrationAvailable());
  CHECK(b.snap().assessment.reconciliation_required);
  CHECK_EQ(fs.checksum(), before);  // no automatic recovery, no ACK
  const ServiceSaveResult s = b.svc.save(goldenRecord(0), boundProfile());
  CHECK(s.guard == ServiceGuard::OK);
  CHECK(s.save.status != SaveStatus::OK);
  CHECK_EQ(fs.checksum(), before);
}

void test_boot_awaiting_ack() {
  g_case = "boot: awaiting ACK";
  FakeStorage fs;
  seed(&fs, 1);
  {
    CalibrationRecordStore store(&fs);
    CHECK(store.save(goldenRecord(0), boundProfile()).status == SaveStatus::OK);
  }
  const uint32_t before = fs.checksum();
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::AWAITING_ACK);
  CHECK_EQ(b.snap().assessment.awaiting_generation, 2);
  CHECK_EQ(b.snap().assessment.acknowledged_generation, 1);
  CHECK(b.snap().assessment.ack_allowed);
  CHECK(!b.svc.calibrationAvailable());  // verified on flash is not acknowledged
  CHECK_EQ(fs.checksum(), before);
  // A new SAVE must not overwrite the protected generation.
  const ServiceSaveResult s = b.svc.save(goldenRecord(0), boundProfile());
  CHECK(s.save.status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK_EQ(fs.checksum(), before);
}

void test_boot_acknowledged_generation_lost() {
  g_case = "boot: acknowledged generation lost";
  FakeStorage fs;
  seed(&fs, 1);
  fs.present[0] = false;
  fs.data[0].clear();
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::ACKNOWLEDGED_GENERATION_LOST);
  CHECK(!b.svc.calibrationAvailable());
  CHECK(b.snap().assessment.reconciliation_required);

  // Same, but the slot is damaged rather than missing.
  FakeStorage fs2;
  seed(&fs2, 1);
  corruptCrc(&fs2.data[0]);
  Booted b2(&fs2);
  CHECK(b2.snap().verdict == PersistenceVerdict::RECORD_CORRUPT);
  CHECK(!b2.svc.calibrationAvailable());
}

void test_boot_marker_problems() {
  g_case = "boot: marker problems";
  {  // marker missing, a record present
    FakeStorage fs;
    seed(&fs, 1);
    fs.marker_present = false;
    fs.marker_data.clear();
    Booted b(&fs);
    CHECK(b.snap().verdict == PersistenceVerdict::MARKER_MISSING);
    CHECK(!b.svc.calibrationAvailable());
  }
  {  // marker corrupt
    FakeStorage fs;
    seed(&fs, 1);
    corruptCrc(&fs.marker_data);
    Booted b(&fs);
    CHECK(b.snap().verdict == PersistenceVerdict::MARKER_CORRUPT);
    CHECK(!b.svc.calibrationAvailable());
  }
  {  // marker of a foreign schema
    FakeStorage fs;
    seed(&fs, 1);
    fs.marker_data[4] = 3;
    refreshCrc(&fs.marker_data);
    Booted b(&fs);
    CHECK(b.snap().verdict == PersistenceVerdict::MARKER_INCOMPATIBLE);
    CHECK(!b.svc.calibrationAvailable());
  }
  {  // marker unreadable
    FakeStorage fs;
    seed(&fs, 1);
    fs.fail_marker_read = true;
    Booted b(&fs);
    CHECK(b.snap().verdict == PersistenceVerdict::STORAGE_ERROR);
    CHECK(!b.svc.calibrationAvailable());
  }
}

void test_boot_incompatible_record() {
  g_case = "boot: incompatible record";
  FakeStorage fs;
  seed(&fs, 1);
  fs.data[0][4] = 2;  // an envelope schema this build does not know
  refreshCrc(&fs.data[0]);
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::RECORD_INCOMPATIBLE);
  CHECK(!b.svc.calibrationAvailable());

  // A record from another geometry: valid envelope, wrong provenance.
  FakeStorage fs2;
  CalibrationRecord foreign = goldenRecord(1);
  foreign.digest[0][0] ^= 0x01;
  {
    actuator::GeometryProvenance p = actuator::geometry_data::kProvenance;
    static const char kDigits[] = "0123456789abcdef";
    char* dst[6] = {p.urdf_sha256, p.mesh_manifest_sha256, p.endpoint_semantic_sha256,
                    p.parking_semantic_sha256, p.safety_policy_semantic_sha256, p.allocation_sha256};
    for (int k = 0; k < 6; ++k) {
      for (int i = 0; i < 32; ++i) {
        dst[k][2 * i] = kDigits[foreign.digest[k][i] >> 4];
        dst[k][2 * i + 1] = kDigits[foreign.digest[k][i] & 15];
      }
      dst[k][64] = '\0';
    }
    foreign.geometry_tag = actuator::geometryProvenanceTag(p);  // coherent: a different geometry
  }
  fs2.putSlot(0, foreign);
  fs2.putMarker(SaveMarkerState::IDLE, 1, 1);
  Booted b2(&fs2);
  CHECK(b2.snap().verdict == PersistenceVerdict::RECORD_INCOMPATIBLE);
  CHECK(!b2.svc.calibrationAvailable());
}

// ---------------------------------------------------------------------------
// STATUS is a snapshot read

void test_status_has_no_side_effects() {
  g_case = "status";
  FakeStorage fs;
  seed(&fs, 2);
  Booted b(&fs);
  fs.clearCounters();
  const uint32_t before = fs.checksum();
  for (int i = 0; i < 5; ++i) {
    const PersistenceSnapshot& s = b.svc.snapshot();
    (void)b.svc.calibrationAvailable();
    (void)b.svc.writesBlocked();
    (void)b.svc.storeWriteState();
    CHECK(s.verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
  }
  CHECK_EQ(fs.calls(), 0);
  CHECK_EQ(fs.checksum(), before);
  CHECK_EQ(b.snap().load_count, 1);
  CHECK(scratchIsZero(b.svc));
}

// ---------------------------------------------------------------------------
// SAVE then ACK

void test_save_then_ack() {
  g_case = "save + ack";
  FakeStorage fs;
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::NO_RECORD);

  CalibrationRecord* rec = b.svc.scratchRecord();
  *rec = goldenRecord(0);
  const ServiceSaveResult s = b.svc.save(*rec, boundProfile());
  CHECK(s.guard == ServiceGuard::OK);
  CHECK(s.save.status == SaveStatus::OK);
  CHECK(!s.uncertain);
  CHECK_EQ(s.save.generation, 1);
  // SAVE is not concluded before the ACK: verified on flash, not available.
  CHECK(b.snap().verdict == PersistenceVerdict::AWAITING_ACK);
  CHECK_EQ(b.snap().assessment.awaiting_generation, 1);
  CHECK(!b.svc.calibrationAvailable());
  CHECK(scratchIsZero(b.svc));  // the refresh dropped the working copy

  const ServiceAckResult a = b.svc.acknowledge(1, boundProfile());
  CHECK(a.guard == ServiceGuard::OK);
  CHECK(a.ack.status == AckStatus::OK);
  CHECK(!a.uncertain);
  CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
  CHECK_EQ(b.snap().loaded_generation, 1);
  CHECK(b.svc.calibrationAvailable());
  SaveMarker m;
  CHECK(fs.decodedMarker(&m));
  CHECK(m.state == SaveMarkerState::IDLE);
  CHECK_EQ(m.acknowledged_generation, 1);
}

void test_second_save_keeps_previous_acknowledged_until_ack() {
  g_case = "second save";
  FakeStorage fs;
  seed(&fs, 1);
  Booted b(&fs);
  std::vector<uint8_t> acked = fs.data[0];
  *b.svc.scratchRecord() = goldenRecord(0);
  const ServiceSaveResult s = b.svc.save(*b.svc.scratchRecord(), boundProfile());
  CHECK(s.save.status == SaveStatus::OK);
  CHECK_EQ(s.save.generation, 2);
  CHECK(s.save.previous_record_intact);
  CHECK(fs.data[0] == acked);  // the acknowledged generation was not touched
  CHECK(b.snap().verdict == PersistenceVerdict::AWAITING_ACK);
  CHECK(!b.svc.calibrationAvailable());
}

void test_ack_outcomes() {
  g_case = "ack outcomes";
  FakeStorage fs;
  seed(&fs, 1);
  {  // nothing awaits an ACK
    Booted b(&fs);
    const ServiceAckResult a = b.svc.acknowledge(2, boundProfile());
    CHECK(a.ack.status == AckStatus::NOT_AWAITING || a.ack.status == AckStatus::GENERATION_MISMATCH);
    CHECK(b.svc.calibrationAvailable());
  }
  {  // duplicate ACK of the acknowledged generation is idempotent
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    const ServiceAckResult a = b.svc.acknowledge(1, boundProfile());
    CHECK(a.ack.status == AckStatus::ALREADY_ACKNOWLEDGED);
    CHECK_EQ(fs.checksum(), before);
    CHECK(b.svc.calibrationAvailable());
  }
  {  // wrong generation while another awaits
    CalibrationRecordStore store(&fs);
    CHECK(store.save(goldenRecord(0), boundProfile()).status == SaveStatus::OK);  // gen 2 awaits
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    const ServiceAckResult a = b.svc.acknowledge(7, boundProfile());
    CHECK(a.ack.status == AckStatus::GENERATION_MISMATCH);
    CHECK(!a.uncertain);
    CHECK_EQ(fs.checksum(), before);
    CHECK(!b.svc.calibrationAvailable());
    const ServiceAckResult z = b.svc.acknowledge(0, boundProfile());
    CHECK(z.ack.status == AckStatus::BAD_ARGUMENT);
    CHECK_EQ(fs.checksum(), before);
    // the right one then works
    CHECK(b.svc.acknowledge(2, boundProfile()).ack.status == AckStatus::OK);
    CHECK(b.svc.calibrationAvailable());
    CHECK_EQ(b.snap().loaded_generation, 2);
    // and repeating it is "already registered", not an error and not a write
    const uint32_t after = fs.checksum();
    CHECK(b.svc.acknowledge(2, boundProfile()).ack.status == AckStatus::ALREADY_ACKNOWLEDGED);
    CHECK_EQ(fs.checksum(), after);
  }
}

void test_ack_invalid_record() {
  g_case = "ack invalid record";
  FakeStorage fs;
  seed(&fs, 1);
  {
    CalibrationRecordStore store(&fs);
    CHECK(store.save(goldenRecord(0), boundProfile()).status == SaveStatus::OK);  // gen 2 -> slot B
  }
  corruptCrc(&fs.data[1]);  // the awaiting record rotted after the SAVE
  Booted b(&fs);
  const uint32_t before = fs.checksum();
  const ServiceAckResult a = b.svc.acknowledge(2, boundProfile());
  CHECK(a.ack.status == AckStatus::RECORD_NOT_VALID);
  CHECK(!a.uncertain);
  CHECK_EQ(fs.checksum(), before);
  CHECK(!b.svc.calibrationAvailable());
}

void test_ack_uncertain_error_blocks_writes() {
  g_case = "ack uncertain";
  const MarkerFault faults[] = {MarkerFault::IO_NOTHING, MarkerFault::AFTER_DATA, MarkerFault::SILENT};
  for (MarkerFault f : faults) {
    FakeStorage fs;
    seed(&fs, 1);
    {
      CalibrationRecordStore store(&fs);
      CHECK(store.save(goldenRecord(0), boundProfile()).status == SaveStatus::OK);  // gen 2 awaits
    }
    Booted b(&fs);
    fs.marker_fault = f;
    fs.marker_fault_at = fs.marker_writes + 1;
    const ServiceAckResult a = b.svc.acknowledge(2, boundProfile());
    CHECK(a.guard == ServiceGuard::OK);
    CHECK(a.ack.status == AckStatus::MARKER_WRITE_FAILED || a.ack.status == AckStatus::MARKER_VERIFY_FAILED);
    CHECK(a.uncertain);
    if (f == MarkerFault::AFTER_DATA) {
      // The ACK landed, but uncertainty remains latched for mutating calls.
      CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
      CHECK(b.svc.ackUncertain());
      CHECK(b.svc.writesBlocked());
    } else if (f == MarkerFault::IO_NOTHING) {
      CHECK(b.snap().verdict == PersistenceVerdict::AWAITING_ACK);
      CHECK(b.svc.ackUncertain());
      CHECK(b.svc.writesBlocked());
      CHECK(!b.svc.calibrationAvailable());
      // blocked for SAVE and RECONCILE too
      CHECK(b.svc.save(goldenRecord(0), boundProfile()).guard == ServiceGuard::WRITES_BLOCKED);
      CHECK(b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 2, true, boundProfile()).status ==
            ReconcileStatus::WRITES_BLOCKED);
    } else {  // SILENT: the marker on flash is damaged
      CHECK(b.svc.ackUncertain());
      CHECK(b.svc.writesBlocked());
      CHECK(!b.svc.calibrationAvailable());
      CHECK(b.snap().verdict != PersistenceVerdict::VALID_ACKNOWLEDGED);
    }
    // A fresh boot of the same storage is the way out: it classifies what is really there.
    Booted rebooted(&fs);
    CHECK(!rebooted.svc.writesBlocked());
    if (f == MarkerFault::AFTER_DATA) {
      // Persisted ACK with its reply lost: reboot sees generation 2 as acked.
      CHECK(rebooted.svc.calibrationAvailable());
      CHECK_EQ(rebooted.snap().loaded_generation, 2);
      const int writes = fs.marker_writes + fs.slot_writes;
      CHECK(rebooted.svc.acknowledge(2, boundProfile()).ack.status == AckStatus::ALREADY_ACKNOWLEDGED);
      CHECK_EQ(fs.marker_writes + fs.slot_writes, writes);
    }
  }
}

void test_uncertain_operations_block_every_mutating_entry_point() {
  g_case = "uncertain operation blocks SAVE ACK RECONCILE";
  for (int operation = 0; operation < 3; ++operation) {
    for (MarkerFault fault : {MarkerFault::IO_NOTHING, MarkerFault::AFTER_DATA, MarkerFault::SILENT}) {
      FakeStorage fs;
      seed(&fs, 1);
      const auto acknowledged = fs.data[0];
      if (operation != 0) {
        CalibrationRecordStore store(&fs);
        CHECK(store.save(goldenRecord(0), boundProfile()).status == SaveStatus::OK);
      }
      Booted b(&fs);
      fs.marker_fault = fault;
      fs.marker_fault_at = fs.marker_writes + (operation == 0 ? 2 : 1);
      if (operation == 0) {
        CHECK(b.svc.save(goldenRecord(0), boundProfile()).uncertain);
      } else if (operation == 1) {
        CHECK(b.svc.acknowledge(2, boundProfile()).uncertain);
      } else {
        // Original finding: discard B/2, but the marker write is uncertain.
        CHECK(b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, true,
                              boundProfile()).uncertain);
      }
      CHECK(b.svc.writesBlocked());
      const auto marker = fs.marker_data;
      const auto a = fs.data[0], c = fs.data[1];
      const int writes = fs.marker_writes + fs.slot_writes;
      CHECK(b.svc.save(goldenRecord(0), boundProfile()).guard == ServiceGuard::WRITES_BLOCKED);
      CHECK(b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, true,
                            boundProfile()).status == ReconcileStatus::WRITES_BLOCKED);
      const ServiceAckResult ack = b.svc.acknowledge(2, boundProfile());
      if (operation == 1 && fault == MarkerFault::AFTER_DATA) {
        CHECK(ack.guard == ServiceGuard::OK);
        CHECK(ack.ack.status == AckStatus::ALREADY_ACKNOWLEDGED);
      } else {
        CHECK(ack.guard == ServiceGuard::WRITES_BLOCKED);
      }
      CHECK_EQ(fs.marker_writes + fs.slot_writes, writes);
      CHECK(fs.marker_data == marker && fs.data[0] == a && fs.data[1] == c);
      CHECK(fs.data[0] == acknowledged);
      CHECK(b.svc.writesBlocked());
      // A read failure cannot turn the read-only exception into a write.
      fs.fail_marker_read = true;
      CHECK(b.svc.acknowledge(2, boundProfile()).guard == ServiceGuard::WRITES_BLOCKED);
      CHECK_EQ(fs.marker_writes + fs.slot_writes, writes);
      fs.fail_marker_read = false;
      Booted rebooted(&fs);
      CHECK(!rebooted.svc.writesBlocked());
    }
  }
}

void test_save_uncertain_blocks_writes() {
  g_case = "save uncertain";
  FakeStorage fs;
  Booted b(&fs);
  fs.marker_fault = MarkerFault::AFTER_DATA;
  fs.marker_fault_at = 2;  // the AWAITING_ACK marker
  *b.svc.scratchRecord() = goldenRecord(0);
  const ServiceSaveResult s = b.svc.save(*b.svc.scratchRecord(), boundProfile());
  CHECK(s.guard == ServiceGuard::OK);
  CHECK(s.save.status != SaveStatus::OK);
  CHECK(s.uncertain);
  CHECK(b.svc.writesBlocked());
  CHECK(!b.svc.calibrationAvailable());
  CHECK(b.svc.save(goldenRecord(0), boundProfile()).guard == ServiceGuard::WRITES_BLOCKED);
}

// ---------------------------------------------------------------------------
// RECONCILIATION

void test_reconcile_adopt_awaiting_generation() {
  g_case = "reconcile: adopt awaiting";
  auto make = [](FakeStorage* fs) {
    seed(fs, 1);
    CalibrationRecordStore store(fs);
    CHECK(store.save(goldenRecord(0), boundProfile()).status == SaveStatus::OK);  // gen 2 awaits
    fs->clearCounters();
  };
  {  // the P2.4.1 classifier lists ADOPT for AWAITING_ACK: adopting the awaiting
     // generation is an ACK by another route; it drops nothing, so no confirmation
    FakeStorage fs;
    make(&fs);
    Booted b(&fs);
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 2, false, boundProfile());
    CHECK(r.status == ReconcileStatus::OK);
    CHECK(!r.discards);
    CHECK_EQ(fs.marker_writes, 1);
    CHECK_EQ(fs.slot_writes, 0);
    CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
    CHECK_EQ(b.snap().loaded_generation, 2);
  }
  {  // going back to the older acknowledged generation discards the verified newer one
    FakeStorage fs;
    make(&fs);
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, false, boundProfile());
    CHECK(r.status == ReconcileStatus::DISCARD_NOT_CONFIRMED);
    CHECK(r.discards);
    CHECK_EQ(fs.checksum(), before);
  }
}

void test_reconcile_pending_adopt_and_refusals() {
  g_case = "reconcile: pending";
  // Interrupted SAVE of generation 2, a verified record 2 on flash, 1 acknowledged.
  auto make = [](FakeStorage* fs) {
    seed(fs, 1);
    fs->putMarker(SaveMarkerState::PENDING, 1, 2);
    fs->putSlot(1, goldenRecord(2));
  };
  {  // wrong generation: no such valid record
    FakeStorage fs;
    make(&fs);
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 9, true, boundProfile());
    CHECK(r.status == ReconcileStatus::GENERATION_NOT_VALID);
    CHECK_EQ(fs.checksum(), before);
    const ReconcileResult z = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 0, true, boundProfile());
    CHECK(z.status == ReconcileStatus::BAD_ARGUMENT);
    CHECK_EQ(fs.checksum(), before);
  }
  {  // DECLARE while the acknowledged record is intact: refused even when "confirmed"
    FakeStorage fs;
    make(&fs);
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    const ReconcileResult r =
        b.svc.reconcile(ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, true, boundProfile());
    CHECK(r.status == ReconcileStatus::DECLARE_REFUSED_ACKNOWLEDGED_INTACT);
    CHECK_EQ(fs.checksum(), before);
    CHECK_EQ(b.snap().assessment.acknowledged_generation, 1);
  }
  {  // ADOPT the older acknowledged generation drops the newer verified one: needs confirmation
    FakeStorage fs;
    make(&fs);
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, false, boundProfile());
    CHECK(r.status == ReconcileStatus::DISCARD_NOT_CONFIRMED);
    CHECK(r.discards);
    CHECK_EQ(fs.marker_writes, 0);
    CHECK_EQ(fs.checksum(), before);
    r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, true, boundProfile());
    CHECK(r.status == ReconcileStatus::OK);
    CHECK(r.after == PersistenceClass::CONSISTENT);
    CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
    CHECK_EQ(b.snap().loaded_generation, 1);
    CHECK_EQ(fs.marker_writes, 1);  // exactly one marker, nothing else
    CHECK_EQ(fs.slot_writes, 0);
    SaveMarker m;
    CHECK(fs.decodedMarker(&m));
    CHECK(m.state == SaveMarkerState::IDLE);
    CHECK_EQ(m.acknowledged_generation, 1);
    CHECK(b.svc.calibrationAvailable());
  }
  {  // ADOPT the newer verified generation: no acknowledged generation is lost, no confirmation needed
    FakeStorage fs;
    make(&fs);
    Booted b(&fs);
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 2, false, boundProfile());
    CHECK(r.status == ReconcileStatus::OK);
    CHECK(!r.discards);
    CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
    CHECK_EQ(b.snap().loaded_generation, 2);
  }
}

void test_reconcile_declare_when_acknowledged_generation_lost() {
  g_case = "reconcile: declare";
  FakeStorage fs;
  seed(&fs, 1);
  fs.present[0] = false;
  fs.data[0].clear();  // the only record is gone; the marker still attests generation 1
  Booted b(&fs);
  CHECK(b.snap().verdict == PersistenceVerdict::ACKNOWLEDGED_GENERATION_LOST);
  const uint32_t before = fs.checksum();

  // Giving up an attested generation needs an explicit confirmation.
  ReconcileResult r = b.svc.reconcile(ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, false, boundProfile());
  CHECK(r.status == ReconcileStatus::DISCARD_NOT_CONFIRMED);
  CHECK(r.discards);
  CHECK_EQ(fs.checksum(), before);
  CHECK(b.snap().verdict == PersistenceVerdict::ACKNOWLEDGED_GENERATION_LOST);

  r = b.svc.reconcile(ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, true, boundProfile());
  CHECK(r.status == ReconcileStatus::OK);
  CHECK(r.after == PersistenceClass::NOTHING_ACKNOWLEDGED);
  CHECK(b.snap().verdict == PersistenceVerdict::NO_RECORD);
  CHECK(!b.svc.calibrationAvailable());  // declaring nothing never invents a calibration
  CHECK_EQ(fs.marker_writes, 1);
  CHECK_EQ(fs.slot_writes, 0);
  SaveMarker m;
  CHECK(fs.decodedMarker(&m));
  CHECK(m.state == SaveMarkerState::IDLE);
  CHECK_EQ(m.acknowledged_generation, 0);
}

void test_reconcile_not_required_and_missing_marker() {
  g_case = "reconcile: not required";
  {  // a healthy store needs nothing
    FakeStorage fs;
    seed(&fs, 1);
    Booted b(&fs);
    const uint32_t before = fs.checksum();
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, true, boundProfile());
    CHECK(r.status != ReconcileStatus::OK);
    CHECK_EQ(fs.checksum(), before);
    CHECK(b.svc.calibrationAvailable());
  }
  {  // a corrupt marker: adopting the record on disk is the offered action
    FakeStorage fs;
    seed(&fs, 1);
    corruptCrc(&fs.marker_data);
    Booted b(&fs);
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 1, true, boundProfile());
    CHECK(r.status == ReconcileStatus::OK || r.status == ReconcileStatus::ACTION_NOT_ALLOWED);
    if (r.status == ReconcileStatus::OK) CHECK(b.snap().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
  }
}

void test_reconcile_marker_faults_are_verified_and_block() {
  g_case = "reconcile: marker faults";
  const MarkerFault faults[] = {MarkerFault::NOT_MODIFIED, MarkerFault::IO_NOTHING, MarkerFault::AFTER_DATA,
                                MarkerFault::SILENT};
  for (MarkerFault f : faults) {
    FakeStorage fs;
    seed(&fs, 1);
    fs.putMarker(SaveMarkerState::PENDING, 1, 2);
    fs.putSlot(1, goldenRecord(2));
    Booted b(&fs);
    fs.marker_fault = f;
    fs.marker_fault_at = fs.marker_writes + 1;
    const ReconcileResult r = b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 2, true, boundProfile());
    CHECK(r.status != ReconcileStatus::OK);
    if (f == MarkerFault::NOT_MODIFIED) {
      CHECK(r.status == ReconcileStatus::MARKER_NOT_WRITTEN);
      CHECK(!r.uncertain);
      CHECK(!b.svc.writesBlocked());
    } else {
      CHECK(r.uncertain);
      CHECK(b.svc.writesBlocked());
      CHECK(!b.svc.calibrationAvailable());
    }
  }
}

void test_no_slot_is_ever_written_by_load_ack_reconcile() {
  g_case = "no slot writes";
  FakeStorage fs;
  seed(&fs, 1);
  fs.putMarker(SaveMarkerState::PENDING, 1, 2);
  fs.putSlot(1, goldenRecord(2));
  Booted b(&fs);
  b.svc.load(boundProfile());
  (void)b.svc.acknowledge(2, boundProfile());
  (void)b.svc.reconcile(ReconciliationAction::ADOPT_VALID_RECORD, 2, true, boundProfile());
  (void)b.svc.acknowledge(2, boundProfile());
  CHECK_EQ(fs.slot_writes, 0);  // only SAVE writes record slots
}

}  // namespace

int main() {
  test_boot_no_record();
  test_boot_valid_acknowledged();
  test_boot_nvs_not_ready_touches_nothing();
  test_boot_pending_marker();
  test_boot_awaiting_ack();
  test_boot_acknowledged_generation_lost();
  test_boot_marker_problems();
  test_boot_incompatible_record();
  test_status_has_no_side_effects();
  test_save_then_ack();
  test_second_save_keeps_previous_acknowledged_until_ack();
  test_ack_outcomes();
  test_ack_invalid_record();
  test_ack_uncertain_error_blocks_writes();
  test_uncertain_operations_block_every_mutating_entry_point();
  test_save_uncertain_blocks_writes();
  test_reconcile_adopt_awaiting_generation();
  test_reconcile_pending_adopt_and_refusals();
  test_reconcile_declare_when_acknowledged_generation_lost();
  test_reconcile_not_required_and_missing_marker();
  test_reconcile_marker_faults_are_verified_and_block();
  test_no_slot_is_ever_written_by_load_ack_reconcile();

  std::printf("test_calibration_persistence_service: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
