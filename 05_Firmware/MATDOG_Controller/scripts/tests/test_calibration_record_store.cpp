// Offline tests for the A/B calibration record store
// (src/calibration/CalibrationRecordStore.*) against a deterministic fake
// storage with fault injection. No flash, no NVS, no hardware.
//
// What is proven here: slot alternation and generations, selection of the
// newest VALID record, recovery of the previous record when the newest slot is
// damaged or never completed, and fail-closed behaviour when nothing usable
// remains - for an interruption before, during and after the write/commit.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "calibration_record_golden.h"
#include "../../src/calibration/CalibrationRecordStore.h"

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

enum class WriteFault {
  NONE,
  FAIL_BEFORE_WRITE,   // nothing reaches the slot, storage says NOT_MODIFIED (certain)
  FAIL_NOTHING_WRITTEN,  // nothing reaches the slot, but storage only says IO_ERROR (uncertain)
  TORN,                // half the bytes reach the slot, then IO error
  FAIL_AFTER_DATA,     // all bytes reach the slot, commit reported as failed
  NO_SPACE,            // refused, slot untouched
  SILENT_CORRUPTION,   // reports OK but one byte is wrong
};

class FakeStorage : public CalibrationRecordStorage {
 public:
  bool present[2] = {false, false};
  std::vector<uint8_t> data[2];
  bool fail_read[2] = {false, false};
  bool oversize[2] = {false, false};
  WriteFault fault = WriteFault::NONE;
  bool fail_read_after_write = false;
  int reads = 0;
  int writes = 0;
  int last_write_slot = -1;

  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity, size_t* length) override {
    ++reads;
    const int i = static_cast<int>(slot);
    *length = 0;
    if (fail_read[i] || (fail_read_after_write && writes > 0)) return StorageIoStatus::IO_ERROR;
    if (oversize[i]) return StorageIoStatus::BUFFER_TOO_SMALL;
    if (!present[i]) return StorageIoStatus::ABSENT;
    if (data[i].size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, data[i].data(), data[i].size());
    *length = data[i].size();
    return StorageIoStatus::OK;
  }

  StorageIoStatus write(CalibrationSlot slot, const uint8_t* bytes, size_t length) override {
    const int i = static_cast<int>(slot);
    ++writes;
    last_write_slot = i;
    switch (fault) {
      case WriteFault::NONE:
        break;
      case WriteFault::FAIL_BEFORE_WRITE:
        return StorageIoStatus::NOT_MODIFIED;
      case WriteFault::FAIL_NOTHING_WRITTEN:
        return StorageIoStatus::IO_ERROR;
      case WriteFault::TORN:
        present[i] = true;
        data[i].assign(bytes, bytes + length / 2);
        return StorageIoStatus::IO_ERROR;
      case WriteFault::FAIL_AFTER_DATA:
        present[i] = true;
        data[i].assign(bytes, bytes + length);
        return StorageIoStatus::IO_ERROR;
      case WriteFault::NO_SPACE:
        return StorageIoStatus::NO_SPACE;
      case WriteFault::SILENT_CORRUPTION:
        present[i] = true;
        data[i].assign(bytes, bytes + length);
        data[i][100] ^= 0x10;
        return StorageIoStatus::OK;
    }
    present[i] = true;
    data[i].assign(bytes, bytes + length);
    return StorageIoStatus::OK;
  }

  void put(int slot, const CalibrationRecord& r) {
    std::vector<uint8_t> b(kCalibrationRecordV1EncodedBytes);
    size_t n = 0;
    encodeCalibrationRecord(r, b.data(), b.size(), &n);
    b.resize(n);
    present[slot] = true;
    data[slot] = b;
  }
};

CalibrationRecord record() { return goldenRecord(0); }

void expectSelected(CalibrationRecordStore& store, uint32_t generation, CalibrationSlot slot, bool degraded) {
  const CalibrationRecord expect = goldenRecord(generation);
  CalibrationRecord loaded;
  const LoadResult r = store.load(boundProfile(), &loaded);
  CHECK(r.status == LoadStatus::OK);
  if (r.status != LoadStatus::OK) return;
  CHECK_EQ(r.generation, generation);
  CHECK(r.slot == slot);
  CHECK_EQ(r.degraded, degraded);
  CHECK_EQ(loaded.generation, generation);
  CHECK_EQ(loaded.joint[7].q0_tick, expect.joint[7].q0_tick);
}

// ---------------------------------------------------------------------------

void test_empty_storage() {
  g_case = "empty storage";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  CalibrationRecord loaded;
  const LoadResult r = store.load(boundProfile(), &loaded);
  CHECK(r.status == LoadStatus::NOT_FOUND);
  CHECK(r.report[0].state == SlotState::ABSENT);
  CHECK(r.report[1].state == SlotState::ABSENT);
  CHECK_EQ(fs.writes, 0);  // load never writes
}

void test_first_second_third_save_alternate() {
  g_case = "A/B alternation";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  const auto profile = boundProfile();

  SaveResult r = store.save(record(), profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK(r.slot == CalibrationSlot::A);
  CHECK_EQ(r.generation, 1);
  CHECK(!r.previous_record_intact);
  CHECK_EQ(fs.data[0].size(), kCalibrationRecordV1EncodedBytes);
  CHECK(!fs.present[1]);
  expectSelected(store, 1, CalibrationSlot::A, false);

  const std::vector<uint8_t> a_after_first = fs.data[0];
  r = store.save(record(), profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK(r.slot == CalibrationSlot::B);
  CHECK_EQ(r.generation, 2);
  CHECK(r.previous_record_intact);
  CHECK_EQ(r.previous_generation, 1);
  CHECK(fs.data[0] == a_after_first);  // the previous slot was not touched
  expectSelected(store, 2, CalibrationSlot::B, false);

  const std::vector<uint8_t> b_after_second = fs.data[1];
  r = store.save(record(), profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK(r.slot == CalibrationSlot::A);
  CHECK_EQ(r.generation, 3);
  CHECK(fs.data[1] == b_after_second);
  expectSelected(store, 3, CalibrationSlot::A, false);

  for (int i = 4; i <= 30; ++i) {
    r = store.save(record(), profile);
    CHECK(r.status == SaveStatus::OK);
    CHECK_EQ(r.generation, i);
    CHECK_EQ(static_cast<int>(r.slot), (i - 1) % 2);
  }
  expectSelected(store, 30, CalibrationSlot::B, false);

  // The caller's generation is ignored: the store owns it.
  CalibrationRecord forced = goldenRecord(999);
  r = store.save(forced, profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK_EQ(r.generation, 31);

  // What is stored is exactly the record: a load returns the saved content.
  CalibrationRecord loaded;
  CHECK(store.load(profile, &loaded).status == LoadStatus::OK);
  CHECK(std::memcmp(loaded.digest, goldenRecord(1).digest, sizeof(loaded.digest)) == 0);
  CHECK_EQ(loaded.parameters_approved, 0);
}

void test_save_refuses_invalid_without_touching_storage() {
  g_case = "invalid record";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  CalibrationRecord bad = record();
  bad.parameters_approved = 1;
  SaveResult r = store.save(bad, boundProfile());
  CHECK(r.status == SaveStatus::INVALID_RECORD);
  CHECK(r.validation == CalibrationRecordStatus::FORBIDDEN_AUTHORIZATION);
  CHECK_EQ(fs.reads, 0);
  CHECK_EQ(fs.writes, 0);

  bad = record();
  bad.joint[0].bus_id = 14;
  r = store.save(bad, boundProfile());
  CHECK(r.status == SaveStatus::INVALID_RECORD);
  CHECK(r.validation == CalibrationRecordStatus::IDENTITY_MISMATCH);
  CHECK_EQ(fs.writes, 0);

  actuator::CalibrationGeometryProfile unbound;
  r = store.save(record(), unbound);
  CHECK(r.status == SaveStatus::INVALID_RECORD);
  CHECK_EQ(fs.writes, 0);

  CalibrationRecordStore null_store(nullptr);
  CHECK(null_store.save(record(), boundProfile()).status == SaveStatus::BAD_ARGUMENT);
  CalibrationRecord out;
  CHECK(null_store.load(boundProfile(), &out).status == LoadStatus::BAD_ARGUMENT);
  CHECK(store.load(boundProfile(), nullptr).status == LoadStatus::BAD_ARGUMENT);
}

// Seeds storage with generations 1 (slot A) and 2 (slot B) through the store itself.
void seedTwo(FakeStorage* fs, CalibrationRecordStore* store) {
  CHECK(store->save(record(), boundProfile()).status == SaveStatus::OK);
  CHECK(store->save(record(), boundProfile()).status == SaveStatus::OK);
  fs->writes = 0;
  fs->reads = 0;
}

void test_failure_before_write() {
  g_case = "failure before write";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedTwo(&fs, &store);
  const std::vector<uint8_t> a = fs.data[0], b = fs.data[1];

  fs.fault = WriteFault::FAIL_BEFORE_WRITE;
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(r.io == StorageIoStatus::NOT_MODIFIED);
  CHECK(r.slot == CalibrationSlot::A);  // target was the older slot
  CHECK(r.previous_record_intact);
  CHECK_EQ(r.previous_generation, 2);
  CHECK(fs.data[0] == a && fs.data[1] == b);
  CHECK(store.writeState() == WriteState::OPEN);  // certain: storage was not modified
  fs.fault = WriteFault::NONE;
  expectSelected(store, 2, CalibrationSlot::B, false);

  // Retry on the SAME instance behaves as before: it succeeds into the older slot.
  const SaveResult retry = store.save(record(), boundProfile());
  CHECK(retry.status == SaveStatus::OK);
  CHECK(retry.slot == CalibrationSlot::A);
  CHECK_EQ(retry.generation, 3);
  CHECK(fs.data[1] == b);
  CHECK(store.writeState() == WriteState::OPEN);
}

void test_failure_during_write_torn() {
  g_case = "failure during write (torn)";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedTwo(&fs, &store);
  const std::vector<uint8_t> b = fs.data[1];

  fs.fault = WriteFault::TORN;
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(r.previous_record_intact);
  CHECK(fs.data[1] == b);  // the record being protected is byte-identical
  fs.fault = WriteFault::NONE;

  CalibrationRecord loaded;
  const LoadResult l = store.load(boundProfile(), &loaded);
  CHECK(l.status == LoadStatus::OK);
  CHECK_EQ(l.generation, 2);
  CHECK(l.slot == CalibrationSlot::B);
  CHECK(l.degraded);  // the newest save was lost and reported as such
  CHECK(l.report[0].state == SlotState::CORRUPT);

  // The failed instance refuses to write again; the good slot stays untouched.
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(fs.data[1] == b);

  // After a reboot (new instance) the next save heals it: it overwrites the
  // damaged slot, never the good one.
  CalibrationRecordStore rebooted(&fs);
  const SaveResult next = rebooted.save(record(), boundProfile());
  CHECK(next.status == SaveStatus::OK);
  CHECK(next.slot == CalibrationSlot::A);
  CHECK_EQ(next.generation, 3);
  CHECK(fs.data[1] == b);
  expectSelected(rebooted, 3, CalibrationSlot::A, false);
}

void test_failure_during_commit() {
  g_case = "failure during commit";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedTwo(&fs, &store);
  const std::vector<uint8_t> b = fs.data[1];

  // The commit is reported as failed although the bytes did land. The store
  // reports failure; whatever state storage is in must still load safely.
  fs.fault = WriteFault::FAIL_AFTER_DATA;
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(fs.data[1] == b);
  fs.fault = WriteFault::NONE;
  CalibrationRecord loaded;
  const LoadResult l = store.load(boundProfile(), &loaded);
  CHECK(l.status == LoadStatus::OK);
  // Complete and CRC-valid: it is a legitimate generation 3 (the caller was
  // told the save failed; after a reboot the next save yields generation 4).
  CHECK_EQ(l.generation, 3);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CalibrationRecordStore rebooted(&fs);
  const SaveResult retry = rebooted.save(record(), boundProfile());
  CHECK(retry.status == SaveStatus::OK);
  CHECK_EQ(retry.generation, 4);
  CHECK(retry.slot == CalibrationSlot::B);
}

void test_no_space() {
  g_case = "insufficient space";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedTwo(&fs, &store);
  fs.fault = WriteFault::NO_SPACE;
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::NO_SPACE);
  CHECK(r.io == StorageIoStatus::NO_SPACE);
  CHECK(r.previous_record_intact);
  // Backends cannot promise that NO_SPACE left the slot alone: treated as uncertain.
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  fs.fault = WriteFault::NONE;
  expectSelected(store, 2, CalibrationSlot::B, false);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
}

void test_readback_failures() {
  g_case = "read-back";
  {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    seedTwo(&fs, &store);
    fs.fault = WriteFault::SILENT_CORRUPTION;
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::READBACK_MISMATCH);
    CHECK_EQ(r.generation, 0);  // no success is claimed
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    fs.fault = WriteFault::NONE;
    expectSelected(store, 2, CalibrationSlot::B, true);  // recovered, flagged
  }
  {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    seedTwo(&fs, &store);
    fs.fail_read_after_write = true;
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::READBACK_FAILED);
    CHECK_EQ(r.generation, 0);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  }
}

void test_recovery_when_newest_slot_is_invalid() {
  g_case = "recovery";
  const auto profile = boundProfile();
  {  // newest slot bit-flipped
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    seedTwo(&fs, &store);
    fs.data[1][500] ^= 0x01;
    expectSelected(store, 1, CalibrationSlot::A, true);
    CalibrationRecord out;
    const LoadResult l = store.load(profile, &out);
    CHECK(l.report[1].state == SlotState::CORRUPT);
    CHECK(l.report[1].detail == CalibrationRecordStatus::BAD_CRC);
    CHECK(l.report[0].state == SlotState::VALID);
  }
  {  // newest slot truncated
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    seedTwo(&fs, &store);
    fs.data[1].resize(700);
    expectSelected(store, 1, CalibrationSlot::A, true);
  }
  {  // newest slot semantically contradictory but CRC-valid
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    CalibrationRecord bad = goldenRecord(2);
    bad.joint[3].diagnostics.scale_permille += 1;
    fs.put(1, bad);
    CalibrationRecord out;
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::OK);
    CHECK_EQ(l.generation, 1);
    CHECK(l.degraded);
    CHECK(l.report[1].state == SlotState::CORRUPT);
    CHECK(l.report[1].detail == CalibrationRecordStatus::INCOHERENT);
    // Its generation still counts: the next save is 3, never a reuse of 2.
    const SaveResult s = store.save(record(), profile);
    CHECK(s.status == SaveStatus::OK);
    CHECK_EQ(s.generation, 3);
    CHECK(s.slot == CalibrationSlot::B);
  }
  {  // newest slot foreign (another schema, intact)
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    fs.put(1, goldenRecord(2));
    fs.data[1][4] = 2;  // schema 2
    const uint32_t crc = calibrationCrc32(fs.data[1].data(), fs.data[1].size() - 4);
    for (int i = 0; i < 4; ++i) fs.data[1][fs.data[1].size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
    CalibrationRecord out;
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::OK);
    CHECK_EQ(l.generation, 1);
    CHECK(l.degraded);
    CHECK(l.report[1].state == SlotState::INCOMPATIBLE);
  }
}

void test_generation_selection() {
  g_case = "generation selection";
  const auto profile = boundProfile();
  {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(0, goldenRecord(3));
    fs.put(1, goldenRecord(10));
    expectSelected(store, 10, CalibrationSlot::B, false);
  }
  {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(0, goldenRecord(10));
    fs.put(1, goldenRecord(3));
    expectSelected(store, 10, CalibrationSlot::A, false);
  }
  {  // only one slot holds a record
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(1, goldenRecord(4));
    expectSelected(store, 4, CalibrationSlot::B, false);
  }
  {  // equal generations contradict the protocol: nothing is selected
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(0, goldenRecord(5));
    fs.put(1, goldenRecord(5));
    CalibrationRecord out;
    CHECK(store.load(profile, &out).status == LoadStatus::NO_VALID_RECORD);
  }
  {  // generation exhaustion refuses to wrap
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(0, goldenRecord(0xFFFFFFFFu));
    const SaveResult r = store.save(record(), profile);
    CHECK(r.status == SaveStatus::GENERATION_EXHAUSTED);
    CHECK_EQ(fs.writes, 0);
  }
  {  // a foreign intact record keeps generations monotonic and is not overwritten
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(1, goldenRecord(9));
    fs.data[1][4] = 2;
    const uint32_t crc = calibrationCrc32(fs.data[1].data(), fs.data[1].size() - 4);
    for (int i = 0; i < 4; ++i) fs.data[1][fs.data[1].size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
    const std::vector<uint8_t> foreign = fs.data[1];
    const SaveResult r = store.save(record(), profile);
    CHECK(r.status == SaveStatus::OK);
    CHECK_EQ(r.generation, 10);
    CHECK(r.slot == CalibrationSlot::A);  // the absent slot, not the foreign one
    CHECK(fs.data[1] == foreign);
  }
}

void test_both_slots_unusable() {
  g_case = "both slots unusable";
  const auto profile = boundProfile();
  CalibrationRecord out;
  {  // both corrupt
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(0, goldenRecord(1));
    fs.put(1, goldenRecord(2));
    fs.data[0][300] ^= 1;
    fs.data[1][300] ^= 1;
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::NO_VALID_RECORD);
    CHECK(l.report[0].state == SlotState::CORRUPT && l.report[1].state == SlotState::CORRUPT);
  }
  {  // one corrupt, one absent
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.put(0, goldenRecord(1));
    fs.data[0][300] ^= 1;
    CHECK(store.load(profile, &out).status == LoadStatus::NO_VALID_RECORD);
    // Saving into this state is allowed (nothing valid to protect) and heals it.
    const SaveResult s = store.save(record(), profile);
    CHECK(s.status == SaveStatus::OK);
    CHECK(s.slot == CalibrationSlot::B);  // the absent slot first
    CHECK_EQ(s.generation, 1);            // hint was unknowable (CRC bad)
  }
  {  // both foreign: a different geometry (consistent tag) is INCOMPATIBLE, not corrupt
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    CalibrationRecord other = goldenRecord(1);
    other.digest[0][0] ^= 1;
    actuator::GeometryProvenance p = actuator::geometry_data::kProvenance;
    static const char kDigits[] = "0123456789abcdef";
    char* dst[6] = {p.urdf_sha256, p.mesh_manifest_sha256, p.endpoint_semantic_sha256,
                    p.parking_semantic_sha256, p.safety_policy_semantic_sha256, p.allocation_sha256};
    for (int k = 0; k < 6; ++k) {
      for (int b = 0; b < 32; ++b) {
        dst[k][2 * b] = kDigits[other.digest[k][b] >> 4];
        dst[k][2 * b + 1] = kDigits[other.digest[k][b] & 15];
      }
      dst[k][64] = '\0';
    }
    other.geometry_tag = actuator::geometryProvenanceTag(p);
    fs.put(0, other);
    other.generation = 2;
    fs.put(1, other);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::INCOMPATIBLE);
    CHECK(l.report[0].state == SlotState::INCOMPATIBLE);
    CHECK(l.report[0].detail == CalibrationRecordStatus::PROVENANCE_MISMATCH);
  }
  {  // an installation difference (bus id) is incompatible too
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    CalibrationRecord other = goldenRecord(1);
    other.joint[0].bus_id = 14;
    fs.put(0, other);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::INCOMPATIBLE);
    CHECK(l.report[0].detail == CalibrationRecordStatus::IDENTITY_MISMATCH);
  }
  {  // a slot larger than any V1 record
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.oversize[0] = true;
    fs.oversize[1] = true;
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::NO_VALID_RECORD);
    CHECK(l.report[0].state == SlotState::CORRUPT);
    CHECK(l.report[0].detail == CalibrationRecordStatus::BAD_LENGTH);
  }
}

void test_storage_errors_fail_closed() {
  g_case = "storage I/O errors";
  const auto profile = boundProfile();
  CalibrationRecord out;
  for (int bad = 0; bad < 2; ++bad) {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    seedTwo(&fs, &store);
    fs.fail_read[bad] = true;
    const LoadResult l = store.load(profile, &out);
    // Never selects on partial knowledge, even if the other slot is fine.
    CHECK(l.status == LoadStatus::IO_ERROR);
    CHECK(l.report[bad].state == SlotState::IO_ERROR);
    const SaveResult s = store.save(record(), profile);
    CHECK(s.status == SaveStatus::STORAGE_UNUSABLE);
    CHECK_EQ(fs.writes, 0);  // refuses to write blind
  }
}

void test_roundtrip_content_equality() {
  g_case = "content equality";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  const auto profile = boundProfile();
  CHECK(store.save(record(), profile).status == SaveStatus::OK);
  CalibrationRecord loaded;
  CHECK(store.load(profile, &loaded).status == LoadStatus::OK);
  const CalibrationRecord expect = goldenRecord(1);
  uint8_t a[kCalibrationRecordV1EncodedBytes], b[kCalibrationRecordV1EncodedBytes];
  size_t na = 0, nb = 0;
  CHECK(encodeCalibrationRecord(loaded, a, sizeof(a), &na) == CalibrationRecordStatus::OK);
  CHECK(encodeCalibrationRecord(expect, b, sizeof(b), &nb) == CalibrationRecordStatus::OK);
  CHECK_EQ(na, nb);
  CHECK(std::memcmp(a, b, na) == 0);
}

// --- P2.1: write-uncertainty block -----------------------------------------

// SAVE confirmed B/2; the next SAVE fails with an uncertain error but leaves a
// complete valid A/3. Without the block a retry would select A/3, target B and
// overwrite the last confirmed record.
void seedConfirmedB2ThenUncertainA3(FakeStorage* fs, CalibrationRecordStore* store) {
  seedTwo(fs, store);
  fs->fault = WriteFault::FAIL_AFTER_DATA;
  const SaveResult r = store->save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);  // never turned into success
  CHECK_EQ(r.generation, 0);
  CHECK(r.slot == CalibrationSlot::A);
  fs->fault = WriteFault::NONE;
}

void test_uncertain_error_blocks_retry() {
  g_case = "uncertain error blocks retry";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedConfirmedB2ThenUncertainA3(&fs, &store);
  const std::vector<uint8_t> b2 = fs.data[1];
  const std::vector<uint8_t> a3 = fs.data[0];
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK(std::strcmp(toString(store.writeState()), "BLOCKED_UNCERTAIN_WRITE") == 0);

  fs.reads = 0;
  fs.writes = 0;
  const SaveResult retry = store.save(record(), boundProfile());
  CHECK(retry.status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(std::strcmp(toString(retry.status), "BLOCKED_UNCERTAIN_WRITE") == 0);
  CHECK_EQ(retry.generation, 0);
  CHECK(!retry.previous_record_intact);  // nothing was scanned
  CHECK_EQ(fs.writes, 0);
  CHECK_EQ(fs.reads, 0);
  CHECK(fs.data[1] == b2);  // the last confirmed record is byte-identical
  CHECK(fs.data[0] == a3);

  // Even a retry whose write would itself tear the slot never gets to write.
  fs.fault = WriteFault::TORN;
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(fs.writes, 0);
  CHECK(fs.data[1] == b2);
  fs.fault = WriteFault::NONE;
}

void test_repeated_attempts_cannot_overwrite() {
  g_case = "repeated attempts after uncertain error";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedConfirmedB2ThenUncertainA3(&fs, &store);
  const std::vector<uint8_t> b2 = fs.data[1];
  for (int i = 0; i < 5; ++i) {
    CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  }
  CHECK_EQ(fs.writes, 1);  // only the original, failed attempt
  CHECK(fs.data[1] == b2);
}

void test_load_does_not_unlock() {
  g_case = "load does not unlock";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedConfirmedB2ThenUncertainA3(&fs, &store);
  const std::vector<uint8_t> b2 = fs.data[1];

  // LOAD stays available and reports what is in storage: the valid highest generation.
  expectSelected(store, 3, CalibrationSlot::A, false);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  expectSelected(store, 3, CalibrationSlot::A, false);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK(fs.data[1] == b2);
}

void test_reboot_selects_highest_valid_record() {
  g_case = "reboot after uncertain error";
  FakeStorage fs;
  {
    CalibrationRecordStore store(&fs);
    seedConfirmedB2ThenUncertainA3(&fs, &store);
  }
  // New instance = reboot. Nothing remembers which record the operator saw
  // confirmed: the valid record with the highest generation is simply selected.
  CalibrationRecordStore rebooted(&fs);
  CHECK(rebooted.writeState() == WriteState::OPEN);
  expectSelected(rebooted, 3, CalibrationSlot::A, false);
  const SaveResult r = rebooted.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::OK);
  CHECK(r.slot == CalibrationSlot::B);
  CHECK_EQ(r.generation, 4);
  CHECK(r.previous_record_intact);
  CHECK_EQ(r.previous_generation, 3);  // the selected record, not "the last confirmed"
}

void test_every_uncertain_outcome_blocks() {
  g_case = "uncertain outcomes block";
  const WriteFault faults[] = {WriteFault::FAIL_NOTHING_WRITTEN, WriteFault::TORN,
                               WriteFault::FAIL_AFTER_DATA, WriteFault::NO_SPACE,
                               WriteFault::SILENT_CORRUPTION};
  for (WriteFault f : faults) {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    seedTwo(&fs, &store);
    const std::vector<uint8_t> b = fs.data[1];
    fs.fault = f;
    CHECK(store.save(record(), boundProfile()).status != SaveStatus::OK);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    fs.fault = WriteFault::NONE;
    CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
    CHECK(fs.data[1] == b);
  }
}

void test_certain_errors_do_not_block() {
  g_case = "certain errors do not block";
  const auto profile = boundProfile();
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  seedTwo(&fs, &store);
  const std::vector<uint8_t> a = fs.data[0], b = fs.data[1];

  // Rejected before any storage access.
  CalibrationRecord bad = record();
  bad.parameters_approved = 1;
  CHECK(store.save(bad, profile).status == SaveStatus::INVALID_RECORD);
  CHECK(store.writeState() == WriteState::OPEN);

  // Could not read a slot: refused before any write.
  fs.fail_read[0] = true;
  CHECK(store.save(record(), profile).status == SaveStatus::STORAGE_UNUSABLE);
  CHECK(store.writeState() == WriteState::OPEN);
  fs.fail_read[0] = false;

  // Storage said it did not modify anything.
  fs.fault = WriteFault::FAIL_BEFORE_WRITE;
  CHECK(store.save(record(), profile).status == SaveStatus::WRITE_FAILED);
  CHECK(store.writeState() == WriteState::OPEN);
  CHECK(fs.data[0] == a && fs.data[1] == b);
  fs.fault = WriteFault::NONE;

  const SaveResult ok = store.save(record(), profile);
  CHECK(ok.status == SaveStatus::OK);
  CHECK_EQ(ok.generation, 3);
  CHECK(fs.data[1] == b);
}

void test_uncertain_failure_on_first_save_blocks() {
  g_case = "uncertain failure on first save";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  fs.fault = WriteFault::FAIL_AFTER_DATA;  // A/1 lands, error reported
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::WRITE_FAILED);
  fs.fault = WriteFault::NONE;
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  expectSelected(store, 1, CalibrationSlot::A, false);
}

}  // namespace

int main() {
  test_empty_storage();
  test_first_second_third_save_alternate();
  test_save_refuses_invalid_without_touching_storage();
  test_failure_before_write();
  test_failure_during_write_torn();
  test_failure_during_commit();
  test_no_space();
  test_readback_failures();
  test_recovery_when_newest_slot_is_invalid();
  test_generation_selection();
  test_both_slots_unusable();
  test_storage_errors_fail_closed();
  test_roundtrip_content_equality();
  test_uncertain_error_blocks_retry();
  test_repeated_attempts_cannot_overwrite();
  test_load_does_not_unlock();
  test_reboot_selects_highest_valid_record();
  test_every_uncertain_outcome_blocks();
  test_certain_errors_do_not_block();
  test_uncertain_failure_on_first_save_blocks();
  std::printf("test_calibration_record_store: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
