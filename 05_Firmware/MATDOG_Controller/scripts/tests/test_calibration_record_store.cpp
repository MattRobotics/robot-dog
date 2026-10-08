// Offline tests for the A/B calibration record store and its SAVE-marker
// protocol (src/calibration/CalibrationRecordStore.*) against a deterministic
// fake storage with fault injection per mutation. No flash, no NVS, no hardware.
//
// What is proven here: the SAVE protocol order (PENDING marker, record to the
// inactive slot, full read-back, AWAITING_ACK marker, marker read-back), the
// separate durable acknowledgment (P2.4.1), that no failure at any step produces
// a false acknowledgment, that every post-PENDING failure blocks the instance
// (P2.1), that a "rebooted" instance never serves an unacknowledged record and
// never overwrites the acknowledged one, and that the explicit reconciliation
// contract leads back to a usable state.
//
// A power cut is modelled as: the faulting mutation is lost / partly written /
// fully written, the storage then goes "dead" (every call fails) until
// reboot() brings it back with exactly the bytes it held.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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

enum class Fault {
  NONE,
  NOT_MODIFIED,       // storage certifies it did nothing
  IO_NOTHING,         // nothing reaches storage, storage only says IO_ERROR (uncertain)
  TORN,               // half the bytes reach storage, then IO error
  AFTER_DATA,         // all bytes reach storage, error reported (commit failed)
  NO_SPACE,           // refused, nothing written
  SILENT_CORRUPTION,  // reports OK but one byte is wrong
  POWER_BEFORE,       // power lost before the mutation: nothing written, storage dead
  POWER_TORN,         // power lost mid-write: half written, storage dead
  POWER_AFTER,        // power lost after the data landed, before the return: dead
};

const Fault kAllFaults[] = {Fault::NOT_MODIFIED, Fault::IO_NOTHING, Fault::TORN,
                            Fault::AFTER_DATA,   Fault::NO_SPACE,   Fault::SILENT_CORRUPTION,
                            Fault::POWER_BEFORE, Fault::POWER_TORN, Fault::POWER_AFTER};

const char* faultName(Fault f) {
  switch (f) {
    case Fault::NONE: return "NONE";
    case Fault::NOT_MODIFIED: return "NOT_MODIFIED";
    case Fault::IO_NOTHING: return "IO_NOTHING";
    case Fault::TORN: return "TORN";
    case Fault::AFTER_DATA: return "AFTER_DATA";
    case Fault::NO_SPACE: return "NO_SPACE";
    case Fault::SILENT_CORRUPTION: return "SILENT_CORRUPTION";
    case Fault::POWER_BEFORE: return "POWER_BEFORE";
    case Fault::POWER_TORN: return "POWER_TORN";
    case Fault::POWER_AFTER: return "POWER_AFTER";
  }
  return "?";
}

class FakeStorage : public CalibrationRecordStorage {
 public:
  bool present[2] = {false, false};
  std::vector<uint8_t> data[2];
  bool marker_present = false;
  std::vector<uint8_t> marker_data;

  bool fail_read[2] = {false, false};
  bool oversize[2] = {false, false};
  bool fail_marker_read = false;
  bool oversize_marker = false;
  // Slot reads fail once a slot write was attempted; marker reads fail once the
  // n-th marker write was attempted (1 = PENDING, 2 = AWAITING_ACK, 3 = ACK). 0 = never.
  bool fail_slot_read_after_slot_write = false;
  int fail_marker_read_from_marker_write = 0;

  int fault_at = 0;  // 1-based index of the mutation (slot or marker write) that faults
  Fault fault = Fault::NONE;
  bool dead = false;

  int mutations = 0;
  int slot_writes = 0;
  int marker_writes = 0;
  int reads = 0;
  std::vector<std::string> log;  // "wM", "wA", "wB", "rM", "rA", "rB"

  void arm(int at, Fault f) {
    fault_at = at;
    fault = f;
  }
  void reboot() {
    dead = false;
    fault_at = 0;
    fault = Fault::NONE;
    fail_slot_read_after_slot_write = false;
    fail_marker_read_from_marker_write = 0;
    slot_writes = marker_writes = mutations = 0;
  }

  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity, size_t* length) override {
    ++reads;
    const int i = static_cast<int>(slot);
    log.push_back(i == 0 ? "rA" : "rB");
    *length = 0;
    if (dead || fail_read[i] || (fail_slot_read_after_slot_write && slot_writes > 0)) {
      return StorageIoStatus::IO_ERROR;
    }
    if (oversize[i]) return StorageIoStatus::BUFFER_TOO_SMALL;
    if (!present[i]) return StorageIoStatus::ABSENT;
    if (data[i].size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, data[i].data(), data[i].size());
    *length = data[i].size();
    return StorageIoStatus::OK;
  }

  StorageIoStatus readMarker(uint8_t* buffer, size_t capacity, size_t* length) override {
    ++reads;
    log.push_back("rM");
    *length = 0;
    if (dead || fail_marker_read ||
        (fail_marker_read_from_marker_write > 0 && marker_writes >= fail_marker_read_from_marker_write)) {
      return StorageIoStatus::IO_ERROR;
    }
    if (oversize_marker) return StorageIoStatus::BUFFER_TOO_SMALL;
    if (!marker_present) return StorageIoStatus::ABSENT;
    if (marker_data.size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, marker_data.data(), marker_data.size());
    *length = marker_data.size();
    return StorageIoStatus::OK;
  }

  StorageIoStatus write(CalibrationSlot slot, const uint8_t* bytes, size_t length) override {
    const int i = static_cast<int>(slot);
    ++slot_writes;
    log.push_back(i == 0 ? "wA" : "wB");
    return mutate(&present[i], &data[i], bytes, length, 100);
  }

  StorageIoStatus writeMarker(const uint8_t* bytes, size_t length) override {
    ++marker_writes;
    log.push_back("wM");
    return mutate(&marker_present, &marker_data, bytes, length, 14);
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

 private:
  StorageIoStatus mutate(bool* pres, std::vector<uint8_t>* d, const uint8_t* bytes, size_t length,
                         size_t flip_at) {
    ++mutations;
    if (dead) return StorageIoStatus::IO_ERROR;
    const Fault f = mutations == fault_at ? fault : Fault::NONE;
    switch (f) {
      case Fault::NONE:
        break;
      case Fault::NOT_MODIFIED:
        return StorageIoStatus::NOT_MODIFIED;
      case Fault::IO_NOTHING:
        return StorageIoStatus::IO_ERROR;
      case Fault::NO_SPACE:
        return StorageIoStatus::NO_SPACE;
      case Fault::TORN:
        *pres = true;
        d->assign(bytes, bytes + length / 2);
        return StorageIoStatus::IO_ERROR;
      case Fault::AFTER_DATA:
        *pres = true;
        d->assign(bytes, bytes + length);
        return StorageIoStatus::IO_ERROR;
      case Fault::SILENT_CORRUPTION:
        *pres = true;
        d->assign(bytes, bytes + length);
        (*d)[flip_at] ^= 0x10;
        return StorageIoStatus::OK;
      case Fault::POWER_BEFORE:
        dead = true;
        return StorageIoStatus::IO_ERROR;
      case Fault::POWER_TORN:
        *pres = true;
        d->assign(bytes, bytes + length / 2);
        dead = true;
        return StorageIoStatus::IO_ERROR;
      case Fault::POWER_AFTER:
        *pres = true;
        d->assign(bytes, bytes + length);
        dead = true;
        return StorageIoStatus::IO_ERROR;
    }
    *pres = true;
    d->assign(bytes, bytes + length);
    return StorageIoStatus::OK;
  }
};

CalibrationRecord record() { return goldenRecord(0); }

// Storage with n generations written by the store itself AND acknowledged.
void seed(FakeStorage* fs, int n) {
  CalibrationRecordStore store(fs);
  for (int i = 0; i < n; ++i) {
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::OK);
    CHECK(store.acknowledge(r.generation, boundProfile()).status == AckStatus::OK);
  }
  fs->log.clear();
  fs->reads = 0;
  fs->slot_writes = fs->marker_writes = fs->mutations = 0;
}

void expectSelected(CalibrationRecordStore& store, uint32_t generation, CalibrationSlot slot, bool degraded) {
  const CalibrationRecord expect = goldenRecord(generation);
  CalibrationRecord loaded;
  const LoadResult r = store.load(boundProfile(), &loaded);
  CHECK(r.status == LoadStatus::OK);
  CHECK(r.assessment.cls == PersistenceClass::CONSISTENT);
  if (r.status != LoadStatus::OK) return;
  CHECK_EQ(r.generation, generation);
  CHECK(r.slot == slot);
  CHECK_EQ(r.degraded, degraded);
  CHECK_EQ(loaded.generation, generation);
  CHECK_EQ(loaded.joint[7].q0_tick, expect.joint[7].q0_tick);
}

PersistenceClass classOf(FakeStorage& fs) {
  CalibrationRecordStore s(&fs);
  CalibrationRecord out;
  return s.load(boundProfile(), &out).assessment.cls;
}

// ---------------------------------------------------------------------------

void test_empty_storage_is_first_install() {
  g_case = "first install";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  CalibrationRecord loaded;
  const LoadResult r = store.load(boundProfile(), &loaded);
  CHECK(r.status == LoadStatus::NOT_FOUND);
  CHECK(r.assessment.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED);
  CHECK(!r.assessment.record_available);
  CHECK(r.assessment.save_allowed);  // the first SAVE after a fresh Full Calibration
  CHECK(r.report[0].state == SlotState::ABSENT);
  CHECK(r.report[1].state == SlotState::ABSENT);
  CHECK(r.marker.state == MarkerObservation::ABSENT);
  CHECK_EQ(fs.slot_writes + fs.marker_writes, 0);  // load never writes

  // The first marker is a PENDING{acknowledged 0, begun 1}; the SAVE ends at
  // AWAITING_ACK{0, 1} - no previous generation is invented.
  const SaveResult s = store.save(record(), boundProfile());
  CHECK(s.status == SaveStatus::OK);
  CHECK(s.phase == SavePhase::DONE);
  CHECK(s.persistence == PersistenceClass::NEVER_INITIALIZED_OR_ERASED);
  CHECK_EQ(s.generation, 1);
  SaveMarker m;
  CHECK(fs.decodedMarker(&m));
  CHECK(m.state == SaveMarkerState::AWAITING_ACK);
  CHECK_EQ(m.acknowledged_generation, 0);
  CHECK_EQ(m.begun_generation, 1);
  // Verified on flash is not calibration data: nothing is served until the ACK.
  CHECK(store.load(boundProfile(), &loaded).status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK(classOf(fs) == PersistenceClass::AWAITING_ACK);
  const AckResult ack = store.acknowledge(1, boundProfile());
  CHECK(ack.status == AckStatus::OK);
  CHECK(fs.decodedMarker(&m));
  CHECK(m.state == SaveMarkerState::IDLE);
  CHECK_EQ(m.acknowledged_generation, 1);
  CHECK_EQ(m.begun_generation, 1);
  expectSelected(store, 1, CalibrationSlot::A, false);
}

void test_protocol_order() {
  g_case = "protocol order";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  const SaveResult s = store.save(record(), boundProfile());
  CHECK(s.status == SaveStatus::OK);
  // scan (rA rB rM), PENDING + verify, slot + full read-back, AWAITING_ACK + verify
  const std::vector<std::string> expected = {"rA", "rB", "rM", "wM", "rM", "wA", "rA", "wM", "rM"};
  CHECK(fs.log == expected);
  CHECK_EQ(fs.mutations, 3);
  // ACK: rescan, then one marker write and its read-back. No record is written.
  fs.log.clear();
  CHECK(store.acknowledge(s.generation, boundProfile()).status == AckStatus::OK);
  const std::vector<std::string> expected_ack = {"rA", "rB", "rM", "wM", "rM"};
  CHECK(fs.log == expected_ack);
  CHECK_EQ(fs.mutations, 4);
  CHECK_EQ(fs.slot_writes, 1);  // still only the SAVE's record write
}

void test_alternation_and_generations() {
  g_case = "A/B alternation";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  const auto profile = boundProfile();

  SaveResult r = store.save(record(), profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK(r.slot == CalibrationSlot::A);
  CHECK(!r.previous_record_intact);
  CHECK(!fs.present[1]);
  CHECK(store.acknowledge(1, profile).status == AckStatus::OK);
  expectSelected(store, 1, CalibrationSlot::A, false);

  const std::vector<uint8_t> a_after_first = fs.data[0];
  r = store.save(record(), profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK(r.slot == CalibrationSlot::B);
  CHECK_EQ(r.generation, 2);
  CHECK(r.previous_record_intact);
  CHECK_EQ(r.previous_generation, 1);
  CHECK(fs.data[0] == a_after_first);  // the acknowledged slot was not touched
  // Until it is acknowledged the new generation is neither served nor does it release A.
  CalibrationRecord loaded0;
  CHECK(store.load(profile, &loaded0).status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK(store.save(record(), profile).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK(fs.data[0] == a_after_first);
  CHECK(store.acknowledge(2, profile).status == AckStatus::OK);
  expectSelected(store, 2, CalibrationSlot::B, false);

  const std::vector<uint8_t> b_after_second = fs.data[1];
  r = store.save(record(), profile);
  CHECK(store.acknowledge(3, profile).status == AckStatus::OK);
  CHECK(r.slot == CalibrationSlot::A);
  CHECK_EQ(r.generation, 3);
  CHECK(fs.data[1] == b_after_second);

  for (int i = 4; i <= 30; ++i) {
    r = store.save(record(), profile);
    CHECK(r.status == SaveStatus::OK);
    CHECK_EQ(r.generation, i);
    CHECK_EQ(static_cast<int>(r.slot), (i - 1) % 2);
    CHECK(store.acknowledge(i, profile).status == AckStatus::OK);
  }
  expectSelected(store, 30, CalibrationSlot::B, false);

  // The caller's generation is ignored: the store owns it.
  r = store.save(goldenRecord(999), profile);
  CHECK(r.status == SaveStatus::OK);
  CHECK_EQ(r.generation, 31);
  CHECK(store.acknowledge(31, profile).status == AckStatus::OK);

  CalibrationRecord loaded;
  CHECK(store.load(profile, &loaded).status == LoadStatus::OK);
  CHECK(std::memcmp(loaded.digest, goldenRecord(1).digest, sizeof(loaded.digest)) == 0);
  CHECK_EQ(loaded.parameters_approved, 0);
}

void test_invalid_record_does_not_touch_storage() {
  g_case = "invalid record";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  CalibrationRecord bad = record();
  bad.parameters_approved = 1;
  SaveResult r = store.save(bad, boundProfile());
  CHECK(r.status == SaveStatus::INVALID_RECORD);
  CHECK(r.validation == CalibrationRecordStatus::FORBIDDEN_AUTHORIZATION);
  CHECK(r.phase == SavePhase::VALIDATE);
  CHECK_EQ(fs.reads, 0);
  CHECK_EQ(fs.mutations, 0);
  CHECK(store.writeState() == WriteState::OPEN);  // a refusal before storage is not an uncertain write

  bad = record();
  bad.joint[0].bus_id = 14;
  r = store.save(bad, boundProfile());
  CHECK(r.validation == CalibrationRecordStatus::IDENTITY_MISMATCH);
  CHECK_EQ(fs.mutations, 0);

  actuator::CalibrationGeometryProfile unbound;
  CHECK(store.save(record(), unbound).status == SaveStatus::INVALID_RECORD);
  CHECK_EQ(fs.mutations, 0);

  CalibrationRecordStore null_store(nullptr);
  CHECK(null_store.save(record(), boundProfile()).status == SaveStatus::BAD_ARGUMENT);
  CalibrationRecord out;
  CHECK(null_store.load(boundProfile(), &out).status == LoadStatus::BAD_ARGUMENT);
  CHECK(store.load(boundProfile(), nullptr).status == LoadStatus::BAD_ARGUMENT);
}

// ---- named failure scenarios (state seeded: gen 1 acknowledged in slot A) -----

// Common post-conditions of a failed SAVE started from a acknowledged gen 1.
void expectAcknowledgedUntouched(FakeStorage& fs, const std::vector<uint8_t>& slot_a_before) {
  CHECK(fs.present[0]);
  CHECK(fs.data[0] == slot_a_before);  // the acknowledged record's bytes never changed
}

void test_failure_before_pending_write() {
  g_case = "failure before PENDING";
  for (Fault f : {Fault::NOT_MODIFIED}) {
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    const std::vector<uint8_t> a = fs.data[0];
    SaveMarker before;
    CHECK(fs.decodedMarker(&before));
    fs.arm(1, f);
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::MARKER_WRITE_FAILED);
    CHECK(r.phase == SavePhase::MARKER_PENDING_WRITE);
    CHECK(r.io == StorageIoStatus::NOT_MODIFIED);
    CHECK_EQ(fs.slot_writes, 0);  // the record write never started
    // Storage certified it was untouched: nothing is uncertain, a retry is legitimate...
    CHECK(store.writeState() == WriteState::OPEN);
    SaveMarker after;
    CHECK(fs.decodedMarker(&after) && after == before);
    expectAcknowledgedUntouched(fs, a);
    expectSelected(store, 1, CalibrationSlot::A, false);
    // ...and succeeds, without any reconciliation: nothing was published.
    fs.arm(0, Fault::NONE);
    const SaveResult again = store.save(record(), boundProfile());
    CHECK(again.status == SaveStatus::OK);
    CHECK_EQ(again.generation, 2);
  }
  {  // storage fails without certifying: uncertain, blocked
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.arm(1, Fault::IO_NOTHING);
    CHECK(store.save(record(), boundProfile()).status == SaveStatus::MARKER_WRITE_FAILED);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    fs.arm(0, Fault::NONE);
    CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  }
}

void test_pending_published_record_not_written() {
  g_case = "PENDING published, record not written";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  const std::vector<uint8_t> a = fs.data[0];
  fs.arm(2, Fault::NOT_MODIFIED);  // even a certified "nothing written" is uncertain now
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(r.phase == SavePhase::RECORD_WRITE);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(r.generation, 0);  // no acknowledged generation reported
  SaveMarker m;
  CHECK(fs.decodedMarker(&m));
  CHECK(m.state == SaveMarkerState::PENDING);
  CHECK_EQ(m.acknowledged_generation, 1);
  CHECK_EQ(m.begun_generation, 2);
  expectAcknowledgedUntouched(fs, a);

  // Same instance: refused, nothing read, nothing written.
  fs.arm(0, Fault::NONE);
  const int reads = fs.reads, muts = fs.mutations;
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(fs.reads, reads);
  CHECK_EQ(fs.mutations, muts);

  // New instance after reboot: PENDING with an absent record; not served, not overwritten.
  CalibrationRecordStore rebooted(&fs);
  CalibrationRecord out;
  const LoadResult l = rebooted.load(boundProfile(), &out);
  CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
  CHECK(l.assessment.cls == PersistenceClass::PENDING_RECORD_ABSENT);
  CHECK(l.assessment.acknowledged_record_intact);  // gen 1 is still there, but not served
  CHECK(l.assessment.reconciliation_required);
  const SaveResult s = rebooted.save(record(), boundProfile());
  CHECK(s.status == SaveStatus::RECONCILIATION_REQUIRED);
  CHECK(s.persistence == PersistenceClass::PENDING_RECORD_ABSENT);
  CHECK_EQ(fs.mutations, muts);
  expectAcknowledgedUntouched(fs, a);
}

void test_record_partially_written() {
  g_case = "record partially written";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  const std::vector<uint8_t> a = fs.data[0];
  fs.arm(2, Fault::TORN);
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(r.phase == SavePhase::RECORD_WRITE);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  expectAcknowledgedUntouched(fs, a);
  CHECK_EQ(fs.data[1].size(), kCalibrationRecordV1EncodedBytes / 2);

  fs.arm(0, Fault::NONE);
  CalibrationRecordStore rebooted(&fs);
  CalibrationRecord out;
  const LoadResult l = rebooted.load(boundProfile(), &out);
  CHECK(l.assessment.cls == PersistenceClass::PENDING_RECORD_ABSENT);  // the torn blob is not a record
  CHECK(l.report[1].state == SlotState::CORRUPT);
  CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
}

void test_record_written_but_commit_failed() {
  g_case = "record written, commit failed";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  const std::vector<uint8_t> a = fs.data[0];
  fs.arm(2, Fault::AFTER_DATA);
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::WRITE_FAILED);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  // The new record is complete and valid on disk, but the marker still says PENDING.
  SaveMarker m;
  CHECK(fs.decodedMarker(&m) && m.state == SaveMarkerState::PENDING);
  fs.arm(0, Fault::NONE);
  CalibrationRecordStore rebooted(&fs);
  CalibrationRecord out;
  const LoadResult l = rebooted.load(boundProfile(), &out);
  CHECK(l.assessment.cls == PersistenceClass::PENDING_RECORD_PRESENT);
  CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);  // NOT promoted
  CHECK(!l.assessment.record_available);
  CHECK(rebooted.save(record(), boundProfile()).status == SaveStatus::RECONCILIATION_REQUIRED);
  expectAcknowledgedUntouched(fs, a);
}

void test_record_readback_failed() {
  g_case = "record read-back failed";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  fs.fail_slot_read_after_slot_write = true;
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::READBACK_FAILED);
  CHECK(r.phase == SavePhase::RECORD_VERIFY);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(fs.marker_writes, 1);  // the AWAITING_ACK marker was never written
  SaveMarker m;
  CHECK(fs.decodedMarker(&m) && m.state == SaveMarkerState::PENDING);

  fs.reboot();
  CHECK(classOf(fs) == PersistenceClass::PENDING_RECORD_PRESENT);
}

void test_record_readback_mismatch() {
  g_case = "record read-back mismatch";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  fs.arm(2, Fault::SILENT_CORRUPTION);
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::READBACK_MISMATCH);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(fs.marker_writes, 1);
  fs.reboot();
  CHECK(classOf(fs) == PersistenceClass::PENDING_RECORD_ABSENT);  // the corrupt blob is no record
}

void test_completed_marker_partially_written() {
  g_case = "AWAITING_ACK marker partially written";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  const std::vector<uint8_t> a = fs.data[0];
  fs.arm(3, Fault::TORN);
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::MARKER_WRITE_FAILED);
  CHECK(r.phase == SavePhase::MARKER_AWAITING_ACK_WRITE);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(r.generation, 0);
  fs.arm(0, Fault::NONE);
  CalibrationRecordStore rebooted(&fs);
  CalibrationRecord out;
  const LoadResult l = rebooted.load(boundProfile(), &out);
  CHECK(l.assessment.cls == PersistenceClass::MARKER_CORRUPT);
  CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
  CHECK(rebooted.save(record(), boundProfile()).status == SaveStatus::RECONCILIATION_REQUIRED);
  expectAcknowledgedUntouched(fs, a);
}

void test_awaiting_published_but_error_returned() {
  g_case = "AWAITING_ACK published, error returned";
  FakeStorage fs;
  seed(&fs, 1);
  const std::vector<uint8_t> a = fs.data[0];
  CalibrationRecordStore store(&fs);
  fs.arm(3, Fault::AFTER_DATA);
  const SaveResult r = store.save(record(), boundProfile());
  CHECK(r.status == SaveStatus::MARKER_WRITE_FAILED);  // the caller is NOT told it succeeded
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK_EQ(r.generation, 0);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  // The record and the marker are intact on disk, but the caller never got SAVE=OK:
  // a reboot reads "verified, awaiting ACK". Generation 2 is NOT served, NOT promoted,
  // and generation 1 stays protected.
  fs.reboot();
  CalibrationRecordStore rebooted(&fs);
  CalibrationRecord out;
  const LoadResult l = rebooted.load(boundProfile(), &out);
  CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK(l.assessment.cls == PersistenceClass::AWAITING_ACK);
  CHECK(!l.assessment.record_available);
  CHECK(rebooted.save(record(), boundProfile()).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
  CHECK_EQ(fs.mutations, 0);
  expectAcknowledgedUntouched(fs, a);
}

void test_marker_readback_failed() {
  g_case = "marker read-back failed";
  {  // PENDING verify read fails
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.fail_marker_read_from_marker_write = 1;
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::MARKER_VERIFY_FAILED);
    CHECK(r.phase == SavePhase::MARKER_PENDING_VERIFY);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    CHECK_EQ(fs.slot_writes, 0);  // the record was never written on an unverified PENDING
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::PENDING_RECORD_ABSENT);
  }
  {  // AWAITING_ACK verify read fails: marker is in fact fine on disk, instance still blocked
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.fail_marker_read_from_marker_write = 2;
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::MARKER_VERIFY_FAILED);
    CHECK(r.phase == SavePhase::MARKER_AWAITING_ACK_VERIFY);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    CHECK_EQ(r.generation, 0);
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::AWAITING_ACK);  // verified on disk, never acknowledged
  }
  {  // AWAITING_ACK written with a silent flipped byte: verify catches it
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.arm(3, Fault::SILENT_CORRUPTION);
    const SaveResult r = store.save(record(), boundProfile());
    CHECK(r.status == SaveStatus::MARKER_VERIFY_FAILED);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::MARKER_CORRUPT);
  }
}

// ---- every (mutation, fault) pair, then a reboot ---------------------------

void test_fault_matrix() {
  g_case = "fault matrix";
  int combos = 0;
  for (int at = 1; at <= 3; ++at) {
    for (Fault f : kAllFaults) {
      ++combos;
      FakeStorage fs;
      seed(&fs, 1);
      const std::vector<uint8_t> a = fs.data[0];
      CalibrationRecordStore store(&fs);
      fs.arm(at, f);
      const SaveResult r = store.save(record(), boundProfile());
      const std::string label = std::string("at=") + std::to_string(at) + " fault=" + faultName(f);
      g_case = label.c_str();

      CHECK(r.status != SaveStatus::OK);        // a fault is never reported as success
      CHECK_EQ(r.generation, 0);                // ...and never carries a acknowledged generation
      CHECK(fs.data[0] == a);                   // the acknowledged record's bytes are untouched
      const int muts_at_failure = fs.mutations;
      CHECK(muts_at_failure <= at);             // nothing was attempted after the fault

      const bool certain_untouched = at == 1 && f == Fault::NOT_MODIFIED;
      CHECK_EQ(store.writeState() == WriteState::OPEN, certain_untouched);

      if (!certain_untouched) {
        const int reads = fs.reads;
        CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
        CHECK_EQ(fs.reads, reads);
        CHECK_EQ(fs.mutations, muts_at_failure);  // no automatic retry
      }

      // Reboot: a new instance on the very same bytes.
      fs.reboot();
      CalibrationRecordStore rebooted(&fs);
      CalibrationRecord out;
      const LoadResult l = rebooted.load(boundProfile(), &out);
      CHECK(fs.data[0] == a);
      if (l.status == LoadStatus::OK) {
        // Served only if CONSISTENT with the marker's own acknowledged generation, and
        // that can only be the generation 1 acknowledged before the faulty SAVE.
        SaveMarker m;
        CHECK(fs.decodedMarker(&m));
        CHECK(m.state == SaveMarkerState::IDLE);
        CHECK_EQ(m.acknowledged_generation, l.generation);
        CHECK_EQ(l.generation, 1);
        CHECK(out.generation == l.generation);
        CHECK(l.assessment.cls == PersistenceClass::CONSISTENT);
      } else {
        const bool awaiting = l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED;
        CHECK(awaiting || l.status == LoadStatus::RECONCILIATION_REQUIRED);
        CHECK(l.assessment.reconciliation_required);
        CHECK(!l.assessment.save_allowed && !l.assessment.record_available);
        const int before = fs.mutations;
        const SaveResult s = rebooted.save(record(), boundProfile());
        CHECK(s.status == (awaiting ? SaveStatus::ACKNOWLEDGMENT_REQUIRED : SaveStatus::RECONCILIATION_REQUIRED));
        CHECK_EQ(fs.mutations, before);
        CHECK(fs.data[0] == a);
      }
      g_case = "fault matrix";
    }
  }
  CHECK_EQ(combos, 27);
}

// Same, starting from an empty partition (first install).
void test_fault_matrix_first_install() {
  g_case = "fault matrix, first install";
  for (int at = 1; at <= 3; ++at) {
    for (Fault f : kAllFaults) {
      FakeStorage fs;
      CalibrationRecordStore store(&fs);
      fs.arm(at, f);
      const SaveResult r = store.save(record(), boundProfile());
      const std::string label = std::string("first at=") + std::to_string(at) + " fault=" + faultName(f);
      g_case = label.c_str();
      CHECK(r.status != SaveStatus::OK);
      fs.reboot();
      CalibrationRecordStore rebooted(&fs);
      CalibrationRecord out;
      const LoadResult l = rebooted.load(boundProfile(), &out);
      // A SAVE that was never acknowledged cannot be served, whatever it reached.
      CHECK(l.status != LoadStatus::OK);
      CHECK(!l.assessment.record_available);
      CHECK(l.status == LoadStatus::NOT_FOUND || l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED ||
            l.status == LoadStatus::RECONCILIATION_REQUIRED);
      if (at == 3 && (f == Fault::AFTER_DATA || f == Fault::POWER_AFTER)) {
        CHECK(l.assessment.cls == PersistenceClass::AWAITING_ACK);  // first SAVE, verified, unacknowledged
      }
      g_case = "fault matrix, first install";
    }
  }
}

void test_two_consecutive_errors() {
  g_case = "two consecutive errors";
  FakeStorage fs;
  seed(&fs, 1);
  const std::vector<uint8_t> a = fs.data[0];
  CalibrationRecordStore store(&fs);
  fs.arm(2, Fault::IO_NOTHING);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::WRITE_FAILED);
  fs.arm(0, Fault::NONE);
  const int muts = fs.mutations;
  for (int i = 0; i < 3; ++i) {
    CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
  }
  CHECK_EQ(fs.mutations, muts);

  // After a reboot the persistent PENDING marker keeps refusing, repeatedly.
  for (int i = 0; i < 3; ++i) {
    fs.reboot();
    CalibrationRecordStore again(&fs);
    CHECK(again.save(record(), boundProfile()).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(again.save(record(), boundProfile()).status == SaveStatus::RECONCILIATION_REQUIRED);
  }
  CHECK_EQ(fs.mutations, 0);  // reboot() reset the counter: no mutation since
  CHECK(fs.data[0] == a);
  CHECK(fs.present[0]);
}

void test_error_then_error_cannot_erase_acknowledged() {
  g_case = "retries cannot erase the acknowledged record";
  // A long sequence of failing boots/saves never changes slot A.
  FakeStorage fs;
  seed(&fs, 1);
  const std::vector<uint8_t> a = fs.data[0];
  {
    CalibrationRecordStore s(&fs);
    fs.arm(2, Fault::POWER_TORN);
    s.save(record(), boundProfile());
  }
  for (int i = 0; i < 10; ++i) {
    fs.reboot();
    CalibrationRecordStore s(&fs);
    CHECK(s.save(record(), boundProfile()).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK_EQ(fs.mutations, 0);
    CHECK(fs.data[0] == a);
  }
}

// ---- states a reboot can find ---------------------------------------------

void test_load_serves_only_consistent() {
  g_case = "load serves only CONSISTENT";
  const auto profile = boundProfile();
  CalibrationRecord out;
  {  // records but no marker (layout from before the marker, or a lost marker)
    FakeStorage fs;
    fs.putSlot(0, goldenRecord(1));
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(l.assessment.cls == PersistenceClass::MARKER_MISSING);
    CHECK(store.save(record(), profile).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK_EQ(fs.mutations, 0);
  }
  {  // marker attests 3, only 2 survives: NOT healthy
    FakeStorage fs;
    fs.putSlot(1, goldenRecord(2));
    fs.putMarker(SaveMarkerState::IDLE, 3, 3);
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
    CHECK(l.assessment.older_record_survives);
    CHECK(!l.assessment.record_available);
    CHECK(store.save(record(), profile).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK_EQ(fs.mutations, 0);
  }
  {  // marker attests 3, nothing valid survives
    FakeStorage fs;
    fs.putSlot(0, goldenRecord(3));
    fs.data[0][600] ^= 1;
    fs.putMarker(SaveMarkerState::IDLE, 3, 3);
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
    CHECK(!l.assessment.older_record_survives);
  }
  {  // record above the acknowledged generation, marker IDLE
    FakeStorage fs;
    fs.putSlot(0, goldenRecord(1));
    fs.putSlot(1, goldenRecord(2));
    fs.putMarker(SaveMarkerState::IDLE, 1, 1);
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(l.assessment.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);
    CHECK(store.save(record(), profile).status == SaveStatus::RECONCILIATION_REQUIRED);
  }
  {  // both records valid with the same generation
    FakeStorage fs;
    fs.putSlot(0, goldenRecord(2));
    fs.putSlot(1, goldenRecord(2));
    fs.putMarker(SaveMarkerState::IDLE, 2, 2);
    CalibrationRecordStore store(&fs);
    CHECK(store.load(profile, &out).assessment.cls == PersistenceClass::RECORD_GENERATION_CONFLICT);
  }
  {  // both records corrupt, marker IDLE
    FakeStorage fs;
    fs.putSlot(0, goldenRecord(1));
    fs.putSlot(1, goldenRecord(2));
    fs.data[0][500] ^= 1;
    fs.data[1][500] ^= 1;
    fs.putMarker(SaveMarkerState::IDLE, 2, 2);
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  }
  {  // corrupt marker, valid records
    FakeStorage fs;
    seed(&fs, 2);
    fs.marker_data[13] ^= 0x01;
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
    CHECK(l.assessment.cls == PersistenceClass::MARKER_CORRUPT);
    CHECK(l.marker.detail == SaveMarkerStatus::BAD_CRC);
  }
  {  // foreign marker schema: this build neither reads nor replaces it
    FakeStorage fs;
    seed(&fs, 2);
    fs.marker_data[4] = 3;  // a schema this build does not know ...
    const uint32_t crc = calibrationCrc32(fs.marker_data.data(), fs.marker_data.size() - 4);
    for (int i = 0; i < 4; ++i) fs.marker_data[fs.marker_data.size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::INCOMPATIBLE);
    CHECK(l.assessment.cls == PersistenceClass::MARKER_INCOMPATIBLE);
    CHECK(store.save(record(), profile).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK_EQ(fs.mutations, 0);
  }
  {  // acknowledged record foreign (another schema), intact
    FakeStorage fs;
    seed(&fs, 1);
    fs.data[0][4] = 2;
    const uint32_t crc = calibrationCrc32(fs.data[0].data(), fs.data[0].size() - 4);
    for (int i = 0; i < 4; ++i) fs.data[0][fs.data[0].size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::INCOMPATIBLE);
    CHECK(l.assessment.cls == PersistenceClass::RECORD_INCOMPATIBLE);
  }
  {  // marker IDLE{0,1}: nothing acknowledged; no calibration, saving allowed
    FakeStorage fs;
    fs.putMarker(SaveMarkerState::IDLE, 0, 1);
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::NOT_FOUND);
    CHECK(l.assessment.cls == PersistenceClass::NOTHING_ACKNOWLEDGED);
    const SaveResult s = store.save(record(), profile);
    CHECK(s.status == SaveStatus::OK);
    CHECK_EQ(s.generation, 2);  // never reuses a begun generation
  }
}

void test_storage_errors_fail_closed() {
  g_case = "storage errors fail closed";
  const auto profile = boundProfile();
  CalibrationRecord out;
  for (int which = 0; which < 3; ++which) {
    FakeStorage fs;
    seed(&fs, 2);
    if (which < 2) fs.fail_read[which] = true; else fs.fail_marker_read = true;
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.status == LoadStatus::IO_ERROR);
    CHECK(l.assessment.cls == PersistenceClass::IO_ERROR);
    CHECK(!l.assessment.record_available);
    fs.mutations = 0;
    const SaveResult s = store.save(record(), profile);
    CHECK(s.status == SaveStatus::STORAGE_UNUSABLE);
    CHECK_EQ(fs.mutations, 0);
    CHECK(store.writeState() == WriteState::OPEN);  // nothing was written, nothing is uncertain
  }
  {  // oversize blobs are damage, not I/O errors
    FakeStorage fs;
    seed(&fs, 1);
    fs.oversize[0] = true;
    CalibrationRecordStore store(&fs);
    const LoadResult l = store.load(profile, &out);
    CHECK(l.report[0].state == SlotState::CORRUPT);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  }
  {
    FakeStorage fs;
    seed(&fs, 1);
    fs.oversize_marker = true;
    CalibrationRecordStore store(&fs);
    CHECK(store.load(profile, &out).assessment.cls == PersistenceClass::MARKER_CORRUPT);
  }
}

void test_generation_monotonic_over_leftovers() {
  g_case = "generations over leftovers";
  const auto profile = boundProfile();
  FakeStorage fs;
  seed(&fs, 1);
  // A corrupt-but-envelope-intact leftover with a high generation, plus a marker above it.
  CalibrationRecord bad = goldenRecord(7);
  bad.joint[3].diagnostics.scale_permille += 1;
  fs.putSlot(1, bad);
  fs.putMarker(SaveMarkerState::IDLE, 1, 7);  // "7 was discarded"
  CalibrationRecordStore store(&fs);
  expectSelected(store, 1, CalibrationSlot::A, true);
  const SaveResult s = store.save(record(), profile);
  CHECK(s.status == SaveStatus::OK);
  CHECK_EQ(s.generation, 8);
  CHECK(s.slot == CalibrationSlot::B);
  CHECK(store.acknowledge(8, profile).status == AckStatus::OK);
  expectSelected(store, 8, CalibrationSlot::B, false);
}

void test_both_slots_unusable_first_save_recovers_nothing() {
  g_case = "nothing acknowledged, leftovers";
  const auto profile = boundProfile();
  FakeStorage fs;
  fs.putSlot(0, goldenRecord(4));  // valid but never acknowledged ...
  fs.putMarker(SaveMarkerState::IDLE, 0, 4);  // ... explicitly declared not acknowledged
  CalibrationRecordStore store(&fs);
  CalibrationRecord out;
  const LoadResult l = store.load(profile, &out);
  CHECK(l.status == LoadStatus::NOT_FOUND);
  CHECK(l.assessment.cls == PersistenceClass::NOTHING_ACKNOWLEDGED);
  const SaveResult s = store.save(record(), profile);
  CHECK(s.status == SaveStatus::OK);
  CHECK_EQ(s.generation, 5);
  CHECK(s.slot == CalibrationSlot::B);  // never the leftover when an absent slot exists
}

// ---- explicit reconciliation, end to end ----------------------------------

void applyPlan(FakeStorage* fs, const SaveMarker& m) {
  fs->putMarker(m.state, m.acknowledged_generation, m.begun_generation);
}

PersistenceInputs inputsOf(FakeStorage& fs) {
  CalibrationRecordStore s(&fs);
  CalibrationRecord out;
  const LoadResult l = s.load(boundProfile(), &out);
  PersistenceInputs in;
  for (int i = 0; i < 2; ++i) in.slot[i] = l.report[i];
  in.marker = l.marker;
  return in;
}

void test_reconciliation_end_to_end() {
  g_case = "reconciliation";
  const auto profile = boundProfile();
  CalibrationRecord out;
  {  // interrupted SAVE whose record is complete: operator adopts it
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.arm(3, Fault::POWER_BEFORE);
    store.save(record(), profile);
    fs.reboot();
    PersistenceInputs in = inputsOf(fs);
    CHECK(classifyPersistence(in).cls == PersistenceClass::PENDING_RECORD_PRESENT);
    SaveMarker m;
    CHECK(planReconciliation(in, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::OK);
    applyPlan(&fs, m);
    CalibrationRecordStore after(&fs);
    expectSelected(after, 2, CalibrationSlot::B, false);
    const SaveResult s = after.save(record(), profile);
    CHECK(s.status == SaveStatus::OK);
    CHECK_EQ(s.generation, 3);
  }
  {  // same, operator prefers the previous acknowledged one
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.arm(3, Fault::POWER_BEFORE);
    store.save(record(), profile);
    fs.reboot();
    PersistenceInputs in = inputsOf(fs);
    SaveMarker m;
    CHECK(planReconciliation(in, ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
    applyPlan(&fs, m);
    CalibrationRecordStore after(&fs);
    expectSelected(after, 1, CalibrationSlot::A, false);  // gen 2 is a valid, discarded leftover
    const SaveResult s = after.save(record(), profile);
    CHECK(s.status == SaveStatus::OK);
    CHECK_EQ(s.generation, 3);  // 2 is never reused
    CHECK(s.slot == CalibrationSlot::B);
  }
  {  // acknowledged generation lost: declare nothing acknowledged, then a fresh SAVE
    FakeStorage fs;
    fs.putSlot(1, goldenRecord(2));
    fs.putMarker(SaveMarkerState::IDLE, 3, 3);
    PersistenceInputs in = inputsOf(fs);
    SaveMarker m;
    CHECK(planReconciliation(in, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
    applyPlan(&fs, m);
    CalibrationRecordStore after(&fs);
    CHECK(after.load(profile, &out).status == LoadStatus::NOT_FOUND);
    const SaveResult s = after.save(record(), profile);
    CHECK(s.status == SaveStatus::OK);
    CHECK_EQ(s.generation, 4);
    CHECK(after.acknowledge(4, profile).status == AckStatus::OK);  // the old leftover (gen 2) does not block it
    expectSelected(after, 4, CalibrationSlot::A, false);
  }
}

void test_load_does_not_unlock() {
  g_case = "load does not unlock";
  FakeStorage fs;
  seed(&fs, 1);
  CalibrationRecordStore store(&fs);
  fs.arm(2, Fault::IO_NOTHING);
  store.save(record(), boundProfile());
  fs.arm(0, Fault::NONE);
  CalibrationRecord out;
  for (int i = 0; i < 3; ++i) store.load(boundProfile(), &out);
  CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
  CHECK(store.save(record(), boundProfile()).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
}

void test_roundtrip_content_equality() {
  g_case = "round trip";
  FakeStorage fs;
  CalibrationRecordStore store(&fs);
  CalibrationRecord rec = goldenRecord(0);
  CHECK(store.save(rec, boundProfile()).status == SaveStatus::OK);
  CHECK(store.acknowledge(1, boundProfile()).status == AckStatus::OK);
  CalibrationRecord loaded;
  CHECK(store.load(boundProfile(), &loaded).status == LoadStatus::OK);
  rec.generation = 1;
  uint8_t a[1536], b[1536];
  size_t na = 0, nb = 0;
  CHECK(encodeCalibrationRecord(rec, a, sizeof(a), &na) == CalibrationRecordStatus::OK);
  CHECK(encodeCalibrationRecord(loaded, b, sizeof(b), &nb) == CalibrationRecordStatus::OK);
  CHECK(na == nb && std::memcmp(a, b, na) == 0);
}

void test_generation_exhausted() {
  g_case = "generation exhausted";
  FakeStorage fs;
  fs.putSlot(0, goldenRecord(0xFFFFFFFFu));
  fs.putMarker(SaveMarkerState::IDLE, 0xFFFFFFFFu, 0xFFFFFFFFu);
  CalibrationRecordStore store(&fs);
  const SaveResult s = store.save(record(), boundProfile());
  CHECK(s.status == SaveStatus::GENERATION_EXHAUSTED);
  CHECK_EQ(fs.mutations, 0);
}

void test_to_strings() {
  g_case = "toString";
  CHECK(std::strcmp(toString(SaveStatus::MARKER_VERIFY_FAILED), "MARKER_VERIFY_FAILED") == 0);
  CHECK(std::strcmp(toString(LoadStatus::RECONCILIATION_REQUIRED), "RECONCILIATION_REQUIRED") == 0);
  CHECK(std::strcmp(toString(SavePhase::MARKER_AWAITING_ACK_VERIFY), "MARKER_AWAITING_ACK_VERIFY") == 0);
  CHECK(std::strcmp(toString(SaveStatus::BLOCKED_UNCERTAIN_WRITE), "BLOCKED_UNCERTAIN_WRITE") == 0);
  CHECK(std::strcmp(toString(SaveStatus::ACKNOWLEDGMENT_REQUIRED), "ACKNOWLEDGMENT_REQUIRED") == 0);
  CHECK(std::strcmp(toString(LoadStatus::ACKNOWLEDGMENT_REQUIRED), "ACKNOWLEDGMENT_REQUIRED") == 0);
  CHECK(std::strcmp(toString(AckStatus::ALREADY_ACKNOWLEDGED), "ALREADY_ACKNOWLEDGED") == 0);
  CHECK(std::strcmp(toString(AckStatus::MARKER_VERIFY_FAILED), "MARKER_VERIFY_FAILED") == 0);
}

// ---- P2.4.1: durable acknowledgment ----------------------------------------

AckStatus ackOf(FakeStorage& fs, uint32_t g) {
  CalibrationRecordStore s(&fs);
  return s.acknowledge(g, boundProfile()).status;
}

void test_acknowledge_contract() {
  g_case = "acknowledge contract";
  const auto profile = boundProfile();
  {  // nothing stored / nothing awaiting
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    CHECK(store.acknowledge(1, profile).status == AckStatus::NOT_AWAITING);
    CHECK(store.acknowledge(0, profile).status == AckStatus::BAD_ARGUMENT);
    CalibrationRecordStore null_store(nullptr);
    CHECK(null_store.acknowledge(1, profile).status == AckStatus::BAD_ARGUMENT);
    CHECK_EQ(fs.mutations, 0);
  }
  {  // acknowledged state: idempotent for the acknowledged generation only
    FakeStorage fs;
    seed(&fs, 2);
    CHECK(ackOf(fs, 2) == AckStatus::ALREADY_ACKNOWLEDGED);
    CHECK(ackOf(fs, 1) == AckStatus::NOT_AWAITING);
    CHECK(ackOf(fs, 3) == AckStatus::NOT_AWAITING);
    CHECK_EQ(fs.mutations, 0);
  }
  {  // awaiting generation 2: only 2, exactly once
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    fs.mutations = 0;
    const std::vector<uint8_t> a = fs.data[0], b = fs.data[1];
    CHECK(store.acknowledge(1, profile).status == AckStatus::GENERATION_MISMATCH);  // a neighbour
    CHECK(store.acknowledge(3, profile).status == AckStatus::GENERATION_MISMATCH);  // a future one
    CHECK(store.acknowledge(0, profile).status == AckStatus::BAD_ARGUMENT);
    CHECK_EQ(fs.mutations, 0);
    CHECK(store.writeState() == WriteState::OPEN);  // refusals are not uncertain writes
    CHECK(classOf(fs) == PersistenceClass::AWAITING_ACK);
    const AckResult ok = store.acknowledge(2, profile);
    CHECK(ok.status == AckStatus::OK);
    CHECK(ok.persistence == PersistenceClass::AWAITING_ACK);
    CHECK_EQ(fs.mutations, 1);
    CHECK(fs.data[0] == a && fs.data[1] == b);  // an ACK writes the marker only
    CHECK(classOf(fs) == PersistenceClass::CONSISTENT);
    CHECK(store.acknowledge(2, profile).status == AckStatus::ALREADY_ACKNOWLEDGED);  // duplicate
    CHECK(store.acknowledge(1, profile).status == AckStatus::NOT_AWAITING);          // stale
    CHECK_EQ(fs.mutations, 1);
    expectSelected(store, 2, CalibrationSlot::B, false);
  }
  {  // ACK of an incomplete record: the SAVE never reached its verified marker
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    fs.arm(3, Fault::POWER_BEFORE);  // record on flash, marker still PENDING
    store.save(record(), profile);
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::PENDING_RECORD_PRESENT);
    CHECK(ackOf(fs, 2) == AckStatus::RECORD_NOT_VALID);
    CHECK(ackOf(fs, 1) == AckStatus::GENERATION_MISMATCH);
    CHECK_EQ(fs.mutations, 0);
    CHECK(classOf(fs) == PersistenceClass::PENDING_RECORD_PRESENT);
  }
  {  // verified generation whose record is damaged afterwards / incompatible / gone
    for (int kind = 0; kind < 3; ++kind) {
      FakeStorage fs;
      seed(&fs, 1);
      CalibrationRecordStore store(&fs);
      CHECK(store.save(record(), profile).status == SaveStatus::OK);
      fs.mutations = 0;
      if (kind == 0) fs.data[1][600] ^= 1;
      if (kind == 1) {
        fs.data[1][4] = 2;
        const uint32_t crc = calibrationCrc32(fs.data[1].data(), fs.data[1].size() - 4);
        for (int i = 0; i < 4; ++i) fs.data[1][fs.data[1].size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
      }
      if (kind == 2) fs.present[1] = false;
      CHECK(store.acknowledge(2, profile).status == AckStatus::RECORD_NOT_VALID);
      CHECK_EQ(fs.mutations, 0);
      CHECK(classOf(fs) != PersistenceClass::CONSISTENT);
    }
  }
  {  // acknowledged generation lost while another awaits: reconciliation, not ACK
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    fs.mutations = 0;
    fs.present[0] = false;
    const AckResult r = store.acknowledge(2, profile);
    CHECK(r.status == AckStatus::RECONCILIATION_REQUIRED);
    CHECK(r.persistence == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
    CHECK_EQ(fs.mutations, 0);
  }
  {  // marker missing / corrupt
    for (int kind = 0; kind < 2; ++kind) {
      FakeStorage fs;
      seed(&fs, 1);
      CalibrationRecordStore store(&fs);
      CHECK(store.save(record(), profile).status == SaveStatus::OK);
      fs.mutations = 0;
      if (kind == 0) fs.marker_present = false; else fs.marker_data[13] ^= 1;
      CHECK(store.acknowledge(2, profile).status == AckStatus::RECONCILIATION_REQUIRED);
      CHECK_EQ(fs.mutations, 0);
    }
  }
  {  // unreadable storage
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    fs.mutations = 0;
    fs.fail_read[1] = true;
    CHECK(store.acknowledge(2, profile).status == AckStatus::STORAGE_UNUSABLE);
    CHECK_EQ(fs.mutations, 0);
    CHECK(store.writeState() == WriteState::OPEN);
  }
}

// Fault on the ACK marker write itself. The protected slot is never touched, the
// outcome is conservative, and a reboot reads the real state.
void test_ack_write_faults() {
  g_case = "ack write faults";
  const auto profile = boundProfile();
  for (Fault f : kAllFaults) {
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    const std::vector<uint8_t> a = fs.data[0], b = fs.data[1];
    fs.reboot();  // counters back to zero; same store instance
    fs.arm(1, f);
    const AckResult r = store.acknowledge(2, profile);
    const std::string label = std::string("ack fault=") + faultName(f);
    g_case = label.c_str();
    CHECK(r.status != AckStatus::OK);  // a fault is never reported as an acknowledgment
    CHECK(r.status == (f == Fault::SILENT_CORRUPTION ? AckStatus::MARKER_VERIFY_FAILED
                                                     : AckStatus::MARKER_WRITE_FAILED));
    CHECK(fs.data[0] == a && fs.data[1] == b);
    CHECK_EQ(fs.slot_writes, 0);
    // Conservative: unless storage certified it did nothing, this instance writes no more.
    CHECK_EQ(store.writeState() == WriteState::OPEN, f == Fault::NOT_MODIFIED);
    if (f != Fault::NOT_MODIFIED) {
      CHECK(store.save(record(), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);
    }

    fs.reboot();
    CalibrationRecordStore rebooted(&fs);
    CalibrationRecord out;
    const LoadResult l = rebooted.load(profile, &out);
    switch (l.assessment.cls) {
      case PersistenceClass::AWAITING_ACK: {  // the ACK did not land: still awaiting, still protected
        CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);
        CHECK(rebooted.save(record(), profile).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
        CHECK_EQ(fs.mutations, 0);
        CHECK(rebooted.acknowledge(2, profile).status == AckStatus::OK);  // and can be repeated
        CHECK(classOf(fs) == PersistenceClass::CONSISTENT);
        break;
      }
      case PersistenceClass::CONSISTENT: {  // the ACK landed, the reply was lost
        CHECK(l.status == LoadStatus::OK);
        CHECK_EQ(l.generation, 2);
        CHECK(rebooted.acknowledge(2, profile).status == AckStatus::ALREADY_ACKNOWLEDGED);
        CHECK_EQ(fs.mutations, 0);
        break;
      }
      case PersistenceClass::MARKER_CORRUPT: {  // torn marker: nothing is guessed
        CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);
        CHECK(rebooted.save(record(), profile).status == SaveStatus::RECONCILIATION_REQUIRED);
        CHECK(rebooted.acknowledge(2, profile).status == AckStatus::RECONCILIATION_REQUIRED);
        CHECK_EQ(fs.mutations, 0);
        break;
      }
      default:
        CHECK(false);  // no other state is reachable from a faulty ACK
    }
    CHECK(fs.data[0] == a && fs.data[1] == b);
    g_case = "ack write faults";
  }
  // The exact stories.
  {
    FakeStorage fs;  // ACK persisted, reply lost (commit error after the data landed)
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    const std::vector<uint8_t> a = fs.data[0], b = fs.data[1];
    fs.reboot();
    fs.arm(1, Fault::AFTER_DATA);
    CHECK(store.acknowledge(2, profile).status == AckStatus::MARKER_WRITE_FAILED);
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::CONSISTENT);  // really acknowledged
    CalibrationRecordStore rebooted(&fs);
    CHECK(rebooted.acknowledge(2, profile).status == AckStatus::ALREADY_ACKNOWLEDGED);
    // Only now is slot A reusable: the next SAVE writes it, and only it.
    const SaveResult n = rebooted.save(record(), profile);
    CHECK(n.status == SaveStatus::OK && n.slot == CalibrationSlot::A && n.generation == 3);
    CHECK(fs.data[1] == b);
    CHECK(fs.data[0] != a);
  }
  {
    FakeStorage fs;  // ACK marker verify read fails: the marker is in fact fine on disk
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
    fs.reboot();
    fs.fail_marker_read_from_marker_write = 1;
    CHECK(store.acknowledge(2, profile).status == AckStatus::MARKER_VERIFY_FAILED);
    CHECK(store.writeState() == WriteState::BLOCKED_UNCERTAIN_WRITE);
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::CONSISTENT);  // the marker had landed after all
  }
}

// P2.4.1 regression for the review finding: A/1 acknowledged; B/2 written and
// verified; the marker is published but the caller never gets SAVE=OK; reboot; a
// new SAVE; a power cut in its write. A/1 must stay byte-for-byte intact.
void test_review_finding_regression() {
  g_case = "review finding";
  const auto profile = boundProfile();
  enum Variant { WRITE_FAILED_AFTER_DATA, WRITE_FAILED_POWER_AFTER, VERIFY_FAILED, OK_NEVER_ACKED, ACK_NEVER_SENT_N_BOOTS };
  for (int v = 0; v <= ACK_NEVER_SENT_N_BOOTS; ++v) {
    FakeStorage fs;
    seed(&fs, 1);
    const std::vector<uint8_t> a = fs.data[0];
    {
      CalibrationRecordStore store(&fs);
      switch (v) {
        case WRITE_FAILED_AFTER_DATA:
          fs.arm(3, Fault::AFTER_DATA);
          CHECK(store.save(record(), profile).status == SaveStatus::MARKER_WRITE_FAILED);
          break;
        case WRITE_FAILED_POWER_AFTER:
          fs.arm(3, Fault::POWER_AFTER);
          CHECK(store.save(record(), profile).status == SaveStatus::MARKER_WRITE_FAILED);
          break;
        case VERIFY_FAILED:
          fs.fail_marker_read_from_marker_write = 2;
          CHECK(store.save(record(), profile).status == SaveStatus::MARKER_VERIFY_FAILED);
          break;
        default:
          CHECK(store.save(record(), profile).status == SaveStatus::OK);  // reply delivered, ACK not sent
          break;
      }
    }
    const int boots = v == ACK_NEVER_SENT_N_BOOTS ? 6 : 1;
    for (int boot = 0; boot < boots; ++boot) {
      fs.reboot();
      CalibrationRecordStore rebooted(&fs);
      CalibrationRecord out;
      const LoadResult l = rebooted.load(profile, &out);
      CHECK(l.status == LoadStatus::ACKNOWLEDGMENT_REQUIRED);  // B/2 is NOT confirmed
      CHECK(l.assessment.cls == PersistenceClass::AWAITING_ACK);
      CHECK(!l.assessment.record_available);
      // A new SAVE with a power cut in "its" write: it must never get to write.
      fs.arm(1, Fault::POWER_TORN);
      const SaveResult s = rebooted.save(record(), profile);
      CHECK(s.status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
      CHECK_EQ(fs.mutations, 0);
      CHECK_EQ(fs.slot_writes, 0);
      CHECK(fs.data[0] == a);
      fs.arm(2, Fault::POWER_TORN);  // and with the cut at the record write instead
      CHECK(rebooted.save(record(), profile).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
      CHECK_EQ(fs.mutations, 0);
      CHECK(fs.data[0] == a);
    }
    // The generation was never lost either: the caller can still acknowledge it.
    fs.reboot();
    CHECK(ackOf(fs, 2) == AckStatus::OK);
    CHECK(fs.data[0] == a);  // even now A is untouched until a later SAVE reuses it
    CalibrationRecordStore after(&fs);
    expectSelected(after, 2, CalibrationSlot::B, false);
  }
}

void test_first_save_states_are_distinguishable() {
  g_case = "first SAVE states";
  const auto profile = boundProfile();
  struct Row { int at; Fault f; PersistenceClass expect; };
  const Row rows[] = {
      {1, Fault::POWER_BEFORE, PersistenceClass::NEVER_INITIALIZED_OR_ERASED},  // nothing happened
      {1, Fault::POWER_AFTER, PersistenceClass::PENDING_RECORD_ABSENT},         // PENDING marker only
      {2, Fault::POWER_BEFORE, PersistenceClass::PENDING_RECORD_ABSENT},
      {2, Fault::POWER_TORN, PersistenceClass::PENDING_RECORD_ABSENT},          // torn record = no record
      {2, Fault::POWER_AFTER, PersistenceClass::PENDING_RECORD_PRESENT},        // record, marker still PENDING
      {3, Fault::POWER_BEFORE, PersistenceClass::PENDING_RECORD_PRESENT},
      {3, Fault::POWER_TORN, PersistenceClass::MARKER_CORRUPT},
      {3, Fault::POWER_AFTER, PersistenceClass::AWAITING_ACK},                  // verified, never acknowledged
  };
  for (const Row& row : rows) {
    FakeStorage fs;
    CalibrationRecordStore store(&fs);
    fs.arm(row.at, row.f);
    CHECK(store.save(record(), profile).status != SaveStatus::OK);
    fs.reboot();
    const PersistenceClass c = classOf(fs);
    CHECK(c == row.expect);
    CalibrationRecordStore rebooted(&fs);
    CalibrationRecord out;
    const LoadResult l = rebooted.load(profile, &out);
    CHECK(!l.assessment.record_available);  // no calibration data is declared available
    CHECK(l.status != LoadStatus::OK);
    if (c == PersistenceClass::AWAITING_ACK) {
      CHECK(l.assessment.acknowledged_generation == 0 && !l.assessment.acknowledged_record_intact);
      CHECK(rebooted.save(record(), profile).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
      CHECK(rebooted.acknowledge(1, profile).status == AckStatus::OK);
      expectSelected(rebooted, 1, CalibrationSlot::A, false);
    } else if (c != PersistenceClass::NEVER_INITIALIZED_OR_ERASED) {
      CHECK(!l.assessment.save_allowed);
    }
  }
}

void test_acknowledged_generation_lost_with_awaiting() {
  g_case = "acknowledged lost, one awaiting";
  const auto profile = boundProfile();
  for (int kind = 0; kind < 2; ++kind) {
    FakeStorage fs;
    seed(&fs, 1);
    CalibrationRecordStore store(&fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);  // B/2 verified, awaiting
    const std::vector<uint8_t> b = fs.data[1];
    if (kind == 0) fs.data[0][500] ^= 1; else fs.present[0] = false;  // A/1 damaged / gone
    fs.mutations = 0;
    CalibrationRecordStore rebooted(&fs);
    CalibrationRecord out;
    const LoadResult l = rebooted.load(profile, &out);
    CHECK(l.assessment.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);  // reported, not hidden
    CHECK(l.status == LoadStatus::RECONCILIATION_REQUIRED);  // no silent fallback to B/2
    CHECK(!l.assessment.record_available);
    CHECK(rebooted.save(record(), profile).status == SaveStatus::RECONCILIATION_REQUIRED);
    CHECK(rebooted.acknowledge(2, profile).status == AckStatus::RECONCILIATION_REQUIRED);
    CHECK_EQ(fs.mutations, 0);
    CHECK(fs.data[1] == b);
  }
}

void test_reconciliation_of_awaiting() {
  g_case = "reconciliation of awaiting";
  const auto profile = boundProfile();
  auto awaitingStorage = [&](FakeStorage* fs) {
    seed(fs, 1);
    CalibrationRecordStore store(fs);
    CHECK(store.save(record(), profile).status == SaveStatus::OK);
  };
  {  // ADOPT the verified generation: an operator acknowledgment
    FakeStorage fs;
    awaitingStorage(&fs);
    SaveMarker m;
    CHECK(planReconciliation(inputsOf(fs), ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::OK);
    applyPlan(&fs, m);
    CalibrationRecordStore after(&fs);
    expectSelected(after, 2, CalibrationSlot::B, false);
    const SaveResult s = after.save(record(), profile);
    CHECK(s.status == SaveStatus::OK && s.slot == CalibrationSlot::A && s.generation == 3);
  }
  {  // ADOPT the previous one: the verified generation is discarded, A stays byte-identical
    FakeStorage fs;
    awaitingStorage(&fs);
    const std::vector<uint8_t> a = fs.data[0];
    SaveMarker m;
    CHECK(planReconciliation(inputsOf(fs), ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
    applyPlan(&fs, m);
    CalibrationRecordStore after(&fs);
    expectSelected(after, 1, CalibrationSlot::A, false);
    fs.reboot();
    fs.arm(2, Fault::POWER_TORN);  // the next SAVE is cut in its record write
    after.save(record(), profile);
    CHECK(fs.data[0] == a);
    fs.reboot();
    CHECK(classOf(fs) == PersistenceClass::PENDING_RECORD_ABSENT);
    CHECK(fs.data[0] == a);
  }
  {  // a damaged / foreign verified record is never adoptable
    for (int kind = 0; kind < 2; ++kind) {
      FakeStorage fs;
      awaitingStorage(&fs);
      if (kind == 0) {
        fs.data[1][600] ^= 1;
      } else {
        fs.data[1][4] = 2;
        const uint32_t crc = calibrationCrc32(fs.data[1].data(), fs.data[1].size() - 4);
        for (int i = 0; i < 4; ++i) fs.data[1][fs.data[1].size() - 4 + i] = static_cast<uint8_t>(crc >> (8 * i));
      }
      SaveMarker m;
      CHECK(planReconciliation(inputsOf(fs), ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
      CHECK(planReconciliation(inputsOf(fs), ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
    }
  }
  {  // DECLARE nothing: fresh calibration required, the leftovers are never reused
    FakeStorage fs;
    awaitingStorage(&fs);
    SaveMarker m;
    CHECK(planReconciliation(inputsOf(fs), ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
    applyPlan(&fs, m);
    CalibrationRecordStore after(&fs);
    CalibrationRecord out;
    CHECK(after.load(profile, &out).status == LoadStatus::NOT_FOUND);
    const SaveResult s = after.save(record(), profile);
    CHECK(s.status == SaveStatus::OK && s.generation == 3);
  }
}

void test_two_consecutive_errors_awaiting() {
  g_case = "two consecutive errors, awaiting";
  const auto profile = boundProfile();
  FakeStorage fs;
  seed(&fs, 1);
  const std::vector<uint8_t> a = fs.data[0];
  CalibrationRecordStore store(&fs);
  fs.arm(3, Fault::AFTER_DATA);
  CHECK(store.save(record(), profile).status == SaveStatus::MARKER_WRITE_FAILED);  // error 1
  fs.arm(0, Fault::NONE);
  CHECK(store.save(record(), profile).status == SaveStatus::BLOCKED_UNCERTAIN_WRITE);  // error 2
  for (int i = 0; i < 4; ++i) {
    fs.reboot();
    CalibrationRecordStore again(&fs);
    CHECK(again.save(record(), profile).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
    CHECK(again.save(record(), profile).status == SaveStatus::ACKNOWLEDGMENT_REQUIRED);
    CHECK_EQ(fs.mutations, 0);
    CHECK(fs.data[0] == a);
  }
}

}  // namespace

int main() {
  test_empty_storage_is_first_install();
  test_protocol_order();
  test_alternation_and_generations();
  test_invalid_record_does_not_touch_storage();
  test_failure_before_pending_write();
  test_pending_published_record_not_written();
  test_record_partially_written();
  test_record_written_but_commit_failed();
  test_record_readback_failed();
  test_record_readback_mismatch();
  test_completed_marker_partially_written();
  test_awaiting_published_but_error_returned();
  test_marker_readback_failed();
  test_fault_matrix();
  test_fault_matrix_first_install();
  test_two_consecutive_errors();
  test_error_then_error_cannot_erase_acknowledged();
  test_load_serves_only_consistent();
  test_storage_errors_fail_closed();
  test_generation_monotonic_over_leftovers();
  test_both_slots_unusable_first_save_recovers_nothing();
  test_reconciliation_end_to_end();
  test_load_does_not_unlock();
  test_roundtrip_content_equality();
  test_generation_exhausted();
  test_acknowledge_contract();
  test_ack_write_faults();
  test_review_finding_regression();
  test_first_save_states_are_distinguishable();
  test_acknowledged_generation_lost_with_awaiting();
  test_reconciliation_of_awaiting();
  test_two_consecutive_errors_awaiting();
  test_to_strings();
  std::printf("calibration record store: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
