// Offline host tests for OTA-A: the update state machine
// (src/update/OtaPolicy.*), the first-boot rollback guard
// (src/update/OtaBootGuard.*) and the streaming SHA-256
// (src/update/Sha256.*).
//
// Links the REAL firmware translation units. That is possible because the
// ESP-IDF half is behind the OtaBackend interface, so a fake backend can
// substitute the flash I/O while the shipped decision logic runs unmodified.
// Only the flash is faked; every state transition, every ordering rule and
// every refusal below is the code that runs on the device.
//
// What this suite does NOT claim: nothing here proves a real image was
// written to a real partition, or that a real device rolled back. Those are
// hardware tests.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../../src/update/OtaBootGuard.h"
#include "../../src/update/OtaPolicy.h"
#include "../../src/update/Sha256.h"

using namespace matdog::update;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (!(cond)) {                                                             \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                          \
  } while (0)

#define CHECK_EQ(actual, expected)                                             \
  do {                                                                         \
    ++g_checks;                                                                \
    const long a_ = (long)(actual);                                            \
    const long e_ = (long)(expected);                                          \
    if (a_ != e_) {                                                            \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,      \
                  __FILE__, __LINE__, #actual, a_, e_);                        \
    }                                                                          \
  } while (0)

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                              \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));          \
    }                                                                          \
  } while (0)

#define CHECK_FAULT(policy, expected)                                          \
  do {                                                                         \
    ++g_checks;                                                                \
    if ((policy).fault() != (expected)) {                                      \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: fault == %s, expected %s\n", g_case,     \
                  __FILE__, __LINE__, toString((policy).fault()),              \
                  toString(expected));                                         \
    }                                                                          \
  } while (0)

// ---------------------------------------------------------------------------
// Fake backend: substitutes flash, fails anything on demand
// ---------------------------------------------------------------------------

static OtaPartitionInfo makePartition(const char* label, uint32_t addr, uint32_t size,
                                      uint8_t subtype) {
  OtaPartitionInfo p{};
  p.valid = true;
  p.address = addr;
  p.size = size;
  p.subtype = subtype;
  std::snprintf(p.label, sizeof(p.label), "%s", label);
  return p;
}

// The real MATDOG layout, from the app3M_fat9M_16MB partition table.
static OtaPartitionInfo app0() { return makePartition("app0", 0x10000,  0x300000, 0x10); }
static OtaPartitionInfo app1() { return makePartition("app1", 0x310000, 0x300000, 0x11); }

class FakeBackend : public OtaBackend {
 public:
  // --- injectable behaviour ---
  OtaPartitionInfo running = app0();
  OtaPartitionInfo next = app1();
  OtaImgState running_state = OtaImgState::VALID;
  bool fail_begin = false;
  bool fail_end = false;
  bool fail_set_boot = false;
  bool fail_mark_valid = false;
  bool rollback_ok = true;
  int fail_write_after_n_chunks = -1;  // -1 = never
  uint32_t short_write_after_bytes = 0xFFFFFFFFu;  // simulate a partial write

  // --- observed behaviour ---
  std::vector<uint8_t> written;
  int begin_calls = 0, write_calls = 0, end_calls = 0, abort_calls = 0;
  int set_boot_calls = 0, mark_valid_calls = 0;
  OtaPartitionInfo last_boot_target{};
  bool stream_open = false;

  OtaPartitionInfo runningPartition() override { return running; }
  OtaPartitionInfo nextUpdatePartition() override { return next; }
  OtaImgState imageState(const OtaPartitionInfo& p) override {
    return p.sameAs(running) ? running_state : OtaImgState::UNDEFINED;
  }
  bool beginWrite(const OtaPartitionInfo&, uint32_t) override {
    begin_calls++;
    if (fail_begin) return false;
    written.clear();
    stream_open = true;
    return true;
  }
  bool write(const uint8_t* data, uint32_t len) override {
    write_calls++;
    if (!stream_open) return false;
    if (fail_write_after_n_chunks >= 0 && write_calls > fail_write_after_n_chunks) return false;
    if (written.size() + len > short_write_after_bytes) return false;
    written.insert(written.end(), data, data + len);
    return true;
  }
  bool endWrite() override {
    end_calls++;
    stream_open = false;
    return !fail_end;
  }
  void abortWrite() override { abort_calls++; stream_open = false; }
  bool setBootPartition(const OtaPartitionInfo& target) override {
    set_boot_calls++;
    if (fail_set_boot) return false;
    last_boot_target = target;
    return true;
  }
  bool markAppValid() override { mark_valid_calls++; return !fail_mark_valid; }
  bool rollbackPossible() override { return rollback_ok; }
};

// ---------------------------------------------------------------------------
// Fixture helpers
// ---------------------------------------------------------------------------

static std::vector<uint8_t> makeImage(size_t n, uint8_t seed = 0x5A) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(seed + (i * 31u));
  return v;
}

static OtaImageMetadata metaFor(const std::vector<uint8_t>& image,
                                const char* build_id = "9953165904e0") {
  OtaImageMetadata m{};
  m.schema_version = kOtaMetadataSchema;
  m.image_size = static_cast<uint32_t>(image.size());
  Sha256 h;
  h.update(image.data(), image.size());
  h.finish(m.sha256);
  std::snprintf(m.build_id, sizeof(m.build_id), "%s", build_id);
  return m;
}

static OtaStageAGate g_gate;

static void wire(OtaPolicy& p, FakeBackend& b) { p.begin(&b, &g_gate); }

// Streams an image through in fixed chunks. Returns false at the first
// refusal, like a transport would.
static bool streamAll(OtaPolicy& p, const std::vector<uint8_t>& img, size_t chunk) {
  size_t i = 0;
  while (i < img.size()) {
    const size_t n = std::min(chunk, img.size() - i);
    if (!p.writeChunk(img.data() + i, static_cast<uint32_t>(n))) return false;
    i += n;
  }
  return true;
}

// Full happy path, used as the baseline for "did anything change?" cases.
static bool runHappyPath(OtaPolicy& p, FakeBackend&, const std::vector<uint8_t>& img) {
  if (!p.prepare(metaFor(img))) return false;
  if (!p.openStream()) return false;
  if (!streamAll(p, img, 1024)) return false;
  if (!p.finishStream()) return false;
  return p.commitBootTarget();
}

// ---------------------------------------------------------------------------
// Happy path
// ---------------------------------------------------------------------------

static void test_happy_path() {
  g_case = "happy_path";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(9000);

  CHECK(p.prepare(metaFor(img)));
  CHECK_EQ((int)p.state(), (int)OtaState::TARGET_RESOLVED);
  // Nothing has been written and the boot target is untouched at this point.
  CHECK_EQ(b.begin_calls, 0);
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
  CHECK_EQ(p.status().target.address, app1().address);

  CHECK(p.openStream());
  CHECK_EQ((int)p.state(), (int)OtaState::RECEIVING);
  CHECK_EQ(b.begin_calls, 1);
  CHECK_EQ(b.set_boot_calls, 0);

  CHECK(streamAll(p, img, 1024));
  CHECK_EQ(p.status().bytes_written, img.size());
  CHECK_EQ(p.status().chunks_written, 9u);
  CHECK_EQ(b.set_boot_calls, 0);  // still untouched mid-stream

  CHECK(p.finishStream());
  CHECK_EQ((int)p.state(), (int)OtaState::IDENTITY_VERIFIED);
  CHECK_EQ(b.end_calls, 1);
  CHECK_EQ(b.set_boot_calls, 0);  // still untouched after validation
  CHECK_STR(p.status().computed_sha256, p.status().declared_sha256);

  CHECK(p.commitBootTarget());
  CHECK_EQ((int)p.state(), (int)OtaState::BOOT_TARGET_SET);
  CHECK_EQ(b.set_boot_calls, 1);
  CHECK(p.status().boot_target_changed);
  CHECK_EQ(b.last_boot_target.address, app1().address);
  CHECK(!b.last_boot_target.sameAs(b.running));

  // The bytes that landed in the slot are exactly the bytes we sent.
  CHECK_EQ(b.written.size(), img.size());
  CHECK(std::memcmp(b.written.data(), img.data(), img.size()) == 0);
  CHECK_EQ(p.status().counters.updates_committed, 1u);
  CHECK_EQ(p.status().counters.updates_failed, 0u);
}

static void test_target_is_always_the_inactive_slot() {
  g_case = "target_is_always_the_inactive_slot";
  // Running from app1 must target app0, and vice versa.
  FakeBackend b; b.running = app1(); b.next = app0();
  OtaPolicy p; wire(p, b);
  const auto img = makeImage(512);
  CHECK(p.prepare(metaFor(img)));
  CHECK_EQ(p.status().target.address, app0().address);
  CHECK(!p.status().target.sameAs(b.running));
}

// ---------------------------------------------------------------------------
// Target resolution failures — all must fail closed, before any flash write
// ---------------------------------------------------------------------------

static void expectPrepareFails(const char* name, FakeBackend& b, OtaFault expected,
                               const OtaImageMetadata& meta) {
  g_case = name;
  OtaPolicy p; wire(p, b);
  CHECK(!p.prepare(meta));
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, expected);
  // The invariant that matters: a refused prepare touches nothing.
  CHECK_EQ(b.begin_calls, 0);
  CHECK_EQ(b.write_calls, 0);
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
  // And the stream cannot be opened afterwards.
  CHECK(!p.openStream());
  CHECK_EQ(b.begin_calls, 0);
}

static void test_target_equals_running_partition() {
  FakeBackend b;
  b.next = app0();      // backend wrongly hands back the running slot
  b.running = app0();
  expectPrepareFails("target_equals_running_partition", b, OtaFault::TARGET_IS_RUNNING,
                     metaFor(makeImage(512)));
}

static void test_target_is_not_an_ota_slot() {
  FakeBackend b;
  b.next = makePartition("factory", 0x310000, 0x300000, 0x00);  // factory, not ota_N
  expectPrepareFails("target_is_not_an_ota_slot", b, OtaFault::TARGET_NOT_OTA_SLOT,
                     metaFor(makeImage(512)));

  FakeBackend b2;
  b2.next = makePartition("test", 0x310000, 0x300000, 0x20);    // TEST app subtype
  expectPrepareFails("target_is_test_app_subtype", b2, OtaFault::TARGET_NOT_OTA_SLOT,
                     metaFor(makeImage(512)));
}

static void test_no_inactive_slot_available() {
  FakeBackend b;
  b.next = OtaPartitionInfo{};  // esp_ota_get_next_update_partition() returned NULL
  expectPrepareFails("no_inactive_slot_available", b, OtaFault::NO_INACTIVE_SLOT,
                     metaFor(makeImage(512)));
}

static void test_image_larger_than_partition() {
  FakeBackend b;
  OtaImageMetadata m = metaFor(makeImage(512));
  m.image_size = app1().size + 1;   // 3 MiB + 1
  expectPrepareFails("image_larger_than_partition", b, OtaFault::IMAGE_TOO_LARGE, m);

  // Exactly the partition size is allowed - the bound is <=, not <.
  g_case = "image_exactly_partition_size_is_allowed";
  FakeBackend b2; OtaPolicy p2; wire(p2, b2);
  OtaImageMetadata exact = metaFor(makeImage(512));
  exact.image_size = app1().size;
  CHECK(p2.prepare(exact));
  CHECK_EQ((int)p2.state(), (int)OtaState::TARGET_RESOLVED);
}

static void test_zero_length_image() {
  FakeBackend b;
  OtaImageMetadata m = metaFor(makeImage(512));
  m.image_size = 0;
  expectPrepareFails("zero_length_image", b, OtaFault::IMAGE_EMPTY, m);
}

// ---------------------------------------------------------------------------
// Metadata failures
// ---------------------------------------------------------------------------

static void test_metadata_incomplete() {
  const auto img = makeImage(512);

  { FakeBackend b; OtaImageMetadata m = metaFor(img); m.schema_version = 99;
    expectPrepareFails("metadata_unknown_schema", b, OtaFault::METADATA_SCHEMA, m); }

  { FakeBackend b; OtaImageMetadata m = metaFor(img); m.build_id[0] = '\0';
    expectPrepareFails("metadata_missing_build_id", b, OtaFault::METADATA_INCOMPLETE, m); }

  { FakeBackend b; OtaImageMetadata m = metaFor(img, "unknown");
    // An image that cannot say which commit it came from is refused: that is
    // BuildConfig.h's fallback for a build that skipped scripts/build.sh.
    expectPrepareFails("metadata_build_id_unknown", b, OtaFault::METADATA_INCOMPLETE, m); }

  { FakeBackend b; OtaImageMetadata m = metaFor(img);
    std::memset(m.sha256, 0, sizeof(m.sha256));
    expectPrepareFails("metadata_missing_hash", b, OtaFault::METADATA_INCOMPLETE, m); }

  { FakeBackend b; OtaImageMetadata m = metaFor(img);
    m.build_id[0] = 0x07;  // non-printable: this string is echoed to diagnostics
    expectPrepareFails("metadata_build_id_not_printable", b,
                       OtaFault::METADATA_INCOMPLETE, m); }
}

// ---------------------------------------------------------------------------
// Running image must be confirmed before a new update can start
// ---------------------------------------------------------------------------

static void test_update_refused_while_running_image_unconfirmed() {
  // Verified against ESP-IDF v5.5.5: esp_ota_begin() returns
  // ESP_ERR_OTA_ROLLBACK_INVALID_STATE when rollback is enabled and the
  // running app is PENDING_VERIFY. The real build has ROLLBACK_ENABLE=y.
  for (OtaImgState s : {OtaImgState::PENDING_VERIFY, OtaImgState::NEW}) {
    FakeBackend b; b.running_state = s;
    expectPrepareFails("update_refused_while_running_image_unconfirmed", b,
                       OtaFault::RUNNING_IMAGE_UNCONFIRMED, metaFor(makeImage(512)));
  }
  // ... and is allowed once the running image is confirmed.
  g_case = "update_allowed_once_confirmed";
  FakeBackend ok; ok.running_state = OtaImgState::VALID;
  OtaPolicy p; wire(p, ok);
  CHECK(p.prepare(metaFor(makeImage(512))));
}

// ---------------------------------------------------------------------------
// Stream failures — none may change the boot target
// ---------------------------------------------------------------------------

static void test_write_open_rejected() {
  g_case = "write_open_rejected";
  FakeBackend b; b.fail_begin = true;
  OtaPolicy p; wire(p, b);
  const auto img = makeImage(512);
  CHECK(p.prepare(metaFor(img)));
  CHECK(!p.openStream());
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, OtaFault::WRITE_OPEN_REJECTED);
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
}

static void test_write_error_mid_stream() {
  g_case = "write_error_mid_stream";
  FakeBackend b; b.fail_write_after_n_chunks = 3;
  OtaPolicy p; wire(p, b);
  const auto img = makeImage(9000);
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(!streamAll(p, img, 1024));
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, OtaFault::WRITE_FAILED);
  CHECK_EQ(b.abort_calls, 1);        // the open stream was released
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
  // No further writes are accepted after the failure.
  const uint8_t junk[4] = {1, 2, 3, 4};
  CHECK(!p.writeChunk(junk, 4));
}

static void test_partial_write() {
  g_case = "partial_write";
  // The backend accepts some bytes then refuses - a short write, not a
  // clean failure at a chunk boundary.
  FakeBackend b; b.short_write_after_bytes = 2500;
  OtaPolicy p; wire(p, b);
  const auto img = makeImage(9000);
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(!streamAll(p, img, 1024));
  CHECK_FAULT(p, OtaFault::WRITE_FAILED);
  CHECK(b.written.size() < img.size());
  CHECK_EQ(b.set_boot_calls, 0);
}

static void test_interrupted_stream_never_finishes() {
  g_case = "interrupted_stream_never_finishes";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(9000);
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  // Transport dies after 4 of 9 chunks.
  CHECK(streamAll(p, std::vector<uint8_t>(img.begin(), img.begin() + 4096), 1024));
  CHECK_EQ(p.status().bytes_written, 4096u);

  CHECK(!p.finishStream());
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, OtaFault::INCOMPLETE_STREAM);
  // Our own completeness check runs BEFORE the backend's image check, so the
  // backend is never even asked to validate a truncated image.
  CHECK_EQ(b.end_calls, 0);
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
}

static void test_overrun_is_rejected_before_it_is_written() {
  g_case = "overrun_is_rejected_before_it_is_written";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(streamAll(p, img, 1024));
  const size_t written_before = b.written.size();

  const uint8_t extra[16] = {0};
  CHECK(!p.writeChunk(extra, 16));
  CHECK_FAULT(p, OtaFault::OVERRUN);
  // Not one extra byte reached the slot.
  CHECK_EQ(b.written.size(), written_before);
  CHECK_EQ(b.set_boot_calls, 0);
}

static void test_image_rejected_by_backend() {
  g_case = "image_rejected_by_backend";
  // esp_ota_end() returning ESP_ERR_OTA_VALIDATE_FAILED: the bytes are not a
  // loadable app image.
  FakeBackend b; b.fail_end = true;
  OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(streamAll(p, img, 1024));
  CHECK(!p.finishStream());
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, OtaFault::IMAGE_REJECTED);
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
  CHECK(!p.commitBootTarget());
  CHECK_EQ(b.set_boot_calls, 0);
}

static void test_hash_mismatch() {
  g_case = "hash_mismatch";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);
  // Declared hash belongs to a DIFFERENT image of the same length: a valid
  // image, but not the expected one. This is the case esp_ota_end() alone
  // cannot catch.
  OtaImageMetadata m = metaFor(makeImage(4096, 0x11));
  m.image_size = static_cast<uint32_t>(img.size());

  CHECK(p.prepare(m));
  CHECK(p.openStream());
  CHECK(streamAll(p, img, 1024));
  CHECK(!p.finishStream());
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, OtaFault::HASH_MISMATCH);
  // The backend's own image check passed; ours did not.
  CHECK_EQ(b.end_calls, 1);
  CHECK_EQ(b.set_boot_calls, 0);
  CHECK(!p.status().boot_target_changed);
  CHECK(std::strcmp(p.status().computed_sha256, p.status().declared_sha256) != 0);
}

static void test_boot_switch_rejected() {
  g_case = "boot_switch_rejected";
  FakeBackend b; b.fail_set_boot = true;
  OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(streamAll(p, img, 1024));
  CHECK(p.finishStream());
  CHECK(!p.commitBootTarget());
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK_FAULT(p, OtaFault::BOOT_SWITCH_REJECTED);
  CHECK(!p.status().boot_target_changed);
}

// ---------------------------------------------------------------------------
// Ordering: the boot target changes only from the one validated state
// ---------------------------------------------------------------------------

static void test_commit_is_unreachable_from_every_other_state() {
  g_case = "commit_is_unreachable_from_every_other_state";
  const auto img = makeImage(4096);

  // IDLE
  { FakeBackend b; OtaPolicy p; wire(p, b);
    CHECK(!p.commitBootTarget()); CHECK_EQ(b.set_boot_calls, 0); }
  // TARGET_RESOLVED
  { FakeBackend b; OtaPolicy p; wire(p, b);
    CHECK(p.prepare(metaFor(img)));
    CHECK(!p.commitBootTarget()); CHECK_EQ(b.set_boot_calls, 0); }
  // RECEIVING, nothing sent
  { FakeBackend b; OtaPolicy p; wire(p, b);
    CHECK(p.prepare(metaFor(img))); CHECK(p.openStream());
    CHECK(!p.commitBootTarget()); CHECK_EQ(b.set_boot_calls, 0); }
  // RECEIVING, partially sent
  { FakeBackend b; OtaPolicy p; wire(p, b);
    CHECK(p.prepare(metaFor(img))); CHECK(p.openStream());
    CHECK(streamAll(p, std::vector<uint8_t>(img.begin(), img.begin() + 1024), 512));
    CHECK(!p.commitBootTarget()); CHECK_EQ(b.set_boot_calls, 0); }
  // FAILED
  { FakeBackend b; b.fail_end = true; OtaPolicy p; wire(p, b);
    CHECK(p.prepare(metaFor(img))); CHECK(p.openStream());
    CHECK(streamAll(p, img, 1024)); CHECK(!p.finishStream());
    CHECK(!p.commitBootTarget()); CHECK_EQ(b.set_boot_calls, 0); }
}

static void test_out_of_order_calls_are_refused() {
  g_case = "out_of_order_calls_are_refused";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);

  // Write before a stream is open.
  CHECK(!p.writeChunk(img.data(), 16));
  CHECK_EQ(b.write_calls, 0);
  // Finish before a stream is open.
  CHECK(!p.finishStream());
  CHECK_EQ(b.end_calls, 0);
  // Open before a target is resolved.
  CHECK(!p.openStream());
  CHECK_EQ(b.begin_calls, 0);

  CHECK(p.prepare(metaFor(img)));
  // Open twice.
  CHECK(p.openStream());
  CHECK(!p.openStream());
  CHECK_EQ(b.begin_calls, 1);
}

static void test_duplicate_completion_is_refused() {
  g_case = "duplicate_completion_is_refused";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);
  CHECK(runHappyPath(p, b, img));
  CHECK_EQ(b.set_boot_calls, 1);

  // A replayed completion must not write otadata a second time.
  CHECK(!p.finishStream());
  CHECK(!p.commitBootTarget());
  CHECK_EQ(b.set_boot_calls, 1);
  CHECK_EQ(b.end_calls, 1);
  CHECK_EQ((int)p.state(), (int)OtaState::BOOT_TARGET_SET);
  CHECK_EQ(p.status().counters.updates_committed, 1u);

  // And a replayed prepare cannot silently replace a pending boot switch.
  CHECK(!p.prepare(metaFor(img)));
  CHECK_EQ((int)p.state(), (int)OtaState::BOOT_TARGET_SET);
  CHECK_FAULT(p, OtaFault::WRONG_STATE);
}

// ---------------------------------------------------------------------------
// Abort / reset
// ---------------------------------------------------------------------------

static void test_abort_is_idempotent() {
  g_case = "abort_is_idempotent";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);

  p.abort();  // from IDLE: must do nothing at all
  CHECK_EQ(b.abort_calls, 0);
  CHECK_EQ((int)p.state(), (int)OtaState::IDLE);

  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(streamAll(p, std::vector<uint8_t>(img.begin(), img.begin() + 2048), 512));

  p.abort();
  CHECK_EQ((int)p.state(), (int)OtaState::IDLE);
  CHECK_EQ(b.abort_calls, 1);
  CHECK_EQ(b.set_boot_calls, 0);
  for (int i = 0; i < 5; ++i) p.abort();
  CHECK_EQ(b.abort_calls, 1);   // idempotent: the backend is not re-poked
  CHECK_EQ((int)p.state(), (int)OtaState::IDLE);

  // A fresh update works after an abort.
  CHECK(runHappyPath(p, b, img));
}

static void test_abort_after_commit_does_not_undo_the_boot_switch() {
  g_case = "abort_after_commit_does_not_undo_the_boot_switch";
  FakeBackend b; OtaPolicy p; wire(p, b);
  CHECK(runHappyPath(p, b, makeImage(4096)));
  p.abort();
  // Undoing would be a SECOND boot switch dressed as a cancellation, and the
  // new image is already validated. The pending switch stays reported.
  CHECK_EQ((int)p.state(), (int)OtaState::BOOT_TARGET_SET);
  CHECK_EQ(b.set_boot_calls, 1);
  CHECK(p.status().boot_target_changed);
}

static void test_software_reset_of_the_state_machine() {
  g_case = "software_reset_of_the_state_machine";
  FakeBackend b; OtaPolicy p; wire(p, b);
  const auto img = makeImage(4096);

  // From FAILED.
  OtaImageMetadata bad = metaFor(img); bad.image_size = 0;
  CHECK(!p.prepare(bad));
  CHECK_EQ((int)p.state(), (int)OtaState::FAILED);
  CHECK(p.reset());
  CHECK_EQ((int)p.state(), (int)OtaState::IDLE);
  CHECK_FAULT(p, OtaFault::NONE);
  CHECK_EQ(p.status().bytes_written, 0u);
  CHECK_EQ(p.status().declared_size, 0u);
  CHECK_STR(p.status().declared_sha256, "");

  // Mid-stream.
  CHECK(p.prepare(metaFor(img)));
  CHECK(p.openStream());
  CHECK(streamAll(p, std::vector<uint8_t>(img.begin(), img.begin() + 1024), 512));
  CHECK(p.reset());
  CHECK_EQ((int)p.state(), (int)OtaState::IDLE);

  // But NOT after a commit: the pending boot switch is a real fact.
  CHECK(runHappyPath(p, b, img));
  CHECK(!p.reset());
  CHECK_EQ((int)p.state(), (int)OtaState::BOOT_TARGET_SET);
}

// ---------------------------------------------------------------------------
// OTA-B authorization boundary
// ---------------------------------------------------------------------------

class RefusingGate : public OtaAuthorizationGate {
 public:
  OtaGateVerdict otaPermitted() const override {
    return OtaGateVerdict::REFUSED_BY_AUTHORITY;
  }
};

static void test_authorization_gate() {
  g_case = "authorization_gate_fails_closed_with_no_gate";
  { FakeBackend b; OtaPolicy p; p.begin(&b, nullptr);
    CHECK(!p.prepare(metaFor(makeImage(512))));
    CHECK_FAULT(p, OtaFault::NOT_AUTHORIZED);
    CHECK_EQ((int)p.status().last_gate_verdict,
             (int)OtaGateVerdict::REFUSED_NO_GATE_INSTALLED);
    CHECK_EQ(b.begin_calls, 0);
    CHECK_EQ(b.set_boot_calls, 0); }

  g_case = "authorization_gate_refusal_is_honoured";
  { FakeBackend b; RefusingGate gate; OtaPolicy p; p.begin(&b, &gate);
    CHECK(!p.prepare(metaFor(makeImage(512))));
    CHECK_FAULT(p, OtaFault::NOT_AUTHORIZED);
    CHECK_EQ((int)p.status().last_gate_verdict, (int)OtaGateVerdict::REFUSED_BY_AUTHORITY);
    CHECK_EQ(b.begin_calls, 0); }

  g_case = "ota_a_gate_says_why_it_permits";
  { OtaStageAGate gate;
    CHECK_EQ((int)gate.otaPermitted(),
             (int)OtaGateVerdict::PERMITTED_OTA_A_NO_AUTHORITY_MODEL_YET);
    CHECK(isPermitted(gate.otaPermitted()));
    CHECK(!isPermitted(OtaGateVerdict::REFUSED_NO_GATE_INSTALLED));
    CHECK(!isPermitted(OtaGateVerdict::REFUSED_BY_AUTHORITY)); }
}

// ---------------------------------------------------------------------------
// First-boot rollback lifecycle
// ---------------------------------------------------------------------------

static OtaSelfCheckConfig guardConfig() { return OtaSelfCheckConfig{15000, 2000}; }

static OtaSelfCheckInputs goodInputs(uint32_t uptime_ms) {
  OtaSelfCheckInputs in{};
  in.controller_initialized = true;
  in.command_router_bound = true;
  in.identity_readable = true;
  in.fatal_reset_reason = false;
  in.uptime_ms = uptime_ms;
  return in;
}

static void test_boot_guard_confirmed_image_does_nothing() {
  g_case = "boot_guard_confirmed_image_does_nothing";
  FakeBackend b; b.running_state = OtaImgState::VALID;
  OtaBootGuard g; g.begin(&b, guardConfig());
  for (int i = 0; i < 5000; ++i) g.update(goodInputs(60000));
  CHECK_EQ((int)g.state(), (int)OtaBootState::CONFIRMED);
  CHECK_EQ(b.mark_valid_calls, 0);   // nothing to confirm
  CHECK(!g.rollbackStillArmed());
  CHECK(g.settled());
}

static void test_boot_guard_not_ota_managed() {
  g_case = "boot_guard_not_ota_managed";
  FakeBackend b; b.running_state = OtaImgState::UNDEFINED;
  OtaBootGuard g; g.begin(&b, guardConfig());
  g.update(goodInputs(60000));
  CHECK_EQ((int)g.state(), (int)OtaBootState::NOT_OTA_MANAGED);
  CHECK_EQ(b.mark_valid_calls, 0);
  CHECK(!g.rollbackStillArmed());
}

static void test_boot_guard_valid_first_boot() {
  g_case = "boot_guard_valid_first_boot";
  FakeBackend b; b.running_state = OtaImgState::PENDING_VERIFY;
  OtaBootGuard g; g.begin(&b, guardConfig());

  // Not confirmed at line 1 of startup, nor anywhere near it.
  g.update(goodInputs(0));
  CHECK_EQ((int)g.state(), (int)OtaBootState::PENDING_SELF_CHECK);
  CHECK(g.rollbackStillArmed());
  CHECK_EQ(b.mark_valid_calls, 0);

  // Uptime satisfied but not enough loop ticks: a controller wedged in one
  // long call must not be able to confirm itself.
  for (int i = 0; i < 100; ++i) g.update(goodInputs(60000));
  CHECK_EQ((int)g.state(), (int)OtaBootState::PENDING_SELF_CHECK);
  CHECK_EQ((int)g.fault(), (int)OtaSelfCheckFault::WAITING_LOOP_TICKS);
  CHECK_EQ(b.mark_valid_calls, 0);

  // Ticks satisfied but not uptime.
  { FakeBackend b2; b2.running_state = OtaImgState::PENDING_VERIFY;
    OtaBootGuard g2; g2.begin(&b2, guardConfig());
    for (int i = 0; i < 3000; ++i) g2.update(goodInputs(5000));
    CHECK_EQ((int)g2.state(), (int)OtaBootState::PENDING_SELF_CHECK);
    CHECK_EQ((int)g2.fault(), (int)OtaSelfCheckFault::WAITING_STABLE_UPTIME);
    CHECK_EQ(b2.mark_valid_calls, 0); }

  // Both satisfied: confirmed exactly once.
  for (int i = 0; i < 3000; ++i) g.update(goodInputs(60000));
  CHECK_EQ((int)g.state(), (int)OtaBootState::SELF_CHECK_PASSED);
  CHECK_EQ(b.mark_valid_calls, 1);
  CHECK(!g.rollbackStillArmed());
  CHECK_EQ(g.confirmedAtMs(), 60000u);

  // Settled: further ticks do nothing.
  for (int i = 0; i < 1000; ++i) g.update(goodInputs(90000));
  CHECK_EQ(b.mark_valid_calls, 1);
}

static void test_boot_guard_new_state_is_treated_as_pending() {
  g_case = "boot_guard_new_state_is_treated_as_pending";
  // ESP_OTA_IMG_NEW is what set_boot_partition writes; the bootloader turns
  // it into PENDING_VERIFY. Seeing it from the app still means unconfirmed.
  FakeBackend b; b.running_state = OtaImgState::NEW;
  OtaBootGuard g; g.begin(&b, guardConfig());
  g.update(goodInputs(0));
  CHECK_EQ((int)g.state(), (int)OtaBootState::PENDING_SELF_CHECK);
  CHECK(g.rollbackStillArmed());
}

static void test_boot_guard_failed_first_boot() {
  g_case = "boot_guard_failed_first_boot_fatal_reset";
  FakeBackend b; b.running_state = OtaImgState::PENDING_VERIFY;
  OtaBootGuard g; g.begin(&b, guardConfig());

  OtaSelfCheckInputs in = goodInputs(60000);
  in.fatal_reset_reason = true;      // this boot followed a panic / watchdog
  for (int i = 0; i < 3000; ++i) g.update(in);

  CHECK_EQ((int)g.state(), (int)OtaBootState::SELF_CHECK_REFUSED);
  CHECK_EQ((int)g.fault(), (int)OtaSelfCheckFault::FATAL_RESET_REASON);
  // The refusal is the whole mechanism: we never confirm, so the bootloader
  // marks the image ABORTED on the next boot and falls back on its own. We
  // do NOT trigger a reboot ourselves.
  CHECK_EQ(b.mark_valid_calls, 0);
  CHECK(g.settled());
}

static void test_boot_guard_identity_unreadable_refuses() {
  g_case = "boot_guard_identity_unreadable_refuses";
  FakeBackend b; b.running_state = OtaImgState::PENDING_VERIFY;
  OtaBootGuard g; g.begin(&b, guardConfig());
  OtaSelfCheckInputs in = goodInputs(60000);
  in.identity_readable = false;
  for (int i = 0; i < 3000; ++i) g.update(in);
  CHECK_EQ((int)g.state(), (int)OtaBootState::SELF_CHECK_REFUSED);
  CHECK_EQ((int)g.fault(), (int)OtaSelfCheckFault::IDENTITY_UNREADABLE);
  CHECK_EQ(b.mark_valid_calls, 0);
}

static void test_boot_guard_mark_valid_failure_is_reported() {
  g_case = "boot_guard_mark_valid_failure_is_reported";
  FakeBackend b; b.running_state = OtaImgState::PENDING_VERIFY; b.fail_mark_valid = true;
  OtaBootGuard g; g.begin(&b, guardConfig());
  for (int i = 0; i < 3000; ++i) g.update(goodInputs(60000));
  CHECK_EQ((int)g.state(), (int)OtaBootState::MARK_VALID_FAILED);
  CHECK_EQ(b.mark_valid_calls, 1);   // tried once, did not spin
  CHECK(g.settled());
}

static void test_boot_guard_previously_invalidated() {
  g_case = "boot_guard_previously_invalidated";
  for (OtaImgState s : {OtaImgState::INVALID, OtaImgState::ABORTED}) {
    FakeBackend b; b.running_state = s;
    OtaBootGuard g; g.begin(&b, guardConfig());
    g.update(goodInputs(60000));
    CHECK_EQ((int)g.state(), (int)OtaBootState::PREVIOUSLY_INVALIDATED);
    CHECK_EQ(b.mark_valid_calls, 0);
  }
}

static void test_self_check_verdict_is_pure_and_total() {
  g_case = "self_check_verdict_is_pure_and_total";
  const OtaSelfCheckConfig cfg = guardConfig();
  OtaSelfCheckFault f = OtaSelfCheckFault::NONE;

  OtaSelfCheckInputs in{};
  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 0, &f), (int)OtaSelfCheckVerdict::PENDING);
  CHECK_EQ((int)f, (int)OtaSelfCheckFault::WAITING_CONTROLLER_INIT);

  in.controller_initialized = true;
  in.identity_readable = true;
  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 0, &f), (int)OtaSelfCheckVerdict::PENDING);
  CHECK_EQ((int)f, (int)OtaSelfCheckFault::WAITING_COMMAND_ROUTER);

  in.command_router_bound = true;
  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 0, &f), (int)OtaSelfCheckVerdict::PENDING);
  CHECK_EQ((int)f, (int)OtaSelfCheckFault::WAITING_STABLE_UPTIME);

  in.uptime_ms = 15000;
  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 1999, &f), (int)OtaSelfCheckVerdict::PENDING);
  CHECK_EQ((int)f, (int)OtaSelfCheckFault::WAITING_LOOP_TICKS);

  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 2000, &f), (int)OtaSelfCheckVerdict::PASS);
  CHECK_EQ((int)f, (int)OtaSelfCheckFault::NONE);

  // A fatal reset outranks everything, including a fully satisfied run.
  in.fatal_reset_reason = true;
  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 99999, &f), (int)OtaSelfCheckVerdict::REFUSE);
  CHECK_EQ((int)f, (int)OtaSelfCheckFault::FATAL_RESET_REASON);

  // No output pointer must be survivable.
  in.fatal_reset_reason = false;
  CHECK_EQ((int)evaluateSelfCheck(in, cfg, 2000, nullptr), (int)OtaSelfCheckVerdict::PASS);
}

// ---------------------------------------------------------------------------
// SHA-256 (FIPS 180-4) and hex helpers
// ---------------------------------------------------------------------------

static std::string hashString(const std::string& s, size_t chunk) {
  Sha256 h;
  size_t i = 0;
  while (i < s.size()) {
    const size_t n = std::min(chunk, s.size() - i);
    h.update(reinterpret_cast<const uint8_t*>(s.data()) + i, n);
    i += n;
  }
  uint8_t d[kSha256DigestBytes];
  h.finish(d);
  char hex[kSha256HexBytes];
  toHex(d, hex, sizeof(hex));
  return std::string(hex);
}

static void test_sha256_official_vectors() {
  g_case = "sha256_official_vectors";
  CHECK(hashString("", 64) ==
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  CHECK(hashString("abc", 64) ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  CHECK(hashString("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 64) ==
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  CHECK(hashString("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmno"
                   "ijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu", 64) ==
        "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
  CHECK(hashString(std::string(1000000, 'a'), 64) ==
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

static void test_sha256_is_chunking_invariant() {
  g_case = "sha256_is_chunking_invariant";
  // OTA chunks are transport-sized and have no relationship to the 64-byte
  // block boundary, so this is the property the streaming path depends on.
  const std::string s(200000, 'x');
  const std::string want = hashString(s, 65536);
  for (size_t c : {size_t(1), size_t(7), size_t(63), size_t(64), size_t(65),
                   size_t(127), size_t(1000), size_t(4096)}) {
    CHECK(hashString(s, c) == want);
  }
}

static void test_sha256_helpers() {
  g_case = "sha256_helpers";
  uint8_t d[kSha256DigestBytes];
  const char* want = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
  CHECK(parseHex(want, 64, d));
  char hex[kSha256HexBytes];
  toHex(d, hex, sizeof(hex));
  CHECK_STR(hex, want);

  // Uppercase is accepted; malformed input is refused without writing.
  uint8_t d2[kSha256DigestBytes];
  std::memset(d2, 0xAA, sizeof(d2));
  CHECK(parseHex("E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855",
                 64, d2));
  CHECK(digestsEqual(d, d2));

  uint8_t untouched[kSha256DigestBytes];
  std::memset(untouched, 0x7F, sizeof(untouched));
  CHECK(!parseHex("zz", 2, untouched));
  CHECK(!parseHex(want, 63, untouched));
  CHECK(!parseHex("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b8zz",
                  64, untouched));
  CHECK(!parseHex(nullptr, 64, untouched));
  for (size_t i = 0; i < sizeof(untouched); ++i) CHECK_EQ(untouched[i], 0x7F);

  CHECK(!digestsEqual(nullptr, d));
  uint8_t off_by_one[kSha256DigestBytes];
  std::memcpy(off_by_one, d, sizeof(off_by_one));
  off_by_one[31] ^= 0x01;
  CHECK(!digestsEqual(d, off_by_one));

  // toHex must not write past a short buffer.
  char small[8];
  std::memset(small, 'Z', sizeof(small));
  toHex(d, small, sizeof(small));
  CHECK_EQ(small[0], '\0');
}

// ---------------------------------------------------------------------------
// Presentation
// ---------------------------------------------------------------------------

static void test_tostring_is_total() {
  g_case = "tostring_is_total";
  const OtaState states[] = {OtaState::IDLE, OtaState::TARGET_RESOLVED, OtaState::RECEIVING,
                             OtaState::IMAGE_SEALED, OtaState::IDENTITY_VERIFIED,
                             OtaState::BOOT_TARGET_SET, OtaState::FAILED};
  for (OtaState s : states) CHECK(std::strcmp(toString(s), "UNKNOWN") != 0);

  for (uint8_t i = 0; i <= (uint8_t)OtaFault::BOOT_SWITCH_REJECTED; ++i) {
    CHECK(std::strcmp(toString((OtaFault)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)OtaImgState::UNREADABLE; ++i) {
    CHECK(std::strcmp(toString((OtaImgState)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)OtaBootState::PREVIOUSLY_INVALIDATED; ++i) {
    CHECK(std::strcmp(toString((OtaBootState)i), "UNKNOWN") != 0);
  }
  for (uint8_t i = 0; i <= (uint8_t)OtaSelfCheckFault::FATAL_RESET_REASON; ++i) {
    CHECK(std::strcmp(toString((OtaSelfCheckFault)i), "UNKNOWN") != 0);
  }
  CHECK_STR(toString(OtaFault::TARGET_IS_RUNNING), "TARGET_IS_RUNNING");
  CHECK_STR(toString(OtaBootState::PENDING_SELF_CHECK), "PENDING_SELF_CHECK");
}

static void test_partition_predicates() {
  g_case = "partition_predicates";
  CHECK(app0().isOtaAppSlot());
  CHECK(app1().isOtaAppSlot());
  CHECK(makePartition("ota15", 0, 1, 0x1F).isOtaAppSlot());
  CHECK(!makePartition("factory", 0, 1, 0x00).isOtaAppSlot());
  CHECK(!makePartition("test", 0, 1, 0x20).isOtaAppSlot());
  CHECK(!makePartition("justbelow", 0, 1, 0x0F).isOtaAppSlot());
  CHECK(!OtaPartitionInfo{}.isOtaAppSlot());

  CHECK(app0().sameAs(app0()));
  CHECK(!app0().sameAs(app1()));
  // An invalid partition is never "the same as" anything, including itself.
  CHECK(!OtaPartitionInfo{}.sameAs(OtaPartitionInfo{}));
}

int main() {
  std::printf("MATDOG OTA-A offline tests (policy, boot guard, sha256)\n");

  test_happy_path();
  test_target_is_always_the_inactive_slot();
  test_target_equals_running_partition();
  test_target_is_not_an_ota_slot();
  test_no_inactive_slot_available();
  test_image_larger_than_partition();
  test_zero_length_image();
  test_metadata_incomplete();
  test_update_refused_while_running_image_unconfirmed();
  test_write_open_rejected();
  test_write_error_mid_stream();
  test_partial_write();
  test_interrupted_stream_never_finishes();
  test_overrun_is_rejected_before_it_is_written();
  test_image_rejected_by_backend();
  test_hash_mismatch();
  test_boot_switch_rejected();
  test_commit_is_unreachable_from_every_other_state();
  test_out_of_order_calls_are_refused();
  test_duplicate_completion_is_refused();
  test_abort_is_idempotent();
  test_abort_after_commit_does_not_undo_the_boot_switch();
  test_software_reset_of_the_state_machine();
  test_authorization_gate();
  test_boot_guard_confirmed_image_does_nothing();
  test_boot_guard_not_ota_managed();
  test_boot_guard_valid_first_boot();
  test_boot_guard_new_state_is_treated_as_pending();
  test_boot_guard_failed_first_boot();
  test_boot_guard_identity_unreadable_refuses();
  test_boot_guard_mark_valid_failure_is_reported();
  test_boot_guard_previously_invalidated();
  test_self_check_verdict_is_pure_and_total();
  test_sha256_official_vectors();
  test_sha256_is_chunking_invariant();
  test_sha256_helpers();
  test_tostring_is_total();
  test_partition_predicates();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("OTA_POLICY_TESTS = FAIL\n");
    return 1;
  }
  std::printf("OTA_POLICY_TESTS = PASS\n");
  return 0;
}
