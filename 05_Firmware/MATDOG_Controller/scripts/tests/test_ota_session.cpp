// Offline adversarial tests for the OTA authentication/session layer
// (src/update/OtaSession.*) — I7, per the operator's 2026-09-25 correction.
//
// Links the REAL session logic and the REAL Hmac256/Sha256 primitives — a
// mocked HMAC here would test the mock's idea of authentication, not the
// shipped one. This class has no reference to OtaManager and cannot write
// flash; that boundary is proven by inspection (no #include of OtaManager.h
// here or in OtaSession.h) rather than by a test, since a pure decision
// core cannot be made to call something it has no handle to.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/update/OtaSession.h"

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

const uint8_t kSecret[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

OtaImageMetadata sampleMetadata() {
  OtaImageMetadata m{};
  m.schema_version = 1;
  m.image_size = 384000;
  for (size_t i = 0; i < kSha256DigestBytes; ++i) m.sha256[i] = static_cast<uint8_t>(i);
  std::strncpy(m.build_id, "abc123def456", sizeof(m.build_id) - 1);
  return m;
}

uint8_t nonceByte(uint8_t seed) {
  return seed;
}

void fillNonce(uint8_t out[kOtaNonceBytes], uint8_t seed) {
  for (size_t i = 0; i < kOtaNonceBytes; ++i) out[i] = nonceByte(static_cast<uint8_t>(seed + i));
}

// Signs exactly what an honest client would: the nonce the server issued,
// plus the metadata it intends to send, under the given secret.
void signFor(const uint8_t* secret, size_t secret_len, const uint8_t nonce[kOtaNonceBytes],
            const OtaImageMetadata& metadata, uint8_t out_sig[kHmac256DigestBytes]) {
  uint8_t payload[kOtaSignedPayloadBytes];
  buildOtaSignedPayload(nonce, metadata, payload);
  hmac256(secret, secret_len, payload, sizeof(payload), out_sig);
}

// ---------------------------------------------------------------------------
// No secret configured — fail closed
// ---------------------------------------------------------------------------

void test_no_secret_configured_rejects_every_request() {
  g_case = "no secret configured";
  OtaSession session;
  session.begin(OtaSessionConfig{}, nullptr, 0);

  uint8_t random[kOtaNonceBytes];
  fillNonce(random, 1);
  uint8_t nonce[kOtaNonceBytes];
  session.issueChallenge(1000, random, nonce);

  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig[kHmac256DigestBytes];
  // Even a "correct" signature computed with an empty key must be refused —
  // REJECTED_NO_SECRET is checked before the signature is even examined.
  signFor(nullptr, 0, nonce, metadata, sig);

  CHECK_EQ((int)session.authenticate(1000, nonce, metadata, sig),
          (int)OtaAuthResult::REJECTED_NO_SECRET);
  CHECK(!session.sessionAuthenticated());
}

// ---------------------------------------------------------------------------
// No outstanding challenge
// ---------------------------------------------------------------------------

void test_authenticate_without_a_challenge_is_refused() {
  g_case = "no outstanding challenge";
  OtaSession session;
  session.begin(OtaSessionConfig{}, kSecret, sizeof(kSecret));

  uint8_t nonce[kOtaNonceBytes];
  fillNonce(nonce, 2);
  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce, metadata, sig);

  CHECK_EQ((int)session.authenticate(1000, nonce, metadata, sig),
          (int)OtaAuthResult::REJECTED_NO_CHALLENGE);
}

// ---------------------------------------------------------------------------
// Nonce mismatch, and the outstanding challenge is still consumed
// ---------------------------------------------------------------------------

void test_wrong_nonce_is_rejected_without_burning_the_real_challenge() {
  g_case = "nonce mismatch";
  OtaSession session;
  session.begin(OtaSessionConfig{}, kSecret, sizeof(kSecret));

  uint8_t random[kOtaNonceBytes];
  fillNonce(random, 3);
  uint8_t issued_nonce[kOtaNonceBytes];
  session.issueChallenge(1000, random, issued_nonce);

  uint8_t wrong_nonce[kOtaNonceBytes];
  fillNonce(wrong_nonce, 99);  // deliberately different
  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), wrong_nonce, metadata, sig);

  CHECK_EQ((int)session.authenticate(1000, wrong_nonce, metadata, sig),
          (int)OtaAuthResult::REJECTED_NONCE_MISMATCH);

  // The REAL outstanding challenge must survive a garbage attempt that
  // never validly referenced it — a mismatched nonce must not be able to
  // deny the legitimate caller their one use of the challenge they hold.
  uint8_t correct_sig[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), issued_nonce, metadata, correct_sig);
  CHECK_EQ((int)session.authenticate(1000, issued_nonce, metadata, correct_sig),
          (int)OtaAuthResult::OK);
}

// ---------------------------------------------------------------------------
// Expiry
// ---------------------------------------------------------------------------

void test_expired_challenge_is_rejected() {
  g_case = "expired challenge";
  OtaSessionConfig config;
  config.challenge_ttl_ms = 5000;
  OtaSession session;
  session.begin(config, kSecret, sizeof(kSecret));

  uint8_t random[kOtaNonceBytes];
  fillNonce(random, 4);
  uint8_t nonce[kOtaNonceBytes];
  session.issueChallenge(1000, random, nonce);

  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce, metadata, sig);

  // Exactly at the boundary is still valid; one tick past is not.
  CHECK_EQ((int)session.authenticate(1000 + 5000, nonce, metadata, sig),
          (int)OtaAuthResult::OK);

  session.reset();
  session.issueChallenge(1000, random, nonce);
  signFor(kSecret, sizeof(kSecret), nonce, metadata, sig);
  CHECK_EQ((int)session.authenticate(1000 + 5001, nonce, metadata, sig),
          (int)OtaAuthResult::REJECTED_CHALLENGE_EXPIRED);
}

// ---------------------------------------------------------------------------
// Signature verification, and tamper-detection on every field
// ---------------------------------------------------------------------------

void test_correct_signature_is_accepted_exactly_once() {
  g_case = "correct signature accepted once";
  OtaSession session;
  session.begin(OtaSessionConfig{}, kSecret, sizeof(kSecret));

  uint8_t random[kOtaNonceBytes];
  fillNonce(random, 5);
  uint8_t nonce[kOtaNonceBytes];
  session.issueChallenge(1000, random, nonce);
  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce, metadata, sig);

  CHECK_EQ((int)session.authenticate(1000, nonce, metadata, sig), (int)OtaAuthResult::OK);
  CHECK(session.sessionAuthenticated());

  // Replaying the identical (nonce, metadata, signature) a second time must
  // fail — the nonce is gone, proving the capture-and-replay attack this
  // design exists to prevent does not work.
  CHECK_EQ((int)session.authenticate(1000, nonce, metadata, sig),
          (int)OtaAuthResult::REJECTED_NO_CHALLENGE);
}

void test_wrong_secret_is_rejected() {
  g_case = "wrong secret";
  OtaSession session;
  session.begin(OtaSessionConfig{}, kSecret, sizeof(kSecret));

  uint8_t random[kOtaNonceBytes];
  fillNonce(random, 6);
  uint8_t nonce[kOtaNonceBytes];
  session.issueChallenge(1000, random, nonce);
  const OtaImageMetadata metadata = sampleMetadata();

  const uint8_t attacker_secret[] = {9, 9, 9, 9};
  uint8_t sig[kHmac256DigestBytes];
  signFor(attacker_secret, sizeof(attacker_secret), nonce, metadata, sig);

  CHECK_EQ((int)session.authenticate(1000, nonce, metadata, sig),
          (int)OtaAuthResult::REJECTED_SIGNATURE);
}

// Every one of these fields is part of what gets committed to flash via
// OtaManager::prepare(metadata) — an attacker who could change any ONE of
// them after signing (a valid signature for a DIFFERENT sha256, say) would
// have found a way to authorise an unintended image. None must be possible.
void test_tampering_with_any_metadata_field_invalidates_the_signature() {
  g_case = "metadata tamper detection";
  struct Case {
    const char* name;
    void (*mutate)(OtaImageMetadata*);
  };
  const Case cases[] = {
      {"schema_version", [](OtaImageMetadata* m) { m->schema_version += 1; }},
      {"image_size", [](OtaImageMetadata* m) { m->image_size += 1; }},
      {"sha256[0]", [](OtaImageMetadata* m) { m->sha256[0] ^= 0xFF; }},
      {"sha256[31]", [](OtaImageMetadata* m) { m->sha256[kSha256DigestBytes - 1] ^= 0xFF; }},
      {"build_id", [](OtaImageMetadata* m) { m->build_id[0] = (m->build_id[0] == 'a') ? 'b' : 'a'; }},
  };

  for (const Case& c : cases) {
    OtaSession session;
    session.begin(OtaSessionConfig{}, kSecret, sizeof(kSecret));
    uint8_t random[kOtaNonceBytes];
    fillNonce(random, 7);
    uint8_t nonce[kOtaNonceBytes];
    session.issueChallenge(1000, random, nonce);

    OtaImageMetadata signed_metadata = sampleMetadata();
    uint8_t sig[kHmac256DigestBytes];
    signFor(kSecret, sizeof(kSecret), nonce, signed_metadata, sig);

    OtaImageMetadata tampered = signed_metadata;
    c.mutate(&tampered);

    g_case = c.name;
    CHECK_EQ((int)session.authenticate(1000, nonce, tampered, sig),
            (int)OtaAuthResult::REJECTED_SIGNATURE);
  }
}

// ---------------------------------------------------------------------------
// issueChallenge() invalidates any previous outstanding nonce
// ---------------------------------------------------------------------------

void test_new_challenge_invalidates_the_previous_one() {
  g_case = "new challenge invalidates old";
  OtaSession session;
  session.begin(OtaSessionConfig{}, kSecret, sizeof(kSecret));

  uint8_t random1[kOtaNonceBytes];
  fillNonce(random1, 8);
  uint8_t nonce1[kOtaNonceBytes];
  session.issueChallenge(1000, random1, nonce1);

  uint8_t random2[kOtaNonceBytes];
  fillNonce(random2, 50);
  uint8_t nonce2[kOtaNonceBytes];
  session.issueChallenge(1001, random2, nonce2);

  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig1[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce1, metadata, sig1);

  // The FIRST nonce is no longer the outstanding one at all.
  CHECK_EQ((int)session.authenticate(1001, nonce1, metadata, sig1),
          (int)OtaAuthResult::REJECTED_NONCE_MISMATCH);

  // The second, current nonce still works.
  uint8_t sig2[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce2, metadata, sig2);
  CHECK_EQ((int)session.authenticate(1001, nonce2, metadata, sig2), (int)OtaAuthResult::OK);
}

// ---------------------------------------------------------------------------
// Liveness / timeout
// ---------------------------------------------------------------------------

void test_timeout_tracks_activity_and_reset_clears_it() {
  g_case = "timeout and activity";
  OtaSessionConfig config;
  config.activity_timeout_ms = 1000;
  OtaSession session;
  session.begin(config, kSecret, sizeof(kSecret));

  CHECK(!session.timedOut(999999));  // no active session — never "timed out"

  uint8_t random[kOtaNonceBytes];
  fillNonce(random, 9);
  uint8_t nonce[kOtaNonceBytes];
  session.issueChallenge(1000, random, nonce);
  const OtaImageMetadata metadata = sampleMetadata();
  uint8_t sig[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce, metadata, sig);
  CHECK_EQ((int)session.authenticate(1000, nonce, metadata, sig), (int)OtaAuthResult::OK);

  CHECK(!session.timedOut(1000 + 999));   // just under the threshold
  CHECK(session.timedOut(1000 + 1001));   // past it

  session.noteActivity(1000 + 999);
  CHECK(!session.timedOut(1000 + 999 + 999));  // the clock was reset by activity
  CHECK(session.timedOut(1000 + 999 + 1001));

  session.reset();
  CHECK(!session.sessionAuthenticated());
  CHECK(!session.timedOut(999999999));  // reset — nothing to time out anymore

  // A fresh challenge still works after reset() — reset() does not disturb
  // challenge/nonce state, only the authenticated-session flag.
  uint8_t random2[kOtaNonceBytes];
  fillNonce(random2, 20);
  uint8_t nonce2[kOtaNonceBytes];
  session.issueChallenge(2000, random2, nonce2);
  uint8_t sig2[kHmac256DigestBytes];
  signFor(kSecret, sizeof(kSecret), nonce2, metadata, sig2);
  CHECK_EQ((int)session.authenticate(2000, nonce2, metadata, sig2), (int)OtaAuthResult::OK);
}

// ---------------------------------------------------------------------------
// buildOtaSignedPayload() determinism — the fix for uninitialised build_id
// tail bytes
// ---------------------------------------------------------------------------

void test_signed_payload_ignores_garbage_past_the_build_id_nul() {
  g_case = "build_id tail determinism";
  OtaImageMetadata clean = sampleMetadata();

  OtaImageMetadata dirty = clean;
  // Poison every byte after the NUL terminator with non-zero garbage, as a
  // caller who filled the struct via strncpy() without zeroing it first
  // realistically could.
  const size_t len = strnlen(dirty.build_id, sizeof(dirty.build_id));
  for (size_t i = len + 1; i < sizeof(dirty.build_id); ++i) {
    dirty.build_id[i] = static_cast<char>(0xAA);
  }

  uint8_t nonce[kOtaNonceBytes];
  fillNonce(nonce, 10);
  uint8_t payload_clean[kOtaSignedPayloadBytes];
  uint8_t payload_dirty[kOtaSignedPayloadBytes];
  buildOtaSignedPayload(nonce, clean, payload_clean);
  buildOtaSignedPayload(nonce, dirty, payload_dirty);

  CHECK(memcmp(payload_clean, payload_dirty, kOtaSignedPayloadBytes) == 0);
}

// ---------------------------------------------------------------------------
// toString()
// ---------------------------------------------------------------------------

void test_to_string_covers_every_value() {
  g_case = "to_string";
  CHECK(strcmp(toString(OtaAuthResult::OK), "OK") == 0);
  CHECK(strcmp(toString(OtaAuthResult::REJECTED_NO_SECRET), "REJECTED_NO_SECRET") == 0);
  CHECK(strcmp(toString(OtaAuthResult::REJECTED_NO_CHALLENGE), "REJECTED_NO_CHALLENGE") == 0);
  CHECK(strcmp(toString(OtaAuthResult::REJECTED_NONCE_MISMATCH), "REJECTED_NONCE_MISMATCH") == 0);
  CHECK(strcmp(toString(OtaAuthResult::REJECTED_CHALLENGE_EXPIRED),
              "REJECTED_CHALLENGE_EXPIRED") == 0);
  CHECK(strcmp(toString(OtaAuthResult::REJECTED_SIGNATURE), "REJECTED_SIGNATURE") == 0);
  CHECK(strcmp(toString(static_cast<OtaAuthResult>(200)), "UNKNOWN") == 0);
}

}  // namespace

int main() {
  test_no_secret_configured_rejects_every_request();
  test_authenticate_without_a_challenge_is_refused();
  test_wrong_nonce_is_rejected_without_burning_the_real_challenge();
  test_expired_challenge_is_rejected();
  test_correct_signature_is_accepted_exactly_once();
  test_wrong_secret_is_rejected();
  test_tampering_with_any_metadata_field_invalidates_the_signature();
  test_new_challenge_invalidates_the_previous_one();
  test_timeout_tracks_activity_and_reset_clears_it();
  test_signed_payload_ignores_garbage_past_the_build_id_nul();
  test_to_string_covers_every_value();

  std::printf("test_ota_session: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
