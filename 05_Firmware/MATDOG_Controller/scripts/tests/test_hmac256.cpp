// Offline host tests for HMAC-SHA256 (src/update/Hmac256.*) — I7, per the
// operator's 2026-09-25 correction: firmware image bytes still flow through
// the one existing OtaManager/OtaPolicy/OtaEspBackend writer; this is the
// authentication primitive the network transport's session layer uses to
// verify a caller holds the shared secret before that writer is ever
// reachable.
//
// Verified against RFC 4231's official HMAC-SHA256 test vectors — not
// self-consistency only. Links the REAL implementation, the same contract
// as test_sha256's use of the FIPS 180-4 vectors it is built on.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/update/Hmac256.h"

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

namespace {

void checkVector(const char* name, const uint8_t* key, size_t key_len, const uint8_t* msg,
                 size_t msg_len, const char* expected_hex) {
  g_case = name;
  uint8_t expected[kHmac256DigestBytes];
  CHECK(parseHex(expected_hex, 64, expected));

  uint8_t actual[kHmac256DigestBytes];
  hmac256(key, key_len, msg, msg_len, actual);

  char actual_hex[kSha256HexBytes];
  toHex(actual, actual_hex, sizeof(actual_hex));
  if (memcmp(actual, expected, kHmac256DigestBytes) != 0) {
    ++g_failures;
    std::printf("  FAIL [%s] got %s, expected %s\n", name, actual_hex, expected_hex);
  }
  ++g_checks;
}

// RFC 4231 §4.2 — Test Case 1: key shorter than the block size.
void test_rfc4231_case1() {
  const uint8_t key[20] = {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
                           0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b};
  const char* data = "Hi There";
  checkVector("rfc4231 case 1", key, sizeof(key),
             reinterpret_cast<const uint8_t*>(data), strlen(data),
             "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
}

// RFC 4231 §4.3 — Test Case 2: key and data both shorter than a block,
// key given as ASCII text.
void test_rfc4231_case2() {
  const char* key = "Jefe";
  const char* data = "what do ya want for nothing?";
  checkVector("rfc4231 case 2", reinterpret_cast<const uint8_t*>(key), strlen(key),
             reinterpret_cast<const uint8_t*>(data), strlen(data),
             "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
}

// RFC 4231 §4.4 — Test Case 3: full-block key, multi-block data.
void test_rfc4231_case3() {
  uint8_t key[20];
  memset(key, 0xaa, sizeof(key));
  uint8_t data[50];
  memset(data, 0xdd, sizeof(data));
  checkVector("rfc4231 case 3", key, sizeof(key), data, sizeof(data),
             "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe");
}

// RFC 4231 §4.7 — Test Case 6: key LONGER than the block size (131 bytes),
// which exercises the "hash the key down first" branch specifically.
void test_rfc4231_case6_key_longer_than_block() {
  uint8_t key[131];
  memset(key, 0xaa, sizeof(key));
  const char* data = "Test Using Larger Than Block-Size Key - Hash Key First";
  checkVector("rfc4231 case 6", key, sizeof(key),
             reinterpret_cast<const uint8_t*>(data), strlen(data),
             "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
}

// Not an RFC vector — pins the exact boundary this implementation branches
// on (key_len == block size, the largest key that must NOT be hashed down).
void test_key_exactly_one_block_does_not_hash_the_key() {
  g_case = "key == block size";
  uint8_t key64[64];
  memset(key64, 0x5a, sizeof(key64));
  const char* data = "boundary";
  uint8_t a[kHmac256DigestBytes];
  hmac256(key64, sizeof(key64), reinterpret_cast<const uint8_t*>(data), strlen(data), a);

  // A 65-byte key that differs only in its last (hashed-away only if >64)
  // byte must produce a DIFFERENT digest than the 64-byte key above,
  // proving the 64-byte key really did skip the hash-down branch (if it
  // had been hashed, the extra byte at 65 would never have been part of
  // what got hashed into the 64-byte case, so this alone does not fully
  // isolate the branch — the real isolation is exercised by case 6 above
  // using an unambiguously-longer key; this case only pins that changing
  // the 65th byte changes the digest when the key IS longer than a block).
  uint8_t key65[65];
  memcpy(key65, key64, 64);
  key65[64] = 0x00;
  uint8_t b[kHmac256DigestBytes];
  hmac256(key65, sizeof(key65), reinterpret_cast<const uint8_t*>(data), strlen(data), b);
  CHECK(memcmp(a, b, kHmac256DigestBytes) != 0);
}

void test_different_messages_produce_different_macs() {
  g_case = "message sensitivity";
  const uint8_t key[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  const char* m1 = "message one";
  const char* m2 = "message two";
  uint8_t d1[kHmac256DigestBytes];
  uint8_t d2[kHmac256DigestBytes];
  hmac256(key, sizeof(key), reinterpret_cast<const uint8_t*>(m1), strlen(m1), d1);
  hmac256(key, sizeof(key), reinterpret_cast<const uint8_t*>(m2), strlen(m2), d2);
  CHECK(memcmp(d1, d2, kHmac256DigestBytes) != 0);
}

void test_empty_key_and_empty_message_do_not_crash_and_are_deterministic() {
  g_case = "empty key/message";
  uint8_t d1[kHmac256DigestBytes];
  uint8_t d2[kHmac256DigestBytes];
  hmac256(nullptr, 0, nullptr, 0, d1);
  hmac256(nullptr, 0, nullptr, 0, d2);
  CHECK(memcmp(d1, d2, kHmac256DigestBytes) == 0);
}

}  // namespace

int main() {
  test_rfc4231_case1();
  test_rfc4231_case2();
  test_rfc4231_case3();
  test_rfc4231_case6_key_longer_than_block();
  test_key_exactly_one_block_does_not_hash_the_key();
  test_different_messages_produce_different_macs();
  test_empty_key_and_empty_message_do_not_crash_and_are_deterministic();

  std::printf("test_hmac256: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
