#ifndef MATDOG_UPDATE_SHA256_H
#define MATDOG_UPDATE_SHA256_H

#include <stddef.h>
#include <stdint.h>

// Streaming SHA-256 (FIPS 180-4), deliberately implemented here rather than
// called through mbedtls.
//
// mbedtls IS linked into the image (the Wi-Fi stack pulls it in), so this is
// not about availability. It is about which layer owns the hash. OTA image
// identity is decided in update/OtaPolicy.*, which is Arduino-free and
// ESP-IDF-free precisely so scripts/tests/test_ota_policy.cpp can link the
// REAL decision logic. A hash reached through an ESP-IDF handle could not be
// used there, so the host tests would be verifying a different hash than the
// device computes — the exact drift the project's host-linkable rule exists
// to prevent.
//
// Same precedent as the CRC-16/MODBUS in power/DalyProtocol.*: a specified,
// test-vector-checkable algorithm is implemented locally and pinned by the
// offline suite, instead of being reached through a transport-bound library.
//
// Allocation-free and constant-space: one 64-byte block buffer. Safe to feed
// in arbitrary chunk sizes — the OTA stream arrives in transport-sized
// pieces that have nothing to do with the 64-byte block boundary.

namespace matdog {
namespace update {

constexpr size_t kSha256DigestBytes = 32;
// 64 hex characters + NUL.
constexpr size_t kSha256HexBytes = 2 * kSha256DigestBytes + 1;

class Sha256 {
 public:
  Sha256() { reset(); }

  void reset();
  void update(const uint8_t* data, size_t len);
  // Writes kSha256DigestBytes into `out`. The object must not be updated
  // again afterwards without reset().
  void finish(uint8_t* out);

  uint64_t bytesHashed() const { return total_bits_ / 8; }

 private:
  void compress(const uint8_t block[64]);

  uint32_t h_[8];
  uint8_t block_[64];
  size_t block_len_;
  uint64_t total_bits_;
};

// Lowercase hex, NUL-terminated. `out_size` must be >= kSha256HexBytes.
void toHex(const uint8_t* digest, char* out, size_t out_size);

// Parses 64 lowercase/uppercase hex characters into kSha256DigestBytes.
// Returns false (and leaves `out` untouched) on any non-hex character or
// wrong length — metadata arriving over a transport is never trusted.
bool parseHex(const char* hex, size_t hex_len, uint8_t* out);

// Constant-time-ish comparison. Not a defence against a timing attacker
// (OTA-A has no authentication yet), but it avoids an early-out compare
// becoming the thing that has to be revisited when authentication lands.
bool digestsEqual(const uint8_t* a, const uint8_t* b);

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_SHA256_H
