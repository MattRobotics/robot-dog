#include "Sha256.h"

namespace matdog {
namespace update {
namespace {

// FIPS 180-4 section 4.2.2: first 32 bits of the fractional parts of the
// cube roots of the first 64 primes.
const uint32_t kK[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
    0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
    0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
    0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
    0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
    0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

inline uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32u - n)); }
inline uint32_t ch(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (~x & z); }
inline uint32_t maj(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (x & z) ^ (y & z); }
inline uint32_t bsig0(uint32_t x) { return rotr(x, 2) ^ rotr(x, 13) ^ rotr(x, 22); }
inline uint32_t bsig1(uint32_t x) { return rotr(x, 6) ^ rotr(x, 11) ^ rotr(x, 25); }
inline uint32_t ssig0(uint32_t x) { return rotr(x, 7) ^ rotr(x, 18) ^ (x >> 3); }
inline uint32_t ssig1(uint32_t x) { return rotr(x, 17) ^ rotr(x, 19) ^ (x >> 10); }

}  // namespace

void Sha256::reset() {
  // FIPS 180-4 section 5.3.3.
  h_[0] = 0x6a09e667u; h_[1] = 0xbb67ae85u; h_[2] = 0x3c6ef372u; h_[3] = 0xa54ff53au;
  h_[4] = 0x510e527fu; h_[5] = 0x9b05688cu; h_[6] = 0x1f83d9abu; h_[7] = 0x5be0cd19u;
  block_len_ = 0;
  total_bits_ = 0;
  for (size_t i = 0; i < sizeof(block_); ++i) block_[i] = 0;
}

void Sha256::compress(const uint8_t block[64]) {
  uint32_t w[64];
  for (uint32_t i = 0; i < 16; ++i) {
    w[i] = (static_cast<uint32_t>(block[i * 4 + 0]) << 24) |
           (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
           (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
           (static_cast<uint32_t>(block[i * 4 + 3]));
  }
  for (uint32_t i = 16; i < 64; ++i) {
    w[i] = ssig1(w[i - 2]) + w[i - 7] + ssig0(w[i - 15]) + w[i - 16];
  }

  uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3];
  uint32_t e = h_[4], f = h_[5], g = h_[6], hh = h_[7];

  for (uint32_t i = 0; i < 64; ++i) {
    const uint32_t t1 = hh + bsig1(e) + ch(e, f, g) + kK[i] + w[i];
    const uint32_t t2 = bsig0(a) + maj(a, b, c);
    hh = g; g = f; f = e; e = d + t1;
    d = c; c = b; b = a; a = t1 + t2;
  }

  h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d;
  h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += hh;
}

void Sha256::update(const uint8_t* data, size_t len) {
  if (data == nullptr) return;
  total_bits_ += static_cast<uint64_t>(len) * 8u;

  size_t i = 0;
  // Top up a partially filled block first: OTA chunks are transport-sized
  // and have no relationship to the 64-byte block boundary.
  if (block_len_ > 0) {
    while (i < len && block_len_ < 64) block_[block_len_++] = data[i++];
    if (block_len_ == 64) {
      compress(block_);
      block_len_ = 0;
    }
  }
  while (len - i >= 64) {
    compress(data + i);
    i += 64;
  }
  while (i < len) block_[block_len_++] = data[i++];
}

void Sha256::finish(uint8_t* out) {
  if (out == nullptr) return;
  const uint64_t bits = total_bits_;

  // Padding: 0x80, then zeros, then the 64-bit big-endian bit count.
  block_[block_len_++] = 0x80u;
  if (block_len_ > 56) {
    while (block_len_ < 64) block_[block_len_++] = 0u;
    compress(block_);
    block_len_ = 0;
  }
  while (block_len_ < 56) block_[block_len_++] = 0u;
  for (int i = 7; i >= 0; --i) {
    block_[block_len_++] = static_cast<uint8_t>((bits >> (i * 8)) & 0xFFu);
  }
  compress(block_);

  for (uint32_t i = 0; i < 8; ++i) {
    out[i * 4 + 0] = static_cast<uint8_t>((h_[i] >> 24) & 0xFFu);
    out[i * 4 + 1] = static_cast<uint8_t>((h_[i] >> 16) & 0xFFu);
    out[i * 4 + 2] = static_cast<uint8_t>((h_[i] >> 8) & 0xFFu);
    out[i * 4 + 3] = static_cast<uint8_t>(h_[i] & 0xFFu);
  }
}

void toHex(const uint8_t* digest, char* out, size_t out_size) {
  if (out == nullptr || out_size == 0) return;
  static const char kHex[] = "0123456789abcdef";
  if (digest == nullptr || out_size < kSha256HexBytes) {
    out[0] = '\0';
    return;
  }
  for (size_t i = 0; i < kSha256DigestBytes; ++i) {
    out[i * 2 + 0] = kHex[(digest[i] >> 4) & 0x0Fu];
    out[i * 2 + 1] = kHex[digest[i] & 0x0Fu];
  }
  out[kSha256DigestBytes * 2] = '\0';
}

bool parseHex(const char* hex, size_t hex_len, uint8_t* out) {
  if (hex == nullptr || out == nullptr) return false;
  if (hex_len != kSha256DigestBytes * 2) return false;

  uint8_t staged[kSha256DigestBytes];
  for (size_t i = 0; i < kSha256DigestBytes; ++i) {
    uint8_t byte = 0;
    for (size_t nibble = 0; nibble < 2; ++nibble) {
      const char c = hex[i * 2 + nibble];
      uint8_t v;
      if (c >= '0' && c <= '9')      v = static_cast<uint8_t>(c - '0');
      else if (c >= 'a' && c <= 'f') v = static_cast<uint8_t>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F') v = static_cast<uint8_t>(c - 'A' + 10);
      else return false;  // fail closed: nothing is written on malformed input
      byte = static_cast<uint8_t>((byte << 4) | v);
    }
    staged[i] = byte;
  }
  // Staged first so a malformed tail cannot leave a half-written digest.
  for (size_t i = 0; i < kSha256DigestBytes; ++i) out[i] = staged[i];
  return true;
}

bool digestsEqual(const uint8_t* a, const uint8_t* b) {
  if (a == nullptr || b == nullptr) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < kSha256DigestBytes; ++i) {
    diff = static_cast<uint8_t>(diff | (a[i] ^ b[i]));
  }
  return diff == 0;
}

}  // namespace update
}  // namespace matdog
