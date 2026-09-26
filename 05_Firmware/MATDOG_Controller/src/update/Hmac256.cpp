#include "Hmac256.h"

#include <string.h>

namespace matdog {
namespace update {

namespace {

// RFC 2104 step 1: keys longer than the block size are hashed down; shorter
// keys are zero-padded up. Either way the result is exactly one block.
void keyBlock(const uint8_t* key, size_t key_len, uint8_t out_block[kHmac256BlockBytes]) {
  if (key_len > kHmac256BlockBytes) {
    Sha256 hasher;
    hasher.update(key, key_len);
    uint8_t digest[kSha256DigestBytes];
    hasher.finish(digest);
    memcpy(out_block, digest, kSha256DigestBytes);
    memset(out_block + kSha256DigestBytes, 0, kHmac256BlockBytes - kSha256DigestBytes);
  } else {
    if (key_len > 0) memcpy(out_block, key, key_len);
    memset(out_block + key_len, 0, kHmac256BlockBytes - key_len);
  }
}

}  // namespace

void hmac256(const uint8_t* key, size_t key_len, const uint8_t* message, size_t message_len,
            uint8_t out[kHmac256DigestBytes]) {
  uint8_t k0[kHmac256BlockBytes];
  keyBlock(key, key_len, k0);

  uint8_t ipad[kHmac256BlockBytes];
  uint8_t opad[kHmac256BlockBytes];
  for (size_t i = 0; i < kHmac256BlockBytes; ++i) {
    ipad[i] = static_cast<uint8_t>(k0[i] ^ 0x36);
    opad[i] = static_cast<uint8_t>(k0[i] ^ 0x5c);
  }

  Sha256 inner;
  inner.update(ipad, kHmac256BlockBytes);
  if (message_len > 0) inner.update(message, message_len);
  uint8_t inner_digest[kSha256DigestBytes];
  inner.finish(inner_digest);

  Sha256 outer;
  outer.update(opad, kHmac256BlockBytes);
  outer.update(inner_digest, kSha256DigestBytes);
  outer.finish(out);
}

}  // namespace update
}  // namespace matdog
