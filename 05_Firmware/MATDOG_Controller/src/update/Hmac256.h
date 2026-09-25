#ifndef MATDOG_UPDATE_HMAC256_H
#define MATDOG_UPDATE_HMAC256_H

#include <stddef.h>
#include <stdint.h>

#include "Sha256.h"

// HMAC-SHA256 (RFC 2104 / FIPS 198-1), built on the existing reviewed
// Sha256 — no new dependency, no mbedtls call, same host-linkable contract.
//
// WHY LOCAL AND NOT mbedtls_md_hmac(): the same reason Sha256.h gives for
// its own hash — OtaSession.* (the layer that verifies every OTA
// authentication request) must stay Arduino-free and ESP-IDF-free so
// scripts/tests/test_ota_session.cpp links the REAL verification logic, not
// a copy. An HMAC reached through an ESP-IDF handle could not be used
// there, and mbedtls IS available (see Sha256.h) but is deliberately not
// reached for here, for the same layering reason.
//
// Message sizes here are always small and fixed (a nonce plus a few
// metadata fields, never the firmware image itself), so this is a one-shot
// API, not a streaming one — Sha256 remains the streaming primitive for the
// image hash itself.

namespace matdog {
namespace update {

constexpr size_t kHmac256DigestBytes = kSha256DigestBytes;  // 32
constexpr size_t kHmac256BlockBytes = 64;                    // SHA-256 block size

// Computes HMAC-SHA256(key, message) into out[kHmac256DigestBytes]. A key
// longer than the block size is first hashed down, exactly as RFC 2104
// specifies — callers never need to pre-hash a long secret themselves. A
// zero-length key is accepted and computed per spec; whether an empty key
// is an authentication failure is a policy decision made by the caller
// (OtaSession), not by this pure primitive.
void hmac256(const uint8_t* key, size_t key_len, const uint8_t* message, size_t message_len,
            uint8_t out[kHmac256DigestBytes]);

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_HMAC256_H
