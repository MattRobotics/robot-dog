#ifndef MATDOG_UPDATE_OTA_SESSION_H
#define MATDOG_UPDATE_OTA_SESSION_H

#include <stddef.h>
#include <stdint.h>

#include "Hmac256.h"
#include "OtaPolicy.h"
#include "Sha256.h"

// The authentication/session layer between a network transport and the one
// existing OtaManager/OtaPolicy/OtaEspBackend writer — I7, per the
// operator's 2026-09-25 correction:
//
//   network transport
//        |
//        v
//   OtaSession            <- THIS: authentication, replay protection, liveness
//        |
//        v
//   OtaManager / OtaPolicy / OtaEspBackend      [UNCHANGED, the one writer]
//
// Deliberately <stddef.h>/<stdint.h> plus the local crypto primitives and
// OtaPolicy.h (for OtaImageMetadata, itself already pure) — no
// <Arduino.h>, no esp_http_server, no OtaManager. Pure and host-linkable,
// the same contract as every other decision core in this codebase:
// scripts/tests/test_ota_session.cpp links the REAL verification logic,
// not a copy.
//
// WHAT THIS DOES NOT DO
// ----------------------
// It does not write flash, does not call OtaManager, does not know about
// esp_http_server or Wi-Fi. It answers exactly one question per call:
// "does this request prove the caller holds the shared secret, against a
// nonce that has not already been used or expired?" The caller (the HTTP
// transport adapter) is the one that, on a successful authenticate(), goes
// on to call OtaManager::prepare(metadata)/writeChunk()/... with the SAME
// metadata struct that was authenticated — this class has no reference to
// OtaManager and cannot call any of its five ingest entry points itself.
//
// REPLAY PROTECTION, AND WHY A NONCE INSTEAD OF A TIMESTAMP
// -----------------------------------------------------------
// A pre-shared HMAC key alone authenticates a message but does not prevent
// an attacker who observed one valid (metadata, signature) pair from
// replaying it verbatim later. A monotonic timestamp would need clock sync
// this device does not have. A single-use, server-issued random nonce needs
// neither: issueChallenge() hands out one nonce with a bounded lifetime;
// authenticate() consumes it as soon as the PRESENTED nonce matches the
// outstanding one, whether the attempt then succeeds or fails on expiry or
// signature — so a captured (metadata, signature) pair can never be
// replayed, and repeated wrong-signature guesses against one nonce cannot
// be retried either. A presented nonce that does NOT match the outstanding
// one consumes nothing: it never validly referenced the real challenge, so
// it must not be able to deny the legitimate holder of that challenge
// their one use of it — an availability bug an early version of this class
// had, caught by its own adversarial test suite before being trusted.
//
// SIGNED PAYLOAD FORMAT — fixed-width, on purpose
// --------------------------------------------------
// nonce[16] || schema_version(BE32) || image_size(BE32) || sha256[32] ||
// build_id[kOtaBuildIdBytes], exactly 16+4+4+32+kOtaBuildIdBytes bytes,
// every field fixed-width and zero-padded where short. No field is
// length-prefixed or delimiter-separated: with fixed widths there is no
// encoding under which two different (nonce, metadata) pairs serialize to
// the same bytes, which is what makes the signature unambiguous — a
// classic source of real HMAC verification bugs is exactly this kind of
// framing ambiguity, and fixed-width fields close it by construction.

namespace matdog {
namespace update {

constexpr size_t kOtaNonceBytes = 16;
constexpr size_t kOtaSignedPayloadBytes =
    kOtaNonceBytes + 4 + 4 + kSha256DigestBytes + kOtaBuildIdBytes;

// Pure. Exposed so the transport adapter and the test suite build the
// identical byte layout the signature actually covers — never
// re-implemented ad hoc at either call site.
void buildOtaSignedPayload(const uint8_t nonce[kOtaNonceBytes], const OtaImageMetadata& metadata,
                          uint8_t out[kOtaSignedPayloadBytes]);

struct OtaSessionConfig {
  uint32_t challenge_ttl_ms = 30000;     // how long an issued nonce stays valid
  uint32_t activity_timeout_ms = 30000;  // max gap between chunks before the
                                         // caller must abort the transfer
};

enum class OtaAuthResult : uint8_t {
  OK                         = 0,
  REJECTED_NO_SECRET         = 1,  // no shared secret configured — fail closed
  REJECTED_NO_CHALLENGE      = 2,  // authenticate() called with no outstanding nonce
  REJECTED_NONCE_MISMATCH    = 3,
  REJECTED_CHALLENGE_EXPIRED = 4,
  REJECTED_SIGNATURE         = 5,  // HMAC did not verify
};

const char* toString(OtaAuthResult result);

class OtaSession {
 public:
  // secret/secret_len may be null/0, which means "no secret configured" —
  // authenticate() then fails closed with REJECTED_NO_SECRET for every
  // request, the same fail-closed shape WifiPolicy uses for absent
  // credentials. The pointer is stored, not copied: the caller (the
  // Arduino-side owner reading a gitignored local credentials header) must
  // outlive this object, exactly like every other pointer-holding module
  // pattern in this codebase.
  void begin(const OtaSessionConfig& config, const uint8_t* secret, size_t secret_len);

  // Issues a fresh nonce into out_nonce. `random_bytes` is caller-supplied
  // entropy (esp_random() on device, a fixed test vector offline) — this
  // class never reaches for a hardware RNG itself. Always succeeds:
  // issuing a new challenge invalidates any previous outstanding one, so
  // there is never more than one live nonce, and a caller who abandons a
  // challenge cannot lock out future ones.
  void issueChallenge(uint32_t now_ms, const uint8_t random_bytes[kOtaNonceBytes],
                      uint8_t out_nonce[kOtaNonceBytes]);

  // Verifies an authenticated "begin update" request against the exact
  // metadata the caller intends to pass to OtaManager::prepare(). On OK,
  // the nonce is consumed (single-use) and the session's activity clock
  // starts; the caller is now expected to call OtaManager::prepare(metadata)
  // and go on to noteActivity()/timedOut() for every subsequent chunk.
  OtaAuthResult authenticate(uint32_t now_ms, const uint8_t nonce[kOtaNonceBytes],
                             const OtaImageMetadata& metadata,
                             const uint8_t signature[kHmac256DigestBytes]);

  // Resets the liveness clock — call once per chunk received.
  void noteActivity(uint32_t now_ms);

  // True once authenticate() has succeeded and reset() has not been called
  // since (a timeout does not clear this by itself — see timedOut()).
  bool sessionAuthenticated() const { return session_active_; }

  // Pure predicate: has more than activity_timeout_ms passed since the last
  // noteActivity()/authenticate() call while a session is active? The
  // caller is responsible for acting on this (calling OtaManager::abort()
  // and this class's reset()) — this class holds no reference to
  // OtaManager and cannot abort anything itself.
  bool timedOut(uint32_t now_ms) const;

  // Ends the authenticated session (success, failure, or a caller-detected
  // timeout) without touching challenge/nonce state — a fresh
  // issueChallenge() is still required for the next attempt regardless.
  void reset();

 private:
  bool hasSecret() const { return secret_ != nullptr && secret_len_ > 0; }

  OtaSessionConfig config_{};
  const uint8_t* secret_ = nullptr;
  size_t secret_len_ = 0;

  bool challenge_outstanding_ = false;
  uint8_t nonce_[kOtaNonceBytes] = {0};
  uint32_t challenge_issued_ms_ = 0;

  bool session_active_ = false;
  uint32_t last_activity_ms_ = 0;
};

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_SESSION_H
