#include "OtaSession.h"

#include <string.h>

namespace matdog {
namespace update {

void buildOtaSignedPayload(const uint8_t nonce[kOtaNonceBytes], const OtaImageMetadata& metadata,
                          uint8_t out[kOtaSignedPayloadBytes]) {
  size_t offset = 0;
  memcpy(out + offset, nonce, kOtaNonceBytes);
  offset += kOtaNonceBytes;

  out[offset++] = static_cast<uint8_t>((metadata.schema_version >> 24) & 0xFF);
  out[offset++] = static_cast<uint8_t>((metadata.schema_version >> 16) & 0xFF);
  out[offset++] = static_cast<uint8_t>((metadata.schema_version >> 8) & 0xFF);
  out[offset++] = static_cast<uint8_t>(metadata.schema_version & 0xFF);

  out[offset++] = static_cast<uint8_t>((metadata.image_size >> 24) & 0xFF);
  out[offset++] = static_cast<uint8_t>((metadata.image_size >> 16) & 0xFF);
  out[offset++] = static_cast<uint8_t>((metadata.image_size >> 8) & 0xFF);
  out[offset++] = static_cast<uint8_t>(metadata.image_size & 0xFF);

  memcpy(out + offset, metadata.sha256, kSha256DigestBytes);
  offset += kSha256DigestBytes;

  // Zero-padded explicitly here, rather than copying metadata.build_id's
  // raw fixed-size array verbatim: bytes past the NUL terminator are only
  // guaranteed zero by OtaImageMetadata's own default member initializer,
  // not by every caller that fills one in. Copying raw trailing bytes would
  // make the signed payload depend on whatever garbage happened to occupy
  // an unrelated stack/struct slot — nondeterministic, and a correctness
  // bug independent of any security concern.
  const size_t build_id_len = strnlen(metadata.build_id, kOtaBuildIdBytes);
  memcpy(out + offset, metadata.build_id, build_id_len);
  if (build_id_len < kOtaBuildIdBytes) {
    memset(out + offset + build_id_len, 0, kOtaBuildIdBytes - build_id_len);
  }
  offset += kOtaBuildIdBytes;

  (void)offset;  // == kOtaSignedPayloadBytes; asserted by the caller's buffer size
}

void OtaSession::begin(const OtaSessionConfig& config, const uint8_t* secret, size_t secret_len) {
  config_ = config;
  secret_ = secret;
  secret_len_ = secret_len;
  challenge_outstanding_ = false;
  session_active_ = false;
}

void OtaSession::issueChallenge(uint32_t now_ms, const uint8_t random_bytes[kOtaNonceBytes],
                                uint8_t out_nonce[kOtaNonceBytes]) {
  // Idempotent while a still-valid challenge is outstanding (I7 hardening,
  // 2026-09-25): an unauthenticated caller repeatedly hitting the challenge
  // endpoint must not be able to invalidate a legitimate client's in-flight
  // nonce. Re-issuing the SAME nonce is safe — it is not bound to a caller
  // identity, only consumed once by whichever request first presents a
  // valid signature over it (authenticate() still single-use-consumes it) —
  // so this closes the availability gap without weakening anything: replay
  // is still rejected, expiration is still bounded, and an abandoned
  // challenge still cannot lock out future ones past challenge_ttl_ms.
  if (challenge_outstanding_ &&
      (now_ms - challenge_issued_ms_) <= config_.challenge_ttl_ms) {
    memcpy(out_nonce, nonce_, kOtaNonceBytes);
    return;
  }
  memcpy(nonce_, random_bytes, kOtaNonceBytes);
  challenge_issued_ms_ = now_ms;
  challenge_outstanding_ = true;
  memcpy(out_nonce, nonce_, kOtaNonceBytes);
}

OtaAuthResult OtaSession::authenticate(uint32_t now_ms, const uint8_t nonce[kOtaNonceBytes],
                                       const OtaImageMetadata& metadata,
                                       const uint8_t signature[kHmac256DigestBytes]) {
  if (!hasSecret()) return OtaAuthResult::REJECTED_NO_SECRET;
  if (!challenge_outstanding_) return OtaAuthResult::REJECTED_NO_CHALLENGE;

  if (memcmp(nonce, nonce_, kOtaNonceBytes) != 0) {
    // Does NOT consume the real outstanding challenge: this attempt never
    // validly referenced it, so it must not be able to deny the legitimate
    // holder of that challenge their one use of it. Without this, a single
    // garbage request (or a captured, already-superseded old nonce) could
    // burn a challenge it was never entitled to and deny the real caller —
    // an availability bug immediately adjacent to the security property
    // this class exists to provide.
    return OtaAuthResult::REJECTED_NONCE_MISMATCH;
  }

  // The presented nonce IS the real outstanding one: every outcome below
  // consumes it, whether this attempt ultimately succeeds or fails — which
  // is what makes both replay (succeed twice) and brute-force (fail, then
  // retry against the same still-live nonce) impossible.
  const bool expired = (now_ms - challenge_issued_ms_) > config_.challenge_ttl_ms;
  challenge_outstanding_ = false;

  if (expired) return OtaAuthResult::REJECTED_CHALLENGE_EXPIRED;

  uint8_t payload[kOtaSignedPayloadBytes];
  buildOtaSignedPayload(nonce, metadata, payload);
  uint8_t expected[kHmac256DigestBytes];
  hmac256(secret_, secret_len_, payload, sizeof(payload), expected);

  if (!digestsEqual(expected, signature)) return OtaAuthResult::REJECTED_SIGNATURE;

  session_active_ = true;
  last_activity_ms_ = now_ms;
  return OtaAuthResult::OK;
}

void OtaSession::noteActivity(uint32_t now_ms) {
  if (session_active_) last_activity_ms_ = now_ms;
}

bool OtaSession::timedOut(uint32_t now_ms) const {
  if (!session_active_) return false;
  return (now_ms - last_activity_ms_) > config_.activity_timeout_ms;
}

void OtaSession::reset() {
  session_active_ = false;
  last_activity_ms_ = 0;
}

const char* toString(OtaAuthResult result) {
  switch (result) {
    case OtaAuthResult::OK:                         return "OK";
    case OtaAuthResult::REJECTED_NO_SECRET:         return "REJECTED_NO_SECRET";
    case OtaAuthResult::REJECTED_NO_CHALLENGE:      return "REJECTED_NO_CHALLENGE";
    case OtaAuthResult::REJECTED_NONCE_MISMATCH:    return "REJECTED_NONCE_MISMATCH";
    case OtaAuthResult::REJECTED_CHALLENGE_EXPIRED: return "REJECTED_CHALLENGE_EXPIRED";
    case OtaAuthResult::REJECTED_SIGNATURE:         return "REJECTED_SIGNATURE";
  }
  return "UNKNOWN";
}

}  // namespace update
}  // namespace matdog
