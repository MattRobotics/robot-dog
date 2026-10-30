// Remembered-device credential and session resume: production PortalSecurity,
// HMAC-SHA256 and SHA-256, synthetic credentials only. No device, no socket.
#include <cstdio>
#include <cstring>
#include <string>
#include "network/PortalSecurity.h"
#include "update/Sha256.h"
using namespace matdog::network;
static unsigned checks = 0, failures = 0;
static void check(bool result, const char *name) {
  ++checks;
  if (!result) { ++failures; std::printf("FAIL %s\n", name); }
}
static void digestOf(const std::string &token, uint8_t out[32]) {
  matdog::update::Sha256 sha;
  sha.update(reinterpret_cast<const uint8_t *>(token.data()), token.size());
  sha.finish(out);
}
int main() {
  constexpr uint32_t peer = 0x0204a8c0;
  const std::string token(64, 'a');
  uint8_t digest[32], other_digest[32], zero[32]{}, id[16], random[64];
  digestOf(token, digest);
  digestOf(std::string(64, 'c'), other_digest);
  for (unsigned i = 0; i < 16; ++i) id[i] = static_cast<uint8_t>(i);
  // Distinct bytes, so the session cookie and the CSRF token differ.
  for (unsigned i = 0; i < 64; ++i) random[i] = static_cast<uint8_t>(i + 1);

  // Known answer, computed independently (Python hmac/hashlib):
  //   HMAC-SHA256(SHA256("a"*64), "MATDOG-PORTAL-DEVICE-1" || 00..0f)[0..16)
  char device[65];
  portalDeviceIssue(digest, id, device);
  check(std::strcmp(device, "000102030405060708090a0b0c0d0e0fceea07aeba50473d3d5f64801344cd2f") == 0,
        "known-answer credential");
  check(portalDeviceValid(digest, device), "issued credential verifies");
  check(std::strlen(device) == 64 && std::string(device) != token, "64 hex, never the token");
  check(std::string(device).find(std::string(device).substr(32)) == 32, "id then tag");

  // It is bound to the administrator digest and to its own id.
  check(!portalDeviceValid(other_digest, device), "another administrator token: invalid");
  check(!portalDeviceValid(zero, device), "no administrator provisioned: invalid");
  char zero_device[65];
  portalDeviceIssue(zero, id, zero_device);
  check(!portalDeviceValid(zero, zero_device), "nothing can be remembered without an administrator");
  for (unsigned i = 0; i < 64; ++i) {
    std::string forged = device;
    forged[i] = forged[i] == '0' ? '1' : '0';
    if (portalDeviceValid(digest, forged.c_str())) { check(false, "single-character forgery accepted"); break; }
  }
  check(true, "every single-character change is rejected");
  check(!portalDeviceValid(digest, nullptr), "null credential");
  check(!portalDeviceValid(digest, ""), "empty credential");
  check(!portalDeviceValid(digest, std::string(device).substr(0, 63).c_str()), "short credential");
  check(!portalDeviceValid(digest, (std::string(device) + "0").c_str()), "long credential");
  check(!portalDeviceValid(digest, std::string(64, 'z').c_str()), "non-hex credential");
  check(!portalDeviceValid(digest, token.c_str()), "the token is not a credential");
  uint8_t id2[16];
  std::memset(id2, 0x5a, sizeof(id2));
  char device2[65];
  portalDeviceIssue(digest, id2, device2);
  check(std::strcmp(device, device2) != 0 && portalDeviceValid(digest, device2), "each device gets its own credential");
  check(std::strncmp(device + 32, device2 + 32, 32) != 0, "tag depends on the id");

  // resume(): a normal session, with the normal limits.
  constexpr uint64_t t0 = 10000, cycle = uint64_t{1} << 32;
  PortalSession s;
  check(!s.current(t0, peer, device), "a device credential is not a session cookie");
  check(!s.authorize(t0, peer, device, device), "a device credential authorizes nothing");
  check(s.resume(t0, peer, device, digest, random), "resume opens a session");
  char cookie[65], csrf[65];
  std::strcpy(cookie, s.cookie());
  std::strcpy(csrf, s.csrf());
  check(std::strcmp(cookie, device) != 0 && std::strcmp(csrf, device) != 0, "session secrets are independent of the credential");
  check(s.authorize(t0, peer, cookie, csrf), "resumed session authorizes with cookie and CSRF");
  check(!s.authorize(t0, peer + 1, cookie, csrf), "peer binding");
  check(!s.authorize(t0, peer, cookie, cookie), "CSRF still required");
  check(s.current(t0, peer, cookie), "holder recognised without CSRF");
  check(!s.current(t0, peer + 1, cookie), "holder check is peer bound");
  check(!s.current(t0, peer, csrf), "CSRF is not the cookie");
  // Authenticated use renews the idle expiry: a Dashboard polling every
  // 350 ms keeps the session for 20 minutes; 300 s without use ends it.
  uint64_t used = t0;
  for (uint64_t t = t0; t <= t0 + 1200000; t += 350) { if (!s.authorize(t, peer, cookie, csrf)) { check(false, "continuous use keeps the session"); break; } used = t; }
  check(s.authorize(used + 299999, peer, cookie, csrf), "idle TTL minus one");
  used += 299999;
  check(!s.current(used + 299999, peer, csrf), "CSRF still never a cookie");
  check(!s.authorize(used + 1, peer + 1, cookie, csrf) && !s.authorize(used + 1, peer, cookie, cookie), "a refused request renews nothing");
  check(!s.authorize(used + 300000, peer, cookie, csrf), "300 s without use: resumed session expires like any other");
  check(!s.current(used + 300000, peer, cookie), "expired session has no holder");
  check(!s.authorize(used, peer, cookie, csrf), "expiry is irreversible");
  // ...and is renewed from the device, with new secrets.
  for (unsigned i = 0; i < 64; ++i) random[i] = static_cast<uint8_t>(200 - i);
  check(s.resume(used + 300000, peer, device, digest, random), "renewal after expiry, no token");
  check(!s.authorize(used + 300000, peer, cookie, csrf), "old cookie/CSRF not reusable after renewal");
  check(s.authorize(used + 300000, peer, s.cookie(), s.csrf()), "renewed session valid");
  check(s.resume(t0 + cycle, peer, device, digest, random) && s.authorize(t0 + cycle + 1, peer, s.cookie(), s.csrf()),
        "valid across the 32-bit millisecond wrap");

  // Failures are throttled; absent credentials are not attempts.
  PortalSession t;
  std::string forged = device;
  forged[63] = forged[63] == '0' ? '1' : '0';
  check(!t.resume(t0, peer, forged.c_str(), digest, random), "forged credential refused");
  check(!t.resume(t0 + 999, peer, device, digest, random), "valid credential throttled for 1 s after a failure");
  check(t.resume(t0 + 1000, peer, device, digest, random), "1 s throttle releases");
  PortalSession q;
  for (unsigned i = 0; i < 5; ++i)
    check(!q.resume(t0 + i * 1000, peer, forged.c_str(), digest, random), "failure accounting");
  check(!q.resume(t0 + 4000 + 59999, peer, device, digest, random), "60 s throttle minus one");
  check(q.resume(t0 + 4000 + 60000, peer, device, digest, random), "60 s throttle releases");
  PortalSession absent;
  for (unsigned i = 0; i < 20; ++i) {
    check(!absent.resume(t0 + i, peer, "", digest, random), "no credential, no session");
    check(!absent.resume(t0 + i, peer, nullptr, digest, random), "null credential, no session");
  }
  check(absent.resume(t0 + 20, peer, device, digest, random), "absent credentials never throttle the owner");
  PortalSession back;
  check(!back.resume(t0, peer, forged.c_str(), digest, random), "clock regression setup");
  check(!back.resume(t0 - 1, peer, device, digest, random), "backward clock fails closed");
  PortalSession revoked;
  check(revoked.resume(t0, peer, device, digest, random), "revocation setup");
  check(!revoked.resume(t0 + 1, peer, device, other_digest, random), "administrator token change revokes the device");
  check(!revoked.resume(t0 + 5000, peer, device, zero, random), "cleared administrator revokes the device");

  // Token login and the device path keep independent throttles.
  PortalSession both;
  check(!both.login(t0, peer, "wrong", digest, random), "wrong token");
  check(both.resume(t0 + 1, peer, device, digest, random), "device not blocked by a token failure");
  check(!both.resume(t0 + 2, peer, forged.c_str(), digest, random), "forged device");
  check(both.login(t0 + 1000, peer, token.c_str(), digest, random), "token not blocked by a device failure");
  std::printf("PORTAL_DEVICE: %u checks, %u failures\n", checks, failures);
  return failures ? 1 : 0;
}
