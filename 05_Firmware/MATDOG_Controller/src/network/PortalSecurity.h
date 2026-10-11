#ifndef MATDOG_NETWORK_PORTAL_SECURITY_H
#define MATDOG_NETWORK_PORTAL_SECURITY_H
#include <stddef.h>
#include <stdint.h>

#include "NetworkConfig.h"
namespace matdog {
namespace network {
constexpr size_t kPortalBodyBytes = 1536;
bool portalOrigin(bool ap_socket, const char *host, const char *origin);
struct ConfigPatch {
  NetworkConfig config{};
  uint8_t profiles = 0, passwords = 0;
  bool performance = false, ap = false, sleep = false;
  uint8_t test_profile = 0;
};
bool parseConfigForm(const char *body, size_t length, ConfigPatch *patch);
// "Remember this device": a long-lived credential the owner's browser keeps
// after one successful token login. It is stateless on the controller:
//   credential = hex(id[16]) || hex(HMAC-SHA256(admin_digest, label || id)[0..16))
// Nothing is stored per device and nothing is written to NVS, so it survives a
// reboot and needs no wall clock. It is valid exactly as long as the admin
// digest it was derived from: provisioning a new administrator token revokes
// every remembered device at once. It never authorizes a write by itself; it
// can only open a normal short session (cookie + CSRF, peer-bound).
constexpr size_t kPortalDeviceIdBytes = 16;
void portalDeviceIssue(const uint8_t digest[32],
                       const uint8_t id[kPortalDeviceIdBytes], char out[65]);
bool portalDeviceValid(const uint8_t digest[32], const char *credential);
class PortalSession {
public:
  bool login(uint64_t now, uint32_t peer, const char *supplied,
             const uint8_t digest[32], const uint8_t random[64]);
  // Opens a session from a remembered-device credential instead of the token.
  // Same session, same expiry, same peer binding; failures are throttled.
  bool resume(uint64_t now, uint32_t peer, const char *device,
              const uint8_t digest[32], const uint8_t random[64]);
  // The caller holds the live session (cookie, peer, not expired), without
  // the CSRF proof: enough to be told its own CSRF token again after a page
  // reload, or to end the session. Never enough for a write.
  bool current(uint64_t now, uint32_t peer, const char *cookie);
  bool authorize(uint64_t now, uint32_t peer, const char *cookie,
                 const char *csrf);
  const char *cookie() const { return cookie_; }
  const char *csrf() const { return csrf_; }
  void reset() { active_ = false; }

private:
  void open(uint64_t now, uint32_t peer, const uint8_t random[64]);
  bool active_ = false, attempted_ = false, resume_failed_ = false;
  // Monotonic milliseconds from esp_timer, never a widened millis() sample.
  uint64_t issued_ = 0, last_attempt_ = 0, resume_failed_at_ = 0;
  uint32_t peer_ = 0;
  uint8_t failures_ = 0, resume_failures_ = 0;
  char cookie_[65]{}, csrf_[65]{};
};
} // namespace network
} // namespace matdog
#endif
