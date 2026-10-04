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
class PortalSession {
public:
  bool login(uint32_t now, uint32_t peer, const char *supplied,
             const uint8_t digest[32], const uint8_t random[64]);
  bool authorize(uint32_t now, uint32_t peer, const char *cookie,
                 const char *csrf) const;
  const char *cookie() const { return cookie_; }
  const char *csrf() const { return csrf_; }
  void reset() { active_ = false; }

private:
  bool active_ = false, attempted_ = false;
  uint32_t issued_ = 0, last_attempt_ = 0, peer_ = 0;
  uint8_t failures_ = 0;
  char cookie_[65]{}, csrf_[65]{};
};
} // namespace network
} // namespace matdog
#endif
