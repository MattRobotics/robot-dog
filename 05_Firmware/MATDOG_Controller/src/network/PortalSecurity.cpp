#include "PortalSecurity.h"

#include <string.h>

#include "../update/Hmac256.h"
#include "../update/Sha256.h"
namespace matdog {
namespace network {
namespace {
int hex(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return -1;
}
bool decode(const char *p, size_t n, char *out, size_t cap) {
  size_t w = 0;
  for (size_t i = 0; i < n; ++i) {
    unsigned char c = p[i];
    if (c == '%') {
      if (i + 2 >= n || hex(p[i + 1]) < 0 || hex(p[i + 2]) < 0)
        return false;
      c = (hex(p[i + 1]) << 4) | hex(p[i + 2]);
      i += 2;
    } else if (c == '+')
      c = ' ';
    if (!c || c < 32 || c == 127 || w + 1 >= cap)
      return false;
    out[w++] = c;
  }
  out[w] = 0;
  return true;
}
bool number(const char *s, uint32_t *out) {
  uint32_t v = 0;
  if (!*s)
    return false;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9' || v > 3600000)
      return false;
    v = v * 10 + (*s - '0');
  }
  *out = v;
  return true;
}
bool tokenEqual(const char *a, const char *b) {
  if (!a || strlen(a) != 64)
    return false;
  uint8_t v = 0;
  for (unsigned i = 0; i < 64; ++i)
    v |= a[i] ^ b[i];
  return v == 0;
}
// Domain separation: the admin digest is also the login verifier.
constexpr char kDeviceLabel[] = "MATDOG-PORTAL-DEVICE-1";
bool anyByte(const uint8_t *bytes, size_t n) {
  uint8_t v = 0;
  for (size_t i = 0; i < n; ++i)
    v |= bytes[i];
  return v != 0;
}
void deviceTag(const uint8_t digest[32], const uint8_t id[kPortalDeviceIdBytes],
               uint8_t tag[update::kHmac256DigestBytes]) {
  uint8_t message[sizeof(kDeviceLabel) - 1 + kPortalDeviceIdBytes];
  memcpy(message, kDeviceLabel, sizeof(kDeviceLabel) - 1);
  memcpy(message + sizeof(kDeviceLabel) - 1, id, kPortalDeviceIdBytes);
  update::hmac256(digest, 32, message, sizeof(message), tag);
}
} // namespace
void portalDeviceIssue(const uint8_t digest[32],
                       const uint8_t id[kPortalDeviceIdBytes], char out[65]) {
  static const char digits[] = "0123456789abcdef";
  uint8_t tag[update::kHmac256DigestBytes];
  deviceTag(digest, id, tag);
  for (size_t i = 0; i < kPortalDeviceIdBytes; ++i) {
    out[2 * i] = digits[id[i] >> 4];
    out[2 * i + 1] = digits[id[i] & 15];
    out[32 + 2 * i] = digits[tag[i] >> 4];
    out[32 + 2 * i + 1] = digits[tag[i] & 15];
  }
  out[64] = 0;
  memset(tag, 0, sizeof(tag));
}
bool portalDeviceValid(const uint8_t digest[32], const char *credential) {
  // No administrator provisioned: nothing can be remembered.
  if (!credential || strlen(credential) != 64 || !anyByte(digest, 32))
    return false;
  uint8_t raw[32], tag[update::kHmac256DigestBytes];
  if (!update::parseHex(credential, 64, raw))
    return false;
  deviceTag(digest, raw, tag);
  uint8_t diff = 0;
  for (size_t i = 0; i < 16; ++i)
    diff |= raw[16 + i] ^ tag[i];
  memset(tag, 0, sizeof(tag));
  return diff == 0;
}
bool portalOrigin(bool ap, const char *host, const char *origin) {
  return ap && host && origin &&
         (strcmp(host, "192.168.4.1") == 0 ||
          strcmp(host, "192.168.4.1:80") == 0) &&
         (strcmp(origin, "http://192.168.4.1") == 0 ||
          strcmp(origin, "http://192.168.4.1:80") == 0);
}
bool parseConfigForm(const char *body, size_t n, ConfigPatch *out) {
  if (!body || !out || !n || n >= kPortalBodyBytes)
    return false;
  ConfigPatch patch{};
  uint64_t seen = 0;
  size_t pos = 0;
  while (pos < n) {
    size_t end = pos;
    while (end < n && body[end] != '&')
      ++end;
    size_t eq = pos;
    while (eq < end && body[eq] != '=')
      ++eq;
    if (eq == end)
      return false;
    char name[32], value[65];
    if (!decode(body + pos, eq - pos, name, sizeof(name)) ||
        !decode(body + eq + 1, end - eq - 1, value, sizeof(value)))
      return false;
    int field = -1;
    const char *keys[] = {
        "p0_ssid",        "p0_password",     "p0_enabled",
        "p0_dhcp",        "p0_ip",           "p0_mask",
        "p0_gateway",     "p0_dns",          "p1_ssid",
        "p1_password",    "p1_enabled",      "p1_dhcp",
        "p1_ip",          "p1_mask",         "p1_gateway",
        "p1_dns",         "test_profile",    "roam_enabled",
        "roam_threshold", "roam_hysteresis", "scan_interval_ms",
        "roam_dwell_ms",  "bandwidth_mhz",   "ap_name",
        "ap_password",    "ap_timeout_ms",   "ap_always",
        "usb_sleep"};
    for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i)
      if (strcmp(name, keys[i]) == 0) {
        field = i;
        break;
      }
    if (field < 0 || (seen & (1ull << field)))
      return false;
    seen |= 1ull << field;
    uint32_t v = 0;
    if (field < 16) {
      unsigned idx = field / 8, sub = field % 8;
      auto &s = patch.config.sta[idx];
      patch.profiles |= 1u << idx;
      if (sub == 0) {
        if (strlen(value) > 32)
          return false;
        memcpy(s.ssid, value, strlen(value) + 1);
      } else if (sub == 1) {
        if (*value) {
          memcpy(s.password, value, strlen(value) + 1);
          patch.passwords |= 1u << idx;
        }
      } else if (sub == 2 || sub == 3) {
        if (!number(value, &v) || v > 1)
          return false;
        if (sub == 2)
          s.enabled = v;
        else
          s.dhcp = v;
      } else {
        if (!parseIpv4(value, &v))
          return false;
        if (sub == 4)
          s.ip = v;
        else if (sub == 5)
          s.mask = v;
        else if (sub == 6)
          s.gateway = v;
        else
          s.dns = v;
      }
    } else if (field == 23) {
      if (strlen(value) > 32)
        return false;
      memcpy(patch.config.ap_name, value, strlen(value) + 1);
      patch.ap = true;
    } else if (field == 24) {
      memcpy(patch.config.ap_password, value, strlen(value) + 1);
      patch.ap = true;
    } else {
      if (field == 18) {
        if (*value != '-' || !number(value + 1, &v) || v > 127)
          return false;
        patch.config.roam_threshold = -static_cast<int>(v);
      } else {
        if (!number(value, &v))
          return false;
        switch (field) {
        case 16:
          if (v > 1)
            return false;
          patch.test_profile = v;
          break;
        case 17:
          if (v > 1)
            return false;
          patch.config.roam_enabled = v;
          break;
        case 19:
          if (v > 30)
            return false;
          patch.config.roam_hysteresis = v;
          break;
        case 20:
          patch.config.scan_interval_ms = v;
          break;
        case 21:
          patch.config.roam_dwell_ms = v;
          break;
        case 22:
          if (v != 20 && v != 40)
            return false;
          patch.config.bandwidth_mhz = v;
          break;
        case 25:
          patch.config.ap_timeout_ms = v;
          break;
        case 26:
          if (v > 1)
            return false;
          patch.config.ap_always = v;
          break;
        case 27:
          if (v > 2)
            return false;
          patch.config.usb_sleep = v;
          patch.sleep = true;
          break;
        default:
          return false;
        }
      }
      if (field >= 17 && field <= 22)
        patch.performance = true;
      if (field >= 25 && field <= 26)
        patch.ap = true;
    }
    pos = end + 1;
  }
  // Whole profile and whole performance/AP blocks, to avoid accidental reset
  // via partial forms.
  for (unsigned idx = 0; idx < 2; ++idx)
    if (patch.profiles & (1u << idx)) {
      uint64_t required = (0xfdu << (idx * 8));
      if ((seen & required) != required)
        return false;
    }
  if (patch.performance && (seen & 0x7e0000u) != 0x7e0000u)
    return false;
  if (patch.ap && (seen & 0x7800000u) != 0x7800000u)
    return false;
  if (!patch.profiles && !patch.performance && !patch.ap &&
      !(seen & (1ull << 27)))
    return false;
  *out = patch;
  return true;
}
bool PortalSession::login(uint64_t now, uint32_t peer, const char *supplied,
                          const uint8_t digest[32], const uint8_t random[64]) {
  // A regressing clock cannot bypass throttling or preserve an old session.
  if (attempted_ && now < last_attempt_) {
    reset();
    return false;
  }
  if (attempted_ && now - last_attempt_ < (failures_ >= 5 ? 60000u : 1000u))
    return false;
  attempted_ = true;
  last_attempt_ = now;
  uint8_t hash[32]{};
  bool valid = supplied && strlen(supplied) == 64;
  uint8_t any = 0;
  for (unsigned i = 0; i < 32; ++i)
    any |= digest[i];
  valid = valid && any;
  if (valid) {
    for (unsigned i = 0; i < 64; ++i)
      if (hex(supplied[i]) < 0)
        valid = false;
    update::Sha256 sha;
    sha.update(reinterpret_cast<const uint8_t *>(supplied), 64);
    sha.finish(hash);
    valid = valid && update::digestsEqual(hash, digest);
  }
  if (!valid) {
    if (failures_ < 5)
      ++failures_;
    return false;
  }
  failures_ = 0;
  open(now, peer, random);
  return true;
}
void PortalSession::open(uint64_t now, uint32_t peer, const uint8_t random[64]) {
  active_ = true;
  issued_ = now;
  peer_ = peer;
  update::toHex(random, cookie_, sizeof(cookie_));
  update::toHex(random + 32, csrf_, sizeof(csrf_));
}
bool PortalSession::resume(uint64_t now, uint32_t peer, const char *device,
                           const uint8_t digest[32], const uint8_t random[64]) {
  // No credential presented is not an attempt: a browser without the cookie
  // must not be able to hold the throttle against the owner's device.
  if (!device || !device[0])
    return false;
  // Same fail-closed clock rule as login(); a bad credential is throttled so
  // the comparison cannot be driven at request rate.
  if (resume_failed_ && now < resume_failed_at_) {
    reset();
    return false;
  }
  if (resume_failed_ &&
      now - resume_failed_at_ < (resume_failures_ >= 5 ? 60000u : 1000u))
    return false;
  if (!portalDeviceValid(digest, device)) {
    resume_failed_ = true;
    resume_failed_at_ = now;
    if (resume_failures_ < 5)
      ++resume_failures_;
    return false;
  }
  resume_failed_ = false;
  resume_failures_ = 0;
  open(now, peer, random);
  return true;
}
bool PortalSession::current(uint64_t now, uint32_t peer, const char *cookie) {
  // Expiry is irreversible until a fresh authenticated login. Reject a clock
  // regression too; subtraction must never revive an expired session.
  if (active_ && !(now >= issued_ && now - issued_ < 300000))
    reset();
  return active_ && peer == peer_ && tokenEqual(cookie, cookie_);
}
bool PortalSession::authorize(uint64_t now, uint32_t peer, const char *cookie,
                              const char *csrf) {
  // Authenticated use (peer + cookie + CSRF) keeps the session alive: it ends
  // 300 s after its last use, at logout or on a new AP generation, never
  // while the Dashboard is in use.
  if (!current(now, peer, cookie) || !tokenEqual(csrf, csrf_)) return false;
  issued_ = now;
  return true;
}
} // namespace network
} // namespace matdog
