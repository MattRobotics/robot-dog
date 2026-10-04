#include "NetworkConfig.h"

#include <string.h>
namespace matdog {
namespace network {
namespace {
uint32_t crc(const uint8_t *b, size_t n) {
  uint32_t c = 0xffffffffu;
  for (size_t i = 0; i < n; ++i) {
    c ^= b[i];
    for (unsigned j = 0; j < 8; ++j)
      c = (c >> 1) ^ ((0u - (c & 1u)) & 0xedb88320u);
  }
  return ~c;
}
bool bounded(const char *s, size_t n) { return memchr(s, 0, n) != nullptr; }
bool password(const char *s) {
  if (!bounded(s, 65))
    return false;
  size_t n = strlen(s);
  if (n < 8 || n > 63)
    return false;
  for (size_t i = 0; i < n; ++i)
    if (static_cast<unsigned char>(s[i]) < 32 ||
        static_cast<unsigned char>(s[i]) > 126)
      return false;
  return true;
}
uint32_t be(uint32_t v) {
  return ((v & 255) << 24) | ((v & 0xff00) << 8) | ((v >> 8) & 0xff00) |
         (v >> 24);
}
bool unicast(uint32_t v) {
  uint32_t b = be(v);
  return b && b != 0xffffffffu && (b >> 24) != 0 && (b >> 24) != 127 &&
         (b >> 24) < 224;
}
void put(uint8_t *&p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i)
    *p++ = static_cast<uint8_t>(v >> (i * 8));
}
uint32_t get(const uint8_t *&p) {
  uint32_t v = 0;
  for (unsigned i = 0; i < 4; ++i)
    v |= static_cast<uint32_t>(*p++) << (i * 8);
  return v;
}
} // namespace
bool parseIpv4(const char *t, uint32_t *out) {
  if (!t || !out)
    return false;
  uint32_t v = 0;
  for (unsigned i = 0; i < 4; ++i) {
    unsigned n = 0, digits = 0;
    while (*t >= '0' && *t <= '9') {
      n = n * 10 + (*t++ - '0');
      if (++digits > 3 || n > 255)
        return false;
    }
    if (!digits)
      return false;
    v |= n << (i * 8);
    if (i < 3 && *t++ != '.')
      return false;
  }
  if (*t)
    return false;
  *out = v;
  return true;
}
bool hasAdmin(const NetworkConfig &c) {
  uint8_t v = 0;
  for (auto b : c.admin_digest)
    v |= b;
  return v != 0;
}
bool validNetworkConfig(const NetworkConfig &c) {
  if (!bounded(c.ap_name, 33) || !bounded(c.ap_password, 65))
    return false;
  if (c.ap_password[0] && !password(c.ap_password))
    return false;
  if (c.ap_timeout_ms < 60000 || c.ap_timeout_ms > 3600000 ||
      c.roam_threshold < -95 || c.roam_threshold > -50 ||
      c.roam_hysteresis < 3 || c.roam_hysteresis > 30 ||
      c.scan_interval_ms < 30000 || c.scan_interval_ms > 3600000 ||
      c.roam_dwell_ms < 60000 || c.roam_dwell_ms > 3600000 ||
      (c.bandwidth_mhz != 20 && c.bandwidth_mhz != 40) || c.usb_sleep > 2)
    return false;
  for (const auto &s : c.sta) {
    if (!bounded(s.ssid, 33) || !bounded(s.password, 65))
      return false;
    for (size_t i = 0; s.ssid[i]; ++i)
      if (static_cast<unsigned char>(s.ssid[i]) < 32 ||
          static_cast<unsigned char>(s.ssid[i]) == 127)
        return false;
    if (!s.enabled)
      continue;
    if (!s.ssid[0] || (s.password[0] && !password(s.password)))
      return false;
    if (!s.dhcp) {
      uint32_t mask = be(s.mask), host = ~mask, ip = be(s.ip),
               gw = be(s.gateway);
      if (!mask || host < 3 || (host & (host + 1)) || !unicast(s.ip) ||
          !unicast(s.gateway) || (s.dns && !unicast(s.dns)) ||
          (ip & mask) != (gw & mask) || !(ip & host) || (ip & host) == host ||
          !(gw & host) || (gw & host) == host)
        return false;
      if ((ip & 0xffffff00u) == 0xc0a80400u)
        return false; // no collision with recovery AP /24
    }
  }
  return true;
}
void encodeNetworkConfig(const NetworkConfig &c,
                         uint8_t out[kNetworkRecordBytes]) {
  memset(out, 0, kNetworkRecordBytes);
  uint8_t *p = out;
  put(p, 0x314e444du);
  put(p, 1);
  for (const auto &s : c.sta) {
    memcpy(p, s.ssid, 33);
    p += 33;
    memcpy(p, s.password, 65);
    p += 65;
    *p++ = s.enabled;
    *p++ = s.dhcp;
    put(p, s.ip);
    put(p, s.mask);
    put(p, s.gateway);
    put(p, s.dns);
  }
  memcpy(p, c.ap_name, 33);
  p += 33;
  memcpy(p, c.ap_password, 65);
  p += 65;
  memcpy(p, c.admin_digest, 32);
  p += 32;
  put(p, c.ap_timeout_ms);
  *p++ = c.ap_always;
  *p++ = c.roam_enabled;
  *p++ = static_cast<uint8_t>(c.roam_threshold);
  *p++ = c.roam_hysteresis;
  put(p, c.scan_interval_ms);
  put(p, c.roam_dwell_ms);
  *p++ = c.bandwidth_mhz;
  *p++ = c.usb_sleep;
  p = out + kNetworkRecordBytes - 4;
  put(p, crc(out, kNetworkRecordBytes - 4));
}
bool decodeNetworkConfig(const uint8_t *b, size_t n, NetworkConfig *out) {
  if (!b || !out || n != kNetworkRecordBytes)
    return false;
  const uint8_t *tail = b + n - 4;
  if (get(tail) != crc(b, n - 4))
    return false;
  const uint8_t *p = b;
  if (get(p) != 0x314e444du || get(p) != 1)
    return false;
  NetworkConfig c{};
  for (auto &s : c.sta) {
    memcpy(s.ssid, p, 33);
    p += 33;
    memcpy(s.password, p, 65);
    p += 65;
    if (p[0] > 1 || p[1] > 1)
      return false;
    s.enabled = *p++;
    s.dhcp = *p++;
    s.ip = get(p);
    s.mask = get(p);
    s.gateway = get(p);
    s.dns = get(p);
  }
  memcpy(c.ap_name, p, 33);
  p += 33;
  memcpy(c.ap_password, p, 65);
  p += 65;
  memcpy(c.admin_digest, p, 32);
  p += 32;
  c.ap_timeout_ms = get(p);
  if (p[0] > 1 || p[1] > 1)
    return false;
  c.ap_always = *p++;
  c.roam_enabled = *p++;
  c.roam_threshold = static_cast<int8_t>(*p++);
  c.roam_hysteresis = *p++;
  c.scan_interval_ms = get(p);
  c.roam_dwell_ms = get(p);
  c.bandwidth_mhz = *p++;
  c.usb_sleep = *p++;
  if (!validNetworkConfig(c))
    return false;
  *out = c;
  return true;
}
void ConfigTransaction::begin(NetworkStorage *s, const NetworkConfig &c) {
  storage_ = s;
  active_ = c;
  phase_ = ConfigPhase::IDLE;
}
bool ConfigTransaction::stage(const NetworkConfig &c, uint8_t profile,
                              uint32_t now, bool alt, bool critical) {
  if (busy() || critical || !alt || profile > 1 || !c.sta[profile].enabled ||
      !validNetworkConfig(c) || !storage_)
    return false;
  candidate_ = c;
  phase_ = ConfigPhase::CONFIG_PENDING;
  if (!storage_->pending(c)) {
    phase_ = ConfigPhase::STORAGE_ERROR;
    return false;
  }
  started_ = now;
  phase_ = ConfigPhase::TESTING;
  return true;
}
void ConfigTransaction::update(uint32_t now, bool associated, bool ip,
                               bool critical) {
  if (!busy())
    return;
  if (critical || now - started_ >= 30000) {
    phase_ = ConfigPhase::ROLLBACK;
    return;
  }
  if (associated && ip) {
    if (storage_->activate(candidate_)) {
      active_ = candidate_;
      phase_ = ConfigPhase::COMMITTED;
    } else
      phase_ = ConfigPhase::STORAGE_ERROR;
  }
}
const char *toString(ConfigPhase p) {
  switch (p) {
  case ConfigPhase::IDLE:
    return "IDLE";
  case ConfigPhase::CONFIG_PENDING:
    return "CONFIG_PENDING";
  case ConfigPhase::TESTING:
    return "TESTING";
  case ConfigPhase::COMMITTED:
    return "COMMITTED";
  case ConfigPhase::ROLLBACK:
    return "ROLLBACK";
  case ConfigPhase::STORAGE_ERROR:
    return "STORAGE_ERROR";
  }
  return "UNKNOWN";
}
} // namespace network
} // namespace matdog
