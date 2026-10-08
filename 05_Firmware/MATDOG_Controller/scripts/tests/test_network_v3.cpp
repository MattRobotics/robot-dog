#include <nvs.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "../../src/network/HttpMailbox.h"
#include "../../src/network/NetworkConfig.h"
#include "../../src/network/NetworkConfigNvs.h"
#include "../../src/network/NetworkPolicy.h"
#include "../../src/network/PortalSecurity.h"
#include "../../src/update/Sha256.h"
using namespace matdog::network;
static int checks = 0, failures = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(c)) {                                                                \
      ++failures;                                                              \
      std::printf("FAIL line %d: %s\n", __LINE__, #c);                         \
    }                                                                          \
  } while (0)
static std::map<std::string, std::vector<uint8_t>> blobs;
static int opens = 0, sets = 0, commits = 0;
static bool fail_open = false, fail_set = false, fail_commit = false,
            fail_read = false;
esp_err_t nvs_open_from_partition(const char *p, const char *ns,
                                  nvs_open_mode_t mode, nvs_handle_t *h) {
  CHECK(!std::strcmp(p, "nvs"));
  CHECK(!std::strcmp(ns, "md_net_v1"));
  ++opens;
  if (fail_open)
    return ESP_FAIL;
  *h = mode == NVS_READONLY ? 1 : 2;
  return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t, const char *key, void *out, size_t *size) {
  if (fail_read)
    return ESP_FAIL;
  auto it = blobs.find(key);
  if (it == blobs.end())
    return ESP_ERR_NVS_NOT_FOUND;
  if (*size < it->second.size())
    return ESP_FAIL;
  *size = it->second.size();
  std::memcpy(out, it->second.data(), *size);
  return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *v,
                       size_t size) {
  CHECK(h == 2);
  CHECK(std::strcmp(key, "active") == 0 || std::strcmp(key, "pending") == 0);
  CHECK(size == kNetworkRecordBytes);
  ++sets;
  if (fail_set)
    return ESP_FAIL;
  const uint8_t *b = static_cast<const uint8_t *>(v);
  blobs[key] = {b, b + size};
  return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) {
  CHECK(h == 2);
  ++commits;
  return fail_commit ? ESP_FAIL : ESP_OK;
}
void nvs_close(nvs_handle_t) {}
NetworkConfig good() {
  NetworkConfig c{};
  std::strcpy(c.sta[0].ssid, "network");
  std::strcpy(c.sta[0].password, "correct-password");
  c.sta[0].enabled = true;
  std::strcpy(c.ap_password, "device-unique-ap");
  c.admin_digest[0] = 1;
  return c;
}
void configuration() {
  auto c = good();
  CHECK(validNetworkConfig(c));
  uint32_t ip;
  CHECK(parseIpv4("192.168.1.53", &ip));
  CHECK(ip == 0x3501a8c0);
  for (auto s : {"256.1.1.1", "1.2.3", "1.2.3.4x", "-1.2.3.4", "1..2.3",
                 "99999.2.3.4", "1.2.3.4.5", ""})
    CHECK(!parseIpv4(s, &ip));
  c.sta[0].dhcp = false;
  CHECK(parseIpv4("192.168.1.53", &c.sta[0].ip));
  CHECK(parseIpv4("255.255.255.0", &c.sta[0].mask));
  CHECK(parseIpv4("192.168.1.1", &c.sta[0].gateway));
  CHECK(validNetworkConfig(c));
  auto bad = c;
  bad.sta[0].mask = 0;
  CHECK(!validNetworkConfig(bad));
  bad = c;
  parseIpv4("255.0.255.0", &bad.sta[0].mask);
  CHECK(!validNetworkConfig(bad));
  for (auto s : {"192.168.1.0", "192.168.1.255", "192.168.4.5", "224.1.1.1",
                 "127.0.0.1"}) {
    bad = c;
    parseIpv4(s, &bad.sta[0].ip);
    CHECK(!validNetworkConfig(bad));
  }
  bad = c;
  bad.bandwidth_mhz = 80;
  CHECK(!validNetworkConfig(bad));
  bad = c;
  bad.ap_timeout_ms = 1;
  CHECK(!validNetworkConfig(bad));
  bad = c;
  std::memset(bad.sta[0].ssid, 'x', 33);
  CHECK(!validNetworkConfig(bad));
  uint8_t record[kNetworkRecordBytes];
  encodeNetworkConfig(c, record);
  NetworkConfig loaded{};
  CHECK(decodeNetworkConfig(record, sizeof(record), &loaded));
  CHECK(std::strcmp(loaded.sta[0].password, c.sta[0].password) == 0);
  CHECK(!decodeNetworkConfig(record, sizeof(record) - 1, &loaded));
  for (size_t i = 0; i < sizeof(record); ++i)
    for (unsigned bit = 0; bit < 8; ++bit) {
      record[i] ^= 1u << bit;
      CHECK(!decodeNetworkConfig(record, sizeof(record), &loaded));
      record[i] ^= 1u << bit;
    }
}
void transactions() {
  NetworkConfigNvs store;
  auto c = good();
  NetworkConfig boot;
  int writes = sets;
  CHECK(!store.load(&boot));
  CHECK(sets == writes);
  CHECK(store.activate(c));
  CHECK(store.load(&boot));
  CHECK(std::strcmp(boot.sta[0].ssid, "network") == 0);
  ConfigTransaction tx;
  tx.begin(&store, c);
  auto next = c;
  std::strcpy(next.sta[0].ssid, "next-network");
  CHECK(!tx.stage(next, 0, 0, false, false));
  CHECK(!tx.stage(next, 0, 0, true, true));
  CHECK(tx.stage(next, 0, 0, true, false));
  CHECK(tx.phase() == ConfigPhase::TESTING);
  CHECK(!tx.stage(c, 0, 0, true, false));
  CHECK(store.load(&boot));
  CHECK(std::strcmp(boot.sta[0].ssid, "network") == 0);
  tx.update(1000, true, false, false);
  CHECK(tx.busy());
  tx.update(30000, false, false, false);
  CHECK(tx.phase() == ConfigPhase::ROLLBACK);
  CHECK(store.load(&boot));
  CHECK(std::strcmp(boot.sta[0].ssid, "network") == 0);
  tx.begin(&store, c);
  CHECK(tx.stage(next, 0, 0, true, false));
  tx.update(2000, true, true, false);
  CHECK(tx.phase() == ConfigPhase::COMMITTED);
  CHECK(store.load(&boot));
  CHECK(std::strcmp(boot.sta[0].ssid, "next-network") == 0);
  fail_commit = true;
  tx.begin(&store, next);
  CHECK(!tx.stage(c, 0, 0, true, false));
  CHECK(tx.phase() == ConfigPhase::STORAGE_ERROR);
  fail_commit = false;
  fail_set = true;
  CHECK(!store.activate(c));
  fail_set = false;
  fail_open = true;
  CHECK(!store.load(&boot));
  CHECK(!store.activate(c));
  fail_open = false;
  fail_read = true;
  CHECK(!store.activate(c));
  fail_read = false;
  tx.begin(&store, c);
  CHECK(tx.stage(next, 0, 0, true, false));
  tx.update(1, true, true, true);
  CHECK(tx.phase() == ConfigPhase::ROLLBACK);
  tx.begin(&store, c);
  CHECK(tx.stage(next, 0, 0xfffffff0u, true, false));
  tx.update(30000, true, true, false);
  CHECK(tx.phase() == ConfigPhase::ROLLBACK);
}
void policies() {
  SleepInputs i;
  CHECK(!modemSleep(i));
  i.session_open = true;
  CHECK(!modemSleep(i));
  i.session_trusted = true;
  CHECK(modemSleep(i));
  i.ota = true;
  CHECK(!modemSleep(i));
  i.ota = false;
  i.provisioning = true;
  CHECK(!modemSleep(i));
  i.provisioning = false;
  i.preference = 2;
  CHECK(!modemSleep(i));
  i.preference = 1;
  i.session_open = false;
  CHECK(!modemSleep(i));
  auto c = good();
  RoamPolicy r;
  CHECK(!r.scan(0, c, true, -90, false));
  c.roam_enabled = true;
  CHECK(!r.scan(0, c, true, -90, true));
  CHECK(!r.scan(0, c, false, -90, false));
  CHECK(!r.scan(0, c, true, -79, false));
  CHECK(r.scan(0, c, true, -90, false));
  CHECK(!r.scan(59999, c, true, -90, false));
  CHECK(r.scan(60000, c, true, -90, false));
  CHECK(!r.roam(60000, c, -85, -80, true, false));
  CHECK(!r.roam(60000, c, -85, -60, false, false));
  CHECK(!r.roam(60000, c, -85, -60, true, true));
  CHECK(r.roam(60000, c, -85, -60, true, false));
  CHECK(!r.scan(120000, c, true, -90, false));
  CHECK(r.scan(180000, c, true, -90, false));
  CHECK(!expireAp(900000, 0, c, false, false, 0));
  CHECK(!expireAp(900000, 0, c, true, true, 0));
  CHECK(!expireAp(900000, 0, c, true, false, 1));
  CHECK(expireAp(900000, 0, c, true, false, 0));
  c.ap_always = true;
  CHECK(!expireAp(900000, 0, c, true, false, 0));
}
void security() {
  CHECK(portalOrigin(true, "192.168.4.1", "http://192.168.4.1"));
  CHECK(!portalOrigin(false, "192.168.4.1", "http://192.168.4.1"));
  CHECK(!portalOrigin(true, "evil.example", "http://192.168.4.1"));
  CHECK(!portalOrigin(true, "192.168.4.1", "http://evil.example"));
  CHECK(!portalOrigin(true, "192.168.4.1", "null"));
  const char *token =
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  uint8_t digest[32], random[64];
  for (unsigned i = 0; i < 64; ++i)
    random[i] = i;
  matdog::update::Sha256 sha;
  sha.update(reinterpret_cast<const uint8_t *>(token), 64);
  sha.finish(digest);
  PortalSession session;
  CHECK(!session.authorize(0, 1, token, token));
  CHECK(!session.login(0, 1, "wrong", digest, random));
  CHECK(!session.login(1, 1, token, digest, random));
  CHECK(session.login(1000, 1, token, digest, random));
  CHECK(session.authorize(1000, 1, session.cookie(), session.csrf()));
  CHECK(!session.authorize(1000, 2, session.cookie(), session.csrf()));
  CHECK(!session.authorize(1000, 1, token, session.csrf()));
  CHECK(!session.authorize(1000, 1, session.cookie(), token));
  CHECK(!session.authorize(301000, 1, session.cookie(), session.csrf()));
  session.reset();
  CHECK(!session.authorize(2000, 1, session.cookie(), session.csrf()));
  ConfigPatch p;
  const std::string valid = "p0_ssid=router&p0_password=abcdefgh&p0_enabled=1&"
                            "p0_dhcp=1&p0_ip=0.0.0.0&p0_mask=0.0.0.0&p0_"
                            "gateway=0.0.0.0&p0_dns=0.0.0.0&test_profile=0";
  CHECK(parseConfigForm(valid.data(), valid.size(), &p));
  CHECK(p.profiles == 1 && p.passwords == 1);
  CHECK(std::strcmp(p.config.sta[0].password, "abcdefgh") == 0);
  for (auto extra :
       {"&p0_ssid=duplicate", "&unknown=1", "&p0_password=%00hidden",
        "&p1_ssid=partial", "&roam_enabled=1", "&test_profile=1",
        "&ap_name=partial"}) {
    std::string b = valid + extra;
    CHECK(!parseConfigForm(b.data(), b.size(), &p));
  }
  CHECK(!parseConfigForm("p0_ssid=%zz", 11, &p));
  CHECK(!parseConfigForm("usb_sleep=99999999999999", 24, &p));
  std::string big(1536, 'x');
  CHECK(!parseConfigForm(big.data(), big.size(), &p));
}
void mailbox() {
  HttpMailbox m;
  m.beginDispatch();
  CHECK(m.abandon());
  CHECK(!m.claimDelivery());
  m.beginDispatch();
  CHECK(m.claimDelivery());
  CHECK(!m.abandon());
  CHECK(!m.claimDelivery());
  for (int i = 0; i < 1000; ++i) {
    m.beginDispatch();
    std::atomic<int> wins{0};
    std::thread a([&] {
      if (m.abandon())
        ++wins;
    });
    std::thread b([&] {
      if (m.claimDelivery())
        ++wins;
    });
    a.join();
    b.join();
    CHECK(wins == 1);
  }
}
int main() {
  configuration();
  transactions();
  policies();
  security();
  mailbox();
  std::printf("test_network_v3: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
