// Radio role by operating mode: MAINTENANCE keeps the administrative AP, RUN
// joins the configured router and falls back to a recovery AP. Production
// WifiManager, WifiPolicy, NetworkPolicy, NetworkConfig and NVS owner; the
// Wi-Fi driver, its events, the scheduler, the HTTP lifecycle task and NVS are
// doubles. No radio proof: association, WPA2 and DHCP are hardware checks.
// The mode is an input here exactly as in production (setRunMode); nothing in
// this file, and nothing in WifiManager, can change the controller's mode.
#include "../../src/network/WifiManager.h"
#include <WiFi.h>
#include <cstdio>
#include <cstring>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
using namespace matdog::network;
int checks = 0, failures = 0;
uint32_t now = 0;
#define CHECK(x)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(x)) {                                                                \
      ++failures;                                                              \
      std::printf("FAIL %d: %s\n", __LINE__, #x);                              \
    }                                                                          \
  } while (0)
WifiStub WiFi;
wifi_ps_type_t ps = WIFI_PS_NONE;
wifi_bandwidth_t bw = WIFI_BW_HT20;
uint32_t millis() { return now; }
uint32_t micros() { return now * 1000; }
struct Q {
  bool full = false;
  std::vector<uint8_t> bytes;
};
static std::vector<std::unique_ptr<Q>> queues;
QueueHandle_t xQueueCreate(unsigned n, size_t size) {
  CHECK(n == 1);
  queues.emplace_back(new Q);
  queues.back()->bytes.resize(size);
  return queues.back().get();
}
BaseType_t xQueueSend(QueueHandle_t h, const void *b, TickType_t t) {
  CHECK(t == 0);
  auto *q = static_cast<Q *>(h);
  if (q->full)
    return 0;
  std::memcpy(q->bytes.data(), b, q->bytes.size());
  q->full = true;
  return 1;
}
BaseType_t xQueueReceive(QueueHandle_t h, void *b, TickType_t t) {
  CHECK(t == 0);
  auto *q = static_cast<Q *>(h);
  if (!q->full)
    return 0;
  std::memcpy(b, q->bytes.data(), q->bytes.size());
  q->full = false;
  return 1;
}
SemaphoreHandle_t xSemaphoreCreateMutex() {
  static std::vector<std::unique_ptr<std::mutex>> locks;
  locks.emplace_back(new std::mutex); return locks.back().get();
}
void vQueueDelete(QueueHandle_t) {}
void vSemaphoreDelete(SemaphoreHandle_t) {}
BaseType_t xSemaphoreTake(SemaphoreHandle_t h, TickType_t t) {
  CHECK(t == 0); return static_cast<std::mutex*>(h)->try_lock();
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t h) { static_cast<std::mutex*>(h)->unlock(); return 1; }
BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *,
                                   uint32_t stack, void *, unsigned, void *,
                                   int core) {
  CHECK(stack == 8192 && core == 0);
  return 1;
}
void vTaskDelay(TickType_t) {}
unsigned uxTaskGetStackHighWaterMark(void *) { return 4096; }
esp_err_t esp_wifi_get_mode(wifi_mode_t *mode) {
  if (!WiFi.mode_read_ok) return -1;
  *mode = static_cast<wifi_mode_t>(WiFi.mode_value); return ESP_OK;
}
esp_err_t esp_wifi_get_mac(wifi_interface_t, uint8_t *m) {
  uint8_t mac[6] = {0x14, 0xc1, 0x9f, 0x22, 0x75, 0x94};
  std::memcpy(m, mac, 6);
  return 0;
}
esp_err_t esp_wifi_set_config(wifi_interface_t i, const wifi_config_t *c) {
  if (i == WIFI_IF_AP) return WiFi.setApConfig(c);
  CHECK(!c->sta.bssid_set);
  WiFi.driver = *c;
  return 0;
}
esp_err_t esp_wifi_get_config(wifi_interface_t i, wifi_config_t *c) {
  if (i == WIFI_IF_AP) return WiFi.getApConfig(c);
  *c = WiFi.driver;
  return 0;
}
static bool connect_ok = true; // false: the driver refuses the connect call
esp_err_t esp_wifi_connect() {
  ++WiFi.connect_calls;
  return connect_ok ? 0 : -1;
}
esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *r) {
  if (!WiFi.STA.running) return ESP_ERR_WIFI_CONN;
  if (!WiFi.connected) return ESP_ERR_WIFI_NOT_CONNECT;
  std::memcpy(r->ssid, WiFi.associated_ssid.c_str(),
              WiFi.associated_ssid.size());
  uint8_t mac[6] = {1, 2, 3, 4, 5, 7};
  std::memcpy(r->bssid, mac, 6);
  return 0;
}
esp_err_t esp_wifi_scan_stop() {
  WiFi.scan_state = -2;
  return 0;
}
esp_err_t esp_wifi_set_ps(wifi_ps_type_t v) {
  ps = v;
  return 0;
}
esp_err_t esp_wifi_get_ps(wifi_ps_type_t *v) {
  *v = ps;
  return 0;
}
esp_err_t esp_wifi_set_bandwidth(wifi_interface_t, wifi_bandwidth_t v) {
  bw = v;
  return 0;
}
esp_err_t esp_wifi_get_bandwidth(wifi_interface_t, wifi_bandwidth_t *v) {
  *v = bw;
  return 0;
}
static std::map<std::string, std::vector<uint8_t>> blobs;
static int sets = 0, commits = 0;
esp_err_t nvs_open_from_partition(const char *p, const char *ns,
                                  nvs_open_mode_t, nvs_handle_t *h) {
  CHECK(std::strcmp(p, "nvs") == 0);
  CHECK(std::strcmp(ns, "md_net_v1") == 0);
  *h = 1;
  return 0;
}
esp_err_t nvs_get_blob(nvs_handle_t, const char *k, void *b, size_t *n) {
  auto i = blobs.find(k);
  if (i == blobs.end())
    return ESP_ERR_NVS_NOT_FOUND;
  if (*n < i->second.size())
    return -1;
  *n = i->second.size();
  std::memcpy(b, i->second.data(), *n);
  return 0;
}
esp_err_t nvs_set_blob(nvs_handle_t, const char *k, const void *b, size_t n) {
  const uint8_t *bytes = static_cast<const uint8_t *>(b);
  blobs[k] = {bytes, bytes + n};
  ++sets;
  return 0;
}
esp_err_t nvs_commit(nvs_handle_t) {
  ++commits;
  return 0;
}
void nvs_close(nvs_handle_t) {}
namespace matdog {
namespace network {
class WifiRuntimeTest {
public:
  static void initialize(WifiManager &m) {
    m.begin(now);
    m.initializeWorker();
    m.workerTick(now);
    publish(m);
  }
  static void tick(WifiManager &m, uint32_t t) {
    now = t;
    m.workerTick(t);
    publish(m);
  }
  static void publish(WifiManager &m) {
    m.shared_status_ = m.worker_status_;
    std::memcpy(m.shared_admin_digest_, m.config_.admin_digest, 32);
    m.update(now);
  }
};
} // namespace network
} // namespace matdog
void seed() {
  blobs.clear();
  WiFi = WifiStub{};
  NetworkConfig c{};
  std::strcpy(c.sta[0].ssid, "network");
  std::strcpy(c.sta[0].password, "correct-password");
  c.sta[0].enabled = true;
  std::strcpy(c.ap_password, "unique-device-key");
  c.admin_digest[0] = 1;
  NetworkConfigNvs store;
  CHECK(store.activate(c));
  sets = commits = 0;
  now = 0;
}
ConfigPatch patch(const char *ssid) {
  ConfigPatch p{};
  p.profiles = p.passwords = 1;
  p.test_profile = 0;
  p.config.sta[0].enabled = true;
  std::strcpy(p.config.sta[0].ssid, ssid);
  std::strcpy(p.config.sta[0].password, "new-password");
  return p;
}
void association(const char *ssid) {
  WiFi.connected = true;
  WiFi.ip = 0x3501a8c0;
  WiFi.associated_ssid = ssid;
}

// AP-only replaces the earlier AP+STA browser-continuity contract. Driver,
// events, task scheduling and NVS are doubles; all manager code is production.
void closeListeners(WifiManager &m) {
  auto &g=m.adminAccess(); AdminLease drain(g); CHECK(bool(drain));
  CHECK(g.closing.load()); g.listeners_closed.store(true);
}
void enter(WifiManager &m,uint32_t t) {
  CHECK(m.requestAp(true)); WifiRuntimeTest::tick(m,t);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  closeListeners(m); WifiRuntimeTest::tick(m,t+50);
  CHECK(m.adminAccess().allowed(m.adminAccess().generation()));
  CHECK(WifiManager::verifyAdminRadio()); CHECK(WiFi.mode_value==WIFI_AP);
}

using Phase = NetworkPhase;
Phase phase(WifiManager &m) { return m.adminAccess().phase.load(); }
RoleReason role(const WifiManager &m) { return static_cast<RoleReason>(m.status().role_reason); }
// What md_http_life does for the worker: acknowledge listener closure once a
// closure was requested and no lease is held.
void lifecycle(WifiManager &m) {
  auto &g = m.adminAccess();
  AdminLease drain(g);
  if (drain && g.closing.load()) g.listeners_closed.store(true);
}
uint32_t clock_ms = 0;
void advance(WifiManager &m, uint32_t duration, uint32_t step = 50) {
  for (const uint32_t end = clock_ms + duration; clock_ms < end;) {
    clock_ms += step;
    WifiRuntimeTest::tick(m, clock_ms);
    lifecycle(m);
  }
}
// Runs until the phase is reached; returns the elapsed time, or UINT32_MAX.
uint32_t until(WifiManager &m, Phase wanted, uint32_t limit) {
  const uint32_t start = clock_ms;
  while (clock_ms - start < limit) {
    advance(m, 50);
    if (phase(m) == wanted) return clock_ms - start;
  }
  return UINT32_MAX;
}
void sta(int event, uint16_t reason = 0) {
  WiFiEventInfo_t info{};
  info.wifi_sta_disconnected.reason = reason;
  WiFi.callback(event, info);
}
void store(bool primary, bool secondary, bool ap_always = false, bool ap_key = true) {
  blobs.clear();
  WiFi = WifiStub{};
  NetworkConfig c{};
  std::strcpy(c.sta[0].ssid, "home-router");
  std::strcpy(c.sta[0].password, "correct-password");
  c.sta[0].enabled = primary;
  std::strcpy(c.sta[1].ssid, "backup-router");
  std::strcpy(c.sta[1].password, "backup-password");
  c.sta[1].enabled = secondary;
  if (ap_key) std::strcpy(c.ap_password, "unique-device-key");
  c.admin_digest[0] = 1;
  c.ap_always = ap_always;
  NetworkConfigNvs nvs;
  CHECK(nvs.activate(c));
  sets = commits = 0;
  now = clock_ms = 0;
}
std::string driverSsid() {
  return std::string(reinterpret_cast<const char *>(WiFi.driver.sta.ssid),
                     strnlen(reinterpret_cast<const char *>(WiFi.driver.sta.ssid), 32));
}
bool verifiedAp(WifiManager &m) {
  return phase(m) == Phase::ADMIN_AP && m.adminAccess().allowed(m.adminAccess().generation()) &&
         WifiManager::verifyAdminRadio() && WiFi.mode_value == WIFI_AP && m.status().ap_active &&
         m.status().ap_config_verified && WiFi.ap_key == "unique-device-key" &&
         WiFi.ap_profile.ap.authmode == WIFI_AUTH_WPA2_PSK;
}
bool clean(WifiManager &m) { // no radio fault, no error=11
  return m.status().config_error == 0 && m.status().fault_count == 0 && phase(m) != Phase::FAULT;
}
int main() {
  // ---- the pure decision ---------------------------------------------------------
  using R = RoleReason;
  for (int bits = 0; bits < 16; ++bits) {
    RoleInputs in;
    in.run = bits & 1; in.sta_profile = bits & 2; in.ap_always = bits & 4; in.recovery = bits & 8;
    const R r = selectRole(in);
    if (!in.run) CHECK(r == R::MAINTENANCE_AP);          // maintenance outranks every preference
    else if (in.ap_always) CHECK(r == R::RUN_FORCED_AP);
    else if (!in.sta_profile) CHECK(r == R::RUN_NO_PROFILE_AP);
    else CHECK(r == (in.recovery ? R::RUN_RECOVERY_AP : R::RUN_STA));
    CHECK(roleIsSta(r) == (in.run && in.sta_profile && !in.ap_always && !in.recovery));
  }
  CHECK(recoveryDwellMs(0, 900000) == 60000 && recoveryDwellMs(1, 900000) == 120000);
  CHECK(recoveryDwellMs(3, 900000) == 480000 && recoveryDwellMs(4, 900000) == 900000);
  CHECK(recoveryDwellMs(200, 900000) == 900000 && recoveryDwellMs(5, 60000) == 60000);
  std::map<std::string, int> names;
  for (int i = 0; i <= 4; ++i) ++names[toString(static_cast<R>(i))];
  CHECK(names.size() == 5 && !names.count("UNKNOWN"));

  // ---- 1. boot in MAINTENANCE with a router already configured -> AP ---------------
  {
    store(true, true); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
    CHECK(role(m) == R::MAINTENANCE_AP && !m.status().run_mode);
    CHECK(m.status().credentials_present); // the profile is stored, and not used
    const uint32_t generation = m.adminAccess().generation();
    advance(m, 3 * 3600 * 1000u, 500); // three hours, nobody connected
    CHECK(WiFi.connect_calls == 0 && WiFi.disconnect_calls == 0 && !m.status().connected);
    CHECK(verifiedAp(m) && m.adminAccess().generation() == generation); // no idle timeout
    CHECK(clean(m));
    CHECK(!m.requestAp(false) && m.lastPost() == PostResult::INVALID); // AP is mandatory here
  }

  // ---- 2. MAINTENANCE without a router -> AP ------------------------------------------
  {
    store(false, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
    const uint32_t generation = m.adminAccess().generation();
    advance(m, 3600 * 1000u, 500);
    CHECK(verifiedAp(m) && m.adminAccess().generation() == generation && WiFi.connect_calls == 0);
    CHECK(role(m) == R::MAINTENANCE_AP);
  }

  {
    // No AP key provisioned yet: there is no AP to open, and MAINTENANCE still
    // does not join the stored router on its own.
    store(true, false, false, false); WifiManager m; WifiRuntimeTest::initialize(m);
    advance(m, 600000, 500);
    CHECK(phase(m) == Phase::CLIENT_STA && WiFi.mode_value == WIFI_STA && !WiFi.ap);
    CHECK(WiFi.connect_calls == 0 && !m.status().connected && role(m) == R::MAINTENANCE_AP && clean(m));
    m.setRunMode(true); advance(m, 1000);
    CHECK(WiFi.connect_calls == 1 && role(m) == R::RUN_STA);
  }

  // ---- 3. RUN with valid credentials -> router, and it stays there -------------------
  {
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    const uint32_t generation = m.adminAccess().generation();
    WiFi.clients = 1; // an administrator is connected: the mode change still decides
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    WiFi.clients = 0;
    CHECK(!m.adminAccess().allowed(generation)); // the portal closed with the AP
    CHECK(WiFi.mode_value == WIFI_STA && !WiFi.AP.started());
    advance(m, 500);
    CHECK(WiFi.connect_calls == 1 && driverSsid() == "home-router");
    CHECK(role(m) == R::RUN_STA && m.status().run_mode && !m.status().connected);
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED); WiFi.connected = true; WiFi.ip = 0;
    advance(m, 1000);
    CHECK(!m.status().connected && m.status().state == WifiState::CONNECTING); // no address, not connected
    association("home-router");
    advance(m, 200);
    CHECK(m.status().connected && m.status().state == WifiState::CONNECTED && m.status().ipv4 == 0x3501a8c0);
    advance(m, 3600 * 1000u, 500);
    CHECK(phase(m) == Phase::CLIENT_STA && m.status().connected && WiFi.connect_calls == 1);
    CHECK(m.adminAccess().generation() == generation && clean(m)); // no AP generation was opened
    CHECK(sets == 0); // the role policy never writes configuration
  }

  // ---- 4. RUN without credentials -> recovery AP, no bouncing -------------------------
  {
    store(false, false); WifiManager m; m.setRunMode(true); WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m));
    CHECK(role(m) == R::RUN_NO_PROFILE_AP && m.status().run_mode);
    const uint32_t generation = m.adminAccess().generation();
    advance(m, 3 * 3600 * 1000u, 500);
    CHECK(verifiedAp(m) && m.adminAccess().generation() == generation && WiFi.connect_calls == 0);
    CHECK(clean(m));
  }
  {
    // "AP always" is honoured in RUN as well: the stored router is not joined.
    store(true, false, true); WifiManager m; m.setRunMode(true); WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && role(m) == R::RUN_FORCED_AP);
    advance(m, 3600 * 1000u, 500);
    CHECK(verifiedAp(m) && WiFi.connect_calls == 0 && !m.requestAp(false));
  }

  // ---- 5/6. RUN, router absent or password refused -> bounded tries, recovery AP ------
  for (uint16_t reason : {201, 15}) {
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    bool ever_connected = false;
    uint32_t elapsed = 0;
    int last_calls = 0;
    while (phase(m) == Phase::CLIENT_STA && elapsed < 120000) {
      advance(m, 50); elapsed += 50;
      if (WiFi.connect_calls != last_calls) { last_calls = WiFi.connect_calls; sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, reason); }
      ever_connected = ever_connected || m.status().connected || m.status().state == WifiState::CONNECTED;
    }
    CHECK(!ever_connected && m.status().counters.connects == 0); // never a false CONNECTED
    CHECK(WiFi.connect_calls == static_cast<int>(WifiManager::kStaEpisodeAttempts));
    CHECK(elapsed >= 2 * WifiManager::kConnectTimeoutMs && elapsed < 2 * WifiManager::kConnectTimeoutMs + 5000);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
    CHECK(role(m) == R::RUN_RECOVERY_AP && m.status().recovery_retry_ms > 0);
    CHECK(m.status().disconnect_reason == reason && m.status().fault == WifiFault::CONNECT_TIMEOUT);
    // The router is tried again after 60 s, then 120, 240, 480 s, and never more
    // often than the configured AP timeout (900 s): no AP/STA oscillation.
    const uint32_t expected[] = {60000, 120000, 240000, 480000, 900000, 900000};
    for (unsigned cycle = 0; cycle < 6; ++cycle) {
      const uint32_t entered = clock_ms;
      CHECK(m.status().recovery_level == cycle);
      advance(m, expected[cycle] - 5000);
      CHECK(verifiedAp(m)); // still serving the recovery portal
      const uint32_t left = until(m, Phase::CLIENT_STA, 10000);
      CHECK(left != UINT32_MAX);
      const uint32_t dwell = clock_ms - entered;
      CHECK(dwell >= expected[cycle] - 2000 && dwell <= expected[cycle] + 2000);
      const int calls = WiFi.connect_calls;
      uint32_t episode = 0;
      while (phase(m) == Phase::CLIENT_STA && episode < 120000) {
        advance(m, 50); episode += 50;
        ever_connected = ever_connected || m.status().connected;
      }
      CHECK(WiFi.connect_calls == calls + static_cast<int>(WifiManager::kStaEpisodeAttempts));
      CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
    }
    CHECK(!ever_connected && m.status().counters.connects == 0);
    // Somebody is using the recovery portal: it is not taken away from them.
    WiFi.clients = 1;
    const int calls = WiFi.connect_calls;
    advance(m, 3600 * 1000u, 500);
    CHECK(verifiedAp(m) && WiFi.connect_calls == calls);
    WiFi.clients = 0;
    CHECK(until(m, Phase::CLIENT_STA, 5000) != UINT32_MAX);
    CHECK(sets == 0);
  }

  // ---- 7. RUN, connection lost -> retry, fallback profile, recovery, return -----------
  {
    store(true, true); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    advance(m, 500); CHECK(driverSsid() == "home-router");
    association("home-router"); advance(m, 5 * 60000);
    CHECK(m.status().connected && WiFi.connect_calls == 1);
    // The link drops and the router answers again on the fallback profile.
    WiFi.connected = false; WiFi.ip = 0; sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 200);
    advance(m, 200);
    CHECK(!m.status().connected && m.status().fault == WifiFault::LINK_LOST && m.status().counters.link_losses == 1);
    advance(m, 3000);
    CHECK(WiFi.connect_calls == 2 && driverSsid() == "home-router"); // primary first
    advance(m, WifiManager::kConnectTimeoutMs + 5000);
    CHECK(WiFi.connect_calls == 3 && driverSsid() == "backup-router"); // then the fallback
    CHECK(phase(m) == Phase::CLIENT_STA);
    association("backup-router"); advance(m, 500);
    CHECK(m.status().connected && m.status().active_profile == 1 && phase(m) == Phase::CLIENT_STA);
    CHECK(role(m) == R::RUN_STA && clean(m));
    // Lost for good: both profiles fail once, then the recovery AP.
    advance(m, 5 * 60000);
    WiFi.connected = false; WiFi.ip = 0; sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 200);
    const int calls = WiFi.connect_calls;
    CHECK(until(m, Phase::ADMIN_AP, 60000) != UINT32_MAX && verifiedAp(m));
    CHECK(WiFi.connect_calls == calls + 2 && role(m) == R::RUN_RECOVERY_AP && clean(m));
    // The router comes back: after the wait the controller rejoins it and stays.
    CHECK(until(m, Phase::CLIENT_STA, 70000) != UINT32_MAX);
    advance(m, 500); association("home-router"); advance(m, 500);
    CHECK(m.status().connected && role(m) == R::RUN_STA);
    CHECK(m.status().recovery_level == 1); // not forgotten at once...
    advance(m, WifiManager::kStaStableMs + 1000);
    CHECK(m.status().recovery_level == 0); // ...only after the link proved stable
    advance(m, 3600 * 1000u, 500);
    CHECK(phase(m) == Phase::CLIENT_STA && m.status().connected && clean(m));
    // A link that connects and drops at once must not reset the wait: flapping
    // router, growing waits.
    WiFi.connected = false; WiFi.ip = 0;
    CHECK(until(m, Phase::ADMIN_AP, 60000) != UINT32_MAX);
    CHECK(until(m, Phase::CLIENT_STA, 70000) != UINT32_MAX);
    advance(m, 500); association("home-router"); advance(m, 2000);
    WiFi.connected = false; WiFi.ip = 0;
    CHECK(until(m, Phase::ADMIN_AP, 60000) != UINT32_MAX);
    CHECK(m.status().recovery_level == 1);
    advance(m, 100000); CHECK(verifiedAp(m)); // 120 s wait now, not 60
    CHECK(until(m, Phase::CLIENT_STA, 30000) != UINT32_MAX);
  }

  // ---- 8. back to MAINTENANCE -> verified AP, from every RUN situation ----------------
  {
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    advance(m, 500); association("home-router"); advance(m, 60000);
    CHECK(m.status().connected);
    const uint32_t old = m.adminAccess().generation();
    m.setRunMode(false); // connected to the router -> maintenance
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
    CHECK(!WiFi.STA.started() && !m.status().connected && m.status().ipv4 == 0);
    CHECK(m.adminAccess().generation() != old && role(m) == R::MAINTENANCE_AP);
    const int calls = WiFi.connect_calls;
    advance(m, 3600 * 1000u, 500);
    CHECK(verifiedAp(m) && WiFi.connect_calls == calls); // and it does not go back by itself
    // Mid-attempt -> maintenance.
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    WiFi.connected = false; WiFi.ip = 0;
    advance(m, 5000); CHECK(m.status().state == WifiState::CONNECTING);
    m.setRunMode(false);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
    // Recovery AP -> maintenance: same AP, no bounce, and the wait is forgotten.
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    CHECK(until(m, Phase::ADMIN_AP, 60000) != UINT32_MAX && role(m) == R::RUN_RECOVERY_AP);
    const uint32_t recovery_generation = m.adminAccess().generation();
    m.setRunMode(false); advance(m, 200);
    CHECK(verifiedAp(m) && m.adminAccess().generation() == recovery_generation);
    CHECK(role(m) == R::MAINTENANCE_AP && m.status().recovery_retry_ms == 0 && m.status().recovery_level == 0);
    advance(m, 3600 * 1000u, 500);
    CHECK(verifiedAp(m) && m.adminAccess().generation() == recovery_generation);
  }

  // ---- 9/10. manual router test in MAINTENANCE -> AP; reboot; then RUN uses it ---------
  {
    store(false, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    const uint32_t generation = m.adminAccess().generation();
    CHECK(m.configure(patch("tested-router"), generation));
    CHECK(until(m, Phase::CLIENT_STA, 5000) != UINT32_MAX && commits == 1);
    CHECK(role(m) == R::MAINTENANCE_AP); // a transaction, not a role change
    advance(m, 500); CHECK(WiFi.connect_calls == 1 && driverSsid() == "tested-router");
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED); association("tested-router");
    CHECK(until(m, Phase::ADMIN_AP, 10000) != UINT32_MAX && verifiedAp(m) && clean(m));
    CHECK(m.status().config_phase == ConfigPhase::COMMITTED && commits == 2);
    CHECK(static_cast<StaTestOutcome>(m.status().sta_test.outcome) == StaTestOutcome::CONNECTED);
    const int calls = WiFi.connect_calls;
    const uint32_t after = m.adminAccess().generation();
    advance(m, 3600 * 1000u, 500);
    // The saved router is NOT joined while in MAINTENANCE.
    CHECK(verifiedAp(m) && WiFi.connect_calls == calls && m.adminAccess().generation() == after);
    // A failed test also ends on the AP, with the saved profile intact.
    CHECK(m.configure(patch("wrong-router"), after));
    CHECK(until(m, Phase::CLIENT_STA, 5000) != UINT32_MAX);
    WiFi.connected = false; WiFi.ip = 0; advance(m, 500); sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 15);
    CHECK(until(m, Phase::ADMIN_AP, 40000) != UINT32_MAX && verifiedAp(m));
    CHECK(m.status().config_phase == ConfigPhase::ROLLBACK);
    CHECK(std::strcmp(m.status().profiles[0].ssid, "tested-router") == 0);
    // Reboot: the persistent configuration is loaded, the mode is MAINTENANCE.
    WiFi = WifiStub{}; now = clock_ms = 0;
    const int writes = sets;
    WifiManager rebooted; WifiRuntimeTest::initialize(rebooted);
    CHECK(until(rebooted, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(rebooted) && clean(rebooted));
    CHECK(std::strcmp(rebooted.status().profiles[0].ssid, "tested-router") == 0 && rebooted.status().nvs_active);
    advance(rebooted, 600000, 500);
    CHECK(WiFi.connect_calls == 0 && sets == writes && role(rebooted) == R::MAINTENANCE_AP);
    rebooted.setRunMode(true);
    CHECK(until(rebooted, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    advance(rebooted, 500);
    CHECK(WiFi.connect_calls == 1 && driverSsid() == "tested-router" && sets == writes);
  }
  {
    // A mode change in the middle of a test: the test is abandoned (critical
    // context, as the Controller reports it in RUN), nothing is saved.
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    CHECK(m.configure(patch("other-router"), m.adminAccess().generation()));
    CHECK(until(m, Phase::CLIENT_STA, 5000) != UINT32_MAX);
    advance(m, 1000);
    m.setContext(true, false); m.setRunMode(true);
    advance(m, 200);
    CHECK(m.status().config_phase == ConfigPhase::ROLLBACK && !m.configBusy());
    CHECK(static_cast<StaTestOutcome>(m.status().sta_test.outcome) == StaTestOutcome::ABORTED);
    CHECK(until(m, Phase::CLIENT_STA, 5000) != UINT32_MAX);
    advance(m, 500);
    CHECK(driverSsid() == "home-router" && commits == 1); // the known-good profile, only PENDING written
    NetworkConfig c{}; NetworkConfigNvs nvs; CHECK(nvs.load(&c) && std::strcmp(c.sta[0].ssid, "home-router") == 0);
  }

  // ---- 11. role events across mode changes: no fault, no error=11 ----------------------
  {
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    CHECK(WiFi.ap_set_calls == 1); // the profile write that restarts the AP (AP_STOP, AP_START)
    for (int cycle = 0; cycle < 5; ++cycle) {
      m.setRunMode(true);
      CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX && clean(m));
      advance(m, 500); association("home-router"); advance(m, 2000);
      CHECK(m.status().connected && clean(m));
      m.setRunMode(false);
      CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && clean(m));
      advance(m, 2000);
      CHECK(verifiedAp(m) && clean(m));
    }
    CHECK(m.status().fault_count == 0 && m.status().last_fault_site == 0 && m.status().config_error == 0);
    // Same with the role events delivered late, as the Arduino queue does.
    WiFi.deliver_events = false;
    m.setRunMode(true);
    for (int i = 0; i < 40 && phase(m) != Phase::CLIENT_STA; ++i) { advance(m, 50); WiFi.drainEvents(); }
    CHECK(phase(m) == Phase::CLIENT_STA && clean(m));
    m.setRunMode(false);
    for (int i = 0; i < 40 && phase(m) != Phase::ADMIN_AP; ++i) { advance(m, 50); WiFi.drainEvents(); }
    CHECK(verifiedAp(m) && clean(m));
    WiFi.deliver_events = true;
  }
  // A radio error is never turned into a working state by the role policy.
  for (bool run : {false, true}) {
    store(false, false); WifiManager m; m.setRunMode(run); WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP, {}); // not part of any command
    advance(m, 200);
    CHECK(phase(m) == Phase::FAULT && m.status().config_error == 11 && m.status().fault_count == 1);
    CHECK(!m.status().ap_active && !m.adminAccess().allowed(m.adminAccess().generation()));
    advance(m, 3600 * 1000u, 500);
    CHECK(phase(m) == Phase::FAULT && WiFi.mode_value == WIFI_OFF); // closed, and it stays closed
    m.setRunMode(!run); advance(m, 5000);
    CHECK(phase(m) == Phase::FAULT); // a mode change is not an explicit recovery request
    m.setRunMode(false);
    CHECK(m.requestAp(true)); // the explicit request is
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && m.status().config_error == 0);
  }
  {
    // The driver refuses the change to STA: FAULT, not "connected".
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    WiFi.mode_write_ok = false;
    m.setRunMode(true); advance(m, 2000);
    CHECK(phase(m) == Phase::FAULT && !m.status().connected && m.status().fault_count == 1);
    CHECK(m.status().last_fault_site == static_cast<uint8_t>(FaultSite::MODE_COMMAND));
    WiFi.mode_write_ok = true;
  }
  {
    // The driver refuses the connect call itself: reported, then the recovery AP.
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    connect_ok = false;
    m.setRunMode(true);
    CHECK(until(m, Phase::CLIENT_STA, 3000) != UINT32_MAX);
    advance(m, 1000);
    CHECK(m.status().fault == WifiFault::CONNECT_CALL_REJECTED && !m.status().connected);
    CHECK(until(m, Phase::ADMIN_AP, 20000) != UINT32_MAX && role(m) == R::RUN_RECOVERY_AP);
    CHECK(m.status().counters.connects == 0);
    connect_ok = true;
  }

  // ---- radio switched off by the operator stays off, in either mode ----------------------
  {
    store(true, false); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX);
    CHECK(m.setEnabled(false, clock_ms));
    CHECK(until(m, Phase::CLOSED, 3000) != UINT32_MAX && WiFi.mode_value == WIFI_OFF);
    m.setRunMode(true); advance(m, 60000);
    CHECK(phase(m) == Phase::CLOSED && WiFi.connect_calls == 0);
    m.setRunMode(false); advance(m, 60000);
    CHECK(phase(m) == Phase::CLOSED);
    CHECK(m.setEnabled(true, clock_ms));
    CHECK(until(m, Phase::ADMIN_AP, 3000) != UINT32_MAX && verifiedAp(m) && WiFi.connect_calls == 0);
  }
  printf("WIFI_MODE_POLICY: %d checks, %d failures (driver/NVS/lifecycle doubles; no radio proof)\n", checks, failures);
  return failures ? 1 : 0;
}
