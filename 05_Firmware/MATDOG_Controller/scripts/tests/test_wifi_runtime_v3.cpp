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
esp_err_t esp_wifi_connect() {
  ++WiFi.connect_calls;
  return 0;
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
int main() {
  // The router is joined automatically only in RUN (radio role policy).
  seed(); WifiManager client; client.setRunMode(true); WifiRuntimeTest::initialize(client);
  WifiRuntimeTest::tick(client,100); WifiRuntimeTest::tick(client,250); WifiRuntimeTest::tick(client,400);
  CHECK(WiFi.connect_calls>0 && WiFi.mode_value==WIFI_STA && !WiFi.ap);
  CHECK(WiFi.driver.sta.scan_method==WIFI_ALL_CHANNEL_SCAN);
  CHECK(WiFi.driver.sta.sort_method==WIFI_CONNECT_AP_BY_SIGNAL);
  seed(); WifiManager m; WifiRuntimeTest::initialize(m);
  CHECK(!WiFi.persistent_setting && !WiFi.reconnect);
  CHECK(!m.configure(patch("denied-sta"))); CHECK(!m.setSleep(0));
  enter(m,50); uint32_t generation=m.adminAccess().generation();
  CHECK(m.status().ap_active); CHECK(!WiFi.STA.started());
  CHECK(!m.requestScan()); int scans=WiFi.scan_calls,connects=WiFi.connect_calls;
  for(uint32_t t:{1000u,40000u,700000u}) WifiRuntimeTest::tick(m,t);
  CHECK(WiFi.scan_calls==scans && WiFi.connect_calls==connects);
  CHECK(WiFi.mode_value==WIFI_AP); CHECK(commits==0);
  // One accepted RAM intent; the AP connection closes before PENDING NVS.
  CHECK(m.configure(patch("new-network"),generation));
  WifiRuntimeTest::tick(m,700050); CHECK(commits==0);
  CHECK(m.status().config_phase==ConfigPhase::CONFIG_PENDING);
  CHECK(!m.setEnabled(false,700100)); CHECK(!m.requestAp(false));
  WifiRuntimeTest::tick(m,701100); CHECK(!m.adminAccess().allowed(generation));
  CHECK(WiFi.mode_value==WIFI_AP && commits==0);
  // Failed/not-yet-completed listener stop cannot authorize mode change.
  WifiRuntimeTest::tick(m,702100); CHECK(WiFi.mode_value==WIFI_AP && commits==0);
  closeListeners(m); WifiRuntimeTest::tick(m,702150);
  CHECK(WiFi.mode_value==WIFI_STA && commits==1);
  CHECK(m.status().config_phase==ConfigPhase::TESTING);
  WifiRuntimeTest::tick(m,702300); association("stale");
  WifiRuntimeTest::tick(m,702350); CHECK(commits==1);
  association("new-network"); WifiRuntimeTest::tick(m,702400);
  // Association + address must be HELD: one matching tick is not proof.
  CHECK(commits==1 && m.status().config_phase==ConfigPhase::TESTING);
  WifiRuntimeTest::tick(m,702400+WifiManager::kStaTestHoldMs-1); CHECK(commits==1);
  WifiRuntimeTest::tick(m,702400+WifiManager::kStaTestHoldMs);
  CHECK(commits==2); CHECK(!m.configBusy());
  CHECK(m.status().sta_test.outcome==static_cast<uint8_t>(StaTestOutcome::CONNECTED));
  CHECK(m.status().sta_test.ipv4==0x3501a8c0 && !m.status().sta_test.rolled_back);
  CHECK(m.status().config_phase==ConfigPhase::COMMITTED);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  closeListeners(m); WifiRuntimeTest::tick(m,704450);
  CHECK(m.adminAccess().generation()!=generation);
  CHECK(!m.adminAccess().allowed(generation));
  CHECK(m.status().config_phase==ConfigPhase::COMMITTED);
  NetworkConfig c{}; NetworkConfigNvs store; CHECK(store.load(&c));
  CHECK(strcmp(c.sta[0].ssid,"new-network")==0);
  // Queue accepted under old generation then revoked before worker mutation.
  int writes=sets; generation=m.adminAccess().generation();
  CHECK(m.configure(patch("queued-stale"),generation)); m.adminAccess().revoke();
  WifiRuntimeTest::tick(m,704500); CHECK(sets==writes); CHECK(!m.configBusy());
  // Explicit re-entry is mandatory; no spontaneous reopen on radio start.
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP,{}); WifiRuntimeTest::tick(m,704550);
  CHECK(!m.adminAccess().allowed(generation));
  WiFi.ap=true; WiFi.AP.running=true; WifiRuntimeTest::tick(m,704600);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::FAULT);
  enter(m,704650); CHECK(m.adminAccess().generation()!=generation);
  // Unknown driver state/APSTA deny, and a consumed request never persists.
  CHECK(m.configure(patch("unknown"))); WiFi.mode_read_ok=false;
  WifiRuntimeTest::tick(m,704750); CHECK(sets==writes);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::FAULT);
  WiFi.mode_read_ok=true; enter(m,704800);
  CHECK(m.configure(patch("apsta"))); WiFi.mode_value=WIFI_AP_STA; WiFi.STA.running=true;
  CHECK(!WifiManager::verifyAdminRadio()); WifiRuntimeTest::tick(m,704900);
  CHECK(sets==writes); enter(m,704950);
  // Rollback test, then fresh ADMIN generation with old ACTIVE intact.
  CHECK(m.configure(patch("wrong-password"))); WifiRuntimeTest::tick(m,705050);
  WifiRuntimeTest::tick(m,706100); closeListeners(m); WifiRuntimeTest::tick(m,706150);
  WifiRuntimeTest::tick(m,706300); WiFi.connected=false;
  WifiRuntimeTest::tick(m,736200); CHECK(m.status().config_phase==ConfigPhase::ROLLBACK);
  CHECK(store.load(&c) && strcmp(c.sta[0].ssid,"new-network")==0);
  closeListeners(m); WifiRuntimeTest::tick(m,736250);
  // In MAINTENANCE the AP is mandatory; RUN with a router profile leaves it,
  // and the STA scan remains operational there.
  CHECK(!m.requestAp(false)); CHECK(m.lastPost()==PostResult::INVALID);
  m.setRunMode(true); WifiRuntimeTest::tick(m,736300);
  closeListeners(m); WifiRuntimeTest::tick(m,736350);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::CLIENT_STA);
  CHECK(m.requestScan()); WifiRuntimeTest::tick(m,736400); CHECK(m.status().scan_running);
  m.setContext(true,true); WifiRuntimeTest::tick(m,736450);
  CHECK(!m.status().scan_running && !m.status().sleep_effective);
  m.setContext(false,false);
  CHECK(m.setEnabled(false,736500)); WifiRuntimeTest::tick(m,736500);
  closeListeners(m); WifiRuntimeTest::tick(m,736550);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::CLOSED && WiFi.mode_value==WIFI_OFF);
  CHECK(m.provisionAp("bootstrap-replacement-key")); WifiRuntimeTest::tick(m,736600);
  CHECK(m.status().config_phase==ConfigPhase::COMMITTED);
  CHECK(m.setEnabled(true,736650)); WifiRuntimeTest::tick(m,736650);
  closeListeners(m); WifiRuntimeTest::tick(m,736700);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::CLIENT_STA);
  CHECK(!m.provisionAp("refused-in-client-mode"));
  // Reboot loads ACTIVE, never promotes PENDING or writes.
  WifiManager rebooted; writes=sets; WifiRuntimeTest::initialize(rebooted);
  CHECK(sets==writes && strcmp(rebooted.status().profiles[0].ssid,"new-network")==0);
  seed(); WifiManager observed; WifiRuntimeTest::initialize(observed);
  enter(observed,100); auto observed_generation=observed.adminAccess().generation();
  CHECK(observed.adminAccess().allowed(observed_generation));
  CHECK(observed.configure(patch("revoked-by-event")));
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP,{});
  CHECK(!observed.adminAccess().allowed(observed_generation));
  CHECK(!observed.configure(patch("post-event")));
  WifiRuntimeTest::tick(observed,200);
  CHECK(observed.adminAccess().phase.load()==NetworkPhase::FAULT && commits==0);
  WiFi.AP.running=true; WifiRuntimeTest::tick(observed,250);
  CHECK(!observed.adminAccess().allowed(observed_generation));
  printf("test_wifi_runtime_v3 AP-only: %d checks, %d failures (radio/NVS doubles)\n",checks,failures);
  return failures?1:0;
}
