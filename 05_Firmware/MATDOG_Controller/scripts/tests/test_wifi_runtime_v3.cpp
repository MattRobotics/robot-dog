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
  return reinterpret_cast<void *>(1);
}
void vQueueDelete(QueueHandle_t) {}
void vSemaphoreDelete(SemaphoreHandle_t) {}
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t t) {
  CHECK(t == 0);
  return 1;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return 1; }
BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *,
                                   uint32_t stack, void *, unsigned, void *,
                                   int core) {
  CHECK(stack == 8192 && core == 0);
  return 1;
}
void vTaskDelay(TickType_t) {}
unsigned uxTaskGetStackHighWaterMark(void *) { return 4096; }
esp_err_t esp_wifi_get_mac(wifi_interface_t, uint8_t *m) {
  uint8_t mac[6] = {0x14, 0xc1, 0x9f, 0x22, 0x75, 0x94};
  std::memcpy(m, mac, 6);
  return 0;
}
esp_err_t esp_wifi_set_config(wifi_interface_t, const wifi_config_t *c) {
  CHECK(!c->sta.bssid_set);
  WiFi.driver = *c;
  return 0;
}
esp_err_t esp_wifi_connect() {
  ++WiFi.connect_calls;
  return 0;
}
esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *r) {
  if (!WiFi.connected)
    return -1;
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
int main() {
  seed();
  WifiManager m;
  WifiRuntimeTest::initialize(m);
  CHECK(!WiFi.persistent_setting && !WiFi.reconnect);
  CHECK(std::strcmp(m.status().ap_ssid, "MATDOG-227594") == 0);
  CHECK(m.requestAp(true));
  WifiRuntimeTest::tick(m, 50);
  CHECK(m.status().ap_active);
  CHECK(WiFi.ap_key == "unique-device-key");
  CHECK(!m.status().sleep_effective);
  association("network");
  WiFi.delayed_disconnect = true;
  CHECK(m.configure(patch("network")));
  CHECK(m.configBusy());
  WifiRuntimeTest::tick(m, 100);
  CHECK(m.status().config_phase == ConfigPhase::TESTING);
  WifiRuntimeTest::tick(m, 1000);
  CHECK(m.status().config_phase == ConfigPhase::TESTING);
  CHECK(commits == 1); // PENDING only, old association cannot commit
  WiFi.connected = false;
  WiFi.delayed_disconnect = false;
  WifiRuntimeTest::tick(m, 1100);
  CHECK(WiFi.connect_calls > 0);
  association("network");
  WifiRuntimeTest::tick(m, 1200);
  CHECK(m.status().config_phase == ConfigPhase::COMMITTED);
  CHECK(!m.configBusy());
  CHECK(commits == 2);
  NetworkConfig c{};
  NetworkConfigNvs store;
  CHECK(store.load(&c));
  CHECK(std::strcmp(c.sta[0].ssid, "network") == 0);
  CHECK(std::strcmp(c.sta[0].password, "new-password") == 0);
  int baseline = sets;
  m.setContext(true, false);
  CHECK(!m.configure(patch("refused")));
  CHECK(!m.requestScan());
  WifiRuntimeTest::tick(m, 1300);
  CHECK(sets == baseline);
  m.setContext(false, false);
  CHECK(m.requestScan());
  WifiRuntimeTest::tick(m, 40000);
  CHECK(m.status().scan_running);
  m.setContext(true, true);
  WifiRuntimeTest::tick(m, 40050);
  CHECK(!m.status().scan_running);
  CHECK(!m.status().sleep_effective);
  m.setContext(false, false);
  CHECK(m.configure(patch("wrong-password")));
  WifiRuntimeTest::tick(m, 41000);
  WiFi.connected = false;
  WifiRuntimeTest::tick(m, 41100);
  WifiRuntimeTest::tick(m, 71000);
  CHECK(m.status().config_phase == ConfigPhase::ROLLBACK);
  CHECK(!m.configBusy());
  CHECK(store.load(&c));
  CHECK(std::strcmp(c.sta[0].ssid, "network") == 0);
  // AP form survives identity commit until its current client has left.
  association("network");
  WiFi.clients = 1;
  WifiRuntimeTest::tick(m, 71500);
  ConfigPatch ap{};
  ap.ap = true;
  ap.test_profile = 0;
  std::strcpy(ap.config.ap_name, "MATDOG-private");
  std::strcpy(ap.config.ap_password, "replacement-unique-key");
  CHECK(m.configure(ap));
  WifiRuntimeTest::tick(m, 72000);
  WifiRuntimeTest::tick(m, 72100);
  association("network");
  WifiRuntimeTest::tick(m, 72200);
  CHECK(m.status().config_phase == ConfigPhase::COMMITTED);
  CHECK(m.status().ap_reload_pending);
  CHECK(WiFi.ap_key == "unique-device-key");
  WiFi.clients = 0;
  WifiRuntimeTest::tick(m, 72300);
  CHECK(!m.status().ap_reload_pending);
  CHECK(WiFi.ap_key == "replacement-unique-key");
  CHECK(WiFi.ap_name == "MATDOG-private");
  // Reboot discards pending; no write at tick or during initialization/load.
  WifiManager rebooted;
  baseline = sets;
  WifiRuntimeTest::initialize(rebooted);
  CHECK(sets == baseline);
  CHECK(std::strcmp(rebooted.status().profiles[0].ssid, "network") == 0);
  std::printf("test_wifi_runtime_v3: %d checks, %d failures\n", checks,
              failures);
  return failures ? 1 : 0;
}
