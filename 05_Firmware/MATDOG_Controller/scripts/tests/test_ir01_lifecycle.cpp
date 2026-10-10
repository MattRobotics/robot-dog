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
  if (WiFi.mode_value == WIFI_OFF) return ESP_ERR_WIFI_NOT_INIT;
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

// Independent narrow probe: all storage and radio I/O use declared host doubles.
// Exercise only the real production worker and its registered event callback.
// A queued AP_STOP may arrive after WiFi.mode(STA) returns. The installed core
// queues Arduino callbacks via Network.postEvent; there is no synchronous drain.
void show(const char *label, WifiManager &m) {
  printf("%s phase=%s mode=%d closing=%d admitted=%d commits=%d\n", label,
    toString(m.adminAccess().phase.load()), WiFi.mode_value,
    int(m.adminAccess().closing.load()),
    int(m.adminAccess().allowed(m.adminAccess().generation())), commits);
}

void finishReturn(WifiManager &m, uint32_t t) {
  CHECK(WiFi.pending_events.size()==2);
  WiFi.pending_events.clear();
  // START may be delivered before the old STOP: both belong to this command.
  WiFi.callback(ARDUINO_EVENT_WIFI_STA_START,{}); WifiRuntimeTest::tick(m,t);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::WAIT_AP_CLOSE);
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP,{}); WifiRuntimeTest::tick(m,t);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  closeListeners(m); WifiRuntimeTest::tick(m,t+50);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  CHECK(!m.adminAccess().allowed(m.adminAccess().generation()));
  CHECK(WiFi.pending_events.size()==2); WiFi.pending_events.clear();
  WiFi.callback(ARDUINO_EVENT_WIFI_STA_STOP,{}); WifiRuntimeTest::tick(m,t+75);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  CHECK(!m.adminAccess().allowed(m.adminAccess().generation()));
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_START,{}); WifiRuntimeTest::tick(m,t+100);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ADMIN_AP);
}
void rotationOrStage(int kind) {
  seed(); WifiManager m; WifiRuntimeTest::initialize(m); enter(m,50);
  auto old=m.adminAccess().generation(); WiFi.deliver_events=false;
  if(kind==0) { std::string token(64,'a'); CHECK(m.provisionAdmin(token.c_str())); }
  else CHECK(m.configure(patch("new-network"),old));
  WifiRuntimeTest::tick(m,150);
  if(kind) { CHECK(commits==0); WifiRuntimeTest::tick(m,1200); }
  CHECK(m.adminAccess().phase.load()==NetworkPhase::LEAVE_ADMIN);
  if(kind==1) m.setContext(true,false);
  closeListeners(m); WifiRuntimeTest::tick(m,1250);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::WAIT_AP_CLOSE);
  CHECK(!m.adminAccess().allowed(old));
  CHECK(WiFi.mode_value==WIFI_STA);
  WifiRuntimeTest::tick(m,1300); // old AP_STOP remains in Arduino queue
  CHECK(m.adminAccess().phase.load()==NetworkPhase::WAIT_AP_CLOSE);
  CHECK(m.adminAccess().generation()==old);
  if(kind!=2) finishReturn(m,1350);
  else {
    WiFi.drainEvents(); WifiRuntimeTest::tick(m,1350);
    CHECK(m.adminAccess().phase.load()==NetworkPhase::CLIENT_STA);
    CHECK(commits==1);
    WifiRuntimeTest::tick(m,1500); association("new-network");
    WifiRuntimeTest::tick(m,1600);
    CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
    closeListeners(m); WifiRuntimeTest::tick(m,1650);
    CHECK(!m.adminAccess().allowed(old));
    WiFi.drainEvents(); WifiRuntimeTest::tick(m,1700);
    CHECK(m.adminAccess().phase.load()==NetworkPhase::ADMIN_AP);
    CHECK(commits==2);
  }
  CHECK(m.adminAccess().generation()!=old); CHECK(!m.adminAccess().allowed(old));
  CHECK(m.adminAccess().allowed(m.adminAccess().generation()));
  if(kind==1) CHECK(commits==0);
  show(kind==0?"rotation/after":kind==1?"refused-stage/after":"successful-stage/after",m);
}
void negatives() {
  seed(); WifiManager m; WifiRuntimeTest::initialize(m); enter(m,50);
  auto old=m.adminAccess().generation();
  CHECK(m.configure(patch("queued"),old));
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP,{});
  CHECK(!m.adminAccess().allowed(old)); WifiRuntimeTest::tick(m,150);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::FAULT); CHECK(commits==0);
  for(int unexpected:{ARDUINO_EVENT_WIFI_STA_START,ARDUINO_EVENT_WIFI_AP_START}) {
    seed(); WifiManager n; WifiRuntimeTest::initialize(n); enter(n,50);
    auto gen=n.adminAccess().generation(); WiFi.callback(unexpected,{});
    CHECK(!n.adminAccess().allowed(gen)); WifiRuntimeTest::tick(n,150);
    CHECK(n.adminAccess().phase.load()==NetworkPhase::FAULT);
  }
  // Delayed start acknowledgments are required even when driver bits are ready.
  seed(); WifiManager n; WifiRuntimeTest::initialize(n); WiFi.deliver_events=false;
  CHECK(n.requestAp(true)); WifiRuntimeTest::tick(n,50); closeListeners(n);
  WifiRuntimeTest::tick(n,100); WifiRuntimeTest::tick(n,200);
  CHECK(n.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  CHECK(n.adminAccess().generation()==0);
  // The role change is acknowledged; the first AP profile of this driver is a
  // second owned command. Writing it restarts the soft-AP inside the driver
  // (AP_STOP then AP_START), and that pair must be acknowledged as well.
  WiFi.drainEvents(); WifiRuntimeTest::tick(n,250);
  CHECK(n.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN);
  CHECK(n.adminAccess().generation()==0 && WiFi.ap_set_calls==1);
  CHECK(WiFi.pending_events.size()==2);
  CHECK(WiFi.pending_events[0]==ARDUINO_EVENT_WIFI_AP_STOP);
  CHECK(WiFi.pending_events[1]==ARDUINO_EVENT_WIFI_AP_START);
  WifiRuntimeTest::tick(n,280); CHECK(n.adminAccess().generation()==0);
  WiFi.drainEvents(); WifiRuntimeTest::tick(n,300);
  CHECK(n.adminAccess().allowed(n.adminAccess().generation()));
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_START,{}); // duplicate after acknowledgment
  CHECK(!n.adminAccess().allowed(n.adminAccess().generation()));
  WifiRuntimeTest::tick(n,350); CHECK(n.adminAccess().phase.load()==NetworkPhase::FAULT);
  // Missing close acknowledgment: no new command/generation, bounded fault.
  seed(); WifiManager missing; WifiRuntimeTest::initialize(missing); enter(missing,50);
  auto gen=missing.adminAccess().generation(); WiFi.deliver_events=false;
  std::string token(64,'a'); CHECK(missing.provisionAdmin(token.c_str()));
  WifiRuntimeTest::tick(missing,150); closeListeners(missing);
  WifiRuntimeTest::tick(missing,200); WifiRuntimeTest::tick(missing,3200);
  CHECK(missing.adminAccess().phase.load()==NetworkPhase::FAULT);
  CHECK(missing.adminAccess().generation()==gen && !missing.adminAccess().allowed(gen));
  // An event from a future mode cannot acknowledge the current closure.
  seed(); WifiManager order; WifiRuntimeTest::initialize(order); enter(order,50);
  WiFi.deliver_events=false; CHECK(order.provisionAdmin(token.c_str()));
  WifiRuntimeTest::tick(order,150); closeListeners(order); WifiRuntimeTest::tick(order,200);
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_START,{}); WifiRuntimeTest::tick(order,250);
  CHECK(order.adminAccess().phase.load()==NetworkPhase::FAULT);
}
void unverifiableClosure() {
  seed(); WifiManager m; WifiRuntimeTest::initialize(m); enter(m,50);
  auto gen=m.adminAccess().generation(); WiFi.deliver_events=false;
  std::string token(64,'a'); CHECK(m.provisionAdmin(token.c_str()));
  WifiRuntimeTest::tick(m,150); closeListeners(m); WifiRuntimeTest::tick(m,200);
  WiFi.drainEvents(); WiFi.mode_read_ok=false; WifiRuntimeTest::tick(m,250);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::WAIT_AP_CLOSE);
  CHECK(m.adminAccess().generation()==gen && !m.adminAccess().allowed(gen));
  WifiRuntimeTest::tick(m,3200);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::FAULT);
  WiFi.mode_read_ok=true; WifiRuntimeTest::tick(m,3250);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::FAULT);
}

void closedDriver() {
  seed(); WifiManager m; WifiRuntimeTest::initialize(m); enter(m,50);
  CHECK(m.setEnabled(false,150)); WifiRuntimeTest::tick(m,150);
  closeListeners(m); WifiRuntimeTest::tick(m,200);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::CLOSED);
  CHECK(m.setEnabled(true,250)); WifiRuntimeTest::tick(m,250);
  closeListeners(m); WifiRuntimeTest::tick(m,300);
  CHECK(WiFi.mode_value==WIFI_STA);
  CHECK(m.adminAccess().phase.load()==NetworkPhase::ENTER_ADMIN); // retained manual AP policy
  closeListeners(m); WifiRuntimeTest::tick(m,350);
  CHECK(m.adminAccess().allowed(m.adminAccess().generation()));
}

int main() {
  rotationOrStage(0); rotationOrStage(1); rotationOrStage(2); negatives(); closedDriver(); unverifiableClosure();
  printf("IR01_LIFECYCLE: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
