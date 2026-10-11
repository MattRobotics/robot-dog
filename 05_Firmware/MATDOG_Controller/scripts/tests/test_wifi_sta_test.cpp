// STA router test ("Prova e salva"): outcome report, bounded retry, stability
// hold and recovery of a verified AP. Production WifiManager, WifiPolicy,
// NetworkConfig and NVS owner; the Wi-Fi driver, its events, the scheduler and
// NVS are doubles. This is not radio proof: association, WPA2 and DHCP against
// a real router are hardware checks.
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
wifi_bandwidth_t bw = WIFI_BW_HT40; // installed driver's initial STA bandwidth
bool bandwidth_write_ok = true, bandwidth_read_ok = true, bandwidth_restart = true;
int bandwidth_writes = 0;
bool bandwidth_defer_events = false;
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
  ++bandwidth_writes;
  if (!bandwidth_write_ok) return -1;
  const bool changed = bw != v;
  bw = v;
  // Installed libnet80211 wifi_set_bw_process stops/starts the requested role.
  if (changed && bandwidth_restart && WiFi.STA.running) {
    const bool deliver = WiFi.deliver_events;
    if (bandwidth_defer_events) WiFi.deliver_events = false;
    WiFi.emit(ARDUINO_EVENT_WIFI_STA_STOP);
    WiFi.emit(ARDUINO_EVENT_WIFI_STA_START);
    WiFi.deliver_events = deliver;
  }
  return 0;
}
esp_err_t esp_wifi_get_bandwidth(wifi_interface_t, wifi_bandwidth_t *v) {
  if (!bandwidth_read_ok) return -1;
  *v = bw;
  return 0;
}
static std::map<std::string, std::vector<uint8_t>> blobs;
static int sets = 0, commits = 0;
static const char *fail_key = nullptr; // NVS refuses writes of this blob
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
  if (fail_key && std::strcmp(k, fail_key) == 0)
    return -1;
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
  static uint8_t pending(WifiManager &m) { return m.expected_events_.load(); }
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
  bw = WIFI_BW_HT40;
  bandwidth_write_ok = bandwidth_read_ok = bandwidth_restart = true;
  bandwidth_writes = 0;
  bandwidth_defer_events = false;
  NetworkConfig c{};
  std::strcpy(c.sta[0].ssid, "network");
  std::strcpy(c.sta[0].password, "correct-password");
  c.sta[0].enabled = true;
  std::strcpy(c.ap_password, "unique-device-key");
  c.admin_digest[0] = 1;
  NetworkConfigNvs store;
  CHECK(store.activate(c));
  sets = commits = 0;
  fail_key = nullptr;
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

StaTestOutcome outcome(const WifiManager &m) {
  return static_cast<StaTestOutcome>(m.status().sta_test.outcome);
}
void sta(int event, uint16_t reason = 0) {
  WiFiEventInfo_t info{};
  info.wifi_sta_disconnected.reason = reason;
  WiFi.callback(event, info);
}
std::string activeSsid() {
  NetworkConfig c{};
  NetworkConfigNvs store;
  return store.load(&c) ? std::string(c.sta[0].ssid) : std::string("<unreadable>");
}
// From a seeded controller to "test running, first association issued".
// Returns the time of that first attempt; `stage` is when PENDING was written
// (the start of the 30 s transaction window).
uint32_t startTest(WifiManager &m, const char *ssid, uint32_t *stage, uint32_t *seq,
                   const ConfigPatch *form = nullptr, bool pause_at_stage = false) {
  WifiRuntimeTest::initialize(m);
  enter(m, 50);
  const uint32_t generation = m.adminAccess().generation();
  CHECK(m.configure(form ? *form : patch(ssid), generation));
  *seq = m.lastPostSeq();
  WifiRuntimeTest::tick(m, 1000);
  CHECK(m.status().config_phase == ConfigPhase::CONFIG_PENDING && commits == 0);
  WifiRuntimeTest::tick(m, 2050); // the AP leaves after the response interval
  CHECK(!m.adminAccess().allowed(generation));
  closeListeners(m);
  WifiRuntimeTest::tick(m, 2100); // exclusive STA, PENDING written
  CHECK(m.adminAccess().phase.load() == NetworkPhase::CLIENT_STA && WiFi.mode_value == WIFI_STA);
  CHECK(m.status().config_phase == ConfigPhase::TESTING && commits == 1);
  CHECK(outcome(m) == StaTestOutcome::RUNNING && m.status().sta_test.seq == *seq);
  CHECK(!m.adminAccess().radio_fault.load() && m.status().fault_count == 0);
  CHECK(bw == WIFI_BW_HT20 && bandwidth_writes == 1);
  CHECK(std::strcmp(m.status().sta_test.ssid, ssid) == 0);
  *stage = 2100;
  if (pause_at_stage) return *stage;
  const int before = WiFi.connect_calls;
  WifiRuntimeTest::tick(m, 2250);
  CHECK(WiFi.connect_calls == before + 1 && m.status().sta_test.attempts == 1);
  return 2250;
}
// Every ending must lead back to a freshly verified, WPA2-protected AP that
// still uses the stored AP key.
void backToAp(WifiManager &m, uint32_t t, uint32_t old_generation) {
  CHECK(m.adminAccess().phase.load() == NetworkPhase::ENTER_ADMIN);
  CHECK(!m.configBusy());
  closeListeners(m);
  WifiRuntimeTest::tick(m, t);
  CHECK(m.adminAccess().phase.load() == NetworkPhase::ADMIN_AP);
  CHECK(m.adminAccess().generation() != old_generation);
  CHECK(m.adminAccess().allowed(m.adminAccess().generation()));
  CHECK(WifiManager::verifyAdminRadio() && WiFi.mode_value == WIFI_AP);
  CHECK(m.status().ap_active && m.status().ap_config_verified);
  CHECK(WiFi.ap_key == "unique-device-key" && WiFi.ap_profile.ap.authmode == WIFI_AUTH_WPA2_PSK);
  CHECK(m.status().config_error == 0);
}
struct Failure { uint16_t reason; StaTestOutcome expected; };
int main() {
  // ---- the verdict table ------------------------------------------------------
  using O = StaTestOutcome;
  CHECK(classifyStaTestFailure(true, true, true, 15) == O::STORAGE_ERROR);
  CHECK(classifyStaTestFailure(false, true, true, 15) == O::ABORTED);
  for (uint16_t r : {15, 202, 204}) {
    CHECK(classifyStaTestFailure(false, false, false, r) == O::AUTH_REJECTED);
    CHECK(classifyStaTestFailure(false, false, true, r) == O::AUTH_REJECTED);
  }
  for (uint16_t r : {201, 210, 211, 212})
    CHECK(classifyStaTestFailure(false, false, false, r) == O::AP_NOT_FOUND);
  for (uint16_t r : {0, 2, 3, 8, 200, 203, 205, 209, 213})
    CHECK(classifyStaTestFailure(false, false, false, r) == O::TIMEOUT);
  for (uint16_t r : {0, 2, 8, 200})
    CHECK(classifyStaTestFailure(false, false, true, r) == O::NO_IP);
  std::map<std::string, int> names;
  for (int i = 0; i <= 8; ++i) ++names[toString(static_cast<O>(i))];
  CHECK(names.size() == 9 && !names.count("UNKNOWN"));

  // ---- no test since boot ---------------------------------------------------------
  {
    seed(); WifiManager m; WifiRuntimeTest::initialize(m);
    CHECK(outcome(m) == O::NONE && m.status().sta_test.seq == 0);
  }

  // ---- owned bandwidth restart: drain both events before connect/commit/AP return ----
  {
    seed(); bandwidth_defer_events = true;
    WifiManager m; uint32_t stage = 0, seq = 0;
    startTest(m, "home-router", &stage, &seq, nullptr, true);
    const uint32_t generation = m.adminAccess().generation();
    CHECK(WifiRuntimeTest::pending(m) == 6); // STA_STOP | STA_START
    CHECK(WiFi.pending_events == std::vector<int>({ARDUINO_EVENT_WIFI_STA_STOP,
                                                 ARDUINO_EVENT_WIFI_STA_START}));
    CHECK(!m.adminAccess().allowed(generation));
    CHECK(!m.configure(patch("stale-request"), generation));
    const int calls = WiFi.connect_calls;
    association("home-router"); // stale link cannot prove the candidate while ack is pending
    WifiRuntimeTest::tick(m, stage + 150);
    CHECK(WiFi.connect_calls == calls && commits == 1 && bandwidth_writes == 1);
    sta(ARDUINO_EVENT_WIFI_STA_STOP);
    CHECK(WifiRuntimeTest::pending(m) == 2 && !m.adminAccess().radio_fault.load());
    WifiRuntimeTest::tick(m, stage + 200);
    CHECK(WiFi.connect_calls == calls && commits == 1 && m.configBusy());
    WiFi.connected = false; WiFi.ip = 0;
    sta(ARDUINO_EVENT_WIFI_STA_START); WiFi.pending_events.clear();
    WifiRuntimeTest::tick(m, stage + 250);
    CHECK(WifiRuntimeTest::pending(m) == 0 && WiFi.connect_calls == calls + 1);
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED); association("home-router");
    WifiRuntimeTest::tick(m, stage + 500);
    WifiRuntimeTest::tick(m, stage + 500 + WifiManager::kStaTestHoldMs);
    CHECK(outcome(m) == O::CONNECTED && commits == 2 && activeSsid() == "home-router");
    CHECK(m.adminAccess().phase.load() == NetworkPhase::ENTER_ADMIN);
    // Returning to AP is also fenced: a new generation requires its own acks.
    closeListeners(m); WiFi.deliver_events = false;
    WifiRuntimeTest::tick(m, stage + 2050);
    CHECK(!m.adminAccess().allowed(generation) && m.adminAccess().generation() == generation);
    CHECK(WifiRuntimeTest::pending(m) == 12); // STA_STOP | AP_START
    WifiRuntimeTest::tick(m, stage + 2100);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::ENTER_ADMIN);
    WiFi.deliver_events = true; WiFi.drainEvents();
    WifiRuntimeTest::tick(m, stage + 2150);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::ADMIN_AP);
    CHECK(m.adminAccess().generation() != generation && !m.adminAccess().allowed(generation));
    CHECK(m.adminAccess().allowed(m.adminAccess().generation()) && WifiManager::verifyAdminRadio());
    CHECK(m.status().fault_count == 0 && m.status().ap_config_verified);
  }

  // ---- missing restart acknowledgment: bounded fault, rollback, no ACTIVE write ----
  {
    seed(); bandwidth_defer_events = true;
    WifiManager m; uint32_t stage = 0, seq = 0;
    startTest(m, "home-router", &stage, &seq, nullptr, true);
    sta(ARDUINO_EVENT_WIFI_STA_STOP); // missing STA_START must never allow progress
    WifiRuntimeTest::tick(m, stage + 2999);
    CHECK(outcome(m) == O::RUNNING && m.configBusy() && WiFi.connect_calls == 0);
    WifiRuntimeTest::tick(m, stage + 3000);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::FAULT);
    CHECK(m.status().fault_site == static_cast<uint8_t>(FaultSite::ROLE_ACK_TIMEOUT));
    WifiRuntimeTest::tick(m, stage + 3050);
    CHECK(outcome(m) == O::ABORTED && m.status().config_phase == ConfigPhase::ROLLBACK);
    CHECK(!m.configBusy() && commits == 1 && activeSsid() == "network");
    CHECK(!m.adminAccess().allowed(m.adminAccess().generation()));
  }

  // ---- duplicate/unowned STA_STOP still revokes and aborts immediately ----
  for (bool during_test : {false, true}) {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    if (during_test) startTest(m, "home-router", &stage, &seq);
    else { WifiRuntimeTest::initialize(m); enter(m, 50); }
    const uint32_t generation = m.adminAccess().generation();
    CHECK(WifiRuntimeTest::pending(m) == 0);
    sta(ARDUINO_EVENT_WIFI_STA_STOP);
    CHECK(m.adminAccess().radio_fault.load() && !m.adminAccess().allowed(generation));
    WifiRuntimeTest::tick(m, during_test ? stage + 200 : 150);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::FAULT);
    CHECK(m.status().fault_site == static_cast<uint8_t>(FaultSite::UNEXPECTED_ROLE_EVENT));
    CHECK(m.status().fault_event == 4 && m.status().fault_expected == 0);
    CHECK(activeSsid() == "network" && commits == (during_test ? 1 : 0));
    if (during_test) CHECK(outcome(m) == O::ABORTED && m.status().sta_test.rolled_back);
  }

  // A second STOP while only START remains armed is also unexpected.
  {
    seed(); bandwidth_defer_events = true;
    WifiManager m; uint32_t stage = 0, seq = 0;
    startTest(m, "home-router", &stage, &seq, nullptr, true);
    sta(ARDUINO_EVENT_WIFI_STA_STOP);
    sta(ARDUINO_EVENT_WIFI_STA_STOP);
    CHECK(m.adminAccess().radio_fault.load());
    WifiRuntimeTest::tick(m, stage + 50);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::FAULT);
    CHECK(m.status().fault_site == static_cast<uint8_t>(FaultSite::UNEXPECTED_ROLE_EVENT));
    CHECK(m.status().fault_event == 4 && m.status().fault_expected == 2);
    CHECK(outcome(m) == O::ABORTED && m.status().sta_test.rolled_back && WiFi.connect_calls == 0);
  }

  // ---- equal/unknown bandwidth never arms a speculative restart ----
  for (bool read_ok : {true, false}) {
    seed(); bw = WIFI_BW_HT20; bandwidth_read_ok = read_ok;
    WifiManager m; WifiRuntimeTest::initialize(m); enter(m, 50);
    CHECK(m.configure(patch("home-router"), m.adminAccess().generation()));
    WifiRuntimeTest::tick(m, 1000); WifiRuntimeTest::tick(m, 2050);
    closeListeners(m); WifiRuntimeTest::tick(m, 2100);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::CLIENT_STA);
    CHECK(bandwidth_writes == 0 && WifiRuntimeTest::pending(m) == 0);
    CHECK(m.status().bandwidth_apply_ok == read_ok && m.status().fault_count == 0);
  }
  // A refused owned setter must fail closed rather than start association.
  {
    seed(); bandwidth_write_ok = false;
    WifiManager m; WifiRuntimeTest::initialize(m); enter(m, 50);
    CHECK(m.configure(patch("home-router"), m.adminAccess().generation()));
    WifiRuntimeTest::tick(m, 1000); WifiRuntimeTest::tick(m, 2050);
    closeListeners(m); WifiRuntimeTest::tick(m, 2100);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::FAULT);
    CHECK(m.status().fault_site == static_cast<uint8_t>(FaultSite::MODE_COMMAND));
    WifiRuntimeTest::tick(m, 2150);
    CHECK(outcome(m) == O::ABORTED && m.status().config_phase == ConfigPhase::ROLLBACK);
    CHECK(WiFi.connect_calls == 0 && commits == 1 && activeSsid() == "network");
  }

  // ---- refused by the router, router absent: retry, rollback, AP back ------------
  for (Failure f : {Failure{15, O::AUTH_REJECTED}, Failure{204, O::AUTH_REJECTED},
                    Failure{202, O::AUTH_REJECTED}, Failure{201, O::AP_NOT_FOUND},
                    Failure{211, O::AP_NOT_FOUND}}) {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    const uint32_t generation = m.adminAccess().generation();
    const int calls = WiFi.connect_calls;
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, f.reason);
    WifiRuntimeTest::tick(m, first + WifiManager::kStaTestRetryMs - 1);
    CHECK(WiFi.connect_calls == calls);            // never faster than the retry interval
    CHECK(m.status().sta_test.reason == f.reason); // visible on USB while running
    WifiRuntimeTest::tick(m, first + WifiManager::kStaTestRetryMs);
    CHECK(WiFi.connect_calls == calls + 1 && m.status().sta_test.attempts == 2);
    // No new verdict from the driver: the attempt is still in flight, no retry.
    WifiRuntimeTest::tick(m, first + 2 * WifiManager::kStaTestRetryMs);
    CHECK(WiFi.connect_calls == calls + 1);
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, f.reason);
    WifiRuntimeTest::tick(m, first + 2 * WifiManager::kStaTestRetryMs + 50);
    CHECK(WiFi.connect_calls == calls + 2 && m.status().sta_test.attempts == 3);
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, f.reason);
    WifiRuntimeTest::tick(m, stage + 29999);
    CHECK(m.status().config_phase == ConfigPhase::TESTING && m.configBusy());
    CHECK(WiFi.mode_value == WIFI_STA); // nothing else on the air during the test
    const int sets_before = sets;
    WifiRuntimeTest::tick(m, stage + 30000);
    CHECK(m.status().config_phase == ConfigPhase::ROLLBACK);
    CHECK(m.status().tx_result == static_cast<uint8_t>(TxResult::ROLLBACK));
    CHECK(outcome(m) == f.expected && m.status().sta_test.reason == f.reason);
    CHECK(m.status().sta_test.rolled_back && !m.status().sta_test.associated);
    CHECK(m.status().sta_test.ipv4 == 0 && m.status().sta_test.elapsed_ms == 30000);
    CHECK(m.status().sta_test.seq == seq && m.status().sta_test.attempts == 4);
    // ACTIVE was never written: one PENDING write in the whole test.
    CHECK(sets == sets_before && commits == 1 && activeSsid() == "network");
    backToAp(m, stage + 30050, generation);
    CHECK(outcome(m) == f.expected); // still readable once the AP is back
    // A restart after the failure loads the known-good record, never PENDING.
    WifiManager rebooted; const int writes = sets; WifiRuntimeTest::initialize(rebooted);
    CHECK(sets == writes && std::strcmp(rebooted.status().profiles[0].ssid, "network") == 0);
  }

  // ---- accepted by the router, no address ---------------------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    const uint32_t generation = m.adminAccess().generation();
    const int calls = WiFi.connect_calls;
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 2); // an earlier failed attempt
    WifiRuntimeTest::tick(m, first + 4000);
    CHECK(WiFi.connect_calls == calls + 1);
    // A late report of an earlier disconnect, then the association itself:
    // the driver is associated, so no further connect may be issued.
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 8);
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WifiRuntimeTest::tick(m, first + 9000);
    CHECK(m.status().sta_test.associated && m.status().sta_test.reason == 0);
    CHECK(WiFi.connect_calls == calls + 1); // an associated link is never disturbed
    WiFi.connected = true; WiFi.ip = 0; WiFi.associated_ssid = "home-router"; // static/empty address
    WifiRuntimeTest::tick(m, first + 20000);
    CHECK(m.status().config_phase == ConfigPhase::TESTING && commits == 1);
    WifiRuntimeTest::tick(m, stage + 30000);
    CHECK(outcome(m) == O::NO_IP && m.status().sta_test.rolled_back && m.status().sta_test.associated);
    CHECK(activeSsid() == "network" && commits == 1);
    backToAp(m, stage + 30050, generation);
  }

  // ---- nothing conclusive: plain timeout, exactly one attempt ------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    startTest(m, "home-router", &stage, &seq);
    const uint32_t generation = m.adminAccess().generation();
    const int calls = WiFi.connect_calls;
    for (uint32_t t = stage + 1000; t < stage + 30000; t += 1000) WifiRuntimeTest::tick(m, t);
    CHECK(WiFi.connect_calls == calls && m.status().config_phase == ConfigPhase::TESTING);
    WifiRuntimeTest::tick(m, stage + 30000);
    CHECK(outcome(m) == O::TIMEOUT && m.status().sta_test.rolled_back && m.status().sta_test.attempts == 1);
    backToAp(m, stage + 30050, generation);
  }

  // ---- an association to another network is never proof ---------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    association("neighbour");
    for (uint32_t t = first + 500; t < stage + 30000; t += 2500) WifiRuntimeTest::tick(m, t);
    CHECK(commits == 1 && m.status().config_phase == ConfigPhase::TESTING);
    WifiRuntimeTest::tick(m, stage + 30000);
    CHECK(m.status().config_phase == ConfigPhase::ROLLBACK && activeSsid() == "network");
  }

  // ---- success: only a HELD link commits -------------------------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    const uint32_t generation = m.adminAccess().generation();
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED);
    association("home-router"); WiFi.channel_value = 6;
    WifiRuntimeTest::tick(m, first + 2000);
    CHECK(commits == 1 && outcome(m) == O::RUNNING); // first sighting is not proof
    WifiRuntimeTest::tick(m, first + 3000);
    CHECK(commits == 1);
    // The link flickers: the stability window starts again from zero.
    WiFi.connected = false; WifiRuntimeTest::tick(m, first + 3100);
    association("home-router"); WifiRuntimeTest::tick(m, first + 3200);
    WifiRuntimeTest::tick(m, first + 3200 + WifiManager::kStaTestHoldMs - 1);
    CHECK(commits == 1 && m.status().config_phase == ConfigPhase::TESTING);
    WifiRuntimeTest::tick(m, first + 3200 + WifiManager::kStaTestHoldMs);
    CHECK(commits == 2 && m.status().config_phase == ConfigPhase::COMMITTED);
    CHECK(m.status().tx_result == static_cast<uint8_t>(TxResult::COMMITTED));
    const auto &report = m.status().sta_test;
    CHECK(outcome(m) == O::CONNECTED && !report.rolled_back && report.associated);
    CHECK(report.ipv4 == 0x3501a8c0 && report.rssi_dbm == -85 && report.channel == 6);
    CHECK(report.attempts == 1 && report.reason == 0 && report.seq == seq);
    CHECK(report.elapsed_ms == first + 3200 + WifiManager::kStaTestHoldMs - stage);
    CHECK(activeSsid() == "home-router");
    backToAp(m, first + 6000, generation);
    CHECK(outcome(m) == O::CONNECTED);
    WifiManager rebooted; WifiRuntimeTest::initialize(rebooted);
    CHECK(std::strcmp(rebooted.status().profiles[0].ssid, "home-router") == 0);
    CHECK(outcome(rebooted) == O::NONE); // the report is RAM only
  }

  // ---- first attempt fails, the retry connects --------------------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    const int calls = WiFi.connect_calls;
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 2);
    WifiRuntimeTest::tick(m, first + 4000);
    CHECK(WiFi.connect_calls == calls + 1);
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED); association("home-router");
    WifiRuntimeTest::tick(m, first + 6000);
    WifiRuntimeTest::tick(m, first + 6000 + WifiManager::kStaTestHoldMs);
    CHECK(outcome(m) == O::CONNECTED && m.status().sta_test.attempts == 2);
    CHECK(m.status().sta_test.reason == 0 && commits == 2);
  }

  // ---- critical context ends the test at once -----------------------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    const uint32_t generation = m.adminAccess().generation();
    sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 15);
    m.setContext(true, false);
    const int calls = WiFi.connect_calls;
    WifiRuntimeTest::tick(m, first + 5000);
    CHECK(WiFi.connect_calls == calls); // no retry in a critical context
    CHECK(m.status().config_phase == ConfigPhase::ROLLBACK && outcome(m) == O::ABORTED);
    CHECK(m.status().sta_test.rolled_back && activeSsid() == "network" && commits == 1);
    m.setContext(false, false);
    backToAp(m, first + 5050, generation);
  }

  // ---- radio fault during the test -----------------------------------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    WiFi.callback(ARDUINO_EVENT_WIFI_AP_START, {}); // not part of any command
    WifiRuntimeTest::tick(m, first + 1000);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::FAULT);
    CHECK(outcome(m) == O::ABORTED && m.status().sta_test.rolled_back && !m.configBusy());
    CHECK(m.status().config_phase == ConfigPhase::ROLLBACK);
    CHECK(activeSsid() == "network" && commits == 1);
    // FAULT is fail closed; an explicit request (USB) restores the AP.
    closeListeners(m); WifiRuntimeTest::tick(m, first + 1050);
    enter(m, first + 5000);
    CHECK(WiFi.ap_key == "unique-device-key" && outcome(m) == O::ABORTED);
  }

  // ---- intent accepted, radio fault before any test: nothing was tried ------------------
  {
    seed(); WifiManager m; WifiRuntimeTest::initialize(m); enter(m, 50);
    CHECK(m.configure(patch("home-router"), m.adminAccess().generation()));
    const uint32_t seq = m.lastPostSeq();
    WifiRuntimeTest::tick(m, 1000);
    WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP, {});
    WifiRuntimeTest::tick(m, 1050);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::FAULT && commits == 0);
    CHECK(outcome(m) == O::ABORTED && m.status().sta_test.seq == seq);
    CHECK(m.status().sta_test.attempts == 0 && m.status().sta_test.rolled_back);
    CHECK(std::strcmp(m.status().sta_test.ssid, "home-router") == 0 && activeSsid() == "network");
  }

  // ---- NVS refuses PENDING: no test, AP back, known-good untouched -----------------------
  {
    seed(); WifiManager m; WifiRuntimeTest::initialize(m); enter(m, 50);
    const uint32_t generation = m.adminAccess().generation();
    CHECK(m.configure(patch("home-router"), generation));
    WifiRuntimeTest::tick(m, 1000); WifiRuntimeTest::tick(m, 2050);
    closeListeners(m);
    fail_key = "pending";
    const int calls = WiFi.connect_calls;
    WifiRuntimeTest::tick(m, 2100);
    fail_key = nullptr;
    CHECK(outcome(m) == O::STORAGE_ERROR && !m.status().sta_test.rolled_back);
    CHECK(m.status().config_phase == ConfigPhase::STORAGE_ERROR && activeSsid() == "network");
    WifiRuntimeTest::tick(m, 2300);
    CHECK(WiFi.connect_calls == calls); // the candidate never reached the radio
    CHECK(m.adminAccess().phase.load() == NetworkPhase::ENTER_ADMIN && !m.configBusy());
    closeListeners(m); WifiRuntimeTest::tick(m, 2350);
    CHECK(m.adminAccess().phase.load() == NetworkPhase::ADMIN_AP && WiFi.ap_key == "unique-device-key");
  }

  // ---- NVS refuses ACTIVE after a proven link: reported, never called success ---------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq);
    sta(ARDUINO_EVENT_WIFI_STA_CONNECTED); association("home-router");
    WifiRuntimeTest::tick(m, first + 2000);
    fail_key = "active";
    WifiRuntimeTest::tick(m, first + 2000 + WifiManager::kStaTestHoldMs);
    fail_key = nullptr;
    CHECK(outcome(m) == O::STORAGE_ERROR && !m.status().sta_test.rolled_back);
    CHECK(m.status().tx_result == static_cast<uint8_t>(TxResult::STORAGE_ERROR));
    CHECK(activeSsid() == "network");
  }

  // ---- one form carrying router credentials and the AP preference ---------------------------
  // One request, one transaction: both are stored, or neither.
  const char body[] =
      "p0_ssid=home-router&p0_password=router+secret%21&p0_enabled=1&p0_dhcp=1&p0_ip=0.0.0.0"
      "&p0_mask=0.0.0.0&p0_gateway=0.0.0.0&p0_dns=0.0.0.0&p1_ssid=&p1_password=&p1_enabled=0"
      "&p1_dhcp=1&p1_ip=0.0.0.0&p1_mask=0.0.0.0&p1_gateway=0.0.0.0&p1_dns=0.0.0.0"
      "&test_profile=0&ap_name=MATDOG-227594&ap_password=&ap_timeout_ms=900000&ap_always=1";
  ConfigPatch form{};
  CHECK(parseConfigForm(body, sizeof(body) - 1, &form));
  CHECK(form.profiles == 3 && form.passwords == 1 && form.ap && form.config.ap_always);
  CHECK(std::strcmp(form.config.sta[0].password, "router secret!") == 0 && !form.config.ap_password[0]);
  CHECK(!parseConfigForm("p0_ssid=x&boot_policy=1", 23, &form)); // a page-only field is refused
  CHECK(parseConfigForm(body, sizeof(body) - 1, &form));
  for (bool success : {true, false}) {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    const uint32_t first = startTest(m, "home-router", &stage, &seq, &form);
    const uint32_t generation = m.adminAccess().generation();
    uint32_t end = stage + 30000;
    if (success) {
      sta(ARDUINO_EVENT_WIFI_STA_CONNECTED); association("home-router");
      WifiRuntimeTest::tick(m, first + 2000);
      end = first + 2000 + WifiManager::kStaTestHoldMs;
    } else {
      sta(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, 15);
    }
    WifiRuntimeTest::tick(m, end);
    NetworkConfig c{}; NetworkConfigNvs store; CHECK(store.load(&c));
    CHECK(c.ap_always == success);
    CHECK(std::strcmp(c.sta[0].ssid, success ? "home-router" : "network") == 0);
    CHECK(std::strcmp(c.sta[0].password, success ? "router secret!" : "correct-password") == 0);
    CHECK(std::strcmp(c.ap_password, "unique-device-key") == 0); // empty field keeps the AP key
    backToAp(m, end + 50, generation);
    CHECK(WiFi.ap_name == "MATDOG-227594" && m.status().ap_always == success);
    // No second AP bounce after the result is published.
    const uint32_t settled = m.adminAccess().generation();
    for (uint32_t t = end + 100; t < end + 5000; t += 500) WifiRuntimeTest::tick(m, t);
    CHECK(m.adminAccess().generation() == settled && m.adminAccess().allowed(settled));
    // In MAINTENANCE the AP is mandatory whatever the stored preference says.
    CHECK(!m.requestAp(false));
  }

  // ---- the report can never carry a passphrase ---------------------------------------------
  {
    seed(); WifiManager m; uint32_t stage = 0, seq = 0;
    startTest(m, "home-router", &stage, &seq);
    const auto &w = m.status();
    const char *raw = reinterpret_cast<const char *>(&w);
    bool leaked = false;
    for (const char *secret : {"new-password", "correct-password", "unique-device-key"})
      for (size_t i = 0; i + std::strlen(secret) <= sizeof(w); ++i)
        if (std::memcmp(raw + i, secret, std::strlen(secret)) == 0) leaked = true;
    CHECK(!leaked);
  }
  printf("WIFI_STA_TEST: %d checks, %d failures (driver/NVS doubles; no radio proof)\n", checks, failures);
  return failures ? 1 : 0;
}
