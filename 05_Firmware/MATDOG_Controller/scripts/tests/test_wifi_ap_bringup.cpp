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
  static bool held(WifiManager &m) { return m.held_valid_; }
  static void publish(WifiManager &m) {
    m.shared_status_ = m.worker_status_;
    std::memcpy(m.shared_admin_digest_, m.config_.admin_digest, 32);
    m.update(now);
  }
};
} // namespace network
} // namespace matdog

// ---------------------------------------------------------------------------
// WPA2 / AP bring-up regression (root cause of config_error=11 on 41753ac).
//
// All manager code is production. Driver, events, scheduling and NVS are
// doubles, but the soft-AP double now reproduces the two facts of the real
// driver that the previous double hid: the AP role starts on the mode change,
// and writing a profile to a started AP restarts it (AP_STOP, AP_START).
// Nothing here proves radio authentication: that stays a hardware gate.
// ---------------------------------------------------------------------------
const char *kKeyA = "unique-device-key";
const char *kKeyB = "rotated-device-key-2";
// A provisioned robot with NO station profile: the state of the hardware.
void seedApOnly(const char *key = kKeyA) {
  blobs.clear();
  WiFi = WifiStub{};
  NetworkConfig c{};
  std::strcpy(c.ap_password, key);
  c.admin_digest[0] = 1;
  NetworkConfigNvs store;
  CHECK(store.activate(c));
  sets = commits = 0;
  now = 0;
}
void closeListeners(WifiManager &m) {
  auto &g = m.adminAccess();
  AdminLease drain(g);
  CHECK(bool(drain));
  CHECK(g.closing.load());
  g.listeners_closed.store(true);
}
NetworkPhase phase(WifiManager &m) { return m.adminAccess().phase.load(); }
bool admitted(WifiManager &m) {
  return m.adminAccess().allowed(m.adminAccess().generation());
}
std::string driverKey() {
  const char *key = reinterpret_cast<const char *>(WiFi.ap_profile.ap.password);
  return std::string(key, strnlen(key, 64));
}
std::string storedKey() {
  NetworkConfig c{};
  NetworkConfigNvs store;
  CHECK(store.load(&c));
  return c.ap_password;
}
uint8_t code(FaultSite site) { return static_cast<uint8_t>(site); }
uint8_t code(TxResult result) { return static_cast<uint8_t>(result); }
uint8_t code(VerifyResult result) { return static_cast<uint8_t>(result); }
bool kindIs(WifiManager &m, const char *name) {
  return std::strcmp(WifiManager::requestKindName(m.status().tx_kind), name) == 0;
}
// Boot to a verified ADMIN_AP with synchronous event delivery.
void bootToAdmin(WifiManager &m) {
  WifiRuntimeTest::initialize(m);
  CHECK(phase(m) == NetworkPhase::ENTER_ADMIN);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 50);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP);
  CHECK(admitted(m));
}

// The double itself: if it ever stops restarting the AP on a profile write,
// every PASS below would be the old false PASS again.
void doubleReproducesTheDriver() {
  WiFi = WifiStub{};
  std::vector<int> seen;
  WiFi.onEvent([&](WiFiEvent_t e, WiFiEventInfo_t) { seen.push_back(e); });
  CHECK(WiFi.mode(WIFI_AP));
  CHECK(WiFi.AP.started()); // the role is up before any profile exists
  CHECK(seen.size() == 2 && seen[0] == ARDUINO_EVENT_WIFI_STA_STOP &&
        seen[1] == ARDUINO_EVENT_WIFI_AP_START);
  seen.clear();
  CHECK(WiFi.softAP("MATDOG-227594", kKeyA, 1, false, 4)); // the stock call
  CHECK(seen.size() == 2 && seen[0] == ARDUINO_EVENT_WIFI_AP_STOP &&
        seen[1] == ARDUINO_EVENT_WIFI_AP_START);
  seen.clear();
  CHECK(WiFi.softAP("MATDOG-227594", kKeyA, 1, false, 4)); // identical: no write
  CHECK(seen.empty() && WiFi.ap_set_calls == 1);
  CHECK(WiFi.softAP("MATDOG-227594", kKeyB, 1, false, 4)); // new key: restart
  CHECK(seen.size() == 2 && WiFi.ap_set_calls == 2);
  WiFi = WifiStub{};
}

void bootWithoutStaProfiles() {
  seedApOnly();
  WifiManager m;
  bootToAdmin(m);
  const auto &w = m.status();
  CHECK(w.ap_active && w.ap_config_verified);
  CHECK(w.config_error == 0 && w.fault_site == 0 && w.fault_count == 0);
  CHECK(w.admin_phase == static_cast<uint8_t>(NetworkPhase::ADMIN_AP));
  CHECK(w.driver_mode == WIFI_MODE_AP && w.driver_ap_started && !w.driver_sta_started);
  CHECK(WiFi.mode_value == WIFI_AP && WiFi.ap_set_calls == 1);
  CHECK(std::strcmp(w.ap_ssid, "MATDOG-227594") == 0 && WiFi.ap_name == "MATDOG-227594");
  CHECK(driverKey() == kKeyA && storedKey() == kKeyA);
  CHECK(WiFi.ap_profile.ap.authmode == WIFI_AUTH_WPA2_PSK);
  CHECK(WiFi.ap_profile.ap.pairwise_cipher == WIFI_CIPHER_TYPE_CCMP);
  CHECK(WiFi.ap_profile.ap.channel == 1 && WiFi.ap_profile.ap.max_connection == 4);
  CHECK(WiFi.ap_profile.ap.ssid_len == 13 && !WiFi.ap_profile.ap.ssid_hidden);
  CHECK(sets == 0 && commits == 0); // boot never writes NVS
  // Stable: nothing reopens, rewrites or faults while idle.
  for (uint32_t t : {100u, 1000u, 60000u}) WifiRuntimeTest::tick(m, t);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && WiFi.ap_set_calls == 1);
  CHECK(m.status().config_error == 0 && m.status().fault_count == 0);
}

// Same boot with the events queued, as the Arduino event task delivers them.
void profileRestartIsAcknowledged() {
  seedApOnly();
  WifiManager m;
  WifiRuntimeTest::initialize(m);
  WiFi.deliver_events = false;
  closeListeners(m);
  WifiRuntimeTest::tick(m, 50);
  // Command 1 issued: the role change only. The profile is not written while
  // its acknowledgments are outstanding.
  CHECK(WiFi.mode_value == WIFI_AP && WiFi.ap_set_calls == 0);
  CHECK(WiFi.pending_events.size() == 2);
  WifiRuntimeTest::tick(m, 100);
  CHECK(WiFi.ap_set_calls == 0 && phase(m) == NetworkPhase::ENTER_ADMIN);
  WiFi.drainEvents();
  WifiRuntimeTest::tick(m, 150);
  // Command 2 issued: profile written, driver restarted the AP.
  CHECK(WiFi.ap_set_calls == 1 && driverKey() == kKeyA);
  CHECK(WiFi.pending_events.size() == 2);
  CHECK(WiFi.pending_events[0] == ARDUINO_EVENT_WIFI_AP_STOP);
  CHECK(WiFi.pending_events[1] == ARDUINO_EVENT_WIFI_AP_START);
  CHECK(phase(m) == NetworkPhase::ENTER_ADMIN && !admitted(m));
  // Driver bits alone never admit; either delivery order is this command's.
  WifiRuntimeTest::tick(m, 200);
  CHECK(!admitted(m) && m.adminAccess().generation() == 0);
  WiFi.pending_events.clear();
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_START, {});
  WifiRuntimeTest::tick(m, 250);
  CHECK(!admitted(m) && phase(m) == NetworkPhase::ENTER_ADMIN);
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP, {});
  WifiRuntimeTest::tick(m, 300);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && admitted(m));
  CHECK(m.status().config_error == 0 && m.status().fault_count == 0);

  // A missing half of the pair is a bounded fault, never an admission.
  seedApOnly();
  WifiManager lost;
  WifiRuntimeTest::initialize(lost);
  WiFi.deliver_events = false;
  closeListeners(lost);
  WifiRuntimeTest::tick(lost, 50);
  WiFi.drainEvents();
  WifiRuntimeTest::tick(lost, 100);
  WiFi.pending_events.clear();
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP, {});
  WifiRuntimeTest::tick(lost, 2000);
  CHECK(phase(lost) == NetworkPhase::ENTER_ADMIN);
  WifiRuntimeTest::tick(lost, 3200);
  CHECK(phase(lost) == NetworkPhase::FAULT && !admitted(lost));
  CHECK(lost.adminAccess().generation() == 0);
  CHECK(lost.status().config_error == 11);
  CHECK(lost.status().fault_site == code(FaultSite::ROLE_ACK_TIMEOUT));
  CHECK(lost.status().fault_expected != 0); // the unacknowledged bit is reported
}

void unexpectedEventsStillFault() {
  // An AP stop nobody commanded, while administration is open.
  seedApOnly();
  WifiManager m;
  bootToAdmin(m);
  const auto generation = m.adminAccess().generation();
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP, {});
  CHECK(!m.adminAccess().allowed(generation)); // revoked in the event task
  WifiRuntimeTest::tick(m, 100);
  CHECK(phase(m) == NetworkPhase::FAULT);
  CHECK(m.status().config_error == 11 && !m.status().ap_active);
  CHECK(m.status().fault_site == code(FaultSite::UNEXPECTED_ROLE_EVENT));
  CHECK(m.status().fault_event == 1 && m.status().fault_expected == 0);
  CHECK(m.status().fault_count == 1);
  CHECK(!m.provisionAp("refused-while-fault") && m.lastPost() == PostResult::NOT_ADMITTED);

  // Fail-closed on the air: until the listeners are confirmed closed nothing
  // is commanded; then the radio is switched off, once.
  WifiRuntimeTest::tick(m, 150);
  CHECK(WiFi.mode_value == WIFI_AP);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 200);
  CHECK(WiFi.mode_value == WIFI_OFF && !WiFi.AP.started());
  WifiRuntimeTest::tick(m, 250);
  CHECK(phase(m) == NetworkPhase::FAULT); // never reopens by itself
  CHECK(m.status().driver_mode == 0 && !m.status().driver_ap_started);
  for (uint32_t t : {1000u, 700000u}) WifiRuntimeTest::tick(m, t);
  CHECK(phase(m) == NetworkPhase::FAULT && WiFi.mode_value == WIFI_OFF);

  // Explicit request: full bring-up again, error cleared, history kept.
  CHECK(m.requestAp(true));
  WifiRuntimeTest::tick(m, 700050);
  CHECK(phase(m) == NetworkPhase::ENTER_ADMIN);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 700100);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && admitted(m));
  CHECK(m.adminAccess().generation() != generation);
  CHECK(m.status().config_error == 0 && m.status().fault_site == 0);
  CHECK(m.status().last_fault_site == code(FaultSite::UNEXPECTED_ROLE_EVENT));
  CHECK(m.status().fault_count == 1 && driverKey() == kKeyA);

  // A third AP event during the profile command is not part of its pair.
  seedApOnly();
  WifiManager extra;
  WifiRuntimeTest::initialize(extra);
  WiFi.deliver_events = false;
  closeListeners(extra);
  WifiRuntimeTest::tick(extra, 50);
  WiFi.drainEvents();
  WifiRuntimeTest::tick(extra, 100);
  WiFi.drainEvents();
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_START, {});
  WifiRuntimeTest::tick(extra, 150);
  CHECK(phase(extra) == NetworkPhase::FAULT && !admitted(extra));
  CHECK(extra.status().fault_event == 8);

  // A station role appearing during bring-up is not acknowledged by anything.
  seedApOnly();
  WifiManager sta;
  WifiRuntimeTest::initialize(sta);
  closeListeners(sta);
  WiFi.callback(ARDUINO_EVENT_WIFI_STA_START, {});
  WifiRuntimeTest::tick(sta, 50);
  CHECK(phase(sta) == NetworkPhase::FAULT && !admitted(sta));
  CHECK(sta.status().fault_event == 2);
}

void driverProfileIsVerified() {
  // The driver ends up holding another key: no generation is ever opened.
  seedApOnly();
  WifiManager m;
  WifiRuntimeTest::initialize(m);
  WiFi.ap_store_corrupt = true;
  closeListeners(m);
  WifiRuntimeTest::tick(m, 50);
  CHECK(phase(m) == NetworkPhase::FAULT && !admitted(m));
  CHECK(m.adminAccess().generation() == 0 && !m.status().ap_active);
  CHECK(m.status().config_error == 12);
  CHECK(m.status().fault_site == code(FaultSite::AP_CONFIG_MISMATCH));
  CHECK(!m.status().ap_config_verified);

  // The driver refuses the profile: named fault, no stale acknowledgment left.
  seedApOnly();
  WifiManager refused;
  WifiRuntimeTest::initialize(refused);
  WiFi.ap_set_ok = false;
  closeListeners(refused);
  WifiRuntimeTest::tick(refused, 50);
  CHECK(phase(refused) == NetworkPhase::FAULT && refused.status().config_error == 3);
  CHECK(refused.status().fault_site == code(FaultSite::AP_SET_CONFIG));
  WiFi.ap_set_ok = true;
  CHECK(refused.requestAp(true));
  WifiRuntimeTest::tick(refused, 100);
  closeListeners(refused);
  WifiRuntimeTest::tick(refused, 150);
  CHECK(phase(refused) == NetworkPhase::ADMIN_AP && refused.status().config_error == 0);

  // The profile cannot be read back: unknown driver state denies.
  seedApOnly();
  WifiManager unreadable;
  WifiRuntimeTest::initialize(unreadable);
  WiFi.ap_get_ok = false;
  closeListeners(unreadable);
  WifiRuntimeTest::tick(unreadable, 50);
  CHECK(phase(unreadable) == NetworkPhase::FAULT && !admitted(unreadable));
  CHECK(WiFi.ap_set_calls == 0);
}

void provisioningIsConfirmedPerRequest() {
  seedApOnly();
  WifiManager m;
  // Refused before administration exists, with the reason and no sequence.
  WifiRuntimeTest::initialize(m);
  CHECK(!m.provisionAp(kKeyB));
  CHECK(m.lastPost() == PostResult::NOT_ADMITTED && m.lastPostSeq() == 0);
  CHECK(!m.provisionAp("short") && m.lastPost() == PostResult::INVALID);
  CHECK(!m.provisionAdmin("not-hex") && m.lastPost() == PostResult::INVALID);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 50);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && m.status().tx_seq == 0);

  // Critical context refuses at the door.
  m.setContext(true, false);
  CHECK(!m.provisionAp(kKeyB) && m.lastPost() == PostResult::CRITICAL);
  m.setContext(false, false);

  // Accepted: sequence N. A second command before the worker ran is BUSY.
  CHECK(m.provisionAp(kKeyB));
  const uint32_t seq = m.lastPostSeq();
  CHECK(m.lastPost() == PostResult::QUEUED && seq != 0);
  std::string token(64, 'a');
  CHECK(!m.provisionAdmin(token.c_str()));
  CHECK(m.lastPost() == PostResult::BUSY && m.lastPostSeq() == 0);
  CHECK(m.status().tx_seq == 0); // nothing confirmed yet
  WifiRuntimeTest::tick(m, 100);
  CHECK(m.status().tx_seq == seq && kindIs(m, "AP_KEY"));
  CHECK(m.status().tx_result == code(TxResult::COMMITTED));
  CHECK(m.status().config_phase == ConfigPhase::COMMITTED && m.status().config_error == 0);
  CHECK(storedKey() == kKeyB && commits == 1);

  // THE false PASS: the AP is now cycling to apply the key. A command sent in
  // this window is refused. The phase still reads COMMITTED from the request
  // before, but neither the acknowledgment nor the sequence can be mistaken.
  WifiRuntimeTest::tick(m, 150);
  CHECK(phase(m) == NetworkPhase::LEAVE_ADMIN);
  CHECK(!m.provisionAdmin(token.c_str()));
  CHECK(m.lastPost() == PostResult::NOT_ADMITTED && m.lastPostSeq() == 0);
  CHECK(m.status().config_phase == ConfigPhase::COMMITTED); // stale, by design
  CHECK(m.status().tx_seq == seq && kindIs(m, "AP_KEY"));   // still request N
  CHECK(commits == 1);

  // Complete the reload; then the admin key is accepted as request N+1.
  closeListeners(m);
  WifiRuntimeTest::tick(m, 200);
  CHECK(phase(m) == NetworkPhase::ENTER_ADMIN);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 250);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && admitted(m));
  CHECK(m.provisionAdmin(token.c_str()));
  const uint32_t admin_seq = m.lastPostSeq();
  CHECK(admin_seq > seq);
  WifiRuntimeTest::tick(m, 300);
  CHECK(m.status().tx_seq == admin_seq && kindIs(m, "ADMIN_KEY"));
  CHECK(m.status().tx_result == code(TxResult::COMMITTED) && commits == 2);
  CHECK(storedKey() == kKeyB); // the AP key survived the admin rotation

  // Worker-side rejection: queued while admitted, revoked before it ran.
  closeListeners(m);
  WifiRuntimeTest::tick(m, 350);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 400);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP);
  CHECK(m.provisionAp("queued-then-revoked"));
  const uint32_t revoked_seq = m.lastPostSeq();
  m.adminAccess().revoke();
  WifiRuntimeTest::tick(m, 450);
  CHECK(m.status().tx_seq == revoked_seq);
  CHECK(m.status().tx_result == code(TxResult::REJECTED));
  CHECK(m.status().config_error == 5 && !m.configBusy());
  CHECK(storedKey() == kKeyB && commits == 2);
}

void keyRotationWhileActive() {
  seedApOnly();
  WifiManager m;
  bootToAdmin(m);
  const auto generation = m.adminAccess().generation();
  CHECK(m.provisionAp(kKeyB));
  WifiRuntimeTest::tick(m, 100);
  CHECK(storedKey() == kKeyB);
  CHECK(driverKey() == kKeyA); // not on the air yet: the reload is pending
  CHECK(m.status().ap_reload_pending);
  WifiRuntimeTest::tick(m, 150);
  CHECK(phase(m) == NetworkPhase::LEAVE_ADMIN && !m.adminAccess().allowed(generation));
  closeListeners(m);
  WifiRuntimeTest::tick(m, 200);
  CHECK(WiFi.mode_value == WIFI_STA && phase(m) == NetworkPhase::ENTER_ADMIN);
  closeListeners(m);
  WifiRuntimeTest::tick(m, 250);
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && admitted(m));
  CHECK(m.adminAccess().generation() != generation);
  CHECK(driverKey() == kKeyB && WiFi.ap_set_calls == 2); // rewritten once
  CHECK(m.status().config_error == 0 && m.status().fault_count == 0);
  CHECK(m.status().ap_config_verified && !m.status().ap_reload_pending);

  // Requested == persisted == on the air, without ever printing a key.
  CHECK(m.verifyAp(kKeyB));
  uint32_t seq = m.lastPostSeq();
  WifiRuntimeTest::tick(m, 300);
  CHECK(m.status().verify_seq == seq);
  CHECK(m.status().verify_stored == code(VerifyResult::MATCH));
  CHECK(m.status().verify_driver == code(VerifyResult::MATCH));
  // Rate limited, and the old key is a MISMATCH everywhere.
  CHECK(m.verifyAp(kKeyA));
  seq = m.lastPostSeq();
  WifiRuntimeTest::tick(m, 350);
  CHECK(m.status().verify_seq == seq);
  CHECK(m.status().verify_stored == code(VerifyResult::THROTTLED));
  CHECK(m.verifyAp(kKeyA));
  seq = m.lastPostSeq();
  WifiRuntimeTest::tick(m, 2400);
  CHECK(m.status().verify_seq == seq);
  CHECK(m.status().verify_stored == code(VerifyResult::MISMATCH));
  CHECK(m.status().verify_driver == code(VerifyResult::MISMATCH));
  // A prefix is not a match.
  CHECK(m.verifyAp("rotated-device-key-"));
  WifiRuntimeTest::tick(m, 4500);
  CHECK(m.status().verify_stored == code(VerifyResult::MISMATCH));
  CHECK(!m.verifyAp("short") && m.lastPost() == PostResult::INVALID);
  // Comparing changes nothing.
  CHECK(phase(m) == NetworkPhase::ADMIN_AP && commits == 1 && WiFi.ap_set_calls == 2);

  // Restart: a fresh driver, the same NVS. The persisted key is the one applied.
  const int writes = sets;
  const auto saved = blobs;
  WiFi = WifiStub{};
  blobs = saved;
  now = 0;
  WifiManager rebooted;
  bootToAdmin(rebooted);
  CHECK(sets == writes); // boot never writes
  CHECK(driverKey() == kKeyB && storedKey() == kKeyB);
  CHECK(rebooted.status().nvs_active && rebooted.status().ap_provisioned);
  CHECK(rebooted.status().config_error == 0 && rebooted.status().fault_count == 0);
  CHECK(rebooted.verifyAp(kKeyB));
  WifiRuntimeTest::tick(rebooted, 100);
  CHECK(rebooted.status().verify_stored == code(VerifyResult::MATCH));
  CHECK(rebooted.status().verify_driver == code(VerifyResult::MATCH));
  CHECK(rebooted.verifyAp(kKeyA));
  WifiRuntimeTest::tick(rebooted, 2200);
  CHECK(rebooted.status().verify_stored == code(VerifyResult::MISMATCH));

  // With the radio off the stored key is still comparable; the driver is not.
  CHECK(rebooted.setEnabled(false, 2300));
  WifiRuntimeTest::tick(rebooted, 2300);
  closeListeners(rebooted);
  WifiRuntimeTest::tick(rebooted, 2350);
  CHECK(phase(rebooted) == NetworkPhase::CLOSED && WiFi.mode_value == WIFI_OFF);
  CHECK(rebooted.verifyAp(kKeyB));
  WifiRuntimeTest::tick(rebooted, 4400);
  CHECK(rebooted.status().verify_stored == code(VerifyResult::MATCH));
  CHECK(rebooted.status().verify_driver == code(VerifyResult::NOT_AVAILABLE));
}

// md_http_life holds the admission fence for the length of its tick. A request
// that meets it held is retried, bounded, and never dropped without a result.
void fenceContention() {
  seedApOnly();
  WifiManager m;
  bootToAdmin(m);
  CHECK(m.provisionAp(kKeyB));
  const uint32_t seq = m.lastPostSeq();
  {
    AdminLease held(m.adminAccess());
    CHECK(bool(held));
    WifiRuntimeTest::tick(m, 100);
    WifiRuntimeTest::tick(m, 150);
    CHECK(commits == 0 && m.configBusy() && WifiRuntimeTest::held(m));
    CHECK(m.status().tx_seq == 0);
  }
  WifiRuntimeTest::tick(m, 200);
  CHECK(m.status().tx_seq == seq && commits == 1 && !WifiRuntimeTest::held(m));
  CHECK(m.status().tx_result == code(TxResult::COMMITTED));

  // A fence that never frees: bounded refusal carrying the request's sequence.
  seedApOnly();
  WifiManager stuck;
  bootToAdmin(stuck);
  CHECK(stuck.provisionAp(kKeyB));
  const uint32_t stuck_seq = stuck.lastPostSeq();
  {
    AdminLease held(stuck.adminAccess());
    for (uint32_t i = 0; i < 45; ++i) WifiRuntimeTest::tick(stuck, 100 + i * 50);
    CHECK(!WifiRuntimeTest::held(stuck) && !stuck.configBusy());
    CHECK(stuck.status().tx_seq == stuck_seq);
    CHECK(stuck.status().tx_result == code(TxResult::REJECTED));
    CHECK(stuck.status().config_error == 5 && commits == 0);
  }
  CHECK(storedKey() == kKeyA);

  // An AP request queued while the fence is held is applied afterwards.
  seedApOnly();
  WifiManager ap;
  bootToAdmin(ap);
  WiFi.callback(ARDUINO_EVENT_WIFI_AP_STOP, {});
  WifiRuntimeTest::tick(ap, 100);
  CHECK(phase(ap) == NetworkPhase::FAULT);
  CHECK(ap.requestAp(true) && ap.lastPost() == PostResult::QUEUED);
  {
    AdminLease held(ap.adminAccess());
    WifiRuntimeTest::tick(ap, 150);
    CHECK(phase(ap) == NetworkPhase::FAULT && WifiRuntimeTest::held(ap));
  }
  WifiRuntimeTest::tick(ap, 200);
  CHECK(phase(ap) == NetworkPhase::ENTER_ADMIN && !WifiRuntimeTest::held(ap));
}

int main() {
  doubleReproducesTheDriver();
  bootWithoutStaProfiles();
  profileRestartIsAcknowledged();
  unexpectedEventsStillFault();
  driverProfileIsVerified();
  provisioningIsConfirmedPerRequest();
  keyRotationWhileActive();
  fenceContention();
  printf("WIFI_AP_BRINGUP: %d checks, %d failures (driver/NVS doubles; no radio proof)\n",
         checks, failures);
  return failures ? 1 : 0;
}
