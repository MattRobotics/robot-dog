#include "WifiManager.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "../config/WifiCredentials.h"
#include "../update/Sha256.h"
namespace matdog {
namespace network {
namespace {
constexpr uint8_t kApStop = 1, kStaStart = 2, kStaStop = 4, kApStart = 8;
constexpr uint32_t kRoleEventTimeoutMs = 3000;
// The admission fence is a zero-wait mutex shared with md_http_life. A request
// that meets it held is retried for this many worker periods, then refused.
constexpr uint32_t kFenceRetryLimit = 40;
constexpr uint32_t kVerifyIntervalMs = 2000;
void copy(char *out, size_t cap, const char *in) {
  size_t n = strnlen(in, cap - 1);
  memset(out, 0, cap);
  memcpy(out, in, n);
}
// Constant-time equality of two zero-padded 65-byte key buffers.
bool sameKey(const char *a, const char *b) {
  uint8_t diff = 0;
  for (size_t i = 0; i < 65; ++i)
    diff |= static_cast<uint8_t>(a[i]) ^ static_cast<uint8_t>(b[i]);
  return diff == 0;
}
// The eight fields the installed core itself compares before deciding whether
// a soft-AP profile has to be written to the driver.
bool sameApProfile(const wifi_config_t &a, const wifi_config_t &b) {
  return strncmp(reinterpret_cast<const char *>(a.ap.ssid),
                 reinterpret_cast<const char *>(b.ap.ssid), 32) == 0 &&
         strncmp(reinterpret_cast<const char *>(a.ap.password),
                 reinterpret_cast<const char *>(b.ap.password), 64) == 0 &&
         a.ap.channel == b.ap.channel && a.ap.authmode == b.ap.authmode &&
         a.ap.ssid_hidden == b.ap.ssid_hidden &&
         a.ap.max_connection == b.ap.max_connection &&
         a.ap.pairwise_cipher == b.ap.pairwise_cipher &&
         a.ap.ftm_responder == b.ap.ftm_responder;
}
void macText(const uint8_t *mac, char out[18]) {
  if (!mac) {
    out[0] = 0;
    return;
  }
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
}
} // namespace
void WifiManager::begin(uint32_t now_ms) {
  (void)now_ms;
  if (!admin_.begin()) { status_.config_error = 2; return; }
  queue_ = xQueueCreate(1, sizeof(Request));
  mutex_ = xSemaphoreCreateMutex();
  if (!queue_ || !mutex_) {
    status_.config_error = 1;
    if (queue_)
      vQueueDelete(static_cast<QueueHandle_t>(queue_));
    if (mutex_)
      vSemaphoreDelete(static_cast<SemaphoreHandle_t>(mutex_));
    queue_ = mutex_ = nullptr;
    return;
  }
  if (xTaskCreatePinnedToCore(taskEntry, "md_network", 8192, this, 1, nullptr,
                              0) != pdPASS) {
    status_.config_error = 2;
    vQueueDelete(static_cast<QueueHandle_t>(queue_));
    vSemaphoreDelete(static_cast<SemaphoreHandle_t>(mutex_));
    queue_ = mutex_ = nullptr;
  }
}
void WifiManager::taskEntry(void *self) {
  static_cast<WifiManager *>(self)->worker();
}
void WifiManager::initializeWorker() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  const bool stored = storage_.load(&config_);
  worker_status_.nvs_active = stored;
  if (storage_.lastLoadError() != 0) {
    nvs_fault_.store(true);
    worker_status_.config_error = 10;
    worker_status_.config_phase = ConfigPhase::STORAGE_ERROR;
  }
  if (!stored && config::kWifiCredentialsPresent) {
    copy(config_.sta[0].ssid, 33, config::kWifiSsid);
    copy(config_.sta[0].password, 65, config::kWifiPassword);
    config_.sta[0].enabled = true;
  }
  transaction_.begin(&storage_, config_);
  admin_.setDigest(config_.admin_digest); // no generation is admitted at boot
  WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      disconnect_reason_.store(info.wifi_sta_disconnected.reason);
      sta_reason_.store(info.wifi_sta_disconnected.reason);
      sta_link_.store(false);
      sta_disconnects_.fetch_add(1);
    }
    if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      // The router accepted the credentials; an earlier attempt's reason no
      // longer describes the link.
      sta_reason_.store(0);
      sta_associated_.store(true);
      sta_link_.store(true);
    }
    observeRadioEvent(event);
  });
  event_since_ = millis();
  expected_events_.store(kStaStart); // initial driver start, before WiFi initializes it
  wifi_mode_t initial_mode = WIFI_MODE_NULL;
  if (WiFi.mode(WIFI_STA) && esp_wifi_get_mode(&initial_mode) == ESP_OK &&
      initial_mode == WIFI_MODE_STA) {
    admin_.phase.store(NetworkPhase::CLIENT_STA);
    admin_.closing.store(false);
  } else {
    admin_.phase.store(NetworkPhase::FAULT);
    noteFault(FaultSite::INIT_MODE_FAILED, 0);
  }
  uint8_t mac[6]{};
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  snprintf(worker_status_.ap_ssid, 33, "MATDOG-%02X%02X%02X", mac[3], mac[4],
           mac[5]);
  policy_.begin(
      {kRadioSettleMs, kConnectTimeoutMs, kBackoffInitialMs, kBackoffMaxMs},
      config_.sta[0].enabled || config_.sta[1].enabled, millis());
  profile_ = config_.sta[0].enabled ? 0 : 1;
}
void WifiManager::worker() {
  initializeWorker();
  for (;;) {
    const uint32_t start = micros();
    workerTick(millis());
    uint32_t elapsed = micros() - start;
    if (elapsed > worker_status_.worker_max_us)
      worker_status_.worker_max_us = elapsed;
    worker_status_.worker_stack_free = uxTaskGetStackHighWaterMark(nullptr);
    if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), 0) == pdTRUE) {
      shared_status_ = worker_status_;
      memcpy(shared_admin_digest_, config_.admin_digest, 32);
      xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
// Every refusal has a named reason, and every accepted request a sequence
// number. The worker publishes its outcome under that number, so a result left
// behind by an earlier request can never be read as confirmation of this one.
bool WifiManager::post(const Request &r, bool mutation) {
  last_post_seq_ = 0;
  if (!queue_) { last_post_ = PostResult::UNAVAILABLE; return false; }
  if (mutation && context_.load() != 0) { last_post_ = PostResult::CRITICAL; return false; }
  if (mutation && nvs_fault_) { last_post_ = PostResult::NVS_FAULT; return false; }
  const bool bootstrap = (r.kind == RequestKind::AP_KEY || r.kind == RequestKind::ADMIN_KEY) &&
      admin_.phase.load() == NetworkPhase::CLOSED && admin_.listeners_closed.load();
  if (mutation && !bootstrap && (!admin_.allowed(admin_.generation()) || !verifyAdminRadio())) {
    last_post_ = PostResult::NOT_ADMITTED;
    return false;
  }
  if (mutation && config_busy_.exchange(true)) {
    last_post_ = PostResult::BUSY;
    return false;
  }
  Request tagged = r;
  tagged.seq = ++request_seq_;
  if (mutation && !bootstrap && !tagged.generation) tagged.generation = admin_.generation();
  if (xQueueSend(static_cast<QueueHandle_t>(queue_), &tagged, 0) != pdTRUE) {
    if (mutation)
      config_busy_.store(false);
    last_post_ = PostResult::QUEUE_FULL;
    return false;
  }
  memset(tagged.secret, 0, sizeof(tagged.secret));
  last_post_ = PostResult::QUEUED;
  last_post_seq_ = tagged.seq;
  return true;
}
bool WifiManager::setEnabled(bool enabled, uint32_t now_ms) {
  if (configBusy()) { last_post_ = PostResult::BUSY; last_post_seq_ = 0; return false; }
  (void)now_ms;
  Request r{};
  r.kind = RequestKind::ENABLE;
  r.on = enabled;
  const bool ok = post(r, false);
  publishPolicyState();
  return ok;
}
void WifiManager::publishPolicyState() {
  status_.config_busy = config_busy_.load();
} // queued intent; radio state changes only in worker
bool WifiManager::requestScan() {
  if (admin_.phase.load() != NetworkPhase::CLIENT_STA ||
      context_.load() || configBusy())
    return false;
  Request r{};
  r.kind = RequestKind::SCAN;
  return post(r, false);
}
bool WifiManager::requestAp(bool on, uint32_t generation) {
  last_post_seq_ = 0;
  if (configBusy()) { last_post_ = PostResult::BUSY; return false; }
  if (generation && !admin_.allowed(generation)) { last_post_ = PostResult::NOT_ADMITTED; return false; }
  // Closing the AP is not an operator request any more: only the role policy
  // closes it (RUN with a usable router). In MAINTENANCE it is mandatory, and
  // a recovery AP is left by the policy alone.
  if (!on) { last_post_ = PostResult::INVALID; return false; }
  Request r{};
  r.kind = RequestKind::AP;
  r.generation = generation;
  r.on = on;
  return post(r, false);
}
bool WifiManager::setSleep(uint8_t p) {
  if (p > 2) { last_post_ = PostResult::INVALID; return false; }
  Request r{};
  r.kind = RequestKind::SLEEP;
  r.preference = p;
  return post(r, true);
}
bool WifiManager::provisionAp(const char *p) {
  if (!p || strlen(p) < 8 || strlen(p) > 63) {
    last_post_ = PostResult::INVALID; last_post_seq_ = 0;
    return false;
  }
  Request r{};
  r.kind = RequestKind::AP_KEY;
  copy(r.secret, 65, p);
  const bool ok = post(r, true);
  memset(r.secret, 0, sizeof(r.secret));
  return ok;
}
bool WifiManager::provisionAdmin(const char *p) {
  uint8_t parsed[32];
  if (!p || strlen(p) != 64 || !update::parseHex(p, 64, parsed)) {
    last_post_ = PostResult::INVALID; last_post_seq_ = 0;
    return false;
  }
  memset(parsed, 0, sizeof(parsed));
  Request r{};
  r.kind = RequestKind::ADMIN_KEY;
  copy(r.secret, 65, p);
  const bool ok = post(r, true);
  memset(r.secret, 0, sizeof(r.secret));
  return ok;
}
// Read-only: admitted in every phase, never changes configuration or radio.
bool WifiManager::verifyAp(const char *p) {
  if (!p || strlen(p) < 8 || strlen(p) > 63) {
    last_post_ = PostResult::INVALID; last_post_seq_ = 0;
    return false;
  }
  Request r{};
  r.kind = RequestKind::AP_VERIFY;
  copy(r.secret, 65, p);
  const bool ok = post(r, false);
  memset(r.secret, 0, sizeof(r.secret));
  return ok;
}
bool WifiManager::configure(const ConfigPatch &p, uint32_t generation) {
  if (generation && !admin_.allowed(generation)) return false;
  if (!generation && admin_.phase.load() != NetworkPhase::ADMIN_AP) return false;
  Request r{};
  r.kind = RequestKind::CONFIG;
  r.generation = generation ? generation : admin_.generation();
  r.patch = p;
  return post(r, true);
}
void WifiManager::update(uint32_t now_ms) {
  (void)now_ms;
  const uint32_t start = micros();
  if (mutex_ &&
      xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), 0) == pdTRUE) {
    status_ = shared_status_;
    memcpy(admin_digest_, shared_admin_digest_, 32);
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
  publishPolicyState();
  status_.last_update_us = micros() - start;
  if (status_.last_update_us > snapshot_max_us_)
    snapshot_max_us_ = status_.last_update_us;
  status_.max_update_us = snapshot_max_us_;
}
bool WifiManager::verifyAdminRadio() {
  wifi_mode_t mode = WIFI_MODE_NULL;
  wifi_ap_record_t record{};
  const esp_err_t sta = esp_wifi_sta_get_ap_info(&record);
  return esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_AP &&
         WiFi.AP.started() && !WiFi.STA.started() &&
         WiFi.status() != WL_CONNECTED && static_cast<uint32_t>(WiFi.localIP()) == 0 &&
         (sta == ESP_ERR_WIFI_NOT_CONNECT || sta == ESP_ERR_WIFI_CONN) &&
         static_cast<uint32_t>(WiFi.softAPIP()) == static_cast<uint32_t>(IPAddress(192,168,4,1));
}
// Arduino delivers role events asynchronously. Each owned command must consume
// its complete role-change acknowledgment before another command can be issued.
// Events have no epoch tag: serialization, rather than a time delay, separates
// generations. Duplicate/unexpected role events revoke immediately.
void WifiManager::observeRadioEvent(int event) {
  uint8_t bit = 0;
  if (event == ARDUINO_EVENT_WIFI_AP_STOP) bit = kApStop;
  if (event == ARDUINO_EVENT_WIFI_STA_START) bit = kStaStart;
  if (event == ARDUINO_EVENT_WIFI_STA_STOP) bit = kStaStop;
  if (event == ARDUINO_EVENT_WIFI_AP_START) bit = kApStart;
  if (!bit) return;
  auto pending = expected_events_.load();
  while (pending & bit) {
    if (expected_events_.compare_exchange_weak(pending, pending & ~bit)) return;
  }
  // Not part of any armed command: which role event, and what was still armed.
  admin_.raiseFault(FaultSite::UNEXPECTED_ROLE_EVENT, bit, pending);
}
bool WifiManager::changeMode(uint8_t target, uint32_t now) {
  if (target != WIFI_OFF && target != WIFI_STA && target != WIFI_AP) return false;
  wifi_mode_t current = WIFI_MODE_NULL;
  if (expected_events_.load()) return false;
  const auto read = esp_wifi_get_mode(&current);
  // WIFI_OFF deinitializes the stock driver. NOT_INIT is a known closed
  // state only when both interface roles are stopped; other errors deny.
  if (read != ESP_OK && (read != ESP_ERR_WIFI_NOT_INIT ||
      WiFi.AP.started() || WiFi.STA.started())) return false;
  const bool old_ap = (current & WIFI_MODE_AP) != 0;
  const bool old_sta = (current & WIFI_MODE_STA) != 0;
  const bool new_ap = target == WIFI_AP;
  const bool new_sta = target == WIFI_STA;
  const uint8_t expected = (old_ap && !new_ap ? kApStop : 0) |
      (!old_sta && new_sta ? kStaStart : 0) | (old_sta && !new_sta ? kStaStop : 0) |
      (!old_ap && new_ap ? kApStart : 0);
  event_since_ = now;
  expected_events_.store(expected); // arm before the driver can enqueue events
  return WiFi.mode(static_cast<wifi_mode_t>(target));
}
void WifiManager::startAp(uint32_t now) {
  (void)now;
  if (admin_.phase.load() == NetworkPhase::ADMIN_AP ||
      admin_.phase.load() == NetworkPhase::ENTER_ADMIN || !config_.ap_password[0]) return;
  admin_.revoke();
  transition_admin_ = true;
  ap_step_ = ApStep::IDLE;
  fault_close_issued_ = false;
  worker_status_.fault_site = 0; // a new explicit request ends the fault episode
  admin_.phase.store(NetworkPhase::ENTER_ADMIN);
}
void WifiManager::leaveAdmin(bool return_admin, uint32_t now) {
  (void)now;
  admin_.revoke();
  transition_admin_ = false;
  return_admin_ = return_admin;
  leave_command_issued_ = false;
  ap_step_ = ApStep::IDLE;
  fault_close_issued_ = false;
  worker_status_.fault_site = 0;
  admin_.phase.store(NetworkPhase::LEAVE_ADMIN);
}
void WifiManager::stopAp() { leaveAdmin(false, millis()); }
void WifiManager::fault(FaultSite site, int32_t code) {
  admin_.revoke();
  admin_.phase.store(NetworkPhase::FAULT);
  noteFault(site, code);
}
// The first cause of a FAULT episode is kept; the numeric code stays the
// legacy config_error (3 driver refusal, 11 role/verification, 12 profile).
void WifiManager::noteFault(FaultSite site, int32_t code) {
  auto &w = worker_status_;
  if (code) w.config_error = code;
  if (w.fault_site) return;
  w.fault_site = w.last_fault_site = static_cast<uint8_t>(site);
  w.fault_expected = expected_events_.load();
  if (site != FaultSite::UNEXPECTED_ROLE_EVENT) w.fault_event = 0;
  ++w.fault_count;
}
// A verified stable state supersedes an earlier radio-class error, so the
// published code describes the present and not a fault that was recovered.
void WifiManager::clearRadioError() {
  auto &w = worker_status_;
  if (w.config_error == 3 || w.config_error == 11 || w.config_error == 12)
    w.config_error = 0;
  w.fault_site = 0;
}
// FAULT is fail-closed on the air as well: once the listeners are confirmed
// closed and no acknowledgment is outstanding, the radio is switched off once.
// A network that stays visible while administration is unavailable is not a
// state this firmware keeps. Reopening still needs an explicit request.
void WifiManager::faultClose(uint32_t now) {
  if (fault_close_issued_) return;
  AdminLease fence(admin_);
  if (!fence || !admin_.listeners_closed.load()) return;
  fault_close_issued_ = true; // one bounded attempt per fault episode
  wifi_mode_t mode = WIFI_MODE_NULL;
  if (esp_wifi_get_mode(&mode) != ESP_OK || mode == WIFI_MODE_NULL) return;
  connect_due_ = false;
  if (worker_status_.scan_running) {
    esp_wifi_scan_stop(); WiFi.scanDelete(); worker_status_.scan_running = false;
  }
  WiFi.setAutoReconnect(false);
  changeMode(WIFI_OFF, now);
}
// Soft-AP profile exactly as the installed core builds it, except that the
// authentication mode is unconditional: this AP is never opened without WPA2.
static void buildApProfile(wifi_config_t *out, const char *ssid, const char *key) {
  memset(out, 0, sizeof(*out));
  const size_t ssid_len = strnlen(ssid, 32);
  memcpy(out->ap.ssid, ssid, ssid_len);
  out->ap.ssid_len = static_cast<uint8_t>(ssid_len);
  memcpy(out->ap.password, key, strnlen(key, 63));
  out->ap.channel = 1;
  out->ap.max_connection = 4;
  out->ap.beacon_interval = 100;
  out->ap.ssid_hidden = 0;
  out->ap.authmode = WIFI_AUTH_WPA2_PSK;
  out->ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
}
// Second owned command of the bring-up. esp_wifi_set_config() on a started
// soft-AP stops and restarts it inside the driver, which reports AP_STOP and
// then AP_START. Those two role events are the acknowledgment set of THIS
// command and are armed before the driver can enqueue them. When the driver
// already holds the identical profile nothing is written and nothing is armed.
bool WifiManager::issueApConfig(uint32_t now) {
  wifi_config_t want, current;
  buildApProfile(&want, worker_status_.ap_ssid, config_.ap_password);
  memset(&current, 0, sizeof(current));
  bool ok = esp_wifi_get_config(WIFI_IF_AP, &current) == ESP_OK;
  if (ok && !sameApProfile(want, current)) {
    event_since_ = now;
    expected_events_.store(kApStop | kApStart);
    ok = esp_wifi_set_config(WIFI_IF_AP, &want) == ESP_OK;
    if (!ok) expected_events_.store(0);
  }
  memset(&want, 0, sizeof(want));
  memset(&current, 0, sizeof(current));
  return ok;
}
// The profile the driver is really using must equal the stored one before a
// generation is opened: same SSID, same key, WPA2-PSK.
bool WifiManager::apConfigApplied() {
  wifi_config_t want, current;
  buildApProfile(&want, worker_status_.ap_ssid, config_.ap_password);
  memset(&current, 0, sizeof(current));
  const bool ok = esp_wifi_get_config(WIFI_IF_AP, &current) == ESP_OK &&
                  current.ap.authmode == WIFI_AUTH_WPA2_PSK &&
                  sameApProfile(want, current);
  memset(&want, 0, sizeof(want));
  memset(&current, 0, sizeof(current));
  return ok;
}
void WifiManager::enterAdminTick(uint32_t now) {
  if (ap_step_ == ApStep::IDLE) {
    worker_status_.ap_config_verified = false;
    if (config_.ap_name[0]) copy(worker_status_.ap_ssid, 33, config_.ap_name);
    else {
      uint8_t mac[6]{};
      if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK) { fault(FaultSite::MAC_READ_FAILED, 0); return; }
      snprintf(worker_status_.ap_ssid, 33, "MATDOG-%02X%02X%02X", mac[3], mac[4], mac[5]);
    }
    // Command 1: role change only. WIFI_MODE_AP removes the STA driver/interface
    // role; no hidden enableSTA.
    if (!changeMode(WIFI_AP, now)) { fault(FaultSite::AP_MODE_COMMAND, 3); return; }
    ap_since_ = now;
    ap_step_ = ApStep::MODE_ISSUED;
  }
  if (ap_step_ == ApStep::MODE_ISSUED) {
    // AP_START is asynchronous: the profile is not touched until the role
    // change is completely acknowledged and the AP interface reports started.
    if (expected_events_.load() || !WiFi.AP.started()) {
      if (now - ap_since_ >= kRoleEventTimeoutMs) fault(FaultSite::AP_START_TIMEOUT, 11);
      return;
    }
    if (!WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0))) {
      fault(FaultSite::AP_IP_CONFIG, 3); return;
    }
    // Command 2: the AP profile, with its own acknowledgment set.
    if (!issueApConfig(now)) { fault(FaultSite::AP_SET_CONFIG, 3); return; }
    ap_since_ = now;
    ap_step_ = ApStep::CONFIG_ISSUED;
  }
  // Keep ENTER_ADMIN until the stock status bits and exclusive driver mode can
  // both be verified, on this or a subsequent tick.
  if (expected_events_.load() || !verifyAdminRadio()) {
    if (now - ap_since_ >= kRoleEventTimeoutMs) fault(FaultSite::AP_VERIFY_TIMEOUT, 11);
    return;
  }
  if (!apConfigApplied()) { fault(FaultSite::AP_CONFIG_MISMATCH, 12); return; }
  if (!admin_.open()) { fault(FaultSite::ADMIN_OPEN_REFUSED, 0); return; }
  worker_status_.ap_active = true;
  worker_status_.ap_config_verified = true;
  ap_reload_ = false;
  ap_step_ = ApStep::IDLE;
  clearRadioError();
}
void WifiManager::transitionTick(uint32_t now) {
  if (admin_.radio_fault.exchange(false)) {
    const uint8_t source = admin_.fault_source.exchange(0);
    const bool first = !worker_status_.fault_site;
    fault(source ? static_cast<FaultSite>(source) : FaultSite::UNSPECIFIED, 11);
    if (first && source == static_cast<uint8_t>(FaultSite::UNEXPECTED_ROLE_EVENT)) {
      // Which role event arrived, and what was armed when it did.
      worker_status_.fault_event = admin_.fault_event.load();
      worker_status_.fault_expected = admin_.fault_expected.load();
    }
    if (intent_pending_) refuseStaTest(false);
    intent_pending_ = false;
    intent_ = NetworkConfig{};
    config_busy_.store(false);
  }
  if (admin_.phase.load() == NetworkPhase::FAULT) {
    if (!worker_status_.fault_site) {
      const uint8_t source = admin_.fault_source.exchange(0);
      noteFault(source ? static_cast<FaultSite>(source) : FaultSite::UNSPECIFIED, 0);
    }
    if (transaction_.busy()) {
      transaction_.update(now, false, false, true);
      worker_status_.config_phase = transaction_.phase();
      finishStaTest(now, true);
    } else if (intent_pending_) {
      refuseStaTest(false);
    }
    intent_pending_ = false;
    intent_ = NetworkConfig{};
    config_busy_.store(false);
    ap_reload_ = false;
    ap_step_ = ApStep::IDLE;
  }
  if (expected_events_.load()) {
    if (now - event_since_ >= kRoleEventTimeoutMs) fault(FaultSite::ROLE_ACK_TIMEOUT, 11);
    return;
  }
  const auto phase = admin_.phase.load();
  if (phase == NetworkPhase::FAULT) { faultClose(now); return; }
  if (phase == NetworkPhase::ADMIN_AP && !verifyAdminRadio()) {
    fault(FaultSite::ADMIN_RADIO_LOST, 11);
    return;
  }
  if (phase != NetworkPhase::ENTER_ADMIN && phase != NetworkPhase::LEAVE_ADMIN && phase != NetworkPhase::WAIT_AP_CLOSE) return;
  AdminLease fence(admin_);
  if (!fence || !admin_.listeners_closed.load()) return;
  // No command can alter a live listener's network. This worker is neither
  // an HTTPD callback nor the TCP/IP/event task.
  connect_due_ = false;
  if (worker_status_.scan_running) {
    esp_wifi_scan_stop(); WiFi.scanDelete(); worker_status_.scan_running = false;
  }
  WiFi.setAutoReconnect(false);
  if (transition_admin_) {
    enterAdminTick(now);
  } else {
    if (!enabled_) {
      if (!leave_command_issued_) {
        leave_command_issued_ = true;
        admin_.phase.store(NetworkPhase::WAIT_AP_CLOSE);
        if (!changeMode(WIFI_OFF, now)) { fault(FaultSite::MODE_COMMAND, 0); return; }
      }
      if (expected_events_.load()) return;
      wifi_mode_t closed_mode = WIFI_MODE_NULL;
      const auto read = esp_wifi_get_mode(&closed_mode);
      if ((read != ESP_OK && read != ESP_ERR_WIFI_NOT_INIT) ||
          closed_mode != WIFI_MODE_NULL || WiFi.AP.started() || WiFi.STA.started()) {
        if (now - event_since_ >= kRoleEventTimeoutMs) fault(FaultSite::CLOSE_VERIFY_TIMEOUT, 11);
        return;
      }
      worker_status_.ap_active = false;
      clearRadioError();
      admin_.phase.store(NetworkPhase::CLOSED); admin_.closing.store(false); return;
    }
    if (!leave_command_issued_) {
      leave_command_issued_ = true;
      admin_.phase.store(NetworkPhase::WAIT_AP_CLOSE);
      if (!changeMode(WIFI_STA, now)) { fault(FaultSite::MODE_COMMAND, 0); return; }
    }
    if (expected_events_.load()) return;
    wifi_mode_t mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode != WIFI_MODE_STA || WiFi.AP.started() || !WiFi.STA.started()) {
      if (now - event_since_ >= kRoleEventTimeoutMs) fault(FaultSite::STA_VERIFY_TIMEOUT, 11);
      return;
    }
    worker_status_.ap_active = false;
    clearRadioError();
    admin_.phase.store(NetworkPhase::CLIENT_STA);
    admin_.closing.store(false);
    if (intent_pending_) {
      intent_pending_ = false;
      // First persistent PENDING write occurs only after closure and STA mode.
      if (!transaction_.stage(intent_, intent_profile_, now, true, context_.load() != 0)) {
        worker_status_.config_error = 8;
        worker_status_.tx_result = static_cast<uint8_t>(TxResult::REJECTED);
        worker_status_.config_phase = transaction_.phase();
        config_busy_.store(false);
        if (transaction_.phase() == ConfigPhase::STORAGE_ERROR) nvs_fault_.store(true);
        refuseStaTest(transaction_.phase() == ConfigPhase::STORAGE_ERROR);
        startAp(now);
      } else {
        profile_ = intent_profile_;
        beginStaTest(now);
        connectProfile(now);
      }
      intent_ = NetworkConfig{};
    } else {
      policy_.adoptCredentials(config_.sta[0].enabled || config_.sta[1].enabled, false, now);
      sta_attempt_base_ = policy_.counters().connect_attempts; // a new router episode
      if (return_admin_) { return_admin_ = false; startAp(now); }
    }
  }
}
void WifiManager::connectProfile(uint32_t now) {
  if (admin_.phase.load() != NetworkPhase::CLIENT_STA) return;
  connect_due_ = true;
  test_fresh_ = false;
  connect_since_ = now;
  WiFi.disconnectAsync(false, false);
}
void WifiManager::issueConnect(uint32_t now) {
  (void)now;
  if (admin_.phase.load() != NetworkPhase::CLIENT_STA) return;
  const StaProfile &s = transaction_.busy()
                            ? transaction_.candidate().sta[profile_]
                            : config_.sta[profile_];
  if (!s.enabled)
    return;

  if (s.dhcp)
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  else if (!WiFi.config(IPAddress(s.ip), IPAddress(s.gateway),
                        IPAddress(s.mask), IPAddress(s.dns))) {
    worker_status_.config_error = 4;
    return;
  }
  // No channel or BSSID argument. Driver scans all channels and sorts signal
  // quality.
  wifi_config_t drv{};
  memcpy(drv.sta.ssid, s.ssid, strnlen(s.ssid, 32));
  memcpy(drv.sta.password, s.password, strnlen(s.password, 64));
  drv.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  drv.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  drv.sta.bssid_set = false;
  test_connect_ms_ = now;
  test_disconnects_ = sta_disconnects_.load();
  if (transaction_.busy() && worker_status_.sta_test.attempts < 255)
    ++worker_status_.sta_test.attempts;
  if (esp_wifi_set_config(WIFI_IF_STA, &drv) != ESP_OK ||
      esp_wifi_connect() != ESP_OK)
    policy_.reportActionFailed(WifiFault::CONNECT_CALL_REJECTED, millis());
  memset(&drv, 0, sizeof(drv));
}
// ---- STA test report ------------------------------------------------------
// The transaction decides COMMITTED/ROLLBACK exactly as before. These helpers
// only record what was observed, so the operator can tell a refused password
// from a missing router or a DHCP failure once the AP is back.
void WifiManager::beginStaTest(uint32_t now) {
  auto &t = worker_status_.sta_test;
  t = StaTestReport{};
  t.seq = intent_seq_;
  t.outcome = static_cast<uint8_t>(StaTestOutcome::RUNNING);
  t.profile = intent_profile_;
  copy(t.ssid, 33, transaction_.candidate().sta[intent_profile_].ssid);
  test_since_ = now;
  test_stable_ = false;
  sta_reason_.store(0);
  sta_associated_.store(false);
}
void WifiManager::trackStaTest(uint32_t now) {
  auto &t = worker_status_.sta_test;
  t.elapsed_ms = now - test_since_;
  t.reason = sta_reason_.load();
  t.associated = sta_associated_.load();
}
void WifiManager::finishStaTest(uint32_t now, bool aborted) {
  auto &t = worker_status_.sta_test;
  trackStaTest(now);
  const ConfigPhase phase = transaction_.phase();
  if (phase == ConfigPhase::COMMITTED) {
    t.outcome = static_cast<uint8_t>(StaTestOutcome::CONNECTED);
    t.ipv4 = static_cast<uint32_t>(WiFi.localIP());
    t.rssi_dbm = WiFi.RSSI();
    t.channel = WiFi.channel();
    t.rolled_back = false;
    return;
  }
  t.outcome = static_cast<uint8_t>(classifyStaTestFailure(
      phase == ConfigPhase::STORAGE_ERROR, aborted, t.associated, t.reason));
  // A failed ACTIVE write leaves storage in doubt; every other failure leaves
  // the known-good ACTIVE record untouched.
  t.rolled_back = phase == ConfigPhase::ROLLBACK;
}
// The accepted intent never reached the radio: nothing was tested.
void WifiManager::refuseStaTest(bool storage_error) {
  auto &t = worker_status_.sta_test;
  t = StaTestReport{};
  t.seq = intent_seq_;
  t.profile = intent_profile_;
  copy(t.ssid, 33, intent_.sta[intent_profile_ > 1 ? 0 : intent_profile_].ssid);
  t.outcome = static_cast<uint8_t>(
      classifyStaTestFailure(storage_error, true, false, 0));
  t.rolled_back = !storage_error;
}
void WifiManager::recordTx(const Request &r, TxResult result, int32_t error) {
  auto &w = worker_status_;
  w.tx_seq = r.seq;
  w.tx_kind = static_cast<uint8_t>(r.kind);
  w.tx_result = static_cast<uint8_t>(result);
  w.config_error = error;
}
// Compares, never reveals: the published result is MATCH or MISMATCH for the
// stored key and for the profile the driver is really using. Rate limited.
void WifiManager::verifyCandidate(const Request &r, uint32_t now) {
  auto &w = worker_status_;
  w.verify_seq = r.seq;
  if (verify_used_ && now - verify_since_ < kVerifyIntervalMs) {
    w.verify_stored = w.verify_driver = static_cast<uint8_t>(VerifyResult::THROTTLED);
    return;
  }
  verify_used_ = true;
  verify_since_ = now;
  char candidate[65];
  copy(candidate, 65, r.secret);
  w.verify_stored = static_cast<uint8_t>(
      !config_.ap_password[0] ? VerifyResult::NOT_AVAILABLE
      : sameKey(candidate, config_.ap_password) ? VerifyResult::MATCH
                                                : VerifyResult::MISMATCH);
  w.verify_driver = static_cast<uint8_t>(VerifyResult::NOT_AVAILABLE);
  wifi_mode_t mode = WIFI_MODE_NULL;
  wifi_config_t current;
  memset(&current, 0, sizeof(current));
  if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_AP && WiFi.AP.started() &&
      esp_wifi_get_config(WIFI_IF_AP, &current) == ESP_OK) {
    char active[65];
    memset(active, 0, sizeof(active));
    memcpy(active, current.ap.password, 64);
    w.verify_driver = static_cast<uint8_t>(
        current.ap.authmode == WIFI_AUTH_WPA2_PSK && sameKey(candidate, active)
            ? VerifyResult::MATCH : VerifyResult::MISMATCH);
    memset(active, 0, sizeof(active));
  }
  memset(candidate, 0, sizeof(candidate));
  memset(&current, 0, sizeof(current));
}
bool WifiManager::consume(const Request &r, uint32_t now) {
  if (r.kind == RequestKind::ENABLE) {
    if (context_.load() || transaction_.busy() || intent_pending_)
      return true;
    enabled_ = r.on;
    off_since_ = now;
    policy_.setEnabled(enabled_, now);
    if (!enabled_) {
      leaveAdmin(false, now);
    } else if (admin_.phase.load() == NetworkPhase::CLOSED) {
      leaveAdmin(false, now);
    }
    return true;
  }
  if (r.kind == RequestKind::SCAN) {
    scan(now, false);
    return true;
  }
  if (r.kind == RequestKind::AP_VERIFY) {
    verifyCandidate(r, now);
    return true;
  }
  if (r.kind == RequestKind::AP) {
    AdminLease lease(admin_);
    // The fence is momentarily held by md_http_life: keep the request for the
    // next tick. A QUEUED request is never dropped silently.
    if (!lease) return false;
    if (r.generation && (!admin_.allowed(r.generation) || !verifyAdminRadio())) return true;
    if (context_.load() || transaction_.busy() || intent_pending_)
      return true;
    if (r.on) startAp(now);
    return true;
  }
  AdminLease lease(admin_);
  if (!lease) return false;
  const bool bootstrap = !r.generation && !enabled_ &&
      (r.kind == RequestKind::AP_KEY || r.kind == RequestKind::ADMIN_KEY) &&
      admin_.phase.load() == NetworkPhase::CLOSED && admin_.listeners_closed.load() &&
      !WiFi.AP.started() && !WiFi.STA.started();
  if ((r.generation && (!admin_.allowed(r.generation) || !verifyAdminRadio())) ||
      (!r.generation && !bootstrap && admin_.phase.load() != NetworkPhase::ADMIN_AP) || context_.load() || nvs_fault_) {
    config_busy_.store(false);
    recordTx(r, TxResult::REJECTED, 5);
    return true;
  }
  NetworkConfig c = config_;
  if (r.kind == RequestKind::AP_KEY)
    copy(c.ap_password, 65, r.secret);
  if (r.kind == RequestKind::ADMIN_KEY) {
    update::Sha256 sha;
    sha.update(reinterpret_cast<const uint8_t *>(r.secret), 64);
    sha.finish(c.admin_digest);
  }
  if (r.kind == RequestKind::SLEEP)
    c.usb_sleep = r.preference;
  if (r.kind == RequestKind::CONFIG) {
    if (worker_status_.scan_running) {
      esp_wifi_scan_stop();
      WiFi.scanDelete();
      worker_status_.scan_running = false;
      ++worker_status_.scan_inhibited;
    }
    const ConfigPatch &p = r.patch;
    for (unsigned i = 0; i < 2; ++i)
      if (p.profiles & (1u << i)) {
        c.sta[i] = p.config.sta[i];
        if (!(p.passwords & (1u << i))) {
          if (strcmp(c.sta[i].ssid, config_.sta[i].ssid) != 0) {
            recordTx(r, TxResult::REJECTED, 6);
            config_busy_.store(false);
            return true;
          }
          copy(c.sta[i].password, 65, config_.sta[i].password);
        }
      }
    if (p.performance) {
      c.roam_enabled = p.config.roam_enabled;
      c.roam_threshold = p.config.roam_threshold;
      c.roam_hysteresis = p.config.roam_hysteresis;
      c.scan_interval_ms = p.config.scan_interval_ms;
      c.roam_dwell_ms = p.config.roam_dwell_ms;
      c.bandwidth_mhz = p.config.bandwidth_mhz;
    }
    if (p.sleep)
      c.usb_sleep = p.config.usb_sleep;
    if (p.ap) {
      copy(c.ap_name, 33, p.config.ap_name);
      if (p.config.ap_password[0])
        copy(c.ap_password, 65, p.config.ap_password);
      c.ap_timeout_ms = p.config.ap_timeout_ms;
      c.ap_always = p.config.ap_always;
    }
    // Validate in RAM. An accepted intent is the sole authorized continuation
    // in CLIENT_STA, not permission for arbitrary administrative writes there.
    if (!validNetworkConfig(c) || p.test_profile > 1 || !c.sta[p.test_profile].enabled) {
      recordTx(r, TxResult::REJECTED, 8);
      config_busy_.store(false);
      return true;
    }
    intent_ = c;
    intent_profile_ = p.test_profile;
    intent_seq_ = r.seq;
    intent_pending_ = true;
    intent_since_ = now;
    ap_reload_ = p.ap;
    worker_status_.config_phase = ConfigPhase::CONFIG_PENDING;
    recordTx(r, TxResult::INTENT_PENDING, 0);
    return true;
  }
  if (validNetworkConfig(c) && storage_.activate(c)) {
    if (r.kind == RequestKind::AP_KEY && worker_status_.ap_active)
      ap_reload_ = true;
    config_ = c;
    admin_.setDigest(config_.admin_digest);
    if (r.kind == RequestKind::ADMIN_KEY && worker_status_.ap_active) leaveAdmin(true, now);
    transaction_.begin(&storage_, config_);
    worker_status_.nvs_active = true;
    worker_status_.config_phase = ConfigPhase::COMMITTED;
    recordTx(r, TxResult::COMMITTED, 0);
  } else {
    nvs_fault_ = true;
    worker_status_.config_phase = ConfigPhase::STORAGE_ERROR;
    recordTx(r, TxResult::STORAGE_ERROR, 9);
  }
  config_busy_.store(false);
  return true;
}
void WifiManager::scan(uint32_t now, bool roaming) {
  if (admin_.phase.load() != NetworkPhase::CLIENT_STA ||
      context_.load() || transaction_.busy() || worker_status_.scan_running ||
      !enabled_ ||
      (worker_status_.scan_starts && now - radio_poll_ms_ < 30000)) {
    ++worker_status_.scan_inhibited;
    return;
  }
  if (WiFi.scanNetworks(true, true, false, 120) == WIFI_SCAN_RUNNING) {
    worker_status_.scan_running = true;
    scan_roam_ = roaming;
    ++worker_status_.scan_starts;
    radio_poll_ms_ = now;
  } else
    ++worker_status_.scan_failures;
}
// The role the operating mode asks for, and the RUN recovery state behind it.
RoleReason WifiManager::evaluateRole(uint32_t now) {
  const bool run = run_mode_.load();
  const bool profile = config_.sta[0].enabled || config_.sta[1].enabled;
  if (run != last_run_) {
    // A mode change starts from a clean slate in either direction.
    last_run_ = run;
    recovery_ = false;
    recovery_level_ = 0;
    link_was_up_ = false;
    sta_attempt_base_ = policy_.counters().connect_attempts;
  }
  // The recovery AP is left only after its wait, and never while somebody is
  // connected to it. Each failed return doubles the next wait.
  if (recovery_ &&
      now - recovery_since_ >= recoveryDwellMs(recovery_level_, config_.ap_timeout_ms) &&
      (admin_.phase.load() != NetworkPhase::ADMIN_AP || !WiFi.softAPgetStationNum())) {
    recovery_ = false;
    if (recovery_level_ < 16) ++recovery_level_;
    policy_.adoptCredentials(profile, false, now);
    sta_attempt_base_ = policy_.counters().connect_attempts;
  }
  RoleInputs in;
  in.run = run;
  in.sta_profile = profile;
  in.recovery = recovery_;
  return selectRole(in);
}
void WifiManager::workerTick(uint32_t now) {
  if (!enabled_ && admin_.phase.load() == NetworkPhase::CLOSED &&
      now - off_since_ >= 600000) { enabled_ = true; leaveAdmin(false, now); }
  transitionTick(now);
  // A bandwidth change restarts the running STA in the installed driver.
  // Drain its complete acknowledgment before association, commit or AP return;
  // role events have no generation tag, so commands must remain serialized.
  if (admin_.phase.load() == NetworkPhase::CLIENT_STA && expected_events_.load()) {
    refreshSnapshot(now, false);
    return;
  }
  role_ = evaluateRole(now);
  if (ap_reload_ && !intent_pending_ && !transaction_.busy() &&
      admin_.phase.load() == NetworkPhase::ADMIN_AP) leaveAdmin(true, now);
  if (intent_pending_ && admin_.phase.load() == NetworkPhase::ADMIN_AP &&
      now - intent_since_ >= 1000) leaveAdmin(true, now);
  if (!held_valid_ &&
      xQueueReceive(static_cast<QueueHandle_t>(queue_), &held_, 0) == pdTRUE) {
    held_valid_ = true;
    held_attempts_ = 0;
  }
  if (held_valid_) {
    bool done = consume(held_, now);
    if (!done && ++held_attempts_ >= kFenceRetryLimit) {
      // Bounded: a fence that never frees refuses the request, with a result.
      if (held_.kind != RequestKind::AP) {
        config_busy_.store(false);
        recordTx(held_, TxResult::REJECTED, 5);
      }
      done = true;
    }
    if (done) {
      memset(held_.secret, 0, sizeof(held_.secret));
      held_ = Request{};
      held_valid_ = false;
    }
  }
  if (admin_.phase.load() != NetworkPhase::CLIENT_STA) {
    worker_status_.ap_active = admin_.phase.load() == NetworkPhase::ADMIN_AP;
    worker_status_.ap_reload_pending = ap_reload_;
    // The AP is left for the router only when the role says so: RUN, an
    // enabled profile, no recovery wait in force. In MAINTENANCE this is
    // never true, so no idle timeout can close a maintenance session.
    if (worker_status_.ap_active && roleIsSta(role_) && !configBusy())
      stopAp();
    refreshSnapshot(now, false);
    return;
  }
  if (connect_due_ && WiFi.status() != WL_CONNECTED &&
      now - connect_since_ >= 100) {
    connect_due_ = false;
    test_fresh_ = true;
    issueConnect(now);
  }
  bool link_up = WiFi.status() == WL_CONNECTED &&
                 static_cast<uint32_t>(WiFi.localIP()) != 0;
  if (transaction_.busy()) {
    // Reject stale pre-test association/IP: the real driver identity must match
    // the tested profile after an asynchronous disconnect has completed.
    wifi_ap_record_t record{};
    const auto &target = transaction_.candidate().sta[profile_];
    bool match =
        test_fresh_ && link_up && esp_wifi_sta_get_ap_info(&record) == ESP_OK &&
        strncmp(reinterpret_cast<const char *>(record.ssid), target.ssid, 32) ==
            0;
    AdminLease commit(admin_);
    if (!commit) { refreshSnapshot(now, link_up); return; }
    wifi_mode_t mode = WIFI_MODE_NULL;
    const bool client_verified = !admin_.closing.load() &&
        admin_.phase.load() == NetworkPhase::CLIENT_STA &&
        esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA;
    const bool critical = context_.load() != 0 || !client_verified;
    // A link that only flickers up is not proof: the candidate is committed
    // after association and address were held without interruption.
    if (!match) test_stable_ = false;
    else if (!test_stable_) { test_stable_ = true; test_stable_since_ = now; }
    const bool proven = match && now - test_stable_since_ >= kStaTestHoldMs;
    // The driver reported the attempt over and nothing is associated: try
    // again inside the same transaction window. The profile is already in the
    // driver; this never touches configuration, mode or storage.
    if (!critical && test_fresh_ && !link_up && !sta_link_.load() &&
        sta_disconnects_.load() != test_disconnects_ &&
        now - test_connect_ms_ >= kStaTestRetryMs) {
      test_connect_ms_ = now;
      test_disconnects_ = sta_disconnects_.load();
      if (worker_status_.sta_test.attempts < 255) ++worker_status_.sta_test.attempts;
      esp_wifi_connect();
    }
    trackStaTest(now);
    transaction_.update(now, proven, link_up, critical);
    worker_status_.config_phase = transaction_.phase();
    if (!transaction_.busy()) {
      finishStaTest(now, critical);
      worker_status_.tx_result = static_cast<uint8_t>(
          transaction_.phase() == ConfigPhase::COMMITTED ? TxResult::COMMITTED
          : transaction_.phase() == ConfigPhase::STORAGE_ERROR ? TxResult::STORAGE_ERROR
                                                               : TxResult::ROLLBACK);
      if (transaction_.phase() == ConfigPhase::COMMITTED) {
        config_ = transaction_.active();
        worker_status_.nvs_active = true;
      } else {
        ap_reload_ = false;
        if (transaction_.phase() == ConfigPhase::STORAGE_ERROR)
          nvs_fault_ = true;
        profile_ = config_.sta[0].enabled ? 0 : 1;
        connectProfile(now);
      }
      policy_.adoptCredentials(config_.sta[0].enabled || config_.sta[1].enabled,
                               transaction_.phase() == ConfigPhase::COMMITTED,
                               now);
      config_busy_.store(false);
      if (return_admin_) {
        return_admin_ = false;
        startAp(now);
      }
    }
  } else if (enabled_ && roleIsSta(role_)) {
    // Only the RUN router role connects by itself. In MAINTENANCE a stored
    // profile is never joined outside the explicit test transaction above.
    const WifiAction action = policy_.update(now, link_up);
    switch (action) {
    case WifiAction::START_RADIO: {
      // Initialization already started the STA radio. If mode no longer
      // matches, recover through the same listener closure barrier.
      wifi_mode_t mode = WIFI_MODE_NULL;
      if (esp_wifi_get_mode(&mode) != ESP_OK || mode != WIFI_MODE_STA)
        leaveAdmin(false, now);
      break;
    }
    case WifiAction::START_CONNECT: {
      // Every episode starts with the primary profile; the secondary one is
      // the fallback for the attempt after a failure.
      const uint8_t primary = config_.sta[0].enabled ? 0 : 1;
      const uint32_t attempt = policy_.counters().connect_attempts - sta_attempt_base_;
      profile_ = (attempt % 2 == 0 && config_.sta[1 - primary].enabled) ? 1 - primary : primary;
      connectProfile(now);
      break;
    }
    case WifiAction::STOP_RADIO:
      link_up = false;
      break;
    case WifiAction::NONE:
      break;
    }
  }
  if (roleIsSta(role_) && !transaction_.busy()) {
    if (link_up) {
      // Connected with an address: the failure count starts over. The recovery
      // wait goes back to its minimum only after the link has proven stable.
      sta_attempt_base_ = policy_.counters().connect_attempts;
      if (!link_was_up_) { link_was_up_ = true; link_since_ = now; }
      else if (now - link_since_ >= kStaStableMs) recovery_level_ = 0;
    } else {
      link_was_up_ = false;
      // The last attempt of the episode is over (timeout or refused call) and
      // nothing is connected: stop trying, fall back to the recovery AP.
      if (policy_.state() == WifiState::BACKOFF &&
          policy_.counters().connect_attempts - sta_attempt_base_ >= kStaEpisodeAttempts) {
        recovery_ = true;
        recovery_since_ = now;
        role_ = RoleReason::RUN_RECOVERY_AP;
      }
    }
  }
  if (enabled_ && !transaction_.busy() && !configBusy() && !roleIsSta(role_))
    startAp(now);
  if (admin_.phase.load() != NetworkPhase::CLIENT_STA) {
    refreshSnapshot(now, false); return;
  }
  if (worker_status_.scan_running && context_.load()) {
    esp_wifi_scan_stop();
    WiFi.scanDelete();
    worker_status_.scan_running = false;
    ++worker_status_.scan_inhibited;
  }
  if (worker_status_.scan_running) {
    int n = WiFi.scanComplete();
    if (n != WIFI_SCAN_RUNNING) {
      worker_status_.scan_running = false;
      worker_status_.scan_count = 0;
      int best = worker_status_.rssi_dbm;
      bool different = false;
      for (int i = 0; i < n && i < 12; ++i) {
        auto &entry = worker_status_.scan[worker_status_.scan_count++];
        copy(entry.ssid, 33, WiFi.SSID(i).c_str());
        macText(WiFi.BSSID(i), entry.bssid);
        entry.rssi = WiFi.RSSI(i);
        entry.channel = WiFi.channel(i);
        entry.secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
        if (strcmp(entry.ssid, config_.sta[profile_].ssid) == 0 &&
            strcmp(entry.bssid, worker_status_.bssid) != 0 &&
            entry.rssi > best) {
          best = entry.rssi;
          different = true;
        }
      }
      if (n < 0)
        ++worker_status_.scan_failures;
      WiFi.scanDelete();
      if (scan_roam_ && roam_.roam(now, config_, worker_status_.rssi_dbm, best,
                                   different, context_.load() != 0)) {
        ++worker_status_.roam_count;
        WiFi.disconnectAsync(false, false);
        policy_.reportActionFailed(WifiFault::LINK_LOST, now);
        link_up = false;
      }
    }
  }
  if (roam_.scan(now, config_, link_up, worker_status_.rssi_dbm,
                 context_.load() != 0))
    scan(now, true);
  SleepInputs sleep{};
  sleep.preference = config_.usb_sleep;
  sleep.ota = (context_.load() & 2u) != 0;
  sleep.provisioning =
      worker_status_.ap_active ||
      transaction_.busy(); // no trusted HWCDC session proof => OFF
  const bool desired = modemSleep(sleep);
  const wifi_ps_type_t ps = desired ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE;
  wifi_ps_type_t effective = WIFI_PS_NONE;
  worker_status_.sleep_apply_ok =
      esp_wifi_get_ps(&effective) == ESP_OK && effective == ps;
  if (!worker_status_.sleep_apply_ok)
    worker_status_.sleep_apply_ok = esp_wifi_set_ps(ps) == ESP_OK;
  worker_status_.sleep_effective =
      esp_wifi_get_ps(&effective) == ESP_OK && effective != WIFI_PS_NONE;
  if (enabled_ && !context_.load()) {
    const auto desired_bw =
        config_.bandwidth_mhz == 40 ? WIFI_BW_HT40 : WIFI_BW_HT20;
    wifi_bandwidth_t actual_bw = WIFI_BW_HT20;
    const bool bandwidth_read = esp_wifi_get_bandwidth(WIFI_IF_STA, &actual_bw) == ESP_OK;
    worker_status_.bandwidth_apply_ok = bandwidth_read && actual_bw == desired_bw;
    // wifi_set_bw_process in esp32s3-libs 3.3.11 calls _do_wifi_stop(1)
    // and _do_wifi_start(1) when STA bandwidth differs. This is an owned
    // radio command, not a harmless setter. Never arm on an unknown read,
    // never overlap another command, and never admit an AP generation here.
    if (bandwidth_read && !worker_status_.bandwidth_apply_ok &&
        !expected_events_.load() && !admin_.radio_fault.load() &&
        admin_.phase.load() == NetworkPhase::CLIENT_STA &&
        !admin_.closing.load() && WiFi.STA.started() && !WiFi.AP.started()) {
      event_since_ = now;
      expected_events_.store(kStaStop | kStaStart);
      worker_status_.bandwidth_apply_ok =
          esp_wifi_set_bandwidth(WIFI_IF_STA, desired_bw) == ESP_OK;
      if (!worker_status_.bandwidth_apply_ok) fault(FaultSite::MODE_COMMAND, 3);
    }
  }
  refreshSnapshot(now, link_up);
}
void WifiManager::refreshSnapshot(uint32_t now, bool link_up) {
  auto &w = worker_status_;
  w.state = policy_.state();
  w.fault = policy_.fault();
  w.enabled = enabled_;
  w.credentials_present = config_.sta[0].enabled || config_.sta[1].enabled;
  w.connected = link_up;
  w.active_profile = profile_;
  w.counters = policy_.counters();
  w.backoff_ms = policy_.backoffMs();
  w.state_since_ms = policy_.stateSinceMs();
  w.disconnect_reason = disconnect_reason_.load();
  w.run_mode = last_run_;
  w.role_reason = static_cast<uint8_t>(role_);
  w.recovery_level = recovery_level_;
  {
    const uint32_t dwell = recoveryDwellMs(recovery_level_, config_.ap_timeout_ms);
    const uint32_t waited = now - recovery_since_;
    w.recovery_retry_ms = recovery_ && waited < dwell ? dwell - waited : 0;
  }
  // The driver's own view, independent of the administrative phase.
  w.admin_phase = static_cast<uint8_t>(admin_.phase.load());
  wifi_mode_t driver_mode = WIFI_MODE_NULL;
  const auto driver_read = esp_wifi_get_mode(&driver_mode);
  w.driver_mode = driver_read == ESP_OK ? static_cast<uint8_t>(driver_mode)
                  : driver_read == ESP_ERR_WIFI_NOT_INIT ? 0 : 255;
  w.driver_ap_started = WiFi.AP.started();
  w.driver_sta_started = WiFi.STA.started();
  if (!w.ap_active) w.ap_config_verified = false;
  w.ap_provisioned = config_.ap_password[0];
  w.admin_provisioned = hasAdmin(config_);
  w.ap_clients = w.ap_active ? WiFi.softAPgetStationNum() : 0;
  w.ap_ipv4 = w.ap_active ? static_cast<uint32_t>(WiFi.softAPIP()) : 0;
  w.sleep_configured = config_.usb_sleep;
  w.bandwidth_configured = config_.bandwidth_mhz;
  w.config_busy = configBusy();
  w.roam_enabled = config_.roam_enabled;
  w.roam_threshold = config_.roam_threshold;
  w.roam_hysteresis = config_.roam_hysteresis;
  w.scan_interval_ms = config_.scan_interval_ms;
  w.roam_dwell_ms = config_.roam_dwell_ms;
  w.ap_timeout_ms = config_.ap_timeout_ms;
  w.ap_always = config_.ap_always;
  wifi_bandwidth_t bw = WIFI_BW_HT20;
  w.bandwidth_mhz =
      esp_wifi_get_bandwidth(WIFI_IF_STA, &bw) == ESP_OK && bw == WIFI_BW_HT40
          ? 40
          : 20;
  for (unsigned i = 0; i < 2; ++i) {
    auto &dst = w.profiles[i];
    const auto &s = config_.sta[i];
    copy(dst.ssid, 33, s.ssid);
    dst.enabled = s.enabled;
    dst.dhcp = s.dhcp;
    dst.ip = s.ip;
    dst.mask = s.mask;
    dst.gateway = s.gateway;
    dst.dns = s.dns;
  }
  copy(w.ssid, 33,
       transaction_.busy() ? transaction_.candidate().sta[profile_].ssid
                           : config_.sta[profile_].ssid);
  w.ipv4 = link_up ? static_cast<uint32_t>(WiFi.localIP()) : 0;
  w.rssi_dbm = link_up ? WiFi.RSSI() : 0;
  w.channel = (link_up || w.ap_active) ? WiFi.channel() : 0;
  w.bssid[0] = 0;
  if (link_up) {
    wifi_ap_record_t record{};
    if (esp_wifi_sta_get_ap_info(&record) == ESP_OK)
      macText(record.bssid, w.bssid);
  }
}
} // namespace network
} // namespace matdog
