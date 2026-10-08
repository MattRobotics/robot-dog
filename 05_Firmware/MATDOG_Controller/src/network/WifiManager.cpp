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
void copy(char *out, size_t cap, const char *in) {
  size_t n = strnlen(in, cap - 1);
  memset(out, 0, cap);
  memcpy(out, in, n);
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
  WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
      disconnect_reason_.store(info.wifi_sta_disconnected.reason);
  });
  WiFi.mode(
      WIFI_STA); // default remains radio ON, including missing credentials
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
bool WifiManager::post(const Request &r, bool mutation) {
  if (!queue_ || (mutation && (context_.load() != 0 || nvs_fault_)))
    return false;
  if (mutation && config_busy_.exchange(true))
    return false;
  if (xQueueSend(static_cast<QueueHandle_t>(queue_), &r, 0) != pdTRUE) {
    if (mutation)
      config_busy_.store(false);
    return false;
  }
  return true;
}
bool WifiManager::setEnabled(bool enabled, uint32_t now_ms) {
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
  if (context_.load() || configBusy())
    return false;
  Request r{};
  r.kind = RequestKind::SCAN;
  return post(r, false);
}
bool WifiManager::requestAp(bool on) {
  if (!on && status_.ap_always)
    return false;
  Request r{};
  r.kind = RequestKind::AP;
  r.on = on;
  return post(r, false);
}
bool WifiManager::setSleep(uint8_t p) {
  if (p > 2)
    return false;
  Request r{};
  r.kind = RequestKind::SLEEP;
  r.preference = p;
  return post(r, true);
}
bool WifiManager::provisionAp(const char *p) {
  if (!p || strlen(p) < 8 || strlen(p) > 63)
    return false;
  Request r{};
  r.kind = RequestKind::AP_KEY;
  copy(r.secret, 65, p);
  return post(r, true);
}
bool WifiManager::provisionAdmin(const char *p) {
  uint8_t parsed[32];
  if (!p || strlen(p) != 64 || !update::parseHex(p, 64, parsed))
    return false;
  Request r{};
  r.kind = RequestKind::ADMIN_KEY;
  copy(r.secret, 65, p);
  return post(r, true);
}
bool WifiManager::configure(const ConfigPatch &p) {
  Request r{};
  r.kind = RequestKind::CONFIG;
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
void WifiManager::startAp(uint32_t now) {
  if (worker_status_.ap_active || !config_.ap_password[0])
    return;
  if (config_.ap_name[0])
    copy(worker_status_.ap_ssid, 33, config_.ap_name);
  else {
    uint8_t mac[6]{};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(worker_status_.ap_ssid, 33, "MATDOG-%02X%02X%02X", mac[3], mac[4],
             mac[5]);
  }
  WiFi.mode(WIFI_AP_STA);
  if (WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                        IPAddress(255, 255, 255, 0)) &&
      WiFi.softAP(worker_status_.ap_ssid, config_.ap_password, 1, false, 4)) {
    worker_status_.ap_active = true;
    ap_since_ = now;
  } else
    worker_status_.config_error = 3;
}
void WifiManager::stopAp() {
  if (worker_status_.ap_active) {
    WiFi.softAPdisconnect(false);
    WiFi.mode(WIFI_STA);
    worker_status_.ap_active = false;
  }
}
void WifiManager::connectProfile(uint32_t now) {
  connect_due_ = true;
  test_fresh_ = false;
  connect_since_ = now;
  WiFi.disconnectAsync(false, false);
}
void WifiManager::issueConnect(uint32_t now) {
  (void)now;
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
  if (esp_wifi_set_config(WIFI_IF_STA, &drv) != ESP_OK ||
      esp_wifi_connect() != ESP_OK)
    policy_.reportActionFailed(WifiFault::CONNECT_CALL_REJECTED, millis());
  memset(&drv, 0, sizeof(drv));
}
void WifiManager::consume(const Request &r, uint32_t now) {
  if (r.kind == RequestKind::ENABLE) {
    if (context_.load() || transaction_.busy())
      return;
    enabled_ = r.on;
    off_since_ = now;
    policy_.setEnabled(enabled_, now);
    if (!enabled_) {
      stopAp();
      WiFi.disconnectAsync(false, false);
      WiFi.mode(WIFI_OFF);
    } else
      WiFi.mode(WIFI_STA);
    return;
  }
  if (r.kind == RequestKind::SCAN) {
    scan(now, false);
    return;
  }
  if (r.kind == RequestKind::AP) {
    if (context_.load() || transaction_.busy())
      return;
    if (r.on) {
      ap_manual_ = true;
      startAp(now);
    } else if (worker_status_.connected && !worker_status_.ap_clients &&
               !config_.ap_always) {
      ap_manual_ = false;
      stopAp();
    }
    return;
  }
  if (context_.load() || nvs_fault_) {
    config_busy_.store(false);
    worker_status_.config_error = 5;
    return;
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
            worker_status_.config_error = 6;
            config_busy_.store(false);
            return;
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
    // AP credential changes require an existing STA path, so the user's only
    // management connection is never invalidated by the form being saved.
    if (p.ap && !worker_status_.connected) {
      worker_status_.config_error = 7;
      config_busy_.store(false);
      return;
    }
    if (!transaction_.stage(c, p.test_profile, now, worker_status_.ap_active,
                            false)) {
      worker_status_.config_error = 8;
      if (transaction_.phase() == ConfigPhase::STORAGE_ERROR)
        nvs_fault_.store(true);
      config_busy_.store(false);
      return;
    }
    ap_reload_ = p.ap;
    worker_status_.config_error = 0;
    profile_ = p.test_profile;
    connectProfile(now);
    return;
  }
  if (validNetworkConfig(c) && storage_.activate(c)) {
    if (r.kind == RequestKind::AP_KEY && worker_status_.ap_active)
      ap_reload_ = true;
    config_ = c;
    transaction_.begin(&storage_, config_);
    worker_status_.nvs_active = true;
    worker_status_.config_phase = ConfigPhase::COMMITTED;
    worker_status_.config_error = 0;
  } else {
    nvs_fault_ = true;
    worker_status_.config_error = 9;
    worker_status_.config_phase = ConfigPhase::STORAGE_ERROR;
  }
  config_busy_.store(false);
}
void WifiManager::scan(uint32_t now, bool roaming) {
  if (context_.load() || transaction_.busy() || worker_status_.scan_running ||
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
void WifiManager::workerTick(uint32_t now) {
  Request req{};
  if (xQueueReceive(static_cast<QueueHandle_t>(queue_), &req, 0) == pdTRUE) {
    consume(req, now);
    memset(req.secret, 0, sizeof(req.secret));
  }
  if (!enabled_ && now - off_since_ >= 600000) {
    enabled_ = true;
    WiFi.mode(WIFI_STA);
    policy_.setEnabled(true, now);
  } // temporary USB radio OFF, recovery after 10 min
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
    transaction_.update(now, match, link_up, context_.load() != 0);
    worker_status_.config_phase = transaction_.phase();
    if (!transaction_.busy()) {
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
    }
  } else if (enabled_) {
    const WifiAction action = policy_.update(now, link_up);
    switch (action) {
    case WifiAction::START_RADIO:
      WiFi.mode(worker_status_.ap_active ? WIFI_AP_STA : WIFI_STA);
      break;
    case WifiAction::START_CONNECT:
      if (policy_.counters().connect_timeouts &&
          config_.sta[1 - profile_].enabled)
        profile_ = 1 - profile_;
      connectProfile(now);
      break;
    case WifiAction::STOP_RADIO:
      link_up = false;
      break;
    case WifiAction::NONE:
      break;
    }
  }
  if (enabled_ && (ap_manual_ || config_.ap_always ||
                   (!link_up && (!worker_status_.credentials_present ||
                                 policy_.counters().connect_timeouts >= 2))))
    startAp(now);
  // Apply AP identity only after the last client leaves; never cut a form
  // response.
  worker_status_.ap_clients =
      worker_status_.ap_active ? WiFi.softAPgetStationNum() : 0;
  if (ap_reload_ && worker_status_.ap_active && !worker_status_.ap_clients &&
      !transaction_.busy()) {
    stopAp();
    startAp(now);
    ap_reload_ = false;
  }
  worker_status_.ap_reload_pending = ap_reload_;
  if (expireAp(now, ap_since_, config_, link_up,
               transaction_.busy() || configBusy(),
               worker_status_.ap_clients)) {
    ap_manual_ = false;
    stopAp();
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
    worker_status_.bandwidth_apply_ok =
        esp_wifi_get_bandwidth(WIFI_IF_STA, &actual_bw) == ESP_OK &&
        actual_bw == desired_bw;
    if (!worker_status_.bandwidth_apply_ok)
      worker_status_.bandwidth_apply_ok =
          esp_wifi_set_bandwidth(WIFI_IF_STA, desired_bw) == ESP_OK;
    if (worker_status_.ap_active &&
        (esp_wifi_get_bandwidth(WIFI_IF_AP, &actual_bw) != ESP_OK ||
         actual_bw != WIFI_BW_HT20))
      esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  }
  refreshSnapshot(now, link_up);
}
void WifiManager::refreshSnapshot(uint32_t now, bool link_up) {
  (void)now;
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
