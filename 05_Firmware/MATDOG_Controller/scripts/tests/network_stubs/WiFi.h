#pragma once
#include "esp_wifi.h"
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
constexpr int WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3, WIFI_OFF = 0, WL_CONNECTED = 3,
              WIFI_SCAN_RUNNING = -1, INADDR_NONE = -1;
constexpr int ARDUINO_EVENT_WIFI_STA_DISCONNECTED = 1, ARDUINO_EVENT_WIFI_AP_STOP = 2, ARDUINO_EVENT_WIFI_STA_START = 3, ARDUINO_EVENT_WIFI_STA_STOP = 4, ARDUINO_EVENT_WIFI_AP_START = 5;
using WiFiEvent_t = int;
struct WiFiEventInfo_t {
  struct {
    uint16_t reason = 0;
  } wifi_sta_disconnected;
};
class IPAddress {
public:
  IPAddress(uint32_t v = 0) : v_(v) {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
      : v_(a | (uint32_t(b) << 8) | (uint32_t(c) << 16) | (uint32_t(d) << 24)) {
  }
  operator uint32_t() const { return v_; }

private:
  uint32_t v_;
};
struct WifiStub {
  struct Interface { bool running=false; bool started() const {return running;} };
  Interface AP, STA;
  int mode_value = WIFI_STA;
  bool mode_read_ok=true, mode_write_ok=true;
  bool deliver_events = true;
  std::vector<int> pending_events;
  void emit(int event) { if (deliver_events && callback) callback(event,{}); else pending_events.push_back(event); }
  void drainEvents() { auto events=pending_events; pending_events.clear(); for(int e:events) if(callback) callback(e,{}); }
  bool connected = false, delayed_disconnect = false, ap = false,
       persistent_setting = true, reconnect = true;
  int channel_value = 1, clients = 0, scan_state = -2, scan_calls = 0,
      connect_calls = 0, disconnect_calls = 0;
  uint32_t ip = 0;
  std::string associated_ssid, ap_name, ap_key;
  wifi_config_t driver{};
  std::function<void(WiFiEvent_t, WiFiEventInfo_t)> callback;
  void persistent(bool v) { persistent_setting = v; }
  void setAutoReconnect(bool v) { reconnect = v; }
  void onEvent(std::function<void(WiFiEvent_t, WiFiEventInfo_t)> f) {
    callback = f;
  }
  // ---- Soft-AP driver model -------------------------------------------
  // Mirrors what esp32:esp32 3.3.11 / its Wi-Fi library really do, because the
  // difference is exactly what a host double must not hide:
  //   * the AP role STARTS as soon as the mode includes AP, with whatever
  //     profile the driver holds (AP_START), before any profile is written;
  //   * writing a profile to a started AP (esp_wifi_set_config(WIFI_IF_AP))
  //     stops and restarts it inside the driver: AP_STOP, then AP_START;
  //   * the profile lives in driver RAM across STA<->AP changes and is lost
  //     when the driver is deinitialized (WIFI_OFF).
  wifi_config_t ap_profile{};
  bool ap_set_ok = true, ap_get_ok = true, ap_restart_events = true,
       ap_store_corrupt = false;
  int ap_set_calls = 0;
  esp_err_t setApConfig(const wifi_config_t *c) {
    ++ap_set_calls;
    if (!ap_set_ok) return -1;
    ap_profile = *c;
    if (ap_store_corrupt) ap_profile.ap.password[0] ^= 1; // driver holds another key
    ap_name.assign(reinterpret_cast<const char *>(ap_profile.ap.ssid),
                   strnlen(reinterpret_cast<const char *>(ap_profile.ap.ssid), 32));
    ap_key.assign(reinterpret_cast<const char *>(ap_profile.ap.password),
                  strnlen(reinterpret_cast<const char *>(ap_profile.ap.password), 64));
    if (AP.running) {
      ap = true;
      if (ap_restart_events) {
        emit(ARDUINO_EVENT_WIFI_AP_STOP);
        emit(ARDUINO_EVENT_WIFI_AP_START);
      }
    }
    return 0;
  }
  esp_err_t getApConfig(wifi_config_t *c) {
    if (!ap_get_ok) return -1;
    *c = ap_profile;
    return 0;
  }
  bool mode(int v) {
    if (!mode_write_ok) return false;
    const int old = mode_value;
    const bool sta_started = STA.running;
    mode_value = v; STA.running = (v & WIFI_STA) != 0;
    AP.running = (v & WIFI_AP) != 0;
    ap = AP.running && !ap_name.empty();
    if (!(v & WIFI_STA)) { connected=false; ip=0; }
    if (v == WIFI_OFF) { ap_profile = wifi_config_t{}; ap_name.clear(); ap_key.clear(); }
    if ((old & WIFI_AP) && !(v & WIFI_AP)) emit(ARDUINO_EVENT_WIFI_AP_STOP);
    if ((old & WIFI_STA) && !(v & WIFI_STA)) emit(ARDUINO_EVENT_WIFI_STA_STOP);
    if ((v & WIFI_STA) && (!(old & WIFI_STA) || !sta_started)) emit(ARDUINO_EVENT_WIFI_STA_START);
    if ((v & WIFI_AP) && !(old & WIFI_AP)) emit(ARDUINO_EVENT_WIFI_AP_START);
    return true;
  }
  int status() { return connected ? WL_CONNECTED : 0; }
  IPAddress localIP() { return IPAddress(ip); }
  int RSSI(int = -1) { return -85; }
  int channel(int = -1) { return channel_value; }
  bool disconnectAsync(bool, bool) {
    ++disconnect_calls;
    if (!delayed_disconnect)
      connected = false;
    return true;
  }
  bool config(IPAddress, IPAddress, IPAddress, IPAddress = IPAddress()) {
    return true;
  }
  bool softAPConfig(IPAddress, IPAddress, IPAddress) { return true; }
  // The stock call, as the installed core implements it: enable the AP role,
  // then write the profile only when it differs from the driver's.
  bool softAP(const char *s, const char *p, int channel, bool hidden, int max_connection) {
    if (!(mode_value & WIFI_AP) && !mode(mode_value | WIFI_AP)) return false;
    wifi_config_t c{};
    std::memcpy(c.ap.ssid, s, strnlen(s, 32));
    c.ap.ssid_len = static_cast<uint8_t>(strnlen(s, 32));
    std::memcpy(c.ap.password, p, strnlen(p, 63));
    c.ap.channel = static_cast<uint8_t>(channel);
    c.ap.max_connection = static_cast<uint8_t>(max_connection);
    c.ap.beacon_interval = 100;
    c.ap.ssid_hidden = hidden;
    c.ap.authmode = WIFI_AUTH_WPA2_PSK;
    c.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
    if (std::memcmp(&c.ap, &ap_profile.ap, sizeof(c.ap)) != 0) return setApConfig(&c) == 0;
    ap = true;
    return true;
  }
  bool softAPdisconnect(bool) {
    ap = false;
    return true;
  }
  uint8_t softAPgetStationNum() { return clients; }
  IPAddress softAPIP() { return IPAddress(192, 168, 4, 1); }
  int scanNetworks(bool async, bool, bool, int) {
    if (async) {
      ++scan_calls;
      scan_state = WIFI_SCAN_RUNNING;
    }
    return scan_state;
  }
  int scanComplete() { return scan_state; }
  void scanDelete() { scan_state = -2; }
  std::string SSID(int) { return "network"; }
  uint8_t *BSSID(int) {
    static uint8_t mac[6] = {1, 2, 3, 4, 5, 6};
    return mac;
  }
  int encryptionType(int) { return 1; }
};
extern WifiStub WiFi;
