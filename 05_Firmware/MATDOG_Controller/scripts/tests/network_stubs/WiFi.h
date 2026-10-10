#pragma once
#include "esp_wifi.h"
#include <cstdint>
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
  bool mode(int v) {
    if (!mode_write_ok) return false;
    const int old = mode_value;
    const bool sta_started = STA.running;
    mode_value = v; STA.running = (v & WIFI_STA) != 0;
    AP.running = (v & WIFI_AP) != 0 && ap;
    if (!(v & WIFI_STA)) { connected=false; ip=0; }
    if (!(v & WIFI_AP)) { ap=false; AP.running=false; }
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
  bool softAP(const char *s, const char *p, int, bool, int) {
    ap = true; AP.running=true;
    ap_name = s;
    ap_key = p;
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
