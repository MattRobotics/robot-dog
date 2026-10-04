#pragma once
#include <cstdint>
#include <nvs.h>
enum wifi_interface_t { WIFI_IF_STA, WIFI_IF_AP };
enum wifi_ps_type_t { WIFI_PS_NONE, WIFI_PS_MIN_MODEM };
enum wifi_bandwidth_t { WIFI_BW_HT20, WIFI_BW_HT40 };
constexpr int WIFI_ALL_CHANNEL_SCAN = 1, WIFI_CONNECT_AP_BY_SIGNAL = 1,
              WIFI_AUTH_OPEN = 0;
struct wifi_ap_record_t {
  uint8_t ssid[33]{}, bssid[6]{};
};
struct wifi_config_t {
  struct {
    uint8_t ssid[32], password[64];
    int scan_method, sort_method;
    bool bssid_set;
  } sta;
};
esp_err_t esp_wifi_get_mac(wifi_interface_t, uint8_t *);
esp_err_t esp_wifi_set_config(wifi_interface_t, const wifi_config_t *);
esp_err_t esp_wifi_connect();
esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *);
esp_err_t esp_wifi_scan_stop();
esp_err_t esp_wifi_set_ps(wifi_ps_type_t);
esp_err_t esp_wifi_get_ps(wifi_ps_type_t *);
esp_err_t esp_wifi_set_bandwidth(wifi_interface_t, wifi_bandwidth_t);
esp_err_t esp_wifi_get_bandwidth(wifi_interface_t, wifi_bandwidth_t *);
