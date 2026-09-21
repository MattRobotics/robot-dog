#include "WifiManager.h"

#include <Arduino.h>
#include <WiFi.h>

#include "../config/WifiCredentials.h"

// THE ONLY translation unit in MATDOG Controller that touches the radio.
//
// It deliberately contains no reference to any servo primitive. That is not
// a style preference: scripts/static_audit.py::check_no_network_to_servo_path
// FAILS the build if one file names both a network transport symbol and a
// servo primitive, which is how ARCHITECTURE.md's permanent rule
// "network callback != servo command authority" is actually enforced.
//
// Blocking-call audit of esp32:esp32 3.3.11, since "non-blocking" has to
// mean something checkable:
//
//   WiFi.mode(WIFI_STA)   -> WiFiGenericClass::mode(): wifiLowLevelInit +
//                            esp_wifi_set_mode + espWiFiStart. No
//                            waitStatusBits, no delay(). The first call
//                            initializes the driver and is the single most
//                            expensive call here; its real cost is measured
//                            into WifiStatus::max_update_us rather than
//                            assumed.
//   WiFi.begin(ssid,pass) -> STAClass::begin() contains
//                            waitStatusBits(ESP_NETIF_STARTED_BIT, 1000) —
//                            a wait of up to ONE SECOND if the netif is not
//                            up yet. The RADIO_STARTING state exists purely
//                            so this call is never reached before the bit
//                            is already set. STAClass::connect() then ends
//                            at esp_wifi_connect(), which is asynchronous.
//   WiFi.disconnectAsync  -> disconnect(..., timeout 0): esp_wifi_disconnect
//                            with no wait loop. The plain disconnect()
//                            overload defaults to a 100 ms polling wait and
//                            is therefore NOT used.
//   WiFi.status/RSSI/
//   localIP/channel       -> status-bit and cached-info reads.
//
// Also note what is NOT called: WiFi.waitForConnectResult() (blocks),
// WiFi.scanNetworks() in its blocking form, and WiFi.SSID() in the refresh
// path (returns an Arduino String and would allocate every poll).

namespace matdog {
namespace network {

void WifiManager::begin(uint32_t now_ms) {
  const WifiPolicyConfig config{kRadioSettleMs, kConnectTimeoutMs, kBackoffInitialMs,
                                kBackoffMaxMs};
  policy_.begin(config, config::kWifiCredentialsPresent, now_ms);

  // Pure setters in 3.3.11 (they assign a member and return); neither
  // starts the driver, so both are safe before any radio call.
  //
  // persistent(false): stop the core from writing the credentials into NVS
  // on every connect. That avoids flash wear in a reconnect loop and keeps
  // the passphrase out of a second, longer-lived storage location.
  //
  // setAutoReconnect(false): WifiPolicy is the single owner of retry
  // timing. With the core silently retrying underneath, the state machine
  // and its counters would be describing a process they do not control.
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);

  // Configured network identity, copied once. Bounded manual copy rather
  // than strncpy so there is no truncation-warning ambiguity under
  // --warnings all.
  const char* ssid = config::kWifiSsid;
  size_t i = 0;
  while (i + 1 < sizeof(status_.ssid) && ssid[i] != '\0') {
    status_.ssid[i] = ssid[i];
    ++i;
  }
  status_.ssid[i] = '\0';

  refreshSnapshot(now_ms, false);
}

void WifiManager::update(uint32_t now_ms) {
  const uint32_t started_us = micros();

  // One cheap status-bit read. This is the only place link state enters the
  // state machine, so CONNECTED can never mean anything but "the radio said
  // so on this tick".
  const bool link_up = (WiFi.status() == WL_CONNECTED);

  const WifiAction action = policy_.update(now_ms, link_up);

  switch (action) {
    case WifiAction::START_RADIO:
      if (!WiFi.mode(WIFI_STA)) {
        policy_.reportActionFailed(WifiFault::RADIO_START_FAILED, now_ms);
      }
      break;

    case WifiAction::START_CONNECT: {
      // kWifiPassword is used HERE and nowhere else in the firmware. It is
      // not copied into WifiStatus, not returned by an accessor and not
      // printed. static_audit.py checks that this remains the only use.
      //
      // The policy never emits START_CONNECT while CONNECTED, which matters:
      // STAClass::connect() calls disconnect(true, 1000) first if it finds
      // an existing link, and that would block for up to a second.
      const wl_status_t result = WiFi.begin(config::kWifiSsid, config::kWifiPassword);
      if (result == WL_CONNECT_FAILED) {
        policy_.reportActionFailed(WifiFault::CONNECT_CALL_REJECTED, now_ms);
      }
      break;
    }

    case WifiAction::STOP_RADIO:
      // Async form on purpose — see the blocking-call audit above.
      WiFi.disconnectAsync(true /* wifioff */, false /* eraseap */);
      radio_polled_ = false;
      break;

    case WifiAction::NONE:
      break;
  }

  refreshSnapshot(now_ms, link_up);

  // Measured, not claimed. micros() wraps every ~71 minutes; the unsigned
  // difference stays correct across the wrap.
  const uint32_t elapsed_us = micros() - started_us;
  status_.last_update_us = elapsed_us;
  if (elapsed_us > status_.max_update_us) {
    status_.max_update_us = elapsed_us;
  }
}

bool WifiManager::setEnabled(bool enabled, uint32_t now_ms) {
  if (enabled && !policy_.credentialsPresent()) {
    policy_.setEnabled(true, now_ms);  // records NO_CREDENTIALS, changes nothing else
    refreshSnapshot(now_ms, false);
    return false;
  }
  policy_.setEnabled(enabled, now_ms);
  return true;
}

void WifiManager::refreshSnapshot(uint32_t now_ms, bool link_up) {
  status_.state = policy_.state();
  status_.fault = policy_.fault();
  status_.enabled = policy_.enabled();
  status_.credentials_present = policy_.credentialsPresent();
  status_.connected = link_up;
  status_.backoff_ms = policy_.backoffMs();
  status_.state_since_ms = policy_.stateSinceMs();
  status_.counters = policy_.counters();

  if (!link_up) {
    // Clear the link-scoped facts so a stale RSSI/IP can never be read as
    // current. The SSID is the CONFIGURED identity and stays.
    status_.rssi_dbm = 0;
    status_.ipv4 = 0;
    status_.channel = 0;
    radio_polled_ = false;
    return;
  }

  // Rate-limited: telemetry consumers read the snapshot, they never cause
  // a radio query.
  if (radio_polled_ && (now_ms - radio_poll_ms_) < kRadioPollIntervalMs) return;
  radio_polled_ = true;
  radio_poll_ms_ = now_ms;

  status_.rssi_dbm = WiFi.RSSI();
  status_.ipv4 = static_cast<uint32_t>(WiFi.localIP());
  const int32_t ch = WiFi.channel();
  status_.channel = (ch > 0 && ch < 256) ? static_cast<uint8_t>(ch) : 0;
}

}  // namespace network
}  // namespace matdog
