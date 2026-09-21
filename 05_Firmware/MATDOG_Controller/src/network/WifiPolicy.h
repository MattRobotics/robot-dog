#ifndef MATDOG_NETWORK_WIFI_POLICY_H
#define MATDOG_NETWORK_WIFI_POLICY_H

#include <stdint.h>

// Deliberately <stdint.h>, not <Arduino.h>, and deliberately no <WiFi.h>
// (W1): this unit carries ALL the Wi-Fi decision logic — when to start the
// radio, when to retry, how long to back off, what the observable state is —
// with no radio dependency at all. Two consequences the project already
// requires elsewhere:
//
//   1. scripts/tests/test_wifi_policy.cpp links the REAL implementation,
//      not a second copy of the rules that could drift. Same contract as
//      power/DalyProtocol.* and servo/ServoPopulation.* (G2,
//      static_audit.py::check_g2_state_is_transport_independent).
//   2. WifiStatus is a plain copyable snapshot, so USB CDC today and the
//      embedded Web UI later format the SAME structured state instead of
//      re-querying the radio per request — the telemetry-snapshot model in
//      01_Docs/02_Architecture/ARCHITECTURE.md.
//
// The radio itself is touched in exactly one translation unit,
// network/WifiManager.cpp.

namespace matdog {
namespace network {

// NOTE: the obvious name for the first state is DISABLED. It cannot be used.
// <esp32-hal-gpio.h> #defines DISABLED, so an enumerator by that name is
// textually replaced and the build breaks in an unrelated-looking way. The
// project has already been bitten by exactly this (see the -DDISABLED=0x00
// flag in scripts/tests/run_host_tests.sh, added so the DALY suite
// reproduces the core macro on the host). INACTIVE says the same thing and
// survives the preprocessor.
enum class WifiState : uint8_t {
  INACTIVE       = 0,  // radio off: no credentials, or switched off by the operator
  IDLE           = 1,  // allowed to connect, radio not started yet
  RADIO_STARTING = 2,  // driver/netif coming up; see kRadioSettleMs for why this state exists
  CONNECTING     = 3,  // association attempt in flight, connect deadline armed
  CONNECTED      = 4,  // link up
  BACKOFF        = 5,  // attempt failed or link lost; waiting before the next attempt
};

// Why the last transition happened. Kept separate from WifiState so a
// BACKOFF caused by a lost link is distinguishable from one caused by an
// association timeout without inventing extra states.
enum class WifiFault : uint8_t {
  NONE                  = 0,
  NO_CREDENTIALS        = 1,  // fail closed: nothing configured, radio never started
  CONNECT_TIMEOUT       = 2,  // association did not complete within connect_timeout_ms
  LINK_LOST             = 3,  // was CONNECTED, link went down
  RADIO_START_FAILED    = 4,  // the driver refused to start
  CONNECT_CALL_REJECTED = 5,  // the connect call itself was rejected before any association
};

// What the policy wants the radio owner to do on this tick. The policy
// never performs I/O; WifiManager executes exactly one of these per tick.
enum class WifiAction : uint8_t {
  NONE          = 0,
  START_RADIO   = 1,  // bring the driver/netif up (no association yet)
  START_CONNECT = 2,  // issue one association attempt
  STOP_RADIO    = 3,  // tear the radio down
};

struct WifiPolicyConfig {
  // Gap between "driver started" and "issue the association".
  //
  // This is not cosmetic padding. In esp32:esp32 3.3.11, WiFi.begin() calls
  // STAClass::begin(), which calls waitStatusBits(ESP_NETIF_STARTED_BIT,
  // 1000) — a blocking wait of up to ONE SECOND if the netif has not come
  // up yet. A one-second stall in Controller::update() is exactly what
  // ARCHITECTURE.md forbids ("network failure must not starve ServoBus,
  // BNO085, DALY or motion"). Splitting the start into WiFi.mode(WIFI_STA)
  // (which contains no such wait) and, some ticks later, WiFi.begin()
  // means the started bit is already set by the time the waiting call runs,
  // so it returns immediately. WifiManager measures the real cost of every
  // tick (WifiStatus::max_update_us) so this reasoning stays falsifiable.
  uint32_t radio_settle_ms;

  uint32_t connect_timeout_ms;   // association+DHCP deadline before giving up
  uint32_t backoff_initial_ms;   // first retry wait
  uint32_t backoff_max_ms;       // ceiling for the doubling retry wait
};

struct WifiCounters {
  uint32_t radio_starts;
  uint32_t connect_attempts;
  uint32_t connects;          // successful associations this boot
  uint32_t connect_timeouts;
  uint32_t link_losses;
};

// The whole observable Wi-Fi surface, as one copyable struct.
//
// There is deliberately NO password field, and no field that could ever
// hold one. The passphrase is read once, inside WifiManager.cpp, straight
// into the connect call; it is never stored here, never returned by an
// accessor and therefore cannot reach a log, @STATUS, or a future web
// response by accident. scripts/static_audit.py enforces that.
struct WifiStatus {
  WifiState state = WifiState::INACTIVE;
  WifiFault fault = WifiFault::NONE;

  bool enabled = false;              // operator/build intent, not link state
  bool credentials_present = false;  // an SSID is configured at all
  bool connected = false;            // the radio reports an associated link

  // The CONFIGURED network identity, copied once at begin(). Not read back
  // from the radio per refresh: WiFi.SSID() returns an Arduino String and
  // would allocate on every poll, in the control loop. An SSID is not a
  // secret, so publishing it is intentional.
  char ssid[33] = {0};

  int32_t rssi_dbm = 0;  // valid only while connected
  // Octet 0 in the least-significant byte — the same layout Arduino's
  // IPAddress::operator uint32_t() produces, so the manager can assign it
  // directly without byte-swapping. Use formatIpv4() to render it.
  uint32_t ipv4 = 0;
  uint8_t channel = 0;

  uint32_t backoff_ms = 0;       // the wait currently applied in BACKOFF
  uint32_t state_since_ms = 0;   // millis() timestamp of the last transition

  WifiCounters counters{};

  // Evidence that the Wi-Fi layer is bounded, measured rather than claimed.
  // Wall time spent inside WifiManager::update() on the last tick, and the
  // worst tick since boot.
  uint32_t last_update_us = 0;
  uint32_t max_update_us = 0;

  // Reconnects, in the operator's sense: associations after the first one.
  uint32_t reconnects() const {
    return counters.connects > 0 ? counters.connects - 1 : 0;
  }
};

// Pure, bounded, allocation-free Wi-Fi lifecycle state machine.
//
// It is driven entirely by (now_ms, link_up) and returns at most one action
// per call. It makes at most one state transition per call, so no single
// Controller tick can be dragged through several radio operations.
class WifiPolicy {
 public:
  void begin(const WifiPolicyConfig& config, bool credentials_present, uint32_t now_ms);

  // Operator intent (@WIFI ON/OFF). Switching off does not itself touch the
  // radio; the next update() returns STOP_RADIO.
  void setEnabled(bool enabled, uint32_t now_ms);

  // One bounded evaluation. `link_up` is what the radio reports right now.
  WifiAction update(uint32_t now_ms, bool link_up);

  // The radio owner reports that the action it was just told to perform
  // failed. Fails closed: the policy goes to BACKOFF rather than assuming
  // the radio is in the state it asked for.
  void reportActionFailed(WifiFault fault, uint32_t now_ms);

  WifiState state() const { return state_; }
  WifiFault fault() const { return fault_; }
  bool enabled() const { return enabled_; }
  bool credentialsPresent() const { return credentials_present_; }
  bool radioStarted() const { return radio_started_; }
  uint32_t backoffMs() const { return backoff_ms_; }
  uint32_t stateSinceMs() const { return state_since_ms_; }
  const WifiCounters& counters() const { return counters_; }

 private:
  void enter(WifiState next, uint32_t now_ms);
  void enterBackoff(uint32_t now_ms);

  WifiPolicyConfig config_{0, 0, 0, 0};
  bool enabled_ = false;
  bool credentials_present_ = false;
  bool radio_started_ = false;
  WifiState state_ = WifiState::INACTIVE;
  WifiFault fault_ = WifiFault::NONE;
  uint32_t state_since_ms_ = 0;
  uint32_t backoff_ms_ = 0;
  WifiCounters counters_{0, 0, 0, 0, 0};
};

const char* toString(WifiState state);
const char* toString(WifiFault fault);

// Renders `ipv4` (octet 0 in the low byte) as "a.b.c.d" into `out`, which
// must hold at least 16 bytes. Always NUL-terminates. Pure so the host
// tests cover the exact formatting the operator will read on the wire.
void formatIpv4(uint32_t ipv4, char* out, uint32_t out_size);

}  // namespace network
}  // namespace matdog

#endif  // MATDOG_NETWORK_WIFI_POLICY_H
