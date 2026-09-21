#ifndef MATDOG_NETWORK_WIFI_MANAGER_H
#define MATDOG_NETWORK_WIFI_MANAGER_H

#include <stdint.h>

#include "WifiPolicy.h"

// Intentionally does NOT include <WiFi.h>. The radio lives in exactly one
// translation unit (WifiManager.cpp), so:
//
//   - core/Controller.h and core/CommandRouter.h can own and read this
//     module without pulling the Wi-Fi stack into their headers;
//   - scripts/static_audit.py::check_no_network_to_servo_path sees exactly
//     one network translation unit to police, and that unit contains no
//     servo primitive. That check is the executable form of the permanent
//     rule in ARCHITECTURE.md: "network callback != servo command
//     authority".

namespace matdog {
namespace network {

// Owns the Wi-Fi radio and nothing else.
//
// It does not own a task, a thread, a queue or a callback. Link state is
// POLLED from Controller::update(), on purpose: an Arduino WiFi event
// handler runs in the system event task, so any state it touched would be
// shared across tasks and would need locking to stay honest. Polling a
// status word costs a few microseconds and keeps the entire Wi-Fi state
// machine single-threaded inside the Controller loop, which is also what
// makes WifiPolicy host-testable. If a future feature genuinely needs
// events (it should be justified, not assumed), it must not write shared
// state from the event task.
//
// Nothing here can command an actuator, change OperatingMode, or reach
// ServoBus. There is no path from this module to any of them.
class WifiManager {
 public:
  // Gap between starting the driver and issuing the association — see
  // WifiPolicyConfig::radio_settle_ms for the 1000 ms blocking wait inside
  // WiFi.begin() that this avoids. 100 ms is thousands of Controller ticks.
  static constexpr uint32_t kRadioSettleMs = 100;

  // Association + DHCP deadline. A busy 2.4 GHz band with a slow DHCP
  // lease can legitimately take ~10 s; below that the policy would give up
  // on connections that were about to succeed and churn the radio.
  static constexpr uint32_t kConnectTimeoutMs = 15000;

  static constexpr uint32_t kBackoffInitialMs = 2000;

  // Ceiling on the doubling retry ladder: 2s, 4s, 8s ... 60s. An AP that
  // is off overnight must not produce a retry storm, but the robot must
  // still rejoin within a minute of the AP coming back without operator
  // action.
  static constexpr uint32_t kBackoffMaxMs = 60000;

  // RSSI/IP/channel are refreshed at most at this rate, not every tick.
  // The snapshot is what @STATUS and the future Web UI read, so telemetry
  // consumers never drive radio queries (ARCHITECTURE.md, telemetry
  // snapshot model).
  static constexpr uint32_t kRadioPollIntervalMs = 1000;

  // Configures the policy and publishes the initial snapshot. Deliberately
  // does NOT touch the radio: the first WiFi.mode() call initializes the
  // Wi-Fi driver and allocates tens of KB of heap, and Controller::begin()
  // is not the place for that. The radio comes up from update().
  void begin(uint32_t now_ms);

  // Bounded: one link-state poll, at most one radio action, one snapshot
  // refresh. No loop, no delay, no wait-for-result. The measured cost of
  // this call is published in status().last_update_us / max_update_us.
  void update(uint32_t now_ms);

  // Operator intent (@WIFI ON / @WIFI OFF). Returns false, and changes
  // nothing, when there are no credentials to connect with.
  bool setEnabled(bool enabled, uint32_t now_ms);

  // The structured snapshot. Returning the cached struct — never a live
  // radio query — is what lets USB CDC and a future Web adapter render the
  // same state without a duplicate hardware path.
  const WifiStatus& status() const { return status_; }

 private:
  void refreshSnapshot(uint32_t now_ms, bool link_up);

  WifiPolicy policy_{};
  WifiStatus status_{};
  uint32_t radio_poll_ms_ = 0;
  bool radio_polled_ = false;
};

}  // namespace network
}  // namespace matdog

#endif  // MATDOG_NETWORK_WIFI_MANAGER_H
