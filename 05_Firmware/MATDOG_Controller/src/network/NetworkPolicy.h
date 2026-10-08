#ifndef MATDOG_NETWORK_POLICY_H
#define MATDOG_NETWORK_POLICY_H
#include "NetworkConfig.h"
namespace matdog {
namespace network {
struct SleepInputs {
  bool session_trusted = false, session_open = false, ota = false,
       provisioning = false;
  uint8_t preference = 0;
};
inline bool modemSleep(const SleepInputs &i) {
  return i.session_trusted && i.session_open && !i.ota && !i.provisioning &&
         i.preference != 2;
}
class RoamPolicy {
public:
  bool scan(uint32_t now, const NetworkConfig &c, bool connected, int rssi,
            bool critical) {
    if (critical || !connected || !c.roam_enabled || rssi > c.roam_threshold ||
        (scanned_ && now - last_scan_ < c.scan_interval_ms) ||
        (roamed_ && now - last_roam_ < c.roam_dwell_ms))
      return false;
    scanned_ = true;
    last_scan_ = now;
    return true;
  }
  bool roam(uint32_t now, const NetworkConfig &c, int current, int candidate,
            bool different, bool critical) {
    if (critical || !different || !c.roam_enabled ||
        current > c.roam_threshold || candidate - current < c.roam_hysteresis ||
        (roamed_ && now - last_roam_ < c.roam_dwell_ms))
      return false;
    roamed_ = true;
    last_roam_ = now;
    return true;
  }

private:
  bool scanned_ = false, roamed_ = false;
  uint32_t last_scan_ = 0, last_roam_ = 0;
};
inline bool expireAp(uint32_t now, uint32_t since, const NetworkConfig &c,
                     bool sta, bool transaction, unsigned clients) {
  return sta && !transaction && !clients && !c.ap_always &&
         now - since >= c.ap_timeout_ms;
}
} // namespace network
} // namespace matdog
#endif
