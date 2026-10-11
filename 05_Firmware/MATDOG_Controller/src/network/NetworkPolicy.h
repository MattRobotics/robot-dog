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
// ---- Radio role by operating mode (2026-10-11) ---------------------------------
// One radio, one role at a time. The controller's operating mode decides it,
// together with one stored fact: whether a router profile is enabled.
//
//   MAINTENANCE  -> ADMIN_AP, always. A stored router profile is not used; the
//                   only STA activity is the explicit, temporary "Prova e
//                   salva" transaction, which returns to the AP by itself.
//   RUN          -> CLIENT_STA when a router profile is enabled, else the
//                   recovery AP. A router that stays unreachable or refuses the
//                   credentials also leads to the recovery AP, from which the
//                   router is tried again after a growing wait.
//
// The stored "AP always" preference of earlier firmware (NetworkConfig) is
// NOT an input: a record that still carries it is read and kept as it is, and
// it can no longer keep the controller off a configured router in RUN.
//
// This is a pure decision: it reads the mode, it can never change it, and it
// has no access to authority, permits or actuators.
enum class RoleReason : uint8_t {
  MAINTENANCE_AP = 0,     // maintenance: administrative AP, portal reachable
  RUN_STA = 1,            // run: join the configured router
  RUN_NO_PROFILE_AP = 2,  // run: no enabled router profile -> recovery AP
  RUN_RECOVERY_AP = 3,    // run: router attempts failed -> recovery AP, retry later
};
struct RoleInputs {
  bool run = false, sta_profile = false, recovery = false;
};
inline RoleReason selectRole(const RoleInputs &i) {
  if (!i.run)
    return RoleReason::MAINTENANCE_AP;
  if (!i.sta_profile)
    return RoleReason::RUN_NO_PROFILE_AP;
  return i.recovery ? RoleReason::RUN_RECOVERY_AP : RoleReason::RUN_STA;
}
inline bool roleIsSta(RoleReason r) { return r == RoleReason::RUN_STA; }
inline const char *toString(RoleReason r) {
  switch (r) {
  case RoleReason::MAINTENANCE_AP: return "MAINTENANCE_AP";
  case RoleReason::RUN_STA: return "RUN_STA";
  case RoleReason::RUN_NO_PROFILE_AP: return "RUN_NO_PROFILE_AP";
  case RoleReason::RUN_RECOVERY_AP: return "RUN_RECOVERY_AP";
  }
  return "UNKNOWN";
}
// How long the recovery AP is kept before the router is tried again: 60 s,
// doubling after every failed return, never above the configured AP timeout.
// This is what keeps a missing router from turning into AP/STA oscillation.
constexpr uint32_t kRecoveryDwellInitialMs = 60000;
inline uint32_t recoveryDwellMs(uint8_t level, uint32_t cap_ms) {
  const uint64_t dwell = static_cast<uint64_t>(kRecoveryDwellInitialMs)
                         << (level > 16 ? 16 : level);
  return dwell > cap_ms ? cap_ms : static_cast<uint32_t>(dwell);
}
inline bool expireAp(uint32_t now, uint32_t since, const NetworkConfig &c,
                     bool sta, bool transaction, unsigned clients) {
  return sta && !transaction && !clients && !c.ap_always &&
         now - since >= c.ap_timeout_ms;
}
} // namespace network
} // namespace matdog
#endif
