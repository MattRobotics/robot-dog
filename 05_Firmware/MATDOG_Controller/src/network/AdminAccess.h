#ifndef MATDOG_ADMIN_ACCESS_H
#define MATDOG_ADMIN_ACCESS_H
#include <atomic>
#include <stdint.h>
#include <string.h>
namespace matdog { namespace network {
enum class NetworkPhase : uint8_t { CLOSED, ENTER_ADMIN, ADMIN_AP, LEAVE_ADMIN, WAIT_AP_CLOSE, CLIENT_STA, FAULT };
inline const char *toString(NetworkPhase phase) {
  switch (phase) {
  case NetworkPhase::CLOSED: return "CLOSED";
  case NetworkPhase::ENTER_ADMIN: return "ENTER_ADMIN";
  case NetworkPhase::ADMIN_AP: return "ADMIN_AP";
  case NetworkPhase::LEAVE_ADMIN: return "LEAVE_ADMIN";
  case NetworkPhase::WAIT_AP_CLOSE: return "WAIT_AP_CLOSE";
  case NetworkPhase::CLIENT_STA: return "CLIENT_STA";
  case NetworkPhase::FAULT: return "FAULT";
  }
  return "FAULT";
}
// Where a FAULT was first observed. Diagnostic only: it never authorizes or
// reopens anything, and carries no credential material.
enum class FaultSite : uint8_t {
  NONE = 0,
  UNEXPECTED_ROLE_EVENT = 1,   // role event outside the armed acknowledgment set
  ROLE_ACK_TIMEOUT = 2,        // an armed acknowledgment never arrived
  ADMIN_RADIO_LOST = 3,        // worker: ADMIN_AP no longer verifiable on the driver
  AP_START_TIMEOUT = 4,        // AP role not started after the mode command
  AP_VERIFY_TIMEOUT = 5,       // AP configured but exclusive AP state not verifiable
  CLOSE_VERIFY_TIMEOUT = 6,    // radio-off closure not verifiable
  STA_VERIFY_TIMEOUT = 7,      // return to STA not verifiable
  AP_MODE_COMMAND = 8,         // driver refused the AP mode command
  AP_IP_CONFIG = 9,            // AP netif address configuration refused
  AP_SET_CONFIG = 10,          // driver refused the AP profile
  AP_CONFIG_MISMATCH = 11,     // driver AP profile differs from the stored one
  HTTP_RADIO_LOST = 12,        // lifecycle task: ADMIN_AP no longer verifiable
  WEB_MANUAL_STOP = 13,        // operator stopped the web server
  LISTENER_STOP_FAILED = 14,   // HTTPD refused to stop
  MODE_COMMAND = 15,           // driver refused a leave/close mode command
  ADMIN_OPEN_REFUSED = 16,     // generation exhausted or fence missing
  MAC_READ_FAILED = 17,
  INIT_MODE_FAILED = 18,       // initial STA driver start failed
  UNSPECIFIED = 19,
};
inline const char *toString(FaultSite site) {
  switch (site) {
  case FaultSite::NONE: return "NONE";
  case FaultSite::UNEXPECTED_ROLE_EVENT: return "UNEXPECTED_ROLE_EVENT";
  case FaultSite::ROLE_ACK_TIMEOUT: return "ROLE_ACK_TIMEOUT";
  case FaultSite::ADMIN_RADIO_LOST: return "ADMIN_RADIO_LOST";
  case FaultSite::AP_START_TIMEOUT: return "AP_START_TIMEOUT";
  case FaultSite::AP_VERIFY_TIMEOUT: return "AP_VERIFY_TIMEOUT";
  case FaultSite::CLOSE_VERIFY_TIMEOUT: return "CLOSE_VERIFY_TIMEOUT";
  case FaultSite::STA_VERIFY_TIMEOUT: return "STA_VERIFY_TIMEOUT";
  case FaultSite::AP_MODE_COMMAND: return "AP_MODE_COMMAND";
  case FaultSite::AP_IP_CONFIG: return "AP_IP_CONFIG";
  case FaultSite::AP_SET_CONFIG: return "AP_SET_CONFIG";
  case FaultSite::AP_CONFIG_MISMATCH: return "AP_CONFIG_MISMATCH";
  case FaultSite::HTTP_RADIO_LOST: return "HTTP_RADIO_LOST";
  case FaultSite::WEB_MANUAL_STOP: return "WEB_MANUAL_STOP";
  case FaultSite::LISTENER_STOP_FAILED: return "LISTENER_STOP_FAILED";
  case FaultSite::MODE_COMMAND: return "MODE_COMMAND";
  case FaultSite::ADMIN_OPEN_REFUSED: return "ADMIN_OPEN_REFUSED";
  case FaultSite::MAC_READ_FAILED: return "MAC_READ_FAILED";
  case FaultSite::INIT_MODE_FAILED: return "INIT_MODE_FAILED";
  case FaultSite::UNSPECIFIED: return "UNSPECIFIED";
  }
  return "UNSPECIFIED";
}
// Admission revocation is nonblocking, including from the Arduino event task.
// The mutex fences an admitted operation through its last mutation. Closing
// drains it before acknowledging listener shutdown. Never wait in callbacks.
class AdminAccess {
public:
  bool begin();
  bool lock();
  void unlock();
  // Digest access is protected by AdminLease, like generation-bound login.
  void setDigest(const uint8_t digest[32]) { memcpy(digest_, digest, 32); }
  const uint8_t *digest() const { return digest_; }
  uint32_t generation() const { return generation_.load(); }
  bool allowed(uint32_t g) const {
    return g && admitted_.load() && !closing.load() && !radio_fault.load() &&
           phase.load() == NetworkPhase::ADMIN_AP && g == generation_.load();
  }
  void revoke() {
    admitted_.store(false);
    // First close request invalidates the acknowledgement; repeated observations
    // cannot erase an acknowledgement of this same closure.
    if (!closing.exchange(true)) listeners_closed.store(false);
  }
  // Nonblocking fault report usable from the event task and the lifecycle
  // task. The first cause of an episode is kept until the worker consumes it.
  void raiseFault(FaultSite site, uint8_t event = 0, uint8_t expected = 0) {
    uint8_t none = 0;
    if (fault_source.compare_exchange_strong(none, static_cast<uint8_t>(site))) {
      fault_event.store(event);
      fault_expected.store(expected);
    }
    revoke();
    radio_fault.store(true);
  }
  // As raiseFault, for a task that already decided the phase itself.
  void noteFault(FaultSite site) {
    uint8_t none = 0;
    fault_source.compare_exchange_strong(none, static_cast<uint8_t>(site));
  }
  bool open() { // caller holds fence, after driver/AP/STA verification
    if (!mutex_ || generation_.load() == UINT32_MAX) return false;
    generation_.fetch_add(1);
    phase.store(NetworkPhase::ADMIN_AP);
    closing.store(false);
    admitted_.store(true);
    return true;
  }
  std::atomic<NetworkPhase> phase{NetworkPhase::CLOSED};
  std::atomic<bool> closing{true}, listeners_closed{false}, radio_fault{false};
  std::atomic<uint8_t> fault_source{0}, fault_event{0}, fault_expected{0};
private:
  uint8_t digest_[32]{};
  void *mutex_ = nullptr;
  std::atomic<uint32_t> generation_{0};
  std::atomic<bool> admitted_{false};
};
class AdminLease {
public:
  explicit AdminLease(AdminAccess &a) : access_(a), held_(a.lock()) {}
  ~AdminLease() { if (held_) access_.unlock(); }
  explicit operator bool() const { return held_; }
  AdminLease(const AdminLease &) = delete;
  AdminLease &operator=(const AdminLease &) = delete;
private:
  AdminAccess &access_;
  bool held_;
};
} }
#endif
