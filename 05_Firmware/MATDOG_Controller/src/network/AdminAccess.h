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
