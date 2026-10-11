#ifndef MATDOG_NETWORK_WIFI_MANAGER_H
#define MATDOG_NETWORK_WIFI_MANAGER_H
#include <atomic>

#include "AdminAccess.h"
#include "NetworkConfigNvs.h"
#include "NetworkPolicy.h"
#include "PortalSecurity.h"
#include "WifiPolicy.h"
namespace matdog {
namespace network {
// One low-priority worker owns all radio and network NVS I/O. Controller update
// only takes a snapshot with a zero-wait mutex. Requests use a bounded one-slot
// queue.
// Synchronous outcome of handing a request to the worker. QUEUED is an intent;
// its result is published later under the same sequence number.
enum class PostResult : uint8_t {
  NONE, QUEUED, UNAVAILABLE, INVALID, CRITICAL, NVS_FAULT, NOT_ADMITTED, BUSY,
  QUEUE_FULL
};
// Worker-side outcome of the last consumed configuration request.
enum class TxResult : uint8_t {
  NONE, COMMITTED, REJECTED, STORAGE_ERROR, INTENT_PENDING, ROLLBACK
};
enum class VerifyResult : uint8_t { NONE, MATCH, MISMATCH, NOT_AVAILABLE, THROTTLED };
inline const char *toString(PostResult r) {
  switch (r) {
  case PostResult::NONE: return "NONE";
  case PostResult::QUEUED: return "QUEUED";
  case PostResult::UNAVAILABLE: return "UNAVAILABLE";
  case PostResult::INVALID: return "INVALID";
  case PostResult::CRITICAL: return "CRITICAL_CONTEXT";
  case PostResult::NVS_FAULT: return "NVS_FAULT";
  case PostResult::NOT_ADMITTED: return "NOT_ADMITTED";
  case PostResult::BUSY: return "BUSY";
  case PostResult::QUEUE_FULL: return "QUEUE_FULL";
  }
  return "UNKNOWN";
}
inline const char *toString(TxResult r) {
  switch (r) {
  case TxResult::NONE: return "NONE";
  case TxResult::COMMITTED: return "COMMITTED";
  case TxResult::REJECTED: return "REJECTED";
  case TxResult::STORAGE_ERROR: return "STORAGE_ERROR";
  case TxResult::INTENT_PENDING: return "INTENT_PENDING";
  case TxResult::ROLLBACK: return "ROLLBACK";
  }
  return "UNKNOWN";
}
inline const char *toString(VerifyResult r) {
  switch (r) {
  case VerifyResult::NONE: return "NONE";
  case VerifyResult::MATCH: return "MATCH";
  case VerifyResult::MISMATCH: return "MISMATCH";
  case VerifyResult::NOT_AVAILABLE: return "NOT_AVAILABLE";
  case VerifyResult::THROTTLED: return "THROTTLED";
  }
  return "UNKNOWN";
}
class WifiManager {
public:
  static constexpr uint32_t kRadioSettleMs = 100, kConnectTimeoutMs = 15000,
                            kBackoffInitialMs = 2000, kBackoffMaxMs = 60000,
                            kRadioPollIntervalMs = 1000,
                            // STA test ("Prova e salva"): a failed attempt is
                            // repeated inside the 30 s transaction window, and
                            // the candidate is committed only after association
                            // and address were held for the stability window.
                            kStaTestRetryMs = 4000, kStaTestHoldMs = 1500;
  void begin(uint32_t now_ms);
  void update(uint32_t now_ms);
  bool setEnabled(bool enabled, uint32_t now_ms);
  bool requestScan();
  bool requestAp(bool on, uint32_t generation = 0);
  bool setSleep(uint8_t preference);
  bool provisionAp(const char *password);
  bool provisionAdmin(const char *token);
  // Compares an operator-supplied AP key with the stored one and with the
  // profile the driver is really using. Publishes MATCH/MISMATCH only.
  bool verifyAp(const char *candidate);
  // Controller-thread view of the most recent post(): why it was refused, or
  // the sequence number under which the worker will publish its result.
  PostResult lastPost() const { return last_post_; }
  uint32_t lastPostSeq() const { return last_post_seq_; }
  static const char *requestKindName(uint8_t kind) {
  switch (static_cast<RequestKind>(kind)) {
  case RequestKind::ENABLE: return "ENABLE";
  case RequestKind::AP: return "AP";
  case RequestKind::SCAN: return "SCAN";
  case RequestKind::SLEEP: return "SLEEP";
  case RequestKind::AP_KEY: return "AP_KEY";
  case RequestKind::ADMIN_KEY: return "ADMIN_KEY";
  case RequestKind::CONFIG: return "CONFIG";
  case RequestKind::AP_VERIFY: return "AP_VERIFY";
  }
  return "UNKNOWN";
}
  bool configure(const ConfigPatch &patch, uint32_t generation = 0);
  AdminAccess &adminAccess() { return admin_; }
  static bool verifyAdminRadio();
  bool configBusy() const { return config_busy_.load(); }
  void setContext(bool critical, bool ota) {
    context_.store((critical ? 1u : 0u) | (ota ? 2u : 0u));
  }
  const WifiStatus &status() const { return status_; }
  // Called only by ControllerService from Controller thread. Hash, never raw
  // admin token.
  const uint8_t *adminDigest() const { return admin_digest_; }

private:
  friend class WifiRuntimeTest;
  void initializeWorker();
  bool ap_reload_ = false;
  enum class RequestKind : uint8_t {
    ENABLE,
    AP,
    SCAN,
    SLEEP,
    AP_KEY,
    ADMIN_KEY,
    CONFIG,
    AP_VERIFY
  };
  struct Request {
    RequestKind kind{};
    uint32_t generation = 0;
    uint32_t seq = 0;
    bool on = false;
    uint8_t preference = 0;
    char secret[65]{};
    ConfigPatch patch{};
  };
  bool post(const Request &request, bool mutation);
  static void taskEntry(void *self);
  void worker();
  void workerTick(uint32_t now);
  // false: the admission fence was momentarily held elsewhere; retry next tick.
  bool consume(const Request &request, uint32_t now);
  void recordTx(const Request &request, TxResult result, int32_t error);
  void verifyCandidate(const Request &request, uint32_t now);
  void fault(FaultSite site, int32_t code);
  void noteFault(FaultSite site, int32_t code);
  void clearRadioError();
  void faultClose(uint32_t now);
  void enterAdminTick(uint32_t now);
  bool issueApConfig(uint32_t now);
  void beginStaTest(uint32_t now);
  void trackStaTest(uint32_t now);
  void finishStaTest(uint32_t now, bool aborted);
  void refuseStaTest(bool storage_error);
  bool apConfigApplied();
  // Bring-up is two serialized driver commands, each with its own complete
  // acknowledgment set: the role change, then the AP profile.
  enum class ApStep : uint8_t { IDLE, MODE_ISSUED, CONFIG_ISSUED };
  ApStep ap_step_ = ApStep::IDLE;
  bool fault_close_issued_ = false;
  Request held_{};
  bool held_valid_ = false, verify_used_ = false;
  uint32_t held_attempts_ = 0, verify_since_ = 0, request_seq_ = 0, last_post_seq_ = 0;
  PostResult last_post_ = PostResult::NONE;
  void connectProfile(uint32_t now);
  void issueConnect(uint32_t now);
  void startAp(uint32_t now);
  void stopAp();
  void transitionTick(uint32_t now);
  bool changeMode(uint8_t mode, uint32_t now);
  void observeRadioEvent(int event);
  std::atomic<uint8_t> expected_events_{0};
  uint32_t event_since_ = 0;
  bool leave_command_issued_ = false;
  void leaveAdmin(bool return_admin, uint32_t now);
  void scan(uint32_t now, bool roaming);
  void publishPolicyState();
  void refreshSnapshot(uint32_t now, bool link_up);
  AdminAccess admin_{};
  bool transition_admin_ = false, intent_pending_ = false, return_admin_ = false;
  NetworkConfig intent_{};
  uint8_t intent_profile_ = 0;
  uint32_t intent_since_ = 0;
  WifiPolicy policy_{};
  NetworkConfigNvs storage_{};
  ConfigTransaction transaction_{};
  RoamPolicy roam_{};
  NetworkConfig config_{};
  WifiStatus status_{}, shared_status_{}, worker_status_{};
  uint8_t admin_digest_[32]{}, shared_admin_digest_[32]{};
  void *queue_ = nullptr;
  void *mutex_ = nullptr;
  std::atomic<unsigned> context_{0};
  std::atomic<bool> config_busy_{false};
  std::atomic<uint16_t> disconnect_reason_{0};
  // STA test evidence, written by the event task, read by the worker.
  std::atomic<uint16_t> sta_reason_{0};
  std::atomic<uint32_t> sta_disconnects_{0};
  std::atomic<bool> sta_associated_{false}, sta_link_{false};
  uint32_t intent_seq_ = 0, test_since_ = 0, test_connect_ms_ = 0,
           test_stable_since_ = 0, test_disconnects_ = 0;
  bool test_stable_ = false;
  uint32_t ap_since_ = 0, radio_poll_ms_ = 0, off_since_ = 0;
  bool enabled_ = true, ap_manual_ = false, scan_roam_ = false;
  std::atomic<bool> nvs_fault_{false};
  bool connect_due_ = false, test_fresh_ = false;
  uint32_t connect_since_ = 0, snapshot_max_us_ = 0;
  uint8_t profile_ = 0;
  bool radio_polled_ = false;
};
} // namespace network
} // namespace matdog
#endif
