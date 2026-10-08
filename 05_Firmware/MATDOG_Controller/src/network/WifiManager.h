#ifndef MATDOG_NETWORK_WIFI_MANAGER_H
#define MATDOG_NETWORK_WIFI_MANAGER_H
#include <atomic>

#include "NetworkConfigNvs.h"
#include "NetworkPolicy.h"
#include "PortalSecurity.h"
#include "WifiPolicy.h"
namespace matdog {
namespace network {
// One low-priority worker owns all radio and network NVS I/O. Controller update
// only takes a snapshot with a zero-wait mutex. Requests use a bounded one-slot
// queue.
class WifiManager {
public:
  static constexpr uint32_t kRadioSettleMs = 100, kConnectTimeoutMs = 15000,
                            kBackoffInitialMs = 2000, kBackoffMaxMs = 60000,
                            kRadioPollIntervalMs = 1000;
  void begin(uint32_t now_ms);
  void update(uint32_t now_ms);
  bool setEnabled(bool enabled, uint32_t now_ms);
  bool requestScan();
  bool requestAp(bool on);
  bool setSleep(uint8_t preference);
  bool provisionAp(const char *password);
  bool provisionAdmin(const char *token);
  bool configure(const ConfigPatch &patch);
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
    CONFIG
  };
  struct Request {
    RequestKind kind{};
    bool on = false;
    uint8_t preference = 0;
    char secret[65]{};
    ConfigPatch patch{};
  };
  bool post(const Request &request, bool mutation);
  static void taskEntry(void *self);
  void worker();
  void workerTick(uint32_t now);
  void consume(const Request &request, uint32_t now);
  void connectProfile(uint32_t now);
  void issueConnect(uint32_t now);
  void startAp(uint32_t now);
  void stopAp();
  void scan(uint32_t now, bool roaming);
  void publishPolicyState();
  void refreshSnapshot(uint32_t now, bool link_up);
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
