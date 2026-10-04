#ifndef MATDOG_NETWORK_CONFIG_H
#define MATDOG_NETWORK_CONFIG_H
#include <stddef.h>
#include <stdint.h>
namespace matdog {
namespace network {
struct StaProfile {
  char ssid[33]{};
  char password[65]{};
  bool enabled = false;
  bool dhcp = true;
  uint32_t ip = 0, mask = 0, gateway = 0, dns = 0;
};
struct NetworkConfig {
  StaProfile sta[2]{};
  char ap_name[33]{}; // empty => MATDOG-<STA MAC suffix>
  char ap_password[65]{};
  uint8_t admin_digest[32]{}; // SHA256 of high-entropy USB-provisioned 64-hex
                              // admin token
  uint32_t ap_timeout_ms = 900000;
  bool ap_always = false;
  bool roam_enabled =
      false; // deliberate opt-in until jitter and AP behaviour qualified
  int8_t roam_threshold = -80;
  uint8_t roam_hysteresis = 8;
  uint32_t scan_interval_ms = 60000, roam_dwell_ms = 120000;
  uint8_t bandwidth_mhz = 20;
  uint8_t usb_sleep =
      0; // AUTO; 1 ON, 2 OFF. Actual HWCDC adapter remains untrusted.
};
constexpr size_t kNetworkRecordBytes = 512;
bool parseIpv4(const char *text, uint32_t *out);
bool validNetworkConfig(const NetworkConfig &config);
bool hasAdmin(const NetworkConfig &config);
void encodeNetworkConfig(const NetworkConfig &config,
                         uint8_t out[kNetworkRecordBytes]);
bool decodeNetworkConfig(const uint8_t *bytes, size_t len, NetworkConfig *out);
class NetworkStorage {
public:
  virtual ~NetworkStorage() = default;
  virtual bool load(NetworkConfig *out) = 0;
  virtual bool pending(const NetworkConfig &config) = 0;
  virtual bool activate(const NetworkConfig &config) = 0;
};
enum class ConfigPhase : uint8_t {
  IDLE,
  CONFIG_PENDING,
  TESTING,
  COMMITTED,
  ROLLBACK,
  STORAGE_ERROR
};
const char *toString(ConfigPhase phase);
// Atomic ACTIVE blob is the known-good authority. PENDING is never activated at
// boot.
class ConfigTransaction {
public:
  void begin(NetworkStorage *storage, const NetworkConfig &known_good);
  bool stage(const NetworkConfig &candidate, uint8_t tested_profile,
             uint32_t now, bool alternate_access, bool critical);
  void update(uint32_t now, bool associated, bool valid_ip, bool critical);
  bool busy() const {
    return phase_ == ConfigPhase::CONFIG_PENDING ||
           phase_ == ConfigPhase::TESTING;
  }
  ConfigPhase phase() const { return phase_; }
  const NetworkConfig &active() const { return active_; }
  const NetworkConfig &candidate() const { return candidate_; }

private:
  NetworkStorage *storage_ = nullptr;
  NetworkConfig active_{}, candidate_{};
  ConfigPhase phase_ = ConfigPhase::IDLE;
  uint32_t started_ = 0;
};
} // namespace network
} // namespace matdog
#endif
