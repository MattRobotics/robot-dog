#ifndef MATDOG_NETWORK_CONFIG_NVS_H
#define MATDOG_NETWORK_CONFIG_NVS_H
#include "NetworkConfig.h"
namespace matdog {
namespace network {
class NetworkConfigNvs : public NetworkStorage {
public:
  bool load(NetworkConfig *out) override;
  bool pending(const NetworkConfig &config) override;
  bool activate(const NetworkConfig &config) override;
  int32_t lastLoadError() const { return load_error_; }

private:
  int32_t load_error_ = 0;
  bool write(const char *key, const NetworkConfig &config);
};
} // namespace network
} // namespace matdog
#endif
