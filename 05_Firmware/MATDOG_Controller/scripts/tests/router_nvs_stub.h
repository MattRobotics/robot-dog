#ifndef MATDOG_TEST_ROUTER_NVS_STUB_H
#define MATDOG_TEST_ROUTER_NVS_STUB_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

// Platform NVS API only. The router, service, record store and NVS adapter
// are real; counters observe calls made by CalibrationRecordNvsBackend.
namespace router_nvs_test {
struct State {
  std::map<std::string, std::vector<uint8_t>> blobs;
  bool initialized = false;
  bool namespace_present = false;
  bool opened_rw = false;
  unsigned set_calls = 0;
  unsigned commit_calls = 0;
  unsigned rw_open_calls = 0;
  unsigned contract_errors = 0;
};
extern State state;
void reset();
}  // namespace router_nvs_test

#endif
