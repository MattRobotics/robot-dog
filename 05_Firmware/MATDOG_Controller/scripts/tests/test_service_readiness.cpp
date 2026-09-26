// Offline host tests for the HostLink readiness classifier
// (src/core/ServiceReadiness.*) — I6, implemented per the 2026-09-25
// objective-change instruction.
//
// Links the REAL classifier, the same contract as every other pure policy
// suite in this codebase. Pure enum-in, enum-out logic: no framework, a
// CHECK macro and a pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/core/ServiceReadiness.h"

using namespace matdog::core;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK_EQ(actual, expected)                                         \
  do {                                                                     \
    ++g_checks;                                                            \
    const long a_ = (long)(actual);                                        \
    const long e_ = (long)(expected);                                      \
    if (a_ != e_) {                                                        \
      ++g_failures;                                                        \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,  \
                  __FILE__, __LINE__, #actual, a_, e_);                    \
    }                                                                      \
  } while (0)

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                              \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));          \
    }                                                                          \
  } while (0)

namespace {

void test_actuator_and_calibration_capabilities_track_hardware_motion_authorized() {
  g_case = "actuator/calibration capabilities";
  for (ServiceCapability cap : {ServiceCapability::ACTUATOR_TORQUE_ENABLE,
                                ServiceCapability::ACTUATOR_POSITION_COMMAND,
                                ServiceCapability::CALIBRATION_CONTACT_PROBE,
                                ServiceCapability::CALIBRATION_AUXILIARY_MOVE}) {
    ServiceReadinessInputs in;
    in.hardware_motion_authorized = false;
    CHECK_EQ((int)classifyCapability(cap, in), (int)ServiceReadiness::BLOCKED);

    in.hardware_motion_authorized = true;
    CHECK_EQ((int)classifyCapability(cap, in), (int)ServiceReadiness::TO_TEST);
  }
}

void test_direction_verify_is_always_to_test_never_blocked() {
  g_case = "direction verify";
  ServiceReadinessInputs in;
  in.hardware_motion_authorized = false;
  CHECK_EQ((int)classifyCapability(ServiceCapability::CALIBRATION_DIRECTION_VERIFY, in),
          (int)ServiceReadiness::TO_TEST);
  in.hardware_motion_authorized = true;
  CHECK_EQ((int)classifyCapability(ServiceCapability::CALIBRATION_DIRECTION_VERIFY, in),
          (int)ServiceReadiness::TO_TEST);
}

void test_wifi_and_ota_track_their_own_hardware_validation_flag_only() {
  g_case = "wifi/ota independence";
  ServiceReadinessInputs in;
  // Neither flag set: both TO_TEST.
  CHECK_EQ((int)classifyCapability(ServiceCapability::WIFI_HARDWARE_ASSOCIATION, in),
          (int)ServiceReadiness::TO_TEST);
  CHECK_EQ((int)classifyCapability(ServiceCapability::OTA_END_TO_END, in),
          (int)ServiceReadiness::TO_TEST);

  // Wi-Fi validated, OTA not: only Wi-Fi becomes READY.
  in.wifi_hardware_validated = true;
  CHECK_EQ((int)classifyCapability(ServiceCapability::WIFI_HARDWARE_ASSOCIATION, in),
          (int)ServiceReadiness::READY);
  CHECK_EQ((int)classifyCapability(ServiceCapability::OTA_END_TO_END, in),
          (int)ServiceReadiness::TO_TEST);

  // OTA validated too: both READY. hardware_motion_authorized never
  // participates in either decision.
  in.ota_hardware_validated = true;
  in.hardware_motion_authorized = true;
  CHECK_EQ((int)classifyCapability(ServiceCapability::WIFI_HARDWARE_ASSOCIATION, in),
          (int)ServiceReadiness::READY);
  CHECK_EQ((int)classifyCapability(ServiceCapability::OTA_END_TO_END, in),
          (int)ServiceReadiness::READY);
}

void test_web_dashboard_is_blocked_until_wifi_hardware_validated() {
  g_case = "web dashboard entry condition";
  ServiceReadinessInputs in;
  in.wifi_hardware_validated = false;
  CHECK_EQ((int)classifyCapability(ServiceCapability::WEB_READ_ONLY_DASHBOARD, in),
          (int)ServiceReadiness::BLOCKED);
  in.wifi_hardware_validated = true;
  CHECK_EQ((int)classifyCapability(ServiceCapability::WEB_READ_ONLY_DASHBOARD, in),
          (int)ServiceReadiness::TO_TEST);
}

void test_to_string_covers_every_value_and_fails_closed_on_corruption() {
  g_case = "to_string";
  CHECK_STR(toString(ServiceReadiness::READY), "READY");
  CHECK_STR(toString(ServiceReadiness::TO_TEST), "TO_TEST");
  CHECK_STR(toString(ServiceReadiness::BLOCKED), "BLOCKED");
  CHECK_STR(toString(static_cast<ServiceReadiness>(200)), "UNKNOWN");

  CHECK_STR(toString(ServiceCapability::ACTUATOR_TORQUE_ENABLE), "ACTUATOR_TORQUE_ENABLE");
  CHECK_STR(toString(ServiceCapability::ACTUATOR_POSITION_COMMAND), "ACTUATOR_POSITION_COMMAND");
  CHECK_STR(toString(ServiceCapability::CALIBRATION_CONTACT_PROBE), "CALIBRATION_CONTACT_PROBE");
  CHECK_STR(toString(ServiceCapability::CALIBRATION_AUXILIARY_MOVE), "CALIBRATION_AUXILIARY_MOVE");
  CHECK_STR(toString(ServiceCapability::CALIBRATION_DIRECTION_VERIFY),
           "CALIBRATION_DIRECTION_VERIFY");
  CHECK_STR(toString(ServiceCapability::WIFI_HARDWARE_ASSOCIATION), "WIFI_HARDWARE_ASSOCIATION");
  CHECK_STR(toString(ServiceCapability::OTA_END_TO_END), "OTA_END_TO_END");
  CHECK_STR(toString(ServiceCapability::WEB_READ_ONLY_DASHBOARD), "WEB_READ_ONLY_DASHBOARD");
  CHECK_STR(toString(static_cast<ServiceCapability>(200)), "UNKNOWN");
}

}  // namespace

int main() {
  test_actuator_and_calibration_capabilities_track_hardware_motion_authorized();
  test_direction_verify_is_always_to_test_never_blocked();
  test_wifi_and_ota_track_their_own_hardware_validation_flag_only();
  test_web_dashboard_is_blocked_until_wifi_hardware_validated();
  test_to_string_covers_every_value_and_fails_closed_on_corruption();

  std::printf("test_service_readiness: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
