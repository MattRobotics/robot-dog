#include "../../src/core/CommandRouter.h"
#include "../../src/network/HttpTransport.h"
// These methods are retained by the real router's complete dispatch function,
// but none should be called by PERSIST or RAM Q0 promotion. Any invocation is
// counted and fails the test. No persistence/parser/promotion method is faked.
namespace matdog {
namespace imu {
core::AvailabilityStatus Bno085Imu::availability() const { ++router_test::hardware_calls; return {}; }
}
namespace power {
core::AvailabilityStatus DalyBms::availability() const { ++router_test::hardware_calls; return {}; }
bool DalyBms::requestKeyConfigRead() { ++router_test::hardware_calls; return false; }
DalyKeyWriteGate DalyBms::requestKeyLogicDischarge(core::OperatingMode) { ++router_test::hardware_calls; return {}; }
}
namespace status {
core::AvailabilityStatus LedRing::availability() const { ++router_test::hardware_calls; return {}; }
void LedRing::off() { ++router_test::hardware_calls; }
bool LedRing::startTest() { ++router_test::hardware_calls; return false; }
bool LedRing::startSocTest() { ++router_test::hardware_calls; return false; }
const char* toString(LedDiagnostic) { return "UNRELATED_LED_DIAGNOSTIC"; }
}
namespace servo {
core::AvailabilityStatus ServoBus::availability() const { ++router_test::hardware_calls; return {}; }
bool ServoBus::startScan(int, int) { ++router_test::hardware_calls; return false; }
bool ServoBus::readRuntimeState(int, RuntimeState*) { ++router_test::hardware_calls; return false; }
SafeOffResult ServoBus::safeOff(int) { ++router_test::hardware_calls; return SafeOffResult::UNVERIFIED_NO_RESPONSE; }
bool ServoCensus::start() { ++router_test::hardware_calls; return false; }
bool ServoPreflight::start() { ++router_test::hardware_calls; return false; }
const char* toString(SafeOffResult) { return "UNRELATED_SAFE_OFF_RESULT"; }
const char* toString(JointPreflightResult) { return "UNRELATED_PREFLIGHT_RESULT"; }
}
namespace network {
bool WifiManager::setEnabled(bool, uint32_t) { ++router_test::hardware_calls; return false; }
bool HttpTransport::start() { ++router_test::hardware_calls; return false; }
void HttpTransport::stop() { ++router_test::hardware_calls; }
}
}  // namespace matdog
