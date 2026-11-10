#include "../../src/core/CommandRouter.h"
#include "../../src/network/HttpTransport.h"
// These methods are retained by the real router's complete dispatch function,
// but none should be called by PERSIST or RAM Q0 promotion. Any invocation is
// counted and fails the test. No persistence/parser/promotion method is faked.
extern "C" int64_t esp_timer_get_time() { return int64_t(router_test::now_ms) * 1000; }
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
bool ServoBus::readProfileRegister(int id, uint8_t address, uint8_t width, int32_t* out,
                                   diagnostics::ReadCapture* capture) {
  ++router_test::hardware_calls;
  auto* s = router_test::servo_read_script;
  if (s == nullptr) return false;
  s->reads += std::to_string(address) + "/" + std::to_string(width) + " ";
  if (capture != nullptr) { *capture = diagnostics::ReadCapture{}; capture->id = id; capture->address = address; capture->width = width; }
  if (!s->answers[address] || width != 1) {
    if (capture != nullptr) capture->validity = diagnostics::ReadValidity::NO_RESPONSE;
    return false;
  }
  if (capture != nullptr) {  // the status packet a servo returns: FF FF ID 03 00 value checksum
    const uint8_t frame[7] = {0xFF, 0xFF, uint8_t(id), 3, 0, s->values[address],
                              uint8_t(~(uint8_t(id) + 3 + s->values[address]))};
    for (unsigned i = 0; i < 7; ++i) capture->bytes[i] = frame[i];
    capture->size = 7; capture->validity = diagnostics::ReadValidity::VALID;
  }
  *out = s->values[address];
  return true;
}
// Same contract as the real primitive (allowlist, torque precondition,
// readback verdict, SAFE_OFF on a torque not read 0) over the scripted servo.
ServoBus::AccDiagWrite ServoBus::writeAccDiagnosticByte(int, uint8_t address, uint8_t value) {
  ++router_test::hardware_calls;
  AccDiagWrite r;
  auto* s = router_test::servo_read_script;
  const bool allowed = (address == kAccDiagAcc && value >= 1 && value <= 254) ||
                       (address == kAccDiagLock && value <= 1) ||
                       (address == kAccDiagMaxAcc && value >= 1 && value <= 254);
  if (s == nullptr || !allowed || !s->answers[40] || s->values[40] != 0) return r;
  r.refused = false;
  s->writes += std::to_string(address) + "=" + std::to_string(value) + " ";
  if (address == kAccDiagMaxAcc) {
    if (!(s->reject_85_locked && s->values[55] != 0)) s->values[85] = value;
    if (s->clobber_86_on_85) s->values[86] ^= 1;
  } else {
    s->values[address] = value;
  }
  if (address == kAccDiagAcc && s->clamp_from_85 && s->values[41] > s->values[85]) s->values[41] = s->values[85];
  if (s->arm_on_write) s->values[40] = 1;
  r.readback = s->values[address];
  r.torque_after = s->values[40];
  r.verify = r.readback == value ? ServoWriteVerifyResult::VERIFIED_APPLIED : ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
  if (r.torque_after != 0) { r.safe_off_issued = true; s->values[40] = 0; r.safe_off = SafeOffResult::VERIFIED_OFF; }
  return r;
}
bool ServoBus::readRuntimeState(int, RuntimeState* out) {
  ++router_test::hardware_calls;
  if (router_test::servo_read_script == nullptr) return false;
  *out = RuntimeState{2048, 0, 0, 115, 30, router_test::servo_read_script->torque, 0};
  return true;
}
SafeOffResult ServoBus::safeOff(int) { ++router_test::hardware_calls; return SafeOffResult::UNVERIFIED_NO_RESPONSE; }
bool ServoCensus::start() { ++router_test::hardware_calls; return false; }
bool ServoPreflight::start() { ++router_test::hardware_calls; return false; }
const char* toString(SafeOffResult) { return "UNRELATED_SAFE_OFF_RESULT"; }
const char* toString(JointPreflightResult) { return "UNRELATED_PREFLIGHT_RESULT"; }
}
namespace network {
bool WifiManager::setEnabled(bool, uint32_t) { ++router_test::hardware_calls; return false; }
bool WifiManager::requestScan() { ++router_test::hardware_calls;return false; }
bool WifiManager::requestAp(bool, uint32_t) { ++router_test::hardware_calls;return false; }
bool WifiManager::setSleep(uint8_t) { ++router_test::hardware_calls;return false; }
bool WifiManager::provisionAp(const char*) { ++router_test::hardware_calls;return false; }
bool WifiManager::provisionAdmin(const char*) { ++router_test::hardware_calls;return false; }
bool WifiManager::verifyAp(const char*) { ++router_test::hardware_calls;return false; }
bool WifiManager::configure(const ConfigPatch&, uint32_t) { ++router_test::hardware_calls;return false; }
bool NetworkConfigNvs::load(NetworkConfig*) { ++router_test::hardware_calls;return false; }
bool NetworkConfigNvs::pending(const NetworkConfig&) { ++router_test::hardware_calls;return false; }
bool NetworkConfigNvs::activate(const NetworkConfig&) { ++router_test::hardware_calls;return false; }
bool HttpTransport::start() { ++router_test::hardware_calls; return false; }
void HttpTransport::stop() { ++router_test::hardware_calls; }
}
}  // namespace matdog

// SOURCE_SIGNATURE reads a default cached OTA status. These platform methods
// supply its vtable without linking any flash implementation; any invocation
// is counted exactly like the other fake hardware above.
namespace matdog { namespace update {
OtaPartitionInfo OtaEspBackend::runningPartition() { ++router_test::hardware_calls; return {}; }
OtaPartitionInfo OtaEspBackend::nextUpdatePartition() { ++router_test::hardware_calls; return {}; }
OtaImgState OtaEspBackend::imageState(const OtaPartitionInfo&) { ++router_test::hardware_calls; return OtaImgState::UNREADABLE; }
bool OtaEspBackend::installedLayoutConforms() { ++router_test::hardware_calls; return false; }
bool OtaEspBackend::beginWrite(const OtaPartitionInfo&, uint32_t) { ++router_test::hardware_calls; return false; }
bool OtaEspBackend::write(const uint8_t*, uint32_t) { ++router_test::hardware_calls; return false; }
bool OtaEspBackend::endWrite() { ++router_test::hardware_calls; return false; }
void OtaEspBackend::abortWrite() { ++router_test::hardware_calls; }
bool OtaEspBackend::setBootPartition(const OtaPartitionInfo&) { ++router_test::hardware_calls; return false; }
bool OtaEspBackend::markAppValid() { ++router_test::hardware_calls; return false; }
bool OtaEspBackend::rollbackPossible() { ++router_test::hardware_calls; return false; }
} }
