#include "ServoBus.h"

#include "../config/BuildConfig.h"
#include "../config/Pins.h"

namespace matdog {
namespace servo {

bool ServoBus::begin() {
  servo_uart_.begin(
      build::kServoBusBaud,
      SERIAL_8N1,
      pins::kServoRx,
      pins::kServoTx);

  st_.pSerial = &servo_uart_;

  // No automatic ping/scan/torque on boot — matches the frozen bench
  // source's own stated invariant ("Automatic ping : DISABLED") and the
  // handoff's "no automatic motion at boot" rule.
  health_ = core::ModuleHealth::OK;
  return true;
}

bool ServoBus::ping(int id) {
  if (id < 0 || id > 253) return false;
  int result = st_.Ping(static_cast<uint8_t>(id));
  return result >= 0;
}

bool ServoBus::readModel(int id, int* model_out) {
  if (id < 0 || id > 253 || model_out == nullptr) return false;
  int model = st_.readWord(static_cast<uint8_t>(id), SMS_STS_MODEL_L);
  if (model < 0) return false;
  *model_out = model;
  return true;
}

ScanResult ServoBus::scan(int lo, int hi) {
  ScanResult result;

  if (lo > hi) {
    int tmp = lo;
    lo = hi;
    hi = tmp;
  }
  lo = constrain(lo, 0, 253);
  hi = constrain(hi, 0, 253);
  if (hi - lo + 1 > kMaxScanRange) {
    hi = lo + kMaxScanRange - 1;
  }

  result.lo = lo;
  result.hi = hi;

  for (int id = lo; id <= hi; ++id) {
    if (st_.Ping(static_cast<uint8_t>(id)) >= 0) {
      if (result.found_count < kMaxScanIds) {
        result.found_ids[result.found_count] = id;
      }
      result.found_count++;
    }
  }

  return result;
}

bool ServoBus::safeOff(int id) {
  if (id < 0 || id > 253) return false;
  int result = st_.EnableTorque(static_cast<uint8_t>(id), 0);
  return result >= 0;
}

bool ServoBus::readRuntimeState(int id, RuntimeState* out) {
  if (id < 0 || id > 253 || out == nullptr) return false;

  int ping = st_.Ping(static_cast<uint8_t>(id));
  if (ping < 0) return false;

  out->present_position    = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_POSITION_L);
  out->present_speed       = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_SPEED_L);
  out->present_load        = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_LOAD_L);
  out->present_voltage     = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_VOLTAGE);
  out->present_temperature = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_TEMPERATURE);
  out->torque_enable       = st_.readByte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE);

  return true;
}

}  // namespace servo
}  // namespace matdog
