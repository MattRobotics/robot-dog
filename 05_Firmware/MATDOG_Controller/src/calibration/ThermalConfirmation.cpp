#include "ThermalConfirmation.h"
namespace matdog { namespace calibration {
ThermalConfirmation ThermalConfirmationState::update(ThermalReadPort* port, uint8_t bus,
                                                       int32_t observed, uint32_t now_ms) {
  if (latched_) {
    result_.decision = ThermalDecision::REPEATED_ANOMALY;
    result_.published_c = kThermalLimitC + 1;
    return result_;
  }
  if (!pending()) {
    result_ = ThermalConfirmation{};
    result_.bus_id = bus;
    result_.samples[0] = observed;
    result_.sample_count = 1;
    result_.published_c = observed;
    if (observed < 0 || observed > 255) {
      result_.decision = ThermalDecision::CONFIRMATION_READ_FAILED;
      result_.published_c = -1;
      return result_;
    }
    if (observed <= kThermalLimitC) return result_;
    result_.decision = ThermalDecision::PENDING;
    last_read_ms_ = now_ms;
    pending_started_ms_ = now_ms;
    return result_;
  }
  if (expired(now_ms) || bus != result_.bus_id || observed < 0 || observed > 255) {
    result_.decision = ThermalDecision::CONFIRMATION_READ_FAILED;
    return result_;
  }
  if (now_ms - last_read_ms_ < kThermalConfirmationDelayMs) return result_;
  int32_t value = -1;
  if (port == nullptr || !port->readPresentTemperatureDirect(bus, &value) ||
      value < 0 || value > 255) {
    result_.decision = ThermalDecision::CONFIRMATION_READ_FAILED;
    return result_;
  }
  last_read_ms_ = now_ms;
  result_.samples[result_.sample_count++] = value;
  uint8_t hot = 0, cool = 0;
  int32_t hottest = 0;
  for (uint8_t i = 0; i < result_.sample_count; ++i) {
    if (result_.samples[i] > kThermalLimitC) ++hot; else ++cool;
    if (result_.samples[i] > hottest) hottest = result_.samples[i];
  }
  if (hot >= kThermalConfirmedOverLimit) {
    result_.decision = ThermalDecision::CONFIRMED;
    result_.published_c = hottest;
  } else if (cool >= 3) {
    // Resolve only with a normal final direct sample. Mixed tails stay closed.
    if (value > kThermalLimitC) {
      result_.decision = ThermalDecision::CONFIRMATION_READ_FAILED;
      return result_;
    }
    if (transients_ == 0 || now_ms - window_start_ms_ >= kThermalAnomalyWindowMs) {
      window_start_ms_ = now_ms;
      transients_ = 0;
    }
    ++transients_;
    ++boot_transients_;
    latched_ = transients_ >= kThermalMaxTransients || boot_transients_ >= kThermalMaxBootTransients;
    result_.decision = latched_ ? ThermalDecision::REPEATED_ANOMALY : ThermalDecision::TRANSIENT;
    result_.published_c = latched_ ? kThermalLimitC + 1 : value;
  }
  return result_;
}
ThermalConfirmation confirmPresentTemperature(ThermalReadPort* port, uint8_t bus,
                                              int32_t observed) {
  ThermalConfirmationState state;
  uint32_t now = 0;
  ThermalConfirmation result = state.update(port, bus, observed, now);
  while (state.pending()) {
    if (port != nullptr) port->delayMs(kThermalConfirmationDelayMs);
    now += kThermalConfirmationDelayMs;
    result = state.update(port, bus, observed, now);
  }
  return result;
}
const char* toString(ThermalDecision d) {
  switch (d) {
    case ThermalDecision::NORMAL: return "NORMAL";
    case ThermalDecision::TRANSIENT: return "TRANSIENT";
    case ThermalDecision::CONFIRMED: return "CONFIRMED";
    case ThermalDecision::CONFIRMATION_READ_FAILED: return "CONFIRMATION_READ_FAILED";
    case ThermalDecision::PENDING: return "PENDING";
    case ThermalDecision::REPEATED_ANOMALY: return "REPEATED_ANOMALY";
  }
  return "UNKNOWN";
}
} }
