#include "ThermalConfirmation.h"
namespace matdog { namespace calibration {
ThermalConfirmation ThermalConfirmationState::telemetryFault() {
  // Fail closed: never the block-read value, never a value at or under the limit.
  result_.decision = ThermalDecision::THERMAL_TELEMETRY_FAULT;
  result_.published_c = kThermalLimitC + 1;
  return result_;
}
ThermalConfirmation ThermalConfirmationState::update(ThermalReadPort* port, uint8_t bus,
                                                       int32_t observed, uint32_t now_ms) {
  if (!pending()) {
    result_ = ThermalConfirmation{};
    result_.bus_id = bus;
    result_.samples[0] = observed;
    result_.sample_count = 1;
    result_.published_c = observed;
    result_.bulk_artifacts = bulk_artifacts_;
    if (observed < 0 || observed > 255) {
      result_.decision = ThermalDecision::THERMAL_TELEMETRY_FAULT;
      result_.published_c = -1;
      return result_;
    }
    if (observed <= kThermalLimitC) return result_;
    // Suspect only: the verdict comes from the direct reads below.
    result_.decision = ThermalDecision::PENDING;
    last_read_ms_ = now_ms;
    pending_started_ms_ = now_ms;
    return result_;
  }
  if (expired(now_ms) || bus != result_.bus_id || observed < 0 || observed > 255) {
    return telemetryFault();
  }
  if (now_ms - last_read_ms_ < kThermalConfirmationDelayMs) return result_;
  int32_t value = -1;
  if (port == nullptr || !port->readPresentTemperatureDirect(bus, &value) ||
      value < 0 || value > 255) {
    return telemetryFault();
  }
  last_read_ms_ = now_ms;
  result_.samples[result_.sample_count++] = value;
  uint8_t hot = 0, cool = 0;
  int32_t hottest = 0;
  // samples[0] is the block read: it is never evidence of temperature.
  for (uint8_t i = 1; i < result_.sample_count; ++i) {
    if (result_.samples[i] > kThermalLimitC) ++hot; else ++cool;
    if (result_.samples[i] > hottest) hottest = result_.samples[i];
  }
  if (hot >= kThermalConfirmedOverLimit) {
    result_.decision = ThermalDecision::CONFIRMED;
    result_.published_c = hottest;
  } else if (cool >= kThermalDirectNormalToClear) {
    if (bulk_artifacts_ < 0xFFFF) ++bulk_artifacts_;
    result_.bulk_artifacts = bulk_artifacts_;
    result_.decision = ThermalDecision::BULK_ARTIFACT;
    result_.published_c = value;
  } else if (result_.sample_count == kThermalConfirmationReads) {
    return telemetryFault();  // direct samples stayed incoherent
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
    case ThermalDecision::BULK_ARTIFACT: return "BULK_TEMP_ARTIFACT_SUSPECT";
    case ThermalDecision::CONFIRMED: return "CONFIRMED";
    case ThermalDecision::THERMAL_TELEMETRY_FAULT: return "THERMAL_TELEMETRY_FAULT";
    case ThermalDecision::PENDING: return "PENDING";
  }
  return "UNKNOWN";
}
} }
