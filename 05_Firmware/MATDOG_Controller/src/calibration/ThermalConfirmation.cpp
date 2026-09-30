#include "ThermalConfirmation.h"

namespace matdog {
namespace calibration {

ThermalConfirmation confirmPresentTemperature(ThermalReadPort* port, uint8_t bus_id,
                                              int32_t observed_c) {
  ThermalConfirmation out{};
  out.bus_id = bus_id;
  out.samples[0] = observed_c;
  out.sample_count = 1;
  out.published_c = observed_c;
  if (observed_c <= kThermalLimitC) return out;  // NORMAL: no confirmation read

  // V25 apply_matdog_direct_temperature(): two more direct reads of the same
  // servo, each after MATDOG_THERMAL_CONFIRMATION_DELAY.
  for (uint8_t i = 1; i < kThermalConfirmationReads; ++i) {
    int32_t celsius = -1;
    if (port == nullptr) {
      out.decision = ThermalDecision::CONFIRMATION_READ_FAILED;
      return out;  // published stays the over-limit trigger: the monitors abort
    }
    port->delayMs(kThermalConfirmationDelayMs);
    if (!port->readPresentTemperatureDirect(bus_id, &celsius) || celsius < 0) {
      out.decision = ThermalDecision::CONFIRMATION_READ_FAILED;
      return out;
    }
    out.samples[out.sample_count++] = celsius;
  }

  // V25 classify_matdog_direct_temperature_samples().
  uint8_t over_limit = 0;
  for (uint8_t i = 0; i < out.sample_count; ++i) {
    if (out.samples[i] > kThermalLimitC) ++over_limit;
  }
  if (over_limit >= kThermalConfirmedOverLimit) {
    // CONFIRMED: publish the highest over-limit value.
    int32_t hottest = observed_c;
    for (uint8_t i = 0; i < out.sample_count; ++i) {
      if (out.samples[i] > hottest) hottest = out.samples[i];
    }
    out.decision = ThermalDecision::CONFIRMED;
    out.published_c = hottest;
  } else if (over_limit == 0) {
    out.decision = ThermalDecision::NORMAL;
    out.published_c = out.samples[out.sample_count - 1];
  } else {
    // TRANSIENT: publish the last normal value (V25 searched from the end).
    out.decision = ThermalDecision::TRANSIENT;
    for (uint8_t i = out.sample_count; i > 0; --i) {
      if (out.samples[i - 1] <= kThermalLimitC) {
        out.published_c = out.samples[i - 1];
        break;
      }
    }
  }
  return out;
}

const char* toString(ThermalDecision decision) {
  switch (decision) {
    case ThermalDecision::NORMAL:                   return "NORMAL";
    case ThermalDecision::TRANSIENT:                return "TRANSIENT";
    case ThermalDecision::CONFIRMED:                return "CONFIRMED";
    case ThermalDecision::CONFIRMATION_READ_FAILED: return "CONFIRMATION_READ_FAILED";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
