#ifndef MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H
#define MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H

#include <stdint.h>

// Runtime PresentTemperature over-limit confirmation, ported from the LF V25
// hardware oracle (NormaCore st3215 driver, port.rs):
//
//   MATDOG_THERMAL_CONFIRMATION_READS = 3, MATDOG_THERMAL_CONFIRMATION_DELAY =
//   50 ms, apply_matdog_direct_temperature(), read_motor_temperature_direct(),
//   classify_matdog_direct_temperature_samples().
//
// An observation reading PresentTemperature > 70 C is NOT yet a thermal abort:
// the same servo is read directly twice more, each read preceded by 50 ms.
// Of the three values, >= 2 over the limit is CONFIRMED (the monitors see the
// highest over-limit value and abort); exactly 1 is a TRANSIENT (they see the
// last normal value and continue). A confirmation read that does not answer
// fails closed: the monitors keep seeing the over-limit trigger and abort, as
// before this port (V25 dropped the observation and stopped its bus worker).
//
// This is ONLY the runtime PresentTemperature classification. The persistent
// MaxTemperature register (EEPROM 0x0D = 70) is verified by the servo-profile
// preflight and is not touched here. Nothing else - communication, status,
// TorqueEnable, TorqueLimit, GoalPosition, current, telemetry age, held
// joints, guards, authority, permit - is confirmed or debounced.
//
// Pure: <stdint.h> only; the bus access and the wait are behind
// ThermalReadPort so the rule is host-tested exactly as the Controller runs it.

namespace matdog {
namespace calibration {

constexpr int32_t kThermalLimitC = 70;                // MATDOG_EXPECTED_TEMPERATURE_LIMIT_C
constexpr uint8_t kThermalConfirmationReads = 3;      // MATDOG_THERMAL_CONFIRMATION_READS
constexpr uint32_t kThermalConfirmationDelayMs = 50;  // MATDOG_THERMAL_CONFIRMATION_DELAY
constexpr uint8_t kThermalConfirmedOverLimit = 2;     // classify_...: over_limit >= 2

enum class ThermalDecision : uint8_t {
  NORMAL                   = 0,  // <= 70 C: no confirmation read was made
  TRANSIENT                = 1,  // exactly one of three over the limit: continue
  CONFIRMED                = 2,  // >= 2 of three over the limit: abort
  CONFIRMATION_READ_FAILED = 3,  // a direct read did not answer: abort (fail closed)
};

class ThermalReadPort {
 public:
  virtual ~ThermalReadPort() = default;
  // One FRESH, DIRECT PresentTemperature read of exactly this servo (never
  // cached bulk telemetry). False: the servo did not answer.
  virtual bool readPresentTemperatureDirect(uint8_t bus_id, int32_t* celsius) = 0;
  virtual void delayMs(uint32_t ms) = 0;
};

struct ThermalConfirmation {
  ThermalDecision decision = ThermalDecision::NORMAL;
  uint8_t bus_id = 0;
  uint8_t sample_count = 0;  // the triggering observation + the direct reads made
  int32_t samples[kThermalConfirmationReads] = {0, 0, 0};
  int32_t published_c = 0;   // the temperature the calibration monitors are given
};

// `observed_c` is the PresentTemperature of the normal observation of
// `bus_id`. At or below the limit (or unread, < 0) it is returned unchanged
// and the port is not touched.
ThermalConfirmation confirmPresentTemperature(ThermalReadPort* port, uint8_t bus_id,
                                              int32_t observed_c);

const char* toString(ThermalDecision decision);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H
