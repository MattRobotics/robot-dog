#ifndef MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H
#define MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H
#include <stdint.h>
namespace matdog { namespace calibration {
constexpr int32_t kThermalLimitC = 70;
constexpr uint8_t kThermalConfirmationReads = 5;
constexpr uint8_t kThermalConfirmedOverLimit = 3;
constexpr uint32_t kThermalConfirmationDelayMs = 50;
constexpr uint32_t kThermalAnomalyWindowMs = 30000;
constexpr uint8_t kThermalMaxTransients = 3;
constexpr uint8_t kThermalMaxBootTransients = 8;
enum class ThermalDecision : uint8_t {
  NORMAL, TRANSIENT, CONFIRMED, CONFIRMATION_READ_FAILED, PENDING, REPEATED_ANOMALY
};
class ThermalReadPort {
 public:
  virtual ~ThermalReadPort() = default;
  virtual bool readPresentTemperatureDirect(uint8_t bus_id, int32_t* celsius) = 0;
  virtual void delayMs(uint32_t ms) = 0;
};
struct ThermalConfirmation {
  ThermalDecision decision = ThermalDecision::NORMAL;
  uint8_t bus_id = 0;
  uint8_t sample_count = 0;
  int32_t samples[kThermalConfirmationReads] = {0};
  int32_t published_c = -1;
};
// Per-servo runtime state; no sleep in update(). A pending verdict pauses
// sequence advancement while fresh current/position/held-role checks continue.
// Three incidents in 30 s, or eight per boot, latch fail-closed until reboot.
class ThermalConfirmationState {
 public:
  ThermalConfirmation update(ThermalReadPort* port, uint8_t bus, int32_t observed,
                             uint32_t now_ms);
  const ThermalConfirmation& result() const { return result_; }
  bool expired(uint32_t now_ms) const { return pending() && now_ms - pending_started_ms_ >= 300; }
  bool directReadDue(uint32_t now_ms) const { return pending() && now_ms - last_read_ms_ >= kThermalConfirmationDelayMs; }
  bool pending() const { return result_.decision == ThermalDecision::PENDING; }
 private:
  ThermalConfirmation result_{};
  uint32_t last_read_ms_ = 0;
  uint32_t pending_started_ms_ = 0;
  uint32_t window_start_ms_ = 0;
  uint8_t transients_ = 0;
  uint8_t boot_transients_ = 0;
  bool latched_ = false;
};
// Blocking host compatibility adapter. Production uses the state above.
ThermalConfirmation confirmPresentTemperature(ThermalReadPort* port, uint8_t bus_id,
                                              int32_t observed_c);
const char* toString(ThermalDecision decision);
} }
#endif
