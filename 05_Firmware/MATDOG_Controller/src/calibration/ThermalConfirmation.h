#ifndef MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H
#define MATDOG_CALIBRATION_THERMAL_CONFIRMATION_H
#include <stdint.h>
namespace matdog { namespace calibration {
constexpr int32_t kThermalLimitC = 70;
constexpr uint8_t kThermalConfirmationReads = 5;
constexpr uint8_t kThermalConfirmedOverLimit = 3;
constexpr uint32_t kThermalConfirmationDelayMs = 50;
// Direct samples at or under the limit that refute a suspect block-read value.
constexpr uint8_t kThermalDirectNormalToClear = 3;
// The 15-byte feedback block read (registers 56..70) of a MOVING servo
// intermittently carries a PresentTemperature far above the true value
// (hardware evidence 2026-10-01/03/06). That byte is therefore diagnostic
// telemetry only: over the limit it opens a confirmation and nothing else.
// The thermal verdict is taken from DIRECT single-register reads alone:
//   - kThermalConfirmedOverLimit direct samples over the unchanged limit
//     -> CONFIRMED (over-temperature);
//   - kThermalDirectNormalToClear direct samples at or under it
//     -> BULK_ARTIFACT, counted for diagnostics and never latched;
//   - a direct read that fails or is invalid, direct samples that stay
//     incoherent, or an expired confirmation -> THERMAL_TELEMETRY_FAULT.
// Both CONFIRMED and THERMAL_TELEMETRY_FAULT publish a value over the limit:
// the run aborts to SAFE_OFF.
enum class ThermalDecision : uint8_t {
  NORMAL, BULK_ARTIFACT, CONFIRMED, THERMAL_TELEMETRY_FAULT, PENDING
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
  int32_t samples[kThermalConfirmationReads] = {0};  // [0] block read, [1..] direct
  int32_t published_c = -1;
  uint16_t bulk_artifacts = 0;  // refuted block-read values so far, diagnostic
};
// Per-servo runtime state; no sleep in update(). A pending verdict pauses
// sequence advancement while fresh current/position/held-role checks continue.
// A refuted block-read value never accumulates into a safety verdict.
class ThermalConfirmationState {
 public:
  ThermalConfirmation update(ThermalReadPort* port, uint8_t bus, int32_t observed,
                             uint32_t now_ms);
  const ThermalConfirmation& result() const { return result_; }
  bool expired(uint32_t now_ms) const { return pending() && now_ms - pending_started_ms_ >= 300; }
  bool directReadDue(uint32_t now_ms) const { return pending() && now_ms - last_read_ms_ >= kThermalConfirmationDelayMs; }
  bool pending() const { return result_.decision == ThermalDecision::PENDING; }
  uint16_t bulkArtifacts() const { return bulk_artifacts_; }
 private:
  ThermalConfirmation telemetryFault();
  ThermalConfirmation result_{};
  uint32_t last_read_ms_ = 0;
  uint32_t pending_started_ms_ = 0;
  uint16_t bulk_artifacts_ = 0;
};
// Blocking host compatibility adapter. Production uses the state above.
ThermalConfirmation confirmPresentTemperature(ThermalReadPort* port, uint8_t bus_id,
                                              int32_t observed_c);
const char* toString(ThermalDecision decision);
} }
#endif
