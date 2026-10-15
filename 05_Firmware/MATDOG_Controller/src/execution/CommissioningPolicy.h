#ifndef MATDOG_EXECUTION_COMMISSIONING_POLICY_H
#define MATDOG_EXECUTION_COMMISSIONING_POLICY_H
#include "ReviewedMotionRuntime.h"
#include <stdint.h>
#include <stddef.h>
namespace matdog { namespace execution {
// Runtime test policy (RAM-only, selected over USB without a reflash).
// DIAGNOSTIC never actuates. FIXTURE_COMMISSIONING is a bounded supervised
// fixture policy. GROUND_VALIDATION / AUTONOMOUS stay on the existing reviewed
// card + SEAL path (@MOTION VALIDATION); selecting them opens nothing new.
// The protections below are NOT parameters: SAFE_OFF/STOP, watchdog and
// session revocation, Safe Actuator limits and authority, feedback freshness,
// bus exclusivity, deadman expiry and the compiled caps of every parameter.
// OPERATIONAL (2026-10-14): bound automatically at READY; same hard limits,
// route and live protections as FIXTURE_COMMISSIONING, but the entry is planned
// from the measured pose and the route runs continuously on an explicit command.
enum class RuntimeTestMode : uint8_t { DIAGNOSTIC, FIXTURE_COMMISSIONING, GROUND_VALIDATION, AUTONOMOUS, OPERATIONAL };
const char* toString(RuntimeTestMode mode);
bool parseRuntimeTestMode(const char* text, RuntimeTestMode* mode);
// Mode selection is a RAM scope restriction; it never supplies a permit.
bool runtimeModeAllowsScope(RuntimeTestMode mode, TrialScope scope);
enum class SegmentApproval : uint8_t { SEGMENT, PHASE, CONTINUOUS };
const char* toString(SegmentApproval approval);
struct CommissioningParameters {
  uint16_t inset_ticks=5;              // hard limit inset from gen-1 contacts
  // Folded-rest prime into the limit (LOWER only, inward). A LOWER resting AT
  // its folded stop sits inset_ticks past the hard limit; the prime target is
  // kPrimeInwardMarginTicks inside it. 12 = inset 5 + margin 2 + recorded
  // contact repeatability <= 3 + 2 ticks of rest settling (2026-10-15):
  // a robot lying on its stop must not depend on a 1-2 tick corridor.
  uint16_t prime_clamp_ticks=12;
  double max_velocity_rad_s=.15, max_acceleration_rad_s2=.6;
  double tracking_tolerance_rad=.006, max_step_rad=.006, sample_period_s=.05;
  double entry_min_stage_s=4, route_time_scale=2;
  double prone_max_deviation_rad=.17453292519943295, prone_max_rms_rad=.12217304763960307;
  uint32_t feedback_max_age_ms=400, batch_timeout_ms=1500, tracking_timeout_ms=4000;
  uint32_t segment_timeout_ms=20000, max_imu_age_ms=600;
  int max_temperature_c=60, min_voltage_dv=100, max_voltage_dv=130;
  uint16_t drive_speed=20; uint8_t drive_acceleration=5; uint8_t transaction_timeout_ms=8;
  SegmentApproval approval=SegmentApproval::SEGMENT;
  bool walk_enabled=false; double walk_clock=9;
};
// Compiled caps. A runtime value outside its cap is refused, never clamped.
struct CommissioningCaps {
  static constexpr uint16_t kMaxPrimeClampTicks=16;  // ~1.4 deg, inward only
  static constexpr double kMaxVelocityRadS=.35, kMaxAccelerationRadS2=2, kMaxTrackingRad=.010;
  static constexpr double kMaxStepRad=.01, kMinSamplePeriodS=.02, kMaxSamplePeriodS=.1;
  static constexpr double kMinStageS=1, kMaxStageS=60, kMinRouteScale=1, kMaxRouteScale=100;
  static constexpr double kMaxProneDeviationRad=.2617993877991494; // 15 deg recognition ceiling
  static constexpr uint32_t kMaxFeedbackAgeMs=1000, kMaxBatchMs=3000, kMaxTrackingMs=10000, kMaxSegmentMs=30000, kMaxImuAgeMs=3000;
  static constexpr int kMaxTemperatureC=70;
};
// Sets one parameter by name. Returns false with a reason; nothing changes.
bool setCommissioningParameter(CommissioningParameters& p, const char* key, const char* value, const char** reason);
bool validCommissioningParameters(const CommissioningParameters& p, const char** reason);
// Prints "key=value ..." into out. Returns false on truncation.
bool formatCommissioningParameters(const CommissioningParameters& p, char* out, size_t size);
// Builds the scoped runtime from validated parameters and twelve hard limits
// in radians [slot][lo,hi]. The fixture-limit route is the only route.
bool buildCommissioningRuntime(const CommissioningParameters& p, const double limits_rad[12][2],
                               uint32_t calibration_generation, uint32_t policy_epoch, ReviewedMotionRuntime* out);
} }
#endif
