#ifndef MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H
#define MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../actuator/MotionDeadman.h"
#include "CalibrationDomain.h"
#include "CalibrationExecutionEngine.h"

// CR3 Priority 5 — the calibration endpoint search for the eight executable
// UPPER endpoints.
//
// Pure: <stdint.h> plus already-pure MATDOG units. No Arduino, no ServoBus,
// no clock of its own — every tick's "now" and every telemetry sample are
// supplied by the caller, the same contract as FirstMotionExecutor.h.
//
// WHY A STAGED SEARCH (hardware findings, 2026-09-29)
// ----------------------------------------------------
// The first design commanded ONE slow GoalPosition to the Geometry V5 contact
// and read a sustained position stall as contact. On LF_UPPER that failed
// three ways: the move could not finish in its budget; the servo's normal
// 4-5 tick tracking shortfall looked like a "stall" 4-5 ticks short of any
// goal; and the real MIN stop was found by hand ~23 ticks PAST the modelled
// contact, beyond the URDF limit. "Sustained no progress" alone is not
// contact evidence.
//
// WHAT THIS PORTS (LF V25 hardware oracle, matdog.rs — the only MATDOG
// calibrator ever validated on hardware; constants named after their V25
// originals below)
// ------------------------------------------------------------------------
//   stage           | V25 counterpart
//   COARSE_TRANSIT  | 64-tick target steps at GOAL_SPEED 160 / ACC 8, up to
//                   | the acceptance corridor entry (URDF limit - 64)
//   FINE_SEARCH     | approach_with_scout(FINE_STEP_TICKS=8) through the
//                   | corridor, never past the guard (URDF limit + 64)
//   BACKOFF         | backoff_and_verify(): 96 ticks back, current recovered
//   FINE_SEARCH #2  | second fine pass; a candidate lagging pass 1 by more than
//                   | 8 ticks is a friction plateau and is stepped past
//   repeatability   | repeatability_spread() <= 16
// Contact is decided by the V25 HybridContactDetector (ContactSearchDetector
// below): the commanded target must still be AHEAD by more than the settle
// tolerance, travel since the pass started >= 24 ticks, per-sample progress
// <= 2 ticks, |speed| <= 10, after 4 start-up samples, for 3 consecutive
// samples — and only inside the acceptance corridor; the same persistence
// outside it is an EARLY_STALL anomaly, never contact. A servo that settles a
// few ticks short of its target is "arrived" (<= 10 ticks) and the search
// simply takes the next 8-tick step; a real stop is where the target runs
// away from the joint.
//
// Deliberate differences from V25, each a narrowing:
//   - no coarse contact scout: the operator's staged spec stops coarse steps
//     at the corridor entry, so contact is only ever met by an 8-tick step;
//     pass 1 is therefore pass 2's depth reference (V25 used the scout);
//   - no kinematic-plateau recovery path after a settle window: a large
//     tracking error without a confirmed contact fails closed;
//   - telemetry is consumed at V25's 20 ms bus-poll cadence
//     (kSearchSampleIntervalMs) so its sample-count rules keep their meaning
//     at this Controller's faster tick rate;
//   - the V25 moving-current baseline is gathered during the first 64 ticks of
//     transit instead of a separate baseline move. As in V25 it is NOT a
//     contact criterion (V25 computed `_current_supports_contact` and never
//     used it): current only drives the hard abort and the post-backoff
//     recovery check. No absolute current threshold is invented here.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY
// ----------------------------------------------
// No ServoBus reference exists here, so this class cannot call safeOff() even
// by accident. Every terminal state that leaves torque possibly applied
// reports ContactProbePhase::SAFE_OFF_REQUIRED; the caller is responsible for
// the real, independent SAFE_OFF.

namespace matdog {
namespace calibration {

// --- LF V25 hardware-oracle constants (matdog.rs), ported unchanged ---------
constexpr uint16_t kSearchCoarseStepTicks = 64;                   // COARSE_STEP_TICKS
constexpr uint16_t kSearchFineStepTicks = 8;                      // FINE_STEP_TICKS
constexpr uint16_t kSearchBackoffTicks = 96;                      // BACKOFF_TICKS
constexpr uint16_t kSearchStaticToleranceTicks = 10;              // STATIC_TOLERANCE_TICKS
constexpr uint16_t kSearchOutsideCorridorSettleToleranceTicks = 16;  // OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS
constexpr uint16_t kSearchTrackingErrorFloorTicks = 16;           // PROBE_TRACKING_ERROR_FLOOR_TICKS
constexpr uint16_t kSearchMinContactTravelTicks = 24;             // MINIMUM_CONTACT_TRAVEL_TICKS
constexpr uint16_t kSearchMaxProgressTicks = 2;                   // HybridContactConfig::max_progress_ticks
constexpr uint16_t kSearchMaxVelocityRaw = 10;                    // HybridContactConfig::max_velocity_raw
constexpr uint8_t kSearchPersistenceSamples = 3;                  // HybridContactConfig::persistence_samples
constexpr uint8_t kSearchTargetStartupSamples = 4;                // TARGET_STARTUP_SAMPLES
constexpr uint32_t kSearchSettleWindowMs = 900;                   // CONTACT_SETTLE_WINDOW
constexpr uint32_t kSearchSampleIntervalMs = 20;                  // port.rs bus poll interval
constexpr uint32_t kSearchMaxTelemetryAgeMs = 3000;               // MAX_TELEMETRY_AGE
constexpr int32_t kSearchHardCurrentAbortRaw = 200;               // HARD_CURRENT_ABORT_RAW
constexpr int32_t kSearchTemperatureLimitC = 70;                  // EXPECTED_TEMPERATURE_LIMIT_C
constexpr uint16_t kSearchFineScoutLagToleranceTicks = 8;         // FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS
constexpr uint16_t kSearchAdaptiveScoutTicks = 32;                // ADAPTIVE_FINE_SCOUT_TICKS
constexpr uint16_t kSearchBaselineTravelTicks = 64;               // BASELINE_TRAVEL_TICKS
constexpr uint8_t kSearchBaselineMinSamples = 6;                  // BASELINE_MIN_SAMPLES
constexpr uint8_t kSearchBaselineMaxSamples = 32;                 // storage bound (ours)
// V25 sized every long move's deadline from a conservative half-speed floor
// of the GOAL_SPEED=160 envelope (MIN_EXPECTED_MOTION_TICKS_PER_SECOND).
constexpr uint16_t kSearchMinExpectedTicksPerSecond = 80;

struct ContactProbeRequest {
  JointIdentity joint{};
  uint8_t bus_id = 0;
  Leg endpoint_leg = Leg::LF;
  JointKind endpoint_joint = JointKind::HIP;
  ContactSide endpoint_side = ContactSide::MIN_SIDE;
  // Resolved by the caller with actuator::resolveCalibrationSearchCorridor()
  // for the same joint, side, geometry and promoted q0 the policy re-derives
  // on every command. Every target this engine issues lies inside it.
  actuator::CalibrationSearchCorridor corridor{};
  // The reviewed repeatability band - never defaulted (see ContactWitness).
  uint16_t repeatability_tolerance_ticks = 0;
  // The 24-contact sequence energizes the joint itself (V25 prepare_motor():
  // GoalPosition := present, RAM TorqueLimit, TorqueEnable - each verified)
  // BEFORE the probe starts, and the MAX side starts where the MIN side's
  // contact left the joint, still torque-on. True skips TORQUE_ENABLE_PENDING.
  bool start_torque_verified = false;
  // The RAM TorqueLimit every sample must read back (V25 checked it on every
  // observation). Never defaulted: 0 is a start() refusal.
  uint16_t expected_torque_limit = 0;
};

enum class ContactProbePhase : uint8_t {
  IDLE                  = 0,
  TORQUE_ENABLE_PENDING = 1,
  STEP_PENDING          = 2,  // write the next transit / search target
  STEP_MONITORING       = 3,  // settle window + contact detector for that target
  BACKOFF_PENDING       = 4,
  BACKOFF_MONITORING    = 5,
  // --- terminal states ----------------------------------------------------
  COMPLETE              = 6,  // two repeatable contacts; witness() is ready
  FAILED_NO_MOTION      = 7,  // ended before torque was ever VERIFIED applied
  SAFE_OFF_REQUIRED     = 8,  // torque was VERIFIED applied and the attempt did
                              // not reach a clean COMPLETE — caller MUST
                              // invoke the real, independent SAFE_OFF now
  // Both contacts accepted; GoalPosition := the pass-2 contact position is
  // written next (V25 stop_pressure()), so the joint rests ON the stop without
  // pressing into it, then COMPLETE. Not terminal.
  RELEASE_PENDING       = 9,
};

enum class ContactSearchStage : uint8_t {
  NONE           = 0,
  COARSE_TRANSIT = 1,
  FINE_SEARCH    = 2,
  BACKOFF        = 3,
};

enum class ContactProbeFailure : uint8_t {
  NONE                            = 0,
  REJECT_PRECONDITIONS            = 1,
  TORQUE_ENABLE_REJECTED          = 2,
  TORQUE_ENABLE_UNCERTAIN         = 3,
  COMMAND_REJECTED                = 4,  // policy/engine refused a step or the backoff
  COMMAND_UNCERTAIN               = 5,
  NO_CONTACT_BEFORE_GUARD         = 6,  // the next step would pass the guard
  EARLY_STALL_OUTSIDE_CORRIDOR    = 7,  // persistent stall before the corridor
  TRACKING_FAILED                 = 8,  // settle window ended far from target, no contact
  REPEATABILITY_FAILED            = 9,
  HARD_CURRENT_ABORT              = 10,
  OVER_TEMPERATURE                = 11,
  STALE_TELEMETRY                 = 12,
  COMMUNICATION_LOST              = 13,
  TORQUE_UNEXPECTEDLY_OFF         = 14,
  INSUFFICIENT_BASELINE           = 15,
  BACKOFF_CROSSES_HOME            = 16,
  CURRENT_NOT_RECOVERED           = 17,
  UNEXPECTED_STALL_DURING_BACKOFF = 18,
  MOTION_TIMEOUT                  = 19,
  OPERATOR_ABORT                  = 20,
  // The rest of V25's per-observation readback (see TelemetrySample).
  TORQUE_LIMIT_CHANGED            = 21,  // RAM TorqueLimit no longer the calibration value
  SERVO_STATUS_FAULT              = 22,  // the servo's own error flags are set
  GOAL_READBACK_MISMATCH          = 23,  // GoalPosition is not what this probe commanded
};

struct ContactProbeConfig {
  // Monitors the one long move of the search, the 96-tick backoff.
  actuator::MotionDeadmanConfig backoff_deadman{};
};

struct ContactProbeStatus {
  ContactProbePhase phase = ContactProbePhase::IDLE;
  ContactProbeFailure failure = ContactProbeFailure::NONE;
  ContactSearchStage stage = ContactSearchStage::NONE;
  uint8_t pass = 0;                   // 1, then 2 after the backoff
  uint16_t pass1_contact_tick = 0;    // accepted contacts, 0 = not yet
  uint16_t pass2_contact_tick = 0;
  uint16_t target_tick = 0;           // last commanded step / backoff target
  uint16_t step_count = 0;            // step writes this attempt (both passes)
  uint16_t plateau_bypass_count = 0;  // pass-2 candidates stepped past (V25)
  uint16_t last_candidate_tick = 0;   // last detector-confirmed candidate
  int32_t last_position = -1;         // last good telemetry sample
  int32_t last_speed = -1;            // raw magnitude
  int32_t last_current = -1;          // raw magnitude
  uint8_t baseline_samples = 0;
  uint16_t baseline_median_current = 0;
  uint16_t baseline_mad_current = 0;
  actuator::WriteDecision last_policy_decision = actuator::WriteDecision::REJECT_NO_ARBITER;
};

// Mirrors FirstMotionExecutor's context shape exactly.
struct ContactProbeContext {
  bool session_active = false;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
};

enum class ContactDetectorState : uint8_t {
  FREE_MOTION       = 0,
  CONTACT_SUSPECTED = 1,
  CONTACT_CONFIRMED = 2,  // persistent, inside the acceptance corridor
  EARLY_STALL       = 3,  // persistent, outside it: an anomaly, never contact
};

// The LF V25 HybridContactDetector::observe(), ported rule for rule (its
// hard-abort branch lives in the engine's per-sample safety checks). Pure;
// fed one cadence-gated sample at a time.
class ContactSearchDetector {
 public:
  // `acceptance_entry_depth` is the corridor entry depth, or (pass 2) the
  // V25 adaptive bound min(entry, pass-1 contact - 32) — HOME-ward only.
  void begin(const actuator::CalibrationSearchCorridor& corridor, uint16_t start_position,
             int32_t acceptance_entry_depth);
  ContactDetectorState observe(uint16_t position, int32_t speed_magnitude,
                               uint16_t commanded_target);

 private:
  actuator::CalibrationSearchCorridor corridor_{};
  uint16_t start_position_ = 0;
  uint16_t previous_position_ = 0;
  int32_t acceptance_entry_depth_ = 0;
  int32_t active_target_ = -1;
  uint8_t target_samples_seen_ = 0;
  uint8_t confirming_samples_ = 0;
};

// median + max(4 * MAD, 5) — V25 BaselineStats::contact_threshold().
uint16_t searchBaselineThreshold(uint16_t median_current, uint16_t mad_current);

class ContactProbeEngine {
 public:
  // No pointer is owned; all may be nullptr, which is a refusal, never a
  // permission. `engine` drives every step and the backoff (reusing
  // CalibrationExecutionEngine's CONTACT_PROBE endpoint-plan/parking checks
  // and the policy's search-corridor bound in full); `policy`/`runtime` are
  // used directly only for TORQUE_ENABLE — the same split
  // FirstMotionExecutor.h uses, for the same reason.
  void begin(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
            CalibrationExecutionEngine* engine,
            const actuator::CalibrationGeometryProfile* geometry,
            const actuator::GeometryProvenance* expected_provenance,
            const ContactProbeConfig& config);

  bool start(const ContactProbeRequest& request, const ContactProbeContext& context,
            uint32_t now_ms);

  // At most one backend call per tick. `telemetry` is the probed joint's
  // sample for this tick; it is consumed at kSearchSampleIntervalMs cadence.
  void update(const ContactProbeContext& context, uint32_t now_ms, bool telemetry_available,
             const actuator::TelemetrySample& telemetry);

  void abort();

  const ContactProbeStatus& status() const { return status_; }
  const ContactProbeRequest& request() const { return request_; }
  bool active() const {
    return status_.phase != ContactProbePhase::IDLE &&
          status_.phase != ContactProbePhase::COMPLETE &&
          status_.phase != ContactProbePhase::FAILED_NO_MOTION &&
          status_.phase != ContactProbePhase::SAFE_OFF_REQUIRED;
  }

  // Valid only once status().phase == COMPLETE. Provenance (key/origin) is
  // the caller's to stamp.
  ContactWitness witness() const;

 private:
  void finish(ContactProbePhase phase, ContactProbeFailure failure,
             actuator::WriteDecision decision);
  void failSafeOff(ContactProbeFailure failure) {
    finish(ContactProbePhase::SAFE_OFF_REQUIRED, failure, status_.last_policy_decision);
  }
  void stepTorqueEnable(const ContactProbeContext& context);
  void stepWrite(const ContactProbeContext& context, uint32_t now_ms);
  void stepMonitor(uint32_t now_ms, bool telemetry_available,
                  const actuator::TelemetrySample& telemetry);
  void stepBackoffWrite(const ContactProbeContext& context, uint32_t now_ms);
  void stepBackoffMonitor(uint32_t now_ms, bool telemetry_available,
                         const actuator::TelemetrySample& telemetry);
  void stepRelease(const ContactProbeContext& context);
  // Records a usable sample. False if it cannot be used at all.
  bool acceptSample(uint32_t now_ms, const actuator::TelemetrySample& telemetry);
  // Hard-current / thermal / torque checks on an accepted sample. False
  // (and the attempt failed) on any violation.
  bool sampleSafe(const actuator::TelemetrySample& telemetry);
  void onCandidate(uint16_t position);
  void beginPass(uint8_t pass, uint16_t start_position);
  bool issueTarget(const ContactProbeContext& context, uint16_t target_tick,
                   actuator::ExecuteResult* result_out);
  int32_t depth(uint16_t tick) const { return actuator::searchDepth(request_.corridor, tick); }
  uint16_t tickAtDepth(int32_t d) const;

  actuator::SafeActuatorPolicy* policy_ = nullptr;
  actuator::ActuatorRuntime* runtime_ = nullptr;
  CalibrationExecutionEngine* engine_ = nullptr;
  const actuator::CalibrationGeometryProfile* geometry_ = nullptr;
  const actuator::GeometryProvenance* expected_provenance_ = nullptr;
  ContactProbeConfig config_{};

  ContactProbeRequest request_{};
  ContactProbeStatus status_{};
  ContactSearchDetector detector_{};
  actuator::MotionDeadmanMonitor deadman_;

  bool has_good_sample_ = false;
  uint32_t last_good_ms_ = 0;
  uint32_t started_ms_ = 0;
  bool has_cadence_sample_ = false;
  uint32_t last_cadence_ms_ = 0;
  int32_t last_cadence_position_ = -1;
  uint32_t step_written_ms_ = 0;
  uint16_t step_ticks_ = 0;
  uint16_t pass_start_position_ = 0;
  bool pass_started_ = false;
  bool baseline_closed_ = false;
  uint16_t baseline_[kSearchBaselineMaxSamples] = {0};
};

const char* toString(ContactProbePhase phase);
const char* toString(ContactProbeFailure failure);
const char* toString(ContactSearchStage stage);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H
