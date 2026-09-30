#ifndef MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H
#define MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../actuator/MotionDeadman.h"
#include "CalibrationDomain.h"
#include "CalibrationExecutionEngine.h"

// The calibration endpoint search of every one of the 24 Full Calibration
// endpoints (4 legs x HIP/UPPER/LOWER x MIN/MAX): one generic implementation.
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
// One endpoint search is V25 measure_lf_contact_side_efficient(), stage for
// stage (the same sequence as V25's generic run()):
//   stage           | V25 counterpart
//   BASELINE        | acquire_moving_current_baseline_forward(): ONE 64-tick
//                   | move from where the joint is, recording the moving
//                   | current (>= 6 samples; MOTION_TIMEOUT 12 s)
//   COARSE_TRANSIT/ | approach_with_scout(COARSE_STEP_TICKS=64, scout=None):
//   COARSE_SCOUT    | the coarse CONTACT SCOUT. 64-tick target steps from the
//                   | baseline end, one detector for the whole pass, the static
//                   | acceptance corridor [URDF limit - 64, guard]. A step whose
//                   | target is still short of the corridor entry is labelled
//                   | COARSE_TRANSIT (free-space travel: a stall there can only
//                   | be an EARLY_STALL, never contact); from the entry on,
//                   | COARSE_SCOUT. V25 has no separate transit move: the
//                   | scout pass IS the travel, so none is added here.
//   RELEASE         | stop_pressure(): GoalPosition := the accepted position,
//                   | verified by readback, after EVERY accepted approach
//   BACKOFF         | backoff_and_verify(scout): 96 ticks back; arrival is V25's
//                   | StableTargetGate (move_motor_to in the LF session): <= 12
//                   | ticks, |speed| <= 4, 4 consecutive samples, >= 400 ms -
//                   | only then is the current-recovery check made and the next
//                   | fine pass allowed to start
//   FINE_SEARCH #1  | approach_with_scout(FINE_STEP_TICKS=8, Some(scout)) from
//                   | the backoff pose
//   BACKOFF         | backoff_and_verify(fine 1)
//   FINE_SEARCH #2  | approach_with_scout(FINE_STEP_TICKS=8, Some(scout)) - the
//                   | same mechanics, the same scout reference
//   repeatability   | repeatability_spread(fine 1, fine 2) <= 16
// The scout tick is REFERENCE EVIDENCE ONLY (V25 logged it "discarded"): it
// shapes both fine passes and is recorded, but metrology, repeatability and
// every derived figure come from the two fine passes alone.
//
// Both fine passes use the scout exactly as V25 did:
//   - adaptive_contact_acceptance_bounds(): the corridor is extended HOME-ward
//     to scout - 32 (ADAPTIVE_FINE_SCOUT_TICKS), never toward the guard;
//   - fine_contact_reproduces_coarse_depth(): a confirmed candidate lagging
//     the scout by more than one fine step (FINE_CONTACT_SCOUT_LAG_TOLERANCE_
//     TICKS = 8) is a friction/chamfer plateau and is stepped past; one at or
//     beyond the scout depth is the contact;
//   - confirm_kinematic_plateau(): when a settle window ends further than
//     max(step + 4, 16) from its target without a confirmed contact, three
//     more samples inside the adaptive corridor, with the target ahead,
//     |speed| <= 10, within 32 ticks of the scout and spanning <= 3 ticks are
//     a kinematic contact (then the same lag rule). Anything else fails
//     closed. The coarse scout itself has no such path (V25: scout = None).
// Contact is decided by the V25 HybridContactDetector (ContactSearchDetector
// below): the commanded target must still be AHEAD by more than the settle
// tolerance, travel since the pass started >= 24 ticks, per-sample progress
// <= 2 ticks, |speed| <= 10, after 4 start-up samples, for 3 consecutive
// samples — and only inside the acceptance corridor; the same persistence
// outside it is an EARLY_STALL anomaly, never contact. A servo that settles a
// few ticks short of its target is "arrived" (<= 10 ticks) and the search
// simply takes the next step; a real stop is where the target runs away from
// the joint. The scout's 64-tick step can leave the target up to one coarse
// step past a stop (V25 behaviour): TorqueLimit 500, the 200-raw hard-current
// abort and the guard bound that pressure.
//
// Differences from V25 (the traceability log lists them with reasons):
//   - every V25 error return (early stall, tracking failed, travel guard,
//     repeatability, ...) is SAFE_OFF_REQUIRED here: the caller's verified
//     torque-off instead of V25's GoalPosition := present then an error;
//   - the contact detector consumes telemetry at V25's 20 ms bus-poll cadence
//     (kSearchSampleIntervalMs) so its sample-count rules keep their meaning
//     at this Controller's faster tick rate; the backoff settle gate counts
//     every sample (V25 counted every observation), its 400 ms bound applies;
//   - the backoff deadline is the deadman's 12 s + travel at 80 ticks/s
//     (13.2 s for 96 ticks; V25: max(12 s, travel + 5 s) = 12 s);
//   - at most kSearchBaselineMaxSamples moving-current samples are kept (V25:
//     an unbounded Vec). As in V25 the baseline is NOT a contact criterion
//     (V25 computed `_current_supports_contact` and never used it): current
//     only drives the hard abort and the post-backoff recovery check. No
//     absolute current threshold is invented here;
//   - the servo's configured temperature-limit register is verified by the
//     persistent-profile preflight, not re-read on every sample (V25 read it
//     with every observation); the present temperature is checked on every
//     sample against the same 70 C.
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
// V25 TELEMETRY_TIMEOUT: no new usable sample for this long ends the search
// (V25 waited at most this long for each new observation).
constexpr uint32_t kSearchTelemetryTimeoutMs = 2000;              // TELEMETRY_TIMEOUT
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
constexpr uint32_t kSearchBaselineTimeoutMs = 12000;              // MOTION_TIMEOUT
constexpr uint8_t kSearchKinematicPlateauSamples = 3;             // KINEMATIC_PLATEAU_SAMPLES
constexpr uint16_t kSearchKinematicPlateauSpanTicks = 3;          // KINEMATIC_PLATEAU_POSITION_SPAN_TICKS
// V25 StableTargetGate as backoff_and_verify() -> move_motor_to() applied it
// in the persistent LF session (tolerance STATIC_TOLERANCE_TICKS + 2).
constexpr uint16_t kSearchBackoffSettleToleranceTicks = 12;       // STATIC_TOLERANCE_TICKS + 2
constexpr int32_t kSearchBackoffSettleMaxSpeedRaw = 4;            // LF_HELD_MAX_SPEED_RAW
constexpr uint8_t kSearchBackoffSettledSamples = 4;               // LF_TRANSITION_SETTLED_SAMPLES
constexpr uint32_t kSearchBackoffSettleWindowMs = 400;            // LF_TRANSITION_SETTLE_WINDOW

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
  STEP_PENDING          = 2,  // write the next coarse-scout / fine-search target
  STEP_MONITORING       = 3,  // settle window + contact detector for that target
  BACKOFF_PENDING       = 4,
  BACKOFF_MONITORING    = 5,
  // --- terminal states ----------------------------------------------------
  COMPLETE              = 6,  // two repeatable contacts; witness() is ready
  FAILED_NO_MOTION      = 7,  // ended before torque was ever VERIFIED applied
  SAFE_OFF_REQUIRED     = 8,  // torque was VERIFIED applied and the attempt did
                              // not reach a clean COMPLETE — caller MUST
                              // invoke the real, independent SAFE_OFF now
  // An approach (scout, fine 1 or fine 2) accepted its contact: GoalPosition
  // := that position is written next (V25 stop_pressure()), so the joint
  // rests ON the stop without pressing into it. Not terminal.
  RELEASE_PENDING       = 9,
  BASELINE_PENDING      = 10,  // write the one 64-tick baseline target
  BASELINE_MONITORING   = 11,  // record the moving current until it arrives
  // The release is verified by readback (V25 set_motor_goal_verified()),
  // then the backoff follows - or, after fine pass 2, repeatability and
  // COMPLETE. Not terminal.
  RELEASE_VERIFYING     = 12,
};

enum class ContactSearchStage : uint8_t {
  NONE           = 0,
  COARSE_TRANSIT = 1,  // coarse-scout step whose target is short of the corridor entry
  FINE_SEARCH    = 2,
  BACKOFF        = 3,
  BASELINE       = 4,
  COARSE_SCOUT   = 5,  // coarse-scout step whose target is inside the corridor
  RELEASE        = 6,
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
  BASELINE_PASSES_GUARD           = 24,  // the 64-tick baseline move would pass the guard
  SCOUT_MISSING                   = 25,  // a fine pass without an accepted coarse scout
};

struct ContactProbeConfig {
  // Monitors the two 96-tick backoffs (after the scout, after fine pass 1).
  actuator::MotionDeadmanConfig backoff_deadman{};
};

struct ContactProbeStatus {
  ContactProbePhase phase = ContactProbePhase::IDLE;
  ContactProbeFailure failure = ContactProbeFailure::NONE;
  ContactSearchStage stage = ContactSearchStage::NONE;
  uint8_t pass = 0;                   // 0 coarse scout, then fine passes 1 and 2
  // The V25 coarse contact scout: REFERENCE evidence for both fine passes,
  // never metrology. scout_valid is set only by an accepted scout contact.
  bool scout_valid = false;
  uint16_t scout_tick = 0;
  uint16_t pass1_contact_tick = 0;    // accepted FINE contacts, 0 = not yet
  uint16_t pass2_contact_tick = 0;
  uint16_t target_tick = 0;           // last commanded baseline / step / release / backoff target
  uint16_t step_count = 0;            // step writes this attempt (scout + both fine passes)
  uint16_t plateau_bypass_count = 0;  // fine candidates stepped past (V25 scout-lag rule)
  uint16_t kinematic_plateau_count = 0;  // V25 confirm_kinematic_plateau() entries
  uint16_t last_candidate_tick = 0;   // last confirmed candidate (detector or kinematic)
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
  // `acceptance_entry_depth` is the corridor entry depth (coarse scout), or
  // (both fine passes) the V25 adaptive bound min(entry, scout - 32) —
  // HOME-ward only.
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

// V25 StableTargetGate::observe_at() (matdog.rs L893), for the backoff: a
// sample qualifies within kSearchBackoffSettleToleranceTicks of the target
// with |speed| <= kSearchBackoffSettleMaxSpeedRaw (an unread speed never does);
// the target is settled once kSearchBackoffSettledSamples consecutive samples
// qualify AND kSearchBackoffSettleWindowMs have passed since the first of them. Any
// non-qualifying sample restarts both. Pure.
class SearchSettleGate {
 public:
  void reset() {
    consecutive_ = 0;
    has_first_ = false;
    first_ms_ = 0;
  }
  bool observe(uint16_t position, int32_t speed_magnitude, uint16_t target, uint32_t now_ms);
  uint8_t consecutive() const { return consecutive_; }

 private:
  uint8_t consecutive_ = 0;
  bool has_first_ = false;
  uint32_t first_ms_ = 0;
};

// The coarse-scout rules both fine passes apply, V25 function for function.
// V25 fine_contact_scout_lag_ticks(): how far `candidate` lies HOME-ward of
// the coarse scout along the probe direction; 0 at or beyond the scout.
int32_t searchScoutLagTicks(const actuator::CalibrationSearchCorridor& corridor,
                            uint16_t candidate, uint16_t scout);
// V25 fine_contact_reproduces_coarse_depth(): lag <= one fine step
// (FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS). A longer lag is a friction /
// chamfer plateau, stepped past.
bool searchFineContactReproducesScout(const actuator::CalibrationSearchCorridor& corridor,
                                      uint16_t candidate, uint16_t scout);
// V25 adaptive_contact_acceptance_bounds(profile, Some(scout)) as a depth: a
// fine pass accepts from min(entry, scout - ADAPTIVE_FINE_SCOUT_TICKS) to the
// guard - extended HOME-ward only, never toward the guard.
int32_t searchAdaptiveAcceptanceEntryDepth(const actuator::CalibrationSearchCorridor& corridor,
                                           uint16_t scout);

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
  void stepBaselineWrite(const ContactProbeContext& context, uint32_t now_ms);
  void stepBaselineMonitor(uint32_t now_ms, bool telemetry_available,
                           const actuator::TelemetrySample& telemetry);
  void finishBaseline(uint16_t position);
  void stepWrite(const ContactProbeContext& context, uint32_t now_ms);
  void stepMonitor(uint32_t now_ms, bool telemetry_available,
                  const actuator::TelemetrySample& telemetry);
  void stepBackoffWrite(const ContactProbeContext& context, uint32_t now_ms);
  void stepBackoffMonitor(uint32_t now_ms, bool telemetry_available,
                         const actuator::TelemetrySample& telemetry);
  void stepRelease(const ContactProbeContext& context);
  void stepReleaseVerify(uint32_t now_ms, bool telemetry_available,
                         const actuator::TelemetrySample& telemetry);
  // The stale / unusable-sample handling every monitor shares. True when
  // `telemetry` is a usable sample that passed sampleSafe().
  bool usableSafeSample(uint32_t now_ms, bool telemetry_available,
                        const actuator::TelemetrySample& telemetry);
  void observeKinematicPlateau(uint16_t position, int32_t speed);
  // Hard-current / thermal / torque checks on an accepted sample. False
  // (and the attempt failed) on any violation.
  bool sampleSafe(const actuator::TelemetrySample& telemetry);
  void onCandidate(uint16_t position);
  void beginRelease(uint16_t contact_tick);
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
  SearchSettleGate settle_{};

  bool has_good_sample_ = false;
  uint32_t last_good_ms_ = 0;
  uint32_t started_ms_ = 0;
  bool has_cadence_sample_ = false;
  uint32_t last_cadence_ms_ = 0;
  int32_t last_cadence_position_ = -1;
  uint32_t step_written_ms_ = 0;
  uint16_t step_ticks_ = 0;
  // Acceptance entry depth of the running approach (static, or adaptive).
  int32_t acceptance_entry_depth_ = 0;
  uint32_t baseline_started_ms_ = 0;
  uint16_t baseline_[kSearchBaselineMaxSamples] = {0};
  // The contact the running release parks on and the next backoff leaves.
  uint16_t contact_tick_ = 0;
  // V25 confirm_kinematic_plateau() in progress.
  bool plateau_active_ = false;
  uint8_t plateau_samples_ = 0;
  int32_t plateau_min_depth_ = 0;
  int32_t plateau_max_depth_ = 0;
};

const char* toString(ContactProbePhase phase);
const char* toString(ContactProbeFailure failure);
const char* toString(ContactSearchStage stage);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H
