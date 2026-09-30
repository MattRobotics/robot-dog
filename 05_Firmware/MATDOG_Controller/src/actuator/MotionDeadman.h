#ifndef MATDOG_ACTUATOR_MOTION_DEADMAN_H
#define MATDOG_ACTUATOR_MOTION_DEADMAN_H

#include <stdint.h>

// CR3 Priority 4 — telemetry health / deadman evaluation for a single
// bounded joint motion, shared by the first-motion executor (Priority 3) and
// the contact-probe engine (Priority 5).
//
// Pure: <stdint.h> only. No Arduino, no ServoBus, no clock of its own - every
// sample and every "now" is supplied by the caller, which is what makes this
// host-testable with synthetic telemetry instead of real hardware.
//
// WHAT TELEMETRY THIS ASSUMES, AND WHY
// -------------------------------------
// TelemetrySample below carries exactly what servo::ServoBus::RuntimeState
// already proves readable (present_position, torque_enable) plus the two
// facts no single register read carries on its own: whether the read attempt
// itself succeeded, and when it was taken. present_speed/present_load/
// present_current are real ST3215/SCServo registers but are not consulted
// here - this monitor's job is "is this move still safe to continue", which
// position + torque + communication health already answer; a future
// consumer with a specific load/current use case can extend TelemetrySample
// without this file inventing a threshold it cannot justify.
//
// AGE WINDOWS, NOT FRAME COUNTS
// -------------------------------
// Every staleness/stall decision here is a wall-clock age comparison against
// a caller-supplied window, never "N consecutive missed polls": Controller's
// own tick rate is not guaranteed uniform (see Controller::update()'s own
// comments on Wi-Fi/OTA ticks being the expensive, irregular ones), so a
// frame-count deadman would silently retune itself whenever the tick rate
// changed. See evaluate()/poll() below.

namespace matdog {
namespace actuator {

struct TelemetrySample {
  bool read_ok = false;
  uint32_t sampled_at_ms = 0;
  int32_t present_position = -1;
  int32_t torque_enable = -1;
  // Raw ST3215 registers, -1 = not read. Not consulted by this monitor; the
  // calibration endpoint search (ContactProbeEngine) uses them exactly as the
  // LF V25 hardware oracle did: speed magnitude for its kinematic contact
  // detector, current for the hard-current abort and the post-backoff
  // recovery check, temperature for the thermal abort.
  int32_t present_speed = -1;
  int32_t present_current = -1;
  int32_t present_temperature = -1;
  // The rest of the LF V25 per-observation readback (matdog.rs
  // validate_lf_active_readback / ensure_observation_safe), -1 = not read:
  // the GoalPosition register, the RAM TorqueLimit register and the servo's
  // own status/error byte (register 65). Filled by the calibration
  // telemetry read (ServoBus::readControlFeedback); the 24-contact
  // sequence treats an unread value as an unusable sample.
  int32_t goal_position = -1;
  int32_t torque_limit = -1;
  int32_t servo_status = -1;
};

// Ordered so a caller can treat CONTINUE/ARRIVED as "keep going" and
// anything else as "stop and escalate" without naming every value.
enum class MotionDeadmanVerdict : uint8_t {
  CONTINUE                = 0,  // healthy; keep monitoring
  ARRIVED                 = 1,  // within tolerance of the commanded target
  STALE_TELEMETRY         = 2,  // no successful sample within the age window
  COMMUNICATION_LOST      = 3,  // the latest poll attempt itself failed, and
                                 // the age window has now genuinely elapsed
  TORQUE_UNEXPECTEDLY_OFF = 4,  // fresh sample, torque reads 0, not yet arrived
  STALLED                 = 5,  // no sufficient position progress within the
                                 // stall window
  TIMED_OUT               = 6,  // the overall motion budget is exhausted
};

struct MotionDeadmanConfig {
  // How stale a successful sample may be before it stops counting as known
  // good. CalibrationDomain.h's CalibrationFailure::TELEMETRY_STALE cites
  // MAX_TELEMETRY_AGE = 3 s as the reviewed LF V25 precedent for this figure;
  // callers should reuse it rather than invent a new one.
  uint32_t max_telemetry_age_ms = 0;
  // Overall wall-clock budget for one bounded move, independent of whether
  // telemetry looks healthy along the way. Outranks every other verdict.
  // With nominal_travel_ticks_per_s == 0 this IS the whole budget; otherwise
  // it is the budget ON TOP OF the move's nominal travel time (see below).
  uint32_t motion_timeout_ms = 0;
  // A window over which SOME minimum position progress is required once
  // motion has been commanded; catches a joint that keeps answering every
  // poll but is not actually moving (mechanical jam, load limit, etc).
  uint32_t stall_window_ms = 0;
  uint16_t stall_progress_ticks = 0;
  uint16_t arrival_tolerance_ticks = 0;
  // The speed the backend actually commands for this move, in ticks/s
  // (servo::ServoBus::kBoundedWriteSpeed for every calibration move). 0 keeps
  // the fixed motion_timeout_ms budget. Non-zero makes the budget
  // travel-aware: the first successful in-range (0..4095) sample fixes the
  // start position, and the budget becomes motion_timeout_ms PLUS the
  // nominal time to cover |target - start| at this rate. Hardware finding
  // 2026-09-29: at the bounded 40 ticks/s a fixed 12 s budget caps any move
  // at ~480 ticks (~42 deg), shorter than the Geometry V5 UPPER contact
  // travel (MIN ~590 ticks, MAX up to ~1980), so every Full-Leg first
  // approach ended TIMED_OUT before it could reach the stop. The start is
  // fixed once and never re-derived: a late first sample (the joint already
  // under way) can only yield a SHORTER budget, never a longer one.
  uint16_t nominal_travel_ticks_per_s = 0;
};

// A small state machine, not a filter: it remembers the last known-good
// sample and the last position progress so a single dropped poll cannot
// itself abort a move, while a genuinely stale/stuck one still does.
class MotionDeadmanMonitor {
 public:
  void begin(const MotionDeadmanConfig& config, uint16_t target_tick, uint32_t started_at_ms);

  // Call once per tick where a poll was actually attempted - sample.read_ok
  // distinguishes "polled and got no answer" from "polled and it looks fine".
  MotionDeadmanVerdict evaluate(const TelemetrySample& sample, uint32_t now_ms);

  // Call on a tick where no poll was even attempted, so elapsed wall-clock
  // time alone can still surface staleness/timeout. Never updates the "last
  // known good" bookkeeping - it can only report a verdict evaluate() could
  // also have reported, never a better one.
  MotionDeadmanVerdict poll(uint32_t now_ms) const;

  uint32_t lastGoodSampleAtMs() const { return last_good_sample_ms_; }
  bool began() const { return began_; }
  // The overall budget currently enforced: motion_timeout_ms until a
  // travel-aware budget (nominal_travel_ticks_per_s != 0) has been fixed by
  // the first in-range sample, the extended budget afterwards.
  uint32_t motionBudgetMs() const { return motion_budget_ms_; }
  // The position this monitor was tracking for stall purposes, valid once a
  // STALLED verdict has been returned (from either evaluate() or poll()) -
  // -1 if no progress sample has been recorded yet. Exists so a caller that
  // treats STALLED as a meaningful physical event (e.g. a contact-probe
  // engine, where a stall IS the signal being sought) can record WHERE it
  // occurred without this class inventing its own evidence type.
  int32_t lastProgressPosition() const { return last_progress_position_; }

 private:
  MotionDeadmanVerdict evaluateAgainstClockOnly(uint32_t now_ms) const;
  // Fixes the travel-aware budget from the first in-range sample (no-op when
  // nominal_travel_ticks_per_s is 0 or the budget is already fixed).
  void fixTravelBudget(int32_t present_position);

  MotionDeadmanConfig config_{};
  uint16_t target_tick_ = 0;
  uint32_t started_at_ms_ = 0;
  uint32_t motion_budget_ms_ = 0;
  bool travel_budget_fixed_ = false;
  uint32_t last_good_sample_ms_ = 0;
  uint32_t last_progress_ms_ = 0;
  int32_t last_progress_position_ = -1;  // negative: no progress sample yet
  bool began_ = false;
};

const char* toString(MotionDeadmanVerdict verdict);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_MOTION_DEADMAN_H
