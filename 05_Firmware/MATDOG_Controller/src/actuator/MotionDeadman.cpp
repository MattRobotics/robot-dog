#include "MotionDeadman.h"

namespace matdog {
namespace actuator {

void MotionDeadmanMonitor::begin(const MotionDeadmanConfig& config, uint16_t target_tick,
                                 uint32_t started_at_ms) {
  config_ = config;
  target_tick_ = target_tick;
  started_at_ms_ = started_at_ms;
  // The staleness/stall clocks start from motion start, not from "forever
  // ago": a monitor that has not yet received its first sample must not
  // immediately report STALE_TELEMETRY/STALLED before the caller has had any
  // chance to poll.
  last_good_sample_ms_ = started_at_ms;
  last_progress_ms_ = started_at_ms;
  last_progress_position_ = -1;
  began_ = true;
}

MotionDeadmanVerdict MotionDeadmanMonitor::evaluateAgainstClockOnly(uint32_t now_ms) const {
  if (!began_) return MotionDeadmanVerdict::STALE_TELEMETRY;  // fail closed if misused

  // Timeout outranks everything: an exhausted budget is exhausted regardless
  // of how healthy telemetry looked a moment ago.
  if (now_ms - started_at_ms_ >= config_.motion_timeout_ms) {
    return MotionDeadmanVerdict::TIMED_OUT;
  }
  if (now_ms - last_good_sample_ms_ >= config_.max_telemetry_age_ms) {
    return MotionDeadmanVerdict::STALE_TELEMETRY;
  }
  if (last_progress_position_ >= 0 &&
      now_ms - last_progress_ms_ >= config_.stall_window_ms) {
    return MotionDeadmanVerdict::STALLED;
  }
  return MotionDeadmanVerdict::CONTINUE;
}

MotionDeadmanVerdict MotionDeadmanMonitor::poll(uint32_t now_ms) const {
  return evaluateAgainstClockOnly(now_ms);
}

MotionDeadmanVerdict MotionDeadmanMonitor::evaluate(const TelemetrySample& sample,
                                                    uint32_t now_ms) {
  if (!began_) return MotionDeadmanVerdict::STALE_TELEMETRY;

  if (now_ms - started_at_ms_ >= config_.motion_timeout_ms) {
    return MotionDeadmanVerdict::TIMED_OUT;
  }

  if (!sample.read_ok) {
    // A single dropped poll inside an otherwise fresh window must not abort
    // a bounded move - only once the age window has genuinely elapsed since
    // the last KNOWN GOOD sample does this become the reported verdict.
    if (now_ms - last_good_sample_ms_ >= config_.max_telemetry_age_ms) {
      return MotionDeadmanVerdict::COMMUNICATION_LOST;
    }
    return MotionDeadmanVerdict::CONTINUE;
  }

  // A successful read. "Known good" tracks communication health, not
  // whether the joint is behaving - update it regardless of what follows.
  last_good_sample_ms_ = sample.sampled_at_ms;

  const int32_t delta = sample.present_position - static_cast<int32_t>(target_tick_);
  const int32_t magnitude = delta < 0 ? -delta : delta;
  if (magnitude <= static_cast<int32_t>(config_.arrival_tolerance_ticks)) {
    return MotionDeadmanVerdict::ARRIVED;
  }

  if (sample.torque_enable == 0) {
    return MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF;
  }

  if (last_progress_position_ < 0) {
    last_progress_position_ = sample.present_position;
    last_progress_ms_ = now_ms;
  } else {
    const int32_t moved = sample.present_position - last_progress_position_;
    const int32_t moved_magnitude = moved < 0 ? -moved : moved;
    if (moved_magnitude >= static_cast<int32_t>(config_.stall_progress_ticks)) {
      last_progress_position_ = sample.present_position;
      last_progress_ms_ = now_ms;
    } else if (now_ms - last_progress_ms_ >= config_.stall_window_ms) {
      return MotionDeadmanVerdict::STALLED;
    }
  }

  return MotionDeadmanVerdict::CONTINUE;
}

const char* toString(MotionDeadmanVerdict verdict) {
  switch (verdict) {
    case MotionDeadmanVerdict::CONTINUE:                return "CONTINUE";
    case MotionDeadmanVerdict::ARRIVED:                 return "ARRIVED";
    case MotionDeadmanVerdict::STALE_TELEMETRY:         return "STALE_TELEMETRY";
    case MotionDeadmanVerdict::COMMUNICATION_LOST:      return "COMMUNICATION_LOST";
    case MotionDeadmanVerdict::TORQUE_UNEXPECTEDLY_OFF: return "TORQUE_UNEXPECTEDLY_OFF";
    case MotionDeadmanVerdict::STALLED:                 return "STALLED";
    case MotionDeadmanVerdict::TIMED_OUT:                return "TIMED_OUT";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
