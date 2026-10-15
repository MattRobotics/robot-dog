#ifndef MATDOG_CORE_MOTION_CLOCK_H
#define MATDOG_CORE_MOTION_CLOCK_H
// Monotonic 64-bit microsecond clock and absolute-deadline periodic scheduler
// for Motion streaming (Issue #52, 2026-10-15). Pure, host-tested.
//
// MonotonicClock64 extends a wrapping 32-bit microsecond counter (Arduino
// micros(), wraps every ~71.6 min) to 64 bits; it must be sampled at least
// once per wrap period (the Controller samples it every loop).
//
// PeriodicScheduler releases at next_release, next_release += period. A late
// poll releases ONCE (no catch-up, no backlog of old instants): every release
// instant that already passed beyond the current one is counted as skipped,
// never replayed. Overruns and lateness are counters, never hidden.
#include <stdint.h>
namespace matdog { namespace core {

class MonotonicClock64 {
 public:
  uint64_t extend(uint32_t now_us) {
    if (!started_) { started_ = true; last_ = now_us; total_ = now_us; return total_; }
    total_ += static_cast<uint32_t>(now_us - last_);  // modular delta handles the wrap
    last_ = now_us;
    return total_;
  }
  uint64_t now() const { return total_; }
 private:
  bool started_ = false;
  uint32_t last_ = 0;
  uint64_t total_ = 0;
};

struct SchedulerRelease {
  bool release = false;
  uint32_t sequence = 0;       // release counter, 1-based
  uint64_t deadline_us = 0;    // the absolute release instant served
  uint64_t lateness_us = 0;    // now - deadline
  uint32_t skipped = 0;        // release instants dropped by this poll
};
struct SchedulerCounters {
  uint32_t releases = 0, overruns = 0, skipped = 0;
  uint64_t max_lateness_us = 0;
};

class PeriodicScheduler {
 public:
  bool start(uint64_t now_us, uint32_t period_us) {
    if (period_us == 0) return false;
    period_us_ = period_us; next_us_ = now_us; running_ = true; sequence_ = 0; counters_ = {};
    return true;
  }
  void stop() { running_ = false; }
  bool running() const { return running_; }
  SchedulerRelease poll(uint64_t now_us) {
    SchedulerRelease r;
    if (!running_ || now_us < next_us_) return r;
    r.release = true; r.deadline_us = next_us_; r.lateness_us = now_us - next_us_;
    r.sequence = ++sequence_;
    const uint64_t missed = r.lateness_us / period_us_;  // whole periods already gone
    if (missed) { r.skipped = static_cast<uint32_t>(missed); ++counters_.overruns; counters_.skipped += r.skipped; }
    next_us_ += (missed + 1) * static_cast<uint64_t>(period_us_);
    ++counters_.releases;
    if (r.lateness_us > counters_.max_lateness_us) counters_.max_lateness_us = r.lateness_us;
    return r;
  }
  uint64_t nextReleaseUs() const { return next_us_; }
  uint32_t periodUs() const { return period_us_; }
  const SchedulerCounters& counters() const { return counters_; }
 private:
  bool running_ = false;
  uint32_t period_us_ = 0, sequence_ = 0;
  uint64_t next_us_ = 0;
  SchedulerCounters counters_{};
};
} }
#endif
