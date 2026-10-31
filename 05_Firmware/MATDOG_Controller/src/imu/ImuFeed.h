#ifndef MATDOG_IMU_IMU_FEED_H
#define MATDOG_IMU_IMU_FEED_H
// IMU acquisition core (Core Motion V5, 2026-10-20): pure logic, no Arduino, host-testable.
//
// Verified in the installed libraries (Adafruit_BNO08x 1.2.x / sh2):
//  * sh2.c sensorhubInputHdlr() invokes the sensor callback ONCE PER REPORT of a
//    SHTP cargo, but Adafruit_BNO08x::sensorHandler() decodes every report into
//    the SAME sh2_SensorValue_t: with several reports in one cargo only the last
//    survives getSensorEvent(); and Bno085Imu::update() consumed ONE getSensorEvent()
//    per Controller loop for five enabled reports (~170 events/s).
//  * spihal_read()/spihal_write() call spihal_wait_for_int(): with INT high that
//    polls delay(1) up to 500 times and then HARDWARE-RESETS the sensor.
//  * The SPI HAL never sets the cargo time (t_us stays 0), so event.timestamp_uS is
//    only the sample time RELATIVE to the interrupt, not an absolute clock.
// Hence: per-report callback, no wait when INT is high, bounded drain, host receive
// timestamps, and a single-writer snapshot the Motion thread reads without waiting.
#include <stdint.h>
#include <string.h>
#include <atomic>
#include <type_traits>

namespace matdog { namespace imu {

inline int32_t diffUs(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b); }

struct RotationSample {
  bool valid = false;
  uint8_t status = 0;
  uint16_t reserved = 0;
  uint32_t seq = 0;            // Rotation Vector events accepted since boot
  uint32_t rx_ms = 0;          // host millis() when the cargo carrying it was read
  uint32_t rx_us = 0;          // host micros() (wraps)
  int32_t sensor_age_us = 0;   // sample time before the interrupt, from the SH-2 stack (0 = unknown)
  float accuracy_rad = 0;
  float wxyz[4] = {1, 0, 0, 0};
};
static_assert(std::is_trivially_copyable<RotationSample>::value, "seqlock copy");
static_assert(sizeof(RotationSample) % 4 == 0, "word copy");

// Single writer, any number of readers, nobody ever waits. Two slots: the writer fills the slot that is NOT
// current (sequence odd while writing, even when done) and only then makes it current, so a reader always has
// the latest COMPLETED sample with its original timestamp, even while the writer task is preempted halfway
// through a publish. (The first version used one slot: a reader that overlapped a preempted writer got
// "no sample" and the Motion Authority read that as IMU_STALE - see test_imu_feed.cpp.) A reader only retries
// if the writer completes two further publishes during one 40-byte copy.
// Sequentially consistent operations throughout: no standalone fences, so the protocol is also checkable
// with ThreadSanitizer.
template <typename T>
class SeqSlot {
  static constexpr unsigned kWords = (sizeof(T) + 3) / 4;
 public:
  void publish(const T& v) { publishBegin(v); publishEnd(); }
  // Split form so a test can freeze the writer in the middle of a publish.
  void publishBegin(const T& v) {
    uint32_t w[kWords] = {0};
    memcpy(w, &v, sizeof(T));
    next_ = (cur_.load() & 1u) ^ 1u;
    Slot& s = slot_[next_];
    const uint32_t q = s.seq.load();
    s.seq.store(q + 1);
    for (unsigned i = 0; i < kWords; ++i) s.words[i].store(w[i]);
    pending_seq_ = q + 2;
  }
  void publishEnd() {
    slot_[next_].seq.store(pending_seq_);
    cur_.store(next_ | 2u);          // bit 1: something has been published
  }
  bool read(T* out) const {
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
      const uint32_t c = cur_.load();
      if (!(c & 2u)) return false;     // nothing published yet
      const Slot& s = slot_[c & 1u];
      const uint32_t a = s.seq.load();
      if (a & 1u) continue;
      uint32_t w[kWords];
      for (unsigned i = 0; i < kWords; ++i) w[i] = s.words[i].load();
      if (s.seq.load() == a) { memcpy(out, w, sizeof(T)); return true; }
    }
    return false;
  }
 private:
  struct Slot { std::atomic<uint32_t> seq{0}; std::atomic<uint32_t> words[kWords]{}; };
  Slot slot_[2];
  std::atomic<uint32_t> cur_{0};
  unsigned next_ = 0;                  // writer-only state
  uint32_t pending_seq_ = 0;
};

struct ImuServiceStats {
  uint32_t service_calls = 0, transfers = 0;
  uint32_t rv = 0, game = 0, mag = 0, acc = 0, gyr = 0;
  uint32_t budget_stops = 0;       // drain ended by the time budget with data still pending
  uint32_t backlog_stops = 0;      // drain ended by the transfer cap with data still pending
  uint32_t max_service_us = 0, max_gap_us = 0;  // longest service pass / longest time between passes
  uint32_t rv_gap_last_us = 0, rv_gap_min_us = 0, rv_gap_max_us = 0;  // host arrival spacing of Rotation Vector events
  uint32_t resets = 0;
};

class ImuFeed {
 public:
  void onRotationVector(const float wxyz[4], float accuracy, uint8_t status, int32_t sensor_age_us, uint32_t now_us, uint32_t now_ms) {
    RotationSample s;
    s.valid = true; s.status = status; s.seq = ++stats_.rv; s.rx_ms = now_ms; s.rx_us = now_us;
    s.sensor_age_us = sensor_age_us; s.accuracy_rad = accuracy;
    for (unsigned i = 0; i < 4; ++i) s.wxyz[i] = wxyz[i];
    if (have_rv_) {
      const uint32_t gap = static_cast<uint32_t>(diffUs(now_us, last_rv_us_));
      stats_.rv_gap_last_us = gap;
      if (gap > stats_.rv_gap_max_us) stats_.rv_gap_max_us = gap;
      if (stats_.rv_gap_min_us == 0 || gap < stats_.rv_gap_min_us) stats_.rv_gap_min_us = gap;
    }
    have_rv_ = true; last_rv_us_ = now_us;
    slot_.publish(s);
  }
  void noteOther(unsigned kind) { if (kind == 0) ++stats_.game; else if (kind == 1) ++stats_.mag; else if (kind == 2) ++stats_.acc; else ++stats_.gyr; }
  void noteReset() { ++stats_.resets; }
  bool snapshot(RotationSample* out) const { return slot_.read(out); }
  ImuServiceStats& stats() { return stats_; }
  const ImuServiceStats& stats() const { return stats_; }
  uint32_t rvCount() const { return stats_.rv; }
 private:
  SeqSlot<RotationSample> slot_;
  ImuServiceStats stats_{};
  bool have_rv_ = false;
  uint32_t last_rv_us_ = 0;
};

// Bounded drain of the SH-2 stack. ready() is the INT line (active low), transfer()
// one sh2_service() = one SHTP transfer (it does not wait because ready() was true),
// now() a wrap-safe microsecond clock.
struct ServicePass { uint32_t transfers = 0; bool budget_stop = false, backlog_stop = false; uint32_t elapsed_us = 0; };
template <class Ready, class Transfer, class Now>
ServicePass servicePass(Ready ready, Transfer transfer, Now now, uint32_t max_transfers, uint32_t budget_us) {
  ServicePass r;
  const uint32_t t0 = now();
  while (ready()) {
    if (r.transfers >= max_transfers) { r.backlog_stop = true; break; }
    if (diffUs(now(), t0) >= static_cast<int32_t>(budget_us)) { r.budget_stop = true; break; }
    transfer();
    ++r.transfers;
  }
  r.elapsed_us = static_cast<uint32_t>(diffUs(now(), t0));
  return r;
}

} }
#endif
