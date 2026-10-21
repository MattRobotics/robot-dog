#ifndef MATDOG_SERVO_BUS_TRANSACTION_H
#define MATDOG_SERVO_BUS_TRANSACTION_H
// Bounded, single-owner servo UART transaction (Core Motion V5, 2026-10-20).
//
// Why: SCServo's IOTimeOut is an INTER-BYTE timeout (SCSerial::readSCS restarts
// it after every byte), and Ack()/Ping()/rFlushSCS() have no overall deadline
// (106 ms for a 15-byte reply at 7 ms/byte with IOTimeOut 8, V4 sdk-timeout.log).
// Every SCServo byte already funnels through the virtual readSCS/writeSCS/
// rFlushSCS hooks, so the deadline is imposed THERE, not by shortening a timeout.
//
// Contract (pure logic: no Arduino, no allocation, host-testable):
//  * Exactly one transaction is open at a time; a second begin() is refused and
//    counted (ownership). Optionally the caller thread token must match the
//    owner token captured at bind().
//  * Absolute budget on a 32-bit microsecond clock compared with signed
//    differences (valid across the 71-minute wrap for budgets < 2^31 us).
//  * A "phase" is the wait for ONE reply: it starts when the request has been
//    handed to the UART and may last at most phase_us, however slowly bytes
//    arrive. The total budget bounds the sum of all phases (write+ACK+readback).
//  * After the total budget or phase is exhausted, or after a preemption, the
//    transaction is CLOSED FOR TX: no further request leaves the UART.
//  * A preemptible transaction (poll/readback) is abandoned as soon as a STOP is
//    pending. Safety transactions (SAFE_OFF, verify-off) are never preemptible
//    but are always bounded.
//  * The result is typed; a half packet is never "OK".
#include <stdint.h>

namespace matdog { namespace servo {

enum class BusOutcome : uint8_t {
  NONE = 0,            // no transaction has completed yet
  OK,                  // every SCServo wait was satisfied
  NO_RESPONSE,         // a phase expired with zero reply bytes
  INCOMPLETE,          // a phase expired mid-packet (slow or truncated reply)
  CORRUPT,             // a full-length reply failed validation (checksum/id/length/status)
  TOTAL_DEADLINE,      // the absolute budget was exhausted (progressive-but-too-slow reply)
  ABORTED_STOP,        // abandoned because a STOP is pending
  RX_FLOOD,            // stale/babbling RX could not be drained within bound
  REFUSED_REENTRANT,   // another transaction was open
  REFUSED_FOREIGN_OWNER, // called from a task that does not own the bus
  REFUSED_UNOWNED_IO,  // SCServo I/O outside any transaction
  RX_PENDING,         // an earlier reply is unresolved; no ambiguous request sent
  CANCELLED_STOP,     // admitted STOP, provably zero bytes transmitted
};
const char* toString(BusOutcome);

struct BusStats {
  uint32_t transactions = 0, ok = 0, no_response = 0, incomplete = 0, corrupt = 0, total_deadline = 0;
  uint32_t aborted_stop = 0, rx_flood = 0, refused_reentrant = 0, refused_foreign = 0, unowned_io = 0;
  uint32_t suppressed_tx = 0;     // requests NOT sent because the transaction was already closed
  uint32_t last_tx_bytes = 0;
  uint32_t max_elapsed_us = 0, last_elapsed_us = 0, max_overrun_us = 0;  // overrun = elapsed - total budget (>0 only by poll granularity)
};

// Signed wrap-safe difference a-b on the 32-bit microsecond clock.
inline int32_t busDiffUs(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b); }

class BusTransaction {
 public:
  typedef uint32_t (*ClockFn)();
  typedef bool (*PreemptFn)(void*);
  typedef uintptr_t (*OwnerFn)();

  // Maximum bytes/time spent discarding stale RX before a new request.
  static constexpr uint16_t kDrainMaxBytes = 64;
  static constexpr uint32_t kDrainMaxUs = 400;
  static constexpr uint32_t kMaxBudgetUs = 0x7fffffffu / 2;

  void bindClock(ClockFn f) { clock_ = f; }
  void bindPreempt(PreemptFn f, void* ctx) { preempt_ = f; preempt_ctx_ = ctx; }
  // The owner token is captured once; any later transaction from a different
  // token is refused. Without a bound owner function no thread check is made.
  void bindOwner(OwnerFn f) { owner_fn_ = f; owner_ = f ? f() : 0; }

  bool open() const { return open_; }
  // For bus use that needs no reply (SYNC_WRITE bursts): true only if no
  // transaction is open and the caller owns the bus; otherwise counted and refused.
  bool idleAndOwned() {
    if (open_) { ++stats_.refused_reentrant; outcome_ = BusOutcome::REFUSED_REENTRANT; return false; }
    if (owner_fn_ && owner_fn_() != owner_) { ++stats_.refused_foreign; outcome_ = BusOutcome::REFUSED_FOREIGN_OWNER; return false; }
    return true;
  }
  bool closedForTx() const { return open_ && (closed_ || abort_); }
  bool cancelledBeforeTx() const { return abort_ && tx_bytes_ == 0; }
  BusOutcome outcome() const { return outcome_; }
  const BusStats& stats() const { return stats_; }
  uint32_t elapsedUs() const { return open_ ? busDiffUs(now(), start_us_) : stats_.last_elapsed_us; }

  // Opens a transaction. total_us bounds the whole operation, phase_us each
  // reply wait. Returns false (and the caller must fail safe) if refused.
  bool begin(uint32_t total_us, uint32_t phase_us, bool preemptible) {
    if (open_) { ++stats_.refused_reentrant; outcome_ = BusOutcome::REFUSED_REENTRANT; return false; }
    if (owner_fn_ && owner_fn_() != owner_) { ++stats_.refused_foreign; outcome_ = BusOutcome::REFUSED_FOREIGN_OWNER; return false; }
    if (total_us == 0 || total_us > kMaxBudgetUs || phase_us == 0) { outcome_ = BusOutcome::NO_RESPONSE; return false; }
    open_ = true; closed_ = false; abort_ = false; preemptible_ = preemptible;
    total_us_ = total_us; phase_us_ = phase_us; start_us_ = now(); phase_start_us_ = start_us_;
    bytes_this_phase_ = 0; bytes_total_ = 0; tx_bytes_ = 0; flood_ = false; pending_rx_ = false; deadline_hit_ = false; failed_wait_ = false;
    ++stats_.transactions;
    return true;
  }

  // Called by the SCServo write hook before bytes leave the UART. False = suppress.
  bool allowTx() {
    if (!open_) { ++stats_.unowned_io; outcome_ = BusOutcome::REFUSED_UNOWNED_IO; return false; }
    if (expired(now()) && total_us_ > phase_us_) deadline_hit_ = true;
    if (closed_ || abort_ || flood_ || expired(now())) { closed_ = true; ++stats_.suppressed_tx; return false; }
    if (preemptible_ && preemptNow()) { ++stats_.suppressed_tx; return false; }
    return true;
  }
  // The request has been handed to the UART: the reply phase starts now.
  void txDone() { if (open_) { phase_start_us_ = now(); bytes_this_phase_ = 0; } }
  void onTx(unsigned sent, unsigned requested) {
    tx_bytes_ += sent;
    if (sent != requested) { closed_ = true; failed_wait_ = true; }
  }

  // One reply-wait poll. Returns true while it is still legitimate to wait.
  // Call when no byte is available; on a received byte call onByte().
  bool mayWait() {
    if (!open_) { ++stats_.unowned_io; outcome_ = BusOutcome::REFUSED_UNOWNED_IO; return false; }
    if (abort_ || closed_) return false;
    const uint32_t t = now();
    // The reply phase is judged first: a single-phase operation (total == phase)
    // that runs out is a slow/absent REPLY, not a "total budget" event. A phase
    // that ran out ends THAT wait only; the next request may still use the rest.
    if (busDiffUs(t, phase_start_us_) >= static_cast<int32_t>(phase_us_)) return false;
    if (busDiffUs(t, start_us_) >= static_cast<int32_t>(total_us_)) {
      closed_ = true;
      if (total_us_ > phase_us_) deadline_hit_ = true;   // single-phase: the late reply IS the phase timeout
      return false;
    }
    if (preemptible_ && preemptNow()) return false;
    return true;
  }
  void onByte() { ++bytes_this_phase_; ++bytes_total_; }
  // A wait ended without the requested byte count.
  void waitFailed() { failed_wait_ = true; }
  void noteFlood() { flood_ = true; closed_ = true; }
  void notePendingRx() { pending_rx_ = true; closed_ = true; }
  uint32_t drainBudgetUs() const { return kDrainMaxUs; }

  // Closes the transaction. `valid` = the caller's own packet validation passed.
  BusOutcome end(bool valid) {
    if (!open_) return outcome_;
    if (valid && !mayWait()) { waitFailed(); valid = false; }
    const uint32_t elapsed = static_cast<uint32_t>(busDiffUs(now(), start_us_));
    // The caller's own validation is the verdict (an informational ACK that never
    // came does not fail a transaction whose readback validated). Otherwise the
    // failure mode is classified from what the bus actually did.
    BusOutcome o;
    if (valid) o = BusOutcome::OK;
    else if (flood_) o = BusOutcome::RX_FLOOD;
    else if (abort_) o = tx_bytes_ ? BusOutcome::ABORTED_STOP : BusOutcome::CANCELLED_STOP;
    else if (deadline_hit_) o = BusOutcome::TOTAL_DEADLINE;
    else if (pending_rx_) o = BusOutcome::RX_PENDING;
    else if (failed_wait_) o = bytes_total_ == 0 ? BusOutcome::NO_RESPONSE : BusOutcome::INCOMPLETE;
    else o = BusOutcome::CORRUPT;
    outcome_ = o; open_ = false;
    stats_.last_tx_bytes = tx_bytes_;
    stats_.last_elapsed_us = elapsed;
    if (elapsed > stats_.max_elapsed_us) stats_.max_elapsed_us = elapsed;
    if (elapsed > total_us_ && elapsed - total_us_ > stats_.max_overrun_us) stats_.max_overrun_us = elapsed - total_us_;
    switch (o) {
      case BusOutcome::OK: ++stats_.ok; break;
      case BusOutcome::NO_RESPONSE: ++stats_.no_response; break;
      case BusOutcome::INCOMPLETE: ++stats_.incomplete; break;
      case BusOutcome::CORRUPT: ++stats_.corrupt; break;
      case BusOutcome::TOTAL_DEADLINE: ++stats_.total_deadline; break;
      case BusOutcome::ABORTED_STOP: ++stats_.aborted_stop; break;
      case BusOutcome::CANCELLED_STOP: ++stats_.aborted_stop; break;
      case BusOutcome::RX_FLOOD: ++stats_.rx_flood; break;
      default: break;
    }
    return o;
  }

 private:
  uint32_t now() const { return clock_ ? clock_() : 0; }
  bool expired(uint32_t t) const {
    return busDiffUs(t, start_us_) >= static_cast<int32_t>(total_us_);
  }
  bool preemptNow() {
    if (preempt_ && preempt_(preempt_ctx_)) { abort_ = true; return true; }
    return false;
  }
  ClockFn clock_ = nullptr; PreemptFn preempt_ = nullptr; void* preempt_ctx_ = nullptr; OwnerFn owner_fn_ = nullptr; uintptr_t owner_ = 0;
  bool open_ = false, closed_ = false, abort_ = false, preemptible_ = false, flood_ = false, pending_rx_ = false, deadline_hit_ = false, failed_wait_ = false;
  uint32_t tx_bytes_ = 0;
  uint32_t total_us_ = 0, phase_us_ = 0, start_us_ = 0, phase_start_us_ = 0;
  uint16_t bytes_this_phase_ = 0, bytes_total_ = 0;
  BusOutcome outcome_ = BusOutcome::NONE;
  BusStats stats_{};
};

inline const char* toString(BusOutcome o) {
  switch (o) {
    case BusOutcome::NONE: return "NONE"; case BusOutcome::OK: return "OK";
    case BusOutcome::NO_RESPONSE: return "NO_RESPONSE"; case BusOutcome::INCOMPLETE: return "INCOMPLETE"; case BusOutcome::CORRUPT: return "CORRUPT";
    case BusOutcome::TOTAL_DEADLINE: return "TOTAL_DEADLINE"; case BusOutcome::ABORTED_STOP: return "ABORTED_STOP";
    case BusOutcome::RX_FLOOD: return "RX_FLOOD"; case BusOutcome::REFUSED_REENTRANT: return "REFUSED_REENTRANT";
    case BusOutcome::REFUSED_FOREIGN_OWNER: return "REFUSED_FOREIGN_OWNER"; case BusOutcome::REFUSED_UNOWNED_IO: return "REFUSED_UNOWNED_IO";
    case BusOutcome::RX_PENDING: return "RX_PENDING"; case BusOutcome::CANCELLED_STOP: return "CANCELLED_STOP";
  }
  return "UNKNOWN";
}

// RAII guard: ends the transaction on every exit path.
class BusGuard {
 public:
  BusGuard(BusTransaction& t, uint32_t total_us, uint32_t phase_us, bool preemptible)
      : t_(t), ok_(t.begin(total_us, phase_us, preemptible)) {}
  ~BusGuard() { if (ok_) t_.end(valid_); }
  bool ok() const { return ok_; }
  // The caller's final judgement of the packet(s) it parsed.
  void result(bool valid) { valid_ = valid; }
  BusGuard(const BusGuard&) = delete; BusGuard& operator=(const BusGuard&) = delete;
 private:
  BusTransaction& t_; bool ok_; bool valid_ = false;
};

} }
#endif
