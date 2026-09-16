#ifndef MATDOG_SERVO_SERVO_CENSUS_H
#define MATDOG_SERVO_SERVO_CENSUS_H

#include <stdint.h>

#include "ServoBus.h"
#include "ServoPopulation.h"

namespace matdog {
namespace servo {

// Controller-owned semantic service: "run a canonical servo census and hold
// the structured result". G2.
//
// WHY THIS IS NOT IN CommandRouter
// --------------------------------
// V0.1's @SERVO SCAN lives entirely inside CommandRouter: it parses a
// range, starts a scan, watches for completion and printf()s the result.
// That is acceptable for a raw bus probe, but a census is domain logic, and
// the handoff is explicit that new G2 policy/state must not exist only
// inside Serial parsing or Serial printing (sections 7/8/9).
//
// So the split is:
//   ServoBus       -> "what did the physical bus observe?"     (transport)
//   ServoCensus    -> "run one, hold the structured meaning"   (service)
//   ServoPopulation-> "what does that mean for this robot?"    (pure policy)
//   CommandRouter  -> formats result() for USB CDC             (adapter)
//
// A future Web UI / HostLink transport adds a second adapter over the same
// result(). It never re-scans the bus to render a page, and it never
// reimplements the classification — which is exactly the telemetry-snapshot
// requirement in handoff section 9.
//
// This class deliberately contains NO Serial and NO printing.
//
// SAFETY
// ------
// The census is strictly read-only: it drives ServoBus::startScan(), whose
// only bus traffic is Ping(). No torque, no GoalPosition, no register
// write, no EEPROM access. It is never started automatically — not at boot,
// not on a timer (see Controller::begin(), which must not call start()).
class ServoCensus {
 public:
  enum class State : uint8_t {
    IDLE     = 0,  // never run, or finished
    RUNNING  = 1,  // a scan started by this service is in progress
    COMPLETE = 2,  // result() holds a classified census
  };

  void begin(ServoBus* bus) { bus_ = bus; }

  // Starts a canonical census over [kCanonicalScanLo, kCanonicalScanHi].
  // Returns false if this service is already running, if the bus is
  // unavailable, or if ServoBus refuses (another scan in progress) — the
  // caller reports that, this never blocks waiting for the bus.
  //
  // Like @SERVO SCAN, this carries bounded per-ID blocking (one Ping() per
  // update() tick, see ServoBus.h) and is therefore MAINTENANCE-only at the
  // command surface. The gate lives in CommandRouter alongside the existing
  // two, not here, so there is exactly one place that policy is stated.
  bool start();

  // Call once per Controller tick, AFTER ServoBus::update(). Detects the
  // scan's RUNNING -> COMPLETE edge and classifies the raw result exactly
  // once. Cheap no-op otherwise.
  void update();

  State state() const { return state_; }

  // The last classified census. verdict == NOT_RUN until one completes.
  // Returned by const reference; copy it by value into any snapshot.
  const CensusResult& result() const { return result_; }

 private:
  ServoBus* bus_ = nullptr;
  State state_ = State::IDLE;
  CensusResult result_{};
};

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_CENSUS_H
