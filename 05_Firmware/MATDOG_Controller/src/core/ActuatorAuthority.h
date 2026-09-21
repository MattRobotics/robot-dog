#ifndef MATDOG_CORE_ACTUATOR_AUTHORITY_H
#define MATDOG_CORE_ACTUATOR_AUTHORITY_H

#include <stdint.h>

#include "OperatingMode.h"

// The single central arbiter of write authority over the actuators.
//
// Deliberately <stdint.h> only: no <Arduino.h>, no ServoBus, no Serial, no
// network. scripts/tests/test_actuator_authority.cpp links the REAL arbiter,
// the same contract as network/WifiPolicy and update/OtaPolicy.
//
// NOT A SECOND OperatingMode. The two are orthogonal axes:
//
//   OperatingMode       what the Controller as a whole is doing
//                       (MAINTENANCE: blocking diagnostics are safe /
//                        RUN: a deterministic motion loop may be active)
//
//   ActuatorAuthority   WHO, if anyone, currently holds the exclusive right
//                       to issue actuator writes
//
// A controller can sit in MAINTENANCE with authority NONE indefinitely; that
// is in fact the normal state today, because no write-capable owner exists
// yet. The mode says what is safe; the authority says who is doing it.
//
// WHAT THIS DOES NOT DO
// ---------------------
// It arbitrates WRITE authority. It does not serialize reads. @SERVO SCAN,
// @SERVO CENSUS and @SERVO READ are read-only (Ping/readByte/readWord) and
// deliberately take no authority: they are gated on MAINTENANCE because they
// BLOCK, not because they write. Making them take a write lock would make
// the system more restrictive than the hazard requires.
//
// SAFE_OFF IS OUTSIDE ARBITRATION - see the note on forceClear() below.

namespace matdog {
namespace core {

// The write-capable owners. NONE is the absence of an owner, not an owner.
enum class ActuatorAuthority : uint8_t {
  NONE         = 0,
  DIAGNOSTICS  = 1,  // diagnostics that genuinely need exclusive/controlled writes
  CALIBRATION  = 2,
  QC           = 3,
  PROVISIONING = 4,
  MOTION       = 5,
};

constexpr uint8_t kActuatorAuthorityCount = 6;

// Fail closed on a corrupted value. The underlying type is fixed at uint8_t,
// so casting an out-of-range byte to this enum is well defined - which is
// exactly why it has to be checked rather than assumed.
bool isKnownAuthority(ActuatorAuthority owner);
bool isWriteCapableOwner(ActuatorAuthority owner);

// Proof of ownership, handed out on a successful grant.
//
// WHY A GENERATION AND NOT JUST THE OWNER: matching the owner alone already
// rejects "CALIBRATION releases while MOTION holds". What it CANNOT catch is
// the same owner across two sessions - CALIBRATION acquires, releases,
// acquires again, and a late callback from the FIRST session calls
// release(CALIBRATION). The owner matches, and the second session's
// authority would be cleared out from under it. The generation makes that
// release provably stale.
struct AuthorityLease {
  ActuatorAuthority owner = ActuatorAuthority::NONE;
  uint32_t generation = 0;
  bool valid() const { return owner != ActuatorAuthority::NONE && generation != 0; }
};

// The exclusivity hold described below. Separate type from AuthorityLease on
// purpose: an inhibit holder is not an owner and must not be able to release
// one.
struct InhibitLease {
  uint32_t generation = 0;
  bool valid() const { return generation != 0; }
};

enum class AuthorityResult : uint8_t {
  GRANTED                = 0,
  ALREADY_OWNED          = 1,  // same owner re-requested; see the note in request()
  RELEASED               = 2,
  REJECTED_BUSY          = 3,  // a different owner holds it
  REJECTED_INHIBITED     = 4,  // an exclusive-activity hold is in place
  REJECTED_MODE          = 5,  // incompatible with the current OperatingMode
  REJECTED_UNKNOWN_OWNER = 6,  // NONE requested, or a corrupted enum value
  REJECTED_NOT_OWNER     = 7,  // release from something that does not hold it
  REJECTED_STALE_LEASE   = 8,  // right owner, wrong generation
  REJECTED_NOT_HELD      = 9,  // release when nothing is held
};

enum class AuthorityClearReason : uint8_t {
  BOOT                   = 0,
  FATAL_FAULT            = 1,
  OWNER_TEARDOWN         = 2,
  OPERATING_MODE_CHANGED = 3,
  OPERATOR_RESET         = 4,
  NEVER_CLEARED          = 5,
};

// Why the actuators are held inhibited. OTA is the first and only user:
// a firmware update is not an actuator owner, it is an actuator INHIBITOR.
enum class InhibitReason : uint8_t {
  NONE            = 0,
  FIRMWARE_UPDATE = 1,
};

struct AuthorityCounters {
  uint32_t grants = 0;
  uint32_t already_owned = 0;
  uint32_t releases = 0;
  uint32_t rejections = 0;
  uint32_t stale_releases = 0;
  uint32_t force_clears = 0;
  uint32_t inhibit_grants = 0;
  uint32_t inhibit_rejections = 0;
};

// Which owners may hold authority in which operating mode.
//
// This is the INITIAL table and it is enforced, not decorative. It changes no
// behaviour today because no owner is ever acquired yet - nothing in the
// firmware can write an actuator except SAFE_OFF, which is outside
// arbitration entirely. It encodes what OperatingMode.h already documents:
// RUN means a deterministic motion loop may be active, so the blocking
// service owners must not be able to appear there; MAINTENANCE means
// blocking diagnostics are acceptable, so motion must not.
//
// TO_REVIEW when the motion loop lands and the RUN default flips.
bool isModeCompatible(OperatingMode mode, ActuatorAuthority owner);

class ActuatorAuthorityArbiter {
 public:
  // Boot and every re-init. Always lands on NONE with no inhibit held; a
  // previous authority is NEVER restored.
  void reset(AuthorityClearReason reason);

  ActuatorAuthority current() const { return owner_; }
  uint32_t generation() const { return generation_; }
  bool inhibited() const { return inhibit_generation_ != 0; }
  InhibitReason inhibitReason() const { return inhibit_reason_; }

  // Exclusive acquisition. At most one write-capable owner exists at a time.
  //
  // Re-requesting the owner that already holds it returns ALREADY_OWNED and
  // deliberately does NOT hand out a lease. Returning a second valid lease
  // would let two holders each believe they own it, and either could then
  // release it out from under the other - the same class of bug the
  // generation exists to prevent. ALREADY_OWNED forces the caller to notice
  // instead of silently creating a second transaction.
  AuthorityResult request(ActuatorAuthority owner, OperatingMode mode,
                          AuthorityLease* out_lease);

  // Only the current owner, holding a current lease, can release.
  AuthorityResult release(const AuthorityLease& lease);

  // Fail-closed path: fatal fault, owner teardown, abort, re-init. Always
  // lands on NONE.
  //
  // NOTE ON SAFE_OFF: nothing here can block a safety de-escalation, and that
  // is structural rather than a promise. This unit has no reference to
  // ServoBus and ServoBus has no reference to this unit, so
  // ServoBus::safeOff() cannot consult an authority even if someone later
  // wanted it to. scripts/static_audit.py enforces both directions.
  void forceClear(AuthorityClearReason reason);

  // An operating-mode change that leaves the current owner incompatible
  // clears it rather than leaving a suspended authority behind.
  void onOperatingModeChanged(OperatingMode mode);

  // ---- Exclusive-activity inhibit ---------------------------------------
  //
  // Solves a real TOCTOU window, and does it without inventing a fake
  // actuator owner.
  //
  // MATDOG's Controller is single-threaded - every subsystem runs
  // cooperatively inside Controller::update(), and nothing in MATDOG's own
  // code owns a task or a callback. So a check-then-act sequence that
  // completes inside one call is already atomic. The problem is not
  // concurrency, it is DURATION: an OTA update spans thousands of loop
  // passes, and between two of them a command can arrive and a future
  // CalibrationManager can acquire authority. A query cannot close that
  // window; only a hold can.
  //
  // requestInhibit() succeeds only from current() == NONE, and the
  // NONE -> inhibited transition is a single call, so there is no window
  // between the check and the hold. While held, every request() is refused.
  //
  // A stuck inhibit is fail-safe: it blocks actuator writes, it does not
  // enable them. It is cleared by reset() at boot, or explicitly.
  AuthorityResult requestInhibit(InhibitReason reason, InhibitLease* out_lease);
  AuthorityResult releaseInhibit(const InhibitLease& lease);
  void forceClearInhibit();

  const AuthorityCounters& counters() const { return counters_; }
  AuthorityClearReason lastClearReason() const { return last_clear_reason_; }
  AuthorityResult lastResult() const { return last_result_; }

 private:
  AuthorityResult record(AuthorityResult result);

  ActuatorAuthority owner_ = ActuatorAuthority::NONE;
  uint32_t generation_ = 0;
  uint32_t inhibit_generation_ = 0;
  InhibitReason inhibit_reason_ = InhibitReason::NONE;
  AuthorityClearReason last_clear_reason_ = AuthorityClearReason::NEVER_CLEARED;
  AuthorityResult last_result_ = AuthorityResult::RELEASED;
  AuthorityCounters counters_{};
};

const char* toString(ActuatorAuthority owner);
const char* toString(AuthorityResult result);
const char* toString(AuthorityClearReason reason);
const char* toString(InhibitReason reason);

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_ACTUATOR_AUTHORITY_H
