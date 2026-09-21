#include "ActuatorAuthority.h"

namespace matdog {
namespace core {

bool isKnownAuthority(ActuatorAuthority owner) {
  return static_cast<uint8_t>(owner) < kActuatorAuthorityCount;
}

bool isWriteCapableOwner(ActuatorAuthority owner) {
  return isKnownAuthority(owner) && owner != ActuatorAuthority::NONE;
}

bool isModeCompatible(OperatingMode mode, ActuatorAuthority owner) {
  if (!isKnownAuthority(owner)) return false;   // fail closed on a corrupted value
  if (owner == ActuatorAuthority::NONE) return true;  // releasing is always compatible

  switch (mode) {
    case OperatingMode::MAINTENANCE:
      // Everything service-shaped, but not motion: MAINTENANCE is where
      // blocking diagnostics are acceptable, and a motion loop must never
      // share a mode with something allowed to block the bus.
      return owner != ActuatorAuthority::MOTION;
    case OperatingMode::RUN:
      // Only motion: RUN means a deterministic loop may be running, and the
      // service owners are exactly the ones that would starve it.
      return owner == ActuatorAuthority::MOTION;
  }
  return false;  // unreachable today; fail closed if OperatingMode ever grows
}

void ActuatorAuthorityArbiter::reset(AuthorityClearReason reason) {
  // A previous authority is never restored. Boot, re-init and fatal recovery
  // all land in exactly the same place.
  owner_ = ActuatorAuthority::NONE;
  generation_ = 0;
  inhibit_generation_ = 0;
  inhibit_reason_ = InhibitReason::NONE;
  last_clear_reason_ = reason;
  last_result_ = AuthorityResult::RELEASED;
  counters_ = AuthorityCounters{};
}

AuthorityResult ActuatorAuthorityArbiter::record(AuthorityResult result) {
  last_result_ = result;
  switch (result) {
    case AuthorityResult::GRANTED:        counters_.grants++; break;
    case AuthorityResult::ALREADY_OWNED:  counters_.already_owned++; break;
    case AuthorityResult::RELEASED:       counters_.releases++; break;
    case AuthorityResult::REJECTED_STALE_LEASE:
      counters_.stale_releases++;
      counters_.rejections++;
      break;
    default:                              counters_.rejections++; break;
  }
  return result;
}

AuthorityResult ActuatorAuthorityArbiter::request(ActuatorAuthority owner, OperatingMode mode,
                                                  AuthorityLease* out_lease) {
  // Never hand out a lease on any path that is not a grant. Doing this first
  // means no early return can leave a caller holding a stale-looking lease.
  if (out_lease != nullptr) *out_lease = AuthorityLease{};

  // NONE is the absence of an owner, not something that can be acquired.
  if (!isWriteCapableOwner(owner)) return record(AuthorityResult::REJECTED_UNKNOWN_OWNER);

  // The inhibit outranks everything, including the current owner being the
  // same one. It exists precisely to make "no new owner may appear" hold for
  // the whole duration of an exclusive activity.
  if (inhibited()) {
    counters_.inhibit_rejections++;
    return record(AuthorityResult::REJECTED_INHIBITED);
  }

  if (!isModeCompatible(mode, owner)) return record(AuthorityResult::REJECTED_MODE);

  if (owner_ == owner) {
    // Deliberately no lease. See the note in the header: a second valid
    // lease for the same owner is a release-out-from-under-me bug waiting to
    // happen. The existing holder keeps the only lease.
    return record(AuthorityResult::ALREADY_OWNED);
  }

  if (owner_ != ActuatorAuthority::NONE) return record(AuthorityResult::REJECTED_BUSY);

  owner_ = owner;
  ++generation_;
  if (generation_ == 0) ++generation_;  // 0 means "no lease"; never wrap onto it
  if (out_lease != nullptr) {
    out_lease->owner = owner;
    out_lease->generation = generation_;
  }
  return record(AuthorityResult::GRANTED);
}

AuthorityResult ActuatorAuthorityArbiter::release(const AuthorityLease& lease) {
  if (owner_ == ActuatorAuthority::NONE) return record(AuthorityResult::REJECTED_NOT_HELD);
  if (lease.owner != owner_) return record(AuthorityResult::REJECTED_NOT_OWNER);

  // Right owner, wrong generation: a late release from a PREVIOUS session of
  // the same owner. Matching the owner alone would have accepted this and
  // cleared the current session's authority.
  if (lease.generation != generation_) return record(AuthorityResult::REJECTED_STALE_LEASE);

  owner_ = ActuatorAuthority::NONE;
  // generation_ is NOT reset: it must keep increasing so a lease from this
  // session can never be confused with one from the next.
  return record(AuthorityResult::RELEASED);
}

void ActuatorAuthorityArbiter::forceClear(AuthorityClearReason reason) {
  last_clear_reason_ = reason;
  if (owner_ == ActuatorAuthority::NONE) return;  // idempotent
  owner_ = ActuatorAuthority::NONE;
  counters_.force_clears++;
  last_result_ = AuthorityResult::RELEASED;
}

void ActuatorAuthorityArbiter::onOperatingModeChanged(OperatingMode mode) {
  if (owner_ == ActuatorAuthority::NONE) return;
  if (isModeCompatible(mode, owner_)) return;
  // An owner left stranded by a mode change is a suspended authority, which
  // is exactly what must never exist.
  forceClear(AuthorityClearReason::OPERATING_MODE_CHANGED);
}

AuthorityResult ActuatorAuthorityArbiter::requestInhibit(InhibitReason reason,
                                                         InhibitLease* out_lease) {
  if (out_lease != nullptr) *out_lease = InhibitLease{};

  if (reason == InhibitReason::NONE) {
    counters_.inhibit_rejections++;
    return record(AuthorityResult::REJECTED_UNKNOWN_OWNER);
  }
  if (inhibited()) {
    // Not re-entrant on purpose: two holders would each be able to release
    // the other's hold, which is the whole problem this is here to avoid.
    counters_.inhibit_rejections++;
    return record(AuthorityResult::REJECTED_INHIBITED);
  }
  if (owner_ != ActuatorAuthority::NONE) {
    // THE ATOMIC CHECK. An exclusive activity may only begin from a state
    // where nobody owns the actuators, and because this check and the hold
    // happen in the same call there is no window between them.
    counters_.inhibit_rejections++;
    return record(AuthorityResult::REJECTED_BUSY);
  }

  ++generation_;
  if (generation_ == 0) ++generation_;
  inhibit_generation_ = generation_;
  inhibit_reason_ = reason;
  counters_.inhibit_grants++;
  if (out_lease != nullptr) out_lease->generation = inhibit_generation_;
  return record(AuthorityResult::GRANTED);
}

AuthorityResult ActuatorAuthorityArbiter::releaseInhibit(const InhibitLease& lease) {
  if (!inhibited()) return record(AuthorityResult::REJECTED_NOT_HELD);
  if (lease.generation != inhibit_generation_) {
    counters_.stale_releases++;
    counters_.rejections++;
    last_result_ = AuthorityResult::REJECTED_STALE_LEASE;
    return AuthorityResult::REJECTED_STALE_LEASE;
  }
  inhibit_generation_ = 0;
  inhibit_reason_ = InhibitReason::NONE;
  return record(AuthorityResult::RELEASED);
}

void ActuatorAuthorityArbiter::forceClearInhibit() {
  if (!inhibited()) return;
  inhibit_generation_ = 0;
  inhibit_reason_ = InhibitReason::NONE;
  counters_.force_clears++;
}

const char* toString(ActuatorAuthority owner) {
  switch (owner) {
    case ActuatorAuthority::NONE:         return "NONE";
    case ActuatorAuthority::DIAGNOSTICS:  return "DIAGNOSTICS";
    case ActuatorAuthority::CALIBRATION:  return "CALIBRATION";
    case ActuatorAuthority::QC:           return "QC";
    case ActuatorAuthority::PROVISIONING: return "PROVISIONING";
    case ActuatorAuthority::MOTION:       return "MOTION";
  }
  return "UNKNOWN";
}

const char* toString(AuthorityResult result) {
  switch (result) {
    case AuthorityResult::GRANTED:                return "GRANTED";
    case AuthorityResult::ALREADY_OWNED:          return "ALREADY_OWNED";
    case AuthorityResult::RELEASED:               return "RELEASED";
    case AuthorityResult::REJECTED_BUSY:          return "REJECTED_BUSY";
    case AuthorityResult::REJECTED_INHIBITED:     return "REJECTED_INHIBITED";
    case AuthorityResult::REJECTED_MODE:          return "REJECTED_MODE";
    case AuthorityResult::REJECTED_UNKNOWN_OWNER: return "REJECTED_UNKNOWN_OWNER";
    case AuthorityResult::REJECTED_NOT_OWNER:     return "REJECTED_NOT_OWNER";
    case AuthorityResult::REJECTED_STALE_LEASE:   return "REJECTED_STALE_LEASE";
    case AuthorityResult::REJECTED_NOT_HELD:      return "REJECTED_NOT_HELD";
  }
  return "UNKNOWN";
}

const char* toString(AuthorityClearReason reason) {
  switch (reason) {
    case AuthorityClearReason::BOOT:                   return "BOOT";
    case AuthorityClearReason::FATAL_FAULT:            return "FATAL_FAULT";
    case AuthorityClearReason::OWNER_TEARDOWN:         return "OWNER_TEARDOWN";
    case AuthorityClearReason::OPERATING_MODE_CHANGED: return "OPERATING_MODE_CHANGED";
    case AuthorityClearReason::OPERATOR_RESET:         return "OPERATOR_RESET";
    case AuthorityClearReason::NEVER_CLEARED:          return "NEVER_CLEARED";
  }
  return "UNKNOWN";
}

const char* toString(InhibitReason reason) {
  switch (reason) {
    case InhibitReason::NONE:            return "NONE";
    case InhibitReason::FIRMWARE_UPDATE: return "FIRMWARE_UPDATE";
  }
  return "UNKNOWN";
}

}  // namespace core
}  // namespace matdog
