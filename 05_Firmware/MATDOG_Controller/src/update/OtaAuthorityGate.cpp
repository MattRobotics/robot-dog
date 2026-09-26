#include "OtaAuthorityGate.h"

namespace matdog {
namespace update {

OtaGateVerdict OtaAuthorityGate::otaPermitted() const {
  // Query only. Truthful about the present instant and nothing more - which
  // is precisely why prepare() uses beginExclusive() instead.
  if (arbiter_ == nullptr) return OtaGateVerdict::REFUSED_NO_GATE_INSTALLED;
  if (holdsExclusive()) return OtaGateVerdict::PERMITTED_BY_AUTHORITY;
  if (arbiter_->inhibited()) return OtaGateVerdict::REFUSED_ALREADY_EXCLUSIVE;
  if (arbiter_->current() != core::ActuatorAuthority::NONE) {
    return OtaGateVerdict::REFUSED_ACTUATOR_OWNER_ACTIVE;
  }
  return OtaGateVerdict::PERMITTED_BY_AUTHORITY;
}

OtaGateVerdict OtaAuthorityGate::beginExclusive() {
  if (arbiter_ == nullptr) return OtaGateVerdict::REFUSED_NO_GATE_INSTALLED;

  // Not re-entrant: a second update attempt must not be able to release the
  // first one's hold. An already-held gate simply reports permitted without
  // taking a second lease.
  if (holdsExclusive()) return OtaGateVerdict::PERMITTED_BY_AUTHORITY;

  // Capture the reason BEFORE asking, so the refusal can say which of the two
  // blocking conditions applied. The arbiter's own result cannot distinguish
  // "someone owns the actuators" from "another exclusive activity is running"
  // as precisely as the operator needs.
  const bool owner_active = arbiter_->current() != core::ActuatorAuthority::NONE;
  const bool already_exclusive = arbiter_->inhibited();

  core::InhibitLease lease;
  const core::AuthorityResult result =
      arbiter_->requestInhibit(core::InhibitReason::FIRMWARE_UPDATE, &lease);

  if (result != core::AuthorityResult::GRANTED) {
    if (already_exclusive) return OtaGateVerdict::REFUSED_ALREADY_EXCLUSIVE;
    if (owner_active) return OtaGateVerdict::REFUSED_ACTUATOR_OWNER_ACTIVE;
    return OtaGateVerdict::REFUSED_BY_AUTHORITY;
  }

  lease_ = lease;
  return OtaGateVerdict::PERMITTED_BY_AUTHORITY;
}

void OtaAuthorityGate::endExclusive() {
  // Idempotent and safe when nothing is held: OtaPolicy calls this from every
  // failure path, including ones where the hold was never taken.
  if (arbiter_ == nullptr || !holdsExclusive()) return;
  arbiter_->releaseInhibit(lease_);
  lease_ = core::InhibitLease{};
}

}  // namespace update
}  // namespace matdog
