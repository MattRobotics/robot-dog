#ifndef MATDOG_UPDATE_OTA_AUTHORITY_GATE_H
#define MATDOG_UPDATE_OTA_AUTHORITY_GATE_H

#include "../core/ActuatorAuthority.h"
#include "OtaPolicy.h"

// OTA-B: the definitive OTA authorization gate, backed by the real
// ActuatorAuthority arbiter. It replaces the OTA-A placeholder entirely -
// there is no longer a "no authority model yet" answer anywhere.
//
// THE DESIGN DECISION THIS FILE ENCODES
// -------------------------------------
// OTA is not an actuator owner. It does not move a joint, read a servo, or
// write an EEPROM. Adding an OTA entry to the ActuatorAuthority enum would
// have made the arbiter describe something that is not an actuator user, and
// would have implied OTA competes with CALIBRATION for the same resource. It
// does not: it requires that NOBODY is using the actuators.
//
// So OTA takes an exclusivity INHIBIT instead of an ownership lease:
//
//     OTA request
//        -> requestInhibit(FIRMWARE_UPDATE)
//        -> granted only when current() == NONE
//        -> while held, every request() is REJECTED_INHIBITED
//
// Because the "is anyone an owner?" check and the hold happen inside one
// arbiter call, there is no window in which calibration could slip in
// between them - which is exactly the TOCTOU hazard a plain
// `if (authority == NONE)` leaves open across a multi-second update.
//
// Pure: no <Arduino.h>, no ESP-IDF, no radio. The offline suite drives the
// real gate against the real arbiter.

namespace matdog {
namespace update {

class OtaAuthorityGate : public OtaAuthorizationGate {
 public:
  // arbiter may be nullptr; that is a refusal, not a permission.
  void bind(core::ActuatorAuthorityArbiter* arbiter) { arbiter_ = arbiter; }

  OtaGateVerdict otaPermitted() const override;
  OtaGateVerdict beginExclusive() override;
  void endExclusive() override;

  bool holdsExclusive() const { return lease_.valid(); }
  const core::InhibitLease& lease() const { return lease_; }

 private:
  core::ActuatorAuthorityArbiter* arbiter_ = nullptr;
  core::InhibitLease lease_{};
};

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_AUTHORITY_GATE_H
