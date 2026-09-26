#ifndef MATDOG_ACTUATOR_ACTUATOR_RUNTIME_H
#define MATDOG_ACTUATOR_ACTUATOR_RUNTIME_H

#include <stdint.h>

#include "ActuatorWritePolicy.h"

// The runtime adapter from SafeActuatorPolicy toward a real transport —
// SAFE_ACTUATOR_LAYER.md §7's TO_IMPLEMENT item (I4, V3 handoff §12).
//
// Deliberately <stdint.h> plus ActuatorWritePolicy.h only: no <Arduino.h>,
// no ServoBus. scripts/tests/test_actuator_runtime.cpp links the REAL
// adapter and the REAL policy/arbiter against a fake backend, the same
// contract as update/OtaPolicy's fake OtaBackend.
//
// WHAT THIS DOES NOT DO, AND WHY
// -------------------------------
// There is still NO production ActuatorBackend implementation anywhere in
// this firmware, and none is added here: ServoBus exposes exactly one
// write, safeOff() (torque OFF), and no EnableTorque(id, 1) / GoalPosition
// write primitive exists to back a real ActuatorBackend with — adding one
// is outside this gate's authorization. scripts/static_audit.py's
// check_actuator_runtime_boundaries() fails the build if ActuatorRuntime is
// referenced anywhere outside src/actuator/ or the offline test suite, so
// its mere existence cannot make ordinary physical motion reachable: nothing
// in Controller.cpp constructs or owns one.
//
// This adapter also does not resolve a joint identity to a bus id — that
// stays the canonical allocation's job (servo/ServoPopulation.h), so this
// file never becomes a second place that mapping could silently disagree
// with. Callers supply the resolved bus id to execute().
//
// This adapter also does not convert a URDF-frame target (MicroRad) or a
// signed tick delta into a raw tick for the three geometry-authorised
// operations (CALIBRATION_CONTACT_PROBE / DIRECTION_VERIFY /
// CALIBRATION_AUXILIARY_MOVE) — that conversion needs an accepted q0/
// direction transform applied by a real execution engine, which is the
// future Calibration Execution Engine (I5), explicitly [NOT IMPLEMENTED]
// per ActuatorWritePolicy.h's own architecture diagram. Guessing that
// conversion here would be exactly the kind of unreviewed architectural
// decision the NextGen handoff says to stop for rather than invent. See
// backendCallFor() below: those three operations always resolve to
// BackendCallKind::NONE today.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY
// --------------------------------------------
// There is no operation class for removing torque and therefore no backend
// method for it either — exactly the same structural absence
// ActuatorWritePolicy.h already documents for itself. ServoBus::safeOff()
// stays the one ungated, policy-independent path.

namespace matdog {
namespace actuator {

// What kind of backend call, if any, a committed ActuatorCommand needs.
// Pure and total over ActuatorOperation, so it is exhaustively host-testable
// without first reconstructing a full policy ACCEPT for every operation —
// in particular the three geometry-authorised ones, whose ACCEPT path needs
// a compiled geometry profile, a live bootstrap context and an endpoint
// plan: exactly what test_actuator_write_policy.cpp already proves at
// length. Duplicating that fixture here would test the policy again, not
// this adapter.
enum class BackendCallKind : uint8_t {
  NONE                = 0,  // no raw tick target exists yet — see the file comment above
  ENABLE_TORQUE       = 1,
  WRITE_GOAL_POSITION = 2,
};

BackendCallKind backendCallFor(ActuatorOperation operation);

// Everything a real transport must supply to execute an ACCEPTed write.
// Deliberately minimal: SafeActuatorPolicy and backendCallFor() have
// already reduced every currently-executable ActuatorOperation to "apply
// torque" or "move to a raw tick" before this interface is ever consulted.
class ActuatorBackend {
 public:
  virtual ~ActuatorBackend() = default;

  // APPLY torque only — never removes it. Bus id, not physical-unit label:
  // identity resolution already happened before execute() was called.
  virtual bool enableTorque(uint8_t bus_id) = 0;

  // Unsigned 0..4095 domain, exactly the GoalPosition contract — signed
  // wrap is forbidden and this interface cannot express one.
  virtual bool writeGoalPosition(uint8_t bus_id, uint16_t target_tick) = 0;
};

enum class ExecuteResult : uint8_t {
  NOT_EXECUTED     = 0,  // commit() did not return ACCEPT — nothing was sent
  NO_BACKEND       = 1,  // ACCEPT, but no backend installed — fail closed
  NO_RAW_TARGET    = 2,  // ACCEPT, but this operation has no raw tick target yet
  WRITTEN          = 3,  // ACCEPT, backend call returned true
  BACKEND_REJECTED = 4,  // ACCEPT, backend call returned false
};

// Bridges SafeActuatorPolicy's decision to an ActuatorBackend. Holds no
// authority, no session and no limits of its own — every safety rule
// already lives in SafeActuatorPolicy, which this class calls and never
// second-guesses. Its only job: on ACCEPT, and only on ACCEPT, issue
// exactly one backend call for the committed command.
class ActuatorRuntime {
 public:
  void begin(SafeActuatorPolicy* policy, ActuatorBackend* backend);

  // Calls policy->commit(transaction) exactly once, then — only if that
  // returned ACCEPT — at most one backend call. Never calls the backend on
  // any other decision, never retries. `bus_id` is the caller's already-
  // resolved identity->bus-id lookup (see the file comment). Returns the
  // decision the policy actually made via decision_out, so the caller can
  // distinguish a policy refusal from a backend failure.
  ExecuteResult execute(ActuatorTransaction* transaction, uint8_t bus_id,
                        WriteDecision* decision_out);

 private:
  SafeActuatorPolicy* policy_ = nullptr;
  ActuatorBackend* backend_ = nullptr;
};

const char* toString(ExecuteResult result);
const char* toString(BackendCallKind kind);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_ACTUATOR_RUNTIME_H
