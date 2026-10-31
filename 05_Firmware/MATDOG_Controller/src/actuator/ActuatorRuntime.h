#ifndef MATDOG_ACTUATOR_ACTUATOR_RUNTIME_H
#define MATDOG_ACTUATOR_ACTUATOR_RUNTIME_H

#include <stdint.h>

#include "ActuatorWritePolicy.h"
#include "../diagnostics/PrimeDiagnostic.h"

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
// CR3 supplies the checked q<->raw target before the policy plan. The three
// calibration move classes therefore carry an absolute unsigned target_tick
// and map to the same one WRITE_GOAL_POSITION backend call.
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
  // The LF V25 RAM TorqueLimit, a fixed LOWERING value (ServoBus::
  // kReviewedRamTorqueLimit). Carries no target and no value: the command
  // cannot choose one.
  WRITE_CALIBRATION_TORQUE_LIMIT = 3,
};

BackendCallKind backendCallFor(ActuatorOperation operation);

// What a backend call proved about the write it just attempted. Three states,
// never a bool: a failed/absent acknowledgement on a real transport does not
// prove the command was not applied (see servo::ServoWriteVerifyResult, which
// this mirrors at the transport-independent layer), so "uncertain" must stay
// distinguishable from "verified not applied" all the way up to the caller
// that decides whether to escalate to SAFE_OFF.
enum class BackendWriteOutcome : uint8_t {
  VERIFIED_APPLIED     = 0,  // the backend independently confirmed the write took effect
  VERIFIED_NOT_APPLIED = 1,  // the backend independently confirmed it did not - proven safe
  UNCERTAIN            = 2,  // no independent confirmation was possible either way
};

const char* toString(BackendWriteOutcome outcome);

// Everything a real transport must supply to execute an ACCEPTed write.
// Deliberately minimal: SafeActuatorPolicy and backendCallFor() have
// already reduced every currently-executable ActuatorOperation to "apply
// torque" or "move to a raw tick" before this interface is ever consulted.
class ActuatorBackend {
 public:
  virtual ~ActuatorBackend() = default;
  // Observation only: copy the last original goal transaction from RAM.
  virtual bool lastGoalWriteCapture(diagnostics::GoalWriteCapture*) const {return false;}

  // APPLY torque only — never removes it. Bus id, not physical-unit label:
  // identity resolution already happened before execute() was called.
  virtual BackendWriteOutcome enableTorque(uint8_t bus_id) = 0;

  // Unsigned 0..4095 domain, exactly the GoalPosition contract — signed
  // wrap is forbidden and this interface cannot express one. `profile` is the
  // policy-checked MotionProfile of the command (see REJECT_MOTION_PROFILE).
  virtual BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick,
                                                MotionProfile profile) = 0;

  // RAM TorqueLimit := the one compiled calibration value (LF V25
  // TORQUE_LIMIT = 500), verified by readback. No argument selects a value.
  virtual BackendWriteOutcome writeCalibrationTorqueLimit(uint8_t bus_id) = 0;

  // Multi-joint Motion frame (Issue #52): the twelve committed GoalPositions in
  // ONE bus transaction (no per-joint readback). Not pure: a backend that does
  // not implement it refuses every frame and sends nothing. The verdict is
  // about TRANSMISSION, never about the servos having applied it:
  //   VERIFIED_NOT_APPLIED - provably nothing left the UART (refused before TX)
  //   UNCERTAIN            - TX started but not completed/confirmed (SAFE_OFF)
  //   VERIFIED_APPLIED     - NEVER returned for a frame (see FrameTxReport)
  // A completed transmission is reported through report->tx_complete and is
  // UNCONFIRMED until measurements follow (MotionExecutor tracking).
  struct FrameTxReport {
    bool tx_started = false, tx_complete = false;
    // True when these are original readings of the Controller's wrapping
    // micros() clock. False retains duration-only reports (host doubles).
    bool controller_clock = false;
    uint32_t tx_started_us = 0, tx_complete_us = 0;
    uint16_t bytes = 0;
  };
  virtual BackendWriteOutcome writeGoalPositionFrame(const uint8_t bus_ids[12], const uint16_t ticks[12],
                                                     FrameTxReport* report) {
    (void)bus_ids; (void)ticks; if (report) *report = FrameTxReport{};
    return BackendWriteOutcome::VERIFIED_NOT_APPLIED;
  }
};

// Outcome of one frame. TX_COMPLETE_UNCONFIRMED is NOT "applied": the bus
// gives no acknowledgement for a SYNC_WRITE; application is confirmed only by
// the following encoder samples.
enum class FrameExecuteResult : uint8_t {
  NOT_EXECUTED = 0,             // commitFrame() refused: zero bytes
  NO_BACKEND = 1,               // fail closed: zero bytes
  INVALID_BUS_IDS = 2,          // duplicate / auxiliary / not canonical: zero bytes
  NOT_SENT = 3,                 // backend refused before TX: zero bytes
  TX_COMPLETE_UNCONFIRMED = 4,  // all bytes left the UART (TX-complete), no ACK exists
  UNCERTAIN_REQUIRES_SAFE_OFF = 5,
};
const char* toString(FrameExecuteResult result);

enum class ExecuteResult : uint8_t {
  NOT_EXECUTED     = 0,  // commit() did not return ACCEPT — nothing was sent
  NO_BACKEND       = 1,  // ACCEPT, but no backend installed — fail closed
  NO_RAW_TARGET    = 2,  // ACCEPT, but this operation has no raw tick target yet
  WRITTEN          = 3,  // ACCEPT, backend independently verified the write applied
  BACKEND_REJECTED = 4,  // ACCEPT, backend independently verified it did NOT apply
  // ACCEPT, but the backend could not verify the outcome either way. The
  // actuator's resulting state is unknown and MUST be treated as potentially
  // energized/moved - the caller that owns the real transport is responsible
  // for escalating to an independent SAFE_OFF immediately on seeing this.
  UNCERTAIN_REQUIRES_SAFE_OFF = 5,
};

// Bridges SafeActuatorPolicy's decision to an ActuatorBackend. Holds no
// authority, no session and no limits of its own — every safety rule
// already lives in SafeActuatorPolicy, which this class calls and never
// second-guesses. Its only job: on ACCEPT, and only on ACCEPT, issue
// exactly one backend call for the committed command.
class ActuatorRuntime {
 public:
  bool lastGoalWriteCapture(diagnostics::GoalWriteCapture* out) const {
    return backend_&&out&&backend_->lastGoalWriteCapture(out);
  }
  void begin(SafeActuatorPolicy* policy, ActuatorBackend* backend);

  // Calls policy->commit(transaction) exactly once, then — only if that
  // returned ACCEPT — at most one backend call. Never calls the backend on
  // any other decision, never retries. `bus_id` is the caller's already-
  // resolved identity->bus-id lookup (see the file comment). Returns the
  // decision the policy actually made via decision_out, so the caller can
  // distinguish a policy refusal from a backend failure.
  ExecuteResult execute(ActuatorTransaction* transaction, uint8_t bus_id,
                        WriteDecision* decision_out);

  // One frame: commitFrame() exactly once, then - only on ACCEPT - exactly one
  // backend frame call. bus_ids are the caller's canonical identity->bus
  // lookups, index-aligned with transaction->frame.joints; they must be twelve
  // distinct ids of the canonical leg allocation (never the auxiliary unit).
  FrameExecuteResult executeFrame(ActuatorFrameTransaction* transaction, const uint8_t bus_ids[12],
                                  WriteDecision* decision_out, ActuatorBackend::FrameTxReport* report);
  // Set by the owner of the canonical allocation (ServoPopulation) - the
  // runtime itself stays free of servo/ includes. nullptr refuses every frame.
  void bindCanonicalBusIds(const uint8_t* ids, unsigned count) { canonical_ids_ = ids; canonical_count_ = count; }

 private:
  SafeActuatorPolicy* policy_ = nullptr;
  ActuatorBackend* backend_ = nullptr;
  const uint8_t* canonical_ids_ = nullptr;
  unsigned canonical_count_ = 0;
};

const char* toString(ExecuteResult result);
const char* toString(BackendCallKind kind);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_ACTUATOR_RUNTIME_H
