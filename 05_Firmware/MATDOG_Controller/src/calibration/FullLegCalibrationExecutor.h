#ifndef MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_EXECUTOR_H
#define MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_EXECUTOR_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../actuator/MotionDeadman.h"
#include "CalibrationDomain.h"
#include "CalibrationExecutionEngine.h"
#include "ContactProbeEngine.h"

// CR3 continuation — the ONE UPPER-endpoint hardware sequence a Full Leg
// Calibration needs: MIN contact (unobstructed) then MAX contact. Whether the
// MAX side needs a named auxiliary joint parked first is a per-leg fact the
// caller reads from Geometry V5 and states in FullLegCalibrationRequest
// (auxiliary_required): LF/RF MAX are obstructed (LH_UPPER/RH_UPPER park
// first), RH/LH MAX are NOT_NEEDED and skip every AUX_* phase. Composes
// ContactProbeEngine (Priority 5) and CalibrationExecutionEngine's
// AUXILIARY_MOVE path — reused whole, not reimplemented.
//
// Pure: <stdint.h> plus already-pure MATDOG units. No Arduino, no ServoBus,
// no clock of its own — same contract as FirstMotionExecutor.h and
// ContactProbeEngine.h: every tick's "now" and every telemetry sample are
// supplied by the caller.
//
// WHAT THIS DOES NOT OWN
// -----------------------
//   - HIP/LOWER envelopes: pure geometry arithmetic
//     (actuator::buildGeometryDerivedEnvelope over the joint's own q0
//     transform + URDF domain), no backend call, no state machine needed —
//     the caller (CommandRouter) computes those directly, synchronously,
//     once this engine's UPPER result is known.
//   - the UPPER envelope itself: also computed by the caller, via
//     actuator::buildContactDerivedEnvelope over the two ContactEvidence
//     records this class exposes once COMPLETE — this class PRODUCES
//     evidence, it does not decide what a safe range is.
//   - the exact endpoint/auxiliary numbers (bus ids, contact/backoff/park
//     targets): supplied by the caller in FullLegCalibrationRequest, exactly
//     as ContactProbeRequest already does. The caller resolves them from the
//     canonical allocation and Geometry V5 (FullLegCalibrationPlan); this
//     class stays generic and carries no leg-specific constant.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY, FOR UP TO *TWO* SERVOS
// -----------------------------------------------------------------
// No ServoBus reference exists here, so this class cannot call safeOff() for
// either the probed joint OR the parked auxiliary. UPPER_MIN_SAFE_OFF and
// FINAL_SAFE_OFF are phases where this class does nothing but wait for the
// caller's independently-verified SAFE_OFF confirmation
// (primarySafeOffVerified / auxiliarySafeOffVerified, supplied to update()
// exactly like telemetry is) — see primaryBusId()/auxiliaryBusId() and the
// Controller-side forcing loop this mirrors (updateFirstMotion()).
//
// WITHOUT AN AUXILIARY (auxiliary_required == false) THERE IS ONLY ONE SERVO
// -----------------------------------------------------------------
// auxiliaryBusId() is 0, auxiliarySafeOffPending() is never true and
// FINAL_SAFE_OFF waits for the primary alone. auxiliaryBusId() == 0 is a
// sentinel for "no auxiliary", never an address: a caller that would pass it
// to a SAFE_OFF must first check auxiliaryRequired().
//
// EVERY FAILURE ROUTES THROUGH A SAFE_OFF-SERVICING PHASE FIRST
// -----------------------------------------------------------------
// Once the primary joint's TorqueEnable has even been ATTEMPTED, every exit
// — success or failure, including a dynamic prerequisite lost mid-sequence —
// requests the real SAFE_OFF before reporting a terminal phase. "It is never
// wrong to send one as routine session hygiene" (ContactProbeEngine.h) is
// applied uniformly here rather than re-deriving, case by case, whether one
// is strictly required.

namespace matdog {
namespace calibration {

enum class FullLegCalibrationPhase : uint8_t {
  IDLE                = 0,
  UPPER_MIN_PROBE     = 1,  // delegates to an owned ContactProbeEngine
  UPPER_MIN_SAFE_OFF  = 2,  // caller must verify SAFE_OFF on primaryBusId()
  AUX_TORQUE_ENABLE   = 3,  // TorqueEnable on the auxiliary joint (skipped without one)
  AUX_MOVE_PENDING    = 4,  // AUXILIARY_MOVE to the compiler's parked pose
  AUX_MOVE_MONITORING = 5,  // wait for ARRIVED; no stall-as-signal here
  UPPER_MAX_PROBE     = 6,  // delegates to the SAME owned ContactProbeEngine
  FINAL_SAFE_OFF      = 7,  // caller must verify SAFE_OFF on every servo in use
  // --- terminal states ---------------------------------------------------
  COMPLETE            = 8,  // both sides witnessed; evidence is ready
  FAILED              = 9,
};

enum class FullLegCalibrationFailure : uint8_t {
  NONE                             = 0,
  REJECT_PRECONDITIONS             = 1,  // start() itself refused
  UPPER_MIN_PROBE_FAILED           = 2,  // detail: probeStatus()
  AUX_TORQUE_ENABLE_REJECTED       = 3,
  AUX_TORQUE_ENABLE_UNCERTAIN      = 4,
  AUX_MOVE_REJECTED                = 5,
  AUX_MOVE_UNCERTAIN               = 6,
  AUX_MOVE_TARGET_RESOLUTION       = 7,
  AUX_MOVE_STALE_TELEMETRY         = 8,
  AUX_MOVE_COMMUNICATION_LOST      = 9,
  AUX_MOVE_TORQUE_UNEXPECTEDLY_OFF = 10,
  AUX_MOVE_STALLED                 = 11,
  AUX_MOVE_TIMEOUT                 = 12,
  UPPER_MAX_PROBE_FAILED           = 13,  // detail: probeStatus()
  DYNAMIC_PREREQUISITE_LOST        = 14,  // permit/session/authority lost mid-sequence
  OPERATOR_ABORT                   = 15,
};

struct FullLegCalibrationConfig {
  actuator::MotionDeadmanConfig probe_approach_deadman{};
  actuator::MotionDeadmanConfig probe_backoff_deadman{};
  actuator::MotionDeadmanConfig aux_move_deadman{};
};

struct FullLegCalibrationRequest {
  // The joint being calibrated (the UPPER of the leg under calibration).
  JointIdentity probe_joint{};
  uint8_t probe_bus_id = 0;
  Leg endpoint_leg = Leg::LF;
  JointKind endpoint_joint = JointKind::UPPER;

  // MIN side — unobstructed in the current Geometry V5 bundle.
  actuator::MicroRad min_approach_urad = 0;
  actuator::MicroRad min_backoff_urad = 0;
  uint16_t min_repeatability_tolerance_ticks = 0;

  // MAX side — obstructed on LF/RF (the auxiliary is parked first), direct on
  // RH/LH (auxiliary_required == false).
  actuator::MicroRad max_approach_urad = 0;
  actuator::MicroRad max_backoff_urad = 0;
  uint16_t max_repeatability_tolerance_ticks = 0;

  // Both sides, both approach passes: raw ticks past the canonical contact
  // (min/max_approach_urad stay exactly the Geometry V5 contacts) - see
  // ContactProbeRequest::approach_overtravel_ticks. Never the backoff.
  uint16_t approach_overtravel_ticks = 0;

  // Whether the MAX side needs an auxiliary joint parked first. Set from the
  // Geometry V5 MAX endpoint record's has_auxiliary by the caller — never
  // hard-coded per leg. Defaults to TRUE (fail-closed: forgetting to state it
  // keeps the stricter, two-servo sequence and its start() checks).
  bool auxiliary_required = true;

  // The auxiliary joint the compiler's own plan names for the MAX side, and
  // its exact parked pose — cross-checked against the bound geometry's own
  // endpoint record by SafeActuatorPolicy::evaluateEndpointPlan(), never
  // re-derived here. Ignored (may be empty / bus 0) when auxiliary_required
  // is false.
  JointIdentity auxiliary_joint{};
  uint8_t auxiliary_bus_id = 0;
  actuator::MicroRad auxiliary_park_target_urad = 0;
};

struct FullLegCalibrationStatus {
  FullLegCalibrationPhase phase = FullLegCalibrationPhase::IDLE;
  FullLegCalibrationFailure failure = FullLegCalibrationFailure::NONE;
  actuator::WriteDecision last_policy_decision = actuator::WriteDecision::REJECT_NO_ARBITER;
};

// Mirrors FirstMotionExecutor's newer, dynamically-rechecked context shape
// (2026-09-28 continuation) rather than ContactProbeEngine's older, simpler
// one: this class re-validates every field on EVERY tick, including while an
// owned ContactProbeEngine sub-probe is mid-flight, so a permit/authority
// pulled mid-sequence is caught within one tick instead of only at the next
// backend call.
struct FullLegCalibrationContext {
  bool session_active = false;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
  bool motion_permit_active = false;
  core::ActuatorAuthority authority = core::ActuatorAuthority::NONE;
  uint32_t authority_generation = 0;
  bool authority_inhibited = false;
};

class FullLegCalibrationExecutor {
 public:
  // No pointer is owned; all may be nullptr, which is a refusal, never a
  // permission. `engine` drives CONTACT_PROBE (via the owned
  // ContactProbeEngine) and AUXILIARY_MOVE; `policy`/`runtime` are used
  // directly only for TorqueEnable — CalibrationExecutionEngine has no
  // intent for it, the same split every executor in this codebase uses.
  void begin(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
            CalibrationExecutionEngine* engine,
            const actuator::CalibrationGeometryProfile* geometry,
            const actuator::GeometryProvenance* expected_provenance,
            const FullLegCalibrationConfig& config);

  bool start(const FullLegCalibrationRequest& request, const FullLegCalibrationContext& context,
            uint32_t now_ms);

  // At most one backend call per tick, exactly like every other executor in
  // this file family. primary_safe_off_verified/auxiliary_safe_off_verified
  // are read only while the corresponding SAFE_OFF-servicing phase is
  // active; ignored otherwise, matching how `telemetry`/`telemetry_available`
  // are ignored outside a monitoring phase.
  void update(const FullLegCalibrationContext& context, uint32_t now_ms,
             bool telemetry_available, const actuator::TelemetrySample& telemetry,
             bool primary_safe_off_verified, bool auxiliary_safe_off_verified);

  void abort();

  const FullLegCalibrationStatus& status() const { return status_; }
  // The owned ContactProbeEngine's own status (phase, failure, pass, coarse/
  // fine stall ticks) for the side that ran last: the MIN side while/after
  // it runs, the MAX side once that has started. Read-only diagnostics - it
  // is what distinguishes a MOTION_TIMEOUT from a NO_CONTACT_DETECTED behind
  // an UPPER_*_PROBE_FAILED on hardware.
  const ContactProbeStatus& probeStatus() const { return probe_.status(); }
  bool active() const {
    return status_.phase != FullLegCalibrationPhase::IDLE &&
          status_.phase != FullLegCalibrationPhase::COMPLETE &&
          status_.phase != FullLegCalibrationPhase::FAILED;
  }

  // Valid from the moment start() returns true, including after a terminal
  // phase — same convention as FirstMotionExecutor::busId(). 0 (never a
  // valid ST3215 id) before the first start(). auxiliaryBusId() becomes
  // meaningful once the AUX phases begin; it is still safe to read (and
  // safe to SAFE_OFF) before that, since 0 addresses nothing.
  uint8_t primaryBusId() const { return request_.probe_bus_id; }
  // The endpoint (leg, joint) the current/last run probes: what
  // SafeActuatorPolicy's parked-auxiliary context must name while
  // auxiliaryParked() is true.
  Leg endpointLeg() const { return request_.endpoint_leg; }
  JointKind endpointJoint() const { return request_.endpoint_joint; }
  // 0 when no auxiliary is required (never an address — see file comment).
  uint8_t auxiliaryBusId() const {
    return request_.auxiliary_required ? request_.auxiliary_bus_id : 0;
  }
  bool auxiliaryRequired() const { return request_.auxiliary_required; }

  // True only while the MAX-side probe is actually running WITH an auxiliary
  // parked — the one window SafeActuatorPolicy's bootstrap context must
  // report the auxiliary as parked. Always false when no auxiliary is
  // required (a NOT_NEEDED direct path must be validated with nothing
  // parked). False before and after: SAFE_OFF never consults it either way.
  bool auxiliaryParked() const {
    return request_.auxiliary_required &&
           status_.phase == FullLegCalibrationPhase::UPPER_MAX_PROBE;
  }

  bool primarySafeOffPending() const {
    return status_.phase == FullLegCalibrationPhase::UPPER_MIN_SAFE_OFF ||
          status_.phase == FullLegCalibrationPhase::FINAL_SAFE_OFF;
  }
  bool auxiliarySafeOffPending() const {
    return request_.auxiliary_required &&
           status_.phase == FullLegCalibrationPhase::FINAL_SAFE_OFF;
  }

  // Valid once status().phase == COMPLETE (or, for whichever side actually
  // finished, immediately after that side's probe terminates). has_measurement
  // is false until then — the same "false means no evidence" rule
  // ContactEvidence itself documents.
  const ContactEvidence& minSideEvidence() const { return min_evidence_; }
  const ContactEvidence& maxSideEvidence() const { return max_evidence_; }

 private:
  void routeToSafeOffAfterLoss();
  // Starts the shared probe_ for the MAX side. False leaves probe_ inactive
  // and the failure/phase untouched — the caller records UPPER_MAX_PROBE_FAILED.
  bool startMaxProbe(const FullLegCalibrationContext& context, uint32_t now_ms);
  void handleMinProbeTerminal();
  void handleMaxProbeTerminal();
  void stepAuxTorqueEnable(const FullLegCalibrationContext& context);
  void stepAuxMovePending(const FullLegCalibrationContext& context, uint32_t now_ms);
  void stepAuxMoveMonitoring(const FullLegCalibrationContext& context, uint32_t now_ms,
                            bool telemetry_available, const actuator::TelemetrySample& telemetry);

  actuator::SafeActuatorPolicy* policy_ = nullptr;
  actuator::ActuatorRuntime* runtime_ = nullptr;
  CalibrationExecutionEngine* engine_ = nullptr;
  const actuator::CalibrationGeometryProfile* geometry_ = nullptr;
  const actuator::GeometryProvenance* expected_provenance_ = nullptr;
  FullLegCalibrationConfig config_{};

  FullLegCalibrationRequest request_{};
  FullLegCalibrationStatus status_{};
  ContactProbeEngine probe_;  // reused sequentially for MIN, then MAX
  actuator::MotionDeadmanMonitor aux_deadman_;
  ContactEvidence min_evidence_{};
  ContactEvidence max_evidence_{};
};

const char* toString(FullLegCalibrationPhase phase);
const char* toString(FullLegCalibrationFailure failure);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_EXECUTOR_H
