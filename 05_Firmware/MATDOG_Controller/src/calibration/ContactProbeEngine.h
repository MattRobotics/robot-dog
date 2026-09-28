#ifndef MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H
#define MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../actuator/MotionDeadman.h"
#include "CalibrationDomain.h"
#include "CalibrationExecutionEngine.h"

// CR3 Priority 5 — the contact-probe state machine for the eight executable
// UPPER endpoints, prepared and offline-tested but never executed against
// real hardware by this session.
//
// Pure: <stdint.h> plus already-pure MATDOG units. No Arduino, no ServoBus,
// no clock of its own — every tick's "now" and every telemetry sample are
// supplied by the caller, the same contract as FirstMotionExecutor.h.
//
// WHAT "POSSIBLE CONTACT" MEANS HERE, AND WHY
// ---------------------------------------------
// No MATDOG bench characterization of ST3215 current/load thresholds exists
// (see servo::ServoBus::RuntimeState::present_current's own comment), so
// this engine does not invent one — "do not fabricate measurements" applies
// to thresholds as much as to results. The signal it uses instead is the one
// already proven and already implemented: sustained position-progress stall
// under continued commanded torque and healthy communication
// (actuator::MotionDeadmanMonitor, CR3 Priority 4) — the standard,
// hardware-agnostic obstruction signal for a closed-loop position-controlled
// actuator. The SAME verdict (STALLED) that FirstMotionExecutor treats as a
// failure requiring SAFE_OFF is, in this engine and only during an approach
// phase, the expected and desired outcome.
//
// A single stall is never trusted alone. Two independent approaches must
// both stall within a caller-supplied repeatability tolerance of each other
// (see CalibrationDomain.h's ContactWitness/makeContactWitness — the exact
// vocabulary the historical LF V25 witness gate used, reused here rather
// than reinvented) before ContactEvidence is produced. Arriving at the
// commanded boundary on EITHER pass with no stall ever observed is not
// contact evidence either — see stepApproachMonitoring().
//
// WHAT THIS DOES NOT DECIDE
// ---------------------------
// The caller supplies BOTH the approach target (the endpoint's own
// geometric contact, already policy-enforced not to be exceeded — see
// SafeActuatorPolicy::evaluateEndpointPlan()) and the backoff target, as
// explicit URDF-frame (MicroRad) values. This engine does not compute a
// backoff distance itself — that is a reviewed, project-level safety margin
// choice, not arithmetic this file should invent.
//
// SAFE_OFF IS OUTSIDE THIS LAYER, STRUCTURALLY
// ----------------------------------------------
// Same structural absence as FirstMotionExecutor.h: no ServoBus reference
// exists here, so this class cannot call safeOff() even by accident. Every
// terminal state that leaves torque possibly applied reports
// ContactProbeState::SAFE_OFF_REQUIRED; the caller is responsible for the
// real, independent SAFE_OFF.

namespace matdog {
namespace calibration {

struct ContactProbeRequest {
  JointIdentity joint{};
  uint8_t bus_id = 0;
  Leg endpoint_leg = Leg::LF;
  JointKind endpoint_joint = JointKind::HIP;
  ContactSide endpoint_side = ContactSide::MIN_SIDE;
  // The endpoint's own geometric contact (caller reads this from the same
  // bound CalibrationGeometryProfile the policy already enforces it
  // against) and a strictly-clear backoff target on the same side of it.
  actuator::MicroRad approach_target_urad = 0;
  actuator::MicroRad backoff_target_urad = 0;
  // The reviewed repeatability band for this endpoint — never defaulted
  // (see ContactWitness's own file comment: a band of zero tolerance is
  // never assumed valid, and neither is any other unreviewed value).
  uint16_t repeatability_tolerance_ticks = 0;
};

enum class ContactProbePhase : uint8_t {
  IDLE                = 0,
  TORQUE_ENABLE_PENDING = 1,
  APPROACH_PENDING    = 2,  // used for both the first and second approach
  APPROACH_MONITORING = 3,
  BACKOFF_PENDING     = 4,
  BACKOFF_MONITORING  = 5,
  // --- terminal states ----------------------------------------------------
  COMPLETE            = 6,  // repeatability confirmed; evidence() is ready
  FAILED_NO_MOTION    = 7,  // ended before torque was ever VERIFIED applied
  SAFE_OFF_REQUIRED   = 8,  // torque was VERIFIED applied and the attempt did
                             // not reach a clean COMPLETE — caller MUST
                             // invoke the real, independent SAFE_OFF now
};

enum class ContactProbeFailure : uint8_t {
  NONE                             = 0,
  REJECT_PRECONDITIONS             = 1,
  TORQUE_ENABLE_REJECTED           = 2,
  TORQUE_ENABLE_UNCERTAIN          = 3,
  REJECT_TARGET_RESOLUTION         = 4,
  COMMAND_REJECTED                 = 5,  // policy/engine refused the approach or backoff
  COMMAND_UNCERTAIN                = 6,
  NO_CONTACT_DETECTED              = 7,  // arrived at the commanded boundary
                                          // without ever stalling
  REPEATABILITY_FAILED             = 8,  // both passes stalled, but not consistently
  STALE_TELEMETRY                  = 9,
  COMMUNICATION_LOST                = 10,
  TORQUE_UNEXPECTEDLY_OFF           = 11,
  UNEXPECTED_STALL_DURING_BACKOFF   = 12,
  MOTION_TIMEOUT                     = 13,
  OPERATOR_ABORT                     = 14,
};

struct ContactProbeConfig {
  actuator::MotionDeadmanConfig approach_deadman{};
  actuator::MotionDeadmanConfig backoff_deadman{};
};

struct ContactProbeStatus {
  ContactProbePhase phase = ContactProbePhase::IDLE;
  ContactProbeFailure failure = ContactProbeFailure::NONE;
  uint8_t pass = 0;  // 1 during/after the first approach, 2 during/after the second
  uint16_t coarse_tick = 0;  // first-pass stall position, once observed
  uint16_t fine_tick_1 = 0;  // second-pass stall position, once observed
  actuator::WriteDecision last_policy_decision = actuator::WriteDecision::REJECT_NO_ARBITER;
};

// Mirrors FirstMotionExecutor's context shape exactly.
struct ContactProbeContext {
  bool session_active = false;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  core::AuthorityLease lease{};
  core::OperatingMode mode = core::OperatingMode::MAINTENANCE;
};

class ContactProbeEngine {
 public:
  // No pointer is owned; all may be nullptr, which is a refusal, never a
  // permission. `engine` drives the actual approach/backoff commands
  // (reusing CalibrationExecutionEngine's already-reviewed CONTACT_PROBE
  // endpoint-plan/parking checks in full); `policy`/`runtime` are used
  // directly only for the one step CalibrationExecutionEngine has no intent
  // for, TORQUE_ENABLE — exactly the same split FirstMotionExecutor.h uses,
  // for the same reason.
  void begin(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
            CalibrationExecutionEngine* engine,
            const actuator::CalibrationGeometryProfile* geometry,
            const actuator::GeometryProvenance* expected_provenance,
            const ContactProbeConfig& config);

  bool start(const ContactProbeRequest& request, const ContactProbeContext& context,
            uint32_t now_ms);

  // At most one backend call per tick — see FirstMotionExecutor::update()'s
  // own comment; the same discipline applies here.
  void update(const ContactProbeContext& context, uint32_t now_ms, bool telemetry_available,
             const actuator::TelemetrySample& telemetry);

  void abort();

  const ContactProbeStatus& status() const { return status_; }
  bool active() const {
    return status_.phase != ContactProbePhase::IDLE &&
          status_.phase != ContactProbePhase::COMPLETE &&
          status_.phase != ContactProbePhase::FAILED_NO_MOTION &&
          status_.phase != ContactProbePhase::SAFE_OFF_REQUIRED;
  }

  // Valid only once status().phase == COMPLETE. Provenance (key/origin) is
  // the caller's to stamp — this class knows the endpoint identity but not
  // which session/leg it is recording evidence for.
  ContactWitness witness() const;

 private:
  void finish(ContactProbePhase phase, ContactProbeFailure failure,
             actuator::WriteDecision decision);
  void stepTorqueEnable(const ContactProbeContext& context, uint32_t now_ms);
  void stepApproachPending(const ContactProbeContext& context, uint32_t now_ms);
  void stepApproachMonitoring(uint32_t now_ms, bool telemetry_available,
                             const actuator::TelemetrySample& telemetry);
  void stepBackoffPending(const ContactProbeContext& context, uint32_t now_ms);
  void stepBackoffMonitoring(uint32_t now_ms, bool telemetry_available,
                            const actuator::TelemetrySample& telemetry);
  bool resolveTarget(actuator::MicroRad target_urad, uint16_t* raw_tick_out) const;

  actuator::SafeActuatorPolicy* policy_ = nullptr;
  actuator::ActuatorRuntime* runtime_ = nullptr;
  CalibrationExecutionEngine* engine_ = nullptr;
  const actuator::CalibrationGeometryProfile* geometry_ = nullptr;
  const actuator::GeometryProvenance* expected_provenance_ = nullptr;
  ContactProbeConfig config_{};

  ContactProbeRequest request_{};
  ContactProbeStatus status_{};
  actuator::MotionDeadmanMonitor deadman_;
};

const char* toString(ContactProbePhase phase);
const char* toString(ContactProbeFailure failure);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CONTACT_PROBE_ENGINE_H
