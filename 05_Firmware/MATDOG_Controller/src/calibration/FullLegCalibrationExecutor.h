#ifndef MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_EXECUTOR_H
#define MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_EXECUTOR_H

#include <stdint.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationSequencePlan.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../actuator/MotionDeadman.h"
#include "CalibrationDomain.h"
#include "CalibrationExecutionEngine.h"
#include "ContactProbeEngine.h"

// FULL CALIBRATION OF ONE LEG = SIX MECHANICAL CONTACTS.
//
// The LF V25 hardware-oracle state machine (matdog.rs run_lf_state_machine,
// the only MATDOG calibrator ever validated on hardware: 58/58 steps, 6/6 LF
// contacts), generalized to LF, RF, RH and LH:
//
//   PREFLIGHT         every leg joint answers, torque OFF, fresh telemetry
//   INITIAL_RECOVERY  every leg joint of the robot actively returned to its
//                     promoted q0, ONE AT A TIME (V25 normalize_all_matdog_
//                     joints_to_q0: prime at present, TorqueLimit 500, torque
//                     on, move to q0, settle 4 samples / 400 ms within 10
//                     ticks at |speed| <= 4, torque off), then all verified
//                     at q0 with torque off. EVERY one of the twelve is
//                     commanded, including a joint already within 10 ticks
//                     of q0 (operator requirement 2026-09-30: a capture next
//                     to the pose is not a controller-verified baseline; V25
//                     skipped such joints). A joint farther than 64 ticks
//                     from q0 is NOT moved: the run fails for repositioning.
//   PARKING           front legs: the rear UPPER to its park pose, HELD for
//                     the whole leg (V25 parked LH UPPER for all of LF)
//   UPPER_MIN         HIP and LOWER energized and HELD at q0, UPPER energized,
//                     then the staged V25 search (ContactProbeEngine)
//   UPPER_MAX         the same search from the MIN contact, no SAFE_OFF between
//   UPPER_HORIZONTAL  UPPER from its MAX contact to the plan pose, HELD
//   LOWER_MIN/MAX     LOWER searched with HIP@q0 and UPPER@horizontal HELD
//   LOWER_FOLDED      LOWER from its MAX contact to folded, HELD; UPPER to the
//                     HIP MIN clearance pose where that differs
//   HIP_MIN           HIP searched with UPPER@clearance and LOWER@folded HELD
//   HIP_MAX           where V25's per-side clearance differs: HIP to q0, UPPER
//                     to the HIP MAX pose; then the search
//   DIAGNOSTICS       six witnesses, ordering, V25 affine span/q0 gates
//   RETURN_HIP        HIP to q0, HELD
//   RETURN_LOWER_HELD LOWER to q0, HELD
//   RETURN_UPPER      UPPER to q0, HELD
//   RESTORE_PARKING   the rear UPPER back to q0
//   CLEANUP           verified SAFE_OFF of every leg joint of the robot
//   TORQUE_OFF        torque off everywhere, the leg at rest near q0
//
// Every prerequisite pose and target is resolved by the caller from the
// current promoted q0 transforms, the Geometry V5 URDF domain and the
// geometry-validated CalibrationSequencePlan (FullLegCalibrationPlan.h); this
// class carries no leg-specific constant.
//
// NOTHING IS LEFT LIMP THAT THE ORACLE HELD, AND NOTHING IS UNWATCHED
// -------------------------------------------------------------------
// Every tick the caller hands in fresh calibration telemetry for the buses
// telemetryRequest() names: the joint being moved/probed, EVERY held joint,
// and one round-robin bystander. A held joint fails the run at once when it
// is not torque-on, its GoalPosition or RAM TorqueLimit readback changed, it
// drifted more than 10 ticks (V25 STATIC_TOLERANCE_TICKS), it moves faster
// than 40 raw on two consecutive samples, its status byte is set, its
// current reaches 200 or its temperature exceeds 70 C, or its telemetry is
// older than 3 s. A limp participant must stay torque-off within 32 ticks of
// q0 (V25 passive corridor); every other leg joint torque-off within 16 ticks
// of where the session left it (V25 non-participating drift).
//
// SAFE_OFF IS OUTSIDE THIS LAYER
// ------------------------------
// No ServoBus reference exists here. Every SAFE_OFF this sequence needs - the
// per-joint torque-off during INITIAL_RECOVERY and the verified SAFE_OFF of
// every leg joint on EVERY exit, success or failure (V25: "global torque OFF
// must be verified on success and every failure path") - is requested through
// safeOffRequest() and confirmed by the caller's independent ServoBus reads.
//
// A safety failure (current, temperature, status, torque, drift, stale or
// lost telemetry, a refused or uncertain write, lost permit/session/authority,
// operator abort) goes straight to CLEANUP: no further motion of any kind.
// A diagnostics rejection is not a safety failure: the leg is returned to q0
// through the reviewed RETURN phases first, then the run is FAILED.
//
// At most ONE backend write per tick. Pure: no Arduino, no ServoBus, no clock.

namespace matdog {
namespace calibration {

// --- LF V25 hardware-oracle constants (matdog.rs), ported unchanged ----------
constexpr uint16_t kSequenceStaticToleranceTicks = 10;     // STATIC_TOLERANCE_TICKS
constexpr uint16_t kSequenceSettleMaxSpeedRaw = 4;         // LF_HELD_MAX_SPEED_RAW
constexpr uint8_t kSequenceSettledSamples = 4;             // LF_TRANSITION_SETTLED_SAMPLES
constexpr uint32_t kSequenceSettleWindowMs = 400;          // LF_TRANSITION_SETTLE_WINDOW
constexpr uint16_t kSequencePassiveCorridorTicks = 32;     // PROBE_PASSIVE_RESTORE_DRIFT_TICKS
constexpr uint16_t kSequenceBystanderDriftTicks = 16;      // NON_PARTICIPATING_MAX_DRIFT_TICKS
constexpr uint16_t kSequenceRestToleranceTicks = 16;       // PROBE_HOME_TOLERANCE_TICKS
constexpr uint32_t kSequenceMotionTimeoutMs = 12000;       // MOTION_TIMEOUT
constexpr uint32_t kSequenceMaxTelemetryAgeMs = 3000;      // MAX_TELEMETRY_AGE
constexpr uint16_t kSequenceMinTicksPerSecond = 80;        // MIN_EXPECTED_MOTION_TICKS_PER_SECOND
constexpr uint16_t kAffineScaleMinPermille = 850;          // AFFINE_SCALE_MIN_PERMILLE
constexpr uint16_t kAffineScaleMaxPermille = 1150;         // AFFINE_SCALE_MAX_PERMILLE
constexpr uint16_t kModelZeroMaxShiftTicks = 96;           // MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS
constexpr uint16_t kModelZeroEndpointConsistencyTicks = 24;  // diagnostic only, as in V25
// Speed is a SETTLING criterion only, exactly as in V25: LF_HELD_MAX_SPEED_RAW
// (kSequenceSettleMaxSpeedRaw) gates the StableTargetGate that promotes a
// moved joint to held, and INITIAL_RECOVERY's settle. An ALREADY-held joint
// is supervised as V25 validate_lf_role_observation(ActivelyHeld): fresh
// telemetry, status, hard current, temperature, TorqueEnable / TorqueLimit /
// GoalPosition readback, and position within kSequenceStaticToleranceTicks of
// its held target. Its instantaneous speed alone never aborts the run.
//
// DIAGNOSTIC ONLY, never an abort: an already-held joint reporting a speed
// above this is logged (CALIBRATION_HELD_SPEED_TRANSIENT), once per rising
// edge per joint and at most kHeldSpeedTransientEventCap times per run. The
// figure is the retired post-V25 D5 abort level, so the transient that
// false-aborted LF UPPER MAX on 2026-09-30 (held position within 10 ticks) is
// now visible as evidence instead.
constexpr uint16_t kHeldSpeedTransientReportRaw = 40;
constexpr uint8_t kHeldSpeedTransientEventCap = 32;

constexpr uint8_t kFullLegContactCount = 6;                // 3 joints x MIN/MAX
constexpr uint8_t kFullLegPopulation = kLegServoSlotCount;   // every leg joint of the robot
constexpr uint8_t kFullLegMaxTelemetry = 6;                // buses read in one tick

enum class FullLegStep : uint8_t {
  IDLE = 0,
  INSPECT,          // needs a fresh sample of the op's joint
  PRIME,            // GoalPosition := present (torque off)
  TORQUE_LIMIT,     // RAM TorqueLimit := 500
  TORQUE_ON,        // TorqueEnable
  VERIFY_ENERGIZED, // torque on, limit 500, goal == prime, settled
  MOVE_WRITE,       // GoalPosition := plan target
  MOVE_MONITOR,     // settle gate / timeout
  PROBE,            // owned ContactProbeEngine
  SAFE_OFF_ONE,     // INITIAL_RECOVERY: torque off the recovered joint
  VERIFY_REST,      // sample-by-sample checks (all at q0 / leg at rest)
  SAFE_OFF_ALL,     // CLEANUP
  COMPLETE,
  FAILED,
};

enum class FullLegFailure : uint8_t {
  NONE = 0,
  REJECT_PRECONDITIONS,
  PREFLIGHT_TORQUE_ON,            // a leg joint was torque-on at entry
  INITIAL_RECOVERY_OUT_OF_RANGE,  // > 64 ticks from q0: reposition by hand
  INITIAL_RECOVERY_NOT_SETTLED,   // recovered joint not at q0 / not torque-off
  PRIME_REJECTED,
  TORQUE_LIMIT_REJECTED,
  TORQUE_ENABLE_REJECTED,
  WRITE_UNCERTAIN,                // any write the backend could not verify
  ENERGIZE_NOT_VERIFIED,          // torque/limit/goal readback wrong after energizing
  MOVE_REJECTED,
  MOVE_TIMEOUT,
  MOVE_READBACK,                  // moving joint: torque off / limit / goal / status
  HELD_JOINT_DRIFT,
  HELD_JOINT_READBACK,            // held joint: torque off / limit / goal changed
  HELD_SET_MISMATCH,              // a probe's required held set is not exactly what is held
  PASSIVE_JOINT_MOVED,            // a limp participant left its corridor or got torque
  BYSTANDER_MOVED,                // a non-participating joint moved or got torque
  SERVO_STATUS_FAULT,
  HARD_CURRENT_ABORT,
  OVER_TEMPERATURE,
  STALE_TELEMETRY,
  UPPER_MIN_PROBE_FAILED,         // detail: probeStatus()
  UPPER_MAX_PROBE_FAILED,
  LOWER_MIN_PROBE_FAILED,
  LOWER_MAX_PROBE_FAILED,
  HIP_MIN_PROBE_FAILED,
  HIP_MAX_PROBE_FAILED,
  DIAGNOSTICS_REJECTED,           // returned to q0 first, then FAILED
  REST_NOT_VERIFIED,              // after SAFE_OFF the leg is not at rest near q0
  DYNAMIC_PREREQUISITE_LOST,      // permit / session / authority / mode
  PHASE_REPORT_REJECTED,          // the session refused the V25 phase order
  OPERATOR_ABORT,
};

struct FullLegCalibrationConfig {
  // The contact search's own 96-tick backoff.
  actuator::MotionDeadmanConfig probe_backoff_deadman{};
};

// One joint the run may touch.
struct FullLegJoint {
  JointIdentity identity{};
  uint8_t bus_id = 0;     // 0 = none
  uint16_t q0_tick = 0;   // promoted, current boot
};

// Everything the run needs, resolved once by resolveFullLegPlan().
struct FullLegCalibrationRequest {
  Leg leg = Leg::LF;
  // Indexed by static_cast<uint8_t>(JointKind): HIP, UPPER, LOWER.
  FullLegJoint joint[kJointKindCount];
  // [joint][side]: the staged-search corridor (canonical contact, URDF limit,
  // entry = limit - 64, guard = limit + 64, probe sign, q0).
  actuator::CalibrationSearchCorridor corridor[kJointKindCount][kContactSideCount];
  // URDF domain of the leg's joints (diagnostics' model endpoints, as V25).
  actuator::MicroRad urdf_lower[kJointKindCount] = {0, 0, 0};
  actuator::MicroRad urdf_upper[kJointKindCount] = {0, 0, 0};
  int8_t direction[kJointKindCount] = {0, 0, 0};

  // Prerequisite poses: URDF q (the command) and the resolved tick (what the
  // held joint's GoalPosition must read back).
  actuator::MicroRad upper_for_lower_urad = 0;    uint16_t upper_for_lower_tick = 0;
  actuator::MicroRad upper_for_hip_min_urad = 0;  uint16_t upper_for_hip_min_tick = 0;
  actuator::MicroRad upper_for_hip_max_urad = 0;  uint16_t upper_for_hip_max_tick = 0;
  actuator::MicroRad lower_folded_urad = 0;       uint16_t lower_folded_tick = 0;

  bool has_rear_park = false;
  FullLegJoint park{};
  actuator::MicroRad park_target_urad = 0;        uint16_t park_target_tick = 0;

  // Every leg joint of the robot: INITIAL_RECOVERY and bystander monitoring.
  uint8_t population_count = 0;
  FullLegJoint population[kFullLegPopulation];

  uint16_t repeatability_tolerance_ticks = 0;  // never defaulted
  uint16_t torque_limit = 0;                   // RAM TorqueLimit readback; never defaulted

  // INITIAL RECOVERY ONLY: PREFLIGHT -> INITIAL_RECOVERY -> TORQUE_OFF (every
  // leg joint SAFE_OFF and verified at rest near q0), no probe, no contact.
  // The controller-verified q0 baseline the operator requires after a fresh
  // q0 promotion, before any leg is calibrated. Same policy phase table, same
  // SAFE_OFF path; COMPLETE only with all twelve joints actively recovered.
  bool recovery_only = false;
};

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

// This tick's samples, keyed by bus id. A bus missing from the frame had no
// read attempted this tick.
struct FullLegTelemetryFrame {
  uint8_t count = 0;
  uint8_t bus_id[kFullLegMaxTelemetry] = {0};
  actuator::TelemetrySample sample[kFullLegMaxTelemetry];
  const actuator::TelemetrySample* find(uint8_t bus) const;
  bool add(uint8_t bus, const actuator::TelemetrySample& s);
};

// The caller's independent SAFE_OFF confirmations for the buses
// safeOffRequest() named last tick.
struct FullLegSafeOffFrame {
  uint8_t count = 0;
  uint8_t bus_id[kFullLegPopulation] = {0};
  bool verified[kFullLegPopulation] = {false};
  bool verifiedFor(uint8_t bus) const;
  bool add(uint8_t bus, bool ok);
};

// V25 per-joint diagnostics (matdog.rs derive_joint_evidence).
struct FullLegJointDiagnostics {
  bool evaluated = false;
  uint16_t min_contact_tick = 0;         // midpoint of the two fine contacts
  uint16_t max_contact_tick = 0;
  bool ordered = false;                  // MAX lies beyond MIN in the joint's +q direction
  uint16_t expected_span_ticks = 0;      // URDF span
  uint16_t measured_span_ticks = 0;
  uint16_t scale_permille = 0;
  uint16_t affine_zero_tick = 0;
  uint16_t affine_shift_from_q0_ticks = 0;
  uint16_t fixed_endpoint_disagreement_ticks = 0;  // diagnostic only
  bool accepted = false;                 // ordered && scale in band && shift <= 96
};

FullLegJointDiagnostics deriveFullLegJointDiagnostics(const FullLegCalibrationRequest& request,
                                                      JointKind joint,
                                                      const ContactEvidence& min_side,
                                                      const ContactEvidence& max_side);

// One held joint as the run observed it: the record of a held-role failure,
// or of a diagnostic speed transient. Evidence only - no decision reads it.
struct FullLegHeldObservation {
  bool valid = false;
  FullLegFailure failure = FullLegFailure::NONE;  // NONE: a speed transient (not a failure)
  CalibrationPhase phase = CalibrationPhase::PREFLIGHT;
  uint8_t bus_id = 0;
  JointIdentity identity{};
  uint16_t target_tick = 0;      // the held target
  // false: no usable sample this tick (read failed / stale); the values below
  // are then the joint's last usable sample, sample_age_ms old.
  bool sample_usable = false;
  bool has_sample = false;       // false: no usable sample since it was held
  uint32_t sample_age_ms = 0;
  int32_t present_position = -1;
  int32_t position_error = 0;    // |present - target|
  int32_t present_speed = -1;    // magnitude, raw
  int32_t goal_position = -1;
  int32_t torque_enable = -1;
  int32_t torque_limit = -1;
  int32_t present_current = -1;  // magnitude, raw
  int32_t present_temperature = -1;
  int32_t servo_status = -1;
  // The joint the run was driving at that moment (the probe, or a move).
  uint8_t active_bus = 0;
  JointKind active_joint = JointKind::HIP;
  bool active_is_probe = false;
  ContactSide active_side = ContactSide::MIN_SIDE;
  uint16_t active_target_tick = 0;
  int32_t active_position = -1;
};

struct FullLegCalibrationStatus {
  CalibrationPhase phase = CalibrationPhase::PREFLIGHT;
  FullLegStep step = FullLegStep::IDLE;
  FullLegFailure failure = FullLegFailure::NONE;
  CalibrationPhase failed_phase = CalibrationPhase::PREFLIGHT;
  JointKind op_joint = JointKind::HIP;   // the joint the current op acts on
  uint8_t op_bus = 0;
  uint16_t op_target_tick = 0;
  uint8_t held_count = 0;
  uint8_t contacts_accepted = 0;         // 0..6
  uint8_t recovered_joints = 0;          // INITIAL_RECOVERY moves actually made
  uint32_t phase_changes = 0;
  actuator::WriteDecision last_policy_decision = actuator::WriteDecision::REJECT_NO_ARBITER;
};

class FullLegCalibrationExecutor {
 public:
  void begin(actuator::SafeActuatorPolicy* policy, actuator::ActuatorRuntime* runtime,
             CalibrationExecutionEngine* engine,
             const actuator::CalibrationGeometryProfile* geometry,
             const actuator::GeometryProvenance* expected_provenance,
             const FullLegCalibrationConfig& config);

  bool start(const FullLegCalibrationRequest& request, const FullLegCalibrationContext& context,
             uint32_t now_ms);

  // At most one backend write per call.
  void update(const FullLegCalibrationContext& context, uint32_t now_ms,
              const FullLegTelemetryFrame& telemetry, const FullLegSafeOffFrame& safe_off);

  void abort();
  // The session refused this run's phase report: the order is broken.
  void phaseReportRejected();

  // What the caller must do BEFORE the next update(): read these buses...
  uint8_t telemetryRequest(uint8_t* buses, uint8_t max) const;
  // ...and SAFE_OFF these, confirming each by independent readback.
  uint8_t safeOffRequest(uint8_t* buses, uint8_t max) const;

  const FullLegCalibrationStatus& status() const { return status_; }
  const FullLegCalibrationRequest& request() const { return request_; }
  const ContactProbeStatus& probeStatus() const { return probe_.status(); }
  const ContactProbeRequest& probeRequest() const { return probe_.request(); }
  bool active() const {
    return status_.step != FullLegStep::IDLE && status_.step != FullLegStep::COMPLETE &&
           status_.step != FullLegStep::FAILED;
  }
  Leg leg() const { return request_.leg; }

  // The bootstrap-context view SafeActuatorPolicy needs (Controller relays it).
  bool sequenceActive() const { return active(); }
  CalibrationPhase sequencePhase() const { return status_.phase; }
  bool prerequisitesVerified() const { return prerequisites_verified_; }

  // [joint][side], has_measurement false until that probe COMPLETEs.
  const ContactEvidence& contact(JointKind joint, ContactSide side) const {
    return contacts_[static_cast<uint8_t>(joint)][static_cast<uint8_t>(side)];
  }
  const FullLegJointDiagnostics& diagnostics(JointKind joint) const {
    return diagnostics_[static_cast<uint8_t>(joint)];
  }
  bool diagnosticsAccepted() const { return diagnostics_accepted_; }
  // The bus the owned probe is (or was last) driving.
  uint8_t probeBusId() const { return probe_.request().bus_id; }

  // Held-joint evidence (print only). The held-role failure that ended the
  // run, if one did (valid == false otherwise)...
  const FullLegHeldObservation& heldRoleFailure() const { return held_role_failure_; }
  // ...and the diagnostic speed transients the LAST update() recorded (0..4;
  // none once kHeldSpeedTransientEventCap were recorded this run).
  uint8_t heldSpeedTransientsThisTick() const { return held_transients_tick_count_; }
  const FullLegHeldObservation& heldSpeedTransient(uint8_t i) const {
    return held_transients_tick_[i < kHeldTransientSlots ? i : 0];
  }
  // Every transient rising edge this run, including those past the cap.
  uint16_t heldSpeedTransientTotal() const { return held_transient_total_; }

 private:
  // One primitive of a phase program.
  enum class Op : uint8_t {
    ENERGIZE,       // prime at present, TorqueLimit, TorqueEnable, verify
    MOVE,           // to target, settle
    HOLD,           // mark held at the last move target
    RELEASE,        // unmark held; the joint becomes the active one
    PROBE,          // the phase's endpoint search
    DIAGNOSE,
    RECOVER_ALL,    // INITIAL_RECOVERY over the whole population
    VERIFY_REST,
    SAFE_OFF_ALL,
  };
  struct ProgramStep {
    Op op = Op::DIAGNOSE;
    uint8_t slot = 0;          // kSlotHip/Upper/Lower/Park
    uint16_t target_tick = 0;  // MOVE
    actuator::MicroRad target_urad = 0;
  };
  static constexpr uint8_t kSlotCount = 4;
  static constexpr uint8_t kSlotPark = 3;
  static constexpr uint8_t kHeldTransientSlots = kSlotCount;
  static constexpr uint8_t kMaxProgram = 12;

  struct SlotState {
    bool energized = false;
    bool held = false;
    uint16_t target_tick = 0;
    bool has_good = false;
    uint32_t last_good_ms = 0;
    // Evidence only: the last usable sample while held, and whether the
    // joint is inside a speed transient (for the rising-edge diagnostic).
    bool has_last_sample = false;
    actuator::TelemetrySample last_sample{};
    bool speed_transient = false;
  };
  struct PopulationState {
    bool has_good = false;
    uint32_t last_good_ms = 0;
    uint16_t entry_tick = 0;
    bool entry_known = false;
  };

  void enterPhase(CalibrationPhase phase, uint32_t now_ms);
  void buildProgram();
  void addStep(Op op, uint8_t slot, uint16_t tick = 0, actuator::MicroRad urad = 0);
  void advanceProgram(uint32_t now_ms);
  void nextPhase(uint32_t now_ms);
  void fail(FullLegFailure failure);
  // The round-robin watch target: a population joint neither energized nor
  // otherwise monitored this tick.
  bool watchIndex(uint8_t* out) const;
  // Held/bystander monitoring is off while every joint is being made limp.
  bool monitoringActive() const;

  bool continuationOk(const FullLegCalibrationContext& context) const;
  const FullLegJoint& slotJoint(uint8_t slot) const;
  bool sampleUsable(const actuator::TelemetrySample* s) const;
  // Status / hard current / temperature, as V25 checks them on every role.
  FullLegFailure commonSafetyFailure(const actuator::TelemetrySample& s) const;
  bool commonSafety(const actuator::TelemetrySample& s);
  bool monitorHeld(uint32_t now_ms, const FullLegTelemetryFrame& telemetry);
  FullLegHeldObservation observeHeld(uint8_t slot, const actuator::TelemetrySample* sample,
                                     uint32_t now_ms, const FullLegTelemetryFrame& telemetry) const;
  // Latches the held-role record (observed BEFORE fail() moves the run to
  // TORQUE_OFF and aborts the probe), then fails the run with `failure`.
  void failHeldRole(FullLegHeldObservation observation, FullLegFailure failure);
  bool monitorBystander(uint32_t now_ms, const FullLegTelemetryFrame& telemetry);
  bool isParticipantBus(uint8_t bus, uint8_t* slot) const;
  void recomputePrerequisites();

  void stepEnergize(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                    const FullLegTelemetryFrame& telemetry);
  void stepMove(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                const FullLegTelemetryFrame& telemetry);
  void stepProbe(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                 const FullLegTelemetryFrame& telemetry);
  void stepRecover(const FullLegCalibrationContext& ctx, uint32_t now_ms,
                   const FullLegTelemetryFrame& telemetry, const FullLegSafeOffFrame& safe_off);
  void stepVerifyRest(uint32_t now_ms, const FullLegTelemetryFrame& telemetry);
  void stepSafeOffAll(const FullLegSafeOffFrame& safe_off, uint32_t now_ms);
  void runDiagnostics();

  // The single write primitives (each is ONE backend call).
  bool writePrime(const FullLegCalibrationContext& ctx, const FullLegJoint& j, uint16_t tick);
  bool writeTorqueLimit(const FullLegCalibrationContext& ctx, const FullLegJoint& j);
  bool writeTorqueOn(const FullLegCalibrationContext& ctx, const FullLegJoint& j);
  bool writeMove(const FullLegCalibrationContext& ctx, const FullLegJoint& j,
                 actuator::MicroRad target_urad);
  bool routeResult(actuator::ExecuteResult result, FullLegFailure rejected);

  actuator::SafeActuatorPolicy* policy_ = nullptr;
  actuator::ActuatorRuntime* runtime_ = nullptr;
  CalibrationExecutionEngine* engine_ = nullptr;
  const actuator::CalibrationGeometryProfile* geometry_ = nullptr;
  const actuator::GeometryProvenance* expected_provenance_ = nullptr;
  FullLegCalibrationConfig config_{};

  FullLegCalibrationRequest request_{};
  FullLegCalibrationStatus status_{};
  ContactProbeEngine probe_;
  ContactEvidence contacts_[kJointKindCount][kContactSideCount];
  FullLegJointDiagnostics diagnostics_[kJointKindCount];
  bool diagnostics_accepted_ = false;
  bool prerequisites_verified_ = false;
  FullLegHeldObservation held_role_failure_{};
  FullLegHeldObservation held_transients_tick_[kHeldTransientSlots];
  uint8_t held_transients_tick_count_ = 0;
  uint16_t held_transient_total_ = 0;
  uint8_t held_transients_recorded_ = 0;

  SlotState slot_[kSlotCount];
  PopulationState population_[kFullLegPopulation];
  ProgramStep program_[kMaxProgram];
  uint8_t program_count_ = 0;
  uint8_t program_index_ = 0;

  // The op in progress.
  uint8_t sub_ = 0;                  // sub-state within an op
  uint32_t op_started_ms_ = 0;
  uint32_t op_deadline_ms_ = 0;
  uint16_t op_prime_tick_ = 0;
  uint8_t settled_samples_ = 0;
  uint32_t settle_first_ms_ = 0;
  // INITIAL_RECOVERY cursor.
  uint8_t recover_index_ = 0;
  uint8_t recover_bus_ = 0;
  // Round-robin bystander/limp-participant cursor.
  uint8_t watch_index_ = 0;
  // CLEANUP bookkeeping.
  bool cleanup_ = false;
  bool return_after_diagnostics_failure_ = false;
  uint16_t safe_off_verified_mask_ = 0;  // bit per population index
  uint32_t now_ms_ = 0;                  // the current update()'s time
};

const char* toString(FullLegStep step);
const char* toString(FullLegFailure failure);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_FULL_LEG_CALIBRATION_EXECUTOR_H
