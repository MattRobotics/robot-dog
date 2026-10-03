#ifndef MATDOG_CORE_CONTROLLER_H
#define MATDOG_CORE_CONTROLLER_H

#include <Arduino.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationGeometryProfileData.h"
#include "../actuator/CalibrationSequencePlanData.h"
#include "../calibration/CalibrationExecutionEngine.h"
#include "../calibration/CalibrationManager.h"
#include "../calibration/CalibrationPersistenceService.h"
#include "../calibration/CalibrationRecordNvsBackend.h"
#include "../calibration/CalibrationMotionPermit.h"
#include "../calibration/CalibrationQ0CaptureSession.h"
#include "../calibration/FirstMotionExecutor.h"
#include "../calibration/FullLegCalibrationExecutor.h"
#include "../calibration/FullLegCalibrationFinalizer.h"
#include "../calibration/ThermalConfirmation.h"
#include "../imu/Bno085Imu.h"
#include "../network/HttpTransport.h"
#include "../network/WifiManager.h"
#include "../power/DalyBms.h"
#include "../servo/ServoBus.h"
#include "../servo/ServoBusActuatorBackend.h"
#include "../servo/ServoCensus.h"
#include "../servo/ServoPreflight.h"
#include "../status/LedRing.h"
#include "../status/LedStatusManager.h"
#include "../update/OtaManager.h"
#include "ActuatorAuthority.h"
#include "CommandRouter.h"
#include "ControllerService.h"
#include "OperatingMode.h"
#include "PowerState.h"
#include "SystemState.h"

namespace matdog {
namespace core {

// Top-level owner of every MATDOG Controller module. Owns initialization
// order, the non-blocking update loop, and health/power-state aggregation.
// Hardware logic lives in the modules below, not here or in the .ino.
class Controller {
 public:
  void begin();
  void update(uint32_t now_ms);

 private:
  void printBootBanner();
  void updateQ0Capture();
  // Every tick, before any command is processed: rebuilds
  // CalibrationMotionPermitFacts from live state, re-checks the permit
  // against them, and pushes the (possibly just-revoked) result into
  // actuator_policy_'s bootstrap context. This is what keeps the policy from
  // ever authorising a write against a stale/cached permit snapshot - see
  // 09_Logs/Development_Log for the production-composition rationale.
  void updateCalibrationMotionPermit();
  // Every tick: advances first_motion_ by at most one backend call while an
  // attempt is in progress (mirrors updateQ0Capture()'s bounded per-tick
  // discipline), and independently keeps retrying the real, ungated
  // ServoBus::safeOff() every tick once the executor reports
  // SAFE_OFF_REQUIRED, until VERIFIED_OFF - regardless of policy, session,
  // authority or permit state. See the CR3 development log for why this is
  // the one place SAFE_OFF is actually invoked from this activation path.
  void updateFirstMotion(uint32_t now_ms);
  // The 24-contact Full Calibration sequence, every tick: calibration
  // telemetry for every bus the executor names, at most one backend write,
  // the V25 phase reported to the session, then the independent SAFE_OFF of
  // every bus the executor names - forced outside policy/session/authority/
  // permit, every tick, until each reads back VERIFIED_OFF.
  void updateFullLegCalibration(uint32_t now_ms);
  // Every tick, after updateFullLegCalibration(): once the armed run's
  // executor is terminal (COMPLETE or FAILED - including a run the operator
  // aborted or whose session was lost), closes its evidence lifecycle exactly
  // once via calibration::finalizeFullLeg() and keeps the record in RAM. It
  // never touches a servo: SAFE_OFF was the executor's, verified before the
  // executor turned terminal.
  void updateFullLegFinalization();
  // Evidence line per search step / probe transition (see its definition).
  void printFullLegSearchEvent();
  // Evidence line per sequence phase / step / joint transition.
  void printFullLegSequenceEvent();
  // Held-joint evidence: the held-role failure (once) and the diagnostic
  // speed transients (bounded by the executor). Print only.
  void printFullLegHeldEvents();
  void printFullLegHeldObservation(const char* tag,
                                   const calibration::FullLegHeldObservation& o);

  servo::ServoBus servo_bus_;
  // The LF V25 runtime PresentTemperature over-limit confirmation's bus port:
  // fresh direct reads through servo_bus_ and the oracle's 50 ms wait.
  class ServoThermalReadPort : public calibration::ThermalReadPort {
   public:
    explicit ServoThermalReadPort(servo::ServoBus* bus) : bus_(bus) {}
    bool readPresentTemperatureDirect(uint8_t bus_id, int32_t* celsius) override {
      int value = -1;
      if (!bus_->readPresentTemperatureDirect(bus_id, &value)) return false;
      *celsius = value;
      return true;
    }
    void delayMs(uint32_t ms) override { delay(ms); }

   private:
    servo::ServoBus* bus_;
  };
  ServoThermalReadPort thermal_read_port_{&servo_bus_};
  calibration::ThermalConfirmationState thermal_state_[calibration::kFullLegPopulation];
  servo::ServoCensus servo_census_;  // semantic census over servo_bus_; never auto-start
  servo::ServoPreflight servo_preflight_;  // H0 leg verification; read-only, never auto-started
  imu::Bno085Imu imu_;
  power::DalyBms daly_;
  status::LedRing led_;
  // The single periodic owner of LED presentation, above led_. See
  // status/LedStatusManager.h.
  status::LedStatusManager led_status_;
  // Owns the radio; owns nothing else. It has no path to ServoBus, to
  // OperatingMode or to any actuator — see network/WifiManager.h.
  network::WifiManager wifi_;
  // Owns the OTA subsystem. In OTA-A it runs the first-boot rollback
  // lifecycle and reports state; it has no transport, so nothing can feed it
  // an image.
  update::OtaManager ota_;

  SystemState system_state_;
  PowerStateMachine power_state_;
  OperatingModeManager operating_mode_;
  // THE central arbiter of actuator write authority. Exactly one instance
  // exists, here. No other component keeps a copy of its state - they hold a
  // pointer and ask. Orthogonal to operating_mode_: that says what the
  // Controller is doing, this says who, if anyone, may write actuators.
  ActuatorAuthorityArbiter authority_;
  // The ONE calibration session manager. It holds no transport and cannot
  // command a joint; it arbitrates a session through authority_ and records
  // evidence. See calibration/CalibrationManager.h.
  calibration::CalibrationManager calibration_;
  // CR2-B read-only evidence acquisition. This is NOT a motion calibration
  // session and owns no actuator authority. It sequences the existing census,
  // preflight and ServoBus read-only runtime snapshots from Controller.
  calibration::CalibrationQ0CaptureSession q0_capture_;
  // Safe Actuator / Calibration Execution infrastructure (I4/I5), CR3-M5
  // production composition, extended by the CR3 continuation session to
  // wire the one reviewed DIRECTION_VERIFY command path all the way to
  // first_motion_ below. actuator_policy_ is bound to the REAL current
  // Geometry V5 profile and actuator_runtime_ to a REAL production backend.
  // Both are safe to be real, and calibration_.startSession()/first_motion_
  // are now safe to be REACHABLE (CR3-M5 kept them structurally
  // unreachable; this continuation instead layers explicit, independently
  // fail-closed gates in front of the one reviewed path — see
  // scripts/static_audit.py's check_actuator_infrastructure_wired_fail_closed()
  // and check_first_motion_command_wiring() for what is mechanically
  // enforced): a live session requires CURRENT population evidence
  // (Objective A), physical motion additionally requires a FRESH,
  // explicitly-granted CalibrationMotionPermit (Objective B) that only
  // @CALIBRATION MOTION PERMIT GRANT can produce, and every one of those
  // gates is re-verified from live state on every tick and on every
  // plan()/commit() call — never trusted from a cached snapshot.
  actuator::CalibrationGeometryProfile geometry_profile_;
  servo::ServoBusActuatorBackend actuator_backend_;
  actuator::SafeActuatorPolicy actuator_policy_;
  actuator::ActuatorRuntime actuator_runtime_;
  calibration::CalibrationExecutionEngine calibration_execution_;
  // CR3-M5: the session-scoped, RAM-only physical-motion permit — distinct
  // from the final hardware_motion_authorized flag, which stays false
  // throughout (CalibrationManager.h). Reset at boot.
  // motion_authorization_ is the shared, mutable bookkeeping BOTH
  // CommandRouter's @CALIBRATION MOTION PERMIT GRANT/REVOKE handlers (which
  // write it) and updateCalibrationMotionPermit()'s per-tick refresh (which
  // reads it) use — see CalibrationMotionPermit.h's own comment on why that
  // split exists. No persistent storage anywhere: both live only in RAM and
  // are zero-initialized at boot, so "do not infer authorization from a
  // previous boot/session" holds by construction, not by convention.
  calibration::CalibrationMotionPermit motion_permit_;
  calibration::CalibrationMotionAuthorizationState motion_authorization_;
  // CR3 continuation, Objective C: the first bounded-motion executor
  // (DIRECTION_VERIFY chain). Real backend, real policy, real geometry —
  // safe to wire because reaching plan()/commit()/execute() through it still
  // requires a live session (Objective A) AND a fresh, explicitly granted
  // permit (Objective B), both themselves gated on their own explicit
  // MAINTENANCE-only commands with exact confirmation phrases. See the CR3
  // development log for the full fail-closed argument.
  calibration::FirstMotionExecutor first_motion_;
  // Tracks whether the real, independent SAFE_OFF this Controller forces on
  // SAFE_OFF_REQUIRED has been verified yet — see updateFirstMotion(). Reset
  // to UNVERIFIED_NO_RESPONSE at the start of every fresh first_motion_
  // attempt, never by anything else, so a past VERIFIED_OFF can never be
  // mistaken for proof about a NEW attempt.
  servo::SafeOffResult first_motion_safe_off_result_ = servo::SafeOffResult::UNVERIFIED_NO_RESPONSE;
  // The 24-contact Full Calibration of one leg (UPPER, LOWER, HIP x MIN/MAX),
  // the LF V25 hardware-oracle state machine generalized to all four legs -
  // see FullLegCalibrationExecutor.h. Real backend, real policy, real
  // geometry, same reachability argument as first_motion_ above: only
  // @CALIBRATION FULL LEG <LF|RF|RH|LH> CONFIRM_FULL_CALIBRATION can start it, and it
  // still requires the SAME live session (Objective A) and fresh permit
  // (Objective B) as every other motion path, plus the geometry-validated
  // sequence plan for every move and probe it makes.
  calibration::FullLegCalibrationExecutor full_leg_calibration_;
  // The run in flight (armed by the FULL LEG command once the executor
  // accepted it) and the RAM-only record of every leg run in this power-up.
  // Neither is persisted: a reboot starts with an empty store.
  calibration::FullLegRunState full_leg_run_;
  calibration::FullLegEvidenceStore full_leg_evidence_;
  // Calibration Persistence V1 (P3a). The ~5.3 KiB store and the 1.4 KiB
  // scratch record live inside this statically allocated Controller, never on
  // a stack. The backend is declared first: the service holds a pointer to it.
  calibration::CalibrationRecordNvsBackend persistence_backend_;
  calibration::CalibrationPersistenceService persistence_{&persistence_backend_};
  // The SAFE_OFF results of the buses the executor named at the END of the
  // previous tick, handed to exactly the next update() and then replaced -
  // a past VERIFIED_OFF is never reused for a new requirement. Cleared
  // whenever no run is active.
  calibration::FullLegSafeOffFrame full_leg_safe_off_frame_{};
  // Which executor phase change was last reported to calibration_, per run.
  uint32_t full_leg_reported_session_ = 0;
  uint32_t full_leg_phase_changes_seen_ = 0;
  // Last state printFullLegSearchEvent() / printFullLegSequenceEvent() reported.
  uint16_t search_event_steps_ = 0;
  uint8_t search_event_probe_phase_ = 0;
  uint8_t search_event_exec_phase_ = 0;
  uint8_t search_event_pass_ = 0;
  uint16_t search_event_bypass_ = 0;
  uint32_t sequence_event_key_ = 0;
  uint32_t sequence_event_changes_ = 0;
  bool held_role_failure_printed_ = false;
  // A recovery-only run (@CALIBRATION INITIAL RECOVERY) is in flight or has
  // just ended and its RESULT line is still to be printed.
  bool recovery_result_pending_ = false;
  // The transport-neutral telemetry layer (I6) — see ControllerService.h.
  // Bound to the same module pointers CommandRouter already holds; adds no
  // module ownership of its own.
  ControllerService service_;
  // The network transport (I7/I8, 2026-09-25 correction) — read-only Web
  // status plus HMAC-authenticated OTA ingest, both through the one
  // existing OtaManager writer. Never started from begin() — see
  // network/HttpTransport.h and the @WEB SERVER command in CommandRouter.
  network::HttpTransport http_transport_;
  CommandRouter command_router_;

  // Set only at the very end of begin(). The OTA self-check must not treat a
  // half-initialized Controller as evidence that the image works.
  bool initialized_ = false;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_CONTROLLER_H
