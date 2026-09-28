#ifndef MATDOG_CORE_CONTROLLER_H
#define MATDOG_CORE_CONTROLLER_H

#include <Arduino.h>

#include "../actuator/ActuatorRuntime.h"
#include "../actuator/ActuatorWritePolicy.h"
#include "../actuator/CalibrationGeometryProfileData.h"
#include "../calibration/CalibrationExecutionEngine.h"
#include "../calibration/CalibrationManager.h"
#include "../calibration/CalibrationMotionPermit.h"
#include "../calibration/CalibrationQ0CaptureSession.h"
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

  servo::ServoBus servo_bus_;
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
  // production composition. actuator_policy_ is bound to the REAL current
  // Geometry V5 profile below and actuator_runtime_ is bound to a REAL
  // production backend — both are safe to be real because no command path
  // anywhere reaches plan()/commit()/execute()/abort() on any of the three
  // (scripts/static_audit.py's
  // check_actuator_infrastructure_wired_fail_closed() enforces this
  // structurally); every geometry-authorised operation independently also
  // requires a live CALIBRATION session, which nothing in this Controller
  // can start yet (calibration_.startSession() has no caller here — the
  // same audit function enforces that too), so the fail-closed guarantee
  // does not rest on any single one of these facts alone.
  actuator::CalibrationGeometryProfile geometry_profile_;
  servo::ServoBusActuatorBackend actuator_backend_;
  actuator::SafeActuatorPolicy actuator_policy_;
  actuator::ActuatorRuntime actuator_runtime_;
  calibration::CalibrationExecutionEngine calibration_execution_;
  // CR3-M5: the session-scoped, RAM-only physical-motion permit — distinct
  // from the final hardware_motion_authorized flag, which stays false
  // throughout (CalibrationManager.h). Reset at boot; motion_permit_token_
  // is the credential from the most recent grant() and is never mutated
  // outside one. operator_calibration_motion_authorized_ has no setter
  // anywhere in this build (see updateCalibrationMotionPermit()'s file
  // comment) — a fresh explicit per-session grant path is deliberately left
  // for the reviewed hardware-authorization gate, not wired here.
  calibration::CalibrationMotionPermit motion_permit_;
  calibration::CalibrationMotionPermitToken motion_permit_token_;
  bool operator_calibration_motion_authorized_ = false;
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
