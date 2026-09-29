#ifndef MATDOG_CORE_COMMAND_ROUTER_H
#define MATDOG_CORE_COMMAND_ROUTER_H

#include <Arduino.h>

#include "../calibration/CalibrationManager.h"
#include "../calibration/CalibrationQ0CaptureSession.h"
#include "../imu/Bno085Imu.h"
#include "../network/WifiManager.h"
#include "../power/DalyBms.h"
#include "../update/OtaManager.h"
#include "../servo/ServoBus.h"
#include "../servo/ServoCensus.h"
#include "../servo/ServoPreflight.h"
#include "../status/LedRing.h"
#include "../status/LedStatusManager.h"
#include "ActuatorAuthority.h"
#include "Availability.h"
#include "OperatingMode.h"
#include "PowerState.h"
#include "SystemState.h"

namespace matdog {
namespace actuator {
// Forward-declared for the same reason as core::ControllerService below:
// only a pointer is needed here, and ActuatorWritePolicy.h is a large
// include CommandRouter.h itself has no other reason to pull in.
class SafeActuatorPolicy;
// Forward-declared for the same reason: only a pointer is needed to pass
// the already-bound profile through to actuator::prepareCurrentQ0Evidence()
// (@CALIBRATION Q0 PROMOTE). CommandRouter never binds/clears it and never
// reads a joint/endpoint record out of it directly.
class CalibrationGeometryProfile;
}  // namespace actuator

namespace calibration {
// Forward-declared for the same reason: CommandRouter.cpp includes the full
// headers wherever it actually builds/parses request structs; this header
// only needs pointer storage. See CalibrationMotionPermit.h for why
// CalibrationMotionAuthorizationState is a separate type from
// CalibrationMotionPermit itself.
class CalibrationMotionPermit;
struct CalibrationMotionAuthorizationState;
class FirstMotionExecutor;
class FullLegCalibrationExecutor;
struct FullLegRunState;
class FullLegEvidenceStore;
}  // namespace calibration

namespace network {
// Forward-declared, not included: HttpTransport.h pulls in
// esp_http_server.h and ControllerService.h, neither of which
// CommandRouter.h needs just to hold a pointer for @WEB SERVER
// START|STOP|STATUS. CommandRouter.cpp includes it directly.
class HttpTransport;
}  // namespace network

namespace core {

// Forward-declared, not included: ControllerService.h includes THIS header
// (it needs the complete Modules struct below), so this direction must stay
// a pointer-only forward declaration to avoid a cycle. CommandRouter.cpp
// includes ControllerService.h directly wherever it actually calls into it.
class ControllerService;

// Small, explicit, safe USB CDC diagnostic command surface.
// See handoff section 22. Deliberately does NOT expose: arbitrary EEPROM
// write, ID recode, factory reset, CalibrationOfs, broadcast write,
// unrestricted GoalPosition, or any DALY write other than the ONE semantic
// KEY configuration (@BMS KEY SET DISCHARGE CONFIRM: 0x0120 := 0x005A). The
// DALY KEY commands are fixed strings with no arguments: no register,
// address or value input.
class CommandRouter {
 public:
  struct Modules {
    servo::ServoBus* servo_bus;
    servo::ServoCensus* servo_census;
    servo::ServoPreflight* servo_preflight;
    imu::Bno085Imu* imu;
    power::DalyBms* daly;
    status::LedRing* led;
    status::LedStatusManager* led_status;
    network::WifiManager* wifi;
    update::OtaManager* ota;
    SystemState* system_state;
    PowerStateMachine* power_state;
    OperatingModeManager* operating_mode;
    // Pointer to the ONE arbiter owned by Controller. Never a copy of its
    // state: the router asks, it does not remember.
    ActuatorAuthorityArbiter* authority;
    calibration::CalibrationManager* calibration;
    // CR2-B read-only evidence-acquisition coordinator. It carries no
    // actuator authority and produces CANDIDATE q0 evidence only.
    calibration::CalibrationQ0CaptureSession* q0_capture;
    // I4/I5 CR3-M5 production composition (2026-09-28). Read-only status
    // commands remain the norm — see ControllerService.h and
    // scripts/static_audit.py's check_actuator_infrastructure_wired_fail_closed()
    // — with exactly one reviewed exception: @CALIBRATION Q0 PROMOTE admits
    // the frozen, re-verified q0 transforms via transforms().admit(). That
    // is RAM-only evidence admission, never a plan()/commit()/execute()/
    // abort() call, and the audit function enforces that distinction too.
    actuator::SafeActuatorPolicy* actuator_policy;
    // The current, already-bound Geometry V5 profile (Controller::geometry_profile_).
    // Read-only here as well: passed straight through to
    // actuator::prepareCurrentQ0Evidence(), never mutated.
    const actuator::CalibrationGeometryProfile* geometry_profile;
    // CR3 continuation. @CALIBRATION MOTION PERMIT GRANT/REVOKE are the only
    // command handlers that call motion_permit->grant()/revoke() or write
    // motion_authorization's fields — every other command (including the
    // first-motion one below) only reads calibration/actuator_policy/
    // operating_mode, exactly like every command before this session.
    calibration::CalibrationMotionPermit* motion_permit;
    calibration::CalibrationMotionAuthorizationState* motion_authorization;
    // CR3 continuation, Objective C. @CALIBRATION MOTION DIRECTION_VERIFY is
    // the one reviewed command that calls first_motion->start() — see
    // check_first_motion_command_wiring() in scripts/static_audit.py for
    // what is mechanically pinned about this one call site.
    calibration::FirstMotionExecutor* first_motion;
    // CR3 continuation: @CALIBRATION FULL LEG <LF|RF|RH|LH>
    // CONFIRM_FULL_CALIBRATION is the one reviewed command that calls
    // full_leg_calibration->start() — see check_full_leg_command_wiring() in
    // scripts/static_audit.py.
    calibration::FullLegCalibrationExecutor* full_leg_calibration;
    // The transport-neutral telemetry layer (I6) — read-only status
    // commands route through this instead of the pointers above directly.
    // Action/write commands still use the module pointers above; see
    // ControllerService.h for the exact scope boundary.
    ControllerService* service;
    // The network transport (I7/I8, 2026-09-25 correction). @WEB SERVER
    // START|STOP|STATUS only starts/stops/queries it — CommandRouter never
    // reaches into its OTA session or HTTP internals. See
    // network/HttpTransport.h.
    network::HttpTransport* http_transport;
    // Four-leg Full Calibration bookkeeping, both owned by Controller. The
    // router ARMS full_leg_run only after full_leg_calibration->start()
    // accepted the run and reads full_leg_evidence for status/export; only
    // Controller finalizes a run and stores its record. Appended last: this
    // struct is aggregate-initialized positionally in Controller::begin().
    calibration::FullLegRunState* full_leg_run;
    calibration::FullLegEvidenceStore* full_leg_evidence;
  };

  void begin(const Modules& modules);
  void update(uint32_t now_ms);

  // Used by the OTA first-boot self-check: "is the command surface usable?"
  // is one of the software-only conditions that must hold before a freshly
  // booted OTA image is allowed to confirm itself.
  bool bound() const { return modules_.system_state != nullptr; }

 private:
  void handleLine(String line);
  bool q0CaptureOwnsServoDiagnostics() const;
  // True while either motion executor (the DIRECTION_VERIFY first-motion
  // path or the Full Leg Calibration sequence) is actively using the one
  // shared ServoBus/UART - see servoDiagnosticBusy()'s own comment for why a
  // diagnostic scan/census/preflight/q0-capture transaction must never
  // interleave with it.
  bool motionExecutorBusy() const;
  bool servoDiagnosticBusy() const;
  void printHelp();
  void printStatus();
  void printImuStatus();
  void printBmsStatus();
  void printBmsKeyReadResult();
  void printBmsKeyStatus();
  static void printBmsKeySnapshot(const power::DalyKeyConfigSnapshot& k);
  void printBmsKeyWriteResult();
  void printBmsKeyWriteStatus();
  void printLedStatus();
  // Pure presentation of wifi->status(). Formats a snapshot the Wi-Fi layer
  // already computed; it never queries the radio, so a future Web adapter
  // renders the same struct without a second hardware path (see
  // ARCHITECTURE.md, telemetry snapshot model).
  void printWifiStatus();
  // Pure presentation of ota->status(). Read-only: OTA-A ships no transport
  // and no command that can start an update.
  void printOtaStatus();
  // Read-only build/source identity (G4 SOURCE_SIGNATURE). Presents facts
  // already computed elsewhere (build::kBuildId, the OTA manager's running-
  // image bookkeeping, the ESP-IDF running partition) — no new hardware
  // read, no new bus traffic. Deliberately does NOT surface esp_app_desc_t:
  // in an Arduino-ESP32 build it describes arduino-lib-builder, not MATDOG
  // (see update/OtaPolicy.h), so it is not authoritative identity here
  // either.
  void printSourceSignature();
  // Read-only HostLink readiness (I6). BLOCKED/TO_TEST/READY per named
  // capability — see core/ServiceReadiness.h. Computed on demand from two
  // facts; nothing here is cached state.
  void printHostLinkReadiness();
  void printServoScanResult();
  // Pure presentation of servo_census->result(). Computes nothing: the
  // classification lives in servo/ServoPopulation.h so a future Web UI /
  // HostLink adapter can render the same structured result without
  // reimplementing it or re-scanning the bus (handoff sections 7/8/9).
  void printServoCensusResult();

  // Pure presentation of servo_preflight->result(). Computes nothing and
  // issues no bus transaction: the service already did the reading.
  void printServoPreflightResult();
  void printServoRead(int id);
  void printServoSafeOff(int id);
  void printModeStatus();
  // Read-only presentation of the central arbiter. There is deliberately no
  // command that acquires or releases authority: no write-capable owner
  // exists yet, and an operator-driven acquire would be a write path this
  // phase explicitly does not add.
  void printAuthorityStatus();
  // Read-only presentation of the calibration manager. There is deliberately
  // no command that starts, runs or moves anything: no write path exists, and
  // the repository declares hardware motion unauthorized.
  void printCalibrationStatus();
  void printCalibrationQ0Status();
  // Read-only presentation of the Safe Actuator policy (I4/I5 fail-closed
  // infrastructure, 2026-09-25 objective change). There is deliberately no
  // command that plans, commits, executes or aborts anything through it —
  // see ControllerService.h and scripts/static_audit.py's
  // check_actuator_infrastructure_wired_fail_closed().
  void printActuatorStatus();
  // Read-only presentation of the Full Leg Calibration sequencer — phase,
  // failure, both bus ids, both SAFE_OFF-pending flags and (once available)
  // both sides' witnessed evidence plus the derived operational envelopes.
  // The ONLY long-running (many-second) command in this router, so unlike
  // every other action handler's own inline response line, this one is also
  // worth polling BETWEEN ticks - hence a dedicated STATUS command.
  void printFullLegCalibrationStatus();
  // Deterministic key=value export of the RAM evidence for all four legs of
  // this power-up (@CALIBRATION EVIDENCE EXPORT). Read-only.
  void printFullLegEvidenceExport();
  void pumpFullLegEvidenceExport();
  // Read-only presentation of the HTTP transport's own lifecycle state
  // (I7/I8). Never reports OTA session secrets or in-flight request
  // contents — those live only in HttpTransport's cross-thread mailbox,
  // which this deliberately does not reach into.
  void printWebStatus();
  static void printAvailabilityLine(const char* label, const AvailabilityStatus& a);

  Modules modules_{};
  bool bms_stream_enabled_ = false;
  uint32_t last_bms_stream_ms_ = 0;
  bool bms_key_read_result_pending_ = false;
  bool bms_key_write_pending_ = false;
  bool bms_key_write_ack_reported_ = false;
  bool servo_scan_result_pending_ = false;
  bool servo_census_result_pending_ = false;
  bool servo_preflight_result_pending_ = false;
  bool q0_capture_result_pending_ = false;
  bool evidence_export_pending_ = false;
  uint16_t evidence_export_next_ = 0;

  static constexpr size_t kLineBufSize = 96;
  char line_buf_[kLineBufSize] = {0};
  size_t line_len_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_COMMAND_ROUTER_H
