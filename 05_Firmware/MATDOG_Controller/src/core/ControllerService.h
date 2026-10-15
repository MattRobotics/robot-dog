#ifndef MATDOG_CORE_CONTROLLER_SERVICE_H
#define MATDOG_CORE_CONTROLLER_SERVICE_H

#include "../actuator/ActuatorWritePolicy.h"
#include "../calibration/CalibrationManager.h"
#include "../calibration/CalibrationMotionPermit.h"
#include "../calibration/FullLegCalibrationFinalizer.h"
#include "ActuatorAuthority.h"
#include "MotionService.h"
#include "Availability.h"
#include "CommandRouter.h"
#include "OperatingMode.h"
#include "PowerState.h"
#include "ServiceReadiness.h"
#include "SystemState.h"

// The transport-neutral semantic/telemetry layer — I6, implemented per the
// 2026-09-25 objective-change instruction: "Serial, future Web and future
// Jetson clients must consume the same semantic model."
//
// SCOPE, DELIBERATELY BOUNDED: this class covers passive status/telemetry
// reads only — the "telemetry" half of "semantic command/telemetry layer".
// Every accessor below forwards a struct its owning module already
// computed (WifiStatus, OtaManagerStatus, CalibrationSessionStatus, a
// DalySample, ...) — nothing here recomputes, caches, or duplicates
// CommandRouter's own logic; it is the same data CommandRouter already
// printed, with the "which module answers this" decision moved to one
// place so a second transport does not need core::CommandRouter::Modules
// (which carries direct pointers to servo::ServoBus and friends) to render
// the same semantics.
//
// Action/write commands (servo scan/census/preflight/read/safe_off, mode
// changes, the DALY KEY write, Wi-Fi enable/disable, LED test/off, system
// shutdown) are deliberately NOT routed through this layer in this gate —
// they have side effects and CommandRouter's existing per-command guards
// (MAINTENANCE-mode gates, the async pending/result state machine) stay
// exactly where they are. Unifying the ACTION half of the semantic model
// is future work, not attempted here.
//
// Presentation stays local to each transport (Serial's printf lines here,
// a future JSON/binary encoding elsewhere) — this class returns structured
// data, never formatted text.

namespace matdog {
namespace core {

struct CalibrationPersistenceView {
  bool published = false;       // false until the Controller's first tick
  bool available = false;       // valid acknowledged record, writes not blocked
  bool record_intact = false;   // the acknowledged generation's record is intact
  uint32_t acknowledged_generation = 0;
  const char* verdict = "NOT_RUN";  // static strings owned by the persistence layer
  const char* nvs = "NOT_INITIALIZED";
};

// Issue #48 Dashboard path. Implemented by the Controller; the transport
// sees only generation + explicit confirmation in, a JSON verdict out.
class CalibrationRestoreHook {
 public:
  virtual ~CalibrationRestoreHook() = default;
  virtual bool restoreJson(uint32_t generation, bool operator_confirmed, char* out, size_t cap) = 0;
};

// Operational deadman (2026-10-14): an authenticated AP/Dashboard request
// proves a present operator. Lapsing heartbeats expire the motion permit.
class OperatorHeartbeatHook {
 public:
  virtual ~OperatorHeartbeatHook() = default;
  virtual void heartbeat(uint32_t now_ms) = 0;
};

class ControllerService {
 public:
  using Modules = CommandRouter::Modules;
  void bindOperatorHeartbeat(OperatorHeartbeatHook* hook) { heartbeat_hook_=hook; }
  void operatorHeartbeat(uint32_t now_ms) { if(heartbeat_hook_)heartbeat_hook_->heartbeat(now_ms); }
  // Operational readiness view for the Dashboard (static strings).
  struct ReadinessView { const char* system_health="BOOTING"; const char* calibration="NOT_RESTORED";
    const char* joint_feedback="UNAVAILABLE"; const char* motion_authority="NOT_GRANTED"; const char* phase="BOOTING"; const char* reason="BOOTING"; };
  void setReadinessView(const ReadinessView& v) { readiness_view_=v; }
  const ReadinessView& readinessView() const { return readiness_view_; }
  void bindCalibrationRestore(CalibrationRestoreHook* hook) { restore_hook_=hook; }
  bool calibrationRestore(uint32_t generation,bool confirmed,char* out,size_t cap) {
    return restore_hook_&&restore_hook_->restoreJson(generation,confirmed,out,cap);
  }
  void bindMotionService(MotionService* motion) { motion_=motion; }
  motion::MotionAck motionCommand(const motion::MotionCommand& command,uint32_t now_ms) {
    return motion_?motion_->command(command,now_ms):motion::MotionAck{};
  }
  bool motionStatus(char* out,size_t cap,uint32_t now_ms) const { return motion_&&motion_->statusJson(out,cap,now_ms); }
  bool motionCapabilities(char* out,size_t cap) const { return motion_&&motion_->capabilitiesJson(out,cap); }
  bool motionAck(const motion::MotionAck& ack,char* out,size_t cap) const { return motion_&&motion_->ackJson(ack,out,cap); }
  bool motionActive() const { return motion_&&motion_->busy(); }
  bool motionEngaged() const { return motion_&&motion_->engaged(); }
  void motionDisconnected(uint32_t now_ms) { if(motion_)motion_->disconnected(now_ms); }


  void begin(const Modules& modules) { modules_ = modules; }

  // --- system / power / mode / authority ------------------------------------
  SystemHealth systemHealth() const { return modules_.system_state->systemHealth(); }
  uint32_t uptimeMillis(uint32_t now_ms) const {
    return modules_.system_state->uptimeMillis(now_ms);
  }
  PowerState powerState() const { return modules_.power_state->state(); }
  OperatingMode operatingMode() const { return modules_.operating_mode->mode(); }

  ActuatorAuthority authorityOwner() const { return modules_.authority->current(); }
  uint32_t authorityGeneration() const { return modules_.authority->generation(); }
  bool authorityInhibited() const { return modules_.authority->inhibited(); }
  InhibitReason authorityInhibitReason() const { return modules_.authority->inhibitReason(); }
  AuthorityResult authorityLastResult() const { return modules_.authority->lastResult(); }
  AuthorityClearReason authorityLastClearReason() const {
    return modules_.authority->lastClearReason();
  }
  const AuthorityCounters& authorityCounters() const { return modules_.authority->counters(); }

  // --- availability (init/detected/expected/result per module) -------------
  AvailabilityStatus imuAvailability() const { return modules_.imu->availability(); }
  AvailabilityStatus bmsAvailability() const { return modules_.daly->availability(); }
  AvailabilityStatus servoAvailability() const { return modules_.servo_bus->availability(); }
  AvailabilityStatus ledAvailability() const { return modules_.led->availability(); }

  // --- IMU -------------------------------------------------------------------
  bool imuStreamEnabled() const { return modules_.imu->streamEnabled(); }
  uint32_t imuRvCount() const { return modules_.imu->rvCount(); }
  uint32_t imuRuntimeResetCount() const { return modules_.imu->runtimeResetCount(); }

  // --- BMS / DALY --------------------------------------------------------------
  power::DalyCommResult bmsLastCommResult() const { return modules_.daly->lastCommResult(); }
  uint32_t bmsLastResultAgeMs(uint32_t now_ms) const {
    return modules_.daly->lastResultAgeMs(now_ms);
  }
  bool bmsHasValidSample() const { return modules_.daly->hasValidSample(); }
  const power::DalySample& bmsSample() const { return modules_.daly->sample(); }
  power::DalyKeyReadResult bmsKeyConfigReadResult() const {
    return modules_.daly->keyConfigReadResult();
  }
  uint32_t bmsKeyConfigReadAgeMs(uint32_t now_ms) const {
    return modules_.daly->keyConfigReadAgeMs(now_ms);
  }
  size_t bmsKeyConfigReadRxBytes() const { return modules_.daly->keyConfigReadRxBytes(); }
  const power::DalyKeyConfigSnapshot& bmsKeySnapshot() const {
    return modules_.daly->keyConfigSnapshot();
  }
  const power::DalyKeyWriteStatus& bmsKeyWriteStatus() const {
    return modules_.daly->keyWriteStatus();
  }

  // --- LED presentation --------------------------------------------------------
  bool ledTestRunning() const { return modules_.led->testRunning(); }
  bool ledDataPinDriven() const { return modules_.led->dataPinDriven(); }
  status::LedPresentationState ledPresentationState() const { return modules_.led_status->state(); }
  const status::LedStatusSnapshot& ledSnapshot() const { return modules_.led_status->snapshot(); }
  const status::LedStatusManager& ledManager() const { return *modules_.led_status; }
  status::LedDiagnostic ledDiagnostic() const { return modules_.led->diagnostic(); }

  // --- Wi-Fi / OTA / Calibration (already-aggregated structs) ------------------
  const network::WifiStatus& wifiStatus() const { return modules_.wifi->status(); }
  const update::OtaManagerStatus& otaStatus() const { return modules_.ota->status(); }
  const calibration::CalibrationSessionStatus& calibrationStatus() const {
    return modules_.calibration->status();
  }
  const calibration::Q0CaptureStatus& calibrationQ0Status() const {
    return modules_.q0_capture->status();
  }
  const calibration::PopulationEvidenceBuildResult& calibrationQ0Population() const {
    return modules_.q0_capture->populationResult();
  }
  const actuator::Q0BootstrapCandidate* calibrationQ0Candidates() const {
    return modules_.q0_capture->candidates();
  }

  // Calibration persistence, as a plain view. Only the Controller integrates
  // the persistence layer (P3a), so it publishes this copy of the last LOAD's
  // RAM snapshot each tick; nothing here reads NVS or names a persistence
  // type. "Available" is evidence that a valid acknowledged record exists; it
  // is not a motion permit and admits no JointTransform.
  void setCalibrationPersistenceView(const CalibrationPersistenceView& view) {
    calibration_persistence_ = view;
  }
  const CalibrationPersistenceView& calibrationPersistence() const {
    return calibration_persistence_;
  }

  // --- servo diagnostic results (formatting-only snapshots) --------------------
  const servo::ScanResult& servoScanResult() const { return modules_.servo_bus->lastScanResult(); }
  const servo::CensusResult& servoCensusResult() const { return modules_.servo_census->result(); }
  const servo::PreflightResult& servoPreflightResult() const {
    return modules_.servo_preflight->result();
  }

  // --- Safe Actuator policy status (I4/I5 fail-closed infrastructure) --------
  // Read-only. Never plan()/commit()/execute()/abort() — enforced by
  // scripts/static_audit.py's check_actuator_infrastructure_wired_fail_closed().
  // ActuatorRuntime and CalibrationExecutionEngine themselves expose no
  // status of their own worth reporting (both are effectively stateless
  // between calls neither of which this class ever makes) — everything
  // observable about the I4/I5 infrastructure lives on the policy they both
  // sit on top of.
  uint32_t actuatorPolicyEpoch() const { return modules_.actuator_policy->epoch(); }
  bool actuatorPolicyHasOutstandingTransaction() const {
    return modules_.actuator_policy->hasOutstandingTransaction();
  }
  const actuator::ActuatorPolicyCounters& actuatorPolicyCounters() const {
    return modules_.actuator_policy->counters();
  }
  actuator::WriteDecision actuatorPolicyLastDecision() const {
    return modules_.actuator_policy->lastDecision();
  }
  uint8_t actuatorPolicyLimitsAdmitted() const { return modules_.actuator_policy->limits().size(); }
  uint8_t actuatorPolicyTransformsAdmitted() const {
    return modules_.actuator_policy->transforms().size();
  }
  bool actuatorPolicyGeometryBound() const {
    return modules_.actuator_policy->currentGeometryTag() != actuator::kNoGeometryProvenance;
  }

  // Network mutations share the Controller's safety facts; no actuator action
  // is exposed here. Config queue sets busy atomically before returning.
  bool networkCritical() const {
    return motionActive() || powerState() != PowerState::RUN || operatingMode() != OperatingMode::MAINTENANCE ||
           authorityOwner() != ActuatorAuthority::NONE || authorityInhibited() ||
           (modules_.calibration && modules_.calibration->sessionLive()) ||
           (modules_.q0_capture && modules_.q0_capture->active()) ||
           (modules_.full_leg_run && modules_.full_leg_run->armed) ||
           (modules_.motion_permit && modules_.motion_permit->active()) ||
           (modules_.startup_qualification &&
            modules_.startup_qualification->phase() !=
                calibration::StartupQualificationPhase::IDLE &&
            modules_.startup_qualification->phase() !=
                calibration::StartupQualificationPhase::COMPLETE &&
            modules_.startup_qualification->phase() !=
                calibration::StartupQualificationPhase::REFUSED);
  }
  bool remoteUpdateAllowed() const {
    return !networkCritical() && !modules_.wifi->configBusy() && wifiStatus().connected &&
           wifiStatus().nvs_active && wifiStatus().admin_provisioned;
  }
  network::AdminAccess &networkAdminAccess() { return modules_.wifi->adminAccess(); }
  bool configureNetwork(const network::ConfigPatch& p, uint32_t generation = 0) {
    return !networkCritical() && modules_.wifi->configure(p, generation);
  }
  bool scanNetwork() { return !networkCritical() && modules_.wifi->requestScan(); }
  bool accessPoint(bool on, uint32_t generation = 0) {
    return !networkCritical() && !modules_.wifi->configBusy() && modules_.wifi->requestAp(on, generation);
  }
  const uint8_t* networkAdminDigest() const { return modules_.wifi->adminDigest(); }

  // --- readiness -----------------------------------------------------------
  // The current repository truth, until the corresponding hardware-
  // validation gate actually passes. These two flags are the ONLY place a
  // future gate PASS needs to flip to update every readiness answer at once.
  ServiceReadiness readiness(ServiceCapability capability) const {
    ServiceReadinessInputs in;
    in.hardware_motion_authorized = calibration::CalibrationManager::hardwareMotionAuthorized();
    in.wifi_hardware_validated = false;  // ROADMAP.md stage 10: not yet associated with an AP
    in.ota_hardware_validated = false;   // no OTA image has ever been received on device
    return classifyCapability(capability, in);
  }

 private:
  Modules modules_{};
  MotionService* motion_=nullptr;
  CalibrationRestoreHook* restore_hook_=nullptr;
  OperatorHeartbeatHook* heartbeat_hook_=nullptr;
  ReadinessView readiness_view_{};
  CalibrationPersistenceView calibration_persistence_{};
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_CONTROLLER_SERVICE_H
