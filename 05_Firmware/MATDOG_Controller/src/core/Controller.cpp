#include "AdmissionDiagnostics.h"
#include "Controller.h"
#include <esp_random.h>

#include <esp_ota_ops.h>
#include <esp_system.h>
#include <string.h>
#include <cmath>
#include "../motion/PairedProneRoute.h"
#include "../motion/OperationalWalkProfile.h"
#include "../servo/ServoBus.h"
static_assert(matdog::motion::operational_walk::kStandardDrive.goalSpeed==matdog::servo::ServoBus::kWalkDriveSpeed&&
              matdog::motion::operational_walk::kStandardDrive.acceleration==matdog::servo::ServoBus::kWalkDriveAcceleration&&
              matdog::motion::operational_walk::kFastDrive.goalSpeed==matdog::servo::ServoBus::kFastWalkDriveSpeed&&
              matdog::motion::operational_walk::kFastDrive.acceleration==matdog::servo::ServoBus::kFastWalkDriveAcceleration,
              "the ServoBus accepts exactly the walk profile table's drive pairs");
static_assert(matdog::motion::operational_walk::kStandProfile.drive.goalSpeed==matdog::servo::ServoBus::kStandDriveSpeed&&
              matdog::motion::operational_walk::kStandProfile.drive.acceleration==matdog::servo::ServoBus::kStandDriveAcceleration,"STAND drive pair");

#include "../config/BuildConfig.h"
#include "../config/InstallationAttestation.h"
#include "../config/OtaCredentials.h"
#include "../config/Pins.h"

// G3.1: the non-blocking USB CDC guarantee in Controller::begin() rests on
// HWCDC::write() as shipped in esp32:esp32 3.3.11, where Serial == HWCDCSerial
// under USBMode=hwcdc + CDCOnBoot=cdc. Re-audit HWCDC.cpp before changing either.
#if !(ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT)
#error "G3.1: USB CDC transmit policy audited for USBMode=hwcdc + CDCOnBoot=cdc only"
#endif
#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "G3.1: HWCDC transmit semantics audited against esp32:esp32 3.3.11 only"
#endif

namespace matdog {
namespace core {
namespace {
// Age of a sample against a loop-start timestamp: a sample taken later in the
// same loop (negative age) is fresh, never 'stale by underflow'.
bool sampleFresh(uint32_t now_ms,uint32_t sampled_at_ms,uint32_t max_age_ms){
  return static_cast<int32_t>(now_ms-sampled_at_ms)<=static_cast<int32_t>(max_age_ms);
}
}  // namespace

namespace {
const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}
}  // namespace

void Controller::begin() {
  // G3.1: USB CDC output must never block this loop. HWCDC keeps treating a
  // host as connected after it closes the port, and with its default 100 ms
  // TX timeout each print then waits up to ~2 s on a full ring (BNO085 fell
  // from 50 Hz to 0.68 Hz). With timeout 0 every such wait becomes an
  // immediate drop. Ring before begin(): the core has not started Serial
  // yet on hwcdc builds, so it is created at its final size before any ISR,
  // byte or host exists. See build::kUsbTxRingBytes for the sizing.
  Serial.setTxBufferSize(build::kUsbTxRingBytes);
  Serial.setTxTimeoutMs(build::kUsbTxTimeoutMs);
  Serial.begin(build::kUsbSerialBaud);
  delay(1500);  // let native USB CDC enumerate, matching every proven bring-up sketch.

  system_state_.beginBoot(millis());

  // Boot always lands on NONE. A previous authority is never restored - not
  // from NVS, not from a retained value, not from anywhere. See
  // core/ActuatorAuthority.h.
  authority_.reset(AuthorityClearReason::BOOT);

  // Before the banner, so the banner can report what the bootloader left us
  // with. Reads the partition table and otadata; writes nothing. In
  // particular it does NOT confirm the running image - that is earned in
  // update(), over seconds, by actually running (see update/OtaBootGuard.h).
  ota_.begin(millis(), &authority_);

  // Binds the calibration manager to the same single arbiter. It starts with
  // no session and, because the repository declares the current calibration
  // stale and hardware motion unauthorized, it refuses to open a live one.
  calibration_.begin(&authority_);

  // Safe Actuator / Calibration Execution infrastructure (I4/I5), CR3-M5
  // production composition — see Controller.h's member comment for why
  // binding the real profile/backend here stays fail-closed. Binding the
  // profile only makes its provenance tag MATCHABLE; it admits no limit or
  // transform (that still requires the explicit @CALIBRATION Q0 PROMOTE
  // command, itself gated on an explicit currentness confirmation) and
  // starts no session.
  geometry_profile_.bind(&actuator::geometry_data::kProvenance, actuator::geometry_data::kJoints,
                         actuator::geometry_data::kJointCount, actuator::geometry_data::kEndpoints,
                         actuator::geometry_data::kEndpointCount);
  // Calibration Persistence V1 (P3a): init the NVS backend and run the
  // read-only boot LOAD BEFORE anything that could authorise an actuator is
  // wired. No servo bus traffic, no write, no format; the verdict only feeds
  // @CALIBRATION PERSIST STATUS and the SAVE gate. "Available" is never a
  // movement authorisation and nothing is restored into the motion path.
  {
    const calibration::NvsInitStatus nvs_status = persistence_backend_.begin();
    persistence_.begin(nvs_status, persistence_backend_.lastEspError());
    persistence_.load(geometry_profile_);
    const calibration::PersistenceSnapshot& boot = persistence_.snapshot();
    Serial.printf("CALIBRATION_PERSISTENCE_BOOT nvs=%s verdict=%s available=%u motion_authorized=0\n",
                  calibration::toString(nvs_status), calibration::toString(boot.verdict),
                  persistence_.calibrationAvailable() ? 1u : 0u);
  }
  actuator_backend_.begin(&servo_bus_);
  actuator_policy_.begin(&authority_);
  actuator_policy_.bindGeometry(&geometry_profile_, &actuator::geometry_data::kProvenance);
  // The geometry-validated 24-contact Full Calibration sequence plan
  // (CalibrationSequencePlan.h). Usable only while it matches the bound
  // model; binding it authorises nothing without a live session + permit.
  actuator_policy_.bindSequencePlan(&actuator::sequence_plan_data::kPlan);
  actuator_runtime_.begin(&actuator_policy_, &actuator_backend_);
  motion_executor_.begin(&actuator_policy_, &actuator_runtime_, &geometry_profile_,
                         &actuator::geometry_data::kProvenance, &authority_);
  motion_coordinator_.begin(&motion_executor_, &actuator_policy_);
  for(unsigned k=0;k<12;++k)canonical_leg_ids_[k]=servo::legServoAt(k)->bus_id;
  actuator_runtime_.bindCanonicalBusIds(canonical_leg_ids_,12);
  motion_coordinator_.bindClock(&motion_clock_);
  operational_session_.begin(&actuator_policy_,&actuator_runtime_,&motion_executor_,&authority_,
                              &geometry_profile_,&actuator::geometry_data::kProvenance);
  motion_coordinator_.bindSession(&operational_session_);
  operational_armer_.owner=this;motion_coordinator_.bindArmer(&operational_armer_);
  motion_service_.begin(&motion_executor_, &motion_coordinator_);
  // No boot restore, planner approval, authority request, torque or permit.

  calibration_execution_.begin(&actuator_policy_, &actuator_runtime_, &geometry_profile_,
                               &actuator::geometry_data::kProvenance);
  // A reboot/default construction always starts revoked (CalibrationMotionPermit.h)
  // — reset() is explicit here anyway so the boot sequence never depends on
  // that default staying true by accident.
  motion_permit_.reset();
  motion_authorization_.revoke();

  // CR3 continuation, Objective 3/4 config: max_telemetry_age_ms (3000) and
  // motion_timeout_ms (12000) reuse CalibrationDomain.h's own documented LF
  // V25 precedents (CalibrationFailure::TELEMETRY_STALE "MAX_TELEMETRY_AGE
  // = 3 s", CalibrationFailure::MOTION_TIMEOUT "MOTION_TIMEOUT = 12 s") —
  // reused, not invented. stall_window_ms/stall_progress_ticks/
  // arrival_tolerance_ticks have no such historical precedent (LF V25 never
  // defined a live-monitoring stall detector); the outer motion_timeout_ms
  // above remains the true safety backstop regardless of how these three
  // are tuned, so getting them imprecise cannot itself make a bounded move
  // unsafe — only less responsive. See MotionDeadman.h and the CR3
  // development log.
  {
    actuator::MotionDeadmanConfig deadman{};
    deadman.max_telemetry_age_ms = 3000;
    deadman.motion_timeout_ms = 12000;
    deadman.stall_window_ms = 2000;
    deadman.stall_progress_ticks = 2;
    deadman.arrival_tolerance_ticks = 4;
    calibration::FirstMotionConfig first_motion_config{};
    first_motion_config.deadman = deadman;
    first_motion_.begin(&actuator_policy_, &actuator_runtime_, &geometry_profile_,
                        &actuator::geometry_data::kProvenance, first_motion_config);

    // 24-contact Full Calibration (LF V25 generalized): the probe's steps are
    // monitored inside ContactProbeEngine by V25's own settle window and
    // contact detector, and every prerequisite / recovery / return move by
    // the executor's own V25 StableTargetGate. The one remaining deadman-
    // monitored move is the 96-tick backoff, at the V25 calibration speed
    // (160) with V25's figures: a travel-aware budget at its conservative
    // MIN_EXPECTED_MOTION_TICKS_PER_SECOND (80) on top of the same 12 s, and
    // the arrival band STATIC_TOLERANCE + 2 = 12 ticks - a position-controlled
    // ST3215 settles a few ticks short of its goal (observed 4-5 on
    // LF_UPPER), which the 4-tick DIRECTION_VERIFY tolerance above would
    // misread as a stall. Inside that band ContactProbeEngine applies V25's
    // StableTargetGate before the backoff counts as arrived.
    actuator::MotionDeadmanConfig full_leg_backoff = deadman;
    full_leg_backoff.nominal_travel_ticks_per_s = calibration::kSearchMinExpectedTicksPerSecond;
    full_leg_backoff.arrival_tolerance_ticks = calibration::kSearchBackoffSettleToleranceTicks;
    // V25 TELEMETRY_TIMEOUT (2 s), as for every other search observation.
    full_leg_backoff.max_telemetry_age_ms = calibration::kSearchTelemetryTimeoutMs;
    calibration::FullLegCalibrationConfig full_leg_config{};
    full_leg_config.probe_backoff_deadman = full_leg_backoff;
    full_leg_calibration_.begin(&actuator_policy_, &actuator_runtime_, &calibration_execution_,
                               &geometry_profile_, &actuator::geometry_data::kProvenance,
                               full_leg_config);
  }

  printBootBanner();

  // Init order: transports that cannot interfere with each other first.
  // No module's begin() may command motion, enable torque, or write
  // EEPROM/DCD/BMS config (handoff sections 5/10/28).
  servo_bus_.begin();
  servo_bus_.setStopPreempt(&Controller::stopPreempt,this);
  system_state_.setServoHealth(servo_bus_.health());

  // Bind the ONE canonical census service to the ONE ServoBus.
  // ROBOT_POWERED now performs an intentional boot self-test: start() only
  // arms the incremental scan; ServoBus::update() performs at most one Ping
  // per later Controller tick. No torque, GoalPosition or EEPROM write is
  // reachable from ServoCensus.
  servo_census_.begin(&servo_bus_);
  servo_preflight_.begin(&servo_bus_);

  if (build::kServoPowerAvailable) {
    startup_servo_census_pending_ = servo_census_.start();
    if (startup_servo_census_pending_) {
      system_state_.setServoHealth(ModuleHealth::NOT_INITIALIZED);
      Serial.printf("STARTUP_SERVO_CENSUS=RUNNING canonical=%u expected_now=%u range=%d..%d\n",
                    (unsigned)servo::canonicalAllocatedCount(),
                    (unsigned)servo::expectedNowCount(),
                    servo::kCanonicalScanLo, servo::kCanonicalScanHi);
    } else {
      // A powered robot that cannot even start its required population
      // self-test must fail closed rather than remain BOOTING forever.
      system_state_.setServoHealth(ModuleHealth::FAULT);
      Serial.println("STARTUP_SERVO_CENSUS=START_FAILED");
    }
  }

  imu_.begin();
  // Core Motion V5: IMU acquisition owns its task (SPI + INT, bounded drain). If the
  // task cannot be created the Controller falls back to the same bounded service pass
  // inline - never the library's blocking wait.
  if(!imu_.startTask())Serial.printf("IMU_TASK=%s\n",imu_.availability().init==InitializationState::INITIALIZED?"START_FAILED_INLINE_FALLBACK":"NOT_STARTED");
  system_state_.setImuHealth(imu_.health());

  daly_.begin();
  system_state_.setBmsHealth(daly_.health());

  led_.begin();
  system_state_.setLedHealth(led_.health());
  led_status_.begin(&led_);

  // Configures the Wi-Fi policy and publishes its first snapshot. It does
  // NOT start the radio: the first WiFi.mode() call initializes the driver
  // and allocates tens of KB of heap, which does not belong in a boot path
  // that must reach SYSTEM_BOOT_COMPLETE promptly. The radio comes up from
  // update(), a few ticks later, if credentials exist.
  //
  // Wi-Fi is deliberately absent from system_state_: a missing access point
  // is not a robot health fact, and the G3/G3.1-validated meaning of
  // SYSTEM health must not silently change because a router rebooted.
  // Wi-Fi is observable through @STATUS and @WIFI STATUS instead.
  wifi_.begin(millis());

  CommandRouter::Modules modules{
      &servo_bus_, &servo_census_, &servo_preflight_, &imu_, &daly_, &led_, &led_status_,
      &wifi_, &ota_,
      &system_state_,
      &power_state_, &operating_mode_, &authority_, &calibration_, &q0_capture_,
      &actuator_policy_,
      &geometry_profile_,
      &motion_permit_,
      &motion_authorization_,
      &first_motion_,
      &full_leg_calibration_,
      &service_,
      &http_transport_,
      &full_leg_run_,
      &full_leg_evidence_,
      &persistence_,
      &first_motion_safe_off_result_,
      &startup_qualification_,
      &runtime_restore_,
  };
  validation_nonce_=esp_random();if(!validation_nonce_)validation_nonce_=1;
  modules.validation_context=this;
  modules.validation_command=[](void* context,const char* line,uint32_t now,char* out,size_t size){
    return static_cast<Controller*>(context)->localValidationCommand(line,now,out,size);
  };
  modules.commissioning_command=[](void* context,const char* line,uint32_t now,char* out,size_t size){
    return static_cast<Controller*>(context)->commissioningCommand(line,now,out,size);
  };
  boot_nonce_=esp_random();if(!boot_nonce_)boot_nonce_=1;
  service_.begin(modules);
  service_.bindMotionService(&motion_service_);
  restore_hook_.owner=this;service_.bindCalibrationRestore(&restore_hook_);
  heartbeat_hook_.owner=this;service_.bindOperatorHeartbeat(&heartbeat_hook_);
  // Never starts the listening socket here — see network/HttpTransport.h.
  // The secret is passed as raw bytes (strlen of the configured string, or
  // 0 if none was configured); OtaSession fails closed on a zero-length
  // secret exactly like WifiPolicy fails closed on an empty SSID.
  http_transport_.begin(&service_, &ota_,
                        reinterpret_cast<const uint8_t*>(config::kOtaSecret),
                        config::kOtaSecretPresent ? strlen(config::kOtaSecret) : 0);
  command_router_.begin(modules);

  system_state_.update();
  power_state_.enterPowerCheck();
  power_state_.enterRun();  // V0.1 has no motion-authorization gate to hold on.

  Serial.printf("SYSTEM_BOOT_COMPLETE health=%s power_state=%s\n",
                toString(system_state_.systemHealth()),
                toString(power_state_.state()));

  // Last statement in begin(), deliberately. This is one of the conditions
  // the OTA first-boot self-check requires, and it must mean "begin() ran to
  // completion", not "begin() started".
  initialized_ = true;
}

void Controller::updateQ0Capture() {
  if (!q0_capture_.active()) return;

  // A q0 evidence capture is a MAINTENANCE diagnostic transaction. Leaving
  // MAINTENANCE ends the transaction; there is no authority to release
  // because read-only evidence acquisition never acquired one.
  if (operating_mode_.mode() != OperatingMode::MAINTENANCE) {
    q0_capture_.fail(calibration::Q0CaptureFailure::MODE_NOT_MAINTENANCE);
    return;
  }

  switch (q0_capture_.status().state) {
    case calibration::Q0CaptureState::NEED_CENSUS_START:
      // Reuse the ONE existing census service and therefore the ONE ServoBus
      // scan state machine. No second scan implementation exists here.
      if (!servo_census_.start()) {
        q0_capture_.fail(calibration::Q0CaptureFailure::CENSUS_START_REFUSED);
        return;
      }
      if (!q0_capture_.markCensusStarted()) {
        q0_capture_.fail(calibration::Q0CaptureFailure::WRONG_STATE);
      }
      return;

    case calibration::Q0CaptureState::WAIT_CENSUS:
      if (servo_census_.state() != servo::ServoCensus::State::COMPLETE) return;
      if (!q0_capture_.submitCensus(servo_census_.result())) {
        q0_capture_.fail(calibration::Q0CaptureFailure::WRONG_STATE);
      }
      return;

    case calibration::Q0CaptureState::NEED_PREFLIGHT_START:
      // Same rule: reuse the existing permanent read-only preflight service.
      if (!servo_preflight_.start()) {
        q0_capture_.fail(calibration::Q0CaptureFailure::PREFLIGHT_START_REFUSED);
        return;
      }
      if (!q0_capture_.markPreflightStarted()) {
        q0_capture_.fail(calibration::Q0CaptureFailure::WRONG_STATE);
      }
      return;

    case calibration::Q0CaptureState::WAIT_PREFLIGHT:
      if (servo_preflight_.state() != servo::ServoPreflight::State::COMPLETE) return;
      // This call builds CR1 formal current population evidence from the
      // census + preflight that THIS acquisition transaction sequenced.
      q0_capture_.submitPreflight(servo_preflight_.result());
      return;

    case calibration::Q0CaptureState::SAMPLING: {
      calibration::Q0ReadRequest request{};
      if (!q0_capture_.nextReadRequest(&request) || !request.valid) {
        q0_capture_.fail(calibration::Q0CaptureFailure::WRONG_STATE);
        return;
      }

      // Exactly one existing ServoBus runtime read per Controller tick.
      // readRuntimeState() supplies the two CR2 facts needed here:
      // present_position and TorqueEnable. No new register accessor exists.
      servo::ServoBus::RuntimeState state{};
      calibration::Q0ReadObservation observation{};
      observation.bus_id = request.bus_id;
      observation.read_ok = servo_bus_.readRuntimeState(request.bus_id, &state);
      if (observation.read_ok) {
        observation.raw_tick = state.present_position;
        observation.torque_enable = state.torque_enable;
      }
      q0_capture_.recordRead(observation);
      return;
    }

    case calibration::Q0CaptureState::IDLE:
    case calibration::Q0CaptureState::COMPLETE:
    case calibration::Q0CaptureState::FAILED:
      return;
  }
}

// CR3-M5, extended by the CR3 continuation session. Every field below is
// read fresh from the live module it names — none is cached across ticks —
// so a permit that was ACTIVE last tick cannot out-live the fact that
// revoked it: the very next call sees the change and, through
// motion_permit_.check() below, revokes before this tick's command (if any)
// is processed by command_router_.update() (see update()'s call order).
// motion_authorization_.operator_authorized is written ONLY by
// CommandRouter's @CALIBRATION MOTION PERMIT GRANT/REVOKE handlers (via the
// modules_.motion_authorization pointer) — this function reads it, never
// sets it, matching the SAME split CalibrationMotionPermit.h documents for
// buildCalibrationMotionPermitFacts()'s two call sites.
void Controller::updateStartupQualification() {
  using Phase=calibration::StartupQualificationPhase;
  const Phase phase=startup_qualification_.phase();
  if (phase==Phase::IDLE || phase==Phase::EXECUTING || phase==Phase::COMPLETE) return;
  if (phase!=Phase::READY && phase!=Phase::REFUSED) {
    if (operating_mode_.mode()!=OperatingMode::MAINTENANCE || system_state_.systemHealth()!=SystemHealth::READY ||
        authority_.current()!=ActuatorAuthority::NONE || millis()-startup_qualification_.startedMs()>10000)
      startup_qualification_.refuse("STARTUP_READ_ONLY_PREREQUISITE_OR_TIMEOUT");
    else switch (phase) {
      case Phase::CENSUS_START:
        if (servo_census_.start()) startup_qualification_.censusStarted();
        else startup_qualification_.refuse("STARTUP_CENSUS_START_FAILED");
        break;
      case Phase::CENSUS_WAIT:
        if (servo_census_.state()==servo::ServoCensus::State::COMPLETE) startup_qualification_.censusComplete();
        break;
      case Phase::PREFLIGHT_START:
        if (servo_preflight_.start()) startup_qualification_.preflightStarted();
        else startup_qualification_.refuse("STARTUP_PREFLIGHT_START_FAILED");
        break;
      case Phase::PREFLIGHT_WAIT:
        if (servo_preflight_.state()==servo::ServoPreflight::State::COMPLETE) {
          calibration::PopulationEvidenceBuildContext context{};
          context.current_observation_bundle=true;context.session_ms=millis();
          startup_qualification_.population(calibration::buildCurrentLegPopulationEvidence(
              servo_census_.result(),servo_preflight_.result(),context));
        }
        break;
      case Phase::OBSERVING: {
        servo::ServoBus::ControlFeedbackSnapshot t{};
        actuator::TelemetrySample s{};
        s.read_ok=servo_bus_.readControlFeedback(startup_qualification_.bus(),&t);
        s.sampled_at_ms=millis();
        if (s.read_ok) {
          s.present_position=t.present_position;s.torque_enable=t.torque_enable;
          s.present_speed=t.present_speed;s.present_current=t.present_current;
          s.present_temperature=t.present_temperature;s.servo_status=t.status;
        }
        // Read-only qualification: a hot/invalid reading refuses admission.
        // The powered executor retains the existing adaptive confirmation.
        startup_qualification_.observe(s,millis());
        break;
      }
      default:break;
    }
  }
  if (!startup_qualification_reported_ && (startup_qualification_.phase()==Phase::READY || startup_qualification_.phase()==Phase::REFUSED)) {
    startup_qualification_reported_=true;
    if (startup_qualification_.phase()==Phase::REFUSED) {
      Serial.printf("CALIBRATION_STARTUP_QUALIFICATION=REFUSED reason=%s bus=%u motion_authorized=0\n",
                    startup_qualification_.reason(),(unsigned)startup_qualification_.failedBus());
      return;
    }
    Serial.printf("CALIBRATION_STARTUP_QUALIFICATION=%s route=%s reference=20261003_134848 geometry=%llx samples=3x12 motion_authorized=0 reason=%s\n",
      startup_qualification_.phase()==Phase::READY ? "PASS" : "REFUSED",
      startup_qualification_.nominal() ? "NOMINAL" : "RF_LOWER_MAX_RETURN",
      (unsigned long long)calibration::kStartupReferenceGeometry,startup_qualification_.reason());
  }
}

// --- Issue #48: conditional RAM-only RESTORE -------------------------------
calibration::RestoreFacts Controller::gatherRestoreFacts(bool read_live,calibration::RecordInspection* in,
                                                         calibration::LivePositionEvidence* live) {
  calibration::RestoreFacts f;
  f.maintenance=operating_mode_.mode()==OperatingMode::MAINTENANCE;
  f.runtime_idle=authority_.current()==ActuatorAuthority::NONE&&!motion_executor_.active()&&
    !motion_executor_.status().safe_off_required&&operational_session_.state()==execution::MotionSessionState::OFF&&
    !motion_service_.busy()&&!calibration_.sessionLive()&&!operational_facts_.operator_permit&&!boot_safe_off_pending_;
  f.nvs_ready=persistence_.snapshot().nvs==calibration::NvsInitStatus::READY;
  f.write_outcome_uncertain=persistence_.writesBlocked();
  f.record_valid_acknowledged=persistence_.calibrationAvailable()&&persistence_.snapshot().assessment.acknowledged_record_intact;
  runtime_restore_.inspect(persistence_,geometry_profile_,actuator::geometry_data::kProvenance,in);
  f.record_inspection_ok=in->ok;f.record_generation=in->generation;f.record_fingerprint=in->fingerprint;
  const auto& pre=servo_preflight_.result();
  f.preflight_complete=servo_preflight_.state()==servo::ServoPreflight::State::COMPLETE&&pre.complete;
  f.preflight_pass=pre.allPass()?12:pre.pass_count>11?11:pre.pass_count;
  const auto& snap=runtime_restore_.snapshot();
  f.already_restored=snap.restored;f.restored_generation=snap.restored_generation;
  *live=calibration::LivePositionEvidence{};
  if(read_live){
    // Twelve read-only register reads; nothing is written to the bus.
    live->present=true;
    for(unsigned i=0;i<12;++i){
      const auto* servo=servo::legServoAt((i/3)*3+2-i%3);servo::ServoBus::RuntimeState st;
      const bool ok=servo&&servo_bus_.readRuntimeState(servo->bus_id,&st);
      if(ok&&st.torque_enable==0)++f.torque_off;
      live->raw[i]=ok&&st.present_position>=0&&st.present_position<4096?static_cast<uint16_t>(st.present_position):4096;
    }
    const uint8_t bad=calibration::firstLivePositionMismatch(*in,*live);
    f.live_positions_inside_record=bad==calibration::kRecordJointCount;f.live_position_failed_joint=bad;
  }
  return f;
}

calibration::RestoreVerdict Controller::requestCalibrationRestore(const calibration::RestoreRequestFacts& q) {
  const bool manual=q.trigger==calibration::RestoreTrigger::OPERATOR_MANUAL;
  const auto attestation=config::installationAttestation(auto_restore_suspended_);
  calibration::RecordInspection in;calibration::LivePositionEvidence live;
  // Twelve read-only register reads (torque + position) back every decision.
  const bool read_live=manual||attestation.auto_restore_enabled;
  const auto facts=gatherRestoreFacts(read_live,&in,&live);
  auto verdict=calibration::evaluateRestore(facts,attestation,q);
  if(verdict.decision==calibration::RestoreDecision::ALLOW_AUTOMATIC||verdict.decision==calibration::RestoreDecision::ALLOW_OPERATOR){
    calibration::RuntimeRestoreRequest request;
    request.expected_generation=facts.record_generation;
    request.current_installation_confirmed=true;  // live evidence (boot) or present operator, decided above
    request.executor_idle=facts.runtime_idle;request.torque_off_confirmed=facts.torque_off==12;
    request.motion_permit_revoked=!operational_facts_.operator_permit;
    const auto result=runtime_restore_.restore(persistence_,geometry_profile_,actuator::geometry_data::kProvenance,
        request,actuator_policy_.transforms(),actuator_policy_.limits(),&live);
    installation_live_verified_=result==calibration::RuntimeRestoreStatus::RESTORED;
    if(result!=calibration::RuntimeRestoreStatus::RESTORED){
      verdict.decision=calibration::RestoreDecision::REFUSED;
      verdict.reason=result==calibration::RuntimeRestoreStatus::REJECT_LIVE_POSITION?calibration::RestoreReason::LIVE_POSITION_OUTSIDE_RECORD:
                     result==calibration::RuntimeRestoreStatus::REJECT_UNSAFE_RUNTIME?calibration::RestoreReason::RUNTIME_NOT_IDLE:
                     calibration::RestoreReason::RECORD_REJECTED;
      verdict.failed_joint=runtime_restore_.snapshot().failed_joint;
    }
  }
  last_restore_verdict_=verdict;last_restore_trigger_=manual?"OPERATOR":"BOOT";
  last_restore_fingerprint_=facts.record_fingerprint;last_restore_generation_=facts.record_generation;
  const auto& snap=runtime_restore_.snapshot();
  {using R=calibration::RestoreReason;const R r=verdict.reason;
   const bool ok=verdict.decision==calibration::RestoreDecision::ALLOW_AUTOMATIC||verdict.decision==calibration::RestoreDecision::ALLOW_OPERATOR||
                 verdict.decision==calibration::RestoreDecision::ALREADY_RESTORED;
   restore_refused_transient_=!ok&&calibration::restoreRefusalIsTransient(r);
   restore_refused_permanent_=!ok&&!restore_refused_transient_;
   restore_refusal_reason_=ok?"":calibration::toString(r);}
  Serial.printf("CALIBRATION_RESTORE_DECISION trigger=%s decision=%s reason=%s failed_joint=%u generation=%lu fingerprint=%016llx "
                "auto=%s restored=%u mapped=%u/12 torque=UNCHANGED_OFF\n",
    last_restore_trigger_,calibration::toString(verdict.decision),calibration::toString(verdict.reason),(unsigned)verdict.failed_joint,
    (unsigned long)facts.record_generation,(unsigned long long)facts.record_fingerprint,
    attestation.auto_restore_enabled?"ENABLED":"SUSPENDED",snap.restored?1u:0u,(unsigned)snap.restored_joints);
  return verdict;
}

void Controller::updateOperationalReadiness(uint32_t now_ms) {
  core::ReadinessFacts f;
  f.now_ms=now_ms;
  f.health_fault=system_state_.systemHealth()==SystemHealth::FAULT;
  f.health_ready=system_state_.systemHealth()==SystemHealth::READY;
  f.boot_safe_off_done=!boot_safe_off_pending_;
  f.census_complete=servo_census_.state()==servo::ServoCensus::State::COMPLETE;
  f.census_pass=f.census_complete&&servo_census_.result().verdict==servo::CensusVerdict::PASS;
  f.preflight_running=servo_preflight_.state()==servo::ServoPreflight::State::RUNNING;
  f.preflight_complete=servo_preflight_.state()==servo::ServoPreflight::State::COMPLETE;
  f.preflight_pass=f.preflight_complete&&servo_preflight_.result().allPass();
  f.persistence_loaded=persistence_.snapshot().load_count>0;
  f.restored=runtime_restore_.snapshot().restored;
  f.restore_refused_permanent=restore_refused_permanent_;f.restore_refused_transient=restore_refused_transient_;
  f.restore_reason=restore_refusal_reason_;
  f.raw_fresh=operational_producer_.rawFresh(now_ms,1000);
  {const auto fb=operational_producer_.snapshot(now_ms,1000);bool all=true;for(unsigned i=0;i<12;++i)all=all&&(fb.read_ok[i]||(operational_session_.state()!=execution::MotionSessionState::OFF&&fb.recovering[i]));f.mapped_fresh=all;if(fb.pending()&&all)f.raw_fresh=true;}
  f.operational_bound=runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&commissioning_bound_;
  f.admit_failed=admit_failed_;f.admit_reason=admit_reason_;
  // now_ms is the loop START; a BMS/IMU sample completed later in this same
  // loop is newer than now_ms. Unsigned now-sampled then underflowed to 'stale'
  // and the Motion Authority flapped GRANTED/REVOKED reason=BMS every 2 s DALY
  // poll (seen on hardware 2026-10-16). Signed age: newer = fresh.
  {const auto imu=imu_.orientationSnapshot();f.imu_fresh=imu.valid&&sampleFresh(now_ms,imu.sampled_at_ms,600);}
  {const auto& b=daly_.sample();f.bms_ok=b.valid&&sampleFresh(now_ms,b.sampled_at_ms,3000);for(unsigned i=0;i<4;++i)f.bms_ok=f.bms_ok&&b.alarms[i]==0;}
  f.power_ok=power_state_.state()==PowerState::RUN;
  f.inhibited=authority_.inhibited();
  f.mode=operating_mode_.mode();
  f.operator_maintenance_hold=operating_mode_.operatorMaintenanceHold();
  f.authority_motion=authority_.current()==ActuatorAuthority::MOTION;
  f.authority_other=authority_.current()!=ActuatorAuthority::NONE&&!f.authority_motion;
  f.session_active=operational_session_.state()!=execution::MotionSessionState::OFF;
  if(!f.authority_motion)motion_authority_lease_={};
  // The operator chose a supervised runtime test mode over USB: no automatic
  // operational admission until reboot or MODE OPERATIONAL.
  const bool operator_runtime_mode=runtime_mode_!=execution::RuntimeTestMode::DIAGNOSTIC&&runtime_mode_!=execution::RuntimeTestMode::OPERATIONAL;
  if(operator_runtime_mode){f.admit_failed=true;f.admit_reason="OPERATOR_SELECTED_RUNTIME_MODE";}
  const auto action=readiness_core_.step(f);
  switch(action){
    case core::ReadinessAction::NONE:break;
    case core::ReadinessAction::START_PREFLIGHT:servo_preflight_.start();break;
    case core::ReadinessAction::RESTORE:{
      calibration::RestoreRequestFacts boot;boot.trigger=calibration::RestoreTrigger::BOOT_AUTOMATIC;
      requestCalibrationRestore(boot);break;}
    case core::ReadinessAction::ADMIT_OPERATIONAL:
      if(operator_runtime_mode||motion_service_.busy()||operational_session_.state()!=execution::MotionSessionState::OFF)break;
      admit_failed_=!admitRuntimePolicy(true,now_ms,now_ms,&admit_reason_);
      Serial.printf("OPERATIONAL_ADMISSION=%s reason=%s\n",admit_failed_?"REFUSED":"BOUND",admit_reason_);
      break;
    case core::ReadinessAction::ENTER_RUN:
      operating_mode_.enterRunAutomatic();authority_.onOperatingModeChanged(OperatingMode::RUN);
      Serial.println("OPERATING_MODE=RUN source=OPERATIONAL_READINESS radio_role=UNCHANGED");break;
    case core::ReadinessAction::GRANT_AUTHORITY:
      if(authority_.request(ActuatorAuthority::MOTION,operating_mode_.mode(),&motion_authority_lease_)!=AuthorityResult::GRANTED)motion_authority_lease_={};
      else Serial.printf("MOTION_AUTHORITY=GRANTED generation=%lu torque=OFF motion=NONE_UNTIL_COMMAND\n",(unsigned long)motion_authority_lease_.generation);
      break;
    case core::ReadinessAction::RELEASE_AUTHORITY:
      authority_.release(motion_authority_lease_);motion_authority_lease_={};
      Serial.printf("MOTION_AUTHORITY=REVOKED reason=%s\n",readiness_core_.status().reason);break;
    case core::ReadinessAction::REVOKE_SESSION:
      // STOP procedure: SAFE_OFF of every joint, then the lease is released.
      motion_service_.disconnected(now_ms);operational_session_.revoke(readiness_core_.status().reason);
      motion_executor_.stop(readiness_core_.status().reason);operational_facts_.operator_permit=false;
      Serial.printf("MOTION_AUTHORITY=REVOKED reason=%s action=SAFE_OFF\n",readiness_core_.status().reason);break;
  }
  if(action!=core::ReadinessAction::NONE)last_readiness_action_=action;
  const auto& st=readiness_core_.status();
  ControllerService::ReadinessView view;view.system_health=st.system_health;view.calibration=st.calibration;
  view.joint_feedback=st.joint_feedback;view.motion_authority=st.motion_authority;view.phase=core::toString(st.phase);view.reason=st.reason;
  service_.setReadinessView(view);
}

bool Controller::operatorPresent(uint32_t now_ms) const {
  return validation_present_&&static_cast<int32_t>(now_ms-validation_presence_ms_)>=0&&now_ms-validation_presence_ms_<=presenceWindowMs();
}

void Controller::HeartbeatHook::heartbeat(uint32_t now_ms) {
  // Only meaningful in OPERATIONAL mode: the USB commissioning deadman keeps
  // its own explicit PRESENT command.
  if(!owner||owner->runtime_mode_!=execution::RuntimeTestMode::OPERATIONAL||!owner->commissioning_bound_)return;
  owner->validation_present_=true;owner->validation_presence_ms_=now_ms;
}

const char* Controller::armOperationalSession(uint32_t now) {
  // preArmCheck may do substantial geometry work. Re-evaluate the SAME
  // producer at admission time; never relax future/stale feedback checks.
  now=millis();
  operational_feedback_=operational_producer_.snapshot(now,reviewed_motion_.execution.feedback_max_age_ms);
  if(runtime_mode_!=execution::RuntimeTestMode::OPERATIONAL||!commissioning_bound_)return "OPERATIONAL_MODE_REQUIRED";
  if(authority_.current()!=ActuatorAuthority::MOTION||!motion_authority_lease_.valid())return "MOTION_AUTHORITY_REQUIRED";
  if(operational_session_.state()!=execution::MotionSessionState::OFF)return "SESSION_BUSY";
  for(unsigned i=0;i<12;++i)if(operational_producer_.raw(i).speed!=0||operational_producer_.raw(i).torque!=0)return "STATIONARY_TORQUE_OFF_REQUIRED_FOR_PRIME";
  // Legacy diagnostic container carries only the route variant and live
  // hardware observation here. No fixture/operator/test attestation is minted.
  execution::LocalTrialEvidence e;e.scope=execution::TrialScope::FIXTURE_COMMISSIONING;e.generation=++validation_sequence_;
  e.imu_fresh=operational_facts_.hardware_qualified;
  e.prime_clamp_max_ticks=commissioning_params_.prime_clamp_ticks;
  operational_facts_.operator_permit=true;operational_facts_.permit_generation=e.generation;operational_facts_.permit_expires_at_ms=e.expires_at_ms;
  operational_facts_.lease=motion_authority_lease_;
  local_trial_evidence_=e;
  // Runtime-governed: no compile-time motion flag is consulted on this path.
  const bool admitted=operational_session_.request(e,operational_facts_,reviewed_motion_.execution,operational_feedback_,true,now);
  // One trace per admission, after the decision, never before the gate. These
  // are the actual inputs, not a later USB telemetry snapshot.
  char trace[kAdmissionTraceBytes];
  const size_t trace_bytes=formatAdmissionTrace(trace,sizeof(trace),now,actuator_policy_.epoch(),
    e.generation,motion_authority_lease_.generation,admitted?"PASS":operational_session_.reason(),operational_feedback_);
  writeAdmissionTrace(Serial,trace,trace_bytes);
  if(!admitted){operational_facts_.operator_permit=false;return operational_session_.reason();}
  const int n=snprintf(trace,sizeof(trace),"OPERATIONAL_SESSION=PRIMING generation=%lu (prime at measured pose -> energize -> arm)\r\n",(unsigned long)e.generation);
  if(n>0&&static_cast<size_t>(n)<sizeof(trace))writeAdmissionTrace(Serial,trace,static_cast<size_t>(n));
  return nullptr;
}

bool Controller::RestoreHook::restoreJson(uint32_t generation,bool operator_confirmed,char* out,size_t cap) {
  if(!owner||!out||!cap)return false;
  calibration::RestoreRequestFacts q;q.trigger=calibration::RestoreTrigger::OPERATOR_MANUAL;
  q.requested_generation=generation;q.operator_confirmed_installation=operator_confirmed;
  const auto v=owner->requestCalibrationRestore(q);
  const auto& snap=owner->runtime_restore_.snapshot();
  const int n=snprintf(out,cap,"{\"version\":1,\"decision\":\"%s\",\"reason\":\"%s\",\"failed_joint\":%d,\"generation\":%lu,"
    "\"fingerprint\":\"%016llx\",\"restored\":%s,\"mapped_joints\":%u,\"limits_approved\":false,\"motion_authorized\":false,"
    "\"torque\":\"OFF_UNCHANGED\",\"authority\":\"%s\"}",
    calibration::toString(v.decision),calibration::toString(v.reason),v.failed_joint<12?(int)v.failed_joint:-1,
    (unsigned long)owner->last_restore_generation_,(unsigned long long)owner->last_restore_fingerprint_,
    snap.restored?"true":"false",(unsigned)snap.restored_joints,
    owner->authority_.current()==ActuatorAuthority::NONE?"NONE":"HELD");
  return n>0&&static_cast<size_t>(n)<cap;
}

void Controller::updateCalibrationMotionPermit() {
  calibration::CalibrationMotionPermitLiveInputs inputs{};
  inputs.operator_calibration_motion_authorized = motion_authorization_.operator_authorized;
  inputs.robot_powered_profile = build::kServoPowerAvailable;
  inputs.mode = operating_mode_.mode();
  inputs.system_health = system_state_.systemHealth();
  inputs.session_active = calibration_.sessionLive();
  inputs.origin = calibration_.status().origin;
  inputs.session_id = calibration_.status().session_id;
  inputs.current_population_pass =
      calibration::populationIsCurrentPass(calibration_.status().population);
  inputs.current_geometry_bound =
      actuator_policy_.currentGeometryTag() != actuator::kNoGeometryProvenance;
  inputs.promoted_transforms_complete =
      actuator_policy_.transforms().size() == calibration::kLegServoSlotCount;
  inputs.startup_recovery_only=startup_qualification_.phase()==calibration::StartupQualificationPhase::EXECUTING;
  inputs.startup_reference_qualified=inputs.startup_recovery_only &&
      full_leg_calibration_.request().startup_recovery && calibration::startupReferenceMatches(geometry_profile_);
  inputs.authority = authority_.current();
  inputs.authority_generation = authority_.generation();
  inputs.authority_inhibited = authority_.inhibited();
  const calibration::CalibrationMotionPermitFacts facts =
      calibration::buildCalibrationMotionPermitFacts(inputs);

  // grant() is deliberately never called here: a permit is never resurrected
  // or (re-)issued implicitly from facts turning healthy. check() only
  // re-verifies an ALREADY-granted permit and revokes on any mismatch;
  // grant() is called from exactly one place, the @CALIBRATION MOTION
  // PERMIT GRANT command handler.
  if (motion_permit_.active()) {
    motion_permit_.check(facts, motion_authorization_.token);
  }

  actuator::CalibrationBootstrapContext ctx{};
  ctx.session_active = facts.session_active;
  ctx.origin = facts.origin;
  ctx.motion_permit_active = motion_permit_.active();
  ctx.motion_permit_generation = motion_permit_.generation();
  ctx.motion_permit_session_id = motion_authorization_.token.session_id;
  ctx.motion_permit_authority_generation = motion_authorization_.token.authority_generation;
  // The operator-approved DIRECTION_VERIFY excursion ceiling granted
  // alongside the permit — independent of motion_permit_active on purpose:
  // SafeActuatorPolicy::evaluate() already refuses any CALIBRATION-owned
  // operation without an active permit BEFORE this budget is ever
  // consulted, so threading it through unconditionally cannot widen access.
  ctx.direction_verify_tick_budget = motion_authorization_.direction_verify_tick_budget;
  // The 24-contact Full Calibration sequence in flight, read from its
  // executor: which leg, which V25 phase, and whether that phase's held
  // prerequisites are verified. SafeActuatorPolicy authorises every sequence
  // move and sequence probe against exactly these facts and its own copy of
  // the geometry-validated sequence plan (evaluateSequenceOperation /
  // evaluateSequenceProbe). Threading them through unconditionally cannot
  // widen access: every CALIBRATION-owned write already needs a live session
  // and an active permit before any of them is consulted. The V5 auxiliary
  // window is never open: no production path parks outside the sequence.
  ctx.auxiliary_parked = false;
  ctx.sequence_active = full_leg_calibration_.sequenceActive();
  ctx.startup_recovery_only=inputs.startup_recovery_only;
  ctx.sequence_leg = full_leg_calibration_.leg();
  ctx.sequence_phase = full_leg_calibration_.sequencePhase();
  ctx.sequence_prerequisites_verified = full_leg_calibration_.prerequisitesVerified();
  full_leg_calibration_.recoveryGrant(&ctx);
  actuator_policy_.setBootstrapContext(ctx);
}

// CR3 continuation, Objective C/D. first_motion_ never touches ServoBus (it
// has no reference to it — see FirstMotionExecutor.h); this is the one place
// its decisions turn into a real bus transaction, and the one place the
// real, independent ServoBus::safeOff() is ever called from this path.
void Controller::updateFirstMotion(uint32_t now_ms) {
  if (first_motion_.active()) {
    // A fresh/current attempt is in progress. A previous attempt's VERIFIED_OFF
    // can never be reused as proof for this one.
    first_motion_safe_off_result_ =
        servo::SafeOffResult::UNVERIFIED_NO_RESPONSE;

    // Every continuation prerequisite is sampled fresh on EVERY Controller
    // tick. FirstMotionExecutor re-checks this snapshot before advancing any
    // active state, including MONITORING.
    calibration::FirstMotionContext context{};
    context.session_active = calibration_.sessionLive();
    context.origin = calibration_.status().origin;
    context.lease = calibration_.authorityLease();
    context.mode = operating_mode_.mode();
    context.motion_permit_active = motion_permit_.active();
    context.authority = authority_.current();
    context.authority_generation = authority_.generation();
    context.authority_inhibited = authority_.inhibited();

    actuator::TelemetrySample sample{};
    bool telemetry_available = false;

    // Runtime telemetry is needed only once the GoalPosition has actually
    // been written. Do not add a bus read in front of TorqueEnable or the
    // GoalPosition transaction.
    if (first_motion_.status().state ==
        calibration::FirstMotionState::MONITORING) {
      servo::ServoBus::RuntimeState state{};
      const bool read_ok =
          servo_bus_.readRuntimeState(first_motion_.busId(), &state);

      telemetry_available = true;
      sample.read_ok = read_ok;
      sample.sampled_at_ms = now_ms;

      if (read_ok) {
        sample.present_position = state.present_position;
        sample.torque_enable = state.torque_enable;
      }
    }

    // At most one state-machine advance / backend write per tick.
    // If this call reaches COMPLETE or SAFE_OFF_REQUIRED, do NOT return:
    // the independent SAFE_OFF path below executes in this SAME Controller
    // tick.
    first_motion_.update(context, now_ms, telemetry_available, sample);
  }

  const calibration::FirstMotionState terminal_state =
      first_motion_.status().state;

  // SAFE_OFF is deliberately outside policy/session/authority/permit.
  //
  // Failure path:
  //   anything after verified Torque ON -> SAFE_OFF_REQUIRED -> SAFE_OFF.
  //
  // Success path:
  //   target ARRIVED -> COMPLETE -> SAFE_OFF.
  //
  // The first CR3 motion therefore NEVER leaves LF_UPPER indefinitely
  // energised after the bounded direction verification. No automatic return
  // to q0 is attempted here: SAFE_OFF is not RESTORE.
  if ((terminal_state ==
           calibration::FirstMotionState::SAFE_OFF_REQUIRED ||
       terminal_state ==
           calibration::FirstMotionState::COMPLETE) &&
      first_motion_safe_off_result_ !=
          servo::SafeOffResult::VERIFIED_OFF) {
    first_motion_safe_off_result_ =
        servo_bus_.safeOff(first_motion_.busId());
  }
}

// The 24-contact Full Calibration sequence. full_leg_calibration_ never
// touches ServoBus itself (no reference to it exists - see
// FullLegCalibrationExecutor.h), so this is the one place its decisions turn
// into real bus transactions:
//   1  calibration telemetry for every bus the executor names (the joint it
//      moves/probes, EVERY held joint, one round-robin bystander);
//   2  update(): at most one policy-gated backend write;
//   3  the phase it is now in, reported to the session in V25 order (a
//      refusal fails the run - the order is the oracle's, not a suggestion);
//   4  the independent, ungated ServoBus::safeOff() for every bus it names,
//      POST-update, so a failure this very tick is made safe this very tick;
//      each result is handed to the NEXT update() and never reused after it.
void Controller::updateFullLegCalibration(uint32_t now_ms) {
  (void)now_ms; // safety ages use the actual acquisition/update clock
  if (!full_leg_calibration_.active()) {
    full_leg_safe_off_frame_ = calibration::FullLegSafeOffFrame{};
    return;
  }

  calibration::FullLegCalibrationContext context{};
  context.startup_motion_permit=motion_authorization_.token.startup_recovery_only;
  context.session_active = calibration_.sessionLive();
  context.origin = calibration_.status().origin;
  context.lease = calibration_.authorityLease();
  context.mode = operating_mode_.mode();
  context.motion_permit_active = motion_permit_.active();
  context.authority = authority_.current();
  context.authority_generation = authority_.generation();
  context.authority_inhibited = authority_.inhibited();

  // 1 - telemetry (two block reads per bus: the V25 per-observation readback).
  uint8_t buses[calibration::kFullLegMaxTelemetry] = {0};
  const uint8_t n = full_leg_calibration_.telemetryRequest(buses, calibration::kFullLegMaxTelemetry);
  calibration::FullLegTelemetryFrame frame{};
  bool thermal_pending = false;
  bool direct_read_used = false;
  for (uint8_t i = 0; i < n; ++i) {
    servo::ServoBus::ControlFeedbackSnapshot t{};
    actuator::TelemetrySample sample{};
    sample.read_ok = servo_bus_.readControlFeedback(buses[i], &t);
    sample.sampled_at_ms = millis();
    if (sample.read_ok) {
      sample.present_position = t.present_position;
      sample.torque_enable = t.torque_enable;
      sample.present_speed = t.present_speed;
      sample.present_current = t.present_current;
      sample.present_temperature = t.present_temperature;
      sample.goal_position = t.goal_position;
      sample.torque_limit = t.torque_limit;
      sample.servo_status = t.status;
      // Do not spend a confirmation transaction on an already fatal status/current.
      uint8_t index = 0;
      while (index < full_leg_calibration_.request().population_count &&
             full_leg_calibration_.request().population[index].bus_id != buses[i]) ++index;
      if (index < calibration::kFullLegPopulation && sample.servo_status == 0 &&
          (sample.present_current & 0x7FFF) < calibration::kSearchHardCurrentAbortRaw) {
        auto& state = thermal_state_[index];
        const bool due = state.directReadDue(millis());
        calibration::ThermalConfirmation thermal = due && direct_read_used && !state.expired(millis()) ? state.result()
            : state.update(&thermal_read_port_, buses[i], sample.present_temperature, millis());
        if (due && !direct_read_used) direct_read_used = true;
        thermal_pending |= thermal.decision == calibration::ThermalDecision::PENDING;
        sample.present_temperature = thermal.decision == calibration::ThermalDecision::PENDING
                                         ? calibration::kThermalLimitC : thermal.published_c;
        if (thermal.decision != calibration::ThermalDecision::NORMAL &&
            thermal.decision != calibration::ThermalDecision::PENDING) {
          // samples[0] is the diagnostic block-read value; the rest are the
          // direct reads the verdict was taken from.
          Serial.printf("CALIBRATION_THERMAL_CONFIRMATION bus=%u decision=%s samples=%ld,%ld,%ld,%ld,%ld "
                        "count=%u published=%ld limit=%ld source=BULK_THEN_DIRECT bulk_artifacts=%u\n",
                        (unsigned)thermal.bus_id, calibration::toString(thermal.decision),
                        (long)thermal.samples[0], (long)thermal.samples[1], (long)thermal.samples[2],
                        (long)thermal.samples[3], (long)thermal.samples[4],
                        (unsigned)thermal.sample_count, (long)thermal.published_c,
                        (long)calibration::kThermalLimitC, (unsigned)thermal.bulk_artifacts);
        }
      }
    }
    frame.add(buses[i], sample);
    if (!sample.read_ok || sample.present_temperature < 0 ||
        sample.present_temperature > calibration::kThermalLimitC || sample.servo_status != 0 ||
        (sample.present_current & 0x7FFF) >= calibration::kSearchHardCurrentAbortRaw) break;
  }

  // 2 - at most one backend write.
  if (thermal_pending) full_leg_calibration_.monitorOnly(context, millis(), frame);
  else full_leg_calibration_.update(context, millis(), frame, full_leg_safe_off_frame_);

  // 3 - the V25 phase order, recorded by the session itself. A recovery-only
  // run is not a leg calibration: it reports no phase (so the leg run that
  // follows in the same session starts its report at PREFLIGHT), and is
  // closed by printInitialRecoveryResult() instead of a finalization.
  if (full_leg_calibration_.request().recovery_only) {
    recovery_result_pending_ = true;
  } else {
    if (full_leg_run_.session_id_at_start != full_leg_reported_session_) {
      full_leg_reported_session_ = full_leg_run_.session_id_at_start;
      full_leg_phase_changes_seen_ = 0;
    }
    if (full_leg_calibration_.status().phase_changes != full_leg_phase_changes_seen_) {
      full_leg_phase_changes_seen_ = full_leg_calibration_.status().phase_changes;
      if (!calibration_.noteExecutionPhase(full_leg_calibration_.status().phase)) {
        full_leg_calibration_.phaseReportRejected();
      }
    }
  }
  printFullLegSequenceEvent();
  printFullLegSearchEvent();
  printFullLegHeldEvents();

  // 4 - SAFE_OFF, outside policy/session/authority/permit, retried every tick
  // until each bus reads back VERIFIED_OFF.
  uint8_t off[calibration::kFullLegPopulation] = {0};
  const uint8_t m = full_leg_calibration_.safeOffRequest(off, calibration::kFullLegPopulation);
  full_leg_safe_off_frame_ = calibration::FullLegSafeOffFrame{};
  for (uint8_t i = 0; i < m; ++i) {
    full_leg_safe_off_frame_.add(off[i],
                                 servo_bus_.safeOff(off[i]) == servo::SafeOffResult::VERIFIED_OFF);
  }
}

// One evidence line per sequence transition (phase / step / joint / held set
// / contacts), so a hardware run shows every V25 stage. Print only.
void Controller::printFullLegSequenceEvent() {
  const calibration::FullLegCalibrationStatus& s = full_leg_calibration_.status();
  const uint32_t key = (static_cast<uint32_t>(s.phase) << 24) | (static_cast<uint32_t>(s.step) << 16) |
                       (static_cast<uint32_t>(s.op_bus) << 8) |
                       (static_cast<uint32_t>(s.held_count) << 4) | s.contacts_accepted;
  if (key == sequence_event_key_ && s.phase_changes == sequence_event_changes_) return;
  sequence_event_key_ = key;
  sequence_event_changes_ = s.phase_changes;
  Serial.printf("CALIBRATION_SEQUENCE leg=%s phase=%s step=%s joint=%s bus=%u target=%u held=%u "
                "contacts=%u/%u recovered=%u prerequisites=%s failure=%s\n",
                calibration::toString(full_leg_calibration_.leg()), calibration::toString(s.phase),
                calibration::toString(s.step), calibration::toString(s.op_joint),
                (unsigned)s.op_bus, (unsigned)s.op_target_tick, (unsigned)s.held_count,
                (unsigned)s.contacts_accepted, (unsigned)calibration::kFullLegContactCount,
                (unsigned)s.recovered_joints,
                full_leg_calibration_.prerequisitesVerified() ? "VERIFIED" : "NO",
                calibration::toString(s.failure));
}

// One unsolicited evidence line per search step / probe state change, so a
// hardware run shows every stage (baseline, coarse transit / scout, release,
// backoff, fine pass 1, backoff, fine pass 2, release) with its geometry and
// telemetry, not only the verdict. Print only; reads status, never state it
// could change.
void Controller::printFullLegSearchEvent() {
  const calibration::ContactProbeStatus& p = full_leg_calibration_.probeStatus();
  const uint8_t exec_phase = static_cast<uint8_t>(full_leg_calibration_.status().phase);
  const uint8_t probe_phase = static_cast<uint8_t>(p.phase);
  if (p.step_count == search_event_steps_ && probe_phase == search_event_probe_phase_ &&
      exec_phase == search_event_exec_phase_ && p.pass == search_event_pass_ &&
      p.plateau_bypass_count == search_event_bypass_) {
    return;
  }
  search_event_steps_ = p.step_count;
  search_event_probe_phase_ = probe_phase;
  search_event_exec_phase_ = exec_phase;
  search_event_pass_ = p.pass;
  search_event_bypass_ = p.plateau_bypass_count;
  if (p.phase == calibration::ContactProbePhase::IDLE) return;

  const calibration::ContactProbeRequest& r = full_leg_calibration_.probeRequest();
  const actuator::CalibrationSearchCorridor& c = r.corridor;
  const long beyond_contact = c.valid() ? static_cast<long>(actuator::searchDepth(c, p.target_tick) -
                                                            actuator::searchDepth(c, c.contact_tick))
                                        : 0L;
  Serial.printf("CALIBRATION_SEARCH exec=%s joint=%s side=%s pass=%u stage=%s probe=%s target=%u "
                "pos=%ld beyond_contact=%ld contact=%u entry=%u guard=%u speed=%ld current=%ld "
                "baseline=%u/%u steps=%u bypass=%u kplateau=%u scout=%u p1=%u p2=%u failure=%s\n",
                calibration::toString(full_leg_calibration_.status().phase),
                calibration::toString(r.endpoint_joint),
                r.endpoint_side == calibration::ContactSide::MIN_SIDE ? "MIN" : "MAX",
                (unsigned)p.pass, calibration::toString(p.stage), calibration::toString(p.phase),
                (unsigned)p.target_tick, (long)p.last_position, beyond_contact,
                (unsigned)c.contact_tick, (unsigned)c.entry_tick, (unsigned)c.guard_tick,
                (long)p.last_speed, (long)p.last_current, (unsigned)p.baseline_median_current,
                (unsigned)calibration::searchBaselineThreshold(p.baseline_median_current,
                                                                p.baseline_mad_current),
                (unsigned)p.step_count, (unsigned)p.plateau_bypass_count,
                (unsigned)p.kinematic_plateau_count,
                p.scout_valid ? (unsigned)p.scout_tick : 0u,
                (unsigned)p.pass1_contact_tick, (unsigned)p.pass2_contact_tick,
                calibration::toString(p.failure));
}

// Held-joint evidence, identifying the exact motor. The held-role failure is
// printed once, on the tick it ended the run; a speed transient of a held
// joint that stayed inside its hold is DIAGNOSTIC ONLY (the executor never
// aborts on speed alone) and bounded by the executor. Print only.
void Controller::printFullLegHeldEvents() {
  const calibration::FullLegHeldObservation& f = full_leg_calibration_.heldRoleFailure();
  if (!f.valid) {
    held_role_failure_printed_ = false;  // a new run
  } else if (!held_role_failure_printed_) {
    held_role_failure_printed_ = true;
    printFullLegHeldObservation("CALIBRATION_HELD_ROLE_FAILURE", f);
  }
  for (uint8_t i = 0; i < full_leg_calibration_.heldSpeedTransientsThisTick(); ++i) {
    printFullLegHeldObservation("CALIBRATION_HELD_SPEED_TRANSIENT",
                                full_leg_calibration_.heldSpeedTransient(i));
  }
}

void Controller::printFullLegHeldObservation(const char* tag,
                                             const calibration::FullLegHeldObservation& o) {
  Serial.printf("%s run_leg=%s phase=%s failure=%s bus=%u leg=%s joint=%s held_target=%u pos=%ld "
                "error=%ld speed=%ld goal=%ld torque=%ld torque_limit=%ld current=%ld temp=%ld "
                "status=%ld sample=%s age_ms=%lu active=%s active_bus=%u active_joint=%s "
                "active_side=%s active_target=%u active_pos=%ld transients=%u\n",
                tag, calibration::toString(full_leg_calibration_.leg()),
                calibration::toString(o.phase), calibration::toString(o.failure),
                (unsigned)o.bus_id, calibration::toString(o.identity.leg),
                calibration::toString(o.identity.joint), (unsigned)o.target_tick,
                (long)o.present_position, (long)o.position_error, (long)o.present_speed,
                (long)o.goal_position, (long)o.torque_enable, (long)o.torque_limit,
                (long)o.present_current, (long)o.present_temperature, (long)o.servo_status,
                o.sample_usable ? "FRESH" : (o.has_sample ? "LAST_GOOD" : "NONE"),
                (unsigned long)o.sample_age_ms,
                o.active_is_probe ? "PROBE" : (o.active_bus != 0 ? "MOVE" : "NONE"),
                (unsigned)o.active_bus, calibration::toString(o.active_joint),
                o.active_side == calibration::ContactSide::MIN_SIDE ? "MIN" : "MAX",
                (unsigned)o.active_target_tick, (long)o.active_position,
                (unsigned)full_leg_calibration_.heldSpeedTransientTotal());
}

// Closes the evidence lifecycle of the armed Full Leg run, exactly once, on
// the tick its executor turns terminal. Everything that decides the verdict
// lives in calibration::finalizeFullLeg() (pure, host-tested); this only
// hands it the live collaborators and keeps the record. The permit and
// authorization it revokes are the same RAM objects the per-tick refresh
// already re-checks, so cleanup between legs is finished before the next
// command line is processed.
bool Controller::verifyStartupNeckOff() {
  return servo_bus_.safeOff(51)==servo::SafeOffResult::VERIFIED_OFF;
}

void Controller::updateFullLegFinalization() {
  if (recovery_result_pending_ && !full_leg_calibration_.active() &&
      full_leg_calibration_.request().recovery_only) {
    // The terminal line of @CALIBRATION INITIAL RECOVERY, printed once. PASS
    // only when the executor COMPLETEd: all twelve actively recovered, each
    // SAFE_OFF verified, all twelve verified at q0 torque-off.
    recovery_result_pending_ = false;
    const calibration::FullLegCalibrationStatus& s = full_leg_calibration_.status();
    const bool startup=full_leg_calibration_.request().startup_recovery;
    const bool neck_off=!startup || verifyStartupNeckOff();
    const bool pass = s.step == calibration::FullLegStep::COMPLETE && neck_off;
    Serial.printf("CALIBRATION_%s_RECOVERY_RESULT verdict=%s recovered=%u/%u failure=%s "
                  "failed_phase=%s last_decision=%s\n",
                  startup ? "STARTUP" : full_leg_calibration_.request().post_abort_recovery ? "POST_ABORT" : "INITIAL",
                  pass ? "PASS" : "FAILED", (unsigned)s.recovered_joints,
                  (unsigned)full_leg_calibration_.request().population_count,
                  neck_off ? calibration::toString(s.failure) : "STARTUP_NECK_SAFE_OFF_UNVERIFIED",
                  s.failure == calibration::FullLegFailure::NONE ? "-"
                                                                 : calibration::toString(s.failed_phase),
                  actuator::toString(s.last_policy_decision));
    if (startup) {
      motion_permit_.revoke(calibration::CalibrationPermitRevokeReason::EXPLICIT);
      motion_authorization_.revoke();calibration_.abortSession();
      startup_qualification_.complete(pass);
      Serial.printf("CALIBRATION_STARTUP_CLOSE safe_off=%s authority=%s motion_authorized=0 reference_admitted=0\n",
                    pass ? "13/13" : "NOT_CERTIFIED",toString(authority_.current()));
    }
  }
  if (!full_leg_run_.armed) return;
  const calibration::FullLegStep step = full_leg_calibration_.status().step;
  if (step != calibration::FullLegStep::COMPLETE && step != calibration::FullLegStep::FAILED) {
    return;
  }

  calibration::FullLegFinalizeContext context{};
  context.manager = &calibration_;
  context.policy = &actuator_policy_;
  context.geometry = &geometry_profile_;
  context.expected_provenance = &actuator::geometry_data::kProvenance;
  context.permit = &motion_permit_;
  context.authorization = &motion_authorization_;
  context.arbiter = &authority_;
  context.parameters = calibration::productionEnvelopeParameters();

  const calibration::FullLegRunOutcome outcome = calibration::outcomeFromExecutor(
      full_leg_calibration_, full_leg_run_.geometry_at_start, full_leg_run_.session_id_at_start);

  calibration::FullLegRecord record{};
  const calibration::FullLegFinalizeFailure failure =
      calibration::finalizeFullLeg(context, full_leg_run_.plan, outcome, &record);
  full_leg_evidence_.put(record);
  full_leg_run_.clear();

  // Printed BEFORE the terminal RESULT record so a host that stops reading at
  // RESULT still has the probe-level cause of any *_PROBE_FAILED.
  const calibration::ContactProbeStatus& probe = full_leg_calibration_.probeStatus();
  const calibration::ContactProbeRequest& pr = full_leg_calibration_.probeRequest();
  Serial.printf("CALIBRATION_FULL_LEG_PROBE_FINAL leg=%s joint=%s side=%s executor_failure=%s "
                "failed_phase=%s probe_phase=%s probe_failure=%s pass=%u stage=%s target=%u "
                "pos=%ld contact=%u guard=%u scout=%u p1=%u p2=%u bypass=%u steps=%u\n",
                calibration::toString(record.leg), calibration::toString(pr.endpoint_joint),
                pr.endpoint_side == calibration::ContactSide::MIN_SIDE ? "MIN" : "MAX",
                calibration::toString(full_leg_calibration_.status().failure),
                calibration::toString(full_leg_calibration_.status().failed_phase),
                calibration::toString(probe.phase), calibration::toString(probe.failure),
                (unsigned)probe.pass, calibration::toString(probe.stage),
                (unsigned)probe.target_tick, (long)probe.last_position,
                (unsigned)pr.corridor.contact_tick, (unsigned)pr.corridor.guard_tick,
                probe.scout_valid ? (unsigned)probe.scout_tick : 0u,
                (unsigned)probe.pass1_contact_tick, (unsigned)probe.pass2_contact_tick,
                (unsigned)probe.plateau_bypass_count, (unsigned)probe.step_count);
  Serial.printf("CALIBRATION_FULL_LEG_CONTACTS leg=%s expected=%u measured=%u accepted=%u "
                "diagnostics_accepted=%s\n",
                calibration::toString(record.leg), (unsigned)record.contacts_expected,
                (unsigned)record.contacts_measured, (unsigned)record.contacts_accepted,
                record.diagnostics_accepted ? "YES" : "NO");
  Serial.printf("CALIBRATION_FULL_LEG_RESULT leg=%s verdict=%s failure=%s\n",
                calibration::toString(record.leg), calibration::toString(record.verdict),
                calibration::toString(failure));
}

void Controller::printBootBanner() {
  Serial.println();
  Serial.println("====================================");
  Serial.printf(" %s %s\n", build::kFirmwareName, build::kFirmwareVersion);
  Serial.println("====================================");
  Serial.printf("build      : %s\n", build::kBuildId);
  Serial.printf("board      : %s\n", build::kBoardName);
  // Profile name and rail facts printed together: both are derived from
  // the same authority (config/HardwareProfile.h), so they can no longer
  // disagree — and the evidence of that is on the boot record.
  Serial.printf("profile    : %s (servo_power=%s battery=%s led_rail=%s)\n",
                build::kTestProfile,
                build::kServoPowerAvailable ? "YES" : "NO",
                build::kBatteryAvailable ? "YES" : "NO",
                build::kLedRailPowered ? "YES" : "NO");
  Serial.printf("servo_pop  : canonical=%u expected_now=%u absent_by_design=%u\n",
                (unsigned)servo::canonicalAllocatedCount(),
                (unsigned)servo::expectedNowCount(),
                (unsigned)servo::absentByDesignCount());
  Serial.printf("servo      : GPIO%d/%d @ %lu\n", pins::kServoTx, pins::kServoRx,
                (unsigned long)build::kServoBusBaud);
  Serial.printf("bms        : GPIO%d/%d @ %lu\n", pins::kDalyTx, pins::kDalyRx,
                (unsigned long)build::kDalyBusBaud);
  Serial.printf("imu        : BNO085 SPI (SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d PS0=%d)\n",
                pins::kBnoSck, pins::kBnoMiso, pins::kBnoMosi, pins::kBnoCs,
                pins::kBnoInt, pins::kBnoRst, pins::kBnoPs0);
  Serial.printf("led        : GPIO%d / %u px\n", pins::kLedRingDin, status::LedRing::kNumPixels);
  // Credentials presence only — never the SSID's passphrase, and never a
  // claim about connectivity: the radio has not been started at this point.
  Serial.printf("wifi       : credentials=%s state=%s (radio starts from update())\n",
                wifi_.status().credentials_present ? "YES" : "NO",
                network::toString(wifi_.status().state));

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running != nullptr) {
    Serial.printf("partition  : %s @ 0x%06x (size 0x%06x)\n",
                  running->label, (unsigned)running->address, (unsigned)running->size);
  }

  // What the bootloader handed us, and whether this image still owes the
  // bootloader a confirmation. Read-only; nothing here confirms anything.
  {
    const update::OtaManagerStatus& o = ota_.status();
    Serial.printf("ota        : img_state=%s rollback_possible=%s ingest=%s build_id=%s\n",
                  update::toString(o.policy.running_image_state),
                  o.policy.rollback_possible ? "YES" : "NO",
                  o.ingest_enabled ? "ENABLED" : "DISABLED (no transport, no auth)",
                  o.running_build_id);
  }

  Serial.printf("reset_reason : %s\n", resetReasonName(esp_reset_reason()));
  Serial.println("startup_motion   : DISABLED");
  Serial.println("startup_torque   : DISABLED");
  Serial.println("startup_servo_census : ENABLED_READ_ONLY_INCREMENTAL");
  Serial.println("daly_write       : KEY_LOGIC_DISCHARGE_ONLY (operator command; no MOS/power-cut write)");
  Serial.printf("operating_mode   : %s\n", toString(operating_mode_.mode()));
  // Two orthogonal axes, printed together so they can never be confused for
  // one. actuator_authority is NONE at boot, always.
  Serial.printf("actuator_authority : %s (inhibit=%s)\n",
                toString(authority_.current()),
                authority_.inhibited() ? toString(authority_.inhibitReason()) : "NONE");
  // The repository's own verdict on the installed robot, on the boot record.
  Serial.printf("calibration      : %s hardware_motion=%s\n",
                calibration_.status().current_calibration_stale
                    ? "STALE_PENDING_FULL_RECALIBRATION" : "SEE_@CALIBRATION_STATUS",
                calibration_.status().hardware_motion_authorized ? "AUTHORIZED" : "BLOCKED");
  Serial.println();
}

bool Controller::bindReviewedMotionRuntime(const execution::ReviewedMotionRuntime& r) {
  const auto& c=runtime_restore_.snapshot();
  if(motion_executor_.active()||authority_.current()!=ActuatorAuthority::NONE||!c.calibration_accepted||
     !c.operational_limits_approved||r.calibration_generation!=c.restored_generation||r.policy_epoch!=actuator_policy_.epoch()||
     !r.execution.approved||!r.max_imu_age_ms||r.max_imu_age_ms>3000||
     r.max_servo_temperature_c<=0||r.max_servo_temperature_c>70||r.min_servo_voltage_dv<=0||
     r.max_servo_voltage_dv<=r.min_servo_voltage_dv)return false;
  if(r.scope!=execution::TrialScope::FIXTURE_MICRO){
    const bool fixture=r.scope==execution::TrialScope::SUPERVISED_FIXTURE_STAND||r.scope==execution::TrialScope::SUPERVISED_FIXTURE_WALK;
    if((!fixture&&r.scope!=execution::TrialScope::AUTONOMOUS_STAND&&r.scope!=execution::TrialScope::AUTONOMOUS_WALK)||
       memcmp(r.route_sha256,motion::pairedProneDigest(r.feedback.fixtureLimitVariant),65)!=0||r.feedback.mode!=(fixture?motion::FeedbackMode::FIXTURE_REVIEW:motion::FeedbackMode::PHYSICAL_REVIEW)||
       !motion::validFeedbackPolicy(r.feedback))return false;
  }
  for(unsigned i=0;i<12&&r.scope!=execution::TrialScope::FIXTURE_MICRO;++i){
    const auto* servo=servo::legServoAt((i/3)*3+2-i%3);calibration::JointIdentity id;
    id.leg=static_cast<calibration::Leg>(i/3);id.joint=static_cast<calibration::JointKind>(i%3);
    strncpy(id.physical_unit,servo->physical_unit,sizeof(id.physical_unit)-1);
    const auto* t=actuator_policy_.transforms().find(id,geometry_profile_.provenanceTag());
    const auto* l=actuator_policy_.limits().find(id,geometry_profile_.provenanceTag());
    if(!t||!l)return false;
    const auto& bound=r.feedback.jointLimits[i/3][i%3];
    const double endpoints[]={bound.lower,bound.upper};
    for(double q:endpoints){uint16_t tick=0;
      if(actuator::resolveUrdfQToRaw(geometry_profile_,actuator::geometry_data::kProvenance,*t,
         static_cast<actuator::MicroRad>(std::llround(q*1e6)),&tick)!=actuator::TargetResolveStatus::OK||!l->contains(tick))return false;
    }
  }
  if(r.scope!=execution::TrialScope::FIXTURE_MICRO){static motion::AcquisitionRoute route;static const motion::AcquisitionRoute kEmptyRoute{};route=kEmptyRoute;route.pairedProne=true;   // off the loop stack
    route.supervisedSegments=true;route.timeScale=r.stand_time_scale;
    route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
    if(!motion_coordinator_.configure(r.feedback,r.walk,&route))return false;}
  reviewed_motion_=r;reviewed_motion_bound_=true;return true;
}
bool Controller::requestLocalMotionTrial(const execution::LocalTrialEvidence& e,uint32_t generation,uint32_t expires) {
  const uint32_t now=millis();
  if(!reviewed_motion_bound_||!execution::runtimeModeAllowsScope(runtime_mode_,e.scope)||e.scope!=reviewed_motion_.scope||!generation||static_cast<int32_t>(now-expires)>=0)return false;
  if(e.scope!=execution::TrialScope::FIXTURE_MICRO&&
     (motion::checkObservation(local_motion_observation_,reviewed_motion_.feedback,motion_clock_.now()/1e6)!=motion::FeedbackReason::OK||
      motion::pairedProneMatch(local_motion_observation_,reviewed_motion_.feedback)<0))return false;
  operational_facts_.operator_permit=true;operational_facts_.permit_generation=generation;operational_facts_.permit_expires_at_ms=expires;
  local_trial_evidence_=e;
  const bool bus=(e.scope==execution::TrialScope::FIXTURE_MICRO||e.scope==execution::TrialScope::SUPERVISED_FIXTURE_STAND||e.scope==execution::TrialScope::SUPERVISED_FIXTURE_WALK)?reviewed_motion_.fixture_bus_timing_qualified:reviewed_motion_.autonomous_bus_timing_qualified;
  operational_facts_.hardware_qualified=operational_facts_.hardware_qualified&&bus;
  operational_facts_.sequential_bus_qualified=operational_facts_.hardware_qualified;
  const bool ok=operational_session_.request(e,operational_facts_,reviewed_motion_.execution,operational_feedback_,(build::kOperationalMotionAuthorized||(build::kSupervisedValidationCapable&&validation_profile_.sealed()&&validation_present_&&now-validation_presence_ms_<=1000&&e.scope==validation_profile_.runtime.scope)),now);
  if(!ok)operational_facts_.operator_permit=false;
  return ok;
}
bool Controller::publishLocalMotionObservation(const motion::AdaptiveObservation& input) {
  if(!reviewed_motion_bound_||reviewed_motion_.scope==execution::TrialScope::FIXTURE_MICRO||
     (operational_session_.state()!=execution::MotionSessionState::ARMED&&operational_session_.state()!=execution::MotionSessionState::OFF))return false;
  auto o=input;o.actual={};o.actual.valid=true;
  const uint32_t now=millis();const auto fb=operational_producer_.snapshot(now,reviewed_motion_.execution.feedback_max_age_ms);
  for(unsigned i=0;i<12;++i)if(!fb.read_ok[i])return false;
  for(unsigned i=0;i<4;++i)o.actual.legs[i]={fb.actual_rad[i*3],fb.actual_rad[i*3+1],fb.actual_rad[i*3+2]};
  {uint64_t oldest=UINT64_MAX;for(unsigned i=0;i<12;++i)if(fb.sampled_at_us[i]<oldest)oldest=fb.sampled_at_us[i];o.stampS=oldest/1e6;}
  if(motion::checkObservation(o,reviewed_motion_.feedback,motion_clock_.now()/1e6)!=motion::FeedbackReason::OK)return false;
  local_motion_observation_=o;motion_coordinator_.observe(o);return true;
}
bool Controller::publishLocalMotionAssessment(const motion::SegmentAssessment& a) {
  return reviewed_motion_bound_&&operational_session_.state()==execution::MotionSessionState::ARMED&&motion_coordinator_.assess(a,millis());
}
bool Controller::requestLocalFixtureMicro(unsigned joint,double delta,double duration,const execution::FrameValidation& path) {
  return operational_session_.requestFixtureMicro(joint,delta,duration,path,operational_facts_,operational_feedback_,millis());
}
void Controller::updateOperationalMotion(uint32_t now_ms) {
  // Boot de-escalation covers every installed servo including the auxiliary.
  // Never prime/energize before all OFF readbacks have succeeded. Retry failures.
  if(build::kServoPowerAvailable&&boot_safe_off_pending_){
    const auto* s=&servo::kCanonicalServos[boot_safe_off_next_];
    if(s->current_config!=servo::CurrentConfig::INSTALLED){boot_safe_off_mask_|=1u<<boot_safe_off_next_;}
    if(s->current_config==servo::CurrentConfig::INSTALLED&&servo_bus_.safeOff(s->bus_id)==servo::SafeOffResult::VERIFIED_OFF)boot_safe_off_mask_|=1u<<boot_safe_off_next_;
    boot_safe_off_next_=(boot_safe_off_next_+1)%servo::kCanonicalServoCount;
    if(boot_safe_off_mask_==((1u<<servo::kCanonicalServoCount)-1))boot_safe_off_pending_=false;
    return;
  }
  if(runtime_mode_!=execution::RuntimeTestMode::OPERATIONAL&&validation_present_&&now_ms-validation_presence_ms_>presenceWindowMs()){
    validation_present_=false;validation_walk_admitted_=false;local_trial_evidence_.local_operator_present=false;
    operational_facts_.operator_permit=false;
    if(operational_session_.state()!=execution::MotionSessionState::OFF)
      operational_session_.revoke("LOCAL_PRESENCE_DEADMAN_EXPIRED");
  }
  // Only continuous teleoperation depends on live transport. STAND is a finite
  // controller command; AP polling is not proof of presence or support.
  if(runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&motion_service_.busy()&&operatorPresent(now_ms))
    motion_service_.renewTeleop(now_ms,static_cast<uint32_t>(reviewed_motion_.walk.maxCommandTtlS*1000));
  const auto& radio=wifi_.adminAccess();const uint32_t generation=radio.generation();
  if((validation_radio_generation_&&validation_radio_generation_!=generation)||radio.closing.load()||radio.radio_fault.load()){
    if(operational_session_.state()!=execution::MotionSessionState::OFF)operational_session_.revoke("RADIO_TRANSITION_REVOKED_LOCAL_SESSION");
    operational_facts_.operator_permit=false;
  }
  validation_radio_generation_=generation;
  // A pending SAFE_OFF goes out before any poll of this tick (Issue #52).
  emergencyTorqueOffBurst();
  // Same cooperative Controller owns every ServoBus operation. Read only when
  // no calibration/discovery producer is using the bus; at most one ID/tick.
  // SAFE_OFF always precedes polling. 100 ms x two reads is still an OPEN WCET
  // gate: this code cannot qualify the sequential bus by executing it.
  const uint32_t poll_started_us=micros();
  if(build::kServoPowerAvailable && !motion_executor_.status().safe_off_required && !operational_session_.safeOffRequired() &&
     (authority_.current()==ActuatorAuthority::NONE||authority_.current()==ActuatorAuthority::MOTION) &&
     servo_bus_.scanState()!=servo::ScanState::RUNNING && servo_preflight_.state()!=servo::ServoPreflight::State::RUNNING && !q0_capture_.active() &&
     (startup_qualification_.phase()==calibration::StartupQualificationPhase::IDLE||
      startup_qualification_.phase()==calibration::StartupQualificationPhase::COMPLETE||
      startup_qualification_.phase()==calibration::StartupQualificationPhase::REFUSED||
      startup_qualification_.phase()==calibration::StartupQualificationPhase::READY) && now_ms-operational_poll_at_ms_>=(profileActive()?2u:20u)){
    const uint32_t transaction_ms=validation_profile_.sealed()?validation_profile_.transaction_timeout_ms:commissioning_bound_?commissioning_params_.transaction_timeout_ms:servo::ServoBus::kOperationalTimeoutMs;
    // ARMING, timed STAND and streaming WALK: three round-robin fast reads
    // per successful tick, stamped individually at completion. Control 40..49
    // rotates at 20 ms (finite) / 100 ms (streaming), with expired-cache retry.
    // Idle, PRIME and ENERGIZE retain one combined read per tick.
    const bool fast=motion_executor_.streaming()||motion_executor_.timedSegmentsActive()||operational_session_.state()==execution::MotionSessionState::ARMING;
    bool poll_ok=true;
    for(unsigned k=0;execution::operationalPollReadAllowed(fast,k,poll_ok);++k){
    // A pending STOP may preempt between synchronous reads, never after a
    // whole new batch. Each individual read keeps its existing IO timeout.
    if(http_transport_.stopPending()&&http_transport_.serviceStopLane(millis())){
      stop_metrics_=StopMetrics{};stop_metrics_.latched_us=motion_clock_.extend(micros());
      emergencyTorqueOffBurst();break;
    }
    const unsigned slot=operational_poll_next_;
    const auto* joint=servo::legServoAt((slot/3)*3+(2-slot%3));
    execution::RawJointObservation r=operational_producer_.raw(slot);
    const uint32_t read_started=millis();
    const uint32_t control_period=motion_coordinator_.streamingWalk()?100:20;
    const uint32_t control_max_age=motion_coordinator_.streamingWalk()?1500:reviewed_motion_.execution.feedback_max_age_ms;
    const bool control_due=!r.read_ok||execution::controlReadDue(fast,slot,operational_control_next_,millis(),
      operational_control_at_ms_,control_period,r.control_at_ms,control_max_age);
    if(control_due){
      servo::ServoBus::ControlFeedbackSnapshot t{};
      r.read_ok=joint&&servo_bus_.readControlFeedbackBudgeted(joint->bus_id,&t,transaction_ms);
      r.torque_limit=t.torque_limit;r.goal_position=t.goal_position;r.goal_speed=t.goal_speed;r.goal_acceleration=t.goal_acceleration;r.torque=t.torque_enable;
      r.speed=t.present_speed<0?-1:t.present_speed&0x7fff;r.speed_signed=t.present_speed<0?0:((t.present_speed&0x8000)?-(t.present_speed&0x7fff):t.present_speed&0x7fff);
      r.position=t.present_position>=0?static_cast<uint16_t>(t.present_position):4096;
      r.temperature=t.present_temperature;r.voltage=t.present_voltage;r.status=t.status;r.load=t.present_load;r.current=t.present_current;r.control_at_ms=millis();
      if(fast&&r.read_ok&&slot==operational_control_next_){operational_control_at_ms_=r.control_at_ms;operational_control_next_=static_cast<uint8_t>((operational_control_next_+1)%12);}
    } else {
      servo::ServoBus::FastFeedback t{};
      r.read_ok=joint&&servo_bus_.readFastFeedback(joint->bus_id,&t,transaction_ms);
      r.speed=t.present_speed<0?-1:t.present_speed&0x7fff;r.speed_signed=t.present_speed<0?0:((t.present_speed&0x8000)?-(t.present_speed&0x7fff):t.present_speed&0x7fff);
      r.position=t.present_position>=0?static_cast<uint16_t>(t.present_position):4096;
      r.temperature=t.present_temperature;r.voltage=t.present_voltage;r.status=t.status;r.load=t.present_load;r.current=t.present_current;
      // Torque state from the slow block, valid only while recent (monitor, not per sample).
      if(millis()-r.control_at_ms>control_max_age)r.read_ok=false;
    }
    if(servo_bus_.lastBusOutcome()==servo::BusOutcome::ABORTED_STOP){
      // A pending STOP abandoned this wait: neither a new sample nor a bus failure, so the
      // previous sample keeps its own timestamp. The STOP is authenticated now; a rejected
      // one simply resumes polling on the next tick.
      if(http_transport_.serviceStopLane(millis())){
        stop_metrics_=StopMetrics{};stop_metrics_.latched_us=motion_clock_.extend(micros());
        emergencyTorqueOffBurst();
      }
      poll_ok=false;break;
    }
    r.transaction_ms=millis()-read_started;
    // The encoder sample instant is the block's completion, before any confirmation read.
    r.sampled_at_ms=millis();r.sampled_at_us=motion_clock_.extend(micros());
    if(profileActive()&&r.transaction_ms>2*transaction_ms+2)r.read_ok=false;
    // dev.3 thermal artifact (2026-10-06 hardware evidence): a MOVING servo can
    // return a spurious >70 C byte inside a valid 56..70 block frame. Confirm a
    // block over-temperature (after the bus budget check of the block
    // itself) with ONE direct register read (the value dev.3
    // decides on); a confirmed over-temperature still invalidates the joint.
    if(r.read_ok&&joint&&r.temperature>70){int direct=-1;
      r.temperature=servo_bus_.readPresentTemperatureDirect(joint->bus_id,&direct,transaction_ms)?direct:r.temperature;}
    operational_producer_.ingest(slot,r,geometry_profile_,actuator::geometry_data::kProvenance,actuator_policy_.transforms());
    poll_ok=r.read_ok&&r.position<4096;
    operational_poll_next_=poll_ok?static_cast<uint8_t>((slot+1)%12):slot;operational_poll_at_ms_=r.sampled_at_ms;
    }
  }
  motion_cycle_.poll_us=static_cast<uint32_t>(micros()-poll_started_us);
  now_ms=millis(); // freshness uses completion time, never the pre-read clock
  operational_feedback_=operational_producer_.snapshot(now_ms,reviewed_motion_bound_?reviewed_motion_.execution.feedback_max_age_ms:500);
  const auto& calibration = runtime_restore_.snapshot();
  MotionReadiness ready;
  // In OPERATIONAL mode the session belongs to the authenticated Dashboard command.
  ready.local_operator_owns_session=reviewed_motion_bound_&&operational_session_.state()!=execution::MotionSessionState::OFF&&
                                    runtime_mode_!=execution::RuntimeTestMode::OPERATIONAL;
  ready.calibration_restored=calibration.restored;
  ready.calibration_accepted=calibration.calibration_accepted;
  ready.limits_approved=calibration.operational_limits_approved||(commissioning_bound_&&calibration.commissioning_limits_active);
  // Offline integration does not qualify physical loaded tracking, contact
  // observation, prime/torque startup or bounded sequential bus timing.
  const auto imu=imu_.orientationSnapshot();
  bool scoped=reviewed_motion_bound_&&execution::runtimeModeAllowsScope(runtime_mode_,reviewed_motion_.scope)&&reviewed_motion_.calibration_generation==calibration.restored_generation&&
      reviewed_motion_.policy_epoch==actuator_policy_.epoch()&&!radio.closing.load()&&!radio.radio_fault.load()&&imu.valid&&sampleFresh(now_ms,imu.sampled_at_ms,reviewed_motion_.max_imu_age_ms);
  bool prime_profiles=true;
  // Reviewed cards or the commissioning policy: the same drive-profile checks.
  // Commissioning reduces no output ceiling (ST3215 default 1000 = no claim).
  // OPERATIONAL fixture WALK (2026-10-15): the reviewed walk drive pair is
  // latched from the first walk frame until the session ends; while servos are
  // rewritten, either reviewed pair is accepted per joint (registers only
  // change on a write), never any other value.
  if(runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&commissioning_bound_){
    // Once ARMED (prime/energize used the commissioning pair) the motion pair of
    // the OperationalWalkProfile.h table is latched: the STAND pair, or the
    // running walk profile's pair while walking.
    if(operational_session_.state()==execution::MotionSessionState::ARMED){walk_drive_latched_=true;
      const auto* profile=motion_coordinator_.walkProfile();walk_drive_=profile?profile->drive:motion::operational_walk::kStandProfile.drive;}
    if(operational_session_.state()==execution::MotionSessionState::OFF)walk_drive_latched_=false;
    servo_bus_.setValidationDrive(walk_drive_latched_?walk_drive_.goalSpeed:commissioning_params_.drive_speed,
                                  walk_drive_latched_?walk_drive_.acceleration:commissioning_params_.drive_acceleration);
  } else walk_drive_latched_=false;
  const uint16_t drive_speed=validation_profile_.sealed()?validation_profile_.drive_speed:walk_drive_latched_?walk_drive_.goalSpeed:commissioning_params_.drive_speed;
  const uint8_t drive_acceleration=validation_profile_.sealed()?validation_profile_.drive_acceleration:walk_drive_latched_?walk_drive_.acceleration:commissioning_params_.drive_acceleration;
  // While latched, a joint may still hold any OTHER reviewed pair of the table
  // until its next frame rewrites it (registers change only on a write).
  auto reviewedPair=[&](uint16_t speed,uint8_t acceleration){
    if(speed==drive_speed&&acceleration==drive_acceleration)return true;
    if(!walk_drive_latched_)return false;
    if(speed==commissioning_params_.drive_speed&&acceleration==commissioning_params_.drive_acceleration)return true;
    for(const auto* p:motion::operational_walk::kProfiles)if(speed==p->drive.goalSpeed&&acceleration==p->drive.acceleration)return true;
    if(speed==motion::operational_walk::kStandProfile.drive.goalSpeed&&acceleration==motion::operational_walk::kStandProfile.drive.acceleration)return true;
    return false;};
  const int output_ceiling=validation_profile_.sealed()?validation_profile_.output_ceiling:1000;
  for(unsigned i=0;i<12;++i){const auto& raw=operational_producer_.raw(i);
    if(operational_feedback_.recovering[i]){prime_profiles=false;continue;}
    scoped=scoped&&raw.read_ok&&raw.temperature<=reviewed_motion_.max_servo_temperature_c&&
      raw.voltage>=reviewed_motion_.min_servo_voltage_dv&&raw.voltage<=reviewed_motion_.max_servo_voltage_dv;
    if(profileActive()){
      scoped=scoped&&raw.torque_limit>=0&&raw.torque_limit<=output_ceiling&&raw.speed>=0&&raw.speed<=drive_speed;
      prime_profiles=prime_profiles&&raw.goal_speed==drive_speed&&raw.goal_acceleration==drive_acceleration;
      if(operational_session_.state()==execution::MotionSessionState::ARMED)
        scoped=scoped&&reviewedPair(raw.goal_speed,raw.goal_acceleration);
    }
  }
  operational_facts_.prime_profiles_verified=profileActive()&&prime_profiles;
  // Diagnosis only (no new condition): which part of the hardware scope was lost. First field attempt (17/10)
  // was revoked 150 ms after ENERGIZE with one aggregate reason while two clamped LOWERs were moving 4 counts
  // inward - the gate cannot tell a speed above the prime drive profile from a stale IMU or a failed read.
  const char* scope_loss=nullptr;
  if(!scoped){
    if(!reviewed_motion_bound_)scope_loss="SCOPE_REVIEWED_MOTION_NOT_BOUND";
    else if(!execution::runtimeModeAllowsScope(runtime_mode_,reviewed_motion_.scope))scope_loss="SCOPE_RUNTIME_MODE";
    else if(reviewed_motion_.calibration_generation!=calibration.restored_generation)scope_loss="SCOPE_CALIBRATION_GENERATION_CHANGED";
    else if(reviewed_motion_.policy_epoch!=actuator_policy_.epoch())scope_loss="SCOPE_POLICY_EPOCH_CHANGED";
    else if(radio.closing.load()||radio.radio_fault.load())scope_loss="SCOPE_RADIO_TRANSITION";
    else if(!imu.valid)scope_loss="SCOPE_IMU_INVALID";
    else if(!sampleFresh(now_ms,imu.sampled_at_ms,reviewed_motion_.max_imu_age_ms))scope_loss="SCOPE_IMU_STALE";
    else for(unsigned i=0;i<12&&!scope_loss;++i){const auto& raw=operational_producer_.raw(i);
      if(!raw.read_ok)scope_loss="SCOPE_SERVO_READ_FAILED";
      else if(raw.temperature>reviewed_motion_.max_servo_temperature_c)scope_loss="SCOPE_SERVO_OVER_TEMPERATURE";
      else if(raw.voltage<reviewed_motion_.min_servo_voltage_dv||raw.voltage>reviewed_motion_.max_servo_voltage_dv)scope_loss="SCOPE_SERVO_VOLTAGE_WINDOW";
      else if(profileActive()){
        if(!(raw.torque_limit>=0&&raw.torque_limit<=output_ceiling))scope_loss="SCOPE_SERVO_TORQUE_LIMIT";
        else if(!(raw.speed>=0&&raw.speed<=drive_speed))scope_loss="SCOPE_SERVO_SPEED_ABOVE_DRIVE_PROFILE";
        else if(operational_session_.state()==execution::MotionSessionState::ARMED&&!reviewedPair(raw.goal_speed,raw.goal_acceleration))scope_loss="SCOPE_SERVO_DRIVE_PAIR";
      }
    }
  }
  ready.hardware_approved=scoped;
  ready.hardware_reason=scope_loss?scope_loss:"HARDWARE_UNAVAILABLE";
  if(validation_walk_admitted_){
    const char* state=motion_coordinator_.state();
    if(!strcmp(state,"WALK_STARTING")||!strcmp(state,"WALK")||!strcmp(state,"STOPPING"))validation_walk_started_=true;
    else if(validation_walk_started_){validation_walk_admitted_=false;validation_walk_started_=false;}
  }
  ready.walk_qualified=reviewed_motion_bound_&&validation_walk_admitted_;
  // WALK availability depends on its executable profile and live limits.
  // The Coordinator owns the automatic transition through STAND.
  if(runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&commissioning_bound_){
    ready.walk_qualified=operational_walk_bound_&&reviewed_motion_bound_;
  }
  ready.first_stride_only=reviewed_motion_bound_&&reviewed_motion_.walk.firstStrideOnly;
  ready.max_vx=reviewed_motion_.walk.maxVxMps;ready.max_vy=reviewed_motion_.walk.maxVyMps;ready.max_yaw_rate=reviewed_motion_.walk.maxYawRateRadS;
  ready.operational_permit=operational_facts_.operator_permit;
  auto observation=local_motion_observation_;observation.actual={};observation.actual.valid=true;
  for(unsigned i=0;i<12;++i)observation.actual.valid=observation.actual.valid&&operational_feedback_.read_ok[i];
  for(unsigned i=0;i<4;++i)observation.actual.legs[i]={operational_feedback_.actual_rad[i*3],operational_feedback_.actual_rad[i*3+1],operational_feedback_.actual_rad[i*3+2]};
  for(unsigned i=0;i<12;++i)observation.stationary=observation.stationary&&operational_producer_.raw(i).speed==0;
  // Encoder timestamp remains the oldest real sample. The operational route
  // uses model support assumptions, never a presence heartbeat as measurement.
  {uint64_t oldest=UINT64_MAX;for(unsigned i=0;i<12;++i)if(operational_feedback_.sampled_at_us[i]<oldest)oldest=operational_feedback_.sampled_at_us[i];
   observation.stampS=observation.actual.valid&&oldest!=UINT64_MAX?oldest/1e6:0;
   observation.externalSupportConfirmed=local_motion_observation_.externalSupportConfirmed&&(commissioning_bound_&&runtime_mode_!=execution::RuntimeTestMode::OPERATIONAL?operatorPresent(now_ms):true);}
  // Diagnostic fixture declarations remain local to diagnostics. Operational
  // geometry uses its configured support model and live joint limits.
  ready.support_observable=commissioning_bound_?
    ((runtime_mode_==execution::RuntimeTestMode::OPERATIONAL||operatorPresent(now_ms))&&local_motion_observation_.externalSupportConfirmed):
    reviewed_motion_bound_&&reviewed_motion_.scope!=execution::TrialScope::FIXTURE_MICRO&&
    motion::checkObservation(observation,reviewed_motion_.feedback,motion_clock_.now()/1e6)==motion::FeedbackReason::OK;
  ready.local_fixture_session=operational_session_.state()==execution::MotionSessionState::ARMED&&
    operational_session_.scope()==execution::TrialScope::FIXTURE_MICRO;
  motion_coordinator_.observe(observation);
  ready.mode=operating_mode_.mode();
  ready.authority_live=authority_.current()==ActuatorAuthority::MOTION;
  ready.inhibited=authority_.inhibited();
  ready.fault=system_state_.systemHealth()==SystemHealth::FAULT;
  ready.power_ok=power_state_.state()==PowerState::RUN;
  const auto& battery=daly_.sample();
  ready.bms_ok=battery.valid && sampleFresh(now_ms,battery.sampled_at_ms,3000);
  for(unsigned i=0;i<4;i++)ready.bms_ok=ready.bms_ok && battery.alarms[i]==0;
  operational_facts_.mode=ready.mode;
  operational_facts_.calibration_accepted=ready.calibration_accepted;
  operational_facts_.envelopes_approved=ready.limits_approved;
  operational_facts_.hardware_qualified=ready.hardware_approved;
  operational_facts_.sequential_bus_qualified=ready.hardware_approved;
  local_trial_evidence_.imu_fresh=scoped;
  local_trial_evidence_.scope_loss=scope_loss;
  local_trial_evidence_.local_operator_present=operatorPresent(now_ms);
  local_trial_evidence_.support_load_observable=reviewed_motion_.feedback.mode==motion::FeedbackMode::PHYSICAL_REVIEW&&ready.support_observable;
  if(reviewed_motion_.feedback.mode==motion::FeedbackMode::FIXTURE_REVIEW){
    local_trial_evidence_.local_operator_present=operatorPresent(now_ms);
    local_trial_evidence_.fixture_support_verified=ready.support_observable;
  }
  operational_facts_.bms_ok=ready.bms_ok;
  operational_facts_.power_ok=ready.power_ok;
  operational_facts_.fault=ready.fault;
  // Validate measured support/pose before the next actuator step. A revoked
  // fixture or lost held contact must stop WRITING without waiting for its ACK.
  const auto session_before=operational_session_.state();
  const bool executor_stopped_before=motion_executor_.status().safe_off_required;
  operational_session_.update(local_trial_evidence_,operational_facts_,operational_feedback_,now_ms);
  if(session_before!=operational_session_.state()&&operational_session_.state()==execution::MotionSessionState::SAFE_OFF_REQUIRED&&
     (session_before==execution::MotionSessionState::PRIMING||session_before==execution::MotionSessionState::ENERGIZING||session_before==execution::MotionSessionState::ARMING)){
    static char line[160];   // static: this loop function is on the stack-budgeted WALK chain
    const int n=snprintf(line,sizeof(line),"OPERATIONAL_SESSION=REVOKED reason=%s joint=%d detail=%s\r\n",operational_session_.lastRevoke(),
                         operational_session_.failedJoint(),operational_session_.failureDetail());
    if(n>0&&static_cast<size_t>(n)<sizeof(line))writeAdmissionTrace(Serial,line,static_cast<size_t>(n));
  }
  if(session_before!=operational_session_.state()){
    char trace[160];const int n=snprintf(trace,sizeof(trace),"MOTION_PHASE now_ms=%lu state=%s reason=%s\r\n",
      (unsigned long)millis(),operational_session_.stateName(),operational_session_.reason());
    if(n>0&&static_cast<size_t>(n)<sizeof(trace))writeAdmissionTrace(Serial,trace,n);
  }
  if(operational_session_.state()==execution::MotionSessionState::ARMED)operational_facts_.lease=operational_session_.lease();
  // The stride's internal TTL lives only as long as the operator heartbeat.
  // During the coordinated stop too: losing the heartbeat at any phase = SAFE_OFF.
  if(runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&commissioning_bound_&&motion_coordinator_.walking()&&operatorPresent(now_ms)){
    motion_coordinator_.renewWalk(now_ms);motion_service_.renewTeleop(now_ms,static_cast<uint32_t>(reviewed_motion_.walk.maxCommandTtlS*1000));}
  if(session_before!=execution::MotionSessionState::ARMED&&operational_session_.state()==execution::MotionSessionState::ARMED)
    operational_work_max_us_=0;
  const uint32_t work_started_us=micros();
  const char* const phase_before_work=motion_coordinator_.state();
  motion_coordinator_.update(operational_facts_,operational_feedback_,now_ms,motion_clock_.extend(micros()));
  if(profileActive()&&(runtime_mode_==execution::RuntimeTestMode::OPERATIONAL||validation_present_)&&
     operational_session_.state()==execution::MotionSessionState::ARMED&&ready.support_observable){
    const auto& frame=motion_coordinator_.frame();
    if(frame.target.valid&&!frame.executable){
      motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=motion_clock_.now()/1e6;
      a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;a.geometryIdentityVerified=true;
      a.sweptCollisionClear=a.continuousBoundsConfirmed=a.supportCompatible=true;
      a.fixtureTrialEnvelopeReviewed=reviewed_motion_.feedback.mode==motion::FeedbackMode::FIXTURE_REVIEW;
      a.loadedTrackingQualified=reviewed_motion_.feedback.mode==motion::FeedbackMode::PHYSICAL_REVIEW;
      a.dynamicStabilityQualified=reviewed_motion_.scope==execution::TrialScope::AUTONOMOUS_WALK&&local_trial_evidence_.loaded_dynamics_qualified;
      // These assertions consume the sealed, externally reviewed tube proof,
      // not an encoder/IMU inference. No profile is distributed pre-approved.
      if(publishLocalMotionAssessment(a))
        motion_coordinator_.update(operational_facts_,operational_feedback_,millis(),motion_clock_.extend(micros()));
    }
  }
  const uint32_t planner_us=static_cast<uint32_t>(micros()-work_started_us);
  operational_work_max_us_=std::max(operational_work_max_us_,planner_us);
  const uint32_t executor_started_us=micros();
  motion_executor_.update(operational_facts_,operational_feedback_,millis(),motion_clock_.extend(micros()));
  const uint32_t executor_us=static_cast<uint32_t>(micros()-executor_started_us);
  emergencyTorqueOffBurst(); // physical stop precedes diagnostics and telemetry
  if(!executor_stopped_before&&motion_executor_.status().safe_off_required){
    char trace[192];const int n=snprintf(trace,sizeof(trace),"MOTION_FIRST_STOP now_ms=%lu session=%s reason=%s tx_sequence=%lu\r\n",
      (unsigned long)millis(),operational_session_.stateName(),motion_executor_.status().reason,
      (unsigned long)motion_executor_.status().tx_sequence);
    if(n>0&&static_cast<size_t>(n)<sizeof(trace))writeAdmissionTrace(Serial,trace,n);
  }
  if(!executor_stopped_before&&motion_executor_.status().safe_off_required){
    char budget[384];const int n=snprintf(budget,sizeof(budget),
      "MOTION_SERVICE_BUDGET phase=%s gap_us=%lu before_motion_us=%lu imu_us=%lu bms_us=%lu poll_us=%lu planner_us=%lu executor_us=%lu\r\n",
      phase_before_work,(unsigned long)motion_cycle_.gap_us,(unsigned long)motion_cycle_.before_motion_us,
      (unsigned long)motion_cycle_.imu_us,(unsigned long)motion_cycle_.bms_us,(unsigned long)motion_cycle_.poll_us,
      (unsigned long)planner_us,(unsigned long)executor_us);
    if(n>0&&static_cast<size_t>(n)<sizeof(budget))writeAdmissionTrace(Serial,budget,n);
    // IMU service evidence (own task): arrival spacing and age of the Rotation Vector the
    // Motion gate judged, and whether the sensor was ever left with data pending.
    const auto& is=imu_.serviceStats();const auto io=imu_.orientationSnapshot();
    const int n2=snprintf(budget,sizeof(budget),
      "MOTION_IMU_SERVICE task=%u rv=%lu rv_age_ms=%ld rv_gap_last_us=%lu rv_gap_max_us=%lu sensor_age_us=%ld service_max_us=%lu service_gap_max_us=%lu budget_stops=%lu backlog_stops=%lu resets=%lu\r\n",
      imu_.taskRunning()?1u:0u,(unsigned long)is.rv,io.valid?static_cast<long>(static_cast<int32_t>(millis()-io.sampled_at_ms)):-1L,
      (unsigned long)is.rv_gap_last_us,(unsigned long)is.rv_gap_max_us,(long)io.sensor_age_us,(unsigned long)is.max_service_us,
      (unsigned long)is.max_gap_us,(unsigned long)is.budget_stops,(unsigned long)is.backlog_stops,(unsigned long)is.resets);
    if(n2>0&&static_cast<size_t>(n2)<sizeof(budget))writeAdmissionTrace(Serial,budget,n2);
  }
  // One bounded record at the first stop only; samples retain their real age.
  if(!executor_stopped_before&&motion_executor_.status().safe_off_required){
    const uint32_t at=millis();unsigned oldest=0;
    for(unsigned i=1;i<12;++i)if(at-operational_feedback_.sampled_at_ms[i]>at-operational_feedback_.sampled_at_ms[oldest])oldest=i;
    const auto& failure=motion_executor_.status();
    if(failure.feedback_failed_joint>=0)oldest=static_cast<unsigned>(failure.feedback_failed_joint);
    const auto& raw=operational_producer_.raw(oldest);
    char detail[384];const int n=snprintf(detail,sizeof(detail),
      "MOTION_FEEDBACK_STOP checker=%s checked_ms=%lu budget_ms=%lu joint=%u sample_ms=%lu age_ms=%lu read_ok=%u recovering=%u raw_ok=%u control_ms=%lu poll_next=%u poll_ms=%lu transaction_ms=%lu executor_age_ms=%lu session_age_ms=%lu frames_tx=%lu work_max_us=%lu\r\n",
      failure.feedback_failed_joint>=0?"EXECUTOR":"SESSION_OR_COORDINATOR",(unsigned long)failure.feedback_checked_ms,
      (unsigned long)failure.feedback_budget_ms,oldest,(unsigned long)operational_feedback_.sampled_at_ms[oldest],(unsigned long)(at-operational_feedback_.sampled_at_ms[oldest]),
      operational_feedback_.read_ok[oldest],operational_feedback_.recovering[oldest],raw.read_ok,(unsigned long)raw.control_at_ms,
      operational_poll_next_,(unsigned long)operational_poll_at_ms_,(unsigned long)raw.transaction_ms,
      (unsigned long)std::min(reviewed_motion_.execution.feedback_max_age_ms,execution::MotionExecutor::kTimedFeedbackMaxAgeMs),
      (unsigned long)reviewed_motion_.execution.feedback_max_age_ms,(unsigned long)motion_executor_.status().frames_tx,(unsigned long)operational_work_max_us_);
    if(n>0&&static_cast<size_t>(n)<sizeof(detail))writeAdmissionTrace(Serial,detail,n);
  }
  ready.operational_permit=operational_facts_.operator_permit&&operational_session_.state()==execution::MotionSessionState::ARMED;
  // Compatibility telemetry only: operational availability is determined by
  // live hardware/authority; no operator permit is consumed by MotionService.
  if(runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&commissioning_bound_&&
     authority_.current()==ActuatorAuthority::MOTION&&
     operational_session_.state()!=execution::MotionSessionState::SAFE_OFF_REQUIRED)ready.operational_permit=true;
  // Terminal service results must use this tick's validated planner/executor.
  motion_service_.refresh(ready,now_ms);
  MotionTelemetry telemetry;telemetry.validation_capable=build::kSupervisedValidationCapable;telemetry.fixture_scope=reviewed_motion_bound_&&reviewed_motion_.feedback.mode==motion::FeedbackMode::FIXTURE_REVIEW;telemetry.load_scope=reviewed_motion_bound_&&reviewed_motion_.feedback.mode==motion::FeedbackMode::PHYSICAL_REVIEW;telemetry.foot_load_proven=telemetry.load_scope&&ready.support_observable&&observation.confirmedFeet==15;telemetry.supervised_segment=motion_coordinator_.proneSegment();telemetry.session_profile_sha256=validation_profile_.sealed()?validation_profile_.workspace.artifact_sha256:"";telemetry.session_state=operational_session_.stateName();telemetry.session_reason=operational_session_.reason();
  telemetry.session_last_revoke=operational_session_.lastRevoke();telemetry.session_failure=operational_session_.failureDetail();telemetry.session_failed_joint=operational_session_.failedJoint();
  const auto orientation=imu_.orientationSnapshot();
  telemetry.quaternion_valid=orientation.valid;
  telemetry.quaternion_at_ms=orientation.sampled_at_ms;
  telemetry.height_valid=ready.support_observable;
  if(telemetry.height_valid)telemetry.base_height_m=observation.body.translationM.z;
  for(unsigned i=0;i<4;i++)telemetry.quaternion[i]=orientation.wxyz[i];
  telemetry.source="HARDWARE_SERVO_FEEDBACK";
  telemetry.feedback_recovering=operational_feedback_.pending();
  telemetry.feedback_max_age_ms=motion_executor_.timedSegmentsActive()||motion_executor_.streaming()?execution::MotionExecutor::kTimedFeedbackMaxAgeMs:reviewed_motion_bound_?reviewed_motion_.execution.feedback_max_age_ms:500;
  telemetry.raw_valid=operational_producer_.rawFresh(now_ms,telemetry.feedback_max_age_ms);telemetry.joints_valid=telemetry.raw_valid;
  telemetry.sampled_at_ms=now_ms;
  for(unsigned i=0;i<12;++i){
    telemetry.joints_valid=telemetry.joints_valid&&operational_feedback_.read_ok[i];
    telemetry.actual_rad[i]=operational_feedback_.actual_rad[i];
    telemetry.joint_at_ms[i]=operational_feedback_.sampled_at_ms[i];
    telemetry.sampled_at_ms=static_cast<int32_t>(now_ms-telemetry.joint_at_ms[i])>static_cast<int32_t>(now_ms-telemetry.sampled_at_ms)?telemetry.joint_at_ms[i]:telemetry.sampled_at_ms;
    telemetry.raw_ticks[i]=operational_producer_.raw(i).position;
    telemetry.joint_mapped[i]=operational_feedback_.read_ok[i];
    telemetry.raw_ok[i]=operational_producer_.raw(i).read_ok&&operational_producer_.raw(i).position<4096;
  }
  {static const auto head=motion::pairedProneSample(0,0,true); // immutable route head, no per-tick planner work
   telemetry.prone_reference_valid=calibration.restored&&commissioning_bound_&&head.target.valid;
   telemetry.prone_stationary=true;
   telemetry.prone_max_deviation_rad=commissioning_params_.prone_max_deviation_rad;
   telemetry.prone_max_rms_rad=commissioning_params_.prone_max_rms_rad;
   for(unsigned i=0;i<12;++i){const auto& q=head.target.legs[i/3];
    telemetry.prone_nominal_rad[i]=i%3==0?q.hip:i%3==1?q.upper:q.lower;
    telemetry.prone_stationary=telemetry.prone_stationary&&operational_producer_.raw(i).speed==0;}}
  telemetry.restore_decision=calibration::toString(last_restore_verdict_.decision);
  telemetry.restore_reason=calibration::toString(last_restore_verdict_.reason);
  if(!strcmp(last_restore_trigger_,"NONE")){telemetry.restore_decision="NOT_EVALUATED";telemetry.restore_reason="BOOT_EVALUATION_PENDING";}
  {const auto& v=service_.readinessView();telemetry.system_health=v.system_health;telemetry.calibration_state=v.calibration;
   telemetry.joint_feedback=v.joint_feedback;telemetry.motion_authority=v.motion_authority;telemetry.readiness_phase=v.phase;telemetry.readiness_reason=v.reason;
   telemetry.operating_mode=operating_mode_.mode()==OperatingMode::RUN?"RUN":"MAINTENANCE";telemetry.entry_reason=motion_coordinator_.lastEntryReason();
   telemetry.operator_present=operatorPresent(now_ms);
   telemetry.walk=runtime_mode_==execution::RuntimeTestMode::OPERATIONAL&&commissioning_bound_?(motion_coordinator_.walking()?motion_coordinator_.state():operational_walk_reason_):"WALK_NOT_BOUND";
   telemetry.walk_cycles=motion_coordinator_.walkCycles();telemetry.walk_drive=walk_drive_latched_&&motion_coordinator_.walkProfile()!=nullptr;}
  telemetry.restore_trigger=last_restore_trigger_;telemetry.restore_generation=last_restore_generation_;
  telemetry.restore_fingerprint=last_restore_fingerprint_;
  // base_link height/contact stay unknown until independent measured
  // observations and runtime transforms exist; IMU never invents height.
  updateLivePose();
  telemetry.runtime_mode=execution::toString(runtime_mode_);
  telemetry.limits_scope=calibration.commissioning_limits_active?"COMMISSIONING_CONTACT_INSET":calibration.operational_limits_approved?"REVIEWED_ROUTE":"NONE";
  telemetry.restore=calibration.restored?"RESTORED":calibration::toString(calibration.restore_status);
  telemetry.approved_joints=calibration.approved_joints;telemetry.commissioning_joints=calibration.commissioning_joints;
  telemetry.pose_class=live_pose_valid_?(live_pose_.kind==motion::FixturePoseKind::PRONE_COMPATIBLE?"PRONE_COMPATIBLE":"UNKNOWN"):"NOT_EVALUATED";
  telemetry.pose_reason=live_pose_valid_?motion::fixturePoseReasonName(live_pose_.reason):(calibration.restored?"FEEDBACK_STALE":"RESTORE_REQUIRED");
  telemetry.pose_joint=live_pose_valid_?static_cast<int>(live_pose_.failedJointIndex<12?live_pose_.failedJointIndex:live_pose_.maxErrorJointIndex):-1;
  telemetry.pose_max_dev_rad=live_pose_valid_?live_pose_.maxNominalErrorRad:-1;telemetry.pose_rms_dev_rad=live_pose_valid_?live_pose_.rmsNominalErrorRad:-1;
  {const auto& stand=motion_coordinator_.stand();telemetry.entry=!stand.entryActive()?"NONE":!stand.entryPending()?"DONE":stand.entryStage()==0?"E0":"E1";
   const char* phases[]={"PRE_ALIGN","REAR_DEPLOY","FRONT_DEPLOY","LOAD_TRANSFER","RISE"};const unsigned ph=static_cast<unsigned>(motion::pairedPronePhase(motion_coordinator_.proneSegment()));
   telemetry.phase=commissioning_bound_?phases[ph<5?ph:4]:"";}
  telemetry.commissioning_reason=commissioning_reason_;telemetry.fixture_declared=commissioning_bound_&&ready.support_observable;
  telemetry.prime_clamp_mask=operational_session_.primeClampMask();
  telemetry.session_profile_sha256=commissioning_bound_?commissioning_entry_.sha256:telemetry.session_profile_sha256;
  motion_service_.publishTelemetry(telemetry);
  emitTelemetry(now_ms);
  if(motion_executor_.status().safe_off_required||operational_session_.safeOffRequired()) {
    // SAFE_OFF is ungated and independent of owner, mode, permit and policy.
    // One leg joint per tick; repeatedly verify every failed joint.
    emergencyTorqueOffBurst();
    // Then independent verification of every joint (readback only); a joint
    // that does not verify gets the reviewed unicast OFF write + readback.
    const auto* joint=servo::legServoAt(motion_safe_off_next_);
    const uint32_t verify_ms=validation_profile_.sealed()?validation_profile_.transaction_timeout_ms:commissioning_bound_?commissioning_params_.transaction_timeout_ms:servo::ServoBus::kOperationalTimeoutMs;
    if(joint&&(servo_bus_.verifyTorqueOff(joint->bus_id,verify_ms)==servo::SafeOffResult::VERIFIED_OFF||servo_bus_.safeOff(joint->bus_id)==servo::SafeOffResult::VERIFIED_OFF))
      motion_safe_off_mask_|=static_cast<uint16_t>(1u<<motion_safe_off_next_);
    if(motion_safe_off_next_==11&&stop_metrics_.verify_rounds<255)++stop_metrics_.verify_rounds;
    motion_safe_off_next_=static_cast<uint8_t>((motion_safe_off_next_+1)%12);
    if(motion_safe_off_mask_==0x0fff){
      stop_metrics_.all_off_verified_us=motion_clock_.extend(micros());
      Serial.printf("MOTION_SAFE_OFF=VERIFIED latch_to_off_tx_us=%llu off_tx_us=%llu latch_to_verified_us=%llu last_target_tx_us=%llu rounds=%u\n",
        (unsigned long long)(stop_metrics_.off_tx_start_us-stop_metrics_.latched_us),(unsigned long long)(stop_metrics_.off_tx_done_us-stop_metrics_.off_tx_start_us),
        (unsigned long long)(stop_metrics_.all_off_verified_us-stop_metrics_.latched_us),(unsigned long long)stop_metrics_.last_target_tx_us,(unsigned)stop_metrics_.verify_rounds);
      motion_executor_.confirmSafeOff(true);
      operational_session_.confirmSafeOff();
      operational_facts_.operator_permit=false;
      operational_facts_.lease=AuthorityLease{};
      if(authority_.current()==ActuatorAuthority::MOTION)
        authority_.forceClear(AuthorityClearReason::OWNER_TEARDOWN);
      motion_safe_off_mask_=0;motion_safe_off_next_=0;
    }
  } else { motion_safe_off_mask_=0;motion_safe_off_next_=0;safe_off_burst_sent_=false; }
}
bool Controller::stopPreempt(void* self) {
  return static_cast<Controller*>(self)->http_transport_.stopPending();
}
void Controller::emergencyTorqueOffBurst() {
  if(safe_off_burst_sent_||!build::kServoPowerAvailable)return;
  if(!(motion_executor_.status().safe_off_required||operational_session_.safeOffRequired()))return;
  safe_off_burst_sent_=true;
  if(stop_metrics_.all_off_verified_us)stop_metrics_=StopMetrics{};
  if(!stop_metrics_.latched_us){stop_metrics_.latched_us=motion_clock_.extend(micros());}
  stop_metrics_.last_target_tx_us=motion_executor_.status().last_tx_complete_us;
  // ONE SYNC_WRITE TorqueEnable=0 to the twelve leg ids: no poll, no readback,
  // no target in between. Verification follows per id; nothing is claimed here.
  stop_metrics_.off_tx_start_us=motion_clock_.extend(micros());
  stop_metrics_.off_bytes=servo_bus_.syncTorqueOff(canonical_leg_ids_,12);
  stop_metrics_.off_tx_done_us=motion_clock_.extend(micros());
}

void Controller::update(uint32_t now_ms) {
  const uint32_t cycle_started_us=micros();
  motion_cycle_.gap_us=motion_cycle_.started_us?cycle_started_us-motion_cycle_.started_us:0;
  motion_cycle_.started_us=cycle_started_us;
  motion_clock_.extend(cycle_started_us);
  // Issue #52: an authenticated Dashboard STOP waits in its own one-slot lane
  // (never behind the ordinary mailbox); it is validated and applied here,
  // first, and the torque-OFF burst leaves before any diagnostic or poll.
  if(http_transport_.serviceStopLane(now_ms)){
    stop_metrics_=StopMetrics{};stop_metrics_.latched_us=motion_clock_.extend(micros());
    emergencyTorqueOffBurst();
  }
  // First, unconditionally: the freshest possible permit/bootstrap state
  // must exist before command_router_.update() can act on it this tick.
  updateCalibrationMotionPermit();

  wifi_.setContext(service_.networkCritical(),ota_.status().holds_actuator_inhibit);
  command_router_.update(now_ms);
  wifi_.setContext(service_.networkCritical(),ota_.status().holds_actuator_inhibit);
  // The radio role follows the operating mode (MAINTENANCE: administrative AP;
  // RUN: router first). One-way: the Wi-Fi owner reads this, it cannot set it.
  // Radio role follows the OPERATOR's @MODE choice only. Automatic RUN entry
  // (operational readiness) keeps the AP that serves the Android Dashboard.
  wifi_.setRunMode(operating_mode_.routerRole());

  const uint32_t imu_started_us=micros();
  if(!imu_.taskRunning())imu_.service(now_ms);
  motion_cycle_.imu_us=static_cast<uint32_t>(micros()-imu_started_us);
  system_state_.setImuHealth(imu_.health());

  const uint32_t bms_started_us=micros();
  daly_.update(now_ms, operating_mode_.mode());
  motion_cycle_.bms_us=static_cast<uint32_t>(micros()-bms_started_us);
  system_state_.setBmsHealth(daly_.health());

  led_.update(now_ms);
  system_state_.setLedHealth(led_.health());

  // Advances at most one servo Ping() per tick when a scan is RUNNING —
  // see ServoBus::update() / ScanState for why this must never be skipped.
  servo_bus_.update(now_ms);
  // Strictly after servo_bus_.update(): it observes that call's
  // RUNNING -> COMPLETE edge and classifies the raw scan exactly once.
  servo_census_.update();
  servo_preflight_.update();
  updateOperationalReadiness(millis());
  // CR2-B: advances at most one read-only acquisition action per tick.
  // It never acquires actuator authority and never writes the servo bus.
  updateQ0Capture();
  updateStartupQualification();
  // CR3 continuation: advances the first-motion attempt (if any) by at most
  // one backend call, and independently forces real SAFE_OFF retries while
  // one is required — see updateFirstMotion()'s own comment.
  updateFirstMotion(now_ms);
  // CR3 continuation: same bounded, at-most-one-backend-call-per-tick
  // discipline, for the Full Leg Calibration sequence (if any is active).
  updateFullLegCalibration(now_ms);
  // Same tick the executor turned terminal: record, complete the session,
  // revoke the permit. See updateFullLegFinalization().
  updateFullLegFinalization();
  motion_cycle_.before_motion_us=static_cast<uint32_t>(micros()-cycle_started_us);
  updateOperationalMotion(now_ms);

  // System-level servo health is a population verdict under ROBOT_POWERED,
  // not the outcome of whichever individual Ping/read happened last.
  //
  // During the automatic startup census the system stays BOOTING. Once a
  // full canonical census completes, only PASS can make the servo subsystem
  // OK; PROFILE_MISMATCH/RANGE_INCOMPLETE fail closed as FAULT.
  //
  // A later operator-triggered census may replace that population verdict
  // when it completes. While such a later census is RUNNING, retain the
  // previous completed population health so calibration qualification does
  // not transiently turn the whole system back into BOOTING.
  if (!build::kServoPowerAvailable) {
    // Preserve the proven USB_ONLY semantics.
    system_state_.setServoHealth(servo_bus_.health());
  } else if (servo_census_.state() == servo::ServoCensus::State::COMPLETE) {
    const servo::CensusResult& census = servo_census_.result();
    const bool population_pass =
        census.verdict == servo::CensusVerdict::PASS;

    system_state_.setServoHealth(
        population_pass ? ModuleHealth::OK : ModuleHealth::FAULT);

    if (startup_servo_census_pending_) {
      startup_servo_census_pending_ = false;
      const servo::ScanResult& raw = servo_bus_.lastScanResult();
      Serial.printf(
          "STARTUP_SERVO_CENSUS=%s verdict=%s expected=%u present=%u missing=%u "
          "absent_by_design=%u absent_present=%u unexpected=%u "
          "elapsed_ms=%lu max_ping_us=%lu\n",
          population_pass ? "PASS" : "FAIL",
          servo::toString(census.verdict),
          (unsigned)census.expected_now,
          (unsigned)census.present_expected,
          (unsigned)census.missing_expected,
          (unsigned)census.absent_by_design,
          (unsigned)census.absent_by_design_present,
          (unsigned)census.unexpected_id,
          (unsigned long)raw.elapsed_ms,
          (unsigned long)raw.max_ping_us);
    }
  } else if (startup_servo_census_pending_) {
    system_state_.setServoHealth(ModuleHealth::NOT_INITIALIZED);
  }

  system_state_.update();

  // Last among the services, on purpose. Within one pass every
  // timing-sensitive module (IMU, DALY, the incremental servo scan step)
  // has already advanced before any network work happens, so a heavy tick
  // here — the first WiFi.mode() call is the expensive one — cannot sit
  // between a bus transaction and its follow-up. The cost of this call is
  // measured, not assumed: @WIFI STATUS reports last_us/max_us.
  wifi_.update(now_ms);

  // Drives the first-boot rollback lifecycle. Bounded, touches no flash, and
  // does almost nothing once the lifecycle has settled. The facts it judges
  // are passed in rather than reached for, so the criteria stay testable off
  // the device.
  {
    update::OtaHostFacts facts;
    facts.controller_initialized = initialized_;
    facts.command_router_bound = command_router_.bound();
    facts.uptime_ms = system_state_.uptimeMillis(now_ms);
    ota_.update(now_ms, facts);
  }

  // Bounded: returns immediately unless a session is live, and touches no
  // hardware in any case. It exists so a session notices authority being
  // taken away from underneath it.
  calibration_.update(operating_mode_.mode());

  // Last: every input below was just refreshed this tick. LED presentation
  // is read-only over all of them — see status/LedStatusPolicy.h for why
  // none of this duplicates a hardware read.
  {
    status::LedStatusInputs led_inputs;
    led_inputs.system_health = system_state_.systemHealth();
    led_inputs.firmware_update_in_progress =
        authority_.inhibited() && authority_.inhibitReason() == InhibitReason::FIRMWARE_UPDATE;
    led_inputs.calibration_in_progress = calibration_.sessionLive();
    led_inputs.wifi_connecting = wifi_.status().state == network::WifiState::RADIO_STARTING ||
                                 wifi_.status().state == network::WifiState::CONNECTING;
    const power::DalySample& battery = daly_.sample();
    // DALY may have stamped a sample after this tick's now_ms was captured.
    const uint32_t led_now_ms = millis();
    led_inputs.sample_valid = battery.valid;
    led_inputs.daly_comm_ok = daly_.lastCommResult() == power::DalyCommResult::OK;
    led_inputs.telemetry_age_ms = led_now_ms - battery.sampled_at_ms;
    led_inputs.soc_percent = battery.soc_percent;
    led_inputs.sampled_at_ms = battery.sampled_at_ms;
    led_inputs.cell_min_mv = battery.cell_min_mv;
    led_inputs.pack_voltage_v = battery.pack_voltage_v;
    led_inputs.battery_charging = strcmp(battery.state_name, "CHARGING") == 0;
    // Presentation only: the one informational KEY-OFF charging bit (word 3,
    // 0x0010) does not show as CHARGING_FAULT; every other bit does. The raw
    // words are untouched - @BMS STATUS still prints all four.
    led_inputs.battery_alarm = status::dalyAlarmBlocksChargingPresentation(battery.alarms);
    // No reviewed charge-completion policy exists yet: the reserved
    // charge_complete_verified input stays false, including at 100% SOC.
    led_status_.update(led_now_ms, led_inputs);
  }

  // Read-only copy of the calibration persistence RAM snapshot for the
  // telemetry layer (Dashboard). No NVS access, no load, no write: only the
  // facts @CALIBRATION PERSIST STATUS already prints.
  {
    const calibration::PersistenceSnapshot& persisted = persistence_.snapshot();
    CalibrationPersistenceView view;
    view.published = true;
    view.available = persistence_.calibrationAvailable();
    view.record_intact = persisted.assessment.acknowledged_record_intact;
    view.acknowledged_generation = persisted.assessment.acknowledged_generation;
    view.verdict = calibration::toString(persisted.verdict);
    view.nvs = calibration::toString(persisted.nvs);
    service_.setCalibrationPersistenceView(view);
  }

  // Drains at most one pending HTTP request, if the Web server was ever
  // started (see @WEB SERVER START). A no-op, bounded check when it was
  // not — see network/HttpTransport.h for the cross-thread handoff this
  // advances.
  // Admission/status must use a clock AFTER this loop's completed servo reads.
  // The loop-entry clock makes the newest joint look future-dated.
  http_transport_.update(millis());

  if (power_state_.state() == PowerState::SHUTDOWN_REQUESTED ||
      power_state_.state() == PowerState::SHUTTING_DOWN ||
      power_state_.state() == PowerState::POWER_CUT_REQUESTED) {
    power_state_.update(now_ms);
  }
}

}  // namespace core
}  // namespace matdog
