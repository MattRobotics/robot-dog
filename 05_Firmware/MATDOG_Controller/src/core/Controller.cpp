#include "Controller.h"

#include <esp_ota_ops.h>
#include <esp_system.h>
#include <string.h>

#include "../config/BuildConfig.h"
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
  };
  service_.begin(modules);
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

void Controller::update(uint32_t now_ms) {
  // First, unconditionally: the freshest possible permit/bootstrap state
  // must exist before command_router_.update() can act on it this tick.
  updateCalibrationMotionPermit();

  wifi_.setContext(service_.networkCritical(),ota_.status().holds_actuator_inhibit);
  command_router_.update(now_ms);
  wifi_.setContext(service_.networkCritical(),ota_.status().holds_actuator_inhibit);

  imu_.update(now_ms);
  system_state_.setImuHealth(imu_.health());

  daly_.update(now_ms, operating_mode_.mode());
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
    led_inputs.battery_charging = strcmp(battery.state_name, "CHARGING") == 0;
    // Presentation only: the one informational KEY-OFF charging bit (word 3,
    // 0x0010) does not show as CHARGING_FAULT; every other bit does. The raw
    // words are untouched - @BMS STATUS still prints all four.
    led_inputs.battery_alarm = status::dalyAlarmBlocksChargingPresentation(battery.alarms);
    // No reviewed charge-completion policy exists yet: the reserved
    // charge_complete_verified input stays false, including at 100% SOC.
    led_status_.update(led_now_ms, led_inputs);
  }

  // Drains at most one pending HTTP request, if the Web server was ever
  // started (see @WEB SERVER START). A no-op, bounded check when it was
  // not — see network/HttpTransport.h for the cross-thread handoff this
  // advances.
  http_transport_.update(now_ms);

  if (power_state_.state() == PowerState::SHUTDOWN_REQUESTED ||
      power_state_.state() == PowerState::SHUTTING_DOWN ||
      power_state_.state() == PowerState::POWER_CUT_REQUESTED) {
    power_state_.update(now_ms);
  }
}

}  // namespace core
}  // namespace matdog
