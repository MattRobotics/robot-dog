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
  actuator_backend_.begin(&servo_bus_);
  actuator_policy_.begin(&authority_);
  actuator_policy_.bindGeometry(&geometry_profile_, &actuator::geometry_data::kProvenance);
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

    // The Full Leg Calibration sequencer reuses the SAME reviewed deadman
    // figures for all three of its own monitored moves (MIN approach/
    // backoff, MAX approach/backoff via the same ContactProbeEngine, and the
    // auxiliary park) rather than inventing per-phase numbers — none of them
    // has a documented LF V25 precedent of its own either, and the outer
    // motion_timeout_ms remains the real backstop regardless.
    calibration::FullLegCalibrationConfig full_leg_config{};
    full_leg_config.probe_approach_deadman = deadman;
    full_leg_config.probe_backoff_deadman = deadman;
    full_leg_config.aux_move_deadman = deadman;
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

  // Binds the census service to the ONE ServoBus. Deliberately does not
  // start a census: no bus traffic whatsoever happens at boot (handoff
  // "no startup torque / no startup motion"), and scripts/static_audit.py
  // fails the build if Controller::begin() ever starts one.
  servo_census_.begin(&servo_bus_);
  servo_preflight_.begin(&servo_bus_);

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
  // Mirrors full_leg_calibration_.auxiliaryParked() exactly: true for the
  // ONE tick window (the MAX-side probe) where the compiled <leg>_UPPER:MAX
  // plan requires the named auxiliary already parked — false before and
  // after, and always false for a leg whose MAX endpoint needs no auxiliary
  // (a NOT_NEEDED plan must be validated with nothing parked). The endpoint
  // named is the one the running request probes, read from the executor
  // itself. See SafeActuatorPolicy::evaluateEndpointPlan() for how these four
  // fields are consumed; unconditionally threading them through cannot widen
  // access for the same reason the tick budget above cannot.
  ctx.auxiliary_parked = full_leg_calibration_.auxiliaryParked();
  ctx.parked_leg = full_leg_calibration_.endpointLeg();
  ctx.parked_joint = full_leg_calibration_.endpointJoint();
  ctx.parked_side = calibration::ContactSide::MAX_SIDE;
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

// CR3 continuation. Same contract as updateFirstMotion(), generalized to two
// buses: full_leg_calibration_ never touches ServoBus itself (no reference
// to it exists — see FullLegCalibrationExecutor.h), so this is the one place
// its decisions turn into real bus transactions, including the two
// independent, ungated ServoBus::safeOff() calls its two SAFE_OFF-servicing
// phases wait on.
void Controller::updateFullLegCalibration(uint32_t now_ms) {
  if (!full_leg_calibration_.active()) return;

  // Read BEFORE this tick's update() call: whether the executor was already
  // waiting on one or both SAFE_OFF confirmations coming into this tick, and
  // therefore whether a cached VERIFIED_OFF from a PRIOR tick is still valid
  // evidence to hand it as this call's primary/auxiliary_safe_off_verified
  // input. A phase that was not yet waiting cannot have valid evidence, so
  // its cached result is cleared first - the same "never reuse a past
  // VERIFIED_OFF for a new requirement" rule updateFirstMotion() applies.
  const bool was_primary_pending = full_leg_calibration_.primarySafeOffPending();
  const bool was_auxiliary_pending = full_leg_calibration_.auxiliarySafeOffPending();
  if (!was_primary_pending) {
    full_leg_primary_safe_off_result_ = servo::SafeOffResult::UNVERIFIED_NO_RESPONSE;
  }
  if (!was_auxiliary_pending) {
    full_leg_auxiliary_safe_off_result_ = servo::SafeOffResult::UNVERIFIED_NO_RESPONSE;
  }
  const bool primary_verified =
      was_primary_pending && full_leg_primary_safe_off_result_ == servo::SafeOffResult::VERIFIED_OFF;
  const bool auxiliary_verified =
      was_auxiliary_pending &&
      full_leg_auxiliary_safe_off_result_ == servo::SafeOffResult::VERIFIED_OFF;

  calibration::FullLegCalibrationContext context{};
  context.session_active = calibration_.sessionLive();
  context.origin = calibration_.status().origin;
  context.lease = calibration_.authorityLease();
  context.mode = operating_mode_.mode();
  context.motion_permit_active = motion_permit_.active();
  context.authority = authority_.current();
  context.authority_generation = authority_.generation();
  context.authority_inhibited = authority_.inhibited();

  // Telemetry is read from whichever bus the CURRENT (pre-update) phase is
  // actually monitoring - the auxiliary while it is being parked, the
  // primary (probed) joint at every other point, including both
  // ContactProbeEngine phases and both SAFE_OFF-servicing waits (harmless
  // there: the executor ignores the sample outside a monitoring phase,
  // exactly like FirstMotionExecutor does).
  const bool aux_phase =
      full_leg_calibration_.status().phase == calibration::FullLegCalibrationPhase::AUX_MOVE_PENDING ||
      full_leg_calibration_.status().phase ==
          calibration::FullLegCalibrationPhase::AUX_MOVE_MONITORING;
  const uint8_t telemetry_bus_id = aux_phase ? full_leg_calibration_.auxiliaryBusId()
                                             : full_leg_calibration_.primaryBusId();
  servo::ServoBus::RuntimeState state{};
  const bool read_ok = servo_bus_.readRuntimeState(telemetry_bus_id, &state);
  actuator::TelemetrySample sample{};
  sample.read_ok = read_ok;
  sample.sampled_at_ms = now_ms;
  if (read_ok) {
    sample.present_position = state.present_position;
    sample.torque_enable = state.torque_enable;
  }

  // At most one state-machine advance / backend write per tick. If this call
  // reaches a SAFE_OFF-servicing phase (including transitioning into one
  // just now), do NOT return: the independent SAFE_OFF forcing below runs in
  // this SAME Controller tick, exactly like updateFirstMotion().
  full_leg_calibration_.update(context, now_ms, /*telemetry_available=*/true, sample,
                               primary_verified, auxiliary_verified);

  // SAFE_OFF is deliberately outside policy/session/authority/permit: forced
  // every tick either phase is pending (POST-update, so a transition into a
  // SAFE_OFF phase this very tick is still serviced this very tick), retried
  // until VERIFIED_OFF.
  if (full_leg_calibration_.primarySafeOffPending()) {
    full_leg_primary_safe_off_result_ =
        servo_bus_.safeOff(full_leg_calibration_.primaryBusId());
  }
  if (full_leg_calibration_.auxiliarySafeOffPending()) {
    full_leg_auxiliary_safe_off_result_ =
        servo_bus_.safeOff(full_leg_calibration_.auxiliaryBusId());
  }
}

// Closes the evidence lifecycle of the armed Full Leg run, exactly once, on
// the tick its executor turns terminal. Everything that decides the verdict
// lives in calibration::finalizeFullLeg() (pure, host-tested); this only
// hands it the live collaborators and keeps the record. The permit and
// authorization it revokes are the same RAM objects the per-tick refresh
// already re-checks, so cleanup between legs is finished before the next
// command line is processed.
void Controller::updateFullLegFinalization() {
  if (!full_leg_run_.armed) return;
  const calibration::FullLegCalibrationPhase phase = full_leg_calibration_.status().phase;
  if (phase != calibration::FullLegCalibrationPhase::COMPLETE &&
      phase != calibration::FullLegCalibrationPhase::FAILED) {
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
  Serial.println("startup_servo_scan : DISABLED");
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

  command_router_.update(now_ms);

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
  system_state_.setServoHealth(servo_bus_.health());

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
    led_inputs.battery_alarm = battery.alarms[0] != 0 || battery.alarms[1] != 0 ||
                               battery.alarms[2] != 0 || battery.alarms[3] != 0;
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
