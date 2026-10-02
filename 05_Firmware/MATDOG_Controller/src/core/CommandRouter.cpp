#include "CommandRouter.h"

#include <esp_ota_ops.h>

#include <initializer_list>
#include <cstring>

#include "../servo/ServoProfileData.h"
#include "../actuator/CalibrationGeometryProfileData.h"
#include "../actuator/CalibrationSequencePlanData.h"
#include "../actuator/CalibrationQ0EvidencePreparation.h"
#include "../calibration/CalibrationSessionOrchestrator.h"
#include "../calibration/CalibrationMotionPermit.h"
#include "../calibration/FirstMotionExecutor.h"
#include "../calibration/FullLegCalibrationExecutor.h"
#include "../calibration/FullLegCalibrationFinalizer.h"
#include "../calibration/FullLegCalibrationPlan.h"
#include "../actuator/OperationalEnvelope.h"

#include "../config/BuildConfig.h"
#include "../config/Pins.h"
#include "../network/HttpTransport.h"
#include "ControllerService.h"

namespace matdog {
namespace core {

namespace {
// Strict "<prefix><LF|RF|RH|LH><suffix>" match on the already-uppercased line.
// Exactly four leg tokens exist and nothing else is accepted: no partial
// match, no abbreviation, no numeric selector. Everything the command then
// does (bus ids, identities, endpoints, parking) is derived from the leg by
// calibration::resolveFullLegPlan(), never from the command text.
bool matchLegCommand(const String& line, const char* prefix, const char* suffix,
                     calibration::Leg* leg_out) {
  static const struct {
    const char* name;
    calibration::Leg leg;
  } kLegs[] = {
      {"LF", calibration::Leg::LF},
      {"RF", calibration::Leg::RF},
      {"RH", calibration::Leg::RH},
      {"LH", calibration::Leg::LH},
  };
  for (const auto& entry : kLegs) {
    String candidate(prefix);
    candidate += entry.name;
    candidate += suffix;
    if (line == candidate) {
      *leg_out = entry.leg;
      return true;
    }
  }
  return false;
}

// One captured line of exportFullLegEvidence(): the router prints the export
// one line at a time so a burst can never overrun the USB CDC ring (see
// CommandRouter::pumpFullLegEvidenceExport()).
struct ExportLineCapture {
  uint16_t want = 0;
  uint16_t seen = 0;
  bool got = false;
  // As large as the exporter's own line buffer (FullLegCalibrationFinalizer.cpp
  // kLineBytes = 512): an evidence line is never truncated here either.
  char line[512] = {0};
};

void captureExportLine(void* user, const char* line) {
  ExportLineCapture* capture = static_cast<ExportLineCapture*>(user);
  if (capture->seen == capture->want) {
    strncpy(capture->line, line, sizeof(capture->line) - 1);
    capture->line[sizeof(capture->line) - 1] = '\0';
    capture->got = true;
  }
  ++capture->seen;
}
}  // namespace

void CommandRouter::begin(const Modules& modules) {
  modules_ = modules;
  resetLine();
}

void CommandRouter::resetLine() {
  memset(line_buf_, 0, sizeof(line_buf_));
  line_len_ = 0;
  line_error_ = LineError::NONE;
}

bool CommandRouter::q0CaptureOwnsServoDiagnostics() const {
  return modules_.q0_capture != nullptr && modules_.q0_capture->active();
}

bool CommandRouter::motionExecutorBusy() const {
  return modules_.first_motion->active() || modules_.full_leg_calibration->active();
}

bool CommandRouter::servoDiagnosticBusy() const {
  if (q0CaptureOwnsServoDiagnostics() || motionExecutorBusy()) return true;
  return modules_.servo_bus->scanState() == servo::ScanState::RUNNING ||
         modules_.servo_census->state() == servo::ServoCensus::State::RUNNING ||
         modules_.servo_preflight->state() == servo::ServoPreflight::State::RUNNING;
}

void CommandRouter::update(uint32_t now_ms) {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') continue;

    if (c == '\n') {
      if (line_error_ != LineError::NONE) {
        Serial.println(line_error_ == LineError::OVERFLOW ? "ERROR=COMMAND_LINE_OVERFLOW"
                                                        : "ERROR=COMMAND_LINE_NUL");
      } else if (line_len_ > 0) {
        line_buf_[line_len_] = '\0';
        handleLine(String(line_buf_));
      }
      resetLine();
      continue;
    }

    if (line_error_ != LineError::NONE) continue;
    if (c == '\0') {
      line_error_ = LineError::NUL;
      continue;
    }
    if (line_len_ == kLineBufSize - 1) {
      line_error_ = LineError::OVERFLOW;
      continue;
    }
    line_buf_[line_len_++] = c;
  }

  if (bms_stream_enabled_ && (now_ms - last_bms_stream_ms_ >= 2000)) {
    last_bms_stream_ms_ = now_ms;
    printBmsStatus();
  }

  // The 0x81 read itself advances inside DalyBms::update(); this reports
  // its completion exactly once, without the command handler blocking.
  if (bms_key_read_result_pending_ &&
      modules_.daly->keyConfigReadResult() != power::DalyKeyReadResult::PENDING) {
    bms_key_read_result_pending_ = false;
    printBmsKeyReadResult();
  }

  // The KEY write reports in two stages: the acknowledgement as soon as it
  // is classified (so it reaches the host even if the load domain drops
  // right after), then the read-back verdict.
  if (bms_key_write_pending_) {
    const power::DalyKeyWriteStatus& w = modules_.daly->keyWriteStatus();
    if (!bms_key_write_ack_reported_ && w.ack != power::DalyKeyWriteAck::NONE &&
        w.ack != power::DalyKeyWriteAck::PENDING) {
      bms_key_write_ack_reported_ = true;
      Serial.printf("BMS_KEY_WRITE=ACK result=%s rx_bytes=%u\n",
                    power::toString(w.ack), (unsigned)w.ack_rx_bytes);
    }
    if (w.state != power::DalyKeyWriteState::PENDING) {
      bms_key_write_pending_ = false;
      printBmsKeyWriteResult();
    }
  }

  // The scan itself advances inside ServoBus::update() (called from
  // Controller::update() before this router runs); this just notices the
  // RUNNING -> COMPLETE transition and reports the result exactly once,
  // without the command handler having to block waiting for it.
  if (servo_scan_result_pending_ &&
      modules_.servo_bus->scanState() == servo::ScanState::COMPLETE) {
    servo_scan_result_pending_ = false;
    printServoScanResult();
  }

  // Same pattern for the census, but it waits on the SERVICE's state, not
  // the bus's: Controller::update() runs ServoCensus::update() after
  // ServoBus::update(), so COMPLETE here means the structured result has
  // already been classified and stored.
  if (servo_census_result_pending_ &&
      modules_.servo_census->state() == servo::ServoCensus::State::COMPLETE) {
    servo_census_result_pending_ = false;
    printServoCensusResult();
  }
  if (servo_preflight_result_pending_ &&
      modules_.servo_preflight->state() == servo::ServoPreflight::State::COMPLETE) {
    servo_preflight_result_pending_ = false;
    printServoPreflightResult();
  }

  // CR2-B completion/failure is reported once. All data are cached by the
  // acquisition coordinator; printing performs no servo transaction.
  if (q0_capture_result_pending_ && modules_.q0_capture->terminal()) {
    q0_capture_result_pending_ = false;
    printCalibrationQ0Status();
  }

  if (evidence_export_pending_) pumpFullLegEvidenceExport();
}

void CommandRouter::handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();
  // Set only by matchLegCommand() on a strict four-token match.
  calibration::Leg command_leg = calibration::Leg::LF;

  if (upper == "@HELP") {
    printHelp();
  } else if (upper == "@STATUS") {
    printStatus();
  } else if (upper == "@IMU STATUS") {
    printImuStatus();
  } else if (upper == "@IMU STREAM ON") {
    modules_.imu->setStreamEnabled(true);
    Serial.println("IMU_STREAM=ON");
  } else if (upper == "@IMU STREAM OFF") {
    modules_.imu->setStreamEnabled(false);
    Serial.println("IMU_STREAM=OFF");
  } else if (upper == "@BMS STATUS") {
    printBmsStatus();
  } else if (upper == "@BMS STREAM ON") {
    bms_stream_enabled_ = true;
    Serial.println("BMS_STREAM=ON");
  } else if (upper == "@BMS STREAM OFF") {
    bms_stream_enabled_ = false;
    Serial.println("BMS_STREAM=OFF");
  } else if (upper == "@BMS KEY READ") {
    // One read-only FC03 transaction on the RS485 bus the BMS also answers
    // telemetry on: a diagnostic, so MAINTENANCE only, like @SERVO SCAN.
    // Takes no arguments - there is deliberately no register/value input.
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("BMS_KEY_READ=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    if (modules_.daly->requestKeyConfigRead()) {
      bms_key_read_result_pending_ = true;
      Serial.println("BMS_KEY_READ=STARTED");
    } else {
      Serial.println("BMS_KEY_READ=BUSY");
      Serial.println("REASON=READ_ALREADY_PENDING");
    }
  } else if (upper == "@BMS KEY STATUS") {
    printBmsKeyStatus();
  } else if (upper == "@BMS KEY SET DISCHARGE CONFIRM") {
    // The ONE DALY write: KEY logic 0x0120 := 0x005A (DISCHARGE). Exact text
    // only - no alias, no argument, no value. DalyBms re-checks every
    // precondition and transmits nothing on refusal.
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("BMS_KEY_WRITE=REFUSED reason=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    const power::DalyKeyWriteGate gate =
        modules_.daly->requestKeyLogicDischarge(modules_.operating_mode->mode());
    if (gate.decision == power::DalyKeyWriteDecision::START) {
      bms_key_write_pending_ = true;
      bms_key_write_ack_reported_ = false;
      Serial.println("BMS_KEY_WRITE=STARTED target=DISCHARGE raw=0x005A");
    } else if (gate.decision == power::DalyKeyWriteDecision::ALREADY_CONFIGURED) {
      Serial.println("BMS_KEY_WRITE=ALREADY_CONFIGURED raw=0x005A tx_bytes=0");
    } else {
      Serial.printf("BMS_KEY_WRITE=REFUSED reason=%s tx_bytes=0\n",
                    power::toString(gate.refusal));
    }
  } else if (upper == "@BMS KEY WRITE STATUS") {
    printBmsKeyWriteStatus();
  } else if (upper == "@LED STATUS") {
    printLedStatus();
  } else if (upper == "@LED OFF") {
    if (!build::kLedRailPowered) {
      Serial.println("LED_OFF=NOOP");
      Serial.println("REASON=LED_RAIL_UNPOWERED (nothing is ever driven in this profile)");
    } else {
      modules_.led->off();
      Serial.println("LED=OFF");
    }
  } else if (upper == "@LED TEST") {
    if (modules_.led->startTest()) {
      Serial.println("LED_TEST=STARTED");
    } else {
      Serial.println("LED_TEST=BLOCKED");
      Serial.printf("REASON=%s\n", status::LedRing::blockedReason());
      Serial.printf("PROFILE=%s\n", build::kTestProfile);
    }
  } else if (upper == "@LED SOC TEST") {
    if (modules_.led->startSocTest()) {
      Serial.println("LED_SOC_TEST=STARTED");
    } else {
      Serial.println("LED_SOC_TEST=BLOCKED");
      Serial.printf("REASON=%s\n", status::LedRing::blockedReason());
      Serial.printf("PROFILE=%s\n", build::kTestProfile);
    }
  } else if (upper == "@WIFI STATUS") {
    printWifiStatus();
  } else if (upper == "@WIFI ON" || upper == "@WIFI OFF") {
    // Deliberately NOT gated by operating mode. The radio is orthogonal to
    // servo safety: it cannot block the bus (WifiManager::update() is
    // bounded and measured) and it has no path to an actuator. Gating it on
    // MAINTENANCE would only make the network unusable in the mode a future
    // motion loop actually runs in.
    const bool on = (upper == "@WIFI ON");
    if (modules_.wifi->setEnabled(on, millis())) {
      Serial.printf("WIFI=%s\n", on ? "ON" : "OFF");
    } else {
      Serial.println("WIFI=REFUSED");
      Serial.println("REASON=NO_CREDENTIALS");
      Serial.println("HINT=create src/config/WifiCredentials.local.h and rebuild");
    }
    printWifiStatus();
  } else if (isPersistCommand(upper)) {
    handlePersistCommand(upper);
  } else if (upper == "@CALIBRATION Q0 STATUS") {
    printCalibrationQ0Status();
  } else if (upper == "@CALIBRATION Q0 ABORT") {
    if (modules_.q0_capture->active()) {
      modules_.q0_capture->fail(calibration::Q0CaptureFailure::EXTERNAL_ABORT);
      q0_capture_result_pending_ = false;
      Serial.println("CALIBRATION_Q0_ABORT=OK");
    } else {
      Serial.println("CALIBRATION_Q0_ABORT=NO_ACTIVE_CAPTURE");
    }
    printCalibrationQ0Status();
  } else if (upper.startsWith("@CALIBRATION Q0 CAPTURE")) {
    // Read-only evidence acquisition is deliberately separate from a live
    // CalibrationManager motion session. No authority is requested here.
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("CALIBRATION_Q0=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      return;
    }
    if (!build::kServoPowerAvailable) {
      Serial.println("CALIBRATION_Q0=BLOCKED");
      Serial.println("REASON=SERVO_RAIL_UNPOWERED_IN_ACTIVE_PROFILE");
      Serial.printf("PROFILE=%s\n", build::kTestProfile);
      return;
    }
    if (modules_.q0_capture->active()) {
      Serial.println("CALIBRATION_Q0=BUSY");
      Serial.println("REASON=CAPTURE_ALREADY_ACTIVE");
      return;
    }
    if (servoDiagnosticBusy()) {
      Serial.println("CALIBRATION_Q0=BUSY");
      Serial.println("REASON=SERVO_DIAGNOSTIC_TRANSACTION_ACTIVE");
      return;
    }

    int samples = -1;
    int stability_spread = -1;
    char confirm[32] = {0};
    char extra[2] = {0};
    const int parsed = sscanf(upper.c_str(),
                              "@CALIBRATION Q0 CAPTURE %d %d %31s %1s",
                              &samples, &stability_spread, confirm, extra);
    if (parsed != 3 || strcmp(confirm, "CONFIRM_Q0_POSE") != 0 ||
        samples < actuator::kQ0BootstrapMinSamples ||
        samples > actuator::kQ0BootstrapMaxSamples ||
        stability_spread < 0 || stability_spread >= 2048) {
      Serial.println("CALIBRATION_Q0=REFUSED");
      Serial.println("REASON=USAGE_OR_CONFIRMATION");
      Serial.println("USAGE=@CALIBRATION Q0 CAPTURE <samples 3..32> "
                     "<stability_ticks 0..2047> CONFIRM_Q0_POSE");
      return;
    }

    calibration::Q0CaptureConfig config{};
    config.samples_per_joint = static_cast<uint8_t>(samples);
    config.stability_budget_specified = true;
    config.max_stability_spread_ticks = static_cast<uint16_t>(stability_spread);
    config.nominal_zero_pose_confirmed = true;
    config.started_at_ms = millis();

    if (!modules_.q0_capture->start(config)) {
      Serial.println("CALIBRATION_Q0=REFUSED");
      Serial.printf("REASON=%s\n",
                    calibration::toString(modules_.q0_capture->status().failure));
      return;
    }

    q0_capture_result_pending_ = true;
    Serial.printf("CALIBRATION_Q0=STARTED session=%lu samples_per_joint=%u "
                  "stability_ticks=%u\n",
                  (unsigned long)modules_.q0_capture->status().capture_session_id,
                  (unsigned)modules_.q0_capture->status().samples_per_joint,
                  (unsigned)config.max_stability_spread_ticks);
    Serial.println("CALIBRATION_Q0_NOTE read-only; torque must already be OFF; "
                   "no motion/authority/EEPROM write");
  } else if (upper == "@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION") {
    // CR3-M5: promotes the twelve q0 candidates of the CURRENT-BOOT read-only
    // capture (@CALIBRATION Q0 CAPTURE) through the exact CR3 acceptance and
    // promotion functions, then admits the resulting PROMOTED transforms into
    // the production transform table, replacing the previous entry of every
    // joint. The frozen CR2-C package is NOT consulted (it stays a regression
    // oracle only). No bus transaction, no authority, no EEPROM write - the
    // operator confirmation asserts the legs were at the nominal q0 pose for
    // this capture and nothing was reassembled since.
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("CALIBRATION_Q0_PROMOTE=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      return;
    }
    if (modules_.geometry_profile == nullptr || modules_.actuator_policy == nullptr) {
      Serial.println("CALIBRATION_Q0_PROMOTE=REFUSED");
      Serial.println("REASON=INFRASTRUCTURE_NOT_BOUND");
      return;
    }
    // Swapping q0 under a live session, a running executor or an armed run
    // would change the numbers a leg is being calibrated with.
    if (modules_.q0_capture->active()) {
      Serial.println("CALIBRATION_Q0_PROMOTE=BUSY");
      Serial.println("REASON=CAPTURE_STILL_ACTIVE");
      return;
    }
    if (motionExecutorBusy() || modules_.full_leg_run->armed ||
        modules_.calibration->sessionLive()) {
      Serial.println("CALIBRATION_Q0_PROMOTE=BUSY");
      Serial.println("REASON=CALIBRATION_SESSION_OR_MOTION_ACTIVE");
      return;
    }
    const actuator::FreshQ0Capture fresh_capture = modules_.q0_capture->freshCapture();
    const actuator::Q0EvidencePreparation prepared = actuator::prepareFreshQ0Evidence(
        *modules_.geometry_profile, actuator::geometry_data::kProvenance, fresh_capture,
        /*explicit_current_installation_confirmation=*/true);
    if (!prepared.ready()) {
      Serial.println("CALIBRATION_Q0_PROMOTE=REFUSED");
      Serial.printf("REASON=%s\n", actuator::toString(prepared.status));
      Serial.printf("FAILED_RECORD_INDEX=%u\n", (unsigned)prepared.failed_record_index);
      return;
    }
    uint8_t admitted = 0;
    for (uint8_t i = 0; i < prepared.transform_count; ++i) {
      if (modules_.actuator_policy->transforms().admit(prepared.transforms[i])) ++admitted;
    }
    const bool promotion_complete = modules_.q0_capture->notePromotionCompleted(
        fresh_capture.capture_session_id, admitted,
        modules_.actuator_policy->currentGeometryTag());
    Serial.printf("CALIBRATION_Q0_PROMOTE=%s admitted=%u/%u source=CURRENT_BOOT_CAPTURE "
                  "capture_session=%lu\n",
                 promotion_complete ? "OK" : "PARTIAL",
                 (unsigned)admitted, (unsigned)prepared.transform_count,
                 (unsigned long)fresh_capture.capture_session_id);
    Serial.println("CALIBRATION_Q0_PROMOTE_NOTE RAM-only; no EEPROM write; no motion; "
                   "no authority acquired; a promoted transform alone authorizes no write");
  } else if (matchLegCommand(upper, "@CALIBRATION SESSION START ", " CONFIRM_CURRENT_Q0",
                             &command_leg)) {
    // CR3 activation gate A, for any of the four legs: creates only a LIVE
    // CalibrationManager session. It performs no ServoBus transaction and
    // grants no motion permit. The current-boot q0 population evidence covers
    // all twelve joints, so the four legs run back to back on ONE capture and
    // ONE promotion - but never concurrently: a leg starts only once the
    // previous leg's run has been finalized and its session, permit and
    // authority have been released.
    const char* leg_name = calibration::toString(command_leg);
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      return;
    }
    if (modules_.system_state->systemHealth() != SystemHealth::READY) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.printf("REASON=SYSTEM_HEALTH_%s\n",
                    toString(modules_.system_state->systemHealth()));
      return;
    }
    if (modules_.q0_capture->status().state !=
        calibration::Q0CaptureState::COMPLETE) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=CURRENT_BOOT_Q0_CAPTURE_NOT_COMPLETE");
      return;
    }
    if (modules_.actuator_policy->currentGeometryTag() ==
            actuator::kNoGeometryProvenance ||
        modules_.actuator_policy->transforms().size() !=
            calibration::kLegServoSlotCount) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=GEOMETRY_OR_PROMOTED_TRANSFORMS_NOT_CURRENT");
      return;
    }
    if (!actuator::freshQ0CaptureIsPromoted(
            modules_.q0_capture->freshCapture(),
            modules_.actuator_policy->transforms(),
            modules_.actuator_policy->currentGeometryTag())) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=CURRENT_BOOT_Q0_NOT_PROMOTED hint=@CALIBRATION_Q0_PROMOTE");
      return;
    }
    if (motionExecutorBusy()) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=MOTION_EXECUTOR_ACTIVE");
      return;
    }
    if (modules_.full_leg_run->armed) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=PREVIOUS_LEG_RUN_NOT_FINALIZED");
      return;
    }
    if (modules_.calibration->sessionLive()) {
      Serial.printf("CALIBRATION_SESSION=REFUSED\nREASON=SESSION_ALREADY_LIVE leg=%s "
                    "hint=finish_its_FULL_LEG_or_SESSION_ABORT\n",
                    calibration::toString(modules_.calibration->status().leg));
      return;
    }
    if (modules_.motion_permit->active()) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=MOTION_PERMIT_STILL_ACTIVE_REVOKE_FIRST");
      return;
    }
    if (modules_.authority->current() != ActuatorAuthority::NONE) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.printf("REASON=AUTHORITY_NOT_NONE owner=%s\n",
                    toString(modules_.authority->current()));
      return;
    }
    // A stale operator authorization (its permit already lapsed) must not
    // survive into the next leg's session.
    modules_.motion_authorization->revoke();

    const calibration::SessionStartFromQ0Result result =
        calibration::startCalibrationSessionFromQ0Evidence(
            *modules_.calibration,
            modules_.q0_capture->status().state,
            modules_.q0_capture->populationResult(),
            command_leg,
            modules_.operating_mode->mode());

    if (result.status != calibration::SessionStartFromQ0Status::STARTED) {
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.printf("REASON=%s manager=%s\n",
                    calibration::toString(result.status),
                    calibration::toString(result.manager_result));
      return;
    }
    if (modules_.calibration->status().leg != command_leg) {
      // The session manager must have opened exactly the requested leg.
      modules_.calibration->abortSession();
      Serial.println("CALIBRATION_SESSION=REFUSED");
      Serial.println("REASON=SESSION_LEG_MISMATCH");
      return;
    }

    Serial.printf("CALIBRATION_SESSION=ACTIVE leg=%s session=%lu authority_generation=%lu\n",
                  leg_name,
                  (unsigned long)modules_.calibration->status().session_id,
                  (unsigned long)modules_.calibration->status().lease_generation);
    Serial.println("CALIBRATION_SESSION_NOTE motion_permit=NOT_GRANTED "
                   "hardware_motion_authorized=FALSE");

  } else if (upper ==
             "@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION") {
    // CR3 activation gate B: one exact, boot/session-local authorization.
    // The 16-tick value is deliberately fixed for the first hardware proof;
    // there is no generic budget parser in this command surface.
    if (modules_.motion_permit->active()) {
      Serial.println("CALIBRATION_MOTION_PERMIT=REFUSED");
      Serial.println("REASON=PERMIT_ALREADY_ACTIVE_REVOKE_FIRST");
      return;
    }

    modules_.motion_authorization->revoke();
    modules_.motion_authorization->operator_authorized = true;
    modules_.motion_authorization->direction_verify_tick_budget = 16;

    const calibration::CalibrationSessionStatus& session =
        modules_.calibration->status();

    calibration::CalibrationMotionPermitLiveInputs inputs{};
    inputs.operator_calibration_motion_authorized =
        modules_.motion_authorization->operator_authorized;
    inputs.robot_powered_profile =
        build::kHardwareProfile == config::HardwareProfile::ROBOT_POWERED;
    inputs.mode = modules_.operating_mode->mode();
    inputs.system_health = modules_.system_state->systemHealth();
    inputs.session_active = modules_.calibration->sessionLive();
    inputs.origin = session.origin;
    inputs.session_id = session.session_id;
    inputs.current_population_pass =
        calibration::populationIsCurrentPass(session.population);
    inputs.current_geometry_bound =
        modules_.actuator_policy->currentGeometryTag() !=
        actuator::kNoGeometryProvenance;
    inputs.promoted_transforms_complete =
        modules_.actuator_policy->transforms().size() ==
        calibration::kLegServoSlotCount;
    inputs.authority = modules_.authority->current();
    inputs.authority_generation = modules_.authority->generation();
    inputs.authority_inhibited = modules_.authority->inhibited();

    const calibration::CalibrationMotionPermitFacts facts =
        calibration::buildCalibrationMotionPermitFacts(inputs);

    calibration::CalibrationMotionPermitToken token{};
    const calibration::CalibrationPermitStatus permit =
        modules_.motion_permit->grant(facts, &token);

    if (permit != calibration::CalibrationPermitStatus::ACTIVE ||
        !token.valid()) {
      modules_.motion_authorization->revoke();
      Serial.println("CALIBRATION_MOTION_PERMIT=REFUSED");
      Serial.printf("REASON=%s\n", calibration::toString(permit));
      return;
    }

    modules_.motion_authorization->token = token;

    Serial.printf(
        "CALIBRATION_MOTION_PERMIT=ACTIVE generation=%lu session=%lu "
        "authority_generation=%lu direction_verify_budget_ticks=16\n",
        (unsigned long)token.permit_generation,
        (unsigned long)token.session_id,
        (unsigned long)token.authority_generation);
    Serial.println("CALIBRATION_MOTION_PERMIT_NOTE RAM_ONLY "
                   "global_hardware_motion_authorized=FALSE");

  } else if (upper == "@CALIBRATION MOTION PERMIT REVOKE") {
    // Safety de-escalation: always allowed. If a first-motion attempt is
    // underway, the next Controller tick observes the lost permit and routes
    // it to SAFE_OFF_REQUIRED before any further progress.
    modules_.motion_permit->revoke(
        calibration::CalibrationPermitRevokeReason::EXPLICIT);
    modules_.motion_authorization->revoke();

    Serial.println("CALIBRATION_MOTION_PERMIT=REVOKED");
    Serial.println("CALIBRATION_MOTION_NOTE active_motion_if_any_will_SAFE_OFF");

  } else if (upper == "@CALIBRATION MOTION ABORT") {
    // Deliberately implemented by withdrawing the prerequisite rather than by
    // calling FirstMotionExecutor/FullLegCalibrationExecutor's own abort()
    // directly (CommandRouter/Controller may only read status from the Safe
    // Actuator/Calibration Execution infrastructure - see
    // check_actuator_infrastructure_wired_fail_closed()). This still takes
    // effect the SAME Controller tick, not one tick later: both executors'
    // context.motion_permit_active is read from motion_permit_.active()
    // fresh in updateFirstMotion()/updateFullLegCalibration(), which run
    // AFTER command_router_.update() in Controller::update()'s call order -
    // see the CR3 development log. If torque was ever verified on, that
    // path requires SAFE_OFF.
    modules_.motion_permit->revoke(
        calibration::CalibrationPermitRevokeReason::EXPLICIT);
    modules_.motion_authorization->revoke();

    Serial.println("CALIBRATION_MOTION_ABORT=REQUESTED");
    Serial.println("CALIBRATION_MOTION_ABORT_NOTE permit_revoked=YES");

  } else if (upper ==
             "@CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER +16 CONFIRM_FIRST_MOTION") {
    // CR3 activation gate C: exactly ONE first-motion primitive is exposed.
    // No raw servo ID, arbitrary target or arbitrary delta is accepted.
    if (modules_.calibration->status().state !=
        calibration::SessionState::ACTIVE) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=NO_ACTIVE_CALIBRATION_SESSION");
      return;
    }
    // The one diagnostic move is pinned to LF_UPPER; it may only run inside
    // the LF session, never inside another leg's.
    if (modules_.calibration->status().leg != calibration::Leg::LF) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=ACTIVE_SESSION_IS_NOT_LF");
      return;
    }
    if (!modules_.motion_permit->active() ||
        !modules_.motion_authorization->operator_authorized ||
        !modules_.motion_authorization->token.valid() ||
        modules_.motion_authorization->direction_verify_tick_budget != 16) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=NO_CURRENT_EXACT_MOTION_PERMIT");
      return;
    }
    if (modules_.first_motion->active() || modules_.full_leg_calibration->active()) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=MOTION_EXECUTOR_ALREADY_ACTIVE");
      return;
    }

    // Resolve the current semantic identity from the canonical allocation,
    // then cross-check it against Geometry V5. Bus ID alone is never identity.
    constexpr uint8_t kFirstMotionBusId = 12;
    const servo::CanonicalServo* canonical =
        servo::findCanonical(kFirstMotionBusId);

    calibration::JointIdentity identity{};
    if (canonical == nullptr ||
        !calibration::semanticIdentityFromCanonical(*canonical, &identity) ||
        identity.leg != calibration::Leg::LF ||
        identity.joint != calibration::JointKind::UPPER) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=CANONICAL_IDENTITY_MISMATCH");
      return;
    }

    const actuator::GeometryJointRecord* geometry_joint =
        modules_.geometry_profile->findJoint(identity);
    if (geometry_joint == nullptr ||
        geometry_joint->bus_id != kFirstMotionBusId ||
        !modules_.geometry_profile->withinDirectionVerifyEnvelope(identity, 16)) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=GEOMETRY_V5_ENVELOPE_OR_IDENTITY_MISMATCH");
      return;
    }

    calibration::FirstMotionRequest request{};
    request.joint = identity;
    request.bus_id = kFirstMotionBusId;
    request.delta_ticks = 16;

    calibration::FirstMotionContext context{};
    context.session_active = modules_.calibration->sessionLive();
    context.origin = modules_.calibration->status().origin;
    context.lease = modules_.calibration->authorityLease();
    context.mode = modules_.operating_mode->mode();
    context.motion_permit_active = modules_.motion_permit->active();
    context.authority = modules_.authority->current();
    context.authority_generation = modules_.authority->generation();
    context.authority_inhibited = modules_.authority->inhibited();

    if (!modules_.first_motion->start(request, context, millis())) {
      Serial.println("CALIBRATION_FIRST_MOTION=REFUSED");
      Serial.println("REASON=FIRST_MOTION_EXECUTOR_START_REFUSED");
      return;
    }

    Serial.println(
        "CALIBRATION_FIRST_MOTION=ARMED joint=LF_UPPER bus=12 delta_ticks=+16");
    Serial.println(
        "CALIBRATION_FIRST_MOTION_NOTE no_write_in_command_handler; "
        "next_Controller_tick_revalidates_all_dynamic_prerequisites");

  } else if (upper == "@CALIBRATION SESSION ABORT") {
    // Complete de-escalation. SAFE_OFF itself remains outside the manager and
    // outside authority; permit revocation alone is same-tick effective for
    // either executor - see @CALIBRATION MOTION ABORT above. abortSession()
    // additionally drops CALIBRATION authority itself, a second independent
    // continuation_ok clause either executor also re-checks every tick.
    modules_.motion_permit->revoke(
        calibration::CalibrationPermitRevokeReason::EXPLICIT);
    modules_.motion_authorization->revoke();
    modules_.calibration->abortSession();

    Serial.println("CALIBRATION_SESSION_ABORT=OK");
    Serial.println("CALIBRATION_SESSION_ABORT_NOTE permit=REVOKED authority=RELEASED");

  } else if (matchLegCommand(upper, "@CALIBRATION INITIAL RECOVERY ", " CONFIRM_Q0_RECOVERY",
                             &command_leg)) {
    // The controller-verified q0 baseline required after a fresh q0 promotion
    // and before any leg is calibrated: EVERY leg joint of the robot actively
    // commanded to its promoted q0, one at a time (prime at present, RAM
    // TorqueLimit, TorqueEnable, move, V25 settle gate, SAFE_OFF), then all
    // twelve verified at q0 torque-off. The SAME executor, policy phase table
    // and SAFE_OFF path as the INITIAL_RECOVERY phase of every Full Leg run;
    // no probe, no contact, no evidence record. Same live-session and
    // fresh-permit gates as @CALIBRATION FULL LEG; the session and permit
    // stay live for the leg run that follows.
    const char* leg_name = calibration::toString(command_leg);
    if (modules_.calibration->status().state != calibration::SessionState::ACTIVE ||
        modules_.calibration->status().leg != command_leg) {
      Serial.println("CALIBRATION_INITIAL_RECOVERY=REFUSED");
      Serial.printf("REASON=NO_ACTIVE_%s_CALIBRATION_SESSION\n", leg_name);
      return;
    }
    if (!modules_.motion_permit->active() ||
        !modules_.motion_authorization->operator_authorized ||
        !modules_.motion_authorization->token.valid()) {
      Serial.println("CALIBRATION_INITIAL_RECOVERY=REFUSED");
      Serial.println("REASON=NO_CURRENT_MOTION_PERMIT");
      return;
    }
    if (modules_.first_motion->active() || modules_.full_leg_calibration->active()) {
      Serial.println("CALIBRATION_INITIAL_RECOVERY=REFUSED");
      Serial.println("REASON=MOTION_EXECUTOR_ALREADY_ACTIVE");
      return;
    }
    if (modules_.full_leg_run->armed) {
      Serial.println("CALIBRATION_INITIAL_RECOVERY=REFUSED");
      Serial.println("REASON=PREVIOUS_LEG_RUN_NOT_FINALIZED");
      return;
    }
    calibration::FullLegPlan plan{};
    const calibration::FullLegPlanStatus plan_status = calibration::resolveFullLegPlan(
        *modules_.geometry_profile, actuator::geometry_data::kProvenance,
        modules_.actuator_policy->transforms(), &actuator::sequence_plan_data::kPlan, command_leg,
        &plan);
    if (plan_status != calibration::FullLegPlanStatus::OK) {
      Serial.println("CALIBRATION_INITIAL_RECOVERY=REFUSED");
      Serial.printf("REASON=FULL_LEG_PLAN_%s\n", calibration::toString(plan_status));
      return;
    }
    plan.request.recovery_only = true;

    calibration::FullLegCalibrationContext context{};
    context.session_active = modules_.calibration->sessionLive();
    context.origin = modules_.calibration->status().origin;
    context.lease = modules_.calibration->authorityLease();
    context.mode = modules_.operating_mode->mode();
    context.motion_permit_active = modules_.motion_permit->active();
    context.authority = modules_.authority->current();
    context.authority_generation = modules_.authority->generation();
    context.authority_inhibited = modules_.authority->inhibited();
    if (!modules_.full_leg_calibration->start(plan.request, context, millis())) {
      Serial.println("CALIBRATION_INITIAL_RECOVERY=REFUSED");
      Serial.println("REASON=FULL_LEG_EXECUTOR_START_REFUSED");
      return;
    }
    // Deliberately NOT armed as a leg run: nothing is finalized, nothing is
    // recorded in the evidence store, the session is not completed.
    Serial.printf("CALIBRATION_INITIAL_RECOVERY=ARMED session_leg=%s joints=%u torque_limit=%u "
                  "phase=PREFLIGHT\n",
                  leg_name, (unsigned)plan.request.population_count,
                  (unsigned)plan.request.torque_limit);
    for (uint8_t i = 0; i < plan.request.population_count; ++i) {
      const calibration::FullLegJoint& j = plan.request.population[i];
      Serial.printf("CALIBRATION_INITIAL_RECOVERY_TARGET bus=%u leg=%s joint=%s unit=%s q0=%u\n",
                    (unsigned)j.bus_id, calibration::toString(j.identity.leg),
                    calibration::toString(j.identity.joint), j.identity.physical_unit,
                    (unsigned)j.q0_tick);
    }
    Serial.println("CALIBRATION_INITIAL_RECOVERY_NOTE no_write_in_command_handler; "
                   "poll with @CALIBRATION FULL LEG STATUS; "
                   "terminal line CALIBRATION_INITIAL_RECOVERY_RESULT");

  } else if (matchLegCommand(upper, "@CALIBRATION FULL LEG ", " CONFIRM_FULL_CALIBRATION",
                             &command_leg)) {
    // The 24-contact Full Calibration of the leg the LIVE SESSION was opened
    // for: all SIX contacts (UPPER, LOWER, HIP x MIN/MAX) in the LF V25
    // state-machine order, with initial q0 recovery, held prerequisites and
    // the reviewed return - see FullLegCalibrationExecutor.h. Exactly the same live-session and
    // fresh-permit gates as @CALIBRATION MOTION DIRECTION_VERIFY, reused
    // rather than re-derived; mutually exclusive with it (both would
    // otherwise contend for the same bus).
    const char* leg_name = calibration::toString(command_leg);
    if (modules_.calibration->status().state != calibration::SessionState::ACTIVE ||
        modules_.calibration->status().leg != command_leg) {
      Serial.println("CALIBRATION_FULL_LEG=REFUSED");
      Serial.printf("REASON=NO_ACTIVE_%s_CALIBRATION_SESSION\n", leg_name);
      return;
    }
    if (!modules_.motion_permit->active() ||
        !modules_.motion_authorization->operator_authorized ||
        !modules_.motion_authorization->token.valid()) {
      Serial.println("CALIBRATION_FULL_LEG=REFUSED");
      Serial.println("REASON=NO_CURRENT_MOTION_PERMIT");
      return;
    }
    if (modules_.first_motion->active() || modules_.full_leg_calibration->active()) {
      Serial.println("CALIBRATION_FULL_LEG=REFUSED");
      Serial.println("REASON=MOTION_EXECUTOR_ALREADY_ACTIVE");
      return;
    }
    if (modules_.full_leg_run->armed) {
      Serial.println("CALIBRATION_FULL_LEG=REFUSED");
      Serial.println("REASON=PREVIOUS_LEG_RUN_NOT_FINALIZED");
      return;
    }

    // Everything the run needs comes from ONE resolver: canonical allocation
    // -> JointIdentity + bus id for all 12 leg joints (cross-checked against
    // Geometry V5), the six search corridors for this installation's q0, and
    // the geometry-validated sequence plan's prerequisite poses and park
    // joint. No leg, bus id, contact, corridor or pose number is typed in
    // this handler.
    calibration::FullLegPlan plan{};
    const calibration::FullLegPlanStatus plan_status = calibration::resolveFullLegPlan(
        *modules_.geometry_profile, actuator::geometry_data::kProvenance,
        modules_.actuator_policy->transforms(), &actuator::sequence_plan_data::kPlan, command_leg,
        &plan);
    if (plan_status != calibration::FullLegPlanStatus::OK) {
      Serial.println("CALIBRATION_FULL_LEG=REFUSED");
      Serial.printf("REASON=FULL_LEG_PLAN_%s\n", calibration::toString(plan_status));
      return;
    }

    calibration::FullLegCalibrationContext context{};
    context.session_active = modules_.calibration->sessionLive();
    context.origin = modules_.calibration->status().origin;
    context.lease = modules_.calibration->authorityLease();
    context.mode = modules_.operating_mode->mode();
    context.motion_permit_active = modules_.motion_permit->active();
    context.authority = modules_.authority->current();
    context.authority_generation = modules_.authority->generation();
    context.authority_inhibited = modules_.authority->inhibited();

    if (!modules_.full_leg_calibration->start(plan.request, context, millis())) {
      Serial.println("CALIBRATION_FULL_LEG=REFUSED");
      Serial.println("REASON=FULL_LEG_EXECUTOR_START_REFUSED");
      return;
    }
    // Armed only after the executor accepted the run; Controller finalizes it
    // when the executor turns terminal, whatever the reason.
    modules_.full_leg_run->arm(plan, modules_.actuator_policy->currentGeometryTag(),
                               modules_.calibration->status().session_id);

    const calibration::FullLegCalibrationRequest& r = plan.request;
    if (r.has_rear_park) {
      Serial.printf("CALIBRATION_FULL_LEG=ARMED leg=%s contacts_expected=%u hip_bus=%u upper_bus=%u "
                    "lower_bus=%u park=%s_%s park_bus=%u phase=PREFLIGHT\n",
                    leg_name, (unsigned)calibration::kFullLegContactCount,
                    (unsigned)plan.hip.bus_id, (unsigned)plan.upper.bus_id,
                    (unsigned)plan.lower.bus_id, calibration::toString(r.park.identity.leg),
                    calibration::toString(r.park.identity.joint), (unsigned)r.park.bus_id);
    } else {
      Serial.printf("CALIBRATION_FULL_LEG=ARMED leg=%s contacts_expected=%u hip_bus=%u upper_bus=%u "
                    "lower_bus=%u park=NONE park_bus=0 phase=PREFLIGHT\n",
                    leg_name, (unsigned)calibration::kFullLegContactCount,
                    (unsigned)plan.hip.bus_id, (unsigned)plan.upper.bus_id,
                    (unsigned)plan.lower.bus_id);
    }
    // The run's own geometry evidence, printed before anything moves: the six
    // calibration search corridors for this q0 and every prerequisite pose.
    const calibration::JointKind order[] = {calibration::JointKind::UPPER,
                                            calibration::JointKind::LOWER,
                                            calibration::JointKind::HIP};
    for (const calibration::JointKind kind : order) {
      for (uint8_t side = 0; side < calibration::kContactSideCount; ++side) {
        const actuator::CalibrationSearchCorridor& c =
            r.corridor[static_cast<uint8_t>(kind)][side];
        Serial.printf("CALIBRATION_FULL_LEG_SEARCH_CORRIDOR joint=%s side=%s probe_sign=%d q0=%u "
                      "contact=%u urdf_limit=%u entry=%u guard=%u opposite_limit=%u "
                      "guard_beyond_contact=%ld\n",
                      calibration::toString(kind), side == 0 ? "MIN" : "MAX", (int)c.probe_sign,
                      (unsigned)c.home_tick, (unsigned)c.contact_tick, (unsigned)c.urdf_limit_tick,
                      (unsigned)c.entry_tick, (unsigned)c.guard_tick,
                      (unsigned)c.opposite_limit_tick,
                      (long)(actuator::searchDepth(c, c.guard_tick) -
                             actuator::searchDepth(c, c.contact_tick)));
      }
    }
    Serial.printf("CALIBRATION_FULL_LEG_PREREQUISITE pose=UPPER_FOR_LOWER urad=%ld tick=%u\n",
                  (long)r.upper_for_lower_urad, (unsigned)r.upper_for_lower_tick);
    Serial.printf("CALIBRATION_FULL_LEG_PREREQUISITE pose=UPPER_FOR_HIP_MIN urad=%ld tick=%u\n",
                  (long)r.upper_for_hip_min_urad, (unsigned)r.upper_for_hip_min_tick);
    Serial.printf("CALIBRATION_FULL_LEG_PREREQUISITE pose=UPPER_FOR_HIP_MAX urad=%ld tick=%u\n",
                  (long)r.upper_for_hip_max_urad, (unsigned)r.upper_for_hip_max_tick);
    Serial.printf("CALIBRATION_FULL_LEG_PREREQUISITE pose=LOWER_FOLDED urad=%ld tick=%u\n",
                  (long)r.lower_folded_urad, (unsigned)r.lower_folded_tick);
    if (r.has_rear_park) {
      Serial.printf("CALIBRATION_FULL_LEG_PREREQUISITE pose=REAR_PARK urad=%ld tick=%u\n",
                    (long)r.park_target_urad, (unsigned)r.park_target_tick);
    }
    Serial.println("CALIBRATION_FULL_LEG_NOTE no_write_in_command_handler; "
                   "next_Controller_tick_revalidates_all_dynamic_prerequisites; "
                   "poll with @CALIBRATION FULL LEG STATUS");

  } else if (upper == "@CALIBRATION FULL LEG STATUS") {
    printFullLegCalibrationStatus();
  } else if (upper == "@CALIBRATION FULL LEG ABORT") {
    // Always allowed, exactly like @CALIBRATION MOTION ABORT - a safety
    // de-escalation is not gated on the same preconditions that started it.
    if (modules_.full_leg_calibration->active()) {
      modules_.full_leg_calibration->abort();
      Serial.println("CALIBRATION_FULL_LEG_ABORT=OK");
    } else {
      Serial.println("CALIBRATION_FULL_LEG_ABORT=NO_ACTIVE_SEQUENCE");
    }
    printFullLegCalibrationStatus();

  } else if (upper == "@CALIBRATION EVIDENCE EXPORT") {
    // Read-only. Refused while a run is in flight so the export can never mix
    // two states of the RAM store.
    if (modules_.full_leg_run->armed || modules_.full_leg_calibration->active()) {
      Serial.println("CALIBRATION_EVIDENCE_EXPORT=REFUSED");
      Serial.println("REASON=FULL_LEG_RUN_IN_PROGRESS");
      return;
    }
    printFullLegEvidenceExport();

  } else if (upper == "@CALIBRATION STATUS") {
    printCalibrationStatus();
  } else if (upper == "@ACTUATOR STATUS") {
    printActuatorStatus();
  } else if (upper == "@AUTHORITY STATUS") {
    printAuthorityStatus();
  } else if (upper == "@OTA STATUS") {
    printOtaStatus();
  } else if (upper == "@WEB SERVER STATUS") {
    printWebStatus();
  } else if (upper == "@WEB SERVER START" || upper == "@WEB SERVER STOP") {
    // Same "physical/USB access is the trust boundary" gate as the DALY KEY
    // write and the servo scan/census/preflight commands: starting the
    // listening socket is a MAINTENANCE-only action, even though the socket
    // itself never grants actuator authority (see HttpTransport.h).
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("WEB_SERVER=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    if (upper == "@WEB SERVER START") {
      if (modules_.http_transport->start()) {
        Serial.println("WEB_SERVER=STARTED");
      } else {
        Serial.println("WEB_SERVER=START_FAILED");
        Serial.println("REASON=ALREADY_STARTED_OR_HTTPD_START_FAILED");
      }
    } else {
      modules_.http_transport->stop();
      Serial.println("WEB_SERVER=STOPPED");
    }
    printWebStatus();
  } else if (upper.startsWith("@SERVO SCAN")) {
    if (q0CaptureOwnsServoDiagnostics() || motionExecutorBusy()) {
      Serial.println("SERVO_SCAN=BLOCKED");
      Serial.println(motionExecutorBusy() ? "REASON=MOTION_EXECUTOR_ACTIVE"
                                          : "REASON=CALIBRATION_Q0_CAPTURE_ACTIVE");
      return;
    }
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("SERVO_SCAN=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    int lo = -1, hi = -1;
    if (sscanf(upper.c_str(), "@SERVO SCAN %d %d", &lo, &hi) == 2) {
      if (modules_.servo_bus->startScan(lo, hi)) {
        servo_scan_result_pending_ = true;
        Serial.printf("SERVO_SCAN=STARTED lo=%d hi=%d\n", lo, hi);
      } else {
        Serial.println("ERROR=SCAN_ALREADY_RUNNING");
      }
    } else {
      Serial.println("ERROR=USAGE @SERVO SCAN <lo> <hi>");
    }
  } else if (upper == "@SERVO CENSUS") {
    if (q0CaptureOwnsServoDiagnostics() || motionExecutorBusy()) {
      Serial.println("SERVO_CENSUS=BLOCKED");
      Serial.println(motionExecutorBusy() ? "REASON=MOTION_EXECUTOR_ACTIVE"
                                          : "REASON=CALIBRATION_Q0_CAPTURE_ACTIVE");
      return;
    }
    // Same bounded per-ID blocking as @SERVO SCAN (it drives the same
    // ServoBus scan), so it carries the same MAINTENANCE-mode gate.
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("SERVO_CENSUS=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    if (modules_.servo_census->start()) {
      servo_census_result_pending_ = true;
      Serial.printf("SERVO_CENSUS=STARTED lo=%d hi=%d\n",
                    servo::kCanonicalScanLo, servo::kCanonicalScanHi);
    } else {
      Serial.println("ERROR=SCAN_ALREADY_RUNNING");
    }
  } else if (upper == "@SERVO PREFLIGHT") {
    if (q0CaptureOwnsServoDiagnostics() || motionExecutorBusy()) {
      Serial.println("SERVO_PREFLIGHT=BLOCKED");
      Serial.println(motionExecutorBusy() ? "REASON=MOTION_EXECUTOR_ACTIVE"
                                          : "REASON=CALIBRATION_Q0_CAPTURE_ACTIVE");
      return;
    }
    // H0 leg preflight. Strictly read-only: Ping plus register reads, no
    // torque, no target, no EEPROM write. It carries the same bounded
    // per-tick blocking as the census (one joint per update()), so it takes
    // the same MAINTENANCE gate.
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("SERVO_PREFLIGHT=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    if (modules_.servo_preflight->start()) {
      servo_preflight_result_pending_ = true;
      Serial.printf("SERVO_PREFLIGHT=STARTED joints=%u profile=%s\n",
                    (unsigned)servo::kLegPreflightCount, servo::profile_data::kProfileId);
    } else {
      Serial.println("ERROR=PREFLIGHT_ALREADY_RUNNING");
    }
  } else if (upper.startsWith("@SERVO READ")) {
    if (q0CaptureOwnsServoDiagnostics() || motionExecutorBusy()) {
      Serial.println("SERVO_READ=BLOCKED");
      Serial.println(motionExecutorBusy() ? "REASON=MOTION_EXECUTOR_ACTIVE"
                                          : "REASON=CALIBRATION_Q0_CAPTURE_ACTIVE");
      return;
    }
    if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {
      Serial.println("SERVO_READ=BLOCKED");
      Serial.println("REASON=NOT_IN_MAINTENANCE_MODE");
      Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
      return;
    }
    int id = -1;
    if (sscanf(upper.c_str(), "@SERVO READ %d", &id) == 1) {
      printServoRead(id);
    } else {
      Serial.println("ERROR=USAGE @SERVO READ <id>");
    }
  } else if (upper.startsWith("@SERVO SAFE_OFF")) {
    // Deliberately NOT gated by operating mode: this is the one write that
    // can only make things safer (torque off), so it must stay reachable
    // regardless of MAINTENANCE/RUN — see OperatingMode.h.
    int id = -1;
    if (sscanf(upper.c_str(), "@SERVO SAFE_OFF %d", &id) == 1) {
      printServoSafeOff(id);
    } else {
      Serial.println("ERROR=USAGE @SERVO SAFE_OFF <id>");
    }
  } else if (upper == "@MODE STATUS") {
    printModeStatus();
  } else if (upper == "@MODE MAINTENANCE" || upper == "@MODE RUN") {
    const OperatingMode next =
        (upper == "@MODE RUN") ? OperatingMode::RUN : OperatingMode::MAINTENANCE;
    modules_.operating_mode->setMode(next);
    // A mode change that leaves the current owner incompatible clears it
    // rather than leaving a suspended authority behind. Today nothing can
    // ever hold one, so this is a no-op - it exists so the first real owner
    // does not have to remember to add it.
    modules_.authority->onOperatingModeChanged(next);
    printModeStatus();
    printAuthorityStatus();
  } else if (upper == "@SYSTEM SOURCE_SIGNATURE") {
    printSourceSignature();
  } else if (upper == "@HOSTLINK READINESS") {
    printHostLinkReadiness();
  } else if (upper == "@SYSTEM SHUTDOWN") {
    modules_.power_state->requestShutdown();
    Serial.println("SHUTDOWN_REQUESTED=YES");
    Serial.println("NOTE=DALY discharge-MOS write protocol not yet verified;");
    Serial.println("NOTE=this will resolve to POWER_CUT_FAILED, not OFF.");
  } else if (line.startsWith("@")) {
    Serial.print("UNKNOWN_COMMAND=");
    Serial.println(line);
    Serial.println("Type @HELP for the command list.");
  }
  // Silently ignore any line that doesn't start with '@' — keeps the
  // diagnostic router from reacting to stray terminal noise.
}

void CommandRouter::printHelp() {
  Serial.println("MATDOG Controller command surface:");
  Serial.println("  @HELP");
  Serial.println("  @STATUS");
  Serial.println("  @IMU STATUS");
  Serial.println("  @IMU STREAM ON|OFF");
  Serial.println("  @BMS STATUS");
  Serial.println("  @BMS STREAM ON|OFF");
  Serial.println("  @BMS KEY READ           (MAINTENANCE mode only; read-only, async result)");
  Serial.println("  @BMS KEY STATUS         (cached KEY snapshot; no bus transaction)");
  Serial.println("  @BMS KEY SET DISCHARGE CONFIRM  (MAINTENANCE only; the one DALY write,");
  Serial.println("                           0x0120 := 0x005A, then read-back; once per boot)");
  Serial.println("  @BMS KEY WRITE STATUS   (cached write result; no bus transaction)");
  Serial.println("  @LED STATUS");
  Serial.println("  @LED OFF");
  Serial.println("  @LED TEST");
  Serial.println("  @LED SOC TEST");
  Serial.println("  @WIFI STATUS           (cached snapshot; no radio query)");
  Serial.println("  @WIFI ON|OFF           (any mode; refused without credentials)");
  Serial.println("  @OTA STATUS            (read-only; OTA-A ships no transport)");
  Serial.println("  @WEB SERVER STATUS     (read-only; is the listening socket up)");
  Serial.println("  @WEB SERVER START|STOP (MAINTENANCE mode only; never auto-started)");
  Serial.println("  @AUTHORITY STATUS      (read-only; no owner can be acquired yet)");
  Serial.println("  @CALIBRATION STATUS    (read-only; no session can move hardware)");
  Serial.println("  @CALIBRATION PERSIST STATUS (read-only: NVS, marker, slots, last LOAD)");
  Serial.println("  @CALIBRATION PERSIST SAVE CHECK (read-only dry run of the SAVE prerequisites)");
  Serial.println("  @CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION (NVS only; ACK still required)");
  Serial.println("  @CALIBRATION PERSIST ACK <generation> (acknowledges a verified SAVE; no motion)");
  Serial.println("  @CALIBRATION PERSIST RECONCILE ADOPT <generation> [CONFIRM_DISCARD]");
  Serial.println("  @CALIBRATION PERSIST RECONCILE DECLARE_NOTHING [CONFIRM_DISCARD]");
  Serial.println("  @CALIBRATION Q0 STATUS (cached CR2-B acquisition state; no bus transaction)");
  Serial.println("  @CALIBRATION Q0 ABORT  (stop future q0 reads; no actuator command)");
  Serial.println("  @CALIBRATION Q0 CAPTURE <samples> <stability_ticks> CONFIRM_Q0_POSE");
  Serial.println("                           (ROBOT_POWERED + MAINTENANCE; read-only)");
  Serial.println("  @CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION");
  Serial.println("                           (MAINTENANCE only; RAM-only transform admission,");
  Serial.println("                           no bus transaction, no authority, no EEPROM write)");
  Serial.println("  @CALIBRATION SESSION START <LF|RF|RH|LH> CONFIRM_CURRENT_Q0");
  Serial.println("                           (starts that leg's live session from the current-boot Q0");
  Serial.println("                           population; one leg at a time, previous leg finalized)");
  Serial.println("  @CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION");
  Serial.println("                           (ROBOT_POWERED; exact RAM-only +16 tick permit)");
  Serial.println("  @CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER +16 CONFIRM_FIRST_MOTION");
  Serial.println("                           (ONLY reviewed first-motion command; no arbitrary target)");
  Serial.println("  @CALIBRATION MOTION ABORT");
  Serial.println("  @CALIBRATION MOTION PERMIT REVOKE");
  Serial.println("  @CALIBRATION SESSION ABORT");
  Serial.println("  @CALIBRATION INITIAL RECOVERY <LF|RF|RH|LH> CONFIRM_Q0_RECOVERY");
  Serial.println("  @CALIBRATION FULL LEG <LF|RF|RH|LH> CONFIRM_FULL_CALIBRATION");
  Serial.println("                           (the session's leg: all SIX contacts, UPPER/LOWER/HIP x");
  Serial.println("                           MIN/MAX, LF V25 order: q0 recovery, park, held");
  Serial.println("                           prerequisites, return, SAFE_OFF; minutes-long sequence)");
  Serial.println("  @CALIBRATION FULL LEG STATUS   (poll while the sequence runs)");
  Serial.println("  @CALIBRATION FULL LEG ABORT");
  Serial.println("  @CALIBRATION EVIDENCE EXPORT   (read-only; RAM record of every leg run this boot)");
  Serial.println("  @ACTUATOR STATUS       (read-only; no command can plan/commit/execute)");
  Serial.println("  @SYSTEM SOURCE_SIGNATURE  (read-only build/source identity)");
  Serial.println("  @HOSTLINK READINESS    (read-only; BLOCKED/TO_TEST/READY per capability)");
  Serial.println("  @SERVO SCAN <lo> <hi>   (MAINTENANCE mode only; incremental, bounded");
  Serial.println("                           per-ID blocking, result follows asynchronously)");
  Serial.println("  @SERVO CENSUS           (MAINTENANCE mode only; canonical 11-55 scan,");
  Serial.println("                           classified against the current servo configuration)");
  Serial.println("  @SERVO PREFLIGHT        (MAINTENANCE mode only; read-only 12-leg");
  Serial.println("                           identity + MATDOG_C018_V1 profile check)");
  Serial.println("  @SERVO READ <id>        (MAINTENANCE mode only)");
  Serial.println("  @SERVO SAFE_OFF <id>    (always allowed, any mode)");
  Serial.println("  @MODE STATUS|MAINTENANCE|RUN");
  Serial.println("  @SYSTEM SHUTDOWN");
}

// static
void CommandRouter::printAvailabilityLine(const char* label, const AvailabilityStatus& a) {
  Serial.printf("%s init=%s detected=%s expected=%s result=%s\n",
                label, toString(a.init), toString(a.detected), toString(a.expected),
                toString(classify(a)));
}

void CommandRouter::printModeStatus() {
  Serial.printf("MODE=%s\n", toString(modules_.operating_mode->mode()));
}

void CommandRouter::printOtaStatus() {
  const update::OtaManagerStatus& o = modules_.service->otaStatus();
  const update::OtaStatus& u = o.policy;

  Serial.printf("OTA_BOOT state=%s self_check=%s rollback_armed=%s ticks=%lu confirmed_at_ms=%lu\n",
                update::toString(o.boot_state), update::toString(o.self_check_fault),
                o.rollback_armed ? "YES" : "NO",
                (unsigned long)o.self_check_ticks, (unsigned long)o.confirmed_at_ms);
  Serial.printf("OTA_IMAGE running=%s @0x%06lx size=0x%06lx img_state=%s rollback_possible=%s\n",
                u.running.label, (unsigned long)u.running.address,
                (unsigned long)u.running.size, update::toString(u.running_image_state),
                u.rollback_possible ? "YES" : "NO");
  Serial.printf("OTA_IDENTITY build_id=%s readable=%s reset_reason=%s fatal=%s\n",
                o.running_build_id, o.identity_readable ? "YES" : "NO",
                o.reset_reason, o.fatal_reset_reason ? "YES" : "NO");
  Serial.printf("OTA_UPDATE state=%s fault=%s gate=%s\n",
                update::toString(u.state), update::toString(u.fault),
                update::toString(u.last_gate_verdict));
  Serial.printf("OTA_TARGET label=%s @0x%06lx size=0x%06lx boot_target_changed=%s\n",
                u.target.valid ? u.target.label : "(unresolved)",
                (unsigned long)u.target.address, (unsigned long)u.target.size,
                u.boot_target_changed ? "YES" : "NO");
  Serial.printf("OTA_STREAM declared=%lu written=%lu chunks=%lu\n",
                (unsigned long)u.declared_size, (unsigned long)u.bytes_written,
                (unsigned long)u.chunks_written);
  Serial.printf("OTA_SHA declared=%s\n", u.declared_sha256[0] ? u.declared_sha256 : "(none)");
  Serial.printf("OTA_SHA computed=%s\n", u.computed_sha256[0] ? u.computed_sha256 : "(none)");
  Serial.printf("OTA_COUNTERS started=%lu committed=%lu failed=%lu aborted=%lu\n",
                (unsigned long)u.counters.updates_started,
                (unsigned long)u.counters.updates_committed,
                (unsigned long)u.counters.updates_failed,
                (unsigned long)u.counters.updates_aborted);
  // Flash erase and write DO block this loop. These are the measured costs,
  // so the OTA-B integration can argue from numbers rather than adjectives.
  Serial.printf("OTA_TIMING open_us=%lu write_us=%lu end_us=%lu tick_us=%lu/%lu err=%ld\n",
                (unsigned long)o.max_open_us, (unsigned long)o.max_write_us,
                (unsigned long)o.max_end_us, (unsigned long)o.last_update_us,
                (unsigned long)o.max_update_us, (long)o.last_backend_error);
  Serial.printf("OTA_INGEST=%s\n",
                o.ingest_enabled ? "ENABLED" : "DISABLED (OTA-A: no transport, no auth)");
}

void CommandRouter::printWebStatus() {
  // Lifecycle only: whether the listening socket is up. It carries no OTA
  // session state (that is not read here even by pointer) and no in-flight
  // request contents — see HttpTransport.h's cross-thread mailbox comment.
  Serial.printf("WEB_SERVER started=%s\n",
                modules_.http_transport->started() ? "YES" : "NO");
  Serial.println("WEB_NOTE never started from Controller::begin(); "
                 "MAINTENANCE mode required to start or stop it");
  Serial.printf("WEB_NOTE ota_ingest_compiled=%s\n",
                update::OtaManager::ingestEnabled() ? "ENABLED" : "DISABLED");
}

void CommandRouter::printWifiStatus() {
  const network::WifiStatus& w = modules_.service->wifiStatus();

  char ip[16];
  network::formatIpv4(w.ipv4, ip, sizeof(ip));

  Serial.printf("WIFI_STATE=%s fault=%s\n", network::toString(w.state),
                network::toString(w.fault));
  Serial.printf("WIFI_CONFIG enabled=%s credentials=%s ssid=%s\n",
                w.enabled ? "YES" : "NO",
                w.credentials_present ? "YES" : "NO",
                w.credentials_present ? w.ssid : "(none)");
  Serial.printf("WIFI_LINK connected=%s ip=%s rssi_dbm=%ld channel=%u\n",
                w.connected ? "YES" : "NO", ip, (long)w.rssi_dbm, (unsigned)w.channel);
  Serial.printf("WIFI_RETRY backoff_ms=%lu state_since_ms=%lu\n",
                (unsigned long)w.backoff_ms, (unsigned long)w.state_since_ms);
  Serial.printf("WIFI_COUNTERS radio_starts=%lu attempts=%lu connects=%lu reconnects=%lu "
                "timeouts=%lu link_losses=%lu\n",
                (unsigned long)w.counters.radio_starts,
                (unsigned long)w.counters.connect_attempts,
                (unsigned long)w.counters.connects,
                (unsigned long)w.reconnects(),
                (unsigned long)w.counters.connect_timeouts,
                (unsigned long)w.counters.link_losses);
  // The bounded-runtime claim, as a measurement the operator can read back.
  Serial.printf("WIFI_TICK last_us=%lu max_us=%lu\n",
                (unsigned long)w.last_update_us, (unsigned long)w.max_update_us);
}

void CommandRouter::printCalibrationStatus() {
  const calibration::CalibrationSessionStatus& c = modules_.service->calibrationStatus();

  // The headline fact, first: the installed robot has no valid calibration.
  Serial.printf("CALIBRATION_CURRENT state=%s hardware_motion=%s\n",
                c.current_calibration_stale ? "STALE_PENDING_FULL_RECALIBRATION" : "SEE_YAML",
                c.hardware_motion_authorized ? "AUTHORIZED" : "BLOCKED");
  Serial.println("CALIBRATION_NOTE source_of_truth=MATDOG_JOINT_CALIBRATION.yaml"
                 " calibration_reset:");
  Serial.printf("CALIBRATION_SESSION state=%s origin=%s leg=%s last_result=%s\n",
                calibration::toString(c.state), calibration::toString(c.origin),
                calibration::toString(c.leg), calibration::toString(c.last_result));
  Serial.printf("CALIBRATION_AUTHORITY held=%s generation=%lu\n",
                c.holds_authority ? "YES" : "NO", (unsigned long)c.lease_generation);
  // Read-only view of the RAM-only motion permit (granted by @CALIBRATION
  // MOTION PERMIT GRANT, re-checked every tick, revoked by any fact change).
  Serial.printf("CALIBRATION_MOTION_PERMIT_STATE active=%s operator_authorized=%s token_valid=%s\n",
                modules_.motion_permit->active() ? "YES" : "NO",
                modules_.motion_authorization->operator_authorized ? "YES" : "NO",
                modules_.motion_authorization->token.valid() ? "YES" : "NO");
  Serial.printf("CALIBRATION_POPULATION verdict=%s observed=%u/%u mask=0x%03x\n",
                calibration::toString(c.population_verdict),
                (unsigned)calibration::observedLegSlotCount(c.population),
                (unsigned)calibration::kLegServoSlotCount,
                (unsigned)c.population.observed_mask);
  Serial.printf("CALIBRATION_PHASE reported=%s last=%s contacts=%u\n",
                c.execution_phase_reported ? "YES" : "NO",
                calibration::toString(c.last_reported_phase),
                (unsigned)c.contacts_recorded);
  Serial.printf("CALIBRATION_RESTORE required=%s torque_off_required=%s cause=%s\n",
                c.restore.required ? "YES" : "NO",
                c.restore.torque_off_required ? "YES" : "NO",
                calibration::toString(c.restore.cause));
  Serial.printf("CALIBRATION_COUNTERS started=%lu completed=%lu aborted=%lu failed=%lu\n",
                (unsigned long)c.sessions_started, (unsigned long)c.sessions_completed,
                (unsigned long)c.sessions_aborted, (unsigned long)c.sessions_failed);
  Serial.println("CALIBRATION_LF_V25=HISTORICAL_HARDWARE_ORACLE (not current calibration)");
}

void CommandRouter::printCalibrationQ0Status() {
  ControllerService* s = modules_.service;
  const calibration::Q0CaptureStatus& q = s->calibrationQ0Status();
  const calibration::PopulationEvidenceBuildResult& population =
      s->calibrationQ0Population();

  Serial.printf("CALIBRATION_Q0 state=%s failure=%s session=%lu "
                "sample_passes=%u/%u next_joint=%u candidates=%u/%u\n",
                calibration::toString(q.state), calibration::toString(q.failure),
                (unsigned long)q.capture_session_id,
                (unsigned)q.completed_sample_passes, (unsigned)q.samples_per_joint,
                (unsigned)q.next_joint_index, (unsigned)q.candidates_complete,
                (unsigned)calibration::kLegServoSlotCount);
  Serial.printf("CALIBRATION_Q0_POPULATION status=%s verdict=%s observed=%u/%u\n",
                calibration::toString(q.population_status),
                calibration::toString(calibration::evaluateLegPopulation(population.evidence)),
                (unsigned)calibration::observedLegSlotCount(population.evidence),
                (unsigned)calibration::kLegServoSlotCount);

  if (q.state == calibration::Q0CaptureState::COMPLETE) {
    const actuator::Q0BootstrapCandidate* candidates = s->calibrationQ0Candidates();
    for (uint8_t i = 0; i < calibration::kLegServoSlotCount; ++i) {
      const actuator::Q0BootstrapCandidate& candidate = candidates[i];
      const calibration::Q0Evidence& e = candidate.evidence;
      Serial.printf("  Q0 bus=%u leg=%s joint=%s unit=%s tick=%u spread=%u "
                    "samples=%u state=%s estimator=%s\n",
                    (unsigned)candidate.bus_id,
                    calibration::toString(e.identity.leg),
                    calibration::toString(e.identity.joint),
                    e.identity.physical_unit,
                    (unsigned)e.tick,
                    (unsigned)candidate.stability_spread_ticks,
                    (unsigned)candidate.sample_count,
                    calibration::toString(e.state),
                    calibration::toString(e.estimator));
    }
    Serial.println("CALIBRATION_Q0_RESULT=12_CANDIDATES_ONLY");
    Serial.println("CALIBRATION_Q0_NOTE accepted=NO promoted=NO "
                   "transform_admitted=NO motion_authorized=NO");
  } else if (q.state == calibration::Q0CaptureState::FAILED) {
    Serial.println("CALIBRATION_Q0_RESULT=FAILED_NO_PROMOTION");
  } else {
    Serial.println("CALIBRATION_Q0_RESULT=IN_PROGRESS_OR_NOT_RUN");
  }
}

void CommandRouter::printActuatorStatus() {
  // Formatting ONLY, from the real SafeActuatorPolicy this Controller owns
  // as fail-closed infrastructure (I4/I5, 2026-09-25). No command reaches
  // plan()/commit()/execute()/abort() — see ControllerService.h.
  ControllerService* s = modules_.service;
  Serial.printf("ACTUATOR_POLICY epoch=%lu outstanding_transaction=%s last_decision=%s\n",
                (unsigned long)s->actuatorPolicyEpoch(),
                s->actuatorPolicyHasOutstandingTransaction() ? "YES" : "NO",
                actuator::toString(s->actuatorPolicyLastDecision()));
  Serial.printf("ACTUATOR_PROVENANCE limits_admitted=%u transforms_admitted=%u "
                "geometry_bound=%s\n",
                (unsigned)s->actuatorPolicyLimitsAdmitted(),
                (unsigned)s->actuatorPolicyTransformsAdmitted(),
                s->actuatorPolicyGeometryBound() ? "YES" : "NO");
  const actuator::ActuatorPolicyCounters& c = s->actuatorPolicyCounters();
  Serial.printf("ACTUATOR_COUNTERS plans=%lu plan_rejections=%lu commits=%lu "
                "commit_rejections=%lu aborts=%lu resets=%lu\n",
                (unsigned long)c.plans, (unsigned long)c.plan_rejections,
                (unsigned long)c.commits, (unsigned long)c.commit_rejections,
                (unsigned long)c.aborts, (unsigned long)c.resets);
  Serial.println("ACTUATOR_NOTE production backend is bound; the only command-reachable "
                 "motion paths are the exact CR3 LF_UPPER +16 DIRECTION_VERIFY gate and "
                 "@CALIBRATION FULL LEG <LF|RF|RH|LH>, each requiring live session + fresh RAM "
                 "permit + Geometry V5 + current authority");
  Serial.printf("ACTUATOR_NOTE hardware_motion_authorized=%s\n",
                calibration::CalibrationManager::hardwareMotionAuthorized() ? "YES" : "NO");
}

void CommandRouter::printFullLegCalibrationStatus() {
  const calibration::FullLegCalibrationExecutor& ex = *modules_.full_leg_calibration;
  const calibration::FullLegCalibrationStatus& s = ex.status();
  Serial.printf("CALIBRATION_FULL_LEG phase=%s step=%s failure=%s failed_phase=%s last_decision=%s\n",
                calibration::toString(s.phase), calibration::toString(s.step),
                calibration::toString(s.failure),
                s.failure == calibration::FullLegFailure::NONE ? "-" : calibration::toString(s.failed_phase),
                actuator::toString(s.last_policy_decision));
  Serial.printf("CALIBRATION_FULL_LEG_RUN mode=%s armed=%s executor_active=%s leg=%s contacts=%u/%u "
                "held=%u recovered=%u prerequisites=%s\n",
                ex.request().recovery_only ? "INITIAL_RECOVERY_ONLY" : "FULL_LEG",
                modules_.full_leg_run->armed ? "YES" : "NO", ex.active() ? "YES" : "NO",
                calibration::toString(ex.leg()), (unsigned)s.contacts_accepted,
                (unsigned)calibration::kFullLegContactCount, (unsigned)s.held_count,
                (unsigned)s.recovered_joints, ex.prerequisitesVerified() ? "VERIFIED" : "NO");
  Serial.printf("CALIBRATION_FULL_LEG_OP joint=%s bus=%u target=%u\n",
                calibration::toString(s.op_joint), (unsigned)s.op_bus, (unsigned)s.op_target_tick);
  // The owned contact probe's own verdict for the endpoint that ran last.
  const calibration::ContactProbeStatus& probe = ex.probeStatus();
  const calibration::ContactProbeRequest& pr = ex.probeRequest();
  Serial.printf("CALIBRATION_FULL_LEG_PROBE joint=%s side=%s phase=%s failure=%s pass=%u stage=%s "
                "target=%u pos=%ld speed=%ld current=%ld steps=%u bypass=%u scout=%u p1=%u p2=%u\n",
                calibration::toString(pr.endpoint_joint),
                pr.endpoint_side == calibration::ContactSide::MIN_SIDE ? "MIN" : "MAX",
                calibration::toString(probe.phase), calibration::toString(probe.failure),
                (unsigned)probe.pass, calibration::toString(probe.stage),
                (unsigned)probe.target_tick, (long)probe.last_position, (long)probe.last_speed,
                (long)probe.last_current, (unsigned)probe.step_count,
                (unsigned)probe.plateau_bypass_count,
                probe.scout_valid ? (unsigned)probe.scout_tick : 0u,
                (unsigned)probe.pass1_contact_tick, (unsigned)probe.pass2_contact_tick);
  const calibration::JointKind order[] = {calibration::JointKind::UPPER,
                                          calibration::JointKind::LOWER,
                                          calibration::JointKind::HIP};
  for (const calibration::JointKind kind : order) {
    for (uint8_t side = 0; side < calibration::kContactSideCount; ++side) {
      const calibration::ContactEvidence& e =
          ex.contact(kind, static_cast<calibration::ContactSide>(side));
      Serial.printf("CALIBRATION_FULL_LEG_CONTACT joint=%s side=%s measured=%s scout=%u fine1=%u "
                    "fine2=%u witness_accepted=%s\n",
                    calibration::toString(kind), side == 0 ? "MIN" : "MAX",
                    e.has_measurement ? "YES" : "NO", (unsigned)e.coarse_tick,
                    (unsigned)e.fine_tick_1, (unsigned)e.fine_tick_2,
                    e.witness.accepted() ? "YES" : "NO");
    }
  }

  // The verdict of a finished run is the finalizer's, kept in the RAM store:
  // the envelopes, limit admission, session completion and cleanup are all
  // decided there and none is recomputed here. Full detail per leg is
  // @CALIBRATION EVIDENCE EXPORT.
  const calibration::FullLegEvidenceStore& store = *modules_.full_leg_evidence;
  for (uint8_t i = 0; i < calibration::kLegCount; ++i) {
    const calibration::Leg leg = static_cast<calibration::Leg>(i);
    const calibration::FullLegRecord* record = store.find(leg);
    if (record == nullptr || !record->present) {
      Serial.printf("CALIBRATION_FULL_LEG_RECORD leg=%s present=NO verdict=NOT_RUN "
                    "contacts_accepted=0/%u\n",
                    calibration::toString(leg), (unsigned)calibration::kFullLegContactsExpected);
      continue;
    }
    Serial.printf("CALIBRATION_FULL_LEG_RECORD leg=%s present=YES attempts=%u verdict=%s "
                  "contacts_accepted=%u/%u contact_calibrated=%s envelope_accepted=%s failure=%s\n",
                  calibration::toString(leg), (unsigned)record->attempts,
                  calibration::toString(record->verdict), (unsigned)record->contacts_accepted,
                  (unsigned)record->contacts_expected,
                  record->hardware_contact_calibrated ? "YES" : "NO",
                  record->operational_envelope_accepted ? "YES" : "NO",
                  calibration::toString(record->failure));
  }
  Serial.printf("CALIBRATION_FULL_LEG_SUMMARY legs_present=%u legs_contact_calibrated=%u "
                "legs_envelope_accepted=%u total_contacts_accepted=%u/%u\n",
                (unsigned)store.legsPresent(), (unsigned)store.legsContactCalibrated(),
                (unsigned)store.legsEnvelopeAccepted(), (unsigned)store.totalContactsAccepted(),
                (unsigned)calibration::kFullCalibrationContactsExpected);
  Serial.println("CALIBRATION_FULL_LEG_NOTE FULL CALIBRATION = 4 legs x 3 joints x MIN/MAX = 24 "
                 "contacts; HARDWARE_CONTACT_CALIBRATED = all 6 of a leg's contacts (UPPER, LOWER, "
                 "HIP x MIN/MAX) recorded + diagnostics + verified SAFE_OFF + session completed; "
                 "FINAL_OPERATIONAL_ENVELOPE_ACCEPTED additionally needs APPROVED envelope "
                 "parameters (none exist in this build)");
}

// Starts a paced export; pumpFullLegEvidenceExport() prints it.
void CommandRouter::printFullLegEvidenceExport() {
  evidence_export_pending_ = true;
  evidence_export_next_ = 0;
  pumpFullLegEvidenceExport();
}

// At most kExportLinesPerTick lines per Controller tick, and never a line the
// USB CDC TX ring cannot hold: with the ring's 0 ms timeout an overrun is a
// silent drop, which for an evidence record is not acceptable. A host that is
// not reading simply stalls the export until it does; re-issuing the command
// restarts it from the top.
void CommandRouter::pumpFullLegEvidenceExport() {
  constexpr uint8_t kExportLinesPerTick = 4;
  for (uint8_t n = 0; n < kExportLinesPerTick; ++n) {
    ExportLineCapture capture;
    capture.want = evidence_export_next_;
    calibration::exportFullLegEvidence(*modules_.full_leg_evidence,
                                       modules_.actuator_policy->currentGeometryTag(),
                                       calibration::productionEnvelopeParameters(),
                                       captureExportLine, &capture);
    if (!capture.got) {
      evidence_export_pending_ = false;
      return;
    }
    const size_t needed = strlen(capture.line) + 2;
    if (static_cast<size_t>(Serial.availableForWrite()) < needed) return;
    Serial.println(capture.line);
    ++evidence_export_next_;
    if (evidence_export_next_ >= capture.seen) {
      evidence_export_pending_ = false;
      return;
    }
  }
}

void CommandRouter::printAuthorityStatus() {
  ControllerService* s = modules_.service;
  const AuthorityCounters& c = s->authorityCounters();

  Serial.printf("AUTHORITY owner=%s generation=%lu last_result=%s\n",
                toString(s->authorityOwner()), (unsigned long)s->authorityGeneration(),
                toString(s->authorityLastResult()));
  Serial.printf("AUTHORITY_INHIBIT active=%s reason=%s\n",
                s->authorityInhibited() ? "YES" : "NO", toString(s->authorityInhibitReason()));
  Serial.printf("AUTHORITY_MODE operating_mode=%s motion_allowed=%s service_allowed=%s\n",
                toString(s->operatingMode()),
                isModeCompatible(s->operatingMode(),
                                 ActuatorAuthority::MOTION) ? "YES" : "NO",
                isModeCompatible(s->operatingMode(),
                                 ActuatorAuthority::CALIBRATION) ? "YES" : "NO");
  Serial.printf("AUTHORITY_LAST_CLEAR %s\n", toString(s->authorityLastClearReason()));
  Serial.printf("AUTHORITY_COUNTERS grants=%lu already_owned=%lu releases=%lu "
                "rejections=%lu stale=%lu force_clears=%lu inhibits=%lu\n",
                (unsigned long)c.grants, (unsigned long)c.already_owned,
                (unsigned long)c.releases, (unsigned long)c.rejections,
                (unsigned long)c.stale_releases, (unsigned long)c.force_clears,
                (unsigned long)c.inhibit_grants);
  // The property that must survive every future phase: a safety
  // de-escalation is not arbitrated.
  Serial.println("AUTHORITY_NOTE SAFE_OFF is outside arbitration and always reachable");
}

void CommandRouter::printStatus() {
  ControllerService* s = modules_.service;
  // authority= is on the SYSTEM line rather than its own, to stay inside the
  // USB CDC TX ring budget the audit enforces (worst single-pass burst was
  // 2758 B against a 3072 B ring after Wi-Fi/OTA; this adds ~24 B).
  Serial.printf("SYSTEM health=%s power_state=%s mode=%s authority=%s uptime_ms=%lu "
                "profile=%s\n",
                toString(s->systemHealth()),
                toString(s->powerState()),
                toString(s->operatingMode()),
                toString(s->authorityOwner()),
                (unsigned long)s->uptimeMillis(millis()),
                build::kTestProfile);

  // Distinguishes "driver initialized" from "hardware physically detected"
  // from "was it even expected to be reachable right now" — see
  // core/Availability.h. This replaces the earlier imu=/servo=/bms=/led=
  // single-word summary, which could not express that.
  printAvailabilityLine("BNO085", s->imuAvailability());
  printAvailabilityLine("DALY  ", s->bmsAvailability());
  printAvailabilityLine("SERVO ", s->servoAvailability());
  printAvailabilityLine("LED   ", s->ledAvailability());

  // Declared servo configuration (compile-time facts) plus the verdict of
  // the last census, if one was run. NOT_RUN is the honest answer after a
  // boot with no census — @STATUS must never imply a population was
  // verified when no bus transaction ever happened.
  Serial.printf("SERVO_POP canonical=%u expected_now=%u absent_by_design=%u last_census=%s\n",
                (unsigned)servo::canonicalAllocatedCount(),
                (unsigned)servo::expectedNowCount(),
                (unsigned)servo::absentByDesignCount(),
                servo::toString(s->servoCensusResult().verdict));

  // One compact Wi-Fi line here; the full picture is @WIFI STATUS. @STATUS
  // has a real byte budget: static_audit.py sizes the USB CDC TX ring
  // against the worst single-pass burst, and with TX timeout 0 anything
  // past the ring is dropped rather than queued (G3.1).
  {
    const network::WifiStatus& w = s->wifiStatus();
    char ip[16];
    network::formatIpv4(w.ipv4, ip, sizeof(ip));
    Serial.printf("WIFI  state=%s connected=%s ip=%s rssi_dbm=%ld fault=%s\n",
                  network::toString(w.state), w.connected ? "YES" : "NO", ip,
                  (long)w.rssi_dbm, network::toString(w.fault));
  }

  Serial.printf("  heap_free=%u heap_min_free=%u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}

void CommandRouter::printImuStatus() {
  ControllerService* s = modules_.service;
  printAvailabilityLine("BNO085", s->imuAvailability());
  Serial.printf("  stream=%s rv_count=%lu runtime_resets=%lu\n",
                s->imuStreamEnabled() ? "ON" : "OFF",
                (unsigned long)s->imuRvCount(),
                (unsigned long)s->imuRuntimeResetCount());
}

void CommandRouter::printBmsStatus() {
  ControllerService* s = modules_.service;
  printAvailabilityLine("DALY  ", s->bmsAvailability());
  Serial.printf("  comm=%s age_ms=%lu\n",
                power::toString(s->bmsLastCommResult()),
                (unsigned long)s->bmsLastResultAgeMs(millis()));

  if (s->bmsHasValidSample()) {
    const power::DalySample& sample = s->bmsSample();
    Serial.printf("  pack_v=%.1f current_a=%.1f soc=%.1f%% cells=%u\n",
                  sample.pack_voltage_v, sample.pack_current_a,
                  sample.soc_percent, sample.cell_count);
    Serial.printf("  cell_max_mv=%u cell_min_mv=%u delta_mv=%u\n",
                  sample.cell_max_mv, sample.cell_min_mv, sample.cell_delta_mv);
    Serial.printf("  charge_mos=%s discharge_mos=%s state=%s alarms=%04X %04X %04X %04X\n",
                  sample.charge_mos_on ? "ON" : "OFF",
                  sample.discharge_mos_on ? "ON" : "OFF",
                  sample.state_name,
                  sample.alarms[0], sample.alarms[1], sample.alarms[2], sample.alarms[3]);
  }
}

// Formatting only, from the stored snapshot. Configuration is reported
// exactly as read; nothing here is inferred from the live MOS state.
void CommandRouter::printBmsKeySnapshot(const power::DalyKeyConfigSnapshot& k) {
  Serial.printf("  key_logic_raw=0x%04X key_logic=%s\n",
                k.key_logic_raw, power::toString(k.key_logic));
  Serial.printf("  charge_mos_control=%u\n", k.charge_mos_control);
  Serial.printf("  discharge_mos_control=%u\n", k.discharge_mos_control);
  Serial.printf("  sleep_time_raw=%u sleep_time_s=%lu\n",
                k.sleep_time_raw, (unsigned long)k.sleep_time_seconds);
}

void CommandRouter::printBmsKeyReadResult() {
  ControllerService* s = modules_.service;
  const power::DalyKeyReadResult result = s->bmsKeyConfigReadResult();
  if (result != power::DalyKeyReadResult::OK) {
    Serial.printf("BMS_KEY_READ=COMPLETE result=%s rx_bytes=%u\n",
                  power::toString(result), (unsigned)s->bmsKeyConfigReadRxBytes());
    return;
  }
  Serial.println("BMS_KEY_READ=COMPLETE result=OK");
  printBmsKeySnapshot(s->bmsKeySnapshot());
}

void CommandRouter::printBmsKeyStatus() {
  // Zero bus transactions: cached state only.
  ControllerService* s = modules_.service;
  const uint32_t now_ms = millis();
  const power::DalyKeyReadResult result = s->bmsKeyConfigReadResult();

  if (result == power::DalyKeyReadResult::NOT_REQUESTED ||
      result == power::DalyKeyReadResult::PENDING) {
    Serial.printf("BMS_KEY_STATUS last_read=%s\n", power::toString(result));
  } else {
    Serial.printf("BMS_KEY_STATUS last_read=%s age_ms=%lu rx_bytes=%u\n",
                  power::toString(result), (unsigned long)s->bmsKeyConfigReadAgeMs(now_ms),
                  (unsigned)s->bmsKeyConfigReadRxBytes());
  }

  const power::DalyKeyConfigSnapshot& k = s->bmsKeySnapshot();
  if (!k.valid) {
    Serial.println("  snapshot=NOT_READ key_logic=UNKNOWN");
    return;
  }
  Serial.printf("  snapshot=VALID age_ms=%lu\n", (unsigned long)(now_ms - k.sampled_at_ms));
  printBmsKeySnapshot(k);
}

void CommandRouter::printBmsKeyWriteResult() {
  ControllerService* s = modules_.service;
  const power::DalyKeyWriteStatus& w = s->bmsKeyWriteStatus();
  if (w.state != power::DalyKeyWriteState::COMPLETE) {
    // Accepted, then refused by the last check before transmitting.
    Serial.printf("BMS_KEY_WRITE=%s reason=%s tx_bytes=0\n", power::toString(w.state),
                  power::toString(w.last_refusal));
    return;
  }
  Serial.printf("BMS_KEY_WRITE=COMPLETE ack=%s readback=%s\n", power::toString(w.ack),
                power::toString(w.readback));
  if (w.readback == power::DalyKeyReadback::READ_FAILED) {
    Serial.printf("  readback_rx_bytes=%u\n", (unsigned)s->bmsKeyConfigReadRxBytes());
    return;
  }
  printBmsKeySnapshot(s->bmsKeySnapshot());
}

void CommandRouter::printBmsKeyWriteStatus() {
  // Zero bus transactions: cached state only.
  const power::DalyKeyWriteStatus& w = modules_.service->bmsKeyWriteStatus();
  const uint32_t now_ms = millis();
  Serial.printf("BMS_KEY_WRITE_STATUS state=%s transmitted=%s last_refusal=%s\n",
                power::toString(w.state), w.transmitted ? "YES" : "NO",
                power::toString(w.last_refusal));
  if (w.ack != power::DalyKeyWriteAck::NONE && w.ack != power::DalyKeyWriteAck::PENDING) {
    Serial.printf("  ack=%s rx_bytes=%u age_ms=%lu\n", power::toString(w.ack),
                  (unsigned)w.ack_rx_bytes, (unsigned long)(now_ms - w.ack_at_ms));
  }
  if (w.readback == power::DalyKeyReadback::READ_FAILED) {
    Serial.printf("  readback=READ_FAILED age_ms=%lu\n",
                  (unsigned long)(now_ms - w.readback_at_ms));
  } else if (w.readback != power::DalyKeyReadback::NONE &&
             w.readback != power::DalyKeyReadback::PENDING) {
    Serial.printf("  readback=%s key_logic_raw=0x%04X age_ms=%lu\n", power::toString(w.readback),
                  w.readback_raw, (unsigned long)(now_ms - w.readback_at_ms));
  }
}

void CommandRouter::printSourceSignature() {
  const update::OtaManagerStatus& o = modules_.service->otaStatus();
  Serial.printf("SOURCE_SIGNATURE build_id=%s firmware=%s version=%s profile=%s board=%s\n",
                build::kBuildId, build::kFirmwareName, build::kFirmwareVersion,
                build::kTestProfile, build::kBoardName);
  Serial.printf("  ota_running_build_id=%s ota_running_image_state=%s reset_reason=%s\n",
                o.running_build_id, update::toString(o.policy.running_image_state),
                o.reset_reason);
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running != nullptr) {
    Serial.printf("  partition=%s address=0x%06x size=0x%06x\n",
                  running->label, (unsigned)running->address, (unsigned)running->size);
  }
}

void CommandRouter::printHostLinkReadiness() {
  // Read-only, computed-on-demand: HostLink's readiness classification is a
  // pure function of two facts (hardware_motion_authorized, and today's
  // unconditional "no hardware validated yet" for Wi-Fi/OTA), never a
  // second source of truth. See core/ServiceReadiness.h.
  ControllerService* s = modules_.service;
  for (ServiceCapability cap : {ServiceCapability::ACTUATOR_TORQUE_ENABLE,
                                ServiceCapability::ACTUATOR_POSITION_COMMAND,
                                ServiceCapability::CALIBRATION_CONTACT_PROBE,
                                ServiceCapability::CALIBRATION_AUXILIARY_MOVE,
                                ServiceCapability::CALIBRATION_DIRECTION_VERIFY,
                                ServiceCapability::WIFI_HARDWARE_ASSOCIATION,
                                ServiceCapability::OTA_END_TO_END,
                                ServiceCapability::WEB_READ_ONLY_DASHBOARD}) {
    Serial.printf("HOSTLINK_READINESS capability=%s readiness=%s\n", toString(cap),
                  toString(s->readiness(cap)));
  }
}

void CommandRouter::printLedStatus() {
  ControllerService* s = modules_.service;
  printAvailabilityLine("LED   ", s->ledAvailability());
  Serial.printf("  pixels=%u pin=%d brightness_max=%u test_running=%s data_pin_driven=%s\n",
                status::LedRing::kNumPixels,
                pins::kLedRingDin,
                status::LedRing::kMaxBrightness,
                s->ledTestRunning() ? "YES" : "NO",
                s->ledDataPinDriven() ? "YES" : "NO");
  // Presentation only - this is what the status manager last decided to
  // show, never a second source of truth. See status/LedStatusPolicy.h.
  const status::LedStatusSnapshot& led = s->ledSnapshot();
  Serial.printf("  presentation=%s soc_valid=%s soc_percent=",
                status::toString(led.presentation), led.soc_valid ? "YES" : "NO");
  if (led.soc_valid) {
    Serial.printf("%.1f", led.soc_percent);
  } else {
    Serial.print("UNKNOWN");
  }
  Serial.printf(" soc_segments=%u charging=%s charging_fault=%s charge_complete_verified=%s battery_warning=%s battery_critical=%s diagnostic=%s\n",
                (unsigned)led.soc_segments,
                led.charging ? "YES" : "NO", led.charging_fault ? "YES" : "NO",
                led.charge_complete_verified ? "YES" : "NO",
                led.battery_warning ? "YES" : "NO", led.battery_critical ? "YES" : "NO",
                status::toString(s->ledDiagnostic()));
}

void CommandRouter::printServoScanResult() {
  const servo::ScanResult& result = modules_.service->servoScanResult();
  Serial.printf("SERVO_SCAN=COMPLETE lo=%d hi=%d found=%d elapsed_ms=%lu max_ping_us=%lu\n",
                result.lo, result.hi, result.found_count,
                (unsigned long)result.elapsed_ms, (unsigned long)result.max_ping_us);
  printAvailabilityLine("SERVO ", modules_.service->servoAvailability());

  int listed = result.found_count < servo::ServoBus::kMaxScanIds
                   ? result.found_count
                   : servo::ServoBus::kMaxScanIds;
  for (int i = 0; i < listed; ++i) {
    Serial.printf("  FOUND id=%d\n", result.found_ids[i]);
  }
}

void CommandRouter::printServoCensusResult() {
  // Formatting ONLY. Every number below is read from the stored
  // CensusResult; none of it is computed here, and no servo transaction is
  // issued to render it.
  const servo::CensusResult& c = modules_.service->servoCensusResult();

  Serial.printf("SERVO_CENSUS=%s lo=%d hi=%d\n",
                servo::toString(c.verdict), c.scan_lo, c.scan_hi);
  Serial.printf("  canonical_allocated=%u expected_now=%u\n",
                (unsigned)c.canonical_allocated, (unsigned)c.expected_now);
  Serial.printf("  present_expected=%u missing_expected=%u absent_by_design=%u\n",
                (unsigned)c.present_expected, (unsigned)c.missing_expected,
                (unsigned)c.absent_by_design);
  Serial.printf("  absent_by_design_present=%u unexpected_id=%u not_probed=%u truncated=%s\n",
                (unsigned)c.absent_by_design_present, (unsigned)c.unexpected_id,
                (unsigned)c.not_probed, c.truncated ? "YES" : "NO");

  for (uint8_t i = 0; i < c.missing_id_count; ++i) {
    const servo::CanonicalServo* e = servo::findCanonical(c.missing_ids[i]);
    Serial.printf("  MISSING_EXPECTED id=%u joint=%s\n",
                  (unsigned)c.missing_ids[i], e != nullptr ? e->joint : "?");
  }
  for (uint8_t i = 0; i < c.absent_by_design_present_id_count; ++i) {
    const servo::CanonicalServo* e =
        servo::findCanonical(c.absent_by_design_present_ids[i]);
    Serial.printf("  ABSENT_BY_DESIGN_PRESENT id=%u joint=%s\n",
                  (unsigned)c.absent_by_design_present_ids[i],
                  e != nullptr ? e->joint : "?");
  }
  for (uint8_t i = 0; i < c.unexpected_id_count; ++i) {
    Serial.printf("  UNEXPECTED_ID id=%u\n", (unsigned)c.unexpected_ids[i]);
  }

  printAvailabilityLine("SERVO ", modules_.service->servoAvailability());
}

void CommandRouter::printServoPreflightResult() {
  // Formatting ONLY. Every value below was read by ServoPreflight; nothing is
  // computed here and no servo transaction is issued to render it.
  const servo::PreflightResult& r = modules_.service->servoPreflightResult();

  Serial.printf("SERVO_PREFLIGHT=%s profile=%s source_sha256=%s\n",
                r.allPass() ? "PASS" : "FAIL", servo::profile_data::kProfileId,
                servo::profile_data::kSourceSha256);
  Serial.printf("  evaluated=%u pass=%u no_response=%u mismatch=%u incomplete=%u\n",
                (unsigned)r.joints_evaluated, (unsigned)r.pass_count,
                (unsigned)r.no_response_count, (unsigned)r.mismatch_count,
                (unsigned)r.incomplete_count);
  // expected_physical_unit is CONFIGURATION. A servo cannot report its unit
  // label, so this column is never an observed hardware identity.
  Serial.println("  expected_unit joint expected_id observed_id model offset "
                 "profile torque position result");

  for (uint8_t i = 0; i < r.joints_evaluated; ++i) {
    const servo::JointPreflightRecord& j = r.joints[i];
    Serial.printf("  JOINT expected_physical_unit=%s joint=%s expected_bus_id=%u "
                  "observed_bus_id=%u model=%ld position_offset=%s%d "
                  "persistent_profile=%s torque_enable=%ld present_position=%ld "
                  "result=%s\n",
                  j.expected_physical_unit, j.joint, (unsigned)j.expected_bus_id,
                  (unsigned)j.observed_bus_id, (long)j.model,
                  j.position_offset_read ? "" : "UNREAD:",
                  j.position_offset_read ? (int)j.position_offset : 0,
                  servo::toString(j.profile), (long)j.torque_enable,
                  (long)j.present_position, servo::toString(j.result));
    if (j.profile_mismatch_count > 0) {
      Serial.printf("    PROFILE_MISMATCH count=%u first_addr=0x%02X expected=%u "
                    "observed=%ld\n",
                    (unsigned)j.profile_mismatch_count,
                    (unsigned)j.first_mismatch_address,
                    (unsigned)j.first_mismatch_expected,
                    (long)j.first_mismatch_observed);
    }
  }
  Serial.println("  NOTE present_position is a raw liveness tick, NOT q0");
  printAvailabilityLine("SERVO ", modules_.service->servoAvailability());
}

void CommandRouter::printServoRead(int id) {
  servo::ServoBus::RuntimeState state;
  if (!modules_.servo_bus->readRuntimeState(id, &state)) {
    Serial.printf("SERVO_READ id=%d result=NO_RESPONSE\n", id);
    printAvailabilityLine("SERVO ", modules_.servo_bus->availability());
    return;
  }

  Serial.printf("SERVO_READ id=%d position=%d speed=%d load=%d voltage=%d temp=%d torque=%d "
                "current=%d\n",
                id, state.present_position, state.present_speed, state.present_load,
                state.present_voltage, state.present_temperature, state.torque_enable,
                state.present_current);
}

void CommandRouter::printServoSafeOff(int id) {
  // Never prints a bare "OK" — see ServoBus::SafeOffResult (Session 2.2
  // Finding D) for why that previously gave a false safety guarantee with
  // no servo even connected.
  servo::SafeOffResult result = modules_.servo_bus->safeOff(id);
  Serial.printf("SERVO_SAFE_OFF id=%d result=%s\n", id, servo::toString(result));

  // CR3 continuation: a manual SAFE_OFF on a bus a motion executor currently
  // owns must never let that executor resume motion afterward. Cutting
  // torque alone is not enough — a *_PENDING phase (not yet monitoring
  // telemetry) would otherwise notice nothing and issue its next planned
  // write on the following tick. Revoking the permit (rather than calling
  // either executor's own abort() from here - CommandRouter/Controller may
  // only read status from the Safe Actuator/Calibration Execution
  // infrastructure, see check_actuator_infrastructure_wired_fail_closed())
  // is same-tick effective for exactly the reason @CALIBRATION MOTION ABORT
  // documents.
  const uint8_t safe_off_id = static_cast<uint8_t>(id);
  // The 24-contact sequence may energize ANY leg joint (INITIAL_RECOVERY) and
  // holds several at once, so while it runs every manual SAFE_OFF is treated
  // as an intervention: the permit goes, and the run fails closed into its
  // own verified SAFE_OFF of every leg joint.
  const bool owns_bus =
      (modules_.first_motion->active() && modules_.first_motion->busId() == safe_off_id) ||
      modules_.full_leg_calibration->active();
  if (owns_bus) {
    modules_.motion_permit->revoke(calibration::CalibrationPermitRevokeReason::EXPLICIT);
    modules_.motion_authorization->revoke();
  }
}

}  // namespace core
}  // namespace matdog
