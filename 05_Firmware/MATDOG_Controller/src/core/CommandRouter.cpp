#include "CommandRouter.h"

#include "../servo/ServoProfileData.h"

#include "../config/BuildConfig.h"
#include "../config/Pins.h"

namespace matdog {
namespace core {

void CommandRouter::begin(const Modules& modules) {
  modules_ = modules;
  line_len_ = 0;
}

void CommandRouter::update(uint32_t now_ms) {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') continue;

    if (c == '\n') {
      line_buf_[line_len_] = '\0';
      if (line_len_ > 0) {
        handleLine(String(line_buf_));
      }
      line_len_ = 0;
      continue;
    }

    if (line_len_ < kLineBufSize - 1) {
      line_buf_[line_len_++] = c;
    }
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
}

void CommandRouter::handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

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
  } else if (upper == "@CALIBRATION STATUS") {
    printCalibrationStatus();
  } else if (upper == "@AUTHORITY STATUS") {
    printAuthorityStatus();
  } else if (upper == "@OTA STATUS") {
    printOtaStatus();
  } else if (upper.startsWith("@SERVO SCAN")) {
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
  Serial.println("  @WIFI STATUS           (cached snapshot; no radio query)");
  Serial.println("  @WIFI ON|OFF           (any mode; refused without credentials)");
  Serial.println("  @OTA STATUS            (read-only; OTA-A ships no transport)");
  Serial.println("  @AUTHORITY STATUS      (read-only; no owner can be acquired yet)");
  Serial.println("  @CALIBRATION STATUS    (read-only; no session can move hardware)");
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
  const update::OtaManagerStatus& o = modules_.ota->status();
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

void CommandRouter::printWifiStatus() {
  const network::WifiStatus& w = modules_.wifi->status();

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
  const calibration::CalibrationSessionStatus& c = modules_.calibration->status();

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

void CommandRouter::printAuthorityStatus() {
  const ActuatorAuthorityArbiter* a = modules_.authority;
  const AuthorityCounters& c = a->counters();

  Serial.printf("AUTHORITY owner=%s generation=%lu last_result=%s\n",
                toString(a->current()), (unsigned long)a->generation(),
                toString(a->lastResult()));
  Serial.printf("AUTHORITY_INHIBIT active=%s reason=%s\n",
                a->inhibited() ? "YES" : "NO", toString(a->inhibitReason()));
  Serial.printf("AUTHORITY_MODE operating_mode=%s motion_allowed=%s service_allowed=%s\n",
                toString(modules_.operating_mode->mode()),
                isModeCompatible(modules_.operating_mode->mode(),
                                 ActuatorAuthority::MOTION) ? "YES" : "NO",
                isModeCompatible(modules_.operating_mode->mode(),
                                 ActuatorAuthority::CALIBRATION) ? "YES" : "NO");
  Serial.printf("AUTHORITY_LAST_CLEAR %s\n", toString(a->lastClearReason()));
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
  SystemState* s = modules_.system_state;
  // authority= is on the SYSTEM line rather than its own, to stay inside the
  // USB CDC TX ring budget the audit enforces (worst single-pass burst was
  // 2758 B against a 3072 B ring after Wi-Fi/OTA; this adds ~24 B).
  Serial.printf("SYSTEM health=%s power_state=%s mode=%s authority=%s uptime_ms=%lu "
                "profile=%s\n",
                toString(s->systemHealth()),
                toString(modules_.power_state->state()),
                toString(modules_.operating_mode->mode()),
                toString(modules_.authority->current()),
                (unsigned long)s->uptimeMillis(millis()),
                build::kTestProfile);

  // Distinguishes "driver initialized" from "hardware physically detected"
  // from "was it even expected to be reachable right now" — see
  // core/Availability.h. This replaces the earlier imu=/servo=/bms=/led=
  // single-word summary, which could not express that.
  printAvailabilityLine("BNO085", modules_.imu->availability());
  printAvailabilityLine("DALY  ", modules_.daly->availability());
  printAvailabilityLine("SERVO ", modules_.servo_bus->availability());
  printAvailabilityLine("LED   ", modules_.led->availability());

  // Declared servo configuration (compile-time facts) plus the verdict of
  // the last census, if one was run. NOT_RUN is the honest answer after a
  // boot with no census — @STATUS must never imply a population was
  // verified when no bus transaction ever happened.
  Serial.printf("SERVO_POP canonical=%u expected_now=%u absent_by_design=%u last_census=%s\n",
                (unsigned)servo::canonicalAllocatedCount(),
                (unsigned)servo::expectedNowCount(),
                (unsigned)servo::absentByDesignCount(),
                servo::toString(modules_.servo_census->result().verdict));

  // One compact Wi-Fi line here; the full picture is @WIFI STATUS. @STATUS
  // has a real byte budget: static_audit.py sizes the USB CDC TX ring
  // against the worst single-pass burst, and with TX timeout 0 anything
  // past the ring is dropped rather than queued (G3.1).
  {
    const network::WifiStatus& w = modules_.wifi->status();
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
  printAvailabilityLine("BNO085", modules_.imu->availability());
  Serial.printf("  stream=%s rv_count=%lu runtime_resets=%lu\n",
                modules_.imu->streamEnabled() ? "ON" : "OFF",
                (unsigned long)modules_.imu->rvCount(),
                (unsigned long)modules_.imu->runtimeResetCount());
}

void CommandRouter::printBmsStatus() {
  power::DalyBms* daly = modules_.daly;
  printAvailabilityLine("DALY  ", daly->availability());
  Serial.printf("  comm=%s age_ms=%lu\n",
                power::toString(daly->lastCommResult()),
                (unsigned long)daly->lastResultAgeMs(millis()));

  if (daly->hasValidSample()) {
    const power::DalySample& sample = daly->sample();
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
  power::DalyBms* daly = modules_.daly;
  const power::DalyKeyReadResult result = daly->keyConfigReadResult();
  if (result != power::DalyKeyReadResult::OK) {
    Serial.printf("BMS_KEY_READ=COMPLETE result=%s rx_bytes=%u\n",
                  power::toString(result), (unsigned)daly->keyConfigReadRxBytes());
    return;
  }
  Serial.println("BMS_KEY_READ=COMPLETE result=OK");
  printBmsKeySnapshot(daly->keyConfigSnapshot());
}

void CommandRouter::printBmsKeyStatus() {
  // Zero bus transactions: cached state only.
  power::DalyBms* daly = modules_.daly;
  const uint32_t now_ms = millis();
  const power::DalyKeyReadResult result = daly->keyConfigReadResult();

  if (result == power::DalyKeyReadResult::NOT_REQUESTED ||
      result == power::DalyKeyReadResult::PENDING) {
    Serial.printf("BMS_KEY_STATUS last_read=%s\n", power::toString(result));
  } else {
    Serial.printf("BMS_KEY_STATUS last_read=%s age_ms=%lu rx_bytes=%u\n",
                  power::toString(result), (unsigned long)daly->keyConfigReadAgeMs(now_ms),
                  (unsigned)daly->keyConfigReadRxBytes());
  }

  const power::DalyKeyConfigSnapshot& k = daly->keyConfigSnapshot();
  if (!k.valid) {
    Serial.println("  snapshot=NOT_READ key_logic=UNKNOWN");
    return;
  }
  Serial.printf("  snapshot=VALID age_ms=%lu\n", (unsigned long)(now_ms - k.sampled_at_ms));
  printBmsKeySnapshot(k);
}

void CommandRouter::printBmsKeyWriteResult() {
  const power::DalyKeyWriteStatus& w = modules_.daly->keyWriteStatus();
  if (w.state != power::DalyKeyWriteState::COMPLETE) {
    // Accepted, then refused by the last check before transmitting.
    Serial.printf("BMS_KEY_WRITE=%s reason=%s tx_bytes=0\n", power::toString(w.state),
                  power::toString(w.last_refusal));
    return;
  }
  Serial.printf("BMS_KEY_WRITE=COMPLETE ack=%s readback=%s\n", power::toString(w.ack),
                power::toString(w.readback));
  if (w.readback == power::DalyKeyReadback::READ_FAILED) {
    Serial.printf("  readback_rx_bytes=%u\n", (unsigned)modules_.daly->keyConfigReadRxBytes());
    return;
  }
  printBmsKeySnapshot(modules_.daly->keyConfigSnapshot());
}

void CommandRouter::printBmsKeyWriteStatus() {
  // Zero bus transactions: cached state only.
  const power::DalyKeyWriteStatus& w = modules_.daly->keyWriteStatus();
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

void CommandRouter::printLedStatus() {
  printAvailabilityLine("LED   ", modules_.led->availability());
  Serial.printf("  pixels=%u pin=%d brightness_max=%u test_running=%s data_pin_driven=%s\n",
                status::LedRing::kNumPixels,
                pins::kLedRingDin,
                status::LedRing::kMaxBrightness,
                modules_.led->testRunning() ? "YES" : "NO",
                modules_.led->dataPinDriven() ? "YES" : "NO");
  // Presentation only - this is what the status manager last decided to
  // show, never a second source of truth. See status/LedStatusPolicy.h.
  Serial.printf("  presentation=%s\n", status::toString(modules_.led_status->state()));
}

void CommandRouter::printServoScanResult() {
  const servo::ScanResult& result = modules_.servo_bus->lastScanResult();
  Serial.printf("SERVO_SCAN=COMPLETE lo=%d hi=%d found=%d elapsed_ms=%lu max_ping_us=%lu\n",
                result.lo, result.hi, result.found_count,
                (unsigned long)result.elapsed_ms, (unsigned long)result.max_ping_us);
  printAvailabilityLine("SERVO ", modules_.servo_bus->availability());

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
  const servo::CensusResult& c = modules_.servo_census->result();

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

  printAvailabilityLine("SERVO ", modules_.servo_bus->availability());
}

void CommandRouter::printServoPreflightResult() {
  // Formatting ONLY. Every value below was read by ServoPreflight; nothing is
  // computed here and no servo transaction is issued to render it.
  const servo::PreflightResult& r = modules_.servo_preflight->result();

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
  printAvailabilityLine("SERVO ", modules_.servo_bus->availability());
}

void CommandRouter::printServoRead(int id) {
  servo::ServoBus::RuntimeState state;
  if (!modules_.servo_bus->readRuntimeState(id, &state)) {
    Serial.printf("SERVO_READ id=%d result=NO_RESPONSE\n", id);
    printAvailabilityLine("SERVO ", modules_.servo_bus->availability());
    return;
  }

  Serial.printf("SERVO_READ id=%d position=%d speed=%d load=%d voltage=%d temp=%d torque=%d\n",
                id, state.present_position, state.present_speed, state.present_load,
                state.present_voltage, state.present_temperature, state.torque_enable);
}

void CommandRouter::printServoSafeOff(int id) {
  // Never prints a bare "OK" — see ServoBus::SafeOffResult (Session 2.2
  // Finding D) for why that previously gave a false safety guarantee with
  // no servo even connected.
  servo::SafeOffResult result = modules_.servo_bus->safeOff(id);
  Serial.printf("SERVO_SAFE_OFF id=%d result=%s\n", id, servo::toString(result));
}

}  // namespace core
}  // namespace matdog
