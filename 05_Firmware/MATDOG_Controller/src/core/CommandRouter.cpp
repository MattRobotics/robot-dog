#include "CommandRouter.h"

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

  // The scan itself advances inside ServoBus::update() (called from
  // Controller::update() before this router runs); this just notices the
  // RUNNING -> COMPLETE transition and reports the result exactly once,
  // without the command handler having to block waiting for it.
  if (servo_scan_result_pending_ &&
      modules_.servo_bus->scanState() == servo::ScanState::COMPLETE) {
    servo_scan_result_pending_ = false;
    printServoScanResult();
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
  } else if (upper.startsWith("@SERVO SCAN")) {
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
  } else if (upper.startsWith("@SERVO READ")) {
    int id = -1;
    if (sscanf(upper.c_str(), "@SERVO READ %d", &id) == 1) {
      printServoRead(id);
    } else {
      Serial.println("ERROR=USAGE @SERVO READ <id>");
    }
  } else if (upper.startsWith("@SERVO SAFE_OFF")) {
    int id = -1;
    if (sscanf(upper.c_str(), "@SERVO SAFE_OFF %d", &id) == 1) {
      printServoSafeOff(id);
    } else {
      Serial.println("ERROR=USAGE @SERVO SAFE_OFF <id>");
    }
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
  Serial.println("  @LED STATUS");
  Serial.println("  @LED OFF");
  Serial.println("  @LED TEST");
  Serial.println("  @SERVO SCAN <lo> <hi>   (non-blocking; result follows asynchronously)");
  Serial.println("  @SERVO READ <id>");
  Serial.println("  @SERVO SAFE_OFF <id>");
  Serial.println("  @SYSTEM SHUTDOWN");
}

// static
void CommandRouter::printAvailabilityLine(const char* label, const AvailabilityStatus& a) {
  Serial.printf("%s init=%s detected=%s expected=%s result=%s\n",
                label, toString(a.init), toString(a.detected), toString(a.expected),
                toString(classify(a)));
}

void CommandRouter::printStatus() {
  SystemState* s = modules_.system_state;
  Serial.printf("SYSTEM health=%s power_state=%s uptime_ms=%lu profile=%s\n",
                toString(s->systemHealth()),
                toString(modules_.power_state->state()),
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

void CommandRouter::printLedStatus() {
  printAvailabilityLine("LED   ", modules_.led->availability());
  Serial.printf("  pixels=%u pin=%d brightness_max=%u test_running=%s data_pin_driven=%s\n",
                status::LedRing::kNumPixels,
                pins::kLedRingDin,
                status::LedRing::kMaxBrightness,
                modules_.led->testRunning() ? "YES" : "NO",
                modules_.led->dataPinDriven() ? "YES" : "NO");
}

void CommandRouter::printServoScanResult() {
  const servo::ScanResult& result = modules_.servo_bus->lastScanResult();
  Serial.printf("SERVO_SCAN=COMPLETE lo=%d hi=%d found=%d\n",
                result.lo, result.hi, result.found_count);
  printAvailabilityLine("SERVO ", modules_.servo_bus->availability());

  int listed = result.found_count < servo::ServoBus::kMaxScanIds
                   ? result.found_count
                   : servo::ServoBus::kMaxScanIds;
  for (int i = 0; i < listed; ++i) {
    Serial.printf("  FOUND id=%d\n", result.found_ids[i]);
  }
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
  bool ok = modules_.servo_bus->safeOff(id);
  Serial.printf("SERVO_SAFE_OFF id=%d result=%s\n", id, ok ? "OK" : "NO_RESPONSE");
}

}  // namespace core
}  // namespace matdog
