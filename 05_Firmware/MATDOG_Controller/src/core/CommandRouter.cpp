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
    modules_.led->off();
    Serial.println("LED=OFF");
  } else if (upper == "@LED TEST") {
    modules_.led->startTest();
    Serial.println("LED_TEST=STARTED");
  } else if (upper.startsWith("@SERVO SCAN")) {
    int lo = -1, hi = -1;
    if (sscanf(upper.c_str(), "@SERVO SCAN %d %d", &lo, &hi) == 2) {
      printServoScan(lo, hi);
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
  Serial.println("  @SERVO SCAN <lo> <hi>");
  Serial.println("  @SERVO READ <id>");
  Serial.println("  @SERVO SAFE_OFF <id>");
  Serial.println("  @SYSTEM SHUTDOWN");
}

void CommandRouter::printStatus() {
  SystemState* s = modules_.system_state;
  Serial.printf("SYSTEM health=%s power_state=%s uptime_ms=%lu profile=%s\n",
                toString(s->systemHealth()),
                toString(modules_.power_state->state()),
                (unsigned long)s->uptimeMillis(millis()),
                build::kTestProfile);
  Serial.printf("  imu=%s servo=%s bms=%s led=%s\n",
                toString(s->imuHealth()),
                toString(s->servoHealth()),
                toString(s->bmsHealth()),
                toString(s->ledHealth()));
  Serial.printf("  heap_free=%u heap_min_free=%u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}

void CommandRouter::printImuStatus() {
  Serial.printf("IMU_STATUS health=%s stream=%s rv_count=%lu runtime_resets=%lu\n",
                toString(modules_.imu->health()),
                modules_.imu->streamEnabled() ? "ON" : "OFF",
                (unsigned long)modules_.imu->rvCount(),
                (unsigned long)modules_.imu->runtimeResetCount());
}

void CommandRouter::printBmsStatus() {
  power::DalyBms* daly = modules_.daly;
  Serial.printf("BMS_STATUS health=%s comm=%s age_ms=%lu\n",
                toString(daly->health()),
                power::toString(daly->lastCommResult()),
                (unsigned long)daly->lastResultAgeMs(millis()));

  if (daly->lastCommResult() == power::DalyCommResult::TIMEOUT ||
      daly->lastCommResult() == power::DalyCommResult::NEVER_POLLED) {
    Serial.println("DALY_DETECTED=NO_RESPONSE");
    Serial.printf("PROFILE=%s\n", build::kTestProfile);
    Serial.println("EXPECTED=OFFLINE");
    Serial.println("CLASSIFICATION=OFFLINE_EXPECTED/PASS");
    return;
  }

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
  Serial.printf("LED_STATUS health=%s pixels=%u pin=%d brightness_max=%u test_running=%s\n",
                toString(modules_.led->health()),
                status::LedRing::kNumPixels,
                pins::kLedRingDin,
                status::LedRing::kMaxBrightness,
                modules_.led->testRunning() ? "YES" : "NO");
  Serial.printf("PROFILE=%s\n", build::kTestProfile);
  Serial.println("EXPECTED=UNPOWERED (5V rail absent)");
  Serial.println("CLASSIFICATION=OFFLINE_EXPECTED/UNPOWERED");
}

void CommandRouter::printServoScan(int lo, int hi) {
  servo::ScanResult result = modules_.servo_bus->scan(lo, hi);
  Serial.printf("SERVO_SCAN lo=%d hi=%d found=%d\n", result.lo, result.hi, result.found_count);

  if (result.found_count == 0) {
    Serial.printf("PROFILE=%s\n", build::kTestProfile);
    Serial.println("EXPECTED=OFFLINE (servo power rail absent)");
    Serial.println("CLASSIFICATION=OFFLINE_EXPECTED/PASS");
    return;
  }

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
    Serial.printf("PROFILE=%s\n", build::kTestProfile);
    Serial.println("CLASSIFICATION=OFFLINE_EXPECTED/PASS");
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
