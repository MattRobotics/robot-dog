#include "Controller.h"

#include <esp_ota_ops.h>
#include <esp_system.h>

#include "../config/BuildConfig.h"
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

  imu_.begin();
  system_state_.setImuHealth(imu_.health());

  daly_.begin();
  system_state_.setBmsHealth(daly_.health());

  led_.begin();
  system_state_.setLedHealth(led_.health());

  CommandRouter::Modules modules{
      &servo_bus_, &servo_census_, &imu_, &daly_, &led_, &system_state_, &power_state_,
      &operating_mode_,
  };
  command_router_.begin(modules);

  system_state_.update();
  power_state_.enterPowerCheck();
  power_state_.enterRun();  // V0.1 has no motion-authorization gate to hold on.

  Serial.printf("SYSTEM_BOOT_COMPLETE health=%s power_state=%s\n",
                toString(system_state_.systemHealth()),
                toString(power_state_.state()));
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

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running != nullptr) {
    Serial.printf("partition  : %s @ 0x%06x (size 0x%06x)\n",
                  running->label, (unsigned)running->address, (unsigned)running->size);
  }

  Serial.printf("reset_reason : %s\n", resetReasonName(esp_reset_reason()));
  Serial.println("startup_motion   : DISABLED");
  Serial.println("startup_torque   : DISABLED");
  Serial.println("startup_servo_scan : DISABLED");
  Serial.println("daly_write       : KEY_LOGIC_DISCHARGE_ONLY (operator command; no MOS/power-cut write)");
  Serial.printf("operating_mode   : %s\n", toString(operating_mode_.mode()));
  Serial.println();
}

void Controller::update(uint32_t now_ms) {
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
  system_state_.setServoHealth(servo_bus_.health());

  system_state_.update();

  if (power_state_.state() == PowerState::SHUTDOWN_REQUESTED ||
      power_state_.state() == PowerState::SHUTTING_DOWN ||
      power_state_.state() == PowerState::POWER_CUT_REQUESTED) {
    power_state_.update(now_ms);
  }
}

}  // namespace core
}  // namespace matdog
