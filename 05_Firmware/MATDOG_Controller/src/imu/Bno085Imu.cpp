#include "Bno085Imu.h"

#include <math.h>
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace matdog {
namespace imu {

namespace {
constexpr uint32_t kMagIntervalUs  = 20000;   // 50 Hz
constexpr uint32_t kGameIntervalUs = 20000;   // 50 Hz
constexpr uint32_t kRvIntervalUs   = 20000;   // 50 Hz
constexpr uint32_t kAccIntervalUs  = 100000;  // 10 Hz
constexpr uint32_t kGyrIntervalUs  = 100000;  // 10 Hz

constexpr float kMaxSaveAccuracyRad = 0.150f;
constexpr float kMaxStillGyroRadS   = 0.050f;
constexpr uint32_t kRequiredStillMs = 5000;

// Service pass bounds (IMU task, Core Motion V5). INT is level-low while the sensor
// has data for the host; a pass never waits, never exceeds these, and leaves any
// remainder to the next pass (counted as backlog).
constexpr uint32_t kMaxTransfersPerPass = 8;
constexpr uint32_t kPassBudgetUs        = 3000;
constexpr uint32_t kTaskStackBytes      = 8192;
constexpr unsigned kTaskPriority        = 2;   // above loopTask (1); core 0, away from the Motion thread
constexpr unsigned kTaskCore            = 0;

float magnitude3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}
}  // namespace

bool Bno085Imu::begin() {
  pinMode(pins::kBnoPs0, OUTPUT);
  digitalWrite(pins::kBnoPs0, HIGH);

  pinMode(pins::kBnoCs, OUTPUT);
  digitalWrite(pins::kBnoCs, HIGH);

  delay(20);

  if (!SPI.begin(pins::kBnoSck, pins::kBnoMiso, pins::kBnoMosi, pins::kBnoCs)) {
    Serial.println("IMU_INIT_FAIL=SPI_BEGIN");
    init_ = core::InitializationState::INIT_FAILED;
    detected_ = core::DetectedState::UNKNOWN;  // host-side SPI setup, not a chip handshake
    return false;
  }

  if (!bno08x_.begin_SPI(pins::kBnoCs, pins::kBnoInt, &SPI)) {
    Serial.println("IMU_INIT_FAIL=BNO08X_BEGIN_SPI");
    init_ = core::InitializationState::INIT_FAILED;
    detected_ = core::DetectedState::NO_RESPONSE;  // chip handshake itself failed
    return false;
  }

  // Per-report delivery BEFORE any report is enabled. The library's own handler
  // writes every report of a cargo into one shared value (last one wins) and
  // dereferences a pointer that is still NULL until the first getSensorEvent().
  sh2_setSensorCallback(&Bno085Imu::onSensorEvent, this);

  if (!configureCalibrationSession()) {
    Serial.println("IMU_INIT_FAIL=CALIBRATION_SESSION");
    init_ = core::InitializationState::INIT_FAILED;
    detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }

  Serial.println("IMU_INIT=PASS");
  // begin_SPI() already performed a real SHTP handshake with the physical
  // chip — that is genuine hardware detection, not just "driver ready".
  init_ = core::InitializationState::INITIALIZED;
  detected_ = core::DetectedState::ONLINE;
  return true;
}

core::AvailabilityStatus Bno085Imu::availability() const {
  core::AvailabilityStatus a;
  a.init = init_.load();
  a.detected = detected_.load();
  a.expected = core::ExpectedState::REQUIRED;  // 3V3-powered; always expected reachable
  return a;
}

bool Bno085Imu::configureCalibrationSession() {
  const uint8_t requested = SH2_CAL_ACCEL | SH2_CAL_MAG;

  if (sh2_setCalConfig(requested) != 0) return false;

  // No automatic DCD save — this module implements no SAVE command at all.
  if (sh2_setDcdAutoSave(false) != 0) return false;

  uint8_t actual = 0;
  if (sh2_getCalConfig(&actual) != 0) return false;
  if ((actual & requested) != requested) return false;

  bool ok = true;
  ok &= bno08x_.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, kMagIntervalUs);
  ok &= bno08x_.enableReport(SH2_GAME_ROTATION_VECTOR, kGameIntervalUs);
  ok &= bno08x_.enableReport(SH2_ROTATION_VECTOR, kRvIntervalUs);
  ok &= bno08x_.enableReport(SH2_ACCELEROMETER, kAccIntervalUs);
  ok &= bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED, kGyrIntervalUs);

  return ok;
}

bool Bno085Imu::dataReady() const {
  return digitalRead(pins::kBnoInt) == LOW;
}

void Bno085Imu::onSensorEvent(void* cookie, struct sh2_SensorEvent* event) {
  auto* self = static_cast<Bno085Imu*>(cookie);
  if (self == nullptr || event == nullptr) return;
  const sh2_SensorEvent_t* ev = reinterpret_cast<const sh2_SensorEvent_t*>(event);
  if (sh2_decodeSensorEvent(&self->decoded_, ev) != SH2_OK) return;
  // The SPI HAL leaves the cargo time at 0, so timestamp_uS is the sample time
  // RELATIVE to the interrupt (<= 0 when the sample precedes it): an age, not a clock.
  const int64_t relative = static_cast<int64_t>(ev->timestamp_uS);
  const int32_t age_us = (relative <= 0 && relative > -2000000) ? static_cast<int32_t>(-relative) : 0;
  self->consume(self->decoded_, age_us, millis());
}

void Bno085Imu::consume(const sh2_SensorValue_t& value, int32_t sensor_age_us, uint32_t now_ms) {
  switch (value.sensorId) {
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag_x_ = value.un.magneticField.x;
      mag_y_ = value.un.magneticField.y;
      mag_z_ = value.un.magneticField.z;
      mag_status_ = value.status;
      mag_count_++;
      feed_.noteOther(1);
      break;

    case SH2_GAME_ROTATION_VECTOR:
      game_count_++;
      feed_.noteOther(0);
      break;

    case SH2_ROTATION_VECTOR: {
      rv_w_ = value.un.rotationVector.real;
      rv_x_ = value.un.rotationVector.i;
      rv_y_ = value.un.rotationVector.j;
      rv_z_ = value.un.rotationVector.k;
      rv_accuracy_ = value.un.rotationVector.accuracy;
      rv_status_ = value.status;
      const float q[4] = {rv_w_, rv_x_, rv_y_, rv_z_};
      feed_.onRotationVector(q, rv_accuracy_, rv_status_, sensor_age_us, static_cast<uint32_t>(micros()), now_ms);
      break;
    }

    case SH2_ACCELEROMETER:
      acc_x_ = value.un.accelerometer.x;
      acc_y_ = value.un.accelerometer.y;
      acc_z_ = value.un.accelerometer.z;
      acc_status_ = value.status;
      acc_count_++;
      feed_.noteOther(2);
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      gyr_x_ = value.un.gyroscope.x;
      gyr_y_ = value.un.gyroscope.y;
      gyr_z_ = value.un.gyroscope.z;
      gyr_status_ = value.status;
      gyr_magnitude_ = magnitude3(gyr_x_, gyr_y_, gyr_z_);
      gyr_count_++;
      feed_.noteOther(3);

      if (gyr_magnitude_ <= kMaxStillGyroRadS) {
        if (still_since_ms_ == 0) still_since_ms_ = now_ms;
      } else {
        still_since_ms_ = 0;
      }
      break;

    default:
      break;
  }
}

void Bno085Imu::service(uint32_t now_ms) {
  const auto init = init_.load();
  if (init == core::InitializationState::INIT_FAILED ||
      init == core::InitializationState::NOT_INITIALIZED) {
    return;
  }

  const uint32_t started_us = static_cast<uint32_t>(micros());
  auto& stats = feed_.stats();
  ++stats.service_calls;
  if (have_service_) {
    const uint32_t gap = static_cast<uint32_t>(diffUs(started_us, last_service_us_));
    if (gap > stats.max_gap_us) stats.max_gap_us = gap;
  }
  have_service_ = true;
  last_service_us_ = started_us;

  // Never enter the SH-2 stack unless INT says a transfer is pending: with INT
  // high spihal_wait_for_int() polls delay(1) up to 500 times and then resets the
  // sensor. Every report of every cargo is delivered through onSensorEvent().
  const ServicePass pass = servicePass(
      [this]() { return dataReady(); },
      []() { sh2_service(); },
      []() { return static_cast<uint32_t>(micros()); },
      kMaxTransfersPerPass, kPassBudgetUs);
  stats.transfers += pass.transfers;
  if (pass.budget_stop) ++stats.budget_stops;
  if (pass.backlog_stop) ++stats.backlog_stops;
  if (pass.elapsed_us > stats.max_service_us) stats.max_service_us = pass.elapsed_us;

  if (bno08x_.wasReset()) {
    if (!startup_reset_observed_) {
      startup_reset_observed_ = true;
      Serial.println("EXPECTED_STARTUP_RESET=YES");
    } else {
      const uint32_t resets = runtime_reset_count_.fetch_add(1) + 1;
      feed_.noteReset();
      Serial.printf("UNEXPECTED_RUNTIME_RESET count=%lu\n", (unsigned long)resets);
    }

    still_since_ms_ = 0;

    if (!configureCalibrationSession()) {
      // Degrade the IMU module only; the rest of the controller continues
      // (handoff section 33 — no peripheral failure may hang the firmware).
      Serial.println("IMU_RECONFIG_FAIL=CALIBRATION_SESSION");
      init_ = core::InitializationState::INIT_FAILED;
      detected_ = core::DetectedState::NO_RESPONSE;
      return;
    }
  }

  if (stream_enabled_.load() && (now_ms - last_print_ms_ >= 500)) {
    last_print_ms_ = now_ms;
    printTelemetryBlock();
  }
}

bool Bno085Imu::startTask() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
  if (task_ != nullptr || init_.load() != core::InitializationState::INITIALIZED) return false;
  TaskHandle_t handle = nullptr;
  if (xTaskCreatePinnedToCore(&Bno085Imu::taskEntry, "md_imu", kTaskStackBytes, this, kTaskPriority, &handle, kTaskCore) != pdPASS) {
    return false;
  }
  task_ = handle;
  return true;
#else
  return false;
#endif
}

void Bno085Imu::taskEntry(void* self) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
  auto* imu = static_cast<Bno085Imu*>(self);
  for (;;) {
    imu->service(millis());
    vTaskDelay(1);   // one tick: INT is polled at the RTOS tick rate, nothing blocks
  }
#else
  (void)self;
#endif
}

void Bno085Imu::printSaveGate() {
  const bool ready =
      runtime_reset_count_.load() == 0 &&
      mag_status_ >= 2 &&
      rv_status_ >= 3 &&
      acc_status_ >= 2 &&
      rv_accuracy_ > 0.0f && rv_accuracy_ <= kMaxSaveAccuracyRad &&
      still_since_ms_ != 0 && (millis() - still_since_ms_) >= kRequiredStillMs;

  const uint32_t still_ms = still_since_ms_ == 0 ? 0 : (millis() - still_since_ms_);

  Serial.printf(
      "SAVE_GATE ready=%s still_ms=%lu ACC=%u GYR=%u MAG=%u RV=%u "
      "rv_accuracy_rad=%.6f gyro_mag=%.6f runtime_resets=%lu\n",
      ready ? "YES" : "NO",
      (unsigned long)still_ms,
      acc_status_, gyr_status_, mag_status_, rv_status_,
      rv_accuracy_, gyr_magnitude_,
      (unsigned long)runtime_reset_count_.load());
}

void Bno085Imu::printTelemetryBlock() {
  Serial.printf(
      "MAG x=%.3f y=%.3f z=%.3f |B|=%.3f status=%u count=%lu\n",
      mag_x_, mag_y_, mag_z_, magnitude3(mag_x_, mag_y_, mag_z_),
      mag_status_, (unsigned long)mag_count_);

  Serial.printf(
      "RV w=%.6f x=%.6f y=%.6f z=%.6f accuracy_rad=%.6f status=%u count=%lu\n",
      rv_w_, rv_x_, rv_y_, rv_z_, rv_accuracy_, rv_status_,
      (unsigned long)feed_.rvCount());

  Serial.printf(
      "GYR x=%.6f y=%.6f z=%.6f |w|=%.6f status=%u\n",
      gyr_x_, gyr_y_, gyr_z_, gyr_magnitude_, gyr_status_);

  printSaveGate();

  Serial.printf(
      "COUNTS acc=%lu gyr=%lu mag=%lu game=%lu rv=%lu runtime_resets=%lu\n\n",
      (unsigned long)acc_count_, (unsigned long)gyr_count_,
      (unsigned long)mag_count_, (unsigned long)game_count_,
      (unsigned long)feed_.rvCount(), (unsigned long)runtime_reset_count_.load());
}

}  // namespace imu
}  // namespace matdog
