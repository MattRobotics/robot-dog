#include "Bno085Imu.h"

#include <math.h>

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
  a.init = init_;
  a.detected = detected_;
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

void Bno085Imu::update(uint32_t now_ms) {
  if (init_ == core::InitializationState::INIT_FAILED ||
      init_ == core::InitializationState::NOT_INITIALIZED) {
    return;
  }

  if (bno08x_.getSensorEvent(&sensor_value_)) {
    switch (sensor_value_.sensorId) {
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag_x_ = sensor_value_.un.magneticField.x;
        mag_y_ = sensor_value_.un.magneticField.y;
        mag_z_ = sensor_value_.un.magneticField.z;
        mag_status_ = sensor_value_.status;
        mag_count_++;
        break;

      case SH2_GAME_ROTATION_VECTOR:
        game_count_++;
        break;

      case SH2_ROTATION_VECTOR:
        rv_w_ = sensor_value_.un.rotationVector.real;
        rv_x_ = sensor_value_.un.rotationVector.i;
        rv_y_ = sensor_value_.un.rotationVector.j;
        rv_z_ = sensor_value_.un.rotationVector.k;
        rv_accuracy_ = sensor_value_.un.rotationVector.accuracy;
        rv_status_ = sensor_value_.status;
        rv_count_++;
        break;

      case SH2_ACCELEROMETER:
        acc_x_ = sensor_value_.un.accelerometer.x;
        acc_y_ = sensor_value_.un.accelerometer.y;
        acc_z_ = sensor_value_.un.accelerometer.z;
        acc_status_ = sensor_value_.status;
        acc_count_++;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyr_x_ = sensor_value_.un.gyroscope.x;
        gyr_y_ = sensor_value_.un.gyroscope.y;
        gyr_z_ = sensor_value_.un.gyroscope.z;
        gyr_status_ = sensor_value_.status;
        gyr_magnitude_ = magnitude3(gyr_x_, gyr_y_, gyr_z_);
        gyr_count_++;

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

  if (bno08x_.wasReset()) {
    if (!startup_reset_observed_) {
      startup_reset_observed_ = true;
      Serial.println("EXPECTED_STARTUP_RESET=YES");
    } else {
      runtime_reset_count_++;
      Serial.printf("UNEXPECTED_RUNTIME_RESET count=%lu\n",
                    (unsigned long)runtime_reset_count_);
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

  if (stream_enabled_ && (now_ms - last_print_ms_ >= 500)) {
    last_print_ms_ = now_ms;
    printTelemetryBlock();
  }
}

void Bno085Imu::printSaveGate() {
  const bool ready =
      runtime_reset_count_ == 0 &&
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
      (unsigned long)runtime_reset_count_);
}

void Bno085Imu::printTelemetryBlock() {
  Serial.printf(
      "MAG x=%.3f y=%.3f z=%.3f |B|=%.3f status=%u count=%lu\n",
      mag_x_, mag_y_, mag_z_, magnitude3(mag_x_, mag_y_, mag_z_),
      mag_status_, (unsigned long)mag_count_);

  Serial.printf(
      "RV w=%.6f x=%.6f y=%.6f z=%.6f accuracy_rad=%.6f status=%u count=%lu\n",
      rv_w_, rv_x_, rv_y_, rv_z_, rv_accuracy_, rv_status_,
      (unsigned long)rv_count_);

  Serial.printf(
      "GYR x=%.6f y=%.6f z=%.6f |w|=%.6f status=%u\n",
      gyr_x_, gyr_y_, gyr_z_, gyr_magnitude_, gyr_status_);

  printSaveGate();

  Serial.printf(
      "COUNTS acc=%lu gyr=%lu mag=%lu game=%lu rv=%lu runtime_resets=%lu\n\n",
      (unsigned long)acc_count_, (unsigned long)gyr_count_,
      (unsigned long)mag_count_, (unsigned long)game_count_,
      (unsigned long)rv_count_, (unsigned long)runtime_reset_count_);
}

}  // namespace imu
}  // namespace matdog
