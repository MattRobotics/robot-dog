#ifndef MATDOG_IMU_BNO085_IMU_H
#define MATDOG_IMU_BNO085_IMU_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

#include "../config/Pins.h"
#include "../core/SystemState.h"

namespace matdog {
namespace imu {

// Acquisition wrapper around the hardware-proven BNO085 SPI transport from
// ~/MATDOG/runtime/esp32/matdog_bno085_dcd_phase_c3/matdog_bno085_dcd_phase_c3.ino
// (SHA256 51bb3016ac20226b812e1e892328768495348278cb19c320de34930cccd103e6).
//
// Preserves exactly: pin map, SPI init sequence, SH2_ROTATION_VECTOR as the
// orientation source, DCD autosave disabled, and the viewer-required text
// protocol lines (RV/MAG/GYR/COUNTS/SAVE_GATE/UNEXPECTED_RUNTIME_RESET),
// printed on the same ~500 ms cadence.
//
// Deliberately NOT preserved: the "SAVE" serial command and its
// sh2_saveDcdNow() path. Handoff section 12/24/28 requires that any
// DCD-save capability be separated from normal runtime and never triggered
// automatically; V0.1 removes the write path entirely rather than gate it,
// since no host consumer (the viewer is read-only) needs it.
//
// Deliberately DIFFERENT: on any SPI/session-config failure, the original
// standalone sketch calls a fatal() that halts forever. That is correct for
// a single-purpose bench sketch but wrong in a unified controller — a
// missing/failed IMU must degrade only the IMU module (handoff section 33).
// begin() therefore returns false and update() stops touching the sensor;
// the rest of the controller (servo/DALY/LED/USB) keeps running.
class Bno085Imu {
 public:
  bool begin();
  void update(uint32_t now_ms);
  core::ModuleHealth health() const { return health_; }

  bool streamEnabled() const { return stream_enabled_; }
  void setStreamEnabled(bool enabled) { stream_enabled_ = enabled; }

  uint32_t rvCount() const { return rv_count_; }
  uint32_t runtimeResetCount() const { return runtime_reset_count_; }

 private:
  bool configureCalibrationSession();
  void printTelemetryBlock();
  void printSaveGate();

  Adafruit_BNO08x bno08x_{pins::kBnoRst};
  sh2_SensorValue_t sensor_value_{};

  core::ModuleHealth health_ = core::ModuleHealth::NOT_INITIALIZED;
  bool stream_enabled_ = true;  // must default true: the viewer sends no commands.

  uint32_t mag_count_ = 0;
  uint32_t game_count_ = 0;
  uint32_t rv_count_ = 0;
  uint32_t acc_count_ = 0;
  uint32_t gyr_count_ = 0;

  uint32_t runtime_reset_count_ = 0;
  bool startup_reset_observed_ = false;

  float mag_x_ = 0, mag_y_ = 0, mag_z_ = 0;
  uint8_t mag_status_ = 0;

  float rv_w_ = 1, rv_x_ = 0, rv_y_ = 0, rv_z_ = 0;
  float rv_accuracy_ = 0;
  uint8_t rv_status_ = 0;

  float acc_x_ = 0, acc_y_ = 0, acc_z_ = 0;
  uint8_t acc_status_ = 0;

  float gyr_x_ = 0, gyr_y_ = 0, gyr_z_ = 0;
  float gyr_magnitude_ = 999.0f;
  uint8_t gyr_status_ = 0;

  uint32_t still_since_ms_ = 0;
  uint32_t last_print_ms_ = 0;
};

}  // namespace imu
}  // namespace matdog

#endif  // MATDOG_IMU_BNO085_IMU_H
