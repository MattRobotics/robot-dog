#ifndef MATDOG_IMU_BNO085_IMU_H
#define MATDOG_IMU_BNO085_IMU_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

#include "../config/Pins.h"
#include "../core/Availability.h"
#include "../core/SystemState.h"
#include "ImuFeed.h"

struct sh2_SensorEvent;   // vendored SH-2 stack (see ImuFeed.h); only a pointer crosses this header

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
//
// Core Motion V5: acquisition runs in its OWN task (startTask()), never in the
// Motion/Controller thread. The library blocks with delay(1) polls (up to 500 ms,
// then a hardware reset) when INT is high, and keeps one event per call; both are
// avoided here - see ImuFeed.h. The Motion thread only reads orientationSnapshot().
class Bno085Imu {
 public:
  bool begin();
  // One bounded service pass: drains the SH-2 stack only while INT says data is
  // pending, one callback per report. Called by the task; also usable inline
  // (tests, or the Controller if the task could not be created).
  void service(uint32_t now_ms);
  void update(uint32_t now_ms) { service(now_ms); }
  bool startTask();
  bool taskRunning() const { return task_ != nullptr; }
  core::ModuleHealth health() const { return core::toModuleHealth(core::classify(availability())); }
  core::AvailabilityStatus availability() const;

  bool streamEnabled() const { return stream_enabled_.load(); }
  void setStreamEnabled(bool enabled) { stream_enabled_ = enabled; }

  // sampled_at_ms = host time the Rotation Vector was RECEIVED (cargo read), not
  // the start of a Controller loop. sensor_age_us is the sample's age at the
  // interrupt according to the SH-2 stack (0 if unknown); never an absolute clock.
  struct OrientationSnapshot { bool valid=false;uint32_t sampled_at_ms=0;double wxyz[4]{1,0,0,0};uint32_t seq=0;uint32_t rx_us=0;int32_t sensor_age_us=0;float accuracy_rad=0; };
  OrientationSnapshot orientationSnapshot() const {
    OrientationSnapshot o;RotationSample s;
    if(!feed_.snapshot(&s)||!s.valid){if(feed_.rvCount())snapshot_failures_.fetch_add(1);return o;}
    o.valid=true;o.sampled_at_ms=s.rx_ms;o.seq=s.seq;o.rx_us=s.rx_us;o.sensor_age_us=s.sensor_age_us;o.accuracy_rad=s.accuracy_rad;
    for(unsigned i=0;i<4;++i){o.wxyz[i]=s.wxyz[i];}
    return o;
  }
  uint32_t rvCount() const { return feed_.rvCount(); }
  // Reads that found no completed sample although one had been published (expected 0 with the two-slot snapshot).
  uint32_t snapshotReadFailures() const { return snapshot_failures_.load(); }
  uint32_t runtimeResetCount() const { return runtime_reset_count_.load(); }
  // Service statistics for the first-stop trace (single writer: the IMU task; torn
  // reads are possible and harmless for diagnostics).
  const ImuServiceStats& serviceStats() const { return feed_.stats(); }

 private:
  bool configureCalibrationSession();
  void printTelemetryBlock();
  void printSaveGate();
  static void onSensorEvent(void* cookie, struct sh2_SensorEvent* event);
  static void taskEntry(void* self);
  void consume(const sh2_SensorValue_t& value, int32_t sensor_age_us, uint32_t now_ms);
  bool dataReady() const;

  Adafruit_BNO08x bno08x_{pins::kBnoRst};
  sh2_SensorValue_t sensor_value_{};

  sh2_SensorValue_t decoded_{};
  // Written by the IMU task, read by the Controller thread: single-byte state.
  std::atomic<core::InitializationState> init_{core::InitializationState::NOT_INITIALIZED};
  std::atomic<core::DetectedState> detected_{core::DetectedState::UNKNOWN};
  std::atomic<bool> stream_enabled_{true};  // must default true: the viewer sends no commands.
  ImuFeed feed_;
  mutable std::atomic<uint32_t> snapshot_failures_{0};
  void* task_ = nullptr;
  uint32_t last_service_us_ = 0;
  bool have_service_ = false;

  uint32_t mag_count_ = 0;
  uint32_t game_count_ = 0;
  uint32_t acc_count_ = 0;
  uint32_t gyr_count_ = 0;

  std::atomic<uint32_t> runtime_reset_count_{0};
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
