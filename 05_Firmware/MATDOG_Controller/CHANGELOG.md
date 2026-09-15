# MATDOG Controller — Changelog

## 0.1.0 — 2026-09-15

First unified operational ESP32-S3 runtime. Integration milestone: brings ST3215
ServoBus, BNO085 IMU, DALY BMS telemetry and a new LED ring module into one modular,
deployable firmware image with a shared USB diagnostic surface, cooperative
non-blocking scheduling, module health aggregation and a power-state machine
baseline.

- `core`: `Controller`, `SystemState`, `PowerState`, `CommandRouter`.
- `servo`: `ServoBus` — read-only ST3215 transport adapted from the frozen Bench QC
  V6.1 source; `@SERVO SAFE_OFF` is the only write, torque-off only.
- `imu`: `Bno085Imu` — adapted from `matdog_bno085_dcd_phase_c3`; preserves the
  viewer-required `RV`/`MAG`/`GYR`/`COUNTS`/`SAVE_GATE` text protocol exactly; no
  DCD-save path exists in this firmware at all.
- `power`: `DalyBms` — adapted from `matdog_daly_rs485_probe_v2`; non-blocking poll
  state machine; `requestDischargeOff()` is a fail-closed placeholder pending K-Series
  write-protocol verification.
- `status`: `LedRing` — new module (no prior MATDOG source existed), built on
  Adafruit NeoPixel 1.15.5; boots OFF, brightness capped well under hardware maximum.
- Scope explicitly excludes gait, operational IK, closed-loop stabilization, ROS 2,
  Wi-Fi/OTA, autonomous behaviour, automatic calibration and any DALY/servo/BNO085
  write beyond the ones listed above.

See `SOURCE_PROVENANCE.md` for exact source hashes and `VALIDATION.md` for what has
and has not been exercised on real hardware.
