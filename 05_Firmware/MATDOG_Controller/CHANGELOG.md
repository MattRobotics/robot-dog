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

## 0.1.0 — Session 2 hardening — 2026-09-15

Corrective/procedural hardening on top of the same 0.1.0 scope above — no new
firmware functionality, no motion/gait/IK/DALY-write/KEY work. Flashed and
hardware-validated as commit `04dfa52d1b5ab37ac4099792bb8c874c5ee4842a`
(`FIRMWARE_SOURCE_COMMIT`; see `VALIDATION.md` Session 2 for why this document's
own commit is necessarily later and must not be confused with it).

- `core`: adds `Availability` (`InitializationState`/`DetectedState`/
  `ExpectedState` → `Classification`), fixing `@STATUS` reporting a module as
  `OK` merely because its driver initialized rather than because the hardware
  was actually detected. `Controller::update()` now drives `ServoBus::update()`
  every tick.
- `status/LedRing`: no longer initializes the NeoPixel transport or transmits
  any WS2812 frame while `build::kLedRailPowered` is false (current `USB_ONLY`
  profile) — GPIO47 is left in a defined `INPUT` state instead. `@LED TEST`
  now refuses explicitly rather than driving an unpowered rail.
- `servo/ServoBus`: `@SERVO SCAN` is now a non-blocking one-`Ping()`-per-tick
  state machine instead of a synchronous loop that could monopolize `loop()`
  for seconds against an unresponsive ID range.
- `scripts/`: adds `flash_app_only.sh` (writes only the verified active
  application/OTA partition, never bootloader/partition-table/boot_app0) and
  `verify_application_partition.py` (reads the device's own partition table +
  otadata rather than assuming an offset). Corrects `upload.sh`'s header
  comment, which had incorrectly described a full Arduino upload as
  application-only.
- `scripts/static_audit.py`: four new regression checks for the above.

See `VALIDATION.md` Session 2 for the full read-only flash audit, the
application-only flash provenance record, and H1–H6 hardware results.

## 0.1.0 — Session 2.1 final pre-merge hardening — 2026-09-15

Two findings from an independent review of Session 2, fixed before merge —
no new functionality. Flashed and hardware-validated as commit
`07592b9d7f284eb6a24d18d69b57351281676e14` (`FIRMWARE_SOURCE_COMMIT`; see
`VALIDATION.md` Session 2.1).

- **Terminology correction**: Session 2's `@SERVO SCAN` was described as
  "non-blocking". `SCServo::Ping()` is still a synchronous call with its own
  bounded per-call timeout, so the accurate description is "incremental
  scan with bounded per-ID blocking". Corrected everywhere it was live
  documentation; Session 2's own record is preserved with a correction
  note, not rewritten.
- `core`: adds `OperatingMode` (`MAINTENANCE`/`RUN`) — a deliberately tiny
  boundary, not a state machine. `@SERVO SCAN`/`@SERVO READ` now refuse
  outside `MAINTENANCE`; `@SERVO SAFE_OFF` stays reachable in every mode
  (it can only remove torque). Default is `MAINTENANCE` (no motion loop
  exists yet); the future motion controller must flip the default to `RUN`.
- `servo/ServoBus`: `kPingTimeoutMs` reduces SCServo's 100ms default
  `IOTimeOut` to 20ms, justified by real hardware measurement (NEW01
  characterization campaign, ~600us measured round trip) rather than an
  assumed value — a public library field, not a vendored-source edit.
  `ScanResult` now reports measured `elapsed_ms`/`max_ping_us` instead of
  asserting scan cost. Measured this session: 45-ID scan in 1218ms
  (was ~4.5s), max single probe 20.815ms; BNO085 RV rate ~48.8Hz baseline
  vs ~47.3Hz during a scan.
- `scripts/ota_partition_logic.py` (new): replaces Session 2's OTA
  slot-selection logic, which compared raw `ota_seq` numbers and mislabeled
  the struct's `ota_state` field as `crc`, never validating the real CRC.
  Rewritten against the exact installed ESP-IDF v5.5.5 source
  (`bootloader_common_loader.c`/`bootloader_utility.c`), fails closed
  (`OtaAmbiguous`) on every state the real bootloader does not
  deterministically resolve. `scripts/tests/test_ota_partition_logic.py`:
  10/10 offline tests pass, including the exact regression case (a
  CRC-invalid entry with a higher raw sequence number than the valid one).
- `scripts/static_audit.py`: three new checks — MAINTENANCE-mode gating on
  servo scan/read (and that SAFE_OFF never gains one), the OTA parser's
  fail-closed primitives, and runs the OTA parser's own offline test suite
  as part of the audit.

See `VALIDATION.md` Session 2.1 for the full measurement evidence, the ESP-IDF
source citations, and H1/H2/H3/H4/H6 hardware re-validation.
