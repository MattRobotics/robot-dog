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

## 0.1.0 — Session 2.2 final merge gate — 2026-09-15

Four findings from a final review of Session 2.1, fixed before merge — no
new functionality. Flashed and hardware-validated as commit
`cb53c63206b0ccad68055ecc993a0a9e03f5b545` (`FIRMWARE_SOURCE_COMMIT`; see
`VALIDATION.md` Session 2.2).

- `scripts/ota_partition_logic.py` (Finding A): OTA subtype recognition
  corrected from a `subtype >= 0x10` threshold to the real ESP-IDF bitmask
  `(subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG` — the old check wrongly
  matched `PART_SUBTYPE_TEST`(0x20) and `PART_SUBTYPE_TEE_0/1`(0x30/0x31)
  as if they were OTA app slots. Also refuses on duplicate OTA slot
  indices instead of silently picking one.
- `scripts/ota_partition_logic.py` (Finding B): the real, installed build
  has `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (confirmed by reading the
  actual sdkconfig, not assumed). `resolve_application_partition()` now
  takes required `rollback_enabled`/`anti_rollback_enabled` parameters
  (no default) and refuses when anti-rollback is enabled at all, or when
  rollback is enabled and the selected otadata entry's state is
  `NEW`/`PENDING_VERIFY` (states the bootloader can autonomously rewrite
  on the next boot). `verify_application_partition.py` now requires
  `--sdkconfig`; `flash_app_only.sh` passes it and prints both flags
  before writing. The real device's state (`ota_state=UNDEFINED`) is
  stable regardless, so flashing proceeded correctly this session.
- `servo/ServoBus` (Finding C): `kPingTimeoutMs` (20ms) is no longer set
  globally in `begin()` — it was silently becoming a standing override for
  every future SCServo call. `ScopedPingTimeout` (RAII) now brackets each
  individual bus transaction and restores the library's own 100ms default
  afterward, on every exit path.
- `servo/ServoBus` (Finding D): `safeOff()` returned `bool` based on
  `EnableTorque()`'s own return value, which is `SCS::Ack()` — `0` on any
  failure, not `-1` like `Ping()`/`readByte()`, so `result >= 0` was always
  true and could never observe a failure. Confirmed live last session:
  `@SERVO SAFE_OFF 11` reported `OK` with the servo bus completely
  unpowered. `safeOff()` now returns `SafeOffResult`
  (`VERIFIED_OFF`/`UNVERIFIED_NO_RESPONSE`/`VERIFY_FAILED`), classified
  strictly from an independent `TorqueEnable` readback taken after the
  write, never from the write's own ACK. Re-tested live this session:
  `UNVERIFIED_NO_RESPONSE`, correctly.
- `core/OperatingMode.h`, `core/PowerState.h`: cross-reference comments
  clarifying `OperatingMode::RUN` and `PowerState::RUN` are orthogonal
  concepts that share a name coincidentally. `OperatingMode`'s approved
  MAINTENANCE/RUN boundary itself is unchanged.
- `scripts/static_audit.py`: five new regression checks for the above;
  27/27 OTA parser offline tests (17 new this session).

See `VALIDATION.md` Session 2.2 for the full measurement evidence, the
ESP-IDF rollback source citations, and H1/H3/H4/H5/H6 hardware
re-validation including live SAFE_OFF and OperatingMode transcripts.

## 0.1.0 — Session 2.3 final consistency fix — 2026-09-15

Three consistency findings from a final review of Session 2.2, fixed before
merge — no new functionality. Flashed and hardware-validated as commit
`5b371da5482f9b0bd2df1c37ed361250ea54ae8f` (`FIRMWARE_SOURCE_COMMIT`; see
`VALIDATION.md` Session 2.3).

- `servo/ServoBus` (Finding 1): Session 2.2's fix still applied the 20ms
  diagnostic timeout to *every* transaction, including `safeOff()` and
  `readRuntimeState()` — so the documented "operational timeout = 100ms,
  diagnostic timeout = 20ms" split was not actually true in code.
  `ScopedPingTimeout` is renamed `ScopedIOTimeout` and now takes an explicit
  timeout argument at every call site; `kDiagnosticTimeoutMs` (20ms, still
  `ping()`/`readModel()`/the scan's per-ID probe) and `kOperationalTimeoutMs`
  (100ms, new: `safeOff()`/`readRuntimeState()` — the safety de-escalation
  path and the primitive a future motion controller will reuse) are now two
  separately named constants.
- `scripts/ota_partition_logic.py` (Finding 2): `parse_sdkconfig_ota_flags()`
  treated a Kconfig symbol completely absent from the sdkconfig text the
  same as one explicitly disabled (both → `False`) — not fail-closed.
  `parse_sdkconfig_flag()` now returns a three-way `SdkconfigFlag`
  (`ENABLED`/`DISABLED`/`UNKNOWN`); `resolve_application_partition()`
  REFUSEs on `UNKNOWN` for either `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` or
  `CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK`, exactly like it does on `ENABLED`.
  The real build's config (`ROLLBACK_ENABLE=y`, `ANTI_ROLLBACK` not set)
  still resolves correctly — re-verified against the actual device.
- `scripts/ota_partition_logic.py` (Finding 3): `ota_app_partitions()` now
  requires OTA slot indices to be exactly `{0, ..., N-1}`; a sparse set
  (`{1}`, `{0,2}`, `{0,1,3}`, ...) raises `OtaAmbiguous` instead of
  resolving. MATDOG has no use for a sparse OTA layout in V0.1; this avoids
  ambiguity between raw OTA subtype numbers and the bootloader's own
  `app_count`/modulo slot selection ahead of any future OTA subsystem.
- `scripts/tests/test_ota_partition_logic.py`: removed
  `test_flags_absent_entirely_defaults_to_disabled`, which encoded the
  non-fail-closed policy Finding 2 corrects; added coverage for
  symbol-absent → `UNKNOWN`, `UNKNOWN` → REFUSE (both symbols,
  independently and together), and the four contiguity scenarios above.
  27 → 40 tests, all PASS.
- `scripts/static_audit.py`: fixed references broken by the Finding 1
  rename and the Finding 2/3 API changes; added a check that
  `safeOff()`/`readRuntimeState()` use `kOperationalTimeoutMs` and never
  `kDiagnosticTimeoutMs`, and extended the OTA fail-closed check to require
  `SdkconfigFlag.UNKNOWN` handling and the slot-contiguity check.

See `VALIDATION.md` Session 2.3 for the full measurement evidence
(including live `@SERVO SAFE_OFF` timing with the servo bus unpowered) and
H1/H2/H3/H4/H5/H6 hardware re-validation.
