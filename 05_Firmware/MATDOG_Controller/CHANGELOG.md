# MATDOG Controller — Changelog

## Unreleased — G3 / G3.1 live closure — 2026-09-18

Documentation only; no firmware change. Records the live validation of `e2fc605`.

- **G3 formal PASS:** a second read-only `@SERVO CENSUS` matched the first exactly — census stable
  across repeats. The complete first-boot banner now shows `startup_motion` / `startup_torque` /
  `startup_servo_scan : DISABLED` directly.
- **G3.1 PASS:** with the host port closed 62 s after a host had opened and closed it, BNO085 RV
  ran at 50.10 Hz (0.68 Hz before the fix); 50.13 Hz open; 50.11 Hz with `@BMS STREAM` enabled;
  `runtime_resets` 0; every command reply complete once the backlog had drained.
- No commanded servo motion occurred and no robot motion was observed during validation.
- DALY `KEY` recorded as OPEN (not a validated shutdown barrier); GPIO19/20 stay native USB D−/D+.
- Next: DALY KEY investigation, then G4 — Diagnostics / Maintenance.

## Unreleased — G3.1 CDC-independent Controller loop — 2026-09-18

Fix for a regression found **live after the G3 powered session**. **Not flashed; live re-test TO_TEST.**
No servo, BMS, LED, KEY or power state was changed to produce it.

**Finding.** Zero-TX passive A/B on one boot, USB cable attached throughout: with the host's
CDC port closed, BNO085 rotation-vector processing fell to **0.68 Hz**; with it open,
**50.07 Hz** (configured 50 Hz). `runtime_resets` stayed 0. Autonomous Controller execution
depended on a host keeping the port open.

**Root cause (installed `esp32:esp32 3.3.11` `cores/esp32/HWCDC.cpp`).** HWCDC latches
`connected = true` on the first host transfer and clears it only on a USB bus reset or SOF
loss; closing the host tty is neither. In that state `HWCDC::write()` waits up to
`tx_timeout_ms` (100 ms) × 20 attempts ≈ 2 s per call on a full 256-byte TX ring.
`operator bool()` reports the same latched flag, so `if (Serial)` cannot guard it. The IMU
services one SH2 event per loop pass, so the stalls were lost acquisition.

- `src/core/Controller.cpp`: `Controller::begin()` calls `Serial.setTxBufferSize(3072)` and
  `Serial.setTxTimeoutMs(0)` before `Serial.begin()`. With timeout 0 every wait in
  `HWCDC::write()` becomes an immediate drop, so USB CDC transmit cannot stall the
  Controller. The ring holds the largest single loop-pass burst (2395 bytes): replies are
  expected complete while a host is reading and draining. Right after reopening a port that
  was closed for a long time, a stale backlog may still occupy the ring and a reply may
  short-write instead of block — accepted for this diagnostic surface, **not** a HostLink
  guarantee. `#error` guards pin hwcdc + CDC-on-boot and core 3.3.11.
- `src/config/BuildConfig.h`: `kUsbTxTimeoutMs = 0`, `kUsbTxRingBytes = 3072`.
- `scripts/static_audit.py`: `check_usb_cdc_tx_never_blocks` — timeout exactly 0, ring
  ≥ 2560, both set before `Serial.begin()` and any output, exactly one call each, guards
  present, no `Serial.flush()` / debug-output routing. 11/11 regression mutations caught.
- Docs: G3 powered no-motion evidence PASS, formal census-repeat criterion outstanding; G3.1
  gate added (`FAIL → FIX UNDER VALIDATION`); stage 4 onward and all motion `BLOCKED` until
  G3.1 PASS.

## Unreleased — pre-G3 provenance closure — 2026-09-16

Final pre-G3 closure amendment. **No hardware was flashed, no rail energized, G3 not
executed.** The compiled-in default profile remains `USB_ONLY`.

**Finding A — the recorded build FQBN went unverified.**
The build manifest has recorded `FQBN` since it was introduced, but `build_manifest.py
verify` never compared it against the FQBN `flash_app_only.sh` pins. The right source
compiled with the wrong toolchain configuration is still the wrong artifact: the FQBN
carries the partition scheme, flash size/mode, PSRAM mode, USB/CDC mode and CPU
frequency, and a partition-scheme change silently relocates the application partition.

- `scripts/build_manifest.py`: `verify_manifest()` now takes a required `expected_fqbn`
  keyword (no default) and refuses on exact inequality with the new stable reason
  `FQBN_MISMATCH`, whose detail names both the manifest value and the expected value.
  The CLI gains a required `--expected-fqbn` and reports `VERIFIED_FQBN` on success.
- `scripts/flash_app_only.sh`: passes its own pinned `"$FQBN"` into the verifier and
  surfaces the verified value in the pre-write report. The FQBN itself is unchanged.
- A successful verification now positively proves source commit + clean build state +
  clean current tree + application filename + exact binary size + exact binary SHA256 +
  hardware profile + FQBN all belong to the artifact being authorized.
- No existing flash protection was weakened: backup size/digest, device MAC,
  partition/otadata verification, rollback/anti-rollback, static audit, single
  application-partition write and post-write `verify-flash` are all retained.
- `scripts/tests/test_build_manifest.py`: 36 → **51 tests**, adding canonical-FQBN PASS,
  differing-FQBN `FQBN_MISMATCH`, per-option drift (partition scheme, flash size, PSRAM,
  CPU frequency, flash mode, CDC mode), exact-not-substring comparison, empty/missing
  FQBN falling to `MANIFEST_INCOMPLETE`, refusal ordering, and both profiles passing
  under their own authorization with the canonical FQBN.
- `scripts/static_audit.py`: new tripwires for a deleted FQBN comparison, a permissive
  `expected_fqbn` default, a defaulted `--expected-fqbn`, a removed `FQBN_MISMATCH`
  reason, and `flash_app_only.sh` no longer passing its pinned `"$FQBN"`. All six
  mutation-tested.

**Finding B — Development Gates could be read as reordering the roadmap.**
The HostLink gate's ENTRY read "Diagnostics/Maintenance foundation present", which could
be taken as authorizing HostLink immediately after Diagnostics, ahead of Service/
Provisioning/QC, calibration integration and formal recalibration.

- [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md): HostLink ENTRY now requires all
  preceding roadmap stages through formal recalibration, and a new global rule 6 states
  that a gate's technical prerequisites never override
  [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md) sequencing — reordering
  requires an explicit reviewed roadmap change. The roadmap sequence itself is unchanged
  and no HostLink implementation was started.

**Finding C** — `ARCHITECTURE.md` header date corrected to 2026-09-16, the date of the
approved Embedded Web UI decision it now contains.

## Unreleased — G2 pre-G3 hardening amendment — 2026-09-16

Resolves two issues raised by an independent review of commit `34afbc7`. Both are
pre-G3 corrections to the G2 software scope. **No hardware was flashed, no rail was
energized, and G3 was not executed.** The compiled-in default profile remains `USB_ONLY`.

**Finding 1 — the build profile was not bound to the flashed artifact.**
`build.sh` can emit a `USB_ONLY` or a `ROBOT_POWERED` image from the same commit to the
same path, and the build id is identical for both — so the pre-existing flash gate could
not tell them apart. A stale image of the wrong profile could have been written under the
wrong assumption. (Confirmed concrete: the two profiles really do produce different
binaries — 387312 vs 387776 bytes, different digests.)

- `scripts/build_manifest.py`: new. Writes and verifies a build manifest binding source
  commit, build id, clean/dirty source state, hardware profile, FQBN, and the binary's
  filename/size/SHA256. Pure logic plus a thin CLI; no device I/O.
- `build.sh`: emits `build/esp32.esp32.esp32s3/matdog_build_manifest.txt` after every
  successful compile, and removes any stale manifest if no binary was produced. The
  manifest lives in the gitignored build directory and is never committed.
- `flash_app_only.sh`: fail-closed manifest gate before any device write. Requires
  manifest present/parseable/known-version, commit == `HEAD`, tree clean at build time
  *and* now, binary present with matching size **and** SHA256, recognized profile, and
  manifest profile == requested profile. Profile is named via `MATDOG_FLASH_PROFILE`
  (default `USB_ONLY`), so a `ROBOT_POWERED` image is refused unless explicitly asked
  for. The verified profile is printed in a banner immediately before the write.
- No pre-existing protection was weakened: backup size/digest, device MAC, verified
  application partition, rollback/anti-rollback, static audit, single-partition write and
  post-write `verify-flash` are all retained and now individually audited.

**Finding 2 — `SystemHealth::READY` was unreachable under `ROBOT_POWERED`.**
`classify()` converted `DetectedState::UNKNOWN` into a verdict, treating "nothing has
established anything" the same as "we asked and it did not answer".

- `core/Availability.cpp`: `UNKNOWN` is now handled separately from `NO_RESPONSE`/
  `UNPOWERED`. `UNKNOWN + OPTIONAL -> PASS`; `UNKNOWN + REQUIRED -> UNKNOWN` (system
  reports `BOOTING`, a visible gap, not a false alarm). Observed-absence escalation is
  unchanged: `NO_RESPONSE + REQUIRED -> FAULT`, `NO_RESPONSE + OPTIONAL -> DEGRADED`.
- This fixed two defects with one rule: the LED ring (non-probeable, so permanently
  `UNKNOWN` when powered) no longer degrades the system, and the servo bus (`REQUIRED`
  but deliberately never probed at boot) no longer reports `FAULT` before anything has
  been asked of it. The second case was found while evaluating the first across both
  profiles, as the review instructed.
- No physical detection is faked: the LED still reports `detected=UNKNOWN`, and the
  static audit forbids `detectedStateForLedRail()` from ever returning `ONLINE`.
- `USB_ONLY` classification is bit-for-bit unchanged — verified by a test that reproduces
  the hardware-validated Session 2 / H3 table exactly.
- `core/SystemState`: `beginBoot()` takes `now_ms` instead of calling `millis()`, making
  the translation unit Arduino-free so the offline tests link the real aggregation.

Tests and tooling:

- `scripts/tests/test_build_manifest.py`: new, 36 offline tests covering every refusal
  reason by exact code, the full authorization matrix, and write→verify round trips.
- `scripts/tests/test_servo_population.cpp`: +4 cases / +39 checks for the Availability
  truth table, the unchanged `USB_ONLY` boot table, `READY` reachability under
  `ROBOT_POWERED`, and a real optional-module failure still degrading.
- `static_audit.py`: two new check groups for profile provenance and the `UNKNOWN`
  distinction, plus anti-weakening assertions anchored to the actual comparisons in
  `flash_app_only.sh` rather than to token presence. All 15 new tripwires mutation-tested.

## Unreleased — G2 ROBOT_POWERED configuration support — 2026-09-16

Software preparation for the powered robot. **No powered hardware validation was
performed** — that is gate G3 and is authorized separately. The compiled-in default
hardware profile deliberately remains `USB_ONLY`.

Release identity is intentionally unchanged (`kFirmwareVersion` stays `0.1.0`): the
`0.2.x` number is decided at the release gate, not by a development branch name.
Development builds are distinguished by the git-SHA build id.

- `config`: new `HardwareProfile.h` — one enum (`USB_ONLY` / `ROBOT_POWERED`), one
  `expectationsFor()` mapping table. `BuildConfig.h` now *derives*
  `kServoPowerAvailable`/`kBatteryAvailable`/`kLedRailPowered`/`kTestProfile` from the
  selected profile instead of storing four independently editable facts that could
  contradict each other. Switching profiles is a one-symbol change.
- `core`: `Availability` gains `expectedStateForServoBus`/`Battery`/`LedRail` and
  `detectedStateForLedRail` — the profile → `ExpectedState` rule stated once instead of
  inlined per module. `classify()` itself is unchanged: the same `NO_RESPONSE` becomes
  `PASS` under `USB_ONLY` and `FAULT` under `ROBOT_POWERED`, which is what the V0.1
  model was designed for. `Availability.h`/`SystemState.h` now include `<stdint.h>`
  rather than `<Arduino.h>` so the offline host tests link the shipped logic.
- `servo`: new `ServoPopulation` — canonical 17 / expected-now 13 / absent-by-design 4
  (52-55) kept explicitly distinct, with pure per-ID and whole-census classification
  (`PRESENT_EXPECTED` / `MISSING_EXPECTED` / `ABSENT_BY_DESIGN` /
  `ABSENT_BY_DESIGN_PRESENT` / `UNEXPECTED_ID` / `NOT_PROBED`, verdict `PASS` /
  `PROFILE_MISMATCH` / `RANGE_INCOMPLETE` / `NOT_RUN`). Fail-closed: a partial or
  truncated scan can never report `PASS`. A healthy powered census is 13 present + 4
  absent by design — "17 = PASS" is never encoded anywhere.
- `servo`: new `ServoCensus` — Controller-owned service that drives the existing
  `ServoBus` scan state machine and holds the structured `CensusResult`. No second bus
  owner, no duplicate UART, no `Serial`, never auto-started.
- `core`: `@SERVO CENSUS` added to the USB command surface (MAINTENANCE-gated, like
  `@SERVO SCAN`). `CommandRouter` only formats the stored result — no G2 domain logic
  lives inside Serial parsing or printing, so a future Web UI / HostLink adapter can
  consume the same `CensusResult` without re-scanning the bus.
- Boot banner reports the profile with its rail facts, the declared servo population,
  and `startup_servo_scan : DISABLED` alongside the existing motion/torque lines.
- `scripts`: static audit extended with six G2 checks (profile authority + the
  `USB_ONLY`-default G3 gate, population model + YAML provenance cross-check, transport
  independence, no startup bus traffic, network→servo tripwire, host test suite). All
  pre-existing checks retained; every new tripwire was mutation-tested.
- `scripts/tests`: new offline C++ suite (`test_servo_population.cpp` +
  `run_host_tests.sh`) covering the required census classification cases and both
  profiles' expected-hardware semantics.

Explicitly NOT in this gate: Wi-Fi, HTTP/WebSocket/REST, Web UI, OTA transport, command
lease/deadman, teleoperation, IK/gait/pose, Safe Actuator, the full ActuatorAuthority
framework, provisioning/QC/source-signature/calibration integration, DALY writes, servo
EEPROM/ID writes, and any motion.

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
