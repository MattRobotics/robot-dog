# MATDOG Controller

The first unified operational ESP32-S3 runtime for MATDOG. It replaces the previous
one-sketch-per-peripheral workflow (ST3215 bench tools, BNO085 Phase C3, DALY probes)
with one modular, deployable firmware image that initializes each subsystem module
together and reports whether its hardware is detected, expected or unavailable.

> **⚠ Flash safety — 2026-10-08, updated for PR-3.** This tree uses the flash layout
> `MATDOG_16M_2x5M_NVS_V1` (`partitions.csv`, `PartitionScheme=custom`), and `scripts/upload.sh`
> refuses every full-image upload. **Do not use this tree to update the robot.**
> - The firmware identity is `0.2.0-dev.3`: the sources correspond to the dev.3 candidate on the
>   robot (provenance `b3fd945`), apart from the deferred material. A build of this tree is a
>   different build from the installed image; binary identity is not asserted.
> - dev.3 is a candidate only (Hardware Validation 2026-10-07: execution COMPLETE, acceptance
>   BLOCKED). No release is approved, and these integrations are neither a flash authorization
>   nor a hardware acceptance.
> - The pure motion library under `src/motion` is unwired. Its execution suites and the
>   G35/G4/G4.1/G5-A evidence are deferred.
>
> `MOTION_AUTHORIZED=0`, `RESTORE=NOT_IMPLEMENTED`. See
> [`FLASH_LAYOUT_SAFETY_NOTICE.md`](FLASH_LAYOUT_SAFETY_NOTICE.md).

```text
MATDOG Controller
├── core/system      boot, version, health aggregation, power-state machine,
│                    cooperative non-blocking scheduling, USB command router
│                    ActuatorAuthority — the one arbiter of actuator write authority
├── calibration/     CalibrationDomain — pure model recovered from the LF V25 oracle
│                    CalibrationManager — session lifecycle over ActuatorAuthority
├── config/          HardwareProfile — the single USB_ONLY / ROBOT_POWERED authority
├── servo/           ServoBus — ST3215 / Seeed bus transport, read-only diagnostics
│                    ServoPopulation / ServoCensus — canonical 17 vs expected-now 13,
│                    live census classification (pure, transport-independent)
├── imu/             Bno085Imu — SH2_ROTATION_VECTOR acquisition, viewer-compatible
├── power/           DalyBms — read-only Modbus RTU battery telemetry
├── network/         WifiPolicy — pure Wi-Fi lifecycle state machine (host-linkable)
│                    WifiManager — the only translation unit that owns the radio
├── update/          OtaPolicy / OtaBootGuard / Sha256 — pure OTA state machine,
│                    first-boot rollback lifecycle, image identity (host-linkable)
│                    OtaEspBackend — the only unit that calls esp_ota_*
│                    OtaManager — Controller-facing owner; no transport in OTA-A
└── status/          LedRing — WS2812B ring, boots OFF, non-blocking effects
```

## Current hardware status — 2026-10-01

> **Historical snapshot (2026-10-01), not the current hardware state.** The results below are
> kept unchanged as evidence of that date. The robot has since been migrated to layout
> `MATDOG_16M_2x5M_NVS_V1` and runs the dev.3 candidate. See
> [`FLASH_LAYOUT_SAFETY_NOTICE.md`](FLASH_LAYOUT_SAFETY_NOTICE.md) for the current flash
> situation.

**TRUE Full Calibration: HARDWARE-VALIDATED 24/24.** The LF V25-derived full-leg sequence was
run on the real robot: LF/RF/RH/LH each reached `HARDWARE_CONTACT_CALIBRATED` 6/6 on the first
attempt. Firmware `dfcecb670d0565d2db1a8152b6cd7ad230bdb87d` (`ROBOT_POWERED`) ended SAFE_OFF 13/13 verified.

The 12 fresh q0 values and 24 contacts were exported and archived ([hardware-validation report](../../09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md)).
Calibration is RAM-only; the export is not an implemented boot restore. No operational envelope
is approved, and stand/gait hardware motion remains blocked. The frozen Controller V0.1 release
is distinct from the later development firmware validated in this session.

The [2026-09-29 calibration overview](#calibration-on-main-2026-09-29) below is retained as a
historical software-only snapshot, not the current project status.

## Official baseline

| Identity | Value |
|---|---|
| Official release tag | `matdog-controller-v0.1.0` |
| Tagged repository commit | `c54862f38a9cbd5e46d6b1770a6d109cc99b5c02` |
| Exact validated firmware source | `5b371da5482f9b0bd2df1c37ed361250ea54ae8f` |
| Validated application SHA256 | `6e6d92f898dbe95000b53dbb252c7eb5d3deaa9a4b161e2b1934436a76b29364` |
| Validated profile | `USB_ONLY` |

The tag identifies the official repository baseline; the source commit identifies the exact code
compiled, flashed and exercised. They are intentionally different because validation documentation
was committed after the firmware source. See [`VALIDATION.md`](VALIDATION.md) for the evidence and
scope.

## Permanent runtime direction

`MATDOG_Controller` is the permanent firmware architecture. Future reviewed stages are expected to
integrate Diagnostics, Maintenance, Service, Servo QC, Provisioning, Full Leg Calibration,
Wi-Fi/OTA and host transport here, followed later by Motion, IK, Gait and Stabilization.

The preserved Full Leg Calibrator V1 tree — annotated tag
`archive/2026-08-29/full-leg-calibrator-v1-h0` -> `15f3fb8f378e6cadf6bc479bfcaca2947741c9fd`, from
the former branch `matdog/full-leg-calibrator-v1`, archived 2026-09-18 — is an oracle for
calibration-engine, safety and evidence logic, not a replacement runtime and not a tree to merge
wholesale. Likewise, the
frozen [`ST3215_Bench_Tools`](../ST3215_Bench_Tools/README.md) remain immutable evidence even when
equivalent service capabilities are later integrated here.

## Standalone sources remain the evidence trail

Nothing under `05_Firmware/ST3215_Bench_Tools/` or `~/MATDOG/runtime/esp32/` was
modified. This firmware **adapts** hardware-proven transport/protocol code from those
sources into modules; it does not replace them as diagnostic tools. Full provenance,
SHA256 hashes and exactly what was preserved vs. deliberately left out is in
[`SOURCE_PROVENANCE.md`](SOURCE_PROVENANCE.md).

## Build

```bash
scripts/build.sh
```

Wraps `arduino-cli compile` with the pinned FQBN and injects a build id (short git SHA,
`-dirty` suffix if the tree has uncommitted changes) visible in the boot banner and
`@STATUS`:

`PartitionScheme=custom` makes the build take its partition table from `partitions.csv`
(`LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1`, pinned by `scripts/matdog_layout.py`; the legacy
`app3M_fat9M_16MB` scheme is refused):

```text
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi
```

Board: YD-ESP32-S3 N16R8 (16 MB flash, 8 MB OPI PSRAM, 240 MHz).

Every successful build also writes a **build manifest** next to the application
binary, inside the gitignored build directory:

```text
build/esp32.esp32.esp32s3/matdog_build_manifest.txt
```

It records the full source commit, build id, clean/dirty source state, selected
hardware profile, pinned FQBN, and the application binary's filename, size and SHA256.
`flash_app_only.sh` refuses to write anything it cannot verify against this file — see
Application-only flashing below. The manifest is a build artifact and is never
committed.

## Update and recovery policy

- **DECIDED — future normal update path:** Wi-Fi / OTA. It is not implemented in V0.1 *(V0.1
  statement; Wi-Fi/OTA V3 is on `main` since PR-2, OTA ingest disabled by default, hardware
  acceptance BLOCKED)*.
- **DECIDED — permanent wired service/recovery:** native USB CDC / USB-C remains available even
  after OTA is implemented. OTA must never remove the wired recovery path.
- **VALIDATED for the V0.1 baseline:** the application-only flashing workflow below operates over
  the wired USB service connection.

The phrase "OTA application partition" below names the ESP-IDF partition type. Writing that
partition with the current USB script is not an implementation of Wi-Fi/OTA.

## Application-only flashing

```bash
scripts/flash_app_only.sh
```

Writes **only** the currently-active OTA application partition — never the
bootloader, partition table or boot_app0/otadata. `scripts/verify_application_partition.py`
determines that partition's real offset/size by reading the device's own partition
table and otadata (layout `MATDOG_16M_2x5M_NVS_V1` has two 5 MiB OTA slots; a device that
still has the legacy table is refused, and the write range may never touch `matdog_nvs`;
this never assumes
which one is active, and recognizes OTA slots by the real ESP-IDF bitmask
`(subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG`, not a numeric threshold that would also
match TEST/TEE partitions, and must be a contiguous set `{0, ..., N-1}` — a sparse OTA
layout like `{0, 2}` refuses rather than resolving). It also reads the real build's
`sdkconfig` as a three-way `SdkconfigFlag` (`ENABLED`/`DISABLED`/`UNKNOWN`) and refuses if
`CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK` is `ENABLED` or `UNKNOWN`, or if
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` is `UNKNOWN`, or if it is `ENABLED` *and* the
selected otadata entry is in a state (`NEW`/`PENDING_VERIFY`) the bootloader can
autonomously rewrite on the next boot — a symbol absent from the sdkconfig text entirely
is `UNKNOWN`, never silently treated as `DISABLED`. See `scripts/ota_partition_logic.py`
and `VALIDATION.md` Session 2.3. Refuses to run unless: the working tree is clean and the
compiled binary's embedded build id matches `HEAD`, the 16 MiB full-flash backup
verifies, the static audit passes, and the connected device's MAC matches the expected
one. Prints `SDKCONFIG_ROLLBACK`/`SDKCONFIG_ANTI_ROLLBACK`/`DEVICE`/`APPLICATION_BINARY`/
`APPLICATION_SHA256`/`APPLICATION_OFFSET`/`APPLICATION_SIZE`/`MAX_PARTITION_SIZE`/`FQBN`/
`SOURCE_COMMIT` before writing anything, and independently re-verifies the write
afterward with `esptool verify-flash`.

### Hardware-profile provenance gate

`build.sh` can produce a `USB_ONLY` **or** a `ROBOT_POWERED` image from the same commit,
at the same path. The commit/build-id gate above cannot tell them apart — the build id
is identical for both — so a stale image of the wrong profile could be written under the
wrong assumption. Every flash is therefore additionally gated on the build manifest and
on the operator naming the profile they intend:

```bash
scripts/flash_app_only.sh                                 # defaults to USB_ONLY
MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh
```

Fail-closed. The write proceeds only if **all** of these hold: the manifest exists and
parses; its schema version is known; its source commit equals `HEAD`; the tree was clean
at build time *and* is clean now; the binary exists and its size **and** SHA256 equal the
recorded ones; the manifest profile is recognized; and it equals the requested profile.

| Manifest | Requested | Result |
|---|---|---|
| `USB_ONLY` | `USB_ONLY` (default) | allowed, subject to all other gates |
| `ROBOT_POWERED` | `ROBOT_POWERED` (explicit) | allowed, subject to all other gates |
| `ROBOT_POWERED` | default / unspecified | **REFUSE** |
| `USB_ONLY` | `ROBOT_POWERED` | **REFUSE** |
| unknown / missing profile | any | **REFUSE** |
| size or digest mismatch | any | **REFUSE** |
| commit mismatch, dirty tree, missing/unparseable manifest | any | **REFUSE** |

The verified profile is printed in a banner immediately before the write, so the operator
sees what is actually going on the device. Logic and refusal reasons live in
`scripts/build_manifest.py`, with offline tests in `scripts/tests/test_build_manifest.py`.

`scripts/upload.sh` is a refusing stub since P2.3: it performs no hardware operation and exits
1. It used to run a full Arduino upload (bootloader + partition table + boot_app0 +
application, every time), which on layout V1 can put the old partition table back or
move/erase `matdog_nvs`. A migration from the legacy layout is a separate, explicitly
authorized procedure. See `VALIDATION.md` Session 2 for why the earlier "application-only"
framing of the full upload was wrong.

## Static safety audit

```bash
python3 scripts/static_audit.py
```

Regression tripwire (not a formal verifier) that fails the build if forbidden
functionality leaks in: `CalibrationOfs`, EEPROM ID/offset writes, automatic
torque-on, `GoalPosition`/motion primitives, automatic BNO085 DCD save, any DALY
write, duplicate GPIO ownership, a UART peripheral collision between ServoBus and
DalyBms, a synchronous multi-ID servo scan loop, `@SERVO SCAN`/`@SERVO READ` losing
their `MAINTENANCE`-mode guard (or `@SERVO SAFE_OFF` gaining one), the LED transport
being driven while `kLedRailPowered` is false, `scripts/flash_app_only.sh` regressing
to reference the bootloader/partition-table/boot_app0 artifacts it must never write,
the OTA partition verifier's fail-closed validity checks regressing (it also runs
that verifier's own offline test suite as part of the audit), the OTA subtype filter
regressing to a numeric threshold instead of the real bitmask, an OTA slot set losing
its contiguous-starting-at-0 requirement, the sdkconfig parser losing its `UNKNOWN`
case (silently treating an absent symbol as `DISABLED` again), the rollback/anti-rollback
parameters gaining an unsafe default, `ServoBus::begin()` assigning `IOTimeOut` directly
again instead of via `ScopedIOTimeout`, `safeOff()`/`readRuntimeState()` using the
diagnostic timeout instead of the operational one, or `safeOff()` classifying success
from `EnableTorque()`'s own return value instead of an independent readback.

G2 additions: the three rail flags regressing from derived values back to independently
editable literals, `expectationsFor()` mismapping a profile, **the compiled-in default
profile being anything other than `USB_ONLY`** (the G3 authorization gate — a
`ROBOT_POWERED` image must never be producible by an unreviewed edit), the servo
population model losing the canonical-17 / expected-now-13 / absent-by-design-4
distinction or drifting from `MATDOG_SERVO_ALLOCATION.yaml`, a "17 responders = PASS"
threshold reappearing, `ServoPopulation`/`ServoCensus` gaining a `Serial` or `<Arduino.h>`
dependency, `Controller::begin()` starting a scan/census at boot, and a direct
network-handler → servo-primitive path. It also compiles and runs the offline C++ census
suite (`scripts/tests/run_host_tests.sh`) as part of the audit, the same way it already
runs the OTA parser's Python suite.

DALY KEY probe additions (2026-09-19): the DALY firmware may transmit **only** the two
whitelisted FC03 read frames (`D2 03 00 00 00 3E D7 B9`, `81 03 01 00 00 78 5B D4`), each
re-verified byte-for-byte with function `0x03` and CRC-16/MODBUS, through exactly one
`bms_uart_.write(dalyRequestFrame(request), kDalyRequestLen)`; any other initialized byte array,
`bms_uart_` use, UART2 route, FC06/FC10 literal or write helper fails. `requestDischargeOff()`
stays a no-op; `@BMS` commands take no arguments and name no write; `@BMS KEY READ` keeps its
`MAINTENANCE` gate and `@BMS KEY STATUS` never starts a transaction; the KEY probe may not leak
into power-state/health logic; `DalyProtocol.*` stays Arduino-free. The audit also runs
`scripts/tests/test_static_audit_daly.py`, which proves those rules fail on mutation (e.g. the KEY
request function `0x03 → 0x06`). The G3.1 TX-ring floor rises from 2560 to 3072 bytes because the
largest single loop pass grew (now 2674 bytes; see `VALIDATION.md`).

DALY KEY discharge configuration additions: the one permitted write is spelled out in
`DALY_THE_ONE_WRITE` (FC06, `0x81`, `0x0120 := 0x005A`). The parameterless
`dalyKeyLogicDischargeWriteFrame()` must match the reviewed recipe exactly; the selector must map
each request to its own frame; the write can be requested only through `@BMS KEY SET DISCHARGE
CONFIRM` (MAINTENANCE-gated, live operating mode) → `DalyBms::requestKeyLogicDischarge(mode)` →
the scheduler, once each. FC10, any other register (MOS control `0x0121`/`0x0122` included), value,
address or second write frame, a caller-supplied target, a generic `@BMS` write spelling, DALY
persistence and any `requestDischargeOff()` body fail the build; the mutation suite proves it.

## USB diagnostic command surface

```text
@HELP
@STATUS
@IMU STATUS | @IMU STREAM ON|OFF
@BMS STATUS | @BMS STREAM ON|OFF
@BMS KEY READ                               (MAINTENANCE mode only; read-only)
@BMS KEY STATUS                             (cached; no bus transaction)
@BMS KEY SET DISCHARGE CONFIRM              (MAINTENANCE only; the ONE DALY write; once per boot)
@BMS KEY WRITE STATUS                       (cached; no bus transaction)
@LED STATUS | @LED OFF | @LED TEST
@WIFI STATUS                                (cached snapshot; never queries the radio)
@WIFI ON | @WIFI OFF                        (any mode; refused without credentials)
@OTA STATUS                                 (read-only; OTA-A ships no transport)
@AUTHORITY STATUS                           (read-only)
@ACTUATOR STATUS                            (read-only)
@CALIBRATION STATUS                         (read-only)
@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE          (MAINTENANCE + ROBOT_POWERED; read-only, Torque OFF)
@CALIBRATION Q0 STATUS | Q0 ABORT
@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION  (MAINTENANCE; RAM only; promotes THAT capture)
@CALIBRATION SESSION START <LF|RF|RH|LH> CONFIRM_CURRENT_Q0
@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION | MOTION PERMIT REVOKE | MOTION ABORT
@CALIBRATION FULL LEG <LF|RF|RH|LH> CONFIRM_FULL_CALIBRATION | FULL LEG STATUS | FULL LEG ABORT
@CALIBRATION EVIDENCE EXPORT                (read-only, deterministic)
@CALIBRATION SESSION ABORT
@SERVO SCAN <lo> <hi> | @SERVO READ <id>   (MAINTENANCE mode only)
@SERVO CENSUS                               (MAINTENANCE mode only)
@SERVO SAFE_OFF <id>                        (always allowed, any mode)
@MODE STATUS | @MODE MAINTENANCE | @MODE RUN
@SYSTEM SHUTDOWN
```

USB CDC transmit can never stall the Controller loop (G3.1): `Controller::begin()` sets the
HWCDC TX timeout to 0 and the TX ring to 3 KB before `Serial.begin()`, so output nobody
drains is dropped instead of waited for. Command replies are expected complete while a host
is reading and draining; right after reopening a port that was closed for a long time, a
stale backlog can still occupy the ring and a reply may short-write rather than block. That
best-effort delivery belongs to this diagnostic surface only; a future HostLink defines its own framing, acknowledgement and reliability.

`@SERVO SAFE_OFF` can only disable torque, never enable it, and stays reachable in
every operating mode — it is the safety de-escalation path. Its reply is never a bare
`OK`: `SCS::Ack()` (which `EnableTorque()` returns) gives `0` on any failure/timeout,
not `-1` like `Ping()`/`readByte()` — a naive `result >= 0` check is therefore always
true and can never observe a failure. `safeOff()` now always performs an independent,
read-only `TorqueEnable` readback after the write and classifies strictly from that:
`VERIFIED_OFF` (readback confirms `0`), `UNVERIFIED_NO_RESPONSE` (no readback response
at all — confirmed live with the servo bus unpowered), or `VERIFY_FAILED` (readback
responded but is non-zero). `@SERVO SCAN`/`@SERVO READ`
are **incremental with bounded per-ID blocking, not non-blocking**: `ServoBus::update()`
still calls `SCServo::Ping()` synchronously, which carries its own bounded per-call
timeout (`kDiagnosticTimeoutMs`, 20ms — see Operating mode below). `@SERVO SCAN` replies
`SERVO_SCAN=STARTED` immediately and the router reports `SERVO_SCAN=COMPLETE
elapsed_ms=.. max_ping_us=..` asynchronously once the scan's one-`Ping()`-per-tick state
machine finishes; that duration is measured, not assumed. Because a scan/read call can
still block for a bounded-but-real amount of time, both commands refuse outside
`MAINTENANCE` mode (`SERVO_SCAN=BLOCKED` / `REASON=NOT_IN_MAINTENANCE_MODE`) — see
Operating mode below. `@LED TEST`/`@LED OFF` are refused/no-op under the current
`USB_ONLY` profile — see Anti-back-power below. `@SYSTEM SHUTDOWN` walks the
power-state machine through its shutdown sequence but always resolves to
`POWER_CUT_FAILED` in V0.1, because the DALY K-Series `Discharge MOS OFF` write
protocol has not been identified or bench-verified (see `VALIDATION.md` and handoff
section 8A.8). No command in this surface can write servo EEPROM, recode an ID, save
the BNO085 DCD, or write DALY MOS state; the only DALY configuration write is the single
semantic KEY setting below.

`@BMS KEY READ` is the read-only DALY KEY probe (implemented and **live-verified read-only**
2026-09-19; see `VALIDATION.md`). It queues one FC03 read of the DALY's second Modbus personality —
`81 03 01 00 00 78 5B D4`, byte-identical to the parameter-block read in DALY's official BMSTool
V1.14.79 — replies `BMS_KEY_READ=STARTED` (or `BUSY` while one is outstanding, `BLOCKED` outside
`MAINTENANCE`), and reports asynchronously — the block below is the **pre-commissioning** live
example captured on 2026-09-19, *before* the single configuration write:

```text
BMS_KEY_READ=COMPLETE result=OK
  key_logic_raw=0x0055 key_logic=DISABLED
  charge_mos_control=1
  discharge_mos_control=1
  sleep_time_raw=360 sleep_time_s=3600
```

or `BMS_KEY_READ=COMPLETE result=TIMEOUT|CRC_FAIL|BAD_HEADER rx_bytes=<n>`. The reply must be
exactly 245 bytes, `51 03 F0`, CRC-valid.

After the single verified commissioning write, the **current last verified value is `0x005A`
DISCHARGE**, so a new read on this unit should normally report `0x005A` unless the BMS was reset,
replaced or reconfigured.

Decoded registers (BMSTool static analysis, read side live-verified on MATDOG's unit): `0x0120`
KEY logic (`0x55` DISABLED, `0xA5` DISCHARGE_AND_SLEEP, `0x5A`
DISCHARGE, `0xAA` CHARGE_AND_DISCHARGE, `0xA6` CHARGE_DISCHARGE_AND_SLEEP, anything else
UNKNOWN), `0x0121`/`0x0122` charge/discharge MOS control, `0x0115` sleep time (raw × 10 s as
BMSTool displays it). One transaction owner (`DalyBusScheduler`) starts both this read and
telemetry, so they never overlap; the KEY read goes first at the next idle boundary, defers at
most one 2 s telemetry poll, and telemetry resumes on its own. A failed read never replaces the
last valid snapshot and never marks the BMS absent. `@BMS KEY STATUS` prints the cached
result/snapshot and its age with zero bus traffic (`snapshot=NOT_READ key_logic=UNKNOWN` until a
read succeeds). Nothing is inferred from the live MOS state.

**Why the commissioning write is retained** (reviewed 2026-09-20). With the register now reading
`0x005A`, the gate answers `ALREADY_CONFIGURED` and transmits nothing, so the command is inert on
this BMS: it can only act again on a unit that reads exactly `0x0055` — a replaced or
factory-reset BMS, which is precisely the re-commissioning case. Removing it would delete the only
Linux-side recovery path (DALY's own BMSTool is Windows-only) while removing no capability that is
currently reachable, so it stays as tightly gated maintenance functionality. It must never be
broadened: no second register, value, argument or alias.

`@BMS KEY SET DISCHARGE CONFIRM` is the **only** DALY write (implemented and sent live **once**,
2026-09-19: acknowledged and read back as `0x005A` — see `VALIDATION.md`). It
sends FC06 `81 06 01 20 00 5A 16 07`: KEY logic `0x0120 := 0x005A` (DISCHARGE — KEY OFF turns the
discharge MOS off, charge MOS kept), the exact frame DALY BMSTool V1.14.79 builds for that setting.
It takes no argument and refuses with zero bytes sent (`BMS_KEY_WRITE=REFUSED reason=…`) unless:
MAINTENANCE; no write already sent this boot; no DALY transaction in flight or queued; `0xD2`
telemetry OK and ≤ 5 s old; no alarms; a successful `@BMS KEY READ` ≤ 30 s old showing exactly
`0x0055` with charge/discharge MOS control `1`/`1` (`0x005A` → `ALREADY_CONFIGURED`). Accepted, it
replies `BMS_KEY_WRITE=STARTED target=DISCHARGE raw=0x005A`, re-checks every precondition (the
live operating mode included) just before
transmitting, then reports `BMS_KEY_WRITE=ACK result=OK|TIMEOUT|CRC_FAIL|BAD_HEADER|BAD_ECHO
rx_bytes=<n>` (only the exact echo `51 06 01 20 00 5A 05 97` is `OK`) and, after an automatic
FC03 read-back, `BMS_KEY_WRITE=COMPLETE ack=… readback=VERIFIED|PENDING_RESTART|MISMATCH|READ_FAILED`
with the decoded register. Only `VERIFIED` (`0x005A` read back) means the setting is stored;
`PENDING_RESTART` means the BMS still reports `0x0055` (BMSTool asks for a BMS restart after every
setting — MATDOG never restarts the BMS). `@BMS KEY WRITE STATUS` prints the cached record with
zero bus traffic. Nothing persists on the ESP32: if it loses power after the write, `@BMS KEY
READ` after reboot shows the real register. No rollback command exists.

`@STATUS` (and the per-module `@IMU`/`@BMS`/`@LED` variants) report each module as
`init=.. detected=.. expected=.. result=..` — separating "did the driver initialize"
from "was the hardware actually detected" from "was it expected to be reachable right
now" from "is that a problem". See `src/core/Availability.h`; under `USB_ONLY`:

```text
BNO085 init=OK       detected=ONLINE      expected=REQUIRED  result=PASS
DALY   init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
SERVO  init=OK       detected=UNKNOWN     expected=OFFLINE   result=PASS
LED    init=DEFERRED detected=UNPOWERED   expected=UNPOWERED result=PASS
SERVO_POP canonical=17 expected_now=13 absent_by_design=4 last_census=NOT_RUN
```

`last_census=NOT_RUN` is the honest answer after a boot with no census: no servo bus
transaction ever happens automatically, so `@STATUS` must never imply a population was
verified.

### "Not observed" is not a verdict

`detected=UNKNOWN` means *nothing has established anything* — it is the absence of
evidence, not evidence of absence. `detected=NO_RESPONSE` means *we asked and it did not
answer*. `classify()` keeps these distinct:

```text
UNKNOWN     + REQUIRED            -> UNKNOWN   (not proven; system reports BOOTING)
UNKNOWN     + OPTIONAL            -> PASS      (absence would not even be a fault)
UNKNOWN     + OFFLINE/UNPOWERED   -> PASS
NO_RESPONSE + REQUIRED            -> FAULT     (a real observed failure)
NO_RESPONSE + OPTIONAL            -> DEGRADED
NO_RESPONSE + OFFLINE/UNPOWERED   -> PASS
```

This matters for two modules under `ROBOT_POWERED`. A WS2812 chain has no readback path
at all, so the LED ring's `detected` is permanently `UNKNOWN` when powered; treating that
as `DEGRADED` made `SystemHealth::READY` unreachable on a perfectly healthy robot.
Separately, nothing probes the servo bus at boot (no automatic scan is allowed), so
`REQUIRED + UNKNOWN` reported `FAULT` before anyone had asked the bus a single question.

Neither is fixed by faking a physical observation: the LED still reports
`detected=UNKNOWN` in `@STATUS`, and `detectedStateForLedRail()` is forbidden by the
static audit from ever returning `ONLINE`. An *observed* failure still escalates exactly
as before. Under `ROBOT_POWERED` the system therefore reads `BOOTING` until the servo bus
is actually probed, `READY` once it answers, and `FAULT` if it is probed and does not.

## Hardware profile (USB_ONLY / ROBOT_POWERED)

`src/config/HardwareProfile.h` is the **single authority** for which physical power
configuration the firmware is built for. V0.1 stated the bench configuration four times
over — a `kTestProfile` string plus three independent `constexpr bool` rail literals —
with nothing tying them together, so a partial edit could produce a firmware whose
printed profile name contradicted its own expectations. Now there is one enum, one
mapping table, and everything else is derived:

```text
USB_ONLY       servo_power=NO  battery=NO  led_rail=NO
ROBOT_POWERED  servo_power=YES battery=YES led_rail=YES
```

`BuildConfig.h` selects exactly one profile and derives `kServoPowerAvailable`,
`kBatteryAvailable`, `kLedRailPowered` and `kTestProfile` from it. Each module's
`ExpectedState` then comes from one shared derivation in `core/Availability.h`
(`expectedStateForServoBus`/`Battery`/`LedRail`) instead of an inlined ternary repeated
per module. The V0.1 `Availability` model itself is unchanged: the same `classify()`
turns an unchanged `NO_RESPONSE` into `PASS` under `USB_ONLY` and `FAULT` under
`ROBOT_POWERED`, which is exactly what it was designed for. The LED ring is deliberately
`OPTIONAL` even when powered — a dead status ring must never fault an otherwise healthy
robot.

Switching profiles is a **one-symbol change**:

```bash
MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh    # one build, source default untouched
```

The source default (`MATDOG_ACTIVE_HARDWARE_PROFILE` in `BuildConfig.h`) is `USB_ONLY`
and `static_audit.py` fails the build if it is anything else, because powered hardware
validation (G3) is a separately authorized gate. The override is deliberately loud:
`build.sh` prints the selected profile and the boot banner reports it with its rail
facts, so a `ROBOT_POWERED` image cannot be produced or flashed silently.

A profile describes the **power/rail** configuration only. Which servos are physically
installed is an orthogonal fact, owned separately — see below.

## Servo population and census

`src/servo/ServoPopulation.h` keeps three populations explicitly distinct:

```text
CANONICAL_ALLOCATED               17   every bus ID the MATDOG design allocates
EXPECTED_IN_CURRENT_CONFIGURATION 13   physically installed today
ABSENT_BY_DESIGN                   4   52 NECK_PITCH, 53 HEAD_ROTATION,
                                       54 HEAD_PITCH, 55 JAW — allocated and
                                       bench-provisioned, not yet mounted
```

A healthy powered census for the current robot is therefore **13 present + 4 absent by
design**, not 17. "17 must respond for PASS" is false for this robot and the static
audit fails the build if such a threshold reappears.

`@SERVO CENSUS` scans the canonical range 11–55 and classifies every ID:

```text
PRESENT_EXPECTED          installed-now servo answered              healthy
ABSENT_BY_DESIGN          not-installed servo stayed silent         healthy
MISSING_EXPECTED          installed-now servo did NOT answer        problem
ABSENT_BY_DESIGN_PRESENT  not-installed servo ANSWERED              problem, surfaced
UNEXPECTED_ID             responder outside the canonical table     problem
NOT_PROBED                canonical ID outside the scanned range    inconclusive
```

Verdict is `PASS`, `PROFILE_MISMATCH`, `RANGE_INCOMPLETE` or `NOT_RUN`. It is
fail-closed: a scan that did not cover every canonical ID, or whose responder list
overflowed, can never report `PASS` — silence only means something where a probe
actually happened.

The embedded canonical table is a deliberately minimal projection of
`06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml`, which remains the
canonical **project** authority (unit identity, provisioning sessions, cold-verify
evidence). The static audit cross-checks the two so drift cannot pass silently; if they
ever disagree, the YAML wins.

**Architecture note.** `ServoBus` answers "what did the physical bus observe";
`ServoPopulation` answers "what does that mean for this robot"; `ServoCensus` is the
Controller-owned service that runs one and holds the structured result; `CommandRouter`
only *formats* it. Nothing in the population/census layer includes `<Arduino.h>` or
touches `Serial`, so the offline host tests link the shipped logic rather than a copy,
and a future Web UI / HostLink adapter can render the same `CensusResult` without
re-scanning the bus or reimplementing the classification.

The census is strictly read-only (`Ping()` only) and is **never started automatically** —
not at boot, not on a timer.

## Offline tests

```bash
python3 scripts/tests/test_ota_partition_logic.py   # OTA slot selection (40 tests)
bash scripts/tests/run_host_tests.sh                # servo population / profile + DALY protocol
                                                    # + Wi-Fi + OTA-A + ActuatorAuthority
                                                    # + safe actuator write policy
                                                    # + calibration bootstrap geometry
                                                    # + calibration domain & manager
python3 scripts/tests/test_static_audit_daly.py     # DALY write-whitelist mutation suite
python3 scripts/tests/test_static_audit_safe_actuator.py  # safe actuator boundary mutation suite
python3 scripts/static_audit.py                     # runs all of the above, plus the audit
```

`scripts/tests/test_servo_population.cpp` compiles the **real** firmware translation
units (`ServoPopulation.cpp`, `Availability.cpp`) on the host with `g++ -Wall -Wextra
-Werror` — no Arduino runtime, no device, no test framework. It covers the canonical/
expected-now distinction, every per-ID classification, missing/unexpected/
absent-but-present cases, the fail-closed partial-scan and truncation paths, and both
profiles' expected-hardware semantics (proving `USB_ONLY` behaviour did not regress
while `ROBOT_POWERED` was added).

`scripts/tests/test_daly_protocol.cpp` does the same for `src/power/DalyProtocol.cpp`: both
request frames and their CRCs, the 245-byte `0x81` reply built from BMSTool's literal byte
positions (proving the register-based offsets), all KEY-logic values, sleep-time width, rejection
of bad header/CRC/length without replacing the last valid snapshot, the unchanged `0xD2`
telemetry decoder, and the bus scheduler (no overlap, KEY priority, bounded telemetry deferral,
automatic resumption). It also covers the one write: its exact bytes and CRC, the acknowledgement
exactly as BMSTool accepts it (address, function, CRC, register and value echo, length), every
precondition and its order, `ALREADY_CONFIGURED`, read-back classification (an ACK alone is never
verification), the status tracker, and scheduling of write → read-back → telemetry after success,
failure and timeout.

`scripts/tests/test_wifi_policy.cpp` does the same for `src/network/WifiPolicy.cpp`: the
credential gate (no SSID configured means the radio is never started, under any number of
ticks), the two-phase radio start, the one-transition-per-tick rule, the connect deadline,
the doubling backoff ladder and its 60 s ceiling, reset-on-success, link loss restarting the
ladder from the bottom, operator enable/disable (including that no teardown is issued for a
radio that was never brought up), fail-closed handling of a refused radio start or connect
call, the invariant that a connect is never issued while connected, `millis()` wraparound,
and the IPv4 formatter including its bounds.

`scripts/tests/test_ota_policy.cpp` does the same for `src/update/OtaPolicy.cpp`,
`OtaBootGuard.cpp` and `Sha256.cpp`, with a fake `OtaBackend` substituting only the flash.
468 checks: target resolution refusals (target == running, factory/TEST subtype, no inactive
slot, oversize, zero length, and exactly-partition-sized which must be *allowed*), metadata
refusals, the PENDING_VERIFY precondition, every stream failure (open rejected, write error,
short write, truncation, overrun, `esp_ota_end` failure, hash mismatch against a different
image of the same length), the ordering property that the boot target does not move from any
state other than `IDENTITY_VERIFIED`, replay/idempotence, and the whole first-boot rollback
lifecycle. SHA-256 is checked against the FIPS 180-4 vectors and against itself at eight
chunk sizes.

`scripts/tests/test_actuator_authority.cpp` does the same for
`src/core/ActuatorAuthority.cpp`. 751 checks, with the conflict matrix exhaustive rather than
illustrative: all 20 ordered pairs of distinct write-capable owners, each challenger tried in
**its own** legal operating mode so a rejection can only be about exclusivity. Also: boot and
re-init always landing on `NONE`, corrupted enum values failing closed, release refused for every
non-owner, the stale-lease case the generation exists for, force-clear under every reason, the
mode-compatibility table, a mode change clearing a stranded owner, and the whole inhibit
lifecycle.

`scripts/tests/test_calibration_domain.cpp` and `test_calibration_manager.cpp` cover
`src/calibration/`: 702 domain checks including the 24-profile completeness derived from the
Cartesian model, the three meanings of 2048 pinned apart, the physical-unit identity trap
(unit M11 versus M33 in the LF lower slot), the leg population gate, the evidence lifecycle with
every shortcut refused, and the LF V25 oracle replay; plus 322 manager checks against the **real**
`ActuatorAuthority`, including the end-to-end stale-lease scenario where a finished session's late
events are fired at a live one and must not touch it.

## Calibration on `main` (2026-09-29)

> **HISTORICAL / SUPERSEDED status snapshot.** Current 24/24 hardware result is above.

**Status: IMPLEMENTED / OFFLINE TESTED. The four-leg Full Calibration has NOT been run on
hardware.** The sections *Calibration foundation* and *Safe Actuator Layer* below are the
historical record of how the foundation was built; where they say "no write path" or "runtime
adapter TO_IMPLEMENT", those statements are historical, not current status.

- **Write path.** A bounded production `ServoBusActuatorBackend` exists and is reachable only
  through the Safe Actuator layer: `ActuatorAuthority` lease, a live calibration session, a fresh
  RAM motion permit and Geometry V5 targets. Nothing writes servo EEPROM, `PositionOffset`, an
  ID or NVS in this path; every calibration record is RAM.
- **q0.** `@CALIBRATION Q0 CAPTURE` produces twelve `CANDIDATE` q0 records read-only (Torque
  OFF, nine samples per joint). `@CALIBRATION Q0 PROMOTE` promotes **that same capture** —
  `prepareFreshQ0Evidence` over the capture session's `FreshQ0Capture` view — through the one
  shared CR3 accept-and-promote rule, all twelve or none, and **replaces** the previous q0 in
  `JointTransformTable`. The frozen CR2-C package (`prepareCurrentQ0Evidence`) is kept only as a
  regression oracle; production code and the static audit forbid using it. `SESSION START`
  refuses (`CURRENT_BOOT_Q0_NOT_PROMOTED`) unless the table holds exactly the current capture.
- **Full Leg Calibration.** Generalized to LF, RF, RH, LH: UPPER MIN contact, an optional
  auxiliary park, UPPER MAX contact, `SAFE_OFF`, then the evidence lifecycle. Parking: LF→
  `LH_UPPER` (bus 42), RF→`RH_UPPER` (bus 32), RH and LH none. Evidence export is deterministic.
- **Acceptance levels.** `HARDWARE_CONTACT_CALIBRATED` is the next hardware acceptance level.
  `FINAL_OPERATIONAL_ENVELOPE_ACCEPTED` is unavailable until an approved stand/gait workspace
  sets `kFullLegOperationalParametersApproved` (audit-pinned `false`); 0/12 final
  `JointLimits` are admitted on purpose.
- **Persistence.** Calibration Persistence V1 is on `main` since PR-1 (2026-10-08).
  - Storage: record and codec, A/B store in the dedicated `matdog_nvs` partition, persistent
    SAVE recovery marker, durable acknowledgment.
  - Commands: `@CALIBRATION PERSIST STATUS | SAVE CHECK | SAVE CONFIRM_SAVE_FULL_CALIBRATION |
    ACK | RECONCILE`.
  - Boot LOAD is information only: nothing is admitted into the transform table, and `RESTORE`
    is not implemented.
  - Offline/host-tested on `main`, with no hardware acceptance of a `main` build
    ([`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md)).
- **Hardware procedure:** [`FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md`](FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md).

## Calibration foundation

**Status (historical, 2026-09-24): implemented, compiled, offline-tested. NOT hardware-tested. NO write path added.**

The installed robot's calibration is `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION` and hardware
motion is **BLOCKED** — that is the repository's own declaration, and
`MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED` defaults to `0` so a live session is refused
before the arbiter is even asked.

The LF V25 archive is a **historical hardware oracle**, replayed offline and matched. It is not
current calibration and cannot become it: every replayed record carries `HISTORICAL_REPLAY`, and
`mayPromote()` refuses that origin.

The full audit — source precedence, what LF V25 actually proved, the three evidence vocabularies,
four discrepancies including the two unrelated meanings of "H1", the Generic V25 component
assessment and the EEPROM boundary — is in
[`CALIBRATION_SOURCE_PRECEDENCE.md`](CALIBRATION_SOURCE_PRECEDENCE.md).

## Operating mode (MAINTENANCE / RUN)

`core::OperatingMode` (`src/core/OperatingMode.h`) is a deliberately tiny boundary —
an enum and a two-line manager class, not a state machine — between potentially
blocking servo diagnostics and a future deterministic motion loop:

```text
MAINTENANCE  servo scan/read diagnostics allowed; no motion allowed (none exists yet)
RUN          future motion loop; @SERVO SCAN/@SERVO READ refuse
```

V0.1 has no motion execution at all, so today this boundary gates diagnostics against
nothing but itself. The point is that it already exists, so a future motion-controller
integration cannot silently inherit `@SERVO SCAN`/`@SERVO READ` as callable from
inside a real-time `RUN` loop. Default is `MAINTENANCE` — there is no motion loop yet
to protect, and Session 2's already hardware-validated diagnostic workflow should not
regress for no protective benefit. **Whoever adds the motion controller must flip the
default to `RUN`** and require an explicit, reviewed transition into `MAINTENANCE`
(with torque confirmed off) before servo diagnostics are reachable again — this is
called out directly in `OperatingMode.h` so it cannot be missed.

The servo bus has two explicitly named timeouts, and every method states which one it
uses — there is no "default/untouched" case to trust implicitly (Session 2.3 Finding 1
corrected Session 2.2, where a single 20ms timeout was silently applied to every
transaction including `safeOff()`/`readRuntimeState()`, making the "100ms
operational / 20ms diagnostic" split not actually true in code):

```text
ServoBus::kDiagnosticTimeoutMs = 20   ping(), readModel(), the scan's per-ID probe —
                                       bounded absence-detection only, MAINTENANCE-only today
ServoBus::kOperationalTimeoutMs = 100 safeOff() (the safety de-escalation write, reachable
                                       from any OperatingMode) and readRuntimeState() (the
                                       primitive behind @SERVO READ today, which a future
                                       motion controller will naturally reuse)
```

`kDiagnosticTimeoutMs` is justified by real hardware measurement rather than an assumed
value: the NEW01 characterization campaign
(`09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/characterization_sessions/`)
timed a live powered ST3215 at 1 Mbaud and recorded Ping+register-read round trips of
593-620us — 20ms is roughly 32x that measured worst case. `kOperationalTimeoutMs` is
simply SCServo's own untouched library default (100ms) — not a value MATDOG has
validated/optimized as final for the powered multi-servo bus (13 installed today, with 17
canonical allocation slots). Both are applied via
`ServoBus::ScopedIOTimeout`, a generic RAII guard around each individual bus transaction
that saves `SCSerial::IOTimeOut` (a public field, set at runtime — the vendored library is
never edited), applies the named timeout for that call site, and restores the previous
value on every exit path. `begin()` never assigns `IOTimeOut` directly, so neither timeout
can silently become a standing global override for some other call site.

## Wi-Fi / provisioning V3 — 2026-10-04

**IMPLEMENTED / OFFLINE TESTED; on `main` since PR-2; hardware acceptance BLOCKED.** No release
is approved.

> *Historical wording, 2026-10-04:* "This isolated branch is based on
> `1a5e0085098eeb907319f6f1a00693869444d9cf`, the latest local post-abort/persistence integration.
> Its calibration persistence hardware acceptance is not evidenced: **CAL_PERSIST_BASE is
> BLOCKED**. This branch is a provisional candidate, not a release." The branch no longer exists
> as a separate line; its content reached `main` through PR-2.

A core-0, low-priority worker owns radio and network NVS. `WifiManager::update()` copies
cached state with a zero-wait mutex; events only store a disconnect reason. One request
slot reserves `config_busy` before returning. Calibration mutations, persistence writes,
RUN entry and OTA preparation cannot overlap that reservation. Critical-state changes
cancel a trial, stop scans and keep modem sleep OFF. Network code cannot command servos.

Two infrastructure profiles share one STA; ACTIVE NVS takes precedence over the old
compile-time credentials. The radio stays ON with no credentials. Retry/backoff is
2–60 s; profiles alternate after failed attempts. Driver association scans all channels,
sorts by signal and never pins a BSSID. Optional roaming defaults OFF, with -80 dBm
threshold, 60 s scans, 8 dB hysteresis and 120 s dwell; scans are asynchronous, limited
to twelve displayed entries and inhibited during critical activity. HT20 is default;
HT40 is configurable and separately reports requested/effective state and application
success. RF performance, reconnect, roaming and scheduling measurements remain TO_TEST.

HWCDC in installed Arduino ESP32 3.3.11 cannot establish a trusted service-session
opening from `isConnected()`, DTR or SOF activity. AUTO/ON preferences therefore both
resolve to **NO_SLEEP**, just like OFF. The pure policy supports trusted session facts,
but the current adapter never manufactures them. AP/provisioning and OTA force OFF.

### Recovery AP and offline interface

Protected AP name is `MATDOG-<last six STA MAC hex digits>` (e.g. `MATDOG-227594`),
address `192.168.4.1/24`. There is no default password or admin token. Before first use,
a separately authorized physical USB session must provision a unique WPA2 passphrase
(8–63 characters) and a random 64-hex admin token; the token is stored as SHA-256 only.
No credentials were provisioned during this development task. Without an AP key no AP
is opened. AP appears on demand, without STA credentials, after repeated failures or
under the permanent policy. Timeout is 15 min, and only expires with working STA,
no clients and no transaction. Permanent AP must be disabled in configuration before
an OFF request can take effect. Shared AP/STA radio can change AP channel on association.

The embedded HTML has Dashboard, Wi-Fi, Access Point, Performance, OTA and Diagnostics;
it needs no CDN, Internet, asset server or actuator action. GET telemetry is sanitized;
SSID/BSSID/channel/RSSI, profiles, scans, config phase, heap, stack and timing are visible.
Password fields are write-only. Blank retains the existing password for the same SSID;
a changed SSID requires an explicit password. Open-network enrollment is not exposed
by this first portal. AP changes require a working STA and apply after the last AP
client disconnects, with `ap_reload_pending` reported until then.

HTTP port 80 serves the local portal. Writes require the actual AP socket, Host and
Origin `http://192.168.4.1`, admin login, random HttpOnly/SameSite cookie, CSRF token,
peer binding and 5-minute session lifetime. Login is throttled. Forms have a 1536-byte
bound and reject duplicate/unknown fields, malformed encodings and invalid addresses.
AP HTTP relies on the private WPA2 link; other holders of that AP passphrase share its
trust boundary. It is not a LAN-wide cleartext administration interface.

### Network configuration ownership

`NetworkConfigNvs` uses standard partition **nvs**, namespace **md_net_v1**, keys
**active/pending**. The fixed 512-byte record includes schema, magic and CRC32.
No initialization, erase, reset or migration occurs on corrupt/unsupported records;
writes lock on storage faults. `matdog_nvs` and calibration A/B/save markers remain
owned exclusively by the existing calibration backend. No diagnostic PHY erase was imported.

Web changes require a protected alternate AP path: CONFIG_PENDING → TESTING → COMMITTED
only after a fresh driver association to the chosen SSID and a valid IP. Old association
or stale IP cannot confirm a changed password. Timeout/critical state gives ROLLBACK to
known-good ACTIVE. PENDING is ignored at boot. The test confirms the chosen profile;
it does not claim that an untested secondary profile works. NVS flash operations are
on the worker, but flash cache stalls can still affect both cores: measure on hardware.

Read-only commands: `@WIFI STATUS`, `@WIFI PROFILES STATUS`, `@WIFI AP STATUS`,
`@WIFI SLEEP STATUS`, `@WIFI ROAM STATUS`, `@WIFI MODE STATUS`.
Authorized software intents: `@WIFI SCAN`, `@WIFI AP ON|OFF`, `@WIFI SLEEP AUTO|ON|OFF`,
`@WIFI ON|OFF` (OFF is temporary, 10 min), and physical USB provisioning
`@WIFI AP KEY <unique-passphrase>` / `@WIFI ADMIN KEY <random-64-hex-token>`.
QUEUED acknowledges an intent; inspect phase/status for completion. Malformed Wi-Fi
commands are redacted. Never paste real secrets into a recorded terminal/history.

See [V3 report](../../09_Logs/Validation_Reports/2026-10-04_WIFI_OTA_SHELLY_V3_OFFLINE.md)
and [hardware runbook](WIFI_OTA_SHELLY_HARDWARE_RUNBOOK.md).

## ActuatorAuthority

**Status: implemented, compiled, offline-tested. NOT hardware-tested.**

The single central arbiter of the exclusive right to **write** actuators. One instance exists,
owned by `Controller`; every other component holds a pointer and asks. `scripts/static_audit.py`
fails the build if a second instance appears or if any component caches the value.

### It is not a second OperatingMode

They are orthogonal axes and the distinction is load-bearing:

```text
OperatingMode       what the Controller as a whole is doing
                    MAINTENANCE: blocking diagnostics are safe
                    RUN:         a deterministic motion loop may be active

ActuatorAuthority   WHO, if anyone, currently holds the exclusive right to
                    issue actuator writes
```

A controller sits in `MAINTENANCE` with authority `NONE` indefinitely — that is the normal state
today, because **no write-capable owner exists yet**. The mode says what is safe; the authority
says who is doing it.

The compatibility table is enforced, not decorative:

| OperatingMode | may host |
|---|---|
| `MAINTENANCE` | `NONE`, `DIAGNOSTICS`, `CALIBRATION`, `QC`, `PROVISIONING` |
| `RUN` | `NONE`, `MOTION` |

A mode change that leaves the current owner incompatible **clears** it rather than leaving a
suspended authority. Today nothing can ever hold one, so this is a no-op — it exists so the first
real owner does not have to remember to add it. The table is the initial one and should be
re-reviewed when the motion loop lands and the `RUN` default flips.

### What it deliberately does not arbitrate

**Reads.** `@SERVO SCAN`, `@SERVO CENSUS` and `@SERVO READ` are `Ping`/`readByte`/`readWord`. They
are `MAINTENANCE`-gated because they **block**, not because they write, and they take no
authority. The arbiter exists to prevent write conflicts, not to serialize every read.

**`SAFE_OFF`.** A safety de-escalation must be reachable in every authority state, including an
inconsistent one. This is structural rather than a promise: `ServoBus` has no reference to the
arbiter and the arbiter has no reference to `ServoBus`, so `safeOff()` **cannot** consult an
authority even if a later edit wanted it to. The audit fails the build if the `@SERVO SAFE_OFF`
branch ever gains an authority or mode condition, or if `ServoBus` ever names one.

### Two semantics worth stating

**Same-owner re-request → `ALREADY_OWNED`, and no second lease.** Returning a second valid lease
would let two holders each believe they own it, and either could then release it out from under
the other. The existing holder keeps the only lease.

**Leases carry a generation.** Matching the owner alone already rejects "`CALIBRATION` releases
while `MOTION` holds". What it cannot catch is the same owner across two sessions:

```text
CALIBRATION acquires   (generation 1)
CALIBRATION releases
CALIBRATION acquires   (generation 2)
late callback from session 1 calls release(CALIBRATION)
    owner matches  ->  session 2 would be cleared out from under itself
```

The generation makes that release provably stale. It is refused and reported.

### No new write path

This phase implements the **arbiter**, not new capabilities. The firmware's only actuator write is
still `EnableTorque(id, 0)` inside `safeOff()`. Nothing can acquire an owner yet, and there is
deliberately **no command** that does — an operator-driven acquire would itself be a new write
path. `@AUTHORITY STATUS` is read-only.

### Exclusivity inhibit — and the TOCTOU problem it solves

Some activities are not actuator users but must exclude all of them. Firmware update is the first.
Adding an OTA entry to the owner enum would have been wrong twice over: OTA drives no actuator, and
it does not *compete* with `CALIBRATION` for a resource — it requires that **nobody** is using one.
The audit fails the build if such an entry is added.

The naive gate is genuinely insufficient:

```cpp
if (authority == NONE) { start OTA }     // not enough
```

Not because of threading — MATDOG's Controller is single-threaded, every subsystem runs
cooperatively inside `Controller::update()`, and nothing in MATDOG's own code owns a task or a
callback, so a check-then-act **inside one call** is already atomic. The hazard is **duration**: an
OTA update spans `prepare` → `openStream` → thousands of `writeChunk` calls → `finishStream` →
`commit`, across seconds of loop passes. Between any two of them a command can arrive and a future
`CalibrationManager` can acquire authority. No amount of re-checking closes that.

So the arbiter exposes a **hold**, not a query:

```text
requestInhibit(FIRMWARE_UPDATE)
    granted only when current() == NONE
    while held, every request() is REJECTED_INHIBITED
```

The "is anyone an owner?" check and the hold happen inside one arbiter call, so there is no window
between them. **No `SystemActivity` layer was needed**, and none was built.

A stuck inhibit is fail-safe: it blocks writes, it does not enable them. `forceClear()` of the
*owner* deliberately does not drop it — a fault must not quietly re-open actuator authority. It is
cleared explicitly, or by the next boot.

## Safe Actuator Layer

**Status (historical, 2026-09-25): policy core implemented, compiled, offline-tested. Runtime adapter TO_IMPLEMENT.
NO write path added.** *(Current: see [Calibration on `main`](#calibration-on-main-2026-09-29) —
bounded calibration was hardware-validated on 2026-10-01; operational motion remains blocked.)*

`src/actuator/ActuatorWritePolicy.*` is the boundary every future actuator write must pass
through. It is a **decision**, not a transport: there is no bus handle in it, so an `ACCEPT`
authorises nothing by itself.

**No "check once then write later".** `plan()` captures the `AuthorityLease`, the
`OperatingMode` and a policy epoch; `commit()` re-reads the live arbiter and compares all of it
again. Authority released and re-acquired, force-cleared, stranded by a mode change or overtaken
by an OTA inhibit each fail closed at commit. At most one transaction is outstanding, which is
what makes replay impossible rather than unlikely — a copy a caller kept is refused on identity
before its contents are read, and `reset()` advances the epoch so anything outstanding can never
match again.

**Limits are provenance first, value second.** A bound is usable only if
`calibration::mayPromote()` and `calibration::isOperationalEvidence()` both accept it and both
identity axes agree — the same test `q0MayBeAppliedTo()` applies. The store is empty and stays
empty: `MATDOG_JOINT_CALIBRATION.yaml` records `{min: null, max: null}` for all twelve leg
joints, so every position-class command resolves to `REJECT_NO_ACCEPTED_LIMITS`. A missing bound
is a refusal, never a fallback to a historical value.

**`SAFE_OFF` is outside this layer structurally.** There is no operation class for *removing*
torque, so a safety de-escalation is inexpressible here and no later edit can make it depend on
an authority check. For the same reason no persistent/provisioning write is expressible while
the repository still records that owner as `TO_DESIGN`.

The runtime adapter is deliberately **not** built yet: it would need a torque-on or
goal-position primitive, and both are prohibited by audits that exist for hardware-era reasons.
The full S0 write-surface map and the design rationale are in
[`SAFE_ACTUATOR_LAYER.md`](SAFE_ACTUATOR_LAYER.md).

### Calibration bootstrap — geometry-authorised moves

`src/actuator/CalibrationGeometryProfile.*` gives the Controller the compiled Geometry
Compiler V5 result as a generated `constexpr` table: **no JSON parser, no heap, no mesh and no
collision maths on the device**. It was produced by
`06_Software/Matdog_Core/calibration/matdog_calibration_geometry_export.py`, a pure reduction
of the canonical bundle that re-verifies every input hash and refuses to emit anything on
drift.

Three authorisation routes, mutually exclusive by construction:

| Operation | Authorised by |
|---|---|
| `POSITION_COMMAND` | accepted joint bounds — **unchanged, not weakened** |
| `DIRECTION_VERIFY` | the symmetric bootstrap envelope **and** the session's approved budget |
| `CALIBRATION_CONTACT_PROBE` | the endpoint plan **and** an accepted raw↔q transform |
| `CALIBRATION_AUXILIARY_MOVE` | the endpoint plan **and** an accepted raw↔q transform |

Three meanings are kept apart: a **geometric contact** is not an **executable target**, a
clearance **PASS** is not a **motion authorization**, and a **diagnostic endpoint** is not a
place the robot may be commanded to. Of 24 canonical endpoints only 8 are executable, and the
8 `UNRESOLVED` clearance verdicts all sit on diagnostic endpoints.

The direction-verify envelope is **symmetric and checked in tick space** — the magnitude of a
tick delta needs neither q0 nor direction, which is what makes the move that measures the sign
safe before the sign is known.

Everything failed closed at the time of writing: no q0 had been captured on the current
installation, so no transform existed and every plan-bound move refused. (Since 2026-09-27 a
read-only q0 capture has run on the robot, and 2026-09-29 promotes the current capture; the
plan-bound moves still refuse until a session, a promoted q0 and a fresh permit exist.) Audit and contract:
[`CALIBRATION_BOOTSTRAP.md`](CALIBRATION_BOOTSTRAP.md).

## OTA-A (update core)

**Status: implemented, compiled, offline-tested. NOT hardware-tested.** No MATDOG device has
received an OTA image. Nothing below is a claim about flash behaviour on real hardware.

OTA-A is the **update core only**. It ships **no transport** and **no authentication**, and
the byte-ingest entry points are compiled out by default. Nothing can feed it an image.

### Layering

```text
network transport                    NOT IMPLEMENTED in OTA-A (substitution point)
      |
      v
update/OtaManager                    Controller-facing owner + first-boot lifecycle
      |
      v
update/OtaPolicy                     the update state machine        (host-linkable)
update/OtaBootGuard                  the first-boot rollback guard   (host-linkable)
update/Sha256                        image identity                  (host-linkable)
      |
      v
OtaBackend (abstract)  ->  update/OtaEspBackend    the ONLY unit calling esp_ota_*
      |
      v
inactive OTA application slot
```

The backend is an interface rather than a direct `esp_ota_*` call so the offline suite drives
the **real** state machine against a fake backend that can fail any individual flash
operation. The logic under test is the shipped logic; only the flash is substituted.

### The one safety rule, made structural

OTA writes the **inactive** slot and only the inactive slot. That is not enforced by a comment:

- `esp_ota_get_next_update_partition(NULL)` is documented never to return the running
  partition — and `OtaPolicy` checks it against the running partition anyway;
- three checks are stated explicitly rather than inferred from a backend refusal:
  `target != running`, `subtype ∈ ota_0..ota_15`, `image_size ≤ target.size`;
- `OtaEspBackend::setBootPartition()` re-reads `esp_ota_get_running_partition()` and refuses
  independently, so the last line of defence does not depend on `OtaPolicy` being correct;
- `commitBootTarget()` is the **only** method that changes the boot target, it has exactly
  one `setBootPartition(` call site, and it is callable from exactly one state.

`scripts/flash_app_only.sh` is **not** reused as the OTA writer. It deliberately writes the
**active** slot over USB — it is a wired service/recovery path with different guarantees, and
its logic was audited for reuse, not adopted.

### State machine

```text
IDLE ──prepare()──► TARGET_RESOLVED ──openStream()──► RECEIVING ──finishStream()──┐
  ▲                        │                             │                        │
  │                        │                             │                   IMAGE_SEALED
  │                        │                             │                        │
  │                        ▼                             ▼                        ▼
  └──── abort()/reset() ──── FAILED ◄──────────────────────────────────  IDENTITY_VERIFIED
                               ▲                                                  │
                    boot target NEVER touched                          commitBootTarget()
                                                                                  │
                                                                                  ▼
                                                                        BOOT_TARGET_SET
```

Each of the distinctions that matter is a separate observable fact: update not started
(`IDLE`), target resolved but no flash touched (`TARGET_RESOLVED`), stream open and writing
(`RECEIVING` + `bytes_written`), stream incomplete (`INCOMPLETE_STREAM`, caught by *our*
byte count before the backend is even asked), `esp_ota_end` outcome (`IMAGE_SEALED` vs
`IMAGE_REJECTED`), image identity (`IDENTITY_VERIFIED` vs `HASH_MISMATCH`), boot target not
yet changed (every state except the last), boot target changed (`BOOT_TARGET_SET`).

### Five kinds of verification, kept distinct

They are not synonyms, and only the third can tell "a valid image" from "the expected image":

| Layer | What it proves | Who does it |
|---|---|---|
| transport integrity | the declared number of bytes arrived | `OtaPolicy` byte accounting |
| image validity | the bytes form a loadable app image | `esp_ota_end()` |
| cryptographic hash identity | the bytes are **the** expected bytes | `Sha256` vs declared digest |
| firmware/build identity | which commit this image came from | `build::kBuildId` in metadata |
| bootloader validity | which slot boots, and its rollback state | otadata / `esp_ota_get_state_partition` |

Metadata reuses the identity scheme the repository already has — `build::kBuildId`, plus the
`APPLICATION_SHA256` and `APPLICATION_SIZE` that `scripts/build_manifest.py` already records.
No second version scheme was invented.

It deliberately does **not** use `esp_app_desc_t`. Parsing the real built binary shows why:

```text
version      = 'ee57070'              <- the arduino-lib-builder commit
project_name = 'arduino-lib-builder'  <- not MATDOG
date/time    = 'Jul 20 2026'          <- when the prebuilt libs were built
```

Those fields describe the core, not this firmware, so they cannot answer "is this the
firmware I expected?".

### First-boot validation and rollback

The real build has `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (read from the generated
`sdkconfig`, not assumed). That makes **never confirming the safe default**: the bootloader
moves a `PENDING_VERIFY` entry to `ABORTED` on the next boot and falls back to the other
slot, with no code of ours involved. Confirming early throws that safety net away for exactly
the case it exists for.

So confirmation is **earned by running**. `esp_ota_mark_app_valid_cancel_rollback()` is called
only after all of these hold, and `scripts/static_audit.py` fails the build if
`Controller::begin()` ever confirms an image:

```text
Controller::begin() ran to COMPLETION     (the flag is its last statement)
CommandRouter is bound and usable
firmware identity is readable             (running slot is an OTA slot, build id present)
this boot did NOT follow PANIC/INT_WDT/TASK_WDT/WDT/BROWNOUT
uptime >= 15 s
completed loop ticks >= 2000
```

Uptime alone would be satisfied by a controller wedged in one long call; ticks alone by a
fast boot loop. Both are required, and both are finite and deterministic.

A refusal deliberately does **not** call the invalidate-and-reboot API. Refusing is already
sufficient, and rebooting a robot is not OTA-A's decision to make. An explicit, authorized
operator rollback is **TO_IMPLEMENT / OTA-B**.

**Provisional until ActuatorAuthority exists:** these criteria contain no servo, DALY, IMU or
motion condition — deliberately, because peripheral presence is not evidence about firmware
and making it one would roll back a perfectly good image because a cable was unplugged. When
the authority model lands, the question of whether confirmation should additionally require a
safe actuator state is **TO_DESIGN**.

### OTA-B authorization — implemented

`src/update/OtaAuthorityGate.*` is the definitive gate, backed by the real
[ActuatorAuthority](#actuatorauthority) arbiter. The OTA-A placeholder is **gone**, not kept
alongside; the audit fails the build if its names reappear.

OTA never becomes an actuator owner — it drives no actuator, and it does not compete with
`CALIBRATION` for a resource, it requires that nobody is using one. It takes an **exclusivity
inhibit** instead, and it takes it as a *hold* rather than a *query*, because an update spans
seconds of loop passes and a query can only be true about the instant it ran. The reasoning and
the TOCTOU analysis are in the
[exclusivity inhibit](#exclusivity-inhibit--and-the-toctou-problem-it-solves) section.

```text
prepare()            -> beginExclusive() -> requestInhibit(FIRMWARE_UPDATE)
                        granted only when authority == NONE
during the update    -> every request() is REJECTED_INHIBITED
failure/abort/reset  -> endExclusive(), so a failed update never leaves the
                        robot permanently unable to calibrate
successful commit    -> hold KEPT until the reboot: a boot switch is pending,
                        and calibrating against an image about to be replaced
                        is not something to permit for convenience
```

The policy still **fails closed** with no gate installed, and now also with a gate whose arbiter
was never bound. Remaining for OTA-B: an explicit, authorized operator rollback.

### V3 transport and security — 2026-10-04

HTTP OTA challenge/upload are refused. `esp_https_server` provides an optional STA-only
443 listener, backed by a unique per-device PEM certificate/private key from gitignored
`src/config/TlsIdentity.local.h`. The distributed candidate has no identity or HMAC
secret. The listener requires quiet MAINTENANCE, persisted network/admin configuration
and a 100 KB largest internal heap block; one socket, 28 KB task stack and a 5 s
handshake timeout bound resources. These settings are provisional, not a hardware TLS
qualification. Losing eligibility shuts down the listener outside an active safe stream.

The existing `OtaSession` challenge/HMAC protocol and
`OtaManager → OtaPolicy → OtaEspBackend` remain the single firmware writer. No
ArduinoOTA, Update.h or portal writer was introduced. Metadata size/hash/build ID,
nonce expiry/single use, partition identity, authority inhibit and first-boot rollback
remain enforced. Timeout, truncated stream and link loss abort; hash/finalization failure
never requests a new boot target. Success remains COMMITTED_PENDING_REBOOT, with the
actuator inhibit held. **Authenticated remote reboot is BLOCKED / unimplemented** in V1;
no endpoint calls restart and there is no implicit reboot.

Source and final artifacts keep `MATDOG_OTA_INGEST_ENABLED=0`. Enabling ingest requires
separate hardware acceptance of TLS resources, USB recovery, calibration baseline,
authenticated explicit reboot and OTA/rollback gates. Offline linking of a disposable
TLS identity does not authorize that override or deployment.

`scripts/ota_tls_client.py` defaults to offline package verification. It requires a
full independently checked source SHA, CLEAN manifest, exact image/hash/FQBN/profile,
layout/table hash and ESP32-S3 identity. Explicit upload additionally requires CA+SAN
verification and an independently provisioned DER certificate SHA256 pin, checked on
every connection, plus the HMAC secret. It never disables TLS verification, follows
redirects, resumes an old nonce/session or automatically reboots. The manifest is a
provenance check, not an image signing PKI. No uploader operation was run on a device.

### Resource cost

Historical I7 measurement (2026-09-25; superseded for the V3 resource budget):

```text
flash       959,043 B -> 967,915 B   (+8,872 B)   30% of the 3 MB slot
static RAM   50,868 B ->  51,676 B   (+808 B)     15%
```

No image is ever held in RAM: bytes are hashed and written per chunk, and the chunk buffer
belongs to the transport. `esp_ota_begin()` uses `OTA_WITH_SEQUENTIAL_WRITES` so the erase is
incremental per sector instead of a single up-front ~1 MB erase that would stall the loop for
seconds.

**Flash erase and write do block the Controller loop.** That is a property of SPI flash, not
something a comment can fix. `@OTA STATUS` reports the measured worst case
(`open_us`/`write_us`/`end_us`), so the OTA-B integration can argue from numbers. Those
numbers do not exist yet — they require a hardware test.

## Anti-back-power (LED ring)

The WS2812B ring's 5V rail is physically absent under `USB_ONLY`
(`build::kLedRailPowered == false`). `LedRing::begin()` sets GPIO47 to `INPUT` and
never calls into `Adafruit_NeoPixel` — no WS2812 frame is ever transmitted toward the
unpowered ring, including via `@LED TEST` (refused with `REASON=LED_RAIL_UNPOWERED`).
`LedRing::dataPinDriven()` lets `@LED STATUS`/tests confirm this held for the whole
session. Since G2, `kLedRailPowered` is no longer independently editable: it and the two
other rail flags are derived from the selected hardware profile (see Hardware profile
above), and `static_audit.py` asserts both that they stay derived and that the
compiled-in default profile remains `USB_ONLY`.

## Power architecture

Canonical power domains, `KEY`/Charge-MOS semantics, power states, daily use, service isolation
and charging live in
[`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md);
wiring and components in [`04_Electronics/README.md`](../../04_Electronics/README.md).

The firmware consequences are:

- DALY `KEY` is wired directly to the bistable logo pushbutton, **not** to an ESP32 GPIO. Power-on
  is hardware-first, and firmware cannot be the primary wake controller because the ESP32 sits
  downstream of the DALY-protected supply it would have to enable.
- KEY controls the **discharge MOS only** (`0x0120 = 0x005A`, live-verified 2026-09-19). The charge
  MOS stays independent and normally ON; firmware must never map KEY to it, and `0x0121`/`0x0122`
  writes stay forbidden.
- Every ordinary load, the ESP32 included, returns through DALY `P-`; never raw `B-`.
- "Powered" and "motion enabled" are separate states: a future docked/charging robot stays powered
  with torque off and motion inhibited, and docking must never write the MOS registers.
- KEY OFF is a **validated power-off** for the robot as built, with no charger and no USB
  connected (post-rewire power gate A–E PASS, 2026-09-24 — see `DEVELOPMENT_GATES.md` and
  `VALIDATION.md`). The historical `B-`/`P-` bypass that made the first physical test inconclusive
  is corrected. A connected charger still backfeeds the `B+`/`P-` load bus independent of KEY
  state, so the fused disconnect remains the trusted maintenance-isolation point whenever a charger
  may be present — see `04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` §§ 7–8.

## Bench test profile

The source default remains the **USB_ONLY** bench profile. `ROBOT_POWERED` is **VALIDATED
no-motion** on the real robot (G3 formal PASS and G3.1 PASS, 2026-09-18; build `e2fc605`) —
see `VALIDATION.md`. Each powered session still needs its own authorization.

Under `USB_ONLY`:
ESP32-S3 powered solely from the host USB link, battery/step-down/servo-power/LED-5V
rails all absent by design. `DalyBms`, the ST3215 scan path and `LedRing` all report an
explicit `OFFLINE_EXPECTED` classification under this profile rather than treating
absent hardware as a fault. See `VALIDATION.md` for what that profile does and does not
cover, and handoff section 7A for the full matrix.

## Directory layout

```text
05_Firmware/MATDOG_Controller/
├── MATDOG_Controller.ino     thin entry point (setup/loop only)
├── src/
│   ├── config/                Pins.h (central GPIO ownership), BuildConfig.h,
│   │                          HardwareProfile.h (USB_ONLY / ROBOT_POWERED authority)
│   ├── core/                  Controller, SystemState, PowerState, CommandRouter,
│   │                          Availability (init/detected/expected/result model),
│   │                          OperatingMode (MAINTENANCE/RUN)
│   ├── actuator/              ActuatorWritePolicy (pure: the Safe Actuator Layer
│   │                          decision core; no transport, no write path),
│   │                          CalibrationGeometryProfile + its GENERATED data
│   │                          table (compiled Geometry V5; no mesh, no FK)
│   ├── servo/                 ServoBus, ServoPopulation (pure policy),
│   │                          ServoCensus (Controller-owned service)
│   ├── imu/                   Bno085Imu
│   ├── power/                 DalyBms (UART owner), DalyProtocol (pure: read frames,
│   │                          decoders, KEY model, single-owner bus scheduler)
│   └── status/                LedRing
└── scripts/
    ├── build.sh
    ├── static_audit.py
    ├── upload.sh                        refusing stub — no full-image upload (P2.3)
    ├── flash_app_only.sh                application-only flash (routine path)
    ├── verify_application_partition.py  device-I/O wrapper used by the above
    ├── ota_partition_logic.py           pure OTA slot-selection logic (offline-testable)
    └── tests/
        ├── test_ota_partition_logic.py  offline unit tests, no device/flash required
        ├── test_servo_population.cpp    offline census/profile tests (host g++)
        ├── test_daly_protocol.cpp       offline DALY protocol / KEY probe tests (host g++)
        ├── test_static_audit_daly.py    DALY write-prohibition audit mutation tests
        ├── test_actuator_write_policy.cpp        offline safe actuator policy tests (host g++)
        ├── test_calibration_geometry.cpp         offline geometry contract tests (host g++)
        ├── test_static_audit_safe_actuator.py    safe actuator boundary audit mutation tests
        └── run_host_tests.sh            compiles + runs the C++ suites
```
