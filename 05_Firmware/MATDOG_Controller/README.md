# MATDOG Controller

The first unified operational ESP32-S3 runtime for MATDOG. It replaces the previous
one-sketch-per-peripheral workflow (ST3215 bench tools, BNO085 Phase C3, DALY probes)
with one modular, deployable firmware image that initializes each subsystem module
together and reports whether its hardware is detected, expected or unavailable.

```text
MATDOG Controller
├── core/system      boot, version, health aggregation, power-state machine,
│                    cooperative non-blocking scheduling, USB command router
├── servo/           ServoBus — ST3215 / Seeed bus transport, read-only diagnostics
├── imu/             Bno085Imu — SH2_ROTATION_VECTOR acquisition, viewer-compatible
├── power/           DalyBms — read-only Modbus RTU battery telemetry
└── status/          LedRing — WS2812B ring, boots OFF, non-blocking effects
```

This is an **integration and platform milestone**, not a motion controller. No gait,
IK, closed-loop stabilization, ROS 2/MoveIt 2, Wi-Fi/OTA or autonomous behaviour is
implemented here — see `VALIDATION.md` for the precise scope.

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

The preserved branch `matdog/full-leg-calibrator-v1` is an oracle for calibration-engine, safety
and evidence logic, not a replacement runtime and not a branch to merge wholesale. Likewise, the
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

```text
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
```

Board: YD-ESP32-S3 N16R8 (16 MB flash, 8 MB OPI PSRAM, 240 MHz).

## Update and recovery policy

- **DECIDED — future normal update path:** Wi-Fi / OTA. It is not implemented in V0.1.
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
table and otadata (the `app3M_fat9M_16MB` scheme has two OTA slots; this never assumes
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

`scripts/upload.sh` (full Arduino upload — bootloader + partition table + boot_app0 +
application, every time) is kept for the legitimate full-image case (e.g. bring-up on
a replacement board) but is **not** the routine flashing path; see `VALIDATION.md`
Session 2 for why its earlier "application-only" framing was wrong and how that was
confirmed harmless in practice.

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

## USB diagnostic command surface

```text
@HELP
@STATUS
@IMU STATUS | @IMU STREAM ON|OFF
@BMS STATUS | @BMS STREAM ON|OFF
@LED STATUS | @LED OFF | @LED TEST
@SERVO SCAN <lo> <hi> | @SERVO READ <id>   (MAINTENANCE mode only)
@SERVO SAFE_OFF <id>                        (always allowed, any mode)
@MODE STATUS | @MODE MAINTENANCE | @MODE RUN
@SYSTEM SHUTDOWN
```

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
the BNO085 DCD, or write DALY configuration/MOS state.

`@STATUS` (and the per-module `@IMU`/`@BMS`/`@LED` variants) report each module as
`init=.. detected=.. expected=.. result=..` — separating "did the driver initialize"
from "was the hardware actually detected" from "was it expected to be reachable right
now" from "is that a problem". See `src/core/Availability.h`; under `USB_ONLY`:

```text
BNO085 init=OK       detected=ONLINE      expected=REQUIRED  result=PASS
DALY   init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
SERVO  init=OK       detected=UNKNOWN     expected=OFFLINE   result=PASS
LED    init=DEFERRED detected=UNPOWERED   expected=UNPOWERED result=PASS
```

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

## Anti-back-power (LED ring)

The WS2812B ring's 5V rail is physically absent under `USB_ONLY`
(`build::kLedRailPowered == false`). `LedRing::begin()` sets GPIO47 to `INPUT` and
never calls into `Adafruit_NeoPixel` — no WS2812 frame is ever transmitted toward the
unpowered ring, including via `@LED TEST` (refused with `REASON=LED_RAIL_UNPOWERED`).
`LedRing::dataPinDriven()` lets `@LED STATUS`/tests confirm this held for the whole
session. Flipping `kLedRailPowered` (and the two other profile flags in
`BuildConfig.h`) to move to a future `ROBOT_POWERED` profile is a deliberate, reviewed
change — `static_audit.py` currently asserts all three read `false`.

## Power architecture

Canonical power and wiring details live in
[`04_Electronics/README.md`](../../04_Electronics/README.md). The firmware consequence is that
DALY `KEY` is controlled directly by the bistable logo pushbutton, not by an ESP32 GPIO. Power-on is
therefore hardware-first, and firmware cannot be the primary wake controller because the ESP32 is
downstream of the DALY-protected supply it would need to enable.

## Bench test profile

This firmware currently ships validated only against the **USB_ONLY** bench profile:
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
│   ├── config/                Pins.h (central GPIO ownership), BuildConfig.h
│   ├── core/                  Controller, SystemState, PowerState, CommandRouter,
│   │                          Availability (init/detected/expected/result model),
│   │                          OperatingMode (MAINTENANCE/RUN)
│   ├── servo/                 ServoBus
│   ├── imu/                   Bno085Imu
│   ├── power/                 DalyBms
│   └── status/                LedRing
└── scripts/
    ├── build.sh
    ├── static_audit.py
    ├── upload.sh                        full Arduino upload (not routine — see above)
    ├── flash_app_only.sh                application-only flash (routine path)
    ├── verify_application_partition.py  device-I/O wrapper used by the above
    ├── ota_partition_logic.py           pure OTA slot-selection logic (offline-testable)
    └── tests/
        └── test_ota_partition_logic.py  offline unit tests, no device/flash required
```
