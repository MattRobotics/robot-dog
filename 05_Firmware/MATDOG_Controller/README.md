# MATDOG Controller

The first unified operational ESP32-S3 runtime for MATDOG. Replaces the previous
one-sketch-per-peripheral workflow (ST3215 bench tools, BNO085 Phase C3, DALY probes)
with one modular, deployable firmware image that boots every installed subsystem
together.

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

## Application-only flashing

```bash
scripts/flash_app_only.sh
```

Writes **only** the currently-active OTA application partition — never the
bootloader, partition table or boot_app0/otadata. `scripts/verify_application_partition.py`
determines that partition's real offset/size by reading the device's own partition
table and otadata (the `app3M_fat9M_16MB` scheme has two OTA slots; this never assumes
which one is active). Refuses to run unless: the working tree is clean and the
compiled binary's embedded build id matches `HEAD`, the 16 MiB full-flash backup
verifies, the static audit passes, and the connected device's MAC matches the expected
one. Prints `DEVICE`/`APPLICATION_BINARY`/`APPLICATION_SHA256`/`APPLICATION_OFFSET`/
`APPLICATION_SIZE`/`MAX_PARTITION_SIZE`/`FQBN`/`SOURCE_COMMIT` before writing anything,
and independently re-verifies the write afterward with `esptool verify-flash`.

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
DalyBms, a blocking multi-ID servo scan, the LED transport being driven while
`kLedRailPowered` is false, or `scripts/flash_app_only.sh` regressing to reference the
bootloader/partition-table/boot_app0 artifacts it must never write.

## USB diagnostic command surface

```text
@HELP
@STATUS
@IMU STATUS | @IMU STREAM ON|OFF
@BMS STATUS | @BMS STREAM ON|OFF
@LED STATUS | @LED OFF | @LED TEST
@SERVO SCAN <lo> <hi> | @SERVO READ <id> | @SERVO SAFE_OFF <id>
@SYSTEM SHUTDOWN
```

`@SERVO SAFE_OFF` can only disable torque, never enable it. `@SERVO SCAN` is
non-blocking: it replies `SERVO_SCAN=STARTED` immediately and the router reports
`SERVO_SCAN=COMPLETE` asynchronously once the scan's one-ID-per-tick state machine
finishes, without ever stalling BNO085 acquisition or the command router itself.
`@LED TEST`/`@LED OFF` are refused/no-op under the current `USB_ONLY` profile — see
Anti-back-power below. `@SYSTEM SHUTDOWN` walks the power-state machine through its
shutdown sequence but always resolves to `POWER_CUT_FAILED` in V0.1, because the DALY
K-Series `Discharge MOS OFF` write protocol has not been identified or bench-verified
(see `VALIDATION.md` and handoff section 8A.8). No command in this surface can write
servo EEPROM, recode an ID, save the BNO085 DCD, or write DALY configuration/MOS state.

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

ESP32-S3 and its 5 V step-down live behind the DALY's protection MOS: positive supply
is **B+**, return is **P−**, never **B−**. DALY `KEY` is hardware-only — a momentary
switch wired directly to the BMS, mechanically latched by the MATDOG logo — and is
never wired to an ESP32 GPIO. Normal power-on is hardware-first (KEY → DALY enables
P− → step-down powers the ESP32 → the ESP32 boots); the firmware cannot be the primary
wake controller because it is itself downstream of the domain it would need to
control. Full baseline in the integration handoff sections 8A/21A.

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
│   │                          Availability (init/detected/expected/result model)
│   ├── servo/                 ServoBus
│   ├── imu/                   Bno085Imu
│   ├── power/                 DalyBms
│   └── status/                LedRing
└── scripts/
    ├── build.sh
    ├── static_audit.py
    ├── upload.sh                        full Arduino upload (not routine — see above)
    ├── flash_app_only.sh                application-only flash (routine path)
    └── verify_application_partition.py  read-only offset/size verifier used by the above
```
