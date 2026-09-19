# MATDOG Controller V0.1 — Validation

Current-facing status vocabulary follows the canonical meanings in the root
[`README.md`](../../README.md#status-vocabulary): **VALIDATED**, **IMPLEMENTED**,
**DECIDED**, **TO_TEST**, **TO_DESIGN**, **FROZEN**, **SUPERSEDED**, and
**HISTORICAL**. Dated session evidence below preserves the status language that
was used when each session was recorded; it is not rewritten into present-day
terminology.

This document will be updated in place as hardware sessions progress; it is not
rewritten per session.

## Present-day baseline and next gate

The official Controller baseline is now merged and tagged. The session records below preserve the
state and merge judgements that were true when each session ended; their historical "not merged"
checkboxes are not current project status and are intentionally not rewritten.

| Identity | Value |
|---|---|
| Official release tag | `matdog-controller-v0.1.0` |
| Tagged repository commit | `c54862f38a9cbd5e46d6b1770a6d109cc99b5c02` |
| Exact validated firmware source | `5b371da5482f9b0bd2df1c37ed361250ea54ae8f` |
| Validated application SHA256 | `6e6d92f898dbe95000b53dbb252c7eb5d3deaa9a4b161e2b1934436a76b29364` |

### VALIDATED

- `USB_ONLY` boot on the real ESP32-S3, native USB CDC and live BNO085 acquisition;
- viewer protocol compatibility, expected-offline classification and USB-only soak behavior;
- unpowered servo diagnostics and honest `SAFE_OFF=UNVERIFIED_NO_RESPONSE` classification;
- the exact source/application provenance recorded above;
- **`ROBOT_POWERED` no-motion operation — G3 formal PASS (2026-09-17 / 2026-09-18):** live
  read-only DALY, live LED, 13/13 expected servos with 4 absent by design in two identical
  censuses, `VERIFIED_OFF` and `torque=0` on all 13, concurrent soak;
- **CDC-independent Controller loop — G3.1 PASS (2026-09-18):** BNO085 acquisition at 50.1 Hz with
  the USB CDC port closed, open, and with `@BMS STREAM` enabled.

Powered build currently on the robot:

| Identity | Value |
|---|---|
| Source commit | `e2fc60531b28351472a2bfa5105a2170147610bb` |
| Hardware profile | `ROBOT_POWERED` build override; the source default remains `USB_ONLY` |
| Application | 387808 bytes, SHA256 `e2b474b5c07e98649fbaf31d3d08970d022ffbdb5f1212d6b606dc28cac065d2` |
| Flash path | `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`, `app0 @ 0x010000` |

### OPEN

- **DALY `KEY` — OPEN.** Toggling the physical KEY switch produced no observed change in any
  DALY-reported state (`discharge_mos=ON` in both positions). KEY is **not** a validated shutdown
  or safety barrier; the DALY's actual KEY configuration and function must be inspected before any
  setting is changed. The fused disconnect remains the trusted physical isolation method.
- **GPIO19/GPIO20 — frozen.** GPIO19 = native USB D−, GPIO20 = native USB D+. The external
  19/20/GND connector remains a future USB service-port candidate — **not** a UART — pending
  electrical and signal-integrity validation.

### Next

```text
DALY KEY investigation
→ G4 Diagnostics / Maintenance
```

No motion, calibration or write-capable service capability exists in the firmware. Every later
stage has its own gate in [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md).

The expected installed servo IDs are `11,12,13,21,22,23,31,32,33,41,42,43,51`: 12 legs plus
`NECK_ROTATION` ID51. IDs 52 `NECK_PITCH`, 53 `HEAD_ROTATION`, 54 `HEAD_PITCH` and 55 `JAW` remain
canonically allocated but are intentionally absent today; their absence is not a failed census.
The allocation authority is
[`MATDOG_SERVO_ALLOCATION.yaml`](../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml),
while the root [`README.md`](../../README.md) owns the current physical-population snapshot.

This gate requires separate hardware authorization. It permits read-only inspection plus the
existing torque-off-only `SAFE_OFF` command and forbids motion, torque-on, servo EEPROM/ID writes,
BNO085 DCD writes and DALY writes. Passing earlier single-device bench campaigns does not constitute
passing this assembled, concurrent ROBOT_POWERED gate.

## Session 1 — 2026-09-15, USB-only bench state

Hardware configuration for this session, per the REV3 handoff:

```text
ESP32-S3      powered solely from ASUS USB
battery       NOT connected
5V step-down  NOT powered
BNO085        wired, 3V3-powered — live-testable
Seeed logic   wired, interface-level only
ST3215        wired, NOT powered — OFFLINE_EXPECTED
DALY (XY-017) wired, NOT powered — OFFLINE_EXPECTED
WS2812B ring  wired, no 5V — OFFLINE_EXPECTED / UNPOWERED
```

### H0 — Offline gates

| Gate | Result |
|---|---|
| Git state audited before editing | PASS — clean `main` @ `03992a4`, branched to `feat/matdog-controller-v01` |
| Source provenance hashed | PASS — see `SOURCE_PROVENANCE.md` |
| Compile (pinned FQBN) | **PASS** — 382248 bytes flash (12%), 27936 bytes RAM (8%), zero warnings from MATDOG_Controller source (only pre-existing vendored SCServo-library warnings) |
| Static safety audit | **PASS** — `scripts/static_audit.py`, 19 source files scanned, 0 findings |
| Existing BNO085 viewer test suite | **PASS** — 59/59 tests, typecheck clean, production build succeeds; viewer directory untouched |
| Full-flash backup verified | **PASS** — `matdog_esp32s3_fullflash_2026-09-10.bin`, 16,777,216 bytes, SHA256 `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32` |

### Upload

```text
tool          : arduino-cli upload (esptool v5.3.1 stub) — a FULL Arduino
                application upload, not an application-only one; see the
                Session 2 correction below
target        : ESP32-S3 QFN56 rev v0.2, MAC 14:c1:9f:22:75:94
port          : /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00
build_id      : 03992a473bba, then 03992a473bba-dirty (after the heap-reporting addition below)
partitions touched : bootloader, partition table, boot_app0, application
                      (no flash erase, no NVS erase)
result        : hash verified on every section written
```

> **Correction recorded in Session 2 (2026-09-15):** `scripts/upload.sh`'s
> header comment at the time of this session claimed this was a "normal
> application upload" that "does not touch the bootloader/partition table".
> That claim was wrong — the log above already correctly listed all four
> regions written, but the script's own documentation contradicted its own
> evidence. See "Session 2 — Hardening & Reproducible Baseline" below for
> the read-only audit that confirmed this was harmless in practice (every
> region came back byte-identical to the pre-Session-1 backup) and for the
> corrected application-only flashing path.

### U1 — Boot — **PASS**

Live boot banner captured from a clean RTS-triggered reset:

```text
====================================
 MATDOG Controller 0.1.0
====================================
build      : 03992a473bba
board      : YD-ESP32-S3 N16R8
profile    : USB_ONLY
servo      : GPIO17/18 @ 1000000
bms        : GPIO15/16 @ 9600
imu        : BNO085 SPI (SCK=1 MISO=2 MOSI=40 CS=41 INT=42 RST=39 PS0=38)
led        : GPIO47 / 12 px
reset_reason : OTHER
startup_motion   : DISABLED
startup_torque   : DISABLED
daly_write       : NOT_IMPLEMENTED (protocol unverified)

IMU_INIT=PASS
SYSTEM_BOOT_COMPLETE health=DEGRADED power_state=RUN
EXPECTED_STARTUP_RESET=YES
```

- Version/build/board/pin banner: **PASS**, all fields correct (build id matches the
  `HEAD` commit at audit time).
- USB CDC: **PASS** — port re-enumerated cleanly after both the flash upload and the
  esptool-triggered hard reset; every command exchange below used the same port.
- No spontaneous servo motion: **PASS** — `ServoBus::begin()` issues no ping/scan/write;
  confirmed by `static_audit.py` and by the absence of any servo activity in the capture.
- No torque-on: **PASS** — banner explicitly prints `startup_torque : DISABLED`; no
  `EnableTorque(id, 1)` call exists anywhere in the source (enforced by the audit script).
- No reboot loop: **PASS** — `EXPECTED_STARTUP_RESET=YES` appears exactly once per
  session; `runtime_resets` stayed at `0` through every subsequent capture, including a
  ~150 s and a ~95 s soak window (see U4).
- `reset_reason=OTHER`: observed on the esptool/RTS-pin hard reset path. This is a
  characteristic of how the ESP32-S3's USB-Serial-JTAG reset line is categorized by
  `esp_reset_reason()`, not a fault — no panic/watchdog/brownout reason was ever
  reported in any capture.

### U2 — BNO085 live — **PASS**

Continuous `RV`/`MAG`/`GYR`/`COUNTS`/`SAVE_GATE` blocks streamed at the expected ~500 ms
cadence (50 Hz internal RV/MAG rate) from boot:

```text
RV w=0.712585 x=-0.008667 y=0.002930 z=0.701477 accuracy_rad=0.071289 status=3 count=528
MAG x=25.438 y=1.250 z=-41.625 |B|=48.798 status=3 count=554
SAVE_GATE ready=YES still_ms=10942 ACC=2 GYR=0 MAG=3 RV=3 rv_accuracy_rad=0.067383 gyro_mag=0.000000 runtime_resets=0
```

- `status` reached full calibration (`3`) for both RV and MAG within ~5 seconds of boot.
- `SAVE_GATE ready` correctly transitioned `NO → YES` once the sensor had been stationary
  for the required 5000 ms — proves the ported interlock logic (still-time, accuracy
  threshold, per-report status gates) behaves identically to the standalone Phase C3
  firmware on real hardware, even though this firmware exposes no path to actually
  execute a save.
- `runtime_resets=0` throughout every capture in this session.
- **Existing viewer compatibility**: the viewer's own automated suite (59/59 tests,
  typecheck, production build) passed unmodified against the unchanged parser/frame
  code (see H0 above). The live line format captured from hardware is byte-identical
  to what `lineParser.ts` and `tests/lineParser.test.ts` expect. The viewer dev server
  was started (`npm run dev`, `http://127.0.0.1:5183/`) against the live device for the
  operator to visually confirm — the agent has no GUI browser automation available in
  this environment and Web Serial requires a human user-gesture to open the port, so
  the interactive "watch the 3D model track the sensor" step was not personally
  operated by the agent. Protocol-level compatibility is validated; the visual
  confirmation is a one-click operator step against an already-running firmware and
  dev server.

### U3 — Expected-offline modules — **PASS**

Exercised through the USB diagnostic surface (`@BMS STATUS`, `@SERVO SCAN 11 55`,
`@LED STATUS`), IDs 11–55 chosen to cover the full current servo allocation range:

```text
BMS_STATUS health=OFFLINE comm=TIMEOUT age_ms=1008
DALY_DETECTED=NO_RESPONSE
PROFILE=USB_ONLY
EXPECTED=OFFLINE
CLASSIFICATION=OFFLINE_EXPECTED/PASS

SERVO_SCAN lo=11 hi=55 found=0
PROFILE=USB_ONLY
EXPECTED=OFFLINE (servo power rail absent)
CLASSIFICATION=OFFLINE_EXPECTED/PASS

LED_STATUS health=OK pixels=12 pin=47 brightness_max=60 test_running=NO
PROFILE=USB_ONLY
EXPECTED=UNPOWERED (5V rail absent)
CLASSIFICATION=OFFLINE_EXPECTED/UNPOWERED
LED_TEST=STARTED
```

All three peripherals classify correctly as expected-absent rather than faulted, in
the exact `PROFILE=/EXPECTED=/CLASSIFICATION=` shape the handoff specified. `@LED TEST`
was accepted and ran its non-blocking chase state machine to completion with no error
and no impact on IMU streaming; no optical confirmation is possible without the 5 V
rail (declared out of scope for this session).

### U4 — Soak — **PASS**

Two soak windows were run with all modules active simultaneously (IMU streaming,
DALY polling every 2 s via `@BMS STREAM ON`, USB command router live):

```text
run 1: ~150 s, BMS streaming enabled
  runtime_resets            : 0
  RV count                  : 1418 → 8895  (≈ 49.8 Hz sustained, no stalls from DALY polling)
  BMS_STATUS prints         : 74  (one every ~2.0 s, exactly as designed)
  BMS comm result           : TIMEOUT every cycle, age_ms bounded ~1240-1245 ms (never grows unbounded)
  fatal/panic/guru meditation : none

run 2: ~95 s, on the final build (post heap-reporting change)
  runtime_resets             : 0
  heap_free  @ t≈5s          : 335172 bytes free / 329532 min-free
  heap_free  @ t≈90s         : 334820 bytes free / 329532 min-free  (delta: -352 bytes, one-time; flat afterward)
  module health              : imu=OK servo=OK bms=OFFLINE led=OK, unchanged throughout
```

No serial corruption, no UART lockup, no missed BNO samples, no watchdog resets. The
DALY non-blocking poll state machine (send → accumulate → 750 ms bounded timeout) cycles
cleanly forever without ever stalling BNO085 acquisition or the command router — the
concrete behavioural goal of moving off the original probes' blocking 750 ms wait.

## What is explicitly NOT validated by this session

```text
DALY live telemetry / MOS / faults        — battery not connected
DALY writes                                — protocol not verified; not implemented
LED optical output / current               — 5V rail absent
ST3215 responses / telemetry / torque / motion — servo power absent
full powered-system coexistence            — requires battery + 5V
KEY / P− transitions                        — requires battery
software self-power-cut                     — DALY write blocked by design
```

## Acceptance criteria — software (handoff section 41)

- [x] clean modular build succeeds
- [x] no frozen source modified
- [x] source provenance table exists
- [x] known FQBN documented
- [x] no GPIO collision (enforced by `static_audit.py`)
- [x] servo UART and DALY UART are separate peripherals (enforced by `static_audit.py`)
- [x] USB CDC remains functional
- [x] BNO viewer tests still pass (59/59)
- [x] `RV` line remains compatible (format preserved verbatim from Phase C3)
- [x] no automatic BNO DCD write (no code path to `sh2_saveDcdNow()` exists)
- [x] no DALY configuration write (`requestDischargeOff()` transmits nothing)
- [x] no servo EEPROM write in operational runtime (enforced by `static_audit.py`)
- [x] no automatic torque/motion on boot (enforced by `static_audit.py`)
- [x] no signed GoalPosition logic (no GoalPosition register touched at all)
- [x] LED boot state OFF
- [x] ESP32/step-down power architecture documented as B+ / P− (README)
- [x] no firmware dependency on an ESP32-controlled DALY KEY
- [x] DALY write path absent/fail-closed until K-Series protocol verification
- [x] shutdown ordering guarantees servo-safe state before any future Discharge MOS OFF
- [x] power-state machine documents OFF → KEY ON → BOOT → CHECK → RUN → SHUTDOWN → OFF
- [x] build memory footprint recorded
- [x] static audit passes

## Acceptance criteria — hardware (handoff section 42, USB-only scope)

- [x] Controller V0.1 boots on the real ESP32-S3
- [x] USB CDC reconnects correctly
- [x] no spontaneous servo motion
- [x] BNO085 initializes
- [x] existing 3D viewer works unchanged (protocol-level: 59/59 automated tests against
      the live-matching line format; interactive browser visual check is a one-click
      operator step — see U2, no GUI automation available to the agent in this environment)
- [x] DALY, LED, ST3215 correctly classified OFFLINE_EXPECTED (not fault) under USB_ONLY
- [x] no UART contention (ServoBus on HardwareSerial(1), DalyBms on HardwareSerial(2);
      soak run confirms no serial corruption with both active)
- [x] no repeated resets/watchdog events (`runtime_resets=0` across all captures)
- [x] soak test stable (two runs, ~150s and ~95s; heap flat, RV cadence steady ~50Hz)
- [x] exact tested commit/hash documented (build id `03992a473bba` / `03992a473bba-dirty`,
      base commit `03992a473bba9220f32172ac57ac5d860f3d1462`)

Full powered-system items (DALY live telemetry, LED optical output, servo read-only
communication with all modules active, no-UART-contention-under-full-load) remain
**NOT YET VALIDATED** — battery and 5V step-down are intentionally absent this
session per the REV3 handoff.

---

## Session 2 — 2026-09-15, Hardening & Reproducible Baseline

Same physical hardware configuration as Session 1 (USB_ONLY, battery/5V/servo-power/
LED-5V all absent by design — see the matrix at the top of this document). This
session's scope was corrective/procedural hardening, not new functionality: five
findings from a Session 1 audit, fixed, tested and re-validated on real hardware.
No gait/IK/motion/DALY-write/KEY-GPIO/servo-calibration work was done.

### Provenance — do not confuse these two commits

```text
FIRMWARE_SOURCE_COMMIT = 04dfa52d1b5ab37ac4099792bb8c874c5ee4842a
  (clean tree; this exact commit was compiled and is what is running on
   the device as of this session — see "Application-only flash" below)

VALIDATION_DOC_COMMIT  = <the commit that records this Session 2 section
   plus the accompanying README/CHANGELOG updates — created AFTER
   FIRMWARE_SOURCE_COMMIT and after the device was already flashed and
   validated>
```

The running firmware's boot banner and `@STATUS` report `build=04dfa52d1b5a`
(the short form of `FIRMWARE_SOURCE_COMMIT`) — this is what to check against, not
whatever commit is at the tip of the branch when this file is read later.

### Finding 1 — `upload.sh` was not application-only (read-only flash audit)

Before any new write, every region Session 1's upload touched was read back
read-only and compared against the pre-Session-1 full-flash backup
(`matdog_esp32s3_fullflash_2026-09-10.bin`, 16,777,216 bytes, SHA256
`5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`):

| Region | Offset | Size | Result |
|---|---|---:|---|
| Bootloader | 0x0 | 20,480 B | **SAME_AS_BACKUP** |
| Partition table | 0x8000 | 4,096 B | **SAME_AS_BACKUP** |
| otadata / boot_app0 | 0xe000 | 8,192 B | **SAME_AS_BACKUP** |

All three came back byte-identical to the backup — Session 1's writes to these
regions were real writes (confirmed by its own upload log) but wrote the same
bytes that were already there, so no drift occurred and no restoration was
needed. This was corroborating evidence, not an assumption: the on-device
otadata was also confirmed byte-identical to the standard `boot_app0.bin` seed
file shipped with the installed esp32 Arduino core (SHA256
`f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0`), and the
on-device partition table was independently parsed (not just hash-compared) to
confirm the `app0`/`ota_0` entry at offset `0x010000`, size `0x300000`
(3,145,728 bytes) — matching `partitions.csv` from the build and the compiler's
own reported max app size.

`scripts/upload.sh`'s header comment (which had claimed "does not touch the
bootloader/partition table") is corrected. The script itself is kept for the
legitimate full-image case; it is not part of the routine flashing path anymore.

### Finding 2 — true application-only flashing

`scripts/verify_application_partition.py` (read-only) parses the device's own
partition table and otadata to determine the currently active OTA slot and its
verified offset/size — it does not assume `0x10000` from Session 1 precedent.
`scripts/flash_app_only.sh` gates on: clean working tree, application binary
present and its embedded build id matching current `HEAD`, backup
existence/size/hash, static audit PASS, device MAC match, and the verified
offset/size — then issues exactly one `esptool write-flash <offset> <binary>`
call. `static_audit.py` now fails the build if this script is ever edited to
reference the bootloader/partition-table/boot_app0 artifact filenames again.

### Finding 3 — LED anti-back-power

Session 1's `LedRing` called `pixels_.begin()`/`show()` and transmitted real
WS2812 frames via `@LED TEST` despite the ring's 5V rail being physically
absent. `LedRing::begin()` now only sets GPIO47 to `INPUT` and never touches
`Adafruit_NeoPixel` when `build::kLedRailPowered == false`; `@LED TEST` is
refused with an explicit reason.

### Finding 4 — initialized/detected/expected/result

Session 1's per-module health was a single ad-hoc `ModuleHealth` value, so
`servo=OK`/`led=OK` in `@STATUS` meant only "driver initialized", not "hardware
present". `core::Availability` separates the four questions; `@STATUS` now
prints all four per module.

### Finding 5 — non-blocking servo scan

Session 1's `scan(lo, hi)` pinged an entire ID range synchronously (each
`Ping()` carries SCServo's ~100ms `IOTimeOut`), so a 45-ID range with nothing
responding could monopolize `loop()` for ~4.5s. `ServoBus` now runs
`startScan()`/`update()`/`scanState()`: one `Ping()` per tick, result reported
asynchronously.

> **Correction recorded in Session 2.1 (2026-09-15):** calling this
> "non-blocking" was imprecise and has been corrected. `SCServo::Ping()` is
> still a synchronous call with its own bounded per-call timeout — one
> `update()` tick can still block the caller for up to that long. The
> accurate description is "incremental scan with bounded per-ID blocking".
> Session 2.1 additionally confines `@SERVO SCAN`/`@SERVO READ` to a new
> `MAINTENANCE` operating mode so this can never be reachable from a future
> motion `RUN` loop, and reduces the per-ID timeout from 100ms to a
> hardware-measurement-justified 20ms. See "Session 2.1" below.

### H0 — Offline gates (Session 2)

| Gate | Result |
|---|---|
| Git state re-audited, branch/HEAD/frozen-source hashes confirmed unchanged from Session 1 | PASS |
| Read-only flash region audit vs backup | PASS — see Finding 1 table above |
| Compile (pinned FQBN, clean `FIRMWARE_SOURCE_COMMIT`) | **PASS** — 383180 bytes flash (12%), 28216 bytes RAM (9%), zero warnings from MATDOG_Controller sources |
| Static safety audit (extended) | **PASS** — 21 source files, 0 findings |
| Existing BNO085 viewer test suite | **PASS** — 59/59, unmodified |
| Application binary provenance | `MATDOG_Controller.ino.bin`, 383,328 bytes, SHA256 `1ca2eed250e470a3c0fc70b99ff2f7dbb4f29c9d7ca1a4aede29a4ce64d6f452`, embeds build id `04dfa52d1b5a` matching `FIRMWARE_SOURCE_COMMIT` |

### Application-only flash

```text
DEVICE               = /dev/serial/.../usb-Espressif_USB_JTAG_...-if00 (MAC 14:c1:9f:22:75:94)
APPLICATION_BINARY   = build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
APPLICATION_SHA256   = 1ca2eed250e470a3c0fc70b99ff2f7dbb4f29c9d7ca1a4aede29a4ce64d6f452
APPLICATION_OFFSET   = 0x010000 (partition 'app0' / ota_0, verified from the device's
                        own partition table + otadata, not assumed)
APPLICATION_SIZE     = 383328 bytes
MAX_PARTITION_SIZE   = 3145728 bytes
FQBN                 = esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,
                        CPUFreq=240,FlashMode=qio,FlashSize=16M,
                        PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
SOURCE_COMMIT        = 04dfa52d1b5ab37ac4099792bb8c874c5ee4842a
```

Write result: esptool's own post-write hash check passed ("Hash of data
verified"). Independent post-write verification via `esptool verify-flash`
(device content compared directly against the local binary) reported
"Verification successful (digest matched)". A raw `read-flash`-into-a-file
readback was tried first and proved intermittently unreliable immediately
after a write+reset in this environment (reproducible "Packet content transfer
stopped" around the same byte offset across several retries) — root-caused to
a USB-Serial-JTAG re-enumeration timing quirk, not a flash content problem,
and superseded by `verify-flash` in the script (see commit history).

**Post-write confirmation that only the application partition changed:**
bootloader, partition table and otadata were re-read and re-verified with
`esptool verify-flash` against the exact same reference files captured during
the Finding-1 audit — all three reported "Verification successful (digest
matched)", i.e. byte-for-byte unchanged by the application-only write.

### H1 — Boot — **PASS**

```text
build        : 04dfa52d1b5a
partition    : app0 @ 0x010000 (size 0x300000)
reset_reason : OTHER
startup_motion   : DISABLED
startup_torque   : DISABLED
daly_write       : NOT_IMPLEMENTED (protocol unverified)
IMU_INIT=PASS
SYSTEM_BOOT_COMPLETE health=READY power_state=RUN
EXPECTED_STARTUP_RESET=YES
```

`SYSTEM_BOOT_COMPLETE health=READY` — a behavioural improvement from Finding 4:
Session 1 always reported `DEGRADED` here (DALY's old `ModuleHealth::OFFLINE`
dragged system health down even though absence was expected and correct for
USB_ONLY). Under the corrected model, expected-offline classifies as PASS, so
nominal USB_ONLY boot now correctly reads READY.

### H2 — BNO085 — **PASS**

RV/MAG reached full calibration (`status=3`) within seconds, exactly as
Session 1; `runtime_resets=0` throughout every capture this session. Existing
viewer protocol unchanged (byte-identical line formats; 59/59 viewer tests
unmodified). No viewer rework needed or performed.

### H3 — USB_ONLY classification — **PASS**

```text
BNO085 init=OK       detected=ONLINE      expected=REQUIRED result=PASS
DALY   init=OK       detected=NO_RESPONSE expected=OFFLINE  result=PASS
SERVO  init=OK       detected=UNKNOWN     expected=OFFLINE  result=PASS   (before first scan)
SERVO  init=OK       detected=NO_RESPONSE expected=OFFLINE  result=PASS   (after @SERVO SCAN, found=0)
LED    init=DEFERRED detected=UNPOWERED   expected=UNPOWERED result=PASS
```

Captured live from `@STATUS`/`@SERVO SCAN`, matching the required semantics
exactly: `SERVO` reports `UNKNOWN` (not yet probed) until an explicit scan
runs, never `ONLINE`/`OK` merely because the UART transport initialized.

### H4 — LED anti-back-power — **PASS**

```text
LED_TEST=BLOCKED
REASON=LED_RAIL_UNPOWERED
PROFILE=USB_ONLY

LED_OFF=NOOP
REASON=LED_RAIL_UNPOWERED (nothing is ever driven in this profile)

LED    init=DEFERRED detected=UNPOWERED expected=UNPOWERED result=PASS
  pixels=12 pin=47 brightness_max=60 test_running=NO data_pin_driven=NO
```

`data_pin_driven=NO` confirmed **after** attempting `@LED TEST` and `@LED OFF`
— no WS2812 frame was transmitted this session. Not verified optically (5V
absent, out of scope); verified at the firmware level, which is what this
finding was about.

### H5 — Servo async scan — **PASS**

```text
SERVO_SCAN=STARTED lo=11 hi=55          <- immediate reply
  ... ~4.5s elapsed, BNO085 RV count advanced 932 -> 939 during this window,
      IMU_STATUS confirmed runtime_resets=0 immediately after ...
SERVO_SCAN=COMPLETE lo=11 hi=55 found=0
```

`found=0` is correct with servos unpowered. RV telemetry (dumped from the raw
capture) shows continuous, gap-free `count` increments spanning the entire
scan window — the scan did not stall BNO085 acquisition or the USB command
router, the concrete behavioural goal of Finding 5.

### H6 — Soak — **PASS**

```text
duration              : ~100 s, BMS streaming enabled, one @SERVO SCAN mid-soak
runtime_resets         : 0
heap_free  @ start     : 337280 bytes free / 331992 min-free
heap_free  @ end        : 337280 bytes free / 331992 min-free   (zero drift)
RV COUNTS               : 1652 -> 6915 (continuous, no gaps)
DALY poll cycles         : 56 (~1 every 1.8-2.0s, as designed)
fatal/panic/guru meditation : none
mid-soak servo scan      : SERVO_SCAN=COMPLETE found=0, no disruption
```

### Frozen/standalone source hashes — re-verified unchanged

```text
matdog_servo_commissioning.ino     74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd
matdog_bno085_dcd_phase_c3.ino     51bb3016ac20226b812e1e892328768495348278cb19c320de34930cccd103e6
matdog_daly_rs485_probe_v2.ino     4fd7fd3982ab57376ad9a5ade7ed497fb2515234465612a59aac22f756acb81c
```

### Acceptance — Session 2 completion checklist

- [x] flash bootloader/partition audit completed
- [x] differences vs backup documented (none found)
- [x] `upload.sh` documentation corrected; `flash_app_only.sh` is genuinely application-only
- [x] `SOURCE_COMMIT` precise (`04dfa52d1b5ab37ac4099792bb8c874c5ee4842a`, full 40-char)
- [x] application binary SHA256 registered
- [x] build from a clean working tree (gated by `flash_app_only.sh` itself)
- [x] LED GPIO47 not driven in USB_ONLY
- [x] `@LED TEST` blocked in USB_ONLY
- [x] health model separates initialized/detected/expected/result
- [x] servo scan does not block the main loop
- [x] static audit extended and PASS
- [x] compile PASS
- [x] viewer tests PASS
- [x] application-only flash PASS (write-time hash + independent verify-flash + non-application-region re-verification)
- [x] boot PASS
- [x] BNO085 live PASS
- [x] offline classification PASS
- [x] short soak PASS
- [x] original frozen source hashes unchanged
- [x] documentation coherent (this section; Session 1 evidence preserved, not deleted)
- [x] git working tree clean

### Open items carried forward (not in scope for Session 2)

```text
DALY live telemetry / MOS / faults    — battery still not connected
DALY writes                            — protocol still not verified
LED optical output                     — 5V rail still absent
ST3215 responses/telemetry/torque/motion — servo power still absent
full powered-system coexistence         — requires battery + 5V
KEY / P- transitions                    — requires battery
software self-power-cut                 — DALY write still blocked by design
```

### Judgement

```text
READY_FOR_REVIEW_BEFORE_MAIN_MERGE
```

All Session 2 hardening findings are fixed, tested and hardware-validated under
the same USB_ONLY bench profile as Session 1, with no regression to previously
validated behaviour (BNO085 SPI/RV/MAG/GYR/SAVE_GATE, viewer protocol, USB CDC,
DALY non-blocking polling, ServoBus/DalyBms pin/baud mapping, LED GPIO47
mapping, power-state architecture) and frozen sources re-confirmed unchanged.
Not merged to `main` — that remains a separate, explicit decision.

---

## Session 2.1 — 2026-09-15, Final Pre-Merge Hardening

An independent review of Session 2 raised two findings before merge. Same
USB_ONLY hardware configuration throughout; no new functionality.

> **Documentation correction recorded in Session 2.2:** the chat-only final
> report at the end of this session stated "10 commits" created; the actual
> number is **5** (`60e3097`, `6fb654c`, `f184881`, `07592b9`, `647dba8`,
> all after `82e0aa7`). That report text was never written to this file or
> any other, so there was nothing to correct in-repo — this note exists
> only so the miscount is not repeated. Not a firmware finding.

### Provenance

```text
FIRMWARE_SOURCE_COMMIT = 07592b9d7f284eb6a24d18d69b57351281676e14
  (clean tree; compiled, application-only flashed and hardware-validated
   this session — build id 07592b9d7f28 visible in the boot banner/@STATUS)
FINAL_BRANCH_HEAD       = <the commit that adds this section plus the
   accompanying README/CHANGELOG updates — created AFTER
   FIRMWARE_SOURCE_COMMIT and after the device was already flashed and
   validated; changes nothing that is running on the device>
```

### Review finding 1 — the Session 2 servo scan was not truly non-blocking

Accurate: `ServoBus::update()` still calls `SCServo::Ping()` synchronously,
and that call carries its own bounded per-call timeout — one `update()` tick
can still block for up to that long. "Non-blocking" was imprecise. Fixed by:

1. **Terminology corrected** everywhere it was live documentation (not a
   historical Session 2 record, which is preserved with a correction note
   instead — see Finding 5 above): "incremental scan with bounded per-ID
   blocking", not "non-blocking". Source comments, README, static audit
   messages, USB diagnostic text (`@SERVO SCAN` help line) all updated.
2. **Architectural boundary introduced**: `core::OperatingMode`
   (`MAINTENANCE`/`RUN`) — deliberately just an enum and a two-line manager
   class. `@SERVO SCAN`/`@SERVO READ` now refuse outside `MAINTENANCE`
   (`SERVO_SCAN=BLOCKED`/`REASON=NOT_IN_MAINTENANCE_MODE`). `@SERVO SAFE_OFF`
   is deliberately **not** gated — it can only remove torque and must stay
   reachable in every mode as the safety de-escalation path. Default is
   `MAINTENANCE` (no motion loop exists yet to protect); whoever adds the
   motion controller must flip the default to `RUN` and require an explicit,
   reviewed transition into `MAINTENANCE` first — documented directly in
   `OperatingMode.h` so it cannot be missed.
3. **`IOTimeOut` audited, not assumed, and reduced with cited evidence**:
   `SCSerial::IOTimeOut` (`SCSerial.h`) is a public field, default 100ms
   (`SCSerial.cpp`, three constructors) — a generic library-wide margin, not
   an ST3215-specific figure. Real hardware measurement exists for this
   exact servo/bus combination: the NEW01 characterization campaign
   (`09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/characterization_sessions/`,
   firmware `matdog_st3215_characterize_v1.ino`) timed a live powered ST3215
   at 1 Mbaud with `micros()` and recorded Ping+register-read round trips
   clustering at 593-620us (hundreds of samples) and a dedicated
   register-read timing of 334-358us. `ServoBus::kPingTimeoutMs = 20`
   (milliseconds) is set at runtime in `begin()` — the vendored SCServo
   library file is never edited — chosen as roughly 32x that measured worst
   case: enough margin for this session's different bus topology (through
   the Seeed driver, not the characterization rig's point-to-point wiring)
   while cutting the USB_ONLY no-response worst case 5x per ID versus the
   100ms library default.
4. **Real behaviour measured, not asserted**: `ScanResult` gained
   `elapsed_ms`/`max_ping_us`, computed with `micros()`/`millis()` inside
   `ServoBus::update()` and reported in `SERVO_SCAN=COMPLETE`.

**Measured evidence (real hardware, this session, servos unpowered as
designed):**

```text
SERVO_SCAN=COMPLETE lo=11 hi=55 found=0 elapsed_ms=1218 max_ping_us=20815
```

- Full 45-ID scan (the canonical allocation range): **1218 ms total**
  (vs. Session 2's ~4.5s estimate at the 100ms default — ~3.7x faster).
- **Single slowest probe: 20.815ms** — consistent with the configured 20ms
  `kPingTimeoutMs` (small overhead above the nominal timeout is UART framing,
  not unexpected).

**RV rate: baseline vs. during scan** (both measured from real `COUNTS`
telemetry, not inferred from "RV kept advancing"):

```text
baseline (no scan running)   : ~48.8 Hz  (708 RV samples / ~14.5s)
during a scan (scan overlaps
  the middle of the capture)  : ~47.3 Hz  (662 RV samples / ~14.0s)
```

The ~3% difference is within normal sample-window jitter, not a measurable
stall. **This does not mean BNO085 acquisition is guaranteed unaffected
during MAINTENANCE-mode scanning under different conditions** — it means
that under this session's real measurement, the degradation was small and,
more importantly, **is now architecturally confined to `MAINTENANCE` and
explicitly not permitted to coexist with a future motion `RUN` loop**,
which is the actual requirement this finding asked for.

**USB behaviour during scan:** the command router remained responsive
throughout every scan performed this session (immediate `SERVO_SCAN=STARTED`
reply, subsequent commands like `@STATUS` handled normally mid-scan in
earlier interactive tests) — no lockup, no dropped input observed.

**Which servo commands are gated:**

```text
MAINTENANCE mode required : @SERVO SCAN, @SERVO READ
Always allowed (any mode)  : @SERVO SAFE_OFF  (torque-off only, never adds risk)
```

Live confirmation, `@MODE RUN` then each command:

```text
MODE=RUN
SERVO_SCAN=BLOCKED
REASON=NOT_IN_MAINTENANCE_MODE
MODE=RUN

MODE=RUN
SERVO_READ=BLOCKED
REASON=NOT_IN_MAINTENANCE_MODE
MODE=RUN

MODE=RUN
SERVO_SAFE_OFF id=11 result=OK          <- still allowed in RUN

MODE=MAINTENANCE                          <- switched back
SYSTEM health=READY power_state=RUN mode=MAINTENANCE uptime_ms=59840 profile=USB_ONLY
```

### Review finding 2 — OTA slot selection did not match the real bootloader

Session 2's `verify_application_partition.py` picked whichever otadata
sector had the numerically higher `ota_seq` among any non-blank sector, and
mislabeled the struct's `ota_state` field as `crc` — it never validated the
real CRC field at all. On the real device this session
(`OTADATA_SECTOR_SEQS=[1, 0]`) that happened to produce the correct answer,
but the algorithm itself could in principle have selected a slot the actual
bootloader would refuse to boot.

**Ground truth, verified against the exact installed framework, not
memory or generic examples:** the arduino-esp32 3.3.11 core bundles
ESP-IDF **v5.5.5, commit `b774170ff46`** (`versions.txt` in the installed
esp32-libs package). The real `esp_ota_select_entry_t` struct
(`esp_flash_partitions.h`) is 32 bytes: `ota_seq`(4) + `seq_label`(20) +
**`ota_state`(4)** + `crc`(4) — Session 2's parser read the `ota_state`
offset and called it `crc`; the real `crc` field is 4 bytes further in.
The real selection algorithm
(`bootloader_support/src/bootloader_common_loader.c`,
`bootloader_common_ota_select_crc`/`_invalid`/`_valid`/`get_active_otadata`/
`select_otadata`, and the seq→slot mapping in `bootloader_utility.c`):

```text
crc(entry)     = crc32(entry.ota_seq_bytes, init=0xFFFFFFFF)
invalid(entry) = entry.ota_seq == 0xFFFFFFFF
                 or entry.ota_state in {INVALID(3), ABORTED(4)}
valid(entry)   = not invalid(entry) and entry.crc == crc(entry)

both entries invalid (ignoring crc) -> no deterministic answer -> REFUSE
  (real bootloader falls back to a factory partition, which this table
   has none of, or a first-boot ota_0-init path)
both valid    -> pick larger ota_seq (tie -> index 0)
one valid     -> pick that one
neither valid -> REFUSE (ambiguous)

slot_index = (winning_entry.ota_seq - 1) % ota_app_count
```

The CRC32 variant (`esp_rom_crc32_le` semantics) was **verified
empirically**, not assumed: the stored `crc` field of both otadata sectors
in the real, known-good `boot_app0.bin` seed file shipped with this exact
core matches `zlib.crc32(ota_seq_bytes, 0xFFFFFFFF)` (Python's `zlib.crc32`
called with a custom starting value, no additional inversion) exactly —
confirmed against real bytes, four other candidate CRC formulations
rejected by the same test.

**Fix:** logic split into `scripts/ota_partition_logic.py` (pure, no device
I/O, fully offline-testable) implementing the algorithm above exactly, and
`scripts/verify_application_partition.py` (thin device-I/O wrapper).
Fails closed — raises `OtaAmbiguous`, refuses to write — on every case the
real bootloader does not deterministically resolve.

**Offline test suite** (`scripts/tests/test_ota_partition_logic.py`, no
hardware, no flash writes): **10/10 PASS**, covering every required
scenario:

| Test | Result |
|---|---|
| Current real device state (`[1, 0]`, matches actual hardware) | PASS — resolves to `app0 @ 0x10000` |
| Slot 0 valid, slot 1 blank | PASS — `app0` |
| Slot 1 valid, slot 0 blank | PASS — `app1` |
| CRC-invalid entry with a **higher raw seq** than the valid one (the exact Session 2 regression case) | PASS — correctly picks the CRC-valid entry, ignoring the higher-but-corrupt seq |
| `ota_state = INVALID` | PASS — entry excluded |
| `ota_state = ABORTED` | PASS — entry excluded |
| Both entries blank | PASS — `OtaAmbiguous` raised |
| Both entries CRC-invalid (ambiguous) | PASS — `OtaAmbiguous` raised |
| Partition table missing the computed slot | PASS — `OtaAmbiguous` raised |
| No `ota_N` partitions in table at all | PASS — `OtaAmbiguous` raised |

**Re-run read-only against the real device** after the fix — identical
result to Session 2, now backed by the verified algorithm:

```text
OTADATA_SECTOR_SEQS=[1, 0]
OTADATA_ACTIVE_SECTOR=0
OTADATA_ACTIVE_SEQ=1
ACTIVE_OTA_SLOT_INDEX=0
ACTIVE_PARTITION_LABEL=app0
APPLICATION_OFFSET=0x010000
APPLICATION_PARTITION_SIZE=3145728
```

`static_audit.py` now runs this offline test suite as part of the audit
itself (`check_ota_partition_verifier_fail_closed`), so a regression here
fails the same gate `flash_app_only.sh` already requires PASS before any
write — this utility explicitly does **not** replace a future MATDOG OTA
manager; it is a conservative, fail-closed, read-only-verification helper
for the current application-only flashing path only.

### H0 — Offline gates (Session 2.1)

| Gate | Result |
|---|---|
| Git state re-audited: branch, HEAD, Session 2 commit log, frozen-source hashes | PASS — unchanged |
| Compile (pinned FQBN, clean `FIRMWARE_SOURCE_COMMIT`) | **PASS** — 383988 bytes flash (12%), 28232 bytes RAM (9%), zero warnings from MATDOG_Controller sources |
| Static audit (extended) | **PASS** — 23 source files, 0 findings, includes 10/10 OTA parser tests |
| Existing BNO085 viewer test suite | **PASS** — 59/59, unmodified |
| Application binary provenance | `MATDOG_Controller.ino.bin`, 384,128 bytes, SHA256 `158cd25254ef0cfdc4b7e88cb147563214aafc317c5860f3789e228084c1853a`, embeds build id `07592b9d7f28` matching `FIRMWARE_SOURCE_COMMIT` |

### Application-only flash

```text
DEVICE               = /dev/serial/.../usb-Espressif_USB_JTAG_...-if00 (MAC 14:c1:9f:22:75:94)
APPLICATION_BINARY   = build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
APPLICATION_SHA256   = 158cd25254ef0cfdc4b7e88cb147563214aafc317c5860f3789e228084c1853a
APPLICATION_OFFSET   = 0x010000 (partition 'app0' / ota_0, re-verified with the corrected algorithm)
APPLICATION_SIZE     = 384128 bytes
MAX_PARTITION_SIZE   = 3145728 bytes
SOURCE_COMMIT        = 07592b9d7f284eb6a24d18d69b57351281676e14
```

Write result: esptool's post-write hash check passed ("Hash of data
verified"); independent `esptool verify-flash` reported "Verification
successful (digest matched)". **Non-application regions re-verified
unchanged** after this write via `esptool verify-flash` against the exact
reference files captured during Session 2's original audit: bootloader
(0x0), partition table (0x8000) and otadata (0xe000) all reported
"Verification successful (digest matched)".

### H1 — Boot — **PASS**

```text
build        : 07592b9d7f28
partition    : app0 @ 0x010000 (size 0x300000)
operating_mode : MAINTENANCE
reset_reason : OTHER
IMU_INIT=PASS
SYSTEM_BOOT_COMPLETE health=READY power_state=RUN
EXPECTED_STARTUP_RESET=YES
```

### H2 — BNO085 — **PASS**

`runtime_resets=0` across every capture this session, including through
mode switches and a servo scan. See RV baseline-vs-scan measurement above.

### H3 — USB_ONLY classification — **PASS**, unchanged from Session 2

```text
BNO085 init=OK       detected=ONLINE      expected=REQUIRED  result=PASS
DALY   init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
SERVO  init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS   (after a scan)
LED    init=DEFERRED detected=UNPOWERED   expected=UNPOWERED result=PASS
```

### H4 — LED anti-back-power — **PASS**, unchanged from Session 2

```text
LED_TEST=BLOCKED
REASON=LED_RAIL_UNPOWERED
```

Re-confirmed after all Session 2.1 changes — `data_pin_driven` behaviour
untouched by this session's work.

### H6 — Soak — **PASS**

Extended, interactive soak across the whole session (MAINTENANCE/RUN mode
switches, a servo scan, repeated `@STATUS`): `runtime_resets=0` throughout,
including a final dedicated 55s window immediately before closing out the
session. No fatal/panic/watchdog events observed.

### Frozen/standalone source hashes — re-verified unchanged

```text
matdog_servo_commissioning.ino     74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd
matdog_bno085_dcd_phase_c3.ino     51bb3016ac20226b812e1e892328768495348278cb19c320de34930cccd103e6
matdog_daly_rs485_probe_v2.ino     4fd7fd3982ab57376ad9a5ade7ed497fb2515234465612a59aac22f756acb81c
```

### Session 2.1 completion checklist

- [x] blocking nature of `SCServo::Ping()` measured and documented (max 20.815ms single-probe, 1218ms full 45-ID scan)
- [x] no documentation calls the scan falsely non-blocking (Session 2 record preserved with a correction note, not rewritten)
- [x] potentially-blocking servo diagnostics confined to `MAINTENANCE`, refused in `RUN`
- [x] SCServo timeout audited from real library source (public field, 100ms default) and reduced with cited hardware evidence, not assumed
- [x] baseline vs. during-scan BNO085 RV rate documented (~48.8Hz vs ~47.3Hz)
- [x] OTA partition selection follows the real installed bootloader algorithm (verified against ESP-IDF v5.5.5 source) or REFUSEs
- [x] OTA parser has fail-closed offline tests — 10/10 PASS
- [x] static audit PASS (includes the above as gates)
- [x] viewer tests PASS (59/59, unmodified)
- [x] frozen source hashes unchanged
- [x] git working tree clean
- [ ] merge to `main` — explicitly NOT done this session

### Judgement

```text
READY_FOR_MAIN_MERGE
```

Both review findings are fixed, tested (offline where possible, hardware
where required) and documented without deleting or silently rewriting
Session 2's evidence. No new functionality was added; no forbidden hardware
action (battery, 5V, torque-on, GoalPosition, EEPROM, DALY write, MOS write,
bootloader/partition-table write, NVS erase, full-image upload) occurred
this session. Merge decision and execution remain with the operator.

---

## Session 2.2 — 2026-09-15, Final Merge Gate

Four findings from a final review of Session 2.1, fixed before merge — no
new functionality. Same USB_ONLY hardware configuration throughout.

### Provenance

```text
FIRMWARE_SOURCE_COMMIT = cb53c63206b0ccad68055ecc993a0a9e03f5b545
  (clean tree; compiled, application-only flashed and hardware-validated
   this session — build id cb53c63206b0 visible in the boot banner/@STATUS)
FINAL_BRANCH_HEAD       = <the commit that adds this section plus the
   accompanying README/CHANGELOG updates — created AFTER
   FIRMWARE_SOURCE_COMMIT and after the device was already flashed and
   validated; changes nothing that is running on the device>
```

### Finding A — OTA subtype filter was too permissive

`ota_app_partitions()` used `subtype >= 0x10`. The same
`esp_flash_partitions.h` header already cited in Session 2.1 also defines
`PART_SUBTYPE_TEST = 0x20` and `PART_SUBTYPE_TEE_0/1 = 0x30/0x31` — all
numerically `>= 0x10` but not OTA app slots. Fixed to the real ESP-IDF
bitmask: `(subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG(0x10)`, slot index
`= subtype & PART_SUBTYPE_OTA_MASK(0x0F)`. Also now refuses on a duplicate
OTA slot index instead of silently picking one. 5 new offline tests added
(ota_0+ota_1 recognized; TEST alone does not add a third slot; TEE_0/1
excluded; TEST-and-TEE-with-no-OTA-slot correctly finds none; duplicate
slot index refuses) — all PASS.

### Finding B — rollback configuration was never checked against the real build

Verified, not assumed, by reading the actual build's `sdkconfig`
(`build/esp32.esp32.esp32s3/sdkconfig`):

```text
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK is not set
```

Rollback **is** enabled in the real, installed toolchain — this was not
assumed to be off. With it enabled, `bootloader_utility.c`'s
`get_selected_boot_partition()` autonomously rewrites otadata during normal
boot: any sector in `ESP_OTA_IMG_PENDING_VERIFY` is marked `ABORTED`
unconditionally at the start of every boot; the winning sector is marked
`PENDING_VERIFY` if its prior state was `NEW`. A read-only single snapshot
cannot treat those two states as stable.

`resolve_application_partition()` now takes required (no-default)
`rollback_enabled`/`anti_rollback_enabled` parameters — a caller cannot
silently assume "safe". Policy implemented exactly as specified:

```text
anti_rollback enabled              -> REFUSE unconditionally (secure_version/
                                       eFuse semantics this tool does not implement)
rollback enabled + NEW/PENDING_VERIFY selected -> REFUSE (bootloader can
                                       rewrite this on the next boot)
rollback enabled + UNDEFINED/VALID selected     -> proceed (bootloader
                                       never autonomously rewrites these)
both disabled                      -> proceeds exactly as Session 2.1
```

This is a read-only, minimal extension of the existing validity check — not
an OTA manager, no write path, no confirmation/rollback-cancel logic added.
`verify_application_partition.py` now requires `--sdkconfig` and reads the
real flags from the build that produced the binary being flashed;
`flash_app_only.sh` passes it automatically and prints both flags before
writing anything. `static_audit.py` forbids the parameters from ever
regaining a default value and requires `--sdkconfig` to remain wired in.

**The real device's actual state, re-verified this session:**
`OTADATA_SECTOR_SEQS=[1, 0]`, active entry `ota_state=UNDEFINED` — not
`NEW`/`PENDING_VERIFY`, so stable regardless of rollback being enabled.
`flash_app_only.sh` printed `SDKCONFIG_ROLLBACK = ...=y` /
`SDKCONFIG_ANTI_ROLLBACK = ...=n` and proceeded correctly; resolved
`app0 @ 0x010000`, identical to Session 2.1. 9 new offline tests cover
every enabled/disabled × state combination — all PASS.

### Finding C — the 20ms diagnostic timeout had become a standing global override

Session 2.1 set `st_.IOTimeOut = kPingTimeoutMs` once in `ServoBus::begin()`
— every subsequent SCServo call for the rest of the session, including any
future register-read/telemetry call unrelated to `MAINTENANCE`-mode
scanning, would silently inherit 20ms whether appropriate or not. `20ms` is
a diagnostic absence-detection timeout justified by real measurement; it
was never validated as an operational timeout for a future powered
17-servo bus, and was not supposed to become one by default.

Fixed with `ServoBus::ScopedPingTimeout`, a small RAII guard: saves the
current `IOTimeOut`, sets the diagnostic value, restores the saved value in
its destructor (fires on every exit path, including early `return`).
`begin()` no longer touches `IOTimeOut` at all — it stays at the library's
own 100ms default except for the exact duration of a diagnostic
transaction. Applied around every bus transaction: `ping()`, the scan's
per-ID probe, `readModel()`, `safeOff()`, `readRuntimeState()`. No vendored
SCServo file was edited — `IOTimeOut` is a public `SCSerial` field.

### Finding D — SAFE_OFF could report false success

**Root cause, confirmed by reading the real, installed SCServo library**
(`SCServo/src/SCS.cpp`): `EnableTorque()`/`writeByte()` return
`SCS::Ack()`'s result, which is **0 on any failure/timeout/no-response and
1 on a validated ACK — never negative**, unlike `Ping()`/`readByte()`/
`readWord()` (`-1` on failure, documented in-source: "timeout returns
-1"). Session 2.1's `safeOff()` checked `result >= 0`, which is true for
**both** possible outcomes of `Ack()` (0 and 1) — it could never observe a
failure, regardless of whether a servo was even present.

Fixed by never trusting the write's own return value for the safety claim:
`safeOff()` issues the torque-off write, then **always** performs an
independent, read-only `TorqueEnable` readback regardless of what the
write's ACK reported, and classifies strictly from that readback:

```text
VERIFIED_OFF            readback responded, TorqueEnable == 0
UNVERIFIED_NO_RESPONSE  no readback response at all
VERIFY_FAILED           readback responded, TorqueEnable != 0
```

No bare bool, no "OK" reply anywhere in this path.

**Live confirmation, servo bus completely unpowered (USB_ONLY, as this
whole session's hardware configuration requires):**

```text
SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE
SERVO_SAFE_OFF id=99 result=UNVERIFIED_NO_RESPONSE
```

— correctly no longer `OK`/`VERIFIED_OFF`. Re-confirmed with
`OperatingMode::RUN` active (`@SERVO SAFE_OFF` remains reachable there by
design, honestly classified either way):

```text
MODE=RUN
SERVO_SCAN=BLOCKED
REASON=NOT_IN_MAINTENANCE_MODE
MODE=RUN
SERVO_READ=BLOCKED
REASON=NOT_IN_MAINTENANCE_MODE
MODE=RUN
SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE
MODE=MAINTENANCE
SYSTEM health=READY power_state=RUN mode=MAINTENANCE uptime_ms=52396 profile=USB_ONLY
```

### Section 6 — OperatingMode vs PowerState clarified, not redesigned

`core::OperatingMode`'s approved `MAINTENANCE`/`RUN` boundary is unchanged
(the handoff explicitly asked for clarification, not a redesign).
`OperatingMode.h` and `PowerState.h` now cross-reference each other: the
two `RUN` values are orthogonal and the shared name is coincidental — a
running V0.1 controller is normally `PowerState::RUN` **and**
`OperatingMode::MAINTENANCE` simultaneously (visible together in every
`@STATUS`/boot banner capture this session), which is expected, not a
naming collision.

### H0 — Offline gates (Session 2.2)

| Gate | Result |
|---|---|
| Git state re-audited: branch, HEAD (`647dba87cbfd654230ab13f301f8708c41771222`, matched exactly), frozen-source hashes | PASS — unchanged |
| Real sdkconfig read (not assumed) | `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, `CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK` not set |
| Compile (pinned FQBN, clean `FIRMWARE_SOURCE_COMMIT`) | **PASS** — 384248 bytes flash (12%), 28232 bytes RAM (9%), zero warnings from MATDOG_Controller sources |
| Static audit (extended) | **PASS** — 23 source files, 0 findings, includes 27/27 OTA parser tests (10 Session 2.1 baseline + 17 new this session; see totals below) |
| Existing BNO085 viewer test suite | **PASS** — 59/59, unmodified |
| Application binary provenance | `MATDOG_Controller.ino.bin`, 384,400 bytes, SHA256 `9cae184d65c46cf0e9c1735e69843affd4a9bdd4199282c097c16df13aa678b3`, embeds build id `cb53c63206b0` matching `FIRMWARE_SOURCE_COMMIT` |

### Application-only flash

```text
SDKCONFIG_ROLLBACK      = CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
SDKCONFIG_ANTI_ROLLBACK = CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=n
DEVICE                  = /dev/serial/.../usb-Espressif_USB_JTAG_...-if00 (MAC 14:c1:9f:22:75:94)
APPLICATION_BINARY      = build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
APPLICATION_SHA256      = 9cae184d65c46cf0e9c1735e69843affd4a9bdd4199282c097c16df13aa678b3
APPLICATION_OFFSET      = 0x010000 (partition 'app0', re-verified with the rollback-aware algorithm)
APPLICATION_SIZE        = 384400 bytes
MAX_PARTITION_SIZE      = 3145728 bytes
SOURCE_COMMIT           = cb53c63206b0ccad68055ecc993a0a9e03f5b545
```

Write result: esptool's post-write hash check passed ("Hash of data
verified"); independent `esptool verify-flash` reported "Verification
successful (digest matched)". **Non-application regions re-verified
unchanged** via `esptool verify-flash` against the same reference files
captured in Session 2's original audit: bootloader (0x0), partition table
(0x8000) and otadata (0xe000) all reported "Verification successful
(digest matched)".

### H1 — Boot — **PASS**

```text
build          : cb53c63206b0
partition      : app0 @ 0x010000 (size 0x300000)
operating_mode : MAINTENANCE
IMU_INIT=PASS
SYSTEM_BOOT_COMPLETE health=READY power_state=RUN
EXPECTED_STARTUP_RESET=YES
```

### H2 — BNO085 — **PASS**

`runtime_resets=0` across every capture this session.

### H3 — USB_ONLY classification — **PASS**, unchanged from Session 2.1

```text
BNO085 init=OK       detected=ONLINE      expected=REQUIRED  result=PASS
DALY   init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
SERVO  init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
LED    init=DEFERRED detected=UNPOWERED   expected=UNPOWERED result=PASS
```

### H4/H5 — SAFE_OFF and OperatingMode regression — **PASS**

See Finding D above for the full live transcript. Summary:

```text
@SERVO SAFE_OFF 11 (servo bus unpowered)      -> UNVERIFIED_NO_RESPONSE (was: OK)
@MODE RUN; @SERVO SCAN                        -> BLOCKED / NOT_IN_MAINTENANCE_MODE
@MODE RUN; @SERVO READ                        -> BLOCKED / NOT_IN_MAINTENANCE_MODE
@MODE RUN; @SERVO SAFE_OFF                    -> still allowed, honestly classified
@MODE MAINTENANCE                             -> switches back correctly
```

### H6 — Soak — **PASS**

~45s dedicated window post-flash plus the interactive validation above:
`runtime_resets=0` throughout, classification stable, no fatal/panic events.

### Frozen/standalone source hashes — re-verified unchanged

```text
matdog_servo_commissioning.ino     74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd
matdog_bno085_dcd_phase_c3.ino     51bb3016ac20226b812e1e892328768495348278cb19c320de34930cccd103e6
matdog_daly_rs485_probe_v2.ino     4fd7fd3982ab57376ad9a5ade7ed497fb2515234465612a59aac22f756acb81c
```

### OTA parser test totals

```text
Session 2.1 baseline                        10 tests
Session 2.2 Finding A (subtype recognition)   6 tests
Session 2.2 Finding B (rollback awareness)    7 tests
Session 2.2 sdkconfig flag parsing            4 tests
                                              ----
TOTAL                                        27 tests, 27 PASS
```

### Session 2.2 completion checklist

- [x] OTA subtype recognition uses the real bitmask; TEST/TEE never counted as OTA slots
- [x] new OTA subtype tests PASS (6/6)
- [x] real build's rollback/anti-rollback config verified (not assumed): rollback ON, anti-rollback OFF
- [x] flasher REFUSEs on anti-rollback-enabled or rollback-enabled-with-unstable-state (11/11 rollback+parsing tests PASS)
- [x] 20ms confined to diagnostic transactions via `ScopedPingTimeout`; `begin()` no longer sets it globally
- [x] operational (non-diagnostic) timeout unchanged from the library's own 100ms default
- [x] SAFE_OFF can no longer report verified success on an absent servo — confirmed live (`UNVERIFIED_NO_RESPONSE`)
- [x] USB_ONLY SAFE_OFF test returns an honest result
- [x] SCAN/READ remain blocked in RUN (re-confirmed live)
- [x] SAFE_OFF remains available in RUN (re-confirmed live, honestly classified)
- [x] compile PASS
- [x] static audit PASS (23 files, 0 findings, includes 27/27 OTA tests)
- [x] viewer 59/59 PASS
- [x] frozen hashes unchanged
- [x] application-only flash PASS
- [x] non-application flash regions unchanged (bootloader/partition-table/otadata all re-verified)
- [x] git working tree clean
- [ ] merge to `main` — explicitly NOT done this session

### Judgement

```text
READY_FOR_MAIN_MERGE
```

All four findings are fixed, hardware-validated where the fix touches
runtime behaviour (Findings C and D), offline-tested where the fix is pure
host-side logic (Findings A and B), and documented without deleting or
rewriting Session 2/2.1's evidence. No new functionality was added; no
forbidden hardware action occurred this session. Merge decision and
execution remain with the operator.

## Session 2.3 — 2026-09-15, Final Consistency Fix

Three consistency findings from a final review of Session 2.2, fixed before
merge — no new functionality, no gait/IK/motion/Wi-Fi/OTA-manager/
DALY-write/servo-calibration work. Same USB_ONLY hardware configuration
throughout.

### Provenance

```text
FIRMWARE_SOURCE_COMMIT (before this session) = cb53c63206b0ccad68055ecc993a0a9e03f5b545
FINAL_BRANCH_HEAD (before this session)      = 5f36afb9c0023147273b09c6d30190cfe19334f7
FIRMWARE_SOURCE_COMMIT (this session)        = 5b371da5482f9b0bd2df1c37ed361250ea54ae8f
  (clean tree; compiled, application-only flashed and hardware-validated
   this session — build id 5b371da5482f visible in the boot banner/@STATUS)
```

Four commits created this session on `feat/matdog-controller-v01`, none of
which is a merge and none of which touches `main`:

```text
816cda3  fix(controller): split servo bus timeout into diagnostic/operational categories
f962b12  fix(controller): fail-closed sdkconfig tri-state + contiguous OTA slot requirement
60a818d  test(controller): extend OTA offline suite for Session 2.3 findings
5b371da  test(controller): extend static audit for Session 2.3 findings
```

`FIRMWARE_SOURCE_COMMIT = 5b371da5482f9b0bd2df1c37ed361250ea54ae8f` is the
last of these four — the docs commit that records this section is
necessarily later still and changes nothing that is running on the device.

### Finding 1 — the diagnostic/operational timeout split was not actually true in code

Session 2.2 Finding C scoped the 20ms diagnostic timeout to a per-transaction
RAII guard (`ScopedPingTimeout`) instead of a standing global override — a
real fix, but it applied that **same** 20ms value to every transaction,
including `safeOff()` and `readRuntimeState()`. The handoff's own stated
policy ("operational timeout = 100ms, diagnostic timeout = 20ms") was
documented but not actually implemented: nothing in the code used a 100ms
timeout anywhere.

Fixed by splitting into two named constants and requiring every call site to
name its choice explicitly — there is no longer an implicit "whatever it's
currently set to" case:

```text
kDiagnosticTimeoutMs = 20   ping(), readModel(), the scan's per-ID probe —
                             bounded absence-detection only, MAINTENANCE-only today
kOperationalTimeoutMs = 100 safeOff() (the safety de-escalation write, reachable
                             from any OperatingMode) and readRuntimeState() (the
                             primitive behind @SERVO READ today, which a future
                             motion controller will naturally reuse)
```

`kOperationalTimeoutMs` is simply SCServo's own untouched library default
(100ms) here — not a value MATDOG has validated/optimized as final for a
powered 17-servo bus, and the handoff was explicit that it must not be tied
to the diagnostic constant. `ScopedPingTimeout` is renamed `ScopedIOTimeout`
(same mechanics — saves/restores `SCSerial::IOTimeOut` on every exit path,
non-copyable/non-movable) to reflect that it now guards two categories, not
one. No vendored SCServo file was touched.

**Live confirmation, servo bus completely unpowered** — `safeOff()` now
performs an `EnableTorque` write bounded by `kOperationalTimeoutMs` (no
ACK, unpowered) followed by an independent `TorqueEnable` readback also
bounded by `kOperationalTimeoutMs` (no response) — five back-to-back trials:

```text
trial 1: SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE  elapsed=204.71 ms
trial 2: SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE  elapsed=209.99 ms
trial 3: SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE  elapsed=207.07 ms
trial 4: SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE  elapsed=208.33 ms
trial 5: SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE  elapsed=204.31 ms
```

~205-210ms consistently — evidence, not a claim: this is the two
now-conservative 100ms operational-timeout waits (write + readback) plus
USB-serial round-trip overhead, exactly as expected from a bus with nothing
attached. Never `VERIFIED_OFF`, never a bare `OK`.

### Finding 2 — an absent sdkconfig symbol was silently treated as disabled

`parse_sdkconfig_ota_flags()` returned `False` both when it found
`# CONFIG_X is not set` (genuinely, positively disabled) and when it found
neither that line nor `CONFIG_X=y` at all (Kconfig produced no opinion this
tool could read). Both cases collapsed to the same boolean — not
fail-closed, since a silently-missing symbol was treated exactly like a
confirmed-safe one. Proven by the test this session removes,
`test_flags_absent_entirely_defaults_to_disabled`, which asserted precisely
that conflation as correct behaviour.

`parse_sdkconfig_flag()` now returns a three-way `SdkconfigFlag`
(`ENABLED`/`DISABLED`/`UNKNOWN`); `resolve_application_partition()` REFUSEs
on `UNKNOWN` for either symbol, with the same REFUSE wording style as every
other check in this module:

```text
REFUSE: sdkconfig does not explicitly define CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
        (neither 'CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y' nor
        '# CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is not set' found) — cannot
        confirm otadata state stability. Refusing; see handoff Session 2.3 Finding 2.
```

**The real device's actual sdkconfig, re-read this session (not assumed)**:

```text
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=ENABLED
CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=DISABLED
```

Both symbols are explicitly present in the real build's sdkconfig (neither
is `UNKNOWN`), so this session's flash proceeded correctly — the tri-state
change is additive fail-closed behaviour, not a regression against the real
configuration. `verify_application_partition.py`'s printed format changed
from `y`/`n` to the enum's `.value` (`ENABLED`/`DISABLED`/`UNKNOWN`);
`flash_app_only.sh` needed no change, since it only greps whatever string
follows `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=`/`CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=`
regardless of format. 6 new dedicated `UNKNOWN` tests plus a corrected
`parse_sdkconfig_ota_flags()` test — all PASS.

### Finding 3 — OTA slot index sets were not required to be contiguous

`ota_app_partitions()` accepted any set of OTA slot indices found in the
partition table, including sparse ones like `{0, 2}` (a `subtype=0x12`
`ota_2` partition with no `ota_1`). MATDOG has no use for a sparse OTA
layout in V0.1; the goal is avoiding ambiguity between raw OTA subtype
numbers and the bootloader's `app_count`/modulo slot-selection logic ahead
of any future OTA subsystem, not supporting sparse tables.

Fixed with a direct set-equality check after the existing duplicate-index
check: `set(result.keys()) != set(range(len(result)))` raises
`OtaAmbiguous`. 7 new offline tests: `{0}`, `{0,1}`, `{0,1,2}` accept;
`{1}`, `{0,2}`, `{1,2}`, `{0,1,3}` REFUSE — all PASS. The real device's
table (`app0`/`ota_0`, `app1`/`ota_1` → `{0,1}`) is unaffected.

### H0 — Offline gates (Session 2.3)

| Gate | Result |
|---|---|
| Git state re-audited: branch, HEAD (`5f36afb9c0023147273b09c6d30190cfe19334f7`, matched exactly), frozen-source hashes | PASS — unchanged |
| Compile (pinned FQBN, clean `FIRMWARE_SOURCE_COMMIT`) | **PASS** — 384252 bytes flash (12%), 28232 bytes RAM (8%), only pre-existing vendored SCServo warnings |
| Static audit (extended) | **PASS** — 23 source files, 0 findings, includes 40/40 OTA parser tests (27 Session 2.2 baseline + 13 new this session; see totals below) |
| OTA parser offline test suite | **PASS** — 40/40 |
| Existing BNO085 viewer test suite | **PASS** — 59/59, unmodified |
| Application binary provenance | `MATDOG_Controller.ino.bin`, 384,400 bytes, SHA256 `6e6d92f898dbe95000b53dbb252c7eb5d3deaa9a4b161e2b1934436a76b29364`, embeds build id `5b371da5482f` matching `FIRMWARE_SOURCE_COMMIT` |

### Application-only flash

```text
SDKCONFIG_ROLLBACK      = CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=ENABLED
SDKCONFIG_ANTI_ROLLBACK = CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=DISABLED
DEVICE                  = /dev/serial/.../usb-Espressif_USB_JTAG_...-if00 (MAC 14:c1:9f:22:75:94)
APPLICATION_BINARY      = build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
APPLICATION_SHA256      = 6e6d92f898dbe95000b53dbb252c7eb5d3deaa9a4b161e2b1934436a76b29364
APPLICATION_OFFSET      = 0x010000 (partition 'app0')
APPLICATION_SIZE        = 384400 bytes
MAX_PARTITION_SIZE      = 3145728 bytes
SOURCE_COMMIT           = 5b371da5482f9b0bd2df1c37ed361250ea54ae8f
```

Write result: esptool's post-write hash check passed ("Hash of data
verified"); independent `esptool verify-flash` reported "Verification
successful (digest matched)". **Non-application regions re-verified
unchanged**: bootloader (0x0, 20,480 B), partition table (0x8000, 4,096 B)
and otadata (0xe000, 8,192 B) were each extracted from the same verified
Session 2 full-flash backup (`matdog_esp32s3_fullflash_2026-09-10.bin`,
re-confirmed this session: size 16,777,216 bytes, SHA256
`5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`) and
independently compared against the live device with `esptool verify-flash`
— all three reported "Verification successful (digest matched)".

### H1 — Boot — **PASS**

```text
build          : 5b371da5482f
board          : YD-ESP32-S3 N16R8
profile        : USB_ONLY
partition      : app0 @ 0x010000 (size 0x300000)
reset_reason   : OTHER
operating_mode : MAINTENANCE
IMU_INIT=PASS
SYSTEM_BOOT_COMPLETE health=READY power_state=RUN
EXPECTED_STARTUP_RESET=YES
```

### H2 — BNO085 — **PASS**

`runtime_resets=0` across every capture this session, including the
dedicated 60s soak below.

### H3 — USB_ONLY classification — **PASS**, unchanged from Session 2.2

```text
SYSTEM health=READY power_state=RUN mode=MAINTENANCE profile=USB_ONLY
BNO085 init=OK       detected=ONLINE      expected=REQUIRED  result=PASS
DALY   init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
SERVO  init=OK       detected=NO_RESPONSE expected=OFFLINE   result=PASS
LED    init=DEFERRED detected=UNPOWERED   expected=UNPOWERED result=PASS
```

### H4/H5 — SAFE_OFF timing and OperatingMode regression — **PASS**

See Finding 1 above for the full five-trial SAFE_OFF timing transcript.
OperatingMode regression, live:

```text
@MODE RUN                    -> MODE=RUN
@SERVO SCAN 11 55             -> SERVO_SCAN=BLOCKED  REASON=NOT_IN_MAINTENANCE_MODE  MODE=RUN
@SERVO READ 11                -> SERVO_READ=BLOCKED  REASON=NOT_IN_MAINTENANCE_MODE  MODE=RUN
@SERVO SAFE_OFF 11            -> SERVO_SAFE_OFF id=11 result=UNVERIFIED_NO_RESPONSE (still allowed)
@MODE MAINTENANCE             -> MODE=MAINTENANCE
```

Identical behaviour to Session 2.2 — this session touched only the
timeout value SAFE_OFF uses internally, never its MAINTENANCE/RUN
reachability.

### H6 — Soak — **PASS**

Dedicated 60s window after the mode-regression test, plus the interactive
validation above: 715 serial lines observed, `runtime_resets=0` throughout
(`COUNTS acc=771 gyr=741 mag=3706 game=3694 rv=3694 runtime_resets=0` at the
end of the window), classification stable, no fatal/panic events.

### Frozen/standalone source hashes — re-verified unchanged

```text
matdog_servo_commissioning.ino     74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd
matdog_bno085_dcd_phase_c3.ino     51bb3016ac20226b812e1e892328768495348278cb19c320de34930cccd103e6
matdog_daly_rs485_probe_v2.ino     4fd7fd3982ab57376ad9a5ade7ed497fb2515234465612a59aac22f756acb81c
```

(all four candidate locations checked, including both copies of the frozen
bench QC source — see `SOURCE_PROVENANCE.md`)

### OTA parser test totals

```text
Session 2.2 baseline                          27 tests
Session 2.3 Finding 2 (sdkconfig UNKNOWN)      6 tests
Session 2.3 Finding 3 (slot contiguity)        7 tests
                                               ----
TOTAL                                         40 tests, 40 PASS
```

### Session 2.3 completion checklist

- [x] `kDiagnosticTimeoutMs`(20ms)/`kOperationalTimeoutMs`(100ms) are two
      separately named constants; every call site states its choice explicitly
- [x] `safeOff()`/`readRuntimeState()` use `kOperationalTimeoutMs`, never `kDiagnosticTimeoutMs`
- [x] `ping()`/`readModel()`/scan's per-ID probe still use `kDiagnosticTimeoutMs`, unchanged
- [x] live SAFE_OFF timing measured with servo unpowered (~205-210ms, 5 trials) — evidence, not a claim
- [x] `parse_sdkconfig_flag()` returns `UNKNOWN` when a symbol is present in neither form
- [x] `resolve_application_partition()` REFUSEs on `UNKNOWN` for both rollback and anti-rollback
- [x] real build's sdkconfig re-verified: `ROLLBACK_ENABLE=ENABLED`, `ANTI_ROLLBACK=DISABLED` — flash proceeded correctly
- [x] `test_flags_absent_entirely_defaults_to_disabled` removed; replaced with `UNKNOWN`-asserting coverage
- [x] OTA slot sets must be contiguous starting at 0; `{1}`/`{0,2}`/`{1,2}`/`{0,1,3}` REFUSE, `{0}`/`{0,1}`/`{0,1,2}` accept
- [x] compile PASS
- [x] static audit PASS (23 files, 0 findings, includes 40/40 OTA tests)
- [x] viewer 59/59 PASS
- [x] frozen hashes unchanged (4/4)
- [x] application-only flash PASS
- [x] non-application flash regions unchanged (bootloader/partition-table/otadata all re-verified against Session 2 baseline)
- [x] boot banner, BNO085 PASS, 0 runtime resets, USB_ONLY classification correct
- [x] OperatingMode regression re-confirmed live (SCAN/READ blocked in RUN, SAFE_OFF allowed)
- [x] 60s soak PASS
- [x] git working tree clean
- [ ] merge to `main` — explicitly NOT done this session

### Judgement

```text
READY_FOR_MAIN_MERGE
```

All three findings are fixed, hardware-validated where the fix touches
runtime behaviour (Finding 1), offline-tested where the fix is pure
host-side logic (Findings 2 and 3), and documented without deleting or
rewriting any prior session's evidence. No new functionality was added; no
forbidden hardware action occurred this session (battery, servo power, LED
5V rail and DALY remained absent/off throughout; only read-only diagnostics
and the pre-approved `EnableTorque(id, 0)` SAFE_OFF write were exercised).
Merge decision and execution remain with the operator.

---

## G2 — ROBOT_POWERED CONFIGURATION SUPPORT — 2026-09-16

Software-preparation gate. This section records an **offline/software** result only.

```text
G2 SOFTWARE / OFFLINE SCOPE     = VALIDATED
ROBOT_POWERED PROFILE           = IMPLEMENTED (compile-tested only)
ROBOT_POWERED HARDWARE OPERATION = TO_TEST

G3 NOT EXECUTED
```

Nothing in this session validates ROBOT_POWERED hardware operation. No external robot
rail was energized at any point and no firmware was flashed. The powered no-motion
validation is gate G3, designed during G2 in
[`G3_ROBOT_POWERED_VALIDATION_PLAN.md`](G3_ROBOT_POWERED_VALIDATION_PLAN.md) and
authorized separately.

### Provenance and entry state

| Item | Value |
|---|---|
| Development branch | `feat/controller-robot-powered-v02` |
| Accepted starting base | `1a8c5bc3ced4f49ea36902526f232b2785a7ab70` |
| `main` / `origin/main` at entry | `1a8c5bc3ced4f49ea36902526f232b2785a7ab70` (unmoved) |
| Immutable V0.1 tag | `matdog-controller-v0.1.0` → `c54862f38a9cbd5e46d6b1770a6d109cc99b5c02` (unchanged) |

Prior gates, accepted as facts and **not rerun** this session:

```text
G0 POST-CLEANUP ENTRY AUDIT   = PASS (not rerun)
G1 V0.1 REGRESSION FREEZE     = PASS (not rerun)
V2 ARCHITECTURE DELTA AUDIT   = PASS
```

### VALIDATED — G2 software/offline scope

Hardware-profile architecture:

- `src/config/HardwareProfile.h` added as the single profile authority: one enum
  (`USB_ONLY` / `ROBOT_POWERED`) and one `expectationsFor()` mapping table.
- `BuildConfig.h` now **derives** `kServoPowerAvailable`, `kBatteryAvailable`,
  `kLedRailPowered` and `kTestProfile` from the selected profile. V0.1 stated the same
  configuration four times over as independently editable values; a profile name
  contradicting its own rail facts is no longer expressible.
- Module `ExpectedState` derives from one shared rule
  (`core::expectedStateForServoBus`/`Battery`/`LedRail`) instead of a ternary repeated
  per module. `core::classify()` itself is unchanged.
- **The source default remains `USB_ONLY`**, and `static_audit.py` fails the build if it
  is anything else — this is the G3 authorization gate. A powered build requires an
  explicit, announced override (`MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh`).

Servo population model (`src/servo/ServoPopulation.h/.cpp`):

```text
canonical allocated                 = 17
expected in current configuration   = 13
absent by design                    = 52 NECK_PITCH, 53 HEAD_ROTATION,
                                      54 HEAD_PITCH, 55 JAW
```

Healthy census semantics for the current robot:

```text
present_expected = 13
absent_by_design = 4
missing_expected = 0
unexpected_id    = 0
verdict          = PASS
```

The three populations (canonical allocation / expected current configuration / live
observed) are kept explicitly distinct. "17 responders = PASS" is false for this robot
and is not encoded anywhere; the static audit fails the build if such a threshold
reappears. Classification is fail-closed: a scan not covering every canonical ID, or a
truncated responder list, can never report `PASS`.

Transport independence:

- `ServoPopulation` and `ServoCensus` contain no `Serial` and no `<Arduino.h>`
  (audit-enforced). `ServoCensus` is a Controller-owned service holding a structured,
  fixed-size `CensusResult`; `CommandRouter` only formats it.
- No G2 domain logic exists solely inside Serial parsing or Serial printing. A future
  transport adapter can consume the same `CensusResult` without re-scanning the bus or
  reimplementing the classification.

Offline results (all run on the final G2 source):

| Gate | Result |
|---|---|
| New host census/profile tests | **PASS** — 14 cases / 274 checks / 0 failures |
| Static safety audit | **PASS** — 29 source files, 0 findings |
| OTA partition logic suite | **PASS** — 40/40 |
| BNO085 viewer `npm run verify` | **PASS** — 59/59 tests, typecheck PASS, production build PASS |
| Compile, `USB_ONLY` (pinned FQBN) | **PASS** — 387164 bytes flash, 28344 bytes RAM |
| Compile-only check, `ROBOT_POWERED` | **PASS** — 387632 bytes flash, 28344 bytes RAM |

The host suite links the real firmware translation units (`ServoPopulation.cpp`,
`Availability.cpp`) rather than a host-side copy, and exercises **both** profiles in one
run — which is what demonstrates the `USB_ONLY` expected-hardware semantics did not
regress while `ROBOT_POWERED` was added. Every new static-audit tripwire was
mutation-tested and confirmed to fire.

The `ROBOT_POWERED` compile check produced a temporary artifact only. **The default
build artifact was restored to `USB_ONLY` afterwards** and the exported binary was
re-checked to contain the `USB_ONLY` profile string and no `ROBOT_POWERED` string.

### Not implemented in this gate

No Web UI, Wi-Fi, HTTP/WebSocket/REST or OTA transport was added. No ActuatorAuthority
framework, Safe Actuator layer, motion, IK, gait, pose, teleoperation or command
lease/deadman was added. No provisioning, QC, source-signature or calibration
integration was added. Awareness of the V2 target architecture shaped the interfaces;
it did not turn this gate into a refactor.

### TO_TEST — G3 ROBOT_POWERED live hardware validation

Not performed. `ROBOT_POWERED` is **IMPLEMENTED**, not **VALIDATED**: offline tests and
a clean compile only. Specifically **not** done this session:

- no firmware flash of any kind;
- no external robot power enabled;
- no servo power;
- no DALY powered validation;
- no powered LED validation;
- no powered census;
- no hardware SAFE_OFF campaign;
- no motion.

The ESP32-S3 was connected over USB throughout and the full-flash recovery backup was
verified locally against its known size and digest
(`16777216` bytes, SHA256 `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`),
but the optional `USB_ONLY` on-device regression was **deliberately not performed**: it
added no material evidence (the derived `USB_ONLY` values are provably identical to
V0.1, and a census over an unpowered bus can only report all-missing), and the
application-only flash workflow correctly refused a dirty working tree. That refusal was
respected, not bypassed.

Under `USB_ONLY` a `SAFE_OFF` can only ever return `UNVERIFIED_NO_RESPONSE`; G3 P5 is
the first opportunity to observe `VERIFIED_OFF` on real powered hardware.

### Safety invariants preserved

- [x] no Torque ON
- [x] no `GoalPosition` / motion primitive
- [x] no servo EEPROM / ID / `CalibrationOfs` / factory-reset / broadcast write
- [x] no DALY write
- [x] no BNO085 DCD write
- [x] `SAFE_OFF` independent `TorqueEnable` readback semantics unchanged, and it remains
      reachable in every operating mode
- [x] `kDiagnosticTimeoutMs` (20ms) / `kOperationalTimeoutMs` (100ms) split unchanged
- [x] one `ServoBus` owner — no second `HardwareSerial`, no second `SMS_STS`, no
      duplicate UART setup
- [x] no automatic census/scan at boot (now reported as `startup_servo_scan : DISABLED`
      and enforced by the static audit against `Controller::begin()`)
- [x] no startup motion, no startup torque
- [x] pin map unchanged; frozen ST3215 tools and `matdog/full-leg-calibrator-v1` untouched
- [x] joint calibration untouched and still `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`

### Judgement

```text
G2 SOFTWARE/OFFLINE = PASS
G3 ROBOT_POWERED LIVE = NOT EXECUTED / TO_TEST
```

ROBOT_POWERED software support is implemented, offline-tested and compile-verified. It
is **not** validated against powered hardware and must not be described as such. Powered
validation requires separate hardware authorization per
[`G3_ROBOT_POWERED_VALIDATION_PLAN.md`](G3_ROBOT_POWERED_VALIDATION_PLAN.md).

---

## G2 — PRE-G3 HARDENING AMENDMENT — 2026-09-16

Amendment to the G2 gate above, resolving two issues raised by an independent review of
commit `34afbc7808e276d7483de9f2c817750683a65088`. This is a **G2 software/offline
amendment, not G3 execution**.

```text
G2 SOFTWARE / OFFLINE SCOPE      = VALIDATED (revised gates re-run, all PASS)
ROBOT_POWERED PROFILE            = IMPLEMENTED (compile-tested only)
ROBOT_POWERED HARDWARE OPERATION = TO_TEST

G3 NOT EXECUTED
```

No firmware was flashed. No external rail was energized. The compiled-in default profile
remains `USB_ONLY`, and the exported build artifact was restored to `USB_ONLY` after the
`ROBOT_POWERED` compile check.

### Finding 1 — build profile was not bound to the flashed artifact

`build.sh` can emit a `USB_ONLY` or a `ROBOT_POWERED` image from the same commit to the
same path. The pre-existing gate proved the binary embedded the current build id — but
that id is identical for both profiles, so it could not distinguish them. A stale image
of the wrong profile could have been written under the wrong assumption, in either
direction.

The gap was concrete, not theoretical: the two profiles produce genuinely different
artifacts.

```text
USB_ONLY       387312 bytes  sha256 a62f6d72ad38a1e8...
ROBOT_POWERED  387776 bytes  sha256 6ec5c714d11d9cf3...
```

Resolution — a build manifest, written by `build.sh` into the gitignored build directory
adjacent to the binary (never committed), binding:

```text
MATDOG_MANIFEST_VERSION  SOURCE_COMMIT  BUILD_ID  SOURCE_STATE
HARDWARE_PROFILE  FQBN
APPLICATION_BINARY  APPLICATION_SIZE  APPLICATION_SHA256
```

`flash_app_only.sh` verifies it before any device write, fail-closed, and the operator
must name the profile they intend via `MATDOG_FLASH_PROFILE` (default `USB_ONLY`). The
verified profile is printed in a banner immediately before the write.

Refusal matrix, exercised against the two REAL binaries built this session:

| Manifest | Requested | Result |
|---|---|---|
| `USB_ONLY` | `USB_ONLY` (default) | ALLOW |
| `ROBOT_POWERED` | `ROBOT_POWERED` (explicit) | ALLOW |
| `ROBOT_POWERED` | default, no authorization | REFUSE `PROFILE_MISMATCH` |
| `USB_ONLY` | `ROBOT_POWERED` | REFUSE `PROFILE_MISMATCH` |
| `USB_ONLY` manifest beside a `ROBOT_POWERED` binary | `USB_ONLY` | REFUSE `BINARY_SIZE_MISMATCH` |
| profile is an unrecognized string | any | REFUSE `PROFILE_UNKNOWN` |
| manifest file absent | any | REFUSE `MANIFEST_MISSING` |
| commit != HEAD | any | REFUSE `SOURCE_COMMIT_MISMATCH` |
| tree dirty (at build time or now) | any | REFUSE `TREE_NOT_CLEAN` |

Also refused (offline tests): unparseable manifest, incomplete manifest (each required
key removed individually), unknown manifest schema version, SHA256 mismatch at equal
size, missing binary, unknown requested profile.

**No pre-existing application-only protection was weakened.** Backup size and digest,
device MAC, verified application partition offset/size, partition-fit check,
rollback/anti-rollback state, the static-audit gate, the single-partition write and the
independent post-write `verify-flash` are all retained, and each is now individually
asserted by `static_audit.py` against its actual comparison (not merely token presence).

### Finding 2 — `SystemHealth::READY` was unreachable under `ROBOT_POWERED`

`classify()` turned `DetectedState::UNKNOWN` into a verdict, treating "nothing has
established anything" identically to "we asked and it did not answer". Under `USB_ONLY`
this was invisible (both non-`REQUIRED` cases return `PASS` either way); under
`ROBOT_POWERED` it produced two wrong answers:

```text
LED   OPTIONAL + UNKNOWN -> DEGRADED   (WS2812 has no readback path at all, so its
                                        detected state is permanently UNKNOWN when
                                        powered -> READY was unreachable)
SERVO REQUIRED + UNKNOWN -> FAULT      (nothing probes the bus at boot, so a healthy
                                        powered robot reported FAULT before anything
                                        had been asked of it)
```

The second case was found while evaluating the first across both profiles, as the review
required. Both are resolved by one rule rather than two special cases:

```text
UNKNOWN     + REQUIRED            -> UNKNOWN   (not proven; system reports BOOTING)
UNKNOWN     + OPTIONAL            -> PASS
UNKNOWN     + EXPECTED_OFFLINE    -> PASS
UNKNOWN     + EXPECTED_UNPOWERED  -> PASS

NO_RESPONSE + REQUIRED            -> FAULT     unchanged
NO_RESPONSE + OPTIONAL            -> DEGRADED  unchanged
NO_RESPONSE + OFFLINE/UNPOWERED   -> PASS      unchanged
UNPOWERED   + <as above>                       unchanged
ONLINE      + anything            -> PASS      unchanged
INIT_FAILED + anything            -> FAULT     unchanged
```

No physical detection is faked. The LED ring still reports `detected=UNKNOWN` in
`@STATUS`, and `static_audit.py` now fails the build if `detectedStateForLedRail()` ever
returns `ONLINE`. An observed failure still escalates: a probed-and-silent servo bus is
still `FAULT`, and an observed optional-module failure is still `DEGRADED`.

`USB_ONLY` semantics are bit-for-bit unchanged — a test reproduces the hardware-validated
Session 2 / H3 table exactly and asserts it still aggregates to `READY`.

`SystemHealth::READY` is now reachable under a healthy `ROBOT_POWERED` state without
pretending WS2812 detection:

```text
BNO085 init=OK detected=ONLINE  expected=REQUIRED result=PASS
DALY   init=OK detected=ONLINE  expected=REQUIRED result=PASS
SERVO  init=OK detected=ONLINE  expected=REQUIRED result=PASS   (after the P4 census)
LED    init=OK detected=UNKNOWN expected=OPTIONAL result=PASS   (never claimed ONLINE)
                                                  -> SystemHealth::READY
```

Before the P4 census the servo bus is honestly unproven, so the system reports `BOOTING`
— a visible gap, neither a false alarm nor false health. The G3 plan's P1 and P6
acceptance criteria were updated to state exactly this, so code and plan now agree.

Supporting change: `SystemState::beginBoot()` takes `now_ms` instead of calling
`millis()`, so the translation unit is Arduino-free and the offline tests link the real
aggregation rather than a reimplementation.

### Revised offline acceptance — re-run in full after both fixes

| Gate | Result |
|---|---|
| Servo population / profile / availability host tests | **PASS** — 18 cases / 313 checks / 0 failures |
| Build manifest provenance tests | **PASS** — 36/36 |
| Static safety audit | **PASS** — 29 source files, 0 findings |
| OTA partition logic suite | **PASS** — 40/40 |
| Compile, `USB_ONLY` (pinned FQBN) | **PASS** — 387156 bytes flash, 28344 bytes RAM |
| Compile-only check, `ROBOT_POWERED` | **PASS** — 387624 bytes flash, 28344 bytes RAM |
| BNO085 viewer `npm run verify` | **PASS** — 59/59 tests, typecheck PASS, production build PASS |
| `git diff --check` | clean |

All 15 new static-audit tripwires were mutation-tested and confirmed to fire, including
three that an earlier iteration of this amendment failed to catch (they matched text in
comments or in refuse messages rather than real code, and were tightened until they
detected the deletion of the gate they protect).

### Unchanged by this amendment

Servo population model (canonical 17 / expected-now 13 / absent-by-design 52-55), census
classification and its fail-closed verdicts, transport independence, the `USB_ONLY`
source default and its static-audit gate, and every safety invariant recorded in the G2
section above: no Torque ON, no `GoalPosition`, no servo EEPROM/ID/`CalibrationOfs`/
factory-reset/broadcast write, no DALY write, no BNO085 DCD write, `SAFE_OFF` independent
readback unchanged, timeout split unchanged, one `ServoBus` owner, no automatic
census/scan at boot, no startup motion.

### Judgement

```text
G2 SOFTWARE/OFFLINE = PASS
G3 ROBOT_POWERED LIVE = NOT EXECUTED / TO_TEST
```

---

## G2 — PRE-G3 PROVENANCE CLOSURE — 2026-09-16

Final pre-G3 closure amendment on top of the G2 gate and its hardening amendment above.
Software/offline only.

```text
G2 SOFTWARE / OFFLINE SCOPE      = VALIDATED (revised gates re-run, all PASS)
ROBOT_POWERED PROFILE            = IMPLEMENTED (compile-tested only)
ROBOT_POWERED HARDWARE OPERATION = TO_TEST

G3 ROBOT_POWERED LIVE = NOT EXECUTED / TO_TEST
```

No firmware was flashed. No external rail was energized. No hardware interaction of any
kind occurred. The compiled-in default profile remains `USB_ONLY`, and the final build
artifact and manifest were restored to `USB_ONLY`.

### Finding A — the recorded build FQBN was never compared

The build manifest recorded `FQBN` from the moment it was introduced, but the verifier
never checked it. Binding the commit, profile and exact bytes still left the *toolchain
configuration* unverified: the FQBN carries the partition scheme, flash size and mode,
PSRAM mode, USB/CDC mode and CPU frequency. A binary of the correct commit and correct
profile, built under a different partition scheme, would have passed every gate while
targeting a different application-partition layout.

Closed fail-closed:

- `verify_manifest()` takes a **required** `expected_fqbn` keyword with no default and
  refuses on exact inequality with the stable reason `FQBN_MISMATCH`. The refusal detail
  names both the manifest value and the expected value.
- The CLI requires `--expected-fqbn` (no default) and prints `VERIFIED_FQBN` on success.
- `flash_app_only.sh` passes its own pinned `"$FQBN"`. The FQBN string itself is
  unchanged; see the pinned value in
  [the Controller README](README.md#build).

A successful verification now positively proves that **all** of the following belong to
the artifact being authorized:

```text
source commit + clean build state + clean current tree
+ application filename + exact binary size + exact binary SHA256
+ hardware profile + FQBN
```

Refusal matrix, exercised against the two REAL binaries built this session
(`USB_ONLY` 387312 bytes `503735ca…`, `ROBOT_POWERED` 387776 bytes `6abbe734…`):

| Artifact | Requested profile | Expected FQBN | Result |
|---|---|---|---|
| `USB_ONLY` | `USB_ONLY` | canonical | ALLOW |
| `ROBOT_POWERED` | `ROBOT_POWERED` (explicit) | canonical | ALLOW |
| `ROBOT_POWERED` | default, no authorization | canonical | REFUSE `PROFILE_MISMATCH` |
| `USB_ONLY` | `USB_ONLY` | `PartitionScheme=default` | REFUSE `FQBN_MISMATCH` |
| `USB_ONLY` | `USB_ONLY` | `FlashSize=8M` | REFUSE `FQBN_MISMATCH` |
| `USB_ONLY` | `USB_ONLY` | `PSRAM=disabled` | REFUSE `FQBN_MISMATCH` |
| `ROBOT_POWERED` | `ROBOT_POWERED` | `CPUFreq=160` | REFUSE `FQBN_MISMATCH` |
| manifest FQBN tampered, binary/profile/commit identical | `USB_ONLY` | canonical | REFUSE `FQBN_MISMATCH` |
| manifest FQBN empty | `USB_ONLY` | canonical | REFUSE `MANIFEST_INCOMPLETE` |
| manifest FQBN key absent | `USB_ONLY` | canonical | REFUSE `MANIFEST_INCOMPLETE` |

Additionally covered by the offline suite: per-option drift (`FlashMode`, `CDCOnBoot`),
exact-not-substring comparison (prefix, suffix, case change, bare `esp32:esp32:esp32s3`),
refusal ordering (`SOURCE_COMMIT_MISMATCH` outranks `FQBN_MISMATCH`; `FQBN_MISMATCH`
outranks tree-state and binary checks), and both profiles passing under their own
authorization with the canonical FQBN.

**No pre-existing flash protection was weakened.** Backup size and digest, device MAC,
verified application partition offset/size, partition-fit, rollback/anti-rollback state,
the static-audit gate, the single application-partition write and the independent
post-write `verify-flash` are all retained and still individually asserted by
`static_audit.py`.

### Finding B — Development Gates could be read as reordering the roadmap

The HostLink gate's ENTRY condition read "Diagnostics/Maintenance foundation present",
which could be read as authorizing HostLink immediately after Diagnostics — ahead of
Service/Provisioning/QC, Full Leg Calibration integration and formal recalibration in the
canonical sequence.

Corrected in [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md): HostLink ENTRY now requires
all preceding [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md) stages through
formal recalibration, plus the Diagnostics/Maintenance foundation as a technical
prerequisite rather than an authorization. A new global rule states that a gate's
technical prerequisites never override roadmap sequencing, and that reordering requires
an explicit reviewed roadmap change. The roadmap sequence itself was not altered and no
HostLink implementation was started.

### Finding C — architecture document date

`ARCHITECTURE.md` header corrected from 2026-09-15 to **2026-09-16**, the date of the
approved permanent Embedded Web UI decision it now contains. No other architecture change.

### Final offline acceptance — re-run after implementation

| Gate | Result |
|---|---|
| Servo population / profile / availability host tests | **PASS** — 18 cases / 313 checks / 0 failures |
| Build manifest provenance tests (incl. FQBN cases) | **PASS** — 51/51 |
| Static safety audit | **PASS** — 29 source files, 0 findings |
| Mutation check, new FQBN tripwires | **PASS** — 6/6 fire |
| OTA partition logic suite | **PASS** — 40/40 |
| Compile, `USB_ONLY` (pinned FQBN) | **PASS** — 387156 bytes flash, 28344 bytes RAM |
| Compile-only check, `ROBOT_POWERED` | **PASS** — 387624 bytes flash, 28344 bytes RAM |
| BNO085 viewer `npm run verify` | **PASS** — 59/59 tests, typecheck PASS, production build PASS |
| `git diff --check` | clean |

### Judgement

```text
G2 SOFTWARE/OFFLINE = FROZEN PASS
FINAL DOCUMENTATION / ROADMAP GATE = PASS
G3 ROBOT_POWERED LIVE = NOT EXECUTED / TO_TEST
```

`ROBOT_POWERED` remains **IMPLEMENTED**, never **VALIDATED**. Powered validation requires
separate hardware authorization per
[`G3_ROBOT_POWERED_VALIDATION_PLAN.md`](G3_ROBOT_POWERED_VALIDATION_PLAN.md).

---

## G3 — ROBOT_POWERED LIVE, NO MOTION — 2026-09-17

First powered session. Executed step by step under explicit operator authorization, each
step independently reviewed and accepted by the operator.

```text
G3 PREFLIGHT READ-ONLY                 = PASS
G3-A ROBOT_POWERED BUILD/PROVENANCE    = PASS
G3-B APPLICATION-ONLY FLASH            = PASS
G3-P2 DALY LIVE READ-ONLY              = PASS
G3-P3 LED LIVE                         = PASS
G3-P4A0 SINGLE ID51 PING               = PASS
G3-P4A SERVO CENSUS                    = PASS
G3-P4B0/B1 ID51 READ + SAFE_OFF PILOT  = PASS
G3 POWERED NO-MOTION EVIDENCE          = PASS  (operator-accepted)
G3 FORMAL CENSUS-REPEAT CRITERION      = OUTSTANDING (1 census run; 1 repeat required)
```

> **Closed 2026-09-18** — the second census matched the first; G3 formal PASS. See
> § G3 / G3.1 live closure below. The block above is the record as it stood on 2026-09-17.

Robot mechanically suspended throughout. Fused battery disconnect was the established
emergency power-removal path. **No motion, no Torque ON, no GoalPosition, no EEPROM, DALY
or DCD write at any point.**

| Item | Evidence |
|---|---|
| Device | ESP32-S3 MAC `14:c1:9f:22:75:94`, native USB-Serial/JTAG `303a:1001` |
| Recovery backup | 16777216 bytes, sha256 `5cbba0b9…4d67fd32` — verified |
| Flashed image | `6065d86c2da4`, `SOURCE_STATE=CLEAN`, `ROBOT_POWERED`, 387776 bytes, sha256 `0454345808a6aac46ec79fc711f48904c3a80008738b485918060801e4a99442` |
| Flash path | `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`: one write to `app0 @ 0x010000` (erase `0x10000–0x6efff`), rollback ENABLED / anti-rollback DISABLED, `verify-flash` digest matched |
| First boot | `profile: ROBOT_POWERED (servo_power=YES battery=YES led_rail=YES)`, `operating_mode: MAINTENANCE`, `IMU_INIT=PASS`, `health=BOOTING` (SERVO `UNKNOWN` before any probe, by design), `runtime_resets=0` |
| DALY | `ONLINE / REQUIRED / PASS`, `comm=OK`, fresh (`age_ms` < 2 s poll), pack 11.2 V 3S, no alarms, `charge_mos=ON discharge_mos=ON` |
| LED | `@LED TEST` → `LED_TEST=STARTED`; chase visually confirmed by the operator; ring returned to OFF autonomously; `test_running=NO` |
| ID51 ping | `found=1`, `max_ping_us=356` |
| Census | `SERVO_CENSUS=PASS lo=11 hi=55`: `present_expected=13 missing_expected=0 absent_by_design=4 absent_by_design_present=0 unexpected_id=0 not_probed=0 truncated=NO` |
| `SAFE_OFF` | `VERIFIED_OFF` on all 13 installed servos (write ACK ignored; independent `TorqueEnable` readback) |
| Runtime read | 13/13 respond, `torque=0` on all, `speed=0 load=0`, voltage 112–113 (11.2–11.3 V), temp 33–36 °C |
| Soak | 60 s passive, 600/600 telemetry lines valid, `runtime_resets=0`, `health=READY`, heap stable |
| Cold power cycle (2026-09-18) | came back `ROBOT_POWERED` / `MAINTENANCE` — application-only flash persistent, no rollback |

Recorded for completeness, not as regressions:

- **Census repeat-stability — OUTSTANDING.** The formal G3 PASS criterion requires the
  census to be "stable across repeats"; one complete census was executed. The 13
  subsequent `@SERVO READ` transactions and 12 `SAFE_OFF` readbacks each re-confirmed every
  expected ID, but they are not a census and do not substitute for one. The criterion is
  unchanged; one additional read-only `@SERVO CENSUS` at the next authorized live session
  closes it.
- **Boot banner** — the first-boot capture lost two blocks of the banner to a USB CDC
  transmit drop (the host opened the port mid-banner; the 256-byte TX ring overwrote them).
  `startup_motion` / `startup_torque` / `startup_servo_scan` were therefore established
  behaviourally (`SERVO detected=UNKNOWN`, `last_census=NOT_RUN` at 80 s uptime) and by the
  static audit, not read off the banner.
- **Soak scope** — observed with the CDC port open, the only way to observe it. G3.1 below
  shows that is the healthy condition; the soak's IMU conclusion holds for port-open only.
- **DALY KEY** — both switch positions produced byte-identical DALY state
  (`discharge_mos=ON` in both). The readings cannot distinguish "KEY does not gate the
  MOSFETs" from "KEY circuit not effective". `requestDischargeOff()` transmits nothing. The
  fused disconnect is the only demonstrated power-removal path.

---

## G3.1 — CDC-INDEPENDENT CONTROLLER LOOP — 2026-09-18

Regression found live **after** the G3 powered session. It does not invalidate the G3 servo/DALY/LED
evidence above, every item of which was measured transaction by transaction.

```text
G3 POWERED NO-MOTION EVIDENCE         = PASS
G3 FORMAL CENSUS-REPEAT CRITERION     = OUTSTANDING
G3.1 CDC-INDEPENDENT CONTROLLER LOOP  = FAIL -> FIX UNDER VALIDATION
Stage 4 onward (incl. all motion)     = BLOCKED until G3 census repeat + G3.1 PASS
```

> **Closed 2026-09-18** — G3.1 PASS live. See § G3 / G3.1 live closure below. The blocks in
> this section are the record as it stood before the live re-test.

### Finding — zero-TX passive A/B, same boot, cable attached throughout

| Condition | Δ `rv_count` | Elapsed | BNO085 RV rate |
|---|---:|---:|---:|
| Host CDC port **closed** | 41 | 60.218 s | **0.68 Hz** |
| Host CDC port **open** (same fd, immediately after) | 480 | 9.587 s | **50.07 Hz** |
| Configured (`SH2_ROTATION_VECTOR`, 20000 µs) | | | 50 Hz |

`runtime_resets` stayed 0; no boot, brownout, panic or malformed line. The reading port
was opened `O_RDONLY` — zero bytes transmitted. Endpoint staleness was bounded on the
device clock (`SAVE_GATE still_ms`): the closed-interval rate is **≤ 0.68 Hz** whenever the
buffered lines were printed. Recovery to ~50 Hz was immediate on reopen, and 19
consecutive open cycles ran at 49.5–51.3 Hz. Retrospectively, the same collapse explains
the G3-B `rv_count` values (580 at 86 s; 3942 at 41 min — ~1.4 Hz between them), and
it began each time a host process *closed* the port. Boots in which no host had opened
the port ran at 47–48 Hz.

### Root cause — confirmed in the installed core, not upstream

Core `esp32:esp32 3.3.11` (the only one installed), FQBN `USBMode=hwcdc` →
`-DARDUINO_USB_MODE=1`, `CDCOnBoot=cdc` → `-DARDUINO_USB_CDC_ON_BOOT=1`, so
`cores/esp32/HardwareSerial.h:441-444` maps `Serial` to `HWCDCSerial` (`HWCDC`).
In `~/.arduino15/packages/esp32/hardware/esp32/3.3.11/cores/esp32/HWCDC.cpp`:

1. `static volatile bool connected` is set `true` by the ISR on the first host IN pickup
   (`:148`) or OUT packet (`:239`), and cleared only by `BUS_RESET` (`:248`) or
   `!isPlugged()` (`:273-276`).
2. `isPlugged()` is `usb_serial_jtag_is_connected()`, which the ESP-IDF header documents
   as true "so long as it is receiving SOF packets from the host, even if … the serial
   port is not opened". Closing the host tty produces neither a bus reset nor SOF loss, so
   **`connected` stays latched true after the host closes the port.**
3. `HWCDC::write()` (`:540-623`), when `connected`, blocks in `xRingbufferSend(…,
   tx_timeout_ms)` with `tx_timeout_ms = 100` (`:105`, `FREERTOS_HZ=1000`) and retries up
   to `max_consec_timeouts = 20` (`:582`): **up to ~2 s per `Serial.print*()`** once the
   256-byte TX ring (`:421-422`) is full and nothing drains it.
4. `operator bool()` (`:375-377`) returns that same latched state, so `if (Serial)` is
   **true in exactly the failing state** and cannot be the guard.
5. `Bno085Imu::update()` services at most **one** SH2 event per loop pass, so every
   loop stall is directly lost acquisition.

Host condition on the ASUS: `power/control=auto` with a 2000 ms autosuspend delay, yet
the device stays `runtime_status=active` with the port closed (SOFs continue). A host
that did suspend the device would mask the bug — the firmware must not depend on host USB
power policy. `ModemManager` is active on the ASUS: any transient open of the port by any
process is enough to latch the failing state.

### Fix — native HWCDC configuration in `Controller::begin()`

Invariant: **no USB CDC transmit condition may block `Controller::update()`.**

Two core calls before `Serial.begin()`, values in `config/BuildConfig.h`:

```cpp
Serial.setTxBufferSize(build::kUsbTxRingBytes);   // 3072
Serial.setTxTimeoutMs(build::kUsbTxTimeoutMs);    // 0
Serial.begin(build::kUsbSerialBaud);
```

With `tx_timeout_ms = 0` every wait in the installed `HWCDC.cpp` becomes non-blocking: the
TX mutex is only try-locked (`xSemaphoreTake(…, 0)`, `:544`), each `xRingbufferSend(…, 0)`
(`:590`) returns immediately, and the `max_consec_timeouts = 20` loop (`:582`) is at most
20 immediate retries per chunk before a short write. No `delay()`, blocking semaphore or
flush remains on the write path (`delay(1)` exists only in `flush()`, never called). The
worst case for one `Serial.printf()` is formatting (plus `Print::vprintf`'s `malloc` above
63 bytes) and ≤ 20 lock-free ring attempts — bounded, CPU-only, independent of any host.
Output that does not fit is dropped whole per write call: each `printf` line is one write;
`println` is two, so under saturation a line can lose its CRLF.

Ring size from measured bursts (exact format strings, worst-case field widths):

| Burst | Bytes |
|---|---:|
| IMU telemetry block, live / worst | 407 / 510 |
| `@BMS STREAM` block, worst | 307 |
| Boot banner | 779 |
| `@STATUS` worst / `@HELP` | 561 / 628 |
| `@SERVO CENSUS` PASS / worst (13 missing + 4 absent-by-design + 24 unexpected) | 263 / 1578 |
| `@SERVO SCAN` worst (64 found) | 1125 |
| **Largest single loop pass** (worst census + IMU + BMS) | **2395** |

With timeout 0 nothing absorbs a burst, so even a host that is reading loses whatever
exceeds the free ring at the moment of the write (formatting outpaces USB drain). The
256-byte default is smaller than one IMU block, the banner, `@STATUS` and even a PASS
census reply. 3072 covers the 2395-byte worst case with 28 % margin (2560 is the strict
minimum at 512-byte granularity). Heap +2816 bytes; static RAM unchanged.

`#error` guards pin hwcdc + CDC-on-boot and core 3.3.11. `DebugLevel=none`, pinned in the
FQBN the manifest verifies, keeps `printBeforeSetupInfo()` out of the build — it would
begin `Serial` before `setup()` and route debug output through a second, per-character
transmit path.

Accepted behaviour:

- **Host opened, then closed or stopped reading** (flag latched): the ring keeps the oldest
  ≤ 3 KB and new output is dropped; on reopen the host first receives that backlog (~4 s of
  telemetry), then live data. A reply emitted before that backlog has drained may
  short-write rather than block.
- **Port never opened since boot**: the core overwrites the oldest bytes, so the boot banner
  stays in the ring for ~4 s after reset (~2.8 s of telemetry after it). A host must open
  the port within that window to read it.
- **ESP-IDF secondary console** (`CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y`) writes to
  the same hardware outside HWCDC, only on IDF error logs or panics. Pre-existing and
  unchanged; not verifiable from local source (IDF sources are not installed). No such
  output has been observed in any capture.

**Delivery semantics.** The timeout-0 guarantee is that USB CDC transmit cannot stall
Controller progress; it is not a delivery guarantee. Command replies are expected complete
under normal connected/draining conditions with adequate TX-ring free space. After a long
closed-port interval a stale TX backlog may still occupy the ring immediately after reopen,
and a reply emitted before enough space has drained may short-write rather than block. This
is acceptable for the current diagnostic USB command surface and is **not** promoted into the
future HostLink contract, which remains responsible for its own framing, acknowledgement and
reliability.

Alternative evaluated and not adopted: a `SerialTransport` channel abstraction gating
unsolicited output on `availableForWrite()` with a response reserve. It satisfied the same
invariant but changed 5 firmware files, added 2 more and ~430 lines of tests; the native
configuration satisfies it in 2 firmware files.

### Offline acceptance

| Gate | Result |
|---|---|
| New audit rule `check_usb_cdc_tx_never_blocks` | **PASS**; mutation check **11/11 caught** (timeout ≠ 0, ring < 2560, ring or timeout after `begin()`, timeout call removed, output before `begin()`, either guard removed, a second `setTxTimeoutMs()`, `Serial.flush()`, `setDebugOutput()`) |
| Static safety audit | **PASS** — 29 source files, 0 findings |
| Servo population / profile host tests | **PASS** — 313 checks / 0 failures |
| Build manifest provenance tests | **PASS** — 51/51 |
| OTA partition logic suite | **PASS** — 40/40 |
| BNO085 viewer `npm run verify` | **PASS** — 59/59 tests, typecheck, production build |
| Compile, `USB_ONLY` (pinned FQBN) | **PASS** — 387188 bytes flash, 28344 bytes static RAM |
| Compile, `ROBOT_POWERED` (pinned FQBN) | **PASS** — 387656 bytes flash, 28344 bytes static RAM |

Only the two pre-existing third-party SCServo warnings appear; none in MATDOG source.

### Live validation — TO_TEST (requires a clean commit, rebuild and separate authorization)

> Executed 2026-09-18 — results in § G3 / G3.1 live closure below.

1. Commit; `MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh`; verify manifest `CLEAN`;
   `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`.
2. Open a passive (zero-TX) listener **immediately** after the flash script returns: the
   full banner is expected, including `startup_*`, `partition` and `reset_reason` (closes the
   G3 boot-banner gap).
3. Close the port (the flag is now latched — the failing case). Zero-TX A/B: closed ≥ 60 s,
   then open 10 s. **PASS: closed ≥ 45 Hz and open 45–55 Hz**, `runtime_resets` unchanged,
   every line valid.
4. `@BMS STREAM ON`, close the port 60 s, reopen: rate as in 3; the stream resumes.
5. The viewer (zero commands) connects after a closed interval and receives live telemetry.
6. With the host reading and the reconnect backlog drained (wait ≥ 1 s after opening),
   `@STATUS`, `@IMU STATUS`, `@BMS STATUS`, `@LED STATUS`, `@HELP`: every reply complete.
7. **One read-only `@SERVO CENSUS`** — closes the outstanding G3 census-repeat criterion
   (expect `PASS`, 13 present, 4 absent by design); issued after the backlog has drained, it
   also shows a census reply arrives complete under timeout 0.

---

## G3 / G3.1 LIVE CLOSURE — 2026-09-18

```text
G3 FORMAL CENSUS-REPEAT                      = PASS
G3.1 CDC CLOSED-PORT INDEPENDENCE            = PASS
G3.1 BMS STREAM ROBUSTNESS                   = PASS

G3 FORMAL ROBOT_POWERED NO-MOTION VALIDATION = PASS
G3.1 CDC-INDEPENDENT CONTROLLER LOOP         = PASS
```

Robot mechanically suspended, fused disconnect accessible, operator present throughout.
No commanded servo motion occurred and no robot motion was observed during validation.

### G3.1-L1 — clean application-only flash and first boot

| Item | Evidence |
|---|---|
| Source | `e2fc60531b28351472a2bfa5105a2170147610bb`; manifest `SOURCE_STATE=CLEAN`, `HARDWARE_PROFILE=ROBOT_POWERED` |
| Application | 387808 bytes, SHA256 `e2b474b5c07e98649fbaf31d3d08970d022ffbdb5f1212d6b606dc28cac065d2` |
| Flash | `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh` — MAC `14:c1:9f:22:75:94`; one write, erase `0x10000–0x6efff` inside `app0`; bootloader, partition table, otadata and NVS untouched; `verify-flash` digest matched; `APPLICATION_ONLY_FLASH = PASS` |
| First boot (passive, zero bytes sent) | complete banner: `build e2fc60531b28`, `ROBOT_POWERED (servo_power=YES battery=YES led_rail=YES)`, `partition app0 @ 0x010000`, `reset_reason OTHER` (the USB-JTAG RTS reset has no named entry), `startup_motion` / `startup_torque` / `startup_servo_scan : DISABLED`, `daly_write : NOT_IMPLEMENTED`, `operating_mode MAINTENANCE`, `IMU_INIT=PASS`, `SYSTEM_BOOT_COMPLETE health=BOOTING` |
| First telemetry | BNO085 RV 50.12 Hz, `runtime_resets=0`, every line valid |

The complete banner closes the G3 boot-banner gap: the `startup_*` lines are now directly observed.

### CDC independence

Zero-TX passive A/B. A host had already opened and closed the port after boot, so HWCDC was in the
latched state that failed before the fix.

| | Value |
|---|---|
| Pre-close (`still_ms` / `rv` / `runtime_resets`) | S0 = 471826 ms, R0 = 23644, Z0 = 0 |
| Host closed interval | 62.121 s |
| First fresh post-close | S1 = 534345 ms, R1 = 26776, Z1 = 0 |
| Device interval S1 − S0 | 62.519 s (host clock 62.520 s) |
| **Closed-port RV rate** | **50.10 Hz** (0.68 Hz before the fix) |
| Open recovery RV rate | 50.13 Hz (21 fresh blocks, 10.07 s) |
| `runtime_resets` | 0 |

Backlog on reopen: 7 stale complete IMU blocks, 3131 bytes before the first fresh block, stale
`still_ms` offsets +0.503 … +3.524 s after S0 — the 3 KB ring holding ~3.5 s of post-close
telemetry, as designed. No `still_ms` reset. Fresh blocks were identified by the `still_ms`
discontinuity and by counter consistency within each block (`RV count` = `COUNTS rv`).

### Second servo census

```text
SERVO_CENSUS=STARTED lo=11 hi=55
SERVO_CENSUS=PASS lo=11 hi=55
  canonical_allocated=17 expected_now=13
  present_expected=13 missing_expected=0 absent_by_design=4
  absent_by_design_present=0 unexpected_id=0 not_probed=0 truncated=NO
SERVO  init=OK detected=ONLINE expected=REQUIRED result=PASS
```

Identical, line for line, to the first census (G3-P4A, 2026-09-17).
**CENSUS STABLE ACROSS REPEATS = PASS.** The runs used firmware `6065d86` and `e2fc605`; the census
code path is identical in both. The census covers IDs 11..55 only; nothing is claimed about IDs
outside that range. Telemetry continued during and after it.

### Diagnostic replies

With the host reading and the reconnect backlog drained: `@STATUS`, `@IMU STATUS`, `@BMS STATUS`,
`@LED STATUS` and `@HELP` each returned a complete, syntactically valid reply (7, 2, 5, 2 and 18
lines); none was re-issued.

### BMS stream robustness

| | Value |
|---|---|
| `@BMS STREAM ON` | `BMS_STREAM=ON`; 3 stream blocks before close |
| Pre-close | S2 = 553514 ms, R2 = 27706, Z2 = 0 |
| Host closed interval | 62.119 s |
| First fresh post-close | S3 = 616057 ms, R3 = 30840, Z3 = 0 |
| Device interval | 62.543 s (host clock 62.544 s) |
| **Closed-port RV rate, stream enabled** | **50.11 Hz** |
| After reopen | 4 fresh stream blocks: DALY `comm=OK`, `charge_mos=ON`, `discharge_mos=ON`, alarms all zero |
| `@BMS STREAM OFF` | `BMS_STREAM=OFF`; zero stream blocks in the following 4.5 s |
| `runtime_resets` | 0 |

### Final state

```text
SYSTEM health=READY power_state=RUN mode=MAINTENANCE profile=ROBOT_POWERED
BNO085 REQUIRED/PASS   DALY REQUIRED/PASS   SERVO REQUIRED/PASS   LED OPTIONAL/PASS
runtime_resets=0
```

Final passive window 50.11 Hz. No brownout, panic, reboot or FAULT at any point. The BNO085
stillness timer never reset across the 632 s batch (gyro magnitude 0.000000 rad/s). Pack 11.0 V,
SOC 41.4 %, cell Δ12–13 mV, no alarms.

### Still open

- **DALY `KEY` — OPEN.** Toggling the physical KEY switch produced no observed change in any
  DALY-reported state (`discharge_mos=ON` in both positions). KEY is **not** a validated shutdown
  or safety barrier; the DALY's actual KEY configuration and function must be inspected before any
  setting is changed. The fused disconnect remains the trusted physical isolation method.
- **GPIO19/GPIO20 — frozen.** GPIO19 = native USB D−, GPIO20 = native USB D+. The external
  19/20/GND connector remains a future USB service-port candidate — **not** a UART — pending
  electrical and signal-integrity validation.

---

## DALY KEY — RESEARCH AND READ-ONLY PROBE — 2026-09-19

```text
DALY KEY RESEARCH                 = COMPLETE (read-only; no DALY write, no KEY toggle)
DALY KEY READ PROBE               = IMPLEMENTED, OFFLINE-VALIDATED — LIVE VALIDATION TO_TEST
DALY KEY WRITE / CONFIGURATION    = BLOCKED (not implemented; forbidden by the static audit)
KEY AS SHUTDOWN / SAFETY BARRIER  = NOT VALIDATED — fused disconnect remains trusted isolation
```

### Research (read-only session, robot powered, 2026-09-19)

Live baseline, `@BMS STATUS` / `@STATUS` only: `comm=OK`, pack 11.0 V, −0.1 A, SOC 39.3 %,
`charge_mos=ON discharge_mos=ON`, alarms `0000 0000 0000 0000`, `runtime_resets=0`.

- **Public DALY documents** (H/K/M/S operating manual, Smart BMS Control Strategy, Protocols
  page, UART/485 V1.2): KEY defaults to BMS activation, other KEY functions "can be customized"
  / "involve customized software"; sleep needs no communication, no current and no wake-up
  signal, and **leaves both MOS on** except on undervoltage. **No K-series KEY configuration
  register is published.** The legacy UART/485 protocol has no MOS-control or KEY command.
- **DALY BMSTool V1.14.79** (official download, internal name `PCMaster`), inspected statically
  — extracted and its .NET metadata parsed offline, **never run, no BMS access**:
  - a second Modbus personality: request address `0x80 + board` (default `0x81`), reply
    `0x50 + board` (`0x51`); its register map differs from the `0xD2` one MATDOG uses (pack
    voltage `0x38` vs `0x28`), so no register is carried across;
  - parameter-block read `81 03 01 00 00 78 5B D4` (reply `51 03 F0 …`, 245 bytes);
  - `0x0120` KEY logic, written by the tool's Set button with FC06: `0x55` DISABLED, `0xA5`
    DISCHARGE_AND_SLEEP, `0x5A` DISCHARGE, `0xAA` CHARGE_AND_DISCHARGE, `0xA6`
    CHARGE_DISCHARGE_AND_SLEEP; the tool then asks for a BMS restart;
  - `0x0121` / `0x0122` charge / discharge MOS control (0/1), from its Engineering page;
  - `0x0115` sleep time, displayed as raw × 10 s;
  - fault flags "key turn off chg mos" / "key turn off dsg mos" exist in its fault table.
- The phone app's "Charge switch" / "Discharge switch" most likely map to `0x0121`/`0x0122` — a
  software MOS control, **not** the KEY configuration (inference, not verified).
- MATDOG's target — KEY OFF → discharge MOS OFF, charge MOS kept — corresponds to candidate
  `0x0120 = 0x005A`. **Not verified on this unit; not implemented.**

### Implementation (offline only; not flashed)

- `src/power/DalyProtocol.{h,cpp}` — new, Arduino-free: the two constant FC03 frames
  (`static_assert`-checked: address, function, start, count, CRC), `dalyRequestFrame(enum)`,
  register-based response offsets, CRC-16/MODBUS, response validation (length → header → CRC,
  the order telemetry already used), the `0xD2` telemetry decoder **moved verbatim** from
  `DalyBms.cpp`, the `0x81` KEY decoder, and `DalyBusScheduler`.
- `DalyBusScheduler` is the single transaction owner: one transaction in flight at most, never
  cancelled; a requested KEY read starts at the next idle boundary ahead of a due poll but
  defers at most one poll; a 300 ms quiet gap follows every transaction (longer than a whole
  245-byte reply, never binding for telemetry alone); KEY timeout 1000 ms (245 bytes take
  255 ms at 9600 8N1), telemetry timeout unchanged at 750 ms; telemetry resumes automatically.
- `DalyBms` keeps the UART and exactly one `bms_uart_.write(dalyRequestFrame(request),
  kDalyRequestLen)`. The KEY result is separate from telemetry health: a failed or silent
  `0x81` read never touches `detected_`/`last_result_` and never replaces the last valid
  snapshot. `requestDischargeOff()` is unchanged (`{ return false; }`).
- `@BMS KEY READ` (MAINTENANCE only, no arguments) → `STARTED` / `BUSY` / `BLOCKED`, then
  `BMS_KEY_READ=COMPLETE result=OK` + `key_logic_raw` / `key_logic` / `charge_mos_control` /
  `discharge_mos_control` / `sleep_time_raw` / `sleep_time_s`, or
  `result=TIMEOUT|CRC_FAIL|BAD_HEADER rx_bytes=<n>`. `@BMS KEY STATUS` prints the cached
  result and snapshot with ages and zero bus traffic (`snapshot=NOT_READ key_logic=UNKNOWN`).
- Enumerator `DalyKeyLogic::KEY_DISABLED` (printed `DISABLED`): the Arduino-ESP32 core
  `#define`s `DISABLED` (`esp32-hal-gpio.h`); the host suite now compiles with that macro
  defined so the clash is caught offline.
- No PowerState, servo, pin, BNO085, LED, network or boot-banner change.

### USB CDC burst budget (G3.1) — recomputed

| Burst | Bytes |
|---|---:|
| `@HELP` (was 628) | 773 |
| `BMS_KEY_READ=COMPLETE` block, worst | 193 |
| `@BMS KEY STATUS`, worst | 262 |
| **Largest single loop pass** (worst census + KEY result + IMU + BMS; was 2395) | **2588** |

The 3072-byte ring still covers it (16 % margin). The audit floor rises from 2560 to 3072
bytes, the strict minimum for 2588 at 512-byte granularity — a tightening.

### Offline acceptance (uncommitted tree; clean rebuild after commit)

| Gate | Result |
|---|---|
| Static safety audit | **PASS** — 32 source files, 0 findings |
| DALY write-prohibition mutation suite (`test_static_audit_daly.py`) | **PASS** — 24/24: unmutated tree clean; caught KEY FC `0x03→0x06`, `0x03→0x10`, a CRC-valid FC06 write of `0x5A` to `0x0120`, telemetry FC `0x03→0x06`, a third read frame, a second write call, a raw-buffer write, `bms_uart_.print`, `bms_uart_` passed as a stream, a buffer-taking selector, an unlisted frame, `requestDischargeOff()` implemented or called, `@BMS KEY SET`, argument parsing, ungated `@BMS KEY READ`, `@BMS KEY STATUS` transacting, KEY API in PowerState, `Serial2`, an extra `HardwareSerial`, a `0x06` literal, Arduino in the protocol unit, a write frame admitted to the whitelist |
| DALY protocol host tests (`test_daly_protocol.cpp`) | **PASS** — 214 checks / 0 failures; production mutation check **17/17 killed** (swapped MOS registers, KEY register ±1, sleep ×1, 16-bit sleep wrap, `0x5A` misdecoded, reply `0x81` accepted, byte-count / CRC check dropped, snapshot overwritten on failure, header length, no quiet gap, no fairness, pre-emption, double request, telemetry current / MOS register, KEY timeout) |
| Servo population / profile host tests | **PASS** — 313 checks / 0 failures |
| Build manifest provenance tests | **PASS** — 51/51 |
| OTA partition logic suite | **PASS** — 40/40 |
| BNO085 viewer `npm run verify` | **PASS** — 59/59 tests, typecheck, production build |
| Compile, `USB_ONLY` (pinned FQBN) | **PASS** — 389292 bytes flash, 28504 bytes static RAM |
| Compile, `ROBOT_POWERED` (pinned FQBN) | **PASS** — 389760 bytes flash, 28504 bytes static RAM |

Versus G3.1: +2104 bytes flash, +160 bytes static RAM (mostly the receive buffer growing from
129 to 245 bytes). Only the pre-existing third-party SCServo warnings appear.

### Live validation — TO_TEST (read-only; needs a clean build, flash and separate authorization)

> Executed 2026-09-19 — results in § DALY KEY live read-only validation below.

1. Power the robot normally; connect USB (no-reset method). `@STATUS`, `@MODE STATUS`
   (MAINTENANCE is the boot default), `@BMS STATUS` — telemetry must be `comm=OK`.
2. `@BMS KEY READ` once; capture the complete asynchronous result. `@BMS KEY STATUS`.
3. `@BMS STATUS` again: telemetry must still be `comm=OK` (the KEY read deferred at most one
   poll and polling resumed).
4. **No KEY toggle. No write. No mode change beyond reading.**

Decision:

- `result=OK key_logic_raw=0x0055` (DISABLED) — explains the G3 finding; the candidate future
  action is a **separately authorized** `0x0120 = 0x005A`. Not implemented here.
- `result=OK` with another recognized value — investigate wiring and firmware semantics before
  any write.
- `result=OK key_logic=UNKNOWN` — stop; the register map does not match this unit.
- `result=TIMEOUT rx_bytes=0` (no `0x81` reply) or any other failure — **do not escalate to
  writes**; the personality is unconfirmed on this port.

---

## DALY KEY — LIVE READ-ONLY VALIDATION — 2026-09-19

```text
DALY 0x81 READ PERSONALITY        = PASS  (live verified, read-only)
DALY KEY READ PROBE               = PASS  (one transaction, result=OK)
TELEMETRY RESUMPTION              = PASS  (0xD2 comm=OK after the probe)
CURRENT KEY LOGIC (0x0120)        = DISABLED (0x0055)
CANDIDATE DISCHARGE LOGIC 0x005A  = NOT WRITTEN / NOT VALIDATED
DALY CONFIGURATION WRITE          = BLOCKED
PHYSICAL KEY SAFETY BARRIER       = STILL NOT VALIDATED — fused disconnect remains trusted isolation
```

Operator present and authorizing. Robot battery connected, fused disconnect closed, protected
domain, ESP32 step-down and servo rail powered, DALY connected, physical KEY in its normal ON
position (not toggled). The ASUS had no internet route during the session, so `origin` could not be
re-queried live; local branch/HEAD and the remote-tracking refs recorded at push time matched
(`a57fcdd`, `origin/main` `dd5746c`).

### Source and flash

| Step | Result |
|---|---|
| Source | branch `feat/daly-key-readonly-probe`, `a57fcddd29ac9bc7574520c86af57e183f43ae89`, clean tree |
| Static audit | **PASS** (32 files) |
| Clean rebuild, `MATDOG_PROFILE=ROBOT_POWERED` | manifest `SOURCE_STATE=CLEAN`, `BUILD_ID=a57fcddd29ac`, `HARDWARE_PROFILE=ROBOT_POWERED`, canonical FQBN; **byte-identical** to the earlier clean build: 389888 bytes, SHA-256 `7c0d5d35524177045ffc17ec890b131860b9ccf15c3d574ea9c500a4ebc4c6ba` |
| Pre-flash state (previous firmware `e2fc605`) | `@STATUS` / `@BMS STATUS`: ROBOT_POWERED, MAINTENANCE, DALY ONLINE `comm=OK`, 11.0 V, SOC 38.5 %, MOS ON/ON, alarms none, `runtime_resets=0` |
| `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh` | **`APPLICATION_ONLY_FLASH = PASS`** — backup, MAC `14:c1:9f:22:75:94`, static audit, clean manifest, FQBN, profile, rollback ENABLED / anti-rollback DISABLED, partition `app0 @ 0x010000`, size fits (389888 ≤ 3145728), one write, esptool hash verified, independent `verify-flash` digest matched |

### First boot (passive, zero bytes sent)

Complete banner: `MATDOG Controller 0.1.0`, `build a57fcddd29ac`, `ROBOT_POWERED (servo_power=YES
battery=YES led_rail=YES)`, `partition app0 @ 0x010000`, `reset_reason OTHER` (the USB-JTAG RTS
reset), `startup_motion` / `startup_torque` / `startup_servo_scan : DISABLED`, `daly_write :
NOT_IMPLEMENTED`, `operating_mode MAINTENANCE`, `IMU_INIT=PASS`, `SYSTEM_BOOT_COMPLETE
health=BOOTING power_state=RUN`, `runtime_resets=0`. No brownout, panic, watchdog or FAULT.

### Pre-probe baseline (one persistent no-reset session, 2.5 s drain first)

```text
MODE=MAINTENANCE
SYSTEM health=BOOTING power_state=RUN mode=MAINTENANCE uptime_ms=69553 profile=ROBOT_POWERED
DALY   init=OK detected=ONLINE expected=REQUIRED result=PASS
  comm=OK age_ms=1782
  pack_v=11.0 current_a=-0.1 soc=38.4% cells=3
  cell_max_mv=3676 cell_min_mv=3665 delta_mv=11
  charge_mos=ON discharge_mos=ON state=STATIONARY alarms=0000 0000 0000 0000
BMS_KEY_STATUS last_read=NOT_REQUESTED
  snapshot=NOT_READ key_logic=UNKNOWN
```

`health=BOOTING` is the by-design state before any servo probe (no census was run). A scripted
fail-closed gate (MAINTENANCE, DALY REQUIRED/PASS, `comm=OK`, plausible 3-cell pack, no alarms,
KEY cache `NOT_REQUESTED`/`NOT_READ`, ROBOT_POWERED) passed before the probe was sent.

### The single KEY read

`@BMS KEY READ` sent exactly once (no retry); complete result 0.36 s later:

```text
BMS_KEY_READ=STARTED
BMS_KEY_READ=COMPLETE result=OK
  key_logic_raw=0x0055 key_logic=DISABLED
  charge_mos_control=1
  discharge_mos_control=1
  sleep_time_raw=360 sleep_time_s=3600
```

Then `@BMS KEY STATUS` (cache only):

```text
BMS_KEY_STATUS last_read=OK age_ms=3782 rx_bytes=245
  snapshot=VALID age_ms=3782
  key_logic_raw=0x0055 key_logic=DISABLED
  charge_mos_control=1
  discharge_mos_control=1
  sleep_time_raw=360 sleep_time_s=3600
```

- The reply passed every check: 245 bytes, reply address `0x51`, function `0x03`, byte count
  `0xF0`, CRC-16/MODBUS — the `0x81` personality exists on this unit's RS485 port.
- `0x0055` decodes to DISABLED (BMSTool's "失能"): the KEY is not configured to switch any MOS,
  consistent with G3, where both KEY positions left `discharge_mos=ON`.
- Independent corroboration of the map: `0x0115 = 360` → 3600 s equals the manual's documented
  3600 s default sleep time; `0x0121`/`0x0122 = 1` match the phone app's "Charge switch ON" /
  "Discharge switch ON" (consistent with, not proof of, that app mapping).
- The cached status matches the asynchronous result; its age is consistent with no new transaction.

### Post-probe health

```text
DALY   init=OK detected=ONLINE expected=REQUIRED result=PASS
  comm=OK age_ms=395
  pack_v=11.0 current_a=-0.1 soc=38.4% cells=3
  cell_max_mv=3676 cell_min_mv=3665 delta_mv=11
  charge_mos=ON discharge_mos=ON state=STATIONARY alarms=0000 0000 0000 0000
SYSTEM health=BOOTING power_state=RUN mode=MAINTENANCE uptime_ms=81223 profile=ROBOT_POWERED
```

`age_ms=395` is a fresh `0xD2` sample taken after the KEY read — telemetry resumed on its own.
`runtime_resets` 0 before and after; heap unchanged (331224 / 325924); every availability line
identical to the baseline. Anomaly scan of the full raw capture (18828 bytes): no brownout, panic,
watchdog, FAULT, `UNKNOWN_COMMAND` or servo command output.

Commands sent this session: `@STATUS` and `@BMS STATUS` to the previous firmware before the flash;
after it `@MODE STATUS`, `@STATUS` ×2, `@BMS STATUS` ×2, `@BMS KEY STATUS` ×2, and `@BMS KEY
READ` ×1. **No DALY write, no KEY toggle, no MOS command, no `@SYSTEM SHUTDOWN`, no servo command,
no mode change, no motion.**

### Interpretation

The G3 KEY finding is explained by configuration, not wiring: the BMS reports KEY logic DISABLED.
The candidate MATDOG configuration — `0x0120 = 0x005A` (DISCHARGE: KEY OFF → discharge MOS OFF,
charge MOS kept) — is **not written and not validated**. It needs its own authorized session: a
reviewed write path, BMSTool's post-write BMS restart, a read-back, and a physical KEY test before
KEY can be considered any kind of power-off or safety barrier.
