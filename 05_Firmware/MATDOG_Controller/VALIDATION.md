# MATDOG Controller V0.1 — Validation

Vocabulary follows the repository convention: **VALIDATED** (implemented and
exercised on real hardware), **IMPLEMENTED** (built, compiles, not yet — or not
fully — hardware-exercised), **DECIDED** (architectural decision, not yet built),
**NOT YET VALIDATED**, **TBD**.

This document will be updated in place as hardware sessions progress; it is not
rewritten per session.

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
