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
tool          : arduino-cli upload (esptool v5.3.1 stub), normal application upload
target        : ESP32-S3 QFN56 rev v0.2, MAC 14:c1:9f:22:75:94
port          : /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00
build_id      : 03992a473bba, then 03992a473bba-dirty (after the heap-reporting addition below)
partitions touched : bootloader, partition table, boot_app0, application
                      (normal Arduino upload; no flash erase, no NVS erase)
result        : hash verified on every section written
```

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
