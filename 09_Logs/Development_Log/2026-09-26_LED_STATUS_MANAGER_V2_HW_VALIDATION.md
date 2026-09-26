# MATDOG LED Status Manager V2 — focused hardware validation

**Date:** 2026-09-26 · **Branch:** `feat/led-status-manager-v2-final`

**HARDWARE-VALIDATED for the focused checks below.** BOOTING breathing, READY SOC
mapping and the bounded SOC diagnostic passed. Live charging animations and reserved
battery-policy presentations remain unvalidated on hardware.

Evidence source: the operator-provided
`MATDOG_LED_V2_CLOSURE_ALIGNMENT_HANDOFF_2026-09-26.md`, including the reported flash
verification, command results and physical observations. This record documents that
completed session. The subsequent repository closeout is docs-only: no hardware or
serial access, rebuild, flash, calibration work, merge or tag operation is performed.

## Exact installed firmware and repository provenance

```text
SOURCE_COMMIT=88062e1a1217f288ebcc161c6213dbb543ea6f8a
BUILD_ID=88062e1a1217
HARDWARE_PROFILE=ROBOT_POWERED
OTA_INGEST_ENABLED=1
APPLICATION_PARTITION=app0
APPLICATION_OFFSET=0x010000
APPLICATION_SIZE=1029232
APPLICATION_SHA256=772a046e1b2d61888dba0ab7ddc759be13bbd4a33d1722b6be5285c313aac56b
```

Implementation history:

- `021cab7f70a4eb4c1c345ea58349ba3852416503`: SOC renderer and bounded diagnostics.
- `88062e1a1217f288ebcc161c6213dbb543ea6f8a`: reserved battery warning/critical facts;
  **the exact source commit built, physically flashed and validated**.
- The third commit, adding this record and aligning canonical docs, is docs-only.
  It is not the source of the installed firmware. Any eventual merge commit is
  also separate from this firmware provenance anchor.

Closeout started with `origin/main=f14aa40faf3c8c3fb9bace03dd8b71132f09edd5` and
feature HEAD/remote HEAD both at the flashed source commit above. The intended future
immutable tag `matdog-led-v2-focused-hw-validated-v1` must point to `88062e1...`, not
the docs commit or an eventual merge commit. Creating that tag is a later approved
phase; this closeout does not create or move it.

## Application-only installation — PASS

The operator-authorized application-only flash completed successfully:

```text
STATIC_AUDIT=PASS
DEVICE_MAC=14:c1:9f:22:75:94
ESP32=ESP32-S3 QFN56 rev 0.2
PSRAM=8MB
APPLICATION_PARTITION=app0
APPLICATION_OFFSET=0x010000
APPLICATION_SIZE=1029232
MAX_PARTITION_SIZE=3145728
APPLICATION_ONLY_FLASH=PASS
VERIFY_FLASH=PASS
```

The verified application digest is the SHA256 recorded above. Only `app0` was
written. Bootloader, partition table, boot_app0, NVS, servo EEPROM and DALY
configuration were untouched; no full flash occurred.

Recovery backup used by the flash gate (reference only; no binary added to Git):

```text
/home/matteo-manicardi/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-25_205220_nostub.bin
SIZE=16777216
SHA256=8758be21fea080c7a824315cd6c3df77d76de420f99a4265c3490a129b7bfe90
```

## A. Boot before servo preflight — PASS

```text
SYSTEM health=BOOTING power_state=RUN mode=MAINTENANCE authority=NONE
profile=ROBOT_POWERED
BNO085 result=PASS
DALY result=PASS
SERVO detected=UNKNOWN expected=REQUIRED result=UNKNOWN
LED result=PASS
Wi-Fi connected

presentation=BOOTING soc_valid=YES soc_percent=75.5 soc_segments=9
charging=NO charging_fault=NO charge_complete_verified=NO
battery_warning=NO battery_critical=NO diagnostic=NONE
```

The operator clearly observed BOOTING breathing. Firmware specifies white; the
perceived hue was white / slightly cyan-ish. This is a breathing-behavior PASS,
not a claim of calibrated chromatic accuracy.

ServoBus performs no automatic ping/scan at boot. Under ROBOT_POWERED it is REQUIRED,
so detection correctly remains UNKNOWN until an explicit read/preflight and system
health remains BOOTING. The LEDs did not change that system-state contract.

## B. Servo preflight and READY transition — PASS

The session executed `@SERVO PREFLIGHT`:

```text
SERVO_PREFLIGHT=PASS
profile=MATDOG_C018_V1
evaluated=12 pass=12 no_response=0 mismatch=0 incomplete=0
```

All twelve locomotion servos reported `model=777`, `position_offset=0`,
`persistent_profile=MATCH`, `torque_enable=0` and `result=PASS`.
No actuator motion or calibration motion was commanded; no Torque ON,
EEPROM/PositionOffset write or DALY configuration change occurred.

After preflight:

```text
SYSTEM health=READY power_state=RUN mode=MAINTENANCE authority=NONE
BNO085 PASS; DALY PASS; SERVO PASS; LED PASS
```

## C. READY SOC bar from real cached DALY telemetry — PASS

```text
presentation=READY soc_valid=YES soc_percent=75.5 soc_segments=9
charging=NO charging_fault=NO charge_complete_verified=NO
battery_warning=NO battery_critical=NO diagnostic=NONE

comm=OK pack_v=11.5 current_a=-0.2 soc=75.5% cells=3 cell_delta_mv=9
charge_mos=ON discharge_mos=ON state=STATIONARY
alarms=0000 0000 0000 0000
```

The operator observed **nine green LEDs from 12 o'clock through 8 o'clock inclusive**.
This matches `floor(75.5 * 12 / 100) = 9` and the frozen logical clockwise order:

```text
clock positions: 12 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 9 -> 10 -> 11
physical pixels: {1,2,3,4,5,6,7,8,9,10,11,0}
```

READY consumption of fresh cached SOC, origin at noon, clockwise physical ordering
and 75.5% to nine completed segments are hardware PASS. This observation does not
establish BMS capacity accuracy or true charge completion. **SOC 100% != true FULL.**

## D. Bounded `@LED SOC TEST` and automatic resume — PASS

Before the test, status was READY with `soc_valid=YES`, `soc_percent=75.2`,
`soc_segments=9`, `diagnostic=NONE`. The command returned `LED_SOC_TEST=STARTED`.
The operator physically confirmed:

```text
0/12 -> 1/12 -> ... -> 12/12
pause at full ring
12/12 -> 11/12 -> ... -> 0/12
```

Fill ran from noon clockwise through eleven o'clock; drain reversed that order.
The test terminated automatically and returned to the real nine-segment READY bar.
Afterward, status again showed READY, valid 75.2% SOC, nine segments and
`diagnostic=NONE`. System health remained READY, mode MAINTENANCE and authority NONE.

Fill, drain, bounded self-termination and automatic presentation ownership/resume
are hardware PASS. `@LED SOC TEST` intentionally displays fixed SOC bar frames;
it does **not** simulate `DALY state=CHARGING`. No pulsing next segment during this
diagnostic is expected and is not a failure.

## Validation boundary and remaining work

| Check | Result / boundary |
|---|---|
| Exact candidate application-only installation and digest verification | PASS |
| Candidate boot and BOOTING breathing | PASS; exact hue not calibrated |
| Servo preflight and torque state | PASS, 12/12; all `torque_enable=0` |
| READY transition after servo detection | PASS |
| Cached real SOC, 75.x% to nine segments, noon origin and clockwise mapping | PASS |
| SOC diagnostic fill/drain, self-termination and automatic READY resume | PASS |
| Actuator authority throughout validation | NONE |
| Real DALY CHARGING and next-segment breathing | TO_TEST; not exercised |
| Charging at reported 100%, final-segment breathing | TO_TEST; not exercised |
| CHARGING_FAULT from a real charging alarm | TO_TEST; not exercised |
| CHARGE_COMPLETE_VERIFIED from a real producer | NOT HARDWARE-VALIDATED; no producer |
| Battery warning / critical from real producers | NOT HARDWARE-VALIDATED; no producers |
| True charge-completion policy | FUTURE / TO_DESIGN |
| Autonomous dock / unattended charging qualification | FUTURE |
| Pre-existing RF/data-plane and OTA network end-to-end validation | OPEN; association alone does not close them |
| Full calibration workstream | OPEN and separate; no calibration motion performed |

`battery_warning`, `battery_critical` and `charge_complete_verified` remain reserved
presentation facts with no production producer. Their observed NO values do not
validate the corresponding active presentations. Previous offline tests/builds
remain the evidence for unexercised renderers; none were rerun by this docs closeout.

Implementation and offline contracts:
[`2026-09-26_LED_STATUS_MANAGER_V2_FINAL.md`](2026-09-26_LED_STATUS_MANAGER_V2_FINAL.md).
