# CR2-C — Q0 hardware session

**Date:** 2026-09-27
**Branch:** `feat/calibration-readiness-v1`
**Authorization:** RECEIVED from operator
**Authorization phrase:** `AUTORIZZO CR2-C READ-ONLY Q0 HARDWARE SESSION`
**Execution status:** COMPLETE / CR2-C PASS

## Authorized scope

The operator explicitly authorized the CR2-C hardware session prepared in
`CR2C_Q0_HARDWARE_READONLY_RUNBOOK.md`.

The authorized objective is the first current-installation, Torque-OFF, no-motion acquisition of
12 q0 candidates.

Required enabling steps such as recovery verification, exact ROBOT_POWERED build provenance and
application-only flashing remain subject to their existing fail-closed script gates and must be
executed exactly as documented. Authorization does not waive any gate.

## Permanent constraints for this session

```text
NO Torque ON
NO GoalPosition
NO automatic motion
NO EEPROM write
NO PositionOffset write
NO CalibrationOfs
NO servo re-ID
NO merge to main
NO force push / history rewrite
SAFE_OFF remains independent
hardware_motion_authorized remains false
```

## Current state at authorization

CR2-A: OFFLINE VALIDATED
CR2-B: OFFLINE + BUILD VALIDATED
CR2-C: AUTHORIZED / PRE-FLIGHT

No hardware action had been performed by this CR2-C session at the time this log was created.

## Power-topology correction before execution

Before the operator executed the first CR2-C command block, they identified an incorrect
assumption in the initial runbook/guidance: MATDOG does not have an independently powered ESP32
with a switchable/dead servo rail.

Current validated hardware truth:

```text
battery + main fuse/disconnect + DALY KEY ON
  -> DALY protected B+/P- domain energized
  -> ESP32-S3 / TECNOIOT 5 V powered
  -> servo rail powered
  -> LED rail powered
```

This is the intended post-2026-09-24 wiring and is consistent with the existing G3
`ROBOT_POWERED` validation.

CR2-C safety was therefore corrected from:

```text
controller powered + servo rail 0 V
```

to:

```text
ROBOT_POWERED domain energized
+ charger disconnected
+ robot mechanically supported
+ all 13 installed servos VERIFIED_OFF before esptool reset/read/flash
+ no Torque ON / GoalPosition / motion path
```

No hardware command had been executed before this correction. The runbook was amended before
continuing.

## CR2-C.1 — pre-flash powered SAFE_OFF verification

**Result: PASS**

The operator executed the no-reset native USB CDC session with the robot in the validated
`ROBOT_POWERED` topology.

Observed before SAFE_OFF:

```text
SYSTEM health=BOOTING power_state=RUN mode=MAINTENANCE authority=NONE
profile=ROBOT_POWERED
runtime_resets=0
```

The following 13 installed servo IDs were then commanded through the existing de-escalation-only
`@SERVO SAFE_OFF` path:

```text
11 12 13 21 22 23 31 32 33 41 42 43 51
```

Every one returned:

```text
result=VERIFIED_OFF
```

Terminal result:

```text
CR2C_SAFE_OFF=PASS verified=13/13 ids=11,12,13,21,22,23,31,32,33,41,42,43,51
```

Final cached system state:

```text
SYSTEM health=READY power_state=RUN mode=MAINTENANCE authority=NONE
SERVO init=OK detected=ONLINE expected=REQUIRED result=PASS
runtime_resets=0
```

No servo motion was reported. No Torque ON, GoalPosition, EEPROM, PositionOffset or CalibrationOfs
operation was performed.

The powered bus is therefore in the required pre-esptool safe state for CR2-C.2 recovery backup.

## CR2-C.2 — fresh full-flash recovery backup

**Result: PASS (backup acquisition)**

The operator performed a full 16 MiB ROM-loader read with the already validated no-stub method:

```text
READ_METHOD=NO_STUB_FULL
offset=0x000000
size=0x1000000
baud=115200
```

Observed result:

```text
BACKUP_PATH=/home/matteo-manicardi/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-27_160127_nostub.bin
READ_BYTES=16777216
ESPTOOL_RC=0
BACKUP_SIZE=16777216
BACKUP_SHA256=339c01f7805b9f44c113074bf9a82b460e14b8fe55e6796ca25b0f24323cb7ce
READ_DURATION=1580.1 s
READ_RATE=84.9 kbit/s
```

The dump reached address `0x01000000` and esptool completed normally, then reset the board via
RTS. The explicit terminal convenience line `CR2C_BACKUP=PASS` was not captured in the pasted
transcript, but the underlying gate facts (RC=0 and exact 16 MiB size) are present and controlling.

No flash write was performed during CR2-C.2.

Offline structural validation and creation of the companion recovery manifest remain the next
sub-gate before any build/flash action.


## CR2-C.2b - recovery structural gate

**Result: PASS**

Fresh 16 MiB dump SHA256:
`339c01f7805b9f44c113074bf9a82b460e14b8fe55e6796ca25b0f24323cb7ce`.
Partition table/otadata validation selected `app0 @ 0x010000`; the recovery manifest passed
`backup_gate_logic.py`.

## CR2-C.3 - exact ROBOT_POWERED build gate

**Result: PASS**

```text
SOURCE_COMMIT=315d4ade6ff0de59f6f3032f9864accb1680c669
BUILD_ID=315d4ade6ff0
HARDWARE_PROFILE=ROBOT_POWERED
OTA_INGEST_ENABLED=0
APPLICATION_SIZE=1036560
APPLICATION_SHA256=753936ac1dc12d4af11f6ffaed51a8aaeda5b76f260e35e188a3d065f9f52b59
```

Static audit, all host suites and the explicit ROBOT_POWERED build passed.

## CR2-C.4 - application-only flash

**Result: PASS**

The reviewed script wrote only the selected `app0` application partition and verify-flash
confirmed the digest. No bootloader, partition table, NVS or servo EEPROM write occurred.

## CR2-C.5 - post-flash identity and safe-state gate

**Result: PASS**

Running build `315d4ade6ff0`, ROBOT_POWERED, `app0 @ 0x010000`, MAINTENANCE, authority NONE,
hardware motion BLOCKED, null production ActuatorBackend and q0 IDLE were all confirmed. All 13
installed servos returned `VERIFIED_OFF`.

## CR2-C.6 - current-installation q0 capture

**Result: PASS**

Operator established nominal URDF q=0 manually using square/jigs and mechanical support; raw 2048
was not used as the placement target. Command:
`@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE`.

Fresh population evidence passed 12/12. Twelve CANDIDATE q0 records were produced with spread 0
ticks on every joint. Exact values are preserved in the validation package.

## CR2-C.7 - final closeout

**Result: PASS**

```text
SYSTEM health=READY
mode=MAINTENANCE
authority=NONE
runtime_resets=0
hardware_motion=BLOCKED
q0=12_CANDIDATES_ONLY
SAFE_OFF=13/13 VERIFIED_OFF
pack_v=11.3 V
soc=66.3 %
cell_delta=9 mV
alarms=0000 0000 0000 0000
```

Opening pack voltage was not captured in a dedicated BMS snapshot and is recorded as NOT_CAPTURED,
not reconstructed.

## Final CR2-C verdict

**CR2-C = PASS.**

Current 12-joint population and current-installation q0 have been measured under Torque OFF with
current identity/geometry provenance. No evidence was accepted/promoted and no motion was
authorized. CR3 starts from the immutable evidence package.
