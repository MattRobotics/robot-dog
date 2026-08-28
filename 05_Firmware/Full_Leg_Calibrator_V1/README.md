# MATDOG Full Leg Calibrator V1

ESP32-S3-native calibration engine for the 12 MATDOG leg servos. First
current-architecture, **Station-free** calibrator: the ESP32-S3 owns the ST3215 bus and
every motion-safety decision.

**Status:** implemented, offline-validated, H0 hardware smoke test passed.
**Hardware motion (H3+) is LOCKED.** See [Safety gates](#safety-gates).

---

## Architecture

```text
ASUS host runner / evidence manager
   06_Software/Matdog_Core/calibration/matdog_full_leg_calibrator_runner.py
        ↕ USB CDC 115200 (native USB, D− GPIO19 / D+ GPIO20)
ESP32-S3 Full Leg Calibrator firmware
   05_Firmware/Full_Leg_Calibrator_V1/matdog_full_leg_calibrator_v1/
        ↕ UART1, GPIO17 TX / GPIO18 RX, 1 Mbps
Seeed Bus Servo Driver
        ↕ ST3215 serial bus
12 leg servos (ids 11,12,13,21,22,23,31,32,33,41,42,43)
```

NormaCore Station is **not** in the control path and is not a dependency. The host cannot
stream GoalPosition, cannot compose a servo write and cannot lift a gate; it requests a
named operation and records the result.

## Files

| File | Role |
|---|---|
| `matdog_full_leg_calibrator_v1/matdog_full_leg_calibrator_v1.ino` | firmware: command surface, census, q0 capture, write choke point, gates |
| `matdog_full_leg_calibrator_v1/flc_contact_detector.h` | bounded contact/endpoint state machine — pure C++, no Arduino dependency |
| `tests/flc_detector_harness.cpp` | host harness compiling the **same** header for offline fault injection |

`flc_contact_detector.h` is compiled twice — into the firmware and into the host test
harness — so the offline suite exercises the real engine rather than a Python model of
it. There is exactly one implementation of the contact state machine.

## Command surface

Line protocol over USB CDC. Protocol id `FLC1`, scope
`CALIBRATOR_LOCAL_NOT_FINAL_RUNTIME_PROTOCOL` — this is the calibrator's own protocol,
**not** the final MATDOG runtime host↔ESP32 protocol, which remains TBD.

| Command | Stage | Writes | Purpose |
|---|---|---|---|
| `@STATUS` | H0 | none | version, build, gates, safety state, outstanding characterization |
| `@CENSUS` | H1 | none | verify the 12 leg ids and every identity/profile invariant |
| `@CAPTURE_Q0 <n>` | H2 | none | `manual_pose_q0_candidate`, torque OFF, multi-sample |
| `@CALIBRATE_JOINT <id>` | H4 | gated | one joint, bounded endpoint search |
| `@CALIBRATE_LEG <LF\|RF\|RH\|LH>` | H5 | gated | three joints of one leg |
| `@CALIBRATE_ALL` | H6 | gated | LF, RF, RH, LH — 12 joints |
| `@SAFE_OFF` | any | TorqueEnable=0 | unicast torque OFF + readback, idempotent |
| `@HELP` | — | none | command list |

## Safety gates

### EEPROM-write-free by design

`writeAllowed()` is an address allowlist whose `default:` branch is `return false`. The
only writable addresses are:

- `0x28` TorqueEnable (RAM) — value 0 always; value 1 only inside an authorized motion
  transaction; **value 128 refused unconditionally** because `CalibrationOfs()` is
  implemented as `writeByte(ID, 0x28, 128)`;
- `0x30` TorqueLimit (RAM), only in the motion-prep stage;
- the RAM block written by `WritePosEx` (Acc / GoalPosition / GoalTime / GoalSpeed).

`PositionOffset` (`0x1F`), `ID` (`0x05`), `Lock` (`0x37`) and all 20 persistent-profile
registers are unreachable. `unLockEprom()`, `LockEprom()` and `CalibrationOfs()` are never
called. Enforced by `test_matdog_full_leg_calibrator_firmware_sync.py`.

### One GoalPosition authority

`flcWritePosEx()` is the only function that can move a servo. A test asserts exactly one
`st.WritePosEx(` call site exists, and that `RegWritePosEx`, `SyncWritePosEx`, `WheelMode`
and `WriteSpe` appear nowhere.

### No broadcast

Every write takes a concrete unicast id validated by `validLegId()`. Broadcast id 254
cannot satisfy it.

### Unsigned 0..4095

`GoalPosition` is validated in `[0, 4095]` before `WritePosEx`, so its negative
sign-magnitude branch is unreachable. There is no modulo/wrap trick: a planned move that
would cross the 0/4095 boundary is **refused**, not wrapped.

### Progressive hardware stages

```text
H0  ESP32 only, no servos          ← shipped, exercised 2026-08-28
H1  12-servo read-only census      ← later, user present
H2  manual-pose q0 capture         ← later, user present
H3  one joint characterization     ← explicit authorization required
H4  one joint full calibration
H5  one complete leg
H6  four legs sequentially
H7  final 12/12 freeze
```

`AUTHORIZED_STAGE = H0_ESP32_ONLY`. Raising it is a deliberate source change.

### Characterization gate — why H3+ is locked

Motion additionally requires **every** safety-critical contact parameter to be resolved.
Eight are currently `UNRESOLVED`:

`CONTACT_TORQUE_LIMIT`, `CONTACT_GOAL_SPEED`, `CONTACT_ACCELERATION`,
`CONTACT_CURRENT_THRESHOLD_RAW`, `CONTACT_RETREAT_TICKS`,
`CONTACT_REPEATABILITY_TOLERANCE_TICKS`, `ENDPOINT_VS_URDF_TOLERANCE_TICKS`,
`MANUAL_Q0_VS_DERIVED_Q0_TOLERANCE_TICKS`.

None of these can be filled from history. The LF V25 values describe the previous
installation; the Provisioner V6 centering values (TorqueLimit 300, speed 365, acc 50)
were validated for **bench free-shaft centering with no mechanical load**, which says
nothing about driving an assembled leg into a mechanical endstop.

The stage gate and the characterization gate are **independent**: raising the stage to H7
alone still refuses motion.

## Constant provenance

Every safety-relevant constant is tagged:

| Tag | Meaning | May authorize motion |
|---|---|---|
| `[VALIDATED]` | current-hardware invariant, evidence cited | yes |
| `[GENERIC]` | load-independent mechanism from frozen MATDOG ESP32 tooling | yes |
| `[HISTORICAL]` | LF V25 / bench-centering, previous installation | **no** |
| `[CHARACTERIZE]` | no current evidence | **no — blocks motion** |

LF V25 numbers are present but quarantined behind a `HIST_` prefix and wired to nothing.

## Reuse

Adapted from the frozen bench tooling (`05_Firmware/ST3215_Bench_Tools/`, unmodified):

- UART transport, pin map and 1 Mbps setup (Source Signature Survey V1);
- the 71-byte `0x00..0x46` state snapshot and its register offsets;
- the **write choke point with a stage-scoped allowlist** and write/readback verification
  (Provisioner V6);
- torque-OFF recovery discipline and the `@`-prefixed line protocol;
- monitoring cadence and thermal/voltage/telemetry-loss guards (QC V6.1).

Adapted conceptually from LF V25 (`09_Logs/Historical/.../LF_V25_Hardware_Oracle/`):

- hybrid multi-channel contact evidence with persistence;
- startup grace window after each new commanded target;
- median/MAD free-motion current baseline;
- bounded retreat and re-approach for repeatability;
- affine q0 derivation and endpoint/URDF cross-check.

Deliberately rejected: the Station runtime architecture, Station Python bindings, and
every LF V25 numeric hardware result.

## Build and flash

```bash
FQBN='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi'
PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00

arduino-cli compile --fqbn "$FQBN" --export-binaries matdog_full_leg_calibrator_v1
arduino-cli upload -p "$PORT" --fqbn "$FQBN" matdog_full_leg_calibrator_v1
```

Toolchain used: `arduino-cli 1.5.1`, `esp32:esp32 3.3.11`, `SCServo 1.0.2`.

## Test

```bash
cd 06_Software/Matdog_Core/calibration
python3 -m pytest tests/ -k full_leg_calibrator -q          # 135 offline tests
python3 matdog_full_leg_calibrator_runner.py h0-smoke       # ESP32-only hardware test
```

Evidence: [Full_Leg_Calibrator_V1 validation reports](../../09_Logs/Validation_Reports/Full_Leg_Calibrator_V1/README.md).
