# MATDOG Full Leg Calibrator V1

ESP32-S3-native calibration engine for the 12 MATDOG leg servos. First
current-architecture, **Station-free** calibrator: the ESP32-S3 owns the ST3215 bus and
every motion-safety decision.

**Status:** implementation complete, offline validated, H0 hardware smoke test
passed. **H1 and later have NOT been executed on hardware.**

The calibrator is functionally complete: H3 characterization, H4 joint
calibration, H5 leg and H6 four-leg orchestration are real code paths driving
the shared C++ engine, not refusal stubs. Default builds ship at H0; raising the
stage is one explicit build flag. See
[hardware validation handoff](../../09_Logs/Validation_Reports/Full_Leg_Calibrator_V1/MATDOG_FULL_LEG_CALIBRATOR_V1_HARDWARE_VALIDATION_HANDOFF.md).

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
| `matdog_full_leg_calibrator_v1/matdog_full_leg_calibrator_v1.ino` | ST3215 transport, hardware adapter, command parser, runtime safety |
| `matdog_full_leg_calibrator_v1/flc_stage_config.h` | the single build-stage / bootstrap switch, fail-closed by default |
| `matdog_full_leg_calibrator_v1/flc_contact_detector.h` | per-sample contact decision — pure C++ |
| `matdog_full_leg_calibrator_v1/flc_calibration_engine.h` | H3/H4/H5/H6 state machine: baseline, probe, contact, retreat, re-approach, repeatability, orchestration — pure C++ |
| `tests/flc_detector_harness.cpp` | host harness for the detector |
| `tests/flc_engine_harness.cpp` | host harness for the engine, driving a simulated servo with a real mechanical endstop |
| `tools/build_stage.sh` | the supported way to produce a stage-authorized image |

Both headers are compiled twice — into the firmware and into the host harnesses —
so the offline suite exercises the real motion logic rather than a Python model
of it. **There is exactly one implementation of the motion decision path.** The
Python simulator covers policy, census and session bookkeeping only.

### Layering

```text
flc_contact_detector.h    elementary per-sample contact decision
flc_calibration_engine.h  the state machine + leg/four-leg orchestration
matdog_..._v1.ino         transport, adapter, parser, runtime safety
```

## Command surface

Line protocol over USB CDC. Protocol id `FLC1`, scope
`CALIBRATOR_LOCAL_NOT_FINAL_RUNTIME_PROTOCOL` — this is the calibrator's own protocol,
**not** the final MATDOG runtime host↔ESP32 protocol, which remains TBD.

| Command | Stage | Writes | Purpose |
|---|---|---|---|
| `@STATUS` | H0 | none | version, build, gates, bootstrap state, per-joint characterization |
| `@CENSUS` | H1 | none | verify the 12 leg ids and every identity/profile invariant |
| `@CAPTURE_Q0 <n>` | H2 | none | `manual_pose_q0_candidate`, torque OFF, multi-sample |
| `@APPROVE_BOOTSTRAP CONFIRM` | H3 | none | arm the conservative first-motion envelope for this session |
| `@CHARACTERIZE_JOINT <id>` | H3 | RAM only | measure direction, baseline, contact, retreat, repeatability |
| `@CALIBRATE_JOINT <id>` | H4 | RAM only | both endpoints, span, derived q0 candidate |
| `@CALIBRATE_LEG <LF\|RF\|RH\|LH>` | H5 | RAM only | three joints, distal first |
| `@CALIBRATE_ALL` | H6 | RAM only | LF, RF, RH, LH — 12 joints, one session result |
| `@SAFE_OFF` | any | TorqueEnable=0 | unicast torque OFF + readback, idempotent |
| `@HELP` | — | none | command list |

"Writes: RAM only" means TorqueEnable (0x28), TorqueLimit (0x30) and the
`WritePosEx` RAM block. No EEPROM address is reachable from any of them.

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
H0  ESP32 only, no servos          ← default build, exercised 2026-08-28
H1  12-servo read-only census      ← later, operator present
H2  manual-pose q0 capture         ← later, operator present
H3  one joint characterization     ← explicit authorization + bootstrap approval
H4  one joint full calibration
H5  one complete leg
H6  four legs sequentially
H7  separate freeze/promotion gate — NOT part of this calibrator
```

The stage comes from one place, `flc_stage_config.h`, default `H0`. Produce a
validation image with `tools/build_stage.sh <stage> [--bootstrap]` rather than
editing sources.

### Two independent gates protect first motion

**Gate 1 — build stage.** `FLC_AUTHORIZED_STAGE` in `flc_stage_config.h`,
default `H0`. H3 is the lowest stage at which anything may move.

**Gate 2 — pre-motion parameters.** Satisfied either by resolved values or by an
explicitly approved bootstrap envelope, which needs **both** the build flag
`FLC_H3_BOOTSTRAP_APPROVED=1` and the live `@APPROVE_BOOTSTRAP CONFIRM`. A fresh
`@CENSUS` clears the session approval.

Raising the stage is **not** a bypass: identity, fresh census, measured direction
and every hard servo guard are enforced independently. A test asserts that H7
alone still refuses motion.

### How the H3 deadlock was resolved

Treating all eight contact parameters as pre-motion blockers was circular — H4
needed them, H3 was meant to measure them, H3 was blocked by them. They are now
classified by *when* a value can exist:

| Class | Meaning | May block motion |
|---|---|---|
| **A** pre-motion | needed to move at all | yes, unless bootstrap approved |
| **B** measured in H3 | H3 produces it | no |
| **C** derived | computed from H3/H4 data | no |
| **D** acceptance | judges a result post-measure | **never** |

`CONTACT_CURRENT_THRESHOLD_RAW` is **C**: the detector derives its threshold from
the per-joint free-motion median/MAD baseline H3 measures, so no fleet-wide
current constant exists. `ENDPOINT_VS_URDF_TOLERANCE_TICKS`,
`MANUAL_Q0_VS_DERIVED_Q0_TOLERANCE_TICKS` and
`CONTACT_REPEATABILITY_TOLERANCE_TICKS` are **D**: an unknown band leaves a
result `CANDIDATE` rather than blocking the measurement.

### The bootstrap envelope is not a measurement

| | Value | Comparison |
|---|---|---|
| TorqueLimit | 200 | below provisioner bench 300 and LF V25 500 |
| GoalSpeed | 60 | far below LF V25 160 |
| Acceleration | 8 | matches the slowest historical value |

Tagged `H3_BOOTSTRAP_OPERATOR_APPROVED`, never
`CHARACTERIZED_CURRENT_HARDWARE`. Reported by `@STATUS`, never canonical, never
written to EEPROM, and clamped by absolute ceilings a build flag cannot widen.

### Result tiers

```text
MEASURED -> CANDIDATE -> ACCEPTED -> PROMOTED
```

The calibrator can reach `ACCEPTED`. **`PROMOTED` is never reached here**:
writing into `MATDOG_JOINT_CALIBRATION.yaml` is a separate explicit gate.

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
./tools/build_stage.sh                  # H0, bootstrap denied (default, safe)
./tools/build_stage.sh 1 --upload       # H1 census image
./tools/build_stage.sh 3 --bootstrap --upload   # H3 characterization image
./tools/build_stage.sh 6 --bootstrap --upload   # H6 four-leg image
```

`build_stage.sh` is the only supported way to raise the stage: one flag, one
place, fail-closed by default. It refuses `--bootstrap` below H3 and prints a
warning for any image that can command motion.

Toolchain used: `arduino-cli 1.5.1`, `esp32:esp32 3.3.11`, `SCServo 1.0.2`.

## Test

```bash
cd 06_Software/Matdog_Core/calibration
python3 -m pytest tests/ -k full_leg_calibrator -q     # 206 offline tests
python3 -m pytest tests/ -q                            # 526, full calibration suite

python3 matdog_full_leg_calibrator_runner.py h0-smoke  # ESP32-only hardware test
```

The runner exposes named operations only — `status`, `census`, `capture-q0`,
`approve-bootstrap`, `characterize-joint`, `calibrate-joint`, `calibrate-leg`,
`calibrate-all`, `safe-off`, `h0-smoke`. There is deliberately no `--register`,
`--address` or `--raw-goal-position`: the host cannot compose a servo write.

Evidence: [Full_Leg_Calibrator_V1 validation reports](../../09_Logs/Validation_Reports/Full_Leg_Calibrator_V1/README.md).
