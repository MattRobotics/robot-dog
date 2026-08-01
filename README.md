# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The project combines mechanical design, 3D printing, ST3215 serial-bus servos, kinematics, native calibration, gait generation, embedded control and future perception capabilities.

## Current Engineering Milestone — 2026-08-01

The complete native mechanical-calibration program is implemented and verified offline.

A single Station **Auto Calibrate** action can now execute:

```text
LF → RF → RH → LH
24 mechanical contacts
12 model-derived q=0 targets
complete calibrated software HOME
verified global torque OFF
```

The final program is intentionally still draft and hardware-gated. No merge into `main` has been performed.

## Validated foundation

- 4 legs, 17 links, 16 joints and 12 revolute leg joints;
- canonical REV00 CAD-derived URDF and collision meshes;
- exact sparse ST3215 topology: `11,12,13,21,22,23,31,32,33,41,42,43`;
- canonical servo mapping and encoder-to-radian direction contract;
- four-leg live FK through Station telemetry;
- offline IK/contact closure and exact-mesh collision audit;
- historical 12-servo EEPROM digital-zero calibration and read-back;
- Station-owned RAM-only mechanical-contact calibrator;
- repeated contact, backoff, recovery and global torque-OFF verification.

## Canonical servo mapping

```text
LF: hip M13, upper M12, lower M11
RF: hip M23, upper M22, lower M21
RH: hip M33, upper M32, lower M31
LH: hip M43, upper M42, lower M41
```

Canonical calibration order:

```text
LF → RF → RH → LH
```

## LF hardware evidence

All six LF mechanical contacts completed supervised acquisition:

| Joint | Servo | MIN | MAX | Spread |
|---|---:|---:|---:|---:|
| upper | M12 | 1443 / 1443 | 3443 / 3442 | 0 / 1 ticks |
| lower | M11 | 3094 / 3092 | 1664 / 1666 | 2 / 2 ticks |
| hip | M13 | 2530 / 2530 | 1595 / 1595 | 0 / 0 ticks |

The LF HIP combined cycle completed:

```text
MIN → HOME → MAX → HOME
Done 20/20
global torque OFF verified
serial port released
```

The first HIP MAX approach was visibly too slow and probably under-seated. The later full-leg programs therefore use a faster, firmer but still bounded motion envelope.

## Model-derived q=0

The historical displayed HOME at encoder tick 2048 was initially established by visual/mechanical alignment. It remains preserved, but it is no longer assumed to be the final kinematic zero.

For each joint, the program derives two independent candidates from the measured mechanical endpoints and the fixed REV00 URDF model:

```text
zero_from_MIN = measured_MIN - direction × URDF_MIN_delta
zero_from_MAX = measured_MAX - direction × URDF_MAX_delta
```

The encoder scale and direction are not refitted.

Acceptance requires:

```text
circular_distance(zero_from_MIN, zero_from_MAX) <= 24 ticks
circular_distance(estimated_zero, 2048) <= 96 ticks
```

On failure:

```text
MODEL_ZERO_INCONSISTENT <LEG>
contacts remain logged
next leg does not start
calibrated HOME is not applied
global torque OFF verified
```

Current historical LF evidence diagnoses:

| Joint | zero from MIN | zero from MAX | Disagreement | Result |
|---|---:|---:|---:|---|
| M12 upper | 2040 | 2048 | 8 | consistent candidate 2044 |
| M11 lower | 2046 | 2092 | 46 | re-acquire with V38 |
| M13 hip | 2018 | 2107 | 89 | re-acquire with V38 |

This demonstrates why repeatability alone does not prove full mechanical seating.

## V38 — complete LF program

```text
NormaCore branch: matdog/full-calibration-v38
MATDOG record branch: matdog/full-calibration-program-v38
arm token: LF_LEG_FULL_V38
```

One Auto Calibrate executes:

```text
M12 UPPER MIN/MAX
M11 LOWER MIN/MAX
M13 HIP MIN/MAX
three URDF endpoint-consistency gates
LF calibrated software q=0 placement
global torque OFF
```

Offline verification:

```text
100/100 ST3215 tests: PASS
Station viewer build: PASS
Station release build: PASS
```

## V39 — complete LH rear-leg program

```text
NormaCore branch: matdog/lh-full-calibration-v39
MATDOG record branch: matdog/lh-full-calibration-program-v39
arm token: LH_LEG_FULL_V39
```

The exact-mesh checkpoint of 2026-07-20 already proved that an active rear leg does not require any front-leg parking. Therefore LH does **not** move LF forward.

LH HIP prerequisites:

```text
M42 = 3072  # upper horizontal
M41 = 3038  # lower folded/parallel
M43 = probing hip
```

Offline verification:

```text
104/104 ST3215 tests: PASS
Station viewer build: PASS
Station release build: PASS
```

## V40 — complete 24-contact, 12-joint program

```text
NormaCore branch: matdog/all-legs-full-calibration-v40
MATDOG record branch: matdog/all-legs-full-calibration-program-v40
arm token: MATDOG_ALL_LEGS_FULL_V40
```

A single Auto Calibrate action executes:

```text
LF six contacts → LF model-zero gate
RF six contacts → RF model-zero gate
RH six contacts → RH model-zero gate
LH six contacts → LH model-zero gate
12 accepted software q=0 targets
all HIP joints → all UPPER joints → all LOWER joints
verify all 12 holds
global torque OFF
```

### Why final HOME is delayed

During the 24 contact acquisitions, every stage returns to the historical digital HOME and turns torque off. A newly derived q=0 is stored in memory but is not applied immediately, because the restart-safe gate for the next leg still expects the common digital reference.

The 12 calibrated targets are applied only after all four legs pass. This prevents an earlier calibrated leg from invalidating the startup envelope of a later leg.

### Front and rear geometry

- LF and RF front-leg profiles retain the validated ipsilateral rear-UPPER parking prerequisite.
- LF retains the hardware-validated combined HIP stage.
- RH and LH rear-leg profiles require no additional front-leg parking.

### Offline verification

```text
workflow: MATDOG All Legs Full Calibration V40
run: 30696228454
108/108 ST3215 tests: PASS
Station viewer build: PASS
Station release build: PASS
order, fail-closed and safety contracts: PASS
hardware_started=false
serial_opened=false
```

Artifact:

```text
id: 8817381318
digest: sha256:e6868d89cdd2473485b4469fc804f1091cf96587b365aa7c84228953db595ede
matdog.rs: a955f7de9a1c3405cf4d4e705d545e499162ba3cb378261bb9ca7afcf53999b7
matdog_test.rs: 23d60c377ee40b8f71c8969989b70e8098b0ae7126392d81af4e631f347ee696
Station: a5d2bd00ad90ad3c4fc3268f52a847dce390a040042d51a7633de89b7b70ff9c
```

## Motion envelope

```text
TorqueLimit = 500
GoalSpeed   = 160
Acc         = 8
coarse step = 64 ticks
fine step   = 8 ticks
settle      = 900 ms
hard-current abort = 200
```

The faster envelope addresses under-seating without enlarging URDF-derived contact corridors or 64-tick mechanical guards.

## Permanent safety contract

- Station is the only serial owner.
- `GoalPosition` remains standard unsigned ST3215.
- Writes remain RAM-only: TorqueEnable, Acc, GoalPosition, GoalSpeed and TorqueLimit.
- Status, driver-error and hard-current aborts remain active.
- Every contact stage ends with verified global torque OFF.
- Final calibrated HOME ends with verified global torque OFF.
- No EEPROM, Position Offset, LOCK, reset, ResetCalibration, RegWrite, Action, Save or Freeze.
- Any failed stage blocks the next stage.
- The first V38, V39 and V40 runs remain supervised with robot supported, all legs free, operator present and master disconnect accessible.

## Hardware rollout gate

```text
1. LF_LEG_FULL_V38 hardware model-zero PASS
2. LH_LEG_FULL_V39 hardware model-zero PASS
3. MATDOG_ALL_LEGS_FULL_V40 first supervised full run
4. update canonical YAML with 12 accepted software q=0 values
5. four-leg post-calibration FK and collision closure
6. regenerate HOME → LOW_STAND → NOMINAL_STAND trajectories
```

The complete V40 program is implemented, but its launcher remains hard-blocked until the two individual LF and LH checkpoints exist in the evidence archive.

## Canonical REV00 geometry

```text
front-to-rear hip spacing: 225 mm
left-to-right hip spacing: 95 mm
hip-to-knee segment: 90 mm
knee-to-foot mechanical interface center: 110 mm
knee-to-foot contact-frame distance: 118.1 mm
target stand body height: approximately 150 mm
```

Coordinate convention:

```text
X = forward
Y = left
Z = up
units = metres and radians
right-handed coordinate system
```

Canonical URDF package:

```text
03_CAD/URDF/matt_robodog_rev00/
```

## Calibration records

```text
06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml
06_Software/Matdog_Core/calibration/MATDOG_FULL_CALIBRATION_PROGRAM_V38.md
06_Software/Matdog_Core/calibration/MATDOG_ALL_LEGS_FULL_CALIBRATION_V40.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CONTACT_EVIDENCE_2026-08-01.yaml
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
06_Software/Matdog_Core/calibration/matdog_model_zero_solver.py
06_Software/Matdog_Core/calibration/tests/test_matdog_model_zero_solver.py
```

Historical digital-zero records remain preserved in:

```text
06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md
09_Logs/Calibration/C5_R_digital_recenter/
```

## Repository structure

```text
01_Docs/        architecture and technical references
02_BOM/         bills of materials
03_CAD/         CAD, URDF and meshes
04_Electronics/ wiring, power and servo mapping
05_Firmware/    embedded firmware
06_Software/    calibration, kinematics, gait and control
07_Media/       project media
08_Tests/       validation procedures and tests
09_Logs/        evidence and development history
```

## Roadmap

### Completed

- [x] Mechanical architecture and REV00 URDF
- [x] ST3215 bus and canonical mapping
- [x] Historical 12-servo digital zero
- [x] Four-leg FK and offline contact/collision model
- [x] Native 24-contact calibrator foundation
- [x] LF six-contact hardware evidence
- [x] Independent model-zero solver
- [x] LF V38 offline implementation and CI
- [x] LH V39 offline implementation and CI
- [x] Complete V40 offline implementation and CI

### Immediate hardware checkpoints

- [ ] LF V38 model-zero PASS
- [ ] LH V39 model-zero PASS
- [ ] Complete V40 supervised PASS
- [ ] Save 12 accepted software q=0 values in canonical YAML
- [ ] Post-calibration four-leg FK/collision closure

### Locomotion

- [ ] Regenerate calibrated HOME and stand trajectories
- [ ] Supervised suspended stand
- [ ] Four-leg body-height control
- [ ] Single-foot swing trajectory
- [ ] Trot in place
- [ ] First slow walking tests

---

Built and documented by Matt Robotics.
