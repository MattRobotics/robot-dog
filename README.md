# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The project covers the complete robot stack: mechanical design, 3D-printed structure, power distribution, twelve Feetech ST3215 serial-bus servos, CAD-derived kinematics, calibration, locomotion, embedded control and future perception/autonomy.

## Project scope

MATDOG is not only a calibration project. This repository is the engineering source of truth for:

- mechanical architecture, CAD, URDF and collision geometry;
- electronics, power distribution, battery/BMS integration and servo wiring;
- servo mapping, joint conventions and calibration records;
- forward/inverse kinematics, contact geometry and trajectory validation;
- gait, balance, stand-up and future locomotion control;
- embedded-compute, sensing, watchdog and autonomy planning;
- validation reports, decisions and reproducible project evidence.

## System architecture

```text
MATDOG mechanical platform
→ 12 × Feetech ST3215 serial-bus servos
→ custom power-distribution and protection
→ Waveshare Bus Servo Adapter
→ NormaCore Station/ST3215 runtime
→ MATDOG calibration, kinematics and locomotion software
→ future embedded controller, Jetson, IMU and perception stack
```

### Repository responsibilities

```text
MattRobotics/robot-dog
→ public MATDOG source of truth: CAD, URDF, electronics, calibration evidence,
  kinematics, locomotion, validation and project decisions

MattRobotics/norma-core
→ Station/ST3215 integration fork and native MATDOG calibration runtime
```

Private research material is intentionally excluded from this public repository and is not part of the MATDOG runtime or public technical baseline.

## Current validated state

| Area | Status |
|---|---|
| Mechanical architecture and REV00 CAD/URDF | Validated |
| Twelve-servo bus, sparse IDs, mapping and directions | Validated |
| Digital-home commissioning and EEPROM readback | Validated for all 12 servos |
| Encoder-to-radian conversion and live read-only FK | Validated for all four legs |
| Offline contact, collision, timing and support references | Validated as engineering references |
| Mechanical end-stop calibration | **LF only: V25 hardware validated and frozen** |
| RF, RH and LH mechanical calibration | Not yet hardware validated |
| Complete 12-joint persistent profile | Not yet complete |
| Stand-up and locomotion | Pending post-calibration regeneration and validation |
| Embedded/autonomy integration | Planned |

> **Important:** the only mechanically hardware-validated calibration program is **LF V25**. Older V28–V42 experiments and previous “all legs” concepts are historical development records, not current programs and not valid development bases.

## Current milestone — LF V25 calibrated and frozen

On 4 August 2026 the complete left-front leg calibration completed successfully.

```text
58/58 sequence complete
6/6 LF mechanical contacts accepted
URDF affine gate PASS
supervised hardware-witness gate PASS
RAM q0 staging PASS
Station shutdown and serial release PASS
transactional EEPROM freeze PASS
persistent LF profile PASS
global torque OFF verified
```

Canonical technical record:

```text
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

Exact validated NormaCore source:

```text
branch: release/matdog-lf-calibrator-v25
source head: f87dd1fbc7e8100d275c74f9af448642f3429680
implementation PR: MattRobotics/norma-core#11
```

### LF final result

| Joint | Motor | MIN contact | MAX contact | Affine q0 before EEPROM | Final displayed q0 |
|---|---:|---:|---:|---:|---:|
| LF hip | M13 | 2535 | 1600 | 2067 | 2048 |
| LF upper | M12 | 1439 | 3443 | 2040 | 2051 |
| LF lower | M11 | 3093 | 1658 | 2074 | 2046 |

Frozen ST3215 Position Offsets:

| Motor | Previous | Frozen |
|---|---:|---:|
| M11 | 101 | 127 |
| M12 | 859 | 851 |
| M13 | -505 | -486 |

LF V25 is now a frozen reference leg. It must not be rerun unless LF mechanics, servo, mounting, URDF or calibration state changes.

## Robot definition

Canonical REV00 package:

```text
03_CAD/URDF/matt_robodog_rev00/
```

The package includes the canonical URDF, visual/collision meshes, workbook, URDF Studio project and integrity manifest.

### Geometry

```text
front-to-rear hip spacing: 225 mm
left-to-right hip spacing: 95 mm
hip-to-knee segment: 90 mm
knee-to-foot mechanical interface: 110 mm
knee-to-contact-frame distance: 118.1 mm
target nominal body height: about 150 mm
```

Coordinate convention:

```text
X = forward
Y = left
Z = up
units = metres and radians
right-handed frame
```

### Servo mapping

```text
LF: M13 hip, M12 upper, M11 lower
RF: M23 hip, M22 upper, M21 lower
RH: M33 hip, M32 upper, M31 lower
LH: M43 hip, M42 upper, M41 lower
```

Canonical leg order and trot diagonals:

```text
[LF, RF, RH, LH]
LF + RH
RF + LH
```

## Permanent control and calibration rules

- Station is the sole ST3215 serial owner during motion.
- ST3215 `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- Digital-home commissioning remains separate from mechanical endpoint calibration.
- Calibration measures physical endpoints and derives the joint model from the URDF; it does not silently redefine CAD geometry.
- Every contact must satisfy repeatability, model consistency and supervised hardware evidence.
- EEPROM writes are allowed only after complete measurement PASS, verified Station shutdown and serial release.
- EEPROM provisioning is transactional and must include backup, readback, relock and rollback on failure.
- RF/RH/LH must be generalized from merged V25 architecture through data-driven leg profiles, not by reviving old versioned programs.

## Roadmap

### Foundation

- [x] Mechanical architecture and REV00 CAD/URDF
- [x] ST3215 bus, mapping and directions
- [x] Twelve-servo digital-home commissioning
- [x] Encoder/radian conversion and four-leg read-only FK
- [x] Offline contact, collision, timing and support references
- [x] Native Station-mediated calibration foundation

### Mechanical calibration

- [x] LF six-contact calibration
- [x] LF affine q0 derivation and URDF gate
- [x] LF transactional EEPROM freeze
- [x] LF persistent profile
- [ ] Generalize the validated V25 architecture to RF from merged `norma-core/main`
- [ ] Calibrate and freeze RF
- [ ] Calibrate and freeze RH
- [ ] Calibrate and freeze LH
- [ ] Validate the complete twelve-joint persistent profile

### Locomotion

- [ ] Recompute four-leg FK with all frozen profiles
- [ ] Regenerate HOME → LOW_STAND → NOMINAL_STAND
- [ ] Repeat collision/contact/support audit with calibrated limits
- [ ] Supervised suspended stand
- [ ] Gradual load transfer and nominal stand
- [ ] Single-foot swing trajectory
- [ ] Trot in place
- [ ] First slow walking tests

### Embedded integration and autonomy

- [ ] Battery and smart BMS integration
- [ ] Embedded motion-controller evaluation
- [ ] Jetson integration
- [ ] IMU, estimator and watchdog integration
- [ ] Depth vision, object detection, voice and autonomous behaviour

## Repository structure

```text
01_Docs/        architecture and stable technical references
02_BOM/         components, suppliers and costs
03_CAD/         CAD, URDF, meshes and mechanical exports
04_Electronics/ wiring, power and servo mapping
05_Firmware/    future low-level firmware
06_Software/    calibration, kinematics, gait and control
07_Media/       images, renders and videos
08_Tests/       repeatable validation procedures
09_Logs/        decisions, reports and historical evidence
```

## Development policy

- `main` is the only active branch in `robot-dog` after milestone cleanup.
- New calibration development starts from the current merged architecture.
- At most one clearly named active calibration branch should exist while a milestone is under development.
- Temporary version-numbered workflows and branches must be removed after closeout.
- Closed PRs preserve historical experiments; obsolete branches are not retained as operational choices.
- Public documentation must describe MATDOG itself and must not expose unrelated private research material.

---

Built and documented by Matt Robotics.
