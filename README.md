# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. It combines a CAD-derived mechanical design, 3D-printed structure, twelve Feetech ST3215 serial-bus servos, NormaCore Station integration, calibrated kinematics and a future modular gait/embedded-control stack.

## Current milestone — LF calibrated and frozen

On 4 August 2026 the complete left-front leg calibration completed successfully with the validated **MATDOG LF calibrator V25**.

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

The exact NormaCore source is frozen in:

```text
MattRobotics/norma-core
release/matdog-lf-calibrator-v25
source head: f87dd1fbc7e8100d275c74f9af448642f3429680
PR: MattRobotics/norma-core#11
```

Canonical technical record:

```text
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

## LF calibration result

| Joint | Motor | MIN contact | MAX contact | Affine q0 before EEPROM | Final displayed q0 |
|---|---:|---:|---:|---:|---:|
| LF hip | M13 | 2535 | 1600 | 2067 | 2048 |
| LF upper | M12 | 1439 | 3443 | 2040 | 2051 |
| LF lower | M11 | 3093 | 1658 | 2074 | 2046 |

Measured mechanical spans versus the CAD/URDF model:

| Joint | URDF span | Measured span | Difference |
|---|---:|---:|---:|
| M13 hip | 90.00° | 82.18° | −7.82° |
| M12 upper | 174.99° | 176.13° | +1.14° |
| M11 lower | 129.55° | 126.12° | −3.43° |

Frozen ST3215 Position Offsets:

| Motor | Previous | Frozen |
|---|---:|---:|
| M11 | 101 | 127 |
| M12 | 859 | 851 |
| M13 | −505 | −486 |

The persistent affine profile is the source of truth for LF joint-state conversion and future motion planning. Digital `2048` now represents the calibrated physical `q = 0` within the final readback residual of 0–3 ticks.

## Permanent calibration rules

- Station is the sole ST3215 serial owner during motion.
- ST3215 `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- The initial digital-home commissioning program remains separate from mechanical leg calibration.
- Calibration measures physical endpoints and derives q0 from the URDF; it does not redefine the CAD geometry silently.
- A single speed sample is not proof that a held joint moved; real position drift and state integrity are authoritative.
- Bounded friction/chamfer plateaus may be crossed only when a deeper coarse scout already proved that travel.
- Every contact must satisfy repeatability, URDF/affine consistency and the supervised hardware-witness gate.
- EEPROM writes are allowed only after complete measurement PASS, verified Station shutdown and serial release.
- EEPROM provisioning is transactional: backup, unlock, staged writes, Action, readback, relock and rollback on failure.
- LF V25 must not be rerun unless LF mechanics, servo, mounting, URDF or calibration state changes.

## Validated platform

```text
Asus Ubuntu
→ NormaCore Station
→ Waveshare Bus Servo Adapter
→ custom power-distribution board
→ 12 × Feetech ST3215
```

Validated:

- twelve-servo bus and sparse-ID topology;
- custom power distribution and wiring;
- canonical servo mapping and joint directions;
- REV00 CAD/URDF kinematic model;
- mechanical and digital q0 commissioning;
- encoder-to-radian conversion;
- read-only live FK for all four legs;
- offline contact, collision, timing and support-polygon references;
- native Station-mediated mechanical contact calibration;
- complete LF endpoint measurement, affine alignment, EEPROM freeze and persistent profile.

## Canonical robot definition

```text
03_CAD/URDF/matt_robodog_rev00/
```

The REV00 package contains the canonical URDF, visual and collision meshes, workbook, URDF Studio project and integrity manifest.

Coordinate convention:

```text
X = forward
Y = left
Z = up
units = metres and radians
right-handed frame
```

Current geometry:

```text
front-to-rear hip spacing: 225 mm
left-to-right hip spacing: 95 mm
hip-to-knee segment: 90 mm
knee-to-foot mechanical interface: 110 mm
knee-to-contact-frame distance: 118.1 mm
target nominal body height: about 150 mm
```

Canonical leg order and trot diagonals:

```text
[LF, RF, RH, LH]
LF + RH
RF + LH
```

Canonical servo mapping:

```text
LF: M13 hip, M12 upper, M11 lower
RF: M23 hip, M22 upper, M21 lower
RH: M33 hip, M32 upper, M31 lower
LH: M43 hip, M42 upper, M41 lower
```

## Repository roles

```text
MattRobotics/robot-dog
→ MATDOG source of truth: CAD, URDF, calibration evidence, profiles,
  kinematics, gait, electronics, validation and project decisions

MattRobotics/norma-core
→ Station/ST3215 integration fork and native MATDOG calibration runtime

MattRobotics/xgolite-low-level-reconstruction
→ read-only architecture and firmware-research reference; not MATDOG runtime
```

## Roadmap

### Foundation

- [x] Mechanical architecture and REV00 CAD/URDF
- [x] ST3215 bus, mapping, directions and digital-home commissioning
- [x] Encoder/radian conversion and four-leg read-only FK
- [x] Offline stand, contact, collision, timing and stability references
- [x] Native NormaCore MATDOG contact-calibration foundation

### Mechanical calibration

- [x] LF six-contact calibration
- [x] LF affine q0 derivation and URDF gate
- [x] LF transactional EEPROM freeze
- [x] LF persistent calibration profile
- [ ] Generalize the validated V25 architecture to RF
- [ ] Calibrate and freeze RF
- [ ] Calibrate and freeze RH
- [ ] Calibrate and freeze LH
- [ ] Validate the complete twelve-joint persistent profile

### Locomotion

- [ ] Recompute read-only four-leg FK using all frozen profiles
- [ ] Regenerate HOME → LOW_STAND → NOMINAL_STAND
- [ ] Complete collision/contact/support audit with calibrated limits
- [ ] Supervised suspended stand
- [ ] Gradual load transfer and nominal stand
- [ ] Single-foot swing trajectory
- [ ] Trot in place
- [ ] First slow walking tests

### Embedded integration and autonomy

- [ ] Battery and smart BMS integration
- [ ] Jetson integration
- [ ] Low-level motion-controller evaluation
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

Historical experiments and superseded calibration versions remain preserved in closed PRs and `09_Logs/`; they are not current operating instructions.

---

Built and documented by Matt Robotics.
