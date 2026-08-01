# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The project combines mechanical design, 3D printing, ST3215 serial-bus servos, kinematics, native calibration, gait generation, embedded control and future perception capabilities.

## Current Engineering Milestone — 2026-08-01

The REV00 mechanical and kinematic foundation is complete:

- 17 links, 16 joints and 12 revolute leg joints;
- canonical CAD-derived URDF and collision meshes;
- exact 12-servo mapping;
- four-leg live forward kinematics from Station telemetry;
- offline IK/contact closure, collision policy and stand-planning references;
- historical 12-servo EEPROM digital-zero calibration and read-back;
- native NormaCore RAM-only mechanical-contact calibrator.

All six physical contacts of the LF leg have now completed supervised hardware acquisition:

| Joint | Servo | MIN | MAX | Spread |
|---|---:|---:|---:|---:|
| LF upper | M12 | 1443 / 1443 | 3443 / 3442 | 0 / 1 ticks |
| LF lower | M11 | 3094 / 3092 | 1664 / 1666 | 2 / 2 ticks |
| LF hip | M13 | 2530 / 2530 | 1595 / 1595 | 0 / 0 ticks |

The combined HIP program completed MIN → HOME → MAX → HOME, `Done 20/20`, verified global torque OFF and released the serial port.

The next development step is no longer another isolated profile. It is the complete LF sequence:

```text
LF UPPER MIN/MAX
→ LF LOWER MIN/MAX
→ LF HIP MIN/MAX
→ compare both endpoints with the REV00 URDF
→ derive calibrated software q=0
→ place LF at accepted HOME
→ verified global torque OFF
```

## Why HOME is being recalculated

The displayed encoder value `2048` currently corresponds to the visual/mechanical pose captured before full contact acquisition. That pose was useful and repeatable, but it was initially established by physical alignment rather than by solving against both measured mechanical endpoints.

After MIN and MAX are known, each joint produces two independent q=0 candidates:

```text
zero_from_MIN = measured_MIN - direction × URDF_MIN_delta
zero_from_MAX = measured_MAX - direction × URDF_MAX_delta
```

A calibrated HOME is accepted only when both candidates agree within 24 ticks and the resulting target stays within 96 ticks of the existing digital home. Encoder scale and direction remain fixed.

Current LF evidence shows:

| Joint | zero from MIN | zero from MAX | Disagreement | Status |
|---|---:|---:|---:|---|
| M12 upper | 2040 | 2048 | 8 | consistent |
| M11 lower | 2046 | 2092 | 46 | re-acquire |
| M13 hip | 2018 | 2107 | 89 | re-acquire |

The small repeatability spread therefore proves repeatability, but not yet full mechanical seating for M11 and M13. This matches the operator observation that HIP MAX approached too slowly and appeared not to load the stop fully.

## V38 full-LF program

NormaCore branch:

```text
matdog/full-calibration-v38
```

MATDOG record branch:

```text
matdog/full-calibration-program-v38
```

Explicit Station arm token:

```text
LF_LEG_FULL_V38
```

V38 uses a faster, firmer but still bounded envelope:

```text
TorqueLimit = 500
GoalSpeed   = 160
Acc         = 8
coarse step = 64 ticks
fine step   = 8 ticks
settle      = 900 ms
```

Unchanged protections:

- Station is the only serial owner;
- standard unsigned `GoalPosition`;
- RAM writes only to TorqueEnable, Acc, GoalPosition, GoalSpeed and TorqueLimit;
- model-derived contact corridors and 64-tick guards;
- hard-current abort at raw 200;
- status and driver-error aborts;
- repeated-contact verification;
- verified global torque OFF on success or failure.

The new HOME remains software-level. V38 performs no Position Offset, EEPROM LOCK, reset, ResetCalibration, RegWrite, Action, Save or Freeze operation.

When endpoint consistency fails, the program logs the new contacts, reports `MODEL_ZERO_INCONSISTENT`, does not replace HOME and turns torque off globally.

## Rollout to all four legs

Canonical leg order:

```text
LF → RF → RH → LH
```

Canonical servo mapping:

```text
LF: hip M13, upper M12, lower M11
RF: hip M23, upper M22, lower M21
RH: hip M33, upper M32, lower M31
LH: hip M43, upper M42, lower M41
```

RF is the mirrored front-leg application of the validated LF logic.

LH is the next requested hardware development. Its UPPER/LOWER horizontal geometry is expected to match the established HIP strategy, but the exact LF-front parking pose must first be selected by an offline collision sweep. The target will not be guessed from visual inspection.

The final one-button 12-joint program becomes hardware-eligible after:

```text
LF V38 model-zero PASS
→ LH rear-clearance sequence PASS
→ mirrored RF and RH offline tests PASS
→ final four-leg HOME collision and FK audit PASS
→ full LF → RF → RH → LH supervised execution
```

## Validated Platform

```text
Asus Ubuntu
→ NormaCore Station
→ Waveshare Bus Servo Adapter
→ custom power-distribution board
→ 12 × Feetech ST3215
```

## Canonical REV00 Robot Description

```text
4 legs
17 links
16 joints
12 revolute leg joints
4 fixed foot joints
```

Geometry:

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

The canonical URDF package is stored in:

```text
03_CAD/URDF/matt_robodog_rev00/
```

It contains the URDF, baked STL meshes, collision configuration, URDF Studio archive, physical-property records and integrity manifest.

## Calibration records

Canonical configuration:

```text
06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml
```

Current V38 records:

```text
06_Software/Matdog_Core/calibration/MATDOG_FULL_CALIBRATION_PROGRAM_V38.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CONTACT_EVIDENCE_2026-08-01.yaml
06_Software/Matdog_Core/calibration/matdog_model_zero_solver.py
06_Software/Matdog_Core/calibration/tests/test_matdog_model_zero_solver.py
```

Historical digital-zero records:

```text
06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md
09_Logs/Calibration/C5_R_digital_recenter/
```

The EEPROM digital-zero state remains preserved while the new software model-zero is evaluated across all 12 joints.

## Repository Structure

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

### Foundation

- [x] Mechanical architecture
- [x] ST3215 bus and custom power validation
- [x] Canonical 12-servo mapping
- [x] REV00 URDF and collision baseline
- [x] Four-leg live FK and offline IK/contact closure
- [x] Historical digital-zero EEPROM calibration
- [x] Native RAM-only 24-contact calibrator foundation

### Mechanical calibration

- [x] LF upper MIN and MAX
- [x] LF lower MIN and MAX
- [x] LF hip combined MIN and MAX
- [x] Independent offline model-zero solver
- [ ] LF full one-click V38 hardware validation
- [ ] LH rear-leg clearance and full sequence
- [ ] RF mirrored front-leg sequence
- [ ] RH mirrored rear-leg sequence
- [ ] Complete 12-joint Auto Calibrate sequence
- [ ] Canonical YAML update with accepted contacts and software HOME targets
- [ ] Four-leg post-calibration FK and collision closure

### Locomotion

- [ ] Regenerate HOME → LOW_STAND → NOMINAL_STAND from calibrated model zero
- [ ] Supervised suspended stand
- [ ] Four-leg body-height control
- [ ] Single-foot swing trajectory
- [ ] Trot in place
- [ ] First slow walking tests

### Embedded integration and autonomy

- [ ] Battery and BMS integration
- [ ] Jetson integration
- [ ] Low-level motion-controller evaluation
- [ ] IMU and watchdog integration
- [ ] Depth vision, perception and autonomous behaviour

---

Built and documented by Matt Robotics.
