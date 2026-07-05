# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy.

The project combines mechanical design, 3D printing, ST3215 serial-bus servos, kinematics, gait generation, embedded control and future perception capabilities.

## Current Engineering Milestone

**MATDOG REV00 kinematic baseline is archived and validated.**

The current priority is locomotion development:

    Servo calibration
    → stand pose
    → single-leg FK and IK
    → four-leg coordination
    → foot trajectories
    → trot in place
    → first slow walking tests

## Validated Platform

    Asus Ubuntu
    → NormaCore Station
    → Waveshare Bus Servo Adapter
    → custom power-distribution board
    → 12 × Feetech ST3215

Validated so far:

- 12-servo bus communication;
- custom power-distribution board;
- canonical servo mapping;
- CAD-derived URDF structure;
- 17-link / 16-joint REV00 kinematic model;
- visual and collision mesh placement;
- safe upper- and lower-leg joint limits.

## Canonical REV00 Robot Description

The official REV00 package is stored in:

03_CAD/URDF/matt_robodog_rev00/

It contains:

- canonical URDF model;
- baked STL meshes;
- collision configuration;
- URDF Studio project archive;
- kinematic, mass and material workbook;
- SHA-256 integrity manifest.

## Current Robot Description

    4 legs
    17 links
    16 joints
    12 revolute leg joints
    4 fixed foot joints

Canonical leg order:

    [LF, RF, RH, LH]

Trot diagonal pairs:

    LF + RH
    RF + LH

## Current Geometry

    Front-to-rear hip spacing: 225 mm
    Left-to-right hip spacing: 95 mm
    Hip-to-knee segment: 90 mm
    Knee-to-foot mechanical interface center: 110 mm
    Knee-to-foot contact-frame distance: 118.1 mm
    Target stand body height: approximately 150 mm

Coordinate convention:

    X = forward
    Y = left
    Z = up
    Units = metres and radians
    Right-handed coordinate system

## Canonical Servo Mapping

    LF: hip M13, upper M12, lower M11
    RF: hip M23, upper M22, lower M21
    RH: hip M33, upper M32, lower M31
    LH: hip M43, upper M42, lower M41

## Repository Structure

    01_Docs/        Architecture, technical references and project documentation
    02_BOM/         Bills of materials and component records
    03_CAD/         CAD, URDF, meshes and mechanical exports
    04_Electronics/ Wiring, power, servo mapping and electronics design
    05_Firmware/    Future low-level and embedded firmware
    06_Software/    Kinematics, gait, control and software tools
    07_Media/       Images, videos and public project media
    08_Tests/       Validation procedures, scripts and test outputs
    09_Logs/        Decisions, validation reports and development history

## Roadmap

### Phase 1 — Foundation

- [x] Mechanical architecture
- [x] ST3215 bus and power-distribution validation
- [x] Canonical servo mapping
- [x] MATDOG REV00 URDF kinematic baseline

### Phase 2 — Locomotion

- [ ] Mechanical zero calibration for all 12 joints
- [x] Encoder-to-radian calibration contract (software)
- [ ] Single-leg forward kinematics
- [ ] Single-leg inverse kinematics
- [ ] Safe stand pose
- [ ] Four-leg body-height control
- [ ] Trot in place
- [ ] First slow walking tests

### Phase 3 — Embedded Integration

- [ ] Battery and BMS integration
- [ ] Jetson integration
- [ ] Low-level motion-controller evaluation
- [ ] IMU and safety/watchdog integration

### Phase 4 — Perception and Autonomy

- [ ] Depth vision
- [ ] Object detection
- [ ] Voice interaction
- [ ] Autonomous behaviour

## Project Records

- Architecture decisions: 09_Logs/Architecture_Decisions/
- Validation reports: 09_Logs/Validation_Reports/
- Development log: 09_Logs/Development_Log/
- URDF REV00 package: 03_CAD/URDF/matt_robodog_rev00/

---

Built and documented by Matt Robotics.

## Mechanical Segment and Contact-Frame Terminology

- `Hip-to-knee mechanical segment`: `90 mm`.
- `Knee-to-foot mechanical interface center`: `110 mm`. This is the
  nominal distance from the knee axis to the center of the lower-leg end
  where the eccentric rubber foot attaches.
- `Knee-to-foot contact-frame distance`: `118.1 mm`. This is the true
  distance from the knee axis to the URDF `foot_joint`, which represents
  the nominal ground-contact point of the eccentric rubber foot.

The `110 mm` mechanical segment and the `118.1 mm` contact-frame distance
are intentionally different. The contact-frame distance is the value to use
for nominal ground contact, stand-pose development, IK and gait planning.

## ST3215 URDF Effort and Velocity Semantics

The REV00 URDF uses:

    effort   = 0.902244 N*m
    velocity = 3.03687289847 rad/s = 29 rpm

These are conservative MATDOG nominal-operation limits for the intended 3S
operating point. They are not Feetech-published performance values at exactly
11.1 V and must not be interpreted as stall specifications.
