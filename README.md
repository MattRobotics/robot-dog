# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy.

The project combines mechanical design, 3D printing, ST3215 serial-bus servos, kinematics, gait generation, embedded control and future perception capabilities.

## Current Engineering Milestone — 2026-07-21

MATDOG REV00, digital zero, encoder-to-radian conversion, four-leg live FK, offline IK/contact closure, collision policy and offline contact-locked rest-to-stand planning are complete.

The mechanical end-stop calibration architecture is now frozen:

```text
MATDOG AutoCalibrate command
→ native NormaCore ST3215 Rust sequence
→ RAM-only verified control
→ position + velocity + PresentCurrent contact detector
→ backoff + repeated contact
→ measured contact and safe-limit record
```

Command-capable Python end-stop prototypes created during the 2026-07-21 experiments are retired and must not be used again. The next hardware test is exclusively the native `LF_UPPER / M12 / MIN` pilot.

No new C5 stand attempt is command-eligible until:

```text
native M12 pilot PASS
→ 24 validated joint contacts
→ conservative safe limits in MATDOG_JOINT_CALIBRATION.yaml
→ read-only four-leg FK closure
→ regenerated HOME q=0 → LOW_STAND → NOMINAL_STAND targets
→ collision, contact, support and timing audit
→ supervised suspended stand
```

NormaCore foundation currently targets upstream `0.1.0-beta.9`, `normfs 0.1.0-beta.1`, PR #86 and MATDOG sparse servo discovery through ID 43.

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
- safe upper- and lower-leg joint limits;
- validated mechanical q=0 capture for all 12 leg joints;
- 12-servo ST3215 digital-zero EEPROM calibration;
- final post-restart EEPROM read-back with lock and torque verification;
- encoder-to-radian URDF conversion contract;
- global static tracking acceptance policy: <=10 ticks (±0.879°);
- C3 live FK read-only validation for LF, RF, RH and LH.

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

## Calibration Status

Mechanical reference calibration was captured for all 12 leg joints on
2026-07-06. On 2026-07-10 the complete 12-servo digital-zero calibration was
validated: each physical mechanical `q = 0` encoder value was mapped to the
canonical displayed value `2048` by writing the signed ST3215 Position Offset.

The final post-restart read-back confirmed the planned offset on every servo,
EEPROM `LOCK = 1`, `TORQUE = 0`, displayed positions from 2048 to 2051 ticks,
and a maximum physical raw-encoder deviation of 3 ticks from the mechanical-q0
capture.

The canonical encoder-to-radian contract remains the software source of truth
for live joint state, FK and IK. The digital recenter changes the servo-side
encoder representation of `q = 0`; it does not change joint directions, URDF
geometry, URDF limits or mechanical/contact limits.

Canonical utility:

    06_Software/Matdog_Core/calibration/matdog_digital_zero_calibration.py

Procedure:

    06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md

Static tracking acceptance is unified for all 12 ST3215 servos:

    tolerance = <=10 encoder ticks
    tolerance = ±0.879°

This applies to static hold, single-servo micro-probe, return validation and
controlled static-pose checks. It does not change the mechanical `q = 0`
definition, calibrated digital zero, encoder direction, URDF limits,
mechanical/contact limits, stand acceptance or dynamic locomotion acceptance.

M13 (LF hip) was diagnosed separately after an initially tighter 8-tick
threshold exposed a repeatable directional residual. Read-only range checks
excluded a local mechanical stop; the diagnostic probe measured a worst-case
static residual of 10 ticks with low current and status 0x00. M13 is accepted
under the same shared policy as every other joint.

LF live FK was validated on 2026-07-07 with Station as the sole serial owner.
On 2026-07-08 the generalized live FK read-only tool was validated for all four
legs: LF, RF, RH and LH. The tool reads Station telemetry only, converts encoder
state to URDF joint angles and computes the selected `*_foot_link` in
`base_link`, without sending torque, target, speed or accel commands.

C3-B live FK read-only validation is archived in:

    09_Logs/Calibration_Sessions/C3_live_fk/

C4-A offline safe stand candidate is archived in:

    06_Software/Matdog_Core/kinematics/matdog_offline_safe_stand_candidate.py
    09_Logs/Validation_Reports/2026-07-08_175245_C4A_offline_safe_stand_candidate.json

C4-B static offline collision/contact policy is archived in:

    06_Software/Matdog_Core/kinematics/matdog_offline_collision_contact_policy.py
    06_Software/Matdog_Core/kinematics/MATDOG_COLLISION_CONTACT_POLICY.md
    09_Logs/Validation_Reports/C4_collision_contact_policy/2026-07-08_183409_C4B_collision_contact_policy.json

C4-C offline rest-to-stand trajectory sampling is archived in:

    06_Software/Matdog_Core/kinematics/matdog_offline_rest_to_stand_trajectory.py
    06_Software/Matdog_Core/kinematics/MATDOG_REST_TO_STAND_TRAJECTORY_POLICY.md
    09_Logs/Validation_Reports/C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json

C4-D offline trajectory timing and servo-envelope validation is archived in:

    06_Software/Matdog_Core/kinematics/matdog_offline_trajectory_timing_envelope.py
    06_Software/Matdog_Core/kinematics/MATDOG_TRAJECTORY_TIMING_ENVELOPE.md
    09_Logs/Validation_Reports/C4_trajectory_timing_envelope/2026-07-08_191003_C4D_trajectory_timing_envelope.json

C4-E offline static stability / support-polygon validation is archived in:

    06_Software/Matdog_Core/kinematics/matdog_offline_static_stability_support_polygon.py
    06_Software/Matdog_Core/kinematics/MATDOG_STATIC_STABILITY_SUPPORT_POLYGON.md
    09_Logs/Validation_Reports/C4_static_stability_support_polygon/2026-07-08_191840_C4E_static_stability_support_polygon.json

C4-F hardware safe-mode preflight is archived in:

    06_Software/Matdog_Core/kinematics/matdog_offline_hardware_safe_mode_preflight.py
    06_Software/Matdog_Core/kinematics/MATDOG_HARDWARE_SAFE_MODE_PREFLIGHT.md
    09_Logs/Validation_Reports/C4_hardware_safe_mode_preflight/2026-07-08_193744_C4F_hardware_safe_mode_source_audit.json

The next phase is a separate MATDOG mechanical end-stop calibration workflow.
Its intended operator experience is a single controlled calibration action, but
the implementation must remain Station-mediated, collision-aware and
one-joint-at-a-time. It must measure physical contact limits, apply conservative
safety margins, update the canonical calibration record, and complete a new
read-only FK/target audit before any supervised rest-to-stand attempt.

Calibration records:

- canonical configuration: `06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`;
- unified 12-servo tool: `06_Software/Matdog_Core/calibration/matdog_digital_zero_calibration.py`;
- digital-zero procedure: `06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md`;
- mechanical end-stop calibration plan: `06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_CALIBRATION_PLAN.md`;
- final digital-zero read-back: `09_Logs/Calibration/C5_R_digital_recenter/2026-07-10_145457Z_final_12_offset_readback.json`;
- tolerance policy: `09_Logs/Calibration_Sessions/2026-07-07_static_tracking_tolerance_10_ticks.policy.yaml`;
- M13 diagnostic: `09_Logs/Calibration_Sessions/2026-07-07_092043_M13_micro_probe_diagnostic.json`.

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

- [x] Mechanical q=0 calibration for all 12 joints
- [x] Encoder-to-radian calibration contract
- [x] 12-servo ST3215 digital-zero EEPROM calibration and final readback
- [x] Global static tracking policy: <=10 ticks (±0.879°)
- [x] Four-leg live FK and offline contact-reference IK closure
- [x] C4-A…C4-F offline stand, collision, timing, stability and preflight references
- [x] Mechanical end-stop prerequisite geometry and cross-leg parking poses
- [x] NormaCore upstream update to 0.1.0-beta.9 / normfs 0.1.0-beta.1 foundation
- [x] Retire command-capable Python end-stop prototypes
- [ ] Commit and publish MATDOG sparse-ID discovery foundation in NormaCore fork
- [ ] Implement native Rust MATDOG AutoCalibrate dispatcher and RAM-only primitives
- [ ] Native LF_UPPER / M12 / MIN pilot
- [ ] Supervised 12-joint / 24-contact acquisition
- [ ] Record measured contacts and conservative safe limits in canonical YAML
- [ ] Post-calibration read-only joint-state and four-leg FK closure
- [ ] Regenerate and audit HOME q=0 → LOW_STAND → NOMINAL_STAND trajectory
- [ ] Supervised suspended stand and gradual load transfer
- [ ] Four-leg body-height control
- [ ] Single-foot swing trajectory
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
- Calibration sessions: 09_Logs/Calibration_Sessions/
- Digital-zero calibration: 09_Logs/Calibration/C5_R_digital_recenter/
- C5-R post-digital-zero handoff: 09_Logs/Development_Log/2026-07-10_C5R_POST_DIGITAL_ZERO_HANDOFF.md
- Native calibrator handoff: 09_Logs/Development_Log/2026-07-21_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md
- LF live FK validation: 09_Logs/Calibration_Sessions/2026-07-07_lf_fk_live_validation.result.yaml
- Canonical world/contact/IK regressions: 06_Software/Matdog_Core/kinematics/tests/test_matdog_quadruped_leg_contact_ik.py
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
