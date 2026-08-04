# MATDOG Architecture

## Purpose

MATDOG is a custom quadruped robot developed as a dedicated project above the NormaCore ecosystem.

Robot-specific geometry, calibration, kinematics, locomotion and project evidence remain independent from the Station transport implementation.

## Repository responsibilities

| Repository | Responsibility |
|---|---|
| `MattRobotics/robot-dog` | Public MATDOG source of truth: CAD, URDF, electronics, calibration evidence, kinematics, locomotion, tests and engineering documentation |
| `MattRobotics/norma-core` | Station/ST3215 integration fork and native MATDOG calibration runtime |
| `norma-core/norma-core` | Official upstream reference followed through controlled updates |

Unrelated private research repositories are deliberately excluded from the public MATDOG architecture and are not runtime dependencies.

## Runtime layers

```text
MATDOG dashboard / high-level control
        ↓ semantic command
MATDOG control and safety layer
        ↓ calibrated joint targets in radians
MATDOG ST3215 adapter
        ↓ official Station command path
NormaCore ST3215 driver
        ↓
Waveshare Bus Servo Adapter
        ↓
12 × ST3215
```

## Strict ownership rules

1. The normal MATDOG dashboard never sends raw encoder targets.
2. The MATDOG controller owns conversion between joint radians and actuator targets.
3. NormaCore Station remains the sole owner of the ST3215 serial bus during motion.
4. The MATDOG core remains independent from Station queues, protobuf details and serial-device paths.
5. A future embedded controller may replace the actuator backend without replacing the public MATDOG kinematics, gait or action API.
6. Digital-home commissioning and mechanical endpoint calibration remain separate programs.
7. Only hardware-validated calibration results may become persistent operational profiles.

## Current implementation boundary

```text
robot-dog
→ geometry, URDF, calibration evidence, persistent project records,
  kinematics, locomotion and public documentation

norma-core
→ native Station-mediated ST3215 calibration implementation,
  runner/observer contracts and EEPROM provisioner
```

The only mechanically hardware-validated calibration milestone is LF V25. RF, RH and LH must be generalized from the merged V25 architecture and validated individually before any complete all-leg profile is declared operational.

## Development sequence

1. Preserve the REV00 geometry and joint naming.
2. Preserve the validated twelve-servo mapping and digital-home contract.
3. Generalize LF V25 to RF through data-driven leg profiles.
4. Validate and freeze RF, RH and LH individually.
5. Recompute four-leg FK using all frozen profiles.
6. Regenerate and audit stand poses and transitions.
7. Validate supervised stand and gradual load transfer.
8. Add single-foot trajectories, gait and walking.
9. Integrate embedded sensing, watchdog and autonomy.
