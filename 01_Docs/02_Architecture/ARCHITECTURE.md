# MATDOG Architecture

## Purpose

MATDOG is a custom quadruped robot developed as a dedicated project above the NormaCore ecosystem.

The project must keep robot-specific geometry, calibration, kinematics and gait independent from the Station transport implementation.

## Repository Responsibilities

| Repository | Responsibility |
|---|---|
| `MattRobotics/robot-dog` | MATDOG source of truth: geometry, URDF, calibration, kinematics, gait, tests and engineering documentation |
| `MattRobotics/norma-core` | Thin integration layer with NormaCore Station |
| `norma-core/norma-core` | Official upstream reference tracked through controlled updates |
| `MattRobotics/xgolite-low-level-reconstruction` | Separate evidence repository for XGO Lite reverse engineering |
| `MattRobotics/DOGZILLA*` | Reference material only; not MATDOG production code |

## Runtime Layers

```text
MATDOG Dashboard
        ↓ semantic command
MatdogControlDriver
        ↓ calibrated joint targets in radians
MatdogSt3215Adapter
        ↓ official ST3215 command path
NormaCore ST3215 Driver
        ↓
Waveshare Bus Servo Adapter
        ↓
12 × ST3215
```

## Strict Ownership Rules

1. The normal MATDOG dashboard never sends raw encoder targets.
2. The MATDOG controller owns conversion between joint radians and actuator targets.
3. The NormaCore ST3215 driver remains the only owner of the serial bus.
4. The MATDOG core must remain independent from Station queues, protobufs and serial-device paths.
5. A future ESP32 migration may replace the actuator implementation, not the dashboard, gait or IK API.

## Development Sequence

1. Freeze geometry and joint naming.
2. Complete the CAD-derived URDF.
3. Perform kinematic calibration.
4. Validate single-joint visualization.
5. Validate single-leg forward and inverse kinematics.
6. Validate stand pose.
7. Add gait trajectories.
8. Integrate the MATDOG dashboard with Station.
