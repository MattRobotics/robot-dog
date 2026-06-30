# MATDOG Core

MATDOG Core will contain robot-specific control logic that is independent from NormaCore Station transport and ST3215 serial-bus details.

## Current Contents

- config/
  Canonical software configuration derived from the approved CAD and URDF baseline.

## Planned Modules

- calibration/
- kinematics/
- gait/
- safety/
- simulation/
- tests/

## Design Rule

The control flow must remain modular:

    gait generator
    → leg inverse kinematics
    → joint targets in radians
    → actuator adapter

The actuator adapter may initially use NormaCore Station and the Waveshare Bus Servo Adapter.

A future ESP32 motion controller may replace only the actuator layer, without rewriting gait, IK or calibration logic.
