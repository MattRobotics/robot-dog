# MATDOG Tests

This directory contains repeatable tests, test scripts, fixtures and recorded outputs for MATDOG.

## Scope

- URDF structural validation
- mesh and frame validation
- joint-limit validation
- calibration checks
- forward-kinematics tests
- inverse-kinematics tests
- gait trajectory tests
- hardware safety checks
- Station compatibility tests

## Current Status

The first validated engineering baseline is:

    MATDOG URDF REV00

Its validation report is stored in:

    ../09_Logs/Validation_Reports/2026-06-30_URDF_REV00_Kinematic_Validation.md

## Test Development Order

    URDF structure
    → mechanical zero calibration
    → encoder-to-radian conversion
    → single-joint motion
    → single-leg FK
    → single-leg IK
    → stand pose
    → gait trajectory
    → hardware locomotion

Tests must be added before enabling automated multi-servo motion on the physical robot.
