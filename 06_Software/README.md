# MATDOG Software

This directory contains MATDOG-specific host-side software.

## Scope

- joint calibration tools
- encoder-to-radian conversion
- forward kinematics
- inverse kinematics
- stand-pose logic
- gait generation
- safety and motion limits
- future Station integration adapters
- simulation and validation utilities

## Current Structure

- Matdog_Core/
  Robot-specific software configuration and future reusable control modules.

## Current Development Rule

MATDOG software must remain independent from:

- direct serial-port access
- raw ST3215 protocol details
- NormaCore internal queues and protobuf definitions

The MATDOG core will produce semantic joint targets in radians.

The Station integration layer will later convert those targets into official ST3215 commands.

## Current Priority

The immediate software sequence is:

    mechanical zero calibration
    → encoder-to-radian mapping
    → single-leg FK
    → single-leg IK
    → stand pose
    → gait generation

Do not add gait or IK code until the mechanical zero, joint direction and safe limits of all 12 joints are measured and recorded.
