# ADR-002 - MATDOG Repository and Station Integration Boundary

Date: 2026-06-25

## Status

Accepted

## Context

MATDOG requires dedicated software for URDF, calibration, kinematics, stand pose, gait and a future control dashboard. NormaCore Station already provides the ST3215 serial driver, telemetry, command queue and viewer framework.

Putting all MATDOG-specific logic directly into a Station fork would couple robot geometry and gait to generic infrastructure, making upstream updates harder to manage.

## Decision

`MattRobotics/robot-dog` is the MATDOG source of truth.

It owns:

- geometry;
- URDF;
- servo mapping;
- calibration;
- kinematics;
- gait;
- validation tests;
- engineering documentation.

`MattRobotics/norma-core` remains a thin integration fork.

It owns only:

- MATDOG command registration;
- Station driver integration;
- Station dashboard mounting;
- adapter to the official ST3215 command path;
- compatibility changes required by upstream updates.

## Consequences

- MATDOG core logic must not open serial ports directly.
- The official NormaCore ST3215 driver remains the serial-bus owner.
- Dashboard commands remain semantic and must not expose raw encoder targets during normal operation.
- Changes in upstream ST3215 code should be isolated to the MATDOG Station adapter.
- A future ESP32 migration may replace the actuator backend without rewriting gait, IK, calibration format or semantic dashboard commands.
