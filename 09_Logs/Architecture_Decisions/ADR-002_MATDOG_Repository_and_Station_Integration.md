# ADR-002 - MATDOG Repository and Station Integration Boundary

Date: 2026-06-25

## Status

Accepted

> *Annotation 2026-10-08:* partially superseded; see [Status update](#status-update-2026-10-08)
> at the end of this record. The original text below is unchanged.

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

## Status update (2026-10-08)

This section is an annotation. The decision above is kept as it was written on 2026-06-25.

**Still valid:** `MattRobotics/robot-dog` is the MATDOG source of truth for geometry, URDF, servo
mapping, calibration, kinematics, gait, validation tests and engineering documentation.

**SUPERSEDED:** the responsibilities that assigned direct control of the servo bus to NormaCore
Station: that "the official NormaCore ST3215 driver remains the serial-bus owner" and that the
Station adapter is the actuator path. Since the rebuild around the ESP32-S3, the MATDOG Controller
is the sole operational owner of the ST3215 bus; a host sends semantic intent and never raw bus
traffic in normal operation. See [`ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md) and the
root [`README.md`](../../README.md) (status vocabulary: "mandatory NormaCore Station bus
ownership" is SUPERSEDED). The consequence that a future ESP32 migration may replace the actuator
backend is what happened.

This annotation does not decide the fate of the `norma-core` fork or of Station-mediated evidence,
which remains HISTORICAL (see the [historical index](../Historical/README.md)).
