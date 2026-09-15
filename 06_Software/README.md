# MATDOG Software

This area contains MATDOG-specific host-side tools, machine-readable configuration, offline
geometry and kinematics code, validation utilities, and visualization software.

It does not own the current project milestone or the robot's runtime architecture:

| Need | Canonical owner |
|---|---|
| Current project snapshot and immediate milestone | [Root `README.md`](../README.md) |
| Runtime architecture and responsibility boundaries | [`ARCHITECTURE.md`](../01_Docs/02_Architecture/ARCHITECTURE.md) |
| Operational ESP32-S3 firmware | [MATDOG Controller](../05_Firmware/MATDOG_Controller/README.md) |

## Contents

- [`Matdog_Core/config/`](Matdog_Core/config/) — machine-readable allocation, servo-profile, and
  geometry configuration.
- [`Matdog_Core/calibration/`](Matdog_Core/calibration/) — calibration data contracts, offline
  geometry tooling, validators, and preserved calibration-development records.
- [`Matdog_Core/kinematics/`](Matdog_Core/kinematics/) — kinematic models, offline safety policies,
  and associated tests.
- [`Matdog_Core/hardware/`](Matdog_Core/hardware/) — retained hardware-facing development tools;
  they are not current motion authorization.
- [`Matdog_Core/viewer/`](Matdog_Core/viewer/) — host-side visualization tools.

## Current boundary

The permanent operational actuator boundary is the ESP32-S3 MATDOG Controller. New host-side
software may express semantic robot intent, but it must not become an independent owner of the
ST3215 serial bus or redefine Controller responsibilities. The host protocol carried over the
selected transport remains an architecture-level contract.

The calibration directory deliberately contains both reusable offline engineering work and dated
Station-era procedures, handoffs, and evidence. A historical document remains evidence for its
recorded phase; it is not current runtime, current calibration, a current milestone, or permission
to operate hardware. Follow each file's status banner and the
[historical index](../09_Logs/Historical/README.md).
