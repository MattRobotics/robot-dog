# MATDOG Core

MATDOG Core contains robot-specific configuration, offline calibration and geometry tools,
kinematics and safety-policy code, retained hardware-development utilities, tests, and host-side
visualization.

## Contents

- [`config/`](config/) — canonical machine-readable allocation and engineering configuration.
- [`calibration/`](calibration/) — calibration contracts, validators, Geometry Compiler code, and
  preserved historical Station-era procedures and handoffs.
- [`kinematics/`](kinematics/) — FK/IK, contact, stance, trajectory, and offline safety tooling.
- [`hardware/`](hardware/) — retained hardware-facing development utilities; not current motion
  authorization.
- [`viewer/`](viewer/) — the BNO085 full-body viewer and related host tooling.

Current project status lives in the [root README](../../README.md). Runtime ownership and target
direction live in [`ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md).

## Controller boundary

The ESP32-S3 [MATDOG Controller](../../05_Firmware/MATDOG_Controller/README.md) is the permanent
operational actuator boundary and sole ST3215 bus owner. Host-side and offline code in this area
must not create a second bus owner. The eventual host-to-Controller application protocol remains a
separate architecture contract; preserved Station-based code and documents are historical or
transitional references, not the current control flow.

## Full-leg calibration oracle

The Full Leg Calibrator V1 tree is preserved as an engineering oracle and evidence source by the
annotated tag `archive/2026-08-29/full-leg-calibrator-v1-h0` ->
`15f3fb8f378e6cadf6bc479bfcaca2947741c9fd`; the former branch `matdog/full-leg-calibrator-v1` was
archived 2026-09-18 and no longer exists. It is **not** the final runtime architecture and must
not be merged wholesale into the Controller. Its useful calibration engine, safety gates, and evidence logic are candidates for a
later deliberate migration into the unified Controller architecture.
