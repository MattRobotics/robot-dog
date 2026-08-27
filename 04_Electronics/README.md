# MATDOG Electronics

This directory contains the electrical architecture and hardware integration records for MATDOG.

## Current Control Chain

    high-level host  (ASUS now, Jetson-class later)
    → dedicated MATDOG ESP32-S3 coprocessor      ← operational owner of the ST3215 bus
    → Waveshare Bus Servo Adapter
    → custom power-distribution board
    → 17 × Feetech ST-3215-C018                  (12 leg + 5 head/jaw)

The servo bus, branch wiring and power-distribution board are validated for the
locomotion-development phase. Direct ESP32-S3 ↔ ST3215 operation is hardware-validated through
bench QC and the 17/17 provisioning campaign.

> The host ↔ ESP32-S3 transport is **TBD — not frozen**. See
> [ARCHITECTURE.md](../01_Docs/02_Architecture/ARCHITECTURE.md).

## Current Contents

- Servo_Mapping/
  Canonical mapping between semantic MATDOG joints and physical ST3215 IDs.

## Current Servo Mapping

Authoritative source:
[`06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml`](../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)

    LF:   hip 13 (M22),  upper 12 (ELR01), lower 11 (M33)
    RF:   hip 23 (NEW01), upper 22 (ELR03), lower 21 (NEW03)
    RH:   hip 33 (NEW06), upper 32 (ELR02), lower 31 (NEW05)
    LH:   hip 43 (M43),  upper 42 (M42),   lower 41 (M41)
    HEAD: neck rotation 51 (M31), neck pitch 52 (M11),
          head rotation 53 (NEW04), head pitch 54 (NEW02), jaw 55 (ELR04)

> **Replaces the historical 12-servo mapping** (`LF: hip M13, upper M12, lower M11`, …).
> 14 of 17 units were recoded during provisioning, so historical records keyed by bus ID will
> associate the **wrong joint**.

## Future Contents

- wiring diagrams
- power-distribution-board documentation
- battery and BMS design
- charging and docking system
- IMU integration
- head sensors
- Jetson and coprocessor integration
- electrical validation measurements

Raw ST3215 bus diagnostics are provided by the ESP32-S3 bench tools
([`05_Firmware/ST3215_Bench_Tools/`](../05_Firmware/ST3215_Bench_Tools/README.md)).
NormaCore Station is optional/legacy tooling and is not required for any MATDOG bus operation.
MATDOG-specific wiring, mappings and electrical design belong here.
